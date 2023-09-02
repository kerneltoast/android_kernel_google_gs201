// SPDX-License-Identifier: GPL-2.0
/*
 * GXP debugfs support.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/acpm_dvfs.h>

#include "gxp-client.h"
#include "gxp-debug-dump.h"
#include "gxp-debugfs.h"
#include "gxp-firmware-data.h"
#include "gxp-firmware.h"
#include "gxp-internal.h"
#include "gxp-notification.h"
#include "gxp-lpm.h"
#include "gxp-mailbox.h"
#include "gxp-pm.h"
#include "gxp-telemetry.h"
#include "gxp-vd.h"
#include "gxp-wakelock.h"
#include "gxp.h"

static int gxp_debugfs_lpm_test(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *) data;

	dev_info(gxp->dev, "%llu\n", val);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(gxp_lpm_test_fops, NULL, gxp_debugfs_lpm_test,
			 "%llu\n");

static int gxp_debugfs_mailbox(void *data, u64 val)
{
	int core;
	struct gxp_command cmd;
	struct gxp_response resp;
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	core = val / 1000;
	if (core >= GXP_NUM_CORES) {
		dev_notice(gxp->dev,
			   "Mailbox for core %d doesn't exist.\n", core);
		return -EINVAL;
	}

	if (gxp->mailbox_mgr == NULL ||
	    gxp->mailbox_mgr->mailboxes[core] == NULL) {
		dev_notice(gxp->dev,
			   "Unable to send mailbox command -- mailbox %d not ready\n",
			   core);
		return -EINVAL;
	}

	cmd.code = (u16) val;
	cmd.priority = 0;
	cmd.buffer_descriptor.address = 0;
	cmd.buffer_descriptor.size = 0;
	cmd.buffer_descriptor.flags = 0;

	down_read(&gxp->vd_semaphore);
	gxp_mailbox_execute_cmd(gxp->mailbox_mgr->mailboxes[core], &cmd, &resp);
	up_read(&gxp->vd_semaphore);

	dev_info(gxp->dev,
		"Mailbox Command Sent: cmd.code=%d, resp.status=%d, resp.retval=%d\n",
		cmd.code, resp.status, resp.retval);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(gxp_mailbox_fops, NULL, gxp_debugfs_mailbox, "%llu\n");

static int gxp_firmware_run_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *) data;
	struct gxp_client *client;
	int ret = 0;
	uint core;

	ret = gxp_firmware_request_if_needed(gxp);
	if (ret) {
		dev_err(gxp->dev, "Unable to request dsp firmware files\n");
		return ret;
	}

	mutex_lock(&gxp->debugfs_client_lock);

	if (val) {
		if (gxp->debugfs_client) {
			dev_err(gxp->dev, "Firmware already running!\n");
			ret = -EIO;
			goto out;
		}

		/*
		 * Since this debugfs node destroys, then creates new fw_data,
		 * and runs firmware on every DSP core, it cannot be run if
		 * any of the cores already has a VD running on it.
		 */
		down_write(&gxp->vd_semaphore);
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (gxp->core_to_vd[core]) {
				dev_err(gxp->dev,
					"Unable to run firmware with debugfs while other clients are running\n");
				ret = -EBUSY;
				up_write(&gxp->vd_semaphore);
				goto out;
			}
		}

		/*
		 * Cleanup any bad state or corruption the device might've
		 * caused
		 */
		gxp_fw_data_destroy(gxp);
		gxp_fw_data_init(gxp);

		client = gxp_client_create(gxp);
		if (IS_ERR(client)) {
			dev_err(gxp->dev, "Failed to create client\n");
			goto out;
		}
		gxp->debugfs_client = client;

		gxp->debugfs_client->vd = gxp_vd_allocate(gxp, GXP_NUM_CORES);
		if (IS_ERR(gxp->debugfs_client->vd)) {
			dev_err(gxp->dev, "Failed to allocate VD\n");
			ret = PTR_ERR(gxp->debugfs_client->vd);
			goto err_wakelock;
		}

		ret = gxp_wakelock_acquire(gxp);
		if (ret) {
			dev_err(gxp->dev, "Failed to acquire BLOCK wakelock\n");
			goto err_wakelock;
		}
		gxp->debugfs_client->has_block_wakelock = true;
		gxp_pm_update_requested_power_states(gxp, AUR_OFF, true,
						     AUR_UUD, true,
						     AUR_MEM_UNDEFINED,
						     AUR_MEM_UNDEFINED);

		ret = gxp_vd_start(gxp->debugfs_client->vd);
		up_write(&gxp->vd_semaphore);
		if (ret) {
			dev_err(gxp->dev, "Failed to start VD\n");
			goto err_start;
		}
		gxp->debugfs_client->has_vd_wakelock = true;
	} else {
		if (!gxp->debugfs_client) {
			dev_err(gxp->dev, "Firmware not running!\n");
			ret = -EIO;
			goto out;
		}

		/*
		 * Cleaning up the client will stop the VD it owns and release
		 * the BLOCK wakelock it is holding.
		 */
		gxp_client_destroy(gxp->debugfs_client);
		gxp->debugfs_client = NULL;
		gxp_pm_update_requested_power_states(gxp, AUR_UUD, true,
						     AUR_OFF, true,
						     AUR_MEM_UNDEFINED,
						     AUR_MEM_UNDEFINED);
	}

out:
	mutex_unlock(&gxp->debugfs_client_lock);

	return ret;

err_start:
	gxp_wakelock_release(gxp);
	gxp_pm_update_requested_power_states(gxp, AUR_UUD, true, AUR_OFF, true,
					     AUR_MEM_UNDEFINED,
					     AUR_MEM_UNDEFINED);
err_wakelock:
	/* Destroying a client cleans up any VDss or wakelocks it held. */
	gxp_client_destroy(gxp->debugfs_client);
	gxp->debugfs_client = NULL;
	mutex_unlock(&gxp->debugfs_client_lock);
	return ret;
}

static int gxp_firmware_run_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *) data;

	down_read(&gxp->vd_semaphore);
	*val = gxp->firmware_running;
	up_read(&gxp->vd_semaphore);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_firmware_run_fops, gxp_firmware_run_get,
			 gxp_firmware_run_set, "%llx\n");

static int gxp_wakelock_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int ret = 0;

	mutex_lock(&gxp->debugfs_client_lock);

	if (val > 0) {
		/* Wakelock Acquire */
		if (gxp->debugfs_wakelock_held) {
			dev_warn(gxp->dev,
				 "Debugfs wakelock is already held.\n");
			ret = -EBUSY;
			goto out;
		}

		ret = gxp_wakelock_acquire(gxp);
		if (ret) {
			dev_err(gxp->dev,
				"Failed to acquire debugfs wakelock ret=%d\n",
				ret);
			goto out;
		}
		gxp->debugfs_wakelock_held = true;
		gxp_pm_update_requested_power_states(gxp, AUR_OFF, true,
						     AUR_UUD, true,
						     AUR_MEM_UNDEFINED,
						     AUR_MEM_UNDEFINED);
	} else {
		/* Wakelock Release */
		if (!gxp->debugfs_wakelock_held) {
			dev_warn(gxp->dev, "Debugfs wakelock not held.\n");
			ret = -EIO;
			goto out;
		}

		gxp_wakelock_release(gxp);
		gxp->debugfs_wakelock_held = false;
		gxp_pm_update_requested_power_states(gxp, AUR_UUD, true,
						     AUR_OFF, true,
						     AUR_MEM_UNDEFINED,
						     AUR_MEM_UNDEFINED);
	}

out:
	mutex_unlock(&gxp->debugfs_client_lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_wakelock_fops, NULL, gxp_wakelock_set, "%llx\n");

static int gxp_blk_powerstate_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int ret = 0;

	if (gxp_pm_get_blk_state(gxp) == AUR_OFF) {
		dev_warn(
			gxp->dev,
			"Cannot set block power state when the block is off. Obtain a wakelock to power it on.\n");
		return -ENODEV;
	}

	if (val >= AUR_DVFS_MIN_RATE) {
		ret = gxp_pm_blk_set_rate_acpm(gxp, val);
	} else {
		ret = -EINVAL;
		dev_err(gxp->dev, "Incorrect state %llu\n", val);
	}
	return ret;
}

static int gxp_blk_powerstate_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	if (gxp_pm_get_blk_state(gxp) == AUR_OFF) {
		dev_warn(
			gxp->dev,
			"Cannot get block power state when the block is off.\n");
		return -ENODEV;
	}

	*val = gxp_pm_blk_get_state_acpm(gxp);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_blk_powerstate_fops, gxp_blk_powerstate_get,
			 gxp_blk_powerstate_set, "%llx\n");

static int gxp_debugfs_coredump(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int core;

	if (!gxp_debug_dump_is_enabled()) {
		dev_err(gxp->dev, "Debug dump functionality is disabled\n");
		return -EINVAL;
	}

	down_read(&gxp->vd_semaphore);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp_is_fw_running(gxp, core))
			gxp_notification_send(gxp, core,
					      CORE_NOTIF_GENERATE_DEBUG_DUMP);
	}

	up_read(&gxp->vd_semaphore);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_coredump_fops, NULL, gxp_debugfs_coredump,
			 "%llu\n");

static int gxp_log_buff_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int i;
	u64 **buffers;
	u64 *ptr;

	mutex_lock(&gxp->telemetry_mgr->lock);

	if (!gxp->telemetry_mgr->logging_buff_data) {
		dev_err(gxp->dev, "%s: Logging buffer has not been created\n",
			__func__);
		mutex_unlock(&gxp->telemetry_mgr->lock);
		return -ENODEV;
	}

	buffers = (u64 **)gxp->telemetry_mgr->logging_buff_data->buffers;
	for (i = 0; i < GXP_NUM_CORES; i++) {
		ptr = buffers[i];
		*ptr = val;
	}
	dev_dbg(gxp->dev,
		"%s: log buff first bytes: [0] = %llu, [1] = %llu, [2] = %llu, [3] = %llu (val=%llu)\n",
		__func__, *buffers[0], *buffers[1], *buffers[2], *buffers[3],
		val);

	mutex_unlock(&gxp->telemetry_mgr->lock);

	return 0;
}

static int gxp_log_buff_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	u64 **buffers;

	mutex_lock(&gxp->telemetry_mgr->lock);

	if (!gxp->telemetry_mgr->logging_buff_data) {
		dev_err(gxp->dev, "%s: Logging buffer has not been created\n",
			__func__);
		mutex_unlock(&gxp->telemetry_mgr->lock);
		return -ENODEV;
	}

	buffers = (u64 **)gxp->telemetry_mgr->logging_buff_data->buffers;
	dev_dbg(gxp->dev,
		"%s: log buff first bytes: [0] = %llu, [1] = %llu, [2] = %llu, [3] = %llu\n",
		__func__, *buffers[0], *buffers[1], *buffers[2], *buffers[3]);

	*val = *buffers[0];

	mutex_unlock(&gxp->telemetry_mgr->lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_log_buff_fops, gxp_log_buff_get, gxp_log_buff_set,
			 "%llu\n");

static int gxp_log_eventfd_signal_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int ret = 0;

	mutex_lock(&gxp->telemetry_mgr->lock);

	if (!gxp->telemetry_mgr->logging_efd) {
		ret = -ENODEV;
		goto out;
	}

	ret = eventfd_signal(gxp->telemetry_mgr->logging_efd, 1);

out:
	mutex_unlock(&gxp->telemetry_mgr->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_log_eventfd_signal_fops, NULL,
			 gxp_log_eventfd_signal_set, "%llu\n");

/* TODO: Remove these mux entry once experiment is done */
static int gxp_cmu_mux1_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	void *addr;

	if (val > 1) {
		dev_err(gxp->dev, "Incorrect val for cmu_mux1, only 0 and 1 allowed\n");
		return -EINVAL;
	}

	addr = ioremap(gxp->regs.paddr - GXP_CMU_OFFSET, 0x1000);

	if (!addr) {
		dev_err(gxp->dev, "Cannot map CMU1 address\n");
		return -EIO;
	}

	writel(val << 4, addr + PLL_CON0_PLL_AUR);
	iounmap(addr);
	return 0;
}

static int gxp_cmu_mux1_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	void *addr;

	addr = ioremap(gxp->regs.paddr - GXP_CMU_OFFSET, 0x1000);
	*val = readl(addr + PLL_CON0_PLL_AUR);
	iounmap(addr);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_cmu_mux1_fops, gxp_cmu_mux1_get, gxp_cmu_mux1_set,
			 "%llu\n");

static int gxp_cmu_mux2_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	void *addr;

	if (val > 1) {
		dev_err(gxp->dev, "Incorrect val for cmu_mux2, only 0 and 1 allowed\n");
		return -EINVAL;
	}

	addr = ioremap(gxp->regs.paddr - GXP_CMU_OFFSET, 0x1000);

	if (!addr) {
		dev_err(gxp->dev, "Cannot map CMU2 address\n");
		return -EIO;
	}

	writel(val << 4, addr + PLL_CON0_NOC_USER);
	iounmap(addr);
	return 0;
}

static int gxp_cmu_mux2_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	void *addr;

	addr = ioremap(gxp->regs.paddr - GXP_CMU_OFFSET, 0x1000);
	*val = readl(addr + 0x610);
	iounmap(addr);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_cmu_mux2_fops, gxp_cmu_mux2_get, gxp_cmu_mux2_set,
			 "%llu\n");

void gxp_create_debugfs(struct gxp_dev *gxp)
{
	gxp->d_entry = debugfs_create_dir("gxp", NULL);
	if (IS_ERR_OR_NULL(gxp->d_entry))
		return;

	mutex_init(&gxp->debugfs_client_lock);
	gxp->debugfs_wakelock_held = false;

	debugfs_create_file("lpm_test", 0200, gxp->d_entry, gxp,
			    &gxp_lpm_test_fops);
	debugfs_create_file("mailbox", 0200, gxp->d_entry, gxp,
			    &gxp_mailbox_fops);
	debugfs_create_file("firmware_run", 0600, gxp->d_entry, gxp,
			    &gxp_firmware_run_fops);
	debugfs_create_file("wakelock", 0200, gxp->d_entry, gxp,
			    &gxp_wakelock_fops);
	debugfs_create_file("blk_powerstate", 0600, gxp->d_entry, gxp,
			    &gxp_blk_powerstate_fops);
	debugfs_create_file("coredump", 0200, gxp->d_entry, gxp,
			    &gxp_coredump_fops);
	debugfs_create_file("log", 0600, gxp->d_entry, gxp, &gxp_log_buff_fops);
	debugfs_create_file("log_eventfd", 0200, gxp->d_entry, gxp,
			    &gxp_log_eventfd_signal_fops);
	debugfs_create_file("cmumux1", 0600, gxp->d_entry, gxp,
			    &gxp_cmu_mux1_fops);
	debugfs_create_file("cmumux2", 0600, gxp->d_entry, gxp,
			    &gxp_cmu_mux2_fops);
}

void gxp_remove_debugfs(struct gxp_dev *gxp)
{
	debugfs_remove_recursive(gxp->d_entry);

	/*
	 * Now that debugfs is torn down, and no other calls to
	 * `gxp_firmware_run_set()` can occur, destroy any client that may have
	 * been left running.
	 */
	if (gxp->debugfs_client)
		gxp_client_destroy(gxp->debugfs_client);
}
