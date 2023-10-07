// SPDX-License-Identifier: GPL-2.0
/*
 * GXP debugfs support.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/acpm_dvfs.h>

#include <gcip/gcip-pm.h>

#include "gxp-client.h"
#include "gxp-core-telemetry.h"
#include "gxp-debug-dump.h"
#include "gxp-debugfs.h"
#include "gxp-dma.h"
#include "gxp-firmware-data.h"
#include "gxp-firmware-loader.h"
#include "gxp-firmware.h"
#include "gxp-internal.h"
#include "gxp-notification.h"
#include "gxp-lpm.h"
#include "gxp-mailbox.h"
#include "gxp-pm.h"
#include "gxp-vd.h"
#include "gxp.h"

#if GXP_HAS_MCU
#include "gxp-mcu-platform.h"
#endif

static int gxp_debugfs_lpm_test(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	dev_info(gxp->dev, "%llu\n", val);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(gxp_lpm_test_fops, NULL, gxp_debugfs_lpm_test,
			 "%llu\n");

static int gxp_debugfs_mailbox(void *data, u64 val)
{
	int core = 0, retval;
	u16 status;
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	struct gxp_mailbox *mbx;
	struct gxp_client *client;
	struct gxp_power_states power_states = {
		.power = GXP_POWER_STATE_NOM,
		.memory = MEMORY_POWER_STATE_UNDEFINED,
	};
	u16 cmd_code;
	int ret;

	mutex_lock(&gxp->debugfs_client_lock);
	client = gxp->debugfs_client;

#if GXP_HAS_MCU
	if (gxp_is_direct_mode(gxp)) {
#endif
		core = val / 1000;
		if (core >= GXP_NUM_CORES) {
			dev_notice(gxp->dev,
				   "Mailbox for core %d doesn't exist.\n",
				   core);
			ret = -EINVAL;
			goto out;
		}

		if (gxp->mailbox_mgr->mailboxes[core] == NULL) {
			dev_notice(
				gxp->dev,
				"Unable to send mailbox command -- mailbox %d not ready\n",
				core);
			ret = -EINVAL;
			goto out;
		}

		/* Create a dummy client to access @client->gxp from the `execute_cmd` callback. */
		if (!client)
			client = gxp_client_create(gxp);
		mbx = gxp->mailbox_mgr->mailboxes[core];
		cmd_code = GXP_MBOX_CODE_DISPATCH;
#if GXP_HAS_MCU
	} else {
		if (!client) {
			dev_err(gxp->dev,
				"You should load firmwares via gxp/firmware_run first\n");
			ret = -EIO;
			goto out;
		}

		down_read(&gxp->debugfs_client->semaphore);
		if (!gxp_client_has_available_vd(gxp->debugfs_client,
						 "GXP_MAILBOX_COMMAND")) {
			ret = -ENODEV;
			up_read(&gxp->debugfs_client->semaphore);
			goto out;
		}
		up_read(&gxp->debugfs_client->semaphore);

		mbx = to_mcu_dev(gxp)->mcu.uci.mbx;
		if (!mbx) {
			dev_err(gxp->dev, "UCI is not initialized.\n");
			ret = -EIO;
			goto out;
		}

		cmd_code = CORE_COMMAND;
	}
#endif

	retval = gxp->mailbox_mgr->execute_cmd(client, mbx, core, cmd_code, 0,
					       0, 0, 0, 1, power_states, NULL,
					       &status);

	dev_info(
		gxp->dev,
		"Mailbox Command Sent: core=%d, resp.status=%d, resp.retval=%d\n",
		core, status, retval);
	ret = 0;
out:
	if (client && client != gxp->debugfs_client)
		gxp_client_destroy(client);
	mutex_unlock(&gxp->debugfs_client_lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(gxp_mailbox_fops, NULL, gxp_debugfs_mailbox, "%llu\n");

static int gxp_firmware_run_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	struct gxp_client *client;
	int ret = 0;
	uint core;
	bool acquired_block_wakelock;

	ret = gxp_firmware_loader_load_if_needed(gxp);
	if (ret) {
		dev_err(gxp->dev, "Unable to load firmware files\n");
		return ret;
	}

	mutex_lock(&gxp->debugfs_client_lock);

	if (val) {
		if (gxp->debugfs_client) {
			dev_err(gxp->dev, "Firmware is already running!\n");
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
		up_write(&gxp->vd_semaphore);

		client = gxp_client_create(gxp);
		if (IS_ERR(client)) {
			dev_err(gxp->dev, "Failed to create client\n");
			goto out;
		}
		gxp->debugfs_client = client;

		mutex_lock(&gxp->client_list_lock);
		list_add(&client->list_entry, &gxp->client_list);
		mutex_unlock(&gxp->client_list_lock);

		down_write(&client->semaphore);

		ret = gxp_client_allocate_virtual_device(client, GXP_NUM_CORES,
							 0);
		if (ret) {
			dev_err(gxp->dev, "Failed to allocate VD\n");
			goto err_destroy_client;
		}

		ret = gxp_client_acquire_block_wakelock(
			client, &acquired_block_wakelock);
		if (ret) {
			dev_err(gxp->dev, "Failed to acquire BLOCK wakelock\n");
			goto err_destroy_client;
		}

		ret = gxp_client_acquire_vd_wakelock(client, uud_states);
		if (ret) {
			dev_err(gxp->dev, "Failed to acquire VD wakelock\n");
			goto err_release_block_wakelock;
		}

		up_write(&client->semaphore);
	} else {
		if (!gxp->debugfs_client) {
			dev_err(gxp->dev, "Firmware is not running!\n");
			ret = -EIO;
			goto out;
		}

		/*
		 * Cleaning up the client will stop the VD it owns and release
		 * the BLOCK wakelock it is holding.
		 */
		goto out_destroy_client;
	}

out:
	mutex_unlock(&gxp->debugfs_client_lock);

	return ret;

err_release_block_wakelock:
	gxp_client_release_block_wakelock(client);
err_destroy_client:
	up_write(&client->semaphore);
out_destroy_client:
	mutex_lock(&gxp->client_list_lock);
	list_del(&gxp->debugfs_client->list_entry);
	mutex_unlock(&gxp->client_list_lock);

	/* Destroying a client cleans up any VDss or wakelocks it held. */
	gxp_client_destroy(gxp->debugfs_client);
	gxp->debugfs_client = NULL;
	mutex_unlock(&gxp->debugfs_client_lock);
	return ret;
}

static int gxp_firmware_run_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	down_read(&gxp->vd_semaphore);
	*val = gxp->firmware_mgr->firmware_running;
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

		ret = gcip_pm_get(gxp->power_mgr->pm);
		if (ret) {
			dev_err(gxp->dev, "gcip_pm_get failed ret=%d\n", ret);
			goto out;
		}
		gxp->debugfs_wakelock_held = true;
		gxp_pm_update_requested_power_states(gxp, off_states,
						     uud_states);
	} else {
		/* Wakelock Release */
		if (!gxp->debugfs_wakelock_held) {
			dev_warn(gxp->dev, "Debugfs wakelock not held.\n");
			ret = -EIO;
			goto out;
		}

		gcip_pm_put(gxp->power_mgr->pm);
		gxp->debugfs_wakelock_held = false;
		gxp_pm_update_requested_power_states(gxp, uud_states,
						     off_states);
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
	struct gxp_coherent_buf *buffers;
	u64 *ptr;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	if (!gxp->core_telemetry_mgr->logging_buff_data_legacy) {
		dev_err(gxp->dev, "Logging buffer has not been created");
		mutex_unlock(&gxp->core_telemetry_mgr->lock);
		return -ENODEV;
	}

	buffers = gxp->core_telemetry_mgr->logging_buff_data_legacy->buffers;
	for (i = 0; i < GXP_NUM_CORES; i++) {
		ptr = buffers[i].vaddr;
		*ptr = val;
	}

	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	return 0;
}

static int gxp_log_buff_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	struct gxp_coherent_buf *buffers;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	if (!gxp->core_telemetry_mgr->logging_buff_data_legacy) {
		dev_err(gxp->dev, "Logging buffer has not been created");
		mutex_unlock(&gxp->core_telemetry_mgr->lock);
		return -ENODEV;
	}

	buffers = gxp->core_telemetry_mgr->logging_buff_data_legacy->buffers;

	*val = *(u64 *)(buffers[0].vaddr);

	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_log_buff_fops, gxp_log_buff_get, gxp_log_buff_set,
			 "%llu\n");

static int gxp_log_eventfd_signal_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int ret = 0;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	if (!gxp->core_telemetry_mgr->logging_efd) {
		ret = -ENODEV;
		goto out;
	}

	ret = eventfd_signal(gxp->core_telemetry_mgr->logging_efd, 1);

out:
	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_log_eventfd_signal_fops, NULL,
			 gxp_log_eventfd_signal_set, "%llu\n");

static int gxp_cmu_mux1_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	if (IS_ERR_OR_NULL(gxp->cmu.vaddr)) {
		dev_err(gxp->dev, "CMU registers are not mapped");
		return -ENODEV;
	}
	if (val > 1) {
		dev_err(gxp->dev,
			"Incorrect val for cmu_mux1, only 0 and 1 allowed\n");
		return -EINVAL;
	}

	writel(val << 4, gxp->cmu.vaddr + PLL_CON0_PLL_AUR);
	return 0;
}

static int gxp_cmu_mux1_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	if (IS_ERR_OR_NULL(gxp->cmu.vaddr)) {
		dev_err(gxp->dev, "CMU registers are not mapped");
		return -ENODEV;
	}
	*val = readl(gxp->cmu.vaddr + PLL_CON0_PLL_AUR);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_cmu_mux1_fops, gxp_cmu_mux1_get, gxp_cmu_mux1_set,
			 "%llu\n");

static int gxp_cmu_mux2_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	if (IS_ERR_OR_NULL(gxp->cmu.vaddr)) {
		dev_err(gxp->dev, "CMU registers are not mapped");
		return -ENODEV;
	}
	if (val > 1) {
		dev_err(gxp->dev,
			"Incorrect val for cmu_mux2, only 0 and 1 allowed\n");
		return -EINVAL;
	}

	writel(val << 4, gxp->cmu.vaddr + PLL_CON0_NOC_USER);
	return 0;
}

static int gxp_cmu_mux2_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	if (IS_ERR_OR_NULL(gxp->cmu.vaddr)) {
		dev_err(gxp->dev, "CMU registers are not mapped");
		return -ENODEV;
	}
	*val = readl(gxp->cmu.vaddr + PLL_CON0_NOC_USER);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_cmu_mux2_fops, gxp_cmu_mux2_get, gxp_cmu_mux2_set,
			 "%llu\n");

void gxp_create_debugdir(struct gxp_dev *gxp)
{
	gxp->d_entry = debugfs_create_dir(GXP_NAME, NULL);
	if (IS_ERR_OR_NULL(gxp->d_entry)) {
		dev_warn(gxp->dev, "Create debugfs dir failed: %d",
			 PTR_ERR_OR_ZERO(gxp->d_entry));
		gxp->d_entry = NULL;
	}
}

void gxp_create_debugfs(struct gxp_dev *gxp)
{
	if (!gxp->d_entry)
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

void gxp_remove_debugdir(struct gxp_dev *gxp)
{
	if (!gxp->d_entry)
		return;
	debugfs_remove_recursive(gxp->d_entry);

	/*
	 * Now that debugfs is torn down, and no other calls to
	 * `gxp_firmware_run_set()` can occur, destroy any client that may have
	 * been left running.
	 */
	if (gxp->debugfs_client)
		gxp_client_destroy(gxp->debugfs_client);
}
