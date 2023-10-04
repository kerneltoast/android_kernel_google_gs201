// SPDX-License-Identifier: GPL-2.0
/*
 * Platform device driver for GXP.
 *
 * Copyright (C) 2021 Google LLC
 */

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
#include <linux/platform_data/sscoredump.h>
#endif

#include <linux/acpi.h>
#include <linux/cred.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/genalloc.h>
#include <linux/kthread.h>
#include <linux/log2.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>
#if (IS_ENABLED(CONFIG_GXP_TEST) || IS_ENABLED(CONFIG_ANDROID)) && !IS_ENABLED(CONFIG_GXP_GEM5)
#include <soc/google/tpu-ext.h>
#endif

#include "gxp-client.h"
#include "gxp-config.h"
#include "gxp-debug-dump.h"
#include "gxp-debugfs.h"
#include "gxp-dma.h"
#include "gxp-dmabuf.h"
#include "gxp-domain-pool.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-internal.h"
#include "gxp-mailbox.h"
#include "gxp-mailbox-driver.h"
#include "gxp-mapping.h"
#include "gxp-notification.h"
#include "gxp-pm.h"
#include "gxp-telemetry.h"
#include "gxp-thermal.h"
#include "gxp-vd.h"
#include "gxp-wakelock.h"
#include "gxp.h"

static struct gxp_dev *gxp_debug_pointer;

#define __wait_event_lock_irq_timeout_exclusive(wq_head, condition, lock,      \
						timeout, state)                \
	___wait_event(wq_head, ___wait_cond_timeout(condition), state, 1,      \
		      timeout, spin_unlock_irq(&lock);                         \
		      __ret = schedule_timeout(__ret); spin_lock_irq(&lock))

/*
 * wait_event_interruptible_lock_irq_timeout() but set the exclusive flag.
 */
#define wait_event_interruptible_lock_irq_timeout_exclusive(                   \
	wq_head, condition, lock, timeout)                                     \
	({                                                                     \
		long __ret = timeout;                                          \
		if (!___wait_cond_timeout(condition))                          \
			__ret = __wait_event_lock_irq_timeout_exclusive(       \
				wq_head, condition, lock, timeout,             \
				TASK_INTERRUPTIBLE);                           \
		__ret;                                                         \
	})

/* Caller needs to hold client->semaphore */
static bool check_client_has_available_vd(struct gxp_client *client,
					  char *ioctl_name)
{
	struct gxp_dev *gxp = client->gxp;

	lockdep_assert_held(&client->semaphore);
	if (!client->vd) {
		dev_err(gxp->dev,
			"%s requires the client allocate a VIRTUAL_DEVICE\n",
			ioctl_name);
		return false;
	}
	if (client->vd->state == GXP_VD_UNAVAILABLE) {
		dev_err(gxp->dev, "Cannot do %s on a broken virtual device\n",
			ioctl_name);
		return false;
	}
	return true;
}

/* Caller needs to hold client->semaphore for reading */
static bool check_client_has_available_vd_wakelock(struct gxp_client *client,
						   char *ioctl_name)
{
	struct gxp_dev *gxp = client->gxp;

	lockdep_assert_held_read(&client->semaphore);
	if (!client->has_vd_wakelock) {
		dev_err(gxp->dev,
			"%s requires the client hold a VIRTUAL_DEVICE wakelock\n",
			ioctl_name);
		return false;
	}
	if (client->vd->state == GXP_VD_UNAVAILABLE) {
		dev_err(gxp->dev, "Cannot do %s on a broken virtual device\n",
			ioctl_name);
		return false;
	}
	return true;
}

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
static struct sscd_platform_data gxp_sscd_pdata;

static void gxp_sscd_release(struct device *dev)
{
	pr_debug("%s\n", __func__);
}

static struct platform_device gxp_sscd_dev = {
	.name = GXP_DRIVER_NAME,
	.driver_override = SSCD_NAME,
	.id = -1,
	.dev = {
		.platform_data = &gxp_sscd_pdata,
		.release = gxp_sscd_release,
	},
};
#endif  // CONFIG_SUBSYSTEM_COREDUMP

/* Mapping from GXP_POWER_STATE_* to enum aur_power_state in gxp-pm.h */
static const uint aur_state_array[GXP_NUM_POWER_STATES] = {
	AUR_OFF,   AUR_UUD,	 AUR_SUD,      AUR_UD,	   AUR_NOM,
	AUR_READY, AUR_UUD_PLUS, AUR_SUD_PLUS, AUR_UD_PLUS
};
/* Mapping from MEMORY_POWER_STATE_* to enum aur_memory_power_state in gxp-pm.h */
static const uint aur_memory_state_array[MEMORY_POWER_STATE_MAX + 1] = {
	AUR_MEM_UNDEFINED, AUR_MEM_MIN,	      AUR_MEM_VERY_LOW, AUR_MEM_LOW,
	AUR_MEM_HIGH,	   AUR_MEM_VERY_HIGH, AUR_MEM_MAX
};

static int gxp_open(struct inode *inode, struct file *file)
{
	struct gxp_client *client;
	struct gxp_dev *gxp = container_of(file->private_data, struct gxp_dev,
					   misc_dev);
	int ret = 0;

	/* If this is the first call to open(), request the firmware files */
	ret = gxp_firmware_request_if_needed(gxp);
	if (ret) {
		dev_err(gxp->dev,
			"Failed to request dsp firmware files (ret=%d)\n", ret);
		return ret;
	}

	client = gxp_client_create(gxp);
	if (IS_ERR(client))
		return PTR_ERR(client);

	client->tgid = current->tgid;
	client->pid = current->pid;

	file->private_data = client;

	mutex_lock(&gxp->client_list_lock);
	list_add(&client->list_entry, &gxp->client_list);
	mutex_unlock(&gxp->client_list_lock);

	return ret;
}

static int gxp_release(struct inode *inode, struct file *file)
{
	struct gxp_client *client = file->private_data;

	/*
	 * If open failed and no client was created then no clean-up is needed.
	 */
	if (!client)
		return 0;

	if (client->enabled_telemetry_logging)
		gxp_telemetry_disable(client->gxp, GXP_TELEMETRY_TYPE_LOGGING);
	if (client->enabled_telemetry_tracing)
		gxp_telemetry_disable(client->gxp, GXP_TELEMETRY_TYPE_TRACING);

	mutex_lock(&client->gxp->client_list_lock);
	list_del(&client->list_entry);
	mutex_unlock(&client->gxp->client_list_lock);

	gxp_client_destroy(client);

	return 0;
}

static inline enum dma_data_direction mapping_flags_to_dma_dir(u32 flags)
{
	switch (flags & 0x3) {
	case 0x0: /* 0b00 */
		return DMA_BIDIRECTIONAL;
	case 0x1: /* 0b01 */
		return DMA_TO_DEVICE;
	case 0x2: /* 0b10 */
		return DMA_FROM_DEVICE;
	}

	return DMA_NONE;
}

static int gxp_map_buffer(struct gxp_client *client,
			  struct gxp_map_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_map_ioctl ibuf;
	struct gxp_mapping *map;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.size == 0 || ibuf.virtual_core_list == 0)
		return -EINVAL;

	if (ibuf.host_address % L1_CACHE_BYTES || ibuf.size % L1_CACHE_BYTES) {
		dev_err(gxp->dev,
			"Mapped buffers must be cache line aligned and padded.\n");
		return -EINVAL;
	}

	down_read(&client->semaphore);

	if (!check_client_has_available_vd(client, "GXP_MAP_BUFFER")) {
		ret = -ENODEV;
		goto out;
	}

	/* the list contains un-allocated core bits */
	if (ibuf.virtual_core_list & ~(BIT(client->vd->num_cores) - 1)) {
		ret = -EINVAL;
		goto out;
	}

	map = gxp_mapping_create(gxp, client->vd, ibuf.virtual_core_list,
				 ibuf.host_address, ibuf.size,
				 /*gxp_dma_flags=*/0,
				 mapping_flags_to_dma_dir(ibuf.flags));
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		dev_err(gxp->dev, "Failed to create mapping (ret=%d)\n", ret);
		goto out;
	}

	ret = gxp_vd_mapping_store(client->vd, map);
	if (ret) {
		dev_err(gxp->dev, "Failed to store mapping (ret=%d)\n", ret);
		goto error_destroy;
	}

	ibuf.device_address = map->device_address;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		ret = -EFAULT;
		goto error_remove;
	}

	/*
	 * The virtual device acquired its own reference to the mapping when
	 * it was stored in the VD's records. Release the reference from
	 * creating the mapping since this function is done using it.
	 */
	gxp_mapping_put(map);

out:
	up_read(&client->semaphore);

	return ret;

error_remove:
	gxp_vd_mapping_remove(client->vd, map);
error_destroy:
	gxp_mapping_put(map);
	up_read(&client->semaphore);
	return ret;
}

static int gxp_unmap_buffer(struct gxp_client *client,
			    struct gxp_map_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_map_ioctl ibuf;
	struct gxp_mapping *map;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_read(&client->semaphore);

	if (!client->vd) {
		dev_err(gxp->dev,
			"GXP_UNMAP_BUFFER requires the client allocate a VIRTUAL_DEVICE\n");
		ret = -ENODEV;
		goto out;
	}

	map = gxp_vd_mapping_search(client->vd,
				    (dma_addr_t)ibuf.device_address);
	if (!map) {
		dev_err(gxp->dev,
			"Mapping not found for provided device address %#llX\n",
			ibuf.device_address);
		ret = -EINVAL;
		goto out;
	} else if (!map->host_address) {
		dev_err(gxp->dev, "dma-bufs must be unmapped via GXP_UNMAP_DMABUF\n");
		ret = -EINVAL;
		goto out;
	}

	WARN_ON(map->host_address != ibuf.host_address);

	gxp_vd_mapping_remove(client->vd, map);

	/* Release the reference from gxp_vd_mapping_search() */
	gxp_mapping_put(map);

out:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_sync_buffer(struct gxp_client *client,
			   struct gxp_sync_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_sync_ioctl ibuf;
	struct gxp_mapping *map;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_read(&client->semaphore);

	if (!client->vd) {
		dev_err(gxp->dev,
			"GXP_SYNC_BUFFER requires the client allocate a VIRTUAL_DEVICE\n");
		ret = -ENODEV;
		goto out;
	}

	map = gxp_vd_mapping_search(client->vd,
				    (dma_addr_t)ibuf.device_address);
	if (!map) {
		dev_err(gxp->dev,
			"Mapping not found for provided device address %#llX\n",
			ibuf.device_address);
		ret = -EINVAL;
		goto out;
	}

	ret = gxp_mapping_sync(map, ibuf.offset, ibuf.size,
			       ibuf.flags == GXP_SYNC_FOR_CPU);

	/* Release the reference from gxp_vd_mapping_search() */
	gxp_mapping_put(map);

out:
	up_read(&client->semaphore);

	return ret;
}

static int
gxp_mailbox_command_compat(struct gxp_client *client,
			   struct gxp_mailbox_command_compat_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_mailbox_command_compat_ioctl ibuf;
	struct gxp_command cmd;
	struct buffer_descriptor buffer;
	int virt_core, phys_core;
	int ret = 0;
	uint gxp_power_state, memory_power_state;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf))) {
		dev_err(gxp->dev,
			"Unable to copy ioctl data from user-space\n");
		return -EFAULT;
	}

	/* Caller must hold VIRTUAL_DEVICE wakelock */
	down_read(&client->semaphore);

	if (!check_client_has_available_vd_wakelock(client,
						   "GXP_MAILBOX_COMMAND")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	down_read(&gxp->vd_semaphore);

	virt_core = ibuf.virtual_core_id;
	phys_core = gxp_vd_virt_core_to_phys_core(client->vd, virt_core);
	if (phys_core < 0) {
		dev_err(gxp->dev,
			"Mailbox command failed: Invalid virtual core id (%u)\n",
			virt_core);
		ret = -EINVAL;
		goto out;
	}

	if (!gxp_is_fw_running(gxp, phys_core)) {
		dev_err(gxp->dev,
			"Cannot process mailbox command for core %d when firmware isn't running\n",
			phys_core);
		ret = -EINVAL;
		goto out;
	}

	if (gxp->mailbox_mgr == NULL || gxp->mailbox_mgr->mailboxes == NULL ||
	    gxp->mailbox_mgr->mailboxes[phys_core] == NULL) {
		dev_err(gxp->dev, "Mailbox not initialized for core %d\n",
			phys_core);
		ret = -EIO;
		goto out;
	}

	/* Pack the command structure */
	buffer.address = ibuf.device_address;
	buffer.size = ibuf.size;
	buffer.flags = ibuf.flags;
	/* cmd.seq is assigned by mailbox implementation */
	cmd.code = GXP_MBOX_CODE_DISPATCH; /* All IOCTL commands are dispatch */
	cmd.priority = 0; /* currently unused */
	cmd.buffer_descriptor = buffer;
	gxp_power_state = AUR_OFF;
	memory_power_state = AUR_MEM_UNDEFINED;

	ret = gxp_mailbox_execute_cmd_async(
		gxp->mailbox_mgr->mailboxes[phys_core], &cmd,
		&client->vd->mailbox_resp_queues[virt_core].queue,
		&client->vd->mailbox_resp_queues[virt_core].lock,
		&client->vd->mailbox_resp_queues[virt_core].waitq,
		gxp_power_state, memory_power_state, false,
		client->mb_eventfds[virt_core]);
	if (ret) {
		dev_err(gxp->dev, "Failed to enqueue mailbox command (ret=%d)\n",
			ret);
		goto out;
	}

	ibuf.sequence_number = cmd.seq;
	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		dev_err(gxp->dev, "Failed to copy back sequence number!\n");
		ret = -EFAULT;
		goto out;
	}

out:
	up_read(&gxp->vd_semaphore);
out_unlock_client_semaphore:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_mailbox_command(struct gxp_client *client,
			       struct gxp_mailbox_command_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_mailbox_command_ioctl ibuf;
	struct gxp_command cmd;
	struct buffer_descriptor buffer;
	int virt_core, phys_core;
	int ret = 0;
	uint gxp_power_state, memory_power_state;
	bool requested_low_clkmux = false;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf))) {
		dev_err(gxp->dev,
			"Unable to copy ioctl data from user-space\n");
		return -EFAULT;
	}
	if (ibuf.gxp_power_state == GXP_POWER_STATE_OFF) {
		dev_err(gxp->dev,
			"GXP_POWER_STATE_OFF is not a valid value when executing a mailbox command\n");
		return -EINVAL;
	}
	if (ibuf.gxp_power_state < GXP_POWER_STATE_OFF ||
	    ibuf.gxp_power_state >= GXP_NUM_POWER_STATES) {
		dev_err(gxp->dev, "Requested power state is invalid\n");
		return -EINVAL;
	}
	if (ibuf.memory_power_state < MEMORY_POWER_STATE_UNDEFINED ||
	    ibuf.memory_power_state > MEMORY_POWER_STATE_MAX) {
		dev_err(gxp->dev, "Requested memory power state is invalid\n");
		return -EINVAL;
	}

	if (ibuf.gxp_power_state == GXP_POWER_STATE_READY) {
		dev_warn_once(
			gxp->dev,
			"GXP_POWER_STATE_READY is deprecated, please set GXP_POWER_LOW_FREQ_CLKMUX with GXP_POWER_STATE_UUD state");
		ibuf.gxp_power_state = GXP_POWER_STATE_UUD;
	}

	if(ibuf.power_flags & GXP_POWER_NON_AGGRESSOR)
		dev_warn_once(
			gxp->dev,
			"GXP_POWER_NON_AGGRESSOR is deprecated, no operation here");

	/* Caller must hold VIRTUAL_DEVICE wakelock */
	down_read(&client->semaphore);

	if (!check_client_has_available_vd_wakelock(client,
						   "GXP_MAILBOX_COMMAND")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	down_read(&gxp->vd_semaphore);

	virt_core = ibuf.virtual_core_id;
	phys_core = gxp_vd_virt_core_to_phys_core(client->vd, virt_core);
	if (phys_core < 0) {
		dev_err(gxp->dev,
			"Mailbox command failed: Invalid virtual core id (%u)\n",
			virt_core);
		ret = -EINVAL;
		goto out;
	}

	if (!gxp_is_fw_running(gxp, phys_core)) {
		dev_err(gxp->dev,
			"Cannot process mailbox command for core %d when firmware isn't running\n",
			phys_core);
		ret = -EINVAL;
		goto out;
	}

	if (gxp->mailbox_mgr == NULL || gxp->mailbox_mgr->mailboxes == NULL ||
	    gxp->mailbox_mgr->mailboxes[phys_core] == NULL) {
		dev_err(gxp->dev, "Mailbox not initialized for core %d\n",
			phys_core);
		ret = -EIO;
		goto out;
	}

	/* Pack the command structure */
	buffer.address = ibuf.device_address;
	buffer.size = ibuf.size;
	buffer.flags = ibuf.flags;
	/* cmd.seq is assigned by mailbox implementation */
	cmd.code = GXP_MBOX_CODE_DISPATCH; /* All IOCTL commands are dispatch */
	cmd.priority = 0; /* currently unused */
	cmd.buffer_descriptor = buffer;
	gxp_power_state = aur_state_array[ibuf.gxp_power_state];
	memory_power_state = aur_memory_state_array[ibuf.memory_power_state];
	requested_low_clkmux = (ibuf.power_flags & GXP_POWER_LOW_FREQ_CLKMUX) != 0;

	ret = gxp_mailbox_execute_cmd_async(
		gxp->mailbox_mgr->mailboxes[phys_core], &cmd,
		&client->vd->mailbox_resp_queues[virt_core].queue,
		&client->vd->mailbox_resp_queues[virt_core].lock,
		&client->vd->mailbox_resp_queues[virt_core].waitq,
		gxp_power_state, memory_power_state, requested_low_clkmux,
		client->mb_eventfds[virt_core]);
	if (ret) {
		dev_err(gxp->dev, "Failed to enqueue mailbox command (ret=%d)\n",
			ret);
		goto out;
	}

	ibuf.sequence_number = cmd.seq;
	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		dev_err(gxp->dev, "Failed to copy back sequence number!\n");
		ret = -EFAULT;
		goto out;
	}

out:
	up_read(&gxp->vd_semaphore);
out_unlock_client_semaphore:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_mailbox_response(struct gxp_client *client,
				struct gxp_mailbox_response_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_mailbox_response_ioctl ibuf;
	struct gxp_async_response *resp_ptr;
	int virt_core;
	int ret = 0;
	long timeout;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	/* Caller must hold VIRTUAL_DEVICE wakelock */
	down_read(&client->semaphore);

	if (!check_client_has_available_vd_wakelock(client,
						    "GXP_MAILBOX_RESPONSE")) {
		ret = -ENODEV;
		goto out;
	}

	virt_core = ibuf.virtual_core_id;
	if (virt_core >= client->vd->num_cores) {
		dev_err(gxp->dev, "Mailbox response failed: Invalid virtual core id (%u)\n",
			virt_core);
		ret = -EINVAL;
		goto out;
	}

	spin_lock_irq(&client->vd->mailbox_resp_queues[virt_core].lock);

	/*
	 * The "exclusive" version of wait_event is used since each wake
	 * corresponds to the addition of exactly one new response to be
	 * consumed. Therefore, only one waiting response ioctl can ever
	 * proceed per wake event.
	 */
	timeout = wait_event_interruptible_lock_irq_timeout_exclusive(
		client->vd->mailbox_resp_queues[virt_core].waitq,
		!list_empty(&client->vd->mailbox_resp_queues[virt_core].queue),
		client->vd->mailbox_resp_queues[virt_core].lock,
		msecs_to_jiffies(MAILBOX_TIMEOUT));
	if (timeout <= 0) {
		spin_unlock_irq(
			&client->vd->mailbox_resp_queues[virt_core].lock);
		/* unusual case - this only happens when there is no command pushed */
		ret = timeout ? -ETIMEDOUT : timeout;
		goto out;
	}
	resp_ptr = list_first_entry(
		&client->vd->mailbox_resp_queues[virt_core].queue,
		struct gxp_async_response, list_entry);

	/* Pop the front of the response list */
	list_del(&(resp_ptr->list_entry));

	spin_unlock_irq(&client->vd->mailbox_resp_queues[virt_core].lock);

	ibuf.sequence_number = resp_ptr->resp.seq;
	switch (resp_ptr->resp.status) {
	case GXP_RESP_OK:
		ibuf.error_code = GXP_RESPONSE_ERROR_NONE;
		/* retval is only valid if status == GXP_RESP_OK */
		ibuf.cmd_retval = resp_ptr->resp.retval;
		break;
	case GXP_RESP_CANCELLED:
		ibuf.error_code = GXP_RESPONSE_ERROR_TIMEOUT;
		break;
	default:
		/* No other status values are valid at this point */
		WARN(true, "Completed response had invalid status %hu",
		     resp_ptr->resp.status);
		ibuf.error_code = GXP_RESPONSE_ERROR_INTERNAL;
		break;
	}

	/*
	 * We must be absolutely sure the timeout work has been cancelled
	 * and/or completed before freeing the `gxp_async_response`.
	 * There are 3 possible cases when we arrive at this point:
	 *   1) The response arrived normally and the timeout was cancelled
	 *   2) The response timedout and its timeout handler finished
	 *   3) The response handler and timeout handler raced, and the response
	 *      handler "cancelled" the timeout handler while it was already in
	 *      progress.
	 *
	 * This call handles case #3, and ensures any in-process timeout
	 * handler (which may reference the `gxp_async_response`) has
	 * been able to exit cleanly.
	 */
	cancel_delayed_work_sync(&resp_ptr->timeout_work);
	kfree(resp_ptr);

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		ret = -EFAULT;

out:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_get_specs(struct gxp_client *client,
			 struct gxp_specs_ioctl __user *argp)
{
	struct gxp_specs_ioctl ibuf = {
		.core_count = GXP_NUM_CORES,
		.memory_per_core = client->gxp->memory_per_core,
	};

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return 0;
}

static int gxp_allocate_vd(struct gxp_client *client,
			   struct gxp_virtual_device_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_virtual_device_ioctl ibuf;
	struct gxp_virtual_device *vd;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.core_count == 0 || ibuf.core_count > GXP_NUM_CORES) {
		dev_err(gxp->dev, "Invalid core count (%u)\n", ibuf.core_count);
		return -EINVAL;
	}

	if (ibuf.memory_per_core > gxp->memory_per_core) {
		dev_err(gxp->dev, "Invalid memory-per-core (%u)\n",
			ibuf.memory_per_core);
		return -EINVAL;
	}

	down_write(&client->semaphore);

	if (client->vd) {
		dev_err(gxp->dev, "Virtual device was already allocated for client\n");
		ret = -EINVAL;
		goto out;
	}

	vd = gxp_vd_allocate(gxp, ibuf.core_count);
	if (IS_ERR(vd)) {
		ret = PTR_ERR(vd);
		dev_err(gxp->dev,
			"Failed to allocate virtual device for client (%d)\n",
			ret);
		goto out;
	}

	client->vd = vd;

out:
	up_write(&client->semaphore);

	return ret;
}

static int
gxp_etm_trace_start_command(struct gxp_client *client,
			    struct gxp_etm_trace_start_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_etm_trace_start_ioctl ibuf;
	int phys_core;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	ibuf.trace_ram_enable &= ETM_TRACE_LSB_MASK;
	ibuf.atb_enable &= ETM_TRACE_LSB_MASK;

	if (!ibuf.trace_ram_enable && !ibuf.atb_enable)
		return -EINVAL;

	if (!(ibuf.sync_msg_period == 0 ||
	    (ibuf.sync_msg_period <= ETM_TRACE_SYNC_MSG_PERIOD_MAX &&
	     ibuf.sync_msg_period >= ETM_TRACE_SYNC_MSG_PERIOD_MIN &&
	     is_power_of_2(ibuf.sync_msg_period))))
		return -EINVAL;

	if (ibuf.pc_match_mask_length > ETM_TRACE_PC_MATCH_MASK_LEN_MAX)
		return -EINVAL;

	/* Caller must hold VIRTUAL_DEVICE wakelock */
	down_read(&client->semaphore);

	if (!check_client_has_available_vd_wakelock(
		    client, "GXP_ETM_TRACE_START_COMMAND")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	down_read(&gxp->vd_semaphore);

	phys_core =
		gxp_vd_virt_core_to_phys_core(client->vd, ibuf.virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Trace start failed: Invalid virtual core id (%u)\n",
			ibuf.virtual_core_id);
		ret = -EINVAL;
		goto out;
	}

	/*
	 * TODO (b/185260919): Pass the etm trace configuration to system FW
	 * once communication channel between kernel and system FW is ready
	 * (b/185819530).
	 */

out:
	up_read(&gxp->vd_semaphore);
out_unlock_client_semaphore:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_etm_trace_sw_stop_command(struct gxp_client *client,
					 __u16 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	u16 virtual_core_id;
	int phys_core;
	int ret = 0;

	if (copy_from_user(&virtual_core_id, argp, sizeof(virtual_core_id)))
		return -EFAULT;

	/* Caller must hold VIRTUAL_DEVICE wakelock */
	down_read(&client->semaphore);

	if (!check_client_has_available_vd_wakelock(
		    client, "GXP_ETM_TRACE_SW_STOP_COMMAND")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	down_read(&gxp->vd_semaphore);

	phys_core = gxp_vd_virt_core_to_phys_core(client->vd, virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Trace stop via software trigger failed: Invalid virtual core id (%u)\n",
			virtual_core_id);
		ret = -EINVAL;
		goto out;
	}

	/*
	 * TODO (b/185260919): Pass the etm stop signal to system FW once
	 * communication channel between kernel and system FW is ready
	 * (b/185819530).
	 */

out:
	up_read(&gxp->vd_semaphore);
out_unlock_client_semaphore:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_etm_trace_cleanup_command(struct gxp_client *client,
					 __u16 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	u16 virtual_core_id;
	int phys_core;
	int ret = 0;

	if (copy_from_user(&virtual_core_id, argp, sizeof(virtual_core_id)))
		return -EFAULT;

	/* Caller must hold VIRTUAL_DEVICE wakelock */
	down_read(&client->semaphore);

	if (!check_client_has_available_vd_wakelock(
		    client, "GXP_ETM_TRACE_CLEANUP_COMMAND")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	down_read(&gxp->vd_semaphore);

	phys_core = gxp_vd_virt_core_to_phys_core(client->vd, virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Trace cleanup failed: Invalid virtual core id (%u)\n",
			virtual_core_id);
		ret = -EINVAL;
		goto out;
	}

	/*
	 * TODO (b/185260919): Pass the etm clean up signal to system FW once
	 * communication channel between kernel and system FW is ready
	 * (b/185819530).
	 */

out:
	up_read(&gxp->vd_semaphore);
out_unlock_client_semaphore:
	up_read(&client->semaphore);

	return ret;
}

static int
gxp_etm_get_trace_info_command(struct gxp_client *client,
			       struct gxp_etm_get_trace_info_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_etm_get_trace_info_ioctl ibuf;
	int phys_core;
	u32 *trace_header;
	u32 *trace_data;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.type > 1)
		return -EINVAL;

	/* Caller must hold VIRTUAL_DEVICE wakelock */
	down_read(&client->semaphore);

	if (!check_client_has_available_vd_wakelock(
		    client, "GXP_ETM_GET_TRACE_INFO_COMMAND")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	down_read(&gxp->vd_semaphore);

	phys_core = gxp_vd_virt_core_to_phys_core(client->vd, ibuf.virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Get trace info failed: Invalid virtual core id (%u)\n",
			ibuf.virtual_core_id);
		ret = -EINVAL;
		goto out;
	}

	trace_header = kzalloc(GXP_TRACE_HEADER_SIZE, GFP_KERNEL);
	if (!trace_header) {
		ret = -ENOMEM;
		goto out;
	}

	trace_data = kzalloc(GXP_TRACE_RAM_SIZE, GFP_KERNEL);
	if (!trace_data) {
		ret = -ENOMEM;
		goto out_free_header;
	}

	/*
	 * TODO (b/185260919): Get trace information from system FW once
	 * communication channel between kernel and system FW is ready
	 * (b/185819530).
	 */

	if (copy_to_user((void __user *)ibuf.trace_header_addr, trace_header,
			 GXP_TRACE_HEADER_SIZE)) {
		ret = -EFAULT;
		goto out_free_data;
	}

	if (ibuf.type == 1) {
		if (copy_to_user((void __user *)ibuf.trace_data_addr,
				 trace_data, GXP_TRACE_RAM_SIZE)) {
			ret = -EFAULT;
			goto out_free_data;
		}
	}

out_free_data:
	kfree(trace_data);
out_free_header:
	kfree(trace_header);

out:
	up_read(&gxp->vd_semaphore);
out_unlock_client_semaphore:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_enable_telemetry(struct gxp_client *client,
				__u8 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	__u8 type;
	int ret;

	if (copy_from_user(&type, argp, sizeof(type)))
		return -EFAULT;

	if (type != GXP_TELEMETRY_TYPE_LOGGING &&
	    type != GXP_TELEMETRY_TYPE_TRACING)
		return -EINVAL;

	ret = gxp_telemetry_enable(gxp, type);

	/*
	 * Record what telemetry types this client enabled so they can be
	 * cleaned-up if the client closes without disabling them.
	 */
	if (!ret && type == GXP_TELEMETRY_TYPE_LOGGING)
		client->enabled_telemetry_logging = true;
	if (!ret && type == GXP_TELEMETRY_TYPE_TRACING)
		client->enabled_telemetry_tracing = true;

	return ret;
}

static int gxp_disable_telemetry(struct gxp_client *client, __u8 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	__u8 type;
	int ret;

	if (copy_from_user(&type, argp, sizeof(type)))
		return -EFAULT;

	if (type != GXP_TELEMETRY_TYPE_LOGGING &&
	    type != GXP_TELEMETRY_TYPE_TRACING)
		return -EINVAL;

	ret = gxp_telemetry_disable(gxp, type);

	if (!ret && type == GXP_TELEMETRY_TYPE_LOGGING)
		client->enabled_telemetry_logging = false;
	if (!ret && type == GXP_TELEMETRY_TYPE_TRACING)
		client->enabled_telemetry_tracing = false;

	return ret;
}

static int gxp_map_tpu_mbx_queue(struct gxp_client *client,
				 struct gxp_tpu_mbx_queue_ioctl __user *argp)
{
#if (IS_ENABLED(CONFIG_GXP_TEST) || IS_ENABLED(CONFIG_ANDROID)) && !IS_ENABLED(CONFIG_GXP_GEM5)
	struct gxp_dev *gxp = client->gxp;
	struct edgetpu_ext_mailbox_info *mbx_info;
	struct gxp_tpu_mbx_queue_ioctl ibuf;
	struct edgetpu_ext_client_info gxp_tpu_info;
	u32 phys_core_list = 0;
	u32 virtual_core_list;
	u32 core_count;
	int ret = 0;

	if (!gxp->tpu_dev.mbx_paddr) {
		dev_err(gxp->dev, "%s: TPU is not available for interop\n",
			__func__);
		return -EINVAL;
	}

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_write(&client->semaphore);

	if (!check_client_has_available_vd(client, "GXP_MAP_TPU_MBX_QUEUE")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	down_read(&gxp->vd_semaphore);

	virtual_core_list = ibuf.virtual_core_list;
	core_count = hweight_long(virtual_core_list);
	phys_core_list = gxp_vd_virt_core_list_to_phys_core_list(
		client->vd, virtual_core_list);
	if (!phys_core_list) {
		dev_err(gxp->dev, "%s: invalid virtual core list 0x%x\n",
			__func__, virtual_core_list);
		ret = -EINVAL;
		goto out;
	}

	mbx_info =
		kmalloc(sizeof(struct edgetpu_ext_mailbox_info) + core_count *
			sizeof(struct edgetpu_ext_mailbox_descriptor),
			GFP_KERNEL);
	if (!mbx_info) {
		ret = -ENOMEM;
		goto out;
	}

	if (client->tpu_file) {
		dev_err(gxp->dev, "Mappings already exist for TPU mailboxes");
		ret = -EBUSY;
		goto out_free;
	}

	gxp_tpu_info.tpu_fd = ibuf.tpu_fd;
	gxp_tpu_info.mbox_map = phys_core_list;
	gxp_tpu_info.attr = (struct edgetpu_mailbox_attr __user *)ibuf.attr_ptr;
	ret = edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
				     EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
				     ALLOCATE_EXTERNAL_MAILBOX, &gxp_tpu_info,
				     mbx_info);
	if (ret) {
		dev_err(gxp->dev, "Failed to allocate ext TPU mailboxes %d",
			ret);
		goto out_free;
	}
	/*
	 * If someone is attacking us through this interface -
	 * it's possible that ibuf.tpu_fd here is already a different file from
	 * the one passed to edgetpu_ext_driver_cmd() (if the runtime closes the
	 * FD and opens another file exactly between the TPU driver call above
	 * and the fget below).
	 * But the worst consequence of this attack is we fget() ourselves (GXP
	 * FD), which only leads to memory leak (because the file object has a
	 * reference to itself). The race is also hard to hit so we don't insist
	 * on preventing it.
	 */
	client->tpu_file = fget(ibuf.tpu_fd);
	if (!client->tpu_file) {
		edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
				       EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
				       FREE_EXTERNAL_MAILBOX, &gxp_tpu_info,
				       NULL);
		ret = -EINVAL;
		goto out_free;
	}
	/* Align queue size to page size for iommu map. */
	mbx_info->cmdq_size = ALIGN(mbx_info->cmdq_size, PAGE_SIZE);
	mbx_info->respq_size = ALIGN(mbx_info->respq_size, PAGE_SIZE);

	ret = gxp_dma_map_tpu_buffer(gxp, client->vd, virtual_core_list,
				     phys_core_list, mbx_info);
	if (ret) {
		dev_err(gxp->dev, "Failed to map TPU mailbox buffer %d", ret);
		fput(client->tpu_file);
		client->tpu_file = NULL;
		edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
				       EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
				       FREE_EXTERNAL_MAILBOX, &gxp_tpu_info,
				       NULL);
		goto out_free;
	}
	client->mbx_desc.phys_core_list = phys_core_list;
	client->mbx_desc.virt_core_list = virtual_core_list;
	client->mbx_desc.cmdq_size = mbx_info->cmdq_size;
	client->mbx_desc.respq_size = mbx_info->respq_size;

out_free:
	kfree(mbx_info);

out:
	up_read(&gxp->vd_semaphore);
out_unlock_client_semaphore:
	up_write(&client->semaphore);

	return ret;
#else
	return -ENODEV;
#endif
}

static int gxp_unmap_tpu_mbx_queue(struct gxp_client *client,
				   struct gxp_tpu_mbx_queue_ioctl __user *argp)
{
#if (IS_ENABLED(CONFIG_GXP_TEST) || IS_ENABLED(CONFIG_ANDROID)) && !IS_ENABLED(CONFIG_GXP_GEM5)
	struct gxp_dev *gxp = client->gxp;
	struct gxp_tpu_mbx_queue_ioctl ibuf;
	struct edgetpu_ext_client_info gxp_tpu_info;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_write(&client->semaphore);

	if (!client->vd) {
		dev_err(gxp->dev,
			"GXP_UNMAP_TPU_MBX_QUEUE requires the client allocate a VIRTUAL_DEVICE\n");
		ret = -ENODEV;
		goto out;
	}

	if (!client->tpu_file) {
		dev_err(gxp->dev, "No mappings exist for TPU mailboxes");
		ret = -EINVAL;
		goto out;
	}

	gxp_dma_unmap_tpu_buffer(gxp, client->vd, client->mbx_desc);

	gxp_tpu_info.tpu_fd = ibuf.tpu_fd;
	edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
			       EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
			       FREE_EXTERNAL_MAILBOX, &gxp_tpu_info, NULL);
	fput(client->tpu_file);
	client->tpu_file = NULL;

out:
	up_write(&client->semaphore);

	return ret;
#else
	return -ENODEV;
#endif
}

static int gxp_register_telemetry_eventfd(
	struct gxp_client *client,
	struct gxp_register_telemetry_eventfd_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_register_telemetry_eventfd_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	return gxp_telemetry_register_eventfd(gxp, ibuf.type, ibuf.eventfd);
}

static int gxp_unregister_telemetry_eventfd(
	struct gxp_client *client,
	struct gxp_register_telemetry_eventfd_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_register_telemetry_eventfd_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	return gxp_telemetry_unregister_eventfd(gxp, ibuf.type);
}

static int gxp_read_global_counter(struct gxp_client *client,
				   __u64 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	u32 high_first, high_second, low;
	u64 counter_val;
	int ret = 0;

	/* Caller must hold BLOCK wakelock */
	down_read(&client->semaphore);

	if (!client->has_block_wakelock) {
		dev_err(gxp->dev,
			"GXP_READ_GLOBAL_COUNTER requires the client hold a BLOCK wakelock\n");
		ret = -ENODEV;
		goto out;
	}

	high_first = gxp_read_32(gxp, GXP_REG_GLOBAL_COUNTER_HIGH);
	low = gxp_read_32(gxp, GXP_REG_GLOBAL_COUNTER_LOW);

	/*
	 * Check if the lower 32 bits could have wrapped in-between reading
	 * the high and low bit registers by validating the higher 32 bits
	 * haven't changed.
	 */
	high_second = gxp_read_32(gxp, GXP_REG_GLOBAL_COUNTER_HIGH);
	if (high_first != high_second)
		low = gxp_read_32(gxp, GXP_REG_GLOBAL_COUNTER_LOW);

	counter_val = ((u64)high_second << 32) | low;

	if (copy_to_user(argp, &counter_val, sizeof(counter_val)))
		ret = -EFAULT;

out:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_acquire_wake_lock_compat(
	struct gxp_client *client,
	struct gxp_acquire_wakelock_compat_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_acquire_wakelock_compat_ioctl ibuf;
	bool acquired_block_wakelock = false;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.gxp_power_state == GXP_POWER_STATE_OFF) {
		dev_err(gxp->dev,
			"GXP_POWER_STATE_OFF is not a valid value when acquiring a wakelock\n");
		return -EINVAL;
	}
	if (ibuf.gxp_power_state < GXP_POWER_STATE_OFF ||
	    ibuf.gxp_power_state >= GXP_NUM_POWER_STATES) {
		dev_err(gxp->dev, "Requested power state is invalid\n");
		return -EINVAL;
	}
	if ((ibuf.memory_power_state < MEMORY_POWER_STATE_MIN ||
	     ibuf.memory_power_state > MEMORY_POWER_STATE_MAX) &&
	    ibuf.memory_power_state != MEMORY_POWER_STATE_UNDEFINED) {
		dev_err(gxp->dev,
			"Requested memory power state %d is invalid\n",
			ibuf.memory_power_state);
		return -EINVAL;
	}

	if (ibuf.gxp_power_state == GXP_POWER_STATE_READY) {
		dev_warn_once(
			gxp->dev,
			"GXP_POWER_STATE_READY is deprecated, please set GXP_POWER_LOW_FREQ_CLKMUX with GXP_POWER_STATE_UUD state");
		ibuf.gxp_power_state = GXP_POWER_STATE_UUD;
	}

	down_write(&client->semaphore);
	if ((ibuf.components_to_wake & WAKELOCK_VIRTUAL_DEVICE) &&
	    (!client->vd)) {
		dev_err(gxp->dev,
			"Must allocate a virtual device to acquire VIRTUAL_DEVICE wakelock\n");
		ret = -EINVAL;
		goto out;
	}

	/* Acquire a BLOCK wakelock if requested */
	if (ibuf.components_to_wake & WAKELOCK_BLOCK) {
		if (!client->has_block_wakelock) {
			ret = gxp_wakelock_acquire(gxp);
			acquired_block_wakelock = true;
		}

		if (ret) {
			dev_err(gxp->dev,
				"Failed to acquire BLOCK wakelock for client (ret=%d)\n",
				ret);
			goto out;
		}

		client->has_block_wakelock = true;

		/*
		 * Update client's TGID/PID in case the process that opened
		 * /dev/gxp is not the one that called this IOCTL.
		 */
		client->tgid = current->tgid;
		client->pid = current->pid;
	}

	/* Acquire a VIRTUAL_DEVICE wakelock if requested */
	if (ibuf.components_to_wake & WAKELOCK_VIRTUAL_DEVICE) {
		if (!client->has_block_wakelock) {
			dev_err(gxp->dev,
				"Must hold BLOCK wakelock to acquire VIRTUAL_DEVICE wakelock\n");
			ret = -EINVAL;
			goto out;

		}

		if (client->vd->state == GXP_VD_UNAVAILABLE) {
			dev_err(gxp->dev,
				"Cannot acquire VIRTUAL_DEVICE wakelock on a broken virtual device\n");
			ret = -ENODEV;
			goto out;
		}

		if (!client->has_vd_wakelock) {
			down_write(&gxp->vd_semaphore);
			if (client->vd->state == GXP_VD_OFF)
				ret = gxp_vd_start(client->vd);
			else
				ret = gxp_vd_resume(client->vd);
			up_write(&gxp->vd_semaphore);
		}

		if (ret) {
			dev_err(gxp->dev,
				"Failed to acquire VIRTUAL_DEVICE wakelock for client (ret=%d)\n",
				ret);
			goto err_acquiring_vd_wl;
		}

		client->has_vd_wakelock = true;
	}

	gxp_pm_update_requested_power_states(
		gxp, client->requested_power_state, client->requested_low_clkmux,
		aur_state_array[ibuf.gxp_power_state], false,
		client->requested_memory_power_state,
		aur_memory_state_array[ibuf.memory_power_state]);
	client->requested_power_state = aur_state_array[ibuf.gxp_power_state];
	client->requested_low_clkmux = false;
	client->requested_memory_power_state =
		aur_memory_state_array[ibuf.memory_power_state];
out:
	up_write(&client->semaphore);

	return ret;

err_acquiring_vd_wl:
	/*
	 * In a single call, if any wakelock acquisition fails, all of them do.
	 * If the client was acquiring both wakelocks and failed to acquire the
	 * VIRTUAL_DEVICE wakelock after successfully acquiring the BLOCK
	 * wakelock, then release it before returning the error code.
	 */
	if (acquired_block_wakelock) {
		gxp_wakelock_release(gxp);
		client->has_block_wakelock = false;
	}

	up_write(&client->semaphore);

	return ret;
}

static int gxp_acquire_wake_lock(struct gxp_client *client,
				 struct gxp_acquire_wakelock_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_acquire_wakelock_ioctl ibuf;
	bool acquired_block_wakelock = false;
	bool requested_low_clkmux = false;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.gxp_power_state == GXP_POWER_STATE_OFF) {
		dev_err(gxp->dev,
			"GXP_POWER_STATE_OFF is not a valid value when acquiring a wakelock\n");
		return -EINVAL;
	}
	if (ibuf.gxp_power_state < GXP_POWER_STATE_OFF ||
	    ibuf.gxp_power_state >= GXP_NUM_POWER_STATES) {
		dev_err(gxp->dev, "Requested power state is invalid\n");
		return -EINVAL;
	}
	if ((ibuf.memory_power_state < MEMORY_POWER_STATE_MIN ||
	     ibuf.memory_power_state > MEMORY_POWER_STATE_MAX) &&
	    ibuf.memory_power_state != MEMORY_POWER_STATE_UNDEFINED) {
		dev_err(gxp->dev,
			"Requested memory power state %d is invalid\n",
			ibuf.memory_power_state);
		return -EINVAL;
	}

	if (ibuf.gxp_power_state == GXP_POWER_STATE_READY) {
		dev_warn_once(
			gxp->dev,
			"GXP_POWER_STATE_READY is deprecated, please set GXP_POWER_LOW_FREQ_CLKMUX with GXP_POWER_STATE_UUD state");
		ibuf.gxp_power_state = GXP_POWER_STATE_UUD;
	}

	if(ibuf.flags & GXP_POWER_NON_AGGRESSOR)
		dev_warn_once(
			gxp->dev,
			"GXP_POWER_NON_AGGRESSOR is deprecated, no operation here");

	down_write(&client->semaphore);
	if ((ibuf.components_to_wake & WAKELOCK_VIRTUAL_DEVICE) &&
	    (!client->vd)) {
		dev_err(gxp->dev,
			"Must allocate a virtual device to acquire VIRTUAL_DEVICE wakelock\n");
		ret = -EINVAL;
		goto out;
	}

	/* Acquire a BLOCK wakelock if requested */
	if (ibuf.components_to_wake & WAKELOCK_BLOCK) {
		if (!client->has_block_wakelock) {
			ret = gxp_wakelock_acquire(gxp);
			acquired_block_wakelock = true;
		}

		if (ret) {
			dev_err(gxp->dev,
				"Failed to acquire BLOCK wakelock for client (ret=%d)\n",
				ret);
			goto out;
		}

		client->has_block_wakelock = true;
	}

	/* Acquire a VIRTUAL_DEVICE wakelock if requested */
	if (ibuf.components_to_wake & WAKELOCK_VIRTUAL_DEVICE) {
		if (!client->has_block_wakelock) {
			dev_err(gxp->dev,
				"Must hold BLOCK wakelock to acquire VIRTUAL_DEVICE wakelock\n");
			ret = -EINVAL;
			goto out;

		}

		if (client->vd->state == GXP_VD_UNAVAILABLE) {
			dev_err(gxp->dev,
				"Cannot acquire VIRTUAL_DEVICE wakelock on a broken virtual device\n");
			ret = -ENODEV;
			goto err_acquiring_vd_wl;
		}

		if (!client->has_vd_wakelock) {
			down_write(&gxp->vd_semaphore);
			if (client->vd->state == GXP_VD_OFF)
				ret = gxp_vd_start(client->vd);
			else
				ret = gxp_vd_resume(client->vd);
			up_write(&gxp->vd_semaphore);
		}

		if (ret) {
			dev_err(gxp->dev,
				"Failed to acquire VIRTUAL_DEVICE wakelock for client (ret=%d)\n",
				ret);
			goto err_acquiring_vd_wl;
		}

		client->has_vd_wakelock = true;
	}
	requested_low_clkmux = (ibuf.flags & GXP_POWER_LOW_FREQ_CLKMUX) != 0;

	gxp_pm_update_requested_power_states(
		gxp, client->requested_power_state, client->requested_low_clkmux,
		aur_state_array[ibuf.gxp_power_state], requested_low_clkmux,
		client->requested_memory_power_state,
		aur_memory_state_array[ibuf.memory_power_state]);
	client->requested_power_state = aur_state_array[ibuf.gxp_power_state];
	client->requested_low_clkmux = requested_low_clkmux;
	client->requested_memory_power_state =
		aur_memory_state_array[ibuf.memory_power_state];
out:
	up_write(&client->semaphore);

	return ret;

err_acquiring_vd_wl:
	/*
	 * In a single call, if any wakelock acquisition fails, all of them do.
	 * If the client was acquiring both wakelocks and failed to acquire the
	 * VIRTUAL_DEVICE wakelock after successfully acquiring the BLOCK
	 * wakelock, then release it before returning the error code.
	 */
	if (acquired_block_wakelock) {
		gxp_wakelock_release(gxp);
		client->has_block_wakelock = false;
	}

	up_write(&client->semaphore);

	return ret;
}

static int gxp_release_wake_lock(struct gxp_client *client, __u32 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	u32 wakelock_components;
	int ret = 0;

	if (copy_from_user(&wakelock_components, argp,
			   sizeof(wakelock_components)))
		return -EFAULT;

	down_write(&client->semaphore);

	if (wakelock_components & WAKELOCK_VIRTUAL_DEVICE) {
		if (!client->has_vd_wakelock) {
			dev_err(gxp->dev,
				"Client must hold a VIRTUAL_DEVICE wakelock to release one\n");
			ret = -ENODEV;
			goto out;
		}

		/*
		 * Currently VD state will not be GXP_VD_UNAVAILABLE if
		 * has_vd_wakelock is true. Add this check just in case
		 * GXP_VD_UNAVAILABLE will occur in more scenarios in the
		 * future.
		 */
		if (client->vd->state != GXP_VD_UNAVAILABLE) {
			down_write(&gxp->vd_semaphore);
			gxp_vd_suspend(client->vd);
			up_write(&gxp->vd_semaphore);
		}

		client->has_vd_wakelock = false;
	}

	if (wakelock_components & WAKELOCK_BLOCK) {
		if (client->has_vd_wakelock) {
			dev_err(gxp->dev,
				"Client cannot release BLOCK wakelock while holding a VD wakelock\n");
			ret = -EBUSY;
			goto out;
		}

		if (!client->has_block_wakelock) {
			dev_err(gxp->dev,
				"Client must hold a BLOCK wakelock to release one\n");
			ret = -ENODEV;
			goto out;
		}

		gxp_wakelock_release(gxp);
		/*
		 * Other clients may still be using the BLK_AUR, check if we need
		 * to change the power state.
		 */
		gxp_pm_update_requested_power_states(
			gxp, client->requested_power_state,
			client->requested_low_clkmux, AUR_OFF, false,
			client->requested_memory_power_state,
			AUR_MEM_UNDEFINED);
		client->requested_power_state = AUR_OFF;
		client->requested_memory_power_state = AUR_MEM_UNDEFINED;
		client->requested_low_clkmux = false;
		client->has_block_wakelock = false;
	}

out:
	up_write(&client->semaphore);

	return ret;
}

static int gxp_map_dmabuf(struct gxp_client *client,
			  struct gxp_map_dmabuf_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_map_dmabuf_ioctl ibuf;
	struct gxp_mapping *mapping;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.virtual_core_list == 0)
		return -EINVAL;

	down_read(&client->semaphore);

	if (!check_client_has_available_vd(client, "GXP_MAP_DMABUF")) {
		ret = -ENODEV;
		goto out_unlock;
	}

	mapping = gxp_dmabuf_map(gxp, client->vd, ibuf.virtual_core_list,
				 ibuf.dmabuf_fd,
				 /*gxp_dma_flags=*/0,
				 mapping_flags_to_dma_dir(ibuf.flags));
	if (IS_ERR(mapping)) {
		ret = PTR_ERR(mapping);
		dev_err(gxp->dev, "Failed to map dma-buf (ret=%d)\n", ret);
		goto out_unlock;
	}

	ret = gxp_vd_mapping_store(client->vd, mapping);
	if (ret) {
		dev_err(gxp->dev,
			"Failed to store mapping for dma-buf (ret=%d)\n", ret);
		goto out_put;
	}

	ibuf.device_address = mapping->device_address;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		/* If the IOCTL fails, the dma-buf must be unmapped */
		gxp_vd_mapping_remove(client->vd, mapping);
		ret = -EFAULT;
	}

out_put:
	/*
	 * Release the reference from creating the dmabuf mapping
	 * If the mapping was not successfully stored in the owning virtual
	 * device, this will unmap and cleanup the dmabuf.
	 */
	gxp_mapping_put(mapping);

out_unlock:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_unmap_dmabuf(struct gxp_client *client,
			    struct gxp_map_dmabuf_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_map_dmabuf_ioctl ibuf;
	struct gxp_mapping *mapping;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_read(&client->semaphore);

	if (!client->vd) {
		dev_err(gxp->dev,
			"GXP_UNMAP_DMABUF requires the client allocate a VIRTUAL_DEVICE\n");
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Fetch and remove the internal mapping records.
	 * If host_address is not 0, the provided device_address belongs to a
	 * non-dma-buf mapping.
	 */
	mapping = gxp_vd_mapping_search(client->vd, ibuf.device_address);
	if (IS_ERR_OR_NULL(mapping) || mapping->host_address) {
		dev_warn(gxp->dev, "No dma-buf mapped for given IOVA\n");
		/*
		 * If the device address belongs to a non-dma-buf mapping,
		 * release the reference to it obtained via the search.
		 */
		if (!IS_ERR_OR_NULL(mapping))
			gxp_mapping_put(mapping);
		ret = -EINVAL;
		goto out;
	}

	/* Remove the mapping from its VD, releasing the VD's reference */
	gxp_vd_mapping_remove(client->vd, mapping);

	/* Release the reference from gxp_vd_mapping_search() */
	gxp_mapping_put(mapping);

out:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_register_mailbox_eventfd(
	struct gxp_client *client,
	struct gxp_register_mailbox_eventfd_ioctl __user *argp)
{
	struct gxp_register_mailbox_eventfd_ioctl ibuf;
	struct gxp_eventfd *eventfd;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_write(&client->semaphore);

	if (!check_client_has_available_vd(client, "GXP_REGISTER_MAILBOX_EVENTFD")) {
		ret = -ENODEV;
		goto out;
	}

	if (ibuf.virtual_core_id >= client->vd->num_cores) {
		ret = -EINVAL;
		goto out;
	}

	/* Make sure the provided eventfd is valid */
	eventfd = gxp_eventfd_create(ibuf.eventfd);
	if (IS_ERR(eventfd)) {
		ret = PTR_ERR(eventfd);
		goto out;
	}

	/* Set the new eventfd, replacing any existing one */
	if (client->mb_eventfds[ibuf.virtual_core_id])
		gxp_eventfd_put(client->mb_eventfds[ibuf.virtual_core_id]);

	client->mb_eventfds[ibuf.virtual_core_id] = eventfd;

out:
	up_write(&client->semaphore);

	return ret;
}

static int gxp_unregister_mailbox_eventfd(
	struct gxp_client *client,
	struct gxp_register_mailbox_eventfd_ioctl __user *argp)
{
	struct gxp_register_mailbox_eventfd_ioctl ibuf;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_write(&client->semaphore);

	if (!client->vd) {
		dev_err(client->gxp->dev,
			"GXP_UNREGISTER_MAILBOX_EVENTFD requires the client allocate a VIRTUAL_DEVICE\n");
		ret = -ENODEV;
		goto out;
	}

	if (ibuf.virtual_core_id >= client->vd->num_cores) {
		ret = -EINVAL;
		goto out;
	}

	if (client->mb_eventfds[ibuf.virtual_core_id])
		gxp_eventfd_put(client->mb_eventfds[ibuf.virtual_core_id]);

	client->mb_eventfds[ibuf.virtual_core_id] = NULL;

out:
	up_write(&client->semaphore);

	return ret;
}

static int
gxp_get_interface_version(struct gxp_client *client,
			  struct gxp_interface_version_ioctl __user *argp)
{
	struct gxp_interface_version_ioctl ibuf;
	int ret;

	ibuf.version_major = GXP_INTERFACE_VERSION_MAJOR;
	ibuf.version_minor = GXP_INTERFACE_VERSION_MINOR;
	memset(ibuf.version_build, 0, GXP_INTERFACE_VERSION_BUILD_BUFFER_SIZE);
	ret = snprintf(ibuf.version_build,
		       GXP_INTERFACE_VERSION_BUILD_BUFFER_SIZE - 1,
		       GIT_REPO_TAG);

	if (ret < 0 || ret >= GXP_INTERFACE_VERSION_BUILD_BUFFER_SIZE) {
		dev_warn(
			client->gxp->dev,
			"Buffer size insufficient to hold GIT_REPO_TAG (size=%d)\n",
			ret);
	}

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return 0;
}

static int gxp_trigger_debug_dump(struct gxp_client *client,
				  __u32 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	int phys_core, i;
	u32 core_bits;
	int ret = 0;

	if (!uid_eq(current_euid(), GLOBAL_ROOT_UID))
		return -EPERM;

	if (!gxp_debug_dump_is_enabled()) {
		dev_err(gxp->dev, "Debug dump functionality is disabled\n");
		return -EINVAL;
	}

	if (copy_from_user(&core_bits, argp, sizeof(core_bits)))
		return -EFAULT;

	/* Caller must hold VIRTUAL_DEVICE wakelock */
	down_read(&client->semaphore);

	if (!check_client_has_available_vd_wakelock(client,
						    "GXP_TRIGGER_DEBUG_DUMP")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	down_read(&gxp->vd_semaphore);

	for (i = 0; i < GXP_NUM_CORES; i++) {
		if (!(core_bits & BIT(i)))
			continue;
		phys_core = gxp_vd_virt_core_to_phys_core(client->vd, i);
		if (phys_core < 0) {
			dev_err(gxp->dev,
				"Trigger debug dump failed: Invalid virtual core id (%u)\n",
				i);
			ret = -EINVAL;
			continue;
		}

		if (gxp_is_fw_running(gxp, phys_core)) {
			gxp_notification_send(gxp, phys_core,
					      CORE_NOTIF_GENERATE_DEBUG_DUMP);
		}
	}

	up_read(&gxp->vd_semaphore);
out_unlock_client_semaphore:
	up_read(&client->semaphore);

	return ret;
}

static long gxp_ioctl(struct file *file, uint cmd, ulong arg)
{
	struct gxp_client *client = file->private_data;
	void __user *argp = (void __user *)arg;
	long ret;

	switch (cmd) {
	case GXP_MAP_BUFFER:
		ret = gxp_map_buffer(client, argp);
		break;
	case GXP_UNMAP_BUFFER:
		ret = gxp_unmap_buffer(client, argp);
		break;
	case GXP_SYNC_BUFFER:
		ret = gxp_sync_buffer(client, argp);
		break;
	case GXP_MAILBOX_COMMAND_COMPAT:
		ret = gxp_mailbox_command_compat(client, argp);
		break;
	case GXP_MAILBOX_RESPONSE:
		ret = gxp_mailbox_response(client, argp);
		break;
	case GXP_GET_SPECS:
		ret = gxp_get_specs(client, argp);
		break;
	case GXP_ALLOCATE_VIRTUAL_DEVICE:
		ret = gxp_allocate_vd(client, argp);
		break;
	case GXP_ETM_TRACE_START_COMMAND:
		ret = gxp_etm_trace_start_command(client, argp);
		break;
	case GXP_ETM_TRACE_SW_STOP_COMMAND:
		ret = gxp_etm_trace_sw_stop_command(client, argp);
		break;
	case GXP_ETM_TRACE_CLEANUP_COMMAND:
		ret = gxp_etm_trace_cleanup_command(client, argp);
		break;
	case GXP_ETM_GET_TRACE_INFO_COMMAND:
		ret = gxp_etm_get_trace_info_command(client, argp);
		break;
	case GXP_ENABLE_TELEMETRY:
		ret = gxp_enable_telemetry(client, argp);
		break;
	case GXP_DISABLE_TELEMETRY:
		ret = gxp_disable_telemetry(client, argp);
		break;
	case GXP_MAP_TPU_MBX_QUEUE:
		ret = gxp_map_tpu_mbx_queue(client, argp);
		break;
	case GXP_UNMAP_TPU_MBX_QUEUE:
		ret = gxp_unmap_tpu_mbx_queue(client, argp);
		break;
	case GXP_REGISTER_TELEMETRY_EVENTFD:
		ret = gxp_register_telemetry_eventfd(client, argp);
		break;
	case GXP_UNREGISTER_TELEMETRY_EVENTFD:
		ret = gxp_unregister_telemetry_eventfd(client, argp);
		break;
	case GXP_READ_GLOBAL_COUNTER:
		ret = gxp_read_global_counter(client, argp);
		break;
	case GXP_ACQUIRE_WAKE_LOCK_COMPAT:
		ret = gxp_acquire_wake_lock_compat(client, argp);
		break;
	case GXP_RELEASE_WAKE_LOCK:
		ret = gxp_release_wake_lock(client, argp);
		break;
	case GXP_MAP_DMABUF:
		ret = gxp_map_dmabuf(client, argp);
		break;
	case GXP_UNMAP_DMABUF:
		ret = gxp_unmap_dmabuf(client, argp);
		break;
	case GXP_MAILBOX_COMMAND:
		ret = gxp_mailbox_command(client, argp);
		break;
	case GXP_REGISTER_MAILBOX_EVENTFD:
		ret = gxp_register_mailbox_eventfd(client, argp);
		break;
	case GXP_UNREGISTER_MAILBOX_EVENTFD:
		ret = gxp_unregister_mailbox_eventfd(client, argp);
		break;
	case GXP_ACQUIRE_WAKE_LOCK:
		ret = gxp_acquire_wake_lock(client, argp);
		break;
	case GXP_GET_INTERFACE_VERSION:
		ret = gxp_get_interface_version(client, argp);
		break;
	case GXP_TRIGGER_DEBUG_DUMP:
		ret = gxp_trigger_debug_dump(client, argp);
		break;
	default:
		ret = -ENOTTY; /* unknown command */
	}

	return ret;
}

static int gxp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct gxp_client *client = file->private_data;

	if (!client)
		return -ENODEV;

	switch (vma->vm_pgoff << PAGE_SHIFT) {
	case GXP_MMAP_LOG_BUFFER_OFFSET:
		return gxp_telemetry_mmap_buffers(client->gxp,
						  GXP_TELEMETRY_TYPE_LOGGING,
						  vma);
	case GXP_MMAP_TRACE_BUFFER_OFFSET:
		return gxp_telemetry_mmap_buffers(client->gxp,
						  GXP_TELEMETRY_TYPE_TRACING,
						  vma);
	default:
		return -EINVAL;
	}
}

static const struct file_operations gxp_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.mmap = gxp_mmap,
	.open = gxp_open,
	.release = gxp_release,
	.unlocked_ioctl = gxp_ioctl,
};

static int gxp_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gxp_dev *gxp;
	struct resource *r;
	phys_addr_t offset, base_addr;
	struct device_node *np;
	struct platform_device *tpu_pdev;
	struct platform_device *gsa_pdev;
	int ret;
	int  __maybe_unused i;
	bool __maybe_unused tpu_found;
	u64 prop;

	dev_notice(dev, "Probing gxp driver with commit %s\n", GIT_REPO_TAG);

	gxp = devm_kzalloc(dev, sizeof(*gxp), GFP_KERNEL);
	if (!gxp)
		return -ENOMEM;

	platform_set_drvdata(pdev, gxp);
	gxp->dev = dev;

	gxp->misc_dev.minor = MISC_DYNAMIC_MINOR;
	gxp->misc_dev.name = "gxp";
	gxp->misc_dev.fops = &gxp_fops;

	gxp_wakelock_init(gxp);

	ret = misc_register(&gxp->misc_dev);
	if (ret) {
		dev_err(dev, "Failed to register misc device (ret = %d)\n",
			ret);
		devm_kfree(dev, (void *)gxp);
		return ret;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(r)) {
		dev_err(dev, "Failed to get memory resource\n");
		ret = -ENODEV;
		goto err;
	}

	gxp->regs.paddr = r->start;
	gxp->regs.size = resource_size(r);
	gxp->regs.vaddr = devm_ioremap_resource(dev, r);
	if (IS_ERR_OR_NULL(gxp->regs.vaddr)) {
		dev_err(dev, "Failed to map registers\n");
		ret = -ENODEV;
		goto err;
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cmu");
	if (!IS_ERR_OR_NULL(r)) {
		gxp->cmu.paddr = r->start;
		gxp->cmu.size = resource_size(r);
		gxp->cmu.vaddr = devm_ioremap_resource(dev, r);
	}
	/*
	 * TODO (b/224685748): Remove this block after CMU CSR is supported
	 * in device tree config.
	 */
	if (IS_ERR_OR_NULL(r) || IS_ERR_OR_NULL(gxp->cmu.vaddr)) {
		gxp->cmu.paddr = gxp->regs.paddr - GXP_CMU_OFFSET;
		gxp->cmu.size = GXP_CMU_SIZE;
		gxp->cmu.vaddr = devm_ioremap(dev, gxp->cmu.paddr, gxp->cmu.size);
		if (IS_ERR_OR_NULL(gxp->cmu.vaddr))
			dev_warn(dev, "Failed to map CMU registers\n");
	}

	ret = gxp_pm_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to init power management (ret=%d)\n", ret);
		goto err;
	}

	for (i = 0; i < GXP_NUM_CORES; i++) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, i + 1);
		if (IS_ERR_OR_NULL(r)) {
			dev_err(dev, "Failed to get mailbox%d resource\n", i);
			ret = -ENODEV;
			goto err_pm_destroy;
		}

		gxp->mbx[i].paddr = r->start;
		gxp->mbx[i].size = resource_size(r);
		gxp->mbx[i].vaddr = devm_ioremap_resource(dev, r);
		if (IS_ERR_OR_NULL(gxp->mbx[i].vaddr)) {
			dev_err(dev, "Failed to map mailbox%d registers\n", i);
			ret = -ENODEV;
			goto err_pm_destroy;
		}
	}

	tpu_found = true;
	/* Get TPU device from device tree */
	np = of_parse_phandle(dev->of_node, "tpu-device", 0);
	if (IS_ERR_OR_NULL(np)) {
		dev_warn(dev, "No tpu-device in device tree\n");
		tpu_found = false;
	}
	tpu_pdev = of_find_device_by_node(np);
	if (!tpu_pdev) {
		dev_err(dev, "TPU device not found\n");
		tpu_found = false;
	}
	/* get tpu mailbox register base */
	ret = of_property_read_u64_index(np, "reg", 0, &base_addr);
	of_node_put(np);
	if (ret) {
		dev_warn(dev, "Unable to get tpu-device base address\n");
		tpu_found = false;
	}
	/* get gxp-tpu mailbox register offset */
	ret = of_property_read_u64(dev->of_node, "gxp-tpu-mbx-offset",
				   &offset);
	if (ret) {
		dev_warn(dev, "Unable to get tpu-device mailbox offset\n");
		tpu_found = false;
	}
	if (tpu_found) {
		gxp->tpu_dev.dev = &tpu_pdev->dev;
		get_device(gxp->tpu_dev.dev);
		gxp->tpu_dev.mbx_paddr = base_addr + offset;
	} else {
		dev_warn(dev, "TPU will not be available for interop\n");
		gxp->tpu_dev.mbx_paddr = 0;
	}

	ret = gxp_dma_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to initialize GXP DMA interface\n");
		goto err_put_tpu_dev;
	}

	gxp->mailbox_mgr = gxp_mailbox_create_manager(gxp, GXP_NUM_CORES);
	if (IS_ERR_OR_NULL(gxp->mailbox_mgr)) {
		dev_err(dev, "Failed to create mailbox manager\n");
		ret = -ENOMEM;
		goto err_dma_exit;
	}

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	ret = gxp_debug_dump_init(gxp, &gxp_sscd_dev, &gxp_sscd_pdata);
#else
	ret = gxp_debug_dump_init(gxp, NULL, NULL);
#endif  // !CONFIG_SUBSYSTEM_COREDUMP
	if (ret) {
		dev_err(dev, "Failed to initialize debug dump\n");
		gxp_debug_dump_exit(gxp);
	}

	mutex_init(&gxp->dsp_firmware_lock);
	mutex_init(&gxp->pin_user_pages_lock);

	gxp->domain_pool = kmalloc(sizeof(*gxp->domain_pool), GFP_KERNEL);
	if (!gxp->domain_pool) {
		ret = -ENOMEM;
		goto err_debug_dump_exit;
	}
	ret = gxp_domain_pool_init(gxp, gxp->domain_pool,
				   GXP_NUM_PREALLOCATED_DOMAINS);
	if (ret) {
		dev_err(dev,
			"Failed to initialize IOMMU domain pool (ret=%d)\n",
			ret);
		goto err_free_domain_pool;
	}
	ret = gxp_vd_init(gxp);
	if (ret) {
		dev_err(dev,
			"Failed to initialize virtual device manager (ret=%d)\n",
			ret);
		goto err_domain_pool_destroy;
	}
	gxp_dma_init_default_resources(gxp);

	/* Get GSA device from device tree */
	np = of_parse_phandle(dev->of_node, "gsa-device", 0);
	if (!np) {
		dev_warn(
			dev,
			"No gsa-device in device tree. Firmware authentication not available\n");
	} else {
		gsa_pdev = of_find_device_by_node(np);
		if (!gsa_pdev) {
			dev_err(dev, "GSA device not found\n");
			of_node_put(np);
			ret = -ENODEV;
			goto err_vd_destroy;
		}
		gxp->gsa_dev = get_device(&gsa_pdev->dev);
		of_node_put(np);
		dev_info(
			dev,
			"GSA device found, Firmware authentication available\n");
	}

	ret = of_property_read_u64(dev->of_node, "gxp-memory-per-core",
				   &prop);
	if (ret) {
		dev_err(dev, "Unable to get memory-per-core from device tree\n");
		gxp->memory_per_core = 0;
	} else {
		gxp->memory_per_core = (u32)prop;
	}

	gxp_fw_data_init(gxp);
	gxp_telemetry_init(gxp);
	gxp_create_debugfs(gxp);
	gxp->thermal_mgr = gxp_thermal_init(gxp);
	if (!gxp->thermal_mgr)
		dev_err(dev, "Failed to init thermal driver\n");
	dev_dbg(dev, "Probe finished\n");

	INIT_LIST_HEAD(&gxp->client_list);
	mutex_init(&gxp->client_list_lock);

	gxp_debug_pointer = gxp;

	return 0;
err_vd_destroy:
	gxp_vd_destroy(gxp);
err_domain_pool_destroy:
	gxp_domain_pool_destroy(gxp->domain_pool);
err_free_domain_pool:
	kfree(gxp->domain_pool);
err_debug_dump_exit:
	gxp_debug_dump_exit(gxp);
err_dma_exit:
	gxp_dma_exit(gxp);
err_put_tpu_dev:
	put_device(gxp->tpu_dev.dev);
err_pm_destroy:
	gxp_pm_destroy(gxp);
err:
	misc_deregister(&gxp->misc_dev);
	devm_kfree(dev, (void *)gxp);
	return ret;
}

static int gxp_platform_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gxp_dev *gxp = platform_get_drvdata(pdev);

	gxp_remove_debugfs(gxp);
	gxp_fw_data_destroy(gxp);
	if (gxp->gsa_dev)
		put_device(gxp->gsa_dev);
	gxp_vd_destroy(gxp);
	gxp_domain_pool_destroy(gxp->domain_pool);
	kfree(gxp->domain_pool);
	gxp_debug_dump_exit(gxp);
	gxp_dma_exit(gxp);
	put_device(gxp->tpu_dev.dev);
	gxp_pm_destroy(gxp);
	misc_deregister(&gxp->misc_dev);

	devm_kfree(dev, (void *)gxp);

	gxp_debug_pointer = NULL;

	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)

static int gxp_platform_suspend(struct device *dev)
{
	struct gxp_dev *gxp = dev_get_drvdata(dev);

	return gxp_wakelock_suspend(gxp);
}

static int gxp_platform_resume(struct device *dev)
{
	struct gxp_dev *gxp = dev_get_drvdata(dev);

	return gxp_wakelock_resume(gxp);
}

static const struct dev_pm_ops gxp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gxp_platform_suspend, gxp_platform_resume)
};

#endif /* IS_ENABLED(CONFIG_PM_SLEEP) */

#ifdef CONFIG_OF
static const struct of_device_id gxp_of_match[] = {
	{ .compatible = "google,gxp", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, gxp_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id gxp_acpi_match[] = {
	{ "CXRP0001", 0 },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(acpi, gxp_acpi_match);
#endif

static struct platform_driver gxp_platform_driver = {
	.probe = gxp_platform_probe,
	.remove = gxp_platform_remove,
	.driver = {
			.name = GXP_DRIVER_NAME,
			.of_match_table = of_match_ptr(gxp_of_match),
			.acpi_match_table = ACPI_PTR(gxp_acpi_match),
#if IS_ENABLED(CONFIG_PM_SLEEP)
			.pm = &gxp_pm_ops,
#endif
		},
};

static int __init gxp_platform_init(void)
{
#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	/* Registers SSCD platform device */
	if (gxp_debug_dump_is_enabled()) {
		if (platform_device_register(&gxp_sscd_dev))
			pr_err("Unable to register SSCD platform device\n");
	}
#endif
	return platform_driver_register(&gxp_platform_driver);
}

static void __exit gxp_platform_exit(void)
{
	platform_driver_unregister(&gxp_platform_driver);
#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	if (gxp_debug_dump_is_enabled())
		platform_device_unregister(&gxp_sscd_dev);
#endif
}

MODULE_DESCRIPTION("Google GXP platform driver");
MODULE_LICENSE("GPL v2");
MODULE_INFO(gitinfo, GIT_REPO_TAG);
module_init(gxp_platform_init);
module_exit(gxp_platform_exit);
