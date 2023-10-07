// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP platform driver utilities.
 *
 * Copyright (C) 2022 Google LLC
 */

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
#include <linux/platform_data/sscoredump.h>
#endif

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>

#include <gcip/gcip-dma-fence.h>
#include <gcip/gcip-pm.h>

#include "gxp-client.h"
#include "gxp-config.h"
#include "gxp-core-telemetry.h"
#include "gxp-debug-dump.h"
#include "gxp-debugfs.h"
#include "gxp-dma-fence.h"
#include "gxp-dma.h"
#include "gxp-dmabuf.h"
#include "gxp-domain-pool.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-firmware-loader.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-mailbox.h"
#include "gxp-mailbox-driver.h"
#include "gxp-mapping.h"
#include "gxp-notification.h"
#include "gxp-pm.h"
#include "gxp-thermal.h"
#include "gxp-vd.h"
#include "gxp.h"

#if HAS_TPU_EXT
#include <soc/google/tpu-ext.h>
#endif

#if GXP_USE_LEGACY_MAILBOX
#include "gxp-mailbox-impl.h"
#else
#include "gxp-dci.h"
#endif

static struct gxp_dev *gxp_debug_pointer;

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

static void gxp_common_platform_reg_sscd(void)
{
	/* Registers SSCD platform device */
	if (gxp_debug_dump_is_enabled()) {
		if (platform_device_register(&gxp_sscd_dev))
			pr_err("Unable to register SSCD platform device\n");
	}
}

static void gxp_common_platform_unreg_sscd(void)
{
	if (gxp_debug_dump_is_enabled())
		platform_device_unregister(&gxp_sscd_dev);
}

#else /* CONFIG_SUBSYSTEM_COREDUMP */

static void gxp_common_platform_reg_sscd(void)
{
}

static void gxp_common_platform_unreg_sscd(void)
{
}

#endif /* CONFIG_SUBSYSTEM_COREDUMP */

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

	/* If this is the first call to open(), load the firmware files */
	ret = gxp_firmware_loader_load_if_needed(gxp);
	if (ret)
		return ret;

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

	if (client->enabled_core_telemetry_logging)
		gxp_core_telemetry_disable(client->gxp,
					   GXP_TELEMETRY_TYPE_LOGGING);
	if (client->enabled_core_telemetry_tracing)
		gxp_core_telemetry_disable(client->gxp,
					   GXP_TELEMETRY_TYPE_TRACING);

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

	if (ibuf.size == 0)
		return -EINVAL;

	if (ibuf.host_address % L1_CACHE_BYTES || ibuf.size % L1_CACHE_BYTES) {
		dev_err(gxp->dev,
			"Mapped buffers must be cache line aligned and padded.\n");
		return -EINVAL;
	}

	down_read(&client->semaphore);

	if (!gxp_client_has_available_vd(client, "GXP_MAP_BUFFER")) {
		ret = -ENODEV;
		goto out;
	}

	map = gxp_mapping_create(gxp, client->vd->domain, ibuf.host_address,
				 ibuf.size,
				 ibuf.flags,
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

	gxp_mapping_iova_log(client, map,
			     GXP_IOVA_LOG_MAP | GXP_IOVA_LOG_BUFFER);

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

static int gxp_unmap_buffer(struct gxp_client *client, struct gxp_map_ioctl __user *argp)
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

	down_write(&client->vd->mappings_semaphore);

	map = gxp_vd_mapping_search_locked(client->vd, (dma_addr_t)ibuf.device_address);
	if (!map) {
		dev_err(gxp->dev, "Mapping not found for provided device address %#llX\n",
			ibuf.device_address);
		ret = -EINVAL;
	} else if (!map->host_address) {
		dev_err(gxp->dev, "dma-bufs must be unmapped via GXP_UNMAP_DMABUF\n");
		ret = -EINVAL;
	}

	if (ret) {
		up_write(&client->vd->mappings_semaphore);
		goto out_put;
	}

	gxp_vd_mapping_remove_locked(client->vd, map);
	up_write(&client->vd->mappings_semaphore);

	gxp_mapping_iova_log(client, map, GXP_IOVA_LOG_UNMAP | GXP_IOVA_LOG_BUFFER);

out_put:
	/* Release the reference from gxp_vd_mapping_search() */
	if (map)
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

static int gxp_mailbox_command(struct gxp_client *client,
			       struct gxp_mailbox_command_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_mailbox_command_ioctl ibuf;
	int virt_core, phys_core;
	int ret = 0;
	struct gxp_power_states power_states;

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

	if (ibuf.power_flags & GXP_POWER_NON_AGGRESSOR)
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

	if (gxp->mailbox_mgr->mailboxes[phys_core] == NULL) {
		dev_err(gxp->dev, "Mailbox not initialized for core %d\n",
			phys_core);
		ret = -EIO;
		goto out;
	}

	power_states.power = aur_state_array[ibuf.gxp_power_state];
	power_states.memory = aur_memory_state_array[ibuf.memory_power_state];
	power_states.low_clkmux = (ibuf.power_flags & GXP_POWER_LOW_FREQ_CLKMUX) != 0;

	ret = gxp->mailbox_mgr->execute_cmd_async(
		client, gxp->mailbox_mgr->mailboxes[phys_core], virt_core,
		GXP_MBOX_CODE_DISPATCH, 0, ibuf.device_address, ibuf.size,
		ibuf.flags, power_states, &ibuf.sequence_number);
	if (ret) {
		dev_err(gxp->dev, "Failed to enqueue mailbox command (ret=%d)\n",
			ret);
		goto out;
	}

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
	int virt_core;
	int ret = 0;

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

	ret = gxp->mailbox_mgr->wait_async_resp(client, virt_core,
						&ibuf.sequence_number, NULL,
						&ibuf.cmd_retval,
						&ibuf.error_code);
	if (ret)
		goto out;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		ret = -EFAULT;

out:
	up_read(&client->semaphore);

	return ret;
}

static int gxp_get_specs(struct gxp_client *client,
			 struct gxp_specs_ioctl __user *argp)
{
	struct buffer_data *logging_buff_data;
	struct gxp_dev *gxp = client->gxp;
	struct gxp_specs_ioctl ibuf = {
		.core_count = GXP_NUM_CORES,
		.features = !gxp_is_direct_mode(client->gxp),
		.telemetry_buffer_size = 0,
		.secure_telemetry_buffer_size =
			(u8)(SECURE_CORE_TELEMETRY_BUFFER_SIZE /
			     GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE),
		.memory_per_core = client->gxp->memory_per_core,
	};

	if (!IS_ERR_OR_NULL(gxp->core_telemetry_mgr)) {
		logging_buff_data = gxp->core_telemetry_mgr->logging_buff_data;
		if (!IS_ERR_OR_NULL(logging_buff_data)) {
			ibuf.telemetry_buffer_size =
				(u8)(logging_buff_data->size /
				     GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE);
		}
	}

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return 0;
}

static int gxp_allocate_vd(struct gxp_client *client,
			   struct gxp_virtual_device_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_virtual_device_ioctl ibuf;
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
	ret = gxp_client_allocate_virtual_device(client, ibuf.core_count,
						 ibuf.flags);
	up_write(&client->semaphore);
	if (ret)
		return ret;

	ibuf.vdid = client->vd->vdid;
	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		/*
		 * VD will be released once the client FD has been closed, we
		 * don't need to release VD here as this branch should never
		 * happen in usual cases.
		 */
		return -EFAULT;
	}

	return 0;
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

static int gxp_enable_core_telemetry(struct gxp_client *client,
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

	ret = gxp_core_telemetry_enable(gxp, type);

	/*
	 * Record what core telemetry types this client enabled so they can be
	 * cleaned-up if the client closes without disabling them.
	 */
	if (!ret && type == GXP_TELEMETRY_TYPE_LOGGING)
		client->enabled_core_telemetry_logging = true;
	if (!ret && type == GXP_TELEMETRY_TYPE_TRACING)
		client->enabled_core_telemetry_tracing = true;

	return ret;
}

static int gxp_disable_core_telemetry(struct gxp_client *client,
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

	ret = gxp_core_telemetry_disable(gxp, type);

	if (!ret && type == GXP_TELEMETRY_TYPE_LOGGING)
		client->enabled_core_telemetry_logging = false;
	if (!ret && type == GXP_TELEMETRY_TYPE_TRACING)
		client->enabled_core_telemetry_tracing = false;

	return ret;
}

#if HAS_TPU_EXT

/*
 * Map TPU mailboxes to IOVA.
 * This function will be called only when the device is in the direct mode.
 */
static int map_tpu_mbx_queue(struct gxp_client *client,
			     struct gxp_tpu_mbx_queue_ioctl *ibuf)
{
	struct gxp_dev *gxp = client->gxp;
	struct edgetpu_ext_mailbox_info *mbx_info;
	struct edgetpu_ext_client_info gxp_tpu_info;
	u32 phys_core_list = 0;
	u32 core_count;
	int ret = 0;

	down_read(&gxp->vd_semaphore);

	phys_core_list = client->vd->core_list;
	core_count = hweight_long(phys_core_list);

	mbx_info = kmalloc(
		sizeof(struct edgetpu_ext_mailbox_info) +
			core_count *
				sizeof(struct edgetpu_ext_mailbox_descriptor),
		GFP_KERNEL);
	if (!mbx_info) {
		ret = -ENOMEM;
		goto out;
	}

	/*
	 * TODO(b/249440369): Pass @client->tpu_file file pointer. For the backward compatibility,
	 * keep sending @ibuf->tpu_fd here.
	 */
	gxp_tpu_info.tpu_fd = ibuf->tpu_fd;
	gxp_tpu_info.mbox_map = phys_core_list;
	gxp_tpu_info.attr =
		(struct edgetpu_mailbox_attr __user *)ibuf->attr_ptr;
	ret = edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
				     EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
				     ALLOCATE_EXTERNAL_MAILBOX, &gxp_tpu_info,
				     mbx_info);
	if (ret) {
		dev_err(gxp->dev, "Failed to allocate ext TPU mailboxes %d",
			ret);
		goto out_free;
	}

	/* Align queue size to page size for iommu map. */
	mbx_info->cmdq_size = ALIGN(mbx_info->cmdq_size, PAGE_SIZE);
	mbx_info->respq_size = ALIGN(mbx_info->respq_size, PAGE_SIZE);

	ret = gxp_dma_map_tpu_buffer(gxp, client->vd->domain, phys_core_list,
				     mbx_info);
	if (ret) {
		dev_err(gxp->dev, "Failed to map TPU mailbox buffer %d", ret);
		goto err_free_tpu_mbx;
	}
	client->mbx_desc.phys_core_list = phys_core_list;
	client->mbx_desc.cmdq_size = mbx_info->cmdq_size;
	client->mbx_desc.respq_size = mbx_info->respq_size;

	goto out_free;

err_free_tpu_mbx:
	edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
			       EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
			       FREE_EXTERNAL_MAILBOX, &gxp_tpu_info, NULL);
out_free:
	kfree(mbx_info);
out:
	up_read(&gxp->vd_semaphore);

	return ret;
}

/*
 * Unmap TPU mailboxes from IOVA.
 * This function will be called only when the device is in the direct mode.
 */
static void unmap_tpu_mbx_queue(struct gxp_client *client,
				struct gxp_tpu_mbx_queue_ioctl *ibuf)
{
	struct gxp_dev *gxp = client->gxp;
	struct edgetpu_ext_client_info gxp_tpu_info;

	gxp_dma_unmap_tpu_buffer(gxp, client->vd->domain, client->mbx_desc);
	gxp_tpu_info.tpu_fd = ibuf->tpu_fd;
	edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
			       EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
			       FREE_EXTERNAL_MAILBOX, &gxp_tpu_info, NULL);
}

static int gxp_map_tpu_mbx_queue(struct gxp_client *client,
				 struct gxp_tpu_mbx_queue_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_tpu_mbx_queue_ioctl ibuf;
	int ret = 0;

	if (!gxp->tpu_dev.mbx_paddr) {
		dev_err(gxp->dev, "TPU is not available for interop\n");
		return -EINVAL;
	}

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_write(&client->semaphore);

	if (!gxp_client_has_available_vd(client, "GXP_MAP_TPU_MBX_QUEUE")) {
		ret = -ENODEV;
		goto out_unlock_client_semaphore;
	}

	if (client->tpu_file) {
		dev_err(gxp->dev, "Mapping/linking TPU mailbox information already exists");
		ret = -EBUSY;
		goto out_unlock_client_semaphore;
	}

	/*
	 * If someone is attacking us through this interface -
	 * it's possible that ibuf.tpu_fd here is already a different file from the one passed to
	 * edgetpu_ext_driver_cmd() (if the runtime closes the FD and opens another file exactly
	 * between the TPU driver call above and the fget below).
	 *
	 * However, from Zuma, we pass the file pointer directly to the TPU KD and it will check
	 * whether that file is true TPU device file or not. Therefore, our code is safe from the
	 * fd swapping attack.
	 */
	client->tpu_file = fget(ibuf.tpu_fd);
	if (!client->tpu_file) {
		ret = -EINVAL;
		goto out_unlock_client_semaphore;
	}

	if (gxp_is_direct_mode(gxp)) {
		ret = map_tpu_mbx_queue(client, &ibuf);
		if (ret)
			goto err_fput_tpu_file;
	}

	if (gxp->after_map_tpu_mbx_queue) {
		ret = gxp->after_map_tpu_mbx_queue(gxp, client);
		if (ret)
			goto err_unmap_tpu_mbx_queue;
	}

	goto out_unlock_client_semaphore;

err_unmap_tpu_mbx_queue:
	if (gxp_is_direct_mode(gxp))
		unmap_tpu_mbx_queue(client, &ibuf);
err_fput_tpu_file:
	fput(client->tpu_file);
	client->tpu_file = NULL;
out_unlock_client_semaphore:
	up_write(&client->semaphore);

	return ret;
}

static int gxp_unmap_tpu_mbx_queue(struct gxp_client *client,
				   struct gxp_tpu_mbx_queue_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_tpu_mbx_queue_ioctl ibuf;
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

	if (gxp->before_unmap_tpu_mbx_queue)
		gxp->before_unmap_tpu_mbx_queue(gxp, client);

	if (gxp_is_direct_mode(gxp))
		unmap_tpu_mbx_queue(client, &ibuf);

	fput(client->tpu_file);
	client->tpu_file = NULL;

out:
	up_write(&client->semaphore);

	return ret;
}

#else /* HAS_TPU_EXT */

#define gxp_map_tpu_mbx_queue(...) (-ENODEV)
#define gxp_unmap_tpu_mbx_queue(...) (-ENODEV)

#endif /* HAS_TPU_EXT */

static int gxp_register_core_telemetry_eventfd(
	struct gxp_client *client,
	struct gxp_register_telemetry_eventfd_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_register_telemetry_eventfd_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	return gxp_core_telemetry_register_eventfd(gxp, ibuf.type,
						   ibuf.eventfd);
}

static int gxp_unregister_core_telemetry_eventfd(
	struct gxp_client *client,
	struct gxp_register_telemetry_eventfd_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_register_telemetry_eventfd_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	return gxp_core_telemetry_unregister_eventfd(gxp, ibuf.type);
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

static int gxp_acquire_wake_lock(struct gxp_client *client,
				 struct gxp_acquire_wakelock_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_acquire_wakelock_ioctl ibuf;
	bool acquired_block_wakelock = false;
	bool requested_low_clkmux = false;
	struct gxp_power_states power_states;
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

	requested_low_clkmux = (ibuf.flags & GXP_POWER_LOW_FREQ_CLKMUX) != 0;

	/* Acquire a BLOCK wakelock if requested */
	if (ibuf.components_to_wake & WAKELOCK_BLOCK) {
		ret = gxp_client_acquire_block_wakelock(
			client, &acquired_block_wakelock);
		if (ret) {
			dev_err(gxp->dev,
				"Failed to acquire BLOCK wakelock for client (ret=%d)\n",
				ret);
			goto out;
		}
	}

	/* Acquire a VIRTUAL_DEVICE wakelock if requested */
	if (ibuf.components_to_wake & WAKELOCK_VIRTUAL_DEVICE) {
		power_states.power = aur_state_array[ibuf.gxp_power_state];
		power_states.memory = aur_memory_state_array[ibuf.memory_power_state];
		power_states.low_clkmux = requested_low_clkmux;
		ret = gxp_client_acquire_vd_wakelock(client, power_states);
		if (ret) {
			dev_err(gxp->dev,
				"Failed to acquire VIRTUAL_DEVICE wakelock for client (ret=%d)\n",
				ret);
			goto err_acquiring_vd_wl;
		}
	}

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
	if (acquired_block_wakelock)
		gxp_client_release_block_wakelock(client);

	up_write(&client->semaphore);

	return ret;
}

static int gxp_release_wake_lock(struct gxp_client *client, __u32 __user *argp)
{
	u32 wakelock_components;
	int ret = 0;

	if (copy_from_user(&wakelock_components, argp,
			   sizeof(wakelock_components)))
		return -EFAULT;

	down_write(&client->semaphore);

	if (wakelock_components & WAKELOCK_VIRTUAL_DEVICE)
		gxp_client_release_vd_wakelock(client);

	if (wakelock_components & WAKELOCK_BLOCK)
		gxp_client_release_block_wakelock(client);

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

	down_read(&client->semaphore);

	if (!gxp_client_has_available_vd(client, "GXP_MAP_DMABUF")) {
		ret = -ENODEV;
		goto out_unlock;
	}

	mapping = gxp_dmabuf_map(gxp, client->vd->domain, ibuf.dmabuf_fd,
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

	gxp_mapping_iova_log(client, mapping,
			     GXP_IOVA_LOG_MAP | GXP_IOVA_LOG_DMABUF);

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

static int gxp_unmap_dmabuf(struct gxp_client *client, struct gxp_map_dmabuf_ioctl __user *argp)
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

	down_write(&client->vd->mappings_semaphore);

	/*
	 * Fetch and remove the internal mapping records.
	 * If host_address is not 0, the provided device_address belongs to a
	 * non-dma-buf mapping.
	 */
	mapping = gxp_vd_mapping_search_locked(client->vd, ibuf.device_address);
	if (IS_ERR_OR_NULL(mapping) || mapping->host_address) {
		dev_warn(gxp->dev, "No dma-buf mapped for given IOVA\n");
		up_write(&client->vd->mappings_semaphore);
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
	gxp_vd_mapping_remove_locked(client->vd, mapping);
	up_write(&client->vd->mappings_semaphore);

	gxp_mapping_iova_log(client, mapping, GXP_IOVA_LOG_UNMAP | GXP_IOVA_LOG_DMABUF);

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

	if (!gxp_client_has_available_vd(client, "GXP_REGISTER_MAILBOX_EVENTFD")) {
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

static int
gxp_create_sync_fence(struct gxp_client *client,
		      struct gxp_create_sync_fence_data __user *datap)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_create_sync_fence_data data;
	int ret;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	down_read(&client->semaphore);
	if (client->vd) {
		ret = gxp_dma_fence_create(gxp, client->vd, &data);
	} else {
		dev_warn(gxp->dev, "client creating sync fence has no VD");
		ret = -EINVAL;
	}
	up_read(&client->semaphore);
	if (ret)
		return ret;

	if (copy_to_user((void __user *)datap, &data, sizeof(data)))
		ret = -EFAULT;
	return ret;
}

static int
gxp_signal_sync_fence(struct gxp_signal_sync_fence_data __user *datap)
{
	struct gxp_signal_sync_fence_data data;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	return gcip_dma_fence_signal(data.fence, data.error, false);
}

static int gxp_sync_fence_status(struct gxp_sync_fence_status __user *datap)
{
	struct gxp_sync_fence_status data;
	int ret;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	ret = gcip_dma_fence_status(data.fence, &data.status);
	if (ret)
		return ret;
	if (copy_to_user((void __user *)datap, &data, sizeof(data)))
		ret = -EFAULT;
	return ret;
}

static int gxp_register_invalidated_eventfd(
	struct gxp_client *client,
	struct gxp_register_invalidated_eventfd_ioctl __user *argp)
{
	struct gxp_register_invalidated_eventfd_ioctl ibuf;
	struct gxp_eventfd *eventfd;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	down_write(&client->semaphore);

	if (!gxp_client_has_available_vd(client,
					 "GXP_REGISTER_INVALIDATED_EVENTFD")) {
		ret = -ENODEV;
		goto out;
	}

	eventfd = gxp_eventfd_create(ibuf.eventfd);
	if (IS_ERR(eventfd)) {
		ret = PTR_ERR(eventfd);
		goto out;
	}

	if (client->vd->invalidate_eventfd)
		gxp_eventfd_put(client->vd->invalidate_eventfd);
	client->vd->invalidate_eventfd = eventfd;
out:
	up_write(&client->semaphore);
	return ret;
}

static int gxp_unregister_invalidated_eventfd(
	struct gxp_client *client,
	struct gxp_register_invalidated_eventfd_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	int ret = 0;

	down_write(&client->semaphore);

	if (!client->vd) {
		dev_err(gxp->dev,
			"GXP_UNREGISTER_INVALIDATED_EVENTFD requires the client allocate a VIRTUAL_DEVICE\n");
		ret = -ENODEV;
		goto out;
	}

	if (client->vd->invalidate_eventfd)
		gxp_eventfd_put(client->vd->invalidate_eventfd);
	client->vd->invalidate_eventfd = NULL;
out:
	up_write(&client->semaphore);
	return ret;
}

static long gxp_ioctl(struct file *file, uint cmd, ulong arg)
{
	struct gxp_client *client = file->private_data;
	void __user *argp = (void __user *)arg;
	long ret;

	if (client->gxp->handle_ioctl) {
		ret = client->gxp->handle_ioctl(file, cmd, arg);
		if (ret != -ENOTTY)
			return ret;
	}

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
	case GXP_ENABLE_CORE_TELEMETRY:
		ret = gxp_enable_core_telemetry(client, argp);
		break;
	case GXP_DISABLE_CORE_TELEMETRY:
		ret = gxp_disable_core_telemetry(client, argp);
		break;
	case GXP_MAP_TPU_MBX_QUEUE:
		ret = gxp_map_tpu_mbx_queue(client, argp);
		break;
	case GXP_UNMAP_TPU_MBX_QUEUE:
		ret = gxp_unmap_tpu_mbx_queue(client, argp);
		break;
	case GXP_REGISTER_CORE_TELEMETRY_EVENTFD:
		ret = gxp_register_core_telemetry_eventfd(client, argp);
		break;
	case GXP_UNREGISTER_CORE_TELEMETRY_EVENTFD:
		ret = gxp_unregister_core_telemetry_eventfd(client, argp);
		break;
	case GXP_READ_GLOBAL_COUNTER:
		ret = gxp_read_global_counter(client, argp);
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
	case GXP_CREATE_SYNC_FENCE:
		ret = gxp_create_sync_fence(client, argp);
		break;
	case GXP_SIGNAL_SYNC_FENCE:
		ret = gxp_signal_sync_fence(argp);
		break;
	case GXP_SYNC_FENCE_STATUS:
		ret = gxp_sync_fence_status(argp);
		break;
	case GXP_REGISTER_INVALIDATED_EVENTFD:
		ret = gxp_register_invalidated_eventfd(client, argp);
		break;
	case GXP_UNREGISTER_INVALIDATED_EVENTFD:
		ret = gxp_unregister_invalidated_eventfd(client, argp);
		break;
	default:
		ret = -ENOTTY; /* unknown command */
	}

	return ret;
}

static int gxp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct gxp_client *client = file->private_data;
	int ret;

	if (!client)
		return -ENODEV;

	if (client->gxp->handle_mmap) {
		ret = client->gxp->handle_mmap(file, vma);
		if (ret != -EOPNOTSUPP)
			return ret;
	}

	switch (vma->vm_pgoff << PAGE_SHIFT) {
	case GXP_MMAP_CORE_LOG_BUFFER_OFFSET:
		return gxp_core_telemetry_mmap_buffers(
			client->gxp, GXP_TELEMETRY_TYPE_LOGGING, vma);
	case GXP_MMAP_CORE_TRACE_BUFFER_OFFSET:
		return gxp_core_telemetry_mmap_buffers(
			client->gxp, GXP_TELEMETRY_TYPE_TRACING, vma);
	case GXP_MMAP_CORE_LOG_BUFFER_OFFSET_LEGACY:
		return gxp_core_telemetry_mmap_buffers_legacy(
			client->gxp, GXP_TELEMETRY_TYPE_LOGGING, vma);
	case GXP_MMAP_CORE_TRACE_BUFFER_OFFSET_LEGACY:
		return gxp_core_telemetry_mmap_buffers_legacy(
			client->gxp, GXP_TELEMETRY_TYPE_TRACING, vma);
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

static int gxp_set_reg_resources(struct platform_device *pdev, struct gxp_dev *gxp)
{
	struct device *dev = gxp->dev;
	struct resource *r;
	int i;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(r)) {
		dev_err(dev, "Failed to get memory resource\n");
		return -ENODEV;
	}

	gxp->regs.paddr = r->start;
	gxp->regs.size = resource_size(r);
	gxp->regs.vaddr = devm_ioremap_resource(dev, r);
	if (IS_ERR_OR_NULL(gxp->regs.vaddr)) {
		dev_err(dev, "Failed to map registers\n");
		return -ENODEV;
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cmu");
	if (!IS_ERR_OR_NULL(r)) {
		gxp->cmu.paddr = r->start;
		gxp->cmu.size = resource_size(r);
		gxp->cmu.vaddr = devm_ioremap_resource(dev, r);
		if (IS_ERR_OR_NULL(gxp->cmu.vaddr))
			dev_warn(dev, "Failed to map CMU registers\n");
	}
	/*
	 * TODO (b/224685748): Remove this block after CMU CSR is supported
	 * in device tree config.
	 */
#ifdef GXP_CMU_OFFSET
	if (IS_ERR_OR_NULL(r) || IS_ERR_OR_NULL(gxp->cmu.vaddr)) {
		gxp->cmu.paddr = gxp->regs.paddr - GXP_CMU_OFFSET;
		gxp->cmu.size = GXP_CMU_SIZE;
		gxp->cmu.vaddr =
			devm_ioremap(dev, gxp->cmu.paddr, gxp->cmu.size);
		if (IS_ERR_OR_NULL(gxp->cmu.vaddr))
			dev_warn(dev, "Failed to map CMU registers\n");
	}
#endif

#ifdef GXP_SEPARATE_LPM_OFFSET
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lpm");
	if (IS_ERR_OR_NULL(r)) {
		dev_err(dev, "Failed to get LPM resource\n");
		return -ENODEV;
	}
	gxp->lpm_regs.paddr = r->start;
	gxp->lpm_regs.size = resource_size(r);
	gxp->lpm_regs.vaddr = devm_ioremap_resource(dev, r);
	if (IS_ERR_OR_NULL(gxp->lpm_regs.vaddr)) {
		dev_err(dev, "Failed to map LPM registers\n");
		return -ENODEV;
	}
#else
	gxp->lpm_regs.vaddr = gxp->regs.vaddr;
	gxp->lpm_regs.size = gxp->regs.size;
	gxp->lpm_regs.paddr = gxp->regs.paddr;
#endif

	for (i = 0; i < GXP_NUM_MAILBOXES; i++) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, i + 1);
		if (IS_ERR_OR_NULL(r)) {
			dev_err(dev, "Failed to get mailbox%d resource", i);
			return -ENODEV;
		}

		gxp->mbx[i].paddr = r->start;
		gxp->mbx[i].size = resource_size(r);
		gxp->mbx[i].vaddr = devm_ioremap_resource(dev, r);
		if (IS_ERR_OR_NULL(gxp->mbx[i].vaddr)) {
			dev_err(dev, "Failed to map mailbox%d's register", i);
			return -ENODEV;
		}
	}

	return 0;
}

/*
 * Get TPU device from the device tree. Warnings are shown when any expected
 * device tree entry is missing.
 */
static void gxp_get_tpu_dev(struct gxp_dev *gxp)
{
	struct device *dev = gxp->dev;
	struct platform_device *tpu_pdev;
	struct device_node *np;
	phys_addr_t offset, base_addr;
	int ret;

	/* Get TPU device from device tree */
	np = of_parse_phandle(dev->of_node, "tpu-device", 0);
	if (IS_ERR_OR_NULL(np)) {
		dev_warn(dev, "No tpu-device in device tree\n");
		goto out_not_found;
	}
	tpu_pdev = of_find_device_by_node(np);
	if (!tpu_pdev) {
		dev_err(dev, "TPU device not found\n");
		goto out_not_found;
	}
	/* get tpu mailbox register base */
	ret = of_property_read_u64_index(np, "reg", 0, &base_addr);
	of_node_put(np);
	if (ret) {
		dev_warn(dev, "Unable to get tpu-device base address\n");
		goto out_not_found;
	}
	/* get gxp-tpu mailbox register offset */
	ret = of_property_read_u64(dev->of_node, "gxp-tpu-mbx-offset", &offset);
	if (ret) {
		dev_warn(dev, "Unable to get tpu-device mailbox offset\n");
		goto out_not_found;
	}
	gxp->tpu_dev.dev = get_device(&tpu_pdev->dev);
	gxp->tpu_dev.mbx_paddr = base_addr + offset;
	return;

out_not_found:
	dev_warn(dev, "TPU will not be available for interop\n");
	gxp->tpu_dev.dev = NULL;
	gxp->tpu_dev.mbx_paddr = 0;
}

static void gxp_put_tpu_dev(struct gxp_dev *gxp)
{
	/* put_device is no-op on !dev */
	put_device(gxp->tpu_dev.dev);
}

/* Get GSA device from device tree. */
static void gxp_get_gsa_dev(struct gxp_dev *gxp)
{
	struct device *dev = gxp->dev;
	struct device_node *np;
	struct platform_device *gsa_pdev;

	gxp->gsa_dev = NULL;
	np = of_parse_phandle(dev->of_node, "gsa-device", 0);
	if (!np) {
		dev_warn(
			dev,
			"No gsa-device in device tree. Firmware authentication not available\n");
		return;
	}
	gsa_pdev = of_find_device_by_node(np);
	if (!gsa_pdev) {
		dev_err(dev, "GSA device not found\n");
		of_node_put(np);
		return;
	}
	gxp->gsa_dev = get_device(&gsa_pdev->dev);
	of_node_put(np);
	dev_info(dev, "GSA device found, Firmware authentication available\n");
}

static void gxp_put_gsa_dev(struct gxp_dev *gxp)
{
	put_device(gxp->gsa_dev);
}

static int gxp_common_platform_probe(struct platform_device *pdev, struct gxp_dev *gxp)
{
	struct device *dev = &pdev->dev;
	int ret;
	u64 prop;

	dev_notice(dev, "Probing gxp driver with commit %s\n", GIT_REPO_TAG);

	platform_set_drvdata(pdev, gxp);
	gxp->dev = dev;
	if (gxp->parse_dt) {
		ret = gxp->parse_dt(pdev, gxp);
		if (ret)
			return ret;
	}

	ret = gxp_set_reg_resources(pdev, gxp);
	if (ret)
		return ret;

	gxp_create_debugdir(gxp);

	ret = gxp_pm_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to init power management (ret=%d)\n", ret);
		goto err_remove_debugdir;
	}

	gxp_get_gsa_dev(gxp);
	gxp_get_tpu_dev(gxp);

	ret = gxp_dma_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to initialize GXP DMA interface\n");
		goto err_put_tpu_dev;
	}

	gxp->mailbox_mgr = gxp_mailbox_create_manager(gxp, GXP_NUM_MAILBOXES);
	if (IS_ERR(gxp->mailbox_mgr)) {
		ret = PTR_ERR(gxp->mailbox_mgr);
		dev_err(dev, "Failed to create mailbox manager: %d\n", ret);
		goto err_dma_exit;
	}
	if (gxp_is_direct_mode(gxp)) {
#if GXP_USE_LEGACY_MAILBOX
		gxp_mailbox_init(gxp->mailbox_mgr);
#else
		gxp_dci_init(gxp->mailbox_mgr);
#endif
	}

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	ret = gxp_debug_dump_init(gxp, &gxp_sscd_dev, &gxp_sscd_pdata);
#else
	ret = gxp_debug_dump_init(gxp, NULL, NULL);
#endif  // !CONFIG_SUBSYSTEM_COREDUMP
	if (ret)
		dev_warn(dev, "Failed to initialize debug dump\n");

	mutex_init(&gxp->pin_user_pages_lock);
	mutex_init(&gxp->secure_vd_lock);

	gxp->domain_pool = kmalloc(sizeof(*gxp->domain_pool), GFP_KERNEL);
	if (!gxp->domain_pool) {
		ret = -ENOMEM;
		goto err_debug_dump_exit;
	}
	if (gxp_is_direct_mode(gxp))
		ret = gxp_domain_pool_init(gxp, gxp->domain_pool,
					   GXP_NUM_CORES);
	else
		ret = gxp_domain_pool_init(gxp, gxp->domain_pool,
					   GXP_NUM_SHARED_SLICES);
	if (ret) {
		dev_err(dev,
			"Failed to initialize IOMMU domain pool (ret=%d)\n",
			ret);
		goto err_free_domain_pool;
	}

	ret = gxp_fw_init(gxp);
	if (ret) {
		dev_err(dev,
			"Failed to initialize firmware manager (ret=%d)\n",
			ret);
		goto err_domain_pool_destroy;
	}

	ret = gxp_firmware_loader_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to initialize firmware loader (ret=%d)\n",
			ret);
		goto err_fw_destroy;
	}
	gxp_dma_init_default_resources(gxp);
	gxp_vd_init(gxp);

	ret = of_property_read_u64(dev->of_node, "gxp-memory-per-core",
				   &prop);
	if (ret) {
		dev_err(dev, "Unable to get memory-per-core from device tree\n");
		gxp->memory_per_core = 0;
	} else {
		gxp->memory_per_core = (u32)prop;
	}

	ret = gxp_fw_data_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to initialize firmware data: %d\n", ret);
		goto err_vd_destroy;
	}

	ret = gxp_core_telemetry_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to initialize core telemetry (ret=%d)", ret);
		goto err_fw_data_destroy;
	}

	ret = gxp_thermal_init(gxp);
	if (ret)
		dev_warn(dev, "Failed to init thermal driver: %d\n", ret);

	gxp->gfence_mgr = gcip_dma_fence_manager_create(gxp->dev);
	if (IS_ERR(gxp->gfence_mgr)) {
		ret = PTR_ERR(gxp->gfence_mgr);
		dev_err(dev, "Failed to init DMA fence manager: %d\n", ret);
		goto err_thermal_destroy;
	}

	INIT_LIST_HEAD(&gxp->client_list);
	mutex_init(&gxp->client_list_lock);
	if (gxp->after_probe) {
		ret = gxp->after_probe(gxp);
		if (ret)
			goto err_dma_fence_destroy;
	}
	/*
	 * We only know where the system config region is after after_probe is
	 * done so this can't be called earlier.
	 */
	gxp_fw_data_populate_system_config(gxp);

	gxp->misc_dev.minor = MISC_DYNAMIC_MINOR;
	gxp->misc_dev.name = GXP_NAME;
	gxp->misc_dev.fops = &gxp_fops;
	ret = misc_register(&gxp->misc_dev);
	if (ret) {
		dev_err(dev, "Failed to register misc device: %d", ret);
		goto err_before_remove;
	}

	gxp_create_debugfs(gxp);
	gxp_debug_pointer = gxp;

	dev_info(dev, "Probe finished");
	return 0;

err_before_remove:
	if (gxp->before_remove)
		gxp->before_remove(gxp);
err_dma_fence_destroy:
	/* DMA fence manager creation doesn't need revert */
err_thermal_destroy:
	gxp_thermal_exit(gxp);
	gxp_core_telemetry_exit(gxp);
err_fw_data_destroy:
	gxp_fw_data_destroy(gxp);
err_vd_destroy:
	gxp_vd_destroy(gxp);
	gxp_firmware_loader_destroy(gxp);
err_fw_destroy:
	gxp_fw_destroy(gxp);
err_domain_pool_destroy:
	gxp_domain_pool_destroy(gxp->domain_pool);
err_free_domain_pool:
	kfree(gxp->domain_pool);
err_debug_dump_exit:
	gxp_debug_dump_exit(gxp);
	/* mailbox manager init doesn't need revert */
err_dma_exit:
	gxp_dma_exit(gxp);
err_put_tpu_dev:
	gxp_put_tpu_dev(gxp);
	gxp_put_gsa_dev(gxp);
	gxp_pm_destroy(gxp);
err_remove_debugdir:
	gxp_remove_debugdir(gxp);
	return ret;
}

static int gxp_common_platform_remove(struct platform_device *pdev)
{
	struct gxp_dev *gxp = platform_get_drvdata(pdev);

	/*
	 * Call gxp_thermal_exit before gxp_remove_debugdir since it will
	 * remove its own debugfs.
	 */
	gxp_thermal_exit(gxp);
	gxp_remove_debugdir(gxp);
	misc_deregister(&gxp->misc_dev);
	if (gxp->before_remove)
		gxp->before_remove(gxp);
	gxp_core_telemetry_exit(gxp);
	gxp_fw_data_destroy(gxp);
	gxp_vd_destroy(gxp);
	gxp_firmware_loader_destroy(gxp);
	gxp_fw_destroy(gxp);
	gxp_domain_pool_destroy(gxp->domain_pool);
	kfree(gxp->domain_pool);
	gxp_debug_dump_exit(gxp);
	gxp_dma_exit(gxp);
	gxp_put_tpu_dev(gxp);
	gxp_put_gsa_dev(gxp);
	gxp_pm_destroy(gxp);

	gxp_debug_pointer = NULL;

	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)

static int gxp_platform_suspend(struct device *dev)
{
	struct gxp_dev *gxp = dev_get_drvdata(dev);
	struct gxp_client *client;

	if (!gcip_pm_is_powered(gxp->power_mgr->pm))
		return 0;

	/* Log clients currently holding a wakelock */
	if (!mutex_trylock(&gxp->client_list_lock)) {
		dev_warn_ratelimited(
			gxp->dev,
			"Unable to get client list lock on suspend failure\n");
		return -EAGAIN;
	}

	list_for_each_entry(client, &gxp->client_list, list_entry) {
		if (!down_read_trylock(&client->semaphore)) {
			dev_warn_ratelimited(
				gxp->dev,
				"Unable to acquire client lock (tgid=%d pid=%d)\n",
				client->tgid, client->pid);
			continue;
		}

		if (client->has_block_wakelock)
			dev_warn_ratelimited(
				gxp->dev,
				"Cannot suspend with client holding wakelock (tgid=%d pid=%d)\n",
				client->tgid, client->pid);

		up_read(&client->semaphore);
	}

	mutex_unlock(&gxp->client_list_lock);

	return -EAGAIN;
}

static int gxp_platform_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops gxp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gxp_platform_suspend, gxp_platform_resume)
};

#endif /* IS_ENABLED(CONFIG_PM_SLEEP) */
