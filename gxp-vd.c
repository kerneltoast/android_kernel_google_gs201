// SPDX-License-Identifier: GPL-2.0
/*
 * GXP virtual device manager.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/slab.h>

#include "gxp-debug-dump.h"
#include "gxp-dma.h"
#include "gxp-domain-pool.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-host-device-structs.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-mailbox.h"
#include "gxp-notification.h"
#include "gxp-pm.h"
#include "gxp-telemetry.h"
#include "gxp-vd.h"
#include "gxp-wakelock.h"

static inline void hold_core_in_reset(struct gxp_dev *gxp, uint core)
{
	gxp_write_32_core(gxp, core, GXP_REG_ETM_PWRCTL,
			  1 << GXP_REG_ETM_PWRCTL_CORE_RESET_SHIFT);
}

int gxp_vd_init(struct gxp_dev *gxp)
{
	uint core;
	int ret;

	init_rwsem(&gxp->vd_semaphore);

	/* All cores start as free */
	for (core = 0; core < GXP_NUM_CORES; core++)
		gxp->core_to_vd[core] = NULL;

	ret = gxp_fw_init(gxp);

	return ret;
}

void gxp_vd_destroy(struct gxp_dev *gxp)
{
	down_write(&gxp->vd_semaphore);

	gxp_fw_destroy(gxp);

	up_write(&gxp->vd_semaphore);
}

struct gxp_virtual_device *gxp_vd_allocate(struct gxp_dev *gxp,
					   u16 requested_cores)
{
	struct gxp_virtual_device *vd;
	int i;
	int err;

	/* Assumes 0 < requested_cores <= GXP_NUM_CORES */
	if (requested_cores == 0 || requested_cores > GXP_NUM_CORES)
		return ERR_PTR(-EINVAL);

	vd = kzalloc(sizeof(*vd), GFP_KERNEL);
	if (!vd)
		return ERR_PTR(-ENOMEM);

	vd->gxp = gxp;
	vd->num_cores = requested_cores;
	vd->state = GXP_VD_OFF;

	vd->core_domains =
		kcalloc(requested_cores, sizeof(*vd->core_domains), GFP_KERNEL);
	if (!vd->core_domains) {
		err = -ENOMEM;
		goto error_free_vd;
	}
	for (i = 0; i < requested_cores; i++) {
		vd->core_domains[i] = gxp_domain_pool_alloc(gxp->domain_pool);
		if (!vd->core_domains[i]) {
			err = -EBUSY;
			goto error_free_domains;
		}
	}

	vd->mailbox_resp_queues = kcalloc(
		vd->num_cores, sizeof(*vd->mailbox_resp_queues), GFP_KERNEL);
	if (!vd->mailbox_resp_queues) {
		err = -ENOMEM;
		goto error_free_domains;
	}

	for (i = 0; i < vd->num_cores; i++) {
		INIT_LIST_HEAD(&vd->mailbox_resp_queues[i].queue);
		spin_lock_init(&vd->mailbox_resp_queues[i].lock);
		init_waitqueue_head(&vd->mailbox_resp_queues[i].waitq);
	}

	vd->mappings_root = RB_ROOT;
	init_rwsem(&vd->mappings_semaphore);

	return vd;

error_free_domains:
	for (i -= 1; i >= 0; i--)
		gxp_domain_pool_free(gxp->domain_pool, vd->core_domains[i]);
	kfree(vd->core_domains);
error_free_vd:
	kfree(vd);

	return ERR_PTR(err);
}

void gxp_vd_release(struct gxp_virtual_device *vd)
{
	struct gxp_async_response *cur, *nxt;
	int i;
	unsigned long flags;
	struct rb_node *node;
	struct gxp_mapping *mapping;

	/* Cleanup any unconsumed responses */
	for (i = 0; i < vd->num_cores; i++) {
		/*
		 * Since VD is releasing, it is not necessary to lock here.
		 * Do it anyway for consistency.
		 */
		spin_lock_irqsave(&vd->mailbox_resp_queues[i].lock, flags);
		list_for_each_entry_safe(cur, nxt,
					 &vd->mailbox_resp_queues[i].queue,
					 list_entry) {
			list_del(&cur->list_entry);
			kfree(cur);
		}
		spin_unlock_irqrestore(&vd->mailbox_resp_queues[i].lock, flags);
	}

	/*
	 * Release any un-mapped mappings
	 * Once again, it's not necessary to lock the mappings_semaphore here
	 * but do it anyway for consistency.
	 */
	down_write(&vd->mappings_semaphore);
	while ((node = rb_first(&vd->mappings_root))) {
		mapping = rb_entry(node, struct gxp_mapping, node);
		rb_erase(node, &vd->mappings_root);
		gxp_mapping_put(mapping);
	}
	up_write(&vd->mappings_semaphore);

	for (i = 0; i < vd->num_cores; i++)
		gxp_domain_pool_free(vd->gxp->domain_pool, vd->core_domains[i]);
	kfree(vd->core_domains);
	kfree(vd->mailbox_resp_queues);
	kfree(vd);
}

static int map_telemetry_buffers(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint virt_core,
				  uint core)
{
	int ret = 0;
	struct buffer_data *buff_data;

	mutex_lock(&gxp->telemetry_mgr->lock);

	/* Map logging buffers if logging is enabled */
	buff_data = gxp->telemetry_mgr->logging_buff_data;
	if (buff_data && buff_data->is_enabled) {
		ret = gxp_dma_map_allocated_coherent_buffer(
			gxp, buff_data->buffers[core], vd, BIT(virt_core),
			buff_data->size, buff_data->buffer_daddrs[core], 0);
		/* Don't bother checking tracing if logging fails */
		if (ret)
			goto out;
	}

	/* Map tracing buffers if tracing is enabled */
	buff_data = gxp->telemetry_mgr->tracing_buff_data;
	if (buff_data && buff_data->is_enabled) {
		ret = gxp_dma_map_allocated_coherent_buffer(
			gxp, buff_data->buffers[core], vd, BIT(virt_core),
			buff_data->size, buff_data->buffer_daddrs[core], 0);
		/* If tracing fails, unmap logging if it was enabled */
		if (ret) {
			buff_data = gxp->telemetry_mgr->logging_buff_data;
			if (buff_data && buff_data->is_enabled)
				gxp_dma_unmap_allocated_coherent_buffer(
					gxp, vd, BIT(virt_core),
					buff_data->size,
					buff_data->buffer_daddrs[core]);
		}
	}

out:
	mutex_unlock(&gxp->telemetry_mgr->lock);

	return ret;
}

static void unmap_telemetry_buffers(struct gxp_dev *gxp,
				    struct gxp_virtual_device *vd,
				    uint virt_core, uint core)
{
	struct buffer_data *buff_data;

	mutex_lock(&gxp->telemetry_mgr->lock);

	buff_data = gxp->telemetry_mgr->logging_buff_data;
	if (buff_data && buff_data->is_enabled)
		gxp_dma_unmap_allocated_coherent_buffer(
			gxp, vd, BIT(virt_core), buff_data->size,
			buff_data->buffer_daddrs[core]);

	buff_data = gxp->telemetry_mgr->tracing_buff_data;
	if (buff_data && buff_data->is_enabled)
		gxp_dma_unmap_allocated_coherent_buffer(
			gxp, vd, BIT(virt_core), buff_data->size,
			buff_data->buffer_daddrs[core]);

	mutex_unlock(&gxp->telemetry_mgr->lock);
}

static int map_debug_dump_buffer(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd, uint virt_core,
				 uint core)
{
	/* If debug-dump is not enabled, nothing to map */
	if (!gxp->debug_dump_mgr)
		return 0;

	return gxp_dma_map_allocated_coherent_buffer(
		gxp, gxp->debug_dump_mgr->buf.vaddr, vd, BIT(virt_core),
		gxp->debug_dump_mgr->buf.size, gxp->debug_dump_mgr->buf.daddr,
		0);
}

static void unmap_debug_dump_buffer(struct gxp_dev *gxp,
				    struct gxp_virtual_device *vd,
				    uint virt_core, uint core)
{
	if (!gxp->debug_dump_mgr)
		return;

	gxp_dma_unmap_allocated_coherent_buffer(
		gxp, vd, BIT(virt_core), gxp->debug_dump_mgr->buf.size,
		gxp->debug_dump_mgr->buf.daddr);
}

/* Caller must hold gxp->vd_semaphore for writing */
int gxp_vd_start(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint core;
	uint available_cores = 0;
	uint cores_remaining = vd->num_cores;
	uint core_list = 0;
	uint virt_core = 0;
	int ret = 0;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == NULL) {
			if (available_cores < vd->num_cores)
				core_list |= BIT(core);
			available_cores++;
		}
	}

	if (available_cores < vd->num_cores) {
		dev_err(gxp->dev, "Insufficient available cores. Available: %u. Requested: %u\n",
			available_cores, vd->num_cores);
		return -EBUSY;
	}

	vd->fw_app = gxp_fw_data_create_app(gxp, core_list);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (cores_remaining == 0)
			break;

		if (core_list & BIT(core)) {
			gxp->core_to_vd[core] = vd;
			cores_remaining--;
			ret = gxp_dma_domain_attach_device(gxp, vd, virt_core,
							   core);
			if (ret)
				goto err_clean_all_cores;
			ret = gxp_dma_map_core_resources(gxp, vd, virt_core,
							 core);
			if (ret)
				goto err_detach_domain;
			ret = map_telemetry_buffers(gxp, vd, virt_core, core);
			if (ret)
				goto err_unmap_res;
			ret = map_debug_dump_buffer(gxp, vd, virt_core, core);
			if (ret)
				goto err_unmap_telem;
			virt_core++;
		}
	}

	ret = gxp_firmware_run(gxp, vd, core_list);
	if (ret)
		goto err_clean_all_cores;

	vd->state = GXP_VD_RUNNING;
	return ret;

err_unmap_telem:
	unmap_telemetry_buffers(gxp, vd, virt_core, core);
err_unmap_res:
	gxp_dma_unmap_core_resources(gxp, vd, virt_core, core);
err_detach_domain:
	gxp_dma_domain_detach_device(gxp, vd, virt_core);
err_clean_all_cores:
	gxp->core_to_vd[core] = NULL;
	virt_core--;
	while (core > 0) {
		core--;
		if (core_list & BIT(core)) {
			unmap_debug_dump_buffer(gxp, vd, virt_core, core);
			unmap_telemetry_buffers(gxp, vd, virt_core, core);
			gxp_dma_unmap_core_resources(gxp, vd, virt_core, core);
			gxp_dma_domain_detach_device(gxp, vd, virt_core);
			gxp->core_to_vd[core] = NULL;
			virt_core--;
		}
	}
	gxp_fw_data_destroy_app(gxp, vd->fw_app);

	return ret;
}

/* Caller must hold gxp->vd_semaphore for writing */
void gxp_vd_stop(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint core, core_list = 0;
	uint virt_core = 0;
	uint lpm_state;

	if ((vd->state == GXP_VD_OFF || vd->state == GXP_VD_RUNNING) &&
	    gxp_pm_get_blk_state(gxp) != AUR_OFF) {
		/*
		 * Put all cores in the VD into reset so they can not wake each other up
		 */
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (gxp->core_to_vd[core] == vd) {
				lpm_state = gxp_lpm_get_state(gxp, core);
				if (lpm_state != LPM_PG_STATE)
					hold_core_in_reset(gxp, core);
			}
		}
	}

	for (core = 0; core < GXP_NUM_CORES; core++)
		if (gxp->core_to_vd[core] == vd)
			core_list |= BIT(core);

	gxp_firmware_stop(gxp, vd, core_list);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == vd) {
			unmap_debug_dump_buffer(gxp, vd, virt_core, core);
			unmap_telemetry_buffers(gxp, vd, virt_core, core);
			gxp_dma_unmap_core_resources(gxp, vd, virt_core, core);
			if (vd->state == GXP_VD_RUNNING)
				gxp_dma_domain_detach_device(gxp, vd, virt_core);
			gxp->core_to_vd[core] = NULL;
			virt_core++;
		}
	}

	if (!IS_ERR_OR_NULL(vd->fw_app)) {
		gxp_fw_data_destroy_app(gxp, vd->fw_app);
		vd->fw_app = NULL;
	}
}

/*
 * Caller must have locked `gxp->vd_semaphore` for writing.
 */
void gxp_vd_suspend(struct gxp_virtual_device *vd)
{
	uint core;
	struct gxp_dev *gxp = vd->gxp;
	u32 boot_state;
	uint failed_cores = 0;
	uint virt_core;

	lockdep_assert_held_write(&gxp->vd_semaphore);
	dev_info(gxp->dev, "Suspending VD ...\n");
	if (vd->state == GXP_VD_SUSPENDED) {
		dev_err(gxp->dev,
			"Attempt to suspend a virtual device twice\n");
		return;
	}
	gxp_pm_force_clkmux_normal(gxp);
	/*
	 * Start the suspend process for all of this VD's cores without waiting
	 * for completion.
	 */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == vd) {
			if (!gxp_lpm_wait_state_ne(gxp, core, LPM_ACTIVE_STATE)) {
				vd->state = GXP_VD_UNAVAILABLE;
				failed_cores |= BIT(core);
				hold_core_in_reset(gxp, core);
				dev_err(gxp->dev, "Core %u stuck at LPM_ACTIVE_STATE", core);
				continue;
			}
			/* Mark the boot mode as a suspend event */
			gxp_firmware_set_boot_mode(gxp, core,
				GXP_BOOT_MODE_REQUEST_SUSPEND);
			/*
			 * Request a suspend event by sending a mailbox
			 * notification.
			 */
			gxp_notification_send(gxp, core,
					      CORE_NOTIF_SUSPEND_REQUEST);
		}
	}
	virt_core = 0;
	/* Wait for all cores to complete core suspension. */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == vd) {
			if (!(failed_cores & BIT(core))) {
				if (!gxp_lpm_wait_state_eq(gxp, core,
							   LPM_PG_STATE)) {
					boot_state = gxp_firmware_get_boot_mode(
							gxp, core);
					if (boot_state !=
					    GXP_BOOT_MODE_STATUS_SUSPEND_COMPLETED) {
						dev_err(gxp->dev,
							"Suspension request on core %u failed (status: %u)",
							core, boot_state);
						vd->state = GXP_VD_UNAVAILABLE;
						failed_cores |= BIT(core);
						hold_core_in_reset(gxp, core);
					}
				} else {
					/* Re-set PS1 as the default low power state. */
					gxp_lpm_enable_state(gxp, core,
							     LPM_CG_STATE);
				}
			}
			gxp_dma_domain_detach_device(gxp, vd, virt_core);
			virt_core++;
		}
	}
	if (vd->state == GXP_VD_UNAVAILABLE) {
		/* shutdown all cores if virtual device is unavailable */
		for (core = 0; core < GXP_NUM_CORES; core++)
			if (gxp->core_to_vd[core] == vd)
				gxp_pm_core_off(gxp, core);
	} else {
		vd->blk_switch_count_when_suspended =
			gxp_pm_get_blk_switch_count(gxp);
		vd->state = GXP_VD_SUSPENDED;
	}
	gxp_pm_resume_clkmux(gxp);
}

/*
 * Caller must have locked `gxp->vd_semaphore` for writing.
 */
int gxp_vd_resume(struct gxp_virtual_device *vd)
{
	int ret = 0;
	uint core;
	uint virt_core = 0;
	uint timeout;
	u32 boot_state;
	struct gxp_dev *gxp = vd->gxp;
	u64 curr_blk_switch_count;
	uint failed_cores = 0;

	lockdep_assert_held_write(&gxp->vd_semaphore);
	dev_info(gxp->dev, "Resuming VD ...\n");
	if (vd->state != GXP_VD_SUSPENDED) {
		dev_err(gxp->dev,
			"Attempt to resume a virtual device which was not suspended\n");
		return -EBUSY;
	}
	gxp_pm_force_clkmux_normal(gxp);
	curr_blk_switch_count = gxp_pm_get_blk_switch_count(gxp);
	/*
	 * Start the resume process for all of this VD's cores without waiting
	 * for completion.
	 */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == vd) {
			gxp_dma_domain_attach_device(gxp, vd, virt_core, core);
			/*
			 * The comparison is to check if blk_switch_count is
			 * changed. If it's changed, it means the block is rebooted and
			 * therefore we need to set up the hardware again.
			 */
			if (vd->blk_switch_count_when_suspended != curr_blk_switch_count) {
				ret = gxp_firmware_setup_hw_after_block_off(
					gxp, core, /*verbose=*/false);
				if (ret) {
					vd->state = GXP_VD_UNAVAILABLE;
					failed_cores |= BIT(core);
					virt_core++;
					dev_err(gxp->dev, "Failed to power up core %u\n", core);
					continue;
				}
			}
			/* Mark this as a resume power-up event. */
			gxp_firmware_set_boot_mode(gxp, core,
				GXP_BOOT_MODE_REQUEST_RESUME);
			/*
			 * Power on the core by explicitly switching its PSM to
			 * PS0 (LPM_ACTIVE_STATE).
			 */
			gxp_lpm_set_state(gxp, core, LPM_ACTIVE_STATE,
					  /*verbose=*/false);
			virt_core++;
		}
	}
	/* Wait for all cores to complete core resumption. */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == vd) {
			if (!(failed_cores & BIT(core))) {
				/* in microseconds */
				timeout = 1000000;
				while (--timeout) {
					boot_state = gxp_firmware_get_boot_mode(
						gxp, core);
					if (boot_state ==
					    GXP_BOOT_MODE_STATUS_RESUME_COMPLETED)
						break;
					udelay(1 * GXP_TIME_DELAY_FACTOR);
				}
				if (timeout == 0 &&
				    boot_state !=
					    GXP_BOOT_MODE_STATUS_RESUME_COMPLETED) {
					dev_err(gxp->dev,
						"Resume request on core %u failed (status: %u)",
						core, boot_state);
					ret = -EBUSY;
					vd->state = GXP_VD_UNAVAILABLE;
					failed_cores |= BIT(core);
				}
			}
		}
	}
	if (vd->state == GXP_VD_UNAVAILABLE) {
		/* shutdown all cores if virtual device is unavailable */
		virt_core = 0;
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (gxp->core_to_vd[core] == vd) {
				gxp_dma_domain_detach_device(gxp, vd, virt_core);
				gxp_pm_core_off(gxp, core);
				virt_core++;
			}
		}
	} else {
		vd->state = GXP_VD_RUNNING;
	}
	gxp_pm_resume_clkmux(gxp);
	return ret;
}

/* Caller must have locked `gxp->vd_semaphore` for reading */
int gxp_vd_virt_core_to_phys_core(struct gxp_virtual_device *vd, u16 virt_core)
{
	struct gxp_dev *gxp = vd->gxp;
	uint phys_core;
	uint virt_core_index = 0;

	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		if (gxp->core_to_vd[phys_core] == vd) {
			if (virt_core_index == virt_core) {
				/* Found virtual core */
				return phys_core;
			}

			virt_core_index++;
		}
	}

	dev_dbg(gxp->dev, "No mapping for virtual core %u\n", virt_core);
	return -EINVAL;
}

/* Caller must have locked `gxp->vd_semaphore` for reading */
uint gxp_vd_virt_core_list_to_phys_core_list(struct gxp_virtual_device *vd,
					     u16 virt_core_list)
{
	uint phys_core_list = 0;
	uint virt_core = 0;
	int phys_core;

	while (virt_core_list) {
		/*
		 * Get the next virt core by finding the index of the first
		 * set bit in the core list.
		 *
		 * Subtract 1 since `ffs()` returns a 1-based index. Since
		 * virt_core_list cannot be 0 at this point, no need to worry
		 * about wrap-around.
		 */
		virt_core = ffs(virt_core_list) - 1;

		/* Any invalid virt cores invalidate the whole list */
		phys_core = gxp_vd_virt_core_to_phys_core(vd, virt_core);
		if (phys_core < 0)
			return 0;

		phys_core_list |= BIT(phys_core);
		virt_core_list &= ~BIT(virt_core);
	}

	return phys_core_list;
}

/* Caller must have locked `gxp->vd_semaphore` for reading */
int gxp_vd_phys_core_to_virt_core(struct gxp_virtual_device *vd,
						u16 phys_core)
{
	struct gxp_dev *gxp = vd->gxp;
	int virt_core = 0;
	uint core;

	if (gxp->core_to_vd[phys_core] != vd) {
		virt_core = -EINVAL;
		goto out;
	}

	/*
	 * A core's virtual core ID == the number of physical cores in the same
	 * virtual device with a lower physical core ID than its own.
	 */
	for (core = 0; core < phys_core; core++) {
		if (gxp->core_to_vd[core] == vd)
			virt_core++;
	}
out:
	return virt_core;
}

int gxp_vd_mapping_store(struct gxp_virtual_device *vd,
			 struct gxp_mapping *map)
{
	struct rb_node **link;
	struct rb_node *parent = NULL;
	dma_addr_t device_address = map->device_address;
	struct gxp_mapping *mapping;

	link = &vd->mappings_root.rb_node;

	down_write(&vd->mappings_semaphore);

	/* Figure out where to put the new node */
	while (*link) {
		parent = *link;
		mapping = rb_entry(parent, struct gxp_mapping, node);

		if (mapping->device_address > device_address)
			link = &(*link)->rb_left;
		else if (mapping->device_address < device_address)
			link = &(*link)->rb_right;
		else
			goto out;
	}

	/* Add new node and rebalance the tree. */
	rb_link_node(&map->node, parent, link);
	rb_insert_color(&map->node, &vd->mappings_root);

	/* Acquire a reference to the mapping */
	gxp_mapping_get(map);

	up_write(&vd->mappings_semaphore);

	return 0;

out:
	up_write(&vd->mappings_semaphore);
	dev_err(vd->gxp->dev, "Duplicate mapping: %pad\n",
		&map->device_address);
	return -EEXIST;
}

void gxp_vd_mapping_remove(struct gxp_virtual_device *vd,
			   struct gxp_mapping *map)
{
	down_write(&vd->mappings_semaphore);

	/* Drop the mapping from this virtual device's records */
	rb_erase(&map->node, &vd->mappings_root);

	/* Release the reference obtained in gxp_vd_mapping_store() */
	gxp_mapping_put(map);

	up_write(&vd->mappings_semaphore);
}

static bool is_device_address_in_mapping(struct gxp_mapping *mapping,
					 dma_addr_t device_address)
{
	return ((device_address >= mapping->device_address) &&
		(device_address < (mapping->device_address + mapping->size)));
}

static struct gxp_mapping *
gxp_vd_mapping_internal_search(struct gxp_virtual_device *vd,
			       dma_addr_t device_address, bool check_range)
{
	struct rb_node *node;
	struct gxp_mapping *mapping;

	down_read(&vd->mappings_semaphore);

	node = vd->mappings_root.rb_node;

	while (node) {
		mapping = rb_entry(node, struct gxp_mapping, node);
		if ((mapping->device_address == device_address) ||
		    (check_range &&
		     is_device_address_in_mapping(mapping, device_address))) {
			gxp_mapping_get(mapping);
			up_read(&vd->mappings_semaphore);
			return mapping; /* Found it */
		} else if (mapping->device_address > device_address) {
			node = node->rb_left;
		} else {
			node = node->rb_right;
		}
	}

	up_read(&vd->mappings_semaphore);

	return NULL;
}

struct gxp_mapping *gxp_vd_mapping_search(struct gxp_virtual_device *vd,
					  dma_addr_t device_address)
{
	return gxp_vd_mapping_internal_search(vd, device_address, false);
}

struct gxp_mapping *
gxp_vd_mapping_search_in_range(struct gxp_virtual_device *vd,
			       dma_addr_t device_address)
{
	return gxp_vd_mapping_internal_search(vd, device_address, true);
}

struct gxp_mapping *gxp_vd_mapping_search_host(struct gxp_virtual_device *vd,
					       u64 host_address)
{
	struct rb_node *node;
	struct gxp_mapping *mapping;

	/*
	 * dma-buf mappings can not be looked-up by host address since they are
	 * not mapped from a user-space address.
	 */
	if (!host_address) {
		dev_dbg(vd->gxp->dev,
			"Unable to get dma-buf mapping by host address\n");
		return NULL;
	}

	down_read(&vd->mappings_semaphore);

	/* Iterate through the elements in the rbtree */
	for (node = rb_first(&vd->mappings_root); node; node = rb_next(node)) {
		mapping = rb_entry(node, struct gxp_mapping, node);
		if (mapping->host_address == host_address) {
			gxp_mapping_get(mapping);
			up_read(&vd->mappings_semaphore);
			return mapping;
		}
	}

	up_read(&vd->mappings_semaphore);

	return NULL;
}
