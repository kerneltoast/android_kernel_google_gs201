/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP virtual device manager.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_VD_H__
#define __GXP_VD_H__

#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/rbtree.h>
#include <linux/rwsem.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "gxp-internal.h"
#include "gxp-mapping.h"

struct mailbox_resp_queue {
	/* Queue of `struct gxp_async_response`s */
	struct list_head queue;
	/* Lock protecting access to the `queue` */
	spinlock_t lock;
	/* Waitqueue to wait on if the queue is empty */
	wait_queue_head_t waitq;
};

enum gxp_virtual_device_state {
	GXP_VD_OFF = 0,
	GXP_VD_RUNNING = 1,
	GXP_VD_SUSPENDED = 2,
	/*
	 * If the virtual device is in the unavailable state, it won't be changed
	 * back no matter what we do.
	 * Note: this state will only be set on suspend/resume failure.
	 */
	GXP_VD_UNAVAILABLE = 3,
};

struct gxp_virtual_device {
	struct gxp_dev *gxp;
	uint num_cores;
	void *fw_app;
	struct iommu_domain **core_domains;
	struct mailbox_resp_queue *mailbox_resp_queues;
	struct rb_root mappings_root;
	struct rw_semaphore mappings_semaphore;
	enum gxp_virtual_device_state state;
	/*
	 * Record the gxp->power_mgr->blk_switch_count when the vd was
	 * suspended. Use this information to know whether the block has been
	 * restarted and therefore we need to re-program CSRs in the resume
	 * process.
	 */
	u64 blk_switch_count_when_suspended;
};

/*
 * TODO(b/193180931) cleanup the relationship between the internal GXP modules.
 * For example, whether or not gxp_vd owns the gxp_fw module, and if so, if
 * other modules are expected to access the gxp_fw directly or only via gxp_vd.
 */
/*
 * Initializes the device management subsystem and allocates resources for it.
 * This is expected to be called once per driver lifecycle.
 */
int gxp_vd_init(struct gxp_dev *gxp);

/*
 * Tears down the device management subsystem.
 * This is expected to be called once per driver lifecycle.
 */
void gxp_vd_destroy(struct gxp_dev *gxp);

/**
 * gxp_vd_allocate() - Allocate and initialize a struct gxp_virtual_device
 * @gxp: The GXP device the virtual device will belong to
 * @requested_cores: The number of cores the virtual device will have
 *
 * Return: The virtual address of the virtual device or an ERR_PTR on failure
 * * -EINVAL - The number of requested cores was invalid
 * * -ENOMEM - Unable to allocate the virtual device
 * * -EBUSY  - Not enough iommu domains available
 */
struct gxp_virtual_device *gxp_vd_allocate(struct gxp_dev *gxp, u16 requested_cores);

/**
 * gxp_vd_release() - Cleanup and free a struct gxp_virtual_device
 * @vd: The virtual device to be released
 *
 * A virtual device must be stopped before it can be released.
 */
void gxp_vd_release(struct gxp_virtual_device *vd);

/**
 * gxp_vd_start() - Run a virtual device on physical cores
 * @vd: The virtual device to start
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 *
 * Return:
 * * 0      - Success
 * * -EBUSY - Insufficient physical cores were free to start @vd
 */
int gxp_vd_start(struct gxp_virtual_device *vd);

/**
 * gxp_vd_stop() - Stop a running virtual device and free up physical cores
 * @vd: The virtual device to stop
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 */
void gxp_vd_stop(struct gxp_virtual_device *vd);

/*
 * Returns the physical core ID for the specified virtual_core belonging to
 * this virtual device or -EINVAL if this virtual core is not running on a
 * physical core.
 *
 * The caller must have locked gxp->vd_semaphore for reading.
 */
int gxp_vd_virt_core_to_phys_core(struct gxp_virtual_device *vd, u16 virt_core);

/*
 * Converts a bitfield of virtual core IDs to a bitfield of physical core IDs.
 *
 * If the virtual list contains any invalid IDs, the entire physical ID list
 * will be considered invalid and this function will return 0.
 *
 * The caller must have locked gxp->vd_semaphore for reading.
 */
uint gxp_vd_virt_core_list_to_phys_core_list(struct gxp_virtual_device *vd,
					     u16 virt_core_list);

/*
 * Returns the virtual core number assigned the phys_core, inside of this
 * virtual device or -EINVAL if this core is not part of this virtual device.
 *
 * The caller must have locked gxp->vd_semaphore for reading.
 */
int gxp_vd_phys_core_to_virt_core(struct gxp_virtual_device *vd, u16 phys_core);

/**
 * gxp_vd_mapping_store() - Store a mapping in a virtual device's records
 * @vd: The virtual device @map was created for and will be stored in
 * @map: The mapping to store
 *
 * Acquires a reference to @map if it was successfully stored
 *
 * Return:
 * * 0: Success
 * * -EINVAL: @map is already stored in @vd's records
 */
int gxp_vd_mapping_store(struct gxp_virtual_device *vd,
			 struct gxp_mapping *map);

/**
 * gxp_vd_mapping_remove() - Remove a mapping from a virtual device's records
 * @vd: The VD to remove @map from
 * @map: The mapping to remove
 *
 * Releases a reference to @map if it was successfully removed
 */
void gxp_vd_mapping_remove(struct gxp_virtual_device *vd,
			   struct gxp_mapping *map);

/**
 * gxp_vd_mapping_search() - Obtain a reference to the mapping starting at the
 *                           specified device address
 * @vd: The virtual device to search for the mapping
 * @device_address: The starting device address of the mapping to find
 *
 * Obtains a reference to the returned mapping
 *
 * Return: A pointer to the mapping if found; NULL otherwise
 */
struct gxp_mapping *gxp_vd_mapping_search(struct gxp_virtual_device *vd,
					  dma_addr_t device_address);

/**
 * gxp_vd_mapping_search_in_range() - Obtain a reference to the mapping which
 *                                    contains the specified device address
 * @vd: The virtual device to search for the mapping
 * @device_address: A device address contained in the buffer the mapping to
 *                  find describes.
 *
 * Obtains a reference to the returned mapping
 *
 * Return: A pointer to the mapping if found; NULL otherwise
 */
struct gxp_mapping *
gxp_vd_mapping_search_in_range(struct gxp_virtual_device *vd,
			       dma_addr_t device_address);

/**
 * gxp_vd_mapping_search_host() - Obtain a reference to the mapping starting at
 *                                the specified user-space address
 * @vd: The virtual device to search for the mapping
 * @host_address: The starting user-space address of the mapping to find
 *
 * Obtains a reference to the returned mapping
 *
 * Return: A pointer to the mapping if found; NULL otherwise
 */
struct gxp_mapping *gxp_vd_mapping_search_host(struct gxp_virtual_device *vd,
					       u64 host_address);

/**
 * gxp_vd_suspend() - Suspend a running virtual device
 * @vd: The virtual device to suspend
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 */
void gxp_vd_suspend(struct gxp_virtual_device *vd);

/**
 * gxp_vd_resume() - Resume a suspended virtual device
 * @vd: The virtual device to resume
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 *
 * Return:
 * * 0          - Success
 * * -ETIMEDOUT - Fail to power on physical cores
 */
int gxp_vd_resume(struct gxp_virtual_device *vd);

#endif /* __GXP_VD_H__ */
