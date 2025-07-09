/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP virtual device manager.
 *
 * Copyright (C) 2021-2022 Google LLC
 */

#ifndef __GXP_VD_H__
#define __GXP_VD_H__

#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/refcount.h>
#include <linux/rwsem.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <gcip/gcip-image-config.h>
#include <gcip/gcip-iommu-reserve.h>

#include "gxp-host-device-structs.h"
#include "gxp-internal.h"
#include "gxp-mailbox-manager.h"
#include "gxp-mapping.h"

#define GXP_COMMAND_CREDIT_PER_VD 16

/* A special client ID for secure workloads pre-agreed with MCU firmware. */
#define SECURE_CLIENT_ID (3 << 10)

struct mailbox_resp_queue {
	/* Queue of waiting async responses */
	struct list_head wait_queue;
	/* Queue of arrived async responses */
	struct list_head dest_queue;
	/* Lock protecting access to the `queue` */
	spinlock_t lock;
	/* Waitqueue to wait on if the queue is empty */
	wait_queue_head_t waitq;
};

enum gxp_virtual_device_state {
	GXP_VD_OFF,
	GXP_VD_READY,
	GXP_VD_RUNNING,
	GXP_VD_SUSPENDED,
	/*
	 * If the virtual device is in the unavailable state, it won't be changed
	 * back no matter what we do.
	 * Note: this state will only be set on suspend/resume failure.
	 */
	GXP_VD_UNAVAILABLE,
	/*
	 * gxp_vd_release() has been called. VD with this state means it's
	 * waiting for the last reference to be put(). All fields in VD is
	 * invalid in this state.
	 */
	GXP_VD_RELEASED,
};

struct gxp_virtual_device {
	struct gxp_dev *gxp;
	uint num_cores;
	void *fw_app;
	struct gcip_iommu_domain *domain;
	struct mailbox_resp_queue *mailbox_resp_queues;
	struct rb_root mappings_root;
	struct rw_semaphore mappings_semaphore;
	/* Used to save doorbell state on VD resume. */
	uint doorbells_state[GXP_NUM_DOORBELLS_PER_VD];
	enum gxp_virtual_device_state state;
	u32 invalidated_reason;
	/*
	 * Record the gxp->power_mgr->blk_switch_count when the vd was
	 * suspended. Use this information to know whether the block has been
	 * restarted and therefore we need to re-program CSRs in the resume
	 * process.
	 */
	u64 blk_switch_count_when_suspended;
	/*
	 * @domain of each virtual device will map a slice of shared buffer. It stores which index
	 * of slice is used by this VD.
	 */
	int slice_index;
	/*
	 * The SG table that holds the regions specified in the image config's
	 * non-secure IOMMU mappings.
	 */
	struct {
		dma_addr_t daddr;
		struct sg_table *sgt;
	} ns_regions[GCIP_IMG_CFG_MAX_NS_IOMMU_MAPPINGS];
	/*
	 * The config regions specified in image config.
	 * core_cfg's size should be a multiple of GXP_NUM_CORES.
	 */
	struct gxp_mapped_resource core_cfg, vd_cfg, sys_cfg, lpm;
	uint core_list;
	/*
	 * The ID of DSP client. -1 if it is not allocated.
	 * This is allocated by the DSP kernel driver, but will be set to this variable only when
	 * the client of this vd acquires the block wakelock successfully. (i.e, after the kernel
	 * driver allocates a virtual mailbox with the firmware side successfully by sending the
	 * `allocate_vmbox` KCI command.)
	 */
	int client_id;
	/*
	 * The ID of TPU client. -1 if it is not allocated.
	 * This ID will be fetched from the TPU kernel driver.
	 */
	int tpu_client_id;
	/* Whether DSP KD sent `link_offload_vmbox` KCI successfully to MCU FW or not. */
	bool tpu_linked;
	/*
	 * Protects credit. Use a spin lock because the critical section of
	 * using @credit is pretty small.
	 */
	spinlock_t credit_lock;
	/*
	 * Credits for sending mailbox commands. It's initialized as
	 * GXP_COMMAND_CREDIT_PER_VD. The value is decreased on sending
	 * mailbox commands; increased on receiving mailbox responses.
	 * Mailbox command requests are rejected when this value reaches 0.
	 *
	 * Only used in MCU mode.
	 */
	uint credit;
	/* Whether it's the first time allocating a VMBox for this VD. */
	bool first_open;
	bool is_secure;
	refcount_t refcount;
	/* A constant ID assigned after VD is allocated. For debug only. */
	int vdid;
	struct gcip_image_config_parser cfg_parser;
	/* Protects @dma_fence_list. */
	struct mutex fence_list_lock;
	/* List of GXP DMA fences owned by this VD. */
	struct list_head gxp_fence_list;
	/* Protects changing the state of vd while generating a debug dump. */
	struct mutex debug_dump_lock;
	/* An eventfd which will be triggered when this vd is invalidated. */
	struct gxp_eventfd *invalidate_eventfd;
	/*
	 * If true, the MCU FW communicating with this VD has been crashed and it must not work
	 * with any MCU FW anymore regardless of its state.
	 */
	bool mcu_crashed;
	/* The manager of IOMMU reserve regions. */
	struct gcip_iommu_reserve_manager *iommu_reserve_mgr;
	/*
	 * Bitmap of cores for which KD has processed the core dump. This flag is used only
	 * in direct mode.
	 */
	atomic_t core_dump_generated_list;
	/*
	 * Wait queue to wait till the debug dump processing finishes for all the cores for
	 * the current VD before powering them down. This flag is used only in direct mode.
	 */
	wait_queue_head_t finished_dump_processing_waitq;
};

/*
 * Initializes the device management subsystem and allocates resources for it.
 * This is expected to be called once per driver lifecycle.
 */
void gxp_vd_init(struct gxp_dev *gxp);

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
 * The state of VD is initialized to GXP_VD_OFF.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 *
 * Return: The virtual address of the virtual device or an ERR_PTR on failure
 * * -EINVAL - The number of requested cores was invalid
 * * -ENOMEM - Unable to allocate the virtual device
 * * -EBUSY  - Not enough iommu domains available or insufficient physical
 *	       cores to be assigned to @vd
 * * -ENOSPC - There is no more available shared slices
 */
struct gxp_virtual_device *gxp_vd_allocate(struct gxp_dev *gxp,
					   u16 requested_cores);

/**
 * gxp_vd_release() - Cleanup a struct gxp_virtual_device
 * @vd: The virtual device to be released
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 *
 * A virtual device must be stopped before it can be released.
 *
 * If @vd's reference count is 1 before this call, this function frees @vd.
 * Otherwise @vd's state is set to GXP_VD_RELEASED.
 */
void gxp_vd_release(struct gxp_virtual_device *vd);

/**
 * gxp_vd_run() - Run a virtual device on physical cores
 * @vd: The virtual device to run
 *
 * The state of @vd should be GXP_VD_OFF or GXP_VD_READY before calling this
 * function. If this function runs successfully, the state becomes
 * GXP_VD_RUNNING. Otherwise, it would be GXP_VD_UNAVAILABLE.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 * This function is only meaningful in direct mode. On MCU mode it returns 0 directly.
 *
 * Return:
 * * 0         - Success
 * * -EINVAL   - The VD is not in GXP_VD_READY state
 * * Otherwise - Errno returned by firmware running
 */
int gxp_vd_run(struct gxp_virtual_device *vd);

/**
 * gxp_vd_stop() - Stop a running virtual device
 * @vd: The virtual device to stop
 *
 * The state of @vd will be GXP_VD_OFF.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 * This function is only meaningful in direct mode. On MCU mode it returns directly.
 */
void gxp_vd_stop(struct gxp_virtual_device *vd);

/**
 * gxp_vd_check_and_wait_for_debug_dump() - Checks if any of the cores for the given vd has
 *                                          generated the debug dump or has been requested to
 *                                          generate the forced debug dump. If yes then waits
 *                                          till all the debug dumps are processed.
 * @vd: The virtual device to be checked for the crash.
 *
 * The caller should never lock the gxp->vd_semaphore before calling this function. The logic
 * inside this function waits till the debug dumps for all the cores for the given VD are
 * processed. Processing of debug dump logic requires acquiring of gxp->vd_semaphore which will
 * end up in deadlock if this function is called after locking the gxp->vd_semaphore.
 *
 * This function is only meaningful in direct mode. On MCU mode it returns directly.
 */

void gxp_vd_check_and_wait_for_debug_dump(struct gxp_virtual_device *vd);

/*
 * Returns the physical core ID for the specified virtual_core belonging to
 * this virtual device or -EINVAL if this virtual core is not running on a
 * physical core.
 *
 * The caller must have locked gxp->vd_semaphore for reading.
 */
int gxp_vd_virt_core_to_phys_core(struct gxp_virtual_device *vd, u16 virt_core);

/**
 * gxp_vd_phys_core_to_virt_core() -Returns the virtual core ID for the specified
 *                                  @phys_core belonging to this virtual device.
 * @vd: The virtual device for which virtual core ID is requested for.
 * @phys_core: Physical core_id corresponding to which virtual core ID is requested.
 *
 * This function works only in direct mode. The caller must have locked
 * vd->debug_dump_lock before calling this function.
 *
 * Return:
 * * -EINVAL   - If no virtual core ID found for @phys_core or if the function
 *               was not invoked in direct mode.
 * * Otherwise - Returns the virtual core ID for the given @phys_core.
 */
int gxp_vd_phys_core_to_virt_core(struct gxp_virtual_device *vd, u32 phys_core);

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
 * gxp_vd_mapping_remove_locked() - The same as `gxp_vd_mapping_remove` but the caller holds
 *                                  @vd->mappings_semaphore as write.
 */
void gxp_vd_mapping_remove_locked(struct gxp_virtual_device *vd, struct gxp_mapping *map);

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
 * gxp_vd_mapping_search_locked() - The same as `gxp_vd_mapping_search` but the caller holds
 *                                  @vd->mappings_semaphore.
 */
struct gxp_mapping *gxp_vd_mapping_search_locked(struct gxp_virtual_device *vd,
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
 * The state of @vd should be GXP_VD_RUNNING before calling this function.
 * If the suspension runs successfully on all cores, the state becomes
 * GXP_VD_SUSPENDED. Otherwise, it would be GXP_VD_UNAVAILABLE.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 * This function is only meaningful in direct mode. On MCU mode it returns directly.
 */
void gxp_vd_suspend(struct gxp_virtual_device *vd);

/**
 * gxp_vd_resume() - Resume a suspended virtual device
 * @vd: The virtual device to resume
 *
 * The state of @vd should be GXP_VD_SUSPENDED before calling this function.
 * If the resumption runs successfully on all cores, the state becomes
 * GXP_VD_RUNNING. Otherwise, it would be GXP_VD_UNAVAILABLE.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 * This function is only meaningful in direct mode. On MCU mode it returns 0 directly.
 *
 * Return:
 * * 0          - Success
 * * -ETIMEDOUT - Fail to power on physical cores
 */
int gxp_vd_resume(struct gxp_virtual_device *vd);

/**
 * gxp_vd_block_ready() - This is called after the block wakelock is acquired.
 * Does required setup for serving VD such as attaching its IOMMU domain.
 *
 * @vd: The virtual device to prepare the resources
 *
 * The state of @vd should be GXP_VD_OFF before calling this function.
 * If this function runs successfully, the state becomes GXP_VD_READY.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 *
 * Return:
 * * 0          - Success
 * * -EINVAL    - The VD is not in GXP_VD_OFF state
 * * Otherwise  - Errno returned by IOMMU domain attachment
 */
int gxp_vd_block_ready(struct gxp_virtual_device *vd);

/**
 * gxp_vd_block_unready() - This is called before the block wakelock is going to be released.
 *
 * @vd: The virtual device to release the resources
 *
 * This function must be called only when the client holds the block wakelock and allocated a
 * virtual device. It doesn't have a dependency on the state of @vd, but also doesn't change the
 * state in normal situation. However, if an unexpected error happens, the state can be changed
 * to GXP_VD_UNAVAILABLE.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 */
void gxp_vd_block_unready(struct gxp_virtual_device *vd);

/*
 * Checks whether the virtual device has a positive credit, and use 1 credit when
 * yes.
 *
 * Returns true when there is enough credit, false otherwise.
 */
bool gxp_vd_has_and_use_credit(struct gxp_virtual_device *vd);
/*
 * Releases the credit.
 */
void gxp_vd_release_credit(struct gxp_virtual_device *vd);

/* Increases reference count of @vd by one and returns @vd. */
static inline struct gxp_virtual_device *
gxp_vd_get(struct gxp_virtual_device *vd)
{
	WARN_ON_ONCE(!refcount_inc_not_zero(&vd->refcount));
	return vd;
}

/*
 * Decreases reference count of @vd by one.
 *
 * If @vd->refcount becomes 0, @vd will be freed.
 */
void gxp_vd_put(struct gxp_virtual_device *vd);

/*
 * Change the status of the vd of @client_id to GXP_VD_UNAVAILABLE.
 * Internally, it will discard all pending/unconsumed user commands and call the
 * `gxp_vd_block_unready` function.
 *
 * This function will be called when the `CLIENT_FATAL_ERROR_NOTIFY` RKCI has been sent from the
 * firmware side.
 *
 * @gxp: The GXP device to obtain the handler for
 * @client_id: client_id of the crashed vd.
 * @release_vmbox: Releases the vmbox of the vd after invalidating it.
 */
void gxp_vd_invalidate_with_client_id(struct gxp_dev *gxp, int client_id, bool release_vmbox);

/*
 * Changes the status of the @vd to GXP_VD_UNAVAILABLE.
 * Internally, it will discard all pending/unconsumed user commands.
 *
 * This function will be called when some unexpected errors happened and cannot proceed requests
 * anymore with this @vd.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 *
 * @gxp: The GXP device to obtain the handler for.
 * @vd: The virtual device to be invaliated.
 * @reason: The reason why vd being invalidated.
 */
void gxp_vd_invalidate(struct gxp_dev *gxp, struct gxp_virtual_device *vd, u32 reason);

/*
 * Generates a debug dump of @vd which utilizes @core_list cores.
 *
 * This function is usually called in the MCU mode that the kernel driver cannot decide which cores
 * will be used by @vd.
 *
 * The caller must have locked gxp->vd_semaphore for writing.
 *
 * @gxp: The GXP device to obtain the handler for.
 * @vd: The virtual device to be dumped.
 * @core_list: A bitfield enumerating the physical cores on which crash is reported from firmware.
 */
void gxp_vd_generate_debug_dump(struct gxp_dev *gxp,
				struct gxp_virtual_device *vd, uint core_list);

#if GXP_HAS_MCU
/*
 * Releases the vmbox which is allocated to @vd.
 *
 * This function will call the `RELEASE_VMBOX` KCI and will always set @vd->client_id to -1. If the
 * vmbox was linked to the offload vmbox, it will also call the `gxp_vd_unlink_offload_vmbox`
 * function first internally.
 *
 * @gxp: The GXP device to obtain the handler for.
 * @vd: The virtual device to release its vmbox.
 */
void gxp_vd_release_vmbox(struct gxp_dev *gxp, struct gxp_virtual_device *vd);

/*
 * Unlinks the linkage of the vmbox of @vd to the offload chip vmbox.
 *
 * This function will call the `UNLINK_OFFLOAD_VMBOX` KCI to unlink the vmboxes and will always set
 * @vd->tpu_client_id to -1.
 *
 * @gxp: The GXP device to obtain the handler for.
 * @vd: The virtual device to unlink vmboxes.
 * @offload_client_id: The client ID of the offload chip.
 * @offload_chip_type: The type of the offload chip. (See enum gcip_kci_offload_chip_type.)
 */
void gxp_vd_unlink_offload_vmbox(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
				 u32 offload_client_id, u8 offload_chip_type);
#else /* !GXP_HAS_MCU */
static inline void gxp_vd_release_vmbox(struct gxp_dev *gxp, struct gxp_virtual_device *vd)
{
}

static inline void gxp_vd_unlink_offload_vmbox(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
					       u32 offload_client_id, u8 offload_chip_type)
{
}
#endif /* GXP_HAS_MCU */

/*
 * An ID between 0~GXP_NUM_CORES-1 and is unique to each VD.
 * Only used in direct mode.
 */
static inline uint gxp_vd_hw_slot_id(struct gxp_virtual_device *vd)
{
	return ffs(vd->core_list) - 1;
}

/*
 * Flushes pending DCI/UCI commands or unconsumed responses.
 *
 * This function shouldn't be called while holding @gxp->vd_seamphore to prevent potential deadlock.
 */
static inline void gxp_vd_release_unconsumed_async_resps(struct gxp_dev *gxp,
							 struct gxp_virtual_device *vd)
{
	gxp->mailbox_mgr->release_unconsumed_async_resps(vd);
}

#endif /* __GXP_VD_H__ */
