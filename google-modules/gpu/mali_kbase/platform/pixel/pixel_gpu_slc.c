// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022-2023 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */

/* Mali core includes */
#include <mali_kbase.h>

/* UAPI includes */
#include <uapi/gpu/arm/midgard/platform/pixel/pixel_gpu_common_slc.h>
/* Back-door mali_pixel include */
#include <uapi/gpu/arm/midgard/platform/pixel/pixel_memory_group_manager.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_slc.h"


/**
 * struct gpu_slc_liveness_update_info - Buffer info, and live ranges
 *
 * @buffer_va:         Array of buffer base virtual addresses
 * @buffer_sizes:      Array of buffer sizes
 * @live_ranges:       Array of &struct kbase_pixel_gpu_slc_liveness_mark denoting live ranges for
 *                     each buffer
 * @live_ranges_count: Number of elements in the live ranges buffer
 */
struct gpu_slc_liveness_update_info {
	u64* buffer_va;
	u64* buffer_sizes;
	struct kbase_pixel_gpu_slc_liveness_mark* live_ranges;
	u64 live_ranges_count;
};

/**
 * gpu_slc_lock_as - Lock the current process address space
 *
 * @kctx:  The &struct kbase_context
 */
static void gpu_slc_lock_as(struct kbase_context *kctx)
{
	down_write(kbase_mem_get_process_mmap_lock());
	kbase_gpu_vm_lock(kctx);
}

/**
 * gpu_slc_unlock_as - Unlock the current process address space
 *
 * @kctx:  The &struct kbase_context
 */
static void gpu_slc_unlock_as(struct kbase_context *kctx)
{
	kbase_gpu_vm_unlock(kctx);
	up_write(kbase_mem_get_process_mmap_lock());
}

/**
 * gpu_slc_in_group - Check whether the region is SLC cacheable
 *
 * @reg:   The gpu memory region to check for an SLC cacheable memory group.
 */
static bool gpu_slc_in_group(struct kbase_va_region* reg)
{
	return reg->gpu_alloc->group_id == MGM_SLC_GROUP_ID;
}

/**
 * gpu_slc_get_region - Find the gpu memory region from a virtual address
 *
 * @kctx:  The &struct kbase_context
 * @va:    The base gpu virtual address of the region
 *
 * Return: On success, returns a valid memory region. On failure NULL is returned.
 */
static struct kbase_va_region* gpu_slc_get_region(struct kbase_context *kctx, u64 va)
{
	struct kbase_va_region *reg;

	if (!va)
		goto invalid;

	if ((va & ~PAGE_MASK) && (va >= PAGE_SIZE))
		goto invalid;

	/* Find the region that the virtual address belongs to */
	reg = kbase_region_tracker_find_region_base_address(kctx, va);

	/* Validate the region */
	if (kbase_is_region_invalid_or_free(reg))
		goto invalid;

	return reg;

invalid:
	dev_dbg(kctx->kbdev->dev, "pixel: failed to find valid region for gpu_va: %llu", va);
	return NULL;
}

/**
 * gpu_slc_migrate_region - Add PBHA that will make the pages SLC cacheable
 *
 * @kctx:  The &struct kbase_context
 * @reg:   The gpu memory region migrate to an SLC cacheable memory group
 */
static void gpu_slc_migrate_region(struct kbase_context *kctx, struct kbase_va_region* reg)
{
	int err;

	KBASE_DEBUG_ASSERT(kctx);
	KBASE_DEBUG_ASSERT(reg);

	err = kbase_mmu_update_pages(kctx, reg->start_pfn,
			kbase_get_gpu_phy_pages(reg),
			kbase_reg_current_backed_size(reg),
			reg->flags,
			MGM_SLC_GROUP_ID);
	if (err)
		dev_warn(kctx->kbdev->dev, "pixel: failed to move region to SLC: %d", err);
	else
		/* If everything is good, then set the new group on the region. */
		reg->gpu_alloc->group_id = MGM_SLC_GROUP_ID;
}

/**
 * gpu_slc_resize_partition - Attempt to resize the GPU's SLC partition to meet demand.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
static void gpu_slc_resize_partition(struct kbase_device* kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	/* Request that the mgm select an SLC partition that fits our demand */
	pixel_mgm_resize_group_to_fit(kbdev->mgm_dev, MGM_SLC_GROUP_ID, pc->slc.demand);

	dev_dbg(kbdev->dev, "pixel: resized GPU SLC partition to meet demand: %llu", pc->slc.demand);
}

/**
 * gpu_slc_get_partition_size - Query the current size of the GPU's SLC partition.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Returns the size of the GPU's SLC partition.
 */
static u64 gpu_slc_get_partition_size(struct kbase_device* kbdev)
{
	u64 const partition_size = pixel_mgm_query_group_size(kbdev->mgm_dev, MGM_SLC_GROUP_ID);

	dev_dbg(kbdev->dev, "pixel: GPU SLC partition partition size: %llu", partition_size);

	return partition_size;
}

/**
 * gpu_slc_liveness_update - Respond to a liveness update by trying to put the new buffers into free
 *                           SLC space, and resizing the partition to meet demand.
 *
 * @kctx:   The &struct kbase_context corresponding to a user space context which sent the liveness
 *          update
 * @info:   See struct gpu_slc_liveness_update_info
 */
static void gpu_slc_liveness_update(struct kbase_context* kctx,
                                    struct gpu_slc_liveness_update_info* info)
{
	struct kbase_device* kbdev = kctx->kbdev;
	struct pixel_context *pc = kbdev->platform_context;
	struct pixel_platform_data *kctx_pd = kctx->platform_data;
	u64 current_usage = 0;
	u64 current_demand = 0;
	u64 free_space;
	int i;

	/* Lock the process address space before modifying ATE's */
	gpu_slc_lock_as(kctx);

	/* Synchronize updates to the partition size and usage */
	mutex_lock(&pc->slc.lock);

	dev_dbg(kbdev->dev, "pixel: buffer liveness update received");

	/* Remove the usage and demand from the previous liveness update */
	pc->slc.demand -= kctx_pd->slc.peak_demand;
	pc->slc.usage -= kctx_pd->slc.peak_usage;
	kctx_pd->slc.peak_demand = 0;
	kctx_pd->slc.peak_usage = 0;

	/* Calculate the remaining free space in the SLC partition (floored at 0) */
	free_space = gpu_slc_get_partition_size(kbdev);
	free_space -= min(free_space, pc->slc.usage);

	for (i = 0; i < info->live_ranges_count; ++i)
	{
		struct kbase_va_region *reg;
		u64 const size = info->buffer_sizes[info->live_ranges[i].index];
		u64 const va = info->buffer_va[info->live_ranges[i].index];

		reg = gpu_slc_get_region(kctx, va);
		if(!reg)
			continue;

		switch (info->live_ranges[i].type)
		{
		case KBASE_PIXEL_GPU_LIVE_RANGE_BEGIN:
			/* Update demand as though there's no size limit */
			current_demand += size;
			kctx_pd->slc.peak_demand = max(kctx_pd->slc.peak_demand, current_demand);

			/* Check whether there's free space in the partition to store the buffer */
			if (free_space >= current_usage + size)
				gpu_slc_migrate_region(kctx, reg);

			/* This may be true, even if the space calculation above returned false,
			 * as a previous call to this function may have migrated the region.
			 * In such a scenario, the current_usage may exceed the available free_space
			 * and we will be oversubscribed to the SLC partition.
			 * We could migrate the region back to the non-SLC group, but this would
			 * require an SLC flush, so for now we do nothing.
			 */
			if (gpu_slc_in_group(reg)) {
				current_usage += size;
				kctx_pd->slc.peak_usage = max(kctx_pd->slc.peak_usage, current_usage);
			}
			break;
		case KBASE_PIXEL_GPU_LIVE_RANGE_END:
			current_demand -= size;
			if (gpu_slc_in_group(reg))
				current_usage -= size;
			break;
		}
	}
	/* Indicates a missing live range end marker */
	WARN_ON_ONCE(current_demand != 0 || current_usage != 0);

	/* Update the total usage and demand */
	pc->slc.demand += kctx_pd->slc.peak_demand;
	pc->slc.usage += kctx_pd->slc.peak_usage;

	dev_dbg(kbdev->dev,
	        "pixel: kctx_%d, peak_demand: %llu, peak_usage: %llu",
	        kctx->id,
	        kctx_pd->slc.peak_demand,
	        kctx_pd->slc.peak_usage);
	dev_dbg(kbdev->dev, "pixel: kbdev, demand: %llu, usage: %llu", pc->slc.demand, pc->slc.usage);

	/* Trigger partition resize based on the new demand */
	gpu_slc_resize_partition(kctx->kbdev);

	mutex_unlock(&pc->slc.lock);
	gpu_slc_unlock_as(kctx);
}

/**
 * gpu_pixel_handle_buffer_liveness_update_ioctl() - See gpu_slc_liveness_update
 *
 * @kctx:   The &struct kbase_context corresponding to a user space context which sent the liveness
 *          update
 * @update: See struct kbase_ioctl_buffer_liveness_update
 *
 * Context: Process context. Takes and releases the GPU power domain lock. Expects the caller to
 *          hold the DVFS lock.
 */
int gpu_pixel_handle_buffer_liveness_update_ioctl(struct kbase_context* kctx,
                                                  struct kbase_ioctl_buffer_liveness_update* update)
{
	int err = 0;
	struct gpu_slc_liveness_update_info info;
	u64* buff;

	/* Compute the sizes of the user space arrays that we need to copy */
	u64 const buffer_info_size = sizeof(u64) * update->buffer_count;
	u64 const live_ranges_size =
	    sizeof(struct kbase_pixel_gpu_slc_liveness_mark) * update->live_ranges_count;

	/* Nothing to do */
	if (!buffer_info_size || !live_ranges_size)
		goto done;

	/* Guard against nullptr */
	if (!update->live_ranges_address || !update->buffer_va_address || !update->buffer_sizes_address)
		goto done;

	/* Allocate the memory we require to copy from user space */
	buff = kmalloc(buffer_info_size * 2 + live_ranges_size, GFP_KERNEL);
	if (buff == NULL) {
		dev_err(kctx->kbdev->dev, "pixel: failed to allocate buffer for liveness update");
		err = -ENOMEM;
		goto done;
	}

	/* Set up the info struct by pointing into the allocation. All 8 byte aligned */
	info = (struct gpu_slc_liveness_update_info){
	    .buffer_va = buff,
	    .buffer_sizes = buff + update->buffer_count,
	    .live_ranges = (struct kbase_pixel_gpu_slc_liveness_mark*)(buff + update->buffer_count * 2),
	    .live_ranges_count = update->live_ranges_count,
	};

	/* Copy the data from user space */
	err =
	    copy_from_user(info.live_ranges, u64_to_user_ptr(update->live_ranges_address), live_ranges_size);
	if (err) {
		dev_err(kctx->kbdev->dev, "pixel: failed to copy live ranges");
		err = -EFAULT;
		goto done;
	}

	err = copy_from_user(
	    info.buffer_sizes, u64_to_user_ptr(update->buffer_sizes_address), buffer_info_size);
	if (err) {
		dev_err(kctx->kbdev->dev, "pixel: failed to copy buffer sizes");
		err = -EFAULT;
		goto done;
	}

	err = copy_from_user(info.buffer_va, u64_to_user_ptr(update->buffer_va_address), buffer_info_size);
	if (err) {
		dev_err(kctx->kbdev->dev, "pixel: failed to copy buffer addresses");
		err = -EFAULT;
		goto done;
	}

	/* Execute an slc update */
	gpu_slc_liveness_update(kctx, &info);

done:
	kfree(buff);

	return err;
}

/**
 * gpu_slc_kctx_init() - Called when a kernel context is created
 *
 * @kctx: The &struct kbase_context that is being initialized
 *
 * This function is called when the GPU driver is initializing a new kernel context. This event is
 * used to set up data structures that will be used to track this context's usage of the SLC.
 *
 * Return: Returns 0 on success, or an error code on failure.
 */
int gpu_slc_kctx_init(struct kbase_context *kctx)
{
	(void)kctx;
	return 0;
}

/**
 * gpu_slc_kctx_term() - Called when a kernel context is terminated
 *
 * @kctx: The &struct kbase_context that is being terminated
 *
 * Free up SLC space used by the buffers that this context owns.
 */
void gpu_slc_kctx_term(struct kbase_context *kctx)
{
	struct kbase_device* kbdev = kctx->kbdev;
	struct pixel_context *pc = kbdev->platform_context;
	struct pixel_platform_data *kctx_pd = kctx->platform_data;

	mutex_lock(&pc->slc.lock);

	/* Deduct the usage and demand, freeing that SLC space for the next update */
	pc->slc.demand -= kctx_pd->slc.peak_demand;
	pc->slc.usage -= kctx_pd->slc.peak_usage;

	/* Trigger partition resize based on the new demand */
	gpu_slc_resize_partition(kctx->kbdev);

	mutex_unlock(&pc->slc.lock);
}


/**
 * gpu_slc_init - Initialize the SLC partition for the GPU
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: On success, returns 0. On failure an error code is returned.
 */
int gpu_slc_init(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	mutex_init(&pc->slc.lock);

	return 0;
}

/**
 * gpu_slc_term() - Terminates the Pixel GPU SLC partition.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_slc_term(struct kbase_device *kbdev)
{
	(void)kbdev;
}
