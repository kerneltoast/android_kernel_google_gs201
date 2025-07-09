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

#include <uapi/gpu/arm/midgard/platform/pixel/pixel_memory_group_manager.h>

/**
 * enum slc_vote_state - Whether a context is voting for SLC
 */
enum slc_vote_state {
	/** @IDLE: Idle, not voting for SLC */
	IDLE   = 0,
	/** @VOTING: Active, voting for SLC */
	VOTING = 1,
};

/**
 * transition() - Try to transition from one value to another
 *
 * @v:   Value to transition
 * @old: Starting state to transition from
 * @new: Destination state to transition to
 *
 * Return: Whether the transition was successful
 */
static bool transition(int *v, int old, int new)
{
	bool const cond = *v == old;

	if (cond)
		*v = new;

	return cond;
}

#ifndef PIXEL_GPU_SLC_ACPM_SIGNAL
/**
 * struct gpu_slc_liveness_update_info - Buffer info, and live ranges
 *
 * @buffer_sizes:      Array of buffer sizes
 * @buffer_count:      Number of elements in the va and sizes buffers
 * @live_ranges:       Array of &struct kbase_pixel_gpu_slc_liveness_mark denoting live ranges for
 *                     each buffer
 * @live_ranges_count: Number of elements in the live ranges buffer
 */
struct gpu_slc_liveness_update_info {
	u64* buffer_sizes;
	u64 buffer_count;
	struct kbase_pixel_gpu_slc_liveness_mark* live_ranges;
	u64 live_ranges_count;
};

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
	s64 current_demand = 0, peak_demand = 0, old_demand;
	int i;

	dev_dbg(kbdev->dev, "pixel: buffer liveness update received");

	for (i = 0; i < info->live_ranges_count; ++i)
	{
		u64 size;
		u32 index = info->live_ranges[i].index;

		if (unlikely(index >= info->buffer_count))
			continue;

		size = info->buffer_sizes[index];

		switch (info->live_ranges[i].type)
		{
		case KBASE_PIXEL_GPU_LIVE_RANGE_BEGIN:
			/* Update demand as though there's no size limit */
			current_demand += size;
			peak_demand = max(peak_demand, current_demand);
			break;
		case KBASE_PIXEL_GPU_LIVE_RANGE_END:
			current_demand -= size;
			break;
		}
	}

	/* Indicates a missing live range end marker */
	WARN_ON_ONCE(current_demand != 0);

	/* Update the demand */
	old_demand = atomic_xchg(&kctx_pd->slc_demand, peak_demand);
	atomic_add(peak_demand - old_demand, &pc->slc_demand);
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
	int err = -EINVAL;
	struct gpu_slc_liveness_update_info info;
	u64* buff = NULL;
	u64 total_buff_size;

	/* Compute the sizes of the user space arrays that we need to copy */
	u64 const buffer_info_size = sizeof(u64) * update->buffer_count;
	u64 const live_ranges_size =
	    sizeof(struct kbase_pixel_gpu_slc_liveness_mark) * update->live_ranges_count;

	/* Guard against overflows and empty sizes */
	if (!buffer_info_size || !live_ranges_size)
		goto done;
	if (U64_MAX / sizeof(u64) < update->buffer_count)
		goto done;
	if (U64_MAX / sizeof(struct kbase_pixel_gpu_slc_liveness_mark) < update->live_ranges_count)
		goto done;
	/* Guard against nullptr */
	if (!update->live_ranges_address || !update->buffer_sizes_address)
		goto done;
	/* Calculate the total buffer size required and detect overflows */
	if ((U64_MAX - live_ranges_size) < buffer_info_size)
		goto done;

	total_buff_size = buffer_info_size + live_ranges_size;

	/* Allocate the memory we require to copy from user space */
	buff = kmalloc(total_buff_size, GFP_KERNEL);
	if (buff == NULL) {
		dev_err(kctx->kbdev->dev, "pixel: failed to allocate buffer for liveness update");
		err = -ENOMEM;
		goto done;
	}

	/* Set up the info struct by pointing into the allocation. All 8 byte aligned */
	info = (struct gpu_slc_liveness_update_info){
	    .buffer_sizes = buff,
	    .buffer_count = update->buffer_count,
	    .live_ranges = (struct kbase_pixel_gpu_slc_liveness_mark*)(buff + update->buffer_count),
	    .live_ranges_count = update->live_ranges_count,
	};

	/* Copy the data from user space */
	err = copy_from_user(
		info.live_ranges, u64_to_user_ptr(update->live_ranges_address), live_ranges_size);
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

	/* Execute an slc update */
	gpu_slc_liveness_update(kctx, &info);

done:
	kfree(buff);

	return err;
}
#endif /* PIXEL_GPU_SLC_ACPM_SIGNAL */

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
 */
void gpu_slc_kctx_term(struct kbase_context *kctx)
{
	struct pixel_platform_data *pd = kctx->platform_data;

	/* Contexts can be terminated without being idled first */
	if (transition(&pd->slc_vote, VOTING, IDLE))
		pixel_mgm_slc_dec_refcount(kctx->kbdev->mgm_dev);

#ifndef PIXEL_GPU_SLC_ACPM_SIGNAL
	{
		struct pixel_context* pc = kctx->kbdev->platform_context;
		/* Deduct the usage and demand, freeing that SLC space for the next update */
		u64 kctx_demand = atomic_xchg(&pd->slc_demand, 0);
		atomic_sub(kctx_demand, &pc->slc_demand);
	}
#endif /* PIXEL_GPU_SLC_ACPM_SIGNAL */
}

/**
 * gpu_slc_kctx_active() - Called when a kernel context is (re)activated
 *
 * @kctx: The &struct kbase_context that is now active
 */
void gpu_slc_kctx_active(struct kbase_context *kctx)
{
	struct pixel_platform_data *pd = kctx->platform_data;

	lockdep_assert_held(&kctx->kbdev->hwaccess_lock);

	if (transition(&pd->slc_vote, IDLE, VOTING))
		pixel_mgm_slc_inc_refcount(kctx->kbdev->mgm_dev);
}

/**
 * gpu_slc_kctx_idle() - Called when a kernel context is idled
 *
 * @kctx: The &struct kbase_context that is now idle
 */
void gpu_slc_kctx_idle(struct kbase_context *kctx)
{
	struct pixel_platform_data *pd = kctx->platform_data;

	lockdep_assert_held(&kctx->kbdev->hwaccess_lock);

	if (transition(&pd->slc_vote, VOTING, IDLE))
		pixel_mgm_slc_dec_refcount(kctx->kbdev->mgm_dev);
}

/**
 * gpu_slc_tick_tock() - Called when a GPU scheduling kick occurs
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_slc_tick_tock(struct kbase_device *kbdev)
{
#ifndef PIXEL_GPU_SLC_ACPM_SIGNAL
	struct pixel_context* pc = kbdev->platform_context;
	/* Threshold of 4MB */
	u64 signal = atomic_read(&pc->slc_demand) / (4 << 20);

	pixel_mgm_slc_update_signal(kbdev->mgm_dev, signal);
#else
	pixel_mgm_slc_update_signal(kbdev->mgm_dev, 0);
#endif /* PIXEL_GPU_SLC_ACPM_SIGNAL */
}

/**
 * gpu_slc_init - Initialize the SLC context for the GPU
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: On success, returns 0. On failure an error code is returned.
 */
int gpu_slc_init(struct kbase_device *kbdev)
{
	return 0;
}

/**
 * gpu_slc_term() - Terminates the Pixel GPU SLC context.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_slc_term(struct kbase_device *kbdev)
{
	(void)kbdev;
}
