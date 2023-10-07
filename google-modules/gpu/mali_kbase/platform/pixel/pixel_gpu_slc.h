// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022-2023 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */
#ifndef _PIXEL_GPU_SLC_H_
#define _PIXEL_GPU_SLC_H_

#ifdef CONFIG_MALI_PIXEL_GPU_SLC
int gpu_pixel_handle_buffer_liveness_update_ioctl(struct kbase_context* kctx,
                                                  struct kbase_ioctl_buffer_liveness_update* update);

int gpu_slc_init(struct kbase_device *kbdev);

void gpu_slc_term(struct kbase_device *kbdev);

int gpu_slc_kctx_init(struct kbase_context *kctx);

void gpu_slc_kctx_term(struct kbase_context *kctx);
#else
static int __maybe_unused gpu_pixel_handle_buffer_liveness_update_ioctl(struct kbase_context* kctx,
                                                  struct kbase_ioctl_buffer_liveness_update* update)
{
	return (void)kctx, (void)update, 0;
}

int __maybe_unused gpu_slc_init(struct kbase_device *kbdev) { return (void)kbdev, 0; }

void __maybe_unused gpu_slc_term(struct kbase_device *kbdev) { (void)kbdev; }

static int __maybe_unused gpu_slc_kctx_init(struct kbase_context *kctx) { return (void)kctx, 0; }

static void __maybe_unused gpu_slc_kctx_term(struct kbase_context* kctx) { (void)kctx; }
#endif /* CONFIG_MALI_PIXEL_GPU_SLC */

#endif /* _PIXEL_GPU_SLC_H_ */
