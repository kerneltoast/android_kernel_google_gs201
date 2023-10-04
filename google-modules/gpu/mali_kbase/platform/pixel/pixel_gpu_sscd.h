/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */

#ifndef _PIXEL_GPU_SSCD_H_
#define _PIXEL_GPU_SSCD_H_

#include <mali_kbase.h>

#ifdef CONFIG_MALI_PIXEL_GPU_SSCD
int gpu_sscd_fw_log_init(struct kbase_device *kbdev, u32 level);

int gpu_sscd_init(struct kbase_device *kbdev);

void gpu_sscd_term(struct kbase_device *kbdev);

void gpu_sscd_dump(struct kbase_device *kbdev, const char* reason);
#else
static int __maybe_unused gpu_sscd_fw_log_init(struct kbase_device *kbdev, u32 level)
{
	return (void)kbdev, (void)level, 0;
}

static int __maybe_unused gpu_sscd_init(struct kbase_device *kbdev) { return (void)kbdev, 0; }

static void __maybe_unused gpu_sscd_term(struct kbase_device *kbdev) { (void)kbdev; }

static void __maybe_unused gpu_sscd_dump(struct kbase_device *kbdev, const char* reason)
{
	(void)kbdev, (void)reason;
}
#endif /* CONFIG_MALI_PIXEL_GPU_SSCD */

#endif /* _PIXEL_GPU_SSCD_H_ */
