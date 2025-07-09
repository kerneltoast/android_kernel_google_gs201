/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

#ifndef _PIXEL_GPU_CONTROL_H_
#define _PIXEL_GPU_CONTROL_H_

/* Power management */
#ifdef CONFIG_MALI_PIXEL_GPU_PM
bool gpu_pm_get_power_state_nolock(struct kbase_device *kbdev);
bool gpu_pm_get_power_state(struct kbase_device *kbdev);
int gpu_pm_init(struct kbase_device *kbdev);
void gpu_pm_term(struct kbase_device *kbdev);
void* gpu_pm_get_rail_state_log(struct kbase_device *kbdev);
unsigned int gpu_pm_get_rail_state_log_size(struct kbase_device *kbdev);
#else
static bool __maybe_unused gpu_pm_get_power_state(struct kbase_device *kbdev) { return true; }
static int __maybe_unused gpu_pm_init(struct kbase_device *kbdev) { return 0; }
static void __maybe_unused gpu_pm_term(struct kbase_device *kbdev) {}
static void* __maybe_unused gpu_pm_get_rail_state_log(struct kbase_device *kbdev) { return NULL; }
static unsigned int __maybe_unused gpu_pm_get_rail_state_log_size(struct kbase_device *kbdev) {return 0; }
#endif

/* DVFS */
#ifdef CONFIG_MALI_MIDGARD_DVFS
void gpu_dvfs_event_power_on(struct kbase_device *kbdev);
void gpu_dvfs_event_power_off(struct kbase_device *kbdev);

int gpu_dvfs_init(struct kbase_device *kbdev);
void gpu_dvfs_term(struct kbase_device *kbdev);
void gpu_dvfs_disable_updates(struct kbase_device *kbdev);
void gpu_dvfs_enable_updates(struct kbase_device *kbdev);
#else
static int __maybe_unused gpu_dvfs_init(struct kbase_device *kbdev) { return 0; }
static void __maybe_unused gpu_dvfs_term(struct kbase_device *kbdev) {}
static void __maybe_unused gpu_dvfs_disable_updates(struct kbase_device *kbdev) {}
static void __maybe_unused gpu_dvfs_enable_updates(struct kbase_device *kbdev) {}
#endif

/* sysfs */
#ifdef CONFIG_MALI_MIDGARD_DVFS
int gpu_sysfs_init(struct kbase_device *kbdev);
void gpu_sysfs_term(struct kbase_device *kbdev);
#else
static int __maybe_unused gpu_sysfs_init(struct kbase_device *kbdev) { return 0; }
static void __maybe_unused gpu_sysfs_term(struct kbase_device *kbdev) {}
#endif

/* Kernel context callbacks */
#ifdef CONFIG_MALI_MIDGARD_DVFS
int gpu_dvfs_kctx_init(struct kbase_context *kctx);
void gpu_dvfs_kctx_term(struct kbase_context *kctx);
#endif

/* ITMON notifier */
#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
int gpu_itmon_init(struct kbase_device *kbdev);
void gpu_itmon_term(struct kbase_device *kbdev);
#endif

/**
 * pixel_gpu_debugfs_init() - Create a debugfs entry for the Pixel integration
 *
 * @kbdev: Pointer to the GPU device for which to create the debugfs entry
 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
void pixel_gpu_debugfs_init(struct kbase_device *kbdev);
#else
static inline void __maybe_unused pixel_gpu_debugfs_init(struct kbase_device *kbdev) {}
#endif /* CONFIG_DEBUG_FS */


#endif /* _PIXEL_GPU_CONTROL_H_ */
