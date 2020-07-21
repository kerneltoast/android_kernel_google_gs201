/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __GPU_COOLING_H__
#define __GPU_COOLING_H__

#include <linux/of.h>
#include <linux/thermal.h>

#define GPU_TABLE_END     ~1

#if IS_ENABLED(CONFIG_GPU_THERMAL)
#if IS_ENABLED(CONFIG_MALI_DVFS)
int gpu_dvfs_get_clock(int level);
int gpu_dvfs_get_voltage(int clock);
int gpu_dvfs_get_step(void);
int gpu_dvfs_get_cur_clock(void);
int gpu_dvfs_get_utilization(void);
int gpu_dvfs_get_max_freq(void);
#else
static inline int gpu_dvfs_get_clock(int level) { return 0; }
static inline int gpu_dvfs_get_voltage(int clock) { return 0; }
static inline int gpu_dvfs_get_step(void) { return 0; }
static inline int gpu_dvfs_get_cur_clock(void) { return 0; }
static inline int gpu_dvfs_get_utilization(void) { return 0; }
static inline int gpu_dvfs_get_max_freq(void) { return 0; }
#endif
#endif
#endif /* __GPU_COOLING_H__ */
