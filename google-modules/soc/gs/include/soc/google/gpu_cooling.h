/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __GPU_COOLING_H__
#define __GPU_COOLING_H__

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/thermal.h>

#define GPU_TABLE_END     ~1

/**
 * struct gpu_tmu_notification_data - data to store TMU data for GPU driver
 *
 * @gpu_drv_data: Pointer to GPU driver data obtained from GPU DT entry.
 * @data:         Payload of this event.
 */
struct gpu_tmu_notification_data {
	void *gpu_drv_data;
	int data;
};

struct gpufreq_cooling_query_fns {
	int (*get_num_levels)(void *gpu_drv_data);
	int (*get_freqs_for_level)(void *gpu_drv_data, int level, int *clk0, int *clk1);
	int (*get_vols_for_level)(void *gpu_drv_data, int level, int *vol0, int *vol1);
	int (*get_cur_level)(void *gpu_drv_data);
	int (*get_cur_util)(void *gpu_drv_data);
};

#if IS_ENABLED(CONFIG_GPU_THERMAL)
struct thermal_cooling_device *gpufreq_cooling_register(
				struct device_node *np,
				void *gpu_drv_data, struct gpufreq_cooling_query_fns *gpu_fns);

void gpufreq_cooling_unregister(struct thermal_cooling_device *cdev);
#else
static inline struct thermal_cooling_device *gpufreq_cooling_register(
				struct device_node *np,
				void *gpu_drv_data, struct gpufreq_cooling_query_fns *gpu_fns)
{
	return NULL;
}
static inline void gpufreq_cooling_unregister(struct thermal_cooling_device *cdev)
{
}
#endif /* IS_ENABLED(CONFIG_GPU_THERMAL) */

#endif /* __GPU_COOLING_H__ */
