// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Linux includes */
#include <linux/pm_qos.h>

/* SOC includes */
#include <soc/google/tmu.h>
#include <soc/google/gpu_cooling.h>

/* Mali core includes */
#include <mali_kbase.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_dvfs.h"

/**
 * gpu_tmu_get_num_levels() - Returns the number of DVFS OPPs
 *
 * @gpu_drv_data: The data provided to the cooling driver via
 *                &gpufreq_cooling_register in &gpu_tmu_init.
 *
 * Return: The number of DVFS operating points.
 */
static int gpu_tmu_get_num_levels(void *gpu_drv_data)
{
	struct kbase_device *kbdev = gpu_drv_data;
	struct pixel_context *pc = kbdev->platform_context;

	return pc->dvfs.table_size;
}

/**
 * gpu_tmu_get_freqs_for_level() - Returns the frequencies for a DVFS OPP
 *
 * @gpu_drv_data: The data provided to the cooling driver via
 *                &gpufreq_cooling_register in &gpu_tmu_init.
 * @level: The level of the DVFS OPP table to query.
 * @clk0:  Pointer to write the gpu0 clock into. Set to NULL if not required.
 * @clk1:  Pointer to write the gpu1 clock into. Set to NULL if not required.
 *
 * Return: If an invalid level is provided, returns -1, otherwise 0. Values
 *         returned in &clk0 and &clk1 are in kHZ.
 */
static int gpu_tmu_get_freqs_for_level(void *gpu_drv_data, int level, int *clk0, int *clk1)
{
	struct kbase_device *kbdev = gpu_drv_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (level < 0 || level >= pc->dvfs.table_size)
		return -1;

	if (clk0)
		*clk0 = pc->dvfs.table[level].clk[GPU_DVFS_CLK_TOP_LEVEL];

	if (clk1)
		*clk1 = pc->dvfs.table[level].clk[GPU_DVFS_CLK_SHADERS];

	return 0;
}

/**
 * gpu_tmu_get_vols_for_level() - Returns the frequencies for a DVFS OPP
 *
 * @gpu_drv_data: The data provided to the cooling driver via
 *                &gpufreq_cooling_register in &gpu_tmu_init.
 * @level: The level of the DVFS OPP table to query.
 * @vol0:  Pointer to write the gpu0 voltage into. Set to NULL if not required.
 * @vol1:  Pointer to write the gpu1 voltage into. Set to NULL if not required.
 *
 * Return: If an invalid level is provided, returns -1, otherwise 0. Values
 *         returned in &vol0 and &vol1 are in mV.
 */
static int gpu_tmu_get_vols_for_level(void *gpu_drv_data, int level, int *vol0, int *vol1)
{
	struct kbase_device *kbdev = gpu_drv_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (level < 0 || level >= pc->dvfs.table_size)
		return -1;

	if (vol0)
		*vol0 = pc->dvfs.table[level].vol[GPU_DVFS_CLK_TOP_LEVEL];

	if (vol1)
		*vol1 = pc->dvfs.table[level].vol[GPU_DVFS_CLK_SHADERS];

	return 0;
}

/**
 * gpu_tmu_get_cur_level() - Returns current DVFS OPP level
 *
 * @gpu_drv_data: The data provided to the cooling driver via
 *                &gpufreq_cooling_register in &gpu_tmu_init.
 *
 * Context: Process context. Takes and releases the DVFS lock.
 *
 * Return: The current DVFS operating point level.
 */
static int gpu_tmu_get_cur_level(void *gpu_drv_data)
{
	struct kbase_device *kbdev = gpu_drv_data;
	struct pixel_context *pc = kbdev->platform_context;
	int level;

	mutex_lock(&pc->dvfs.lock);
	level = pc->dvfs.level;
	mutex_unlock(&pc->dvfs.lock);

	return level;
}

/**
 * gpu_tmu_get_cur_util() - Returns the utilization of the GPU
 *
 * @gpu_drv_data: The data provided to the cooling driver via
 *                &gpufreq_cooling_register in &gpu_tmu_init.
 *
 * Return: The utilization level of the GPU. This is an integer percentage.
 */
static int gpu_tmu_get_cur_util(void *gpu_drv_data)
{
	struct kbase_device *kbdev = gpu_drv_data;
	struct pixel_context *pc = kbdev->platform_context;
	int util = 0;

	if (gpu_pm_get_power_state(kbdev))
		util = atomic_read(&pc->dvfs.util);

	return util;
}

static struct gpufreq_cooling_query_fns tmu_query_fns = {
	.get_num_levels = &gpu_tmu_get_num_levels,
	.get_freqs_for_level = &gpu_tmu_get_freqs_for_level,
	.get_vols_for_level = &gpu_tmu_get_vols_for_level,
	.get_cur_level = &gpu_tmu_get_cur_level,
	.get_cur_util = &gpu_tmu_get_cur_util
};

/**
 * get_level_from_tmu_data() - Translates GPU cooling data to a target DVFS level
 *
 * @gpu_drv_data: The data provided to the cooling driver via
 *                &gpufreq_cooling_register in &gpu_tmu_init.
 * @data:         Integer value passed by the GPU cooling driver.
 *
 * Return: The target DVFS operating point level indicated by the GPU cooling
 *         driver.
 *
 * This function is written to work with data known to be provided by the GPU
 * cooling device on GS101 which is a target OPP level. This function simply
 * validates that this is a valid level.
 */
static int get_level_from_tmu_data(void *gpu_drv_data, int data)
{
	struct kbase_device *kbdev = gpu_drv_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (data >= 0 && data < pc->dvfs.table_size)
		return data;

	return -1;
}

/**
 * gpu_tmu_notifier() - Processes incoming TMU notifications.
 *
 * @notifier: The &struct notifier_block. Currently unused.
 * @event:    Event id.
 * @v:        Notification block struct.
 *
 * Context: Process context. Takes and releases the DVFS lock.
 *
 * Return: NOTIFY_OK on a valid event. NOTIFY_BAD if the notification data is
 *         invalid and the GPU driver intends to veto the action.
 */
static int gpu_tmu_notifier(struct notifier_block *notifier, unsigned long event, void *v)
{
	struct gpu_tmu_notification_data *nd = v;
	struct kbase_device *kbdev = nd->gpu_drv_data;
	struct pixel_context *pc = kbdev->platform_context;
	int level;

	switch (event) {
	case GPU_COLD:
		dev_dbg(kbdev->dev, "%s: GPU_COLD event received\n", __func__);
		level = pc->dvfs.level_max;
		break;
	case GPU_NORMAL:
		dev_dbg(kbdev->dev, "%s: GPU_NORMAL event received\n", __func__);
		level = pc->dvfs.level_max;
		break;
	case GPU_THROTTLING:
		level = get_level_from_tmu_data(kbdev, nd->data);
		if (level < 0) {
			dev_warn(kbdev->dev,
				"%s: GPU_THROTTLING event received with invalid level: %d\n",
				__func__, nd->data);
			return NOTIFY_BAD;
		}
		dev_info(kbdev->dev,
			"%s: Adjusting GPU clock to %d kHz for thermal constraints (this is normal)\n",
			__func__, pc->dvfs.table[level].clk[GPU_DVFS_CLK_SHADERS]);
		break;
	default:
		dev_warn(kbdev->dev, "%s: Unexpected TMU event received\n", __func__);
		goto done;
	}

	/* Update the TMU lock level */
	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_update_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_THERMAL, -1, level);
	gpu_dvfs_select_level(kbdev);
	mutex_unlock(&pc->dvfs.lock);

done:
	return NOTIFY_OK;
}

static struct notifier_block gpu_tmu_nb = {
	.notifier_call = gpu_tmu_notifier,
};

/**
 * gpu_tmu_init() - Initializes the Pixel TMU handling subsystem
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: Currently always returns 0.
 */
int gpu_tmu_init(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct device_node *np = kbdev->dev->of_node;
	struct thermal_cooling_device *dev;

	dev = gpufreq_cooling_register(np, kbdev, &tmu_query_fns);

	if (IS_ERR(dev)) {
		dev_err(kbdev->dev,
			"%s: Error when registering gpu as a cooling device\n", __func__);
		return PTR_ERR(dev);
	}

	gpufreq_cooling_add_notifier(&gpu_tmu_nb);
	pc->dvfs.tmu.cdev = dev;

	return 0;
}

/**
 * gpu_tmu_term() - Terminates the Pixel GPU TMU handling subsystem.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_tmu_term(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	gpufreq_cooling_remove_notifier(&gpu_tmu_nb);
	gpufreq_cooling_unregister(pc->dvfs.tmu.cdev);
}

