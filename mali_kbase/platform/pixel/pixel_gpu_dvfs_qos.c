// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Linux includes */
#include <linux/of.h>

/* SOC includes */
#ifdef CONFIG_MALI_PIXEL_GPU_BTS
#include <soc/google/bts.h>
#endif

/* Mali core includes */
#include <mali_kbase.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_dvfs.h"

/**
 * qos_reset() - Resets a QOS vote on an IP block
 *
 * @vote:  The &struct gpu_dvfs_qos_vote to reset the vote on.
 *
 * This function resets the GPU's vote for the given IP block to
 * &EXYNOS_PM_QOS_DEFAULT_VALUE
 */
static inline void qos_reset(struct gpu_dvfs_qos_vote *vote) {
	if (unlikely(vote->enabled)) {
		exynos_pm_qos_update_request_async(&vote->req, EXYNOS_PM_QOS_DEFAULT_VALUE);
		vote->enabled = false;
	}
}

/**
 * qos_set() - Set a QOS vote on an IP block
 *
 * @vote:  The &struct gpu_dvfs_qos_vote to set the vote on.
 * @value: The value to vote for.
 */
static inline void qos_set(struct gpu_dvfs_qos_vote *vote, int value) {
	if (unlikely(value)) {
		exynos_pm_qos_update_request_async(&vote->req, value);
		vote->enabled = true;
	}
	else {
		qos_reset(vote);
	}
}

/**
 * gpu_dvfs_qos_set() - Issue QOS requests for a GPU DVFS level.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 * @level: The DVFS level from which to retrieve QOS values from.
 *
 * Context: Process context. Expects caller to hold the DVFS lock.
 */
void gpu_dvfs_qos_set(struct kbase_device *kbdev, int level)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_opp opp = pc->dvfs.table[level];

	lockdep_assert_held(&pc->dvfs.lock);

	if (pc->dvfs.qos.level_last != level) {

		dev_dbg(kbdev->dev,
			"QOS int_min:  %d\n"
			"QOS mif_min:  %d\n"
			"QOS cpu0_min: %d\n"
			"QOS cpu1_min: %d\n"
			"QOS cpu2_max: %d\n",
			opp.qos.int_min, opp.qos.mif_min, opp.qos.cpu0_min,
			opp.qos.cpu1_min, opp.qos.cpu2_max);

		qos_set(&pc->dvfs.qos.int_min,  opp.qos.int_min);
		qos_set(&pc->dvfs.qos.mif_min,  opp.qos.mif_min);
		qos_set(&pc->dvfs.qos.cpu0_min, opp.qos.cpu0_min);
		qos_set(&pc->dvfs.qos.cpu1_min, opp.qos.cpu1_min);
		qos_set(&pc->dvfs.qos.cpu2_max, opp.qos.cpu2_max);

#ifdef CONFIG_MALI_PIXEL_GPU_BTS
		if (opp.clk[GPU_DVFS_CLK_SHADERS] >= pc->dvfs.qos.bts.threshold &&
			!pc->dvfs.qos.bts.enabled) {
			bts_add_scenario(pc->dvfs.qos.bts.scenario);
			pc->dvfs.qos.bts.enabled = true;
		} else if (pc->dvfs.qos.bts.enabled &&
				opp.clk[GPU_DVFS_CLK_SHADERS] < pc->dvfs.qos.bts.threshold) {
			bts_del_scenario(pc->dvfs.qos.bts.scenario);
			pc->dvfs.qos.bts.enabled = false;
		}
#endif /* CONFIG_MALI_PIXEL_GPU_BTS */

		pc->dvfs.qos.level_last = level;
		pc->dvfs.qos.enabled = true;
	}
}

/**
 * gpu_dvfs_qos_reset() - Clears QOS requests.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Context: Process context. Expects caller to hold the DVFS lock.
 */
void gpu_dvfs_qos_reset(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	lockdep_assert_held(&pc->dvfs.lock);

	qos_reset(&pc->dvfs.qos.int_min);
	qos_reset(&pc->dvfs.qos.mif_min);
	qos_reset(&pc->dvfs.qos.cpu0_min);
	qos_reset(&pc->dvfs.qos.cpu1_min);
	qos_reset(&pc->dvfs.qos.cpu2_max);

#ifdef CONFIG_MALI_PIXEL_GPU_BTS
	if (pc->dvfs.qos.bts.enabled) {
		bts_del_scenario(pc->dvfs.qos.bts.scenario);
		pc->dvfs.qos.bts.enabled = false;
	}
#endif /* CONFIG_MALI_PIXEL_GPU_BTS */

	pc->dvfs.qos.level_last = -1;
	pc->dvfs.qos.enabled = false;
}

/**
 * gpu_dvfs_qos_init() - Initializes the Pixel GPU DVFS QOS subsystem.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: On success, returns 0. -EINVAL on error.
 */
int gpu_dvfs_qos_init(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	int ret;

#ifdef CONFIG_MALI_PIXEL_GPU_BTS
	struct device_node *np = kbdev->dev->of_node;
	const char *bts_scenario_name;

	pc->dvfs.qos.bts.enabled = false;

	if (of_property_read_string(np, "gpu_dvfs_qos_bts_scenario", &bts_scenario_name)) {
		dev_err(kbdev->dev, "GPU QOS BTS scenario not specified in DT\n");
		ret = -EINVAL;
		goto done;
	}

	pc->dvfs.qos.bts.scenario = bts_get_scenindex(bts_scenario_name);
	if (!pc->dvfs.qos.bts.scenario) {
		dev_err(kbdev->dev, "invalid GPU QOS BTS scenario specified in DT\n");
		ret = -EINVAL;
		goto done;
	}

	if (of_property_read_u32(np, "gpu_dvfs_qos_bts_threshold", &pc->dvfs.qos.bts.threshold)) {
		dev_err(kbdev->dev, "GPU QOS BTS threshold not specified in DT\n");
		ret = -EINVAL;
		goto done;
	}
#endif /* CONFIG_MALI_PIXEL_GPU_BTS */

	exynos_pm_qos_add_request(&pc->dvfs.qos.int_min.req,  PM_QOS_DEVICE_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&pc->dvfs.qos.mif_min.req,  PM_QOS_BUS_THROUGHPUT,    0);
	exynos_pm_qos_add_request(&pc->dvfs.qos.cpu0_min.req, PM_QOS_CLUSTER0_FREQ_MIN, 0);
	exynos_pm_qos_add_request(&pc->dvfs.qos.cpu1_min.req, PM_QOS_CLUSTER1_FREQ_MIN, 0);
	exynos_pm_qos_add_request(&pc->dvfs.qos.cpu2_max.req, PM_QOS_CLUSTER2_FREQ_MAX,
		PM_QOS_CLUSTER2_FREQ_MAX_DEFAULT_VALUE);

	pc->dvfs.qos.level_last = -1;
	pc->dvfs.qos.enabled = false;

	dev_dbg(kbdev->dev, "GPU QOS initialized\n");
	ret = 0;

#ifdef CONFIG_MALI_PIXEL_GPU_BTS
done:
#endif /* CONFIG_MALI_PIXEL_GPU_BTS */
	return ret;
}

/**
 * gpu_dvfs_qos_term() - Terminates the Pixel GPU DVFS QOS subsystem.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_dvfs_qos_term(struct kbase_device *kbdev)
{
#if IS_ENABLED(CONFIG_EXYNOS_PMU_IF) || defined(CONFIG_MALI_PIXEL_GPU_BTS)
	struct pixel_context *pc = kbdev->platform_context;
#endif

	exynos_pm_qos_remove_request(&pc->dvfs.qos.int_min.req);
	exynos_pm_qos_remove_request(&pc->dvfs.qos.mif_min.req);
	exynos_pm_qos_remove_request(&pc->dvfs.qos.cpu0_min.req);
	exynos_pm_qos_remove_request(&pc->dvfs.qos.cpu1_min.req);
	exynos_pm_qos_remove_request(&pc->dvfs.qos.cpu2_max.req);

#ifdef CONFIG_MALI_PIXEL_GPU_BTS
	if (pc->dvfs.qos.bts.enabled) {
		bts_del_scenario(pc->dvfs.qos.bts.scenario);
		pc->dvfs.qos.bts.enabled = false;
	}
#endif /* CONFIG_MALI_PIXEL_GPU_BTS */
}
