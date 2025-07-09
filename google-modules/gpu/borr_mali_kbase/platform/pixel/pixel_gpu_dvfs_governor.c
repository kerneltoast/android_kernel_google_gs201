// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Mali core includes */
#include <mali_kbase.h>
#include <trace/events/power.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_dvfs.h"
#include "pixel_gpu_trace.h"

/**
 * gpu_dvfs_governor_basic() - The evaluation function for &GPU_DVFS_GOVERNOR_BASIC.
 *
 * @kbdev:      The &struct kbase_device for the GPU.
 * @util_stats: The current GPU utilization statistics.
 *
 * Return: The level that the GPU should run at next.
 *
 * Context: Process context. Expects the caller to hold the DVFS lock.
 */
static int gpu_dvfs_governor_basic(struct kbase_device *kbdev,
	struct gpu_dvfs_utlization *util_stats)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_opp *tbl = pc->dvfs.table;
	int level = pc->dvfs.level;
	int level_max = pc->dvfs.level_max;
	int level_min = pc->dvfs.level_min;
	int util = util_stats->util;

	lockdep_assert_held(&pc->dvfs.lock);

	if ((level > level_max) && (util > tbl[level].util_max)) {
		/* Need to clock up*/
		level--;

		/* Reset hysteresis */
		pc->dvfs.governor.delay = tbl[level].hysteresis;

	} else if ((level < level_min) && (util < tbl[level].util_min)) {
		/* We are clocked too high */
		pc->dvfs.governor.delay--;

		/* Check if we've resisted downclocking long enough */
		if (pc->dvfs.governor.delay == 0) {
			/* Time to clock down */
			level++;

			/* Reset hysteresis */
			pc->dvfs.governor.delay = tbl[level].hysteresis;
		}
	} else {
		/* We are at the correct level, reset hysteresis */
		pc->dvfs.governor.delay = tbl[level].hysteresis;
	}

	return level;
}

#if MALI_USE_CSF
/**
 * level_for_capacity() - Find the lowest level satisfying a given needed capacity.
 *
 * @capacity:  The capacity desired, in whatever units the clocks for levels are in
 * @tbl:       The DVFS operating points to choose from
 * @dev:       The device node, for debug logs.
 * @level_min: The index of the lowest allowable operating point
 * @level_max: The index of the highest allowable operating point
 *
 * Return: The index of the operating point found, or level_max if no operating
 *         point has enough capacity.
 */
static int
level_for_capacity(u32 capacity, struct gpu_dvfs_opp *tbl, struct device *dev,
		   int level_min, int level_max)
{
	int l;

	for (l = level_min; l >= level_max; --l) {
		if ((u32)tbl[l].clk[1] >= capacity) {
			dev_dbg(dev,
				"DVFS needs capacity %u. "
				"Setting max freq %u",
				capacity,
				tbl[l].clk[1]);
			return l;
		}
	}

	dev_dbg(dev,
		"DVFS measured use exceeded maximum capacity."
		"Setting max freq %u",
		tbl[level_max].clk[1]);

	return level_max;
}

/**
 * gpu_dvfs_governor_quickstep_use_mcu_util() - The evaluation function for &GPU_DVFS_GOVERNOR_QUICKSTEP_USE_MCU.
 *
 * @kbdev:      The &struct kbase_device for the GPU.
 * @util_stats: The current GPU utilization statistics.
 *
 * Algorithm:
 *   * If we are within the utilization bounds of the current level then
 *     no change is made.
 *
 *   * If &util or &mcu_util is above the maximum for the current level we calculate how much
 *     above the maximum we are. &util is higher closer to 100% than it is to
 *     the maximum utilization for the current level then we move up &step_up levels.
 *     We also move up &step_up levels if the &mcu_util is more than 25% over
 *     &mcu_up_util of that particular level.
 *     Otherwise we move up just a single level. If we skip a level, we also
 *     halve the hysteresis for the new level, so that we can swiftly correct
 *     overshoots.
 *
 *   * If &util or &mcu_util is lower than the minimm utilization for the current level, then
 *     we decrement the hysteresis value. If this decrement results in
 *     hysteresis being zero, then we drop a level.
 *
 *   * Adjust the target frequency for capacity_headroom.
 *
 * Return: The level that the GPU should run at next.
 *
 * Context: Process context. Expects the caller to hold the DVFS lock.
 */
static int
gpu_dvfs_governor_quickstep_use_mcu_util(struct kbase_device *kbdev,
					 struct gpu_dvfs_utlization *util_stats)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_opp *tbl = pc->dvfs.table;
	int level = pc->dvfs.level_before_headroom;
	int level_max = pc->dvfs.level_max;
	int level_min = pc->dvfs.level_min;
	int util = util_stats->util;
	int mcu_util = util_stats->mcu_util;
	int step_up = pc->dvfs.step_up_val;
	int mcu_scale_num = pc->dvfs.tunable.mcu_down_util_scale_num;
	int mcu_scale_den = pc->dvfs.tunable.mcu_down_util_scale_den;

	lockdep_assert_held(&pc->dvfs.lock);

	if ((level > level_max) && (util > tbl[level].util_max ||
				    mcu_util > tbl[level].mcu_util_max)) {
		/* We need to clock up. */
		if (level >= step_up &&
		    ((util > (100 + tbl[level].util_max) / 2) ||
		     mcu_util > (mcu_scale_num *
				 (tbl[level].mcu_util_max / mcu_scale_den)))) {
			dev_dbg(kbdev->dev,
				"DVFS +%d: %d -> %d (util: %d / %d | mcu: %d / %d)\n",
				step_up, level, level - step_up, util,
				tbl[level].util_max, mcu_util,
				tbl[level].mcu_util_max);
			level -= step_up;
			pc->dvfs.governor.delay = tbl[level].hysteresis / 2;
		} else {
			dev_dbg(kbdev->dev,
				"DVFS +1: %d -> %d (util: %d / %d mcu: %d / %d) \n",
				level, level - 1, util, tbl[level].util_max,
				mcu_util, tbl[level].mcu_util_max);
			level -= 1;
			pc->dvfs.governor.delay = tbl[level].hysteresis;
		}

	} else if ((level < level_min) && (util < tbl[level].util_min) &&
		   (mcu_util < tbl[level].mcu_util_min)) {
		/* We are clocked too high */
		pc->dvfs.governor.delay--;

		/* Check if we've resisted downclocking long enough */
		if (pc->dvfs.governor.delay <= 0) {
			dev_dbg(kbdev->dev,
				"DVFS -1: %d -> %d (util: %d / %d mcu: %d / %d)\n",
				level, level + 1, util, tbl[level].util_min,
				mcu_util, tbl[level].mcu_util_min);

			/* Time to clock down */
			level++;

			/* Reset hysteresis */
			pc->dvfs.governor.delay = tbl[level].hysteresis;
		}
	} else {
		/* We are at the correct level, reset hysteresis */
		pc->dvfs.governor.delay = tbl[level].hysteresis;
	}

	pc->dvfs.level_before_headroom = level;

	if (pc->dvfs.capacity_headroom != 0)
	{
		u32 capacity = tbl[level].clk[1];
		capacity += pc->dvfs.capacity_headroom;
		return level_for_capacity(capacity, tbl, kbdev->dev, level_min, level_max);
	}
	else
	{
		/**
		 * It's conceivable that the governor might choose an operating
		 * point with the same core clock rate but higher QoS votes, so
		 * respect the exact level chosen rather than doing a lookup in
		 * the table solely based on capacity.
		 **/
		return level;
	}
}

/**
 * gpu_dvfs_governor_capacity_use_mcu_util() - The evaluation function for &GPU_DVFS_GOVERNOR_CAPACITY_USE_MCU.
 *
 * @kbdev:      The &struct kbase_device for the GPU.
 * @util_stats: The current GPU utilization statistics.
 *
 * Algorithm:
 *   * If we are above 95% capacity at the current level, move to the highest
 *     operating point.
 *   * Otherwise, find the maximum capacity used in the last
 *     capacity_history_depth DVFS intervals.
 *   * Add capacity_headroom.
 *   * Choose the lowest operating point that has that capacity.
 *
 * Return: The level that the GPU should run at next.
 *
 * Context: Process context. Expects the caller to hold the DVFS lock.
 */
static int
gpu_dvfs_governor_capacity_use_mcu_util(struct kbase_device *kbdev,
					 struct gpu_dvfs_utlization *util_stats)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_opp *tbl = pc->dvfs.table;
	int level = pc->dvfs.level;
	int level_max = pc->dvfs.level_max;
	int level_min = pc->dvfs.level_min;
	u32 capacity_target = 0;
	u64 util = util_stats->util < 0 ? 0 : util_stats->util;
	u64 mcu_util = util_stats->mcu_util < 0 ? 0 : util_stats->mcu_util;
	u64 total_util = util + mcu_util;

	{
		u64 capacity_used = (u64)tbl[level].clk[1] * total_util / 100ul;
		pc->dvfs.capacity_history[pc->dvfs.capacity_history_index] = (u32)capacity_used;
		pc->dvfs.capacity_history_index++;
		pc->dvfs.capacity_history_index %= ARRAY_SIZE(pc->dvfs.capacity_history);
	}

	lockdep_assert_held(&pc->dvfs.lock);

	if (total_util > 95) {
		dev_dbg(kbdev->dev,
			"DVFS load exceeds measurable levels. "
			"Setting max freq %u",
			tbl[level_max].clk[1]);
		return level_max;
	}

	{
		int h;

		for (h = 0; h < ARRAY_SIZE(pc->dvfs.capacity_history); ++h) {
			if (capacity_target < pc->dvfs.capacity_history[h]) {
				capacity_target = pc->dvfs.capacity_history[h];
			}
		}

		capacity_target += pc->dvfs.capacity_headroom;
	}

	return level_for_capacity(capacity_target, tbl, kbdev->dev, level_min, level_max);
}
#endif

/**
 * gpu_dvfs_governor_quickstep() - The evaluation function for &GPU_DVFS_GOVERNOR_QUICKSTEP.
 *
 * @kbdev:      The &struct kbase_device for the GPU.
 * @util_stats: The current GPU utilization statistics.
 *
 * Algorithm:
 *   * If we are within the utilization bounds of the current level then
 *     no change is made.
 *
 *   * If &util is above the maximum for the current level we calculate how much
 *     above the maximum we are. &util is higher closer to 100% than it is to
 *     the maximum utilization for the current level then we move up two levels.
 *     Otherwise we move up just a single level. If we skip a level, we also
 *     halve the hysteresis for the new level, so that we can swiftly correct
 *     overshoots.
 *
 *   * If &util is lower than the minimm utilization for the current level, then
 *     we decrement the hysteresis value. If this decrement results in
 *     hysteresis being zero, then we drop a level.
 *
 * Return: The level that the GPU should run at next.
 *
 * Context: Process context. Expects the caller to hold the DVFS lock.
 */
static int gpu_dvfs_governor_quickstep(struct kbase_device *kbdev,
				       struct gpu_dvfs_utlization *util_stats)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_opp *tbl = pc->dvfs.table;
	int level = pc->dvfs.level;
	int level_max = pc->dvfs.level_max;
	int level_min = pc->dvfs.level_min;
	int util = util_stats->util;
	int step_up = pc->dvfs.step_up_val;

	lockdep_assert_held(&pc->dvfs.lock);

	if ((level > level_max) && (util > tbl[level].util_max)) {
		/* We need to clock up. */
		if (level >= step_up && (util > (100 + tbl[level].util_max) / 2)) {
			dev_dbg(kbdev->dev, "DVFS +%d: %d -> %d (u: %d / %d)\n",
				step_up, level, level - step_up, util, tbl[level].util_max);
			level -= step_up;
			pc->dvfs.governor.delay = tbl[level].hysteresis / 2;
		} else {
			dev_dbg(kbdev->dev, "DVFS +1: %d -> %d (u: %d / %d)\n",
				level, level - 1, util, tbl[level].util_max);
			level -= 1;
			pc->dvfs.governor.delay = tbl[level].hysteresis;
		}

	} else if ((level < level_min) && (util < tbl[level].util_min)) {
		/* We are clocked too high */
		pc->dvfs.governor.delay--;

		/* Check if we've resisted downclocking long enough */
		if (pc->dvfs.governor.delay <= 0) {
			dev_dbg(kbdev->dev, "DVFS -1: %d -> %d (u: %d / %d)\n",
				level, level + 1, util, tbl[level].util_min);

			/* Time to clock down */
			level++;

			/* Reset hysteresis */
			pc->dvfs.governor.delay = tbl[level].hysteresis;
		}
	} else {
		/* We are at the correct level, reset hysteresis */
		pc->dvfs.governor.delay = tbl[level].hysteresis;
	}

	return level;
}

static struct gpu_dvfs_governor_info governors[GPU_DVFS_GOVERNOR_COUNT] = {
	{
		"basic",
		gpu_dvfs_governor_basic,
	},
	{
		"quickstep",
		gpu_dvfs_governor_quickstep,
	},
#if MALI_USE_CSF
	{
		"quickstep_use_mcu",
		gpu_dvfs_governor_quickstep_use_mcu_util,
	},
	{
		"capacity_use_mcu",
		gpu_dvfs_governor_capacity_use_mcu_util,
	},
#endif
};

/**
 * gpu_dvfs_governor_get_next_level() - Requests the current governor to suggest the next level.
 *
 * @kbdev:      The &struct kbase_device for the GPU.
 * @util_stats: Pointer to a &struct gpu_dvfs_utlization storing current GPU utilization statistics.
 *
 * This function calls into the currently enabled DVFS governor to determine the next GPU operating
 * point. It also ensures that the recommended level conforms to any extant level locks.
 *
 * Return: Returns the level the GPU should run at.
 *
 * Context: Process context. Expects the caller to hold the DVFS lock.
 */
int gpu_dvfs_governor_get_next_level(struct kbase_device *kbdev,
	struct gpu_dvfs_utlization *util_stats)
{
	struct pixel_context *pc = kbdev->platform_context;
	int level, ret;

	lockdep_assert_held(&pc->dvfs.lock);
	level = governors[pc->dvfs.governor.curr].evaluate(kbdev, util_stats);
	if (level != pc->dvfs.level) {
		trace_clock_set_rate("gpu_gov_rec", pc->dvfs.table[level].clk[GPU_DVFS_CLK_SHADERS],
			raw_smp_processor_id());
	}

	ret = clamp(level, pc->dvfs.level_scaling_max, pc->dvfs.level_scaling_min);
	if (ret != level) {
		trace_gpu_gov_rec_violate(pc->dvfs.table[level].clk[GPU_DVFS_CLK_SHADERS],
			pc->dvfs.table[ret].clk[GPU_DVFS_CLK_SHADERS],
			pc->dvfs.table[pc->dvfs.level_scaling_min].clk[GPU_DVFS_CLK_SHADERS],
			pc->dvfs.table[pc->dvfs.level_scaling_max].clk[GPU_DVFS_CLK_SHADERS]);
	}

	return ret;
}

/**
 * gpu_dvfs_governor_set_governor() - Sets the currently active DVFS governor.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 * @gov:   &enum gpu_dvfs_governor value of the governor to set.
 *
 * Return: On success returns 0. If @gov is invalid, -EINVAL is returned.
 *
 * Context: Expects the caller to hold the DVFS lock.
 */
int gpu_dvfs_governor_set_governor(struct kbase_device *kbdev, enum gpu_dvfs_governor_type gov)
{
	struct pixel_context *pc = kbdev->platform_context;

	lockdep_assert_held(&pc->dvfs.lock);

	if (gov < 0 || gov >= GPU_DVFS_GOVERNOR_COUNT) {
		dev_warn(kbdev->dev, "Attempted to set invalid DVFS governor\n");
		return -EINVAL;
	}

	pc->dvfs.governor.curr = gov;

	return 0;
}

/**
 * gpu_dvfs_governor_get_id() - Given a valid governor name, returns its ID.
 *
 * @name:  A string contrining the name of the governor.
 *
 * Return: the &enum gpu_dvfs_governor_type for @name. If not found, returns
 *         &GPU_DVFS_GOVERNOR_INVALID.
 */
enum gpu_dvfs_governor_type gpu_dvfs_governor_get_id(const char *name)
{
	int i;

	/* We use sysfs_streq here as name may be a sysfs input string */
	for (i = 0; i < GPU_DVFS_GOVERNOR_COUNT; i++)
		if (sysfs_streq(name, governors[i].name))
			return i;

	return GPU_DVFS_GOVERNOR_INVALID;
}

/**
 * gpu_dvfs_governor_print_available() - Prints the names of the available governors.
 *
 * @buf:  The memory region to write out the governor names to.
 * @size: The maximum amount of data to write into @buf.
 *
 * Return: The amount of chars written to @buf.
 */
ssize_t gpu_dvfs_governor_print_available(char *buf, ssize_t size)
{
	int i;
	ssize_t ret = 0;

	for (i = 0; i < GPU_DVFS_GOVERNOR_COUNT; i++)
		ret += scnprintf(buf + ret, size - ret, "%s ", governors[i].name);

	ret += scnprintf(buf + ret, size - ret, "\n");

	return ret;
}

/**
 * gpu_dvfs_governor_print_curr() - Prints the name of the current governor.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 * @buf:  The memory region to write out the name to.
 * @size: The maximum amount of data to write into @buf.
 *
 * Return: The amount of chars written to @buf.
 */
ssize_t gpu_dvfs_governor_print_curr(struct kbase_device *kbdev, char *buf, ssize_t size)
{
	struct pixel_context *pc = kbdev->platform_context;

	return scnprintf(buf, size, "%s\n", governors[pc->dvfs.governor.curr].name);
}

/**
 * gpu_dvfs_governor_init() - Initializes the Pixel GPU DVFS governor subsystem.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: On success, returns 0. Currently only returns success.
 */
int gpu_dvfs_governor_init(struct kbase_device *kbdev)
{
	const char *governor_name;

	struct pixel_context *pc = kbdev->platform_context;
	struct device_node *np = kbdev->dev->of_node;

	if (of_property_read_string(np, "gpu_dvfs_governor", &governor_name)) {
		dev_warn(kbdev->dev, "GPU DVFS governor not specified in DT, using default\n");
		pc->dvfs.governor.curr = GPU_DVFS_GOVERNOR_BASIC;
		goto done;
	}

	pc->dvfs.governor.curr = gpu_dvfs_governor_get_id(governor_name);
	if (pc->dvfs.governor.curr == GPU_DVFS_GOVERNOR_INVALID) {
		dev_warn(kbdev->dev, "GPU DVFS governor \"%s\" doesn't exist, using default\n",
			governor_name);
		pc->dvfs.governor.curr = GPU_DVFS_GOVERNOR_BASIC;
		goto done;
	}

done:
	return 0;
}

/**
 * gpu_dvfs_governor_term() - Terminates the Pixel GPU DVFS QOS subsystem.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Note that this function currently doesn't do anything.
 */
void gpu_dvfs_governor_term(struct kbase_device *kbdev)
{
}
