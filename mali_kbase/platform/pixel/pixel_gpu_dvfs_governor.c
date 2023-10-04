// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Mali core includes */
#include <mali_kbase.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_dvfs.h"

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
	}
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
	int level;

	lockdep_assert_held(&pc->dvfs.lock);
	level = governors[pc->dvfs.governor.curr].evaluate(kbdev, util_stats);
	return clamp(level, pc->dvfs.level_scaling_max, pc->dvfs.level_scaling_min);
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
