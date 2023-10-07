// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Mali core includes */
#include <mali_kbase.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_dvfs.h"
#include "pixel_gpu_sscd.h"

static const char *gpu_dvfs_level_lock_names[GPU_DVFS_LEVEL_LOCK_COUNT] = {
	"devicetree",
	"compute",
	"hint",
	"sysfs",
#ifdef CONFIG_MALI_PIXEL_GPU_THERMAL
	"thermal",
#endif /* CONFIG_MALI_PIXEL_GPU_THERMAL */
};

/* Helper functions */

/**
 * get_level_from_clock() - Helper function to get the level index corresponding to a G3D clock.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 * @clock: The frequency (in kHz) of the GPU Top Level clock to get the level from.
 *
 * Return: The level corresponding to @clock, -1 on failure.
 */
static int get_level_from_clock(struct kbase_device *kbdev, int clock)
{
	struct pixel_context *pc = kbdev->platform_context;
	int i;

	for (i = 0; i < pc->dvfs.table_size; i++)
		if (pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS] == clock)
			return i;

	return -1;
}

/* Custom attributes */

static ssize_t utilization_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&pc->dvfs.util));
}

static ssize_t clock_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	/* Basic status */

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"BASIC STATUS\n"
		" Power status            : %s\n"
		" gpu0 clock (top level)  : %d kHz\n"
		" gpu1 clock (shaders)    : %d kHz\n",
		(gpu_pm_get_power_state(kbdev) ? "on" : "off"),
		pc->dvfs.table[pc->dvfs.level_target].clk[GPU_DVFS_CLK_TOP_LEVEL],
		pc->dvfs.table[pc->dvfs.level_target].clk[GPU_DVFS_CLK_SHADERS]);

	/* Level lock status */

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"\nLEVEL LOCK STATUS\n"
		" Type            | Min (kHz) | Max (kHz)\n"
		" ----------------+-----------+-----------\n");

	for (i = 0; i < GPU_DVFS_LEVEL_LOCK_COUNT; i++) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			" %-15s |",
			gpu_dvfs_level_lock_names[i]);

		if (gpu_dvfs_level_lock_is_set(pc->dvfs.level_locks[i].level_min))
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " %-10d|",
				pc->dvfs.table[pc->dvfs.level_locks[i].level_min].clk[GPU_DVFS_CLK_SHADERS]);
		else
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " -         |");

		if (gpu_dvfs_level_lock_is_set(pc->dvfs.level_locks[i].level_max))
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " %d\n",
				pc->dvfs.table[pc->dvfs.level_locks[i].level_max].clk[GPU_DVFS_CLK_SHADERS]);
		else
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " -\n");
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		" Effective Range | %-10d| %d\n",
		pc->dvfs.table[pc->dvfs.level_scaling_min].clk[GPU_DVFS_CLK_SHADERS],
		pc->dvfs.table[pc->dvfs.level_scaling_max].clk[GPU_DVFS_CLK_SHADERS]);

	/* QOS status */

#ifdef CONFIG_MALI_PIXEL_GPU_QOS

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"\nQOS STATUS\n");

#ifdef CONFIG_MALI_PIXEL_GPU_BTS
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		" Bus Traffic Shaping     : %s\n",
		(pc->dvfs.qos.bts.enabled ? "on" : "off"));
#endif /* CONFIG_MALI_PIXEL_GPU_BTS */

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		" QOS enabled             : %s\n"
		" INT min clock           : %d kHz\n"
		" MIF min clock           : %d kHz\n"
		" CPU cluster 0 min clock : %d kHz\n"
		" CPU cluster 1 min clock : %d kHz\n",
		(pc->dvfs.qos.enabled ? "yes" : "no"),
		pc->dvfs.table[pc->dvfs.level_target].qos.int_min,
		pc->dvfs.table[pc->dvfs.level_target].qos.mif_min,
		pc->dvfs.table[pc->dvfs.level_target].qos.cpu0_min,
		pc->dvfs.table[pc->dvfs.level_target].qos.cpu1_min);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		" CPU cluster 2 max clock : ");

	if (pc->dvfs.table[pc->dvfs.level_target].qos.cpu2_max == CPU_FREQ_MAX)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "none set\n");
	else
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d kHz\n",
			pc->dvfs.table[pc->dvfs.level_target].qos.cpu2_max);

#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

	return ret;
}

static ssize_t dvfs_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		" gpu_0   gpu_0   gpu_1   gpu_1  util util hyste- int_clk  mif_clk cpu0_clk cpu1_clk cpu2_clk\n"
		"  clk     vol     clk     vol   min  max  resis    min      min     min      min      limit\n"
		"------- ------- ------- ------- ---- ---- ------ ------- -------- -------- -------- --------\n");

	for (i = pc->dvfs.level_max; i <= pc->dvfs.level_min; i++) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"%7d %7d %7d %7d %4d %4d %6d %7d %8d %8d %8d ",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_TOP_LEVEL],
			pc->dvfs.table[i].vol[GPU_DVFS_CLK_TOP_LEVEL],
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS],
			pc->dvfs.table[i].vol[GPU_DVFS_CLK_SHADERS],
			pc->dvfs.table[i].util_min,
			pc->dvfs.table[i].util_max,
			pc->dvfs.table[i].hysteresis,
			pc->dvfs.table[i].qos.int_min,
			pc->dvfs.table[i].qos.mif_min,
			pc->dvfs.table[i].qos.cpu0_min,
			pc->dvfs.table[i].qos.cpu1_min);

		if (pc->dvfs.table[i].qos.cpu2_max == CPU_FREQ_MAX)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%8s\n", "none");
		else
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%8d\n",
				pc->dvfs.table[i].qos.cpu2_max);
	}

	return ret;
}

static ssize_t power_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	/* First trigger an update */
	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_metrics_update(kbdev, pc->dvfs.level, pc->dvfs.level,
		gpu_pm_get_power_state(kbdev));
	mutex_unlock(&pc->dvfs.lock);

	ret = scnprintf(buf + ret, PAGE_SIZE - ret, "DVFS stats: (times in ms)\n");

	for (i = 0; i < pc->dvfs.table_size; i++) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"%d:\n\ttotal_time = %llu\n\tcount = %d\n\tlast_entry_time = %llu\n",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS],
			pc->dvfs.table[i].metrics.time_total / NSEC_PER_MSEC,
			pc->dvfs.table[i].metrics.entry_count,
			pc->dvfs.table[i].metrics.time_last_entry / NSEC_PER_MSEC);
	}


	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Summary stats: (times in ms)\n");

	ret += scnprintf(
		buf + ret, PAGE_SIZE - ret,
		"ON:\n\ttotal_time = %llu\n\tcount = %d\n\tlast_entry_time = %llu\n",
		pc->pm.power_on_metrics.time_total / NSEC_PER_MSEC,
		pc->pm.power_on_metrics.entry_count,
		pc->pm.power_on_metrics.time_last_entry / NSEC_PER_MSEC);

	ret += scnprintf(
		buf + ret, PAGE_SIZE - ret,
		"OFF:\n\ttotal_time = %llu\n\tcount = %d\n\tlast_entry_time = %llu\n",
		pc->pm.power_off_metrics.time_total / NSEC_PER_MSEC,
		pc->pm.power_off_metrics.entry_count,
		pc->pm.power_off_metrics.time_last_entry / NSEC_PER_MSEC);

	return ret;
}

static ssize_t uid_time_in_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_metrics_uid_stats *entry = NULL;

	if (!pc)
		return -ENODEV;

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "uid: ");
	for (i=0; i < pc->dvfs.table_size; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u ",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS]);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");

	list_for_each_entry(entry, &pc->dvfs.metrics.uid_stats_list, uid_list_link) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u: ", __kuid_val(entry->uid));
		for (i=0; i < pc->dvfs.table_size; i++) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%llu ",
				entry->tis_stats[i].time_total / NSEC_PER_MSEC);
		}

		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}

	return ret;
}


static ssize_t uid_time_in_state_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_metrics_uid_stats *entry = NULL;
	u64 *totals;

	if (!pc)
		return -ENODEV;

	totals = kzalloc(sizeof(u64) * pc->dvfs.table_size, GFP_KERNEL);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "            | ");
	for (i=0; i < pc->dvfs.table_size; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%9u  ",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS]);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"\n------------+-----------------------------------------------------------------\n");

	list_for_each_entry(entry, &pc->dvfs.metrics.uid_stats_list, uid_list_link) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%6d (%2d) | ",
			__kuid_val(entry->uid), entry->active_kctx_count);
		for (i=0; i < pc->dvfs.table_size; i++) {
			totals[i] += entry->tis_stats[i].time_total;
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%9llu  ",
				entry->tis_stats[i].time_total / NSEC_PER_MSEC);
		}

		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"------------+-----------------------------------------------------------------\n");

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "     Totals | ");
	for (i=0; i < pc->dvfs.table_size; i++) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%9llu  ",
			totals[i] / NSEC_PER_MSEC);
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");

	kfree(totals);

	return ret;
}

static ssize_t trigger_core_dump_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct kbase_device *kbdev = dev->driver_data;

	(void)attr, (void)buf;

	gpu_sscd_dump(kbdev, "Manual core dump");

	return count;
}

DEVICE_ATTR_RO(utilization);
DEVICE_ATTR_RO(clock_info);
DEVICE_ATTR_RO(dvfs_table);
DEVICE_ATTR_RO(power_stats);
DEVICE_ATTR_RO(uid_time_in_state);
DEVICE_ATTR_RO(uid_time_in_state_h);
DEVICE_ATTR_WO(trigger_core_dump);


/* devfreq-like attributes */

static ssize_t cur_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	/* We use level_target in case the clock has been set while the GPU was powered down */
	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.table[pc->dvfs.level_target].clk[GPU_DVFS_CLK_SHADERS]);
}

static ssize_t available_frequencies_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int i;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	for (i = 0; i < pc->dvfs.table_size; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d ",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS]);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");

	return ret;
}

static ssize_t max_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.table[pc->dvfs.level_max].clk[GPU_DVFS_CLK_SHADERS]);
}

static ssize_t min_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.table[pc->dvfs.level_min].clk[GPU_DVFS_CLK_SHADERS]);
}

static ssize_t scaling_min_compute_freq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.table[pc->dvfs.level_scaling_compute_min].clk[GPU_DVFS_CLK_SHADERS]);
}

static ssize_t hint_max_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	int sysfs_lock_level;

	if (!pc)
		return -ENODEV;

	sysfs_lock_level = pc->dvfs.level_locks[GPU_DVFS_LEVEL_LOCK_HINT].level_max;
	if (sysfs_lock_level < 0)
		sysfs_lock_level = pc->dvfs.level_max;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.table[sysfs_lock_level].clk[GPU_DVFS_CLK_SHADERS]);
}

static ssize_t hint_max_freq_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int level, ret;
	unsigned int clock;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	ret = kstrtoint(buf, 0, &clock);
	if (ret)
		return -EINVAL;

	level = get_level_from_clock(kbdev, clock);
	if (level < 0)
		return -EINVAL;

	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_update_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_HINT, -1, level);
	gpu_dvfs_select_level(kbdev);
	mutex_unlock(&pc->dvfs.lock);

	return count;
}

static ssize_t hint_min_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	int sysfs_lock_level;

	if (!pc)
		return -ENODEV;

	sysfs_lock_level = pc->dvfs.level_locks[GPU_DVFS_LEVEL_LOCK_HINT].level_min;
	if (sysfs_lock_level < 0)
		sysfs_lock_level = pc->dvfs.level_min;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.table[sysfs_lock_level].clk[GPU_DVFS_CLK_SHADERS]);
}

static ssize_t hint_min_freq_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret, level;
	unsigned int clock;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	ret = kstrtoint(buf, 0, &clock);
	if (ret)
		return -EINVAL;

	level = get_level_from_clock(kbdev, clock);
	if (level < 0)
		return -EINVAL;

	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_update_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_HINT, level, -1);
	gpu_dvfs_select_level(kbdev);
	mutex_unlock(&pc->dvfs.lock);

	return count;
}

static ssize_t scaling_max_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	int sysfs_lock_level;

	if (!pc)
		return -ENODEV;

	sysfs_lock_level = pc->dvfs.level_locks[GPU_DVFS_LEVEL_LOCK_SYSFS].level_max;
	if (sysfs_lock_level < 0)
		sysfs_lock_level = pc->dvfs.level_max;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.table[sysfs_lock_level].clk[GPU_DVFS_CLK_SHADERS]);
}

static ssize_t scaling_max_freq_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int level, ret;
	unsigned int clock;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	ret = kstrtoint(buf, 0, &clock);
	if (ret)
		return -EINVAL;

	level = get_level_from_clock(kbdev, clock);
	if (level < 0)
		return -EINVAL;

	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_update_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_SYSFS, -1, level);
	gpu_dvfs_select_level(kbdev);
	mutex_unlock(&pc->dvfs.lock);

	return count;
}

static ssize_t scaling_min_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	int sysfs_lock_level;

	if (!pc)
		return -ENODEV;

	sysfs_lock_level = pc->dvfs.level_locks[GPU_DVFS_LEVEL_LOCK_SYSFS].level_min;
	if (sysfs_lock_level < 0)
		sysfs_lock_level = pc->dvfs.level_min;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.table[sysfs_lock_level].clk[GPU_DVFS_CLK_SHADERS]);
}

static ssize_t scaling_min_freq_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret, level;
	unsigned int clock;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	ret = kstrtoint(buf, 0, &clock);
	if (ret)
		return -EINVAL;

	level = get_level_from_clock(kbdev, clock);
	if (level < 0)
		return -EINVAL;

	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_update_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_SYSFS, level, -1);
	gpu_dvfs_select_level(kbdev);
	mutex_unlock(&pc->dvfs.lock);

	return count;
}

static ssize_t time_in_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	/* First trigger an update */
	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_metrics_update(kbdev, pc->dvfs.level, pc->dvfs.level,
		gpu_pm_get_power_state(kbdev));
	mutex_unlock(&pc->dvfs.lock);

	for (i = pc->dvfs.level_max; i <= pc->dvfs.level_min; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%8d %9d\n",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS],
			(u32)(pc->dvfs.table[i].metrics.time_total / NSEC_PER_MSEC));

	return ret;
}

static ssize_t trans_stat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, j, t, total = 0;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	/* First trigger an update */
	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_metrics_update(kbdev, pc->dvfs.level, pc->dvfs.level,
		gpu_pm_get_power_state(kbdev));
	mutex_unlock(&pc->dvfs.lock);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%9s  :   %s\n", "From", "To");

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%12s", ":");
	for (i = pc->dvfs.level_max; i <= pc->dvfs.level_min; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%10d",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS]);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%11s\n", "time(ms)");

	for (i = pc->dvfs.level_max; i <= pc->dvfs.level_min; i++) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%s%10d:",
			(i == pc->dvfs.level) ? "*" : " ",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS]);
		for (j = pc->dvfs.level_max; j <= pc->dvfs.level_min; j++) {
			t = gpu_dvfs_metrics_transtab_entry(pc, i, j);
			total += t;
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%10d", t);
		}
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%10d\n",
			(u32)(pc->dvfs.table[i].metrics.time_total / NSEC_PER_MSEC));
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Total transition : %d\n", total);

	return ret;
}

static ssize_t available_governors_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return gpu_dvfs_governor_print_available(buf, PAGE_SIZE);
}

static ssize_t governor_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;

	return gpu_dvfs_governor_print_curr(kbdev, buf, PAGE_SIZE);
}

static ssize_t governor_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	enum gpu_dvfs_governor_type gov;
	ssize_t ret = count;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	gov = gpu_dvfs_governor_get_id(buf);

	if (gov == GPU_DVFS_GOVERNOR_INVALID)
		ret = -EINVAL;
	else if (gov != pc->dvfs.governor.curr) {
		mutex_lock(&pc->dvfs.lock);
		if (gpu_dvfs_governor_set_governor(kbdev, gov))
			ret = -EINVAL;
		mutex_unlock(&pc->dvfs.lock);
	}

	return ret;
}

static ssize_t ifpo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_MALI_HOST_CONTROLS_SC_RAILS
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	ssize_t ret = 0;

	if (!pc)
		return -ENODEV;

	mutex_lock(&pc->pm.lock);
	ret = scnprintf(buf, PAGE_SIZE, "%d\n", pc->pm.ifpo_enabled);
	mutex_unlock(&pc->pm.lock);
	return ret;
#else
	return -ENOTSUPP;
#endif
}

static ssize_t ifpo_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef CONFIG_MALI_HOST_CONTROLS_SC_RAILS
	int ret;
	bool enabled;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	if (!pc)
		return -ENODEV;

	ret = strtobool(buf, &enabled);
	if (ret)
		return -EINVAL;

	mutex_lock(&kbdev->csf.scheduler.lock);

	if (!enabled) {
		turn_on_sc_power_rails(kbdev);
	}

	mutex_lock(&pc->pm.lock);
	pc->pm.ifpo_enabled = enabled;
	mutex_unlock(&pc->pm.lock);
	mutex_unlock(&kbdev->csf.scheduler.lock);

	return count;
#else
	return -ENOTSUPP;
#endif
}


/* Define devfreq-like attributes */
DEVICE_ATTR_RO(available_frequencies);
DEVICE_ATTR_RO(cur_freq);
DEVICE_ATTR_RO(max_freq);
DEVICE_ATTR_RO(min_freq);
DEVICE_ATTR_RO(scaling_min_compute_freq);
DEVICE_ATTR_RW(hint_max_freq);
DEVICE_ATTR_RW(hint_min_freq);
DEVICE_ATTR_RW(scaling_max_freq);
DEVICE_ATTR_RW(scaling_min_freq);
DEVICE_ATTR_RO(time_in_state);
DEVICE_ATTR_RO(trans_stat);
DEVICE_ATTR_RO(available_governors);
DEVICE_ATTR_RW(governor);
DEVICE_ATTR_RW(ifpo);

/* Initialization code */

/*
 * attribs - An array containing all sysfs files for the Pixel GPU sysfs system.
 *
 * This array contains the list of all files that will be set up and removed by the Pixel GPU sysfs
 * system. It allows for more compact initialization and termination code below.
 */
static struct {
	const char *name;
	const struct device_attribute *attr;
} attribs[] = {
	{ "utilization", &dev_attr_utilization },
	{ "clock_info", &dev_attr_clock_info },
	{ "dvfs_table", &dev_attr_dvfs_table },
	{ "power_stats", &dev_attr_power_stats },
	{ "uid_time_in_state", &dev_attr_uid_time_in_state },
	{ "uid_time_in_state_h", &dev_attr_uid_time_in_state_h },
	{ "available_frequencies", &dev_attr_available_frequencies },
	{ "cur_freq", &dev_attr_cur_freq },
	{ "max_freq", &dev_attr_max_freq },
	{ "min_freq", &dev_attr_min_freq },
	{ "min_compute_freq", &dev_attr_scaling_min_compute_freq },
	{ "hint_max_freq", &dev_attr_hint_max_freq },
	{ "hint_min_freq", &dev_attr_hint_min_freq },
	{ "scaling_max_freq", &dev_attr_scaling_max_freq },
	{ "scaling_min_freq", &dev_attr_scaling_min_freq },
	{ "time_in_state", &dev_attr_time_in_state },
	{ "trans_stat", &dev_attr_trans_stat },
	{ "available_governors", &dev_attr_available_governors },
	{ "governor", &dev_attr_governor },
	{ "trigger_core_dump", &dev_attr_trigger_core_dump },
	{ "ifpo", &dev_attr_ifpo }
};

/**
 * gpu_sysfs_init() - Initializes the Pixel GPU sysfs system.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: On success, returns 0. -ENOENT if creating a sysfs file results in an error.
 */
int gpu_sysfs_init(struct kbase_device *kbdev)
{
	int i;
	struct device *dev = kbdev->dev;

	for (i = 0; i < ARRAY_SIZE(attribs); i++) {
		if (device_create_file(dev, attribs[i].attr)) {
			dev_err(kbdev->dev, "failed to create sysfs file %s\n",
				attribs[i].name);
			return -ENOENT;
		}
	}

	return 0;
}

/**
 * gpu_sysfs_term() - Terminates the Pixel GPU sysfs system.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_sysfs_term(struct kbase_device *kbdev)
{
	int i;
	struct device *dev = kbdev->dev;

	for (i = 0; i < ARRAY_SIZE(attribs); i++)
		device_remove_file(dev, attribs[i].attr);
}
