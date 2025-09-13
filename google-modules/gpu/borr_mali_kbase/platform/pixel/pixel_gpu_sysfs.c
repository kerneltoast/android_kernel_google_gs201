// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Mali core includes */
#include <mali_kbase.h>
#include <trace/events/power.h>
#include <linux/seq_file.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_dvfs.h"
#include "pixel_gpu_sscd.h"

static const char *gpu_dvfs_level_lock_names[GPU_DVFS_LEVEL_LOCK_COUNT] = {
#if IS_ENABLED(CONFIG_CAL_IF)
	"ect",
#endif /* CONFIG_CAL_IF */
	"devicetree",
	"compute",
	"hint",
	"sysfs",
#ifdef CONFIG_MALI_PIXEL_GPU_THERMAL
	"thermal",
#endif /* CONFIG_MALI_PIXEL_GPU_THERMAL */
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
        "bcl",
#endif
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
		" gpu_0   gpu_0   gpu_1   gpu_1  util util hyste- int_clk  mif_clk cpu0_clk cpu1_clk cpu2_clk    mcu      mcu\n"
		"  clk     vol     clk     vol   min  max  resis    min      min     min      min      limit  down_util up_util\n"
		"------- ------- ------- ------- ---- ---- ------ ------- -------- -------- -------- -------- --------- -------\n");

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
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%8s", "none");
		else
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%8d",
				pc->dvfs.table[i].qos.cpu2_max);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%9d %7d\n",
			pc->dvfs.table[i].mcu_util_min,
			pc->dvfs.table[i].mcu_util_max);
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

static ssize_t gpu_top_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_metrics_uid_stats *entry = NULL;
	unsigned long flags;
	unsigned bkt;

	if (!pc)
		return -ENODEV;

	/* Grab the lock before reading and modifying the entries upon sysfs read. */
	spin_lock_irqsave(&pc->dvfs.metrics.lock, flags);
	hash_for_each(pc->dvfs.metrics.uid_stats_table, bkt, entry, uid_list_node) {
		const u64 delta_ns = ktime_get_ns() - entry->timestamp_ns_last;
#if MALI_USE_CSF
                /* The GPU cycles increase with the top level clock on CSF. */
		const u64 max_cycles_per_ms = pc->dvfs.table[0].clk[GPU_DVFS_CLK_TOP_LEVEL] / 1000;
#else
                /* The GPU cycles increase with the shaders clock on CSF. */
		const u64 max_cycles_per_ms = pc->dvfs.table[0].clk[GPU_DVFS_CLK_SHADERS] / 1000;
#endif
		const u64 max_cycles_since_last_read = (max_cycles_per_ms * delta_ns) / 1000;
		const u64 dec_precision = 100;
		const u64 val_cycles_pct =
                    (entry->gpu_cycles_last * 100 * dec_precision) / max_cycles_since_last_read;
		const u64 val_cycles_pct_int = val_cycles_pct / dec_precision;
		const u64 val_cycles_pct_dec = val_cycles_pct - val_cycles_pct_int * dec_precision;

		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u,%llu.%02llu\n",
			__kuid_val(entry->uid), val_cycles_pct_int, val_cycles_pct_dec);

		entry->gpu_cycles_last = 0;
		entry->gpu_active_ns_last = 0;
		entry->timestamp_ns_last = ktime_get_ns();
	}
	spin_unlock_irqrestore(&pc->dvfs.metrics.lock, flags);

	return ret;
}

static ssize_t uid_time_in_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_metrics_uid_stats *entry = NULL;
	unsigned long flags;
	unsigned bkt;

	if (!pc)
		return -ENODEV;

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "uid: ");
	for (i=0; i < pc->dvfs.table_size; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u ",
			pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS]);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");

	/* Grab the lock before reading the entries upon sysfs read. */
	spin_lock_irqsave(&pc->dvfs.metrics.lock, flags);
	hash_for_each(pc->dvfs.metrics.uid_stats_table, bkt, entry, uid_list_node) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u: ", __kuid_val(entry->uid));
		for (i=0; i < pc->dvfs.table_size; i++) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%llu ",
				entry->tis_stats[i].time_total / NSEC_PER_MSEC);
		}

		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}
	spin_unlock_irqrestore(&pc->dvfs.metrics.lock, flags);

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

#if MALI_USE_CSF
static ssize_t trigger_fw_fault_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct kbase_device *kbdev = dev->driver_data;
	u32 addr = 0;
	struct device_node *dpm = of_find_node_by_name(NULL, "dpm");
	const char *variant = NULL;

	/* Make this a no-op on -user builds. */
	if ((!dpm) || of_property_read_string(dpm, "variant", &variant) ||
	    (!strcmp(variant, "user")))
		return count;

	(void)attr, (void)buf;

	/* pixel: func_call_list_va_start stores FW log callback fn address from FW
	 * header. This is a NOP until FW logging is enabled by the kernel.
	 */
	kbase_csf_read_firmware_memory(kbdev, kbdev->csf.fw_log.func_call_list_va_start, &addr);

	dev_warn(kbdev->dev, "pixel: patching csffw 0x%x to 0x%x\n", addr, 0xFFFFFFFF);
	kbase_csf_update_firmware_memory_exe(kbdev, addr, 0xFFFFFFFF);
	mdelay(1000);
	dev_warn(kbdev->dev, "pixel: patching csffw 0x%x to 0x%x (NOP)\n",
		addr, 0xbf00bf00 /*ARMV7_DOUBLE_NOP_INSTR*/);
	kbase_csf_update_firmware_memory_exe(kbdev, addr,
		0xbf00bf00 /*ARMV7_DOUBLE_NOP_INSTR*/);

	return count;
}
DEVICE_ATTR_WO(trigger_fw_fault);
#endif

DEVICE_ATTR_RO(utilization);
DEVICE_ATTR_RO(clock_info);
DEVICE_ATTR_RO(dvfs_table);
DEVICE_ATTR_RO(power_stats);
DEVICE_ATTR_RO(gpu_top);
DEVICE_ATTR_RO(uid_time_in_state);
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

	trace_clock_set_rate("gpu_hint_max", clock, raw_smp_processor_id());

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

	trace_clock_set_rate("gpu_hint_min", clock, raw_smp_processor_id());

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

#if MALI_USE_CSF
static ssize_t hint_power_on_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	bool enabled;
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	if (!pc)
		return -ENODEV;

	ret = strtobool(buf, &enabled);
	if (ret)
		return -EINVAL;

	if (enabled)
		kthread_queue_work(&kbdev->apc.worker, &kbdev->apc.wakeup_csf_scheduler_work);

	return count;
}
#endif

#if MALI_USE_CSF
static ssize_t capacity_headroom_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		pc->dvfs.capacity_headroom);
}

static ssize_t capacity_headroom_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	int capacity_headroom = 0;

	if (!pc)
		return -ENODEV;

	if (kstrtoint(buf, 0, &capacity_headroom))
		return -EINVAL;

	mutex_lock(&pc->dvfs.lock);
	pc->dvfs.capacity_headroom = capacity_headroom;
	mutex_unlock(&pc->dvfs.lock);
	trace_clock_set_rate("cap_headroom", capacity_headroom, raw_smp_processor_id());

	return count;
}

static ssize_t capacity_history_depth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;

	if (!pc)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%u\n",
		(unsigned int)pc->dvfs.capacity_history_depth);
}

static ssize_t capacity_history_depth_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct kbase_device *kbdev = dev->driver_data;
	struct pixel_context *pc = kbdev->platform_context;
	unsigned int capacity_history_depth = 0;

	if (!pc)
		return -ENODEV;

	if (kstrtouint(buf, 0, &capacity_history_depth))
		return -EINVAL;

	if (capacity_history_depth == 0 || capacity_history_depth > ARRAY_SIZE(pc->dvfs.capacity_history))
		return -EINVAL;

	mutex_lock(&pc->dvfs.lock);
	pc->dvfs.capacity_history_depth = (u8)capacity_history_depth;
	mutex_unlock(&pc->dvfs.lock);

	return count;
}
#endif

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
#if MALI_USE_CSF
DEVICE_ATTR_WO(hint_power_on);
#endif
#if MALI_USE_CSF
DEVICE_ATTR_RW(capacity_headroom);
DEVICE_ATTR_RW(capacity_history_depth);
#endif

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
	{ "gpu_top", &dev_attr_gpu_top },
	{ "uid_time_in_state", &dev_attr_uid_time_in_state },
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
#if MALI_USE_CSF
	{ "capacity_headroom", &dev_attr_capacity_headroom },
	{ "capacity_history_depth", &dev_attr_capacity_history_depth },
	{ "hint_power_on", &dev_attr_hint_power_on },
	{ "trigger_fw_fault", &dev_attr_trigger_fw_fault }
#endif
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

#if IS_ENABLED(CONFIG_DEBUG_FS)

/**
 * gpu_top_h_show() - Print the gpu top in a human readable format
 *
 * @file: The seq_file for printing to
 * @data: The debugfs entry private data, a pointer to kbase_context
 *
 * Return: Negative error code or 0 on success.
 */
static int gpu_top_h_show(struct seq_file *file, void *data)
{
	struct kbase_device *kbdev = file->private;
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_metrics_uid_stats *entry = NULL;
	unsigned long flags;
	unsigned bkt;

	CSTD_UNUSED(data);

	if (!pc)
		return -ENODEV;

	seq_printf(file, "+-------+---------+-------+\n");
	seq_printf(file, "|  UID  | CYCLES%% | TIME%% |\n");
	seq_printf(file, "|-------+---------+-------|\n");

	/* Grab the lock before reading and modifying the entries upon debugfs read. */
	spin_lock_irqsave(&pc->dvfs.metrics.lock, flags);
	hash_for_each(pc->dvfs.metrics.uid_stats_table, bkt, entry, uid_list_node) {
		const u64 delta_ns = ktime_get_ns() - entry->timestamp_ns_last;
#if MALI_USE_CSF
                /* The GPU cycles increase with the top level clock on CSF. */
		const u64 max_cycles_per_ms = pc->dvfs.table[0].clk[GPU_DVFS_CLK_TOP_LEVEL] / 1000;
#else
                /* The GPU cycles increase with the shaders clock on CSF. */
		const u64 max_cycles_per_ms = pc->dvfs.table[0].clk[GPU_DVFS_CLK_SHADERS] / 1000;
#endif
		const u64 max_cycles_since_last_read = (max_cycles_per_ms * delta_ns) / 1000;
		const u64 dec_precision = 100;
		const u64 val_cycles_pct =
                    (entry->gpu_cycles_last * 100 * dec_precision) / max_cycles_since_last_read;
		const u64 val_cycles_pct_int = val_cycles_pct / dec_precision;
		const u64 val_cycles_pct_dec = val_cycles_pct - val_cycles_pct_int * dec_precision;
		const u64 val_nss_pct =
                    (delta_ns != 0 ) ? ((entry->gpu_active_ns_last * 100) / delta_ns) : 0;

		seq_printf(file, "|");
		seq_printf(file, "%6u |", __kuid_val(entry->uid));
		seq_printf(file, "%5llu.%02llu |", val_cycles_pct_int, val_cycles_pct_dec);
		seq_printf(file, "%6llu |", val_nss_pct);
		seq_printf(file, "\n");

		entry->gpu_cycles_last = 0;
		entry->gpu_active_ns_last = 0;
		entry->timestamp_ns_last = ktime_get_ns();
	}
	spin_unlock_irqrestore(&pc->dvfs.metrics.lock, flags);

	seq_printf(file, "+-------+---------+-------+\n");

	return 0;
}

/**
 * uid_time_in_state_h_show() - Print the uid_time_in_state in a human readable format
 *
 * @file: The seq_file for printing to
 * @data: The debugfs entry private data, a pointer to kbase_context
 *
 * Return: Negative error code or 0 on success.
 */
static int uid_time_in_state_h_show(struct seq_file *file, void *data)
{
	int i;
	struct kbase_device *kbdev = file->private;
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_metrics_uid_stats *entry = NULL;
	u64 *totals;
	unsigned long flags;
	unsigned bkt;

	CSTD_UNUSED(data);

	if (!pc)
		return -ENODEV;

	totals = kzalloc(sizeof(u64) * pc->dvfs.table_size, GFP_KERNEL);

	seq_printf(file, "            | ");
	for (i=0; i < pc->dvfs.table_size; i++)
		seq_printf(file, "%9u  ", pc->dvfs.table[i].clk[GPU_DVFS_CLK_SHADERS]);
	seq_printf(file, "\n------------+-----------------------------------------------------------------\n");

	/* Grab the lock before reading the entries upon debugfs read. */
	spin_lock_irqsave(&pc->dvfs.metrics.lock, flags);
	hash_for_each(pc->dvfs.metrics.uid_stats_table, bkt, entry, uid_list_node) {
		seq_printf(file, "%6d (%2d) | ", __kuid_val(entry->uid), entry->active_kctx_count);
		for (i=0; i < pc->dvfs.table_size; i++) {
			totals[i] += entry->tis_stats[i].time_total;
			seq_printf(file, "%9llu  ", entry->tis_stats[i].time_total / NSEC_PER_MSEC);
		}

		seq_printf(file, "\n");
	}
	spin_unlock_irqrestore(&pc->dvfs.metrics.lock, flags);

	seq_printf(file, "------------+-----------------------------------------------------------------\n");

	seq_printf(file, "     Totals | ");
	for (i=0; i < pc->dvfs.table_size; i++) {
		seq_printf(file, "%9llu  ", totals[i] / NSEC_PER_MSEC);
	}
	seq_printf(file, "\n");

	kfree(totals);

	return 0;
}

static int gpu_top_h_open(struct inode *in, struct file *file)
{
	return single_open(file, gpu_top_h_show, in->i_private);
}

static const struct file_operations gpu_top_h_show_fops = {
	.open = gpu_top_h_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int uid_time_in_state_h_open(struct inode *in, struct file *file)
{
	return single_open(file, uid_time_in_state_h_show, in->i_private);
}

static const struct file_operations uid_time_in_state_h_show_fops = {
	.open = uid_time_in_state_h_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void pixel_gpu_debugfs_init(struct kbase_device *kbdev)
{
	struct dentry *file;
	const mode_t mode = 0444;

	if (WARN_ON(!kbdev || IS_ERR_OR_NULL(kbdev->mali_debugfs_directory)))
		return;

	file = debugfs_create_file("gpu_top_h", mode, kbdev->mali_debugfs_directory, kbdev,
				   &gpu_top_h_show_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kbdev->dev, "Unable to create dvfs debugfs entry for gpu_top_h");
	}

	file = debugfs_create_file("uid_time_in_state_h", mode, kbdev->mali_debugfs_directory, kbdev,
				   &uid_time_in_state_h_show_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kbdev->dev, "Unable to create dvfs debugfs entry for uid_time_in_state_h");
	}
}

#endif /* CONFIG_DEBUG_FS */
