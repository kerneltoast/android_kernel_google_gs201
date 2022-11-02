// SPDX-License-Identifier: GPL-2.0-only
/* metrics.c
 *
 * Support for Perf metrics
 *
 * Copyright 2022 Google LLC
 */
#define pr_fmt(fmt) KBUILD_MODNAME": " fmt
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>

#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>


#include <trace/events/irq.h>
#include <trace/hooks/suspend.h>
#include "metrics.h"

static struct resume_latency resume_latency_stats;
static struct kobject *primary_sysfs_folder;

/*********************************************************************
 *                          SYSTEM TRACE
 *********************************************************************/

static void vendor_hook_resume_begin(void *data, void *unused)
{
	resume_latency_stats.resume_start = ktime_get();
}

static void vendor_hook_resume_end(void *data, void *unused)
{
	int resume_latency_index;
	s64 resume_latency_msec;
	/* Exit function when partial resumes */
	if (resume_latency_stats.resume_start == resume_latency_stats.resume_end)
		return;
	resume_latency_stats.resume_end = ktime_get();
	resume_latency_msec = ktime_ms_delta(resume_latency_stats.resume_end,
						resume_latency_stats.resume_start);
	pr_info("resume latency: %lld\n", resume_latency_msec);
	/* Exit function when partial resumes */
	if (resume_latency_msec <= 0)
		return;
	spin_lock(&resume_latency_stats.resume_latency_stat_lock);
	if (resume_latency_msec < RESUME_LATENCY_BOUND_SMALL) {
		resume_latency_index = resume_latency_msec / RESUME_LATENCY_STEP_SMALL;
	} else if (resume_latency_msec < RESUME_LATENCY_BOUND_MID) {
		resume_latency_index = (resume_latency_msec - RESUME_LATENCY_BOUND_SMALL) /
						RESUME_LATENCY_STEP_MID + LATENCY_CNT_SMALL;
	} else if (resume_latency_msec < RESUME_LATENCY_BOUND_MAX) {
		resume_latency_index = (resume_latency_msec - RESUME_LATENCY_BOUND_MID) /
						RESUME_LATENCY_STEP_LARGE + LATENCY_CNT_SMALL +
						LATENCY_CNT_MID;
	} else {
		resume_latency_index = LATENCY_CNT_SMALL + LATENCY_CNT_MID + LATENCY_CNT_LARGE;
	}
	resume_latency_stats.resume_count[resume_latency_index]++;
	resume_latency_stats.resume_latency_sum_ms += resume_latency_msec;
	resume_latency_stats.resume_latency_max_ms = max(resume_latency_stats.resume_latency_max_ms,
						resume_latency_msec);
	spin_unlock(&resume_latency_stats.resume_latency_stat_lock);
	resume_latency_stats.resume_start = resume_latency_stats.resume_end;
}


/*******************************************************************
 *                       		SYSFS			   				   *
 *******************************************************************/

static ssize_t resume_latency_metrics_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	s64 lower_bound;
	s64 upper_bound;
	int index;
	ssize_t count = 0;
	count += sysfs_emit_at(buf, count, "Resume Latency Bucket Count: %d\n",
				RESUME_LATENCY_ARR_SIZE);
	count += sysfs_emit_at(buf, count, "Max Resume Latency: %lld\n",
				resume_latency_stats.resume_latency_max_ms);
	count += sysfs_emit_at(buf, count, "Sum Resume Latency: %llu\n",
				resume_latency_stats.resume_latency_sum_ms);
	for (index = 0; index < RESUME_LATENCY_ARR_SIZE; index++) {
		if (index < LATENCY_CNT_SMALL) {
			lower_bound = index * RESUME_LATENCY_STEP_SMALL;
			upper_bound = lower_bound + RESUME_LATENCY_STEP_SMALL;
			count += sysfs_emit_at(buf, count, "%lld - %lldms ====> %lld\n",
				lower_bound, upper_bound,
				resume_latency_stats.resume_count[index]);
		} else if (index < LATENCY_CNT_SMALL + LATENCY_CNT_MID) {
			lower_bound = RESUME_LATENCY_BOUND_SMALL + RESUME_LATENCY_STEP_MID *
				(index - LATENCY_CNT_SMALL);
			upper_bound = lower_bound + RESUME_LATENCY_STEP_MID;
			count += sysfs_emit_at(buf, count, "%lld - %lldms ====> %lld\n",
				lower_bound, upper_bound,
				resume_latency_stats.resume_count[index]);
		} else if (index < LATENCY_CNT_SMALL + LATENCY_CNT_MID + LATENCY_CNT_LARGE) {
			lower_bound = RESUME_LATENCY_BOUND_MID + RESUME_LATENCY_STEP_LARGE *
				(index - (LATENCY_CNT_SMALL + LATENCY_CNT_MID));
			upper_bound = lower_bound + RESUME_LATENCY_STEP_LARGE;
			count += sysfs_emit_at(buf, count, "%lld - %lldms ====> %lld\n",
				lower_bound, upper_bound,
				resume_latency_stats.resume_count[index]);
		} else {
			lower_bound = RESUME_LATENCY_BOUND_MAX;
			count += sysfs_emit_at(buf, count, "%lld - infms ====> %lld\n",
				lower_bound,
				resume_latency_stats.resume_count[index]);
		}
	}
	return count;
}

static ssize_t resume_latency_metrics_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	spin_lock(&resume_latency_stats.resume_latency_stat_lock);
	resume_latency_stats.resume_latency_max_ms = 0;
	resume_latency_stats.resume_latency_sum_ms = 0;
	memset(resume_latency_stats.resume_count, 0, RESUME_LATENCY_ARR_SIZE *
				sizeof(resume_latency_stats.resume_count[0]));
	spin_unlock(&resume_latency_stats.resume_latency_stat_lock);
	return count;
}


static struct kobj_attribute resume_latency_metrics_attr = __ATTR(resume_latency_metrics,
							  0664,
							  resume_latency_metrics_show,
							  resume_latency_metrics_store);

static struct attribute *resume_latency_attrs[] = {
	&resume_latency_metrics_attr.attr,
	NULL
};

static const struct attribute_group resume_latency_attr_group = {
	.attrs = resume_latency_attrs,
	.name = "resume_latency"
};

/*********************************************************************
 *                  		INITIALIZE DRIVER                        *
 *********************************************************************/

static int __init perf_metrics_init(void)
{
	int ret = 0;
	primary_sysfs_folder = kobject_create_and_add("metrics", kernel_kobj);
	if (!primary_sysfs_folder) {
		pr_err("Failed to create primary sysfs folder!\n");
		return -EINVAL;
	}
	if (sysfs_create_group(primary_sysfs_folder, &resume_latency_attr_group)) {
		pr_err("failed to create resume_latency folder\n");
		return ret;
	}
	spin_lock_init(&resume_latency_stats.resume_latency_stat_lock);
	ret = register_trace_android_vh_early_resume_begin(
					vendor_hook_resume_begin, NULL);
	if (ret) {
		pr_err("Register resume begin vendor hook fail %d\n", ret);
		return ret;
	}
	ret = register_trace_android_vh_resume_end(
					vendor_hook_resume_end, NULL);
	if (ret) {
		pr_err("Register resume end vendor hook fail %d\n", ret);
		return ret;
	}
	pr_info("perf_metrics driver initialized! :D\n");
	return ret;
}

module_init(perf_metrics_init);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ziyi Cui <ziyic@google.com>");
