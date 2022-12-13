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
#include <linux/sort.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>

#include <trace/events/irq.h>
#include <trace/hooks/suspend.h>
#include "perf_metrics.h"

struct irq_entry {
	int irq_num;
	s64 latency;
};
static struct resume_latency resume_latency_stats;
static struct long_irq long_irq_stat;

/*********************************************************************
 *                          SYSTEM TRACE
 *********************************************************************/

static void vendor_hook_resume_begin(void *data, void *unused)
{
	resume_latency_stats.resume_start = ktime_get_mono_fast_ns();
}

static void vendor_hook_resume_end(void *data, void *unused)
{
	int resume_latency_index;
	u64 resume_latency_msec;
	/* Exit function when partial resumes */
	if (resume_latency_stats.resume_start == resume_latency_stats.resume_end)
		return;
	resume_latency_stats.resume_end = ktime_get_mono_fast_ns();
	resume_latency_msec = (resume_latency_stats.resume_end -
						resume_latency_stats.resume_start) / NSEC_PER_MSEC;
	pr_info("resume latency: %llu\n", resume_latency_msec);
	/* Exit function when partial resumes */
	if (resume_latency_stats.resume_end < resume_latency_stats.resume_start)
		return;
	if (resume_latency_stats.display_warning)
		WARN(resume_latency_msec >= resume_latency_stats.resume_latency_threshold,
				"Got a outlier resume latency: %llums\n", resume_latency_msec);
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

static void hook_softirq_begin(void *data, unsigned int vec_nr)
{
	int cpu_num;
	cpu_num = raw_smp_processor_id();
	long_irq_stat.softirq_start[cpu_num][vec_nr] = ktime_get();
}

static void hook_softirq_end(void *data, unsigned int vec_nr)
{
	s64 irq_usec;
	int cpu_num;
	s64 curr_max_irq;
	if (vec_nr >= NR_SOFTIRQS)
		return;
	cpu_num = raw_smp_processor_id();
	long_irq_stat.softirq_end = ktime_get();
	irq_usec = ktime_to_us(ktime_sub(long_irq_stat.softirq_end,
						long_irq_stat.softirq_start[cpu_num][vec_nr]));
	if (irq_usec >= long_irq_stat.long_softirq_threshold) {
		if (long_irq_stat.display_warning)
			WARN(1, "Got a long running softirq: SOFTIRQ %u in cpu: %d\n",
						vec_nr, cpu_num);
		atomic64_inc(&(long_irq_stat.long_softirq_count));
	}
	do {
		curr_max_irq = long_irq_stat.long_softirq_arr[vec_nr];
		if (irq_usec < curr_max_irq)
			return;
	} while (cmpxchg64(&long_irq_stat.long_softirq_arr[vec_nr],
						curr_max_irq, irq_usec) != curr_max_irq);
}

static void hook_irq_begin(void *data, int irq, struct irqaction *action)
{
	int cpu_num;
	cpu_num = raw_smp_processor_id();
	long_irq_stat.irq_start[cpu_num][irq] = ktime_get();
}

static void hook_irq_end(void *data, int irq, struct irqaction *action, int ret)
{
	s64 irq_usec;
	int cpu_num;
	s64 curr_max_irq;
	if (irq >= MAX_IRQ_NUM)
		return;
	cpu_num = raw_smp_processor_id();
	long_irq_stat.irq_end = ktime_get();
	irq_usec = ktime_to_us(ktime_sub(long_irq_stat.irq_end,
				long_irq_stat.irq_start[cpu_num][irq]));
	if (irq_usec >= long_irq_stat.long_irq_threshold) {
		if (long_irq_stat.display_warning)
			WARN(1, "Got a long running hardirq: IRQ %d in cpu: %d\n", irq, cpu_num);
		atomic64_inc(&(long_irq_stat.long_irq_count));
	}
	do {
		curr_max_irq = long_irq_stat.long_irq_arr[irq];
		if (irq_usec < curr_max_irq)
			break;
	} while (cmpxchg64(&long_irq_stat.long_irq_arr[irq],
						curr_max_irq, irq_usec) != curr_max_irq);
}

/*********************************************************************
 *                          HELPER FUNCTIONS                         *
 *********************************************************************/

static int irq_entry_cmp(const void *a, const void *b)
{
	return ((struct irq_entry *)b)->latency - ((struct irq_entry *)a)->latency;
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
	count += sysfs_emit_at(buf, count, "Max Resume Latency: %llu\n",
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

static ssize_t modify_resume_latency_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "%llu\n", resume_latency_stats.resume_latency_threshold);
}

static ssize_t modify_resume_latency_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	u64 new_threshold_ms;
	int err = kstrtou64(buf, 10, &new_threshold_ms);
	if (err)
		return err;
	resume_latency_stats.resume_latency_threshold = new_threshold_ms;
	return count;
}

static ssize_t resume_latency_display_warning_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "%d\n", resume_latency_stats.display_warning);
}

static ssize_t resume_latency_display_warning_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int err = kstrtobool(buf, &resume_latency_stats.display_warning);
	if (err)
		return err;
	return count;
}

static ssize_t long_irq_metrics_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t count = 0;
	int index;
	s64 latency;
	int irq_num;
	struct irq_entry *sorted_softirq_arr;
	struct irq_entry *sorted_irq_arr;
	sorted_softirq_arr = kmalloc(NR_SOFTIRQS * sizeof(struct irq_entry), GFP_KERNEL);
	if (!sorted_softirq_arr)
		return -ENOMEM;
	sorted_irq_arr = kmalloc(MAX_IRQ_NUM * sizeof(struct irq_entry), GFP_KERNEL);
	if (!sorted_irq_arr) {
		kfree(sorted_softirq_arr);
		return -ENOMEM;
	}
	count += sysfs_emit_at(buf, count, "long SOFTIRQ count: %lld\n",
				atomic64_read(&(long_irq_stat.long_softirq_count)));
	count += sysfs_emit_at(buf, count, "long SOFTIRQ detail (num, latency):\n");

	for (index = 0; index < NR_SOFTIRQS; index++) {
		sorted_softirq_arr[index].irq_num = index;
		sorted_softirq_arr[index].latency = long_irq_stat.long_softirq_arr[index];
	}
	sort(sorted_softirq_arr, NR_SOFTIRQS, sizeof(struct irq_entry), irq_entry_cmp, NULL);
	for (index = 0; index < NR_SOFTIRQS; index++) {
		latency = sorted_softirq_arr[index].latency;
		irq_num = sorted_softirq_arr[index].irq_num;
		if (latency > 0)
			count += sysfs_emit_at(buf, count,
				"%d %lld\n", irq_num, latency);
	}
	count += sysfs_emit_at(buf, count, "long IRQ count: %lld\n",
				atomic64_read(&(long_irq_stat.long_irq_count)));
	count += sysfs_emit_at(buf, count, "long IRQ detail (num, latency):\n");

	for (index = 0; index < MAX_IRQ_NUM; index++) {
		sorted_irq_arr[index].irq_num = index;
		sorted_irq_arr[index].latency = long_irq_stat.long_irq_arr[index];
	}
	sort(sorted_irq_arr, MAX_IRQ_NUM, sizeof(struct irq_entry), irq_entry_cmp, NULL);

	for (index = 0; index < IRQ_ARR_LIMIT; index++) {
		latency = sorted_irq_arr[index].latency;
		irq_num = sorted_irq_arr[index].irq_num;
		if (latency > 0)
			count += sysfs_emit_at(buf, count, "%d %lld\n", irq_num, latency);
	}
	kfree(sorted_softirq_arr);
	kfree(sorted_irq_arr);
	return count;
}

static ssize_t modify_softirq_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t count = 0;
	count += sysfs_emit_at(buf, count,"%lld\n", long_irq_stat.long_softirq_threshold);
	return count;
}

static ssize_t modify_softirq_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	s64 new_threshold_us;
	int err = sscanf (buf, "%lld", &new_threshold_us);
	if (!err || new_threshold_us < 0) {
		return count;
	}
	long_irq_stat.long_softirq_threshold = new_threshold_us;
	atomic64_set(&(long_irq_stat.long_softirq_count), 0);
	return count;
}

static ssize_t modify_irq_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t count = 0;
	count += sysfs_emit_at(buf, count,"%lld\n", long_irq_stat.long_irq_threshold);
	return count;
}

static ssize_t modify_irq_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	s64 new_threshold_us;
	int err = sscanf (buf, "%lld", &new_threshold_us);
	if (!err || new_threshold_us < 0) {
		return count;
	}
	long_irq_stat.long_irq_threshold = new_threshold_us;
	atomic64_set(&(long_irq_stat.long_irq_count), 0);
	return count;
}

static ssize_t irq_display_warning_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t count = 0;
	if (long_irq_stat.display_warning) {
		count += sysfs_emit_at(buf, count,"%s",
				"WARN is turned on\n");
	} else {
		count += sysfs_emit_at(buf, count,"%s",
				"WARN is turned off\n");
	}
	return count;
}

static ssize_t irq_display_warning_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int display_warn;
	int err = sscanf (buf, "%d", &display_warn);
	if (!err) {
		return count;
	}
	if (display_warn == 0) {
		long_irq_stat.display_warning = false;
	}
	if (display_warn == 1) {
		long_irq_stat.display_warning = true;
	}
	return count;
}

static struct kobj_attribute resume_latency_metrics_attr = __ATTR(resume_latency_metrics,
							  0664,
							  resume_latency_metrics_show,
							  resume_latency_metrics_store);
static struct kobj_attribute modify_resume_latency_threshold_attr = __ATTR(modify_threshold,
							  0664,
							  modify_resume_latency_threshold_show,
							  modify_resume_latency_threshold_store);
static struct kobj_attribute resume_latency_display_warning_attr = __ATTR(display_warning,
							  0664,
							  resume_latency_display_warning_show,
							  resume_latency_display_warning_store);
static struct kobj_attribute long_irq_metrics_attr = __ATTR(long_irq_metrics,
							  0444,
							  long_irq_metrics_show,
							  NULL);
static struct kobj_attribute modify_softirq_threshold_attr = __ATTR(modify_softirq_threshold,
							  0664,
							  modify_softirq_threshold_show,
							  modify_softirq_threshold_store);
static struct kobj_attribute modify_irq_threshold_attr = __ATTR(modify_irq_threshold,
							  0664,
							  modify_irq_threshold_show,
							  modify_irq_threshold_store);
static struct kobj_attribute irq_display_warning_attr = __ATTR(display_warning,
							  0664,
							  irq_display_warning_show,
							  irq_display_warning_store);

static struct attribute *irq_attrs[] = {
	&long_irq_metrics_attr.attr,
	&modify_softirq_threshold_attr.attr,
	&modify_irq_threshold_attr.attr,
	&irq_display_warning_attr.attr,
	NULL
};

static const struct attribute_group irq_attr_group = {
	.attrs = irq_attrs,
	.name = "irq"
};

static struct attribute *resume_latency_attrs[] = {
	&resume_latency_metrics_attr.attr,
	&modify_resume_latency_threshold_attr.attr,
	&resume_latency_display_warning_attr.attr,
	NULL
};

static const struct attribute_group resume_latency_attr_group = {
	.attrs = resume_latency_attrs,
	.name = "resume_latency"
};

/*********************************************************************
 *                  		INITIALIZE DRIVER                        *
 *********************************************************************/

int perf_metrics_init(struct kobject *metrics_kobj)
{
	int ret = 0;
	if (!metrics_kobj) {
		pr_err("metrics_kobj is not initialized\n");
		return -EINVAL;
	}
	if (sysfs_create_group(metrics_kobj, &resume_latency_attr_group)) {
		pr_err("failed to create resume_latency folder\n");
		return -ENOMEM;
	}
	if (sysfs_create_group(metrics_kobj, &irq_attr_group)) {
		pr_err("failed to create irq folder\n");
		return -ENOMEM;
	}
	spin_lock_init(&resume_latency_stats.resume_latency_stat_lock);
	resume_latency_stats.resume_latency_threshold = RESUME_LATENCY_DEFAULT_THRESHOLD;
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
	long_irq_stat.long_softirq_threshold = 10000;
	long_irq_stat.long_irq_threshold = 500;
	ret = register_trace_softirq_entry(hook_softirq_begin, NULL);
	if (ret) {
		pr_err("Register soft irq handler hook fail %d\n", ret);
		return ret;
	}
	ret = register_trace_softirq_exit(hook_softirq_end, NULL);
	if (ret) {
		pr_err("Register soft irq exit hook fail %d\n", ret);
		return ret;
	}
	ret = register_trace_irq_handler_entry(hook_irq_begin, NULL);
	if (ret) {
		pr_err("Register irq handler hook fail %d\n", ret);
		return ret;
	}
	ret = register_trace_irq_handler_exit(hook_irq_end, NULL);
	if (ret) {
		pr_err("Register irq exit hook fail %d\n", ret);
		return ret;
	}
	pr_info("perf_metrics driver initialized! :D\n");
	return ret;
}

