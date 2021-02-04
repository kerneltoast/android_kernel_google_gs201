// SPDX-License-Identifier: GPL-2.0-only
/* sysfs_node.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sysfs.h>

#include "sched.h"

extern void reset_uclamp_stats(void);
DECLARE_PER_CPU(struct uclamp_stats, uclamp_stats);

bool __read_mostly vendor_sched_enable_prefer_high_cap;
static struct kobject *vendor_sched_kobj;

static int update_prefer_high_cap(const char *buf, bool val)
{
	struct vendor_task_struct *vp;
	struct task_struct *p;
	pid_t pid;

	if (kstrtoint(buf, 0, &pid) || pid <= 0)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(p);
	rcu_read_unlock();

	vp = get_vendor_task_struct(p);
	vp->prefer_high_cap = val;

	put_task_struct(p);

	return 0;
}

static ssize_t set_prefer_high_cap_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	int ret = update_prefer_high_cap(buf, true);

	return ret ?: count;
}

static struct kobj_attribute set_prefer_high_cap_attribute = __ATTR_WO(set_prefer_high_cap);

static ssize_t clear_prefer_high_cap_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	int ret = update_prefer_high_cap(buf, false);

	return ret ?: count;
}

static struct kobj_attribute clear_prefer_high_cap_attribute = __ATTR_WO(clear_prefer_high_cap);

static ssize_t prefer_high_cap_enable_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vendor_sched_enable_prefer_high_cap);
}

static ssize_t prefer_high_cap_enable_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	bool enable;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	vendor_sched_enable_prefer_high_cap = enable;

	return count;
}

static struct kobj_attribute prefer_high_cap_enable_attribute = __ATTR_RW(prefer_high_cap_enable);

static ssize_t uclamp_stats_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i, j, index;
	struct uclamp_stats *stats;
	ssize_t len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len, "value, time(ns), %\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);

		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d - uclamp.min\n", i);
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %d%%\n", index,
					stats->time_in_state_min[j],
					stats->time_in_state_min[j] / (stats->total_time / 100));
			if (len >= PAGE_SIZE)
				break;
		}

		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d - uclamp.max\n", i);
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %d%%\n", index,
					stats->time_in_state_max[j],
					stats->time_in_state_max[j] / (stats->total_time / 100));
			if (len >= PAGE_SIZE)
				break;
		}
	}

	return len;
}

static struct kobj_attribute uclamp_stats_attribute = __ATTR_RO(uclamp_stats);

static ssize_t uclamp_effective_stats_show(struct kobject *kobj, struct kobj_attribute *attr,
					   char *buf)
{
	int i, j, index;
	struct uclamp_stats *stats;
	ssize_t len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len, "value, time(ns), %\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);

		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d - uclamp.min\n", i);
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %llu%%\n", index,
					stats->effect_time_in_state_min[j],
					stats->effect_time_in_state_min[j] /
					(stats->time_in_state_min[j] / 100));
			if (len >= PAGE_SIZE)
				break;
		}

		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d - uclamp.max\n", i);
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %llu%%\n", index,
					stats->effect_time_in_state_max[j],
					stats->effect_time_in_state_max[j] /
					(stats->time_in_state_max[j] / 100));
			if (len >= PAGE_SIZE)
				break;
		}
	}

	return len;
}

static struct kobj_attribute uclamp_effective_stats_attribute = __ATTR_RO(uclamp_effective_stats);

static ssize_t uclamp_util_diff_stats_show(struct kobject *kobj, struct kobj_attribute *attr,
					   char *buf)
{
	int i, j, index;
	struct uclamp_stats *stats;
	ssize_t len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len, "value, time(ns), %\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);
		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d - util_diff_min\n", i);
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %llu%%\n", index,
					stats->util_diff_min[j],
					stats->util_diff_min[j] / (stats->total_time / 100));
			if (len >= PAGE_SIZE)
				break;
		}

		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d - util_diff_max\n", i);
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index -= UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %llu%%\n", index,
					stats->util_diff_max[j],
					stats->util_diff_max[j] / (stats->total_time / 100));
			if (len >= PAGE_SIZE)
				break;
		}
	}

	return len;
}

static struct kobj_attribute uclamp_util_diff_stats_attribute = __ATTR_RO(uclamp_util_diff_stats);


static ssize_t reset_uclamp_stats_store(struct kobject *kobj, struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	bool reset;

	if (kstrtobool(buf, &reset))
		return -EINVAL;

	if (reset)
		reset_uclamp_stats();

	return count;
}

static struct kobj_attribute reset_uclamp_stats_attribute = __ATTR_WO(reset_uclamp_stats);

static struct attribute *attrs[] = {
	&set_prefer_high_cap_attribute.attr,
	&clear_prefer_high_cap_attribute.attr,
	&prefer_high_cap_enable_attribute.attr,
	&uclamp_stats_attribute.attr,
	&uclamp_effective_stats_attribute.attr,
	&uclamp_util_diff_stats_attribute.attr,
	&reset_uclamp_stats_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

int create_sysfs_node(void)
{
	int ret;

	vendor_sched_kobj = kobject_create_and_add("vendor_sched", kernel_kobj);

	if (!vendor_sched_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(vendor_sched_kobj, &attr_group);

	if (ret)
		kobject_put(vendor_sched_kobj);

	return ret;
}
