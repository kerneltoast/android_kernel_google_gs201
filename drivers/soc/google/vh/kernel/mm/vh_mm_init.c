// SPDX-License-Identifier: GPL-2.0-only
/* vendor_mm_init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include "linux/cpumask.h"
#include "linux/spinlock.h"
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/string.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/vmscan.h>
#include <uapi/linux/sched/types.h>

#include "../../include/pixel_mm_hint.h"

static struct task_struct *tsk_kswapd, *tsk_kcompactd;

/*
 * Last requested kcompactd CPU affinity.
 * Don't access directly, use kcompactd_requested_cpu_affinity_get()/set().
 */
static cpumask_t kcompactd_requested_cpu_affinity_value = CPU_MASK_NONE;
static DEFINE_SPINLOCK(kcompactd_requested_cpu_affinity_lock);

#define VENDOR_MM_RW(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

static void kcompactd_requested_cpu_affinity_set(const cpumask_t* affinity) {
	spin_lock(&kcompactd_requested_cpu_affinity_lock);
	cpumask_copy(&kcompactd_requested_cpu_affinity_value, affinity);
	spin_unlock(&kcompactd_requested_cpu_affinity_lock);
}

static void kcompactd_requested_cpu_affinity_get(cpumask_t* out_affinity) {
	spin_lock(&kcompactd_requested_cpu_affinity_lock);
	cpumask_copy(out_affinity, &kcompactd_requested_cpu_affinity_value);
	spin_unlock(&kcompactd_requested_cpu_affinity_lock);
}

static int task_set_uclamp_min(struct task_struct *tsk, const char *buf)
{
	struct sched_attr sched_attr = {0};
	int ret = -EINVAL;
	u32 val = 0;

	if (kstrtou32(buf, 10, &val))
		return ret;

	sched_attr.sched_flags = SCHED_FLAG_UTIL_CLAMP_MIN | SCHED_FLAG_KEEP_ALL;
	sched_attr.sched_util_min = val;

	if (tsk)
		ret = sched_setattr_nocheck(tsk, &sched_attr);

	return ret;
}

static int parse_cpu_affinity(cpumask_t *out_mask, const char *buf)
{
	cpumask_t requested_cpumask;
	int ret;

	ret = cpumask_parse(buf, &requested_cpumask);
	if (ret < 0 || cpumask_empty(&requested_cpumask))
		return -EINVAL;

	cpumask_and(out_mask, &requested_cpumask, cpu_possible_mask);
	return 0;
}

static int task_set_cpu_affinity(struct task_struct *tsk, const char *buf)
{
	cpumask_t affinity;
	int ret;

	ret = parse_cpu_affinity(&affinity, buf);
	if (ret < 0)
		return ret;

	if (tsk) {
		set_cpus_allowed_ptr(tsk, &affinity);
	}

	return 0;
}

static ssize_t kswapd_cpu_affinity_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	cpumask_t cpumask;

	if (tsk_kswapd && (tsk_kswapd->flags & PF_KSWAPD)) {
		cpumask	= tsk_kswapd->cpus_mask;
		return cpumap_print_to_pagebuf(false, buf, &cpumask);
	} else {
		/* we should never get here */
		WARN_ON(1);
		return -ESRCH;
	}
}

static ssize_t kswapd_cpu_affinity_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int ret;

	if (tsk_kswapd) {
		ret = task_set_cpu_affinity(tsk_kswapd, buf);
	} else {
		WARN_ON_ONCE(1);
		return -ESRCH;
	}

	return ret ? ret : len;;
}
VENDOR_MM_RW(kswapd_cpu_affinity);

static ssize_t kswapd_uclamp_min_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	u32 val = 0;

	if (tsk_kswapd && (tsk_kswapd->flags & PF_KSWAPD)) {
		val = tsk_kswapd->uclamp_req[UCLAMP_MIN].value;
		return sysfs_emit(buf, "%u\n", val);
	} else {
		/* we should never get here */
		WARN_ON(1);
		return -ESRCH;
	}
}

static ssize_t kswapd_uclamp_min_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int ret;

	if (tsk_kswapd) {
		ret = task_set_uclamp_min(tsk_kswapd, buf);
	} else {
		WARN_ON_ONCE(1);
		return -ESRCH;
	}

	return ret ? ret : len;
}
VENDOR_MM_RW(kswapd_uclamp_min);

static bool is_kcompactd(struct task_struct *tsk)
{
	char comm[TASK_COMM_LEN];

	return (tsk->flags & PF_KTHREAD) &&
		strstarts(get_task_comm(comm, tsk), "kcompactd");
}

static ssize_t kcompactd_uclamp_min_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	u32 val = 0;

	if (tsk_kcompactd && is_kcompactd(tsk_kcompactd)) {
		val = tsk_kcompactd->uclamp_req[UCLAMP_MIN].value;
		return sysfs_emit(buf, "%u\n", val);
	} else {
		/* we should never get here */
		WARN_ON(1);
		return -ESRCH;
	}
}

static ssize_t kcompactd_uclamp_min_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int ret;

	if (tsk_kcompactd) {
		ret = task_set_uclamp_min(tsk_kcompactd, buf);
	} else {
		WARN_ON_ONCE(1);
		return -ESRCH;
	}

	return ret ? ret : len;
}
VENDOR_MM_RW(kcompactd_uclamp_min);

static ssize_t kcompactd_cpu_affinity_show(struct kobject *kobj,
					   struct kobj_attribute *attr,
					   char *buf)
{
	cpumask_t cpumask;

	if (tsk_kcompactd && is_kcompactd(tsk_kcompactd)) {
		cpumask	= tsk_kcompactd->cpus_mask;
		return cpumap_print_to_pagebuf(false, buf, &cpumask);
	} else {
		/* we should never get here */
		WARN_ON(1);
		return -ESRCH;
	}
}

static ssize_t kcompactd_cpu_affinity_store(struct kobject *kobj,
					    struct kobj_attribute *attr,
					    const char *buf,
					    size_t len)
{
	cpumask_t affinity;
	int ret;

	if (tsk_kcompactd) {
		ret = parse_cpu_affinity(&affinity, buf);
		if (ret == 0) {
			kcompactd_requested_cpu_affinity_set(&affinity);
			set_cpus_allowed_ptr(tsk_kcompactd, &affinity);
		}
	} else {
		WARN_ON_ONCE(1);
		return -ESRCH;
	}

	return ret ? ret : len;
}

/* kcompactd_cpu_online() resets cpumask, so we need to restore it. */
static void vh_kcompactd_cpu_online(void *data, int cpu) {
	cpumask_t affinity;

	if (tsk_kcompactd) {
		kcompactd_requested_cpu_affinity_get(&affinity);
		if (!cpumask_empty(&affinity)) {
			set_cpus_allowed_ptr(tsk_kcompactd, &affinity);
		}
	}
}

VENDOR_MM_RW(kcompactd_cpu_affinity);

static struct attribute *vendor_mm_attrs[] = {
	&kswapd_cpu_affinity_attr.attr,
	&kswapd_uclamp_min_attr.attr,
	&kcompactd_cpu_affinity_attr.attr,
	&kcompactd_uclamp_min_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(vendor_mm);

static int init_get_kswapd_kcompactd_tasks(void)
{
	struct task_struct *tsk;
	int ret = -ESRCH;

	rcu_read_lock();
	for_each_process(tsk) {
		/*
		 * assume we only have 1 kcompactd and kswapd
		 * and they won't change later.
		 */
		if (tsk->flags & PF_KSWAPD) {
			tsk_kswapd = tsk;
		} else if (is_kcompactd(tsk)) {
			tsk_kcompactd = tsk;
		}

		if (tsk_kswapd && tsk_kcompactd) {
			ret = 0;
			break;
		}
	}
	rcu_read_unlock();

	return ret;
}

struct kobject *vendor_mm_kobj;
EXPORT_SYMBOL_GPL(vendor_mm_kobj);

extern int pixel_mm_cma_sysfs(struct kobject *parent);

static int vh_mm_init(void)
{
	int ret;

	ret = init_get_kswapd_kcompactd_tasks();
	if (ret)
		goto out_err;

	vendor_mm_kobj = kobject_create_and_add("vendor_mm", kernel_kobj);
	if (!vendor_mm_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(vendor_mm_kobj, vendor_mm_groups);
	if (ret)
		goto out_err;

	ret = pixel_mm_cma_sysfs(vendor_mm_kobj);
	if (ret)
		goto out_err;

	ret = register_trace_android_vh_mm_kcompactd_cpu_online(
		vh_kcompactd_cpu_online, NULL);
	if (ret)
		goto out_err;

	ret = register_trace_android_vh_tune_swappiness(vh_vmscan_tune_swappiness ,NULL);
	if (ret)
		goto out_err;

	return ret;

out_err:
	kobject_put(vendor_mm_kobj);
	return ret;
}
module_init(vh_mm_init);
MODULE_LICENSE("GPL v2");
