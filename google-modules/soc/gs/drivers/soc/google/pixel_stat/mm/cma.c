// SPDX-License-Identifier: GPL-2.0-only
/* cma.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/mm.h>
#include <linux/cma.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include "../../vh/include/sched.h"

#define DEF_LATENCY_MID_BOUND_MS 1500
#define DEF_LATENCY_LOW_BOUND_MS 500

enum LATENCY_LEVEL {
	LATENCY_LOW = 0,
	LATENCY_MID,
	LATENCY_HIGH,
	LATENCY_NUM_LEVELS,
};

struct cma_pixel_stat {
	spinlock_t lock;
	unsigned long latency[LATENCY_NUM_LEVELS];
	unsigned long bound[LATENCY_NUM_LEVELS];
	struct kobject kobj;
};

struct cma_index {
	const char *name;
	int no;
};

static struct cma_pixel_stat *stats[MAX_CMA_AREAS];

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */

void vh_cma_alloc_start(void *data, const char *name, unsigned long count,
			unsigned int align)
{
	struct vendor_task_struct *tsk;

	tsk = get_vendor_task_struct(current);
	set_vendor_task_struct_private(tsk, jiffies);
}

struct cma *cma;

static int parse_cma_idx(struct cma *cma, void *data)
{
	struct cma_index *arg = data;

	if (arg->name &&
	    !strncmp(cma_get_name(cma), arg->name, strlen(arg->name))) {
		/* bail out the loop from cma_for_each_area */
		return 1;
	}

	arg->no++;
	return 0;
}

void vh_cma_alloc_finish(void *data, const char *name, unsigned long pfn,
			 const struct page *page, unsigned long count,
			 unsigned int align)
{
	struct cma_pixel_stat *cma_stat;
	struct cma_index index = {
		.name = name,
	};

	s64 delta;
	struct vendor_task_struct *tsk;
	unsigned long old_ts;

	tsk = get_vendor_task_struct(current);
	old_ts = get_and_reset_vendor_task_struct_private(tsk);

	delta = jiffies_to_msecs(jiffies - old_ts);
	WARN_ON_ONCE(delta < 0);

	cma_for_each_area(parse_cma_idx, &index);
	cma_stat = stats[index.no];

	spin_lock(&cma_stat->lock);
	if (delta < cma_stat->bound[LATENCY_LOW])
		cma_stat->latency[LATENCY_LOW]++;
	else if (delta < cma_stat->bound[LATENCY_MID])
		cma_stat->latency[LATENCY_MID]++;
	else
		cma_stat->latency[LATENCY_HIGH]++;
	spin_unlock(&cma_stat->lock);
}

#define CMA_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define CMA_ATTR_RW(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

static ssize_t latency_low_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct cma_pixel_stat *cma_stat =
		container_of(kobj, struct cma_pixel_stat, kobj);

	return sysfs_emit(buf, "%lu\n", cma_stat->latency[LATENCY_LOW]);
}
CMA_ATTR_RO(latency_low);

static ssize_t latency_mid_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct cma_pixel_stat *cma_stat =
		container_of(kobj, struct cma_pixel_stat, kobj);

	return sysfs_emit(buf, "%lu\n", cma_stat->latency[LATENCY_MID]);
}
CMA_ATTR_RO(latency_mid);

static ssize_t latency_high_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct cma_pixel_stat *cma_stat =
		container_of(kobj, struct cma_pixel_stat, kobj);

	return sysfs_emit(buf, "%lu\n", cma_stat->latency[LATENCY_HIGH]);
}
CMA_ATTR_RO(latency_high);

static ssize_t latency_low_bound_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct cma_pixel_stat *stat =
		container_of(kobj, struct cma_pixel_stat, kobj);

	return sysfs_emit(buf, "%lu\n", stat->bound[LATENCY_LOW]);
}

static ssize_t latency_low_bound_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	unsigned long val;
	struct cma_pixel_stat *cma_stat =
		container_of(kobj, struct cma_pixel_stat, kobj);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val >= cma_stat->bound[LATENCY_MID]) {
		pr_info("latency_low_bound should be less than latency_mid_bound %lu\n",
				cma_stat->bound[LATENCY_MID]);
		return -EINVAL;
	}

	cma_stat->bound[LATENCY_LOW] = val;
	return len;
}
CMA_ATTR_RW(latency_low_bound);

static ssize_t latency_mid_bound_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct cma_pixel_stat *cma_stat =
		container_of(kobj, struct cma_pixel_stat, kobj);

	return sysfs_emit(buf, "%lu\n", cma_stat->bound[LATENCY_MID]);
}

static ssize_t latency_mid_bound_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	unsigned long val;
	struct cma_pixel_stat *cma_stat =
		container_of(kobj, struct cma_pixel_stat, kobj);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val <= cma_stat->bound[LATENCY_LOW]) {
		pr_info("latency_mid_bound should be greater than latency_low_bound %lu\n",
				cma_stat->bound[LATENCY_LOW]);
		return -EINVAL;
	}

	cma_stat->bound[LATENCY_MID] = val;
	return len;
}
CMA_ATTR_RW(latency_mid_bound);

static struct attribute *cma_attrs[] = {
	&latency_low_attr.attr,
	&latency_mid_attr.attr,
	&latency_high_attr.attr,
	&latency_mid_bound_attr.attr,
	&latency_low_bound_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cma);

static void cma_kobj_release(struct kobject *kobj)
{
	kfree(container_of(kobj, struct cma_pixel_stat, kobj));
}

static struct kobj_type cma_ktype = {
	.release = cma_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = cma_groups,
};

static struct kobject *pixel_cma_kobj;

static int add_cma_sysfs(struct cma *cma, void *data)
{
	struct cma_pixel_stat *cma_stat;
	int *cma_idx = data;
	int ret;

	cma_stat = kzalloc(sizeof(*cma_stat), GFP_KERNEL);
	if (!cma_stat)
		return -ENOMEM;

	cma_stat->bound[LATENCY_MID] = DEF_LATENCY_MID_BOUND_MS;
	cma_stat->bound[LATENCY_LOW] = DEF_LATENCY_LOW_BOUND_MS;
	spin_lock_init(&cma_stat->lock);

	ret = kobject_init_and_add(&cma_stat->kobj, &cma_ktype,
			pixel_cma_kobj,
			"%s", cma_get_name(cma));
	if (ret) {
		kobject_put(&cma_stat->kobj);
		return ret;
	}

	stats[*cma_idx] = cma_stat;

	*cma_idx += 1;
	return 0;
}

static int remove_cma_sysfs_one(struct cma *cma, void *data)
{
	int cma_idx = *((int *)data);

	cma_idx--;
	if (cma_idx < 0)
		return -EINVAL;

	kobject_put(&stats[cma_idx]->kobj);
	stats[cma_idx] = NULL;
	*((int *)data) = cma_idx;
	return 0;
}

void remove_cma_sysfs(void)
{
	int cma_idx;

	for (cma_idx = 0; cma_idx < MAX_CMA_AREAS; cma_idx++) {
		kobject_put(&stats[cma_idx]->kobj);
		stats[cma_idx] = NULL;
	}
}

int create_cma_sysfs(struct kobject *mm_kobj)
{
	int ret;
	int cma_idx = 0;

	pixel_cma_kobj = kobject_create_and_add("cma", mm_kobj);
	if (!pixel_cma_kobj)
		return -ENOMEM;

	ret = cma_for_each_area(add_cma_sysfs, &cma_idx);
	if (ret)
		cma_for_each_area(remove_cma_sysfs_one, &cma_idx);

	return ret;
}
