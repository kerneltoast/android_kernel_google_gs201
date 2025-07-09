// SPDX-License-Identifier: GPL-2.0-only
/* compaction.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include "compaction.h"

#define COMPACTION_ATTR_RW(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)
#define COMPACTION_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)
#define ATTR_ATTR(_name) (&_name##_attr.attr)

#define COMPACTION_BUCKETS 5   /* at least 2 */
#define THRESHOLD_CNT (COMPACTION_BUCKETS - 1)

#ifdef CONFIG_COMPACTION
struct compaction_pixel_stat {
	spinlock_t lock;
	unsigned long thresholds[THRESHOLD_CNT];
	unsigned long count[COMPACTION_BUCKETS];
	unsigned long total_count;
	unsigned long total_time;
	struct kobject kobj;
};
static struct compaction_pixel_stat stat = {
	.lock = __SPIN_LOCK_UNLOCKED(stat.lock),
	.thresholds = {5, 50, 100, 500}
};

/********** vendor hooks *****************************/
void vh_compaction_begin(__always_unused void *data,
		__always_unused struct compact_control *cc, long *ts)
{
	*ts = (long)jiffies;
}

void vh_compaction_end(__always_unused void *data,
		__always_unused struct compact_control *cc, long ts)
{
	int delta;
	int i;

	delta = jiffies_to_msecs(jiffies - ts);
	WARN_ON_ONCE(delta < 0);
	if (delta < 0)
		return;

	spin_lock(&stat.lock);
	stat.total_count++;
	stat.total_time += delta;
	for (i = 0; i < THRESHOLD_CNT; i++) {
		if (delta < stat.thresholds[i])
			break;
	}
	stat.count[i]++;
	spin_unlock(&stat.lock);
}

/********** sysfs *****************************/
static ssize_t mm_compaction_duration_threshold_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int n;

	spin_lock(&stat.lock);
	n = sysfs_emit(buf, "%lu %lu %lu %lu\n",
			stat.thresholds[0],
			stat.thresholds[1],
			stat.thresholds[2],
			stat.thresholds[3]);
	spin_unlock(&stat.lock);
	return n;
}

static ssize_t mm_compaction_duration_threshold_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int n;
	unsigned long th[THRESHOLD_CNT];

	n = sscanf(buf, "%lu %lu %lu %lu",
			&th[0], &th[1], &th[2], &th[3]);
	if (n != THRESHOLD_CNT) {
		pr_info("Error: Input expect %d args but got %d.\n", THRESHOLD_CNT, n);
		return -EINVAL;
	}
	for (n = 1; n < THRESHOLD_CNT; n++) {
		if (th[n-1] >= th[n]) {
			pr_info("Error: Input numbers must be ascending.\n");
			return -EINVAL;
		}
	}

	spin_lock(&stat.lock);
	memcpy(stat.thresholds, th, sizeof(th));
	spin_unlock(&stat.lock);

	return len;
}
COMPACTION_ATTR_RW(mm_compaction_duration_threshold);

static ssize_t mm_compaction_duration_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int n;

	spin_lock(&stat.lock);
	n = sysfs_emit(buf, "%ld %ld %lu %lu %lu %lu %lu\n",
			stat.total_count,
			stat.total_time,
			stat.count[0],
			stat.count[1],
			stat.count[2],
			stat.count[3],
			stat.count[4]);
	spin_unlock(&stat.lock);
	return n;
}
COMPACTION_ATTR_RO(mm_compaction_duration);

static struct attribute *mm_compaction_attrs[] = {
	ATTR_ATTR(mm_compaction_duration_threshold),
	ATTR_ATTR(mm_compaction_duration),
	NULL,
};
ATTRIBUTE_GROUPS(mm_compaction);

static struct kobj_type compaction_ktype = {
	.release = NULL,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = mm_compaction_groups,
};

void remove_compaction_sysfs(void)
{
	kobject_put(&stat.kobj);
}

int compaction_sysfs(struct kobject *parent)
{
	int ret;

	ret = kobject_init_and_add(&stat.kobj, &compaction_ktype, parent, "compaction");
	if (ret) {
		remove_compaction_sysfs();
		return ret;
	}
	return ret;
}

#endif /* CONFIG_COMPACTION */
