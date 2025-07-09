// SPDX-License-Identifier: GPL-2.0-only
/* vmscan.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include "linux/vm_event_item.h"
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/pagemap.h>

#include "../../vh/include/sched.h"

#define CREATE_TRACE_POINTS
#include "pixel_mm_trace.h"

#define OOM_SCORE_ADJ_NATIVE -1
#define OOM_SCORE_ADJ_TOP 0
#define OOM_SCORE_ADJ_VISIBLE 201

enum LATENCY_LEVEL {
	LATENCY_LOW = 0,
	LATENCY_MID,
	LATENCY_HIGH,
	LATENCY_EXTREME_HIGH,
	LATENCY_NUM_LEVELS,
};

enum OOM_SCORE_ADJ_LEVEL {
	NATIVE = 0,
	TOP,
	VISIBLE,
	OTHER,
	OOM_SCORE_ADJ_NUM_LEVELS,
};

static const char *oom_level_name[OOM_SCORE_ADJ_NUM_LEVELS] = {
	"native", "top", "visible", "other"};

static const unsigned long def_latency_threshold[LATENCY_NUM_LEVELS - 1] = {
	5, 50, 500};

struct direct_reclaim_pixel_stat {
	spinlock_t lock;
	unsigned long latency_count[LATENCY_NUM_LEVELS];
	unsigned long latency_threshold[LATENCY_NUM_LEVELS];
	unsigned long total_count;
	unsigned long total_time;
	struct kobject kobj;
};

static struct direct_reclaim_pixel_stat *stats[OOM_SCORE_ADJ_NUM_LEVELS];

void vh_direct_reclaim_begin(void *data, int order, gfp_t gfp_mask)
{
	struct vendor_task_struct *tsk;

	tsk = get_vendor_task_struct(current);
	tsk->direct_reclaim_ts = jiffies;
}

void vh_direct_reclaim_end(void *data, unsigned long nr_reclaimed)
{
	int delta;
	struct direct_reclaim_pixel_stat *stat;
	int oom_score_adj;
	int adj_lvl;
	struct vendor_task_struct *tsk;
	unsigned long old_ts;

	tsk = get_vendor_task_struct(current);
	old_ts = tsk->direct_reclaim_ts;
	oom_score_adj = current->signal->oom_score_adj;

	delta = jiffies_to_msecs(jiffies - old_ts);
	WARN_ON_ONCE(delta < 0);

	if (oom_score_adj <= OOM_SCORE_ADJ_NATIVE)
		adj_lvl = NATIVE;
	else if (oom_score_adj == OOM_SCORE_ADJ_TOP)
		adj_lvl = TOP;
	else if (oom_score_adj < OOM_SCORE_ADJ_VISIBLE)
		adj_lvl = VISIBLE;
	else
		adj_lvl = OTHER;

	stat = stats[adj_lvl];
	spin_lock(&stat->lock);
	if (delta < stat->latency_threshold[LATENCY_LOW])
		stat->latency_count[LATENCY_LOW]++;
	else if (delta < stat->latency_threshold[LATENCY_MID])
		stat->latency_count[LATENCY_MID]++;
	else if (delta < stat->latency_threshold[LATENCY_HIGH])
		stat->latency_count[LATENCY_HIGH]++;
	else
		stat->latency_count[LATENCY_EXTREME_HIGH]++;

	stat->total_count++;
	stat->total_time += delta;
	spin_unlock(&stat->lock);
}

struct vendor_madvise_walk_private {
	struct list_head ret_list;
};

void rvh_madvise_pageout_begin(void *data, void **private)
{
	struct vendor_madvise_walk_private *vendor_private;

	vendor_private = kmalloc(sizeof(struct vendor_madvise_walk_private), GFP_KERNEL);
	if (vendor_private)
		INIT_LIST_HEAD(&vendor_private->ret_list);
	*private = vendor_private;
}

void rvh_madvise_pageout_end(void *data, void *private, struct list_head *folio_list)
{
	struct vendor_madvise_walk_private *vendor_private = (struct vendor_madvise_walk_private *)private;

	if (!vendor_private)
		return;

	while (!list_empty(&vendor_private->ret_list)) {
		struct page *page;

		page = lru_to_page(&vendor_private->ret_list);
		wait_on_page_writeback(page);
		list_move(&page->lru, folio_list);
	}

	kfree(vendor_private);
}

void rvh_reclaim_folio_list(void *data, struct list_head *folio_list, void *private)
{
	struct vendor_madvise_walk_private *vendor_private;

	if (!private)
		return;

	 vendor_private = private;
	 list_splice(folio_list, &vendor_private->ret_list);
}

#define DIRECT_RECLAIM_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define DIRECT_RECLAIM_ATTR_RW(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

static ssize_t latency_threshold_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct direct_reclaim_pixel_stat *stat =
		container_of(kobj, struct direct_reclaim_pixel_stat, kobj);

	return sysfs_emit(buf, "%lu %lu %lu\n",
		stat->latency_threshold[LATENCY_LOW],
		stat->latency_threshold[LATENCY_MID],
		stat->latency_threshold[LATENCY_HIGH]);
}

static ssize_t latency_threshold_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	unsigned long low, mid, high;
	int ret;

	struct direct_reclaim_pixel_stat *stat =
		container_of(kobj, struct direct_reclaim_pixel_stat, kobj);

	ret = sscanf(buf, "%lu %lu %lu", &low, &mid, &high);

	if (ret != 3) {
		pr_err("Expect 3 args, got %d\n", ret);
		return -EINVAL;
	}

	if ((low >= mid) || (mid >= high)) {
		pr_err("Please order the numbers from least to greatest: \
			low < mid < high.\n");
		return -EINVAL;
	}
	spin_lock(&stat->lock);
	stat->latency_threshold[LATENCY_LOW] = low;
	stat->latency_threshold[LATENCY_MID] = mid;
	stat->latency_threshold[LATENCY_HIGH] = high;
	spin_unlock(&stat->lock);
	return len;
}
DIRECT_RECLAIM_ATTR_RW(latency_threshold);

static ssize_t latency_stat_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct direct_reclaim_pixel_stat *stat =
		container_of(kobj, struct direct_reclaim_pixel_stat, kobj);

	return sysfs_emit(buf, "%lu %lu %lu %lu %lu %lu\n",
		stat->total_count, (stat->total_time),
		stat->latency_count[LATENCY_LOW], stat->latency_count[LATENCY_MID],
		stat->latency_count[LATENCY_HIGH], stat->latency_count[LATENCY_EXTREME_HIGH]);
}
DIRECT_RECLAIM_ATTR_RO(latency_stat);

static struct attribute *direct_reclaim_attrs[] = {
	&latency_stat_attr.attr,
	&latency_threshold_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(direct_reclaim);

static void direct_reclaim_kobj_release(struct kobject *kobj)
{
	kfree(container_of(kobj, struct direct_reclaim_pixel_stat, kobj));
}

static struct kobj_type direct_reclaim_ktype = {
	.release = direct_reclaim_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = direct_reclaim_groups,
};

static struct kobject *pixel_vmscan_kobj;
static struct kobject *pixel_direct_reclaim_kobj;

void remove_vmscan_sysfs(void)
{
	int i;

	for (i = 0; i < OOM_SCORE_ADJ_NUM_LEVELS; i++) {
		kobject_put(&stats[i]->kobj);
		stats[i] = NULL;
	}

	kobject_put(pixel_direct_reclaim_kobj);
	kobject_put(pixel_vmscan_kobj);
}

static int add_direct_reclaim_sysfs(struct kobject *direct_reclaim_kobj)
{
	int ret;
	int i, j;

	for (i = 0; i < OOM_SCORE_ADJ_NUM_LEVELS; i++) {
		stats[i] = kzalloc(sizeof(struct direct_reclaim_pixel_stat),
				GFP_KERNEL);

		if (!stats[i])
			return -ENOMEM;

		for (j = 0; j < (LATENCY_NUM_LEVELS - 1); j++)
			stats[i]->latency_threshold[j] = def_latency_threshold[j];

		spin_lock_init(&stats[i]->lock);

		ret = kobject_init_and_add(&stats[i]->kobj, &direct_reclaim_ktype,
				direct_reclaim_kobj,
				"%s", oom_level_name[i]);
		if (ret)
			return -ENOMEM;
	}
	return 0;
}

int create_vmscan_sysfs(struct kobject *mm_kobj)
{
	int retval;

	pixel_vmscan_kobj = kobject_create_and_add("vmscan", mm_kobj);
	if (!pixel_vmscan_kobj) {
		retval = -ENOMEM;
		goto out;
	}

	pixel_direct_reclaim_kobj = kobject_create_and_add("direct_reclaim",
			pixel_vmscan_kobj);
	if (!pixel_direct_reclaim_kobj) {
		retval = -ENOMEM;
		goto err_direct_out;
	}

	retval = add_direct_reclaim_sysfs(pixel_direct_reclaim_kobj);

	if (retval) {
		remove_vmscan_sysfs();
		return retval;
	}

	return 0;

err_direct_out:
	kobject_put(pixel_vmscan_kobj);
out:
	return retval;
}

static unsigned long wake_nr_scanned;
static unsigned long wake_nr_reclaimed;

void rvh_vmscan_kswapd_wake(
	void *data,
	int node_id,
	unsigned int highest_zoneidx,
	unsigned int alloc_order)
{
	unsigned long events[NR_VM_EVENT_ITEMS];

	if (!trace_pixel_mm_kswapd_wake_enabled()) return;

	all_vm_events(events);
	wake_nr_scanned = events[PGSCAN_KSWAPD];
	wake_nr_reclaimed = events[PGSTEAL_KSWAPD];

	trace_pixel_mm_kswapd_wake(0);
}

void rvh_vmscan_kswapd_done(
	void *data,
	int node_id,
	unsigned int highest_zoneidx,
	unsigned int alloc_order,
	unsigned int reclaim_order)
{
	unsigned long events[NR_VM_EVENT_ITEMS];
	unsigned long delta_nr_scanned, delta_nr_reclaimed;

	if (!trace_pixel_mm_kswapd_done_enabled()) return;

	all_vm_events(events);
	delta_nr_scanned = events[PGSCAN_KSWAPD] - wake_nr_scanned;
	delta_nr_reclaimed = events[PGSTEAL_KSWAPD] - wake_nr_reclaimed;

	trace_pixel_mm_kswapd_done(delta_nr_scanned, delta_nr_reclaimed);
}
