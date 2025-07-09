// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF GCMA heap sysfs
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include "gcma_heap.h"
#include "gcma_heap_sysfs.h"

#if IS_ENABLED(CONFIG_VH_MM)
extern struct kobject *vendor_mm_kobj;
#endif
static struct kobject *gcma_heap_kobj;

#define GCMA_HEAP_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)
#define GCMA_HEAP_ATTR_RW(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)
#define GCMA_HEAP_ATTR_WO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_WO(_name)
#define to_gcma_heap_stat(obj) container_of(obj,struct gcma_heap_stat,kobj)

struct gcma_heap_stat {
	spinlock_t lock;
	unsigned long max_usage_bytes;
	unsigned long cur_usage_bytes;
	unsigned long allocstall_bytes;
	struct kobject kobj;
	struct gcma_heap *heap;
	char name[PATH_MAX];
};

void inc_gcma_heap_stat(struct gcma_heap *heap, enum stat_type type,
                        unsigned long size)
{
	struct gcma_heap_stat *stat = heap->stat;
	if (!stat)
		return;

	spin_lock(&stat->lock);
	if (type == USAGE) {
		stat->cur_usage_bytes += size;
		if (stat->cur_usage_bytes > stat->max_usage_bytes)
			 stat->max_usage_bytes = stat->cur_usage_bytes;
	} else if (type == ALLOCSTALL) {
		stat->allocstall_bytes += size;
	}
	spin_unlock(&stat->lock);
}

void dec_gcma_heap_stat(struct gcma_heap *heap, enum stat_type type,
                        unsigned long size)
{
	struct gcma_heap_stat *stat = heap->stat;
	if (!stat)
		return;

	spin_lock(&stat->lock);
	if (type == USAGE)
		stat->cur_usage_bytes -= size;
	else if (type == ALLOCSTALL)
		stat->allocstall_bytes -= size;
	spin_unlock(&stat->lock);
}

static ssize_t cur_usage_kb_show(struct kobject *kobj,
                                 struct kobj_attribute *attr,
                                 char *buf)
{
	struct gcma_heap_stat *stat = to_gcma_heap_stat(kobj);
	unsigned long cur_usage_bytes;

	spin_lock(&stat->lock);
	cur_usage_bytes = stat->cur_usage_bytes;
	spin_unlock(&stat->lock);

	return sysfs_emit(buf, "%lu\n", cur_usage_bytes / 1024);
}
GCMA_HEAP_ATTR_RO(cur_usage_kb);

static ssize_t max_usage_kb_store(struct kobject *kobj,
                                  struct kobj_attribute *attr,
                                  const char *buf, size_t count)
{
	struct gcma_heap_stat *stat = to_gcma_heap_stat(kobj);

	spin_lock(&stat->lock);
	stat->max_usage_bytes = 0;
	spin_unlock(&stat->lock);

	return count;
}

static ssize_t max_usage_kb_show(struct kobject *kobj,
                                 struct kobj_attribute *attr,
                                 char *buf)
{
	struct gcma_heap_stat *stat = to_gcma_heap_stat(kobj);
	unsigned long max_usage_bytes;

	spin_lock(&stat->lock);
	max_usage_bytes = stat->max_usage_bytes;
	spin_unlock(&stat->lock);

	return sysfs_emit(buf, "%lu\n", max_usage_bytes / 1024);
}
GCMA_HEAP_ATTR_RW(max_usage_kb);

static ssize_t alloc_stall_kb_store(struct kobject *kobj,
                                    struct kobj_attribute *attr,
                                    const char *buf, size_t count)
{
	struct gcma_heap_stat *stat = to_gcma_heap_stat(kobj);

	spin_lock(&stat->lock);
	stat->allocstall_bytes = 0;
	spin_unlock(&stat->lock);

	return count;
}

static ssize_t alloc_stall_kb_show(struct kobject *kobj,
                                   struct kobj_attribute *attr,
                                   char *buf)
{
	struct gcma_heap_stat *stat = to_gcma_heap_stat(kobj);
	unsigned long allocstall_bytes;

	spin_lock(&stat->lock);
	allocstall_bytes = stat->allocstall_bytes;
	spin_unlock(&stat->lock);

	return sysfs_emit(buf, "%lu\n", allocstall_bytes / 1024);
}
GCMA_HEAP_ATTR_RW(alloc_stall_kb);

static ssize_t force_empty_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t len)
{
	struct gcma_heap_stat *stat = to_gcma_heap_stat(kobj);
	struct gcma_heap *gcma_heap = stat->heap;
	struct page *page;
	unsigned int req_pages;
	unsigned long req_size;
	char *name = stat->name;

	if (kstrtouint(buf, 0, &req_pages))
		return -EINVAL;

	req_size = req_pages * PAGE_SIZE;

	page = gcma_alloc(gcma_heap, req_size);
	pr_info("%s req_pages %d force_empty %s\n", name, req_pages,
		page ? "succeeded" : "failed");
	if (page)
		gcma_free(gcma_heap->pool, page);
	return page ? len : -ENOMEM;
}
GCMA_HEAP_ATTR_WO(force_empty);

static struct attribute *gcma_heap_attrs[] = {
	&cur_usage_kb_attr.attr,
	&max_usage_kb_attr.attr,
	&alloc_stall_kb_attr.attr,
	&force_empty_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gcma_heap);

static void gcma_heap_kobj_release(struct kobject *obj)
{
	/* Never released the static objects */
}

static struct kobj_type gcma_heap_ktype = {
	.release = gcma_heap_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = gcma_heap_groups,
};

int register_heap_sysfs(struct gcma_heap *heap, const char *name)
{
	int ret = -ENOMEM;
	struct gcma_heap_stat *stat;

	if (!gcma_heap_kobj)
		return ret;

	stat = kzalloc(sizeof(struct gcma_heap_stat), GFP_KERNEL);
	if (!stat) {
		 pr_err("fail to allocate %s gcma heap stat", name);
		 return -ENOMEM;
	}

	spin_lock_init(&stat->lock);

	ret = kobject_init_and_add(&stat->kobj, &gcma_heap_ktype,
		  gcma_heap_kobj, name);
	if (ret) {
		pr_err("register gcma pdev= %s sysfs fail", name);
		kobject_put(&stat->kobj);
		goto out;
	}
	heap->stat = stat;
	stat->heap = heap;
	strncpy(stat->name, name, sizeof(stat->name));
out:
	return ret;
}

int __init gcma_heap_sysfs_init(void)
{
#if IS_ENABLED(CONFIG_VH_MM)
	gcma_heap_kobj = kobject_create_and_add("gcma_heap", vendor_mm_kobj);
#else
	gcma_heap_kobj = kobject_create_and_add("gcma_heap", kernel_kobj);
#endif
	if (!gcma_heap_kobj) {
		pr_err("init gcma heap sysfs fail");
		return -ENOMEM;
	}

	return 0;
}

