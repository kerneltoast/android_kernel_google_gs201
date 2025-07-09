// SPDX-License-Identifier: GPL-2.0-only
/* cma.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/mm.h>
#include <linux/cma.h>
#include <linux/kobject.h>
#include <linux/slab.h>

extern struct kobject *vendor_mm_kobj;

struct cma_node {
	struct kobject kobj;
	struct cma *cma;
};

static struct cma_node *nodes[MAX_CMA_AREAS];

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */

#define CMA_ATTR_WO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_WO(_name)

static ssize_t force_empty_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	struct cma_node *node =
		container_of(kobj, struct cma_node, kobj);
	struct cma *cma = node->cma;
	struct page *page;
	int nr_retry;
	unsigned int req_pages;

	if (kstrtouint(buf, 0, &req_pages))
		return -EINVAL;

	for (nr_retry = 0; nr_retry < 5; nr_retry++) {
		page = cma_alloc(cma, req_pages, 0, false);
		pr_info("%s req_pages %d force_empty %s\n", cma_get_name(cma),
					req_pages, page ? "succeeded" : "failed");
		if (page)
			break;
	}

	cma_release(cma, page, req_pages);
	return page ? len : -ENOMEM;
}
CMA_ATTR_WO(force_empty);

static struct attribute *cma_attrs[] = {
	&force_empty_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cma);

static void cma_kobj_release(struct kobject *kobj)
{
	kfree(container_of(kobj, struct cma_node, kobj));
}

static struct kobj_type cma_ktype = {
	.release = cma_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = cma_groups,
};

static struct kobject *pixel_cma_kobj;

static int add_cma_sysfs(struct cma *cma, void *data)
{
	struct cma_node *node;
	int *cma_idx = data;
	int ret;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	ret = kobject_init_and_add(&node->kobj, &cma_ktype,
				   pixel_cma_kobj,
				   "%s", cma_get_name(cma));
	if (ret) {
		kobject_put(&node->kobj);
		return ret;
	}

	node->cma = cma;
	nodes[*cma_idx] = node;

	*cma_idx += 1;
	return 0;
}

static int remove_cma_sysfs(struct cma *cma, void *data)
{
	int cma_idx = *((int *)data);

	cma_idx--;
	if (cma_idx < 0)
		return -EINVAL;

	kobject_put(&nodes[cma_idx]->kobj);
	nodes[cma_idx] = NULL;
	*((int *)data) = cma_idx;
	return 0;
}

int pixel_mm_cma_sysfs(struct kobject *parent)
{
	int ret;
	int cma_idx = 0;

	pixel_cma_kobj = kobject_create_and_add("cma", parent);
	if (!pixel_cma_kobj)
		return -ENOMEM;

	ret = cma_for_each_area(add_cma_sysfs, &cma_idx);
	if (ret)
		cma_for_each_area(remove_cma_sysfs, &cma_idx);

	return ret;
}
