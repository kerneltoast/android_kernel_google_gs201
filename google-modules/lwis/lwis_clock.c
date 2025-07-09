// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Clock Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-clock: " fmt

#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_clock.h"

struct lwis_clock_list *lwis_clock_list_alloc(int num_clks)
{
	struct lwis_clock_list *list;

	/* No need to allocate if num_clks is invalid */
	if (num_clks <= 0)
		return ERR_PTR(-EINVAL);

	list = kmalloc(sizeof(struct lwis_clock_list), GFP_KERNEL);
	if (!list)
		return ERR_PTR(-ENOMEM);

	list->clk = kcalloc(num_clks, sizeof(struct lwis_clock), GFP_KERNEL);
	if (!list->clk) {
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = num_clks;

	return list;
}

void lwis_clock_list_free(struct lwis_clock_list *list)
{
	if (!list)
		return;

	kfree(list->clk);
	kfree(list);
}

int lwis_clock_get(struct lwis_clock_list *list, char *name, struct device *dev, uint32_t rate)
{
	struct clk *clk;
	int i;
	int index = -1;

	if (!dev || !list)
		return -EINVAL;

	/* Look for empty slot and duplicate entries */
	for (i = 0; i < list->count; ++i) {
		if (list->clk[i].clk == NULL) {
			index = i;
			break;
		} else if (!strcmp(list->clk[i].name, name)) {
			pr_info("Clock %s already allocated\n", name);
			return i;
		}
	}

	/* No empty slot */
	if (index < 0) {
		pr_err("No empty slots in the lwis_clock struct\n");
		return -ENOMEM;
	}

	/* Make sure clock exists */
	clk = devm_clk_get(dev, name);
	if (IS_ERR_OR_NULL(clk)) {
		pr_err("Clock %s not found\n", name);
		return PTR_ERR(clk);
	}

	list->clk[index].clk = clk;
	list->clk[index].name = name;
	list->clk[index].rate = rate;

	return index;
}

int lwis_clock_put_by_idx(struct lwis_clock_list *list, int index, struct device *dev)
{
	if (!dev || !list || index < 0 || index >= list->count)
		return -EINVAL;

	if (IS_ERR_OR_NULL(list->clk[index].clk))
		return -EINVAL;

	devm_clk_put(dev, list->clk[index].clk);
	memset(list->clk + index, 0, sizeof(struct lwis_clock));

	return 0;
}

int lwis_clock_put_by_name(struct lwis_clock_list *list, char *name, struct device *dev)
{
	if (!dev || !list)
		return -EINVAL;

	/* Find entry by name */
	for (int i = 0; i < list->count; ++i) {
		if (!strcmp(list->clk[i].name, name)) {
			if (IS_ERR_OR_NULL(list->clk[i].clk))
				return -EINVAL;

			devm_clk_put(dev, list->clk[i].clk);
			memset(list->clk + i, 0, sizeof(struct lwis_clock));
			return 0;
		}
	}

	pr_err("Clock %s not found\n", name);
	return -EINVAL;
}

int lwis_clock_enable_by_idx(struct lwis_clock_list *list, int index)
{
	int ret = 0;

	if (!list || index < 0 || index >= list->count)
		return -EINVAL;

	ret = clk_prepare_enable(list->clk[index].clk);
	if (ret)
		return ret;

	if (list->clk[index].rate > 0)
		ret = clk_set_rate(list->clk[index].clk, list->clk[index].rate);

	return ret;
}

int lwis_clock_enable_by_name(struct lwis_clock_list *list, char *name)
{
	if (!list)
		return -EINVAL;

	for (int i = 0; i < list->count; ++i) {
		if (!strcmp(list->clk[i].name, name))
			return lwis_clock_enable_by_idx(list, i);
	}

	pr_err("Clock %s not found\n", name);
	return -ENOENT;
}

int lwis_clock_enable_all(struct lwis_clock_list *list)
{
	int ret;

	if (!list)
		return -EINVAL;

	for (int i = 0; i < list->count; ++i) {
		ret = lwis_clock_enable_by_idx(list, i);
		if (ret) {
			pr_err("Error enabling clock %s\n", list->clk[i].name);
			return ret;
		}
	}

	return 0;
}

void lwis_clock_disable_by_idx(struct lwis_clock_list *list, int index)
{
	if (!list || index < 0 || index >= list->count)
		return;

	clk_disable_unprepare(list->clk[index].clk);
}

void lwis_clock_disable_by_name(struct lwis_clock_list *list, char *name)
{
	if (!list)
		return;

	for (int i = 0; i < list->count; ++i) {
		if (!strcmp(list->clk[i].name, name)) {
			lwis_clock_disable_by_idx(list, i);
			return;
		}
	}

	pr_err("Clock %s not found\n", name);
}

void lwis_clock_disable_all(struct lwis_clock_list *list)
{
	if (!list)
		return;

	for (int i = 0; i < list->count; ++i)
		lwis_clock_disable_by_idx(list, i);
}

void lwis_clock_print(struct lwis_clock_list *list)
{
	for (int i = 0; i < list->count; ++i)
		pr_info("%s: %s: rate: %d\n", __func__, list->clk[i].name, list->clk[i].rate);
}
