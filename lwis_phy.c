// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS PHY Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-phy: " fmt

#include "lwis_phy.h"

#include <linux/kernel.h>
#include <linux/slab.h>

struct lwis_phy_list *lwis_phy_list_alloc(int count)
{
	struct lwis_phy_list *list;

	/* No need to allocate if count is invalid */
	if (count <= 0)
		return ERR_PTR(-EINVAL);

	list = kmalloc(sizeof(struct lwis_phy_list), GFP_KERNEL);
	if (!list)
		return ERR_PTR(-ENOMEM);

	list->phy = kcalloc(count, sizeof(struct phy), GFP_KERNEL);
	if (!list->phy) {
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = count;

	return list;
}

void lwis_phy_list_free(struct lwis_phy_list *list)
{
	if (!list)
		return;

	kfree(list->phy);
	kfree(list);
}

int lwis_phy_get(struct lwis_phy_list *list, char *name, struct device *dev)
{
	struct phy *phy;
	int i;
	int index = -1;

	if (!list || !dev)
		return -EINVAL;

	/* Look for empty slot and duplicate entries */
	for (i = 0; i < list->count; ++i) {
		if (list->phy[i].phy == NULL) {
			index = i;
		} else if (!strcmp(list->phy[i].name, name)) {
			pr_info("PHY %s already allocated\n", name);
			return i;
		}
	}

	/* No empty slots */
	if (index < 0) {
		pr_err("No empty slots in the lwis_phy struct\n");
		return -ENOMEM;
	}

	/* Make sure PHY exists */
	phy = devm_phy_get(dev, name);
	if (IS_ERR_OR_NULL(phy)) {
		pr_err("PHY %s not found\n", name);
		return PTR_ERR(phy);
	}

	list->phy[index].phy = phy;
	list->phy[index].name = name;

	return index;
}

int lwis_phy_put_by_idx(struct lwis_phy_list *list, int index, struct device *dev)
{
	if (!list || index < 0 || index >= list->count)
		return -EINVAL;

	if (IS_ERR_OR_NULL(list->phy[index].phy))
		return -EINVAL;

	devm_phy_put(dev, list->phy[index].phy);
	memset(list->phy + index, 0, sizeof(struct lwis_phy));

	return 0;
}

int lwis_phy_put_by_name(struct lwis_phy_list *list, char *name, struct device *dev)
{
	if (!dev || !list)
		return -EINVAL;

	/* Find entry by name */
	for (int i = 0; i < list->count; ++i) {
		if (!strcmp(list->phy[i].name, name)) {
			if (IS_ERR_OR_NULL(list->phy[i].phy))
				return -EINVAL;

			devm_phy_put(dev, list->phy[i].phy);
			memset(list->phy + i, 0, sizeof(struct lwis_phy));
			return 0;
		}
	}

	pr_err("PHY %s not found\n", name);
	return -EINVAL;
}

int lwis_phy_set_power_by_idx(struct lwis_phy_list *list, int index, bool power_on)
{
	if (!list || index < 0 || index >= list->count)
		return -EINVAL;

	if (power_on)
		return phy_power_on(list->phy[index].phy);

	return phy_power_off(list->phy[index].phy);
}

int lwis_phy_set_power_by_name(struct lwis_phy_list *list, char *name, bool power_on)
{
	if (!list)
		return -EINVAL;

	/* Find entry by name */
	for (int i = 0; i < list->count; ++i) {
		if (!strcmp(list->phy[i].name, name))
			return lwis_phy_set_power_by_idx(list, i, power_on);
	}

	/* Entry not found */
	pr_err("PHY %s not found\n", name);
	return -ENOENT;
}

int lwis_phy_set_power_all(struct lwis_phy_list *list, bool power_on)
{
	int i;
	int ret;

	if (!list)
		return -EINVAL;

	for (i = 0; i < list->count; ++i) {
		ret = lwis_phy_set_power_by_idx(list, i, power_on);
		if (ret) {
			pr_err("Failed to set PHY power\n");
			return ret;
		}
	}

	return 0;
}

void lwis_phy_print(struct lwis_phy_list *list)
{
	for (int i = 0; i < list->count; ++i)
		pr_info("%s: PHY: %s\n", __func__, list->phy[i].name);
}
