// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Regulator Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-reg: " fmt

#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_regulator.h"

struct lwis_regulator_list *lwis_regulator_list_alloc(int num_regs)
{
	struct lwis_regulator_list *list;

	if (num_regs < 0)
		return ERR_PTR(-EINVAL);

	list = kmalloc(sizeof(struct lwis_regulator_list), GFP_KERNEL);
	if (!list)
		return ERR_PTR(-ENOMEM);

	list->reg = kcalloc(num_regs, sizeof(struct lwis_regulator), GFP_KERNEL);
	if (!list->reg) {
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = num_regs;

	return list;
}

void lwis_regulator_list_free(struct lwis_regulator_list *list)
{
	if (!list)
		return;

	kfree(list->reg);
	kfree(list);
}

int lwis_regulator_get(struct lwis_regulator_list *list, char *name, int voltage,
		       struct device *dev)
{
	struct regulator *reg;
	int i;
	int index = -1;

	if (!list || !dev)
		return -EINVAL;

	/* Look for empty slot and duplicate entries */
	for (i = 0; i < list->count; ++i) {
		if (list->reg[i].reg == NULL) {
			index = i;
		} else if (!strcmp(list->reg[i].name, name)) {
			pr_info("Regulator %s already allocated\n", name);
			return i;
		}
	}

	/* No empty slot */
	if (index < 0) {
		pr_err("No empty slots in the lwis_regulator struct\n");
		return -ENOMEM;
	}

	/* Make sure regulator exists */
	reg = devm_regulator_get(dev, name);
	if (IS_ERR_OR_NULL(reg))
		return PTR_ERR(reg);

	list->reg[index].reg = reg;
	strscpy(list->reg[index].name, name, LWIS_MAX_NAME_STRING_LEN);
	list->reg[index].voltage = voltage;

	return index;
}

int lwis_regulator_put_by_idx(struct lwis_regulator_list *list, int index)
{
	if (!list || index < 0 || index >= list->count)
		return -EINVAL;

	if (IS_ERR_OR_NULL(list->reg[index].reg))
		return -EINVAL;

	devm_regulator_put(list->reg[index].reg);
	memset(list->reg + index, 0, sizeof(struct lwis_regulator));

	return 0;
}

int lwis_regulator_put_by_name(struct lwis_regulator_list *list, char *name)
{
	if (!list)
		return -EINVAL;

	/* Find entry by name */
	for (int i = 0; i < list->count; ++i) {
		if (!strcmp(list->reg[i].name, name)) {
			if (IS_ERR_OR_NULL(list->reg[i].reg))
				return -EINVAL;

			devm_regulator_put(list->reg[i].reg);
			memset(list->reg + i, 0, sizeof(struct lwis_regulator));
			return 0;
		}
	}

	pr_err("Regulator %s not found\n", name);
	return -EINVAL;
}

int lwis_regulator_put_all(struct lwis_regulator_list *list)
{
	int ret = 0;

	if (!list)
		return -EINVAL;

	for (int i = 0; i < list->count; ++i)
		ret = lwis_regulator_put_by_idx(list, i);

	return ret;
}

int lwis_regulator_enable_by_idx(struct lwis_regulator_list *list, int index)
{
	int ret = 0;
	struct lwis_regulator *lwis_reg;

	if (!list)
		return -EINVAL;

	lwis_reg = &list->reg[index];
	if (lwis_reg->voltage > 0) {
		ret = regulator_set_voltage(lwis_reg->reg, lwis_reg->voltage, lwis_reg->voltage);
		if (ret) {
			pr_err("Failed to set regulator %s voltage to %d\n", lwis_reg->name,
			       lwis_reg->voltage);
			return ret;
		}
	}

	return regulator_enable(list->reg[index].reg);
}

int lwis_regulator_enable_by_name(struct lwis_regulator_list *list, char *name)
{
	if (!list)
		return -EINVAL;

	for (int i = 0; i < list->count; ++i) {
		if (!strcmp(list->reg[i].name, name))
			return lwis_regulator_enable_by_idx(list, i);
	}

	/* No entry found */
	pr_err("Regulator %s not found\n", name);
	return -ENOENT;
}

int lwis_regulator_enable_all(struct lwis_regulator_list *list)
{
	int i;
	int ret;

	for (i = 0; i < list->count; ++i) {
		ret = lwis_regulator_enable_by_idx(list, i);
		if (ret) {
			pr_err("Error enabling regulator %s\n", list->reg[i].name);
			return ret;
		}
	}

	return 0;
}

int lwis_regulator_disable_by_idx(struct lwis_regulator_list *list, int index)
{
	if (!list)
		return -EINVAL;

	return regulator_disable(list->reg[index].reg);
}

int lwis_regulator_disable_by_name(struct lwis_regulator_list *list, char *name)
{
	if (!list)
		return -EINVAL;

	for (int i = 0; i < list->count; ++i) {
		if (!strcmp(list->reg[i].name, name))
			return regulator_disable(list->reg[i].reg);
	}

	/* No entry found */
	pr_err("Regulator %s not found\n", name);
	return -ENOENT;
}

int lwis_regulator_disable_all(struct lwis_regulator_list *list)
{
	int i;
	int ret;

	for (i = 0; i < list->count; ++i) {
		ret = lwis_regulator_disable_by_idx(list, i);
		if (ret) {
			pr_err("Error disabling regulator %s\n", list->reg[i].name);
			return ret;
		}
	}

	return 0;
}

void lwis_regulator_print(struct lwis_regulator_list *list)
{
	for (int i = 0; i < list->count; ++i)
		pr_info("%s: reg: %s voltage: %d\n", __func__, list->reg[i].name,
			list->reg[i].voltage);
}
