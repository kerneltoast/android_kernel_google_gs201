/*
 * Google LWIS GPIO Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-gpio: " fmt

#include <linux/gpio.h>
#include <linux/kernel.h>

#include "lwis_gpio.h"
#include "lwis_interrupt.h"

/* debug function */
void lwis_gpio_list_print(char *name, struct gpio_descs *gpios)
{
	int i;

	if (IS_ERR_OR_NULL(gpios)) {
		pr_info("name: %s error: %ld\n", name, PTR_ERR(gpios));
	} else {
		pr_info("name: %s, count: %d\n", name, gpios->ndescs);
		for (i = 0; i < gpios->ndescs; i++) {
			pr_info("gpio number: %d\n", desc_to_gpio(gpios->desc[i]));
		}
	}
}

struct gpio_descs *lwis_gpio_list_get(struct device *dev, const char *name)
{
	/* By default, the GPIO pins are acquired but uninitialized */
	return devm_gpiod_get_array(dev, name, GPIOD_ASIS);
}

void lwis_gpio_list_put(struct gpio_descs *gpios, struct device *dev)
{
	devm_gpiod_put_array(dev, gpios);
}

int lwis_gpio_list_set_output_value(struct gpio_descs *gpios, int value)
{
	int i;
	int ret;

	if (!gpios) {
		return -EINVAL;
	}

	for (i = 0; i < gpios->ndescs; ++i) {
		ret = gpiod_direction_output(gpios->desc[i], value);
		if (ret) {
			pr_err("Failed to set value for GPIO %d\n", i);
			return ret;
		}
	}

	return 0;
}

int lwis_gpio_list_set_output_value_raw(struct gpio_descs *gpios, int value)
{
	int i;
	int ret;

	if (!gpios) {
		return -EINVAL;
	}

	for (i = 0; i < gpios->ndescs; ++i) {
		ret = gpiod_direction_output_raw(gpios->desc[i], value);
		if (ret) {
			pr_err("Failed to set value for GPIO %d\n", i);
			return ret;
		}
	}

	return 0;
}

int lwis_gpio_list_set_input(struct gpio_descs *gpios)
{
	int i;
	int ret;

	if (!gpios) {
		return -EINVAL;
	}

	for (i = 0; i < gpios->ndescs; ++i) {
		ret = gpiod_direction_input(gpios->desc[i]);
		if (ret) {
			pr_err("Failed to set GPIO %d to input\n", i);
			return ret;
		}
	}

	return 0;
}

struct lwis_gpios_list *lwis_gpios_list_alloc(int count)
{
	struct lwis_gpios_list *list;

	/* No need to allocate if count is invalid */
	if (count <= 0) {
		return ERR_PTR(-EINVAL);
	}

	list = kmalloc(sizeof(struct lwis_gpios_list), GFP_KERNEL);
	if (!list) {
		pr_err("Failed to allocate gpios list\n");
		return ERR_PTR(-ENOMEM);
	}

	list->gpios_info = kmalloc(count * sizeof(struct lwis_gpios_info), GFP_KERNEL);
	if (!list->gpios_info) {
		pr_err("Failed to allocate lwis_gpios_info instances\n");
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = count;

	return list;
}

void lwis_gpios_list_free(struct lwis_gpios_list *list)
{
	if (!list) {
		return;
	}

	if (list->gpios_info->irq_list) {
		lwis_interrupt_list_free(list->gpios_info->irq_list);
	}
	if (list->gpios_info) {
		kfree(list->gpios_info);
	}

	kfree(list);
}

struct lwis_gpios_info *lwis_gpios_get_info_by_name(struct lwis_gpios_list *list, char *name)
{
	int i;

	if (!list || !name) {
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < list->count; ++i) {
		if (!strcmp(list->gpios_info[i].name, name)) {
			return &list->gpios_info[i];
		}
	}
	return ERR_PTR(-EINVAL);
}
