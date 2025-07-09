// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS GPIO Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-gpio: " fmt

#include <linux/gpio.h>
#include <linux/kernel.h>

#include "lwis_gpio.h"
#include "lwis_interrupt.h"

#define SHARED_STRING "shared-"
#define PULSE_STRING "pulse-"

/* debug function */
void lwis_gpio_list_print(char *name, struct gpio_descs *gpios)
{
	if (IS_ERR_OR_NULL(gpios)) {
		pr_info("name: %s error: %ld\n", name, PTR_ERR(gpios));
	} else {
		pr_info("name: %s, count: %d\n", name, gpios->ndescs);
		for (int i = 0; i < gpios->ndescs; i++)
			pr_info("gpio number: %d\n", desc_to_gpio(gpios->desc[i]));
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

	if (!gpios)
		return -EINVAL;

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

	if (!gpios)
		return -EINVAL;

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

	if (!gpios)
		return -EINVAL;

	for (i = 0; i < gpios->ndescs; ++i) {
		ret = gpiod_direction_input(gpios->desc[i]);
		if (ret) {
			pr_err("Failed to set GPIO %d to input\n", i);
			return ret;
		}
	}

	return 0;
}

int lwis_gpios_list_add_info_by_name(struct device *dev, struct list_head *list, const char *name)
{
	struct lwis_gpios_info *gpios_info;
	struct gpio_descs *descs;

	/* Check gpio already exist or not */
	gpios_info = lwis_gpios_get_info_by_name(list, name);
	if (!IS_ERR_OR_NULL(gpios_info))
		return 0;

	gpios_info = kmalloc(sizeof(struct lwis_gpios_info), GFP_KERNEL);
	if (IS_ERR_OR_NULL(gpios_info)) {
		pr_err("Allocate lwis_gpios_info failed\n");
		return -ENOMEM;
	}

	descs = lwis_gpio_list_get(dev, name);
	if (IS_ERR_OR_NULL(descs)) {
		pr_err("Error parsing GPIO list %s (%ld)\n", name, PTR_ERR(descs));
		kfree(gpios_info);
		return PTR_ERR(descs);
	}
	gpios_info->id = desc_to_gpio(descs->desc[0]);
	gpios_info->hold_dev = dev;
	/*
	 * The GPIO pins are valid, release the list as we do not need to hold
	 * on to the pins yet
	 */
	lwis_gpio_list_put(descs, dev);

	gpios_info->gpios = NULL;
	gpios_info->irq_list = NULL;
	strscpy(gpios_info->name, name, LWIS_MAX_NAME_STRING_LEN);

	if (strncmp(SHARED_STRING, name, strlen(SHARED_STRING)) == 0)
		gpios_info->is_shared = true;
	else
		gpios_info->is_shared = false;

	if (strncmp(PULSE_STRING, name, strlen(PULSE_STRING)) == 0)
		gpios_info->is_pulse = true;
	else
		gpios_info->is_pulse = false;

	list_add(&gpios_info->node, list);
	return 0;
}

void lwis_gpios_list_free(struct list_head *list)
{
	struct lwis_gpios_info *gpio_node;
	struct list_head *it_node, *it_tmp;

	if (!list || list_empty(list))
		return;

	list_for_each_safe(it_node, it_tmp, list) {
		gpio_node = list_entry(it_node, struct lwis_gpios_info, node);
		list_del(&gpio_node->node);
		if (gpio_node->irq_list)
			lwis_interrupt_list_free(gpio_node->irq_list);
		kfree(gpio_node);
	}
}

struct lwis_gpios_info *lwis_gpios_get_info_by_name(struct list_head *list, const char *name)
{
	struct lwis_gpios_info *gpio_node;
	struct list_head *it_node, *it_tmp;

	if (!list || !name || list_empty(list))
		return ERR_PTR(-EINVAL);

	list_for_each_safe(it_node, it_tmp, list) {
		gpio_node = list_entry(it_node, struct lwis_gpios_info, node);
		if (!strcmp(gpio_node->name, name))
			return gpio_node;
	}

	return ERR_PTR(-EINVAL);
}
