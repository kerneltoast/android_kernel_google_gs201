/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright 2020 Google LLC. All Rights Reserved.
 *
 * platform driver to interface with uwb reset pin on aoc
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>

#include "aoc.h"
#include "aoc-interface.h"
#include "aoc_uwb_service_dev.h"

#define AOC_UWB_PDRV_NAME "aoc_uwb_pdrv"

static int value_get(struct gpio_chip *gc, unsigned offset)
{
	struct CMD_UWB_GET_RESET_GPIO cmd = { 0 };
	int ret = -1;

	if (offset) {
		dev_err(gc->parent, "aoc_uwb_pdrv wrong offset\n");
		goto err_exit;
	}

	AocCmdHdrSet(&cmd.parent, CMD_UWB_GET_RESET_GPIO_ID, sizeof(cmd));
	ret = aoc_uwb_service_send(&cmd, sizeof(cmd));
	if (ret < 0) {
		dev_err(gc->parent, "error sending pin get to aoc\n");
		goto err_exit;
	}

	return cmd.gpio_value;

err_exit:
	return ret;
}

static void value_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct CMD_UWB_SET_RESET_GPIO cmd = { 0 };
	int ret;

	dev_dbg(gc->parent, "value=%d\n", value);
	if (offset) {
		dev_err(gc->parent, "wrong offset\n");
		return;
	}

	AocCmdHdrSet(&cmd.parent, CMD_UWB_SET_RESET_GPIO_ID, sizeof(cmd));
	cmd.gpio_value = value;
	ret = aoc_uwb_service_send(&cmd, sizeof(cmd));
	if (ret < 0)
		dev_err(gc->parent, "error sending pin set to aoc ret=%d\n", ret);
}

static int aoc_uwb_reset_set_direction(struct gpio_chip *gc, bool output)
{
	struct CMD_UWB_SET_RESET_GPIO_DIRECTION cmd = { 0 };
	int ret = 0;

	dev_dbg(gc->parent, "dir=%s\n", output ? "output" : "input");
	AocCmdHdrSet(&cmd.parent, CMD_UWB_SET_RESET_GPIO_DIRECTION_ID, sizeof(cmd));
	cmd.gpio_direction = output ? 1 : 0;
	ret = aoc_uwb_service_send(&cmd, sizeof(cmd));
	if (ret < 0)
		dev_err(gc->parent, "error sending pin direction to aoc\n");
	return ret < 0 ? ret : 0;
}

static int direction_input(struct gpio_chip *gc, unsigned offset)
{
	return aoc_uwb_reset_set_direction(gc, 0);
}

static int direction_output(struct gpio_chip *gc, unsigned offset, int value)
{
	value_set(gc, offset, value);
	return aoc_uwb_reset_set_direction(gc, 1);
}

static struct gpio_chip chip = {
	.set = value_set,
	.get = value_get,
	.direction_input = direction_input,
	.direction_output = direction_output,
	.base = -1,
};

static int uwb_pdrv_pin_init(struct platform_device *pdev)
{
	struct device_node *node;
	struct device *dev = &pdev->dev;
	int ngpio = 0;
	int ret = 0;

	node = pdev->dev.of_node;
	ret = of_property_read_u32(node, "ngpio", &ngpio);
	if (ret != 0) {
		dev_err(dev, "fail to read ngpio\n");
		return ret;
	}

	chip.label = node->name;
	chip.parent = dev;
	chip.of_node = node;
	chip.ngpio = ngpio;
	return devm_gpiochip_add_data(dev, &chip, NULL);
}

static int aoc_uwb_pdrv_probe(struct platform_device *pdev)
{
	if (!aoc_uwb_service_ready())
		return -EPROBE_DEFER;

	return uwb_pdrv_pin_init(pdev);
}

static int aoc_uwb_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id aoc_match[] = {
	{
		.compatible = "google,aoc_uwb_rst",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aoc_match);

static struct platform_driver aoc_uwb_pdrv = {
	.driver = {
		.name = AOC_UWB_PDRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aoc_match)
	},
	.probe = aoc_uwb_pdrv_probe,
	.remove = aoc_uwb_pdrv_remove,
};

module_platform_driver(aoc_uwb_pdrv);

MODULE_LICENSE("GPL v2");
