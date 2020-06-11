// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>

typedef enum { GPIO_CHIP_S2MPG10, GPIO_CHIP_S2MPG11 } gpio_chip_id_t;

struct s2mpg1x_gpio_chip {
	struct gpio_chip gc;
	gpio_chip_id_t id;
	struct device *parent_dev;
};

int GPIO_CTRL_BASE[] = { 0x1c7, 0x198 };

#define GPIO_CTRL1_OFFSET 0 /* input data */
#define GPIO_CTRL2_OFFSET 1 /* output data */
#define GPIO_CTRL3_OFFSET 2 /* output enabled */
#define GPIO_CTRL4_OFFSET 3 /* Pull-Down enable */
#define GPIO_CTRL5_OFFSET 4 /* Pull-Up enable */
#define GPIO_CTRL6_OFFSET 5 /* Drive strength selection (x2 when 1) */
#define GPIO_CTRL7_OFFSET 6 /* Remote GPIO */

static int s2mpg1x_read_gpio_ctrl_reg(struct s2mpg1x_gpio_chip *gc,
				      unsigned int offset,
				      unsigned int ctrl_offset)
{
	int ret;
	u8 val;

	if (gc->id == GPIO_CHIP_S2MPG10) {
		struct s2mpg10_dev *dev = dev_get_drvdata(gc->parent_dev);

		ret = s2mpg10_read_reg(dev->pmic,
				       GPIO_CTRL_BASE[gc->id] + ctrl_offset,
				       &val);
	} else {
		struct s2mpg11_dev *dev = dev_get_drvdata(gc->parent_dev);

		ret = s2mpg11_read_reg(dev->pmic,
				       GPIO_CTRL_BASE[gc->id] + ctrl_offset,
				       &val);
	}
	if (ret < 0) {
		pr_err("Error: %s %s %d %d", __func__, gc->gc.label, offset,
		       ctrl_offset);
		return ret;
	}

	return val;
}

static int s2mpg1x_read_gpio_ctrl_bit(struct s2mpg1x_gpio_chip *gc,
				      unsigned int offset,
				      unsigned int ctrl_offset)
{
	return (s2mpg1x_read_gpio_ctrl_reg(gc, offset, ctrl_offset) >> offset) &
	       0x1;
}

static int s2mpg1x_update_gpio_ctrl_reg(struct s2mpg1x_gpio_chip *gc,
					unsigned int offset,
					unsigned int ctrl_offset,
					u8 val, u8 mask)
{
	int ret;

	if (gc->id == GPIO_CHIP_S2MPG10) {
		struct s2mpg10_dev *dev = dev_get_drvdata(gc->parent_dev);

		ret = s2mpg10_update_reg(dev->pmic,
					 GPIO_CTRL_BASE[gc->id] + ctrl_offset,
					 val, mask);
	} else {
		struct s2mpg11_dev *dev = dev_get_drvdata(gc->parent_dev);

		ret = s2mpg11_update_reg(dev->pmic,
					 GPIO_CTRL_BASE[gc->id] + ctrl_offset,
					 val, mask);
	}
	if (ret < 0) {
		pr_err("Error: %s %s %d %d", __func__, gc->gc.label,
		       ctrl_offset, offset);
		return ret;
	}

	return ret;
}

static int s2mpg1x_write_gpio_ctrl_bit(struct s2mpg1x_gpio_chip *gc,
				       unsigned int offset,
				       unsigned int ctrl_offset, int value)
{
	return s2mpg1x_update_gpio_ctrl_reg(gc, offset, ctrl_offset,
				(value & 0x1) << offset, 0x1 << offset);
}

static int s2mpg1x_gpio_get_direction(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct s2mpg1x_gpio_chip *data = gpiochip_get_data(chip);

	return !s2mpg1x_read_gpio_ctrl_bit(data, offset, GPIO_CTRL3_OFFSET);
}

static int s2mpg1x_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct s2mpg1x_gpio_chip *data = gpiochip_get_data(chip);

	if (s2mpg1x_gpio_get_direction(chip, offset))
		return s2mpg1x_read_gpio_ctrl_bit(data, offset,
						  GPIO_CTRL1_OFFSET);
	else
		return s2mpg1x_read_gpio_ctrl_bit(data, offset,
						  GPIO_CTRL2_OFFSET);
}

static void s2mpg1x_gpio_set(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	struct s2mpg1x_gpio_chip *data = gpiochip_get_data(chip);

	if (!s2mpg1x_gpio_get_direction(chip, offset))
		s2mpg1x_write_gpio_ctrl_bit(data, offset, GPIO_CTRL2_OFFSET,
					    value);
}

static int s2mpg1x_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct s2mpg1x_gpio_chip *data = gpiochip_get_data(chip);

	return s2mpg1x_write_gpio_ctrl_bit(data, offset, GPIO_CTRL3_OFFSET, 0);
}

static int s2mpg1x_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct s2mpg1x_gpio_chip *data = gpiochip_get_data(chip);

	return s2mpg1x_write_gpio_ctrl_bit(data, offset, GPIO_CTRL3_OFFSET, 1);
}

static int s2mpg1x_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct s2mpg1x_gpio_chip *s2mpg1x_gpio_chip = devm_kzalloc(&pdev->dev,
		sizeof(struct s2mpg1x_gpio_chip), GFP_KERNEL);

	if (!s2mpg1x_gpio_chip)
		return -ENOMEM;

	s2mpg1x_gpio_chip->parent_dev = pdev->dev.parent;
	s2mpg1x_gpio_chip->id = pdev->id_entry->driver_data;

	s2mpg1x_gpio_chip->gc.label = pdev->name;
	s2mpg1x_gpio_chip->gc.parent = &pdev->dev;
	s2mpg1x_gpio_chip->gc.get_direction = s2mpg1x_gpio_get_direction;
	s2mpg1x_gpio_chip->gc.get = s2mpg1x_gpio_get;
	s2mpg1x_gpio_chip->gc.set = s2mpg1x_gpio_set;
	s2mpg1x_gpio_chip->gc.direction_input = s2mpg1x_gpio_direction_input;
	s2mpg1x_gpio_chip->gc.direction_output = s2mpg1x_gpio_direction_output;
	s2mpg1x_gpio_chip->gc.base = -1;
	s2mpg1x_gpio_chip->gc.ngpio = 6;
	s2mpg1x_gpio_chip->gc.can_sleep = true;
	s2mpg1x_gpio_chip->gc.of_node =
		of_find_node_by_name(pdev->dev.parent->of_node, pdev->name);

	if (!s2mpg1x_gpio_chip->gc.of_node) {
		dev_err(&pdev->dev, "Failed to find %s DT node\n", pdev->name);
		return -EINVAL;
	}

	ret = devm_gpiochip_add_data(&pdev->dev, &s2mpg1x_gpio_chip->gc,
				     s2mpg1x_gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register gpio_chip: %d\n", ret);
		return ret;
	}
	platform_set_drvdata(pdev, s2mpg1x_gpio_chip);

	return 0;
}

static int s2mpg1x_gpio_remove(struct platform_device *pdev)
{
	struct s2mpg1x_gpio_chip *s2mpg1x_gpio_chip =
	    platform_get_drvdata(pdev);

	gpiochip_remove(&s2mpg1x_gpio_chip->gc);

	return 0;
}

static const struct platform_device_id s2mpg1x_gpio_id[] = {
	{ "s2mpg10_gpio", GPIO_CHIP_S2MPG10 },
	{ "s2mpg11_gpio", GPIO_CHIP_S2MPG11 },
	{},
};
MODULE_DEVICE_TABLE(platform, s2mpg1x_gpio_id);

static struct platform_driver s2mpg1x_gpio_driver = {
	.driver = {
		   .name = "s2mpg1x_gpio",
		   .owner = THIS_MODULE,
		   },
	.probe = s2mpg1x_gpio_probe,
	.remove = s2mpg1x_gpio_remove,
	.id_table = s2mpg1x_gpio_id,
};

static int __init s2mpg1x_gpio_init(void)
{
	return platform_driver_register(&s2mpg1x_gpio_driver);
}
subsys_initcall(s2mpg1x_gpio_init);

static void __exit s2mpg1x_gpio_exit(void)
{
	platform_driver_unregister(&s2mpg1x_gpio_driver);
}
module_exit(s2mpg1x_gpio_exit);

MODULE_DESCRIPTION("S2MPG10 S2MPG11 GPIO Driver");
MODULE_AUTHOR("Thierry Strudel <tstrudel@google.com>");
MODULE_LICENSE("GPL");
