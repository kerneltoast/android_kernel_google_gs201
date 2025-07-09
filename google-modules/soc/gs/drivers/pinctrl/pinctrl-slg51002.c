// SPDX-License-Identifier: GPL-2.0
/*
 * slg51002 pinctrl driver
 *
 * Copyright (C) 2021 Google, LLC.
 */

#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/mfd/slg51002.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>

struct slg51002_pinctrl {
	struct gpio_chip gc;

	struct pinctrl_ops pctrl_ops;
	struct pinmux_ops pmux_ops;
	struct pinconf_ops pconf_ops;
	struct pinctrl_desc pctrl;

	struct regmap *regmap;
	struct slg51002_dev *chip;
};

static const struct pinctrl_pin_desc slg51002_pins[] = {
	PINCTRL_PIN(SLG51002_GPIO1, "gpio1"),
	PINCTRL_PIN(SLG51002_GPIO2, "gpio2"),
	PINCTRL_PIN(SLG51002_GPIO3, "gpio3"),
	PINCTRL_PIN(SLG51002_GPIO4, "gpio4"),
	PINCTRL_PIN(SLG51002_SEQ1, "seq1"),
	PINCTRL_PIN(SLG51002_SEQ2, "seq2"),
	PINCTRL_PIN(SLG51002_SEQ3, "seq3"),
	PINCTRL_PIN(SLG51002_SEQ4, "seq4"),
};

static const struct pinctrl_pin_desc slg51002_generic_seq_pins[] = {
	PINCTRL_PIN(SLG51002_GENERIC_SEQ0, "seq0"),
	PINCTRL_PIN(SLG51002_GENERIC_SEQ1, "seq1"),
	PINCTRL_PIN(SLG51002_GENERIC_SEQ2, "seq2"),
	PINCTRL_PIN(SLG51002_GENERIC_SEQ3, "seq3"),
	PINCTRL_PIN(SLG51002_GENERIC_SEQ4, "seq4"),
	PINCTRL_PIN(SLG51002_GENERIC_SEQ5, "seq5"),
	PINCTRL_PIN(SLG51002_GENERIC_SEQ6, "seq6"),
	PINCTRL_PIN(SLG51002_GENERIC_SEQ7, "seq7"),
};

static const int slg51002_ctrl_bit_tbl[] = {
	BIT(4), BIT(5), BIT(6), BIT(7),	/* gpio1, gpio2, gpio3, gpio4 */
	BIT(0), BIT(1), BIT(2), BIT(3)	/* seq1, seq2, seq3 seq4 */
};

/* gpio_chip functions */
static int slg51002_gpio_get_direction(struct gpio_chip *chip,
				      unsigned int offset)
{
	int ret;
	unsigned int addr;
	unsigned int val;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	switch (offset) {
	case SLG51002_GPIO1:
		addr = SLG51002_IO_GPIO1_CONF;
		break;
	case SLG51002_GPIO2:
		addr = SLG51002_IO_GPIO2_CONF;
		break;
	case SLG51002_GPIO3:
		addr = SLG51002_IO_GPIO3_CONF;
		break;
	case SLG51002_GPIO4:
		addr = SLG51002_IO_GPIO4_CONF;
		break;
	case SLG51002_SEQ1:
	case SLG51002_SEQ2:
	case SLG51002_SEQ3:
	case SLG51002_SEQ4:
		return GPIOF_DIR_OUT;
	default:
		return -EOPNOTSUPP;
	}

	ret = regmap_read(data->regmap, addr, &val);
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
		return ret;
	}

	return (val & SLG51002_GPIO_DIR_MASK) ? GPIOF_DIR_OUT : GPIOF_DIR_IN;
}

static int slg51002_generic_seq_get_direction(struct gpio_chip *chip,
				unsigned int offset)
{
	switch (offset) {
	case SLG51002_GENERIC_SEQ0:
	case SLG51002_GENERIC_SEQ1:
	case SLG51002_GENERIC_SEQ2:
	case SLG51002_GENERIC_SEQ3:
	case SLG51002_GENERIC_SEQ4:
	case SLG51002_GENERIC_SEQ5:
	case SLG51002_GENERIC_SEQ6:
	case SLG51002_GENERIC_SEQ7:
		return GPIOF_DIR_OUT;
	default:
		return -EOPNOTSUPP;
	}
}

static int slg51002_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	int ret;
	unsigned int val;
	unsigned int addr;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	switch (offset) {
	case SLG51002_GPIO1:
		addr = GPIO1_CTRL;
		break;
	case SLG51002_GPIO2:
		addr = GPIO2_CTRL;
		break;
	case SLG51002_GPIO3:
		addr = GPIO3_CTRL;
		break;
	case SLG51002_GPIO4:
		addr = GPIO4_CTRL;
		break;
	default:
		return -EOPNOTSUPP;
	}

	ret = regmap_read(data->regmap, addr, &val);
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
		return ret;
	}

	return val;
}

static void slg51002_gpio_set(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	int ret;
	int val = !!value; /* convert value to 0/1 */
	unsigned int addr;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	if (data->chip->enter_sw_test_mode)
		data->chip->enter_sw_test_mode(data->regmap);

	switch (offset) {
	case SLG51002_GPIO1:
		addr = GPIO1_CTRL;
		break;
	case SLG51002_GPIO2:
		addr = GPIO2_CTRL;
		break;
	case SLG51002_GPIO3:
		addr = GPIO3_CTRL;
		break;
	case SLG51002_GPIO4:
		addr = GPIO4_CTRL;
		break;
	default:
		dev_err(data->chip->dev, "Unsupported GPIO offset %d\n", offset);
		return;
	}

	ret = regmap_write(data->regmap, addr, val);
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
	}

	if (data->chip->exit_sw_test_mode)
		data->chip->exit_sw_test_mode(data->regmap);
}

static int slg51002_gpio_seq_get(struct gpio_chip *chip, unsigned int offset)
{
	int ret;
	unsigned int val;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	ret = regmap_read(data->regmap, SLG51002_SYSCTL_MATRIX_CONF_A, &val);
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
		return ret;
	}

	return val & slg51002_ctrl_bit_tbl[offset];
}

static void slg51002_gpio_seq_set(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	int ret;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	ret = regmap_update_bits(data->regmap, SLG51002_SYSCTL_MATRIX_CONF_A,
			slg51002_ctrl_bit_tbl[offset],
			value ? slg51002_ctrl_bit_tbl[offset] : 0);
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
	}

	return;
}

static int slg51002_generic_seq_get(struct gpio_chip *chip, unsigned int offset)
{
	int ret;
	unsigned int val;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	ret = regmap_read(data->regmap, SLG51002_SYSCTL_MATRIX_CONF_A, &val);
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
		return ret;
	}

	return val & BIT(offset);
}

static void slg51002_generic_seq_set(struct gpio_chip *chip,
				unsigned int offset, int value)
{
	int ret;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	ret = regmap_update_bits(data->regmap, SLG51002_SYSCTL_MATRIX_CONF_A,
			BIT(offset),
			value ? BIT(offset) : 0);
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
	}

	return;
}

static int slg51002_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	int ret;
	unsigned int addr;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	if (data->chip->enter_sw_test_mode)
		data->chip->enter_sw_test_mode(data->regmap);

	switch (offset) {
	case SLG51002_GPIO1:
		addr = SLG51002_IO_GPIO1_CONF;
		break;
	case SLG51002_GPIO2:
		addr = SLG51002_IO_GPIO2_CONF;
		break;
	case SLG51002_GPIO3:
		addr = SLG51002_IO_GPIO3_CONF;
		break;
	case SLG51002_GPIO4:
		addr = SLG51002_IO_GPIO4_CONF;
		break;
	default:
		dev_err(data->chip->dev, "Unsupported GPIO offset %d\n", offset);
		ret = -EOPNOTSUPP;
		goto out;
	}

	ret = regmap_update_bits(data->regmap, addr, SLG51002_GPIO_DIR_MASK, 0);
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
	}

out:
	if (data->chip->exit_sw_test_mode)
		data->chip->exit_sw_test_mode(data->regmap);

	return ret;
}

static int slg51002_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	int ret;
	unsigned int addr;
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	if (data->chip->enter_sw_test_mode)
		data->chip->enter_sw_test_mode(data->regmap);

	switch (offset) {
	case SLG51002_GPIO1:
		addr = SLG51002_IO_GPIO1_CONF;
		break;
	case SLG51002_GPIO2:
		addr = SLG51002_IO_GPIO2_CONF;
		break;
	case SLG51002_GPIO3:
		addr = SLG51002_IO_GPIO3_CONF;
		break;
	case SLG51002_GPIO4:
		addr = SLG51002_IO_GPIO4_CONF;
		break;
	case SLG51002_SEQ1:
	case SLG51002_SEQ2:
	case SLG51002_SEQ3:
	case SLG51002_SEQ4:
		ret = 0;
		goto out1;
	default:
		dev_err(data->chip->dev, "Unsupported GPIO offset %d\n",
			offset);
		ret = -EOPNOTSUPP;
		goto out;
	}

	ret = regmap_update_bits(data->regmap, addr, SLG51002_GPIO_DIR_MASK,
				 BIT(SLG51002_GPIO_DIR_SHIFT));
	if (ret < 0) {
		dev_err(data->chip->dev, "Failed on GPIO offset %d\n", offset);
		goto out;
	}

out1:
	if (data->gc.set)
		data->gc.set(chip, offset, value);

out:
	if (data->chip->exit_sw_test_mode)
		data->chip->exit_sw_test_mode(data->regmap);

	return ret;
}

static int slg51002_generic_seq_direction_input(struct gpio_chip *chip,
				unsigned int offset)
{
	return -EOPNOTSUPP;
}

static int slg51002_generic_seq_direction_output(struct gpio_chip *chip,
				unsigned int offset, int value)
{
	struct slg51002_pinctrl *data = gpiochip_get_data(chip);

	switch (offset) {
	case SLG51002_GENERIC_SEQ0:
	case SLG51002_GENERIC_SEQ1:
	case SLG51002_GENERIC_SEQ2:
	case SLG51002_GENERIC_SEQ3:
	case SLG51002_GENERIC_SEQ4:
	case SLG51002_GENERIC_SEQ5:
	case SLG51002_GENERIC_SEQ6:
	case SLG51002_GENERIC_SEQ7:
		if (data->gc.set)
			data->gc.set(chip, offset, value);
		return 0;
	default:
		dev_err(data->chip->dev, "Unsupported GPIO offset %d\n",
			offset);
		return -EOPNOTSUPP;
	}
}

/* pinctrl_ops functions */
static int slg51002_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *slg51002_pinctrl_get_group_name(
		struct pinctrl_dev *pctldev, unsigned int group)
{
	return NULL;
}

/* pinmux_ops functions */
static int slg51002_pinmux_get_funcs_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *slg51002_pinmux_get_func_name(struct pinctrl_dev *pctldev,
						  unsigned int function)
{
	return NULL;
}

static int slg51002_pinmux_get_func_groups(struct pinctrl_dev *pctldev,
					    unsigned int function,
					    const char * const **groups,
					    unsigned int * const num_groups)
{
	return -EOPNOTSUPP;
}

static int slg51002_pinmux_set_mux(struct pinctrl_dev *pctldev,
				   unsigned int function, unsigned int group)
{
	return -EOPNOTSUPP;
}

static int slg51002_pinmux_gpio_set_direction(struct pinctrl_dev *pctldev,
				     struct pinctrl_gpio_range *range,
				     unsigned int offset, bool input)
{
	return 0;
}

/* pinconf_ops functions */
static int slg51002_pinconf_get(struct pinctrl_dev *pctldev,
				unsigned int pin, unsigned long *config)
{
	return 0;
}

static int slg51002_pinconf_set(struct pinctrl_dev *pctldev,
				unsigned int pin, unsigned long *configs,
				unsigned int num_configs)
{
	return 0;
}

static int slg51002_pinctrl_probe(struct platform_device *pdev)
{
	int ret;
	struct slg51002_pinctrl *slg51002_pctl;
	struct pinctrl_dev *pctl;
	u32 ngpios;
	const char *pinctrl_of_name = NULL;

	slg51002_pctl = devm_kzalloc(&pdev->dev,
		sizeof(struct slg51002_pinctrl), GFP_KERNEL);
	if (!slg51002_pctl)
		return -ENOMEM;

	slg51002_pctl->chip = dev_get_drvdata(pdev->dev.parent);

	if (slg51002_pctl->chip->op_mode != SLG51002_OP_MODE_LDO_GPIO &&
		slg51002_pctl->chip->op_mode != SLG51002_OP_MODE_SEQ_GPIO &&
		slg51002_pctl->chip->op_mode != SLG51002_OP_MODE_SEQ_GENERIC)
		return -ENODEV;

	/* Get regmap */
	slg51002_pctl->regmap = slg51002_pctl->chip->regmap;

	/* GPIO config */
	slg51002_pctl->gc.label = pdev->name;
	slg51002_pctl->gc.parent = &pdev->dev;

	if (slg51002_pctl->chip->op_mode == SLG51002_OP_MODE_SEQ_GENERIC) {
		slg51002_pctl->gc.get_direction =
				slg51002_generic_seq_get_direction;
		slg51002_pctl->gc.get = slg51002_generic_seq_get;
		slg51002_pctl->gc.set = slg51002_generic_seq_set;
		slg51002_pctl->gc.direction_input =
				slg51002_generic_seq_direction_input;
		slg51002_pctl->gc.direction_output =
				slg51002_generic_seq_direction_output;
	} else {
		slg51002_pctl->gc.get_direction = slg51002_gpio_get_direction;
		if (slg51002_pctl->chip->op_mode == SLG51002_OP_MODE_SEQ_GPIO) {
			slg51002_pctl->gc.get = slg51002_gpio_seq_get;
			slg51002_pctl->gc.set = slg51002_gpio_seq_set;
		} else {
			slg51002_pctl->gc.get = slg51002_gpio_get;
			slg51002_pctl->gc.set = slg51002_gpio_set;
		}
		slg51002_pctl->gc.direction_input =
				slg51002_gpio_direction_input;
		slg51002_pctl->gc.direction_output =
				slg51002_gpio_direction_output;
	}

	slg51002_pctl->gc.base = -1;
	slg51002_pctl->gc.can_sleep = true;
	slg51002_pctl->gc.of_node =
		of_find_node_by_name(pdev->dev.parent->of_node, pdev->name);
	slg51002_pctl->gc.set_config = gpiochip_generic_config;
	slg51002_pctl->gc.request = gpiochip_generic_request;
	slg51002_pctl->gc.free = gpiochip_generic_free;

	if (!slg51002_pctl->gc.of_node) {
		dev_err(&pdev->dev, "Failed to find %s DT node\n", pdev->name);
		return -EINVAL;
	}
	if (of_property_read_u32(slg51002_pctl->gc.of_node,
			"ngpios", &ngpios)) {
		dev_err(&pdev->dev, "Failed to get ngpios from %s DT node\n",
			pdev->name);
		return -EINVAL;
	}
	slg51002_pctl->gc.ngpio = ngpios;

	/* pinctrl config */
	slg51002_pctl->pctrl_ops.get_groups_count =
			slg51002_pinctrl_get_groups_count;
	slg51002_pctl->pctrl_ops.get_group_name =
			slg51002_pinctrl_get_group_name;
	slg51002_pctl->pctrl_ops.dt_node_to_map =
			pinconf_generic_dt_node_to_map_pin;
	slg51002_pctl->pctrl_ops.dt_free_map = pinconf_generic_dt_free_map;

	slg51002_pctl->pmux_ops.get_functions_count =
			slg51002_pinmux_get_funcs_count;
	slg51002_pctl->pmux_ops.get_function_name =
			slg51002_pinmux_get_func_name;
	slg51002_pctl->pmux_ops.get_function_groups =
			slg51002_pinmux_get_func_groups;
	slg51002_pctl->pmux_ops.set_mux = slg51002_pinmux_set_mux;
	slg51002_pctl->pmux_ops.gpio_set_direction =
			slg51002_pinmux_gpio_set_direction;

	slg51002_pctl->pconf_ops.is_generic = true;
	slg51002_pctl->pconf_ops.pin_config_get = slg51002_pinconf_get;
	slg51002_pctl->pconf_ops.pin_config_set = slg51002_pinconf_set;

	/* pins defined in chip */
	if (slg51002_pctl->chip->op_mode == SLG51002_OP_MODE_SEQ_GENERIC) {
		slg51002_pctl->pctrl.pins = slg51002_generic_seq_pins;
		slg51002_pctl->pctrl.npins =
				ARRAY_SIZE(slg51002_generic_seq_pins);
	} else {
		slg51002_pctl->pctrl.pins = slg51002_pins;
		slg51002_pctl->pctrl.npins = ARRAY_SIZE(slg51002_pins);
	}

	slg51002_pctl->pctrl.pctlops = &slg51002_pctl->pctrl_ops;
	slg51002_pctl->pctrl.pmxops = &slg51002_pctl->pmux_ops;
	slg51002_pctl->pctrl.confops = &slg51002_pctl->pconf_ops;
	slg51002_pctl->pctrl.owner = THIS_MODULE;
	slg51002_pctl->pctrl.name = dev_name(&pdev->dev);

	pinctrl_of_name = "slg51002_pinctrl";
	pdev->dev.of_node = of_find_node_by_name(pdev->dev.parent->of_node,
						 pinctrl_of_name);
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Failed to find %s DT node\n",
			pinctrl_of_name);
		return -EINVAL;
	}

	ret = devm_pinctrl_register_and_init(&pdev->dev,
		&slg51002_pctl->pctrl, slg51002_pctl, &pctl);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pinctrl: %d\n", ret);
		return ret;
	}

	ret = pinctrl_enable(pctl);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to enable pinctrl: %d\n", ret);
		return ret;
	}

	ret = devm_gpiochip_add_data(&pdev->dev, &slg51002_pctl->gc,
				     slg51002_pctl);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register gpio_chip: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, slg51002_pctl);

	return 0;
}

static int slg51002_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct platform_device_id slg51002_pinctrl_id[] = {
	{ "slg51002_gpio", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, slg51002_pinctrl_id);

static struct platform_driver slg51002_pinctrl_driver = {
	.driver = {
		.name = "slg51002_gpio",
		.owner = THIS_MODULE,
	},
	.probe = slg51002_pinctrl_probe,
	.remove = slg51002_pinctrl_remove,
	.id_table = slg51002_pinctrl_id,
};

static int __init slg51002_pinctrl_init(void)
{
	return platform_driver_register(&slg51002_pinctrl_driver);
}
subsys_initcall(slg51002_pinctrl_init);

static void __exit slg51002_pinctrl_exit(void)
{
	platform_driver_unregister(&slg51002_pinctrl_driver);
}
module_exit(slg51002_pinctrl_exit);

MODULE_DESCRIPTION("SLG51002 pinctrl and GPIO driver");
MODULE_LICENSE("GPL");
