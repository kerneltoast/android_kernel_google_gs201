// SPDX-License-Identifier: GPL-2.0-only
/*
 * max77779 pinctrl driver
 *
 * Copyright (C) 2023 Google, LLC.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "max77779.h"
#include "max77779_pmic.h"

#define MAX77779_SGPIO_CNFGx_MODE_INPUT		0b01
#define MAX77779_SGPIO_CNFGx_MODE_OUTPUT	0b10

/* DEBUG     */
struct pin_state {
	bool pull_up;
	bool pull_down;
	bool debounce;
};
/* END DEBUG */

struct max77779_pctrl_info {
	struct device		*dev;
	struct device		*core;
	struct pinctrl_dev	*pctl;

	struct mutex		lock;
};

struct max77779_func_group {
	const char *func_name;
	const char * const *groups;
	unsigned int ngroups;
};

static const char * const pinctrl_groups[] = {
	"gpio0",
	"gpio1",
	"gpio2",
	"gpio3",
	"gpio4",
	"gpio5",
	"gpio6",
	"gpio7",
};

static const char * const pinctrl_funcs[] = {
	"gpio",
};

static int max77779_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(pinctrl_groups);
}

static const char *max77779_pinctrl_get_group_name(
		struct pinctrl_dev *pctldev, unsigned int group)
{
	return pinctrl_groups[group];
}

static int max77779_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned int group,
		const unsigned int **pins, unsigned int *num_pins)
{
	return 0;
}

static const struct pinctrl_ops max77779_pinctrl_ops = {
	.get_groups_count = max77779_pinctrl_get_groups_count,
	.get_group_name = max77779_pinctrl_get_group_name,
	.get_group_pins = max77779_pinctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static int max77779_pinctrl_get_funcs_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(pinctrl_funcs);
}

static const char *max77779_pinctrl_get_func_name(struct pinctrl_dev *pctldev,
		unsigned int function)
{
	return pinctrl_funcs[function];
}

static int max77779_pinctrl_get_func_groups(struct pinctrl_dev *pctldev,
		unsigned int function, const char * const **groups,
		unsigned int * const num_groups)
{
	*groups = pinctrl_groups;
	*num_groups = ARRAY_SIZE(pinctrl_groups);
	return 0;
}

static int max77779_pinctrl_set_mux(struct pinctrl_dev *pctldev,
		unsigned int function, unsigned int group)
{
	return 0;
}

static const struct pinmux_ops max77779_pinmux_ops = {
	.get_functions_count	= max77779_pinctrl_get_funcs_count,
	.get_function_name	= max77779_pinctrl_get_func_name,
	.get_function_groups	= max77779_pinctrl_get_func_groups,
	.set_mux		= max77779_pinctrl_set_mux,
};

static int max77779_pinconf_get(struct pinctrl_dev *pctldev,
		unsigned int pin, unsigned long *config)
{
	return 0;
}

static int max77779_pinconf_set(struct pinctrl_dev *pctldev,
		unsigned int pin, unsigned long *configs,
		unsigned int num_configs)
{
	struct max77779_pctrl_info *info = pinctrl_dev_get_drvdata(pctldev);
	int err = 0;
	int i;

	for (i = 0; i < num_configs; i++) {
		const u32 param = pinconf_to_config_param(configs[i]);
		const u32 arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_PULL_UP:
		{
			const u8 enable = pinconf_to_config_argument(configs[i]);

			if (enable) {
				err = max77779_external_pmic_reg_update(info->core,
						MAX77779_PMIC_GPIO_SGPIO_PD, BIT(pin), ~BIT(pin));
				if (err)
					break;
			}

			err = max77779_external_pmic_reg_update(info->core,
					MAX77779_PMIC_GPIO_SGPIO_PU, BIT(pin), enable << pin);
			break;
		}
		case PIN_CONFIG_BIAS_PULL_DOWN:
		{
			const u8 enable = pinconf_to_config_argument(configs[i]);

			if (enable) {
				err = max77779_external_pmic_reg_update(info->core,
						MAX77779_PMIC_GPIO_SGPIO_PU, BIT(pin), ~BIT(pin));
				if (err)
					break;
			}

			err = max77779_external_pmic_reg_update(info->core,
					MAX77779_PMIC_GPIO_SGPIO_PD, BIT(pin), enable << pin);
			if (err)
				return err;

			break;
		}
		case PIN_CONFIG_BIAS_DISABLE:
		{
			err = max77779_external_pmic_reg_update(info->core,
					MAX77779_PMIC_GPIO_SGPIO_PU, BIT(pin), ~BIT(pin));
			if (err)
				break;

			err = max77779_external_pmic_reg_update(info->core,
					MAX77779_PMIC_GPIO_SGPIO_PD, BIT(pin), ~BIT(pin));
			break;
		}
		case PIN_CONFIG_INPUT_ENABLE:
		{
			const u8 reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + pin;
			const u8 mask = MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_MASK;
			const u8 rval = _max77779_pmic_gpio_sgpio_cnfg0_mode_set(reg,
						MAX77779_SGPIO_CNFGx_MODE_INPUT);

			err =  max77779_external_pmic_reg_update(info->core, reg, mask, rval);
			break;
		}
		case PIN_CONFIG_OUTPUT:
		case PIN_CONFIG_OUTPUT_ENABLE:
		{
			const u8 reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + pin;
			const u8 mask = MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_MASK |
					MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_MASK;
			const u8 rval = _max77779_pmic_gpio_sgpio_cnfg0_data_set(reg, !!arg) |
					_max77779_pmic_gpio_sgpio_cnfg0_mode_set(reg,
						MAX77779_SGPIO_CNFGx_MODE_OUTPUT);

			err =  max77779_external_pmic_reg_update(info->core, reg, mask, rval);
			break;
		}
		default:
			err = -ENOTSUPP;
		}

		if (err) {
			dev_err(info->dev,
				"Failed to set config %u on pin %u err = %d\n",
				param, pin, err);
			break;
		}
	}

	return err;
}

static const struct pinconf_ops max77779_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = max77779_pinconf_get,
	.pin_config_set = max77779_pinconf_set,
};

static const struct pinctrl_pin_desc max77779_pins_desc[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
};

static struct pinctrl_desc max77779_pinctrl_desc = {
	.name = "max77779-pinctrl",
	.pins = max77779_pins_desc,
	.npins = ARRAY_SIZE(max77779_pins_desc),
	.pctlops = &max77779_pinctrl_ops,
	.pmxops = &max77779_pinmux_ops,
	.confops = &max77779_pinconf_ops,
};

static int max77779_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77779_pctrl_info *info;
	int err;
	uint8_t val;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->core = dev->parent;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);

	info->pctl = devm_pinctrl_register(dev, &max77779_pinctrl_desc, info);
	err = max77779_external_pmic_reg_read(info->core, MAX77779_PMIC_GPIO_SGPIO_PD, &val);
	if (!err)
		dev_err(info->dev, "MAX77779_PMIC_GPIO_SGPIO_PD = %#02x\n", val);

	return 0;
}

static int max77779_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct platform_device_id max77779_pinctrl_id[] = {
	{ "max77779-pinctrl", 0},
	{},
};

MODULE_DEVICE_TABLE(platform, max77779_pinctrl_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id max77779_pinctrl_match_table[] = {
	{ .compatible = "max77779-pinctrl",},
	{ },
};
#endif

static struct platform_driver max77779_pinctrl_driver = {
	.probe = max77779_pinctrl_probe,
	.remove = max77779_pinctrl_remove,
	.id_table = max77779_pinctrl_id,
	.driver = {
		.name = "max77779-pinctrl",
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = max77779_pinctrl_match_table,
#endif
	},
};

module_platform_driver(max77779_pinctrl_driver);

MODULE_DESCRIPTION("Maxim 77779 pinctrl driver");
MODULE_AUTHOR("James Wylder <jwylder@google.com>");
MODULE_LICENSE("GPL");
