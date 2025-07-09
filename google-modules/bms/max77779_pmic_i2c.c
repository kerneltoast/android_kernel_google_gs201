/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#include <linux/i2c.h>
#include <linux/regmap.h>
#include "max77779_pmic.h"

static const struct regmap_config max77779_pmic_regmap_cfg = {
	.name = "max77779_pmic",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77779_PMIC_GPIO_VGPI_CNFG,
	.readable_reg = max77779_pmic_is_readable,
	.volatile_reg = max77779_pmic_is_readable,
};

static const struct i2c_device_id max77779_pmic_id[] = {
	{"max77779_pmic", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_pmic_id);

static int max77779_pmic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77779_pmic_info *info;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	i2c_set_clientdata(client, info);

	info->regmap = devm_regmap_init_i2c(client, &max77779_pmic_regmap_cfg);
	if (IS_ERR(info->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	return max77779_pmic_init(info);
}

static void max77779_pmic_i2c_remove(struct i2c_client *client)
{
	struct max77779_pmic_info *info = i2c_get_clientdata(client);

	max77779_pmic_remove(info);
}

static const struct of_device_id max77779_pmic_of_match_table[] = {
	{ .compatible = "maxim,max77779pmic-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, max77779_pmic_of_match_table);

static struct i2c_driver max77779_pmic_i2c_driver = {
	.driver = {
		.name = "max77779-pmic",
		.owner = THIS_MODULE,
		.of_match_table = max77779_pmic_of_match_table,
	},
	.id_table = max77779_pmic_id,
	.probe = max77779_pmic_i2c_probe,
	.remove = max77779_pmic_i2c_remove,
};

module_i2c_driver(max77779_pmic_i2c_driver);
MODULE_DESCRIPTION("Maxim 77779 PMIC I2C Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_LICENSE("GPL");
