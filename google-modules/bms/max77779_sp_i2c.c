/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#include <linux/i2c.h>
#include <linux/regmap.h>

#include "max77779_sp.h"

static const struct regmap_config max77779_sp_regmap_cfg = {
	.name = "max77779_scratch",
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77779_SP_MAX_ADDR,
	.readable_reg = max77779_sp_is_reg,
	.volatile_reg = max77779_sp_is_reg,
};

static const struct i2c_device_id max77779_sp_id[] = {
	{"max77779_sp", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_sp_id);

static int max77779_sp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77779_sp_data *data;
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &max77779_sp_regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->regmap = regmap;
	i2c_set_clientdata(client, data);

	return max77779_sp_init(data);
}

static void max77779_sp_i2c_remove(struct i2c_client *client)
{
	struct max77779_sp_data *data = i2c_get_clientdata(client);

	max77779_sp_remove(data);
}

static const struct of_device_id max77779_scratch_of_match_table[] = {
	{ .compatible = "maxim,max77779sp-i2c"},
	{},
};
MODULE_DEVICE_TABLE(of, max77779_scratch_of_match_table);

static struct i2c_driver max77779_scratch_i2c_driver = {
	.driver = {
		.name = "max77779-sp",
		.owner = THIS_MODULE,
		.of_match_table = max77779_scratch_of_match_table,
	},
	.id_table = max77779_sp_id,
	.probe    = max77779_sp_i2c_probe,
	.remove   = max77779_sp_i2c_remove,
};

module_i2c_driver(max77779_scratch_i2c_driver);
MODULE_DESCRIPTION("Maxim 77779 Scratch I2C Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_LICENSE("GPL");
