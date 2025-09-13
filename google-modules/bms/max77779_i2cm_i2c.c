/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#include <linux/i2c.h>
#include <linux/regmap.h>
#include "max77779.h"
#include "max77779_i2cm.h"

static const struct regmap_config max77779_i2cm_regmap_cfg = {
	.name = "max77779_i2cm_regmap_cfg",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = I2CM_MAX_REGISTER,
};

static const struct i2c_device_id id[] = {
	{ "max77779_i2cm", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, id);

static int max77779_i2cm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77779_i2cm_info *info;

	/* pmic-irq driver needs to setup the irq */
	if (client->irq < 0)
		return -EPROBE_DEFER;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->client = client;
	info->irq = client->irq;
	i2c_set_clientdata(client, info);

	/* setup data structures */
	info->regmap = devm_regmap_init_i2c(client, &max77779_i2cm_regmap_cfg);
	if (IS_ERR(info->regmap)) {
		dev_err(dev, "Failed to initialize regmap.\n");
		return -EINVAL;
	}

	return max77779_i2cm_init(info);
}

static void max77779_i2cm_i2c_remove(struct i2c_client *client)
{
	struct max77779_i2cm_info *chip = i2c_get_clientdata(client);

	max77779_i2cm_init(chip);
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id max77779_i2cm_match_table[] = {
	{ .compatible = "maxim,max77779i2cm-i2c",},
	{ },
};
#endif

static struct i2c_driver max77779_i2cm_driver = {
	.probe		= max77779_i2cm_i2c_probe,
	.remove		= max77779_i2cm_i2c_remove,
	.id_table	= id,
	.driver = {
		.name   = "max77779_i2cm",
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = max77779_i2cm_match_table,
#endif
	},
};

module_i2c_driver(max77779_i2cm_driver);
MODULE_DESCRIPTION("Maxim 77779 I2CM I2C Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_LICENSE("GPL");
