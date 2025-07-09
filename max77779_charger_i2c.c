/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#include <linux/i2c.h>
#include <linux/regmap.h>
#include "max77779.h"
#include "max77779_charger.h"

static const struct regmap_config max77779_chg_i2c_regmap_cfg = {
	.name = "max77779_charger",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77779_CHG_CUST_TM,
	.readable_reg = max77779_chg_is_reg,
	.volatile_reg = max77779_chg_is_reg,
};

static const struct i2c_device_id max77779_id[] = {
	{"max77779_charger", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_id);

static int max77779_charger_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77779_chgr_data *data;
	struct regmap *regmap;

	/* pmic-irq driver needs to setup the irq */
	if (client->irq < 0)
		return -EPROBE_DEFER;

	regmap = devm_regmap_init_i2c(client, &max77779_chg_i2c_regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->uc_data.dev = dev;
	data->regmap = regmap;
	data->irq_int = client->irq;

	i2c_set_clientdata(client, data);

	return max77779_charger_init(data);
}

static void max77779_charger_i2c_remove(struct i2c_client *client)
{
	struct max77779_chgr_data *data = i2c_get_clientdata(client);

	max77779_charger_remove(data);
}

static const struct of_device_id max77779_charger_i2c_of_match_table[] = {
	{ .compatible = "maxim,max77779chrg-i2c"},
	{},
};
MODULE_DEVICE_TABLE(of, max77779_charger_i2c_of_match_table);

static const struct dev_pm_ops max77779_charger_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(
		max77779_charger_pm_suspend,
		max77779_charger_pm_resume)
};

static struct i2c_driver max77779_charger_i2c_driver = {
	.driver = {
		.name = "max77779-charger",
		.owner = THIS_MODULE,
		.of_match_table = max77779_charger_i2c_of_match_table,
#if IS_ENABLED(CONFIG_PM)
		.pm = &max77779_charger_pm_ops,
#endif
	},
	.id_table = max77779_id,
	.probe    = max77779_charger_i2c_probe,
	.remove   = max77779_charger_i2c_remove,
};

module_i2c_driver(max77779_charger_i2c_driver);

MODULE_DESCRIPTION("Maxim 77779 Charger I2C Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_LICENSE("GPL");
