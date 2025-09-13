/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#include <linux/i2c.h>
#include <linux/regmap.h>
#include "max77779.h"
#include "max77779_fg.h"

const struct regmap_config max77779_fg_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77779_FG_USR,
	.readable_reg = max77779_fg_is_reg,
	.volatile_reg = max77779_fg_is_reg,
};

const struct regmap_config max77779_fg_debug_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77779_FG_NVM_nProtMiscTh,
	.readable_reg = max77779_fg_dbg_is_reg,
	.volatile_reg = max77779_fg_dbg_is_reg,
};

static const struct i2c_device_id max77779_fg_id[] = {
	{"max77779_fg", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_fg_id);

static int max77779_max17x0x_i2c_regmap_init(struct maxfg_regmap *regmap, struct i2c_client *clnt,
				      const struct regmap_config *regmap_config, bool tag)
{
	struct regmap *map;

	map = devm_regmap_init_i2c(clnt, regmap_config);
	if (IS_ERR(map))
		return IS_ERR_VALUE(map);

	if (tag) {
		regmap->regtags.max = ARRAY_SIZE(max77779_fg);
		regmap->regtags.map = max77779_fg;
	} else {
		regmap->regtags.max = ARRAY_SIZE(max77779_debug_fg);
		regmap->regtags.map = max77779_debug_fg;
	}

	regmap->regmap = map;
	return 0;
}

/* NOTE: NEED TO COME BEFORE REGISTER ACCESS */
static int max77779_fg_i2c_regmap_init(struct max77779_fg_chip *chip, struct i2c_client *primary)
{
	int ret;

	if (!primary || !chip->secondary) {
		dev_err(chip->dev, "Error i2c client not valid. primary:%p secondary:%p",
			primary, chip->secondary);
		return -EINVAL;
	}

	ret = max77779_max17x0x_i2c_regmap_init(&chip->regmap, primary,
						&max77779_fg_regmap_cfg, true);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to re-initialize regmap (%ld)\n",
			IS_ERR_VALUE(chip->regmap.regmap));
		return -EINVAL;
	}

	ret = max77779_max17x0x_i2c_regmap_init(&chip->regmap_debug, chip->secondary,
					    	&max77779_fg_debug_regmap_cfg, false);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to re-initialize debug regmap (%ld)\n",
			IS_ERR_VALUE(chip->regmap_debug.regmap));
		return IS_ERR_VALUE(chip->regmap_debug.regmap);
	}
	return 0;
}

static int max77779_fg_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77779_fg_chip *chip;
	int ret;

	/* pmic-irq driver needs to setup the irq */
	if (client->irq < 0)
		return -EPROBE_DEFER;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->irq = client->irq;
	i2c_set_clientdata(client, chip);

	chip->secondary = i2c_new_ancillary_device(client, "ndbg",
						   MAX77779_FG_NDGB_ADDRESS);
	if (IS_ERR(chip->secondary)) {
		dev_err(dev, "Error setting up ancillary i2c bus(%ld)\n",
			IS_ERR_VALUE(chip->secondary));
		ret = PTR_ERR(chip->secondary);
		goto error;
	}
	i2c_set_clientdata(chip->secondary, chip);

	/* needs chip->secondary */
	ret = max77779_fg_i2c_regmap_init(chip, client);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize regmap(s)\n");
		goto error;
	}

	ret = max77779_fg_init(chip);
	if (!ret)
		return ret;
error:
	if (chip->secondary)
		i2c_unregister_device(chip->secondary);

	return ret;
}

static void max77779_fg_i2c_remove(struct i2c_client *client)
{
	struct max77779_fg_chip *chip = i2c_get_clientdata(client);

	if (chip->secondary)
		i2c_unregister_device(chip->secondary);

	max77779_fg_remove(chip);
}

static const struct of_device_id max77779_fg_i2c_of_match[] = {
	{ .compatible = "maxim,max77779fg-i2c"},
	{},
};
MODULE_DEVICE_TABLE(of, max77779_fg_i2c_of_match);

#if IS_ENABLED(CONFIG_PM)
static const struct dev_pm_ops max77779_fg_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(max77779_fg_pm_suspend, max77779_fg_pm_resume)
};
#endif

static struct i2c_driver max77779_fg_i2c_driver = {
	.driver = {
		   .name = "max77779-fg",
		   .of_match_table = max77779_fg_i2c_of_match,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &max77779_fg_pm_ops,
#endif
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   },
	.id_table = max77779_fg_id,
	.probe = max77779_fg_i2c_probe,
	.remove = max77779_fg_i2c_remove,
};

module_i2c_driver(max77779_fg_i2c_driver);

MODULE_DESCRIPTION("Maxim 77779 Fuel Gauge I2C Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_LICENSE("GPL");
