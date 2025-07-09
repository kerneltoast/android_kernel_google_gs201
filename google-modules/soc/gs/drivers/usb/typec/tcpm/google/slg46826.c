// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Google Inc
 *
 * slg46826 driver for OTG support.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>

#define COMMAND_REGISTER	0x7a
#define CLEAR_RESET_LOCK	0x0
#define ENABLE_TCPC_CONTROL	0x23

static const struct regmap_config slg46826_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x7F,
};

static int write_command_reg(struct regmap *regmap, u8 val)
{
	return regmap_raw_write(regmap, COMMAND_REGISTER, &val, sizeof(u8));
}

int enable_ls(struct i2c_client *client)
{
	struct regmap *regmap = i2c_get_clientdata(client);
	int ret = 0;

	ret = write_command_reg(regmap, CLEAR_RESET_LOCK);
	if (ret < 0)
		dev_err(&client->dev, "Error clearing reset lock: %d\n",
			ret);
	ret = write_command_reg(regmap, ENABLE_TCPC_CONTROL);
	if (ret < 0)
		dev_err(&client->dev, "Error enabling TCPC control: %d\n",
			ret);
	return ret;
}
EXPORT_SYMBOL_GPL(enable_ls);

static int slg46826_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client,
				      &slg46826_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "regmap init failed: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	i2c_set_clientdata(client, regmap);
	return 0;
}

static const struct i2c_device_id slg46826_id[] = {
	{ "slg46826", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, slg46826_id);

#ifdef CONFIG_OF
static const struct of_device_id slg46826_of_match[] = {
	{ .compatible = "slg46826", },
	{},
};
MODULE_DEVICE_TABLE(of, slg46826_of_match);
#endif

static struct i2c_driver slg46826_i2c_driver = {
	.driver = {
		.name = "slg46826",
		.of_match_table = of_match_ptr(slg46826_of_match),
	},
	.probe = slg46826_probe,
	.id_table = slg46826_id,
};
module_i2c_driver(slg46826_i2c_driver);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("slg46826 to enable USB OTG path");
MODULE_LICENSE("GPL");
