/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#include <linux/i2c.h>
#include <linux/regmap.h>

#include "max77779_vimon.h"

static const struct regmap_config max77779_vimon_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77779_VIMON_SIZE,
	.readable_reg = max77779_vimon_is_reg,
	.volatile_reg = max77779_vimon_is_reg,
};

static const struct i2c_device_id max77779_vimon_id[] = {
	{"max77779_vimon", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_vimon_id);

static int max77779_vimon_direct_i2c_read(struct max77779_vimon_data *data, u8 reg,
					  unsigned int *val)
{
	struct i2c_client *i2c;
	struct i2c_msg xfer[2];
	int ret;

	dev_dbg(data->dev, "vimon_direct_i2c_read(%02x)\n", reg);

	i2c = i2c_verify_client(data->dev);
	if (!i2c) {
		dev_err(data->dev, "failed to get i2c_client in direct read\n");
		return -EIO;
	}

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 2;
	xfer[1].buf = (u8 *)val;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret == 2)
		return 0;

	return -EIO;
}

static int max77779_vimon_direct_i2c_write(struct max77779_vimon_data *data, u8 reg,
					   unsigned int val)
{
	struct i2c_client *i2c;
	struct i2c_msg xfer;
	u8 xfer_val[3];
	int ret;

	dev_dbg(data->dev, "vimon_direct_i2c_write(%02x): %04x\n", reg, (u16)val);

	i2c = i2c_verify_client(data->dev);
	if (!i2c) {
		dev_err(data->dev, "failed to get i2c_client in direct write\n");
		return -EIO;
	}

	xfer_val[0] = reg;
	xfer_val[1] = val & 0xFF;
	xfer_val[2] = (val >> 8) & 0xFF;

	xfer.addr = i2c->addr;
	xfer.flags = 0;
	xfer.len = 3;
	xfer.buf = xfer_val;

	ret = i2c_transfer(i2c->adapter, &xfer, 1);
	if (ret == 1)
		return 0;

	return -EIO;
}
static int max77779_vimon_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77779_vimon_data *data;
	struct regmap *regmap;

	/* pmic-irq driver needs to setup the irq */
	if (client->irq < 0)
		return -EPROBE_DEFER;

	regmap = devm_regmap_init_i2c(client, &max77779_vimon_regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->regmap = regmap;
	data->irq = client->irq;
	i2c_set_clientdata(client, data);

	data->direct_reg_read = &max77779_vimon_direct_i2c_read;
	data->direct_reg_write = &max77779_vimon_direct_i2c_write;

	return max77779_vimon_init(data);
}

static void max77779_vimon_i2c_remove(struct i2c_client *client)
{
	struct max77779_vimon_data *data = i2c_get_clientdata(client);

	max77779_vimon_remove(data);
}

static const struct of_device_id max77779_vimon_of_match_table[] = {
	{ .compatible = "maxim,max77779vimon-i2c"},
	{},
};
MODULE_DEVICE_TABLE(of, max77779_vimon_of_match_table);

static struct i2c_driver max77779_vimon_i2c_driver = {
	.driver = {
		.name = "max77779-vimon",
		.owner = THIS_MODULE,
		.of_match_table = max77779_vimon_of_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = max77779_vimon_id,
	.probe	= max77779_vimon_i2c_probe,
	.remove   = max77779_vimon_i2c_remove,
};

module_i2c_driver(max77779_vimon_i2c_driver);
MODULE_DESCRIPTION("Maxim 77779 Vimon I2C Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_LICENSE("GPL");
