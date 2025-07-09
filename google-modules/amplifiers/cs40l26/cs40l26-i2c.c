// SPDX-License-Identifier: GPL-2.0
//
// cs40l26-i2c.c -- CS40L26 I2C Driver
//
// Copyright 2022 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include "cs40l26.h"

static const struct i2c_device_id cs40l26_id_i2c[] = {
	{"cs40l26a", 0},
	{"cs40l26b", 1},
	{"cs40l27a", 2},
	{"cs40l27b", 3},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs40l26_id_i2c);

static const struct of_device_id cs40l26_of_match[CS40L26_NUM_DEVS + 1] = {
	{ .compatible = "cirrus,cs40l26a" },
	{ .compatible = "cirrus,cs40l26b" },
	{ .compatible = "cirrus,cs40l27a" },
	{ .compatible = "cirrus,cs40l27b" },
	{}
};
MODULE_DEVICE_TABLE(of, cs40l26_of_match);

static int cs40l26_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cs40l26_private *cs40l26;
	int error;

	cs40l26 = devm_kzalloc(&client->dev, sizeof(struct cs40l26_private), GFP_KERNEL);
	if (!cs40l26)
		return -ENOMEM;

	i2c_set_clientdata(client, cs40l26);

	cs40l26->regmap = devm_regmap_init_i2c(client, &cs40l26_regmap);
	if (IS_ERR(cs40l26->regmap)) {
		error = PTR_ERR(cs40l26->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n", error);
		return error;
	}

	cs40l26->dev = &client->dev;
	cs40l26->irq = client->irq;

	error = cs40l26_probe(cs40l26);
	if ((error != 0) && (error != -ENOMEM)) {
		if (++cs40l26_probed_retry_count[cs40l26->device_id] ==
				CS40L26_MAX_PROBE_RETRY) {
			dev_err(&client->dev, "Failed to probe.\n");
			cs40l26->cs40l26_not_probed = true;
			cs40l26_add_codec_devices(&client->dev);
			return 0;
		}

		dev_err(&client->dev, "Failed to probe. Try to defer probe: %d\n", error);
		error = -EPROBE_DEFER;
	}
	return error;
}

static void cs40l26_i2c_remove(struct i2c_client *client)
{
	struct cs40l26_private *cs40l26 = i2c_get_clientdata(client);

	cs40l26_remove(cs40l26);
}

static struct i2c_driver cs40l26_i2c_driver = {
	.driver = {
		.name = "cs40l26",
		.of_match_table = cs40l26_of_match,
		.pm = &cs40l26_pm_ops,
	},
	.id_table = cs40l26_id_i2c,
	.probe = cs40l26_i2c_probe,
	.remove = cs40l26_i2c_remove,
};

module_i2c_driver(cs40l26_i2c_driver);

MODULE_DESCRIPTION("CS40L26 I2C Driver");
MODULE_AUTHOR("Fred Treven, Cirrus Logic Inc. <fred.treven@cirrus.com>");
MODULE_LICENSE("GPL");
