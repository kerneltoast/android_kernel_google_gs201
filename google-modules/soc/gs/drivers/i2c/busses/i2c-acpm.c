// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Copyright 2020 Google LLC
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

struct acpm_mfd_bus {
	struct i2c_adapter	adap;

	struct device		*dev;
};

static int acpm_mfd_bus_xfer(struct i2c_adapter *adap,
			     struct i2c_msg *msgs, int num)
{
	return 0;
}

static u32 acpm_mfd_bus_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm acpm_mfd_bus_algorithm = {
	.master_xfer		= acpm_mfd_bus_xfer,
	.functionality		= acpm_mfd_bus_func,
};

static int acpm_mfd_bus_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct acpm_mfd_bus *acpm_mfd_bus;
	int ret;

	dev_info(&pdev->dev, "acpm mfd bus driver probe started\n");

	if (!np) {
		dev_err(&pdev->dev, "no device node\n");
		return -ENOENT;
	}

	acpm_mfd_bus = devm_kzalloc(&pdev->dev,
				    sizeof(struct acpm_mfd_bus), GFP_KERNEL);
	if (!acpm_mfd_bus)
		return -ENOMEM;

	strlcpy(acpm_mfd_bus->adap.name, "i2c-acpm",
		sizeof(acpm_mfd_bus->adap.name));
	acpm_mfd_bus->adap.owner   = THIS_MODULE;
	acpm_mfd_bus->adap.algo    = &acpm_mfd_bus_algorithm;
	acpm_mfd_bus->adap.retries = 2;
	acpm_mfd_bus->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;

	acpm_mfd_bus->dev = &pdev->dev;

	acpm_mfd_bus->adap.dev.of_node = np;
	acpm_mfd_bus->adap.algo_data = acpm_mfd_bus;
	acpm_mfd_bus->adap.dev.parent = &pdev->dev;

	platform_set_drvdata(pdev, acpm_mfd_bus);

	acpm_mfd_bus->adap.nr = -1;
	ret = i2c_add_numbered_adapter(&acpm_mfd_bus->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_probe;
	}

	dev_info(&pdev->dev, "acpm mfd bus driver probe was succeeded\n");

	return 0;

 err_probe:
	dev_err(&pdev->dev, "acpm mfd bus driver probe failed\n");
	return ret;
}

static const struct of_device_id acpm_mfd_bus_match[] = {
	{ .compatible = "google,i2c-acpm" },
	{}
};
MODULE_DEVICE_TABLE(of, acpm_mfd_bus_match);

static struct platform_driver acpm_mfd_bus_driver = {
	.probe		= acpm_mfd_bus_probe,
	.driver		= {
		.name	= "i2c-acpm",
		.owner	= THIS_MODULE,
		.of_match_table = acpm_mfd_bus_match,
	},
};

module_platform_driver(acpm_mfd_bus_driver);

MODULE_SOFTDEP("pre: vh_i2c");
MODULE_DESCRIPTION(" I2C ACPM driver");
MODULE_AUTHOR("Hyeonseong Gil, <hs.gil@samsung.com>");
MODULE_LICENSE("GPL v2");
