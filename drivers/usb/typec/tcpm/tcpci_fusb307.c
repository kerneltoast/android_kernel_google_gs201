// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Google Inc
 *
 * FUSB307B TCPCI driver
 */

#include <linux/ccic/core.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ifconn/ifconn_notifier.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/tcpm.h>
#include "tcpci.h"

struct fusb307b_plat {
	struct tcpci_data data;
	struct tcpci *tcpci;
	struct device *dev;
	struct  regulator *vbus;
	bool vbus_enabled;
	bool data_active;
	enum typec_data_role active_data_role;
};

static int fusb307b_read16(struct fusb307b_plat *chip, unsigned int reg,
			   u16 *val) __attribute__((used));
static int fusb307b_write16(struct fusb307b_plat *chip, unsigned int reg,
			    u16 val) __attribute__((used));
static int fusb307b_read8(struct fusb307b_plat *chip, unsigned int reg,
			  u8 *val) __attribute__((used));
static int fusb307b_write8(struct fusb307b_plat *chip, unsigned int reg,
			   u8 val) __attribute__((used));

static int fusb307b_read16(struct fusb307b_plat *chip, unsigned int reg,
			   u16 *val)
{
	return regmap_raw_read(chip->data.regmap, reg, val, sizeof(u16));
}

static int fusb307b_write16(struct fusb307b_plat *chip, unsigned int reg,
			    u16 val)
{
	return regmap_raw_write(chip->data.regmap, reg, &val, sizeof(u16));
}

static int fusb307b_read8(struct fusb307b_plat *chip, unsigned int reg,
			  u8 *val)
{
	return regmap_raw_read(chip->data.regmap, reg, val, sizeof(u8));
}

static int fusb307b_write8(struct fusb307b_plat *chip, unsigned int reg,
			   u8 val)
{
	return regmap_raw_write(chip->data.regmap, reg, &val, sizeof(u8));
}

static const struct regmap_config fusb307b_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x7F,
};

static struct fusb307b_plat *tdata_to_fusb307(struct tcpci_data *tdata)
{
	return container_of(tdata, struct fusb307b_plat, data);
}

static irqreturn_t fusb307b_irq(int irq, void *dev_id)
{
	struct fusb307b_plat *chip = dev_id;

	if (!chip->tcpci)
		return IRQ_HANDLED;

	return tcpci_irq(chip->tcpci);
}

static int fusb307b_init_alert(struct fusb307b_plat *chip,
			       struct i2c_client *client)
{
	int ret, irq_gpio;

	irq_gpio = of_get_named_gpio(client->dev.of_node, "usbpd,usbpd_int", 0);
	client->irq = gpio_to_irq(irq_gpio);
	if (!client->irq)
		return -ENODEV;

	ret = devm_request_threaded_irq(chip->dev, client->irq, NULL,
					fusb307b_irq,
					(IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND |
					 IRQF_ONESHOT),
					dev_name(chip->dev), chip);
	if (ret < 0)
		return ret;

	enable_irq_wake(client->irq);
	return 0;
}

static int fusb307_set_vbus(struct tcpci *tcpci, struct tcpci_data *tdata,
			    bool source, bool sink)
{
	struct fusb307b_plat *chip = tdata_to_fusb307(tdata);
	int ret;

	/** Ordering is needed here. DO NOT REFACTOR if..else.. **/
	if (!source) {
		if (chip->vbus_enabled) {
			ret = regulator_disable(chip->vbus);
			if (ret < 0)
				return ret;
			chip->vbus_enabled = false;
		}

		ret = regmap_write(chip->data.regmap, TCPC_COMMAND,
				   TCPC_CMD_DISABLE_SRC_VBUS);
		if (ret < 0)
			return ret;
	}

	if (!sink) {
		ret = regmap_write(chip->data.regmap, TCPC_COMMAND,
				   TCPC_CMD_DISABLE_SINK_VBUS);
		if (ret < 0)
			return ret;
	}

	if (source) {
		if (!chip->vbus_enabled) {
			ret = regulator_enable(chip->vbus);
			if (ret < 0)
				return ret;
			chip->vbus_enabled = true;
		}

		ret = regmap_write(chip->data.regmap, TCPC_COMMAND,
				   TCPC_CMD_SRC_VBUS_DEFAULT);
		if (ret < 0)
			return ret;
	}

	if (sink) {
		ret = regmap_write(chip->data.regmap, TCPC_COMMAND,
				   TCPC_CMD_SINK_VBUS);
		if (ret < 0)
			return ret;
	}

	return 0;
}

// Notifier structure inferred from usbpd-manager.c
static int fusb307_set_roles(struct tcpci *tcpci, struct tcpci_data *data,
			     bool attached, enum typec_role role,
			     enum typec_data_role data_role)
{
	struct fusb307b_plat *chip = tdata_to_fusb307(data);
	int ret = 0;

	if ((chip->data_active && chip->active_data_role != data_role) ||
		!attached) {
		USBPD_SEND_DNOTI(IFCONN_NOTIFY_USB, USB,
				 IFCONN_NOTIFY_EVENT_DETACH, NULL);
		USBPD_SEND_DNOTI(IFCONN_NOTIFY_MUIC, ATTACH,
				 IFCONN_NOTIFY_EVENT_DETACH, NULL);
		chip->data_active = false;
	}

	// Data stack needs a clean up to fix this.
	msleep(300);

	if (attached) {
		USBPD_SEND_DNOTI(IFCONN_NOTIFY_MUIC, ATTACH,
				 IFCONN_NOTIFY_EVENT_ATTACH, NULL);

		if (data_role == TYPEC_DEVICE) {
			USBPD_SEND_DNOTI(IFCONN_NOTIFY_USB, USB,
					 IFCONN_NOTIFY_EVENT_USB_ATTACH_UFP,
					 NULL);
		} else if (data_role == TYPEC_HOST) {
			USBPD_SEND_DNOTI(IFCONN_NOTIFY_USB, USB,
					 IFCONN_NOTIFY_EVENT_USB_ATTACH_DFP,
					 NULL);
		}
		chip->data_active = true;
		chip->active_data_role = data_role;
	}

	return ret;
}

static int fusb307b_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	int ret;
	struct fusb307b_plat *chip;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->data.regmap = devm_regmap_init_i2c(client,
						 &fusb307b_regmap_config);
	if (IS_ERR(chip->data.regmap)) {
		dev_err(&client->dev, "regmap init failed: %d\n",
			PTR_ERR(chip->data.regmap));
		return PTR_ERR(chip->data.regmap);
	}

	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	ret = fusb307b_init_alert(chip, client);
	if (ret < 0) {
		dev_err(&client->dev, "Alert failed to initialize: %d\n", ret);
		return ret;
	}

	chip->vbus = devm_regulator_get(&client->dev, "vbus");
	if (IS_ERR(chip->vbus)) {
		dev_err(&client->dev, "Regulator init: %d\n", PTR_ERR(
			chip->vbus));
		return PTR_ERR(chip->vbus);
	}

	chip->data.set_vbus = fusb307_set_vbus;
	chip->data.set_roles = fusb307_set_roles;

	chip->tcpci = tcpci_register_port(chip->dev, &chip->data);
	if (IS_ERR_OR_NULL(chip->tcpci)) {
		dev_err(&client->dev, "TCPCI register failed: %d\n",
			PTR_ERR(chip->tcpci));
		return PTR_ERR(chip->tcpci);
	}

	return 0;
}

static int fusb307b_remove(struct i2c_client *client)
{
	struct fusb307b_plat *chip = i2c_get_clientdata(client);

	tcpci_unregister_port(chip->tcpci);
	return 0;
}

static const struct i2c_device_id fusb307b_id[] = {
	{ "fusb307b", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fusb307b_id);

#ifdef CONFIG_OF
static const struct of_device_id fusb307b_of_match[] = {
	{ .compatible = "fusb307b", },
	{},
};
MODULE_DEVICE_TABLE(of, fusb307b_of_match);
#endif

static struct i2c_driver fusb307b_i2c_driver = {
	.driver = {
		.name = "fusb307b",
		.of_match_table = of_match_ptr(fusb307b_of_match),
	},
	.probe = fusb307b_probe,
	.remove = fusb307b_remove,
	.id_table = fusb307b_id,
};
module_i2c_driver(fusb307b_i2c_driver);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("FUSB307B USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL");
