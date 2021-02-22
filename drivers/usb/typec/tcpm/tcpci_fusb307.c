// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Google Inc
 *
 * FUSB307B TCPCI driver
 */

#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/tcpm.h>
#include <misc/gvotable.h>
#include <misc/logbuffer.h>

#include "tcpci_otg_helper.h"
#include "tcpci.h"
#include "usb_icl_voter.h"
#include "usb_psy.h"

#define VBUS_VOLTAGE_MASK		0x3ff
#define VBUS_VOLTAGE_LSB_MV		25
#define VBUS_HI_HEADROOM_MV		500
#define VBUS_LO_MV			4500

#define PD_ACTIVITY_TIMEOUT_MS 10000

struct fusb307b_plat {
	struct tcpci_data data;
	struct tcpci *tcpci;
	struct device *dev;

	/* Optional: vbus regulator to turn on if set in device tree */
	struct  regulator *vbus;
	bool turn_on_vbus;

	/* Optional: Gpio to set through the MAX PMIC */
	u32 uic_gpio;
	bool vbus_enabled;

	/* Data role notified to the data stack */
	enum typec_data_role active_data_role;
	/* Data role from the TCPM stack */
	enum typec_data_role data_role;
	/* protects tcpc_enable_data_path */
	struct mutex data_path_lock;
	/* Vote for data from BC1.2 */
	bool bc12_data_capable;
	/* Infered from pd caps */
	bool pd_data_capable;
	/* Vote from TCPC for attached */
	bool attached;
	/* Reflects the signal sent out to the data stack */
	bool data_active;
	/* Reflects whether the current partner can do PD */
	bool pd_capable;
	void *usb_psy_data;
	struct power_supply *usb_psy;
	struct gvotable_election *usb_icl_proto_el;
	struct mutex icl_proto_el_lock;
	/* Set vbus voltage alarms */
	bool set_voltage_alarm;
	unsigned int vbus_mv;
	/* USB Data notification */
	struct extcon_dev *extcon;
	bool no_bc_12;
	struct usb_psy_ops psy_ops;

	struct logbuffer *log;

	struct i2c_client *uic_i2c_client;
	struct device_node *uic_device_node;
	struct i2c_client *ls_i2c_client;
	struct device_node *ls_device_node;
};

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

#define FUSB307B_UPDATE_BITS(width, chip, reg, mask, val)		\
{									\
	u##width status;						\
	int ret;							\
									\
	ret = fusb307b_read##width(chip, reg, &status);			\
	if (ret < 0)							\
		return ret;						\
									\
	return fusb307b_write##width(chip, reg, (status & ~mask) |	\
				      val);				\
}

int fusb307b_update_bits16(struct fusb307b_plat *chip, unsigned int reg,
			   u16 mask, u16 val)
{
	FUSB307B_UPDATE_BITS(16, chip, reg, mask, val);
}
EXPORT_SYMBOL_GPL(fusb307b_update_bits16);

int fusb307b_update_bits8(struct fusb307b_plat *chip, unsigned int reg,
			  u8 mask, u8 val)
{
	FUSB307B_UPDATE_BITS(8, chip, reg, mask, val);
}
EXPORT_SYMBOL_GPL(fusb307b_update_bits8);

static struct fusb307b_plat *tdata_to_fusb307b(struct tcpci_data *tdata)
{
	return container_of(tdata, struct fusb307b_plat, data);
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

	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);

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
	if (!client->irq) {
		dev_err(&client->dev, "gpio not found\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(chip->dev, client->irq, NULL,
					fusb307b_irq,
					(IRQF_TRIGGER_LOW | IRQF_ONESHOT),
					dev_name(chip->dev), chip);
	if (ret < 0) {
		dev_err(&client->dev,
			"Interrupt failed to allocate err:%d\n",
			ret);
		return ret;
	}

	enable_irq_wake(client->irq);
	return 0;
}

static enable_load_switch(struct fusb307b_plat *chip)
{
	int ret;

	if (!chip->ls_i2c_client) {
		chip->ls_i2c_client =
			of_find_i2c_device_by_node(chip->ls_device_node);
		if (!chip->ls_i2c_client)
			return -EAGAIN;
	}

	ret = enable_ls(chip->ls_i2c_client);
	logbuffer_log(chip->log, "Load switch enable:%s\n", ret < 0 ?
		      "fail" : "success");
	return ret;
}

static int enable_external_boost(struct fusb307b_plat *chip, bool enable)
{
	int ret = 0;

#if IS_ENABLED(CONFIG_OTG_ENABLE)
	if (!chip->uic_i2c_client) {
		chip->uic_i2c_client =
			of_find_i2c_device_by_node(chip->uic_device_node);
		if (!chip->uic_i2c_client)
			return -EAGAIN;
		max77729_disable_water_detection(chip->uic_i2c_client);
	}

	ret = max77729_gpio_set(chip->uic_i2c_client,
				chip->uic_gpio, true, enable);
	logbuffer_log(chip->log, "Max gpio:%d set %s\n", chip->uic_gpio,
		      ret < 0 ? "fail" : "success");
#endif

	return ret;
}

static int fusb307_set_vbus(struct tcpci *tcpci, struct tcpci_data *tdata,
			    bool source, bool sink)
{
	struct fusb307b_plat *chip = tdata_to_fusb307(tdata);
	int ret;

	/* Ordering is needed here. DO NOT REFACTOR if..else.. */
	if (!source) {
		if (chip->vbus_enabled) {
			if (chip->turn_on_vbus)
				regulator_disable(chip->vbus);
			else if (chip->uic_gpio)
				enable_external_boost(chip, false);
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
			if (chip->turn_on_vbus) {
				regulator_enable(chip->vbus);
			} else if (chip->uic_gpio) {
				ret = enable_load_switch(chip);
				if (ret < 0)
					return ret;

				enable_external_boost(chip, true);
			}
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

static int fusb307b_get_vbus_voltage_max_mv(struct i2c_client *tcpc_client)
{
	u16 raw;
	struct fusb307b_plat *chip = i2c_get_clientdata(tcpc_client);
	int ret;

	if (!chip->set_voltage_alarm)
		return chip->vbus_mv;

	ret = fusb307b_read16(chip, TCPC_VBUS_VOLTAGE_ALARM_HI_CFG, &raw);
	if (ret < 0)
		return chip->vbus_mv;

	return raw * TCPC_VBUS_VOLTAGE_ALARM_HI_CFG - VBUS_HI_HEADROOM_MV;
}

static int fusb307b_set_vbus_voltage_max_mv(struct i2c_client *tcpc_client,
					    unsigned int mv)
{
	struct fusb307b_plat *chip = i2c_get_clientdata(tcpc_client);

	chip->vbus_mv = mv;

	if (!chip->set_voltage_alarm)
		return 0;

	/* Set voltage alarm */
	fusb307b_update_bits16(chip, TCPC_VBUS_VOLTAGE_ALARM_HI_CFG,
			       TCPC_VBUS_VOLTAGE_MASK,
			       (mv + VBUS_HI_HEADROOM_MV) /
			       TCPC_VBUS_VOLTAGE_LSB_MV);
	fusb307b_update_bits16(chip, TCPC_VBUS_VOLTAGE_ALARM_LO_CFG,
			       TCPC_VBUS_VOLTAGE_MASK,
			       VBUS_LO_MV / TCPC_VBUS_VOLTAGE_LSB_MV);
	fusb307b_update_bits8(chip, TCPC_POWER_CTRL, TCPC_DIS_VOLT_ALRM,
			      (u8)~TCPC_DIS_VOLT_ALRM);
	fusb307b_update_bits16(chip, TCPC_ALERT_MASK, TCPC_ALERT_V_ALARM_LO |
			       TCPC_ALERT_V_ALARM_HI, TCPC_ALERT_V_ALARM_LO |
			       TCPC_ALERT_V_ALARM_HI);
	return 0;
}

static int fusb307b_get_vbus_voltage_mv(struct i2c_client *tcpc_client)
{
	u16 raw;
	struct fusb307b_plat *chip = i2c_get_clientdata(tcpc_client);
	int ret;

	/* TCPC_POWER_CTRL_VBUS_VOLT_MON enabled in init_regs */
	ret = fusb307b_read16(chip, TCPC_VBUS_VOLTAGE, &raw);
	if (ret < 0)
		return ret;

	return (raw & TCPC_VBUS_VOLTAGE_MASK) * TCPC_VBUS_VOLTAGE_LSB_MV;
}

static int fusb307b_get_current_limit(struct tcpci *tcpci,
				      struct tcpci_data *tdata)
{
	struct fusb307b_plat *chip = tdata_to_fusb307b(tdata);
	void *vote;
	int ret;

	ret = gvotable_get_vote(chip->usb_icl_proto_el,
				proto_voter_reason[USB_ICL_PD], &vote);
	if (ret < 0)
		logbuffer_log(chip->log, "icl_proto_el get vote failed:%d", ret)
			;
	else
		ret = ((struct usb_vote *)(vote))->val;

	return ret;
}

static void enable_data_path_locked(struct fusb307b_plat *chip)
{
	int ret;
	bool enable_data;

	logbuffer_log(chip->log,
		      "%s pd_capable:%u pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u",
		      __func__, chip->pd_capable ? 1 : 0,
		      chip->pd_data_capable ? 1 : 0, chip->no_bc_12 ? 1 : 0,
		      chip->bc12_data_capable ? 1 : 0, chip->attached ? 1
		      : 0);
	dev_info(chip->dev,
		 "TCPM_DEBUG %s pd_capable:%u pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u",
		 __func__, chip->pd_capable ? 1 : 0, chip->pd_data_capable
		 ? 1 : 0, chip->no_bc_12 ? 1 : 0, chip->bc12_data_capable
		 ? 1 : 0, chip->attached ? 1 : 0);

	enable_data = (chip->pd_capable && chip->pd_data_capable) || chip->no_bc_12 ||
		chip->bc12_data_capable || chip->data_role == TYPEC_HOST;

	if (chip->attached && enable_data && !chip->data_active) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB,
					    1);
		logbuffer_log(chip->log, "%s turning on %s", ret < 0 ?
			      "Failed" : "Succeeded", chip->data_role ==
			      TYPEC_HOST ? "Host" : "Device");
		dev_info(chip->dev, "TCPM_DEBUG %s turning on %s", ret < 0 ?
			 "Failed" : "Succeeded", chip->data_role ==
			 TYPEC_HOST ? "Host" : "Device");

		chip->data_active = true;
		chip->active_data_role = chip->data_role;
	} else if (chip->data_active && (!chip->attached || !enable_data)) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->active_data_role ==
					    TYPEC_HOST ? EXTCON_USB_HOST :
					    EXTCON_USB, 0);

		logbuffer_log(chip->log, "%s turning off %s", ret < 0 ?
			      "Failed" : "Succeeded",
			      chip->active_data_role == TYPEC_HOST ? "Host"
			      : "Device");
		dev_info(chip->dev, "TCPM_DEBUG %s turning off %s",
			 ret < 0 ? "Failed" : "Succeeded",
			 chip->active_data_role == TYPEC_HOST ? "Host"
			 : "Device");
		chip->data_active = false;
	}
}

static void fusb307b_set_pd_capable(struct tcpci *tcpci, struct tcpci_data
				    *data, bool capable)
{
	struct fusb307b_plat *chip = tdata_to_fusb307b(data);

	mutex_lock(&chip->data_path_lock);
	chip->pd_capable = capable;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);
}

static void fusb307b_set_cc_polarity(struct tcpci *tcpci, struct tcpci_data
				     *data, enum typec_cc_polarity polarity)
{
	struct fusb307b_plat *chip = tdata_to_fusb307b(data);
	int ret;

	ret = extcon_set_property(chip->extcon, EXTCON_USB,
				  EXTCON_PROP_USB_TYPEC_POLARITY,
				  (union extcon_property_value)
				  (int)polarity);
	logbuffer_log(chip->log, "%s setting polarity USB %d", ret < 0 ?
		      "Failed" : "Succeeded", polarity);
	dev_info(chip->dev, "TCPM_DEBUG %s setting polarity USB %d",
		 ret < 0 ? "Failed" : "Succeeded", polarity);

	ret = extcon_set_property(chip->extcon, EXTCON_USB_HOST,
				  EXTCON_PROP_USB_TYPEC_POLARITY,
				  (union extcon_property_value)
				  (int)polarity);
	logbuffer_log(chip->log, "%s setting polarity USB_HOST %d", ret < 0 ?
		      "Failed" : "Succeeded", polarity);
	dev_info(chip->dev, "TCPM_DEBUG %s setting polarity USB %d",
		 ret < 0 ? "Failed" : "Succeeded", polarity);
}

static int fusb307b_vote_icl(struct tcpci *tcpci, struct tcpci_data *tdata,
			     u32 max_ma)
{
	struct fusb307b_plat *chip = tdata_to_fusb307b(tdata);
	int ret = 0;
	struct usb_vote vote;

	/*
	 * TCPM sets max_ma to zero for Rp-default which needs to be
	 * ignored.
	 */
	mutex_lock(&chip->icl_proto_el_lock);
	if (!chip->pd_capable && max_ma == 0 && chip->attached)
		goto exit;

	init_vote(&vote, proto_voter_reason[USB_ICL_PD], USB_ICL_PD, max_ma
		  * 1000);
	ret = gvotable_cast_vote(chip->usb_icl_proto_el,
				 proto_voter_reason[USB_ICL_PD], &vote,
				 chip->attached);

	logbuffer_log(chip->log,
		      "%s: %s:%d voting enabled:%s usb proto_el: %d by %s",
		      __func__, ret < 0 ? "error" : "success", ret,
		      chip->attached ? "enabled" : "disabled", vote.val,
		      proto_voter_reason[USB_ICL_PD]);

exit:
	mutex_unlock(&chip->icl_proto_el_lock);
	return ret;
}

static int fusb307b_set_current_limit(struct tcpci *tcpci,
				      struct tcpci_data *tdata,
				      u32 max_ma, u32 mv)
{
	struct fusb307b_plat *chip = tdata_to_fusb307b(tdata);
	union power_supply_propval val = {0};
	int ret;

	/* Setprop in uv */
	val.intval = mv * 1000;
	ret = power_supply_set_property(chip->usb_psy,
					POWER_SUPPLY_PROP_VOLTAGE_MAX,
					&val);
	if (ret < 0) {
		logbuffer_log(chip->log,
			      "unable to set max voltage to %d, ret=%d",
				mv, ret);
		return ret;
	}

	logbuffer_log(chip->log, "mv=%d", mv);
	fusb307b_vote_icl(tcpci, tdata, max_ma);

	return ret;
}

/* Notifier structure inferred from usbpd-manager.c */
static int fusb307_set_roles(struct tcpci *tcpci, struct tcpci_data *data,
			     bool attached, enum typec_role role,
			     enum typec_data_role data_role,
			     bool usb_comm_capable)
{
	struct fusb307b_plat *chip = tdata_to_fusb307(data);
	int ret;
	bool enable_data;

	mutex_lock(&chip->data_path_lock);
	chip->pd_data_capable = usb_comm_capable;

	enable_data = (chip->pd_capable && chip->pd_data_capable) || chip->no_bc_12 ||
		chip->bc12_data_capable || chip->data_role == TYPEC_HOST;

	if (chip->data_active && ((chip->active_data_role != data_role) ||
				  !attached || !enable_data)) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->active_data_role ==
					    TYPEC_HOST ? EXTCON_USB_HOST :
					    EXTCON_USB, 0);

		logbuffer_log(chip->log, "%s turning off %s", ret < 0 ?
			      "Failed" : "Succeeded",
			      chip->active_data_role == TYPEC_HOST ? "Host"
			      : "Device");
		dev_info(chip->dev, "TCPM_DEBUG %s turning off %s", ret < 0 ?
			 "Failed" : "Succeeded", chip->active_data_role ==
			 TYPEC_HOST ? "Host" : "Device");

		chip->data_active = false;
	}

	chip->attached = attached;
	chip->data_role = data_role;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);
	if (!chip->attached)
		fusb307b_vote_icl(tcpci, data, 0);

	/* Signal usb_psy online */
	usb_psy_set_sink_state(chip->usb_psy_data, role == TYPEC_SINK && attached);
	return 0;
}

static void fusb307b_set_port_data_capable(struct i2c_client *tcpc_client,
					   enum power_supply_usb_type
					   usb_type)
{
	struct fusb307b_plat *chip = i2c_get_clientdata(tcpc_client);

	switch (usb_type) {
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_CDP:
	/* Being conservative, enable data path for UNKNOWN as well */
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		mutex_lock(&chip->data_path_lock);
		chip->bc12_data_capable = true;
		enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);
		break;
	default:
		chip->bc12_data_capable = false;
		break;
	}
}

static const unsigned int usbpd_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DISP_DP,
	EXTCON_NONE,
};

static int fusb307b_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	int ret;
	struct fusb307b_plat *chip;
	struct device_node *dn;
	char *usb_psy_name;
	u32 handle;

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
	if (ret < 0)
		return ret;

	chip->vbus = devm_regulator_get(&client->dev, "vbus");
	if (IS_ERR(chip->vbus)) {
		dev_err(&client->dev, "Regulator init: %d\n", PTR_ERR(
			chip->vbus));
	}

	chip->log = logbuffer_register("usbpd");
	if (IS_ERR_OR_NULL(chip->log)) {
		dev_err(&client->dev, "logbuffer get failed");
		chip->log = NULL;
	}

	mutex_init(&chip->data_path_lock);
	mutex_init(&chip->icl_proto_el_lock);

	dn = dev_of_node(&client->dev);
	if (!dn) {
		dev_err(&client->dev, "of node not found\n");
		ret = -EINVAL;
		goto unreg_log;
	}

	chip->turn_on_vbus = of_property_read_bool(dn, "turn-on-vbus-reg");

	if (!of_property_read_u32(dn, "uic", &handle)) {
		chip->uic_device_node = of_find_node_by_phandle(handle);
		of_property_read_u32(dn, "uic-io", &chip->uic_gpio);
		logbuffer_log(chip->log, "uic gpio: %d", chip->uic_gpio);
	} else {
		dev_err(&client->dev, "UIC device node not found\n");
	}

	if (!of_property_read_u32(dn, "ls", &handle)) {
		chip->ls_device_node = of_find_node_by_phandle(handle);
	} else {
		dev_err(&client->dev, "ls device node not found\n");
		ret = -EINVAL;
		goto unreg_log;
	}

	chip->psy_ops.tcpc_get_vbus_voltage_mv =
		fusb307b_get_vbus_voltage_mv;
	chip->psy_ops.tcpc_get_vbus_voltage_max_mv =
		fusb307b_get_vbus_voltage_max_mv;
	chip->psy_ops.tcpc_set_vbus_voltage_max_mv =
		fusb307b_set_vbus_voltage_max_mv;
	chip->psy_ops.tcpc_set_port_data_capable =
		fusb307b_set_port_data_capable;

	chip->usb_psy_data = usb_psy_setup(client, chip->log,
					   &chip->psy_ops);
	if (IS_ERR_OR_NULL(chip->usb_psy_data)) {
		dev_err(&client->dev, "USB psy failed to initialize");
		ret = PTR_ERR(chip->usb_psy_data);
		goto unreg_log;
	}

	chip->data.set_vbus = fusb307_set_vbus;
	chip->data.set_roles = fusb307_set_roles;
	chip->data.get_current_limit = fusb307b_get_current_limit;
	chip->data.set_current_limit = fusb307b_set_current_limit;
	chip->data.set_pd_capable = fusb307b_set_pd_capable;
	chip->data.set_cc_polarity = fusb307b_set_cc_polarity;

	chip->usb_icl_proto_el = gvotable_election_get_handle(USB_ICL_PROTO_EL);
	if (IS_ERR_OR_NULL(chip->usb_icl_proto_el)) {
		dev_err(&client->dev, "TCPCI: USB ICL PROTO EL get failed:%d",
			PTR_ERR(chip->usb_icl_proto_el));
		ret = -ENODEV;
		goto unreg_port;
	}

	usb_psy_name = (char *)of_get_property(dn, "usb-psy-name", NULL);
	if (!usb_psy_name) {
		dev_err(&client->dev, "usb-psy-name not set\n");
		ret = -EINVAL;
		goto unreg_psy;
	}

	chip->no_bc_12 = of_property_read_bool(dn, "no-bc-12");

	chip->usb_psy = power_supply_get_by_name(usb_psy_name);
	if (IS_ERR_OR_NULL(chip->usb_psy)) {
		dev_err(&client->dev, "usb psy not up\n");
		ret = -EPROBE_DEFER;
		goto unreg_psy;
	}

	chip->extcon = devm_extcon_dev_allocate(&client->dev,
						usbpd_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		dev_err(&client->dev, "Error allocating extcon: %d\n",
			PTR_ERR(chip->extcon));
		return PTR_ERR(chip->extcon);
	}

	ret = devm_extcon_dev_register(&client->dev, chip->extcon);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register extcon device");
		return ret;
	}

	extcon_set_property_capability(chip->extcon, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_TYPEC_POLARITY);

	chip->tcpci = tcpci_register_port(chip->dev, &chip->data);
	if (IS_ERR_OR_NULL(chip->tcpci)) {
		dev_err(&client->dev, "TCPCI register failed: %d\n",
			PTR_ERR(chip->tcpci));
		ret = PTR_ERR(chip->tcpci);
		goto psy_put;
	}

	device_init_wakeup(chip->dev, true);
	return 0;

psy_put:
	power_supply_put(chip->usb_psy);
unreg_psy:
	usb_psy_teardown(chip->usb_psy_data);
unreg_port:
	tcpci_unregister_port(chip->tcpci);
unreg_log:
	logbuffer_unregister(chip->log);

	return ret;
}

static int fusb307b_remove(struct i2c_client *client)
{
	struct fusb307b_plat *chip = i2c_get_clientdata(client);

	logbuffer_unregister(chip->log);
	tcpci_unregister_port(chip->tcpci);
	power_supply_put(chip->usb_psy);
	usb_psy_teardown(chip->usb_psy_data);

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
