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
#include <linux/usb/role.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec_mux.h>
#include <misc/gvotable.h>
#include <misc/logbuffer.h>

#include "tcpci_otg_helper.h"
#include "google_tcpci_shim.h"
#include "usb_icl_voter.h"
#include "usb_psy.h"

#define VBUS_VOLTAGE_MASK		0x3ff
#define VBUS_VOLTAGE_LSB_MV		25
#define VBUS_HI_HEADROOM_MV		500
#define VBUS_LO_MV			4500

#define PD_ACTIVITY_TIMEOUT_MS 10000

enum tcpm_psy_online_states {
	TCPM_PSY_OFFLINE = 0,
	TCPM_PSY_FIXED_ONLINE,
	TCPM_PSY_PROG_ONLINE,
};

struct fusb307b_plat {
	struct google_shim_tcpci_data data;
	struct google_shim_tcpci *tcpci;
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

	struct notifier_block psy_notifier;
	int online;
	int usb_type;
	int typec_current_max;
	struct kthread_worker *wq;
	struct kthread_delayed_work icl_work;

	/* Notifier for data role */
	struct usb_role_switch *usb_sw;
	/* Notifier for orientation */
	struct typec_switch_dev *typec_sw;

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

static struct fusb307b_plat *tdata_to_fusb307b(struct google_shim_tcpci_data *tdata)
{
	return container_of(tdata, struct fusb307b_plat, data);
}

static const struct regmap_config fusb307b_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x7F,
};

static struct fusb307b_plat *tdata_to_fusb307(struct google_shim_tcpci_data *tdata)
{
	return container_of(tdata, struct fusb307b_plat, data);
}

static irqreturn_t fusb307b_irq(int irq, void *dev_id)
{
	struct fusb307b_plat *chip = dev_id;

	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);

	if (!chip->tcpci)
		return IRQ_HANDLED;

	return google_tcpci_shim_irq(chip->tcpci);
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

static int enable_load_switch(struct fusb307b_plat *chip)
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

static int fusb307_set_vbus(struct google_shim_tcpci *tcpci, struct google_shim_tcpci_data *tdata,
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
				ret = regulator_enable(chip->vbus);
				if (ret) {
					dev_err(chip->dev, "Error enabling vbus regulator, ret=%d\n", ret);
					return ret;
				}
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

static void enable_data_path_locked(struct fusb307b_plat *chip)
{
	int ret;
	bool enable_data;

	logbuffer_log(chip->log,
		      "%s pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u",
		      __func__, chip->pd_data_capable ? 1 : 0, chip->no_bc_12 ? 1 : 0,
		      chip->bc12_data_capable ? 1 : 0, chip->attached ? 1
		      : 0);
	dev_info(chip->dev,
		 "TCPM_DEBUG %s pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u",
		 __func__, chip->pd_data_capable ? 1 : 0, chip->no_bc_12 ? 1 : 0,
		 chip->bc12_data_capable ? 1 : 0, chip->attached ? 1 : 0);

	enable_data = chip->pd_data_capable || chip->no_bc_12 ||
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

static void fusb307b_set_pd_data_capable(struct google_shim_tcpci *tcpci,
					 struct google_shim_tcpci_data *data, bool capable)
{
	struct fusb307b_plat *chip = tdata_to_fusb307b(data);

	mutex_lock(&chip->data_path_lock);
	chip->pd_data_capable = capable;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);
}

static int fusb307b_usb_set_orientation(struct typec_switch_dev *sw,
					enum typec_orientation orientation)
{
	struct fusb307b_plat *chip = typec_switch_get_drvdata(sw);
	enum typec_cc_polarity polarity = orientation == TYPEC_ORIENTATION_REVERSE ?
		TYPEC_POLARITY_CC2 : TYPEC_POLARITY_CC1;
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
	return ret;
}

static int fusb307b_vote_icl(struct fusb307b_plat *chip, u32 max_ua)
{
	int ret = 0;
	struct usb_vote vote;

	usb_psy_set_sink_state(chip->usb_psy_data, chip->online);
	/*
	 * TCPM sets max_ua to zero for Rp-default which needs to be
	 * ignored. PPS values reflect the requested ones not the max.
	 */
	mutex_lock(&chip->icl_proto_el_lock);
	if ((chip->usb_type != POWER_SUPPLY_USB_TYPE_PD && max_ua == 0 && chip->online) ||
	    chip->online == TCPM_PSY_PROG_ONLINE)
		goto exit;

	init_vote(&vote, proto_voter_reason[USB_ICL_PD], USB_ICL_PD, max_ua);
	ret = gvotable_cast_vote(chip->usb_icl_proto_el,
				 proto_voter_reason[USB_ICL_PD], &vote,
				 chip->online);

	logbuffer_log(chip->log,
		      "%s: %s:%d voting enabled:%s usb proto_el: %d by %s",
		      __func__, ret < 0 ? "error" : "success", ret,
		      chip->online ? "enabled" : "disabled", vote.val,
		      proto_voter_reason[USB_ICL_PD]);

exit:
	mutex_unlock(&chip->icl_proto_el_lock);
	return ret;
}

static void icl_work_item(struct kthread_work *work)
{
	struct fusb307b_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct fusb307b_plat, icl_work);

	fusb307b_vote_icl(chip, chip->typec_current_max);
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct fusb307b_plat *chip = container_of(nb, struct fusb307b_plat, psy_notifier);
	struct power_supply *psy = ptr;
	union power_supply_propval current_max = {0}, voltage_max = {0}, online = {0},
	      usb_type = {0}, val = {0};
	int ret;

	if (!strstr(psy->desc->name, "tcpm-source") || evt != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &current_max);
	power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &voltage_max);
	power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &online);
	power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &usb_type);
	logbuffer_log(chip->log, "psy: %s ONLINE:%d USB_TYPE:%d CURRENT_MAX:%d VOLTAGE_MAX:%d",
		      psy->desc->name, online.intval, usb_type.intval, current_max.intval,
		      voltage_max.intval);

	chip->vbus_mv = voltage_max.intval / 1000;
	ret = power_supply_set_property(chip->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
	if (ret < 0)
		logbuffer_log(chip->log, "unable to set max voltage to %d, ret=%d", chip->vbus_mv,
			      ret);

	chip->online = online.intval;
	chip->usb_type = usb_type.intval;
	chip->typec_current_max = current_max.intval;
	/* Notifier is atomic, hence offloading */
	kthread_mod_delayed_work(chip->wq, &chip->icl_work, 0);
	return NOTIFY_OK;
}

static int fusb307b_usb_set_role(struct usb_role_switch *sw, enum usb_role role)
{
	struct fusb307b_plat *chip = usb_role_switch_get_drvdata(sw);
	enum typec_data_role typec_data_role = TYPEC_DEVICE;
	bool attached = role != USB_ROLE_NONE, enable_data;
	int ret;

	if (role == USB_ROLE_HOST)
		typec_data_role = TYPEC_HOST;

	mutex_lock(&chip->data_path_lock);

	enable_data = chip->pd_data_capable || chip->no_bc_12 ||
		chip->bc12_data_capable || chip->data_role == TYPEC_HOST;

	if (chip->data_active && (chip->active_data_role != typec_data_role ||
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
	chip->data_role = typec_data_role;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);

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

static int fusb307b_setup_data_notifier(struct fusb307b_plat *chip)
{
	struct usb_role_switch_desc desc = { };
	struct typec_switch_desc sw_desc = { };
	u32 conn_handle;
	int ret;

	chip->extcon = devm_extcon_dev_allocate(chip->dev, usbpd_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		dev_err(chip->dev, "Error allocating extcon: %ld\n",
			PTR_ERR(chip->extcon));
		return PTR_ERR(chip->extcon);
	}

	ret = devm_extcon_dev_register(chip->dev, chip->extcon);
	if (ret < 0) {
		dev_err(chip->dev, "failed to register extcon device:%d\n", ret);
		return ret;
	}

	extcon_set_property_capability(chip->extcon, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_TYPEC_POLARITY);

	of_property_read_u32(dev_of_node(chip->dev), "conn", &conn_handle);
	desc.fwnode = &of_find_node_by_phandle(conn_handle)->fwnode;
	desc.driver_data = chip;
	desc.name = fwnode_get_name(dev_fwnode(chip->dev));
	desc.set = fusb307b_usb_set_role;

	chip->usb_sw = usb_role_switch_register(chip->dev, &desc);
	if (IS_ERR(chip->usb_sw)) {
		ret = PTR_ERR(chip->usb_sw);
		dev_err(chip->dev, "Error while registering role switch: %d\n", ret);
		return ret;
	}

	sw_desc.fwnode = dev_fwnode(chip->dev);
	sw_desc.drvdata = chip;
	sw_desc.name = fwnode_get_name(dev_fwnode(chip->dev));
	sw_desc.set = fusb307b_usb_set_orientation;

	chip->typec_sw = typec_switch_register(chip->dev, &sw_desc);
	if (IS_ERR(chip->typec_sw)) {
		ret = PTR_ERR(chip->typec_sw);
		dev_err(chip->dev, "Error while registering orientation switch:%d\n", ret);
		goto usb_sw_free;
	}

	return 0;

usb_sw_free:
	usb_role_switch_unregister(chip->usb_sw);
	return ret;
}

static void fusb307b_teardown_data_notifier(struct fusb307b_plat *chip)
{
	if (!IS_ERR_OR_NULL(chip->usb_sw))
		usb_role_switch_unregister(chip->usb_sw);
}

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
		dev_err(&client->dev, "regmap init failed: %ld\n",
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
		dev_err(&client->dev, "Regulator init: %ld\n", PTR_ERR(
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
					   &chip->psy_ops, chip, NULL);
	if (IS_ERR_OR_NULL(chip->usb_psy_data)) {
		dev_err(&client->dev, "USB psy failed to initialize");
		ret = PTR_ERR(chip->usb_psy_data);
		goto unreg_log;
	}

	chip->data.set_vbus = fusb307_set_vbus;
	chip->data.set_partner_usb_comm_capable = fusb307b_set_pd_data_capable;

	chip->usb_icl_proto_el = gvotable_election_get_handle(USB_ICL_PROTO_EL);
	if (IS_ERR_OR_NULL(chip->usb_icl_proto_el)) {
		dev_err(&client->dev, "TCPCI: USB ICL PROTO EL get failed:%ld",
			PTR_ERR(chip->usb_icl_proto_el));
		ret = -ENODEV;
		goto unreg_psy;
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

	ret = fusb307b_setup_data_notifier(chip);
	if (ret < 0)
		goto psy_put;

	chip->wq = kthread_create_worker(0, "wq-tcpm-tcpc");
	if (IS_ERR_OR_NULL(chip->wq)) {
		ret = PTR_ERR(chip->wq);
		goto teardown_data;
	}

	kthread_init_delayed_work(&chip->icl_work, icl_work_item);

	chip->psy_notifier.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chip->psy_notifier);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register power supply callback\n");
		goto destroy_worker;
	}

	chip->tcpci = google_tcpci_shim_register_port(chip->dev, &chip->data);
	if (IS_ERR_OR_NULL(chip->tcpci)) {
		dev_err(&client->dev, "TCPCI register failed: %ld\n",
			PTR_ERR(chip->tcpci));
		ret = PTR_ERR(chip->tcpci);
		goto unreg_notifier;
	}

	device_init_wakeup(chip->dev, true);
	return 0;

unreg_notifier:
	power_supply_unreg_notifier(&chip->psy_notifier);
destroy_worker:
	kthread_destroy_worker(chip->wq);
teardown_data:
	fusb307b_teardown_data_notifier(chip);
psy_put:
	power_supply_put(chip->usb_psy);
unreg_psy:
	usb_psy_teardown(chip->usb_psy_data);
unreg_log:
	logbuffer_unregister(chip->log);

	return ret;
}

static void fusb307b_remove(struct i2c_client *client)
{
	struct fusb307b_plat *chip = i2c_get_clientdata(client);

	logbuffer_unregister(chip->log);
	google_tcpci_shim_unregister_port(chip->tcpci);
	power_supply_put(chip->usb_psy);
	usb_psy_teardown(chip->usb_psy_data);
	kthread_destroy_worker(chip->wq);
	power_supply_unreg_notifier(&chip->psy_notifier);
	fusb307b_teardown_data_notifier(chip);
}

static void fusb307b_shutdown(struct i2c_client *client)
{
	struct fusb307b_plat *chip = i2c_get_clientdata(client);

	power_supply_unreg_notifier(&chip->psy_notifier);
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
	.shutdown = fusb307b_shutdown,
	.id_table = fusb307b_id,
};
module_i2c_driver(fusb307b_i2c_driver);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("FUSB307B USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL");
