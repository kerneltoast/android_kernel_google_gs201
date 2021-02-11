// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019, Google LLC
 *
 * MAX77759 TCPCI driver
 */

#include <linux/debugfs.h>
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
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <misc/logbuffer.h>

#include "bc_max77759.h"
#include "max77759_helper.h"
#include "tcpci.h"
#include "tcpci_max77759.h"
#include "tcpci_max77759_vendor_reg.h"
#include "usb_icl_voter.h"
#include "usb_psy.h"

#define TCPCI_MODE_VOTER	"TCPCI"

#define TCPC_RECEIVE_BUFFER_COUNT_OFFSET                0
#define TCPC_RECEIVE_BUFFER_FRAME_TYPE_OFFSET           1
#define TCPC_RECEIVE_BUFFER_RX_BYTE_BUF_OFFSET          2

/*
 * LongMessage not supported, hence 32 bytes for buf to be read from RECEIVE_BUFFER.
 * DEVICE_CAPABILITIES_2.LongMessage = 0, the value in READABLE_BYTE_COUNT reg shall be
 * less than or equal to 31. Since, RECEIVE_BUFFER len = 31 + 1(READABLE_BYTE_COUNT).
 */
#define TCPC_RECEIVE_BUFFER_LEN                         32

#define PD_ACTIVITY_TIMEOUT_MS				10000

#define GBMS_MODE_VOTABLE "CHARGER_MODE"

#define MAX77759_DEVICE_ID_A1				0x2

/* system use cases */
enum gbms_charger_modes {
	GBMS_USB_BUCK_ON	= 0x30,
	GBMS_USB_OTG_ON		= 0x31,
	GBMS_USB_OTG_FRS_ON	= 0x32,
};

#define CONTAMINANT_DETECT_MAXQ	2

#define TCPM_RESTART_TOGGLING		0
#define CONTAMINANT_HANDLES_TOGGLING	1

struct tcpci {
	struct device *dev;
	struct tcpm_port *port;
	struct regmap *regmap;
	bool controls_vbus;

	struct tcpc_dev tcpc;
	struct tcpci_data *data;
};

static const struct regmap_range max77759_tcpci_range[] = {
	regmap_reg_range(0x00, 0x95)
};

const struct regmap_access_table max77759_tcpci_write_table = {
	.yes_ranges = max77759_tcpci_range,
	.n_yes_ranges = ARRAY_SIZE(max77759_tcpci_range),
};

static const struct regmap_config max77759_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x95,
	.wr_table = &max77759_tcpci_write_table,
};

static ssize_t frs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->frs);
};

static ssize_t frs_store(struct device *dev, struct device_attribute *attr, const char *buf,
			 size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	int val;

	if (kstrtoint(buf, 10, &val) < 0)
		return -EINVAL;

	chip->frs = val;
	logbuffer_log(chip->log, "[%s]: %d", __func__, chip->frs);
	return count;
}
static DEVICE_ATTR_RW(frs);

static ssize_t auto_discharge_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->data.auto_discharge_disconnect ? 1 : 0);
};

static ssize_t auto_discharge_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	int val;

	if (kstrtoint(buf, 10, &val) < 0)
		return -EINVAL;

	chip->data.auto_discharge_disconnect = !!val;
	logbuffer_log(chip->log, "[%s]: %d", __func__, chip->data.auto_discharge_disconnect);
	tcpci_auto_discharge_update(chip->tcpci);

	return count;
}
static DEVICE_ATTR_RW(auto_discharge);

static ssize_t contaminant_detection_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->contaminant_detection);
};

static ssize_t contaminant_detection_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	int val;

	if (kstrtoint(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&chip->contaminant_detection_lock);
	chip->contaminant_detection = val;

	if (chip->contaminant_detection)
		enable_contaminant_detection(chip, chip->contaminant_detection ==
					     CONTAMINANT_DETECT_MAXQ);
	else
		disable_contaminant_detection(chip);

	logbuffer_log(chip->log, "[%s]: %d", __func__, chip->contaminant_detection);
	mutex_unlock(&chip->contaminant_detection_lock);
	return count;
}
static DEVICE_ATTR_RW(contaminant_detection);

static ssize_t contaminant_detection_status_show(struct device *dev, struct device_attribute *attr,
						 char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	struct max77759_contaminant *contaminant;

	if (!chip)
		return -EAGAIN;

	contaminant = chip->contaminant;

	if (!contaminant)
		return -EAGAIN;

	return scnprintf(buf, PAGE_SIZE, "%d\n", is_contaminant_detected(chip));
}
static DEVICE_ATTR_RO(contaminant_detection_status);

static struct device_attribute *max77759_device_attrs[] = {
	&dev_attr_frs,
	&dev_attr_auto_discharge,
	&dev_attr_contaminant_detection,
	&dev_attr_contaminant_detection_status,
	NULL
};

#ifdef CONFIG_GPIOLIB
static int ext_bst_en_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIOF_DIR_OUT;
}

static int ext_bst_en_gpio_get(struct gpio_chip *gpio, unsigned int offset)
{
	int ret;
	u8 val;
	struct max77759_plat *chip = gpiochip_get_data(gpio);
	struct regmap *regmap = chip->data.regmap;

	ret = max77759_read8(regmap, TCPC_VENDOR_EXTBST_CTRL, &val);
	logbuffer_log(chip->log, "%s: ret:%d", __func__, ret);

	return val & EXT_BST_EN;
}

static void ext_bst_en_gpio_set(struct gpio_chip *gpio, unsigned int offset, int value)
{
	int ret;
	struct max77759_plat *chip = gpiochip_get_data(gpio);
	struct regmap *regmap = chip->data.regmap;

	ret = max77759_write8(regmap, TCPC_VENDOR_EXTBST_CTRL, value ? EXT_BST_EN : 0);
	logbuffer_log(chip->log, "%s: ret:%d", __func__, ret);
}

static int ext_bst_en_gpio_init(struct max77759_plat *chip)
{
	int ret;

	/* Setup GPIO controller */
	chip->gpio.owner = THIS_MODULE;
	chip->gpio.parent = chip->dev;
	chip->gpio.label = "max77759_tcpc_gpio";
	chip->gpio.get_direction = ext_bst_en_gpio_get_direction;
	chip->gpio.get = ext_bst_en_gpio_get;
	chip->gpio.set = ext_bst_en_gpio_set;
	chip->gpio.base = -1;
	chip->gpio.ngpio = 1;
	chip->gpio.can_sleep = true;
	chip->gpio.of_node = of_find_node_by_name(chip->dev->of_node, chip->gpio.label);

	if (!chip->gpio.of_node)
		dev_err(chip->dev, "Failed to find %s DT node\n", chip->gpio.label);

	ret = devm_gpiochip_add_data(chip->dev, &chip->gpio, chip);
	if (ret)
		dev_err(chip->dev, "Failed to initialize gpio chip\n");

	return ret;
}
#endif

static struct max77759_plat *tdata_to_max77759(struct tcpci_data *tdata)
{
	return container_of(tdata, struct max77759_plat, data);
}

static void max77759_init_regs(struct regmap *regmap, struct logbuffer *log)
{
	u16 alert_mask = 0;
	int ret;

	ret = max77759_write16(regmap, TCPC_ALERT, 0xffff);
	if (ret < 0)
		return;

	ret = max77759_write16(regmap, TCPC_VENDOR_ALERT, 0xffff);
	if (ret < 0)
		return;

	ret = regmap_write(regmap, TCPC_EXTENDED_STATUS_MASK,
			   TCPC_EXTENDED_STATUS_VSAFE0V);
	if (ret < 0) {
		logbuffer_log(log,
			      "Error writing TCPC_EXTENDED_STATUS_MASK ret:%d"
			      , ret);
		return;
	}

	logbuffer_log(log, "[%s] Init EXTENDED_STATUS_MASK: VSAFE0V", __func__);

	ret = max77759_write8(regmap, TCPC_ALERT_EXTENDED, 0xff);
	if (ret < 0) {
		logbuffer_log(log, "Unable to clear TCPC_ALERT_EXTENDED ret:%d\n", ret);
		return;
	}

	alert_mask = TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_DISCARDED |
		TCPC_ALERT_TX_FAILED | TCPC_ALERT_RX_HARD_RST |
		TCPC_ALERT_RX_STATUS | TCPC_ALERT_VENDOR | TCPC_ALERT_CC_STATUS |
		TCPC_ALERT_VBUS_DISCNCT | TCPC_ALERT_RX_BUF_OVF |
		TCPC_ALERT_EXTENDED_STATUS | TCPC_ALERT_POWER_STATUS |
		/* Enable Extended alert for detecting Fast Role Swap Signal */
		TCPC_ALERT_EXTND;

	ret = max77759_write16(regmap, TCPC_ALERT_MASK, alert_mask);
	if (ret < 0)
		return;
	logbuffer_log(log, "[%s] Init ALERT_MASK: %u", __func__,
		      alert_mask);

	max77759_read16(regmap, TCPC_ALERT_MASK, &alert_mask);
	logbuffer_log(log, "[%s] Init ALERT_MASK read : %u", __func__,
		      alert_mask);

	/* Enable vbus voltage monitoring, voltage alerts, bleed discharge */
	ret = max77759_update_bits8(regmap, TCPC_POWER_CTRL, TCPC_POWER_CTRL_VBUS_VOLT_MON |
				    TCPC_DIS_VOLT_ALRM | TCPC_POWER_CTRL_BLEED_DISCHARGE,
				    TCPC_POWER_CTRL_BLEED_DISCHARGE);
	if (ret < 0)
		return;
	logbuffer_log(log, "TCPC_POWER_CTRL: Enable voltage monitoring, alarm, bleed discharge");

	ret = max77759_write8(regmap, TCPC_ALERT_EXTENDED_MASK, TCPC_SINK_FAST_ROLE_SWAP);
	if (ret < 0) {
		logbuffer_log(log, "Unable to unmask FAST_ROLE_SWAP interrupt");
		return;
	}

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_VCON_CTRL, VCNILIM_MASK, VCNILIM_300_MA);
	if (ret < 0)
		logbuffer_log(log, "TCPC_VENDOR_VCON_CTRL: update vcnilim to 300mA failed");
}

static void process_rx(struct max77759_plat *chip, u16 status)
{
	struct pd_message msg;
	u8 count, frame_type, rx_buf[TCPC_RECEIVE_BUFFER_LEN];
	int ret, payload_index;
	u8 *rx_buf_ptr;
	struct logbuffer *log = chip->log;
	enum pd_ctrl_msg_type pd_type;

	/*
	 * READABLE_BYTE_COUNT: Indicates the number of bytes in the RX_BUF_BYTE_x registers
	 * plus one (for the RX_BUF_FRAME_TYPE) Table 4-36.
	 * Read the count and frame type.
	 */
	logbuffer_log(log, "%d", __LINE__);
	ret = regmap_raw_read(chip->data.regmap, TCPC_RX_BYTE_CNT, rx_buf, 2);
	logbuffer_log(log, "%d", __LINE__);
	if (ret < 0) {
		dev_err(chip->dev, "TCPC_RX_BYTE_CNT read failed ret:%d", ret);
		return;
	}

	count = rx_buf[TCPC_RECEIVE_BUFFER_COUNT_OFFSET];
	frame_type = rx_buf[TCPC_RECEIVE_BUFFER_FRAME_TYPE_OFFSET];

	if (count == 0 || frame_type != TCPC_RX_BUF_FRAME_TYPE_SOP) {
		max77759_write16(chip->data.regmap, TCPC_ALERT, TCPC_ALERT_RX_STATUS);
		dev_err(chip->dev, "%s", count ==  0 ? "error: count is 0" :
			"error frame_type is not SOP");
		return;
	}

	/*
	 * 1. struct pd_message does not have RX_BUF_FRAME_TYPE.
	 * 2. READABLE_BYTE_COUNT is exclusive of itself.
	 */
	if (count > sizeof(struct pd_message) + 1 || count + 1 > TCPC_RECEIVE_BUFFER_LEN) {
		dev_err(chip->dev, "Invalid TCPC_RX_BYTE_CNT %d", count);
		return;
	}

	/*
	 * Read count + 1 as RX_BUF_BYTE_x is hidden and can only be read through
	 * TCPC_RX_BYTE_CNT
	 */
	count += 1;
	ret = regmap_raw_read(chip->data.regmap, TCPC_RX_BYTE_CNT, rx_buf, count);
	logbuffer_log(log, "%d", __LINE__);
	if (ret < 0) {
		dev_err(chip->dev, "Error: TCPC_RX_BYTE_CNT read failed: %d", ret);
		return;
	}

	rx_buf_ptr = rx_buf + TCPC_RECEIVE_BUFFER_RX_BYTE_BUF_OFFSET;
	msg.header = cpu_to_le16(*(u16 *)rx_buf_ptr);
	rx_buf_ptr = rx_buf_ptr + sizeof(msg.header);
	for (payload_index = 0; payload_index < pd_header_cnt_le(msg.header); payload_index++,
	     rx_buf_ptr += sizeof(msg.payload[0]))
		msg.payload[payload_index] = cpu_to_le32(*(u32 *)rx_buf_ptr);

	logbuffer_log(log, "%d", __LINE__);

	/*
	 * Read complete, clear RX status alert bit.
	 * Clear overflow as well if set.
	 */
	ret = max77759_write16(chip->data.regmap, TCPC_ALERT, status & TCPC_ALERT_RX_BUF_OVF ?
			       TCPC_ALERT_RX_STATUS | TCPC_ALERT_RX_BUF_OVF :
			       TCPC_ALERT_RX_STATUS);
	if (ret < 0)
		return;

	logbuffer_log(log, "rx clear");
	pd_type = pd_header_type_le(msg.header);
	if (pd_type == PD_CTRL_PR_SWAP) {
		logbuffer_log(log, "PD_CTRL_PR_SWAP");
		/* To prevent disconnect during PR_SWAP. */
		ret = max77759_write16(chip->data.regmap, TCPC_VBUS_SINK_DISCONNECT_THRESH, 0);
		/* TODO: tcpci->pr_swap = true; */
		if (ret < 0)
			return;
	}

	tcpm_pd_receive(chip->port, &msg);
}

static void enable_data_path_locked(struct max77759_plat *chip)
{
	int ret;
	bool enable_data = false;
	struct regmap *regmap = chip->data.regmap;

	if (chip->force_device_mode_on) {
		logbuffer_log(chip->log, "%s skipping as force_device_mode_on is set", __func__);
		return;
	}

	logbuffer_log(chip->log,
		      "%s pd_capable:%u pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u debug_acc_conn:%u",
		      __func__, chip->pd_capable ? 1 : 0, chip->pd_data_capable ? 1 : 0,
		      chip->no_bc_12 ? 1 : 0, chip->bc12_data_capable ? 1 : 0, chip->attached ? 1 :
		      0, chip->debug_acc_connected);
	dev_info(chip->dev,
		 "TCPM_DEBUG %s pd_capable:%u pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u debug_acc_conn:%u",
		 __func__, chip->pd_capable ? 1 : 0, chip->pd_data_capable ? 1 : 0, chip->no_bc_12 ?
		 1 : 0, chip->bc12_data_capable ? 1 : 0, chip->attached ? 1 : 0,
		 chip->debug_acc_connected);

	enable_data = (chip->pd_capable && chip->pd_data_capable) || chip->no_bc_12 ||
		chip->bc12_data_capable || chip->data_role == TYPEC_HOST ||
		chip->debug_acc_connected;

	if (chip->attached && enable_data && !chip->data_active) {
		if (chip->data_role == TYPEC_HOST) {
			ret = max77759_write8(regmap, TCPC_VENDOR_USBSW_CTRL, USBSW_CONNECT);
			logbuffer_log(chip->log, "Turning on dp switches %s", ret < 0 ? "fail" :
				      "success");
		}
		ret = extcon_set_state_sync(chip->extcon, chip->data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 1);
		logbuffer_log(chip->log, "%s turning on %s", ret < 0 ? "Failed" : "Succeeded",
			      chip->data_role == TYPEC_HOST ? "Host" : "Device");
		dev_info(chip->dev, "TCPM_DEBUG %s turning on %s", ret < 0 ? "Failed" : "Succeeded",
			 chip->data_role == TYPEC_HOST ? "Host" : "Device");
		chip->data_active = true;
		chip->active_data_role = chip->data_role;
	} else if (chip->data_active && (!chip->attached || !enable_data)) {
		ret = extcon_set_state_sync(chip->extcon, chip->active_data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 0);
		logbuffer_log(chip->log, "%s turning off %s", ret < 0 ? "Failed" : "Succeeded",
			      chip->active_data_role == TYPEC_HOST ? "Host" : "Device");
		dev_info(chip->dev, "TCPM_DEBUG %s turning off %s", ret < 0 ? "Failed" :
			 "Succeeded", chip->active_data_role == TYPEC_HOST ? "Host" : "Device");
		chip->data_active = false;
		if  (chip->active_data_role == TYPEC_HOST) {
			ret = max77759_write8(regmap, TCPC_VENDOR_USBSW_CTRL, USBSW_DISCONNECT);
			logbuffer_log(chip->log, "Turning off dp switches %s", ret < 0 ? "fail" :
				      "success");
		}
	}
}

static int max77759_set_vbus(struct tcpci *tcpci, struct tcpci_data *tdata, bool source, bool sink)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	int ret;
	enum gbms_charger_modes vote = 0xff;

	if (source && sink) {
		logbuffer_log(chip->log, "ERR: both source and sink set. Not voting");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
		chip->charger_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
			logbuffer_log(chip->log, "ERR: GBMS_MODE_VOTABLE lazy get failed",
				      PTR_ERR(chip->charger_mode_votable));
			return 0;
		}
	}

	if (source && !sink) {
		ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
					 (void *)GBMS_USB_OTG_ON, true);
	} else if (sink && !source) {
		ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
					 (void *)GBMS_USB_BUCK_ON, true);
	} else {
		/* just one will do */
		ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
					 (void *)GBMS_USB_BUCK_ON, false);
	}

	if (ret < 0)
		return ret;

	logbuffer_log(chip->log, "%s: GBMS_MODE_VOTABLE voting source:%c sink:%c vote:%u ret:%d",
		      ret < 0 ? "Error" : "Success", source ? 'y' : 'n', sink ? 'y' : 'n',
		      (unsigned int)vote, ret);

	if (source && !chip->sourcing_vbus) {
		chip->sourcing_vbus = 1;
	} else if (!source && chip->sourcing_vbus) {
		chip->sourcing_vbus = 0;
		chip->vbus_present = 0;
		logbuffer_log(chip->log, "[%s]: vbus_present %d", __func__, chip->vbus_present);
		tcpm_vbus_change(tcpci->port);
	}

	return 0;
}

static void max77759_frs_sourcing_vbus(struct tcpci *tcpci, struct tcpci_data *tdata)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	int ret;

	ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
				 (void *)GBMS_USB_OTG_FRS_ON, true);
	logbuffer_log(chip->log, "%s: GBMS_MODE_VOTABLE ret:%d", __func__, ret);

	if (!ret)
		chip->sourcing_vbus = 1;
}

static void process_power_status(struct max77759_plat *chip)
{
	struct tcpci *tcpci = chip->tcpci;
	struct logbuffer *log = chip->log;
	unsigned int pwr_status;
	int ret;

	ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &pwr_status);
	logbuffer_log(log, "TCPC_ALERT_POWER_STATUS status:%d", pwr_status);
	if (ret < 0)
		return;

	if (pwr_status == 0xff) {
		max77759_init_regs(tcpci->regmap, log);
		return;
	}

	if (pwr_status & TCPC_POWER_STATUS_SOURCING_VBUS) {
		chip->sourcing_vbus = 1;
		tcpm_sourcing_vbus(tcpci->port);
	}

	if (pwr_status & TCPC_POWER_STATUS_VBUS_PRES)
		chip->vbus_present = 1;
	else if (!chip->data.auto_discharge_disconnect && !(pwr_status &
							    TCPC_POWER_STATUS_VBUS_PRES))
		chip->vbus_present = 0;
	logbuffer_log(chip->log, "[%s]: vbus_present %d", __func__, chip->vbus_present);
	tcpm_vbus_change(tcpci->port);

	/*
	 * Enable data path when TCPC signals sink debug accesssory connected
	 * and disable when disconnected.
	 */
	if ((!chip->debug_acc_connected && (pwr_status & TCPC_POWER_STATUS_DBG_ACC_CON)) ||
	    (chip->debug_acc_connected && !(pwr_status & TCPC_POWER_STATUS_DBG_ACC_CON))) {
		mutex_lock(&chip->data_path_lock);
		chip->debug_acc_connected = pwr_status & TCPC_POWER_STATUS_DBG_ACC_CON ? 1 : 0;
		chip->data_role = TYPEC_DEVICE;
		chip->attached = chip->debug_acc_connected;
		enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);
		logbuffer_log(log, "Debug accessory %s", chip->debug_acc_connected ? "connected" :
			      "disconnected");
		if (!chip->debug_acc_connected) {
			ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_SBUSW_CTRL,
					      SBUSW_SERIAL_UART);
			logbuffer_log(log, "SBU switch enable %s", ret < 0 ? "fail" : "success");
		}
	}
}

static void process_tx(struct tcpci *tcpci, u16 status, struct logbuffer *log)
{
	if (status & TCPC_ALERT_TX_SUCCESS) {
		logbuffer_log(log, "TCPC_ALERT_TX_SUCCESS");
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_SUCCESS);
	} else if (status & TCPC_ALERT_TX_DISCARDED) {
		logbuffer_log(log, "TCPC_ALERT_TX_DISCARDED");
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_DISCARDED);
	} else if (status & TCPC_ALERT_TX_FAILED) {
		logbuffer_log(log, "TCPC_ALERT_TX_FAILED");
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_FAILED);
	}

	/* Reinit regs as Hard reset sets them to default value */
	if ((status & TCPC_ALERT_TX_SUCCESS) && (status &
						 TCPC_ALERT_TX_FAILED))
		max77759_init_regs(tcpci->regmap, log);
}

static irqreturn_t _max77759_irq(struct max77759_plat *chip, u16 status,
				 struct logbuffer *log)
{
	u16 vendor_status = 0, raw;
	struct tcpci *tcpci = chip->tcpci;
	int ret;
	const u16 mask = status & TCPC_ALERT_RX_BUF_OVF ? status &
		~(TCPC_ALERT_RX_STATUS | TCPC_ALERT_RX_BUF_OVF) :
		status & ~TCPC_ALERT_RX_STATUS;
	u8 reg_status;

	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);
	logbuffer_log(log, "TCPC_ALERT status: %#x", status);
	/**
	 * Clear alert status for everything except RX_STATUS, which shouldn't
	 * be cleared until we have successfully retrieved message.
	 **/
	if (status & ~TCPC_ALERT_RX_STATUS) {
		ret = max77759_write16(tcpci->regmap, TCPC_ALERT, mask);
		if (ret < 0)
			return ret;
	}

	if (status & TCPC_ALERT_RX_BUF_OVF && !(status &
						TCPC_ALERT_RX_STATUS)) {
		logbuffer_log(log, "TCPC_ALERT_RX_BUF_OVF");
		ret = max77759_write16(tcpci->regmap, TCPC_ALERT,
				       (TCPC_ALERT_RX_STATUS |
					TCPC_ALERT_RX_BUF_OVF));
		if (ret < 0)
			return ret;
	}

	if (status & TCPC_ALERT_EXTND) {
		ret = max77759_read8(tcpci->regmap, TCPC_ALERT_EXTENDED, &reg_status);
		if (ret < 0)
			return ret;

		ret = max77759_write8(tcpci->regmap, TCPC_ALERT_EXTENDED, reg_status);
		if (ret < 0)
			return ret;

		if (reg_status & TCPC_SINK_FAST_ROLE_SWAP) {
			logbuffer_log(log, "FRS Signal");
			tcpm_sink_frs(tcpci->port);
		}
	}

	if (status & TCPC_ALERT_RX_STATUS) {
		logbuffer_log(log, "Enter process rx");
		process_rx(chip, status);
	}

	if (status & TCPC_ALERT_TX_DISCARDED)
		logbuffer_log(log, "TX_DISCARDED");

	if (status & TCPC_ALERT_VENDOR) {
		logbuffer_log(log, "TCPC_VENDOR_ALERT Mask");
		ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_ALERT_MASK
				      , 0x0);
		if (ret < 0)
			return ret;

		ret = max77759_write8(tcpci->regmap,
				      TCPC_VENDOR_ALERT_MASK2, 0x1);
		if (ret < 0)
			return ret;

		/* Clear VENDOR_ALERT*/
		ret = max77759_read16(tcpci->regmap, TCPC_VENDOR_ALERT,
				      &vendor_status);
		if (ret < 0)
			return ret;

		process_bc12_alert(chip->bc12, vendor_status);
		ret = max77759_write16(tcpci->regmap, TCPC_VENDOR_ALERT,
				       vendor_status);
		if (ret < 0)
			return ret;
	}

	if (status & TCPC_ALERT_VBUS_DISCNCT) {
		logbuffer_log(log, "TCPC_ALERT_VBUS_DISCNCT");
		chip->vbus_present = 0;
		logbuffer_log(chip->log, "[%s]: vbus_present %d", __func__, chip->vbus_present);
		tcpm_vbus_change(tcpci->port);
		if (chip->force_device_mode_on) {
			ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_USBSW_CTRL, USBSW_CONNECT);
			logbuffer_log(chip->log, "Forcing on dp switches %s", ret < 0 ? "fail" :
				      "success");
		}
	}

	if (status & TCPC_ALERT_CC_STATUS) {
		/**
		 * Process generic CC updates if it doesn't belong to
		 * contaminant detection.
		 */
		mutex_lock(&chip->contaminant_detection_lock);
		if (!chip->contaminant_detection || !tcpm_is_toggling(tcpci->port) ||
		    !process_contaminant_alert(chip->contaminant, false, true))
			tcpm_cc_change(tcpci->port);
		else
			logbuffer_log(log, "CC update: Contaminant algorithm responded");
		mutex_unlock(&chip->contaminant_detection_lock);
	}

	if (status & TCPC_ALERT_POWER_STATUS)
		process_power_status(chip);

	if (status & TCPC_ALERT_V_ALARM_LO) {
		ret = max77759_read16(tcpci->regmap,
				      TCPC_VBUS_VOLTAGE_ALARM_LO_CFG,
				      &raw);
		if (ret < 0)
			return ret;

		logbuffer_log(log, "VBUS LOW ALARM triggered: %u", (raw &
			      TCPC_VBUS_VOLTAGE_MASK) *
			      TCPC_VBUS_VOLTAGE_LSB_MV);
	}

	if (status & TCPC_ALERT_V_ALARM_HI) {
		ret = max77759_read16(tcpci->regmap,
				      TCPC_VBUS_VOLTAGE_ALARM_HI_CFG, &raw)
			;
		if (ret < 0)
			return ret;

		logbuffer_log(log, "VBUS HIGH ALARM triggered: %u", (raw &
			      TCPC_VBUS_VOLTAGE_MASK) *
			      TCPC_VBUS_VOLTAGE_LSB_MV);
	}

	if (status & TCPC_ALERT_RX_HARD_RST) {
		logbuffer_log(log, "TCPC_ALERT_RX_HARD_RST");
		/* To prevent disconnect during hardreset. */
		ret = max77759_write16(tcpci->regmap,
				       TCPC_VBUS_SINK_DISCONNECT_THRESH,
				       0);
		if (ret < 0)
			return ret;

		tcpm_pd_hard_reset(tcpci->port);
		max77759_init_regs(tcpci->regmap, log);
	}

	if (status & TCPC_ALERT_TX_SUCCESS || status &
	    TCPC_ALERT_TX_DISCARDED || status & TCPC_ALERT_TX_FAILED)
		process_tx(tcpci, status, log);

	if (status & TCPC_ALERT_VENDOR) {
		logbuffer_log(log, "Exit TCPC_VENDOR_ALERT Unmask");
		ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_ALERT_MASK
				      , 0xff);
		if (ret < 0)
			return ret;
		ret = max77759_write8(tcpci->regmap,
				      TCPC_VENDOR_ALERT_MASK2, 0xfe);
		if (ret < 0)
			return ret;
	}

	if (status & TCPC_ALERT_EXTENDED_STATUS) {
		ret = max77759_read8(tcpci->regmap, TCPC_EXTENDED_STATUS,
				     (u8 *)&raw);
		if (ret < 0)
			return ret;

		logbuffer_log(log, "VSAFE0V: %c\n", raw & TCPC_EXTENDED_STATUS_VSAFE0V ? 'Y' :
			      'N');
		if (raw & TCPC_EXTENDED_STATUS_VSAFE0V) {
			chip->vbus_present = 0;
			logbuffer_log(chip->log, "[%s]: vbus_present %d", __func__, chip->vbus_present);
			tcpm_vbus_change(tcpci->port);
		}
	}

	logbuffer_log(log, "TCPC_ALERT status done: %#x", status);

	return IRQ_HANDLED;
}

static irqreturn_t max77759_irq(int irq, void *dev_id)
{
	struct max77759_plat *chip = dev_id;
	u16 status;
	irqreturn_t irq_return;
	int ret;

	logbuffer_log(chip->log, "TCPC_ALERT threaded irq running ");
	if (!chip->tcpci)
		return IRQ_HANDLED;

	ret = max77759_read16(chip->tcpci->regmap, TCPC_ALERT, &status);
	if (ret < 0)
		return ret;
	while (status) {
		irq_return = _max77759_irq(chip, status, chip->log);
		/* Do not return if the ALERT is already set. */
		logbuffer_log(chip->log, "TCPC_ALERT read alert status");
		ret = max77759_read16(chip->tcpci->regmap, TCPC_ALERT, &status);
		if (ret < 0)
			break;
		logbuffer_log(chip->log, "TCPC_ALERT status pending: %#x",
			      status);
	}

	return irq_return;
}

static irqreturn_t max77759_isr(int irq, void *dev_id)
{
	struct max77759_plat *chip = dev_id;

	logbuffer_log(chip->log, "TCPC_ALERT triggered ");
	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);

	if (!chip->tcpci)
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static int max77759_init_alert(struct max77759_plat *chip,
			       struct i2c_client *client)
{
	int ret, irq_gpio;

	irq_gpio = of_get_named_gpio(client->dev.of_node, "usbpd,usbpd_int", 0);
	client->irq = gpio_to_irq(irq_gpio);
	if (!client->irq)
		return -ENODEV;

	ret = devm_request_threaded_irq(chip->dev, client->irq, max77759_isr,
					max77759_irq,
					(IRQF_TRIGGER_LOW | IRQF_ONESHOT),
					dev_name(chip->dev), chip);

	if (ret < 0)
		return ret;

	enable_irq_wake(client->irq);
	return 0;
}

static int max77759_start_toggling(struct tcpci *tcpci,
				   struct tcpci_data *tdata,
				   enum typec_cc_status cc)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	unsigned int reg = TCPC_ROLE_CTRL_DRP;
	int ret;

	switch (cc) {
	case TYPEC_CC_RP_DEF:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_DEF <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_1_5:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_1_5 <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_3_0:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_3_0 <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	default:
		break;
	}

	if (cc == TYPEC_CC_RD)
		reg |= (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
	else
		reg |= (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT);

	max77759_init_regs(chip->tcpci->regmap, chip->log);

	/* Kick debug accessory state machine when enabling toggling for the first time */
	if (chip->first_toggle) {
		gpio_set_value_cansleep(chip->in_switch_gpio, 0);
		mdelay(10);
		gpio_set_value_cansleep(chip->in_switch_gpio, 1);
		chip->first_toggle = false;
	}
	mutex_lock(&chip->contaminant_detection_lock);
	if (chip->contaminant_detection) {
		ret = enable_contaminant_detection(chip, chip->contaminant_detection ==
						   CONTAMINANT_DETECT_MAXQ);
	} else {
		ret = max77759_write8(tcpci->regmap, TCPC_ROLE_CTRL, reg);
		if (ret < 0)
			goto error;

		ret = max77759_update_bits8(tcpci->regmap, TCPC_TCPC_CTRL,
					    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
					    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);

		if (ret < 0)
			goto error;

		ret = regmap_write(tcpci->regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	}
error:
	mutex_unlock(&chip->contaminant_detection_lock);

	return ret;
}

static int max77759_get_vbus_voltage_mv(struct i2c_client *tcpc_client)
{
	u16 raw;
	int ret;
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	if (!chip || !chip->tcpci || !chip->tcpci->regmap)
		return -EAGAIN;

	/* TCPC_POWER_CTRL_VBUS_VOLT_MON enabled in init_regs */
	ret = max77759_read16(chip->tcpci->regmap, TCPC_VBUS_VOLTAGE,
			      &raw);
	return ret ? 0 : ((raw & TCPC_VBUS_VOLTAGE_MASK) *
		TCPC_VBUS_VOLTAGE_LSB_MV);
}

static int max77759_check_contaminant(struct tcpci *tcpci, struct tcpci_data *tdata)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	int ret;

	logbuffer_log(chip->log, "%s: debounce path", __func__);
	mutex_lock(&chip->contaminant_detection_lock);
	if (chip->contaminant_detection) {
		process_contaminant_alert(chip->contaminant, true, false);
		ret = CONTAMINANT_HANDLES_TOGGLING;
	} else {
		ret = TCPM_RESTART_TOGGLING;
	}

	mutex_unlock(&chip->contaminant_detection_lock);
	return ret;
}

static int max77759_get_current_limit(struct tcpci *tcpci,
				      struct tcpci_data *tdata)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	void *vote;
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->usb_icl_proto_el))
		return 0;

	ret = gvotable_get_vote(chip->usb_icl_proto_el,
				proto_voter_reason[USB_ICL_PD], &vote);
	if (ret < 0)
		logbuffer_log(chip->log,
			      "icl_proto_el get vote failed:%d", ret);
	else
		ret = ((struct usb_vote *)(vote))->val;

	return ret;
}

static void max77759_set_pd_capable(struct tcpci *tcpci, struct tcpci_data
				     *data, bool capable)
{
	struct max77759_plat *chip = tdata_to_max77759(data);

	mutex_lock(&chip->data_path_lock);
	chip->pd_capable = capable;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);
}

static int max77759_enable_frs(struct tcpci *tcpci, struct tcpci_data *data, bool enable)
{
	struct max77759_plat *chip = tdata_to_max77759(data);

	return !chip->frs ? -1 : 1;
}

static void max77759_set_cc_polarity(struct tcpci *tcpci, struct tcpci_data *data,
				     enum typec_cc_polarity polarity)
{
	struct max77759_plat *chip = tdata_to_max77759(data);
	int ret;

	ret = extcon_set_property(chip->extcon, EXTCON_USB, EXTCON_PROP_USB_TYPEC_POLARITY,
				  (union extcon_property_value)(int)polarity);
	logbuffer_log(chip->log, "%s setting polarity USB %d", ret < 0 ? "Failed" : "Succeeded",
		      polarity);
	dev_info(chip->dev, "TCPM_DEBUG %s setting polarity USB %d", ret < 0 ? "Failed" :
		 "Succeeded", polarity);

	ret = extcon_set_property(chip->extcon, EXTCON_USB_HOST, EXTCON_PROP_USB_TYPEC_POLARITY,
				  (union extcon_property_value)(int)polarity);
	logbuffer_log(chip->log, "%s setting polarity USB_HOST %d", ret < 0 ?
		      "Failed" : "Succeeded", polarity);
	dev_info(chip->dev, "TCPM_DEBUG %s setting polarity USB %d", ret < 0 ? "Failed" :
		 "Succeeded", polarity);
}

static int max77759_vote_icl(struct tcpci *tcpci, struct tcpci_data *tdata,
			     u32 max_ma)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
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

static int max77759_set_current_limit(struct tcpci *tcpci,
				      struct tcpci_data *tdata,
				      u32 max_ma, u32 mv)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	union power_supply_propval val = {0};
	int ret = 0;

	chip->vbus_mv = mv;
	ret = power_supply_set_property(chip->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
	if (ret < 0) {
		logbuffer_log(chip->log, "unable to set max voltage to %d, ret=%d", mv, ret);
		return ret;
	}
	logbuffer_log(chip->log, "max_ma=%d, mv=%d", max_ma, mv);
	max77759_vote_icl(tcpci, tdata, max_ma);

	return ret;
}

static int max77759_get_vbus_voltage_max_mv(struct i2c_client *tcpc_client)
{
	u16 raw;
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);
	int ret;

	if (!chip || !chip->tcpci || !chip->tcpci->regmap || !chip->set_voltage_alarm)
		return chip->vbus_mv;

	ret = max77759_read16(chip->tcpci->regmap,
			      TCPC_VBUS_VOLTAGE_ALARM_HI_CFG, &raw);
	if (ret < 0)
		return chip->vbus_mv;

	return raw * TCPC_VBUS_VOLTAGE_ALARM_HI_CFG - VBUS_HI_HEADROOM_MV;
}

static int max77759_set_vbus_voltage_max_mv(struct i2c_client *tcpc_client,
					    unsigned int mv)
{
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	chip->vbus_mv = mv;
	if (!chip->set_voltage_alarm)
		return 0;

	/* Set voltage alarm */
	max77759_update_bits16(chip->tcpci->regmap,
			       TCPC_VBUS_VOLTAGE_ALARM_HI_CFG,
			       TCPC_VBUS_VOLTAGE_MASK,
			       (mv + VBUS_HI_HEADROOM_MV) /
			       TCPC_VBUS_VOLTAGE_LSB_MV);
	max77759_update_bits16(chip->tcpci->regmap,
			       TCPC_VBUS_VOLTAGE_ALARM_LO_CFG,
			       TCPC_VBUS_VOLTAGE_MASK,
			       VBUS_LO_MV / TCPC_VBUS_VOLTAGE_LSB_MV);
	max77759_update_bits8(chip->tcpci->regmap, TCPC_POWER_CTRL,
			      TCPC_DIS_VOLT_ALRM, (u8)~TCPC_DIS_VOLT_ALRM);
	max77759_update_bits16(chip->tcpci->regmap, TCPC_ALERT_MASK,
			       TCPC_ALERT_V_ALARM_LO | TCPC_ALERT_V_ALARM_HI,
			       TCPC_ALERT_V_ALARM_LO | TCPC_ALERT_V_ALARM_HI);
	return 0;
}

static int max77759_get_vbus(struct tcpci *tcpci, struct tcpci_data *data)
{
	struct max77759_plat *chip = tdata_to_max77759(data);
	u8 pwr_status;
	int ret;

	ret = max77759_read8(tcpci->regmap, TCPC_POWER_STATUS, &pwr_status);
	if (!ret && !chip->vbus_present && (pwr_status & TCPC_POWER_STATUS_VBUS_PRES)) {
		logbuffer_log(chip->log, "[%s]: syncing vbus_present", __func__);
		chip->vbus_present = 1;
	}

	logbuffer_log(chip->log, "[%s]: vbus_present %d", __func__, chip->vbus_present);
	return chip->vbus_present;
}

/* Notifier structure inferred from usbpd-manager.c */
static int max77759_set_roles(struct tcpci *tcpci, struct tcpci_data *data,
			      bool attached, enum typec_role role,
			      enum typec_data_role data_role,
			      bool usb_comm_capable)
{
	struct max77759_plat *chip = tdata_to_max77759(data);
	int ret;
	bool enable_data;

	mutex_lock(&chip->data_path_lock);
	chip->pd_data_capable = usb_comm_capable;

	enable_data = (chip->pd_capable && chip->pd_data_capable) || chip->no_bc_12 ||
		chip->bc12_data_capable || chip->data_role == TYPEC_HOST ||
		chip->debug_acc_connected;

	if (!chip->force_device_mode_on && chip->data_active &&
	    (chip->active_data_role != data_role || !attached || !enable_data)) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->active_data_role ==
					    TYPEC_HOST ? EXTCON_USB_HOST :
					    EXTCON_USB, 0);

		logbuffer_log(chip->log, "%s turning off %s", ret < 0 ?
			      "Failed" : "Succeeded",
			      chip->active_data_role == TYPEC_HOST ? "Host"
			      : "Device");
		chip->data_active = false;

		if  (chip->active_data_role == TYPEC_HOST) {
			ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_USBSW_CTRL,
					      USBSW_DISCONNECT);
			logbuffer_log(chip->log, "Turning off dp switches %s", ret < 0 ? "fail" :
				      "success");
		}
	}

	/*
	 * To prevent data stack enumeration failure, previously there
	 * was a 300msec delay here
	 */

	chip->attached = attached;
	chip->data_role = data_role;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);

	if (!chip->attached)
		max77759_vote_icl(tcpci, data, 0);

	/* Signal usb_psy online */
	usb_psy_set_sink_state(chip->usb_psy_data, role == TYPEC_SINK && attached);

	return 0;
}

static void max77759_set_port_data_capable(struct i2c_client *tcpc_client,
					   enum power_supply_usb_type
					   usb_type)
{
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	switch (usb_type) {
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_CDP:
		mutex_lock(&chip->data_path_lock);
		chip->bc12_data_capable = true;
		enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);
		break;
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
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

static int tcpci_init(struct tcpci *tcpci, struct tcpci_data *data)
{
	/*
	 * Generic TCPCI overwrites the regs once this driver initializes
	 * them. Prevent this by returning -1.
	 */
	return -1;
}

#ifdef CONFIG_DEBUG_FS
static ssize_t force_device_mode_on_write(struct file *file, const char __user *ubuf, size_t count,
					  loff_t *ppos)
{
	struct max77759_plat *chip = file->private_data;
	long result, ret;

	ret = kstrtol_from_user(ubuf, count, 10, &result);
	if (ret)
		return ret;

	if (result == chip->force_device_mode_on)
		return count;

	mutex_lock(&chip->data_path_lock);
	chip->force_device_mode_on = result;
	/* Tear down previous data role if needed */
	if (((result && chip->active_data_role != TYPEC_DEVICE) ||
	    (!result && chip->active_data_role != chip->data_role)) && chip->data_active) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->active_data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 0);

		logbuffer_log(chip->log, "%s: %s turning off %s", __func__, ret < 0 ?
			      "Failed" : "Succeeded", chip->active_data_role == TYPEC_HOST ?
			      "Host" : "Device");
		chip->data_active = false;
	}

	if (result && !chip->data_active) {
		ret = extcon_set_state_sync(chip->extcon, EXTCON_USB, 1);
		logbuffer_log(chip->log, "%s: %s turning on device", __func__, ret < 0 ? "Failed" :
			      "Succeeded");
		chip->data_active = !ret;
		chip->active_data_role = TYPEC_DEVICE;

	} else if (!result) {
		enable_data_path_locked(chip);
	}

	mutex_unlock(&chip->data_path_lock);
	return count;
}

static ssize_t force_device_mode_on_read(struct file *file, char __user *userbuf, size_t count,
					 loff_t *ppos)
{
	struct max77759_plat *chip = file->private_data;
	char buf[16];
	int ret;

	ret = snprintf(buf, sizeof(buf) - 1, "%d\n", chip->force_device_mode_on);

	return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}

static const struct file_operations force_device_mode_on_fops = {
	.read	= force_device_mode_on_read,
	.write	= force_device_mode_on_write,
	.open	= simple_open,
	.llseek = default_llseek,
};
#endif

static int max77759_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	int ret, i;
	struct max77759_plat *chip;
	char *usb_psy_name;
	struct device_node *dn;
	u8 power_status;
	u16 device_id;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->data.regmap = devm_regmap_init_i2c(client,
						 &max77759_regmap_config);
	if (IS_ERR(chip->data.regmap)) {
		dev_err(&client->dev, "Regmap init failed\n");
		return PTR_ERR(chip->data.regmap);
	}

	chip->charger_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
	if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
		dev_err(&client->dev, "TCPCI: GBMS_MODE_VOTABLE get failed",
			PTR_ERR(chip->charger_mode_votable));
		return -EPROBE_DEFER;
	}

	dn = dev_of_node(&client->dev);
	if (!dn) {
		dev_err(&client->dev, "of node not found\n");
		return -EINVAL;
	}

	chip->in_switch_gpio = of_get_named_gpio(dn, "in-switch-gpio", 0);
	if (chip->in_switch_gpio < 0) {
		dev_err(&client->dev, "in-switch-gpio not found\n");
		return -EPROBE_DEFER;
	}

	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->icl_proto_el_lock);
	mutex_init(&chip->data_path_lock);
	mutex_init(&chip->contaminant_detection_lock);
	chip->first_toggle = true;

	ret = max77759_read8(chip->data.regmap, TCPC_POWER_STATUS,
			     &power_status);
	if (ret < 0)
		return ret;

	if (power_status & TCPC_POWER_STATUS_UNINIT) {
		dev_err(&client->dev, "TCPC not ready!");
		return -EPROBE_DEFER;
	}

	/* Chip level tcpci callbacks */
	chip->data.set_vbus = max77759_set_vbus;
	chip->data.get_vbus = max77759_get_vbus;
	chip->data.start_drp_toggling = max77759_start_toggling;
	chip->data.get_current_limit = max77759_get_current_limit;
	chip->data.set_current_limit = max77759_set_current_limit;
	chip->data.TX_BUF_BYTE_x_hidden = 1;
	chip->data.override_toggling = true;
	chip->data.vbus_vsafe0v = true;
	chip->data.set_pd_capable = max77759_set_pd_capable;
	chip->data.set_roles = max77759_set_roles;
	chip->data.init = tcpci_init;
	chip->data.set_cc_polarity = max77759_set_cc_polarity;
	chip->data.frs_sourcing_vbus = max77759_frs_sourcing_vbus;
	chip->data.enable_frs = max77759_enable_frs;
	chip->data.check_contaminant = max77759_check_contaminant;

	chip->log = logbuffer_register("usbpd");
	if (IS_ERR_OR_NULL(chip->log)) {
		dev_err(&client->dev, "logbuffer get failed");
		chip->log = NULL;
	}

	chip->psy_ops.tcpc_get_vbus_voltage_mv =
		max77759_get_vbus_voltage_mv;
	chip->psy_ops.tcpc_get_vbus_voltage_max_mv =
		max77759_get_vbus_voltage_max_mv;
	chip->psy_ops.tcpc_set_vbus_voltage_max_mv =
		max77759_set_vbus_voltage_max_mv;
	chip->psy_ops.tcpc_set_port_data_capable =
		max77759_set_port_data_capable;
	chip->usb_psy_data = usb_psy_setup(client, chip->log,
					   &chip->psy_ops);
	if (IS_ERR_OR_NULL(chip->usb_psy_data)) {
		dev_err(&client->dev, "USB psy failed to initialize");
		ret = PTR_ERR(chip->usb_psy_data);
		goto logbuffer_unreg;
	}

	/* Defered probe returned until usb power supply showup.*/
	chip->bc12 = bc12_init(chip);
	if (IS_ERR_OR_NULL(chip->bc12)) {
		ret = PTR_ERR(chip->bc12);
		goto unreg_psy;
	}

	usb_psy_name = (char *)of_get_property(dn, "usb-psy-name", NULL);
	if (!usb_psy_name) {
		dev_err(&client->dev, "usb-psy-name not set\n");
		ret = -EINVAL;
		goto teardown_bc12;
	}

	chip->no_bc_12 = of_property_read_bool(dn, "no-bc-12");

	chip->usb_psy = power_supply_get_by_name(usb_psy_name);
	if (IS_ERR_OR_NULL(chip->usb_psy) || !chip->usb_psy) {
		dev_err(&client->dev, "usb psy not up\n");
		ret = -EPROBE_DEFER;
		goto teardown_bc12;
	}

	ret = max77759_read16(chip->data.regmap, TCPC_BCD_DEV, &device_id);
	if (ret < 0)
		goto teardown_bc12;

	logbuffer_log(chip->log, "TCPC DEVICE id:%d", device_id);
	/* Default enable on A1 or higher */
	chip->contaminant_detection = device_id >= MAX77759_DEVICE_ID_A1;
	chip->contaminant = max77759_contaminant_init(chip, chip->contaminant_detection);

	chip->extcon = devm_extcon_dev_allocate(&client->dev,
						usbpd_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		dev_err(&client->dev, "Error allocating extcon: %d\n",
			PTR_ERR(chip->extcon));
		ret = PTR_ERR(chip->extcon);
		goto psy_put;
	}

	ret = devm_extcon_dev_register(&client->dev, chip->extcon);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register extcon device");
		goto psy_put;
	}

	extcon_set_property_capability(chip->extcon, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_TYPEC_POLARITY);

	max77759_init_regs(chip->data.regmap, chip->log);
	chip->tcpci = tcpci_register_port(chip->dev, &chip->data);
	if (IS_ERR_OR_NULL(chip->tcpci)) {
		dev_err(&client->dev, "TCPCI port registration failed");
		ret = PTR_ERR(chip->tcpci);
		goto psy_put;
	}
	chip->port = tcpci_get_tcpm_port(chip->tcpci);

	/* Default enable on A1 or higher */
	if (device_id >= MAX77759_DEVICE_ID_A1) {
		chip->data.auto_discharge_disconnect = true;
		chip->frs = true;
		tcpci_auto_discharge_update(chip->tcpci);
	}

	ret = max77759_init_alert(chip, client);
	if (ret < 0)
		goto unreg_port;

	chip->usb_icl_proto_el = gvotable_election_get_handle(USB_ICL_PROTO_EL);
	if (IS_ERR_OR_NULL(chip->usb_icl_proto_el)) {
		dev_err(&client->dev, "TCPCI: USB ICL PROTO EL get failed:%d",
			PTR_ERR(chip->usb_icl_proto_el));
		ret = -ENODEV;
		goto unreg_port;
	}

	device_init_wakeup(chip->dev, true);

	for (i = 0; max77759_device_attrs[i]; i++) {
		ret = device_create_file(&client->dev, max77759_device_attrs[i]);
		if (ret < 0)
			dev_err(&client->dev, "TCPCI: Unable to create device attr[%d] ret:%d:", i,
				ret);
	}

#ifdef CONFIG_DEBUG_FS
	chip->dentry = debugfs_create_dir("tcpci_max77759", NULL);
	if (IS_ERR(chip->dentry)) {
		dev_err(&client->dev, "TCPCI: debugfs dentry failed: %d", PTR_ERR(chip->dentry));
	} else {
		debugfs_create_file("force_device_mode_on", 0644, chip->dentry, chip,
				    &force_device_mode_on_fops);
	}
#endif

#ifdef CONFIG_GPIOLIB
	ret = ext_bst_en_gpio_init(chip);
	if (ret)
		goto remove_files;
#endif
	return 0;

remove_files:
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(chip->dentry);
#endif
	for (i = 0; max77759_device_attrs[i]; i++)
		device_remove_file(&client->dev, max77759_device_attrs[i]);
unreg_port:
	tcpci_unregister_port(chip->tcpci);
psy_put:
	power_supply_put(chip->usb_psy);
teardown_bc12:
	bc12_teardown(chip->bc12);
unreg_psy:
	usb_psy_teardown(chip->usb_psy_data);
logbuffer_unreg:
	logbuffer_unregister(chip->log);

	return ret;
}

static int max77759_remove(struct i2c_client *client)
{
	struct max77759_plat *chip = i2c_get_clientdata(client);
	int i;

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(chip->dentry);
#endif
	for (i = 0; max77759_device_attrs[i]; i++)
		device_remove_file(&client->dev, max77759_device_attrs[i]);
	if (!IS_ERR_OR_NULL(chip->tcpci))
		tcpci_unregister_port(chip->tcpci);
	if (!IS_ERR_OR_NULL(chip->usb_psy))
		power_supply_put(chip->usb_psy);
	if (!IS_ERR_OR_NULL(chip->usb_psy_data))
		usb_psy_teardown(chip->usb_psy_data);
	if (!IS_ERR_OR_NULL(chip->bc12))
		bc12_teardown(chip->bc12);
	if (!IS_ERR_OR_NULL(chip->log))
		logbuffer_unregister(chip->log);
	if (!IS_ERR_OR_NULL(chip->extcon))
		devm_extcon_dev_free(chip->dev, chip->extcon);

	return 0;
}

static const struct i2c_device_id max77759_id[] = {
	{ "max77759tcpc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77759_id);

#ifdef CONFIG_OF
static const struct of_device_id max77759_of_match[] = {
	{ .compatible = "max77759tcpc", },
	{},
};
MODULE_DEVICE_TABLE(of, max77759_of_match);
#endif

static struct i2c_driver max77759_i2c_driver = {
	.driver = {
		.name = "max77759tcpc",
		.of_match_table = of_match_ptr(max77759_of_match),
	},
	.probe = max77759_probe,
	.remove = max77759_remove,
	.id_table = max77759_id,
};
module_i2c_driver(max77759_i2c_driver);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("MAX77759 USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL");
