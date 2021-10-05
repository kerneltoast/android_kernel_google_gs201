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
#include <linux/spinlock.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <misc/logbuffer.h>
#include <trace/hooks/typec.h>

#include "bc_max77759.h"
#include "max77759_export.h"
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

#define TCPCI_HI_Z_CC		0xf
/*
 * LongMessage not supported, hence 32 bytes for buf to be read from RECEIVE_BUFFER.
 * DEVICE_CAPABILITIES_2.LongMessage = 0, the value in READABLE_BYTE_COUNT reg shall be
 * less than or equal to 31. Since, RECEIVE_BUFFER len = 31 + 1(READABLE_BYTE_COUNT).
 */
#define TCPC_RECEIVE_BUFFER_LEN                         32

#define PD_ACTIVITY_TIMEOUT_MS				10000

#define GBMS_MODE_VOTABLE "CHARGER_MODE"

#define MAX77759_DEVICE_ID_A1				0x2

#define MAX77759_DISABLE_TOGGLE				1
#define MAX77759_ENABLE_TOGGLE				0
/* Vote value doesn't matter. Only status matters. */
#define MAX77759_DISABLE_TOGGLE_VOTE			1

/* system use cases */
enum gbms_charger_modes {
	GBMS_USB_BUCK_ON	= 0x30,
	GBMS_USB_OTG_ON		= 0x31,
	GBMS_USB_OTG_FRS_ON	= 0x32,
};

#define CONTAMINANT_DETECT_DISABLE	0
#define CONTAMINANT_DETECT_AP		1
#define CONTAMINANT_DETECT_MAXQ		2

#define TCPM_RESTART_TOGGLING		0
#define CONTAMINANT_HANDLES_TOGGLING	1

#define VOLTAGE_ALARM_HI_EN_MV		3000
#define VOLTAGE_ALARM_HI_DIS_MV		21000
#define VOLTAGE_ALARM_LOW_EN_MV		1500
#define VOLTAGE_ALARM_LOW_DIS_MV	0

#define FLOATING_CABLE_INSTANCE_THRESHOLD	5
#define AUTO_ULTRA_LOW_POWER_MODE_REENABLE_MS	600000

#define REGMAP_REG_MAX_ADDR			0x95
#define REGMAP_REG_COUNT			(REGMAP_REG_MAX_ADDR + 1)

static struct logbuffer *tcpm_log;

static bool modparam_conf_sbu;
module_param_named(conf_sbu, modparam_conf_sbu, bool, 0644);
MODULE_PARM_DESC(conf_sbu, "Configure sbu pins");

static bool hooks_installed;

static u32 partner_src_caps[PDO_MAX_OBJECTS];
static unsigned int nr_partner_src_caps;
spinlock_t g_caps_lock;

static unsigned int sink_discovery_delay_ms;

struct tcpci {
	struct device *dev;
	struct tcpm_port *port;
	struct regmap *regmap;
	bool controls_vbus;

	struct tcpc_dev tcpc;
	struct tcpci_data *data;
};

static const struct regmap_range max77759_tcpci_range[] = {
	regmap_reg_range(0x00, REGMAP_REG_MAX_ADDR)
};

const struct regmap_access_table max77759_tcpci_write_table = {
	.yes_ranges = max77759_tcpci_range,
	.n_yes_ranges = ARRAY_SIZE(max77759_tcpci_range),
};

static const struct regmap_config max77759_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REGMAP_REG_MAX_ADDR,
	.wr_table = &max77759_tcpci_write_table,
};

static ssize_t frs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->frs);
};
static DEVICE_ATTR_RO(frs);

static ssize_t auto_discharge_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->data.auto_discharge_disconnect ? 1 : 0);
};
static DEVICE_ATTR_RO(auto_discharge);

static ssize_t bc12_enabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", bc12_get_status(chip->bc12) ? 1 : 0);
};
static DEVICE_ATTR_RO(bc12_enabled);

/* Debugfs disabled in user builds. */
static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	struct regmap *regmap = chip->data.regmap;
	u8 dump[REGMAP_REG_COUNT];
	int ret, offset = 0, addr;

	ret = regmap_bulk_read(regmap, 0, dump, REGMAP_REG_COUNT);
	if (ret < 0) {
		dev_err(chip->dev, "[%s]: Failed to dump ret:%d\n", __func__, ret);
		return 0;
	}

	for (addr = 0; addr < REGMAP_REG_COUNT; addr++) {
		ret = sysfs_emit_at(buf, offset, "%x: %x\n", addr, dump[addr]);
		if (!ret) {
			dev_err(chip->dev, "[%s]: Not all registers printed. last:%x\n", __func__,
				addr - 1);
			break;
		}
		offset += ret;
	}

	return offset;
};
static DEVICE_ATTR_RO(registers);

static ssize_t contaminant_detection_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->contaminant_detection_userspace);
};

static void update_contaminant_detection_locked(struct max77759_plat *chip, int val)
{

	chip->contaminant_detection = val;

	if (chip->contaminant_detection)
		enable_contaminant_detection(chip, chip->contaminant_detection ==
					     CONTAMINANT_DETECT_MAXQ);
	else
		disable_contaminant_detection(chip);

	logbuffer_log(chip->log, "[%s]: %d", __func__, chip->contaminant_detection);
}

static ssize_t contaminant_detection_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	int val;

	if (kstrtoint(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&chip->rc_lock);
	chip->contaminant_detection_userspace = val;
	update_contaminant_detection_locked(chip, val);
	mutex_unlock(&chip->rc_lock);
	return count;
}
static DEVICE_ATTR_RW(contaminant_detection);

static ssize_t cc_toggle_enable_show(struct device *dev, struct device_attribute *attr,
				     char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->toggle_disable_status ? 0 : 1);
};

static ssize_t cc_toggle_enable_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	int val, ret;

	if (kstrtoint(buf, 10, &val) < 0)
		return -EINVAL;

	ret = gvotable_cast_vote(chip->toggle_disable_votable, "USER_VOTE",
				 (void *)MAX77759_DISABLE_TOGGLE_VOTE, val ?
				 MAX77759_ENABLE_TOGGLE : MAX77759_DISABLE_TOGGLE);
	if (ret < 0)
		dev_err(chip->dev, "Cannot set TOGGLE DISABLE=%d (%d)\n", val, ret);

	return count;
}
static DEVICE_ATTR_RW(cc_toggle_enable);

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
	&dev_attr_bc12_enabled,
	&dev_attr_registers,
	&dev_attr_auto_discharge,
	&dev_attr_contaminant_detection,
	&dev_attr_contaminant_detection_status,
	&dev_attr_cc_toggle_enable,
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

static void enable_dp_pulse(struct max77759_plat *chip)
{
	struct regmap *regmap = chip->data.regmap;
	int ret;

	ret = max77759_update_bits8(regmap, VENDOR_BC_CTRL2, DPDNMAN | DPDRV,
				    DPDNMAN | DPDRV_3V0 << DPDRV_SHIFT);
	if (ret < 0)
		logbuffer_log(chip->log, "%s failed to set dpDnMan and dpDrv", __func__);

	mdelay(100);

	ret = max77759_update_bits8(regmap, VENDOR_BC_CTRL2, DPDNMAN | DPDRV,
				    DPDRV_OPEN << DPDRV_SHIFT);
	if (ret < 0)
		logbuffer_log(chip->log, "%s failed to disable dpDnMan and dpDrv", __func__);
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
		      "%s pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u debug_acc_conn:%u bc12_running:%u",
		      __func__, chip->pd_data_capable ? 1 : 0, chip->no_bc_12 ? 1 : 0,
		      chip->bc12_data_capable ? 1 : 0, chip->attached ? 1 : 0,
		      chip->debug_acc_connected, chip->bc12_running ? 1 : 0);
	dev_info(chip->dev,
		 "TCPM_DEBUG %s pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u debug_acc_conn:%u bc12_running:%u",
		 __func__, chip->pd_data_capable ? 1 : 0, chip->no_bc_12 ? 1 : 0,
		 chip->bc12_data_capable ? 1 : 0, chip->attached ? 1 : 0,
		 chip->debug_acc_connected, chip->bc12_running ? 1 : 0);

	enable_data = ((chip->pd_data_capable || chip->no_bc_12 || chip->bc12_data_capable ||
		       chip->debug_acc_connected) && !chip->bc12_running) ||
		       chip->data_role == TYPEC_HOST;

	if (chip->attached && enable_data && !chip->data_active) {
		/* Disable BC1.2 to prevent BC1.2 detection during PR_SWAP */
		bc12_enable(chip->bc12, false);
		/*
		 * Clear running flag here as PD might have configured data
		 * before BC12 started to run.
		 */
		chip->bc12_running = false;

		/*
		 * b/188614064: While swapping from host to device switches will not be configured
		 * by HW. So always enable the switches here.
		 */
		ret = max77759_write8(regmap, TCPC_VENDOR_USBSW_CTRL, USBSW_CONNECT);
		logbuffer_log(chip->log, "Turning on dp switches %s", ret < 0 ? "fail" :
			      "success");

		if (get_usb_type(chip->bc12) == POWER_SUPPLY_USB_TYPE_CDP &&
		    !chip->pd_data_capable) {
			logbuffer_log(chip->log, "CDP detected, gen dp pulse");
			enable_dp_pulse(chip);
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

static void enable_vbus_work(struct kthread_work *work)
{
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, enable_vbus_work);
	enum gbms_charger_modes vote = 0xff;
	int ret;

	if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
		chip->charger_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
			logbuffer_log(chip->log, "ERR: GBMS_MODE_VOTABLE lazy get failed",
				      PTR_ERR(chip->charger_mode_votable));
			return;
		}
	}

	ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
				 chip->no_external_boost ? (void *)GBMS_USB_OTG_FRS_ON :
				 (void *)GBMS_USB_OTG_ON, true);

	logbuffer_log(chip->log, "%s: GBMS_MODE_VOTABLE voting source vote:%u ret:%d",
		      ret < 0 ? "Error" : "Success", (unsigned int)vote, ret);

	if (ret < 0)
		return;

	if (!chip->sourcing_vbus)
		chip->sourcing_vbus = 1;
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

	kthread_flush_work(&chip->enable_vbus_work.work);

	if (source && !sink) {
		kthread_mod_delayed_work(chip->wq, &chip->enable_vbus_work, 0);
		return 0;
	} else if (sink && !source) {
		ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
					 (void *)GBMS_USB_BUCK_ON, true);
	} else {
		/* just one will do */
		ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
					 (void *)GBMS_USB_BUCK_ON, false);
	}

	logbuffer_log(chip->log, "%s: GBMS_MODE_VOTABLE voting source:%c sink:%c vote:%u ret:%d",
		      ret < 0 ? "Error" : "Success", source ? 'y' : 'n', sink ? 'y' : 'n',
		      (unsigned int)vote, ret);

	if (ret < 0)
		return ret;

	if (!source && chip->sourcing_vbus) {
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

	kthread_flush_work(&chip->enable_vbus_work.work);
	ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
				 (void *)GBMS_USB_OTG_FRS_ON, true);
	logbuffer_log(chip->log, "%s: GBMS_MODE_VOTABLE ret:%d", __func__, ret);

	if (!ret)
		chip->sourcing_vbus = 1;

	/*
	 * TODO: move this line to max77759_set_vbus after the change in TCPM gets upstreamed and
	 * cherry-picked to Pixel codebase.
	 * Be sure to ensure that this will only be called during FR_SWAP.
	 */
	usb_psy_set_sink_state(chip->usb_psy_data, false);
}

static void process_power_status(struct max77759_plat *chip)
{
	struct tcpci *tcpci = chip->tcpci;
	struct logbuffer *log = chip->log;
	unsigned int pwr_status;
	int ret;

	ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &pwr_status);
	logbuffer_log(log, "TCPC_ALERT_POWER_STATUS status:0x%x", pwr_status);
	if (ret < 0)
		return;

	if (pwr_status == 0xff) {
		max77759_init_regs(tcpci->regmap, log);
		return;
	}

	if (pwr_status & TCPC_POWER_STATUS_SOURCING_VBUS) {
		if (!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES)) {
			/*
			 * Sourcing vbus might be set before vbus present is
			 * set. This implies vbus has not reached VSAFE5V yet
			 * (or) TCPC_POWER_STATUS_VBUS_PRES is arriving late.
			 * Hold back signalling sourcing vbus here.
			 */
			logbuffer_log(log, "Discard sourcing vbus. Vbus present not set");
		} else {
			chip->sourcing_vbus = 1;
			tcpm_sourcing_vbus(tcpci->port);
			chip->in_frs = false;
		}
	}

	if (chip->in_frs) {
		chip->in_frs = false;
		/*
		 * While in FRS transition consider vbus present as a signal for
		 * sourcing vbus as controller would have reversed the direction
		 * here. This signal could arrive before or after
		 * TCPC_POWER_STATUS_SOURCING_VBUS
		 */
		if (pwr_status & TCPC_POWER_STATUS_VBUS_PRES) {
			chip->sourcing_vbus = 1;
			tcpm_sourcing_vbus(tcpci->port);
		}
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
		/*
		 * Renable BC1.2 upon disconnect if disabled. Needed for
		 * sink-only mode such as fastbootd/Recovery.
		 */
		if (chip->attached && !chip->debug_acc_connected && !bc12_get_status(chip->bc12))
			bc12_enable(chip->bc12, true);
		chip->attached = chip->debug_acc_connected;
		enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);
		logbuffer_log(log, "Debug accessory %s", chip->debug_acc_connected ? "connected" :
			      "disconnected");
		if (!chip->debug_acc_connected && modparam_conf_sbu) {
			ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_SBUSW_CTRL,
					      SBUSW_SERIAL_UART);
			logbuffer_log(log, "SBU switch enable %s", ret < 0 ? "fail" : "success");
		}
		usb_psy_set_attached_state(chip->usb_psy_data, chip->attached);
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

static int max77759_enable_voltage_alarm(struct max77759_plat *chip, bool enable, bool high)
{
	int ret;

	if (!enable) {
		ret = max77759_update_bits8(chip->tcpci->regmap, TCPC_POWER_CTRL,
					    TCPC_DIS_VOLT_ALRM, TCPC_DIS_VOLT_ALRM);
		if (ret < 0)
			logbuffer_log(chip->log, "Unable to disable voltage alarm, ret = %d",
				      ret);
		return ret;
	}

	/* Set voltage alarm */
	ret = max77759_update_bits16(chip->tcpci->regmap, TCPC_VBUS_VOLTAGE_ALARM_HI_CFG,
				     TCPC_VBUS_VOLTAGE_MASK,
				     (high ? VOLTAGE_ALARM_HI_EN_MV : VOLTAGE_ALARM_HI_DIS_MV) /
				     TCPC_VBUS_VOLTAGE_LSB_MV);
	if (ret < 0) {
		logbuffer_log(chip->log, "Unable to config VOLTAGE_ALARM_HI_CFG, ret = %d", ret);
		return ret;
	}

	ret = max77759_update_bits16(chip->tcpci->regmap, TCPC_VBUS_VOLTAGE_ALARM_LO_CFG,
				     TCPC_VBUS_VOLTAGE_MASK,
				     (!high ? VOLTAGE_ALARM_LOW_EN_MV : VOLTAGE_ALARM_LOW_DIS_MV) /
				     TCPC_VBUS_VOLTAGE_LSB_MV);
	if (ret < 0) {
		logbuffer_log(chip->log, "Unable to config VOLTAGE_ALARM_LO_CFG, ret = %d", ret);
		return ret;
	}

	ret = max77759_update_bits8(chip->tcpci->regmap, TCPC_POWER_CTRL, TCPC_DIS_VOLT_ALRM, 0);
	if (ret < 0) {
		logbuffer_log(chip->log, "Unable to enable voltage alarm, ret = %d", ret);
		return ret;
	}

	ret = max77759_update_bits16(chip->tcpci->regmap, TCPC_ALERT_MASK,
				     TCPC_ALERT_V_ALARM_LO | TCPC_ALERT_V_ALARM_HI,
				     high ? TCPC_ALERT_V_ALARM_HI : TCPC_ALERT_V_ALARM_LO);
	if (ret < 0)
		logbuffer_log(chip->log, "Unable to unmask voltage alarm interrupt, ret = %d", ret);

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
	ret = max77759_read16(chip->tcpci->regmap, TCPC_VBUS_VOLTAGE, &raw);

	return ret ? 0 : ((raw & TCPC_VBUS_VOLTAGE_MASK) * TCPC_VBUS_VOLTAGE_LSB_MV);
}

static irqreturn_t _max77759_irq(struct max77759_plat *chip, u16 status,
				 struct logbuffer *log)
{
	u16 vendor_status = 0, vendor_status2 = 0, raw;
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
			chip->in_frs = true;
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
				      TCPC_VENDOR_ALERT_MASK2, 0x0);
		if (ret < 0)
			return ret;

		/* Clear VENDOR_ALERT*/
		ret = max77759_read16(tcpci->regmap, TCPC_VENDOR_ALERT,
				      &vendor_status);
		if (ret < 0)
			return ret;
		logbuffer_log(log, "TCPC_VENDOR_ALERT 0x%x", vendor_status);

		process_bc12_alert(chip->bc12, vendor_status);
		ret = max77759_write16(tcpci->regmap, TCPC_VENDOR_ALERT,
				       vendor_status);

		ret = max77759_read16(tcpci->regmap, TCPC_VENDOR_ALERT2, &vendor_status2);
		if (ret < 0)
			return ret;
		logbuffer_log(log, "TCPC_VENDOR_ALERT2 0x%x", vendor_status2);

		ret = max77759_write16(tcpci->regmap, TCPC_VENDOR_ALERT2, vendor_status2);
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
		mutex_lock(&chip->rc_lock);
		if (!chip->contaminant_detection || !tcpm_is_toggling(tcpci->port) ||
		    !process_contaminant_alert(chip->contaminant, false, true)) {
			tcpm_cc_change(tcpci->port);
			/* TCPM has detected valid CC terminations */
			if (!tcpm_is_toggling(tcpci->port)) {
				chip->floating_cable_detected = 0;
				disable_auto_ultra_low_power_mode(chip, false);
				logbuffer_log(chip->log, "enable_auto_ultra_low_power_mode");
			}
		} else {
			logbuffer_log(log, "CC update: Contaminant algorithm responded");
			if (is_floating_cable_detected(chip)) {
				chip->floating_cable_detected++;
				logbuffer_log(chip->log, "floating_cable_detected count: %d",
					      chip->floating_cable_detected);
				if (chip->floating_cable_detected >=
				    FLOATING_CABLE_INSTANCE_THRESHOLD) {
					disable_auto_ultra_low_power_mode(chip, true);
					alarm_start_relative(
						&chip->reenable_auto_ultra_low_power_mode_alarm,
						ms_to_ktime(AUTO_ULTRA_LOW_POWER_MODE_REENABLE_MS));
				}
			}
		}
		mutex_unlock(&chip->rc_lock);
	}

	if (status & TCPC_ALERT_POWER_STATUS)
		process_power_status(chip);

	if (status & TCPC_ALERT_V_ALARM_LO) {
		ret = max77759_read16(tcpci->regmap, TCPC_VBUS_VOLTAGE_ALARM_LO_CFG, &raw);
		if (ret < 0)
			return ret;

		logbuffer_log(log, "VBUS LOW ALARM triggered: thresh:%umv vbus:%umv",
			      (raw & TCPC_VBUS_VOLTAGE_MASK) * TCPC_VBUS_VOLTAGE_LSB_MV,
			      max77759_get_vbus_voltage_mv(chip->client));
		max77759_enable_voltage_alarm(chip, true, true);

		ret = extcon_set_state_sync(chip->extcon, EXTCON_MECHANICAL, 0);
		logbuffer_log(chip->log, "%s turning off connected, ret=%d", __func__, ret < 0 ?
			      "Failed" : "Succeeded", ret);
	}

	if (status & TCPC_ALERT_V_ALARM_HI) {
		ret = max77759_read16(tcpci->regmap, TCPC_VBUS_VOLTAGE_ALARM_HI_CFG, &raw);
		if (ret < 0)
			return ret;

		logbuffer_log(log, "VBUS HIGH ALARM triggered: thresh:%umv vbus:%umv",
			      (raw & TCPC_VBUS_VOLTAGE_MASK) * TCPC_VBUS_VOLTAGE_LSB_MV,
			      max77759_get_vbus_voltage_mv(chip->client));
		max77759_enable_voltage_alarm(chip, true, false);

		ret = extcon_set_state_sync(chip->extcon, EXTCON_MECHANICAL, 1);
		logbuffer_log(chip->log, "%s: %s turning on connected, ret=%d", __func__, ret < 0 ?
			      "Failed" : "Succeeded", ret);
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
				      TCPC_VENDOR_ALERT_MASK2, 0xff);
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

/* Called while holding rc_lock */
static void max77759_enable_toggling_locked(struct max77759_plat *chip, bool enable)
{
	int ret;

	if (!enable) {
		ret = max77759_write8(chip->data.regmap, TCPC_ROLE_CTRL, TCPCI_HI_Z_CC);
		logbuffer_log(chip->log, "%s: HI-Z ret:%d", __func__, ret);
		return;
	}

	ret = max77759_write8(chip->data.regmap, TCPC_ROLE_CTRL, chip->role_ctrl_cache);
	if (ret < 0) {
		logbuffer_log(chip->log, "%s: update ROLE_CTRL failed ret:%d", __func__, ret);
		return;
	}

	ret = max77759_update_bits8(chip->data.regmap, TCPC_TCPC_CTRL,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0) {
		logbuffer_log(chip->log, "%s: Enable LK4CONN alert failed ret:%d", __func__, ret);
		return;
	}

	ret = regmap_write(chip->data.regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	if (ret < 0)
		logbuffer_log(chip->log, "%s: Enable LK4CONN failed ret:%d", __func__, ret);
}
static int max77759_start_toggling(struct tcpci *tcpci,
				   struct tcpci_data *tdata,
				   enum typec_cc_status cc)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	u8 reg = TCPC_ROLE_CTRL_DRP, pwr_ctrl;
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

	chip->role_ctrl_cache = reg;
	mutex_lock(&chip->rc_lock);
	if (chip->toggle_disable_status)
		goto unlock;

	/* Kick debug accessory state machine when enabling toggling for the first time */
	if (chip->first_toggle && chip->in_switch_gpio >= 0) {
		logbuffer_log(chip->log, "[%s]: Kick Debug accessory FSM", __func__);
		gpio_set_value_cansleep(chip->in_switch_gpio, 0);
		mdelay(10);
		gpio_set_value_cansleep(chip->in_switch_gpio, 1);
		chip->first_toggle = false;
	}

	/* Renable BC1.2*/
	if (!bc12_get_status(chip->bc12))
		bc12_enable(chip->bc12, true);

	/* Re-enable retry */
	bc12_reset_retry(chip->bc12);

	/* Disable Auto disacharge before enabling toggling */
	ret = max77759_read8(tcpci->regmap, TCPC_POWER_CTRL, &pwr_ctrl);
	logbuffer_log(chip->log, "TCPC_POWER_CTRL:0x%x ret:%d", pwr_ctrl, ret);
	if (pwr_ctrl & TCPC_POWER_CTRL_AUTO_DISCHARGE) {
		logbuffer_log(chip->log, "TCPC_POWER_CTRL_AUTO_DISCHARGE not cleared");
		ret = regmap_update_bits(tcpci->regmap, TCPC_POWER_CTRL,
					 TCPC_POWER_CTRL_AUTO_DISCHARGE, 0);
		if (ret < 0)
			logbuffer_log(chip->log, "[%s]: Disabling auto discharge failed", __func__);
	}

	if (chip->contaminant_detection)
		update_contaminant_detection_locked(chip, chip->contaminant_detection);
	else
		max77759_enable_toggling_locked(chip, true);

unlock:
	mutex_unlock(&chip->rc_lock);

	return 0;
}

static void max77759_check_contaminant(void *unused, struct tcpci *tcpci, struct tcpci_data *tdata,
				       int *ret)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);

	logbuffer_log(chip->log, "%s: debounce path", __func__);
	mutex_lock(&chip->rc_lock);
	if (chip->contaminant_detection) {
		process_contaminant_alert(chip->contaminant, true, false);
		*ret = CONTAMINANT_HANDLES_TOGGLING;
	} else {
		*ret = TCPM_RESTART_TOGGLING;
	}

	mutex_unlock(&chip->rc_lock);
}

static void max77759_set_partner_usb_comm_capable(struct tcpci *tcpci, struct tcpci_data *data,
						  bool capable)
{
	struct max77759_plat *chip = tdata_to_max77759(data);

	mutex_lock(&chip->data_path_lock);
	chip->pd_data_capable = capable;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);
}

static int max77759_usb_set_orientation(struct typec_switch *sw, enum typec_orientation orientation)
{
	struct max77759_plat *chip = typec_switch_get_drvdata(sw);
	enum typec_cc_polarity polarity = orientation == TYPEC_ORIENTATION_REVERSE ?
		TYPEC_POLARITY_CC2 : TYPEC_POLARITY_CC1;
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
	return ret;
}

static int max77759_vote_icl(struct max77759_plat *chip, u32 max_ua)
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
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, icl_work);

	max77759_vote_icl(chip, chip->typec_current_max);
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct max77759_plat *chip = container_of(nb, struct max77759_plat, psy_notifier);
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

static int max77759_get_vbus_voltage_max_mv(struct i2c_client *tcpc_client)
{
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	return chip ? chip->vbus_mv : 0;
}

static int max77759_set_vbus_voltage_max_mv(struct i2c_client *tcpc_client,
					    unsigned int mv)
{
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	if (chip)
		chip->vbus_mv = mv;

	return 0;
}

static void max77759_get_vbus(void *unused, struct tcpci *tcpci, struct tcpci_data *data, int *vbus,
			      int *bypass)
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
	*vbus = chip->vbus_present;
	*bypass = 1;
}

static int max77759_usb_set_role(struct usb_role_switch *sw, enum usb_role role)
{
	struct max77759_plat *chip = usb_role_switch_get_drvdata(sw);
	enum typec_data_role typec_data_role = TYPEC_DEVICE;
	bool attached = role != USB_ROLE_NONE, enable_data;
	int ret;

	if (role == USB_ROLE_HOST)
		typec_data_role = TYPEC_HOST;

	mutex_lock(&chip->data_path_lock);

	enable_data = chip->pd_data_capable || chip->no_bc_12 || chip->bc12_data_capable ||
		chip->data_role == TYPEC_HOST || chip->debug_acc_connected;

	if (!chip->force_device_mode_on && chip->data_active &&
	    (chip->active_data_role != typec_data_role || !attached || !enable_data)) {
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
			ret = max77759_write8(chip->data.regmap, TCPC_VENDOR_USBSW_CTRL,
					      USBSW_DISCONNECT);
			logbuffer_log(chip->log, "Turning off dp switches %s", ret < 0 ? "fail" :
				      "success");
		}
	}

	/* Renable BC1.2 */
	if (chip->attached && !attached && !bc12_get_status(chip->bc12))
		bc12_enable(chip->bc12, true);
	/*
	 * To prevent data stack enumeration failure, previously there
	 * was a 300msec delay here
	 */

	chip->attached = attached;
	chip->data_role = typec_data_role;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);
	usb_psy_set_attached_state(chip->usb_psy_data, chip->attached);

	/*
	 * Renable BC1.2 upon disconnect if disabled. Needed for sink-only mode such as
	 * fastbootd/Recovery.
	 */
	if (chip->attached && !attached && !bc12_get_status(chip->bc12))
		bc12_enable(chip->bc12, true);

	return 0;
}

static void max77759_store_partner_src_caps(void *unused, struct tcpm_port *port,
					    unsigned int *nr_source_caps,
					    u32 (*source_caps)[PDO_MAX_OBJECTS])
{
	int i;

	spin_lock(&g_caps_lock);

	nr_partner_src_caps = *nr_source_caps > PDO_MAX_OBJECTS ?
			      PDO_MAX_OBJECTS : *nr_source_caps;

	for (i = 0; i < nr_partner_src_caps; i++)
		partner_src_caps[i] = (*source_caps)[i];

	spin_unlock(&g_caps_lock);
}

/*
 * Don't call this function in interrupt context. Caller needs to free the
 * memory by calling tcpm_put_partner_src_caps.
 */
int tcpm_get_partner_src_caps(struct tcpm_port *port, u32 **src_pdo)
{
	int i, ret;

	*src_pdo = kcalloc(PDO_MAX_OBJECTS, sizeof(u32), GFP_KERNEL);
	if (!src_pdo)
		return -ENOMEM;

	spin_lock(&g_caps_lock);

	if (!nr_partner_src_caps) {
		ret = -ENODATA;
		goto cleanup;
	}

	for (i = 0, ret = nr_partner_src_caps; i < nr_partner_src_caps; i++)
		(*src_pdo)[i] = partner_src_caps[i];

	goto unlock;

cleanup:
	kfree(*src_pdo);
	*src_pdo = NULL;
unlock:
	spin_unlock(&g_caps_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tcpm_get_partner_src_caps);

void tcpm_put_partner_src_caps(u32 **src_pdo)
{
	kfree(*src_pdo);
	*src_pdo = NULL;
}
EXPORT_SYMBOL_GPL(tcpm_put_partner_src_caps);

void max77759_bc12_is_running(struct max77759_plat *chip, bool running)
{
	if (chip) {
		mutex_lock(&chip->data_path_lock);
		chip->bc12_running = running;
		if (!running)
			enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);
	}
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
	case POWER_SUPPLY_USB_TYPE_DCP:
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		mutex_lock(&chip->data_path_lock);
		chip->bc12_data_capable = false;
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
	EXTCON_MECHANICAL,
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

static void max77759_toggle_disable_votable_callback(struct gvotable_election *el,
						     const char *reason, void *value)
{
	struct max77759_plat *chip = gvotable_get_data(el);
	int disable = (long)value ? MAX77759_DISABLE_TOGGLE : MAX77759_ENABLE_TOGGLE;

	mutex_lock(&chip->rc_lock);
	if (chip->toggle_disable_status == disable) {
		mutex_unlock(&chip->rc_lock);
		return;
	}

	chip->toggle_disable_status = disable;
	if (chip->toggle_disable_status) {
		update_contaminant_detection_locked(chip, CONTAMINANT_DETECT_DISABLE);
		disable_contaminant_detection(chip);
		max77759_enable_toggling_locked(chip, false);
		gpio_set_value_cansleep(chip->in_switch_gpio, 0);
		logbuffer_log(chip->log, "[%s]: Disable in-switch", __func__);
	} else {
		if (chip->contaminant_detection_userspace)
			update_contaminant_detection_locked(chip,
							    chip->contaminant_detection_userspace);
		else
			max77759_enable_toggling_locked(chip, true);
		gpio_set_value_cansleep(chip->in_switch_gpio, 1);
		logbuffer_log(chip->log, "[%s]: Enable in-switch", __func__);
	}
	mutex_unlock(&chip->rc_lock);
	logbuffer_log(chip->log, "%s: reason %s value %ld\n", __func__, reason, (long)value);
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

static void max77759_typec_tcpci_override_toggling(void *unused, struct tcpci *tcpci,
						   struct tcpci_data *data,
						   int *override_toggling)
{
	*override_toggling = 1;
}

static void max77759_get_timer_value(void *unused, const char *state, enum typec_timer timer,
				     unsigned int *val)
{
	switch (timer) {
	case SINK_DISCOVERY_BC12:
		*val = sink_discovery_delay_ms;
		break;
	case SINK_WAIT_CAP:
		*val = 450;
		break;
	case SOURCE_OFF:
		*val = 870;
		break;
	case CC_DEBOUNCE:
		*val = 170;
		break;
	default:
		break;
	}
}

static void max77759_tcpm_log(void *unused, const char *log, bool *bypass)
{
	if (tcpm_log)
		logbuffer_log(tcpm_log, "%s", log);

	*bypass = true;
}

static int max77759_register_vendor_hooks(struct i2c_client *client)
{
	int ret;

	if (hooks_installed)
		return 0;

	ret = register_trace_android_vh_typec_tcpci_override_toggling(
			max77759_typec_tcpci_override_toggling, NULL);

	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_vh_typec_tcpci_override_toggling failed ret:%d",
			ret);
		return ret;
	}

	ret = register_trace_android_rvh_typec_tcpci_chk_contaminant(
			max77759_check_contaminant, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_rvh_typec_tcpci_chk_contaminant failed ret:%d",
			ret);
		return ret;
	}

	ret = register_trace_android_rvh_typec_tcpci_get_vbus(max77759_get_vbus, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_rvh_typec_tcpci_get_vbus failed ret:%d\n", ret);
		return ret;
	}

	ret = register_trace_android_vh_typec_store_partner_src_caps(
			max77759_store_partner_src_caps, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_vh_typec_store_partner_src_caps failed ret:%d\n",
			ret);
		return ret;
	}

	ret = register_trace_android_vh_typec_tcpm_get_timer(max77759_get_timer_value, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_vh_typec_tcpm_get_timer failed ret:%d\n", ret);
		return ret;
	}

	ret = register_trace_android_vh_typec_tcpm_log(max77759_tcpm_log, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_vh_typec_tcpm_log failed ret:%d\n", ret);
		return ret;
	}

	hooks_installed = true;

	return ret;
}

static int max77759_setup_data_notifier(struct max77759_plat *chip)
{
	struct usb_role_switch_desc desc = { };
	struct typec_switch_desc sw_desc = { };
	u32 conn_handle;
	int ret;

	chip->extcon = devm_extcon_dev_allocate(chip->dev, usbpd_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		dev_err(chip->dev, "Error allocating extcon: %d\n",
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
	desc.set = max77759_usb_set_role;

	chip->usb_sw = usb_role_switch_register(chip->dev, &desc);
	if (IS_ERR(chip->usb_sw)) {
		ret = PTR_ERR(chip->usb_sw);
		dev_err(chip->dev, "Error while registering role switch:%d\n", ret);
		return ret;
	}

	sw_desc.fwnode = dev_fwnode(chip->dev);
	sw_desc.drvdata = chip;
	sw_desc.name = fwnode_get_name(dev_fwnode(chip->dev));
	sw_desc.set = max77759_usb_set_orientation;

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

static void max77759_teardown_data_notifier(struct max77759_plat *chip)
{
	if (!IS_ERR_OR_NULL(chip->typec_sw))
		typec_switch_unregister(chip->typec_sw);
	if (!IS_ERR_OR_NULL(chip->usb_sw))
		usb_role_switch_unregister(chip->usb_sw);
}

static void reenable_auto_ultra_low_power_mode_work_item(struct kthread_work *work)
{
	struct max77759_plat *chip = container_of(work, struct max77759_plat,
						  reenable_auto_ultra_low_power_mode_work);

	chip->floating_cable_detected = 0;
	disable_auto_ultra_low_power_mode(chip, false);
}

static enum alarmtimer_restart reenable_auto_ultra_low_power_mode_alarm_handler(struct alarm *alarm,
										ktime_t time)
{
	struct max77759_plat *chip = container_of(alarm, struct max77759_plat,
						  reenable_auto_ultra_low_power_mode_alarm);

	logbuffer_log(chip->log, "timer fired: enable_auto_ultra_low_power_mode");
	if (is_contaminant_detected(chip)) {
		logbuffer_log(chip->log,
			      "Skipping enable_auto_ultra_low_power_mode. Dry detection in progress");
		goto exit;
	}
	kthread_queue_work(chip->wq, &chip->reenable_auto_ultra_low_power_mode_work);
	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);

exit:
	return ALARMTIMER_NORESTART;
}

static int max77759_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	int ret, i;
	struct max77759_plat *chip;
	char *usb_psy_name;
	struct device_node *dn, *ovp_dn;
	u8 power_status;
	u16 device_id;
	u32 ovp_handle;
	const char *ovp_status;

	ret = max77759_register_vendor_hooks(client);
	if (ret)
		return ret;

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

	kthread_init_work(&chip->reenable_auto_ultra_low_power_mode_work,
			  reenable_auto_ultra_low_power_mode_work_item);
	alarm_init(&chip->reenable_auto_ultra_low_power_mode_alarm, ALARM_BOOTTIME,
		   reenable_auto_ultra_low_power_mode_alarm_handler);
	dn = dev_of_node(&client->dev);
	if (!dn) {
		dev_err(&client->dev, "of node not found\n");
		return -EINVAL;
	}

	if (!of_property_read_u32(dn, "max20339,ovp", &ovp_handle)) {
		ovp_dn = of_find_node_by_phandle(ovp_handle);
		if (!IS_ERR_OR_NULL(ovp_dn) &&
		    !of_property_read_string(ovp_dn, "status", &ovp_status) &&
		    strncmp(ovp_status, "disabled", strlen("disabled"))) {
			chip->in_switch_gpio = of_get_named_gpio(dn, "in-switch-gpio", 0);
			if (chip->in_switch_gpio < 0) {
				dev_err(&client->dev, "in-switch-gpio not found\n");
				return -EPROBE_DEFER;
			}
		}
	}

	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->icl_proto_el_lock);
	mutex_init(&chip->data_path_lock);
	mutex_init(&chip->rc_lock);
	spin_lock_init(&g_caps_lock);
	chip->first_toggle = true;

	ret = max77759_read8(chip->data.regmap, TCPC_POWER_STATUS,
			     &power_status);
	if (ret < 0)
		return ret;

	if (power_status & TCPC_POWER_STATUS_UNINIT) {
		dev_err(&client->dev, "TCPC not ready!");
		return -EPROBE_DEFER;
	}

	chip->toggle_disable_votable =
		gvotable_create_bool_election(NULL, max77759_toggle_disable_votable_callback, chip);
	if (IS_ERR_OR_NULL(chip->toggle_disable_votable)) {
		ret = PTR_ERR(chip->toggle_disable_votable);
		dev_err(chip->dev, "no toggle_disable votable (%d)\n", ret);
		return ret;
	}
	gvotable_set_vote2str(chip->toggle_disable_votable, gvotable_v2s_int);
	gvotable_election_set_name(chip->toggle_disable_votable, "TOGGLE_DISABLE");

	/* Chip level tcpci callbacks */
	chip->data.set_vbus = max77759_set_vbus;
	chip->data.start_drp_toggling = max77759_start_toggling;
	chip->data.TX_BUF_BYTE_x_hidden = 1;
	chip->data.vbus_vsafe0v = true;
	chip->data.set_partner_usb_comm_capable = max77759_set_partner_usb_comm_capable;
	chip->data.init = tcpci_init;
	chip->data.frs_sourcing_vbus = max77759_frs_sourcing_vbus;

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
	chip->bc12 = bc12_init(chip, max77759_bc12_is_running);
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
	chip->no_external_boost = of_property_read_bool(dn, "no-external-boost");
	of_property_read_u32(dn, "sink-discovery-delay-ms", &sink_discovery_delay_ms);

	chip->usb_psy = power_supply_get_by_name(usb_psy_name);
	if (IS_ERR_OR_NULL(chip->usb_psy) || !chip->usb_psy) {
		dev_err(&client->dev, "usb psy not up\n");
		ret = -EPROBE_DEFER;
		goto teardown_bc12;
	}

	ret = max77759_read16(chip->data.regmap, TCPC_BCD_DEV, &device_id);
	if (ret < 0)
		goto psy_put;

	logbuffer_log(chip->log, "TCPC DEVICE id:%d", device_id);
	/* Default enable on A1 or higher */
	chip->contaminant_detection = device_id >= MAX77759_DEVICE_ID_A1;
	chip->contaminant_detection_userspace = chip->contaminant_detection;
	chip->contaminant = max77759_contaminant_init(chip, chip->contaminant_detection);

	ret = max77759_setup_data_notifier(chip);
	if (ret < 0)
		goto psy_put;
	max77759_init_regs(chip->data.regmap, chip->log);

	/* Default enable on A1 or higher */
	if (device_id >= MAX77759_DEVICE_ID_A1) {
		chip->data.auto_discharge_disconnect = true;
		chip->frs = true;
	}

	chip->wq = kthread_create_worker(0, "wq-tcpm-tcpc");
	if (IS_ERR_OR_NULL(chip->wq)) {
		ret = PTR_ERR(chip->wq);
		goto teardown_data;
	}

	kthread_init_delayed_work(&chip->icl_work, icl_work_item);
	kthread_init_delayed_work(&chip->enable_vbus_work, enable_vbus_work);

	chip->psy_notifier.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chip->psy_notifier);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register power supply callback\n");
		goto destroy_worker;
	}

	chip->usb_icl_proto_el = gvotable_election_get_handle(USB_ICL_PROTO_EL);
	if (IS_ERR_OR_NULL(chip->usb_icl_proto_el)) {
		dev_err(&client->dev, "TCPCI: USB ICL PROTO EL get failed:%d",
			PTR_ERR(chip->usb_icl_proto_el));
		ret = -ENODEV;
		goto unreg_notifier;
	}

	chip->tcpci = tcpci_register_port(chip->dev, &chip->data);
	if (IS_ERR_OR_NULL(chip->tcpci)) {
		dev_err(&client->dev, "TCPCI port registration failed");
		ret = PTR_ERR(chip->tcpci);
		goto unreg_notifier;
	}
	chip->port = tcpci_get_tcpm_port(chip->tcpci);

	max77759_enable_voltage_alarm(chip, true, true);

	ret = max77759_init_alert(chip, client);
	if (ret < 0)
		goto unreg_port;

	device_init_wakeup(chip->dev, true);

	for (i = 0; max77759_device_attrs[i]; i++) {
		ret = device_create_file(&client->dev, max77759_device_attrs[i]);
		if (ret < 0)
			dev_err(&client->dev, "TCPCI: Unable to create device attr[%d] ret:%d:", i,
				ret);
	}

	if (!modparam_conf_sbu) {
		ret = max77759_write8(chip->data.regmap, TCPC_VENDOR_SBUSW_CTRL, 0);
		logbuffer_log(chip->log, "SBU switch disable %s", ret < 0 ? "fail" : "success");
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
unreg_notifier:
	power_supply_unreg_notifier(&chip->psy_notifier);
destroy_worker:
	kthread_destroy_worker(chip->wq);
teardown_data:
	max77759_teardown_data_notifier(chip);
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
	if (!IS_ERR_OR_NULL(chip->wq))
		kthread_destroy_worker(chip->wq);
	power_supply_unreg_notifier(&chip->psy_notifier);
	max77759_teardown_data_notifier(chip);

	return 0;
}

static void max77759_shutdown(struct i2c_client *client)
{
	struct max77759_plat *chip = i2c_get_clientdata(client);
	int ret;

	dev_info(&client->dev, "disabling Type-C upon shutdown\n");
	/* Set current limit to 0. Will eventually happen after hi-Z as well */
	max77759_vote_icl(chip, 0);
	/* Prevent re-enabling toggling */
	/* Hi-z CC pins to trigger disconnection */
	ret = gvotable_cast_vote(chip->toggle_disable_votable, "SHUTDOWN_VOTE",
				 (void *)MAX77759_DISABLE_TOGGLE_VOTE, MAX77759_DISABLE_TOGGLE);
	if (ret < 0)
		dev_err(chip->dev, "Cannot set TOGGLE DISABLE (%d)\n", ret);
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
	.shutdown = max77759_shutdown,
};

static int __init max77759_i2c_driver_init(void)
{
	tcpm_log = logbuffer_register("tcpm");
	if (IS_ERR_OR_NULL(tcpm_log))
		return -EAGAIN;

	return i2c_add_driver(&max77759_i2c_driver);
}
module_init(max77759_i2c_driver_init);

static void __exit max77759_i2c_driver_exit(void)
{
	i2c_del_driver(&max77759_i2c_driver);
}
module_exit(max77759_i2c_driver_exit);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("MAX77759 USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL");
