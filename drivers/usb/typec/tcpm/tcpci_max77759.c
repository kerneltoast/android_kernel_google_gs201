// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019, Google LLC
 *
 * MAX77759 TCPCI driver
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
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>

#include "bc_max77759.h"
#include "max77759_helper.h"
#include "tcpci.h"
#include "tcpci_max77759.h"
#include "tcpci_max77759_vendor_reg.h"
#include "usb_icl_voter.h"
#include "usb_psy.h"

#include <../../../power/supply/google/logbuffer.h>

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

	ret = regmap_write(regmap, TCPC_EXTENDED_STATUS_MASK,
			   TCPC_EXTENDED_STATUS_VSAFE0V);
	if (ret < 0) {
		logbuffer_log(log,
			      "Error writing TCPC_EXTENDED_STATUS_MASK ret:%d"
			      , ret);
		return;
	}

	logbuffer_log(log, "[%s] Init EXTENDED_STATUS_MASK: VSAFE0V", __func__);

	alert_mask = TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_DISCARDED |
		TCPC_ALERT_TX_FAILED | TCPC_ALERT_RX_HARD_RST |
		TCPC_ALERT_RX_STATUS | TCPC_ALERT_VENDOR | TCPC_ALERT_CC_STATUS |
		TCPC_ALERT_VBUS_DISCNCT | TCPC_ALERT_RX_BUF_OVF |
		TCPC_ALERT_EXTENDED_STATUS;

	alert_mask |= TCPC_ALERT_POWER_STATUS;

	ret = max77759_write16(regmap, TCPC_ALERT_MASK, alert_mask);
	if (ret < 0)
		return;
	logbuffer_log(log, "[%s] Init ALERT_MASK: %u", __func__,
		      alert_mask);

	/* Enable vbus voltage monitoring and voltage alerts */
	ret = max77759_write8(regmap, TCPC_POWER_CTRL, 0);
	if (ret < 0)
		return;
	logbuffer_log(log,
		      "TCPC_POWER_CTRL: Enable voltage monitoring and alarm"
		      );
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
	ret = regmap_raw_read(chip->data.regmap, TCPC_RX_BYTE_CNT, rx_buf, 2);
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

	if (count > sizeof(struct pd_message) || count + 1 > TCPC_RECEIVE_BUFFER_LEN) {
		dev_err(chip->dev, "Invalid TCPC_RX_BYTE_CNT %d", count);
		return;
	}

	/*
	 * Read count + 1 as RX_BUF_BYTE_x is hidden and can only be read through
	 * TCPC_RX_BYTE_CNT
	 */
	count += 1;
	ret = regmap_raw_read(chip->data.regmap, TCPC_RX_BYTE_CNT, rx_buf, count);
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

static void process_power_status(struct tcpci *tcpci, struct logbuffer *log)
{
	unsigned int pwr_status;
	int ret;

	ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &pwr_status);
	logbuffer_log(log, "TCPC_ALERT_POWER_STATUS status:%d", pwr_status);
	if (ret < 0)
		return;

	if (pwr_status == 0xff)
		max77759_init_regs(tcpci->regmap, log);
	else
		tcpm_vbus_change(tcpci->port);
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
		tcpm_vbus_change(tcpci->port);
	}

	if (status & TCPC_ALERT_CC_STATUS) {
		/**
		 * Process generic CC updates if it doesn't belong to
		 * contaminant detection.
		 */
		if (!process_contaminant_alert(chip->contaminant))
			tcpm_cc_change(tcpci->port);
		else
			logbuffer_log(log, "CC update: Contaminant");
	}

	if (status & TCPC_ALERT_POWER_STATUS)
		process_power_status(tcpci, log);

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

	if (status & TCPC_VENDOR_ALERT) {
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
		logbuffer_log(log, "VSAFE0V: %c\n", raw &
			      TCPC_EXTENDED_STATUS_VSAFE0V ? 'Y' : 'N');
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

	if (!chip->tcpci)
		return IRQ_HANDLED;

	ret = max77759_read16(chip->tcpci->regmap, TCPC_ALERT, &status);
	if (ret < 0)
		return ret;
	while (status) {
		irq_return = _max77759_irq(chip, status, chip->log);
		max77759_read16(chip->tcpci->regmap, TCPC_ALERT, &status);
		logbuffer_log(chip->log, "TCPC_ALERT status pending: %#x",
			      status);
	}

	return irq_return;
}

static int max77759_init_alert(struct max77759_plat *chip,
			       struct i2c_client *client)
{
	int ret, irq_gpio;

	irq_gpio = of_get_named_gpio(client->dev.of_node, "usbpd,usbpd_int", 0);
	client->irq = gpio_to_irq(irq_gpio);
	if (!client->irq)
		return -ENODEV;

	ret = devm_request_threaded_irq(chip->dev, client->irq, NULL,
					max77759_irq,
					(IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND
					 | IRQF_ONESHOT),
					dev_name(chip->dev), chip);

	if (ret < 0)
		return ret;

	enable_irq_wake(client->irq);
	return 0;
}

#if 0
struct regmap *get_chg_regmap(void);

static int vote_mode(struct max77759_plat *chip, const char *client, u8 value)
{
	struct regmap *chg_regmap;

	// TODO based on b/151248868
	return 0;

	//struct regmap *chg_regmap = get_chg_regmap();

	if (!chg_regmap) {
		logbuffer_log(chip->log, "vote_mode chg_regmap null\n");
		return 0;
	}

	return max77759_update_bits8(chg_regmap, CHG_CNFG_00, MODE_MASK,
				  value);
}

#else
static void max77759_vbus_enable(struct i2c_client *i2c, int slave_addr, bool
				 enable)
{
	u8 buffer_en[2] = {0xb9, 0xa};
	u8 buffer_dis[2] = {0xb9, 0x5};

	struct i2c_msg msgs[] = {
		{
			.addr = slave_addr,
			.flags = i2c->flags & I2C_M_TEN,
			.len = 2,
			.buf = enable ? buffer_en : buffer_dis,
		},
	};

	i2c_transfer(i2c->adapter, msgs, 1);

}
#endif

static int max77759_set_vbus(struct tcpci *tcpci, struct tcpci_data *tdata,
			     bool source, bool sink)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);

	if (source && sink)
		logbuffer_log(chip->log, "WARN: both source and sink set");

	max77759_vbus_enable(chip->client, 0x69, source);

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

	ret = max77759_write8(tcpci->regmap, TCPC_ROLE_CTRL, reg);
	if (ret < 0)
		return ret;

	ret = max77759_update_bits8(tcpci->regmap, TCPC_TCPC_CTRL,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0)
		return ret;
	/* TODO: REMOVE when enabling contaminant detection */
	return regmap_write(tcpci->regmap, TCPC_COMMAND,
			    TCPC_CMD_LOOK4CONNECTION);
}

int tcpc_get_vbus_voltage_mv(struct i2c_client *tcpc_client)
{
	u16 raw;
	int ret;
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	/* TCPC_POWER_CTRL_VBUS_VOLT_MON enabled in init_regs */
	ret = max77759_read16(chip->tcpci->regmap, TCPC_VBUS_VOLTAGE,
			      &raw);
	return ret ? 0 : ((raw & TCPC_VBUS_VOLTAGE_MASK) *
		TCPC_VBUS_VOLTAGE_LSB_MV);
}
EXPORT_SYMBOL_GPL(tcpc_get_vbus_voltage_mv);

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

	chip->pd_capable = capable;
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

	val.intval = mv * 1000;

	/*
	 ret = power_supply_set_property(chip->usb_psy,
					POWER_SUPPLY_PROP_VOLTAGE_MAX,
					&val);
	if (ret < 0) {
		logbuffer_log(chip->log,
			      "unable to set max voltage to %d, ret=%d",
				mv, ret);
		return ret;
	}
	*/

	logbuffer_log(chip->log, "max_ma=%d, mv=%d", max_ma, mv);
	max77759_vote_icl(tcpci, tdata, max_ma);

	return ret;
}

int tcpc_get_vbus_voltage_max_mv(struct i2c_client *tcpc_client)
{
	u16 raw;
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);
	int ret;

	ret = max77759_read16(chip->tcpci->regmap,
			      TCPC_VBUS_VOLTAGE_ALARM_HI_CFG, &raw);
	if (ret < 0)
		return chip->vbus_mv;

	return raw * TCPC_VBUS_VOLTAGE_ALARM_HI_CFG - VBUS_HI_HEADROOM_MV;
}
EXPORT_SYMBOL_GPL(tcpc_get_vbus_voltage_max_mv);

int tcpc_set_vbus_voltage_max_mv(struct i2c_client *tcpc_client,
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
EXPORT_SYMBOL_GPL(tcpc_set_vbus_voltage_max_mv);

static void enable_data_path(struct max77759_plat *chip)
{
	int ret;

	mutex_lock(&chip->data_path_lock);
	if (chip->attached && (chip->no_bc_12 || chip->data_capable) &&
	    !chip->data_active) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 1
					    );
		logbuffer_log(chip->log, "%s turning on %s", ret < 0 ?
			      "Failed" : "Succeeded", chip->data_role ==
			      TYPEC_HOST ? "Host" : "Device");

		chip->data_active = true;
		chip->active_data_role = chip->data_role;
	}
	mutex_unlock(&chip->data_path_lock);
}

/* Notifier structure inferred from usbpd-manager.c */
static int max77759_set_roles(struct tcpci *tcpci, struct tcpci_data *data
			       , bool attached, enum typec_role role,
			       enum typec_data_role data_role)
{
	struct max77759_plat *chip = tdata_to_max77759(data);
	int ret;

	if (chip->data_active && ((chip->active_data_role != data_role) ||
				  !attached)) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->active_data_role ==
					    TYPEC_HOST ? EXTCON_USB_HOST :
					    EXTCON_USB, 0);

		logbuffer_log(chip->log, "%s turning off %s", ret < 0 ?
			      "Failed" : "Succeeded",
			      chip->active_data_role == TYPEC_HOST ? "Host"
			      : "Device");
		chip->data_active = false;
	}

	/* Data stack needs a clean up to fix this. */
	msleep(300);

	chip->attached = attached;
	chip->data_role = data_role;
	enable_data_path(chip);
	if (!chip->attached)
		max77759_vote_icl(tcpci, data, 0);

	return 0;
}

void tcpc_set_port_data_capable(struct i2c_client *tcpc_client,
				enum power_supply_usb_type usb_type)
{
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	switch (usb_type) {
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_CDP:
	/*
	 * TODO: Remove: Being conservative, enable data path for UNKNOWN
	 * as well.
	 */
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		chip->data_capable = true;
		enable_data_path(chip);
		break;
	default:
		chip->data_capable = false;
		break;
	}
}
EXPORT_SYMBOL_GPL(tcpc_set_port_data_capable);


static const unsigned int usbpd_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DISP_DP,
	EXTCON_NONE,
};

static int max77759_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	int ret;
	struct max77759_plat *chip;
	char *usb_psy_name;
	struct device_node *dn;
	u8 power_status;

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

	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->icl_proto_el_lock);
	mutex_init(&chip->data_path_lock);

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
	chip->data.start_drp_toggling = max77759_start_toggling;
	chip->data.get_current_limit = max77759_get_current_limit;
	chip->data.set_current_limit = max77759_set_current_limit;
	chip->data.TX_BUF_BYTE_x_hidden = 1;
	chip->data.override_toggling = true;
	chip->data.set_pd_capable = max77759_set_pd_capable;
	chip->data.set_roles = max77759_set_roles;

	chip->log = debugfs_logbuffer_register("usbpd");
	if (IS_ERR_OR_NULL(chip->log)) {
		dev_err(&client->dev, "logbuffer get failed");
		return PTR_ERR(chip->log);
	}

	chip->usb_psy_data = usb_psy_setup(client, chip->log);
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

	dn = dev_of_node(&client->dev);
	if (!dn) {
		dev_err(&client->dev, "of node not found\n");
		ret = -EINVAL;
		goto teardown_bc12;
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

	chip->contaminant = max77759_contaminant_init(chip, false);

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

	max77759_init_regs(chip->data.regmap, chip->log);
	chip->tcpci = tcpci_register_port(chip->dev, &chip->data);
	if (IS_ERR_OR_NULL(chip->tcpci)) {
		dev_err(&client->dev, "TCPCI port registration failed");
		ret = PTR_ERR(chip->tcpci);
		goto psy_put;
	}
	chip->port = tcpci_get_tcpm_port(chip->tcpci);

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

	return 0;

unreg_port:
	tcpci_unregister_port(chip->tcpci);
psy_put:
	power_supply_put(chip->usb_psy);
teardown_bc12:
	bc12_teardown(chip->bc12);
unreg_psy:
	usb_psy_teardown(chip->usb_psy_data);
logbuffer_unreg:
	debugfs_logbuffer_unregister(chip->log);

	return ret;
}

static int max77759_remove(struct i2c_client *client)
{
	struct max77759_plat *chip = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(chip->tcpci))
		tcpci_unregister_port(chip->tcpci);
	if (!IS_ERR_OR_NULL(chip->usb_psy))
		power_supply_put(chip->usb_psy);
	if (!IS_ERR_OR_NULL(chip->usb_psy_data))
		usb_psy_teardown(chip->usb_psy_data);
	if (!IS_ERR_OR_NULL(chip->bc12))
		bc12_teardown(chip->bc12);
	if (!IS_ERR_OR_NULL(chip->log))
		debugfs_logbuffer_unregister(chip->log);
	if (!IS_ERR_OR_NULL(chip->extcon))
		devm_extcon_dev_free(chip->dev, chip->extcon);

	return 0;
}

static const struct i2c_device_id max77759_id[] = {
	{ "max77759_tcpc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77759_id);

#ifdef CONFIG_OF
static const struct of_device_id max77759_of_match[] = {
	{ .compatible = "max77759_tcpc", },
	{},
};
MODULE_DEVICE_TABLE(of, max77759_of_match);
#endif

static struct i2c_driver max77759_i2c_driver = {
	.driver = {
		.name = "max77759_tcpc",
		.of_match_table = of_match_ptr(max77759_of_match),
	},
	.probe = max77759_probe,
	.remove = max77759_remove,
	.id_table = max77759_id,
};
module_i2c_driver(max77759_i2c_driver);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("MAX77759 USB Type-C Port Controller Interface Driver");
