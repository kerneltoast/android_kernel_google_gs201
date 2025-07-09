// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung EUSB Repeater driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/extcon.h>

#include "eusb_repeater.h"

struct eusb_repeater_data *g_tud;

/*
 * Time 3 ms is required for the eusb repeater to be ready to accept RAP and I2C requests.
 * See eUSB2 Repeater 7.7 for timing requirement.
 */
static void ensure_eusb_repeater_ready(struct eusb_repeater_data *tud)
{
	mutex_lock(&tud->mutex);
	ktime_t now = ktime_get();
	const ktime_t target = ktime_add_us(tud->start_time, TIME_RH_READY);

	if (!tud->ready) {
		if (ktime_before(now, target))
			udelay(ktime_us_delta(target, now));
		tud->ready = true;
	}
	mutex_unlock(&tud->mutex);
}

int eusb_repeater_i2c_write(struct eusb_repeater_data *tud, u8 reg, u8 *data, int len)
{
	u8 buf[I2C_WRITE_BUFFER_SIZE + 1];
	int ret;
	unsigned char retry;
	struct i2c_msg msg;

	if (len > I2C_WRITE_BUFFER_SIZE) {
		dev_err(&tud->client->dev, "%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr = tud->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;

	mutex_lock(&tud->i2c_mutex);

	for (retry = 0; retry < TUSB_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(tud->client->adapter, &msg, 1);
		if (ret == 1)
			break;

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			dev_err(&tud->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
			tud->comm_err_count++;
		}
	}

	mutex_unlock(&tud->i2c_mutex);

	if (ret == 1)
		return 0;

	return -EIO;
}

int eusb_repeater_i2c_read(struct eusb_repeater_data *tud, u8 reg, u8 *data, int len)
{
	u8 buf[4];
	int ret;
	unsigned char retry;
	struct i2c_msg msg[2];

	if (!len) {
		dev_err(&tud->client->dev,
			"%s: I2c message length is wrong!\n", __func__);
		goto err;
	}

	buf[0] = reg;

	msg[0].addr = tud->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = tud->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	mutex_lock(&tud->i2c_mutex);

	for (retry = 0; retry < TUSB_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(tud->client->adapter, msg, 2);
		if (ret == 2)
			break;
		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			dev_err(&tud->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
			tud->comm_err_count++;
		}
	}

	mutex_unlock(&tud->i2c_mutex);

	return ret;
err:
	return -EIO;
}

static int eusb_repeater_write_reg(struct eusb_repeater_data *tud, u8 reg, u8 *data, int len)
{
	int ret = 0;

	ensure_eusb_repeater_ready(tud);
	ret = eusb_repeater_i2c_write(tud, reg, data, len);
	if (ret < 0) {
		dev_err(tud->dev, "%s: reg(0x%x), ret(%d)\n",
			__func__, reg, ret);
	}
	return ret;
}

static int eusb_repeater_read_reg(struct eusb_repeater_data *tud, u8 reg, u8 *data, int len)
{
	int ret = 0;

	ensure_eusb_repeater_ready(tud);
	ret = eusb_repeater_i2c_read(tud, reg, data, len);
	if (ret < 0) {
		dev_err(tud->dev, "%s: reg(0x%x), ret(%d)\n",
			__func__, reg, ret);
		return ret;
	}
	ret &= 0xff;

	return ret;
}

static int eusb_repeater_fill_tune_param(struct eusb_repeater_data *tud,
				struct device_node *node)
{
	struct device *dev = tud->dev;
	struct device_node *child = NULL;
	struct eusb_repeater_tune_param *tune_param;
	size_t size = sizeof(struct eusb_repeater_tune_param);
	int ret;
	u32 res[4];
	u32 idx = 0;
	const char *name;

	ret = of_property_read_u32_array(node, "repeater_tune_cnt", &res[0], 1);
	tud->tune_cnt = res[0];

	dev_dbg(dev, "%s repeater tune cnt = %d\n", __func__, res[0]);

	tune_param = devm_kzalloc(dev, size * (res[0] + 1), GFP_KERNEL);
	if (!tune_param)
		return -ENOMEM;

	tud->tune_param = tune_param;

	for_each_child_of_node(node, child) {
		ret = of_property_read_string(child, "tune_name", &name);
		if (ret == 0) {
			memcpy(tune_param[idx].name, name, strlen(name));
		} else {
			dev_err(dev, "failed to read tune name from %s node\n", child->name);
			return ret;
		}
		ret = of_property_read_u32_array(child, "tune_value", res, 4);
		if (ret == 0) {
			tud->tune_param[idx].reg = res[0];
			tud->tune_param[idx].value = res[1];
			tud->tune_param[idx].shift = res[2];
			tud->tune_param[idx].mask = res[3];
		} else {
			dev_err(dev, "failed to read tune value from %s node\n", child->name);
			return -EINVAL;
		}
		dev_dbg(dev, "%s, tune name = %s, param = 0x%x, 0x%x, 0x%x, 0x%x\n",
			 __func__, tud->tune_param[idx].name,
			 tud->tune_param[idx].reg, tud->tune_param[idx].value,
			 tud->tune_param[idx].shift, tud->tune_param[idx].mask);

		idx++;
	}

	tune_param[idx].value = EXYNOS_USB_TUNE_LAST;

	return 0;
}

static int eusb_repeater_ctrl(int value)
{
	struct eusb_repeater_data *tud = g_tud;
	int ret = 0;
	u8 read_data, write_data;

	ret = eusb_repeater_read_reg(tud, I2C_GLOBAL_CONFIG, &read_data, 1);
	if (ret < 0)
		goto err;

	write_data = value ? (read_data & ~REG_DISABLE_P1) : (read_data | REG_DISABLE_P1);
	ret = eusb_repeater_write_reg(tud, I2C_GLOBAL_CONFIG, &write_data, 1);
	if (ret < 0)
		goto err;

	ret = eusb_repeater_read_reg(tud, I2C_GLOBAL_CONFIG, &read_data, 1);
	if (ret < 0)
		goto err;

	dev_info(tud->dev, "%s Disabled mode, reg = %x\n", value ? "Exit" : "Enter", read_data);

	if (ret >= 0)
		tud->ctrl_sel_status = value;

	return ret;

err:

	dev_err(tud->dev, "Failed to %s Disabled state, ret:%d\n", value ? "Exit" : "Enter", ret);
	return ret;
}

static ssize_t
eusb_repeater_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct eusb_repeater_data *tud = dev_get_drvdata(dev);
	int len = 0, i;

	len += snprintf(buf + len, PAGE_SIZE, "\t==== Print Repeater Tune Value ====\n");
	len += snprintf(buf + len, PAGE_SIZE, "Tune value count : %d\n", tud->tune_cnt);

	for (i = 0; i < tud->tune_cnt; i++) {
		len += snprintf(buf + len, PAGE_SIZE, "%s\t\t\t: 0x%x\n",
				tud->tune_param[i].name,
				tud->tune_param[i].value);
	}

	return len;
}

static ssize_t
eusb_repeater_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	char tune_name[30];
	u32 tune_val;
	struct eusb_repeater_data *tud = dev_get_drvdata(dev);
	int ret, i;
	u8 read_data, write_data;

	if (sscanf(buf, "%29s %x", tune_name, &tune_val) != 2)
		return -EINVAL;

	if (tud->ctrl_sel_status == false) {
		ret = eusb_repeater_ctrl(true);
		if (ret < 0)
			return -EBUSY;
	}

	for (i = 0; i < tud->tune_cnt; i++) {
		if (!strncmp(tud->tune_param[i].name, tune_name,
			strlen(tud->tune_param[i].name))) {
			write_data = (u8)tune_val;
			ret = eusb_repeater_write_reg(tud, (u8)tud->tune_param[i].reg,
						 &write_data, 1);
			if (ret < 0)
				dev_info(&tud->client->dev, "%s: i2c write error\n", __func__);

			ret = eusb_repeater_read_reg(tud, (u8)tud->tune_param[i].reg,
						&read_data, 1);
			if (ret < 0)
				dev_info(&tud->client->dev, "%s: i2c read error\n", __func__);

			tud->tune_param[i].value = read_data;

			dev_info(&tud->client->dev, "%s, 0x%x = 0x%x\n", tud->tune_param[i].name,
				 tud->tune_param[i].reg, read_data);

		}
	}

	return n;
}
static DEVICE_ATTR_RW(eusb_repeater);

static struct attribute *eusb_repeater_attrs[] = {
	&dev_attr_eusb_repeater.attr,
	NULL
};
ATTRIBUTE_GROUPS(eusb_repeater);

/* -------------------------------- debugfs -------------------------------- */
static int u_tx_adjust_port1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, U_TX_ADJUST_PORT1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(u_tx_adjust_port1_fops, u_tx_adjust_port1_get, NULL, "0x%llx\n");

static int u_hs_tx_pre_emphasus_p1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, U_HS_TX_PRE_EMPHASIS_P1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(u_hs_tx_pre_emphasus_p1_fops, u_hs_tx_pre_emphasus_p1_get, NULL, "0x%llx\n");

static int u_rx_adjust_port1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, U_RX_ADJUST_PORT1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(u_rx_adjust_port1_fops, u_rx_adjust_port1_get, NULL, "0x%llx\n");

static int u_disconnect_squelch_port1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, U_DISCONNECT_SQUELCH_PORT1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(u_disconnect_squelch_port1_fops, u_disconnect_squelch_port1_get, NULL,
			"0x%llx\n");

static int e_hs_tx_pre_emphasus_p1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, E_HS_TX_PRE_EMPHASIS_P1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(e_hs_tx_pre_emphasus_p1_fops, e_hs_tx_pre_emphasus_p1_get, NULL, "0x%llx\n");

static int e_tx_adjust_port1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, E_TX_ADJUST_PORT1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(e_tx_adjust_port1_fops, e_tx_adjust_port1_get, NULL, "0x%llx\n");

static int e_rx_adjust_port1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, E_RX_ADJUST_PORT1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(e_rx_adjust_port1_fops, e_rx_adjust_port1_get, NULL, "0x%llx\n");

static int gpio0_config_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, GPIO0_CONFIG, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(gpio0_config_fops, gpio0_config_get, NULL, "0x%llx\n");

static int gpio1_config_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, GPIO1_CONFIG, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(gpio1_config_fops, gpio1_config_get, NULL, "0x%llx\n");

static int uart_port1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, UART_PORT1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}

static int uart_port1_set(void *p, u64 val)
{
	struct eusb_repeater_data *tud = p;
	u8 data = (u8) val;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	return eusb_repeater_write_reg(tud, UART_PORT1, &data, 1);
}
DEFINE_SIMPLE_ATTRIBUTE(uart_port1_fops, uart_port1_get, uart_port1_set, "0x%llx\n");

static int rev_id_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, REV_ID, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rev_id_fops, rev_id_get, NULL, "0x%llx\n");

static int global_config_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, I2C_GLOBAL_CONFIG, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}

static int global_config_set(void *p, u64 val)
{
	struct eusb_repeater_data *tud = p;
	u8 data = (u8) val;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	return eusb_repeater_write_reg(tud, I2C_GLOBAL_CONFIG, &data, 1);
}
DEFINE_SIMPLE_ATTRIBUTE(global_config_fops, global_config_get, global_config_set, "0x%llx\n");

static int int_enable_1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, INT_ENABLE_1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}

static int int_enable_1_set(void *p, u64 val)
{
	struct eusb_repeater_data *tud = p;
	u8 data = (u8) val;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	return eusb_repeater_write_reg(tud, INT_ENABLE_1, &data, 1);
}
DEFINE_SIMPLE_ATTRIBUTE(int_enable_1_fops, int_enable_1_get, int_enable_1_set, "0x%llx\n");

static int int_enable_2_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, INT_ENABLE_2, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}

static int int_enable_2_set(void *p, u64 val)
{
	struct eusb_repeater_data *tud = p;
	u8 data = (u8) val;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	return eusb_repeater_write_reg(tud, INT_ENABLE_2, &data, 1);
}
DEFINE_SIMPLE_ATTRIBUTE(int_enable_2_fops, int_enable_2_get, int_enable_2_set, "0x%llx\n");

static int bc_control_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, BC_CONTROL, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}

static int bc_control_set(void *p, u64 val)
{
	struct eusb_repeater_data *tud = p;
	u8 data = (u8) val;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	return eusb_repeater_write_reg(tud, BC_CONTROL, &data, 1);
}
DEFINE_SIMPLE_ATTRIBUTE(bc_control_fops, bc_control_get, bc_control_set, "0x%llx\n");

static int bc_status_1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, BC_STATUS_1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bc_status_1_fops, bc_status_1_get, NULL, "0x%llx\n");

static int int_status_1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, INT_STATUS_1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}

static int int_status_1_set(void *p, u64 val)
{
	struct eusb_repeater_data *tud = p;
	u8 data = (u8) val;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	return eusb_repeater_write_reg(tud, INT_STATUS_1, &data, 1);
}
DEFINE_SIMPLE_ATTRIBUTE(int_status_1_fops, int_status_1_get, int_status_1_set, "0x%llx\n");

static int int_status_2_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, INT_STATUS_2, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}

static int int_status_2_set(void *p, u64 val)
{
	struct eusb_repeater_data *tud = p;
	u8 data = (u8) val;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	return eusb_repeater_write_reg(tud, INT_STATUS_2, &data, 1);
}
DEFINE_SIMPLE_ATTRIBUTE(int_status_2_fops, int_status_2_get, int_status_2_set, "0x%llx\n");

static int config_port1_get(void *p, u64 *val)
{
	struct eusb_repeater_data *tud = p;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	ret = eusb_repeater_read_reg(tud, CONFIG_PORT1, &data, 1);
	if (ret < 0)
		return ret;

	*val = data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(config_port1_fops, config_port1_get, NULL, "0x%llx\n");

static int eusb_repeater_all_registers_show(struct seq_file *s, void *unused)
{
	struct eusb_repeater_data *tud = s->private;
	u8 data;
	int ret;

	if (!regulator_is_enabled(tud->vdd33))
		return -ESHUTDOWN;

	seq_puts(s, "----- dump all registers of eUSB repeater (0xff = i2c busy) -----\n");

	/* phy tune */
	ret = eusb_repeater_read_reg(tud, U_TX_ADJUST_PORT1, &data, 1);
	seq_printf(s, "reg u_tx_adjust_port1(%x): 0x%x\n", U_TX_ADJUST_PORT1,
		   (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, U_HS_TX_PRE_EMPHASIS_P1, &data, 1);
	seq_printf(s, "reg u_hs_tx_pre_emphasus_p1(%x): 0x%x\n", U_HS_TX_PRE_EMPHASIS_P1,
		   (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, U_RX_ADJUST_PORT1, &data, 1);
	seq_printf(s, "reg u_rx_adjust_port1(%x): 0x%x\n", U_RX_ADJUST_PORT1,
		   (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, U_DISCONNECT_SQUELCH_PORT1, &data, 1);
	seq_printf(s, "reg u_disconnect_squelch_port1(%x): 0x%x\n", U_DISCONNECT_SQUELCH_PORT1,
		   (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, E_HS_TX_PRE_EMPHASIS_P1, &data, 1);
	seq_printf(s, "reg e_hs_tx_pre_emphasus_p1(%x): 0x%x\n", E_HS_TX_PRE_EMPHASIS_P1,
		   (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, E_TX_ADJUST_PORT1, &data, 1);
	seq_printf(s, "reg e_tx_adjust_port1(%x): 0x%x\n", E_TX_ADJUST_PORT1,
		   (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, E_RX_ADJUST_PORT1, &data, 1);
	seq_printf(s, "reg e_rx_adjust_port1(%x): 0x%x\n", E_RX_ADJUST_PORT1,
		   (ret >= 0) ? data : 0xff);

	/* gpio config */
	ret = eusb_repeater_read_reg(tud, GPIO0_CONFIG, &data, 1);
	seq_printf(s, "reg gpio0_config(%x): 0x%x\n", GPIO0_CONFIG, (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, GPIO1_CONFIG, &data, 1);
	seq_printf(s, "reg gpio1_config(%x): 0x%x\n", GPIO1_CONFIG, (ret >= 0) ? data : 0xff);

	/* uart port */
	ret = eusb_repeater_read_reg(tud, UART_PORT1, &data, 1);
	seq_printf(s, "reg uart_port1(%x): 0x%x\n", UART_PORT1, (ret >= 0) ? data : 0xff);

	/* rev id */
	ret = eusb_repeater_read_reg(tud, REV_ID, &data, 1);
	seq_printf(s, "reg rev_id(%x): 0x%x\n", REV_ID, (ret >= 0) ? data : 0xff);

	/* i2c global config */
	ret = eusb_repeater_read_reg(tud, I2C_GLOBAL_CONFIG, &data, 1);
	seq_printf(s, "reg global_config(%x): 0x%x\n", I2C_GLOBAL_CONFIG, (ret >= 0) ? data : 0xff);

	/* INT enable */
	ret = eusb_repeater_read_reg(tud, INT_ENABLE_1, &data, 1);
	seq_printf(s, "reg int_enable_1(%x): 0x%x\n", INT_ENABLE_1, (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, INT_ENABLE_2, &data, 1);
	seq_printf(s, "reg int_enable_2(%x): 0x%x\n", INT_ENABLE_2, (ret >= 0) ? data : 0xff);

	/* bc control */
	ret = eusb_repeater_read_reg(tud, BC_CONTROL, &data, 1);
	seq_printf(s, "reg bc_control(%x): 0x%x\n", BC_CONTROL, (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, BC_STATUS_1, &data, 1);
	seq_printf(s, "reg bc_status_1(%x): 0x%x\n", BC_STATUS_1, (ret >= 0) ? data : 0xff);

	/* INT status */
	ret = eusb_repeater_read_reg(tud, INT_STATUS_1, &data, 1);
	seq_printf(s, "reg int_status_1(%x): 0x%x\n", INT_STATUS_1, (ret >= 0) ? data : 0xff);
	ret = eusb_repeater_read_reg(tud, INT_STATUS_2, &data, 1);
	seq_printf(s, "reg int_status_2(%x): 0x%x\n", INT_STATUS_2, (ret >= 0) ? data : 0xff);

	/* config port */
	ret = eusb_repeater_read_reg(tud, CONFIG_PORT1, &data, 1);
	seq_printf(s, "reg config_port1(%x): 0x%x\n", CONFIG_PORT1, (ret >= 0) ? data : 0xff);

	return 0;
}

static int eusb_repeater_all_registers_open(struct inode *inode, struct file *file)
{
        return single_open(file, eusb_repeater_all_registers_show, inode->i_private);
}

static const struct file_operations dump_all_registers_fops = {
        .open = eusb_repeater_all_registers_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

/* ------------------------------------------------------------------------- */

static int eusb_repeater_parse_dt(struct device *dev, struct eusb_repeater_data *tud,
				     struct eusb_repeater_plat_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct device_node *tune_node;
	int ret = 0;

	if (np == NULL) {
		dev_err(dev, "%s np is NULL\n", __func__);
		return -1;
	}

	tune_node = of_parse_phandle(dev->of_node, "repeater_tune_param", 0);
	if (tune_node != NULL) {
		ret = eusb_repeater_fill_tune_param(tud, tune_node);
		if (ret < 0) {
			dev_err(dev, "can't fill repeater tuning param\n");
			return -EINVAL;
		}
	} else
		dev_info(dev, "don't need repeater tuning param\n");

	tud->vdd33 = devm_regulator_get(dev, "vdd33");
	if (IS_ERR(tud->vdd33)) {
		dev_err(dev, "vdd33 regulator_get fail: %ld\n", PTR_ERR(tud->vdd33));
		ret = PTR_ERR(tud->vdd33);
		tud->vdd33 = NULL;
	}

	return ret;
}

static const struct of_device_id eusb_repeater_match_table[] = {
		{ .compatible = "samsung,eusb-repeater",},
		{},
};
MODULE_DEVICE_TABLE(of, eusb_repeater_match_table);

void eusb_repeater_update_usb_state(bool on)
{
	struct eusb_repeater_data *tud = g_tud;

	if (!tud)
		return;

	mutex_lock(&tud->mutex);
	if (on)
		tud->start_time = ktime_get();
	tud->ready = false;
	tud->eusb_data_enabled = (extcon_get_state(tud->edev, EXTCON_USB) > 0 ||
				  extcon_get_state(tud->edev, EXTCON_USB_HOST) > 0);
	mutex_unlock(&tud->mutex);

	if (on && tud->eusb_pm_status && !tud->eusb_data_enabled)
		eusb_repeater_ctrl(false);

	return;
}
EXPORT_SYMBOL_GPL(eusb_repeater_update_usb_state);

int eusb_repeater_power_off(void)
{
	struct eusb_repeater_data *tud = g_tud;

	if (!tud)
		return -EEXIST;

	tud->eusb_data_enabled = false;

	return eusb_repeater_ctrl(false);
}
EXPORT_SYMBOL_GPL(eusb_repeater_power_off);

int eusb_repeater_power_on(void)
{
	struct eusb_repeater_data *tud = g_tud;
	u8 read_data, write_data, shift, mask;
	int ret, i;

	if (!tud)
		return -EEXIST;

	ret = eusb_repeater_ctrl(true);
	if (ret < 0)
		goto err;

	ret = eusb_repeater_read_reg(tud, 0x50,
				&read_data, 1);
	if (ret < 0) {
		dev_info(tud->dev, "%s: i2c read error\n", __func__);
		goto err;
	}
	dev_info(tud->dev, "usb: %s : check eusb reg(0x50) = %x\n",
		__func__, read_data);

	for (i = 0; i < tud->tune_cnt; i++) {
		ret = eusb_repeater_read_reg(tud, tud->tune_param[i].reg,
					     &read_data, 1);
		if (ret < 0) {
			dev_err(tud->dev, "%s: i2c read error\n", __func__);
			goto err;
		}
		write_data = (u8)tud->tune_param[i].value;
		shift = (u8)tud->tune_param[i].shift;
		mask = (u8)tud->tune_param[i].mask;
		write_data = (read_data & ~(mask << shift)) | ((write_data & mask) << shift);

		ret = eusb_repeater_write_reg(tud, (u8)tud->tune_param[i].reg,
					      &write_data, 1);
		if (ret < 0) {
			dev_err(tud->dev, "%s: i2c write error\n", __func__);
			goto err;
		}

		ret = eusb_repeater_read_reg(tud, tud->tune_param[i].reg,
					     &read_data, 1);
		if (ret < 0) {
			dev_err(tud->dev, "%s: i2c read error\n", __func__);
			goto err;
		}
		tud->tune_param[i].value = (read_data >> shift) & mask;
		dev_dbg(tud->dev, "%s, %s: 0x%x=0x%x\n",  __func__,
			 tud->tune_param[i].name, tud->tune_param[i].reg,
			 tud->tune_param[i].value);

	}

	tud->eusb_data_enabled = true;

	return 0;
err:
	return ret;

}
EXPORT_SYMBOL_GPL(eusb_repeater_power_on);

static int eusb_repeater_debugfs_init(struct eusb_repeater_data *tud)
{
	tud->root = debugfs_create_dir("eusb_repeater", 0);
	if (IS_ERR_OR_NULL(tud->root))
		return -EINVAL;

	debugfs_create_file("u_tx_adjust_port1", 0444, tud->root, tud, &u_tx_adjust_port1_fops);
	debugfs_create_file("u_hs_tx_pre_emphasus_p1", 0444, tud->root, tud,
			    &u_hs_tx_pre_emphasus_p1_fops);
	debugfs_create_file("u_rx_adjust_port1", 0444, tud->root, tud, &u_rx_adjust_port1_fops);
	debugfs_create_file("u_disconnect_squelch_port1", 0444, tud->root, tud,
			    &u_disconnect_squelch_port1_fops);
	debugfs_create_file("e_hs_tx_pre_emphasus_p1", 0444, tud->root, tud, &e_hs_tx_pre_emphasus_p1_fops);
	debugfs_create_file("e_tx_adjust_port1", 0444, tud->root, tud, &e_tx_adjust_port1_fops);
	debugfs_create_file("e_rx_adjust_port1", 0444, tud->root, tud, &e_rx_adjust_port1_fops);

	debugfs_create_file("gpio0_config", 0444, tud->root, tud, &gpio0_config_fops);
	debugfs_create_file("gpio1_config", 0444, tud->root, tud, &gpio1_config_fops);

	debugfs_create_file("uart_port1", 0644, tud->root, tud, &uart_port1_fops);

	debugfs_create_file("rev_id", 0444, tud->root, tud, &rev_id_fops);

	debugfs_create_file("global_config", 0644, tud->root, tud, &global_config_fops);

	debugfs_create_file("int_enable_1", 0644, tud->root, tud, &int_enable_1_fops);
	debugfs_create_file("int_enable_2", 0644, tud->root, tud, &int_enable_2_fops);

	debugfs_create_file("bc_control", 0644, tud->root, tud, &bc_control_fops);
	debugfs_create_file("bc_status_1", 0444, tud->root, tud, &bc_status_1_fops);

	debugfs_create_file("int_status_1", 0644, tud->root, tud, &int_status_1_fops);
	debugfs_create_file("int_status_2", 0644, tud->root, tud, &int_status_2_fops);

	debugfs_create_file("config_port1", 0444, tud->root, tud, &config_port1_fops);

	debugfs_create_file("registers", 0444, tud->root, tud, &dump_all_registers_fops);
	return 0;
}

static void eusb_repeater_debugfs_remove(struct eusb_repeater_data *tud)
{
	if (!tud->root)
		return;

	debugfs_remove_recursive(tud->root);
	tud->root = NULL;
	return;
}

static int eusb_repeater_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *of_node = client->dev.of_node;
	struct eusb_repeater_data *tud;
	struct eusb_repeater_plat_data *pdata = client->dev.platform_data;
	int ret = 0;

	tud = kzalloc(sizeof(*tud), GFP_KERNEL);
	if (tud == NULL) {
		dev_err(&client->dev, "%s: couldn't allocate eusb_repeater_data\n", __func__);
		ret = -ENOMEM;
		goto err_repeater_nomem;
	}
	tud->dev = &client->dev;

	tud->edev = extcon_get_edev_by_phandle(tud->dev, 0);
	if (IS_ERR_OR_NULL(tud->edev)) {
		dev_err(&client->dev, "%s: couldn't get extcon\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_repeater_func;
	}

	if (of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err_repeater_func;
		}

		ret = eusb_repeater_parse_dt(&client->dev, tud, pdata);
		if (ret < 0)
			goto err_parse_dt;
	} else {
		pdata = client->dev.platform_data;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: EIO err!\n", __func__);
		ret = -EIO;
		goto err_parse_dt;
	}

	tud->client = client;
	tud->pdata = pdata;
	g_tud = tud;

	ret = eusb_repeater_debugfs_init(tud);
	if (ret < 0)
		dev_err(&client->dev, "failed to init debugfs nodes, ret: %d\n", ret);

	tud->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(tud->pinctrl)) {
		dev_err(&client->dev, "failed to allocate pinctrl ret: %ld\n", PTR_ERR(tud->pinctrl));
		ret = PTR_ERR(tud->pinctrl);
		goto err_pinctrl;
	} else {
		tud->init_state = pinctrl_lookup_state(tud->pinctrl, "init_state");
		if (IS_ERR(tud->init_state)) {
			dev_err(&client->dev, "failed to allocate pinctrl ret: %ld\n",
				PTR_ERR(tud->init_state));
			ret = PTR_ERR(tud->init_state);
			tud->pinctrl = NULL;
			tud->init_state = NULL;
			goto err_pinctrl;
		} else
			pinctrl_select_state(tud->pinctrl, tud->init_state);
	}

	i2c_set_clientdata(client, tud);
	mutex_init(&tud->mutex);
	mutex_init(&tud->i2c_mutex);

	/*
	 * eUSB repeater control should be done with ldo on.
	 * So control will be done according to utmi_init/exit.
	 * eusb_repeater_power_on();
	 */

	tud->eusb_pm_status = true;
	return 0;

err_pinctrl:
	eusb_repeater_debugfs_remove(tud);
err_parse_dt:
	devm_kfree(&client->dev, pdata);
err_repeater_func:
	kfree(tud);
err_repeater_nomem:
	dev_err(&client->dev, "%s: err = %d\n", __func__, ret);

	return ret;
}

static const struct i2c_device_id eusb_repeater_id[] = {
	{"eusb-repeater", 0},
	{}
};

static void eusb_repeater_shutdown(struct i2c_client *client)
{
}

static void eusb_repeater_remove(struct i2c_client *client)
{
	struct eusb_repeater_data *tud = i2c_get_clientdata(client);
	mutex_destroy(&tud->i2c_mutex);
}


#if IS_ENABLED(CONFIG_PM)
static int eusb_repeater_suspend(struct device *dev)
{
	struct eusb_repeater_data *tud = dev_get_drvdata(dev);

	tud->eusb_pm_status = false;
	return 0;
}

static int eusb_repeater_resume(struct device *dev)
{
	struct eusb_repeater_data *tud = dev_get_drvdata(dev);

	tud->eusb_pm_status = true;

        if (regulator_is_enabled(tud->vdd33) && !tud->eusb_data_enabled)
		eusb_repeater_ctrl(false);

	return 0;
}
#else
#define eusb_repeater_suspend NULL
#define eusb_repeater_resume NULL
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops eusb_repeater_pm_ops = {
	.suspend = eusb_repeater_suspend,
	.resume = eusb_repeater_resume,
};
#endif

static struct i2c_driver eusb_repeater_driver = {
	.probe  = eusb_repeater_probe,
	.remove = eusb_repeater_remove,
	.shutdown   = eusb_repeater_shutdown,
	.id_table   = eusb_repeater_id,
	.driver = {
		.name = "eusb-repeater",
		.owner = THIS_MODULE,
		.dev_groups = eusb_repeater_groups,
		.pm = &eusb_repeater_pm_ops,
		.of_match_table = eusb_repeater_match_table,
	},
};

static int __init eusb_repeater_init(void)
{
	return i2c_add_driver(&eusb_repeater_driver);
}

static void __exit eusb_repeater_exit(void)
{
	i2c_del_driver(&eusb_repeater_driver);
}
module_init(eusb_repeater_init);
module_exit(eusb_repeater_exit);

MODULE_DESCRIPTION("Samsung eUSB Repeater Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");

