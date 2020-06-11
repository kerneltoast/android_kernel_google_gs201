// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/regulator/s2mpg11-powermeter.c
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <../drivers/pinctrl/samsung/pinctrl-samsung.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#include <linux/mfd/samsung/s2mpg1x-meter.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/regulator/pmic_class.h>
#include <linux/mfd/core.h>

static void s2mpg11_meter_set_acc_mode(struct s2mpg11_meter *s2mpg11,
				       s2mpg1x_meter_mode mode);
static void s2mpg11_meter_read_acc_data_reg(struct s2mpg11_meter *s2mpg11,
					    u64 *data);
static void s2mpg11_meter_read_acc_count(struct s2mpg11_meter *s2mpg11,
					 u32 *count);

#if IS_ENABLED(CONFIG_ODPM)
static struct mfd_cell s2mpg11_meter_devs[] = {
	{
		.name = "s2mpg11-odpm",
	},
};
#endif

int s2mpg11_meter_ext_channel_onoff(struct s2mpg11_meter *s2mpg11, u8 channels)
{
	return s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_METER_CTRL2,
				  channels << EXT_METER_CHANNEL_EN_OFFSET,
				  EXT_METER_CHANNEL_EN_MASK);
}
EXPORT_SYMBOL_GPL(s2mpg11_meter_ext_channel_onoff);

/**
 * Load measurement into registers and read measurement from the registers
 *
 * Note: data must be an array with length S2MPG1X_METER_CHANNEL_MAX
 */
int s2mpg11_meter_load_measurement(struct s2mpg11_meter *s2mpg11,
				   s2mpg1x_meter_mode mode, u64 *data,
				   u32 *count, unsigned long *jiffies_capture)
{
	mutex_lock(&s2mpg11->meter_lock);

	s2mpg11_meter_set_acc_mode(s2mpg11, mode);

	s2mpg11_meter_set_async_blocking(s2mpg11, jiffies_capture);

	if (data)
		s2mpg11_meter_read_acc_data_reg(s2mpg11, data);

	if (count)
		s2mpg11_meter_read_acc_count(s2mpg11, count);

	mutex_unlock(&s2mpg11->meter_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg11_meter_load_measurement);

static u64 muxsel_to_current_resolution(s2mpg11_meter_muxsel m)
{
	switch (m) {
	case BUCK3S:
	case BUCK4S:
	case BUCK5S:
	case BUCK6S:
	case BUCK8S:
	case BUCK9S:
	case BUCK10S:
		return CMS_BUCK_CURRENT;
	case BUCK1S:
		return CMD_BUCK_CURRENT;
	case BUCK2S:
		return CMT_BUCK_CURRENT;
	case BUCK7S:
	case BUCKD:
	case BUCKA:
		return VM_CURRENT;
	case BUCKBOOST:
		return BB_CURRENT;
	case LDO2S:
		return DVS_NLDO_CURRENT_150mA;
	case LDO9S:
		return NLDO_CURRENT_150mA;
	case LDO3S:
	case LDO4S:
	case LDO7S:
	case LDO11S:
	case LDO13S:
	case LDO15S:
		return PLDO_CURRENT_150mA;
	case LDO6S:
	case LDO10S:
	case LDO12S:
	case LDO14S:
		return PLDO_CURRENT_300mA;
	case LDO5S:
		return NLDO_CURRENT_450mA;
	case LDO1S:
		return DVS_NLDO_CURRENT_800mA;
	case LDO8S:
		return NLDO_CURRENT_1000mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}

u32 s2mpg11_muxsel_to_power_resolution(s2mpg11_meter_muxsel m)
{
	switch (m) {
	case BUCK3S:
	case BUCK4S:
	case BUCK5S:
	case BUCK6S:
	case BUCK8S:
	case BUCK9S:
	case BUCK10S:
		return CMS_BUCK_POWER;
	case BUCK1S:
		return CMD_BUCK_POWER;
	case BUCK2S:
		return CMT_BUCK_POWER;
	case BUCK7S:
	case BUCKD:
	case BUCKA:
		return VM_POWER;
	case BUCKBOOST:
		return BB_POWER;
	case LDO2S:
		return DVS_NLDO_POWER_150mA;
	case LDO9S:
		return NLDO_POWER_150mA;
	case LDO3S:
	case LDO4S:
	case LDO7S:
	case LDO11S:
	case LDO13S:
	case LDO15S:
		return PLDO_POWER_150mA;
	case LDO6S:
	case LDO10S:
	case LDO12S:
	case LDO14S:
		return PLDO_POWER_300mA;
	case LDO5S:
		return NLDO_POWER_450mA;
	case LDO1S:
		return DVS_NLDO_POWER_800mA;
	case LDO8S:
		return NLDO_POWER_1000mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}
EXPORT_SYMBOL_GPL(s2mpg11_muxsel_to_power_resolution);

static const char *muxsel_to_str(s2mpg11_meter_muxsel m)
{
	char *ret;

	switch (m) {
		ENUM_STR(BUCK1S, ret);
		ENUM_STR(BUCK2S, ret);
		ENUM_STR(BUCK3S, ret);
		ENUM_STR(BUCK4S, ret);
		ENUM_STR(BUCK5S, ret);
		ENUM_STR(BUCK6S, ret);
		ENUM_STR(BUCK7S, ret);
		ENUM_STR(BUCK8S, ret);
		ENUM_STR(BUCK9S, ret);
		ENUM_STR(BUCK10S, ret);
		ENUM_STR(BUCKD, ret);
		ENUM_STR(BUCKA, ret);
		ENUM_STR(BUCKBOOST, ret);
		ENUM_STR(VSEN_P4, ret);
		ENUM_STR(VSEN_P5, ret);
		ENUM_STR(VSEN_P6, ret);
		ENUM_STR(LDO1S, ret);
		ENUM_STR(LDO2S, ret);
		ENUM_STR(LDO3S, ret);
		ENUM_STR(LDO4S, ret);
		ENUM_STR(LDO5S, ret);
		ENUM_STR(LDO6S, ret);
		ENUM_STR(LDO7S, ret);
		ENUM_STR(LDO8S, ret);
		ENUM_STR(LDO9S, ret);
		ENUM_STR(LDO10S, ret);
		ENUM_STR(LDO11S, ret);
		ENUM_STR(LDO12S, ret);
		ENUM_STR(LDO13S, ret);
		ENUM_STR(LDO14S, ret);
		ENUM_STR(LDO15S, ret);
		ENUM_STR(VSEN_C4, ret);
		ENUM_STR(VSEN_C5, ret);
		ENUM_STR(VSEN_C6, ret);
	default:
		return "invalid";
	}
	return ret;
}

int s2mpg11_meter_onoff(struct s2mpg11_meter *s2mpg11, bool onoff)
{
	int ret;

	if (onoff) {
		pr_info("%s: s2mpg11 meter on\n", __func__);
		ret = s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_METER_CTRL1,
					 METER_EN_MASK, METER_EN_MASK);
		s2mpg11->meter_en = 1;
	} else {
		pr_info("%s: s2mpg11 meter off\n", __func__);
		ret = s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_METER_CTRL1, 0,
					 METER_EN_MASK);
		s2mpg11->meter_en = 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg11_meter_onoff);

int s2mpg11_ext_meter_onoff(struct s2mpg11_meter *s2mpg11, bool onoff)
{
	int ret;

	if (onoff) {
		pr_info("%s: s2mpg11 external meter on\n", __func__);
		ret = s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_METER_CTRL1,
					 EXT_METER_EN_MASK, EXT_METER_EN_MASK);
	} else {
		pr_info("%s: s2mpg11 external meter off\n", __func__);
		ret = s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_METER_CTRL1, 0,
					 EXT_METER_EN_MASK);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg11_ext_meter_onoff);

int s2mpg11_set_int_samp_rate(struct s2mpg11_meter *s2mpg11,
			      s2mpg1x_int_samp_rate hz)
{
	return s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_METER_CTRL1,
				  hz << INT_SAMP_RATE_SHIFT,
				  INT_SAMP_RATE_MASK);
}
EXPORT_SYMBOL_GPL(s2mpg11_set_int_samp_rate);

int s2mpg11_set_ext_samp_rate(struct s2mpg11_meter *s2mpg11,
			      s2mpg1x_ext_samp_rate hz)
{
	return s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_METER_CTRL2, hz,
				  EXT_SAMP_RATE_MASK);
}
EXPORT_SYMBOL_GPL(s2mpg11_set_ext_samp_rate);

int s2mpg11_meter_set_muxsel(struct s2mpg11_meter *s2mpg11, int channel,
			     s2mpg11_meter_muxsel m)
{
	int reg = S2MPG11_METER_MUXSEL0;
	int ret = -EPERM;

	if (channel < 0 || channel >= S2MPG1X_METER_CHANNEL_MAX) {
		pr_err("%s: invalid channel number\n", __func__);
		return ret;
	}

	pr_info("%s: CH%d, %s\n", __func__, channel, muxsel_to_str(m));

	reg += channel;

	mutex_lock(&s2mpg11->meter_lock);
	ret = s2mpg11_update_reg(s2mpg11->i2c, reg, m, MUXSEL_MASK);

	s2mpg11->chg_mux_sel[channel] = m;
	mutex_unlock(&s2mpg11->meter_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg11_meter_set_muxsel);

static void s2mpg11_meter_set_lpf_mode(struct s2mpg11_meter *s2mpg11,
				       s2mpg1x_meter_mode mode)
{
	switch (mode) {
	case S2MPG1X_METER_POWER:
		s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_METER_CTRL5, 0x00);
		break;
	case S2MPG1X_METER_CURRENT:
		s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_METER_CTRL5, 0xFF);
		break;
	}
}

static void s2mpg11_meter_read_lpf_data_reg(struct s2mpg11_meter *s2mpg11)
{
	int i;
	u8 data[S2MPG1X_METER_LPF_BUF];
	u8 reg = S2MPG11_METER_LPF_DATA_CH0_1; /* first lpf data register */

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg11_bulk_read(s2mpg11->i2c, reg, S2MPG1X_METER_LPF_BUF,
				  data);
		s2mpg11->lpf_data[i] =
			data[0] + (data[1] << 8) + ((data[2] & 0x1F) << 16);
		reg += S2MPG1X_METER_LPF_BUF;
	}
}

static void s2mpg11_meter_set_acc_mode(struct s2mpg11_meter *s2mpg11,
				       s2mpg1x_meter_mode mode)
{
	switch (mode) {
	case S2MPG1X_METER_POWER:
		s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_METER_CTRL4, 0x00);
		break;
	case S2MPG1X_METER_CURRENT:
		s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_METER_CTRL4, 0xFF);
		break;
	}
}

int s2mpg11_meter_set_async_blocking(struct s2mpg11_meter *s2mpg11,
				     unsigned long *jiffies_capture)
{
	return s2mpg1x_meter_set_async_blocking(ID_S2MPG11, s2mpg11->i2c,
						jiffies_capture,
						S2MPG11_METER_CTRL2);
}
EXPORT_SYMBOL_GPL(s2mpg11_meter_set_async_blocking);

static void s2mpg11_meter_read_acc_data_reg(struct s2mpg11_meter *s2mpg11,
					    u64 *data)
{
	int i;
	u8 buf[S2MPG1X_METER_ACC_BUF];
	u8 reg = S2MPG11_METER_ACC_DATA_CH0_1; /* first acc data register */

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg11_bulk_read(s2mpg11->i2c, reg, S2MPG1X_METER_ACC_BUF,
				  buf);

		/* 41 bits of data */
		data[i] = ((u64)buf[0] << 0) | ((u64)buf[1] << 8) |
			  ((u64)buf[2] << 16) | ((u64)buf[3] << 24) |
			  ((u64)buf[4] << 32) | (((u64)buf[5] & 0x1) << 8);

		reg += S2MPG1X_METER_ACC_BUF;
	}
}

static void s2mpg11_meter_read_acc_count(struct s2mpg11_meter *s2mpg11,
					 u32 *count)
{
	u8 data[S2MPG1X_METER_COUNT_BUF]; /* ACC_COUNT is 20-bit data */

	s2mpg11_bulk_read(s2mpg11->i2c, S2MPG11_METER_ACC_COUNT_1,
			  S2MPG1X_METER_COUNT_BUF, data);

	*count = data[0] | (data[1] << 8) | ((data[2] & 0x0F) << 16);
}

static void s2mpg11_meter_set_ntc_mode(struct s2mpg11_meter *s2mpg11,
				       bool onoff)
{
	/* in EVT0, ADC_EN should be set as '1' */
	/* if any channel of NTC is set to be '1' */
	if (onoff && !s2mpg11->meter_en &&
	    s2mpg11->iodev->pmic_rev == S2MPG11_EVT0)
		s2mpg11_meter_onoff(s2mpg11, true);

	if (onoff) {
		/* disable IC power shutdown reaching NTC_H_WARN threshold */
		if (s2mpg11->iodev->pmic_rev == S2MPG11_EVT0)
			s2mpg11_write_reg(s2mpg11->trim, 0xEA, 0x02);
		s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_METER_CTRL3, 0x37);
	} else {
		s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_METER_CTRL3, 0x00);
	}
}

static void s2mpg11_set_ntc_samp_rate(struct s2mpg11_meter *s2mpg11,
				      unsigned int hz)
{
	s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_METER_CTRL1,
			   hz << NTC_SAMP_RATE_SHIFT, NTC_SAMP_RATE_MASK);
}

static void s2mpg11_meter_read_ntc_data_reg(struct s2mpg11_meter *s2mpg11)
{
	int i;
	u8 data[S2MPG11_METER_NTC_BUF];
	u8 reg = S2MPG11_METER_LPF_DATA_NTC0_1;	/* first ntc data register */

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg11_bulk_read(s2mpg11->i2c, reg, S2MPG11_METER_NTC_BUF,
				  data);
		s2mpg11->ntc_data[i] = data[0] + ((data[1] & 0xf) << 8);
		reg += S2MPG11_METER_NTC_BUF;
	}
}

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
static ssize_t s2mpg11_muxsel_table_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int muxsel_cnt = 0;
	int muxsel = BUCK1S;
	size_t count = 0;

	while (muxsel <= VSEN_C6) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"%s : 0x%x , ",  muxsel_to_str(muxsel), muxsel);

		if (muxsel == BUCKBOOST)
			muxsel = VSEN_P4;
		else if (muxsel == VSEN_P6)
			muxsel = LDO1S;
		else if (muxsel == LDO15S)
			muxsel = VSEN_C4;
		else if (muxsel == VSEN_C6)
			break;
		else
			muxsel++;

		muxsel_cnt++;
		if (!(muxsel_cnt % 8))
			count += scnprintf(buf + count,
				PAGE_SIZE - count, "\n");
	}

	return count;
}

static ssize_t s2mpg11_channel_muxsel_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct s2mpg11_meter *s2mpg11 = dev_get_drvdata(dev);
	int channel, muxsel;
	int ret;

	if (!buf) {
		pr_err("%s: empty buffer\n", __func__);
		return -EINVAL;
	}

	ret = sscanf(buf, "%x %x", &channel, &muxsel);
	if (ret != 2) {
		pr_err("%s: input error\n", __func__);
		return -EINVAL;
	}

	if (channel < 0 || channel > 7) {
		pr_err("%s: wrong channel %d\n", __func__, channel);
		return -EINVAL;
	}

	if ((muxsel >= BUCK1S && muxsel <= BUCKBOOST) ||
	    (muxsel >= VSEN_P4 && muxsel <= VSEN_P6) ||
	    (muxsel >= LDO1S && muxsel <= LDO15S) ||
	    (muxsel >= VSEN_C4 && muxsel <= VSEN_C6)) {
		s2mpg11_meter_set_muxsel(s2mpg11, channel, muxsel);
	} else {
		pr_err("%s: wrong muxsel 0x%x\n", __func__, muxsel);
		return -EINVAL;
	}

	return size;
}

static ssize_t s2mpg11_channel_muxsel_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct s2mpg11_meter *s2mpg11 = dev_get_drvdata(dev);
	int i;
	size_t count = 0;

	mutex_lock(&s2mpg11->meter_lock);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "CH%d[%s], ",
			 i, muxsel_to_str(s2mpg11->chg_mux_sel[i]));
	}

	mutex_unlock(&s2mpg11->meter_lock);
	return count;
}

static ssize_t s2mpg11_lpf_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg11_meter *s2mpg11 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg11->meter_lock);

	s2mpg11_meter_set_lpf_mode(s2mpg11, S2MPG1X_METER_CURRENT);
	s2mpg11_meter_read_lpf_data_reg(s2mpg11);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg11_meter_muxsel muxsel = s2mpg11->chg_mux_sel[i];

		count += s2mpg1x_format_meter_channel(buf, count, i,
						      muxsel_to_str(muxsel),
						      "(mA)",
						      s2mpg11->lpf_data[i],
						      muxsel_to_current_resolution(muxsel),
						      1);
	}
	mutex_unlock(&s2mpg11->meter_lock);
	return count;
}

static ssize_t s2mpg11_lpf_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg11_meter *s2mpg11 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg11->meter_lock);

	s2mpg11_meter_set_lpf_mode(s2mpg11, S2MPG1X_METER_POWER);
	s2mpg11_meter_read_lpf_data_reg(s2mpg11);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg11_meter_muxsel muxsel = s2mpg11->chg_mux_sel[i];

		count += s2mpg1x_format_meter_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			s2mpg11->lpf_data[i],
			s2mpg11_muxsel_to_power_resolution(muxsel), 1);
	}
	mutex_unlock(&s2mpg11->meter_lock);
	return count;
}

static ssize_t s2mpg11_acc_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg11_meter *s2mpg11 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1X_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg11_meter_load_measurement(s2mpg11, S2MPG1X_METER_CURRENT, acc_data,
				       &acc_count, NULL);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg11_meter_muxsel muxsel = s2mpg11->chg_mux_sel[i];

		count += s2mpg1x_format_meter_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mA)",
			acc_data[i], muxsel_to_current_resolution(muxsel),
			acc_count);
	}

	return count;
}

static ssize_t s2mpg11_acc_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg11_meter *s2mpg11 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1X_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg11_meter_load_measurement(s2mpg11, S2MPG1X_METER_POWER, acc_data,
				       &acc_count, NULL);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg11_meter_muxsel muxsel = s2mpg11->chg_mux_sel[i];

		count += s2mpg1x_format_meter_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			acc_data[i], s2mpg11_muxsel_to_power_resolution(muxsel),
			acc_count);
	}

	return count;
}

static ssize_t s2mpg11_ntc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s2mpg11_meter *s2mpg11 = dev_get_drvdata(dev);
	int i;
	size_t count = 0;

	mutex_lock(&s2mpg11->meter_lock);

	s2mpg11_meter_set_ntc_mode(s2mpg11, true);
	s2mpg11_meter_read_ntc_data_reg(s2mpg11);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"NTC_CH%d: 0x%x\n", i, s2mpg11->ntc_data[i]);
	}
	mutex_unlock(&s2mpg11->meter_lock);
	return count;
}

static DEVICE_ATTR_RO(s2mpg11_muxsel_table);
static DEVICE_ATTR_RW(s2mpg11_channel_muxsel);
static DEVICE_ATTR_RO(s2mpg11_lpf_current);
static DEVICE_ATTR_RO(s2mpg11_lpf_power);
static DEVICE_ATTR_RO(s2mpg11_acc_current);
static DEVICE_ATTR_RO(s2mpg11_acc_power);
static DEVICE_ATTR_RO(s2mpg11_ntc);

int create_s2mpg11_meter_sysfs(struct s2mpg11_meter *s2mpg11)
{
	struct device *s2mpg11_meter_dev = s2mpg11->dev;
	int err = -ENODEV;

	pr_info("%s: s2mpg11 meter sysfs start\n", __func__);

	s2mpg11_meter_dev = pmic_device_create(s2mpg11, "s2mpg11-meter");

	err = device_create_file(s2mpg11_meter_dev,
				 &dev_attr_s2mpg11_lpf_current);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_lpf_current.attr.name);
	}

	err = device_create_file(s2mpg11_meter_dev,
				 &dev_attr_s2mpg11_lpf_power);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_lpf_power.attr.name);
	}

	err = device_create_file(s2mpg11_meter_dev,
				 &dev_attr_s2mpg11_acc_current);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_acc_current.attr.name);
	}

	err = device_create_file(s2mpg11_meter_dev,
				 &dev_attr_s2mpg11_acc_power);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_acc_power.attr.name);
	}

	err = device_create_file(s2mpg11_meter_dev, &dev_attr_s2mpg11_ntc);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_ntc.attr.name);
	}

	err = device_create_file(s2mpg11_meter_dev,
				 &dev_attr_s2mpg11_muxsel_table);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_muxsel_table.attr.name);
	}

	err = device_create_file(s2mpg11_meter_dev,
				 &dev_attr_s2mpg11_channel_muxsel);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_channel_muxsel.attr.name);
	}

	return 0;
}
#endif

static int s2mpg11_meter_probe(struct platform_device *pdev)
{
	struct s2mpg11_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg11_platform_data *pdata = iodev->pdata;
	struct s2mpg11_meter *s2mpg11;
	int ret;
	int i;

	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	s2mpg11 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpg11_meter),
			       GFP_KERNEL);
	if (!s2mpg11)
		return -ENOMEM;

	s2mpg11->iodev = iodev;
	s2mpg11->i2c = iodev->meter;
	s2mpg11->dev = &pdev->dev;
	s2mpg11->trim = iodev->trim;
	mutex_init(&s2mpg11->meter_lock);
	platform_set_drvdata(pdev, s2mpg11);

	/* w/a for reversed NTC_LPF_DATA */
	/* 0x00 is the highest threshold for NTC values, not 0xff  */
	if (s2mpg11->iodev->pmic_rev >= S2MPG11_EVT1) {
		for (i = S2MPG11_METER_NTC_L_WARN0;
			i <= S2MPG11_METER_NTC_H_WARN7; i++)
			s2mpg11_write_reg(s2mpg11->i2c, i, 0x00);
	}
	s2mpg11_set_ntc_samp_rate(s2mpg11, NTC_0P15625HZ);

#if !IS_ENABLED(CONFIG_ODPM)

	/* initial setting */
	/* set BUCK1S ~ BUCK8S muxsel from CH0 to CH7 */
	/* any necessary settings can be added */
	s2mpg11_set_int_samp_rate(s2mpg11, INT_500HZ);

	s2mpg11_meter_set_muxsel(s2mpg11, 0, BUCK1S);
	s2mpg11_meter_set_muxsel(s2mpg11, 1, BUCK2S);
	s2mpg11_meter_set_muxsel(s2mpg11, 2, BUCK3S);
	s2mpg11_meter_set_muxsel(s2mpg11, 3, BUCK4S);
	s2mpg11_meter_set_muxsel(s2mpg11, 4, BUCK5S);
	s2mpg11_meter_set_muxsel(s2mpg11, 5, BUCK6S);
	s2mpg11_meter_set_muxsel(s2mpg11, 6, BUCK7S);
	s2mpg11_meter_set_muxsel(s2mpg11, 7, BUCK8S);

	s2mpg11_meter_onoff(s2mpg11, true);
	s2mpg11_ext_meter_onoff(s2mpg11, false);

#else
	ret = mfd_add_devices(s2mpg11->dev, -1, s2mpg11_meter_devs,
			      ARRAY_SIZE(s2mpg11_meter_devs), NULL, 0, NULL);
	if (ret < 0) {
		mfd_remove_devices(s2mpg11->dev);
		return ret;
	}
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	/* create sysfs */
	ret = create_s2mpg11_meter_sysfs(s2mpg11);
#endif

	return ret;
}

static int s2mpg11_meter_remove(struct platform_device *pdev)
{
	struct s2mpg11_meter *s2mpg11 = platform_get_drvdata(pdev);

	s2mpg11_meter_onoff(s2mpg11, false);
	s2mpg11_ext_meter_onoff(s2mpg11, false);

#if IS_ENABLED(CONFIG_ODPM)
	mfd_remove_devices(s2mpg11->dev);
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	pmic_device_destroy(s2mpg11->dev->devt);
#endif
	return 0;
}

static void s2mpg11_meter_shutdown(struct platform_device *pdev)
{
	struct s2mpg11_meter *s2mpg11 = platform_get_drvdata(pdev);

	s2mpg11_meter_onoff(s2mpg11, false);
	s2mpg11_ext_meter_onoff(s2mpg11, false);
}

static const struct platform_device_id s2mpg11_meter_id[] = {
	{ "s2mpg11-meter", 0 },
	{},
};

MODULE_DEVICE_TABLE(platform, s2mpg11_meter_id);

static struct platform_driver s2mpg11_meter_driver = {
	.driver = {
		   .name = "s2mpg11-meter",
		   .owner = THIS_MODULE,
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg11_meter_probe,
	.remove = s2mpg11_meter_remove,
	.shutdown = s2mpg11_meter_shutdown,
	.id_table = s2mpg11_meter_id,
};

static int __init s2mpg11_meter_init(void)
{
	return platform_driver_register(&s2mpg11_meter_driver);
}

subsys_initcall(s2mpg11_meter_init);

static void __exit s2mpg11_meter_exit(void)
{
	platform_driver_unregister(&s2mpg11_meter_driver);
}

module_exit(s2mpg11_meter_exit);

/* Module information */
MODULE_DESCRIPTION("SAMSUNG S2MPG11 Meter Driver");
MODULE_LICENSE("GPL");
