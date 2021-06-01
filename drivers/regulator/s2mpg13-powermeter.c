// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg13-powermeter.c
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
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/mfd/samsung/s2mpg13-register.h>
#include <linux/mfd/samsung/s2mpg1x-meter.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/regulator/pmic_class.h>
#include <linux/mfd/core.h>

static void s2mpg13_meter_set_acc_mode(struct s2mpg13_meter *s2mpg13,
				       s2mpg1x_meter_mode mode);
static void s2mpg13_meter_read_acc_data_reg(struct s2mpg13_meter *s2mpg13,
					    u64 *data);
static void s2mpg13_meter_read_acc_count(struct s2mpg13_meter *s2mpg13,
					 u32 *count);

#if IS_ENABLED(CONFIG_ODPM)
static struct mfd_cell s2mpg13_meter_devs[] = {
	{
		.name = "s2mpg13-odpm",
	},
};
#endif

/**
 * Load measurement into registers and read measurement from the registers
 *
 * Note: data must be an array with length S2MPG1X_METER_CHANNEL_MAX
 */
int s2mpg13_meter_load_measurement(struct s2mpg13_meter *s2mpg13,
				   s2mpg1x_meter_mode mode, u64 *data,
				   u32 *count, u64 *timestamp_capture)
{
	mutex_lock(&s2mpg13->meter_lock);

	s2mpg13_meter_set_acc_mode(s2mpg13, mode);

	s2mpg1x_meter_set_async_blocking(ID_S2MPG13, s2mpg13->i2c,
					 timestamp_capture);

	if (data)
		s2mpg13_meter_read_acc_data_reg(s2mpg13, data);

	if (count)
		s2mpg13_meter_read_acc_count(s2mpg13, count);

	mutex_unlock(&s2mpg13->meter_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg13_meter_load_measurement);

static u64 muxsel_to_current_resolution(s2mpg13_meter_muxsel m)
{
	switch (m) {
	case BUCK3S:
	case BUCK4S:
	case BUCK5S:
	case BUCK6S:
	case BUCK8S:
	case BUCK9S:
	case BUCK10S:
	case BUCKC:
		return CMS_BUCK_CURRENT;
	case BUCK1S:
	case BUCK2S:
		return CMT_BUCK_CURRENT;
	case BUCK7S:
	case BUCKD:
	case BUCKA:
		return VM_CURRENT;
	case BUCKBOOST:
		return BB_CURRENT;
	case LDO9S:
		return DVS_NLDO_CURRENT_150mA;
	case LDO4S:
	case LDO5S:
	case LDO6S:
	case LDO7S:
	case LDO10S:
	case LDO11S:
	case LDO13S:
	case LDO15S:
	case LDO16S:
	case LDO17S:
	case LDO18S:
	case LDO19S:
	case LDO20S:
	case LDO22S:
	case LDO28S:
		return PLDO_CURRENT_150mA;
	case LDO26S:
		return NLDO_CURRENT_300mA;
	case LDO12S:
	case LDO14S:
	case LDO27S:
		return PLDO_CURRENT_300mA;
	case LDO1S:
	case LDO23S:
	case LDO24S:
	case LDO25S:
		return DVS_NLDO_CURRENT_800mA;
	case LDO2S:
	case LDO3S:
	case LDO21S:
		return NLDO_CURRENT_800mA;
	case LDO8S:
		return NLDO_CURRENT_1200mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}

u32 s2mpg13_muxsel_to_power_resolution(s2mpg13_meter_muxsel m)
{
	switch (m) {
	case BUCK3S:
	case BUCK4S:
	case BUCK5S:
	case BUCK6S:
	case BUCK8S:
	case BUCK9S:
	case BUCK10S:
	case BUCKC:
		return CMS_BUCK_POWER;
	case BUCK1S:
	case BUCK2S:
		return CMT_BUCK_POWER;
	case BUCK7S:
	case BUCKD:
	case BUCKA:
		return VM_POWER;
	case BUCKBOOST:
		return BB_POWER;
	case LDO9S:
		return DVS_NLDO_POWER_150mA;
	case LDO4S:
	case LDO5S:
	case LDO6S:
	case LDO7S:
	case LDO10S:
	case LDO11S:
	case LDO13S:
	case LDO15S:
	case LDO16S:
	case LDO17S:
	case LDO18S:
	case LDO19S:
	case LDO20S:
	case LDO22S:
	case LDO28S:
		return PLDO_POWER_150mA;
	case LDO26S:
		return NLDO_POWER_300mA;
	case LDO12S:
	case LDO14S:
	case LDO27S:
		return PLDO_POWER_300mA;
	case LDO1S:
	case LDO23S:
	case LDO24S:
	case LDO25S:
		return DVS_NLDO_POWER_800mA;
	case LDO2S:
	case LDO3S:
	case LDO21S:
		return NLDO_POWER_800mA;
	case LDO8S:
		return NLDO_POWER_1200mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}
EXPORT_SYMBOL_GPL(s2mpg13_muxsel_to_power_resolution);

static const char *muxsel_to_str(s2mpg13_meter_muxsel m)
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
		ENUM_STR(BUCKC, ret);
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
		ENUM_STR(LDO16S, ret);
		ENUM_STR(LDO17S, ret);
		ENUM_STR(LDO18S, ret);
		ENUM_STR(LDO19S, ret);
		ENUM_STR(LDO20S, ret);
		ENUM_STR(LDO21S, ret);
		ENUM_STR(LDO22S, ret);
		ENUM_STR(LDO23S, ret);
		ENUM_STR(LDO24S, ret);
		ENUM_STR(LDO25S, ret);
		ENUM_STR(LDO26S, ret);
		ENUM_STR(LDO27S, ret);
		ENUM_STR(LDO28S, ret);
		ENUM_STR(VSEN_C4, ret);
		ENUM_STR(VSEN_C5, ret);
		ENUM_STR(VSEN_C6, ret);
	default:
		return "invalid";
	}
	return ret;
}

int s2mpg13_meter_onoff(struct s2mpg13_meter *s2mpg13, bool onoff)
{
	int ret;

	if (onoff) {
		pr_info("%s: s2mpg13 meter on\n", __func__);
		ret = s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_METER_CTRL1,
					 METER_EN_MASK, METER_EN_MASK);
		s2mpg13->meter_en = 1;
	} else {
		pr_info("%s: s2mpg13 meter off\n", __func__);
		ret = s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_METER_CTRL1, 0,
					 METER_EN_MASK);
		s2mpg13->meter_en = 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg13_meter_onoff);

int s2mpg13_ext_meter_onoff(struct s2mpg13_meter *s2mpg13, bool onoff)
{
	if (onoff) {
		dev_info(s2mpg13->dev, "s2mpg13 external meter on\n");
		return s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_METER_CTRL1,
					 EXT_METER_EN_MASK, EXT_METER_EN_MASK);
	}
	dev_info(s2mpg13->dev, "s2mpg13 external meter off\n");
	return s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_METER_CTRL1, 0,
					 EXT_METER_EN_MASK);
}
EXPORT_SYMBOL_GPL(s2mpg13_ext_meter_onoff);

int s2mpg13_meter_set_muxsel(struct s2mpg13_meter *s2mpg13, int channel,
			     s2mpg13_meter_muxsel m)
{
	int reg = S2MPG13_METER_MUXSEL0;
	int ret = -EPERM;

	if (channel < 0 || channel >= S2MPG1X_METER_CHANNEL_MAX) {
		dev_err(s2mpg13->dev, "invalid channel number\n");
		return ret;
	}

	dev_info(s2mpg13->dev, "CH%d, %s\n", channel, muxsel_to_str(m));

	reg += channel;

	mutex_lock(&s2mpg13->meter_lock);
	ret = s2mpg13_update_reg(s2mpg13->i2c, reg, m, MUXSEL_MASK);

	s2mpg13->chg_mux_sel[channel] = m;
	mutex_unlock(&s2mpg13->meter_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg13_meter_set_muxsel);

static void s2mpg13_meter_set_lpf_mode(struct s2mpg13_meter *s2mpg13,
				       s2mpg1x_meter_mode mode)
{
	switch (mode) {
	case S2MPG1X_METER_POWER:
		s2mpg13_write_reg(s2mpg13->i2c, S2MPG13_METER_CTRL6, 0x00);
		s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_METER_CTRL7, 0x00, 0x0F);
		break;
	case S2MPG1X_METER_CURRENT:
		s2mpg13_write_reg(s2mpg13->i2c, S2MPG13_METER_CTRL6, 0xFF);
		s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_METER_CTRL7, 0x0F, 0x0F);
		break;
	}
}

static void s2mpg13_meter_read_lpf_data_reg(struct s2mpg13_meter *s2mpg13)
{
	int i;
	u8 data[S2MPG1X_METER_LPF_BUF];
	u8 reg = S2MPG13_METER_LPF_DATA_CH0_1; /* first lpf data register */

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg13_bulk_read(s2mpg13->i2c, reg, S2MPG1X_METER_LPF_BUF,
				  data);
		s2mpg13->lpf_data[i] =
			data[0] + (data[1] << 8) + ((data[2] & 0x1F) << 16);
		reg += S2MPG1X_METER_LPF_BUF;
	}
}

static void s2mpg13_meter_set_acc_mode(struct s2mpg13_meter *s2mpg13,
				       s2mpg1x_meter_mode mode)
{
	switch (mode) {
	case S2MPG1X_METER_POWER:
		s2mpg13_write_reg(s2mpg13->i2c, S2MPG13_METER_CTRL4, 0x00);
		s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_METER_CTRL5, 0x00, 0x0F);
		break;
	case S2MPG1X_METER_CURRENT:
		s2mpg13_write_reg(s2mpg13->i2c, S2MPG13_METER_CTRL4, 0xFF);
		s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_METER_CTRL5, 0x0F, 0x0F);
		break;
	}
}

static void s2mpg13_meter_read_acc_data_reg(struct s2mpg13_meter *s2mpg13,
					    u64 *data)
{
	int i;
	u8 buf[S2MPG1X_METER_ACC_BUF];
	u8 reg = S2MPG13_METER_ACC_DATA_CH0_1; /* first acc data register */

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg13_bulk_read(s2mpg13->i2c, reg, S2MPG1X_METER_ACC_BUF,
				  buf);

		/* 41 bits of data */
		data[i] = ((u64)buf[0] << 0) | ((u64)buf[1] << 8) |
			  ((u64)buf[2] << 16) | ((u64)buf[3] << 24) |
			  ((u64)buf[4] << 32) | (((u64)buf[5] & 0x1) << 8);

		reg += S2MPG1X_METER_ACC_BUF;
	}

	/* b/184356774 accumulation read failure W/A */
	s2mpg13_update_reg(s2mpg13->iodev->mt_trim, S2MPG13_MT_TRIM_COMMON2,
			0, S2MPG13_PMETER_MRST_MASK);
	s2mpg13_update_reg(s2mpg13->iodev->mt_trim, S2MPG13_MT_TRIM_COMMON2,
			S2MPG13_PMETER_MRST_MASK, S2MPG13_PMETER_MRST_MASK);
	s2mpg13_usleep(2);
	s2mpg13_meter_onoff(s2mpg13, true);
}

static void s2mpg13_meter_read_acc_count(struct s2mpg13_meter *s2mpg13,
					 u32 *count)
{
	u8 data[S2MPG1X_METER_COUNT_BUF]; /* ACC_COUNT is 20-bit data */

	s2mpg13_bulk_read(s2mpg13->i2c, S2MPG13_METER_ACC_COUNT_1,
			  S2MPG1X_METER_COUNT_BUF, data);

	*count = data[0] | (data[1] << 8) | ((data[2] & 0x0F) << 16);
}

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
static ssize_t s2mpg13_muxsel_table_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int muxsel_cnt = 0;
	int muxsel = BUCK1S;
	size_t count = 0;

	while (muxsel <= VSEN_C6) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"%s : 0x%x , ",  muxsel_to_str(muxsel), muxsel);
		if (muxsel == VSEN_C6)
			break;

		if (muxsel == BUCKC)
			muxsel = BUCKBOOST;
		else if (muxsel == BUCKBOOST)
			muxsel = VSEN_P4;
		else if (muxsel == VSEN_P6)
			muxsel = LDO1S;
		else if (muxsel == LDO28S)
			muxsel = VSEN_C4;
		else
			muxsel++;

		muxsel_cnt++;
		if (!(muxsel_cnt % 8))
			count += scnprintf(buf + count,
				PAGE_SIZE - count, "\n");
	}

	return count;
}

static ssize_t s2mpg13_channel_muxsel_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct s2mpg13_meter *s2mpg13 = dev_get_drvdata(dev);
	int channel, muxsel;
	int ret;

	if (!buf) {
		dev_err(s2mpg13->dev, "empty buffer\n");
		return -EINVAL;
	}

	ret = sscanf(buf, "%d %x", &channel, &muxsel);
	if (ret != 2) {
		dev_err(s2mpg13->dev, "input error\n");
		return -EINVAL;
	}

	if (channel < 0 || channel >= S2MPG1X_METER_CHANNEL_MAX) {
		dev_err(s2mpg13->dev, "wrong channel %d\n", channel);
		return -EINVAL;
	}

	if ((muxsel >= BUCK1S && muxsel <= BUCKC) ||
	    muxsel == BUCKBOOST ||
	    (muxsel >= VSEN_P4 && muxsel <= VSEN_P6) ||
	    (muxsel >= LDO1S && muxsel <= LDO28S) ||
	    (muxsel >= VSEN_C4 && muxsel <= VSEN_C6)) {
		s2mpg13_meter_set_muxsel(s2mpg13, channel, muxsel);
	} else {
		dev_err(s2mpg13->dev, "wrong muxsel 0x%x\n", muxsel);
		return -EINVAL;
	}

	return size;
}

static ssize_t s2mpg13_channel_muxsel_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct s2mpg13_meter *s2mpg13 = dev_get_drvdata(dev);
	int i;
	size_t count = 0;

	mutex_lock(&s2mpg13->meter_lock);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "CH%d[%s], ",
			 i, muxsel_to_str(s2mpg13->chg_mux_sel[i]));
	}

	mutex_unlock(&s2mpg13->meter_lock);
	return count;
}

static ssize_t s2mpg13_lpf_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg13_meter *s2mpg13 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg13->meter_lock);

	s2mpg13_meter_set_lpf_mode(s2mpg13, S2MPG1X_METER_CURRENT);
	s2mpg13_meter_read_lpf_data_reg(s2mpg13);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg13_meter_muxsel muxsel = s2mpg13->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
						      muxsel_to_str(muxsel),
						      "(mA)",
						      s2mpg13->lpf_data[i],
						      muxsel_to_current_resolution(muxsel),
						      1);
	}
	mutex_unlock(&s2mpg13->meter_lock);
	return count;
}

static ssize_t s2mpg13_lpf_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg13_meter *s2mpg13 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg13->meter_lock);

	s2mpg13_meter_set_lpf_mode(s2mpg13, S2MPG1X_METER_POWER);
	s2mpg13_meter_read_lpf_data_reg(s2mpg13);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg13_meter_muxsel muxsel = s2mpg13->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			s2mpg13->lpf_data[i],
			s2mpg13_muxsel_to_power_resolution(muxsel), 1);
	}
	mutex_unlock(&s2mpg13->meter_lock);
	return count;
}

static ssize_t s2mpg13_acc_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg13_meter *s2mpg13 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1X_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg13_meter_load_measurement(s2mpg13, S2MPG1X_METER_CURRENT, acc_data,
				       &acc_count, NULL);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg13_meter_muxsel muxsel = s2mpg13->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mA)",
			acc_data[i], muxsel_to_current_resolution(muxsel),
			acc_count);
	}

	return count;
}

static ssize_t s2mpg13_acc_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg13_meter *s2mpg13 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1X_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg13_meter_load_measurement(s2mpg13, S2MPG1X_METER_POWER, acc_data,
				       &acc_count, NULL);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg13_meter_muxsel muxsel = s2mpg13->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			acc_data[i], s2mpg13_muxsel_to_power_resolution(muxsel),
			acc_count);
	}

	return count;
}

static DEVICE_ATTR_RO(s2mpg13_muxsel_table);
static DEVICE_ATTR_RW(s2mpg13_channel_muxsel);
static DEVICE_ATTR_RO(s2mpg13_lpf_current);
static DEVICE_ATTR_RO(s2mpg13_lpf_power);
static DEVICE_ATTR_RO(s2mpg13_acc_current);
static DEVICE_ATTR_RO(s2mpg13_acc_power);

int create_s2mpg13_meter_sysfs(struct s2mpg13_meter *s2mpg13)
{
	struct device *s2mpg13_meter_dev = s2mpg13->dev;
	int err = -ENODEV;

	s2mpg13_meter_dev = pmic_device_create(s2mpg13, "s2mpg13-meter");

	err = device_create_file(s2mpg13_meter_dev,
				 &dev_attr_s2mpg13_lpf_current);
	if (err) {
		dev_err(s2mpg13->dev,
			"s2mpg13_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg13_lpf_current.attr.name);
	}

	err = device_create_file(s2mpg13_meter_dev,
				 &dev_attr_s2mpg13_lpf_power);
	if (err) {
		dev_err(s2mpg13->dev,
			"s2mpg13_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg13_lpf_power.attr.name);
	}

	err = device_create_file(s2mpg13_meter_dev,
				 &dev_attr_s2mpg13_acc_current);
	if (err) {
		dev_err(s2mpg13->dev,
			"s2mpg13_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg13_acc_current.attr.name);
	}

	err = device_create_file(s2mpg13_meter_dev,
				 &dev_attr_s2mpg13_acc_power);
	if (err) {
		dev_err(s2mpg13->dev,
			"s2mpg13_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg13_acc_power.attr.name);
	}

	err = device_create_file(s2mpg13_meter_dev,
				 &dev_attr_s2mpg13_muxsel_table);
	if (err) {
		dev_err(s2mpg13->dev,
			"s2mpg13_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg13_muxsel_table.attr.name);
	}

	err = device_create_file(s2mpg13_meter_dev,
				 &dev_attr_s2mpg13_channel_muxsel);
	if (err) {
		dev_err(s2mpg13->dev,
			"s2mpg13_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg13_channel_muxsel.attr.name);
	}

	return 0;
}
#endif

static int s2mpg13_meter_probe(struct platform_device *pdev)
{
	struct s2mpg13_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg13_platform_data *pdata = iodev->pdata;
	struct s2mpg13_meter *s2mpg13;
	int ret = 0;

	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	s2mpg13 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpg13_meter),
			       GFP_KERNEL);
	if (!s2mpg13)
		return -ENOMEM;

	s2mpg13->iodev = iodev;
	s2mpg13->i2c = iodev->meter;
	s2mpg13->dev = &pdev->dev;
	s2mpg13->trim = iodev->trim;
	mutex_init(&s2mpg13->meter_lock);
	platform_set_drvdata(pdev, s2mpg13);

#if !IS_ENABLED(CONFIG_ODPM)

	/* initial setting */
	/* set BUCK1S ~ BUCK8S muxsel from CH0 to CH7 */
	/* any necessary settings can be added */
	s2mpg1x_meter_set_int_samp_rate(ID_S2MPG13, s2mpg13->i2c, INT_62P_5HZ);

	s2mpg13_meter_set_muxsel(s2mpg13, 0, BUCK1S);
	s2mpg13_meter_set_muxsel(s2mpg13, 1, BUCK2S);
	s2mpg13_meter_set_muxsel(s2mpg13, 2, BUCK3S);
	s2mpg13_meter_set_muxsel(s2mpg13, 3, BUCK4S);
	s2mpg13_meter_set_muxsel(s2mpg13, 4, BUCK5S);
	s2mpg13_meter_set_muxsel(s2mpg13, 5, BUCK6S);
	s2mpg13_meter_set_muxsel(s2mpg13, 6, BUCK7S);
	s2mpg13_meter_set_muxsel(s2mpg13, 7, BUCK8S);
	s2mpg13_meter_set_muxsel(s2mpg13, 8, BUCK9S);
	s2mpg13_meter_set_muxsel(s2mpg13, 9, BUCK10S);
	s2mpg13_meter_set_muxsel(s2mpg13, 10, BUCKD);
	s2mpg13_meter_set_muxsel(s2mpg13, 11, BUCKA);

	s2mpg13_meter_onoff(s2mpg13, true);
	s2mpg13_ext_meter_onoff(s2mpg13, false);

#else
	ret = mfd_add_devices(s2mpg13->dev, -1, s2mpg13_meter_devs,
			      ARRAY_SIZE(s2mpg13_meter_devs), NULL, 0, NULL);
	if (ret < 0) {
		mfd_remove_devices(s2mpg13->dev);
		return ret;
	}
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	/* create sysfs */
	ret = create_s2mpg13_meter_sysfs(s2mpg13);
#endif

	return ret;
}

static int s2mpg13_meter_remove(struct platform_device *pdev)
{
	struct s2mpg13_meter *s2mpg13 = platform_get_drvdata(pdev);

	s2mpg13_meter_onoff(s2mpg13, false);
	s2mpg13_ext_meter_onoff(s2mpg13, false);

#if IS_ENABLED(CONFIG_ODPM)
	mfd_remove_devices(s2mpg13->dev);
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	pmic_device_destroy(s2mpg13->dev->devt);
#endif
	return 0;
}

static void s2mpg13_meter_shutdown(struct platform_device *pdev)
{
	struct s2mpg13_meter *s2mpg13 = platform_get_drvdata(pdev);

	s2mpg13_meter_onoff(s2mpg13, false);
	s2mpg13_ext_meter_onoff(s2mpg13, false);
}

static const struct platform_device_id s2mpg13_meter_id[] = {
	{ "s2mpg13-meter", 0 },
	{},
};

MODULE_DEVICE_TABLE(platform, s2mpg13_meter_id);

static struct platform_driver s2mpg13_meter_driver = {
	.driver = {
		   .name = "s2mpg13-meter",
		   .owner = THIS_MODULE,
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg13_meter_probe,
	.remove = s2mpg13_meter_remove,
	.shutdown = s2mpg13_meter_shutdown,
	.id_table = s2mpg13_meter_id,
};

static int __init s2mpg13_meter_init(void)
{
	return platform_driver_register(&s2mpg13_meter_driver);
}

subsys_initcall(s2mpg13_meter_init);

static void __exit s2mpg13_meter_exit(void)
{
	platform_driver_unregister(&s2mpg13_meter_driver);
}

module_exit(s2mpg13_meter_exit);

/* Module information */
MODULE_DESCRIPTION("SAMSUNG S2MPG13 Meter Driver");
MODULE_LICENSE("GPL");
