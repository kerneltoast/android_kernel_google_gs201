// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg10-powermeter.c
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
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg1x-meter.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/regulator/pmic_class.h>
#include <linux/mfd/core.h>

#if IS_ENABLED(CONFIG_ODPM)
static struct mfd_cell s2mpg10_meter_devs[] = {
	{
		.name = "s2mpg10-odpm",
	},
};
#endif

u32 s2mpg10_muxsel_to_current_resolution(s2mpg1x_meter_muxsel m)
{
	switch (m) {
	case MUXSEL_NONE:
	case VSEN_C1: /* Requires shunt value */
	case VSEN_C2: /* Requires shunt value */
	case VSEN_C3: /* Requires shunt value */
		return INVALID_RESOLUTION;
	case BUCK1:
	case BUCK4:
	case BUCK6:
	case BUCK7:
	case BUCK8:
	case BUCK9:
		return CMS_BUCK_CURRENT;
	case BUCK3:
		return CMD_BUCK_CURRENT;
	case BUCK2:
	case BUCK5:
	case BUCK10:
		return CMT_BUCK_CURRENT;
	case LDO1:
	case LDO11:
		return DVS_NLDO_CURRENT_150mA;
	case LDO5:
	case LDO6:
	case LDO24:
		return NLDO_CURRENT_150mA;
	case LDO4:
	case LDO9:
	case LDO10:
	case LDO14:
	case LDO18:
	case LDO19:
	case LDO20:
	case LDO22:
	case LDO25:
	case LDO26:
	case LDO27:
	case LDO29:
	case LDO30:
		return PLDO_CURRENT_150mA;
	case LDO8:
	case LDO28:
		return NLDO_CURRENT_300mA;
	case LDO23:
	case LDO31:
	case LDO21:
		return PLDO_CURRENT_300mA;
	case LDO7:
	case LDO13:
		return DVS_NLDO_CURRENT_450mA;
	case LDO12:
	case LDO15:
		return DVS_NLDO_CURRENT_800mA;
	case LDO3:
	case LDO16:
	case LDO17:
		return NLDO_CURRENT_800mA;
	case LDO2:
		return PLDO_CURRENT_800mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}
EXPORT_SYMBOL_GPL(s2mpg10_muxsel_to_current_resolution);

u32 s2mpg10_muxsel_to_power_resolution(s2mpg1x_meter_muxsel m)
{
	switch (m) {
	case MUXSEL_NONE:
	case VSEN_C1: /* Requires shunt value */
	case VSEN_C2: /* Requires shunt value */
	case VSEN_C3: /* Requires shunt value */
		return INVALID_RESOLUTION;
	case BUCK1:
	case BUCK4:
	case BUCK6:
	case BUCK7:
	case BUCK8:
	case BUCK9:
		return CMS_BUCK_POWER;
	case BUCK3:
		return CMD_BUCK_POWER;
	case BUCK2:
	case BUCK5:
	case BUCK10:
		return CMT_BUCK_POWER;
	case LDO1:
	case LDO11:
		return DVS_NLDO_POWER_150mA;
	case LDO5:
	case LDO6:
	case LDO24:
		return NLDO_POWER_150mA;
	case LDO4:
	case LDO9:
	case LDO10:
	case LDO14:
	case LDO18:
	case LDO19:
	case LDO20:
	case LDO22:
	case LDO25:
	case LDO26:
	case LDO27:
	case LDO29:
	case LDO30:
		return PLDO_POWER_150mA;
	case LDO8:
	case LDO28:
		return NLDO_POWER_300mA;
	case LDO23:
	case LDO31:
	case LDO21:
		return PLDO_POWER_300mA;
	case LDO7:
	case LDO13:
		return DVS_NLDO_POWER_450mA;
	case LDO12:
	case LDO15:
		return DVS_NLDO_POWER_800mA;
	case LDO3:
	case LDO16:
	case LDO17:
		return NLDO_POWER_800mA;
	case LDO2:
		return PLDO_POWER_800mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}
EXPORT_SYMBOL_GPL(s2mpg10_muxsel_to_power_resolution);

static const char *muxsel_to_str(s2mpg1x_meter_muxsel m)
{
	char *ret;

	switch (m) {
		ENUM_STR(MUXSEL_NONE, "M", ret);
		ENUM_STR(BUCK1, "M", ret);
		ENUM_STR(BUCK2, "M", ret);
		ENUM_STR(BUCK3, "M", ret);
		ENUM_STR(BUCK4, "M", ret);
		ENUM_STR(BUCK5, "M", ret);
		ENUM_STR(BUCK6, "M", ret);
		ENUM_STR(BUCK7, "M", ret);
		ENUM_STR(BUCK8, "M", ret);
		ENUM_STR(BUCK9, "M", ret);
		ENUM_STR(BUCK10, "M", ret);
		ENUM_STR(VSEN_V1, "", ret);
		ENUM_STR(VSEN_V2, "", ret);
		ENUM_STR(VSEN_V3, "", ret);
		ENUM_STR(LDO1, "M", ret);
		ENUM_STR(LDO2, "M", ret);
		ENUM_STR(LDO3, "M", ret);
		ENUM_STR(LDO4, "M", ret);
		ENUM_STR(LDO5, "M", ret);
		ENUM_STR(LDO6, "M", ret);
		ENUM_STR(LDO7, "M", ret);
		ENUM_STR(LDO8, "M", ret);
		ENUM_STR(LDO9, "M", ret);
		ENUM_STR(LDO10, "M", ret);
		ENUM_STR(LDO11, "M", ret);
		ENUM_STR(LDO12, "M", ret);
		ENUM_STR(LDO13, "M", ret);
		ENUM_STR(LDO14, "M", ret);
		ENUM_STR(LDO15, "M", ret);
		ENUM_STR(LDO16, "M", ret);
		ENUM_STR(LDO17, "M", ret);
		ENUM_STR(LDO18, "M", ret);
		ENUM_STR(LDO19, "M", ret);
		ENUM_STR(LDO20, "M", ret);
		ENUM_STR(LDO21, "M", ret);
		ENUM_STR(LDO22, "M", ret);
		ENUM_STR(LDO23, "M", ret);
		ENUM_STR(LDO24, "M", ret);
		ENUM_STR(LDO25, "M", ret);
		ENUM_STR(LDO26, "M", ret);
		ENUM_STR(LDO27, "M", ret);
		ENUM_STR(LDO28, "M", ret);
		ENUM_STR(LDO29, "M", ret);
		ENUM_STR(LDO30, "M", ret);
		ENUM_STR(LDO31, "M", ret);
		ENUM_STR(VSEN_C1, "", ret);
		ENUM_STR(VSEN_C2, "", ret);
		ENUM_STR(VSEN_C3, "", ret);
	default:
		return "invalid";
	}
	return ret;
}

int s2mpg10_meter_onoff(struct s2mpg10_meter *s2mpg10, bool onoff)
{
	int ret;

	if (onoff) {
		pr_info("%s: s2mpg10 meter on\n", __func__);
		ret = s2mpg10_update_reg(s2mpg10->i2c, S2MPG10_METER_CTRL1,
					 METER_EN_MASK, METER_EN_MASK);
	} else {
		pr_info("%s: s2mpg10 meter off\n", __func__);
		ret = s2mpg10_update_reg(s2mpg10->i2c, S2MPG10_METER_CTRL1, 0,
					 METER_EN_MASK);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg10_meter_onoff);

int s2mpg10_ext_meter_onoff(struct s2mpg10_meter *s2mpg10, bool onoff)
{
	int ret;

	if (onoff) {
		pr_info("%s: s2mpg10 external meter on\n", __func__);
		ret = s2mpg10_update_reg(s2mpg10->i2c, S2MPG10_METER_CTRL1,
					 EXT_METER_EN_MASK, EXT_METER_EN_MASK);
	} else {
		pr_info("%s: s2mpg10 external meter off\n", __func__);
		ret = s2mpg10_update_reg(s2mpg10->i2c, S2MPG10_METER_CTRL1, 0,
					 EXT_METER_EN_MASK);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg10_ext_meter_onoff);

int s2mpg10_meter_set_muxsel(struct s2mpg10_meter *s2mpg10, int channel,
			     s2mpg1x_meter_muxsel m)
{
	int reg = S2MPG10_METER_MUXSEL0;
	int ret = -EPERM;

	if (channel < 0 || channel >= S2MPG1X_METER_CHANNEL_MAX) {
		pr_err("%s: invalid channel number\n", __func__);
		return ret;
	}

	pr_debug("%s: CH%d, %s\n", __func__, channel, muxsel_to_str(m));

	reg += channel;

	mutex_lock(&s2mpg10->meter_lock);
	ret = s2mpg10_update_reg(s2mpg10->i2c, reg, m, MUXSEL_MASK);

	s2mpg10->chg_mux_sel[channel] = m;
	mutex_unlock(&s2mpg10->meter_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg10_meter_set_muxsel);

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
static ssize_t s2mpg10_muxsel_table_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int muxsel_cnt = 0;
	int muxsel = BUCK1;
	size_t count = 0;

	while (muxsel <= VSEN_C3) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"%s : 0x%x , ", muxsel_to_str(muxsel), muxsel);

		if (muxsel == BUCK10)
			muxsel = VSEN_V1;
		else if (muxsel == VSEN_V3)
			muxsel = LDO1;
		else if (muxsel == LDO31)
			muxsel = VSEN_C1;
		else if (muxsel == VSEN_C3)
			break;
		else
			muxsel++;

		muxsel_cnt++;
		if  (!(muxsel_cnt % 8))
			count += scnprintf(buf + count,
				PAGE_SIZE - count, "\n");
	}

	return count;
}

static ssize_t s2mpg10_channel_muxsel_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct s2mpg10_meter *s2mpg10 = dev_get_drvdata(dev);
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

	if ((muxsel >= BUCK1 && muxsel <= BUCK10) ||
	    (muxsel >= VSEN_V1 && muxsel <= VSEN_V3) ||
	    (muxsel >= LDO1 && muxsel <= LDO31) ||
	    (muxsel >= VSEN_C1 && muxsel <= VSEN_C3)) {
		s2mpg10_meter_set_muxsel(s2mpg10, channel, muxsel);
	} else {
		pr_err("%s: wrong muxsel 0x%x\n", __func__, muxsel);
		return -EINVAL;
	}

	return size;
}

static ssize_t s2mpg10_channel_muxsel_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct s2mpg10_meter *s2mpg10 = dev_get_drvdata(dev);
	int i;
	size_t count = 0;

	mutex_lock(&s2mpg10->meter_lock);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "CH%d[%s], ",
			 i, muxsel_to_str(s2mpg10->chg_mux_sel[i]));
	}

	mutex_unlock(&s2mpg10->meter_lock);
	return count;
}

static ssize_t s2mpg10_lpf_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg10_meter *s2mpg10 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg10->meter_lock);

	s2mpg1x_meter_set_lpf_mode(ID_S2MPG10, s2mpg10->i2c,
				   S2MPG1X_METER_CURRENT);
	s2mpg1x_meter_read_lpf_data_reg(ID_S2MPG10, s2mpg10->i2c,
					s2mpg10->lpf_data);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_meter_muxsel muxsel = s2mpg10->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mA)",
			s2mpg10->lpf_data[i],
			s2mpg10_muxsel_to_current_resolution(muxsel), 1);
	}
	mutex_unlock(&s2mpg10->meter_lock);
	return count;
}

static ssize_t s2mpg10_lpf_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg10_meter *s2mpg10 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg10->meter_lock);

	s2mpg1x_meter_set_lpf_mode(ID_S2MPG10, s2mpg10->i2c,
				   S2MPG1X_METER_POWER);
	s2mpg1x_meter_read_lpf_data_reg(ID_S2MPG10, s2mpg10->i2c,
					s2mpg10->lpf_data);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_meter_muxsel muxsel = s2mpg10->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			s2mpg10->lpf_data[i],
			s2mpg10_muxsel_to_power_resolution(muxsel), 1);
	}
	mutex_unlock(&s2mpg10->meter_lock);
	return count;
}

static ssize_t s2mpg10_acc_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg10_meter *s2mpg10 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1X_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg1x_meter_measure_acc(ID_S2MPG10, s2mpg10->i2c,
				  &s2mpg10->meter_lock,
				  S2MPG1X_METER_CURRENT, acc_data,
				  &acc_count, NULL, INT_125HZ);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_meter_muxsel muxsel = s2mpg10->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mA)",
			acc_data[i],
			s2mpg10_muxsel_to_current_resolution(muxsel),
			acc_count);
	}

	return count;
}

static ssize_t s2mpg10_acc_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg10_meter *s2mpg10 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1X_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg1x_meter_measure_acc(ID_S2MPG10, s2mpg10->i2c,
				  &s2mpg10->meter_lock,
				  S2MPG1X_METER_POWER, acc_data,
				  &acc_count, NULL, INT_125HZ);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_meter_muxsel muxsel = s2mpg10->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			acc_data[i], s2mpg10_muxsel_to_power_resolution(muxsel),
			acc_count);
	}

	return count;
}

static DEVICE_ATTR_RO(s2mpg10_muxsel_table);
static DEVICE_ATTR_RW(s2mpg10_channel_muxsel);
static DEVICE_ATTR_RO(s2mpg10_lpf_current);
static DEVICE_ATTR_RO(s2mpg10_lpf_power);
static DEVICE_ATTR_RO(s2mpg10_acc_current);
static DEVICE_ATTR_RO(s2mpg10_acc_power);

int create_s2mpg10_meter_sysfs(struct s2mpg10_meter *s2mpg10)
{
	struct device *s2mpg10_meter_dev = s2mpg10->dev;
	int err = -ENODEV;

	pr_info("%s: s2mpg10 meter sysfs start\n", __func__);

	s2mpg10_meter_dev = pmic_device_create(s2mpg10, "s2mpg10-meter");

	err = device_create_file(s2mpg10_meter_dev,
				 &dev_attr_s2mpg10_lpf_current);
	if (err) {
		pr_err("s2mpg10_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg10_lpf_current.attr.name);
	}

	err = device_create_file(s2mpg10_meter_dev,
				 &dev_attr_s2mpg10_lpf_power);
	if (err) {
		pr_err("s2mpg10_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg10_lpf_power.attr.name);
	}

	err = device_create_file(s2mpg10_meter_dev,
				 &dev_attr_s2mpg10_acc_current);
	if (err) {
		pr_err("s2mpg10_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg10_acc_current.attr.name);
	}

	err = device_create_file(s2mpg10_meter_dev,
				 &dev_attr_s2mpg10_acc_power);
	if (err) {
		pr_err("s2mpg10_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg10_acc_power.attr.name);
	}

	err = device_create_file(s2mpg10_meter_dev,
				 &dev_attr_s2mpg10_channel_muxsel);
	if (err) {
		pr_err("s2mpg10_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg10_channel_muxsel.attr.name);
	}

	err = device_create_file(s2mpg10_meter_dev,
				 &dev_attr_s2mpg10_muxsel_table);
	if (err) {
		pr_err("s2mpg10_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg10_muxsel_table.attr.name);
	}

	return 0;
}
#endif

static int s2mpg10_meter_probe(struct platform_device *pdev)
{
	struct s2mpg10_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg10_platform_data *pdata = iodev->pdata;
	struct s2mpg10_meter *s2mpg10;
	u8 dsm_trim_high = 0, dsm_trim_low = 0;
	char dsm_trim, dsm_result;
	int ret;

	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	s2mpg10 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpg10_meter),
			       GFP_KERNEL);
	if (!s2mpg10)
		return -ENOMEM;

	s2mpg10->i2c = iodev->meter;
	s2mpg10->dev = &pdev->dev;

	mutex_init(&s2mpg10->meter_lock);
	platform_set_drvdata(pdev, s2mpg10);

	/* w/a powermeter offset rising */
	ret = s2mpg10_read_reg(s2mpg10->i2c, S2MPG10_METER_DSM_TRIM_OFFSET, &dsm_trim_low);
	ret = s2mpg10_read_reg(s2mpg10->i2c, S2MPG10_METER_BUCK_METER_TRIM3, &dsm_trim_high);
	pr_info("%s: DSM_TRIM_OFFSET: 0x%x\n", __func__, dsm_trim_low);
	pr_info("%s: BUCK_METER_TRIM3: 0x%x\n", __func__, dsm_trim_high);

	dsm_trim = (dsm_trim_high & S2MPG10_METER_DSM_TRIM_HIGH_MASK) |
				(dsm_trim_low & S2MPG10_METER_DSM_TRIM_LOW_MASK);
	if (dsm_trim & BIT(7)) {
		dsm_result = dsm_trim + S2MPG10_DSM_TRIM_OFFSET_ADD_VALUE;
	} else {
		if (dsm_trim >= S2MPG10_DSM_TRIM_OFFSET_THRESHOLD)
			dsm_result = S2MPG10_DSM_TRIM_OFFSET_MAX_VALUE;
		else
			dsm_result = dsm_trim + S2MPG10_DSM_TRIM_OFFSET_ADD_VALUE;
	}
	pr_info("%s: dsm_trim: %d, dsm_result: %d\n", __func__, dsm_trim, dsm_result);
	ret = s2mpg10_update_reg(s2mpg10->i2c, S2MPG10_METER_DSM_TRIM_OFFSET,
				dsm_result & S2MPG10_METER_DSM_TRIM_LOW_MASK,
				S2MPG10_METER_DSM_TRIM_LOW_MASK);
	ret = s2mpg10_update_reg(s2mpg10->i2c, S2MPG10_METER_BUCK_METER_TRIM3,
				dsm_result & S2MPG10_METER_DSM_TRIM_HIGH_MASK,
				S2MPG10_METER_DSM_TRIM_HIGH_MASK);

#if !IS_ENABLED(CONFIG_ODPM)

	/* initial setting */
	/* set BUCK1M ~ BUCK8m muxsel from CH0 to CH7 */
	/* any necessary settings can be added */
	s2mpg1x_meter_set_int_samp_rate(ID_S2MPG10, s2mpg10->i2c, INT_125HZ);

	s2mpg10_meter_set_muxsel(s2mpg10, 0, BUCK1);
	s2mpg10_meter_set_muxsel(s2mpg10, 1, BUCK2);
	s2mpg10_meter_set_muxsel(s2mpg10, 2, BUCK3);
	s2mpg10_meter_set_muxsel(s2mpg10, 3, BUCK4);
	s2mpg10_meter_set_muxsel(s2mpg10, 4, BUCK5);
	s2mpg10_meter_set_muxsel(s2mpg10, 5, BUCK6);
	s2mpg10_meter_set_muxsel(s2mpg10, 6, BUCK7);
	s2mpg10_meter_set_muxsel(s2mpg10, 7, BUCK8);

	s2mpg10_meter_onoff(s2mpg10, true);
	s2mpg10_ext_meter_onoff(s2mpg10, false);

#else
	ret = mfd_add_devices(s2mpg10->dev, -1, s2mpg10_meter_devs,
			      ARRAY_SIZE(s2mpg10_meter_devs), NULL, 0, NULL);
	if (ret < 0) {
		mfd_remove_devices(s2mpg10->dev);
		return ret;
	}
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	/* create sysfs */
	ret = create_s2mpg10_meter_sysfs(s2mpg10);
#endif

	return ret;
}

static int s2mpg10_meter_remove(struct platform_device *pdev)
{
	struct s2mpg10_meter *s2mpg10 = platform_get_drvdata(pdev);

	s2mpg10_meter_onoff(s2mpg10, false);
	s2mpg10_ext_meter_onoff(s2mpg10, false);

#if IS_ENABLED(CONFIG_ODPM)
	mfd_remove_devices(s2mpg10->dev);
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	pmic_device_destroy(s2mpg10->dev->devt);
#endif
	return 0;
}

static void s2mpg10_meter_shutdown(struct platform_device *pdev)
{
	struct s2mpg10_meter *s2mpg10 = platform_get_drvdata(pdev);

	s2mpg10_meter_onoff(s2mpg10, false);
	s2mpg10_ext_meter_onoff(s2mpg10, false);
}

static const struct platform_device_id s2mpg10_meter_id[] = {
	{ "s2mpg10-meter", 0 },
	{},
};

MODULE_DEVICE_TABLE(platform, s2mpg10_meter_id);

static struct platform_driver s2mpg10_meter_driver = {
	.driver = {
		   .name = "s2mpg10-meter",
		   .owner = THIS_MODULE,
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg10_meter_probe,
	.remove = s2mpg10_meter_remove,
	.shutdown = s2mpg10_meter_shutdown,
	.id_table = s2mpg10_meter_id,
};

static int __init s2mpg10_meter_init(void)
{
	return platform_driver_register(&s2mpg10_meter_driver);
}

subsys_initcall(s2mpg10_meter_init);

static void __exit s2mpg10_meter_exit(void)
{
	platform_driver_unregister(&s2mpg10_meter_driver);
}

module_exit(s2mpg10_meter_exit);

/* Module information */
MODULE_DESCRIPTION("SAMSUNG S2MPG10 Meter Driver");
MODULE_LICENSE("GPL");
