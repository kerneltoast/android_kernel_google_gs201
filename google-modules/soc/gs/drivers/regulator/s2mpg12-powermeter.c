// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg12-powermeter.c
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
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg12-register.h>
#include <linux/mfd/samsung/s2mpg1x-meter.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/regulator/pmic_class.h>
#include <linux/mfd/core.h>

#if IS_ENABLED(CONFIG_ODPM)
static struct mfd_cell s2mpg12_meter_devs[] = {
	{
		.name = "s2mpg12-odpm",
	},
};
#endif

u32 s2mpg12_muxsel_to_current_resolution(s2mpg1x_meter_muxsel m)
{
	switch (m) {
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
	case LDO5:
		return DVS_NLDO_CURRENT_150mA;
	case LDO4:
	case LDO9:
	case LDO10:
	case LDO14:
	case LDO18:
	case LDO20:
	case LDO23:
	case LDO25:
	case LDO27:
		return PLDO_CURRENT_150mA;
	case LDO1:
	case LDO3:
	case LDO8:
		return NLDO_CURRENT_300mA;
	case LDO21:
	case LDO26:
		return PLDO_CURRENT_300mA;
	case LDO6:
	case LDO28:
		return NLDO_CURRENT_450mA;
	case LDO24:
		return PLDO_CURRENT_600mA;
	case LDO7:
	case LDO11:
	case LDO12:
	case LDO13:
	case LDO15:
	case LDO17:
	case LDO19:
	case LDO22:
		return DVS_NLDO_CURRENT_800mA;
	case LDO16:
		return NLDO_CURRENT_800mA;
	case LDO2:
		return PLDO_CURRENT_800mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}
EXPORT_SYMBOL_GPL(s2mpg12_muxsel_to_current_resolution);

u32 s2mpg12_muxsel_to_power_resolution(s2mpg1x_meter_muxsel m)
{
	switch (m) {
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
	case LDO5:
		return DVS_NLDO_POWER_150mA;
	case LDO4:
	case LDO9:
	case LDO10:
	case LDO14:
	case LDO18:
	case LDO20:
	case LDO23:
	case LDO25:
	case LDO27:
		return PLDO_POWER_150mA;
	case LDO1:
	case LDO3:
	case LDO8:
		return NLDO_POWER_300mA;
	case LDO21:
	case LDO26:
		return PLDO_POWER_300mA;
	case LDO6:
	case LDO28:
		return NLDO_POWER_450mA;
	case LDO24:
		return PLDO_POWER_600mA;
	case LDO7:
	case LDO11:
	case LDO12:
	case LDO13:
	case LDO15:
	case LDO17:
	case LDO19:
	case LDO22:
		return DVS_NLDO_POWER_800mA;
	case LDO16:
		return NLDO_POWER_800mA;
	case LDO2:
		return PLDO_POWER_800mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}
EXPORT_SYMBOL_GPL(s2mpg12_muxsel_to_power_resolution);

static const char *muxsel_to_str(s2mpg1x_meter_muxsel m)
{
	char *ret;

	switch (m) {
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
		ENUM_STR(VSEN_C1, "", ret);
		ENUM_STR(VSEN_C2, "", ret);
		ENUM_STR(VSEN_C3, "", ret);
	default:
		return "invalid";
	}
	return ret;
}

int s2mpg12_meter_onoff(struct s2mpg12_meter *s2mpg12, bool onoff)
{
	if (onoff) {
		dev_info(s2mpg12->dev, "s2mpg12 meter on\n");
		return s2mpg12_update_reg(s2mpg12->i2c, S2MPG12_METER_CTRL1,
					 METER_EN_MASK, METER_EN_MASK);
	}
	dev_info(s2mpg12->dev, "s2mpg12 meter off\n");
	return s2mpg12_update_reg(s2mpg12->i2c, S2MPG12_METER_CTRL1, 0,
					 METER_EN_MASK);
}
EXPORT_SYMBOL_GPL(s2mpg12_meter_onoff);

int s2mpg12_ext_meter_onoff(struct s2mpg12_meter *s2mpg12, bool onoff)
{
	if (onoff) {
		dev_info(s2mpg12->dev, "s2mpg12 external meter on\n");
		return s2mpg12_update_reg(s2mpg12->i2c, S2MPG12_METER_CTRL1,
					 EXT_METER_EN_MASK, EXT_METER_EN_MASK);
	}
	dev_info(s2mpg12->dev, "s2mpg12 external meter off\n");
	return s2mpg12_update_reg(s2mpg12->i2c, S2MPG12_METER_CTRL1, 0,
					 EXT_METER_EN_MASK);
}
EXPORT_SYMBOL_GPL(s2mpg12_ext_meter_onoff);

int s2mpg12_meter_set_muxsel(struct s2mpg12_meter *s2mpg12, int channel,
			     s2mpg1x_meter_muxsel m)
{
	int reg = S2MPG12_METER_MUXSEL0;
	int ret = -EPERM;

	if (channel < 0 || channel >= S2MPG1X_METER_CHANNEL_MAX) {
		dev_err(s2mpg12->dev, "invalid channel number\n");
		return ret;
	}

	dev_info(s2mpg12->dev, "CH%d, %s\n", channel, muxsel_to_str(m));

	reg += channel;

	mutex_lock(&s2mpg12->meter_lock);
	ret = s2mpg12_update_reg(s2mpg12->i2c, reg, m, MUXSEL_MASK);

	s2mpg12->chg_mux_sel[channel] = m;
	mutex_unlock(&s2mpg12->meter_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg12_meter_set_muxsel);

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
static ssize_t s2mpg12_muxsel_table_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int muxsel_cnt = 0;
	int muxsel = BUCK1;
	size_t count = 0;

	while (muxsel <= VSEN_C3) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"%s : 0x%x , ", muxsel_to_str(muxsel), muxsel);
		if (muxsel == VSEN_C3)
			break;

		if (muxsel == BUCK10)
			muxsel = VSEN_V1;
		else if (muxsel == VSEN_V3)
			muxsel = LDO1;
		else if (muxsel == LDO28)
			muxsel = VSEN_C1;
		else
			muxsel++;

		muxsel_cnt++;
		if  (!(muxsel_cnt % 8))
			count += scnprintf(buf + count,
				PAGE_SIZE - count, "\n");
	}

	return count;
}

static ssize_t s2mpg12_channel_muxsel_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct s2mpg12_meter *s2mpg12 = dev_get_drvdata(dev);
	int channel, muxsel;
	int ret;

	if (!buf) {
		dev_err(s2mpg12->dev, "empty buffer\n");
		return -EINVAL;
	}

	ret = sscanf(buf, "%d %x", &channel, &muxsel);
	if (ret != 2) {
		dev_err(s2mpg12->dev, "input error\n");
		return -EINVAL;
	}

	if (channel < 0 || channel >= S2MPG1X_METER_CHANNEL_MAX) {
		dev_err(s2mpg12->dev, "wrong channel %d\n", channel);
		return -EINVAL;
	}

	if ((muxsel >= BUCK1 && muxsel <= BUCK10) ||
	    (muxsel >= VSEN_V1 && muxsel <= VSEN_V3) ||
	    (muxsel >= LDO1 && muxsel <= LDO28) ||
	    (muxsel >= VSEN_C1 && muxsel <= VSEN_C3)) {
		s2mpg12_meter_set_muxsel(s2mpg12, channel, muxsel);
	} else {
		dev_err(s2mpg12->dev, "wrong muxsel 0x%x\n", muxsel);
		return -EINVAL;
	}

	return size;
}

static ssize_t s2mpg12_channel_muxsel_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct s2mpg12_meter *s2mpg12 = dev_get_drvdata(dev);
	int i;
	size_t count = 0;

	mutex_lock(&s2mpg12->meter_lock);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "CH%d[%s], ",
			 i, muxsel_to_str(s2mpg12->chg_mux_sel[i]));
	}

	mutex_unlock(&s2mpg12->meter_lock);
	return count;
}

static ssize_t s2mpg12_lpf_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg12_meter *s2mpg12 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg12->meter_lock);

	s2mpg1x_meter_set_lpf_mode(ID_S2MPG12, s2mpg12->i2c,
				   S2MPG1X_METER_CURRENT);
	s2mpg1x_meter_read_lpf_data_reg(ID_S2MPG12, s2mpg12->i2c,
					s2mpg12->lpf_data);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_meter_muxsel muxsel = s2mpg12->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mA)",
			s2mpg12->lpf_data[i],
			s2mpg12_muxsel_to_current_resolution(muxsel), 1);
	}
	mutex_unlock(&s2mpg12->meter_lock);
	return count;
}

static ssize_t s2mpg12_lpf_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg12_meter *s2mpg12 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg12->meter_lock);

	s2mpg1x_meter_set_lpf_mode(ID_S2MPG12, s2mpg12->i2c,
				   S2MPG1X_METER_POWER);
	s2mpg1x_meter_read_lpf_data_reg(ID_S2MPG12, s2mpg12->i2c,
					s2mpg12->lpf_data);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_meter_muxsel muxsel = s2mpg12->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			s2mpg12->lpf_data[i],
			s2mpg12_muxsel_to_power_resolution(muxsel), 1);
	}
	mutex_unlock(&s2mpg12->meter_lock);
	return count;
}

static ssize_t s2mpg12_acc_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg12_meter *s2mpg12 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1X_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg1x_meter_measure_acc(ID_S2MPG12, s2mpg12->i2c,
				  &s2mpg12->meter_lock,
				  S2MPG1X_METER_CURRENT, acc_data,
				  &acc_count, NULL, INT_125HZ);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_meter_muxsel muxsel = s2mpg12->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mA)",
			acc_data[i],
			s2mpg12_muxsel_to_current_resolution(muxsel),
			acc_count);
	}

	return count;
}

static ssize_t s2mpg12_acc_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg12_meter *s2mpg12 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1X_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg1x_meter_measure_acc(ID_S2MPG12, s2mpg12->i2c,
				  &s2mpg12->meter_lock,
				  S2MPG1X_METER_POWER, acc_data,
				  &acc_count, NULL, INT_125HZ);

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_meter_muxsel muxsel = s2mpg12->chg_mux_sel[i];

		count += s2mpg1x_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			acc_data[i], s2mpg12_muxsel_to_power_resolution(muxsel),
			acc_count);
	}

	return count;
}

static DEVICE_ATTR_RO(s2mpg12_muxsel_table);
static DEVICE_ATTR_RW(s2mpg12_channel_muxsel);
static DEVICE_ATTR_RO(s2mpg12_lpf_current);
static DEVICE_ATTR_RO(s2mpg12_lpf_power);
static DEVICE_ATTR_RO(s2mpg12_acc_current);
static DEVICE_ATTR_RO(s2mpg12_acc_power);

int create_s2mpg12_meter_sysfs(struct s2mpg12_meter *s2mpg12)
{
	struct device *s2mpg12_meter_dev = s2mpg12->dev;
	int err = -ENODEV;

	s2mpg12_meter_dev = pmic_device_create(s2mpg12, "s2mpg12-meter");

	err = device_create_file(s2mpg12_meter_dev,
				 &dev_attr_s2mpg12_lpf_current);
	if (err) {
		dev_err(s2mpg12->dev,
			"s2mpg12_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg12_lpf_current.attr.name);
	}

	err = device_create_file(s2mpg12_meter_dev,
				 &dev_attr_s2mpg12_lpf_power);
	if (err) {
		dev_err(s2mpg12->dev,
			"s2mpg12_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg12_lpf_power.attr.name);
	}

	err = device_create_file(s2mpg12_meter_dev,
				 &dev_attr_s2mpg12_acc_current);
	if (err) {
		dev_err(s2mpg12->dev,
			"s2mpg12_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg12_acc_current.attr.name);
	}

	err = device_create_file(s2mpg12_meter_dev,
				 &dev_attr_s2mpg12_acc_power);
	if (err) {
		dev_err(s2mpg12->dev,
			"s2mpg12_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg12_acc_power.attr.name);
	}

	err = device_create_file(s2mpg12_meter_dev,
				 &dev_attr_s2mpg12_channel_muxsel);
	if (err) {
		dev_err(s2mpg12->dev,
			"s2mpg12_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg12_channel_muxsel.attr.name);
	}

	err = device_create_file(s2mpg12_meter_dev,
				 &dev_attr_s2mpg12_muxsel_table);
	if (err) {
		dev_err(s2mpg12->dev,
			"s2mpg12_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg12_muxsel_table.attr.name);
	}

	return 0;
}
#endif

static int s2mpg12_meter_probe(struct platform_device *pdev)
{
	struct s2mpg12_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg12_platform_data *pdata = iodev->pdata;
	struct s2mpg12_meter *s2mpg12;
	int ret = 0;

	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	s2mpg12 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpg12_meter),
			       GFP_KERNEL);
	if (!s2mpg12)
		return -ENOMEM;

	s2mpg12->iodev = iodev;
	s2mpg12->i2c = iodev->meter;
	s2mpg12->dev = &pdev->dev;

	mutex_init(&s2mpg12->meter_lock);
	platform_set_drvdata(pdev, s2mpg12);

#if !IS_ENABLED(CONFIG_ODPM)

	/* initial setting */
	/* set BUCK1M ~ BUCK8m muxsel from CH0 to CH7 */
	/* any necessary settings can be added */
	s2mpg1x_meter_set_int_samp_rate(ID_S2MPG12, s2mpg12->i2c, INT_125HZ);

	s2mpg12_meter_set_muxsel(s2mpg12, 0, BUCK1);
	s2mpg12_meter_set_muxsel(s2mpg12, 1, BUCK2);
	s2mpg12_meter_set_muxsel(s2mpg12, 2, BUCK3);
	s2mpg12_meter_set_muxsel(s2mpg12, 3, BUCK4);
	s2mpg12_meter_set_muxsel(s2mpg12, 4, BUCK5);
	s2mpg12_meter_set_muxsel(s2mpg12, 5, BUCK6);
	s2mpg12_meter_set_muxsel(s2mpg12, 6, BUCK7);
	s2mpg12_meter_set_muxsel(s2mpg12, 7, BUCK9);
	s2mpg12_meter_set_muxsel(s2mpg12, 8, BUCK10);
	s2mpg12_meter_set_muxsel(s2mpg12, 9, LDO1);
	s2mpg12_meter_set_muxsel(s2mpg12, 10, LDO2);
	s2mpg12_meter_set_muxsel(s2mpg12, 11, LDO3);

	s2mpg12_meter_onoff(s2mpg12, true);
	s2mpg12_ext_meter_onoff(s2mpg12, false);

#else
	ret = mfd_add_devices(s2mpg12->dev, -1, s2mpg12_meter_devs,
			      ARRAY_SIZE(s2mpg12_meter_devs), NULL, 0, NULL);
	if (ret < 0) {
		mfd_remove_devices(s2mpg12->dev);
		return ret;
	}
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	/* create sysfs */
	ret = create_s2mpg12_meter_sysfs(s2mpg12);
#endif

	return ret;
}

static int s2mpg12_meter_remove(struct platform_device *pdev)
{
	struct s2mpg12_meter *s2mpg12 = platform_get_drvdata(pdev);

	s2mpg12_meter_onoff(s2mpg12, false);
	s2mpg12_ext_meter_onoff(s2mpg12, false);

#if IS_ENABLED(CONFIG_ODPM)
	mfd_remove_devices(s2mpg12->dev);
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	pmic_device_destroy(s2mpg12->dev->devt);
#endif
	return 0;
}

static void s2mpg12_meter_shutdown(struct platform_device *pdev)
{
	struct s2mpg12_meter *s2mpg12 = platform_get_drvdata(pdev);

	s2mpg12_meter_onoff(s2mpg12, false);
	s2mpg12_ext_meter_onoff(s2mpg12, false);
}

static const struct platform_device_id s2mpg12_meter_id[] = {
	{ "s2mpg12-meter", 0 },
	{},
};

MODULE_DEVICE_TABLE(platform, s2mpg12_meter_id);

static struct platform_driver s2mpg12_meter_driver = {
	.driver = {
		   .name = "s2mpg12-meter",
		   .owner = THIS_MODULE,
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg12_meter_probe,
	.remove = s2mpg12_meter_remove,
	.shutdown = s2mpg12_meter_shutdown,
	.id_table = s2mpg12_meter_id,
};

static int __init s2mpg12_meter_init(void)
{
	return platform_driver_register(&s2mpg12_meter_driver);
}

subsys_initcall(s2mpg12_meter_init);

static void __exit s2mpg12_meter_exit(void)
{
	platform_driver_unregister(&s2mpg12_meter_driver);
}

module_exit(s2mpg12_meter_exit);

/* Module information */
MODULE_DESCRIPTION("SAMSUNG S2MPG12 Meter Driver");
MODULE_LICENSE("GPL");
