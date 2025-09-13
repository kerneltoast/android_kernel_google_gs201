// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg15-powermeter.c
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd
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
#include <linux/mfd/samsung/s2mpg15.h>
#include <linux/mfd/samsung/s2mpg15-register.h>
#include <linux/mfd/samsung/s2mpg1415-meter.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/regulator/pmic_class.h>
#include <linux/mfd/core.h>

#if IS_ENABLED(CONFIG_ODPM)
static struct mfd_cell s2mpg15_meter_devs[] = {
	{
		.name="s2mpg15-odpm",
	},
};
#endif

u32 s2mpg15_muxsel_to_current_resolution(enum s2mpg1415_meter_muxsel m)
{
	switch (m) {
	case MUXSEL_NONE:
	case BUCK3:
	case BUCK4:
	case BUCK5:
	case BUCK6:
	case BUCK8:
	case BUCK9:
	case BUCK10:
	case BUCK11:
	case BUCKC:
		return CMS_BUCK_CURRENT;
	case BUCK1:
	case BUCK2:
	case BUCK12:
		return CMD_BUCK_CURRENT;
	case BUCK7:
	case BUCKD:
	case BUCKA:
		return VM_CURRENT;
	case BUCKBOOST:
		return BB_CURRENT;
	case LDO9:
		return NLDO_CURRENT_150mA;
	case LDO4:
	case LDO5:
	case LDO6:
	case LDO7:
	case LDO8:
	case LDO10:
	case LDO11:
	case LDO15:
	case LDO16:
	case LDO17:
	case LDO18:
	case LDO19:
	case LDO20:
	case LDO22:
	case LDO25:
	case LDO26:
	case LDO27:
	case LDO28:
	case LDO29:
		return PLDO_CURRENT_150mA;
	case LDO12:
	case LDO13:
	case LDO14:
		return PLDO_CURRENT_300mA;
	case LDO3:
		return NLDO_CURRENT_450mA;
	case LDO23:
		return DVS_NLDO_CURRENT_800mA;
	case LDO21:
	case LDO24:
		return NLDO_CURRENT_800mA;
	case LDO1:
		return DVS_NLDO_CURRENT_1200mA;
	case LDO2:
		return NLDO_CURRENT_1200mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}
EXPORT_SYMBOL_GPL(s2mpg15_muxsel_to_current_resolution);

u32 s2mpg15_muxsel_to_power_resolution(enum s2mpg1415_meter_muxsel m)
{
	switch (m) {
	case MUXSEL_NONE:
	case BUCK3:
	case BUCK4:
	case BUCK5:
	case BUCK6:
	case BUCK8:
	case BUCK9:
	case BUCK10:
	case BUCK11:
	case BUCKC:
		return CMS_BUCK_POWER;
	case BUCK1:
	case BUCK2:
	case BUCK12:
		return CMD_BUCK_POWER;
	case BUCK7:
	case BUCKD:
	case BUCKA:
		return VM_POWER;
	case BUCKBOOST:
		return BB_POWER;
	case LDO9:
		return NLDO_POWER_150mA;
	case LDO4:
	case LDO5:
	case LDO6:
	case LDO7:
	case LDO8:
	case LDO10:
	case LDO11:
	case LDO15:
	case LDO16:
	case LDO17:
	case LDO18:
	case LDO19:
	case LDO20:
	case LDO22:
	case LDO25:
	case LDO26:
	case LDO27:
	case LDO28:
	case LDO29:
		return PLDO_POWER_150mA;
	case LDO12:
	case LDO13:
	case LDO14:
		return PLDO_POWER_300mA;
	case LDO3:
		return NLDO_POWER_450mA;
	case LDO23:
		return DVS_NLDO_POWER_800mA_OFF;
	case LDO21:
	case LDO24:
		return NLDO_POWER_800mA;
	case LDO1:
		return DVS_NLDO_POWER_1200mA_ON;
	case LDO2:
		return NLDO_POWER_1200mA;
	default:
		pr_err("%s: wrong muxsel\n", __func__);
		return INVALID_RESOLUTION;
	}
}
EXPORT_SYMBOL_GPL(s2mpg15_muxsel_to_power_resolution);

static const char *muxsel_to_str(enum s2mpg1415_meter_muxsel m)
{
	char *ret;

	switch (m) {
		ENUM_STR(MUXSEL_NONE, "S", ret);
		ENUM_STR(BUCK1, "S", ret);
		ENUM_STR(BUCK2, "S", ret);
		ENUM_STR(BUCK3, "S", ret);
		ENUM_STR(BUCK4, "S", ret);
		ENUM_STR(BUCK5, "S", ret);
		ENUM_STR(BUCK6, "S", ret);
		ENUM_STR(BUCK7, "S", ret);
		ENUM_STR(BUCK8, "S", ret);
		ENUM_STR(BUCK9, "S", ret);
		ENUM_STR(BUCK10, "S", ret);
		ENUM_STR(BUCK11, "S", ret);
		ENUM_STR(BUCK12, "S", ret);
		ENUM_STR(BUCKD, "", ret);
		ENUM_STR(BUCKA, "", ret);
		ENUM_STR(BUCKC, "", ret);
		ENUM_STR(BUCKBOOST, "", ret);
		ENUM_STR(VSEN_P4, "", ret);
		ENUM_STR(VSEN_P5, "", ret);
		ENUM_STR(VSEN_P6, "", ret);
		ENUM_STR(LDO1, "S", ret);
		ENUM_STR(LDO2, "S", ret);
		ENUM_STR(LDO3, "S", ret);
		ENUM_STR(LDO4, "S", ret);
		ENUM_STR(LDO5, "S", ret);
		ENUM_STR(LDO6, "S", ret);
		ENUM_STR(LDO7, "S", ret);
		ENUM_STR(LDO8, "S", ret);
		ENUM_STR(LDO9, "S", ret);
		ENUM_STR(LDO10, "S", ret);
		ENUM_STR(LDO11, "S", ret);
		ENUM_STR(LDO12, "S", ret);
		ENUM_STR(LDO13, "S", ret);
		ENUM_STR(LDO14, "S", ret);
		ENUM_STR(LDO15, "S", ret);
		ENUM_STR(LDO16, "S", ret);
		ENUM_STR(LDO17, "S", ret);
		ENUM_STR(LDO18, "S", ret);
		ENUM_STR(LDO19, "S", ret);
		ENUM_STR(LDO20, "S", ret);
		ENUM_STR(LDO21, "S", ret);
		ENUM_STR(LDO22, "S", ret);
		ENUM_STR(LDO23, "S", ret);
		ENUM_STR(LDO24, "S", ret);
		ENUM_STR(LDO25, "S", ret);
		ENUM_STR(LDO26, "S", ret);
		ENUM_STR(LDO27, "S", ret);
		ENUM_STR(LDO28, "S", ret);
		ENUM_STR(LDO29, "S", ret);
	default:
		return "invalid";
	}
	return ret;
}

int s2mpg15_meter_onoff(struct s2mpg15_meter *s2mpg15, bool onoff)
{
	dev_dbg(s2mpg15->dev, "s2mpg15 meter %s\n", onoff ? "on" : "off");

	return s2mpg15_update_reg(s2mpg15->i2c, S2MPG15_METER_CTRL1,
				  onoff ? METER_EN_MASK : 0, METER_EN_MASK);
}
EXPORT_SYMBOL_GPL(s2mpg15_meter_onoff);

int s2mpg15_ext_meter_onoff(struct s2mpg15_meter *s2mpg15, bool onoff)
{
	dev_dbg(s2mpg15->dev, "s2mpg15 external meter %s\n",
		onoff ? "on" : "off");

	return s2mpg15_update_reg(s2mpg15->i2c, S2MPG15_METER_CTRL1,
				  onoff ? EXT_METER_EN_MASK : 0,
				  EXT_METER_EN_MASK);
}
EXPORT_SYMBOL_GPL(s2mpg15_ext_meter_onoff);

int s2mpg15_meter_set_muxsel(struct s2mpg15_meter *s2mpg15, int channel,
			     enum s2mpg1415_meter_muxsel m)
{
	int reg = S2MPG15_METER_MUXSEL0;
	int ret = -EPERM;

	if (channel < 0 || channel >= S2MPG1415_METER_CHANNEL_MAX) {
		dev_err(s2mpg15->dev, "invalid channel number\n");
		return ret;
	}

	dev_dbg(s2mpg15->dev, "CH%d, %s\n", channel, muxsel_to_str(m));

	reg += channel;

	mutex_lock(&s2mpg15->meter_lock);
	ret = s2mpg15_update_reg(s2mpg15->i2c, reg, m, MUXSEL_MASK);

	s2mpg15->chg_mux_sel[channel] = m;
	mutex_unlock(&s2mpg15->meter_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg15_meter_set_muxsel);

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
static ssize_t s2mpg15_muxsel_table_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int muxsel_cnt = 0;
	int muxsel = BUCK1;
	size_t size = 0;

	while (muxsel <= LDO29) {
		size += scnprintf(buf + size, PAGE_SIZE - size, "%s : 0x%x , ",
				  muxsel_to_str(muxsel), muxsel);
		if (muxsel == LDO29)
			break;

		switch (muxsel) {
		case BUCKC:
			muxsel = BUCKBOOST;
			break;
		case BUCKBOOST:
			muxsel = VSEN_P4;
			break;
		case VSEN_P6:
			muxsel = LDO1;
			break;
		default:
			muxsel++;
			break;
		}

		muxsel_cnt++;
		if (!(muxsel_cnt % 8))
			size += scnprintf(buf + size, PAGE_SIZE - size, "\n");
	}

	return size;
}

static ssize_t s2mpg15_channel_muxsel_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct s2mpg15_meter *s2mpg15 = dev_get_drvdata(dev);
	int channel, muxsel;
	int ret;

	if (!buf) {
		dev_err(s2mpg15->dev, "empty buffer\n");
		return -EINVAL;
	}

	ret = sscanf(buf, "%d %x", &channel, &muxsel);
	if (ret != 2) {
		dev_err(s2mpg15->dev, "input error\n");
		return -EINVAL;
	}

	if (channel < 0 || channel >= S2MPG1415_METER_CHANNEL_MAX) {
		dev_err(s2mpg15->dev, "wrong channel %d\n", channel);
		return -EINVAL;
	}

	if ((muxsel >= BUCK1 && muxsel <= BUCKC) ||
	    muxsel == BUCKBOOST ||
	    (muxsel >= VSEN_P4 && muxsel <= VSEN_P6) ||
	    (muxsel >= LDO1 && muxsel <= LDO29)) {
		s2mpg15_meter_set_muxsel(s2mpg15, channel, muxsel);
		return size;
	}

	dev_err(s2mpg15->dev, "wrong muxsel 0x%x\n", muxsel);
	return -EINVAL;
}

static ssize_t s2mpg15_channel_muxsel_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct s2mpg15_meter *s2mpg15 = dev_get_drvdata(dev);
	int i;
	size_t count = 0;

	mutex_lock(&s2mpg15->meter_lock);

	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "CH%d[%s], ",
			 i, muxsel_to_str(s2mpg15->chg_mux_sel[i]));
	}

	mutex_unlock(&s2mpg15->meter_lock);
	return count;
}

static ssize_t s2mpg15_lpf_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg15_meter *s2mpg15 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg15->meter_lock);

	s2mpg1415_meter_set_lpf_mode(ID_S2MPG15, s2mpg15->i2c,
				     S2MPG1415_METER_CURRENT);
	s2mpg1415_meter_read_lpf_data_reg(ID_S2MPG15, s2mpg15->i2c,
					  s2mpg15->lpf_data);

	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		enum s2mpg1415_meter_muxsel muxsel = s2mpg15->chg_mux_sel[i];

		count += s2mpg1415_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mA)",
			s2mpg15->lpf_data[i],
			s2mpg15_muxsel_to_current_resolution(muxsel), 1);
	}
	mutex_unlock(&s2mpg15->meter_lock);
	return count;
}

static ssize_t s2mpg15_lpf_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg15_meter *s2mpg15 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	mutex_lock(&s2mpg15->meter_lock);

	s2mpg1415_meter_set_lpf_mode(ID_S2MPG15, s2mpg15->i2c,
				     S2MPG1415_METER_POWER);
	s2mpg1415_meter_read_lpf_data_reg(ID_S2MPG15, s2mpg15->i2c,
					  s2mpg15->lpf_data);

	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		enum s2mpg1415_meter_muxsel muxsel = s2mpg15->chg_mux_sel[i];

		count += s2mpg1415_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			s2mpg15->lpf_data[i],
			s2mpg15_muxsel_to_power_resolution(muxsel),
			1);
	}
	mutex_unlock(&s2mpg15->meter_lock);
	return count;
}

static ssize_t s2mpg15_acc_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct s2mpg15_meter *s2mpg15 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1415_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg1415_meter_measure_acc(ID_S2MPG15, s2mpg15->i2c,
				    &s2mpg15->meter_lock,
				    S2MPG1415_METER_CURRENT, acc_data,
				    &acc_count, NULL, INT_125HZ);

	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		enum s2mpg1415_meter_muxsel muxsel = s2mpg15->chg_mux_sel[i];

		count += s2mpg1415_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mA)",
			acc_data[i],
			s2mpg15_muxsel_to_current_resolution(muxsel),
			acc_count);
	}

	return count;
}

static ssize_t s2mpg15_acc_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg15_meter *s2mpg15 = dev_get_drvdata(dev);
	int i;
	ssize_t count = 0;

	u64 acc_data[S2MPG1415_METER_CHANNEL_MAX];
	u32 acc_count;

	s2mpg1415_meter_measure_acc(ID_S2MPG15, s2mpg15->i2c,
				    &s2mpg15->meter_lock,
				    S2MPG1415_METER_POWER, acc_data,
				    &acc_count, NULL, INT_125HZ);

	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		enum s2mpg1415_meter_muxsel muxsel = s2mpg15->chg_mux_sel[i];

		count += s2mpg1415_meter_format_channel(buf, count, i,
			muxsel_to_str(muxsel), "(mW)",
			acc_data[i],
			s2mpg15_muxsel_to_power_resolution(muxsel),
			acc_count);
	}

	return count;
}

static DEVICE_ATTR_RO(s2mpg15_muxsel_table);
static DEVICE_ATTR_RW(s2mpg15_channel_muxsel);
static DEVICE_ATTR_RO(s2mpg15_lpf_current);
static DEVICE_ATTR_RO(s2mpg15_lpf_power);
static DEVICE_ATTR_RO(s2mpg15_acc_current);
static DEVICE_ATTR_RO(s2mpg15_acc_power);

int create_s2mpg15_meter_sysfs(struct s2mpg15_meter *s2mpg15)
{
	struct device *s2mpg15_meter_dev = s2mpg15->dev;
	int err = -ENODEV;

	s2mpg15_meter_dev = pmic_device_create(s2mpg15, "s2mpg15-meter");

	err = device_create_file(s2mpg15_meter_dev,
				 &dev_attr_s2mpg15_lpf_current);
	if (err) {
		dev_err(s2mpg15->dev,
			"s2mpg15_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg15_lpf_current.attr.name);
	}

	err = device_create_file(s2mpg15_meter_dev,
				 &dev_attr_s2mpg15_lpf_power);
	if (err) {
		dev_err(s2mpg15->dev,
			"s2mpg15_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg15_lpf_power.attr.name);
	}

	err = device_create_file(s2mpg15_meter_dev,
				 &dev_attr_s2mpg15_acc_current);
	if (err) {
		dev_err(s2mpg15->dev,
			"s2mpg15_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg15_acc_current.attr.name);
	}

	err = device_create_file(s2mpg15_meter_dev,
				 &dev_attr_s2mpg15_acc_power);
	if (err) {
		dev_err(s2mpg15->dev,
			"s2mpg15_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg15_acc_power.attr.name);
	}

	err = device_create_file(s2mpg15_meter_dev,
				 &dev_attr_s2mpg15_muxsel_table);
	if (err) {
		dev_err(s2mpg15->dev,
			"s2mpg15_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg15_muxsel_table.attr.name);
	}

	err = device_create_file(s2mpg15_meter_dev,
				 &dev_attr_s2mpg15_channel_muxsel);
	if (err) {
		dev_err(s2mpg15->dev,
			"s2mpg15_sysfs: failed to create device file, %s\n",
				 dev_attr_s2mpg15_channel_muxsel.attr.name);
	}

	return 0;
}
#endif

static int s2mpg15_meter_probe(struct platform_device *pdev)
{
	struct s2mpg15_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg15_platform_data *pdata = iodev->pdata;
	struct s2mpg15_meter *s2mpg15;
	int ret = 0;

	if (!pdata)
		return -ENODEV;

	s2mpg15 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpg15_meter),
			       GFP_KERNEL);
	if (!s2mpg15)
		return -ENOMEM;

	s2mpg15->iodev = iodev;
	s2mpg15->i2c = iodev->meter;
	s2mpg15->dev = &pdev->dev;
	mutex_init(&s2mpg15->meter_lock);
	platform_set_drvdata(pdev, s2mpg15);

#if !IS_ENABLED(CONFIG_ODPM)
	/* initial setting */
	/* set BUCK1S ~ BUCK8S muxsel from CH0 to CH7 */
	/* any necessary settings can be added */
	s2mpg1415_meter_set_int_samp_rate(ID_S2MPG15, s2mpg15->i2c, INT_125HZ);

	s2mpg15_meter_set_muxsel(s2mpg15, 0, BUCK1);
	s2mpg15_meter_set_muxsel(s2mpg15, 1, BUCK2);
	s2mpg15_meter_set_muxsel(s2mpg15, 2, BUCK3);
	s2mpg15_meter_set_muxsel(s2mpg15, 3, BUCK4);
	s2mpg15_meter_set_muxsel(s2mpg15, 4, BUCK5);
	s2mpg15_meter_set_muxsel(s2mpg15, 5, BUCK6);
	s2mpg15_meter_set_muxsel(s2mpg15, 6, BUCK7);
	s2mpg15_meter_set_muxsel(s2mpg15, 7, BUCK8);
	s2mpg15_meter_set_muxsel(s2mpg15, 8, BUCK9);
	s2mpg15_meter_set_muxsel(s2mpg15, 9, BUCK10);
	s2mpg15_meter_set_muxsel(s2mpg15, 10, BUCK11);
	s2mpg15_meter_set_muxsel(s2mpg15, 11, BUCK12);

	s2mpg15_meter_onoff(s2mpg15, true);
	s2mpg15_ext_meter_onoff(s2mpg15, false);

#else
	ret = mfd_add_devices(s2mpg15->dev, -1, s2mpg15_meter_devs,
        ARRAY_SIZE(s2mpg15_meter_devs), NULL, 0, NULL);
	if (ret < 0) {
		mfd_remove_devices(s2mpg15->dev);
                return ret;
	}
#endif

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	/* create sysfs */
	ret = create_s2mpg15_meter_sysfs(s2mpg15);
#endif

	return ret;
}

static int s2mpg15_meter_remove(struct platform_device *pdev)
{
	struct s2mpg15_meter *s2mpg15 = platform_get_drvdata(pdev);

	s2mpg15_meter_onoff(s2mpg15, false);
	s2mpg15_ext_meter_onoff(s2mpg15, false);

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	pmic_device_destroy(s2mpg15->dev->devt);
#endif
	return 0;
}

static void s2mpg15_meter_shutdown(struct platform_device *pdev)
{
	struct s2mpg15_meter *s2mpg15 = platform_get_drvdata(pdev);

	s2mpg15_meter_onoff(s2mpg15, false);
	s2mpg15_ext_meter_onoff(s2mpg15, false);
}

static const struct platform_device_id s2mpg15_meter_id[] = {
	{ "s2mpg15-meter", 0 },
	{},
};

MODULE_DEVICE_TABLE(platform, s2mpg15_meter_id);

static struct platform_driver s2mpg15_meter_driver = {
	.driver = {
		   .name = "s2mpg15-meter",
		   .owner = THIS_MODULE,
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg15_meter_probe,
	.remove = s2mpg15_meter_remove,
	.shutdown = s2mpg15_meter_shutdown,
	.id_table = s2mpg15_meter_id,
};

static int __init s2mpg15_meter_init(void)
{
	return platform_driver_register(&s2mpg15_meter_driver);
}

subsys_initcall(s2mpg15_meter_init);

static void __exit s2mpg15_meter_exit(void)
{
	platform_driver_unregister(&s2mpg15_meter_driver);
}

module_exit(s2mpg15_meter_exit);

/* Module information */
MODULE_DESCRIPTION("SAMSUNG S2MPG15 Meter Driver");
MODULE_LICENSE("GPL");
