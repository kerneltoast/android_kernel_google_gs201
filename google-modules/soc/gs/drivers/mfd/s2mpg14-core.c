// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg14-core.c
 *
 * Copyright (C) 2022 Samsung Electronics
 *
 * mfd core driver for the s2mpg14
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mfd/samsung/s2mpg14-register.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <soc/google/acpm_mfd.h>

#if IS_ENABLED(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

static struct device_node *acpm_mfd_node;

static struct mfd_cell s2mpg14_devs[] = {
	{
		.name = "s2mpg14-regulator",
	},
	{
		.name = "s2mpg14-rtc",
	},
	{
		.name = "s2mpg14-meter",
	},
	{
		.name = "s2mpg14_gpio",
	},
	{
		.name = "s2mpg14-power-keys",
	},
};

static u8 s2mpg14_pmic_rev;

int s2mpg14_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpg14_dev *s2mpg14 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg14->i2c_lock);
	ret = exynos_acpm_read_reg(acpm_mfd_node, channel, i2c->addr,
				   reg, dest);
	mutex_unlock(&s2mpg14->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg14_read_reg);

int s2mpg14_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg14_dev *s2mpg14 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg14->i2c_lock);
	ret = exynos_acpm_bulk_read(acpm_mfd_node, channel, i2c->addr,
				    reg, count, buf);
	mutex_unlock(&s2mpg14->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg14_bulk_read);

int s2mpg14_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpg14_dev *s2mpg14 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg14->i2c_lock);
	ret = exynos_acpm_write_reg(acpm_mfd_node, channel,
				    i2c->addr, reg, value);
	mutex_unlock(&s2mpg14->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg14_write_reg);

int s2mpg14_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg14_dev *s2mpg14 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg14->i2c_lock);
	ret = exynos_acpm_bulk_write(acpm_mfd_node, channel,
				     i2c->addr, reg, count, buf);
	mutex_unlock(&s2mpg14->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg14_bulk_write);

int s2mpg14_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpg14_dev *s2mpg14 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg14->i2c_lock);
	ret = exynos_acpm_update_reg(acpm_mfd_node, channel,
				     i2c->addr, reg, val, mask);
	mutex_unlock(&s2mpg14->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg14_update_reg);

u8 s2mpg14_get_rev_id(void)
{
	return s2mpg14_pmic_rev;
}
EXPORT_SYMBOL_GPL(s2mpg14_get_rev_id);

struct i2c_client *s2mpg14_get_i2c_client(struct s2mpg14_dev *dev,
					  unsigned int reg)
{
	struct i2c_client *client = NULL;

	if (reg >> 8 == I2C_ADDR_TOP)
		client = dev->i2c;
	else if (reg >> 8 == I2C_ADDR_PMIC)
		client = dev->pmic;
	else if (reg >> 8 == I2C_ADDR_RTC)
		client = dev->rtc;
	else if (reg >> 8 == I2C_ADDR_METER)
		client = dev->meter;
	else if (reg >> 8 == I2C_ADDR_WLWP)
		client = dev->wlwp;
	else if (reg >> 8 == I2C_ADDR_GPIO)
		client = dev->gpio;

	return client;
}

int s2mpg14_regmap_read_reg(void *context, unsigned int reg,
			    unsigned int *dest)
{
	u8 ureg = reg;
	u8 *udest = (u8 *)dest;
	struct s2mpg14_dev *dev = context;
	struct i2c_client *client = s2mpg14_get_i2c_client(dev, reg);

	if (!client)
		return -EFAULT;

	*dest = 0;
	return s2mpg14_read_reg(client, ureg, udest);
}

int s2mpg14_regmap_write_reg(void *context, unsigned int reg,
			     unsigned int value)
{
	u8 ureg = reg;
	u8 uvalue = value;
	struct s2mpg14_dev *dev = context;
	struct i2c_client *client = s2mpg14_get_i2c_client(dev, reg);

	if (!client)
		return -EFAULT;

	return s2mpg14_write_reg(client, ureg, uvalue);
}

static const struct regmap_range s2mpg14_valid_regs[] = {
	regmap_reg_range(0x000, 0x029), /* Common Block */
	regmap_reg_range(0x100, 0x1E4), /* Power Management Block */
	regmap_reg_range(0x200, 0x230), /* RTC (real-time clock) */
	regmap_reg_range(0xA00, 0xA5A), /* Power Meter config */
	regmap_reg_range(0xA63, 0xAE5), /* Power Meter data */
	regmap_reg_range(0xB00, 0xB10), /* WLWP */
	regmap_reg_range(0xC04, 0xC10), /* GPIO */
};

static const struct regmap_range s2mpg14_read_only_regs[] = {
	regmap_reg_range(0x000, 0x029), /* Common Block */
	regmap_reg_range(0x100, 0x104), /* Power Management INT1~5 */
	regmap_reg_range(0x10A, 0x10E), /* Power Management STATUS */
	regmap_reg_range(0xA63, 0xAE5), /* Power Meter data */
	regmap_reg_range(0xC04, 0xC04), /* GPIO */
};

const struct regmap_access_table s2mpg14_read_register_set = {
	.yes_ranges = s2mpg14_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(s2mpg14_valid_regs),
};

const struct regmap_access_table s2mpg14_write_register_set = {
	.yes_ranges = s2mpg14_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(s2mpg14_valid_regs),
	.no_ranges = s2mpg14_read_only_regs,
	.n_no_ranges = ARRAY_SIZE(s2mpg14_read_only_regs),
};

static struct regmap_config s2mpg14_regmap_config = {
	.name = "s2mpg14",
	.reg_bits = 12,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0xC10,
	.reg_read = s2mpg14_regmap_read_reg,
	.reg_write = s2mpg14_regmap_write_reg,
	.rd_table = &s2mpg14_read_register_set,
	.wr_table = &s2mpg14_write_register_set,
};

#if IS_ENABLED(CONFIG_OF)
static int of_s2mpg14_dt(struct device *dev,
			 struct s2mpg14_platform_data *pdata,
			 struct s2mpg14_dev *s2mpg14)
{
	struct device_node *np = dev->of_node;
	int ret;
	const char *status;
	u32 val;

	if (!np)
		return -EINVAL;

	acpm_mfd_node = np;

	status = of_get_property(np, "s2mpg14,wakeup", NULL);
	if (!status)
		return -EINVAL;
	pdata->wakeup = !strcmp(status, "enabled") || !strcmp(status, "okay");

	/* WTSR, SMPL */
	pdata->wtsr_smpl =
		devm_kzalloc(dev, sizeof(*pdata->wtsr_smpl), GFP_KERNEL);
	if (!pdata->wtsr_smpl)
		return -ENOMEM;

	ret = of_property_read_u32(np, "wtsr_en", &val);
	if (ret)
		return -EINVAL;
	pdata->wtsr_smpl->wtsr_en = !!val;

	status = of_get_property(np, "smpl_en", NULL);
	if (!status)
		return -EINVAL;
	pdata->wtsr_smpl->smpl_en = !strcmp(status, "enabled") || !strcmp(status, "okay");

	ret = of_property_read_u32(np, "wtsr_timer_val",
				   &pdata->wtsr_smpl->wtsr_timer_val);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "smpl_timer_val",
				   &pdata->wtsr_smpl->smpl_timer_val);
	if (ret)
		return -EINVAL;

	status = of_get_property(np, "coldrst_en", NULL);
	if (!status)
		return -EINVAL;
	pdata->wtsr_smpl->coldrst_en = !strcmp(status, "enabled") || !strcmp(status, "okay");

	ret = of_property_read_u32(np, "coldrst_timer_val",
				   &pdata->wtsr_smpl->coldrst_timer_val);
	if (ret)
		return -EINVAL;

	status = of_get_property(np, "sub_smpl_en", NULL);
	if (!status)
		return -EINVAL;
	pdata->wtsr_smpl->sub_smpl_en = !strcmp(status, "enabled") || !strcmp(status, "okay");

	ret = of_property_read_u32(np, "check_jigon", &val);
	if (ret)
		return -EINVAL;
	pdata->wtsr_smpl->check_jigon = !!val;

	/* init time */
	pdata->init_time =
		devm_kzalloc(dev, sizeof(*pdata->init_time), GFP_KERNEL);
	if (!pdata->init_time)
		return -ENOMEM;

	ret = of_property_read_u32(np, "init_time,sec",
				   &pdata->init_time->tm_sec);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,min",
				   &pdata->init_time->tm_min);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,hour",
				   &pdata->init_time->tm_hour);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,mday",
				   &pdata->init_time->tm_mday);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,mon",
				   &pdata->init_time->tm_mon);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,year",
				   &pdata->init_time->tm_year);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,wday",
				   &pdata->init_time->tm_wday);
	if (ret)
		return -EINVAL;

	/* rtc optimize */
	ret = of_property_read_u32(np, "osc-bias-up", &val);
	if (!ret)
		pdata->osc_bias_up = val;
	else
		pdata->osc_bias_up = -1;

	ret = of_property_read_u32(np, "rtc_cap_sel", &val);
	if (!ret)
		pdata->cap_sel = val;
	else
		pdata->cap_sel = -1;

	ret = of_property_read_u32(np, "rtc_osc_xin", &val);
	if (!ret)
		pdata->osc_xin = val;
	else
		pdata->osc_xin = -1;

	ret = of_property_read_u32(np, "rtc_osc_xout", &val);
	if (!ret)
		pdata->osc_xout = val;
	else
		pdata->osc_xout = -1;

	return 0;
}
#else
static int of_s2mpg14_dt(struct device *dev,
			 struct s2mpg14_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static int s2mpg14_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *dev_id)
{
	struct s2mpg14_dev *s2mpg14;
	struct s2mpg14_platform_data *pdata = i2c->dev.platform_data;
	u8 reg_data;
	int ret = 0;

	dev_dbg(&i2c->dev, "%s i2c probe\n", S2MPG14_MFD_DEV_NAME);

	s2mpg14 = devm_kzalloc(&i2c->dev, sizeof(*s2mpg14), GFP_KERNEL);
	if (!s2mpg14)
		return -ENOMEM;

	pdata = devm_kzalloc(&i2c->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = of_s2mpg14_dt(&i2c->dev, pdata, s2mpg14);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to get device of_node\n");
		return ret;
	}
	i2c->dev.platform_data = pdata;

	pdata->irq_base = devm_irq_alloc_descs(&i2c->dev, -1, 0,
					       S2MPG14_IRQ_NR, -1);
	if (pdata->irq_base < 0) {
		dev_err(&i2c->dev, "%s: devm_irq_alloc_desc Fail!: %d\n",
			__func__, pdata->irq_base);
		return -EINVAL;
	}

	s2mpg14->dev = &i2c->dev;
	s2mpg14->dev->init_name = "i2c-" S2MPG14_MFD_DEV_NAME;
	i2c->addr = I2C_ADDR_TOP;
	s2mpg14->i2c = i2c;
	s2mpg14->irq = i2c->irq;
	s2mpg14->device_type = S2MPG14X;
	s2mpg14->pdata = pdata;
	s2mpg14->wakeup = pdata->wakeup;
	s2mpg14->irq_base = pdata->irq_base;

	mutex_init(&s2mpg14->i2c_lock);

	s2mpg14->pmic = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PMIC);
	s2mpg14->rtc = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_RTC);
	s2mpg14->meter = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_METER);
	s2mpg14->wlwp = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_WLWP);
	s2mpg14->gpio = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_GPIO);
	s2mpg14->mt_trim = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_MT_TRIM);
	s2mpg14->pm_trim1 = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PM_TRIM1);
	s2mpg14->pm_trim2 = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PM_TRIM2);

	i2c_set_clientdata(s2mpg14->i2c, s2mpg14);
	i2c_set_clientdata(s2mpg14->pmic, s2mpg14);
	i2c_set_clientdata(s2mpg14->rtc, s2mpg14);
	i2c_set_clientdata(s2mpg14->meter, s2mpg14);
	i2c_set_clientdata(s2mpg14->wlwp, s2mpg14);
	i2c_set_clientdata(s2mpg14->gpio, s2mpg14);
	i2c_set_clientdata(s2mpg14->mt_trim, s2mpg14);
	i2c_set_clientdata(s2mpg14->pm_trim1, s2mpg14);
	i2c_set_clientdata(s2mpg14->pm_trim2, s2mpg14);

	ret = s2mpg14_read_reg(i2c, S2MPG14_COMMON_CHIPID, &reg_data);
	if (ret < 0) {
		dev_warn(s2mpg14->dev,
			"device not found on this channel (not an error)\n");
		goto err_w_lock;
	}
	s2mpg14->pmic_rev = reg_data;
	s2mpg14_pmic_rev = s2mpg14->pmic_rev;

	dev_info(s2mpg14->dev, "device found: rev.0x%02x\n", s2mpg14->pmic_rev);

	s2mpg14->regmap = devm_regmap_init(s2mpg14->dev, NULL, s2mpg14,
					   &s2mpg14_regmap_config);
	if (IS_ERR(s2mpg14->regmap)) {
		dev_err(s2mpg14->dev, "regmap_init failed!\n");
		ret = PTR_ERR(s2mpg14->regmap);
		goto err_w_lock;
	}

	ret = s2mpg14_irq_init(s2mpg14);
	if (ret < 0)
		goto err_irq_init;

	ret = mfd_add_devices(s2mpg14->dev, -1, s2mpg14_devs,
			      ARRAY_SIZE(s2mpg14_devs), NULL, 0, NULL);
	if (ret < 0)
		goto err_mfd;

	ret = device_init_wakeup(s2mpg14->dev, pdata->wakeup);
		if (ret < 0) {
		dev_err(&i2c->dev, "%s: device_init_wakeup fail(%d)\n", __func__, ret);
		goto err_mfd;
	}

	return ret;
err_mfd:
	mfd_remove_devices(s2mpg14->dev);
err_irq_init:
	s2mpg14_irq_exit(s2mpg14);
err_w_lock:
	mutex_destroy(&s2mpg14->i2c_lock);
	return ret;
}

static void s2mpg14_i2c_remove(struct i2c_client *i2c)
{
	struct s2mpg14_dev *s2mpg14 = i2c_get_clientdata(i2c);

	mfd_remove_devices(s2mpg14->dev);
	i2c_unregister_device(s2mpg14->i2c);
}

static const struct i2c_device_id s2mpg14_i2c_id[] = {
	{ S2MPG14_MFD_DEV_NAME, TYPE_S2MPG14 }, {} };

MODULE_DEVICE_TABLE(i2c, s2mpg14_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s2mpg14_i2c_dt_ids[] = {
	{ .compatible = "samsung,s2mpg14mfd" },
	{},
};
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_PM)
static int s2mpg14_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct s2mpg14_dev *s2mpg14 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(s2mpg14->irq);

	disable_irq(s2mpg14->irq);

	return 0;
}

static int s2mpg14_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct s2mpg14_dev *s2mpg14 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		disable_irq_wake(s2mpg14->irq);

	enable_irq(s2mpg14->irq);

	return 0;
}
#else
#define s2mpg14_suspend NULL
#define s2mpg14_resume NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops s2mpg14_pm = {
	.suspend_late = s2mpg14_suspend,
	.resume_early = s2mpg14_resume,
};

static struct i2c_driver s2mpg14_i2c_driver = {
	.driver = {
		   .name = S2MPG14_MFD_DEV_NAME,
		   .owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &s2mpg14_pm,
#endif /* CONFIG_PM */
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = s2mpg14_i2c_dt_ids,
#endif /* CONFIG_OF */
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg14_i2c_probe,
	.remove = s2mpg14_i2c_remove,
	.id_table = s2mpg14_i2c_id,
};

static int __init s2mpg14_i2c_init(void)
{
	return i2c_add_driver(&s2mpg14_i2c_driver);
}

/* init early so consumer devices can complete system boot */
subsys_initcall(s2mpg14_i2c_init);

static void __exit s2mpg14_i2c_exit(void)
{
	i2c_del_driver(&s2mpg14_i2c_driver);
}

module_exit(s2mpg14_i2c_exit);

MODULE_SOFTDEP("pre: s2mpg15-mfd");
MODULE_DESCRIPTION("s2mpg14 multi-function core driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
