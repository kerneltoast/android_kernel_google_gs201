// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg12-core.c
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * mfd core driver for the s2mpg12
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg12-register.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <soc/google/acpm_mfd.h>

#if IS_ENABLED(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#define I2C_ADDR_TOP 0x00
#define I2C_ADDR_PMIC 0x01
#define I2C_ADDR_RTC 0x02
#define I2C_ADDR_METER 0x0A
#define I2C_ADDR_WLWP 0x0B
#define I2C_ADDR_GPIO 0x0C
#define I2C_ADDR_MT_TRIM 0x0E
#define I2C_ADDR_TRIM 0x0F

static struct device_node *acpm_mfd_node;

static struct mfd_cell s2mpg12_devs[] = {
	{
		.name = "s2mpg12-regulator",
	},
	{
		.name = "s2mpg12-rtc",
	},
	{
		.name = "s2mpg12-meter",
	},
	{
		.name = "s2mpg12_gpio",
	},
	{
		.name = "s2mpg12-power-keys",
	},
};

static u8 s2mpg12_pmic_rev;

int s2mpg12_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpg12_dev *s2mpg12 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg12->i2c_lock);
	ret = exynos_acpm_read_reg(acpm_mfd_node, channel, i2c->addr,
				   reg, dest);
	mutex_unlock(&s2mpg12->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg12_read_reg);

int s2mpg12_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg12_dev *s2mpg12 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg12->i2c_lock);
	ret = exynos_acpm_bulk_read(acpm_mfd_node, channel, i2c->addr,
				    reg, count, buf);
	mutex_unlock(&s2mpg12->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg12_bulk_read);

int s2mpg12_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpg12_dev *s2mpg12 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg12->i2c_lock);
	ret = exynos_acpm_write_reg(acpm_mfd_node, channel,
				    i2c->addr, reg, value);
	mutex_unlock(&s2mpg12->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg12_write_reg);

int s2mpg12_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg12_dev *s2mpg12 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg12->i2c_lock);
	ret = exynos_acpm_bulk_write(acpm_mfd_node, channel,
				     i2c->addr, reg, count, buf);
	mutex_unlock(&s2mpg12->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg12_bulk_write);

int s2mpg12_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpg12_dev *s2mpg12 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg12->i2c_lock);
	ret = exynos_acpm_update_reg(acpm_mfd_node, channel,
				     i2c->addr, reg, val, mask);
	mutex_unlock(&s2mpg12->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg12_update_reg);

u8 s2mpg12_get_rev_id(void)
{
	return s2mpg12_pmic_rev;
}
EXPORT_SYMBOL_GPL(s2mpg12_get_rev_id);

struct i2c_client *s2mpg12_get_i2c_client(struct s2mpg12_dev *dev,
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
	else if (reg >> 8 == I2C_ADDR_GPIO)
		client = dev->gpio;

	return client;
}

int s2mpg12_regmap_read_reg(void *context, unsigned int reg,
			    unsigned int *dest)
{
	u8 ureg = reg;
	u8 *udest = (u8 *)dest;
	struct s2mpg12_dev *dev = context;
	struct i2c_client *client = s2mpg12_get_i2c_client(dev, reg);

	if (!client)
		return -EFAULT;

	*dest = 0;
	return s2mpg12_read_reg(client, ureg, udest);
}

int s2mpg12_regmap_write_reg(void *context, unsigned int reg,
			     unsigned int value)
{
	u8 ureg = reg;
	u8 uvalue = value;
	struct s2mpg12_dev *dev = context;
	struct i2c_client *client = s2mpg12_get_i2c_client(dev, reg);

	if (!client)
		return -EFAULT;

	return s2mpg12_write_reg(client, ureg, uvalue);
}

static const struct regmap_range s2mpg12_valid_regs[] = {
	regmap_reg_range(0x000, 0x003), /* Common Block - VGPIO */
	regmap_reg_range(0x004, 0x029), /* Common Block */
	regmap_reg_range(0x100, 0x1EC), /* Power Management Block */
	regmap_reg_range(0x200, 0x230), /* RTC (real-time clock) */
	regmap_reg_range(0xA00, 0xA5A), /* Meter config */
	regmap_reg_range(0xA63, 0xAD3), /* Meter data */
	regmap_reg_range(0xC05, 0xC10), /* GPIO */
};

static const struct regmap_range s2mpg12_read_only_regs[] = {
	regmap_reg_range(0x000, 0x00B), /* Common Block */
	regmap_reg_range(0x020, 0x023), /* Common Block */
	regmap_reg_range(0x027, 0x029), /* Common Block */
	regmap_reg_range(0x100, 0x104), /* INT1~5 */
	regmap_reg_range(0x10A, 0x10B), /* STATUS */
	regmap_reg_range(0xA63, 0xAD3), /* Meter data */
};

const struct regmap_access_table s2mpg12_read_register_set = {
	.yes_ranges = s2mpg12_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(s2mpg12_valid_regs),
};

const struct regmap_access_table s2mpg12_write_register_set = {
	.yes_ranges = s2mpg12_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(s2mpg12_valid_regs),
	.no_ranges = s2mpg12_read_only_regs,
	.n_no_ranges = ARRAY_SIZE(s2mpg12_read_only_regs),
};

static struct regmap_config s2mpg12_regmap_config = {
	.name = "s2mpg12",
	.reg_bits = 12,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0xC10,
	.reg_read = s2mpg12_regmap_read_reg,
	.reg_write = s2mpg12_regmap_write_reg,
	.rd_table = &s2mpg12_read_register_set,
	.wr_table = &s2mpg12_write_register_set,
};

#if IS_ENABLED(CONFIG_OF)
static int of_s2mpg12_dt(struct device *dev,
			 struct s2mpg12_platform_data *pdata,
			 struct s2mpg12_dev *s2mpg12)
{
	struct device_node *np = dev->of_node;
	int ret;
	const char *status;
	u32 val;

	if (!np)
		return -EINVAL;

	acpm_mfd_node = np;

	status = of_get_property(np, "s2mpg12,wakeup", NULL);
	if (!status)
		return -EINVAL;
	pdata->wakeup = !strcmp(status, "enabled") || !strcmp(status, "okay");

	/* WTSR, SMPL */
	pdata->wtsr_smpl =
		devm_kzalloc(dev, sizeof(*pdata->wtsr_smpl), GFP_KERNEL);
	if (!pdata->wtsr_smpl)
		return -ENOMEM;

	status = of_get_property(np, "wtsr_en", NULL);
	if (!status)
		return -EINVAL;
	pdata->wtsr_smpl->wtsr_en = !strcmp(status, "enabled") || !strcmp(status, "okay");

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
static int of_s2mpg12_dt(struct device *dev,
			 struct s2mpg12_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static void s2mpg12_set_rev_id(struct s2mpg12_dev *s2mpg12, int id)
{
	if (id == 0x0 || id == 0x1 || id == 0x2)
		s2mpg12->pmic_rev = S2MPG12_EVT0;
	else
		s2mpg12->pmic_rev = S2MPG12_EVT1;
}

static int s2mpg12_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *dev_id)
{
	struct s2mpg12_dev *s2mpg12;
	struct s2mpg12_platform_data *pdata = i2c->dev.platform_data;
	u8 reg_data;
	int ret = 0;

	dev_info(&i2c->dev, "%s i2c probe\n", S2MPG12_MFD_DEV_NAME);

	s2mpg12 = kzalloc(sizeof(*s2mpg12), GFP_KERNEL);
	if (!s2mpg12)
		return -ENOMEM;

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
				     sizeof(struct s2mpg12_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err;
		}

		ret = of_s2mpg12_dt(&i2c->dev, pdata, s2mpg12);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to get device of_node\n");
			goto err;
		}

		i2c->dev.platform_data = pdata;
	} else {
		pdata = i2c->dev.platform_data;
	}

	s2mpg12->dev = &i2c->dev;
	s2mpg12->dev->init_name = "i2c-" S2MPG12_MFD_DEV_NAME;
	i2c->addr = I2C_ADDR_TOP;
	s2mpg12->i2c = i2c;
	s2mpg12->irq = i2c->irq;
	s2mpg12->device_type = S2MPG12X;

	if (pdata) {
		s2mpg12->pdata = pdata;

		pdata->irq_base = irq_alloc_descs(-1, 0, S2MPG12_IRQ_NR, -1);
		if (pdata->irq_base < 0) {
			pr_err("%s:%s irq_alloc_descs Fail! ret(%d)\n",
			       S2MPG12_MFD_DEV_NAME, __func__, pdata->irq_base);
			ret = -EINVAL;
			goto err;
		} else {
			s2mpg12->irq_base = pdata->irq_base;
		}

		s2mpg12->wakeup = pdata->wakeup;
	} else {
		ret = -EINVAL;
		goto err;
	}
	mutex_init(&s2mpg12->i2c_lock);

	i2c_set_clientdata(i2c, s2mpg12);

	if (s2mpg12_read_reg(i2c, S2MPG12_COMMON_CHIPID, &reg_data) < 0) {
		dev_err(s2mpg12->dev,
			"device not found on this channel (this is not an error)\n");
		ret = -ENODEV;
		goto err_w_lock;
	} else {
		s2mpg12_set_rev_id(s2mpg12, reg_data & 0x7);
	}

	s2mpg12->pmic = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PMIC);
	s2mpg12->rtc = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_RTC);
	s2mpg12->meter = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_METER);
	s2mpg12->wlwp = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_WLWP);
	s2mpg12->gpio = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_GPIO);
	s2mpg12->mt_trim = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_MT_TRIM);
	s2mpg12->trim = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_TRIM);
	s2mpg12_pmic_rev = s2mpg12->pmic_rev;

	i2c_set_clientdata(s2mpg12->pmic, s2mpg12);
	i2c_set_clientdata(s2mpg12->rtc, s2mpg12);
	i2c_set_clientdata(s2mpg12->meter, s2mpg12);
	i2c_set_clientdata(s2mpg12->wlwp, s2mpg12);
	i2c_set_clientdata(s2mpg12->gpio, s2mpg12);
	i2c_set_clientdata(s2mpg12->mt_trim, s2mpg12);
	i2c_set_clientdata(s2mpg12->trim, s2mpg12);

	pr_info("%s device found: rev.0x%02x\n", __func__, s2mpg12->pmic_rev);

	s2mpg12->regmap = devm_regmap_init(s2mpg12->dev, NULL, s2mpg12,
					   &s2mpg12_regmap_config);
	if (IS_ERR(s2mpg12->regmap)) {
		dev_err(s2mpg12->dev, "regmap_init failed!\n");
		ret = PTR_ERR(s2mpg12->regmap);
		goto err_w_lock;
	}

	ret = s2mpg12_irq_init(s2mpg12);
	if (ret < 0)
		goto err_irq_init;

	ret = mfd_add_devices(s2mpg12->dev, -1, s2mpg12_devs,
			      ARRAY_SIZE(s2mpg12_devs), NULL, 0, NULL);
	if (ret < 0)
		goto err_mfd;

	device_init_wakeup(s2mpg12->dev, pdata->wakeup);

	return ret;

err_mfd:
	mfd_remove_devices(s2mpg12->dev);
err_irq_init:
	i2c_unregister_device(s2mpg12->i2c);
err_w_lock:
	mutex_destroy(&s2mpg12->i2c_lock);
err:
	kfree(s2mpg12);
	return ret;
}

static void s2mpg12_i2c_remove(struct i2c_client *i2c)
{
	struct s2mpg12_dev *s2mpg12 = i2c_get_clientdata(i2c);

	mfd_remove_devices(s2mpg12->dev);
	i2c_unregister_device(s2mpg12->i2c);
	kfree(s2mpg12);
}

static const struct i2c_device_id s2mpg12_i2c_id[] = {
	{ S2MPG12_MFD_DEV_NAME, TYPE_S2MPG12 }, {} };

MODULE_DEVICE_TABLE(i2c, s2mpg12_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s2mpg12_i2c_dt_ids[] = {
	{ .compatible = "samsung,s2mpg12mfd" },
	{},
};
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_PM)
static int s2mpg12_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct s2mpg12_dev *s2mpg12 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(s2mpg12->irq);

	disable_irq(s2mpg12->irq);

	return 0;
}

static int s2mpg12_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct s2mpg12_dev *s2mpg12 = i2c_get_clientdata(i2c);

#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
	pr_info("%s:%s\n", S2MPG12_MFD_DEV_NAME, __func__); // it will be removed.
#endif /* CONFIG_SAMSUNG_PRODUCT_SHIP */

	if (device_may_wakeup(dev))
		disable_irq_wake(s2mpg12->irq);

	enable_irq(s2mpg12->irq);

	return 0;
}
#else
#define s2mpg12_suspend NULL
#define s2mpg12_resume NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops s2mpg12_pm = {
	.suspend_late = s2mpg12_suspend,
	.resume_early = s2mpg12_resume,
};

static struct i2c_driver s2mpg12_i2c_driver = {
	.driver = {
		   .name = S2MPG12_MFD_DEV_NAME,
		   .owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &s2mpg12_pm,
#endif /* CONFIG_PM */
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = s2mpg12_i2c_dt_ids,
#endif /* CONFIG_OF */
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg12_i2c_probe,
	.remove = s2mpg12_i2c_remove,
	.id_table = s2mpg12_i2c_id,
};

static int __init s2mpg12_i2c_init(void)
{
	return i2c_add_driver(&s2mpg12_i2c_driver);
}

/* init early so consumer devices can complete system boot */
subsys_initcall(s2mpg12_i2c_init);

static void __exit s2mpg12_i2c_exit(void)
{
	i2c_del_driver(&s2mpg12_i2c_driver);
}

module_exit(s2mpg12_i2c_exit);

MODULE_DESCRIPTION("s2mpg12 multi-function core driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
