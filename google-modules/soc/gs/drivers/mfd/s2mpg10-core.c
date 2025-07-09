// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/mdf/s2mpg10-core.c
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * mfd core driver for the s2mpg10
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
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
#define I2C_ADDR_TRIM 0x0F

static struct device_node *acpm_mfd_node;

static struct mfd_cell s2mpg10_devs[] = {
	{
		.name = "s2mpg10-regulator",
	},
	{
		.name = "s2mpg10-rtc",
	},
	{
		.name = "s2mpg10-meter",
	},
	{
		.name = "s2mpg10_gpio",
	},
};

static u8 s2mpg10_pmic_rev;

int s2mpg10_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpg10_dev *s2mpg10 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg10->i2c_lock);
	ret = exynos_acpm_read_reg(acpm_mfd_node, channel, i2c->addr,
				   reg, dest);
	mutex_unlock(&s2mpg10->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg10_read_reg);

int s2mpg10_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg10_dev *s2mpg10 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg10->i2c_lock);
	ret = exynos_acpm_bulk_read(acpm_mfd_node, channel, i2c->addr,
				    reg, count, buf);
	mutex_unlock(&s2mpg10->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg10_bulk_read);

int s2mpg10_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpg10_dev *s2mpg10 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg10->i2c_lock);
	ret = exynos_acpm_write_reg(acpm_mfd_node, channel,
				    i2c->addr, reg, value);
	mutex_unlock(&s2mpg10->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg10_write_reg);

int s2mpg10_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg10_dev *s2mpg10 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg10->i2c_lock);
	ret = exynos_acpm_bulk_write(acpm_mfd_node, channel,
				     i2c->addr, reg, count, buf);
	mutex_unlock(&s2mpg10->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg10_bulk_write);

int s2mpg10_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpg10_dev *s2mpg10 = i2c_get_clientdata(i2c);
	u8 channel = 0;
	int ret;

	mutex_lock(&s2mpg10->i2c_lock);
	ret = exynos_acpm_update_reg(acpm_mfd_node, channel,
				     i2c->addr, reg, val, mask);
	mutex_unlock(&s2mpg10->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg10_update_reg);

u8 s2mpg10_get_rev_id(void)
{
	return s2mpg10_pmic_rev;
}
EXPORT_SYMBOL_GPL(s2mpg10_get_rev_id);

struct i2c_client *s2mpg10_get_i2c_client(struct s2mpg10_dev *dev,
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

	return client;
}

int s2mpg10_regmap_read_reg(void *context, unsigned int reg,
			    unsigned int *dest)
{
	u8 ureg = reg;
	u8 *udest = (u8 *)dest;
	struct s2mpg10_dev *dev = context;
	struct i2c_client *client = s2mpg10_get_i2c_client(dev, reg);

	if (!client)
		return -EFAULT;

	*dest = 0;
	return s2mpg10_read_reg(client, ureg, udest);
}

int s2mpg10_regmap_write_reg(void *context, unsigned int reg,
			     unsigned int value)
{
	u8 ureg = reg;
	u8 uvalue = value;
	struct s2mpg10_dev *dev = context;
	struct i2c_client *client = s2mpg10_get_i2c_client(dev, reg);

	if (!client)
		return -EFAULT;

	return s2mpg10_write_reg(client, ureg, uvalue);
}

static const struct regmap_range s2mpg10_valid_regs[] = {
	regmap_reg_range(0x000, 0x000), /* Common Block - CHIP_ID */
	regmap_reg_range(0x00A, 0x00C), /* Common Block - SPD_CTRL */
	regmap_reg_range(0x01A, 0x02A), /* Common Block */
	regmap_reg_range(0x100, 0x1F6), /* Power Management Block */
	regmap_reg_range(0x200, 0x22B), /* RTC (real-time clock) */
	regmap_reg_range(0xA00, 0xA21), /* Meter config */
	regmap_reg_range(0xA40, 0xA8A), /* Meter data */
};

static const struct regmap_range s2mpg10_read_only_regs[] = {
	regmap_reg_range(0x000, 0x000), /* Common Block - CHIP_ID_M */
	regmap_reg_range(0x00A, 0x00C), /* Common Block - SPD_CTRL */
	regmap_reg_range(0x100, 0x105), /* INT1~6 */
	regmap_reg_range(0x10C, 0x10D), /* STATUS */
	regmap_reg_range(0xA40, 0xA8A), /* Meter data */
};

const struct regmap_access_table s2mpg10_read_register_set = {
	.yes_ranges = s2mpg10_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_valid_regs),
};

const struct regmap_access_table s2mpg10_write_register_set = {
	.yes_ranges = s2mpg10_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_valid_regs),
	.no_ranges = s2mpg10_read_only_regs,
	.n_no_ranges = ARRAY_SIZE(s2mpg10_read_only_regs),
};

static struct regmap_config s2mpg10_regmap_config = {
	.name = "s2mpg10",
	.reg_bits = 12,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0xA8A,
	.reg_read = s2mpg10_regmap_read_reg,
	.reg_write = s2mpg10_regmap_write_reg,
	.rd_table = &s2mpg10_read_register_set,
	.wr_table = &s2mpg10_write_register_set,
};

#if IS_ENABLED(CONFIG_OF)
static int of_s2mpg10_dt(struct device *dev,
			 struct s2mpg10_platform_data *pdata,
			 struct s2mpg10_dev *s2mpg10)
{
	struct device_node *np = dev->of_node;
	int ret, strlen;
	const char *status;
	u32 val;

	if (!np)
		return -EINVAL;

	acpm_mfd_node = np;

	pdata->irq_gpio = of_get_named_gpio(np, "s2mpg10,irq-gpio", 0);

	status = of_get_property(np, "s2mpg10,wakeup", &strlen);
	if (!status)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wakeup = true;
		else
			pdata->wakeup = false;
	}

	pr_info("%s: irq-gpio: %u\n", __func__, pdata->irq_gpio);

	/* WTSR, SMPL */
	pdata->wtsr_smpl =
		devm_kzalloc(dev, sizeof(*pdata->wtsr_smpl), GFP_KERNEL);
	if (!pdata->wtsr_smpl)
		return -ENOMEM;

	status = of_get_property(np, "wtsr_en", &strlen);
	if (!status)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wtsr_smpl->wtsr_en = true;
		else
			pdata->wtsr_smpl->wtsr_en = false;
	}

	status = of_get_property(np, "smpl_en", &strlen);
	if (!status)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wtsr_smpl->smpl_en = true;
		else
			pdata->wtsr_smpl->smpl_en = false;
	}

	ret = of_property_read_u32(np, "wtsr_timer_val",
				   &pdata->wtsr_smpl->wtsr_timer_val);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "smpl_timer_val",
				   &pdata->wtsr_smpl->smpl_timer_val);
	if (ret)
		return -EINVAL;

	status = of_get_property(np, "coldrst_en", &strlen);
	if (!status)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wtsr_smpl->coldrst_en = true;
		else
			pdata->wtsr_smpl->coldrst_en = false;
	}

	ret = of_property_read_u32(np, "coldrst_timer_val",
				   &pdata->wtsr_smpl->coldrst_timer_val);
	if (ret)
		return -EINVAL;

	status = of_get_property(np, "sub_smpl_en", &strlen);
	if (!status)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wtsr_smpl->sub_smpl_en = true;
		else
			pdata->wtsr_smpl->sub_smpl_en = false;
	}

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
static int of_s2mpg10_dt(struct device *dev,
			 struct s2mpg10_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static void s2mpg10_set_rev_id(struct s2mpg10_dev *s2mpg10, int id)
{
	if (id == 0x0 || id == 0x1)
		s2mpg10->pmic_rev = S2MPG10_EVT0;
	else
		s2mpg10->pmic_rev = S2MPG10_EVT1;
}

static int s2mpg10_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *dev_id)
{
	struct s2mpg10_dev *s2mpg10;
	struct s2mpg10_platform_data *pdata = i2c->dev.platform_data;

	u8 reg_data;
	int ret = 0;

	pr_info("%s:%s\n", S2MPG10_MFD_DEV_NAME, __func__);

	s2mpg10 = kzalloc(sizeof(*s2mpg10), GFP_KERNEL);
	if (!s2mpg10)
		return -ENOMEM;

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
				     sizeof(struct s2mpg10_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c->dev, "Failed to allocate memory\n");
			ret = -ENOMEM;
			goto err;
		}

		ret = of_s2mpg10_dt(&i2c->dev, pdata, s2mpg10);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to get device of_node\n");
			goto err;
		}

		i2c->dev.platform_data = pdata;
	} else {
		pdata = i2c->dev.platform_data;
	}

	s2mpg10->dev = &i2c->dev;
	s2mpg10->dev->init_name = "i2c-" S2MPG10_MFD_DEV_NAME;
	i2c->addr = I2C_ADDR_TOP;
	s2mpg10->i2c = i2c;
	s2mpg10->irq = i2c->irq;
	s2mpg10->device_type = S2MPG10X;

	if (pdata) {
		s2mpg10->pdata = pdata;

		pdata->irq_base = irq_alloc_descs(-1, 0, S2MPG10_IRQ_NR, -1);
		if (pdata->irq_base < 0) {
			pr_err("%s:%s irq_alloc_descs Fail! ret(%d)\n",
			       S2MPG10_MFD_DEV_NAME, __func__, pdata->irq_base);
			ret = -EINVAL;
			goto err;
		} else {
			s2mpg10->irq_base = pdata->irq_base;
		}

		s2mpg10->irq_gpio = pdata->irq_gpio;
		s2mpg10->wakeup = pdata->wakeup;
	} else {
		ret = -EINVAL;
		goto err;
	}
	mutex_init(&s2mpg10->i2c_lock);

	i2c_set_clientdata(i2c, s2mpg10);

	if (s2mpg10_read_reg(i2c, S2MPG10_COMMON_CHIPID, &reg_data) < 0) {
		dev_err(s2mpg10->dev,
			"device not found on this channel (this is not an error)\n");
		ret = -ENODEV;
		goto err_w_lock;
	} else {
		s2mpg10_set_rev_id(s2mpg10, reg_data & 0x7);
	}

	s2mpg10->pmic = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PMIC);
	s2mpg10->rtc = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_RTC);
	s2mpg10->meter = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_METER);
	s2mpg10->wlwp = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_WLWP);
	s2mpg10->trim = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_TRIM);
	s2mpg10_pmic_rev = s2mpg10->pmic_rev;

	i2c_set_clientdata(s2mpg10->pmic, s2mpg10);
	i2c_set_clientdata(s2mpg10->rtc, s2mpg10);
	i2c_set_clientdata(s2mpg10->meter, s2mpg10);
	i2c_set_clientdata(s2mpg10->wlwp, s2mpg10);
	i2c_set_clientdata(s2mpg10->trim, s2mpg10);

	pr_info("%s device found: rev.0x%2x\n", __func__, s2mpg10->pmic_rev);

	s2mpg10->regmap = devm_regmap_init(s2mpg10->dev, NULL, s2mpg10,
					   &s2mpg10_regmap_config);
	if (IS_ERR(s2mpg10->regmap)) {
		dev_err(s2mpg10->dev, "regmap_init failed!\n");
		return PTR_ERR(s2mpg10->regmap);
	}

	ret = s2mpg10_irq_init(s2mpg10);
	if (ret < 0)
		goto err_irq_init;

	ret = mfd_add_devices(s2mpg10->dev, -1, s2mpg10_devs,
			      ARRAY_SIZE(s2mpg10_devs), NULL, 0, NULL);
	if (ret < 0)
		goto err_mfd;

	device_init_wakeup(s2mpg10->dev, pdata->wakeup);

	return ret;

err_mfd:
	mfd_remove_devices(s2mpg10->dev);
err_irq_init:
	i2c_unregister_device(s2mpg10->i2c);
err_w_lock:
	mutex_destroy(&s2mpg10->i2c_lock);
err:
	kfree(s2mpg10);
	return ret;
}

static void s2mpg10_i2c_remove(struct i2c_client *i2c)
{
	struct s2mpg10_dev *s2mpg10 = i2c_get_clientdata(i2c);

	mfd_remove_devices(s2mpg10->dev);
	i2c_unregister_device(s2mpg10->i2c);
	kfree(s2mpg10);
}

static const struct i2c_device_id s2mpg10_i2c_id[] = {
	{ S2MPG10_MFD_DEV_NAME, TYPE_S2MPG10 }, {} };

MODULE_DEVICE_TABLE(i2c, s2mpg10_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s2mpg10_i2c_dt_ids[] = {
	{ .compatible = "samsung,s2mpg10mfd" },
	{},
};
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_PM)
static int s2mpg10_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct s2mpg10_dev *s2mpg10 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(s2mpg10->irq);

	disable_irq(s2mpg10->irq);

	return 0;
}

static int s2mpg10_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct s2mpg10_dev *s2mpg10 = i2c_get_clientdata(i2c);

#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
	pr_info("%s:%s\n", S2MPG10_MFD_DEV_NAME, __func__);
#endif /* CONFIG_SAMSUNG_PRODUCT_SHIP */

	enable_irq(s2mpg10->irq);

	if (device_may_wakeup(dev))
		disable_irq_wake(s2mpg10->irq);

	return 0;
}
#else
#define s2mpg10_suspend NULL
#define s2mpg10_resume NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops s2mpg10_pm = {
	.suspend = s2mpg10_suspend,
	.resume = s2mpg10_resume,
};

static struct i2c_driver s2mpg10_i2c_driver = {
	.driver = {
		   .name = S2MPG10_MFD_DEV_NAME,
		   .owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &s2mpg10_pm,
#endif /* CONFIG_PM */
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = s2mpg10_i2c_dt_ids,
#endif /* CONFIG_OF */
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg10_i2c_probe,
	.remove = s2mpg10_i2c_remove,
	.id_table = s2mpg10_i2c_id,
};

static int __init s2mpg10_i2c_init(void)
{
	pr_info("%s:%s\n", S2MPG10_MFD_DEV_NAME, __func__);
	return i2c_add_driver(&s2mpg10_i2c_driver);
}

/* init early so consumer devices can complete system boot */
subsys_initcall(s2mpg10_i2c_init);

static void __exit s2mpg10_i2c_exit(void)
{
	i2c_del_driver(&s2mpg10_i2c_driver);
}

module_exit(s2mpg10_i2c_exit);

MODULE_DESCRIPTION("s2mpg10 multi-function core driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
