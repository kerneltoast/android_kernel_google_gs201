// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg13.c
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * mfd core driver for the s2mpg13
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/mfd/samsung/s2mpg13-register.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <soc/google/acpm_mfd.h>

#if IS_ENABLED(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#define I2C_ADDR_TOP 0x00
#define I2C_ADDR_PMIC 0x01
#define I2C_ADDR_METER 0x0A
#define I2C_ADDR_WLWP 0x0B
#define I2C_ADDR_GPIO 0x0C
#define I2C_ADDR_MT_TRIM 0x0E
#define I2C_ADDR_TRIM 0x0F

static struct device_node *acpm_mfd_node;

static struct mfd_cell s2mpg13_devs[] = {
	{
		.name = "s2mpg13-regulator",
	},
	{
		.name = "s2mpg13-meter",
	},
	{
		.name = "s2mpg13_gpio",
	},
	{
		.name = "s2mpg13-spmic-thermal",
		.of_compatible = "google,s2mpg13-spmic-thermal",
	},
};

int s2mpg13_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpg13_dev *s2mpg13 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg13->i2c_lock);
	ret = exynos_acpm_read_reg(acpm_mfd_node, channel,
				   i2c->addr, reg, dest);
	mutex_unlock(&s2mpg13->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg13_read_reg);

int s2mpg13_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg13_dev *s2mpg13 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg13->i2c_lock);
	ret = exynos_acpm_bulk_read(acpm_mfd_node, channel,
				    i2c->addr, reg, count, buf);
	mutex_unlock(&s2mpg13->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg13_bulk_read);

int s2mpg13_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpg13_dev *s2mpg13 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg13->i2c_lock);
	ret = exynos_acpm_write_reg(acpm_mfd_node, channel,
				    i2c->addr, reg, value);
	mutex_unlock(&s2mpg13->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg13_write_reg);

int s2mpg13_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpg13_dev *s2mpg13 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg13->i2c_lock);
	ret = exynos_acpm_bulk_write(acpm_mfd_node, channel,
				     i2c->addr, reg, count, buf);
	mutex_unlock(&s2mpg13->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg13_bulk_write);

int s2mpg13_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpg13_dev *s2mpg13 = i2c_get_clientdata(i2c);
	u8 channel = 1;
	int ret;

	mutex_lock(&s2mpg13->i2c_lock);
	ret = exynos_acpm_update_reg(acpm_mfd_node, channel,
				     i2c->addr, reg, val, mask);
	mutex_unlock(&s2mpg13->i2c_lock);
	if (ret)
		pr_err("[%s] acpm ipc fail!\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg13_update_reg);

struct i2c_client *s2mpg13_get_i2c_client(struct s2mpg13_dev *dev,
					  unsigned int reg)
{
	struct i2c_client *client = NULL;

	if (reg >> 8 == I2C_ADDR_TOP)
		client = dev->i2c;
	else if (reg >> 8 == I2C_ADDR_PMIC)
		client = dev->pmic;
	else if (reg >> 8 == I2C_ADDR_METER)
		client = dev->meter;
	else if (reg >> 8 == I2C_ADDR_GPIO)
		client = dev->gpio;

	return client;
}

int s2mpg13_regmap_read_reg(void *context, unsigned int reg,
			    unsigned int *dest)
{
	u8 ureg = reg;
	u8 *udest = (u8 *)dest;
	struct s2mpg13_dev *dev = context;
	struct i2c_client *client = s2mpg13_get_i2c_client(dev, reg);

	if (!client)
		return -EFAULT;

	*dest = 0;
	return s2mpg13_read_reg(client, ureg, udest);
}

int s2mpg13_regmap_write_reg(void *context, unsigned int reg,
			     unsigned int value)
{
	u8 ureg = reg;
	u8 uvalue = value;
	struct s2mpg13_dev *dev = context;
	struct i2c_client *client = s2mpg13_get_i2c_client(dev, reg);

	if (!client)
		return -EFAULT;

	return s2mpg13_write_reg(client, ureg, uvalue);
}

static const struct regmap_range s2mpg13_valid_regs[] = {
	regmap_reg_range(0x000, 0x003), /* Common Block - VGPIO */
	regmap_reg_range(0x004, 0x029), /* Common Block */
	regmap_reg_range(0x100, 0x1D7), /* Power Management Block */
	regmap_reg_range(0xA00, 0xA62), /* Meter config, NTC */
	regmap_reg_range(0xA63, 0xAE5), /* Meter data */
	regmap_reg_range(0xC05, 0xC14), /* GPIO */
};

static const struct regmap_range s2mpg13_read_only_regs[] = {
	regmap_reg_range(0x000, 0x00B), /* Common Block */
	regmap_reg_range(0x020, 0x023), /* Common Block */
	regmap_reg_range(0x027, 0x029), /* Common Block */
	regmap_reg_range(0x100, 0x103), /* INT1~4 */
	regmap_reg_range(0x10A, 0x10A), /* OFFSRC */
	regmap_reg_range(0xA63, 0xAE5), /* Meter data */
};

const struct regmap_access_table s2mpg13_read_register_set = {
	.yes_ranges = s2mpg13_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(s2mpg13_valid_regs),
};

const struct regmap_access_table s2mpg13_write_register_set = {
	.yes_ranges = s2mpg13_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(s2mpg13_valid_regs),
	.no_ranges = s2mpg13_read_only_regs,
	.n_no_ranges = ARRAY_SIZE(s2mpg13_read_only_regs),
};

static struct regmap_config s2mpg13_regmap_config = {
	.name = "s2mpg13",
	.reg_bits = 12,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0xC14,
	.reg_read = s2mpg13_regmap_read_reg,
	.reg_write = s2mpg13_regmap_write_reg,
	.rd_table = &s2mpg13_read_register_set,
	.wr_table = &s2mpg13_write_register_set,
};

#if IS_ENABLED(CONFIG_OF)
static int of_s2mpg13_dt(struct device *dev,
			 struct s2mpg13_platform_data *pdata,
			 struct s2mpg13_dev *s2mpg13)
{
	struct device_node *np = dev->of_node;
	const char *status;

	if (!np)
		return -EINVAL;

	acpm_mfd_node = np;

	status = of_get_property(np, "s2mpg13,wakeup", NULL);
	if (!status)
		return -EINVAL;
	pdata->wakeup = !strcmp(status, "enabled") || !strcmp(status, "okay");

	return 0;
}
#else
static int of_s2mpg13_dt(struct device *dev,
			 struct s2mpg13_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static void s2mpg13_get_rev_id(struct s2mpg13_dev *s2mpg13, int id)
{
	if (id == 0x0 || id == 0x1 || id == 0x2)
		s2mpg13->pmic_rev = S2MPG13_EVT0;
	else
		s2mpg13->pmic_rev = S2MPG13_EVT1;
}

static int s2mpg13_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *dev_id)
{
	struct s2mpg13_dev *s2mpg13;
	struct s2mpg13_platform_data *pdata = i2c->dev.platform_data;
	u8 reg_data;
	int ret = 0;

	dev_info(&i2c->dev, "%s i2c probe\n", S2MPG13_MFD_DEV_NAME);

	s2mpg13 = kzalloc(sizeof(*s2mpg13), GFP_KERNEL);
	if (!s2mpg13)
		return -ENOMEM;

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
				     sizeof(struct s2mpg13_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err;
		}

		ret = of_s2mpg13_dt(&i2c->dev, pdata, s2mpg13);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to get device of_node\n");
			goto err;
		}

		i2c->dev.platform_data = pdata;
	} else {
		pdata = i2c->dev.platform_data;
	}

	s2mpg13->dev = &i2c->dev;
	s2mpg13->dev->init_name = "i2c-" S2MPG13_MFD_DEV_NAME;
	i2c->addr = I2C_ADDR_TOP;
	s2mpg13->i2c = i2c;
	s2mpg13->device_type = S2MPG13X;

	if (pdata) {
		s2mpg13->pdata = pdata;

		pdata->irq_base = irq_alloc_descs(-1, 0, S2MPG13_IRQ_NR, -1);
		if (pdata->irq_base < 0) {
			pr_err("%s:%s devm_irq_alloc_descs Fail! ret(%d)\n",
			       S2MPG13_MFD_DEV_NAME, __func__, pdata->irq_base);
			ret = pdata->irq_base;
			goto err;
		}

		s2mpg13->irq_base = pdata->irq_base;
		s2mpg13->wakeup = pdata->wakeup;
	} else {
		ret = -EINVAL;
		goto err;
	}
	mutex_init(&s2mpg13->i2c_lock);

	i2c_set_clientdata(i2c, s2mpg13);

	if (s2mpg13_read_reg(i2c, S2MPG13_COMMON_CHIPID, &reg_data) < 0) {
		dev_warn(s2mpg13->dev,
			 "device not found on this channel\n");
		ret = -ENODEV;
		goto err_w_lock;
	}
	s2mpg13_get_rev_id(s2mpg13, reg_data & 0x7);

	s2mpg13->pmic = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_PMIC);
	s2mpg13->meter = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_METER);
	s2mpg13->wlwp = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_WLWP);
	s2mpg13->gpio = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_GPIO);
	s2mpg13->mt_trim = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_MT_TRIM);
	s2mpg13->trim = i2c_new_dummy_device(i2c->adapter, I2C_ADDR_TRIM);

	i2c_set_clientdata(s2mpg13->pmic, s2mpg13);
	i2c_set_clientdata(s2mpg13->meter, s2mpg13);
	i2c_set_clientdata(s2mpg13->wlwp, s2mpg13);
	i2c_set_clientdata(s2mpg13->gpio, s2mpg13);
	i2c_set_clientdata(s2mpg13->mt_trim, s2mpg13);
	i2c_set_clientdata(s2mpg13->trim, s2mpg13);

	dev_info(s2mpg13->dev, "device found: rev.0x%02x\n", s2mpg13->pmic_rev);

	s2mpg13->regmap = devm_regmap_init(s2mpg13->dev, NULL, s2mpg13,
					   &s2mpg13_regmap_config);
	if (IS_ERR(s2mpg13->regmap)) {
		dev_err(s2mpg13->dev, "regmap_init failed!\n");
		ret = PTR_ERR(s2mpg13->regmap);
		goto err_w_lock;
	}

	ret = s2mpg13_notifier_init(s2mpg13);
	if (ret) {
		dev_err(s2mpg13->dev, "s2mpg13_notifier_init fail\n");
		goto err_w_lock;
	}

	ret = mfd_add_devices(s2mpg13->dev, -1, s2mpg13_devs,
			      ARRAY_SIZE(s2mpg13_devs), NULL, 0, NULL);
	if (ret)
		goto err_mfd;

	ret = device_init_wakeup(s2mpg13->dev, pdata->wakeup);
	if (ret) {
		dev_err(s2mpg13->dev, "device_init_wakeup fail(%d)\n", ret);
		goto err_mfd;
	}

	return ret;

err_mfd:
	mfd_remove_devices(s2mpg13->dev);
err_w_lock:
	mutex_destroy(&s2mpg13->i2c_lock);
err:
	kfree(s2mpg13);
	return ret;
}

static void s2mpg13_i2c_remove(struct i2c_client *i2c)
{
	struct s2mpg13_dev *s2mpg13 = i2c_get_clientdata(i2c);

	if (s2mpg13->pdata->wakeup)
		device_init_wakeup(s2mpg13->dev, false);
	mfd_remove_devices(s2mpg13->dev);
	i2c_unregister_device(s2mpg13->i2c);
	kfree(s2mpg13);
}

static const struct i2c_device_id s2mpg13_i2c_id[] = {
	{ S2MPG13_MFD_DEV_NAME, TYPE_S2MPG13 }, {} };

MODULE_DEVICE_TABLE(i2c, s2mpg13_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s2mpg13_i2c_dt_ids[] = {
	{ .compatible = "samsung,s2mpg13mfd" },
	{},
};
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_PM)
static int s2mpg13_suspend(struct device *dev)
{
	return 0;
}

static int s2mpg13_resume(struct device *dev)
{
	return 0;
}
#else
#define s2mpg13_suspend NULL
#define s2mpg13_resume NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops s2mpg13_pm = {
	.suspend_late = s2mpg13_suspend,
	.resume_early = s2mpg13_resume,
};

static struct i2c_driver s2mpg13_i2c_driver = {
	.driver = {
		   .name = S2MPG13_MFD_DEV_NAME,
		   .owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &s2mpg13_pm,
#endif /* CONFIG_PM */
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = s2mpg13_i2c_dt_ids,
#endif /* CONFIG_OF */
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg13_i2c_probe,
	.remove = s2mpg13_i2c_remove,
	.id_table = s2mpg13_i2c_id,
};

static int __init s2mpg13_i2c_init(void)
{
	return i2c_add_driver(&s2mpg13_i2c_driver);
}

/* init early so consumer devices can complete system boot */
subsys_initcall(s2mpg13_i2c_init);

static void __exit s2mpg13_i2c_exit(void)
{
	i2c_del_driver(&s2mpg13_i2c_driver);
}

module_exit(s2mpg13_i2c_exit);

MODULE_DESCRIPTION("s2mpg13 multi-function core driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
