// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/regulator/pmic_class.c
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/regulator/pmic_class.h>
#include <linux/module.h>

/* CAUTION : Do not be declared as external pmic_class  */
static struct class *pmic_class;
static atomic_t pmic_dev;

struct device *pmic_device_create(void *drvdata, const char *fmt)
{
	struct device *dev;

	if (WARN_ON(!pmic_class))
		return NULL;

	if (WARN_ON(IS_ERR(pmic_class)))
		return NULL;

	dev = device_create(pmic_class, NULL, atomic_inc_return(&pmic_dev),
			    drvdata, fmt);
	if (IS_ERR(dev)) {
		pr_err("Failed to create device %s %ld\n", fmt, PTR_ERR(dev));
		return NULL;
	}
	pr_debug("%s : %s : %d\n", __func__, fmt, dev->devt);

	return dev;
}
EXPORT_SYMBOL_GPL(pmic_device_create);

struct device *pmic_subdevice_create(struct device *parent, const struct attribute_group **groups,
				     void *drvdata, const char *fmt)
{
	struct device *dev;

	dev = device_create_with_groups(pmic_class, parent, atomic_inc_return(&pmic_dev),
					drvdata, groups, fmt);
	if (IS_ERR(dev)) {
		dev_err(dev, "Failed to create device %s %ld\n", fmt, PTR_ERR(dev));
		return NULL;
	}
	pr_debug("%s : %s : %d\n", __func__, fmt, dev->devt);

	return dev;
}
EXPORT_SYMBOL_GPL(pmic_subdevice_create);


void pmic_device_destroy(dev_t devt)
{
	device_destroy(pmic_class, devt);
}
EXPORT_SYMBOL_GPL(pmic_device_destroy);

static int __init pmic_class_create(void)
{
	pmic_class = class_create(THIS_MODULE, "pmic");

	if (IS_ERR(pmic_class)) {
		pr_err("Failed to create class(pmic) %ld\n",
		       PTR_ERR(pmic_class));
		return PTR_ERR(pmic_class);
	}

	return 0;
}
arch_initcall(pmic_class_create);

MODULE_DESCRIPTION("pmic sysfs");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");

