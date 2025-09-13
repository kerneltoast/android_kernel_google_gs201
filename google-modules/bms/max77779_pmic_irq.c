// SPDX-License-Identifier: GPL-2.0-only
/*
 * max77779 pmic irq driver
 *
 * Copyright 2023 Google, LLC
 */

#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include "max77779_pmic.h"

#define MAX77779_NUM_IRQS	8   /* number of irqs to export */

struct max77779_pmic_irq_info {
	struct device		*dev;
	struct device		*core;
	struct irq_domain	*domain;
	struct mutex		lock;
	int			irq;
	unsigned int		mask;
	unsigned int		mask_u;  /* pending updates */
	unsigned int		trig_type;

	unsigned int		wake_u;
	unsigned int		wake;
};

static irqreturn_t max77779_pmic_irq_handler(int irq, void *ptr)
{
	struct max77779_pmic_irq_info *info = ptr;
	struct device *core = info->core;
	uint8_t intsrc_sts;
	int sub_irq;
	int offset;
	int err;

	pm_stay_awake(info->dev);
	err = max77779_external_pmic_reg_read(core, MAX77779_PMIC_INTSRC_STS, &intsrc_sts);
	if (err) {
		dev_err_ratelimited(info->dev, "read error %d\n", err);
		pm_relax(info->dev);
		return IRQ_NONE;
	}

	for (offset = 0; offset < MAX77779_NUM_IRQS; offset++) {
		if (intsrc_sts & (1 << offset)) {
			sub_irq = irq_find_mapping(info->domain, offset);
			if (sub_irq)
				handle_nested_irq(sub_irq);
		}
	}

	err = max77779_external_pmic_reg_write(core, MAX77779_PMIC_INTSRC_STS, intsrc_sts);
	if (err)
		dev_err_ratelimited(info->dev, "write error %d\n", err);

	pm_relax(info->dev);
	return IRQ_HANDLED;
}

static void max77779_pmic_irq_mask(struct irq_data *d)
{
	struct max77779_pmic_irq_info *info = irq_data_get_irq_chip_data(d);

	info->mask |= BIT(d->hwirq);
	info->mask_u |= BIT(d->hwirq);
}

static void max77779_pmic_irq_unmask(struct irq_data *d)
{
	struct max77779_pmic_irq_info *info = irq_data_get_irq_chip_data(d);

	info->mask &= ~BIT(d->hwirq);
	info->mask_u |= BIT(d->hwirq);
}

static void max77779_pmic_irq_disable(struct irq_data *d)
{
	max77779_pmic_irq_mask(d);
}

static void max77779_pmic_irq_enable(struct irq_data *d)
{
	max77779_pmic_irq_unmask(d);
}

static int max77779_pmic_set_irq_type(struct irq_data *d, unsigned int type)
{
	struct max77779_pmic_irq_info *info = irq_data_get_irq_chip_data(d);

	switch (type) {
	case IRQF_TRIGGER_NONE:
	case IRQF_TRIGGER_RISING:
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_LOW:
		info->trig_type &= (0xf << (d->hwirq * 4));
		info->trig_type |= (type << (d->hwirq * 4));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max77779_pmic_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct max77779_pmic_irq_info *info = irq_data_get_irq_chip_data(d);

	info->wake_u |= BIT(d->hwirq);
	info->wake &= ~BIT(d->hwirq);
	info->wake |= on << d->hwirq;

	return 0;
}

static void max77779_pmic_bus_lock(struct irq_data *d)
{
	struct max77779_pmic_irq_info *info = irq_data_get_irq_chip_data(d);

	mutex_lock(&info->lock);
}

static void max77779_pmic_bus_sync_unlock(struct irq_data *d)
{
	struct max77779_pmic_irq_info *info = irq_data_get_irq_chip_data(d);
	struct device *core = info->core;
	uint8_t offset, value, intb_mask;
	unsigned int id;
	int err;

	if (!info->mask_u)
		goto unlock_out;

	err = max77779_external_pmic_reg_read(core, MAX77779_PMIC_INTB_MASK, &intb_mask);
	if (err < 0) {
		dev_err(info->dev, "Unable to read interrupt mask (%d)\n", err);
		goto unlock_out;
	}

	while (info->mask_u) {
		offset = __ffs(info->mask_u);
		value = !!(info->mask & (1 << offset));

		intb_mask &= ~(1 << offset);
		intb_mask |= value << offset;

		/* clear pending updates */
		info->mask_u &= ~(1 << offset);
	}

	err = max77779_external_pmic_reg_write(core, MAX77779_PMIC_INTB_MASK, intb_mask);
	if (err < 0) {
		dev_err(info->dev, "Unable to write interrupt mask (%d)\n", err);
		goto unlock_out;
	}

	while (info->wake_u) {
		id = __ffs(info->wake_u);
		irq_set_irq_wake(info->irq, !!(info->wake & BIT(id)));
		info->wake_u &= ~BIT(id);
	}

 unlock_out:
	mutex_unlock(&info->lock);
}

static struct irq_chip max77779_pmic_irq_chip = {
	.name = "max77779_pmic_irq",
	.irq_enable = max77779_pmic_irq_enable,
	.irq_disable = max77779_pmic_irq_disable,
	.irq_mask = max77779_pmic_irq_mask,
	.irq_unmask = max77779_pmic_irq_unmask,
	.irq_set_type = max77779_pmic_set_irq_type,
	.irq_set_wake = max77779_pmic_irq_set_wake,
	.irq_bus_lock = max77779_pmic_bus_lock,
	.irq_bus_sync_unlock = max77779_pmic_bus_sync_unlock,
};

static int max77779_pmic_irq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77779_pmic_irq_info *info;
	int irq_gpio;
	int i;
	int err;

	if (!dev->of_node)
		return -ENODEV;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = dev;
	info->dev->init_name = "max77779-pmic-irq";
	info->core = dev->parent;
	mutex_init(&info->lock);

	/* this is our input gpio from sequoia */
	irq_gpio = of_get_named_gpio(dev->of_node, "max777x9,irq-gpio", 0);
	if (irq_gpio < 0) {
		dev_err(dev, "irq_gpio is not defined\n");
		return -ENODEV;
	}

	err = devm_gpio_request_one(dev, irq_gpio, 0, "max77779-irq");
	if (err) {
		dev_err(dev, "Unable to request max77779-irq err =  %d\n", err);
		return err;
	}

	err = gpio_direction_input(irq_gpio);
	if (err < 0) {
		dev_err(dev, "Error setting irq_gpio to input\n");
		return err;
	}

	info->irq = gpio_to_irq(irq_gpio);
	if (info->irq < 0) {
		dev_err(dev, "Error getting irq (%d)\n", info->irq);
		return info->irq;
	}

	device_init_wakeup(dev, true);

	/* mask and clear all interrupts */
	err =  max77779_external_pmic_reg_write(info->core, MAX77779_PMIC_INTB_MASK, 0xff);
	if (err) {
		dev_err(dev, "Unable to clear mask. err = %d\n", err);
		return err;
	}

	err = max77779_external_pmic_reg_write(info->core, MAX77779_PMIC_INTSRC_STS, 0xff);
	if (err) {
		dev_err(dev, "Unable to clear ints. err = %d\n", err);
		return err;
	}

	info->trig_type = 0x00000000;
	info->domain = irq_domain_add_linear(dev->of_node, MAX77779_NUM_IRQS,
			&irq_domain_simple_ops, info);
	if (!info->domain) {
		dev_err(info->dev, "Unable to get irq domain\n");
		return -ENODEV;
	}

	for (i = 0; i < MAX77779_NUM_IRQS; i++) {
		int irq = irq_create_mapping(info->domain, i);

		if (!irq) {
			dev_err(dev, "failed irq create map\n");
			return -EINVAL;
		}
		irq_set_chip_data(irq, info);
		irq_set_chip_and_handler(irq, &max77779_pmic_irq_chip,
				handle_simple_irq);
	}

	err = devm_request_threaded_irq(info->dev, info->irq, NULL,
			max77779_pmic_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"max77779_pmic_irq", info);
	if (err < 0) {
		dev_err(dev, "failed get irq thread err = %d\n", err);
		return -ENODEV;
	}

	return 0;
}

static int max77779_pmic_irq_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, false);
	return 0;
}

static const struct platform_device_id max77779_pmic_irq_id[] = {
	{ "max77779-pmic-irq", 0},
	{},
};

MODULE_DEVICE_TABLE(platform, max77779_pmic_irq_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id max77779_pmic_irq_match_table[] = {
	{ .compatible = "max77779-pmic-irq",},
	{ },
};
#endif

static struct platform_driver max77779_pmic_irq_driver = {
	.probe = max77779_pmic_irq_probe,
	.remove = max77779_pmic_irq_remove,
	.id_table = max77779_pmic_irq_id,
	.driver = {
		.name = "max77779-pmic-irq",
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = max77779_pmic_irq_match_table,
#endif
	},
};

module_platform_driver(max77779_pmic_irq_driver);

MODULE_DESCRIPTION("Maxim 77779 SGPIO driver");
MODULE_AUTHOR("James Wylder <jwylder@google.com>");
MODULE_LICENSE("GPL");
