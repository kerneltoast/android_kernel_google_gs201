// SPDX-License-Identifier: GPL-2.0-only
/*
 * max77779 sgpio driver
 *
 * Copyright (C) 2023 Google, LLC.
 */

#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include "max77779_pmic.h"

#define MAX77779_SGPIO_CNFGx_MODE_INPUT		0b01
#define MAX77779_SGPIO_CNFGx_MODE_OUTPUT	0b10

#define MAX77779_SGPIO_CNFG_DBNC_DISABLE	0x0
#define MAX77779_SGPIO_CNFG_DBNC_7MS		0x1
#define MAX77779_SGPIO_CNFG_DBNC_15MS		0x2
#define MAX77779_SGPIO_CNFG_DBNC_31MS		0x3

#define MAX77779_SGPIO_CNFG_IRQ_DISABLE		0b00
#define MAX77779_SGPIO_CNFG_IRQ_FALLING		0b01
#define MAX77779_SGPIO_CNFG_IRQ_RISING		0b10
#define MAX77779_SGPIO_CNFG_IRQ_BOTH		0b11

#define MAX77779_SGPIO_NUM_GPIOS		8
struct max77779_pmic_sgpio_info {
	struct device		*dev;
	struct device		*core;
	struct gpio_chip	gpio_chip;
	struct mutex		lock;

	int			irq;

	unsigned int		mask;
	unsigned int		mask_u;  /* mask update pending */

	unsigned int		trig_type_u;  /* trig_type update pending */
	unsigned int		trig_type[MAX77779_SGPIO_NUM_GPIOS];

	unsigned int		wake_u;  /* wake update pending */
	unsigned int		wake;
};

static int max77779_pmic_sgpio_get_direction(struct gpio_chip *gc,
		unsigned int offset)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	const uint8_t reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + offset;
	uint8_t mode;
	int err;

	if (offset >= gc->ngpio)
		return -EINVAL;

	err = max77779_external_pmic_reg_read(core, reg, &mode);
	if (err) {
		dev_err(info->dev, "Unable to read SGPIO config (%d)\n", err);
		return err;
	}
	mode &= MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_MASK;
	mode >>= MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_SHIFT;

	switch (mode) {
	case MAX77779_SGPIO_CNFGx_MODE_INPUT:
		return GPIO_LINE_DIRECTION_IN;
	case MAX77779_SGPIO_CNFGx_MODE_OUTPUT:
		return GPIO_LINE_DIRECTION_OUT;
	case 0b00:
	case 0b11:
	default:
		return -ENODEV;
	}
}

static int max77779_pmic_sgpio_direction_input(struct gpio_chip *gc,
		unsigned int offset)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	const uint8_t reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + offset;
	const uint8_t mask = MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_MASK;
	uint8_t val;

	if (offset >= gc->ngpio)
		return -EINVAL;

	val = MAX77779_SGPIO_CNFGx_MODE_INPUT << MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_SHIFT;
	return max77779_external_pmic_reg_update(core, reg, mask, val);
}

static int max77779_pmic_sgpio_direction_output(struct gpio_chip *gc,
		unsigned int offset, int value)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	const uint8_t reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + offset;
	const uint8_t mask = MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_MASK |
			     MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_MASK;
	uint8_t val;

	if (offset >= gc->ngpio)
		return -EINVAL;

	val = (!!value) << MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_SHIFT;
	val |= MAX77779_SGPIO_CNFGx_MODE_OUTPUT << MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_SHIFT;

	return max77779_external_pmic_reg_update(core, reg, mask, val);
}

static int max77779_pmic_sgpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	const uint8_t reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + offset;
	uint8_t val;
	int err;

	if (offset >= gc->ngpio)
		return -EINVAL;

	err = max77779_external_pmic_reg_read(core, reg, &val);
	if (err) {
		dev_err(info->dev, "Unable to read SGPIO config (%d)\n", err);
		return err;
	}
	val = !!(val & MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_MASK);
	return val;
}

static void max77779_pmic_sgpio_set(struct gpio_chip *gc,
		unsigned int offset, int value)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	const uint8_t reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + offset;
	const uint8_t mask = MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_MASK;
	uint8_t val;

	if (offset >= gc->ngpio)
		return;

	val = !!value << MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_SHIFT;
	max77779_external_pmic_reg_update(core, reg, mask, val);
}

static void max77779_pmic_sgpio_set_irq_valid_mask(struct gpio_chip *gc,
	unsigned long *valid_mask, unsigned int ngpios)
{
	bitmap_clear(valid_mask, 0, ngpios);
	bitmap_set(valid_mask, 0, ngpios);
}

static int max77779_pmic_sgpio_irq_init_hw(struct gpio_chip *gc)
{
	return 0;
}

static void max77779_pmic_sgpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	info->mask |= BIT(d->hwirq);
	info->mask_u |= BIT(d->hwirq);
}

static void max77779_pmic_sgpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	info->mask &= ~BIT(d->hwirq);
	info->mask_u |= BIT(d->hwirq);
}

static void max77779_pmic_sgpio_irq_disable(struct irq_data *d)
{
	max77779_pmic_sgpio_irq_mask(d);
}

static void max77779_pmic_sgpio_irq_enable(struct irq_data *d)
{
	max77779_pmic_sgpio_irq_unmask(d);
}

static int max77779_pmic_sgpio_set_irq_type(struct irq_data *d,
		unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	switch (type) {
	case IRQF_TRIGGER_NONE:
	case IRQF_TRIGGER_RISING:
	case IRQF_TRIGGER_FALLING:
	case (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING):
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_LOW:
		info->trig_type[d->hwirq] = type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max77779_pmic_sgpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	info->wake_u |= BIT(d->hwirq);
	info->wake &= ~BIT(d->hwirq);
	info->wake |= on << d->hwirq;

	return 0;
}

static void max77779_pmic_sgpio_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	mutex_lock(&info->lock);
}

static int max77779_pmic_sgpio_irqf2cnfg(unsigned int irqf)
{
	switch (irqf) {
	case IRQF_TRIGGER_NONE:
		return MAX77779_SGPIO_CNFG_IRQ_DISABLE;
	case IRQF_TRIGGER_RISING:
		return MAX77779_SGPIO_CNFG_IRQ_RISING;
	case IRQF_TRIGGER_FALLING:
		return MAX77779_SGPIO_CNFG_IRQ_FALLING;
	case (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING):
		return MAX77779_SGPIO_CNFG_IRQ_BOTH;
	case IRQF_TRIGGER_HIGH:
		return MAX77779_SGPIO_CNFG_IRQ_RISING;
	case IRQF_TRIGGER_LOW:
		return MAX77779_SGPIO_CNFG_IRQ_FALLING;
	default:
		return MAX77779_SGPIO_CNFG_IRQ_DISABLE;
	}
}

static void max77779_pmic_sgpio_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	unsigned int id;

	if (!(info->trig_type_u | info->mask_u | info->wake_u))
		goto unlock_out;

	while (info->mask_u) {
		id = __ffs(info->mask_u);

		/* signal trig_type to handle this id */
		info->trig_type_u |= BIT(id);
		info->mask_u &= ~BIT(id);
	}

	while (info->trig_type_u) {
		unsigned int masked;
		unsigned int reg;
		unsigned int cnfg_val;

		id = __ffs(info->trig_type_u);

		masked = BIT(id) & info->mask;
		if (masked)
			cnfg_val = MAX77779_SGPIO_CNFG_IRQ_DISABLE;
		else
			cnfg_val = max77779_pmic_sgpio_irqf2cnfg(info->trig_type[id]);
		cnfg_val <<= MAX77779_PMIC_GPIO_SGPIO_CNFG0_IRQ_SEL_SHIFT;

		reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + id;
		max77779_external_pmic_reg_update(core, reg,
						  MAX77779_PMIC_GPIO_SGPIO_CNFG0_IRQ_SEL_MASK,
						  cnfg_val);

		info->trig_type_u &= ~BIT(id);
	}

	while (info->wake_u) {
		id = __ffs(info->wake_u);
		irq_set_irq_wake(info->irq, !!(info->wake & BIT(id)));
		info->wake_u &= ~BIT(id);
	}


unlock_out:
	mutex_unlock(&info->lock);
}

static bool max77779_sgpio_handle_nested_irq(struct max77779_pmic_sgpio_info *info,
					     int offset)
{
	struct irq_domain *domain = info->gpio_chip.irq.domain;
	uint8_t sgpio_sts_reg = MAX77779_PMIC_GPIO_SGPIO_CNFG0 + offset;
	int sub_irq;
	struct device *core = info->core;
	uint8_t sgpio_sts, sgpio_val;
	unsigned int trig_type;
	bool lvl_active;
	int err;

	sub_irq = irq_find_mapping(domain, offset);
	if (sub_irq)
		handle_nested_irq(sub_irq);

	trig_type = info->trig_type[offset];
	if (trig_type & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW)) {
		/* check that the level condition has been handled */
		err = max77779_external_pmic_reg_read(core, sgpio_sts_reg, &sgpio_sts);
		if (err) {
			dev_err_ratelimited(info->dev, "read error %d\n", err);
			return true;
		}

		sgpio_val = sgpio_sts & MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_MASK;
		trig_type = info->trig_type[offset];
		lvl_active = ((trig_type == IRQF_TRIGGER_LOW) && !sgpio_val) ||
			     ((trig_type == IRQF_TRIGGER_HIGH) && sgpio_val);

		return !lvl_active;
	}
	return true;
}

static irqreturn_t max77779_sgpio_irq_handler(int irq, void *ptr)
{
	struct max77779_pmic_sgpio_info *info = ptr;
	struct device *core = info->core;
	uint8_t sgpio_int, sgpio_handled = 0;
	int offset;
	int err;
	bool handled;

	err = max77779_external_pmic_reg_read(core, MAX77779_PMIC_GPIO_SGPIO_INT,
			&sgpio_int);
	if (err) {
		dev_err_ratelimited(info->dev, "read error %d\n", err);
		return IRQ_NONE;
	}

	for (offset = 0; offset < MAX77779_SGPIO_NUM_GPIOS; offset++) {
		if (sgpio_int & BIT(offset)) {
			handled = max77779_sgpio_handle_nested_irq(info,
					offset);
			sgpio_handled |= handled << offset;
		}
	}

	/*
	 * Only clear the handled bits.
	 * We will be called again for any that don't get cleared.
	 */
	err = max77779_external_pmic_reg_write(core, MAX77779_PMIC_GPIO_SGPIO_INT,
			sgpio_handled);
	if (err)
		dev_err_ratelimited(info->dev, "write error %d\n", err);

	return IRQ_HANDLED;
}

static struct irq_chip max77779_pmic_sgpio_irq_chip = {
	.name = "max77779_sgpio_irq",
	.irq_enable = max77779_pmic_sgpio_irq_enable,
	.irq_disable = max77779_pmic_sgpio_irq_disable,
	.irq_mask = max77779_pmic_sgpio_irq_mask,
	.irq_unmask = max77779_pmic_sgpio_irq_unmask,
	.irq_set_type = max77779_pmic_sgpio_set_irq_type,
	.irq_set_wake = max77779_pmic_sgpio_irq_set_wake,
	.irq_bus_lock = max77779_pmic_sgpio_bus_lock,
	.irq_bus_sync_unlock = max77779_pmic_sgpio_bus_sync_unlock
};

static int max77779_pmic_sgpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77779_pmic_sgpio_info *info;
	struct gpio_chip *gpio_chip;
	int irq_in;
	int err;

	if (!dev->of_node)
		return -ENODEV;

	irq_in = platform_get_irq(pdev, 0);
	if (irq_in < 0) {
		dev_err(dev, "%s failed to get irq ret = %d\n", __func__, irq_in);
		return -ENODEV;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->irq = irq_in;
	info->dev = dev;
	info->core = dev->parent;
	mutex_init(&info->lock);

	/* Setup GPIO controller */
	gpio_chip = &info->gpio_chip;

	gpio_chip->label = "max77779_sgpio";
	gpio_chip->parent = info->dev;
	gpio_chip->owner = THIS_MODULE;
	gpio_chip->get_direction = max77779_pmic_sgpio_get_direction;
	gpio_chip->direction_input = max77779_pmic_sgpio_direction_input;
	gpio_chip->direction_output = max77779_pmic_sgpio_direction_output;
	gpio_chip->get = max77779_pmic_sgpio_get;
	gpio_chip->set = max77779_pmic_sgpio_set;
	gpio_chip->request = gpiochip_generic_request;
	gpio_chip->free = gpiochip_generic_free;
	gpio_chip->set_config = gpiochip_generic_config;
	gpio_chip->base = -1;
	gpio_chip->can_sleep = true;
	gpio_chip->of_node = dev->of_node;
	gpio_chip->ngpio = MAX77779_SGPIO_NUM_GPIOS;

	gpio_irq_chip_set_chip(&gpio_chip->irq, &max77779_pmic_sgpio_irq_chip);

	gpio_chip->irq.default_type = IRQ_TYPE_NONE;
	gpio_chip->irq.handler = handle_simple_irq;
	gpio_chip->irq.parent_handler = NULL;
	gpio_chip->irq.num_parents = 0;
	gpio_chip->irq.parents = NULL;
	gpio_chip->irq.threaded = true;
	gpio_chip->irq.init_hw = max77779_pmic_sgpio_irq_init_hw;
	gpio_chip->irq.init_valid_mask = max77779_pmic_sgpio_set_irq_valid_mask;
	gpio_chip->irq.first = 0;

	platform_set_drvdata(pdev, info);

	err = devm_gpiochip_add_data(dev, gpio_chip, info);
	if (err) {
		dev_err(dev, "Failed to initialize gpio chip err = %d\n", err);
		return err;
	}

	err = devm_request_threaded_irq(info->dev, irq_in, NULL,
			max77779_sgpio_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"max77779_pmic_sgpio_irq", info);
	if (err < 0) {
		dev_err(dev, "failed get irq thread err = %d\n", err);
		return -ENODEV;
	}

	return 0;
}

static int max77779_pmic_sgpio_remove(struct platform_device *pdev)
{
	return 0;
}
static const struct platform_device_id max77779_pmic_sgpio_id[] = {
	{ "max77779-pmic-sgpio", 0},
	{},
};

MODULE_DEVICE_TABLE(platform, max77779_pmic_sgpio_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id max77779_pmic_sgpio_match_table[] = {
	{ .compatible = "max77779-pmic-sgpio",},
	{ },
};
#endif

static struct platform_driver max77779_pmic_sgpio_driver = {
	.probe = max77779_pmic_sgpio_probe,
	.remove = max77779_pmic_sgpio_remove,
	.id_table = max77779_pmic_sgpio_id,
	.driver = {
		.name = "max77779-pmic-sgpio",
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = max77779_pmic_sgpio_match_table,
#endif
	},
};

module_platform_driver(max77779_pmic_sgpio_driver);

MODULE_DESCRIPTION("Maxim 77779 SGPIO driver");
MODULE_AUTHOR("James Wylder <jwylder@google.com>");
MODULE_LICENSE("GPL");
