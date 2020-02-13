// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *	      http://www.samsung.com/
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <soc/google/debug-snapshot.h>

static irqreturn_t exynos_ecc_handler(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	dbg_snapshot_ecc_dump();
	if (desc && desc->action && desc->action->name)
		panic("%s", desc->action->name);
	else
		panic("%s-%d", __func__, irq);

	return IRQ_HANDLED;
}

static int exynos_ecc_handler_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct property *prop;
	const char *name;
	int err = 0, i = 0;
	unsigned int irq;

	of_property_for_each_string(np, "interrupt-names", prop, name) {
		if (!name) {
			dev_err(&pdev->dev, "no such name\n");
			err = -EINVAL;
			break;
		}

		irq = platform_get_irq(pdev, i++);
		err = devm_request_irq(&pdev->dev, irq, exynos_ecc_handler,
				IRQF_NOBALANCING, name, NULL);

		if (err) {
			dev_err(&pdev->dev, "unable to request irq%u for ecc handler[%s]\n",
					irq, name);
			break;
		} else {
			dev_info(&pdev->dev, "request irq%u for ecc handler[%s]\n",
					irq, name);
		}
		irq_set_affinity_hint(irq, cpu_possible_mask);
	}

	return err;
}

static const struct of_device_id exynos_ecc_handler_matches[] = {
	{ .compatible = "google,exynos-ecc-handler", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_ecc_handler_matches);

static struct platform_driver exynos_ecc_handler_driver = {
	.probe	= exynos_ecc_handler_probe,
	.driver	= {
		.name	= "exynos-ecc-handler",
		.of_match_table	= of_match_ptr(exynos_ecc_handler_matches),
	},
};
module_platform_driver(exynos_ecc_handler_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ECC Handler Driver");
