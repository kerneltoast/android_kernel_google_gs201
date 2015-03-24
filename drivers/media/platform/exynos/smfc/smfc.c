/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * The main source file of Samsung Exynos SMFC Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/io.h>

#include "smfc.h"
#include "smfc-regs.h"

static irqreturn_t exynos_smfc_irq_handler(int irq, void *priv)
{
	return IRQ_HANDLED;
}


static int smfc_init_clock(struct device *dev, struct smfc_dev *smfc)
{
	smfc->clk_gate = devm_clk_get(dev, "gate");
	if (IS_ERR(smfc->clk_gate)) {
		if (PTR_ERR(smfc->clk_gate) != -ENOENT) {
			dev_err(dev, "Failed(%ld) to get 'gate' clock",
				PTR_ERR(smfc->clk_gate));
			return PTR_ERR(smfc->clk_gate);
		}

		dev_info(dev, "'gate' clock is note present\n");
		smfc->clk_gate2 = ERR_PTR(-ENOENT);
		return 0;
	}

	smfc->clk_gate2 = devm_clk_get(dev, "gate2");
	if (IS_ERR(smfc->clk_gate2) && (PTR_ERR(smfc->clk_gate2) != -ENOENT)) {
		dev_err(dev, "Failed(%ld) to get 'gate2' clock\n",
				PTR_ERR(smfc->clk_gate2));
		clk_put(smfc->clk_gate);
		return PTR_ERR(smfc->clk_gate2);
	}

	return 0;
}

static int smfc_find_hw_version(struct device *dev, struct smfc_dev *smfc)
{
	int ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "Failed(%d) to get the local power\n", ret);
		return ret;
	}

	if (!IS_ERR(smfc->clk_gate)) {
		ret = clk_prepare_enable(smfc->clk_gate);
		if (!ret && !IS_ERR(smfc->clk_gate2))
			ret = clk_prepare_enable(smfc->clk_gate2);
		if (ret) {
			clk_disable_unprepare(smfc->clk_gate);
			dev_err(dev, "Failed(%d) to get gate clocks\n", ret);
			goto err_clk;
		}
	}

	if (ret >= 0) {
		smfc->hwver = readl(smfc->reg + REG_IP_VERSION_NUMBER);
		if (!IS_ERR(smfc->clk_gate)) {
			clk_disable_unprepare(smfc->clk_gate);
			if (!IS_ERR(smfc->clk_gate2))
				clk_disable_unprepare(smfc->clk_gate2);
		}
	}

err_clk:
	pm_runtime_put(dev);

	return ret;
}

static int exynos_smfc_probe(struct platform_device *pdev)
{
	struct smfc_dev *smfc;
	struct resource *res;
	int ret;

	smfc = devm_kzalloc(&pdev->dev, sizeof(*smfc), GFP_KERNEL);
	if (!smfc) {
		dev_err(&pdev->dev, "Failed to get allocate drvdata");
		return -ENOMEM;
	}

	smfc->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smfc->reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(smfc->reg))
		return PTR_ERR(smfc->reg);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Failed to get IRQ resource");
		return -ENOENT;
	}

	ret = devm_request_irq(&pdev->dev, res->start, exynos_smfc_irq_handler,
				0, pdev->name, smfc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to install IRQ handler");
		return ret;
	}

	ret = smfc_init_clock(&pdev->dev, smfc);
	if (ret)
		return ret;

	smfc->device_id = of_alias_get_id(pdev->dev.of_node, "jpeg");
	if (smfc->device_id < 0) {
		dev_info(&pdev->dev,
			"device ID is not declared: unique device\n");
		smfc->device_id = -1;
	}

	pm_runtime_enable(&pdev->dev);

	ret = smfc_find_hw_version(&pdev->dev, smfc);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, smfc);

	dev_info(&pdev->dev, "Probed H/W Version: %02x.%02x.%04x\n",
			(smfc->hwver >> 24) & 0xFF, (smfc->hwver >> 16) & 0xFF,
			smfc->hwver & 0xFFFF);
	return 0;
}

static void smfc_deinit_clock(struct smfc_dev *smfc)
{
	if (!IS_ERR(smfc->clk_gate2))
		clk_put(smfc->clk_gate2);
	if (!IS_ERR(smfc->clk_gate))
		clk_put(smfc->clk_gate);
}

static int exynos_smfc_remove(struct platform_device *pdev)
{
	struct smfc_dev *smfc = platform_get_drvdata(pdev);

	smfc_deinit_clock(smfc);

	return 0;
}

static const struct of_device_id exynos_smfc_match[] = {
	{
		.compatible = "samsung,exynos-jpeg",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_smfc_match);

#ifdef CONFIG_PM_SLEEP
static int smfc_suspend(struct device *dev)
{
	return 0;
}

static int smfc_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM
static int smfc_runtime_resume(struct device *dev)
{
	return 0;
}

static int smfc_runtime_suspend(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops exynos_smfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(smfc_suspend, smfc_resume)
	SET_RUNTIME_PM_OPS(NULL, smfc_runtime_resume, smfc_runtime_suspend)
};

static struct platform_driver exynos_smfc_driver = {
	.probe		= exynos_smfc_probe,
	.remove		= exynos_smfc_remove,
	.driver = {
		.name	= "exynos-jpeg",
		.owner	= THIS_MODULE,
		.pm	= &exynos_smfc_pm_ops,
		.of_match_table = of_match_ptr(exynos_smfc_match),
	}
};

module_platform_driver(exynos_smfc_driver);

MODULE_AUTHOR("Cho KyongHo <pullip.cho@samsung.com>");
MODULE_DESCRIPTION("Exynos Still MFC(JPEG) V4L2 Driver");
MODULE_LICENSE("GPL");
