// SPDX-License-Identifier: GPL-2.0
/*
 * Janeiro platform device driver for the Google Edge TPU ML accelerator.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-mobile-platform.h"
#include "janeiro-platform.h"

#include "edgetpu-mobile-platform.c"

static const struct of_device_id edgetpu_of_match[] = {
	/* TODO(b/190677977): remove  */
	{ .compatible = "google,darwinn", },
	{ .compatible = "google,edgetpu-gs201", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, edgetpu_of_match);

/*
 * Set shareability for enabling IO coherency in Janeiro
 */
static int janeiro_mmu_set_shareability(struct device *dev, u32 reg_base)
{
	void __iomem *addr = ioremap(reg_base, PAGE_SIZE);

	if (!addr) {
		dev_err(dev, "sysreg ioremap failed\n");
		return -ENOMEM;
	}

	writel_relaxed(SHAREABLE_WRITE | SHAREABLE_READ | INNER_SHAREABLE,
		       addr + EDGETPU_SYSREG_TPU_SHAREABILITY);
	iounmap(addr);

	return 0;
}

static int janeiro_parse_set_dt_property(struct edgetpu_mobile_platform_dev *etmdev)
{
	int ret;
	u32 reg;
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	struct device *dev = etdev->dev;

	ret = edgetpu_mobile_platform_set_fw_ctx_memory(etmdev);
	if (ret) {
		etdev_err(etdev, "Failed to initialize fw context memory: %d", ret);
		return ret;
	}

	if (!of_find_property(dev->of_node, "edgetpu,shareability", NULL)) {
		ret = -ENODEV;
		goto err;
	}
	ret = of_property_read_u32_index(dev->of_node, "edgetpu,shareability", 0, &reg);
	if (ret)
		goto err;
	ret = janeiro_mmu_set_shareability(dev, reg);
err:
	if (ret)
		etdev_warn(etdev, "failed to enable shareability: %d", ret);

	return 0;
}

static int janeiro_platform_after_probe(struct edgetpu_mobile_platform_dev *etmdev)
{
	return janeiro_parse_set_dt_property(etmdev);
}

static int edgetpu_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct janeiro_platform_dev *jpdev;
	struct edgetpu_mobile_platform_dev *etmdev;

	jpdev = devm_kzalloc(dev, sizeof(*jpdev), GFP_KERNEL);
	if (!jpdev)
		return -ENOMEM;

	etmdev = &jpdev->mobile_dev;
	etmdev->after_probe = janeiro_platform_after_probe;
	return edgetpu_mobile_platform_probe(pdev, etmdev);
}

static int edgetpu_platform_remove(struct platform_device *pdev)
{
	return edgetpu_mobile_platform_remove(pdev);
}

#if IS_ENABLED(CONFIG_PM_SLEEP)

static int edgetpu_platform_suspend(struct device *dev)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_pm_suspend(etdev);
}

static int edgetpu_platform_resume(struct device *dev)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_pm_resume(etdev);
}

#endif /* IS_ENABLED(CONFIG_PM_SLEEP) */

static const struct dev_pm_ops edgetpu_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(edgetpu_platform_suspend,
				edgetpu_platform_resume)
};

static struct platform_driver edgetpu_platform_driver = {
	.probe = edgetpu_platform_probe,
	.remove = edgetpu_platform_remove,
	.driver = {
		.name = "edgetpu_platform",
		.of_match_table = edgetpu_of_match,
		.pm = &edgetpu_pm_ops,
	},
};

static int __init edgetpu_platform_init(void)
{
	int ret;

	ret = edgetpu_init();
	if (ret)
		return ret;
	return platform_driver_register(&edgetpu_platform_driver);
}

static void __exit edgetpu_platform_exit(void)
{
	platform_driver_unregister(&edgetpu_platform_driver);
	edgetpu_exit();
}

MODULE_DESCRIPTION("Google Edge TPU platform driver");
MODULE_LICENSE("GPL v2");
module_init(edgetpu_platform_init);
module_exit(edgetpu_platform_exit);
MODULE_FIRMWARE(EDGETPU_DEFAULT_FIRMWARE_NAME);
