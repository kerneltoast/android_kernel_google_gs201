// SPDX-License-Identifier: GPL-2.0
/*
 * Platform device driver for GXP.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include "gxp-internal.h"

#include "gxp-common-platform.c"

void gxp_iommu_setup_shareability(struct gxp_dev *gxp)
{
	/* IO coherency not supported */
}

static int gxp_platform_probe(struct platform_device *pdev)
{
	struct gxp_dev *gxp =
		devm_kzalloc(&pdev->dev, sizeof(*gxp), GFP_KERNEL);

	if (!gxp)
		return -ENOMEM;

	return gxp_common_platform_probe(pdev, gxp);
}

static int gxp_platform_remove(struct platform_device *pdev)
{
	return gxp_common_platform_remove(pdev);
}

#ifdef CONFIG_OF
static const struct of_device_id gxp_of_match[] = {
	{ .compatible = "google,gxp", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, gxp_of_match);
#endif

static struct platform_driver gxp_platform_driver = {
	.probe = gxp_platform_probe,
	.remove = gxp_platform_remove,
	.driver = {
			.name = GXP_DRIVER_NAME,
			.of_match_table = of_match_ptr(gxp_of_match),
#if IS_ENABLED(CONFIG_PM_SLEEP)
			.pm = &gxp_pm_ops,
#endif
		},
};

static int __init gxp_platform_init(void)
{
	gxp_common_platform_reg_sscd();
	return platform_driver_register(&gxp_platform_driver);
}

static void __exit gxp_platform_exit(void)
{
	platform_driver_unregister(&gxp_platform_driver);
	gxp_common_platform_unreg_sscd();
}

bool gxp_is_direct_mode(struct gxp_dev *gxp)
{
	return true;
}

enum gxp_chip_revision gxp_get_chip_revision(struct gxp_dev *gxp)
{
	return GXP_CHIP_ANY;
}

MODULE_DESCRIPTION("Google GXP platform driver");
MODULE_LICENSE("GPL v2");
MODULE_INFO(gitinfo, GIT_REPO_TAG);
module_init(gxp_platform_init);
module_exit(gxp_platform_exit);
