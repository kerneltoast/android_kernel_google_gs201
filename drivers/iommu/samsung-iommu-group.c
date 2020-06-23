// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/iommu.h>
#include <linux/kmemleak.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

static int sysmmu_group_device_probe(struct platform_device *pdev)
{
	struct iommu_group *data;
	struct device *dev = &pdev->dev;

	data = iommu_group_alloc();
	if (IS_ERR(data)) {
		dev_err(dev, "Failed to alloc group, ret:%d\n", PTR_ERR(data));
		return PTR_ERR(data);
	}

	platform_set_drvdata(pdev, data);
	iommu_group_set_name(data, dev->of_node->name);

	dev_info(dev, "Initialized IOMMU group[%s]\n", dev->of_node->name);

	return 0;
}

static void sysmmu_group_device_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id sysmmu_group_of_match[] = {
	{ .compatible = "samsung,sysmmu-group" },
	{ }
};

static struct platform_driver sysmmu_group_driver = {
	.driver	= {
		.name			= "sysmmu-group",
		.of_match_table		= of_match_ptr(sysmmu_group_of_match),
		.suppress_bind_attrs	= true,
	},
	.probe = sysmmu_group_device_probe,
	.shutdown = sysmmu_group_device_shutdown,
};
module_platform_driver(sysmmu_group_driver);
MODULE_LICENSE("GPL v2");
