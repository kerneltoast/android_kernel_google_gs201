// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos PM domain debugfs support.
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include <soc/google/exynos-pd.h>

#ifdef CONFIG_DEBUG_FS
static struct dentry *exynos_pd_dbg_root;

static int exynos_pd_dbg_long_test(struct device *dev)
{
	int ret = 0, i;

	dev_info(dev, "Starting test\n");

	if (pm_runtime_enabled(dev) && pm_runtime_active(dev)) {
		ret = pm_runtime_put_sync(dev);
		if (ret) {
			dev_err(dev, "put failed\n");
			return ret;
		}
	}

	for (i = 0; i < 100; i++) {
		ret = pm_runtime_get_sync(dev);
		if (ret) {
			dev_err(dev, "%d get failed\n", i);
			return ret;
		}
		mdelay(50);
		ret = pm_runtime_put_sync(dev);
		if (ret) {
			dev_err(dev, "%d put failed\n", i);
			return ret;
		}
		mdelay(50);
	}

	dev_info(dev, "Test done\n");

	return ret;
}

static int debug_state_get(void *data, u64 *val)
{
	struct device *dev = data;

	*val = dev->power.runtime_status;

	return 0;
}

static int debug_state_set(void *data, u64 val)
{
	struct device *dev = data;
	int ret = 0;

	if (val == 0) {
		ret = pm_runtime_put_sync(dev);
		if (ret)
			dev_err(dev, "put failed");
	} else if (val == 1) {
		ret = pm_runtime_get_sync(dev);
		if (ret)
			dev_err(dev, "get failed");
	} else {
		ret = exynos_pd_dbg_long_test(dev);
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(state_fops, debug_state_get, debug_state_set, "%llu\n");
#endif

static int exynos_pd_dbg_probe(struct platform_device *pdev)
{
#ifdef CONFIG_DEBUG_FS
	if (!exynos_pd_dbg_root)
		exynos_pd_dbg_root = debugfs_create_dir("exynos-pd", NULL);

	debugfs_create_file(dev_name(&pdev->dev), 0644, exynos_pd_dbg_root,
			    &pdev->dev, &state_fops);
#endif

	pm_runtime_enable(&pdev->dev);

	if (pm_runtime_get_sync(&pdev->dev))
		dev_err(&pdev->dev, "probe get failed\n");
	else if (pm_runtime_put_sync(&pdev->dev))
		dev_err(&pdev->dev, "probe put failed\n");

	return 0;
}

static int exynos_pd_dbg_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(exynos_pd_dbg_root);
	exynos_pd_dbg_root = NULL;
#endif
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id exynos_pd_dbg_match[] = {
	{
		.compatible = "samsung,exynos-pd-dbg",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_pd_dbg_match);
#endif

static struct platform_driver exynos_pd_dbg_drv = {
	.probe		= exynos_pd_dbg_probe,
	.remove		= exynos_pd_dbg_remove,
	.driver		= {
		.name	= "exynos_pd_dbg",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = exynos_pd_dbg_match,
#endif
	},
};

static int __init exynos_pd_dbg_init(void)
{
	return platform_driver_register(&exynos_pd_dbg_drv);
}
late_initcall(exynos_pd_dbg_init);

MODULE_SOFTDEP("pre: exynos-pd");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos PM domain debug");
