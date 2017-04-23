/*
 * linux/drivers/gpu/exynos/g2d/g2d_drv.c
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * Contact: Hyesoo Yu <hyesoo.yu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/exynos_iovmm.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "g2d.h"
#include "g2d_regs.h"
#include "g2d_task.h"
#include "g2d_uapi.h"

#define MODULE_NAME "exynos-g2d"

int g2d_device_run(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	return 0;
}

static irqreturn_t g2d_irq_handler(int irq, void *priv)
{
	return IRQ_HANDLED;
}

static __u32 get_hw_version(struct g2d_device *g2d_dev, __u32 *version)
{
	int ret;

	ret = pm_runtime_get_sync(g2d_dev->dev);
	if (ret < 0) {
		dev_err(g2d_dev->dev, "Failed to enable power (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(g2d_dev->clock);
	if (ret < 0) {
		dev_err(g2d_dev->dev, "Failed to enable clock (%d)\n", ret);
	} else {
		*version = readl_relaxed(g2d_dev->reg + G2D_VERSION_INFO_REG);
		clk_disable_unprepare(g2d_dev->clock);
	}

	pm_runtime_put(g2d_dev->dev);

	return ret;
}

static int g2d_open(struct inode *inode, struct file *filp)
{
	struct g2d_device *g2d_dev = container_of(filp->private_data,
						  struct g2d_device, misc);
	struct g2d_context *g2d_ctx;

	g2d_ctx = kzalloc(sizeof(*g2d_ctx), GFP_KERNEL);
	if (!g2d_ctx)
		return -ENOMEM;

	filp->private_data = g2d_ctx;

	g2d_ctx->g2d_dev = g2d_dev;

	return 0;
}

static int g2d_release(struct inode *inode, struct file *filp)
{
	struct g2d_context *g2d_ctx = filp->private_data;

	kfree(g2d_ctx);

	return 0;
}

static long g2d_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct g2d_context *ctx = filp->private_data;
	struct g2d_device *g2d_dev = ctx->g2d_dev;

	switch (cmd) {
	case G2D_IOC_PROCESS:
	{
		struct g2d_task *task;

		task = g2d_get_free_task(g2d_dev);
		if (task == NULL)
			return -EBUSY;

		kref_init(&task->starter);

		g2d_start_task(task);
	}
	}
	return 0;
}

static long g2d_compat_ioctl(struct file *filp,
			     unsigned int cmd, unsigned long arg)
{
	return 0;
}

static const struct file_operations g2d_fops = {
	.owner          = THIS_MODULE,
	.open           = g2d_open,
	.release        = g2d_release,
	.unlocked_ioctl	= g2d_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= g2d_compat_ioctl,
#endif
};

static int g2d_probe(struct platform_device *pdev)
{
	struct g2d_device *g2d_dev;
	struct resource *res;
	__u32 version;
	int ret;

	g2d_dev = devm_kzalloc(&pdev->dev, sizeof(*g2d_dev), GFP_KERNEL);
	if (!g2d_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, g2d_dev);
	g2d_dev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	g2d_dev->reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(g2d_dev->reg))
		return PTR_ERR(g2d_dev->reg);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get IRQ resource");
		return -ENOENT;
	}

	ret = devm_request_irq(&pdev->dev, res->start,
			       g2d_irq_handler, 0, pdev->name, g2d_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to install IRQ handler");
		return ret;
	}

	g2d_dev->clock = devm_clk_get(&pdev->dev, "gate");
	if (IS_ERR(g2d_dev->clock)) {
		dev_err(&pdev->dev, "Failed to get clock (%ld)\n",
					PTR_ERR(g2d_dev->clock));
		return PTR_ERR(g2d_dev->clock);
	}

	ret = iovmm_activate(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to activate iommu\n");
		return ret;
	}

	/* prepare clock and enable runtime pm */
	pm_runtime_enable(&pdev->dev);

	ret = get_hw_version(g2d_dev, &version);
	if (ret < 0)
		goto err;

	g2d_dev->misc.minor = MISC_DYNAMIC_MINOR;
	g2d_dev->misc.name = "g2d";
	g2d_dev->misc.fops = &g2d_fops;

	/* misc register */
	ret = misc_register(&g2d_dev->misc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register misc device");
		goto err;
	}

	spin_lock_init(&g2d_dev->lock_task);

	INIT_LIST_HEAD(&g2d_dev->tasks_free);
	INIT_LIST_HEAD(&g2d_dev->tasks_prepared);
	INIT_LIST_HEAD(&g2d_dev->tasks_active);

	ret = g2d_create_tasks(g2d_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to create tasks");
		goto err_task;
	}

	dev_info(&pdev->dev, "Probed FIMG2D version %#010x\n", version);

	return 0;
err_task:
	misc_deregister(&g2d_dev->misc);
err:
	pm_runtime_disable(&pdev->dev);
	iovmm_deactivate(g2d_dev->dev);

	dev_err(&pdev->dev, "Failed to probe FIMG2D\n");

	return ret;
}

static int g2d_remove(struct platform_device *pdev)
{
	struct g2d_device *g2d_dev = platform_get_drvdata(pdev);

	g2d_destroy_tasks(g2d_dev);

	misc_deregister(&g2d_dev->misc);

	pm_runtime_disable(&pdev->dev);

	iovmm_deactivate(g2d_dev->dev);

	return 0;
}

static const struct of_device_id of_g2d_match[] = {
	{
		.compatible = "samsung,exynos9810-g2d",
	},
	{},
};

static struct platform_driver g2d_driver = {
	.probe		= g2d_probe,
	.remove		= g2d_remove,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_g2d_match),
	}
};

module_platform_driver(g2d_driver);
