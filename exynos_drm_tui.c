// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 * Authors:
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) "[TUI Driver] " fmt

#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/dma-buf.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <drm/samsung_drm.h>

#include "exynos_drm_tui.h"
#include "exynos_drm_drv.h"

#define DRV_NAME "tui-driver"

struct tui_driver_priv {
	struct device *dev;
	struct miscdevice *misc;
};

/*
 * Device file ops
 */
static int exynos_tui_open(__maybe_unused struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct tui_driver_priv *priv;

	pr_debug("%s\n", __func__);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = mdev->this_device;
	filp->private_data = priv;

	return 0;
}

static int exynos_tui_release(struct inode *inode, struct file *filp)
{
	struct tui_driver_priv *priv = filp->private_data;
	(void)inode;

	pr_debug("%s\n", __func__);

	kfree(priv);

	return 0;
}

/*
 * Ioctls
 */
static long exynos_tui_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	long ret = 0;

	/* Handle command */
	switch (cmd) {
	case EXYNOS_START_TUI:
		/* Prepare display for TUI / Deactivate linux UI drivers */
		ret = exynos_atomic_enter_tui();
		if (ret < 0)
			pr_err("failed to enter TUI\n");
		break;
	case EXYNOS_FINISH_TUI:
		ret = exynos_atomic_exit_tui();
		if (ret < 0)
			pr_err("failed to exit TUI\n");
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static const struct file_operations exynos_tui_fops = {
	.open           = exynos_tui_open,
	.release        = exynos_tui_release,
	.unlocked_ioctl = exynos_tui_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = exynos_tui_ioctl,
#endif
};

static int exynos_tui_probe(struct platform_device *pdev)
{
	struct miscdevice *misc_dev;
	static uint64_t dma_mask;
	int ret = 0;

	/* Create a char device: we want to create it anew */
	misc_dev = devm_kzalloc(&pdev->dev, sizeof(*misc_dev), GFP_KERNEL);
	if (!misc_dev)
		return -ENOMEM;

	dma_mask = DMA_BIT_MASK(48);

	misc_dev->minor = MISC_DYNAMIC_MINOR;
	misc_dev->fops = &exynos_tui_fops;
	misc_dev->name = DRV_NAME;
	misc_dev->parent = pdev->dev.parent;

	dev_set_drvdata(&pdev->dev, misc_dev);

	ret = misc_register(misc_dev);
	if (ret == 0) {
		misc_dev->this_device->dma_mask = &dma_mask;
		dma_set_coherent_mask(misc_dev->this_device, dma_mask);
	}
	return ret;
}

static int exynos_tui_remove(struct platform_device *pdev)
{
	struct miscdevice *misc_dev = dev_get_drvdata(&pdev->dev);

	misc_deregister(misc_dev);

	return 0;
}

struct platform_driver tui_driver = {
	.driver = {
		.name = DRV_NAME,
	},
	.probe = exynos_tui_probe,
	.remove = exynos_tui_remove,
};

MODULE_DESCRIPTION("Samsung SoC TUI Driver");
MODULE_LICENSE("GPL");
