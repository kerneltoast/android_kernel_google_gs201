/*
 * linux/drivers/gpu/exynos/g2d/g2d.h
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * Samsung Graphics 2D driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_G2D_H__
#define __EXYNOS_G2D_H__

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

struct g2d_device {
	struct miscdevice	misc;
	struct device		*dev;
	struct clk		*clock;
	void __iomem		*reg;
};

struct g2d_context {
	struct g2d_device	*g2d_dev;
};

#endif /* __EXYNOS_G2D_H__ */
