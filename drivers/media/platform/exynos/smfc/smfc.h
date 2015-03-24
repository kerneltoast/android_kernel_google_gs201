/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * The main header file of Samsung Exynos SMFC Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MEDIA_EXYNOS_SMFC_H_
#define _MEDIA_EXYNOS_SMFC_H_

struct device;

struct smfc_dev {
	struct device *dev;
	void __iomem *reg;
	int device_id;
	u32 hwver;

	struct clk *clk_gate;
	struct clk *clk_gate2; /* available if clk_gate is valid */
};

#endif /* _MEDIA_EXYNOS_SMFC_H_ */
