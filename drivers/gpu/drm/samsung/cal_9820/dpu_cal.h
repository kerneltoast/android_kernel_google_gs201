/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS9_CAL_DPU_H__
#define __EXYNOS9_CAL_DPU_H__

struct dpu_panel_timing {
	unsigned int vactive;
	unsigned int vfp;
	unsigned int vsa;
	unsigned int vbp;

	unsigned int hactive;
	unsigned int hfp;
	unsigned int hsa;
	unsigned int hbp;
};

#endif
