/* SPDX-License-Identifier: GPL-2.0-only
 * exynos_drm_dpp.h
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Authors:
 *	Seong-gyu Park <seongyu.park@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _EXYNOS_DRM_DPP_H_
#define _EXYNOS_DRM_DPP_H_

#include "exynos_drm_drv.h"
#include "cal_9820/dpp_cal.h"

enum EXYNOS9_DPP_FEATURES {
	/* Can reads the graphical image */
	DPP_SUPPORT_GRAPHIC	= 0 << 0,
	/* Can reads ARM Frame Buffer Compression (AFBC) encoded stream */
	DPP_SUPPORT_AFBC	= 1 << 0,
	/* Can reads the graphical image and video image */
	DPP_SUPPORT_VIDEO	= 1 << 1,
	/* Flips the image by X/Y axis */
	DPP_SUPPORT_FLIP	= 1 << 2,
};

enum dpp_state {
	DPP_STATE_OFF = 0,
	DPP_STATE_ON,
};

struct dpp_size_range {
	u32 min;
	u32 max;
	u32 align;
};

struct dpp_restriction {
	struct dpp_size_range src_f_w;
	struct dpp_size_range src_f_h;
	struct dpp_size_range src_w;
	struct dpp_size_range src_h;
	u32 src_x_align;
	u32 src_y_align;

	struct dpp_size_range dst_f_w;
	struct dpp_size_range dst_f_h;
	struct dpp_size_range dst_w;
	struct dpp_size_range dst_h;
	u32 dst_x_align;
	u32 dst_y_align;

	struct dpp_size_range blk_w;
	struct dpp_size_range blk_h;
	u32 blk_x_align;
	u32 blk_y_align;

	u32 src_h_rot_max; /* limit of source img height in case of rotation */

	u32 scale_down;
	u32 scale_up;
};

struct dpp_device {
	struct device *dev;

	u32 id;
	u32 is_support;
	u32 port;
	unsigned long attr;
	enum dpp_state state;

	int dma_irq;
	int dpp_irq;

	const uint32_t *pixel_formats;
	unsigned int num_pixel_formats;

	struct dpp_regs	regs;
	struct dpp_params_info win_config;

	spinlock_t slock;
	spinlock_t dma_slock;

	int decon_id; /* connected DECON id */

	struct dpp_restriction restriction;

	int (*check)(struct dpp_device *this_dpp,
				const struct exynos_drm_plane_state *state);
	int (*update)(struct dpp_device *this_dpp,
				const struct exynos_drm_plane_state *state);
	int (*disable)(struct dpp_device *this_dpp);
};

#ifdef CONFIG_OF
struct dpp_device *of_find_dpp_by_node(struct device_node *np);
#else
static struct dpp_device *of_find_dpp_by_node(struct device_node *np)
{
	return NULL;
}
#endif

#endif
