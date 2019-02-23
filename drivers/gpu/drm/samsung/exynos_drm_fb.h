/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _EXYNOS_DRM_FB_H_
#define _EXYNOS_DRM_FB_H_

#include <linux/dma-buf.h>

#define MAX_FB_BUFFER	4

struct exynos_drm_buf {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
};

struct exynos_drm_fb {
	struct drm_framebuffer fb;
	dma_addr_t dma_addr[MAX_FB_BUFFER];
	struct exynos_drm_buf *exynos_buf[MAX_FB_BUFFER];
};

dma_addr_t exynos_drm_fb_dma_addr(struct drm_framebuffer *fb, int index);

void exynos_drm_mode_config_init(struct drm_device *dev);

#endif
