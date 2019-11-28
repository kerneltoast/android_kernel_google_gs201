/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 * Authors:
 *	Jiun Yu <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __EXYNOS_DRM_GEM__
#define __EXYNOS_DRM_GEM__

#include <drm/drm_gem.h>
#include <drm/drm_file.h>
#include <drm/drm_device.h>
#include <drm/drm_mode.h>
#include <linux/dma-buf.h>

struct exynos_drm_gem {
	struct drm_gem_object base;
	unsigned long size;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	void *kaddr;
	unsigned int flags;
};

int exynos_drm_gem_dumb_create(struct drm_file *file_priv,
			       struct drm_device *dev,
			       struct drm_mode_create_dumb *args);
int exynos_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma);
int exynos_drm_gem_dumb_map_offset(struct drm_file *file_priv,
				   struct drm_device *dev, uint32_t handle,
				   uint64_t *offset);
void exynos_drm_gem_free_object(struct drm_gem_object *obj);
struct exynos_drm_gem *exynos_drm_gem_get(struct drm_file *filp,
					  unsigned int gem_handle);

static inline void exynos_drm_gem_put(struct exynos_drm_gem *exynos_gem)
{
	drm_gem_object_put_unlocked(&exynos_gem->base);
}

#define to_exynos_gem(x)    container_of(x, struct exynos_drm_gem, base)

#endif /* __EXYNOS_DRM_GEM__ */
