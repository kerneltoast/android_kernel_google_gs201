// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_fb.c
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

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <uapi/drm/exynos_drm.h>
#include <linux/dma-buf.h>
#include <linux/ion_exynos.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_fbdev.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_dsim.h"

#define to_exynos_fb(x)	container_of(x, struct exynos_drm_fb, fb)

static void exynos_drm_free_buf_object(struct exynos_drm_buf *obj)
{
	if (!IS_ERR_VALUE(obj->dma_addr))
		ion_iovmm_unmap(obj->attachment, obj->dma_addr);

	if (!IS_ERR_OR_NULL(obj->attachment) && !IS_ERR_OR_NULL(obj->sgt))
		dma_buf_unmap_attachment(obj->attachment, obj->sgt,
				DMA_TO_DEVICE);

	if (obj->dmabuf && !IS_ERR_OR_NULL(obj->attachment))
		dma_buf_detach(obj->dmabuf, obj->attachment);

	if (obj->dmabuf)
		dma_buf_put(obj->dmabuf);

	kfree(obj);
}

static void exynos_drm_fb_destroy(struct drm_framebuffer *fb)
{
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	unsigned int i;

	drm_framebuffer_cleanup(fb);

	for (i = 0; i < ARRAY_SIZE(exynos_fb->exynos_buf); i++) {
		if (exynos_fb->exynos_buf[i] == NULL)
			continue;

		exynos_drm_free_buf_object(exynos_fb->exynos_buf[i]);
	}

	kfree(exynos_fb);
	exynos_fb = NULL;
}

static int exynos_drm_fb_create_handle(struct drm_framebuffer *fb,
					struct drm_file *file_priv,
					unsigned int *handle)
{
	return 0;
}

static const struct drm_framebuffer_funcs exynos_drm_fb_funcs = {
	.destroy	= exynos_drm_fb_destroy,
	.create_handle	= exynos_drm_fb_create_handle,
};

struct drm_framebuffer *
exynos_drm_framebuffer_init(struct drm_device *dev,
			    const struct drm_mode_fb_cmd2 *mode_cmd,
			    struct exynos_drm_buf **exynos_buf,
			    int count)
{
	struct exynos_drm_fb *exynos_fb;
	int i;
	int ret;

	exynos_fb = kzalloc(sizeof(*exynos_fb), GFP_KERNEL);
	if (!exynos_fb)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < count; i++) {
		exynos_fb->exynos_buf[i] = exynos_buf[i];
		/*
		 * TODO: buffer handling will be modified in case of
		 * multi planar
		 */
		exynos_fb->dma_addr[i] = exynos_buf[i]->dma_addr
						+ mode_cmd->offsets[i];
	}

	drm_helper_mode_fill_fb_struct(dev, &exynos_fb->fb, mode_cmd);

	ret = drm_framebuffer_init(dev, &exynos_fb->fb, &exynos_drm_fb_funcs);
	if (ret < 0) {
		DRM_ERROR("failed to initialize framebuffer\n");
		goto err;
	}

	return &exynos_fb->fb;

err:
	kfree(exynos_fb);
	return ERR_PTR(ret);
}

int exynos_drm_import_handle(struct exynos_drm_buf *obj, u32 handle,
		size_t size)
{
	/* TODO: iovmm_activate will be moved to exynos_drm_bind */
	struct dsim_device *dsim = dsim_drvdata[0];

	obj->dmabuf = dma_buf_get(handle);
	if (IS_ERR_OR_NULL(obj->dmabuf)) {
		DRM_ERROR("failed to get dma_buf:%ld\n", PTR_ERR(obj->dmabuf));
		return PTR_ERR(obj->dmabuf);
	}

	DRM_INFO("%s:%d, dma_buf->size(%d)\n", __func__, __LINE__,
			obj->dmabuf->size);

	obj->attachment = dma_buf_attach(obj->dmabuf, dsim->dev);
	if (IS_ERR_OR_NULL(obj->attachment)) {
		DRM_ERROR("dma_buf_attach() failed: %ld\n",
				PTR_ERR(obj->attachment));
		return PTR_ERR(obj->attachment);
	}

	obj->sgt = dma_buf_map_attachment(obj->attachment, DMA_TO_DEVICE);
	if (IS_ERR_OR_NULL(obj->sgt)) {
		DRM_ERROR("dma_buf_map_attachment() failed: %ld\n",
				PTR_ERR(obj->sgt));
		return PTR_ERR(obj->sgt);
	}

	/* This is DVA(Device Virtual Address) for setting base address SFR */
	obj->dma_addr = ion_iovmm_map(obj->attachment, 0, size, DMA_TO_DEVICE,
			0);
	if (IS_ERR_VALUE(obj->dma_addr)) {
		DRM_ERROR("ion_iovmm_map() failed: %pa\n", &obj->dma_addr);
		return -EINVAL;
	}

	DRM_INFO("%s:%d, handle(%d), DVA(0x%p)\n", __func__, __LINE__,
			handle, obj->dma_addr);

	return 0;
}

static struct drm_framebuffer *
exynos_user_fb_create(struct drm_device *dev, struct drm_file *file_priv,
		      const struct drm_mode_fb_cmd2 *mode_cmd)
{
	const struct drm_format_info *info = drm_get_format_info(dev, mode_cmd);
	struct exynos_drm_buf *exynos_buf[MAX_FB_BUFFER];
	struct drm_framebuffer *fb;
	u32 height;
	size_t size;
	int i;
	int ret;

	DRM_INFO("%s +\n", __func__);

	for (i = 0; i < MAX_FB_BUFFER; i++)
		exynos_buf[i] = NULL;

	for (i = 0; i < info->num_planes; i++) {
		exynos_buf[i] = kzalloc(sizeof(struct exynos_drm_buf),
				GFP_KERNEL);
		if (!exynos_buf[i])
			return ERR_PTR(-ENOMEM);

		height = (i == 0) ? mode_cmd->height :
				     DIV_ROUND_UP(mode_cmd->height, info->vsub);
		size = height * mode_cmd->pitches[i] + mode_cmd->offsets[i];

		ret = exynos_drm_import_handle(exynos_buf[i],
				mode_cmd->handles[i], size);
		if (ret)
			return ERR_PTR(ret);
	}

	DRM_INFO("width(%d), height(%d), pitches(%d)\n", mode_cmd->width,
			mode_cmd->height, mode_cmd->pitches[0]);
	DRM_INFO("offset(%d), handle(%d), size(%d)\n", mode_cmd->offsets[0],
			mode_cmd->handles[0], size);

	fb = exynos_drm_framebuffer_init(dev, mode_cmd, exynos_buf, i);
	if (IS_ERR(fb))
		return fb;

	return fb;
}

dma_addr_t exynos_drm_fb_dma_addr(struct drm_framebuffer *fb, int index)
{
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);

	if (WARN_ON_ONCE(index >= MAX_FB_BUFFER))
		return 0;

	DRM_INFO("%s:%d, dma_addr[%d] = 0x%p\n", __func__, __LINE__,
			index, exynos_fb->dma_addr[index]);
	return exynos_fb->dma_addr[index];
}

static struct drm_mode_config_helper_funcs exynos_drm_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static const struct drm_mode_config_funcs exynos_drm_mode_config_funcs = {
	.fb_create = exynos_user_fb_create,
	.output_poll_changed = exynos_drm_output_poll_changed,
	.atomic_check = exynos_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

void exynos_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * set max width and height as default value(4096x4096).
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &exynos_drm_mode_config_funcs;
	dev->mode_config.helper_private = &exynos_drm_mode_config_helpers;

	dev->mode_config.allow_fb_modifiers = true;
}
