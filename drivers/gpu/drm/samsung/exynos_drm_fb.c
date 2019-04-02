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

#include "exynos_drm_decon.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_fbdev.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_format.h"

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

	DRM_INFO("%s:%d, handle(%d), DVA(0x%llx)\n", __func__, __LINE__,
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
	DRM_INFO("offset(%d), handle(%d), size(%lu)\n", mode_cmd->offsets[0],
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

	DRM_INFO("%s:%d, dma_addr[%d] = 0x%llx\n", __func__, __LINE__,
			index, exynos_fb->dma_addr[index]);
	return exynos_fb->dma_addr[index];
}

void plane_state_to_win_config(struct decon_device *decon,
		struct exynos_drm_plane_state *state, int plane_idx)
{
	struct dpu_bts_win_config *win_config;
	struct exynos_drm_fb *exynos_fb = container_of(state->base.fb,
			struct exynos_drm_fb, fb);
	int zpos;

	zpos = state->base.zpos;
	win_config = &decon->bts.win_config[zpos];

	win_config->src_x = state->base.src_x >> 16;
	win_config->src_y = state->base.src_y >> 16;
	win_config->src_w = state->base.src_w >> 16;
	win_config->src_h = state->base.src_h >> 16;

	win_config->dst_x = state->base.crtc_x;
	win_config->dst_y = state->base.crtc_y;
	win_config->dst_w = state->base.crtc_w;
	win_config->dst_h = state->base.crtc_h;

	win_config->is_rot = state->base.rotation;
	win_config->is_afbc = state->afbc;
	win_config->state = DPU_WIN_STATE_BUFFER;
	win_config->format = convert_drm_format(state->base.fb->format->format);
	win_config->dpp_ch = plane_idx;

	memcpy(&decon->dpp[plane_idx]->fb, exynos_fb, sizeof(*exynos_fb));

	DRM_INFO("%s: src[%d %d %d %d], dst[%d %d %d %d]\n", __func__,
			win_config->src_x, win_config->src_y,
			win_config->src_w, win_config->src_h,
			win_config->dst_x, win_config->dst_y,
			win_config->dst_w, win_config->dst_h);
	DRM_INFO("%s: rot[%d], afbc[%d], format[%d], ch[%d], zpos[%d]\n",
			__func__,
			win_config->is_rot, win_config->is_afbc,
			win_config->format, win_config->dpp_ch, zpos);
	DRM_INFO("%s: alpha[%d] blend mode[%d]\n", __func__,
			state->alpha, state->blend_mode);
}

static void display_mode_to_bts_info(struct drm_display_mode *mode,
		struct decon_device *decon)
{
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	decon->config.image_width = vm.hactive;
	decon->config.image_height = vm.vactive;
	decon->config.dsc.slice_width = DIV_ROUND_UP(decon->config.image_width,
			decon->config.dsc.slice_count);
	decon->bts.vbp = vm.vback_porch;
	decon->bts.vfp = vm.vfront_porch;
	decon->bts.vsa = vm.vsync_len;
	decon->bts.fps = mode->vrefresh;
}

void exynos_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	int i, j;
	struct drm_plane *plane;
	struct drm_plane_state *new_plane_state;
	struct exynos_drm_plane_state *new_exynos_state;
	struct exynos_drm_crtc *exynos_crtc;
	struct decon_device *decon[MAX_DECON_CNT] = {};
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	int max_planes;
	int id = 0;

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		DRM_INFO("[CRTC:%d:%s] old en:%d active:%d change[%d %d %d]\n",
				crtc->base.id, crtc->name,
				old_crtc_state->enable, old_crtc_state->active,
				old_crtc_state->planes_changed,
				old_crtc_state->mode_changed,
				old_crtc_state->active_changed);

		DRM_INFO("[CRTC:%d:%s] new en:%d active:%d change[%d %d %d]\n",
				crtc->base.id, crtc->name,
				new_crtc_state->enable, new_crtc_state->active,
				new_crtc_state->planes_changed,
				new_crtc_state->mode_changed,
				new_crtc_state->active_changed);

		exynos_crtc = container_of(crtc, struct exynos_drm_crtc, base);
		id = crtc->index;
		decon[id] = exynos_crtc->ctx;

		/* acquire initial bandwidth when DECON is enabled. */
		if (!old_crtc_state->active && new_crtc_state->active) {
			display_mode_to_bts_info(&new_crtc_state->mode,
					decon[id]);
			decon[id]->bts.ops->bts_acquire_bw(decon[id]);
		}

		/* initialize BTS structure of each DECON */
		if (new_crtc_state->planes_changed && new_crtc_state->active) {
			max_planes =
				old_state->dev->mode_config.num_total_plane;
			for (j = 0; j < max_planes; ++j) {
				decon[id]->bts.win_config[j].state =
					DPU_WIN_STATE_DISABLED;

				memset(&decon[id]->dpp[i]->fb, 0,
						sizeof(struct exynos_drm_fb));
			}
		}
	}

	for_each_new_plane_in_state(old_state, plane, new_plane_state, i) {
		if (!new_plane_state->crtc)
			continue;

		new_exynos_state = to_exynos_plane_state(new_plane_state);
		exynos_crtc = container_of(new_plane_state->crtc,
				struct exynos_drm_crtc, base);
		id = new_plane_state->crtc->index;
		decon[id] = exynos_crtc->ctx;

		plane_state_to_win_config(decon[id], new_exynos_state, i);
	}

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		exynos_crtc = container_of(crtc, struct exynos_drm_crtc, base);
		id = crtc->index;
		decon[id] = exynos_crtc->ctx;

		if (new_crtc_state->planes_changed && new_crtc_state->active) {
			DPU_EVENT_LOG_ATOMIC_COMMIT(decon[id]->id);
			decon[id]->bts.ops->bts_calc_bw(decon[i]);
			decon[id]->bts.ops->bts_update_bw(decon[i], false);
		}
	}

	drm_atomic_helper_commit_tail_rpm(old_state);

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		exynos_crtc = container_of(crtc, struct exynos_drm_crtc, base);
		id = crtc->index;
		decon[id] = exynos_crtc->ctx;

		if (new_crtc_state->planes_changed && new_crtc_state->active) {
			if (decon[id]->config.mode.op_mode ==
					DECON_MIPI_COMMAND_MODE)
				decon_reg_set_trigger(decon[id]->id,
						&decon[id]->config.mode,
						DECON_TRIG_MASK);

			decon[id]->bts.ops->bts_update_bw(decon[i], true);
		}

		if (old_crtc_state->active && !new_crtc_state->active)
			decon[id]->bts.ops->bts_release_bw(decon[id]);
	}
}

static struct drm_mode_config_helper_funcs exynos_drm_mode_config_helpers = {
	.atomic_commit_tail = exynos_atomic_commit_tail,
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
