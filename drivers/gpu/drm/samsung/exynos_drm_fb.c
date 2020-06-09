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
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <uapi/drm/exynos_drm.h>
#include <uapi/linux/videodev2_exynos_media.h>
#include <linux/dma-buf.h>

#include <exynos_drm_decon.h>
#include <exynos_drm_drv.h>
#include <exynos_drm_fb.h>
#include <exynos_drm_fbdev.h>
#include <exynos_drm_crtc.h>
#include <exynos_drm_dsim.h>
#include <exynos_drm_format.h>
#include <exynos_drm_gem.h>
#include <exynos_drm_hibernation.h>

static const struct drm_framebuffer_funcs exynos_drm_fb_funcs = {
	.destroy	= drm_gem_fb_destroy,
	.create_handle	= drm_gem_fb_create_handle,
};

struct drm_framebuffer *
exynos_drm_framebuffer_init(struct drm_device *dev,
			    const struct drm_mode_fb_cmd2 *mode_cmd,
			    struct drm_gem_object **obj,
			    int count)
{
	struct drm_framebuffer *fb;
	int i;
	int ret;

	fb = kzalloc(sizeof(*fb), GFP_KERNEL);
	if (!fb)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < count; i++)
		fb->obj[i] = obj[i];

	drm_helper_mode_fill_fb_struct(dev, fb, mode_cmd);

	ret = drm_framebuffer_init(dev, fb, &exynos_drm_fb_funcs);
	if (ret < 0) {
		DRM_ERROR("failed to initialize framebuffer\n");
		kfree(fb);
		return ERR_PTR(ret);
	}

	return fb;
}

static size_t get_plane_size(const struct drm_mode_fb_cmd2 *mode_cmd, u32 idx,
		const struct drm_format_info *info)
{
	u32 height;
	size_t size = 0;
	bool is_10bpc;

	if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_YUV_8_2_SPLIT,
				mode_cmd->modifier[idx])) {
		if (idx == 0)
			size = Y_SIZE_8P2(mode_cmd->width, mode_cmd->height);
		else if (idx == 1)
			size = UV_SIZE_8P2(mode_cmd->width, mode_cmd->height);
	} else if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0),
				mode_cmd->modifier[idx])) {
		is_10bpc = IS_10BPC(dpu_find_fmt_info(mode_cmd->pixel_format));

		/*
		 * mapping size[0] : luminance PL/HD
		 * mapping size[1] : chrominance PL/HD
		 */
		if (idx == 0)
			size = Y_SIZE_SBWC(mode_cmd->width, mode_cmd->height,
					is_10bpc);
		else if (idx == 1)
			size = UV_SIZE_SBWC(mode_cmd->width, mode_cmd->height,
					is_10bpc);
	} else {
		height = (idx == 0) ? mode_cmd->height :
			DIV_ROUND_UP(mode_cmd->height, info->vsub);
		size = height * mode_cmd->pitches[idx];
	}

	return size;
}

static struct drm_framebuffer *
exynos_user_fb_create(struct drm_device *dev, struct drm_file *file_priv,
		      const struct drm_mode_fb_cmd2 *mode_cmd)
{
	const struct drm_format_info *info = drm_get_format_info(dev, mode_cmd);
	struct drm_gem_object *obj[MAX_FB_BUFFER] = { 0 };
	struct drm_framebuffer *fb;
	size_t size;
	int i;
	int ret;

	DRM_DEBUG("%s +\n", __func__);

	if (unlikely(info->num_planes > MAX_FB_BUFFER))
		return ERR_PTR(-EINVAL);

	for (i = 0; i < info->num_planes; i++) {
		if (mode_cmd->modifier[i] ==
				DRM_FORMAT_MOD_SAMSUNG_COLORMAP) {
			struct exynos_drm_gem *exynos_gem;

			exynos_gem = exynos_drm_gem_alloc(dev, 0,
						EXYNOS_DRM_GEM_FLAG_COLORMAP);
			if (IS_ERR(exynos_gem)) {
				DRM_ERROR("failed to create colormap gem\n");
				ret = PTR_ERR(exynos_gem);
				goto err;
			}

			exynos_gem->dma_addr = mode_cmd->handles[i];
			obj[i] = &exynos_gem->base;
			continue;
		}

		obj[i] = drm_gem_object_lookup(file_priv, mode_cmd->handles[i]);
		if (!obj[i]) {
			DRM_ERROR("failed to lookup gem object\n");
			ret = -ENOENT;
			goto err;
		}

		size = get_plane_size(mode_cmd, i, info);
		if (mode_cmd->offsets[i] + size > obj[i]->size) {
			DRM_ERROR("offsets[%d](%d) size(%d) obj[%d]->size(%d)\n",
					i, mode_cmd->offsets[i], size, i,
					obj[i]->size);
			i++;
			ret = -EINVAL;
			goto err;
		}
	}

	DRM_DEBUG("width(%d), height(%d), pitches(%d)\n", mode_cmd->width,
			mode_cmd->height, mode_cmd->pitches[0]);
	DRM_DEBUG("offset(%d), handle(%d), size(%lu)\n", mode_cmd->offsets[0],
			mode_cmd->handles[0], size);

	fb = exynos_drm_framebuffer_init(dev, mode_cmd, obj, i);
	if (IS_ERR(fb)) {
		ret = PTR_ERR(fb);
		goto err;
	}

	return fb;

err:
	while (i--)
		drm_gem_object_put_unlocked(obj[i]);

	return ERR_PTR(ret);

}

static const struct drm_format_info *
exynos_get_format_info(const struct drm_mode_fb_cmd2 *cmd)
{
	struct drm_format_name_buf n;
	const char *format_name;
	const struct drm_format_info *info = NULL;

	if (cmd->modifier[0] == DRM_FORMAT_MOD_SAMSUNG_COLORMAP) {
		info = drm_format_info(cmd->pixel_format);
		if (info->format == DRM_FORMAT_BGRA8888)
			return info;

		format_name = drm_get_format_name(info->format, &n);
		DRM_WARN("%s is not proper format for colormap\n", format_name);
	}

	return NULL;
}

dma_addr_t exynos_drm_fb_dma_addr(const struct drm_framebuffer *fb, int index)
{
	const struct exynos_drm_gem *exynos_gem;

	if (WARN_ON_ONCE(index >= MAX_FB_BUFFER) || !fb->obj[index])
		return 0;

	exynos_gem = to_exynos_gem(fb->obj[index]);

	DRM_DEBUG("%s:%d, dma_addr[%d] = 0x%llx (+%llx)\n", __func__, __LINE__,
			index, exynos_gem->dma_addr, fb->offsets[index]);

	return exynos_gem->dma_addr + fb->offsets[index];
}

void plane_state_to_win_config(struct decon_device *decon,
		struct exynos_drm_plane_state *state, int plane_idx)
{
	struct dpu_bts_win_config *win_config;
	const struct drm_framebuffer *fb = state->base.fb;
	int zpos;
	unsigned int simplified_rot;

	zpos = state->base.normalized_zpos;
	win_config = &decon->bts.win_config[zpos];

	win_config->src_x = state->base.src_x >> 16;
	win_config->src_y = state->base.src_y >> 16;
	win_config->src_w = state->base.src_w >> 16;
	win_config->src_h = state->base.src_h >> 16;

	win_config->dst_x = state->base.crtc_x;
	win_config->dst_y = state->base.crtc_y;
	win_config->dst_w = state->base.crtc_w;
	win_config->dst_h = state->base.crtc_h;

	if (has_all_bits(DRM_FORMAT_MOD_ARM_AFBC(0), fb->modifier) ||
			has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0),
				fb->modifier))
		win_config->is_comp = true;
	else
		win_config->is_comp = false;

	if (exynos_drm_fb_is_colormap(fb))
		win_config->state = DPU_WIN_STATE_COLOR;
	else
		win_config->state = DPU_WIN_STATE_BUFFER;

	win_config->format = fb->format->format;
	win_config->dpp_ch = plane_idx;

	win_config->comp_src = 0;
	if (has_all_bits(DRM_FORMAT_MOD_ARM_AFBC(0), fb->modifier))
		win_config->comp_src =
			(fb->modifier & AFBC_FORMAT_MOD_SOURCE_MASK);

	simplified_rot = drm_rotation_simplify(state->base.rotation,
			DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
			DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);
	win_config->is_rot = false;
	if (simplified_rot & DRM_MODE_ROTATE_90)
		win_config->is_rot = true;

	decon->dpp[plane_idx]->dbg_dma_addr = exynos_drm_fb_dma_addr(fb, 0);

	DRM_DEBUG("src[%d %d %d %d], dst[%d %d %d %d]\n",
			win_config->src_x, win_config->src_y,
			win_config->src_w, win_config->src_h,
			win_config->dst_x, win_config->dst_y,
			win_config->dst_w, win_config->dst_h);
	DRM_DEBUG("rot[%d] afbc[%d] format[%d] ch[%d] zpos[%d] comp_src[%lu]\n",
			win_config->is_rot, win_config->is_comp,
			win_config->format, win_config->dpp_ch, zpos,
			win_config->comp_src);
	DRM_DEBUG("alpha[%d] blend mode[%d]\n",
			state->base.alpha, state->base.pixel_blend_mode);
	DRM_DEBUG("simplified rot[0x%x]\n", simplified_rot);
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

#define TIMEOUT	msecs_to_jiffies(50)
void exynos_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	int i, j, ret;
	struct drm_device *dev = old_state->dev;
	struct drm_plane *plane;
	struct drm_plane_state *new_plane_state;
	struct exynos_drm_plane_state *new_exynos_state;
	struct exynos_drm_crtc *exynos_crtc;
	struct decon_device *decon;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	const struct dpu_bts_win_config *win_config;
	struct dpp_device *dpp;
	int max_planes;

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		DRM_DEBUG("[CRTC-%d] old en:%d active:%d change[%d %d %d]\n",
				drm_crtc_index(crtc),
				old_crtc_state->enable, old_crtc_state->active,
				old_crtc_state->planes_changed,
				old_crtc_state->mode_changed,
				old_crtc_state->active_changed);

		DRM_DEBUG("[CRTC-%d] new en:%d active:%d change[%d %d %d]\n",
				drm_crtc_index(crtc),
				new_crtc_state->enable, new_crtc_state->active,
				new_crtc_state->planes_changed,
				new_crtc_state->mode_changed,
				new_crtc_state->active_changed);

		exynos_crtc = container_of(crtc, struct exynos_drm_crtc, base);
		decon = exynos_crtc->ctx;

		DPU_EVENT_LOG(DPU_EVT_REQ_CRTC_INFO_OLD, decon->id,
				old_crtc_state);
		DPU_EVENT_LOG(DPU_EVT_REQ_CRTC_INFO_NEW, decon->id,
				new_crtc_state);

		/* acquire initial bandwidth when DECON is enabled. */
		if (!old_crtc_state->active && new_crtc_state->active) {
			display_mode_to_bts_info(&new_crtc_state->mode, decon);

			if (IS_ENABLED(CONFIG_EXYNOS_BTS))
				decon->bts.ops->acquire_bw(decon);
		}

		/* initialize BTS structure of each DECON */
		if (new_crtc_state->planes_changed && new_crtc_state->active) {
			max_planes =
				old_state->dev->mode_config.num_total_plane;
			for (j = 0; j < max_planes; ++j) {
				decon->bts.win_config[j].state =
					DPU_WIN_STATE_DISABLED;
				decon->dpp[i]->dbg_dma_addr = 0;
			}
		}
	}

	for_each_new_plane_in_state(old_state, plane, new_plane_state, i) {
		if (!new_plane_state->crtc)
			continue;

		new_exynos_state = to_exynos_plane_state(new_plane_state);
		exynos_crtc = container_of(new_plane_state->crtc,
				struct exynos_drm_crtc, base);
		decon = exynos_crtc->ctx;

		plane_state_to_win_config(decon, new_exynos_state, i);
	}

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		exynos_crtc = container_of(crtc, struct exynos_drm_crtc, base);
		decon = exynos_crtc->ctx;

		if (new_crtc_state->active || new_crtc_state->active_changed)
			hibernation_block_exit(decon->hibernation);

		if (new_crtc_state->planes_changed && new_crtc_state->active) {
			DPU_EVENT_LOG_ATOMIC_COMMIT(decon->id);
			if (IS_ENABLED(CONFIG_EXYNOS_BTS)) {
				decon->bts.ops->calc_bw(decon);
				decon->bts.ops->update_bw(decon, false);
			}
		}

		if (old_crtc_state->active && !new_crtc_state->active) {
			/*
			 * If hw related to crtc is processing, it also should
			 * be delayed. If not, the hw does not shut down
			 * normally.
			 */
			ret = wait_event_interruptible_timeout(
					decon->framedone_wait,
					decon->busy == false, TIMEOUT);
			if (ret == 0)
				pr_err("decon%d framedone timeout\n",
						decon->id);
		}
	}

	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	drm_atomic_helper_commit_modeset_enables(dev, old_state);

	drm_atomic_helper_commit_planes(dev, old_state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	/*
	 * hw is flushed at this point, signal flip done for fake commit to
	 * unblock nonblocking atomic commits once vblank occurs
	 */
	if (old_state->fake_commit)
		complete_all(&old_state->fake_commit->flip_done);

	drm_atomic_helper_fake_vblank(old_state);

	drm_atomic_helper_wait_for_vblanks(dev, old_state);

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		exynos_crtc = container_of(crtc, struct exynos_drm_crtc, base);
		decon = exynos_crtc->ctx;

		if (new_crtc_state->active) {
			struct decon_mode *mode = &decon->config.mode;

			if (!wait_for_completion_timeout(
					&decon->framestart_done, TIMEOUT)) {
				DPU_EVENT_LOG(DPU_EVT_FRAMESTART_TIMEOUT,
						decon->id, NULL);
				pr_warn("decon%d framestart timeout\n",
						decon->id);
			}

			if (mode->op_mode == DECON_COMMAND_MODE) {
				DPU_EVENT_LOG(DPU_EVT_DECON_TRIG_MASK,
						decon->id, NULL);
				decon_reg_set_trigger(decon->id, mode,
						DECON_TRIG_MASK);
			}
		}

		if (new_crtc_state->planes_changed && new_crtc_state->active) {
			/* plane order == dpp channel order */
			for (j = 0; j < MAX_PLANE; ++j) {
				dpp = decon->dpp[j];
				if (dpp->win_id >= MAX_PLANE)
					continue;

				win_config =
					&decon->bts.win_config[dpp->win_id];
				if (win_config->state == DPU_WIN_STATE_BUFFER &&
						win_config->is_comp)
					dpp->comp_src = win_config->comp_src;
			}

			if (IS_ENABLED(CONFIG_EXYNOS_BTS))
				decon->bts.ops->update_bw(decon, true);
			DPU_EVENT_LOG(DPU_EVT_DECON_RSC_OCCUPANCY, 0, NULL);
		}

		if (new_crtc_state->active || new_crtc_state->active_changed)
			hibernation_unblock(decon->hibernation);

		if ((old_crtc_state->active && !new_crtc_state->active) &&
				IS_ENABLED(CONFIG_EXYNOS_BTS))
			decon->bts.ops->release_bw(decon);
	}

	drm_atomic_helper_commit_hw_done(old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);
}

static struct drm_mode_config_helper_funcs exynos_drm_mode_config_helpers = {
	.atomic_commit_tail = exynos_atomic_commit_tail,
};

static const struct drm_mode_config_funcs exynos_drm_mode_config_funcs = {
	.fb_create = exynos_user_fb_create,
	.get_format_info = exynos_get_format_info,
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
