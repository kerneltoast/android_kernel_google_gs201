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

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <uapi/drm/exynos_drm.h>
#include <uapi/linux/videodev2_exynos_media.h>
#include <linux/dma-buf.h>
#include <linux/pm_runtime.h>
#include <linux/of_reserved_mem.h>

#include <trace/dpu_trace.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_format.h"
#include "exynos_drm_gem.h"
#include "exynos_drm_hibernation.h"
#include "exynos_drm_recovery.h"

extern const struct dpp_restriction dpp_drv_data;

static const struct drm_framebuffer_funcs exynos_drm_fb_funcs = {
	.destroy	= drm_gem_fb_destroy,
	.create_handle	= drm_gem_fb_create_handle,
};

static struct drm_framebuffer *
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
		if (!size || mode_cmd->offsets[i] + size > obj[i]->size) {
			DRM_ERROR("offsets[%d](%d) size(%zd) obj[%d]->size(%zd)\n",
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
		drm_gem_object_put(obj[i]);

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

	DRM_DEBUG("%s:%d, dma_addr[%d] = 0x%llx (+0x%x)\n", __func__, __LINE__,
			index, exynos_gem->dma_addr, fb->offsets[index]);

	return exynos_gem->dma_addr + fb->offsets[index];
}

void *exynos_drm_fb_to_vaddr(const struct drm_framebuffer *fb)
{
	struct exynos_drm_gem *exynos_gem;

	if (WARN_ON_ONCE(!fb->obj[0]))
		return 0;

	exynos_gem = to_exynos_gem(fb->obj[0]);

	return exynos_drm_gem_get_vaddr(exynos_gem);
}

static void plane_state_to_win_config(struct dpu_bts_win_config *win_config,
				      const struct drm_plane_state *plane_state)
{
	const struct drm_framebuffer *fb = plane_state->fb;
	unsigned int simplified_rot;

	win_config->src_x = plane_state->src.x1 >> 16;
	win_config->src_y = plane_state->src.y1 >> 16;
	win_config->src_w = drm_rect_width(&plane_state->src) >> 16;
	win_config->src_h = drm_rect_height(&plane_state->src) >> 16;

	win_config->dst_x = plane_state->dst.x1;
	win_config->dst_y = plane_state->dst.y1;
	win_config->dst_w = drm_rect_width(&plane_state->dst);
	win_config->dst_h = drm_rect_height(&plane_state->dst);

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
	win_config->dpp_ch = plane_state->plane->index;

	win_config->comp_src = 0;
	if (has_all_bits(DRM_FORMAT_MOD_ARM_AFBC(0), fb->modifier))
		win_config->comp_src =
			(fb->modifier & AFBC_FORMAT_MOD_SOURCE_MASK);

	simplified_rot = drm_rotation_simplify(plane_state->rotation,
			DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
			DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);
	win_config->is_rot = false;
	if (simplified_rot & DRM_MODE_ROTATE_90)
		win_config->is_rot = true;

	win_config->is_secure = (fb->modifier & DRM_FORMAT_MOD_PROTECTION) != 0;

	DRM_DEBUG("src[%d %d %d %d], dst[%d %d %d %d]\n",
			win_config->src_x, win_config->src_y,
			win_config->src_w, win_config->src_h,
			win_config->dst_x, win_config->dst_y,
			win_config->dst_w, win_config->dst_h);
	DRM_DEBUG("rot[%d] afbc[%d] format[%d] ch[%d] zpos[%d] comp_src[%llu]\n",
			win_config->is_rot, win_config->is_comp,
			win_config->format, win_config->dpp_ch,
			plane_state->normalized_zpos,
			win_config->comp_src);
	DRM_DEBUG("alpha[%d] blend mode[%d]\n",
			plane_state->alpha, plane_state->pixel_blend_mode);
	DRM_DEBUG("simplified rot[0x%x]\n", simplified_rot);
}

static void conn_state_to_win_config(struct dpu_bts_win_config *win_config,
				const struct drm_connector_state *conn_state)
{
	const struct writeback_device *wb = conn_to_wb_dev(conn_state->connector);
	const struct drm_framebuffer *fb = conn_state->writeback_job->fb;

	win_config->src_x = 0;
	win_config->src_y = 0;
	win_config->src_w = fb->width;
	win_config->src_h = fb->height;
	win_config->dst_x = 0;
	win_config->dst_y = 0;
	win_config->dst_w = fb->width;
	win_config->dst_h = fb->height;

	win_config->is_comp = false;
	win_config->state = DPU_WIN_STATE_BUFFER;
	win_config->format = fb->format->format;
	win_config->dpp_ch = wb->id;
	win_config->comp_src = 0;
	win_config->is_rot = false;
	win_config->is_secure = (fb->modifier & DRM_FORMAT_MOD_PROTECTION) != 0;

	DRM_DEBUG("src[%d %d %d %d], dst[%d %d %d %d]\n",
			win_config->src_x, win_config->src_y,
			win_config->src_w, win_config->src_h,
			win_config->dst_x, win_config->dst_y,
			win_config->dst_w, win_config->dst_h);
	DRM_DEBUG("rot[%d] afbc[%d] format[%d] ch[%d] comp_src[%llu]\n",
			win_config->is_rot, win_config->is_comp,
			win_config->format, win_config->dpp_ch,
			win_config->comp_src);
}

static void exynos_atomic_bts_pre_update(struct drm_device *dev,
					 struct drm_atomic_state *old_state)
{
	struct decon_device *decon;
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state;
	struct drm_plane *plane;
	const struct drm_plane_state *old_plane_state, *new_plane_state;
	struct dpu_bts_win_config *win_config;
	struct drm_connector *conn;
	const struct drm_connector_state *old_conn_state, *new_conn_state;
	int i;
	struct dpp_device *dpp;
	struct exynos_drm_crtc *exynos_crtc;

	if (!IS_ENABLED(CONFIG_EXYNOS_BTS))
		return;

	for_each_oldnew_plane_in_state(old_state, plane, old_plane_state,
				       new_plane_state, i) {
		dpp = plane_to_dpp(to_exynos_plane(plane));
		if (test_bit(DPP_ATTR_RCD, &dpp->attr)) {
			if (new_plane_state->crtc) {
				decon = crtc_to_decon(new_plane_state->crtc);
				win_config = &decon->bts.rcd_win_config.win;
				plane_state_to_win_config(win_config, new_plane_state);

				decon->bts.rcd_win_config.dma_addr =
					exynos_drm_fb_dma_addr(new_plane_state->fb, 0);
			}
			continue;
		}
		if (new_plane_state->crtc) {
			const int zpos = new_plane_state->normalized_zpos;

			decon = crtc_to_decon(new_plane_state->crtc);
			win_config = &decon->bts.win_config[zpos];

			plane_state_to_win_config(win_config, new_plane_state);

			decon->dpp[i]->dbg_dma_addr =
				exynos_drm_fb_dma_addr(new_plane_state->fb, 0);
		}

		if ((old_plane_state->crtc != new_plane_state->crtc) &&
		    old_plane_state->crtc) {
			decon = crtc_to_decon(old_plane_state->crtc);

			decon->dpp[i]->dbg_dma_addr = 0;
		}
	}

	for_each_oldnew_connector_in_state(old_state, conn, old_conn_state,
					new_conn_state, i) {
		bool old_job, new_job;

		if (conn->connector_type != DRM_MODE_CONNECTOR_WRITEBACK)
			continue;

		conn_to_wb_dev(conn);

		old_job = wb_check_job(old_conn_state);
		new_job = wb_check_job(new_conn_state);

		if (!old_job && new_job) {
			decon = crtc_to_decon(new_conn_state->crtc);
			win_config = &decon->bts.wb_config;
			conn_state_to_win_config(win_config, new_conn_state);
		} else if (old_job && !new_job) {
			decon = crtc_to_decon(old_conn_state->crtc);
			win_config = &decon->bts.wb_config;
			win_config->state = DPU_WIN_STATE_DISABLED;
		}
	}

	for_each_new_crtc_in_state(old_state, crtc, new_crtc_state, i) {
		decon = crtc_to_decon(crtc);
		exynos_crtc = to_exynos_crtc(crtc);

		if (!new_crtc_state->active)
			continue;

		if (new_crtc_state->planes_changed) {
			const size_t num_planes =
				hweight32(new_crtc_state->plane_mask &
						~exynos_crtc->rcd_plane_mask);
			int j;

			for (j = num_planes; j < MAX_WIN_PER_DECON; j++) {
				win_config = &decon->bts.win_config[j];
				win_config->state = DPU_WIN_STATE_DISABLED;
			}

			if ((new_crtc_state->plane_mask & exynos_crtc->rcd_plane_mask) == 0) {
				win_config = &decon->bts.rcd_win_config.win;
				win_config->state = DPU_WIN_STATE_DISABLED;
			}
		}

		DPU_EVENT_LOG_ATOMIC_COMMIT(decon->id);
		decon_mode_bts_pre_update(decon, new_crtc_state, old_state);
	}
}

static void exynos_atomic_bts_post_update(struct drm_device *dev,
					  struct drm_atomic_state *old_state)
{
	struct decon_device *decon;
	struct drm_crtc *crtc;
	const struct drm_crtc_state *new_crtc_state;
	struct dpp_device *dpp;
	const struct dpu_bts_win_config *win_config;
	int i, j;

	if (!IS_ENABLED(CONFIG_EXYNOS_BTS))
		return;

	for_each_new_crtc_in_state(old_state, crtc, new_crtc_state, i) {
		decon = crtc_to_decon(crtc);

		if (new_crtc_state->active) {

			/*
			 * keeping a copy of comp src in dpp after it has been
			 * applied in hardware for debugging purposes.
			 * plane order == dpp channel order
			 */
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

			decon->bts.ops->update_bw(decon, true);
			DPU_EVENT_LOG(DPU_EVT_DECON_RSC_OCCUPANCY, decon->id, NULL);
		}

		if (!new_crtc_state->active && new_crtc_state->active_changed)
			decon->bts.ops->release_bw(decon);
	}
}

/*
 * rmem_device_init is called in of_reserved_mem_device_init_by_idx function
 * when reserved memory is required.
 */
static int rmem_device_init(struct reserved_mem *rmem, struct device *dev)
{
	struct decon_device *decon = dev_get_drvdata(dev);

	decon->fb_handover.phys_addr = rmem->base;
	decon->fb_handover.phys_size = rmem->size;
	pr_debug("%s base(%pa) size(%pa)\n", __func__, &decon->fb_handover.phys_addr,
			&decon->fb_handover.phys_size);

	return 0;
}

/*
 * rmem_device_release is called in of_reserved_mem_device_release function
 * when reserved memory is no longer required.
 */
static void rmem_device_release(struct reserved_mem *rmem, struct device *dev)
{
	struct page *first = phys_to_page(PAGE_ALIGN(rmem->base));
	struct page *last = phys_to_page((rmem->base + rmem->size) & PAGE_MASK);
	struct page *page;

	for (page = first; page != last; page++) {
		__ClearPageReserved(page);
		set_page_count(page, 1);
		__free_pages(page, 0);
		adjust_managed_page_count(page, 1);
	}
}

static const struct reserved_mem_ops rmem_ops = {
	.device_init    = rmem_device_init,
	.device_release = rmem_device_release,
};

void exynos_rmem_register(struct decon_device *decon)
{
	struct device_node *np, *rmem_np;
	struct reserved_mem *rmem;
	struct device *dev = decon->dev;

	np = dev->of_node;

	rmem_np = of_parse_phandle(np, "memory-region", 0);
	if (!rmem_np) {
		pr_debug("failed to get reserve memory phandle\n");
		return;
	}

	rmem = of_reserved_mem_lookup(rmem_np);
	if (!rmem) {
		pr_err("failed to reserve memory lookup\n");
		return;
	}

	of_node_put(rmem_np);
	rmem->ops = &rmem_ops;
	decon->fb_handover.rmem = rmem;
	of_reserved_mem_device_init_by_idx(dev, np, 0);
}

static void exynos_rmem_free(struct decon_device *decon)
{
	of_reserved_mem_device_release(decon->dev);

	decon->fb_handover.rmem = NULL;
	decon->fb_handover.phys_size = 0;
}


static void exynos_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	int i;
	struct drm_device *dev = old_state->dev;
	struct decon_device *decon;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *old_conn_state;
	struct drm_connector_state *new_conn_state;
	unsigned int disabling_crtc_mask = 0;

	DPU_ATRACE_BEGIN("exynos_atomic_commit_tail");

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		const struct drm_crtc_helper_funcs *funcs;

		decon = crtc_to_decon(crtc);

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

		DPU_EVENT_LOG(DPU_EVT_REQ_CRTC_INFO_OLD, decon->id,
				old_crtc_state);
		DPU_EVENT_LOG(DPU_EVT_REQ_CRTC_INFO_NEW, decon->id,
				new_crtc_state);

		if (drm_atomic_crtc_effectively_active(old_crtc_state) && !new_crtc_state->active) {
			/* keep runtime vote while disabling is taking place */
			pm_runtime_get_sync(decon->dev);
			disabling_crtc_mask |= drm_crtc_mask(crtc);
		}

		if (old_crtc_state->active && drm_atomic_crtc_needs_modeset(new_crtc_state)) {
			DPU_ATRACE_BEGIN("crtc_disable");

			funcs = crtc->helper_private;
			if (new_crtc_state->enable && funcs->prepare)
				funcs->prepare(crtc);
			else if (funcs->atomic_disable)
				funcs->atomic_disable(crtc, old_crtc_state);
			else if (funcs->disable)
				funcs->disable(crtc);
			else if (funcs->dpms)
				funcs->dpms(crtc, DRM_MODE_DPMS_OFF);

			DPU_ATRACE_END("crtc_disable");
		}
		/*  TODO(b/237120310): needs cleanup after we identify mode_changed handling */
		if (old_crtc_state->self_refresh_active && new_crtc_state->mode_changed)
			old_crtc_state->active = true;
	}

	DPU_ATRACE_BEGIN("modeset");
	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	exynos_atomic_bts_pre_update(dev, old_state);

	drm_atomic_helper_commit_modeset_enables(dev, old_state);
	DPU_ATRACE_END("modeset");

	DPU_ATRACE_BEGIN("connector_pre_commit");
	for_each_oldnew_connector_in_state(old_state, connector,
				 old_conn_state, new_conn_state, i) {
		if (!new_conn_state->crtc)
			continue;

		new_crtc_state = drm_atomic_get_new_crtc_state(old_state, new_conn_state->crtc);
		if (!new_crtc_state->active)
			continue;

		if (is_exynos_drm_connector(connector)) {
			struct exynos_drm_connector *exynos_connector =
				to_exynos_connector(connector);
			const struct exynos_drm_connector_helper_funcs *funcs =
				exynos_connector->helper_private;
			if (!funcs->atomic_pre_commit)
				continue;

			funcs->atomic_pre_commit(exynos_connector,
					to_exynos_connector_state(old_conn_state),
					to_exynos_connector_state(new_conn_state));
		}
	}
	DPU_ATRACE_END("connector_pre_commit");

	DPU_ATRACE_BEGIN("commit_planes");
	drm_atomic_helper_commit_planes(dev, old_state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);
	DPU_ATRACE_END("commit_planes");

	/*
	 * hw is flushed at this point, signal flip done for fake commit to
	 * unblock nonblocking atomic commits once vblank occurs
	 */
	if (old_state->fake_commit)
		complete_all(&old_state->fake_commit->flip_done);

	drm_atomic_helper_fake_vblank(old_state);

	DPU_ATRACE_BEGIN("connector_commit");
	for_each_oldnew_connector_in_state(old_state, connector,
				 old_conn_state, new_conn_state, i) {
		if (!new_conn_state->crtc)
			continue;

		new_crtc_state = drm_atomic_get_new_crtc_state(old_state, new_conn_state->crtc);
		if (!new_crtc_state->active)
			continue;

		if (is_exynos_drm_connector(connector)) {
			struct exynos_drm_connector *exynos_connector =
				to_exynos_connector(connector);
			const struct exynos_drm_connector_helper_funcs *funcs =
				exynos_connector->helper_private;

			funcs->atomic_commit(exynos_connector,
					to_exynos_connector_state(old_conn_state),
					to_exynos_connector_state(new_conn_state));
		}
	}
	DPU_ATRACE_END("connector_commit");
	DPU_ATRACE_BEGIN("wait_for_crtc_flip");
	exynos_crtc_wait_for_flip_done(old_state);
	DPU_ATRACE_END("wait_for_crtc_flip");

	DPU_ATRACE_BEGIN("wait_for_flip_done");
	drm_atomic_helper_wait_for_flip_done(dev, old_state);
	DPU_ATRACE_END("wait_for_flip_done");

	exynos_atomic_bts_post_update(dev, old_state);

	for_each_new_crtc_in_state(old_state, crtc, new_crtc_state, i) {
		decon = crtc_to_decon(crtc);
		if (disabling_crtc_mask & drm_crtc_mask(crtc))
			pm_runtime_put_sync(decon->dev);
		if (decon->fb_handover.rmem) {
			struct exynos_drm_crtc_state *exynos_crtc_state =
				to_exynos_crtc_state(new_crtc_state);

			if (!exynos_crtc_state->skip_update)
				exynos_rmem_free(decon);
		}
	}

	drm_atomic_helper_commit_hw_done(old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);

	DPU_ATRACE_END("exynos_atomic_commit_tail");
}

static struct drm_mode_config_helper_funcs exynos_drm_mode_config_helpers = {
	.atomic_commit_tail = exynos_atomic_commit_tail,
};

static const struct drm_mode_config_funcs exynos_drm_mode_config_funcs = {
	.fb_create = exynos_user_fb_create,
	.get_format_info = exynos_get_format_info,
	.atomic_check = exynos_atomic_check,
	.atomic_commit = exynos_atomic_commit,
};

void exynos_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * set min/max width and height respecting dpp_drv_data config.
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = dpp_drv_data.dst_f_w.max;
	dev->mode_config.max_height = dpp_drv_data.dst_f_h.max;
	dev->mode_config.min_width = dpp_drv_data.dst_f_w.min;
	dev->mode_config.min_height = dpp_drv_data.dst_f_h.min;

	dev->mode_config.funcs = &exynos_drm_mode_config_funcs;
	dev->mode_config.helper_private = &exynos_drm_mode_config_helpers;

	dev->mode_config.allow_fb_modifiers = true;
}
