// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#define pr_fmt(fmt)  "[PARTIAL]: %s: " fmt, __func__

#include <linux/device.h>
#include <linux/of.h>
#include <video/mipi_display.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fourcc_gs101.h>
#include "exynos_drm_decon.h"
#include "exynos_drm_format.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_connector.h"
#include "cal_common/dsim_cal.h"

#define pr_region(str, r)	\
	pr_debug("%s["DRM_RECT_FMT"]\n", (str), DRM_RECT_ARG(r))

static int exynos_partial_init(struct exynos_partial *partial,
		const struct exynos_display_partial *partial_mode,
		const struct drm_display_mode *mode)
{
	partial->min_w = partial_mode->min_width;
	partial->min_h = partial_mode->min_height;

	if ((partial->min_w < MIN_WIN_BLOCK_WIDTH) ||
			(partial->min_h < MIN_WIN_BLOCK_HEIGHT)) {
		pr_err("invalid min size(%dx%d) of partial update\n",
				partial->min_w, partial->min_h);
		return -EINVAL;
	}

	if ((mode->hdisplay % partial->min_w) ||
			(mode->vdisplay % partial->min_h)) {
		pr_err("cannot support partial update(%dx%d, %dx%d)\n",
				mode->hdisplay, mode->vdisplay,
				partial->min_w, partial->min_h);
		return -EINVAL;
	}

	return 0;
}

void exynos_partial_set_full(const struct drm_display_mode *mode,
				struct drm_rect *partial_r)
{
	partial_r->x1 = 0;
	partial_r->y1 = 0;
	partial_r->x2 = mode->hdisplay;
	partial_r->y2 = mode->vdisplay;
}

static int exynos_partial_adjust_region(struct exynos_partial *partial,
			const struct drm_display_mode *mode,
			const struct drm_rect *req, struct drm_rect *r)
{
	pr_region("requested update region", req);

	if (!req->x1 && !req->y1 && !req->x2 && !req->y2) {
		pr_region("invalid partial update region", req);
		return -EINVAL;
	}

	if ((req->x2 > mode->hdisplay) || (req->y2 > mode->vdisplay)) {
		pr_debug("changed full: requested region is bigger\n");
		return -EINVAL;
	}

	/* adjusted update region */
	r->y1 = rounddown(req->y1, partial->min_h);
	r->y2 = roundup(req->y2, partial->min_h);
	/*
	 * TODO: Currently, partial width is fixed by LCD width. This will be
	 * changed to be configurable in the future.
	 */
	r->x1 = 0;
	r->x2 = mode->hdisplay;

	pr_region("adjusted update region", r);

	return 0;
}

static void exynos_plane_print_info(const struct drm_plane_state *state)
{
	const struct drm_plane *plane = state->plane;
	const struct drm_rect src = drm_plane_state_src(state);
	const struct drm_rect dst = drm_plane_state_dest(state);
	const struct drm_rect *clipped_src = &state->src;
	const struct drm_rect *clipped_dst = &state->dst;

	pr_debug("plane%d/win%d src["DRM_RECT_FP_FMT"] dst["DRM_RECT_FMT"]\n",
			drm_plane_index(plane), state->normalized_zpos,
			DRM_RECT_FP_ARG(&src), DRM_RECT_ARG(&dst));

	pr_debug("\t\tclipped src["DRM_RECT_FP_FMT"] dst["DRM_RECT_FMT"]\n",
			DRM_RECT_FP_ARG(clipped_src), DRM_RECT_ARG(clipped_dst));
}

static inline bool
exynos_plane_state_rotation(const struct drm_plane_state *state)
{
	unsigned int simplified_rot;

	simplified_rot = drm_rotation_simplify(state->rotation,
			DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
			DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);

	return (simplified_rot & DRM_MODE_ROTATE_90) != 0;
}

static inline bool
exynos_plane_state_scaling(const struct drm_plane_state *state)
{
	return (state->src_w >> 16 != state->crtc_w) ||
		(state->src_h >> 16 != state->crtc_h);
}

static bool is_partial_supported(const struct drm_plane_state *state,
		const struct drm_rect *crtc_r, const struct drm_rect *partial_r,
		const struct dpp_restriction *res)
{
	const struct dpu_fmt *fmt_info;
	unsigned int adj_src_x = 0, adj_src_y = 0;
	u32 format;
	int sz_align = 1;

	if (exynos_plane_state_rotation(state)) {
		pr_debug("rotation is detected. partial->full\n");
		goto not_supported;
	}

	if (exynos_plane_state_scaling(state)) {
		pr_debug("scaling is detected. partial->full\n");
		goto not_supported;
	}

	format = state->fb->format->format;
	fmt_info = dpu_find_fmt_info(format);
	if (IS_YUV(fmt_info)) {
		adj_src_x = state->src_x >> 16;
		adj_src_y = state->src_y >> 16;
		sz_align = 2;

		if (partial_r->x1 > state->crtc_x)
			adj_src_x += partial_r->x1 - state->crtc_x;

		if (partial_r->y1 > state->crtc_y)
			adj_src_y += partial_r->y1 - state->crtc_y;

		/* YUV format must be aligned to 2 */
		if (!IS_ALIGNED(adj_src_x, sz_align) ||
				!IS_ALIGNED(adj_src_y, sz_align)) {
			pr_debug("align limitation. src_x/y[%d/%d] align[%d]\n",
					adj_src_x, adj_src_y, sz_align);
			goto not_supported;
		}
	}

	if ((drm_rect_width(crtc_r) < res->src_f_w.min * sz_align) ||
			(drm_rect_height(crtc_r) < res->src_f_h.min * sz_align)) {
		pr_debug("min size limitation. width[%d] height[%d]\n",
				drm_rect_width(crtc_r), drm_rect_height(crtc_r));
		goto not_supported;
	}

	return true;

not_supported:
	exynos_plane_print_info(state);
	return false;
}

#define to_dpp_device(x)	container_of(x, struct dpp_device, plane)
static bool exynos_partial_check(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state)
{
	struct drm_crtc_state *crtc_state = &exynos_crtc_state->base;
	struct drm_plane *plane;
	const struct drm_plane_state *plane_state;
	const struct drm_rect *partial_r = &exynos_crtc_state->partial_region;
	struct drm_rect r;
	const struct dpp_device *dpp;
	const struct dpp_restriction *res;

	drm_for_each_plane_mask(plane, crtc_state->state->dev, crtc_state->plane_mask) {
		plane_state = drm_atomic_get_plane_state(crtc_state->state, plane);
		if (IS_ERR(plane_state))
			return false;

		r = drm_plane_state_dest(plane_state);

		if (!drm_rect_intersect(&r, partial_r))
			continue;

		dpp = to_dpp_device(to_exynos_plane(plane));
		res = &dpp->restriction;
		pr_debug("checking plane%d ...\n", drm_plane_index(plane));

		if (!is_partial_supported(plane_state, &r, partial_r, res))
			return false;
	}

	return true;
}

static int exynos_partial_send_command(struct exynos_partial *partial,
					const struct drm_rect *partial_r)
{
	struct decon_device *decon = partial->decon;
	struct dsim_device *dsim;
	int ret;

	pr_debug("partial command: [%d %d %d %d]\n",
			partial_r->x1, partial_r->y1,
			drm_rect_width(partial_r), drm_rect_height(partial_r));

	if (!decon)
		return -ENODEV;

	dsim = decon_get_dsim(decon);
	if (!dsim)
		return -ENODEV;

	ret = mipi_dsi_dcs_set_column_address(dsim->dsi_device, partial_r->x1,
			partial_r->x2 - 1);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_set_page_address(dsim->dsi_device, partial_r->y1,
			partial_r->y2 - 1);
	if (ret)
		return ret;

	return ret;
}

static void exynos_partial_find_included_slice(struct exynos_dsc *dsc,
				const struct drm_rect *rect, bool in_slice[])
{
	int slice_left, slice_right;
	int i;

	for (i = 0; i < dsc->slice_count; ++i) {
		slice_left = dsc->slice_width * i;
		slice_right = slice_left + dsc->slice_width;
		in_slice[i] = (slice_left >= rect->x1) && (slice_right <= rect->x2);

		pr_debug("slice left(%d) right(%d)\n", slice_left, slice_right);
		pr_debug("slice[%d] is %s\n", i, in_slice[i] ? "in" : "out");
	}
}

#define MAX_DSC_SLICE_CNT	4
static void exynos_partial_set_size(struct exynos_partial *partial,
					const struct drm_rect *partial_r)
{
	struct decon_device *decon = partial->decon;
	struct dsim_device *dsim;
	struct dsim_reg_config dsim_config;
	bool in_slice[MAX_DSC_SLICE_CNT];
	bool dsc_en;
	u32 partial_w, partial_h;

	if (!decon)
		return;

	dsim = decon_get_dsim(decon);
	if (!dsim)
		return;

	dsc_en = dsim->config.dsc.enabled;
	partial_w = drm_rect_width(partial_r);
	partial_h = drm_rect_height(partial_r);

	memcpy(&dsim_config, &dsim->config, sizeof(struct dsim_reg_config));
	dsim_config.p_timing.hactive = partial_w;
	dsim_config.p_timing.vactive = partial_h;
	dsim_config.p_timing.hfp +=
		((dsim->config.p_timing.hactive - partial_w) / (dsc_en ? 3 : 1));
	dsim_config.p_timing.vfp += (dsim->config.p_timing.vactive - partial_h);
	dsim_reg_set_partial_update(dsim->id, &dsim_config);

	exynos_partial_find_included_slice(&decon->config.dsc, partial_r,
				in_slice);
	decon_reg_set_partial_update(decon->id, &decon->config, in_slice,
			partial_w, partial_h);

	if (decon->dqe) {
		dqe_reg_set_size(decon->id, partial_w, partial_h);
		decon_reg_update_req_dqe(decon->id);
	}

	pr_debug("partial[%dx%d] vporch[%d %d %d] hporch[%d %d %d]\n",
			partial_w, partial_h,
			dsim_config.p_timing.vbp, dsim_config.p_timing.vfp,
			dsim_config.p_timing.vsa, dsim_config.p_timing.hbp,
			dsim_config.p_timing.hfp, dsim_config.p_timing.hsa);
}

static const struct exynos_partial_funcs partial_funcs = {
	.init			 = exynos_partial_init,
	.check			 = exynos_partial_check,
	.adjust_partial_region	 = exynos_partial_adjust_region,
	.send_partial_command	 = exynos_partial_send_command,
	.set_partial_size	 = exynos_partial_set_size,
};

struct exynos_partial *exynos_partial_initialize(struct decon_device *decon,
			const struct exynos_display_partial *partial_mode,
			const struct drm_display_mode *mode)
{
	int ret;
	struct exynos_partial *partial = NULL;
	struct device *dev = decon->dev;

	if (!partial_mode->enabled) {
		pr_debug("This panel doesn't support partial update feature\n");
		return NULL;
	}

	if (!decon->partial) {
		partial = devm_kzalloc(dev, sizeof(struct exynos_partial),
				GFP_KERNEL);
		if (!partial)
			return NULL;

		partial->decon = decon;
		partial->funcs = &partial_funcs;
	} else {
		partial = decon->partial;
	}

	ret = partial->funcs->init(partial, partial_mode, mode);
	if (ret) {
		pr_err("failed to initialize partial update\n");
		kfree(partial);
		return NULL;
	}

	pr_debug("partial update is initialized: min rect(%dx%d)\n",
			partial->min_w, partial->min_h);
	DPU_EVENT_LOG(DPU_EVT_PARTIAL_INIT, partial->decon->id, partial);

	return partial;
}

static void
exynos_partial_save_log(struct dpu_log_partial *plog, const struct drm_rect *prev,
				struct drm_rect *req, struct drm_rect *adj,
				bool reconfigure)
{
	memcpy(&plog->prev, prev, sizeof(struct drm_rect));
	memcpy(&plog->req, req, sizeof(struct drm_rect));
	memcpy(&plog->adj, adj, sizeof(struct drm_rect));
	plog->reconfigure = reconfigure;
}

static bool
exynos_partial_is_full(const struct drm_display_mode *mode, const struct drm_rect *rect)
{
	struct drm_rect full;

	exynos_partial_set_full(mode, &full);

	return drm_rect_equals(&full, rect);
}

void exynos_partial_prepare(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *old_exynos_crtc_state,
			struct exynos_drm_crtc_state *new_exynos_crtc_state)
{
	struct drm_crtc_state *crtc_state = &new_exynos_crtc_state->base;
	struct drm_rect *partial_r = &new_exynos_crtc_state->partial_region;
	const struct drm_rect *old_partial_r = &old_exynos_crtc_state->partial_region;
	struct decon_device *decon = partial->decon;
	struct dpu_log_partial plog;
	struct drm_clip_rect *req_region;
	struct drm_rect req;
	int ret = -ENOENT;
	bool region_changed = false;

	pr_debug("plane mask[0x%x]\n", crtc_state->plane_mask);

	new_exynos_crtc_state->needs_reconfigure = false;

	if (drm_atomic_crtc_needs_modeset(crtc_state)) {
		exynos_partial_set_full(&crtc_state->mode, partial_r);
		return;
	}

	if (!crtc_state->plane_mask)
		return;

	if (old_exynos_crtc_state->partial != new_exynos_crtc_state->partial) {
		if (new_exynos_crtc_state->partial) {
			req_region = new_exynos_crtc_state->partial->data;
			req.x1 = req_region->x1;
			req.y1 = req_region->y1;
			req.x2 = req_region->x2;
			req.y2 = req_region->y2;

			/* find adjusted update region on LCD */
			ret = partial->funcs->adjust_partial_region(partial,
					&crtc_state->mode, &req, partial_r);
		}

		if (ret)
			exynos_partial_set_full(&crtc_state->mode, partial_r);

		region_changed = !drm_rect_equals(partial_r, old_partial_r);
	}

	if (!region_changed) {
		if (!crtc_state->planes_changed) {
			new_exynos_crtc_state->needs_reconfigure =
				!exynos_partial_is_full(&crtc_state->mode, partial_r);
			return;
		} else if (exynos_partial_is_full(&crtc_state->mode, partial_r)) {
			return;
		}
	} else {
		/* if region changed, DQE needs to be updated */
		crtc_state->color_mgmt_changed = true;
	}

	/* check DPP hw limit if violated, update region is changed to full */
	if (!partial->funcs->check(partial, new_exynos_crtc_state))
		exynos_partial_set_full(&crtc_state->mode,
				&new_exynos_crtc_state->partial_region);

	pr_region("final update region", partial_r);

	/*
	 * If partial update region is requested, source and destination
	 * coordinates are needed to change if overlapped with update region.
	 */
	new_exynos_crtc_state->needs_reconfigure =
			!exynos_partial_is_full(&crtc_state->mode, partial_r);

	pr_debug("reconfigure(%d)\n", new_exynos_crtc_state->needs_reconfigure);

	exynos_partial_save_log(&plog, old_partial_r, &req, partial_r,
				new_exynos_crtc_state->needs_reconfigure);
	DPU_EVENT_LOG(DPU_EVT_PARTIAL_PREPARE, decon->id, &plog);
}

void exynos_partial_reconfig_coords(struct exynos_partial *partial,
			struct drm_plane_state *plane_state,
			const struct drm_rect *partial_r)
{
	plane_state->visible = drm_rect_clip_scaled(&plane_state->src,
			&plane_state->dst, partial_r);
	if (!plane_state->visible)
		return;

	drm_rect_translate(&plane_state->dst, -(partial_r->x1), -(partial_r->y1));

	pr_debug("reconfigured coordinates:\n");
	exynos_plane_print_info(plane_state);
}

void exynos_partial_update(struct exynos_partial *partial,
				const struct drm_rect *old_partial_region,
				struct drm_rect *new_partial_region)
{
	struct decon_device *decon = partial->decon;

	if (!decon)
		return;

	if (drm_rect_equals(old_partial_region, new_partial_region))
		return;

	partial->funcs->send_partial_command(partial, new_partial_region);
	partial->funcs->set_partial_size(partial, new_partial_region);
	DPU_EVENT_LOG(DPU_EVT_PARTIAL_UPDATE, decon->id, new_partial_region);
	pr_region("applied partial region", new_partial_region);
}

void exynos_partial_restore(struct exynos_partial *partial)
{
	struct decon_device *decon = partial->decon;
	struct drm_crtc_state *crtc_state;
	struct exynos_drm_crtc_state *exynos_crtc_state;
	struct drm_rect *old_partial_region;

	if (!decon)
		return;

	crtc_state = decon->crtc->base.state;
	if (!crtc_state)
		return;

	exynos_crtc_state = to_exynos_crtc_state(crtc_state);
	old_partial_region = &exynos_crtc_state->partial_region;
	if (exynos_partial_is_full(&crtc_state->mode, old_partial_region))
		return;

	partial->funcs->set_partial_size(partial, old_partial_region);
	DPU_EVENT_LOG(DPU_EVT_PARTIAL_RESTORE, decon->id, old_partial_region);
	pr_region("restored partial region", old_partial_region);
}
