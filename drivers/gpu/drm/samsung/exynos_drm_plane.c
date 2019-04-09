// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <drm/drmP.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/exynos_drm.h>
#include "exynos_drm_drv.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_dpp.h"

/*
 * This function is to get X or Y size shown via screen. This needs length and
 * start position of CRTC.
 *
 *      <--- length --->
 * CRTC ----------------
 *      ^ start        ^ end
 *
 * There are six cases from a to f.
 *
 *             <----- SCREEN ----->
 *             0                 last
 *   ----------|------------------|----------
 * CRTCs
 * a -------
 *        b -------
 *        c --------------------------
 *                 d --------
 *                           e -------
 *                                  f -------
 */
static int exynos_plane_get_size(int start, unsigned int length,
		unsigned int last)
{
	int end = start + length;
	int size = 0;

	if (start <= 0) {
		if (end > 0)
			size = min_t(unsigned int, end, last);
	} else if (start <= last) {
		size = min_t(unsigned int, last - start, length);
	}

	return size;
}

static void exynos_plane_mode_set(struct exynos_drm_plane_state *exynos_state)
{
	struct drm_plane_state *state = &exynos_state->base;
	struct drm_crtc *crtc = state->crtc;
	struct drm_crtc_state *crtc_state =
			drm_atomic_get_existing_crtc_state(state->state, crtc);
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	int crtc_x, crtc_y;
	unsigned int crtc_w, crtc_h;
	unsigned int src_x, src_y;
	unsigned int src_w, src_h;
	unsigned int actual_w;
	unsigned int actual_h;

	/*
	 * The original src/dest coordinates are stored in exynos_state->base,
	 * but we want to keep another copy internal to our driver that we can
	 * clip/modify ourselves.
	 */

	crtc_x = state->crtc_x;
	crtc_y = state->crtc_y;
	crtc_w = state->crtc_w;
	crtc_h = state->crtc_h;

	src_x = state->src_x >> 16;
	src_y = state->src_y >> 16;
	src_w = state->src_w >> 16;
	src_h = state->src_h >> 16;

	/* set ratio */
	exynos_state->h_ratio = (src_w << 16) / crtc_w;
	exynos_state->v_ratio = (src_h << 16) / crtc_h;

	/* clip to visible area */
	actual_w = exynos_plane_get_size(crtc_x, crtc_w, mode->hdisplay);
	actual_h = exynos_plane_get_size(crtc_y, crtc_h, mode->vdisplay);

	if (crtc_x < 0) {
		if (actual_w)
			src_x += ((-crtc_x) * exynos_state->h_ratio) >> 16;
		crtc_x = 0;
	}

	if (crtc_y < 0) {
		if (actual_h)
			src_y += ((-crtc_y) * exynos_state->v_ratio) >> 16;
		crtc_y = 0;
	}

	/* set drm framebuffer data. */
	exynos_state->src.x = src_x;
	exynos_state->src.y = src_y;
	exynos_state->src.w = (actual_w * exynos_state->h_ratio) >> 16;
	exynos_state->src.h = (actual_h * exynos_state->v_ratio) >> 16;

	/* set plane range to be displayed. */
	exynos_state->crtc.x = crtc_x;
	exynos_state->crtc.y = crtc_y;
	exynos_state->crtc.w = actual_w;
	exynos_state->crtc.h = actual_h;

	if (crtc_x >= mode->hdisplay || crtc_y >= mode->vdisplay)
		state->visible = false;
	else
		state->visible = true;

	DRM_DEBUG_KMS("plane : offset_x/y(%d,%d), width/height(%d,%d)",
			exynos_state->crtc.x, exynos_state->crtc.y,
			exynos_state->crtc.w, exynos_state->crtc.h);
}

static struct drm_plane_state *
exynos_drm_plane_duplicate_state(struct drm_plane *plane)
{
	struct exynos_drm_plane_state *exynos_state;
	struct exynos_drm_plane_state *copy;

	exynos_state = to_exynos_plane_state(plane->state);
	copy = kzalloc(sizeof(*exynos_state), GFP_KERNEL);
	if (!copy)
		return NULL;

	memcpy(copy, exynos_state, sizeof(*exynos_state));

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);
	return &copy->base;
}

static void exynos_drm_plane_destroy_state(struct drm_plane *plane,
					   struct drm_plane_state *old_state)
{
	struct exynos_drm_plane_state *old_exynos_state =
					to_exynos_plane_state(old_state);
	__drm_atomic_helper_plane_destroy_state(old_state);
	kfree(old_exynos_state);
}

static void exynos_drm_plane_reset(struct drm_plane *plane)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane_state *exynos_state;

	if (plane->state) {
		exynos_drm_plane_destroy_state(plane, plane->state);
		plane->state = NULL;
	}

	exynos_state = kzalloc(sizeof(*exynos_state), GFP_KERNEL);
	if (exynos_state) {
		plane->state = &exynos_state->base;
		plane->state->plane = plane;
		plane->state->zpos = exynos_plane->config->zpos;
	}
}

static int exynos_drm_plane_set_property(struct drm_plane *plane,
				   struct drm_plane_state *state,
				   struct drm_property *property,
				   uint64_t val)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane_state *exynos_state =
						to_exynos_plane_state(state);

	if (property == exynos_plane->props.afbc)
		exynos_state->afbc = val;
	else if (property == exynos_plane->props.alpha)
		exynos_state->alpha = val;
	else if (property == exynos_plane->props.blend_mode)
		exynos_state->blend_mode = val;
	else
		return -EINVAL;

	return 0;
}

static int exynos_drm_plane_get_property(struct drm_plane *plane,
				   const struct drm_plane_state *state,
				   struct drm_property *property,
				   uint64_t *val)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane_state *exynos_state =
			to_exynos_plane_state((struct drm_plane_state *)state);

	if (property == exynos_plane->props.afbc)
		*val = exynos_state->afbc;
	else if (property == exynos_plane->props.alpha)
		*val = exynos_state->alpha;
	else if (property == exynos_plane->props.blend_mode)
		*val = exynos_state->blend_mode;
	else
		return -EINVAL;

	return 0;
}

static struct drm_plane_funcs exynos_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= drm_plane_cleanup,
	.reset		= exynos_drm_plane_reset,
	.atomic_duplicate_state = exynos_drm_plane_duplicate_state,
	.atomic_destroy_state = exynos_drm_plane_destroy_state,
	.atomic_set_property = exynos_drm_plane_set_property,
	.atomic_get_property = exynos_drm_plane_get_property,
};

static int
exynos_drm_plane_check_format(const struct exynos_drm_plane_config *config,
			      struct exynos_drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->base.fb;

	if (fb->modifier) {
		if (fb->modifier & DRM_FORMAT_MOD_ARM_AFBC(0))
			return 0;

		DRM_ERROR("not supported modifier(0x%llx)\n", fb->modifier);
		return -ENOTSUPP;
	}

	return 0;
}

static int exynos_plane_atomic_check(struct drm_plane *plane,
				     struct drm_plane_state *state)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane_state *exynos_state =
						to_exynos_plane_state(state);
	struct dpp_device *dpp = plane_to_dpp(exynos_plane);
	struct decon_device *decon;
	struct drm_crtc_state *new_crtc_state;
	int ret = 0;

	DRM_INFO("%s +\n", __func__);

	if (!state->crtc || !state->fb)
		return 0;

	decon = to_exynos_crtc(state->crtc)->ctx;

	new_crtc_state = state->state->crtcs[state->crtc->index].new_state;

	if (!new_crtc_state->planes_changed || !new_crtc_state->active)
		return 0;

	/* translate state into exynos_state */
	exynos_plane_mode_set(exynos_state);

	if (dpp->check) {
		ret = dpp->check(dpp, exynos_state);
		if (ret)
			return ret;
	}

	ret = exynos_drm_plane_check_format(exynos_plane->config, exynos_state);
	if (ret)
		return ret;

	/*
	 * If multiple planes request the same zpos which means DECON window
	 * number, HW resources can be conflicted.
	 */
	if (test_and_set_bit(exynos_state->base.zpos, &decon->req_windows)) {
		DRM_ERROR("zpos %d conflict.\n", exynos_state->base.zpos);
		clear_bit(exynos_state->base.zpos, &decon->req_windows);
		return -EINVAL;
	}

	DRM_INFO("%s -\n", __func__);

	return ret;
}

static void exynos_plane_atomic_update(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(state->crtc);
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

	if (!state->crtc)
		return;

	plane->crtc = state->crtc;

	if (state->visible && exynos_crtc->ops->update_plane)
		exynos_crtc->ops->update_plane(exynos_crtc, exynos_plane);
	else if (exynos_crtc->ops->disable_plane)
		exynos_crtc->ops->disable_plane(exynos_crtc, exynos_plane);
}

static void exynos_plane_atomic_disable(struct drm_plane *plane,
					struct drm_plane_state *old_state)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(plane->crtc);

	if (!plane->crtc)
		return;

	if (exynos_crtc->ops->disable_plane)
		exynos_crtc->ops->disable_plane(exynos_crtc, exynos_plane);
}

static const struct drm_plane_helper_funcs plane_helper_funcs = {
	.atomic_check = exynos_plane_atomic_check,
	.atomic_update = exynos_plane_atomic_update,
	.atomic_disable = exynos_plane_atomic_disable,
};

static int exynos_plane_create_property(struct exynos_drm_plane *exynos_plane,
				const struct exynos_drm_plane_config *config)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;

	if (config->capabilities & EXYNOS_DRM_PLANE_CAP_AFBC) {
		prop = drm_property_create_bool(plane->dev, 0, "afbc");
		if (!prop) {
			DRM_ERROR("failed to create afbc property\n");
			return -ENOMEM;
		}

		drm_object_attach_property(&plane->base, prop, 0);

		exynos_plane->props.afbc = prop;
	}

	return 0;
}

int exynos_drm_plane_create_alpha_property(
				struct exynos_drm_plane *exynos_plane,
				const struct exynos_drm_plane_config *config)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;

	prop = drm_property_create_range(plane->dev, 0, "alpha",
					 0, EXYNOS_DRM_BLEND_ALPHA_OPAQUE);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop,
					EXYNOS_DRM_BLEND_ALPHA_OPAQUE);
	exynos_plane->props.alpha = prop;

	return 0;
}

int exynos_drm_plane_create_blend_mode_property(
				struct exynos_drm_plane *exynos_plane,
				const struct exynos_drm_plane_config *config)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;
	static const struct drm_prop_enum_list blend_mode_list[] = {
		{ EXYNOS_DRM_MODE_BLEND_PIXEL_NONE, "None" },
		{ EXYNOS_DRM_MODE_BLEND_PREMULTI, "Pre-multiplied" },
		{ EXYNOS_DRM_MODE_BLEND_COVERAGE, "Coverage" },
	};

	prop = drm_property_create_enum(plane->dev, 0, "pixel blend mode",
			blend_mode_list, ARRAY_SIZE(blend_mode_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop,
					EXYNOS_DRM_MODE_BLEND_PREMULTI);
	exynos_plane->props.blend_mode = prop;

	return 0;
}

int exynos_plane_init(struct drm_device *dev,
		      struct exynos_drm_plane *exynos_plane, unsigned int index,
		      const struct exynos_drm_plane_config *config)
{
	int err;
	struct dpp_device *dpp = plane_to_dpp(exynos_plane);

	err = drm_universal_plane_init(dev, &exynos_plane->base,
				       1 << dev->mode_config.num_crtc,
				       &exynos_plane_funcs,
				       config->pixel_formats,
				       config->num_pixel_formats,
				       NULL, config->type, NULL);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		return err;
	}

	drm_plane_helper_add(&exynos_plane->base, &plane_helper_funcs);

	exynos_plane->index = index;
	exynos_plane->config = config;

	exynos_plane_create_property(exynos_plane, config);

	drm_plane_create_zpos_property(&exynos_plane->base, 0, 0,
			MAX_PLANE - 1);

	if (test_bit(DPP_ATTR_ROT, &dpp->attr))
		drm_plane_create_rotation_property(&exynos_plane->base,
				DRM_MODE_ROTATE_0,
				DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
				DRM_MODE_ROTATE_180 | DRM_MODE_ROTATE_270 |
				DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);

	exynos_drm_plane_create_alpha_property(exynos_plane, config);
	exynos_drm_plane_create_blend_mode_property(exynos_plane, config);

	return 0;
}
