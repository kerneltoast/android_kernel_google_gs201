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

#include <exynos_drm_drv.h>
#include <exynos_drm_crtc.h>
#include <exynos_drm_fb.h>
#include <exynos_drm_plane.h>
#include <exynos_drm_dpp.h>
#include <exynos_drm_format.h>

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
		plane->state->normalized_zpos = exynos_plane->config->zpos;
		plane->state->alpha = DRM_BLEND_ALPHA_OPAQUE;
		plane->state->pixel_blend_mode = DRM_MODE_BLEND_PREMULTI;
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

	if (property == exynos_plane->props.max_luminance)
		exynos_state->max_luminance = val;
	else if (property == exynos_plane->props.min_luminance)
		exynos_state->min_luminance = val;
	else if (property == exynos_plane->props.standard)
		exynos_state->standard = val;
	else if (property == exynos_plane->props.transfer)
		exynos_state->transfer = val;
	else if (property == exynos_plane->props.range)
		exynos_state->range = val;
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

	if (property == exynos_plane->props.restriction)
		*val = exynos_state->blob_id_restriction;
	else if (property == exynos_plane->props.max_luminance)
		*val = exynos_state->max_luminance;
	else if (property == exynos_plane->props.min_luminance)
		*val = exynos_state->min_luminance;
	else if (property == exynos_plane->props.standard)
		*val = exynos_state->standard;
	else if (property == exynos_plane->props.transfer)
		*val = exynos_state->transfer;
	else if (property == exynos_plane->props.range)
		*val = exynos_state->range;
	else
		return -EINVAL;

	return 0;
}

static void exynos_drm_plane_print_state(struct drm_printer *p,
					 const struct drm_plane_state *state)
{
	struct exynos_drm_plane_state *exynos_state =
		to_exynos_plane_state(state);
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(state->plane);
	struct dpp_device *dpp = plane_to_dpp(exynos_plane);

	drm_printf(p, "\talpha: 0x%x\n", state->alpha);
	drm_printf(p, "\tluminance: min=%d max=%d\n",
		   exynos_state->min_luminance, exynos_state->max_luminance);
	drm_printf(p, "\tDPP #%d", dpp->id);
	if (dpp->state == DPP_STATE_OFF) {
		drm_printf(p, " (off)\n");
	} else {
		drm_printf(p, "\n\t\tdecon_id=%d\n", dpp->decon_id);
		if (dpp->is_win_connected)
			drm_printf(p, "\t\twin_id=%d\n", dpp->win_id);
	}
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
	.atomic_print_state = exynos_drm_plane_print_state,
};

static int
exynos_drm_plane_check_format(const struct exynos_drm_plane_config *config,
			      struct exynos_drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->base.fb;

	if (!fb)
		return 0;

	if (fb->modifier) {
		if (has_all_bits(DRM_FORMAT_MOD_ARM_AFBC(0), fb->modifier))
			return 0;

		if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_COLORMAP, fb->modifier))
			return 0;

		if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0), fb->modifier))
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
	struct drm_crtc_state *new_crtc_state;
	int ret = 0;

	DRM_DEBUG("%s +\n", __func__);

	if (!state->crtc || !state->fb)
		return 0;

	new_crtc_state = drm_atomic_get_new_crtc_state(state->state,
							state->crtc);

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

	DRM_DEBUG("%s -\n", __func__);

	return ret;
}

static void exynos_plane_disable(struct drm_plane *plane, struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

	if (exynos_crtc->ops->disable_plane)
		exynos_crtc->ops->disable_plane(exynos_crtc, exynos_plane);
}

static void exynos_plane_atomic_update(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct exynos_drm_crtc *exynos_crtc;
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	const struct dpp_device *dpp = plane_to_dpp(exynos_plane);

	if (!state->crtc)
		return;

	exynos_crtc = to_exynos_crtc(state->crtc);

	if (!state->visible || (dpp->win_id != state->normalized_zpos))
		exynos_plane_disable(plane, state->crtc);

	if (state->visible && exynos_crtc->ops->update_plane)
		exynos_crtc->ops->update_plane(exynos_crtc, exynos_plane);
}

static void exynos_plane_atomic_disable(struct drm_plane *plane,
					struct drm_plane_state *old_state)
{
	if (!old_state || !old_state->crtc)
		return;

	exynos_plane_disable(plane, old_state->crtc);
}

static const struct drm_plane_helper_funcs plane_helper_funcs = {
	.atomic_check = exynos_plane_atomic_check,
	.atomic_update = exynos_plane_atomic_update,
	.atomic_disable = exynos_plane_atomic_disable,
};

static int exynos_drm_plane_create_restriction_property(
				struct exynos_drm_plane *exynos_plane,
				const struct exynos_drm_plane_config *config)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property_blob *blob;
	struct drm_property *prop;
	struct dpp_ch_restriction res;
	const struct dpp_device *dpp = plane_to_dpp(exynos_plane);

	memcpy(&res.restriction, &dpp->restriction,
				sizeof(struct dpp_restriction));
	res.id = dpp->id;
	res.attr = dpp->attr;

	blob = drm_property_create_blob(plane->dev, sizeof(res), &res);
	if (IS_ERR(blob))
		return PTR_ERR(blob);

	prop = drm_property_create(plane->dev,
				   DRM_MODE_PROP_IMMUTABLE | DRM_MODE_PROP_BLOB,
				   "hw restrictions", 0);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, blob->base.id);
	exynos_plane->props.restriction = prop;

	return 0;
}

static int exynos_drm_plane_create_standard_property(
				struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;
	static const struct drm_prop_enum_list standard_list[] = {
		{ EXYNOS_STANDARD_UNSPECIFIED, "Unspecified" },
		{ EXYNOS_STANDARD_BT709, "BT709" },
		{ EXYNOS_STANDARD_BT601_625, "BT601_625" },
		{ EXYNOS_STANDARD_BT601_625_UNADJUSTED, "BT601_625_UNADJUSTED"},
		{ EXYNOS_STANDARD_BT601_525, "BT601_525" },
		{ EXYNOS_STANDARD_BT601_525_UNADJUSTED, "BT601_525_UNADJUSTED"},
		{ EXYNOS_STANDARD_BT2020, "BT2020" },
		{ EXYNOS_STANDARD_BT2020_CONSTANT_LUMINANCE,
						"BT2020_CONSTANT_LUMINANCE"},
		{ EXYNOS_STANDARD_BT470M, "BT470M" },
		{ EXYNOS_STANDARD_FILM, "FILM" },
		{ EXYNOS_STANDARD_DCI_P3, "DCI-P3" },
		{ EXYNOS_STANDARD_ADOBE_RGB, "Adobe RGB" },
	};

	prop = drm_property_create_enum(plane->dev, 0, "standard",
				standard_list, ARRAY_SIZE(standard_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop,
				EXYNOS_STANDARD_UNSPECIFIED);
	exynos_plane->props.standard = prop;

	return 0;
}

static int exynos_drm_plane_create_transfer_property(
				struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;
	static const struct drm_prop_enum_list transfer_list[] = {
		{ EXYNOS_TRANSFER_UNSPECIFIED, "Unspecified" },
		{ EXYNOS_TRANSFER_LINEAR, "Linear" },
		{ EXYNOS_TRANSFER_SRGB, "sRGB" },
		{ EXYNOS_TRANSFER_SMPTE_170M, "SMPTE 170M" },
		{ EXYNOS_TRANSFER_GAMMA2_2, "Gamma 2.2" },
		{ EXYNOS_TRANSFER_GAMMA2_6, "Gamma 2.6" },
		{ EXYNOS_TRANSFER_GAMMA2_8, "Gamma 2.8" },
		{ EXYNOS_TRANSFER_ST2084, "ST2084" },
		{ EXYNOS_TRANSFER_HLG, "HLG" },
	};

	prop = drm_property_create_enum(plane->dev, 0, "transfer",
				transfer_list, ARRAY_SIZE(transfer_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop,
				EXYNOS_TRANSFER_UNSPECIFIED);
	exynos_plane->props.transfer = prop;

	return 0;
}

static int exynos_drm_plane_create_range_property(
				struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;
	static const struct drm_prop_enum_list range_list[] = {
		{ EXYNOS_RANGE_UNSPECIFIED, "Unspecified" },
		{ EXYNOS_RANGE_FULL, "Full" },
		{ EXYNOS_RANGE_LIMITED, "Limited" },
		{ EXYNOS_RANGE_EXTENDED, "Extended" },
	};

	prop = drm_property_create_enum(plane->dev, 0, "range", range_list,
						ARRAY_SIZE(range_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop,
				EXYNOS_RANGE_UNSPECIFIED);
	exynos_plane->props.range = prop;

	return 0;
}

static int exynos_drm_plane_create_max_luminance_property(
				struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;

	prop = drm_property_create_range(plane->dev, 0, "max_luminance", 0,
			UINT_MAX);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, 0);
	exynos_plane->props.max_luminance = prop;

	return 0;
}

static int exynos_drm_plane_create_min_luminance_property(
				struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;

	prop = drm_property_create_range(plane->dev, 0, "min_luminance", 0,
			UINT_MAX);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, 0);
	exynos_plane->props.min_luminance = prop;

	return 0;
}

int exynos_plane_init(struct drm_device *dev,
		      struct exynos_drm_plane *exynos_plane, unsigned int index,
		      const struct exynos_drm_plane_config *config)
{
	int err;
	struct dpp_device *dpp = plane_to_dpp(exynos_plane);
	unsigned int supported_modes = BIT(DRM_MODE_BLEND_PIXEL_NONE) |
				       BIT(DRM_MODE_BLEND_PREMULTI) |
				       BIT(DRM_MODE_BLEND_COVERAGE);
	struct drm_plane *plane = &exynos_plane->base;

	err = drm_universal_plane_init(dev, plane,
				       1 << dev->mode_config.num_crtc,
				       &exynos_plane_funcs,
				       config->pixel_formats,
				       config->num_pixel_formats,
				       NULL, config->type, NULL);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		return err;
	}

	drm_plane_helper_add(plane, &plane_helper_funcs);

	exynos_plane->index = index;
	exynos_plane->config = config;

	drm_plane_create_alpha_property(plane);
	drm_plane_create_blend_mode_property(plane, supported_modes);
	drm_plane_create_zpos_property(plane, 0, 0, MAX_PLANE - 1);

	if (test_bit(DPP_ATTR_ROT, &dpp->attr))
		drm_plane_create_rotation_property(plane, DRM_MODE_ROTATE_0,
				DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
				DRM_MODE_ROTATE_180 | DRM_MODE_ROTATE_270 |
				DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);
	else if (test_bit(DPP_ATTR_FLIP, &dpp->attr))
		drm_plane_create_rotation_property(plane, DRM_MODE_ROTATE_0,
				DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_180 |
				DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);

	if (test_bit(DPP_ATTR_HDR, &dpp->attr)) {
		exynos_drm_plane_create_max_luminance_property(exynos_plane);
		exynos_drm_plane_create_min_luminance_property(exynos_plane);
	}

	exynos_drm_plane_create_restriction_property(exynos_plane, config);
	exynos_drm_plane_create_standard_property(exynos_plane);
	exynos_drm_plane_create_transfer_property(exynos_plane);
	exynos_drm_plane_create_range_property(exynos_plane);

	return 0;
}
