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

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/exynos_drm.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_dpp.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_format.h"
#include "exynos_drm_plane.h"
#include "exynos_drm_decon.h"

static struct drm_plane_state *
exynos_drm_plane_duplicate_state(struct drm_plane *plane)
{
	struct drm_plane_state *old_state = plane->state;
	struct exynos_drm_plane_state *old_exynos_state, *new_exynos_state;
	struct exynos_drm_plane_state *copy;

	old_exynos_state = to_exynos_plane_state(old_state);
	copy = kzalloc(sizeof(*old_exynos_state), GFP_KERNEL);
	if (!copy)
		return NULL;

	memcpy(copy, old_exynos_state, sizeof(*old_exynos_state));

	if (copy->eotf_lut)
		drm_property_blob_get(copy->eotf_lut);
	if (copy->oetf_lut)
		drm_property_blob_get(copy->oetf_lut);
	if (copy->gm)
		drm_property_blob_get(copy->gm);
	if (copy->tm)
		drm_property_blob_get(copy->tm);
	if (copy->block)
		drm_property_blob_get(copy->block);

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);

	new_exynos_state = copy;
	if (old_state->fb) {
		drm_framebuffer_get(old_state->fb);
		new_exynos_state->old_fb = old_state->fb;
	} else {
		new_exynos_state->old_fb = NULL;
	}

	return &copy->base;
}

static void exynos_drm_plane_destroy_state(struct drm_plane *plane,
					   struct drm_plane_state *old_state)
{
	struct exynos_drm_plane_state *old_exynos_state =
					to_exynos_plane_state(old_state);

	if (old_exynos_state->old_fb) {
		drm_framebuffer_put(old_exynos_state->old_fb);
		old_exynos_state->old_fb = NULL;
	}

	drm_property_blob_put(old_exynos_state->eotf_lut);
	drm_property_blob_put(old_exynos_state->oetf_lut);
	drm_property_blob_put(old_exynos_state->gm);
	drm_property_blob_put(old_exynos_state->tm);
	drm_property_blob_put(old_exynos_state->block);
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
		plane->state->zpos = exynos_plane->index;
		plane->state->normalized_zpos = exynos_plane->index;
		plane->state->alpha = DRM_BLEND_ALPHA_OPAQUE;
		plane->state->pixel_blend_mode = DRM_MODE_BLEND_PREMULTI;
	}
}

static int
exynos_drm_replace_property_blob_from_id(struct drm_device *dev,
					 struct drm_property_blob **blob,
					 uint64_t blob_id,
					 ssize_t expected_size)
{
	struct drm_property_blob *new_blob = NULL;

	if (blob_id != 0) {
		new_blob = drm_property_lookup_blob(dev, blob_id);
		if (!new_blob)
			return -EINVAL;

		if (new_blob->length != expected_size) {
			drm_property_blob_put(new_blob);
			return -EINVAL;
		}
	}

	drm_property_replace_blob(blob, new_blob);
	drm_property_blob_put(new_blob);

	return 0;
}

static int exynos_drm_plane_set_property(struct drm_plane *plane,
				   struct drm_plane_state *state,
				   struct drm_property *property,
				   uint64_t val)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane_state *exynos_state =
						to_exynos_plane_state(state);
	int ret = 0;

	if (property == exynos_plane->props.max_luminance) {
		exynos_state->max_luminance = val;
	} else if (property == exynos_plane->props.min_luminance) {
		exynos_state->min_luminance = val;
	} else if (property == exynos_plane->props.standard) {
		exynos_state->standard = val;
	} else if (property == exynos_plane->props.transfer) {
		exynos_state->transfer = val;
	} else if (property == exynos_plane->props.range) {
		exynos_state->range = val;
	} else if (property == exynos_plane->props.colormap) {
		exynos_state->colormap = val;
	} else if (property == exynos_plane->props.eotf_lut) {
#if defined(CONFIG_SOC_ZUMA)
		ret = exynos_drm_replace_property_blob_from_id(
				state->plane->dev, &exynos_state->eotf_lut,
				val, sizeof(struct hdr_eotf_lut_v2p2));
#else
		ret = exynos_drm_replace_property_blob_from_id(
				state->plane->dev, &exynos_state->eotf_lut,
				val, sizeof(struct hdr_eotf_lut));
#endif
	} else if (property == exynos_plane->props.oetf_lut) {
#if defined(CONFIG_SOC_ZUMA)
		ret = exynos_drm_replace_property_blob_from_id(
				state->plane->dev, &exynos_state->oetf_lut,
				val, sizeof(struct hdr_oetf_lut_v2p2));
#else
		ret = exynos_drm_replace_property_blob_from_id(
				state->plane->dev, &exynos_state->oetf_lut,
				val, sizeof(struct hdr_oetf_lut));
#endif
	} else if (property == exynos_plane->props.gm) {
		ret = exynos_drm_replace_property_blob_from_id(
				state->plane->dev, &exynos_state->gm,
				val, sizeof(struct hdr_gm_data));
	} else if (property == exynos_plane->props.tm) {
#if defined(CONFIG_SOC_ZUMA)
		ret = exynos_drm_replace_property_blob_from_id(
				state->plane->dev, &exynos_state->tm,
				val, sizeof(struct hdr_tm_data_v2p2));
#else
		ret = exynos_drm_replace_property_blob_from_id(
				state->plane->dev, &exynos_state->tm,
				val, sizeof(struct hdr_tm_data));
#endif
	} else if (property == exynos_plane->props.block) {
		ret = exynos_drm_replace_property_blob_from_id(
				state->plane->dev, &exynos_state->block,
				val, sizeof(struct decon_win_rect));
	} else {
		return -EINVAL;
	}

	return ret;
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
	else if (property == exynos_plane->props.colormap)
		*val = exynos_state->colormap;
	else if (property == exynos_plane->props.eotf_lut)
		*val = (exynos_state->eotf_lut) ?
			exynos_state->eotf_lut->base.id : 0;
	else if (property == exynos_plane->props.oetf_lut)
		*val = (exynos_state->oetf_lut) ?
			exynos_state->oetf_lut->base.id : 0;
	else if (property == exynos_plane->props.gm)
		*val = (exynos_state->gm) ? exynos_state->gm->base.id : 0;
	else if (property == exynos_plane->props.tm)
		*val = (exynos_state->tm) ? exynos_state->tm->base.id : 0;
	else if (property == exynos_plane->props.block)
		*val = (exynos_state->block) ? exynos_state->block->base.id : 0;
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

static int exynos_drm_plane_late_register(struct drm_plane *plane)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

	return exynos_drm_debugfs_plane_add(exynos_plane);
}

static int
exynos_drm_check_format_modifier(u32 format, u64 modifier)
{
	if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_COLORMAP, modifier))
		return 0;

	/* allow rest of the modifiers to support content protection */
	modifier &= ~DRM_FORMAT_MOD_PROTECTION;

	if (!modifier ||
	    has_all_bits(DRM_FORMAT_MOD_ARM_AFBC(0), modifier) ||
	    has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0), modifier))
		return 0;

	/* TODO: is DRM_FORMAT_MOD_SAMSUNG_YUV_8_2_SPLIT and DRM_FORMAT_MOD_LINEAR
	 * supported??
	 */

	DRM_ERROR("not supported modifier(0x%llx)\n", modifier);
	return -EOPNOTSUPP;
}

static int
exynos_drm_plane_check_format(struct exynos_drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->base.fb;

	if (!fb || !fb->modifier || !fb->format)
		return 0;

	return exynos_drm_check_format_modifier(fb->format->format, fb->modifier);
}

static bool
exynos_drm_plane_format_mod_supported_per_plane(struct drm_plane *plane,
						uint32_t format,
						uint64_t modifier)
{
	return exynos_drm_check_format_modifier(format, modifier) == 0;
}

static struct drm_plane_funcs exynos_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= exynos_drm_plane_reset,
	.atomic_duplicate_state	= exynos_drm_plane_duplicate_state,
	.atomic_destroy_state	= exynos_drm_plane_destroy_state,
	.atomic_set_property	= exynos_drm_plane_set_property,
	.atomic_get_property	= exynos_drm_plane_get_property,
	.atomic_print_state	= exynos_drm_plane_print_state,
	.late_register		= exynos_drm_plane_late_register,
	.format_mod_supported	= exynos_drm_plane_format_mod_supported_per_plane,
};

#if defined(CONFIG_SOC_ZUMA)
static void
exynos_plane_update_hdr_params(struct exynos_drm_plane_state *exynos_state)
{
	struct exynos_hdr_state *hdr_state = &exynos_state->hdr_state;
	struct hdr_eotf_lut_v2p2 *eotf_lut;
	struct hdr_oetf_lut_v2p2 *oetf_lut;
	struct hdr_gm_data *gm;
	struct hdr_tm_data_v2p2 *tm;

	if (exynos_state->eotf_lut) {
		eotf_lut = (struct hdr_eotf_lut_v2p2 *)exynos_state->eotf_lut->data;
		hdr_state->eotf_lut = eotf_lut;
	} else {
		hdr_state->eotf_lut = NULL;
	}

	if (exynos_state->oetf_lut) {
		oetf_lut = (struct hdr_oetf_lut_v2p2 *)exynos_state->oetf_lut->data;
		hdr_state->oetf_lut = oetf_lut;
	} else {
		hdr_state->oetf_lut = NULL;
	}

	if (exynos_state->gm) {
		gm = (struct hdr_gm_data *)exynos_state->gm->data;
		hdr_state->gm = gm;
	} else {
		hdr_state->gm = NULL;
	}

	if (exynos_state->tm) {
		tm = (struct hdr_tm_data_v2p2 *)exynos_state->tm->data;
		hdr_state->tm = tm;
	} else {
		hdr_state->tm = NULL;
	}
}
#else
static void
exynos_plane_update_hdr_params(struct exynos_drm_plane_state *exynos_state)
{
	struct exynos_hdr_state *hdr_state = &exynos_state->hdr_state;
	struct hdr_eotf_lut *eotf_lut;
	struct hdr_oetf_lut *oetf_lut;
	struct hdr_gm_data *gm;
	struct hdr_tm_data *tm;

	if (exynos_state->eotf_lut) {
		eotf_lut = (struct hdr_eotf_lut *)exynos_state->eotf_lut->data;
		hdr_state->eotf_lut = eotf_lut;
	} else {
		hdr_state->eotf_lut = NULL;
	}

	if (exynos_state->oetf_lut) {
		oetf_lut = (struct hdr_oetf_lut *)exynos_state->oetf_lut->data;
		hdr_state->oetf_lut = oetf_lut;
	} else {
		hdr_state->oetf_lut = NULL;
	}

	if (exynos_state->gm) {
		gm = (struct hdr_gm_data *)exynos_state->gm->data;
		hdr_state->gm = gm;
	} else {
		hdr_state->gm = NULL;
	}

	if (exynos_state->tm) {
		tm = (struct hdr_tm_data *)exynos_state->tm->data;
		hdr_state->tm = tm;
	} else {
		hdr_state->tm = NULL;
	}
}
#endif

static int exynos_plane_atomic_check(struct drm_plane *plane,
				     struct drm_atomic_state *atomic_state)
{
	struct drm_plane_state *state = drm_atomic_get_new_plane_state(atomic_state, plane);
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane_state *exynos_state =
						to_exynos_plane_state(state);
	struct dpp_device *dpp = plane_to_dpp(exynos_plane);
	struct drm_crtc_state *new_crtc_state;
	struct exynos_drm_crtc_state *new_exynos_crtc_state;
	struct decon_device *decon;
	int ret = 0;

	DRM_DEBUG("%s +\n", __func__);

	if (!state->crtc || !state->fb)
		return 0;

	decon = to_exynos_crtc(state->crtc)->ctx;
	new_crtc_state = drm_atomic_get_new_crtc_state(state->state,
							state->crtc);

	if (!new_crtc_state->active)
		return 0;

	new_exynos_crtc_state = to_exynos_crtc_state(new_crtc_state);

	/*
	 * while exiting hibernation, there's no point on running remaining checks
	 */
	if (new_exynos_crtc_state->hibernation_exit)
		return 0;

	ret = drm_atomic_helper_check_plane_state(state, new_crtc_state, 0,
			INT_MAX, true, false);
	if (ret)
		return ret;

	if (decon->partial && new_exynos_crtc_state->needs_reconfigure)
		exynos_partial_reconfig_coords(decon->partial, state,
				&new_exynos_crtc_state->partial_region);

	exynos_plane_update_hdr_params(exynos_state);

	if (dpp->check && state->visible) {
		ret = dpp->check(dpp, exynos_state);
		if (ret)
			return ret;
	}

	ret = exynos_drm_plane_check_format(exynos_state);
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
				       struct drm_atomic_state *atomic_state)
{
	struct drm_plane_state *state = plane->state;
	struct exynos_drm_crtc *exynos_crtc;
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

	if (!state->crtc)
		return;

	exynos_crtc = to_exynos_crtc(state->crtc);

	if (!state->visible)
		exynos_plane_disable(plane, state->crtc);
	else if (exynos_crtc->ops->update_plane)
		exynos_crtc->ops->update_plane(exynos_crtc, exynos_plane);
}

static void exynos_plane_atomic_disable(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(state, plane);

	if (!old_state || !old_state->crtc)
		return;

	exynos_plane_disable(plane, old_state->crtc);
}

static int exynos_plane_prepare_fb(struct drm_plane *plane,
				   struct drm_plane_state *new_state)
{
	struct decon_device *decon;

	if (!new_state || !new_state->crtc) {
		/* Do nothing because we only record event log on decon level */
		return 0;
	}

	decon = crtc_to_decon(new_state->crtc);
	DPU_EVENT_LOG(DPU_EVT_PLANE_PREPARE_FB, decon->id, new_state);

	return 0;
}

static void exynos_plane_cleanup_fb(struct drm_plane *plane,
				   struct drm_plane_state *old_state)
{
	struct decon_device *decon;

	if (!old_state || !old_state->crtc)
		return;

	decon = crtc_to_decon(old_state->crtc);
	DPU_EVENT_LOG(DPU_EVT_PLANE_CLEANUP_FB, decon->id, old_state);
}

static const struct drm_plane_helper_funcs plane_helper_funcs = {
	.prepare_fb = exynos_plane_prepare_fb,
	.cleanup_fb = exynos_plane_cleanup_fb,
	.atomic_check = exynos_plane_atomic_check,
	.atomic_update = exynos_plane_atomic_update,
	.atomic_disable = exynos_plane_atomic_disable,
};

static int exynos_drm_plane_create_restriction_property(
				struct exynos_drm_plane *exynos_plane)
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

static int exynos_drm_plane_create_colormap_property(
				struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_property *prop;

	prop = drm_property_create_range(plane->dev, 0, "colormap", 0,
			UINT_MAX);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, 0);
	exynos_plane->props.colormap = prop;

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

static int
exynos_drm_plane_create_eotf_lut_property(struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_device *dev = plane->dev;
	struct drm_property *prop;

	prop = drm_property_create(dev, DRM_MODE_PROP_BLOB, "eotf_lut", 0);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, 0);
	exynos_plane->props.eotf_lut = prop;

	return 0;
}

static int
exynos_drm_plane_create_oetf_lut_property(struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_device *dev = plane->dev;
	struct drm_property *prop;

	prop = drm_property_create(dev, DRM_MODE_PROP_BLOB, "oetf_lut", 0);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, 0);
	exynos_plane->props.oetf_lut = prop;

	return 0;
}

static int
exynos_drm_plane_create_gm_property(struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_device *dev = plane->dev;
	struct drm_property *prop;

	prop = drm_property_create(dev, DRM_MODE_PROP_BLOB, "gammut_matrix", 0);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, 0);
	exynos_plane->props.gm = prop;

	return 0;
}

static int
exynos_drm_plane_create_tm_property(struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_device *dev = plane->dev;
	struct drm_property *prop;

	prop = drm_property_create(dev, DRM_MODE_PROP_BLOB, "tone_mapping", 0);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, 0);
	exynos_plane->props.tm = prop;

	return 0;
}

static int
exynos_drm_plane_create_block_property(struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_device *dev = plane->dev;
	struct drm_property *prop;

	prop = drm_property_create(dev, DRM_MODE_PROP_BLOB, "block", 0);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&plane->base, prop, 0);
	exynos_plane->props.block = prop;

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

	err = drm_universal_plane_init(dev, plane, 0, &exynos_plane_funcs,
				       config->pixel_formats,
				       config->num_pixel_formats,
				       config->format_modifiers, config->type, NULL);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		return err;
	}

	drm_plane_helper_add(plane, &plane_helper_funcs);

	exynos_plane->index = index;

	if (!test_bit(DPP_ATTR_RCD, &dpp->attr)) {
		drm_plane_create_alpha_property(plane);
		drm_plane_create_blend_mode_property(plane, supported_modes);
		drm_plane_create_zpos_property(plane, config->zpos, 0, MAX_PLANE - 1);
		exynos_drm_plane_create_standard_property(exynos_plane);
		exynos_drm_plane_create_transfer_property(exynos_plane);
		exynos_drm_plane_create_range_property(exynos_plane);
		exynos_drm_plane_create_colormap_property(exynos_plane);
	} else {
		drm_plane_create_zpos_immutable_property(plane, MAX_PLANE);
		exynos_drm_plane_create_block_property(exynos_plane);
	}

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
		exynos_drm_plane_create_eotf_lut_property(exynos_plane);
		exynos_drm_plane_create_oetf_lut_property(exynos_plane);
		exynos_drm_plane_create_gm_property(exynos_plane);
	}

	if (test_bit(DPP_ATTR_HDR10_PLUS, &dpp->attr))
		exynos_drm_plane_create_tm_property(exynos_plane);

	exynos_drm_plane_create_restriction_property(exynos_plane);

	return 0;
}
