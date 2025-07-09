// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_crtc.c
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
#include <drm/drm_atomic_uapi.h>
#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fourcc_gs101.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_vblank.h>
#include <drm/drm.h>

#include <dqe_cal.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_plane.h"

enum crtc_active_state {
	CRTC_STATE_INACTIVE,
	CRTC_STATE_ACTIVE,
	CRTC_STATE_SELF_REFRESH,
};

static void exynos_drm_crtc_atomic_enable(struct drm_crtc *crtc,
					  struct drm_atomic_state *state)
{
	struct drm_crtc_state *old_state = drm_atomic_get_old_crtc_state(state, crtc);
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	const enum crtc_active_state active_state = CRTC_STATE_ACTIVE;

	if (active_state == exynos_crtc->active_state)
		return;

	if (exynos_crtc->ops->enable)
		exynos_crtc->ops->enable(exynos_crtc, old_state);

	if (exynos_crtc->active_state == CRTC_STATE_INACTIVE)
		drm_crtc_vblank_on(crtc);

	exynos_crtc->active_state = active_state;
}

static void exynos_drm_crtc_atomic_disable(struct drm_crtc *crtc,
					   struct drm_atomic_state *state)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	const enum crtc_active_state active_state =
		crtc->state->self_refresh_active ? CRTC_STATE_SELF_REFRESH : CRTC_STATE_INACTIVE;

	if (active_state == exynos_crtc->active_state)
		return;

	if (exynos_crtc->ops->disable)
		exynos_crtc->ops->disable(exynos_crtc);

	if (crtc->state->event && !crtc->state->active) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);

		crtc->state->event = NULL;
	}

	if (active_state == CRTC_STATE_INACTIVE)
		drm_crtc_vblank_off(crtc);

	exynos_crtc->active_state = active_state;
}

static void exynos_crtc_update_lut(struct drm_crtc *crtc,
					struct drm_crtc_state *state)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct drm_color_lut *degamma_lut, *gamma_lut;
	struct cgc_lut *cgc_lut;
	struct decon_device *decon = exynos_crtc->ctx;
	struct exynos_drm_crtc_state *exynos_state;
	struct exynos_dqe_state *dqe_state;
	int i;

	if (!decon->dqe)
		return;

	exynos_state = to_exynos_crtc_state(state);
	dqe_state = &exynos_state->dqe;

	if (exynos_state->cgc_lut) {
		cgc_lut = exynos_state->cgc_lut->data;
		dqe_state->cgc_lut = cgc_lut;
	} else {
		dqe_state->cgc_lut = NULL;
	}

	if (exynos_state->disp_dither)
		dqe_state->disp_dither_config = exynos_state->disp_dither->data;
	else
		dqe_state->disp_dither_config = NULL;

	if (exynos_state->cgc_dither)
		dqe_state->cgc_dither_config = exynos_state->cgc_dither->data;
	else
		dqe_state->cgc_dither_config = NULL;

	for (i = 0; i < HISTOGRAM_MAX; i++) {
		if (exynos_state->histogram[i])
			dqe_state->hist_chan[i].config = exynos_state->histogram[i]->data;
		else
			dqe_state->hist_chan[i].config = NULL;
	}

	if (exynos_state->linear_matrix)
		dqe_state->linear_matrix = exynos_state->linear_matrix->data;
	else
		dqe_state->linear_matrix = NULL;

	if (exynos_state->gamma_matrix)
		dqe_state->gamma_matrix = exynos_state->gamma_matrix->data;
	else
		dqe_state->gamma_matrix = NULL;

	if (state->degamma_lut) {
		degamma_lut = state->degamma_lut->data;
		dqe_state->degamma_lut = degamma_lut;
	} else {
		dqe_state->degamma_lut = NULL;
	}

	if (state->gamma_lut) {
		gamma_lut = state->gamma_lut->data;
		dqe_state->regamma_lut = gamma_lut;
	} else {
		dqe_state->regamma_lut = NULL;
	}

	dqe_state->cgc_gem = exynos_state->cgc_gem;
}

static int exynos_crtc_atomic_check(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_crtc_state *new_exynos_state =
						to_exynos_crtc_state(crtc_state);
	const struct drm_crtc_state *old_crtc_state =
		drm_atomic_get_old_crtc_state(crtc_state->state, crtc);
	struct exynos_drm_crtc_state *old_exynos_state =
						to_exynos_crtc_state(old_crtc_state);
	struct drm_plane *plane;
	const struct drm_plane_state *plane_state;
	const struct decon_device *decon = exynos_crtc->ctx;
	const struct exynos_dqe *dqe = decon->dqe;
	uint32_t max_bpc;

	DRM_DEBUG("%s +\n", __func__);

	if (!crtc_state->enable)
		return 0;

	if (crtc_state->color_mgmt_changed)
		exynos_crtc_update_lut(crtc, crtc_state);

	if (exynos_crtc->ops->atomic_check)
		exynos_crtc->ops->atomic_check(exynos_crtc, crtc_state);

	if (dqe && (dqe->force_disabled || !new_exynos_state->dqe.enabled) &&
			(decon->config.out_bpc == 8)) {
		new_exynos_state->in_bpc = 8;
	} else if (decon->config.out_type & DECON_OUT_DP) {
		/*
		 * Now, it is force to configure 8 BPC output for DP Path.
		 * To support 10 BPC input stream, DECON Input BPC needs to be set as 10 BPC.
		 */
		new_exynos_state->in_bpc = 10;	// Force 10 BPC Input
	} else if (new_exynos_state->force_bpc == EXYNOS_BPC_MODE_UNSPECIFIED) {
		/*
		 * When force_bpc is not specified,
		 * CRTC's input BPC should be binded as output BPC or Plane's format.
		 */
		if (decon->config.out_bpc == 10) {
			max_bpc = 10;
		} else {
			max_bpc = 8; /* initial bpc value */
			drm_atomic_crtc_state_for_each_plane_state(plane, plane_state, crtc_state) {
				const struct drm_format_info *info;
				const struct dpu_fmt *fmt_info;

				info = plane_state->fb->format;
				fmt_info = dpu_find_fmt_info(info->format);
				if (fmt_info->bpc == 10) {
					max_bpc = 10;
					break;
				}
			}
		}
		new_exynos_state->in_bpc = max_bpc;
	} else {
		new_exynos_state->in_bpc =
			new_exynos_state->force_bpc == EXYNOS_BPC_MODE_10 ?
			10 : 8;
	}

        /* do this only if we have plane to update, this is to avoid skip_update
         * always be ignored on the first commit(from continuous splash)
         */
	if (crtc_state->plane_mask && (old_exynos_state->in_bpc != new_exynos_state->in_bpc))
		crtc_state->color_mgmt_changed = true;

	/*
	 * if the the following conditions are met then skip the update to keep self refresh
	 * contents on the screen and save some cycles
	 *  1. Display is in self refresh
	 *  2. No color mgmt updates
	 *  3. There are no planes updated
	 *
	 * NOTE: may need to handle the case where there is a regular mode set coming out of self
	 * refresh that requires an update, however most mode set updates require planes to be
	 * updated too, and/or we may actually want to just update encoder/bridges/connectors only.
	 */
	if (new_exynos_state->hibernation_exit) {
		new_exynos_state->skip_update = true;
		crtc_state->no_vblank = true;
	} else if (old_crtc_state->self_refresh_active && !crtc_state->color_mgmt_changed &&
		!new_exynos_state->planes_updated) {
		new_exynos_state->skip_update = true;
	} else if (drm_atomic_crtc_effectively_active(old_crtc_state) &&
		   (crtc_state->plane_mask & (~exynos_crtc->rcd_plane_mask)) == 0) {
		DRM_WARN("%s: plane-less update is detected, mask=0x%08X\n", __func__,
			 crtc_state->plane_mask);
	}

	if (decon->rcd) {
		uint32_t rcd_mask = crtc_state->plane_mask & exynos_crtc->rcd_plane_mask;
		uint32_t old_rcd_mask = old_crtc_state->plane_mask & exynos_crtc->rcd_plane_mask;

		new_exynos_state->dqe.rcd_enabled = false;
		crtc_state->color_mgmt_changed |= rcd_mask != old_rcd_mask;

		if (rcd_mask) {
			drm_atomic_crtc_state_for_each_plane_state(plane, plane_state, crtc_state) {
				if (rcd_mask & drm_plane_mask(plane)) {
					new_exynos_state->dqe.rcd_enabled = plane_state->visible;
					break;
				}
			}
		}
	}

	DRM_DEBUG("%s -\n", __func__);

	return 0;
}

static void exynos_crtc_atomic_begin(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	if (exynos_crtc->ops->atomic_begin)
		exynos_crtc->ops->atomic_begin(exynos_crtc, state);
}

static void exynos_crtc_atomic_flush(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct drm_crtc_state *old_crtc_state = drm_atomic_get_old_crtc_state(state, crtc);
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	if (exynos_crtc->ops->atomic_flush)
		exynos_crtc->ops->atomic_flush(exynos_crtc, old_crtc_state);
}

static enum drm_mode_status exynos_crtc_mode_valid(struct drm_crtc *crtc,
	const struct drm_display_mode *mode)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	if (exynos_crtc->ops->mode_valid)
		return exynos_crtc->ops->mode_valid(exynos_crtc, mode);

	return MODE_OK;
}

static bool exynos_crtc_mode_fixup(struct drm_crtc *crtc,
				   const struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	if (exynos_crtc->ops->mode_fixup)
		return exynos_crtc->ops->mode_fixup(exynos_crtc, mode,
						    adjusted_mode);

	return true;
}

static void exynos_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	const struct drm_crtc_state *crtc_state = crtc->state;

	if (exynos_crtc->ops->mode_set)
		return exynos_crtc->ops->mode_set(exynos_crtc,
						  &crtc_state->mode,
						  &crtc_state->adjusted_mode);
}

static const struct drm_crtc_helper_funcs exynos_crtc_helper_funcs = {
	.mode_valid	= exynos_crtc_mode_valid,
	.mode_fixup	= exynos_crtc_mode_fixup,
	.mode_set_nofb	= exynos_crtc_mode_set_nofb,
	.atomic_check	= exynos_crtc_atomic_check,
	.atomic_begin	= exynos_crtc_atomic_begin,
	.atomic_flush	= exynos_crtc_atomic_flush,
	.atomic_enable	= exynos_drm_crtc_atomic_enable,
	.atomic_disable	= exynos_drm_crtc_atomic_disable,
};

void exynos_crtc_handle_event(struct exynos_drm_crtc *exynos_crtc)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct drm_pending_vblank_event *event = crtc->state->event;
	unsigned long flags;

	if (!event)
		return;
	crtc->state->event = NULL;

	WARN_ON(drm_crtc_vblank_get(crtc) != 0);

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	drm_crtc_arm_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static int exynos_drm_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	if (exynos_crtc->ops->enable_vblank)
		return exynos_crtc->ops->enable_vblank(exynos_crtc);

	return 0;
}

static void exynos_drm_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	if (exynos_crtc->ops->disable_vblank)
		exynos_crtc->ops->disable_vblank(exynos_crtc);
}

static u32 exynos_drm_crtc_get_vblank_counter(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	if (exynos_crtc->ops->get_vblank_counter)
		return exynos_crtc->ops->get_vblank_counter(exynos_crtc);

	return 0;
}

static void exynos_drm_crtc_destroy_state(struct drm_crtc *crtc,
					struct drm_crtc_state *state)
{
	struct exynos_drm_crtc_state *exynos_crtc_state;
	int i;

	exynos_crtc_state = to_exynos_crtc_state(state);
	drm_property_blob_put(exynos_crtc_state->cgc_lut);
	drm_property_blob_put(exynos_crtc_state->disp_dither);
	drm_property_blob_put(exynos_crtc_state->cgc_dither);
	drm_property_blob_put(exynos_crtc_state->linear_matrix);
	drm_property_blob_put(exynos_crtc_state->gamma_matrix);
	drm_property_blob_put(exynos_crtc_state->histogram_roi);
	drm_property_blob_put(exynos_crtc_state->histogram_weights);
	drm_property_blob_put(exynos_crtc_state->partial);
	for (i = 0; i < HISTOGRAM_MAX; i++)
		drm_property_blob_put(exynos_crtc_state->histogram[i]);

	if (exynos_crtc_state->cgc_gem)
		drm_gem_object_put(exynos_crtc_state->cgc_gem);
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(exynos_crtc_state);
}

static void exynos_drm_crtc_reset(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc_state *exynos_crtc_state;

	if (crtc->state) {
		exynos_drm_crtc_destroy_state(crtc, crtc->state);
		crtc->state = NULL;
	}

	exynos_crtc_state = kzalloc(sizeof(*exynos_crtc_state), GFP_KERNEL);
	if (exynos_crtc_state) {
		exynos_crtc_state->dqe.enabled = true;
		__drm_atomic_helper_crtc_reset(crtc, &exynos_crtc_state->base);
	} else {
		pr_err("failed to allocate exynos crtc state\n");
	}
}

static struct drm_crtc_state *
exynos_drm_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc_state *exynos_crtc_state;
	struct exynos_drm_crtc_state *copy;
	int i;

	exynos_crtc_state = to_exynos_crtc_state(crtc->state);
	copy = kzalloc(sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return NULL;

	memcpy(copy, exynos_crtc_state, sizeof(*copy));

	if (copy->cgc_lut)
		drm_property_blob_get(copy->cgc_lut);

	if (copy->disp_dither)
		drm_property_blob_get(copy->disp_dither);

	if (copy->cgc_dither)
		drm_property_blob_get(copy->cgc_dither);

	if (copy->linear_matrix)
		drm_property_blob_get(copy->linear_matrix);

	if (copy->gamma_matrix)
		drm_property_blob_get(copy->gamma_matrix);

	if (copy->histogram_roi)
		drm_property_blob_get(copy->histogram_roi);

	if (copy->histogram_weights)
		drm_property_blob_get(copy->histogram_weights);

	for (i = 0; i < HISTOGRAM_MAX; i++) {
		if (copy->histogram[i])
			drm_property_blob_get(copy->histogram[i]);
	}

	if (copy->partial)
		drm_property_blob_get(copy->partial);

	if (copy->cgc_gem)
		drm_gem_object_get(copy->cgc_gem);

	__drm_atomic_helper_crtc_duplicate_state(crtc, &copy->base);

	copy->seamless_mode_changed = false;
	copy->skip_update = false;
	copy->planes_updated = false;
	copy->hibernation_exit = false;

	return &copy->base;
}

struct drm_atomic_state
*exynos_duplicate_active_crtc_state(struct drm_crtc *crtc,
				struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_device *dev = crtc->dev;
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct decon_device *decon = exynos_crtc->ctx;
	int err;

	state = drm_atomic_state_alloc(dev);
	if (!state)
		return ERR_PTR(-ENOMEM);

	state->acquire_ctx = ctx;

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		err = PTR_ERR(crtc_state);
		goto free_state;
	}

	if (!crtc_state->active) {
		if (!atomic_read(&decon->recovery.recovering)) {
			drm_atomic_state_put(state);
			return NULL;
		}
		pr_warn("crtc[%s]: skipping duplication of inactive crtc state\n", crtc->name);
		err = -EPERM;
		goto free_state;
	}

	err = drm_atomic_add_affected_planes(state, crtc);
	if (err)
		goto free_state;

	err = drm_atomic_add_affected_connectors(state, crtc);
	if (err)
		goto free_state;

	/* clear the acquire context so that it isn't accidentally reused */
	state->acquire_ctx = NULL;

free_state:
	if (err < 0) {
		drm_atomic_state_put(state);
		state = ERR_PTR(err);
	}

	return state;
}

struct drm_atomic_state
*exynos_crtc_suspend(struct drm_crtc *crtc,
			struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_crtc_state *crtc_state;
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	struct drm_atomic_state *state, *suspend_state;
	int ret, i;

	suspend_state = exynos_duplicate_active_crtc_state(crtc, ctx);
	if (IS_ERR_OR_NULL(suspend_state))
		return suspend_state;

	state = drm_atomic_state_alloc(crtc->dev);
	if (!state) {
		drm_atomic_state_put(suspend_state);
		return ERR_PTR(-ENOMEM);
	}
	state->acquire_ctx = ctx;
retry:
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		ret = PTR_ERR(crtc_state);
		goto out;
	}

	crtc_state->active = false;

	ret = drm_atomic_set_mode_prop_for_crtc(crtc_state, NULL);
	if (ret)
		goto out;

	ret = drm_atomic_add_affected_planes(state, crtc);
	if (ret)
		goto out;

	ret = drm_atomic_add_affected_connectors(state, crtc);
	if (ret)
		goto out;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		ret = drm_atomic_set_crtc_for_connector(conn_state, NULL);
		if (ret)
			goto out;
	}

	for_each_new_plane_in_state(state, plane, plane_state, i) {
		ret = drm_atomic_set_crtc_for_plane(plane_state, NULL);
		if (ret)
			goto out;

		drm_atomic_set_fb_for_plane(plane_state, NULL);
	}

	ret = drm_atomic_commit(state);
out:
	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		drm_atomic_state_clear(suspend_state);
		ret = drm_modeset_backoff(ctx);
		if (!ret)
			goto retry;
	} else if (ret) {
		drm_atomic_state_put(suspend_state);
		suspend_state = ERR_PTR(ret);
	}

	drm_atomic_state_put(state);

	return suspend_state;
}

int exynos_crtc_resume(struct drm_atomic_state *state,
				struct drm_modeset_acquire_ctx *ctx)
{
	return drm_atomic_helper_commit_duplicated_state(state, ctx);
}

static int
exynos_drm_replace_property_blob_from_id(struct drm_device *dev,
					 struct drm_property_blob **blob,
					 uint64_t blob_id,
					 ssize_t expected_size,
					 ssize_t expected_elem_size,
					 bool *replaced)
{
	struct drm_property_blob *new_blob = NULL;

	if (blob_id != 0) {
		new_blob = drm_property_lookup_blob(dev, blob_id);
		if (new_blob == NULL)
			return -EINVAL;

		if (expected_size > 0 &&
		    new_blob->length != expected_size) {
			drm_property_blob_put(new_blob);
			return -EINVAL;
		}
		if (expected_elem_size > 0 &&
		    new_blob->length % expected_elem_size != 0) {
			drm_property_blob_put(new_blob);
			return -EINVAL;
		}
	}

	*replaced |= drm_property_replace_blob(blob, new_blob);
	drm_property_blob_put(new_blob);

	return 0;
}

static int exynos_drm_crtc_set_property(struct drm_crtc *crtc,
					struct drm_crtc_state *state,
					struct drm_property *property,
					uint64_t val)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_crtc_state *exynos_crtc_state;
	int ret = 0;
	bool replaced = false;

	exynos_crtc_state = to_exynos_crtc_state(state);

	if (property == exynos_crtc->props.color_mode) {
		if (val != exynos_crtc_state->color_mode) {
			exynos_crtc_state->color_mode = val;
			replaced = true;
		}
	} else if (property == exynos_crtc->props.force_bpc) {
		if (val != exynos_crtc_state->force_bpc) {
			exynos_crtc_state->force_bpc = val;
			replaced = true;
		}
	} else if (property == exynos_crtc->props.ppc ||
			property == exynos_crtc->props.max_disp_freq) {
		return 0;
	} else if (property == exynos_crtc->props.dqe_enabled) {
		if (val != exynos_crtc_state->dqe.enabled) {
			exynos_crtc_state->dqe.enabled = val;
			replaced = true;
		}
	} else if (property == exynos_crtc->props.cgc_lut) {
		ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
				&exynos_crtc_state->cgc_lut, val,
				sizeof(struct cgc_lut), -1, &replaced);
	} else if (property == exynos_crtc->props.disp_dither) {
		ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
				&exynos_crtc_state->disp_dither, val,
				sizeof(struct dither_config), -1, &replaced);
	} else if (property == exynos_crtc->props.cgc_dither) {
		ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
				&exynos_crtc_state->cgc_dither, val,
				sizeof(struct dither_config), -1, &replaced);
	} else if (property == exynos_crtc->props.linear_matrix) {
		ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
				&exynos_crtc_state->linear_matrix, val,
				sizeof(struct exynos_matrix), -1, &replaced);
	} else if (property == exynos_crtc->props.gamma_matrix) {
		ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
				&exynos_crtc_state->gamma_matrix, val,
				sizeof(struct exynos_matrix), -1, &replaced);
	} else if (property == exynos_crtc->props.histogram_roi) {
		pr_warn_once("legacy property(%s): ignored\n", property->name);
		ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
				&exynos_crtc_state->histogram_roi, val,
				sizeof(struct histogram_roi), -1, &replaced);
	} else if (property == exynos_crtc->props.histogram_weights) {
		pr_warn_once("legacy property(%s): ignored\n", property->name);
		ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
				&exynos_crtc_state->histogram_weights, val,
				sizeof(struct histogram_weights), -1, &replaced);
	} else if (property == exynos_crtc->props.histogram_pos) {
		pr_warn_once("legacy property(%s): ignored\n", property->name);
		if (val != exynos_crtc_state->dqe.histogram_pos) {
			exynos_crtc_state->dqe.histogram_pos = val;
			replaced = true;
		}
	} else if (property == exynos_crtc->props.histogram_threshold) {
		pr_warn_once("legacy property(%s): ignored\n", property->name);
		if (val != exynos_crtc_state->dqe.histogram_threshold) {
			exynos_crtc_state->dqe.histogram_threshold = val;
			replaced = true;
		}
	} else if (!strncmp(property->name, "histogram_", 10)) {
		int i;

		ret = -EINVAL; /* assume an error by default */
		for (i = 0; i < HISTOGRAM_MAX; i++) {
			if (property == exynos_crtc->props.histogram[i]) {
				ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
					&exynos_crtc_state->histogram[i], val,
					sizeof(struct histogram_channel_config), -1, &replaced);
				break;
			}
		}
	} else if (property == exynos_crtc->props.partial) {
		ret = exynos_drm_replace_property_blob_from_id(state->crtc->dev,
				&exynos_crtc_state->partial, val,
				sizeof(struct drm_clip_rect), -1, &replaced);
		return ret;
	} else if (property == exynos_crtc->props.cgc_lut_fd) {
		if (exynos_crtc_state->cgc_gem)
			drm_gem_object_put(exynos_crtc_state->cgc_gem);
		exynos_crtc_state->cgc_gem = (U642I64(val) >= 0) ?
			exynos_drm_gem_fd_to_obj(crtc->dev, U642I64(val)) : NULL;
		replaced = true;
	} else if (property == exynos_crtc->props.expected_present_time) {
		exynos_crtc_state->expected_present_time = val;
	} else {
		return -EINVAL;
	}

	state->color_mgmt_changed |= replaced;
	return ret;
}

static int exynos_drm_crtc_get_property(struct drm_crtc *crtc,
					const struct drm_crtc_state *state,
					struct drm_property *property,
					uint64_t *val)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_crtc_state *exynos_crtc_state;
	struct decon_device *decon = exynos_crtc->ctx;

	exynos_crtc_state =
		to_exynos_crtc_state((struct drm_crtc_state *)state);

	if (property == exynos_crtc->props.color_mode) {
		*val = exynos_crtc_state->color_mode;
	} else if (property == exynos_crtc->props.ppc) {
		*val = decon->bts.ppc;
	} else if (property == exynos_crtc->props.max_disp_freq) {
		*val = decon->bts.dvfs_max_disp_freq;
	} else if (property == exynos_crtc->props.force_bpc) {
		*val = exynos_crtc_state->force_bpc;
	} else if (property == exynos_crtc->props.dqe_enabled) {
		*val = exynos_crtc_state->dqe.enabled;
	} else if (property == exynos_crtc->props.cgc_lut) {
		*val = (exynos_crtc_state->cgc_lut) ?
			exynos_crtc_state->cgc_lut->base.id : 0;
	} else if (property == exynos_crtc->props.disp_dither) {
		*val = (exynos_crtc_state->disp_dither) ?
			exynos_crtc_state->disp_dither->base.id : 0;
	} else if (property == exynos_crtc->props.cgc_dither) {
		*val = (exynos_crtc_state->cgc_dither) ?
			exynos_crtc_state->cgc_dither->base.id : 0;
	} else if (property == exynos_crtc->props.linear_matrix) {
		*val = (exynos_crtc_state->linear_matrix) ?
			exynos_crtc_state->linear_matrix->base.id : 0;
	} else if (property == exynos_crtc->props.gamma_matrix) {
		*val = (exynos_crtc_state->gamma_matrix) ?
			exynos_crtc_state->gamma_matrix->base.id : 0;
	} else if (property == exynos_crtc->props.partial) {
		*val = (exynos_crtc_state->partial) ?
			exynos_crtc_state->partial->base.id : 0;
	} else if (property == exynos_crtc->props.cgc_lut_fd) {
		*val =  (exynos_crtc_state->cgc_gem) ?
			dma_buf_fd(exynos_crtc_state->cgc_gem->dma_buf, 0) : 0;
	} else if (property == exynos_crtc->props.expected_present_time) {
		*val = exynos_crtc_state->expected_present_time;
	} else if (property == exynos_crtc->props.rcd_plane_id) {
		*val = decon->rcd->plane.base.base.id;
	} else if (property == exynos_crtc->props.histogram_roi) {
		*val = (exynos_crtc_state->histogram_roi) ?
			exynos_crtc_state->histogram_roi->base.id : 0;
	} else if (property == exynos_crtc->props.histogram_weights) {
		*val = (exynos_crtc_state->histogram_weights) ?
			exynos_crtc_state->histogram_weights->base.id : 0;
	} else if (property == exynos_crtc->props.histogram_pos) {
		*val = exynos_crtc_state->dqe.histogram_pos;
	} else if (property == exynos_crtc->props.histogram_threshold) {
		*val = exynos_crtc_state->dqe.histogram_threshold;
	} else if (!strncmp(property->name, "histogram_", 10)) {
		/*
		 * value 0: channel is free
		 * value 1: channel is occupied
		 */
		int i;
		for (i = 0; i < HISTOGRAM_MAX; i++) {
			if (property == exynos_crtc->props.histogram[i]) {
				struct exynos_dqe *dqe = decon->dqe;
				struct histogram_chan_state *hist_chan = &dqe->state.hist_chan[i];

				*val = (exynos_crtc_state->histogram[i] || hist_chan->cb) ? 1 : 0;
				return 0;
			}
		}

		return -EINVAL;
	} else {
		return -EINVAL;
	}

	return 0;
}

static void exynos_drm_crtc_print_state(struct drm_printer *p,
					const struct drm_crtc_state *state)
{
	const struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(state->crtc);
	const struct exynos_drm_crtc_state *exynos_crtc_state = to_exynos_crtc_state(state);
	const struct decon_device *decon = exynos_crtc->ctx;
	const struct decon_config *cfg = &decon->config;
	struct exynos_drm_crtc_state *exynos_state;
	struct drm_clip_rect *partial_region;

	exynos_state = container_of(state, struct exynos_drm_crtc_state, base);

	drm_printf(p, "\treserved_win_mask=0x%x\n", exynos_crtc_state->reserved_win_mask);
	drm_printf(p, "\tDecon #%u (state:%d)\n", decon->id, decon->state);
	drm_printf(p, "\t\ttype=0x%x\n", cfg->out_type);
	drm_printf(p, "\t\tsize=%dx%d\n", cfg->image_width, cfg->image_height);
	if (cfg->mode.dsi_mode != DSI_MODE_NONE) {
		drm_printf(p, "\t\tdsi_mode=%s (%d)\n",
			cfg->mode.op_mode == DECON_VIDEO_MODE ? "vid" : "cmd",
			cfg->mode.dsi_mode);
		if (cfg->mode.op_mode == DECON_COMMAND_MODE)
			drm_printf(p, "\t\ttrig_mode=%s ddi=%d\n",
				cfg->mode.trig_mode == DECON_HW_TRIG ? "hw" : "sw",
				cfg->te_from);
	}
	drm_printf(p, "\t\tbpc=%d\n", cfg->out_bpc);

	if (exynos_state->partial) {
		partial_region =
			(struct drm_clip_rect *)exynos_state->partial->data;
		drm_printf(p, "\t\tpartial region[%d %d %d %d]\n",
				partial_region->x1, partial_region->y1,
				partial_region->x2 - partial_region->x1,
				partial_region->y2 - partial_region->y1);
	} else {
		drm_printf(p, "\t\tno partial region request\n");
	}
}

static int exynos_drm_crtc_late_register(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct decon_device *decon = exynos_crtc->ctx;

	return dpu_init_debug(decon);
}

static const struct drm_crtc_funcs exynos_crtc_funcs = {
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= exynos_drm_crtc_reset,
	.atomic_duplicate_state	= exynos_drm_crtc_duplicate_state,
	.atomic_destroy_state	= exynos_drm_crtc_destroy_state,
	.atomic_set_property	= exynos_drm_crtc_set_property,
	.atomic_get_property	= exynos_drm_crtc_get_property,
	.atomic_print_state     = exynos_drm_crtc_print_state,
	.enable_vblank		= exynos_drm_crtc_enable_vblank,
	.disable_vblank		= exynos_drm_crtc_disable_vblank,
	.get_vblank_counter	= exynos_drm_crtc_get_vblank_counter,
	.late_register		= exynos_drm_crtc_late_register,
};

static int
exynos_drm_crtc_create_color_mode_property(struct exynos_drm_crtc *exynos_crtc)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct drm_property *prop;
	static const struct drm_prop_enum_list color_mode_list[] = {
		{ HAL_COLOR_MODE_NATIVE, "Native" },
		{ HAL_COLOR_MODE_STANDARD_BT601_625, "BT601_625" },
		{ HAL_COLOR_MODE_STANDARD_BT601_625_UNADJUSTED,
						"BT601_625_UNADJUSTED" },
		{ HAL_COLOR_MODE_STANDARD_BT601_525, "BT601_525" },
		{ HAL_COLOR_MODE_STANDARD_BT601_525_UNADJUSTED,
						"BT601_525_UNADJUSTED" },
		{ HAL_COLOR_MODE_STANDARD_BT709, "BT709" },
		{ HAL_COLOR_MODE_DCI_P3, "DCI-P3" },
		{ HAL_COLOR_MODE_SRGB, "sRGB" },
		{ HAL_COLOR_MODE_ADOBE_RGB, "Adobe RGB" },
		{ HAL_COLOR_MODE_DISPLAY_P3, "Display P3" },
		{ HAL_COLOR_MODE_BT2020, "BT2020" },
		{ HAL_COLOR_MODE_BT2100_PQ, "BT2100 PQ" },
		{ HAL_COLOR_MODE_BT2100_HLG, "BT2100 HLG" },
	};

	prop = drm_property_create_enum(crtc->dev, 0, "color mode",
			color_mode_list, ARRAY_SIZE(color_mode_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, prop, HAL_COLOR_MODE_NATIVE);
	exynos_crtc->props.color_mode = prop;

	return 0;
}

static int
exynos_drm_crtc_create_force_bpc_property(struct exynos_drm_crtc *exynos_crtc)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct drm_property *prop;
	static const struct drm_prop_enum_list bpc_list[] = {
		{ EXYNOS_BPC_MODE_UNSPECIFIED, "Unspecified" },
		{ EXYNOS_BPC_MODE_8, "8bpc" },
		{ EXYNOS_BPC_MODE_10, "10bpc" },
	};

	prop = drm_property_create_enum(crtc->dev, 0, "force_bpc", bpc_list,
			ARRAY_SIZE(bpc_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, prop,
				EXYNOS_BPC_MODE_UNSPECIFIED);
	exynos_crtc->props.force_bpc = prop;

	return 0;
}

static int exynos_drm_crtc_create_bool(struct drm_crtc *crtc, const char *name,
		struct drm_property **prop)
{
	struct drm_property *p;

	p = drm_property_create_bool(crtc->dev, 0, name);
	if (!p)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, p, 0);
	*prop = p;

	return 0;
}

static int exynos_drm_crtc_create_range(struct drm_crtc *crtc, const char *name,
		struct drm_property **prop, uint64_t min, uint64_t max)
{
	struct drm_property *p;

	p = drm_property_create_range(crtc->dev, 0, name, min, max);
	if (!p)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, p, 0);
	*prop = p;

	return 0;
}

static int exynos_drm_crtc_create_signed_range(struct drm_crtc *crtc, const char *name,
		struct drm_property **prop, uint64_t min, uint64_t max)
{
	struct drm_property *p;

	p = drm_property_create_signed_range(crtc->dev, 0, name, min, max);
	if (!p)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, p, 0);
	*prop = p;

	return 0;
}

static int exynos_drm_crtc_create_blob(struct drm_crtc *crtc, const char *name,
		struct drm_property **prop)
{
	struct drm_property *p;

	p = drm_property_create(crtc->dev, DRM_MODE_PROP_BLOB, name, 0);
	if (!p)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, p, 0);
	*prop = p;

	return 0;
}

static int exynos_drm_crtc_histogram_pos_property(struct exynos_drm_crtc *exynos_crtc)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct drm_property *prop;
	static const struct drm_prop_enum_list histogram_pos_list[] = {
		{ POST_DQE, "Post DQE" },
		{ PRE_DQE, "Pre DQE" },
	};
	u32 flags = 0;

	if (IS_ENABLED(CONFIG_SOC_GS101))
		flags |= DRM_MODE_PROP_IMMUTABLE;

	prop = drm_property_create_enum(crtc->dev, flags, "histogram_pos",
				histogram_pos_list, ARRAY_SIZE(histogram_pos_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, prop, POST_DQE);
	exynos_crtc->props.histogram_pos = prop;

	return 0;
}

static int exynos_drm_crtc_histogram_channels_property(struct exynos_drm_crtc *exynos_crtc)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct drm_property *prop;
	static const struct drm_prop_enum_list histogram_list[] = {
		{HISTOGRAM_0, "histogram_0"},
#ifdef CONFIG_SOC_ZUMA
		{HISTOGRAM_1, "histogram_1"},
		{HISTOGRAM_2, "histogram_2"},
		{HISTOGRAM_3, "histogram_3"},
#endif
	};
	u32 bitmask = BIT(HISTOGRAM_MAX) - 1;

	prop = drm_property_create_bitmask(crtc->dev, DRM_MODE_PROP_IMMUTABLE,
					   "histogram_channels",
					   histogram_list, ARRAY_SIZE(histogram_list), bitmask);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, prop, HISTOGRAM_MAX);
	exynos_crtc->props.histogram_channels = prop;

	return 0;
}

static int exynos_drm_crtc_create_histogram_properties(
			struct exynos_drm_crtc *exynos_crtc)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	int ret;
	int i;

	/* legacy properties */
	ret = exynos_drm_crtc_create_blob(crtc, "histogram_roi",
				&exynos_crtc->props.histogram_roi);
	if (ret)
		return ret;

	ret = exynos_drm_crtc_create_blob(crtc, "histogram_weights",
				&exynos_crtc->props.histogram_weights);
	if (ret)
		return ret;

	ret = exynos_drm_crtc_create_range(crtc, "histogram_threshold",
				&exynos_crtc->props.histogram_threshold,
				0, GENMASK(9, 0));
	if (ret)
		return ret;

	ret = exynos_drm_crtc_histogram_pos_property(exynos_crtc);
	if (ret)
		return ret;

	/* multi-channel support */
	ret = exynos_drm_crtc_histogram_channels_property(exynos_crtc);
	if (ret)
		return ret;

	for (i = 0; i < HISTOGRAM_MAX; i++) {
		char name[32];

		snprintf(name, sizeof(name), "histogram_%d", i);
		ret = exynos_drm_crtc_create_blob(crtc, name, &exynos_crtc->props.histogram[i]);
		if (ret) {
			pr_err("%s: create properties(%s): ret %d\n", __func__, name, ret);
			return ret;
		}
	}

	return ret;
}

static int
exynos_drm_crtc_create_partial_property(struct exynos_drm_crtc *exynos_crtc)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct drm_property *prop;

	prop = drm_property_create(crtc->dev, DRM_MODE_PROP_BLOB,
			"partial_region", 0);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, prop, 0);
	exynos_crtc->props.partial = prop;

	return 0;
}

static int exynos_drm_crtc_create_rcd_id_property(struct exynos_drm_crtc *exynos_crtc,
						  u32 rcd_plane_id)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct drm_property *prop;

	prop = drm_property_create_range(crtc->dev, DRM_MODE_PROP_IMMUTABLE, "rcd_plane_id", 0,
					 UINT_MAX);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&crtc->base, prop, rcd_plane_id);
	exynos_crtc->props.rcd_plane_id = prop;

	return 0;
}

struct exynos_drm_crtc *exynos_drm_crtc_create(struct drm_device *drm_dev,
					struct drm_plane *plane,
					enum exynos_drm_output_type type,
					const struct exynos_drm_crtc_ops *ops,
					void *ctx)
{
	struct exynos_drm_crtc *exynos_crtc;
	struct drm_crtc *crtc;
	const struct decon_device *decon = ctx;
	int ret;

	exynos_crtc =
		drmm_crtc_alloc_with_planes(drm_dev, struct exynos_drm_crtc, base, plane, NULL,
					    &exynos_crtc_funcs, "exynos-crtc-%u", decon->id);
	if (IS_ERR(exynos_crtc)) {
		ret = PTR_ERR(exynos_crtc);
		goto err_crtc;
	}
	exynos_crtc->possible_type = type;
	exynos_crtc->ops = ops;
	exynos_crtc->ctx = ctx;
	exynos_crtc->active_state = CRTC_STATE_INACTIVE;

	crtc = &exynos_crtc->base;

	drm_crtc_helper_add(crtc, &exynos_crtc_helper_funcs);

	ret = exynos_drm_crtc_create_color_mode_property(exynos_crtc);
	if (ret)
		goto err_crtc;

	ret = exynos_drm_crtc_create_force_bpc_property(exynos_crtc);
	if (ret)
		goto err_crtc;

	ret = exynos_drm_crtc_create_range(crtc, "ppc", &exynos_crtc->props.ppc,
			0, UINT_MAX);
	if (ret)
		goto err_crtc;

	ret = exynos_drm_crtc_create_range(crtc, "max_disp_freq",
			&exynos_crtc->props.max_disp_freq, 0, UINT_MAX);
	if (ret)
		goto err_crtc;

	if (decon->dqe) {
		ret = exynos_drm_crtc_create_blob(crtc, "disp_dither",
				&exynos_crtc->props.disp_dither);
		if (ret)
			goto err_crtc;

		ret = exynos_drm_crtc_create_blob(crtc, "cgc_dither",
				&exynos_crtc->props.cgc_dither);
		if (ret)
			goto err_crtc;

		drm_crtc_enable_color_mgmt(crtc, DEGAMMA_LUT_SIZE, false,
				REGAMMA_LUT_SIZE);

		ret = exynos_drm_crtc_create_blob(crtc, "linear_matrix",
				&exynos_crtc->props.linear_matrix);
		if (ret)
			goto err_crtc;

		ret = exynos_drm_crtc_create_blob(crtc, "gamma_matrix",
				&exynos_crtc->props.gamma_matrix);
		if (ret)
			goto err_crtc;

		ret = exynos_drm_crtc_create_bool(crtc, "dqe_enabled",
				&exynos_crtc->props.dqe_enabled);
		if (ret)
			goto err_crtc;

		ret = exynos_drm_crtc_create_histogram_properties(exynos_crtc);
		if (ret)
			goto err_crtc;

		if (decon->cgc_dma) {
			ret = exynos_drm_crtc_create_signed_range(crtc, "cgc_lut_fd",
					&exynos_crtc->props.cgc_lut_fd, INT_MIN, INT_MAX);
			if (ret)
				goto err_crtc;
		} else {
			ret = exynos_drm_crtc_create_blob(crtc, "cgc_lut",
					&exynos_crtc->props.cgc_lut);
		}
		if (ret)
			goto err_crtc;
	}

	if (decon->rcd) {
		ret = exynos_drm_crtc_create_rcd_id_property(exynos_crtc,
							     decon->rcd->plane.base.base.id);
		if (ret)
			goto err_crtc;
	}

	ret = exynos_drm_crtc_create_partial_property(exynos_crtc);
	if (ret)
		goto err_crtc;

	if (exynos_drm_crtc_create_range(crtc, "expected_present_time",
		&exynos_crtc->props.expected_present_time, 0, (uint64_t)(~((uint64_t)0))))
		pr_err("create drm property expected_present_time failed\n");

	return exynos_crtc;

err_crtc:
	return ERR_PTR(ret);
}

uint32_t exynos_drm_get_possible_crtcs(const struct drm_encoder *encoder,
		enum exynos_drm_output_type out_type)
{
	const struct drm_crtc *crtc;
	uint32_t possible_crtcs = 0;

	drm_for_each_crtc(crtc, encoder->dev) {
		if (to_exynos_crtc(crtc)->possible_type & out_type)
			possible_crtcs |= drm_crtc_mask(crtc);
	}

	return possible_crtcs;
}

void exynos_drm_crtc_te_handler(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	if (exynos_crtc->ops->te_handler)
		exynos_crtc->ops->te_handler(exynos_crtc);
}

void exynos_crtc_wait_for_flip_done(struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	int i;

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state, new_crtc_state, i) {
		struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

		if (exynos_crtc->ops->wait_for_flip_done)
			exynos_crtc->ops->wait_for_flip_done(exynos_crtc,
							old_crtc_state, new_crtc_state);
	}
}

bool exynos_crtc_needs_disable(struct drm_crtc_state *old_state,
			struct drm_crtc_state *new_state)
{
	/*
	 * No new_state means the CRTC is off, so the only criteria is whether
	 * it's currently active or in self refresh mode.
	 */
	if (!new_state)
		return drm_atomic_crtc_effectively_active(old_state);

	/*
	 * We need to disable bridge(s) and CRTC if we're transitioning out of
	 * self-refresh and changing CRTCs at the same time, because the
	 * bridge tracks self-refresh status via CRTC state.
	 */
	if (old_state->self_refresh_active && old_state->crtc != new_state->crtc)
		return true;

	/*
	 * We also need to run through the crtc_funcs->disable() function if
	 * the CRTC is currently on, if it's transitioning to self refresh
	 * mode, or if it's in self refresh mode and needs to be fully
	 * disabled.
	 */
	return old_state->active ||
			(old_state->self_refresh_active && !new_state->enable) ||
			new_state->self_refresh_active;
}

void exynos_crtc_set_mode(struct drm_device *dev,
			struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *new_conn_state;
	int i;

	for_each_new_crtc_in_state(old_state, crtc, new_crtc_state, i) {
		const struct drm_crtc_helper_funcs *funcs;

		if (!new_crtc_state->mode_changed)
			continue;

		funcs = crtc->helper_private;

		if (new_crtc_state->enable && funcs->mode_set_nofb) {
			DRM_DEBUG_ATOMIC("modeset on [CRTC:%u:%s]\n",
					crtc->base.id, crtc->name);

			funcs->mode_set_nofb(crtc);
		}
	}

	for_each_new_connector_in_state(old_state, connector, new_conn_state, i) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_encoder *encoder;
		struct drm_display_mode *mode, *adjusted_mode;
		struct drm_bridge *bridge;

		if (!new_conn_state->best_encoder)
			continue;

		encoder = new_conn_state->best_encoder;
		funcs = encoder->helper_private;
		new_crtc_state = new_conn_state->crtc->state;
		mode = &new_crtc_state->mode;
		adjusted_mode = &new_crtc_state->adjusted_mode;

		if (!new_crtc_state->mode_changed)
			continue;

		DRM_DEBUG_ATOMIC("modeset on [ENCODER:%u:%s]\n",
				encoder->base.id, encoder->name);

		/*
		 * Each encoder has at most one connector (since we always steal
		 * it away), so we won't call mode_set hooks twice.
		 */
		if (funcs && funcs->atomic_mode_set) {
			funcs->atomic_mode_set(encoder, new_crtc_state,
							new_conn_state);
		} else if (funcs && funcs->mode_set) {
			funcs->mode_set(encoder, mode, adjusted_mode);
		}

		bridge = drm_bridge_chain_get_first_bridge(encoder);
		drm_bridge_chain_mode_set(bridge, mode, adjusted_mode);
	}
}
