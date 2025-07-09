/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include "gs_panel/drm_panel_funcs_defaults.h"
#include "gs_panel/gs_panel.h"

#define drm_to_gs_panel(panel) container_of(panel, struct gs_panel, base)

int gs_panel_disable(struct drm_panel *panel)
{
	struct gs_panel *ctx = drm_to_gs_panel(panel);

	dev_dbg(ctx->dev, "%s+\n", __func__);
	ctx->hbm_mode = GS_HBM_OFF;
	ctx->dimming_on = false;
	ctx->idle_data.self_refresh_active = false;
	ctx->idle_data.panel_idle_vrefresh = 0;
	ctx->cabc_mode = GCABC_OFF;
	ctx->ssc_en = false;

	if (ctx->touch_bridge_data.touch_dev && !ctx->touch_bridge_data.attached) {
		dev_warn(ctx->dev,
			 "Panel has DT link to touch bridge but did not attach after %u tries\n",
			 ctx->touch_bridge_data.retry_count);
	}
	ctx->touch_bridge_data.retry_count = 0;

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	gs_panel_disable_normal_feat_locked(ctx);
	gs_panel_send_cmdset(ctx, ctx->desc->off_cmdset);
	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	dev_dbg(ctx->dev, "%s\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(gs_panel_disable);

int gs_panel_unprepare(struct drm_panel *panel)
{
	struct gs_panel *ctx = drm_to_gs_panel(panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	gs_panel_set_power_helper(ctx, false);
	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(gs_panel_unprepare);

int gs_panel_prepare(struct drm_panel *panel)
{
	struct gs_panel *ctx = drm_to_gs_panel(panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	gs_panel_set_power_helper(ctx, true);
	dev_dbg(ctx->dev, "%s -\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(gs_panel_prepare);

static void gs_panel_mode_set_name(struct drm_display_mode *mode)
{
	scnprintf(mode->name, DRM_DISPLAY_MODE_LEN, "%ux%ux%d@%d",
		  mode->hdisplay, mode->vdisplay,
		  drm_mode_vrefresh(mode), gs_drm_mode_te_freq(mode));
}

int gs_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct gs_panel *ctx = drm_to_gs_panel(panel);
	struct drm_display_mode *preferred_mode = NULL;
	int i = 0;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (ctx->desc->modes) {
		for (i = 0; i < ctx->desc->modes->num_modes; i++) {
			const struct gs_panel_mode *pmode = &ctx->desc->modes->modes[i];
			struct drm_display_mode *mode;

			mode = drm_mode_duplicate(connector->dev, &pmode->mode);
			if (!mode)
				return -ENOMEM;

			if (!mode->name[0])
				gs_panel_mode_set_name(mode);

			mode->type |= DRM_MODE_TYPE_DRIVER;
			drm_mode_probed_add(connector, mode);

			dev_dbg(ctx->dev, "added display mode: %s\n", mode->name);

			if (!preferred_mode || (mode->type & DRM_MODE_TYPE_PREFERRED))
				preferred_mode = mode;
		}
	}

	if (preferred_mode) {
		dev_dbg(ctx->dev, "preferred display mode: %s\n", preferred_mode->name);
		preferred_mode->type |= DRM_MODE_TYPE_PREFERRED;
		connector->display_info.width_mm = preferred_mode->width_mm;
		connector->display_info.height_mm = preferred_mode->height_mm;
	}

	dev_dbg(ctx->dev, "%s -\n", __func__);

	return i;
}
EXPORT_SYMBOL_GPL(gs_panel_get_modes);
