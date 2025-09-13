/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_format.h"

static inline struct exynos_drm_connector_properties *
dev_get_exynos_connector_properties(struct drm_device *dev)
{
	struct exynos_drm_private *priv = drm_to_exynos_dev(dev);

	return &priv->connector_props;
}

int exynos_drm_connector_set_lhbm_hist(struct exynos_drm_connector *conn, int w, int h, int d,
				       int r)
{
	/* calculate ROI rectangle side length based on b/259177174 */
	int half_side_len = mult_frac(r, 1000, 1414);
	int x = (w / 2) - half_side_len;
	int y = (h / 2) + d - half_side_len;

	return exynos_drm_drv_set_lhbm_hist(conn, x, y, 2 * half_side_len, 2 * half_side_len);
}
EXPORT_SYMBOL_GPL(exynos_drm_connector_set_lhbm_hist);

int exynos_drm_connector_get_lhbm_gray_level(struct exynos_drm_connector *conn)
{
	return exynos_drm_drv_get_lhbm_gray_level(conn);
}
EXPORT_SYMBOL_GPL(exynos_drm_connector_get_lhbm_gray_level);

struct exynos_drm_connector_properties *
exynos_drm_connector_get_properties(struct exynos_drm_connector *exynos_connector)
{
	return dev_get_exynos_connector_properties(exynos_connector->base.dev);
}
EXPORT_SYMBOL_GPL(exynos_drm_connector_get_properties);

static void exynos_drm_connector_destroy(struct drm_connector *connector)
{
	sysfs_remove_link(&connector->kdev->kobj, "panel");

	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static void exynos_drm_connector_destroy_state(struct drm_connector *connector,
					  struct drm_connector_state *connector_state)
{
	struct exynos_drm_connector_state *exynos_connector_state;

	exynos_connector_state = to_exynos_connector_state(connector_state);
	__drm_atomic_helper_connector_destroy_state(connector_state);
	kfree(exynos_connector_state);
}

static void exynos_drm_connector_reset(struct drm_connector *connector)
{
	struct exynos_drm_connector_state *exynos_connector_state;

	if (connector->state) {
		exynos_drm_connector_destroy_state(connector, connector->state);
		connector->state = NULL;
	}

	exynos_connector_state = kzalloc(sizeof(*exynos_connector_state), GFP_KERNEL);
	if (exynos_connector_state) {
		connector->state = &exynos_connector_state->base;
		connector->state->connector = connector;
	} else {
		DRM_ERROR("failed to allocate exynos connector state\n");
	}
}

static struct drm_connector_state *
exynos_drm_connector_duplicate_state(struct drm_connector *connector)
{
	struct exynos_drm_connector_state *exynos_connector_state;
	struct exynos_drm_connector_state *copy;

	exynos_connector_state = to_exynos_connector_state(connector->state);
	copy = kmemdup(exynos_connector_state, sizeof(*exynos_connector_state), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_connector_duplicate_state(connector, &copy->base);

	/* clear pending update */
	copy->pending_update_flags = 0;

	copy->mipi_sync = MIPI_CMD_SYNC_NONE;

	return &copy->base;
}

static int exynos_drm_connector_get_property(struct drm_connector *connector,
					     const struct drm_connector_state *connector_state,
					     struct drm_property *property,
					     uint64_t *val)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	const struct exynos_drm_connector_state *exynos_connector_state =
		to_exynos_connector_state(connector_state);
	const struct exynos_drm_connector_funcs *funcs = exynos_connector->funcs;

	if (funcs && funcs->atomic_get_property)
		return funcs->atomic_get_property(exynos_connector, exynos_connector_state,
						  property, val);

	return -EINVAL;
}

static int exynos_drm_connector_set_property(struct drm_connector *connector,
					     struct drm_connector_state *connector_state,
					     struct drm_property *property,
					     uint64_t val)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	struct exynos_drm_connector_state *exynos_connector_state =
		to_exynos_connector_state(connector_state);
	const struct exynos_drm_connector_funcs *funcs = exynos_connector->funcs;

	if (funcs && funcs->atomic_set_property)
		return funcs->atomic_set_property(exynos_connector, exynos_connector_state,
						  property, val);

	return -EINVAL;
}

static void exynos_drm_connector_print_state(struct drm_printer *p,
					     const struct drm_connector_state *state)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(state->connector);
	const struct exynos_drm_connector_state *exynos_connector_state =
		to_exynos_connector_state(state);
	const struct exynos_drm_connector_funcs *funcs = exynos_connector->funcs;

	if (funcs && funcs->atomic_print_state)
		funcs->atomic_print_state(p, exynos_connector_state);
}

static int exynos_drm_connector_late_register(struct drm_connector *connector)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	const struct exynos_drm_connector_funcs *funcs = exynos_connector->funcs;

	if (funcs && funcs->late_register)
		return funcs->late_register(exynos_connector);

	return -EINVAL;
}

static const struct drm_connector_funcs exynos_drm_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = exynos_drm_connector_reset,
	.destroy = exynos_drm_connector_destroy,
	.atomic_duplicate_state = exynos_drm_connector_duplicate_state,
	.atomic_destroy_state = exynos_drm_connector_destroy_state,
	.atomic_get_property = exynos_drm_connector_get_property,
	.atomic_set_property = exynos_drm_connector_set_property,
	.atomic_print_state = exynos_drm_connector_print_state,
	.late_register = exynos_drm_connector_late_register,
};

bool is_exynos_drm_connector(const struct drm_connector *connector)
{
	return connector->funcs == &exynos_drm_connector_funcs;
}
EXPORT_SYMBOL_GPL(is_exynos_drm_connector);

int exynos_drm_connector_init(struct drm_device *dev,
			      struct exynos_drm_connector *exynos_connector,
			      const struct exynos_drm_connector_funcs *funcs,
			      const struct exynos_drm_connector_helper_funcs *helper_funcs,
			      int connector_type)
{
	exynos_connector->funcs = funcs;
	exynos_connector->helper_private = helper_funcs;

	return drm_connector_init(dev, &exynos_connector->base,
				 &exynos_drm_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
}
EXPORT_SYMBOL_GPL(exynos_drm_connector_init);

static int exynos_drm_connector_create_brightness_properties(struct drm_device *dev)
{
	struct drm_property *prop;
	struct exynos_drm_connector_properties *p = dev_get_exynos_connector_properties(dev);
	static const struct drm_prop_enum_list hbm_enum_list[] = {
		{ HBM_OFF, "Off" },
		{ HBM_ON_IRC_ON, "On IRC On" },
		{ HBM_ON_IRC_OFF, "On IRC Off" },
	};
	static const struct drm_prop_enum_list mipi_sync_list[] = {
		{ __builtin_ffs(MIPI_CMD_SYNC_NONE) - 1, "sync_none" },
		{ __builtin_ffs(MIPI_CMD_SYNC_REFRESH_RATE) - 1, "sync_refresh_rate" },
		{ __builtin_ffs(MIPI_CMD_SYNC_LHBM) - 1, "sync_lhbm" },
		{ __builtin_ffs(MIPI_CMD_SYNC_GHBM) - 1, "sync_ghbm" },
		{ __builtin_ffs(MIPI_CMD_SYNC_BL) - 1, "sync_bl" },
		{ __builtin_ffs(MIPI_CMD_SYNC_OP_RATE) - 1, "sync_op_rate" },
	};

	prop = drm_property_create(dev, DRM_MODE_PROP_BLOB|DRM_MODE_PROP_IMMUTABLE,
		 "brightness_capability", 0);
	if (!prop) {
		pr_err("create brightness_capability property failed");
		return -ENOMEM;
	}
	p->brightness_capability = prop;

	prop = drm_property_create_enum(dev, 0, "hbm_mode",
				hbm_enum_list, ARRAY_SIZE(hbm_enum_list));
	if (!prop)
		return -ENOMEM;
	p->global_hbm_mode = prop;

	prop = drm_property_create_bool(dev, 0, "local_hbm_mode");
	if (!prop)
		return -ENOMEM;
	p->local_hbm_on = prop;

	prop = drm_property_create_bool(dev, 0, "dimming_on");
	if (!prop)
		return -ENOMEM;
	p->dimming_on = prop;

	prop = drm_property_create_range(dev, 0, "brightness_level", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	p->brightness_level = prop;

	prop = drm_property_create_range(dev, 0, "operation_rate", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	p->operation_rate = prop;

	prop = drm_property_create_bitmask(
		dev, 0, "mipi_sync", mipi_sync_list,
		ARRAY_SIZE(mipi_sync_list),
		MIPI_CMD_SYNC_NONE | MIPI_CMD_SYNC_REFRESH_RATE | MIPI_CMD_SYNC_LHBM |
			MIPI_CMD_SYNC_GHBM | MIPI_CMD_SYNC_BL | MIPI_CMD_SYNC_OP_RATE);
	if (!prop)
		return -ENOMEM;
	p->mipi_sync = prop;

	return 0;
}

static int exynos_drm_connector_create_hdr_formats_property(struct drm_device *dev)
{
	struct exynos_drm_connector_properties *p = dev_get_exynos_connector_properties(dev);

	p->hdr_formats = exynos_create_hdr_formats_drm_property(dev, DRM_MODE_PROP_IMMUTABLE);
	if (!p->hdr_formats)
		return -ENOMEM;

	return 0;
}

static int exynos_drm_connector_create_luminance_properties(struct drm_device *dev)
{
	struct exynos_drm_connector_properties *p = dev_get_exynos_connector_properties(dev);

	p->max_luminance = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE,
						     "max_luminance", 0, UINT_MAX);
	if (!p->max_luminance)
		return -ENOMEM;

	p->max_avg_luminance = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE,
							 "max_avg_luminance", 0, UINT_MAX);
	if (!p->max_avg_luminance)
		return -ENOMEM;

	p->min_luminance = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE,
						     "min_luminance", 0, UINT_MAX);
	if (!p->min_luminance)
		return -ENOMEM;

	return 0;
}

static int exynos_drm_connector_create_orientation_property(struct drm_device *dev)
{
	struct exynos_drm_connector_properties *p = dev_get_exynos_connector_properties(dev);
	static const struct drm_prop_enum_list drm_panel_orientation_enum_list[] = {
		{ DRM_MODE_PANEL_ORIENTATION_NORMAL,	"Normal"	},
		{ DRM_MODE_PANEL_ORIENTATION_BOTTOM_UP,	"Upside Down"	},
		{ DRM_MODE_PANEL_ORIENTATION_LEFT_UP,	"Left Side Up"	},
		{ DRM_MODE_PANEL_ORIENTATION_RIGHT_UP,	"Right Side Up"	},
	};

	p->panel_orientation = drm_property_create_enum(dev, DRM_MODE_PROP_IMMUTABLE,
						"panel orientation",
						drm_panel_orientation_enum_list,
						ARRAY_SIZE(drm_panel_orientation_enum_list));
	if (!p->panel_orientation)
		return -ENOMEM;

	return 0;
}

int exynos_drm_connector_create_properties(struct drm_device *dev)
{
	struct exynos_drm_connector_properties *p = dev_get_exynos_connector_properties(dev);
	int ret;

	p->lp_mode = drm_property_create(dev, DRM_MODE_PROP_BLOB, "lp_mode", 0);
	if (!p->lp_mode)
		return -ENOMEM;

	p->is_partial = drm_property_create_bool(dev, DRM_MODE_PROP_IMMUTABLE,
			"is_partial");
	if (!p->is_partial)
		return -ENOMEM;

	p->panel_idle_support = drm_property_create_bool(dev, DRM_MODE_PROP_IMMUTABLE,
			"panel_idle_support");
	if (!p->panel_idle_support)
		return -ENOMEM;

	p->rr_switch_duration = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE,
						"rr_switch_duration", 0, UINT_MAX);
	if (!p->rr_switch_duration)
		return -ENOMEM;

	p->refresh_on_lp = drm_property_create_bool(dev, DRM_MODE_PROP_IMMUTABLE, "refresh_on_lp");
	if (!p->refresh_on_lp)
		return -ENOMEM;

	ret = exynos_drm_connector_create_luminance_properties(dev);
	if (ret)
		return ret;

	ret = exynos_drm_connector_create_brightness_properties(dev);
	if (ret)
		return ret;

	ret = exynos_drm_connector_create_orientation_property(dev);
	if (ret)
		return ret;

	return exynos_drm_connector_create_hdr_formats_property(dev);
}

int exynos_drm_mode_te_freq(const struct drm_display_mode *mode)
{
	int freq = drm_mode_vrefresh(mode);

	if (mode->vscan > 1)
		return freq * mode->vscan;

	return freq;
}
EXPORT_SYMBOL_GPL(exynos_drm_mode_te_freq);

int exynos_drm_mode_bts_fps(const struct drm_display_mode *mode, unsigned int min_bts_fps)
{
	int bts_fps = drm_mode_vrefresh(mode);

	if (!bts_fps) {
		bts_fps = 120;
		pr_warn("invalid drm mode, use default bts_fps=120\n");
	}
	return (min_bts_fps > bts_fps) ? min_bts_fps : bts_fps;
}
EXPORT_SYMBOL_GPL(exynos_drm_mode_bts_fps);

int exynos_bts_fps_to_drm_mode_clock(const struct drm_display_mode *mode, int bts_fps)
{
	/* TOD: calculate required mode clock correctly */
	return DIV_ROUND_UP(mode->htotal * mode->vtotal * bts_fps, 1000);
}
EXPORT_SYMBOL_GPL(exynos_bts_fps_to_drm_mode_clock);
