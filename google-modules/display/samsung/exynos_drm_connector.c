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
#include "exynos_drm_connector.h"

#define HDR_DOLBY_VISION	BIT(1)
#define HDR_HDR10		BIT(2)
#define HDR_HLG			BIT(3)

static inline struct exynos_drm_connector_properties *
dev_get_exynos_connector_properties(struct drm_device *dev)
{
	struct exynos_drm_private *priv = drm_to_exynos_dev(dev);

	return &priv->connector_props;
}

struct exynos_drm_connector_properties *
exynos_drm_connector_get_properties(struct exynos_drm_connector *exynos_connector)
{
	return dev_get_exynos_connector_properties(exynos_connector->base.dev);
}
EXPORT_SYMBOL(exynos_drm_connector_get_properties);

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

	copy->mipi_sync = 0;

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

static const struct drm_connector_funcs exynos_drm_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = exynos_drm_connector_reset,
	.atomic_duplicate_state = exynos_drm_connector_duplicate_state,
	.atomic_destroy_state = exynos_drm_connector_destroy_state,
	.atomic_get_property = exynos_drm_connector_get_property,
	.atomic_set_property = exynos_drm_connector_set_property,
	.atomic_print_state = exynos_drm_connector_print_state,
};

bool is_exynos_drm_connector(const struct drm_connector *connector)
{
	return connector->funcs == &exynos_drm_connector_funcs;
}

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
EXPORT_SYMBOL(exynos_drm_connector_init);

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
		{ __builtin_ffs(MIPI_CMD_SYNC_REFRESH_RATE) - 1, "sync_refresh_rate" },
		{ __builtin_ffs(MIPI_CMD_SYNC_LHBM) - 1, "sync_lhbm" },
		{ __builtin_ffs(MIPI_CMD_SYNC_GHBM) - 1, "sync_ghbm" },
		{ __builtin_ffs(MIPI_CMD_SYNC_BL) - 1, "sync_bl" },
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

	prop = drm_property_create_bitmask(
		dev, 0, "mipi_sync", mipi_sync_list,
		ARRAY_SIZE(mipi_sync_list),
		MIPI_CMD_SYNC_REFRESH_RATE | MIPI_CMD_SYNC_LHBM | MIPI_CMD_SYNC_GHBM |
				MIPI_CMD_SYNC_BL);
	if (!prop)
		return -ENOMEM;
	p->mipi_sync = prop;

	return 0;
}

static int exynos_drm_connector_create_hdr_formats_property(struct drm_device *dev)
{
	static const struct drm_prop_enum_list props[] = {
		{ __builtin_ffs(HDR_DOLBY_VISION) - 1,	"Dolby Vision"	},
		{ __builtin_ffs(HDR_HDR10) - 1,		"HDR10"		},
		{ __builtin_ffs(HDR_HLG) - 1,		"HLG"		},
	};
	struct exynos_drm_connector_properties *p = dev_get_exynos_connector_properties(dev);

	p->hdr_formats = drm_property_create_bitmask(dev, DRM_MODE_PROP_IMMUTABLE, "hdr_formats",
						     props, ARRAY_SIZE(props),
						     HDR_DOLBY_VISION | HDR_HDR10 | HDR_HLG);
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
	if (!p)
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
	if (IS_ERR(p->lp_mode))
                return PTR_ERR(p->lp_mode);

	p->is_partial = drm_property_create_bool(dev, DRM_MODE_PROP_IMMUTABLE,
			"is_partial");
	if (IS_ERR(p->is_partial))
		return PTR_ERR(p->is_partial);

	p->panel_idle_support = drm_property_create_bool(dev, DRM_MODE_PROP_IMMUTABLE,
			"panel_idle_support");
	if (IS_ERR(p->panel_idle_support))
		return PTR_ERR(p->panel_idle_support);

	p->vrr_switch_duration = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE,
						"vrr_switch_duration", 0, UINT_MAX);
	if (IS_ERR(p->vrr_switch_duration))
		return PTR_ERR(p->vrr_switch_duration);

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
