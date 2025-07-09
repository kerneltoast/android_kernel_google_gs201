/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include "gs_drm/gs_drm_connector.h"

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#define HDR_DOLBY_VISION BIT(1)
#define HDR_HDR10 BIT(2)
#define HDR_HLG BIT(3)

#define DSIM_LABEL_LEN 5
#define PANEL_ID_LENGTH 4
#define LEGACY_PANEL_ID_LENGTH 3
#define PANEL_DRV_LEN 64

#define HOST_PORT 0
#define HOST_ENDPOINT 0

#ifndef DISPLAY_PANEL_INDEX_PRIMARY
#define DISPLAY_PANEL_INDEX_PRIMARY 0
#endif
#ifndef DISPLAY_PANEL_INDEX_SECONDARY
#define DISPLAY_PANEL_INDEX_SECONDARY 1
#endif

static char panel_name[PANEL_DRV_LEN] = "panel-gs-simple";
module_param_string(panel_name, panel_name, sizeof(panel_name), 0644);
MODULE_PARM_DESC(panel_name, "preferred panel name");
static char sec_panel_name[PANEL_DRV_LEN] = "panel-gs-simple";
module_param_string(sec_panel_name, sec_panel_name, sizeof(sec_panel_name), 0644);
MODULE_PARM_DESC(sec_panel_name, "preferred panel name for secondary panel");

int gs_drm_mode_bts_fps(const struct drm_display_mode *mode, unsigned int min_bts_fps)
{
	unsigned int bts_fps = drm_mode_vrefresh(mode);

	return (min_bts_fps > bts_fps) ? min_bts_fps : bts_fps;
}
EXPORT_SYMBOL_GPL(gs_drm_mode_bts_fps);

int gs_bts_fps_to_drm_mode_clock(const struct drm_display_mode *mode, int bts_fps)
{
	int clock = DIV_ROUND_UP(mode->htotal * mode->vtotal * bts_fps, 1000);

	if (mode->vscan > 1)
		clock *= mode->vscan;
	return clock;
}
EXPORT_SYMBOL_GPL(gs_bts_fps_to_drm_mode_clock);

struct gs_drm_connector_properties *
gs_drm_connector_get_properties(struct gs_drm_connector *gs_connector)
{
	return &gs_connector->properties;
}
EXPORT_SYMBOL_GPL(gs_drm_connector_get_properties);

void gs_connector_set_panel_name(const char *new_name, size_t len, int idx)
{
	switch (idx) {
	case DISPLAY_PANEL_INDEX_PRIMARY:
		strscpy(panel_name, new_name, sizeof(panel_name));
		break;
	case DISPLAY_PANEL_INDEX_SECONDARY:
		strscpy(sec_panel_name, new_name, sizeof(sec_panel_name));
		break;
	default:
		pr_warn("Unsupported panel index %d\n", idx);
	}
}
EXPORT_SYMBOL_GPL(gs_connector_set_panel_name);

static void gs_drm_connector_destroy(struct drm_connector *connector)
{
	sysfs_remove_link(&connector->kdev->kobj, "panel");

	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static void gs_drm_connector_destroy_state(struct drm_connector *connector,
					   struct drm_connector_state *connector_state)
{
	struct gs_drm_connector_state *gs_connector_state;

	gs_connector_state = to_gs_connector_state(connector_state);
	__drm_atomic_helper_connector_destroy_state(connector_state);
	kfree(gs_connector_state);
}

static void gs_drm_connector_reset(struct drm_connector *connector)
{
	struct gs_drm_connector_state *gs_connector_state;

	dev_dbg(connector->kdev, "%s+\n", __func__);

	if (connector->state) {
		gs_drm_connector_destroy_state(connector, connector->state);
		connector->state = NULL;
	}

	gs_connector_state = kzalloc(sizeof(*gs_connector_state), GFP_KERNEL);
	if (gs_connector_state) {
		connector->state = &gs_connector_state->base;
		connector->state->connector = connector;
	} else {
		DRM_ERROR("failed to allocate gs connector state\n");
	}
	dev_dbg(connector->kdev, "%s-\n", __func__);
}

static struct drm_connector_state *gs_drm_connector_duplicate_state(struct drm_connector *connector)
{
	struct gs_drm_connector_state *gs_connector_state;
	struct gs_drm_connector_state *copy;

	gs_connector_state = to_gs_connector_state(connector->state);
	copy = kmemdup(gs_connector_state, sizeof(*gs_connector_state), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_connector_duplicate_state(connector, &copy->base);

	/* clear pending update */
	copy->pending_update_flags = 0;

	/* clear frame_interval us */
	copy->frame_interval_us = 0;

	copy->mipi_sync = GS_MIPI_CMD_SYNC_NONE;

	return &copy->base;
}

static int gs_drm_connector_get_property(struct drm_connector *connector,
					 const struct drm_connector_state *connector_state,
					 struct drm_property *property, uint64_t *val)
{
	struct gs_drm_connector *gs_connector = to_gs_connector(connector);
	const struct gs_drm_connector_state *gs_connector_state =
		to_gs_connector_state(connector_state);
	const struct gs_drm_connector_funcs *funcs = gs_connector->funcs;

	if (funcs && funcs->atomic_get_property)
		return funcs->atomic_get_property(gs_connector, gs_connector_state, property, val);

	return -EINVAL;
}

static int gs_drm_connector_set_property(struct drm_connector *connector,
					 struct drm_connector_state *connector_state,
					 struct drm_property *property, uint64_t val)
{
	struct gs_drm_connector *gs_connector = to_gs_connector(connector);
	struct gs_drm_connector_state *gs_connector_state = to_gs_connector_state(connector_state);
	const struct gs_drm_connector_funcs *funcs = gs_connector->funcs;

	if (funcs && funcs->atomic_set_property)
		return funcs->atomic_set_property(gs_connector, gs_connector_state, property, val);

	return -EINVAL;
}

static void gs_drm_connector_print_state(struct drm_printer *p,
					 const struct drm_connector_state *state)
{
	struct gs_drm_connector *gs_connector = to_gs_connector(state->connector);
	const struct gs_drm_connector_state *gs_connector_state = to_gs_connector_state(state);
	const struct gs_drm_connector_funcs *funcs = gs_connector->funcs;

	if (funcs && funcs->atomic_print_state)
		funcs->atomic_print_state(p, gs_connector_state);
}

static int gs_drm_connector_late_register(struct drm_connector *connector)
{
	struct gs_drm_connector *gs_connector = to_gs_connector(connector);
	const struct gs_drm_connector_funcs *funcs = gs_connector->funcs;

	if (funcs && funcs->late_register)
		return funcs->late_register(gs_connector);

	return -EINVAL;
}

static const struct drm_connector_funcs base_drm_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = gs_drm_connector_reset,
	.destroy = gs_drm_connector_destroy,
	.atomic_duplicate_state = gs_drm_connector_duplicate_state,
	.atomic_destroy_state = gs_drm_connector_destroy_state,
	.atomic_get_property = gs_drm_connector_get_property,
	.atomic_set_property = gs_drm_connector_set_property,
	.atomic_print_state = gs_drm_connector_print_state,
	.late_register = gs_drm_connector_late_register,
};

bool is_gs_drm_connector(const struct drm_connector *connector)
{
	return connector->funcs == &base_drm_connector_funcs;
}
EXPORT_SYMBOL_GPL(is_gs_drm_connector);

static int gs_drm_connector_create_brightness_properties(struct gs_drm_connector *gs_connector)
{
	struct drm_property *prop;
	struct drm_device *dev = gs_connector->base.dev;
	struct gs_drm_connector_properties *p = gs_drm_connector_get_properties(gs_connector);
	static const struct drm_prop_enum_list hbm_enum_list[] = {
		{ GS_HBM_OFF, "Off" },
		{ GS_HBM_ON_IRC_ON, "On IRC On" },
		{ GS_HBM_ON_IRC_OFF, "On IRC Off" },
	};
	static const struct drm_prop_enum_list mipi_sync_list[] = {
		{ __builtin_ffs(GS_MIPI_CMD_SYNC_NONE) - 1, "sync_none" },
		{ __builtin_ffs(GS_MIPI_CMD_SYNC_REFRESH_RATE) - 1, "sync_refresh_rate" },
		{ __builtin_ffs(GS_MIPI_CMD_SYNC_LHBM) - 1, "sync_lhbm" },
		{ __builtin_ffs(GS_MIPI_CMD_SYNC_GHBM) - 1, "sync_ghbm" },
		{ __builtin_ffs(GS_MIPI_CMD_SYNC_BL) - 1, "sync_bl" },
		{ __builtin_ffs(GS_MIPI_CMD_SYNC_OP_RATE) - 1, "sync_op_rate" },
	};

	prop = drm_property_create(dev, DRM_MODE_PROP_BLOB | DRM_MODE_PROP_IMMUTABLE,
				   "brightness_capability", 0);
	if (!prop) {
		pr_err("create brightness_capability property failed");
		return -ENOMEM;
	}
	p->brightness_capability = prop;

	prop = drm_property_create_enum(dev, 0, "hbm_mode", hbm_enum_list,
					ARRAY_SIZE(hbm_enum_list));
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
		dev, 0, "mipi_sync", mipi_sync_list, ARRAY_SIZE(mipi_sync_list),
		GS_MIPI_CMD_SYNC_NONE | GS_MIPI_CMD_SYNC_REFRESH_RATE | GS_MIPI_CMD_SYNC_LHBM |
			GS_MIPI_CMD_SYNC_GHBM | GS_MIPI_CMD_SYNC_BL | GS_MIPI_CMD_SYNC_OP_RATE);
	if (!prop)
		return -ENOMEM;
	p->mipi_sync = prop;

	return 0;
}

static int gs_drm_connector_create_hdr_formats_property(struct gs_drm_connector *gs_connector)
{
	static const struct drm_prop_enum_list props[] = {
		{ __builtin_ffs(HDR_DOLBY_VISION) - 1, "Dolby Vision" },
		{ __builtin_ffs(HDR_HDR10) - 1, "HDR10" },
		{ __builtin_ffs(HDR_HLG) - 1, "HLG" },
	};
	struct drm_device *dev = gs_connector->base.dev;
	struct gs_drm_connector_properties *p = gs_drm_connector_get_properties(gs_connector);

	p->hdr_formats = drm_property_create_bitmask(dev, DRM_MODE_PROP_IMMUTABLE, "hdr_formats",
						     props, ARRAY_SIZE(props),
						     HDR_DOLBY_VISION | HDR_HDR10 | HDR_HLG);
	if (!p->hdr_formats)
		return -ENOMEM;

	return 0;
}

static int gs_drm_connector_create_luminance_properties(struct gs_drm_connector *gs_connector)
{
	struct drm_device *dev = gs_connector->base.dev;
	struct gs_drm_connector_properties *p = gs_drm_connector_get_properties(gs_connector);

	p->max_luminance = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE, "max_luminance",
						     0, UINT_MAX);
	if (!p->max_luminance)
		return -ENOMEM;

	p->max_avg_luminance = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE,
							 "max_avg_luminance", 0, UINT_MAX);
	if (!p->max_avg_luminance)
		return -ENOMEM;

	p->min_luminance = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE, "min_luminance",
						     0, UINT_MAX);
	if (!p->min_luminance)
		return -ENOMEM;

	return 0;
}

static int gs_drm_connector_create_orientation_property(struct gs_drm_connector *gs_connector)
{
	struct drm_device *dev = gs_connector->base.dev;
	struct gs_drm_connector_properties *p = gs_drm_connector_get_properties(gs_connector);
	static const struct drm_prop_enum_list drm_panel_orientation_enum_list[] = {
		{ DRM_MODE_PANEL_ORIENTATION_NORMAL, "Normal" },
		{ DRM_MODE_PANEL_ORIENTATION_BOTTOM_UP, "Upside Down" },
		{ DRM_MODE_PANEL_ORIENTATION_LEFT_UP, "Left Side Up" },
		{ DRM_MODE_PANEL_ORIENTATION_RIGHT_UP, "Right Side Up" },
	};

	p->panel_orientation = drm_property_create_enum(
		dev, DRM_MODE_PROP_IMMUTABLE, "panel orientation", drm_panel_orientation_enum_list,
		ARRAY_SIZE(drm_panel_orientation_enum_list));
	if (!p->panel_orientation)
		return -ENOMEM;

	return 0;
}

int gs_drm_connector_create_properties(struct drm_connector *connector)
{
	struct gs_drm_connector *gs_connector = to_gs_connector(connector);
	struct drm_device *drm_dev = connector->dev;
	struct device *dev = gs_connector->kdev;
	struct gs_drm_connector_properties *p = &gs_connector->properties;
	int ret;

	dev_dbg(dev, "%s+\n", __func__);

	p->lp_mode = drm_property_create(drm_dev, DRM_MODE_PROP_BLOB, "lp_mode", 0);
	if (!p->lp_mode)
		return -ENOMEM;

	p->is_partial = drm_property_create_bool(drm_dev, DRM_MODE_PROP_IMMUTABLE, "is_partial");
	if (!p->is_partial)
		return -ENOMEM;

	p->panel_idle_support =
		drm_property_create_bool(drm_dev, DRM_MODE_PROP_IMMUTABLE, "panel_idle_support");
	if (!p->panel_idle_support)
		return -ENOMEM;

	p->rr_switch_duration = drm_property_create_range(drm_dev, DRM_MODE_PROP_IMMUTABLE,
							  "rr_switch_duration", 0, UINT_MAX);
	if (!p->rr_switch_duration)
		return -ENOMEM;

	p->refresh_on_lp =
		drm_property_create_bool(drm_dev, DRM_MODE_PROP_IMMUTABLE, "refresh_on_lp");
	if (!p->refresh_on_lp)
		return -ENOMEM;

	ret = gs_drm_connector_create_luminance_properties(gs_connector);
	if (ret)
		return ret;

	ret = gs_drm_connector_create_brightness_properties(gs_connector);
	if (ret)
		return ret;

	ret = gs_drm_connector_create_orientation_property(gs_connector);
	if (ret)
		return ret;

	ret = gs_drm_connector_create_hdr_formats_property(gs_connector);
	if (ret)
		return ret;

	p->frame_interval = drm_property_create_range(drm_dev, 0, "frame_interval", 0, UINT_MAX);
	if (!p->frame_interval)
		return -ENOMEM;

	dev_dbg(dev, "%s-\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(gs_drm_connector_create_properties);

/* LHBM Histogram and Gray Level */

void gs_drm_connector_update_gray_level_callback(struct drm_connector *connector, int gray_level)
{
	struct gs_drm_connector *gs_connector = to_gs_connector(connector);

	gs_connector->lhbm_gray_level = gray_level;
	dev_dbg(gs_connector->kdev, "gs_drm_connector set gray_level to %d\n", gray_level);
}
EXPORT_SYMBOL_GPL(gs_drm_connector_update_gray_level_callback);

/* Component Model Functions */

int gs_connector_bind(struct device *dev, struct device *master, void *data)
{
	struct gs_drm_connector *gs_connector = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret = 0;

	dev_dbg(dev, "%s+\n", __func__);

	/* Store some data for later initialization on the panel side */

	gs_connector->base.dev = drm_dev;
	gs_connector->base.funcs = &base_drm_connector_funcs;

	/* Create properties */

	gs_drm_connector_create_properties(&gs_connector->base);

	dev_dbg(dev, "%s-\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(gs_connector_bind);

/* Parsing */

static struct device_node *
gs_drm_connector_find_host_node(const struct gs_drm_connector *gs_connector, int port, int endpoint)
{
	struct device *dev;
	struct device_node *remote;

	if (!gs_connector)
		return ERR_PTR(-EINVAL);
	dev = gs_connector->kdev;
	if (!dev)
		return ERR_PTR(-EINVAL);

	remote = of_graph_get_remote_node(dev->of_node, port, endpoint);
	return remote;
}

static int connector_add_mipi_dsi_device(struct gs_drm_connector *gs_connector, const char *pname)
{
	struct mipi_dsi_device_info info = {
		.node = NULL,
		.channel = 1,
	};
	struct device_node *node;
	struct device *dev = gs_connector->kdev;
	struct mipi_dsi_host *host;
	const char *node_label;
	const char *p;
	u32 cmp_len;

	p = strchr(pname, '.');
	cmp_len = p ? min((ptrdiff_t)PANEL_DRV_LEN, p - pname) : strnlen(pname, PANEL_DRV_LEN);

	dev_dbg(dev, "%s+ Preferred panel %s\n", __func__, pname);

	/* Get mipi_dsi_host */
	host = gs_connector->dsi_host_device;

	/* Search for matching child node */
	for_each_available_child_of_node(dev->of_node, node) {
		bool found;

		if (info.node) {
			continue;
		}

		if (of_property_read_u32(node, "channel", &info.channel)) {
			/* if cannot read channel, continue */
			continue;
		}

		node_label = of_get_property(node, "label", NULL);
		if (!node_label)
			continue;
		found = !strncmp(node_label, pname, cmp_len);
		if (found) {
			/*TODO(tknelms): case for no pname provided */
			strscpy(info.type, node_label, sizeof(info.type));
			info.node = of_node_get(node);
		}
	}

	if (info.node) {
		mipi_dsi_device_register_full(host, &info);
		dev_dbg(dev, "%s-\n", __func__);
		return 0;
	} else {
		dev_err(dev, "Unable to find panel matching name %s\n", pname);
		return -ENODEV;
	}
}

static const char *get_dsim_label(const struct gs_drm_connector *gs_connector)
{
	static const char *dsim_label;
	struct device *dev = gs_connector->kdev;
	struct device_node *parent_node =
		gs_drm_connector_find_host_node(gs_connector, HOST_PORT, HOST_ENDPOINT);

	if (IS_ERR_OR_NULL(parent_node)) {
		dev_warn(dev, "Invalid parent_node %p\n", parent_node);
		return NULL;
	}

	if (of_property_read_string(parent_node, "label", &dsim_label)) {
		dev_warn(dev, "No label property found for dsim\n");
		dsim_label = NULL;
	}

	of_node_put(parent_node);
	return dsim_label;
}

/*
 * Parses the bootloader-provided name in the form "dsimX:preferred_panel",
 * compares "dsimX" with the label of the connector's parent DT entry.
 * Returns NULL if not a match, or "preferred_panel" if there is a match.
 */
static const char *get_panel_name(struct gs_drm_connector *gs_connector, const char *name)
{
	/* if ":" not in name, return entire name */
	const char *p = strchr(name, ':');
	const char *dsim_label;
	int len;

	if (!p)
		return name;

	dsim_label = get_dsim_label(gs_connector);
	if (dsim_label) {
		len = p - name;
		if ((len != strlen(dsim_label)) || (strncmp(name, dsim_label, len)))
			return NULL;
		if (name[DSIM_LABEL_LEN] != ':')
			return NULL;
	}

	return (p + 1);
}

/**
 * dsim_get_panel_id() - Parses the panel_id string at the end of name
 * @name: name string to parse
 *
 * The panel name string parsed by the bootloader may be in the form of
 * "panel_name.panel_id", where panel_id is a 6- or 8-character hex string. This
 * function parses that string into an integer.
 *
 * Return: 32-bit integer representing the panel_id, or INVALID_PANEL_ID if
 *         invalid or missing
 */
static u32 dsim_get_panel_id(const char *name)
{
	char *p = strchr(name, '.');
	u8 panel_id[PANEL_ID_LENGTH] = { 0 };

	/*
	 * if period is found, expect 6 or 8 hex characters (ex. panel_name.000000),
	 * otherwise return an invalid panel ID
	 */
	if (!p)
		return INVALID_PANEL_ID;

	p++;

	if ((strlen(p) == (PANEL_ID_LENGTH * 2) && !hex2bin(panel_id, p, PANEL_ID_LENGTH)) ||
	    (strlen(p) == (LEGACY_PANEL_ID_LENGTH * 2) &&
	     !hex2bin(panel_id + 1, p, LEGACY_PANEL_ID_LENGTH))) {
		return __builtin_bswap32(*(u32 *)panel_id);
	}

	return INVALID_PANEL_ID;
}

/*
 * Parses panel name supplied by bootloader, matches with child node,
 * adds the appropriate child panel into mipi_dsi system
 */
static int parse_panel_name(struct gs_drm_connector *gs_connector)
{
	const char *pref_panel_name;

	if (gs_connector->panel_index == DISPLAY_PANEL_INDEX_SECONDARY)
		pref_panel_name = get_panel_name(gs_connector, sec_panel_name);
	else
		pref_panel_name = get_panel_name(gs_connector, panel_name);

	if (pref_panel_name && pref_panel_name[0]) {
		gs_connector->panel_id = dsim_get_panel_id(pref_panel_name);

		return connector_add_mipi_dsi_device(gs_connector, pref_panel_name);
	}

	return -ENODEV;
}

/**
 * gs_drm_connector_parse_panel_index() - parses display-index DT entry
 * @gs_connector: Pointer to gs_connector
 *
 * Return: 0 on success, negative value on fatal error
 */
static int gs_drm_connector_parse_panel_index(struct gs_drm_connector *gs_connector)
{
	struct device *dev = gs_connector->kdev;
	struct device_node *node = dev->of_node;
	int ret;

	ret = of_property_read_s32(node, "google,device-index", &gs_connector->panel_index);
	if (ret == -EINVAL) {
		dev_dbg(dev,
			"No value found for \"google,device-index\", defaulting to primary panel\n");
		gs_connector->panel_index = DISPLAY_PANEL_INDEX_PRIMARY;
		ret = 0;
	} else if (ret) {
		dev_warn(dev, "ret value %d while parsing google,device-index; exiting\n", ret);
	}

	return ret;
}

/*
 * Gets the host device info based on port and endpoint,
 * fills in gs_drm_connector struct with correct info
 */
static int gs_drm_connector_find_host(struct gs_drm_connector *gs_connector, int port, int endpoint)
{
	int ret;
	struct device *dev;
	struct device_node *remote;
	struct mipi_dsi_host *host;

	remote = gs_drm_connector_find_host_node(gs_connector, port, endpoint);
	if (IS_ERR(remote))
		return PTR_ERR(remote);

	if (!remote)
		return -ENODEV;

	dev = gs_connector->base.kdev;
	host = of_find_mipi_dsi_host_by_node(remote);
	if (!host) {
		ret = -EPROBE_DEFER;
	} else {
		dev_dbg(dev, "gs_drm_connector host found at %p\n", host);
		gs_connector->dsi_host_device = host;
		ret = 0;
	}

	of_node_put(remote);
	return ret;
}

/* Device/Module Functions */

static int gs_drm_connector_probe(struct platform_device *pdev)
{
	struct gs_drm_connector *gs_connector;
	struct device *dev = &pdev->dev;
	int ret;

	dev_dbg(dev, "%s+\n", __func__);

	gs_connector = devm_kzalloc(dev, sizeof(struct gs_drm_connector), GFP_KERNEL);
	if (!gs_connector)
		return -ENOMEM;

	gs_connector->kdev = dev;

	platform_set_drvdata(pdev, gs_connector);

	ret = gs_drm_connector_find_host(gs_connector, HOST_PORT, HOST_ENDPOINT);
	if (ret)
		return ret;

	ret = gs_drm_connector_parse_panel_index(gs_connector);
	if (ret)
		return ret;

	if (parse_panel_name(gs_connector))
		dev_err(dev, "%s parse_panel_name failed\n", __func__);

	dev_info(dev, "gs_drm_connector successfully probed\n");

	return 0;
};

static int gs_drm_connector_remove(struct platform_device *pdev)
{
	struct gs_drm_connector *gs_connector = platform_get_drvdata(pdev);
	(void)gs_connector; /*TODO(tknelms): use*/

	return 0;
};

static const struct of_device_id gs_connector_of_match[] = {
	{
		.compatible = "google,drm_connector",
	},
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, gs_connector_of_match);

static struct platform_driver gs_drm_connector_driver = {
	.probe = gs_drm_connector_probe,
	.remove = gs_drm_connector_remove,
	.driver = {
		.name = "gs-drm-connector",
		.owner = THIS_MODULE,
		.of_match_table = gs_connector_of_match,
	},
};
module_platform_driver(gs_drm_connector_driver);

MODULE_AUTHOR("Taylor Nelms <tknelms@google.com>");
MODULE_DESCRIPTION("DRM dpu connector abstraction");
MODULE_LICENSE("Dual MIT/GPL");
