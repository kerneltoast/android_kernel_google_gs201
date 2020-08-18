// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based Samsung common panel driver.
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_encoder.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>

#include <panel-samsung-drv.h>

#define PANEL_ID_REG		0xA1
#define PANEL_ID_LEN		7
#define PANEL_ID_OFFSET		6
#define PANEL_ID_READ_SIZE	(PANEL_ID_LEN + PANEL_ID_OFFSET)

static const char ext_info_regs[] = { 0xDA, 0xDB, 0xDC };

#define connector_to_exynos_panel(c)                                           \
	container_of((c), struct exynos_panel, connector)

#define bridge_to_exynos_panel(b) \
	container_of((b), struct exynos_panel, bridge)

static inline bool in_tui(struct exynos_panel *ctx)
{
	const struct drm_connector_state *conn_state = ctx->connector.state;

	if (conn_state && conn_state->crtc) {
		const struct drm_crtc_state *crtc_state =
						conn_state->crtc->state;

		if (crtc_state && (crtc_state->adjusted_mode.private_flags &
				 EXYNOS_DISPLAY_MODE_FLAG_TUI))
			return true;
	}

	return false;
}

static int exynos_panel_parse_gpios(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		dev_info(ctx->dev, "no reset/enable pins on emulator\n");
		return 0;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "failed to get reset-gpios %ld",
				PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		ctx->enable_gpio = NULL;

	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}

static int exynos_panel_parse_regulators(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;

	ctx->vddi = devm_regulator_get(dev, "vddi");
	if (IS_ERR(ctx->vddi)) {
		dev_warn(ctx->dev, "failed to get panel vddi.\n");
		ctx->vddi = NULL;
	}

	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci)) {
		dev_warn(ctx->dev, "failed to get panel vci.\n");
		ctx->vci = NULL;
	}

	return 0;
}

static int exynos_panel_read_id(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[PANEL_ID_READ_SIZE];
	int ret;

	ret = mipi_dsi_dcs_read(dsi, PANEL_ID_REG, buf, PANEL_ID_READ_SIZE);
	if (ret != PANEL_ID_READ_SIZE) {
		dev_warn(ctx->dev, "Unable to read panel id (%d)\n", ret);
		return ret;
	}

	exynos_bin2hex(buf + PANEL_ID_OFFSET, PANEL_ID_LEN,
		       ctx->panel_id, sizeof(ctx->panel_id));

	return 0;
}

static int exynos_panel_read_extinfo(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const size_t extinfo_len = ARRAY_SIZE(ext_info_regs);
	char buf[extinfo_len];
	int i, ret;

	for (i = 0; i < extinfo_len; i++) {
		ret = mipi_dsi_dcs_read(dsi, ext_info_regs[i], buf + i, 1);
		if (ret != 1) {
			dev_warn(ctx->dev,
				 "Unable to read panel extinfo (0x%x: %d)\n",
				 ext_info_regs[i], ret);
			return ret;
		}

	}
	exynos_bin2hex(buf, i, ctx->panel_extinfo, sizeof(ctx->panel_extinfo));

	return 0;
}

static int exynos_panel_init(struct exynos_panel *ctx)
{
	int ret;

	if (ctx->initialized)
		return 0;


	ret = exynos_panel_read_id(ctx);
	if (ret)
		return ret;

	ret = exynos_panel_read_extinfo(ctx);
	if (!ret)
		ctx->initialized = true;

	return ret;
}

void exynos_panel_reset(struct exynos_panel *ctx)
{
	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return;

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);

	dev_dbg(ctx->dev, "%s -\n", __func__);

	exynos_panel_init(ctx);
}
EXPORT_SYMBOL(exynos_panel_reset);

int exynos_panel_set_power(struct exynos_panel *ctx, bool on)
{
	int ret;

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return 0;

	if (!ctx->vddi) {
		dev_dbg(ctx->dev, "trying to get vddi regulator\n");
		ctx->vddi = devm_regulator_get(ctx->dev, "vddi");
		if (IS_ERR(ctx->vddi)) {
			dev_warn(ctx->dev, "failed to get vddi regulator\n");
			ctx->vddi = NULL;
		}
	}

	if (!ctx->vci) {
		dev_dbg(ctx->dev, "trying to get vci regulator\n");
		ctx->vci = devm_regulator_get(ctx->dev, "vci");
		if (IS_ERR(ctx->vci)) {
			dev_warn(ctx->dev, "failed to get vci regulator\n");
			ctx->vci = NULL;
		}
	}

	if (on) {
		if (ctx->enable_gpio) {
			gpiod_set_value(ctx->enable_gpio, 1);
			usleep_range(10000, 11000);
		}

		if (ctx->vddi) {
			ret = regulator_enable(ctx->vddi);
			if (ret) {
				dev_err(ctx->dev, "vddi enable failed\n");
				return ret;
			}
			usleep_range(5000, 6000);
		}

		if (ctx->vci) {
			ret = regulator_enable(ctx->vci);
			if (ret) {
				dev_err(ctx->dev, "vci enable failed\n");
				return ret;
			}
		}
	} else {
		gpiod_set_value(ctx->reset_gpio, 0);
		if (ctx->enable_gpio)
			gpiod_set_value(ctx->enable_gpio, 0);

		if (ctx->vddi) {
			ret = regulator_disable(ctx->vddi);
			if (ret) {
				dev_err(ctx->dev, "vddi disable failed\n");
				return ret;
			}
		}

		if (ctx->vci > 0) {
			ret = regulator_disable(ctx->vci);
			if (ret) {
				dev_err(ctx->dev, "vci disable failed\n");
				return ret;
			}
		}
	}

	ctx->bl->props.power = on ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;

	return 0;
}
EXPORT_SYMBOL(exynos_panel_set_power);

static int exynos_panel_parse_dt(struct exynos_panel *ctx)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(ctx->dev->of_node)) {
		dev_err(ctx->dev, "no device tree information of exynos panel\n");
		return -EINVAL;
	}

	ret = exynos_panel_parse_gpios(ctx);
	if (ret)
		goto err;

	ret = exynos_panel_parse_regulators(ctx);
	if (ret)
		goto err;

	ctx->touch_dev = of_parse_phandle(ctx->dev->of_node, "touch", 0);

err:
	return ret;
}

int exynos_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	struct drm_display_mode *mode;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	mode = drm_mode_duplicate(connector->dev, ctx->desc->mode);
	if (!mode) {
		dev_err(ctx->dev, "failed to add mode %ux%ux@%u\n",
			ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
			drm_mode_vrefresh(ctx->desc->mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	dev_dbg(ctx->dev, "%s -\n", __func__);

	return 1;
}
EXPORT_SYMBOL(exynos_panel_get_modes);

static int exynos_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int exynos_update_status(struct backlight_device *bl)
{
	struct exynos_panel *ctx = bl_get_data(bl);
	const struct exynos_panel_funcs *exynos_panel_func;
	int brightness = bl->props.brightness;

	dev_dbg(ctx->dev, "br: %d, max br: %d\n", brightness,
		bl->props.max_brightness);

	if (!ctx->enabled) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	/* check if backlight is forced off */
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (exynos_panel_func && exynos_panel_func->set_brightness)
		exynos_panel_func->set_brightness(ctx, brightness);
	else
		exynos_dcs_set_brightness(ctx, brightness);

	return 0;
}

static const struct backlight_ops exynos_backlight_ops = {
	.get_brightness = exynos_get_brightness,
	.update_status = exynos_update_status,
};

static ssize_t serial_number_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_id);
}

static ssize_t panel_extinfo_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_extinfo);
}

static DEVICE_ATTR_RO(serial_number);
static DEVICE_ATTR_RO(panel_extinfo);

static const struct attribute *panel_attrs[] = {
	&dev_attr_serial_number.attr,
	&dev_attr_panel_extinfo.attr,
	NULL
};

static int exynos_drm_connector_get_property(struct drm_connector *connector,
				const struct drm_connector_state *state,
				struct drm_property *property,
				uint64_t *val)
{
	const struct exynos_panel *ctx = connector_to_exynos_panel(connector);

	if (property == ctx->props.max_luminance)
		*val = ctx->desc->max_luminance;
	else if (property == ctx->props.max_avg_luminance)
		*val = ctx->desc->max_avg_luminance;
	else if (property == ctx->props.min_luminance)
		*val = ctx->desc->min_luminance;
	else if (property == ctx->props.hdr_formats)
		*val = ctx->desc->hdr_formats;
	else
		return -EINVAL;

	return 0;
}

void exynos_drm_connector_print_state(struct drm_printer *p,
				      const struct drm_connector_state *state)
{
	const struct exynos_panel *ctx =
		connector_to_exynos_panel(state->connector);
	const struct exynos_panel_desc *desc = ctx->desc;

	drm_printf(p, "\tenabled: %d\n", ctx->enabled);
	drm_printf(p, "\text_info: %s\n", ctx->panel_extinfo);
	drm_printf(p, "\tluminance: [%u, %u] avg: %u\n",
		   desc->min_luminance, desc->max_luminance,
		   desc->max_avg_luminance);
	drm_printf(p, "\thdr_formats: 0x%x\n", desc->hdr_formats);
}

static const struct drm_connector_funcs exynos_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_get_property = exynos_drm_connector_get_property,
	.atomic_print_state = exynos_drm_connector_print_state,
};

static int exynos_drm_connector_modes(struct drm_connector *connector)
{
	struct exynos_panel *ctx = connector_to_exynos_panel(connector);
	int ret;

	ret = drm_panel_get_modes(&ctx->panel, connector);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to get panel display modes\n");
		return ret;
	}

	return ret;
}

int exynos_drm_connector_atomic_check(struct drm_connector *connector,
				      struct drm_atomic_state *state)
{
	struct drm_connector_state *new_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct drm_bridge *bridge;
	struct drm_crtc_state *crtc_state;
	struct drm_encoder *encoder = new_state->best_encoder;
	struct exynos_panel *ctx = connector_to_exynos_panel(connector);

	if (!ctx->touch_dev)
		return 0;

	if (!encoder) {
		dev_warn(ctx->dev, "%s encoder is null\n", __func__);
		return 0;
	}

	crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);
	if (!drm_atomic_crtc_needs_modeset(crtc_state))
		return 0;

	bridge = of_drm_find_bridge(ctx->touch_dev);
	if (!bridge || bridge->dev)
		return 0;

	drm_bridge_attach(encoder, bridge, NULL, 0);
	dev_info(ctx->dev, "attach bridge %p to encoder %p\n", bridge, encoder);

	return 0;
}

static const struct drm_connector_helper_funcs exynos_connector_helper_funcs = {
	.atomic_check = exynos_drm_connector_atomic_check,
	.get_modes = exynos_drm_connector_modes,
};

static int exynos_drm_connector_create_luminance_properties(
				struct drm_connector *connector)
{
	struct exynos_panel *ctx = connector_to_exynos_panel(connector);
	struct drm_property *prop;

	prop = drm_property_create_range(connector->dev, 0, "max_luminance",
			0, UINT_MAX);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&connector->base, prop, 0);
	ctx->props.max_luminance = prop;

	prop = drm_property_create_range(connector->dev, 0, "max_avg_luminance",
			0, UINT_MAX);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&connector->base, prop, 0);
	ctx->props.max_avg_luminance = prop;

	prop = drm_property_create_range(connector->dev, 0, "min_luminance",
			0, UINT_MAX);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&connector->base, prop, 0);
	ctx->props.min_luminance = prop;

	return 0;
}

static int exynos_drm_connector_create_hdr_formats_property(
				struct drm_connector *connector)
{
	static const struct drm_prop_enum_list props[] = {
		{ __builtin_ffs(HDR_DOLBY_VISION) - 1,	"Dolby Vision"	},
		{ __builtin_ffs(HDR_HDR10) - 1,		"HDR10"		},
		{ __builtin_ffs(HDR_HLG) - 1,		"HLG"		},
	};
	struct exynos_panel *ctx = connector_to_exynos_panel(connector);
	struct drm_property *prop;

	prop = drm_property_create_bitmask(connector->dev, 0, "hdr_formats",
					   props, ARRAY_SIZE(props),
					   HDR_DOLBY_VISION | HDR_HDR10 |
					   HDR_HLG);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&connector->base, prop, 0);
	ctx->props.hdr_formats = prop;

	return 0;
}

static int exynos_panel_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_connector *connector = &ctx->connector;
	int ret;

	ret = drm_connector_init(bridge->dev, connector,
				 &exynos_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		dev_err(ctx->dev, "failed to initialize connector with drm\n");
		return ret;
	}

	ret = drm_panel_attach(&ctx->panel, connector);
	if (ret) {
		dev_err(ctx->dev, "unable to attach drm panel\n");
		return ret;
	}

	drm_connector_helper_add(connector, &exynos_connector_helper_funcs);

	exynos_drm_connector_create_luminance_properties(connector);
	exynos_drm_connector_create_hdr_formats_property(connector);

	drm_connector_register(connector);

	drm_connector_attach_encoder(connector, bridge->encoder);
	connector->funcs->reset(connector);
	connector->status = connector_status_connected;

	ret = sysfs_create_link(&connector->kdev->kobj, &ctx->dev->kobj,
				"panel");
	if (ret)
		dev_warn(ctx->dev, "unable to link panel sysfs (%d)\n", ret);

#ifdef CONFIG_DRM_DEBUGFS_PANEL
	mipi_dsi_debugfs_add(to_mipi_dsi_device(ctx->dev), ctx->panel.debugfs_entry);
#endif

	drm_kms_helper_hotplug_event(connector->dev);

	return 0;
}

static void exynos_panel_bridge_detach(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	drm_panel_detach(&ctx->panel);
	drm_connector_unregister(&ctx->connector);
	drm_connector_cleanup(&ctx->connector);
}

static void exynos_panel_enable(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (in_tui(ctx)) {
		dev_info(ctx->dev, "tui state : skip %s\n", __func__);
		return;
	}

	drm_panel_enable(&ctx->panel);
}

static void exynos_panel_pre_enable(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (in_tui(ctx)) {
		dev_info(ctx->dev, "tui state : skip %s\n", __func__);
		return;
	}

	drm_panel_prepare(&ctx->panel);
}

static void exynos_panel_disable(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (in_tui(ctx)) {
		dev_info(ctx->dev, "tui state : skip %s\n", __func__);
		return;
	}

	drm_panel_disable(&ctx->panel);
}

static void exynos_panel_post_disable(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (in_tui(ctx)) {
		dev_info(ctx->dev, "tui state : skip %s\n", __func__);
		return;
	}

	drm_panel_unprepare(&ctx->panel);
}

static bool exynos_panel_mode_fixup(struct drm_bridge *bridge,
				    const struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_display_mode *m;

	list_for_each_entry(m, &ctx->connector.modes, head) {
		if (drm_mode_equal(m, adjusted_mode)) {
		/* TODO: b/165347448 port mode switching to android-gs-pixel-mainline */
#if 0
			adjusted_mode->private = m->private;
			adjusted_mode->private_flags = m->private_flags;

			if (mode->private_flags & EXYNOS_DISPLAY_MODE_FLAG_TUI)
				adjusted_mode->private_flags |=
					EXYNOS_DISPLAY_MODE_FLAG_TUI;
			else
				adjusted_mode->private_flags &=
					~EXYNOS_DISPLAY_MODE_FLAG_TUI;
#endif

			return true;
		}
	}

	dev_err(ctx->dev, "unsupported mode %s\n", adjusted_mode->name);

	return false;
}

static const struct drm_bridge_funcs exynos_panel_bridge_funcs = {
	.attach = exynos_panel_bridge_attach,
	.detach = exynos_panel_bridge_detach,
	.pre_enable = exynos_panel_pre_enable,
	.enable = exynos_panel_enable,
	.disable = exynos_panel_disable,
	.post_disable = exynos_panel_post_disable,
	.mode_fixup = exynos_panel_mode_fixup,
};

int exynos_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct exynos_panel *ctx;
	const struct exynos_display_mode *mode_priv;
	int ret = 0;
	char name[32];

	ctx = devm_kzalloc(dev, sizeof(struct exynos_panel), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	dev_dbg(dev, "%s +\n", __func__);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);
	mode_priv = drm_mode_to_exynos(ctx->desc->mode);
	if (!mode_priv) {
		dev_err(ctx->dev, "missing exynos display mode config\n");
		return -EINVAL;
	}

	dsi->lanes = ctx->desc->data_lane_cnt;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = mode_priv->mode_flags;

	exynos_panel_parse_dt(ctx);

	snprintf(name, sizeof(name), "panel%d-backlight", dsi->channel);
	ctx->bl = devm_backlight_device_register(ctx->dev, name, NULL,
			ctx, &exynos_backlight_ops, NULL);
	if (IS_ERR(ctx->bl)) {
		dev_err(ctx->dev, "failed to register backlight device\n");
		return PTR_ERR(ctx->bl);
	}
	ctx->bl->props.max_brightness = ctx->desc->max_brightness;
	ctx->bl->props.brightness = ctx->desc->dft_brightness;

	drm_panel_init(&ctx->panel, dev, ctx->desc->panel_func, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ctx->bridge.funcs = &exynos_panel_bridge_funcs;
#ifdef CONFIG_OF
	ctx->bridge.of_node = ctx->dev->of_node;
#endif
	drm_bridge_add(&ctx->bridge);

	ret = sysfs_create_files(&dev->kobj, panel_attrs);
	if (ret)
		pr_warn("unable to add panel sysfs files (%d)\n", ret);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		goto err_panel;

	dev_info(ctx->dev, "samsung common panel driver has been probed\n");

	return 0;

err_panel:
	drm_panel_detach(&ctx->panel);
	drm_panel_remove(&ctx->panel);
	dev_err(ctx->dev, "failed to probe samsung panel driver(%d)\n", ret);

	return ret;
}
EXPORT_SYMBOL(exynos_panel_probe);

int exynos_panel_remove(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	drm_bridge_remove(&ctx->bridge);
	devm_backlight_device_unregister(ctx->dev, ctx->bl);

	return 0;
}
EXPORT_SYMBOL(exynos_panel_remove);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung common panel driver");
MODULE_LICENSE("GPL");
