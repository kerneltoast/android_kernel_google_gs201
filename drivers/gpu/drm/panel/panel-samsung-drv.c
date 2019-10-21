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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_encoder.h>
#include <drm/drm_atomic_helper.h>

#include <panel-samsung-drv.h>

int panel_log_level = 7;

static int exynos_panel_parse_gpios(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;

	panel_info(ctx, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		panel_info(ctx, "no reset/enable pins on emulator\n");
		return 0;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		panel_err(ctx, "failed to get reset-gpios %ld",
				PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio)) {
		panel_err(ctx, "failed to get enable-gpios %ld\n",
				PTR_ERR(ctx->enable_gpio));
		return PTR_ERR(ctx->enable_gpio);
	}

	panel_info(ctx, "%s -\n", __func__);
	return 0;
}

static int exynos_panel_parse_regulators(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;
	char *str_regulator[MAX_REGULATORS];
	int i;

	for (i = 0; i < MAX_REGULATORS; ++i) {
		ctx->regulator[i] = NULL;
		str_regulator[i] = NULL;
	}

	if (!of_property_read_string(dev->of_node, "regulator_1p8v",
				(const char **)&str_regulator[0])) {
		ctx->regulator[0] = regulator_get(dev, str_regulator[0]);
		if (IS_ERR(ctx->regulator[0])) {
			panel_err(ctx, "panel regulator 1.8V get failed\n");
			ctx->regulator[0] = NULL;
		}
	}

	if (!of_property_read_string(dev->of_node, "regulator_3p3v",
				(const char **)&str_regulator[1])) {
		ctx->regulator[1] = regulator_get(dev, str_regulator[1]);
		if (IS_ERR(ctx->regulator[1])) {
			panel_err(ctx, "panel regulator 3.3V get failed\n");
			ctx->regulator[1] = NULL;
		}
	}

	return 0;
}

void exynos_panel_reset(struct exynos_panel *ctx)
{
	panel_dbg(ctx, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return;

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);

	panel_dbg(ctx, "%s -\n", __func__);
}

int exynos_panel_set_power(struct exynos_panel *ctx, bool on)
{
	int ret;

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return 0;

	if (on) {
		gpiod_set_value(ctx->enable_gpio, 1);
		usleep_range(10000, 11000);

		if (ctx->regulator[0] > 0) {
			ret = regulator_enable(ctx->regulator[0]);
			if (ret) {
				panel_err(ctx, "panel regulator 1.8V enable failed\n");
				return ret;
			}
			usleep_range(5000, 6000);
		}

		if (ctx->regulator[1] > 0) {
			ret = regulator_enable(ctx->regulator[1]);
			if (ret) {
				panel_err(ctx, "panel regulator 3.3V enable failed\n");
				return ret;
			}
		}
	} else {
		gpiod_set_value(ctx->reset_gpio, 0);
		gpiod_set_value(ctx->enable_gpio, 0);

		if (ctx->regulator[0] > 0) {
			ret = regulator_disable(ctx->regulator[0]);
			if (ret) {
				panel_err(ctx, "panel regulator 1.8V disable failed\n");
				return ret;
			}
		}

		if (ctx->regulator[1] > 0) {
			ret = regulator_disable(ctx->regulator[1]);
			if (ret) {
				panel_err(ctx, "panel regulator 3.3V disable failed\n");
				return ret;
			}
		}
	}

	return 0;
}

static int exynos_panel_parse_dt(struct exynos_panel *ctx)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(ctx->dev->of_node)) {
		panel_err(ctx, "no device tree information of exynos panel\n");
		return -EINVAL;
	}

	ret = exynos_panel_parse_gpios(ctx);
	if (ret)
		goto err;

	ret = exynos_panel_parse_regulators(ctx);
	if (ret)
		goto err;

err:
	return ret;
}

int exynos_panel_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	struct drm_display_mode *mode;

	panel_info(ctx, "%s +\n", __func__);

	mode = drm_mode_duplicate(panel->drm, ctx->desc->mode);
	if (!mode) {
		panel_err(ctx, "failed to add mode %ux%ux@%u\n",
			ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
			ctx->desc->mode->vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	panel_info(ctx, "%s -\n", __func__);

	return 0;
}

static int exynos_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct exynos_panel *ctx;
	int ret = 0;

	ctx = devm_kzalloc(dev, sizeof(struct exynos_panel), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);

	panel_info(ctx, "%s +\n", __func__);

	dsi->lanes = ctx->desc->data_lane_cnt;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = ctx->desc->mode_flags;

	exynos_panel_parse_dt(ctx);

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = ctx->desc->panel_func;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

	panel_info(ctx, "%s -\n", __func__);

	return ret;
}

static int exynos_panel_remove(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,emul", .data = &samsung_emul },
	{ .compatible = "samsung,s6e3ha8", .data = &samsung_s6e3ha8 },
	{ .compatible = "samsung,s6e3ha9", .data = &samsung_s6e3ha9 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = exynos_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-drv",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung common panel driver");
MODULE_LICENSE("GPL");
