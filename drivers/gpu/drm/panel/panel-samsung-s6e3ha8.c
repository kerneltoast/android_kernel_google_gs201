// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based s6e3ha8 AMOLED LCD panel driver.
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
#include <linux/gpio/consumer.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_encoder.h>
#include <drm/drm_atomic_helper.h>

static int panel_log_level = 7;

#define panel_info(ctx, fmt, ...)					\
	do {								\
		if (panel_log_level >= 6) {				\
			DRM_INFO("%s: "fmt, ctx->dev->driver->name,	\
					##__VA_ARGS__);			\
		}							\
	} while (0)

#define panel_warn(ctx, fmt, ...)					\
	do {								\
		if (panel_log_level >= 4) {				\
			DRM_WARN("%s: "fmt, ctx->dev->driver->name,	\
					##__VA_ARGS__);			\
		}							\
	} while (0)

#define panel_err(ctx, fmt, ...)					\
	do {								\
		if (panel_log_level >= 3) {				\
			DRM_ERROR("%s: "fmt, ctx->dev->driver->name,	\
					##__VA_ARGS__);			\
		}							\
	} while (0)

#define panel_dbg(ctx, fmt, ...)					\
	do {								\
		if (panel_log_level >= 7) {				\
			DRM_INFO("%s: "fmt, ctx->dev->driver->name,	\
					##__VA_ARGS__);			\
		}							\
	} while (0)


#define MAX_REGULATORS		3

static unsigned char SEQ_PPS_SLICE2[] = {
	// QHD :2960x1440
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0B, 0x90,
	0x05, 0xA0, 0x00, 0x28, 0x02, 0xD0, 0x02, 0xD0,
	0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x04, 0x6C,
	0x00, 0x0A, 0x00, 0x0C, 0x02, 0x77, 0x01, 0xE9,
	0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
	0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

struct s6e3ha8_panel_desc {
	const struct drm_display_mode *mode;
	bool dsc_en;
	u32 dsc_slice_cnt;
	u32 dsc_slice_height;
	u32 data_lane_cnt;
};

struct s6e3ha8 {
	struct device *dev;
	struct drm_panel panel;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator *regulator[MAX_REGULATORS];

	const struct s6e3ha8_panel_desc *desc;
};

static int s6e3ha8_dcs_write(struct s6e3ha8 *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_write_buffer(dsi, data, len);
}

static int s6e3ha8_dcs_compression_mode(struct s6e3ha8 *ctx, u8 mode)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_compression_mode(dsi, mode);
}

#define s6e3ha8_dcs_write_seq(ctx, seq...) do {				\
	u8 d[] = { seq };						\
	int ret;							\
	ret = s6e3ha8_dcs_write(ctx, d, ARRAY_SIZE(d));			\
	if (ret < 0)							\
		panel_err(ctx, "failed to write cmd(%d)\n", ret);	\
} while (0)

#define s6e3ha8_dcs_write_seq_delay(ctx, delay, seq...) do {		\
	s6e3ha8_dcs_write_seq(ctx, seq);				\
	msleep(delay);							\
} while (0)

#define s6e3ha8_dcs_write_table(ctx, table) do {			\
	int ret;							\
	ret = s6e3ha8_dcs_write(ctx, table, ARRAY_SIZE(table));		\
	if (ret < 0)							\
		panel_err(ctx, "failed to write cmd(%d)\n", ret);	\
} while (0)

#define s6e3ha8_pps_long_write(ctx, pps) do {				\
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);	\
	int ret;							\
	ret = mipi_dsi_pps_long_write(dsi, pps, ARRAY_SIZE(pps));	\
	if (ret < 0)							\
		panel_err(ctx, "failed to write cmd(%d)\n", ret);	\
} while (0)

static int exynos_panel_parse_gpios(struct s6e3ha8 *ctx)
{
	struct device *dev = ctx->dev;

	panel_info(ctx, "%s +\n", __func__);

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

static int exynos_panel_parse_regulators(struct s6e3ha8 *ctx)
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

static void exynos_panel_reset(struct s6e3ha8 *ctx)
{
	panel_dbg(ctx, "%s +\n", __func__);

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);

	panel_dbg(ctx, "%s -\n", __func__);
}

static int exynos_panel_set_power(struct s6e3ha8 *ctx, bool on)
{
	int ret;

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

static int exynos_panel_parse_dt(struct s6e3ha8 *ctx)
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

static int s6e3ha8_disable(struct drm_panel *panel)
{
	DRM_INFO("%s +\n", __func__);
	return 0;
}

static int s6e3ha8_unprepare(struct drm_panel *panel)
{
	struct s6e3ha8 *ctx = container_of(panel, struct s6e3ha8, panel);

	panel_dbg(ctx, "%s +\n", __func__);
	exynos_panel_set_power(ctx, false);
	panel_dbg(ctx, "%s -\n", __func__);
	return 0;
}

static int s6e3ha8_prepare(struct drm_panel *panel)
{
	struct s6e3ha8 *ctx = container_of(panel, struct s6e3ha8, panel);

	panel_dbg(ctx, "%s +\n", __func__);
	exynos_panel_set_power(ctx, true);
	panel_dbg(ctx, "%s -\n", __func__);

	return 0;
}

static int s6e3ha8_enable(struct drm_panel *panel)
{
	struct s6e3ha8 *ctx = container_of(panel, struct s6e3ha8, panel);

	panel_dbg(ctx, "%s +\n", __func__);

	exynos_panel_reset(ctx);

	s6e3ha8_dcs_write_seq(ctx, 0xf0, 0x5a, 0x5a);
	s6e3ha8_dcs_write_seq(ctx, 0xfc, 0x5a, 0x5a);

	/* DSC related configuration */
	s6e3ha8_dcs_compression_mode(ctx, 0x1);
	s6e3ha8_pps_long_write(ctx, SEQ_PPS_SLICE2);

	/* sleep out: 120ms delay */
	s6e3ha8_dcs_write_seq_delay(ctx, 120, 0x11);
	s6e3ha8_dcs_write_seq(ctx, 0xB9, 0x00, 0xB0, 0x8F, 0x09, 0x00, 0x00,
			0x00, 0x11, 0x01);
	s6e3ha8_dcs_write_seq(ctx, 0x1A, 0x1F, 0x00, 0x00, 0x00, 0x00);

	s6e3ha8_dcs_write_seq(ctx, 0x35); /* TE on */
	s6e3ha8_dcs_write_seq(ctx, 0xED, 0x44);

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
	s6e3ha8_dcs_write_seq(ctx, 0xB9, 0x01, 0xB0, 0x91, 0x09);
#else
	s6e3ha8_dcs_write_seq(ctx, 0xB9, 0x00, 0xB0, 0x9C, 0x09);
#endif

	s6e3ha8_dcs_write_seq(ctx, 0xCE, 0x0D, 0x58, 0x14, 0x64, 0x38, 0xB8,
			0xF2, 0x03, 0x00, 0xFF, 0x02, 0x0A, 0x0A, 0x0A, 0x0A,
			0x0F, 0x23);

	s6e3ha8_dcs_write_seq(ctx, 0x53, 0x20); /* enable brightness control */
	s6e3ha8_dcs_write_seq(ctx, 0x51, 0x01, 0x80); /* brightness level */

	s6e3ha8_dcs_write_seq(ctx, 0x29); /* display on */

	panel_dbg(ctx, "%s -\n", __func__);

	return 0;
}

static const struct drm_display_mode s6e3ha8_mode = {
	.clock = 56125,	/* 898Mbps / 2 = 449Mhz / 8 = 56.125Mhz */
	.hdisplay = 1440,
	.hsync_start = 1440 + 2,
	.hsync_end = 1440 + 2 + 2,
	.htotal = 1440 + 2 + 2 + 2,
	.vdisplay = 2960,
	.vsync_start = 2960 + 8,
	.vsync_end = 2960 + 8 + 1,
	.vtotal = 2960 + 8 + 1 + 15,
	.vrefresh = 60,
	.flags = 0,
	.width_mm = 69,
	.height_mm = 142,
};

static const struct s6e3ha8_panel_desc samsung_s6e3ha8 = {
	.mode = &s6e3ha8_mode,
	.dsc_en = true,
	.dsc_slice_cnt = 2,
	.dsc_slice_height = 40,
	.data_lane_cnt = 4,
};

static int s6e3ha8_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct s6e3ha8 *ctx = container_of(panel, struct s6e3ha8, panel);
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

static const struct drm_panel_funcs s6e3ha8_drm_funcs = {
	.disable = s6e3ha8_disable,
	.unprepare = s6e3ha8_unprepare,
	.prepare = s6e3ha8_prepare,
	.enable = s6e3ha8_enable,
	.get_modes = s6e3ha8_get_modes,
};

static int s6e3ha8_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3ha8 *ctx;
	int ret = 0;

	ctx = devm_kzalloc(dev, sizeof(struct s6e3ha8), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);

	panel_info(ctx, "%s +\n", __func__);

	dsi->lanes = ctx->desc->data_lane_cnt;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	exynos_panel_parse_dt(ctx);

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &s6e3ha8_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

	panel_info(ctx, "%s -\n", __func__);

	return ret;
}

static int s6e3ha8_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3ha8 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id s6e3ha8_of_match[] = {
	{ .compatible = "samsung,s6e3ha8", .data = &samsung_s6e3ha8 },
	{ }
};
MODULE_DEVICE_TABLE(of, s6e3ha8_of_match);

static struct mipi_dsi_driver s6e3ha8_driver = {
	.probe = s6e3ha8_probe,
	.remove = s6e3ha8_remove,
	.driver = {
		.name = "panel-samsung-s6e3ha8",
		.of_match_table = s6e3ha8_of_match,
	},
};
module_mipi_dsi_driver(s6e3ha8_driver);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based s6e3ha8 AMOLED LCD Panel Driver");
MODULE_LICENSE("GPL");
