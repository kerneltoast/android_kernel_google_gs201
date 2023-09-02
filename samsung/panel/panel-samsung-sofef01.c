// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based sofef01 AMOLED LCD panel driver.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <video/mipi_display.h>

#include "panel-samsung-drv.h"

#define SOFEF01_WRCTRLD_BCTRL_BIT      0x20
#define SOFEF01_WRCTRLD_HBM_BIT        0xC0

static const u8 unlock_cmd_f0[] = { 0xF0, 0x5A, 0x5A };
static const u8 lock_cmd_f0[] = { 0xF0, 0xA5, 0xA5 };
static const u8 global_para_05[] = { 0xB0, 0x05 };
static const u8 err_flag_setting[] = { 0xF4, 0x01 };
static const u8 display_off[] = { 0x28 };
static const u8 sleep_in[] = { 0x10 };

static const struct exynos_dsi_cmd sofef01_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 20),
	EXYNOS_DSI_CMD(sleep_in, 0),
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD(global_para_05, 0),
	EXYNOS_DSI_CMD(err_flag_setting, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 120),
};

static const struct exynos_dsi_cmd_set sofef01_off_cmd_set = {
	.num_cmd = ARRAY_SIZE(sofef01_off_cmds),
	.cmds = sofef01_off_cmds
};

static void sofef01_write_display_mode(struct exynos_panel *ctx,
				       const struct drm_display_mode *mode)
{
	u8 val = SOFEF01_WRCTRLD_BCTRL_BIT;

	if (IS_HBM_ON(ctx->hbm_mode))
		val |= SOFEF01_WRCTRLD_HBM_BIT;

	dev_dbg(ctx->dev, "%s(wrctrld:0x%x, hbm: %s, refresh: %uhz)\n",
		__func__, val, IS_HBM_ON(ctx->hbm_mode) ? "on" : "off", drm_mode_vrefresh(mode));

	EXYNOS_DCS_WRITE_SEQ(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);
}

static int sofef01_pre_enable(struct exynos_panel *ctx)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);

	/* sleep out: 120ms delay */
	EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, 10, 0x11);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0x5A, 0x5A);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x07);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB7, 0x08);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x35); /* TE on */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xEB, 0x17, 0x41, 0x92, 0x0E, 0x10, 0x86, 0x5A);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0xA5, 0xA5);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x2A, 0x00, 0x00, 0x04, 0x37);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x2B, 0x00, 0x00, 0x09, 0x23);

	EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, 110, 0x53, 0x28);

	sofef01_write_display_mode(ctx, &pmode->mode);

	return 0;
}

static void sofef01_post_enable(struct exynos_panel *ctx)
{
	dev_dbg(ctx->dev, "%s\n", __func__);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x51, 0x01, 0x80); /* brightness level */

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x29); /* display on */

	ctx->enabled = true;
}

static int sofef01_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;
	int ret;

	ctx = container_of(panel, struct exynos_panel, panel);

	ret = sofef01_pre_enable(ctx);
	if (ret)
		return ret;

	sofef01_post_enable(ctx);

	return 0;
}

static void sofef01_set_hbm_mode(struct exynos_panel *exynos_panel,
				enum exynos_hbm_mode mode)
{
	const struct exynos_panel_mode *pmode = exynos_panel->current_mode;

	exynos_panel->hbm_mode = mode;

	sofef01_write_display_mode(exynos_panel, &pmode->mode);
}

static const struct exynos_display_underrun_param underrun_param = {
	.te_idle_us = 1000,
	.te_var = 1,
};

static const struct exynos_panel_mode sofef01_modes[] = {
	{
		/* 1080x2340 @ 60Hz */
		.mode = {
			.name = "1080x2340x60",
			.clock = 162633,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32,
			.hsync_end = 1080 + 32 + 32,
			.htotal = 1080 + 32 + 32 + 6,
			.vdisplay = 2340,
			.vsync_start = 2340 + 8,
			.vsync_end = 2340 + 8 + 1,
			.vtotal = 2340 + 8 + 1 + 8,
			.flags = 0,
			.width_mm = 66,
			.height_mm = 144,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = false,
			},
			.underrun_param = &underrun_param,
		},
	},
};

static const struct drm_panel_funcs sofef01_drm_funcs = {
	.disable = exynos_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = sofef01_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs sofef01_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_hbm_mode = sofef01_set_hbm_mode,
};

const struct exynos_panel_desc samsung_sofef01 = {
	.data_lane_cnt = 4,
	.max_brightness = 1023,
	.dft_brightness = 511,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 5400000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.modes = sofef01_modes,
	.num_modes = ARRAY_SIZE(sofef01_modes),
	.off_cmd_set = &sofef01_off_cmd_set,
	.panel_func = &sofef01_drm_funcs,
	.exynos_panel_func = &sofef01_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,sofef01", .data = &samsung_sofef01 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = exynos_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-sofef01",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung sofef01 panel driver");
MODULE_LICENSE("GPL");
