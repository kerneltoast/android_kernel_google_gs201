// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based emulator panel driver.
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <panel-samsung-drv.h>

static int emul_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;

	ctx = container_of(panel, struct exynos_panel, panel);
	panel_info(ctx, "%s +\n", __func__);
	return 0;
}

static int emul_unprepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx;

	ctx = container_of(panel, struct exynos_panel, panel);

	panel_dbg(ctx, "%s +\n", __func__);
	exynos_panel_set_power(ctx, false);
	panel_dbg(ctx, "%s -\n", __func__);
	return 0;
}

static int emul_prepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx;

	ctx = container_of(panel, struct exynos_panel, panel);

	panel_dbg(ctx, "%s +\n", __func__);
	exynos_panel_set_power(ctx, true);
	panel_dbg(ctx, "%s -\n", __func__);

	return 0;
}

static int emul_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;

	ctx = container_of(panel, struct exynos_panel, panel);

	panel_dbg(ctx, "%s +\n", __func__);

	/* sleep out: 120ms delay */
	EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, 120, 0x11);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x29); /* display on */

	panel_dbg(ctx, "%s -\n", __func__);

	return 0;
}

static const struct drm_display_mode emul_mode = {
	.clock = 64125,	/* 898Mbps / 2 = 449Mhz / 8 = 56.125Mhz */
	.hdisplay = 720,
	.hsync_start = 720 + 20,
	.hsync_end = 720 + 20 + 20,
	.htotal = 720 + 20 + 20 + 20,
	.vdisplay = 1280,
	.vsync_start = 1280 + 20,
	.vsync_end = 1280 + 20 + 20,
	.vtotal = 1280 + 20 + 20 + 20,
	.vrefresh = 60,
	.flags = 0,
	.width_mm = 80,
	.height_mm = 120,
};

static const struct drm_panel_funcs emul_drm_funcs = {
	.disable = emul_disable,
	.unprepare = emul_unprepare,
	.prepare = emul_prepare,
	.enable = emul_enable,
	.get_modes = exynos_panel_get_modes,
};

const struct exynos_panel_desc samsung_emul = {
	.dsc_en = false,
	.data_lane_cnt = 4,
	.mode_flags = MIPI_DSI_MODE_VIDEO,
	.mode = &emul_mode,
	.panel_func = &emul_drm_funcs,
};
