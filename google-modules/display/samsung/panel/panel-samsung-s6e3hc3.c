// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based s6e3hc3 AMOLED LCD panel driver.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <drm/drm_vblank.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <video/mipi_display.h>
#include <trace/dpu_trace.h>

#include "panel-samsung-drv.h"

/**
 * struct s6e3hc3_panel - panel specific runtime info
 *
 * This struct maintains s6e3hc3 panel specific runtime info, any fixed details about panel should
 * most likely go into struct exynos_panel_desc
 */
struct s6e3hc3_panel {
	/** @base: base panel struct */
	struct exynos_panel base;

	/**
	 * @auto_mode_vrefresh: indicates current minimum refresh rate while in auto mode,
	 *			if 0 it means that auto mode is not enabled
	 */
	u32 auto_mode_vrefresh;

	/** @is_hbm_update: indicates if there is a hbm state update request */
	bool is_hbm_update;

	/** @force_changeable_te: force changeable TE (instead of fixed) during early exit */
	bool force_changeable_te;
};

#define to_spanel(ctx) container_of(ctx, struct s6e3hc3_panel, base)

/**
 * struct s6e3hc3_mode_data - panel mode specific details
 *
 * This struct maintains panel mode specific details used to help with transitions between
 * different panel modes/refresh rates.
 */
struct s6e3hc3_mode_data {
	/**
	 * @manual_mode_cmd_set:
	 *
	 * This cmd set is sent to panel during mode switch to enable manual mode. This mode is
	 * typically enabled when driver is not allowed to change modes while idle. In this mode,
	 * the panel should remain in this mode (regardless of idleness) until we indicate
	 * otherwise.
	 *
	 * If auto mode cmd set is defined, then manual mode cmd set should also be defined.
	 */
	const struct exynos_dsi_cmd_set *manual_mode_cmd_set;

	/**
	 * @manual_mode_ghbm_cmd_set:
	 *
	 * This cmd set is sent to panel during mode switch to enable manual mode in GHBM on
	 * because the manual mode command is different in normal and global high brightness
	 * modes on EVT1_1 and after. This mode is typically enabled when driver is not
	 * allowed to change modes while idle. In this mode, the panel should remain in this
	 * mode (regardless of idleness) until we indicate otherwise.
	 *
	 * If auto mode cmd set is defined, then manual mode cmd set should also be defined.
	 */
	const struct exynos_dsi_cmd_set *manual_mode_ghbm_cmd_set;

	/**
	 * @common_mode_cmd_set:
	 *
	 * This cmd set is sent to panel for every mode switch after either manual or auto mode are
	 * sent to panel.
	 *
	 * If manual or auto mode cmd sets are not supported, then this cmd set MUST be defined.
	 * Otherwise if manual/auto mode cmd sets are defined this is optional.
	 */
	const struct exynos_dsi_cmd_set *common_mode_cmd_set;

	/**
	 * @wakeup_mode_cmd_set:
	 *
	 * When driver is allowed to change modes while idle, this function will be called when
	 * display is coming out of self refresh (i.e. waking up after idle) to go back to expected
	 * operating mode/refresh rate.
	 *
	 * This function is expected be defined if idle_vrefresh is defined in order to revert
	 * optimizations done while entering idle.
	 */
	const struct exynos_dsi_cmd_set *wakeup_mode_cmd_set;
};

static const unsigned char WQHD_PPS_SETTING[DSC_PPS_SIZE] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0C, 0x30,
	0x05, 0xA0, 0x00, 0x34, 0x02, 0xD0, 0x02, 0xD0,
	0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x05, 0xC6,
	0x00, 0x0A, 0x00, 0x0C, 0x01, 0xE2, 0x01, 0x78,
	0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
	0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
};

static const unsigned char FHD_PPS_SETTING[DSC_PPS_SIZE] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x24,
	0x04, 0x38, 0x00, 0x4E, 0x02, 0x1C, 0x02, 0x1C,
	0x02, 0x00, 0x02, 0x0E, 0x00, 0x20, 0x07, 0x93,
	0x00, 0x07, 0x00, 0x0C, 0x01, 0x40, 0x01, 0x4E,
	0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
	0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
};

#define S6E3HC3_WRCTRLD_DIMMING_BIT    0x08
#define S6E3HC3_WRCTRLD_BCTRL_BIT      0x20
#define S6E3HC3_WRCTRLD_HBM_BIT        0xC0
#define S6E3HC3_WRCTRLD_LOCAL_HBM_BIT  0x10

#define S6E3HC3_TE2_CHANGEABLE 0x31
#define S6E3HC3_TE2_FIXED      0x41

static const u8 unlock_cmd_f0[] = { 0xF0, 0x5A, 0x5A };
static const u8 lock_cmd_f0[]   = { 0xF0, 0xA5, 0xA5 };
static const u8 unlock_cmd_f1[] = { 0xF1, 0x5A, 0x5A };
static const u8 lock_cmd_f1[]   = { 0xF1, 0xA5, 0xA5 };
static const u8 display_off[] = { 0x28 };
static const u8 display_on[] = { 0x29 };
static const u8 sleep_in[] = { 0x10 };
static const u8 freq_update[] = { 0xF7, 0x0F };

static const struct exynos_dsi_cmd s6e3hc3_lp_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 0),
	EXYNOS_DSI_CMD0_REV(unlock_cmd_f0, PANEL_REV_GE(PANEL_REV_PROTO1_1)),
	/* Fixed TE */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1), 0xB9, 0x41),

	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1), 0xB0, 0x00, 0x01, 0x60),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1), 0x60, 0x00),	/* 30Hz */

	/* enable fast exit */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			       0xBD, 0x21, 0x02),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			       0xB0, 0x00, 0x10, 0xBD),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			       0xBD, 0x10),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			       0xB0, 0x00, 0x5E, 0xBD),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			       0xBD, 0x00, 0x00, 0x00, 0x02), /* HLPM mode */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			       0xB0, 0x00, 0x18, 0xBD),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			       0xBD, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00), /* 10hz step setting */

	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			       0xBD, 0x25), /* auto mode */
	EXYNOS_DSI_CMD0_REV(freq_update, PANEL_REV_GE(PANEL_REV_PROTO1_1)),
	EXYNOS_DSI_CMD0_REV(lock_cmd_f0, PANEL_REV_GE(PANEL_REV_PROTO1_1)),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_lp);

static const struct exynos_dsi_cmd s6e3hc3_lp_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 0)
};

static const struct exynos_dsi_cmd s6e3hc3_lp_low_cmds[] = {
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD_SEQ(0x53, 0x25),	/* aod 10 nit */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_PROTO1, 0x49, 0x01),	/* hlpm gamma */
	EXYNOS_DSI_CMD(lock_cmd_f0, 34),
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_dsi_cmd s6e3hc3_lp_high_cmds[] = {
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD_SEQ(0x53, 0x24),	/* aod 50 nit */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_PROTO1, 0x49, 0x01),	/* hlpm gamma */
	EXYNOS_DSI_CMD(lock_cmd_f0, 34),
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_binned_lp s6e3hc3_binned_lp[] = {
	BINNED_LP_MODE("off", 0, s6e3hc3_lp_off_cmds),
	/* rising time = 0, falling time = 48 */
	BINNED_LP_MODE_TIMING("low", 80, s6e3hc3_lp_low_cmds, 0, 48),
	BINNED_LP_MODE_TIMING("high", 2047, s6e3hc3_lp_high_cmds, 0, 48)
};

static const u8 early_exit_global_para[] = { 0xB0, 0x00, 0x10, 0xBD };
static const u8 early_exit_step_global_para[] = { 0xB0, 0x00, 0x21, 0xBD };

static const struct exynos_dsi_cmd s6e3hc3_early_exit_disable_cmds[] = {
	/* Changeable TE */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1), 0xB9, 0x00),

	EXYNOS_DSI_CMD_SEQ(0xBD, 0x21, 0x82),
	EXYNOS_DSI_CMD0(early_exit_global_para),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x00),
	EXYNOS_DSI_CMD0(early_exit_step_global_para),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x03, 0x00, 0x09, 0x00, 0x21, 0x00, 0x21, 0x00,
				 0x21, 0x00, 0x21, 0x00, 0x21, 0x00, 0x00, 0x00,
				 0x03, 0x00, 0x06, 0x00, 0x09, 0x00, 0x0C, 0x00,
				 0x0F, 0x00, 0x0F, 0x00, 0x0F),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_early_exit_disable);

static const struct exynos_dsi_cmd s6e3hc3_early_exit_enable_cmds[] = {
	/* Fixed TE */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1), 0xB9, 0x41),

	EXYNOS_DSI_CMD_SEQ(0xBD, 0x21, 0x02),
	EXYNOS_DSI_CMD0(early_exit_global_para),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x10),
	EXYNOS_DSI_CMD0(early_exit_step_global_para),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x01, 0x00, 0x03, 0x00, 0x0B, 0x00, 0x0B, 0x00,
				 0x0B, 0x00, 0x0B, 0x00, 0x0B, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0x00, 0x00, 0x00),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_early_exit_enable);

static const u8 auto_step_global_para[] = { 0xB0, 0x00, 0x12, 0xBD };
static const u8 auto_mode[] = { 0xBD, 0x23 };
static const u8 manual_mode[] = { 0xBD, 0x21 };
static const u8 mode_set_120hz[] = { 0x60, 0x00 };
static const u8 mode_set_120hz_GHBM[] = { 0x60, 0x10 };
static const u8 mode_set_60hz[] = { 0x60, 0x01 };

static const struct exynos_dsi_cmd s6e3hc3_mode_120_manual_cmds[] = {
	EXYNOS_DSI_CMD0(manual_mode),
	EXYNOS_DSI_CMD0(mode_set_120hz),
	EXYNOS_DSI_CMD0(freq_update),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_mode_120_manual);

static const struct exynos_dsi_cmd s6e3hc3_mode_120_ghbm_manual_cmds[] = {
	EXYNOS_DSI_CMD0_REV(manual_mode, PANEL_REV_GE(PANEL_REV_EVT1_1)),
	EXYNOS_DSI_CMD0_REV(mode_set_120hz_GHBM, PANEL_REV_GE(PANEL_REV_EVT1_1)),
	EXYNOS_DSI_CMD0_REV(freq_update, PANEL_REV_GE(PANEL_REV_EVT1_1)),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_mode_120_ghbm_manual);

static const struct s6e3hc3_mode_data s6e3hc3_mode_120 = {
	.manual_mode_cmd_set = &s6e3hc3_mode_120_manual_cmd_set,
	.manual_mode_ghbm_cmd_set = &s6e3hc3_mode_120_ghbm_manual_cmd_set,
};

static const struct exynos_dsi_cmd s6e3hc3_mode_60_common_cmds[] = {
	EXYNOS_DSI_CMD0(manual_mode),
	EXYNOS_DSI_CMD0(mode_set_60hz),
	EXYNOS_DSI_CMD0(freq_update),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_mode_60_common);

static const struct exynos_dsi_cmd s6e3hc3_mode_60_wakeup_cmds[] = {
	EXYNOS_DSI_CMD0(auto_step_global_para),
	/* 60hz step setting with early exit off */
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x01, 0x00, 0x03, 0x00, 0x06, 0x01),

	EXYNOS_DSI_CMD0(manual_mode),
	EXYNOS_DSI_CMD0(mode_set_60hz),
	EXYNOS_DSI_CMD0(freq_update),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_mode_60_wakeup);

static const struct s6e3hc3_mode_data s6e3hc3_mode_60 = {
	.common_mode_cmd_set = &s6e3hc3_mode_60_common_cmd_set,
	.wakeup_mode_cmd_set = &s6e3hc3_mode_60_wakeup_cmd_set,
};

static u8 s6e3hc3_get_te2_option(struct exynos_panel *ctx)
{
	struct s6e3hc3_panel *spanel = to_spanel(ctx);

	if (!ctx || !ctx->current_mode)
		return S6E3HC3_TE2_CHANGEABLE;

	/* proto 1.0 only supports changeable TE2 */
	if (ctx->panel_rev == PANEL_REV_PROTO1)
		return S6E3HC3_TE2_CHANGEABLE;

	if (ctx->current_mode->exynos_mode.is_lp_mode ||
	    spanel->auto_mode_vrefresh)
		return S6E3HC3_TE2_FIXED;

	return S6E3HC3_TE2_CHANGEABLE;
}

static void s6e3hc3_update_te2(struct exynos_panel *ctx)
{
	struct exynos_panel_te2_timing timing;
	u8 width[7] = {0xB9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30}; /* default timing */
	u32 rising, falling;
	u8 option = s6e3hc3_get_te2_option(ctx);
	int ret;

	if (!ctx)
		return;

	ret = exynos_panel_get_current_mode_te2(ctx, &timing);
	if (!ret) {
		rising = timing.rising_edge;
		falling = timing.falling_edge;

		width[1] = (rising >> 8) & 0xF;
		width[2] = rising & 0xFF;

		if (option == S6E3HC3_TE2_CHANGEABLE) {
			width[3] = width[4] = 0;
		} else { /* S6E3HC3_TE2_FIXED */
			width[3] = width[1];
			width[4] = width[2];
		}

		width[5] = (falling >> 8) & 0xF;
		width[6] = falling & 0xFF;
	} else if (ret == -EAGAIN) {
		dev_dbg(ctx->dev, "Panel is not ready, use default setting\n");
	} else {
		return;
	}

	dev_dbg(ctx->dev,
		"TE2 updated: option %s, idle %s, width 0xb9 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		(option == S6E3HC3_TE2_CHANGEABLE) ? "changeable" : "fixed",
		ctx->panel_idle_vrefresh ? "active" : "inactive",
		width[1], width[2], width[3], width[4], width[5], width[6]);

	EXYNOS_DCS_BUF_ADD_SET(ctx, unlock_cmd_f0);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x4F, 0xF2);
	EXYNOS_DCS_BUF_ADD(ctx, 0xF2, 0x0D);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x01, 0xB9);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB9, option);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x14, 0xB9);
	EXYNOS_DCS_BUF_ADD_SET(ctx, width);
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, lock_cmd_f0);
}

static inline bool is_auto_mode_preferred(struct exynos_panel *ctx)
{
	/* don't want to enable auto mode/early exit during hbm or dimming on */
	if (IS_HBM_ON(ctx->hbm_mode) || ctx->dimming_on)
		return false;

	if (ctx->idle_delay_ms) {
		const unsigned int delta_ms = panel_get_idle_time_delta(ctx);

		if (delta_ms < ctx->idle_delay_ms)
			return false;
	}

	return ctx->panel_idle_enabled;
}

static void s6e3hc3_update_early_exit(struct exynos_panel *ctx, bool enable)
{
	const struct s6e3hc3_panel *spanel = to_spanel(ctx);
	const u32 flags = PANEL_CMD_SET_QUEUE;

	dev_dbg(ctx->dev, "%s: en=%d\n", __func__, enable);

	if (enable) {
		exynos_panel_send_cmd_set_flags(ctx, &s6e3hc3_early_exit_enable_cmd_set, flags);
		if (spanel->force_changeable_te)
			EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xB9, 0x00);
	} else {
		exynos_panel_send_cmd_set_flags(ctx, &s6e3hc3_early_exit_disable_cmd_set, flags);
	}
}

static u32 s6e3hc3_get_min_idle_vrefresh(struct exynos_panel *ctx,
					 const struct exynos_panel_mode *pmode)
{
	const u32 vrefresh = drm_mode_vrefresh(&pmode->mode);
	int idle_vrefresh = ctx->min_vrefresh;

	if ((idle_vrefresh < 0) || !is_auto_mode_preferred(ctx))
		return 0;

	if (idle_vrefresh <= 10)
		idle_vrefresh = 10;
	else if (idle_vrefresh <= 30)
		idle_vrefresh = 30;
	else if (idle_vrefresh <= 60)
		idle_vrefresh = 60;
	else /* 120hz: no idle available */
		return 0;

	if (idle_vrefresh >= vrefresh) {
		dev_dbg(ctx->dev, "idle vrefresh (%u) higher than target (%u)\n",
			idle_vrefresh, vrefresh);
		return 0;
	}

	return idle_vrefresh;
}

static void s6e3hc3_set_early_exit_auto_mode(struct exynos_panel *ctx,
					     const u32 idle_vrefresh)
{
	struct s6e3hc3_panel *spanel = to_spanel(ctx);
	u8 step_cmd[] = {0xBD, 0x01, 0x00, 0x0B, 0x00, 0x03, 0x01}; /* 10hz step setting */
	const struct exynos_dsi_cmd auto_mode_cmds[] = {
		EXYNOS_DSI_CMD0(mode_set_120hz),
		EXYNOS_DSI_CMD0(auto_step_global_para),
		EXYNOS_DSI_CMD0(step_cmd),
		EXYNOS_DSI_CMD0(auto_mode),
		EXYNOS_DSI_CMD0(freq_update),
	};
	DEFINE_EXYNOS_CMD_SET(auto_mode);
	const u32 flags = PANEL_CMD_SET_IGNORE_VBLANK | PANEL_CMD_SET_BATCH;
	u32 step_vrefresh;

	/* write auto step setting depending on target idle refresh rate */

	if (idle_vrefresh <= 10) {
		/* nothing to update in cmds */
		step_vrefresh = 10;
	} else if (idle_vrefresh <= 30) {
		step_cmd[3] = 0x03;
		step_cmd[5] = 0x02;
		step_vrefresh = 30;
	} else if (idle_vrefresh <= 60) {
		step_cmd[3] = 0x01;
		step_cmd[5] = 0x02;
		step_vrefresh = 60;
	} else {
		dev_err(ctx->dev, "%s: invalid idle fps=%u\n", __func__, idle_vrefresh);
		return;
	}

	dev_dbg(ctx->dev, "%s: sending %uhz step setting (idle_fps=%u)\n",
		__func__, step_vrefresh, idle_vrefresh);

	s6e3hc3_update_early_exit(ctx, true);

	exynos_panel_send_cmd_set_flags(ctx, &auto_mode_cmd_set, flags);

	spanel->auto_mode_vrefresh = step_vrefresh;
}

static void s6e3hc3_set_manual_mode(struct exynos_panel *ctx, const struct exynos_panel_mode *pmode)
{
	const u32 flags = PANEL_CMD_SET_IGNORE_VBLANK | PANEL_CMD_SET_BATCH;
	struct s6e3hc3_panel *spanel = to_spanel(ctx);
	const struct s6e3hc3_mode_data *mdata = pmode->priv_data;
	const struct exynos_dsi_cmd_set *cmdset;

	cmdset = (!spanel->is_hbm_update && ctx->hbm_mode) ?
			mdata->manual_mode_ghbm_cmd_set : mdata->manual_mode_cmd_set;

	if (cmdset)
		exynos_panel_send_cmd_set_flags(ctx, cmdset, flags);

	spanel->auto_mode_vrefresh = 0;
}

static void s6e3hc3_update_refresh_mode(struct exynos_panel *ctx,
					const struct exynos_panel_mode *pmode, int idle_vrefresh)
{
	const u32 flags = PANEL_CMD_SET_IGNORE_VBLANK | PANEL_CMD_SET_BATCH;
	const struct s6e3hc3_mode_data *mdata = pmode->priv_data;

	if (unlikely(!mdata))
		return;

	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);

	if (idle_vrefresh) {
		dev_dbg(ctx->dev, "%s: mode: %s with auto mode idle_vrefresh: %d\n", __func__,
			pmode->mode.name, idle_vrefresh);
		s6e3hc3_set_early_exit_auto_mode(ctx, idle_vrefresh);
	} else {
		dev_dbg(ctx->dev, "%s: mode: %s in manual mode\n", __func__,
			pmode->mode.name);

		s6e3hc3_update_early_exit(ctx, false);

		s6e3hc3_set_manual_mode(ctx, pmode);
	}

	if (mdata->common_mode_cmd_set)
		exynos_panel_send_cmd_set_flags(ctx, mdata->common_mode_cmd_set, flags);

	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);

	/* when mode is explicitly set (manual) panel idle effect would be disabled */
	ctx->panel_idle_vrefresh = 0;
}

static void s6e3hc3_change_frequency(struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	u32 idle_vrefresh = 0;

	if (unlikely(!ctx))
		return;

	if (pmode->idle_mode == IDLE_MODE_ON_INACTIVITY)
		idle_vrefresh = s6e3hc3_get_min_idle_vrefresh(ctx, pmode);

	s6e3hc3_update_refresh_mode(ctx, pmode, idle_vrefresh);

	dev_dbg(ctx->dev, "%s: change to %uhz\n", __func__, drm_mode_vrefresh(&pmode->mode));
}

static bool s6e3hc3_set_self_refresh(struct exynos_panel *ctx, bool enable)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	const struct s6e3hc3_mode_data *mdata;
	struct s6e3hc3_panel *spanel = to_spanel(ctx);
	u16 dsi_flags = PANEL_CMD_SET_IGNORE_VBLANK | PANEL_CMD_SET_BATCH;
	u32 idle_vrefresh;

	if (unlikely(!pmode))
		return false;

	/* self refresh is not supported in lp mode since that always makes use of early exit */
	if (pmode->exynos_mode.is_lp_mode) {
		/* set 10Hz while self refresh is active, otherwise clear it */
		ctx->panel_idle_vrefresh = enable ? 10 : 0;
		notify_panel_mode_changed(ctx, true);
		return false;
	}

	idle_vrefresh = s6e3hc3_get_min_idle_vrefresh(ctx, pmode);

	if (pmode->idle_mode != IDLE_MODE_ON_SELF_REFRESH) {
		/*
		 * if idle mode is on inactivity, may need to update the target fps for auto mode,
		 * or switch to manual mode if idle should be disabled (idle_vrefresh=0)
		 */
		if (pmode->idle_mode == IDLE_MODE_ON_INACTIVITY) {
			/* simply update idle vrefresh follow by self refresh */
			ctx->panel_idle_vrefresh = enable ? idle_vrefresh : 0;
			notify_panel_mode_changed(ctx, false);
			if (spanel->auto_mode_vrefresh != idle_vrefresh) {
				dev_dbg(ctx->dev,
					"early exit update needed for mode: %s (idle_vrefresh: %d)\n",
					pmode->mode.name, idle_vrefresh);
				s6e3hc3_update_refresh_mode(ctx, pmode, idle_vrefresh);
				return true;
			}
		}
		return false;
	}

	if (!enable)
		idle_vrefresh = 0;

	/* if there's no change in idle state then skip cmds */
	if (ctx->panel_idle_vrefresh == idle_vrefresh)
		return false;

	DPU_ATRACE_BEGIN(__func__);
	ctx->panel_idle_vrefresh = idle_vrefresh;

	dev_dbg(ctx->dev, "change panel idle vrefresh: %u for mode: %s\n", idle_vrefresh,
		pmode->mode.name);

	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	if (idle_vrefresh) {
		s6e3hc3_set_early_exit_auto_mode(ctx, idle_vrefresh);
	} else {
		mdata = pmode->priv_data;
		if (unlikely(!mdata)) {
			dev_err(ctx->dev, "%s: failed to get mode data\n", __func__);
			EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);
			return false;
		}
		/* disable early exit after coming out of idle */
		s6e3hc3_update_early_exit(ctx, false);
		spanel->auto_mode_vrefresh = 0;

		exynos_panel_send_cmd_set_flags(ctx, mdata->wakeup_mode_cmd_set, dsi_flags);
	}
	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);

	notify_panel_mode_changed(ctx, false);

	DPU_ATRACE_END(__func__);

	return true;
}

static int s6e3hc3_atomic_check(struct exynos_panel *ctx, struct drm_atomic_state *state)
{
	struct drm_connector *conn = &ctx->exynos_connector.base;
	struct drm_connector_state *new_conn_state = drm_atomic_get_new_connector_state(state, conn);
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	struct s6e3hc3_panel *spanel = to_spanel(ctx);

	if (!ctx->current_mode || drm_mode_vrefresh(&ctx->current_mode->mode) == 120 ||
	    !new_conn_state || !new_conn_state->crtc)
		return 0;

	new_crtc_state = drm_atomic_get_new_crtc_state(state, new_conn_state->crtc);
	old_crtc_state = drm_atomic_get_old_crtc_state(state, new_conn_state->crtc);
	if (!old_crtc_state || !new_crtc_state || !new_crtc_state->active)
		return 0;

	if ((spanel->auto_mode_vrefresh && old_crtc_state->self_refresh_active) ||
	    !drm_atomic_crtc_effectively_active(old_crtc_state)) {
		struct drm_display_mode *mode = &new_crtc_state->adjusted_mode;

		/* set clock to max refresh rate on self refresh exit or resume due to early exit */
		mode->clock = mode->htotal * mode->vtotal * 120 / 1000;

		if (mode->clock != new_crtc_state->mode.clock) {
			new_crtc_state->mode_changed = true;
			dev_dbg(ctx->dev, "raise mode (%s) clock to 120hz on %s\n",
				mode->name,
				old_crtc_state->self_refresh_active ? "self refresh exit" : "resume");
		}
	} else if (old_crtc_state->active_changed &&
		   (old_crtc_state->adjusted_mode.clock != old_crtc_state->mode.clock)) {
		/* clock hacked in last commit due to self refresh exit or resume, undo that */
		new_crtc_state->mode_changed = true;
		new_crtc_state->adjusted_mode.clock = new_crtc_state->mode.clock;
		dev_dbg(ctx->dev, "restore mode (%s) clock after self refresh exit or resume\n",
			new_crtc_state->mode.name);
	}

	return 0;
}

static void s6e3hc3_write_display_mode(struct exynos_panel *ctx,
				       const struct drm_display_mode *mode)
{
	u8 val = S6E3HC3_WRCTRLD_BCTRL_BIT;

	if (IS_HBM_ON(ctx->hbm_mode))
		val |= S6E3HC3_WRCTRLD_HBM_BIT;

	if (ctx->hbm.local_hbm.enabled)
		val |= S6E3HC3_WRCTRLD_LOCAL_HBM_BIT;

	if (ctx->dimming_on)
		val |= S6E3HC3_WRCTRLD_DIMMING_BIT;

	dev_dbg(ctx->dev,
		"%s(wrctrld:0x%x, hbm: %s, dimming: %s local_hbm: %s)\n",
		__func__, val, IS_HBM_ON(ctx->hbm_mode) ? "on" : "off",
		ctx->dimming_on ? "on" : "off",
		ctx->hbm.local_hbm.enabled ? "on" : "off");

	EXYNOS_DCS_WRITE_SEQ(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);
}

static void s6e3hc3_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	unsigned int vrefresh = drm_mode_vrefresh(&pmode->mode);
	u32 delay_us = mult_frac(1000, 1020, vrefresh);

	if (!ctx->enabled)
		return;

	EXYNOS_DCS_WRITE_TABLE(ctx, display_off);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0x5A, 0x5A);
	/* backlight control and dimming */
	s6e3hc3_write_display_mode(ctx, &pmode->mode);
	if (ctx->panel_rev == PANEL_REV_PROTO1)
		EXYNOS_DCS_WRITE_SEQ(ctx, 0x49, 0x02);	/* normal gamma */
	s6e3hc3_change_frequency(ctx, pmode);
	usleep_range(delay_us, delay_us + 10);
	EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	dev_info(ctx->dev, "exit LP mode\n");
}

static const struct exynos_dsi_cmd s6e3hc3_init_cmds[] = {
	EXYNOS_DSI_CMD_SEQ(0x9D, 0x01),			/* PPS_DSC_EN */

	EXYNOS_DSI_CMD_SEQ_DELAY(120, 0x11),		/* sleep out: 120ms delay */

	EXYNOS_DSI_CMD_SEQ(0x35),			/* TE on */

	EXYNOS_DSI_CMD0(unlock_cmd_f0),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1), 0xB0, 0x00, 0x06, 0xB9),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1),
			0xB9, 0x0C, 0x44, 0x0C, 0x44, 0x00, 0x1C), /* TE Setting */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x22, 0xB9),	/* SEQ_GLOBAL_TSP_SYNC */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0xB1, 0xA1),		/* SEQ_TSP_SYNC_ON */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x05, 0xF2),	/* SEQ_GLOBAL_TSP_SYNC */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x52),			/* SEQ_TSP_SYNC_ON */
	EXYNOS_DSI_CMD0(lock_cmd_f0),

	EXYNOS_DSI_CMD_SEQ(0x2A, 0x00, 0x00, 0x05, 0x9F), /* CASET */
	EXYNOS_DSI_CMD_SEQ(0x2B, 0x00, 0x00, 0x0C, 0x2F), /* PASET */
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_init);

#define S6E3HC3_LOCAL_HBM_GAMMA_CMD_SIZE 6
#define S6E3HC3_PANEL_ID3_DOE            0x20
#define S6E3HC3_PANEL_ID3_DOE3           0x29
static bool s6e3hc3_get_panel_id3(struct exynos_panel *ctx, u32 *out)
{
	if (kstrtou32(ctx->panel_extinfo, 16, out)) {
		dev_err(ctx->dev, "failed to get panel extinfo\n");
		return false;
	}
	*out = *out & 0xff;
	return true;
}

static bool s6e3hc3_panel_id3_doe3_config(struct exynos_panel *ctx)
{
	u32 id;
	bool ret = false;

	if (s6e3hc3_get_panel_id3(ctx, &id))
		if (id == S6E3HC3_PANEL_ID3_DOE3)
			ret = true;

	return ret;
}

static int s6e3hc3_lhbm_gamma_read(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	u8 *gamma_cmd = ctx->hbm.local_hbm.gamma_cmd;
	int ret;

	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x00, 0x22, 0xD8); /* global para */
	ret = mipi_dsi_dcs_read(dsi, 0xD8, gamma_cmd + 1,
			 S6E3HC3_LOCAL_HBM_GAMMA_CMD_SIZE - 1);
	if (ret == (S6E3HC3_LOCAL_HBM_GAMMA_CMD_SIZE -1)) {
		gamma_cmd[0] = 0x66;
		ctx->hbm.local_hbm.gamma_para_ready = true;
		ret = 0;
	} else {
		ret = -EIO;
		dev_err(ctx->dev, "fail to read LHBM gamma\n");
	}
	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);
	return ret;
}

static void s6e3hc3_lhbm_gamma_write(struct exynos_panel *ctx)
{
	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x01, 0x59, 0x66);
	exynos_dcs_write(ctx, ctx->hbm.local_hbm.gamma_cmd,
			 S6E3HC3_LOCAL_HBM_GAMMA_CMD_SIZE); /* write gamma */
	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);
}

static void s6e3hc3_extra_lhbm_settings(struct exynos_panel *ctx,
				 bool local_hbm_en)
{
	dev_dbg(ctx->dev, "%s(panel_rev: 0x%x)\n", __func__, ctx->panel_rev);
	if (ctx->panel_rev >= PANEL_REV_EVT1) {
		EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, unlock_cmd_f0, 0);
		if (local_hbm_en) {
			if (ctx->panel_rev == PANEL_REV_EVT1) {
				u32 id = 0;
				bool is_doe_cfg = false;

				if (!s6e3hc3_get_panel_id3(ctx, &id))
					dev_err(ctx->dev,
						"fail to get panel id and load evt1 por settings!\n");
				is_doe_cfg = (id & S6E3HC3_PANEL_ID3_DOE) != 0;

				if (is_doe_cfg) {
					EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, unlock_cmd_f1, 0);
					/* 2 cycle */
					EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xBD, 0x21, 0x81);
					EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xB1, 0x90);
				} else {
					/* 1 cycle */
					EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xBD, 0x21, 0x80);
				}
				/* 120 hz */
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0x60, 0x00);
				/* update key */
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xF7, 0x0F);
				/* global para */
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xB0, 0x01, 0xE7, 0x1F);
				/* center posi. */
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0x1F, 0x2D, 0x09, 0x39);
				if (is_doe_cfg)
					EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, lock_cmd_f1, 0);
			} else {
				EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, unlock_cmd_f1, 0);
				/* 2 cycle */
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xBD, 0x21, 0x81);
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xB1, 0x90);
				/* 120 hz */
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0x60, 0x00);
				/* update key */
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xF7, 0x0F);
				EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, lock_cmd_f1, 0);
			}
		} else {
			const int vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);

			EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xBD, 0x21, 0x82);
			EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xB1, 0x00);
			if (vrefresh == 60)
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0x60, 0x01);
			else if (vrefresh == 120)
				EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0x60, 0x00);
			else
				dev_err(ctx->dev, "unsupported refresh rate!\n");
			EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, 0xF7, 0x0F);
		}
		EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, lock_cmd_f0, 0);
	}
}

static inline bool is_local_gamma_supported(struct exynos_panel *ctx)
{
	if ((ctx->panel_rev == PANEL_REV_EVT1) &&
			 s6e3hc3_panel_id3_doe3_config(ctx))
		return true;
	if (ctx->panel_rev >= PANEL_REV_EVT1_1)
		return true;
	return false;
}

static int s6e3hc3_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	const struct drm_display_mode *mode;
	const bool needs_reset = !is_panel_enabled(ctx);
	bool is_fhd;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}
	mode = &pmode->mode;
	is_fhd = mode->hdisplay == 1080;

	dev_dbg(ctx->dev, "%s\n", __func__);

	if (needs_reset)
		exynos_panel_reset(ctx);

	/* DSC related configuration */
	EXYNOS_PPS_WRITE_BUF(ctx, is_fhd ? FHD_PPS_SETTING : WQHD_PPS_SETTING);
	if (needs_reset)
		exynos_panel_send_cmd_set(ctx, &s6e3hc3_init_cmd_set);

	EXYNOS_DCS_BUF_ADD_SET(ctx, unlock_cmd_f0);
	EXYNOS_DCS_BUF_ADD(ctx, 0xC3, is_fhd ? 0x01: 0x00);
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, lock_cmd_f0);
	s6e3hc3_write_display_mode(ctx, mode); /* dimming and HBM */
	s6e3hc3_change_frequency(ctx, pmode);
	if (is_local_gamma_supported(ctx))
		if (ctx->hbm.local_hbm.gamma_para_ready)
			s6e3hc3_lhbm_gamma_write(ctx);


	ctx->enabled = true;

	if (pmode->exynos_mode.is_lp_mode)
		exynos_panel_set_lp_mode(ctx, pmode);
	else if (needs_reset || (ctx->panel_state == PANEL_STATE_BLANK))
		EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	return 0;
}

static int s6e3hc3_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	int ret;

	/* skip disable sequence if going through modeset */
	if (ctx->panel_state == PANEL_STATE_MODESET)
		return 0;

	ret = exynos_panel_disable(panel);
	if (ret)
		return ret;

	EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, 20, display_off);

	if (ctx->panel_state == PANEL_STATE_OFF)
		EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, 100, sleep_in);

	return 0;
}

/*
 * 120hz auto mode takes at least 2 frames to start lowering refresh rate in addition to
 * time to next vblank. Use just over 2 frames time to consider worst case scenario
 */
#define EARLY_EXIT_THRESHOLD_US 17000

/**
 * s6e3hc3_trigger_early_exit - trigger early exit command to panel
 * @ctx: panel struct
 *
 * Sends a command to panel to indicate a frame is about to come in case its been a while since the
 * last frame update and auto mode may have started to take effect and lowering refresh rate
 */
static void s6e3hc3_trigger_early_exit(struct exynos_panel *ctx)
{
	const ktime_t delta = ktime_sub(ktime_get(), ctx->last_commit_ts);
	const s64 delta_us = ktime_to_us(delta);

	if (delta_us < EARLY_EXIT_THRESHOLD_US) {
		dev_dbg(ctx->dev, "skip early exit. %lldus since last commit\n",
			delta_us);
		return;
	}
	/* triggering early exit causes a switch to 120hz */
	ctx->last_mode_set_ts = ktime_get();

	DPU_ATRACE_BEGIN(__func__);
	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	if (ctx->idle_delay_ms) {
		const struct exynos_panel_mode *pmode = ctx->current_mode;

		/*
		 * setting manual mode (at 120hz) removes any effect from early exit,
		 * so there's no need to update early exit status
		 */
		s6e3hc3_set_manual_mode(ctx, pmode);

		dev_dbg(ctx->dev, "%s: set manual mode for: %s\n", __func__, pmode->mode.name);
	} else {
		dev_dbg(ctx->dev, "sending early exit out cmd\n");
		EXYNOS_DCS_WRITE_TABLE(ctx, freq_update);
	}
	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);
	DPU_ATRACE_END(__func__);
}

static void s6e3hc3_commit_done(struct exynos_panel *ctx)
{
	const struct s6e3hc3_mode_data *mdata;
	const struct s6e3hc3_panel *spanel = to_spanel(ctx);

	if (!ctx->enabled || ctx->current_mode->exynos_mode.is_lp_mode)
		return;

	mdata = ctx->current_mode->priv_data;
	if (!mdata)
		return;

	if (spanel->auto_mode_vrefresh)
		s6e3hc3_trigger_early_exit(ctx);
}

static void s6e3hc3_set_hbm_brightness(struct exynos_panel *ctx,
				bool is_hbm_on)
{
	u32 level;

	if (is_hbm_on)
		level = ctx->desc->brt_capability->hbm.level.min;
	else
		level = ctx->desc->brt_capability->normal.level.max;
	exynos_panel_set_brightness(ctx, level);
}

static void s6e3hc3_set_hbm_setting(struct exynos_panel *ctx,
				const struct drm_display_mode *mode, bool is_hbm_on)
{
	const int vrefresh = drm_mode_vrefresh(mode);

	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	if (ctx->panel_rev == PANEL_REV_PROTO1) {
		if (is_hbm_on) {
			EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x00, 0x01, 0x49);
			EXYNOS_DCS_WRITE_SEQ(ctx, 0x49, 0x00);
			usleep_range(17000, 17010);
		} else {
			usleep_range(17000, 17010);
			EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x00, 0x01, 0x49);
			EXYNOS_DCS_WRITE_SEQ(ctx, 0x49, 0x01);
		}
	} else if (ctx->panel_rev >= PANEL_REV_EVT1_1) {
		if (vrefresh == 60)
			EXYNOS_DCS_WRITE_TABLE(ctx, mode_set_60hz);
		else if (vrefresh == 120)
			EXYNOS_DCS_WRITE_TABLE(ctx, mode_set_120hz);
		EXYNOS_DCS_WRITE_TABLE(ctx, freq_update);
	}
	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);

	s6e3hc3_write_display_mode(ctx, mode);
	s6e3hc3_set_hbm_brightness(ctx, is_hbm_on);
}

static void s6e3hc3_set_hbm_mode(struct exynos_panel *ctx,
				 enum exynos_hbm_mode mode)
{
	struct s6e3hc3_panel *spanel = to_spanel(ctx);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	const bool hbm_update =
		(IS_HBM_ON(ctx->hbm_mode) != IS_HBM_ON(mode));
	const bool irc_update =
		(IS_HBM_ON_IRC_OFF(ctx->hbm_mode) != IS_HBM_ON_IRC_OFF(mode));

	if (mode == ctx->hbm_mode)
		return;

	if (unlikely(!pmode || !pmode->priv_data))
		return;

	ctx->hbm_mode = mode;

	if (hbm_update) {
		const bool is_hbm_on = IS_HBM_ON(mode);

		spanel->is_hbm_update = hbm_update;
		if (is_hbm_on)
			s6e3hc3_set_self_refresh(ctx, false);
		s6e3hc3_set_hbm_setting(ctx, &pmode->mode, is_hbm_on);
		if (!is_hbm_on)
			s6e3hc3_set_self_refresh(ctx, ctx->self_refresh_active);
		spanel->is_hbm_update = false;
	}
	if (irc_update) {
		EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
		EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x02, 0xB6, 0x1D);
		EXYNOS_DCS_WRITE_SEQ(ctx, 0x1D, IS_HBM_ON_IRC_OFF(mode) ? 0x05 : 0x25);
		EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);
	}

	dev_info(ctx->dev, "hbm_on=%d hbm_ircoff=%d\n", IS_HBM_ON(ctx->hbm_mode),
		 IS_HBM_ON_IRC_OFF(ctx->hbm_mode));
}

static void s6e3hc3_set_dimming_on(struct exynos_panel *ctx,
				 bool dimming_on)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	ctx->dimming_on = dimming_on;
	if (pmode->exynos_mode.is_lp_mode) {
		dev_info(ctx->dev,"in lp mode, skip to update");
		return;
	}
	s6e3hc3_write_display_mode(ctx, &pmode->mode);
}

static void s6e3hc3_set_local_hbm_mode(struct exynos_panel *ctx,
				 bool local_hbm_en)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	s6e3hc3_extra_lhbm_settings(ctx, local_hbm_en);
	s6e3hc3_write_display_mode(ctx, &pmode->mode);
}

static void s6e3hc3_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{
	s6e3hc3_change_frequency(ctx, pmode);
}

static bool s6e3hc3_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	const struct drm_display_mode *c = &ctx->current_mode->mode;
	const struct drm_display_mode *n = &pmode->mode;

	/* seamless mode set can happen if active region resolution is same */
	return (c->vdisplay == n->vdisplay) && (c->hdisplay == n->hdisplay) &&
	       (c->flags == n->flags);
}

static void s6e3hc3_get_panel_rev(struct exynos_panel *ctx, u32 id)
{
	/* extract command 0xDB */
	u8 build_code = (id & 0xFF00) >> 8;
	u8 rev = ((build_code & 0xE0) >> 3) | (build_code & 0x03);

	exynos_panel_get_panel_rev(ctx, rev);
}

static const struct exynos_display_underrun_param underrun_param = {
	.te_idle_us = 350,
	.te_var = 1,
};

static const u32 s6e3hc3_bl_range[] = {
	94, 180, 270, 360, 2047
};

static const int s6e3hc3_vrefresh_range[] = {
	10, 30, 60, 120
};

static const int s6e3hc3_lp_vrefresh_range[] = {
	10, 30
};

static const struct exynos_panel_mode s6e3hc3_modes[] = {
	{
		.mode = {
			.name = "1440x3120x60",
			.clock = 298620,
			.hdisplay = 1440,
			.hsync_start = 1440 + 80, // add hfp
			.hsync_end = 1440 + 80 + 24, // add hsa
			.htotal = 1440 + 80 + 24 + 36, // add hbp
			.vdisplay = 3120,
			.vsync_start = 3120 + 12, // add vfp
			.vsync_end = 3120 + 12 + 4, // add vsa
			.vtotal = 3120 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 8500,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 52,
			},
			.underrun_param = &underrun_param,
		},
		.priv_data = &s6e3hc3_mode_60,
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_UNSUPPORTED,
	},
	{
		.mode = {
			.name = "1440x3120x120",
			.clock = 597240,
			.hdisplay = 1440,
			.hsync_start = 1440 + 80, // add hfp
			.hsync_end = 1440 + 80 + 24, // add hsa
			.htotal = 1440 + 80 + 24 + 36, // add hbp
			.vdisplay = 3120,
			.vsync_start = 3120 + 12, // add vfp
			.vsync_end = 3120 + 12 + 4, // add vsa
			.vtotal = 3120 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 146,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 52,
			},
			.underrun_param = &underrun_param,
		},
		.priv_data = &s6e3hc3_mode_120,
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_ON_INACTIVITY,
	},
	{
		.mode = {
			.name = "1080x2340x60",
			.clock = 173484,
			.hdisplay = 1080,
			.hsync_start = 1080 + 80, // add hfp
			.hsync_end = 1080 + 80 + 24, // add hsa
			.htotal = 1080 + 80 + 24 + 36, // add hbp
			.vdisplay = 2340,
			.vsync_start = 2340 + 12, // add vfp
			.vsync_end = 2340 + 12 + 4, // add vsa
			.vtotal = 2340 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 8500,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 78,
			},
			.underrun_param = &underrun_param,
		},
		.priv_data = &s6e3hc3_mode_60,
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_UNSUPPORTED,
	},
	{
		.mode = {
			.name = "1080x2340x120",
			.clock = 346968,
			.hdisplay = 1080,
			.hsync_start = 1080 + 80, // add hfp
			.hsync_end = 1080 + 80 + 24, // add hsa
			.htotal = 1080 + 80 + 24 + 36, // add hbp
			.vdisplay = 2340,
			.vsync_start = 2340 + 12, // add vfp
			.vsync_end = 2340 + 12 + 4, // add vsa
			.vtotal = 2340 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 146,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 78,
			},
			.underrun_param = &underrun_param,
		},
		.priv_data = &s6e3hc3_mode_120,
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_ON_INACTIVITY,
	},
};

static const struct exynos_panel_mode s6e3hc3_lp_modes[] = {
	{
		.mode = {
			.name = "1440x3120x30",
			.clock = 149310,
			.hdisplay = 1440,
			.hsync_start = 1440 + 80, // add hfp
			.hsync_end = 1440 + 80 + 24, // add hsa
			.htotal = 1440 + 80 + 24 + 36, // add hbp
			.vdisplay = 3120,
			.vsync_start = 3120 + 12, // add vfp
			.vsync_end = 3120 + 12 + 4, // add vsa
			.vtotal = 3120 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 25200,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 52,
			},
			.underrun_param = &underrun_param,
			.is_lp_mode = true,
		},
	},
	{
		.mode = {
			.name = "1080x2340x30",
			.clock = 86742,
			.hdisplay = 1080,
			.hsync_start = 1080 + 80, // add hfp
			.hsync_end = 1080 + 80 + 24, // add hsa
			.htotal = 1080 + 80 + 24 + 36, // add hbp
			.vdisplay = 2340,
			.vsync_start = 2340 + 12, // add vfp
			.vsync_end = 2340 + 12 + 4, // add vsa
			.vtotal = 2340 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 25200,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 78,
			},
			.underrun_param = &underrun_param,
			.is_lp_mode = true,
		},
	},
};

static void s6e3hc3_panel_mode_create_cmdset(struct exynos_panel *ctx,
					     struct dentry *parent,
					     const struct exynos_panel_mode *pmode)
{
	struct dentry *root;
	const struct s6e3hc3_mode_data *mdata = pmode->priv_data;

	if (!mdata)
		return;

	root = debugfs_create_dir(pmode->mode.name, parent);
	if (!root) {
		dev_err(ctx->dev, "unable to create %s mode debugfs dir\n", pmode->mode.name);
		return;
	}

	exynos_panel_debugfs_create_cmdset(ctx, root, mdata->manual_mode_cmd_set, "manual_mode");
	exynos_panel_debugfs_create_cmdset(ctx, root, mdata->manual_mode_ghbm_cmd_set,
					   "manual_ghbm_mode");
	exynos_panel_debugfs_create_cmdset(ctx, root, mdata->common_mode_cmd_set, "common_mode");
	exynos_panel_debugfs_create_cmdset(ctx, root, mdata->wakeup_mode_cmd_set, "wakeup");
}

static void s6e3hc3_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	struct dentry *panel_root, *csroot;
	struct s6e3hc3_panel *spanel;
	int i;

	if (!ctx)
		return;

	panel_root = debugfs_lookup("panel", root);
	if (!panel_root)
		return;

	csroot = debugfs_lookup("cmdsets", panel_root);
	if (!csroot) {
		dput(panel_root);
		return;
	}

	spanel = to_spanel(ctx);

	debugfs_create_bool("force_changeable_te", 0644, panel_root,
			    &spanel->force_changeable_te);
	exynos_panel_debugfs_create_cmdset(ctx, csroot, &s6e3hc3_init_cmd_set, "init");
	exynos_panel_debugfs_create_cmdset(ctx, csroot, &s6e3hc3_early_exit_enable_cmd_set,
					   "early_exit_enable");
	exynos_panel_debugfs_create_cmdset(ctx, csroot, &s6e3hc3_early_exit_disable_cmd_set,
					   "early_exit_disable");
	for (i = 0; i < ctx->desc->num_modes; i++)
		s6e3hc3_panel_mode_create_cmdset(ctx, csroot, &ctx->desc->modes[i]);

	dput(csroot);
	dput(panel_root);
}

static void s6e3hc3_panel_init(struct exynos_panel *ctx)
{
	if (is_local_gamma_supported(ctx))
		if (!s6e3hc3_lhbm_gamma_read(ctx))
			s6e3hc3_lhbm_gamma_write(ctx);
}

static int s6e3hc3_panel_probe(struct mipi_dsi_device *dsi)
{
	struct s6e3hc3_panel *spanel;

	spanel = devm_kzalloc(&dsi->dev, sizeof(*spanel), GFP_KERNEL);
	if (!spanel)
		return -ENOMEM;

	return exynos_panel_common_init(dsi, &spanel->base);
}

static const struct drm_panel_funcs s6e3hc3_drm_funcs = {
	.disable = s6e3hc3_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3hc3_enable,
	.get_modes = exynos_panel_get_modes,
	.debugfs_init = s6e3hc3_debugfs_init,
};

static const struct exynos_panel_funcs s6e3hc3_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_lp_mode = exynos_panel_set_lp_mode,
	.set_nolp_mode = s6e3hc3_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.set_hbm_mode = s6e3hc3_set_hbm_mode,
	.set_dimming_on = s6e3hc3_set_dimming_on,
	.set_local_hbm_mode = s6e3hc3_set_local_hbm_mode,
	.is_mode_seamless = s6e3hc3_is_mode_seamless,
	.mode_set = s6e3hc3_mode_set,
	.panel_init = s6e3hc3_panel_init,
	.get_panel_rev = s6e3hc3_get_panel_rev,
	.get_te2_edges = exynos_panel_get_te2_edges,
	.configure_te2_edges = exynos_panel_configure_te2_edges,
	.update_te2 = s6e3hc3_update_te2,
	.commit_done = s6e3hc3_commit_done,
	.atomic_check = s6e3hc3_atomic_check,
	.set_self_refresh = s6e3hc3_set_self_refresh,
};

const struct brightness_capability s6e3hc3_brightness_capability = {
	.normal = {
		.nits = {
			.min = 2,
			.max = 500,
		},
		.level = {
			.min = 4,
			.max = 2047,
		},
		.percentage = {
			.min = 0,
			.max = 62,
		},
	},
	.hbm = {
		.nits = {
			.min = 550,
			.max = 800,
		},
		.level = {
			.min = 2232,
			.max = 3152,
		},
		.percentage = {
			.min = 62,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc samsung_s6e3hc3 = {
	.data_lane_cnt = 4,
	.max_brightness = 3152,
	.dft_brightness = 1023,
	.brt_capability = &s6e3hc3_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 8000000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = s6e3hc3_bl_range,
	.bl_num_ranges = ARRAY_SIZE(s6e3hc3_bl_range),
	.modes = s6e3hc3_modes,
	.num_modes = ARRAY_SIZE(s6e3hc3_modes),
	.vrefresh_range = s6e3hc3_vrefresh_range,
	.vrefresh_range_count = ARRAY_SIZE(s6e3hc3_vrefresh_range),
	.lp_cmd_set = &s6e3hc3_lp_cmd_set,
	.lp_mode = s6e3hc3_lp_modes,
	.lp_mode_count = ARRAY_SIZE(s6e3hc3_lp_modes),
	.lp_vrefresh_range = s6e3hc3_lp_vrefresh_range,
	.lp_vrefresh_range_count = ARRAY_SIZE(s6e3hc3_lp_vrefresh_range),
	.binned_lp = s6e3hc3_binned_lp,
	.num_binned_lp = ARRAY_SIZE(s6e3hc3_binned_lp),
	.is_panel_idle_supported = true,
	.panel_func = &s6e3hc3_drm_funcs,
	.exynos_panel_func = &s6e3hc3_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,s6e3hc3", .data = &samsung_s6e3hc3 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = s6e3hc3_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-s6e3hc3",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung s6e3hc3 panel driver");
MODULE_LICENSE("GPL");
