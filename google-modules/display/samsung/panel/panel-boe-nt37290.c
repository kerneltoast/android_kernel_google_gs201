// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based nt37290 AMOLED LCD panel driver.
 *
 * Copyright (c) 2021 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <drm/drm_file.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <trace/dpu_trace.h>
#include <video/mipi_display.h>
#include <drm/drm_vblank.h>

#include "panel-samsung-drv.h"

/* when refresh rate can go lower than this value (in auto mode), fixed TE2 should be enabled */
#define NT37290_TE2_MIN_RATE   30
#define NT37290_TE2_CHANGEABLE 0x02
#define NT37290_TE2_FIXED      0x22

#define NT37290_DDIC_ID_LEN 8

/**
 * enum nt37290_panel_feature - features supported by this panel
 * @G10_FEAT_EARLY_EXIT: early exit from a long frame
 * @G10_FEAT_FRAME_AUTO: automatic (not manual) frame control
 * @G10_FEAT_MAX: placeholder, counter for number of features
 *
 * The following features are correlated, if one or more of them change, the others need
 * to be updated unconditionally.
 */
enum nt37290_panel_feature {
	G10_FEAT_EARLY_EXIT = 0,
	G10_FEAT_FRAME_AUTO,
	G10_FEAT_MAX,
};

/**
 * enum nt37290_dfc_mode - dynamic frame rate control mode
 * @DFC_MODE_MANUAL: manual mode, running at based frame rate
 * @DFC_MODE_PANEL_LP: low power mode, running at lower frame rates
 *
 * Select DDIC frame rate control mode. For 120Hz based, MANUAL will use 120Hz, and
 * PANEL_LP will use 60/30/10Hz. For 30Hz based (AOD), MANUAL will use 30Hz, and
 * PANEL_LP will use 10Hz.
 */
enum nt37290_dfc_mode {
	DFC_MODE_MANUAL = 0,
	DFC_MODE_PANEL_LP = 3,
};

/**
 * struct nt37290_panel - panel specific runtime info
 *
 * This struct maintains nt37290 panel specific runtime info, any fixed details about panel
 * should most likely go into struct exynos_panel_desc. The variables with the prefix hw_ keep
 * track of the features that were actually committed to hardware, and should be modified
 * after sending cmds to panel, i.e. updating hw state.
 */
struct nt37290_panel {
	/** @base: base panel struct */
	struct exynos_panel base;
	/** @feat: software/working correlated features, not guaranteed to be effective in panel */
	DECLARE_BITMAP(feat, G10_FEAT_MAX);
	/** @hw_feat: correlated states effective in panel */
	DECLARE_BITMAP(hw_feat, G10_FEAT_MAX);
	/** @hw_vrefresh: vrefresh rate effective in panel */
	int hw_vrefresh;
	/** @hw_idle_vrefresh: idle vrefresh rate effective in panel */
	int hw_idle_vrefresh;
	/** @hw_dbv: hw brightness value sent to panel */
	u16 hw_dbv;
	/**
	 * @auto_mode_vrefresh: indicates current minimum refresh rate while in auto mode,
	 *                      if 0 it means that auto mode is not enabled
	 */
	u32 auto_mode_vrefresh;
	/**
	 * @delayed_idle: indicates idle mode set is delayed due to idle_delay_ms,
	 *                we should avoid changing idle_mode when it's true
	 */
	bool delayed_idle;
	/** @hw_osc2_clk_idx: current index of OSC2 clock table in panel */
	int hw_osc2_clk_idx;
	/** @rrs_in_progress: indicate whether RRS (Runtime Resolution Switch) is in progress */
	bool rrs_in_progress;
};

#define to_spanel(ctx) container_of(ctx, struct nt37290_panel, base)

/**
 * struct nt37290_osc2_clk_data - OSC2 clock info in panel
 *
 * This struct includes the panel OSC2 clock (kHz) and its value (FCON[2:0]) of frame
 * rate selection command (2Fh). Selecting appropriate clock dynamically in normal and
 * AOD modes can reduce radio interferences.
 */
struct nt37290_osc2_clk_data {
	/** @clk_khz: the clock in kHz */
	unsigned int clk_khz;
	/** @fcon_val: the value of FCON[2:0] in command 2Fh */
	u8 fcon_val;
};

static const u8 display_off[] = { 0x28 };
static const u8 display_on[] = { 0x29 };
static const u8 cmd2_page0[] = { 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00 };
static const u8 stream_2c[] = { 0x2C };

static const struct exynos_dsi_cmd nt37290_lp_cmds[] = {
	/* enter AOD */
	EXYNOS_DSI_CMD_SEQ(0x39),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_lp);

static const struct exynos_dsi_cmd nt37290_lp_off_cmds[] = {
	EXYNOS_DSI_CMD0(display_off),
};

static const struct exynos_dsi_cmd nt37290_lp_low_cmds[] = {
	/* 10 nit */
	EXYNOS_DSI_CMD_SEQ_DELAY(9, 0x51, 0x00, 0x00, 0x00, 0x00, 0x03, 0x33),
	/* 2Ch needs to be sent twice in next 2 vsync */
	EXYNOS_DSI_CMD(stream_2c, 9),
	EXYNOS_DSI_CMD0(stream_2c),
	EXYNOS_DSI_CMD0(display_on),
};

static const struct exynos_dsi_cmd nt37290_lp_high_cmds[] = {
	/* 50 nit */
	EXYNOS_DSI_CMD_SEQ_DELAY(9, 0x51, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFE),
	/* 2Ch needs to be sent twice in next 2 vsync */
	EXYNOS_DSI_CMD(stream_2c, 9),
	EXYNOS_DSI_CMD0(stream_2c),
	EXYNOS_DSI_CMD0(display_on),
};

static const struct exynos_binned_lp nt37290_binned_lp[] = {
	BINNED_LP_MODE("off", 0, nt37290_lp_off_cmds),
	/* rising = 0, falling = 48 */
	BINNED_LP_MODE_TIMING("low", 360, nt37290_lp_low_cmds, 0, 48),
	BINNED_LP_MODE_TIMING("high", 3584, nt37290_lp_high_cmds, 0, 48),
};

static const struct exynos_dsi_cmd nt37290_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 100),
	EXYNOS_DSI_CMD_SEQ_DELAY(120, 0x10),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_off);

static const struct exynos_dsi_cmd nt37290_lhbm_on_setting_cmds[] = {
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x07),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xC0, 0xB1),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x08),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xC0, 0x55),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xD5, 0x21, 0x00, 0x39, 0x31, 0x39,
				0x31, 0x00, 0x00, 0x3F, 0xC9, 0xEF, 0xAE, 0x3F, 0xC9, 0xEF, 0xAE,
				0x00, 0x0C, 0xC6, 0xDB, 0x61, 0x23, 0x00, 0x00, 0x79, 0x00, 0x00,
				0x79, 0x33, 0xF0, 0x87, 0x87, 0x39, 0x31, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xD6, 0x27, 0x00, 0x39, 0x31, 0x39,
				0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xC9, 0xEF, 0xAE,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x7A, 0xF3, 0x00, 0x00,
				0x79, 0x33, 0x30, 0x79, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xD7, 0x2B, 0x00, 0x39, 0x31, 0x39,
				0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x7F, 0xF3, 0x39, 0x24, 0x9F, 0x55, 0x00, 0x7A, 0xF3, 0x00, 0x7A,
				0xF3, 0x33, 0x0F, 0x79, 0x79, 0xC6, 0xCF, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xD8, 0x2D, 0x00, 0x39, 0x31, 0x39,
				0x31, 0x00, 0x00, 0x3F, 0xC9, 0xEF, 0xAE, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x00, 0x00, 0x79, 0x00, 0x7A,
				0xF3, 0x33, 0xC0, 0x87, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	EXYNOS_DSI_CMD0_REV(cmd2_page0, PANEL_REV_EVT1),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x05),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x02),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x13),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x00, 0x7A, 0x00, 0x7A),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x1B),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x00, 0x00, 0x00, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x1F),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x00, 0xF3, 0x00, 0xF3),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x2B),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x3F, 0xFF, 0x3F, 0xFF, 0x3F,
				0xFF),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x31),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x22),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x32),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x2A),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x33),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x2A),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x34),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x16),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x35),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x36),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x02),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x37),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x01),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x38),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x0C, 0x38),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x3A),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x01, 0x1F, 0x00, 0x61, 0x00,
				0x93),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x40),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x00, 0xF8, 0x01, 0x07, 0x00,
				0x2E),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x46),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x00, 0x99, 0x00, 0x29, 0x00,
				0x88),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x4C),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x1F, 0xFC, 0x1F, 0xFC, 0x1F,
				0xFC),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x52),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x0A, 0x99, 0x22, 0xDA, 0x3E,
				0xB5),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x58),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x3D, 0xDC, 0x28, 0xD5, 0x1D,
				0x52),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x5E),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x13, 0x51, 0x13, 0xCD, 0x0D,
				0x4E),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x64),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x3B, 0x3F, 0x2E, 0x39, 0x35,
				0xF2),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x6A),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x25, 0x35, 0x18, 0x3C, 0x30,
				0xCF),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x70),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x3E, 0xD6, 0x03, 0xE4, 0x3F,
				0xF5),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x76),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x23, 0x19, 0x1C, 0x89, 0x37,
				0x4B),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x7C),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x3F, 0x69, 0x0A, 0xC7, 0x3C,
				0xB5),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x82),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x13, 0x61, 0x1E, 0x2E, 0x03,
				0xA9),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x88),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0xDF, 0x40),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x87, 0x07, 0x5E),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x03),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x87, 0x07, 0x5E),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x6F, 0x05),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_EVT1, 0x87, 0x07, 0x5E, 0x07, 0x5E, 0x07,
				0x5E, 0x07, 0x5E, 0x07, 0x5E, 0x07, 0x5E, 0x07, 0x5E, 0x07, 0x5E),

	EXYNOS_DSI_CMD_SEQ(0x88, 0x01), /* enable */
	/* circle center: x=720, y=2361 */
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0x88, 0x02, 0xD0, 0x09, 0x39),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x15),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x0A, 0x86),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x17),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x0F, 0xFF),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x19),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x01, 0x4F, 0x06, 0x45, 0x0B, 0x98, 0x01, 0x96, 0x08, 0x19, 0x0A,
					 0xFD, 0x01, 0x55, 0x05, 0x84),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x3D),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x01, 0x4A),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x3F),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x08, 0xBB),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x41),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x08, 0xF4, 0x0C, 0xAB, 0x00, 0xD4, 0x08, 0x80, 0x09, 0x91, 0x0A,
					 0x87, 0x04, 0x1D, 0x0B, 0x9C),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x65),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x07, 0x68),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x67),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x01, 0x1C),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x69),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x0B, 0x3C, 0x0D, 0x16, 0x04, 0x32, 0x07, 0x83, 0x0D, 0x92, 0x0C,
					 0x87, 0x07, 0x4B, 0x07, 0x18),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x29),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x09, 0xBE),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x2B),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x0D, 0x95),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x2D),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x0E, 0x45, 0x07, 0xCE, 0x04, 0x18, 0x03, 0x47, 0x0B, 0x52, 0x00,
					 0x7C, 0x0D, 0x90, 0x0A, 0x8B),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x51),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x02, 0x10),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x53),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x07, 0x9D),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x55),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x01, 0x11, 0x04, 0x28, 0x00, 0xF0, 0x0B, 0x8C, 0x0C, 0xC0, 0x04,
					 0x0F, 0x05, 0x1F, 0x0E, 0x89),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x79),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x07, 0x8C),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x7B),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x0C, 0xE2),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x7D),
	EXYNOS_DSI_CMD_SEQ(0x87, 0x09, 0x08, 0x02, 0xF9, 0x01, 0x08, 0x0D, 0x17, 0x04, 0x6B, 0x00,
					 0xD0, 0x04, 0x77, 0x05, 0x7D),

	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_PROTO1_1, 0x51, 0x02, 0x50),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_PROTO1_1, 0x53, 0x20),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_PROTO1_1, 0xFF, 0xAA, 0x55, 0xA5, 0x84),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_PROTO1_1, 0x6F, 0x7C),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_PROTO1_1, 0xF3, 0x01),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_lhbm_on_setting);

static const struct exynos_dsi_cmd nt37290_dsc_wqhd_cmds[] = {
	/* scaling off (1440x3120) */
	EXYNOS_DSI_CMD_SEQ(0x8F, 0x00),

	/* row address */
	EXYNOS_DSI_CMD_SEQ(0x2B, 0x00, 0x00, 0x0C, 0x2F),
	/* PPS1: slice 30, 2 decoders, 1440x3120 */
	EXYNOS_DSI_CMD_SEQ(0x90, 0x03, 0x03),
	EXYNOS_DSI_CMD_SEQ(0x03, 0x00),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_dsc_wqhd);

static const struct exynos_dsi_cmd nt37290_dsc_fhd_cmds[] = {
	/* scaling up 1.33x (1080x2340) */
	EXYNOS_DSI_CMD_SEQ(0x8F, 0x01),
	EXYNOS_DSI_CMD0(cmd2_page0),
	/* horizontal display resolution selection */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x00, 0x05, 0xA0),
	/* set the display vertical scan line */
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x0C, 0x30),

	/* row address */
	EXYNOS_DSI_CMD_SEQ(0x2B, 0x00, 0x00, 0x09, 0x23),
	/* PPS2: slice 30, 2 decoders, 1080x2340 */
	EXYNOS_DSI_CMD_SEQ(0x90, 0x03, 0x03),
	EXYNOS_DSI_CMD_SEQ(0x03, 0x10),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_dsc_fhd);

static const struct exynos_dsi_cmd nt37290_dsc_init_cmds[] = {
	/* PPS1: slice 30, 2 decoder, scaling off (1440x3120) */
	EXYNOS_DSI_CMD_SEQ(0x91, 0x89, 0x28, 0x00, 0x1E, 0xD2, 0x00, 0x02,
			   0x86, 0x03, 0x28, 0x00, 0x0A, 0x03, 0x97, 0x02,
			   0x8B, 0x10, 0xF0),
	/* PPS2: slice 30, 2 decoder, scaling up 1.33x (1080x2340) */
	EXYNOS_DSI_CMD_SEQ(0x93, 0x89, 0x28, 0x00, 0x1E, 0xD2, 0x00, 0x02,
			   0x25, 0x02, 0xC5, 0x00, 0x07, 0x03, 0x97, 0x03,
			   0x64, 0x10, 0xF0),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_dsc_init);

static const struct exynos_dsi_cmd nt37290_init_cmds[] = {
	/* CMD1 */
	/* set for higher MIPI speed: 1346Mbps */
	EXYNOS_DSI_CMD_SEQ(0x1F, 0xF0),
	/* gamma curve */
	EXYNOS_DSI_CMD_SEQ(0x26, 0x00),
	/* TE output line */
	EXYNOS_DSI_CMD_SEQ(0x35),
	/* select brightness value for proto1.1 and EVT1 (~142nits) */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_EVT1_1),
			       0x51, 0x02, 0x42, 0x02, 0x42, 0x0F, 0xFE),
	/* select brightness value for EVT1.1 and after (~142nits) */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1_1),
			       0x51, 0x08, 0x8D, 0x08, 0x8D, 0x0F, 0xFE),
	/* control brightness */
	EXYNOS_DSI_CMD_SEQ(0x53, 0x20),
	EXYNOS_DSI_CMD_SEQ(0x5A, 0x01),
	/* change refresh frame to 1 after 2Ch command in skip mode */
	EXYNOS_DSI_CMD0(cmd2_page0),
	EXYNOS_DSI_CMD_SEQ(0xBA, 0x00),
	/* dimming config: 32 frame */
	EXYNOS_DSI_CMD_SEQ(0xB2, 0x19),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x05),
	EXYNOS_DSI_CMD_SEQ(0xB2, 0x20),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x06),
	EXYNOS_DSI_CMD_SEQ(0xB2, 0x20),

	/* CMD2 Page 1 */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xC5, 0x00, 0x0B, 0x0B, 0x0B),
	/* CMD2 Page 0: display driving voltage Vkeep 6.35V->5.95V */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_DVT1, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_DVT1, 0x6F, 0x14),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_DVT1, 0xC2, 0x00, 0x50),
	/* CMD2 Page 3: tune internal power on sequence */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x10),
	EXYNOS_DSI_CMD_SEQ(0xB6, 0x1F),
	/* CMD2 Page 5: tune internal power on sequence */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05),
	EXYNOS_DSI_CMD_SEQ(0xB8, 0x04, 0x00, 0x00, 0x02),
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x04, 0x00, 0x00, 0x02),
	EXYNOS_DSI_CMD_SEQ(0xBA, 0x84, 0x00, 0x00, 0x02),
	/* CMD2 Page 3: tune internal power on sequence */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x12),
	EXYNOS_DSI_CMD_SEQ(0xD8, 0x14),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x34),
	EXYNOS_DSI_CMD_SEQ(0xB6, 0x0F, 0x00, 0x3C, 0x00, 0x3C, 0x00, 0x3C, 0x00, 0x3C),
	/* CMD2 Page 8: IRC IP settings */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_DVT1, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_DVT1, 0x6F, 0x10),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_DVT1, 0xB7, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08,
					       0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08,
					       0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_DVT1, 0x6F, 0x28),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_DVT1, 0xB8, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00),
	/* CMD2 Page 5: remove long TE2 pulse */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05),
	EXYNOS_DSI_CMD_SEQ(0xB6, 0x06, 0x03),

	/* CMD3 Page 0 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xAA, 0x55, 0xA5, 0x80),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x1B),
	EXYNOS_DSI_CMD_SEQ(0xF4, 0x55),
	/* CMD3 Page 1 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xAA, 0x55, 0xA5, 0x81),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x12),
	EXYNOS_DSI_CMD_SEQ(0xF5, 0x00),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x09),
	EXYNOS_DSI_CMD_SEQ(0xF9, 0x10),
	/* CMD3 Page 2: MIPI termination resistor 90ohm */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xFF, 0xAA, 0x55, 0xA5, 0x82),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xF2, 0x37),
	/* CMD3 Page 3 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xAA, 0x55, 0xA5, 0x83),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x14),
	EXYNOS_DSI_CMD_SEQ(0xF8, 0x0D),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xF9, 0x06),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xFA, 0x06),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xFB, 0x06),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xFC, 0x06),
	/* CMD3 Page 4 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xAA, 0x55, 0xA5, 0x84),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x1C),
	EXYNOS_DSI_CMD_SEQ(0xF8, 0x3A),

	EXYNOS_DSI_CMD_SEQ_DELAY(120, 0x11),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_init);

static void nt37290_set_local_hbm_mode(struct exynos_panel *ctx, bool local_hbm_en);

static u8 nt37290_get_te2_option(struct exynos_panel *ctx)
{
	struct nt37290_panel *spanel = to_spanel(ctx);

	if (!ctx || !ctx->current_mode)
		return NT37290_TE2_CHANGEABLE;

	/* AOD mode only supports fixed TE2 */
	if (ctx->current_mode->exynos_mode.is_lp_mode ||
	    (spanel->hw_idle_vrefresh > 0 && spanel->hw_idle_vrefresh < NT37290_TE2_MIN_RATE))
		return NT37290_TE2_FIXED;

	return NT37290_TE2_CHANGEABLE;
}

static void nt37290_update_te2(struct exynos_panel *ctx)
{
	struct nt37290_panel *spanel = to_spanel(ctx);
	struct exynos_panel_te2_timing timing;
	/* default timing */
	u8 rising = 0, falling = 0x30;
	u8 option = nt37290_get_te2_option(ctx);
	int ret;

	if (!ctx)
		return;

	if (spanel->rrs_in_progress) {
		dev_dbg(ctx->dev, "%s: RRS in progress, skip\n", __func__);
		return;
	}

	ret = exynos_panel_get_current_mode_te2(ctx, &timing);
	if (!ret) {
		rising = timing.rising_edge & 0xFF;
		falling = timing.falling_edge & 0xFF;
	} else if (ret == -EAGAIN) {
		dev_dbg(ctx->dev, "Panel is not ready, use default timing\n");
	} else {
		dev_warn(ctx->dev, "Failed to get current timing\n");
		return;
	}

	ctx->te2.option = (option == NT37290_TE2_FIXED) ? TE2_OPT_FIXED : TE2_OPT_CHANGEABLE;

	/* option */
	EXYNOS_DCS_BUF_ADD(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03);
	EXYNOS_DCS_BUF_ADD(ctx, 0xC3, option);
	EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x04);
	EXYNOS_DCS_BUF_ADD(ctx, 0xC3, option);
	/* timing */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xC4, 0x00, 0x00, 0x00, 0x00,
				     0x00, rising, 0x10, falling);

	dev_dbg(ctx->dev,
		"TE2 updated: option %s, idle mode %s, rising 0x%x, falling 0x%x\n",
		(option == NT37290_TE2_CHANGEABLE) ? "changeable" : "fixed",
		spanel->hw_idle_vrefresh ? "enabled" : "disabled",
		rising, falling);
}

static inline bool is_auto_mode_allowed(struct exynos_panel *ctx)
{
	/* don't want to enable auto mode/early exit during hbm or dimming on */
	if (IS_HBM_ON(ctx->hbm_mode) || ctx->dimming_on)
		return false;

	return ctx->panel_idle_enabled;
}

static void nt37290_update_min_idle_vrefresh(struct exynos_panel *ctx,
					     const struct exynos_panel_mode *pmode)
{
	struct nt37290_panel *spanel = to_spanel(ctx);
	const int vrefresh = drm_mode_vrefresh(&pmode->mode);
	int idle_vrefresh = ctx->min_vrefresh;

	if (idle_vrefresh < 0 || !is_auto_mode_allowed(ctx) ||
	    pmode->idle_mode == IDLE_MODE_UNSUPPORTED)
		idle_vrefresh = 0;
	else if (idle_vrefresh <= 10)
		idle_vrefresh = 10;
	else if (idle_vrefresh <= 30)
		idle_vrefresh = 30;
	else if (idle_vrefresh <= 60)
		idle_vrefresh = 60;
	else /* 120hz: no idle available */
		idle_vrefresh = 0;

	if (idle_vrefresh >= vrefresh) {
		dev_dbg(ctx->dev, "idle vrefresh (%d) higher than target (%d)\n",
			idle_vrefresh, vrefresh);
		idle_vrefresh = 0;
	}

	if (idle_vrefresh && ctx->idle_delay_ms &&
	    (panel_get_idle_time_delta(ctx) < ctx->idle_delay_ms)) {
		spanel->delayed_idle = true;
		idle_vrefresh = 0;
	} else {
		spanel->delayed_idle = false;
	}

	spanel->auto_mode_vrefresh = idle_vrefresh;
}

static const struct nt37290_osc2_clk_data osc2_clk_data[] = {
	{ .clk_khz = 170500, .fcon_val = 0 },
	{ .clk_khz = 165400, .fcon_val = 1 },
};

static inline u8 nt37290_get_frame_rate_ctrl(struct exynos_panel *ctx,
					     enum nt37290_dfc_mode mode)
{
	struct nt37290_panel *spanel = to_spanel(ctx);
	u8 fcon = osc2_clk_data[spanel->hw_osc2_clk_idx].fcon_val;
	u8 val = ((mode & 0x7) << 4) | (fcon & 0x7);

	return val;
}

static bool nt37290_update_panel_feat(struct exynos_panel *ctx,
				      const struct exynos_panel_mode *pmode, bool enforce)
{
	struct nt37290_panel *spanel = to_spanel(ctx);
	int vrefresh;
	int idle_vrefresh = spanel->auto_mode_vrefresh;
	DECLARE_BITMAP(changed_feat, G10_FEAT_MAX);
	bool ee, fi;

	if (pmode)
		vrefresh = drm_mode_vrefresh(&pmode->mode);
	else
		vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);

	/* when panel feat func is called, idle effect should be disabled */
	ctx->panel_idle_vrefresh = 0;

	if (enforce) {
		bitmap_fill(changed_feat, G10_FEAT_MAX);
	} else {
		bitmap_xor(changed_feat, spanel->feat, spanel->hw_feat, G10_FEAT_MAX);
		if (bitmap_empty(changed_feat, G10_FEAT_MAX) &&
		    vrefresh == spanel->hw_vrefresh &&
		    idle_vrefresh == spanel->hw_idle_vrefresh)
			return false;
	}

	spanel->hw_vrefresh = vrefresh;
	spanel->hw_idle_vrefresh = idle_vrefresh;
	bitmap_copy(spanel->hw_feat, spanel->feat, G10_FEAT_MAX);
	ee = test_bit(G10_FEAT_EARLY_EXIT, spanel->feat);
	fi = test_bit(G10_FEAT_FRAME_AUTO, spanel->feat);

	dev_dbg(ctx->dev, "ee=%s fi=%s vrefresh=%d idle_vrefresh=%d osc2=%u\n",
		ee ? "on" : "off", fi ? "auto" : "manual",
		vrefresh, idle_vrefresh, ctx->osc2_clk_khz);

	DPU_ATRACE_BEGIN(__func__);

	if (vrefresh == 120 && !fi) {
		/* DFC mode manual */
		EXYNOS_DCS_BUF_ADD(ctx, 0x2F,
				   nt37290_get_frame_rate_ctrl(ctx, DFC_MODE_MANUAL));
		/* 120Hz gamma band */
		if (ctx->panel_rev == PANEL_REV_EVT1_1)
			EXYNOS_DCS_BUF_ADD(ctx, 0x26, 0x00);
		/* restore TE timing (no shift) */
		EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x44, 0x00, 0x00);
	} else {
		/* DFC mode panel low power */
		EXYNOS_DCS_BUF_ADD(ctx, 0x2F,
				   nt37290_get_frame_rate_ctrl(ctx, DFC_MODE_PANEL_LP));
		/* early exit */
		EXYNOS_DCS_BUF_ADD(ctx, 0x5A, !ee);

		/* set auto frame insertion */
		EXYNOS_DCS_BUF_ADD_SET(ctx, cmd2_page0);
		EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x1C);
		/* auto frame insertion off (manual) */
		if (!fi) {
			if (vrefresh == 60)
				EXYNOS_DCS_BUF_ADD(ctx,
					0xBA, 0x91, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00);
			else
				dev_warn(ctx->dev,
					 "Unsupported vrefresh %dHz for manual mode\n", vrefresh);
		/* auto frame insertion on */
		} else {
			if (vrefresh == 60) {
				if (idle_vrefresh == 10)
					EXYNOS_DCS_BUF_ADD(ctx, 0xBA, 0x91, 0x09, 0x03, 0x00, 0x11,
							   0x0B, 0x0B, 0x00, 0x06);
				else if (idle_vrefresh == 30)
					EXYNOS_DCS_BUF_ADD(ctx, 0xBA, 0x91, 0x03, 0x02, 0x00, 0x11,
							   0x03, 0x03, 0x00, 0x04);
				else
					dev_warn(ctx->dev,
						 "Unsupported idle_vrefresh %dHz for auto mode\n",
						 idle_vrefresh);
			} else if (vrefresh == 120) {
				/* two 120Hz frames are removed on EVT1.1 */
				u8 val = (ctx->panel_rev == PANEL_REV_EVT1_1) ? 0x91 : 0x93;

				if (idle_vrefresh == 10)
					EXYNOS_DCS_BUF_ADD(ctx, 0xBA, val, 0x09, 0x03, 0x00, 0x31,
							   0x0B, 0x0B, 0x00, 0x06);
				else if (idle_vrefresh == 30)
					EXYNOS_DCS_BUF_ADD(ctx, 0xBA, val, 0x03, 0x02, 0x00, 0x11,
							   0x03, 0x03, 0x00, 0x04);
				else if (idle_vrefresh == 60)
					EXYNOS_DCS_BUF_ADD(ctx, 0xBA, 0x93, 0x01, 0x01, 0x00, 0x01,
							   0x01, 0x01, 0x00, 0x00);
				else
					dev_warn(ctx->dev,
						 "Unsupported idle_vrefresh %dHz for auto mode\n",
						 idle_vrefresh);
			} else {
				dev_warn(ctx->dev, "Unsupported vrefresh %dHz for auto mode\n",
					 vrefresh);
			}
		}

		EXYNOS_DCS_BUF_ADD(ctx, 0x2C);

		if (ctx->panel_rev >= PANEL_REV_EVT1_1) {
			EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x03);
			EXYNOS_DCS_BUF_ADD(ctx, 0xC0,
					   (spanel->hw_osc2_clk_idx == 1) ? 0x21 : 0x20);
		}

		/* RR gamma band (60~10Hz) */
		if (ctx->panel_rev == PANEL_REV_EVT1_1)
			EXYNOS_DCS_BUF_ADD(ctx, 0x26, 0x01);

		if (vrefresh == 120)
			/* restore TE timing (no shift) */
			EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x44, 0x00, 0x00);
		else
			/* TE shift 8.2ms */
			EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x44, 0x00, 0x01);
	}

	DPU_ATRACE_END(__func__);

	return true;
}

static bool nt37290_change_frequency(struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	struct nt37290_panel *spanel = to_spanel(ctx);
	int vrefresh = drm_mode_vrefresh(&pmode->mode);
	bool idle_active = false;
	bool was_lp_mode = ctx->current_mode->exynos_mode.is_lp_mode;
	bool updated;

	nt37290_update_min_idle_vrefresh(ctx, pmode);

	if (spanel->auto_mode_vrefresh &&
	    (pmode->idle_mode == IDLE_MODE_ON_INACTIVITY ||
	    (pmode->idle_mode == IDLE_MODE_ON_SELF_REFRESH && ctx->self_refresh_active)))
		idle_active = true;

	if (idle_active) {
		set_bit(G10_FEAT_EARLY_EXIT, spanel->feat);
		set_bit(G10_FEAT_FRAME_AUTO, spanel->feat);
	} else {
		clear_bit(G10_FEAT_EARLY_EXIT, spanel->feat);
		clear_bit(G10_FEAT_FRAME_AUTO, spanel->feat);
	}

	/* need to update 2Fh command while exiting AOD */
	updated = nt37290_update_panel_feat(ctx, pmode, was_lp_mode);

	ctx->panel_idle_vrefresh = ctx->self_refresh_active ? spanel->hw_idle_vrefresh : 0;

	if (updated) {
		notify_panel_mode_changed(ctx, false);
		dev_dbg(ctx->dev, "change to %dHz, idle %s, was_lp_mode %d\n",
			vrefresh, idle_active ? "active" : "deactive", was_lp_mode);
	}

	return updated;
}


static void nt37290_panel_idle_notification(struct exynos_panel *ctx,
				u32 display_id, u32 vrefresh, u32 idle_te_vrefresh)
{
	char event_string[64];
	char *envp[] = { event_string, NULL };
	struct drm_device *dev = ctx->bridge.dev;

	if (vrefresh == idle_te_vrefresh)
		return;

	if (!dev) {
		dev_warn(ctx->dev, "%s: drm_device is null\n", __func__);
	} else {
		snprintf(event_string, sizeof(event_string),
			"PANEL_IDLE_ENTER=%u,%u,%u", display_id, vrefresh, idle_te_vrefresh);
		kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, envp);
	}
}

static bool nt37290_set_self_refresh(struct exynos_panel *ctx, bool enable)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	bool updated;

	if (unlikely(!pmode))
		return false;

	/* self refresh is not supported in lp mode since that always makes use of early exit */
	if (pmode->exynos_mode.is_lp_mode)
		return false;

	/* do not change frequency when LHBM is enabled */
	if (ctx->hbm.local_hbm.enabled) {
		dev_warn(ctx->dev, "%s: frequency change is not allowed while LHBM is enabled\n",
			__func__);
		return false;
	}

	DPU_ATRACE_BEGIN(__func__);

	updated = nt37290_change_frequency(ctx, pmode);

	if (pmode->idle_mode == IDLE_MODE_ON_SELF_REFRESH) {
		const int vrefresh = drm_mode_vrefresh(&pmode->mode);

		if (enable && ctx->panel_idle_vrefresh)
			nt37290_panel_idle_notification(ctx, 0, vrefresh, 120);

		dev_dbg(ctx->dev, "%s: %s idle (%dHz) for mode %s\n",
			__func__, enable ? "enter" : "exit",
			ctx->panel_idle_vrefresh ? : vrefresh,
			pmode->mode.name);
	}

	DPU_ATRACE_END(__func__);

	return updated;
}

/**
 * EARLY_EXIT_THRESHOLD_US
 *
 * 120hz auto mode takes at least 2 frames to start lowering refresh rate in addition to
 * time to next vblank. Use just over 2 frames time to consider worst case scenario
 */
#define EARLY_EXIT_THRESHOLD_US 17000

/**
 * IDLE_DELAY_THRESHOLD_US
 *
 * Use a threshold to avoid disabling idle auto mode too frequently while continuously
 * updating frames. Considering the hibernation time for this scenario.
 */
#define IDLE_DELAY_THRESHOLD_US 50000

/**
 * nt37290_trigger_early_exit - trigger early exit command to panel
 * @ctx: panel struct
 *
 * Sends a command to panel to indicate a frame is about to come in case its been a while since
 * the last frame update and auto mode may have started to take effect and lowering refresh rate
 */
static bool nt37290_trigger_early_exit(struct exynos_panel *ctx)
{
	const ktime_t delta = ktime_sub(ktime_get(), ctx->last_commit_ts);
	const s64 delta_us = ktime_to_us(delta);
	bool updated = false;

	if (delta_us < EARLY_EXIT_THRESHOLD_US) {
		dev_dbg(ctx->dev, "skip early exit. %lldus since last commit\n",
			delta_us);
		return false;
	}

	/* triggering early exit causes a switch to 120hz */
	ctx->last_mode_set_ts = ktime_get();

	DPU_ATRACE_BEGIN(__func__);

	if (ctx->idle_delay_ms && delta_us > IDLE_DELAY_THRESHOLD_US &&
	    !ctx->hbm.local_hbm.enabled) {
		const struct exynos_panel_mode *pmode = ctx->current_mode;

		dev_dbg(ctx->dev, "%s: disable auto idle mode for %s\n",
			 __func__, pmode->mode.name);
		updated = nt37290_change_frequency(ctx, pmode);
	} else {
		EXYNOS_DCS_WRITE_TABLE(ctx, stream_2c);
	}

	DPU_ATRACE_END(__func__);

	return updated;
}

/* TODO: move update te2 to common display driver for other panel drivers */
static void nt37290_commit_done(struct exynos_panel *ctx)
{
	struct nt37290_panel *spanel = to_spanel(ctx);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	bool updated = false;

	if (!is_panel_active(ctx) || pmode->exynos_mode.is_lp_mode)
		return;

	if (spanel->rrs_in_progress) {
		/* we should finish RRS in this commit */
		spanel->rrs_in_progress = false;
		return;
	}

	if (test_bit(G10_FEAT_EARLY_EXIT, spanel->feat))
		updated = nt37290_trigger_early_exit(ctx);
	/**
	 * For IDLE_MODE_ON_INACTIVITY, we should go back to auto mode again
	 * after the delay time has elapsed.
	 */
	else if (pmode->idle_mode == IDLE_MODE_ON_INACTIVITY &&
		 spanel->delayed_idle && !ctx->hbm.local_hbm.enabled)
		updated = nt37290_change_frequency(ctx, pmode);

	if (updated)
		nt37290_update_te2(ctx);
}

static void nt37290_set_lp_mode(struct exynos_panel *ctx,
				const struct exynos_panel_mode *pmode)
{
	exynos_panel_set_lp_mode(ctx, pmode);

	/* enable early exit and auto frame insertion (10Hz) */
	EXYNOS_DCS_BUF_ADD(ctx, 0x2F,
			   nt37290_get_frame_rate_ctrl(ctx, DFC_MODE_PANEL_LP));
	EXYNOS_DCS_BUF_ADD(ctx, 0x5A, 0x00);
	EXYNOS_DCS_BUF_ADD_SET(ctx, cmd2_page0);
	EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x1C);
	EXYNOS_DCS_BUF_ADD(ctx, 0xBA, 0x95, 0x02, 0x02, 0x00, 0x11, 0x02, 0x02, 0x00);
	/* make sure TE timing is no shift in AOD */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x44, 0x00, 0x00);

	dev_dbg(ctx->dev, "%s: done\n", __func__);
}

#define TE_WIDTH_USEC 162
static void nt37290_wait_for_vsync_done(struct exynos_panel *ctx, u32 vrefresh)
{
	if (vrefresh != 60 && vrefresh != 120) {
		dev_err(ctx->dev, "%s: unsupported refresh rate (%d)\n",
			__func__, vrefresh);
		return;
	}

	exynos_panel_wait_for_vsync_done(ctx, TE_WIDTH_USEC,
			EXYNOS_VREFRESH_TO_PERIOD_USEC(vrefresh));
}

static void nt37290_wait_one_vblank(struct exynos_panel *ctx,
				    const struct exynos_panel_mode *pmode)
{
	struct drm_crtc *crtc = NULL;

	if (ctx->exynos_connector.base.state)
		crtc = ctx->exynos_connector.base.state->crtc;

	DPU_ATRACE_BEGIN(__func__);

	if (crtc && !drm_crtc_vblank_get(crtc)) {
		drm_crtc_wait_one_vblank(crtc);
		drm_crtc_vblank_put(crtc);
	} else {
		int vrefresh = drm_mode_vrefresh(&pmode->mode);
		u32 delay_us = mult_frac(1000, 1020, vrefresh);

		dev_warn(ctx->dev, "%s: failed to get vblank for %dhz\n", __func__, vrefresh);
		usleep_range(delay_us, delay_us + 10);
	}

	DPU_ATRACE_END(__func__);
}

static void nt37290_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	if (!is_panel_active(ctx))
		return;

	DPU_ATRACE_BEGIN(__func__);

	/* exit AOD */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x38);
	nt37290_wait_one_vblank(ctx, ctx->current_mode);

	nt37290_change_frequency(ctx, pmode);

	/* 2Ch needs to be sent twice in next 2 vsync */
	EXYNOS_DCS_WRITE_TABLE(ctx, stream_2c);
	nt37290_wait_one_vblank(ctx, pmode);
	EXYNOS_DCS_WRITE_TABLE(ctx, stream_2c);

	EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	DPU_ATRACE_END(__func__);

	dev_info(ctx->dev, "exit LP mode\n");
}

static int nt37290_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	const struct drm_display_mode *mode;
	const bool needs_reset = !is_panel_enabled(ctx);
	bool is_fhd;
	u32 vrefresh;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}

	mode = &pmode->mode;
	is_fhd = mode->hdisplay == 1080;
	vrefresh = needs_reset ? 60 : drm_mode_vrefresh(mode);

	dev_dbg(ctx->dev, "%s (%s)\n", __func__, is_fhd ? "fhd" : "wqhd");

	DPU_ATRACE_BEGIN(__func__);

	if (needs_reset) {
		exynos_panel_reset(ctx);
		exynos_panel_send_cmd_set(ctx, &nt37290_init_cmd_set);
		exynos_panel_send_cmd_set(ctx, &nt37290_dsc_init_cmd_set);
		exynos_panel_send_cmd_set(ctx, &nt37290_lhbm_on_setting_cmd_set);
		nt37290_update_panel_feat(ctx, pmode, true);
	}

	/* make sure both DPU and panel PPS are set in the same VSYNC */
	if (ctx->mode_in_progress == MODE_RES_IN_PROGRESS)
		nt37290_wait_for_vsync_done(ctx, vrefresh);
	else if (ctx->mode_in_progress == MODE_RES_AND_RR_IN_PROGRESS)
		nt37290_wait_for_vsync_done(ctx, ctx->last_rr);

	exynos_panel_send_cmd_set(ctx,
				  is_fhd ? &nt37290_dsc_fhd_cmd_set : &nt37290_dsc_wqhd_cmd_set);

	if (pmode->exynos_mode.is_lp_mode)
		nt37290_set_lp_mode(ctx, pmode);
	else {
		if (!needs_reset)
			nt37290_change_frequency(ctx, pmode);

		if (needs_reset || ctx->panel_state == PANEL_STATE_BLANK)
			EXYNOS_DCS_WRITE_TABLE(ctx, display_on);
	}

	DPU_ATRACE_END(__func__);

	return 0;
}

static int nt37290_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	struct nt37290_panel *spanel = to_spanel(ctx);

	/* skip disable sequence if going through modeset */
	if (ctx->panel_state == PANEL_STATE_MODESET) {
		spanel->rrs_in_progress = true;
		return 0;
	}

	/* panel register state gets reset after disabling hardware */
	bitmap_clear(spanel->hw_feat, 0, G10_FEAT_MAX);
	spanel->hw_vrefresh = 60;
	spanel->hw_idle_vrefresh = 0;

	if (ctx->hbm.local_hbm.enabled) {
		dev_warn(ctx->dev, "%s: lhbm is still enabled while disabling panel\n",
			 __func__);
		mutex_lock(&ctx->mode_lock);
		/* disable lhbm immediately */
		nt37290_set_local_hbm_mode(ctx, false);
		mutex_unlock(&ctx->mode_lock);
	}

	return exynos_panel_disable(panel);
}

/**
 * struct brightness_data - nits, DBV, and coefficients in each brightness band
 *
 * @nits: brightness in nits.
 * @level: brightness in DBV.
 *
 * @coef_a: coefficient a in the equation of a nonlinear curve
 * @coef_b: coefficient b in the equation of a nonlinear curve
 * @coef_c: coefficient c in the equation of a nonlinear curve
 */
struct brightness_data {
	u16 nits;
	u16 level;

	s64 coef_a;
	s64 coef_b;
	s64 coef_c;
};

/**
 * struct brightness_settings - brightness data of all bands in hbm and normal mode.
 *
 * @hbm_band_data: pointer to brightness data in hbm bands.
 * @num_of_hbm_bands: number of brightness bands in hbm.
 * @normal_band_data: pointer to brightness data in normal bands.
 * @num_of_normal_bands: number of brightness bands in normal mode.
 *
 * @nonlinear_range_data: pointer to nonlinear data in all ranges. If hbm_band_data
 *                        and normal_band_data are not defined, this should be specified.
 * @num_of_nonlinear_ranges: number of nonlinear brightness ranges.
 */
struct brightness_settings {
	const struct brightness_data * const hbm_band_data;
	const unsigned int num_of_hbm_bands;
	const struct brightness_data * const normal_band_data;
	const unsigned int num_of_normal_bands;

	const struct brightness_data * const nonlinear_range_data;
	const unsigned int num_of_nonlinear_ranges;
};

static const struct brightness_data evt1_1_hbm_band_data[] = {
	{ .nits = 1000, .level = 4094 }, /* band 0 */
	{ .nits = 600, .level = 3585 },  /* band 1 */
};

static const struct brightness_data evt1_1_normal_band_data[] = {
	{ .nits = 600, .level = 3584 }, /* band 0 */
	{ .nits = 300, .level = 3072 }, /* band 1 */
	{ .nits = 200, .level = 2560 }, /* band 2 */
	{ .nits = 120, .level = 2048 }, /* band 3 */
	{ .nits = 80, .level = 1536 },  /* band 4 */
	{ .nits = 50, .level = 1024 },  /* band 5 */
	{ .nits = 25, .level = 512 },   /* band 6 */
	{ .nits = 5, .level = 256 },    /* band 7 */
	{ .nits = 2, .level = 3 },      /* band 8 */
};

/*
 * struct brightness_data dvt1_range_data
 *
 * All coefficients are multiplied by 10^6 to have sufficient accuracy.
 * For example, the original equation of range 0 is:
 *
 * y = 0.0001123*x^2 - 0.06496*x - 611.79
 */
static const struct brightness_data dvt1_range_data[] = {
	/* nits, level, coef_a, coef_b, coef_c*/
	{ 1000, 4094, 112, -64960, -611790000 }, /* range 0: HBM ~ band 0 */
	{ 600, 3584, 271, -1227200, 1520800000 }, /* range 1: band 0 ~ band 1 */
	{ 300, 3072, 72, -213700, 276570000 }, /* range 2: band 1 ~ band 2 */
	{ 200, 2560, 53, -87800, 77395000 }, /* range 3: band 2 ~ band 3 */
	{ 120, 2048, 26, -17300, 45440000 }, /* range 4: band 3 ~ band 4 */
	{ 80, 1536, 28, -12700, 34007000 }, /* range 5: band 4 ~ band 5 */
	{ 50, 1024, 20, 16300, 11355000 }, /* range 6: band 5 ~ band 6 */
	{ 25, 512, 186, -68900, 10625000 }, /* range 7: band 6 ~ band 7 */
	{ 5, 256, 17, 7293, 1989200 }, /* range 8: band 7 ~ band 8 */
	{ 2, 3 }, /* range 9: band 8 */
};

static const struct brightness_settings evt1_1_br_settings = {
	.hbm_band_data = evt1_1_hbm_band_data,
	.num_of_hbm_bands = ARRAY_SIZE(evt1_1_hbm_band_data),
	.normal_band_data = evt1_1_normal_band_data,
	.num_of_normal_bands = ARRAY_SIZE(evt1_1_normal_band_data),
};

static const struct brightness_data evt1_hbm_band_data[] = {
	{ .nits = 1000, .level = 4094 }, /* band 0 */
	{ .nits = 500, .level = 2048 },  /* band 1 */
};

static const struct brightness_data evt1_normal_band_data[] = {
	{ .nits = 500, .level = 2047 }, /* band 0 */
	/* skip band 1 to 7 due to linear brightness in evt1 normal mode */
	{ .nits = 2, .level = 3 },      /* band 8 */
};

static const struct brightness_settings evt1_br_settings = {
	.hbm_band_data = evt1_hbm_band_data,
	.num_of_hbm_bands = ARRAY_SIZE(evt1_hbm_band_data),
	.normal_band_data = evt1_normal_band_data,
	.num_of_normal_bands = ARRAY_SIZE(evt1_normal_band_data),
};

static const struct brightness_settings dvt1_br_settings = {
	.nonlinear_range_data = dvt1_range_data,
	.num_of_nonlinear_ranges = ARRAY_SIZE(dvt1_range_data),
};

/**
 * linear_interpolation - linear interpolation for given x
 * @x1: x coordinate
 * @x2: x coordinate, greater than x1
 * @y1: y coordinate
 * @y2: y coordinate, greater than y1
 * @x: x coordinate, in the interval (x1, x2)
 *
 * Given x, do linear interpolation according to x1, x2, y1, and y2.
 * Return a new value after calculation. Return negative if the inputs are invalid.
 */
static inline u64 linear_interpolation(u64 x1, u64 x2, u64 y1, u64 y2, u64 x)
{
	if (x == x1)
		return y1;
	else if (x == x2)
		return y2;
	else
		return (y1 + DIV_ROUND_CLOSEST((x - x1) * (y2 - y1), x2 - x1));
}

/**
 * nonlinear_calculation - nonlinear calculation for given y
 * @a: coefficient a in the equation
 * @b: coefficient b in the equation
 * @c: coefficient c in the equation
 * @y: y coordinate
 * @x: pointer of x coordinate
 *
 * Given y, do the calculation for equation y = a*x^2 + b*x + c for a nonlinear curve.
 * The x can be obtained by int_sqrt(((y - c) / a) + (b / a / 2)^2) - b / a / 2
 * Only positive square root is considered for x. Return negative while invalid.
 */
static inline int nonlinear_calculation(s64 a, s64 b, s64 c, s64 y, s64 *x)
{
	s64 p, q, r;

	if (!a)
		return -EINVAL;

	p = DIV_ROUND_CLOSEST(y - c, a);
	q = DIV_ROUND_CLOSEST(b * b, a * a * 4);
	r = DIV_ROUND_CLOSEST(b, a * 2);

	if ((p + q) < 0)
		return -EINVAL;

	*x = int_sqrt(p + q) - r;

	return 0;
}

static u16 nt37290_convert_to_evt1_1_nonlinear_br(struct exynos_panel *ctx, u16 br)
{
	u16 i = 0, nits = 0, level = 0, band = 0;
	u16 num_band = evt1_1_br_settings.num_of_normal_bands;

	nits = linear_interpolation(evt1_1_br_settings.normal_band_data[num_band - 1].level,
					 evt1_1_br_settings.normal_band_data[0].level,
					 evt1_1_br_settings.normal_band_data[num_band - 1].nits,
					 evt1_1_br_settings.normal_band_data[0].nits,
					 br);
	for (i = 1; i < num_band; i++) {
		if (nits >= evt1_1_br_settings.normal_band_data[i].nits) {
			band = i;
			break;
		}
	}
	level = linear_interpolation(evt1_1_br_settings.normal_band_data[band].nits,
				     evt1_1_br_settings.normal_band_data[band - 1].nits,
				     evt1_1_br_settings.normal_band_data[band].level,
				     evt1_1_br_settings.normal_band_data[band - 1].level,
				     nits);

	dev_dbg(ctx->dev, "%s: nits %u, band %u, level %u->%u\n",
		__func__, nits, band, br, level);

	return level;
}

static u16 nt37290_convert_to_evt1_br(struct exynos_panel *ctx, u16 br)
{
	u16 x1, x2, y1, y2, num_band, level;

	if (br <= evt1_1_br_settings.normal_band_data[0].level) {
		num_band = evt1_1_br_settings.num_of_normal_bands;
		x1 = evt1_1_br_settings.normal_band_data[num_band - 1].level;
		x2 = evt1_1_br_settings.normal_band_data[0].level;
		num_band = evt1_br_settings.num_of_normal_bands;
		y1 = evt1_br_settings.normal_band_data[num_band - 1].level;
		y2 = evt1_br_settings.normal_band_data[0].level;
	} else {
		num_band = evt1_1_br_settings.num_of_hbm_bands;
		x1 = evt1_1_br_settings.hbm_band_data[num_band - 1].level;
		x2 = evt1_1_br_settings.hbm_band_data[0].level;
		num_band = evt1_br_settings.num_of_hbm_bands;
		y1 = evt1_br_settings.hbm_band_data[num_band - 1].level;
		y2 = evt1_br_settings.hbm_band_data[0].level;
	}

	level = linear_interpolation(x1, x2, y1, y2, br);
	dev_dbg(ctx->dev, "%s: level %u->%u\n",	__func__, br, level);
	return level;
}

static u16 nt37290_convert_to_dvt1_nonlinear_br(struct exynos_panel *ctx, u16 br)
{
	u16 i, range = 0;
	u16 num_range = dvt1_br_settings.num_of_nonlinear_ranges;
	u16 min_br_nbm = dvt1_br_settings.nonlinear_range_data[num_range - 1].level;
	u16 max_br_nbm = dvt1_br_settings.nonlinear_range_data[1].level;
	u16 min_br_hbm = max_br_nbm + 1;
	u16 max_br_hbm = dvt1_br_settings.nonlinear_range_data[0].level;
	u64 x1, x2, y1, y2, x;
	s64 y, a, b, c, level = br;
	int ret;

	if (br <= min_br_nbm || br == max_br_nbm || br >= max_br_hbm) {
		dev_dbg(ctx->dev, "%s: level %u\n", __func__, br);
		return br;
	}

	/**
	 * x is level (DBV), y is brightness (nits). Multiplied by 10^5 to
	 * have sufficient accuracy without overflow.
	 */
	if (br <= max_br_nbm ) {
		x1 = min_br_nbm * 100000;
		x2 = max_br_nbm * 100000;
		y1 = (dvt1_br_settings.nonlinear_range_data[num_range - 1].nits) * 100000;
		y2 = (dvt1_br_settings.nonlinear_range_data[1].nits) * 100000;
	} else {
		x1 = min_br_hbm * 100000;
		x2 = max_br_hbm * 100000;
		y1 = (dvt1_br_settings.nonlinear_range_data[1].nits) * 100000;
		y2 = (dvt1_br_settings.nonlinear_range_data[0].nits) * 100000;
	}
	x = br * 100000;
	y = linear_interpolation(x1, x2, y1, y2, x);
	/**
	 * Multiplied by 10 to have the same accuracy as nonlinear
	 * coefficients without overflow for data type long. */
	y *= 10;

	/* find the range for the calculated nits (y) */
	for (i = 1; i < num_range; i++) {
		u64 nits = dvt1_br_settings.nonlinear_range_data[i].nits * 1000000;

		if (y > nits) {
			range = i - 1;
			break;
		}
	}

	/**
	 * level is DBV, y is brightness (nits). The coefficients are varied
	 * according to different ranges.
	 */
	a = dvt1_br_settings.nonlinear_range_data[range].coef_a;
	b = dvt1_br_settings.nonlinear_range_data[range].coef_b;
	c = dvt1_br_settings.nonlinear_range_data[range].coef_c;
	ret = nonlinear_calculation(a, b, c, y, &level);
	if (ret || level < min_br_nbm || level > max_br_hbm) {
		dev_warn(ctx->dev, "%s: failed to convert for brightness %u, ret %d\n",
			 __func__, br, ret);
		if (!ret)
			br = (level < min_br_nbm) ? min_br_nbm : max_br_hbm;
		return br;
	}

	dev_dbg(ctx->dev, "%s: nits %lld, range %u, level %u->%lld\n",
		__func__, DIV_ROUND_CLOSEST(y, 1000000), range, br, level);

	return level;
}

static int nt37290_set_brightness(struct exynos_panel *ctx, u16 br)
{
	u16 brightness;
	struct nt37290_panel *spanel = to_spanel(ctx);
	const bool lp_mode = ctx->current_mode->exynos_mode.is_lp_mode;

	spanel->hw_dbv = br;

	if (!lp_mode && spanel->hw_dbv == 0) {
		// turn off panel and set brightness directly.
		return exynos_dcs_set_brightness(ctx, 0);
	}

	if (spanel->hw_dbv) {
		if (ctx->panel_rev >= PANEL_REV_DVT1) {
			spanel->hw_dbv = nt37290_convert_to_dvt1_nonlinear_br(ctx, br);
		} else if (ctx->panel_rev == PANEL_REV_EVT1_1) {
			if (br <= evt1_1_br_settings.normal_band_data[0].level)
				spanel->hw_dbv =
					nt37290_convert_to_evt1_1_nonlinear_br(ctx, br);
		} else {
			spanel->hw_dbv = nt37290_convert_to_evt1_br(ctx, br);
		}
	}

	if (lp_mode) {
		const struct exynos_panel_funcs *funcs;

		funcs = ctx->desc->exynos_panel_func;
		if (funcs && funcs->set_binned_lp)
			funcs->set_binned_lp(ctx, spanel->hw_dbv);
		return 0;
	}

	if (ctx->panel_rev >= PANEL_REV_EVT1 && ctx->hbm.local_hbm.enabled) {
		u16 level = spanel->hw_dbv * 4;
		u8 val1 = level >> 8;
		u8 val2 = level & 0xff;

		/* LHBM DBV value write */
		EXYNOS_DCS_BUF_ADD_SET(ctx, cmd2_page0);
		EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x4C);
		EXYNOS_DCS_BUF_ADD(ctx, 0xDF, val1, val2, val1, val2, val1, val2);
	}

	brightness = (spanel->hw_dbv & 0xff) << 8 | spanel->hw_dbv >> 8;

	return exynos_dcs_set_brightness(ctx, brightness);
}

static void nt37290_set_hbm_mode(struct exynos_panel *ctx,
				 enum exynos_hbm_mode mode)
{
	if (ctx->hbm_mode == mode)
		return;

	if (IS_HBM_ON(ctx->hbm_mode) != IS_HBM_ON(mode)) {
		EXYNOS_DCS_BUF_ADD_SET(ctx, cmd2_page0);
		EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x11);
		EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xB2, mode ? 0x00 : 0x01);
	}

	if (IS_HBM_ON_IRC_OFF(ctx->hbm_mode) != IS_HBM_ON_IRC_OFF(mode)) {
		if (ctx->panel_rev >= PANEL_REV_DVT1) {
			EXYNOS_DCS_BUF_ADD(ctx, 0x5F, IS_HBM_ON_IRC_OFF(mode) ? 0x01 : 0x00);
			EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x26,
				 IS_HBM_ON_IRC_OFF(mode) ? 0x03 : 0x00);
		} else {
			EXYNOS_DCS_BUF_ADD(ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x84);
			EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x02);
			EXYNOS_DCS_BUF_ADD(ctx, 0xF5, 0x01);
			EXYNOS_DCS_BUF_ADD(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08);
			EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xB9,
				 IS_HBM_ON_IRC_OFF(mode) ? 0x00 : 0x01);
		}
	}

	ctx->hbm_mode = mode;
	dev_info(ctx->dev, "hbm_on=%d hbm_ircoff=%d\n", IS_HBM_ON(ctx->hbm_mode),
		 IS_HBM_ON_IRC_OFF(ctx->hbm_mode));
}

static void nt37290_set_local_hbm_mode(struct exynos_panel *ctx,
				       bool local_hbm_en)
{
	if (local_hbm_en) {
		if (ctx->panel_rev >= PANEL_REV_EVT1) {
			struct nt37290_panel *spanel = to_spanel(ctx);
			u16 level = spanel->hw_dbv * 4;
			u8 val1 = level >> 8;
			u8 val2 = level & 0xff;

			/* LHBM DBV value write */
			EXYNOS_DCS_BUF_ADD_SET(ctx, cmd2_page0);
			EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x4C);
			EXYNOS_DCS_BUF_ADD(ctx, 0xDF, val1, val2, val1, val2, val1, val2);
			/* FPS gamma timing */
			EXYNOS_DCS_BUF_ADD(ctx, 0x2F, 0x02);
			/* Enter FPS mode */
			EXYNOS_DCS_BUF_ADD(ctx, 0x87, 0x01);
		} else {
			EXYNOS_DCS_BUF_ADD(ctx, 0x87, 0x21);
		}
		/* LHBM on */
		EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x85);
	} else {
		/* LHBM off */
		EXYNOS_DCS_BUF_ADD(ctx, 0x86);
		if (ctx->panel_rev >= PANEL_REV_EVT1) {
			/* Exit FPS mode */
			EXYNOS_DCS_BUF_ADD(ctx, 0x87, 0x00);
			/* normal gamma timing */
			EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x2F, 0x00);
		} else {
			EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x87, 0x20);
		}
	}
}

static void nt37290_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{
	nt37290_change_frequency(ctx, pmode);
}

static bool nt37290_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	const struct drm_display_mode *c = &ctx->current_mode->mode;
	const struct drm_display_mode *n = &pmode->mode;

	/* seamless mode set can happen if active region resolution is same */
	return (c->vdisplay == n->vdisplay) && (c->hdisplay == n->hdisplay) &&
	       (c->flags == n->flags);
}

static void nt37290_get_panel_rev(struct exynos_panel *ctx, u32 id)
{
	/* extract command 0xDB */
	u8 build_code = (id & 0xFF00) >> 8;
	u8 main = (build_code & 0xE0) >> 3;
	u8 sub = 0;

	if ((build_code & 0x03) == 0 && (build_code & 0x0C) != 0)
		/* bit[3:2] */
		sub = (build_code & 0x0C) >> 2;
	else if ((build_code & 0x03) != 0 && (build_code & 0x0C) == 0)
		/* bit[1:0] */
		sub = build_code & 0x03;

	exynos_panel_get_panel_rev(ctx, main | sub);
}

static void nt37290_set_osc2_clk_khz(struct exynos_panel *ctx, unsigned int clk_khz)
{
	struct nt37290_panel *spanel = to_spanel(ctx);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	int num_clk = ARRAY_SIZE(osc2_clk_data);
	int idx = 0;
	int i;

	/* only EVT1.1 and later versions are allowed to change OSC2 clock */
	if (ctx->panel_rev < PANEL_REV_EVT1_1)
		return;

	if (!pmode)
		return;

	for (i = 0; i < num_clk; i++) {
		if (clk_khz == osc2_clk_data[i].clk_khz) {
			idx = i;
			break;
		}
	}

	if (i == num_clk) {
		dev_warn(ctx->dev, "Invalid OSC2 clock (%u)\n", clk_khz);
		return;
	}

	if (idx != spanel->hw_osc2_clk_idx) {
		spanel->hw_osc2_clk_idx = idx;
		ctx->osc2_clk_khz = clk_khz;

		/**
		 * Update the clock only when panel is in normal state. For other
		 * states, the value will be saved and applied when the state is
		 * changed to normal.
		 */
		if (ctx->panel_state == PANEL_STATE_NORMAL) {
			/* trigger update since OSC2 clock is changed */
			nt37290_update_panel_feat(ctx, pmode, true);
			dev_dbg(ctx->dev, "%s: %u (idx %d)\n", __func__, clk_khz, idx);
		} else {
			dev_dbg(ctx->dev, "%s: pending change for %u (idx %d)\n",
				__func__, clk_khz, idx);
		}
	}
}

static ssize_t nt37290_list_osc2_clk_khz(struct exynos_panel *ctx, char *buf)
{
	ssize_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(osc2_clk_data); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u\n",
			osc2_clk_data[i].clk_khz);

	return len;
}

static void nt37290_set_dimming_on(struct exynos_panel *ctx,
				 bool dimming_on)
{
	ctx->dimming_on = dimming_on;
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x53, ctx->dimming_on ? 0x28 : 0x20);
	dev_dbg(ctx->dev, "%s dimming_on=%d \n", __func__, dimming_on);
}

static int nt37290_read_id(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[NT37290_DDIC_ID_LEN] = {0};
	int ret;

	if (ctx->panel_rev < PANEL_REV_DVT1)
		return exynos_panel_read_id(ctx);

	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx,
		0xFF, 0xAA, 0x55, 0xA5, 0x81);
	ret = mipi_dsi_dcs_read(dsi, 0xF2, buf, NT37290_DDIC_ID_LEN);
	if (ret != NT37290_DDIC_ID_LEN) {
		dev_warn(ctx->dev, "Unable to read DDIC id (%d)\n", ret);
		return ret;
	}

	exynos_bin2hex(buf, NT37290_DDIC_ID_LEN,
		ctx->panel_id, sizeof(ctx->panel_id));
	return 0;
}

static const struct exynos_display_underrun_param underrun_param = {
	.te_idle_us = 350,
	.te_var = 1,
};

static const u32 nt37290_bl_range[] = {
	94, 180, 270, 360, 3584
};

static const int nt37290_vrefresh_range[] = {
	10, 30, 60, 120
};

static const int nt37290_lp_vrefresh_range[] = {
	10, 30
};

/* Truncate 8-bit signed value to 6-bit signed value */
#define TO_6BIT_SIGNED(v) (v & 0x3F)

static const struct drm_dsc_config nt37290_dsc_cfg = {
	.first_line_bpg_offset = 13,
	.rc_range_params = {
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{4, 10, TO_6BIT_SIGNED(-10)},
		{5, 10, TO_6BIT_SIGNED(-10)},
		{5, 11, TO_6BIT_SIGNED(-10)},
		{5, 11, TO_6BIT_SIGNED(-12)},
		{8, 12, TO_6BIT_SIGNED(-12)},
		{12, 13, TO_6BIT_SIGNED(-12)},
	},
};

#define NT37290_DSC_CONFIG \
	.dsc = { \
		.enabled = true, \
		.dsc_count = 2, \
		.slice_count = 2, \
		.slice_height = 30, \
		.cfg = &nt37290_dsc_cfg, \
	}

static const struct exynos_panel_mode nt37290_modes[] = {
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
			.bpc = 8,
			NT37290_DSC_CONFIG,
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_ON_SELF_REFRESH,
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
			.bpc = 8,
			NT37290_DSC_CONFIG,
			.underrun_param = &underrun_param,
		},
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
			.bpc = 8,
			NT37290_DSC_CONFIG,
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_ON_SELF_REFRESH,
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
			.bpc = 8,
			NT37290_DSC_CONFIG,
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_ON_INACTIVITY,
	},
};

static const struct exynos_panel_mode nt37290_lp_modes[] = {
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
			.bpc = 8,
			NT37290_DSC_CONFIG,
			.underrun_param = &underrun_param,
			.is_lp_mode = true,
		}
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
			.bpc = 8,
			NT37290_DSC_CONFIG,
			.underrun_param = &underrun_param,
			.is_lp_mode = true,
		}
	},
};

static void nt37290_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	struct dentry *csroot = debugfs_lookup("cmdsets", root);

	if (!csroot || !ctx)
		return;

	exynos_panel_debugfs_create_cmdset(ctx, csroot, &nt37290_init_cmd_set, "init");
	dput(csroot);
}

static void nt37290_panel_init(struct exynos_panel *ctx)
{
	struct nt37290_panel *spanel = to_spanel(ctx);

	exynos_panel_send_cmd_set(ctx, &nt37290_dsc_init_cmd_set);
	exynos_panel_send_cmd_set(ctx, &nt37290_lhbm_on_setting_cmd_set);

	ctx->osc2_clk_khz = osc2_clk_data[spanel->hw_osc2_clk_idx].clk_khz;
}

static int nt37290_panel_probe(struct mipi_dsi_device *dsi)
{
	struct nt37290_panel *spanel;

	spanel = devm_kzalloc(&dsi->dev, sizeof(*spanel), GFP_KERNEL);
	if (!spanel)
		return -ENOMEM;

	spanel->hw_vrefresh = 60;
	spanel->hw_idle_vrefresh = 0;
	spanel->auto_mode_vrefresh = 0;
	spanel->delayed_idle = false;
	spanel->hw_osc2_clk_idx = 0;

	return exynos_panel_common_init(dsi, &spanel->base);
}

static const struct drm_panel_funcs nt37290_drm_funcs = {
	.disable = nt37290_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = nt37290_enable,
	.get_modes = exynos_panel_get_modes,
	.debugfs_init = nt37290_debugfs_init,
};

static const struct exynos_panel_funcs nt37290_exynos_funcs = {
	.set_brightness = nt37290_set_brightness,
	.set_lp_mode = nt37290_set_lp_mode,
	.set_nolp_mode = nt37290_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.set_hbm_mode = nt37290_set_hbm_mode,
	.set_dimming_on = nt37290_set_dimming_on,
	.set_local_hbm_mode = nt37290_set_local_hbm_mode,
	.is_mode_seamless = nt37290_is_mode_seamless,
	.mode_set = nt37290_mode_set,
	.panel_init = nt37290_panel_init,
	.get_panel_rev = nt37290_get_panel_rev,
	.get_te2_edges = exynos_panel_get_te2_edges,
	.configure_te2_edges = exynos_panel_configure_te2_edges,
	.update_te2 = nt37290_update_te2,
	.set_self_refresh = nt37290_set_self_refresh,
	.commit_done = nt37290_commit_done,
	.set_osc2_clk_khz = nt37290_set_osc2_clk_khz,
	.list_osc2_clk_khz = nt37290_list_osc2_clk_khz,
	.read_id = nt37290_read_id,
};

const struct brightness_capability nt37290_brightness_capability = {
	.normal = {
		.nits = {
			.min = 2,
			.max = 600,
		},
		.level = {
			.min = 3,
			.max = 3584,
		},
		.percentage = {
			.min = 0,
			.max = 60,
		},
	},
	.hbm = {
		.nits = {
			.min = 600,
			.max = 1000,
		},
		.level = {
			.min = 3585,
			.max = 4094,
		},
		.percentage = {
			.min = 60,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc boe_nt37290 = {
	.panel_id_reg = 0xAC,
	.data_lane_cnt = 4,
	.max_brightness = 4094,
	.min_brightness = 3,
	.dft_brightness = 1023,
	.brt_capability = &nt37290_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 10000000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = nt37290_bl_range,
	.bl_num_ranges = ARRAY_SIZE(nt37290_bl_range),
	.modes = nt37290_modes,
	.num_modes = ARRAY_SIZE(nt37290_modes),
	.vrefresh_range = nt37290_vrefresh_range,
	.vrefresh_range_count = ARRAY_SIZE(nt37290_vrefresh_range),
	.off_cmd_set = &nt37290_off_cmd_set,
	.lp_mode = nt37290_lp_modes,
	.lp_mode_count = ARRAY_SIZE(nt37290_lp_modes),
	.lp_vrefresh_range = nt37290_lp_vrefresh_range,
	.lp_vrefresh_range_count = ARRAY_SIZE(nt37290_lp_vrefresh_range),
	.lp_cmd_set = &nt37290_lp_cmd_set,
	.binned_lp = nt37290_binned_lp,
	.num_binned_lp = ARRAY_SIZE(nt37290_binned_lp),
	.is_panel_idle_supported = true,
	.panel_func = &nt37290_drm_funcs,
	.exynos_panel_func = &nt37290_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "boe,nt37290", .data = &boe_nt37290 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = nt37290_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-boe-nt37290",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Chris Lu <luchris@google.com>");
MODULE_DESCRIPTION("MIPI-DSI based BOE nt37290 panel driver");
MODULE_LICENSE("GPL");
