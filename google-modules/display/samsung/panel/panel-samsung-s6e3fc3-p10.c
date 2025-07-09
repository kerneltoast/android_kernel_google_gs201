// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based s6e3fc3-p10 AMOLED LCD panel driver.
 *
 * Copyright (c) 2021 Google LLC
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
#include "trace/dpu_trace.h"

#include "panel-samsung-drv.h"

static const unsigned char PPS_SETTING[] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60,
	0x04, 0x38, 0x00, 0x30, 0x02, 0x1C, 0x02, 0x1C,
	0x02, 0x00, 0x02, 0x0E, 0x00, 0x20, 0x04, 0xA6,
	0x00, 0x07, 0x00, 0x0C, 0x02, 0x0B, 0x02, 0x1F,
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
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define S6E3FC3_P10_WRCTRLD_DIMMING_BIT    0x08
#define S6E3FC3_P10_WRCTRLD_BCTRL_BIT      0x20
#define S6E3FC3_P10_WRCTRLD_HBM_BIT        0xC0
#define S6E3FC3_P10_WRCTRLD_LOCAL_HBM_BIT  0x10

static const u8 display_off[] = { 0x28 };
static const u8 display_on[] = { 0x29 };
static const u8 test_key_on_f0[] = { 0xF0, 0x5A, 0x5A };
static const u8 test_key_off_f0[] = { 0xF0, 0xA5, 0xA5 };
static const u8 test_key_on_f1[] = { 0xF1, 0x5A, 0x5A };
static const u8 test_key_off_f1[] = { 0xF1, 0xA5, 0xA5 };
static const u8 freq_update[] = { 0xF7, 0x0F };
static const u8 new_gamma_ip_bypass[] = { 0x91, 0x01 };
static const u8 new_gamma_ip_enable[] = { 0x91, 0x02 };

static const struct exynos_dsi_cmd s6e3fc3_p10_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 0),
	EXYNOS_DSI_CMD_SEQ_DELAY(120, 0x10), /* sleep in */
};
static DEFINE_EXYNOS_CMD_SET(s6e3fc3_p10_off);

static const struct exynos_dsi_cmd s6e3fc3_p10_lp_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 0),
	EXYNOS_DSI_CMD0_REV(test_key_on_f0, PANEL_REV_GE(PANEL_REV_EVT1)),
	EXYNOS_DSI_CMD0_REV(new_gamma_ip_bypass, PANEL_REV_GE(PANEL_REV_EVT1)),
	EXYNOS_DSI_CMD0_REV(test_key_off_f0, PANEL_REV_GE(PANEL_REV_EVT1)),
};
static DEFINE_EXYNOS_CMD_SET(s6e3fc3_p10_lp);

static const struct exynos_dsi_cmd s6e3fc3_p10_lp_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 0)
};

static const struct exynos_dsi_cmd s6e3fc3_p10_lp_low_cmds[] = {
	EXYNOS_DSI_CMD_SEQ_DELAY(17, 0x53, 0x25), /* AOD 10 nit */
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_dsi_cmd s6e3fc3_p10_lp_high_cmds[] = {
	EXYNOS_DSI_CMD_SEQ_DELAY(17, 0x53, 0x24), /* AOD 50 nit */
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_binned_lp s6e3fc3_p10_binned_lp[] = {
	BINNED_LP_MODE("off", 0, s6e3fc3_p10_lp_off_cmds),
	/* rising time = delay = 0, falling time = delay + width = 0 + 16 */
	BINNED_LP_MODE_TIMING("low", 80, s6e3fc3_p10_lp_low_cmds, 0, 0 + 16),
	BINNED_LP_MODE_TIMING("high", 2047, s6e3fc3_p10_lp_high_cmds, 0, 0 + 16)
};

static const struct exynos_dsi_cmd s6e3fc3_p10_init_cmds[] = {
	EXYNOS_DSI_CMD_SEQ_DELAY(120, 0x11), /* sleep out */
	EXYNOS_DSI_CMD_SEQ(0x35), /* TE on */
	EXYNOS_DSI_CMD_SEQ(0x2A, 0x00, 0x00, 0x04, 0x37), /* CASET */
	EXYNOS_DSI_CMD_SEQ(0x2B, 0x00, 0x00, 0x09, 0x5F), /* PASET */

	EXYNOS_DSI_CMD0(test_key_on_f0),
	/* TE width */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x01, 0x09, 0x5C, 0x00, 0x0B),

	EXYNOS_DSI_CMD_SEQ(0xB0, 0x27, 0xF2), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x00), /* FQ CON */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x0E, 0xF2), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x04, 0xD4), /* HW, V-total */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x4C, 0xF6), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF6, 0x48, 0x21), /* Source Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x88, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x2D, 0x22), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x8E, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x2D, 0x22), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xA6, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x05, 0x18, 0x22, 0x21), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xBF, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x09, 0x28), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xC5, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x09, 0x28), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xB0, 0xDF, 0x66),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0x66, 0x71), /* EM off setting */
	EXYNOS_DSI_CMD0(freq_update),
	EXYNOS_DSI_CMD0(test_key_off_f0)
};
static DEFINE_EXYNOS_CMD_SET(s6e3fc3_p10_init);

static void s6e3fc3_p10_evt1_send_irc_setting_cmds(struct exynos_panel *ctx)
{
	dev_dbg(ctx->dev, "%s\n", __func__);
	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);

	/* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x28, 0xF2);
	/* 10 bits : must flush */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xCC);
	/* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0x18, 0x68);
	/* IRC setting */
	EXYNOS_DCS_BUF_ADD(ctx, 0x68, 0xE2, 0x0A, 0x0A, 0x0A, 0x82, 0x0E, 0xCB,
		0x80, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x90);
	/* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x28, 0xF2);
	/* 8 bits : must flush */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xC4);
	EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update);

	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);
}

static void s6e3fc3_p10_send_lhbm_location_cmds(struct exynos_panel *ctx)
{
	dev_dbg(ctx->dev, "%s\n", __func__);

	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f1);

	/* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x28, 0xF2);
	/* 10 bits : must flush */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xCC);
	/* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0x33, 0x68);
	/* box location */
	EXYNOS_DCS_BUF_ADD(ctx, 0x68, 0xCC);
	/* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0x34, 0x68);
	/* center position set, x: 0x21C, y: 0x6B1, size: 0x63 */
	EXYNOS_DCS_BUF_ADD(ctx, 0x68, 0x21, 0xC6, 0xB1, 0x63);
	/* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x28, 0xF2);
	/* 8 bits : must flush */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xC4);

	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_off_f1);
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);
}

static const struct exynos_dsi_cmd s6e3fc3_p10_mode_ns_60_cmds[] = {
	EXYNOS_DSI_CMD0(test_key_on_f0),
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x00), /* TE width (default) */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x27, 0xF2), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x80), /* FQ CON */
	EXYNOS_DSI_CMD_SEQ(0x60, 0x00), /* 60 hz NS */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x0E, 0xF2), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x00, 0x14), /* HW, V-total */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x4C, 0xF6), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF6, 0x48, 0x15), /* Source Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x88, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_EVT1), 0xCB, 0x1D, 0x30), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xCB, 0x1D, 0x16), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x8E, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_EVT1), 0xCB, 0x1D, 0x30), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xCB, 0x1D, 0x16), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xA6, 0xCB), /* Global Para */
	/* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_EVT1), 0xCB, 0x03, 0x10, 0x16, 0x30),
	/* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xCB, 0x03, 0x10, 0x16, 0x16),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xBF, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_EVT1), 0xCB, 0x06, 0x29), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xCB, 0x06, 0x34), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xC5, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_EVT1), 0xCB, 0x06, 0x29), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xCB, 0x06, 0x34), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xB0, 0xDF, 0x66),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0x66, 0x4C), /* EM off setting */
	EXYNOS_DSI_CMD0(freq_update),
	EXYNOS_DSI_CMD0(test_key_off_f0)
};
static DEFINE_EXYNOS_CMD_SET(s6e3fc3_p10_mode_ns_60);

static const struct exynos_dsi_cmd s6e3fc3_p10_mode_hs_60_cmds[] = {
	EXYNOS_DSI_CMD0(test_key_on_f0),
	/* TE width */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x01, 0x09, 0x5C, 0x00, 0x0B),

	EXYNOS_DSI_CMD_SEQ(0xB0, 0x27, 0xF2), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x00), /* FQ CON */
	EXYNOS_DSI_CMD_SEQ(0x60, 0x00), /* 60 hz HS */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x0E, 0xF2), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x04, 0xD4), /* HW, V-total */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x4C, 0xF6), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF6, 0x48, 0x21), /* Source Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x88, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x2D, 0x22), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x8E, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x2D, 0x22), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xA6, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x05, 0x18, 0x22, 0x21), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xBF, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x09, 0x28), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xC5, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x09, 0x28), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xB0, 0xDF, 0x66),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0x66, 0x71), /* EM off setting */
	EXYNOS_DSI_CMD0(freq_update),
	EXYNOS_DSI_CMD0(test_key_off_f0)
};
static DEFINE_EXYNOS_CMD_SET(s6e3fc3_p10_mode_hs_60);

static const struct exynos_dsi_cmd s6e3fc3_p10_mode_hs_90_cmds[] = {
	EXYNOS_DSI_CMD0(test_key_on_f0),
	/* TE width */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x01, 0x09, 0x5C, 0x00, 0x0B),

	EXYNOS_DSI_CMD_SEQ(0xB0, 0x27, 0xF2), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x00), /* FQ CON */
	EXYNOS_DSI_CMD_SEQ(0x60, 0x08), /* 90 hz HS */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x0E, 0xF2), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x04, 0xD4), /* HW, V-total */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x4C, 0xF6), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xF6, 0x48, 0x21), /* Source Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x88, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x2D, 0x22), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x8E, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x2D, 0x22), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xA6, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x05, 0x18, 0x22, 0x21), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xBF, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x09, 0x28), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0xC5, 0xCB), /* Global Para */
	EXYNOS_DSI_CMD_SEQ(0xCB, 0x09, 0x28), /* LTPS Change */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0xB0, 0xDF, 0x66),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_EVT1), 0x66, 0x71), /* EM off setting */
	EXYNOS_DSI_CMD0(freq_update),
	EXYNOS_DSI_CMD0(test_key_off_f0)
};
static DEFINE_EXYNOS_CMD_SET(s6e3fc3_p10_mode_hs_90);

#define LHBM_GAMMA_CMD_SIZE 6
/**
 * struct s6e3fc3_p10_panel - panel specific runtime info
 *
 * This struct maintains s6e3fc3_p10 panel specific runtime info, any fixed details about panel
 * should most likely go into struct exynos_panel_desc
 */
struct s6e3fc3_p10_panel {
	/** @base: base panel struct */
	struct exynos_panel base;

	/** @local_hbm_gamma: lhbm gamma data */
	struct local_hbm_gamma {
		u8 hs_cmd[LHBM_GAMMA_CMD_SIZE];
		u8 ns_cmd[LHBM_GAMMA_CMD_SIZE];
		u8 aod_cmd[LHBM_GAMMA_CMD_SIZE];
	} local_hbm_gamma;
};

#define to_spanel(ctx) container_of(ctx, struct s6e3fc3_p10_panel, base)

static void s6e3fc3_p10_lhbm_gamma_read(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct s6e3fc3_p10_panel *spanel = to_spanel(ctx);
	int ret;
	u8 *hs_cmd = spanel->local_hbm_gamma.hs_cmd;
	u8 *ns_cmd = spanel->local_hbm_gamma.ns_cmd;
	u8 *aod_cmd = spanel->local_hbm_gamma.aod_cmd;
	u8 buf[LHBM_GAMMA_CMD_SIZE * 2] = {0};

	dev_dbg(ctx->dev, "%s\n", __func__);

	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x28, 0xF2); /* global para */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xCC); /* 10 bit : must flush */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xB0, 0x00, 0x22, 0xD8); /* global para */
	ret = mipi_dsi_dcs_read(dsi, 0xD8, hs_cmd + 1, LHBM_GAMMA_CMD_SIZE - 1);
	if (ret == (LHBM_GAMMA_CMD_SIZE - 1)) {
		/* fill in gamma write command 0x65 in offset 0 */
		hs_cmd[0] = 0x65;
		exynos_bin2hex(hs_cmd + 1, LHBM_GAMMA_CMD_SIZE - 1,
			buf, sizeof(buf));
		dev_info(ctx->dev, "%s: hs_gamma: %s\n", __func__, buf);
	} else {
		dev_err(ctx->dev, "fail to read LHBM gamma for HS\n");
	}
	/* global para */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xB0, 0x00, 0x1D, 0xD8);
	ret = mipi_dsi_dcs_read(dsi, 0xD8, ns_cmd + 1, LHBM_GAMMA_CMD_SIZE - 1);
	if (ret == (LHBM_GAMMA_CMD_SIZE - 1)) {
		/* fill in gamma write command 0x65 in offset 0 */
		ns_cmd[0] = 0x65;
		exynos_bin2hex(ns_cmd + 1, LHBM_GAMMA_CMD_SIZE - 1,
			buf, sizeof(buf));
		dev_info(ctx->dev, "%s: ns_gamma: %s\n", __func__, buf);
	} else {
		dev_err(ctx->dev, "fail to read LHBM gamma for NS\n");
	}
	if (ctx->panel_rev >= PANEL_REV_EVT1) {
		/* global para */
		EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xB0, 0x00, 0x18, 0xD8);
		ret = mipi_dsi_dcs_read(dsi, 0xD8, aod_cmd + 1, LHBM_GAMMA_CMD_SIZE - 1);
		if (ret == (LHBM_GAMMA_CMD_SIZE - 1)) {
			/* fill in gamma write command 0x65 in offset 0 */
			aod_cmd[0] = 0x65;
			exynos_bin2hex(aod_cmd + 1, LHBM_GAMMA_CMD_SIZE - 1,
				buf, sizeof(buf));
			dev_info(ctx->dev, "%s: aod_gamma: %s\n", __func__, buf);
		} else {
			dev_err(ctx->dev, "fail to read LHBM gamma for AOD\n");
		}
	}
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x28, 0xF2); /* global para */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xC4); /* 8 bit : must flush */
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);
}

static void s6e3fc3_p10_lhbm_gamma_write(struct exynos_panel *ctx)
{
	struct s6e3fc3_p10_panel *spanel = to_spanel(ctx);
	const u8 *hs_cmd = spanel->local_hbm_gamma.hs_cmd;
	const u8 *ns_cmd = spanel->local_hbm_gamma.ns_cmd;
	const u8 *aod_cmd = spanel->local_hbm_gamma.aod_cmd;

	if (!hs_cmd[0] && !ns_cmd[0] && !aod_cmd[0]) {
		dev_err(ctx->dev, "%s: no lhbm gamma!\n", __func__);
		return;
	}

	dev_dbg(ctx->dev, "%s\n", __func__);
	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x28, 0xF2); /* global para */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xCC); /* 10 bit : must flush */
	if (hs_cmd[0]) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x03, 0xCD, 0x65); /* global para */
		EXYNOS_DCS_BUF_ADD_SET(ctx, spanel->local_hbm_gamma.hs_cmd); /* write gamma */
	}
	if (ns_cmd[0]) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x03, 0xC8, 0x65); /* global para */
		EXYNOS_DCS_BUF_ADD_SET(ctx, spanel->local_hbm_gamma.ns_cmd); /* write gamma */
	}
	if (aod_cmd[0]) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x03, 0xD7, 0x65); /* global para */
		EXYNOS_DCS_BUF_ADD_SET(ctx, spanel->local_hbm_gamma.aod_cmd); /* write gamma */
	}

	/* workaround of higher LHBM brightness with enabling HBM */
	if (ctx->panel_rev >= PANEL_REV_DVT1) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0x6C, 0x66);
		EXYNOS_DCS_BUF_ADD(ctx, 0x66, 0x39, 0x39, 0x39, 0x3C, 0x3C, 0x3C);
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0x90, 0x66);
		EXYNOS_DCS_BUF_ADD(ctx, 0x66, 0x2C, 0x2C, 0x2C, 0x32, 0x32, 0x32);
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0xB4, 0x66);
		EXYNOS_DCS_BUF_ADD(ctx, 0x66, 0x1E, 0x1E, 0x1E, 0x15, 0x15, 0x15);
	}

	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x28, 0xF2); /* global para */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xC4); /* 8 bit : must flush */
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);
}

static void s6e3fc3_p10_update_te2(struct exynos_panel *ctx)
{
	/* TE2 settings will not change after init. Still Need this to generate the sysfs node */
	return;
}

static void s6e3fc3_p10_send_te2_cmds(struct exynos_panel *ctx)
{
	u8 setting[2][4] = {
		{0xCB, 0x00, 0x00, 0x30}, // normal 60Hz
		{0xCB, 0x00, 0x00, 0x30}, // normal 90Hz
	};
	u8 lp_setting[4] = {0xCB, 0x00, 0x00, 0x10}; // lp low/high

	if (!ctx)
		return;

	dev_dbg(ctx->dev, "%s\n", __func__);
	DPU_ATRACE_BEGIN(__func__);
	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x28, 0xF2); /* global para  */
	/* must flush */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xCC); /* global para 10 bit */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x26, 0xF2); /* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xF2, 0x03, 0x14); /* TE2 on */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0xAF, 0xCB); /* global para */
	EXYNOS_DCS_BUF_ADD_SET(ctx, setting[0]); /* 60Hz control */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0x2F, 0xCB); /* global para */
	EXYNOS_DCS_BUF_ADD_SET(ctx, setting[1]); /* 90Hz control */

	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0xAF, 0xCB); /* global para */
	EXYNOS_DCS_BUF_ADD_SET(ctx, lp_setting); /* HLPM mode */

	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x28, 0xF2); /* global para */
	/* must flush */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF2, 0xC4); /* global para 8 bit */
	EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update); /* LTPS update */
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);
	DPU_ATRACE_END(__func__);
}

static void s6e3fc3_p10_change_frequency(struct exynos_panel *ctx,
					const unsigned int vrefresh)
{
	if (!ctx || (vrefresh != 60 && vrefresh != 90))
		return;

	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
	EXYNOS_DCS_BUF_ADD(ctx, 0x60, (vrefresh == 90) ? 0x08 : 0x00);
	EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update);
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);

	dev_dbg(ctx->dev, "%s: change to %uhz\n", __func__, vrefresh);
}

static int s6e3fc3_p10_set_op_hz(struct exynos_panel *ctx, unsigned int hz)
{
	const unsigned int vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);

	if (!is_panel_active(ctx))
		return -EPERM;

	if ((vrefresh > hz) || ((hz != 60) && (hz != 90))) {
		dev_err(ctx->dev, "invalid op_hz=%u for vrefresh=%u\n",
			hz, vrefresh);
		return -EINVAL;
	}

	ctx->op_hz = hz;
	if (ctx->op_hz == 60) {
		exynos_panel_send_cmd_set(ctx,
			&s6e3fc3_p10_mode_ns_60_cmd_set);
	} else {
		if (vrefresh == 60) {
			exynos_panel_send_cmd_set(ctx,
				&s6e3fc3_p10_mode_hs_60_cmd_set);
		} else {
			exynos_panel_send_cmd_set(ctx,
				&s6e3fc3_p10_mode_hs_90_cmd_set);
		}
	}
	dev_info(ctx->dev, "set op_hz at %u\n", hz);
	return 0;
}

static void s6e3fc3_p10_update_wrctrld(struct exynos_panel *ctx)
{
	u8 val = S6E3FC3_P10_WRCTRLD_BCTRL_BIT;

	if (IS_HBM_ON(ctx->hbm_mode))
		val |= S6E3FC3_P10_WRCTRLD_HBM_BIT;

	if (ctx->hbm.local_hbm.enabled)
		val |= S6E3FC3_P10_WRCTRLD_LOCAL_HBM_BIT;

	if (ctx->dimming_on)
		val |= S6E3FC3_P10_WRCTRLD_DIMMING_BIT;

	dev_dbg(ctx->dev,
		"%s(wrctrld:0x%x, hbm: %s, dimming: %s, local_hbm: %s)\n",
		__func__, val, IS_HBM_ON(ctx->hbm_mode) ? "on" : "off",
		ctx->dimming_on ? "on" : "off",
		ctx->hbm.local_hbm.enabled ? "on" : "off");

	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);

	/* TODO: need to perform gamma updates */
}

static void s6e3fc3_p10_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	unsigned int vrefresh = drm_mode_vrefresh(&pmode->mode);
	u32 delay_us = mult_frac(1000, 1020, vrefresh);

	if (!ctx->enabled)
		return;

	EXYNOS_DCS_BUF_ADD_SET(ctx, display_off);
	if (ctx->panel_rev >= PANEL_REV_EVT1) {
		EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
		EXYNOS_DCS_BUF_ADD_SET(ctx, new_gamma_ip_enable);
	}
	/* backlight control and dimming */
	s6e3fc3_p10_update_wrctrld(ctx);
	if (ctx->panel_rev >= PANEL_REV_EVT1)
		EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_off_f0);
	s6e3fc3_p10_change_frequency(ctx, vrefresh);
	usleep_range(delay_us, delay_us + 10);
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, display_on);

	dev_info(ctx->dev, "exit LP mode\n");
}

static int s6e3fc3_p10_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	const struct drm_display_mode *mode;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}
	mode = &pmode->mode;

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);

	exynos_panel_send_cmd_set(ctx, &s6e3fc3_p10_init_cmd_set);

	if (ctx->panel_rev == PANEL_REV_EVT1)
		s6e3fc3_p10_evt1_send_irc_setting_cmds(ctx);

	s6e3fc3_p10_change_frequency(ctx, drm_mode_vrefresh(mode));
	s6e3fc3_p10_send_te2_cmds(ctx);

	s6e3fc3_p10_lhbm_gamma_write(ctx);
	s6e3fc3_p10_send_lhbm_location_cmds(ctx);

	/* DSC related configuration */
	exynos_dcs_compression_mode(ctx, 0x1); /* DSC_DEC_ON */
	EXYNOS_PPS_LONG_WRITE(ctx); /* PPS_SETTING */
	EXYNOS_DCS_BUF_ADD(ctx, 0xC2, 0x14); /* PPS_MIC_OFF */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x9D, 0x01); /* PPS_DSC_EN */

	s6e3fc3_p10_update_wrctrld(ctx); /* dimming and HBM */

	ctx->enabled = true;

	if (pmode->exynos_mode.is_lp_mode)
		exynos_panel_set_lp_mode(ctx, pmode);
	else
		EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x29); /* display on */

	return 0;
}

static void s6e3fc3_p10_set_hbm_mode(struct exynos_panel *exynos_panel,
				enum exynos_hbm_mode mode)
{
	const bool hbm_update =
		(IS_HBM_ON(exynos_panel->hbm_mode) != IS_HBM_ON(mode));
	const bool irc_update =
		(IS_HBM_ON_IRC_OFF(exynos_panel->hbm_mode) != IS_HBM_ON_IRC_OFF(mode));

	exynos_panel->hbm_mode = mode;

	if (hbm_update)
		s6e3fc3_p10_update_wrctrld(exynos_panel);

	if (irc_update) {
		EXYNOS_DCS_BUF_ADD_SET(exynos_panel, test_key_on_f0);
		EXYNOS_DCS_BUF_ADD(exynos_panel, 0xB0, 0x03, 0x8F);
		EXYNOS_DCS_BUF_ADD(exynos_panel, 0x8F,
			IS_HBM_ON_IRC_OFF(mode) ? 0x05 : 0x25);
		EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(exynos_panel, test_key_off_f0);
	}
	dev_info(exynos_panel->dev, "hbm_on=%d hbm_ircoff=%d\n", IS_HBM_ON(exynos_panel->hbm_mode),
		 IS_HBM_ON_IRC_OFF(exynos_panel->hbm_mode));
}

static void s6e3fc3_p10_set_dimming_on(struct exynos_panel *exynos_panel,
				 bool dimming_on)
{
	const struct exynos_panel_mode *pmode = exynos_panel->current_mode;

	exynos_panel->dimming_on = dimming_on;
	if (pmode->exynos_mode.is_lp_mode) {
		dev_info(exynos_panel->dev,"in lp mode, skip to update\n");
		return;
	}
	s6e3fc3_p10_update_wrctrld(exynos_panel);
}

static void s6e3fc3_p10_set_local_hbm_mode(struct exynos_panel *exynos_panel,
				 bool local_hbm_en)
{
	s6e3fc3_p10_update_wrctrld(exynos_panel);
}

static void s6e3fc3_p10_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{
	s6e3fc3_p10_change_frequency(ctx, drm_mode_vrefresh(&pmode->mode));
}

static bool s6e3fc3_p10_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	/* seamless mode switch is possible if only changing refresh rate */
	return drm_mode_equal_no_clocks(&ctx->current_mode->mode, &pmode->mode);
}

static void s6e3fc3_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	struct dentry *csroot = debugfs_lookup("cmdsets", root);

	if (!csroot || !ctx)
		return;

	exynos_panel_debugfs_create_cmdset(ctx, csroot,
					   &s6e3fc3_p10_init_cmd_set, "init");
	dput(csroot);
}

static void s6e3fc3_p10_panel_init(struct exynos_panel *ctx)
{
	s6e3fc3_p10_lhbm_gamma_read(ctx);
	s6e3fc3_p10_lhbm_gamma_write(ctx);
	s6e3fc3_p10_send_te2_cmds(ctx);
}

static int s6e3fc3_p10_read_id(struct exynos_panel *ctx)
{
	if (ctx->panel_rev < PANEL_REV_DVT1)
		return exynos_panel_read_id(ctx);
	return exynos_panel_read_ddic_id(ctx);
}

static void s6e3fc3_p10_get_panel_rev(struct exynos_panel *ctx, u32 id)
{
	/* extract command 0xDB */
	u8 build_code = (id & 0xFF00) >> 8;
	u8 rev = ((build_code & 0xE0) >> 3) | ((build_code & 0x0C) >> 2);

	exynos_panel_get_panel_rev(ctx, rev);
}

static int s6e3fc3_p10_panel_probe(struct mipi_dsi_device *dsi)
{
	struct s6e3fc3_p10_panel *spanel;

	spanel = devm_kzalloc(&dsi->dev, sizeof(*spanel), GFP_KERNEL);
	if (!spanel)
		return -ENOMEM;

	spanel->base.op_hz = 90;

	return exynos_panel_common_init(dsi, &spanel->base);
}

static bool check_for_8_bit_10_bit_cmds(struct exynos_panel *ctx,
	const struct mipi_dsi_msg *msg, const bool is_last)
{
	static bool has_8_bit = false;
	static bool has_10_bit = false;
	static bool global_para_bit_set = false;
	const u8 *tx_buf = msg->tx_buf;

	if (global_para_bit_set && msg->tx_len == 2 && tx_buf[0] == 0xF2 && tx_buf[1] == 0xCC) {
		has_10_bit = true;
		if (!is_last)
			dev_warn(ctx->dev, "10 bit DDIC command is not flushed!\n");
	} else if (global_para_bit_set && msg->tx_len == 2 && tx_buf[0] == 0xF2 && tx_buf[1] == 0xC4) {
		has_8_bit = true;
		if (!is_last)
			dev_warn(ctx->dev, "8 bit DDIC command is not flushed!\n");
	}

	global_para_bit_set = (
		(msg->tx_len == 3 && tx_buf[0] == 0xB0 && tx_buf[1] == 0x28 && tx_buf[2] == 0xF2) ||
		(msg->tx_len == 4 && tx_buf[0] == 0xB0 && tx_buf[1] == 0x00 && tx_buf[2] == 0x28 &&
			tx_buf[3] == 0xF2));

	if (has_8_bit && has_10_bit) {
		has_8_bit = false;
		has_10_bit = false;
		return true;
	}
	if (is_last) {
		has_8_bit = false;
		has_10_bit = false;
	}

	return false;
}

static void s6e3fc3_p10_on_queue_ddic_command(struct exynos_panel *ctx,
	const struct mipi_dsi_msg *msg, const bool is_last)
{
	if (check_for_8_bit_10_bit_cmds(ctx, msg, is_last)) {
		dev_warn(ctx->dev, "DSIM Contains both 8 and 10 bit DDIC commands in the same batch!\n");
		dump_stack();
	}
}


static const struct exynos_display_underrun_param underrun_param = {
	.te_idle_us = 1000,
	.te_var = 1,
};

static const u32 s6e3fc3_p10_bl_range[] = {
	95, 205, 315, 400, 2047
};

static const struct exynos_panel_mode s6e3fc3_p10_modes[] = {
	{
		/* 1080x2400 @ 60Hz */
		.mode = {
			.name = "1080x2400x60",
			.clock = 168498,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32, // add hfp
			.hsync_end = 1080 + 32 + 12, // add hsa
			.htotal = 1080 + 32 + 12 + 26, // add hbp
			.vdisplay = 2400,
			.vsync_start = 2400 + 12, // add vfp
			.vsync_end = 2400 + 12 + 4, // add vsa
			.vtotal = 2400 + 12 + 4 + 26, // add vbp
			.flags = 0,
			.width_mm = 66,
			.height_mm = 146,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 5760,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 48,
			},
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 0 + 48,
		},
	},
	{
		/* 1080x2400 @ 90Hz */
		.mode = {
			.name = "1080x2400x90",
			.clock = 252747,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32, // add hfp
			.hsync_end = 1080 + 32 + 12, // add hsa
			.htotal = 1080 + 32 + 12 + 26, // add hbp
			.vdisplay = 2400,
			.vsync_start = 2400 + 12, // add vfp
			.vsync_end = 2400 + 12 + 4, // add vsa
			.vtotal = 2400 + 12 + 4 + 26, // add vbp
			.flags = 0,
			.width_mm = 66,
			.height_mm = 146,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 215,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 48,
			},
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 0 + 48,
		},
	},
};

static const struct exynos_panel_mode s6e3fc3_p10_lp_mode = {
	.mode = {
		/* 1080x2400 @ 30Hz */
		.name = "1080x2400x30",
		.clock = 84249,
		.hdisplay = 1080,
		.hsync_start = 1080 + 32, // add hfp
		.hsync_end = 1080 + 32 + 12, // add hsa
		.htotal = 1080 + 32 + 12 + 26, // add hbp
		.vdisplay = 2400,
		.vsync_start = 2400 + 12, // add vfp
		.vsync_end = 2400 + 12 + 4, // add vsa
		.vtotal = 2400 + 12 + 4 + 26, // add vbp
		.flags = 0,
		.type = DRM_MODE_TYPE_DRIVER,
		.width_mm = 66,
		.height_mm = 146,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		.dsc = {
			.enabled = true,
			.dsc_count = 2,
			.slice_count = 2,
			.slice_height = 48,
		},
		.underrun_param = &underrun_param,
		.is_lp_mode = true,
	}
};

static const struct drm_panel_funcs s6e3fc3_p10_drm_funcs = {
	.disable = exynos_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3fc3_p10_enable,
	.get_modes = exynos_panel_get_modes,
	.debugfs_init = s6e3fc3_debugfs_init,
};

static const struct exynos_panel_funcs s6e3fc3_p10_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_lp_mode = exynos_panel_set_lp_mode,
	.set_nolp_mode = s6e3fc3_p10_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.set_hbm_mode = s6e3fc3_p10_set_hbm_mode,
	.set_dimming_on = s6e3fc3_p10_set_dimming_on,
	.set_local_hbm_mode = s6e3fc3_p10_set_local_hbm_mode,
	.is_mode_seamless = s6e3fc3_p10_is_mode_seamless,
	.mode_set = s6e3fc3_p10_mode_set,
	.panel_init = s6e3fc3_p10_panel_init,
	.get_panel_rev = s6e3fc3_p10_get_panel_rev,
	.get_te2_edges = exynos_panel_get_te2_edges,
	.configure_te2_edges = exynos_panel_configure_te2_edges,
	.update_te2 = s6e3fc3_p10_update_te2,
	.set_op_hz = s6e3fc3_p10_set_op_hz,
	.read_id = s6e3fc3_p10_read_id,
	.on_queue_ddic_cmd = s6e3fc3_p10_on_queue_ddic_command,
};

const struct brightness_capability s6e3fc3_p10_brightness_capability = {
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
			.max = 50,
		},
	},
	.hbm = {
		.nits = {
			.min = 500,
			.max = 1000,
		},
		.level = {
			.min = 2048,
			.max = 4095,
		},
		.percentage = {
			.min = 50,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc samsung_s6e3fc3_p10 = {
	.dsc_pps = PPS_SETTING,
	.dsc_pps_len = ARRAY_SIZE(PPS_SETTING),
	.data_lane_cnt = 4,
	.max_brightness = 4095,
	.min_brightness = 4,
	.dft_brightness = 1023,
	.brt_capability = &s6e3fc3_p10_brightness_capability,
	.dbv_extra_frame = true,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 10000000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = s6e3fc3_p10_bl_range,
	.bl_num_ranges = ARRAY_SIZE(s6e3fc3_p10_bl_range),
	.modes = s6e3fc3_p10_modes,
	.num_modes = ARRAY_SIZE(s6e3fc3_p10_modes),
	.off_cmd_set = &s6e3fc3_p10_off_cmd_set,
	.lp_mode = &s6e3fc3_p10_lp_mode,
	.lp_cmd_set = &s6e3fc3_p10_lp_cmd_set,
	.binned_lp = s6e3fc3_p10_binned_lp,
	.num_binned_lp = ARRAY_SIZE(s6e3fc3_p10_binned_lp),
	.panel_func = &s6e3fc3_p10_drm_funcs,
	.exynos_panel_func = &s6e3fc3_p10_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,s6e3fc3-p10", .data = &samsung_s6e3fc3_p10 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = s6e3fc3_p10_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-s6e3fc3-p10",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("William Tai <williamtai@google.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung s6e3fc3-p10 panel driver");
MODULE_LICENSE("GPL");
