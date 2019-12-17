/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DSIM_CAL_H__
#define __DSIM_CAL_H__

#include <exynos_panel.h>

#define MAX_DSI_CNT 2

#define DSIM_PIXEL_FORMAT_RGB24			0x3E
#define DSIM_PIXEL_FORMAT_RGB18_PACKED		0x1E
#define DSIM_PIXEL_FORMAT_RGB18			0x2E
#define DSIM_PIXEL_FORMAT_RGB30_PACKED		0x0D
#define MAX_DSIM_DATALANE_CNT			4

/* EVT1 feature */
#define DPHY_PLL_CLK_GATE_EN	0
#define DPHY_PLL_SLEEP_EN	0

enum dsim_regs_id {
	REGS_DSIM0_ID = 0,
	REGS_DSIM1_ID,
	REGS_DSIM_ID_MAX
};

enum dsim_regs_type {
	REGS_DSIM_DSI = 0,
	REGS_DSIM_PHY,
	REGS_DSIM_PHY_BIAS,
	REGS_DSIM_SYS,
	REGS_DSIM_TYPE_MAX
};

/* define dsi bist pattern */
enum {
	DSIM_COLOR_BAR = 0,
	DSIM_GRAY_GRADATION,
	DSIM_USER_DEFINED,
	DSIM_PRB7_RANDOM,
};

/* define DSI lane types. */
enum {
	DSIM_LANE_CLOCK	= (1 << 0),
	DSIM_LANE_DATA0	= (1 << 1),
	DSIM_LANE_DATA1	= (1 << 2),
	DSIM_LANE_DATA2	= (1 << 3),
	DSIM_LANE_DATA3	= (1 << 4),
};

/* DSI Error report bit definitions */
enum {
	MIPI_DSI_ERR_SOT			= (1 << 0),
	MIPI_DSI_ERR_SOT_SYNC			= (1 << 1),
	MIPI_DSI_ERR_EOT_SYNC			= (1 << 2),
	MIPI_DSI_ERR_ESCAPE_MODE_ENTRY_CMD	= (1 << 3),
	MIPI_DSI_ERR_LOW_POWER_TRANSMIT_SYNC	= (1 << 4),
	MIPI_DSI_ERR_HS_RECEIVE_TIMEOUT		= (1 << 5),
	MIPI_DSI_ERR_FALSE_CONTROL		= (1 << 6),
	/* Bit 7 is reserved */
	MIPI_DSI_ERR_ECC_SINGLE_BIT		= (1 << 8),
	MIPI_DSI_ERR_ECC_MULTI_BIT		= (1 << 9),
	MIPI_DSI_ERR_CHECKSUM			= (1 << 10),
	MIPI_DSI_ERR_DATA_TYPE_NOT_RECOGNIZED	= (1 << 11),
	MIPI_DSI_ERR_VCHANNEL_ID_INVALID	= (1 << 12),
	MIPI_DSI_ERR_INVALID_TRANSMIT_LENGTH	= (1 << 13),

	/* Bit 14 is reserved */

	/*
	 * DSI_PROTOCAL_VIOLATION[15] is for protocol violation that is caused
	 * EoTp missing So this bit is egnored because of not supportung
	 * @S.LSI AP
	 */
	MIPI_DSI_ERR_PROTOCAL_VIOLATION		= (1 << 15),
	MIPI_DSI_ERR_BIT_MASK			= (0x3f3f),
};

/* define video timer interrupt */
enum {
	DSIM_VBP = 0,
	DSIM_VSYNC,
	DSIM_V_ACTIVE,
	DSIM_VFP,
};

struct dsim_clks {
	unsigned int hs_clk;
	unsigned int esc_clk;
	unsigned int byte_clk;
	unsigned int word_clk;
};

struct dsim_regs {
	void __iomem *regs;
	void __iomem *ss_regs;
	void __iomem *phy_regs;
	void __iomem *phy_regs_ex;
};

struct dsim_pll_param {
	char *name;
	unsigned int p;
	unsigned int m;
	unsigned int s;
	unsigned int k;
	unsigned int mfr;
	unsigned int mrr;
	unsigned int sel_pf;
	unsigned int icp;
	unsigned int afc_enb;
	unsigned int extafc;
	unsigned int feed_en;
	unsigned int fsel;
	unsigned int fout_mask;
	unsigned int rsel;
	bool dither_en;

	unsigned int pll_freq; /* in/out parameter: Mhz */
	unsigned int esc_freq; /* Mhz */
	unsigned int cmd_underrun_cnt;
};

enum dsim_panel_mode {
	DSIM_VIDEO_MODE = 0,
	DSIM_COMMAND_MODE,
};

struct stdphy_pms {
	unsigned int p;
	unsigned int m;
	unsigned int s;
	unsigned int k;
	unsigned int mfr;
	unsigned int mrr;
	unsigned int sel_pf;
	unsigned int icp;
	unsigned int afc_enb;
	unsigned int extafc;
	unsigned int feed_en;
	unsigned int fsel;
	unsigned int fout_mask;
	unsigned int rsel;
	bool dither_en;
};

struct dphy_timing_value {
	unsigned int bps;
	unsigned int clk_prepare;
	unsigned int clk_zero;
	unsigned int clk_post;
	unsigned int clk_trail;
	unsigned int hs_prepare;
	unsigned int hs_zero;
	unsigned int hs_trail;
	unsigned int lpx;
	unsigned int hs_exit;
	unsigned int b_dphyctl;
};

struct dsim_reg_config {
	struct dpu_panel_timing	p_timing;
	enum dsim_panel_mode	mode;
	struct stdphy_pms	dphy_pms;
	enum type_of_ddi	ddi_type;
	unsigned int		data_lane_cnt;
	unsigned int		vt_compensation;
	struct exynos_dsc	dsc;
	unsigned int		mres_mode;
	unsigned int		cmd_underrun_cnt[MAX_RES_NUMBER];
};

void dsim_regs_desc_init(void __iomem *reg_base, const char *name,
		enum dsim_regs_type type, unsigned int id);

/*************** DSIM CAL APIs exposed to DSIM driver ***************/
/* DSIM control */
void dsim_reg_preinit(u32 id);
void dsim_reg_init(u32 id, struct dsim_reg_config *config,
		struct dsim_clks *clks, bool panel_ctrl);
void dsim_reg_start(u32 id);
int dsim_reg_stop(u32 id, u32 lanes);

/* ULPS control */
int dsim_reg_exit_ulps_and_start(u32 id, u32 ddi_type, u32 lanes);
int dsim_reg_stop_and_enter_ulps(u32 id, u32 ddi_type, u32 lanes);

/* DSIM interrupt control */
int dsim_reg_get_int_and_clear(u32 id);
void dsim_reg_clear_int(u32 id, u32 int_src);

/* DSIM read/write command control */
void dsim_reg_wr_tx_header(u32 id, u32 d_id, unsigned long d0, u32 d1, u32 bta);
void dsim_reg_wr_tx_payload(u32 id, u32 payload);
u32 dsim_reg_header_fifo_is_empty(u32 id);
u32 dsim_reg_is_writable_fifo_state(u32 id);
u32 dsim_reg_get_rx_fifo(u32 id);
u32 dsim_reg_rx_fifo_is_empty(u32 id);
int dsim_reg_rx_err_handler(u32 id, u32 rx_fifo);

/* For reading DSIM shadow SFR */
void dsim_reg_enable_shadow_read(u32 id, u32 en);

/* For window update and multi resolution feature */
void dsim_reg_function_reset(u32 id);
void dsim_reg_set_partial_update(u32 id, struct dsim_reg_config *config);
void dsim_reg_set_mres(u32 id, struct dsim_reg_config *config);

/* DSIM BIST for test */
void dsim_reg_set_bist(u32 id, u32 en);

void dsim_reg_set_cmd_transfer_mode(u32 id, u32 lp);

#if !defined(CONFIG_SOC_EXYNOS9820_EVT0)
/* Frequency Hopping feature for EVT1 */
void dsim_reg_set_dphy_freq_hopping(u32 id, u32 p, u32 m, u32 k, u32 en);
#endif

/* DSIM SFR dump */
void __dsim_dump(u32 id, struct dsim_regs *regs);


int dsim_set_panel_power(int id, bool on);
#endif /* __DSIM_CAL_H__ */
