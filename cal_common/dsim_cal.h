/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS101 DSIM CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DSIM_CAL_H__
#define __SAMSUNG_DSIM_CAL_H__

#include <drm/drm_print.h>
#include <exynos_panel.h>

#define MAX_DSI_CNT 2

#define DSIM_PIXEL_FORMAT_RGB24			0x3E
#define MAX_DSIM_DATALANE_CNT			4

/* EVT1 feature */
#define DPHY_PLL_CLK_GATE_EN	0

#define DSIM_LANE_CLOCK		(1 << 0)
#define DSIM_LANE_DATA0		(1 << 1)
#define DSIM_LANE_DATA1		(1 << 2)
#define DSIM_LANE_DATA2		(1 << 3)
#define DSIM_LANE_DATA3		(1 << 4)

#define MIPI_DSI_ERR_SOT			(1 << 0)
#define MIPI_DSI_ERR_SOT_SYNC			(1 << 1)
#define MIPI_DSI_ERR_EOT_SYNC			(1 << 2)
#define MIPI_DSI_ERR_ESCAPE_MODE_ENTRY_CMD	(1 << 3)
#define MIPI_DSI_ERR_LOW_POWER_TRANSMIT_SYNC	(1 << 4)
#define MIPI_DSI_ERR_HS_RECEIVE_TIMEOUT		(1 << 5)
#define MIPI_DSI_ERR_FALSE_CONTROL		(1 << 6)
#define MIPI_DSI_ERR_ECC_SINGLE_BIT		(1 << 8)
#define MIPI_DSI_ERR_ECC_MULTI_BIT		(1 << 9)
#define MIPI_DSI_ERR_CHECKSUM			(1 << 10)
#define MIPI_DSI_ERR_DATA_TYPE_NOT_RECOGNIZED	(1 << 11)
#define MIPI_DSI_ERR_VCHANNEL_ID_INVALID	(1 << 12)
#define MIPI_DSI_ERR_INVALID_TRANSMIT_LENGTH	(1 << 13)

/*
 * DSI_PROTOCAL_VIOLATION[15] is for protocol violation that is caused
 * EoTp missing So this bit is ignored because of not supporting
 * @S.LSI AP
 */
#define MIPI_DSI_ERR_PROTOCAL_VIOLATION		(1 << 15)
#define MIPI_DSI_ERR_BIT_MASK			(0x3f3f)

#define DSIM_VBP	0
#define DSIM_VSYNC	1
#define DSIM_V_ACTIVE	2
#define DSIM_VFP	3

#define RX_PHK_HEADER_SIZE	4
#define MAX_RX_FIFO		((64 * 4) - RX_PHK_HEADER_SIZE)
#define MAX_PH_FIFO		32
#define MAX_PL_FIFO		2048

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

enum {
	DSIM_COLOR_BAR = 0,
	DSIM_GRAY_GRADATION,
	DSIM_USER_DEFINED,
	DSIM_PRB7_RANDOM,
	DSIM_BIST_MODE_MAX,
};

struct dsim_clks {
	u32 hs_clk; /* megabits per second */
	u32 esc_clk;
	u32 byte_clk;
	u32 word_clk;
	u32 pending_hs_clk; /* megabits per second */
	bool hs_clk_changed;
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

/* Max registers number in one diag command: 1 clock + 4 data lanes */
#define MAX_DIAG_REG_NUM 5

struct dsim_dphy_diag {
	const char *name;
	const char *desc;
	const char *help;
	uint8_t num_reg;
	/* REGS_DSIM_PHY or REGS_DSIM_PHY_BIAS */
	uint8_t reg_base;
	uint16_t reg_offset[MAX_DIAG_REG_NUM];
	uint8_t bit_start;
	uint8_t bit_end;
	bool read_only;

	bool override;
	uint32_t user_value;
	void *private;
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
	unsigned int		bpp;
	unsigned int num_dphy_diags;
	struct dsim_dphy_diag *dphy_diags;
	bool			dual_dsi;
};

void dsim_regs_desc_init(void __iomem *reg_base, phys_addr_t start,
			 const char *name, enum dsim_regs_type type, unsigned int id);

/*************** DSIM CAL APIs exposed to DSIM driver ***************/
/* DSIM control */
void dsim_reg_init(u32 id, struct dsim_reg_config *config,
		struct dsim_clks *clks, bool panel_ctrl);
void dsim_reg_start(u32 id);
int dsim_reg_stop(u32 id, u32 lanes);
void dsim_reg_set_rr_config(u32 id, const struct dsim_reg_config *config,
		struct dsim_clks *clks);
bool dsim_reg_is_pll_stable(u32 id);

/* ULPS control */
int dsim_reg_exit_ulps_and_start(u32 id, u32 ddi_type, u32 lanes);
int dsim_reg_stop_and_enter_ulps(u32 id, u32 ddi_type, u32 lanes);

/* DSIM interrupt control */
int dsim_reg_get_int_and_clear(u32 id);
void dsim_reg_clear_int(u32 id, u32 int_src);

/* DSIM read/write command control */
void dsim_reg_wr_tx_header(u32 id, u8 di, u8 d0, u8 d1, bool bta);
void dsim_reg_wr_tx_payload(u32 id, u32 payload);
u32 dsim_reg_header_fifo_is_empty(u32 id);
u32 dsim_reg_payload_fifo_is_empty(u32 id);
u32 dsim_reg_get_rx_fifo(u32 id);
u32 dsim_reg_rx_fifo_is_empty(u32 id);
int dsim_reg_rx_err_handler(u32 id, u32 rx_fifo);
u32 dsim_reg_get_ph_cnt(u32 id);
bool dsim_reg_has_pend_cmd(u32 id);

/* For reading DSIM shadow SFR */
void dsim_reg_enable_shadow_read(u32 id, u32 en);

/* For window update and multi resolution feature */
void dsim_reg_function_reset(u32 id);
void dsim_reg_set_partial_update(u32 id, struct dsim_reg_config *config);
void dsim_reg_set_mres(u32 id, struct dsim_reg_config *config);

/* DSIM BIST for test */
void dsim_reg_set_bist(u32 id, bool en, u32 mode);

void dsim_reg_set_cmd_transfer_mode(u32 id, u32 lp);

/* Frequency Hopping feature for EVT1 */
void dsim_reg_set_dphy_freq_hopping(u32 id, u32 p, u32 m, u32 k, u32 en);

/* DSIM SFR dump */
void __dsim_dump(struct drm_printer *p, u32 id, struct dsim_regs *regs);

/* For dphy diagnosis */
u32 diag_dsim_dphy_reg_read_mask(u32 id, u16 offset, u32 mask);
u32 diag_dsim_dphy_extra_reg_read_mask(u32 id, u16 offset, u32 mask);
int dsim_dphy_diag_mask_from_range(uint8_t start, uint8_t end, uint32_t *mask);

void dsim_reg_enable_packetgo(u32 id, u32 en);
void dsim_reg_ready_packetgo(u32 id, u32 en);

void dsim_reg_set_drm_write_protected(u32 id, bool write_protected);
#endif /* __SAMSUNG_DSIM_CAL_H__ */
