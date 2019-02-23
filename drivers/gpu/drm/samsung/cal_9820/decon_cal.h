/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for Exynos9820 DECON CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DECON_CAL_H__
#define __SAMSUNG_DECON_CAL_H__

#include "exynos_panel.h"

#define MAX_WIN_PER_DECON	6
#define MAX_DECON_CNT		3

enum decon_regs_id {
	REGS_DECON0_ID = 0,
	REGS_DECON1_ID,
	REGS_DECON2_ID,
	REGS_DECON_ID_MAX
};

enum decon_regs_type {
	REGS_DECON = 0,
	REGS_DECON_SYS,
	REGS_DECON_TYPE_MAX
};

enum decon_idma_type {
	IDMA_G0 = 0,
	IDMA_G1,
	IDMA_VG0,
	IDMA_VG1,
	IDMA_VGF0,
	IDMA_VGF1, /* VGRF in case of Exynos9810 */
	ODMA_WB,
	MAX_DECON_DMA_TYPE,
};

#define IDMA_GF0	IDMA_G0
#define IDMA_GF1	IDMA_G1
#define IDMA_VG		IDMA_VG0
#define IDMA_VGF	IDMA_VG1
#define IDMA_VGS	IDMA_VGF0
#define IDMA_VGRFS	IDMA_VGF1

enum decon_fifo_mode {
	DECON_FIFO_00K = 0,
	DECON_FIFO_04K,
	DECON_FIFO_08K,
	DECON_FIFO_12K,
	DECON_FIFO_16K,
};

enum decon_dsi_mode {
	DSI_MODE_SINGLE = 0,
	DSI_MODE_DUAL_DSI,
	DSI_MODE_DUAL_DISPLAY,
	DSI_MODE_NONE
};

enum decon_data_path {
	/* No comp - OUTFIFO0 DSIM_IF0 */
	DPATH_NOCOMP_OUTFIFO0_DSIMIF0			= 0x001,
	/* No comp - FF0 - FORMATTER1 - DSIM_IF1 */
	DPATH_NOCOMP_OUTFIFO0_DSIMIF1			= 0x002,
	/* No comp - SPLITTER - FF0/1 - FORMATTER0/1 - DSIM_IF0/1 */
	DPATH_NOCOMP_SPLITTER_OUTFIFO01_DSIMIF01	= 0x003,
	/* No comp - OUTFIFO0 - WB */
	DPATH_NOCOMP_OUTFIFO0_WB			= 0x004,

	/* DSC_ENC0 - OUTFIFO0 - DSIM_IF0 */
	DPATH_DSCENC0_OUTFIFO0_DSIMIF0		= 0x011,
	/* DSC_ENC0 - OUTFIFO0 - DSIM_IF1 */
	DPATH_DSCENC0_OUTFIFO0_DSIMIF1		= 0x012,
	/* DSC_ENC0 - OUTFIFO0 - WB */
	DPATH_DSCENC0_OUTFIFO0_WB		= 0x014,

	/* DSCC,DSC_ENC0/1 - OUTFIFO01 DSIM_IF0 */
	DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF0	= 0x0B1,
	/* DSCC,DSC_ENC0/1 - OUTFIFO01 DSIM_IF1 */
	DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF1	= 0x0B2,
	/* DSCC,DSC_ENC0/1 - OUTFIFO01 DSIM_IF0/1 */
	DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF01	= 0x0B3,
	/* DSCC,DSC_ENC0/1 - OUTFIFO01 - WB */
	DPATH_DSCC_DSCENC01_OUTFIFO01_WB	= 0x0B4,

	/* WB_PRE */
	DPATH_WBPRE_ONLY			= 0x100,

	/* No comp - OUTFIFO0 DSIM_IF0 */
	DECON1_NOCOMP_OUTFIFO0_DSIMIF0	= 0x001,
	/* No comp - OUTFIFO0 DP_IF */
	DECON1_NOCOMP_OUTFIFO0_DPIF		= 0x008,
	/* DSC_ENC1 - OUTFIFO0 - DSIM_IF0 */
	DECON1_DSCENC1_OUTFIFO0_DSIMIF0	= 0x021,
	/* DSC_ENC1 - OUTFIFO0 - DP_IF */
	DECON1_DSCENC1_OUTFIFO0_DPIF	= 0x028,

	/* No comp - OUTFIFO0 DP_IF */
	DECON2_NOCOMP_OUTFIFO0_DPIF		= 0x008,
	/* DSC_ENC2 - OUTFIFO0 - DP_IF0 */
	DECON2_DSCENC2_OUTFIFO0_DPIF	= 0x048,
};

enum decon_scaler_path {
	SCALERPATH_OFF	= 0x0,
	SCALERPATH_VGF	= 0x1,
	SCALERPATH_VGRF	= 0x2,
};

enum decon_path_cfg {
	PATH_CON_ID_DSIM_IF0 = 0,
	PATH_CON_ID_DSIM_IF1 = 1,
	PATH_CON_ID_DP = 3,
	PATH_CON_ID_DUAL_DSC = 4,
	PATH_CON_ID_DSCC_EN = 7,
};

enum decon_op_mode {
	DECON_VIDEO_MODE = 0,
	DECON_MIPI_COMMAND_MODE = 1,
};

enum decon_trig_mode {
	DECON_HW_TRIG = 0,
	DECON_SW_TRIG
};

enum decon_out_type {
	DECON_OUT_DSI0 = 1 << 0,
	DECON_OUT_DSI1 = 1 << 1,
	DECON_OUT_DSI  = DECON_OUT_DSI0 | DECON_OUT_DSI1,

	DECON_OUT_DP0 = 1 << 4,
	DECON_OUT_DP1 = 1 << 5,
	DECON_OUT_DP  = DECON_OUT_DP0 | DECON_OUT_DP1,

	DECON_OUT_WB  = 1 << 8,
};

enum decon_rgb_order {
	DECON_RGB = 0x0,
	DECON_GBR = 0x1,
	DECON_BRG = 0x2,
	DECON_BGR = 0x4,
	DECON_RBG = 0x5,
	DECON_GRB = 0x6,
};

enum decon_win_func {
	PD_FUNC_CLEAR			= 0x0,
	PD_FUNC_COPY			= 0x1,
	PD_FUNC_DESTINATION		= 0x2,
	PD_FUNC_SOURCE_OVER		= 0x3,
	PD_FUNC_DESTINATION_OVER	= 0x4,
	PD_FUNC_SOURCE_IN		= 0x5,
	PD_FUNC_DESTINATION_IN		= 0x6,
	PD_FUNC_SOURCE_OUT		= 0x7,
	PD_FUNC_DESTINATION_OUT		= 0x8,
	PD_FUNC_SOURCE_A_TOP		= 0x9,
	PD_FUNC_DESTINATION_A_TOP	= 0xa,
	PD_FUNC_XOR			= 0xb,
	PD_FUNC_PLUS			= 0xc,
	PD_FUNC_USER_DEFINED		= 0xd,
};

enum decon_blending {
	DECON_BLENDING_NONE = 0,
	DECON_BLENDING_PREMULT = 1,
	DECON_BLENDING_COVERAGE = 2,
	DECON_BLENDING_MAX = 3,
};

enum decon_set_trig {
	DECON_TRIG_DISABLE = 0,
	DECON_TRIG_ENABLE
};

struct decon_mode {
	enum decon_op_mode op_mode;
	enum decon_dsi_mode dsi_mode;
	enum decon_trig_mode trig_mode;
};

struct decon_config {
	enum decon_out_type	out_type;
	unsigned int		image_height;
	unsigned int		image_width;
	struct decon_mode	mode;
	struct exynos_dsc	dsc;
};

struct win_color_map {
	bool en;
	u32 val;
};

struct decon_win_config {
	u32 ch_id;
	u32 start_pos;
	u32 end_pos;
	u32 start_time;
	struct win_color_map color_map;
};

struct decon_regs {
	void __iomem *base_addr;
	void __iomem *ss_regs;
};

struct decon_window_regs {
	u32 wincon;
	u32 start_pos;
	u32 end_pos;
	u32 colormap;
	u32 start_time;
	u32 pixel_count;
	u32 whole_w;
	u32 whole_h;
	u32 offset_x;
	u32 offset_y;
	u32 winmap_state;
	int ch;
	int plane_alpha;
	enum decon_blending blend;
};

struct decon_bts_bw {
	u32 val;
	u32 ch_num;
};

struct decon_dsc {
	unsigned int comp_cfg;
	unsigned int bit_per_pixel;
	unsigned int pic_height;
	unsigned int pic_width;
	unsigned int slice_height;
	unsigned int slice_width;
	unsigned int chunk_size;
	unsigned int initial_xmit_delay;
	unsigned int initial_dec_delay;
	unsigned int initial_scale_value;
	unsigned int scale_increment_interval;
	unsigned int scale_decrement_interval;
	unsigned int first_line_bpg_offset;
	unsigned int nfl_bpg_offset;
	unsigned int slice_bpg_offset;
	unsigned int initial_offset;
	unsigned int final_offset;
	unsigned int rc_range_parameters;
	unsigned int overlap_w;
	unsigned int width_per_enc;
	unsigned char *dec_pps_t;
};

void decon_regs_desc_init(void __iomem *regs, const char *name,
		enum decon_regs_type type, unsigned int id);

/*************** DECON CAL APIs exposed to DECON driver ***************/
/* DECON control */
int decon_reg_init(u32 id, struct decon_config *config);
int decon_reg_start(u32 id, struct decon_config *config);
int decon_reg_stop(u32 id, struct decon_config *config, bool rst, u32 fps);

/* DECON window control */
void decon_reg_set_win_enable(u32 id, u32 win_idx, u32 en);
void decon_reg_win_enable_and_update(u32 id, u32 win_idx, u32 en);
void decon_reg_all_win_shadow_update_req(u32 id);
void decon_reg_set_window_control(u32 id, int win_idx,
		struct decon_window_regs *regs, u32 winmap_en);
void decon_reg_update_req_window_mask(u32 id, u32 win_idx);
void decon_reg_update_req_window(u32 id, u32 win_idx);

/* DECON shadow update and trigger control */
void decon_reg_set_trigger(u32 id, struct decon_mode *mode,
		enum decon_set_trig en);
void decon_reg_update_req_and_unmask(u32 id, struct decon_mode *mode);
int decon_reg_wait_update_done_timeout(u32 id, unsigned long timeout_us);
int decon_reg_wait_update_done_and_mask(u32 id, struct decon_mode *mode,
		u32 timeout_us);

/* For window update and multi resolution feature */
int decon_reg_wait_idle_status_timeout(u32 id, unsigned long timeout);
void decon_reg_set_partial_update(u32 id, struct decon_config *config,
		bool in_slice[], u32 partial_w, u32 partial_h);
void decon_reg_set_mres(u32 id, struct decon_config *config);

/* For writeback configuration */
void decon_reg_release_resource(u32 id, struct decon_mode *mode);
void decon_reg_config_wb_size(u32 id, struct decon_config *config);

/* DECON interrupt control */
void decon_reg_set_interrupts(u32 id, u32 en);
int decon_reg_get_interrupt_and_clear(u32 id, u32 *ext_irq);

/* DECON SFR dump */
void __decon_dump(u32 id, void __iomem *regs, void __iomem *base_regs,
		bool dsc_en);

void decon_reg_set_start_crc(u32 id, u32 en);
void decon_reg_set_select_crc_bits(u32 id, u32 bit_sel);
void decon_reg_get_crc_data(u32 id, u32 *w0_data, u32 *w1_data);

/* DPU hw limitation check */
struct decon_device;
struct decon_win_config;

/* TODO: this will be removed later */
void decon_reg_update_req_global(u32 id);

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
/* PLL sleep related functions */
void decon_reg_set_pll_sleep(u32 id, u32 en);
void decon_reg_set_pll_wakeup(u32 id, u32 en);
#endif

/*********************************************************************/

#endif /* __SAMSUNG_DECON_CAL_H__ */
