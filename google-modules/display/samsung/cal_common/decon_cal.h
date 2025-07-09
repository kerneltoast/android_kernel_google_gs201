/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS101/GS201/ZUMA DECON CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DECON_CAL_H__
#define __SAMSUNG_DECON_CAL_H__

#include <cal_config.h>
#include <exynos_panel.h>

#ifdef CONFIG_SOC_ZUMA
#define MAX_WIN_PER_DECON	14
#define MAX_AXI_PORT		2
#define NUM_INTERCONNECT_CH	2
#else
#define MAX_WIN_PER_DECON	6
#define MAX_AXI_PORT		3
#define NUM_INTERCONNECT_CH	4
#endif

#define MAX_DECON_CNT		3
#define BTS_DFS_MAX		7 /* DPU DVFS Level */

#define DECON_BLENDING_PREMULT		0
#define DECON_BLENDING_COVERAGE		1
#define DECON_BLENDING_NONE		2

#ifdef CONFIG_SOC_GS201
#define MAX_RCD_CNT		2
#else
#define MAX_RCD_CNT		0
#endif

/* connected IDMA starts ahead by 32 lines based on y-position */
#define DECON_WIN_START_TIME			32

enum decon_regs_id {
	REGS_DECON0_ID = 0,
	REGS_DECON1_ID,
	REGS_DECON2_ID,
	REGS_DECON_ID_MAX
};

enum decon_regs_type {
	REGS_DECON = 0,
	REGS_DECON_SYS,
	REGS_DECON_WIN,
	REGS_DECON_SUB,
	REGS_DECON_WINCON,
	REGS_DECON_TYPE_MAX
};

extern struct cal_regs_desc regs_decon[REGS_DECON_TYPE_MAX][REGS_DECON_ID_MAX];

#define decon_regs_desc(id)			(&regs_decon[REGS_DECON][id])
#define decon_read(id, offset)			\
	cal_read(decon_regs_desc(id), offset)
#define decon_write(id, offset, val)		\
	cal_write(decon_regs_desc(id), offset, val)
#define decon_read_mask(id, offset, mask)	\
	cal_read_mask(decon_regs_desc(id), offset, mask)
#define decon_write_mask(id, offset, val, mask)	\
	cal_write_mask(decon_regs_desc(id), offset, val, mask)

#define win_regs_desc(id)			\
	(&regs_decon[REGS_DECON_WIN][id])
#define win_read(id, offset)			\
	cal_read(win_regs_desc(id), offset)
#define win_write(id, offset, val)		\
	cal_write(win_regs_desc(id), offset, val)
#define win_read_mask(id, offset, mask)		\
	cal_read_mask(win_regs_desc(id), offset, mask)
#define win_write_mask(id, offset, val, mask)	\
	cal_write_mask(win_regs_desc(id), offset, val, mask)

#define wincon_regs_desc(id)				\
	(&regs_decon[REGS_DECON_WINCON][id])
#define wincon_read(id, offset)				\
	cal_read(wincon_regs_desc(id), offset)
#define wincon_write(id, offset, val)			\
	cal_write(wincon_regs_desc(id), offset, val)
#define wincon_read_mask(id, offset, mask)		\
	cal_read_mask(wincon_regs_desc(id), offset, mask)
#define wincon_write_mask(id, offset, val, mask)	\
	cal_write_mask(wincon_regs_desc(id), offset, val, mask)

#define sub_regs_desc(id)			\
	(&regs_decon[REGS_DECON_SUB][id])
#define dsimif_read(id, offset)			\
	cal_read(sub_regs_desc(id), offset)
#define dsimif_write(id, offset, val)		\
	cal_write(sub_regs_desc(id), offset, val)
#define dsimif_read_mask(id, offset, mask)		\
	cal_read_mask(sub_regs_desc(id), offset, mask)
#define dsimif_write_mask(id, offset, val, mask)	\
	cal_write_mask(sub_regs_desc(id), offset, val, mask)

#define dpif_read(id, offset)			\
	cal_read(sub_regs_desc(id), offset)
#define dpif_write(id, offset, val)			\
	cal_write(sub_regs_desc(id), offset, val)
#define dpif_read_mask(id, offset, mask)		\
	cal_read_mask(sub_regs_desc(id), offset, mask)
#define dpif_write_mask(id, offset, val, mask)	\
	cal_write_mask(sub_regs_desc(id), offset, val, mask)

#define dsc_read(id, offset)			\
	cal_read(sub_regs_desc(id), offset)
#define dsc_write(id, offset, val)			\
	cal_write(sub_regs_desc(id), offset, val)
#define dsc_read_mask(id, offset, mask)		\
	cal_read_mask(sub_regs_desc(id), offset, mask)
#define dsc_write_mask(id, offset, val, mask)	\
	cal_write_mask(sub_regs_desc(id), offset, val, mask)

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
	/* No comp - OUTFIFO0/1/2 - WBIF */
	DPATH_NOCOMP_OUTFIFO0_WBIF			= 0x004,

	/* DSC_ENC0 - OUTFIFO0 - DSIM_IF0 */
	DPATH_DSCENC0_OUTFIFO0_DSIMIF0		= 0x011,
	/* DSC_ENC0 - OUTFIFO0 - DSIM_IF1 */
	DPATH_DSCENC0_OUTFIFO0_DSIMIF1		= 0x012,
	/* DSC_ENC0 - OUTFIFO0 - WBIF */
	DPATH_DSCENC0_OUTFIFO0_WBIF		= 0x014,

	/* DSCC,DSC_ENC0/1 - OUTFIFO01 DSIM_IF0 */
	DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF0	= 0x0B1,
	/* DSCC,DSC_ENC0/1 - OUTFIFO01 DSIM_IF1 */
	DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF1	= 0x0B2,
	/* DSCC,DSC_ENC0/1 - OUTFIFO01 DSIM_IF0/1 */
	DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF01	= 0x0B3,
	/* DSCC,DSC_ENC0/1 - OUTFIFO01 - WBIF */
	DPATH_DSCC_DSCENC01_OUTFIFO01_WBIF	= 0x0B4,

	/* No comp - OUTFIFO0 DSIM_IF0 */
	DECON1_NOCOMP_OUTFIFO1_DSIMIF0		= 0x001,
	/* No comp - OUTFIFO0 DP_IF */
	DECON1_NOCOMP_OUTFIFO1_DPIF		= 0x008,
	/* DSC_ENC1 - OUTFIFO0 - DSIM_IF0 */
	DECON1_DSCENC1_OUTFIFO1_DSIMIF0		= 0x021,
	/* DSC_ENC1 - OUTFIFO0 - DP_IF */
	DECON1_DSCENC1_OUTFIFO1_DPIF		= 0x028,
	/* DSC_ENC1 - OUTFIFO1 - WBIF */
	DECON1_DSCENC1_OUTFIFO1_WBIF		= 0x024,

	/* No comp - OUTFIFO2 DP_IF */
	DECON2_NOCOMP_OUTFIFO2_DPIF		= 0x008,
	/* DSC_ENC2 - OUTFIFO2 - DP_IF0 */
	DECON2_DSCENC2_OUTFIFO2_DPIF		= 0x048,
	/* DSC_ENC2 - OUTFIFO2 - WBIF */
	DECON2_DSCENC2_OUTFIFO2_WBIF		= 0x044,
};

enum decon_enhance_path {
	ENHANCEPATH_ENHANCE_ALL_OFF	= 0x0,
	ENHANCEPATH_DITHER_ON		= 0x1,
	ENHANCEPATH_DQE_ON		= 0x2,
	ENHANCEPATH_DQE_DITHER_ON	= 0x3, /* post ENH stage is 8bpc */
	ENHANCEPATH_RCD_ON		= 0x4,
	ENHANCEPATH_RCD_DQE_ON		= 0x6,
	ENHANCEPATH_RCD_DQE_DITHER_ON	= 0x7,
};

enum decon_path_cfg {
	PATH_CON_ID_DSIM_IF0 = 0,
	PATH_CON_ID_DSIM_IF1 = 1,
	PATH_CON_ID_WB = 2,
	PATH_CON_ID_DP = 3,
	PATH_CON_ID_DUAL_DSC = 4,
	PATH_CON_ID_DSC0 = 4,
	PATH_CON_ID_DSC1 = 5,
	PATH_CON_ID_DSC2 = 6,
	PATH_CON_ID_DSCC_EN = 7,
};

enum decon_op_mode {
	DECON_VIDEO_MODE = 0,
	DECON_COMMAND_MODE = 1,
};

enum decon_trig_mode {
	DECON_HW_TRIG = 0,
	DECON_SW_TRIG
};

enum decon_enh_path {
	DECON_ENH_DITHER = 1 << 0,
	DECON_ENH_DQE    = 1 << 1,
	DECON_ENH_RCD    = 1 << 2,
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

enum decon_src_cwb_path {
	SRC_ENHANCER_OUT = 0,
	SRC_ENHANCER_IN
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

enum decon_set_trig {
	DECON_TRIG_MASK = 0,
	DECON_TRIG_UNMASK
};

enum decon_te_from {
	DECON_TE_FROM_DDI0 = 0,
	DECON_TE_FROM_DDI1 = 1,
	DECON_TE_FROM_DDI2 = 2,
	MAX_DECON_TE_FROM_DDI = 3,
};

struct decon_mode {
	enum decon_op_mode op_mode;
	enum decon_dsi_mode dsi_mode;
	enum decon_trig_mode trig_mode;
};

struct decon_urgent {
	u32 rd_en;
	u32 rd_hi_thres;
	u32 rd_lo_thres;
	u32 rd_wait_cycle;
	u32 wr_en;
	u32 wr_hi_thres;
	u32 wr_lo_thres;
	bool dta_en;
	u32 dta_hi_thres;
	u32 dta_lo_thres;
};

struct decon_config {
	enum decon_out_type	out_type;
	enum decon_enh_path	enh_path;
	enum decon_te_from	te_from;
	unsigned int		image_height;
	unsigned int		image_width;
	struct decon_mode	mode;
	struct decon_urgent	urgent;
	struct exynos_dsc	dsc;
	unsigned int		out_bpc;
	unsigned int		in_bpc;
	int			main_dsim_id;
};

struct decon_regs {
	void __iomem *regs;
	void __iomem *win_regs;
	void __iomem *sub_regs;
	void __iomem *wincon_regs;
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
	u32 blend;
	u32 in_bpc;
};

struct decon_dsc {
	const struct drm_dsc_config *cfg;
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

#if defined(CONFIG_SOC_GS201) || defined(CONFIG_SOC_ZUMA)
void decon_reg_set_rcd_enable_internal(u32 id, bool en);
#else
static inline void decon_reg_set_rcd_enable_internal(u32 id, bool en) {}
void decon_video_mode_reg_update_req_internal(u32 id, bool dqe_need_update);
#endif

#if defined(CONFIG_SOC_GS201)
void decon_reg_update_req_cgc_internal(u32 id);
#else
static inline void decon_reg_update_req_cgc_internal(u32 id) {}
#endif

void decon_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		enum decon_regs_type type, unsigned int id);

/*************** DECON CAL APIs exposed to DECON driver ***************/
/* DECON control */
int decon_reg_init(u32 id, struct decon_config *config);
void decon_reg_direct_on_off(u32 id, u32 en);
int decon_reg_start(u32 id, struct decon_config *config);
int decon_reg_stop(u32 id, struct decon_config *config, bool rst, u32 fps);
void decon_reg_set_bpc_and_dither_path(u32 id, struct decon_config *config);

/* DECON window control */
void decon_reg_set_win_enable(u32 id, u32 win_idx, u32 en);
int decon_reg_get_win_ch(u32 id, u32 win_idx, u32 *ch);
void decon_reg_win_enable_and_update(u32 id, u32 win_idx, u32 en);
void decon_reg_all_win_shadow_update_req(u32 id);
void decon_reg_set_window_control(u32 id, int win_idx,
		struct decon_window_regs *regs, u32 winmap_en);
void decon_reg_update_req_window_mask(u32 id, u32 win_idx);
void decon_reg_update_req_window(u32 id, u32 win_idx);

/* DECON shadow update and trigger control */
void decon_reg_set_trigger(u32 id, struct decon_mode *mode,
		enum decon_set_trig trig);
void decon_reg_update_req_and_unmask(u32 id, struct decon_mode *mode);
int decon_reg_wait_update_done_timeout(u32 id, unsigned long timeout_us);
int decon_reg_wait_update_done_and_mask(u32 id, struct decon_mode *mode,
		u32 timeout_us);

bool decon_reg_is_idle(u32 id);
/* For window update and multi resolution feature */
int decon_reg_wait_idle_status_timeout(u32 id, unsigned long timeout);
void decon_reg_set_partial_update(u32 id, struct decon_config *config,
		bool in_slice[], u32 partial_w, u32 partial_h);
void decon_reg_set_mres(u32 id, struct decon_config *config);

/* For writeback configuration */
void decon_reg_release_resource(u32 id, struct decon_mode *mode);
void decon_reg_config_wb_size(u32 id, struct decon_config *config);
void decon_reg_set_cwb_enable(u32 id, bool en);

/* DECON interrupt control */
void decon_reg_set_interrupts(u32 id, u32 en);
int decon_reg_get_interrupt_and_clear(u32 id, u32 *ext_irq);
int decon_reg_get_fs_interrupt_and_clear(u32 id);

/* DECON SFR dump */
void __decon_dump(struct drm_printer *p, u32 id, struct decon_regs *regs,
		bool dsc_en, bool dqe_en);

void decon_reg_set_start_crc(u32 id, u32 en);
void decon_reg_get_crc_data(u32 id, u32 crc_data[3]);

/* For DQE configuration */
void decon_reg_set_dqe_enable(u32 id, bool en);
void decon_reg_update_req_dqe(u32 id);
void decon_reg_update_req_cgc(u32 id);

/* DPU hw limitation check */
struct decon_device;

/* TODO: this will be removed later */
void decon_reg_update_req_global(u32 id);

/* PLL sleep related functions */
void decon_reg_set_pll_sleep(u32 id, u32 en);
void decon_reg_set_pll_wakeup(u32 id, u32 en);

u64 decon_reg_get_rsc_ch(u32 id);
u64 decon_reg_get_rsc_win(u32 id);

static inline u32 PLANE2DPPCH(u32 plane_index) { return plane_index; }
static inline u32 DPPCH2PLANE(u32 ch) { return ch; }
#ifdef __linux__
struct decon_device;
int __decon_init_resources(struct decon_device *decon);
void __decon_unmap_regs(struct decon_device *decon);
#endif
bool is_decon_using_ch(u32 id, u64 rsc_ch, u32 ch);
bool is_decon_using_win(u32 id, u64 rsc_win, u32 win);

#if defined(CONFIG_SOC_GS201) || defined(CONFIG_SOC_ZUMA)
void decon_video_mode_reg_update_req(u32 id, bool cgc_need_update, bool dqe_need_update);
#else
static inline void decon_video_mode_reg_update_req(u32 id,
	__maybe_unused bool cgc_need_update, bool dqe_need_update)
{
	decon_video_mode_reg_update_req_internal(id, dqe_need_update);
}
#endif

static inline void decon_reg_set_rcd_enable(u32 dqe_id, bool en)
{
	decon_reg_set_rcd_enable_internal(dqe_id, en);
}

void decon_reg_set_drm_write_protected(u32 id, bool protected);
/*********************************************************************/

#endif /* __SAMSUNG_DECON_CAL_H__ */
