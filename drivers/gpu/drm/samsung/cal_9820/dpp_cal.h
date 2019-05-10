/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS9_CAL_DPP_H__
#define __EXYNOS9_CAL_DPP_H__

#if defined(__linux__)
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/io.h>
#endif

#include "../exynos_drm_format.h"

#define MAX_DPP_CNT		6

enum dpp_attr {
	DPP_ATTR_AFBC		= 0,
	DPP_ATTR_BLOCK		= 1,
	DPP_ATTR_FLIP		= 2,
	DPP_ATTR_ROT		= 3,
	DPP_ATTR_CSC		= 4,
	DPP_ATTR_SCALE		= 5,
	DPP_ATTR_HDR		= 6,
	DPP_ATTR_C_HDR		= 7,
	DPP_ATTR_C_HDR10_PLUS	= 8,

	DPP_ATTR_IDMA		= 16,
	DPP_ATTR_ODMA		= 17,
	DPP_ATTR_DPP		= 18,
	DPP_ATTR_WBMUX		= 19,
};

enum dpp_csc_defs {
	/* csc_type */
	DPP_CSC_BT_601 = 0,
	DPP_CSC_BT_709 = 1,
	/* csc_range */
	DPP_CSC_NARROW = 0,
	DPP_CSC_WIDE = 1,
	/* csc_mode */
	CSC_COEF_HARDWIRED = 0,
	CSC_COEF_CUSTOMIZED = 1,
	/* csc_id used in csc_3x3_t[] : increase by even value */
	DPP_CSC_ID_BT_2020 = 0,
	DPP_CSC_ID_DCI_P3 = 2,
};

/* The array index of the variables must match the "dpp,id" defined DT */
static const char * const dpp_name[] = {
	"DPU_GF0",
	"DPU_VGRFS",
	"DPU_GF1",
	"DPU_VGF",
	"DPU_VG",
	"DPU_VGS",
};

struct dpp_regs {
	void __iomem *dpp_base_regs;
	void __iomem *dma_base_regs;
	void __iomem *dma_common_base_regs;
};

enum dpp_regs_id {
	REGS_DPP0_ID = 0,
	REGS_DPP1_ID,
	REGS_DPP2_ID,
	REGS_DPP3_ID,
	REGS_DPP4_ID,
	REGS_DPP5_ID,
	REGS_DPP_ID_MAX
};

enum dpp_regs_type {
	REGS_DMA = 0,
	REGS_DPP,
	REGS_DMA_COMMON,
	REGS_DPP_TYPE_MAX
};

/*
 *                      src_width
 *    -----------------------------------------------
 *    | src_offset_x,                               |
 *    | src_offset_y    img_width                   |
 *    |      0-------------------------------       |
 *    |      |//blk_offset_x,///////////////|       |
 *    |      |//blk_offset_y////////////////|       |
 *    |      |////0-------------//blk///////| img   |  src
 *    |      |////|            |/height/////|height | height
 *    |      |////--------------////////////|       |
 *    |      |///////blk_width//////////////|       |
 *    |      --------------------------------       |
 *    |                                             |
 *    -----------------------------------------------
 *
 *                              ///// accessed region
 */
struct dpp_region {
	unsigned int src_width, src_height;
	unsigned int src_offset_x, src_offset_y;

	unsigned int img_width, img_height;

	unsigned int blk_width, blk_height;
	unsigned int blk_offset_x, blk_offset_y;
};

struct dpp_config {
	struct dpp_region region;
	enum dpu_pixel_format format;
	u32 idma_addr[4];

	bool is_afbc;
	bool is_scale;
	bool is_block;
	bool is_4p;
	u32 y_2b_strd;
	u32 c_2b_strd;
};

struct decon_frame {
	int x;
	int y;
	u32 w;
	u32 h;
	u32 f_w;
	u32 f_h;
};

struct decon_win_rect {
	int x;
	int y;
	u32 w;
	u32 h;
};

enum dpp_hdr_standard {
	DPP_HDR_OFF = 0,
	DPP_HDR_ST2084,
	DPP_HDR_HLG,
};

#define DPP_X_FLIP		(1 << 0)
#define DPP_Y_FLIP		(1 << 1)
#define DPP_ROT			(1 << 2)
#define DPP_ROT_FLIP_MASK	(DPP_X_FLIP | DPP_Y_FLIP | DPP_ROT)
enum dpp_rotate {
	DPP_ROT_NORMAL	 = 0x0,
	DPP_ROT_XFLIP	 = DPP_X_FLIP,
	DPP_ROT_YFLIP	 = DPP_Y_FLIP,
	DPP_ROT_180	 = DPP_X_FLIP | DPP_Y_FLIP,
	DPP_ROT_90	 = DPP_ROT,
	DPP_ROT_90_XFLIP = DPP_ROT | DPP_X_FLIP,
	DPP_ROT_90_YFLIP = DPP_ROT | DPP_Y_FLIP,
	DPP_ROT_270	 = DPP_ROT | DPP_X_FLIP | DPP_Y_FLIP,
};

enum dpp_reg_area {
	REG_AREA_DPP = 0,
	REG_AREA_DMA,
	REG_AREA_DMA_COM,
};

#define MAX_PLANE_ADDR_CNT	4

#define CSC_STANDARD_MASK	0x3F	/* 6 bits */
#define CSC_RANGE_MASK		0x7	/* 3 bits */
enum dpp_csc_eq {
	/* eq_mode : 6bits [5:0] */
	CSC_STANDARD_SHIFT = 0,
	CSC_BT_601 = 0,
	CSC_BT_709 = 1,
	CSC_BT_2020 = 2,
	CSC_DCI_P3 = 3,
	CSC_STANDARD_UNSPECIFIED = 63,
	/* eq_mode : 3bits [8:6] */
	CSC_RANGE_SHIFT = 6,
	CSC_RANGE_LIMITED = 0x0,
	CSC_RANGE_FULL = 0x1,
	CSC_RANGE_UNSPECIFIED = 7,
};

struct dpp_params_info {
	struct decon_frame src;
	struct decon_frame dst;
	struct decon_win_rect block;
	u32 rot;

	enum dpp_hdr_standard hdr;
	u32 min_luminance;
	u32 max_luminance;
	bool is_4p;
	u32 y_2b_strd;
	u32 c_2b_strd;

	bool is_comp;
	bool is_scale;
	bool is_block;
	enum dpu_pixel_format format;
	dma_addr_t addr[MAX_PLANE_ADDR_CNT];
	enum dpp_csc_eq eq_mode;
	int h_ratio;
	int v_ratio;

	unsigned long rcv_num;
};

void dpp_regs_desc_init(void __iomem *regs, const char *name,
		enum dpp_regs_type type, unsigned int id);

void dpp_reg_init(u32 id, const unsigned long attr);
int dpp_reg_deinit(u32 id, bool reset, const unsigned long attr);
void dpp_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr);
void __dpp_dump(u32 id, void __iomem *regs, void __iomem *dma_regs,
		unsigned long attr);

/* DPU_DMA and DPP interrupt handler */
u32 dpp_reg_get_irq_and_clear(u32 id);
u32 idma_reg_get_irq_and_clear(u32 id);
u32 odma_reg_get_irq_and_clear(u32 id);

#endif
