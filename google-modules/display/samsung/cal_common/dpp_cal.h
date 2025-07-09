/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS101 DPP CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DPP_CAL_H__
#define __SAMSUNG_DPP_CAL_H__

#include <drm/samsung_drm.h>
#include <cal_config.h>

#include "../exynos_drm_format.h"

#if defined(CONFIG_SOC_ZUMA)
/* 14xRDMA + 2xODMA + 2xRCD + 2xCGC*/
#define MAX_DPP_CNT		20
#elif defined(CONFIG_SOC_GS201)
/* 6xRDMA + 1xODMA + 2xRCD */
#define MAX_DPP_CNT		9
#else
/* 6xRDMA + 1xODMA */
#define MAX_DPP_CNT		7
#endif

#define DPP_CSC_IDX_BT601_625			0
#define DPP_CSC_IDX_BT601_625_UNADJUSTED	2
#define DPP_CSC_IDX_BT601_525			4
#define DPP_CSC_IDX_BT601_525_UNADJUSTED	6
#define DPP_CSC_IDX_BT2020_CONSTANT_LUMINANCE	8
#define DPP_CSC_IDX_BT470M			10
#define DPP_CSC_IDX_FILM			12
#define DPP_CSC_IDX_ADOBE_RGB			14
/* for R-to-Y conversion */
#define DPP_CSC_IDX_BT709			16
#define DPP_CSC_IDX_BT2020			18
#define DPP_CSC_IDX_DCI_P3			20

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
	DPP_ATTR_WCG            = 9,
	DPP_ATTR_SBWC           = 10,
	DPP_ATTR_HDR10_PLUS	= 11,

	DPP_ATTR_IDMA		= 16,
	DPP_ATTR_ODMA		= 17,
	DPP_ATTR_DPP		= 18,
	DPP_ATTR_SRAMC		= 19,
	DPP_ATTR_RCD		= 20,
	DPP_ATTR_SCL_COEF	= 21,
	DPP_ATTR_HDR_COMM	= 22,
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
	CSC_CUSTOMIZED_START = 4,
};

enum dpp_comp_type {
	COMP_TYPE_NONE = 0,
	COMP_TYPE_AFBC,
	COMP_TYPE_SBWC,
};

enum dpp_afbc_blk_size {
	AFBC_BLK_16x16 = 0,
	AFBC_BLK_32x8,
	AFBC_BLK_64x4,
};

enum dpp_sbwc_blk_size {
	SBWC_BLK_32x2 = 2,
	SBWC_BLK_32x3 = 3,
	SBWC_BLK_32x4 = 4,
	SBWC_BLK_32x5 = 5,
	SBWC_BLK_32x6 = 6,
};

struct dpp_regs {
	void __iomem *dpp_base_regs;
	void __iomem *dma_base_regs;
	void __iomem *scl_coef_base_regs;
	void __iomem *sramc_base_regs;
	void __iomem *hdr_base_regs;
	void __iomem *hdr_comm_base_regs;
	void __iomem *rcd_base_regs;
};

#ifdef CONFIG_SOC_ZUMA
enum dpp_regs_id {
	REGS_DPP0_ID = 0,
	REGS_DPP1_ID,
	REGS_DPP2_ID,
	REGS_DPP3_ID,
	REGS_DPP4_ID,
	REGS_DPP5_ID,
	REGS_DPP6_ID,
	REGS_DPP7_ID,
	REGS_DPP8_ID,
	REGS_DPP9_ID,
	REGS_DPP10_ID,
	REGS_DPP11_ID,
	REGS_DPP12_ID,
	REGS_DPP13_ID,
	REGS_RCD0_ID,
	REGS_RCD1_ID,
	REGS_WB0_ID,
	REGS_WB1_ID,
	REGS_CGC0_ID,
	REGS_CGC1_ID,
	REGS_DPP_ID_MAX
};
#else
enum dpp_regs_id {
	REGS_DPP0_ID = 0,
	REGS_DPP1_ID,
	REGS_DPP2_ID,
	REGS_DPP3_ID,
	REGS_DPP4_ID,
	REGS_DPP5_ID,
	REGS_DPP8_ID,
	REGS_DPP9_ID,
	REGS_DPP12_ID,
	REGS_CGC0_ID,
	REGS_CGC1_ID,
	REGS_DPP_ID_MAX
};
#endif

enum dpp_regs_type {
	REGS_DMA = 0,
	REGS_DPP,
	REGS_SRAMC,
	REGS_SCL_COEF,
	REGS_HDR_COMM,
	REGS_DPP_TYPE_MAX
};

extern struct cal_regs_desc regs_dpp[REGS_DPP_TYPE_MAX][REGS_DPP_ID_MAX];

#define dpp_regs_desc(id)			(&regs_dpp[REGS_DPP][id])
#define dpp_read(id, offset)			\
	cal_read(dpp_regs_desc(id), offset)
#define dpp_write(id, offset, val)		\
	cal_write(dpp_regs_desc(id), offset, val)
#define dpp_read_mask(id, offset, mask)	\
	cal_read_mask(dpp_regs_desc(id), offset, mask)
#define dpp_write_mask(id, offset, val, mask)	\
	cal_write_mask(dpp_regs_desc(id), offset, val, mask)

#define dma_regs_desc(id)			(&regs_dpp[REGS_DMA][id])
#define dma_read(id, offset)			\
	cal_read(dma_regs_desc(id), offset)
#define dma_write(id, offset, val)		\
	cal_write(dma_regs_desc(id), offset, val)
#define dma_read_mask(id, offset, mask)	\
	cal_read_mask(dma_regs_desc(id), offset, mask)
#define dma_write_mask(id, offset, val, mask)	\
	cal_write_mask(dma_regs_desc(id), offset, val, mask)

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

enum dpp_bpc {
	DPP_BPC_8 = 0,
	DPP_BPC_10,
};

#define MAX_PLANE_ADDR_CNT	4

struct dpp_params_info {
	struct decon_frame src;
	struct decon_frame dst;
	struct decon_win_rect block;
	u32 rot;

	enum dpp_hdr_standard hdr;
	u32 min_luminance;
	u32 max_luminance;
	u32 y_hd_y2_stride; /* Luminance header (or Y-2B) stride */
	u32 y_pl_c2_stride; /* Luminance payload (or C-2B) stride */
	u32 c_hd_stride;    /* Chrominance header stride */
	u32 c_pl_stride;    /* Chrominance payload stride */

	bool is_scale;
	bool is_block;
	u32 format;
	dma_addr_t addr[MAX_PLANE_ADDR_CNT];
	u32 dataspace;
	int h_ratio;
	int v_ratio;
	u32 standard;
	u32 transfer;
	u32 range;
	enum dpp_bpc in_bpc;

	unsigned long rcv_num;
	enum dpp_comp_type comp_type;
	u32 blk_size;
	bool is_lossy;
};

#if defined(CONFIG_SOC_GS201) || defined(CONFIG_SOC_ZUMA)
void rcd_reg_init(u32 id);
int rcd_reg_deinit(u32 id, bool reset, const unsigned long attr);
void rcd_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr);
#else
static inline void rcd_reg_init(u32 id) {return;}
static inline int rcd_reg_deinit(u32 id, bool reset, const unsigned long attr) {return 0;}
static inline void rcd_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr) {return;}
#endif

#if defined(CONFIG_SOC_GS201) || defined(CONFIG_SOC_ZUMA)
u32 cgc_reg_get_irq_and_clear_internal(u32 id);
void cgc_reg_set_cgc_start_internal(u32 id);
void cgc_reg_set_config_internal(u32 id, bool en, dma_addr_t addr);
#else
static inline u32 cgc_reg_get_irq_and_clear_internal(u32 id) {return 0;}
static inline void cgc_reg_set_cgc_start_internal(u32 id) {return;}
static inline void cgc_reg_set_config_internal(u32 id, bool en, dma_addr_t addr) {return;}
#endif

void dpp_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		enum dpp_regs_type type, unsigned int id);

/* DPP CAL APIs exposed to DPP driver */
void dpp_reg_init(u32 id, const unsigned long attr);
int dpp_reg_deinit(u32 id, bool reset, const unsigned long attr);
void dpp_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr);

void dma_reg_dump_com_debug_regs(struct drm_printer *p, int id);
void rcd_dma_dump_regs(struct drm_printer *p, u32 id, void __iomem *dma_regs);
void cgc_dma_dump_regs(struct drm_printer *p, u32 id, void __iomem *dma_regs);

/* DPU_DMA, DPP DEBUG */
void __dpp_dump(struct drm_printer *p, u32 id, void __iomem *regs, void __iomem *dma_regs,
		void __iomem *sramc_regs, void __iomem *hdr_comm_regs, void __iomem *hdr_regs,
		unsigned long attr);

static inline void
 __rcd_dump(struct drm_printer *p, u32 id, void __iomem *regs,
	    void __iomem *dma_regs, unsigned long attr)
{
	dma_reg_dump_com_debug_regs(p, id);

	rcd_dma_dump_regs(p, id, dma_regs);
}

static inline void
__cgc_dump(struct drm_printer *p, u32 id, void __iomem *dma_regs)
{
	cgc_dma_dump_regs(p, id, dma_regs);
}

/* DPP hw limitation check */
int __dpp_check(u32 id, const struct dpp_params_info *p, unsigned long attr);

/* DPU_DMA and DPP interrupt handler */
u32 dpp_reg_get_irq_and_clear(u32 id);
u32 idma_reg_get_irq_and_clear(u32 id);
u32 odma_reg_get_irq_and_clear(u32 id);

static inline u32 cgc_reg_get_irq_and_clear(u32 id)
{
	return cgc_reg_get_irq_and_clear_internal(id);
}
static inline void cgc_reg_set_config(u32 id, bool en, dma_addr_t addr)
{
	cgc_reg_set_config_internal(id, en, addr);
}
static inline void cgc_reg_set_cgc_start(u32 id)
{
	cgc_reg_set_cgc_start_internal(id);
}

void dma_reg_get_shd_addr(u32 id, u32 shd_addr[], const unsigned long attr);
bool dma_reg_is_mst_security_enabled(u32 id, u32 *out_mst_security);

#ifdef __linux__
struct dpp_device;
static inline int __dpp_init_resources(struct dpp_device *dpp) { return 0; }
#endif

#endif /* __SAMSUNG_DPP_CAL_H__ */
