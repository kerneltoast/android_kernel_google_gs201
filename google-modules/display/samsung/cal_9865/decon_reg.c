// SPDX-License-Identifier: GPL-2.0-only
/*
 * cal_9865/decon_reg.c
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register access functions for Samsung EXYNOS DECON driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <cal_config.h>
#include <decon_cal.h>
#include <dqe_cal.h>
#ifdef __linux__
#include "../exynos_drm_decon.h"
#include <linux/of_address.h>
#endif

#include "regs-decon.h"
#include "regs-dqe.h"

enum decon_dsc_id {
	DECON_DSC_ENC0 = 0x0,
	DECON_DSC_ENC1 = 0x1,
	DECON_DSC_ENC2 = 0x2,
};

enum decon_fifo {
	DECON0_OFIFO0 = 0x0,
	DECON0_OFIFO1 = 0x1,
	DECON1_OFIFO0 = 0x2,
	DECON_OFIFO_RSVD = 0x3,
	DECON2_OFIFO0 = 0x4,
	DECON_OFIFO_NONE = 0xF,
};

enum decon_win_alpha_coef {
	BND_COEF_ZERO		  = 0x0,
	BND_COEF_ONE		  = 0x1,
	BND_COEF_AF		  = 0x2,
	BND_COEF_1_M_AF		  = 0x3,
	BND_COEF_AB		  = 0x4,
	BND_COEF_1_M_AB		  = 0x5,
	BND_COEF_PLANE_ALPHA0	  = 0x6,
	BND_COEF_1_M_PLANE_ALPHA0 = 0x7,
	BND_COEF_PLANE_ALPHA1	  = 0x8,
	BND_COEF_1_M_PLANE_ALPHA1 = 0x9,
	BND_COEF_ALPHA_MULT	  = 0xA,
	BND_COEF_1_M_ALPHA_MULT	  = 0xB,
};

enum decon_win_alpha_sel {
	ALPHA_MULT_SRC_SEL_ALPHA0 = 0,
	ALPHA_MULT_SRC_SEL_ALPHA1 = 1,
	ALPHA_MULT_SRC_SEL_AF = 2,
	ALPHA_MULT_SRC_SEL_AB = 3,
};

struct cal_regs_desc regs_decon[REGS_DECON_TYPE_MAX][REGS_DECON_ID_MAX];

void decon_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		enum decon_regs_type type, unsigned int id)
{
	cal_regs_desc_check(type, id, REGS_DECON_TYPE_MAX, REGS_DECON_ID_MAX);
	cal_regs_desc_set(regs_decon, regs, start, name, type, id);
}

/******************* DECON CAL functions *************************/
static int decon_reg_reset(u32 id)
{
	u32 val;
	int ret;

	decon_write_mask(id, GLOBAL_CON, ~0, GLOBAL_CON_SRESET);
	ret = readl_poll_timeout_atomic(decon_regs_desc(id)->regs + GLOBAL_CON,
			val, !(val & GLOBAL_CON_SRESET), 10, 2000);
	if (ret)
		cal_log_err(id, "failed to reset decon%d\n", id);

	return ret;
}

/* select op mode */
static void decon_reg_set_operation_mode(u32 id, enum decon_op_mode mode)
{
	u32 val, mask;

	mask = GLOBAL_CON_OPERATION_MODE_F;
	if (mode == DECON_COMMAND_MODE)
		val = GLOBAL_CON_OPERATION_MODE_CMD_F;
	else
		val = GLOBAL_CON_OPERATION_MODE_VIDEO_F;
	decon_write_mask(id, GLOBAL_CON, val, mask);
}

void decon_reg_direct_on_off(u32 id, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	mask = (GLOBAL_CON_DECON_EN | GLOBAL_CON_DECON_EN_F);
	decon_write_mask(id, GLOBAL_CON, val, mask);
}

static void decon_reg_per_frame_off(u32 id)
{
	decon_write_mask(id, GLOBAL_CON, 0, GLOBAL_CON_DECON_EN_F);
}

static int decon_reg_wait_run_status_timeout(u32 id, unsigned long timeout_us)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(decon_regs_desc(id)->regs + GLOBAL_CON,
			val, (val & GLOBAL_CON_RUN_STATUS), 10, timeout_us);
	if (ret) {
		cal_log_err(id, "failed to change running status of DECON%d\n",
				id);
		return ret;
	}

	return 0;
}

/* Determine that DECON is perfectly shut off through checking this function */
static int decon_reg_wait_run_is_off_timeout(u32 id, unsigned long timeout_us)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(decon_regs_desc(id)->regs + GLOBAL_CON,
			val, !(val & GLOBAL_CON_RUN_STATUS), 10, timeout_us);
	if (ret) {
		cal_log_err(id, "failed to change off status of DECON%d\n", id);
		return ret;
	}

	return 0;
}

/* In bring-up, all bits are disabled */
static void decon_reg_set_clkgate_mode(u32 id, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	/* all unmask */
	mask = CLOCK_CON_AUTO_CG_MASK | CLOCK_CON_QACTIVE_MASK;
	decon_write_mask(id, CLOCK_CON(id), val, mask);
}

static void decon_reg_set_qactive_pll_mode(u32 id, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	/* all unmask */
	mask = CLOCK_CON_QACTIVE_PLL_ON;
	decon_write_mask(id, CLOCK_CON(id), val, mask);
}

/*
 * Sharing Resources : SRAM for OUTFIFO
 *
 * DECON0_PRI / DECON0_SEC / DECON1_PRI / DECON1_SEC / DECON2_PRI / DECON2_SEC
 * should be assigned SRAM exclusively.
 *
 * PRI(Primary)   : for Main Display path
 * SEC(Secondary) : for Concurrent WB path (no CWB path in DECON2)
 *
 * Mapping info for RSC_STATUS_7 ~ 11(SRAM 0 ~ 32).
 * Initial configurations for bring up
 */
static u64 pri_sram[MAX_DECON_CNT] = {
	GENMASK_ULL(10, 0),  /* decon0 : 11 SRAM instances */
	GENMASK_ULL(21, 11), /* decon1 : 11 SRAM instances */
	0,                   /* decon2 : dynamic allocate SRAM instances */
};

static u64 sec_sram[MAX_DECON_CNT] = {
	0, /* decon0 : dynamic allocate SRAM instances */
	0, /* decon1 : dynamic allocate SRAM instances */
	0, /* decon2 : none */
};

/* SRAM instance share to decon2 PRI and decon0/1 SEC */
const u64 shared_sram = GENMASK_ULL(32, 22);

static bool decon_reg_alloc_shared_sram_instance(u32 id, bool is_pri, bool en)
{
	/* allocate SRAM for decon2 PRI */
	if (id == 2 && is_pri) {
		if (en) {
			if (sec_sram[0] || sec_sram[1]) {
				cal_log_warn(id,
					"clear CWB SARAM instance (0x%llx, 0x%llx)\n",
					sec_sram[0], sec_sram[1]);
				sec_sram[0] = 0;
				sec_sram[1] = 0;
			}
			pri_sram[id] = shared_sram;
		} else {
			pri_sram[id] = 0;
		}
		return true;
	}

	/* allocate SRAM for decon0/1 CWB */
	if ((id == 0 || id == 1) && !is_pri) {
		if (en) {
			if (pri_sram[2]) {
				cal_log_err(id,
					"DECON2 is active, not able to use CWB on DECON%u\n", id);
				return false;
			}
			sec_sram[id] = shared_sram;
		} else {
			sec_sram[id] = 0;
		}
		return true;
	}

	return false;
}

static void decon_reg_set_sram_enable(u32 id)
{
	int i, n, j;
	u32 val1 = 0, val2 = 0;

	n = 0;
	for (j = 0; j < SRAM_EN_OF_PRI_REG_CNT; j++) {
		for (i = 0; i < SRAM_EN_CNT; i++) {
			n = 8 * j + i;
			if (pri_sram[id] & BIT(n))
				val1 |= SRAM_EN_ID(i % 8);
			if (sec_sram[id] & BIT(n))
				val2 |= SRAM_EN_ID(i % 8);
		}
		decon_write(id, SRAM_EN_OF_PRI(j), val1);
		decon_write(id, SRAM_EN_OF_SEC(j), val2);

		val1 = 0;
		val2 = 0;
	}

	/* sram32 */
	if (pri_sram[id] & BIT(32))
		decon_write(id, SRAM_EN_OF_PRI_4, SRAM32_EN_F);
	if (sec_sram[id] & BIT(32))
		decon_write(id, SRAM_EN_OF_SEC_4, SRAM32_EN_F);
}

static void decon_reg_set_outfifo_size_ctl0(u32 id, u32 width, u32 height)
{
	u32 val;
	u32 th, mask;

	/* OUTFIFO */
	val = OUTFIFO_HEIGHT_F(height) | OUTFIFO_WIDTH_F(width);
	decon_write(id, OF_SIZE_0, val);

	/* may be implemented later by considering 1/2H transfer */
	th = OUTFIFO_TH_1H_F; /* 1H transfer */
	mask = OUTFIFO_TH_MASK;
	decon_write_mask(id, OF_TH_TYPE, th, mask);
}

static void decon_reg_set_outfifo_size_ctl1(u32 id, u32 width)
{
	u32 val;

	/* only decon0 has outfifo_1 */
	if (WARN_ON(id != 0))
		return;

	val = OUTFIFO_1_WIDTH_F(width);
	decon_write(0, OF_SIZE_1, val);
}

static void decon_reg_set_outfifo_size_ctl2(u32 id, u32 width, u32 height)
{
	u32 val;

	val = OUTFIFO_COMPRESSED_SLICE_HEIGHT_F(height) |
			OUTFIFO_COMPRESSED_SLICE_WIDTH_F(width);

	decon_write(id, OF_SIZE_2, val);
}

static void decon_reg_set_rgb_order(u32 id, enum decon_rgb_order order)
{
	u32 val, mask;

	val = OUTFIFO_PIXEL_ORDER_SWAP_F(order);
	mask = OUTFIFO_PIXEL_ORDER_SWAP_MASK;
	decon_write_mask(id, OF_PIXEL_ORDER, val, mask);
}

#if defined(CONFIG_EXYNOS_LATENCY_MONITOR)
/* need to set once at init time */
static void decon_reg_set_latency_monitor_enable(u32 id, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	mask = LATENCY_COUNTER_ENABLE;
	decon_write_mask(id, OF_LAT_MON, val, mask);
}
#else
static inline void decon_reg_set_latency_monitor_enable(u32 id, u32 en) { }
#endif

/* Enable WR/RD urgent signal */
static void decon_reg_set_wr_urgent_enable(u32 id, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	mask = WRITE_URGENT_GENERATION_EN_F;
	decon_write_mask(id, OF_URGENT_EN, val, mask);
}

static void decon_reg_set_rd_urgent_enable(u32 id, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	mask = READ_URGENT_GENERATION_EN_F;
	decon_write_mask(id, OF_URGENT_EN, val, mask);
}

static void decon_reg_set_rd_urgent_threshold(u32 id, u32 high, u32 low)
{
	u32 val;

	val = READ_URGENT_HIGH_THRESHOLD_F(high) |
		READ_URGENT_LOW_THRESHOLD_F(low);
	decon_write(id, OF_RD_URGENT_0, val);
}

static void decon_reg_set_rd_wait_cycle(u32 id, u32 wait_cycle)
{
	u32 val;

	val = READ_URGENT_WAIT_CYCLE_F(wait_cycle);
	decon_write(id, OF_RD_URGENT_1, val);
}

static void decon_reg_set_wr_urgent_threshold(u32 id, u32 high, u32 low)
{
	u32 val;

	val = WRITE_URGENT_HIGH_THRESHOLD_F(high) |
		WRITE_URGENT_LOW_THRESHOLD_F(low);
	decon_write(id, OF_WR_URGENT_0, val);
}

/* Enable DTA control */
static void decon_reg_set_dta_enable(u32 id, bool en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	mask = DTA_EN_F;
	decon_write_mask(id, OF_DTA_CONTROL, val, mask);
}

static void decon_reg_set_dta_threshold(u32 id, u32 high, u32 low)
{
	u32 val;

	val = DTA_HIGH_TH_F(high) | DTA_LOW_TH_F(low);
	decon_write(id, OF_DTA_THRESHOLD, val);
}

#define OUTIF_DSI0	BIT(0)
#define OUTIF_DSI1	BIT(1)
#define OUTIF_WB	BIT(2)
#define OUTIF_DPIF	BIT(3)
/*
 * COMP_DSC(0) means
 *    1. DSC_ENC0 in DSC_TOP0 for DECON 0
 *    2. DSC_ENC1 in DSC_TOP0 for DECON 1
 *    3. DSC_ENC4 in DSC_TOP2 for DECON 2
 * COMP_DSC(1) means
 *    1. DSC_ENC1 in DSC_TOP0 for DECON 0
 *    2. DSC_ENC5 in DSC_TOP2 for DECON 2
 */
#define COMP_DSC(id)	BIT((id) + 4) /* COMP_DSC 0,1 */
#define COMP_DSCC	BIT(7)

static void decon_reg_log_dsimif(u32 id, u32 dsimif, enum decon_fifo sel)
{
	if (sel <= DECON2_OFIFO0 && sel != DECON_OFIFO_RSVD)
		cal_log_debug(id, "sel(0x%X) OFIFO.%u - dsim%u\n", sel, sel%2, dsimif);
	else
		cal_log_debug(id, "sel(0x%X) not connect to dsim%u\n", sel, dsimif);
}

static void decon_reg_dsimif_write(u32 id, u32 dsimif, enum decon_fifo sel)
{
	decon_reg_log_dsimif(id, dsimif, sel);
	dsimif_write(id, DSIMIF_SEL(dsimif), SEL_DSIM(sel));
}

static void decon_reg_clear_dsimif(u32 id, u32 dsimif)
{
	u32 val;

	val = SEL_DSIM_GET(dsimif_read(id, DSIMIF_SEL(dsimif)));
	if ((id == 0 && val < DECON1_OFIFO0) || (id == 1 && val == DECON1_OFIFO0)) {
		cal_log_info(id, "clearing dsimif%u sel, val: %u\n", dsimif, val);
		decon_reg_dsimif_write(id, dsimif, DECON_OFIFO_NONE);
	}
}

static inline bool is_dsim0_main_for_dual_dsi(struct decon_config *cfg)
{
	return !cfg->main_dsim_id;
}

static void decon_reg_set_data_path(u32 id, struct decon_config *cfg)
{
	enum decon_out_type out_type = cfg->out_type;
	u32 dsc_count = cfg->dsc.dsc_count;
	u32 val;

	if (id > 2) {
		cal_log_err(id, "invalid decon\n");
		return;
	}

	switch (out_type) {
	case DECON_OUT_DSI0:
		val = OUTIF_DSI0;
		decon_reg_clear_dsimif(id, 1);
		decon_reg_dsimif_write(id, 0, id == 1 ? DECON1_OFIFO0 : DECON0_OFIFO0);
		break;
	/*
	 * OUTIF_DSIx in DECON determines that blended data is transferred
	 * to which output device. It may be a little confusing, In single
	 * DSI mode, only OUTIF_DSI0 is used whether the output device is
	 * DSI0 or DSI1.
	 * OUTIF_DSI1 is only used in case of dual DSI mode.
	 */
	case DECON_OUT_DSI1:
		val = OUTIF_DSI0;
		decon_reg_clear_dsimif(id, 0);
		decon_reg_dsimif_write(id, 1, id == 1 ? DECON1_OFIFO0 : DECON0_OFIFO0);
		break;
	case DECON_OUT_DSI:
		val = OUTIF_DSI0 | OUTIF_DSI1;
		/*
		 * Refer to decon block diagram in decon spec Figure.1-1,
		 * for Dual DSI, DSIMIFx HW supports OF0_0 and OF0_1 as
		 * dual dsi paths.
		 */
		if (is_dsim0_main_for_dual_dsi(cfg)) {
			decon_reg_dsimif_write(id, 0, DECON0_OFIFO0);
			decon_reg_dsimif_write(id, 1, DECON0_OFIFO1);
		} else {
			decon_reg_dsimif_write(id, 0, DECON0_OFIFO1);
			decon_reg_dsimif_write(id, 1, DECON0_OFIFO0);
		}
		break;
	case DECON_OUT_DP0:
		val = OUTIF_DPIF;
		dpif_write(id, DPIF_SEL(0), SEL_DP(id));
		break;
	case DECON_OUT_DP1:
		val = OUTIF_DPIF;
		dpif_write(id, DPIF_SEL(1), SEL_DP(id));
		break;
	case DECON_OUT_WB:
		val = OUTIF_WB;
		cal_log_debug(id, "OUTIF is WB\n");
		break;
	default:
		val = OUTIF_DSI0;
		cal_log_warn(id, "default outif is set(DSI0)\n");
		break;
	}

	if (dsc_count == 2) {
		if (id == 1) {
			cal_log_err(id, "unsupported dsc path\n");
			return;
		}
		val |= COMP_DSCC | COMP_DSC(1) | COMP_DSC(0);
	} else if (dsc_count == 1) {
		val |= COMP_DSC(0);
	}

	cal_log_debug(id, "d_path(0x%02X)", val);
	decon_write_mask(id, DATA_PATH_CON_0, COMP_OUTIF_PATH_F(val),
			COMP_OUTIF_PATH_MASK);
}

void decon_reg_set_cwb_enable(u32 id, bool en)
{
	u32 val, mask, d_path;

	if (!decon_reg_alloc_shared_sram_instance(id, false, en)) {
		cal_log_err(id, "not able allocate SRAM inst for DECON%u CWB\n", id);
		return;
	}

	cal_log_info(id, "set cwb for DECON%u (en:%d)\n", id, en);
	val = decon_read(id, DATA_PATH_CON_0);
	d_path = COMP_OUTIF_PATH_GET(val);

	if (en)
		d_path |= CWB_PATH_EN;
	else
		d_path &= ~CWB_PATH_EN;

	mask = COMP_OUTIF_PATH_MASK;
	decon_write_mask(id, DATA_PATH_CON_0, d_path, mask);
	/* Select OUTFIFO2, WBIF0 for CWB path*/
	decon_write_mask(id, DATA_PATH_CON_1,
					CWB_OF_IDX_F(CWB_OF_IDX_OF2) |
					WB_SEL_IF_F(WB_SEL_IF_0),
					CWB_OF_IDX_MASK | WB_SEL_IF_MASK);
	decon_reg_set_sram_enable(id);
}

void decon_reg_set_dqe_enable(u32 id, bool en)
{
	u32 val, mask;

	val = en ? ENHANCE_DQE_ON : 0;
	mask = en ? ENHANCE_DQE_ON : ENHANCE_PATH_MASK;
	decon_write_mask(id, DATA_PATH_CON_0, val, mask);
}

/*
 * Check major configuration of data_path_control
 *    DSCC[7]
 *    DSC_ENC2[6] DSC_ENC1[5] DSC_ENC0[4]
 *    DPIF[3]
 *    WBIF[2]
 *    DSIMIF1[1] DSIMIF0[0]
 */
static u32 decon_reg_get_data_path_cfg(u32 id, enum decon_path_cfg con_id)
{
	u32 val;
	u32 d_path;
	u32 bRet = 0;

	val = decon_read(id, DATA_PATH_CON_0);
	d_path = COMP_OUTIF_PATH_GET(val);

	switch (con_id) {
	case PATH_CON_ID_DSCC_EN:
		if (d_path & (0x1 << PATH_CON_ID_DSCC_EN))
			bRet = 1;
		break;
	case PATH_CON_ID_DUAL_DSC:
		if ((d_path & (0x3 << PATH_CON_ID_DUAL_DSC)) == 0x30)
			bRet = 1;
		break;
	case PATH_CON_ID_DP:
		if (d_path & (0x1 << PATH_CON_ID_DP))
			bRet = 1;
		break;
	case PATH_CON_ID_WB:
		if (d_path & (0x1 << PATH_CON_ID_WB))
			bRet = 1;
		break;
	case PATH_CON_ID_DSIM_IF0:
		if (d_path & (0x1 << PATH_CON_ID_DSIM_IF0))
			bRet = 1;
		break;
	case PATH_CON_ID_DSIM_IF1:
		if (d_path & (0x1 << PATH_CON_ID_DSIM_IF1))
			bRet = 1;
		break;
	default:
		break;
	}

	return bRet;
}

/*
 * width : width of updated LCD region
 * height : height of updated LCD region
 * is_dsc : 1: DSC is enabled 0: DSC is disabled
 */
static void decon_reg_set_data_path_size(u32 id, u32 width, u32 height,
		bool is_dsc, u32 dsc_cnt, u32 slice_w, u32 slice_h,
		u32 ds_en[2], u32 bpc)
{
	u32 outfifo_w;
	u32 comp_slice_width; /* compressed slice width */

	comp_slice_width = decon_get_comp_dsc_width(slice_w, bpc);

	if (is_dsc)
		outfifo_w = ALIGN((comp_slice_width << ds_en[0]), 4);
	else
		outfifo_w = width;

	/* OUTFIFO size is compressed size if DSC is enabled */
	decon_reg_set_outfifo_size_ctl0(id, outfifo_w, height);
	if (dsc_cnt == 2)
		decon_reg_set_outfifo_size_ctl1(id, outfifo_w);
	if (is_dsc)
		decon_reg_set_outfifo_size_ctl2(id, ALIGN(comp_slice_width, 4),
				slice_h);
}

/*
 * 'DATA_PATH_CON' SFR must be set before calling this function!!
 * [width]
 * - no compression  : x-resolution
 * - dsc compression : width_per_enc
 */
static void decon_reg_config_data_path_size(u32 id, u32 width, u32 height, u32 overlap_w,
		struct decon_dsc *p, struct exynos_dsc *dsc, u32 bpc, enum decon_dsi_mode mode)
{
	u32 width_f;
	u32 comp_slice_width; /* compressed slice width */

	/* OUTFIFO */
	if (dsc->enabled) {
		width_f = p->width_per_enc;
		/* OUTFIFO_COMPRESSED_SLICE_WIDTH must be a multiple of 2 */
		comp_slice_width = decon_get_comp_dsc_width(dsc->slice_width, bpc);
		if (dsc->dsc_count == 1) {
			decon_reg_set_outfifo_size_ctl0(id, width_f, height);
			decon_reg_set_outfifo_size_ctl2(id,
					comp_slice_width, p->slice_height);
		} else if (dsc->dsc_count == 2) {
			decon_reg_set_outfifo_size_ctl0(id, width_f, height);
			decon_reg_set_outfifo_size_ctl1(id, width_f);
			decon_reg_set_outfifo_size_ctl2(id,
					comp_slice_width, p->slice_height);
		}
	} else {
		if (mode == DSI_MODE_DUAL_DSI) {
			decon_reg_set_outfifo_size_ctl0(id, width / 2, height);
			decon_reg_set_outfifo_size_ctl1(id, width / 2);
		} else {
			decon_reg_set_outfifo_size_ctl0(id, width, height);
		}
	}
}

static void decon_reg_set_bpc(u32 id, u32 bpc)
{
	u32 val = 0, mask;

	if (bpc == 10)
		val = GLOBAL_CON_TEN_BPC_MODE_F;

	mask = GLOBAL_CON_TEN_BPC_MODE_MASK;

	decon_write_mask(id, GLOBAL_CON, val, mask);

	cal_log_debug(id, "%d bpc mode is set\n", decon_read_mask(id,
			GLOBAL_CON, GLOBAL_CON_TEN_BPC_MODE_MASK) ? 10 : 8);
}

static void decon_reg_config_win_channel(u32 id, u32 win_idx, int ch)
{
	u32 val, mask;

	/* L7 layer is not present in Zuma */
	if (ch > 6)
		ch = ch + 1;

	val = WIN_CHMAP_F(ch);
	mask = WIN_CHMAP_MASK;
	wincon_write_mask(id, DECON_CON_WIN(win_idx), val, mask);
}

static void decon_reg_init_trigger(u32 id, struct decon_config *cfg)
{
	u32 val, mask;
	enum decon_trig_mode mode = cfg->mode.trig_mode;

	mask = HW_TRIG_EN | HW_TRIG_SEL_MASK | HW_TRIG_MASK_DECON;
	val = (mode == DECON_SW_TRIG) ? 0 : HW_TRIG_EN;

	if (cfg->te_from == DECON_TE_FROM_DDI2)
		val |= HW_TRIG_SEL_FROM_DDI2;
	else if (cfg->te_from == DECON_TE_FROM_DDI1)
		val |= HW_TRIG_SEL_FROM_DDI1;
	else if (cfg->te_from == DECON_TE_FROM_DDI0)
		val |= HW_TRIG_SEL_FROM_DDI0;
	else
		val |= HW_TRIG_SEL_FROM_NONE;

	/* The trigger is masked initially */
	val |= HW_TRIG_MASK_DECON;

	decon_write_mask(id, TRIG_CON, val, mask);
}

/*
 * wakeup_us : usec unit
 * cnt : TE rising ~ expire
 * (example)
 *    if 60fps, TE period = 16666us(=1/fps) & wakeup_us = 100
 *    cnt = (16666 - 100) time = 16566us
 *    <meaning> wakeup at 16.566ms after TE rising
 */
#if defined(CONFIG_EXYNOS_EWR)
static u32 decon_get_ewr_cycle(int fps, int wakeup_us)
{
	u32 cnt;

	cnt = ((1000000 / fps) - wakeup_us) * 26;

	return cnt;
}

static void decon_reg_set_ewr_enable(u32 id, u32 en)
{
	u32 val, mask;

	mask = EWR_EN_F;
	val = en ? ~0 : 0;
	decon_write_mask(id, EWR_CON, val, mask);
}

static void decon_reg_set_ewr_timer(u32 id, u32 cnt)
{
	decon_write(id, EWR_TIMER, cnt);
}

static void decon_reg_set_ewr_control(u32 id, u32 cnt, u32 en)
{
	decon_reg_set_ewr_timer(id, cnt);
	decon_reg_set_ewr_enable(id, en);
}
#endif

static void decon_reg_update_req_compress(u32 id)
{
	decon_write_mask(id, SHD_REG_UP_REQ, ~0, SHD_REG_UP_REQ_CMP);
}

static void dsc_reg_swreset(u32 id, u32 dsc_id)
{
	dsc_write_mask(id, DSC_CONTROL1(dsc_id), 1, DSC_SW_RESET);
}

static void dsc_reg_set_slice_mode_change(u32 id, u32 dsc_id, u32 en)
{
	u32 val;

	val = DSC_SLICE_MODE_CH_F(en);
	dsc_write_mask(id, DSC_CONTROL1(dsc_id), val, DSC_SLICE_MODE_CH_MASK);
}

static void dsc_reg_set_dual_slice(u32 id, u32 dsc_id, u32 en)
{
	u32 val;

	val = DSC_DUAL_SLICE_EN_F(en);
	dsc_write_mask(id, DSC_CONTROL1(dsc_id), val, DSC_DUAL_SLICE_EN_MASK);
}

/*
 * dsc PPS Configuration
 */

/*
 * APIs which user setting or calculation is required are implemented
 * - PPS04 ~ PPS35 except reserved
 * - PPS58 ~ PPS59
 */
static void dsc_reg_set_pps_06_07_picture_height(u32 id, u32 dsc_id, u32 height)
{
	u32 val, mask;

	val = PPS06_07_PIC_HEIGHT(height);
	mask = PPS06_07_PIC_HEIGHT_MASK;
	dsc_write_mask(id, DSC_PPS04_07(dsc_id), val, mask);
}

static void dsc_reg_set_pps_58_59_rc_range_param0(u32 id, u32 dsc_id, u32 rc_range)
{
	u32 val, mask;

	val = PPS58_59_RC_RANGE_PARAM(rc_range);
	mask = PPS58_59_RC_RANGE_PARAM_MASK;
	dsc_write_mask(id, DSC_PPS56_59(dsc_id), val, mask);
}

static void dsc_reg_set_pps_44_57_rc_buf_thresh(u32 id, u32 dsc_id,
		const struct drm_dsc_config *cfg)
{
	u32 i, val, mask, offset = 0;

	for (i = 0; i < 12; i += 4) {
		val = cfg->rc_buf_thresh[i] << 24;
		val |= cfg->rc_buf_thresh[i + 1] << 16;
		val |= cfg->rc_buf_thresh[i + 2] << 8;
		val |= cfg->rc_buf_thresh[i + 3];

		if (!val)
			continue;
		offset += i;
		dsc_write(id, DSC_PPS44_47(dsc_id) + offset, val);
	}

	if (cfg->rc_buf_thresh[12] || cfg->rc_buf_thresh[13]) {
		val = PPS56_RC_BUF_THRESH_C(cfg->rc_buf_thresh[12]);
		val |= PPS57_RC_BUF_THRESH_D(cfg->rc_buf_thresh[13]);
		mask = PPS56_RC_BUF_THRESH_C_MASK | PPS57_RC_BUF_THRESH_D_MASK;
		dsc_write_mask(id, DSC_PPS56_59(dsc_id), val, mask);
	}
}

static u16 dsc_reg_get_rc_range_parameter(const struct drm_dsc_rc_range_parameters *rc)
{
	return (rc->range_min_qp <<	DSC_PPS_RC_RANGE_MINQP_SHIFT) |
		(rc->range_max_qp << DSC_PPS_RC_RANGE_MAXQP_SHIFT) |
		rc->range_bpg_offset;
}

static void dsc_reg_set_pps_58_87_rc_range_params(u32 id, u32 dsc_id,
		const struct drm_dsc_config *cfg)
{
	u32 i, offset, val,  mask;

	val = dsc_reg_get_rc_range_parameter(&cfg->rc_range_params[0]);
	if (val) {
		val = PPS58_59_RC_RANGE_PARAM(val);
		mask = PPS58_59_RC_RANGE_PARAM_MASK;
		dsc_write_mask(id, DSC_PPS56_59(dsc_id), val, mask);
	}

	for (i = 1; i < ARRAY_SIZE(cfg->rc_range_params); i++) {
		val = dsc_reg_get_rc_range_parameter(&cfg->rc_range_params[i]);

		if (!val)
			continue;

		val = i % 2 ? val << 16 : val;
		mask = i % 2 ? 0xFFFF0000 : 0x0000FFFF;
		offset = (i - 1) / 2;
		offset *= 4;
		dsc_write_mask(id, DSC_PPS60_63(dsc_id) + offset, val, mask);
	}
}

/* full size default value */
static u32 dsc_get_dual_slice_mode(struct exynos_dsc *dsc)
{
	u32 dual_slice_en = 0;

	if (dsc->dsc_count == 1) {
		if (dsc->slice_count == 2)
			dual_slice_en = 1;
	} else if (dsc->dsc_count == 2) {
		if (dsc->slice_count == 4)
			dual_slice_en = 1;
	} else {
		dual_slice_en = 0;
	}

	return dual_slice_en;
}

/* full size default value */
static u32 dsc_get_slice_mode_change(struct exynos_dsc *dsc)
{
	u32 slice_mode_ch = 0;

	if ((dsc->dsc_count == 2) && (dsc->slice_count == 2))
		slice_mode_ch = 1;

	return slice_mode_ch;
}

static void dsc_get_partial_update_info(u32 id, u32 slice_cnt, u32 dsc_cnt,
		bool in_slice[4], u32 ds_en[2], u32 sm_ch[2])
{
	switch (slice_cnt) {
	case 4:
		if ((in_slice[0] + in_slice[1]) % 2) {
			ds_en[DECON_DSC_ENC0] = 0;
			sm_ch[DECON_DSC_ENC0] = 1;
		} else {
			ds_en[DECON_DSC_ENC0] = 1;
			sm_ch[DECON_DSC_ENC0] = 0;
		}

		if ((in_slice[2] + in_slice[3]) % 2) {
			ds_en[DECON_DSC_ENC1] = 0;
			sm_ch[DECON_DSC_ENC1] = 1;
		} else {
			ds_en[DECON_DSC_ENC1] = 1;
			sm_ch[DECON_DSC_ENC1] = 0;
		}

		break;
	case 2:
		if (dsc_cnt == 2) {
			ds_en[DECON_DSC_ENC0] = 0;
			sm_ch[DECON_DSC_ENC0] = 1;

			ds_en[DECON_DSC_ENC1] = 0;
			sm_ch[DECON_DSC_ENC1] = 1;
		} else {
			ds_en[DECON_DSC_ENC0] = 1;
			sm_ch[DECON_DSC_ENC0] = 0;
			ds_en[DECON_DSC_ENC1] = ds_en[DECON_DSC_ENC0];
			sm_ch[DECON_DSC_ENC1] = sm_ch[DECON_DSC_ENC0];
		}
		break;
	case 1:
		ds_en[DECON_DSC_ENC0] = 0;
		sm_ch[DECON_DSC_ENC0] = 0;

		ds_en[DECON_DSC_ENC1] = 0;
		sm_ch[DECON_DSC_ENC1] = 0;
		break;
	default:
		cal_log_err(id, "Not specified case for Partial Update\n");
		break;
	}
}

static void dsc_reg_config_control(u32 id, u32 dsc_id, u32 ds_en, u32 sm_ch,
		u32 slice_width)
{
	u32 val;
	u32 remainder, grpcntline;

	val = DSC_SWAP(0x0, 0x1, 0x0);
	val |= DSC_DUAL_SLICE_EN_F(ds_en);
	val |= DSC_SLICE_MODE_CH_F(sm_ch);
	val |= DSC_FLATNESS_DET_TH_F(0x2);
	dsc_write(id, DSC_CONTROL1(dsc_id), val);

	remainder = slice_width % 3 ? : 3;
	grpcntline = (slice_width + 2) / 3;

	val = DSC_REMAINDER_F(remainder) | DSC_GRPCNTLINE_F(grpcntline);
	dsc_write(id, DSC_CONTROL3(dsc_id), val);
}

/*
 * overlap_w
 * - default : 0
 * - range : [0, 32] & (multiples of 2)
 *    if non-zero value is applied, this means slice_w increasing.
 *    therefore, DECON & DSIM setting must also be aligned.
 *    --> must check if DDI module is supporting this feature !!!
 */
#define NUM_EXTRA_MUX_BITS	246
static void dsc_calc_pps_info(struct decon_config *config, u32 dscc_en,
	struct decon_dsc *dsc_enc)
{
	u32 width, height;
	u32 slice_width, slice_height;
	u32 pic_width, pic_height;
	u32 width_eff;
	u32 dual_slice_en = 0;
	u32 bpp, chunk_size;
	u32 slice_bits;
	u32 groups_per_line, groups_total;

	/* initial values, also used for other pps calcualtion */
	const u32 rc_model_size = 0x2000;
	u32 num_extra_mux_bits = NUM_EXTRA_MUX_BITS;
	/* TODO: This values are copied from gs101. It will be updated
	   when dsc is enabled for zuma */
	const u32 initial_xmit_delay = 0x200;
	u32 initial_dec_delay = 0x4c0;
	/* when 'slice_w >= 70' */
	const u32 initial_scale_value = 0x20;
	u32 first_line_bpg_offset = 0x0c;
	const u32 initial_offset = 0x1800;
	const u32 rc_range_parameters = 0x0102;
	const struct drm_dsc_config *cfg = config->dsc.cfg;

	u32 final_offset, final_scale;
	u32 flag, nfl_bpg_offset, slice_bpg_offset;
	u32 scale_increment_interval, scale_decrement_interval;
	u32 slice_width_byte_unit, comp_slice_width_byte_unit;
	u32 comp_slice_width_pixel_unit;
	u32 overlap_w = 0;
	u32 dsc_enc0_w = 0;
	u32 dsc_enc1_w = 0;
	u32 tmp, hrd_delay;
	u32 i, j;

	width = config->image_width;
	height = config->image_height;

	if (cfg && cfg->first_line_bpg_offset)
		first_line_bpg_offset = cfg->first_line_bpg_offset;

	overlap_w = dsc_enc->overlap_w;

	if (dscc_en)
		/* OVERLAP can be used in the dual-slice case (if one ENC) */
		width_eff = (width >> 1) + overlap_w;
	else
		width_eff = width + overlap_w;

	pic_width = width_eff;
	dual_slice_en = dsc_get_dual_slice_mode(&config->dsc);
	if (dual_slice_en)
		slice_width = width_eff >> 1;
	else
		slice_width = width_eff;

	pic_height = height;
	slice_height = config->dsc.slice_height;

	bpp = 8;
	chunk_size = slice_width;
	slice_bits = 8 * chunk_size * slice_height;

	while ((slice_bits - num_extra_mux_bits) % 48)
		num_extra_mux_bits--;

	groups_per_line = (slice_width + 2) / 3;
	groups_total = groups_per_line * slice_height;

	final_offset = rc_model_size - ((initial_xmit_delay * (8<<4) + 8)>>4)
		+ num_extra_mux_bits;
	final_scale = 8 * rc_model_size / (rc_model_size - final_offset);

	flag = (first_line_bpg_offset * 2048) % (slice_height - 1);
	nfl_bpg_offset = (first_line_bpg_offset * 2048) / (slice_height - 1);
	if (flag)
		nfl_bpg_offset = nfl_bpg_offset + 1;

	flag = 2048 * (rc_model_size - initial_offset + num_extra_mux_bits)
		% groups_total;
	slice_bpg_offset = 2048
		* (rc_model_size - initial_offset + num_extra_mux_bits)
		/ groups_total;
	if (flag)
		slice_bpg_offset = slice_bpg_offset + 1;

	scale_increment_interval = (2048 * final_offset) / ((final_scale - 9)
		* (nfl_bpg_offset + slice_bpg_offset));
	scale_decrement_interval = groups_per_line / (initial_scale_value - 8);

	/*
	 * FIXME: the following initial_dec_delay math is a bit different from drm
	 * standard algorithm. Please revisit if seeing issue.
	 */
	tmp = groups_per_line * first_line_bpg_offset;
	hrd_delay = ((6144 + tmp) + 7) / bpp;
	initial_dec_delay = hrd_delay - 512;

	/* 3bytes per pixel */
	slice_width_byte_unit = slice_width * 3;
	/* integer value, /3 for 1/3 compression */
	comp_slice_width_byte_unit = slice_width_byte_unit / 3;
	/* integer value, /3 for pixel unit */
	comp_slice_width_pixel_unit = comp_slice_width_byte_unit / 3;

	i = comp_slice_width_byte_unit % 3;
	j = comp_slice_width_pixel_unit % 2;

	if (i == 0 && j == 0) {
		dsc_enc0_w = comp_slice_width_pixel_unit;
		if (dscc_en) {
			dsc_enc1_w = comp_slice_width_pixel_unit;
		}
	} else if (i == 0 && j != 0) {
		dsc_enc0_w = comp_slice_width_pixel_unit + 1;
		if (dscc_en) {
			dsc_enc1_w = comp_slice_width_pixel_unit + 1;
		}
	} else if (i != 0) {
		while (1) {
			comp_slice_width_pixel_unit++;
			j = comp_slice_width_pixel_unit % 2;
			if (j == 0)
				break;
		}
		dsc_enc0_w = comp_slice_width_pixel_unit;
		if (dscc_en) {
			dsc_enc1_w = comp_slice_width_pixel_unit;
		}
	}

	if (dual_slice_en) {
		dsc_enc0_w = dsc_enc0_w * 2;
		if (dscc_en)
			dsc_enc1_w = dsc_enc1_w * 2;
	}

	/* Save information to structure variable */
	dsc_enc->cfg = cfg;
	dsc_enc->comp_cfg = 0x30;
	dsc_enc->bit_per_pixel = bpp << 4;
	dsc_enc->pic_height = pic_height;
	dsc_enc->pic_width = pic_width;
	dsc_enc->slice_height = slice_height;
	dsc_enc->slice_width = slice_width;
	dsc_enc->chunk_size = chunk_size;
	dsc_enc->initial_xmit_delay = initial_xmit_delay;
	dsc_enc->initial_dec_delay = initial_dec_delay;
	dsc_enc->initial_scale_value = initial_scale_value;
	dsc_enc->scale_increment_interval = scale_increment_interval;
	dsc_enc->scale_decrement_interval = scale_decrement_interval;
	dsc_enc->first_line_bpg_offset = first_line_bpg_offset;
	dsc_enc->nfl_bpg_offset = nfl_bpg_offset;
	dsc_enc->slice_bpg_offset = slice_bpg_offset;
	dsc_enc->initial_offset = initial_offset;
	dsc_enc->final_offset = final_offset;
	dsc_enc->rc_range_parameters = rc_range_parameters;

	dsc_enc->width_per_enc = dsc_enc0_w;
}

static void dsc_reg_dump_pps(u32 id, u32 dsc_id)
{
#if defined(DECON_DSC_PPS_DEBUG)
	u32 i;
	for (i = 0; i < (88 / 4); i++) {
		u32 offset = i * 4;

		cal_log_info(id, "0x%08X - PPS[%u..%u]\n",
			dsc_read(id, DSC_PPS00_03(dsc_id) + offset),
			offset, offset + 3);
	}
#endif
}

static void dsc_reg_set_pps(u32 id, u32 dsc_id, struct decon_dsc *dsc_enc)
{
	u32 val;
	u8 b;
	const struct drm_dsc_config *cfg = dsc_enc->cfg;

	if (cfg)
		b = (cfg->dsc_version_major << DSC_PPS_VERSION_MAJOR_SHIFT)
			| cfg->dsc_version_minor;
	/* default DSC version is 1.1 */
	val = PPS00_DSC_VER((cfg && b) ? b : 0x11);
	/* default BPC = 8 */
	val |= PPS03_BPC((cfg && cfg->bits_per_component) ?
		cfg->bits_per_component : 8);
	/* default linebuf_depth = 9 bits */
	val |= PPS03_LBD((cfg && cfg->line_buf_depth) ?
		cfg->line_buf_depth : 9);
	dsc_write(id, DSC_PPS00_03(dsc_id), val);

	if (cfg)
		b = (cfg->block_pred_enable << DSC_PPS_BLOCK_PRED_EN_SHIFT) |
			(cfg->convert_rgb << DSC_PPS_CONVERT_RGB_SHIFT ) |
			(cfg->simple_422 << DSC_PPS_SIMPLE422_SHIFT) |
			(cfg->vbr_enable << DSC_PPS_VBR_EN_SHIFT);
	val = PPS04_COMP_CFG(cfg && b ? b : dsc_enc->comp_cfg);
	val |= PPS05_BPP(cfg && cfg->bits_per_pixel ?
		cfg->bits_per_pixel : dsc_enc->bit_per_pixel);
	val |= PPS06_07_PIC_HEIGHT(cfg && cfg->pic_height ?
		cfg->pic_height : dsc_enc->pic_height);
	dsc_write(id, DSC_PPS04_07(dsc_id), val);

	val = PPS08_09_PIC_WIDTH(cfg && cfg->pic_width ?
		cfg->pic_width : dsc_enc->pic_width);
	val |= PPS10_11_SLICE_HEIGHT(cfg && cfg->slice_height ?
		cfg->slice_height : dsc_enc->slice_height);
	dsc_write(id, DSC_PPS08_11(dsc_id), val);

	val = PPS12_13_SLICE_WIDTH(cfg && cfg->slice_width ?
		cfg->slice_width : dsc_enc->slice_width);
	val |= PPS14_15_CHUNK_SIZE(cfg && cfg->slice_chunk_size ?
		cfg->slice_chunk_size : dsc_enc->chunk_size);
	dsc_write(id, DSC_PPS12_15(dsc_id), val);

	val = PPS16_17_INIT_XMIT_DELAY(cfg && cfg->initial_xmit_delay ?
		cfg->initial_xmit_delay : dsc_enc->initial_xmit_delay);
	val |= PPS18_19_INIT_DEC_DELAY(cfg && cfg->initial_dec_delay ?
		cfg->initial_dec_delay : dsc_enc->initial_dec_delay);
	dsc_write(id, DSC_PPS16_19(dsc_id), val);

	val = PPS21_INIT_SCALE_VALUE( cfg && cfg->initial_scale_value ?
		cfg->initial_scale_value : dsc_enc->initial_scale_value);
	val |= PPS22_23_SCALE_INC_INTERVAL(cfg && cfg->scale_increment_interval ?
		cfg->scale_increment_interval : dsc_enc->scale_increment_interval);
	dsc_write(id, DSC_PPS20_23(dsc_id), val);

	val = PPS24_25_SCALE_DEC_INTERVAL(cfg && cfg->scale_decrement_interval ?
		cfg->scale_decrement_interval : dsc_enc->scale_decrement_interval);
	val |= PPS27_FL_BPG_OFFSET(cfg && cfg->first_line_bpg_offset ?
		cfg->first_line_bpg_offset : dsc_enc->first_line_bpg_offset);
	dsc_write(id, DSC_PPS24_27(dsc_id), val);

	val = PPS28_29_NFL_BPG_OFFSET(cfg && cfg->nfl_bpg_offset ?
		cfg->nfl_bpg_offset : dsc_enc->nfl_bpg_offset);
	val |= PPS30_31_SLICE_BPG_OFFSET(cfg && cfg->slice_bpg_offset ?
		cfg->slice_bpg_offset : dsc_enc->slice_bpg_offset);
	dsc_write(id, DSC_PPS28_31(dsc_id), val);

	val = PPS32_33_INIT_OFFSET(cfg && cfg->initial_offset ?
		cfg->initial_offset : dsc_enc->initial_offset);
	val |= PPS34_35_FINAL_OFFSET(cfg && cfg->final_offset ?
		cfg->final_offset : dsc_enc->final_offset);
	dsc_write(id, DSC_PPS32_35(dsc_id), val);

	if (cfg) {
		val = PPS36_FLATNESS_MIN_QP(cfg->flatness_min_qp);
		val |= PPS37_FLATNESS_MAX_QP(cfg->flatness_max_qp);
		val |= PPS38_39_RC_MODEL_SIZE(cfg->rc_model_size);
		if (val)
			dsc_write(id, DSC_PPS36_39(dsc_id), val);

		val = PPS40_RC_EDGE_FACTOR(cfg->rc_edge_factor);
		val |= PPS41_RC_QUANT_INCR_LIMIT0(cfg->rc_quant_incr_limit0);
		val |= PPS42_RC_QUANT_INCR_LIMIT1(cfg->rc_quant_incr_limit1);
		val |= PPS43_RC_TGT_OFFSET_HI(cfg->rc_tgt_offset_high);
		val |= PPS43_RC_TGT_OFFSET_LO(cfg->rc_tgt_offset_low);
		if (val)
			dsc_write(id, DSC_PPS40_43(dsc_id), val);

		dsc_reg_set_pps_44_57_rc_buf_thresh(id, dsc_id, cfg);
		dsc_reg_set_pps_58_87_rc_range_params(id, dsc_id, cfg);
	} else {
		/* min_qp0 = 0 , max_qp0 = 4 , bpg_off0 = 2 */
		dsc_reg_set_pps_58_59_rc_range_param0(id, dsc_id,
			dsc_enc->rc_range_parameters);

#ifndef VESA_SCR_V4
		/* PPS79 ~ PPS87 : 3HF4 is different with VESA SCR v4 */
		dsc_write(id, DSC_PPS76_79(dsc_id), 0x1AB62AF6);
		dsc_write(id, DSC_PPS80_83(dsc_id), 0x2B342B74);
		dsc_write(id, DSC_PPS84_87(dsc_id), 0x3B746BF4);
#endif
	}

	dsc_reg_dump_pps(id, dsc_id);
}

/*
 * Following PPS SFRs will be set from DDI PPS Table (DSC Decoder)
 * : not 'fix' type
 *   - PPS04 ~ PPS35
 *   - PPS58 ~ PPS59
 *   <PPS Table e.g.> SEQ_PPS_SLICE4[] @ s6e3hf4_param.h
 */
static void dsc_get_decoder_pps_info(struct decon_dsc *dsc_dec,
		const unsigned char pps_t[90])
{
	dsc_dec->comp_cfg = (u32) pps_t[4];
	dsc_dec->bit_per_pixel = (u32) pps_t[5];
	dsc_dec->pic_height = (u32) (pps_t[6] << 8 | pps_t[7]);
	dsc_dec->pic_width = (u32) (pps_t[8] << 8 | pps_t[9]);
	dsc_dec->slice_height = (u32) (pps_t[10] << 8 | pps_t[11]);
	dsc_dec->slice_width = (u32) (pps_t[12] << 8 | pps_t[13]);
	dsc_dec->chunk_size = (u32) (pps_t[14] << 8 | pps_t[15]);
	dsc_dec->initial_xmit_delay = (u32) (pps_t[16] << 8 | pps_t[17]);
	dsc_dec->initial_dec_delay = (u32) (pps_t[18] << 8 | pps_t[19]);
	dsc_dec->initial_scale_value = (u32) pps_t[21];
	dsc_dec->scale_increment_interval = (u32) (pps_t[22] << 8 | pps_t[23]);
	dsc_dec->scale_decrement_interval = (u32) (pps_t[24] << 8 | pps_t[25]);
	dsc_dec->first_line_bpg_offset = (u32) pps_t[27];
	dsc_dec->nfl_bpg_offset = (u32) (pps_t[28] << 8 | pps_t[29]);
	dsc_dec->slice_bpg_offset = (u32) (pps_t[30] << 8 | pps_t[31]);
	dsc_dec->initial_offset = (u32) (pps_t[32] << 8 | pps_t[33]);
	dsc_dec->final_offset = (u32) (pps_t[34] << 8 | pps_t[35]);
	dsc_dec->rc_range_parameters = (u32) (pps_t[58] << 8 | pps_t[59]);
}

static u32 dsc_cmp_pps_enc_dec(u32 id, struct decon_dsc *p_enc,
		struct decon_dsc *p_dec)
{
	u32 diff_cnt = 0;

	if (p_enc->comp_cfg != p_dec->comp_cfg) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] comp_cfg(enc:dec=%d:%d)\n",
			p_enc->comp_cfg, p_dec->comp_cfg);
	}
	if (p_enc->bit_per_pixel != p_dec->bit_per_pixel) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] bit_per_pixel(enc:dec=%d:%d)\n",
			p_enc->bit_per_pixel, p_dec->bit_per_pixel);
	}
	if (p_enc->pic_height != p_dec->pic_height) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] pic_height(enc:dec=%d:%d)\n",
			p_enc->pic_height, p_dec->pic_height);
	}
	if (p_enc->pic_width != p_dec->pic_width) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] pic_width(enc:dec=%d:%d)\n",
			p_enc->pic_width, p_dec->pic_width);
	}
	if (p_enc->slice_height != p_dec->slice_height) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] slice_height(enc:dec=%d:%d)\n",
			p_enc->slice_height, p_dec->slice_height);
	}
	if (p_enc->slice_width != p_dec->slice_width) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] slice_width(enc:dec=%d:%d)\n",
			p_enc->slice_width, p_dec->slice_width);
	}
	if (p_enc->chunk_size != p_dec->chunk_size) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] chunk_size(enc:dec=%d:%d)\n",
			p_enc->chunk_size, p_dec->chunk_size);
	}
	if (p_enc->initial_xmit_delay != p_dec->initial_xmit_delay) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] init_xmit_delay(enc:dec=%d:%d)\n",
			p_enc->initial_xmit_delay, p_dec->initial_xmit_delay);
	}
	if (p_enc->initial_dec_delay != p_dec->initial_dec_delay) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] init_dec_delay(enc:dec=%d:%d)\n",
			p_enc->initial_dec_delay, p_dec->initial_dec_delay);
	}
	if (p_enc->initial_scale_value != p_dec->initial_scale_value) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] init_scale_value(enc:dec=%d:%d)\n",
			p_enc->initial_scale_value,
			p_dec->initial_scale_value);
	}
	if (p_enc->scale_increment_interval !=
			p_dec->scale_increment_interval) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] scl_inc_interval(enc:dec=%d:%d)\n",
					p_enc->scale_increment_interval,
					p_dec->scale_increment_interval);
	}
	if (p_enc->scale_decrement_interval !=
			p_dec->scale_decrement_interval) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] scl_dec_interval(enc:dec=%d:%d)\n",
					p_enc->scale_decrement_interval,
					p_dec->scale_decrement_interval);
	}
	if (p_enc->first_line_bpg_offset != p_dec->first_line_bpg_offset) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] first_line_bpg(enc:dec=%d:%d)\n",
					p_enc->first_line_bpg_offset,
					p_dec->first_line_bpg_offset);
	}
	if (p_enc->nfl_bpg_offset != p_dec->nfl_bpg_offset) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] nfl_bpg_offset(enc:dec=%d:%d)\n",
			p_enc->nfl_bpg_offset, p_dec->nfl_bpg_offset);
	}
	if (p_enc->slice_bpg_offset != p_dec->slice_bpg_offset) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] slice_bpg_offset(enc:dec=%d:%d)\n",
			p_enc->slice_bpg_offset, p_dec->slice_bpg_offset);
	}
	if (p_enc->initial_offset != p_dec->initial_offset) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] initial_offset(enc:dec=%d:%d)\n",
			p_enc->initial_offset, p_dec->initial_offset);
	}
	if (p_enc->final_offset != p_dec->final_offset) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] final_offset(enc:dec=%d:%d)\n",
			p_enc->final_offset, p_dec->final_offset);
	}
	if (p_enc->rc_range_parameters != p_dec->rc_range_parameters) {
		diff_cnt++;
		cal_log_debug(id, "[dsc_pps] rc_range_params(enc:dec=%d:%d)\n",
						p_enc->rc_range_parameters,
						p_dec->rc_range_parameters);
	}

	cal_log_debug(id, "[dsc_pps] total different count : %d\n", diff_cnt);

	return diff_cnt;
}

static void dsc_reg_set_partial_update(u32 id, u32 dsc_id, u32 dual_slice_en,
	u32 slice_mode_ch, u32 pic_h)
{
	/*
	 * Following SFRs must be considered
	 * - dual_slice_en
	 * - slice_mode_change
	 * - picture_height
	 * - picture_width (don't care @KC) : decided by DSI (-> dual: /2)
	 */
	dsc_reg_set_dual_slice(id, dsc_id, dual_slice_en);
	dsc_reg_set_slice_mode_change(id, dsc_id, slice_mode_ch);
	dsc_reg_set_pps_06_07_picture_height(id, dsc_id, pic_h);
}

/*
 * This table is only used to check DSC setting value when debugging
 * Copy or Replace table's data from current using LCD information
 * ( e.g. : SEQ_PPS_SLICE4 @ s6e3hf4_param.h )
 */
static const unsigned char DDI_PPS_INFO[] = {
	0x11, 0x00, 0x00, 0x89, 0x30,
	0x80, 0x0A, 0x00, 0x05, 0xA0,
	0x00, 0x40, 0x01, 0x68, 0x01,
	0x68, 0x02, 0x00, 0x01, 0xB4,

	0x00, 0x20, 0x04, 0xF2, 0x00,
	0x05, 0x00, 0x0C, 0x01, 0x87,
	0x02, 0x63, 0x18, 0x00, 0x10,
	0xF0, 0x03, 0x0C, 0x20, 0x00,

	0x06, 0x0B, 0x0B, 0x33, 0x0E,
	0x1C, 0x2A, 0x38, 0x46, 0x54,
	0x62, 0x69, 0x70, 0x77, 0x79,
	0x7B, 0x7D, 0x7E, 0x01, 0x02,

	0x01, 0x00, 0x09, 0x40, 0x09,
	0xBE, 0x19, 0xFC, 0x19, 0xFA,
	0x19, 0xF8, 0x1A, 0x38, 0x1A,
	0x78, 0x1A, 0xB6, 0x2A, 0xF6,

	0x2B, 0x34, 0x2B, 0x74, 0x3B,
	0x74, 0x6B, 0xF4, 0x00, 0x00
};

static void dsc_reg_set_encoder(u32 id, struct decon_config *config,
		struct decon_dsc *dsc_enc, u32 chk_en)
{
	u32 dsc_id;
	u32 dscc_en = 1;
	u32 ds_en = 0;
	u32 sm_ch = 0;
	/* DDI PPS table : for compare with ENC PPS value */
	struct decon_dsc dsc_dec;
	/* set corresponding table like 'SEQ_PPS_SLICE4' */
	const unsigned char *pps_t = DDI_PPS_INFO;

	ds_en = dsc_get_dual_slice_mode(&config->dsc);
	cal_log_debug(id, "dual slice(%d)\n", ds_en);

	sm_ch = dsc_get_slice_mode_change(&config->dsc);
	cal_log_debug(id, "slice mode change(%d)\n", sm_ch);

	dscc_en = decon_reg_get_data_path_cfg(id, PATH_CON_ID_DSCC_EN);
	dsc_calc_pps_info(config, dscc_en, dsc_enc);

	if (id == 1) {
		dsc_reg_config_control(id, DECON_DSC_ENC1, ds_en, sm_ch,
				dsc_enc->slice_width);
		dsc_reg_set_pps(id, DECON_DSC_ENC1, dsc_enc);
	} else if (id == 2) {	/* only for DP */
		dsc_reg_config_control(id, DECON_DSC_ENC2, ds_en, sm_ch,
				dsc_enc->slice_width);
		dsc_reg_set_pps(id, DECON_DSC_ENC2, dsc_enc);
	} else {
		for (dsc_id = 0; dsc_id < config->dsc.dsc_count; dsc_id++) {
			dsc_reg_config_control(id, dsc_id, ds_en, sm_ch,
					dsc_enc->slice_width);
			dsc_reg_set_pps(id, dsc_id, dsc_enc);
		}
	}

	if (chk_en) {
		dsc_get_decoder_pps_info(&dsc_dec, pps_t);
		if (dsc_cmp_pps_enc_dec(id, dsc_enc, &dsc_dec))
			cal_log_debug(id, "[WARNING] Check PPS value!!\n");
	}
}

static int dsc_reg_init(u32 id, struct decon_config *config, u32 overlap_w,
		u32 swrst)
{
	u32 dsc_id;
	struct decon_dsc dsc_enc;

	/* Basically, all SW-resets in DPU are not necessary */
	if (swrst) {
		for (dsc_id = 0; dsc_id < config->dsc.dsc_count; dsc_id++)
			dsc_reg_swreset(id, dsc_id);
	}

	dsc_enc.overlap_w = overlap_w;
	dsc_reg_set_encoder(id, config, &dsc_enc, 0);
	decon_reg_config_data_path_size(id, dsc_enc.width_per_enc,
			config->image_height, overlap_w, &dsc_enc, &config->dsc,
			config->out_bpc, config->mode.dsi_mode);

	return 0;
}

static void decon_reg_clear_int(u32 id, u32 mask)
{
	decon_write_mask(id, DECON_INT_PEND, mask, mask);
}

static void decon_reg_clear_int_all(u32 id)
{
	u32 mask;

	mask = INT_PEND_DQE_DIMMING_END | INT_PEND_DQE_DIMMING_START |
		INT_PEND_FRAME_DONE | INT_PEND_FRAME_START;
	decon_reg_clear_int(id, mask);

	mask = INT_PEND_RESOURCE_CONFLICT | INT_PEND_TIME_OUT;
	decon_write_mask(id, DECON_INT_PEND_EXTRA, ~0, mask);
}

static void decon_reg_configure_lcd(u32 id, struct decon_config *config)
{
	u32 overlap_w = 0;
	enum decon_rgb_order rgb_order = DECON_RGB;

	if ((config->out_type & DECON_OUT_DSI) && !(config->dsc.enabled))
		rgb_order = DECON_BGR;
	else
		rgb_order = DECON_RGB;
	decon_reg_set_rgb_order(id, rgb_order);

	decon_reg_set_data_path(id, config);

	if (config->dsc.enabled) {
		/* call decon_reg_config_data_path_size () inside */
		dsc_reg_init(id, config, overlap_w, 0);
	} else {
		decon_reg_config_data_path_size(id, config->image_width,
				config->image_height, overlap_w, NULL, &config->dsc,
				config->out_bpc, config->mode.dsi_mode);
	}

	decon_reg_per_frame_off(id);
}

static void decon_reg_set_blender_bg_size(u32 id, enum decon_dsi_mode dsi_mode,
		u32 bg_w, u32 bg_h)
{
	u32 width, val;

	width = bg_w;

	val = BLENDER_BG_HEIGHT_F(bg_h) | BLENDER_BG_WIDTH_F(width);
	decon_write(id, BLD_BG_IMG_SIZE_PRI, val);
}

static void decon_reg_set_splitter(u32 id, struct decon_config *config)
{
	u32 val;

	if (config->mode.dsi_mode != DSI_MODE_DUAL_DSI)
		return;

	val = SPLITTER_IN_HEIGHT_F(config->image_height) | SPLITTER_IN_WIDTH_F(config->image_width);
	decon_write(id, OF_SPLIT_SIZE, val);

	val = SPLITTER_SPLIT_IDX_F(config->image_width / 2);
	decon_write_mask(id, OF_SPLIT_IDX, val, SPLITTER_SPLIT_IDX_MASK);
}

static int decon_reg_stop_perframe_dsi(u32 id, struct decon_config *config,
		u32 fps)
{
	int ret = 0;
	/* timeout_us : 1000000us / fps + 50% margin */
	const int timeout_us = DIV_ROUND_UP(USEC_PER_SEC * 15, fps * 10);

	cal_log_debug(id, "%s +\n", __func__);

	if ((config->mode.op_mode == DECON_COMMAND_MODE) &&
			(config->mode.trig_mode == DECON_HW_TRIG))
		decon_reg_set_trigger(id, &config->mode, DECON_TRIG_MASK);

	/* perframe stop */
	decon_reg_per_frame_off(id);
	decon_reg_update_req_global(id);

	ret = decon_reg_wait_run_is_off_timeout(id, timeout_us);

	cal_log_debug(id, "%s -\n", __func__);
	return ret;
}

static int decon_reg_stop_perframe_dp(u32 id, u32 fps)
{
	int ret = 0;
	/* timeout_us : 1000000us / fps + 50% margin */
	const int timeout_us = DIV_ROUND_UP(USEC_PER_SEC * 15, fps * 10);

	cal_log_debug(id, "%s +\n", __func__);

	/* perframe stop */
	decon_reg_per_frame_off(id);
	decon_reg_update_req_global(id);

	ret = decon_reg_wait_run_is_off_timeout(id, timeout_us);

	cal_log_debug(id, "%s -\n", __func__);
	return ret;
}

static int decon_reg_stop_inst_dsi(u32 id, struct decon_config *config, u32 fps)
{
	int ret = 0;
	const int timeout_us = DIV_ROUND_UP(USEC_PER_SEC * 15, fps * 10);

	cal_log_debug(id, "%s +\n", __func__);

	if ((config->mode.op_mode == DECON_COMMAND_MODE) &&
			(config->mode.trig_mode == DECON_HW_TRIG))
		decon_reg_set_trigger(id, &config->mode, DECON_TRIG_MASK);

	/* instant stop */
	decon_reg_direct_on_off(id, 0);
	decon_reg_update_req_global(id);

	ret = decon_reg_wait_run_is_off_timeout(id, timeout_us);

	cal_log_debug(id, "%s -\n", __func__);
	return ret;
}

static int decon_reg_stop_inst_dp(u32 id, u32 fps)
{
	int ret = 0;
	const int timeout_us = DIV_ROUND_UP(USEC_PER_SEC * 15, fps * 10);

	cal_log_debug(id, "%s +\n", __func__);

	/* instant stop */
	decon_reg_direct_on_off(id, 0);
	decon_reg_update_req_global(id);

	ret = decon_reg_wait_run_is_off_timeout(id, timeout_us);

	cal_log_debug(id, "%s -\n", __func__);
	return ret;
}

void decon_reg_set_win_enable(u32 id, u32 win_idx, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	mask = _WIN_EN_F;
	wincon_write_mask(id, DECON_CON_WIN(win_idx), val, mask);
	cal_log_debug(id, "%s: WINCON%d = 0x%x\n", __func__, win_idx,
			wincon_read(id, DECON_CON_WIN(win_idx)));
}

int decon_reg_get_win_ch(u32 id, u32 win_idx, u32 *ch)
{
	u32 val, mask;

	if (!ch)
		return -EINVAL;

	mask = _WIN_EN_F | WIN_CHMAP_MASK;
	val = wincon_read_mask(id, DECON_CON_WIN(win_idx), mask);
	if ((val & _WIN_EN_F) == 0)
		return -ENOENT;

	*ch = WIN_CHMAP_GET(val);
	/* L7 layer is not present in Zuma */
	if (*ch > 7)
		*ch = *ch - 1;
	else if (WARN_ON(*ch == 7))
		cal_log_warn(id, "L7 layer is not supported\n");

	return 0;
}

/*
 * argb_color : 32-bit
 * A[31:24] - R[23:16] - G[15:8] - B[7:0]
 */
static void decon_reg_set_win_mapcolor(u32 id, u32 win_idx, u32 argb_color, u32 in_bpc)
{
	u32 val, mask;
	u32 mc_alpha = 0, mc_red = 0;
	u32 mc_green = 0, mc_blue = 0;

	mc_alpha = (argb_color >> 24) & 0xFF;
	mc_red = (argb_color >> 16) & 0xFF;
	mc_green = (argb_color >> 8) & 0xFF;
	mc_blue = (argb_color >> 0) & 0xFF;

	if (in_bpc == 10) {
		mc_red = (mc_red << 2) | ((mc_red >> 6) & 0x3);
		mc_green = (mc_green << 2) | ((mc_green >> 6) & 0x3);
		mc_blue = (mc_blue << 2) | ((mc_blue >> 6) & 0x3);
	}
	cal_log_debug(id, "TEN_BPC=%d : A=%02Xh, R=%03Xh, G=%03Xh, B=%03Xh\n",
			in_bpc ? 1 : 0, mc_alpha, mc_red, mc_green, mc_blue);


	val = WIN_MAPCOLOR_A_F(mc_alpha) | WIN_MAPCOLOR_R_F(mc_red);
	mask = WIN_MAPCOLOR_A_MASK | WIN_MAPCOLOR_R_MASK;
	win_write_mask(id, WIN_COLORMAP_0(win_idx), val, mask);

	val = WIN_MAPCOLOR_G_F(mc_green) | WIN_MAPCOLOR_B_F(mc_blue);
	mask = WIN_MAPCOLOR_G_MASK | WIN_MAPCOLOR_B_MASK;
	win_write_mask(id, WIN_COLORMAP_1(win_idx), val, mask);
}

static void decon_reg_set_win_plane_alpha(u32 id, u32 win_idx, u32 a0, u32 a1)
{
	u32 val, mask;

	val = WIN_ALPHA1_F(a1) | WIN_ALPHA0_F(a0);
	mask = WIN_ALPHA1_MASK | WIN_ALPHA0_MASK;
	win_write_mask(id, WIN_FUNC_CON_0(win_idx), val, mask);
}

static void decon_reg_set_winmap(u32 id, u32 win_idx, u32 color, u32 in_bpc, u32 en)
{
	u32 val, mask;

	/* Enable */
	val = en ? ~0 : 0;
	mask = WIN_MAPCOLOR_EN_F;
	wincon_write_mask(id, DECON_CON_WIN(win_idx), val, mask);
	cal_log_debug(id, "%s: 0x%x\n", __func__,
			wincon_read(id, DECON_CON_WIN(win_idx)));

	/* Color Set */
	decon_reg_set_win_mapcolor(id, win_idx, color, in_bpc);
}

/* ALPHA_MULT selection used in (a',b',c',d') coefficient */
static void decon_reg_set_win_alpha_mult(u32 id, u32 win_idx, u32 a_sel)
{
	u32 val, mask;

	val = WIN_ALPHA_MULT_SRC_SEL_F(a_sel);
	mask = WIN_ALPHA_MULT_SRC_SEL_MASK;
	win_write_mask(id, WIN_FUNC_CON_0(win_idx), val, mask);
}

static void decon_reg_set_win_sub_coeff(u32 id, u32 win_idx,
		u32 fgd, u32 bgd, u32 fga, u32 bga)
{
	u32 val, mask;

	/*
	 * [ Blending Equation ]
	 * Color : Cr = (a x Cf) + (b x Cb)  <Cf=FG pxl_C, Cb=BG pxl_C>
	 * Alpha : Ar = (c x Af) + (d x Ab)  <Af=FG pxl_A, Ab=BG pxl_A>
	 *
	 * [ User-defined ]
	 * a' = WINx_FG_ALPHA_D_SEL : Af' that is multiplied by FG Pixel Color
	 * b' = WINx_BG_ALPHA_D_SEL : Ab' that is multiplied by BG Pixel Color
	 * c' = WINx_FG_ALPHA_A_SEL : Af' that is multiplied by FG Pixel Alpha
	 * d' = WINx_BG_ALPHA_A_SEL : Ab' that is multiplied by BG Pixel Alpha
	 */

	val = (WIN_FG_ALPHA_D_SEL_F(fgd)
		| WIN_BG_ALPHA_D_SEL_F(bgd)
		| WIN_FG_ALPHA_A_SEL_F(fga)
		| WIN_BG_ALPHA_A_SEL_F(bga));
	mask = (WIN_FG_ALPHA_D_SEL_MASK
		| WIN_BG_ALPHA_D_SEL_MASK
		| WIN_FG_ALPHA_A_SEL_MASK
		| WIN_BG_ALPHA_A_SEL_MASK);
	win_write_mask(id, WIN_FUNC_CON_1(win_idx), val, mask);
}

static void decon_reg_set_win_func(u32 id, u32 win_idx,
		enum decon_win_func pd_func)
{
	u32 val, mask;

	val = WIN_FUNC_F(pd_func);
	mask = WIN_FUNC_MASK;
	win_write_mask(id, WIN_FUNC_CON_0(win_idx), val, mask);
}

static void decon_reg_set_win_bnd_function(u32 id, u32 win_idx,
		struct decon_window_regs *regs)
{
	int plane_a = regs->plane_alpha;
	u32 blend = regs->blend;
	enum decon_win_func pd_func = PD_FUNC_USER_DEFINED;
	u8 alpha0 = plane_a;
	u8 alpha1 = 0;
	bool is_plane_a = false;
	u32 af_d = BND_COEF_ONE, ab_d = BND_COEF_ZERO,
		af_a = BND_COEF_ONE, ab_a = BND_COEF_ZERO;

	if ((plane_a > 0) && (plane_a <= 0xff))
		is_plane_a = true;

	if ((blend == DECON_BLENDING_NONE) && is_plane_a) {
		af_d = BND_COEF_PLANE_ALPHA0;
		ab_d = BND_COEF_ZERO;
		af_a = BND_COEF_PLANE_ALPHA0;
		ab_a = BND_COEF_ZERO;
	} else if (blend == DECON_BLENDING_COVERAGE) {
		af_d = BND_COEF_ALPHA_MULT;
		ab_d = BND_COEF_1_M_ALPHA_MULT;
		af_a = BND_COEF_ALPHA_MULT;
		ab_a = BND_COEF_1_M_ALPHA_MULT;
	} else if (blend == DECON_BLENDING_PREMULT) {
		af_d = BND_COEF_PLANE_ALPHA0;
		ab_d = BND_COEF_1_M_ALPHA_MULT;
		af_a = BND_COEF_PLANE_ALPHA0;
		ab_a = BND_COEF_1_M_ALPHA_MULT;
	} else if (blend == DECON_BLENDING_NONE) {
		cal_log_debug(id, "none blending(no plane alpha) mode\n");
	} else {
		cal_log_warn(id, "undefined blending mode\n");
	}

	decon_reg_set_win_plane_alpha(id, win_idx, alpha0, alpha1);
	decon_reg_set_win_alpha_mult(id, win_idx, ALPHA_MULT_SRC_SEL_AF);
	decon_reg_set_win_func(id, win_idx, pd_func);
	if (pd_func == PD_FUNC_USER_DEFINED)
		decon_reg_set_win_sub_coeff(id,
				win_idx, af_d, ab_d, af_a, ab_a);
}

/*
 * SLEEP_CTRL_MODE_F
 * 0 = Bypass shadow update request to DSIMIF
 * 1 = Postpone shadow update request to DSIMIF until PLL lock
 */
void decon_reg_set_pll_sleep(u32 id, u32 en)
{
	u32 val, mask;

	if (id >= 2) {
		cal_log_info(id, "pll sleep is not allowed\n");
		return;
	}
	val = en ? ~0 : 0;
	mask = (id == 0) ? PLL_SLEEP_EN_OUTIF0_F : PLL_SLEEP_EN_OUTIF1_F;
	mask |= SLEEP_CTRL_MODE_F;
	decon_write_mask(id, PLL_SLEEP_CON, val, mask);
}

void decon_reg_set_pll_wakeup(u32 id, u32 en)
{
	u32 val, mask;

	if (id >= 2) {
		cal_log_info(id, "pll wake-up is not allowed\n");
		return;
	}
	val = en ? ~0 : 0;
	mask = (id == 0) ? PLL_SLEEP_MASK_OUTIF0 : PLL_SLEEP_MASK_OUTIF1;
	decon_write_mask(id, PLL_SLEEP_CON, val, mask);
}

static void decon_reg_set_dither_path(u32 id, bool en)
{
	u32 val;

	if (id >= MAX_DECON_CNT) {
		cal_log_err(id, "decon(%d) is invalid to set dither\n", id);
		return;
	}

	/*
	 * DECON0 uses display dither in DQE and DECON1/2 use just dither
	 * which is placed between blender and DSC.
	 *
	 * In order for DECON0 to use display dither in DQE, both DQE_ON
	 * and DITHER_ON must be set in the ENHANCE_PATH_F. The meaning of
	 * DITHER_ON in DECON0 case indicates that the DQE output has been
	 * dithered.
	 *
	 * However, the meaning of DITHER_ON in DECON1/2 is to just use dither
	 * which is placed between blender and DSC.
	 */
	val = en ? ENHANCE_PATH_F(ENHANCEPATH_DITHER_ON) : 0;
	decon_write_mask(id, DATA_PATH_CON_0, val, ENHANCE_DITHER_ON);
}

static void decon_reg_set_urgent(u32 id, struct decon_config *config)
{
	decon_reg_set_rd_urgent_enable(id, config->urgent.rd_en);
	decon_reg_set_rd_urgent_threshold(id,
					  config->urgent.rd_hi_thres,
					  config->urgent.rd_lo_thres);
	decon_reg_set_rd_wait_cycle(id, config->urgent.rd_wait_cycle);
	decon_reg_set_wr_urgent_enable(id, config->urgent.wr_en);
	decon_reg_set_wr_urgent_threshold(id,
					  config->urgent.wr_hi_thres,
					  config->urgent.wr_lo_thres);
	decon_reg_set_dta_enable(id, config->urgent.dta_en);
	decon_reg_set_dta_threshold(id,
				    config->urgent.dta_hi_thres,
				    config->urgent.dta_lo_thres);
}

/******************** EXPORTED DECON CAL APIs ********************/
/* TODO: maybe this function will be moved to internal DECON CAL function */
void decon_reg_update_req_global(u32 id)
{
	u32 mask = SHD_REG_UP_REQ_GLOBAL;

	if (id != 2)
		mask |= SHD_REG_UP_REQ_CMP;

	decon_write_mask(id, SHD_REG_UP_REQ, ~0, mask);
}

int decon_reg_init(u32 id, struct decon_config *config)
{
	decon_reg_set_clkgate_mode(id, 0);

	if (config->out_type & DECON_OUT_DP)
		decon_reg_set_qactive_pll_mode(id, 1);

	decon_reg_alloc_shared_sram_instance(id, true, true);

	decon_reg_set_sram_enable(id);

	decon_reg_set_operation_mode(id, config->mode.op_mode);

	decon_reg_set_blender_bg_size(id, config->mode.dsi_mode,
			config->image_width, config->image_height);

	/* enable once at init time */
	decon_reg_set_latency_monitor_enable(id, 1);

	/* enable rd/wr urgent */
	decon_reg_set_urgent(id, config);

	decon_reg_init_trigger(id, config);
	decon_reg_configure_lcd(id, config);
	decon_reg_set_splitter(id, config);

	if (config->mode.op_mode == DECON_COMMAND_MODE) {
#if defined(CONFIG_EXYNOS_EWR)
		/* 60fps: request wakeup at 16.566ms after TE rising */
		/* TODO: 60 will be changed to fps value */
		decon_reg_set_ewr_control(id,
				decon_get_ewr_cycle(60, 100), 1);
#endif
	}

	/* asserted interrupt should be cleared before initializing decon hw */
	decon_reg_clear_int_all(id);

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
	/* TODO : register for outfifo2 doesn't exist, needs a confirm */
	if (config->mode.op_mode == DECON_COMMAND_MODE &&
			config->mode.dsi_mode != DSI_MODE_DUAL_DSI)
		decon_reg_set_pll_sleep(id, 1);
#endif

	return 0;
}

int decon_reg_start(u32 id, struct decon_config *config)
{
	int ret = 0;

	if (config->mode.op_mode == DECON_COMMAND_MODE) {
		decon_reg_direct_on_off(id, 1);
		decon_reg_update_req_global(id);
	}

	/* clear pending frame start if any to ensure next frame start reflects new update */
	decon_reg_clear_int(id, INT_EN_FRAME_START);

	/*
	 * DECON goes to run-status as soon as
	 * request shadow update without HW_TE
	 */
	ret = decon_reg_wait_run_status_timeout(id, 2 * 1000); /* timeout 2ms */

	/* wait until run-status, then trigger */
	if (config->mode.op_mode == DECON_COMMAND_MODE)
		decon_reg_set_trigger(id, &config->mode, DECON_TRIG_UNMASK);
	return ret;
}

/*
 * stop sequence should be carefully for stability
 * try sequecne
 *	1. perframe off
 *	2. instant off
 */
int decon_reg_stop(u32 id, struct decon_config *config, bool rst, u32 fps)
{
	int ret = 0;

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
	/* when pll is asleep, need to wake it up before stopping */
	if (config->mode.op_mode == DECON_COMMAND_MODE &&
			config->mode.dsi_mode != DSI_MODE_DUAL_DSI)
		decon_reg_set_pll_wakeup(id, 1);
#endif

	if (decon_reg_alloc_shared_sram_instance(id, true, false))
		decon_reg_set_sram_enable(id);

	if (config->out_type & DECON_OUT_DP)
		decon_reg_set_qactive_pll_mode(id, 0);

	/* call perframe stop */
	if (config->out_type & DECON_OUT_DSI) {
		ret = decon_reg_stop_perframe_dsi(id, config, fps);
		if (ret < 0) {
			cal_log_err(id, "failed to perframe_stop(DSI)\n");
			/* if fails, call decon instant off */
			ret = decon_reg_stop_inst_dsi(id, config, fps);
			if (ret < 0)
				cal_log_err(id, "failed to inst_stop(DSI)\n");
		}
	} else if (config->out_type & DECON_OUT_DP) {
		ret = decon_reg_stop_perframe_dp(id, fps);
		if (ret < 0) {
			cal_log_err(id, "failed to perframe_stop(DP)\n");
			/* if fails, call decon instant off */
			ret = decon_reg_stop_inst_dp(id, fps);
			if (ret < 0)
				cal_log_err(id, "failed to instant_stop(DP)\n");
		}
	}

	/* assert reset when stopped normally or requested */
	if (!ret && rst)
		decon_reg_reset(id);

	decon_reg_clear_int_all(id);

	return ret;
}

void decon_reg_set_bpc_and_dither_path(u32 id, struct decon_config *config)
{
	/*
	 * decon processes data in the bpc mode of DPP,
	 * If any input data of DECON from DPP is 10bpc mode,
	 * then DECON and the others DPP also support 10bpc mode.
	 */
	decon_reg_set_bpc(id, config->in_bpc);

	/*
	 * If input data of decon is 10bpc and output data of decon
	 * is transferred to 8bpc mode panel, dither in decon must be enabled
	 * to change 10bpc to 8bpc data.
	 */
	if (config->in_bpc == 10 && config->out_bpc == 8)
		decon_reg_set_dither_path(id, true);
	else
		decon_reg_set_dither_path(id, false);
}

void decon_reg_win_enable_and_update(u32 id, u32 win_idx, u32 en)
{
	decon_reg_set_win_enable(id, win_idx, en);
	decon_reg_update_req_window(id, win_idx);
}

void decon_reg_all_win_shadow_update_req(u32 id)
{
	u32 mask;

	mask = SHD_REG_UP_REQ_FOR_DECON;

	decon_write_mask(id, SHD_REG_UP_REQ, ~0, mask);
}

void decon_video_mode_reg_update_req(u32 id, bool cgc_need_update, bool dqe_need_update)
{
	u32 mask;

	mask = SHD_REG_UP_REQ_FOR_DECON | SHD_REG_UP_REQ_GLOBAL;
	if (id != 2)
		mask |= SHD_REG_UP_REQ_CMP;
	if (cgc_need_update)
		mask |= SHD_REG_UP_REQ_DQE_CGC;
	if (dqe_need_update)
		mask |= SHD_REG_UP_REQ_DQE;

	decon_write_mask(id, SHD_REG_UP_REQ, ~0, mask);
}

void decon_reg_set_window_control(u32 id, int win_idx,
		struct decon_window_regs *regs, u32 winmap_en)
{
	cal_log_debug(id, "win id = %d\n", win_idx);
	decon_reg_set_win_bnd_function(id, win_idx, regs);
	win_write(id, WIN_START_POSITION(win_idx), regs->start_pos);
	win_write(id, WIN_END_POSITION(win_idx), regs->end_pos);
	win_write(id, WIN_START_TIME_CON(win_idx), regs->start_time);
	decon_reg_set_winmap(id, win_idx, regs->colormap, regs->in_bpc, winmap_en);

	decon_reg_config_win_channel(id, win_idx, regs->ch);
	decon_reg_set_win_enable(id, win_idx, 1);

	cal_log_debug(id, "regs->ch(%d)\n", regs->ch);
}

void decon_reg_update_req_window_mask(u32 id, u32 win_idx)
{
	u32 mask;

	mask = SHD_REG_UP_REQ_FOR_DECON;
	mask &= ~(SHD_REG_UP_REQ_WIN(win_idx));
	decon_write_mask(id, SHD_REG_UP_REQ, ~0, mask);
}

void decon_reg_update_req_window(u32 id, u32 win_idx)
{
	u32 mask;

	mask = SHD_REG_UP_REQ_WIN(win_idx);
	decon_write_mask(id, SHD_REG_UP_REQ, ~0, mask);
}

void decon_reg_update_req_dqe(u32 id)
{
	decon_write_mask(id, SHD_REG_UP_REQ, ~0, SHD_REG_UP_REQ_DQE);
}

void decon_reg_set_trigger(u32 id, struct decon_mode *mode,
		enum decon_set_trig trig)
{
	u32 val, mask;

	if (mode->op_mode == DECON_VIDEO_MODE)
		return;

	if (mode->trig_mode == DECON_SW_TRIG) {
		val = (trig == DECON_TRIG_UNMASK) ?
				(SW_TRIG_EN | SW_TRIG_DET_EN) : 0;
		mask = HW_TRIG_EN | SW_TRIG_EN | SW_TRIG_DET_EN;
	} else { /* DECON_HW_TRIG */
		val = (trig == DECON_TRIG_UNMASK) ?
				HW_TRIG_EN : HW_TRIG_MASK_DECON;
		mask = HW_TRIG_EN | HW_TRIG_MASK_DECON;
	}

	decon_write_mask(id, TRIG_CON, val, mask);
}

void decon_reg_update_req_and_unmask(u32 id, struct decon_mode *mode)
{
	decon_reg_update_req_global(id);

	if (mode->op_mode == DECON_COMMAND_MODE)
		decon_reg_set_trigger(id, mode, DECON_TRIG_UNMASK);
}

int decon_reg_wait_update_done_timeout(u32 id, unsigned long timeout_us)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(
			decon_regs_desc(id)->regs + SHD_REG_UP_REQ, val,
			!val, 10, timeout_us);
	if (ret) {
		cal_log_err(id, "timeout of updating decon registers (0x%x)\n", val);

		return ret;
	}

	return 0;
}

int decon_reg_wait_update_done_and_mask(u32 id, struct decon_mode *mode,
		u32 timeout_us)
{
	int result;

	result = decon_reg_wait_update_done_timeout(id, timeout_us);

	if (mode->op_mode == DECON_COMMAND_MODE)
		decon_reg_set_trigger(id, mode, DECON_TRIG_MASK);

	return result;
}

bool decon_reg_is_idle(u32 id)
{
	return decon_read_mask(id, GLOBAL_CON, GLOBAL_CON_IDLE_STATUS) != 0;
}

int decon_reg_wait_idle_status_timeout(u32 id, unsigned long timeout)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(
			decon_regs_desc(id)->regs + GLOBAL_CON, val,
			(val & GLOBAL_CON_IDLE_STATUS), 10, timeout);
	if (ret) {
		cal_log_err(id, "wait timeout decon idle status\n");
		return ret;
	}

	return 0;
}

void decon_reg_set_partial_update(u32 id, struct decon_config *config,
		bool in_slice[], u32 partial_w, u32 partial_h)
{
	u32 dual_slice_en[2] = {1, 1};
	u32 slice_mode_ch[2] = {0, 0};

	decon_reg_set_blender_bg_size(id, config->mode.dsi_mode,
			partial_w, partial_h);

	if (config->dsc.enabled) {
		/* get correct DSC configuration */
		dsc_get_partial_update_info(id, config->dsc.slice_count,
				config->dsc.dsc_count, in_slice,
				dual_slice_en, slice_mode_ch);
		/* To support dual-display : DECON1 have to set DSC1 */
		dsc_reg_set_partial_update(id, id, dual_slice_en[0],
				slice_mode_ch[0], partial_h);
		if (config->dsc.dsc_count == 2)
			dsc_reg_set_partial_update(id, 1, dual_slice_en[1],
					slice_mode_ch[1], partial_h);

		decon_reg_update_req_compress(id);
	}

	decon_reg_set_data_path_size(id, partial_w, partial_h,
			config->dsc.enabled, config->dsc.dsc_count,
			config->dsc.slice_width, config->dsc.slice_height,
			dual_slice_en, config->out_bpc);
}

void decon_reg_set_mres(u32 id, struct decon_config *config)
{
	u32 overlap_w = 0;

	if (config->mode.op_mode != DECON_COMMAND_MODE) {
		cal_log_info(id, "op mode[%d] doesn't support multi resol\n",
				config->mode.op_mode);
		return;
	}

	decon_reg_set_blender_bg_size(id, config->mode.dsi_mode,
			config->image_width, config->image_height);

	if (config->dsc.enabled)
		dsc_reg_init(id, config, overlap_w, 0);
	else
		decon_reg_config_data_path_size(id, config->image_width,
				config->image_height, overlap_w, NULL, &config->dsc,
				config->out_bpc, config->mode.dsi_mode);
}

void decon_reg_release_resource(u32 id, struct decon_mode *mode)
{
	decon_reg_per_frame_off(id);
	decon_reg_update_req_global(id);
	decon_reg_set_trigger(id, mode, DECON_TRIG_UNMASK);
}

void decon_reg_config_wb_size(u32 id, struct decon_config *config)
{
	decon_reg_set_blender_bg_size(id, config->mode.dsi_mode,
			config->image_width, config->image_height);
	decon_reg_config_data_path_size(id, config->image_width,
			config->image_height, 0, NULL, &config->dsc,
			config->out_bpc, config->mode.dsi_mode);
}

void decon_reg_set_interrupts(u32 id, u32 en)
{
	u32 val = 0;
	u32 mask;

	decon_reg_clear_int_all(id);

	if (en) {
		if (id == 0)
			val = INT_EN_DQE_DIMMING_END | INT_EN_DQE_DIMMING_START;

		val |= INT_EN_FRAME_DONE | INT_EN_FRAME_START | INT_EN_EXTRA |
			INT_EN;

		decon_write_mask(id, DECON_INT_EN, val, INT_EN_MASK);
		cal_log_debug(id, "interrupt val = %x\n", val);

		val = (INT_EN_RESOURCE_CONFLICT | INT_EN_TIME_OUT);
		decon_write(id, DECON_INT_EN_EXTRA, val);
	} else {
		mask = (INT_EN_EXTRA | INT_EN);
		decon_write_mask(id, DECON_INT_EN, 0, mask);
	}
}

/* opt: 1 = print SEL_SRAM */
static void decon_reg_read_resource_status(u32 id, u32 opt)
{
	u32 i;

	cal_log_warn(id, "decon%d RSC_STATUS_0: SEL_CH  = 0x%x\n",
		id, decon_read(id, RSC_STATUS_0));
	cal_log_warn(id, "decon%d RSC_STATUS_2: SEL_WIN = 0x%x\n",
		id, decon_read(id, RSC_STATUS_2));
	cal_log_warn(id, "decon%d RSC_STATUS_4: SEL_DSC = 0x%x\n",
		id, decon_read(id, RSC_STATUS_4));
	cal_log_warn(id, "decon%d RSC_STATUS_5: SEL_DSCC = 0x%x\n",
		id, decon_read(id, RSC_STATUS_5));

	if (opt) {
		for (i = 0; i < 5; i++) {
			cal_log_warn(id, "decon%d RSC_STATUS_%d = 0x%x\n",
				id, (7 + i),
				decon_read(id, RSC_STATUS_7 + (i * 4)));
		}
	}
}

int decon_reg_get_interrupt_and_clear(u32 id, u32 *ext_irq)
{
	u32 val, val1;
	u32 mask = INT_PEND_DQE_DIMMING_START | INT_PEND_DQE_DIMMING_END |
		INT_PEND_FRAME_DONE | INT_PEND_EXTRA;

	val = decon_read_mask(id, DECON_INT_PEND, mask);
	if (val)
		decon_write(id, DECON_INT_PEND, val);

	if (val & INT_PEND_EXTRA) {
		mask = INT_PEND_RESOURCE_CONFLICT | INT_EN_TIME_OUT;
		val1 = decon_read_mask(id, DECON_INT_PEND_EXTRA, mask);
		decon_write(id, DECON_INT_PEND_EXTRA, val1);
		*ext_irq = val1;

		if (val1 & INT_PEND_RESOURCE_CONFLICT)
			decon_reg_read_resource_status(id, 0);
	}

	return val;
}

int decon_reg_get_fs_interrupt_and_clear(u32 id)
{
	u32 val;

	val = decon_read_mask(id, DECON_INT_PEND, INT_PEND_FRAME_START);
	if (val)
		decon_write(id, DECON_INT_PEND, val);

	return val;
}

/* id: dsim_id */
void decon_reg_set_start_crc(u32 id, u32 en)
{
	if (sub_regs_desc(id)->write_protected) {
		pr_debug("ignored dsim%d crc in protected status\n", id);
		return;
	}

	dsimif_write_mask(id, DSIMIF_CRC_CON(id), en ? ~0 : 0, CRC_START);
}

/* crc_data: [0]=R, [1]=G, [2]=B */
void decon_reg_get_crc_data(u32 id, u32 crc_data[3])
{
	crc_data[0] = dsimif_read(id, DSIMIF_CRC_DATA_R(id));
	crc_data[1] = dsimif_read(id, DSIMIF_CRC_DATA_G(id));
	crc_data[2] = dsimif_read(id, DSIMIF_CRC_DATA_B(id));
}

void __decon_dump(struct drm_printer *p, u32 id, struct decon_regs *regs, bool dsc_en, bool dqe_en)
{
	int i;
	void __iomem *main_regs = regs->regs;
	void __iomem *win_regs = regs->win_regs;
	void __iomem *sub_regs = regs->sub_regs;
	void __iomem *wincon_regs = regs->wincon_regs;

	/* decon_main */
	cal_drm_printf(p, id, "\n=== DECON%d_MAIN SFR DUMP ===\n", id);
	dpu_print_hex_dump(p, main_regs, main_regs + 0x0000, 0x344);
	dpu_print_hex_dump(p, main_regs, main_regs + 0x700, 0x4);
	dpu_print_hex_dump(p, main_regs, main_regs + 0xD00, 0x300);
	/* shadow */
	cal_drm_printf(p, id, "=== DECON%d_MAIN SHADOW SFR DUMP ===\n", id);
	dpu_print_hex_dump(p, main_regs, main_regs + SHADOW_OFFSET, 0x2B0);

	/* decon_win & decon_wincon : 6EA */
	for (i = 0; i < MAX_WIN_PER_DECON; i++) {
		cal_drm_printf(p, id, "\n=== DECON_WIN%d SFR DUMP ===\n", i);
		dpu_print_hex_dump(p, win_regs, win_regs + WIN_OFFSET(i), 0x20);
		cal_drm_printf(p, id, "=== DECON_WINCON%d SFR DUMP ===\n", i);
		dpu_print_hex_dump(p, wincon_regs, wincon_regs + WIN_OFFSET(i),
				0x4);
		/* shadow */
		cal_drm_printf(p, id, "=== DECON_WIN%d SHADOW SFR DUMP ===\n", i);
		dpu_print_hex_dump(p, win_regs,
				win_regs + WIN_OFFSET(i) + SHADOW_OFFSET, 0x20);
		cal_drm_printf(p, id, "=== DECON_WINCON%d SHADOW SFR DUMP ===\n", i);
		dpu_print_hex_dump(p, wincon_regs,
				wincon_regs + WIN_OFFSET(i) + SHADOW_OFFSET,
				0x4);
	}

	/* dsimif : 2EA */
	for (i = 0; i < 2; i++) {
		cal_drm_printf(p, id, "\n=== DECON_SUB.DSIMIF%d SFR DUMP ===\n", i);
		dpu_print_hex_dump(p, sub_regs,
				sub_regs + DSIMIF_OFFSET(i), 0x10);
		dpu_print_hex_dump(p, sub_regs,
				sub_regs + DSIMIF_OFFSET(i) + 0x80, 0x10);
		/* shadow */
		cal_drm_printf(p, id, "= DECON_SUB.DSIMIF%d SHADOW SFR DUMP =\n", i);
		dpu_print_hex_dump(p, sub_regs,
				sub_regs + DSIMIF_OFFSET(i) + SHADOW_OFFSET,
				0x10);
		dpu_print_hex_dump(p, sub_regs,
				sub_regs + DSIMIF_OFFSET(i) + SHADOW_OFFSET +
				0x80, 0x10);
	}

	/* dpif : 2EA */
	for (i = 0; i < 2; i++) {
		cal_drm_printf(p, id, "\n=== DECON_SUB.DPIF%d SFR DUMP ===\n", i);
		dpu_print_hex_dump(p, sub_regs, sub_regs + DPIF_OFFSET(i), 0x4);
		dpu_print_hex_dump(p, sub_regs,
				sub_regs + DPIF_OFFSET(i) + 0x80, 0x10);
		/* shadow */
		cal_drm_printf(p, id, "= DECON_SUB.DPIF%d SHADOW SFR DUMP =\n", i);
		dpu_print_hex_dump(p, sub_regs,
				sub_regs + DPIF_OFFSET(i) + SHADOW_OFFSET, 0x4);
		dpu_print_hex_dump(p, sub_regs,
				sub_regs + DPIF_OFFSET(i) + SHADOW_OFFSET +
				0x80, 0x10);
	}

	/* dsc : 3EA */
	if (dsc_en) {
		for (i = 0; i < 3; i++) {
			cal_drm_printf(p, id, "\n= DECON_SUB.DSC%d SFR DUMP =\n", i);
			dpu_print_hex_dump(p, sub_regs,
					sub_regs + DSC_OFFSET(i), 0x88);
			/* shadow */
			cal_drm_printf(p, id, "=== DECON_SUB.DSC%d SHADOW SFR DUMP ===\n",
					i);
			dpu_print_hex_dump(p, sub_regs,
					sub_regs + DSC_OFFSET(i) +
					SHADOW_OFFSET, 0x88);
		}
	}

	if (dqe_en)
		dqe_dump(p, id);
}

u64 decon_reg_get_rsc_ch(u32 id)
{
	/* L7 does not exist in real HW. SW CHs is CH0 to CH13 map to L0-L6, L8-L14. But
	 * L14 mapping is not connected to RSC_STATUS, so this register can't provide
	 * resource status information for L14.
	 */
	return (decon_read(id, RSC_STATUS_0) & 0xFFFFFFF) |
		(u64)(decon_read(id, RSC_STATUS_1)) << 28;
}

u64 decon_reg_get_rsc_win(u32 id)
{
	return decon_read(id, RSC_STATUS_2) |
		(u64)(decon_read(id, RSC_STATUS_3)) << 32;
}

#ifdef __linux__
int __decon_init_resources(struct decon_device *decon)
{
	struct resource res;
	struct device *dev = decon->dev;
	struct device_node *np = dev->of_node;
	int i, ret = 0;

	i = of_property_match_string(np, "reg-names", "win");
	if (of_address_to_resource(np, i, &res)) {
		cal_log_err(decon->id, "failed to get win resource\n");
		goto err;
	}
	decon->regs.win_regs = ioremap(res.start, resource_size(&res));
	if (IS_ERR(decon->regs.win_regs)) {
		cal_log_err(decon->id, "failed decon win ioremap\n");
		ret = PTR_ERR(decon->regs.win_regs);
		goto err;
	}
	decon_regs_desc_init(decon->regs.win_regs, res.start, "decon_win",
			REGS_DECON_WIN, decon->id);

	i = of_property_match_string(np, "reg-names", "sub");
	if (of_address_to_resource(np, i, &res)) {
		cal_log_err(decon->id, "failed to get sub resource\n");
		goto err_win;
	}
	decon->regs.sub_regs = ioremap(res.start, resource_size(&res));
	if (IS_ERR(decon->regs.sub_regs)) {
		cal_log_err(decon->id, "failed decon sub ioremap\n");
		ret = PTR_ERR(decon->regs.sub_regs);
		goto err_win;
	}
	decon_regs_desc_init(decon->regs.sub_regs, res.start, "decon_sub",
			REGS_DECON_SUB, decon->id);

	i = of_property_match_string(np, "reg-names", "wincon");
	if (of_address_to_resource(np, i, &res)) {
		cal_log_err(decon->id, "failed to get wincon resource\n");
		goto err_sub;
	}
	decon->regs.wincon_regs = ioremap(res.start, resource_size(&res));
	if (IS_ERR(decon->regs.wincon_regs)) {
		cal_log_err(decon->id, "failed wincon ioremap\n");
		ret = PTR_ERR(decon->regs.wincon_regs);
		goto err_sub;
	}
	decon_regs_desc_init(decon->regs.wincon_regs, res.start, "decon_wincon",
			REGS_DECON_WINCON, decon->id);

	return ret;

err_sub:
	iounmap(decon->regs.sub_regs);
err_win:
	iounmap(decon->regs.win_regs);
err:
	return ret;
}

void __decon_unmap_regs(struct decon_device *decon)
{
	iounmap(decon->regs.wincon_regs);
	iounmap(decon->regs.sub_regs);
	iounmap(decon->regs.win_regs);
}
#endif

#define OCCUPIED_BY_DECON(id)	(id)
bool is_decon_using_ch(u32 id, u64 rsc_ch, u32 ch)
{
	return ((rsc_ch >> (ch * 4)) & 0xF) == OCCUPIED_BY_DECON(id);
}

bool is_decon_using_win(u32 id, u64 rsc_win, u32 win)
{
	return ((rsc_win >> (win * 4)) & 0xF) == OCCUPIED_BY_DECON(id);
}

void decon_reg_set_drm_write_protected(u32 id, bool protected)
{
	cal_set_write_protected(sub_regs_desc(id), protected);
}

void decon_reg_update_req_cgc(u32 id)
{
	decon_write_mask(id, SHD_REG_UP_REQ, ~0, SHD_REG_UP_REQ_DQE_CGC);
}

void decon_reg_set_rcd_enable_internal(u32 dqe_id, bool en)
{
	u32 val;

	val = en ? ENHANCE_RCD_ON : 0;
	decon_write_mask(dqe_id, DATA_PATH_CON_0, val, ENHANCE_RCD_ON);
}
