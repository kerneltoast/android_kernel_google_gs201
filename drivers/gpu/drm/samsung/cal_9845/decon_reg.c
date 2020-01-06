// SPDX-License-Identifier: GPL-2.0-only
/*
 * dpu30/cal_9845/decon_reg.c
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
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
#include <regs-decon.h>
#ifdef __linux__
#include <exynos_drm_decon.h>
#endif

enum decon_dsc_id {
	DECON_DSC_ENC0 = 0x0,
	DECON_DSC_ENC1 = 0x1,
	DECON_DSC_ENC2 = 0x2,
};

enum decon_win_alpha_coef {
	BND_COEF_ZERO		  = 0x0,
	BND_COEF_ONE		  = 0x1,
	BND_COEF_AF		  = 0x2,
	BND_COEF_1_M_AF		  = 0x3,
	BND_COEF_AB		  = 0x4,
	BND_COEF_1_M_AB		  = 0x5,
	BND_COEF_PLNAE_ALPHA0	  = 0x6,
	BND_COEF_1_M_PLNAE_ALPHA0 = 0x7,
	BND_COEF_PLNAE_ALPHA1	  = 0x8,
	BND_COEF_1_M_PLNAE_ALPHA1 = 0x9,
	BND_COEF_ALPHA_MULT	  = 0xA,
	BND_COEF_1_M_ALPHA_MULT	  = 0xB,
};

enum decon_win_alpha_sel {
	ALPHA_MULT_SRC_SEL_ALPHA0 = 0,
	ALPHA_MULT_SRC_SEL_ALPHA1 = 1,
	ALPHA_MULT_SRC_SEL_AF = 2,
	ALPHA_MULT_SRC_SEL_AB = 3,
};

static struct cal_regs_desc regs_decon[REGS_DECON_TYPE_MAX][REGS_DECON_ID_MAX];

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
	cal_read(win_regs_desc(0), offset)
#define win_write(id, offset, val)		\
	cal_write(win_regs_desc(0), offset, val)
#define win_read_mask(id, offset, mask)		\
	cal_read_mask(win_regs_desc(0), offset, mask)
#define win_write_mask(id, offset, val, mask)	\
	cal_write_mask(win_regs_desc(0), offset, val, mask)

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
#define dsimif_read(offset)			\
	cal_read(sub_regs_desc(0), offset)
#define dsimif_write(offset, val)		\
	cal_write(sub_regs_desc(0), offset, val)
#define dsimif_read_mask(offset, mask)		\
	cal_read_mask(sub_regs_desc(0), offset, mask)
#define dsimif_write_mask(offset, val, mask)	\
	cal_write_mask(sub_regs_desc(0), offset, val, mask)

#define dpif_read(offset)			\
	cal_read(sub_regs_desc(0), offset)
#define dpif_write(offset, val)			\
	cal_write(sub_regs_desc(0), offset, val)
#define dpif_read_mask(offset, mask)		\
	cal_read_mask(sub_regs_desc(0), offset, mask)
#define dpif_write_mask(offset, val, mask)	\
	cal_write_mask(sub_regs_desc(0), offset, val, mask)

#define dsc_read(offset)			\
	cal_read(sub_regs_desc(0), offset)
#define dsc_write(offset, val)			\
	cal_write(sub_regs_desc(0), offset, val)
#define dsc_read_mask(offset, mask)		\
	cal_read_mask(sub_regs_desc(0), offset, mask)
#define dsc_write_mask(offset, val, mask)	\
	cal_write_mask(sub_regs_desc(0), offset, val, mask)

void decon_regs_desc_init(void __iomem *regs, const char *name,
		enum decon_regs_type type, unsigned int id)
{
	cal_regs_desc_check(type, id, REGS_DECON_TYPE_MAX, REGS_DECON_ID_MAX);
	cal_regs_desc_set(regs_decon, regs, name, type, id);
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
	if (mode == DECON_MIPI_COMMAND_MODE)
		val = GLOBAL_CON_OPERATION_MODE_CMD_F;
	else
		val = GLOBAL_CON_OPERATION_MODE_VIDEO_F;
	decon_write_mask(id, GLOBAL_CON, val, mask);
}

static void decon_reg_direct_on_off(u32 id, u32 en)
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
	decon_write_mask(0, CLOCK_CON, val, mask);
}

static void decon_reg_set_qactive_pll_mode(u32 id, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	/* all unmask */
	mask = CLOCK_CON_QACTIVE_PLL_ON;
	decon_write_mask(0, CLOCK_CON, val, mask);
}

/*
 * Current API does not support to configure various cases fully!
 * Therefore, modify/add configuration cases if necessary
 *
 * [9845 : not sharing]

 * Primary : for Display path
 * Secondary : for Concurrent WB path (no CWB path in DECON2)
 * - Mapping info for RSC_STATUS_7~11(SRAM 0~32)
 * - DECON0 :  0 ~ 12  (if CWB enabled, only 0 ~ 6 are available in OF_SEC)
 * - DECON1 : 13 ~ 25  (if CWB enabled, only 0 ~ 6 are available in OF_SEC)
 * - DECON2 : 26 ~ 32
 */
static void decon_reg_set_sram_enable(u32 id,
		u32 pri[MAX_SRAM_EN_CNT], u32 sec[MAX_SRAM_EN_CNT])
{
	int i, n;
	u32 val1 = 0, val2 = 0;

	n = 0;
	for (i = 0; i < MAX_SRAM_EN_CNT; i++) {
		if (pri[n])
			val1 |= SRAM_EN_ID(n % 8);
		if (sec[n])
			val2 |= SRAM_EN_ID(n % 8);

		n++;
		if (n == 8) {
			/* sram0 ~ 7 */
			decon_write(id, SRAM_EN_OF_PRI_0, val1);
			if (id != 2)
				decon_write(id, SRAM_EN_OF_SEC_0, val2);
			val1 = 0;
			val2 = 0;
		}
	}

	if (id != 2) {
		/* sram8 ~ 12 */
		decon_write(id, SRAM_EN_OF_PRI_1, val1);
		decon_write(id, SRAM_EN_OF_SEC_1, val2);
	}
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

#define OUTIF_DSI0	BIT(0)
#define OUTIF_DSI1	BIT(1)
#define OUTIF_WB	BIT(2)
#define OUTIF_DPIF	BIT(3)
#define COMP_DSC(id)	BIT(id + 4) /* DSC 0,1,2 */
#define COMP_DSCC	BIT(7)
static void decon_reg_set_data_path(u32 id, struct decon_config *cfg)
{
	enum decon_out_type out_type = cfg->out_type;
	u32 dsc_count = cfg->dsc.dsc_count;
	u32 val;

	switch (out_type) {
	case DECON_OUT_DSI0:
		val = OUTIF_DSI0;
		dsimif_write(DSIMIF_SEL(0), SEL_DSIM(id == 1 ? 2 : 0));
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
		dsimif_write(DSIMIF_SEL(1), SEL_DSIM(id == 1 ? 2 : 0));
		break;
	case DECON_OUT_DSI:
		val = OUTIF_DSI0 | OUTIF_DSI1;
		dsimif_write(DSIMIF_SEL(0), SEL_DSIM(0));
		dsimif_write(DSIMIF_SEL(1), SEL_DSIM(1));
		break;
	case DECON_OUT_DP0:
		val = OUTIF_DPIF;
		dpif_write(DPIF_SEL(0), SEL_DP(id));
		break;
	case DECON_OUT_DP1:
		val = OUTIF_DPIF;
		dpif_write(DPIF_SEL(1), SEL_DP(id));
		break;
	case DECON_OUT_WB:
		val = OUTIF_WB;
		break;
	default:
		val = OUTIF_DSI0;
		cal_log_warn(id, "default outif is set(DSI0)\n");
		break;
	}

	if (dsc_count == 2) {
		if (id != 0) {
			cal_log_err(id, "unsupported dsc path\n");
			return;
		}
		val |= COMP_DSCC | COMP_DSC(1) | COMP_DSC(0);
	} else if (dsc_count == 1) {
		if (id > 2) {
			cal_log_err(id, "invalid decon id(%d)\n", id);
			return;
		}
		val |= COMP_DSC(id);
	}

	decon_write_mask(id, DATA_PATH_CON, COMP_OUTIF_PATH_F(val),
			COMP_OUTIF_PATH_MASK);
}

void decon_reg_set_cwb_enable(u32 id, u32 en)
{
	u32 val, mask, d_path;

	val = decon_read(id, DATA_PATH_CON);
	d_path = COMP_OUTIF_PATH_GET(val);

	if (en)
		d_path |= CWB_PATH_EN;
	else
		d_path &= ~CWB_PATH_EN;

	mask = COMP_OUTIF_PATH_MASK;
	decon_write_mask(id, DATA_PATH_CON, d_path, mask);
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

	val = decon_read(id, DATA_PATH_CON);
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
		u32 ds_en[2])
{
	u32 outfifo_w;
	u32 comp_slice_width; /* compressed slice width */

	comp_slice_width = DIV_ROUND_UP(slice_w, 3);

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
static void decon_reg_config_data_path_size(u32 id, u32 width, u32 height,
		u32 overlap_w, struct decon_dsc *p, struct exynos_dsc *dsc)
{
	u32 width_f;
	u32 comp_slice_width; /* compressed slice width */

	/* OUTFIFO */
	if (dsc->enabled) {
		width_f = p->width_per_enc;
		/* OUTFIFO_COMPRESSED_SLICE_WIDTH must be a multiple of 2 */
		comp_slice_width = get_comp_dsc_width(dsc);
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
		decon_reg_set_outfifo_size_ctl0(id, width, height);
	}
}

static void decon_reg_set_bpc(u32 id, u32 bpc)
{
	u32 val = 0, mask;

	if (bpc == 10)
		val = GLOBAL_CON_TEN_BPC_MODE_F;

	mask = GLOBAL_CON_TEN_BPC_MODE_MASK;

	decon_write_mask(id, GLOBAL_CON, val, mask);
}

static void decon_reg_config_win_channel(u32 id, u32 win_idx, int ch)
{
	u32 val, mask;

	val = WIN_CHMAP_F(ch);
	mask = WIN_CHMAP_MASK;
	wincon_write_mask(id, DECON_CON_WIN(win_idx), val, mask);
}

static void decon_reg_configure_trigger(u32 id, enum decon_trig_mode mode)
{
	u32 val, mask;

	mask = HW_TRIG_EN;
	val = (mode == DECON_SW_TRIG) ? 0 : ~0;
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

static void dsc_reg_swreset(u32 dsc_id)
{
	dsc_write_mask(DSC_CONTROL1(dsc_id), 1, DSC_SW_RESET);
}

static void dsc_reg_set_slice_mode_change(u32 dsc_id, u32 en)
{
	u32 val;

	val = DSC_SLICE_MODE_CH_F(en);
	dsc_write_mask(DSC_CONTROL1(dsc_id), val, DSC_SLICE_MODE_CH_MASK);
}

static void dsc_reg_set_dual_slice(u32 dsc_id, u32 en)
{
	u32 val;

	val = DSC_DUAL_SLICE_EN_F(en);
	dsc_write_mask(DSC_CONTROL1(dsc_id), val, DSC_DUAL_SLICE_EN_MASK);
}

/*
 * dsc PPS Configuration
 */

/*
 * APIs which user setting or calculation is required are implemented
 * - PPS04 ~ PPS35 except reserved
 * - PPS58 ~ PPS59
 */
static void dsc_reg_set_pps_06_07_picture_height(u32 dsc_id, u32 height)
{
	u32 val, mask;

	val = PPS06_07_PIC_HEIGHT(height);
	mask = PPS06_07_PIC_HEIGHT_MASK;
	dsc_write_mask(DSC_PPS04_07(dsc_id), val, mask);
}

static void dsc_reg_set_pps_58_59_rc_range_param0(u32 dsc_id, u32 rc_range)
{
	u32 val, mask;

	val = PPS58_59_RC_RANGE_PARAM(rc_range);
	mask = PPS58_59_RC_RANGE_PARAM_MASK;
	dsc_write_mask(DSC_PPS56_59(dsc_id), val, mask);
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

static void dsc_reg_config_control(u32 dsc_id, u32 ds_en, u32 sm_ch,
		u32 slice_width)
{
	u32 val;
	u32 remainder, grpcntline;

	val = DSC_SWAP(0x0, 0x1, 0x0);
	val |= DSC_DUAL_SLICE_EN_F(ds_en);
	val |= DSC_SLICE_MODE_CH_F(sm_ch);
	val |= DSC_FLATNESS_DET_TH_F(0x2);
	dsc_write(DSC_CONTROL0(dsc_id), val);

	remainder = slice_width % 3 ? : 3;
	grpcntline = (slice_width + 2) / 3;

	val = DSC_REMAINDER_F(remainder) | DSC_GRPCNTLINE_F(grpcntline);
	dsc_write(DSC_CONTROL3(dsc_id), val);
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
	const u32 initial_xmit_delay = 0x200;
	const u32 initial_dec_delay = 0x4c0;
	/* when 'slice_w >= 70' */
	const u32 initial_scale_value = 0x20;
	const u32 first_line_bpg_offset = 0x0c;
	const u32 initial_offset = 0x1800;
	const u32 rc_range_parameters = 0x0102;

	u32 final_offset, final_scale;
	u32 flag, nfl_bpg_offset, slice_bpg_offset;
	u32 scale_increment_interval, scale_decrement_interval;
	u32 slice_width_byte_unit, comp_slice_width_byte_unit;
	u32 comp_slice_width_pixel_unit;
	u32 overlap_w = 0;
	u32 dsc_enc0_w = 0, dsc_enc0_h;
	u32 dsc_enc1_w = 0, dsc_enc1_h;
	u32 i, j;

	width = config->image_width;
	height = config->image_height;

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
		dsc_enc0_h = pic_height;
		if (dscc_en) {
			dsc_enc1_w = comp_slice_width_pixel_unit;
			dsc_enc1_h = pic_height;
		}
	} else if (i == 0 && j != 0) {
		dsc_enc0_w = comp_slice_width_pixel_unit + 1;
		dsc_enc0_h = pic_height;
		if (dscc_en) {
			dsc_enc1_w = comp_slice_width_pixel_unit + 1;
			dsc_enc1_h = pic_height;
		}
	} else if (i != 0) {
		while (1) {
			comp_slice_width_pixel_unit++;
			j = comp_slice_width_pixel_unit % 2;
			if (j == 0)
				break;
		}
		dsc_enc0_w = comp_slice_width_pixel_unit;
		dsc_enc0_h = pic_height;
		if (dscc_en) {
			dsc_enc1_w = comp_slice_width_pixel_unit;
			dsc_enc1_h = pic_height;
		}
	}

	if (dual_slice_en) {
		dsc_enc0_w = dsc_enc0_w * 2;
		if (dscc_en)
			dsc_enc1_w = dsc_enc1_w * 2;
	}

	/* Save information to structure variable */
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

static void dsc_reg_set_pps(u32 dsc_id, struct decon_dsc *dsc_enc)
{
	u32 val;
	u32 initial_dec_delay;

	val = PPS04_COMP_CFG(dsc_enc->comp_cfg);
	val |= PPS05_BPP(dsc_enc->bit_per_pixel);
	val |= PPS06_07_PIC_HEIGHT(dsc_enc->pic_height);
	dsc_write(DSC_PPS04_07(dsc_id), val);

	val = PPS08_09_PIC_WIDHT(dsc_enc->pic_width);
	val |= PPS10_11_SLICE_HEIGHT(dsc_enc->slice_height);
	dsc_write(DSC_PPS08_11(dsc_id), val);

	val = PPS12_13_SLICE_WIDTH(dsc_enc->slice_width);
	val |= PPS14_15_CHUNK_SIZE(dsc_enc->chunk_size);
	dsc_write(DSC_PPS12_15(dsc_id), val);

#ifndef VESA_SCR_V4
	initial_dec_delay = 0x01B4;
#else
	initial_dec_delay = dsc_enc->initial_dec_delay;
#endif
	val = PPS18_19_INIT_DEC_DELAY(initial_dec_delay);
	val |= PPS16_17_INIT_XMIT_DELAY(dsc_enc->initial_xmit_delay);
	dsc_write(DSC_PPS16_19(dsc_id), val);

	val = PPS21_INIT_SCALE_VALUE(dsc_enc->initial_scale_value);
	val |= PPS22_23_SCALE_INC_INTERVAL(dsc_enc->scale_increment_interval);
	dsc_write(DSC_PPS20_23(dsc_id), val);

	val = PPS24_25_SCALE_DEC_INTERVAL(dsc_enc->scale_decrement_interval);
	val |= PPS27_FL_BPG_OFFSET(dsc_enc->first_line_bpg_offset);
	dsc_write(DSC_PPS24_27(dsc_id), val);

	val = PPS28_29_NFL_BPG_OFFSET(dsc_enc->nfl_bpg_offset);
	val |= PPS30_31_SLICE_BPG_OFFSET(dsc_enc->slice_bpg_offset);
	dsc_write(DSC_PPS28_31(dsc_id), val);

	val = PPS32_33_INIT_OFFSET(dsc_enc->initial_offset);
	val |= PPS34_35_FINAL_OFFSET(dsc_enc->final_offset);
	dsc_write(DSC_PPS32_35(dsc_id), val);

	/* min_qp0 = 0 , max_qp0 = 4 , bpg_off0 = 2 */
	dsc_reg_set_pps_58_59_rc_range_param0(dsc_id,
		dsc_enc->rc_range_parameters);

#ifndef VESA_SCR_V4
	/* PPS79 ~ PPS87 : 3HF4 is different with VESA SCR v4 */
	dsc_write(DSC_PPS76_79(dsc_id), 0x1AB62AF6);
	dsc_write(DSC_PPS80_83(dsc_id), 0x2B342B74);
	dsc_write(DSC_PPS84_87(dsc_id), 0x3B746BF4);
#endif
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

static void dsc_reg_set_partial_update(u32 dsc_id, u32 dual_slice_en,
	u32 slice_mode_ch, u32 pic_h)
{
	/*
	 * Following SFRs must be considered
	 * - dual_slice_en
	 * - slice_mode_change
	 * - picture_height
	 * - picture_width (don't care @KC) : decided by DSI (-> dual: /2)
	 */
	dsc_reg_set_dual_slice(dsc_id, dual_slice_en);
	dsc_reg_set_slice_mode_change(dsc_id, slice_mode_ch);
	dsc_reg_set_pps_06_07_picture_height(dsc_id, pic_h);
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
		dsc_reg_config_control(DECON_DSC_ENC1, ds_en, sm_ch,
				dsc_enc->slice_width);
		dsc_reg_set_pps(DECON_DSC_ENC1, dsc_enc);
	} else if (id == 2) {	/* only for DP */
		dsc_reg_config_control(DECON_DSC_ENC2, ds_en, sm_ch,
				dsc_enc->slice_width);
		dsc_reg_set_pps(DECON_DSC_ENC2, dsc_enc);
	} else {
		for (dsc_id = 0; dsc_id < config->dsc.dsc_count; dsc_id++) {
			dsc_reg_config_control(dsc_id, ds_en, sm_ch,
					dsc_enc->slice_width);
			dsc_reg_set_pps(dsc_id, dsc_enc);
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
			dsc_reg_swreset(dsc_id);
	}

	dsc_enc.overlap_w = overlap_w;
	dsc_reg_set_encoder(id, config, &dsc_enc, 0);
	decon_reg_config_data_path_size(id, dsc_enc.width_per_enc,
			config->image_height, overlap_w, &dsc_enc,
			&config->dsc);

	return 0;
}

static void decon_reg_clear_int_all(u32 id)
{
	u32 mask;

	mask = (INT_EN_FRAME_DONE
			| INT_EN_FRAME_START);
	decon_write_mask(id, DECON_INT_PEND, ~0, mask);

	mask = (INT_EN_RESOURCE_CONFLICT | INT_EN_TIME_OUT);
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
				config->image_height, overlap_w, NULL,
				&config->dsc);

		/* TODO: if 10 BPC is supported, this will be changed later */
		if (id == 2)
			decon_reg_set_bpc(id, 8);
	}

	decon_reg_per_frame_off(id);
}

static void decon_reg_set_blender_bg_size(u32 id, enum decon_dsi_mode dsi_mode,
		u32 bg_w, u32 bg_h)
{
	u32 width, val;

	width = bg_w;
	if (dsi_mode == DSI_MODE_DUAL_DSI)
		width = width * 2;

	val = BLENDER_BG_HEIGHT_F(bg_h) | BLENDER_BG_WIDTH_F(width);
	decon_write(id, BLD_BG_IMG_SIZE_PRI, val);
}

static int decon_reg_stop_perframe_dsi(u32 id, struct decon_config *config,
		u32 fps)
{
	int ret = 0;
	/* timeout_us : 1000000us / fps + 50% margin */
	const int timeout_us = DIV_ROUND_UP(USEC_PER_SEC * 15, fps * 10);

	cal_log_debug(id, "%s +\n", __func__);

	if ((config->mode.op_mode == DECON_MIPI_COMMAND_MODE) &&
			(config->mode.trig_mode == DECON_HW_TRIG))
		decon_reg_set_trigger(id, &config->mode, DECON_TRIG_MASK);

	/* perframe stop */
	decon_reg_per_frame_off(id);
	decon_reg_update_req_global(id);

	ret = decon_reg_wait_run_is_off_timeout(id, timeout_us);

	cal_log_debug(id, "%s -\n", __func__);
	return ret;
}

#if defined(CONFIG_EXYNOS_DISPLAYPORT)
static int decon_reg_stop_perframe_dp(u32 id, u32 fps)
{
	int ret = 0;
	/* timeout_us : 1000000us / fps + 50% margin */
	const int timeout_us = DIV_ROUND_UP(USEC_PER_SEC * 15, fps * 10);
	u32 sst_id = SST1;

	cal_log_debug(id, "%s +\n", __func__);

	/* perframe stop */
	decon_reg_per_frame_off(id);
	decon_reg_update_req_global(id);

	ret = decon_reg_wait_run_is_off_timeout(id, timeout_us);

	sst_id = displayport_get_sst_id_with_decon_id(id);
	displayport_reg_lh_p_ch_power(sst_id, 0);

	cal_log_debug(id, "%s -\n", __func__);
	return ret;
}
#else
static inline int decon_reg_stop_perframe_dp(u32 id, u32 fps) { return 0; }
#endif

static int decon_reg_stop_inst_dsi(u32 id, struct decon_config *config, u32 fps)
{
	int ret = 0;
	const int timeout_us = DIV_ROUND_UP(USEC_PER_SEC * 15, fps * 10);

	cal_log_debug(id, "%s +\n", __func__);

	if ((config->mode.op_mode == DECON_MIPI_COMMAND_MODE) &&
			(config->mode.trig_mode == DECON_HW_TRIG))
		decon_reg_set_trigger(id, &config->mode, DECON_TRIG_MASK);

	/* instant stop */
	decon_reg_direct_on_off(id, 0);
	decon_reg_update_req_global(id);

	ret = decon_reg_wait_run_is_off_timeout(id, timeout_us);

	cal_log_debug(id, "%s -\n", __func__);
	return ret;
}

#if defined(CONFIG_EXYNOS_DISPLAYPORT)
static int decon_reg_stop_inst_dp(u32 id, u32 fps)
{
	int ret = 0;
	const int timeout_us = DIV_ROUND_UP(USEC_PER_SEC * 15, fps * 10);
	u32 sst_id = SST1;

	cal_log_debug(id, "%s +\n", __func__);

	/* instant stop */
	decon_reg_direct_on_off(id, 0);
	decon_reg_update_req_global(id);

	sst_id = displayport_get_sst_id_with_decon_id(id);
	displayport_reg_lh_p_ch_power(sst_id, 0);

	ret = decon_reg_wait_run_is_off_timeout(id, timeout_us);

	cal_log_debug(id, "%s -\n", __func__);
	return ret;
}
#else
static inline int decon_reg_stop_inst_dp(u32 id, u32 fps)
{
	return 0;
}
#endif

void decon_reg_set_win_enable(u32 id, u32 win_idx, u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	mask = _WIN_EN_F;
	wincon_write_mask(id, DECON_CON_WIN(win_idx), val, mask);
	cal_log_debug(id, "%s: 0x%x\n", __func__,
			wincon_read(id, DECON_CON_WIN(win_idx)));
}

/*
 * argb_color : 32-bit
 * A[31:24] - R[23:16] - G[15:8] - B[7:0]
 */
static void decon_reg_set_win_mapcolor(u32 id, u32 win_idx, u32 argb_color)
{
	u32 val, mask;
	u32 mc_alpha = 0, mc_red = 0;
	u32 mc_green = 0, mc_blue = 0;

	mc_alpha = (argb_color >> 24) & 0xFF;
	mc_red = (argb_color >> 16) & 0xFF;
	mc_green = (argb_color >> 8) & 0xFF;
	mc_blue = (argb_color >> 0) & 0xFF;

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

static void decon_reg_set_winmap(u32 id, u32 win_idx, u32 color, u32 en)
{
	u32 val, mask;

	/* Enable */
	val = en ? ~0 : 0;
	mask = WIN_MAPCOLOR_EN_F;
	wincon_write_mask(id, DECON_CON_WIN(win_idx), val, mask);
	cal_log_debug(id, "%s: 0x%x\n", __func__,
			wincon_read(id, DECON_CON_WIN(win_idx)));

	/* Color Set */
	decon_reg_set_win_mapcolor(id, win_idx, color);
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
	u8 alpha0 = 0xff;
	u8 alpha1 = 0xff;
	bool is_plane_a = false;
	u32 af_d = BND_COEF_ONE, ab_d = BND_COEF_ZERO,
		af_a = BND_COEF_ONE, ab_a = BND_COEF_ZERO;

	if (blend == DECON_BLENDING_NONE)
		pd_func = PD_FUNC_COPY;

	if ((plane_a >= 0) && (plane_a <= 0xff)) {
		alpha0 = plane_a;
		alpha1 = 0;
		is_plane_a = true;
	}

	if ((blend == DECON_BLENDING_COVERAGE) && !is_plane_a) {
		af_d = BND_COEF_AF;
		ab_d = BND_COEF_1_M_AF;
		af_a = BND_COEF_AF;
		ab_a = BND_COEF_1_M_AF;
	} else if ((blend == DECON_BLENDING_COVERAGE) && is_plane_a) {
		af_d = BND_COEF_ALPHA_MULT;
		ab_d = BND_COEF_1_M_ALPHA_MULT;
		af_a = BND_COEF_ALPHA_MULT;
		ab_a = BND_COEF_1_M_ALPHA_MULT;
	} else if ((blend == DECON_BLENDING_PREMULT) && !is_plane_a) {
		af_d = BND_COEF_ONE;
		ab_d = BND_COEF_1_M_AF;
		af_a = BND_COEF_ONE;
		ab_a = BND_COEF_1_M_AF;
	} else if ((blend == DECON_BLENDING_PREMULT) && is_plane_a) {
		af_d = BND_COEF_PLNAE_ALPHA0;
		ab_d = BND_COEF_1_M_ALPHA_MULT;
		af_a = BND_COEF_PLNAE_ALPHA0;
		ab_a = BND_COEF_1_M_ALPHA_MULT;
	} else if (blend == DECON_BLENDING_NONE) {
		cal_log_debug(id, "none blending mode\n");
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

/******************** EXPORTED DECON CAL APIs ********************/
/* TODO: maybe this function will be moved to internal DECON CAL function */
void decon_reg_update_req_global(u32 id)
{
	decon_write_mask(id, SHD_REG_UP_REQ, ~0, SHD_REG_UP_REQ_GLOBAL);
}

int decon_reg_init(u32 id, struct decon_config *config)
{
	/* for bring-up */
	u32 pri_sram[3][MAX_SRAM_EN_CNT] = {
		{1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* decon0 : 13 */
		{1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* decon1 : 13 */
		{1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* decon2 : 7 */
	};
	u32 sec_sram[3][MAX_SRAM_EN_CNT] = {
		{1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* decon0 : 13 */
		{1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* decon1 : 13 */
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* decon2 : none */
	};

	decon_reg_set_clkgate_mode(id, 0);

	if (config->out_type & DECON_OUT_DP)
		decon_reg_set_qactive_pll_mode(id, 1);

	decon_reg_set_sram_enable(id, pri_sram[id], sec_sram[id]);

	decon_reg_set_operation_mode(id, config->mode.op_mode);

	decon_reg_set_blender_bg_size(id, config->mode.dsi_mode,
			config->image_width, config->image_height);

	/* enable once at init time */
	decon_reg_set_latency_monitor_enable(id, 1);

	if (id == 2) {
		/* Set a TRIG mode */
		/* This code is for only DECON 2 s/w trigger mode */
		decon_reg_configure_trigger(id, config->mode.trig_mode);
		decon_reg_configure_lcd(id, config);
	} else {
		decon_reg_configure_lcd(id, config);
		if (config->mode.op_mode == DECON_MIPI_COMMAND_MODE) {
#if defined(CONFIG_EXYNOS_EWR)
			/* 60fps: request wakeup at 16.566ms after TE rising */
			/* TODO: 60 will be changed to fps value */
			decon_reg_set_ewr_control(id,
				decon_get_ewr_cycle(60, 100), 1);
#endif
			decon_reg_set_trigger(id, &config->mode,
					DECON_TRIG_MASK);
		}
	}

	/* asserted interrupt should be cleared before initializing decon hw */
	decon_reg_clear_int_all(id);

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
	/* TODO : register for outfifo2 doesn't exist, needs a confirm */
	if (config->mode.op_mode == DECON_MIPI_COMMAND_MODE &&
			config->mode.dsi_mode != DSI_MODE_DUAL_DSI)
		decon_reg_set_pll_sleep(id, 1);
#endif

	return 0;
}

int decon_reg_start(u32 id, struct decon_config *config)
{
	int ret = 0;
#if defined(CONFIG_EXYNOS_DISPLAYPORT)
	u32 sst_id = SST1;

	if (config->out_type & DECON_OUT_DP) {
		sst_id = displayport_get_sst_id_with_decon_id(id);
		displayport_reg_lh_p_ch_power(sst_id, 1);
	}
#endif

	decon_reg_direct_on_off(id, 1);
	decon_reg_update_req_global(id);

	/*
	 * DECON goes to run-status as soon as
	 * request shadow update without HW_TE
	 */
	ret = decon_reg_wait_run_status_timeout(id, 2 * 1000); /* timeout 2ms */

	/* wait until run-status, then trigger */
	if (config->mode.op_mode == DECON_MIPI_COMMAND_MODE)
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
	if (config->mode.op_mode == DECON_MIPI_COMMAND_MODE &&
			config->mode.dsi_mode != DSI_MODE_DUAL_DSI)
		decon_reg_set_pll_wakeup(id, 1);
#endif

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
			cal_log_err(id, "failed to perframe_stop\n");
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

void decon_reg_set_window_control(u32 id, int win_idx,
		struct decon_window_regs *regs, u32 winmap_en)
{
	cal_log_debug(id, "win id = %d\n", win_idx);
	decon_reg_set_win_bnd_function(0, win_idx, regs);
	win_write(id, WIN_START_POSITION(win_idx), regs->start_pos);
	win_write(id, WIN_END_POSITION(win_idx), regs->end_pos);
	win_write(id, WIN_START_TIME_CON(win_idx), regs->start_time);
	decon_reg_set_winmap(id, win_idx, regs->colormap, winmap_en);

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

	if (mode->op_mode == DECON_MIPI_COMMAND_MODE)
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
		cal_log_err(id, "timeout of updating decon registers\n");
		return ret;
	}

	return 0;
}

int decon_reg_wait_update_done_and_mask(u32 id, struct decon_mode *mode,
		u32 timeout_us)
{
	int result;

	result = decon_reg_wait_update_done_timeout(id, timeout_us);

	if (mode->op_mode == DECON_MIPI_COMMAND_MODE)
		decon_reg_set_trigger(id, mode, DECON_TRIG_MASK);

	return result;
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
		dsc_reg_set_partial_update(id, dual_slice_en[0],
				slice_mode_ch[0], partial_h);
		if (config->dsc.dsc_count == 2)
			dsc_reg_set_partial_update(1, dual_slice_en[1],
					slice_mode_ch[1], partial_h);
	}

	decon_reg_set_data_path_size(id, partial_w, partial_h,
			config->dsc.enabled, config->dsc.dsc_count,
			config->dsc.slice_width, config->dsc.slice_height,
			dual_slice_en);
}

void decon_reg_set_mres(u32 id, struct decon_config *config)
{
	u32 overlap_w = 0;

	if (config->mode.op_mode != DECON_MIPI_COMMAND_MODE) {
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
				config->image_height, overlap_w, NULL,
				&config->dsc);
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
			config->image_height, 0, NULL, &config->dsc);
}

void decon_reg_set_interrupts(u32 id, u32 en)
{
	u32 val, mask;

	decon_reg_clear_int_all(id);

	if (en) {
		val = (INT_EN_FRAME_DONE
			| INT_EN_FRAME_START
			| INT_EN_EXTRA
			| INT_EN);

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
	u32 reg_id;

	reg_id = DECON_INT_PEND;
	val = decon_read(id, reg_id);

	if (val & INT_PEND_FRAME_START)
		decon_write(id, reg_id, INT_PEND_FRAME_START);

	if (val & INT_PEND_FRAME_DONE)
		decon_write(id, reg_id, INT_PEND_FRAME_DONE);

	if (val & INT_PEND_EXTRA) {
		decon_write(id, reg_id, INT_PEND_EXTRA);

		reg_id = DECON_INT_PEND_EXTRA;
		val1 = decon_read(id, reg_id);
		*ext_irq = val1;

		if (val1 & INT_PEND_RESOURCE_CONFLICT) {
			decon_write(id, reg_id, INT_PEND_RESOURCE_CONFLICT);
			decon_reg_read_resource_status(id, 0);
		}

		if (val1 & INT_EN_TIME_OUT)
			decon_write(id, reg_id, INT_EN_TIME_OUT);
	}

	return val;
}

/* id: dsim_id */
void decon_reg_set_start_crc(u32 id, u32 en)
{
	dsimif_write_mask(DSIMIF_CRC_CON(id), en ? ~0 : 0, CRC_START);
}

/* crc_data: [0]=R, [1]=G, [2]=B */
void decon_reg_get_crc_data(u32 id, u32 crc_data[3])
{
	crc_data[0] = dsimif_read(DSIMIF_CRC_DATA_R(id));
	crc_data[1] = dsimif_read(DSIMIF_CRC_DATA_G(id));
	crc_data[2] = dsimif_read(DSIMIF_CRC_DATA_B(id));
}

void __decon_dump(u32 id, struct decon_regs *regs, bool dsc_en)
{
	int i;
	void __iomem *main_regs = regs->regs;
	void __iomem *win_regs = regs->win_regs;
	void __iomem *sub_regs = regs->sub_regs;
	void __iomem *wincon_regs = regs->wincon_regs;

	/* decon_main */
	cal_log_info(id, "\n=== DECON%d_MAIN SFR DUMP ===\n", id);
	dpu_print_hex_dump(main_regs, main_regs + 0x0000, 0x344);
	/* shadow */
	cal_log_info(id, "=== DECON%d_MAIN SHADOW SFR DUMP ===\n", id);
	dpu_print_hex_dump(main_regs, main_regs + SHADOW_OFFSET, 0x2B0);

	/* decon_win & decon_wincon : 6EA */
	for (i = 0; i < 6; i++) {
		cal_log_info(id, "\n=== DECON_WIN%d SFR DUMP ===\n", i);
		dpu_print_hex_dump(win_regs, win_regs + WIN_OFFSET(i), 0x20);
		cal_log_info(id, "=== DECON_WINCON%d SFR DUMP ===\n", i);
		dpu_print_hex_dump(wincon_regs, wincon_regs + WIN_OFFSET(i),
				0x4);
		/* shadow */
		cal_log_info(id, "=== DECON_WIN%d SHADOW SFR DUMP ===\n", i);
		dpu_print_hex_dump(win_regs,
				win_regs + WIN_OFFSET(i) + SHADOW_OFFSET, 0x20);
		cal_log_info(id, "=== DECON_WINCON%d SHADOW SFR DUMP ===\n", i);
		dpu_print_hex_dump(wincon_regs,
				wincon_regs + WIN_OFFSET(i) + SHADOW_OFFSET,
				0x4);
	}

	/* dsimif : 2EA */
	for (i = 0; i < 2; i++) {
		cal_log_info(id, "\n=== DECON_SUB.DSIMIF%d SFR DUMP ===\n", i);
		dpu_print_hex_dump(sub_regs,
				sub_regs + DSIMIF_OFFSET(i), 0x10);
		dpu_print_hex_dump(sub_regs,
				sub_regs + DSIMIF_OFFSET(i) + 0x80, 0x10);
		/* shadow */
		cal_log_info(id, "= DECON_SUB.DSIMIF%d SHADOW SFR DUMP =\n", i);
		dpu_print_hex_dump(sub_regs,
				sub_regs + DSIMIF_OFFSET(i) + SHADOW_OFFSET,
				0x10);
		dpu_print_hex_dump(sub_regs,
				sub_regs + DSIMIF_OFFSET(i) + SHADOW_OFFSET +
				0x80, 0x10);
	}

	/* dpif : 2EA */
	for (i = 0; i < 2; i++) {
		cal_log_info(id, "\n=== DECON_SUB.DPIF%d SFR DUMP ===\n", i);
		dpu_print_hex_dump(sub_regs, sub_regs + DPIF_OFFSET(i), 0x4);
		dpu_print_hex_dump(sub_regs,
				sub_regs + DPIF_OFFSET(i) + 0x80, 0x10);
		/* shadow */
		cal_log_info(id, "= DECON_SUB.DPIF%d SHADOW SFR DUMP =\n", i);
		dpu_print_hex_dump(sub_regs,
				sub_regs + DPIF_OFFSET(i) + SHADOW_OFFSET, 0x4);
		dpu_print_hex_dump(sub_regs,
				sub_regs + DPIF_OFFSET(i) + SHADOW_OFFSET +
				0x80, 0x10);
	}

	/* dsc : 3EA */
	if (dsc_en) {
		for (i = 0; i < 3; i++) {
			cal_log_info(id, "\n= DECON_SUB.DSC%d SFR DUMP =\n", i);
			dpu_print_hex_dump(sub_regs,
					sub_regs + DSC_OFFSET(i), 0x88);
			/* shadow */
			cal_log_info(id, "=== DECON_SUB.DSC%d SHADOW SFR DUMP ===\n",
					i);
			dpu_print_hex_dump(sub_regs,
					sub_regs + DSC_OFFSET(i) +
					SHADOW_OFFSET, 0x88);
		}
	}
}

u32 decon_reg_get_rsc_ch(u32 id)
{
	return decon_read(id, RSC_STATUS_0);
}

u32 decon_reg_get_rsc_win(u32 id)
{
	return decon_read(id, RSC_STATUS_2);
}

#ifdef __linux__
int __decon_init_resources(struct decon_device *decon)
{
	struct resource *res;
	struct device *dev = decon->dev;
	struct platform_device *pdev;

	pdev = container_of(dev, struct platform_device, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	decon->regs.win_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(decon->regs.win_regs)) {
		cal_log_err(decon->id, "failed decon win ioremap\n");
		return PTR_ERR(decon->regs.win_regs);
	}
	decon_regs_desc_init(decon->regs.win_regs, "decon_win",
			REGS_DECON_WIN, decon->id);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	decon->regs.sub_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(decon->regs.sub_regs)) {
		cal_log_err(decon->id, "failed decon sub ioremap\n");
		return PTR_ERR(decon->regs.sub_regs);
	}
	decon_regs_desc_init(decon->regs.sub_regs, "decon_sub",
			REGS_DECON_SUB, decon->id);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	decon->regs.wincon_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(decon->regs.wincon_regs)) {
		cal_log_err(decon->id, "failed wincon ioremap\n");
		return PTR_ERR(decon->regs.wincon_regs);
	}
	decon_regs_desc_init(decon->regs.wincon_regs, "decon_wincon",
			REGS_DECON_WINCON, decon->id);

	return 0;
}
#endif
