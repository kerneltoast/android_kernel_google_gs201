// SPDX-License-Identifier: GPL-2.0-only
/*
 * linux/drivers/gpu/drm/samsung/cal_9820/dpp_reg.c
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung EXYNOS9 SoC series Display Pre Processor driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <video/exynos_hdr_tunables.h>

#include <exynos_hdr_lut.h>
#include <exynos_dpp_coef.h>
#include <exynos_drm_format.h>
#include <exynos_drm_dpp.h>
#include <cal_config.h>
#include <dpp_cal.h>
#include <regs-dpp.h>

#define DPP_SC_RATIO_MAX	((1 << 20) * 8 / 8)
#define DPP_SC_RATIO_7_8	((1 << 20) * 8 / 7)
#define DPP_SC_RATIO_6_8	((1 << 20) * 8 / 6)
#define DPP_SC_RATIO_5_8	((1 << 20) * 8 / 5)
#define DPP_SC_RATIO_4_8	((1 << 20) * 8 / 4)
#define DPP_SC_RATIO_3_8	((1 << 20) * 8 / 3)

static struct cal_regs_desc regs_dpp[REGS_DPP_TYPE_MAX][REGS_DPP_ID_MAX];

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

#define dma_com_regs_desc(id)			(&regs_dpp[REGS_DMA_COMMON][0])
#define dma_com_read(id, offset)			\
	cal_read(dma_com_regs_desc(id), offset)
#define dma_com_write(id, offset, val)		\
	cal_write(dma_com_regs_desc(id), offset, val)
#define dma_com_read_mask(id, offset, mask)	\
	cal_read_mask(dma_com_regs_desc(id), offset, mask)
#define dma_com_write_mask(id, offset, val, mask)	\
	cal_write_mask(dma_com_regs_desc(id), offset, val, mask)

void dpp_regs_desc_init(void __iomem *regs, const char *name,
		enum dpp_regs_type type, unsigned int id)
{
	cal_regs_desc_check(type, id, REGS_DPP_TYPE_MAX, REGS_DPP_ID_MAX);
	cal_regs_desc_set(regs_dpp, regs, name, type, id);
}

/****************** IDMA CAL functions ******************/
static void idma_reg_set_irq_mask_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, IDMA_IRQ, val, IDMA_ALL_IRQ_MASK);
}

static void idma_reg_set_irq_enable(u32 id)
{
	dma_write_mask(id, IDMA_IRQ, ~0, IDMA_IRQ_ENABLE);
}

static void idma_reg_set_clock_gate_en_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, IDMA_ENABLE, val, IDMA_ALL_CLOCK_GATE_EN_MASK);
}

static void idma_reg_set_in_qos_lut(u32 id, u32 lut_id, u32 qos_t)
{
	u32 reg_id;

	if (lut_id == 0)
		reg_id = DPU_DMA_QOS_LUT07_00;
	else
		reg_id = DPU_DMA_QOS_LUT15_08;
	dma_com_write(id, reg_id, qos_t);
}

static void idma_reg_set_dynamic_gating_en_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, IDMA_DYNAMIC_GATING_EN, val, IDMA_DG_EN_ALL);
}

static void idma_reg_set_out_frame_alpha(u32 id, u32 alpha)
{
	dma_write_mask(id, IDMA_OUT_CON, IDMA_OUT_FRAME_ALPHA(alpha),
			IDMA_OUT_FRAME_ALPHA_MASK);
}

static void idma_reg_clear_irq(u32 id, u32 irq)
{
	dma_write_mask(id, IDMA_IRQ, ~0, irq);
}

static void idma_reg_set_sw_reset(u32 id)
{
	dma_write_mask(id, IDMA_ENABLE, ~0, IDMA_SRESET);
}

static int idma_reg_wait_sw_reset_status(u32 id)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(dma_regs_desc(id)->regs + IDMA_ENABLE,
			val, !(val & IDMA_SRESET), 10, 2000); /* timeout 2ms */
	if (ret) {
		cal_log_err(id, "[idma] timeout sw-reset\n");
		return ret;
	}

	return 0;
}

static void idma_reg_set_coordinates(u32 id, struct decon_frame *src)
{
	dma_write(id, IDMA_SRC_OFFSET,
			IDMA_SRC_OFFSET_Y(src->y) | IDMA_SRC_OFFSET_X(src->x));
	dma_write(id, IDMA_SRC_SIZE,
			IDMA_SRC_HEIGHT(src->f_h) | IDMA_SRC_WIDTH(src->f_w));
	dma_write(id, IDMA_IMG_SIZE,
			IDMA_IMG_HEIGHT(src->h) | IDMA_IMG_WIDTH(src->w));
}

static void idma_reg_set_rotation(u32 id, u32 rot)
{
	dma_write_mask(id, IDMA_IN_CON, IDMA_ROTATION(rot), IDMA_ROTATION_MASK);
}

static void idma_reg_set_block_mode(u32 id, bool en, int x, int y, u32 w, u32 h)
{
	if (!en) {
		dma_write_mask(id, IDMA_IN_CON, 0, IDMA_BLOCK_EN);
		return;
	}

	dma_write(id, IDMA_BLOCK_OFFSET,
			IDMA_BLK_OFFSET_Y(y) | IDMA_BLK_OFFSET_X(x));
	dma_write(id, IDMA_BLOCK_SIZE, IDMA_BLK_HEIGHT(h) | IDMA_BLK_WIDTH(w));
	dma_write_mask(id, IDMA_IN_CON, ~0, IDMA_BLOCK_EN);

	cal_log_debug(id, "dpp%d: block x(%d) y(%d) w(%d) h(%d)\n",
			id, x, y, w, h);
}

static void idma_reg_set_format(u32 id, u32 fmt)
{
	dma_write_mask(id, IDMA_IN_CON, IDMA_IMG_FORMAT(fmt),
			IDMA_IMG_FORMAT_MASK);
}

#if defined(DMA_BIST)
static void idma_reg_set_test_pattern(u32 id, u32 pat_id, u32 *pat_dat)
{
	dma_write_mask(id, IDMA_IN_REQ_DEST, ~0, IDMA_IN_REG_DEST_SEL_MASK);

	if (pat_id == 0) {
		dma_com_write(id, DPU_DMA_TEST_PATTERN0_0, pat_dat[0]);
		dma_com_write(id, DPU_DMA_TEST_PATTERN0_1, pat_dat[1]);
		dma_com_write(id, DPU_DMA_TEST_PATTERN0_2, pat_dat[2]);
		dma_com_write(id, DPU_DMA_TEST_PATTERN0_3, pat_dat[3]);
	} else {
		dma_com_write(id, DPU_DMA_TEST_PATTERN1_0, pat_dat[4]);
		dma_com_write(id, DPU_DMA_TEST_PATTERN1_1, pat_dat[5]);
		dma_com_write(id, DPU_DMA_TEST_PATTERN1_2, pat_dat[6]);
		dma_com_write(id, DPU_DMA_TEST_PATTERN1_3, pat_dat[7]);
	}
}
#endif

static void idma_reg_set_afbc(u32 id, bool en, u32 rcv_num)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, IDMA_IN_CON, val, IDMA_AFBC_EN);
	dma_write_mask(id, IDMA_RECOVERY_CTRL, val, IDMA_RECOVERY_EN);
	dma_com_write_mask(id, DPU_DMA_RECOVERY_NUM_CTRL,
			DPU_DMA_RECOVERY_NUM(rcv_num),
			DPU_DMA_RECOVERY_NUM_MASK);
}

static void idma_reg_print_irqs_msg(u32 id, u32 irqs)
{
	u32 cfg_err;

	if (irqs & IDMA_AFBC_CONFLICT_IRQ)
		cal_log_err(id, "IDMA AFBC conflict irq occur\n");

	if (irqs & IDMA_AFBC_TIMEOUT_IRQ)
		cal_log_err(id, "IDMA AFBC timeout irq occur\n");

	if (irqs & IDMA_READ_SLAVE_ERROR)
		cal_log_err(id, "IDMA read slave error irq occur\n");

	if (irqs & IDMA_STATUS_DEADLOCK_IRQ)
		cal_log_err(id, "IDMA deadlock irq occur\n");

	if (irqs & IDMA_CONFIG_ERROR) {
		cfg_err = dma_read(id, IDMA_CFG_ERR_STATE);
		cal_log_err(id, "IDMA cfg err irq occur(0x%x)\n", cfg_err);
	}
}

/****************** ODMA CAL functions ******************/
static void odma_reg_set_irq_mask_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, ODMA_IRQ, val, ODMA_ALL_IRQ_MASK);
}

static void odma_reg_set_irq_enable(u32 id)
{
	dma_write_mask(id, ODMA_IRQ, ~0, ODMA_IRQ_ENABLE);
}

static void odma_reg_set_in_qos_lut(u32 id, u32 lut_id, u32 qos_t)
{
	u32 reg_id;

	if (lut_id == 0)
		reg_id = ODMA_OUT_QOS_LUT07_00;
	else
		reg_id = ODMA_OUT_QOS_LUT15_08;
	dma_write(id, reg_id, qos_t);
}

static void odma_reg_set_dynamic_gating_en_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, ODMA_DYNAMIC_GATING_EN, val, ODMA_DG_EN_ALL);
}

static void odma_reg_set_out_frame_alpha(u32 id, u32 alpha)
{
	dma_write_mask(id, ODMA_OUT_CON1, ODMA_OUT_FRAME_ALPHA(alpha),
			ODMA_OUT_FRAME_ALPHA_MASK);
}

static void odma_reg_clear_irq(u32 id, u32 irq)
{
	dma_write_mask(id, ODMA_IRQ, ~0, irq);
}

static void odma_reg_set_sw_reset(u32 id)
{
	dma_write_mask(id, ODMA_ENABLE, ~0, ODMA_SRSET);
}

static int odma_reg_wait_sw_reset_status(u32 id)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(dma_regs_desc(id)->regs + ODMA_ENABLE,
			val, !(val & ODMA_SRSET), 10, 2000); /* timeout 2ms */
	if (ret) {
		cal_log_err(id, "[odma] timeout sw-reset\n");
		return ret;
	}

	return 0;
}

static void odma_reg_set_coordinates(u32 id, struct decon_frame *dst)
{
	dma_write(id, ODMA_DST_OFFSET,
			ODMA_DST_OFFSET_Y(dst->y) | ODMA_DST_OFFSET_X(dst->x));
	dma_write(id, ODMA_DST_SIZE,
			ODMA_DST_HEIGHT(dst->f_h) | ODMA_DST_WIDTH(dst->f_w));
	dma_write(id, ODMA_OUT_IMG_SIZE,
			ODMA_OUT_IMG_HEIGHT(dst->h) |
			ODMA_OUT_IMG_WIDTH(dst->w));
}

static void odma_reg_set_format(u32 id, u32 fmt)
{
	dma_write_mask(id, ODMA_OUT_CON0, ODMA_IMG_FORMAT(fmt),
			ODMA_IMG_FORMAT_MASK);
}

static void odma_reg_print_irqs_msg(u32 id, u32 irqs)
{
	u32 cfg_err;

	if (irqs & ODMA_WRITE_SLAVE_ERROR)
		cal_log_err(id, "ODMA write slave error irq occur\n");

	if (irqs & ODMA_STATUS_DEADLOCK_IRQ)
		cal_log_err(id, "ODMA deadlock error irq occur\n");

	if (irqs & ODMA_CONFIG_ERROR) {
		cfg_err = dma_read(id, ODMA_CFG_ERR_STATE);
		cal_log_err(id, "ODMA cfg err irq occur(0x%x)\n", cfg_err);
	}
}

/****************** DPP CAL functions ******************/
static void dpp_reg_set_irq_mask_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dpp_write_mask(id, DPP_IRQ, val, DPP_ALL_IRQ_MASK);
}

static void dpp_reg_set_irq_enable(u32 id)
{
	dpp_write_mask(id, DPP_IRQ, ~0, DPP_IRQ_ENABLE);
}

static void dpp_reg_set_clock_gate_en_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dpp_write_mask(id, DPP_ENABLE, val, DPP_ALL_CLOCK_GATE_EN_MASK);
}

static void dpp_reg_set_dynamic_gating_en_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dpp_write_mask(id, DPP_DYNAMIC_GATING_EN, val, DPP_DG_EN_ALL);
}

static void dpp_reg_set_linecnt(u32 id, u32 en)
{
	if (en)
		dpp_write_mask(id, DPP_LINECNT_CON,
				DPP_LC_MODE(0) | DPP_LC_ENABLE(1),
				DPP_LC_MODE_MASK | DPP_LC_ENABLE_MASK);
	else
		dpp_write_mask(id, DPP_LINECNT_CON, DPP_LC_ENABLE(0),
				DPP_LC_ENABLE_MASK);
}

static void dpp_reg_clear_irq(u32 id, u32 irq)
{
	dpp_write_mask(id, DPP_IRQ, ~0, irq);
}

static void dpp_reg_set_sw_reset(u32 id)
{
	dpp_write_mask(id, DPP_ENABLE, ~0, DPP_SRSET);
}

static int dpp_reg_wait_sw_reset_status(u32 id)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(dpp_regs_desc(id)->regs + DPP_ENABLE,
			val, !(val & DPP_SRSET), 10, 2000);
	if (ret) {
		cal_log_err(id, "[dpp] timeout sw reset\n");
		return ret;
	}

	return 0;
}

#if defined(SUPPORT_USER_COEF)
static void dpp_reg_set_csc_coef(u32 id, u32 std, u32 range)
{
	u32 val, mask;
	u32 csc_id = DPP_CSC_IDX_BT601_625;
	u32 c00, c01, c02;
	u32 c10, c11, c12;
	u32 c20, c21, c22;

	switch (std) {
	case EXYNOS_STANDARD_BT601_625:
		csc_id = DPP_CSC_IDX_BT601_625;
		break;
	case EXYNOS_STANDARD_BT601_625_UNADJUSTED:
		csc_id = DPP_CSC_IDX_BT601_625_UNADJUSTED;
		break;
	case EXYNOS_STANDARD_BT601_525:
		csc_id = DPP_CSC_IDX_BT601_525;
		break;
	case EXYNOS_STANDARD_BT601_525_UNADJUSTED:
		csc_id = DPP_CSC_IDX_BT601_525_UNADJUSTED;
		break;
	case EXYNOS_STANDARD_BT2020_CONSTANT_LUMINANCE:
		csc_id = DPP_CSC_IDX_BT2020_CONSTANT_LUMINANCE;
		break;
	case EXYNOS_STANDARD_BT470M:
		csc_id = DPP_CSC_IDX_BT470M;
		break;
	case EXYNOS_STANDARD_FILM:
		csc_id = DPP_CSC_IDX_FILM;
		break;
	case EXYNOS_STANDARD_ADOBE_RGB:
		csc_id = DPP_CSC_IDX_ADOBE_RGB;
		break;
	default:
		range = EXYNOS_RANGE_LIMITED;
		cal_log_err(id, "invalid CSC type\n");
		cal_log_err(id, "BT601 with limited range is set as default\n");
	}

	/*
	 * The matrices are provided only for full or limited range
	 * and limited range is used as default.
	 */
	if (range == EXYNOS_RANGE_FULL)
		csc_id += 1;

	c00 = csc_y2r_3x3_t[csc_id][0][0];
	c01 = csc_y2r_3x3_t[csc_id][0][1];
	c02 = csc_y2r_3x3_t[csc_id][0][2];

	c10 = csc_y2r_3x3_t[csc_id][1][0];
	c11 = csc_y2r_3x3_t[csc_id][1][1];
	c12 = csc_y2r_3x3_t[csc_id][1][2];

	c20 = csc_y2r_3x3_t[csc_id][2][0];
	c21 = csc_y2r_3x3_t[csc_id][2][1];
	c22 = csc_y2r_3x3_t[csc_id][2][2];

	mask = (DPP_CSC_COEF_H_MASK | DPP_CSC_COEF_L_MASK);
	val = (DPP_CSC_COEF_H(c01) | DPP_CSC_COEF_L(c00));
	dpp_write_mask(id, DPP_CSC_COEF0, val, mask);

	val = (DPP_CSC_COEF_H(c10) | DPP_CSC_COEF_L(c02));
	dpp_write_mask(id, DPP_CSC_COEF1, val, mask);

	val = (DPP_CSC_COEF_H(c12) | DPP_CSC_COEF_L(c11));
	dpp_write_mask(id, DPP_CSC_COEF2, val, mask);

	val = (DPP_CSC_COEF_H(c21) | DPP_CSC_COEF_L(c20));
	dpp_write_mask(id, DPP_CSC_COEF3, val, mask);

	mask = DPP_CSC_COEF_L_MASK;
	val = DPP_CSC_COEF_L(c22);
	dpp_write_mask(id, DPP_CSC_COEF4, val, mask);

	cal_log_debug(id, "---[DPP%d Y2R CSC Type: std=%d, rng=%d]---\n",
		id, std, range);
	cal_log_debug(id, "0x%4x  0x%4x  0x%4x\n", c00, c01, c02);
	cal_log_debug(id, "0x%4x  0x%4x  0x%4x\n", c10, c11, c12);
	cal_log_debug(id, "0x%4x  0x%4x  0x%4x\n", c20, c21, c22);
}
#else
static inline void dpp_reg_set_csc_coef(u32 id, u32 std, u32 range) { }
#endif

static void dpp_reg_set_csc_params(u32 id, u32 std, u32 range)
{
	u32 type, hw_range, mode, val, mask;

	mode = DPP_CSC_MODE_HARDWIRED;

	switch (std) {
	case EXYNOS_STANDARD_UNSPECIFIED:
		type = DPP_CSC_TYPE_BT601;
		cal_log_debug(id, "unspecified CSC type! -> BT_601\n");
		break;
	case EXYNOS_STANDARD_BT709:
		type = DPP_CSC_TYPE_BT709;
		break;
	case EXYNOS_STANDARD_BT601_625:
	case EXYNOS_STANDARD_BT601_625_UNADJUSTED:
	case EXYNOS_STANDARD_BT601_525:
	case EXYNOS_STANDARD_BT601_525_UNADJUSTED:
		type = DPP_CSC_TYPE_BT601;
		break;
	case EXYNOS_STANDARD_BT2020:
		type = DPP_CSC_TYPE_BT2020;
		break;
	case EXYNOS_STANDARD_DCI_P3:
		type = DPP_CSC_TYPE_DCI_P3;
		break;
	default:
		type = DPP_CSC_TYPE_BT601;
		mode = DPP_CSC_MODE_CUSTOMIZED;
		break;
	}

	/*
	 * DPP hardware supports full or limited range.
	 * Limited range is used as default
	 */
	if (range == EXYNOS_RANGE_FULL)
		hw_range = DPP_CSC_RANGE_FULL;
	else if (range == EXYNOS_RANGE_LIMITED)
		hw_range = DPP_CSC_RANGE_LIMITED;
	else
		hw_range = DPP_CSC_RANGE_LIMITED;

	val = type | hw_range | mode;
	mask = (DPP_CSC_TYPE_MASK | DPP_CSC_RANGE_MASK | DPP_CSC_MODE_MASK);
	dpp_write_mask(id, DPP_IN_CON, val, mask);

	if (mode == DPP_CSC_MODE_CUSTOMIZED)
		dpp_reg_set_csc_coef(id, std, range);
}

static void dpp_reg_set_h_coef(u32 id, u32 h_ratio)
{
	int i, j, k, sc_ratio;

	if (h_ratio <= DPP_SC_RATIO_MAX)
		sc_ratio = 0;
	else if (h_ratio <= DPP_SC_RATIO_7_8)
		sc_ratio = 1;
	else if (h_ratio <= DPP_SC_RATIO_6_8)
		sc_ratio = 2;
	else if (h_ratio <= DPP_SC_RATIO_5_8)
		sc_ratio = 3;
	else if (h_ratio <= DPP_SC_RATIO_4_8)
		sc_ratio = 4;
	else if (h_ratio <= DPP_SC_RATIO_3_8)
		sc_ratio = 5;
	else
		sc_ratio = 6;

	for (i = 0; i < 9; i++)
		for (j = 0; j < 8; j++)
			for (k = 0; k < 2; k++)
				dpp_write(id, DPP_H_COEF(i, j, k),
						h_coef_8t[sc_ratio][i][j]);
}

static void dpp_reg_set_v_coef(u32 id, u32 v_ratio)
{
	int i, j, k, sc_ratio;

	if (v_ratio <= DPP_SC_RATIO_MAX)
		sc_ratio = 0;
	else if (v_ratio <= DPP_SC_RATIO_7_8)
		sc_ratio = 1;
	else if (v_ratio <= DPP_SC_RATIO_6_8)
		sc_ratio = 2;
	else if (v_ratio <= DPP_SC_RATIO_5_8)
		sc_ratio = 3;
	else if (v_ratio <= DPP_SC_RATIO_4_8)
		sc_ratio = 4;
	else if (v_ratio <= DPP_SC_RATIO_3_8)
		sc_ratio = 5;
	else
		sc_ratio = 6;

	for (i = 0; i < 9; i++)
		for (j = 0; j < 4; j++)
			for (k = 0; k < 2; k++)
				dpp_write(id, DPP_V_COEF(i, j, k),
						v_coef_4t[sc_ratio][i][j]);
}

static void dpp_reg_set_scale_ratio(u32 id, struct dpp_params_info *p)
{
	dpp_write_mask(id, DPP_MAIN_H_RATIO, DPP_H_RATIO(p->h_ratio),
			DPP_H_RATIO_MASK);
	dpp_write_mask(id, DPP_MAIN_V_RATIO, DPP_V_RATIO(p->v_ratio),
			DPP_V_RATIO_MASK);

	dpp_reg_set_h_coef(id, p->h_ratio);
	dpp_reg_set_v_coef(id, p->v_ratio);

	cal_log_debug(id, "h_ratio : %#x, v_ratio : %#x\n",
			p->h_ratio, p->v_ratio);
}

static void dpp_reg_set_img_size(u32 id, u32 w, u32 h)
{
	dpp_write(id, DPP_IMG_SIZE, DPP_IMG_HEIGHT(h) | DPP_IMG_WIDTH(w));
}

static void dpp_reg_set_scaled_img_size(u32 id, u32 w, u32 h)
{
	dpp_write(id, DPP_SCALED_IMG_SIZE,
			DPP_SCALED_IMG_HEIGHT(h) | DPP_SCALED_IMG_WIDTH(w));
}

static void dpp_reg_set_alpha_type(u32 id, u32 type)
{
	/* [type] 0=per-frame, 1=per-pixel */
	dpp_write_mask(id, DPP_IN_CON, DPP_ALPHA_SEL(type), DPP_ALPHA_SEL_MASK);
}

static void dpp_reg_set_format(u32 id, u32 fmt)
{
	dpp_write_mask(id, DPP_IN_CON, DPP_IMG_FORMAT(fmt),
			DPP_IMG_FORMAT_MASK);
}

static void dpp_reg_set_eotf_lut(u32 id, struct dpp_params_info *p)
{
	u32 i = 0;
	const u32 *lut_x;
	const u32 *lut_y;

	if (p->transfer == EXYNOS_TRANSFER_ST2084) {
		if (p->max_luminance > 4000) {
			cal_log_err(id, "%d nits is not supported now.\n",
					p->max_luminance);
			return;
		} else if (p->max_luminance > 1000) {
			lut_x = eotf_x_axis_st2084_4000;
			lut_y = eotf_y_axis_st2084_4000;
		} else {
			lut_x = eotf_x_axis_st2084_1000;
			lut_y = eotf_y_axis_st2084_1000;
		}
	} else if (p->transfer == EXYNOS_TRANSFER_HLG) {
		lut_x = eotf_x_axis_hlg;
		lut_y = eotf_y_axis_hlg;
	} else {
		cal_log_err(id, "Undefined HDR standard Type!!!\n");
		return;
	}

	for (i = 0; i < MAX_EOTF; i++) {
		dpp_write_mask(id,
			DPP_HDR_EOTF_X_AXIS_ADDR(i),
			DPP_HDR_EOTF_X_AXIS_VAL(i, lut_x[i]),
			DPP_HDR_EOTF_MASK(i));
		dpp_write_mask(id,
			DPP_HDR_EOTF_Y_AXIS_ADDR(i),
			DPP_HDR_EOTF_Y_AXIS_VAL(i, lut_y[i]),
			DPP_HDR_EOTF_MASK(i));
	}
}

static void dpp_reg_set_gm_lut(u32 id, struct dpp_params_info *p)
{
	u32 i = 0;
	const u32 *lut_gm;

	if (p->standard == EXYNOS_STANDARD_BT2020) {
		lut_gm = gm_coef_2020_p3;
	} else {
		cal_log_err(id, "Undefined HDR CSC Type!!!\n");
		return;
	}

	for (i = 0; i < MAX_GM; i++) {
		dpp_write_mask(id,
			DPP_HDR_GM_COEF_ADDR(i),
			lut_gm[i],
			DPP_HDR_GM_COEF_MASK);
	}
}

static void dpp_reg_set_tm_lut(u32 id, struct dpp_params_info *p)
{
	u32 i = 0;
	const u32 *lut_x;
	const u32 *lut_y;
	u32 tm_x_tune[MAX_TM] = { 0, };
	u32 tm_y_tune[MAX_TM] = { 0, };

	if ((p->max_luminance > 1000) && (p->max_luminance < 10000)) {
		lut_x = tm_x_axis_gamma_2P2_4000;
		lut_y = tm_y_axis_gamma_2P2_4000;
	} else {
		lut_x = tm_x_axis_gamma_2P2_1000;
		lut_y = tm_y_axis_gamma_2P2_1000;
	}

	if (IS_ENABLED(CONFIG_EXYNOS_HDR_TUNABLE_TONEMAPPING) &&
			exynos_hdr_get_tm_lut_xy(tm_x_tune, tm_y_tune)) {
		lut_x = tm_x_tune;
		lut_y = tm_y_tune;
	}

	for (i = 0; i < MAX_TM; i++) {
		dpp_write_mask(id,
			DPP_HDR_TM_X_AXIS_ADDR(i),
			DPP_HDR_TM_X_AXIS_VAL(i, lut_x[i]),
			DPP_HDR_TM_MASK(i));
		dpp_write_mask(id,
			DPP_HDR_TM_Y_AXIS_ADDR(i),
			DPP_HDR_TM_Y_AXIS_VAL(i, lut_y[i]),
			DPP_HDR_TM_MASK(i));
	}
}

static void dpp_reg_set_hdr_params(u32 id, struct dpp_params_info *p)
{
	u32 val, val2, mask;

	val = ((p->transfer == EXYNOS_TRANSFER_ST2084) ||
		(p->transfer == EXYNOS_TRANSFER_HLG)) ? ~0 : 0;
	mask = DPP_HDR_ON_MASK | DPP_EOTF_ON_MASK | DPP_TM_ON_MASK;
	dpp_write_mask(id, DPP_VGRF_HDR_CON, val, mask);

	val2 = (p->standard != EXYNOS_STANDARD_DCI_P3) ? ~0 : 0;
	dpp_write_mask(id, DPP_VGRF_HDR_CON, val2,  DPP_GM_ON_MASK);

	if (val) {
		dpp_reg_set_eotf_lut(id, p);
		dpp_reg_set_gm_lut(id, p);
		dpp_reg_set_tm_lut(id, p);
	}
}

static void dpp_reg_print_irqs_msg(u32 id, u32 irqs)
{
	u32 cfg_err;

	if (irqs & DPP_CONFIG_ERROR) {
		cfg_err = dpp_read(id, DPP_CFG_ERR_STATE);
		cal_log_err(id, "DPP cfg err irq occur(0x%x)\n", cfg_err);
	}
}

/****************** WB MUX CAL functions ******************/
static void wb_mux_reg_set_sw_reset(u32 id)
{
	dpp_write_mask(id, DPU_WB_ENABLE, ~0, DPU_WB_SRSET);
}

static int wb_mux_reg_wait_sw_reset_status(u32 id)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(dpp_regs_desc(id)->regs + DPU_WB_ENABLE,
			val, !(val & DPU_WB_SRSET), 10, 2000);
	if (ret) {
		cal_log_err(id, "[WBMUX] dpp%d timeout sw-reset\n");
		return ret;
	}

	return 0;
}

static void wb_mux_reg_set_clock_gate_en_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dpp_write_mask(id, DPU_WB_ENABLE, val, DPU_WB_ALL_CLOCK_GATE_EN_MASK);
}

static void wb_mux_reg_set_format(u32 id, const struct dpu_fmt *fmt_info)
{
	u32 val = 0, mask;

	mask = DPU_WB_YUV_TYPE | DPU_WB_CSC_R2Y;

	/* RGB888, YUV420 or YUV422 ? */
	if (IS_YUV(fmt_info)) {
		val = DPU_WB_CSC_R2Y;
		if (IS_YUV422(fmt_info))
			val |= DPU_WB_YUV_TYPE;
	}

	dpp_write_mask(id, DPU_WB_CSC_CON, val, mask);
}

static void wb_mux_reg_set_uv_offset(u32 id, u32 off_x, u32 off_y)
{
	u32 val, mask;

	val = DPU_WB_UV_OFFSET_Y(off_y) | DPU_WB_UV_OFFSET_X(off_x);
	mask = DPU_WB_UV_OFFSET_Y_MASK | DPU_WB_UV_OFFSET_X_MASK;
	dpp_write_mask(id, DPU_WB_CSC_CON, val, mask);
}

static void wb_mux_reg_set_csc_params(u32 id, u32 std, u32 range)
{
	u32 val = 0;

	switch (std) {
	case EXYNOS_STANDARD_BT601_625:
	case EXYNOS_STANDARD_BT601_625_UNADJUSTED:
	case EXYNOS_STANDARD_BT601_525:
	case EXYNOS_STANDARD_BT601_525_UNADJUSTED:
		if (range == EXYNOS_RANGE_LIMITED)
			val = DPU_WB_CSC_TYPE_601_LIMIT;
		else if (range == EXYNOS_RANGE_FULL)
			val = DPU_WB_CSC_TYPE_601_FULL;
		else
			goto not_supp;
		break;
	case EXYNOS_STANDARD_BT709:
		if (range == EXYNOS_RANGE_LIMITED)
			val = DPU_WB_CSC_TYPE_709_LIMIT;
		else if (range == EXYNOS_RANGE_FULL)
			val = DPU_WB_CSC_TYPE_709_FULL;
		else
			goto not_supp;
		break;
	default:
		goto not_supp;
	}

	dpp_write_mask(id, DPU_WB_CSC_CON, val, DPU_WB_CSC_TYPE_MASK);
	return;

not_supp:
	cal_log_warn(id, "Not supported standard(%d) range(%d)\n", std, range);
}

static void wb_mux_reg_set_dst_size(u32 id, u32 w, u32 h)
{
	dpp_write(id, DPU_WB_IMG_SIZE,
			DPU_WB_IMG_HEIGHT(h) | DPU_WB_IMG_WIDTH(w));
}

static void wb_mux_reg_set_dynamic_gating_en_all(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dpp_write_mask(id, DPU_WB_DYNAMIC_GATING_EN, val,
			DPU_WB_DYNAMIC_GATING_EN_ALL);
}

/********** IDMA and ODMA combination CAL functions **********/
static void dma_reg_set_base_addr(u32 id, struct dpp_params_info *p,
		const unsigned long attr)
{
	const struct dpu_fmt *fmt_info = dpu_find_fmt_info(p->format);

	if (test_bit(DPP_ATTR_IDMA, &attr)) {
		dma_write(id, IDMA_IN_BASE_ADDR_Y8, p->addr[0]);
		if (p->comp_type == COMP_TYPE_AFBC)
			dma_write(id, IDMA_IN_BASE_ADDR_C8, p->addr[0]);
		else
			dma_write(id, IDMA_IN_BASE_ADDR_C8, p->addr[1]);

		if (fmt_info->num_planes == 4) { /* use 4 base addresses */
			dma_write(id, IDMA_IN_BASE_ADDR_Y2, p->addr[2]);
			dma_write(id, IDMA_IN_BASE_ADDR_C2, p->addr[3]);
			dma_write_mask(id, IDMA_2BIT_STRIDE,
				IDMA_LUMA_2B_STRIDE(p->y_hd_y2_stride),
				IDMA_LUMA_2B_STRIDE_MASK);
			dma_write_mask(id, IDMA_2BIT_STRIDE,
				IDMA_CHROMA_2B_STRIDE(p->y_pl_c2_stride),
				IDMA_CHROMA_2B_STRIDE_MASK);
		}
	} else if (test_bit(DPP_ATTR_ODMA, &attr)) {
		dma_write(id, ODMA_IN_BASE_ADDR_Y8, p->addr[0]);
		dma_write(id, ODMA_IN_BASE_ADDR_C8, p->addr[1]);

		if (fmt_info->num_planes == 4) {
			dma_write(id, ODMA_IN_BASE_ADDR_Y2, p->addr[2]);
			dma_write(id, ODMA_IN_BASE_ADDR_C2, p->addr[3]);
			dma_write_mask(id, ODMA_2BIT_STRIDE,
				ODMA_LUMA_2BIT_STRIDE(p->y_hd_y2_stride),
				ODMA_LUMA_2BIT_STRIDE_MASK);
			dma_write_mask(id, ODMA_2BIT_STRIDE,
				ODMA_CHROM_2BIT_STRIDE(p->y_pl_c2_stride),
				ODMA_CHROM_2BIT_STRIDE_MASK);
		}
	}
	cal_log_debug(id, "dpp%d: addr 1p(0x%p) 2p(0x%p) 3p(0x%p) 4p(0x%p)\n",
			id,
			(void *)p->addr[0], (void *)p->addr[1],
			(void *)p->addr[2], (void *)p->addr[3]);
}

/********** IDMA, ODMA, DPP and WB MUX combination CAL functions **********/
static void dma_dpp_reg_set_coordinates(u32 id, struct dpp_params_info *p,
		const unsigned long attr)
{
	if (test_bit(DPP_ATTR_IDMA, &attr)) {
		idma_reg_set_coordinates(id, &p->src);

		if (test_bit(DPP_ATTR_DPP, &attr)) {
			if (p->rot > DPP_ROT_180)
				dpp_reg_set_img_size(id, p->src.h, p->src.w);
			else
				dpp_reg_set_img_size(id, p->src.w, p->src.h);
		}

		if (test_bit(DPP_ATTR_SCALE, &attr))
			dpp_reg_set_scaled_img_size(id, p->dst.w, p->dst.h);
	} else if (test_bit(DPP_ATTR_ODMA, &attr)) {
		odma_reg_set_coordinates(id, &p->src);
		wb_mux_reg_set_dst_size(id, p->src.w, p->src.h);
	}
}

static int dma_dpp_reg_set_format(u32 id, struct dpp_params_info *p,
		const unsigned long attr)
{
	u32 alpha_type = 0; /* 0: per-frame, 1: per-pixel */
	const struct dpu_fmt *fmt_info = dpu_find_fmt_info(p->format);

	alpha_type = (fmt_info->len_alpha > 0) ? 1 : 0;

	if (test_bit(DPP_ATTR_IDMA, &attr)) {
		idma_reg_set_format(id, fmt_info->dma_fmt);
		if (test_bit(DPP_ATTR_DPP, &attr)) {
			dpp_reg_set_alpha_type(id, alpha_type);
			dpp_reg_set_format(id, fmt_info->dpp_fmt);
		}
	} else if (test_bit(DPP_ATTR_ODMA, &attr)) {
		odma_reg_set_format(id, fmt_info->dma_fmt);
		wb_mux_reg_set_format(id, fmt_info);
		wb_mux_reg_set_uv_offset(id, 0, 0);
	}

	return 0;
}

/******************** EXPORTED DPP CAL APIs ********************/
void dpp_reg_init(u32 id, const unsigned long attr)
{
	if (test_bit(DPP_ATTR_IDMA, &attr)) {
		idma_reg_set_irq_mask_all(id, 0);
		idma_reg_set_irq_enable(id);
		idma_reg_set_clock_gate_en_all(id, 0);
		idma_reg_set_in_qos_lut(id, 0, 0x44444444);
		idma_reg_set_in_qos_lut(id, 1, 0x44444444);
		idma_reg_set_dynamic_gating_en_all(id, 0);
		idma_reg_set_out_frame_alpha(id, 0xFF);
	}

	if (test_bit(DPP_ATTR_DPP, &attr)) {
		dpp_reg_set_irq_mask_all(id, 0);
		dpp_reg_set_irq_enable(id);
		dpp_reg_set_clock_gate_en_all(id, 0);
		dpp_reg_set_dynamic_gating_en_all(id, 0);
		dpp_reg_set_linecnt(id, 1);
	}

	if (test_bit(DPP_ATTR_ODMA, &attr)) {
		odma_reg_set_irq_mask_all(id, 0); /* irq unmask */
		odma_reg_set_irq_enable(id);
		odma_reg_set_in_qos_lut(id, 0, 0x44444444);
		odma_reg_set_in_qos_lut(id, 1, 0x44444444);
		odma_reg_set_dynamic_gating_en_all(id, 0);
		odma_reg_set_out_frame_alpha(id, 0xFF);
		/* TODO: clock gating will be enabled */
		wb_mux_reg_set_clock_gate_en_all(id, 0);
		wb_mux_reg_set_dynamic_gating_en_all(id, 0);
	}
}

int dpp_reg_deinit(u32 id, bool reset, const unsigned long attr)
{
	if (test_bit(DPP_ATTR_IDMA, &attr)) {
		idma_reg_clear_irq(id, IDMA_ALL_IRQ_CLEAR);
		idma_reg_set_irq_mask_all(id, 1);
	}

	if (test_bit(DPP_ATTR_DPP, &attr)) {
		dpp_reg_clear_irq(id, DPP_ALL_IRQ_CLEAR);
		dpp_reg_set_irq_mask_all(id, 1);
	}

	if (test_bit(DPP_ATTR_ODMA, &attr)) {
		odma_reg_clear_irq(id, ODMA_ALL_IRQ_CLEAR);
		odma_reg_set_irq_mask_all(id, 1); /* irq mask */
	}

	if (reset) {
		if (test_bit(DPP_ATTR_IDMA, &attr) &&
				!test_bit(DPP_ATTR_DPP, &attr)) { /* IDMA */
			idma_reg_set_sw_reset(id);
			if (idma_reg_wait_sw_reset_status(id))
				return -1;
		} else if (test_bit(DPP_ATTR_IDMA, &attr) &&
				test_bit(DPP_ATTR_DPP, &attr)) { /* IDMA/DPP */
			idma_reg_set_sw_reset(id);
			dpp_reg_set_sw_reset(id);
			if (idma_reg_wait_sw_reset_status(id) ||
					dpp_reg_wait_sw_reset_status(id))
				return -1;
		} else if (test_bit(DPP_ATTR_ODMA, &attr)) { /* writeback */
			odma_reg_set_sw_reset(id);
			wb_mux_reg_set_sw_reset(id);
			if (odma_reg_wait_sw_reset_status(id) ||
					wb_mux_reg_wait_sw_reset_status(id))
				return -1;
		} else {
			cal_log_err(id, "%s: not support attribute case(0x%lx)\n",
					__func__, attr);
		}
	}

	return 0;
}

#if defined(DMA_BIST)
static u32 pattern_data[] = {
	0xffffffff,
	0xffffffff,
	0xffffffff,
	0xffffffff,
	0x000000ff,
	0x000000ff,
	0x000000ff,
	0x000000ff,
};
#endif

void dpp_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr)
{
	if (test_bit(DPP_ATTR_CSC, &attr) && test_bit(DPP_ATTR_DPP, &attr))
		dpp_reg_set_csc_params(id, p->standard, p->range);
	else if (test_bit(DPP_ATTR_CSC, &attr) &&
			test_bit(DPP_ATTR_ODMA, &attr))
		wb_mux_reg_set_csc_params(id, p->standard, p->range);

	if (test_bit(DPP_ATTR_SCALE, &attr))
		dpp_reg_set_scale_ratio(id, p);

	/* configure coordinates and size of IDMA, DPP, ODMA and WB MUX */
	dma_dpp_reg_set_coordinates(id, p, attr);

	if (test_bit(DPP_ATTR_ROT, &attr) || test_bit(DPP_ATTR_FLIP, &attr))
		idma_reg_set_rotation(id, p->rot);

	/* configure base address of IDMA and ODMA */
	dma_reg_set_base_addr(id, p, attr);

	if (test_bit(DPP_ATTR_BLOCK, &attr))
		idma_reg_set_block_mode(id, p->is_block, p->block.x, p->block.y,
				p->block.w, p->block.h);

	/* configure image format of IDMA, DPP, ODMA and WB MUX */
	dma_dpp_reg_set_format(id, p, attr);

	if (test_bit(DPP_ATTR_HDR, &attr))
		dpp_reg_set_hdr_params(id, p);

	if (test_bit(DPP_ATTR_AFBC, &attr))
		idma_reg_set_afbc(id, p->comp_type == COMP_TYPE_AFBC,
				p->rcv_num);

#if defined(DMA_BIST)
	idma_reg_set_test_pattern(id, 0, pattern_data);
#endif
}

u32 dpp_reg_get_irq_and_clear(u32 id)
{
	u32 val;

	val = dpp_read(id, DPP_IRQ);
	dpp_reg_print_irqs_msg(id, val);
	dpp_reg_clear_irq(id, val);

	return val;
}

u32 idma_reg_get_irq_and_clear(u32 id)
{
	u32 val;

	val = dma_read(id, IDMA_IRQ);
	idma_reg_print_irqs_msg(id, val);
	idma_reg_clear_irq(id, val);

	return val;
}

u32 odma_reg_get_irq_and_clear(u32 id)
{
	u32 val;

	val = dma_read(id, ODMA_IRQ);
	odma_reg_print_irqs_msg(id, val);
	odma_reg_clear_irq(id, val);

	return val;
}

void dma_reg_get_shd_addr(u32 id, u32 s_addr[], const unsigned long attr)
{
	if (test_bit(DPP_ATTR_IDMA, &attr)) {
		s_addr[0] = dma_read(id, IDMA_IN_BASE_ADDR_Y8 + DMA_SHD_OFFSET);
		s_addr[1] = dma_read(id, IDMA_IN_BASE_ADDR_C8 + DMA_SHD_OFFSET);
		s_addr[2] = dma_read(id, IDMA_IN_BASE_ADDR_Y2 + DMA_SHD_OFFSET);
		s_addr[3] = dma_read(id, IDMA_IN_BASE_ADDR_C2 + DMA_SHD_OFFSET);
	} else if (test_bit(DPP_ATTR_ODMA, &attr)) {
		s_addr[0] = dma_read(id, ODMA_IN_BASE_ADDR_Y8 + DMA_SHD_OFFSET);
		s_addr[1] = dma_read(id, ODMA_IN_BASE_ADDR_C8 + DMA_SHD_OFFSET);
		s_addr[2] = dma_read(id, ODMA_IN_BASE_ADDR_Y2 + DMA_SHD_OFFSET);
		s_addr[3] = dma_read(id, ODMA_IN_BASE_ADDR_C2 + DMA_SHD_OFFSET);
	}

	cal_log_debug(id, "shadow addr [%p %p %p %p]\n",
			s_addr[0], s_addr[1], s_addr[2], s_addr[3]);
}

static void dpp_reg_dump_ch_data(struct drm_printer *p, int id, enum dpp_reg_area reg_area,
					const u32 sel[], u32 cnt)
{
	unsigned char linebuf[128] = {0, };
	int i, ret;
	int len = 0;
	u32 data;

	for (i = 0; i < cnt; i++) {
		if (!(i % 4) && i != 0) {
			linebuf[len] = '\0';
			len = 0;
			cal_drm_printf(p, id, "%s\n", linebuf);
		}

		if (reg_area == REG_AREA_DPP) {
			dpp_write(id, 0xC04, sel[i]);
			data = dpp_read(id, 0xC10);
		} else if (reg_area == REG_AREA_DMA) {
			dma_write(id, IDMA_DEBUG_CONTROL,
					IDMA_DEBUG_CONTROL_SEL(sel[i]) |
					IDMA_DEBUG_CONTROL_EN);
			data = dma_read(id, IDMA_DEBUG_DATA);
		} else { /* REG_AREA_DMA_COM */
			dma_com_write(0, DPU_DMA_DEBUG_CONTROL,
					DPU_DMA_DEBUG_CONTROL_SEL(sel[i]) |
					DPU_DMA_DEBUG_CONTROL_EN);
			data = dma_com_read(0, DPU_DMA_DEBUG_DATA);
		}

		ret = snprintf(linebuf + len, sizeof(linebuf) - len,
				"[0x%08x: %08x] ", sel[i], data);
		if (ret >= sizeof(linebuf) - len) {
			cal_log_err(id, "overflow: %d %ld %d\n",
					ret, sizeof(linebuf), len);
			return;
		}
		len += ret;
	}
	cal_drm_printf(p, id, "%s\n", linebuf);
}

static void dma_reg_dump_com_debug_regs(struct drm_printer *p, int id)
{
	static bool checked;

	const u32 sel_ch0[30] = {
		0x0000, 0x0001, 0x0005, 0x0009, 0x000D, 0x000E, 0x0020, 0x0021,
		0x0025, 0x0029, 0x002D, 0x002E, 0x0100, 0x0101, 0x0105, 0x0109,
		0x010D, 0x010E, 0x0120, 0x0121, 0x0125, 0x0129, 0x012D, 0x012E,
		0x0200, 0x0300, 0x0400, 0x0401, 0x0402, 0x0403
	};

	const u32 sel_ch1[29] = {
		0x4000, 0x4001, 0x4005, 0x4009, 0x400D, 0x400E, 0x4020, 0x4021,
		0x4025, 0x4029, 0x402D, 0x402E, 0x4100, 0x4101, 0x4105, 0x4109,
		0x410D, 0x410E, 0x4120, 0x4121, 0x4125, 0x4129, 0x412D, 0x412E,
		0x4200, 0x4300, 0x4400, 0x4401, 0x4402
	};

	const u32 sel_ch2[5] = {
		0x8000, 0x8001, 0x8002, 0xC000, 0xC001
	};

	cal_drm_printf(p, id, "%s: checked = %d\n", __func__, checked);
	if (checked)
		return;

	cal_drm_printf(p, id, "-< DMA COMMON DEBUG SFR(CH0) >-\n");
	dpp_reg_dump_ch_data(p, id, REG_AREA_DMA_COM, sel_ch0, 30);

	cal_drm_printf(p, id, "-< DMA COMMON DEBUG SFR(CH1) >-\n");
	dpp_reg_dump_ch_data(p, id, REG_AREA_DMA_COM, sel_ch1, 29);

	cal_drm_printf(p, id, "-< DMA COMMON DEBUG SFR(CH2) >-\n");
	dpp_reg_dump_ch_data(p, id, REG_AREA_DMA_COM, sel_ch2, 5);

	checked = true;
}

static void dma_reg_dump_debug_regs(struct drm_printer *p, int id, unsigned long attr)
{
	const u32 sel_plane0[10] = {
		0x0000, 0x0001, 0x0002, 0x0003, 0x0007, 0x0008, 0x0400, 0x0401,
		0x0402, 0x0403
	};

	const u32 sel_plane1[10] = {
		0x1000, 0x1001, 0x1002, 0x1003, 0x1007, 0x1008, 0x1400, 0x1401,
		0x1402, 0x1403
	};

	const u32 sel_plane2[10] = {
		0x2000, 0x2001, 0x2002, 0x2003, 0x2007, 0x2008, 0x2400, 0x2401,
		0x2402, 0x2403
	};

	const u32 sel_plane3[10] = {
		0x3000, 0x3001, 0x3002, 0x3003, 0x3007, 0x3008, 0x3400, 0x3401,
		0x3402, 0x3403
	};

	const u32 sel_yuv[6] = {
		0x4001, 0x4002, 0x4003, 0x4005, 0x4006, 0x4007
	};

	const u32 sel_fbc[15] = {
		0x5100, 0x5101, 0x5104, 0x5105, 0x5200, 0x5201, 0x5202, 0x5203,
		0x5204, 0x5300, 0x5301, 0x5302, 0x5303, 0x5304, 0x5305
	};

	const u32 sel_rot[16] = {
		0x6100, 0x6101, 0x6102, 0x6103, 0x6200, 0x6201, 0x6202, 0x6203,
		0x6300, 0x6301, 0x6305, 0x6306, 0x6400, 0x6401, 0x6405, 0x6406
	};

	const u32 sel_pix[4] = {
		0x7000, 0x7001, 0x7002, 0x7003
	};

	cal_drm_printf(p, id, "-< DPU_DMA%d DEBUG SFR >-\n", id);
	dpp_reg_dump_ch_data(p, id, REG_AREA_DMA, sel_plane0, 10);

	if (test_bit(DPP_ATTR_CSC, &attr)) {
		dpp_reg_dump_ch_data(p, id, REG_AREA_DMA, sel_plane1, 10);
		dpp_reg_dump_ch_data(p, id, REG_AREA_DMA, sel_plane2, 10);
		dpp_reg_dump_ch_data(p, id, REG_AREA_DMA, sel_plane3, 10);
		dpp_reg_dump_ch_data(p, id, REG_AREA_DMA, sel_yuv, 6);
	}

	if (test_bit(DPP_ATTR_AFBC, &attr))
		dpp_reg_dump_ch_data(p, id, REG_AREA_DMA, sel_fbc, 15);

	if (test_bit(DPP_ATTR_ROT, &attr))
		dpp_reg_dump_ch_data(p, id, REG_AREA_DMA, sel_rot, 16);

	dpp_reg_dump_ch_data(p, id, REG_AREA_DMA, sel_pix, 4);
}

static void dpp_reg_dump_debug_regs(struct drm_printer *p, int id)
{
	const u32 sel_gf[3] = {0x0000, 0x0100, 0x0101};
	const u32 sel_vg_vgf[19] = {0x0000, 0x0100, 0x0101, 0x0200, 0x0201,
		0x0202, 0x0203, 0x0204, 0x0205, 0x0206, 0x0207, 0x0208, 0x0300,
		0x0301, 0x0302, 0x0303, 0x0304, 0x0400, 0x0401};
	const u32 sel_vgs_vgrfs[37] = {0x0000, 0x0100, 0x0101, 0x0200, 0x0201,
		0x0210, 0x0211, 0x0220, 0x0221, 0x0230, 0x0231, 0x0240, 0x0241,
		0x0250, 0x0251, 0x0300, 0x0301, 0x0302, 0x0303, 0x0304, 0x0305,
		0x0306, 0x0307, 0x0308, 0x0400, 0x0401, 0x0402, 0x0403, 0x0404,
		0x0500, 0x0501, 0x0502, 0x0503, 0x0504, 0x0505, 0x0600, 0x0601};
	u32 cnt;
	const u32 *sel = NULL;

	if (id == 0 || id == 2) { /* GF0, GF1 */
		sel = sel_gf;
		cnt = 3;
	} else if (id == 3 || id == 4) { /* VGF, VG */
		sel = sel_vg_vgf;
		cnt = 19;
	} else if (id == 1 || id == 5) { /* VGRFS, VGS */
		sel = sel_vgs_vgrfs;
		cnt = 37;
	} else {
		cal_log_err(id, "DPP%d is wrong ID\n", id);
		return;
	}

	dpp_write(id, 0x0C00, 0x1);
	cal_drm_printf(p, id, "-< DPP%d DEBUG SFR >-\n", id);
	dpp_reg_dump_ch_data(p, id, REG_AREA_DPP, sel, cnt);
}

static void dma_dump_regs(struct drm_printer *p, u32 id, void __iomem *dma_regs)
{
	cal_drm_printf(p, id, "\n=== DPU_DMA%d SFR DUMP ===\n", id);
	dpu_print_hex_dump(p, dma_regs, dma_regs, 0x6C);
	dpu_print_hex_dump(p, dma_regs, dma_regs + 0x100, 0x8);

	cal_drm_printf(p, id, "=== DPU_DMA%d SHADOW SFR DUMP ===\n", id);
	dpu_print_hex_dump(p, dma_regs, dma_regs + 0x800, 0x74);
	dpu_print_hex_dump(p, dma_regs, dma_regs + 0x900, 0x8);
}

static void dpp_dump_regs(struct drm_printer *p, u32 id, void __iomem *regs, unsigned long attr)
{
	cal_drm_printf(p, id, "=== DPP%d SFR DUMP ===\n", id);

	dpu_print_hex_dump(p, regs, regs, 0x4C);
	if (test_bit(DPP_ATTR_AFBC, &attr))
		dpu_print_hex_dump(p, regs, regs + 0x5B0, 0x10);

	if (test_bit(DPP_ATTR_ROT, &attr))
		dpu_print_hex_dump(p, regs, regs + 0x600, 0x1E0);

	dpu_print_hex_dump(p, regs, regs + 0xA54, 0x4);
	dpu_print_hex_dump(p, regs, regs + 0xB00, 0x4C);
	if (test_bit(DPP_ATTR_AFBC, &attr))
		dpu_print_hex_dump(p, regs, regs + 0xBB0, 0x10);

	dpu_print_hex_dump(p, regs, regs + 0xD00, 0xC);
}

void __dpp_dump(struct drm_printer *p, u32 id, void __iomem *regs, void __iomem *dma_regs,
		unsigned long attr)
{
	dma_reg_dump_com_debug_regs(p, id);

	dma_dump_regs(p, id, dma_regs);
	dma_reg_dump_debug_regs(p, id, attr);

	dpp_dump_regs(p, id, regs, attr);
	dpp_reg_dump_debug_regs(p, id);
}

int __dpp_check(u32 id, const struct dpp_params_info *p, unsigned long attr)
{
	const struct dpu_fmt *fmt = dpu_find_fmt_info(p->format);

	if (p->comp_type == COMP_TYPE_AFBC) {
		if (p->rot & DPP_ROT_FLIP_MASK) {
			cal_log_err(id,	"AFBC with rot is not supported\n");
			return -EINVAL;
		} else if (IS_YUV(fmt)) {
			cal_log_err(id,	"AFBC with YUV fmt is not supported\n");
			return -EINVAL;
		} else if (IS_RGB32(fmt) && IS_10BPC(fmt)) {
			cal_log_err(id,
				"AFBC with 10BPC RGB fmt is not supported\n");
			return -EINVAL;
		}
	}

	return 0;
}

#ifdef __linux__
int __dpp_init_resources(struct dpp_device *dpp)
{
	struct resource *res;
	struct device *dev = dpp->dev;
	struct platform_device *pdev;

	if (dpp->id != 0)
		return 0;

	pdev = container_of(dev, struct platform_device, dev);

	/* DPP0 channel can only access common area of DPU_DMA */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		cal_log_err(dpp->id, "failed to get mem resource\n");
		return -ENOENT;
	}

	dpp->regs.dma_common_base_regs = devm_ioremap_resource(dev, res);
	if (!dpp->regs.dma_common_base_regs) {
		cal_log_err(dpp->id, "failed to map DPU_DMA COMMON SFR\n");
		return -EINVAL;
	}
	dpp_regs_desc_init(dpp->regs.dma_common_base_regs, "dma_common",
			REGS_DMA_COMMON, dpp->id);
	cal_log_info(dpp->id, "dma common res start:0x%x end:0x%x vir:0x%llx\n",
			(u32)res->start, (u32)res->end,
			(u64)dpp->regs.dma_common_base_regs);

	return 0;
}
#endif
