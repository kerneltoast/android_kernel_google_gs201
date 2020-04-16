// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register access functions for Samsung Display Quality Enhancer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <cal_config.h>
#include <regs-dqe.h>
#include <dqe_cal.h>
#include <decon_cal.h>
#include <drm/samsung_drm.h>

static struct cal_regs_desc regs_dqe;

#define dqe_read(offset)	cal_read((&regs_dqe), offset)
#define dqe_write(offset, val)	cal_write((&regs_dqe), offset, val)
#define dqe_read_mask(offset, mask)		\
		cal_read_mask((&regs_dqe), offset, mask)
#define dqe_write_mask(offset, val, mask)	\
		cal_write_mask((&regs_dqe), offset, val, mask)

void dqe_regs_desc_init(void __iomem *regs, const char *name)
{
	regs_dqe.regs = regs;
	regs_dqe.name = name;
}

static void dqe_reg_set_img_size(u32 width, u32 height)
{
	u32 val;

	val = DQE_IMG_VSIZE(height) | DQE_IMG_HSIZE(width);
	dqe_write(DQE0_TOP_IMG_SIZE, val);
}

static void dqe_reg_set_full_img_size(u32 width, u32 height)
{
	u32 val;

	val = DQE_FULL_IMG_VSIZE(height) | DQE_FULL_IMG_HSIZE(width);
	dqe_write(DQE0_TOP_FRM_SIZE, val);

	val = DQE_FULL_PXL_NUM(width * height);
	dqe_write(DQE0_TOP_FRM_PXL_NUM, val);
}

/* exposed to driver layer for DQE CAL APIs */
void dqe_reg_init(u32 width, u32 height)
{
	cal_log_debug(0, "%s +\n", __func__);
	decon_reg_set_dqe_enable(0, true);
	dqe_reg_set_img_size(width, height);
	dqe_reg_set_full_img_size(width, height);
	cal_log_debug(0, "%s -\n", __func__);
}

void dqe_reg_set_degamma_lut(const struct drm_color_lut *lut)
{
	int i;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lut) {
		dqe_write(DQE0_DEGAMMA_CON, 0);
		return;
	}

	dqe_write(DQE0_DEGAMMA_CON, DEGAMMA_EN);
	for (i = 0; i < DIV_ROUND_UP(DEGAMMA_LUT_SIZE, 2); ++i) {
		val = DEGAMMA_LUT_H(lut[i * 2 + 1].red) |
			DEGAMMA_LUT_L(lut[i * 2].red);
		dqe_write(DQE0_DEGAMMALUT(i), val);

		cal_log_debug(0, "[%d] 0x%x\n", i, val);
	}

	cal_log_debug(0, "%s -\n", __func__);
}

void dqe_reg_set_cgc_lut(const struct cgc_lut *lut)
{
	int i;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lut) {
		dqe_write_mask(DQE0_CGC_CON, 0, CGC_EN);
		return;
	}

	dqe_write_mask(DQE0_CGC_CON, ~0, CGC_EN);
	for (i = 0; i < DRM_SAMSUNG_CGC_LUT_REG_CNT; ++i) {
		dqe_write(DQE0_CGC_LUT_R(i), lut->r_values[i]);
		cal_log_debug(0, "[%d]   red: 0x%x\n", i, lut->r_values[i]);

		dqe_write(DQE0_CGC_LUT_G(i), lut->g_values[i]);
		cal_log_debug(0, "[%d] green: 0x%x\n", i, lut->g_values[i]);

		dqe_write(DQE0_CGC_LUT_B(i), lut->b_values[i]);
		cal_log_debug(0, "[%d]  blue: 0x%x\n", i, lut->b_values[i]);
	}

	cal_log_debug(0, "%s -\n", __func__);
}

void dqe_reg_set_regamma_lut(const struct drm_color_lut *lut)
{
	int i;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lut) {
		dqe_write(DQE0_REGAMMA_CON, 0);
		return;
	}

	dqe_write(DQE0_REGAMMA_CON, REGAMMA_EN);
	for (i = 0; i < DIV_ROUND_UP(REGAMMA_LUT_SIZE, 2); ++i) {
		val = REGAMMA_LUT_H(lut[i * 2 + 1].red) |
				REGAMMA_LUT_L(lut[i * 2].red);
		dqe_write(DQE0_REGAMMALUT_R(i), val);
		cal_log_debug(0, "[%d]   red: 0x%x\n", i, val);

		val = REGAMMA_LUT_H(lut[i * 2 + 1].green) |
				REGAMMA_LUT_L(lut[i * 2].green);
		dqe_write(DQE0_REGAMMALUT_G(i), val);
		cal_log_debug(0, "[%d] green: 0x%x\n", i, val);

		val = REGAMMA_LUT_H(lut[i * 2 + 1].blue) |
				REGAMMA_LUT_L(lut[i * 2].blue);
		dqe_write(DQE0_REGAMMALUT_B(i), val);
		cal_log_debug(0, "[%d]  blue: 0x%x\n", i, val);
	}

	cal_log_debug(0, "%s -\n", __func__);
}
