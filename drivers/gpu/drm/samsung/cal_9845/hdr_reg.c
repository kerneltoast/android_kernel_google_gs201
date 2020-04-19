// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register access functions for Samsung HDR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <cal_config.h>
#include <regs-hdr.h>
#include <hdr_cal.h>
#include <dpp_cal.h>

static struct cal_regs_desc regs_hdr[REGS_DPP_ID_MAX - 1];

#define hdr_regs_desc(id)			(&regs_hdr[id])
#define hdr_read(id, offset)			\
	cal_read(hdr_regs_desc(id), offset)
#define hdr_write(id, offset, val)		\
	cal_write(hdr_regs_desc(id), offset, val)
#define hdr_read_mask(id, offset, mask)		\
	cal_read_mask(hdr_regs_desc(id), offset, mask)
#define hdr_write_mask(id, offset, val, mask)	\
	cal_write_mask(hdr_regs_desc(id), offset, val, mask)

void hdr_regs_desc_init(void __iomem *regs, const char *name, u32 id)
{
	regs_hdr[id].regs = regs;
	regs_hdr[id].name = name;
}

void hdr_reg_set_hdr(u32 id, bool en)
{
	cal_log_debug(0, "%s +\n", __func__);
	hdr_write_mask(id, HDR_LSI_L_COM_CTRL, en ? ~0 : 0,
			COM_CTRL_ENABLE_MASK);
	cal_log_debug(0, "%s -\n", __func__);
}

void hdr_reg_set_eotf_lut(u32 id, struct hdr_eotf_lut *lut)
{
	int i;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lut) {
		hdr_write_mask(id, HDR_LSI_L_MOD_CTRL, MOD_CTRL_EEN(0),
				MOD_CTRL_EEN_MASK);
		return;
	}

	hdr_write_mask(id, HDR_LSI_L_MOD_CTRL, MOD_CTRL_EEN(1),
			MOD_CTRL_EEN_MASK);

	for (i = 0; i < HDR_EOTF_POSX_LUT_REG_CNT; i++) {
		val = EOTF_POSX_H(lut->posx[i * 2 + 1]) |
			EOTF_POSX_L(lut->posx[i * 2]);
		hdr_write(id, HDR_LSI_L_EOTF_POSX(i), val);
		cal_log_debug(0, "POSX[%d]: 0x%x\n", i, val);
	}

	for (i = 0; i < HDR_EOTF_POSY_LUT_REG_CNT; i++) {
		hdr_write(id, HDR_LSI_L_EOTF_POSY(i), lut->posy[i]);
		cal_log_debug(0, "POSY[%d]: 0x%x\n", i, lut->posy[i]);
	}

	cal_log_debug(0, "%s -\n", __func__);
}

void hdr_reg_set_oetf_lut(u32 id, struct hdr_oetf_lut *lut)
{
	int i;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lut) {
		hdr_write_mask(id, HDR_LSI_L_MOD_CTRL, MOD_CTRL_OEN(0),
				MOD_CTRL_OEN_MASK);
		return;
	}

	hdr_write_mask(id, HDR_LSI_L_MOD_CTRL, MOD_CTRL_OEN(1),
			MOD_CTRL_OEN_MASK);

	for (i = 0; i < HDR_OETF_POSX_LUT_REG_CNT; i++) {
		val = OETF_POSX_H(lut->posx[i * 2 + 1]) |
			OETF_POSX_L(lut->posx[i * 2]);
		hdr_write(id, HDR_LSI_L_OETF_POSX(i), val);
		cal_log_debug(0, "POSX[%d]: 0x%x\n", i, val);
	}

	for (i = 0; i < HDR_OETF_POSY_LUT_REG_CNT; i++) {
		val = OETF_POSY_H(lut->posy[i * 2 + 1]) |
			OETF_POSY_L(lut->posy[i * 2]);
		hdr_write(id, HDR_LSI_L_OETF_POSY(i), val);
		cal_log_debug(0, "POSY[%d]: 0x%x\n", i, val);
	}

	cal_log_debug(0, "%s -\n", __func__);
}

/*
 * |Rout| = |C00 C01 C02| |Rin| + |offset0|
 * |Gout| = |C10 C11 C12| |Gin| + |offset1|
 * |Bout| = |C20 C21 C22| |Bin| + |offset2|
 */
void hdr_reg_set_gm(u32 id, struct hdr_gm_data *data)
{
	int i;

	cal_log_debug(0, "%s +\n", __func__);

	if (!data) {
		hdr_write_mask(id, HDR_LSI_L_MOD_CTRL, MOD_CTRL_GEN(0),
				MOD_CTRL_GEN_MASK);
		return;
	}

	hdr_write_mask(id, HDR_LSI_L_MOD_CTRL, MOD_CTRL_GEN(1),
			MOD_CTRL_GEN_MASK);

	for (i = 0; i < HDR_GM_COEF_REG_CNT; ++i) {
		hdr_write(id, HDR_LSI_L_GM_COEF(i), data->coeffs[i]);
		cal_log_debug(0, "COEFFS[%d]: 0x%x\n", i, data->coeffs[i]);
	}

	for (i = 0; i < HDR_GM_OFFS_REG_CNT; ++i) {
		hdr_write(id, HDR_LSI_L_GM_OFFS(i), data->offsets[i]);
		cal_log_debug(0, "OFFSETS[%d]: 0x%x\n", i, data->offsets[i]);
	}

	cal_log_debug(0, "%s -\n", __func__);
}

void hdr_reg_set_tm(u32 id, struct hdr_tm_data *tm)
{
	int i;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!tm) {
		hdr_write_mask(id, HDR_LSI_L_MOD_CTRL, 0, MOD_CTRL_TEN_MASK);
		return;
	}

	hdr_write_mask(id, HDR_LSI_L_MOD_CTRL, ~0, MOD_CTRL_TEN_MASK);

	val = TM_COEFB(tm->coeff_b) | TM_COEFG(tm->coeff_g) |
		TM_COEFR(tm->coeff_r);
	hdr_write(id, HDR_LSI_L_TM_COEF, val);
	cal_log_debug(0, "COEFF: 0x%x\n", val);

	val = TM_RNGX_MAXX(tm->rng_x_max) | TM_RNGX_MINX(tm->rng_x_min);
	hdr_write(id, HDR_LSI_L_TM_RNGX, val);
	cal_log_debug(0, "RNGX: 0x%x\n", val);

	val = TM_RNGY_MAXY(tm->rng_y_max) | TM_RNGY_MINY(tm->rng_y_min);
	hdr_write(id, HDR_LSI_L_TM_RNGY, val);
	cal_log_debug(0, "RNGY: 0x%x\n", val);

	for (i = 0; i < HDR_TM_POSX_LUT_REG_CNT; i++) {
		val = TM_POSX_H(tm->posx[i * 2 + 1]) |
			TM_POSX_L(tm->posx[i * 2]);
		hdr_write(id, HDR_LSI_L_TM_POSX(i), val);
		cal_log_debug(0, "POSX[%d]: 0x%x\n", i, val);
	}

	for (i = 0; i < HDR_TM_POSY_LUT_REG_CNT; i++) {
		hdr_write(id, HDR_LSI_L_TM_POSY(i), tm->posy[i]);
		cal_log_debug(0, "POSY[%d]: 0x%x\n", i, tm->posy[i]);
	}

	cal_log_debug(0, "%s -\n", __func__);
}
