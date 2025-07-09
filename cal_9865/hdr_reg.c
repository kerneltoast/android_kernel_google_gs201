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
#include <hdr_cal.h>
#include <dpp_cal.h>
#include <drm/drm_print.h>

#include "regs-hdr.h"

static struct cal_regs_desc regs_hdr[REGS_DPP_ID_MAX];

#define hdr_regs_desc(id)			(&regs_hdr[id])
#define hdr_read(id, offset)			\
	cal_read(hdr_regs_desc(id), offset)
#define hdr_write(id, offset, val)		\
	cal_write(hdr_regs_desc(id), offset, val)
#define hdr_read_mask(id, offset, mask)		\
	cal_read_mask(hdr_regs_desc(id), offset, mask)
#define hdr_write_mask(id, offset, val, mask)	\
	cal_write_mask(hdr_regs_desc(id), offset, val, mask)
#define hdr_write_relaxed(id, offset, val)	\
	cal_write_relaxed(hdr_regs_desc(id), offset, val)

void hdr_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name, u32 id)
{
	regs_hdr[id].regs = regs;
	regs_hdr[id].name = name;
	regs_hdr[id].start = start;
}

void hdr_reg_set_fp16(u32 id, bool fp16_en, bool fp16_cvt_en)
{
	cal_log_debug(id, "%s +\n", __func__);
	hdr_write_mask(id, HDR_HDR_CON,
			FP16_EN(fp16_en) | FP16_CVT_EN(fp16_cvt_en),
			(FP16_EN_MASK | FP16_CVT_EN_MASK));
	cal_log_debug(id, "%s -\n", __func__);
}

void hdr_reg_set_hdr(u32 id, bool en)
{
	cal_log_debug(id, "%s +\n", __func__);
	hdr_write_mask(id, HDR_HDR_CON, en ? ~0 : 0, HDR_EN_MASK);
	cal_log_debug(id, "%s -\n", __func__);
}

void hdr_reg_set_eotf_lut(u32 id, struct hdr_eotf_lut_v2p2 *lut)
{
	int i;
	u32 val;

	cal_log_debug(id, "%s +\n", __func__);

	if (!lut) {
		hdr_write_mask(id, HDR_HDR_CON, EOTF_EN(0), EOTF_EN_MASK);
		return;
	}

	for (i = 0; i < HDR_EOTF_LUT_REG_CNT; i++) {
		val = EOTF_TS_ODD(lut->ts[i].odd) | EOTF_TS_EVEN(lut->ts[i].even);
		hdr_write_relaxed(id, HDR_EOTF_LUT_TS(i), val);
		cal_log_debug(id, "EOTF TS[%d]: 0x%x\n", i, val);
	}

	for (i = 0; i < HDR_EOTF_LUT_REG_CNT; i++) {
		val = EOTF_VS_ODD(lut->vs[i].odd) | EOTF_VS_EVEN(lut->vs[i].even);
		hdr_write_relaxed(id, HDR_EOTF_LUT_VS(i), val);
		cal_log_debug(id, "EOTF VS[%d]: 0x%x\n", i, val);
	}

	hdr_write_relaxed(id, HDR_EOTF_SCALER, EOTF_SCALER(lut->scaler));
	hdr_write_mask(id, HDR_HDR_CON, EOTF_LUT_EN(lut->lut_en), EOTF_LUT_EN_MASK);

	hdr_write_mask(id, HDR_HDR_CON, EOTF_EN(1), EOTF_EN_MASK);

	cal_log_debug(id, "%s -\n", __func__);
}

void hdr_reg_set_oetf_lut(u32 id, struct hdr_oetf_lut_v2p2 *lut)
{
	int i;
	u32 val;

	cal_log_debug(id, "%s +\n", __func__);

	if (!lut) {
		hdr_write_mask(id, HDR_HDR_CON, OETF_EN(0), OETF_EN_MASK);
		return;
	}

	for (i = 0; i < HDR_OETF_LUT_REG_CNT; i++) {
		val = OETF_TS_ODD(lut->ts[i].odd) | OETF_TS_EVEN(lut->ts[i].even);
		hdr_write_relaxed(id, HDR_OETF_LUT_TS(i), val);
		cal_log_debug(id, "OETF TS[%d]: 0x%x\n", i, val);
	}

	for (i = 0; i < HDR_OETF_LUT_REG_CNT; i++) {
		val = OETF_VS_ODD(lut->vs[i].odd) | OETF_VS_EVEN(lut->vs[i].even);
		hdr_write_relaxed(id, HDR_OETF_LUT_VS(i), val);
		cal_log_debug(id, "OETF VS[%d]: 0x%x\n", i, val);
	}

	hdr_write_mask(id, HDR_HDR_CON, OETF_EN(1), OETF_EN_MASK);

	cal_log_debug(id, "%s -\n", __func__);
}

/*
 * |Rout| = |C00 C01 C02| |Rin| + |offset0|
 * |Gout| = |C10 C11 C12| |Gin| + |offset1|
 * |Bout| = |C20 C21 C22| |Bin| + |offset2|
 */
void hdr_reg_set_gm(u32 id, struct hdr_gm_data *data)
{
	int i;

	cal_log_debug(id, "%s +\n", __func__);

	if (!data) {
		hdr_write_mask(id, HDR_HDR_CON, GM_EN(0), GM_EN_MASK);
		return;
	}

	for (i = 0; i < HDR_GM_COEF_REG_CNT; ++i) {
		hdr_write_relaxed(id, HDR_GM_COEFF(i), data->coeffs[i]);
		cal_log_debug(id, "COEFFS[%d]: 0x%x\n", i, data->coeffs[i]);
	}

	for (i = 0; i < HDR_GM_OFFS_REG_CNT; ++i) {
		hdr_write_relaxed(id, HDR_GM_OFF(i), data->offsets[i]);
		cal_log_debug(id, "OFFSETS[%d]: 0x%x\n", i, data->offsets[i]);
	}

	hdr_write_mask(id, HDR_HDR_CON, GM_EN(1), GM_EN_MASK);
	cal_log_debug(id, "%s -\n", __func__);
}

void hdr_reg_set_tm(u32 id, struct hdr_tm_data_v2p2 *tm)
{
	int i;
	u32 val;

	cal_log_debug(id, "%s +\n", __func__);

	if (!tm) {
		hdr_write_mask(id, HDR_HDR_CON, TM_EN(0), TM_EN_MASK);
		return;
	}

	val = TCOEFF00(tm->coeff_00);
	hdr_write_relaxed(id, HDR_TM_COEFF00, val);
	cal_log_debug(id, "COEFF00: 0x%x\n", val);


	val = TCOEFF01(tm->coeff_01);
	hdr_write_relaxed(id, HDR_TM_COEFF01, val);
	cal_log_debug(id, "COEFF01: 0x%x\n", val);

	val = TCOEFF02(tm->coeff_02);
	hdr_write_relaxed(id, HDR_TM_COEFF02, val);
	cal_log_debug(id, "COEFF02: 0x%x\n", val);

	val = YMIX_TF(tm->ymix_tf);
	hdr_write_relaxed(id, HDR_TM_YMIX_TF, val);
	cal_log_debug(id, "YMIX_TF: 0x%x\n", val);

	val = YMIX_VF(tm->ymix_vf);
	hdr_write_relaxed(id, HDR_TM_YMIX_VF, val);
	cal_log_debug(id, "YMIX_VF: 0x%x\n", val);

	val = YMIX_TF(tm->ymix_slope);
	hdr_write_relaxed(id, HDR_TM_YMIX_SLOPE, val);
	cal_log_debug(id, "YMIX_SLOPE: 0x%x\n", val);

	val = YMIX_DV(tm->ymix_dv);
	hdr_write_relaxed(id, HDR_TM_YMIX_DV, val);
	cal_log_debug(id, "YMIX_DV: 0x%x\n", val);

	for (i = 0; i < HDR_TM_LUT_REG_CNT; i++) {
		val = TM_TS_ODD(tm->ts[i].odd) | TM_TS_EVEN(tm->ts[i].even);
		hdr_write_relaxed(id, HDR_TM_LUT_TS(i), val);
		cal_log_debug(id, "TS[%d]: 0x%x\n", i, val);
	}

	for (i = 0; i < HDR_TM_LUT_REG_CNT; i++) {
		val = TM_VS_ODD(tm->vs[i].odd) | TM_VS_EVEN(tm->vs[i].even);
		hdr_write_relaxed(id, HDR_TM_LUT_VS(i), val);
		cal_log_debug(id, "VS[%d]: 0x%x\n", i, val);
	}

	hdr_write_mask(id, HDR_HDR_CON, TM_EN(1), TM_EN_MASK);

	cal_log_debug(id, "%s -\n", __func__);
}

static void hdr_reg_print(u32 id, u32 start, u32 count, enum elem_size size,
							struct drm_printer *pr)
{
	u32 reg, val;
	int idx, i;
	char buf[256];
	char *p = buf;
	const char *end = buf + sizeof(buf);
	bool is_32 = (size == ELEM_SIZE_32);
	u32 pcount;

	pcount = is_32 ? count : DIV_ROUND_UP(count, 2);

	for (i = 0; i < pcount; ++i) {
		reg = hdr_read(id, start + i * 4);

		idx = is_32 ? i : i * 2;
		val = is_32 ? reg : GET_LUT_L(reg);

		p += scnprintf(p, end - p, "[%4d]%8x ", idx, val);

		if (((i * 2 + 1) != count) && !is_32)
			p += scnprintf(p, end - p, "[%4d]%8x ", i * 2 + 1,
					GET_LUT_H(reg));

		if ((i % 5) == 4) {
			cal_drm_printf(pr, id, "%s\n", buf);
			p = buf;
		}
	}

	if (p != buf)
		cal_drm_printf(pr, id, "%s\n", buf);
}

void hdr_reg_print_eotf_lut(u32 id, struct drm_printer *p)
{
	u32 val;

	val = hdr_read_mask(id, HDR_HDR_CON, EOTF_EN_MASK);
	cal_drm_printf(p, id, "HDR: eotf %s\n", val ? "on" : "off");

	if (!val)
		return;

	cal_drm_printf(p, id, "TS:\n");
	hdr_reg_print(id, HDR_EOTF_LUT_TS(0), DRM_SAMSUNG_HDR_EOTF_V2P2_LUT_LEN,
							ELEM_SIZE_32, p);
	cal_drm_printf(p, id, "VS:\n");
	hdr_reg_print(id, HDR_EOTF_LUT_VS(0), DRM_SAMSUNG_HDR_EOTF_V2P2_LUT_LEN,
							ELEM_SIZE_32, p);
}

void hdr_reg_print_oetf_lut(u32 id, struct drm_printer *p)
{
	u32 val;

	val = hdr_read_mask(id, HDR_HDR_CON, OETF_EN_MASK);
	cal_drm_printf(p, id, "HDR: oetf %s\n", val ? "on" : "off");

	if (!val)
		return;

	cal_drm_printf(p, id, "TS:\n");
	hdr_reg_print(id, HDR_OETF_LUT_TS(0), DRM_SAMSUNG_HDR_OETF_V2P2_LUT_LEN,
							ELEM_SIZE_32, p);
	cal_drm_printf(p, id, "VS:\n");
	hdr_reg_print(id, HDR_OETF_LUT_VS(0), DRM_SAMSUNG_HDR_OETF_V2P2_LUT_LEN,
							ELEM_SIZE_32, p);
}

void hdr_reg_print_gm(u32 id, struct drm_printer *p)
{
	u32 val;

	val = hdr_read_mask(id, HDR_HDR_CON, GM_EN_MASK);
	cal_drm_printf(p, id, "HDR: gammut %s\n", val ? "on" : "off");

	if (!val)
		return;

	cal_drm_printf(p, id, "COEFFS:\n");
	hdr_reg_print(id, HDR_GM_COEFF(0), HDR_GM_COEF_REG_CNT,
			ELEM_SIZE_32, p);

	cal_drm_printf(p, id, "OFFSETS:\n");
	hdr_reg_print(id, HDR_GM_OFF(0), HDR_GM_OFFS_REG_CNT,
			ELEM_SIZE_32, p);
}

void hdr_reg_print_tm(u32 id, struct drm_printer *p)
{
	u32 val;

	val = hdr_read_mask(id, HDR_HDR_CON, TM_EN_MASK);
	cal_drm_printf(p, id, "HDR: tone mapping %s\n", val ? "on" : "off");

	cal_drm_printf(p, id, "COEFF00: 0x%x\n", hdr_read(id, HDR_TM_COEFF00));
	cal_drm_printf(p, id, "COEFF01: 0x%x\n", hdr_read(id, HDR_TM_COEFF01));
	cal_drm_printf(p, id, "COEFF01: 0x%x\n", hdr_read(id, HDR_TM_COEFF02));
	cal_drm_printf(p, id, "YMIX_TF: 0x%x\n", hdr_read(id, HDR_TM_YMIX_TF));
	cal_drm_printf(p, id, "YMIX_VF: 0x%x\n", hdr_read(id, HDR_TM_YMIX_VF));
	cal_drm_printf(p, id, "YMIX_SLOPE: 0x%x\n", hdr_read(id, HDR_TM_YMIX_SLOPE));
	cal_drm_printf(p, id, "YMIX_DV: 0x%x\n", hdr_read(id, HDR_TM_YMIX_DV));

	cal_drm_printf(p, id, "TS:\n");
	hdr_reg_print(id, HDR_TM_LUT_TS(0), DRM_SAMSUNG_HDR_TM_V2P2_LUT_LEN,
							ELEM_SIZE_32, p);
	cal_drm_printf(p, id, "VS:\n");
	hdr_reg_print(id, HDR_TM_LUT_VS(0), DRM_SAMSUNG_HDR_TM_V2P2_LUT_LEN,
							ELEM_SIZE_32, p);
}
