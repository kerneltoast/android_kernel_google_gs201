/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for Samsung EXYNOS Panel Information.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_PANEL_H__
#define __EXYNOS_PANEL_H__

#include <linux/kernel.h>
#include <drm/display/drm_dsc.h>

enum type_of_ddi {
	TYPE_OF_SM_DDI = 0,
	TYPE_OF_MAGNA_DDI,
	TYPE_OF_NORMAL_DDI,
};

#define MAX_RES_NUMBER		5
#define HDR_CAPA_NUM		4

struct lcd_res_info {
	unsigned int width;
	unsigned int height;
	unsigned int dsc_en;
	unsigned int dsc_width;
	unsigned int dsc_height;
};

struct lcd_mres_info {
	u32 en;
	u32 number;
	struct lcd_res_info res_info[MAX_RES_NUMBER];
};

struct lcd_hdr_info {
	unsigned int num;
	unsigned int type[HDR_CAPA_NUM];
	unsigned int max_luma;
	unsigned int max_avg_luma;
	unsigned int min_luma;
};

struct dpu_panel_timing {
	unsigned int vactive;
	unsigned int vfp;
	unsigned int vsa;
	unsigned int vbp;

	unsigned int hactive;
	unsigned int hfp;
	unsigned int hsa;
	unsigned int hbp;

	unsigned int vrefresh;

	unsigned int te_idle_us;
	unsigned int te_var;
};

/*
 * dec_sw : slice width in DDI side
 * enc_sw : slice width in AP(DECON & DSIM) side
 */
struct dsc_slice {
	unsigned int dsc_dec_sw[MAX_RES_NUMBER];
	unsigned int dsc_enc_sw[MAX_RES_NUMBER];
};

struct exynos_dsc {
	bool enabled;
	bool is_scrv4;
	u32 dsc_count;
	u32 slice_count;
	u32 slice_width;
	u32 slice_height;
	const struct drm_dsc_config *cfg;
};

/* return compressed DSC slice width */
static inline u32 get_comp_dsc_width(const struct exynos_dsc *dsc)
{
	return ALIGN(DIV_ROUND_UP(dsc->slice_width, 3), 4);
}

static inline u32 decon_get_comp_dsc_width(u32 slice_width, u32 bpc)
{
	unsigned int slice_width_pixels = DIV_ROUND_UP(slice_width * bpc, 8);

	return DIV_ROUND_UP(slice_width_pixels, 6) * 2;
}

#endif /* __EXYNOS_PANEL_H__ */
