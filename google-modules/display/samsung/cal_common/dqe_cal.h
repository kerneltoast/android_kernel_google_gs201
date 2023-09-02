/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS101 DQE CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DQE_CAL_H__
#define __SAMSUNG_DQE_CAL_H__

#include <linux/types.h>
#include <drm/samsung_drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_print.h>
#include <cal_config.h>

#define DEGAMMA_LUT_SIZE		65
#define REGAMMA_LUT_SIZE		65
#define CGC_LUT_SIZE			4913
#define HIST_BIN_SIZE			256
#define LPD_ATC_REG_CNT			45
#define GAMMA_MATRIX_COEFFS_CNT		9
#define GAMMA_MATRIX_OFFSETS_CNT	3
#define LINEAR_MATRIX_COEFFS_CNT	9
#define LINEAR_MATRIX_OFFSETS_CNT	3

enum dqe_version {
	DQE_V1, 		/* GS101(9845) EVT0/A0 */
	DQE_V2, 		/* GS101(9845) EVT1/B0 */
	DQE_V3, 		/* GS201(9855) */
	DQE_VERSION_MAX,
};

struct cal_regs_dqe {
	struct cal_regs_desc desc;
	enum dqe_version version;
};

enum dqe_regs_id {
	REGS_DQE0_ID = 0,
	REGS_DQE1_ID,
	REGS_DQE_ID_MAX,
};

extern struct cal_regs_dqe regs_dqe[REGS_DQE_ID_MAX];

/*
 * There are several types of DQE versions.
 * Some SFR's offset might be different as DQE versions.
 * This table uses to adjust for real offset.
 */
struct cal_regs_offset {
	u32 dither_offset;      /* CGC and disp dither */
	u32 matrix_offset;      /* linear and gamma matrix */
	u32 degamma_offset;
	u32 cgc_offset;
	u32 regamma_offset;
	u32 hist_offset;
};

static struct cal_regs_offset regs_dqe_offset[DQE_VERSION_MAX] = {
	{0x0,   0x0,   0x0,   0x0,   0x0,   0x0},       /* GS101(9845) EVT0/A0 */
	{0x400, 0x800, 0x800, 0x800, 0x400, 0x400},     /* GS101(9845) EVT1/B0 */
	{0x400, 0x800, 0x800, 0x800, 0x400, 0x400},	/* GS201(9855) */
};


#define dqe_regs_desc(dqe_id)				(&regs_dqe[dqe_id].desc)
#define dqe_read(dqe_id, offset)			\
	cal_read(dqe_regs_desc(dqe_id), offset)
#define dqe_write(dqe_id, offset, val)			\
	cal_write(dqe_regs_desc(dqe_id), offset, val)
#define dqe_read_mask(dqe_id, offset, mask)		\
	cal_read_mask(dqe_regs_desc(dqe_id), offset, mask)
#define dqe_write_mask(dqe_id, offset, val, mask)	\
	cal_write_mask(dqe_regs_desc(dqe_id), offset, val, mask)
#define dqe_read_relaxed(dqe_id, offset)		\
	cal_read_relaxed(dqe_regs_desc(dqe_id), offset)
#define dqe_write_relaxed(dqe_id, offset, val)		\
	cal_write_relaxed(dqe_regs_desc(dqe_id), offset, val)

#define dither_offset(ver)				(regs_dqe_offset[ver].dither_offset)
#define dither_read(dqe_id, offset)			\
	dqe_read(dqe_id, offset + dither_offset(regs_dqe[dqe_id].version))
#define dither_write(dqe_id, offset, val)		\
	dqe_write(dqe_id, offset + dither_offset(regs_dqe[dqe_id].version), val)

#define matrix_offset(ver)				(regs_dqe_offset[ver].matrix_offset)
#define matrix_write(dqe_id, offset, val)		\
	dqe_write(dqe_id, offset + matrix_offset(regs_dqe[dqe_id].version), val)
#define matrix_write_relaxed(dqe_id, offset, val)	\
	dqe_write_relaxed(dqe_id, offset + matrix_offset(regs_dqe[dqe_id].version), val)
#define matrix_read_mask(dqe_id, offset, mask)		\
	dqe_read_mask(dqe_id, offset + matrix_offset(regs_dqe[dqe_id].version), mask)

#define degamma_offset(ver)				(regs_dqe_offset[ver].degamma_offset)
#define degamma_read(dqe_id, offset)			\
	dqe_read(dqe_id, offset + degamma_offset(regs_dqe[dqe_id].version))
#define degamma_write(dqe_id, offset, val)		\
	dqe_write(dqe_id, offset + degamma_offset(regs_dqe[dqe_id].version), val)
#define degamma_write_relaxed(dqe_id, offset, val)	\
	dqe_write_relaxed(dqe_id, offset + degamma_offset(regs_dqe[dqe_id].version), val)

#define cgc_offset(ver)					(regs_dqe_offset[ver].cgc_offset)
#define cgc_read_mask(dqe_id, offset, mask)		\
	dqe_read_mask(dqe_id, offset + cgc_offset(regs_dqe[dqe_id].version), mask)
#define cgc_write_mask(dqe_id, offset, val, mask)	\
	dqe_write_mask(dqe_id, offset + cgc_offset(regs_dqe[dqe_id].version), val, mask)

#define regamma_offset(ver)				(regs_dqe_offset[ver].regamma_offset)
#define regamma_read(dqe_id, offset)			\
	dqe_read(dqe_id, offset + regamma_offset(regs_dqe[dqe_id].version))
#define regamma_write(dqe_id, offset, val)		\
	dqe_write(dqe_id, offset + regamma_offset(regs_dqe[dqe_id].version), val)
#define regamma_write_relaxed(dqe_id, offset, val)	\
	dqe_write_relaxed(dqe_id, offset + regamma_offset(regs_dqe[dqe_id].version), val)

#define hist_offset(ver)				(regs_dqe_offset[ver].hist_offset)
#define hist_read(dqe_id, offset)			\
	dqe_read(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version))
#define hist_read_mask(dqe_id, offset, mask)		\
	dqe_read_mask(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), mask)
#define hist_write(dqe_id, offset, val)			\
	dqe_write(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), val)
#define hist_write_mask(dqe_id, offset, val, mask)	\
	dqe_write_mask(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), val, mask)
#define hist_read_relaxed(dqe_id, offset)		\
	dqe_read_relaxed(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version))


enum dqe_dither_type {
	CGC_DITHER = 0,
	DISP_DITHER = 1,
};

enum histogram_state {
	HISTOGRAM_OFF,
	HISTOGRAM_FULL,
	HISTOGRAM_ROI,
};

struct exynos_atc {
	bool en;
	bool dirty;
	__u8 lt;
	__u8 ns;
	__u8 st;
	bool dither;
	__u8 pl_w1;
	__u8 pl_w2;
	__u8 ctmode;
	bool pp_en;
	__u8 upgrade_on;
	__u16 tdr_max;
	__u16 tdr_min;
	__u8 ambient_light;
	__u8 back_light;
	__u8 dstep;
	__u8 actual_dstep;
	__u8 scale_mode;
	__u8 threshold_1;
	__u8 threshold_2;
	__u8 threshold_3;
	__u16 gain_limit;
	__u8 lt_calc_ab_shift;
};

void dqe_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
			enum dqe_version ver, u32 dqe_id);
void dqe_reg_init(u32 dqe_id, u32 width, u32 height);
void dqe_reg_set_degamma_lut(u32 dqe_id, const struct drm_color_lut *lut);
void dqe_reg_set_cgc_lut(u32 dqe_id, const struct cgc_lut *lut);
void dqe_reg_set_regamma_lut(u32 dqe_id, const struct drm_color_lut *lut);
void dqe_reg_set_cgc_dither(u32 dqe_id, struct dither_config *config);
void dqe_reg_set_disp_dither(u32 dqe_id, struct dither_config *config);
void dqe_reg_set_linear_matrix(u32 dqe_id, const struct exynos_matrix *lm);
void dqe_reg_set_gamma_matrix(u32 dqe_id, const struct exynos_matrix *matrix);
void dqe_reg_set_atc(u32 dqe_id, const struct exynos_atc *atc);
void dqe_reg_print_dither(u32 dqe_id, enum dqe_dither_type dither,
			  struct drm_printer *p);
void dqe_reg_print_degamma_lut(u32 dqe_id, struct drm_printer *p);
void dqe_reg_print_cgc_lut(u32 dqe_id, u32 count, struct drm_printer *p);
void dqe_reg_print_regamma_lut(u32 dqe_id, struct drm_printer *p);
void dqe_reg_print_hist(u32 dqe_id, struct drm_printer *p);
void dqe_reg_print_gamma_matrix(u32 dqe_id, struct drm_printer *p);
void dqe_reg_print_linear_matrix(u32 dqe_id, struct drm_printer *p);
void dqe_reg_print_atc(u32 dqe_id, struct drm_printer *p);
void dqe_reg_save_lpd_atc(u32 dqe_id, u32 *lpd_atc_regs);
void dqe_reg_restore_lpd_atc(u32 dqe_id, u32 *lpd_atc_regs);
bool dqe_reg_dimming_in_progress(u32 dqe_id);
void dqe_reg_set_histogram_roi(u32 dqe_id, struct histogram_roi *roi);
void dqe_reg_set_histogram_weights(u32 dqe_id, struct histogram_weights *weights);
void dqe_reg_set_histogram_threshold(u32 dqe_id, u32 threshold);
void dqe_reg_set_histogram(u32 dqe_id, enum histogram_state state);
void dqe_reg_get_histogram_bins(u32 dqe_id, struct histogram_bins *bins);
void dqe_reg_set_histogram_pos(u32 dqe_id, enum exynos_prog_pos pos);
void dqe_reg_set_size(u32 dqe_id, u32 width, u32 height);
void dqe_dump(struct drm_printer *p, u32 dqe_id);
void dqe_reg_set_rcd_en(u32 dqe_id, bool en);
void dqe_reg_set_drm_write_protected(u32 dqe_id, bool protected);
void dqe_reg_set_cgc_coef_dma_req(u32 dqe_id);
void dqe_reg_set_cgc_en(u32 dqe_id, bool en);
void dqe_reg_wait_cgc_dma_done(u32 dqe_id, u32 timeout_us);
#endif /* __SAMSUNG_DQE_CAL_H__ */
