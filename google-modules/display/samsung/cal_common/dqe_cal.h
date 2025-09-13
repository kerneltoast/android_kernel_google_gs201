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

#include <cal_config.h>
#include <drm/drm_mode.h>
#include <drm/drm_print.h>
#include <drm/samsung_drm.h>
#include <linux/dma-direct.h>
#include <linux/types.h>
#if defined(CONFIG_SOC_ZUMA)
#include <linux/soc/samsung/exynos-smc.h>
#endif

#ifdef CONFIG_SOC_ZUMA
#define DEGAMMA_LUT_SIZE		66
#define REGAMMA_LUT_SIZE		66
#else
#define DEGAMMA_LUT_SIZE		65
#define REGAMMA_LUT_SIZE		65
#endif
#define CGC_LUT_SIZE			4913
#define HIST_BIN_SIZE			256
#define LPD_ATC_REG_CNT			45
#define GAMMA_MATRIX_COEFFS_CNT		9
#define GAMMA_MATRIX_OFFSETS_CNT	3
#define LINEAR_MATRIX_COEFFS_CNT	9
#define LINEAR_MATRIX_OFFSETS_CNT	3

#define CGC_EN(_v)			((_v) << 0)
#define CGC_EN_MASK			(1 << 0)

enum dqe_version {
	DQE_V1, 		/* GS101(9845) EVT0/A0 */
	DQE_V2, 		/* GS101(9845) EVT1/B0 */
	DQE_V3, 		/* GS201(9855) */
	DQE_V4, 		/* ZUMA(9865)  */
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

#ifdef CONFIG_SOC_ZUMA
enum exynos_histogram_id {
	HISTOGRAM_0,
	HISTOGRAM_1,
	HISTOGRAM_2,
	HISTOGRAM_3,
	HISTOGRAM_MAX,
};

#define HISTOGRAM_CHAN_LHBM HISTOGRAM_3
#else
enum exynos_histogram_id {
	HISTOGRAM_0,
	HISTOGRAM_MAX,
};
#endif

extern struct cal_regs_dqe regs_dqe[REGS_DQE_ID_MAX];
extern struct cal_regs_dqe regs_dqe_cgc[REGS_DQE_ID_MAX];

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
	{0x400, 0x800, 0x800, 0x800, 0x400, 0x400},	/* Zuma(9865) */
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

#define dqe_cgc_regs_desc(dqe_id)			(&regs_dqe_cgc[dqe_id].desc)
#define dqe_cgc_read(dqe_id, offset)			\
	cal_read(dqe_cgc_regs_desc(dqe_id), offset)
#define dqe_cgc_read_mask(dqe_id, offset, mask)			\
	cal_read(dqe_cgc_regs_desc(dqe_id), offset)
#define dqe_cgc_write_mask(dqe_id, offset, val, mask)	\
	cal_write_mask(dqe_cgc_regs_desc(dqe_id), offset, val, mask)
#define dqe_cgc_write_relaxed(dqe_id, offset, val)		\
	cal_write_relaxed(dqe_cgc_regs_desc(dqe_id), offset, val)

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

#if defined(CONFIG_SOC_ZUMA)
static inline uint32_t dqe_smc_read(u32 dqe_id, u32 offset)
{
	struct cal_regs_desc *rdesc = dqe_regs_desc(dqe_id);
	return (u32)exynos_smc(SMC_DRM_HISTOGRAM_SEC, (u32)rdesc->start + offset, 0, 0);
}

static inline uint32_t dqe_smc_read_mask(u32 dqe_id, u32 offset, u32 mask)
{
	return (dqe_smc_read(dqe_id, offset) & mask);
}

static inline uint32_t dqe_smc_write(u32 dqe_id, u32 offset, u32 val)
{
	struct cal_regs_desc *rdesc = dqe_regs_desc(dqe_id);
	return ((u32)exynos_smc(SMC_DRM_HISTOGRAM_SEC, (u32)rdesc->start + offset, 1, val));
}

static inline uint32_t dqe_smc_write_mask(u32 dqe_id, u32 offset, u32 val, u32 mask)
{
	uint32_t cur = dqe_smc_read(dqe_id, offset);

	val = (val & mask) | (cur & ~mask);
	return dqe_smc_write(dqe_id, offset, val);
}

#define hist_offset(ver)	(regs_dqe_offset[ver].hist_offset)
#define hist_read(dqe_id, offset)	\
	dqe_smc_read(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version))
#define hist_read_mask(dqe_id, offset, mask)	\
	dqe_smc_read_mask(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), mask)
#define hist_write(dqe_id, offset, val)	\
	dqe_smc_write(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), val)
#define hist_write_mask(dqe_id, offset, val, mask)	\
	dqe_smc_write_mask(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), val, mask)
#else
#define hist_offset(ver)	(regs_dqe_offset[ver].hist_offset)
#define hist_read(dqe_id, offset)	\
	dqe_read(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version))
#define hist_read_mask(dqe_id, offset, mask)	\
	dqe_read_mask(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), mask)
#define hist_write(dqe_id, offset, val)	\
	dqe_write(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), val)
#define hist_write_mask(dqe_id, offset, val, mask)	\
	dqe_write_mask(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version), val, mask)
#define hist_read_relaxed(dqe_id, offset)	\
	dqe_read_relaxed(dqe_id, offset + hist_offset(regs_dqe[dqe_id].version))
#endif

enum dqe_dither_type {
	CGC_DITHER = 0,
	DISP_DITHER = 1,
};

/**
 * enum histogram_state - defines histrogram channel state
 */
enum histogram_state {
	HISTOGRAM_OFF,
	HISTOGRAM_FULL, /* ON, ROI is disabled, full screen */
	HISTOGRAM_ROI, /* ON, ROI is enabled */
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	HISTOGRAM_BLOCKED_FULL,
	HISTOGRAM_BLOCKED_ROI,
#endif
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
	__u16 dim_ratio;
#ifdef CONFIG_SOC_ZUMA
	bool la_w_on;
	__u8 la_w;
	bool lt_calc_mode;
	__u8 gt_lamda_dstep;
	__u16 gt_lamda;
	bool gt_he_enable;
	__u32 he_clip_min_0;
	__u32 he_clip_min_1;
	__u32 he_clip_min_2;
	__u32 he_clip_min_3;
	__u32 he_clip_max_0;
	__u32 he_clip_max_1;
	__u32 he_clip_max_2;
	__u32 he_clip_max_3;
#endif
};

#if defined(CONFIG_SOC_GS201)
void dqe_reg_set_histogram_pos_internal(u32 id, enum exynos_histogram_id hist_id,
					enum histogram_prog_pos pos);
#elif defined(CONFIG_SOC_ZUMA)
void dqe_reg_set_histogram_pos_internal(u32 id, enum exynos_histogram_id hist_id,
					enum histogram_prog_pos pos);
void dqe_reg_set_atc_he(u32 dqe_id, const struct exynos_atc *atc);
void dqe_reg_print_atc_he(u32 dqe_id, struct drm_printer *p);
#else
/* Stubs for non-gs201 SoCs */
static inline void dqe_reg_set_rcd_en_internal(u32 id, bool en) {}
static inline void dqe_reg_set_histogram_pos_internal(u32 id, enum exynos_histogram_id hist_id,
						      enum histogram_prog_pos pos)
{
	return;
}
static inline void dqe_reg_set_atc_he(u32 dqe_id, const struct exynos_atc *atc) {}
static inline void dqe_reg_print_atc_he(u32 dqe_id, struct drm_printer *p) {}
#endif

#if defined(CONFIG_SOC_GS201) || defined(CONFIG_SOC_ZUMA)
void dqe_reg_set_cgc_en_internal(u32 dqe_id, bool en);
void dqe_reg_set_cgc_coef_dma_req_internal(u32 dqe_id);
int dqe_reg_wait_cgc_dma_done_internal(u32 id, unsigned long timeout_us);
void dqe_reg_set_rcd_en_internal(u32 id, bool en);
#else
static inline void dqe_reg_set_cgc_en_internal(u32 dqe_id, bool en) {return; }
static inline void dqe_reg_set_cgc_coef_dma_req_internal(u32 dqe_id) {return; }
static inline int dqe_reg_wait_cgc_dma_done_internal(u32 id, unsigned long timeout_us) {return 0; }
#endif

#if defined(CONFIG_SOC_ZUMA)
void dqe_cgc_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
			    enum dqe_version ver, unsigned int dqe_id);
#else
static inline void dqe_cgc_regs_desc_init(void __iomem *regs, phys_addr_t start,
					  const char *name, enum dqe_version ver,
					  unsigned int dqe_id) {return; }
#endif

void dqe_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
			enum dqe_version ver, u32 dqe_id);
void dqe_reg_init(u32 dqe_id, u32 width, u32 height);
void dqe_reg_set_degamma_lut(u32 dqe_id, const struct drm_color_lut *lut);
void dqe_reg_set_cgc_lut(u32 dqe_id, const struct cgc_lut *lut);
void dqe_reg_set_regamma_lut(u32 dqe_id, u32 regamma_id, const struct drm_color_lut *lut);
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
void dqe_reg_print_hist_ch(u32 dqe_id, u32 hist_id, struct drm_printer *p);
static inline void dqe_reg_print_hist(u32 dqe_id, struct drm_printer *p) {
	int i;

	for (i = 0; i < HISTOGRAM_MAX; i++)
		dqe_reg_print_hist_ch(dqe_id, i, p);
}
void dqe_reg_print_gamma_matrix(u32 dqe_id, struct drm_printer *p);
void dqe_reg_print_linear_matrix(u32 dqe_id, struct drm_printer *p);
void dqe_reg_print_atc(u32 dqe_id, struct drm_printer *p);
void dqe_reg_save_lpd_atc(u32 dqe_id, u32 *lpd_atc_regs);
void dqe_reg_restore_lpd_atc(u32 dqe_id, u32 *lpd_atc_regs);
bool dqe_reg_dimming_in_progress(u32 dqe_id);
void dqe_reg_set_histogram_roi(u32 dqe_id, enum exynos_histogram_id hist_id,
			       struct histogram_roi *roi);
#if defined(CONFIG_SOC_ZUMA)
void dqe_reg_set_histogram_block_roi(u32 dqe_id, enum exynos_histogram_id hist_id,
				     struct histogram_roi *roi);
#endif
void dqe_reg_set_histogram_weights(u32 dqe_id, enum exynos_histogram_id hist_id,
				   struct histogram_weights *weights);
void dqe_reg_set_histogram_threshold(u32 dqe_id, enum exynos_histogram_id hist_id, u32 threshold);
void dqe_reg_set_histogram(u32 dqe_id, enum exynos_histogram_id hist_id,
			   enum histogram_state state);
void dqe_reg_get_histogram_bins(struct device *dev, u32 dqe_id, enum exynos_histogram_id hist_id,
				struct histogram_bins *bins);
static inline void dqe_reg_set_histogram_pos(u32 dqe_id, enum exynos_histogram_id hist_id,
					     enum histogram_prog_pos pos)
{
	dqe_reg_set_histogram_pos_internal(dqe_id, hist_id, pos);
}
void dqe_reg_set_size(u32 dqe_id, u32 width, u32 height);
void dqe_dump(struct drm_printer *p, u32 dqe_id);
static inline void dqe_reg_set_rcd_en(u32 dqe_id, bool en)
{
	dqe_reg_set_rcd_en_internal(dqe_id, en);
}

void dqe_reg_set_drm_write_protected(u32 dqe_id, bool protected);
static inline void dqe_reg_set_cgc_en(u32 dqe_id, bool en)
{
	dqe_reg_set_cgc_en_internal(dqe_id, en);
}
static inline void dqe_reg_wait_cgc_dma_done(u32 dqe_id, u32 timeout_us)
{
	dqe_reg_wait_cgc_dma_done_internal(dqe_id, timeout_us);
}
static inline void dqe_reg_set_cgc_coef_dma_req(u32 dqe_id)
{
	dqe_reg_set_cgc_coef_dma_req_internal(dqe_id);
}

#endif /* __SAMSUNG_DQE_CAL_H__ */
