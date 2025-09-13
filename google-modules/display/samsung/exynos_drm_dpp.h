/* SPDX-License-Identifier: GPL-2.0-only
 * exynos_drm_dpp.h
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Authors:
 *	Seong-gyu Park <seongyu.park@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _EXYNOS_DRM_DPP_H_
#define _EXYNOS_DRM_DPP_H_

#include <drm/samsung_drm.h>
#include <drm/drm_fourcc_gs101.h>

#include <dpp_cal.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_dqe.h"

enum EXYNOS9_DPP_FEATURES {
	/* Can reads the graphical image */
	DPP_SUPPORT_GRAPHIC	= 0 << 0,
	/* Can reads ARM Frame Buffer Compression (AFBC) encoded stream */
	DPP_SUPPORT_AFBC	= 1 << 0,
	/* Can reads the graphical image and video image */
	DPP_SUPPORT_VIDEO	= 1 << 1,
	/* Flips the image by X/Y axis */
	DPP_SUPPORT_FLIP	= 1 << 2,
};

enum dpp_state {
	DPP_STATE_OFF = 0,
	DPP_STATE_ON,
	DPP_STATE_HANDOVER,
};

struct eotf_debug_override {
	struct exynos_debug_info info;
#if defined(CONFIG_SOC_ZUMA)
	struct hdr_eotf_lut_v2p2 force_lut;
#else
	struct hdr_eotf_lut force_lut;
#endif
};

struct oetf_debug_override {
	struct exynos_debug_info info;
#if defined(CONFIG_SOC_ZUMA)
	struct hdr_oetf_lut_v2p2 force_lut;
#else
	struct hdr_oetf_lut force_lut;
#endif
};

struct gm_debug_override {
	struct exynos_debug_info info;
	struct hdr_gm_data force_data;
};

struct tm_debug_override {
	struct exynos_debug_info info;
#if defined(CONFIG_SOC_ZUMA)
	struct hdr_tm_data_v2p2 force_data;
#else
	struct hdr_tm_data force_data;
#endif
};

struct exynos_hdr {
	struct exynos_hdr_state state;

	struct eotf_debug_override eotf;
	struct oetf_debug_override oetf;
	struct gm_debug_override gm;
	struct tm_debug_override tm;
	bool fp16_en;
	bool fp16_cvt_en;
};

struct dpp_device {
	struct device *dev;

	u32 id;
	u32 is_support;
	u32 port;
	unsigned long attr;
	enum dpp_state state;

	int dma_irq;
	int dpp_irq;

	const uint32_t *pixel_formats;
	unsigned int num_pixel_formats;

	const uint64_t *format_modifiers;

	struct dpp_regs	regs;
	struct dpp_params_info win_config;

	spinlock_t slock;
	spinlock_t dma_slock;

	int decon_id;		/* connected DECON id */
	unsigned int win_id;	/* connected window id */
	bool is_win_connected;	/* Is dpp connected to window ? */
	bool protection;
	u32 rdma_mst_security;	/* read MST_SECURITY from rdma register */

	/*
	 * comp_src means compression source of input buffer compressed by
	 * AFBC encoder of G2D or GPU.
	 *
	 * comp_src in dpu_bts_win_config structure is the information of
	 * the buffer requested by user-side.
	 *
	 * But comp_src in dpp_device structure is the information of the buffer
	 * already applied to the HW.
	 */
	u64 comp_src;
	u32 recovery_cnt;

	struct dpp_restriction restriction;

	int (*check)(struct dpp_device *this_dpp,
				const struct exynos_drm_plane_state *state);
	int (*update)(struct dpp_device *this_dpp,
				struct exynos_drm_plane_state *state);
	int (*disable)(struct dpp_device *this_dpp);

	/*
	 * this dma_addr is used for debugging purposes only, should look at
	 * plane->state->fb for current framebuffer instead
	 */
	dma_addr_t dbg_dma_addr;
	struct exynos_drm_plane plane;

	struct exynos_hdr hdr;
};

struct exynos_dma {
	int id;
	void __iomem *regs;
	int dma_irq;
	spinlock_t dma_slock;
};

#ifdef CONFIG_OF
struct dpp_device *of_find_dpp_by_node(struct device_node *np);
#else
static struct dpp_device *of_find_dpp_by_node(struct device_node *np)
{
	return NULL;
}
#endif

void dpp_dump(struct drm_printer *p, struct dpp_device *dpp);
void rcd_dump(struct drm_printer *p, struct dpp_device *dpp);
void dpp_dump_buffer(struct drm_printer *p, struct dpp_device *dpp);
void cgc_dump(struct drm_printer *p, struct exynos_dma *dma);
bool dpp_need_enable_hdr(const struct dpp_device *dpp);

static __always_inline const char *get_comp_src_name(u64 comp_src)
{
	if (comp_src == AFBC_FORMAT_MOD_SOURCE_GPU)
		return "GPU";
	else if (comp_src == AFBC_FORMAT_MOD_SOURCE_G2D)
		return "G2D";
	else
		return "";
}

struct exynos_dma *exynos_cgc_dma_register(struct decon_device *decon);
#endif
