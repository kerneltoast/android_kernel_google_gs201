// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * BTS file for Samsung EXYNOS DPU driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <soc/google/bts.h>
#include <soc/google/cal-if.h>

#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
#include <soc/google/exynos-devfreq.h>
#if defined(CONFIG_SOC_GS101)
#include <dt-bindings/soc/google/gs101-devfreq.h>
#elif defined(CONFIG_SOC_GS201)
#include <dt-bindings/soc/google/gs201-devfreq.h>
#elif defined(CONFIG_SOC_ZUMA)
#include <dt-bindings/soc/google/zuma-devfreq.h>
#endif
#endif
#if defined(CONFIG_SOC_GS101)
#include <dt-bindings/clock/gs101.h>
#elif defined(CONFIG_SOC_GS201)
#include <dt-bindings/clock/gs201.h>
#elif defined(CONFIG_SOC_ZUMA)
#include <dt-bindings/clock/zuma.h>
#endif

#if defined(CONFIG_SOC_EXYNOS9610)
#include <dt-bindings/clock/exynos9610.h>
#elif defined(CONFIG_SOC_EXYNOS9820)
#include <dt-bindings/clock/exynos9820.h>
#endif

#include <linux/kernel.h>
#include <trace/dpu_trace.h>
#include "exynos_drm_decon.h"
#include "exynos_drm_format.h"
#include "exynos_drm_writeback.h"

#define DISP_FACTOR_PCT		100UL
#define MULTI_FACTOR		(1UL << 10)
#define ROT_READ_BYTE		(32) /* unit : BYTE(= pixel, based on NV12) */

#define ACLK_100MHZ_PERIOD	10000UL
#define FRAME_TIME_NSEC		1000000000UL	/* 1sec */

#define DPU_DEBUG_BTS(fmt, args...)	pr_debug("[BTS] "fmt,  ##args)
#define DPU_INFO_BTS(fmt, args...)	pr_info("[BTS] "fmt,  ##args)
#define DPU_ERR_BTS(fmt, args...)	pr_err("[BTS] "fmt, ##args)

/*
 * 1. function clock
 *    panel_clk = panel_w * panel_h * fps * margin / ppc
 *    vertical scale-down case (src_h > dst_h)
 *     clk[i] = (line_a * ratio_a + line_b * (1 - ratio_a)) *
 *                  panel_h * fps * margin / ppc
 *        - line_a = max((ratio_v - 2) * src_w + max(src_w, dst_w), panel_w + diff_w)
 *        - line_b = max((ratio_v - 1) * src_w + max(src_w, dst_w), panel_w + diff_w)
 *        - ratio_v = ceiling(src_h / dst_h)
 *        - ratio_a = ratio_v - (src_h / dst_h)
 *        - diff_w = (src_w <= dst_w) ? 0 : src_w - dst_w
 *    non-vertical scale-down case
 *     clk[i] = ((panel_w + diff_w) * ratio_v + panel_w * (1 - ratio_v)) *
 *                  panel_h * fps * margin / ppc
 *        - ratio_v = (src_h >= dst_h) ? 1 : src_h / dst_h
 *        - diff_w = (src_w <= dst_w) ? 0 : src_w - dst_w
 *    margin = 1.1 + HW bubble cycles
 *    aclk1 = max(panel_clk, clk[i])
 * 2. AXI throughput clock
 *    1) fps based
 *       clk_bw[i] = src_w * src_h * fps * (bpp / 8) * (panel_h / dst_h) * 1.1
 *                       / (bus_width * bus_util_pct)
 *    2) rotation throughput for initial latency
 *       clk_r[i] = src_h * 32 * (bpp / 8) / (bus_width * rot_util_pct) / (v_blank)
 *       # v_blank : command - TE_hi_pulse
 *                   video - (vbp) @initial-frame, (vbp+vfp) @inter-frame
 *    if (clk_bw[i] < clk_r[i])
 *       clk_bw[i] = clk_r[i]
 *    aclk2 = max(clk for sum(same axi overlap bw[i]))
 *
 * => aclk_dpu = max(aclk1, aclk2)
 */

/* unit : usec x 1000 -> 5592 (5.592us) for WQHD+ case */
static inline u32 dpu_bts_get_one_line_time(u32 lcd_height, u32 vbp, u32 vfp,
		u32 vsa, u32 fps)
{
	u32 tot_v;
	int tmp;

	tot_v = lcd_height + vfp + vsa + vbp;
	tmp = DIV_ROUND_UP(FRAME_TIME_NSEC, fps);

	return (tmp / tot_v);
}

/* framebuffer compressor(AFBC, SBWC) line delay is usually 4 */
static inline u32 dpu_bts_comp_latency(u32 src_w, u32 ppc, u32 line_delay)
{
	return mult_frac(src_w, line_delay, ppc);
}

/* scaler line delay is usually 3
 * scaling order : horizontal -> vertical scale
 * -> need to reflect scale-ratio
 */
static inline u32 dpu_bts_scale_latency(u32 src_w, u32 dst_w, u32 ppc,
				u32 line_delay)
{
	if (src_w > dst_w)
		return mult_frac(src_w * line_delay, src_w, dst_w * ppc);
	else
		return DIV_ROUND_CLOSEST(src_w * line_delay, ppc);
}

/* rotator ppc is usually 4 or 8
 * 1-read : 32BYTE (pixel)
 */
static inline u32 dpu_bts_rotate_latency(u32 src_w, u32 r_ppc)
{
	return (src_w * (ROT_READ_BYTE / r_ppc));
}

/*
 * [DSC]
 * Line memory is necessary like following.
 *  1EA(1ppc) : 2-line for 2-slice, 1-line for 1-slice
 *  2EA(2ppc) : 3.5-line for 4-slice (DSCC 0.5-line + DSC 3-line)
 *        2.5-line for 2-slice (DSCC 0.5-line + DSC 2-line)
 *
 * [DECON] none
 * When 1H is filled at OUT_FIFO, it immediately transfers to DSIM.
 */
static inline u32 dpu_bts_dsc_latency(u32 slice_num, u32 dsc_cnt,
		u32 dst_w, u32 ppc)
{
	u32 lat_dsc = dst_w;

	switch (slice_num) {
	case 1:
		/* DSC: 1EA */
		lat_dsc = dst_w * 1;
		break;
	case 2:
		if (dsc_cnt == 1)
			lat_dsc = dst_w * 2;
		else
			lat_dsc = (dst_w * 25) / (10 * ppc);
		break;
	case 4:
		/* DSC: 2EA */
		lat_dsc = (dst_w * 35) / (10 * ppc);
		break;
	default:
		break;
	}

	return lat_dsc;
}

/*
 * unit : nsec x 1000
 * reference aclk : 100MHz (-> 10ns x 1000)
 * # cycles = usec * aclk_mhz
 */
static inline u32 dpu_bts_convert_aclk_to_ns(u32 aclk_mhz)
{
	return ((ACLK_100MHZ_PERIOD * 100) / aclk_mhz);
}

/*
 * return : kHz value based on 1-pixel processing pipe-line
 */
static u64 dpu_bts_get_resol_clock(u32 xres, u32 yres, u32 fps)
{
	u64 margin;
	u64 resol_khz;

	/*
	 * aclk_khz = vclk_1pix * ( 1.1 + (48+20)/WIDTH ) : x1000
	 * @ (1.1)   : BUS Latency Considerable Margin (10%)
	 * @ (48+20) : HW bubble cycles
	 *      - 48 : 12 cycles per slice, total 4 slice
	 *      - 20 : hblank cycles for other HW module
	 */
	margin = 1100 + ((48000 + 20000) / xres);
	/* convert to kHz unit */
	resol_khz = (xres * yres * fps * margin / 1000) / 1000;

	return resol_khz;
}

static u32 dpu_bts_get_vblank_time_ns(struct decon_device *decon)
{
	u32 line_t_ns, v_blank_t_ns;

	line_t_ns = dpu_bts_get_one_line_time(decon->config.image_height,
		decon->bts.vbp, decon->bts.vfp, decon->bts.vsa, decon->bts.fps);
	if (decon->config.mode.op_mode == DECON_VIDEO_MODE)
		v_blank_t_ns = (decon->bts.vbp + decon->bts.vfp) * line_t_ns;
	else
		v_blank_t_ns = decon->bts.vblank_usec * 1000U;

	DPU_DEBUG_BTS("  -line_t_ns(%u) v_blank_t_ns(%u)\n",
			line_t_ns, v_blank_t_ns);

	return v_blank_t_ns;
}

static u32 dpu_bts_find_nearest_high_freq(struct decon_device *decon, u32 aclk_base)
{
	int i;

	if (aclk_base > decon->bts.dfs_lv_khz[0]) {
		DPU_DEBUG_BTS("  aclk_base is greater than L0 frequency!");
		i = 0;
	} else {
		/* search from low frequency level */
		for (i = (decon->bts.dfs_lv_cnt - 1); i >= 0; i--) {
			if (aclk_base <= decon->bts.dfs_lv_khz[i])
				break;
		}
	}
	DPU_DEBUG_BTS("  Nearest DFS: %u KHz @L%d\n", decon->bts.dfs_lv_khz[i], i);

	return i;
}

/*
 * [caution] src_w/h is rotated size info
 * - src_w : src_h @original input image
 * - src_h : src_w @original input image
 */
static u64 dpu_bts_calc_rotate_aclk(struct decon_device *decon, u32 aclk_base,
		u32 ppc, u32 src_w, u32 dst_w,
		bool is_comp, bool is_downscale, bool is_dsc)
{
	u32 dfs_idx = 0;
	u32 dpu_cycle, basic_cycle, dsi_cycle, module_cycle = 0;
	u32 comp_cycle = 0, rot_cycle = 0, scale_cycle = 0, dsc_cycle = 0;
	u32 rot_init_bw = 0; /* KB/s */
	u64 rot_clk, rot_need_clk;
	u32 aclk_x_1k_ns, dpu_lat_t_ns, max_lat_t_ns, tx_allow_t_ns;
	u32 bus_perf;
	u32 temp_clk;
	bool retry_flag = false;

	DPU_DEBUG_BTS("[ROT+] BEFORE latency check: %u KHz\n", aclk_base);

	dfs_idx = dpu_bts_find_nearest_high_freq(decon, aclk_base);
	rot_clk = decon->bts.dfs_lv_khz[dfs_idx];

	/* post DECON OUTFIFO based on 1H transfer */
	dsi_cycle = decon->config.image_width;

	/* get additional pipeline latency */
	if (is_comp) {
		comp_cycle = dpu_bts_comp_latency(src_w, ppc,
			decon->bts.delay_comp);
		DPU_DEBUG_BTS("  COMP: lat_cycle(%u)\n", comp_cycle);
		module_cycle += comp_cycle;
	} else {
		rot_cycle = dpu_bts_rotate_latency(src_w,
			decon->bts.ppc_rotator);
		DPU_DEBUG_BTS("  ROT: lat_cycle(%u)\n", rot_cycle);
		module_cycle += rot_cycle;
	}
	if (is_downscale) {
		scale_cycle = dpu_bts_scale_latency(src_w, dst_w,
			decon->bts.ppc_scaler, decon->bts.delay_scaler);
		DPU_DEBUG_BTS("  SCALE: lat_cycle(%u)\n", scale_cycle);
		module_cycle += scale_cycle;
	}
	if (is_dsc) {
		dsc_cycle = dpu_bts_dsc_latency(decon->config.dsc.slice_count,
			decon->config.dsc.dsc_count, dst_w, ppc);
		DPU_DEBUG_BTS("  DSC: lat_cycle(%u)\n", dsc_cycle);
		module_cycle += dsc_cycle;
		dsi_cycle = (dsi_cycle + 2) / 3;
	}

	/*
	 * basic cycle(+ bubble: 10%) + additional cycle based on function
	 * cycle count increases when ACLK goes up due to other conditions
	 * At latency monitor experiment using unit test,
	 *  cycles at 400Mhz were increased by about 800 compared to 200Mhz.
	 * Here, (aclk_mhz * 2) cycles are reflected referring to the result
	 *  because the exact value is unknown.
	 */
	basic_cycle = (decon->config.image_width * 11 / 10 + dsi_cycle) / ppc;

retry_hi_freq:
	dpu_cycle = (basic_cycle + module_cycle) + rot_clk * 2 / 1000U;
	aclk_x_1k_ns = dpu_bts_convert_aclk_to_ns(rot_clk / 1000U);
	dpu_lat_t_ns = mult_frac(aclk_x_1k_ns, dpu_cycle, 1000);
	max_lat_t_ns = dpu_bts_get_vblank_time_ns(decon);
	if (max_lat_t_ns > dpu_lat_t_ns) {
		tx_allow_t_ns = max_lat_t_ns - dpu_lat_t_ns;
	} else {
		/* abnormal case : apply bus_util_pct of v_blank */
		tx_allow_t_ns = (max_lat_t_ns * decon->bts.bus_util_pct) / 100;
		DPU_DEBUG_BTS("  WARN: latency calc is abnormal!(-> %u%%)\n",
				decon->bts.bus_util_pct);
	}

	bus_perf = decon->bts.bus_width * decon->bts.rot_util_pct;
	/* apply as worst(P010: 3) case to simplify */
	rot_init_bw = mult_frac(NSEC_PER_SEC, src_w * ROT_READ_BYTE * 3, tx_allow_t_ns) / 1000;
	rot_need_clk = rot_init_bw * 100 / bus_perf;

	if (rot_need_clk > rot_clk) {
		/* not max level */
		if (dfs_idx) {
			/* check if calc_clk is greater than 1-step */
			dfs_idx--;
			temp_clk = decon->bts.dfs_lv_khz[dfs_idx];
			if ((rot_need_clk > temp_clk) && (!retry_flag)) {
				DPU_DEBUG_BTS("  -allow_ns(%u) dpu_ns(%u)\n",
					tx_allow_t_ns, dpu_lat_t_ns);
				rot_clk = temp_clk;
				retry_flag = true;
				goto retry_hi_freq;
			}
		}
		rot_clk = rot_need_clk;
	}

	DPU_DEBUG_BTS("  -dpu_cycle(%u) aclk_x_1k_ns(%u) dpu_lat_t_ns(%u)\n",
			dpu_cycle, aclk_x_1k_ns, dpu_lat_t_ns);
	DPU_DEBUG_BTS("  -tx_allow_t_ns(%u) rot_init_bw(%u) rot_need_clk(%llu)\n",
			tx_allow_t_ns, rot_init_bw, rot_need_clk);
	DPU_DEBUG_BTS("[ROT-] AFTER latency check: %llu KHz\n", rot_clk);

	return rot_clk;
}

static u64 dpu_bts_calc_aclk_disp(struct decon_device *decon,
				  const struct dpu_bts_win_config *config, u64 resol_clk,
				  u32 max_clk)
{
	u64 aclk_disp, aclk_base, aclk_panel_khz, aclk_disp_khz;
	u32 ppc;
	u32 src_w, src_h;
	u32 diff_w, ratio_v;
	u32 is_downscale = false;
	u32 is_dsc = false;
	u64 margin;

	if (config->is_rot) {
		src_w = config->src_h;
		src_h = config->src_w;
	} else {
		src_w = config->src_w;
		src_h = config->src_h;
	}

	if (src_w > config->dst_w || src_h > config->dst_h)
		is_downscale = true;

	/* when calculating aclk for panel, if DSC is enabled, consider DSC encoder
	 * count as its ppc.
	 */
	if (decon->config.dsc.enabled) {
		ppc = min(decon->bts.ppc, decon->config.dsc.dsc_count);
		is_dsc = true;
	} else {
		ppc = decon->bts.ppc;
	}
	aclk_panel_khz = resol_clk / ppc;

	margin = 1100 + ((48000 + 20000) / decon->config.image_width);
	diff_w = (src_w <= config->dst_w) ? 0 : src_w - config->dst_w;

	if (src_h > config->dst_h) {
		u32 ratio_a, line_a, line_b;

		ratio_v = DIV_ROUND_UP(src_h, config->dst_h);
		ratio_a = (ratio_v * 1000) - mult_frac(src_h, 1000, config->dst_h);
		line_a = max((ratio_v - 2) * src_w + max(src_w, config->dst_w),
				decon->config.image_width + diff_w);
		line_b = max((ratio_v - 1) * src_w + max(src_w, config->dst_w),
				decon->config.image_width + diff_w);
		aclk_disp = (u64)(line_a * ratio_a + line_b * (1000 - ratio_a));
	} else {
		ratio_v = (src_h >= config->dst_h) ? 1000 : mult_frac(src_h, 1000, config->dst_h);
		aclk_disp = (u64)((decon->config.image_width + diff_w) *
			ratio_v + decon->config.image_width * (1000 - ratio_v));
	}
	aclk_disp = mult_frac(aclk_disp, decon->config.image_height * decon->bts.fps, 1000);
	aclk_disp_khz = (aclk_disp * margin / 1000) / 1000;
	if (decon->bts.afbc_clk_ppc_margin && config->is_comp)
		aclk_disp_khz = mult_frac(aclk_disp_khz, decon->bts.afbc_clk_ppc_margin, 100);
	aclk_disp_khz /= decon->bts.ppc;

	if (aclk_disp_khz < aclk_panel_khz)
		aclk_disp_khz = aclk_panel_khz;

	if (!config->is_rot)
		return aclk_disp_khz;

	/* rotation case: check if latency conditions are met */
	if (aclk_disp_khz > max_clk)
		aclk_base = aclk_disp_khz;
	else
		aclk_base = max_clk;

	aclk_disp_khz = dpu_bts_calc_rotate_aclk(decon, (u32)aclk_base, ppc,
			src_w, config->dst_w, config->is_comp, is_downscale, is_dsc);

	return aclk_disp_khz;
}

static void dpu_bts_sum_all_decon_bw(u32 id, u32 *max_overlap_bw, u32 *max_disp_ch_bw)
{
	int i, j;
	struct decon_device *decon;
	u32 overlap_bw = 0, disp_ch_bw[MAX_AXI_PORT] = { 0 };

	if (id < 0 || id >= MAX_DECON_CNT) {
		DPU_ERR_BTS("[%s] undefined decon id(%u)!\n", __func__, id);
		*max_overlap_bw = 0;
		*max_disp_ch_bw = 0;
		return;
	}

	for (i = 0; i < MAX_DECON_CNT; i++) {
		decon = get_decon_drvdata(i);
		if (decon == NULL)
			continue;

		if (decon->bts.rt_avg_bw) {
			overlap_bw += decon->bts.rt_avg_bw;
			if (decon->id != id)
				DPU_DEBUG_BTS("    add DECON%d Overlap BW = %u\n",
					i, decon->bts.rt_avg_bw);
		}

		for (j = 0; j < MAX_AXI_PORT; j++) {
			if (decon->bts.ch_bw[j]) {
				disp_ch_bw[j] += decon->bts.ch_bw[j];
				if (decon->id != id)
					DPU_DEBUG_BTS("    add DECON%d AXI_DPU%d = %u\n",
						i, j, decon->bts.ch_bw[j]);
			}
		}
	}

	*max_overlap_bw = overlap_bw;
	*max_disp_ch_bw = disp_ch_bw[0];
	for (i = 1; i < MAX_AXI_PORT; i++)
		*max_disp_ch_bw = max(*max_disp_ch_bw, disp_ch_bw[i]);

	DPU_DEBUG_BTS("  Max Overlap BW = %u, Max AXI_DPU = %u\n",
		*max_overlap_bw, *max_disp_ch_bw);
}

static u32 dpu_bts_calc_disp_with_full_size(struct decon_device *decon)
{
	struct dpu_bts_win_config config;

	memset(&config, 0, sizeof(struct dpu_bts_win_config));
	config.src_w = config.dst_w = decon->config.image_width;
	config.src_h = config.dst_h = decon->config.image_height;
	config.format = DRM_FORMAT_ARGB8888;

	decon->bts.resol_clk = dpu_bts_get_resol_clock(
			decon->config.image_width,
			decon->config.image_height, decon->bts.fps);

	return dpu_bts_calc_aclk_disp(decon, &config, decon->bts.resol_clk,
		decon->bts.resol_clk);
}

static bool is_win_half_covered(const struct dpu_bts_win_config *config0,
				const struct dpu_bts_win_config *config1)
{
	u32 start_y0;
	u32 start_y1, end_y1;

	if (unlikely(!config0) || unlikely(!config1)) {
		pr_err("win config is invalid\n");
		return false;
	}

	if (config0 == config1)
		return true;

	start_y0 = config0->dst_y;
	start_y1 = config1->dst_y;
	end_y1 = config1->dst_y + config1->dst_h;

	if (start_y0 >= start_y1 && start_y0 < end_y1)
		return true;

	return false;
}

static u32 dpu_bts_get_rt_bw(struct decon_device *decon,
				const struct dpu_bts_win_config *win_config)
{
	const u32 plane_id = DPPCH2PLANE(win_config->dpp_id);

	return decon->bts.rt_bw[plane_id].val;
}

static void dpu_bts_update_overlap_bw(struct decon_device *decon,
				       const struct dpu_bts_win_config *win_config,
				       const struct dpu_bts_win_config *rcd_config)
{
	int i, j;
	u32 win_max_overlap_bw = 0, rcd_max_overlap_bw = 0;

	/* overlap rt bandwidth requirement */
	/* TODO: take write rt bandwidth into account */
	for (i = 0; i < decon->win_cnt; i++) {
		u32 overlap_bw;

		if (win_config[i].state != DPU_WIN_STATE_BUFFER)
			continue;

		overlap_bw = 0;
		for (j = 0; j < decon->win_cnt; j++) {
			if (win_config[j].state != DPU_WIN_STATE_BUFFER)
				continue;

			if (is_win_half_covered(&win_config[i], &win_config[j]))
				overlap_bw += dpu_bts_get_rt_bw(decon, &win_config[j]);
		}

		if (rcd_config->state == DPU_WIN_STATE_BUFFER) {
			if (is_win_half_covered(&win_config[i], rcd_config))
				overlap_bw += dpu_bts_get_rt_bw(decon, rcd_config);

			if (is_win_half_covered(rcd_config, &win_config[i]))
				rcd_max_overlap_bw += dpu_bts_get_rt_bw(decon, &win_config[i]);
		}

		DPU_DEBUG_BTS("  Overlap BW%d = %u\n", i, overlap_bw);
		win_max_overlap_bw = max(win_max_overlap_bw, overlap_bw);
	}

	if (rcd_config->state == DPU_WIN_STATE_BUFFER)
		rcd_max_overlap_bw += dpu_bts_get_rt_bw(decon, rcd_config);

	decon->bts.rt_avg_bw = max(win_max_overlap_bw, rcd_max_overlap_bw);
}

static void dpu_bts_update_disp_ch_bw(struct decon_device *decon,
				       const struct dpu_bts_win_config *win_config,
				       const struct dpu_bts_win_config *rcd_config)
{
	int i, j;
	u32 disp_ch_bw[MAX_AXI_PORT];
	u32 rcd_overlap_ch_bw = 0;

	/* DPU AXI bandwidth requirement */
	/* TODO: take write rt bandwidth into account */
	memset(disp_ch_bw, 0, sizeof(disp_ch_bw));
	for (i = 0; i < decon->win_cnt; i++) {
		u32 dpp_id = win_config[i].dpp_id;
		u32 ch_num = decon->bts.rt_bw[DPPCH2PLANE(dpp_id)].ch_num;
		u32 overlap_ch_bw;

		if (win_config[i].state != DPU_WIN_STATE_BUFFER)
			continue;

		if (ch_num >= MAX_AXI_PORT) {
			pr_err("invalid DPU AXI channel number %u\n", ch_num);
			continue;
		}

		overlap_ch_bw = 0;
		for (j = 0; j < decon->win_cnt; j++) {
			int dpp_id = win_config[j].dpp_id;

			if (win_config[j].state != DPU_WIN_STATE_BUFFER)
				continue;

			if (ch_num == decon->bts.rt_bw[DPPCH2PLANE(dpp_id)].ch_num &&
				(is_win_half_covered(&win_config[i], &win_config[j])))
				overlap_ch_bw += dpu_bts_get_rt_bw(decon, &win_config[j]);
		}

		if (rcd_config->state == DPU_WIN_STATE_BUFFER) {
			u32 rcd_dpp_id = rcd_config->dpp_id;

			if (ch_num == decon->bts.rt_bw[DPPCH2PLANE(rcd_dpp_id)].ch_num) {
				if (is_win_half_covered(&win_config[i], rcd_config))
					overlap_ch_bw += dpu_bts_get_rt_bw(decon, rcd_config);

				if (is_win_half_covered(rcd_config, &win_config[i]))
					rcd_overlap_ch_bw +=
						dpu_bts_get_rt_bw(decon, &win_config[i]);
			}
		}
		disp_ch_bw[ch_num] = max(disp_ch_bw[ch_num], overlap_ch_bw);
	}

	if (rcd_config->state == DPU_WIN_STATE_BUFFER) {
		u32 rcd_dpp_id = rcd_config->dpp_id;
		u32 rcd_ch_num = decon->bts.rt_bw[DPPCH2PLANE(rcd_dpp_id)].ch_num;

		if (rcd_ch_num >= MAX_AXI_PORT) {
			pr_err("invalid RCD AXI channel number %u\n", rcd_ch_num);
		} else {
			rcd_overlap_ch_bw += dpu_bts_get_rt_bw(decon, rcd_config);
			disp_ch_bw[rcd_ch_num] = max(disp_ch_bw[rcd_ch_num], rcd_overlap_ch_bw);
		}
	}

	for (i = 0; i < MAX_AXI_PORT; ++i) {
		decon->bts.ch_bw[i] = disp_ch_bw[i];
		if (decon->bts.ch_bw[i])
			DPU_DEBUG_BTS("  AXI_DPU%d = %u\n", i, decon->bts.ch_bw[i]);
	}
}

static void dpu_bts_find_max_disp_freq(struct decon_device *decon)
{
	int i;
	u32 max_overlap_bw;
	u32 max_disp_ch_bw;
	u32 disp_op_freq = 0;
	const struct dpu_bts_win_config *win_config = decon->bts.win_config;
	const struct dpu_bts_win_config *rcd_config = &decon->bts.rcd_win_config.win;

	dpu_bts_update_overlap_bw(decon, win_config, rcd_config);
	dpu_bts_update_disp_ch_bw(decon, win_config, rcd_config);
	dpu_bts_sum_all_decon_bw(decon->id, &max_overlap_bw, &max_disp_ch_bw);

	decon->bts.max_disp_freq = max_disp_ch_bw * 100 /
			(decon->bts.bus_width * decon->bts.bus_util_pct);

	/* TODO: the final INT should be max(max_peak_bw, total_peak_bw / NUM_DRAM_CH). It nees
	 * some changes in bts driver to allow client request peak_bw. Before we lock down the
	 * design, DPU requests max(max_ch_bw, max_overlap_bw / NUM_INTERCONNECT_CH) as peak.
	 * After we take write bw into account, we don't need to check write bw here.
	 */
	decon->bts.peak = max3(max_disp_ch_bw, max_overlap_bw / NUM_INTERCONNECT_CH,
				decon->bts.write_bw);

	for (i = 0; i < decon->win_cnt; ++i) {
		u32 freq;

		if ((win_config[i].state != DPU_WIN_STATE_BUFFER) &&
				(win_config[i].state != DPU_WIN_STATE_COLOR))
			continue;

		freq = dpu_bts_calc_aclk_disp(decon, &win_config[i],
				(u64)decon->bts.resol_clk, decon->bts.max_disp_freq);
		disp_op_freq = max(disp_op_freq, freq);
	}

	/*
	 * At least one window is used for colormap if there is a request of
	 * disabling all windows. So, disp frequency for a window of LCD full
	 * size is necessary.
	 */
	if (disp_op_freq == 0)
		disp_op_freq = dpu_bts_calc_disp_with_full_size(decon);

	DPU_DEBUG_BTS("  DISP bus freq(%u), operating freq(%u)\n",
			decon->bts.max_disp_freq, disp_op_freq);

	decon->bts.max_disp_freq = max(decon->bts.max_disp_freq, disp_op_freq);

	DPU_DEBUG_BTS("  MAX DISP CH FREQ = %u\n", decon->bts.max_disp_freq);
}

static void
dpu_bts_calc_dpp_bw(struct bts_dpp_info *dpp, u32 fps, u32 lcd_h, u32 vblank_us, u32 dpp_id,
		const struct dpu_bts *bts)
{
	u32 avg_bw, rt_bw, rot_bw = 0;
	u32 src_w = dpp->src_w;
	u32 src_h = dpp->src_h;
	u32 dst_h = dpp->dst.y2 - dpp->dst.y1;
	u32 bpp = dpp->bpp;

	/* Bandwidth requirement for layer
	 * - AVG BW (KB) : sw * sh * fps * (bpp / 8) / 1000
	 * - RT BW (KB) : AVG_BW * panel_h / dh * 1.1
	 */
	avg_bw = src_w * src_h * bpp / 8 * fps / 1000;
	rt_bw = mult_frac(avg_bw, lcd_h * 11, dst_h * 10);

	if (dpp->rotation) {
		/* ROT BW(KB) : sh * 32B * (bpp / 8) / v_blank */
		rot_bw = mult_frac(src_h * ROT_READ_BYTE * bpp / 8,
				USEC_PER_SEC, vblank_us) / 1000;
	}

	DPU_DEBUG_BTS("  DPP%d bandwidth: avg %u, rt %u, rot %u\n", dpp_id, avg_bw, rt_bw, rot_bw);

	rt_bw = max(rt_bw, rot_bw);
	if (dpp->is_afbc) {
		u32 afbc_util_pct, afbc_rt_util_pct;

		if (dpp->is_yuv) {
			afbc_util_pct = bts->afbc_yuv_util_pct;
			afbc_rt_util_pct = bts->afbc_yuv_rt_util_pct;
		} else {
			afbc_util_pct = bts->afbc_rgb_util_pct;
			afbc_rt_util_pct = bts->afbc_rgb_rt_util_pct;
		}

		avg_bw = mult_frac(avg_bw, afbc_util_pct, 100);
		rt_bw = mult_frac(rt_bw, afbc_rt_util_pct, 100);
	}

	dpp->bw = avg_bw;
	dpp->rt_bw = rt_bw;
	DPU_DEBUG_BTS("           final: avg %u, rt %u\n", dpp->bw, dpp->rt_bw);
}

static void dpu_bts_convert_config_to_info(struct bts_dpp_info *dpp,
				const struct dpu_bts_win_config *config)
{
	const struct dpu_fmt *fmt_info;

	fmt_info = dpu_find_fmt_info(config->format);
	if (!fmt_info) {
		DPU_ERR_BTS("DPP%d: unable to find fmt_info\n", config->dpp_id);
		return;
	}
	dpp->bpp = fmt_info->bpp + fmt_info->padding;
	dpp->src_w = config->src_w;
	dpp->src_h = config->src_h;
	dpp->dst.x1 = config->dst_x;
	dpp->dst.x2 = config->dst_x + config->dst_w;
	dpp->dst.y1 = config->dst_y;
	dpp->dst.y2 = config->dst_y + config->dst_h;
	dpp->rotation = config->is_rot;
	dpp->is_afbc = config->is_comp;
	dpp->is_yuv = IS_YUV(fmt_info);

	DPU_DEBUG_BTS("  DPP%d : bpp(%u) src w(%u) h(%u) rot(%d) afbc(%d) yuv(%d)\n",
			config->dpp_id, dpp->bpp, dpp->src_w,
			dpp->src_h, dpp->rotation, dpp->is_afbc, dpp->is_yuv);
	DPU_DEBUG_BTS("        dst x(%u) right(%u) y(%u) bottom(%u)\n",
			dpp->dst.x1, dpp->dst.x2, dpp->dst.y1, dpp->dst.y2);
}

static inline u32 dpu_bts_find_max_dpp_read_rt_bw(void)
{
	u32 i, max_dpp_read_rt_bw = 0;
	struct decon_device *decon;

	for (i = 0; i < MAX_DECON_CNT; i++) {
		decon = get_decon_drvdata(i);
		if (decon == NULL)
			continue;

		if (max_dpp_read_rt_bw < decon->bts.max_dpp_read_rt_bw)
			max_dpp_read_rt_bw = decon->bts.max_dpp_read_rt_bw;
	}

	return max_dpp_read_rt_bw;
}

static void dpu_bts_calc_urgent_latency(struct decon_device *decon)
{
	u32 i, max_dpp_read_rt_bw;

	if (!decon->bts_urgent_rd_lat.enabled)
		return;

	max_dpp_read_rt_bw = dpu_bts_find_max_dpp_read_rt_bw();
	for (i = 0; i < decon->bts_urgent_rd_lat.bw_lat_map_cnt - 1; i++) {
		if (max_dpp_read_rt_bw <=
				decon->bts_urgent_rd_lat.bw_lat_tbl[i].bw_kbps)
			break;
	}

	decon->bts.urgent_rd_lat = decon->bts_urgent_rd_lat.bw_lat_tbl[i].latency_ns;
}

static void dpu_bts_calc_bw(struct decon_device *decon)
{
	struct drm_crtc *crtc = &decon->crtc->base;
	struct drm_crtc_state *crtc_state = crtc->state;
	struct dpu_bts_win_config *config;
	struct bts_decon_info bts_info;
	int idx, i, wb_idx = -1, rcd_idx = -1;
	u32 read_bw = 0, write_bw, video_num = 0, max_dpp_read_rt_bw = 0;
	u64 resol_clock;
	u32 vblank_us;

	if (!decon->bts.enabled)
		return;

	DPU_DEBUG_BTS("%s + : DECON%u\n", __func__, decon->id);

	memset(&bts_info, 0, sizeof(struct bts_decon_info));

	resol_clock = dpu_bts_get_resol_clock(decon->config.image_width,
				decon->config.image_height, decon->bts.fps);
	decon->bts.resol_clk = (u32)resol_clock;
	DPU_DEBUG_BTS("[Run: D%u] resol clock = %u Khz @%u fps\n",
		decon->id, decon->bts.resol_clk, decon->bts.fps);

	bts_info.vclk = decon->bts.resol_clk;
	bts_info.lcd_w = decon->config.image_width;
	bts_info.lcd_h = decon->config.image_height;
	vblank_us = dpu_bts_get_vblank_time_ns(decon) / 1000U;
	/* reflect bus_util_pct for dpu processing latency when rotation */
	vblank_us = (vblank_us * decon->bts.rot_util_pct) / 100;

	/* read bw calculation */
	config = decon->bts.win_config;
	for (i = 0; i < decon->win_cnt; ++i) {
		if (config[i].state != DPU_WIN_STATE_BUFFER)
			continue;

		idx = DPPCH2PLANE(config[i].dpp_id);
		dpu_bts_convert_config_to_info(&bts_info.rdma[idx], &config[i]);
		dpu_bts_calc_dpp_bw(&bts_info.rdma[idx], decon->bts.fps,
				decon->config.image_height, vblank_us,
				config[i].dpp_id, &decon->bts);
		read_bw += bts_info.rdma[idx].bw;
		if (max_dpp_read_rt_bw < bts_info.rdma[idx].rt_bw)
			max_dpp_read_rt_bw = bts_info.rdma[idx].rt_bw;
		if (bts_info.rdma[idx].is_yuv)
			video_num++;
	}

	/* write bw calculation */
	config = &decon->bts.wb_config;
	if (config->state == DPU_WIN_STATE_BUFFER) {
		wb_idx = DPPCH2PLANE(config->dpp_id);
		dpu_bts_convert_config_to_info(&bts_info.odma, config);
		dpu_bts_calc_dpp_bw(&bts_info.odma, decon->bts.fps, bts_info.lcd_h,
				vblank_us, config->dpp_id, &decon->bts);
		write_bw = bts_info.odma.bw;
	} else {
		wb_idx = -1;
		write_bw = 0;
	}

	/* rcd bw calculation */
	config = &decon->bts.rcd_win_config.win;
	if (config->state == DPU_WIN_STATE_BUFFER) {
		rcd_idx = DPPCH2PLANE(config->dpp_id);
		dpu_bts_convert_config_to_info(&bts_info.rcddma, config);
		dpu_bts_calc_dpp_bw(&bts_info.rcddma, decon->bts.fps, bts_info.lcd_h,
				vblank_us, config->dpp_id, &decon->bts);
		if (max_dpp_read_rt_bw < bts_info.rcddma.rt_bw)
			max_dpp_read_rt_bw = bts_info.rcddma.rt_bw;
		read_bw += bts_info.rcddma.bw;
	} else {
		rcd_idx = -1;
	}


	for (i = 0; i < MAX_DPP_CNT; i++) {
		if (i < MAX_WIN_PER_DECON)
			decon->bts.rt_bw[i].val = bts_info.rdma[i].rt_bw;
		else if (i == wb_idx)
			decon->bts.rt_bw[i].val = bts_info.odma.rt_bw;
		else if (i == rcd_idx)
			decon->bts.rt_bw[i].val = bts_info.rcddma.rt_bw;
		else
			decon->bts.rt_bw[i].val = 0;
	}

	decon->bts.read_bw = read_bw;
	decon->bts.write_bw = write_bw;
	decon->bts.total_bw = read_bw + write_bw;
	decon->bts.max_dpp_read_rt_bw = max_dpp_read_rt_bw;
	decon->bts.video_num = video_num;

	DPU_DEBUG_BTS("  DECON%u total bw = %u, read bw = %u, write bw = %u, max dpp rt_bw = %u\n",
			decon->id, decon->bts.total_bw, decon->bts.read_bw,
			decon->bts.write_bw, decon->bts.max_dpp_read_rt_bw);

	if (decon->bts.total_bw) {
		dpu_bts_find_max_disp_freq(decon);
	} else {
		/* no bw requirement */
		decon->bts.peak = 0;
		decon->bts.rt_avg_bw = 0;
		decon->bts.max_disp_freq = dpu_bts_calc_disp_with_full_size(decon);
	}

	dpu_bts_calc_urgent_latency(decon);
	DPU_EVENT_LOG(DPU_EVT_BTS_CALC_BW, decon->id, crtc_state);
	DPU_DEBUG_BTS("%s -\n", __func__);
}

static inline void dpu_bts_update_bw(struct decon_device *decon, struct bts_bw bw)
{
	int ret;

	DPU_ATRACE_BEGIN("dpu_bts_update_bw");
	ret = bts_update_bw(decon->bts.bw_idx, bw);
	if (ret < 0) {
		pr_warn("decon%u failed to update bw(%d) to bts(%d)\n", decon->id,
			decon->bts.bw_idx, ret);
		decon_dump_event_condition(decon, DPU_EVT_CONDITION_FAIL_UPDATE_BW);
	}
	DPU_ATRACE_INT("dpu_vote_peak_bw", bw.peak);
	DPU_ATRACE_INT("dpu_vote_avg_bw", bw.read + bw.write);
	DPU_ATRACE_INT("dpu_vote_rt_bw", bw.rt);
	DPU_ATRACE_END("dpu_bts_update_bw");
}

static inline void dpu_bts_update_disp(struct decon_device *decon, u32 disp_freq)
{
	DPU_ATRACE_BEGIN("dpu_bts_update_disp");
	exynos_pm_qos_update_request(&decon->bts.disp_qos, disp_freq);
	DPU_ATRACE_INT("dpu_vote_clock", disp_freq);
	DPU_ATRACE_END("dpu_bts_update_disp");
}

static inline void dpu_bts_update_scenario(struct decon_device *decon, struct bts_bw bw)
{
	bool need_vote = false;

	if (!decon->bts_scen.enabled)
		return;

	if (decon->config.image_width >= decon->bts_scen.min_panel_width &&
		decon->config.image_height >= decon->bts_scen.min_panel_height &&
		bw.rt >= decon->bts_scen.min_rt_bw && bw.rt <= decon->bts_scen.max_rt_bw &&
		bw.peak >= decon->bts_scen.min_peak_bw && bw.peak <= decon->bts_scen.max_peak_bw &&
		decon->bts.fps >= decon->bts_scen.min_fps &&
		(!decon->bts_scen.skip_with_video ||
		 (decon->bts_scen.skip_with_video && !decon->bts.video_num))) {
		need_vote = true;
	}

	if (decon->bts_scen.voted != need_vote) {
		DPU_ATRACE_BEGIN(__func__);
		if (need_vote) {
			DPU_DEBUG_BTS("decon%u add bts scenario %u",
				decon->id, decon->bts_scen.idx);
			bts_add_scenario(decon->bts_scen.idx);
		} else {
			DPU_DEBUG_BTS("decon%u del bts scenario %u",
				decon->id, decon->bts_scen.idx);
			bts_del_scenario(decon->bts_scen.idx);
		}
		decon->bts_scen.voted = need_vote;
		DPU_ATRACE_END(__func__);
	}
}

static inline void dpu_bts_update_urgent_latency(struct decon_device *decon)
{
	unsigned int i;

	if (!decon->bts_urgent_rd_lat.enabled ||
		decon->bts.urgent_rd_lat == decon->bts.prev_urgent_rd_lat) {
		return;
	}

	DPU_ATRACE_BEGIN(__func__);
	for (i = 0; i < MAX_AXI_PORT; i++) {
		int index = decon->bts.urgent_rd_lat_index[i];

		bts_set_urgent_lat_read(index, decon->bts.urgent_rd_lat);
	}
	decon->bts.prev_urgent_rd_lat = decon->bts.urgent_rd_lat;
	DPU_ATRACE_END(__func__);
}

static void dpu_bts_update_resources(struct decon_device *decon, bool shadow_updated)
{
	struct bts_bw bw = { 0 };

	DPU_DEBUG_BTS("%s +\n", __func__);

	if (!decon->bts.enabled)
		return;

	/* update peak & R/W bandwidth per DPU port */
	bw.peak = decon->bts.peak;
	bw.rt = decon->bts.rt_avg_bw;
	bw.read = decon->bts.read_bw;
	bw.write = decon->bts.write_bw;
	DPU_DEBUG_BTS("  peak = %u, rt = %u, read = %u, write = %u\n",
		bw.peak, bw.rt, bw.read, bw.write);

	/* When concurrent writeback is enabled, writeback instant off may occur if the outfifo
	 * for writeback is full meanwhile the outfifo for LCD is not full yet.
	 * We can limit max_disp_freq to avoid it when concurrent writeback is enabled.
	 * Currently, the issue only occurs when all layers are solid color layers (read = 0).
	 */
	if ((decon->bts.max_dfs_lv_for_wb > 0) && (bw.read == 0) && (bw.write > 0)) {
		decon->bts.max_disp_freq =
			min(decon->bts.max_disp_freq, decon->bts.max_dfs_lv_for_wb);
	}

	dpu_bts_update_scenario(decon, bw);
	dpu_bts_update_urgent_latency(decon);
	if (shadow_updated) {
		/* after DECON h/w configs are updated to shadow SFR */
		if (decon->bts.total_bw < decon->bts.prev_total_bw ||
				decon->bts.peak < decon->bts.prev_peak ||
				decon->bts.rt_avg_bw < decon->bts.prev_rt_avg_bw)
			dpu_bts_update_bw(decon, bw);

		if (decon->bts.max_disp_freq < decon->bts.prev_max_disp_freq)
			dpu_bts_update_disp(decon, decon->bts.max_disp_freq);

		decon->bts.prev_total_bw = decon->bts.total_bw;
		decon->bts.prev_peak = decon->bts.peak;
		decon->bts.prev_rt_avg_bw = decon->bts.rt_avg_bw;
		decon->bts.prev_max_disp_freq = decon->bts.max_disp_freq;
	} else {
		if (decon->bts.total_bw > decon->bts.prev_total_bw ||
				decon->bts.peak > decon->bts.prev_peak ||
				decon->bts.rt_avg_bw > decon->bts.prev_rt_avg_bw)
			dpu_bts_update_bw(decon, bw);

		if (decon->bts.max_disp_freq > decon->bts.prev_max_disp_freq)
			dpu_bts_update_disp(decon, decon->bts.max_disp_freq);
	}

	DPU_EVENT_LOG(DPU_EVT_BTS_UPDATE_BW, decon->id, NULL);

	DPU_DEBUG_BTS("%s -\n", __func__);
}

static void dpu_bts_release_resources(struct decon_device *decon)
{
	struct bts_bw bw = { 0 };

	DPU_DEBUG_BTS("%s +\n", __func__);

	if (!decon->bts.enabled)
		return;

	dpu_bts_update_scenario(decon, bw);
	dpu_bts_update_bw(decon, bw);
	decon->bts.prev_peak = 0;
	decon->bts.prev_rt_avg_bw = 0;
	decon->bts.prev_total_bw = 0;
	dpu_bts_update_disp(decon, 0);
	decon->bts.prev_max_disp_freq = 0;

	// clear shared decon resources
	decon->bts.rt_avg_bw = 0;
	decon->bts.max_dpp_read_rt_bw = 0;
	memset(decon->bts.ch_bw, 0, sizeof(decon->bts.ch_bw));

	DPU_EVENT_LOG(DPU_EVT_BTS_RELEASE_BW, decon->id, NULL);
	DPU_DEBUG_BTS("%s -\n", __func__);
}

#define MAX_IDX_NAME_SIZE	16
static void dpu_bts_init(struct decon_device *decon)
{
	int i;
	char bts_idx_name[MAX_IDX_NAME_SIZE];
	const struct drm_encoder *encoder;

	DPU_DEBUG_BTS("%s +\n", __func__);

	decon->bts.enabled = false;

	if (!IS_ENABLED(CONFIG_EXYNOS_BTS) ||
			(!IS_ENABLED(CONFIG_EXYNOS_PM_QOS) &&
			 !IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE))) {
		DPU_ERR_BTS("decon%u bts feature is disabled\n", decon->id);
		pr_info("%s:%d\n", __func__, __LINE__);
		return;
	}

	memset(bts_idx_name, 0, MAX_IDX_NAME_SIZE);
	snprintf(bts_idx_name, MAX_IDX_NAME_SIZE, "DECON%u", decon->id);
	decon->bts.bw_idx = bts_get_bwindex(bts_idx_name);

	decon->bts.dvfs_max_disp_freq =
			(u32)cal_dfs_get_max_freq(ACPM_DVFS_DISP);

	decon->bts.rt_avg_bw = 0;
	for (i = 0; i < MAX_AXI_PORT; i++) {
		decon->bts.ch_bw[i] = 0;
		if (decon->bts_urgent_rd_lat.enabled) {
			snprintf(bts_idx_name, MAX_IDX_NAME_SIZE, "bts_dpu%u", i);
			decon->bts.urgent_rd_lat_index[i] =
				bts_get_urgent_lat_bts_index(bts_idx_name);
			if (decon->bts.urgent_rd_lat_index[i] < 0) {
				decon->bts_urgent_rd_lat.enabled = false;
				break;
			}
		}
	}

	DPU_DEBUG_BTS("BTS_BW_TYPE(%d)\n", decon->bts.bw_idx);
	exynos_pm_qos_add_request(&decon->bts.mif_qos,
					PM_QOS_BUS_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&decon->bts.int_qos,
					PM_QOS_DEVICE_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&decon->bts.disp_qos,
					PM_QOS_DISPLAY_THROUGHPUT, 0);

	for (i = 0; i < decon->dpp_cnt; ++i) { /* dma type order */
		decon->bts.rt_bw[i].ch_num = decon->dpp[i]->port;
		DPU_INFO_BTS("IDMA_TYPE(%d) CH(%d) Port(%u)\n", i,
				PLANE2DPPCH(i), decon->bts.rt_bw[i].ch_num);
	}

	drm_for_each_encoder(encoder, decon->drm_dev) {
		const struct writeback_device *wb;

		if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL) {
			wb = enc_to_wb_dev(encoder);
			decon->bts.rt_bw[DPPCH2PLANE(wb->id)].ch_num = wb->port;
			break;
		}
	}

	decon->bts_scen.enabled = false;
	if (decon->bts_scen.name && decon->bts_scen.name[0]) {
		decon->bts_scen.idx = bts_get_scenindex(decon->bts_scen.name);
		if (decon->bts_scen.idx) {
			decon->bts_scen.enabled = true;
			DPU_INFO_BTS("decon%u gets `%s bts scenario idx %u\n",
				decon->id, decon->bts_scen.name, decon->bts_scen.idx);
		} else {
			DPU_ERR_BTS("decon%u failed to get `%s` bts scenario",
				decon->id, decon->bts_scen.name);
		}
	}

	decon->bts.enabled = true;

	DPU_INFO_BTS("decon%u bts feature is enabled\n", decon->id);
}

static void dpu_bts_deinit(struct decon_device *decon)
{
	if (!decon->bts.enabled)
		return;

	DPU_DEBUG_BTS("%s +\n", __func__);
	exynos_pm_qos_remove_request(&decon->bts.disp_qos);
	exynos_pm_qos_remove_request(&decon->bts.int_qos);
	exynos_pm_qos_remove_request(&decon->bts.mif_qos);
	if (decon->bts_scen.enabled && decon->bts_scen.voted)
		bts_del_scenario(decon->bts_scen.idx);
	DPU_DEBUG_BTS("%s -\n", __func__);
}

struct dpu_bts_ops dpu_bts_control = {
	.init		= dpu_bts_init,
	.calc_bw	= dpu_bts_calc_bw,
	.update_bw	= dpu_bts_update_resources,
	.release_bw	= dpu_bts_release_resources,
	.deinit		= dpu_bts_deinit,
};
