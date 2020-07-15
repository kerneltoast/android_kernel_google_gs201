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

#include <exynos_drm_decon.h>
#include <exynos_drm_format.h>

#include <soc/google/bts.h>
#if defined(CONFIG_SOC_GS101)
#include <soc/google/exynos-devfreq.h>
#include <dt-bindings/soc/google/gs101-devfreq.h>
#endif
#if defined(CONFIG_CAL_IF)
#include <soc/samsung/cal-if.h>
#endif
#if defined(CONFIG_SOC_EXYNOS9610)
#include <dt-bindings/clock/exynos9610.h>
#elif defined(CONFIG_SOC_EXYNOS9820)
#include <dt-bindings/clock/exynos9820.h>
#endif

#define DISP_FACTOR		100UL
#define MULTI_FACTOR		(1UL << 10)
/* bus utilization 70% : same value with INT_UTIL */
#define BUS_UTIL		70

#define DPP_SCALE_NONE		0
#define DPP_SCALE_UP		1
#define DPP_SCALE_DOWN		2

#define ACLK_100MHZ_PERIOD	10000UL
#define ACLK_MHZ_INC_STEP	50U	/* 50Mhz */
#define FRAME_TIME_NSEC		1000000000UL	/* 1sec */
#define TOTAL_BUS_LATENCY	3000UL	/* 3us: BUS(1) + PWT(1) + Request(1) */

/* tuning parameters for rotation */
#define ROTATION_FACTOR_BPP	32UL
#define ROTATION_FACTOR_SCUP	1332UL	/* 1.3x */
#define ROTATION_FACTOR_SCDN	1434UL	/* 1.4x */
#define RESOL_QHDP_21_TO_9	(3440 * 1440UL) /* for MIF min-lock */

#define DPU_DEBUG_BTS(fmt, args...)	pr_debug("[BTS] "fmt,  ##args)
#define DPU_INFO_BTS(fmt, args...)	pr_info("[BTS] "fmt,  ##args)
#define DPU_ERR_BTS(fmt, args...)	pr_err("[BTS] "fmt, ##args)

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

/* lmc : line memory count (usually 4) */
static inline u32 dpu_bts_afbc_latency(u32 src_w, u32 ppc, u32 lmc)
{
	return mult_frac(src_w, lmc, ppc);
}

/*
 * line memory max size : 4096
 * lmc : line memory count (usually 4)
 */
static inline u32 dpu_bts_scale_latency(u32 is_s, u32 src_w, u32 dst_w,
		u32 ppc, u32 lmc)
{
	if (is_s == DPP_SCALE_DOWN)
		return mult_frac(src_w * lmc, src_w, dst_w * ppc);
	else
		return DIV_ROUND_CLOSEST(src_w * lmc, ppc);
}

/*
 * src_h : height of original input source image
 * cpl : cycles per line
 */
static inline u32 dpu_bts_rotate_latency(u32 src_h, u32 cpl)
{
	return (src_h * cpl * 12 / 10);
}

/*
 * [DSC]
 * Line memory is necessary like following.
 *  1EA : 2-line for 2-slice, 1-line for 1-slice
 *  2EA : 3.5-line for 4-slice (DSCC 0.5-line + DSC 3-line)
 *        2.5-line for 2-slice (DSCC 0.5-line + DSC 2-line)
 *
 * [DECON] none
 * When 1H is filled at OUT_FIFO, it immediately transfers to DSIM.
 */
static inline u32 dpu_bts_dsc_latency(u32 slice_num, u32 dsc_cnt, u32 dst_w)
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
			lat_dsc = (dst_w * 25) / 10;
		break;
	case 4:
		/* DSC: 2EA */
		lat_dsc = (dst_w * 35) / 10;
		break;
	default:
		break;
	}

	return lat_dsc;
}

/*
 * unit : cycles
 * rotate and afbc are incompatible
 */
static u32 dpu_bts_get_initial_latency(bool is_r, u32 is_s, bool is_c,
		struct exynos_dsc *dsc, u32 src_w, u32 dst_w,
		u32 ppc, u32 cpl, u32 lmc)
{
	u32 lat_cycle = 0;
	u32 tmp;

	if (dsc->enabled) {
		lat_cycle = dpu_bts_dsc_latency(dsc->slice_count,
				dsc->dsc_count, dst_w);
		DPU_DEBUG_BTS("\tDSC_lat_cycle(%d)\n", lat_cycle);
	}

	/* src_w : rotation reflected value */
	if (is_r) {
		tmp = dpu_bts_rotate_latency(src_w, cpl);
		DPU_DEBUG_BTS("\tR_lat_cycle(%d)\n", tmp);
		lat_cycle += tmp;
	}
	if (is_s) {
		tmp = dpu_bts_scale_latency(is_s, src_w, dst_w, ppc, lmc);
		DPU_DEBUG_BTS("\tS_lat_cycle(%d)\n", tmp);
		lat_cycle += tmp;
	}
	if (is_c) {
		tmp = dpu_bts_afbc_latency(src_w, ppc, lmc);
		DPU_DEBUG_BTS("\tC_lat_cycle(%d)\n", tmp);
		lat_cycle += tmp;
	}

	return lat_cycle;
}

/*
 * unit : nsec x 1000
 * reference aclk : 100MHz (-> 10ns x 1000)
 */
static inline u32 dpu_bts_get_aclk_period_time(u32 aclk_mhz)
{
	return ((ACLK_100MHZ_PERIOD * 100) / aclk_mhz);
}

/* find min-ACLK to meet latency */
static u32 dpu_bts_find_latency_meet_aclk(u32 lat_cycle, u32 line_time,
		u32 criteria_v, u32 aclk_disp,
		bool is_yuv10, bool is_rotate, u32 rot_factor)
{
	u32 aclk_mhz = aclk_disp / 1000UL;
	u32 aclk_period, lat_time;
	u32 lat_time_max;

	DPU_DEBUG_BTS("\t(rot_factor = %d) (is_yuv10 = %d)\n",
			rot_factor, is_yuv10);

	/* lat_time_max: usec x 1000 */
	lat_time_max = line_time * criteria_v;

	/* find min-ACLK to able to cover initial latency */
	while (1) {
		/* aclk_period: nsec x 1000 */
		aclk_period = dpu_bts_get_aclk_period_time(aclk_mhz);
		lat_time = (lat_cycle * aclk_period) / 1000UL;
		lat_time = lat_time << is_yuv10;
		lat_time += TOTAL_BUS_LATENCY;
		if (is_rotate)
			lat_time = (lat_time * rot_factor) / MULTI_FACTOR;

		DPU_DEBUG_BTS("\tloop: (aclk_period = %d) (lat_time = %d)\n",
			aclk_period, lat_time);
		if (lat_time < lat_time_max)
			break;

		aclk_mhz += ACLK_MHZ_INC_STEP;
	}

	DPU_DEBUG_BTS("\t(lat_time = %d) (lat_time_max = %d)\n",
		lat_time, lat_time_max);

	return (aclk_mhz * 1000UL);
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

u64 dpu_bts_calc_aclk_disp(struct decon_device *decon,
		struct dpu_bts_win_config *config, u64 resol_clock)
{
	u64 s_ratio_h, s_ratio_v;
	u64 aclk_disp;
	u64 ppc;
	const struct dpu_fmt *fmt_info = dpu_find_fmt_info(config->format);
	u32 src_w, src_h;
	u32 is_scale;
	u32 lat_cycle, line_time;
	u32 cycle_per_line, line_mem_cnt;
	u32 criteria_v, tot_v;
	u32 rot_factor = ROTATION_FACTOR_SCUP;

	if (config->is_rot) {
		src_w = config->src_h;
		src_h = config->src_w;
	} else {
		src_w = config->src_w;
		src_h = config->src_h;
	}

	s_ratio_h = (src_w <= config->dst_w) ? MULTI_FACTOR : MULTI_FACTOR *
		(u64)src_w / (u64)config->dst_w;
	s_ratio_v = (src_h <= config->dst_h) ? MULTI_FACTOR : MULTI_FACTOR *
		(u64)src_h / (u64)config->dst_h;

	/* case for using dsc encoder 1ea at decon0 or decon1 */
	if ((decon->id != 2) && (decon->config.dsc.dsc_count == 1))
		ppc = ((decon->bts.ppc / 2UL) >= 1UL) ?
				(decon->bts.ppc / 2UL) : 1UL;
	else
		ppc = decon->bts.ppc;

	aclk_disp = resol_clock * s_ratio_h * s_ratio_v * DISP_FACTOR  / 100UL /
		ppc * (MULTI_FACTOR * (u64)config->dst_w /
		(u64)decon->config.image_width) /
		(MULTI_FACTOR * MULTI_FACTOR * MULTI_FACTOR);

	if (aclk_disp < (resol_clock / ppc))
		aclk_disp = resol_clock / ppc;

	DPU_DEBUG_BTS("BEFORE latency calc: aclk_disp = %d\n", (u32)aclk_disp);

	/* scaling latency: width only */
	if (src_w < config->dst_w)
		is_scale = DPP_SCALE_UP;
	else if (src_w > config->dst_w) {
		is_scale = DPP_SCALE_DOWN;
		rot_factor = ROTATION_FACTOR_SCDN;
	} else
		is_scale = DPP_SCALE_NONE;

	/* to check initial latency */
	cycle_per_line = decon->bts.cycle_per_line;
	line_mem_cnt = decon->bts.line_mem_cnt;
	lat_cycle = dpu_bts_get_initial_latency(config->is_rot, is_scale,
			config->is_comp, &decon->config.dsc, src_w,
			config->dst_w, (u32)ppc, cycle_per_line, line_mem_cnt);
	line_time = dpu_bts_get_one_line_time(decon->config.image_height,
			decon->bts.vbp, decon->bts.vfp, decon->bts.vsa,
			decon->bts.fps);

	if (decon->config.mode.op_mode == DECON_VIDEO_MODE) {
		criteria_v = decon->bts.vbp;
	} else {
		/* command mode margin : apply 20% of v-blank time */
		tot_v = decon->bts.vfp + decon->bts.vsa + decon->bts.vbp;
		criteria_v = tot_v - (tot_v * 20 / 100);
	}

	aclk_disp = dpu_bts_find_latency_meet_aclk(lat_cycle, line_time,
			criteria_v, aclk_disp, IS_YUV10(fmt_info),
			config->is_rot, rot_factor);

	DPU_DEBUG_BTS("\t[R:%d C:%d S:%d] (lat_cycle=%d) (line_time=%d)\n",
		config->is_rot, config->is_comp, is_scale, lat_cycle,
		line_time);
	DPU_DEBUG_BTS("AFTER latency calc: aclk_disp = %d\n", (u32)aclk_disp);

	return aclk_disp;
}

static void dpu_bts_sum_all_decon_bw(struct decon_device *decon, u32 ch_bw[])
{
	int i, j;

	if (decon->id < 0 || decon->id >= MAX_DECON_CNT) {
		DPU_INFO_BTS("[%s] undefined decon id(%d)!\n", __func__,
				decon->id);
		return;
	}

	for (i = 0; i < MAX_DECON_CNT; ++i)
		decon->bts.ch_bw[decon->id][i] = ch_bw[i];

	for (i = 0; i < MAX_DECON_CNT; ++i) {
		if (decon->id == i)
			continue;

		for (j = 0; j < MAX_DECON_CNT; ++j)
			ch_bw[j] += decon->bts.ch_bw[i][j];
	}
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

	return dpu_bts_calc_aclk_disp(decon, &config, decon->bts.resol_clk);
}

static void dpu_bts_find_max_disp_freq(struct decon_device *decon)
{
	int i, j;
	u32 disp_ch_bw[MAX_DECON_CNT];
	u32 max_disp_ch_bw;
	u32 disp_op_freq = 0, freq = 0;
	struct dpu_bts_win_config *config = decon->bts.win_config;

	memset(disp_ch_bw, 0, sizeof(disp_ch_bw));

	for (i = 0; i < MAX_DPP_CNT; ++i)
		for (j = 0; j < MAX_DECON_CNT; ++j)
			if (decon->bts.bw[i].ch_num == j)
				disp_ch_bw[j] += decon->bts.bw[i].val;

	/* must be considered other decon's bw */
	dpu_bts_sum_all_decon_bw(decon, disp_ch_bw);

	for (i = 0; i < MAX_DECON_CNT; ++i)
		if (disp_ch_bw[i])
			DPU_DEBUG_BTS("\tPort%d = %d\n", i, disp_ch_bw[i]);

	max_disp_ch_bw = disp_ch_bw[0];
	for (i = 1; i < MAX_DECON_CNT; ++i)
		if (max_disp_ch_bw < disp_ch_bw[i])
			max_disp_ch_bw = disp_ch_bw[i];

	decon->bts.peak = max_disp_ch_bw;
	decon->bts.max_disp_freq = max_disp_ch_bw * 100 / (16 * BUS_UTIL) + 1;

	DPU_DEBUG_BTS("\tDECON%d : resol clock = %d Khz\n", decon->id,
			decon->bts.resol_clk);

	for (i = 0; i < decon->win_cnt; ++i) {
		if ((config[i].state != DPU_WIN_STATE_BUFFER) &&
				(config[i].state != DPU_WIN_STATE_COLOR))
			continue;

		freq = dpu_bts_calc_aclk_disp(decon, &config[i],
				(u64)decon->bts.resol_clk);
		if (disp_op_freq < freq)
			disp_op_freq = freq;
	}

	/*
	 * At least one window is used for colormap if there is a request of
	 * disabling all windows. So, disp frequency for a window of LCD full
	 * size is necessary.
	 */
	if (disp_op_freq == 0)
		disp_op_freq = dpu_bts_calc_disp_with_full_size(decon);

	DPU_DEBUG_BTS("\tDISP bus freq(%d), operating freq(%d)\n",
			decon->bts.max_disp_freq, disp_op_freq);

	if (decon->bts.max_disp_freq < disp_op_freq)
		decon->bts.max_disp_freq = disp_op_freq;

	DPU_DEBUG_BTS("\tMAX DISP CH FREQ = %d\n", decon->bts.max_disp_freq);
}

static void dpu_bts_share_bw_info(int id)
{
	int i, j;
	struct decon_device *decon[3];

	for (i = 0; i < MAX_DECON_CNT; i++)
		decon[i] = NULL;

	for (i = 0; i < MAX_DECON_CNT; i++)
		decon[i] = get_decon_drvdata(i);

	for (i = 0; i < MAX_DECON_CNT; ++i) {
		if (id == i || decon[i] == NULL)
			continue;

		for (j = 0; j < MAX_DECON_CNT; ++j)
			decon[i]->bts.ch_bw[id][j] =
				decon[id]->bts.ch_bw[id][j];
	}
}

static void dpu_bts_calc_dpp_bw(struct bts_decon_info *bts_info, int idx)
{
	struct bts_dpp_info *dpp = &bts_info->dpp[idx];
	unsigned int dst_w, dst_h;

	dst_w = dpp->dst.x2 - dpp->dst.x1;
	dst_h = dpp->dst.y2 - dpp->dst.y1;

	dpp->bw = ((u64)dpp->src_h * dpp->src_w * dpp->bpp * bts_info->vclk) /
		(8 * dst_h * bts_info->lcd_w);

	DPU_DEBUG_BTS("\tDPP%d bandwidth = %d\n", idx, dpp->bw);
}

void dpu_bts_calc_bw(struct decon_device *decon)
{
	struct dpu_bts_win_config *config = decon->bts.win_config;
	struct bts_decon_info bts_info;
	const struct dpu_fmt *fmt_info;
	int idx, i;
	u32 total_bw = 0;
	u64 resol_clock;

	if (!decon->bts.enabled)
		return;

	DPU_DEBUG_BTS("\n");
	DPU_DEBUG_BTS("%s + : DECON%d\n", __func__, decon->id);

	memset(&bts_info, 0, sizeof(struct bts_decon_info));

	resol_clock = dpu_bts_get_resol_clock(decon->config.image_width,
				decon->config.image_height, decon->bts.fps);
	decon->bts.resol_clk = (u32)resol_clock;
	DPU_DEBUG_BTS("[Run: D%d] resol clock = %d Khz @%d fps\n",
		decon->id, decon->bts.resol_clk, decon->bts.fps);

	bts_info.vclk = decon->bts.resol_clk;
	bts_info.lcd_w = decon->config.image_width;
	bts_info.lcd_h = decon->config.image_height;

	for (i = 0; i < decon->win_cnt; ++i) {
		if (config[i].state == DPU_WIN_STATE_BUFFER) {
			idx = config[i].dpp_ch; /* ch */
			/*
			 * TODO: Array index of bts_info structure uses dma type
			 * This array index will be changed to DPP channel
			 * number in the future.
			 */
			bts_info.dpp[idx].used = true;
		} else {
			continue;
		}

		fmt_info = dpu_find_fmt_info(config[i].format);
		bts_info.dpp[idx].bpp = fmt_info->bpp + fmt_info->padding;
		bts_info.dpp[idx].src_w = config[i].src_w;
		bts_info.dpp[idx].src_h = config[i].src_h;
		bts_info.dpp[idx].dst.x1 = config[i].dst_x;
		bts_info.dpp[idx].dst.x2 = config[i].dst_x + config[i].dst_w;
		bts_info.dpp[idx].dst.y1 = config[i].dst_y;
		bts_info.dpp[idx].dst.y2 = config[i].dst_y + config[i].dst_h;
		bts_info.dpp[idx].rotation = config[i].is_rot;

		/*
		 * [ GUIDE : #if 0 ]
		 * Need to apply twice instead of x1.25 as many bandwidth
		 * of 8-bit YUV if it is a 8P2 format rotation.
		 *
		 * [ APPLY : #else ]
		 * In case of rotation, MIF & INT and DISP clock frequencies
		 * are important factors related to the issue of underrun.
		 * So, relatively high frequency is required to avoid underrun.
		 * By using 32 instead of 12/15/24 as bpp(bit-per-pixel) value,
		 * MIF & INT frequency can be increased because
		 * those are decided by the bandwidth.
		 * ROTATION_FACTOR_BPP(= ARGB8888 value) is a tunable value.
		 */
		if (bts_info.dpp[idx].rotation)
			bts_info.dpp[idx].bpp = ROTATION_FACTOR_BPP;

		DPU_DEBUG_BTS("\tDPP%d : bpp(%d) src w(%d) h(%d) rot(%d)\n",
				DPU_DMA2CH(idx), bts_info.dpp[idx].bpp,
				bts_info.dpp[idx].src_w,
				bts_info.dpp[idx].src_h,
				bts_info.dpp[idx].rotation);
		DPU_DEBUG_BTS("\t\t\t\tdst x(%d) right(%d) y(%d) bottom(%d)\n",
				bts_info.dpp[idx].dst.x1,
				bts_info.dpp[idx].dst.x2,
				bts_info.dpp[idx].dst.y1,
				bts_info.dpp[idx].dst.y2);

		dpu_bts_calc_dpp_bw(&bts_info, idx);
		total_bw += bts_info.dpp[idx].bw;
	}

	decon->bts.total_bw = total_bw;
	memcpy(&decon->bts.bts_info, &bts_info, sizeof(struct bts_decon_info));

	for (i = 0; i < MAX_DPP_CNT; ++i) {
		decon->bts.bw[i].val = bts_info.dpp[i].bw;
		if (decon->bts.bw[i].val)
			DPU_DEBUG_BTS("\tDPP%d bandwidth = %d\n",
					DPU_DMA2CH(i), decon->bts.bw[i].val);
	}

	DPU_DEBUG_BTS("\tDECON%d total bandwidth = %d\n", decon->id,
			decon->bts.total_bw);

	dpu_bts_find_max_disp_freq(decon);

	/* update bw for other decons */
	dpu_bts_share_bw_info(decon->id);

	DPU_DEBUG_BTS("%s -\n", __func__);
}

void dpu_bts_update_bw(struct decon_device *decon, bool shadow_updated)
{
	struct bts_bw bw = { 0, };

	DPU_DEBUG_BTS("%s +\n", __func__);

	if (!decon->bts.enabled)
		return;

	/* update peak & read bandwidth per DPU port */
	bw.peak = decon->bts.peak;
	bw.read = decon->bts.total_bw;
	DPU_DEBUG_BTS("\tpeak = %d, read = %d\n", bw.peak, bw.read);

	if (bw.read == 0)
		bw.peak = 0;

	if (shadow_updated) {
		/* after DECON h/w configs are updated to shadow SFR */
		if (decon->bts.total_bw < decon->bts.prev_total_bw)
			bts_update_bw(decon->bts.bw_idx, bw);

		if (decon->bts.max_disp_freq < decon->bts.prev_max_disp_freq)
			exynos_pm_qos_update_request(&decon->bts.disp_qos,
					decon->bts.max_disp_freq);

		decon->bts.prev_total_bw = decon->bts.total_bw;
		decon->bts.prev_max_disp_freq = decon->bts.max_disp_freq;
	} else {
		if (decon->bts.total_bw > decon->bts.prev_total_bw)
			bts_update_bw(decon->bts.bw_idx, bw);

		if (decon->bts.max_disp_freq > decon->bts.prev_max_disp_freq)
			exynos_pm_qos_update_request(&decon->bts.disp_qos,
					decon->bts.max_disp_freq);
	}

	DPU_DEBUG_BTS("%s -\n", __func__);
}

void dpu_bts_acquire_bw(struct decon_device *decon)
{
	u32 aclk_freq = 0;

	DPU_DEBUG_BTS("%s +\n", __func__);

	if (!decon->bts.enabled)
		return;

	if (decon->config.out_type & DECON_OUT_DSI) {
		aclk_freq = dpu_bts_calc_disp_with_full_size(decon);
		DPU_DEBUG_BTS("Initial calculated disp freq(%u)\n", aclk_freq);
		/*
		 * If current disp freq is higher than calculated freq,
		 * it must not be set. if not, underrun can occur.
		 */
#if defined(CONFIG_SOC_GS101)
		if (exynos_devfreq_get_domain_freq(DEVFREQ_DISP) < aclk_freq)
			exynos_pm_qos_update_request(&decon->bts.disp_qos,
					aclk_freq);

		DPU_DEBUG_BTS("Get initial disp freq(%lu)\n",
				exynos_devfreq_get_domain_freq(DEVFREQ_DISP));
#else
		if (cal_dfs_get_rate(ACPM_DVFS_DISP) < aclk_freq)
			exynos_pm_qos_update_request(&decon->bts.disp_qos,
					aclk_freq);

		DPU_DEBUG_BTS("Get initial disp freq(%lu)\n",
				cal_dfs_get_rate(ACPM_DVFS_DISP));
#endif

		decon->bts.prev_max_disp_freq = aclk_freq;

		return;
	}

	DPU_DEBUG_BTS("%s -\n", __func__);
}

void dpu_bts_release_bw(struct decon_device *decon)
{
	struct bts_bw bw = { 0, };

	DPU_DEBUG_BTS("%s +\n", __func__);

	if (!decon->bts.enabled)
		return;

	if (decon->config.out_type & DECON_OUT_DSI) {
		bts_update_bw(decon->bts.bw_idx, bw);
		decon->bts.prev_total_bw = 0;
		exynos_pm_qos_update_request(&decon->bts.disp_qos, 0);
		decon->bts.prev_max_disp_freq = 0;
	}

	DPU_DEBUG_BTS("%s -\n", __func__);
}

#define MAX_IDX_NAME_SIZE	16
void dpu_bts_init(struct decon_device *decon)
{
	int i;
	char bts_idx_name[MAX_IDX_NAME_SIZE];

	DPU_DEBUG_BTS("%s +\n", __func__);

	decon->bts.enabled = false;

	if (!IS_ENABLED(CONFIG_EXYNOS_BTS) ||
			(!IS_ENABLED(CONFIG_EXYNOS_PM_QOS) &&
			 !IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE))) {
		DPU_ERR_BTS("decon%d bts feature is disabled\n", decon->id);
		pr_info("%s:%d\n", __func__, __LINE__);
		return;
	}

	memset(bts_idx_name, 0, MAX_IDX_NAME_SIZE);
	snprintf(bts_idx_name, MAX_IDX_NAME_SIZE, "DECON%d", decon->id);
	decon->bts.bw_idx = bts_get_bwindex(bts_idx_name);

	for (i = 0; i < MAX_DECON_CNT; i++)
		decon->bts.ch_bw[decon->id][i] = 0;

	DPU_DEBUG_BTS("BTS_BW_TYPE(%d)\n", decon->bts.bw_idx);
	exynos_pm_qos_add_request(&decon->bts.mif_qos,
					PM_QOS_BUS_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&decon->bts.int_qos,
					PM_QOS_DEVICE_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&decon->bts.disp_qos,
					PM_QOS_DISPLAY_THROUGHPUT, 0);
	decon->bts.scen_updated = 0;

	for (i = 0; i < MAX_DPP_CNT; ++i) { /* dma type order */
		decon->bts.bw[i].ch_num = decon->dpp[DPU_DMA2CH(i)]->port;
		DPU_INFO_BTS("IDMA_TYPE(%d) CH(%d) Port(%d)\n", i,
				DPU_DMA2CH(i), decon->bts.bw[i].ch_num);
	}

	decon->bts.enabled = true;

	DPU_INFO_BTS("decon%d bts feature is enabled\n", decon->id);
}

void dpu_bts_deinit(struct decon_device *decon)
{
	if (!decon->bts.enabled)
		return;

	DPU_DEBUG_BTS("%s +\n", __func__);
	exynos_pm_qos_remove_request(&decon->bts.disp_qos);
	exynos_pm_qos_remove_request(&decon->bts.int_qos);
	exynos_pm_qos_remove_request(&decon->bts.mif_qos);
	DPU_DEBUG_BTS("%s -\n", __func__);
}

struct dpu_bts_ops dpu_bts_control = {
	.init		= dpu_bts_init,
	.calc_bw	= dpu_bts_calc_bw,
	.update_bw	= dpu_bts_update_bw,
	.acquire_bw	= dpu_bts_acquire_bw,
	.release_bw	= dpu_bts_release_bw,
	.deinit		= dpu_bts_deinit,
};
