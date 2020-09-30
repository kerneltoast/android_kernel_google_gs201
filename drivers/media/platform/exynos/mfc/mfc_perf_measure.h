/*
 * drivers/media/platform/exynos/mfc/mfc_perf_measure.h
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_PERF_MEASURE_H
#define __MFC_PERF_MEASURE_H __FILE__

#include <linux/clk.h>

#include "mfc_core_reg_api.h"

void mfc_perf_register(struct mfc_core *core);
void __mfc_measure_init(void);
void __mfc_measure_on(struct mfc_core *core);
void __mfc_measure_off(struct mfc_core *core);
void __mfc_measure_store(struct mfc_core *core, int diff);
void mfc_perf_print(void);

//#define PERF_MEASURE

#ifndef PERF_MEASURE

static inline void mfc_perf_init(struct mfc_core *core) {}
static inline void mfc_perf_cancel_drv_margin(struct mfc_core *core) {}
static inline void mfc_perf_measure_on(struct mfc_core *core) {}
static inline void mfc_perf_measure_off(struct mfc_core *core) {}

#else

extern unsigned int perf_measure_option;

static inline void mfc_perf_init(struct mfc_core *core)
{
	core->perf.new_start = 0;
	core->perf.count = 0;
	core->perf.drv_margin = 0;

	__mfc_measure_init();

	mfc_core_info("MFC frequency : %ld\n", clk_get_rate(core->pm.clock));
}

static inline void mfc_perf_cancel_drv_margin(struct mfc_core *core)
{
	core->perf.drv_margin = 0;
}

static inline void mfc_perf_measure_on(struct mfc_core *core)
{
	int diff;

	if (core->perf.drv_margin) {
		ktime_get_ts64(&core->perf.end);

		diff = (core->perf.end.tv_sec * NSEC_PER_SEC + core->perf.end.tv_nsec)
			- (core->perf.begin.tv_sec * NSEC_PER_SEC + core->perf.begin.tv_nsec);

		mfc_core_info("IRQ -> NAL_START time(ms) = %03d.%06d\n",
			diff / NSEC_PER_MSEC, diff % NSEC_PER_MSEC);

		core->perf.drv_margin = 0;
	}

	ktime_get_ts64(&core->perf.begin);

	__mfc_measure_on(core);

	core->perf.new_start = 1;
	core->perf.count++;
}

static inline void mfc_perf_measure_off(struct mfc_core *core)
{
	unsigned int diff;

	if ((core->perf.new_start) && (core->perf.count > 0)) {
		__mfc_measure_off(core);

		ktime_get_ts64(&core->perf.end);

		diff = (core->perf.end.tv_sec * NSEC_PER_SEC + core->perf.end.tv_nsec)
			- (core->perf.begin.tv_sec * NSEC_PER_SEC + core->perf.begin.tv_nsec);

		__mfc_measure_store(core, diff / NSEC_PER_SEC);

		mfc_core_debug(3, "uDECtype :%d, uENCtype :%d, codectype :%d\n",
				mfc_core_get_dec_frame_type(),
				mfc_core_get_enc_slice_type(),
				MFC_CORE_READL(MFC_REG_CODEC_TYPE));

		core->perf.drv_margin = 1;

		ktime_get_ts64(&core->perf.begin);
	}

	core->perf.new_start = 0;
}

#endif

#endif /* __MFC_PERF_MEASURE_H */
