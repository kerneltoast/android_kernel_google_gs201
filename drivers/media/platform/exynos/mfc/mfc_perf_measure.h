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

#include "mfc_reg_api.h"

void mfc_perf_register(struct mfc_dev *dev);
void __mfc_measure_init(void);
void __mfc_measure_on(struct mfc_dev *dev);
void __mfc_measure_off(struct mfc_dev *dev);
void __mfc_measure_store(struct mfc_dev *dev, int diff);
void mfc_perf_print(void);

//#define PERF_MEASURE

#ifndef PERF_MEASURE

static inline void mfc_perf_init(struct mfc_dev *dev) {}
static inline void mfc_perf_cancel_drv_margin(struct mfc_dev *dev) {}
static inline void mfc_perf_measure_on(struct mfc_dev *dev) {}
static inline void mfc_perf_measure_off(struct mfc_dev *dev) {}

#else

extern unsigned int perf_measure_option;

static inline void mfc_perf_init(struct mfc_dev *dev)
{
	dev->perf.new_start = 0;
	dev->perf.count = 0;
	dev->perf.drv_margin = 0;

	__mfc_measure_init();

	mfc_dev_info("MFC frequency : %ld\n", clk_get_rate(dev->pm.clock));
}

static inline void mfc_perf_cancel_drv_margin(struct mfc_dev *dev)
{
	dev->perf.drv_margin = 0;
}

static inline void mfc_perf_measure_on(struct mfc_dev *dev)
{
	int diff;

	if (dev->perf.drv_margin) {
		ktime_get_ts64(&dev->perf.end);

		diff = (dev->perf.end.tv_sec * NSEC_PER_SEC + dev->perf.end.tv_nsec)
			- (dev->perf.begin.tv_sec * NSEC_PER_SEC + dev->perf.begin.tv_nsec);

		mfc_dev_info("IRQ -> NAL_START time(ms) = %03d.%06d\n",
				diff / NSEC_PER_MSEC, diff % NSEC_PER_MSEC);

		dev->perf.drv_margin = 0;
	}

	ktime_get_ts64(&dev->perf.begin);

	__mfc_measure_on(dev);

	dev->perf.new_start = 1;
	dev->perf.count++;
}

static inline void mfc_perf_measure_off(struct mfc_dev *dev)
{
	unsigned int diff;

	if ((dev->perf.new_start) && (dev->perf.count > 0)) {
		__mfc_measure_off(dev);

		ktime_get_ts64(&dev->perf.end);

		diff = (dev->perf.end.tv_sec * NSEC_PER_SEC + dev->perf.end.tv_nsec)
			- (dev->perf.begin.tv_sec * NSEC_PER_SEC + dev->perf.begin.tv_nsec);

		__mfc_measure_store(dev, diff / NSEC_PER_USEC);

		mfc_dev_debug(3, "uDECtype :%d, uENCtype :%d, codectype :%d\n",
			mfc_get_dec_frame_type(), mfc_get_enc_slice_type(), MFC_READL(MFC_REG_CODEC_TYPE));

		dev->perf.drv_margin = 1;

		ktime_get_ts_64(&dev->perf.begin);
	}

	dev->perf.new_start = 0;
}

#endif

#endif /* __MFC_PERF_MEASURE_H */
