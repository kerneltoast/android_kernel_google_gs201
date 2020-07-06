/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 */

#ifndef _G2D_PERF_H_
#define _G2D_PERF_H_

struct g2d_context;
struct g2d_performance_data;

#define perf_index_fmt(layer) \
		(fls((((layer)->layer_attr) & G2D_PERF_LAYER_FMTMASK) >> 4))
#define perf_index_rotate(layer) \
		(((layer)->layer_attr) & G2D_PERF_LAYER_ROTATE)
#define is_perf_frame_colorfill(frame) \
		(((frame)->frame_attr) & G2D_PERF_FRAME_SOLIDCOLORFILL)

#define BTS_PEAK_FPS_RATIO 1667

u32 g2d_calc_device_frequency(struct g2d_device *g2d_dev,
			      struct g2d_performance_data *data);
void g2d_update_performance(struct g2d_device *g2d_dev);

#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
#include <soc/google/exynos-devfreq.h>
static inline unsigned long g2d_get_current_freq(unsigned int type)
{
	return exynos_devfreq_get_domain_freq(type);
}
#else
static inline unsigned long g2d_get_current_freq(unsigned int type)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
#include <soc/google/bts.h>

static inline void g2d_update_bw(struct bts_bw bw)
{
	int id = bts_get_bwindex("g2d");

	if (id >= 0)
		bts_update_bw(id, bw);
}
#elif IS_ENABLED(CONFIG_EXYNOS9820_BTS)
#include <soc/samsung/bts.h>
static inline void g2d_update_bw(struct bts_bw bw)
{
	bts_update_bw(BTS_BW_G2D, bw);
}
#else
/* dummy definition for build success when the BTS module does not exist */
struct bts_bw {
	char *name;
	unsigned int peak;
	unsigned int read;
	unsigned int write;
};

#define g2d_update_bw(bw) do { } while (0)
#endif

#endif /* _G2D_PERF_H_ */
