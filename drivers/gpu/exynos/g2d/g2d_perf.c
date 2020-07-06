// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * Contact: Hyesoo Yu <hyesoo.yu@samsung.com>
 */

#include <linux/workqueue.h>

#include "g2d.h"
#include "g2d_perf.h"
#include "g2d_task.h"
#include "g2d_uapi.h"
#include "g2d_debug.h"

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
static void g2d_pm_qos_update_devfreq(struct g2d_device *g2d_dev, u32 freq)
{
	if (!exynos_pm_qos_request_active(&g2d_dev->req))
		exynos_pm_qos_add_request(&g2d_dev->req, PM_QOS_DEVICE_THROUGHPUT, 0);

	exynos_pm_qos_update_request(&g2d_dev->req, freq);
}

static void g2d_pm_qos_remove_devfreq(struct g2d_device *g2d_dev)
{
	if (exynos_pm_qos_request_active(&g2d_dev->req))
		exynos_pm_qos_remove_request(&g2d_dev->req);
}
#else
#define g2d_pm_qos_update_devfreq(g2d_dev, freq) do { } while (0)
#define g2d_pm_qos_remove_devfreq(g2d_dev) do { } while (0)
#endif

/*
 * The reference point is pixelcount scaling ratio that both width
 * and height are 1 times, 1/2 times, 1/3 times or 1/4 times.
 * To eliminate decimal point, shift to the left by 10 and
 * that value divided by the reference value is as follows.
 */
static u32 perf_basis[PPC_SC] = {1024, 1023, 256, 113, 64, 0};

static char perf_index_sc(struct g2d_performance_layer_data *layer)
{
	u32 ratio = (((u64)layer->window_w * layer->window_h) << 10) /
			((u32)layer->crop_w * layer->crop_h);
	int i;

	for (i = 0; i < PPC_SC; i++) {
		if (ratio > perf_basis[i])
			return i;
	}

	return PPC_SC_DOWN_16;
}

u32 g2d_calc_device_frequency(struct g2d_device *g2d_dev,
			      struct g2d_performance_data *data)
{
	struct g2d_performance_frame_data *frame;
	struct g2d_performance_layer_data *layer;
	u32 (*ppc)[PPC_ROT][PPC_SC] = (u32 (*)[PPC_ROT][PPC_SC])g2d_dev->hw_ppc;
	unsigned int cycle, ip_clock, crop, window;
	int i, j;
	int sc, fmt, rot;

	cycle = 0;

	for (i = 0; i < data->num_frame; i++) {
		frame = &data->frame[i];

		rot = 0;
		for (j = 0; j < frame->num_layers; j++) {
			if (perf_index_rotate(&frame->layer[j])) {
				rot++;
				break;
			}
		}

		for (j = 0; j < frame->num_layers; j++) {
			layer = &frame->layer[j];

			crop = (u32)layer->crop_w * layer->crop_h;
			window = (u32)layer->window_w * layer->window_h;

			fmt = perf_index_fmt(layer);
			sc = perf_index_sc(layer);

			if (fmt == PPC_FMT)
				return 0;

			cycle += max(crop, window) / ppc[fmt][rot][sc];

			/*
			 * If frame has colorfill layer on the bottom,
			 * upper layaer is treated as opaque.
			 * In this case, colorfill is not be processed
			 * as much as the overlapping area.
			 */
			if (!j && is_perf_frame_colorfill(frame)) {
				unsigned int pixelcount;
				unsigned int colorfill_cycle;

				pixelcount = frame->target_pixelcount - window;
				colorfill_cycle = (pixelcount > 0) ?
					pixelcount /
					g2d_dev->hw_ppc[PPC_COLORFILL] : 0;

				g2d_perf("%d: dst %8d win %8d ppc %4d cycl %8d",
					 j, frame->target_pixelcount, window,
					 g2d_dev->hw_ppc[PPC_COLORFILL],
					 colorfill_cycle);

				cycle += colorfill_cycle;
			}

			g2d_perf("%d: crop %8d window %8d ppc %4d cycle %8d",
				 is_perf_frame_colorfill(frame) ? j + 1 : j,
				 crop, window, ppc[fmt][rot][sc], cycle);
		}
	}

	/* ip_clock(Mhz) = cycles / time_in_ms * 1000 * 10% */
	ip_clock = (cycle / 7) * 1100;

	for (i = 0; i < g2d_dev->dvfs_table_cnt; i++) {
		if (ip_clock > g2d_dev->dvfs_table[i].freq) {
			ip_clock = (i == 0) ?
					g2d_dev->dvfs_table[i].lv :
					g2d_dev->dvfs_table[i - 1].lv;
			break;
		}
	}

	return ip_clock;
}

void g2d_update_performance(struct g2d_device *g2d_dev)
{
	struct g2d_context *ctx;
	struct g2d_task *task;
	struct bts_bw bw;
	struct g2d_qos qos = {0, };

	/* Find maximum request from contexts */
	list_for_each_entry(ctx, &g2d_dev->qos_contexts, qos_node) {
		qos.rbw = max_t(u64, ctx->ctxqos.rbw, qos.rbw);
		qos.wbw = max_t(u64, ctx->ctxqos.wbw, qos.wbw);
		qos.devfreq = max_t(u32, ctx->ctxqos.devfreq,
				       qos.devfreq);
	}

	/* Update maximum performance among contexts */
	g2d_dev->qos = qos;

	/* Find task to need more than current performance */
	for (task = g2d_dev->tasks; task != NULL; task = task->next) {
		qos.rbw = max_t(u64, task->taskqos.rbw, qos.rbw);
		qos.wbw = max_t(u64, task->taskqos.wbw, qos.wbw);
		qos.devfreq = max_t(u32, task->taskqos.devfreq,
				       qos.devfreq);
	}

	if (!qos.devfreq)
		g2d_pm_qos_remove_devfreq(g2d_dev);
	else
		g2d_pm_qos_update_devfreq(g2d_dev, qos.devfreq);

	g2d_perf("DVFS_INT freq : request %u, current %lu",
		 qos.devfreq, g2d_get_current_freq(g2d_dev->dvfs_int));

	bw.peak = (qos.rbw + qos.wbw) / 2;
	bw.write = qos.wbw;
	bw.read = qos.rbw;

	g2d_update_bw(bw);

	g2d_perf("DVFS_MIF freq : request r %d w %d current %lu",
		 bw.read, bw.write,
		 g2d_get_current_freq(g2d_dev->dvfs_mif));

	/*
	 * Request of performance should be released by explicit user request
	 * or delayed work after some time or context released.
	 */
	if (!qos.rbw && !qos.wbw && !qos.devfreq) {
		cancel_delayed_work(&g2d_dev->dwork);
	} else {
		mod_delayed_work(system_wq,
				 &g2d_dev->dwork, msecs_to_jiffies(50));
	}
}
