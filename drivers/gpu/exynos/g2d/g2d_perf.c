/*
 * linux/drivers/gpu/exynos/g2d/g2d_perf.c
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * Contact: Hyesoo Yu <hyesoo.yu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include "g2d.h"
#include "g2d_perf.h"
#include "g2d_task.h"
#include "g2d_uapi.h"

#ifdef CONFIG_PM_DEVFREQ
static void g2d_pm_qos_update_devfreq(struct pm_qos_request *req, u32 freq)
{
	/*
	 * FIXME: PM_QOS_DEVICE_THROUGHPUT is not in the upstream kernel.
	 * if (!pm_qos_request_active(req))
	 *	pm_qos_add_request(req, PM_QOS_DEVICE_THROUGHPUT, 0);
	 */

	pm_qos_update_request(req, freq);
}

static void g2d_pm_qos_remove_devfreq(struct pm_qos_request *req)
{
	if (pm_qos_request_active(req))
		pm_qos_remove_request(req);
}
#else
#define g2d_pm_qos_update_devfreq(req, freq) do { } while (0)
#define g2d_pm_qos_remove_devfreq(req) do { } while (0)
#endif

static bool g2d_still_need_perf(struct g2d_device *g2d_dev)
{
	struct g2d_task *task;
	unsigned long flags;

	spin_lock_irqsave(&g2d_dev->lock_task, flags);
	for (task = g2d_dev->tasks; task != NULL; task = task->next) {
		if (!is_task_state_idle(task)) {
			spin_unlock_irqrestore(&g2d_dev->lock_task, flags);
			return true;
		}
	}
	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

	return false;
}

static void g2d_set_device_frequency(struct g2d_context *g2d_ctx,
					  struct g2d_performance_data *data)
{
	struct g2d_device *g2d_dev = g2d_ctx->g2d_dev;
	struct g2d_performance_frame_data *frame;
	struct g2d_performance_layer_data *layer, *pair;
	unsigned int cycle, cycle_src, cycle_dst, ip_clock;
	unsigned int rot_size, no_rot_size;
	unsigned int dst_ppc, ppc[G2D_MAX_IMAGES];
	int i, j;
	char sc, yuv2p, rot, rot_skip, gap;

	cycle = 0;
	gap = false;

	for (i = 0; i < data->num_frame; i++) {
		frame = &data->frame[i];

		rot_size = 0;
		no_rot_size = 0;
		cycle_src = 0;

		/*
		 * The rotate variable means that the rotated layers and
		 * non-rotated layers are mixed.
		 * If all layers are rotated or are non rotated, that is
		 * excluded.
		 */
		rot = 0;
		for (j = 0; j < frame->num_layers; j++) {
			if (is_perf_layer_rotate(&frame->layer[j]))
				rot++;
		}
		rot_skip = (rot == frame->num_layers) ? 1 : 0;

		for (j = 0; j < frame->num_layers; j++) {
			layer = &frame->layer[j];

			yuv2p = is_perf_layer_yuv2p(layer) ? 1 : 0;
			sc = is_perf_layer_scaling(layer) ? 1 : 0;
			rot = !rot_skip && is_perf_layer_rotate(layer) ? 1 : 0;

			ppc[j] =
				g2d_dev->hw_ppc[(yuv2p << 2) | (rot << 1) | sc];

			cycle_src += layer->pixelcount / ppc[j];

			/*
			 * check rotated size for cycle_dst. rotated size is
			 * bigger than non-rotated size, g2d write direction
			 * is vertical, and it affects performance.
			 */
			if (is_perf_layer_rotate(layer))
				rot_size += layer->pixelcount;
			else
				no_rot_size += layer->pixelcount;

			/*
			 * The rotated layer affects the pair layer,
			 * so we add the cycle using gap_ppc between pair
			 * N layer and N+1 layer. The gap ppc is calculated
			 * on odd layer and gap_pixelcount is pair layer's
			 * nested region from 2 layers that means
			 * the smaller region.
			 */
			if (rot && (yuv2p || sc))
				gap = true;

			if (gap && (j & 0x1)) {
				unsigned int gap_pixelcount, gap_ppc;

				pair = &frame->layer[j - 1];
				gap = false;

				gap_ppc = (ppc[j] > ppc[j - 1]) ?
					(ppc[j] - ppc[j - 1]) :
					(ppc[j - 1] - ppc[j]);
				if (!gap_ppc)
					continue;

				gap_ppc = (ppc[j] * ppc[j - 1]) / gap_ppc;

				gap_pixelcount = min(layer->pixelcount, pair->pixelcount);

				cycle_src += gap_pixelcount / gap_ppc;
			}
		}

		rot = (rot_size > no_rot_size) ? 1 : 0;
		if (!rot && is_perf_frame_yuv2p(frame))
			dst_ppc = g2d_dev->hw_ppc[G2D_PPC_DST_YUV2P];
		else if (!rot)
			dst_ppc = g2d_dev->hw_ppc[G2D_PPC_DST_DEFAULT];
		else
			dst_ppc = g2d_dev->hw_ppc[G2D_PPC_DST_ROT];

		cycle_dst = frame->target_pixelcount / dst_ppc;

		cycle += max(cycle_src, cycle_dst);

		if (is_perf_frame_colorfill(frame))
			cycle += frame->target_pixelcount /
					g2d_dev->hw_ppc[G2D_PPC_COLORFILL];
	}

	/* ip_clock(Mhz) = cycles / time_in_ms * 1000 */
	ip_clock = (cycle / 8) * 1000;

	for (i = 0; i < g2d_dev->dvfs_table_cnt; i++) {
		if (ip_clock > g2d_dev->dvfs_table[i].freq) {
			ip_clock = (i == 0) ?
					g2d_dev->dvfs_table[i].lv :
					g2d_dev->dvfs_table[i - 1].lv;
			break;
		}
	}

	if (!ip_clock && !g2d_still_need_perf(g2d_dev))
		g2d_pm_qos_remove_devfreq(&g2d_ctx->req);
	else if (ip_clock)
		g2d_pm_qos_update_devfreq(&g2d_ctx->req, ip_clock);
}

static void g2d_set_qos_frequency(struct g2d_context *g2d_ctx,
					  struct g2d_performance_data *data)
{
	struct g2d_device *g2d_dev = g2d_ctx->g2d_dev;
	struct g2d_performance_frame_data *frame;
	u32 cur_rbw, rbw;
	u32 cur_wbw, wbw;
	int i;

	cur_rbw = 0;
	cur_wbw = 0;
	rbw = 0;
	wbw = 0;

	for (i = 0; i < data->num_frame; i++) {
		frame = &data->frame[i];

		rbw += frame->bandwidth_read;
		wbw += frame->bandwidth_write;
	}

	if (list_empty(&g2d_ctx->qos_node) && !rbw && !wbw)
		return;

	if (!rbw && !rbw && g2d_still_need_perf(g2d_dev))
		return;

	mutex_lock(&g2d_dev->lock_qos);

	if (!list_empty(&g2d_dev->qos_contexts)) {
		struct g2d_context *ctx_qos;

		ctx_qos = list_first_entry(&g2d_dev->qos_contexts,
					   struct g2d_context, qos_node);
		cur_rbw = ctx_qos->r_bw;
		cur_wbw = ctx_qos->w_bw;
	}

	/* this works although ctx is not attached to qos_contexts */
	list_del_init(&g2d_ctx->qos_node);

	g2d_ctx->r_bw = rbw;
	g2d_ctx->w_bw = wbw;

	if (rbw || wbw) {
		struct list_head *node;

		for (node = g2d_dev->qos_contexts.prev;
				node != &g2d_dev->qos_contexts;
						node = node->prev) {
			struct g2d_context *curctx = list_entry(node,
					struct g2d_context, qos_node);
			if ((curctx->r_bw + curctx->w_bw) > (rbw + wbw))
				break;
		}
		/*
		 * node always points to the head node or the smallest bw node
		 * among the larger bw nodes than qosnode
		 */
		list_add(&g2d_ctx->qos_node, node);
	}

	if (!list_empty(&g2d_dev->qos_contexts)) {
		struct g2d_context *ctx_qos;

		ctx_qos = list_first_entry(&g2d_dev->qos_contexts,
				      struct g2d_context, qos_node);
		/* bandwidth request is changed */
		rbw = ctx_qos->r_bw;
		wbw = ctx_qos->w_bw;
	}

	if ((rbw != cur_rbw) || (wbw != cur_wbw)) {
		/*
		 * FIXME: BTS is not available for now
		 * struct bts_bw bw;
		 *
		 * bw.peak = ((rbw + wbw) / 1000) * BTS_PEAK_FPS_RATIO / 2;
		 * bw.write = wbw;
		 * bw.read = rbw;
		 * bts_update_bw(BTS_BW_G2D, bw);
		 */
	}

	mutex_unlock(&g2d_dev->lock_qos);
}

void g2d_set_performance(struct g2d_context *ctx,
				struct g2d_performance_data *data)
{
	g2d_set_qos_frequency(ctx, data);
	g2d_set_device_frequency(ctx, data);
}

void g2d_put_performance(struct g2d_context *ctx)
{
	struct g2d_performance_data data;

	data.num_frame = 0;

	g2d_set_performance(ctx, &data);
}
