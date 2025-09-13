// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/component.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_uapi.h>
#include <drm/drm_drv.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_self_refresh_helper.h>
#include <drm/drm_vblank.h>

#include <drm/exynos_drm.h>
#include <drm/samsung_drm.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_connector.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_gem.h"
#include "exynos_drm_plane.h"
#include "exynos_drm_writeback.h"
#include "exynos_drm_dqe.h"

#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
#include "gs_drm/gs_drm_connector.h"
#endif

#define CREATE_TRACE_POINTS
#include <trace/dpu_trace.h>

#define DRIVER_NAME	"exynos"
#define DRIVER_DESC	"Samsung SoC DRM"
#define DRIVER_DATE	"20110530"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

#define EXYNOS_DRM_WAIT_FENCE_TIMEOUT_MS 250

EXPORT_TRACEPOINT_SYMBOL(tracing_mark_write);
EXPORT_TRACEPOINT_SYMBOL(dsi_label_scope);

static void exynos_drm_drv_set_lhbm_hist_helper(struct histogram_channel_config *cfg,
						struct histogram_roi *roi,
						struct histogram_weights *weight)
{
	/*
	 * HIST_THRESHOLD 1 ~ 1023 Histogram threshold
	 */
	memcpy(&cfg->weights, weight, sizeof(cfg->weights));
	memcpy(&cfg->roi, roi, sizeof(cfg->roi));
	/* rounded up with min value of 1 (threshold range: 1 ~ 1023)*/
	cfg->threshold = ((roi->hsize * roi->vsize) >> 16) + 1;
	cfg->pos = POST_DQE;
}

/* connector->base.dev->mode_config.connection_mutex should be acquired at calling this function */
int exynos_drm_drv_set_lhbm_hist(struct exynos_drm_connector *connector, int x, int y, int w, int h)
{
	struct drm_crtc *crtc = NULL;
	struct exynos_drm_crtc *exynos_crtc;
	struct decon_device *decon;
	struct histogram_channel_config *cfg;
	struct histogram_roi roi = { .start_x = x, .start_y = y, .hsize = w, .vsize = h };

	if (connector->base.state)
		crtc = connector->base.state->crtc;
	if (!crtc)
		return -EIO;

	exynos_crtc = to_exynos_crtc(crtc);
	decon = (struct decon_device *)exynos_crtc->ctx;
	if (!decon || !decon->dqe)
		return -EIO;
	cfg = &decon->dqe->lhbm_hist_config;

	exynos_drm_drv_set_lhbm_hist_helper(cfg, &roi, &lhbm_hist_weight[LHBM_CIRCLE_WEIGHT]);
	pr_debug("%s: set lhbm histogram config (x=%d, y=%d, w=%d, h=%d)\n", crtc->name, x, y, w, h);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_drm_drv_set_lhbm_hist);

#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
int exynos_drm_drv_set_lhbm_hist_gs(struct decon_device *decon, struct histogram_roi *roi,
				    struct histogram_weights *weight)
{
	struct histogram_channel_config *cfg;

	if (!decon || !decon->dqe)
		return -EIO;
	cfg = &decon->dqe->lhbm_hist_config;

	exynos_drm_drv_set_lhbm_hist_helper(cfg, roi, weight);
	pr_debug(
		"%s: set lhbm histogram config (x=%d, y=%d, w=%d, h=%d weight(R=%d, B=%d, B=%d))\n",
		decon->crtc->base.name, roi->start_x, roi->start_y, roi->hsize, roi->vsize,
		weight->weight_r, weight->weight_g, weight->weight_b);
	decon->dqe->lhbm_hist_configured = true;

	return 0;
}
#endif

int exynos_drm_drv_get_lhbm_gray_level(struct exynos_drm_connector *conn)
{
	struct drm_crtc *crtc = NULL;
	struct exynos_drm_crtc *exynos_crtc;
	struct decon_device *decon;

	if (conn->base.state)
		crtc = conn->base.state->crtc;
	if (!crtc)
		return 0;

	exynos_crtc = to_exynos_crtc(crtc);
	decon = (struct decon_device *)exynos_crtc->ctx;
	if (!decon || !decon->dqe)
		return 0;
	return decon->dqe->lhbm_gray_level;
}
EXPORT_SYMBOL_GPL(exynos_drm_drv_get_lhbm_gray_level);

static struct exynos_drm_priv_state *exynos_drm_get_priv_state(struct drm_atomic_state *state)
{
	struct exynos_drm_private *priv = drm_to_exynos_dev(state->dev);

	struct drm_private_state *priv_state;

	priv_state = drm_atomic_get_private_obj_state(state, &priv->obj);
	if (IS_ERR(priv_state))
		return ERR_CAST(priv_state);

	return to_exynos_priv_state(priv_state);
}

static unsigned int find_set_bits_mask(unsigned int mask, size_t count)
{
	unsigned int out = 0;
	int i;

	for (i = count; i > 0; i--) {
		int bit = ffs(mask) - 1;

		if (bit < 0)
			return 0;

		mask &= ~BIT(bit);
		out |= BIT(bit);
	}

	return out;
}

static unsigned int exynos_drm_crtc_get_win_cnt(struct drm_crtc_state *crtc_state)
{
	unsigned int num_planes;
	const struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc_state->crtc);

	if (!crtc_state->enable || !drm_atomic_crtc_effectively_active(crtc_state))
		return 0;

	num_planes = hweight32(crtc_state->plane_mask & ~exynos_crtc->rcd_plane_mask);

	/* at least one window is required for color map when active and there are no planes */
	return num_planes ? : 1;
}

static int exynos_atomic_check_windows(struct drm_device *dev, struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	struct exynos_drm_priv_state *exynos_priv_state;
	unsigned int freed_win_mask = 0;
	unsigned int win_mask;
	int i;

	for_each_oldnew_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		struct exynos_drm_crtc_state *new_exynos_crtc_state;
		unsigned int old_win_cnt, new_win_cnt;

		new_exynos_crtc_state = to_exynos_crtc_state(new_crtc_state);

		if (!new_crtc_state->active_changed && !new_crtc_state->zpos_changed &&
		    !new_crtc_state->connectors_changed &&
		    (old_crtc_state->plane_mask == new_crtc_state->plane_mask))
			continue;

		old_win_cnt = exynos_drm_crtc_get_win_cnt(old_crtc_state);
		new_win_cnt = exynos_drm_crtc_get_win_cnt(new_crtc_state);

		if (old_win_cnt == new_win_cnt)
			continue;

		pr_debug("%s: win cnt changed: %d -> %d\n", crtc->name, old_win_cnt, new_win_cnt);

		if (old_win_cnt > new_win_cnt) {
			const unsigned int curr_win_mask = new_exynos_crtc_state->reserved_win_mask;

			if (new_win_cnt)
				win_mask = find_set_bits_mask(curr_win_mask,
							      old_win_cnt - new_win_cnt);
			else /* freeing all windows */
				win_mask = curr_win_mask;

			WARN(!win_mask, "%s: invalid reserved mask (0x%x) for win_cnt (%d->%d)",
			     crtc->name, curr_win_mask, old_win_cnt, new_win_cnt);

			/*
			 * we don't want to immediately put the windows back into available_win_mask
			 * for next crtc to use since the windows will get freed only after commit
			 * to hw is done for this commit
			 */
			freed_win_mask |= win_mask;
			new_exynos_crtc_state->reserved_win_mask &= ~win_mask;
		} else {
			exynos_priv_state = exynos_drm_get_priv_state(state);
			if (IS_ERR(exynos_priv_state))
				return PTR_ERR(exynos_priv_state);

			win_mask = find_set_bits_mask(exynos_priv_state->available_win_mask,
						      new_win_cnt - old_win_cnt);
			if (!win_mask) {
				DRM_WARN("%s: No windows available for req win cnt=%d->%d (0x%x)\n",
					 crtc->name, old_win_cnt, new_win_cnt,
					 exynos_priv_state->available_win_mask);
				return -ENOENT;
			}
			pr_debug("%s: current win_mask=0x%x new=0x%x avail=0x%x\n", crtc->name,
				 new_exynos_crtc_state->reserved_win_mask, win_mask,
				 exynos_priv_state->available_win_mask);

			exynos_priv_state->available_win_mask &= ~win_mask;
			new_exynos_crtc_state->reserved_win_mask |= win_mask;
		}
	}

	for_each_new_crtc_in_state(state, crtc, new_crtc_state, i) {
		struct exynos_drm_crtc_state *new_exynos_crtc_state =
			to_exynos_crtc_state(new_crtc_state);
		const unsigned long reserved_win_mask =
			new_exynos_crtc_state->reserved_win_mask;
		unsigned long visible_zpos = 0;
		struct drm_plane *plane;
		const struct drm_plane_state *plane_state;
		unsigned int pmask, bit;

		new_exynos_crtc_state->visible_win_mask = 0;

		if (!new_crtc_state->plane_mask)
			continue;

		drm_atomic_crtc_state_for_each_plane_state(plane, plane_state,
				new_crtc_state)
			if (plane_state->visible)
				visible_zpos |= BIT(plane_state->normalized_zpos);

		pmask = 1;
		for_each_set_bit(bit, &reserved_win_mask, MAX_WIN_PER_DECON) {
			if (pmask & visible_zpos)
				new_exynos_crtc_state->visible_win_mask |= BIT(bit);
			pmask <<= 1;
		}
		pr_debug("%s: visible_win_mask(0x%x)\n", __func__,
				new_exynos_crtc_state->visible_win_mask);
	}

	if (freed_win_mask) {
		exynos_priv_state = exynos_drm_get_priv_state(state);
		if (IS_ERR(exynos_priv_state))
			return PTR_ERR(exynos_priv_state);

		pr_debug("%s: avail_win=0x%x freed_mask=0x%x\n", __func__,
			 exynos_priv_state->available_win_mask, freed_win_mask);

		/*
		 * FIXME: these windows will be made available for next atomic commit as soon as
		 * atomic state is swapped. This can lead to a (very rare) race condition if commit
		 * to hw hasn't finished before an atomic commit comes in for a different crtc.
		 */
		exynos_priv_state->available_win_mask |= freed_win_mask;
	}

	return 0;
}

static void exynos_atomic_prepare_partial_update(struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	struct exynos_drm_crtc_state *old_exynos_crtc_state, *new_exynos_crtc_state;
	struct decon_device *decon;
	struct exynos_partial *partial;
	int i;

	for_each_oldnew_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		decon = to_exynos_crtc(crtc)->ctx;

		if (!new_crtc_state->active)
			continue;

		if (drm_atomic_crtc_needs_modeset(new_crtc_state)) {
			const struct drm_connector_state *conn_state =
				crtc_get_phys_connector_state(state, new_crtc_state);
			if (conn_state && is_exynos_drm_connector(conn_state->connector)) {
				const struct exynos_display_partial *p =
					&to_exynos_connector_state(conn_state)->partial;

				decon->partial = exynos_partial_initialize(decon,
						p, &new_crtc_state->mode);
			}
#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
			else if (conn_state && is_gs_drm_connector(conn_state->connector)) {
				const struct gs_drm_connector_state *gs_conn_state =
					to_gs_connector_state(conn_state);
				const struct exynos_display_partial p = {
					.enabled = gs_conn_state->partial.enabled,
					.min_width = gs_conn_state->partial.min_width,
					.min_height = gs_conn_state->partial.min_height,
				};

				decon->partial =
					exynos_partial_initialize(decon, &p, &new_crtc_state->mode);
			}
#endif
		}

		partial = decon->partial;
		if (!partial)
			continue;

		new_exynos_crtc_state = to_exynos_crtc_state(new_crtc_state);
		old_exynos_crtc_state = to_exynos_crtc_state(old_crtc_state);

		exynos_partial_prepare(partial, old_exynos_crtc_state,
						new_exynos_crtc_state);
	}
}

static void exynos_check_updated_planes(struct drm_device *dev, struct drm_atomic_state *state)
{
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	int i;

	for_each_new_plane_in_state(state, plane, plane_state, i) {
		struct drm_crtc_state *crtc_state;
		struct exynos_drm_crtc_state *exynos_crtc_state;

		if (plane_state->crtc) {
			crtc_state = drm_atomic_get_new_crtc_state(state, plane_state->crtc);

			if (WARN_ON(!crtc_state))
				return;

			exynos_crtc_state = to_exynos_crtc_state(crtc_state);
			exynos_crtc_state->planes_updated = true;
		}
	}
}

static int exynos_add_relevant_connectors(struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	struct drm_connector_list_iter conn_iter;
	u32 connector_mask = 0;
	int ret = 0;
	int i;

	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		if (!crtc_state->active)
			continue;

		connector_mask |= crtc_state->connector_mask;
	}

	drm_connector_list_iter_begin(state->dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		if (!(connector_mask & drm_connector_mask(connector)))
			continue;

		if (is_exynos_drm_connector(connector)) {
			const struct exynos_drm_connector *exynos_connector =
					to_exynos_connector(connector);

			if (!exynos_connector->needs_commit)
				continue;

			conn_state = drm_atomic_get_connector_state(state, connector);
			if (IS_ERR(conn_state)) {
				ret = PTR_ERR(conn_state);
				break;
			}
		}
#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
		else if (is_gs_drm_connector(connector)) {
			const struct gs_drm_connector *gs_connector =
					to_gs_connector(connector);

			if (!gs_connector->needs_commit)
				continue;

			conn_state = drm_atomic_get_connector_state(state, connector);
			if (IS_ERR(conn_state)) {
				ret = PTR_ERR(conn_state);
				break;
			}
		}
#endif
		else
			continue;

	}
	drm_connector_list_iter_end(&conn_iter);

	return ret;
}

int exynos_atomic_check(struct drm_device *dev,
			struct drm_atomic_state *state)
{
	const struct exynos_drm_private *private = drm_to_exynos_dev(dev);
	int ret;

	if (private->tui_enabled) {
		pr_info("tui enabled reject commit(%pK)\n", state);
		return -EPERM;
	}

	exynos_check_updated_planes(dev, state);

	ret = exynos_add_relevant_connectors(state);
	if (ret)
		return ret;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	exynos_atomic_prepare_partial_update(state);

	ret = drm_atomic_normalize_zpos(dev, state);
	if (ret)
		return ret;

	ret = drm_atomic_helper_check_planes(dev, state);
	if (ret)
		return ret;

	ret = exynos_atomic_check_windows(dev, state);
	if (ret)
		return ret;

	drm_self_refresh_helper_alter_state(state);

	return 0;
}

static struct drm_private_state *exynos_atomic_duplicate_priv_state(struct drm_private_obj *obj)
{
	const struct exynos_drm_priv_state *old_state = to_exynos_priv_state(obj->state);
	struct exynos_drm_priv_state *new_state;

	new_state = kmemdup(old_state, sizeof(*old_state), GFP_KERNEL);
	if (!new_state)
		return NULL;

	__drm_atomic_helper_private_obj_duplicate_state(obj, &new_state->base);

	return &new_state->base;
}

static void exynos_atomic_destroy_priv_state(struct drm_private_obj *obj,
					     struct drm_private_state *state)
{
	struct exynos_drm_priv_state *priv_state = to_exynos_priv_state(state);

	kfree(priv_state);
}

static const struct drm_private_state_funcs exynos_priv_state_funcs = {
	.atomic_duplicate_state = exynos_atomic_duplicate_priv_state,
	.atomic_destroy_state = exynos_atomic_destroy_priv_state,
};

static void print_drm_plane_state_info(struct drm_printer *p,
	struct drm_plane_state *state)
{
	drm_printf(p, "plane: rotation=%x zpos=%x alpha=%x blend_mode=%x\n",
		state->rotation, state->normalized_zpos, state->alpha,
		state->pixel_blend_mode);
	drm_printf(p, "plane: crtc-pos=" DRM_RECT_FMT "\n", DRM_RECT_ARG(&state->dst));
	drm_printf(p, "plane: src-pos=" DRM_RECT_FP_FMT "\n", DRM_RECT_FP_ARG(&state->src));
	drm_printf(p, "plane: fb allocated by = %s\n", state->fb->comm);
}

static int exynos_atomic_helper_wait_for_fences(struct drm_device *dev,
				      struct drm_atomic_state *state,
				      bool pre_swap)
{
	struct drm_plane *plane;
	struct drm_plane_state *new_plane_state;
	int i, ret, err = 0;
	struct drm_printer p = drm_info_printer(dev->dev);
	long tmo = msecs_to_jiffies(EXYNOS_DRM_WAIT_FENCE_TIMEOUT_MS);

	for_each_new_plane_in_state(state, plane, new_plane_state, i) {
		struct dma_fence *fence = new_plane_state->fence;

		if (!fence)
			continue;

		WARN_ON(!new_plane_state->fb);
		ret = dma_fence_wait_timeout(fence, pre_swap, tmo);
		if (ret == 0) {
			struct drm_crtc *crtc = new_plane_state->crtc;

			pr_err("%s: timeout of waiting for fence, name:%s idx:%d\n",
				__func__, plane->name ? : "NA", plane->index);
			if (crtc) {
				const struct decon_device *decon = crtc_to_decon(crtc);
				const struct decon_config *cfg = &decon->config;
				struct exynos_drm_crtc_state *new_exynos_crtc_state;
				struct drm_crtc_state *new_crtc_state;

				new_crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
				if (!new_crtc_state ||
					!new_crtc_state->enable || !new_crtc_state->active)
					continue;
				new_exynos_crtc_state = to_exynos_crtc_state(new_crtc_state);
				if (!new_exynos_crtc_state->skip_update &&
					   cfg->mode.op_mode != DECON_VIDEO_MODE) {
					new_exynos_crtc_state->skip_update = true;
					pr_warn("%s: skip frame update at %s\n",
									__func__, crtc->name);
				}
			}
			print_drm_plane_state_info(&p, new_plane_state);

			spin_lock_irq(fence->lock);
			drm_printf(&p, "fence: %s-%s %llu-%llu status:%s\n",
				fence->ops ? fence->ops->get_driver_name(fence) : "none",
				fence->ops ? fence->ops->get_timeline_name(fence) : "none",
				fence->context, fence->seqno,
				dma_fence_get_status_locked(fence) < 0 ? "error" : "active");
			if (test_bit(DMA_FENCE_FLAG_TIMESTAMP_BIT, &fence->flags)) {
				struct timespec64 ts64 = ktime_to_timespec64(fence->timestamp);
				drm_printf(&p, "fence: timestamp:%lld.%09ld\n",
					(s64)ts64.tv_sec, ts64.tv_nsec);
			}
			if (fence->error)
				drm_printf(&p, "fence: err=%d\n", fence->error);
			spin_unlock_irq(fence->lock);

			tmo = 0;
			err = -ETIMEDOUT;
		} else if (ret < 0) {
			pr_warn("%s: error of waiting for dma fence, ret=%d\n", __func__, ret);
			print_drm_plane_state_info(&p, new_plane_state);
			return ret;
		}
		dma_fence_put(fence);
		new_plane_state->fence = NULL;
	}

	return err;
}

static void commit_tail(struct drm_atomic_state *old_state)
{
	int i;
	const struct drm_mode_config_helper_funcs *funcs;
	struct decon_device *decon;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	struct drm_device *dev = old_state->dev;
	unsigned int hibernation_crtc_mask = 0;

	funcs = dev->mode_config.helper_private;

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		decon = crtc_to_decon(crtc);
		if (new_crtc_state->active || old_crtc_state->active) {
			hibernation_block(decon->hibernation);

			hibernation_crtc_mask |= drm_crtc_mask(crtc);
		}
	}

	DPU_ATRACE_BEGIN("wait_for_fences");
	exynos_atomic_helper_wait_for_fences(dev, old_state, false);
	DPU_ATRACE_END("wait_for_fences");

	drm_atomic_helper_wait_for_dependencies(old_state);

	if (funcs && funcs->atomic_commit_tail)
		funcs->atomic_commit_tail(old_state);
	else
		drm_atomic_helper_commit_tail(old_state);

	for_each_new_crtc_in_state(old_state, crtc, new_crtc_state, i) {
		decon = crtc_to_decon(crtc);
		if (hibernation_crtc_mask & drm_crtc_mask(crtc))
			hibernation_unblock_enter(decon->hibernation);
	}

	drm_atomic_helper_commit_cleanup_done(old_state);

	drm_atomic_state_put(old_state);
}

static void commit_kthread_work(struct kthread_work *work)
{
	struct exynos_drm_crtc_state *old_exynos_crtc_state =
		container_of(work, struct exynos_drm_crtc_state, commit_work);
	struct drm_atomic_state *old_state = old_exynos_crtc_state->base.state;

	BUG_ON(!old_state);
	commit_tail(old_state);
}

static void commit_work(struct work_struct *work)
{
	struct drm_atomic_state *old_state =
		container_of(work, struct drm_atomic_state, commit_work);

	commit_tail(old_state);
}

static void exynos_atomic_queue_work(struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	int i;

	/*
	 * queuing to first decon worker in atomic commit even if there are
	 * multiple displays updated within same commit
	 *
	 * TODO: can work be split per display?
	 */
	for_each_old_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		struct decon_device *decon = crtc_to_decon(crtc);
		struct kthread_work *work = &to_exynos_crtc_state(old_crtc_state)->commit_work;

		if (decon->hibernation && old_crtc_state->active) {
			hibernation_block(decon->hibernation);
			hibernation_unblock_enter(decon->hibernation);
		}

		kthread_init_work(work, commit_kthread_work);
		kthread_queue_work(&decon->worker, work);

		return;
	}

	/* fallback to regular commit work if we get here (no crtcs in commit state) */
	INIT_WORK(&old_state->commit_work, commit_work);
	queue_work(system_highpri_wq, &old_state->commit_work);
}

int exynos_atomic_commit(struct drm_device *dev, struct drm_atomic_state *state, bool nonblock)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	int i, ret;
	bool stall = !nonblock;

	DPU_ATRACE_BEGIN("exynos_atomic_commit");

	/*
	 * if self refresh was activated on last commit or coming out of self refresh/hibernation,
	 * it's okay to stall instead of failing since commit should finish rather quickly
	 */
	if (!stall) {
		const struct exynos_drm_crtc_state *old_exynos_crtc_state;

		for_each_old_crtc_in_state(state, crtc, old_crtc_state, i) {
			old_exynos_crtc_state = to_exynos_crtc_state(old_crtc_state);

			if (old_crtc_state->self_refresh_active ||
			    old_exynos_crtc_state->hibernation_exit) {
				stall = true;
				break;
			}
		}
	}

	ret = drm_atomic_helper_setup_commit(state, !stall);
	if (ret)
		goto err;

	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret)
		goto err;

	/*
	 * This is the point of no return - everything below never fails except
	 * when the hw goes bonghits. Which means we can commit the new state on
	 * the software side now.
	 */

	ret = drm_atomic_helper_swap_state(state, true);
	if (ret) {
		drm_atomic_helper_cleanup_planes(dev, state);
		goto err;
	}

	/*
	 * Everything below can be run asynchronously without the need to grab
	 * any modeset locks at all under one condition: It must be guaranteed
	 * that the asynchronous work has either been cancelled (if the driver
	 * supports it, which at least requires that the framebuffers get
	 * cleaned up with drm_atomic_helper_cleanup_planes()) or completed
	 * before the new state gets committed on the software side with
	 * drm_atomic_helper_swap_state().
	 *
	 * This scheme allows new atomic state updates to be prepared and
	 * checked in parallel to the asynchronous completion of the previous
	 * update. Which is important since compositors need to figure out the
	 * composition of the next frame right after having submitted the
	 * current layout.
	 *
	 * NOTE: Commit work has multiple phases, first hardware commit, then
	 * cleanup. We want them to overlap, hence need system_unbound_wq to
	 * make sure work items don't artificially stall on each another.
	 */

	drm_atomic_state_get(state);
	if (!nonblock)
		commit_tail(state);
	else
		exynos_atomic_queue_work(state);

err:
	DPU_ATRACE_END("exynos_atomic_commit");

	return ret;
}

static ssize_t tui_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	const struct exynos_drm_private *private = drm_to_exynos_dev(drm_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", private->tui_enabled ? 1 : 0);
}
static DEVICE_ATTR_RO(tui_status);

int exynos_atomic_enter_tui(void)
{
	int i, ret = 0;
	struct decon_device *decon = get_decon_drvdata(0);
	struct drm_device *dev = decon->drm_dev;
	struct drm_atomic_state *state;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_plane_state *plane_state;
	struct drm_plane *plane;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	struct drm_crtc *crtc;
	u32 tui_crtc_mask = 0;
	struct exynos_drm_private *private = drm_to_exynos_dev(dev);

	pr_debug("%s +\n", __func__);

	if (private->tui_enabled)
		return -EBUSY;

	if (mutex_trylock(&private->dp_tui_lock) == 0) {
		/* DP connection is active, bail out */
		pr_info("%s: unable to enter TUI, DP is active\n", __func__);
		return -EBUSY;
	}

	drm_for_each_crtc(crtc, dev) {
		decon = crtc_to_decon(crtc);
		hibernation_block_exit(decon->hibernation);
	}

	DRM_MODESET_LOCK_ALL_BEGIN(dev, ctx, 0, ret);

	state = drm_atomic_helper_duplicate_state(dev, &ctx);
	if (IS_ERR(state)) {
		ret = -ENOMEM;
		goto err_dup;
	}

	mode_config->suspend_state = state;

	state = drm_atomic_state_alloc(dev);
	if (!state) {
		ret = -ENOMEM;
		goto err_state_alloc;
	}

	state->acquire_ctx = &ctx;

	drm_for_each_crtc(crtc, dev) {
		crtc_state = drm_atomic_get_crtc_state(state, crtc);
		if (IS_ERR(crtc_state)) {
			ret = PTR_ERR(crtc_state);
			goto err;
		}

		if (!crtc_state->enable || !crtc_state->active)
			continue;

		decon = crtc_to_decon(crtc);
		/* get an extra ref count while in TUI, to keep power domain active */
		pm_runtime_get_sync(decon->dev);

		crtc_state->active = false;
		tui_crtc_mask |= drm_crtc_mask(crtc);

		ret = drm_atomic_add_affected_planes(state, crtc);
		if (ret < 0)
			goto err;

		ret = drm_atomic_add_affected_connectors(state, crtc);
		if (ret)
			goto err;

		conn_state = crtc_get_phys_connector_state(state, crtc_state);

		if (conn_state) {
			if (conn_state->self_refresh_aware) {
				struct exynos_drm_crtc_state *exynos_crtc_state =
					to_exynos_crtc_state(crtc_state);

				exynos_crtc_state->bypass = true;
				crtc_state->self_refresh_active = true;
			}

			if (is_exynos_drm_connector(conn_state->connector)) {
				struct exynos_drm_connector_state *exynos_conn_state =
					to_exynos_connector_state(conn_state);

				exynos_conn_state->blanked_mode = true;
			}
#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
			else if (is_gs_drm_connector(conn_state->connector)) {
				struct gs_drm_connector_state *gs_conn_state =
					to_gs_connector_state(conn_state);

				gs_conn_state->blanked_mode = true;
			}
#endif
		}
	}

	if (!tui_crtc_mask) {
		pr_err("%s:unable to enter tui without any active crtcs\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	for_each_new_plane_in_state(state, plane, plane_state, i) {
		ret = drm_atomic_set_crtc_for_plane(plane_state, NULL);
		if (ret < 0)
			goto err;

		drm_atomic_set_fb_for_plane(plane_state, NULL);
	}

	ret = drm_atomic_commit(state);
	if (!ret) {
		private->tui_enabled = true;
		/* send TUI status changing event to userspace */
		sysfs_notify(&dev->dev->kobj, NULL, "tui_status");
	}

err:
	drm_atomic_state_put(state);

err_state_alloc:
	if (ret) {
		drm_atomic_state_put(mode_config->suspend_state);
		mode_config->suspend_state = NULL;
	}
err_dup:
	if (ret)
		mutex_unlock(&private->dp_tui_lock);
	DRM_MODESET_LOCK_ALL_END(dev, ctx, ret);
	pr_debug("%s -\n", __func__);

	drm_for_each_crtc(crtc, dev) {
		decon = crtc_to_decon(crtc);
		hibernation_unblock_enter(decon->hibernation);

		/* remove pm refs on failure */
		if (ret && (tui_crtc_mask & drm_crtc_mask(crtc)))
			pm_runtime_put_sync(decon->dev);
	}

	return ret;
}

int exynos_atomic_exit_tui(void)
{
	int ret;
	struct decon_device *decon = get_decon_drvdata(0);
	struct drm_device *dev = decon->drm_dev;
	struct drm_atomic_state *state;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct drm_modeset_acquire_ctx ctx;
	struct exynos_drm_private *private = drm_to_exynos_dev(dev);
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	int i;

	pr_debug("%s +\n", __func__);

	if (!private->tui_enabled) {
		pr_err("%s: not in tui\n", __func__);
		return -EINVAL;
	}

	state = mode_config->suspend_state;
	if (!state) {
		pr_err("%s: there is no suspend_state\n", __func__);
		return -EINVAL;
	}

	DRM_MODESET_LOCK_ALL_BEGIN(dev, ctx, 0, ret);

	private->tui_enabled = false;
	/* send TUI status changing event to userspace */
	sysfs_notify(&dev->dev->kobj, NULL, "tui_status");

	ret = drm_atomic_helper_commit_duplicated_state(state, &ctx);
	if (ret < 0)
		pr_err("%s: failed to atomic commit suspend_state(0x%x)\n", __func__, ret);
	else
		mode_config->suspend_state = NULL;

	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		if (crtc_state->active) {
			decon = crtc_to_decon(crtc);
			/* drop the ref taken during enter tui */
			pm_runtime_put_sync(decon->dev);
		}
	}

	DRM_MODESET_LOCK_ALL_END(dev, ctx, ret);
	if (!ret)
		drm_atomic_state_put(state);
	mutex_unlock(&private->dp_tui_lock);

	pr_debug("%s -\n", __func__);
	return ret;
}

static int exynos_drm_open(struct drm_device *dev, struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv;

	file_priv = kzalloc(sizeof(*file_priv), GFP_KERNEL);
	if (!file_priv)
		return -ENOMEM;

	file->driver_priv = file_priv;

	return 0;
}

static void exynos_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
	kfree(file->driver_priv);
	file->driver_priv = NULL;
}


static const struct drm_ioctl_desc exynos_ioctls[] = {
	DRM_IOCTL_DEF_DRV(EXYNOS_HISTOGRAM_REQUEST, histogram_request_ioctl, 0),
	DRM_IOCTL_DEF_DRV(EXYNOS_HISTOGRAM_CANCEL, histogram_cancel_ioctl, 0),
	DRM_IOCTL_DEF_DRV(EXYNOS_HISTOGRAM_CHANNEL_REQUEST, histogram_channel_request_ioctl, 0),
	DRM_IOCTL_DEF_DRV(EXYNOS_HISTOGRAM_CHANNEL_CANCEL, histogram_channel_cancel_ioctl, 0),
	DRM_IOCTL_DEF_DRV(EXYNOS_CONTEXT_HISTOGRAM_EVENT_REQUEST, histogram_event_request_ioctl, 0),
	DRM_IOCTL_DEF_DRV(EXYNOS_CONTEXT_HISTOGRAM_EVENT_CANCEL, histogram_event_cancel_ioctl, 0),
};

static const struct file_operations exynos_drm_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= exynos_drm_gem_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
	.compat_ioctl	= drm_compat_ioctl,
	.release	= drm_release,
};

static struct drm_driver exynos_drm_driver = {
	.driver_features	   = DRIVER_MODESET | DRIVER_ATOMIC |
				     DRIVER_RENDER | DRIVER_GEM,
	.open			   = exynos_drm_open,
	.postclose		   = exynos_drm_postclose,
	.dumb_create		   = exynos_drm_gem_dumb_create,
	.dumb_map_offset	   = exynos_drm_gem_dumb_map_offset,
	.prime_handle_to_fd	   = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	   = drm_gem_prime_fd_to_handle,
	.gem_prime_import	   = exynos_drm_gem_prime_import,
	.gem_prime_import_sg_table = exynos_drm_gem_prime_import_sg_table,
	.ioctls			   = exynos_ioctls,
	.num_ioctls		   = ARRAY_SIZE(exynos_ioctls),
	.fops			   = &exynos_drm_driver_fops,
	.name			   = DRIVER_NAME,
	.desc			   = DRIVER_DESC,
	.date			   = DRIVER_DATE,
	.major			   = DRIVER_MAJOR,
	.minor			   = DRIVER_MINOR,
};

/* forward declaration */
static struct platform_driver exynos_drm_platform_driver;

struct exynos_drm_driver_info {
	struct platform_driver *driver;
	unsigned int flags;
};

#define DRM_COMPONENT_DRIVER	BIT(0)	/* supports component framework */
#define DRM_VIRTUAL_DEVICE	BIT(1)	/* create virtual platform device */
#define DRM_DMA_DEVICE		BIT(2)	/* can be used for dma allocations */

#define DRV_PTR(drv, cond) (IS_ENABLED(cond) ? &drv : NULL)

/*
 * Connector drivers should not be placed before associated crtc drivers,
 * because connector requires pipe number of its crtc during initialization.
 */
static struct exynos_drm_driver_info exynos_drm_drivers[] = {
	{
		DRV_PTR(dpp_driver, CONFIG_DRM_SAMSUNG_DPP),
		DRM_COMPONENT_DRIVER
	}, {
		/* writeback connector should be bound before bts init. */
		DRV_PTR(writeback_driver, CONFIG_DRM_SAMSUNG_WB),
		DRM_COMPONENT_DRIVER
	}, {
		DRV_PTR(decon_driver, CONFIG_DRM_SAMSUNG_DECON),
		DRM_COMPONENT_DRIVER | DRM_DMA_DEVICE
	}, {
		DRV_PTR(dsim_driver, CONFIG_DRM_SAMSUNG_DSI),
		DRM_COMPONENT_DRIVER
	},
#if IS_ENABLED(CONFIG_DRM_SAMSUNG_DP)
	{
		DRV_PTR(dp_driver, CONFIG_DRM_SAMSUNG_DP),
		DRM_COMPONENT_DRIVER
	},
#endif
	{
		DRV_PTR(tui_driver, CONFIG_DRM_SAMSUNG_TUI),
		DRM_VIRTUAL_DEVICE
	}, {
		&exynos_drm_platform_driver,
		DRM_VIRTUAL_DEVICE
	}
};

static int compare_dev(struct device *dev, void *data)
{
	return dev == (struct device *)data;
}

static struct component_match *exynos_drm_match_add(struct device *dev)
{
	struct component_match *match = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(exynos_drm_drivers); ++i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];
		struct device *p = NULL, *d;

		if (!info->driver || !(info->flags & DRM_COMPONENT_DRIVER))
			continue;

		while ((d = platform_find_device_by_driver(p,
						&info->driver->driver))) {
			put_device(p);
			component_match_add(dev, &match, compare_dev, d);
			p = d;
		}
		put_device(p);
	}

	return match ?: ERR_PTR(-ENODEV);
}

static int exynos_drm_bind(struct device *dev)
{
	struct exynos_drm_private *private;
	struct drm_encoder *encoder;
	struct drm_device *drm;
	struct exynos_drm_priv_state *priv_state;
	u32 wb_mask = 0;
	u32 encoder_mask = 0;
	int ret;

	private = devm_drm_dev_alloc(dev, &exynos_drm_driver, struct exynos_drm_private, drm);
	if (IS_ERR(private)) {
		dev_err(dev, "[" DRM_NAME ":%s] devm_drm_dev_alloc failed: %li\n",
			__func__, PTR_ERR(private));
		return PTR_ERR(private);
	}

	drm = &private->drm;

	init_waitqueue_head(&private->wait);
	spin_lock_init(&private->lock);

	private->tui_enabled = false;
	mutex_init(&private->dp_tui_lock);

	dev_set_drvdata(dev, drm);

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	exynos_drm_mode_config_init(drm);

	/* create properties ahead of binding to make them available to all drivers */
	exynos_drm_connector_create_properties(drm);

	priv_state = kzalloc(sizeof(*priv_state), GFP_KERNEL);
	if (!priv_state)
		return -ENOMEM;

	priv_state->available_win_mask = BIT(MAX_WIN_PER_DECON) - 1;

	drm_atomic_private_obj_init(drm, &private->obj, &priv_state->base,
				    &exynos_priv_state_funcs);

	/* Try to bind all sub drivers. */
	ret = component_bind_all(dev, drm);
	if (ret)
		goto err_priv_state_cleanup;

	drm_for_each_encoder(encoder, drm) {
		if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL)
			wb_mask |= drm_encoder_mask(encoder);

		encoder_mask |= drm_encoder_mask(encoder);
	}

	drm_for_each_encoder(encoder, drm) {
		if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL) {
			struct writeback_device *wb = enc_to_wb_dev(encoder);

			encoder->possible_clones = encoder_mask;
			encoder->possible_crtcs =
				exynos_drm_get_possible_crtcs(encoder, wb->output_type);
		} else {
			encoder->possible_clones = drm_encoder_mask(encoder) |
				wb_mask;
		}

		pr_info("encoder[%d] type(0x%x) possible_clones(0x%x)\n",
				encoder->index, encoder->encoder_type,
				encoder->possible_clones);
	}

	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret)
		goto err_unbind_all;

	drm_mode_config_reset(drm);

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = true, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *	just specific driver own one instead because
	 *	drm framework supports only one irq handler.
	 */
#if IS_ENABLED(CONFIG_DRM_LEGACY)
	drm->irq_enabled = true;
#endif

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(drm);

	/* register the DRM device */
	ret = drm_dev_register(drm, 0);
	if (ret < 0)
		goto err_cleanup_poll;

	/* create sysfs node for TUI status */
	device_create_file(dev, &dev_attr_tui_status);

	return 0;

err_cleanup_poll:
	drm_kms_helper_poll_fini(drm);
err_unbind_all:
	component_unbind_all(dev, drm);
err_priv_state_cleanup:
	drm_atomic_private_obj_fini(&private->obj);

	return ret;
}

static void exynos_drm_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct exynos_drm_private *private = drm_to_exynos_dev(drm);

	/* destroy sysfs node for TUI status */
	device_remove_file(dev, &dev_attr_tui_status);

	drm_dev_unregister(drm);

	drm_atomic_private_obj_fini(&private->obj);

	drm_kms_helper_poll_fini(drm);

	component_unbind_all(dev, drm);

	drm_dev_put(drm);

	dev_set_drvdata(dev, NULL);
}

static const struct component_master_ops exynos_drm_ops = {
	.bind		= exynos_drm_bind,
	.unbind		= exynos_drm_unbind,
};

static int exynos_drm_platform_probe(struct platform_device *pdev)
{
	struct component_match *match;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	platform_set_drvdata(pdev, NULL);

	match = exynos_drm_match_add(&pdev->dev);
	if (IS_ERR(match))
		return PTR_ERR(match);

	return component_master_add_with_match(&pdev->dev, &exynos_drm_ops,
					       match);
}

static int exynos_drm_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &exynos_drm_ops);
	return 0;
}

static void exynos_drm_platform_shutdown(struct platform_device *pdev)
{
	struct drm_device *drm_dev = platform_get_drvdata(pdev);

	if (drm_dev)
		drm_atomic_helper_shutdown(drm_dev);
}

static struct platform_driver exynos_drm_platform_driver = {
	.driver	= {
		.name	= "exynos-drm",
	},
	.probe	= exynos_drm_platform_probe,
	.remove	= exynos_drm_platform_remove,
	.shutdown = exynos_drm_platform_shutdown,
};

static void exynos_drm_unregister_devices(void)
{
	int i;

	for (i = ARRAY_SIZE(exynos_drm_drivers) - 1; i >= 0; --i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];
		struct device *dev;

		if (!info->driver || !(info->flags & DRM_VIRTUAL_DEVICE))
			continue;

		while ((dev = platform_find_device_by_driver(NULL,
						&info->driver->driver))) {
			put_device(dev);
			platform_device_unregister(to_platform_device(dev));
		}
	}
}

static int exynos_drm_register_devices(void)
{
	struct platform_device *pdev;
	int i;

	for (i = 0; i < ARRAY_SIZE(exynos_drm_drivers); ++i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];

		if (!info->driver || !(info->flags & DRM_VIRTUAL_DEVICE))
			continue;

		pdev = platform_device_register_simple(
					info->driver->driver.name, -1, NULL, 0);
		if (IS_ERR(pdev))
			goto fail;
	}

	return 0;
fail:
	exynos_drm_unregister_devices();
	return PTR_ERR(pdev);
}

static void exynos_drm_unregister_drivers(void)
{
	int i;

	for (i = ARRAY_SIZE(exynos_drm_drivers) - 1; i >= 0; --i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];

		if (!info->driver)
			continue;

		platform_driver_unregister(info->driver);
	}
}

static int exynos_drm_register_drivers(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(exynos_drm_drivers); ++i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];

		if (!info->driver)
			continue;

		ret = platform_driver_register(info->driver);
		if (ret)
			goto fail;
	}
	return 0;
fail:
	exynos_drm_unregister_drivers();
	return ret;
}

static int exynos_drm_init(void)
{
	int ret;

	ret = exynos_drm_register_devices();
	if (ret)
		return ret;

	ret = exynos_drm_register_drivers();
	if (ret)
		goto err_unregister_pdevs;

	return 0;

err_unregister_pdevs:
	exynos_drm_unregister_devices();

	return ret;
}

static void exynos_drm_exit(void)
{
	exynos_drm_unregister_drivers();
	exynos_drm_unregister_devices();
}

module_init(exynos_drm_init);
module_exit(exynos_drm_exit);

MODULE_AUTHOR("Inki Dae <inki.dae@samsung.com>");
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_AUTHOR("Seung-Woo Kim <sw0312.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC DRM Driver");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_LICENSE("GPL");
