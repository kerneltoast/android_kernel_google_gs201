// SPDX-License-Identifier: MIT
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <uapi/linux/sched/types.h>
#include <drm/drm_vblank.h>

#include "gs_panel/gs_panel.h"
#include "gs_panel_internal.h"
#include "trace/panel_trace.h"

/**
 * panel_update_lhbm_notimeout - Updates lhbm state to match requested state
 * @ctx: panel struct
 *
 * Context: Expects ctx->mode_lock to be locked
 *
 * Return: true on success, false if error
 */
static bool panel_update_lhbm_notimeout(struct gs_panel *ctx)
{
	const struct gs_panel_mode *pmode;
	struct gs_local_hbm *lhbm = &ctx->lhbm;

	if (!gs_panel_has_func(ctx, set_local_hbm_mode))
		return false;

	/*
	 * If the requested state is already effective, or if we're in the
	 * process of enabling the requested state, don't do anything
	 */
	if (lhbm->effective_state == lhbm->requested_state)
		return false;
	if (lhbm->effective_state != GLOCAL_HBM_DISABLED &&
	    lhbm->requested_state != GLOCAL_HBM_DISABLED)
		return false;

	pmode = ctx->current_mode;
	if (unlikely(pmode == NULL)) {
		dev_err(ctx->dev, "%s: unknown current mode\n", __func__);
		return false;
	}

	if (lhbm->requested_state != GLOCAL_HBM_DISABLED && ctx->desc->lhbm_desc &&
	    !ctx->desc->lhbm_desc->no_lhbm_rr_constraints) {
		const int vrefresh = drm_mode_vrefresh(&pmode->mode);
		/* only allow to turn on LHBM at max refresh rate to comply with HW constraint */
		if (ctx->max_vrefresh && vrefresh != ctx->max_vrefresh) {
			dev_err(ctx->dev, "unexpected mode `%s` while enabling LHBM, give up\n",
				pmode->mode.name);
			return false;
		}
	}

	if (gs_is_local_hbm_post_enabling_supported(ctx)) {
		if (lhbm->requested_state != GLOCAL_HBM_DISABLED) {
			lhbm->timestamps.en_cmd_ts = ktime_get();
			kthread_queue_work(&lhbm->work_data.worker, &lhbm->work_data.post_work);
		} else {
			/*
			 * post_work also holds mode_lock. Release the lock
			 * before finishing post_work to avoid deadlock.
			 */
			mutex_unlock(&ctx->mode_lock);
			kthread_cancel_work_sync(&lhbm->work_data.post_work);
			mutex_lock(&ctx->mode_lock);
		}
	}

	dev_dbg(ctx->dev, "%s: requested %d, effective %d\n", __func__, lhbm->requested_state,
		lhbm->effective_state);
	lhbm->effective_state =
		(lhbm->requested_state && gs_is_local_hbm_post_enabling_supported(ctx)) ?
			GLOCAL_HBM_ENABLING :
			lhbm->requested_state;

	PANEL_ATRACE_BEGIN(__func__);
	ctx->desc->gs_panel_func->set_local_hbm_mode(ctx, lhbm->effective_state);
	sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
	PANEL_ATRACE_END(__func__);

	return true;
}

void panel_update_lhbm(struct gs_panel *ctx)
{
	struct gs_local_hbm_work_data *work_data = &ctx->lhbm.work_data;

	if (ctx->lhbm.requested_state != GLOCAL_HBM_DISABLED) {
		/* reset timeout timer if re-enabling lhbm */
		if (!gs_is_local_hbm_disabled(ctx)) {
			mod_delayed_work(work_data->wq, &work_data->timeout_work,
					 msecs_to_jiffies(ctx->lhbm.max_timeout_ms));
			return;
		}
		if (!panel_update_lhbm_notimeout(ctx))
			return;
		queue_delayed_work(work_data->wq, &work_data->timeout_work,
				   msecs_to_jiffies(ctx->lhbm.max_timeout_ms));
	} else {
		cancel_delayed_work(&work_data->timeout_work);
		panel_update_lhbm_notimeout(ctx);
	}
}

/**
 * local_hbm_timeout_work() - Callback for when lhbm timeout occurs
 * @work: Reference to work struct executing this callback
 */
static void local_hbm_timeout_work(struct work_struct *work)
{
	struct gs_panel *ctx =
		container_of(work, struct gs_panel, lhbm.work_data.timeout_work.work);
	struct device *dev = ctx->dev;

	dev_info(dev, "lhbm_timeout_work: turn off LHBM\n");
	mutex_lock(&ctx->mode_lock); /* TODO(b/267170999): MODE */
	ctx->lhbm.requested_state = GLOCAL_HBM_DISABLED;
	panel_update_lhbm_notimeout(ctx);
	mutex_unlock(&ctx->mode_lock); /* TODO(b/267170999): MODE */
}

/**
 * usleep_since_ts() - Sleeps until offset_us after ts
 * @ts: Fixed point to measure from
 * @offset_us: Time after ts to wait
 */
static void usleep_since_ts(ktime_t ts, u32 offset_us)
{
	u32 us = ktime_us_delta(ktime_get(), ts);

	if (offset_us <= us)
		return;
	us = offset_us - us;
	usleep_range(us, us + 10);
}

/**
 * lhbm_wait_vblank_and_delay() - Waits until a specified time after next vblank
 * @lhbm: Handle for waiting panel's lhbm information
 * @crtc: Associated crtc for panel
 * @frames: number of full frames to wait
 * @offset_us: Time after ts to wait
 */
static void lhbm_wait_vblank_and_delay(struct gs_local_hbm *lhbm, struct drm_crtc *crtc, int frames,
				       u32 offset_us)
{
	struct gs_local_hbm_timestamps *timestamps = &lhbm->timestamps;

	frames -= lhbm->frame_index;
	while (frames > 0) {
		ktime_t now;

		drm_crtc_wait_one_vblank(crtc);
		now = ktime_get();
		if (lhbm->frame_index == 0)
			timestamps->next_vblank_ts = now;
		lhbm->frame_index++;
		frames--;
		timestamps->last_vblank_ts = now;
	}

	usleep_since_ts(timestamps->last_vblank_ts, offset_us);
}

static void local_hbm_wait_and_send_post_cmd(struct gs_panel *ctx, struct drm_crtc *crtc)
{
	const u32 per_frame_us = get_current_frame_duration_us(ctx);
	u32 frames = ctx->desc->lhbm_desc->post_cmd_delay_frames;
	struct gs_local_hbm *lhbm = &ctx->lhbm;

	if (frames == 0)
		return;

	if (crtc)
		/* wait for 0.5 frame time to ensure panel internal scanout or vsync has started */
		lhbm_wait_vblank_and_delay(lhbm, crtc, frames, per_frame_us / 2);
	else
		/* align with the time of sending enabling cmd */
		usleep_since_ts(lhbm->timestamps.en_cmd_ts, per_frame_us * frames);

	dev_dbg(ctx->dev, "%s: delay(us): %lld(EN), %lld(TE)\n", __func__,
		ktime_us_delta(ktime_get(), lhbm->timestamps.en_cmd_ts),
		lhbm->timestamps.next_vblank_ts ?
			ktime_us_delta(ktime_get(), lhbm->timestamps.next_vblank_ts) :
			0);
	if (gs_panel_has_func(ctx, set_local_hbm_mode_post)) {
		mutex_lock(&ctx->mode_lock); /* TODO(b/267170999): MODE */
		ctx->desc->gs_panel_func->set_local_hbm_mode_post(ctx);
		mutex_unlock(&ctx->mode_lock); /* TODO(b/267170999): MODE */
	}
}

static u32 get_local_hbm_effective_delay_frames(struct gs_panel *ctx)
{
	if (gs_panel_has_func(ctx, get_local_hbm_mode_effective_delay_frames))
		return ctx->desc->gs_panel_func->get_local_hbm_mode_effective_delay_frames(ctx);

	return ctx->desc->lhbm_desc->effective_delay_frames;
}

static void local_hbm_wait_and_notify_effectiveness(struct gs_panel *ctx, struct drm_crtc *crtc)
{
	const u32 per_frame_us = get_current_frame_duration_us(ctx);
	const u32 offset_us = per_frame_us * 4 / 5;
	u32 frames = get_local_hbm_effective_delay_frames(ctx);
	struct gs_local_hbm *lhbm = &ctx->lhbm;

	if (frames == 0)
		return;

	if (crtc)
		/* wait for 0.8 frame time to ensure finishing LHBM spot scanout */
		lhbm_wait_vblank_and_delay(lhbm, crtc, frames, offset_us);
	else
		/* take worst case (cmd sent immediately after last vsync) into account */
		usleep_since_ts(lhbm->timestamps.en_cmd_ts, per_frame_us * frames + offset_us);

	dev_dbg(ctx->dev, "%s: delay(us): %lld(EN), %lld(TE) %u(Frames)\n", __func__,
		ktime_us_delta(ktime_get(), lhbm->timestamps.en_cmd_ts),
		lhbm->timestamps.next_vblank_ts ?
			ktime_us_delta(ktime_get(), lhbm->timestamps.next_vblank_ts) :
			0,
		frames);
	if (lhbm->effective_state == GLOCAL_HBM_ENABLING) {
		lhbm->effective_state = GLOCAL_HBM_ENABLED;
		sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
	} else {
		dev_warn(ctx->dev, "%s: LHBM state = %d before becoming effective\n", __func__,
			 lhbm->effective_state);
	}
}

/**
 * local_hbm_post_work() - Callback for synchronous post-lhbm-command work
 * @work: Reference to work struct executing callback
 *
 * This function is the entrypoint to the thread which waits after the lhbm-on
 * command is given to execute any delayed lhbm commands, dependent on the panel
 * architecture.
 */
static void local_hbm_post_work(struct kthread_work *work)
{
	struct gs_panel *ctx = container_of(work, struct gs_panel, lhbm.work_data.post_work);
	const struct gs_panel_desc *desc = ctx->desc;
	struct drm_crtc *crtc = get_gs_panel_connector_crtc(ctx);

	PANEL_ATRACE_BEGIN(__func__);
	if (crtc && drm_crtc_vblank_get(crtc))
		crtc = NULL;
	ctx->lhbm.timestamps.next_vblank_ts = 0;
	ctx->lhbm.frame_index = 0;
	/* TODO: delay time might be inaccurate if refresh rate changes around here */
	if (desc->lhbm_desc->post_cmd_delay_frames <= desc->lhbm_desc->effective_delay_frames) {
		local_hbm_wait_and_send_post_cmd(ctx, crtc);
		local_hbm_wait_and_notify_effectiveness(ctx, crtc);
	} else {
		local_hbm_wait_and_notify_effectiveness(ctx, crtc);
		local_hbm_wait_and_send_post_cmd(ctx, crtc);
	}
	if (crtc)
		drm_crtc_vblank_put(crtc);
	PANEL_ATRACE_END(__func__);
}

void gs_panel_init_lhbm(struct gs_panel *ctx)
{
	struct device *dev = ctx->dev;
	struct gs_local_hbm_work_data *work_data = &ctx->lhbm.work_data;

	ctx->lhbm.max_timeout_ms = LOCAL_HBM_MAX_TIMEOUT_MS;
	ctx->lhbm.requested_state = GLOCAL_HBM_DISABLED;
	ctx->lhbm.effective_state = GLOCAL_HBM_DISABLED;
	work_data->wq = create_singlethread_workqueue("hbm_workq");
	if (!work_data->wq)
		dev_err(dev, "failed to create hbm workq!\n");
	else
		INIT_DELAYED_WORK(&work_data->timeout_work, local_hbm_timeout_work);

	if (gs_is_local_hbm_post_enabling_supported(ctx)) {
		kthread_init_worker(&work_data->worker);
		work_data->thread =
			kthread_run(kthread_worker_fn, &work_data->worker, "lhbm_kthread");
		if (IS_ERR(work_data->thread))
			dev_err(dev, "failed to run display lhbm kthread\n");
		else {
			struct sched_param param = {
				.sched_priority = 2, // MAX_RT_PRIO - 1,
			};
			sched_setscheduler_nocheck(work_data->thread, SCHED_FIFO, &param);
			kthread_init_work(&work_data->post_work, local_hbm_post_work);
		}
	}
}
