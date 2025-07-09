/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include "gs_panel_internal.h"

#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/sysfs.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "gs_drm/gs_drm_connector.h"
#include "gs_panel/gs_panel.h"
#include "trace/panel_trace.h"

#define bridge_to_gs_panel(b) container_of((b), struct gs_panel, bridge)

#ifndef DISPLAY_PANEL_INDEX_PRIMARY
#define DISPLAY_PANEL_INDEX_PRIMARY 0
#endif
#ifndef DISPLAY_PANEL_INDEX_SECONDARY
#define DISPLAY_PANEL_INDEX_SECONDARY 1
#endif

static unsigned long get_backlight_state_from_panel(struct backlight_device *bl,
						    enum gs_panel_state panel_state)
{
	unsigned long state = bl->props.state;

	switch (panel_state) {
	case GPANEL_STATE_NORMAL:
		state &= ~(BL_STATE_STANDBY | BL_STATE_LP);
		break;
	case GPANEL_STATE_LP:
		state &= ~(BL_STATE_STANDBY);
		state |= BL_STATE_LP;
		break;
	case GPANEL_STATE_MODESET: /* no change */
		break;
	case GPANEL_STATE_OFF:
	case GPANEL_STATE_BLANK:
	default:
		state &= ~(BL_STATE_LP);
		state |= BL_STATE_STANDBY;
		break;
	}

	return state;
}

void gs_panel_set_backlight_state(struct gs_panel *ctx, enum gs_panel_state panel_state)
{
	struct backlight_device *bl = ctx->bl;
	unsigned long state;
	bool state_changed = false;

	if (!bl)
		return;

	mutex_lock(&ctx->bl_state_lock); /*TODO(b/267170999): BL*/

	state = get_backlight_state_from_panel(bl, panel_state);
	if (state != bl->props.state) {
		bl->props.state = state;
		state_changed = true;
	}

	mutex_unlock(&ctx->bl_state_lock); /*TODO(b/267170999): BL*/

	if (state_changed) {
		notify_panel_mode_changed(ctx);
		dev_info(ctx->dev, "panel: %s | bl: brightness@%u, state@%#x\n",
			gs_get_panel_state_string(panel_state), bl->props.brightness,
			bl->props.state);
	}
}

static const char *gs_panel_get_sysfs_name(struct gs_panel *ctx)
{
	switch (ctx->gs_connector->panel_index) {
	case DISPLAY_PANEL_INDEX_PRIMARY:
		return "primary-panel";
	case DISPLAY_PANEL_INDEX_SECONDARY:
		return "secondary-panel";
	default:
		dev_warn(ctx->dev, "Unsupported panel_index value %d\n",
			 ctx->gs_connector->panel_index);
		return "primary-panel";
	}
}

void gs_panel_node_attach(struct gs_drm_connector *gs_connector)
{
	struct gs_panel *ctx = gs_connector_to_panel(gs_connector);
	struct drm_connector *connector = &gs_connector->base;
	const char *sysfs_name;
	struct drm_bridge *bridge;
	int ret;

	if (unlikely(!ctx)) {
		WARN(1, "%s: failed to get gs_panel\n", __func__);
		return;
	}

	/* Create sysfs links from connector to panel */
	ret = sysfs_create_link(&gs_connector->kdev->kobj, &ctx->dev->kobj, "panel");
	if (ret)
		dev_warn(ctx->dev, "unable to link connector platform dev to panel (%d)\n", ret);

	ret = sysfs_create_link(&connector->kdev->kobj, &ctx->dev->kobj, "panel");
	if (ret)
		dev_warn(ctx->dev, "unable to link connector drm dev to panel (%d)\n", ret);

	/* debugfs entries */
	gs_panel_create_debugfs_entries(ctx, connector->debugfs_entry);

	bridge = &ctx->bridge;
	sysfs_name = gs_panel_get_sysfs_name(ctx);

	ret = sysfs_create_link(&bridge->dev->dev->kobj, &ctx->dev->kobj, sysfs_name);
	if (ret)
		dev_warn(ctx->dev, "unable to link %s sysfs (%d)\n", sysfs_name, ret);
	else
		dev_dbg(ctx->dev, "succeed to link %s sysfs\n", sysfs_name);
}

static int gs_panel_bridge_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags)
{
	struct gs_panel *ctx = bridge_to_gs_panel(bridge);
	struct device *dev = ctx->dev;
	struct gs_drm_connector *gs_connector = get_gs_drm_connector_parent(ctx);
	struct drm_connector *connector = &gs_connector->base;
	int ret;

	/* Initialize connector, attach properties, and register */
	ret = gs_panel_initialize_gs_connector(ctx, bridge->dev, gs_connector);
	if (ret) {
		return ret;
	}

	ret = drm_connector_attach_encoder(connector, bridge->encoder);
	if (ret) {
		dev_warn(dev, "%s attaching encoder returned nonzero code (%d)\n", __func__, ret);
	}

	if (gs_panel_has_func(ctx, commit_done))
		ctx->gs_connector->needs_commit = true;

	if (connector->dev->mode_config.poll_enabled)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0)
		drm_kms_helper_connector_hotplug_event(connector);
#else
		drm_kms_helper_hotplug_event(connector->dev);
#endif

	return 0;
}

static void gs_panel_bridge_detach(struct drm_bridge *bridge)
{
	struct gs_panel *ctx = bridge_to_gs_panel(bridge);
	struct drm_connector *connector = &ctx->gs_connector->base;
	const char *sysfs_name = gs_panel_get_sysfs_name(ctx);

	sysfs_remove_link(&bridge->dev->dev->kobj, sysfs_name);

	/* TODO(tknelms): debugfs removal */
	sysfs_remove_link(&connector->kdev->kobj, "panel");
	/* TODO(tknelms): evaluate what needs to be done to clean up connector */
	drm_connector_unregister(connector);
	drm_connector_cleanup(&ctx->gs_connector->base);
}

static void gs_panel_bridge_enable(struct drm_bridge *bridge,
				   struct drm_bridge_state *old_bridge_state)
{
	struct gs_panel *ctx = bridge_to_gs_panel(bridge);
	const struct drm_connector_state *conn_state = ctx->gs_connector->base.state;
	struct gs_drm_connector_state *gs_conn_state = to_gs_connector_state(conn_state);
	bool need_update_backlight = false;
	bool is_active;
	const bool is_lp_mode = ctx->current_mode && ctx->current_mode->gs_mode.is_lp_mode;

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	if (ctx->panel_state == GPANEL_STATE_HANDOFF) {
		is_active = !gs_panel_first_enable_helper(ctx);
	} else if (ctx->panel_state == GPANEL_STATE_HANDOFF_MODESET) {
		if (!gs_panel_first_enable_helper(ctx)) {
			ctx->panel_state = GPANEL_STATE_MODESET;
			mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
			drm_panel_disable(&ctx->base);
			mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
		}
		is_active = false;
	} else {
		is_active = gs_is_panel_active(ctx);
	}

	/* avoid turning on panel again if already enabled (ex. while booting or self refresh) */
	if (!is_active) {
		drm_panel_enable(&ctx->base);
		need_update_backlight = true;
	}
	ctx->panel_state = is_lp_mode ? GPANEL_STATE_LP : GPANEL_STATE_NORMAL;

	if (gs_panel_has_func(ctx, update_ffc) &&
	    (!ctx->idle_data.self_refresh_active || gs_conn_state->dsi_hs_clk_changed))
		ctx->desc->gs_panel_func->update_ffc(ctx, gs_conn_state->dsi_hs_clk_mbps);

	if (ctx->idle_data.self_refresh_active) {
		dev_dbg(ctx->dev, "self refresh state : %s\n", __func__);

		ctx->idle_data.self_refresh_active = false;
		panel_update_idle_mode_locked(ctx, false);
	} else {
		gs_panel_set_backlight_state(ctx, ctx->panel_state);
		if (ctx->panel_state == GPANEL_STATE_NORMAL)
			gs_panel_update_te2(ctx);
	}

	if (is_lp_mode && gs_panel_has_func(ctx, set_post_lp_mode))
		ctx->desc->gs_panel_func->set_post_lp_mode(ctx);

	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

	if (need_update_backlight && ctx->bl) {
		backlight_update_status(ctx->bl);
	}

	if (!is_active && gs_panel_has_func(ctx, run_normal_mode_work)) {
		dev_dbg(ctx->dev, "%s: schedule normal_mode_work\n", __func__);
		schedule_delayed_work(&ctx->normal_mode_work,
				      msecs_to_jiffies(ctx->normal_mode_work_delay_ms));
	}
}

/**
 * gs_panel_vsync_start_time_us() - Get vsync start time within TE period
 * @te_us: TE pulse time.
 * @te_period_us: TE period.
 */
static u64 gs_panel_vsync_start_time_us(u32 te_us, u32 te_period_us)
{
	/* Approximate the VSYNC start time with TE falling edge. */
	if (te_us > 0 && te_us < te_period_us)
		return te_us * 105 / 100; /* add 5% for variation */

	/* Approximate the TE falling edge with 55% TE width */
	return te_period_us * 55 / 100;
}

static u32 get_rr_switch_applied_te_count(const struct gs_panel_timestamps *timestamps)
{
	/* New refresh rate should take effect immediately after exiting AOD mode */
	if (timestamps->last_rr_switch_ts == timestamps->last_lp_exit_ts)
		return 1;

	/* New rr will take effect at the first vsync after sending rr command, but
	 * we only know te rising ts. The worse case, new rr take effect at 2nd TE.
	 */
	return 2;
}

static bool is_last_rr_applied(struct gs_panel *ctx, ktime_t last_te)
{
	s64 rr_switch_delta_us;
	u32 te_period_before_rr_switch_us;
	u32 rr_switch_applied_te_count;

	if (last_te == 0)
		return false;

	rr_switch_delta_us = ktime_us_delta(last_te, ctx->timestamps.last_rr_switch_ts);
	te_period_before_rr_switch_us = ctx->te2.last_rr != 0 ? USEC_PER_SEC / ctx->te2.last_rr : 0;
	rr_switch_applied_te_count = get_rr_switch_applied_te_count(&ctx->timestamps);

	if (rr_switch_delta_us > ((rr_switch_applied_te_count - 1) * te_period_before_rr_switch_us))
		return true;

	return false;
}

/* avoid accumulate te varaince cause predicted value is not precision enough */
#define ACCEPTABLE_TE_PERIOD_DETLA_NS (3 * NSEC_PER_SEC)
#define ACCEPTABLE_TE_RR_SWITCH_DELTA_US (500)
static ktime_t gs_panel_te_ts_prediction(struct gs_panel *ctx, ktime_t last_te, u32 te_period_us)
{
	s64 rr_switch_delta_us, te_last_rr_switch_delta_us;
	u32 te_period_before_rr_switch_us;
	u32 rr_switch_applied_te_count;
	s64 te_period_delta_ns;

	if (last_te == 0)
		return 0;

	rr_switch_delta_us = ktime_us_delta(last_te, ctx->timestamps.last_rr_switch_ts);
	te_period_before_rr_switch_us = ctx->te2.last_rr != 0 ? USEC_PER_SEC / ctx->te2.last_rr : 0;
	rr_switch_applied_te_count = get_rr_switch_applied_te_count(&ctx->timestamps);

	if (rr_switch_delta_us < 0 && te_period_before_rr_switch_us != 0) {
		/* last know TE ts is before sending last rr switch */
		ktime_t last_te_before_rr_switch, now;
		s64 since_last_te_us;
		s64 accumlated_te_period_delta_ns;

		/* donʼt predict if last rr switch ts too close to te */
		te_last_rr_switch_delta_us = (-rr_switch_delta_us % te_period_before_rr_switch_us);
		if (te_last_rr_switch_delta_us >
			    (te_period_before_rr_switch_us - ACCEPTABLE_TE_RR_SWITCH_DELTA_US) ||
		    te_last_rr_switch_delta_us < ACCEPTABLE_TE_RR_SWITCH_DELTA_US) {
			return 0;
		}

		te_period_delta_ns = (-rr_switch_delta_us / te_period_before_rr_switch_us) *
				     te_period_before_rr_switch_us * NSEC_PER_USEC;
		if (te_period_delta_ns < ACCEPTABLE_TE_PERIOD_DETLA_NS) {
			/* try to get last TE ts before sending last rr switch command */
			ktime_t first_te_after_rr_switch;

			last_te_before_rr_switch = last_te + te_period_delta_ns;
			now = ktime_get();
			since_last_te_us = ktime_us_delta(now, last_te_before_rr_switch);
			if (since_last_te_us < te_period_before_rr_switch_us) {
				/* now and last predict te is in the same te */
				return last_te_before_rr_switch;
			}

			first_te_after_rr_switch =
				last_te_before_rr_switch + te_period_before_rr_switch_us;

			if (rr_switch_applied_te_count == 1) {
				since_last_te_us = ktime_us_delta(now, first_te_after_rr_switch);
				accumlated_te_period_delta_ns = te_period_delta_ns;
				te_period_delta_ns = (since_last_te_us / te_period_us) *
						     te_period_us * NSEC_PER_USEC;
				accumlated_te_period_delta_ns += te_period_delta_ns;
				if (accumlated_te_period_delta_ns < ACCEPTABLE_TE_PERIOD_DETLA_NS)
					return (first_te_after_rr_switch + te_period_delta_ns);
			} else {
				return first_te_after_rr_switch;
			}
		}
	} else if (is_last_rr_applied(ctx, last_te)) {
		/* new rr has already taken effect at last know TE ts */
		ktime_t now;
		s64 since_last_te_us;

		now = ktime_get();
		since_last_te_us = ktime_us_delta(now, last_te);
		te_period_delta_ns =
			(since_last_te_us / te_period_us) * te_period_us * NSEC_PER_USEC;

		if (te_period_delta_ns < ACCEPTABLE_TE_PERIOD_DETLA_NS)
			return (last_te + te_period_delta_ns);
	}

	return 0;
}

void gs_panel_wait_for_cmd_tx_window(struct drm_crtc *crtc,
				     const struct gs_panel_mode *current_mode,
				     const struct gs_panel_mode *target_mode, struct gs_panel *ctx)
{
	struct device *dev = ctx->dev;
	u32 te_period_us;
	u32 te_usec;
	int retry;
	u64 left, right;
	bool vblank_taken = false;
	bool is_rr_sent_at_te_high = false;

	if (WARN_ON(!current_mode))
		return;

	PANEL_ATRACE_BEGIN("mipi_time_window");
	te_period_us = USEC_PER_SEC / gs_drm_mode_te_freq(&current_mode->mode);

	if (gs_panel_has_func(ctx, get_te_usec))
		te_usec = ctx->desc->gs_panel_func->get_te_usec(ctx, current_mode);
	else
		te_usec = current_mode->gs_mode.te_usec;
	dev_dbg(dev, "%s: check mode_set timing enter. te_period_us %u, te_usec %u\n", __func__,
		te_period_us, te_usec);

	if (gs_panel_has_func(ctx, rr_need_te_high))
		is_rr_sent_at_te_high = ctx->desc->gs_panel_func->rr_need_te_high(ctx, target_mode);

	/*
	 * Safe time window to send RR (refresh rate) command illustrated below.
	 *
	 * When is_rr_sent_at_te_high is false, it sends RR command in TE low
	 * to make RR switch and scanout happen in the same VSYNC period because the frame
	 * content might be adjusted specific to this refresh rate.
	 *
	 * An estimation is [55% * TE_duration, TE_duration - 1ms] before driver has the
	 * accurate TE pulse width (VSYNC rising is a bit ahead of TE falling edge).
	 *
	 *         -->|     |<-- safe time window to send RR
	 *
	 *        +----+     +----+     +-+
	 *        |    |     |    |     | |
	 * TE   --+    +-----+    +-----+ +---
	 *               RR  SCANOUT
	 *
	 *            |          |       |
	 *            |          |       |
	 * VSYNC------+----------+-------+----
	 *            RR1        RR2
	 *
	 * When is_rr_sent_at_te_high is true, it makes RR switch commands are sent in TE
	 * high(skip frame) to makes RR switch happens prior to scanout. This is requested
	 * from specific DDIC to avoid transition flicker. This should not be used for TE
	 * pulse width is very short case.
	 *
	 * An estimation is [0.5ms, 55% * TE_duration - 1ms] before driver has the accurate
	 * TE pulse width (VSYNC rising is a bit ahead of TE falling edge).
	 *
	 *                -->|    |<-- safe time window to send RR
	 *
	 *        +----+     +----+     +-+
	 *        |    |     |    |     | |
	 * TE   --+    +-----+    +-----+ +---
	 *                     RR       SCANOUT
	 *
	 *            |          |       |
	 *            |          |       |
	 * VSYNC------+----------+-------+----
	 *            RR1        RR2
	 *
	 */
	retry = te_period_us / USEC_PER_MSEC + 1;

	do {
		u32 cur_te_period_us = te_period_us;
		u32 cur_te_usec = te_usec;
		ktime_t last_te = 0, now;
		s64 since_last_te_us;
		u64 vblank_counter;
		bool last_rr_applied;

		vblank_counter = drm_crtc_vblank_count_and_time(crtc, &last_te);
		now = ktime_get();
		since_last_te_us = ktime_us_delta(now, last_te);
		if (!vblank_taken) {
			ktime_t predicted_te = gs_panel_te_ts_prediction(
				ctx, last_te,
				USEC_PER_SEC / gs_drm_mode_te_freq(&current_mode->mode));
			if (predicted_te) {
				PANEL_ATRACE_BEGIN("predicted_te");
				last_te = predicted_te;
				since_last_te_us = ktime_us_delta(now, last_te);
				PANEL_ATRACE_INT("predicted_te_delta_us", (int)since_last_te_us);
				PANEL_ATRACE_END("predicted_te");
			}
		}
		/**
		 * If a refresh rate switch happens right before last_te. last TE width could be for
		 * new rr or for old rr depending on if last rr is sent at TE high or low.
		 * If the refresh rate switch happens after last_te, last TE width won't change.
		 */
		last_rr_applied = is_last_rr_applied(ctx, last_te);
		if (ctx->te2.last_rr != 0 &&
		    ((vblank_counter - ctx->te2.last_rr_te_counter <= 1 &&
		      ctx->te2.last_rr_te_gpio_value == 0 && !last_rr_applied) ||
		     ktime_after(ctx->timestamps.last_rr_switch_ts, last_te))) {
			cur_te_period_us = USEC_PER_SEC / ctx->te2.last_rr;
			cur_te_usec = ctx->te2.last_rr_te_usec;
		}

		if (!is_rr_sent_at_te_high) {
			left = gs_panel_vsync_start_time_us(cur_te_usec, cur_te_period_us);
			right = cur_te_period_us - USEC_PER_MSEC;
		} else {
			/* TODO(tknelms): if frame_transfer_pending, continue */
			left = USEC_PER_MSEC * 0.5;
			right = gs_panel_vsync_start_time_us(cur_te_usec, cur_te_period_us) -
				USEC_PER_MSEC;
		}

		dev_dbg(dev,
			"%s: rr-te: %lld, te-now: %lld, time window [%llu, %llu] te/pulse: %u/%u\n",
			__func__, ktime_us_delta(last_te, ctx->timestamps.last_rr_switch_ts),
			ktime_us_delta(now, last_te), left, right, cur_te_period_us, cur_te_usec);

		/* Only use the most recent TE as a reference point if it's not obsolete */
		if (since_last_te_us > cur_te_period_us) {
			PANEL_ATRACE_BEGIN("time_window_wait_crtc");
			if (vblank_taken || !drm_crtc_vblank_get(crtc)) {
				drm_crtc_wait_one_vblank(crtc);
				vblank_taken = true;
			} else {
				pr_warn("%s failed to get vblank for ref point.\n", __func__);
			}
			PANEL_ATRACE_END("time_window_wait_crtc");
			continue;
		}

		if (since_last_te_us <= right) {
			if (since_last_te_us < left) {
				u32 delay_us = left - since_last_te_us;

				PANEL_ATRACE_BEGIN("time_window_wait_te_state");
				usleep_range(delay_us, delay_us + 100);
				PANEL_ATRACE_END("time_window_wait_te_state");
				/*
				 * if a mode switch happens, a TE signal might
				 * happen during the sleep. need to re-sync
				 */
				continue;
			}
			break;
		}

		/* retry in 1ms */
		usleep_range(USEC_PER_MSEC, USEC_PER_MSEC + 100);
	} while (--retry > 0);

	if (vblank_taken)
		drm_crtc_vblank_put(crtc);

	PANEL_ATRACE_END("mipi_time_window");
}

void gs_panel_disable_normal_feat_locked(struct gs_panel *ctx)
{
	bool is_lhbm_enabled = !gs_is_local_hbm_disabled(ctx);
	bool is_hbm_enabled = GS_IS_HBM_ON(ctx->hbm_mode);

	if (is_lhbm_enabled && gs_panel_has_func(ctx, set_local_hbm_mode)) {
		ctx->lhbm.requested_state = GLOCAL_HBM_DISABLED;
		panel_update_lhbm(ctx);
		/* restore the state while calling restore function */
		ctx->lhbm.requested_state = GLOCAL_HBM_ENABLED;
	}
	/* TODO: restore hbm if needed */
	if (is_hbm_enabled && gs_panel_has_func(ctx, set_hbm_mode))
		ctx->desc->gs_panel_func->set_hbm_mode(ctx, GS_HBM_OFF);

	if (!is_lhbm_enabled && !is_hbm_enabled)
		return;

	dev_warn(ctx->dev,
		 "unexpected lhbm(%d) or hbm(%d) @ %s, force off to avoid unpredictable issue\n",
		 is_lhbm_enabled, is_hbm_enabled, (!gs_is_panel_enabled(ctx)) ? "OFF" : "ON or LP");
}

static void bridge_mode_set_enter_lp_mode(struct gs_panel *ctx, const struct gs_panel_mode *pmode,
					  bool is_active)
{
	if (!gs_panel_has_func(ctx, set_lp_mode))
		return;
	if (is_active) {
		gs_panel_disable_normal_feat_locked(ctx);
		ctx->desc->gs_panel_func->set_lp_mode(ctx, pmode);
		ctx->panel_state = GPANEL_STATE_LP;

		if (gs_panel_has_func(ctx, run_normal_mode_work)) {
			dev_dbg(ctx->dev, "%s: cancel normal_mode_work\n", __func__);
			cancel_delayed_work(&ctx->normal_mode_work);
		}
	}
	if (!ctx->regulator.post_vddd_lp_enabled)
		gs_panel_set_vddd_voltage(ctx, true);
	else
		ctx->regulator.need_post_vddd_lp = true;
}

static void bridge_mode_set_leave_lp_mode(struct gs_panel *ctx, const struct gs_panel_mode *pmode,
					  bool is_active)
{
	gs_panel_set_vddd_voltage(ctx, false);
	if (is_active && gs_panel_has_func(ctx, set_nolp_mode)) {
		ctx->desc->gs_panel_func->set_nolp_mode(ctx, pmode);
		ctx->panel_state = GPANEL_STATE_NORMAL;
		/*TODO(b/279521693): lhbm_on_delay_frames*/

		if (gs_panel_has_func(ctx, run_normal_mode_work)) {
			dev_dbg(ctx->dev, "%s: schedule normal_mode_work\n", __func__);
			schedule_delayed_work(&ctx->normal_mode_work,
					      msecs_to_jiffies(ctx->normal_mode_work_delay_ms));
		}
	}
	ctx->current_binned_lp = NULL;

	gs_panel_set_backlight_state(ctx, is_active ? GPANEL_STATE_NORMAL :
						      GPANEL_STATE_OFF);
}

static void bridge_mode_set_normal(struct gs_panel *ctx, const struct gs_panel_mode *pmode,
				 const struct gs_panel_mode *old_mode)
{
	struct drm_connector_state *connector_state = ctx->gs_connector->base.state;
	struct drm_crtc *crtc = connector_state->crtc;
	struct gs_drm_connector_state *gs_connector_state = to_gs_connector_state(connector_state);
	const bool is_active = gs_is_panel_active(ctx);
	const bool was_lp_mode = old_mode && old_mode->gs_mode.is_lp_mode;

	if ((GS_MIPI_CMD_SYNC_REFRESH_RATE & gs_connector_state->mipi_sync) && old_mode)
		gs_panel_wait_for_cmd_tx_window(crtc, old_mode, pmode, ctx);
	if (!gs_is_local_hbm_disabled(ctx) && ctx->desc->lhbm_desc &&
	    !ctx->desc->lhbm_desc->no_lhbm_rr_constraints)
		dev_warn(ctx->dev, "do mode change (`%s`) unexpectedly when LHBM is ON\n",
			 pmode->mode.name);
	ctx->desc->gs_panel_func->mode_set(ctx, pmode);

	if (was_lp_mode)
		gs_panel_set_backlight_state(ctx, is_active ? GPANEL_STATE_NORMAL :
							      GPANEL_STATE_OFF);
	else if (ctx->bl)
		notify_panel_mode_changed(ctx);
}

static void bridge_mode_set_update_timestamps(struct gs_panel *ctx,
					      const struct gs_panel_mode *pmode,
					      const struct gs_panel_mode *old_mode,
					      bool come_out_lp_mode)
{
	struct drm_connector_state *connector_state = ctx->gs_connector->base.state;
	struct drm_crtc *crtc = connector_state->crtc;
	struct gs_drm_connector_state *gs_connector_state = to_gs_connector_state(connector_state);

	if (!old_mode)
		return;
	if ((drm_mode_vrefresh(&pmode->mode) == drm_mode_vrefresh(&old_mode->mode)) &&
		((gs_drm_mode_te_freq(&pmode->mode) == gs_drm_mode_te_freq(&old_mode->mode))))
		return;

	/* save the context in order to predict TE width in
	 * gs_panel_wait_for_cmd_tx_window
	 */
	ctx->timestamps.last_rr_switch_ts = ktime_get();
	ctx->te2.last_rr = gs_drm_mode_te_freq(&old_mode->mode);
	ctx->te2.last_rr_te_gpio_value = gpio_get_value(gs_connector_state->te_gpio);
	ctx->te2.last_rr_te_counter = drm_crtc_vblank_count(crtc);
	if (gs_panel_has_func(ctx, get_te_usec))
		ctx->te2.last_rr_te_usec = ctx->desc->gs_panel_func->get_te_usec(ctx, old_mode);
	else
		ctx->te2.last_rr_te_usec = old_mode->gs_mode.te_usec;
	if (come_out_lp_mode)
		ctx->timestamps.last_lp_exit_ts = ctx->timestamps.last_rr_switch_ts;
	sysfs_notify(&ctx->dev->kobj, NULL, "refresh_rate");
}

static void gs_panel_bridge_mode_set(struct drm_bridge *bridge, const struct drm_display_mode *mode,
				     const struct drm_display_mode *adjusted_mode)
{
	struct gs_panel *ctx = bridge_to_gs_panel(bridge);
	struct device *dev = ctx->dev;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const struct gs_panel_mode *pmode = gs_panel_get_mode(ctx, mode);
	const struct gs_panel_funcs *funcs = ctx->desc->gs_panel_func;
	const struct gs_panel_mode *old_mode;
	bool need_update_backlight = false;
	bool come_out_lp_mode = false;
	u64 waiting_time_us = 0;
	bool needs_waiting = false;

	if (WARN_ON(!pmode))
		return;

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	old_mode = ctx->current_mode;
	if (old_mode != pmode)
		needs_waiting = gs_dsi_cmd_need_wait_for_present_time_locked(ctx, &waiting_time_us);
	mutex_unlock(&ctx->mode_lock);

	if (old_mode == pmode)
		return;

	/* TODO(b/378255330): leave more time for composition */
	if (needs_waiting) {
		PANEL_ATRACE_BEGIN("%s: delay %llu us", __func__, waiting_time_us);
		usleep_range(waiting_time_us, waiting_time_us + 10);
		PANEL_ATRACE_END("%s", __func__);
	}

	mutex_lock(&ctx->mode_lock);
	if (ctx->panel_state == GPANEL_STATE_HANDOFF) {
		dev_warn(dev, "mode change at boot to %s\n", adjusted_mode->name);
		ctx->panel_state = GPANEL_STATE_HANDOFF_MODESET;
	}

	dev_dbg(dev, "changing display mode to %dx%d@%d\n", pmode->mode.hdisplay,
		pmode->mode.vdisplay, drm_mode_vrefresh(&pmode->mode));

	dsi->mode_flags = pmode->gs_mode.mode_flags;
	ctx->timestamps.last_mode_set_ts = ktime_get();

	PANEL_ATRACE_BEGIN("%s: %dx%dx%d@%d", __func__, pmode->mode.hdisplay, pmode->mode.vdisplay,
			   drm_mode_vrefresh(&pmode->mode), gs_drm_mode_te_freq(&pmode->mode));
	if (funcs) {
		const bool is_active = gs_is_panel_active(ctx);
		const bool was_lp_mode = old_mode && old_mode->gs_mode.is_lp_mode;
		const bool is_lp_mode = pmode->gs_mode.is_lp_mode;
		bool state_changed = false;

		if (is_lp_mode) {
			bridge_mode_set_enter_lp_mode(ctx, pmode, is_active);
			if (is_active)
				need_update_backlight = true;
		} else if (was_lp_mode && !is_lp_mode) {
			ctx->regulator.need_post_vddd_lp = false;
			bridge_mode_set_leave_lp_mode(ctx, pmode, is_active);
			if (is_active) {
				state_changed = true;
				need_update_backlight = true;
				come_out_lp_mode = true;
			}
		} else if (gs_panel_has_func(ctx, mode_set)) {
			if (is_active) {
				bridge_mode_set_normal(ctx, pmode, old_mode);
				state_changed = true;
			} else
				dev_warn(
					ctx->dev,
					"don't do mode change (`%s`) when panel isn't in interactive mode\n",
					pmode->mode.name);
		}
		ctx->current_mode = pmode;
		if (state_changed) {
			if (!is_lp_mode)
				gs_panel_update_te2(ctx);
		}
	} else {
		ctx->current_mode = pmode;
	}

	bridge_mode_set_update_timestamps(ctx, pmode, old_mode, come_out_lp_mode);

	if (pmode->gs_mode.is_lp_mode && gs_panel_has_func(ctx, set_post_lp_mode))
		funcs->set_post_lp_mode(ctx);

	PANEL_ATRACE_INT_PID_FMT(drm_mode_vrefresh(mode), ctx->trace_pid, "vrefresh[%s]",
				 ctx->panel_model);
	PANEL_ATRACE_INT_PID_FMT(gs_drm_mode_te_freq(mode), ctx->trace_pid, "vsync[%s]",
				 ctx->panel_model);
	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

	if (need_update_backlight && ctx->bl)
		backlight_update_status(ctx->bl);

	PANEL_ATRACE_END(__func__);
}

static void gs_panel_bridge_disable(struct drm_bridge *bridge,
				    struct drm_bridge_state *old_bridge_state)
{
	struct gs_panel *ctx = bridge_to_gs_panel(bridge);
	struct device *dev = ctx->dev;
	const struct drm_connector_state *conn_state = ctx->gs_connector->base.state;
	struct gs_drm_connector_state *gs_conn_state = to_gs_connector_state(conn_state);
	struct drm_crtc_state *crtc_state = !conn_state->crtc ? NULL : conn_state->crtc->state;
	const bool self_refresh_active = crtc_state && crtc_state->self_refresh_active;

	if (self_refresh_active && !gs_conn_state->blanked_mode) {
		mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
		dev_dbg(dev, "self refresh state : %s\n", __func__);

		ctx->idle_data.self_refresh_active = true;
		panel_update_idle_mode_locked(ctx, false);
		mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

		if (ctx->regulator.post_vddd_lp_enabled && ctx->regulator.need_post_vddd_lp) {
			gs_panel_set_vddd_voltage(ctx, true);
			ctx->regulator.need_post_vddd_lp = false;
		}

		if (gs_panel_has_func(ctx, pre_update_ffc) &&
		    (gs_conn_state->dsi_hs_clk_changed || gs_conn_state->pending_dsi_hs_clk_mbps))
			ctx->desc->gs_panel_func->pre_update_ffc(ctx);
	} else {
		if (gs_conn_state->blanked_mode) {
			/* blanked mode takes precedence over normal modeset */
			ctx->panel_state = GPANEL_STATE_BLANK;
		} else if (crtc_state && crtc_state->mode_changed &&
			   drm_atomic_crtc_effectively_active(crtc_state)) {
			ctx->panel_state = GPANEL_STATE_MODESET;
		} else if (ctx->force_power_on) {
			/* force blank state instead of power off */
			ctx->panel_state = GPANEL_STATE_BLANK;
		} else {
			ctx->panel_state = GPANEL_STATE_OFF;
			ctx->mode_in_progress = MODE_DONE;

			if (gs_panel_has_func(ctx, run_normal_mode_work)) {
				dev_dbg(dev, "%s: cancel normal_mode_work\n", __func__);
				cancel_delayed_work(&ctx->normal_mode_work);
			}
		}

		drm_panel_disable(&ctx->base);
	}
}

static void gs_panel_bridge_pre_enable(struct drm_bridge *bridge,
				       struct drm_bridge_state *old_bridge_state)
{
	struct gs_panel *ctx = bridge_to_gs_panel(bridge);

	if (ctx->panel_state == GPANEL_STATE_BLANK) {
		if (gs_panel_has_func(ctx, panel_reset))
			ctx->desc->gs_panel_func->panel_reset(ctx);
	} else if (!gs_is_panel_enabled(ctx))
		drm_panel_prepare(&ctx->base);
}

static void gs_panel_set_partial(struct gs_display_partial *partial,
				 const struct gs_panel_mode *pmode, bool is_partial)
{
	const struct gs_display_dsc *dsc = &pmode->gs_mode.dsc;
	const struct drm_display_mode *mode = &pmode->mode;

	partial->enabled = is_partial;
	if (!partial->enabled)
		return;

	if (dsc->enabled && dsc->cfg) {
		partial->min_width = DIV_ROUND_UP(mode->hdisplay, dsc->cfg->slice_count);
		partial->min_height = dsc->cfg->slice_height;
	} else {
		partial->min_width = MIN_WIN_BLOCK_WIDTH;
		partial->min_height = MIN_WIN_BLOCK_HEIGHT;
	}
}

/**
 * gs_panel_is_mode_seamless() - check if mode transition can be done seamlessly
 * @ctx: Reference to panel data
 * @mode: Proposed display mode
 *
 * Checks whether the panel can transition to the new mode seamlessly without
 * having to turn the display off before the mode change.
 *
 * In most cases, this is only possible if only the clocks and refresh rates are
 * changing.
 *
 * Return: true if seamless transition possible, false otherwise
 */
static bool gs_panel_is_mode_seamless(const struct gs_panel *ctx, const struct gs_panel_mode *mode)
{
	if (!gs_panel_has_func(ctx, is_mode_seamless))
		return false;
	return ctx->desc->gs_panel_func->is_mode_seamless(ctx, mode);
}

static int gs_drm_connector_check_mode(struct gs_panel *ctx,
				       struct drm_connector_state *connector_state,
				       struct drm_crtc_state *crtc_state)
{
	struct gs_drm_connector_state *gs_connector_state = to_gs_connector_state(connector_state);
	const struct gs_panel_mode *pmode = gs_panel_get_mode(ctx, &crtc_state->mode);
	bool is_video_mode;

	if (!pmode) {
		dev_warn(ctx->dev, "invalid mode %s\n", crtc_state->mode.name);
		return -EINVAL;
	}

	is_video_mode = (pmode->gs_mode.mode_flags & MIPI_DSI_MODE_VIDEO) != 0;

	/* self refresh is only supported in command mode */
	connector_state->self_refresh_aware = !is_video_mode;

	if (crtc_state->connectors_changed || !gs_is_panel_active(ctx))
		gs_connector_state->seamless_possible = false;
	else
		gs_connector_state->seamless_possible = gs_panel_is_mode_seamless(ctx, pmode);

	gs_connector_state->gs_mode = pmode->gs_mode;
	gs_panel_set_partial(&gs_connector_state->partial, pmode, ctx->desc->is_partial);

	return 0;
}

/*
 * this atomic check is called after adjusted mode is populated, so it's safe to modify
 * adjusted_mode if needed at this point
 */
static int gs_panel_bridge_atomic_check(struct drm_bridge *bridge,
					struct drm_bridge_state *bridge_state,
					struct drm_crtc_state *new_crtc_state,
					struct drm_connector_state *conn_state)
{
	struct gs_panel *ctx = bridge_to_gs_panel(bridge);
	struct drm_atomic_state *state = new_crtc_state->state;
	const struct drm_display_mode *current_mode = &ctx->current_mode->mode;
	int ret;

	if (unlikely(!new_crtc_state))
		return 0;

	if (unlikely(!current_mode)) {
		dev_warn(ctx->dev, "%s: failed to get current mode, skip mode check\n", __func__);
	} else {
		struct drm_display_mode *target_mode = &new_crtc_state->adjusted_mode;
		struct gs_drm_connector_state *gs_conn_state = to_gs_connector_state(conn_state);
		int current_vrefresh = drm_mode_vrefresh(current_mode);
		int target_vrefresh = drm_mode_vrefresh(target_mode);
		int current_bts_fps = gs_drm_mode_bts_fps(current_mode,
			ctx->current_mode->gs_mode.min_bts_fps);
		int target_bts_fps = gs_drm_mode_bts_fps(target_mode,
			gs_conn_state->gs_mode.min_bts_fps);

		int clock;

		/* if resolution changing */
		if (current_mode->hdisplay != target_mode->hdisplay &&
		    current_mode->vdisplay != target_mode->vdisplay) {
			/* if refresh rate changing */
			if (current_vrefresh != target_vrefresh ||
			    current_bts_fps != target_bts_fps) {
				/*
				 * While switching resolution and refresh rate (from high to low) in
				 * the same commit, the frame transfer time will become longer due
				 * to BTS update. In the case, frame done time may cross to the next
				 * vsync, which will hit DDIC’s constraint and cause the noises.
				 * Keep the current BTS (higher one) for a few frames to avoid
				 * the problem.
				 */
				if (current_bts_fps > target_bts_fps) {
					target_mode->clock = gs_bts_fps_to_drm_mode_clock(
						target_mode, current_bts_fps);
					if (target_mode->clock != new_crtc_state->mode.clock) {
						new_crtc_state->mode_changed = true;
						dev_dbg(ctx->dev,
							"%s: keep mode (%s) clock %dhz on rrs\n",
							__func__, target_mode->name,
							current_bts_fps);
					}
					clock = target_mode->clock;
				}

				ctx->mode_in_progress = MODE_RES_AND_RR_IN_PROGRESS;
			/* else refresh rate not changing */
			} else {
				ctx->mode_in_progress = MODE_RES_IN_PROGRESS;
			}
		/* else resolution not changing */
		} else {
			if (ctx->mode_in_progress == MODE_RES_AND_RR_IN_PROGRESS &&
			    new_crtc_state->adjusted_mode.clock != new_crtc_state->mode.clock) {
				new_crtc_state->mode_changed = true;
				new_crtc_state->adjusted_mode.clock = new_crtc_state->mode.clock;
				clock = new_crtc_state->mode.clock;
				dev_dbg(ctx->dev, "%s: restore mode (%s) clock after rrs\n",
					__func__, new_crtc_state->mode.name);
			}

			if ((current_vrefresh != target_vrefresh) ||
			    (current_bts_fps != target_bts_fps))
				ctx->mode_in_progress = MODE_RR_IN_PROGRESS;
			else
				ctx->mode_in_progress = MODE_DONE;
		}

		/* debug output */
		if (current_mode->hdisplay != target_mode->hdisplay ||
		    current_mode->vdisplay != target_mode->vdisplay ||
		    current_vrefresh != target_vrefresh || current_bts_fps != target_bts_fps)
			dev_dbg(ctx->dev,
				"%s: current %dx%d@%d(bts %d), target %dx%d@%d(bts %d), type %d\n",
				__func__, current_mode->hdisplay, current_mode->vdisplay,
				current_vrefresh, current_bts_fps, target_mode->hdisplay,
				target_mode->vdisplay, target_vrefresh, target_bts_fps,
				ctx->mode_in_progress);

		/*
		 * We may transfer the frame for the first TE after switching to higher
		 * op_hz. In this case, the DDIC read speed will become higher while
		 * the DPU write speed will remain the same, so underruns would happen.
		 * Use higher BTS can avoid the issue. Also consider the clock from RRS
		 * and select the higher one.
		 */
		if ((gs_conn_state->pending_update_flags & GS_FLAG_OP_RATE_UPDATE) &&
		    gs_conn_state->operation_rate > ctx->op_hz) {
			target_mode->clock =
				gs_bts_fps_to_drm_mode_clock(target_mode, ctx->peak_bts_fps);
			/* use the higher clock to avoid underruns */
			if (target_mode->clock < clock)
				target_mode->clock = clock;

			if (target_mode->clock != new_crtc_state->mode.clock) {
				new_crtc_state->mode_changed = true;
				ctx->boosted_for_op_hz = true;
				dev_dbg(ctx->dev, "%s: raise mode clock %dhz on op_hz %d\n",
					__func__, ctx->peak_bts_fps, gs_conn_state->operation_rate);
			}
		} else if (ctx->boosted_for_op_hz &&
			   new_crtc_state->adjusted_mode.clock != new_crtc_state->mode.clock) {
			new_crtc_state->mode_changed = true;
			ctx->boosted_for_op_hz = false;
			/* use the higher clock to avoid underruns */
			if (new_crtc_state->mode.clock < clock)
				new_crtc_state->adjusted_mode.clock = clock;
			else
				new_crtc_state->adjusted_mode.clock = new_crtc_state->mode.clock;

			dev_dbg(ctx->dev, "%s: restore mode clock after op_hz\n", __func__);
		}
	}

	if (gs_panel_has_func(ctx, atomic_check)) {
		ret = ctx->desc->gs_panel_func->atomic_check(ctx, state);
		if (ret)
			return ret;
	}

	if (!drm_atomic_crtc_needs_modeset(new_crtc_state))
		return 0;

	if (ctx->panel_state == GPANEL_STATE_HANDOFF) {
		struct drm_crtc_state *old_crtc_state =
			drm_atomic_get_old_crtc_state(state, new_crtc_state->crtc);

		if (!old_crtc_state->enable)
			old_crtc_state->self_refresh_active = true;
	}

	return gs_drm_connector_check_mode(ctx, conn_state, new_crtc_state);
}

static void gs_panel_bridge_post_disable(struct drm_bridge *bridge,
					 struct drm_bridge_state *old_bridge_state)
{
	struct gs_panel *ctx = bridge_to_gs_panel(bridge);

	/* fully power off only if panel is in full off mode */
	if (!gs_is_panel_enabled(ctx))
		drm_panel_unprepare(&ctx->base);

	gs_panel_set_backlight_state(ctx, ctx->panel_state);
}

static const struct drm_bridge_funcs gs_panel_bridge_funcs = {
	.attach = gs_panel_bridge_attach,
	.detach = gs_panel_bridge_detach,
	.atomic_enable = gs_panel_bridge_enable,
	.atomic_disable = gs_panel_bridge_disable,
	.atomic_check = gs_panel_bridge_atomic_check,
	.atomic_pre_enable = gs_panel_bridge_pre_enable,
	.atomic_post_disable = gs_panel_bridge_post_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.mode_set = gs_panel_bridge_mode_set,
};

const struct drm_bridge_funcs *get_panel_drm_bridge_funcs(void)
{
	return &gs_panel_bridge_funcs;
}
