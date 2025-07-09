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
#include <linux/sysfs.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_vblank.h>
#include <video/mipi_display.h>

#include "gs_panel/gs_panel.h"
#include "trace/panel_trace.h"

/* Sysfs Node */

static ssize_t serial_number_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	if (!strcmp(ctx->panel_id, ""))
		return -EINVAL;

	return sysfs_emit(buf, "%s\n", ctx->panel_id);
}

static ssize_t panel_extinfo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	return sysfs_emit(buf, "%s\n", ctx->panel_extinfo);
}

static ssize_t panel_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const char *p;

	/* filter priority info in the dsi device name */
	p = strstr(dsi->name, ":");
	if (!p)
		p = dsi->name;
	else
		p++;

	return sysfs_emit(buf, "%s\n", p);
}

static ssize_t panel_model_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return sysfs_emit(buf, "%s\n", ctx->panel_model);
}

static ssize_t panel_idle_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	bool idle_enabled;
	int ret;

	ret = kstrtobool(buf, &idle_enabled);
	if (ret) {
		dev_err(dev, "invalid panel idle value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	if (idle_enabled != ctx->idle_data.panel_idle_enabled) {
		ctx->idle_data.panel_idle_enabled = idle_enabled;

		if (idle_enabled)
			ctx->timestamps.last_panel_idle_set_ts = ktime_get();

		panel_update_idle_mode_locked(ctx, true);
	}
	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

	return count;
}

static ssize_t panel_idle_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return sysfs_emit(buf, "%d\n", ctx->idle_data.panel_idle_enabled);
}

static ssize_t panel_need_handle_idle_exit_store(struct device *dev, struct device_attribute *attr,
						 const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	bool idle_handle_exit;
	int ret;

	ret = kstrtobool(buf, &idle_handle_exit);
	if (ret) {
		dev_err(dev, "invalid panel idle handle exit value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	ctx->idle_data.panel_need_handle_idle_exit = idle_handle_exit;
	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

	return count;
}

static ssize_t panel_need_handle_idle_exit_show(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return sysfs_emit(buf, "%d\n", ctx->idle_data.panel_need_handle_idle_exit);
}

static ssize_t idle_delay_ms_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	u32 idle_delay_ms;
	int ret;

	ret = kstrtou32(buf, 0, &idle_delay_ms);
	if (ret) {
		dev_err(dev, "invalid idle delay ms\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	if (ctx->idle_data.idle_delay_ms != idle_delay_ms) {
		ctx->idle_data.idle_delay_ms = idle_delay_ms;
		panel_update_idle_mode_locked(ctx, true);
	}
	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

	return count;
}
static ssize_t idle_delay_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return sysfs_emit(buf, "%d\n", ctx->idle_data.idle_delay_ms);
}

static ssize_t op_hz_store(struct device *dev, struct device_attribute *attr, const char *buf,
			   size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;
	u32 hz;

	if (!count)
		return -EINVAL;

	ret = kstrtou32(buf, 0, &hz);
	if (ret) {
		dev_err(ctx->dev, "invalid op_hz value\n");
		return ret;
	}

	ret = gs_panel_set_op_hz(ctx, hz);
	if (ret)
		return ret;

	return count;
}

static ssize_t op_hz_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!gs_is_panel_initialized(ctx))
		return -EAGAIN;

	if (!gs_panel_has_func(ctx, set_op_hz))
		return -EINVAL;

	return sysfs_emit(buf, "%u\n", ctx->op_hz);
}

static ssize_t refresh_rate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct gs_panel_mode *current_mode;
	int rr = -1;

	mutex_lock(&ctx->mode_lock);
	current_mode = ctx->current_mode;
	if (current_mode != NULL)
		rr = drm_mode_vrefresh(&current_mode->mode);
	mutex_unlock(&ctx->mode_lock);

	return sysfs_emit(buf, "%d\n", rr);
}

static ssize_t refresh_ctrl_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;
	u32 ctrl;
	bool auto_fi;
	bool onetime_fi;

	if (!count)
		return -EINVAL;

	if (!gs_panel_has_func(ctx, refresh_ctrl))
		return -EINVAL;

	ret = kstrtou32(buf, 0, &ctrl);
	if (ret) {
		dev_err(ctx->dev, "%s: failed to parse input\n", __func__);
		return -EINVAL;
	}

	auto_fi = ctrl & GS_PANEL_REFRESH_CTRL_FI_AUTO;
	onetime_fi = ctrl & GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_MASK;
	if (auto_fi && onetime_fi) {
		dev_err(ctx->dev, "%s: invalid command combination: 0x%X\n", __func__, ctrl);
		return -EINVAL;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->refresh_ctrl = ctrl;
	if (!gs_is_panel_initialized(ctx) || !gs_is_panel_enabled(ctx)) {
		dev_info(dev, "%s: cache ctrl=0x%08lX\n", __func__,
			 ctrl & GS_PANEL_REFRESH_CTRL_FEATURE_MASK);
	} else {
		PANEL_ATRACE_INT("refresh_ctrl_value", ctrl);
		ctx->desc->gs_panel_func->refresh_ctrl(ctx);
	}
	ctx->refresh_ctrl &= GS_PANEL_REFRESH_CTRL_FEATURE_MASK;
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t refresh_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return sysfs_emit(buf, "%s\n", gs_panel_has_func(ctx, refresh_ctrl) ? "Enabled" :
									      "Disabled");
}

static ssize_t min_vrefresh_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int min_vrefresh;
	int ret;

	ret = kstrtoint(buf, 0, &min_vrefresh);
	if (ret) {
		dev_err(dev, "invalid min vrefresh value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	if (ctx->min_vrefresh != min_vrefresh) {
		ctx->min_vrefresh = min_vrefresh;
		panel_update_idle_mode_locked(ctx, true);
	}
	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

	return count;
}

static ssize_t min_vrefresh_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return sysfs_emit(buf, "%d\n", ctx->min_vrefresh);
}

/**
 * gs_get_te2_timing() - Outputs te2 timingss to sysfs
 * @ctx: panel struct
 * @buf: output buffer for human-readable te2 data
 * @lp_mode: whether these timings apply to LP modes
 *
 * Return: number of bytes written to buffer
 */
static ssize_t gs_get_te2_timing(struct gs_panel *ctx, char *buf, bool lp_mode)
{
	size_t len;

	if (!gs_panel_has_func(ctx, get_te2_edges))
		return -EPERM;

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	len = ctx->desc->gs_panel_func->get_te2_edges(ctx, buf, lp_mode);
	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

	return len;
}

static ssize_t te2_timing_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!gs_is_panel_initialized(ctx))
		return -EAGAIN;

	ret = gs_set_te2_timing(ctx, count, buf, false);
	if (ret < 0)
		dev_err(ctx->dev, "failed to set normal mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_timing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!gs_is_panel_initialized(ctx))
		return -EAGAIN;

	ret = gs_get_te2_timing(ctx, buf, false);
	if (ret < 0)
		dev_err(ctx->dev, "failed to get normal mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_lp_timing_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!gs_is_panel_initialized(ctx))
		return -EAGAIN;

	ret = gs_set_te2_timing(ctx, count, buf, true);
	if (ret < 0)
		dev_err(ctx->dev, "failed to set LP mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_lp_timing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!gs_is_panel_initialized(ctx))
		return -EAGAIN;

	ret = gs_get_te2_timing(ctx, buf, true);
	if (ret < 0)
		dev_err(ctx->dev, "failed to get LP mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t time_in_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	struct display_stats *stats = &ctx->disp_stats;
	int state, vrefresh_idx, res_idx, time_state_idx;
	u64 time, delta_ms;
	ssize_t len = 0;

	if (!stats->initialized)
		return -ENODEV;

	mutex_lock(&stats->lock);
	delta_ms = ktime_ms_delta(ktime_get_boottime(), stats->last_update);
	for (state = 0; state < DISPLAY_STATE_MAX; state++) {
		int *vrefresh_range;
		size_t vrefresh_range_count;

		if (!stats->time_in_state[state].available_count)
			continue;

		if (state == DISPLAY_STATE_OFF) {
			time = stats->time_in_state[state].time[0];
			if (stats->last_state == state)
				time += delta_ms;
			if (time) {
				len += sysfs_emit_at(buf, len, "%d 0 0 0 %llu\n",
				DISPLAY_STATE_OFF, time);
			}
			continue;
		}

		if (state == DISPLAY_STATE_LP) {
			vrefresh_range = stats->lp_vrefresh_range;
			vrefresh_range_count = stats->lp_vrefresh_range_count;
		} else {
			vrefresh_range = stats->vrefresh_range;
			vrefresh_range_count = stats->vrefresh_range_count;
		}
		for (res_idx = 0; res_idx < stats->res_table_count; res_idx++) {
			for (vrefresh_idx = 0; vrefresh_idx < vrefresh_range_count;
					vrefresh_idx++) {
				int vrefresh;

				vrefresh = vrefresh_range[vrefresh_idx];
				time_state_idx = get_disp_stats_time_state_idx(
					ctx, state, vrefresh, stats->res_table[res_idx]);
				if (time_state_idx < 0)
					continue;

				time = stats->time_in_state[state].time[time_state_idx];
				if (state == stats->last_state &&
					time_state_idx == stats->last_time_state_idx) {
					time += delta_ms;
				}
				if (!time)
					continue;

				len += sysfs_emit_at(buf, len,
					"%d %u %u %d %llu\n", state,
					stats->res_table[res_idx].hdisplay,
					stats->res_table[res_idx].vdisplay,
					vrefresh,
					time);
			}
		}
	}

	mutex_unlock(&stats->lock);
	return len;
}

static ssize_t available_disp_stats_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	struct display_stats *stats = &ctx->disp_stats;
	int state, vrefresh_idx, res_idx;
	ssize_t len = 0;

	if (!stats->initialized)
		return -ENODEV;

	mutex_lock(&stats->lock);
	for (state = 0; state < DISPLAY_STATE_MAX; state++) {
		int *vrefresh_range;
		size_t vrefresh_range_count;

		if (!stats->time_in_state[state].available_count)
			continue;

		if (state == DISPLAY_STATE_OFF) {
			len += sysfs_emit_at(buf, len, "%d 0 0 0\n", state);
			continue;
		}

		if (state == DISPLAY_STATE_LP) {
			vrefresh_range = stats->lp_vrefresh_range;
			vrefresh_range_count = stats->lp_vrefresh_range_count;
		} else {
			vrefresh_range = stats->vrefresh_range;
			vrefresh_range_count = stats->vrefresh_range_count;
		}
		for (res_idx = 0; res_idx < stats->res_table_count; res_idx++) {
			for (vrefresh_idx = 0; vrefresh_idx < vrefresh_range_count;
					vrefresh_idx++) {
				u16 hdisplay, vdisplay;
				int vrefresh;

				hdisplay = stats->res_table[res_idx].hdisplay;
				vdisplay = stats->res_table[res_idx].vdisplay;
				vrefresh = vrefresh_range[vrefresh_idx];
				len += sysfs_emit_at(buf, len, "%d %u %u %d\n",
						state, hdisplay, vdisplay, vrefresh);
			}
		}
	}

	mutex_unlock(&stats->lock);
	return len;
}

static ssize_t te_rate_hz_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int freq;
	bool changeable;

	if (!gs_is_panel_active(ctx))
		return -EPERM;

	mutex_lock(&ctx->mode_lock);
	changeable = (ctx->hw_status.te.option == TEX_OPT_CHANGEABLE);
	if (changeable) {
		const struct gs_panel_mode *current_mode = ctx->current_mode;

		if (!current_mode) {
			mutex_unlock(&ctx->mode_lock);
			return -EINVAL;
		}
		freq = drm_mode_vrefresh(&current_mode->mode);
	} else {
		freq = ctx->hw_status.te.rate_hz;
	}
	mutex_unlock(&ctx->mode_lock);

	return sysfs_emit(buf, "%d\n", freq);
}

static ssize_t te_option_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	bool changeable;

	if (!gs_is_panel_active(ctx))
		return -EPERM;

	mutex_lock(&ctx->mode_lock);
	changeable = (ctx->hw_status.te.option == TEX_OPT_CHANGEABLE);
	mutex_unlock(&ctx->mode_lock);

	return sysfs_emit(buf, "%s\n", changeable ? "changeable" : "fixed");
}

static ssize_t te2_rate_hz_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;
	u32 rate_hz;

	if (!gs_panel_has_func(ctx, set_te2_rate))
		return -ENOTSUPP;

	ret = kstrtouint(buf, 0, &rate_hz);
	if (ret) {
		dev_err(dev, "invalid TE2 rate value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	if (!gs_is_panel_active(ctx)) {
		dev_warn(ctx->dev, "%s: cache rate(%u)\n", __func__, rate_hz);
		ctx->te2.rate_hz = rate_hz;
	} else if (ctx->desc->gs_panel_func->set_te2_rate(ctx, rate_hz)) {
		/**
		 * The TE2 rate reflects the display refresh rate. And we're interested in the
		 * rates while the display is active or idle. Notify immediately if the it's
		 * active since we usually hope to jump to the peak refresh rate soon. If it's
		 * idle, we may have several inserted frames before dropping to the lower refresh
		 * rate to avoid flickers. Adding an estimated delay can help make the notification
		 * more accurate.
		 */
		int vrefresh =
			ctx->current_mode ? drm_mode_vrefresh(&ctx->current_mode->mode) : 0;
		bool need_delay = (ctx->te2.option == TEX_OPT_CHANGEABLE) && vrefresh &&
				  (rate_hz != vrefresh);
		u32 delay_ms = need_delay ? ctx->desc->notify_te2_rate_changed_work_delay_ms : 0;

		dev_dbg(dev, "%s: vrefresh %d, rate_hz %u, delay_ms %u\n", __func__,
			vrefresh, rate_hz, delay_ms);
		notify_panel_te2_rate_changed(ctx, delay_ms);
	}
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t te2_rate_hz_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	if (!gs_panel_has_func(ctx, get_te2_rate))
		return -ENOTSUPP;

	/**
	 * Still allow the read if the panel is inactive at this moment since we may change
	 * the rate during the transition to active.
	 */
	if (!gs_is_panel_active(ctx))
		dev_warn(ctx->dev, "%s: panel is not enabled, may show previous rate\n", __func__);

	mutex_lock(&ctx->mode_lock);
	ret = sysfs_emit(buf, "%u\n", ctx->desc->gs_panel_func->get_te2_rate(ctx));
	mutex_unlock(&ctx->mode_lock);

	return ret;
}

static ssize_t te2_option_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;
	u32 option;

	if (!gs_panel_has_func(ctx, set_te2_option))
		return -ENOTSUPP;

	ret = kstrtou32(buf, 0, &option);
	if (ret) {
		dev_err(dev, "invalid TE2 option value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	if (!gs_is_panel_active(ctx)) {
		dev_warn(ctx->dev, "%s: cache option(%u)\n", __func__, option);
		ctx->te2.option = option;
	} else if (ctx->desc->gs_panel_func->set_te2_option(ctx, option)) {
		notify_panel_te2_option_changed(ctx);
	}
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t te2_option_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	enum gs_panel_tex_opt option;

	if (!gs_panel_has_func(ctx, get_te2_option))
		return -ENOTSUPP;

	if (!gs_is_panel_active(ctx)) {
		dev_warn(ctx->dev, "%s: panel is not enabled\n", __func__);
		return -EPERM;
	}

	mutex_lock(&ctx->mode_lock);
	option = ctx->desc->gs_panel_func->get_te2_option(ctx);
	mutex_unlock(&ctx->mode_lock);

	return sysfs_emit(buf, "%s\n", (option == TEX_OPT_CHANGEABLE) ? "changeable" : "fixed");
}

static ssize_t power_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	enum display_stats_state state;

	mutex_lock(&ctx->bl_state_lock);
	state = gs_get_current_display_state_locked(ctx);
	mutex_unlock(&ctx->bl_state_lock);

	return sysfs_emit(buf, "%s\n", get_disp_state_str(state));
}

static ssize_t error_count_te_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	u32 count;

	mutex_lock(&ctx->mode_lock);
	count = sysfs_emit(buf, "%u\n", ctx->error_counter.te);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t error_count_unknown_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	u32 count;

	mutex_lock(&ctx->mode_lock);
	count = sysfs_emit(buf, "%u\n", ctx->error_counter.unknown);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t color_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t raw_len, sysfs_idx = 0;

	if (!ctx->desc->calibration_desc ||
	    (!gs_panel_has_func(ctx, get_color_data) && !ctx->color_data.ready))
		return -EOPNOTSUPP;

	if (!gs_is_panel_active(ctx)) {
		dev_warn(ctx->dev, "%s: panel is not active, state %d\n", __func__,
			 ctx->panel_state);
		return -EPERM;
	}

	mutex_lock(&ctx->mode_lock);
	if (!ctx->color_data.data) {
		mutex_unlock(&ctx->mode_lock);
		return -EINVAL;
	}

	if (!ctx->color_data.ready) {
		raw_len = ctx->desc->gs_panel_func->get_color_data(ctx, ctx->color_data.data,
								   ctx->color_data.size);
		if (raw_len != ctx->color_data.size) {
			mutex_unlock(&ctx->mode_lock);
			dev_err(dev, "Invalid result %zd from color data read\n", raw_len);
			return raw_len < 0 ? raw_len : -EIO;
		}
		ctx->color_data.ready = true;
	}
	mutex_unlock(&ctx->mode_lock);

	for (u16 buf_idx = 0; buf_idx < ctx->color_data.size; buf_idx++)
		sysfs_idx += sysfs_emit_at(buf, sysfs_idx, "%02X", ctx->color_data.data[buf_idx]);
	dev_dbg(dev, "Wrote color data for %zd into sysfs\n", sysfs_idx);

	return sysfs_idx;
}

static ssize_t color_data_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	char *buf_dup;
	u32 options[COLOR_OPTION_DEPTH] = { 0 };
	enum color_data_type type;
	ssize_t ret;
	int option_count;

	if (count == 0)
		return -EINVAL;

	buf_dup = kstrndup(buf, count, GFP_KERNEL);
	if (!buf_dup)
		return -ENOMEM;

	if (strlen(buf_dup) != count) {
		ret = -EINVAL;
		goto out;
	}

	option_count = parse_u32_buf(buf_dup, count + 1, options, COLOR_OPTION_DEPTH);
	if (option_count <= 0) {
		dev_warn(ctx->dev, "Error parsing color_data input buf (%d)\n", option_count);
		ret = option_count < 0 ? option_count : -EINVAL;
		goto out;
	}

	ret = gs_panel_allocate_color_data(ctx, options[0]);
	if (ret < 0)
		goto out;

	type = (enum color_data_type)options[0];
	if (options[0] != COLOR_DATA_TYPE_FAKE_CIE) {
		dev_info(ctx->dev, "Set color data read_type %u dbv %u\n", type, options[1]);
		ret = gs_panel_validate_color_option(ctx, type, options[1]);
		if (ret < 0)
			goto out;

		mutex_lock(&ctx->mode_lock);
		ctx->color_data.ready = false;
		if (gs_panel_has_func(ctx, set_color_data_config))
			ret = ctx->desc->gs_panel_func->set_color_data_config(ctx, type,
									      options[1]);
		mutex_unlock(&ctx->mode_lock);
	} else {
		ret = gs_panel_set_fake_color_data(ctx, options, option_count);
	}

	if (ret != 0)
		goto out;

	ret = count;
out:
	kfree(buf_dup);
	return ret;
}

static ssize_t force_power_on_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;
	bool force_on;

	ret = kstrtobool(buf, &force_on);
	if (ret) {
		dev_err(dev, "invalid force_power_on value\n");
		return ret;
	}

	drm_modeset_lock(&ctx->bridge.base.lock, NULL);
	if (force_on && ctx->panel_state == GPANEL_STATE_OFF) {
		drm_panel_prepare(&ctx->base);
		ctx->panel_state = GPANEL_STATE_BLANK;
	}

	ctx->force_power_on = force_on;
	drm_modeset_unlock(&ctx->bridge.base.lock);

	return count;
}

static ssize_t force_power_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	u32 count;

	mutex_lock(&ctx->mode_lock);
	count = sysfs_emit(buf, "%d\n", ctx->force_power_on);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t power_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int err;
	u8 power_mode;

	if (!gs_is_panel_active(ctx)) {
		dev_warn(dev, "%s: panel is not enabled\n", __func__);
		return -EPERM;
	}

	mutex_lock(&ctx->mode_lock);
	err = mipi_dsi_dcs_read(dsi, MIPI_DCS_GET_POWER_MODE, &power_mode, sizeof(power_mode));
	mutex_unlock(&ctx->mode_lock);
	if (err <= 0) {
		if (err == 0)
			err = -ENODATA;
		dev_warn(dev, "Unable to read power mode register (%#02x: %d)\n",
			MIPI_DCS_GET_POWER_MODE, err);
		return err;
	}

	power_mode &= (MIPI_DSI_DCS_POWER_MODE_DISPLAY |
	    MIPI_DSI_DCS_POWER_MODE_NORMAL | MIPI_DSI_DCS_POWER_MODE_SLEEP);

	return sysfs_emit(buf, "%#02x\n", power_mode);
}

static ssize_t frame_rate_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;
	u16 frame_rate;

	ret = kstrtou16(buf, 0, &frame_rate);
	if (ret || frame_rate < 1 || frame_rate > 120) {
		dev_err(dev, "invalid frame rate value: %u\n", frame_rate);
		return ret;
	}

	if (!gs_is_panel_active(ctx)) {
		dev_warn(ctx->dev, "%s: panel is not enabled\n", __func__);
		return -EPERM;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->desc->gs_panel_func->set_frame_rate(ctx, frame_rate);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

/* assign new timeline's expected present timestamp for align sending CMD timing */
static ssize_t expected_present_time_ns_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;
	u64 pf_timestamp;

	ret = kstrtou64(buf, 0, &pf_timestamp);
	if (ret) {
		dev_err(dev, "invalid timeline present_ts input:%d\n", ret);
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->timestamps.timeline_expected_present_ts = pf_timestamp;
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t expected_present_time_ns_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	mutex_lock(&ctx->mode_lock);
	ret = sysfs_emit(buf, "%llu\n",
			ctx->timestamps.timeline_expected_present_ts);
	mutex_unlock(&ctx->mode_lock);

	return ret;
}

/* assign new timeline's frame interval for align sending CMD timing */
static ssize_t frame_interval_ns_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;
	u64 frame_interval;

	ret = kstrtou64(buf, 0, &frame_interval);
	if (ret) {
		dev_err(dev, "invalid frame_interval input:%d\n", ret);
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	if (frame_interval != 0)
		do_div(frame_interval, NSEC_PER_USEC);
	PANEL_ATRACE_INT("sysfs frame_interval", frame_interval);
	ctx->frame_interval_us = frame_interval;
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t frame_interval_ns_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	mutex_lock(&ctx->mode_lock);
	ret = sysfs_emit(buf, "%lu\n", (ctx->frame_interval_us * NSEC_PER_USEC));
	mutex_unlock(&ctx->mode_lock);

	return ret;
}

static DEVICE_ATTR_RO(serial_number);
static DEVICE_ATTR_RO(panel_extinfo);
static DEVICE_ATTR_RO(panel_name);
static DEVICE_ATTR_RO(panel_model);
static DEVICE_ATTR_RW(panel_idle);
static DEVICE_ATTR_RW(panel_need_handle_idle_exit);
static DEVICE_ATTR_RW(idle_delay_ms);
static DEVICE_ATTR_RW(op_hz);
static DEVICE_ATTR_RO(refresh_rate);
static DEVICE_ATTR_RW(refresh_ctrl);
static DEVICE_ATTR_RW(min_vrefresh);
static DEVICE_ATTR_RW(te2_timing);
static DEVICE_ATTR_RW(te2_lp_timing);
static DEVICE_ATTR_RO(time_in_state);
static DEVICE_ATTR_RO(available_disp_stats);
static DEVICE_ATTR_RO(te_rate_hz);
static DEVICE_ATTR_RO(te_option);
static DEVICE_ATTR_RW(te2_rate_hz);
static DEVICE_ATTR_RW(te2_option);
static DEVICE_ATTR_RO(power_state);
static DEVICE_ATTR_RO(error_count_te);
static DEVICE_ATTR_RO(error_count_unknown);
static DEVICE_ATTR_RW(color_data);
static DEVICE_ATTR_RW(force_power_on);
static DEVICE_ATTR_RO(power_mode);
static DEVICE_ATTR_WO(frame_rate);
static DEVICE_ATTR_RW(expected_present_time_ns);
static DEVICE_ATTR_RW(frame_interval_ns);
/* TODO(tknelms): re-implement below */
#if 0
static DEVICE_ATTR_WO(gamma);
static DEVICE_ATTR_RW(osc2_clk_khz);
static DEVICE_ATTR_RO(available_osc2_clk_khz);
#endif

static const struct attribute *panel_attrs[] = { &dev_attr_serial_number.attr,
						 &dev_attr_panel_extinfo.attr,
						 &dev_attr_panel_name.attr,
						 &dev_attr_panel_model.attr,
						 &dev_attr_panel_idle.attr,
						 &dev_attr_panel_need_handle_idle_exit.attr,
						 &dev_attr_idle_delay_ms.attr,
						 &dev_attr_op_hz.attr,
						 &dev_attr_refresh_rate.attr,
						 &dev_attr_refresh_ctrl.attr,
						 &dev_attr_min_vrefresh.attr,
						 &dev_attr_te2_timing.attr,
						 &dev_attr_te2_lp_timing.attr,
						 &dev_attr_te_rate_hz.attr,
						 &dev_attr_te_option.attr,
						 &dev_attr_te2_rate_hz.attr,
						 &dev_attr_te2_option.attr,
						 &dev_attr_power_state.attr,
						 &dev_attr_error_count_te.attr,
						 &dev_attr_error_count_unknown.attr,
						 &dev_attr_color_data.attr,
						 &dev_attr_force_power_on.attr,
						 &dev_attr_power_mode.attr,
						 &dev_attr_expected_present_time_ns.attr,
						 &dev_attr_frame_interval_ns.attr,
/* TODO(tknelms): re-implement below */
#if 0
						 &dev_attr_gamma.attr,
						 &dev_attr_osc2_clk_khz.attr,
						 &dev_attr_available_osc2_clk_khz.attr,
#endif
						 NULL };

int gs_panel_sysfs_create_files(struct device *dev, struct gs_panel *ctx)
{
	if (ctx->disp_stats.initialized) {
		if (sysfs_create_file(&dev->kobj, &dev_attr_time_in_state.attr))
			dev_err(ctx->dev, "unable to add time_in_state panel sysfs file\n");

		if (sysfs_create_file(&dev->kobj, &dev_attr_available_disp_stats.attr))
			dev_err(ctx->dev, "unable to add available_disp_stats sysfs file\n");
	}

	if (gs_panel_has_func(ctx, set_frame_rate)) {
		if (sysfs_create_file(&dev->kobj, &dev_attr_frame_rate.attr))
			dev_err(ctx->dev, "unable to add set_frame_rate sysfs file\n");
	}

	return sysfs_create_files(&dev->kobj, panel_attrs);
}

/* Backlight Sysfs Node */

static ssize_t hbm_mode_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bd);
	const struct gs_panel_mode *pmode;
	u32 hbm_mode;
	int ret;

	if (!gs_panel_has_func(ctx, set_hbm_mode)) {
		dev_err(ctx->dev, "HBM is not supported\n");
		return -ENOTSUPP;
	}

	mutex_lock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/
	pmode = ctx->current_mode;

	if (!gs_is_panel_active(ctx) || !pmode) {
		dev_err(ctx->dev, "panel is not enabled\n");
		ret = -EPERM;
		goto unlock;
	}

	if (pmode->gs_mode.is_lp_mode) {
		dev_dbg(ctx->dev, "hbm unsupported in LP mode\n");
		ret = -EPERM;
		goto unlock;
	}

	ret = kstrtouint(buf, 0, &hbm_mode);
	if (ret || (hbm_mode >= GS_HBM_STATE_MAX)) {
		dev_err(ctx->dev, "invalid hbm_mode value\n");
		goto unlock;
	}

	if (hbm_mode != ctx->hbm_mode) {
		ctx->desc->gs_panel_func->set_hbm_mode(ctx, hbm_mode);
		notify_panel_mode_changed(ctx);
	}

unlock:
	mutex_unlock(&ctx->mode_lock); /*TODO(b/267170999): MODE*/

	return ret ? ret : count;
}

static ssize_t hbm_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bd);

	return sysfs_emit(buf, "%u\n", ctx->hbm_mode);
}

static ssize_t dimming_on_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bd);
	bool dimming_on;
	int ret;

	if (!gs_is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	ret = kstrtobool(buf, &dimming_on);
	if (ret) {
		dev_err(ctx->dev, "invalid dimming_on value\n");
		return ret;
	}

	gs_panel_set_dimming(ctx, dimming_on);

	return count;
}

static ssize_t dimming_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bd);

	return sysfs_emit(buf, "%d\n", ctx->dimming_on);
}

static ssize_t local_hbm_mode_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bd);
	bool local_hbm_en;
	int ret;
	struct drm_crtc *crtc = get_gs_panel_connector_crtc(ctx);

	if (!gs_is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	if (!gs_panel_has_func(ctx, set_local_hbm_mode)) {
		dev_err(ctx->dev, "Local HBM is not supported\n");
		return -ENOTSUPP;
	}

	ret = kstrtobool(buf, &local_hbm_en);
	if (ret) {
		dev_err(ctx->dev, "invalid local_hbm_mode value\n");
		return ret;
	}

	if (crtc && !drm_crtc_vblank_get(crtc)) {
		struct drm_vblank_crtc vblank = crtc->dev->vblank[crtc->index];
		u32 delay_us = vblank.framedur_ns / 2000;

		drm_crtc_wait_one_vblank(crtc);
		drm_crtc_vblank_put(crtc);
		/* wait for 0.5 frame to send to ensure it is done in one frame */
		usleep_range(delay_us, delay_us + 10);
	}

	dev_info(ctx->dev, "%s: set LHBM to %d\n", __func__, local_hbm_en);
	mutex_lock(&ctx->mode_lock); /* TODO(b/267170999): MODE */
	ctx->lhbm.requested_state = local_hbm_en;
	panel_update_lhbm(ctx);
	mutex_unlock(&ctx->mode_lock); /* TODO(b/267170999): MODE */

	return count;
}

static ssize_t local_hbm_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bd);

	return sysfs_emit(buf, "%d\n", ctx->lhbm.effective_state);
}

static ssize_t local_hbm_max_timeout_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bd);
	int ret;

	ret = kstrtou32(buf, 0, &ctx->lhbm.max_timeout_ms);
	if (ret) {
		dev_err(ctx->dev, "invalid local_hbm_max_timeout_ms value\n");
		return ret;
	}

	return count;
}

static ssize_t local_hbm_max_timeout_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bd);

	return sysfs_emit(buf, "%d\n", ctx->lhbm.max_timeout_ms);
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	enum display_stats_state state;
	int rc, ret_cnt;

	mutex_lock(&ctx->bl_state_lock);
	state = gs_get_current_display_state_locked(ctx);
	mutex_unlock(&ctx->bl_state_lock);

	ret_cnt = sysfs_emit(buf, "%s", get_disp_state_str(state));
	rc = ret_cnt;

	if (rc > 0 && state != DISPLAY_STATE_OFF) {
		const struct gs_panel_mode *pmode;

		mutex_lock(&ctx->mode_lock);
		pmode = ctx->current_mode;
		mutex_unlock(&ctx->mode_lock);
		if (pmode) {
			ret_cnt = sysfs_emit_at(buf, ret_cnt, ": %ux%u@%d\n", pmode->mode.hdisplay,
						pmode->mode.vdisplay, gs_get_actual_vrefresh(ctx));
			if (ret_cnt > 0)
				rc += ret_cnt;
		} else {
			ret_cnt = sysfs_emit_at(buf, ret_cnt, "\n");
			if (ret_cnt > 0)
				rc += ret_cnt;
		}
	} else if (rc > 0) {
		ret_cnt = sysfs_emit_at(buf, ret_cnt, "\n");
		if (ret_cnt > 0)
			rc += ret_cnt;
	}

	dev_dbg(ctx->dev, "%s: %s\n", __func__, rc > 0 ? buf : "");

	return rc;
}

static ssize_t acl_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;
	u32 acl_mode;

	if (!gs_is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EAGAIN;
	}

	if (!gs_panel_has_func(ctx, set_acl_mode)) {
		dev_err(ctx->dev, "ACL is not supported\n");
		return -ENOTSUPP;
	}

	ret = kstrtouint(buf, 0, &acl_mode);
	if (ret || (acl_mode > ACL_ENHANCED)) {
		dev_err(dev, "invalid acl mode\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->acl_mode = acl_mode;
	ctx->desc->gs_panel_func->set_acl_mode(ctx, acl_mode);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t acl_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!gs_is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EAGAIN;
	}

	return sysfs_emit(buf, "%d\n", ctx->acl_mode);
}

static ssize_t ssc_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;
	bool ssc_en;

	if (!gs_panel_has_func(ctx, set_ssc_en)) {
		dev_err(ctx->dev, "SSC is not supported\n");
		return -ENOTSUPP;
	}

	if (!gs_is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EAGAIN;
	}

	ret = kstrtobool(buf, &ssc_en);
	if (ret) {
		dev_err(dev, "invalid SSC mode value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->desc->gs_panel_func->set_ssc_en(ctx, ssc_en);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t ssc_en_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct gs_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!gs_panel_has_func(ctx, set_ssc_en)) {
		dev_err(ctx->dev, "SSC is not supported\n");
		return -ENOTSUPP;
	}

	if (!gs_is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EAGAIN;
	}

	return sysfs_emit(buf, "%d\n", ctx->ssc_en);
}

static ssize_t als_table_store(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	ssize_t bl_num_ranges;
	char *buf_dup;
	u32 ranges[MAX_BL_RANGES] = { 0 };
	int ret = 0;
	u32 i;

	if (count == 0)
		return -EINVAL;

	buf_dup = kstrndup(buf, count, GFP_KERNEL);
	if (!buf_dup)
		return -ENOMEM;

	if (strlen(buf_dup) != count) {
		ret = -EINVAL;
		goto out;
	}

	bl_num_ranges = parse_u32_buf(buf_dup, count + 1, ranges, MAX_BL_RANGES);
	if (bl_num_ranges < 0) {
		dev_warn(ctx->dev, "error parsing als_table input buf (%ld)\n", bl_num_ranges);
		ret = bl_num_ranges;
		goto out;
	}

	mutex_lock(&ctx->bl_state_lock); /* TODO(b/267170999): BL */

	ctx->bl_notifier.num_ranges = bl_num_ranges;
	for (i = 0; i < ctx->bl_notifier.num_ranges; i++)
		ctx->bl_notifier.ranges[i] = ranges[i];

	mutex_unlock(&ctx->bl_state_lock); /* TODO(b/267170999): BL */

	ret = count;
out:
	kfree(buf_dup);
	return ret;
}

static ssize_t als_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	size_t len = 0;
	u32 i;

	mutex_lock(&ctx->bl_state_lock); /* TODO(b/267170999): BL */

	for (i = 0; i < ctx->bl_notifier.num_ranges; i++)
		len += sysfs_emit_at(buf, len, "%u ", ctx->bl_notifier.ranges[i]);

	mutex_unlock(&ctx->bl_state_lock); /* TODO(b/267170999): BL */

	len += sysfs_emit_at(buf, len, "\n");

	return len;
}

static ssize_t cabc_mode_store(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	ssize_t ret;
	u32 cabc_mode;

	if (!gs_panel_has_func(ctx, set_cabc_mode)) {
		dev_err(ctx->dev, "CABC is not supported\n");
		return -EOPNOTSUPP;
	}

	if (!gs_is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EAGAIN;
	}

	ret = kstrtouint(buf, 0, &cabc_mode);
	if (ret || (cabc_mode > GCABC_MOVIE_MODE)) {
		dev_err(ctx->dev, "invalid CABC mode value");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->cabc_mode = cabc_mode;
	ctx->desc->gs_panel_func->set_cabc_mode(ctx, cabc_mode);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t cabc_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);
	const char *mode;

	switch (ctx->cabc_mode) {
	case GCABC_OFF:
		mode = "OFF";
		break;
	case GCABC_UI_MODE:
		mode = "UI";
		break;
	case GCABC_STILL_MODE:
		mode = "STILL";
		break;
	case GCABC_MOVIE_MODE:
		mode = "MOVIE";
		break;
	default:
		dev_err(ctx->dev, "unknown CABC mode : %d\n", ctx->cabc_mode);
		return -EINVAL;
	}

	return sysfs_emit(buf, "%s\n", mode);
}

static ssize_t dim_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gs_panel *ctx = bl_get_data(bl);

	return sysfs_emit(buf, "%d\n", ctx->desc->brightness_desc->lower_min_brightness);
}

static DEVICE_ATTR_RW(hbm_mode);
static DEVICE_ATTR_RW(dimming_on);
static DEVICE_ATTR_RW(local_hbm_mode);
static DEVICE_ATTR_RW(local_hbm_max_timeout);
static DEVICE_ATTR_RO(state);
static DEVICE_ATTR_RW(acl_mode);
static DEVICE_ATTR_RW(ssc_en);
static DEVICE_ATTR_RW(als_table);
static DEVICE_ATTR_RW(cabc_mode);
static DEVICE_ATTR_RO(dim_brightness);

static struct attribute *bl_device_attrs[] = { &dev_attr_hbm_mode.attr,
					       &dev_attr_dimming_on.attr,
					       &dev_attr_local_hbm_mode.attr,
					       &dev_attr_local_hbm_max_timeout.attr,
					       &dev_attr_acl_mode.attr,
					       &dev_attr_state.attr,
					       &dev_attr_ssc_en.attr,
					       &dev_attr_als_table.attr,
					       &dev_attr_dim_brightness.attr,
					       NULL };
ATTRIBUTE_GROUPS(bl_device);

int gs_panel_sysfs_create_bl_files(struct device *bl_dev, struct gs_panel *ctx)
{
	if (gs_panel_has_func(ctx, set_cabc_mode)) {
		if (sysfs_create_file(&bl_dev->kobj, &dev_attr_cabc_mode.attr))
			dev_err(bl_dev, "unable to add set_cabc_mode sysfs file\n");
	}
	return sysfs_create_groups(&bl_dev->kobj, bl_device_groups);
}
