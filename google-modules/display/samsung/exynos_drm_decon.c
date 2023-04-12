// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_decon.c
 *
 * Copyright (C) 2018 Samsung Electronics Co.Ltd
 * Authors:
 *	Hyung-jun Kim <hyungjun07.kim@samsung.com>
 *	Seong-gyu Park <seongyu.park@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_vblank.h>
#include <drm/exynos_drm.h>

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>
#include <linux/console.h>
#include <linux/iommu.h>
#include <uapi/linux/sched/types.h>

#include <soc/google/exynos-cpupm.h>
#include <video/videomode.h>

#include <decon_cal.h>
#include <regs-decon.h>
#include <trace/dpu_trace.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_dpp.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"

struct decon_device *decon_drvdata[MAX_DECON_CNT];

#define decon_info(decon, fmt, ...)	\
pr_info("%s[%u]: "fmt, decon->dev->driver->name, decon->id, ##__VA_ARGS__)

#define decon_warn(decon, fmt, ...)	\
pr_warn("%s[%u]: "fmt, decon->dev->driver->name, decon->id, ##__VA_ARGS__)

#define decon_err(decon, fmt, ...)	\
pr_err("%s[%u]: "fmt, decon->dev->driver->name, decon->id, ##__VA_ARGS__)

#define decon_debug(decon, fmt, ...)	\
pr_debug("%s[%u]: "fmt, decon->dev->driver->name, decon->id, ##__VA_ARGS__)

#define SHADOW_UPDATE_TIMEOUT_US	(300 * USEC_PER_MSEC) /* 300ms */

static const struct of_device_id decon_driver_dt_match[] = {
	{.compatible = "samsung,exynos-decon"},
	{},
};
MODULE_DEVICE_TABLE(of, decon_driver_dt_match);

static void decon_seamless_mode_set(struct exynos_drm_crtc *exynos_crtc,
				    struct drm_crtc_state *old_crtc_state);
static int decon_request_te_irq(struct exynos_drm_crtc *exynos_crtc,
				const struct exynos_drm_connector_state *exynos_conn_state);
static bool decon_check_fs_pending_locked(struct decon_device *decon);

#define FRAME_TIMEOUT msecs_to_jiffies(100)

/* wait at least one frame time on top of common timeout */
static inline unsigned long fps_timeout(int fps)
{
	/* default to 60 fps, if fps is not provided */
	const int frame_time_ms = DIV_ROUND_UP(MSEC_PER_SEC, fps ? : 60);

	return msecs_to_jiffies(frame_time_ms) + FRAME_TIMEOUT;
}

void decon_dump(const struct decon_device *decon)
{
	int i;
	struct decon_device *d;
	struct drm_printer p = console_set_on_cmdline ?
		drm_debug_printer("[drm]") : drm_info_printer(decon->dev);

	for (i = 0; i < REGS_DECON_ID_MAX; ++i) {
		d = get_decon_drvdata(i);
		if (!d)
			continue;

		if (d->state != DECON_STATE_ON) {
			drm_printf(&p, "%s[%u]: DECON state is not On(%d)\n",
				d->dev->driver->name, d->id, d->state);
			continue;
		}

		__decon_dump(&p, d->id, &d->regs, d->config.dsc.enabled, d->dqe != NULL);
	}

	if (decon->state != DECON_STATE_ON)
		return;

	for (i = 0; i < decon->dpp_cnt; ++i)
		dpp_dump(&p, decon->dpp[i]);

	if (decon->rcd)
		rcd_dump(&p, decon->rcd);

	if (decon->cgc_dma)
		cgc_dump(&p, decon->cgc_dma);
}

static inline u32 win_start_pos(int x, int y)
{
	return (WIN_STRPTR_Y_F(y) | WIN_STRPTR_X_F(x));
}

static inline u32 win_end_pos(int x2, int y2)
{
	return (WIN_ENDPTR_Y_F(y2 - 1) | WIN_ENDPTR_X_F(x2 - 1));
}

/* ARGB value */
#define COLOR_MAP_VALUE			0x00340080

/*
 * This function can be used in cases where all windows are disabled
 * but need something to be rendered for display. This will make a black
 * frame via decon using a single window with color map enabled.
 */
static void decon_set_color_map(struct decon_device *decon, u32 win_id,
						u32 hactive, u32 vactive)
{
	struct decon_window_regs win_info;

	decon_debug(decon, "%s +\n", __func__);

	memset(&win_info, 0, sizeof(struct decon_window_regs));
	win_info.start_pos = win_start_pos(0, 0);
	win_info.end_pos = win_end_pos(hactive, vactive);
	win_info.start_time = 0;
	win_info.colormap = 0x000000; /* black */
	win_info.blend = DECON_BLENDING_NONE;
	decon_reg_set_window_control(decon->id, win_id, &win_info, true);
	decon_reg_update_req_window(decon->id, win_id);

	decon_debug(decon, "%s -\n", __func__);
}

static inline bool decon_is_effectively_active(const struct decon_device *decon)
{
	return decon->state == DECON_STATE_ON || decon->state == DECON_STATE_HIBERNATION;
}

static inline bool decon_is_te_enabled(const struct decon_device *decon)
{
	return (decon->config.mode.op_mode == DECON_COMMAND_MODE) &&
		(decon->config.mode.trig_mode == DECON_HW_TRIG);
}

void decon_enable_te_irq(struct decon_device *decon, bool enable)
{
	if (enable) {
		if (atomic_inc_return(&decon->te_ref) == 1)
			enable_irq(decon->irq_te);
	} else {
		int ret = atomic_dec_if_positive(&decon->te_ref);
		if (!ret)
			disable_irq_nosync(decon->irq_te);
		else if (ret < 0)
			WARN(1, "unbalanced te irq (%d)\n", ret);
	}
}

static int decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	if (!decon_is_effectively_active(decon)) {
		WARN(1, "decon is not ready");
		return -EINVAL;
	}

	decon_debug(decon, "%s +\n", __func__);

	hibernation_block(decon->hibernation);

	if (decon_is_te_enabled(decon))
		decon_enable_te_irq(decon, true);
	else /* if te is not enabled, use framestart interrupt to track vsyncs */
		enable_irq(decon->irq_fs);

	DPU_EVENT_LOG(DPU_EVT_VBLANK_ENABLE, decon->id, NULL);

	decon_debug(decon, "%s -\n", __func__);

	return 0;
}

static void decon_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	decon_debug(decon, "%s +\n", __func__);

	if (decon_is_te_enabled(decon))
		decon_enable_te_irq(decon, false);
	else /* if te is not enabled, we're using framestart interrupt to track vsyncs */
		disable_irq_nosync(decon->irq_fs);

	DPU_EVENT_LOG(DPU_EVT_VBLANK_DISABLE, decon->id, NULL);

	hibernation_unblock_enter(decon->hibernation);

	decon_debug(decon, "%s -\n", __func__);
}

static int decon_get_crtc_out_type(const struct drm_crtc_state *crtc_state)
{
	const struct drm_crtc *crtc = crtc_state->crtc;
	const struct drm_device *dev = crtc->dev;
	const struct drm_encoder *encoder;
	const struct dsim_device *dsim;
	int out_type = 0;

	drm_for_each_encoder_mask(encoder, dev, crtc_state->encoder_mask) {
		switch (encoder->encoder_type) {
		case DRM_MODE_ENCODER_VIRTUAL:
			/* if anything else is connected operate in cwb mode */
			if (!out_type)
				out_type = DECON_OUT_WB;
			break;
		case DRM_MODE_ENCODER_DSI:
			/* if wb is also connected, operate in dsi+cwb mode */
			out_type &= ~DECON_OUT_WB;

			if (out_type & ~DECON_OUT_DSI) {
				pr_err("Unable to support DSI along with out_type: 0x%x\n",
				       out_type);
				return -EINVAL;
			}

			dsim = encoder_to_dsim(encoder);
			if (dsim->dual_dsi != DSIM_DUAL_DSI_NONE) {
				out_type |= DECON_OUT_DSI;
			} else if (dsim->id == 0) {
				out_type |= DECON_OUT_DSI0;
			} else if (dsim->id == 1) {
				out_type |= DECON_OUT_DSI1;
			} else {
				pr_err("Invalid dsim id: %d\n", dsim->id);
				return -EINVAL;
			}
			break;
		default:
			pr_err("Unsupported encoder type: %d\n", encoder->encoder_type);
			return -ENOTSUPP;
		}
	}

	if (!out_type)
		return -EINVAL;

	return out_type;
}

static bool has_writeback_job(struct drm_crtc_state *new_crtc_state)
{
	int i;
	struct drm_atomic_state *state = new_crtc_state->state;
	struct drm_connector_state *conn_state;
	struct drm_connector *conn;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		if (!(new_crtc_state->connector_mask &
					drm_connector_mask(conn)))
			continue;

		if (wb_check_job(conn_state))
			return true;
	}
	return false;
}

static void decon_update_dsi_config(struct decon_config *config,
				    const struct drm_crtc_state *crtc_state,
				    const struct exynos_drm_connector_state *exynos_conn_state)
{
	const struct exynos_display_mode *exynos_mode = &exynos_conn_state->exynos_mode;
	bool is_vid_mode;

	config->dsc.enabled = exynos_mode->dsc.enabled;
	if (exynos_mode->dsc.enabled) {
		config->dsc.dsc_count = exynos_mode->dsc.dsc_count;
		config->dsc.slice_count = exynos_mode->dsc.slice_count;
		config->dsc.slice_height = exynos_mode->dsc.slice_height;
		config->dsc.slice_width = DIV_ROUND_UP(config->image_width,
						       config->dsc.slice_count);
		config->dsc.cfg = exynos_mode->dsc.cfg;
		config->dsc.delay_reg_init_us = exynos_mode->dsc.delay_reg_init_us;
		config->dsc.is_scrv4 = exynos_mode->dsc.is_scrv4;
	}

	is_vid_mode = (exynos_mode->mode_flags & MIPI_DSI_MODE_VIDEO) != 0;

	config->mode.op_mode = is_vid_mode ? DECON_VIDEO_MODE : DECON_COMMAND_MODE;

	if (!is_vid_mode && !exynos_mode->sw_trigger) {
		if (exynos_conn_state->te_from >= MAX_DECON_TE_FROM_DDI) {
			pr_warn("TE from DDI is not valid (%d)\n", exynos_conn_state->te_from);
		} else {
			config->mode.trig_mode = DECON_HW_TRIG;
			config->te_from = exynos_conn_state->te_from;
			pr_debug("TE from DDI%d\n", config->te_from);
		}
	}
}

static int decon_get_main_dsim_id(void)
{
	const struct dsim_device *dsim = exynos_get_dual_dsi(DSIM_DUAL_DSI_MAIN);

	if (!dsim) {
		pr_err("%s: fail to get dsim, suppose dsim0\n", __func__);
		return 0;
	}

	return dsim->id;
}

static void decon_update_config(struct decon_config *config,
				const struct drm_crtc_state *crtc_state,
				const struct exynos_drm_connector_state *exynos_conn_state)
{
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;

	config->image_width = mode->hdisplay;
	config->image_height = mode->vdisplay;

	config->out_type = decon_get_crtc_out_type(crtc_state);
	if (config->out_type == DECON_OUT_DSI) {
		config->mode.dsi_mode = DSI_MODE_DUAL_DSI;
		config->main_dsim_id = decon_get_main_dsim_id();
	} else if (config->out_type & (DECON_OUT_DSI0 | DECON_OUT_DSI1)) {
		config->mode.dsi_mode = DSI_MODE_SINGLE;
	} else {
		config->mode.dsi_mode = DSI_MODE_NONE;
	}

	/* defaults if not dsi, if video mode or if hw trigger is not configured properly */
	config->mode.trig_mode = DECON_SW_TRIG;
	config->te_from = MAX_DECON_TE_FROM_DDI;
	config->dsc.enabled = false;
	config->mode.op_mode = DECON_COMMAND_MODE;

	if (!exynos_conn_state) {
		pr_debug("%s: no private mode config\n", __func__);

		/* default bpc */
		config->out_bpc = 8;
		return;
	}

	if (config->mode.dsi_mode != DSI_MODE_NONE)
		decon_update_dsi_config(config, crtc_state, exynos_conn_state);

	config->out_bpc = exynos_conn_state->exynos_mode.bpc;
}

static bool decon_is_seamless_possible(const struct decon_device *decon,
				       const struct drm_crtc_state *crtc_state,
				       const struct exynos_drm_connector_state *exynos_conn_state)
{
	struct decon_config new_config = decon->config;

	decon_update_config(&new_config, crtc_state, exynos_conn_state);

	/* don't allow any changes in decon config */
	return !memcmp(&new_config, &decon->config, sizeof(new_config));
}

static int decon_check_modeset(struct exynos_drm_crtc *exynos_crtc,
			       struct drm_crtc_state *crtc_state)
{
	struct drm_atomic_state *state = crtc_state->state;
	const struct decon_device *decon = exynos_crtc->ctx;
	struct exynos_drm_crtc_state *exynos_crtc_state;
	const struct exynos_drm_connector_state *exynos_conn_state;
	struct drm_crtc *crtc = &exynos_crtc->base;
	const struct drm_crtc_state *old_crtc_state = drm_atomic_get_old_crtc_state(state, crtc);

	exynos_conn_state = crtc_get_exynos_connector_state(state, crtc_state);
	if (!exynos_conn_state)
		return 0;

	/* only decon0 supports more than 1 dsc */
	if (decon->id != 0) {
		const struct exynos_display_mode *mode_priv = &exynos_conn_state->exynos_mode;

		if (mode_priv->dsc.enabled && (mode_priv->dsc.dsc_count > 1)) {
			decon_err(decon, "cannot support %d dsc\n", mode_priv->dsc.dsc_count);
			return -EINVAL;
		}
	}

	if (exynos_conn_state->seamless_possible && !crtc_state->connectors_changed &&
	    drm_atomic_crtc_effectively_active(old_crtc_state) && crtc_state->active) {
		if (!decon_is_seamless_possible(decon, crtc_state, exynos_conn_state)) {
			decon_warn(decon, "seamless not possible for mode %s\n",
				   crtc_state->adjusted_mode.name);
		} else {
			exynos_crtc_state = to_exynos_crtc_state(crtc_state);
			exynos_crtc_state->seamless_mode_changed = true;
			crtc_state->mode_changed = false;

			decon_debug(decon, "switch to mode %s can be seamless\n",
				    crtc_state->adjusted_mode.name);
		}
	}

	return 0;
}

static int _decon_handover_check(struct exynos_drm_crtc *exynos_crtc,
				 struct drm_crtc_state *crtc_state)
{
	const struct decon_device *decon = exynos_crtc->ctx;
	struct exynos_drm_crtc_state *exynos_crtc_state = to_exynos_crtc_state(crtc_state);
	unsigned long win_mask = 0;
	u32 ch;
	int i, j, ret;
	bool found_handover_dpp = false;

	if (exynos_crtc_state->planes_updated) {
		drm_info(decon, "%s: planes updated on commit, skipping handover\n", __func__);
		return 0;
	}

	for (i = 0; i < MAX_WIN_PER_DECON; ++i) {
		ret = decon_reg_get_win_ch(decon->id, i, &ch);
		if (ret)
			continue;

		decon_debug(decon, "%s: win=%d enabled dpp_ch=%d\n", __func__, i, ch);
		win_mask = BIT(i);

		for (j = 0; j < decon->dpp_cnt; ++j) {
			struct dpp_device *dpp = decon->dpp[j];

			if (dpp->id != ch)
				continue;

			if ((dpp->decon_id >= 0) && (dpp->decon_id != decon->id)) {
				decon_warn(decon, "%s: dpp is owned by decon #%d\n", __func__,
					   dpp->decon_id);
				continue;
			}

			dpp->state = DPP_STATE_HANDOVER;
			dpp->win_id = i;
			dpp->decon_id = decon->id;
			dpp->is_win_connected = true;
			found_handover_dpp = true;
		}
	}

	decon_debug(decon, "%s: final win_mask=0x%lx\n", __func__, win_mask);

	if (!win_mask) {
		drm_warn(decon, "%s: handover memory defined, but no windows attached\n", __func__);
		return -ENOENT;
	}

	if (!found_handover_dpp) {
		drm_warn(decon, "%s: handover memory defined, but cannot find handover dpp\n",
				__func__);
		return -EBUSY;
	}

	return 0;
}


static int decon_atomic_check(struct exynos_drm_crtc *exynos_crtc,
			      struct drm_crtc_state *crtc_state)
{
	const struct decon_device *decon = exynos_crtc->ctx;
	const bool is_wb = has_writeback_job(crtc_state);
	bool is_swb;
	struct exynos_drm_crtc_state *exynos_crtc_state = to_exynos_crtc_state(crtc_state);
	int out_type;
	int ret = 0;

	if (exynos_crtc_state->bypass && !crtc_state->self_refresh_active) {
		decon_err(decon, "bypass mode only supported in self refresh\n");
		return -EINVAL;
	}

	if (crtc_state->mode_changed) {
		out_type = decon_get_crtc_out_type(crtc_state);

		if (out_type < 0) {
			decon_err(decon, "unsupported decon output (%d)\n", out_type);
			return out_type;
		}
		ret = decon_check_modeset(exynos_crtc, crtc_state);
	} else {
		out_type = decon->config.out_type;
	}

	is_swb = out_type == DECON_OUT_WB;
	if (is_wb)
		exynos_crtc_state->wb_type = is_swb ? EXYNOS_WB_SWB : EXYNOS_WB_CWB;
	else
		exynos_crtc_state->wb_type = EXYNOS_WB_NONE;

	if (is_swb)
		crtc_state->no_vblank = true;

	/*
	 * toggle hibernation during atomic check runs so that hibernation
	 * is pushed out (if needed) ahead of commit
	 */
	if (crtc_state->active) {
		hibernation_block(decon->hibernation);
		hibernation_unblock_enter(decon->hibernation);

		if (decon->state == DECON_STATE_HANDOVER)
			ret = _decon_handover_check(exynos_crtc, crtc_state);
	}

	return ret;
}

static void decon_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	decon_debug(decon, "%s +\n", __func__);
	DPU_EVENT_LOG(DPU_EVT_ATOMIC_BEGIN, decon->id, NULL);
	decon_reg_wait_update_done_and_mask(decon->id, &decon->config.mode,
			SHADOW_UPDATE_TIMEOUT_US);
	decon_debug(decon, "%s -\n", __func__);
}

static int decon_get_win_id(const struct drm_crtc_state *crtc_state, int zpos)
{
	const struct exynos_drm_crtc_state *exynos_crtc_state = to_exynos_crtc_state(crtc_state);
	const unsigned long win_mask = exynos_crtc_state->reserved_win_mask;
	int bit, i = 0;

	for_each_set_bit(bit, &win_mask, MAX_WIN_PER_DECON) {
		if (i == zpos)
			return bit;
		i++;
	}

	return -EINVAL;
}

static bool decon_is_win_used(const struct drm_crtc_state *crtc_state, int win_id)
{
	const struct exynos_drm_crtc_state *exynos_crtc_state = to_exynos_crtc_state(crtc_state);
	const unsigned long win_mask = exynos_crtc_state->visible_win_mask;

	if (win_id > MAX_WIN_PER_DECON)
		return false;

	return (BIT(win_id) & win_mask) != 0;
}

static void decon_disable_win(struct decon_device *decon, int win_id)
{
	const struct drm_crtc *crtc = &decon->crtc->base;

	decon_debug(decon, "disabling winid:%d\n", win_id);

	/*
	 * When disabling the plane, previously connected window (win_id) should be
	 * disabled, not the newly requested one. Only disable the old window if it
	 * was previously connected and it's not going to be used by any other plane.
	 */
	if ((win_id < MAX_WIN_PER_DECON) && !decon_is_win_used(crtc->state, win_id))
		decon_reg_set_win_enable(decon->id, win_id, 0);
}

static void _dpp_disable(struct dpp_device *dpp)
{
	if (dpp->disable)
		dpp->disable(dpp);
	dpp->is_win_connected = false;
}

static void decon_update_plane(struct exynos_drm_crtc *exynos_crtc,
			       struct exynos_drm_plane *exynos_plane)
{
	const struct drm_plane_state *plane_state = exynos_plane->base.state;
	struct exynos_drm_plane_state *exynos_plane_state =
		to_exynos_plane_state(plane_state);
	const struct drm_crtc_state *crtc_state = exynos_crtc->base.state;
	const struct exynos_drm_crtc_state *exynos_crtc_state =
					to_exynos_crtc_state(crtc_state);
	struct dpp_device *dpp = plane_to_dpp(exynos_plane);
	struct decon_device *decon = exynos_crtc->ctx;
	struct decon_window_regs win_info;
	unsigned int zpos;
	int win_id;
	bool is_colormap = false;
	u16 hw_alpha;

	decon_debug(decon, "%s +\n", __func__);

	dpp->decon_id = decon->id;

	if (test_bit(DPP_ATTR_RCD, &dpp->attr)) {
		decon_debug(decon, "%s -\n", __func__);
		dpp->update(dpp, exynos_plane_state);
		dpp->win_id = MAX_WIN_PER_DECON;
		return;
	}

	zpos = plane_state->normalized_zpos;

	if (!dpp->is_win_connected || crtc_state->zpos_changed) {
		win_id = decon_get_win_id(exynos_crtc->base.state, zpos);
		decon_debug(decon, "new win_id=%d zpos=%d mask=0x%x\n",
			    win_id, zpos, crtc_state->plane_mask);
	} else {
		win_id = dpp->win_id;
		decon_debug(decon, "reuse existing win_id=%d zpos=%d mask=0x%x\n",
			    win_id, zpos, crtc_state->plane_mask);
	}

	if (WARN(win_id < 0 || win_id > MAX_WIN_PER_DECON,
		 "couldn't find win id (%d) for zpos=%d plane_mask=0x%x\n",
		 win_id, zpos, crtc_state->plane_mask))
		return;

	memset(&win_info, 0, sizeof(struct decon_window_regs));

	is_colormap = plane_state->fb && exynos_drm_fb_is_colormap(plane_state->fb);
	if (is_colormap)
		win_info.colormap = exynos_plane_state->colormap;

	win_info.start_pos = win_start_pos(exynos_plane_state->base.dst.x1,
					exynos_plane_state->base.dst.y1);
	win_info.end_pos = win_end_pos(exynos_plane_state->base.dst.x2,
					exynos_plane_state->base.dst.y2);
	win_info.start_time = 0;

	win_info.ch = dpp->id; /* DPP's id is DPP channel number */

	hw_alpha = DIV_ROUND_CLOSEST(plane_state->alpha * EXYNOS_PLANE_ALPHA_MAX,
			DRM_BLEND_ALPHA_OPAQUE);
	win_info.plane_alpha = hw_alpha;
	win_info.blend = plane_state->pixel_blend_mode;
	win_info.in_bpc = exynos_crtc_state->in_bpc;

	if (zpos == 0 && hw_alpha == EXYNOS_PLANE_ALPHA_MAX)
		win_info.blend = DRM_MODE_BLEND_PIXEL_NONE;

	/* disable previous window if zpos has changed */
	if (dpp->win_id != win_id)
		decon_disable_win(decon, dpp->win_id);

	decon_reg_set_window_control(decon->id, win_id, &win_info, is_colormap);

	if (!is_colormap) {
		dpp->update(dpp, exynos_plane_state);
		dpp->is_win_connected = true;
	} else {
		_dpp_disable(dpp);
	}

	dpp->win_id = win_id;

	DPU_EVENT_LOG(DPU_EVT_PLANE_UPDATE, decon->id, dpp);
	decon_debug(decon, "plane idx[%d]: alpha(0x%x) hw alpha(0x%x)\n",
			drm_plane_index(&exynos_plane->base), plane_state->alpha,
			hw_alpha);
	decon_debug(decon, "blend_mode(%d) color(%s:0x%x)\n", win_info.blend,
			is_colormap ? "enable" : "disable", win_info.colormap);
	decon_debug(decon, "%s -\n", __func__);
}

static void decon_disable_plane(struct exynos_drm_crtc *exynos_crtc,
				struct exynos_drm_plane *exynos_plane)
{
	struct decon_device *decon = exynos_crtc->ctx;
	struct dpp_device *dpp = plane_to_dpp(exynos_plane);

	decon_debug(decon, "%s +\n", __func__);

	decon_disable_win(decon, dpp->win_id);
	_dpp_disable(dpp);

	DPU_EVENT_LOG(DPU_EVT_PLANE_DISABLE, decon->id, dpp);
	decon_debug(decon, "%s -\n", __func__);
}

static void decon_send_vblank_event_locked(struct decon_device *decon)
{
	struct drm_crtc *crtc = &decon->crtc->base;
	struct drm_device *dev = crtc->dev;

	if (!decon->event)
		return;

	spin_lock(&dev->event_lock);
	drm_send_event_locked(dev, &decon->event->base);
	spin_unlock(&dev->event_lock);

	drm_crtc_vblank_put(crtc);

	decon->event = NULL;
}

void decon_force_vblank_event(struct decon_device *decon)
{
	unsigned long flags;

	spin_lock_irqsave(&decon->slock, flags);
	decon_send_vblank_event_locked(decon);
	spin_unlock_irqrestore(&decon->slock, flags);
}

static void decon_arm_event_locked(struct exynos_drm_crtc *exynos_crtc)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct decon_device *decon = exynos_crtc->ctx;
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (!event)
		return;

	crtc->state->event = NULL;

	/* in the rare case that event wasn't signaled before, signal it now */
	if (WARN_ON(decon->event))
		decon_send_vblank_event_locked(decon);

	WARN_ON(drm_crtc_vblank_get(crtc) != 0);
	decon->event = event;
}

#define VSYNC_PERIOD_VARIANCE_NS		2000000
static void decon_wait_earliest_process_time(
		const struct exynos_drm_crtc_state *old_exynos_crtc_state,
		const struct exynos_drm_crtc_state *new_exynos_crtc_state)
{
	const struct drm_crtc_state *old_crtc_state = &old_exynos_crtc_state->base;
	const struct drm_crtc_state *new_crtc_state = &new_exynos_crtc_state->base;
	int32_t vrefresh, vsync_period_ns;
	ktime_t earliest_process_time, now;

	vrefresh = drm_mode_vrefresh(&old_crtc_state->mode);
	if (vrefresh == 0) {
		/* decon just be enabled */
		vrefresh = drm_mode_vrefresh(&new_crtc_state->mode);
	}
	vsync_period_ns = mult_frac(1000, 1000 * 1000, vrefresh);
	if (ktime_compare(new_exynos_crtc_state->expected_present_time,
				vsync_period_ns - VSYNC_PERIOD_VARIANCE_NS) <= 0) {
		return;
	}

	earliest_process_time = ktime_sub_ns(new_exynos_crtc_state->expected_present_time,
					vsync_period_ns - VSYNC_PERIOD_VARIANCE_NS);
	now = ktime_get();

	if (ktime_after(earliest_process_time, now)) {
		int32_t max_delay_us = (10 * vsync_period_ns) / 1000;
		int32_t delay_until_process;

		DPU_ATRACE_BEGIN("wait for earliest present time");

		delay_until_process = (int32_t)ktime_us_delta(earliest_process_time, now);
		if (delay_until_process > max_delay_us) {
			delay_until_process = max_delay_us;
			pr_warn("expected present time seems incorrect(now %llu, earliest %llu)\n",
					now, earliest_process_time);
		}
		usleep_range(delay_until_process, delay_until_process + 10);

		DPU_ATRACE_END("wait for earliest process time");
	}
}

static void decon_atomic_flush(struct exynos_drm_crtc *exynos_crtc,
		struct drm_crtc_state *old_crtc_state)
{
	struct decon_device *decon = exynos_crtc->ctx;
	struct drm_crtc_state *new_crtc_state = exynos_crtc->base.state;
	struct exynos_drm_crtc_state *new_exynos_crtc_state =
					to_exynos_crtc_state(new_crtc_state);
	struct exynos_drm_crtc_state *old_exynos_crtc_state =
					to_exynos_crtc_state(old_crtc_state);
	struct exynos_dqe *dqe = decon->dqe;
	struct exynos_partial *partial = decon->partial;
	u32 width, height;
	unsigned long flags;

	decon_debug(decon, "%s +\n", __func__);

	if (new_exynos_crtc_state->wb_type == EXYNOS_WB_NONE &&
			decon->config.out_type == DECON_OUT_WB)
		return;

	if (new_exynos_crtc_state->skip_update) {
		/* for seamless mode change, change pipeline but skip update from decon */
		if (new_exynos_crtc_state->seamless_mode_changed)
			decon_seamless_mode_set(exynos_crtc, old_crtc_state);

		/*
		 * during skip update, send vblank event on next vsync instead of frame start
		 * when it comes to video mode, vblank event is handled at fs_irq_handler.
		 * If fb handover is enabled, vblank event should be handled once because
		 * fs irq could be started after decon start by calling decon_reg_start().
		 */
		if (!new_crtc_state->no_vblank) {
			exynos_crtc_handle_event(exynos_crtc);
			if (decon->fb_handover.rmem) {
				decon_force_vblank_event(decon);
				drm_crtc_handle_vblank(&decon->crtc->base);
			}
		}

		return;
	}

	if (new_exynos_crtc_state->wb_type == EXYNOS_WB_CWB)
		decon_reg_set_cwb_enable(decon->id, true);

	/* if there are no dpp planes attached, enable colormap as fallback */
	if ((new_crtc_state->plane_mask & ~exynos_crtc->rcd_plane_mask) == 0) {
		const int win_id = decon_get_win_id(new_crtc_state, 0);

		if (win_id < 0) {
			decon_warn(decon, "unable to get free win_id=%d mask=0x%x\n",
				   win_id, new_exynos_crtc_state->reserved_win_mask);
			return;
		}
		decon_debug(decon, "no planes, enable color map win_id=%d\n", win_id);

		/*
		 * TODO: window id needs to be unique when using dual display, current hack is to
		 * use decon id, but it could conflict if planes are assigned to other display
		 */
		decon_set_color_map(decon, win_id, decon->config.image_width,
				decon->config.image_height);
	}

	decon->config.in_bpc = new_exynos_crtc_state->in_bpc;
	decon_reg_set_bpc_and_dither_path(decon->id, &decon->config);
	decon_debug(decon, "in/out/force bpc(%d/%d/%d)\n",
			new_exynos_crtc_state->in_bpc, decon->config.out_bpc,
			new_exynos_crtc_state->force_bpc);

	if (dqe && (new_crtc_state->color_mgmt_changed || !dqe->initialized ||
		    dqe->force_atc_config.dirty)) {
		if (partial && new_exynos_crtc_state->partial) {
			width = drm_rect_width(
					&new_exynos_crtc_state->partial_region);
			height = drm_rect_height(
					&new_exynos_crtc_state->partial_region);
		} else {
			width = decon->config.image_width;
			height = decon->config.image_height;
		}
		exynos_dqe_update(dqe, &new_exynos_crtc_state->dqe,
				width, height);
	}

	if (partial)
		exynos_partial_update(partial, &old_exynos_crtc_state->partial_region,
				&new_exynos_crtc_state->partial_region);

	decon_reg_all_win_shadow_update_req(decon->id);

	if (new_exynos_crtc_state->seamless_mode_changed)
		decon_seamless_mode_set(exynos_crtc, old_crtc_state);

	decon_wait_earliest_process_time(old_exynos_crtc_state, new_exynos_crtc_state);

	spin_lock_irqsave(&decon->slock, flags);
	decon_reg_start(decon->id, &decon->config);
	atomic_inc(&decon->frames_pending);
	if (!new_crtc_state->no_vblank)
		decon_arm_event_locked(exynos_crtc);
	spin_unlock_irqrestore(&decon->slock, flags);

	DPU_EVENT_LOG(DPU_EVT_ATOMIC_FLUSH, decon->id, NULL);

	decon_debug(decon, "%s -\n", __func__);
}

static void decon_print_config_info(struct decon_device *decon)
{
	char *str_output = NULL;
	char *str_trigger = NULL;

	if (decon->config.mode.trig_mode == DECON_HW_TRIG)
		str_trigger = "hw trigger.";
	else if (decon->config.mode.trig_mode == DECON_SW_TRIG)
		str_trigger = "sw trigger.";
	if (decon->config.mode.op_mode == DECON_VIDEO_MODE)
		str_trigger = "";

	if (decon->config.out_type == DECON_OUT_DSI)
		str_output = "Dual DSI";
	else if (decon->config.out_type & DECON_OUT_DSI0)
		str_output = "DSI0";
	else if  (decon->config.out_type & DECON_OUT_DSI1)
		str_output = "DSI1";
	else if  (decon->config.out_type & DECON_OUT_DP0)
		str_output = "DP0";
	else if  (decon->config.out_type & DECON_OUT_DP1)
		str_output = "DP1";
	else if  (decon->config.out_type & DECON_OUT_WB)
		str_output = "WB";

	decon_info(decon, "%s mode. %s %s output.(%dx%d@%dhz)\n",
			decon->config.mode.op_mode ? "command" : "video",
			str_trigger, str_output,
			decon->config.image_width, decon->config.image_height,
			decon->bts.fps);
}

static void decon_enable_irqs(struct decon_device *decon)
{
	decon_reg_set_interrupts(decon->id, 1);

	enable_irq(decon->irq_fd);
	enable_irq(decon->irq_ext);
	if (decon_is_te_enabled(decon))
		enable_irq(decon->irq_fs);
	if (decon->irq_ds >= 0)
		enable_irq(decon->irq_ds);
	if (decon->irq_de >= 0)
		enable_irq(decon->irq_de);
}

static void _decon_enable_locked(struct decon_device *decon)
{
	decon_reg_init(decon->id, &decon->config);
	decon_enable_irqs(decon);
}

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
static void _decon_mode_update_bts_handover(struct decon_device *decon,
					    const struct drm_display_mode *mode)
{
	int i, j;
	struct dpu_bts_win_config *config;

	decon_debug(decon, "%s: configure bts for handover\n", __func__);

	for (i = 0, j = 0; i < decon->dpp_cnt; i++) {
		struct dpp_device *dpp = decon->dpp[i];

		if (dpp->state != DPP_STATE_HANDOVER)
			continue;

		config = &decon->bts.win_config[j];
		if (config->state != DPU_WIN_STATE_DISABLED) {
			decon_warn(decon, "win config[%d] set during handover\n", j);
			return;
		}

		memset(config, 0, sizeof(*config));

		config->state = DPU_WIN_STATE_BUFFER;
		config->src_w = mode->hdisplay;
		config->src_h = mode->vdisplay;
		config->dst_w = mode->hdisplay;
		config->dst_h = mode->vdisplay;
		config->format = DRM_FORMAT_ARGB8888;
		config->dpp_id = dpp->id;
		j++;
	}
}

static void decon_mode_update_bts(struct decon_device *decon,
				const struct drm_display_mode *mode,
				const unsigned int vblank_usec)
{
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	decon->bts.vbp = vm.vback_porch;
	decon->bts.vfp = vm.vfront_porch;
	decon->bts.vsa = vm.vsync_len;
	decon->bts.fps = drm_mode_vrefresh(mode);
	decon->bts.vblank_usec = vblank_usec;

	decon->config.image_width = mode->hdisplay;
	decon->config.image_height = mode->vdisplay;

	decon_debug(decon, "update decon bts config for mode: %dx%dx%d\n",
		    mode->hdisplay, mode->vdisplay, decon->bts.fps);

	atomic_set(&decon->bts.delayed_update, 0);

	if (decon->state == DECON_STATE_HANDOVER)
		_decon_mode_update_bts_handover(decon, mode);
}

static void decon_seamless_mode_bts_update(struct decon_device *decon,
					const struct drm_display_mode *mode,
					const unsigned int vblank_usec)
{
	DPU_ATRACE_BEGIN(__func__);

	decon_debug(decon, "seamless mode change from %dhz to %dhz\n",
		    decon->bts.fps, drm_mode_vrefresh(mode));

	/*
	 * when going from high->low refresh rate need to run with the higher fps while the
	 * switch takes effect in display, this could happen within 2 vsyncs in the worst case
	 *
	 * TODO: change to 3 to extend the time of higher fps due to b/196466885. Restore to
	 * 2 once the issue is clarified.
	 */
	if (decon->bts.fps > drm_mode_vrefresh(mode)) {
		decon->bts.pending_vblank_usec = vblank_usec;
		atomic_set(&decon->bts.delayed_update, 3);
	} else {
		decon_mode_update_bts(decon, mode, vblank_usec);
	}
	DPU_ATRACE_END(__func__);
}

#define DEFAULT_VBLANK_USEC	100

static unsigned int decon_get_vblank_usec(const struct drm_crtc_state *crtc_state,
					const struct drm_atomic_state *old_state)
{
	const struct exynos_drm_connector_state *exynos_conn_state =
			crtc_get_exynos_connector_state(old_state, crtc_state);

	if (WARN_ON(!exynos_conn_state))
		return DEFAULT_VBLANK_USEC;

	return exynos_conn_state->exynos_mode.vblank_usec;
}

void decon_mode_bts_pre_update(struct decon_device *decon,
				const struct drm_crtc_state *crtc_state,
				const struct drm_atomic_state *old_state)
{
	const struct exynos_drm_crtc_state *exynos_crtc_state = to_exynos_crtc_state(crtc_state);

	if (exynos_crtc_state->seamless_mode_changed) {
		unsigned int vblank_usec = decon_get_vblank_usec(crtc_state, old_state);

		decon_seamless_mode_bts_update(decon, &crtc_state->adjusted_mode, vblank_usec);
	} else if (drm_atomic_crtc_needs_modeset(crtc_state)) {
		unsigned int vblank_usec = decon_get_vblank_usec(crtc_state, old_state);

		decon_mode_update_bts(decon, &crtc_state->adjusted_mode, vblank_usec);
	} else if (!atomic_dec_if_positive(&decon->bts.delayed_update)) {
		decon_mode_update_bts(decon, &crtc_state->mode, decon->bts.pending_vblank_usec);
	}

	decon->bts.ops->calc_bw(decon);
	decon->bts.ops->update_bw(decon, false);
}
#endif

static void decon_seamless_mode_set(struct exynos_drm_crtc *exynos_crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	struct drm_crtc *crtc = &exynos_crtc->base;
	struct decon_device *decon = exynos_crtc->ctx;
	struct drm_crtc_state *crtc_state = crtc->state;
	struct drm_atomic_state *old_state = old_crtc_state->state;
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	struct drm_display_mode *mode, *adjusted_mode;
	int i;

	mode = &crtc_state->mode;
	adjusted_mode = &crtc_state->adjusted_mode;

	decon_debug(decon, "seamless mode set to %s\n", mode->name);

	for_each_new_connector_in_state(old_state, conn, conn_state, i) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_encoder *encoder;
		struct drm_bridge *bridge;

		if (!(crtc_state->connector_mask & drm_connector_mask(conn)))
			continue;

		if (!conn_state->best_encoder)
			continue;

		encoder = conn_state->best_encoder;
		funcs = encoder->helper_private;

		bridge = drm_bridge_chain_get_first_bridge(encoder);
		drm_bridge_chain_mode_set(bridge, mode, adjusted_mode);

		if (funcs && funcs->atomic_mode_set)
			funcs->atomic_mode_set(encoder, crtc_state, conn_state);
		else if (funcs && funcs->mode_set)
			funcs->mode_set(encoder, mode, adjusted_mode);
	}
}

static int _decon_reinit_locked(struct decon_device *decon)
{
	int i;

	for (i = 0; i < MAX_WIN_PER_DECON; ++i)
		decon_reg_set_win_enable(decon->id, i, 0);

	for (i = 0; i < decon->dpp_cnt; ++i) {
		struct dpp_device *dpp = decon->dpp[i];

		if (dpp->state == DPP_STATE_HANDOVER)
			continue;

		if ((dpp->decon_id >= 0) && (dpp->decon_id != decon->id))
			continue;

		_dpp_disable(dpp);

		if (dpp->win_id < MAX_WIN_PER_DECON) {
			dpp->win_id = 0xFF;
			dpp->dbg_dma_addr = 0;
		}
	}

	if (decon->rcd)
		_dpp_disable(decon->rcd);

	return 0;
}

static void _decon_stop_locked(struct decon_device *decon, bool reset, u32 vrefresh)
{
	int i;
	const u32 fps = min(decon->bts.fps, vrefresh) ? : 60;

	decon_debug(decon, "%s: reset=%d\n", __func__, reset);

	/*
	 * Make sure all window connections are disabled when getting disabled,
	 * in case there are any stale mappings.
	 */
	for (i = 0; i < MAX_WIN_PER_DECON; ++i)
		decon->bts.win_config[i].state = DPU_WIN_STATE_DISABLED;

	decon->bts.rcd_win_config.win.state = DPU_WIN_STATE_DISABLED;
	decon->bts.rcd_win_config.dma_addr = 0;

	_decon_reinit_locked(decon);

	decon_reg_stop(decon->id, &decon->config, reset, fps);

	if (reset && decon->dqe)
		exynos_dqe_reset(decon->dqe);
}

static void decon_exit_hibernation(struct decon_device *decon)
{
	unsigned long flags;

	if (decon->state != DECON_STATE_HIBERNATION)
		return;

	DPU_EVENT_LOG(DPU_EVT_EXIT_HIBERNATION_IN, decon->id, NULL);
	DPU_ATRACE_BEGIN(__func__);
	decon_debug(decon, "%s +\n", __func__);

	if (pm_runtime_get_sync(decon->dev) < 0)
		decon_err(decon, "%s: failed to pm_runtime_get_sync\n", __func__);

	spin_lock_irqsave(&decon->slock, flags);
	_decon_enable_locked(decon);
	exynos_dqe_restore_lpd_data(decon->dqe);
	if (decon->partial)
		exynos_partial_restore(decon->partial);
	decon->state = DECON_STATE_ON;
	spin_unlock_irqrestore(&decon->slock, flags);

	decon_debug(decon, "%s -\n", __func__);
	DPU_ATRACE_END(__func__);
	DPU_EVENT_LOG(DPU_EVT_EXIT_HIBERNATION_OUT, decon->id, NULL);
}

static void decon_wait_for_te(struct decon_device *decon, int vrefresh)
{
	unsigned int te_period_ms = DIV_ROUND_UP(MSEC_PER_SEC, vrefresh);

	reinit_completion(&decon->te_rising);

	DPU_ATRACE_BEGIN(__func__);

	/* Wait for next TE rising or one TE period */
	if (!wait_for_completion_timeout(&decon->te_rising, te_period_ms))
		decon_debug(decon, "%s: exceed 1 TE period for %dhz\n",
			    __func__, vrefresh);

	DPU_ATRACE_END(__func__);
}

static void decon_enable(struct exynos_drm_crtc *exynos_crtc, struct drm_crtc_state *old_crtc_state)
{
	const struct drm_crtc_state *crtc_state = exynos_crtc->base.state;
	struct exynos_drm_crtc_state *old_exynos_crtc_state = to_exynos_crtc_state(old_crtc_state);
	struct decon_device *decon = exynos_crtc->ctx;
	int vrefresh = drm_mode_vrefresh(&old_crtc_state->mode);
	unsigned long flags;

	if (decon->state == DECON_STATE_ON) {
		decon_info(decon, "already enabled(%d)\n", decon->state);
		return;
	}

	DPU_ATRACE_BEGIN(__func__);

	if (decon->state == DECON_STATE_HIBERNATION) {
		WARN_ON(!old_crtc_state->self_refresh_active ||
			crtc_state->mode_changed || crtc_state->connectors_changed);

		if (old_exynos_crtc_state->bypass) {
			spin_lock_irqsave(&decon->slock, flags);
			_decon_stop_locked(decon, true, vrefresh);
			spin_unlock_irqrestore(&decon->slock, flags);
		}

		decon_exit_hibernation(decon);
		goto ret;
	}

	decon_info(decon, "%s +\n", __func__);

	if (crtc_state->mode_changed || crtc_state->connectors_changed) {
		const struct drm_atomic_state *state = old_crtc_state->state;
		const struct exynos_drm_connector_state *exynos_conn_state =
			crtc_get_exynos_connector_state(state, crtc_state);

		decon_update_config(&decon->config, crtc_state, exynos_conn_state);
		DPU_EVENT_LOG(DPU_EVT_DECON_UPDATE_CONFIG, decon->id, NULL);

		if (decon_is_te_enabled(decon))
			decon_request_te_irq(exynos_crtc, exynos_conn_state);
	}

	pm_runtime_get_sync(decon->dev);

	spin_lock_irqsave(&decon->slock, flags);
	if (decon->state == DECON_STATE_HANDOVER) {
		_decon_reinit_locked(decon);
		/* remove pm_runtime ref taken during probe */
		pm_runtime_put(decon->dev);
	} else if (decon->state == DECON_STATE_INIT) {
		_decon_stop_locked(decon, true, drm_mode_vrefresh(&old_crtc_state->mode));
	}
	_decon_enable_locked(decon);
	decon->state = DECON_STATE_ON;
	spin_unlock_irqrestore(&decon->slock, flags);

	decon_print_config_info(decon);

	DPU_EVENT_LOG(DPU_EVT_DECON_ENABLED, decon->id, decon);

	decon_info(decon, "%s -\n", __func__);

ret:
	if (decon->config.dsc.enabled && decon->config.dsc.delay_reg_init_us) {
		struct drm_atomic_state *state = old_crtc_state->state;
		struct exynos_drm_connector_state *exynos_conn_state =
					crtc_get_exynos_connector_state(state, crtc_state);
		struct exynos_display_mode *exynos_mode = &exynos_conn_state->exynos_mode;
		unsigned int delay_us = decon->config.dsc.delay_reg_init_us;
		unsigned int extra_delay_us =
				DIV_ROUND_UP(MSEC_PER_SEC, vrefresh) * MSEC_PER_SEC - delay_us;

		decon_wait_for_te(decon, vrefresh);
		usleep_range(extra_delay_us, extra_delay_us + 100);

		/* remove the delay */
		exynos_mode->dsc.delay_reg_init_us = 0;
		decon->config.dsc.delay_reg_init_us = 0;

		decon_dsc_reg_init(decon->id, &decon->config, 0, 0);
	}

	/* drop extra vote taken to avoid power disable during bypass mode */
	if (old_exynos_crtc_state->bypass) {
		decon_debug(decon, "bypass mode: drop extra power ref\n");
		pm_runtime_put_sync(decon->dev);
	}

	DPU_ATRACE_END(__func__);

	WARN_ON(!pm_runtime_active(decon->dev));
}

static void decon_disable_irqs(struct decon_device *decon)
{
	disable_irq_nosync(decon->irq_fd);
	disable_irq_nosync(decon->irq_ext);
	if (decon->irq_ds >= 0)
		disable_irq_nosync(decon->irq_ds);
	if (decon->irq_de >= 0)
		disable_irq_nosync(decon->irq_de);
	decon_reg_set_interrupts(decon->id, 0);
	if (decon_is_te_enabled(decon))
		disable_irq_nosync(decon->irq_fs);
}

static u32 _decon_get_current_fps(struct decon_device *decon)
{
	struct drm_crtc *crtc = &decon->crtc->base;
	const struct drm_crtc_state *crtc_state = crtc->state;

	return min_t(u32, decon->bts.fps, drm_mode_vrefresh(&crtc_state->mode)) ?: 60;
}

static bool _decon_wait_for_framedone(struct decon_device *decon)
{
	const u32 fps = _decon_get_current_fps(decon);
	const u64 timeout = fps_timeout(fps);
	u64 ret;

	ret = wait_event_timeout(decon->framedone_wait,
				 atomic_read(&decon->frames_pending) == 0 ||
				 decon_reg_is_idle(decon->id),
				 timeout);
	if (!ret) {
		WARN(1, "decon%d: wait for frame done timed out (%dhz)", decon->id, fps);
		return true;
	} else {
		struct drm_crtc *crtc = &decon->crtc->base;
		const struct drm_crtc_state *crtc_state = crtc->state;
		bool reset = drm_atomic_crtc_needs_modeset(crtc_state);

		decon_debug(decon, "%s: frame done after: ~%dus (%dhz)", __func__,
			    jiffies_to_usecs(timeout - ret), fps);
		return reset;
	}
}

static void _decon_disable_locked(struct decon_device *decon, bool reset)
{
	decon_disable_irqs(decon);
	atomic_set(&decon->frames_pending, 0);
	_decon_stop_locked(decon, reset, _decon_get_current_fps(decon));
}

static void decon_enter_hibernation(struct decon_device *decon)
{
	bool reset = false;
	unsigned long flags;

	if (decon->state != DECON_STATE_ON)
		return;

	decon_debug(decon, "%s +\n", __func__);

	DPU_ATRACE_BEGIN(__func__);
	DPU_EVENT_LOG(DPU_EVT_ENTER_HIBERNATION_IN, decon->id, NULL);

	reset = _decon_wait_for_framedone(decon);
	spin_lock_irqsave(&decon->slock, flags);
	exynos_dqe_hibernation_enter(decon->dqe);
	_decon_disable_locked(decon, reset);
	pm_runtime_put(decon->dev);
	decon->state = DECON_STATE_HIBERNATION;
	spin_unlock_irqrestore(&decon->slock, flags);

	DPU_EVENT_LOG(DPU_EVT_ENTER_HIBERNATION_OUT, decon->id, NULL);
	DPU_ATRACE_END(__func__);

	decon_debug(decon, "%s -\n", __func__);
}

static void decon_disable(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;
	struct drm_crtc_state *crtc_state = crtc->base.state;
	struct exynos_drm_crtc_state *exynos_crtc_state = to_exynos_crtc_state(crtc_state);
	const enum decon_state old_decon_state = decon->state;
	bool reset;
	unsigned long flags;

	if (old_decon_state == DECON_STATE_OFF)
		return;

	if (exynos_crtc_state->bypass) {
		decon_debug(decon, "bypass mode: get extra power ref\n");
		pm_runtime_get_sync(decon->dev);
	}

	if (crtc_state->self_refresh_active) {
		decon_enter_hibernation(decon);
		return;
	}

	decon_info(decon, "%s +\n", __func__);

	if (crtc_state->mode_changed || crtc_state->connectors_changed) {
		if (decon->irq_te >= 0) {
			if (atomic_read(&decon->te_ref))
				disable_irq(decon->irq_te);
			devm_free_irq(decon->dev, decon->irq_te, decon);
			decon->irq_te = -1;
			decon->te_gpio = 0;
		}
	}

	reset = _decon_wait_for_framedone(decon);
	spin_lock_irqsave(&decon->slock, flags);
	if (old_decon_state == DECON_STATE_ON) {
		_decon_disable_locked(decon, reset);
		pm_runtime_put(decon->dev);
	}
	decon->state = DECON_STATE_OFF;
	spin_unlock_irqrestore(&decon->slock, flags);

	DPU_EVENT_LOG(DPU_EVT_DECON_DISABLED, decon->id, decon);

	decon_info(decon, "%s -\n", __func__);
}

static void decon_wait_for_flip_done(struct exynos_drm_crtc *crtc,
				const struct drm_crtc_state *old_crtc_state,
				const struct drm_crtc_state *new_crtc_state)
{
	struct decon_device *decon = crtc->ctx;
	struct drm_crtc_commit *commit = new_crtc_state->commit;
	struct decon_mode *mode;
	struct exynos_drm_crtc_state *new_exynos_crtc_state =
					to_exynos_crtc_state(new_crtc_state);
	int fps, recovering;

	if (!new_crtc_state->active)
		return;

	if (WARN_ON(!commit))
		return;

	fps = drm_mode_vrefresh(&new_crtc_state->mode);
	if (old_crtc_state->active)
		fps = min(fps, drm_mode_vrefresh(&old_crtc_state->mode));

	if (!wait_for_completion_timeout(&commit->flip_done, fps_timeout(fps))) {
		unsigned long flags;
		bool fs_irq_pending;

		spin_lock_irqsave(&decon->slock, flags);
		fs_irq_pending = decon_check_fs_pending_locked(decon);
		spin_unlock_irqrestore(&decon->slock, flags);

		if (!fs_irq_pending) {
			DPU_EVENT_LOG(DPU_EVT_FRAMESTART_TIMEOUT, decon->id, NULL);
			recovering = atomic_read(&decon->recovery.recovering);
			decon_err(decon, "framestart timeout (%dhz), recovering: %d, pending: %d\n",
				    fps, recovering, atomic_read(&decon->frames_pending));

			atomic_set(&decon->frames_pending, 0);
			if (!recovering)
				decon_dump_all(decon, DPU_EVT_CONDITION_DEFAULT, false);

			decon_force_vblank_event(decon);

			if (!recovering)
				decon_trigger_recovery(decon);
		} else {
			pr_warn("decon%u scheduler late to service fs irq handle (%d fps)\n",
					decon->id, fps);
		}
	}

	mode = &decon->config.mode;
	if (mode->op_mode == DECON_COMMAND_MODE && !decon->keep_unmask) {
		DPU_EVENT_LOG(DPU_EVT_DECON_TRIG_MASK, decon->id, NULL);
		decon_reg_set_trigger(decon->id, mode, DECON_TRIG_MASK);
	}

	if (new_exynos_crtc_state->wb_type == EXYNOS_WB_CWB)
		decon_reg_set_cwb_enable(decon->id, false);
}

static const struct exynos_drm_crtc_ops decon_crtc_ops = {
	.enable = decon_enable,
	.disable = decon_disable,
	.enable_vblank = decon_enable_vblank,
	.disable_vblank = decon_disable_vblank,
	.atomic_check = decon_atomic_check,
	.atomic_begin = decon_atomic_begin,
	.update_plane = decon_update_plane,
	.disable_plane = decon_disable_plane,
	.atomic_flush = decon_atomic_flush,
	.wait_for_flip_done = decon_wait_for_flip_done,
};

static int dpu_sysmmu_fault_handler(struct iommu_fault *fault, void *data)
{
	struct decon_device *decon = data;

	if (!decon || !decon_is_effectively_active(decon))
		return 0;

	DPU_EVENT_LOG(DPU_EVT_SYSMMU_FAULT, decon->id, NULL);
	decon_warn(decon, "%s +\n", __func__);

	decon_dump_all(decon, DPU_EVT_CONDITION_DEFAULT, false);

	return 0;
}

static ssize_t early_wakeup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t early_wakeup_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct decon_device *decon;
	bool trigger;

	if (!dev || !buf || !len) {
		pr_err("%s: invalid input param(s)\n", __func__);
		return -EINVAL;
	}

	if (kstrtobool(buf, &trigger) < 0)
		return -EINVAL;

	if (!trigger)
		return len;

	decon = dev_get_drvdata(dev);
	exynos_hibernation_async_exit(decon->hibernation);

	return len;
}
static DEVICE_ATTR_RW(early_wakeup);

static int decon_bind(struct device *dev, struct device *master, void *data)
{
	struct decon_device *decon = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_private *priv = drm_to_exynos_dev(drm_dev);
	struct drm_plane *default_plane;
	int i;
	char symlink_name_buffer[7];

	decon->drm_dev = drm_dev;

	default_plane = &decon->dpp[decon->id]->plane.base;

	decon->crtc = exynos_drm_crtc_create(drm_dev, default_plane,
			decon->con_type, &decon_crtc_ops, decon);
	if (IS_ERR(decon->crtc))
		return PTR_ERR(decon->crtc);

	for (i = 0; i < decon->dpp_cnt; ++i) {
		struct dpp_device *dpp = decon->dpp[i];
		struct drm_plane *plane = &dpp->plane.base;

		plane->possible_crtcs |=
			drm_crtc_mask(&decon->crtc->base);
		decon_debug(decon, "plane possible_crtcs = 0x%x\n",
				plane->possible_crtcs);
	}

	if (decon->rcd) {
		struct dpp_device *rcd = decon->rcd;
		struct drm_plane *plane = &rcd->plane.base;

		plane->possible_crtcs |= drm_crtc_mask(&decon->crtc->base);
		decon_debug(decon, "plane possible_crtcs = 0x%x\n",
				plane->possible_crtcs);
		decon->crtc->rcd_plane_mask |= drm_plane_mask(plane);
	}

	priv->iommu_client = dev;

	iommu_register_device_fault_handler(dev, dpu_sysmmu_fault_handler, decon);

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	decon->itmon_nb.notifier_call = dpu_itmon_notifier;
	itmon_notifier_chain_register(&decon->itmon_nb);
#endif

	if (IS_ENABLED(CONFIG_EXYNOS_BTS)) {
		decon->bts.ops = &dpu_bts_control;
		decon->bts.ops->init(decon);
	}

	/* Create symlink to decon device */
	snprintf(symlink_name_buffer, 7, "decon%d", decon->id);
	sysfs_create_link(&decon->drm_dev->dev->kobj, &decon->dev->kobj,
			  (const char *) symlink_name_buffer);

	device_create_file(dev, &dev_attr_early_wakeup);
	decon_debug(decon, "%s -\n", __func__);
	return 0;
}

static void decon_unbind(struct device *dev, struct device *master,
			void *data)
{
	char symlink_name_buffer[7];
	struct decon_device *decon = dev_get_drvdata(dev);
	decon_debug(decon, "%s +\n", __func__);

	/* Remove symlink to decon device */
	snprintf(symlink_name_buffer, 7, "decon%d", decon->id);
	sysfs_remove_link(&decon->drm_dev->dev->kobj,
			  (const char *) symlink_name_buffer);

	device_remove_file(dev, &dev_attr_early_wakeup);
	if (IS_ENABLED(CONFIG_EXYNOS_BTS))
		decon->bts.ops->deinit(decon);

	decon_disable(decon->crtc);
	decon_debug(decon, "%s -\n", __func__);
}

static const struct component_ops decon_component_ops = {
	.bind	= decon_bind,
	.unbind = decon_unbind,
};

static irqreturn_t decon_irq_handler(int irq, void *dev_data)
{
	struct decon_device *decon = dev_data;
	u32 irq_sts_reg;
	u32 ext_irq = 0;

	spin_lock(&decon->slock);
	if (decon->state != DECON_STATE_ON) {
		decon_warn(decon, "%s: irq occurs with decon->state=%d\n", __func__, decon->state);
		goto irq_end;
	}

	irq_sts_reg = decon_reg_get_interrupt_and_clear(decon->id, &ext_irq);
	decon_debug(decon, "%s: irq_sts_reg = %x, ext_irq = %x\n",
			__func__, irq_sts_reg, ext_irq);

	if (irq_sts_reg & DPU_FRAME_DONE_INT_PEND) {
		DPU_ATRACE_INT_PID("frame_transfer", 0, decon->thread->pid);
		DPU_EVENT_LOG(DPU_EVT_DECON_FRAMEDONE, decon->id, decon);
		exynos_dqe_save_lpd_data(decon->dqe);
		atomic_dec_if_positive(&decon->frames_pending);
		if (decon->dqe)
			handle_histogram_event(decon->dqe);
		wake_up_all(&decon->framedone_wait);
		decon_debug(decon, "%s: frame done\n", __func__);
	}

	if (irq_sts_reg & INT_PEND_DQE_DIMMING_START) {
		decon->keep_unmask = true;
		if (decon->config.mode.op_mode == DECON_COMMAND_MODE)
			decon_reg_set_trigger(decon->id, &decon->config.mode,
					DECON_TRIG_UNMASK);

		DPU_EVENT_LOG(DPU_EVT_DIMMING_START, decon->id, NULL);
	}

	if (irq_sts_reg & INT_PEND_DQE_DIMMING_END) {
		decon->keep_unmask = false;
		if (!decon->event && decon->config.mode.op_mode == DECON_COMMAND_MODE)
			decon_reg_set_trigger(decon->id, &decon->config.mode,
					DECON_TRIG_MASK);

		DPU_EVENT_LOG(DPU_EVT_DIMMING_END, decon->id, NULL);
	}

	if (ext_irq & DPU_RESOURCE_CONFLICT_INT_PEND)
		decon_debug(decon, "%s: resource conflict\n", __func__);

	if (ext_irq & DPU_TIME_OUT_INT_PEND) {
		decon_err(decon, "%s: timeout irq occurs\n", __func__);
		decon_dump(decon);
		WARN_ON(1);
	}

irq_end:
	spin_unlock(&decon->slock);
	return IRQ_HANDLED;
}

static bool decon_check_fs_pending_locked(struct decon_device *decon)
{
	u32 pending_irq;

	if (decon->state != DECON_STATE_ON)
		return false;

	pending_irq = decon_reg_get_fs_interrupt_and_clear(decon->id);

	if (pending_irq & DPU_FRAME_START_INT_PEND) {
		DPU_ATRACE_INT_PID("frame_transfer", 1, decon->thread->pid);
		DPU_EVENT_LOG(DPU_EVT_DECON_FRAMESTART, decon->id, decon);
		decon_send_vblank_event_locked(decon);
		if (decon->config.mode.op_mode == DECON_VIDEO_MODE)
			drm_crtc_handle_vblank(&decon->crtc->base);

		return true;
	}

	return false;
}


static irqreturn_t decon_fs_irq_handler(int irq, void *dev_data)
{
	struct decon_device *decon = dev_data;

	spin_lock(&decon->slock);

	if (decon_check_fs_pending_locked(decon))
		decon_debug(decon, "%s: frame start\n", __func__);

	spin_unlock(&decon->slock);
	return IRQ_HANDLED;
}

static int decon_parse_dt(struct decon_device *decon, struct device_node *np)
{
	struct device_node *dpp_np = NULL;
	struct property *prop;
	const __be32 *cur;
	u32 val;
	int ret = 0, i;
	int dpp_id;
	u32 dfs_lv_cnt, dfs_lv_khz[BTS_DFS_MAX] = {400000, 0, };
	bool err_flag = false;

	of_property_read_u32(np, "decon,id", &decon->id);

	ret = of_property_read_u32(np, "max_win", &decon->win_cnt);
	if (ret) {
		decon_err(decon, "failed to parse max windows count\n");
		return ret;
	}

	ret = of_property_read_u32(np, "rd_en", &decon->config.urgent.rd_en);
	if (ret)
		decon_warn(decon, "failed to parse urgent rd_en(%d)\n", ret);

	ret = of_property_read_u32(np, "rd_hi_thres",
			&decon->config.urgent.rd_hi_thres);
	if (ret) {
		decon_warn(decon, "failed to parse urgent rd_hi_thres(%d)\n",
				ret);
	}

	ret = of_property_read_u32(np, "rd_lo_thres",
			&decon->config.urgent.rd_lo_thres);
	if (ret) {
		decon_warn(decon, "failed to parse urgent rd_lo_thres(%d)\n",
				ret);
	}

	ret = of_property_read_u32(np, "rd_wait_cycle",
			&decon->config.urgent.rd_wait_cycle);
	if (ret) {
		decon_warn(decon, "failed to parse urgent rd_wait_cycle(%d)\n",
				ret);
	}

	ret = of_property_read_u32(np, "wr_en", &decon->config.urgent.wr_en);
	if (ret)
		decon_warn(decon, "failed to parse urgent wr_en(%d)\n", ret);

	ret = of_property_read_u32(np, "wr_hi_thres",
			&decon->config.urgent.wr_hi_thres);
	if (ret) {
		decon_warn(decon, "failed to parse urgent wr_hi_thres(%d)\n",
				ret);
	}

	ret = of_property_read_u32(np, "wr_lo_thres",
			&decon->config.urgent.wr_lo_thres);
	if (ret) {
		decon_warn(decon, "failed to parse urgent wr_lo_thres(%d)\n",
				ret);
	}

	decon->config.urgent.dta_en = of_property_read_bool(np, "dta_en");
	if (decon->config.urgent.dta_en) {
		ret = of_property_read_u32(np, "dta_hi_thres",
				&decon->config.urgent.dta_hi_thres);
		if (ret) {
			decon_err(decon, "failed to parse dta_hi_thres(%d)\n",
					ret);
		}
		ret = of_property_read_u32(np, "dta_lo_thres",
				&decon->config.urgent.dta_lo_thres);
		if (ret) {
			decon_err(decon, "failed to parse dta_lo_thres(%d)\n",
					ret);
		}
	}

	if (of_property_read_u32(np, "ppc", (u32 *)&decon->bts.ppc))
		decon->bts.ppc = 2U;
	decon_info(decon, "PPC(%u)\n", decon->bts.ppc);

	if (of_property_read_u32(np, "ppc_rotator",
					(u32 *)&decon->bts.ppc_rotator)) {
		decon->bts.ppc_rotator = 4U;
		decon_warn(decon, "WARN: rotator ppc is not defined in DT.\n");
	}
	decon_info(decon, "rotator ppc(%d)\n", decon->bts.ppc_rotator);

	if (of_property_read_u32(np, "ppc_scaler",
					(u32 *)&decon->bts.ppc_scaler)) {
		decon->bts.ppc_scaler = 2U;
		decon_warn(decon, "WARN: scaler ppc is not defined in DT.\n");
	}
	decon_info(decon, "scaler ppc(%d)\n", decon->bts.ppc_scaler);

	if (of_property_read_u32(np, "delay_comp",
				(u32 *)&decon->bts.delay_comp)) {
		decon->bts.delay_comp = 4UL;
		decon_warn(decon, "WARN: comp line delay is not defined in DT.\n");
	}
	decon_info(decon, "line delay comp(%d)\n", decon->bts.delay_comp);

	if (of_property_read_u32(np, "delay_scaler",
				(u32 *)&decon->bts.delay_scaler)) {
		decon->bts.delay_scaler = 2UL;
		decon_warn(decon, "WARN: scaler line delay is not defined in DT.\n");
	}
	decon_info(decon, "line delay scaler(%d)\n", decon->bts.delay_scaler);

	if (of_property_read_u32(np, "bus_width", &decon->bts.bus_width)) {
		decon->bts.bus_width = 16;
		decon_warn(decon, "WARN: bus_width is not defined in DT.\n");
	}
	if (of_property_read_u32(np, "bus_util", &decon->bts.bus_util_pct)) {
		decon->bts.bus_util_pct = 65;
		decon_debug(decon, "WARN: bus_util_pct is not defined in DT.\n");
	}
	if (of_property_read_u32(np, "rot_util", &decon->bts.rot_util_pct)) {
		decon->bts.rot_util_pct = 60;
		decon_debug(decon, "WARN: rot_util_pct is not defined in DT.\n");
	}
	if (of_property_read_u32(np, "afbc_rgb_util_pct", &decon->bts.afbc_rgb_util_pct)) {
		decon->bts.afbc_rgb_util_pct = 100;
		decon_debug(decon, "WARN: afbc_rgb_util_pct is not defined in DT.\n");
	}
	if (of_property_read_u32(np, "afbc_yuv_util_pct", &decon->bts.afbc_yuv_util_pct)) {
		decon->bts.afbc_yuv_util_pct = 100;
		decon_debug(decon, "WARN: afbc_yuv_util_pct is not defined in DT.\n");
	}
	if (of_property_read_u32(np, "afbc_rgb_rt_util_pct", &decon->bts.afbc_rgb_rt_util_pct)) {
		decon->bts.afbc_rgb_rt_util_pct = 100;
		decon_debug(decon, "WARN: afbc_rgb_rt_util_pct is not defined in DT.\n");
	}
	if (of_property_read_u32(np, "afbc_yuv_rt_util_pct", &decon->bts.afbc_yuv_rt_util_pct)) {
		decon->bts.afbc_yuv_rt_util_pct = 100;
		decon_debug(decon, "WARN: afbc_yuv_rt_util_pct is not defined in DT.\n");
	}

	decon_debug(decon, "bus_width(%u) bus_util(%u) rot_util(%u)\n",
			decon->bts.bus_width, decon->bts.bus_util_pct,
			decon->bts.rot_util_pct);

	decon_debug(decon, "afbc: rgb_util(%u) yuv_util(%u) rgb_rt_util(%u) yuv_rt_util(%u)\n",
			decon->bts.afbc_rgb_util_pct, decon->bts.afbc_yuv_util_pct,
			decon->bts.afbc_rgb_rt_util_pct, decon->bts.afbc_yuv_rt_util_pct);

	if (of_property_read_u32(np, "dfs_lv_cnt", &dfs_lv_cnt)) {
		err_flag = true;
		dfs_lv_cnt = 1;
		decon->bts.dfs_lv_khz[0] = 400000U; /* 400Mhz */
		decon_warn(decon, "WARN: DPU DFS Info is not defined in DT.\n");
	}
	decon->bts.dfs_lv_cnt = dfs_lv_cnt;

	if (!err_flag) {
		of_property_read_u32_array(np, "dfs_lv", dfs_lv_khz, dfs_lv_cnt);
		decon_info(decon, "DPU DFS Level : ");
		for (i = 0; i < dfs_lv_cnt; i++) {
			decon->bts.dfs_lv_khz[i] = dfs_lv_khz[i];
			decon_info(decon, "%6d ", dfs_lv_khz[i]);
		}
		decon_info(decon, "\n");
	}

	if (of_property_read_u32(np, "max_dfs_lv_for_wb", &decon->bts.max_dfs_lv_for_wb)) {
		decon->bts.max_dfs_lv_for_wb = 0;
		decon_debug(decon, "max_dfs_lv_for_wb is not defined in DT.\n");
	} else {
		decon_debug(decon, "max_dfs_lv_for_wb(%u)\n", decon->bts.max_dfs_lv_for_wb);
	}

	decon->dpp_cnt = of_count_phandle_with_args(np, "dpps", NULL);
	for (i = 0; i < decon->dpp_cnt; ++i) {
		dpp_np = of_parse_phandle(np, "dpps", i);
		if (!dpp_np) {
			decon_err(decon, "can't find dpp%d node\n", i);
			return -EINVAL;
		}

		decon->dpp[i] = of_find_dpp_by_node(dpp_np);
		if (!decon->dpp[i]) {
			decon_err(decon, "can't find dpp%d structure\n", i);
			return -EINVAL;
		}

		dpp_id = decon->dpp[i]->id;
		decon_info(decon, "found dpp%d\n", dpp_id);

		if (dpp_np)
			of_node_put(dpp_np);
	}

	/* RCD Function */
	dpp_np = of_parse_phandle(np, "rcd", 0);
	if (!dpp_np)
		decon_debug(decon, "can't find rcd node\n");

	decon->rcd = of_find_dpp_by_node(dpp_np);
	if (!decon->rcd)
		decon_debug(decon, "can't find rcd structure\n");
	else
		decon_debug(decon, "found rcd: dpp%d\n", decon->rcd->id);

	if (dpp_np)
		of_node_put(dpp_np);

	of_property_for_each_u32(np, "connector", prop, cur, val)
		decon->con_type |= val;

	return 0;
}

static int decon_remap_regs(struct decon_device *decon)
{
	struct resource res;
	struct device *dev = decon->dev;
	struct device_node *np = dev->of_node;
	int i, ret = 0;

	i = of_property_match_string(np, "reg-names", "main");
	if (of_address_to_resource(np, i, &res)) {
		decon_err(decon, "failed to get main resource\n");
		goto err;
	}
	decon->regs.regs = ioremap(res.start, resource_size(&res));
	if (IS_ERR(decon->regs.regs)) {
		decon_err(decon, "failed decon ioremap\n");
		ret = PTR_ERR(decon->regs.regs);
		goto err;
	}
	decon_regs_desc_init(decon->regs.regs, res.start, "decon", REGS_DECON,
			decon->id);

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9-disp_ss");
	if (IS_ERR_OR_NULL(np)) {
		decon_err(decon, "failed to find disp_ss node");
		ret = PTR_ERR(np);
		goto err_main;
	}
	i = of_property_match_string(np, "reg-names", "sys");
	if (of_address_to_resource(np, i, &res)) {
		decon_err(decon, "failed to get sys resource\n");
		goto err_main;
	}
	decon->regs.ss_regs = ioremap(res.start, resource_size(&res));
	if (!decon->regs.ss_regs) {
		decon_err(decon, "failed to map sysreg-disp address.");
		ret = -ENOMEM;
		goto err_main;
	}
	decon_regs_desc_init(decon->regs.ss_regs, res.start, "decon-ss", REGS_DECON_SYS,
			decon->id);

	return ret;

err_main:
	iounmap(decon->regs.regs);
err:
	return ret;
}

static irqreturn_t decon_te_irq_handler(int irq, void *dev_id)
{
	struct decon_device *decon = dev_id;

	if (!decon)
		goto end;

	pr_debug("%s: state(%d)\n", __func__, decon->state);

	if (decon->state != DECON_STATE_ON &&
				decon->state != DECON_STATE_HIBERNATION)
		goto end;

	if (decon->d.force_te_on && decon->te_gpio > 0) {
		bool level = gpio_get_value(decon->te_gpio);

		DPU_ATRACE_INT_PID("TE", level, decon->thread->pid);
		if (!level)
			goto end;
	} else {
		DPU_ATRACE_INT_PID("TE", decon->d.te_cnt++ & 1, decon->thread->pid);
	}
	DPU_EVENT_LOG(DPU_EVT_TE_INTERRUPT, decon->id, NULL);

	if (decon->config.dsc.delay_reg_init_us)
		complete_all(&decon->te_rising);

	if (decon->config.mode.op_mode == DECON_COMMAND_MODE)
		drm_crtc_handle_vblank(&decon->crtc->base);

end:
	return IRQ_HANDLED;
}

static int decon_request_te_irq(struct exynos_drm_crtc *exynos_crtc,
				const struct exynos_drm_connector_state *exynos_conn_state)
{
	struct decon_device *decon = exynos_crtc->ctx;
	int ret, irq;
	unsigned long flags = IRQF_TRIGGER_RISING;

	if (WARN_ON(!exynos_conn_state))
		return -EINVAL;

	WARN(decon->irq_te >= 0, "unbalanced te irq\n");

	irq = gpio_to_irq(exynos_conn_state->te_gpio);
	if (decon->d.force_te_on && exynos_conn_state->te_gpio > 0) {
		flags |= IRQF_TRIGGER_FALLING;
		decon->te_gpio = exynos_conn_state->te_gpio;
	}

	decon_debug(decon, "TE irq number(%d)\n", irq);
	irq_set_status_flags(irq, IRQ_DISABLE_UNLAZY);
	ret = devm_request_irq(decon->dev, irq, decon_te_irq_handler, flags,
			       exynos_crtc->base.name, decon);
	if (!ret) {
		decon->irq_te = irq;
		if (!atomic_read(&decon->te_ref))
			disable_irq(irq);
	}

	return ret;
}

static int decon_register_irqs(struct decon_device *decon)
{
	struct device *dev = decon->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev;
	int ret = 0;

	pdev = container_of(dev, struct platform_device, dev);

	/* 1: FRAME START */
	decon->irq_fs = of_irq_get_byname(np, "frame_start");
	ret = devm_request_irq(dev, decon->irq_fs, decon_fs_irq_handler,
			0, pdev->name, decon);
	if (ret) {
		decon_err(decon, "failed to install FRAME START irq\n");
		return ret;
	}
	disable_irq(decon->irq_fs);

	/* 2: FRAME DONE */
	decon->irq_fd = of_irq_get_byname(np, "frame_done");
	ret = devm_request_irq(dev, decon->irq_fd, decon_irq_handler,
			0, pdev->name, decon);
	if (ret) {
		decon_err(decon, "failed to install FRAME DONE irq\n");
		return ret;
	}
	disable_irq(decon->irq_fd);

	/* 3: EXTRA: resource conflict, timeout and error irq */
	decon->irq_ext = of_irq_get_byname(np, "extra");
	ret = devm_request_irq(dev, decon->irq_ext, decon_irq_handler,
			0, pdev->name, decon);
	if (ret) {
		decon_err(decon, "failed to install EXTRA irq\n");
		return ret;
	}
	disable_irq(decon->irq_ext);

	/* 4: DIMMING START */
	decon->irq_ds = of_irq_get_byname(np, "dimming_start");
	if (devm_request_irq(dev, decon->irq_ds, decon_irq_handler,
			0, pdev->name, decon)) {
		decon->irq_ds = -1;
		decon_info(decon, "dimming start irq is not supported\n");
	} else {
		disable_irq(decon->irq_ds);
	}

	/* 5: DIMMING END */
	decon->irq_de = of_irq_get_byname(np, "dimming_end");
	if (devm_request_irq(dev, decon->irq_de, decon_irq_handler,
			0, pdev->name, decon)) {
		decon->irq_de = -1;
		decon_info(decon, "dimming end irq is not supported\n");
	} else {
		disable_irq(decon->irq_de);
	}

	decon->irq_te = -1;

	return ret;
}

#ifndef CONFIG_BOARD_EMULATOR
static int decon_get_clock(struct decon_device *decon)
{
	decon->res.aclk = devm_clk_get(decon->dev, "aclk");
	if (IS_ERR_OR_NULL(decon->res.aclk)) {
		decon_info(decon, "failed to get aclk(optional)\n");
		decon->res.aclk = NULL;
	}

	decon->res.aclk_disp = devm_clk_get(decon->dev, "aclk-disp");
	if (IS_ERR_OR_NULL(decon->res.aclk_disp)) {
		decon_info(decon, "failed to get aclk_disp(optional)\n");
		decon->res.aclk_disp = NULL;
	}

	return 0;
}
#else
static inline int decon_get_clock(struct decon_device *decon) { return 0; }
#endif

static int decon_init_resources(struct decon_device *decon)
{
	int ret = 0;

	ret = decon_remap_regs(decon);
	if (ret)
		goto err;

	ret = decon_register_irqs(decon);
	if (ret)
		goto err;

	ret = decon_get_clock(decon);
	if (ret)
		goto err;

	ret = __decon_init_resources(decon);
	if (ret)
		goto err;

err:
	return ret;
}

static int decon_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct decon_device *decon;
	struct device *dev = &pdev->dev;
	struct sched_param param = {
		.sched_priority = 20
	};

	decon = devm_kzalloc(dev, sizeof(struct decon_device), GFP_KERNEL);
	if (!decon)
		return -ENOMEM;

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));

	decon->dev = dev;

	ret = decon_parse_dt(decon, dev->of_node);
	if (ret)
		goto err;

	decon_drvdata[decon->id] = decon;

	spin_lock_init(&decon->slock);
	init_waitqueue_head(&decon->framedone_wait);
	init_completion(&decon->te_rising);

	ret = decon_init_resources(decon);
	if (ret)
		goto err;

	/* set drvdata */
	platform_set_drvdata(pdev, decon);

	kthread_init_worker(&decon->worker);
	decon->thread = kthread_run(kthread_worker_fn, &decon->worker,
				    "decon%u_kthread", decon->id);
	if (IS_ERR(decon->thread)) {
		decon_err(decon, "failed to run display thread\n");
		ret = PTR_ERR(decon->thread);
		goto err;
	}
	sched_setscheduler_nocheck(decon->thread, SCHED_FIFO, &param);

	decon->hibernation = exynos_hibernation_register(decon);
	exynos_recovery_register(decon);

	decon->dqe = exynos_dqe_register(decon);

	decon->cgc_dma = exynos_cgc_dma_register(decon);
	exynos_rmem_register(decon);

	decon->state = decon->fb_handover.rmem ? DECON_STATE_HANDOVER : DECON_STATE_INIT;
	pm_runtime_enable(decon->dev);

	if (decon->state == DECON_STATE_HANDOVER)
		pm_runtime_get_sync(decon->dev);

	ret = component_add(dev, &decon_component_ops);
	if (ret)
		goto err;

	decon_info(decon, "successfully probed");

err:
	return ret;
}

static int decon_remove(struct platform_device *pdev)
{
	struct decon_device *decon = platform_get_drvdata(pdev);

	if (decon->thread)
		kthread_stop(decon->thread);

	exynos_hibernation_destroy(decon->hibernation);

	component_del(&pdev->dev, &decon_component_ops);

	__decon_unmap_regs(decon);
	iounmap(decon->regs.regs);

	return 0;
}

#ifdef CONFIG_PM
static int decon_runtime_suspend(struct device *dev)
{
	struct decon_device *decon = dev_get_drvdata(dev);

	if (decon->state != DECON_STATE_HIBERNATION &&
		decon->state != DECON_STATE_OFF) {
		decon_warn(decon, "decon state = %u at suspending\n",
			decon->state);
		WARN_ON(1);
		decon_dump_all(decon, DPU_EVT_CONDITION_DEFAULT, false);
		return -EINVAL;
	}

	if (decon->res.aclk)
		clk_disable_unprepare(decon->res.aclk);

	if (decon->res.aclk_disp)
		clk_disable_unprepare(decon->res.aclk_disp);

	if (decon->dqe)
		exynos_dqe_reset(decon->dqe);

	DPU_EVENT_LOG(DPU_EVT_DECON_RUNTIME_SUSPEND, decon->id, NULL);

	decon_debug(decon, "suspended\n");

	return 0;
}

static int decon_runtime_resume(struct device *dev)
{
	struct decon_device *decon = dev_get_drvdata(dev);

	if (decon->state == DECON_STATE_ON) {
		decon_warn(decon, "decon state = %u at resuming\n",
			decon->state);
		WARN_ON(1);
		decon_dump_all(decon, DPU_EVT_CONDITION_DEFAULT, false);
		return -EINVAL;
	}

	if (decon->res.aclk)
		clk_prepare_enable(decon->res.aclk);

	if (decon->res.aclk_disp)
		clk_prepare_enable(decon->res.aclk_disp);

	DPU_EVENT_LOG(DPU_EVT_DECON_RUNTIME_RESUME, decon->id, NULL);

	decon_debug(decon, "resumed\n");

	return 0;
}

static int decon_suspend(struct device *dev)
{
	struct decon_device *decon = dev_get_drvdata(dev);
	int ret;

	if (!decon_is_effectively_active(decon))
		return 0;

	decon_debug(decon, "%s\n", __func__);

	ret = exynos_hibernation_suspend(decon->hibernation);

	DPU_EVENT_LOG(DPU_EVT_DECON_SUSPEND, decon->id, NULL);

	return ret;
}

static int decon_resume(struct device *dev)
{
	struct decon_device *decon = dev_get_drvdata(dev);

	if (!decon_is_effectively_active(decon))
		return 0;

	decon_debug(decon, "%s\n", __func__);

	DPU_EVENT_LOG(DPU_EVT_DECON_RESUME, decon->id, NULL);

	return 0;
}
#endif

static const struct dev_pm_ops decon_pm_ops = {
	SET_RUNTIME_PM_OPS(decon_runtime_suspend, decon_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(decon_suspend, decon_resume)
};

struct platform_driver decon_driver = {
	.probe		= decon_probe,
	.remove		= decon_remove,
	.driver		= {
		.name	= "exynos-decon",
		.pm	= &decon_pm_ops,
		.of_match_table = decon_driver_dt_match,
	},
};

MODULE_AUTHOR("Hyung-jun Kim <hyungjun07.kim@samsung.com>");
MODULE_AUTHOR("Seong-gyu Park <seongyu.park@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC Display and Enhancement Controller");
MODULE_LICENSE("GPL v2");
