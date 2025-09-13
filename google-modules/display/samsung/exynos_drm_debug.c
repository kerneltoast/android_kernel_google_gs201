// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * DPU Event log file for Samsung EXYNOS DPU driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/debugfs.h>
#include <linux/moduleparam.h>
#include <linux/pm_runtime.h>
#include <linux/sched/clock.h>
#include <linux/sysfs.h>
#include <linux/time.h>
#include <video/mipi_display.h>
#include <drm/drm_print.h>
#include <drm/drm_managed.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fourcc_gs101.h>
#include <trace/dpu_trace.h>

#include <cal_config.h>
#include <soc/google/exynos_tty.h>

#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
#include <soc/google/exynos-devfreq.h>
#if defined(CONFIG_SOC_GS101)
#include <dt-bindings/soc/google/gs101-devfreq.h>
#elif defined(CONFIG_SOC_GS201)
#include <dt-bindings/soc/google/gs201-devfreq.h>
#elif defined(CONFIG_SOC_ZUMA)
#include <dt-bindings/soc/google/zuma-devfreq.h>
#endif
#endif

#include <dqe_cal.h>
#include <hdr_cal.h>

#include "exynos_drm_decon.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_writeback.h"

/* Default is 1024 entries array for event log buffer */
static unsigned int dpu_event_log_max = 1024;
static unsigned int dpu_event_print_max = 512;
static unsigned int dpu_event_print_underrun = 128;
static unsigned int dpu_event_print_fail_update_bw = 32;
static unsigned int dpu_debug_dump_mask = DPU_EVT_CONDITION_DEFAULT |
	DPU_EVT_CONDITION_UNDERRUN | DPU_EVT_CONDITION_FAIL_UPDATE_BW |
	DPU_EVT_CONDITION_FIFO_TIMEOUT | DPU_EVT_CONDITION_IDMA_ERROR_COMPACT;

module_param_named(event_log_max, dpu_event_log_max, uint, 0);
module_param_named(event_print_max, dpu_event_print_max, uint, 0600);
module_param_named(debug_dump_mask, dpu_debug_dump_mask, uint, 0600);

MODULE_PARM_DESC(event_log_max, "entry count of event log buffer array");
MODULE_PARM_DESC(event_print_max, "print entry count of event log buffer");
MODULE_PARM_DESC(debug_dump_mask, "mask for dump debug event log");

/* If event are happened continuously, then ignore */
static bool dpu_event_ignore
	(enum dpu_event_type type, struct decon_device *decon)
{
	int latest = atomic_read(&decon->d.event_log_idx) % dpu_event_log_max;
	int idx, offset;

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return true;

	for (offset = 0; offset < DPU_EVENT_KEEP_CNT; ++offset) {
		idx = (latest + dpu_event_log_max - offset) % dpu_event_log_max;
		if (type != decon->d.event_log[idx].type)
			return false;
	}

	return true;
}

#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
static void dpu_event_save_freqs(struct dpu_log_freqs *freqs)
{
	freqs->mif_freq = exynos_devfreq_get_domain_freq(DEVFREQ_MIF);
	freqs->int_freq = exynos_devfreq_get_domain_freq(DEVFREQ_INT);
	freqs->disp_freq = exynos_devfreq_get_domain_freq(DEVFREQ_DISP);
}
#else
static void dpu_event_save_freqs(struct dpu_log_freqs *freqs) { }
#endif

static struct dpu_log *dpu_event_get_next(struct decon_device *decon)
{
	struct dpu_log *log;
	unsigned long flags;
	int idx;

	if (!decon) {
		pr_err("%s: invalid decon\n", __func__);
		return NULL;
	}

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return NULL;

	spin_lock_irqsave(&decon->d.event_lock, flags);
	idx = atomic_inc_return(&decon->d.event_log_idx) % dpu_event_log_max;
	log = &decon->d.event_log[idx];
	log->type = DPU_EVT_NONE;
	spin_unlock_irqrestore(&decon->d.event_lock, flags);

	log->ts_nsec = local_clock();

	return log;
}

/* ===== EXTERN APIs ===== */

/*
 * DPU_EVENT_LOG() - store information to log buffer by common API
 * @type: event type
 * @index: event log index
 * @priv: pointer to DECON, DSIM or DPP device structure
 *
 * Store information related to DECON, DSIM or DPP. Each DECON has event log
 * So, DECON id is used as @index
 */
void DPU_EVENT_LOG(enum dpu_event_type type, int index, void *priv)
{
	struct decon_device *decon = NULL;
	struct dpp_device *dpp = NULL;
	struct dsim_device *dsim = NULL;
	struct dpu_log *log;
	struct drm_crtc_state *crtc_state;
	struct drm_plane_state *plane_state;
	const struct drm_format_info *fb_format;
	struct exynos_partial *partial;
	struct drm_rect *partial_region;
	bool skip_excessive = true;

	if (index < 0 || index >= MAX_DECON_CNT) {
		DRM_ERROR("%s: decon id is not valid(%d)\n", __func__, index);
		return;
	}

	decon = get_decon_drvdata(index);
	if (unlikely(!decon)) {
		pr_err("%s: Invalid decon (%d)\n", __func__, index);
		return;
	}
	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	switch (type) {
	case DPU_EVT_DECON_FRAMESTART:
		decon->d.auto_refresh_frames++;
		fallthrough;
	case DPU_EVT_DECON_FRAMEDONE:
	case DPU_EVT_DPP_FRAMEDONE:
	case DPU_EVT_DSIM_FRAMEDONE:
		if (decon->d.auto_refresh_frames > 3)
			return;
		break;
	case DPU_EVT_TE_INTERRUPT:
		break;
	case DPU_EVT_DSIM_UNDERRUN:
		decon->d.underrun_cnt++;
		break;
	case DPU_EVT_DSIM_CRC:
		decon->d.crc_cnt++;
		break;
	case DPU_EVT_DSIM_ECC:
		decon->d.ecc_cnt++;
		break;
	case DPU_EVT_IDMA_AFBC_CONFLICT:
	case DPU_EVT_IDMA_FBC_ERROR:
	case DPU_EVT_IDMA_READ_SLAVE_ERROR:
	case DPU_EVT_IDMA_DEADLOCK:
	case DPU_EVT_IDMA_CFG_ERROR:
		decon->d.idma_err_cnt++;
		DPU_ATRACE_INT_PID("IDMA_ERROR", decon->d.idma_err_cnt & 1, decon->thread->pid);
		break;
	default:
		skip_excessive = false;
		break;
	}

	/*
	 * If the same event occurs DPU_EVENT_KEEP_CNT times
	 * continuously, it will be skipped.
	 */
	if (skip_excessive && dpu_event_ignore(type, decon))
		return;

	log = dpu_event_get_next(decon);
	if (!log)
		return;

	switch (type) {
	case DPU_EVT_DPP_FRAMEDONE:
		dpp = (struct dpp_device *)priv;
		log->data.dpp.id = dpp->id;
		log->data.dpp.win_id = dpp->win_id;
		break;
	case DPU_EVT_DPP_SET_PROTECTION:
		dpp = (struct dpp_device *)priv;
		log->data.dpp.id = dpp->id;
		log->data.dpp.mst_security = dpp->rdma_mst_security;
		log->data.dpp.last_secure_pid = current->pid;
		break;
	case DPU_EVT_DMA_RECOVERY:
		dpp = (struct dpp_device *)priv;
		log->data.dpp.id = dpp->id;
		log->data.dpp.win_id = dpp->win_id;
		log->data.dpp.comp_src = dpp->comp_src;
		log->data.dpp.recovery_cnt = dpp->recovery_cnt;
		break;
	case DPU_EVT_IDMA_AFBC_CONFLICT:
	case DPU_EVT_IDMA_FBC_ERROR:
	case DPU_EVT_IDMA_READ_SLAVE_ERROR:
	case DPU_EVT_IDMA_DEADLOCK:
	case DPU_EVT_IDMA_CFG_ERROR:
		dpp = (struct dpp_device *)priv;
		log->data.dpp.id = dpp->id;
		log->data.dpp.win_id = dpp->win_id;
		log->data.dpp.comp_src = dpp->comp_src;
		break;
	case DPU_EVT_DECON_RSC_OCCUPANCY:
		pm_runtime_get_sync(decon->dev);
		log->data.rsc.rsc_ch = decon_reg_get_rsc_ch(decon->id);
		log->data.rsc.rsc_win = decon_reg_get_rsc_win(decon->id);
		pm_runtime_put_sync(decon->dev);
		break;
	case DPU_EVT_DECON_RUNTIME_SUSPEND:
	case DPU_EVT_DECON_RUNTIME_RESUME:
	case DPU_EVT_DECON_SUSPEND:
	case DPU_EVT_DECON_RESUME:
	case DPU_EVT_ENTER_HIBERNATION_IN:
	case DPU_EVT_ENTER_HIBERNATION_OUT:
	case DPU_EVT_EXIT_HIBERNATION_IN:
	case DPU_EVT_EXIT_HIBERNATION_OUT:
		log->data.pd.decon_state = decon->state;
		log->data.pd.rpm_active = pm_runtime_active(decon->dev);
		break;
	case DPU_EVT_DECON_UPDATE_CONFIG:
		log->data.decon_cfg.fps = decon->bts.fps;
		log->data.decon_cfg.image_height = decon->config.image_height;
		log->data.decon_cfg.image_width = decon->config.image_width;
		log->data.decon_cfg.out_type = decon->config.out_type;
		log->data.decon_cfg.mode.op_mode = decon->config.mode.op_mode;
		log->data.decon_cfg.mode.dsi_mode = decon->config.mode.dsi_mode;
		log->data.decon_cfg.mode.trig_mode = decon->config.mode.trig_mode;
		break;
	case DPU_EVT_DSIM_RUNTIME_SUSPEND:
	case DPU_EVT_DSIM_RUNTIME_RESUME:
	case DPU_EVT_DSIM_SUSPEND:
	case DPU_EVT_DSIM_RESUME:
		dsim = (struct dsim_device *)priv;
		log->data.pd.rpm_active = pm_runtime_active(decon->dev);
		log->data.pd.dsim_state = dsim->state;
		log->data.pd.dsim_rpm_active = pm_runtime_active(dsim->dev);
		break;
	case DPU_EVT_PLANE_PREPARE_FB:
	case DPU_EVT_PLANE_CLEANUP_FB:
		plane_state = (struct drm_plane_state *)priv;
		fb_format = plane_state->fb->format;
		log->data.plane_info.dma_addr =
				exynos_drm_fb_dma_addr(plane_state->fb, 0);
		log->data.plane_info.width = plane_state->fb->width;
		log->data.plane_info.height = plane_state->fb->height;
		log->data.plane_info.zpos = plane_state->normalized_zpos;
		log->data.plane_info.format = fb_format->format;
		log->data.plane_info.index = plane_state->plane->index;
		break;
	case DPU_EVT_PLANE_UPDATE:
	case DPU_EVT_PLANE_DISABLE:
		dpp = (struct dpp_device *)priv;
		log->data.win.win_idx = dpp->win_id;
		log->data.win.plane_idx = dpp->id;
		log->data.win.secure = dpp->protection;
		log->data.win.hdr_en = dpp_need_enable_hdr(dpp);
		break;
	case DPU_EVT_REQ_CRTC_INFO_OLD:
	case DPU_EVT_REQ_CRTC_INFO_NEW:
		crtc_state = (struct drm_crtc_state *)priv;
		log->data.crtc_info.enable = crtc_state->enable;
		log->data.crtc_info.active = crtc_state->active;
		log->data.crtc_info.self_refresh = crtc_state->self_refresh_active;
		log->data.crtc_info.planes_changed = crtc_state->planes_changed;
		log->data.crtc_info.mode_changed = crtc_state->mode_changed;
		log->data.crtc_info.active_changed = crtc_state->active_changed;
		log->data.crtc_info.connectors_changed = crtc_state->connectors_changed;
		break;
	case DPU_EVT_BTS_RELEASE_BW:
	case DPU_EVT_BTS_UPDATE_BW:
		dpu_event_save_freqs(&log->data.bts_update.freqs);
		log->data.bts_update.peak = decon->bts.peak;
		log->data.bts_update.prev_peak = decon->bts.prev_peak;
		log->data.bts_update.rt_avg_bw = decon->bts.rt_avg_bw;
		log->data.bts_update.prev_rt_avg_bw = decon->bts.prev_rt_avg_bw;
		log->data.bts_update.total_bw = decon->bts.total_bw;
		log->data.bts_update.prev_total_bw = decon->bts.prev_total_bw;
		log->data.bts_update.urgent_rd_lat = decon->bts.urgent_rd_lat;
		log->data.bts_update.prev_urgent_rd_lat = decon->bts.prev_urgent_rd_lat;
		if (decon->bts_scen.enabled && decon->bts_scen.voted)
			log->data.bts_update.dpu_scen_idx = decon->bts_scen.idx;
		else
			log->data.bts_update.dpu_scen_idx = 0;
		break;
	case DPU_EVT_BTS_CALC_BW:
		dpu_event_save_freqs(&log->data.bts_cal.freqs);
		log->data.bts_cal.disp_freq = decon->bts.max_disp_freq;
		log->data.bts_cal.peak = decon->bts.peak;
		log->data.bts_cal.rt_avg_bw = decon->bts.rt_avg_bw;
		log->data.bts_cal.read_bw = decon->bts.read_bw;
		log->data.bts_cal.write_bw = decon->bts.write_bw;
		log->data.bts_cal.fps = decon->bts.fps;
		crtc_state = (struct drm_crtc_state *)priv;
		log->data.bts_cal.crtc_fps = drm_mode_vrefresh(&crtc_state->mode);
		log->data.bts_cal.op_rate = decon->bts.op_rate;
		break;
	case DPU_EVT_DSIM_UNDERRUN:
		dpu_event_save_freqs(&log->data.bts_event.freqs);
		log->data.bts_event.value = decon->d.underrun_cnt;
		break;
	case DPU_EVT_PARTIAL_INIT:
		partial = priv;
		log->data.partial.min_w = partial->min_w;
		log->data.partial.min_h = partial->min_h;
		break;
	case DPU_EVT_PARTIAL_PREPARE:
		memcpy(&log->data.partial, priv, sizeof(struct dpu_log_partial));
		break;
	case DPU_EVT_PARTIAL_RESTORE:
	case DPU_EVT_PARTIAL_UPDATE:
		partial_region = priv;
		memcpy(&log->data.partial.prev, partial_region,
					sizeof(struct drm_rect));
		break;
	case DPU_EVT_HIST_COLLECT_BINS:
		log->data.hist_info.id = *(enum exynos_histogram_id *) priv;
		break;
	case DPU_EVT_DSIM_CRC:
		log->data.value = decon->d.crc_cnt;
		break;
	case DPU_EVT_DSIM_ECC:
		log->data.value = decon->d.ecc_cnt;
		break;
	case DPU_EVT_TE_INTERRUPT:
		log->data.value = decon->d.te_cnt;
		break;
	default:
		break;
	}

	log->type = type;
}

/*
 * DPU_EVENT_LOG_ATOMIC_COMMIT() - store all windows information
 * @index: event log index
 *
 * Store all windows information which includes window id, DVA, source and
 * destination coordinates, connected DPP and so on
 */
void DPU_EVENT_LOG_ATOMIC_COMMIT(int index)
{
	struct decon_device *decon;
	struct dpu_log *log;
	int i, dpp_id;

	if (index < 0) {
		DRM_ERROR("%s: decon id is not valid(%d)\n", __func__, index);
		return;
	}

	decon = get_decon_drvdata(index);
	if (unlikely(!decon)) {
		pr_err("%s: Invalid decon (%d)\n", __func__, index);
		return;
	}
	log = dpu_event_get_next(decon);
	if (!log)
		return;

	decon->d.auto_refresh_frames = 0;

	for (i = 0; i < MAX_WIN_PER_DECON; ++i) {
		memcpy(&log->data.atomic.win_config[i].win,
				&decon->bts.win_config[i],
				sizeof(struct dpu_bts_win_config));

		if (decon->bts.win_config[i].state == DPU_WIN_STATE_BUFFER) {
			dpp_id = decon->bts.win_config[i].dpp_id;

			log->data.atomic.win_config[i].dma_addr =
				decon->dpp[DPPCH2PLANE(dpp_id)]->dbg_dma_addr;
		}
	}

	memcpy(&log->data.atomic.rcd_win_config, &decon->bts.rcd_win_config,
	       sizeof(log->data.atomic.rcd_win_config));

	log->type = DPU_EVT_ATOMIC_COMMIT;
}

extern void *return_address(unsigned int);

/*
 * DPU_EVENT_LOG_CMD() - store DSIM command information
 * @index: event log index
 * @dsim: pointer to dsim device structure
 * @type : DSIM command id
 * @d0: data0 in command buffer
 * @len: length of payload
 *
 * Stores command id and first data in command buffer and return addresses
 * in callstack which lets you know who called this function.
 */
void
DPU_EVENT_LOG_CMD(struct dsim_device *dsim, u8 type, u8 d0, u16 len)
{
	int i;
	struct decon_device *decon = (struct decon_device *)dsim_get_decon(dsim);
	struct dpu_log *log;

	if (unlikely(!decon)) {
		pr_err("%s: Invalid decon\n", __func__);
		return;
	}
	log = dpu_event_get_next(decon);
	if (!log)
		return;

	log->data.cmd.id = type;
	log->data.cmd.d0 = d0;
	log->data.cmd.len = len;

	for (i = 0; i < DPU_CALLSTACK_MAX; i++)
		log->data.cmd.caller[i] =
			(void *)((size_t)return_address(i + 1));

	log->type = DPU_EVT_DSIM_COMMAND;
}

static void dpu_print_log_win_config(const struct decon_win_config *const win_config,
				     bool is_rcd, struct drm_printer *p)
{
	static const char *const str_state[3] = { "DISABLED", "COLOR", "BUFFER" };
	const struct dpu_bts_win_config *const win = &win_config->win;
	const struct dpu_fmt *const fmt = dpu_find_fmt_info(win->format);

	char buf[128];
	int len = scnprintf(buf, sizeof(buf),
			"\t\t\t\t\t%s: %s[0x%llx] CH%d SRC[%d %d %d %d] %s%s%s%s",
			is_rcd ? "RCD" : "WIN", str_state[win->state],
			(win->state == DPU_WIN_STATE_BUFFER) ? win_config->dma_addr : 0,
			(win->state == DPU_WIN_STATE_COLOR) ? -1 : win->dpp_id,
			win->src_x, win->src_y, win->src_w, win->src_h,
			(win->is_comp) ? "AFBC " : "", (win->is_rot) ? "ROT " : "",
			(win->is_secure) ? "SECURE " : "",
			(win->hdr_en) ? "HDR " : "");
	len += scnprintf(buf + len, sizeof(buf) - len, "DST[%d %d %d %d] ", win->dst_x, win->dst_y,
			 win->dst_w, win->dst_h);
	len += scnprintf(buf + len, sizeof(buf) - len, "ZPOS%d", win->zpos);
	drm_printf(p, "%s %s %s\n", buf, dpu_get_fmt_name(fmt), get_comp_src_name(win->comp_src));
}

static void dpu_print_log_atomic(struct dpu_log_atomic *atomic,
						struct drm_printer *p)
{
	int i;
	const struct dpu_bts_win_config *win;

	for (i = 0; i < MAX_WIN_PER_DECON; ++i) {
		win = &atomic->win_config[i].win;

		if (win->state == DPU_WIN_STATE_DISABLED)
			continue;

		if (win->state < DPU_WIN_STATE_DISABLED ||
				win->state > DPU_WIN_STATE_BUFFER) {
			pr_warn("%s: invalid win state %d\n", __func__, win->state);
			continue;
		}
		dpu_print_log_win_config(&atomic->win_config[i], false, p);
	}

	win = &atomic->rcd_win_config.win;
	if (win->state == DPU_WIN_STATE_BUFFER) {
		dpu_print_log_win_config(&atomic->rcd_win_config, true, p);
	}
}

static void dpu_print_log_rsc(char *buf, int len, u32 decon_id, struct dpu_log_rsc_occupancy *rsc)
{
	int i, len_chs, len_wins;
	char str_chs[128];
	char str_wins[128];
	bool using_ch, using_win;

	len_chs = sprintf(str_chs, "CHs: ");
	len_wins = sprintf(str_wins, "WINs: ");

	for (i = 0; i < MAX_PLANE; ++i) {
		using_ch = is_decon_using_ch(decon_id, rsc->rsc_ch, i);
		len_chs += sprintf(str_chs + len_chs, "%d[%c] ", i,
				using_ch ? 'O' : 'X');

		using_win = is_decon_using_win(decon_id, rsc->rsc_win, i);
		len_wins += sprintf(str_wins + len_wins, "%d[%c] ", i,
				using_win ? 'O' : 'X');
	}

	sprintf(buf + len, "\t%s\t%s", str_chs, str_wins);
}

#define LOG_BUF_SIZE	176
static int dpu_print_log_bts_update(char *buf, int len, struct dpu_log_bts_update *update)
{
	return scnprintf(buf + len, LOG_BUF_SIZE - len,
		"\tmif(%lu) int(%lu) disp(%lu) peak(%u,%u) rt(%u,%u) total(%u,%u) rd(%u,%u) scen(%u)",
		update->freqs.mif_freq, update->freqs.int_freq, update->freqs.disp_freq,
		update->prev_peak, update->peak, update->prev_rt_avg_bw, update->rt_avg_bw,
		update->prev_total_bw, update->total_bw,
		update->prev_urgent_rd_lat, update->urgent_rd_lat, update->dpu_scen_idx);
}

static int dpu_print_log_partial(char *buf, int len, struct dpu_log_partial *p)
{
	len += scnprintf(buf + len, LOG_BUF_SIZE - len,
			"\treq[%d %d %d %d] adj[%d %d %d %d] prev[%d %d %d %d]",
			p->req.x1, p->req.y1,
			drm_rect_width(&p->req), drm_rect_height(&p->req),
			p->adj.x1, p->adj.y1,
			drm_rect_width(&p->adj), drm_rect_height(&p->adj),
			p->prev.x1, p->prev.y1,
			drm_rect_width(&p->prev), drm_rect_height(&p->prev));
	return scnprintf(buf + len, LOG_BUF_SIZE - len,
			" reconfig(%d)", p->reconfigure);
}

static const char *get_event_name(enum dpu_event_type type)
{
	static const char events[][32] = {
		"NONE",
		"DECON_ENABLED",
		"DECON_DISABLED",
		"DECON_FRAMEDONE",
		"DECON_FRAMESTART",
		"DECON_RSC_OCCUPANCY",
		"DECON_TRIG_MASK",
		"DECON_UPDATE_CONFIG",
		"DSIM_ENABLED",
		"DSIM_DISABLED",
		"DSIM_COMMAND",
		"DSIM_ULPS_ENTER",
		"DSIM_ULPS_EXIT",
		"DSIM_UNDERRUN",
		"DSIM_FRAMEDONE",
		"DSIM_PH_FIFO_TIMEOUT",
		"DSIM_PL_FIFO_TIMEOUT",
		"DPP_FRAMEDONE",
		"DPP_SET_PROTECTION",
		"DMA_RECOVERY",
		"IDMA_AFBC_CONFLICT",
		"IDMA_FBC_ERROR",
		"IDMA_READ_SLAVE_ERROR",
		"IDMA_DEADLOCK",
		"IDMA_CFG_ERROR",
		"ATOMIC_COMMIT",
		"TE_INTERRUPT",
		"DECON_RUNTIME_SUSPEND",
		"DECON_RUNTIME_RESUME",
		"DECON_SUSPEND",
		"DECON_RESUME",
		"DSIM_RUNTIME_SUSPEND",
		"DSIM_RUNTIME_RESUME",
		"DSIM_SUSPEND",
		"DSIM_RESUME",
		"ENTER_HIBERNATION_IN",
		"ENTER_HIBERNATION_OUT",
		"EXIT_HIBERNATION_IN",
		"EXIT_HIBERNATION_OUT",
		"ATOMIC_BEGIN",
		"ATOMIC_FLUSH",
		"WB_ENABLE",
		"WB_DISABLE",
		"WB_ATOMIC_COMMIT",
		"WB_FRAMEDONE",
		"WB_ENTER_HIBERNATION",
		"WB_EXIT_HIBERNATION",
		"PREPARE_FB",
		"CLEANUP_FB",
		"PLANE_UPDATE",
		"PLANE_DISABLE",
		"REQ_CRTC_INFO_OLD",
		"REQ_CRTC_INFO_NEW",
		"FRAMESTART_TIMEOUT",
		"BTS_RELEASE_BW",
		"BTS_CALC_BW",
		"BTS_UPDATE_BW",
		"PARTIAL_INIT",
		"PARTIAL_PREPARE",
		"PARTIAL_UPDATE",
		"PARTIAL_PESTORE",
		"HIST_COLLECT_BINS",
		"DSIM_CRC",
		"DSIM_ECC",
		"VBLANK_ENABLE",
		"VBLANK_DISABLE",
		"DIMMING_START",
		"DIMMING_END",
		"CGC_FRAMEDONE",
		"ITMON_ERROR",
		"SYSMMU_FAULT",
	};

	if (type >= DPU_EVT_MAX)
		return NULL;

	return events[type];
}

static bool is_skip_dpu_event_dump(enum dpu_event_type type, enum dpu_event_condition condition)
{
	if (condition == DPU_EVT_CONDITION_DEFAULT)
		return false;

	if (condition == DPU_EVT_CONDITION_UNDERRUN) {
		switch (type) {
		case DPU_EVT_DECON_FRAMEDONE:
		case DPU_EVT_DECON_FRAMESTART:
		case DPU_EVT_DSIM_COMMAND:
		case DPU_EVT_DSIM_ULPS_ENTER:
		case DPU_EVT_DSIM_ULPS_EXIT:
		case DPU_EVT_DSIM_RUNTIME_SUSPEND:
		case DPU_EVT_DSIM_RUNTIME_RESUME:
		case DPU_EVT_DSIM_SUSPEND:
		case DPU_EVT_DSIM_RESUME:
		case DPU_EVT_DSIM_UNDERRUN:
		case DPU_EVT_DSIM_FRAMEDONE:
		case DPU_EVT_ATOMIC_COMMIT:
		case DPU_EVT_TE_INTERRUPT:
		case DPU_EVT_DECON_RUNTIME_SUSPEND:
		case DPU_EVT_DECON_RUNTIME_RESUME:
		case DPU_EVT_DECON_SUSPEND:
		case DPU_EVT_DECON_RESUME:
		case DPU_EVT_ENTER_HIBERNATION_IN:
		case DPU_EVT_ENTER_HIBERNATION_OUT:
		case DPU_EVT_EXIT_HIBERNATION_IN:
		case DPU_EVT_EXIT_HIBERNATION_OUT:
		case DPU_EVT_ATOMIC_BEGIN:
		case DPU_EVT_ATOMIC_FLUSH:
		case DPU_EVT_PLANE_PREPARE_FB:
		case DPU_EVT_PLANE_CLEANUP_FB:
		case DPU_EVT_PLANE_UPDATE:
		case DPU_EVT_PLANE_DISABLE:
		case DPU_EVT_BTS_RELEASE_BW:
		case DPU_EVT_BTS_CALC_BW:
		case DPU_EVT_BTS_UPDATE_BW:
		case DPU_EVT_DECON_RSC_OCCUPANCY:
			return false;
		default:
			return true;
		}
	}

	if (condition == DPU_EVT_CONDITION_FAIL_UPDATE_BW) {
		switch (type) {
		case DPU_EVT_ATOMIC_COMMIT:
		case DPU_EVT_BTS_RELEASE_BW:
		case DPU_EVT_BTS_CALC_BW:
		case DPU_EVT_BTS_UPDATE_BW:
			return false;
		default:
			return true;
		}
	}

	if (condition == DPU_EVT_CONDITION_FIFO_TIMEOUT) {
		switch (type) {
		case DPU_EVT_DECON_FRAMEDONE:
		case DPU_EVT_DECON_FRAMESTART:
		case DPU_EVT_DSIM_COMMAND:
		case DPU_EVT_DSIM_ULPS_ENTER:
		case DPU_EVT_DSIM_ULPS_EXIT:
		case DPU_EVT_DSIM_RUNTIME_SUSPEND:
		case DPU_EVT_DSIM_RUNTIME_RESUME:
		case DPU_EVT_DSIM_SUSPEND:
		case DPU_EVT_DSIM_RESUME:
		case DPU_EVT_DSIM_FRAMEDONE:
		case DPU_EVT_DSIM_PH_FIFO_TIMEOUT:
		case DPU_EVT_DSIM_PL_FIFO_TIMEOUT:
		case DPU_EVT_ATOMIC_COMMIT:
		case DPU_EVT_TE_INTERRUPT:
		case DPU_EVT_DECON_RUNTIME_SUSPEND:
		case DPU_EVT_DECON_RUNTIME_RESUME:
		case DPU_EVT_DECON_SUSPEND:
		case DPU_EVT_DECON_RESUME:
		case DPU_EVT_ENTER_HIBERNATION_OUT:
		case DPU_EVT_EXIT_HIBERNATION_OUT:
		case DPU_EVT_ATOMIC_BEGIN:
		case DPU_EVT_ATOMIC_FLUSH:
			return false;
		default:
			return true;
		}
	}

	if (condition == DPU_EVT_CONDITION_IDMA_ERROR ||
		condition == DPU_EVT_CONDITION_IDMA_ERROR_COMPACT) {
		switch (type) {
		case DPU_EVT_DECON_FRAMEDONE:
		case DPU_EVT_DECON_FRAMESTART:
		case DPU_EVT_DSIM_FRAMEDONE:
		case DPU_EVT_DPP_FRAMEDONE:
		case DPU_EVT_DMA_RECOVERY:
		case DPU_EVT_DPP_SET_PROTECTION:
		case DPU_EVT_IDMA_AFBC_CONFLICT:
		case DPU_EVT_IDMA_FBC_ERROR:
		case DPU_EVT_IDMA_READ_SLAVE_ERROR:
		case DPU_EVT_IDMA_DEADLOCK:
		case DPU_EVT_IDMA_CFG_ERROR:
		case DPU_EVT_SYSMMU_FAULT:
		case DPU_EVT_ATOMIC_COMMIT:
		case DPU_EVT_TE_INTERRUPT:
		case DPU_EVT_DSIM_RUNTIME_SUSPEND:
		case DPU_EVT_DSIM_RUNTIME_RESUME:
		case DPU_EVT_DSIM_SUSPEND:
		case DPU_EVT_DSIM_RESUME:
		case DPU_EVT_DECON_RUNTIME_SUSPEND:
		case DPU_EVT_DECON_RUNTIME_RESUME:
		case DPU_EVT_DECON_SUSPEND:
		case DPU_EVT_DECON_RESUME:
		case DPU_EVT_ENTER_HIBERNATION_OUT:
		case DPU_EVT_EXIT_HIBERNATION_OUT:
		case DPU_EVT_ATOMIC_BEGIN:
		case DPU_EVT_ATOMIC_FLUSH:
		case DPU_EVT_PLANE_UPDATE:
		case DPU_EVT_PLANE_DISABLE:
			return false;
		default:
			return true;
		}
	}

	return false;
}

static void dpu_event_log_print(const struct decon_device *decon, struct drm_printer *p,
				size_t max_logs, enum dpu_event_condition condition)
{
	int idx = atomic_read(&decon->d.event_log_idx);
	struct decon_device *decon_dev = (struct decon_device *)decon;
	struct dpu_log dump_log;
	struct dpu_log *log = &dump_log;
	int latest = idx % dpu_event_log_max;
	unsigned long rem_nsec;
	u64 ts;
	const char *str_comp;
	char buf[LOG_BUF_SIZE];
	const struct dpu_fmt *fmt;
	int len;
	unsigned long flags;

	if (IS_ERR_OR_NULL(decon_dev->d.event_log))
		return;

	drm_printf(p, "----------------------------------------------------\n");
	drm_printf(p, "%14s  %20s  %20s\n", "Time", "Event ID", "Remarks");
	drm_printf(p, "----------------------------------------------------\n");

	/* Seek a oldest from current index */
	if (max_logs > dpu_event_log_max)
		max_logs = dpu_event_log_max;

	if (idx < max_logs)
		idx = 0;
	else
		idx = (idx - max_logs) % dpu_event_log_max;

	do {
		if (++idx >= dpu_event_log_max)
			idx = 0;

		/* Seek a index and copy log for dump */
		spin_lock_irqsave(&decon_dev->d.event_lock, flags);
		memcpy(&dump_log, &decon_dev->d.event_log[idx], sizeof(dump_log));
		spin_unlock_irqrestore(&decon_dev->d.event_lock, flags);

		if (is_skip_dpu_event_dump(log->type, condition))
			continue;

		/* If there is no timestamp, then exit directly */
		ts = log->ts_nsec;
		if (!ts)
			break;

		rem_nsec = do_div(ts, 1000000000);
		len = scnprintf(buf, sizeof(buf), "<%6llu.%06lu> %20s",
				ts, rem_nsec / 1000, get_event_name(log->type));

		switch (log->type) {
		case DPU_EVT_DECON_RSC_OCCUPANCY:
			dpu_print_log_rsc(buf, len, decon->id, &log->data.rsc);
			break;
		case DPU_EVT_DSIM_COMMAND:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tCMD_ID: 0x%x\tDATA[0]: 0x%x len: %d",
					log->data.cmd.id, log->data.cmd.d0,
					log->data.cmd.len);
			break;
		case DPU_EVT_DPP_FRAMEDONE:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tCH:%u WIN:%u", log->data.dpp.id, log->data.dpp.win_id);
			break;
		case DPU_EVT_DPP_SET_PROTECTION:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tID:%u mst_security:%#x PID: %d",
					log->data.dpp.id,
					log->data.dpp.mst_security,
					log->data.dpp.last_secure_pid);
			break;
		case DPU_EVT_DMA_RECOVERY:
			str_comp = get_comp_src_name(log->data.dpp.comp_src);
			scnprintf(buf + len, sizeof(buf) - len,
					"\tCH:%u WIN:%u SRC:%s COUNT:%u",
					log->data.dpp.id, log->data.dpp.win_id,
					str_comp, log->data.dpp.recovery_cnt);
			break;
		case DPU_EVT_IDMA_AFBC_CONFLICT:
		case DPU_EVT_IDMA_FBC_ERROR:
		case DPU_EVT_IDMA_READ_SLAVE_ERROR:
		case DPU_EVT_IDMA_DEADLOCK:
		case DPU_EVT_IDMA_CFG_ERROR:
			scnprintf(buf + len, sizeof(buf) - len,
				"\tCH:%d WIN:%d SRC:%llu",
				log->data.dpp.id, log->data.dpp.win_id,
				log->data.dpp.comp_src);
			break;
		case DPU_EVT_DECON_RUNTIME_SUSPEND:
		case DPU_EVT_DECON_RUNTIME_RESUME:
		case DPU_EVT_DECON_SUSPEND:
		case DPU_EVT_DECON_RESUME:
		case DPU_EVT_ENTER_HIBERNATION_IN:
		case DPU_EVT_ENTER_HIBERNATION_OUT:
		case DPU_EVT_EXIT_HIBERNATION_IN:
		case DPU_EVT_EXIT_HIBERNATION_OUT:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tDPU POWER:%s DECON STATE:%u",
					log->data.pd.rpm_active ? "ON" : "OFF",
					log->data.pd.decon_state);
			break;
		case DPU_EVT_DECON_UPDATE_CONFIG:
			scnprintf(buf + len, sizeof(buf) - len,
				  "\t%s mode, %s_trigger, out type:0x%x, dsi_mode:%d.(%dx%d@%dhz)",
				  log->data.decon_cfg.mode.op_mode ? "command" : "video",
				  log->data.decon_cfg.mode.trig_mode ? "sw" : "hw",
				  log->data.decon_cfg.out_type, log->data.decon_cfg.mode.dsi_mode,
				  log->data.decon_cfg.image_width, log->data.decon_cfg.image_height,
				  log->data.decon_cfg.fps);
			break;
		case DPU_EVT_DSIM_RUNTIME_SUSPEND:
		case DPU_EVT_DSIM_RUNTIME_RESUME:
		case DPU_EVT_DSIM_SUSPEND:
		case DPU_EVT_DSIM_RESUME:
			scnprintf(buf + len, sizeof(buf) - len,
				  "\tDPU POWER:%s DSIM STATE:%u DSIM POWER:%s",
				  log->data.pd.rpm_active ? "ON" : "OFF", log->data.pd.dsim_state,
				  log->data.pd.dsim_rpm_active ? "ON" : "OFF");
			break;
		case DPU_EVT_PLANE_PREPARE_FB:
		case DPU_EVT_PLANE_CLEANUP_FB:
			fmt = dpu_find_fmt_info(log->data.plane_info.format);
			scnprintf(buf + len, sizeof(buf) - len, "\tCH%u: 0x%llx, %ux%u, ZPOS%u, %s",
				PLANE2DPPCH(log->data.plane_info.index),
				log->data.plane_info.dma_addr,
				log->data.plane_info.width, log->data.plane_info.height,
				log->data.plane_info.zpos, dpu_get_fmt_name(fmt));
			break;
		case DPU_EVT_PLANE_UPDATE:
		case DPU_EVT_PLANE_DISABLE:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tCH:%d, WIN:%d, %s%s",
					log->data.win.plane_idx,
					log->data.win.win_idx,
					log->data.win.secure ? "SECURE " : "",
					log->data.win.hdr_en ? "HDR" : "");
			break;
		case DPU_EVT_REQ_CRTC_INFO_OLD:
		case DPU_EVT_REQ_CRTC_INFO_NEW:
			scnprintf(buf + len, sizeof(buf) - len,
				"\tenable(%d) active(%d) sr(%d) [p:%d m:%d a:%d c:%d]",
					log->data.crtc_info.enable,
					log->data.crtc_info.active,
					log->data.crtc_info.self_refresh,
					log->data.crtc_info.planes_changed,
					log->data.crtc_info.mode_changed,
					log->data.crtc_info.active_changed,
					log->data.crtc_info.connectors_changed);
			break;
		case DPU_EVT_BTS_RELEASE_BW:
		case DPU_EVT_BTS_UPDATE_BW:
			dpu_print_log_bts_update(buf, len, &log->data.bts_update);
			break;
		case DPU_EVT_BTS_CALC_BW:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tdisp(%u) peak(%u) rt(%u) read(%u) write(%u) %uhz (crtc: %uhz, op: %uhz)",
					log->data.bts_cal.disp_freq, log->data.bts_cal.peak,
					log->data.bts_cal.rt_avg_bw, log->data.bts_cal.read_bw,
					log->data.bts_cal.write_bw, log->data.bts_cal.fps,
					log->data.bts_cal.crtc_fps, log->data.bts_cal.op_rate);
			break;
		case DPU_EVT_DSIM_UNDERRUN:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tunderrun count(%u)",
					log->data.bts_event.value);
			break;
		case DPU_EVT_PARTIAL_INIT:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tminimum rect size[%dx%d]",
					log->data.partial.min_w,
					log->data.partial.min_h);
			break;
		case DPU_EVT_PARTIAL_PREPARE:
			len += dpu_print_log_partial(buf, len,
					&log->data.partial);
			break;
		case DPU_EVT_PARTIAL_RESTORE:
		case DPU_EVT_PARTIAL_UPDATE:
			scnprintf(buf + len, sizeof(buf) - len,
				"\t[%d %d %d %d]",
				log->data.partial.prev.x1,
				log->data.partial.prev.y1,
				drm_rect_width(&log->data.partial.prev),
				drm_rect_height(&log->data.partial.prev));
			break;
		case DPU_EVT_HIST_COLLECT_BINS:
			scnprintf(buf + len, sizeof(buf) - len,
					"\thist_id: %d",
					log->data.hist_info.id);
			break;
		case DPU_EVT_DSIM_CRC:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tcrc count(%u)",
					log->data.value);
			break;
		case DPU_EVT_DSIM_ECC:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tecc count(%u)",
					log->data.value);
			break;
		case DPU_EVT_TE_INTERRUPT:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tte cnt(%u)",
					log->data.value);
			break;
		default:
			break;
		}

		drm_printf(p, "%s\n", buf);

		switch (log->type) {
		case DPU_EVT_ATOMIC_COMMIT:
			dpu_print_log_atomic(&log->data.atomic, p);
			break;
		default:
			break;
		}
	} while (latest != idx);

	drm_printf(p, "----------------------------------------------------\n");
}

static int dpu_debug_event_show(struct seq_file *s, void *unused)
{
	struct decon_device *decon = s->private;
	struct drm_printer p = drm_seq_file_printer(s);

	dpu_event_log_print(decon, &p, dpu_event_log_max, DPU_EVT_CONDITION_DEFAULT);
	return 0;
}

static int dpu_debug_event_open(struct inode *inode, struct file *file)
{
	return single_open(file, dpu_debug_event_show, inode->i_private);
}

static const struct file_operations dpu_event_fops = {
	.open = dpu_debug_event_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int dpu_debug_reg_dump_show(struct seq_file *s, void *unused)
{
	struct decon_device *decon = s->private;
	struct dsim_device *dsim;
	struct drm_printer p = drm_seq_file_printer(s);

	if (decon->state == DECON_STATE_ON &&
		pm_runtime_get_if_in_use(decon->dev) == 1) {
		decon_dump(decon, &p);
		dsim = decon_get_dsim(decon);
		if (dsim) {
			dsim_dump(dsim, &p);
			if (dsim->dual_dsi == DSIM_DUAL_DSI_MAIN) {
				dsim = exynos_get_dual_dsi(DSIM_DUAL_DSI_SEC);
				if (dsim)
					dsim_dump(dsim, &p);
			}
		}
		pm_runtime_put(decon->dev);
	} else {
		drm_printf(&p, "%s[%u]: DECON disabled(%d)\n",
			decon->dev->driver->name, decon->id, decon->state);
	}

	return 0;
}

static int dpu_debug_reg_dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, dpu_debug_reg_dump_show, inode->i_private);
}

static const struct file_operations dpu_reg_dump_fops = {
	.open = dpu_debug_reg_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static bool is_dqe_supported(struct drm_device *drm_dev, u32 dqe_id)
{
	struct drm_crtc *crtc;
	struct decon_device *decon;

	drm_for_each_crtc(crtc, drm_dev) {
		decon = crtc_to_decon(crtc);
		if ((decon->id == dqe_id) && decon->dqe)
			return true;
	}

	return false;
}

static int dump_show(struct seq_file *s, void *unused)
{
	struct debugfs_dump *dump = s->private;
	struct drm_device *drm_dev = dump->priv;
	struct drm_printer p = drm_seq_file_printer(s);

	if (!drm_dev || !is_power_on(drm_dev))
		return 0;

	if (dump->type <= DUMP_TYPE_DQE_MAX)
		if (!is_dqe_supported(drm_dev, dump->id))
			return 0;

	if (dump->type == DUMP_TYPE_CGC_LUT)
		dqe_reg_print_cgc_lut(dump->id, CGC_LUT_SIZE, &p);
	else if (dump->type == DUMP_TYPE_DEGAMMA_LUT)
		dqe_reg_print_degamma_lut(dump->id, &p);
	else if (dump->type == DUMP_TYPE_REGAMMA_LUT)
		dqe_reg_print_regamma_lut(dump->id, &p);
	else if (dump->type == DUMP_TYPE_GAMMA_MATRIX)
		dqe_reg_print_gamma_matrix(dump->id, &p);
	else if (dump->type == DUMP_TYPE_LINEAR_MATRIX)
		dqe_reg_print_linear_matrix(dump->id, &p);
	else if (dump->type == DUMP_TYPE_ATC)
		dqe_reg_print_atc(dump->id, &p);
	else if (dump->type == DUMP_TYPE_DISP_DITHER ||
			dump->type == DUMP_TYPE_CGC_DIHTER)
		dqe_reg_print_dither(dump->id, dump->dither_type, &p);
	else if (dump->type == DUMP_TYPE_HISTOGRAM)
		dqe_reg_print_hist(dump->id, &p);

	else if (dump->type == DUMP_TYPE_HDR_EOTF)
		hdr_reg_print_eotf_lut(dump->id, &p);
	else if (dump->type == DUMP_TYPE_HDR_OETF)
		hdr_reg_print_oetf_lut(dump->id, &p);
	else if (dump->type == DUMP_TYPE_HDR_GAMMUT)
		hdr_reg_print_gm(dump->id, &p);
	else if (dump->type == DUMP_TYPE_HDR_TONEMAP)
		hdr_reg_print_tm(dump->id, &p);

	return 0;
}

static int dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, dump_show, inode->i_private);
}

static const struct file_operations dump_fops = {
	.open	 = dump_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release,
};

static void exynos_debugfs_add_dump(enum dump_type type, umode_t mode,
		struct dentry *parent, u32 id, enum dqe_dither_type dither,
		struct drm_device *drm_dev)
{
	struct debugfs_dump *dump = drmm_kzalloc(drm_dev,
			sizeof(struct debugfs_dump), GFP_KERNEL);

	dump->type = type;
	dump->id = id;
	dump->dither_type = dither;
	dump->priv = drm_dev;
	debugfs_create_file("dump", mode, parent, dump, &dump_fops);
}

static struct dentry *exynos_debugfs_add_dqe_override(const char *name,
			enum dump_type dump, enum dqe_dither_type dither,
			struct dither_debug_override *d, struct dentry *parent,
			struct drm_device *drm_dev)
{
	struct dentry *dent;

	dent = debugfs_create_dir(name, parent);
	if (!dent) {
		pr_err("failed to create %s dir\n", name);
		return NULL;
	}
	debugfs_create_bool("force_enable", 0664, dent, &d->force_en);
	debugfs_create_bool("verbose", 0664, dent, &d->verbose);
	debugfs_create_u32("val", 0664, dent, (u32 *)&d->val);

	exynos_debugfs_add_dump(dump, 0444, dent, 0, dither, drm_dev);

	return dent;
}

static int get_lut(char *lut_buf, u32 count, void **lut, enum elem_size elem_size)
{
	int i = 0, ret = 0;
	char *token;
	u16 *plut16;
	u32 *plut32;

	if (elem_size == ELEM_SIZE_16)
		plut16 = *lut;
	else if (elem_size == ELEM_SIZE_32)
		plut32 = *lut;
	else
		return -EINVAL;

	while ((token = strsep(&lut_buf, " "))) {
		if (i >= count)
			break;

		if (elem_size == ELEM_SIZE_16)
			ret = kstrtou16(token, 0, &plut16[i++]);
		else if (elem_size == ELEM_SIZE_32)
			ret = kstrtou32(token, 0, &plut32[i++]);

		if (ret)
			return -EINVAL;
	}

	return 0;
}

static int lut_show(struct seq_file *s, void *unused)
{
	struct debugfs_lut *lut = s->private;
	struct drm_printer p = drm_seq_file_printer(s);
	char buf[128] = {0};
	int len = 0;
	int i;
	u16 *plut16;
	u32 *plut32;

	if (lut->elem_size == ELEM_SIZE_16)
		plut16 = lut->lut_ptr;
	else if (lut->elem_size == ELEM_SIZE_32)
		plut32 = lut->lut_ptr;
	else
		return -EINVAL;

	if (!lut->pcount || lut->pcount > lut->count)
		lut->pcount = lut->count;

	for (i = 0; i < lut->pcount; ++i) {
		if (lut->elem_size == ELEM_SIZE_16)
			len += sprintf(buf + len, "[%2d] %4x  ", i, plut16[i]);
		else if (lut->elem_size == ELEM_SIZE_32)
			len += sprintf(buf + len, "[%2d] %4x  ", i, plut32[i]);

		if ((i % 4) == 3) {
			drm_printf(&p, "%s\n", buf);
			len = 0;
			memset(buf, 0, sizeof(buf));
		}
	}

	if (len)
		drm_printf(&p, "%s\n", buf);

	return 0;
}

static int lut_open(struct inode *inode, struct file *file)
{
	return single_open(file, lut_show, inode->i_private);
}

static ssize_t lut_write(struct file *file, const char __user *buffer,
		size_t len, loff_t *ppos)
{
	char *tmpbuf;
	struct debugfs_lut *lut =
		((struct seq_file *)file->private_data)->private;
	struct drm_color_lut *dlut = lut->dlut_ptr;
	int ret, i;
	u16 *plut;

	if (len == 0)
		return 0;

	tmpbuf = memdup_user_nul(buffer, len);
	if (IS_ERR(tmpbuf))
		return PTR_ERR(tmpbuf);

	pr_debug("read %d bytes from userspace\n", (int)len);

	ret = get_lut(tmpbuf, lut->count, &lut->lut_ptr, lut->elem_size);
	if (ret)
		goto err;

	if (dlut) {
		plut = lut->lut_ptr;

		for (i = 0; i < lut->count; ++i) {
			if (!strcmp(lut->name, "red") ||
					!strcmp(lut->name, "lut"))
				dlut[i].red  = plut[i];
			else if (!strcmp(lut->name, "green"))
				dlut[i].green = plut[i];
			else if (!strcmp(lut->name, "blue"))
				dlut[i].blue = plut[i];
		}
	}

	*(lut->dirty) = true;

	ret = len;
err:
	kfree(tmpbuf);

	return ret;
}

static const struct file_operations lut_fops = {
	.open	 = lut_open,
	.read	 = seq_read,
	.write	 = lut_write,
	.llseek	 = seq_lseek,
	.release = seq_release,
};

static void exynos_debugfs_add_lut(const char *name, umode_t mode,
		struct dentry *parent, size_t count, size_t pcount,
		void *lut_ptr, struct drm_color_lut *dlut_ptr,
		enum elem_size elem_size, bool *dirty)
{
	struct debugfs_lut *lut = kmalloc(sizeof(struct debugfs_lut),
			GFP_KERNEL);
	if (!lut)
		return;

	if (!lut_ptr) {
		lut_ptr = kmalloc(count * (elem_size >> 3), GFP_KERNEL);
		if (!lut_ptr) {
			kfree(lut);
			return;
		}
	}

	strncpy(lut->name, name, MAX_NAME_SIZE);
	lut->lut_ptr = lut_ptr;
	lut->dlut_ptr = dlut_ptr;
	lut->elem_size = elem_size;
	lut->count = count;
	lut->pcount = pcount;
	lut->dirty = dirty;

	debugfs_create_file(name, mode, parent, lut, &lut_fops);
}

#define DEFAULT_PRINT_CNT	128
static struct dentry *
exynos_debugfs_add_cgc(struct cgc_debug_override *cgc, struct dentry *parent,
							struct drm_device *drm)
{
	struct dentry *dent, *dent_lut;
	struct exynos_debug_info *info = &cgc->info;

	dent = debugfs_create_dir("cgc", parent);
	if (!dent) {
		pr_err("failed to create cgc directory\n");
		return NULL;
	}

	debugfs_create_bool("force_enable", 0664, dent, &info->force_en);
	debugfs_create_bool("verbose", 0664, dent, &info->verbose);
	debugfs_create_u32("verbose_count", 0664, dent, &cgc->verbose_cnt);
	cgc->verbose_cnt = DEFAULT_PRINT_CNT;
	exynos_debugfs_add_dump(DUMP_TYPE_CGC_LUT, 0444, dent, 0, 0, drm);

	dent_lut = debugfs_create_dir("lut", dent);
	if (!dent_lut) {
		pr_err("failed to create cgc lut directory\n");
		debugfs_remove_recursive(dent);
		return NULL;
	}

	exynos_debugfs_add_lut("red", 0664, dent_lut,
			DRM_SAMSUNG_CGC_LUT_REG_CNT, cgc->verbose_cnt,
			cgc->force_lut.r_values, NULL, ELEM_SIZE_32, &info->dirty);
	exynos_debugfs_add_lut("green", 0664, dent_lut,
			DRM_SAMSUNG_CGC_LUT_REG_CNT, cgc->verbose_cnt,
			cgc->force_lut.g_values, NULL, ELEM_SIZE_32, &info->dirty);
	exynos_debugfs_add_lut("blue", 0664, dent_lut,
			DRM_SAMSUNG_CGC_LUT_REG_CNT, cgc->verbose_cnt,
			cgc->force_lut.b_values, NULL, ELEM_SIZE_32, &info->dirty);

	return dent;
}

static struct dentry *
exynos_debugfs_add_regamma(struct regamma_debug_override *regamma,
				struct dentry *parent, struct drm_device *drm)
{
	struct dentry *dent, *dent_lut;
	struct exynos_debug_info *info = &regamma->info;

	dent = debugfs_create_dir("regamma", parent);
	if (!dent) {
		pr_err("failed to create regamma directory\n");
		return NULL;
	}

	debugfs_create_bool("force_enable", 0664, dent, &info->force_en);
	debugfs_create_bool("verbose", 0664, dent, &info->verbose);
	exynos_debugfs_add_dump(DUMP_TYPE_REGAMMA_LUT, 0444, dent, 0, 0, drm);

	dent_lut = debugfs_create_dir("lut", dent);
	if (!dent_lut) {
		pr_err("failed to create regamma lut directory\n");
		debugfs_remove_recursive(dent);
		return NULL;
	}

	exynos_debugfs_add_lut("red", 0664, dent_lut, REGAMMA_LUT_SIZE, 0,
			NULL, regamma->force_lut, ELEM_SIZE_16, &info->dirty);
	exynos_debugfs_add_lut("green", 0664, dent_lut, REGAMMA_LUT_SIZE, 0,
			NULL, regamma->force_lut, ELEM_SIZE_16, &info->dirty);
	exynos_debugfs_add_lut("blue", 0664, dent_lut, REGAMMA_LUT_SIZE, 0,
			NULL, regamma->force_lut, ELEM_SIZE_16, &info->dirty);

	return dent;
}

static struct dentry *
exynos_debugfs_add_degamma(struct degamma_debug_override *degamma,
				struct dentry *parent, struct drm_device *drm)
{
	struct dentry *dent;
	struct exynos_debug_info *info = &degamma->info;

	dent = debugfs_create_dir("degamma", parent);
	if (!dent) {
		pr_err("failed to create degamma directory\n");
		return NULL;
	}

	debugfs_create_bool("force_enable", 0664, dent, &info->force_en);
	debugfs_create_bool("verbose", 0664, dent, &info->verbose);
	exynos_debugfs_add_lut("lut", 0664, dent, DEGAMMA_LUT_SIZE, 0, NULL,
			degamma->force_lut, ELEM_SIZE_16, &info->dirty);
	exynos_debugfs_add_dump(DUMP_TYPE_DEGAMMA_LUT, 0444, dent, 0, 0, drm);

	return dent;
}

static struct dentry *exynos_debugfs_add_histogram(struct exynos_dqe *dqe,
		struct dentry *parent, struct drm_device *drm)
{
	struct dentry *dent;

	dent = debugfs_create_dir("histogram", parent);
	if (!dent) {
		pr_err("failed to create histogram directory\n");
		return NULL;
	}

	debugfs_create_bool("verbose", 0664, dent, &dqe->verbose_hist);
	exynos_debugfs_add_dump(DUMP_TYPE_HISTOGRAM, 0444, dent, 0, 0, drm);

	return dent;
}

static struct dentry *exynos_debugfs_add_atc(struct exynos_dqe *dqe,
		struct dentry *parent, struct drm_device *drm)
{
	struct dentry *dent;

	dent = debugfs_create_dir("atc", parent);
	if (!dent) {
		pr_err("failed to create atc directory\n");
		return NULL;
	}

	debugfs_create_bool("verbose", 0664, dent, &dqe->verbose_atc);
	exynos_debugfs_add_dump(DUMP_TYPE_ATC, 0444, dent, 0, 0, drm);

	return dent;
}

static struct dentry *
exynos_debugfs_add_matrix(struct matrix_debug_override *matrix,
				const char *name, struct dentry *parent,
				enum dump_type dump, struct drm_device *drm)
{
	struct dentry *dent, *dent_matrix;
	struct exynos_debug_info *info = &matrix->info;
	u32 coeffs_cnt = DRM_SAMSUNG_MATRIX_DIMENS * DRM_SAMSUNG_MATRIX_DIMENS;
	u32 offsets_cnt = DRM_SAMSUNG_MATRIX_DIMENS;

	dent = debugfs_create_dir(name, parent);
	if (!dent) {
		pr_err("failed to create %s matrix directory\n", name);
		return NULL;
	}

	debugfs_create_bool("force_enable", 0664, dent, &info->force_en);
	debugfs_create_bool("verbose", 0664, dent, &info->verbose);
	exynos_debugfs_add_dump(dump, 0444, dent, 0, 0, drm);
	dent_matrix = debugfs_create_dir("matrix", dent);
	if (!dent_matrix) {
		pr_err("failed to create %s directory\n", name);
		debugfs_remove_recursive(dent);
		return NULL;
	}

	exynos_debugfs_add_lut("coeffs", 0664, dent_matrix, coeffs_cnt, 0,
			matrix->force_matrix.coeffs , NULL, ELEM_SIZE_16,
			&info->dirty);
	exynos_debugfs_add_lut("offsets", 0664, dent_matrix, offsets_cnt, 0,
			matrix->force_matrix.offsets, NULL, ELEM_SIZE_16,
			&info->dirty);

	return dent;
}

static void
exynos_debugfs_add_dqe(struct exynos_dqe *dqe, struct dentry *parent)
{
	struct dentry *dent_dir;
	struct matrix_debug_override *gamma = &dqe->gamma;
	struct matrix_debug_override *linear = &dqe->linear;
	struct drm_device *drm = dqe->decon->drm_dev;

	if (!dqe)
		return;

	dent_dir = debugfs_create_dir("dqe", parent);
	if (!dent_dir) {
		pr_err("failed to create dqe directory\n");
		return;
	}

	if (!exynos_debugfs_add_dqe_override("cgc_dither", DUMP_TYPE_CGC_DIHTER,
			CGC_DITHER, &dqe->cgc_dither_override,
			dent_dir, drm))
		goto err;

	if (!exynos_debugfs_add_dqe_override("disp_dither",
			DUMP_TYPE_DISP_DITHER, DISP_DITHER,
			&dqe->disp_dither_override, dent_dir, drm))
		goto err;

	exynos_debugfs_add_cgc(&dqe->cgc, dent_dir, drm);
	exynos_debugfs_add_regamma(&dqe->regamma, dent_dir, drm);
	exynos_debugfs_add_degamma(&dqe->degamma, dent_dir, drm);

	if (!exynos_debugfs_add_atc(dqe, dent_dir, drm))
		goto err;

	if (!exynos_debugfs_add_histogram(dqe, dent_dir, drm))
		goto err;

	if (!exynos_debugfs_add_matrix(linear, "linear_matrix", dent_dir,
				DUMP_TYPE_LINEAR_MATRIX, drm))
		goto err;

	if (!exynos_debugfs_add_matrix(gamma, "gamma_matrix", dent_dir,
				DUMP_TYPE_GAMMA_MATRIX, drm))
		goto err;

	debugfs_create_bool("force_disabled", 0664, dent_dir,
			&dqe->force_disabled);

	return;

err:
	debugfs_remove_recursive(dent_dir);
}

static ssize_t counters_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t underrun_cnt, crc_cnt, ecc_cnt, idma_err_cnt;
	const struct decon_device *decon = to_decon_device(dev);

	if (!decon)
		return -ENODEV;

	underrun_cnt = decon->d.underrun_cnt;
	crc_cnt = decon->d.crc_cnt;
	ecc_cnt = decon->d.ecc_cnt;
	idma_err_cnt = decon->d.idma_err_cnt;

	return snprintf(buf, PAGE_SIZE,
			"underrun: %u\n"
			"crc: %u\n"
			"ecc: %u\n"
			"idma_error: %u\n",
			underrun_cnt, crc_cnt, ecc_cnt, idma_err_cnt);
}

static DEVICE_ATTR_RO(counters);

static ssize_t hibernation_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct decon_device *decon = dev_get_drvdata(dev);
	struct exynos_hibernation *hiber = decon->hibernation;

	if (!decon || !hiber) {
		pr_err("%s: invalid device\n", __func__);
		return -ENODEV;
	}

	return scnprintf(buf, PAGE_SIZE,
				"state: %s\n"
				"block_count: %d\n",
				hiber->enabled ? "enabled" : "disabled",
				atomic_read(&hiber->block_cnt));
}

static ssize_t hibernation_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct decon_device *decon;
	struct exynos_hibernation *hiber;
	bool enable;

	if (!dev || !buf || !len) {
		pr_err("%s: invalid input param\n", __func__);
		return -EINVAL;
	}

	if (kstrtobool(buf, &enable) < 0)
		return -EINVAL;

	decon = dev_get_drvdata(dev);
	hiber = decon->hibernation;

	if (!decon || !hiber) {
		pr_err("%s: invalid device\n", __func__);
		return -ENODEV;
	}

	DPU_ATRACE_BEGIN(__func__);
	if (!enable) {
		/* if disabling, make sure it gets out of hibernation before disabling */
		hibernation_block_exit(hiber);
		hiber->enabled = false;
	} else {
		/* if enabling, vote to make sure hibernation is scheduled after unblocking */
		hiber->enabled = true;
		hibernation_block(hiber);
	}
	hibernation_unblock_enter(hiber);
	DPU_ATRACE_END(__func__);

	return len;
}
static DEVICE_ATTR_RW(hibernation);

static const struct attribute *decon_debug_attrs[] = {
	&dev_attr_counters.attr,
	&dev_attr_hibernation.attr,
	NULL
};

static int recovery_show(struct seq_file *s, void *unused)
{
	struct decon_device *decon = s->private;

	seq_printf(s, "%d\n", decon->recovery.count);

	return 0;
}

static int recovery_open(struct inode *inode, struct file *file)
{
	return single_open(file, recovery_show, inode->i_private);
}

static ssize_t recovery_write(struct file *file, const char *user_buf,
			      size_t count, loff_t *f_pos)
{
	struct seq_file *s = file->private_data;
	struct decon_device *decon = s->private;

	decon_trigger_recovery(decon);

	return count;
}

static const struct file_operations recovery_fops = {
	.open = recovery_open,
	.read = seq_read,
	.write = recovery_write,
	.llseek = seq_lseek,
	.release = seq_release,
};

bool is_console_enabled(void)
{
	return exynos_uart_console_enabled();
}

static void buf_dump_all(const struct decon_device *decon)
{
	struct drm_printer p = is_console_enabled() ?
		drm_debug_printer("[drm]") : drm_info_printer(decon->dev);
	int i;

	for (i = 0; i < decon->dpp_cnt; ++i)
		dpp_dump_buffer(&p, decon->dpp[i]);
}

static void buf_dump_handler(struct kthread_work *work)
{
	const struct decon_device *decon =
		container_of(work, struct decon_device, buf_dump_work);

	buf_dump_all(decon);
}

static ssize_t force_te_write(struct file *file, const char __user *buffer,
			      size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct decon_device *decon = s->private;
	unsigned long flags;
	int ret;
	bool en;

	ret = kstrtobool_from_user(buffer, len, &en);
	if (ret)
		return ret;

	spin_lock_irqsave(&decon->slock, flags);
	if (en != decon->d.force_te_on) {
		decon_enable_te_irq(decon, en);
		decon->d.force_te_on = en;
	}
	spin_unlock_irqrestore(&decon->slock, flags);

	return len;
}

static int force_te_show(struct seq_file *s, void *unused)
{
	struct decon_device *decon = s->private;

	seq_printf(s, "%d\n", decon->d.force_te_on);

	return 0;
}

static int force_te_open(struct inode *inode, struct file *file)
{
	return single_open(file, force_te_show, inode->i_private);
}

static const struct file_operations force_te_fops = {
	.open = force_te_open,
	.read = seq_read,
	.write = force_te_write,
	.llseek = seq_lseek,
	.release = seq_release,
};

static ssize_t tout_write(struct file *file, const char __user *buffer,
			  size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct decon_device *decon = s->private;
	unsigned long flags;
	int ret;
	bool en;

	ret = kstrtobool_from_user(buffer, len, &en);
	if (ret)
		return ret;

	spin_lock_irqsave(&decon->slock, flags);
	if (en != decon->d.tout_en) {
		decon_enable_tout_irq(decon, en);
		decon->d.tout_en = en;
	}
	spin_unlock_irqrestore(&decon->slock, flags);

	return len;
}

static int tout_show(struct seq_file *s, void *unused)
{
	struct decon_device *decon = s->private;

	seq_printf(s, "%d\n", decon->d.tout_en);

	return 0;
}

static int tout_open(struct inode *inode, struct file *file)
{
	return single_open(file, tout_show, inode->i_private);
}

static const struct file_operations tout_fops = {
	.open = tout_open,
	.read = seq_read,
	.write = tout_write,
	.llseek = seq_lseek,
	.release = seq_release,
};

int dpu_init_debug(struct decon_device *decon)
{
	int i;
	u32 event_cnt;
	struct drm_crtc *crtc;
	struct exynos_dqe *dqe = decon->dqe;
	struct dentry *debug_event;
	struct dentry *debug_reg_dump;
	struct dentry *urgent_dent;
	int ret;

	decon->d.event_log = NULL;
	event_cnt = dpu_event_log_max;

	for (i = 0; i < DPU_EVENT_LOG_RETRY; ++i) {
		event_cnt = event_cnt >> i;
		decon->d.event_log =
			vzalloc(sizeof(struct dpu_log) * event_cnt);
		if (IS_ERR_OR_NULL(decon->d.event_log)) {
			DRM_WARN("failed to alloc event log buf[%d]. retry\n",
					event_cnt);
			continue;
		}

		DRM_INFO("#%d event log buffers are allocated\n", event_cnt);
		break;
	}
	spin_lock_init(&decon->d.event_lock);
	decon->d.event_log_cnt = event_cnt;
	atomic_set(&decon->d.event_log_idx, -1);

	kthread_init_work(&decon->buf_dump_work, buf_dump_handler);

	if (!decon->crtc)
		goto err_event_log;

	crtc = &decon->crtc->base;

	debug_event = debugfs_create_file("event", 0444, crtc->debugfs_entry,
			decon, &dpu_event_fops);
	if (!debug_event) {
		DRM_ERROR("failed to create debugfs event file\n");
		goto err_event_log;
	}

	debug_reg_dump = debugfs_create_file("reg_dump", 0444, crtc->debugfs_entry,
			decon, &dpu_reg_dump_fops);
	if (!debug_reg_dump) {
		DRM_ERROR("failed to create debugfs reg dump file\n");
		goto err_debugfs;
	}

	if (!debugfs_create_file("recovery", 0644, crtc->debugfs_entry, decon,
				&recovery_fops)) {
		DRM_ERROR("failed to create debugfs recovery file\n");
		goto err_debugfs;
	}

	/* Create sysfs nodes */
	ret = sysfs_create_files(&decon->dev->kobj, decon_debug_attrs);
	if (ret)
		pr_warn("unable to add decon_debug sysfs files (%d)\n", ret);

	debugfs_create_file("force_te_on", 0664, crtc->debugfs_entry, decon, &force_te_fops);
	debugfs_create_file("tout_en", 0664, crtc->debugfs_entry, decon, &tout_fops);
	debugfs_create_u32("underrun_cnt", 0664, crtc->debugfs_entry, &decon->d.underrun_cnt);
	debugfs_create_u32("crc_cnt", 0444, crtc->debugfs_entry, &decon->d.crc_cnt);
	debugfs_create_u32("ecc_cnt", 0444, crtc->debugfs_entry, &decon->d.ecc_cnt);
	debugfs_create_u32("idma_err_cnt", 0444, crtc->debugfs_entry, &decon->d.idma_err_cnt);
	debugfs_create_u32("te_count", 0444, crtc->debugfs_entry, &decon->d.te_cnt);
	debugfs_create_u32("frame_done_count", 0444, crtc->debugfs_entry, &decon->d.framedone_cnt);

	urgent_dent = debugfs_create_dir("urgent", crtc->debugfs_entry);
	if (!urgent_dent) {
		DRM_ERROR("failed to create debugfs urgent directory\n");
		goto err_debugfs;
	}

	debugfs_create_u32("rd_en", 0664, urgent_dent, &decon->config.urgent.rd_en);
	debugfs_create_x32("rd_hi_thres", 0664, urgent_dent, &decon->config.urgent.rd_hi_thres);
	debugfs_create_x32("rd_lo_thres", 0664, urgent_dent, &decon->config.urgent.rd_lo_thres);
	debugfs_create_x32("rd_wait_cycle", 0664, urgent_dent, &decon->config.urgent.rd_wait_cycle);
	debugfs_create_u32("wr_en", 0664, urgent_dent, &decon->config.urgent.wr_en);
	debugfs_create_x32("wr_hi_thres", 0664, urgent_dent, &decon->config.urgent.wr_hi_thres);
	debugfs_create_x32("wr_lo_thres", 0664, urgent_dent, &decon->config.urgent.wr_lo_thres);
	debugfs_create_bool("dta_en", 0664, urgent_dent, &decon->config.urgent.dta_en);
	debugfs_create_x32("dta_hi_thres", 0664, urgent_dent, &decon->config.urgent.dta_hi_thres);
	debugfs_create_x32("dta_lo_thres", 0664, urgent_dent, &decon->config.urgent.dta_lo_thres);

	if (dqe)
		exynos_debugfs_add_dqe(dqe, crtc->debugfs_entry);

	return 0;

err_debugfs:
	debugfs_remove(debug_event);
err_event_log:
	vfree(decon->d.event_log);
	return -ENOENT;
}

#if defined(CONFIG_SOC_ZUMA)
static struct dentry *exynos_debugfs_add_hdr_lut_v2p2(const char *name,
		struct dentry *parent, struct exynos_debug_info *info,
		void *ts_lut, size_t ts_cnt, enum elem_size ts_type,
		void *vs_lut, size_t vs_cnt, enum elem_size vs_type,
		enum dump_type dump, unsigned int index,
		struct drm_device *drm)
{
	struct dentry *dent, *dent_lut;

	dent = debugfs_create_dir(name, parent);
	if (!dent) {
		pr_err("failed to create %s directory\n", name);
		return NULL;
	}

	debugfs_create_bool("force_enable", 0664, dent, &info->force_en);
	debugfs_create_bool("verbose", 0664, dent, &info->verbose);
	exynos_debugfs_add_dump(dump, 0444, dent, index, 0, drm);

	dent_lut = debugfs_create_dir("lut", dent);
	if (!dent_lut) {
		pr_err("failed to create %s lut directory\n", name);
		debugfs_remove_recursive(dent);
		return NULL;
	}

	exynos_debugfs_add_lut("ts", 0664, dent_lut, ts_cnt, 0, ts_lut,
			NULL,  ts_type, &info->dirty);
	exynos_debugfs_add_lut("vs", 0664, dent_lut, vs_cnt, 0, vs_lut,
			NULL,  vs_type, &info->dirty);

	return dent;
}
#else
static struct dentry *exynos_debugfs_add_hdr_lut(const char *name,
		struct dentry *parent, struct exynos_debug_info *info,
		void *posx_lut, size_t posx_cnt, enum elem_size posx_type,
		void *posy_lut, size_t posy_cnt, enum elem_size posy_type,
		enum dump_type dump, unsigned int index,
		struct drm_device *drm)
{
	struct dentry *dent, *dent_lut;

	dent = debugfs_create_dir(name, parent);
	if (!dent) {
		pr_err("failed to create %s directory\n", name);
		return NULL;
	}

	debugfs_create_bool("force_enable", 0664, dent, &info->force_en);
	debugfs_create_bool("verbose", 0664, dent, &info->verbose);
	exynos_debugfs_add_dump(dump, 0444, dent, index, 0, drm);

	dent_lut = debugfs_create_dir("lut", dent);
	if (!dent_lut) {
		pr_err("failed to create %s lut directory\n", name);
		debugfs_remove_recursive(dent);
		return NULL;
	}

	exynos_debugfs_add_lut("posx", 0664, dent_lut, posx_cnt, 0, posx_lut,
			NULL,  posx_type, &info->dirty);
	exynos_debugfs_add_lut("posy", 0664, dent_lut, posy_cnt, 0, posy_lut,
			NULL,  posy_type, &info->dirty);

	return dent;
}
#endif

static struct dentry *exynos_debugfs_add_gammut(struct exynos_hdr *hdr,
		struct dentry *parent, enum dump_type dump, unsigned int index,
		struct drm_device *drm)
{
	struct dentry *dent, *dent_matrix;
	struct gm_debug_override *gm = &hdr->gm;
	struct exynos_debug_info *info = &gm->info;

	dent = debugfs_create_dir("gammut", parent);
	if (!dent) {
		pr_err("failed to create gammut directory\n");
		return NULL;
	}

	debugfs_create_bool("force_enable", 0664, dent, &info->force_en);
	debugfs_create_bool("verbose", 0664, dent, &info->verbose);
	exynos_debugfs_add_dump(dump, 0444, dent, index, 0, drm);

	dent_matrix = debugfs_create_dir("matrix", dent);
	if (!dent_matrix) {
		pr_err("failed to create gammut matrix directory\n");
		debugfs_remove_recursive(dent);
		return NULL;
	}

	exynos_debugfs_add_lut("coeffs", 0664, dent_matrix,
			DRM_SAMSUNG_HDR_GM_DIMENS * DRM_SAMSUNG_HDR_GM_DIMENS,
			0, gm->force_data.coeffs, NULL, ELEM_SIZE_32, &info->dirty);
	exynos_debugfs_add_lut("offsets", 0664, dent_matrix,
			DRM_SAMSUNG_HDR_GM_DIMENS, 0,
			gm->force_data.offsets, NULL, ELEM_SIZE_32, &info->dirty);

	return dent;
}

#if defined(CONFIG_SOC_ZUMA)
static struct dentry *exynos_debugfs_add_tm_v2p2(struct exynos_hdr *hdr,
		struct dentry *parent, enum dump_type dump, unsigned int index,
		struct drm_device *drm)
{
	struct dentry *dent;
	struct tm_debug_override *tm = &hdr->tm;
	struct exynos_debug_info *info = &tm->info;
	struct hdr_tm_data_v2p2 *tm_data = &tm->force_data;

	dent = exynos_debugfs_add_hdr_lut_v2p2("tone_mapping", parent, info,
			tm->force_data.ts, DRM_SAMSUNG_HDR_TM_V2P2_LUT_LEN, ELEM_SIZE_32,
			tm->force_data.vs, DRM_SAMSUNG_HDR_TM_V2P2_LUT_LEN, ELEM_SIZE_32,
			DUMP_TYPE_HDR_TONEMAP, index, drm);

	debugfs_create_u16("coeff_00", 0664, dent, &tm_data->coeff_00);
	debugfs_create_u16("coeff_01", 0664, dent, &tm_data->coeff_01);
	debugfs_create_u16("coeff_02", 0664, dent, &tm_data->coeff_02);

	debugfs_create_u16("ymix_tf", 0664, dent, &tm_data->ymix_tf);
	debugfs_create_u16("ymix_vf", 0664, dent, &tm_data->ymix_vf);
	debugfs_create_u16("ymix_slope", 0664, dent, &tm_data->ymix_slope);
	debugfs_create_u16("ymix_dv", 0664, dent, &tm_data->ymix_dv);

	return dent;
}
#else
static struct dentry *exynos_debugfs_add_tm(struct exynos_hdr *hdr,
		struct dentry *parent, enum dump_type dump, unsigned int index,
		struct drm_device *drm)
{
	struct dentry *dent;
	struct tm_debug_override *tm = &hdr->tm;
	struct exynos_debug_info *info = &tm->info;
	struct hdr_tm_data *tm_data = &tm->force_data;

	dent = exynos_debugfs_add_hdr_lut("tone_mapping", parent, info,
			tm->force_data.posx, DRM_SAMSUNG_HDR_TM_LUT_LEN, ELEM_SIZE_16,
			tm->force_data.posy, DRM_SAMSUNG_HDR_TM_LUT_LEN, ELEM_SIZE_32,
			DUMP_TYPE_HDR_TONEMAP, index, drm);

	debugfs_create_u16("coeff_r", 0664, dent, &tm_data->coeff_r);
	debugfs_create_u16("coeff_g", 0664, dent, &tm_data->coeff_g);
	debugfs_create_u16("coeff_b", 0664, dent, &tm_data->coeff_b);

	debugfs_create_u16("range_x_min", 0664, dent, &tm_data->rng_x_min);
	debugfs_create_u16("range_x_max", 0664, dent, &tm_data->rng_x_max);
	debugfs_create_u16("range_y_min", 0664, dent, &tm_data->rng_y_min);
	debugfs_create_u16("range_y_max", 0664, dent, &tm_data->rng_y_max);

	return dent;
}
#endif

int exynos_drm_debugfs_plane_add(struct exynos_drm_plane *exynos_plane)
{
	struct drm_plane *plane = &exynos_plane->base;
	struct drm_minor *minor = plane->dev->primary;
	struct dpp_device *dpp = plane_to_dpp(exynos_plane);
	struct dentry *root, *ent, *hdr_dent;
	struct exynos_hdr *hdr = &dpp->hdr;
	unsigned int plane_index = drm_plane_index(plane);
	struct drm_device *drm = plane->dev;

	root = debugfs_create_dir(plane->name, minor->debugfs_root);
	if (!root)
		return -ENOMEM;

	exynos_plane->debugfs_entry = root;

	if (test_bit(DPP_ATTR_HDR, &dpp->attr)) {
		hdr_dent = debugfs_create_dir("hdr", root);
		if (!hdr_dent)
			goto err;

#if defined(CONFIG_SOC_ZUMA)
		debugfs_create_bool("fp16_en", 0664, hdr_dent, &hdr->fp16_en);
		debugfs_create_bool("fp16_cvt_en", 0664, hdr_dent, &hdr->fp16_cvt_en);
		ent = exynos_debugfs_add_hdr_lut_v2p2("eotf", hdr_dent,
				&hdr->eotf.info, hdr->eotf.force_lut.ts,
				DRM_SAMSUNG_HDR_EOTF_V2P2_LUT_LEN, ELEM_SIZE_32,
				hdr->eotf.force_lut.vs,
				DRM_SAMSUNG_HDR_EOTF_V2P2_LUT_LEN, ELEM_SIZE_32,
				DUMP_TYPE_HDR_EOTF, plane_index, drm);
		debugfs_create_u16("scaler", 0664, ent, &hdr->eotf.force_lut.scaler);
		debugfs_create_bool("lut_en", 0664, ent, &hdr->eotf.force_lut.lut_en);
#else
		ent = exynos_debugfs_add_hdr_lut("eotf", hdr_dent,
				&hdr->eotf.info, hdr->eotf.force_lut.posx,
				DRM_SAMSUNG_HDR_EOTF_LUT_LEN, ELEM_SIZE_16,
				hdr->eotf.force_lut.posy,
				DRM_SAMSUNG_HDR_EOTF_LUT_LEN, ELEM_SIZE_32,
				DUMP_TYPE_HDR_EOTF, plane_index, drm);
#endif
		if (!ent)
			goto err;

#if defined(CONFIG_SOC_ZUMA)
		ent = exynos_debugfs_add_hdr_lut_v2p2("oetf", hdr_dent,
				&hdr->oetf.info, hdr->oetf.force_lut.ts,
				DRM_SAMSUNG_HDR_OETF_V2P2_LUT_LEN, ELEM_SIZE_32,
				hdr->oetf.force_lut.vs,
				DRM_SAMSUNG_HDR_OETF_V2P2_LUT_LEN, ELEM_SIZE_32,
				DUMP_TYPE_HDR_OETF, plane_index, drm);
#else
		ent = exynos_debugfs_add_hdr_lut("oetf", hdr_dent,
				&hdr->oetf.info, hdr->oetf.force_lut.posx,
				DRM_SAMSUNG_HDR_OETF_LUT_LEN, ELEM_SIZE_16,
				hdr->oetf.force_lut.posy,
				DRM_SAMSUNG_HDR_OETF_LUT_LEN, ELEM_SIZE_16,
				DUMP_TYPE_HDR_OETF, plane_index, drm);
#endif
		if (!ent)
			goto err;

		ent = exynos_debugfs_add_gammut(hdr, hdr_dent,
				DUMP_TYPE_HDR_GAMMUT, plane_index, drm);
		if (!ent)
			goto err;
	}

	if (test_bit(DPP_ATTR_HDR10_PLUS, &dpp->attr)) {
#if defined(CONFIG_SOC_ZUMA)
		ent = exynos_debugfs_add_tm_v2p2(hdr, hdr_dent ? : root,
				DUMP_TYPE_HDR_TONEMAP, plane_index, drm);
#else
		ent = exynos_debugfs_add_tm(hdr, hdr_dent ? : root,
				DUMP_TYPE_HDR_TONEMAP, plane_index, drm);
#endif
		if (!ent)
			goto err;
	}

	return 0;
err:
	debugfs_remove_recursive(exynos_plane->debugfs_entry);
	exynos_plane->debugfs_entry = NULL;
	return -ENOMEM;
}


#define PREFIX_LEN	40
#define ROW_LEN		32
void dpu_print_hex_dump(struct drm_printer *p, void __iomem *regs, const void *buf, size_t len)
{
	char prefix_buf[PREFIX_LEN];
	unsigned char linebuf[96];
	unsigned long offset;
	int i, linelen;

	for (i = 0; i < len; i += ROW_LEN) {
		const u8 *ptr = buf + i;

		offset = buf - regs + i;

		if (len - i < ROW_LEN)
			linelen = len - i;
		else
			linelen = ROW_LEN;

		if (regs)
			snprintf(prefix_buf, sizeof(prefix_buf), "[%08lX] ", offset);
		else
			snprintf(prefix_buf, sizeof(prefix_buf), "[%08X] ", i);
		hex_dump_to_buffer(ptr, linelen, ROW_LEN, 4,
				linebuf, sizeof(linebuf), false);

		drm_printf(p, "%s%s\n", prefix_buf, linebuf);
	}
}

bool decon_dump_ignore(enum dpu_event_condition condition)
{
	return !(dpu_debug_dump_mask & condition);
}

void decon_dump_all(struct decon_device *decon,
		enum dpu_event_condition condition, bool async_buf_dump)
{
	bool active;
	struct kthread_worker *worker = &decon->worker;

	if (decon_dump_ignore(condition))
		return;

	if ((condition == DPU_EVT_CONDITION_DEFAULT) ||
		(condition == DPU_EVT_CONDITION_IDMA_ERROR)) {
		if (async_buf_dump)
			kthread_queue_work(worker, &decon->buf_dump_work);
		else
			buf_dump_all(decon);
	}

	active = pm_runtime_get_if_in_use(decon->dev) == 1;
	pr_info("%s: power %s state\n",
		dev_name(decon->dev), active ? "on" : "off");

	decon_dump_event_condition(decon, condition);

	if (active) {
		decon_dump(decon, NULL);
		pm_runtime_put(decon->dev);
	}
}

void decon_dump_event_condition(const struct decon_device *decon,
		enum dpu_event_condition condition)
{
	struct drm_printer p = is_console_enabled() ?
		drm_debug_printer("[drm]") : drm_info_printer(decon->dev);
	u32 print_log_size;

	if (decon_dump_ignore(condition))
		return;

	switch (condition) {
	case DPU_EVT_CONDITION_UNDERRUN:
	case DPU_EVT_CONDITION_FIFO_TIMEOUT:
	case DPU_EVT_CONDITION_IDMA_ERROR:
	case DPU_EVT_CONDITION_IDMA_ERROR_COMPACT:
		print_log_size = dpu_event_print_underrun;
		break;
	case DPU_EVT_CONDITION_FAIL_UPDATE_BW:
		print_log_size = dpu_event_print_fail_update_bw;
		break;
	case DPU_EVT_CONDITION_DEFAULT:
	default:
		print_log_size = dpu_event_print_max;
		break;
	}

	dpu_event_log_print(decon, &p, print_log_size, condition);
}

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)

#define MAX_DPU_ITMON_STR_NUM 2
static bool dpu_itmon_check(struct decon_device *decon, char *str_itmon, char *str_attr)
{
	const char *name[MAX_DPU_ITMON_STR_NUM];
	int count, i;

	if (!str_itmon)
		return false;

	count = of_property_count_strings(decon->dev->of_node, str_attr);
	if (count <= 0 || count > MAX_DPU_ITMON_STR_NUM) {
		pr_warn("%s: invalid number: %d\n", __func__, count);
		return false;
	}

	of_property_read_string_array(decon->dev->of_node,
				      str_attr, name, count);
	for (i = 0; i < count; i++) {
		if (strncmp(str_itmon, name[i], strlen(name[i])) == 0)
			return true;
	}

	return false;
}

int dpu_itmon_notifier(struct notifier_block *nb, unsigned long act, void *data)
{
	struct decon_device *decon;
	struct itmon_notifier *itmon_data = data;

	decon = container_of(nb, struct decon_device, itmon_nb);

	pr_debug("%s: DECON%u +\n", __func__, decon->id);

	if (decon->itmon_notified)
		return NOTIFY_DONE;

	if (IS_ERR_OR_NULL(itmon_data))
		return NOTIFY_DONE;

	/* port is master and dest is target */
	if (dpu_itmon_check(decon, itmon_data->port, "itmon,port") ||
	    dpu_itmon_check(decon, itmon_data->dest, "itmon,dest")) {
		DPU_EVENT_LOG(DPU_EVT_ITMON_ERROR, decon->id, NULL);
		pr_info("%s: port: %s, dest: %s\n", __func__,
				itmon_data->port, itmon_data->dest);

		decon_dump_all(decon, DPU_EVT_CONDITION_DEFAULT, true);

		decon->itmon_notified = true;
		return NOTIFY_OK;
	}

	pr_debug("%s -\n", __func__);

	return NOTIFY_DONE;
}

#endif

#ifdef CONFIG_DEBUG_FS
static int dphy_diag_text_show(struct seq_file *m, void *p)
{
	char *text = m->private;

	seq_printf(m, "%s\n", text);
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(dphy_diag_text);

static ssize_t dphy_diag_reg_write(struct file *file, const char *user_buf,
			      size_t count, loff_t *f_pos)
{
	int ret;
	uint32_t val;
	struct seq_file *m = file->private_data;
	struct dsim_dphy_diag *diag = m->private;
	struct dsim_device *dsim = diag->private;

	ret = kstrtou32_from_user(user_buf, count, 0, &val);
	if (ret)
		return ret;

	/* config dphy of another dsim_device for dual dsi */
	if (dsim->dual_dsi == DSIM_DUAL_DSI_MAIN) {
		int i;
		struct dsim_device *sec_dsi = NULL;
		struct dsim_dphy_diag *sec_diag = NULL;

		sec_dsi = exynos_get_dual_dsi(DSIM_DUAL_DSI_SEC);
		if (sec_dsi) {
			/* found the corresponding diag of second dsim device */
			for (i = 0; i < sec_dsi->config.num_dphy_diags; ++i) {
				if (!strcmp(sec_dsi->config.dphy_diags[i].name, diag->name)) {
					sec_diag = &sec_dsi->config.dphy_diags[i];
					break;
				}
			}

			ret = dsim_dphy_diag_set_reg(sec_dsi, sec_diag, val);
			if (ret)
				return ret;
		} else {
			return -ENODEV;
		}
	}
	ret = dsim_dphy_diag_set_reg(diag->private, diag, val);
	if (ret)
		return ret;

	return count;
}

static int dphy_diag_reg_show(struct seq_file *m, void *data)
{
	struct dsim_dphy_diag *diag = m->private;
	uint32_t regs[MAX_DIAG_REG_NUM];
	uint32_t ix;
	int ret;

	ret = dsim_dphy_diag_get_reg(diag->private, diag, regs);

	if (ret == 0) {
		for (ix = 0; ix < diag->num_reg; ++ix)
			seq_printf(m, "%d ", regs[ix]);
		seq_puts(m, "\n");
	}

	return ret;
}

static int dphy_diag_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, dphy_diag_reg_show, inode->i_private);
}

static const struct file_operations dphy_diag_reg_fops = {
	.owner = THIS_MODULE,
	.open = dphy_diag_reg_open,
	.write = dphy_diag_reg_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void dsim_diag_create_debugfs(struct dsim_device *dsim) {
	struct dentry *dent_dphy;
	struct dentry *dent_diag;
	struct dsim_dphy_diag *diag;
	char dir_name[32];
	int ix;

	scnprintf(dir_name, sizeof(dir_name), "dsim%d", dsim->id);
	dsim->debugfs_entry = debugfs_create_dir(
		dir_name, dsim->encoder.dev->primary->debugfs_root);
	if (!dsim->debugfs_entry) {
		pr_warn("%s: failed to create %s\n", __func__, dir_name);
		return;
	}

	debugfs_create_u32("state", 0400, dsim->debugfs_entry, &dsim->state);
	debugfs_create_bool("force_set_hs_clk", 0600, dsim->debugfs_entry, &dsim->force_set_hs_clk);

	if (dsim->config.num_dphy_diags == 0)
		return;

	dent_dphy = debugfs_create_dir("dphy", dsim->debugfs_entry);
	if (!dent_dphy) {
		pr_warn("%s: failed to create %s\n", __func__, dir_name);
		return;
	}

	for (ix = 0; ix < dsim->config.num_dphy_diags; ++ix) {
		diag = &dsim->config.dphy_diags[ix];
		dent_diag = debugfs_create_dir(diag->name, dent_dphy);
		if (!dent_diag) {
			pr_warn("%s: failed to create %s\n", __func__,
				diag->name);
			continue;
		}
		debugfs_create_file("desc", 0400, dent_diag, (void *)diag->desc,
				    &dphy_diag_text_fops);
		debugfs_create_file("help", 0400, dent_diag, (void *)diag->help,
				    &dphy_diag_text_fops);
		diag->private = dsim;
		debugfs_create_file("value", diag->read_only ? 0400 : 0600,
				    dent_diag, diag, &dphy_diag_reg_fops);
	}
}

void dsim_diag_remove_debugfs(struct dsim_device *dsim) {
	debugfs_remove_recursive(dsim->debugfs_entry);
	dsim->debugfs_entry = NULL;
}
#endif
