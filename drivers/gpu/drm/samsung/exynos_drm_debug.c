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

#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/pm_runtime.h>
#include <linux/moduleparam.h>
#include <video/mipi_display.h>
#include <drm/drm_print.h>

#include <exynos_drm_decon.h>
#include <exynos_drm_dsim.h>
#include <exynos_drm_writeback.h>
#include <cal_config.h>

/* Default is 1024 entries array for event log buffer */
static unsigned int dpu_event_log_max = 1024;
static unsigned int dpu_event_print_max = 512;

module_param_named(event_log_max, dpu_event_log_max, uint, 0);
module_param_named(event_print_max, dpu_event_print_max, uint, 0600);
MODULE_PARM_DESC(event_log_max, "entry count of event log buffer array");
MODULE_PARM_DESC(event_print_max, "print entry count of event log buffer");

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
	struct dpu_log *log;
	struct drm_crtc_state *crtc_state;
	unsigned long flags;
	int idx;

	if (index < 0) {
		DRM_ERROR("%s: decon id is not valid(%d)\n", __func__, index);
		return;
	}

	decon = get_decon_drvdata(index);
	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	switch (type) {
	case DPU_EVT_DSIM_UNDERRUN:
		decon->d.underrun_cnt++;
	case DPU_EVT_TE_INTERRUPT:
		/*
		 * If the same event occurs DPU_EVENT_KEEP_CNT times
		 * continuously, it will be skipped.
		 */
		if (dpu_event_ignore(type, decon))
			return;
		break;
	default:
		break;
	}

	spin_lock_irqsave(&decon->d.event_lock, flags);
	idx = atomic_inc_return(&decon->d.event_log_idx) % dpu_event_log_max;
	log = &decon->d.event_log[idx];
	spin_unlock_irqrestore(&decon->d.event_lock, flags);

	log->time = ktime_get();
	log->type = type;

	switch (type) {
	case DPU_EVT_DPP_FRAMEDONE:
		dpp = (struct dpp_device *)priv;
		log->data.dpp.id = dpp->id;
		break;
	case DPU_EVT_DMA_RECOVERY:
		dpp = (struct dpp_device *)priv;
		log->data.dpp.id = dpp->id;
		log->data.dpp.comp_src = dpp->comp_src;
		log->data.dpp.recovery_cnt = dpp->recovery_cnt;
		break;
	case DPU_EVT_DECON_RSC_OCCUPANCY:
		pm_runtime_get_sync(decon->dev);
		log->data.rsc.rsc_ch = decon_reg_get_rsc_ch(decon->id);
		log->data.rsc.rsc_win = decon_reg_get_rsc_win(decon->id);
		pm_runtime_put_sync(decon->dev);
		break;
	case DPU_EVT_ENTER_HIBERNATION_IN:
	case DPU_EVT_ENTER_HIBERNATION_OUT:
	case DPU_EVT_EXIT_HIBERNATION_IN:
	case DPU_EVT_EXIT_HIBERNATION_OUT:
		log->data.pd.rpm_active = pm_runtime_active(decon->dev);
		break;
	case DPU_EVT_PLANE_UPDATE:
	case DPU_EVT_PLANE_DISABLE:
		dpp = (struct dpp_device *)priv;
		log->data.win.win_idx = dpp->win_id;
		log->data.win.plane_idx = dpp->id;
		break;
	case DPU_EVT_REQ_CRTC_INFO_OLD:
	case DPU_EVT_REQ_CRTC_INFO_NEW:
		crtc_state = (struct drm_crtc_state *)priv;
		log->data.crtc_info.enable = crtc_state->enable;
		log->data.crtc_info.active = crtc_state->active;
		log->data.crtc_info.planes_changed = crtc_state->planes_changed;
		log->data.crtc_info.mode_changed = crtc_state->mode_changed;
		log->data.crtc_info.active_changed = crtc_state->active_changed;
		break;
	default:
		break;
	}
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
	unsigned long flags;
	int idx, i, dpp_ch;

	if (index < 0) {
		DRM_ERROR("%s: decon id is not valid(%d)\n", __func__, index);
		return;
	}

	decon = get_decon_drvdata(index);

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	spin_lock_irqsave(&decon->d.event_lock, flags);
	idx = atomic_inc_return(&decon->d.event_log_idx) % dpu_event_log_max;
	log = &decon->d.event_log[idx];
	spin_unlock_irqrestore(&decon->d.event_lock, flags);

	log->type = DPU_EVT_ATOMIC_COMMIT;
	log->time = ktime_get();

	for (i = 0; i < MAX_WIN_PER_DECON; ++i) {
		memcpy(&log->data.atomic.win_config[i].win,
				&decon->bts.win_config[i],
				sizeof(struct dpu_bts_win_config));

		if (decon->bts.win_config[i].state == DPU_WIN_STATE_BUFFER) {
			dpp_ch = decon->bts.win_config[i].dpp_ch;

			log->data.atomic.win_config[i].dma_addr =
				decon->dpp[dpp_ch]->dbg_dma_addr;
		}
	}
}

extern void *return_address(int);

/*
 * DPU_EVENT_LOG_CMD() - store DSIM command information
 * @index: event log index
 * @dsim: pointer to dsim device structure
 * @cmd_id : DSIM command id
 * @data: command buffer data
 *
 * Stores command id and first data in command buffer and return addresses
 * in callstack which lets you know who called this function.
 */
void DPU_EVENT_LOG_CMD(int index, struct dsim_device *dsim, u32 cmd_id,
		unsigned long data)
{
	struct decon_device *decon;
	struct dpu_log *log;
	unsigned long flags;
	int idx, i;

	if (index < 0) {
		DRM_ERROR("%s: decon id is not valid(%d)\n", __func__, index);
		return;
	}

	decon = get_decon_drvdata(index);

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	spin_lock_irqsave(&decon->d.event_lock, flags);
	idx = atomic_inc_return(&decon->d.event_log_idx) % dpu_event_log_max;
	log = &decon->d.event_log[idx];
	spin_unlock_irqrestore(&decon->d.event_lock, flags);

	log->type = DPU_EVT_DSIM_COMMAND;
	log->time = ktime_get();

	log->data.cmd.id = cmd_id;
	if (cmd_id == MIPI_DSI_DCS_LONG_WRITE)
		log->data.cmd.buf = *(u8 *)(data);
	else
		log->data.cmd.buf = (u8)data;

	for (i = 0; i < DPU_CALLSTACK_MAX; i++)
		log->data.cmd.caller[i] =
			(void *)((size_t)return_address(i + 1));
}

static void dpu_print_log_atomic(struct dpu_log_atomic *atomic,
						struct drm_printer *p)
{
	int i;
	struct dpu_bts_win_config *win;
	char *str_state[3] = {"DISABLED", "COLOR", "BUFFER"};
	const char *str_comp;
	const struct dpu_fmt *fmt;
	char buf[128];
	int len;

	for (i = 0; i < MAX_WIN_PER_DECON; ++i) {
		win = &atomic->win_config[i].win;

		if (win->state == DPU_WIN_STATE_DISABLED)
			continue;

		fmt = dpu_find_fmt_info(win->format);

		len = scnprintf(buf, sizeof(buf),
				"\t\t\t\t\tWIN%d: %s[0x%llx] SRC[%d %d %d %d] ",
				i, str_state[win->state],
				(win->state == DPU_WIN_STATE_BUFFER) ?
				atomic->win_config[i].dma_addr : 0,
				win->src_x, win->src_y, win->src_w, win->src_h);
		len += scnprintf(buf + len, sizeof(buf) - len,
				"DST[%d %d %d %d] ",
				win->dst_x, win->dst_y, win->dst_w, win->dst_h);
		if (win->state == DPU_WIN_STATE_BUFFER)
			len += scnprintf(buf + len, sizeof(buf) - len, "CH%d ",
					win->dpp_ch);

		str_comp = get_comp_src_name(win->comp_src);
		drm_printf(p, "%s %s %s\n", buf, fmt->name, str_comp);
	}
}

void dpu_print_log_rsc(char *buf, int len, struct dpu_log_rsc_occupancy *rsc)
{
	int i, len_chs, len_wins;
	char str_chs[128];
	char str_wins[128];
	bool using_ch, using_win;

	len_chs = sprintf(str_chs, "CHs: ");
	len_wins = sprintf(str_wins, "WINs: ");

	for (i = 0; i < MAX_PLANE; ++i) {
		using_ch = is_decon_using_ch(0, rsc->rsc_ch, i);
		len_chs += sprintf(str_chs + len_chs, "%d[%c] ", i,
				using_ch ? 'O' : 'X');

		using_win = is_decon_using_win(0, rsc->rsc_win, i);
		len_wins += sprintf(str_wins + len_wins, "%d[%c] ", i,
				using_win ? 'O' : 'X');
	}

	sprintf(buf + len, "\t%s\t%s", str_chs, str_wins);
}

const char *get_event_name(enum dpu_event_type type)
{
	static const char events[][32] = {
		"NONE",				"DECON_ENABLED",
		"DECON_DISABLED",		"DECON_FRAMEDONE",
		"DECON_FRAMESTART",		"DECON_RSC_OCCUPANCY",
		"DECON_TRIG_MASK",		"DSIM_ENABLED",
		"DSIM_DISABLED",		"DSIM_COMMAND",
		"DSIM_UNDERRUN",		"DSIM_FRAMEDONE",
		"DPP_FRAMEDONE", 		"DMA_RECOVERY",
		"ATOMIC_COMMIT",		"TE_INTERRUPT",
		"ENTER_HIBERNATION_IN",		"ENTER_HIBERNATION_OUT",
		"EXIT_HIBERNATION_IN",		"EXIT_HIBERNATION_OUT",
		"ATOMIC_BEGIN",			"ATOMIC_FLUSH",
		"WB_ENABLE",			"WB_DISABLE",
		"WB_ATOMIC_COMMIT",		"WB_FRAMEDONE",
		"WB_ENTER_HIBERNATION",		"WB_EXIT_HIBERNATION",
		"PLANE_UPDATE",			"PLANE_DISABLE",
		"REQ_CRTC_INFO_OLD",		"REQ_CRTC_INFO_NEW",
		"FRAMESTART_TIMEOUT",
	};

	if (type >= DPU_EVT_MAX)
		return NULL;

	return events[type];
}

static void DPU_EVENT_SHOW(struct decon_device *decon, struct drm_printer *p)
{
	int idx = atomic_read(&decon->d.event_log_idx) % dpu_event_log_max;
	struct dpu_log *log;
	int latest = idx;
	struct timespec64 ts;
	ktime_t prev_ktime;
	const char *str_comp;
	char buf[128];
	int len;

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	drm_printf(p, "----------------------------------------------------\n");
	drm_printf(p, "%14s  %20s  %20s\n", "Time", "Event ID", "Remarks");
	drm_printf(p, "----------------------------------------------------\n");

	/* Seek a oldest from current index */
	if (dpu_event_print_max > dpu_event_log_max)
		dpu_event_print_max = dpu_event_log_max;
	idx = (idx + dpu_event_log_max - dpu_event_print_max) %
						dpu_event_log_max;
	prev_ktime = ktime_set(0, 0);
	do {
		if (++idx >= dpu_event_log_max)
			idx = 0;

		/* Seek a index */
		log = &decon->d.event_log[idx];

		/* TIME */
		ts = ktime_to_timespec64(log->time);
		len = scnprintf(buf, sizeof(buf), "[%6ld.%09ld] ", ts.tv_sec,
				ts.tv_nsec);

		/* If there is no timestamp, then exit directly */
		if (!ts.tv_sec)
			break;

		len += scnprintf(buf + len, sizeof(buf) - len,  "%20s",
				get_event_name(log->type));

		switch (log->type) {
		case DPU_EVT_DECON_RSC_OCCUPANCY:
			dpu_print_log_rsc(buf, len, &log->data.rsc);
			break;
		case DPU_EVT_DSIM_COMMAND:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tCMD_ID: 0x%x\tDATA[0]: 0x%x",
					log->data.cmd.id, log->data.cmd.buf);
			break;
		case DPU_EVT_DPP_FRAMEDONE:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tID:%d", log->data.dpp.id);
			break;
		case DPU_EVT_DMA_RECOVERY:
			str_comp = get_comp_src_name(log->data.dpp.comp_src);
			scnprintf(buf + len, sizeof(buf) - len,
					"\tID:%d SRC:%s COUNT:%d",
					log->data.dpp.id, str_comp,
					log->data.dpp.recovery_cnt);
			break;
		case DPU_EVT_ENTER_HIBERNATION_IN:
		case DPU_EVT_ENTER_HIBERNATION_OUT:
		case DPU_EVT_EXIT_HIBERNATION_IN:
		case DPU_EVT_EXIT_HIBERNATION_OUT:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tDPU POWER %s",
					log->data.pd.rpm_active ? "ON" : "OFF");
			break;
		case DPU_EVT_PLANE_UPDATE:
		case DPU_EVT_PLANE_DISABLE:
			scnprintf(buf + len, sizeof(buf) - len,
					"\tCH:%d, WIN:%d",
					log->data.win.plane_idx,
					log->data.win.win_idx);
			break;
		case DPU_EVT_REQ_CRTC_INFO_OLD:
		case DPU_EVT_REQ_CRTC_INFO_NEW:
			scnprintf(buf + len, sizeof(buf) - len,
				"\tenable(%d) active(%d) [p:%d m:%d a:%d]",
					log->data.crtc_info.enable,
					log->data.crtc_info.active,
					log->data.crtc_info.planes_changed,
					log->data.crtc_info.mode_changed,
					log->data.crtc_info.active_changed);
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

	DPU_EVENT_SHOW(decon, &p);
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

#define MAX_NAME_SIZE	32
int dpu_init_debug(struct decon_device *decon)
{
	int i;
	int ret = 0;
	u32 event_cnt;
	struct drm_crtc *crtc;
	struct dentry *debug_event;
	struct dentry *urgent_dent;

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

	if (!decon->crtc) {
		ret = -ENOENT;
		goto err_event_log;
	}

	crtc = &decon->crtc->base;

	debug_event = debugfs_create_file("event", 0444, crtc->debugfs_entry,
			decon, &dpu_event_fops);
	if (!debug_event) {
		DRM_ERROR("failed to create debugfs event file\n");
		goto err_event_log;
	}

	debugfs_create_u32("underrun_cnt", 0664, crtc->debugfs_entry,
			&decon->d.underrun_cnt);

	urgent_dent = debugfs_create_dir("urgent", crtc->debugfs_entry);
	if (!urgent_dent) {
		DRM_ERROR("failed to create debugfs urgent directory\n");
		goto err_debugfs;
	}

	debugfs_create_u32("rd_en", 0664, urgent_dent, &decon->config.urgent.rd_en);

	debugfs_create_x32("rd_hi_thres", 0664, urgent_dent,
			&decon->config.urgent.rd_hi_thres);

	debugfs_create_x32("rd_lo_thres", 0664, urgent_dent,
			&decon->config.urgent.rd_lo_thres);

	debugfs_create_x32("rd_wait_cycle", 0664, urgent_dent,
			&decon->config.urgent.rd_wait_cycle);

	debugfs_create_u32("wr_en", 0664, urgent_dent, &decon->config.urgent.wr_en);

	debugfs_create_x32("wr_hi_thres", 0664, urgent_dent,
			&decon->config.urgent.wr_hi_thres);

	debugfs_create_x32("wr_lo_thres", 0664,
			urgent_dent, &decon->config.urgent.wr_lo_thres);

	debugfs_create_bool("dta_en", 0664,
			urgent_dent, &decon->config.urgent.dta_en);

	debugfs_create_x32("dta_hi_thres", 0664,
			urgent_dent, &decon->config.urgent.dta_hi_thres);

	debugfs_create_x32("dta_lo_thres", 0664,
			urgent_dent, &decon->config.urgent.dta_lo_thres);

	return 0;

err_debugfs:
	debugfs_remove(debug_event);
err_event_log:
	vfree(decon->d.event_log);
	return -ENOENT;
}

#define PREFIX_LEN	40
#define ROW_LEN		32
void dpu_print_hex_dump(void __iomem *regs, const void *buf, size_t len)
{
	char prefix_buf[PREFIX_LEN];
	unsigned long p;
	int i, row;

	for (i = 0; i < len; i += ROW_LEN) {
		p = buf - regs + i;

		if (len - i < ROW_LEN)
			row = len - i;
		else
			row = ROW_LEN;

		snprintf(prefix_buf, sizeof(prefix_buf), "[%08lX] ", p);
		print_hex_dump(KERN_INFO, prefix_buf, DUMP_PREFIX_NONE,
				32, 4, buf + i, row, false);
	}
}

#if defined(CONFIG_EXYNOS_ITMON)
int dpu_itmon_notifier(struct notifier_block *nb, unsigned long act, void *data)
{
	struct decon_device *decon;
	struct itmon_notifier *itmon_data = data;
	struct drm_printer p;
	bool active;

	decon = container_of(nb, struct decon_device, itmon_nb);
	p = drm_info_printer(decon->dev);

	pr_debug("%s: DECON%d +\n", __func__, decon->id);

	if (decon->itmon_notified)
		return NOTIFY_DONE;

	if (IS_ERR_OR_NULL(itmon_data))
		return NOTIFY_DONE;

	/* port is master and dest is target */
	if ((itmon_data->port &&
		(strncmp("DISP", itmon_data->port, sizeof("DISP") - 1) == 0)) ||
		(itmon_data->dest &&
		(strncmp("DISP", itmon_data->dest, sizeof("DISP") - 1) == 0))) {
		pr_info("%s: port: %s, dest: %s\n", __func__,
				itmon_data->port, itmon_data->dest);

		active = pm_runtime_active(decon->dev);
		pr_info("DPU power %s state\n", active ? "on" : "off");

		DPU_EVENT_SHOW(decon, &p);

		if (active)
			decon_dump(decon);

		decon->itmon_notified = true;
		return NOTIFY_OK;
	}

	pr_debug("%s -\n", __func__);

	return NOTIFY_DONE;
}
#endif
