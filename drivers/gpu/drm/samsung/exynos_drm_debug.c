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
#include <video/mipi_display.h>
#include <drm/drmP.h>
#include "exynos_drm_decon.h"
#include "exynos_drm_dsim.h"

/* If event are happened continuously, then ignore */
static bool dpu_event_ignore
	(enum dpu_event_type type, struct decon_device *decon)
{
	int latest = atomic_read(&decon->d.event_log_idx) % DPU_EVENT_LOG_MAX;
	int idx, offset;

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return true;

	for (offset = 0; offset < DPU_EVENT_KEEP_CNT; ++offset) {
		idx = (latest + DPU_EVENT_LOG_MAX - offset) % DPU_EVENT_LOG_MAX;
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
	int idx;

	if (index < 0) {
		DRM_ERROR("%s: decon id is not valid(%d)\n", __func__, index);
		return;
	}

	decon = get_decon_drvdata(index);
	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	switch (type) {
	case DPU_EVT_TE_INTERRUPT:
	case DPU_EVT_DSIM_UNDERRUN:
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

	idx = atomic_inc_return(&decon->d.event_log_idx) % DPU_EVENT_LOG_MAX;
	log = &decon->d.event_log[idx];

	log->time = ktime_get();
	log->type = type;

	switch (type) {
	case DPU_EVT_DPP_FRAMEDONE:
		dpp = (struct dpp_device *)priv;
		log->data.dpp.id = dpp->id;
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
	int idx, i, dpp_ch;

	if (index < 0) {
		DRM_ERROR("%s: decon id is not valid(%d)\n", __func__, index);
		return;
	}

	decon = get_decon_drvdata(index);

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	idx = atomic_inc_return(&decon->d.event_log_idx) % DPU_EVENT_LOG_MAX;
	log = &decon->d.event_log[idx];

	log->type = DPU_EVT_ATOMIC_COMMIT;
	log->time = ktime_get();

	for (i = 0; i < MAX_WIN_PER_DECON; ++i) {
		memcpy(&log->data.atomic.win_config[i].win,
				&decon->bts.win_config[i],
				sizeof(struct dpu_bts_win_config));

		if (decon->bts.win_config[i].state == DPU_WIN_STATE_BUFFER) {
			dpp_ch = decon->bts.win_config[i].dpp_ch;

			log->data.atomic.win_config[i].dma_addr =
				decon->dpp[dpp_ch]->fb.exynos_buf[0]->dma_addr;
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
	int idx, i;

	if (index < 0) {
		DRM_ERROR("%s: decon id is not valid(%d)\n", __func__, index);
		return;
	}

	decon = get_decon_drvdata(index);

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	idx = atomic_inc_return(&decon->d.event_log_idx) % DPU_EVENT_LOG_MAX;
	log = &decon->d.event_log[idx];

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

void dpu_print_log_atomic(struct seq_file *s, struct dpu_log_atomic *atomic)
{
	int i;
	struct dpu_bts_win_config *win;
	char *str_state[3] = {"DISABLED", "COLOR", "BUFFER"};

	for (i = 0; i < MAX_WIN_PER_DECON; ++i) {
		win = &atomic->win_config[i].win;

		if (win->state == DPU_WIN_STATE_DISABLED)
			continue;

		seq_printf(s, "\t\t\t\t\tWIN%d: %s[0x%llx] SRC[%d %d %d %d] ",
				i, str_state[win->state],
				(win->state == DPU_WIN_STATE_BUFFER) ?
				atomic->win_config[i].dma_addr : 0,
				win->src_x, win->src_y, win->src_w, win->src_h);
		seq_printf(s, "DST[%d %d %d %d] DPP%d\n",
				win->dst_x, win->dst_y, win->dst_w, win->dst_h,
				win->dpp_ch);
	}
}

void DPU_EVENT_SHOW(struct seq_file *s, struct decon_device *decon)
{
	int idx = atomic_read(&decon->d.event_log_idx) % DPU_EVENT_LOG_MAX;
	struct dpu_log *log;
	int latest = idx;
	struct timeval tv;
	ktime_t prev_ktime;

	if (IS_ERR_OR_NULL(decon->d.event_log))
		return;

	seq_puts(s, "-------------------------------------------------------------\n");
	seq_printf(s, "%14s  %20s  %20s\n", "Time", "Event ID", "Remarks");
	seq_puts(s, "-------------------------------------------------------------\n");

	/* Seek a oldest from current index */
	idx = (idx + DPU_EVENT_LOG_MAX - DPU_EVENT_PRINT_MAX) %
						DPU_EVENT_LOG_MAX;
	prev_ktime = ktime_set(0, 0);
	do {
		if (++idx >= DPU_EVENT_LOG_MAX)
			idx = 0;

		/* Seek a index */
		log = &decon->d.event_log[idx];

		/* TIME */
		tv = ktime_to_timeval(log->time);
		seq_printf(s, "[%6ld.%06ld] ", tv.tv_sec, tv.tv_usec);

		/* If there is no timestamp, then exit directly */
		if (!tv.tv_sec)
			break;

		/* EVETN ID + Information */
		switch (log->type) {
		case DPU_EVT_DECON_ENABLED:
			seq_printf(s, "%20s  %20s", "DECON_ENABLED", "-\n");
			break;
		case DPU_EVT_DECON_DISABLED:
			seq_printf(s, "%20s  %20s", "DECON_DISABLED", "-\n");
			break;
		case DPU_EVT_DECON_FRAMEDONE:
			seq_printf(s, "%20s  %20s", "DECON_FRAMEDONE", "-\n");
			break;
		case DPU_EVT_DECON_FRAMESTART:
			seq_printf(s, "%20s  %20s", "DECON_FRAMESTART", "-\n");
			break;
		case DPU_EVT_DECON_TRIG_UNMASK:
			seq_printf(s, "%20s  %20s", "TRIG_UNMASK", "-\n");
			break;
		case DPU_EVT_DSIM_ENABLED:
			seq_printf(s, "%20s  %20s", "DSIM_ENABLED", "-\n");
			break;
		case DPU_EVT_DSIM_DISABLED:
			seq_printf(s, "%20s  %20s", "DSIM_DISABLED", "-\n");
			break;
		case DPU_EVT_DSIM_COMMAND:
			seq_printf(s, "%20s  ", "DSIM_COMMAND");
			seq_printf(s, "\tCMD_ID: 0x%x\tDATA[0]: 0x%x\n",
					log->data.cmd.id, log->data.cmd.buf);
			break;
		case DPU_EVT_TE_INTERRUPT:
			seq_printf(s, "%20s  %20s", "TE", "-\n");
			break;
		case DPU_EVT_DPP_FRAMEDONE:
			seq_printf(s, "%20s  ", "DPP_FRAMEDONE");
			seq_printf(s, "\tID:%d\n", log->data.dpp.id);
			break;
		case DPU_EVT_ATOMIC_COMMIT:
			seq_printf(s, "%20s  %20s", "ATOMIC_COMMIT", "-\n");
			dpu_print_log_atomic(s, &log->data.atomic);
			break;
		default:
			seq_printf(s, "%20s  (%2d)\n", "NO_DEFINED", log->type);
			break;
		}
	} while (latest != idx);

	seq_puts(s, "-------------------------------------------------------------\n");
}

static int dpu_debug_event_show(struct seq_file *s, void *unused)
{
	struct decon_device *decon = s->private;

	DPU_EVENT_SHOW(s, decon);
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
	char name[MAX_NAME_SIZE];
	int ret = 0;
	int i;
	u32 event_cnt;

	decon->d.event_log = NULL;
	event_cnt = DPU_EVENT_LOG_MAX;

	for (i = 0; i < DPU_EVENT_LOG_RETRY; ++i) {
		event_cnt = event_cnt >> i;
		decon->d.event_log = kcalloc(event_cnt, sizeof(struct dpu_log),
				GFP_KERNEL);
		if (IS_ERR_OR_NULL(decon->d.event_log)) {
			DRM_WARN("failed to alloc event log buf[%d]. retry\n",
					event_cnt);
			continue;
		}

		DRM_INFO("#%d event log buffers are allocated\n", event_cnt);
		break;
	}
	decon->d.event_log_cnt = event_cnt;

	if (!decon->id) {
		decon->d.debug_root = debugfs_create_dir("decon", NULL);
		if (!decon->d.debug_root) {
			DRM_ERROR("failed to create debugfs root directory.\n");
			ret = -ENOENT;
			goto err_event_log;
		}
	}

	if (decon->id == 1 || decon->id == 2)
		decon->d.debug_root = decon_drvdata[0]->d.debug_root;

	snprintf(name, MAX_NAME_SIZE, "event%d", decon->id);
	atomic_set(&decon->d.event_log_idx, -1);
	decon->d.debug_event = debugfs_create_file(name, 0444,
			decon->d.debug_root, decon, &dpu_event_fops);
	if (!decon->d.debug_event) {
		DRM_ERROR("failed to create debugfs file\n");
		ret = -ENOENT;
		goto err_debugfs;
	}

	return 0;

err_debugfs:
	debugfs_remove_recursive(decon->d.debug_root);
err_event_log:
	kfree(decon->d.event_log);
	decon->d.event_log = NULL;
	return ret;
}

void dpu_deinit_debug(struct decon_device *decon)
{
	debugfs_remove(decon->d.debug_root);
	debugfs_remove(decon->d.debug_event);
}
