/* SPDX-License-Identifier: GPL-2.0-only
 *
 * linux/drivers/gpu/drm/samsung/exynos_drm_decon.h
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Headef file for Samsung MIPI DSI Master driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_DECON_H__
#define __EXYNOS_DRM_DECON_H__

#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/pm_qos.h>

#include <soc/samsung/bts.h>
#include <drm/drm_device.h>
#include <video/videomode.h>

#include "exynos_drm_dpp.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"

#include "cal_9820/decon_cal.h"

enum decon_state {
	DECON_STATE_INIT = 0,
	DECON_STATE_ON,
	DECON_STATE_DOZE,
	DECON_STATE_HIBER,
	DECON_STATE_DOZE_SUSPEND,
	DECON_STATE_OFF,
	DECON_STATE_TUI,
};

enum dpu_win_state {
	DPU_WIN_STATE_DISABLED = 0,
	DPU_WIN_STATE_COLOR,
	DPU_WIN_STATE_BUFFER,
};

struct decon_win {
	u32				idx;
	struct exynos_drm_plane		plane;
	struct exynos_drm_plane_config	plane_config;
	struct exynos_drm_fb		fb;
};

struct decon_resources {
	struct pinctrl *pinctrl;
	struct pinctrl_state *te_on;
	struct pinctrl_state *te_off;
	struct clk *aclk;
};

struct dpu_bts_ops {
	void (*bts_init)(struct decon_device *decon);
	void (*bts_acquire_bw)(struct decon_device *decon);
	void (*bts_release_bw)(struct decon_device *decon);
	void (*bts_calc_bw)(struct decon_device *decon);
	void (*bts_update_bw)(struct decon_device *decon, bool shadow_updated);
	void (*bts_deinit)(struct decon_device *decon);
};

struct dpu_bts_bw {
	u32 val;
	u32 ch_num;
};

struct dpu_bts_win_config {
	enum dpu_win_state state;
	u32 src_x;
	u32 src_y;
	u32 src_w;
	u32 src_h;
	int dst_x;
	int dst_y;
	u32 dst_w;
	u32 dst_h;
	bool is_rot;
	bool is_afbc;
	int dpp_ch;
	enum dpu_pixel_format format;
};

struct dpu_bts {
	bool enabled;
	u32 resol_clk;
	u32 peak;
	u32 total_bw;
	u32 prev_total_bw;
	u32 max_disp_freq;
	u32 prev_max_disp_freq;
	u64 ppc;
	u32 line_mem_cnt;
	u32 cycle_per_line;
	u32 vbp;
	u32 vfp;
	u32 vsa;
	u32 fps;
#if defined(CONFIG_EXYNOS_BTS)
	struct dpu_bts_bw bw[BTS_DPP_MAX];

	/* each decon must know other decon's BW to get overall BW */
	u32 ch_bw[3][BTS_DPU_MAX];
	enum bts_bw_type type;
	struct bts_decon_info bts_info;
#endif
	struct dpu_bts_ops *ops;
	struct pm_qos_request mif_qos;
	struct pm_qos_request int_qos;
	struct pm_qos_request disp_qos;
	u32 scen_updated;

	struct dpu_bts_win_config win_config[MAX_WIN_PER_DECON];
};

/**
 * Display Subsystem event management status.
 *
 * These status labels are used internally by the DECON to indicate the
 * current status of a device with operations.
 */
enum dpu_event_type {
	DPU_EVT_NONE = 0,

	DPU_EVT_DECON_ENABLED,
	DPU_EVT_DECON_DISABLED,
	DPU_EVT_DECON_FRAMEDONE,

	DPU_EVT_DSIM_ENABLED,
	DPU_EVT_DSIM_DISABLED,

	DPU_EVT_DPP_FRAMEDONE,

	DPU_EVT_ATOMIC_COMMIT,
	DPU_EVT_TE_INTERRUPT,

	DPU_EVT_MAX, /* End of EVENT */
};

struct dpu_log_dpp {
	u32 id;
};

struct decon_win_config {
	struct dpu_bts_win_config win;
	dma_addr_t dma_addr;
};

struct dpu_log_atomic {
	struct decon_win_config win_config[MAX_WIN_PER_DECON];
};

struct dpu_log {
	ktime_t time;
	enum dpu_event_type type;

	union {
		struct dpu_log_dpp dpp;
		struct dpu_log_atomic atomic;
	} data;
};

/* Definitions below are used in the DECON */
#define DPU_EVENT_LOG_MAX	SZ_512
#define DPU_EVENT_PRINT_MAX	(DPU_EVENT_LOG_MAX >> 3)
#define DPU_EVENT_LOG_RETRY	3
#define DPU_EVENT_KEEP_CNT	3

struct decon_debug {
	struct dentry *debug_root;
	struct dentry *debug_event;
	/* ring buffer of event log */
	struct dpu_log *event_log;
	/* count of log buffers in each event log */
	u32 event_log_cnt;
	/* array index of log buffer in event log */
	atomic_t event_log_idx;
};

struct decon_device {
	u32				id;
	enum decon_state		state;
	struct decon_regs		regs;
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct exynos_drm_crtc		*crtc;
	/* window information saved in window number order */
	struct decon_win		win[MAX_WIN_PER_DECON];
	/* dpp information saved in dpp channel number order */
	struct dpp_device		*dpp[MAX_DPP_CNT];
	u32				win_cnt;
	enum exynos_drm_output_type	con_type;
	struct decon_config		config;
	struct decon_resources		res;
	struct dpu_bts			bts;
	struct decon_debug		d;

	u32				irq_fs;	/* frame start irq number*/
	u32				irq_fd;	/* frame done irq number*/
	u32				irq_ext;/* extra irq number*/
	u32				irq_te;

	spinlock_t			slock;

	int				decon_cnt;
};

extern struct dpu_bts_ops dpu_bts_control;
extern struct decon_device *decon_drvdata[MAX_DECON_CNT];

static inline struct decon_device *get_decon_drvdata(u32 id)
{
	return decon_drvdata[id];
}

int dpu_init_debug(struct decon_device *decon);
void dpu_deinit_debug(struct decon_device *decon);
void DPU_EVENT_LOG(enum dpu_event_type type, int index, void *priv);
void DPU_EVENT_LOG_ATOMIC_COMMIT(int index);

#endif /* __EXYNOS_DRM_DECON_H__ */
