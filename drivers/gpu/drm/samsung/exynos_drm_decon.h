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
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
#include <soc/google/exynos_pm_qos.h>
#endif
#include <linux/notifier.h>

#if defined(CONFIG_EXYNOS_ITMON)
#include <soc/samsung/exynos-itmon.h>
#endif
#include <soc/google/bts.h>
#include <drm/drm_device.h>
#include <video/videomode.h>

#include <exynos_drm_dpp.h>
#include <exynos_drm_drv.h>
#include <exynos_drm_fb.h>
#include <exynos_drm_dsim.h>
#include <exynos_drm_hibernation.h>
#include <exynos_drm_writeback.h>
#include <exynos_drm_dqe.h>

#include <decon_cal.h>

enum decon_state {
	DECON_STATE_INIT = 0,
	DECON_STATE_ON,
	DECON_STATE_DOZE,
	DECON_STATE_HIBERNATION,
	DECON_STATE_DOZE_SUSPEND,
	DECON_STATE_OFF,
	DECON_STATE_TUI,
};

enum dpu_win_state {
	DPU_WIN_STATE_DISABLED = 0,
	DPU_WIN_STATE_COLOR,
	DPU_WIN_STATE_BUFFER,
};

struct decon_resources {
	struct pinctrl *pinctrl;
	struct pinctrl_state *te_on;
	struct pinctrl_state *te_off;
	struct clk *aclk;
	struct clk *aclk_disp;
};

struct dpu_bts_ops {
	void (*init)(struct decon_device *decon);
	void (*acquire_bw)(struct decon_device *decon);
	void (*release_bw)(struct decon_device *decon);
	void (*calc_bw)(struct decon_device *decon);
	void (*update_bw)(struct decon_device *decon, bool shadow_updated);
	void (*deinit)(struct decon_device *decon);
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
	bool is_comp;
	int dpp_ch;
	u32 format;
	u64 comp_src;
};

struct bts_layer_position {
	u32 x1;
	u32 x2; /* x2 = x1 + width */
	u32 y1;
	u32 y2; /* y2 = y1 + height */
};

struct bts_dpp_info {
	bool used;
	u32 bpp;
	u32 src_h;
	u32 src_w;
	struct bts_layer_position dst;
	u32 bw;
	bool rotation;
};

struct bts_decon_info {
	struct bts_dpp_info rdma[MAX_WIN_PER_DECON];
	struct bts_dpp_info odma;
	u32 vclk; /* Khz */
	u32 lcd_w;
	u32 lcd_h;
};

struct dpu_bts {
	bool enabled;
	u32 resol_clk;
	u32 peak;
	u32 read_bw;
	u32 write_bw;
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
	/* includes writeback dpp */
	struct dpu_bts_bw bw[MAX_DPP_CNT];

	/* each decon must know other decon's BW to get overall BW */
	u32 ch_bw[3][MAX_DECON_CNT];
	int bw_idx;
	struct bts_decon_info bts_info;
	struct dpu_bts_ops *ops;
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	struct exynos_pm_qos_request mif_qos;
	struct exynos_pm_qos_request int_qos;
	struct exynos_pm_qos_request disp_qos;
#endif

	u32 scen_updated;

	struct dpu_bts_win_config win_config[MAX_WIN_PER_DECON];
	struct dpu_bts_win_config wb_config;
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
	DPU_EVT_DECON_FRAMESTART,
	DPU_EVT_DECON_RSC_OCCUPANCY,
	DPU_EVT_DECON_TRIG_MASK,

	DPU_EVT_DSIM_ENABLED,
	DPU_EVT_DSIM_DISABLED,
	DPU_EVT_DSIM_COMMAND,
	DPU_EVT_DSIM_UNDERRUN,
	DPU_EVT_DSIM_FRAMEDONE,

	DPU_EVT_DPP_FRAMEDONE,
	DPU_EVT_DMA_RECOVERY,

	DPU_EVT_ATOMIC_COMMIT,
	DPU_EVT_TE_INTERRUPT,

	DPU_EVT_ENTER_HIBERNATION_IN,
	DPU_EVT_ENTER_HIBERNATION_OUT,
	DPU_EVT_EXIT_HIBERNATION_IN,
	DPU_EVT_EXIT_HIBERNATION_OUT,

	DPU_EVT_ATOMIC_BEGIN,
	DPU_EVT_ATOMIC_FLUSH,

	DPU_EVT_WB_ENABLE,
	DPU_EVT_WB_DISABLE,
	DPU_EVT_WB_ATOMIC_COMMIT,
	DPU_EVT_WB_FRAMEDONE,
	DPU_EVT_WB_ENTER_HIBERNATION,
	DPU_EVT_WB_EXIT_HIBERNATION,

	DPU_EVT_PLANE_UPDATE,
	DPU_EVT_PLANE_DISABLE,

	DPU_EVT_REQ_CRTC_INFO_OLD,
	DPU_EVT_REQ_CRTC_INFO_NEW,

	DPU_EVT_FRAMESTART_TIMEOUT,

	DPU_EVT_MAX, /* End of EVENT */
};

#define DPU_CALLSTACK_MAX 10
struct dpu_log_dsim_cmd {
	u32 id;
	u8 buf;
	void *caller[DPU_CALLSTACK_MAX];
};

struct dpu_log_dpp {
	u32 id;
	u64 comp_src;
	u32 recovery_cnt;
};

struct dpu_log_win {
	u32 win_idx;
	u32 plane_idx;
};

struct dpu_log_rsc_occupancy {
	u32 rsc_ch;
	u32 rsc_win;
};

struct decon_win_config {
	struct dpu_bts_win_config win;
	dma_addr_t dma_addr;
};

struct dpu_log_atomic {
	struct decon_win_config win_config[MAX_WIN_PER_DECON];
};

/* Event log structure for DPU power domain status */
struct dpu_log_pd {
	bool rpm_active;
};

struct dpu_log_crtc_info {
	bool enable;
	bool active;
	bool planes_changed;
	bool mode_changed;
	bool active_changed;
};

struct dpu_log {
	ktime_t time;
	enum dpu_event_type type;

	union {
		struct dpu_log_dpp dpp;
		struct dpu_log_atomic atomic;
		struct dpu_log_dsim_cmd cmd;
		struct dpu_log_rsc_occupancy rsc;
		struct dpu_log_pd pd;
		struct dpu_log_win win;
		struct dpu_log_crtc_info crtc_info;
	} data;
};

/* Definitions below are used in the DECON */
#define DPU_EVENT_LOG_RETRY	3
#define DPU_EVENT_KEEP_CNT	3

struct decon_debug {
	/* ring buffer of event log */
	struct dpu_log *event_log;
	/* count of log buffers in each event log */
	u32 event_log_cnt;
	/* count of underrun interrupt */
	u32 underrun_cnt;
	/* array index of log buffer in event log */
	atomic_t event_log_idx;
	/* lock for saving log to event log buffer */
	spinlock_t event_lock;
};

struct decon_device {
	u32				id;
	enum decon_state		state;
	struct decon_regs		regs;
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct exynos_drm_crtc		*crtc;
	/* dpp information saved in dpp channel number order */
	struct dpp_device		*dpp[MAX_WIN_PER_DECON];
	u32				dpp_cnt;
	u32				win_cnt;
	enum exynos_drm_output_type	con_type;
	struct decon_config		config;
	struct decon_resources		res;
	struct dpu_bts			bts;
	struct decon_debug		d;
	struct exynos_hibernation	*hibernation;
	struct completion		framestart_done;
	struct exynos_dqe		*dqe;

	u32				irq_fs;	/* frame start irq number*/
	u32				irq_fd;	/* frame done irq number*/
	u32				irq_ext;/* extra irq number*/
	u32				irq_te;

	spinlock_t			slock;

#if defined(CONFIG_EXYNOS_ITMON)
	struct notifier_block itmon_nb;
	bool itmon_notified;
#endif

	bool busy;
	wait_queue_head_t framedone_wait;
};

extern struct dpu_bts_ops dpu_bts_control;
extern struct decon_device *decon_drvdata[MAX_DECON_CNT];

static inline struct decon_device *get_decon_drvdata(u32 id)
{
	if (id >= 0 && id < MAX_DECON_CNT)
		return decon_drvdata[id];

	return NULL;
}

void decon_dump(struct decon_device *decon);
int dpu_init_debug(struct decon_device *decon);
void DPU_EVENT_LOG(enum dpu_event_type type, int index, void *priv);
void DPU_EVENT_LOG_ATOMIC_COMMIT(int index);
void DPU_EVENT_LOG_CMD(int index, struct dsim_device *dsim, u32 cmd_id,
		unsigned long data);

void decon_enter_hibernation(struct decon_device *decon);
void decon_exit_hibernation(struct decon_device *decon);

#if defined(CONFIG_EXYNOS_ITMON)
int dpu_itmon_notifier(struct notifier_block *nb, unsigned long action,
		void *data);
#endif

static inline struct drm_encoder*
decon_get_encoder(const struct decon_device *decon, u32 encoder_type)
{
	const struct drm_crtc *crtc = &decon->crtc->base;
	const struct drm_device *dev = crtc->dev;
	struct drm_encoder *encoder;

	if (!crtc->state)
		return NULL;

	drm_for_each_encoder_mask(encoder, dev, crtc->state->encoder_mask)
		if (encoder->crtc == crtc &&
				encoder->encoder_type == encoder_type)
			return encoder;

	return NULL;
}

static inline struct dsim_device*
decon_get_dsim(struct decon_device *decon)
{
	struct drm_encoder *encoder;

	encoder = decon_get_encoder(decon, DRM_MODE_ENCODER_DSI);
	if (!encoder)
		return NULL;

	return container_of(encoder, struct dsim_device, encoder);
}

static inline struct writeback_device*
decon_get_wb(struct decon_device *decon)
{
	struct drm_encoder *encoder;
	struct drm_writeback_connector *wb_connector;

	encoder = decon_get_encoder(decon, DRM_MODE_ENCODER_VIRTUAL);
	if (!encoder)
		return NULL;

	wb_connector = container_of(encoder, struct drm_writeback_connector,
			encoder);

	if (!wb_connector)
		return NULL;

	return container_of(wb_connector, struct writeback_device, writeback);
}
#endif /* __EXYNOS_DRM_DECON_H__ */
