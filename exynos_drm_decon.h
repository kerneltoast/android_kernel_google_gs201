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
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
#include <soc/google/exynos_pm_qos.h>
#endif
#include <linux/notifier.h>
#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
#include <soc/google/exynos-itmon.h>
#endif
#include <soc/google/bts.h>
#include <drm/drm_device.h>
#include <video/videomode.h>

#include <decon_cal.h>

#include "exynos_drm_dpp.h"
#include "exynos_drm_dqe.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_dsim.h"

#include "exynos_drm_fb.h"
#include "exynos_drm_hibernation.h"
#include "exynos_drm_recovery.h"
#include "exynos_drm_writeback.h"
#include "exynos_drm_partial.h"

enum decon_state {
	DECON_STATE_INIT = 0,
	DECON_STATE_ON,
	DECON_STATE_HIBERNATION,
	DECON_STATE_OFF,
	DECON_STATE_HANDOVER,
};

enum dpu_win_state {
	DPU_WIN_STATE_DISABLED = 0,
	DPU_WIN_STATE_COLOR,
	DPU_WIN_STATE_BUFFER,
};

struct decon_resources {
	struct clk *aclk;
	struct clk *aclk_disp;
};

struct dpu_bts_ops {
	void (*init)(struct decon_device *decon);
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
	bool is_secure;
	bool hdr_en;
	u32 dpp_id;
	u32 zpos;
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
	u32 bpp;
	u32 src_h;
	u32 src_w;
	struct bts_layer_position dst;
	u32 bw;
	u32 rt_bw;
	bool rotation;
	bool is_afbc;
	bool is_yuv;
};

struct bts_decon_info {
	struct bts_dpp_info rdma[MAX_WIN_PER_DECON];
	struct bts_dpp_info odma;
	struct bts_dpp_info rcddma;
	u32 vclk; /* Khz */
	u32 lcd_w;
	u32 lcd_h;
};

struct decon_win_config {
	struct dpu_bts_win_config win;
	dma_addr_t dma_addr;
};

struct dpu_bts {
	bool enabled;
	bool pending_fps_update;
	u32 resol_clk;
	u32 peak;
	u32 prev_peak;
	u32 rt_avg_bw;
	u32 prev_rt_avg_bw;
	u32 read_bw;
	u32 write_bw;
	u32 total_bw;
	u32 prev_total_bw;
	u32 urgent_rd_lat;
	u32 prev_urgent_rd_lat;
	u32 max_disp_freq;
	u32 prev_max_disp_freq;
	u32 dvfs_max_disp_freq;
	u32 max_dpp_read_rt_bw;
	u32 video_num;
	u32 ppc;
	u32 ppc_rotator;
	u32 ppc_scaler;
	u32 delay_comp;
	u32 delay_scaler;
	u32 bus_width;
	u32 bus_util_pct;
	u32 rot_util_pct;
	u32 afbc_rgb_util_pct;
	u32 afbc_yuv_util_pct;
	u32 afbc_rgb_rt_util_pct;
	u32 afbc_yuv_rt_util_pct;
	u32 afbc_clk_ppc_margin;
	u32 dfs_lv_cnt;
	u32 dfs_lv_khz[BTS_DFS_MAX];
	u32 max_dfs_lv_for_wb;
	u32 vbp;
	u32 vfp;
	u32 vsa;
	u32 fps;
	u32 op_rate;
	u32 pending_vblank_usec;
	u32 vblank_usec;
	/* includes writeback dpp */
	struct dpu_bts_bw rt_bw[MAX_DPP_CNT];

	u32 ch_bw[MAX_AXI_PORT];
	int urgent_rd_lat_index[MAX_AXI_PORT];
	int bw_idx;
	struct dpu_bts_ops *ops;
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	struct exynos_pm_qos_request mif_qos;
	struct exynos_pm_qos_request int_qos;
	struct exynos_pm_qos_request disp_qos;
#endif

	struct dpu_bts_win_config win_config[MAX_WIN_PER_DECON];
	struct dpu_bts_win_config wb_config;
	struct decon_win_config rcd_win_config;
	atomic_t delayed_update;
};

struct dpu_bts_scenario {
	bool enabled;
	bool voted;
	bool skip_with_video;
	u32 min_panel_width;
	u32 min_panel_height;
	u32 min_fps;
	u32 min_rt_bw;
	u32 max_rt_bw;
	u32 min_peak_bw;
	u32 max_peak_bw;
	u32 idx;
	const char *name;
};

struct bw_latency_map {
	u32 bw_kbps;
	u32 latency_ns;
};

struct dpu_bts_urgent_lat {
	bool enabled;
	struct bw_latency_map *bw_lat_tbl;
	u32 bw_lat_map_cnt;
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
	DPU_EVT_DECON_UPDATE_CONFIG,

	DPU_EVT_DSIM_ENABLED,
	DPU_EVT_DSIM_DISABLED,
	DPU_EVT_DSIM_COMMAND,
	DPU_EVT_DSIM_ULPS_ENTER,
	DPU_EVT_DSIM_ULPS_EXIT,
	DPU_EVT_DSIM_UNDERRUN,
	DPU_EVT_DSIM_FRAMEDONE,
	DPU_EVT_DSIM_PH_FIFO_TIMEOUT,
	DPU_EVT_DSIM_PL_FIFO_TIMEOUT,

	DPU_EVT_DPP_FRAMEDONE,
	DPU_EVT_DPP_SET_PROTECTION,
	DPU_EVT_DMA_RECOVERY,

	DPU_EVT_IDMA_AFBC_CONFLICT,
	DPU_EVT_IDMA_FBC_ERROR,
	DPU_EVT_IDMA_READ_SLAVE_ERROR,
	DPU_EVT_IDMA_DEADLOCK,
	DPU_EVT_IDMA_CFG_ERROR,

	DPU_EVT_ATOMIC_COMMIT,
	DPU_EVT_TE_INTERRUPT,

	DPU_EVT_DECON_RUNTIME_SUSPEND,
	DPU_EVT_DECON_RUNTIME_RESUME,
	DPU_EVT_DECON_SUSPEND,
	DPU_EVT_DECON_RESUME,
	DPU_EVT_DSIM_RUNTIME_SUSPEND,
	DPU_EVT_DSIM_RUNTIME_RESUME,
	DPU_EVT_DSIM_SUSPEND,
	DPU_EVT_DSIM_RESUME,
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

	DPU_EVT_PLANE_PREPARE_FB,
	DPU_EVT_PLANE_CLEANUP_FB,
	DPU_EVT_PLANE_UPDATE,
	DPU_EVT_PLANE_DISABLE,

	DPU_EVT_REQ_CRTC_INFO_OLD,
	DPU_EVT_REQ_CRTC_INFO_NEW,

	DPU_EVT_FRAMESTART_TIMEOUT,

	DPU_EVT_BTS_RELEASE_BW,
	DPU_EVT_BTS_CALC_BW,
	DPU_EVT_BTS_UPDATE_BW,

	DPU_EVT_PARTIAL_INIT,
	DPU_EVT_PARTIAL_PREPARE,
	DPU_EVT_PARTIAL_UPDATE,
	DPU_EVT_PARTIAL_RESTORE,

	DPU_EVT_HIST_COLLECT_BINS,

	DPU_EVT_DSIM_CRC,
	DPU_EVT_DSIM_ECC,

	DPU_EVT_VBLANK_ENABLE,
	DPU_EVT_VBLANK_DISABLE,

	DPU_EVT_DIMMING_START,
	DPU_EVT_DIMMING_END,

	DPU_EVT_CGC_FRAMEDONE,
	DPU_EVT_ITMON_ERROR,
	DPU_EVT_SYSMMU_FAULT,

	DPU_EVT_MAX, /* End of EVENT */
};

enum dpu_event_condition {
	DPU_EVT_CONDITION_DEFAULT		= 1U << 0,
	DPU_EVT_CONDITION_UNDERRUN		= 1U << 1,
	DPU_EVT_CONDITION_FAIL_UPDATE_BW	= 1U << 2,
	DPU_EVT_CONDITION_FIFO_TIMEOUT		= 1U << 3,
	DPU_EVT_CONDITION_IDMA_ERROR		= 1U << 4,
	DPU_EVT_CONDITION_IDMA_ERROR_COMPACT	= 1U << 5,
};

#define DPU_CALLSTACK_MAX 10
struct dpu_log_dsim_cmd {
	u8 id;
	u8 d0;
	u16 len;
	void *caller[DPU_CALLSTACK_MAX];
};

struct dpu_log_dpp {
	u32 id;
	u32 win_id;
	u64 comp_src;
	u32 recovery_cnt;
	pid_t last_secure_pid; /* record last PID which wrote mst_security */
	bool mst_security;
};

struct dpu_log_win {
	u32 win_idx;
	u32 plane_idx;
	bool secure;
	bool hdr_en;
};

struct dpu_log_rsc_occupancy {
	u64 rsc_ch;
	u64 rsc_win;
};

struct dpu_log_atomic {
	struct decon_win_config win_config[MAX_WIN_PER_DECON];
	struct decon_win_config rcd_win_config;
};

/* Event log structure for DPU power domain status */
struct dpu_log_pd {
	enum decon_state decon_state;
	bool rpm_active;
	enum dsim_state dsim_state;
	bool dsim_rpm_active;
};

struct dpu_log_crtc_info {
	bool enable;
	bool active;
	bool planes_changed;
	bool mode_changed;
	bool active_changed;
	bool self_refresh;
	bool connectors_changed;
};

struct dpu_log_freqs {
	unsigned long mif_freq;
	unsigned long int_freq;
	unsigned long disp_freq;
};

struct dpu_log_bts_update {
	struct dpu_log_freqs freqs;
	u32 peak;
	u32 prev_peak;
	u32 rt_avg_bw;
	u32 prev_rt_avg_bw;
	u32 total_bw;
	u32 prev_total_bw;
	u32 urgent_rd_lat;
	u32 prev_urgent_rd_lat;
	u32 dpu_scen_idx;
};

struct dpu_log_bts_cal {
	struct dpu_log_freqs freqs;
	u32 disp_freq;
	u32 peak;
	u32 rt_avg_bw;
	u32 read_bw;
	u32 write_bw;
	u32 fps; /* bts fps */
	u32 crtc_fps; /* crtc fps */
	u32 op_rate;
};

struct dpu_log_bts_event {
	struct dpu_log_freqs freqs;
	u32 value;
};

struct dpu_log_partial {
	u32 min_w;
	u32 min_h;
	struct drm_rect prev;
	struct drm_rect req;
	struct drm_rect adj;
	bool reconfigure;
};

struct dpu_log_plane_info {
	dma_addr_t dma_addr;
	u32 index;
	u32 width;
	u32 height;
	u32 zpos;
	u32 format;
};

struct dpu_log_hist_info {
	enum exynos_histogram_id id;
};

struct dpu_log_decon_cfg {
	u32 fps;
	u32 image_width;
	u32 image_height;
	enum decon_out_type out_type;
	struct decon_mode mode;
};

struct dpu_log {
	u64 ts_nsec;
	enum dpu_event_type type;

	union {
		struct dpu_log_dpp dpp;
		struct dpu_log_atomic atomic;
		struct dpu_log_dsim_cmd cmd;
		struct dpu_log_rsc_occupancy rsc;
		struct dpu_log_pd pd;
		struct dpu_log_win win;
		struct dpu_log_crtc_info crtc_info;
		struct dpu_log_freqs freqs;
		struct dpu_log_bts_update bts_update;
		struct dpu_log_bts_cal bts_cal;
		struct dpu_log_bts_event bts_event;
		struct dpu_log_partial partial;
		struct dpu_log_plane_info plane_info;
		struct dpu_log_hist_info hist_info;
		struct dpu_log_decon_cfg decon_cfg;
		unsigned int value;
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
	/* count of crc interrupt */
	u32 crc_cnt;
	/* count of ecc interrupt */
	u32 ecc_cnt;
	/* count of idma error interrupt */
	u32 idma_err_cnt;
	/* array index of log buffer in event log */
	atomic_t event_log_idx;
	/* lock for saving log to event log buffer */
	spinlock_t event_lock;

	u32 auto_refresh_frames;

	u32 te_cnt;
	u32 framedone_cnt;
	bool force_te_on;
	bool tout_en;
};

struct decon_device {
	u32				id;
	enum decon_state		state;
	struct decon_regs		regs;
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct exynos_drm_crtc		*crtc;
	struct drm_atomic_state		*suspend_state;
	/* dpp information saved in dpp channel number order */
	struct dpp_device		*dpp[MAX_WIN_PER_DECON];
	struct dpp_device		*rcd;
	u32				dpp_cnt;
	u32				win_cnt;
	enum exynos_drm_output_type	con_type;
	struct decon_config		config;
	struct decon_resources		res;
	struct dpu_bts			bts;
	struct dpu_bts_scenario		bts_scen;
	struct dpu_bts_urgent_lat	bts_urgent_rd_lat;
	struct decon_debug		d;
	struct exynos_hibernation	*hibernation;
	struct drm_pending_vblank_event *event;
	struct exynos_dqe		*dqe;
	struct task_struct		*thread;
	struct kthread_worker		worker;
	struct kthread_work		buf_dump_work;
	struct exynos_recovery		recovery;
	struct exynos_dma		*cgc_dma;
	struct exynos_fb_handover	fb_handover;

	u32				irq_fs;	/* frame start irq number*/
	u32				irq_fd;	/* frame done irq number*/
	u32				irq_ext;/* extra irq number*/
	int				irq_te;
	int				irq_tout;
	int				irq_ds;	/* dimming start irq number */
	int				irq_de;	/* dimming end irq number */
	int				te_gpio;
	int				tout_gpio;
	atomic_t			te_ref;
	atomic_t			tout_ref;

	spinlock_t			slock;

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	struct notifier_block itmon_nb;
	bool itmon_notified;
#endif

	atomic_t frames_pending;
	atomic_t frame_transfer_pending;
	wait_queue_head_t framedone_wait;

	bool keep_unmask;
	struct exynos_partial *partial;
	bool cgc_need_update;
	bool dqe_need_update;
};

static inline struct decon_device *to_decon_device(const struct device *dev)
{
	/* could skip with dev_get_drvdata directly, but using pdev because
	that's how drvdata was set originally */
	struct platform_device *pdev = to_platform_device(dev);

	return (struct decon_device *)platform_get_drvdata(pdev);
}

extern struct dpu_bts_ops dpu_bts_control;
extern struct decon_device *decon_drvdata[MAX_DECON_CNT];

static inline struct decon_device *get_decon_drvdata(u32 id)
{
	if (id >= 0 && id < MAX_DECON_CNT)
		return decon_drvdata[id];

	return NULL;
}
bool is_console_enabled(void);
bool decon_dump_ignore(enum dpu_event_condition condition);
void decon_dump(struct decon_device *decon, struct drm_printer *p);
void decon_dump_locked(const struct decon_device *decon, struct drm_printer *p);
void decon_dump_all(struct decon_device *decon,
		enum dpu_event_condition cond, bool async_buf_dump);
void decon_enable_te_irq(struct decon_device *decon, bool enable);
void decon_enable_tout_irq(struct decon_device *decon, bool enable);
void decon_dump_event_condition(const struct decon_device *decon,
		enum dpu_event_condition condition);
int dpu_init_debug(struct decon_device *decon);
void DPU_EVENT_LOG(enum dpu_event_type type, int index, void *priv);
void DPU_EVENT_LOG_ATOMIC_COMMIT(int index);
void DPU_EVENT_LOG_CMD(struct dsim_device *dsim, u8 type, u8 d0, u16 len);
void decon_force_vblank_event(struct decon_device *decon);

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
void decon_mode_bts_pre_update(struct decon_device *decon,
				const struct drm_crtc_state *crtc_state,
				const struct drm_atomic_state *state);
void decon_mode_bts_op_rate_update(struct decon_device *decon,
				const u32 op_rate);
#else
static inline void decon_mode_bts_pre_update(struct decon_device *decon,
				const struct drm_crtc_state *crtc_state,
				const struct drm_atomic_state *state) { }
static inline void decon_mode_bts_op_rate_update(struct decon_device *decon,
				const u32 op_rate) { }
#endif

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
int dpu_itmon_notifier(struct notifier_block *nb, unsigned long action,
		void *data);
#endif

static inline struct drm_encoder *
crtc_find_first_encoder_by_type(const struct drm_crtc_state *crtc_state, u32 encoder_type)
{
	const struct drm_crtc *crtc = crtc_state->crtc;
	const struct drm_device *dev = crtc->dev;
	struct drm_encoder *encoder;

	drm_for_each_encoder_mask(encoder, dev, crtc_state->encoder_mask)
		if (encoder->crtc == crtc && encoder->encoder_type == encoder_type)
			return encoder;

	return NULL;
}

static inline struct drm_encoder*
decon_get_encoder(const struct decon_device *decon, u32 encoder_type)
{
	const struct drm_crtc *crtc = &decon->crtc->base;

	if (!crtc->state)
		return NULL;

	return crtc_find_first_encoder_by_type(crtc->state, encoder_type);
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

#define crtc_to_decon(crtc)                                                    \
	(container_of((crtc), struct exynos_drm_crtc, base)->ctx)
static inline bool is_power_on(struct drm_device *drm_dev)
{
	struct drm_crtc *crtc;
	struct decon_device *decon;
	bool ret = false;

	drm_for_each_crtc(crtc, drm_dev) {
		decon = crtc_to_decon(crtc);
		ret |= pm_runtime_active(decon->dev);
	}

	return ret;
}

static inline void decon_trigger_recovery(struct decon_device *decon)
{
	struct exynos_recovery *recovery = &decon->recovery;

	atomic_inc(&recovery->recovering);
	queue_work(system_highpri_wq, &recovery->work);
}

#endif /* __EXYNOS_DRM_DECON_H__ */
