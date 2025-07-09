/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for Samsung DisplayPort driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_DP_H__
#define __EXYNOS_DRM_DP_H__

#include <drm/drm_encoder.h>
#include <drm/drm_connector.h>
#include <drm/display/drm_dp_helper.h>
#include <linux/extcon.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_crtc.h"

#include "dp_cal.h"


int get_dp_log_level(void);

#define dp_info(dp, fmt, ...)	\
pr_info("%s: "fmt, dp->dev->driver->name, ##__VA_ARGS__)

#define dp_warn(dp, fmt, ...)	\
pr_warn("%s: "fmt, dp->dev->driver->name, ##__VA_ARGS__)

#define dp_err(dp, fmt, ...)	\
pr_err("%s: "fmt, dp->dev->driver->name, ##__VA_ARGS__)

#define dp_debug(dp, fmt, ...)	\
pr_debug("%s: "fmt, dp->dev->driver->name, ##__VA_ARGS__)

extern struct dp_device *dp_drvdata;

enum dp_state {
	DP_STATE_INIT,
	DP_STATE_ON,
	DP_STATE_RUN,
};

enum hotplug_state {
	EXYNOS_HPD_UNPLUG = 0,
	EXYNOS_HPD_PLUG,
	EXYNOS_HPD_IRQ,
};

struct dp_link {
	int  link_rate;
	u8   num_lanes;
	u8   support_tps;
	bool fast_training;
	bool enhanced_frame;
	bool fec;
	bool ssc;
};

struct dp_host {
	int  link_rate;
	u8   num_lanes;
	u8   max_bpc;
	u8   support_tps;
	bool fast_training;
	bool enhanced_frame;
	bool fec;
	bool ssc;
	bool scrambler;

	/* Link Training */
	u8  volt_swing_max;
	u8  pre_emphasis_max;
	u8  vol_swing_level[MAX_LANE_CNT];
	u8  pre_empha_level[MAX_LANE_CNT];
	u8  max_reach_value[MAX_LANE_CNT];
};

#define SINK_NAME_LEN	14	/* monitor name */
struct dp_sink {
	u8   revision;
	int  link_rate;
	u8   num_lanes;
	u8   support_tps;
	bool fast_training;
	bool enhanced_frame;
	bool dsc;
	bool fec;
	bool ssc;

	/* From EDID */
	char sink_name[SINK_NAME_LEN];
	u8   edid_manufacturer[4];
	u32  edid_product;
	u32  edid_serial;

	bool has_pcm_audio;
	u8 audio_ch_num;
	u8 audio_sample_rates;
	u8 audio_bit_rates;
};

struct dp_resources {
	int aux_ch_mux_gpio;
	int irq;
	void __iomem *link_regs;
	void __iomem *phy_regs;
#ifdef CONFIG_SOC_ZUMA
	void __iomem *phy_tca_regs;
	struct clk *dposc_clk;
#endif
};

enum dp_audio_state {
	DP_AUDIO_DISABLE = 0,
	DP_AUDIO_ENABLE,
	DP_AUDIO_START,
	DP_AUDIO_REQ_BUF_READ,
	DP_AUDIO_WAIT_BUF_FULL,
	DP_AUDIO_STOP,
};

struct dp_audio_config {
	enum dp_audio_state audio_state;
	u32 num_audio_ch;
	enum audio_sampling_frequency audio_fs;
	enum audio_bit_per_channel audio_bit;
	enum audio_16bit_dma_mode audio_packed_mode;
	enum audio_dma_word_length audio_word_length;
};

enum dp_state_for_hdcp22 {
	DP_DISCONNECT,
	DP_CONNECT,
	DP_PHYSICAL_DISCONNECT,
	DP_SHUTDOWN,
};

enum link_training_status {
	LINK_TRAINING_UNKNOWN,
	LINK_TRAINING_SUCCESS,
	LINK_TRAINING_FAILURE,
	LINK_TRAINING_FAILURE_SINK,
};

struct dp_stats_counters {
	u32 link_negotiation_failures;
	u32 edid_read_failures;
	u32 dpcd_read_failures;
	u32 edid_invalid_failures;
	u32 sink_count_invalid_failures;
	u32 link_unstable_failures;

	u32 max_res_1366_768;
	u32 max_res_1440_900;
	u32 max_res_1600_900;
	u32 max_res_1920_1080;
	u32 max_res_2560_1080;
	u32 max_res_2560_1440;
	u32 max_res_3440_1440;
	u32 max_res_3840_2160;
	u32 max_res_5120_2880;
	u32 max_res_7680_4320;
	u32 max_res_other;

	u32 fec_dsc_supported;
	u32 fec_dsc_not_supported;
};

/* DisplayPort Device */
struct dp_device {
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct drm_dp_aux dp_aux;

	enum exynos_drm_output_type output_type;

	struct device *dev;
	struct dp_resources res;

	struct workqueue_struct *dp_wq;
	struct work_struct hpd_plug_work;
	struct work_struct hpd_unplug_work;
	struct work_struct hpd_irq_work;

	struct mutex cmd_lock;
	struct mutex hpd_lock;
	struct mutex training_lock;

	/* HPD State */
	enum hotplug_state hpd_current_state;
	struct mutex hpd_state_lock;
	int dp_hotplug_error_code;

	/* DP Driver State */
	volatile enum dp_state state;

	/* DRM Mode */
	struct drm_display_mode cur_mode;
	int num_modes;
	int num_sads;

	/* DP Capabilities */
	struct dp_link link;
	struct dp_host host;
	struct dp_sink sink;

	/* DP Branch Device support */
	bool branch_dev;
	int sink_count;
	int dfp_count;

	/* BIST */
	int bist_mode;

	/* Audio */
	enum dp_audio_state audio_state;
	struct mutex audio_lock;

	/* DP HW Configurations */
	struct dp_hw_config hw_config;

	/* Type-C Userspace Cached Values */
	struct mutex typec_notification_lock;
	enum plug_orientation typec_orientation;
	enum pin_assignment typec_pin_assignment;
	enum link_training_status typec_link_training_status;
	bool aux_auto_orientation;

	/* DP Link CRCs enabled */
	bool dp_link_crc_enabled;
	struct dentry *dp_crc_enabled_debugfs_file;
	struct dentry *dp_crc_values_debugfs_file;

	bool hdcp_and_audio_enabled;

	bool restart_pending;

	/* DP stats/error counters */
	struct dp_stats_counters stats;
};

static inline struct dp_device *get_dp_drvdata(void)
{
	return dp_drvdata;
}

/* Prototypes of export symbol to handshake other modules */
/* DP Audio Prototype */
int dp_audio_config(struct dp_audio_config *audio_config);

/* HDCP 2.2 Prototype */

/* Used to update the current authentication status through uevent.
 * pass in either DRM_MODE_CONTENT_PROTECTION_ENABLED or
 * DRM_MODE_CONTENT_PROTECTION_DESIRED */
void dp_hdcp_update_cp(u32 drm_cp_status);

int  dp_dpcd_read_for_hdcp22(u32 address, u32 length, u8 *data);
int  dp_dpcd_write_for_hdcp22(u32 address, u32 length, u8 *data);

// From Exynos-HDCP2-DPLink-Inter.c
extern int hdcp_dplink_handle_irq(void);
extern void hdcp_dplink_connect_state(enum dp_state_for_hdcp22 state);

extern void dp_register_func_for_hdcp22(void (*func0)(u32 en),
	int (*func1)(u32 address, u32 length, u8 *data),
	int (*func2)(u32 address, u32 length, u8 *data));

// External functions
void dp_enable_dposc(struct dp_device *dp);
void dp_disable_dposc(struct dp_device *dp);
int dp_get_clock(struct dp_device *dp);
int dp_remap_regs_other(struct dp_device *dp);

#endif // __EXYNOS_DRM_DP_H__
