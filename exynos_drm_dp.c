// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *
 * Samsung DisplayPort driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/of_address.h>
#include <linux/irq.h>
#include <linux/hdmi.h>
#include <linux/reboot.h>
#include <video/videomode.h>

#include <drm/display/drm_hdcp_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_edid.h>

#include "exynos_drm_dp.h"

struct dp_device *dp_drvdata;
EXPORT_SYMBOL_GPL(dp_drvdata);

struct blocking_notifier_head dp_ado_notifier_head =
		BLOCKING_NOTIFIER_INIT(dp_ado_notifier_head);
EXPORT_SYMBOL_GPL(dp_ado_notifier_head);

#define DP_SUPPORT_TPS(_v) BIT((_v)-1)

static inline struct dp_device *encoder_to_dp(struct drm_encoder *e)
{
	return container_of(e, struct dp_device, encoder);
}

static inline struct dp_device *connector_to_dp(struct drm_connector *c)
{
	return container_of(c, struct dp_device, connector);
}

static inline struct dp_device *dp_aux_to_dp(struct drm_dp_aux *a)
{
	return container_of(a, struct dp_device, dp_aux);
}

static u32 dp_get_audio_state(struct dp_device *dp)
{
	u32 audio_state;

	mutex_lock(&dp->audio_lock);
	audio_state = dp->audio_state;
	mutex_unlock(&dp->audio_lock);

	return audio_state;
}

static void dp_set_audio_state(struct dp_device *dp, u32 state)
{
	mutex_lock(&dp->audio_lock);
	dp->audio_state = state;
	mutex_unlock(&dp->audio_lock);
}

static enum hotplug_state dp_get_hpd_state(struct dp_device *dp)
{
	enum hotplug_state hpd_current_state;

	mutex_lock(&dp->hpd_state_lock);
	hpd_current_state = dp->hpd_current_state;
	mutex_unlock(&dp->hpd_state_lock);

	return hpd_current_state;
}

static void dp_set_hpd_state(struct dp_device *dp,
			     enum hotplug_state hpd_current_state)
{
	mutex_lock(&dp->hpd_state_lock);
	dp_info(dp, "DP HPD changed to %d\n", hpd_current_state);
	dp->hpd_current_state = hpd_current_state;
	mutex_unlock(&dp->hpd_state_lock);
}

static void dp_init_info(struct dp_device *dp)
{
	dp->state = DP_STATE_INIT;
	dp->hpd_current_state = EXYNOS_HPD_UNPLUG;
	dp->audio_state = DP_AUDIO_DISABLE;
	dp->output_type = EXYNOS_DISPLAY_TYPE_DP0_SST1;
	dp->dp_link_crc_enabled = false;
}

static int dp_get_max_link_rate(struct dp_device *dp)
{
	/*
	 * When the host is limited in software to RBR and
	 * the sink supports only max 2 lanes, it leads to
	 * best-case link of RBR/2 lanes. Such link cannot
	 * support 1920x1080@60 video resolution, which is
	 * not an optimal experience.
	 *
	 * For max 2-lane devices, such as USB 3.0 hubs,
	 * let's bump the link speed to HBR, so we can get
	 * HBR/2 lanes link for 1920x1080@60 support.
	 */
	if (dp->host.link_rate == drm_dp_bw_code_to_link_rate(DP_LINK_BW_1_62) &&
	    dp->sink.num_lanes < 4) {
		return min(drm_dp_bw_code_to_link_rate(DP_LINK_BW_2_7), dp->sink.link_rate);
	} else {
		return min(dp->host.link_rate, dp->sink.link_rate);
	}
}

static u8 dp_get_max_num_lanes(struct dp_device *dp)
{
	return min(dp->host.num_lanes, dp->sink.num_lanes);
}

static u8 dp_get_supported_pattern(struct dp_device *dp)
{
	return (dp->host.support_tps & dp->sink.support_tps);
}

static bool dp_get_enhanced_mode(struct dp_device *dp)
{
	return (dp->host.enhanced_frame && dp->sink.enhanced_frame);
}

static bool dp_get_ssc(struct dp_device *dp)
{
	return (dp->host.ssc && dp->sink.ssc);
}

static bool dp_get_fec(struct dp_device *dp)
{
	return (dp->host.fec && dp->sink.fec);
}

static bool dp_get_fast_training(struct dp_device *dp)
{
	return (dp->host.fast_training && dp->sink.fast_training);
}

#define MAX_VOLTAGE_LEVEL 3
#define MAX_PREEMPH_LEVEL 3

#define DP_LINK_RATE_RBR  0
#define DP_LINK_RATE_HBR  1
#define DP_LINK_RATE_HBR2 2
#define DP_LINK_RATE_HBR3 3

static unsigned long dp_rate = DP_LINK_RATE_RBR;    /* RBR is the default */
module_param(dp_rate, ulong, 0664);
MODULE_PARM_DESC(dp_rate, "use specific DP link rate by setting dp_rate=x");

static unsigned long dp_lanes = 4;    /* 4 lanes is the default */
module_param(dp_lanes, ulong, 0664);
MODULE_PARM_DESC(dp_lanes, "use specific number of DP lanes by setting dp_lanes=x");

static unsigned long dp_bpc = 8;    /* 8 bpc is the default */
module_param(dp_bpc, ulong, 0664);
MODULE_PARM_DESC(dp_bpc, "use specific BPC by setting dp_bpc=x");

static bool dp_fec = false;
module_param(dp_fec, bool, 0664);
MODULE_PARM_DESC(dp_fec, "Enable/disable DP link forward error correction");

static bool dp_ssc = true;
module_param(dp_ssc, bool, 0664);
MODULE_PARM_DESC(dp_ssc, "Enable/disable DP link spread spectrum clocking");

static bool dp_phy_boost = true;
module_param(dp_phy_boost, bool, 0664);
MODULE_PARM_DESC(dp_phy_boost, "Enable/disable DP PHY current boost");

#define DP_BIST_OFF     0
#define DP_BIST_ON      1
#define DP_BIST_ON_HDCP 2

static unsigned long dp_bist_mode = DP_BIST_OFF;
module_param(dp_bist_mode, ulong, 0664);
MODULE_PARM_DESC(dp_bist_mode, "use BIST mode by setting dp_bist_mode=x");

static bool dp_edid_hexdump = false;
module_param(dp_edid_hexdump, bool, 0664);
MODULE_PARM_DESC(dp_edid_hexdump, "Enable/disable DP EDID hexdump");

static bool dp_audio = true;
module_param(dp_audio, bool, 0664);
MODULE_PARM_DESC(dp_audio, "Enable/disable DP audio");

static int dp_emulation_mode;

static void dp_fill_host_caps(struct dp_device *dp)
{
	switch (dp_rate) {
	case DP_LINK_RATE_RBR:
		dp->host.link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_1_62);
		break;
	case DP_LINK_RATE_HBR:
		dp->host.link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_2_7);
		break;
	case DP_LINK_RATE_HBR3:
		dp->host.link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_8_1);
		break;
	case DP_LINK_RATE_HBR2:
	default:
		dp->host.link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_5_4);
		break;
	}

	switch (dp_lanes) {
	case 1:
		dp->host.num_lanes = 1;
		break;
	case 2:
		dp->host.num_lanes = 2;
		break;
	case 4:
	default:
		dp->host.num_lanes = 4;
		break;
	}

	switch (dp_bpc) {
	case 10:
		dp->host.max_bpc = 10;
		break;
	case 8:
		dp->host.max_bpc = 8;
		break;
	case 6:
	default:
		dp->host.max_bpc = 6;
		break;
	}

	switch (dp_bist_mode) {
	case DP_BIST_ON_HDCP:
		dp->bist_mode = DP_BIST_ON_HDCP;
		break;
	case DP_BIST_ON:
		dp->bist_mode = DP_BIST_ON;
		break;
	case DP_BIST_OFF:
	default:
		dp->bist_mode = DP_BIST_OFF;
		break;
	}

	dp->host.volt_swing_max = MAX_VOLTAGE_LEVEL;
	dp->host.pre_emphasis_max = MAX_PREEMPH_LEVEL;
	dp->host.support_tps = DP_SUPPORT_TPS(1) | DP_SUPPORT_TPS(2) |
			       DP_SUPPORT_TPS(3) | DP_SUPPORT_TPS(4);
	dp->host.fast_training = false;
	dp->host.enhanced_frame = true;
	dp->host.scrambler = true;
	dp->host.fec = dp_fec;
	dp->host.ssc = dp_ssc;
}

static bool dp_check_fec_caps(struct dp_device *dp, u8 fec_dpcd)
{
	u8 fec_data;

	if (!drm_dp_sink_supports_fec(fec_dpcd) || !dp->host.fec)
		return false;

	fec_data = DP_FEC_DECODE_EN_DETECTED | DP_FEC_DECODE_DIS_DETECTED;
	if (drm_dp_dpcd_writeb(&dp->dp_aux, DP_FEC_STATUS, fec_data) < 0)
		return false;

	fec_data = DP_FEC_READY;
	if (drm_dp_dpcd_writeb(&dp->dp_aux, DP_FEC_CONFIGURATION, fec_data) < 0)
		return false;

	if (drm_dp_dpcd_readb(&dp->dp_aux, DP_FEC_CONFIGURATION, &fec_data) < 0)
		return false;

	return true;
}

static void dp_fill_sink_caps(struct dp_device *dp, u8 dpcd[DP_RECEIVER_CAP_SIZE])
{
	u8 fec_dpcd = 0;
	u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE + 1];

	memset(&dp->sink, 0, sizeof(struct dp_sink));

	dp->sink.revision = dpcd[0];
	dp->sink.link_rate = drm_dp_max_link_rate(dpcd);
	dp->sink.num_lanes = drm_dp_max_lane_count(dpcd);
	dp->sink.enhanced_frame = drm_dp_enhanced_frame_cap(dpcd);

	/* Set SSC support */
	dp->sink.ssc = !!(dpcd[DP_MAX_DOWNSPREAD] & DP_MAX_DOWNSPREAD_0_5);

	/* Set TPS support */
	dp->sink.support_tps = DP_SUPPORT_TPS(1) | DP_SUPPORT_TPS(2);
	if (drm_dp_tps3_supported(dpcd))
		dp->sink.support_tps |= DP_SUPPORT_TPS(3);
	if (drm_dp_tps4_supported(dpcd))
		dp->sink.support_tps |= DP_SUPPORT_TPS(4);

	/* Set fast link support */
	dp->sink.fast_training = drm_dp_fast_training_cap(dpcd);

	/* Set FEC support */
	if (drm_dp_dpcd_readb(&dp->dp_aux, DP_FEC_CAPABILITY, &fec_dpcd) == 1) {
		dp->sink.fec = dp_check_fec_caps(dp, fec_dpcd);
	} else {
		dp->stats.dpcd_read_failures++;
		dp_warn(dp, "DP Sink: failed to read FEC support register\n");
	}

	/* Set DSC support */
	if (drm_dp_dpcd_read(&dp->dp_aux, DP_DSC_SUPPORT, dsc_dpcd, sizeof(dsc_dpcd)) ==
			sizeof(dsc_dpcd)) {
		dp->sink.dsc = !!dsc_dpcd[0];
	} else {
		dp->stats.dpcd_read_failures++;
		dp_warn(dp, "DP Sink: failed to read DSC support registers\n");
	}
}

// Callback function for DRM DP Helper
static ssize_t dp_aux_transfer(struct drm_dp_aux *dp_aux,
			       struct drm_dp_aux_msg *msg)
{
	struct dp_device *dp = dp_aux_to_dp(dp_aux);
	int ret;

	switch (msg->request) {
	case DP_AUX_NATIVE_WRITE:
		ret = dp_hw_write_dpcd_burst(msg->address, msg->size,
					     msg->buffer);
		break;
	case DP_AUX_NATIVE_READ:
		ret = dp_hw_read_dpcd_burst(msg->address, msg->size,
					    msg->buffer);
		break;
	default:
		dp_err(dp, "unsupported DP Request(0x%X)\n", msg->request);
		return -EINVAL;
	}

	if (ret)
		msg->reply = DP_AUX_NATIVE_REPLY_NACK;
	else {
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		ret = msg->size;
	}

	// Should be return size or error code
	return ret;
}

// Callback function for DRM EDID Helper
static int dp_get_edid_block(void *data, u8 *edid, unsigned int block,
			     size_t length)
{
	struct dp_device *dp = data;

	// Hard-coded EDID for DP emulation mode
	static const unsigned char emul_edid_block0[] = {
		0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1e, 0x6d, 0x01, 0x00, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x1b, 0x01, 0x03, 0x80, 0xa0, 0x5a, 0x78, 0x0a, 0xee,
		0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26, 0x0f, 0x50, 0x54, 0xa1, 0x08, 0x00, 0x31,
		0x40, 0x45, 0x40, 0x61, 0x40, 0x71, 0x40, 0x81, 0x80, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x08, 0xe8, 0x00, 0x30, 0xf2, 0x70, 0x5a, 0x80, 0xb0, 0x58, 0x8a,
		0x00, 0x40, 0x84, 0x63, 0x00, 0x00, 0x1e, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38,
		0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0x40, 0x84, 0x63, 0x00, 0x00, 0x1e, 0x00,
		0x00, 0x00, 0xfd, 0x00, 0x3a, 0x79, 0x1e, 0x88, 0x3c, 0x00, 0x0a, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x4c, 0x47, 0x20, 0x54,
		0x56, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x63
	};
	static const unsigned char emul_edid_block1[] = {
		0x02, 0x03, 0x60, 0xf1, 0x5a, 0x61, 0x60, 0x10, 0x1f, 0x66, 0x65, 0x04, 0x13,
		0x05, 0x14, 0x03, 0x02, 0x12, 0x20, 0x21, 0x22, 0x15, 0x01, 0x5d, 0x5e, 0x5f,
		0x62, 0x63, 0x64, 0x3f, 0x40, 0x2f, 0x09, 0x57, 0x07, 0x15, 0x07, 0x50, 0x57,
		0x07, 0x01, 0x3d, 0x06, 0xc0, 0x67, 0x04, 0x03, 0x6e, 0x03, 0x0c, 0x00, 0x30,
		0x00, 0xb8, 0x3c, 0x20, 0x00, 0x80, 0x01, 0x02, 0x03, 0x04, 0x67, 0xd8, 0x5d,
		0xc4, 0x01, 0x78, 0x80, 0x03, 0xe2, 0x00, 0xcf, 0xe3, 0x05, 0xc0, 0x00, 0xe3,
		0x06, 0x0d, 0x01, 0xe2, 0x0f, 0x33, 0xeb, 0x01, 0x46, 0xd0, 0x00, 0x26, 0x0a,
		0x09, 0x75, 0x80, 0x5b, 0x6c, 0x66, 0x21, 0x50, 0xb0, 0x51, 0x00, 0x1b, 0x30,
		0x40, 0x70, 0x36, 0x00, 0x40, 0x84, 0x63, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe1
	};

	if (!dp_emulation_mode) {
		dp_hw_read_edid(block, length, edid);
		if (dp_edid_hexdump) {
			dp_info(dp, "EDID: block %u length %zu\n", block, length);
			print_hex_dump(KERN_INFO, "exynos-drmdp: ", DUMP_PREFIX_NONE,
					16, 1, edid, length, true);
		}
		return 0;
	}

	if (block <= 1) {
		const unsigned char *blk = block ? emul_edid_block1 : emul_edid_block0;
		size_t blk_size = block ? sizeof(emul_edid_block1) : sizeof(emul_edid_block0);

		memcpy(edid, blk, min(length, blk_size));
		return 0;
	}

	dp_warn(dp, "%s: unknown EDID block %d requested\n", __func__, block);
	return -1;
}

//------------------------------------------------------------------------------
//
static int dp_sink_power_up(struct dp_device *dp, bool up)
{
	u8 val = 0;
	int ret;

	if (dp->sink.revision >= DP_DPCD_REV_11) {
		ret = drm_dp_dpcd_readb(&dp->dp_aux, DP_SET_POWER, &val);
		if (ret < 0) {
			dp_err(dp,
			       "DP Sink: failed to read sink power state\n");
			return ret;
		}

		val &= ~DP_SET_POWER_MASK;
		if (up)
			val |= DP_SET_POWER_D0;
		else
			val |= DP_SET_POWER_D3;

		ret = drm_dp_dpcd_writeb(&dp->dp_aux, DP_SET_POWER, val);
		if (ret < 0) {
			dp_err(dp,
			       "DP Sink: failed to write sink power state\n");
			return ret;
		}

		usleep_range(1000, 2000);
	}

	dp_info(dp, "DP Sink: sink power state set to %s\n", up ? "D0" : "D3");
	return 0;
}



static u32 dp_get_training_interval_us(struct dp_device *dp, u32 interval)
{
	if (interval == 0)
		return 400;
	if (interval < 5)
		return 4000 << (interval - 1);

	dp_warn(dp, "invalid link training AUX read interval value(%u)\n", interval);
	return 4000;
}

static void dp_print_swing_level(struct dp_device *dp)
{
	u8 *vol_swing_level = dp->host.vol_swing_level;
	u8 *pre_empha_level = dp->host.pre_empha_level;

	dp_info(dp, "Voltage_Swing: %02X %02X %02X %02X\n", vol_swing_level[0],
		 vol_swing_level[1], vol_swing_level[2], vol_swing_level[3]);
	dp_info(dp, "Pre_Emphasis : %02X %02X %02X %02X\n", pre_empha_level[0],
		 pre_empha_level[1], pre_empha_level[2], pre_empha_level[3]);
}

static void dp_validate_link_training(struct dp_device *dp, bool *cr_done,
				      bool *same_before_adjust,
				      bool *max_swing_reached,
				      u8 before_cr[MAX_LANE_CNT],
				      u8 after_cr[DP_LINK_STATUS_SIZE])
{
	u8 *req_vol = dp->host.vol_swing_level;
	u8 *req_pre = dp->host.pre_empha_level;
	u8 *req_max = dp->host.max_reach_value;

	u8 adjust_volt, adjust_pre;
	bool same_pre, same_volt;
	int i;

	*same_before_adjust = false;
	*max_swing_reached = false;
	*cr_done = drm_dp_clock_recovery_ok(after_cr, dp->link.num_lanes);

	for (i = 0; i < dp->link.num_lanes; i++) {
		req_max[i] = 0;

		adjust_volt = drm_dp_get_adjust_request_voltage(after_cr, i);
		req_vol[i] = min(adjust_volt, dp->host.volt_swing_max);
		if (req_vol[i] == dp->host.volt_swing_max)
			req_max[i] |= DP_TRAIN_MAX_SWING_REACHED;

		adjust_pre =
			drm_dp_get_adjust_request_pre_emphasis(after_cr, i) >>
			DP_TRAIN_PRE_EMPHASIS_SHIFT;
		req_pre[i] = min(adjust_pre, dp->host.pre_emphasis_max);
		if (req_pre[i] == dp->host.pre_emphasis_max)
			req_max[i] |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

		same_pre = (before_cr[i] & DP_TRAIN_PRE_EMPHASIS_MASK) ==
			   req_pre[i] << DP_TRAIN_PRE_EMPHASIS_SHIFT;
		same_volt = (before_cr[i] & DP_TRAIN_VOLTAGE_SWING_MASK) ==
			    req_vol[i];
		if (same_pre && same_volt)
			*same_before_adjust = true;

		if (!*cr_done && req_max[i] != 0) {
			*max_swing_reached = true;
		}
	}
}

static void dp_set_hwconfig_dplink(struct dp_device *dp)
{
	u8 link_rate = drm_dp_link_rate_to_bw_code(dp->link.link_rate);

	if (link_rate == DP_LINK_BW_1_62)
		dp->hw_config.link_rate = LINK_RATE_RBR;
	else if (link_rate == DP_LINK_BW_2_7)
		dp->hw_config.link_rate = LINK_RATE_HBR;
	else if (link_rate == DP_LINK_BW_5_4)
		dp->hw_config.link_rate = LINK_RATE_HBR2;
	else if (link_rate == DP_LINK_BW_8_1)
		dp->hw_config.link_rate = LINK_RATE_HBR3;

	dp->hw_config.num_lanes = dp->link.num_lanes;
}

static void dp_set_hwconfig_video(struct dp_device *dp)
{
	dp->hw_config.enhanced_mode = dp_get_enhanced_mode(dp);
	dp->hw_config.use_fec = dp_get_fec(dp);
	dp->hw_config.use_ssc = dp_get_ssc(dp);
}

static int dp_link_configure(struct dp_device *dp)
{
	u8 value;
	int err;

	value = drm_dp_link_rate_to_bw_code(dp->link.link_rate);
	err = drm_dp_dpcd_writeb(&dp->dp_aux, DP_LINK_BW_SET, value);
	if (err < 0) {
		dp_err(dp, "Failed to write DP_LINK_BW_SET");
		return err;
	}

	value = dp->link.num_lanes;

	if (dp_get_enhanced_mode(dp))
		value |= DP_LANE_COUNT_ENHANCED_FRAME_EN;

	err = drm_dp_dpcd_writeb(&dp->dp_aux, DP_LANE_COUNT_SET, value);
	if (err < 0) {
		dp_err(dp, "Failed to write DP_LANE_COUNT_SET");
		return err;
	}

	value = dp->link.ssc ? DP_SPREAD_AMP_0_5 : 0;
	err = drm_dp_dpcd_writeb(&dp->dp_aux, DP_DOWNSPREAD_CTRL, value);
	if (err < 0) {
		dp_err(dp, "Failed to write DP_DOWNSPREAD_CTRL");
		return err;
	}

	value = DP_SET_ANSI_8B10B;
	err = drm_dp_dpcd_writeb(&dp->dp_aux, DP_MAIN_LINK_CHANNEL_CODING_SET, value);
	if (err < 0) {
		dp_err(dp, "Failed to write DP_MAIN_LINK_CHANNEL_CODING_SET");
		return err;
	}

	return 0;
}

static int dp_init_link_training_cr(struct dp_device *dp)
{
	int ret;

	/* Disable DP Training Pattern */
	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_DISABLE);

	/* Reconfigure DP HW */
	dp_set_hwconfig_dplink(dp);
	dp_set_hwconfig_video(dp);
	ret = dp_hw_reinit(&dp->hw_config);
	if (ret) {
		dp_err(dp, "dp_hw_reinit() failed\n");
		return ret;
	}

	dp_info(dp, "HW configured with Rate(%d) and Lanes(%u)\n",
		dp->hw_config.link_rate, dp->hw_config.num_lanes);

	/* Configure FEC before link training */
	dp_hw_set_fec(dp->link.fec);

	/* Reconfigure DP Link */
	dp_link_configure(dp);

	/* Set DP Training Pattern in DP PHY */
	dp_info(dp, "enable DP training pattern 1\n");
	dp_hw_set_training_pattern(TRAINING_PATTERN_1);

	/* Enable DP Training Pattern */
	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_1 | DP_LINK_SCRAMBLING_DISABLE);

	return 0;
}

static bool dp_do_link_training_cr(struct dp_device *dp, u32 interval_us)
{
	u8 *vol_swing_level = dp->host.vol_swing_level;
	u8 *pre_empha_level = dp->host.pre_empha_level;
	u8 *max_reach_value = dp->host.max_reach_value;

	u8 lanes_data[MAX_LANE_CNT]; // before_cr
	u8 link_status[DP_LINK_STATUS_SIZE]; // after_cr
	bool cr_done;
	bool same_before_adjust, max_swing_reached, try_max_swing = false;
	u8 fail_counter_short = 0, fail_counter_long = 0;
	int i;

	dp_info(dp, "Link Training CR Phase with Rate(%d) and Lanes(%u)\n",
		dp->link.link_rate / 100, dp->link.num_lanes);

	for (i = 0; i < MAX_LANE_CNT; i++) {
		vol_swing_level[i] = 0;
		pre_empha_level[i] = 0;
		max_reach_value[i] = 0;
	}

	if (dp_init_link_training_cr(dp))
		goto err;

	// Voltage Swing
	do {
		// Set Voltage
		dp_print_swing_level(dp);
		dp_hw_set_voltage_and_pre_emphasis(&dp->hw_config,
						   vol_swing_level,
						   pre_empha_level);

		// Write Link Training
		for (i = 0; i < dp->link.num_lanes; i++)
			lanes_data[i] = (pre_empha_level[i]
					 << DP_TRAIN_PRE_EMPHASIS_SHIFT) |
					vol_swing_level[i] | max_reach_value[i];
		drm_dp_dpcd_write(&dp->dp_aux, DP_TRAINING_LANE0_SET,
				  lanes_data, dp->link.num_lanes);

		udelay(interval_us);

		// Read Link Status
		drm_dp_dpcd_read_link_status(&dp->dp_aux, link_status);

		// Validate CR
		dp_validate_link_training(dp, &cr_done, &same_before_adjust,
					  &max_swing_reached, lanes_data,
					  link_status);

		if (cr_done) {
			dp_info(dp, "CR Done. Move to Training_EQ.\n");
			return true;
		}

		fail_counter_long++;

		/*
		 * Per DP spec, if the sink requests adjustment to
		 * max voltage swing or max pre-emphasis value, it
		 * will be retried only once.
		 */
		if (max_swing_reached) {
			if (!try_max_swing) {
				dp_info(dp, "adjust to max swing level\n");
				try_max_swing = true;
				continue;
			} else {
				dp_err(dp, "reached max swing level\n");
				goto err;
			}
		}

		/*
		 * Per DP spec, if the sink requests the exact same
		 * voltage swing and pre-emphasis levels again, it
		 * will be retried only max 5 times.
		 */
		if (same_before_adjust) {
			dp_info(dp, "requested same level. Retry...\n");
			fail_counter_short++;
			continue;
		}

		fail_counter_short = 0;
	} while (fail_counter_short < 5 && fail_counter_long < 10);

err:
	dp_err(dp, "failed Link Training CR phase with Rate(%d) and Lanes(%u)\n",
	       dp->link.link_rate / 100, dp->link.num_lanes);
	return false;
}

static void dp_init_link_training_eq(struct dp_device *dp, u8 tps)
{
	/* Set DP Training Pattern */
	if (tps & DP_SUPPORT_TPS(4)) {
		dp_info(dp, "enable DP training pattern 4\n");
		dp_hw_set_training_pattern(TRAINING_PATTERN_4);
		drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
				   DP_TRAINING_PATTERN_4);
	} else if (tps & DP_SUPPORT_TPS(3)) {
		dp_info(dp, "enable DP training pattern 3\n");
		dp_hw_set_training_pattern(TRAINING_PATTERN_3);
		drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
				   DP_TRAINING_PATTERN_3 |
					   DP_LINK_SCRAMBLING_DISABLE);
	} else {
		dp_info(dp, "enable DP training pattern 2\n");
		dp_hw_set_training_pattern(TRAINING_PATTERN_2);
		drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
				   DP_TRAINING_PATTERN_2 |
					   DP_LINK_SCRAMBLING_DISABLE);
	}
}

static bool dp_do_link_training_eq(struct dp_device *dp, u32 interval_us,
				   u8 tps)
{
	u8 *vol_swing_level = dp->host.vol_swing_level;
	u8 *pre_empha_level = dp->host.pre_empha_level;
	u8 *max_reach_value = dp->host.max_reach_value;

	u8 lanes_data[MAX_LANE_CNT]; // before_cr
	u8 link_status[DP_LINK_STATUS_SIZE]; // after_cr
	bool cr_done;
	bool same_before_adjust, max_swing_reached = false;
	u8 fail_counter = 0;
	int i;

	dp_info(dp, "Link Training EQ Phase with Rate(%d) and Lanes(%u)\n",
		dp->link.link_rate / 100, dp->link.num_lanes);

	dp_init_link_training_eq(dp, tps);

	do {
		// Set Voltage
		dp_print_swing_level(dp);
		dp_hw_set_voltage_and_pre_emphasis(&dp->hw_config,
						   vol_swing_level,
						   pre_empha_level);

		// Write Link Training
		for (i = 0; i < dp->link.num_lanes; i++)
			lanes_data[i] = (pre_empha_level[i]
					 << DP_TRAIN_PRE_EMPHASIS_SHIFT) |
					vol_swing_level[i] | max_reach_value[i];
		drm_dp_dpcd_write(&dp->dp_aux, DP_TRAINING_LANE0_SET,
				  lanes_data, dp->link.num_lanes);

		udelay(interval_us);

		// Read Link Status
		drm_dp_dpcd_read_link_status(&dp->dp_aux, link_status);

		// Validate EQ
		dp_validate_link_training(dp, &cr_done, &same_before_adjust,
					  &max_swing_reached, lanes_data,
					  link_status);

		if (cr_done) {
			if (drm_dp_channel_eq_ok(link_status, dp->link.num_lanes)) {
				dp_info(dp, "EQ Done. Move to Training_Done.\n");
				return true;
			}
		} else {
			dp_err(dp, "CR failed during EQ phase\n");
			goto err;
		}

		fail_counter++;
	} while (fail_counter < 6);

err:
	dp_err(dp, "failed Link Training EQ phase with Rate(%d) and Lanes(%u)\n",
	       dp->link.link_rate / 100, dp->link.num_lanes);
	return false;
}

static void dp_get_lower_link_rate(struct dp_link *link)
{
	switch (drm_dp_link_rate_to_bw_code(link->link_rate)) {
	case DP_LINK_BW_2_7:
		link->link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_1_62);
		break;
	case DP_LINK_BW_5_4:
		link->link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_2_7);
		break;
	case DP_LINK_BW_8_1:
		link->link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_5_4);
		break;
	}
}

static int dp_do_full_link_training(struct dp_device *dp, u32 interval_us)
{
	const u8 supported_tps = dp_get_supported_pattern(dp);
	bool training_done = false;

	do {
		if (dp_get_hpd_state(dp) == EXYNOS_HPD_UNPLUG) {
			dp_info(dp, "DP Link: detected pending HPD UNPLUG\n");
			goto err;
		}

		// Link Training: CR (Clock Revovery)
		if (!dp_do_link_training_cr(dp, interval_us)) {
			if (drm_dp_link_rate_to_bw_code(dp->link.link_rate) !=
			    DP_LINK_BW_1_62) {
				dp_get_lower_link_rate(&dp->link);

				dp_info(dp,
					"reducing link rate to %d during CR phase\n",
					dp->link.link_rate / 100);
				continue;
			} else if (dp->link.num_lanes > 1) {
				dp->link.num_lanes >>= 1;
				dp->link.link_rate = dp_get_max_link_rate(dp);

				dp_info(dp,
					"reducing lanes number to %u during CR phase\n",
					dp->link.num_lanes);
				continue;
			}

			dp_err(dp, "Link training failed during CR phase\n");
			goto err;
		}

		// Link Training: Channel Equalization
		if (!dp_do_link_training_eq(dp, interval_us, supported_tps)) {
			if (drm_dp_link_rate_to_bw_code(dp->link.link_rate) !=
			    DP_LINK_BW_1_62) {
				dp_get_lower_link_rate(&dp->link);

				dp_info(dp,
					"reducing link rate to %d during EQ phase\n",
					dp->link.link_rate / 100);
				continue;
			} else if (dp->link.num_lanes > 1) {
				dp->link.num_lanes >>= 1;
				dp->link.link_rate = dp_get_max_link_rate(dp);

				dp_info(dp,
					"reducing lanes number to %u during EQ phase\n",
					dp->link.num_lanes);
				continue;
			}

			dp_err(dp, "Link training failed during EQ phase\n");
			goto err;
		}

		training_done = true;
	} while (!training_done);

	dp_info(dp, "DP Link: training done: Rate(%d Mbps) and Lanes(%u)\n",
		dp->link.link_rate / 100, dp->link.num_lanes);

	dp_hw_set_training_pattern(NORMAL_DATA);
	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
			   dp->host.scrambler ? 0 : DP_LINK_SCRAMBLING_DISABLE);

	return 0;
err:
	dp_info(dp, "DP Link: training failed\n");

	dp_hw_set_training_pattern(NORMAL_DATA);
	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_DISABLE);

	return -EIO;
}

/* Increment stats counters based off DSC and FEC support */
static void dp_stat_fec_dsc(struct dp_device *dp, bool dp_fec, bool dp_dsc)
{
	if (dp_fec && dp_dsc) {
		dp->stats.fec_dsc_supported++;
	} else {
		dp->stats.fec_dsc_not_supported++;
	}
}

static int dp_link_up(struct dp_device *dp)
{
	u8 dpcd[DP_RECEIVER_CAP_SIZE + 1];
	u8 dfp_info[DP_MAX_DOWNSTREAM_PORTS];
	u32 interval, interval_us;
	int ret;

	mutex_lock(&dp->training_lock);

	/* Fill host capabilities again, as they can be modified via sysfs */
	dp_fill_host_caps(dp);

	/* Update max BPC settings */
	dp->connector.max_bpc_property->values[1] = dp->host.max_bpc;
	dp->connector.state->max_bpc = dp->host.max_bpc;
	dp->connector.state->max_requested_bpc = dp->host.max_bpc;

	if (dp_emulation_mode) {
		dp_debug(dp, "dp_emulation_mode=1, skipped link training\n");

		// dp_emulation_mode is enabled, use hard-coded sink params.
		dp->sink.revision = DP_DPCD_REV_12;
		dp->sink.link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_5_4);
		dp->sink.num_lanes = 2;
		dp->sink.enhanced_frame = false;
		dp->sink.fec = false;
		dp->sink.ssc = false;
		dp->sink.support_tps = DP_SUPPORT_TPS(1) | DP_SUPPORT_TPS(2) |
				       DP_SUPPORT_TPS(3) | DP_SUPPORT_TPS(4);
		dp->sink.fast_training = true;
		dp->sink_count = 1;

		dp->link.link_rate = dp_get_max_link_rate(dp);
		dp->link.num_lanes = dp_get_max_num_lanes(dp);
		dp->link.enhanced_frame = dp_get_enhanced_mode(dp);
		dp->link.fec = dp_get_fec(dp);
		dp->link.ssc = dp_get_ssc(dp);
		dp->link.support_tps = dp_get_supported_pattern(dp);
		dp->link.fast_training = dp_get_fast_training(dp);

		/* Reconfigure DP HW for emulation mode */
		dp_set_hwconfig_dplink(dp);
		dp_set_hwconfig_video(dp);

		dp->hw_config.dp_emul = true;
		dp->hw_config.num_lanes = 2;
		dp->hw_config.orient_type = PLUG_NORMAL;
		dp->hw_config.pin_type = PIN_TYPE_D;
		if (!dp_hw_reinit(&dp->hw_config)) {
			dp_info(dp, "HW configured with Rate(%d) and Lanes(%u)\n",
				dp->hw_config.link_rate, dp->hw_config.num_lanes);
		} else {
			dp_err(dp, "dp_hw_reinit() failed\n");
		}

		mutex_unlock(&dp->training_lock);
		return 0;
	}

	/* Read DP Sink device's Capabilities */
	ret = drm_dp_dpcd_read(&dp->dp_aux, DP_DPCD_REV, dpcd, DP_RECEIVER_CAP_SIZE + 1);
	if (ret < 0) {
		dp_err(dp, "failed to read DP Sink device capabilities\n");
		mutex_unlock(&dp->training_lock);
		dp->stats.dpcd_read_failures++;
		return -EIO;
	}

	if (dpcd[DP_TRAINING_AUX_RD_INTERVAL] & DP_EXTENDED_RECEIVER_CAP_FIELD_PRESENT) {
		ret = drm_dp_dpcd_read(&dp->dp_aux, DP_DP13_DPCD_REV, dpcd,
				       DP_RECEIVER_CAP_SIZE + 1);
		if (ret < 0) {
			dp_err(dp, "failed to read DP Sink device capabilities\n");
			mutex_unlock(&dp->training_lock);
			dp->stats.dpcd_read_failures++;
			return -EIO;
		}
	}

	/*
	 * Sanity-check DP_DPCD_REV and DP_MAX_LINK_RATE values.
	 *
	 * Per DP CTS test 4.2.2.2, on future sinks, these values can be
	 * higher than 0x14 (DPCD r1.4) and 0x1E (HBR3).
	 *
	 * If connected to such sink, adjust the max link rate to HBR3.
	 */
	if (dpcd[DP_DPCD_REV] >= DP_DPCD_REV_14 &&
	    drm_dp_max_link_rate(dpcd) > drm_dp_bw_code_to_link_rate(DP_LINK_BW_8_1)) {
		dp_info(dp, "DP Sink: DPCD_%X MAX_LINK_RATE 0x%X, adjust max to HBR3\n",
			dpcd[DP_DPCD_REV], dpcd[DP_MAX_LINK_RATE]);
		dpcd[DP_MAX_LINK_RATE] = DP_LINK_BW_8_1;
	}

	/* Fill Sink Capabilities */
	dp_fill_sink_caps(dp, dpcd);

	/* Dump Sink Capabilities */
	dp_info(dp, "DP Sink: DPCD_%X Rate(%d Mbps) Lanes(%u) EF(%d) SSC(%d) FEC(%d) DSC(%d)\n",
		dp->sink.revision, dp->sink.link_rate / 100, dp->sink.num_lanes,
		dp->sink.enhanced_frame, dp->sink.ssc, dp->sink.fec, dp->sink.dsc);

	/* Stats check if FEC/DSC is supported */
	dp_stat_fec_dsc(dp, dp->sink.fec, dp->sink.dsc);

	/* Sanity-check the sink's max link rate */
	if (dp->sink.link_rate != drm_dp_bw_code_to_link_rate(DP_LINK_BW_1_62) &&
	    dp->sink.link_rate != drm_dp_bw_code_to_link_rate(DP_LINK_BW_2_7) &&
	    dp->sink.link_rate != drm_dp_bw_code_to_link_rate(DP_LINK_BW_5_4) &&
	    dp->sink.link_rate != drm_dp_bw_code_to_link_rate(DP_LINK_BW_8_1)) {
		dp_err(dp, "DP Sink: invalid max link rate\n");
		mutex_unlock(&dp->training_lock);
		dp->stats.dpcd_read_failures++;
		return -EINVAL;
	}

	/* Sanity-check the sink's max lane count */
	if (dp->sink.num_lanes != 1 && dp->sink.num_lanes != 2 && dp->sink.num_lanes != 4) {
		dp_err(dp, "DP Sink: invalid max lane count\n");
		mutex_unlock(&dp->training_lock);
		dp->stats.dpcd_read_failures++;
		return -EINVAL;
	}

	/* Power DP Sink device Up */
	dp_sink_power_up(dp, true);

	/* Get sink count */
	dp->sink_count = drm_dp_read_sink_count(&dp->dp_aux);
	if (dp->sink_count < 0) {
		dp_err(dp, "DP Sink: failed to read sink count\n");
		mutex_unlock(&dp->training_lock);
		dp->stats.dpcd_read_failures++;
		return -EIO;
	}

	/* Get DFP count */
	if (drm_dp_is_branch(dpcd)) {
		dp->branch_dev = true;
		dp->dfp_count = dpcd[DP_DOWN_STREAM_PORT_COUNT] & DP_PORT_COUNT_MASK;
		ret = drm_dp_dpcd_read(&dp->dp_aux, DP_DOWNSTREAM_PORT_0, dfp_info,
				       DP_MAX_DOWNSTREAM_PORTS);
		if (ret < 0) {
			dp_err(dp, "DP Branch Device: failed to read DP Downstream Port info\n");
			mutex_unlock(&dp->training_lock);
			dp->stats.dpcd_read_failures++;
			return -EIO;
		}

		dp_info(dp, "DP Branch Device: DFP count = %d, sink count = %d\n",
			dp->dfp_count, dp->sink_count);

		/* Sanity-check the sink count */
		if (dp->sink_count > dp->dfp_count + 1) {
			dp_err(dp, "DP Branch Device: invalid sink count\n");
			mutex_unlock(&dp->training_lock);
			dp->stats.sink_count_invalid_failures++;
			return -EINVAL;
		}
	} else {
		dp->branch_dev = false;
		dp->dfp_count = 0;
		dp_info(dp, "DP Sink: sink count = %d\n", dp->sink_count);

		/* Sanity-check the sink count */
		if (dp->sink_count != 1) {
			dp_err(dp, "DP Sink: invalid sink count\n");
			mutex_unlock(&dp->training_lock);
			dp->stats.sink_count_invalid_failures++;
			return -EINVAL;
		}
	}

	if (dp->sink_count == 0) {
		dp_info(dp, "DP Link: training defer: DP Branch Device, sink count = 0\n");
		mutex_unlock(&dp->training_lock);
		return 0;
	}

	/* Pick link parameters */
	dp->link.link_rate = dp_get_max_link_rate(dp);
	dp->link.num_lanes = dp_get_max_num_lanes(dp);
	dp->link.enhanced_frame = dp_get_enhanced_mode(dp);
	dp->link.fec = dp_get_fec(dp);
	dp->link.ssc = dp_get_ssc(dp);
	dp->link.support_tps = dp_get_supported_pattern(dp);
	dp->link.fast_training = dp_get_fast_training(dp);
	dp_info(dp, "DP Link: training start: Rate(%d Mbps) Lanes(%u) EF(%d) SSC(%d) FEC(%d)\n",
		dp->link.link_rate / 100, dp->link.num_lanes, dp->link.enhanced_frame,
		dp->link.ssc, dp->link.fec);

	/* Link Training */
	interval = dpcd[DP_TRAINING_AUX_RD_INTERVAL] & DP_TRAINING_AUX_RD_MASK;
	interval_us = dp_get_training_interval_us(dp, interval);
	if (!interval_us || dp_do_full_link_training(dp, interval_us)) {
		dp_err(dp, "failed to train DP Link\n");
		mutex_unlock(&dp->training_lock);
		dp->stats.link_negotiation_failures++;
		return -ENOLINK;
	}

	mutex_unlock(&dp->training_lock);
	return 0;
}

static int dp_link_down(struct dp_device *dp)
{
	if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG) {
		dp_sink_power_up(dp, false);
	}

	return 0;
}

static enum bit_depth dp_get_bpc(struct dp_device *dp)
{
	struct drm_connector *connector = &dp->connector;
	struct drm_display_info *display_info = &connector->display_info;

	/*
	 * drm_atomic_connector_check() has been called.
	 * We can use connector->state->max_bpc directly.
	 */
	u8 bpc = connector->state->max_bpc;

	dp_info(dp, "display_info->bpc = %u, bpc = %u\n", display_info->bpc, bpc);

	switch (bpc) {
	case 10:
		return BPC_10;
	case 8:
		return BPC_8;
	default:
		return BPC_6;
	}
}

static void dp_set_video_timing(struct dp_device *dp)
{
	struct videomode vm;

	drm_display_mode_to_videomode(&dp->cur_mode, &vm);

	dp->hw_config.vtiming.clock = dp->cur_mode.clock;

	dp->hw_config.vtiming.htotal = dp->cur_mode.htotal;
	dp->hw_config.vtiming.vtotal = dp->cur_mode.vtotal;
	dp->hw_config.vtiming.hfp = vm.hfront_porch;
	dp->hw_config.vtiming.hbp = vm.hback_porch;
	dp->hw_config.vtiming.hactive = vm.hactive;
	dp->hw_config.vtiming.vfp = vm.vfront_porch;
	dp->hw_config.vtiming.vbp = vm.vback_porch;
	dp->hw_config.vtiming.vactive = vm.vactive;

	dp->hw_config.vtiming.vsync_polarity =
		(dp->cur_mode.flags & DRM_MODE_FLAG_NVSYNC) ? SYNC_NEGATIVE :
							      SYNC_POSITIVE;
	dp->hw_config.vtiming.hsync_polarity =
		(dp->cur_mode.flags & DRM_MODE_FLAG_NHSYNC) ? SYNC_NEGATIVE :
							      SYNC_POSITIVE;
}

static int dp_make_avi_infoframe_data(struct dp_device *dp, struct infoframe *avi_infoframe)
{
	int ret;
	struct hdmi_avi_infoframe infoframe;
	u8 buffer[HDMI_INFOFRAME_SIZE(AVI)];

	ret = drm_hdmi_avi_infoframe_from_display_mode(&infoframe, &dp->connector, &dp->cur_mode);
	if (ret < 0) {
		dp_err(dp, "failed to setup AVI infoframe: %d\n", ret);
		return ret;
	}

	/*
	 * drm_hdmi_avi_infoframe_from_display_mode() might not always figure out
	 * the picture aspect ratio correctly, if the current mode does not match
	 * exactly one of the CEA VIC modes. If that's the case, then we try to
	 * fix it here.
	 */
	if (infoframe.picture_aspect == HDMI_PICTURE_ASPECT_NONE) {
		u16 hdisplay = dp->cur_mode.hdisplay;
		u16 vdisplay = dp->cur_mode.vdisplay;

		if (hdisplay == vdisplay * 16 / 9)
			infoframe.picture_aspect = HDMI_PICTURE_ASPECT_16_9;
		else if (hdisplay == vdisplay * 4 / 3)
			infoframe.picture_aspect = HDMI_PICTURE_ASPECT_4_3;
	}

	ret = hdmi_avi_infoframe_pack(&infoframe, buffer, sizeof(buffer));
	if (ret < 0) {
		dp_err(dp, "failed to pack AVI infoframe: %d\n", ret);
		return ret;
	}

	avi_infoframe->type_code = buffer[0];
	avi_infoframe->version_number = buffer[1];
	avi_infoframe->length = buffer[2];
	memcpy(&avi_infoframe->data[0], &buffer[4], avi_infoframe->length);

	return 0;
}

static int dp_set_avi_infoframe(struct dp_device *dp)
{
	struct infoframe avi_infoframe;

	if (!dp_make_avi_infoframe_data(dp, &avi_infoframe))
		dp_hw_send_avi_infoframe(avi_infoframe);

	return 0;
}

static int dp_make_spd_infoframe_data(struct infoframe *spd_infoframe)
{
	spd_infoframe->type_code = INFOFRAME_PACKET_TYPE_SPD;
	spd_infoframe->version_number = SPD_INFOFRAME_VERSION;
	spd_infoframe->length = SPD_INFOFRAME_LENGTH;

	/* Data bytes 1-8: vendor name */
	strncpy(&spd_infoframe->data[0], "Google", 6);

	/* Data bytes 9-24: product description */
	strncpy(&spd_infoframe->data[8], "Pixel", 5);

	return 0;
}

static int dp_set_spd_infoframe(void)
{
	struct infoframe spd_infoframe;

	memset(&spd_infoframe, 0, sizeof(spd_infoframe));
	dp_make_spd_infoframe_data(&spd_infoframe);
	dp_hw_send_spd_infoframe(spd_infoframe);

	return 0;
}


static int dp_make_audio_infoframe_data(struct dp_device *dp,
					struct infoframe *audio_infoframe)
{
	int i;

	audio_infoframe->type_code = INFOFRAME_PACKET_TYPE_AUDIO;
	audio_infoframe->version_number = AUDIO_INFOFRAME_VERSION;
	audio_infoframe->length = AUDIO_INFOFRAME_LENGTH;

	for (i = 0; i < AUDIO_INFOFRAME_LENGTH; i++)
		audio_infoframe->data[i] = 0x00;

	/*
	 * Currently, DP audio stream is always supposed to be:
	 * PCM, 2 channels, 48 kHz, 16-bit
	 */
	if (dp->hw_config.num_audio_ch != 2)
		dp_warn(dp, "%s: num_audio_ch != 2\n", __func__);
	if (dp->hw_config.audio_fs != FS_48KHZ)
		dp_warn(dp, "%s: audio_fs != 48 kHz\n", __func__);
	if (dp->hw_config.audio_bit != AUDIO_16_BIT)
		dp_warn(dp, "%s: audio_bit != 16-bit\n", __func__);

	/* Data Byte 1, PCM type + audio channel count - 1 */
	audio_infoframe->data[0] = (HDMI_AUDIO_CODING_TYPE_PCM << 4) | 0x1;

	/* Data Byte 2, Sample frequency + sample size */
	audio_infoframe->data[1] = (HDMI_AUDIO_SAMPLE_FREQUENCY_48000 << 2) |
				   HDMI_AUDIO_SAMPLE_SIZE_16;

	/* Data Byte 4, how various speaker locations are allocated */
	if (dp->hw_config.num_audio_ch == 8)
		audio_infoframe->data[3] = 0x13;
	else if (dp->hw_config.num_audio_ch == 6)
		audio_infoframe->data[3] = 0x0b;
	else
		audio_infoframe->data[3] = 0;

	dp_info(dp,
		"audio_infoframe: type and ch_cnt %02x, SF and bit size %02x, ch_allocation %02x\n",
		audio_infoframe->data[0], audio_infoframe->data[1], audio_infoframe->data[3]);

	return 0;
}

static int dp_set_audio_infoframe(struct dp_device *dp)
{
	struct infoframe audio_infoframe;

	memset(&audio_infoframe, 0, sizeof(audio_infoframe));
	dp_make_audio_infoframe_data(dp, &audio_infoframe);
	dp_hw_send_audio_infoframe(audio_infoframe);

	return 0;
}

static void dp_enable(struct drm_encoder *encoder)
{
	struct dp_device *dp = encoder_to_dp(encoder);

	if (dp->restart_pending) {
		dp_debug(dp, "%s: ignored, because of restart_pending", __func__);
		return;
	}

	mutex_lock(&dp->cmd_lock);

	dp->hw_config.bpc = dp_get_bpc(dp);
	dp->hw_config.range = VESA_RANGE;
	dp_set_video_timing(dp);

	if (dp->bist_mode == DP_BIST_OFF) {
		dp_hw_set_video_config(&dp->hw_config);
	} else {
		/* BIST mode */
		dp->hw_config.bist_mode = true;
		dp->hw_config.bist_type = COLOR_BAR;
		dp_hw_set_bist_video_config(&dp->hw_config);
	}

	dp_set_avi_infoframe(dp);
	dp_set_spd_infoframe();

	dp->dp_link_crc_enabled = false;
	enable_irq(dp->res.irq);
	dp_hw_start();
	dp_info(dp, "enabled DP as cur_mode = %s@%d\n", dp->cur_mode.name,
		drm_mode_vrefresh(&dp->cur_mode));

	if (dp->bist_mode != DP_BIST_OFF) {
		/* BIST mode */
		dp->hw_config.num_audio_ch = 2;
		dp->hw_config.audio_fs = FS_48KHZ;
		dp->hw_config.audio_bit = AUDIO_16_BIT;
		dp->hw_config.audio_packed_mode = NORMAL_MODE;
		dp->hw_config.audio_word_length = WORD_LENGTH_1;

		dp_hw_init_audio();
		dp_hw_set_bist_audio_config(&dp->hw_config);
		dp_hw_start_audio();

		dp_set_audio_infoframe(dp);
	}

	dp->state = DP_STATE_RUN;
	dp_info(dp, "%s: DP State changed to RUN\n", __func__);

	mutex_unlock(&dp->cmd_lock);
}

static void dp_disable(struct drm_encoder *encoder)
{
	struct dp_device *dp = encoder_to_dp(encoder);

	if (dp->restart_pending) {
		dp_debug(dp, "%s: ignored, because of restart_pending", __func__);
		return;
	}

	if (!pm_runtime_get_if_in_use(dp->dev)) {
		dp_debug(dp, "%s: DP is already disabled\n", __func__);
		return;
	}

	mutex_lock(&dp->cmd_lock);

	if (dp->state == DP_STATE_RUN) {
		disable_irq(dp->res.irq);
		if (dp->bist_mode != DP_BIST_OFF) {
			/* BIST mode */
			dp_hw_stop_audio();
			dp_hw_deinit_audio();
		}
		dp_hw_stop();
		dp_info(dp, "disabled DP\n");

		dp->state = DP_STATE_ON;
		dp_info(dp, "%s: DP State changed to ON\n", __func__);
	} else
		dp_info(dp, "%s: DP State is not RUN\n", __func__);

	mutex_unlock(&dp->cmd_lock);
	pm_runtime_put(dp->dev);
}

// For BIST
static void dp_parse_edid(struct dp_device *dp, struct edid *edid)
{
	u8 *edid_vendor = dp->sink.edid_manufacturer;
	u32 edid_prod_id = 0;

	if (edid == NULL)
		return;

	edid_vendor[0] = ((edid->mfg_id[0] & 0x7c) >> 2) + '@';
	edid_vendor[1] = (((edid->mfg_id[0] & 0x3) << 3) |
			  ((edid->mfg_id[1] & 0xe0) >> 5)) +
			 '@';
	edid_vendor[2] = (edid->mfg_id[1] & 0x1f) + '@';

	edid_prod_id |= EDID_PRODUCT_ID(edid);

	dp->sink.edid_product = edid_prod_id;
	dp->sink.edid_serial = edid->serial;

	drm_edid_get_monitor_name(edid, dp->sink.sink_name, SINK_NAME_LEN);

	dp_info(dp, "EDID: Sink Manufacturer: %s\n", dp->sink.edid_manufacturer);
	dp_info(dp, "EDID: Sink Product: %x\n", dp->sink.edid_product);
	dp_info(dp, "EDID: Sink Serial: %x\n", dp->sink.edid_serial);
	dp_info(dp, "EDID: Sink Name: %s\n", dp->sink.sink_name);
}

static void dp_clean_drm_modes(struct dp_device *dp)
{
	struct drm_display_mode *mode, *t;
	struct drm_connector *connector = &dp->connector;

	memset(&dp->cur_mode, 0, sizeof(struct drm_display_mode));

	list_for_each_entry_safe (mode, t, &connector->probed_modes, head) {
		list_del(&mode->head);
		drm_mode_destroy(connector->dev, mode);
	}
}

static const struct drm_display_mode failsafe_mode[1] = {
	/* 1 - 640x480@60Hz 4:3 */
	{
		DRM_MODE("640x480", DRM_MODE_TYPE_DRIVER, 25175, 640, 656, 752,
			 800, 0, 480, 490, 492, 525, 0,
			 DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
		.picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3,
	}
};

static void dp_sad_to_audio_info(struct dp_device *dp, struct cea_sad *sads)
{
	int i;

	dp->sink.has_pcm_audio = false;

	/* enum hdmi_audio_coding_type & enum hdmi_audio_sample_frequency are defined in hdmi.h */
	for (i = 0; i < dp->num_sads; i++) {
		dp_info(dp, "EDID: SAD %d: fmt: 0x%02x, ch: 0x%02x, freq: 0x%02x, byte2: 0x%02x\n",
			i + 1, sads[i].format, sads[i].channels, sads[i].freq, sads[i].byte2);

		if (sads[i].format == HDMI_AUDIO_CODING_TYPE_PCM) {
			dp->sink.has_pcm_audio = true;
			dp->sink.audio_ch_num = sads[i].channels + 1;
			dp->sink.audio_sample_rates = sads[i].freq;
			dp->sink.audio_bit_rates = sads[i].byte2;
		}
	}

	if (dp->sink.has_pcm_audio)
		dp_info(dp, "EDID: PCM audio: ch: %u, sample_rates: 0x%02x, bit_rates: 0x%02x\n",
			dp->sink.audio_ch_num, dp->sink.audio_sample_rates,
			dp->sink.audio_bit_rates);

	if (!dp_audio) {
		dp_info(dp, "DP audio path is disabled: dp_audio=N\n");
		dp->sink.has_pcm_audio = false;
	}
}

static const u8 dp_fake_edid[EDID_LENGTH] = {
	/* header */
	0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0,
	/* vendor/product info */
	0x1d, 0xef, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0x21,
	/* EDID version */
	0x1, 0x2,
	/* basic display parameters */
	0xa5, 0x46, 0x27, 0x78, 0x0,
	/* color characteristics */
	0xba, 0xc5, 0xa9, 0x53, 0x4e, 0xa6, 0x25, 0xe, 0x50, 0x54,
	/* established timings: 640x480 @ 60 */
	0x20, 0x0, 0x0,
	/* standard timings: none */
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	/* detailed timings: none */
	0x0, 0x0, 0x0, 0x10, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x10, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x10, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x10, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	/* extension flag + checksum */
	0x0, 0x97,
};

static enum drm_mode_status dp_conn_mode_valid(struct drm_connector *connector,
						struct drm_display_mode *mode);

static void dp_select_bist_mode(struct dp_device *dp)
{
	struct drm_connector *connector = &dp->connector;
	struct drm_display_mode *mode;

	drm_mode_sort(&connector->probed_modes);

	/* pick the largest @ 60 Hz mode that works */
	list_for_each_entry(mode, &connector->probed_modes, head) {
		if (dp_conn_mode_valid(connector, mode) == MODE_OK &&
		    drm_mode_vrefresh(mode) == 60) {
			goto done;
		}
	}

	/* fallback: pick the largest mode that works */
	list_for_each_entry(mode, &connector->probed_modes, head) {
		if (dp_conn_mode_valid(connector, mode) == MODE_OK) {
			goto done;
		}
	}

	/* last resort: failsafe mode */
	mode = (struct drm_display_mode *)failsafe_mode;

done:
	dp_info(dp, "Mode(" DRM_MODE_FMT ") is requested\n", DRM_MODE_ARG(mode));
	drm_mode_copy(&dp->cur_mode, mode);
}

static int dp_wait_state_change(struct dp_device *dp, int max_wait_time, enum dp_state state)
{
	int wait_cnt = max_wait_time;

	do {
		wait_cnt--;
		usleep_range(1000, 1030);
	} while ((state != dp->state) && (wait_cnt > 0));

	dp_info(dp, "dp_wait_state_change: time = %d ms, state = %d\n",
		max_wait_time - wait_cnt, state);

	return (wait_cnt > 0) ? wait_cnt : -ETIME;
}

/* Match the largest resolution obtained from the EDID against preset resolutions */
static void dp_check_max_res(struct dp_device *dp, u16 h_res, u16 v_res)
{
	/* Check if input is void */
	if (!h_res || !v_res) {
		dp_warn(dp, "Null resolution found for maximum resolution stats logging\n");
		return;
	}
	/* Check the obtained largest resolution against
	 * ten preset resolutions
	 * to increment the respective stat counter
	 */
	if (h_res == 1366 && v_res == 768) {
		dp->stats.max_res_1366_768++;
	} else if (h_res == 1440 && v_res == 900) {
		dp->stats.max_res_1440_900++;
	} else if (h_res == 1600 && v_res == 900) {
		dp->stats.max_res_1600_900++;
	} else if (h_res == 1920 && v_res == 1080) {
		dp->stats.max_res_1920_1080++;
	} else if (h_res == 2560 && v_res == 1080) {
		dp->stats.max_res_2560_1080++;
	} else if (h_res == 2560 && v_res == 1440) {
		dp->stats.max_res_2560_1440++;
	} else if (h_res == 3440 && v_res == 1440) {
		dp->stats.max_res_3440_1440++;
	} else if (h_res == 3840 && v_res == 2160) {
		dp->stats.max_res_3840_2160++;
	} else if (h_res == 5120 && v_res == 2880) {
		dp->stats.max_res_5120_2880++;
	} else if (h_res == 7680 && v_res == 4320) {
		dp->stats.max_res_7680_4320++;
	} else {
		dp->stats.max_res_other++;
	}
}

static void dp_on_by_hpd_plug(struct dp_device *dp)
{
	struct drm_connector *connector = &dp->connector;
	struct drm_device *dev = connector->dev;
	struct edid *edid;
	struct cea_sad *sads;
	struct drm_display_mode *fs_mode;
	int timeout;

	edid = drm_do_get_edid(connector, dp_get_edid_block, dp);
	if (!edid) {
		dp_err(dp, "EDID: failed to read EDID from sink, using fake EDID\n");
		dp->stats.edid_read_failures++;
		edid = kmemdup(dp_fake_edid, EDID_LENGTH, GFP_KERNEL);
	} else if (!drm_edid_is_valid(edid)) {
		dp_err(dp, "EDID: invalid EDID, using fake EDID\n");
		dp->stats.edid_invalid_failures++;
		kfree(edid);
		edid = kmemdup(dp_fake_edid, EDID_LENGTH, GFP_KERNEL);
	}

	if (drm_connector_update_edid_property(connector, edid))
		dp_err(dp, "EDID: drm_connector_update_edid_property() failed\n");

	dp_parse_edid(dp, edid);
	mutex_lock(&dev->mode_config.mutex);
	dp_clean_drm_modes(dp);
	dp->num_modes = drm_add_edid_modes(connector, edid);
	fs_mode = drm_mode_duplicate(connector->dev, failsafe_mode);
	if (fs_mode) {
		drm_mode_probed_add(connector, fs_mode);
		dp->num_modes++;
	}
	mutex_unlock(&dev->mode_config.mutex);

	dp->num_sads = drm_edid_to_sad(edid, &sads);
	dp_sad_to_audio_info(dp, sads);
	if (dp->num_sads > 0)
		kfree(sads);
	kfree(edid);

	/* Sort the obtained modes so that the largest
	 * preferred resolution is at head of the list
	 */
	drm_mode_sort(&connector->probed_modes);

	/* Set the respective stats counters from the largest preferred resolution */
	dp_check_max_res(
		dp, list_first_entry(&connector->probed_modes, typeof(*fs_mode), head)->hdisplay,
		list_first_entry(&connector->probed_modes, typeof(*fs_mode), head)->vdisplay);

	dp->state = DP_STATE_ON;
	dp_info(dp, "%s: DP State changed to ON\n", __func__);

	if (dp_get_hpd_state(dp) == EXYNOS_HPD_UNPLUG) {
		dp_info(dp, "%s: detected pending HPD UNPLUG\n", __func__);
		return;
	}

	if (dp->bist_mode == DP_BIST_OFF) {
		/*
		 * Notify userspace only about DP video path here.
		 * Once the DP connection usage has been confirmed,
		 * then enable HDCP and DP audio in dp_conn_atomic_check.
		 */
		if (dev) {
			connector->status = connector_status_connected;
			dp_info(dp,
				"call drm_kms_helper_hotplug_event (connected)\n");
			drm_kms_helper_hotplug_event(dev);

			/* Wait for DRM/KMS to call dp_enable() */
			timeout = dp_wait_state_change(dp, 1000, DP_STATE_RUN);
			if (timeout == -ETIME) {
				dp_warn(dp, "dp_wait_state_change: timeout for enable\n");
			}
		}
	} else {
		/* BIST mode */
		dp_select_bist_mode(dp);
		dp_enable(&dp->encoder);
		if (dp->bist_mode == DP_BIST_ON_HDCP)
			hdcp_dplink_connect_state(DP_CONNECT);
	}
}

static int dp_wait_audio_state_change(struct dp_device *dp, int max_wait_time,
				      enum dp_audio_state state)
{
	int ret = 0;
	int wait_cnt = max_wait_time;

	do {
		wait_cnt--;
		usleep_range(1000, 1030);
	} while ((dp_get_audio_state(dp) != state) && (wait_cnt > 0));

	dp_info(dp, "dp_wait_audio_state_change: time = %d ms, state = %d\n",
		max_wait_time - wait_cnt, state);

	if (wait_cnt == 0) {
		dp_err(dp, "dp_wait_audio_state_change: timeout\n");
		ret =  -ETIME;
	} else
		ret = wait_cnt;

	return ret;

}

static void dp_off_by_hpd_plug(struct dp_device *dp)
{
	struct drm_connector *connector = &dp->connector;
	struct drm_device *dev = connector->dev;
	int timeout;

	if (dp->state >= DP_STATE_ON) {
		if (dp->bist_mode == DP_BIST_OFF) {
			hdcp_dplink_connect_state(DP_DISCONNECT);

			if (dev) {
				connector->status =
					connector_status_disconnected;
				dp_info(dp,
					"call drm_kms_helper_hotplug_event (disconnected)\n");
				drm_kms_helper_hotplug_event(dev);
			}

			if (dp->sink.has_pcm_audio) {
				dp_info(dp, "call DP audio notifier (disconnected)\n");
				blocking_notifier_call_chain(&dp_ado_notifier_head, -1UL, NULL);
			}

			dp->hdcp_and_audio_enabled = false;

			/* Wait Audio is stopped if Audio is working. */
			if (dp_get_audio_state(dp) != DP_AUDIO_DISABLE) {
				timeout = dp_wait_audio_state_change(dp, 3000, DP_AUDIO_DISABLE);
				if (timeout == -ETIME)
					dp_err(dp, "dp_wait_audio_state_change: timeout for disable\n");
			}

			/* Wait for DRM/KMS to call dp_disable() */
			timeout = dp_wait_state_change(dp, 3000, DP_STATE_ON);
			if (timeout == -ETIME) {
				dp_err(dp, "dp_wait_state_change: timeout for disable\n");
				dp_disable(&dp->encoder);
			}
		} else {
			/* BIST mode */
			if (dp->bist_mode == DP_BIST_ON_HDCP)
				hdcp_dplink_connect_state(DP_DISCONNECT);
			dp_disable(&dp->encoder);
		}
	}
}

static void dp_get_voltage_and_pre_emphasis_max_reach(u8 *voltage_swing,
						    u8 *pre_emphasis, u8 *max_reach_value)
{
	int i;

	for (i = 0; i < MAX_LANE_CNT; i++) {
		if (voltage_swing[i] >= MAX_VOLTAGE_LEVEL)
			max_reach_value[i] |= DP_TRAIN_MAX_SWING_REACHED;

		if (pre_emphasis[i] >= MAX_PREEMPH_LEVEL)
			max_reach_value[i] |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;
	}
}

static void dp_automated_test_set_lane_req(struct dp_device *dp, u8 *val)
{
	u8 voltage_swing[MAX_LANE_CNT];
	u8 pre_emphasis[MAX_LANE_CNT];
	u8 max_reach_value[MAX_LANE_CNT] = {0, };
	u8 lanes_data[MAX_LANE_CNT];
	int i;

	voltage_swing[0] = (val[0] & DP_ADJUST_VOLTAGE_SWING_LANE0_MASK) >>
			   DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT;
	pre_emphasis[0]  = (val[0] & DP_ADJUST_PRE_EMPHASIS_LANE0_MASK) >>
			   DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT;
	voltage_swing[1] = (val[0] & DP_ADJUST_VOLTAGE_SWING_LANE1_MASK) >>
			   DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT;
	pre_emphasis[1]  = (val[0] & DP_ADJUST_PRE_EMPHASIS_LANE1_MASK) >>
			   DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT;
	voltage_swing[2] = (val[1] & DP_ADJUST_VOLTAGE_SWING_LANE0_MASK) >>
			   DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT;
	pre_emphasis[2]  = (val[1] & DP_ADJUST_PRE_EMPHASIS_LANE0_MASK) >>
			   DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT;
	voltage_swing[3] = (val[1] & DP_ADJUST_VOLTAGE_SWING_LANE1_MASK) >>
			   DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT;
	pre_emphasis[3]  = (val[1] & DP_ADJUST_PRE_EMPHASIS_LANE1_MASK) >>
			   DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT;

	for (i = 0; i < MAX_LANE_CNT; i++) {
		dp_info(dp, "AutoTest: voltage swing[%d] = %x\n", i, voltage_swing[i]);
		dp_info(dp, "AutoTest: pre_emphasis[%d] = %x\n", i, pre_emphasis[i]);
	}

	dp_hw_set_voltage_and_pre_emphasis(&dp->hw_config, voltage_swing, pre_emphasis);
	dp_get_voltage_and_pre_emphasis_max_reach(voltage_swing, pre_emphasis,
						max_reach_value);

	lanes_data[0] = (pre_emphasis[0] << DP_TRAIN_PRE_EMPHASIS_SHIFT) |
			voltage_swing[0] | max_reach_value[0];
	lanes_data[1] = (pre_emphasis[1] << DP_TRAIN_PRE_EMPHASIS_SHIFT) |
			voltage_swing[1] | max_reach_value[1];
	lanes_data[2] = (pre_emphasis[2] << DP_TRAIN_PRE_EMPHASIS_SHIFT) |
			voltage_swing[2] | max_reach_value[2];
	lanes_data[3] = (pre_emphasis[3] << DP_TRAIN_PRE_EMPHASIS_SHIFT) |
			voltage_swing[3] | max_reach_value[3];

	dp_info(dp, "AutoTest: set %02x %02x %02x %02x\n",
		lanes_data[0], lanes_data[1], lanes_data[2], lanes_data[3]);

	drm_dp_dpcd_write(&dp->dp_aux, DP_TRAINING_LANE0_SET, lanes_data, MAX_LANE_CNT);
}

static int dp_automated_test_irq_handler(struct dp_device *dp)
{
	u8 dpcd_test_req = 0;
	u8 dpcd_test_res = DP_TEST_ACK;
	u8 dpcd_req_lane[2] = { 0, 0 };
	u8 dpcd_phy_test_pattern = 0;

	drm_dp_dpcd_readb(&dp->dp_aux, DP_TEST_REQUEST, &dpcd_test_req);

	if (dpcd_test_req & DP_TEST_LINK_PHY_TEST_PATTERN) {
		dp_info(dp, "Automated Test: PHY Test Pattern\n");

		dp_hw_stop();
		msleep(120);

		/* Set Swing & Preemp */
		drm_dp_dpcd_read(&dp->dp_aux, DP_ADJUST_REQUEST_LANE0_1, dpcd_req_lane, 2);
		dp_automated_test_set_lane_req(dp, dpcd_req_lane);

		/* Set Quality Pattern */
		drm_dp_dpcd_readb(&dp->dp_aux, DP_PHY_TEST_PATTERN, &dpcd_phy_test_pattern);
		switch (dpcd_phy_test_pattern & DP_PHY_TEST_PATTERN_SEL_MASK) {
		case DP_PHY_TEST_PATTERN_NONE:
			dp_hw_set_quality_pattern(DISABLE_PATTERN, ENABLE_SCRAM);
			break;
		case DP_PHY_TEST_PATTERN_D10_2:
			dp_hw_set_quality_pattern(D10_2_PATTERN, DISABLE_SCRAM);
			break;
		case DP_PHY_TEST_PATTERN_ERROR_COUNT:
			dp_hw_set_quality_pattern(SERP_PATTERN, ENABLE_SCRAM);
			break;
		case DP_PHY_TEST_PATTERN_PRBS7:
			dp_hw_set_quality_pattern(PRBS7, DISABLE_SCRAM);
			break;
		case DP_PHY_TEST_PATTERN_80BIT_CUSTOM:
			dp_hw_set_quality_pattern(CUSTOM_80BIT, DISABLE_SCRAM);
			break;
		case DP_PHY_TEST_PATTERN_CP2520:
			dp_hw_set_quality_pattern(HBR2_COMPLIANCE, ENABLE_SCRAM);
			break;
		default:
			dp_err(dp, "Unsupported PHY Test Pattern: %02x\n", dpcd_phy_test_pattern);
			dpcd_test_res = DP_TEST_NAK;
			break;
		}

	} else if (dpcd_test_req & DP_TEST_LINK_EDID_READ) {
		u8 data[EDID_LENGTH];
		u8 ext_count;
		u8 checksum;
		u8 i;

		dp_info(dp, "Automated Test: EDID Read\n");

		/* read EDID block 0 */
		dp_hw_read_edid(0, EDID_LENGTH, data);
		dp_info(dp, "EDID: block 0\n");
		print_hex_dump(KERN_INFO, "exynos-drmdp: ", DUMP_PREFIX_NONE, 16, 1,
				data, EDID_LENGTH, true);

		/* read all extension blocks */
		ext_count = data[EDID_LENGTH - 2];
		for (i = 1; i <= ext_count; i++) {
			dp_hw_read_edid(i, EDID_LENGTH, data);
			dp_info(dp, "EDID: block %u\n", i);
			print_hex_dump(KERN_INFO, "exynos-drmdp: ", DUMP_PREFIX_NONE, 16, 1,
					data, EDID_LENGTH, true);
		}

		/* re-calculate checksum from the last EDID block */
		checksum = 0;
		for (i = 0; i < EDID_LENGTH - 1; i++)
			checksum += data[i];
		checksum = -checksum;
		dp_info(dp, "EDID: last block checksum = %02x\n", checksum);

		drm_dp_dpcd_writeb(&dp->dp_aux, DP_TEST_EDID_CHECKSUM, checksum);
		dpcd_test_res |= DP_TEST_EDID_CHECKSUM_WRITE;

	} else {
		dp_err(dp, "Automated Test: Unsupported Request: %02x\n", dpcd_test_req);
		dpcd_test_res = DP_TEST_NAK;
	}

	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TEST_RESPONSE, dpcd_test_res);
	return 0;
}

static void dp_update_link_status(struct dp_device *dp, enum link_training_status link_status)
{
	if (link_status != dp->typec_link_training_status) {
		dp->typec_link_training_status = link_status;
		sysfs_notify(&dp->dev->kobj, "drm-displayport", "link_status");
	}
}

static int dp_link_down_event_handler(struct dp_device *dp)
{
	int ret;

	/* Step_1. DP Off */
	dp_off_by_hpd_plug(dp);

	/* Step_2. DP Link Up again */
	ret = dp_link_up(dp);
	if (ret < 0) {
		if (ret == -ENOLINK)
			dp_update_link_status(dp, LINK_TRAINING_FAILURE);
		else
			dp_update_link_status(dp, LINK_TRAINING_FAILURE_SINK);
		dp_err(dp, "failed to DP Link Up during re-negotiation\n");
		return ret;
	}

	/* Step_3. DP On */
	dp_update_link_status(dp, LINK_TRAINING_SUCCESS);
	dp_on_by_hpd_plug(dp);

	return 0;
}

static int dp_downstream_port_event_handler(struct dp_device *dp, int new_sink_count)
{
	int ret;

	if (dp->sink_count == 0 && new_sink_count > 0) {
		/* establish DP link */
		dp->sink_count = new_sink_count;
		ret = dp_link_up(dp);
		if (ret < 0) {
			if (ret == -ENOLINK)
				dp_update_link_status(dp, LINK_TRAINING_FAILURE);
			else
				dp_update_link_status(dp, LINK_TRAINING_FAILURE_SINK);
			dp_err(dp, "failed to DP Link Up during DFP event\n");
			return ret;
		}

		dp_update_link_status(dp, LINK_TRAINING_SUCCESS);
		dp_on_by_hpd_plug(dp);
	} else if (dp->sink_count > 0 && new_sink_count == 0) {
		/* tear down DP link */
		dp->sink_count = new_sink_count;
		dp_off_by_hpd_plug(dp);
		dp_link_down(dp);
	} else {
		dp->sink_count = new_sink_count;
	}

	return 0;
}

/* Works */
static void dp_work_hpd(enum hotplug_state state)
{
	struct dp_device *dp = get_dp_drvdata();
	struct drm_connector *connector = &dp->connector;
	struct drm_device *dev = connector->dev;
	struct exynos_drm_private *private = drm_to_exynos_dev(dev);
	enum link_training_status link_status = LINK_TRAINING_UNKNOWN;
	int ret;
	u8 irq = 0;

	if (dp->restart_pending) {
		dp_debug(dp, "%s: ignored, because of restart_pending", __func__);
		return;
	}

	mutex_lock(&dp->hpd_lock);

	if (state == EXYNOS_HPD_PLUG) {
		dp_info(dp, "[HPD_PLUG start]\n");

		if (mutex_trylock(&private->dp_tui_lock) == 0) {
			/* TUI is active, bail out */
			dp_info(dp, "unable to handle HPD_PLUG, TUI is active\n");
			mutex_unlock(&dp->hpd_lock);
			return;
		}

		/* block suspend and increment power usage count */
		pm_stay_awake(dp->dev);
		pm_runtime_get_sync(dp->dev);
		dp_debug(dp, "pm_rtm_get_sync usage_cnt(%d)\n",
			 atomic_read(&dp->dev->power.usage_count));

		dp_enable_dposc(dp);
		dp->dp_hotplug_error_code = 0;

		/* PHY power on */
		usleep_range(10000, 10030);
		ret = dp_hw_init(&dp->hw_config); /* for AUX ch read/write. */
		if (ret) {
			dp_err(dp, "dp_hw_init() failed\n");
			goto HPD_PLUG_FAIL;
		}
		usleep_range(10000, 11000);

		ret = dp_link_up(dp);
		if (ret < 0) {
			if (ret == -ENOLINK)
				link_status = LINK_TRAINING_FAILURE;
			else
				link_status = LINK_TRAINING_FAILURE_SINK;
			dp_err(dp, "failed to DP Link Up\n");
			goto HPD_PLUG_FAIL;
		}

		if (dp->sink_count > 0) {
			dp_update_link_status(dp, LINK_TRAINING_SUCCESS);
			dp_on_by_hpd_plug(dp);
		}

		/* check for automated test request and schedule HPD_IRQ to handle it */
		if (dp->sink.revision <= DP_DPCD_REV_12)
			drm_dp_dpcd_readb(&dp->dp_aux, DP_DEVICE_SERVICE_IRQ_VECTOR, &irq);
		else
			drm_dp_dpcd_readb(&dp->dp_aux, DP_DEVICE_SERVICE_IRQ_VECTOR_ESI0, &irq);

		if (irq & DP_AUTOMATED_TEST_REQUEST)
			queue_work(dp->dp_wq, &dp->hpd_irq_work);

		dp_info(dp, "[HPD_PLUG done]\n");

	} else if (state == EXYNOS_HPD_UNPLUG) {
		dp_info(dp, "[HPD_UNPLUG start]\n");

		if (!pm_runtime_get_if_in_use(dp->dev)) {
			dp_info(dp, "%s: DP is already powered off\n", __func__);
			mutex_unlock(&dp->hpd_lock);
			return;
		}

		if (dp->sink_count > 0) {
			dp_off_by_hpd_plug(dp);
			dp_link_down(dp);
		}

		/* PHY power off */
		dp_hw_deinit(&dp->hw_config);
		dp_disable_dposc(dp);

		dp->state = DP_STATE_INIT;
		dp_info(dp, "%s: DP State changed to INIT\n", __func__);

		/* decrement power usage count and unblock suspend */
		pm_runtime_put(dp->dev);
		pm_runtime_put_sync(dp->dev);  /* obtained during HPD_PLUG */
		dp_debug(dp, "pm_rtm_put_sync usage_cnt(%d)\n",
			 atomic_read(&dp->dev->power.usage_count));
		pm_relax(dp->dev);

		mutex_unlock(&private->dp_tui_lock);

		dp_info(dp, "[HPD_UNPLUG done]\n");
	}

	mutex_unlock(&dp->hpd_lock);

	return;

HPD_PLUG_FAIL:
	dp_err(dp, "[HPD_PLUG fail] Check CCIC or USB!!\n");
	dp_set_hpd_state(dp, EXYNOS_HPD_UNPLUG);
	hdcp_dplink_connect_state(DP_DISCONNECT);
	dp_hw_deinit(&dp->hw_config);
	dp_disable_dposc(dp);
	dp_init_info(dp);

	/* in error case, add delay to avoid very short interval reconnection */
	msleep(300);

	// Use cached error so LINK_TRAINING_FAILURE doesn't retrigger hpd immediately.
	dp_update_link_status(dp, link_status);

	// TODO: We need to define more error codes, but for now use just 1 for generic error code
	dp->dp_hotplug_error_code = 1;
	dp_info(dp, "[HPD_PLUG fail] call drm_kms_helper_hotplug_event(dp_hotplug_error_code=%d)\n",
		dp->dp_hotplug_error_code);
	drm_kms_helper_hotplug_event(dp->connector.dev);

	/* decrement power usage count and unblock suspend */
	pm_runtime_put_sync(dp->dev);
	dp_debug(dp, "pm_rtm_put_sync usage_cnt(%d)\n",
		 atomic_read(&dp->dev->power.usage_count));
	pm_relax(dp->dev);

	mutex_unlock(&private->dp_tui_lock);
	dp_info(dp, "[HPD_PLUG done]\n");
	mutex_unlock(&dp->hpd_lock);
}

static void dp_work_hpd_plug(struct work_struct *work)
{
	dp_work_hpd(EXYNOS_HPD_PLUG);
}

static void dp_work_hpd_unplug(struct work_struct *work)
{
	dp_work_hpd(EXYNOS_HPD_UNPLUG);
}

static u8 sysfs_triggered_irq = 0;

static void dp_work_hpd_irq(struct work_struct *work)
{
	struct dp_device *dp = get_dp_drvdata();
	u8 sink_count = 0;
	u8 irq = 0, irq2 = 0, irq3 = 0;
	u8 link_status[DP_LINK_STATUS_SIZE];

	if (dp->restart_pending) {
		dp_debug(dp, "%s: ignored, because of restart_pending", __func__);
		return;
	}

	mutex_lock(&dp->hpd_lock);
	dp_info(dp, "[HPD_IRQ start]\n");

	if (!pm_runtime_get_if_in_use(dp->dev)) {
		dp_debug(dp, "[HPD IRQ] IRQ work skipped as power is off\n");
		mutex_unlock(&dp->hpd_lock);
		return;
	}

	if (sysfs_triggered_irq != 0) {
		irq = sysfs_triggered_irq;
		sysfs_triggered_irq = 0;
		goto process_irq;
	}

	if (dp->sink.revision <= DP_DPCD_REV_12) {
		/*
		 * Some DPCD 1.2 sinks/hubs haven't properly implemented the IRQ ESI registers.
		 * Thus, we will force all DPCD 1.2 sinks/hubs to use the legacy IRQ registers.
		 */
		sink_count = drm_dp_read_sink_count(&dp->dp_aux);
		dp_info(dp, "[HPD IRQ] sink count = %u\n", sink_count);

		if (drm_dp_dpcd_readb(&dp->dp_aux, DP_DEVICE_SERVICE_IRQ_VECTOR, &irq) == 1) {
			dp_info(dp, "[HPD IRQ] device irq vector = %02x\n", irq);
			drm_dp_dpcd_writeb(&dp->dp_aux, DP_DEVICE_SERVICE_IRQ_VECTOR, irq);
		} else
			dp_err(dp, "[HPD IRQ] cannot read DP_DEVICE_SERVICE_IRQ_VECTOR\n");

		if (drm_dp_dpcd_read_link_status(&dp->dp_aux, link_status) == DP_LINK_STATUS_SIZE)
			dp_info(dp, "[HPD IRQ] link status = %02x %02x %02x %02x\n",
				link_status[0], link_status[1], link_status[2], link_status[3]);
		else
			dp_err(dp, "[HPD IRQ] cannot read link status\n");
	} else {
		if (drm_dp_dpcd_readb(&dp->dp_aux, DP_SINK_COUNT_ESI, &sink_count) == 1) {
			sink_count = DP_GET_SINK_COUNT(sink_count);
			dp_info(dp, "[HPD IRQ] sink count = %u\n", sink_count);
		} else
			dp_err(dp, "[HPD IRQ] cannot read DP_SINK_COUNT_ESI\n");

		if (drm_dp_dpcd_readb(&dp->dp_aux, DP_DEVICE_SERVICE_IRQ_VECTOR_ESI0, &irq) == 1) {
			dp_info(dp, "[HPD IRQ] device irq vector esi0 = %02x\n", irq);
			drm_dp_dpcd_writeb(&dp->dp_aux, DP_DEVICE_SERVICE_IRQ_VECTOR_ESI0, irq);
		} else
			dp_err(dp, "[HPD IRQ] cannot read DP_DEVICE_SERVICE_IRQ_VECTOR_ESI0\n");

		if (drm_dp_dpcd_readb(&dp->dp_aux, DP_DEVICE_SERVICE_IRQ_VECTOR_ESI1, &irq2) == 1) {
			dp_info(dp, "[HPD IRQ] device irq vector esi1 = %02x\n", irq2);
			drm_dp_dpcd_writeb(&dp->dp_aux, DP_DEVICE_SERVICE_IRQ_VECTOR_ESI1, irq2);
		} else
			dp_err(dp, "[HPD IRQ] cannot read DP_DEVICE_SERVICE_IRQ_VECTOR_ESI1\n");

		if (drm_dp_dpcd_readb(&dp->dp_aux, DP_LINK_SERVICE_IRQ_VECTOR_ESI0, &irq3) == 1) {
			dp_info(dp, "[HPD IRQ] link irq vector esi0 = %02x\n", irq3);
			drm_dp_dpcd_writeb(&dp->dp_aux, DP_LINK_SERVICE_IRQ_VECTOR_ESI0, irq3);
		} else
			dp_err(dp, "[HPD IRQ] cannot read DP_LINK_SERVICE_IRQ_VECTOR_ESI0\n");

		if (drm_dp_dpcd_read(&dp->dp_aux, DP_LANE0_1_STATUS_ESI, link_status,
				     DP_LINK_STATUS_SIZE) == DP_LINK_STATUS_SIZE)
			dp_info(dp, "[HPD IRQ] link status = %02x %02x %02x %02x\n",
				link_status[0], link_status[1], link_status[2], link_status[3]);
		else
			dp_err(dp, "[HPD IRQ] cannot read link status\n");
	}

	if (dp->branch_dev) {
		/* Sanity-check the sink count */
		if (sink_count > dp->dfp_count + 1) {
			dp_err(dp, "[HPD IRQ] invalid sink count, adjusting to 0\n");
			sink_count = 0;
		}

		if ((link_status[2] & DP_DOWNSTREAM_PORT_STATUS_CHANGED) ||
		    (dp->sink_count != sink_count)) {
			dp_info(dp, "[HPD IRQ] DP downstream port status change\n");
			dp_downstream_port_event_handler(dp, sink_count);
			goto release_irq_resource;
		}

		if (sink_count == 0) {
			goto release_irq_resource;
		}
	}

	if (!drm_dp_channel_eq_ok(link_status, dp->link.num_lanes)) {
		dp_info(dp, "[HPD IRQ] DP link is down, re-establish the link\n");
		dp_link_down_event_handler(dp);
		dp->stats.link_unstable_failures++;
		goto release_irq_resource;
	}

process_irq:
	if (irq & DP_CP_IRQ) {
		dp_info(dp, "[HPD IRQ] Content Protection\n");
		hdcp_dplink_handle_irq();
		irq &= ~DP_CP_IRQ;
	}

	if (irq & DP_AUTOMATED_TEST_REQUEST) {
		dp_info(dp, "[HPD IRQ] Automated Test Request\n");
		dp_automated_test_irq_handler(dp);
		irq &= ~DP_AUTOMATED_TEST_REQUEST;
	}

	if (irq)
		dp_info(dp, "[HPD IRQ] unknown IRQ (0x%X)\n", irq);

release_irq_resource:
	pm_runtime_put(dp->dev);
	dp_info(dp, "[HPD_IRQ done]\n");
	mutex_unlock(&dp->hpd_lock);
}

/* Type-C Handshaking Functions */
static void dp_hpd_changed(struct dp_device *dp, enum hotplug_state state)
{
	if (dp_get_hpd_state(dp) == state) {
		dp_debug(dp, "DP HPD is same state (%x): Skip\n", state);
		return;
	}

	if (state == EXYNOS_HPD_PLUG) {
		dp_set_hpd_state(dp, state);
		if (!queue_work(dp->dp_wq, &dp->hpd_plug_work))
			dp_warn(dp, "DP HPD PLUG work was already queued");
	} else if (state == EXYNOS_HPD_UNPLUG) {
		dp_set_hpd_state(dp, state);
		if (!queue_work(dp->dp_wq, &dp->hpd_unplug_work))
			dp_warn(dp, "DP HPD UNPLUG work was already queued");
	} else
		dp_err(dp, "DP HPD changed to abnormal state(%d)\n", state);
}

static bool dp_enabled = false;
module_param(dp_enabled, bool, 0664);
MODULE_PARM_DESC(dp_enabled, "Enable/disable DP notification processing");

static int param_dp_emulation_mode_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%d\n", dp_emulation_mode);
}

static int param_dp_emulation_mode_set(const char *val, const struct kernel_param *kp)
{
	u32 new_value = 0;

	if (kstrtou32(val, 10, &new_value)) {
		dp_warn(dp_drvdata, "%s: dp_emulation_mode parse error\n", __func__);
		return -EINVAL;
	}

	if (new_value != dp_emulation_mode) {
		dp_info(dp_drvdata, "%s: dp_emulation_mode=%d\n", __func__, new_value);
		dp_emulation_mode = new_value;
		if (dp_emulation_mode) {
			memset(&dp_drvdata->hw_config, 0, sizeof(struct dp_hw_config));
			dp_drvdata->hw_config.dp_emul = true;
			dp_drvdata->hw_config.num_lanes = 2;
			dp_drvdata->hw_config.orient_type = PLUG_NORMAL;
			dp_drvdata->hw_config.pin_type = PIN_TYPE_D;
			dp_hpd_changed(dp_drvdata, EXYNOS_HPD_PLUG);
		} else
			dp_hpd_changed(dp_drvdata, EXYNOS_HPD_UNPLUG);
	}
	return 0;
}

static const struct kernel_param_ops param_ops_dp_emulation_mode = {
	.get = param_dp_emulation_mode_get,
	.set = param_dp_emulation_mode_set,
};

module_param_cb(dp_emulation_mode, &param_ops_dp_emulation_mode, NULL, 0664);
MODULE_PARM_DESC(dp_emulation_mode, "Enable DisplayPort emulation mode");

/*
 * Function should be called with typec_notification_lock held.
 */
static int usb_typec_dp_notification_locked(struct dp_device *dp, enum hotplug_state hpd)
{
	if (!dp_enabled) {
		dp_info(dp, "%s: DP is disabled, ignoring DP notifications\n", __func__);
		return NOTIFY_OK;
	}

	if (hpd == EXYNOS_HPD_PLUG) {
		if (dp_get_hpd_state(dp) == EXYNOS_HPD_UNPLUG) {
			dp_info(dp, "%s: USB Type-C is HPD PLUG status\n", __func__);

			memset(&dp->hw_config, 0, sizeof(struct dp_hw_config));
			dp_info(dp, "%s: disp_orientation = %d\n", __func__, dp->typec_orientation);
			dp->hw_config.orient_type = dp->typec_orientation;

			dp_info(dp, "%s: disp_pin_config = %d\n", __func__,
				dp->typec_pin_assignment);
			dp->hw_config.pin_type = dp->typec_pin_assignment;

			dp->hw_config.aux_auto_orientation = dp->aux_auto_orientation;
			dp->hw_config.phy_boost = dp_phy_boost;
			dp_hpd_changed(dp, EXYNOS_HPD_PLUG);
		}
	} else if (hpd == EXYNOS_HPD_IRQ) {
		if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG) {
			dp_info(dp, "%s: Service IRQ from sink\n", __func__);
			if (!queue_work(dp->dp_wq, &dp->hpd_irq_work))
				dp_warn(dp, "DP HPD IRQ work was already queued");
		}
	} else {
		dp_info(dp, "%s: USB Type-C is HPD UNPLUG status, or not in display ALT mode\n",
			__func__);

		if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG)
			dp_hpd_changed(dp, EXYNOS_HPD_UNPLUG);

		/* Mark unknown on HPD UNPLUG */
		dp_update_link_status(dp, LINK_TRAINING_UNKNOWN);
	}

	return NOTIFY_OK;
}

/* Audio(ALSA) Handshaking Functions */
int dp_audio_config(struct dp_audio_config *audio_config)
{
	struct dp_device *dp = get_dp_drvdata();

	if (dp->state == DP_STATE_INIT) {
		dp_warn(dp, "DP Power Status is off\n");
		return -EINVAL;
	}

	dp_info(dp, "audio state (%d ==> %d)\n",
		dp_get_audio_state(dp), audio_config->audio_state);

	if (audio_config->audio_state == dp_get_audio_state(dp))
		return 0;

	dp_set_audio_state(dp, audio_config->audio_state);

	switch (dp_get_audio_state(dp)) {
	case DP_AUDIO_ENABLE:
		dp_hw_init_audio();
		break;
	case DP_AUDIO_START:
		dp_info(dp, "audio_config: ch(%d), fs(%d), bit(%d), packed(%d), word_len(%d)\n",
				audio_config->num_audio_ch, audio_config->audio_fs,
				audio_config->audio_bit, audio_config->audio_packed_mode,
				audio_config->audio_word_length);

		dp->hw_config.num_audio_ch = audio_config->num_audio_ch;
		dp->hw_config.audio_fs = audio_config->audio_fs;
		dp->hw_config.audio_bit = audio_config->audio_bit;
		dp->hw_config.audio_packed_mode = audio_config->audio_packed_mode;
		dp->hw_config.audio_word_length = audio_config->audio_word_length;

		dp_hw_set_audio_config(&dp->hw_config);
		dp_hw_start_audio();
		dp_set_audio_infoframe(dp);
		break;
	case DP_AUDIO_REQ_BUF_READ:
		dp_hw_set_audio_dma(1);
		break;
	case DP_AUDIO_WAIT_BUF_FULL:
		dp_hw_set_audio_dma(0);
		break;
	case DP_AUDIO_STOP:
		dp_hw_stop_audio();
		dp_set_audio_infoframe(dp);
		break;
	case DP_AUDIO_DISABLE:
		dp_hw_deinit_audio();
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dp_audio_config);

/* HDCP Driver Handshaking Functions */
void dp_hdcp_update_cp(u32 drm_cp_status)
{
	struct dp_device *dp = get_dp_drvdata();
	struct drm_connector *connector = &dp->connector;

	dp_info(dp, "dp_hdcp_update_cp to %d\n", drm_cp_status);

	drm_modeset_lock(&connector->dev->mode_config.connection_mutex, NULL);
	drm_hdcp_update_content_protection(connector, drm_cp_status);
	drm_modeset_unlock(&connector->dev->mode_config.connection_mutex);
}
EXPORT_SYMBOL_GPL(dp_hdcp_update_cp);

int dp_dpcd_read_for_hdcp22(u32 address, u32 length, u8 *data)
{
	struct dp_device *dp = get_dp_drvdata();
	int ret = -EFAULT;

	if (pm_runtime_get_if_in_use(dp->dev)) {
		ret = drm_dp_dpcd_read(&dp->dp_aux, address, data, length);
		pm_runtime_put(dp->dev);
	}

	if (ret == length)
		return 0;

	dp_err(dp, "dpcd_read_for_hdcp22 fail(%d): 0x%X\n", ret, address);
	return (ret < 0) ? ret : -EIO;
}
EXPORT_SYMBOL_GPL(dp_dpcd_read_for_hdcp22);

int dp_dpcd_write_for_hdcp22(u32 address, u32 length, u8 *data)
{
	struct dp_device *dp = get_dp_drvdata();
	int ret = -EFAULT;

	if (pm_runtime_get_if_in_use(dp->dev)) {
		ret = drm_dp_dpcd_write(&dp->dp_aux, address, data, length);
		pm_runtime_put(dp->dev);
	}

	if (ret == length)
		return 0;

	dp_err(dp, "dpcd_write_for_hdcp22 fail(%d): 0x%X\n", ret, address);
	return (ret < 0) ? ret : -EIO;
}
EXPORT_SYMBOL_GPL(dp_dpcd_write_for_hdcp22);

/* DP DRM Connector Helper Functions */
static enum drm_mode_status dp_mode_valid(struct drm_encoder *encoder,
					  const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void dp_atomic_mode_set(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{
	struct dp_device *dp = encoder_to_dp(encoder);
	struct drm_display_mode *adjusted_mode = &crtc_state->adjusted_mode;

	dp_info(dp, "Mode(" DRM_MODE_FMT ") is requested\n",
		DRM_MODE_ARG(adjusted_mode));

	if (!drm_mode_equal(&dp->cur_mode, adjusted_mode)) {
		drm_mode_copy(&dp->cur_mode, adjusted_mode);
	}
}

static int dp_atomic_check(struct drm_encoder *encoder,
			   struct drm_crtc_state *crtc_state,
			   struct drm_connector_state *state)
{
	return 0;
}

static const struct drm_encoder_helper_funcs dp_encoder_helper_funcs = {
	.mode_valid = dp_mode_valid,
	.atomic_mode_set = dp_atomic_mode_set,
	.enable = dp_enable,
	.disable = dp_disable,
	.atomic_check = dp_atomic_check,
};

/* DP DRM Encoder Functions */
static const struct drm_encoder_funcs dp_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static void dp_connector_reset(struct drm_connector *connector)
{
	/* Need to preserve max BPC settings over reset */
	u8 max_bpc = connector->state->max_bpc;
	u8 max_req_bpc = connector->state->max_requested_bpc;

	drm_atomic_helper_connector_reset(connector);

	connector->state->max_bpc = max_bpc;
	connector->state->max_requested_bpc = max_req_bpc;
}

/* DP DRM Connector Functions */
static ssize_t dp_crc_enabled_write(struct file *file, const char __user *buffer, size_t len,
				    loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct dp_device *dp = s->private;
	int res;
	int enable = 0;

	res = kstrtoint_from_user(buffer, len, 0, &enable);
	if (res) {
		dp_warn(dp, "%s: invalid value %s\n", __func__, buffer);
		return len;
	}

	res = dp_crc_set_enabled(SST1, enable);
	if (res == 0) {
		mutex_lock(&dp->cmd_lock);
		dp->dp_link_crc_enabled = enable ? true : false;
		mutex_unlock(&dp->cmd_lock);
		dp_debug(dp, "%s: dp_crc_set_enabled(SST1, %d) successfully\n", __func__, enable);
	} else
		dp_warn(dp, "%s: dp_crc_set_enabled(SST1, %d) failed res=%d\n", __func__, enable,
			res);

	return len;
}

static int dp_crc_enabled_show(struct seq_file *s, void *unused)
{
	struct dp_device *dp = s->private;

	mutex_lock(&dp->cmd_lock);
	seq_printf(s, "%d\n", dp->dp_link_crc_enabled);
	mutex_unlock(&dp->cmd_lock);
	return 0;
}

static int dp_crc_enabled_open(struct inode *inode, struct file *file)
{
	return single_open(file, dp_crc_enabled_show, inode->i_private);
}

static const struct file_operations dp_crc_enabled_fops = {
	.owner = THIS_MODULE,
	.open = dp_crc_enabled_open,
	.read = seq_read,
	.write = dp_crc_enabled_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dp_crc_values_show(struct seq_file *s, void *unused)
{
	struct dp_device *dp = s->private;
	bool crc_enabled = false;
	int res = 0;
	u32 crc_data[3];

	mutex_lock(&dp->cmd_lock);
	crc_enabled = dp->dp_link_crc_enabled;
	mutex_unlock(&dp->cmd_lock);
	if (!crc_enabled) {
		seq_puts(s, "CRCs are disabled\n");
		return 0;
	}

	res = dp_crc_get(SST1, crc_data);
	if (res != 0) {
		seq_printf(s, "dp_crc_get failed, res=%d\n", res);
		dp_warn(dp, "%s: dp_crc_get failed, res=%d\n", __func__, res);
		return 0;
	}

	dp_debug(dp, "%s: Got CRCs R:%04X G:%04X B:%04X\n", __func__, crc_data[0], crc_data[1],
		 crc_data[2]);
	seq_printf(s, "R:%04X G:%04X B:%04X\n", crc_data[0], crc_data[1], crc_data[2]);
	return 0;
}

static int dp_crc_values_open(struct inode *inode, struct file *file)
{
	return single_open(file, dp_crc_values_show, inode->i_private);
}

static ssize_t dp_crc_values_write(struct file *file, const char __user *buffer, size_t len,
				   loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct dp_device *dp = s->private;
	int res = 0;

	res = dp_crc_reset(SST1);
	if (res)
		dp_warn(dp, "%s: dp_crc_reset failed, res=%d\n", __func__, res);
	else
		dp_debug(dp, "%s: dp_crc_reset finished successfully\n", __func__);

	return len;
}

static const struct file_operations dp_crc_values_fops = {
	.owner = THIS_MODULE,
	.open = dp_crc_values_open,
	.read = seq_read,
	.write = dp_crc_values_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int drm_dp_late_register(struct drm_connector *connector)
{
	struct dentry *root = connector->debugfs_entry;

	dp_drvdata->dp_crc_enabled_debugfs_file =
		debugfs_create_file("dp_crc_enabled", 0644, root, dp_drvdata, &dp_crc_enabled_fops);
	dp_drvdata->dp_crc_values_debugfs_file =
		debugfs_create_file("dp_crc_values", 0644, root, dp_drvdata, &dp_crc_values_fops);
	return 0;
}

static void drm_dp_early_unregister(struct drm_connector *connector)
{
	debugfs_remove(dp_drvdata->dp_crc_enabled_debugfs_file);
	debugfs_remove(dp_drvdata->dp_crc_values_debugfs_file);
}

static const struct drm_connector_funcs dp_connector_funcs = {
	.reset = dp_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.late_register = drm_dp_late_register,
	.early_unregister = drm_dp_early_unregister,
};

/* DP DRM Connector Helper Functions */
static int dp_detect(struct drm_connector *connector,
		     struct drm_modeset_acquire_ctx *ctx, bool force)
{
	struct dp_device *dp = connector_to_dp(connector);

	dp_info(dp, "%s: Connector Status = %d\n", __func__, connector->status);

	return (int)connector->status;
}

static int dp_get_modes(struct drm_connector *connector)
{
	struct dp_device *dp = connector_to_dp(connector);

	if (dp->state == DP_STATE_INIT) {
		dp_warn(dp, "%s: DP is not ON\n", __func__);
		return 0;
	}

	dp_info(dp, "dp->num_modes = %d\n", dp->num_modes);
	return dp->num_modes;
}

static enum drm_mode_status dp_conn_mode_valid(struct drm_connector *connector, struct drm_display_mode *mode)
{
	struct dp_device *dp = connector_to_dp(connector);
	struct drm_display_info *display_info = &connector->display_info;

	/*
	 * drm_atomic_connector_check() hasn't been called yet.
	 * We can't use connector->state->max_bpc directly.
	 * Need to do the same math here.
	 */
	u8 dbpc = display_info->bpc ? display_info->bpc : 8;
	u8 bpc = min(dbpc, connector->state->max_requested_bpc);

	/*
	 * DP link max data rate in Kbps
	 *
	 * dp->link.link_rate unit is weird: Mbps * 100
	 * The calculation becomes:
	 * (dp->link.link_rate * 10) * dp->link.num_lanes * (8 / 10)
	 */
	u32 link_data_rate = dp->link.link_rate * dp->link.num_lanes * 8;

	/* DRM display mode data rate in Kbps */
	u32 mode_data_rate = mode->clock * 3 * bpc;

	if (link_data_rate < mode_data_rate) {
		dp_info(dp, "DROP: " DRM_MODE_FMT "\n", DRM_MODE_ARG(mode));
		return MODE_CLOCK_HIGH;
	}

	dp_info(dp, "PICK: " DRM_MODE_FMT "\n", DRM_MODE_ARG(mode));
	return MODE_OK;
}

static int dp_conn_atomic_check(struct drm_connector *c, struct drm_atomic_state *state)
{
	struct dp_device *dp = connector_to_dp(c);

	dp_debug(dp, "%s: c->status=%d dp->state=%d c->dpms=%d dp->hdcp_and_audio_enabled=%d\n",
		 __func__, c->status, dp->state, c->dpms, dp->hdcp_and_audio_enabled);

	if (c->status == connector_status_connected && dp->state == DP_STATE_RUN &&
	    c->dpms == DRM_MODE_DPMS_ON && !dp->hdcp_and_audio_enabled) {
		/*
		 * Connector has transitioned to DRM_MODE_DPMS_ON.
		 * This means DP connection usage has been confirmed.
		 * Enable HDCP and DP audio.
		 */
		hdcp_dplink_connect_state(DP_CONNECT);

		if (dp->sink.has_pcm_audio) {
			dp_info(dp, "call DP audio notifier (connected)\n");
			blocking_notifier_call_chain(&dp_ado_notifier_head, 1UL, NULL);
		}

		dp->hdcp_and_audio_enabled = true;
	}

	return 0;
}

static const struct drm_connector_helper_funcs dp_connector_helper_funcs = {
	.detect_ctx = dp_detect,
	.get_modes = dp_get_modes,
	.mode_valid = dp_conn_mode_valid,
	.atomic_check = dp_conn_atomic_check,
};

/* DP DRM Component Functions */
static int dp_create_connector(struct drm_encoder *encoder)
{
	struct dp_device *dp = encoder_to_dp(encoder);
	struct drm_connector *connector = &dp->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(encoder->dev, connector, &dp_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		dp_err(dp, "failed to initialize connector with drm\n");
		return ret;
	}

	connector->status = connector_status_disconnected;
	connector->dpms = DRM_MODE_DPMS_OFF;
	drm_connector_helper_add(connector, &dp_connector_helper_funcs);
	drm_connector_register(connector);
	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static int dp_create_dp_aux(struct dp_device *dp)
{
	drm_dp_aux_init(&dp->dp_aux);
	dp->dp_aux.dev = dp->dev;
	dp->dp_aux.transfer = dp_aux_transfer;

	return 0;
}

static int dp_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dp_device *dp = encoder_to_dp(encoder);
	struct drm_device *drm_dev = data;
	int ret = 0;

	drm_encoder_init(drm_dev, encoder, &dp_encoder_funcs,
			 DRM_MODE_ENCODER_LVDS, NULL);
	drm_encoder_helper_add(encoder, &dp_encoder_helper_funcs);

	encoder->possible_crtcs =
		exynos_drm_get_possible_crtcs(encoder, dp->output_type);
	if (!encoder->possible_crtcs) {
		dp_err(dp, "failed to get possible crtc, ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return -ENOTSUPP;
	}

	ret = dp_create_connector(encoder);
	if (ret) {
		dp_err(dp, "failed to create connector ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	ret = dp_create_dp_aux(dp);
	if (ret) {
		dp_err(dp, "failed to create dp_aux ret = %d\n", ret);
		return ret;
	}

	dp_fill_host_caps(dp);

	drm_atomic_helper_connector_reset(&dp->connector);
	drm_connector_attach_max_bpc_property(&dp->connector, 6, dp->host.max_bpc);
	drm_connector_attach_content_protection_property(&dp->connector,
		DRM_MODE_HDCP_CONTENT_TYPE0);

	ret = sysfs_create_link(&encoder->dev->dev->kobj, &dp->dev->kobj, "displayport");
	if (ret)
		dp_err(dp, "unable to link displayport sysfs (%d)\n", ret);

	dp_info(dp, "DP Driver has been binded\n");

	return ret;
}

static void dp_unbind(struct device *dev, struct device *master, void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dp_device *dp = encoder_to_dp(encoder);

	dp_debug(dp, "%s +\n", __func__);
	sysfs_remove_link(&encoder->dev->dev->kobj, "displayport");
	dp_debug(dp, "%s -\n", __func__);
}

static const struct component_ops dp_component_ops = {
	.bind = dp_bind,
	.unbind = dp_unbind,
};

/* DP DRM Driver */
static int dp_parse_dt(struct dp_device *dp, struct device *dev)
{
	if (IS_ERR_OR_NULL(dev->of_node)) {
		dp_err(dp, "no device tree information\n");
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t dp_irq_handler(int irq, void *dev_data)
{
	struct dp_device *dp = dev_data;
	bool common = false, str = false, pcs = false;
	unsigned int intr_val = 0;

	dp_hw_get_intr_source(&common, &str, &pcs);

	if (common) {
		intr_val = dp_hw_get_and_clear_common_intr();
		dp_debug(dp, "Common Intr = 0x%08X\n", intr_val);
	}

	if (str) {
		intr_val = dp_hw_get_and_clear_video_intr();
		dp_debug(dp, "Video Intr = 0x%08X\n", intr_val);
	}

	if (pcs) {
		dp_debug(dp, "triggered intr from PCS block\n");
	}

	return IRQ_HANDLED;
}

static int dp_remap_regs(struct dp_device *dp, struct platform_device *pdev)
{
	struct resource res;
	struct device *dev = dp->dev;
	struct device_node *np = dev->of_node;
	int i, ret = 0;

	/* DP Link SFR */
	i = of_property_match_string(np, "reg-names", "link");
	if (of_address_to_resource(np, i, &res)) {
		dp_err(dp, "failed to get DP Link resource\n");
		goto err;
	}

	dp->res.link_regs = devm_platform_ioremap_resource_byname(pdev, "link");
	if (IS_ERR(dp->res.link_regs)) {
		dp_err(dp, "failed to remap DP LINK SFR region\n");
		ret = PTR_ERR(dp->res.link_regs);
		goto err;
	}
	dp_regs_desc_init(dp->res.link_regs, res.start, "LINK", REGS_LINK, SST1);

	/* USBDP Combo PHY SFR */
	/*
	 * PHY HW is a Combo PHY for USB and DP.
	 * USB is master IP for this PHY and controlled the life cycle of it.
	 * To avoid abnormal clean-up the resource, it doesn't use managed resource.
	 */
	i = of_property_match_string(np, "reg-names", "phy");
	if (of_address_to_resource(np, i, &res)) {
		dp_err(dp, "failed to get USB/DP Combo PHY resource\n");
		goto err;
	}

	dp->res.phy_regs = ioremap(res.start, resource_size(&res));
	if (!dp->res.phy_regs) {
		dp_err(dp, "failed to remap USB/DP Combo PHY SFR region\n");
		ret = -ENOMEM;
		goto err;
	}
	dp_regs_desc_init(dp->res.phy_regs, res.start, "PHY", REGS_PHY, SST1);

	if (dp_remap_regs_other(dp))
		goto err_phy;

	return 0;

err_phy:
	if (dp->res.phy_regs)
		iounmap(dp->res.phy_regs);
err:
	return ret;
}

static int dp_register_irq(struct dp_device *dp, struct platform_device *pdev)
{
	int irq;
	int ret = 0;

	irq = platform_get_irq_byname(pdev, "dp_link");
	if (irq <= 0) {
		dp_err(dp, "failed to get irq resource (%d)\n", irq);
		return -ENOENT;
	}

	dp->res.irq = irq;
	ret = devm_request_irq(dp->dev, irq, dp_irq_handler, 0,
			       pdev->name, dp);
	if (ret) {
		dp_err(dp, "failed to install DP irq\n");
		return -EINVAL;
	}
	disable_irq(dp->res.irq);

	return 0;
}

static int dp_init_resources(struct dp_device *dp, struct platform_device *pdev)
{
	int ret = 0;

	ret = dp_remap_regs(dp, pdev);
	if (ret) {
		dp_err(dp, "failed to remap DP registers\n");
		return -EINVAL;
	}

	ret = dp_get_clock(dp);
	if (ret) {
		dp_err(dp, "failed to get DP clks\n");
		return -EINVAL;
	}

	ret = dp_register_irq(dp, pdev);
	if (ret) {
		dp_err(dp, "failed to get DP interrupts\n");
		return -EINVAL;
	}

	return 0;
}

static const char *const orientations[] = {
	[PLUG_NONE] = "unknown",
	[PLUG_NORMAL] = "normal",
	[PLUG_FLIPPED] = "reverse",
};

static ssize_t orientation_store(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t size)
{
	int orientation;
	struct dp_device *dp = dev_get_drvdata(dev);

	orientation = sysfs_match_string(orientations, buf);
	if (orientation < 0)
		return orientation;

	dp->typec_orientation = (enum plug_orientation)orientation;
	return size;
}

static ssize_t orientation_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->hw_config.orient_type);
}
static DEVICE_ATTR_RW(orientation);

static const char *const pin_assignments[] = {
	// Two blanks required because PIN_TYPE_A starts at 2, and
	// sysfs_match_string requires all indices to be filled.
	[0] = "NA",	    [1] = "NA",		[PIN_TYPE_A] = "A", [PIN_TYPE_B] = "B",
	[PIN_TYPE_C] = "C", [PIN_TYPE_D] = "D", [PIN_TYPE_E] = "E", [PIN_TYPE_F] = "F",
};

static ssize_t pin_assignment_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int pin_assign;
	struct dp_device *dp = dev_get_drvdata(dev);

	pin_assign = sysfs_match_string(pin_assignments, buf);
	if (pin_assign < (int)PIN_TYPE_A)
		return -EINVAL;

	dp->typec_pin_assignment = (enum pin_assignment)pin_assign;
	return size;
}

static ssize_t pin_assignment_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->hw_config.pin_type);
}
static DEVICE_ATTR_RW(pin_assignment);

static const char *const hpds[] = {
	[EXYNOS_HPD_UNPLUG] = "0",
	[EXYNOS_HPD_PLUG] = "1",
};

static ssize_t hpd_store(struct device *dev, struct device_attribute *attr, const char *buf,
			 size_t size)
{
	int hpd;
	struct dp_device *dp = dev_get_drvdata(dev);

	hpd = sysfs_match_string(hpds, buf);
	if (hpd < 0)
		return hpd;

	mutex_lock(&dp->typec_notification_lock);
	usb_typec_dp_notification_locked(dp, (enum hotplug_state)hpd);
	mutex_unlock(&dp->typec_notification_lock);
	return size;
}

static ssize_t hpd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp_get_hpd_state(dp));
}
static DEVICE_ATTR_RW(hpd);

static ssize_t dp_hotplug_error_code_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->dp_hotplug_error_code);
}

static ssize_t dp_hotplug_error_code_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct dp_device *dp = get_dp_drvdata();
	int new_value = 0;

	if (kstrtoint(buf, 0, &new_value) < 0) {
		dp_warn(dp, "%s: parse error, buf=%s\n", __func__, buf);
		return -EINVAL;
	}
	dp->dp_hotplug_error_code = new_value;
	return size;
}

static DEVICE_ATTR_RW(dp_hotplug_error_code);

static ssize_t link_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->typec_link_training_status);
}
static DEVICE_ATTR_RO(link_status);

static ssize_t irq_hpd_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t size)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	mutex_lock(&dp->typec_notification_lock);
	usb_typec_dp_notification_locked(dp, EXYNOS_HPD_IRQ);
	mutex_unlock(&dp->typec_notification_lock);
	return size;
}
static DEVICE_ATTR_WO(irq_hpd);

static ssize_t usbc_cable_disconnect_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	hdcp_dplink_connect_state(DP_PHYSICAL_DISCONNECT);

	return size;
}
static DEVICE_ATTR_WO(usbc_cable_disconnect);

static struct attribute *dp_attrs[] = { &dev_attr_orientation.attr,
					&dev_attr_pin_assignment.attr,
					&dev_attr_hpd.attr,
					&dev_attr_dp_hotplug_error_code.attr,
					&dev_attr_link_status.attr,
					&dev_attr_irq_hpd.attr,
					&dev_attr_usbc_cable_disconnect.attr,
					NULL };

static const struct attribute_group dp_group = {
	.name = "drm-displayport",
	.attrs = dp_attrs,
};

static ssize_t link_negotiation_failures_show(struct device *dev, struct device_attribute *attr,
					      char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.link_negotiation_failures);
}
static DEVICE_ATTR_RO(link_negotiation_failures);

static ssize_t edid_read_failures_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.edid_read_failures);
}
static DEVICE_ATTR_RO(edid_read_failures);

static ssize_t dpcd_read_failures_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.dpcd_read_failures);
}
static DEVICE_ATTR_RO(dpcd_read_failures);

static ssize_t edid_invalid_failures_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.edid_invalid_failures);
}
static DEVICE_ATTR_RO(edid_invalid_failures);

static ssize_t sink_count_invalid_failures_show(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.sink_count_invalid_failures);
}
static DEVICE_ATTR_RO(sink_count_invalid_failures);

static ssize_t link_unstable_failures_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.link_unstable_failures);
}
static DEVICE_ATTR_RO(link_unstable_failures);

/* Resolution Sysfs */
static ssize_t max_res_1366_768_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_1366_768);
}
static DEVICE_ATTR_RO(max_res_1366_768);

static ssize_t max_res_1440_900_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_1440_900);
}
static DEVICE_ATTR_RO(max_res_1440_900);

static ssize_t max_res_1600_900_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_1600_900);
}
static DEVICE_ATTR_RO(max_res_1600_900);

static ssize_t max_res_1920_1080_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_1920_1080);
}
static DEVICE_ATTR_RO(max_res_1920_1080);

static ssize_t max_res_2560_1080_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_2560_1080);
}
static DEVICE_ATTR_RO(max_res_2560_1080);

static ssize_t max_res_2560_1440_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_2560_1440);
}
static DEVICE_ATTR_RO(max_res_2560_1440);

static ssize_t max_res_3440_1440_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_3440_1440);
}
static DEVICE_ATTR_RO(max_res_3440_1440);

static ssize_t max_res_3840_2160_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_3840_2160);
}
static DEVICE_ATTR_RO(max_res_3840_2160);

static ssize_t max_res_5120_2880_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_5120_2880);
}
static DEVICE_ATTR_RO(max_res_5120_2880);

static ssize_t max_res_7680_4320_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_7680_4320);
}
static DEVICE_ATTR_RO(max_res_7680_4320);

static ssize_t max_res_other_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.max_res_other);
}
static DEVICE_ATTR_RO(max_res_other);

/* FEC/DSC Support Sysfs */
static ssize_t fec_dsc_supported_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.fec_dsc_supported);
}
static DEVICE_ATTR_RO(fec_dsc_supported);

static ssize_t fec_dsc_not_supported_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct dp_device *dp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", dp->stats.fec_dsc_not_supported);
}
static DEVICE_ATTR_RO(fec_dsc_not_supported);

static struct attribute *dp_stats_attrs[] = { &dev_attr_link_negotiation_failures.attr,
					      &dev_attr_edid_read_failures.attr,
					      &dev_attr_dpcd_read_failures.attr,
					      &dev_attr_edid_invalid_failures.attr,
					      &dev_attr_sink_count_invalid_failures.attr,
					      &dev_attr_link_unstable_failures.attr,
					      &dev_attr_max_res_1366_768.attr,
					      &dev_attr_max_res_1440_900.attr,
					      &dev_attr_max_res_1600_900.attr,
					      &dev_attr_max_res_1920_1080.attr,
					      &dev_attr_max_res_2560_1080.attr,
					      &dev_attr_max_res_2560_1440.attr,
					      &dev_attr_max_res_3440_1440.attr,
					      &dev_attr_max_res_3840_2160.attr,
					      &dev_attr_max_res_5120_2880.attr,
					      &dev_attr_max_res_7680_4320.attr,
					      &dev_attr_max_res_other.attr,
					      &dev_attr_fec_dsc_supported.attr,
					      &dev_attr_fec_dsc_not_supported.attr,
					      NULL };

static const struct attribute_group dp_stats_group = {
	.name = "drm-displayport-stats",
	.attrs = dp_stats_attrs,
};

static int dp_reboot_handler(struct notifier_block *nb, unsigned long state, void *data)
{
	struct dp_device *dp = get_dp_drvdata();

	hdcp_dplink_connect_state(DP_SHUTDOWN);
	dp_disable(&dp->encoder);
	dp->restart_pending = true;

	return NOTIFY_DONE;
}

static struct notifier_block dp_reboot_block = {
	.notifier_call = dp_reboot_handler,
};

static int dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dp_device *dp = NULL;
	int ret = 0;

	ret = sysfs_create_group(&dev->kobj, &dp_group);
	if (ret) {
		dev_err(dev, "failed to allocate dp attributes\n");
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &dp_stats_group);
	if (ret) {
		dev_err(dev, "failed to allocate dp stats attributes\n");
		return ret;
	}

	dp = devm_kzalloc(dev, sizeof(struct dp_device), GFP_KERNEL);
	if (!dp) {
		dev_err(dev, "failed to allocate dp device.\n");
		return -ENOMEM;
	}
	dp->dev = dev;

	ret = dp_parse_dt(dp, dev);
	if (ret) {
		dp_err(dp, "failed to parse dp device tree.\n");
		return ret;
	}

	ret = dp_init_resources(dp, pdev);
	if (ret) {
		dp_err(dp, "failed to init dp resources.\n");
		return ret;
	}

	ret = of_property_match_string(dp->dev->of_node, "aux-orientation", "auto");
	if (ret >= 0)
		dp->aux_auto_orientation = true;

	platform_set_drvdata(pdev, dp);

	/* Driver Initialization */
	dp_drvdata = dp;
	dp_init_info(dp);

	dma_set_mask(dev, DMA_BIT_MASK(32));

	mutex_init(&dp->cmd_lock);
	mutex_init(&dp->hpd_lock);
	mutex_init(&dp->hpd_state_lock);
	mutex_init(&dp->audio_lock);
	mutex_init(&dp->training_lock);
	mutex_init(&dp->typec_notification_lock);

	/* Create WorkQueue & Works for HPD */
	dp->dp_wq = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!dp->dp_wq) {
		dp_err(dp, "create wq failed.\n");
		ret = -ENOMEM;
		goto err;
	}

	INIT_WORK(&dp->hpd_plug_work, dp_work_hpd_plug);
	INIT_WORK(&dp->hpd_unplug_work, dp_work_hpd_unplug);
	INIT_WORK(&dp->hpd_irq_work, dp_work_hpd_irq);

	device_init_wakeup(dev, true);
	pm_runtime_enable(dev);
	register_reboot_notifier(&dp_reboot_block);

	/* Register callback to HDCP */
	dp_register_func_for_hdcp22(dp_hdcp_update_cp, dp_dpcd_read_for_hdcp22,
		dp_dpcd_write_for_hdcp22);

	dp_info(dp, "DP Driver has been probed.\n");
	return component_add(dp->dev, &dp_component_ops);

err:
	sysfs_remove_group(&dev->kobj, &dp_group);
	sysfs_remove_group(&dev->kobj, &dp_stats_group);
	return ret;
}

static int dp_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dp_device *dp = platform_get_drvdata(pdev);

	unregister_reboot_notifier(&dp_reboot_block);
	pm_runtime_disable(&pdev->dev);
	device_init_wakeup(dev, false);

	mutex_destroy(&dp->cmd_lock);
	mutex_destroy(&dp->hpd_lock);
	mutex_destroy(&dp->hpd_state_lock);
	mutex_destroy(&dp->training_lock);
	mutex_destroy(&dp->typec_notification_lock);

	sysfs_remove_group(&dev->kobj, &dp_group);
	sysfs_remove_group(&dev->kobj, &dp_stats_group);

	destroy_workqueue(dp->dp_wq);

	dp_info(dp, "DP Driver has been removed\n");
	return 0;
}

static const struct of_device_id dp_of_match[] = {
	{ .compatible = "samsung,exynos-dp" },
	{},
};
MODULE_DEVICE_TABLE(of, dp_of_match);

struct platform_driver dp_driver
	__refdata = { .probe = dp_probe,
		      .remove = dp_remove,
		      .driver = {
			      .name = "exynos-drmdp",
			      .owner = THIS_MODULE,
			      .of_match_table = of_match_ptr(dp_of_match),
		      } };

MODULE_AUTHOR("YongWook Shin <yongwook.shin@samsung.com>");
MODULE_AUTHOR("Petri Gynther <pgynther@google.com>");
MODULE_DESCRIPTION("Samsung SoC DisplayPort driver");
MODULE_LICENSE("GPL");
