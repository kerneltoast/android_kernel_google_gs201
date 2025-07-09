/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for DisplayPort CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DP_CAL_H__
#define __SAMSUNG_DP_CAL_H__

#include <linux/usb/typec_dp.h>
#include <linux/usb/dwc3-exynos.h>

/* Register definition */
enum sst_id {
	SST1 = 0,
	SST2,
	SST_MAX
};

enum dp_regs_type {
	REGS_LINK = 0,
	REGS_PHY,
#ifdef CONFIG_SOC_ZUMA
	REGS_PHY_TCA,
#endif
	REGS_DP_TYPE_MAX,
};

/* DFP_D (USB Type-C) */
// It should be matched with enum in \include\linux\usb\typec_dp.h
enum pin_assignment {
	PIN_TYPE_A = TYPEC_DP_STATE_A,	/* Not supported after Alt Mode Spec v1.0b */
	PIN_TYPE_B = TYPEC_DP_STATE_B,	/* Not supported after Alt Mode Spec v1.0b */
	PIN_TYPE_C = TYPEC_DP_STATE_C,
	PIN_TYPE_D = TYPEC_DP_STATE_D,
	PIN_TYPE_E = TYPEC_DP_STATE_E,
	PIN_TYPE_F = TYPEC_DP_STATE_F,	/* Not supported after Alt Mode Spec v1.0b */
};

// It should be matched with enum typec_orientation in \include\linux\usb\typec.h
enum plug_orientation {
	PLUG_NONE = 0,		// TYPEC_ORIENTATION_NONE
	PLUG_NORMAL,		// TYPEC_ORIENTATION_NORMAL
	PLUG_FLIPPED,		// TYPEC_ORIENTATION_REVERSE
};

/* AUX Ch Definitions for DPCD/EDID */
#define DPCD_BUF_SIZE 12

#define AUX_DATA_BUF_COUNT 16
#define AUX_RETRY_COUNT 3

#define MAX_EDID_BLOCK 4
#define EDID_BLOCK_SIZE 128
#define EDID_ADDRESS 0x50
#define DDC_SEGMENT_ADDR 0x30

enum dp_aux_ch_command_type {
	I2C_READ_NO_MOT = 0x1,
	I2C_WRITE = 0x4,
	I2C_READ = 0x5,
	DPCD_WRITE = 0x8,
	DPCD_READ = 0x9,
};

enum dp_aux_ch_cmd_status {
	AUX_CMD_STATUS_OK			= 0,
	AUX_CMD_STATUS_NACK_ERROR,
	AUX_CMD_STATUS_TIMEOUT_ERROR,
	AUX_CMD_STATUS_UNKNOWN_ERROR,
	AUX_CMD_STATUS_MUCH_DEFER_ERROR,
	AUX_CMD_STATUS_TX_SHORT_ERROR,
	AUX_CMD_STATUS_RX_SHORT_ERROR,
	AUX_CMD_STATUS_NACK_WITHOUT_M_ERROR,
	AUX_CMD_STATUS_I2C_NACK_ERROR,
};


/* Display HW enum definition */
// Link Config
#define MAX_LANE_CNT 4

enum dp_link_rate_type {
	LINK_RATE_RBR	= 0,	// DP_LINK_BW_1_62
	LINK_RATE_HBR,		// DP_LINK_BW_2_7
	LINK_RATE_HBR2,		// DP_LINK_BW_5_4
	LINK_RATE_HBR3,		// DP_LINK_BW_8_1
};

typedef enum {
	NORMAL_DATA = 0,
	TRAINING_PATTERN_1 = 1,
	TRAINING_PATTERN_2 = 2,
	TRAINING_PATTERN_3 = 3,
	TRAINING_PATTERN_4 = 5,
} dp_training_pattern;

typedef enum {
	DISABLE_PATTERN	= 0,
	D10_2_PATTERN	= 1,	// D10.2 Test Pattern
	SERP_PATTERN	= 2,	// Symbol Error Rate measurement Pattern
	PRBS7		= 3,	// PRBS 7bit Pattern
	CUSTOM_80BIT	= 4,	// Custom 80bit (PLT) Test Pattern
	HBR2_COMPLIANCE	= 5,	// HBR2 Compliance Test Pattern CP2520
} dp_qual_pattern;

typedef enum {
	ENABLE_SCRAM	= 0,
	DISABLE_SCRAM	= 1,
} dp_scrambling;


// Interrupts
enum dp_interrupt_mask {
	PLL_LOCK_CHG_INT_MASK,
	HOTPLUG_CHG_INT_MASK,
	HPD_LOST_INT_MASK,
	PLUG_INT_MASK,
	HPD_IRQ_INT_MASK,
	RPLY_RECEIV_INT_MASK,
	AUX_ERR_INT_MASK,
	HDCP_LINK_CHECK_INT_MASK,
	HDCP_LINK_FAIL_INT_MASK,
	HDCP_R0_READY_INT_MASK,
	VIDEO_FIFO_UNDER_FLOW_MASK,
	VSYNC_DET_INT_MASK,
	AUDIO_FIFO_UNDER_RUN_INT_MASK,
	AUDIO_FIFO_OVER_RUN_INT_MASK,

	ALL_INT_MASK
};

// HDCP Config
enum hdcp_mode {
	HDCP_MODE_NONE = 0,
	HDCP_MODE_1_3,
	HDCP_MODE_2_2,
};

// Video Config
enum video_mode {
	VIDEO_MODE_MASTER = 0,
	VIDEO_MODE_SLAVE,
};

enum dp_dynamic_range_type {
	VESA_RANGE = 0,   /* (0 ~ 255) */
	CEA_RANGE = 1,    /* (16 ~ 235) */
};

enum bit_depth {
	BPC_6 = 0,
	BPC_8,
	BPC_10,
};

enum color_format {
	COLOR_RGB = 0,
};

enum sync_polarity {
	SYNC_POSITIVE = 0,
	SYNC_NEGATIVE,
};

struct video_timing {
	u32 clock;
	u32 htotal;
	u32 vtotal;
	u32 hfp;
	u32 hbp;
	u32 hactive;
	u32 vfp;
	u32 vbp;
	u32 vactive;
	enum sync_polarity vsync_polarity;
	enum sync_polarity hsync_polarity;
};

// Audio Config
enum audio_mode {
	AUDIO_MODE_ASYNC = 0,	// Asynchronous Clock Mode
	AUDIO_MODE_SYNC,	// Synchronous Clock Mode
};

enum audio_sampling_frequency {
	FS_32KHZ = 0,	// 32 KHz
	FS_44KHZ,	// 44.1 KHz
	FS_48KHZ,	// 48 Khz
	FS_88KHZ,	// 88.2 KHz
	FS_96KHZ,	// 96 KHz
	FS_176KHZ,	// 176.4 KHz
	FS_192KHZ,	// 192 KHz
};

enum audio_bit_per_channel {
	AUDIO_16_BIT = 0,
	AUDIO_20_BIT,
	AUDIO_24_BIT,
};

enum audio_16bit_dma_mode {
	NORMAL_MODE = 0,
	PACKED_MODE = 1,
	PACKED_MODE2 = 2,
};

enum audio_dma_word_length {
	WORD_LENGTH_1 = 0,
	WORD_LENGTH_2,
	WORD_LENGTH_3,
	WORD_LENGTH_4,
	WORD_LENGTH_5,
	WORD_LENGTH_6,
	WORD_LENGTH_7,
	WORD_LENGTH_8,
};

enum audio_clock_accuracy {
	Level2 = 0,
	Level1 = 1,
	Level3 = 2,
	NOT_MATCH = 3,
};

// BIST Config
enum bist_pattern_type {
	// Normal BIST Patterns
	COLOR_BAR = 0,
	WGB_BAR,
	MW_BAR,
	// CTS BIST Patterns
	CTS_COLOR_RAMP,
	CTS_BLACK_WHITE,
	CTS_COLOR_SQUARE_VESA,
	CTS_COLOR_SQUARE_CEA,
};

// InfoFrame
#define INFOFRAME_PACKET_TYPE_SPD 0x83		/* Source Product Description InfoFrame */
#define INFOFRAME_PACKET_TYPE_AUDIO 0x84	/* Audio Information InfoFrame */
#define MAX_INFOFRAME_LENGTH 27

#define SPD_INFOFRAME_VERSION 0x01
#define SPD_INFOFRAME_LENGTH 25

#define AUDIO_INFOFRAME_VERSION 0x01
#define AUDIO_INFOFRAME_LENGTH 0x0A

struct infoframe {
	u8 type_code;
	u8 version_number;
	u8 length;
	u8 data[MAX_INFOFRAME_LENGTH];
};


/*
 * DisplayPort HW Configuration Structure Definition
 * It can be used CAL function and driver layer
 */
struct dp_hw_config {
	/* USB Type-C */
	enum pin_assignment pin_type;
	enum plug_orientation orient_type;
	bool aux_auto_orientation;

	/* DP Link */
	enum dp_link_rate_type link_rate;
	u32 num_lanes;

	/* HDCP */
	enum hdcp_mode hdcp;

	/* Video */
	enum dp_dynamic_range_type range;
	enum bit_depth bpc;
	struct video_timing vtiming;
	bool enhanced_mode;
	bool use_fec;
	bool use_ssc;

	/* Audio */
	u32 num_audio_ch;
	enum audio_sampling_frequency audio_fs;
	enum audio_bit_per_channel audio_bit;
	enum audio_16bit_dma_mode audio_packed_mode;
	enum audio_dma_word_length audio_word_length;

	/* BIST Mode */
	bool bist_mode;
	enum bist_pattern_type bist_type;

	/* USBDP combo phy enable ref count */
	atomic_t usbdp_phy_en_cnt;

	/* DP PHY boost */
	bool phy_boost;

	/* DP Emulation Mode */
	bool dp_emul;
};


/* DP memory map interface */
void dp_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		       enum dp_regs_type type, unsigned int id);

/* DPCD (DisplayPort Configuration Data) Read/Write Interfaces */
int dp_hw_write_dpcd_burst(u32 address, u32 length, u8 *data);
int dp_hw_read_dpcd_burst(u32 address, u32 length, u8 *data);
int dp_hw_read_edid(u8 block_cnt, u32 length, u8 *data);

/* DP Hardware Control Interfaces */
int dp_hw_init(struct dp_hw_config *hw_config);
int dp_hw_reinit(struct dp_hw_config *hw_config);
void dp_hw_deinit(struct dp_hw_config *hw_config);
void dp_hw_start(void);
void dp_hw_stop(void);

void dp_hw_set_video_config(struct dp_hw_config *hw_config);
int  dp_hw_set_bist_video_config(struct dp_hw_config *hw_config);

void dp_hw_get_intr_source(bool *common, bool *str, bool *pcs);
u32  dp_hw_get_and_clear_common_intr(void);
u32  dp_hw_get_and_clear_video_intr(void);

void dp_hw_send_audio_infoframe(struct infoframe audio_infoframe);
void dp_hw_send_avi_infoframe(struct infoframe avi_infoframe);
void dp_hw_send_spd_infoframe(struct infoframe spd_infoframe);

void dp_hw_set_fec(bool en);
void dp_hw_set_training_pattern(dp_training_pattern pattern);
void dp_hw_set_quality_pattern(dp_qual_pattern pattern, dp_scrambling scramble);
void dp_hw_set_voltage_and_pre_emphasis(struct dp_hw_config *hw_config, u8 *voltage, u8 *pre_emphasis);

void dp_hw_init_audio(void);
void dp_hw_deinit_audio(void);
void dp_hw_start_audio(void);
void dp_hw_stop_audio(void);
void dp_hw_set_audio_config(struct dp_hw_config *hw_config);
int  dp_hw_set_bist_audio_config(struct dp_hw_config *hw_config);
void dp_hw_set_audio_dma(u32 en);

bool dp_hw_get_hdcp13_key_valid(void);
void dp_hw_set_hdcp13_repeater(u8 is_repeater);
void dp_hw_set_hdcp13_bksv(u8 *bksv);
void dp_hw_get_hdcp13_an(u8 *an);
bool dp_hw_get_hdcp13_aksv(u8 *aksv);
void dp_hw_get_hdcp13_r0(u32 *r0);
void dp_hw_get_hdcp13_am0(u8 *am);
void dp_hw_set_hdcp13_function(u32 en);
void dp_hw_set_hdcp13_encryption(u32 en);
void dp_hw_set_hdcp22_function(u32 en);
void dp_hw_set_hdcp22_encryption(u32 en);

int dp_crc_set_enabled(u32 id, u32 en);
int dp_crc_get(u32 id, u32 crc_data[3]);
int dp_crc_reset(u32 id);

#endif /* __SAMSUNG_DP_CAL_H__ */
