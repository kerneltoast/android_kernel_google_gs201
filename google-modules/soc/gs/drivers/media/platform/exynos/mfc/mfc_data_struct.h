/*
 * drivers/media/platform/exynos/mfc/mfc_data_struct.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_DATA_STRUCT_H
#define __MFC_DATA_STRUCT_H __FILE__

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
#define CONFIG_MFC_USE_BUS_DEVFREQ
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
#define CONFIG_MFC_USE_BTS
#if IS_ENABLED(CONFIG_EXYNOS9610_BTS)
#define CONFIG_MFC_NO_RENEWAL_BTS
#endif
#endif

/* Encoder test using register dump */
//#define CONFIG_MFC_REG_TEST

#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
#include <soc/google/exynos_pm_qos.h>
#endif
#ifdef CONFIG_MFC_USE_BTS
#include <soc/google/bts.h>
#endif
#include <linux/videodev2.h>
#if IS_ENABLED(CONFIG_EXYNOS_ITMON) || IS_ENABLED(CONFIG_EXYNOS_ITMON_V2)
#define CONFIG_MFC_USE_ITMON
#include <soc/google/exynos-itmon.h>
#endif
#if IS_ENABLED(CONFIG_EXYNOS_MEMORY_LOGGER)
#include <soc/samsung/memlogger.h>
#endif
#if IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
#include <soc/samsung/imgloader.h>
#endif
#if IS_ENABLED(CONFIG_EXYNOS_SYSTEM_EVENT)
#include <soc/samsung/sysevent.h>
#include <soc/samsung/sysevent_notif.h>
#endif
#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
#include <linux/platform_data/sscoredump.h>
#define CONFIG_MFC_USE_COREDUMP
#endif

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>

#include "mfc_media.h"

#define MFC_NUM_CORE			2
#define MFC_NUM_CONTEXTS		32
#define MFC_MAX_PLANES			3
#define MFC_MAX_DPBS			64
#define MFC_MAX_BUFFERS			32
#define MFC_MAX_EXTRA_BUF		10
#define MFC_SFR_LOGGING_COUNT_SET0	10
#define MFC_SFR_LOGGING_COUNT_SET1	28
#define MFC_SFR_LOGGING_COUNT_SET2	32
#define MFC_LOGGING_DATA_SIZE		950
#define MFC_MAX_DEFAULT_PARAM		100
#define MFC_NUM_EXTRA_DPB		5
/* The number of display DRC max frames that can occur continuously in NAL_Q */
#define MFC_MAX_DRC_FRAME              4

/* SLC */
#define MFC_MAX_SLC_PARTITIONS		3

/* OTF */
#define HWFC_MAX_BUF			10
#define OTF_MAX_BUF			30

/* HDR */
#define HDR_MAX_WINDOWS			3
#define HDR_MAX_SCL			3
#define HDR_MAX_DISTRIBUTION		15
#define HDR_MAX_BEZIER_CURVES		15
#define HDR10_PLUS_DATA_SIZE		1024

/* AV1 Film Grain */
#define AV1_FG_LUM_POS_SIZE 14
#define AV1_FG_CHR_POS_SIZE 10
#define AV1_FG_LUM_AR_COEF_SIZE 24
#define AV1_FG_CHR_AR_COEF_SIZE 25


/* Maximum number of temporal layers */
#define VIDEO_MAX_TEMPORAL_LAYERS	7

/* Batch mode */
#define MAX_NUM_IMAGES_IN_VB		8
#define MAX_NUM_BUFCON_BUFS		32

/* QoS */
#define MAX_TIME_INDEX			15
#define MAX_NUM_CLUSTER			3
#define MAX_NUM_MFC_BPS			2
#define MAX_NUM_MFC_FREQ		10

/* MFC FMT FLAG */
#define MFC_FMT_FLAG_SBWCL_50		0x0001
#define MFC_FMT_FLAG_SBWCL_60		0x0002
#define MFC_FMT_FLAG_SBWCL_75		0x0004
#define MFC_FMT_FLAG_SBWCL_80		0x0008

/* MFC meminfo */
#define MFC_MEMINFO_MAX_NUM		10

/* MFC SSR info length */
#define MFC_CRASH_INFO_LEN             100
/*
 *  MFC region id for smc
 */
enum {
	FC_MFC_EXYNOS_ID_MFC_SH        = 0,
	FC_MFC_EXYNOS_ID_VIDEO         = 1,
	FC_MFC_EXYNOS_ID_MFC_FW        = 2,
	FC_MFC_EXYNOS_ID_SECTBL        = 3,
	FC_MFC_EXYNOS_ID_G2D_WFD       = 4,
	FC_MFC_EXYNOS_ID_MFC_NFW       = 5,
	FC_MFC_EXYNOS_ID_VIDEO_EXT     = 6,
};

/**
 * enum mfc_inst_type - The type of an MFC device node.
 */
enum mfc_node_type {
	MFCNODE_INVALID = -1,
	MFCNODE_DECODER = 0,
	MFCNODE_ENCODER = 1,
	MFCNODE_DECODER_DRM = 2,
	MFCNODE_ENCODER_DRM = 3,
	MFCNODE_ENCODER_OTF = 4,
	MFCNODE_ENCODER_OTF_DRM = 5,
};

/**
 * enum mfc_core_state - The type of MFC core device.
 */
enum mfc_core_state {
	MFCCORE_INIT	= 0,
	MFCCORE_ERROR	= 1,
};

/**
 * enum mfc_inst_type - The type of an MFC instance.
 */
enum mfc_inst_type {
	MFCINST_INVALID = 0,
	MFCINST_DECODER = 1,
	MFCINST_ENCODER = 2,
};

/**
 * enum mfc_inst_state - The state of an MFC instance.
 */
enum mfc_inst_state {
	MFCINST_FREE = 0,
	MFCINST_INIT = 100,
	MFCINST_GOT_INST,
	MFCINST_HEAD_PARSED,
	MFCINST_RUNNING_BUF_FULL,
	MFCINST_RUNNING,
	MFCINST_FINISHING,
	MFCINST_RETURN_INST,
	MFCINST_ERROR,
	MFCINST_ABORT,
	MFCINST_RES_CHANGE_INIT,
	MFCINST_RES_CHANGE_FLUSH,
	MFCINST_RES_CHANGE_END,
	MFCINST_FINISHED,
	MFCINST_ABORT_INST,
	MFCINST_DPB_FLUSHING,
	MFCINST_SPECIAL_PARSING,
	MFCINST_SPECIAL_PARSING_NAL,
	MFCINST_MOVE_INST,
};

enum mfc_inst_state_query {
	EQUAL = 0,
	BIGGER,
	SMALLER,
	EQUAL_BIGGER,
	EQUAL_SMALLER,
	EQUAL_OR,
};

/**
 * enum mfc_queue_state - The state of buffer queue.
 */
enum mfc_queue_state {
	QUEUE_FREE = 0,
	QUEUE_BUFS_REQUESTED,
	QUEUE_BUFS_QUERIED,
	QUEUE_BUFS_MMAPED,
};

enum mfc_dec_wait_state {
	WAIT_NONE	= 0,
	WAIT_G_FMT	= (1 << 0),
	WAIT_STOP	= (1 << 1),
};

/**
 * enum mfc_check_state - The state for user notification
 */
enum mfc_check_state {
	MFCSTATE_PROCESSING = 0,
	MFCSTATE_DEC_RES_DETECT,
	MFCSTATE_DEC_TERMINATING,
	MFCSTATE_ENC_NO_OUTPUT,
	MFCSTATE_DEC_S3D_REALLOC,
};

enum mfc_buf_usage_type {
	MFCBUF_INVALID = 0,
	MFCBUF_NORMAL,
	MFCBUF_DRM,
	MFCBUF_NORMAL_FW,
	MFCBUF_DRM_FW,
};

enum mfc_buf_process_type {
	MFCBUFPROC_DEFAULT		= 0x0,
	MFCBUFPROC_COPY			= (1 << 0),
	MFCBUFPROC_SHARE		= (1 << 1),
	MFCBUFPROC_META			= (1 << 2),
	MFCBUFPROC_ANBSHARE		= (1 << 3),
	MFCBUFPROC_ANBSHARE_NV12L	= (1 << 4),
};

enum mfc_ctrl_type {
	MFC_CTRL_TYPE_GET_SRC	= 0x1,
	MFC_CTRL_TYPE_GET_DST	= 0x2,
	MFC_CTRL_TYPE_SET_SRC	= 0x4,
	MFC_CTRL_TYPE_SET_DST	= 0x8,
};

enum mfc_ctrl_mode {
	MFC_CTRL_MODE_NONE	= 0x0,
	MFC_CTRL_MODE_SFR	= 0x1,
	MFC_CTRL_MODE_CST	= 0x2,
};

enum mfc_mb_flag {
	/* Driver set to user when DST DQbuf */
	MFC_FLAG_HDR_CONTENT_LIGHT	= 0,
	MFC_FLAG_HDR_DISPLAY_COLOUR	= 1,
	MFC_FLAG_HDR_MAXTIX_COEFF	= 2,
	MFC_FLAG_HDR_COLOUR_DESC	= 3,
	MFC_FLAG_HDR_VIDEO_SIGNAL_TYPE	= 4,
	MFC_FLAG_BLACKBAR_DETECT	= 5,
	MFC_FLAG_HDR_PLUS		= 6,
	MFC_FLAG_DISP_RES_CHANGE	= 7,
	MFC_FLAG_UNCOMP			= 8,
	MFC_FLAG_FRAMERATE_CH		= 9,
	MFC_FLAG_SYNC_FRAME		= 10,
	MFC_FLAG_AV1_FILM_GRAIN		= 11,
	MFC_FLAG_MULTIFRAME		= 12,
	/* Driver set to user when SRC DQbuf */
	MFC_FLAG_CONSUMED_ONLY		= 15,
	/* User set to driver when SRC Qbuf */
	MFC_FLAG_ENC_SRC_FAKE		= 27,
	MFC_FLAG_ENC_SRC_UNCOMP		= 28,
	MFC_FLAG_CSD			= 29,
	MFC_FLAG_EMPTY_DATA		= 30,
	MFC_FLAG_LAST_FRAME		= 31,
};

enum mfc_frame_error_type {
	MFC_ERR_FRAME_NO_ERR		= 0,
	MFC_ERR_FRAME_CONCEALMENT	= 1,
	MFC_ERR_FRAME_SYNC_POINT	= 2,
	MFC_ERR_FRAME_BROKEN		= 3,
};

enum mfc_do_cache_flush {
	MFC_NO_CACHEFLUSH		= 0,
	MFC_CACHEFLUSH			= 1,
};

enum mfc_idle_mode {
	MFC_IDLE_MODE_NONE	= 0,
	MFC_IDLE_MODE_RUNNING	= 1,
	MFC_IDLE_MODE_IDLE	= 2,
	MFC_IDLE_MODE_CANCEL	= 3,
};

enum mfc_enc_src {
	MFC_ENC_SRC_SBWC_NO	= 0,
	MFC_ENC_SRC_SBWC_OFF	= 1,
	MFC_ENC_SRC_SBWC_ON	= 2,
};

enum mfc_nal_q_stop_cause {
	/* nal_q stop check cause */
	NALQ_STOP_DRM			= 0,
	NALQ_STOP_NO_RUNNING		= 1,
	NALQ_STOP_OTF			= 2,
	NALQ_STOP_BPG			= 3,
	NALQ_STOP_LAST_FRAME		= 4,
	NALQ_STOP_MULTI_FRAME		= 5,
	NALQ_STOP_DPB_FULL		= 6,
	NALQ_STOP_INTERLACE		= 7,
	NALQ_STOP_BLACK_BAR		= 8,
	NALQ_STOP_INTER_DRC		= 9,
	NALQ_STOP_SLICE_MODE		= 10,
	NALQ_STOP_RC_MODE		= 11,
	NALQ_STOP_NO_STRUCTURE		= 12,
	NALQ_STOP_2CORE			= 13,
	NALQ_STOP_TWO_PASS_ENC		= 14,
	NALQ_STOP_ADAPTIVE_GOP		= 15,
	NALQ_STOP_QPE_TWO_PASS		= 16,
	/* nal_q exception cause */
	NALQ_EXCEPTION_DRC		= 25,
	NALQ_EXCEPTION_NEED_DPB		= 26,
	NALQ_EXCEPTION_INTER_DRC	= 27,
	NALQ_EXCEPTION_SBWC_INTERLACE	= 28,
	NALQ_EXCEPTION_INTERLACE	= 29,
	NALQ_EXCEPTION_MULTI_FRAME	= 30,
	NALQ_EXCEPTION_ERROR		= 31,
};

enum mfc_regression_option {
	MFC_TEST_DEFAULT		= 0x1,
	MFC_TEST_ENC_QP			= 0x2,
	MFC_TEST_DEC_PER_FRAME		= 0x4,
};

enum mfc_debug_cause {
	/* panic cause */
	MFC_CAUSE_0WRITE_PAGE_FAULT		= 0,
	MFC_CAUSE_0PAGE_FAULT			= 1,
	MFC_CAUSE_1WRITE_PAGE_FAULT		= 2,
	MFC_CAUSE_1PAGE_FAULT			= 3,
	MFC_CAUSE_NO_INTERRUPT			= 4,
	MFC_CAUSE_NO_SCHEDULING			= 5,
	MFC_CAUSE_FAIL_STOP_NAL_Q		= 6,
	MFC_CAUSE_FAIL_STOP_NAL_Q_FOR_OTHER	= 7,
	MFC_CAUSE_FAIL_CLOSE_INST		= 8,
	MFC_CAUSE_FAIL_SLEEP			= 9,
	MFC_CAUSE_FAIL_WAKEUP			= 10,
	MFC_CAUSE_FAIL_RISC_ON			= 11,
	MFC_CAUSE_FAIL_DPB_FLUSH		= 12,
	MFC_CAUSE_FAIL_CACHE_FLUSH		= 13,
	MFC_CAUSE_FAIL_MOVE_INST		= 14,
	/* last information */
	MFC_LAST_INFO_BLACK_BAR                 = 26,
	MFC_LAST_INFO_NAL_QUEUE                 = 27,
	MFC_LAST_INFO_CLOCK                     = 28,
	MFC_LAST_INFO_POWER                     = 29,
	MFC_LAST_INFO_SHUTDOWN                  = 30,
	MFC_LAST_INFO_DRM                       = 31,
};

enum mfc_request_work {
	MFC_WORK_BUTLER		= 0x1,
	MFC_WORK_TRY		= 0x2,
};

enum mfc_qos_control {
	MFC_QOS_ON		= 0x1,
	MFC_QOS_OFF		= 0x2,
	MFC_QOS_TRIGGER		= 0x3,
};

enum mfc_core_type {
	MFC_CORE_INVALID		= -1,
	MFC_CORE_MAIN			= 0,
	MFC_CORE_SUB			= 1,
	MFC_CORE_TYPE_NUM		= 2,
};

enum mfc_op_core_type {
	MFC_OP_CORE_NOT_FIXED	= -1,
	MFC_OP_CORE_FIXED_0	= 0,
	MFC_OP_CORE_FIXED_1	= 1,
	MFC_OP_CORE_ALL		= 2,
};

enum mfc_op_mode {
	MFC_OP_SINGLE			= 0,
	MFC_OP_TWO_MODE1		= 1,
	MFC_OP_TWO_MODE2		= 2,
	MFC_OP_SWITCHING		= 3,
	MFC_OP_SWITCH_TO_SINGLE		= 4,
	MFC_OP_SWITCH_BUT_MODE2		= 5,
};

enum mfc_real_time {
	/* real-time */
	MFC_RT			= 0,
	/* low-priority real-time */
	MFC_RT_LOW		= 1,
	/* constrained real-time */
	MFC_RT_CON		= 2,
	/* non real-time */
	MFC_NON_RT		= 3,
	MFC_RT_UNDEFINED	= 4,
};

/* Secure Protection */
#define EXYNOS_SECBUF_VIDEO_FW_PROT_ID	2
#define EXYNOS_SECBUF_PROT_ALIGNMENTS	0x10000

struct buffer_smc_prot_info {
	unsigned int chunk_count;
	unsigned int dma_addr;
	unsigned int protect_id;
	unsigned int chunk_size;
	unsigned long paddr;
};

/* core driver */
extern struct platform_driver mfc_core_driver;

struct mfc_ctx;
struct mfc_core_ctx;

struct mfc_debug {
	u32	fw_version;
	u32	cause;
	u8	fault_status;
	u32	fault_trans_info;
	u32	fault_addr;
	u32     SFRs_set0[MFC_SFR_LOGGING_COUNT_SET0];
	u32     SFRs_set1[MFC_SFR_LOGGING_COUNT_SET1];
	u32	SFRs_set2[MFC_SFR_LOGGING_COUNT_SET2];
	u8	curr_ctx;
	u8	state;
	u8	last_cmd;
	u32	last_cmd_sec;
	u32	last_cmd_nsec;
	u8	last_int;
	u32	last_int_sec;
	u32	last_int_nsec;
	u32	frame_cnt;
	u8	hwlock_dev;
	u32	hwlock_ctx;
	u8	num_inst;
	u8	num_drm_inst;
	u8	power_cnt;
	u8	clock_cnt;
	/* for decoder only */
	u64	dynamic_used;
	u32	last_src_addr;
	u32	last_dst_addr[MFC_MAX_PLANES];
	/* total logging data */
	char	errorinfo[MFC_LOGGING_DATA_SIZE];
};

/**
 * struct mfc_buf - MFC buffer
 *
 */
struct mfc_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t addr[MAX_NUM_IMAGES_IN_VB][MFC_MAX_PLANES];
	phys_addr_t paddr;
	struct dma_buf *dmabufs[MAX_NUM_IMAGES_IN_VB][MFC_MAX_PLANES];
	struct dma_buf_attachment *attachments[MAX_NUM_IMAGES_IN_VB][MFC_MAX_PLANES];
	int src_index;
	int dpb_index;
	int next_index;
	int done_index;
	int used;
	int num_bufs_in_batch;
	int num_valid_bufs;
	unsigned char *vir_addr;
	u32 flag;
};

struct mfc_buf_queue {
	struct list_head head;
	unsigned int count;
};

struct mfc_bits {
	unsigned long bits;
	spinlock_t lock;
};

struct mfc_hwlock {
	struct list_head waiting_list;
	unsigned int wl_count;
	unsigned long bits;
	unsigned long dev;
	unsigned int owned_by_irq;
	unsigned int transfer_owner;
	spinlock_t lock;
	int migrate;
	struct mfc_core_ctx *mig_core_ctx;
};

struct mfc_listable_wq {
	struct list_head list;
	wait_queue_head_t wait_queue;
	struct mutex wait_mutex;
	struct mfc_dev *dev;
	struct mfc_core *core;
	struct mfc_ctx *ctx;
	struct mfc_core_ctx *core_ctx;
};

struct mfc_core_intlock {
	int lock;
	unsigned long bits;
	unsigned long pending;
	struct mutex core_mutex;
};

struct mfc_core_lock {
	int cnt;
	int migrate;
	spinlock_t lock;
	wait_queue_head_t wq;
	wait_queue_head_t migrate_wq;
};

struct mfc_pm {
	struct clk	*clock;
	atomic_t	pwr_ref;
	atomic_t	protect_ref;
	struct device	*device;
	spinlock_t	clklock;

	int clock_on_steps;
	int clock_off_steps;
	enum mfc_buf_usage_type base_type;
};

enum mfc_fw_status {
	MFC_FW_NONE		= 0,
	MFC_FW_ALLOC		= (1 << 0),	// 0x1
	MFC_CTX_ALLOC		= (1 << 1),	// 0x2
	MFC_FW_LOADED		= (1 << 2),	// 0x4
	MFC_FW_VERIFIED		= (1 << 3),	// 0x8
	MFC_FW_INITIALIZED	= (1 << 4),	// 0x10
};

struct mfc_fw {
	int			date;
	int			fimv_info;
	size_t			fw_size;
	enum mfc_fw_status	status;
	enum mfc_fw_status	drm_status;
};

struct mfc_ctx_buf_size {
	size_t dev_ctx;
	size_t h264_dec_ctx;
	size_t other_dec_ctx;
	size_t h264_enc_ctx;
	size_t hevc_enc_ctx;
	size_t other_enc_ctx;
	size_t dbg_info_buf;
};

struct mfc_buf_size {
	size_t firmware_code;
	unsigned int cpb_buf;
	struct mfc_ctx_buf_size *ctx_buf;
};

struct mfc_variant {
	struct mfc_buf_size *buf_size;
	int	num_entities;
};

enum mfc_sfr_dump_type {
	MFC_DUMP_NONE			= 0,
	MFC_DUMP_DEC_SEQ_START		= (1 << 0),
	MFC_DUMP_DEC_INIT_BUFS		= (1 << 1),
	MFC_DUMP_DEC_FIRST_NAL_START	= (1 << 2),
	MFC_DUMP_ENC_SEQ_START		= (1 << 3),
	MFC_DUMP_ENC_INIT_BUFS		= (1 << 4),
	MFC_DUMP_ENC_FIRST_NAL_START	= (1 << 5),
	MFC_DUMP_ERR_INT		= (1 << 6),
	MFC_DUMP_WARN_INT		= (1 << 7),
	MFC_DUMP_DEC_NAL_START		= (1 << 8),
	MFC_DUMP_DEC_FRAME_DONE		= (1 << 9),
	MFC_DUMP_ENC_NAL_START		= (1 << 10),
	MFC_DUMP_ENC_FRAME_DONE		= (1 << 11),
	MFC_DUMP_MOVE_INSTANCE_RET	= (1 << 12),
};

enum mfc_logging_option {
	MFC_LOGGING_NONE		= 0,
	MFC_LOGGING_PRINTK		= (1 << 0),
	MFC_LOGGING_MEMLOG_PRINTF	= (1 << 1),
	MFC_LOGGING_MEMLOG_SFR_DUMP	= (1 << 2),
	MFC_LOGGING_MEMLOG		= ((1 << 1) | (1 << 2)),
	MFC_LOGGING_ALL			= 0x7,
};

enum mfc_feature_option {
	MFC_OPTION_NONE			= 0,
	MFC_OPTION_RECON_SBWC_DISABLE	= (1 << 0),
	MFC_OPTION_DECODING_ORDER	= (1 << 1),
	MFC_OPTION_MEERKAT_DISABLE	= (1 << 2),
	MFC_OPTION_OTF_PATH_TEST_ENABLE	= (1 << 3),
	MFC_OPTION_MULTI_CORE_DISABLE	= (1 << 4),
	MFC_OPTION_SET_MULTI_CORE_FORCE	= (1 << 5),
	MFC_OPTION_BLACK_BAR_ENABLE	= (1 << 6),
};

enum mfc_get_img_size {
	MFC_GET_RESOL_SIZE		= 0,
	MFC_GET_RESOL_DPB_SIZE		= 1,
};

enum mfc_color_space {
	MFC_COLORSPACE_UNSPECIFICED	= 0,
	MFC_COLORSPACE_BT601		= 1,
	MFC_COLORSPACE_BT709		= 2,
	MFC_COLORSPACE_SMPTE_170	= 3,
	MFC_COLORSPACE_SMPTE_240	= 4,
	MFC_COLORSPACE_BT2020		= 5,
	MFC_COLORSPACE_RESERVED		= 6,
	MFC_COLORSPACE_SRGB		= 7,
};

enum mfc_color_primaries {
	MFC_PRIMARIES_RESERVED		= 0,
	MFC_PRIMARIES_BT709_5		= 1,
	MFC_PRIMARIES_UNSPECIFIED	= 2,
	MFC_PRIMARIES_BT470_6M		= 4,
	MFC_PRIMARIES_BT601_6_625	= 5,
	MFC_PRIMARIES_BT601_6_525	= 6,
	MFC_PRIMARIES_SMPTE_240M	= 7,
	MFC_PRIMARIES_GENERIC_FILM	= 8,
	MFC_PRIMARIES_BT2020		= 9,
};

enum mfc_transfer_characteristics {
	MFC_TRANSFER_RESERVED		= 0,
	MFC_TRANSFER_BT709		= 1,
	MFC_TRANSFER_UNSPECIFIED	= 2,
	/* RESERVED			= 3, */
	MFC_TRANSFER_GAMMA_22		= 4,
	MFC_TRANSFER_GAMMA_28		= 5,
	MFC_TRANSFER_SMPTE_170M		= 6,
	MFC_TRANSFER_SMPTE_240M		= 7,
	MFC_TRANSFER_LINEAR		= 8,
	MFC_TRANSFER_LOGARITHMIC	= 9,
	MFC_TRANSFER_LOGARITHMIC_S	= 10,
	MFC_TRANSFER_XvYCC		= 11,
	MFC_TRANSFER_BT1361		= 12,
	MFC_TRANSFER_SRGB		= 13,
	MFC_TRANSFER_BT2020_1		= 14,
	MFC_TRANSFER_BT2020_2		= 15,
	MFC_TRANSFER_ST2084		= 16,
	MFC_TRANSFER_ST428		= 17,
	MFC_TRANSFER_HLG		= 18,
};

enum mfc_matrix_coeff {
	MFC_MATRIX_COEFF_IDENTITY		= 0,
	MFC_MATRIX_COEFF_REC709			= 1,
	MFC_MATRIX_COEFF_UNSPECIFIED		= 2,
	MFC_MATRIX_COEFF_RESERVED		= 3,
	MFC_MATRIX_COEFF_470_SYSTEM_M		= 4,
	MFC_MATRIX_COEFF_470_SYSTEM_BG		= 5,
	MFC_MATRIX_COEFF_SMPTE170M		= 6,
	MFC_MATRIX_COEFF_SMPTE240M		= 7,
	MFC_MATRIX_COEFF_BT2020			= 9,
	MFC_MATRIX_COEFF_BT2020_CONSTANT	= 10,
};

struct mfc_debugfs {
	struct dentry *root;
};

/**
 * struct mfc_special_buf - represents internal used buffer
 * @daddr:		device virtual address
 * @virt:		kernel virtual address, only valid when the
 *			buffer accessed by driver
 */
struct mfc_special_buf {
	enum mfc_buf_usage_type	buftype;
	struct dma_buf			*dma_buf;
	struct dma_buf_attachment	*attachment;
	struct sg_table			*sgt;
	dma_addr_t			daddr;
	phys_addr_t			paddr;
	void				*vaddr;
	size_t				size;
	size_t				map_size;
};

struct mfc_mem {
	struct list_head	list;
	dma_addr_t		addr;
	size_t			size;
};

enum mfc_meminfo_type {
	MFC_MEMINFO_FW			= 0,
	MFC_MEMINFO_INTERNAL		= 1,
	MFC_MEMINFO_INPUT		= 2,
	MFC_MEMINFO_OUTPUT		= 3,
	MFC_MEMINFO_CTX_ALL		= 4,
	MFC_MEMINFO_CTX_MAX		= 5,
	MFC_MEMINFO_DEV_ALL		= 6,
};

struct mfc_meminfo {
	enum mfc_meminfo_type	type;
	const char		*name;
	unsigned int		count;
	size_t			size;
	size_t			total;
};

struct mfc_bw_data {
	unsigned int	peak;
	unsigned int	read;
	unsigned int	write;
};

struct mfc_bw_info {
	struct mfc_bw_data bw_enc_h264;
	struct mfc_bw_data bw_enc_hevc;
	struct mfc_bw_data bw_enc_hevc_10bit;
	struct mfc_bw_data bw_enc_vp8;
	struct mfc_bw_data bw_enc_vp9;
	struct mfc_bw_data bw_enc_vp9_10bit;
	struct mfc_bw_data bw_enc_mpeg4;
	struct mfc_bw_data bw_dec_h264;
	struct mfc_bw_data bw_dec_hevc;
	struct mfc_bw_data bw_dec_hevc_10bit;
	struct mfc_bw_data bw_dec_vp8;
	struct mfc_bw_data bw_dec_vp9;
	struct mfc_bw_data bw_dec_vp9_10bit;
	struct mfc_bw_data bw_dec_av1;
	struct mfc_bw_data bw_dec_av1_10bit;
	struct mfc_bw_data bw_dec_mpeg4;
};

/*
 * threshold_mb - threshold of total MB(macroblock) count
 * Total MB count can be calculated by
 *	(MB of width) * (MB of height) * fps
 */
struct mfc_qos {
	unsigned int threshold_mb;
	unsigned int freq_mfc;
	unsigned int freq_int;
	unsigned int freq_mif;
	unsigned int mo_value;
	unsigned int mo_10bit_value;
	unsigned int mo_uhd_enc60_value;
	unsigned int time_fw;
	unsigned int bts_scen_idx;
	const char *name;
};

struct mfc_qos_boost {
	unsigned int num_cluster;
	unsigned int freq_mfc;
	unsigned int freq_int;
	unsigned int freq_mif;
	unsigned int freq_cluster[MAX_NUM_CLUSTER];
	unsigned int bts_scen_idx;
	const char *name;
};

struct mfc_qos_weight {
	unsigned int weight_h264_hevc;
	unsigned int weight_vp8_vp9;
	unsigned int weight_other_codec;
	unsigned int weight_3plane;
	unsigned int weight_10bit;
	unsigned int weight_422;
	unsigned int weight_bframe;
	unsigned int weight_num_of_ref;
	unsigned int weight_gpb;
	unsigned int weight_num_of_tile;
	unsigned int weight_super64_bframe;
	unsigned int weight_mbaff;
};

struct mfc_feature {
	unsigned int support;
	unsigned int version;
};

struct mfc_platdata {
	/* Debug mode */
	unsigned int debug_mode;
	/* Default 10bit format for decoding and dithering for display */
	unsigned int P010_decoding;
	unsigned int dithering_enable;
	unsigned int stride_align;
	unsigned int stride_type;
	unsigned int support_8K_cavlc;
	/* Formats */
	unsigned int support_10bit;
	unsigned int support_422;
	unsigned int support_rgb;
	/* SBWC */
	unsigned int support_sbwc;
	unsigned int support_sbwcl;
	/* SBWC decoder max resolution */
	unsigned int sbwc_dec_max_width;
	unsigned int sbwc_dec_max_height;
	unsigned int sbwc_dec_hdr10_off;
	/* HDR10+ */
	unsigned int max_hdr_win;
	/* error type for sync_point display */
	unsigned int display_err_type;
	unsigned int security_ctrl;
	/* NAL-Q size */
	unsigned int nal_q_entry_size;
	unsigned int nal_q_dump_size;
	/* Features */
	struct mfc_feature nal_q;
	struct mfc_feature skype;
	struct mfc_feature black_bar;
	struct mfc_feature color_aspect_dec;
	struct mfc_feature static_info_dec;
	struct mfc_feature color_aspect_enc;
	struct mfc_feature static_info_enc;
	struct mfc_feature hdr10_plus;
	struct mfc_feature vp9_stride_align;
	struct mfc_feature sbwc_uncomp;
	struct mfc_feature mem_clear;
	struct mfc_feature wait_fw_status;
	struct mfc_feature wait_nalq_status;
	struct mfc_feature drm_switch_predict;
	struct mfc_feature sbwc_enc_src_ctrl;
	struct mfc_feature average_qp;
	struct mfc_feature mv_search_mode;
	struct mfc_feature enc_idr_flag;
	struct mfc_feature min_quality_mode;
	struct mfc_feature hevc_pic_output_flag;
	struct mfc_feature metadata_interface;
	struct mfc_feature hdr10_plus_full;
	struct mfc_feature enc_capability;
	struct mfc_feature enc_sub_gop;
	struct mfc_feature enc_ts_delta;

	/* AV1 Decoder */
	unsigned int support_av1_dec;
	struct mfc_feature av1_film_grain;

	/* Encoder default parameter */
	unsigned int enc_param_num;
	unsigned int enc_param_addr[MFC_MAX_DEFAULT_PARAM];
	unsigned int enc_param_val[MFC_MAX_DEFAULT_PARAM];

	/* Encoder min bit count control */
	unsigned int enc_min_bit_cnt;

	struct mfc_bw_info mfc_bw_info;
	struct mfc_bw_info mfc_bw_info_sbwc;
	struct mfc_bw_info mfc_bw_info_dpb_sbwc;
	struct mfc_qos_weight qos_weight;

	unsigned int ip_ver;
	int num_mfc_freq;
	unsigned int mfc_freqs[MAX_NUM_MFC_FREQ];
	unsigned int max_Kbps[MAX_NUM_MFC_BPS];
	unsigned int core_balance;
	unsigned int iova_threshold;
	unsigned int idle_clk_ctrl;

	unsigned int enc_rgb_csc_by_fw;
};

struct mfc_core_platdata {
	/* MFC version */
	unsigned int ip_ver;
	/* Sysmmu check */
	unsigned int share_sysmmu;
	unsigned int axid_mask;
	unsigned int mfc_fault_num;
	unsigned int trans_info_offset;
	unsigned int fault_status_offset;
	unsigned int fault_pmmuid_offset;
	unsigned int fault_pmmuid_shift;

	/* vOTF */
	unsigned int mfc_votf_base;
	unsigned int gdc_votf_base;
	unsigned int dpu_votf_base;
	unsigned int votf_start_offset;
	unsigned int votf_end_offset;
	/* QoS */
	unsigned int num_default_qos_steps;
	unsigned int num_encoder_qos_steps;
	unsigned int max_mb;
	unsigned int mfc_freq_control;
	unsigned int mo_control;
	unsigned int bw_control;
	unsigned int mfc_bw_index;
	struct mfc_qos *default_qos_table;
	struct mfc_qos *encoder_qos_table;
	struct mfc_qos_boost *qos_boost_table;
};

/************************ NAL_Q data structure ************************/
#define NAL_Q_ENTRY_SIZE_FOR_HDR10	512

/* slot 4 * max instance 32 = 128 */
#define NAL_Q_QUEUE_SIZE		128
#define NAL_Q_DECODER_MARKER		0xAAAAAAAA
#define NAL_Q_ENCODER_MARKER		0xBBBBBBBB

typedef struct __DecoderInputStr {
	int StartCode; /* NAL_Q_DECODER_MARKER */
	int CommandId;
	int InstanceId;
	int PictureTag;
	unsigned int CpbBufferAddr;
	int CpbBufferSize;
	int CpbBufferOffset;
	int StreamDataSize;
	int AvailableDpbFlagUpper;
	int AvailableDpbFlagLower;
	int DynamicDpbFlagUpper;
	int DynamicDpbFlagLower;
	unsigned int FrameAddr[3];
	int FrameSize[3];
	int NalStartOptions;
	int FrameStrideSize[3];
	int Frame2BitSize[2];
	int Frame2BitStrideSize[2];
	unsigned int ScratchBufAddr;
	int ScratchBufSize;
} DecoderInputStr; /* 28*4 = 112 bytes */

typedef struct __EncoderInputStr {
	int StartCode; /* NAL_Q_ENCODER_MARKER */
	int CommandId;
	int InstanceId;
	int PictureTag;
	unsigned int FrameAddr[3];
	unsigned int StreamBufferAddr;
	int StreamBufferSize;
	int StreamBufferOffset;
	int RcRoiCtrl;
	unsigned int RoiBufferAddr;
	int ParamChange;
	int IrSize;
	int GopConfig;
	int RcFrameRate;
	int RcBitRate;
	int MsliceMode;
	int MsliceSizeMb;
	int MsliceSizeBits;
	int FrameInsertion;
	int HierarchicalBitRateLayer[7];
	int H264RefreshPeriod;
	int HevcRefreshPeriod;
	int RcQpBound;
	int RcQpBoundPb;
	int FixedPictureQp;
	int PictureProfile;
	int BitCountEnable;
	int MaxBitCount;
	int MinBitCount;
	int NumTLayer;
	int H264NalControl;
	int HevcNalControl;
	int Vp8NalControl;
	int Vp9NalControl;
	int H264HDSvcExtension0;
	int H264HDSvcExtension1;
	int GopConfig2;
	int Frame2bitAddr[2];
	int Weight;
	int ExtCtbQpAddr;
	int WeightUpper;
	int RcMode;
	int St2094_40sei[30];
	int SourcePlaneStride[3];
	int SourcePlane2BitStride[2];
	int MVHorRange;
	int MVVerRange;
	int TimeStampDelta;
} EncoderInputStr; /* 89*4 = 356 bytes */

typedef struct __DecoderOutputStr {
	int StartCode; /* NAL_Q_DECODER_MARKER */
	int CommandId;
	int InstanceId;
	int ErrorCode;
	int PictureTagTop;
	int PictureTimeTop;
	int DisplayFrameWidth;
	int DisplayFrameHeight;
	int DisplayStatus;
	unsigned int DisplayAddr[3];
	int DisplayFrameType;
	int DisplayCropInfo1;
	int DisplayCropInfo2;
	int DisplayPictureProfile;
	int DisplayAspectRatio;
	int DisplayExtendedAr;
	int DecodedNalSize;
	int UsedDpbFlagUpper;
	int UsedDpbFlagLower;
	int SeiAvail;
	int FramePackArrgmentId;
	int FramePackSeiInfo;
	int FramePackGridPos;
	int DisplayRecoverySeiInfo;
	int H264Info;
	int DisplayFirstCrc;
	int DisplaySecondCrc;
	int DisplayThirdCrc;
	int DisplayFirst2BitCrc;
	int DisplaySecond2BitCrc;
	int DecodedFrameWidth;
	int DecodedFrameHeight;
	int DecodedStatus;
	unsigned int DecodedAddr[3];
	int DecodedFrameType;
	int DecodedCropInfo1;
	int DecodedCropInfo2;
	int DecodedPictureProfile;
	int DecodedRecoverySeiInfo;
	int DecodedFirstCrc;
	int DecodedSecondCrc;
	int DecodedThirdCrc;
	int DecodedFirst2BitCrc;
	int DecodedSecond2BitCrc;
	int PictureTagBot;
	int PictureTimeBot;
	int ChromaFormat;
	int Mpeg4Info;
	int HevcInfo;
	int Vc1Info;
	int VideoSignalType;
	int ContentLightLevelInfoSei;
	int MasteringDisplayColourVolumeSei0;
	int MasteringDisplayColourVolumeSei1;
	int MasteringDisplayColourVolumeSei2;
	int MasteringDisplayColourVolumeSei3;
	int MasteringDisplayColourVolumeSei4;
	int MasteringDisplayColourVolumeSei5;
	int FirstPlaneDpbSize;
	int SecondPlaneDpbSize;
	int St2094_40sei[30];
	int Vp9Info;
	unsigned int MfcHwCycle;
	unsigned int MfcProcessingCycle;
	unsigned int DpbStrideSize[3];
	unsigned int Dpb2bitStrideSize[2];
	int MetadataStatus;
	unsigned int MetadataAddrConcealedMb;
	int MetadataSizeConcealedMb;
	unsigned int MetadataAddrVc1Mb;
	int MetadataSizeVc1Mb;
	unsigned int MetadataAddrSeiMb;
	int MetadataSizeSeiMb;
	unsigned int MetadataAddrVuiMb;
	int MetadataSizeVuiMb;
	unsigned int MetadataAddrMvcMb;
	int MetadataSizeMvcMb;
	/* Below is not used */
	int AV1Info;
	int FilmGrain[44];
} DecoderOutputStr; /* 113*4 = 452 bytes */

typedef struct __EncoderOutputStr {
	int StartCode; /* NAL_Q_ENCODER_MARKER */
	int CommandId;
	int InstanceId;
	int ErrorCode;
	int PictureTag;
	unsigned int EncodedFrameAddr[3];
	unsigned int StreamBufferAddr;
	int StreamBufferOffset;
	int StreamSize;
	int SliceType;
	int NalDoneInfo;
	unsigned int ReconLumaDpbAddr;
	unsigned int ReconChromaDpbAddr;
	int EncCnt;
	unsigned int MfcHwCycle;
	unsigned int MfcProcessingCycle;
	unsigned int SumSkipMb;
	unsigned int SumIntraMb;
	unsigned int SumZeroMvMb;
} EncoderOutputStr; /* 21*4 = 84 bytes */

/**
 * enum nal_queue_state - The state for nal queue operation.
 */
typedef enum _nal_queue_state {
	NAL_Q_STATE_CREATED = 0,
	NAL_Q_STATE_STARTED, /* when mfc_nal_q_start() is called */
	NAL_Q_STATE_STOPPED, /* when mfc_nal_q_stop() is called */
} nal_queue_state;

struct _nal_queue_handle;
typedef struct _nal_queue_in_handle {
	struct _nal_queue_handle *nal_q_handle;
	struct mfc_special_buf in_buf;
	unsigned int in_exe_count;
	void *nal_q_in_addr;
} nal_queue_in_handle;

typedef struct _nal_queue_out_handle {
	struct _nal_queue_handle *nal_q_handle;
	struct mfc_special_buf out_buf;
	unsigned int out_exe_count;
	void *nal_q_out_addr;
	int nal_q_ctx;
} nal_queue_out_handle;

typedef struct _nal_queue_handle {
	nal_queue_in_handle *nal_q_in_handle;
	nal_queue_out_handle *nal_q_out_handle;
	nal_queue_state nal_q_state;
	unsigned int nal_q_clk_cnt;
	spinlock_t lock;
	int nal_q_exception;
} nal_queue_handle;

/************************ OTF data structure ************************/
struct _otf_buf_addr {
	dma_addr_t otf_daddr[HWFC_MAX_BUF][3];
	struct dma_buf_attachment *otf_buf_attach[HWFC_MAX_BUF];
	struct sg_table *sgt[HWFC_MAX_BUF];
};

struct _otf_buf_info {
	int pixel_format;
	int width;
	int height;
	int buffer_count;
	struct dma_buf *bufs[HWFC_MAX_BUF];
};

struct _otf_debug {
	struct mfc_special_buf stream_buf[OTF_MAX_BUF];
	unsigned int stream_size[OTF_MAX_BUF];
	unsigned char frame_cnt;
};

struct _otf_handle {
	int otf_work_bit;
	int otf_buf_index;
	int otf_job_id;
	u64 otf_time_stamp;
	struct _otf_buf_addr otf_buf_addr;
	struct _otf_buf_info otf_buf_info;
	struct _otf_debug otf_debug;
};
/********************************************************************/

struct mfc_perf {
	void __iomem *regs_base0;
	void __iomem *regs_base1;

	struct timespec64 begin;
	struct timespec64 end;

	int new_start;
	int count;
	int drv_margin;
};

extern struct mfc_dump_ops mfc_dump_ops;
struct mfc_dump_ops {
	void (*dump_info_context)(struct mfc_dev *dev);
	void (*dump_and_stop_debug_mode)(struct mfc_dev *dev);
};

extern struct mfc_core_dump_ops mfc_core_dump_ops;
struct mfc_core_dump_ops {
	void (*dump_regs)(struct mfc_core *core);
	void (*dump_info)(struct mfc_core *core);
	void (*dump_info_without_regs)(struct mfc_core *core);
	void (*dump_and_stop_always)(struct mfc_core *core);
	void (*dump_and_stop_debug_mode)(struct mfc_core *core);
};

struct mfc_bitrate_table {
	int mfc_freq;
	int bps_interval;
};

struct mfc_dev_memlog {
#if IS_ENABLED(CONFIG_EXYNOS_MEMORY_LOGGER)
	struct memlog *desc;
	struct memlog_obj *log_obj;
#endif
	unsigned int log_enable;
};

struct mfc_core_memlog {
#if IS_ENABLED(CONFIG_EXYNOS_MEMORY_LOGGER)
	struct memlog_obj *sfr_obj;
	char sfr_obj_name[9];
#endif
	unsigned int sfr_enable;
};

/**
 * struct mfc_dev - The struct containing driver internal parameters.
 */
struct mfc_dev {
	struct mfc_core	*core[MFC_NUM_CORE];
	int num_core;
	int fw_date;
	size_t fw_rmem_offset;

	struct device		*device;
	struct device		*cache_op_dev;
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd_dec;
	struct video_device	*vfd_enc;
	struct video_device	*vfd_dec_drm;
	struct video_device	*vfd_enc_drm;
	struct video_device	*vfd_enc_otf;
	struct video_device	*vfd_enc_otf_drm;
	struct mfc_platdata	*pdata;
	struct mfc_variant	*variant;

	int num_inst;
	int num_otf_inst;
	int num_drm_inst;

	unsigned long otf_inst_bits;
	unsigned long multi_core_inst_bits;

	struct mutex mfc_mutex;
	struct mutex mfc_migrate_mutex;
	struct mutex sysmmu_fault_mutex;

	struct mfc_ctx *ctx[MFC_NUM_CONTEXTS];
	struct mfc_ctx *move_ctx[MFC_NUM_CONTEXTS];
	int move_ctx_cnt;
	struct list_head ctx_list;
	spinlock_t ctx_list_lock;

	atomic_t queued_cnt;

	/* Trace */
	atomic_t trace_ref;
	struct _mfc_trace *mfc_trace;
	atomic_t trace_ref_longterm;
	struct _mfc_trace *mfc_trace_longterm;
	atomic_t trace_ref_rm;
	struct _mfc_trace *mfc_trace_rm;

	/* Debugfs and dump */
	struct mfc_debugfs debugfs;
	struct mfc_dump_ops *dump_ops;

	/* Instance migration worker */
	struct workqueue_struct *migration_wq;
	struct work_struct migration_work;

	/* Butler */
	struct workqueue_struct *butler_wq;
	struct work_struct butler_work;

	/* QoS bitrate */
	struct mfc_bitrate_table bitrate_table[MAX_NUM_MFC_FREQ];
	int bps_ratio;

	/* Lazy unmap disable */
	int skip_lazy_unmap;

	/* Reg test */
	char *reg_buf;
	unsigned int *reg_val;
	unsigned int reg_cnt;

	/* Regression test result */
	unsigned int *regression_val;
	unsigned int regression_cnt;

	struct mfc_meminfo meminfo[MFC_MEMINFO_DEV_ALL + 1];
	struct mfc_dev_memlog memlog;
	char dev_crash_info[MFC_CRASH_INFO_LEN];
};

struct mfc_core_ops {
	int (*instance_init)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	int (*instance_deinit)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	int (*instance_open)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	int (*instance_cache_flush)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	int (*instance_move_to)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	int (*instance_move_from)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	int (*request_work)(struct mfc_core *core,
			enum mfc_request_work work,
			struct mfc_ctx *ctx);
	/* for DEC */
	void (*instance_csd_parsing)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	void (*instance_dpb_flush)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	int (*instance_init_buf)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	/* for ENC */
	void (*instance_q_flush)(struct mfc_core *core,
			struct mfc_ctx *ctx);
	void (*instance_finishing)(struct mfc_core *core,
			struct mfc_ctx *ctx);
};

struct dump_info {
	char		*name;
	void            *addr;
	u64             size;
};

struct mfc_core {
	struct device		*device;
	struct iommu_domain	*domain;

	const struct mfc_core_ops *core_ops;

	void __iomem		*regs_base;
	void __iomem		*sysmmu0_base;
	void __iomem		*sysmmu1_base;
	void __iomem		*hwfc_base;
	void __iomem		*votf_base;
	void __iomem		*cmu_busc_base;
	void __iomem		*cmu_mif0_base;
	void __iomem		*cmu_mif1_base;
	void __iomem		*cmu_mif2_base;
	void __iomem		*cmu_mif3_base;
	/* for SLC */
	void __iomem		*ssmt0_base;
	void __iomem		*ssmt1_base;
	void __iomem		*sysreg_base;

	unsigned int		id;
	char			name[10];
	int			irq;
	struct resource		*mfc_mem;
#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
	struct pt_handle	*pt_handle;

	/* 0: internal buffer, 1: DPB reference write, 2:Reference pixel read */
	int			ptid[MFC_MAX_SLC_PARTITIONS];

	/* num of slc partition from device tree */
	int 			num_slc_pt;

	/* index of current slc partition which be enabled */
	int			curr_slc_pt_idx[MFC_MAX_SLC_PARTITIONS];

	/* current slc option be used */
	int			curr_slc_option;
#endif

	struct mfc_variant	*variant;
	struct mfc_core_platdata *core_pdata;

	enum mfc_core_state state;

	bool has_2sysmmu;
	bool has_hwfc;
	bool has_mfc_votf;
	bool has_gdc_votf;
	bool has_dpu_votf;
	bool has_cmu;
	int has_llc;
	int has_slc;
	int need_llc_flush;
	int llc_on_status;
	int slc_on_status;

	/* Power and Clock */
	atomic_t clk_ref;
	struct mfc_pm	pm;
	struct mfc_perf perf;
	bool continue_clock_on;
	bool sleep;
	bool shutdown;
#if IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
	/* Exynos Image Loader */
	struct imgloader_desc   mfc_imgloader_desc;
#endif
	/* Internal buffers */
	struct mfc_fw		fw;
	struct mfc_special_buf	fw_buf;
	struct mfc_special_buf	drm_fw_buf;
	struct mfc_special_buf	common_ctx_buf;
	struct mfc_special_buf	drm_common_ctx_buf;
	struct mfc_special_buf	dbg_info_buf;

	/* Secure F/W prot information */
	struct buffer_smc_prot_info *drm_fw_prot;

	/* Context information */
	struct mfc_dev *dev;
	struct mfc_core_ctx *core_ctx[MFC_NUM_CONTEXTS];
	int curr_core_ctx;
	int preempt_core_ctx;
	int curr_core_ctx_is_drm;
	int num_inst;

	int num_drm_inst;
	int int_condition;
	int int_reason;
	unsigned int int_err;

	/* HW lock */
	struct mfc_bits work_bits;
	struct mfc_hwlock hwlock;
	struct mfc_listable_wq hwlock_wq;
	wait_queue_head_t cmd_wq;

	struct mfc_core_dump_ops *dump_ops;

	/* Meerkat */
	atomic_t meerkat_tick_running;
	atomic_t meerkat_tick_cnt;
	atomic_t meerkat_run;
	struct timer_list meerkat_timer;
	struct workqueue_struct *meerkat_wq;
	struct work_struct meerkat_work;

	/* QoS idle */
	atomic_t hw_run_cnt;
	atomic_t during_release;
	atomic_t during_idle_resume;
	struct mutex idle_qos_mutex;
	enum mfc_idle_mode idle_mode;
	struct timer_list mfc_idle_timer;
	struct workqueue_struct *mfc_idle_wq;
	struct work_struct mfc_idle_work;

	/* for DRM */
	int cache_flush_flag;
	int last_cmd_has_cache_flush;

	/* Butler */
	struct workqueue_struct *butler_wq;
	struct work_struct butler_work;

	/* QoS */
	struct list_head qos_queue;
	atomic_t qos_req_cur;
#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
	struct exynos_pm_qos_request qos_req_mfc_noidle;
	struct exynos_pm_qos_request qos_req_mfc;
	struct exynos_pm_qos_request qos_req_int;
	struct exynos_pm_qos_request qos_req_mif;
	struct exynos_pm_qos_request qos_req_cluster[MAX_NUM_CLUSTER];
#endif
	struct mutex qos_mutex;
	int mfc_freq_by_bps;
	int last_mfc_freq;
#if IS_ENABLED(CONFIG_EXYNOS_BTS)
	struct bts_bw mfc_bw;
	unsigned int prev_bts_scen_idx;
#endif
	unsigned long total_mb;

	/* NAL_Q */
	nal_queue_handle *nal_q_handle;
	unsigned int nal_q_stop_cause;

	/* Logging trace data */
	atomic_t trace_ref_log;
	struct _mfc_trace_logging *mfc_trace_logging;
	struct mfc_debug *logging_data;
	int last_cmd;
	int last_int;
	struct timespec64 last_cmd_time;
	struct timespec64 last_int_time;
	/* debug info dump */
	struct dump_info dbg_info;
	struct platform_device *sscd_dev;

	/* ITMON */
#ifdef CONFIG_MFC_USE_ITMON
	struct notifier_block itmon_nb;
#endif
	int itmon_notified;

#if IS_ENABLED(CONFIG_EXYNOS_SYSTEM_EVENT)
	/* System Event */
	struct sysevent_desc sysevent_desc;
	struct sysevent_device *sysevent_dev;
#endif

	/* Debug */
	struct mfc_debugfs debugfs;
	char *reg_buf;
	unsigned int *reg_val;
	unsigned int reg_cnt;
	struct mfc_meminfo meminfo[MFC_MEMINFO_DEV_ALL + 1];
	struct mfc_core_memlog memlog;
	char crash_info[MFC_CRASH_INFO_LEN];
};


/**
 *
 */
struct mfc_h264_enc_params {
	enum v4l2_mpeg_video_h264_profile profile;
	u8 level;
	u8 interlace;
	u16 open_gop_size;
	u8 open_gop;
	u8 _8x8_transform;
	s8 loop_filter_alpha;
	s8 loop_filter_beta;
	enum v4l2_mpeg_video_h264_loop_filter_mode loop_filter_mode;
	enum v4l2_mpeg_video_h264_entropy_mode entropy_mode;
	u8 rc_frame_qp;
	u8 rc_min_qp;
	u8 rc_max_qp;
	u8 rc_min_qp_p;
	u8 rc_max_qp_p;
	u8 rc_min_qp_b;
	u8 rc_max_qp_b;
	u8 rc_mb_dark;
	u8 rc_mb_smooth;
	u8 rc_mb_static;
	u8 rc_mb_activity;
	u8 rc_p_frame_qp;
	u8 rc_b_frame_qp;
	u8 ar_vui;
	u8 sei_gen_enable;
	u8 sei_fp_curr_frame_0;

	enum v4l2_mpeg_video_h264_vui_sar_idc ar_vui_idc;
	u16 ext_sar_width;
	u16 ext_sar_height;

	enum v4l2_mpeg_video_h264_hierarchical_coding_type hier_qp_type;
	u32 hier_bit_layer[7];
	u8 hier_qp_layer[7];
	u8 hier_qp_enable;
	u8 num_hier_layer;
	u8 hier_ref_type;
	u8 enable_ltr;
	u8 num_of_ltr;
	u32 set_priority;
	u32 base_priority;

	enum v4l2_mpeg_video_h264_sei_fp_arrangement_type sei_fp_arrangement_type;
	u32 fmo_enable;
	u32 fmo_slice_map_type;
	u32 fmo_slice_num_grp;
	u32 fmo_run_length[4];
	u32 fmo_sg_dir;
	u32 fmo_sg_rate;
	u32 aso_enable;
	u32 aso_slice_order[8];

	u32 prepend_sps_pps_to_idr;
	u32 vui_enable;
	u8 sub_gop_enable;
};

/**
 *
 */
struct mfc_mpeg4_enc_params {
	/* MPEG4 Only */
	enum v4l2_mpeg_video_mpeg4_profile profile;
	u8 level;
	u8 quarter_pixel;
	u8 rc_b_frame_qp;
	/* Common for MPEG4, H263 */
	u8 rc_frame_qp;
	u8 rc_min_qp;
	u8 rc_max_qp;
	u8 rc_min_qp_p;
	u8 rc_max_qp_p;
	u8 rc_min_qp_b;
	u8 rc_max_qp_b;
	u8 rc_p_frame_qp;
	u16 vop_frm_delta;
};

/**
 *
 */
struct mfc_vp9_enc_params {
	/* VP9 Only */
	u8 profile;
	u8 level;
	u8 rc_min_qp;
	u8 rc_max_qp;
	u8 rc_min_qp_p;
	u8 rc_max_qp_p;
	u8 rc_frame_qp;
	u8 rc_p_frame_qp;
	u16 vp9_gfrefreshperiod;
	u8 vp9_goldenframesel;
	u8 hier_qp_enable;
	u8 num_hier_layer;
	u8 hier_qp_layer[3];
	u32 hier_bit_layer[3];
	u8 max_partition_depth;
	u8 intra_pu_split_disable;
};

/**
 *
 */
struct mfc_vp8_enc_params {
	/* VP8 Only */
	u8 vp8_version;
	u8 rc_min_qp;
	u8 rc_max_qp;
	u8 rc_min_qp_p;
	u8 rc_max_qp_p;
	u8 rc_frame_qp;
	u8 rc_p_frame_qp;
	u8 vp8_numberofpartitions;
	u8 vp8_filterlevel;
	u8 vp8_filtersharpness;
	u16 vp8_gfrefreshperiod;
	u8 vp8_goldenframesel;
	u8 intra_4x4mode_disable;
	u8 num_hier_layer;
	u8 hier_qp_enable;
	u8 hier_qp_layer[3];
	u32 hier_bit_layer[3];
};

/**
 *
 */
struct mfc_hevc_enc_params {
	u8 profile;
	u8 level;
	u8 tier_flag;
	/* HEVC Only */
	u8 rc_min_qp;
	u8 rc_max_qp;
	u8 rc_min_qp_p;
	u8 rc_max_qp_p;
	u8 rc_min_qp_b;
	u8 rc_max_qp_b;
	u8 rc_lcu_dark;
	u8 rc_lcu_smooth;
	u8 rc_lcu_static;
	u8 rc_lcu_activity;
	u8 rc_frame_qp;
	u8 rc_p_frame_qp;
	u8 rc_b_frame_qp;
	u8 max_partition_depth;
	u8 refreshtype;
	u16 refreshperiod;
	s32 lf_beta_offset_div2;
	s32 lf_tc_offset_div2;
	u8 loopfilter_disable;
	u8 loopfilter_across;
	u8 nal_control_length_filed;
	u8 nal_control_user_ref;
	u8 nal_control_store_ref;
	u8 const_intra_period_enable;
	u8 lossless_cu_enable;
	u8 wavefront_enable;
	enum v4l2_mpeg_video_hevc_hier_coding_type hier_qp_type;
	u8 enable_ltr;
	u8 hier_qp_enable;
	u8 hier_ref_type;
	u8 num_hier_layer;
	u32 hier_bit_layer[7];
	u8 hier_qp_layer[7];
	u8 general_pb_enable;
	u8 temporal_id_enable;
	u8 strong_intra_smooth;
	u8 intra_pu_split_disable;
	u8 tmv_prediction_disable;
	u8 max_num_merge_mv;
	u8 eco_mode_enable;
	u8 encoding_nostartcode_enable;
	u8 size_of_length_field;
	u8 user_ref;
	u8 store_ref;
	u8 prepend_sps_pps_to_idr;
	u8 sub_gop_enable;
};

/**
 *
 */
struct mfc_bpg_enc_params {
	u32 thumb_size;
	u32 exif_size;
};

/**
 *
 */
struct mfc_enc_params {
	enum v4l2_mpeg_video_multi_slice_mode slice_mode;
	u32 slice_mb;
	u32 slice_bit;
	u32 slice_mb_row;

	u32 gop_ctrl;
	u32 gop_size;
	u32 intra_refresh_mb;
	u32 i_frm_ctrl_mode;
	u32 i_frm_ctrl;

	u8 pad;
	u8 pad_luma;
	u8 pad_cb;
	u8 pad_cr;

	u8 rc_mb;		/* H.264: MFCv5, MPEG4/H.263: MFCv6 */
	u8 rc_pvc;
	u8 rc_frame;
	u8 drop_control;
	u32 rc_bitrate;
	u32 rc_framerate;
	u16 rc_reaction_coeff;
	u16 rc_frame_delta;	/* MFC6.1 Only */
	u32 rc_framerate_res;

	u32 config_qp;
	u32 dynamic_qp;

	u8 frame_tag;
	u8 ratio_intra;
	u8 num_b_frame;		/* H.264, HEVC, MPEG4 */
	u8 num_refs_for_p;	/* H.264, HEVC, VP8, VP9 */
	enum v4l2_mpeg_video_header_mode seq_hdr_mode;
	enum v4l2_mpeg_mfc51_video_frame_skip_mode frame_skip_mode;
	u16 vbv_buf_size;
	u8 num_hier_max_layer;
	u8 hier_bitrate_ctrl;
	u8 weighted_enable;
	u8 roi_enable;
	u8 ivf_header_disable;	/* VP8, VP9 */
	u8 fixed_target_bit;
	u8 min_quality_mode;	/* H.264, HEVC when RC_MODE is 2(VBR) */
	u8 wp_two_pass_enable;
	u8 adaptive_gop_enable;

	u32 check_color_range;
	u32 color_range;
	u32 colour_primaries;
	u32 transfer_characteristics;
	u32 matrix_coefficients;

	u32 static_info_enable;
	u32 max_pic_average_light;
	u32 max_content_light;
	u32 max_display_luminance;
	u32 min_display_luminance;
	u32 white_point;
	u32 display_primaries_0;
	u32 display_primaries_1;
	u32 display_primaries_2;
	u32 chroma_qp_offset_cb; /* H.264, HEVC */
	u32 chroma_qp_offset_cr; /* H.264, HEVC */

	u32 mv_search_mode;
	u32 mv_hor_pos_l0;
	u32 mv_hor_pos_l1;
	u32 mv_ver_pos_l0;
	u32 mv_ver_pos_l1;
	u32 mv_hor_range;
	u32 mv_ver_range;

	u8 qpe_two_pass_enable;

	union {
		struct mfc_h264_enc_params h264;
		struct mfc_mpeg4_enc_params mpeg4;
		struct mfc_vp8_enc_params vp8;
		struct mfc_vp9_enc_params vp9;
		struct mfc_hevc_enc_params hevc;
		struct mfc_bpg_enc_params bpg;
	} codec;
};

struct mfc_ctx_ctrl_val {
	int has_new;
	int val;
};

struct mfc_ctx_ctrl {
	struct list_head list;
	enum mfc_ctrl_type type;
	unsigned int id;
	unsigned int addr;
	struct mfc_ctx_ctrl_val set;
	struct mfc_ctx_ctrl_val get;
};

struct mfc_buf_ctrl {
	struct list_head list;
	unsigned int id;
	enum mfc_ctrl_type type;
	int has_new;
	int val;
	unsigned int old_val;		/* only for MFC_CTRL_TYPE_SET */
	unsigned int old_val2;		/* only for MFC_CTRL_TYPE_SET */
	unsigned int is_volatile;	/* only for MFC_CTRL_TYPE_SET */
	unsigned int updated;
	unsigned int mode;
	unsigned int addr;
	unsigned int mask;
	unsigned int shft;
	unsigned int flag_mode;		/* only for MFC_CTRL_TYPE_SET */
	unsigned int flag_addr;		/* only for MFC_CTRL_TYPE_SET */
	unsigned int flag_shft;		/* only for MFC_CTRL_TYPE_SET */
	int (*read_cst) (struct mfc_ctx *ctx,
			struct mfc_buf_ctrl *buf_ctrl);
	void (*write_cst) (struct mfc_ctx *ctx,
			struct mfc_buf_ctrl *buf_ctrl);
};

struct mfc_ctrl_cfg {
	enum mfc_ctrl_type type;
	unsigned int id;
	unsigned int is_volatile;	/* only for MFC_CTRL_TYPE_SET */
	unsigned int mode;
	unsigned int addr;
	unsigned int mask;
	unsigned int shft;
	unsigned int flag_mode;		/* only for MFC_CTRL_TYPE_SET */
	unsigned int flag_addr;		/* only for MFC_CTRL_TYPE_SET */
	unsigned int flag_shft;		/* only for MFC_CTRL_TYPE_SET */
	int (*read_cst) (struct mfc_ctx *ctx,
			struct mfc_buf_ctrl *buf_ctrl);
	void (*write_cst) (struct mfc_ctx *ctx,
			struct mfc_buf_ctrl *buf_ctrl);
};

/* per buffer contol */
struct mfc_ctrls_ops {
	/* controls per buffer */
	int (*init_ctx_ctrls) (struct mfc_ctx *ctx);
	int (*cleanup_ctx_ctrls) (struct mfc_ctx *ctx);
	int (*init_buf_ctrls) (struct mfc_ctx *ctx,
			enum mfc_ctrl_type type, unsigned int index);
	void (*reset_buf_ctrls) (struct list_head *head);
	int (*cleanup_buf_ctrls) (struct mfc_ctx *ctx,
			enum mfc_ctrl_type type, unsigned int index);
	int (*to_buf_ctrls) (struct mfc_ctx *ctx, struct list_head *head);
	int (*to_ctx_ctrls) (struct mfc_ctx *ctx, struct list_head *head);
	int (*get_buf_ctrl_val) (struct mfc_ctx *ctx,
			struct list_head *head, unsigned int id);
	int (*get_buf_update_val) (struct mfc_ctx *ctx,
			struct list_head *head, unsigned int id, int value);
	/* new core per buffer ctrls */
	int (*core_set_buf_ctrls_val) (struct mfc_core *core,
			struct mfc_ctx *ctx, struct list_head *head);
	int (*core_get_buf_ctrls_val) (struct mfc_core *core,
			struct mfc_ctx *ctx, struct list_head *head);
	int (*core_recover_buf_ctrls_val) (struct mfc_core *core,
			struct mfc_ctx *ctx, struct list_head *head);
	int (*set_buf_ctrls_val_nal_q_dec) (struct mfc_ctx *ctx,
			struct list_head *head, DecoderInputStr *pInStr);
	int (*get_buf_ctrls_val_nal_q_dec) (struct mfc_ctx *ctx,
			struct list_head *head, DecoderOutputStr *pOutStr);
	int (*set_buf_ctrls_val_nal_q_enc) (struct mfc_ctx *ctx,
			struct list_head *head, EncoderInputStr *pInStr);
	int (*get_buf_ctrls_val_nal_q_enc) (struct mfc_ctx *ctx,
			struct list_head *head, EncoderOutputStr *pOutStr);
	int (*recover_buf_ctrls_nal_q) (struct mfc_ctx *ctx, struct list_head *head);
};

struct stored_dpb_info {
	int fd[MFC_MAX_PLANES];
};

struct dec_dpb_ref_info {
	int index;
	struct stored_dpb_info dpb[MFC_MAX_BUFFERS];
};

struct temporal_layer_info {
	unsigned int temporal_layer_count;
	unsigned int temporal_layer_bitrate[VIDEO_MAX_TEMPORAL_LAYERS];
};

struct mfc_enc_roi_info {
	char *addr;
	int size;
	int upper_qp;
	int lower_qp;
	bool enable;
};

struct mfc_user_shared_handle {
	int fd;
	struct dma_buf *dma_buf;
	void *vaddr;
	size_t data_size;
};

struct mfc_raw_info {
	int num_planes;
	int stride[3];
	int plane_size[3];
	int stride_2bits[3];
	int plane_size_2bits[3];
	unsigned int total_plane_size;
};

/* HDR10+ ST 2094 40 Metadata HEVC SEI Message */
struct hdr10_plus_meta_per_win {
	unsigned int  maxscl[HDR_MAX_SCL];
	unsigned int  average_maxrgb;
	unsigned char num_distribution_maxrgb_percentiles;
	unsigned char distribution_maxrgb_percentages[HDR_MAX_DISTRIBUTION];
	unsigned int  distribution_maxrgb_percentiles[HDR_MAX_DISTRIBUTION];
	unsigned int  fraction_bright_pixels;

	unsigned short tone_mapping_flag;
	unsigned short knee_point_x;
	unsigned short knee_point_y;
	unsigned short num_bezier_curve_anchors;
	unsigned short bezier_curve_anchors[HDR_MAX_BEZIER_CURVES];

	unsigned char color_saturation_mapping_flag;
	unsigned char color_saturation_weight;

	/*
	 * This field is reserved for ST2094-40 SEI below or the others
	 * window_upper_left_corner_x
	 * window_upper_left_corner_y
	 * window_lower_right_corner_x
	 * window_lower_right_corner_y
	 * center_of_ellipse_x
	 * center_of_ellipse_y
	 * rotation_angle
	 * semimajor_axis_internal_ellipse
	 * semimajor_axis_external_ellipse
	 * semiminor_axis_external_ellipse
	 * overlap_process_option
	 */
	unsigned int reserved[11];
};

struct hdr10_plus_meta {
	unsigned int valid;

	unsigned char  t35_country_code;
	unsigned short t35_terminal_provider_code;
	unsigned short t35_terminal_provider_oriented_code;
	unsigned char  application_identifier;
	unsigned short application_version;
	unsigned char  num_windows;

	unsigned int  target_maximum_luminance;
	unsigned char target_actual_peak_luminance_flag;
	unsigned char num_rows_target_luminance;
	unsigned char num_cols_target_luminance;

	unsigned char mastering_actual_peak_luminance_flag;
	unsigned char num_rows_mastering_luminance;
	unsigned char num_cols_mastering_luminance;

	struct hdr10_plus_meta_per_win win_info[HDR_MAX_WINDOWS];

	/*
	 * This field is reserved for ST2094-40 SEI below or the others
	 * targeted_system_display_actual_peak_luminance[rows][cols]
	 * mastering_display_actual_peak_luminance[rows][cols]
	 */
	unsigned int reserved[11];
};

struct av1_film_grain_meta {
	unsigned char apply_grain;
	unsigned short grain_seed;

	unsigned char update_grain;
	unsigned char film_grain_params_ref_idx;

	unsigned char num_y_points;
	unsigned char point_y_value[AV1_FG_LUM_POS_SIZE];
	char point_y_scaling[AV1_FG_LUM_POS_SIZE];

	char chroma_scaling_from_luma;

	unsigned char num_cb_points;
	unsigned char point_cb_value[AV1_FG_CHR_POS_SIZE];
	char point_cb_scaling[AV1_FG_CHR_POS_SIZE];

	unsigned char num_cr_points;
	unsigned char point_cr_value[AV1_FG_CHR_POS_SIZE];
	char point_cr_scaling[AV1_FG_CHR_POS_SIZE];

	unsigned char grain_scaling_minus_8;

	char ar_coeff_lag;
	char ar_coeffs_y_plus_128[AV1_FG_LUM_AR_COEF_SIZE];
	char ar_coeffs_cb_plus_128[AV1_FG_CHR_AR_COEF_SIZE];
	char ar_coeffs_cr_plus_128[AV1_FG_CHR_AR_COEF_SIZE];
	unsigned char ar_coeff_shift_minus_6;

	char grain_scale_shift;

	char cb_mult;
	char cb_luma_mult;
	short cb_offset;

	char cr_mult;
	char cr_luma_mult;
	short cr_offset;

	unsigned char overlap_flag;
	unsigned char clip_to_restricted_range;
	unsigned char mc_identity;
};

struct mfc_timestamp {
	struct list_head list;
	struct timespec64 timestamp;
	int index;
	int interval;
};

struct mfc_bitrate {
	struct list_head list;
	int bytesused;
};

struct dpb_table {
	dma_addr_t addr[MFC_MAX_PLANES];
	phys_addr_t paddr;
	size_t size;
	int fd[MFC_MAX_PLANES];
	int new_fd; /* it means first plane only */
	int mapcnt;
	int ref;
	int queued;
	struct dma_buf *dmabufs[MFC_MAX_PLANES];
	struct dma_buf_attachment *attach[MFC_MAX_PLANES];
	struct sg_table *sgt[MFC_MAX_PLANES];
};

struct disp_drc_info {
	int disp_res_change;
	int push_idx;
	int pop_idx;
	int width[MFC_MAX_DRC_FRAME];
	int height[MFC_MAX_DRC_FRAME];
};

struct mfc_dec {
	int total_dpb_count;

	unsigned int src_buf_size;

	int loop_filter_mpeg4;
	int display_delay;
	int immediate_display;
	int slice_enable;
	int mv_count;
	int idr_decoding;
	int is_interlaced;
	int is_mbaff;
	int is_dts_mode;
	int stored_tag;
	int inter_res_change;
	struct disp_drc_info disp_drc;

	int crc_enable;
	int crc_luma0;
	int crc_chroma0;
	int crc_luma1;
	int crc_chroma1;

	unsigned int consumed;
	dma_addr_t y_addr_for_pb;

	int sei_parse;

	int cr_left, cr_right, cr_top, cr_bot;

	int detect_black_bar;
	bool black_bar_updated;
	struct v4l2_rect black_bar;

	/* For dynamic DPB */
	int is_dynamic_dpb;
	int is_dpb_full;
	int display_index;
	unsigned long queued_dpb;
	unsigned long dynamic_set;
	unsigned long dynamic_used;

	int is_multiframe;
	int has_multiframe;
	int is_multiple_show;

	unsigned int num_of_tile_over_4;
	unsigned int super64_bframe;

	unsigned int color_range;
	unsigned int color_space;

	unsigned int decoding_order;
	unsigned int frame_display_delay;

	struct mfc_fmt *uncomp_fmt;

	/* for Dynamic DPB */
	struct dpb_table dpb[MFC_MAX_DPBS];
	struct mutex dpb_mutex;
	unsigned long dpb_table_used;
	struct dec_dpb_ref_info *ref_info;
	struct stored_dpb_info ref_buf[MFC_MAX_BUFFERS];
	int refcnt;
	int last_dpb_max_index;
	struct mfc_user_shared_handle sh_handle_dpb;

	/* for HDR10+ */
	struct mfc_user_shared_handle sh_handle_hdr;
	struct hdr10_plus_meta *hdr10_plus_info;
	void *hdr10_plus_full;

	/* for AV1 Film Grain meta */
	struct mfc_user_shared_handle sh_handle_av1_film_grain;
	struct av1_film_grain_meta *av1_film_grain_info;
	char av1_film_grain_info_data[128];
	int av1_film_grain_present;

	/* for debugging about black bar detection */
	void *frame_vaddr[3][30];
	dma_addr_t frame_daddr[3][30];
	int index[3][30];
	int fd[3][30];
	unsigned int frame_size[3][30];
	unsigned char frame_cnt;
};

struct mfc_enc {
	unsigned int dst_buf_size;
	unsigned int header_size;

	enum v4l2_mpeg_mfc51_video_frame_type frame_type;
	enum v4l2_mpeg_mfc51_video_force_frame_type force_frame_type;

	size_t luma_dpb_size;
	size_t chroma_dpb_size;
	size_t me_buffer_size;
	size_t tmv_buffer_size;

	unsigned int slice_mode;
	unsigned int slice_size_mb;
	unsigned int slice_size_bits;
	unsigned int in_slice;
	unsigned int buf_full;

	int config_qp;

	int sbwc_option;
	struct mfc_fmt *uncomp_fmt;

	int fake_src;
	int empty_data;

	int stored_tag;
	int roi_index;
	int is_cbr_fix;
	struct mfc_special_buf roi_buf[MFC_MAX_EXTRA_BUF];
	struct mfc_enc_roi_info roi_info[MFC_MAX_EXTRA_BUF];

	struct mfc_enc_params params;

	struct mfc_user_shared_handle sh_handle_svc;
	struct mfc_user_shared_handle sh_handle_roi;
	struct mfc_user_shared_handle sh_handle_hdr;

	bool nal_q_disable_for_qpe_two_pass;
};

struct mfc_fmt {
	char *name;
	u32 fourcc;
	u32 codec_mode;
	u32 type;
	u32 num_planes;
	u32 mem_planes;
};

/**
 * struct mfc_ctx - This struct contains the instance context
 */
struct mfc_ctx {
	struct mfc_dev *dev;
	struct mfc_dec *dec_priv;
	struct mfc_enc *enc_priv;
	struct _otf_handle *otf_handle;

	int num;
	int prio;
	enum mfc_real_time rt;

	struct mfc_fmt *src_fmt;
	struct mfc_fmt *dst_fmt;

	struct mfc_buf_queue src_buf_ready_queue;
	struct mfc_buf_queue dst_buf_queue;
	struct mfc_buf_queue dst_buf_err_queue;
	struct mfc_buf_queue src_buf_nal_queue;
	struct mfc_buf_queue dst_buf_nal_queue;
	struct mfc_buf_queue ref_buf_queue;
	spinlock_t buf_queue_lock;

	enum mfc_inst_type type;
	int subcore_inst_no;

	int img_width;
	int img_height;
	int crop_width;
	int crop_height;
	int crop_left;
	int crop_top;
	int dpb_count;
	int rgb_bpp;

	int min_dpb_size[3];
	int min_dpb_size_2bits[3];

	int bytesperline[3];
	struct mfc_raw_info raw_buf;

	enum mfc_queue_state capture_state;
	enum mfc_queue_state output_state;

	unsigned long src_ctrls_avail;
	unsigned long dst_ctrls_avail;

	unsigned int sequence;

	/* operation mode */
	int op_core_num[MFC_NUM_CORE];
	int move_core_num[MFC_NUM_CORE];
	enum mfc_op_mode op_mode;
	enum mfc_op_core_type op_core_type;
	struct mfc_core_lock corelock;
	int is_migration;
	wait_queue_head_t migrate_wq;
	int serial_src_index;
	int curr_src_index;
	struct mutex cpb_mutex;

	/* interrupt lock */
	struct mfc_core_intlock intlock;

	/* Control values */
	int codec_mode;
	__u32 pix_format;

	/* Profile infomation */
	int is_10bit;
	int is_422;

	/* SBWC */
	int is_sbwc;
	int is_sbwc_lossy;
	int sbwcl_ratio;
	int sbwc_disabled;

	/* for DRM */
	int is_drm;

	/* for 8K */
	int is_8k;

	/* for AV1 Annex B */
	int is_av1_annex_b;

	int is_dpb_realloc;
	enum mfc_dec_wait_state wait_state;
	struct mutex drc_wait_mutex;
	int clear_work_bit;

	/* Extra Buffers */
	int mv_buffer_allocated;
	int metadata_buffer_allocated;
	struct mfc_special_buf mv_buf;
	struct mfc_special_buf metadata_buf;

	unsigned long framerate;
	unsigned long last_framerate;
	unsigned long operating_framerate;
	unsigned int qos_ratio;
	bool update_framerate;
	bool update_bitrate;

	struct mfc_timestamp ts_array[MAX_TIME_INDEX];
	int ts_interval_array[MAX_TIME_INDEX];
	struct list_head ts_list;
	int ts_count;
	int ts_is_full;
	int ts_last_interval;

	/* bitrate control for QoS*/
	struct mfc_bitrate bitrate_array[MAX_TIME_INDEX];
	struct list_head bitrate_list;
	int bitrate_index;
	int bitrate_is_full;
	int Kbps;
	int last_bps_section;
	int load;
	unsigned long weighted_mb;
	struct list_head list;

	int buf_process_type;

	int frame_cnt;
	u32 last_src_addr;
	u32 last_dst_addr[MFC_MAX_PLANES];

	int batch_mode;
	bool mem_type_10bit;

	int gdc_votf;

	/* Lazy unmap disable */
	int skip_lazy_unmap;

	/* Count NAL QUEUE buffer for tracing performance */
	int nal_q_cnt;

	/* external structure */
	struct v4l2_fh fh;
	struct vb2_queue vq_src;
	struct vb2_queue vq_dst;

	/* per buffer controls */
	struct mfc_ctrls_ops *c_ops;
	struct list_head ctrls;
	struct list_head src_ctrls[MFC_MAX_BUFFERS];
	struct list_head dst_ctrls[MFC_MAX_BUFFERS];

	/* Extra Buffers size */
	size_t mv_size;
	size_t scratch_buf_size;
	size_t loopfilter_luma_size;
	size_t loopfilter_chroma_size;

	/* mem info */
	struct mfc_buf_queue	meminfo_inbuf_q;
	struct mfc_buf_queue	meminfo_outbuf_q;
	spinlock_t		meminfo_queue_lock;
	struct mfc_meminfo	meminfo[MFC_MEMINFO_MAX_NUM];
	size_t			meminfo_size[MFC_MEMINFO_CTX_MAX + 1];
};

struct mfc_core_ctx {
	struct mfc_core *core;
	struct mfc_ctx *ctx;

	int num;
	int inst_no;
	int int_condition;
	int int_reason;
	unsigned int int_err;
	bool check_dump;

	/* for DRM */
	int is_drm;

	struct mfc_buf_queue src_buf_queue;
	struct mfc_buf_queue dst_buf_queue;
	spinlock_t buf_queue_lock;
	unsigned long dynamic_set;

	enum mfc_inst_state state;

	/* QoS */
	struct list_head qos_list;

	/* Extra Buffers */
	int codec_buffer_allocated;
	int scratch_buffer_allocated;
	struct mfc_special_buf codec_buf;
	struct mfc_special_buf instance_ctx_buf;
	struct mfc_special_buf scratch_buf;

	/* wait queue */
	wait_queue_head_t cmd_wq;
	struct mfc_listable_wq hwlock_wq;
};

#endif /* __MFC_DATA_STRUCT_H */
