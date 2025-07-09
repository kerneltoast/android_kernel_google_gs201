/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __LINUX_GSA_MBOX_H
#define __LINUX_GSA_MBOX_H

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

/**
 * enum gsa_mbox_cmd - mailbox commands
 */
enum gsa_mbox_cmd {
	/* Inherited ROM commands */
	GSA_MB_CMD_AUTH_IMG = 1,
	GSA_MB_CMD_LOAD_FW_IMG = 2,
	GSA_MB_CMD_GET_CHIP_ID = 3,
	GSA_MB_CMD_RESERVED_CMD_4 = 4,
	GSA_MB_CMD_DBG_GET_BOOT_CRUMBS = 5,
	GSA_MB_CMD_RESERVED_CMD_6 = 6,

	/* GSA mailbox test commands */
	GSA_MB_TEST_CMD_ECHO = 34,
	GSA_MB_TEST_CMD_RESERVED_35,
	GSA_MB_TEST_CMD_RESERVED_36,
	GSA_MB_TEST_CMD_START_UNITTEST,
	GSA_MB_TEST_CMD_RUN_UNITTEST,
	GSA_MB_TEST_CMD_UNHANDLED,

	/* TPU management */
	GSA_MB_CMD_LOAD_TPU_FW_IMG = 50,
	GSA_MB_CMD_TPU_CMD = 51,
	GSA_MB_CMD_UNLOAD_TPU_FW_IMG = 52,

	/* GSC commands */
	GSA_MB_CMD_GSC_HARD_RESET = 60,
	GSA_MB_CMD_GSC_TPM_DATAGRAM = 61,
	GSA_MB_CMD_GSC_NOS_CALL = 65,

	/* KDN */
	GSA_MB_CMD_KDN_GENERATE_KEY = 70,
	GSA_MB_CMD_KDN_EPHEMERAL_WRAP_KEY = 71,
	GSA_MB_CMD_KDN_DERIVE_RAW_SECRET = 72,
	GSA_MB_CMD_KDN_PROGRAM_KEY = 73,
	GSA_MB_CMD_KDN_RESTORE_KEYS = 74,
	GSA_MB_CMD_KDN_SET_OP_MODE = 75,

	/* AOC management */
	GSA_MB_CMD_LOAD_AOC_FW_IMG = 90,
	GSA_MB_CMD_AOC_CMD = 91,
	GSA_MB_CMD_UNLOAD_AOC_FW_IMG = 92,

	/* SJTAG */
	GSA_MB_CMD_SJTAG_GET_STATUS = 100,
	GSA_MB_CMD_SJTAG_GET_CHIP_ID = 101,
	GSA_MB_CMD_SJTAG_GET_PUB_KEY_HASH = 102,
	GSA_MB_CMD_SJTAG_SET_PUB_KEY = 103,
	GSA_MB_CMD_SJTAG_GET_CHALLENGE = 104,
	GSA_MB_CMD_SJTAG_ENABLE = 105,
	GSA_MB_CMD_SJTAG_FINISH = 106,

	/* PM commands */
	GSA_MB_CMD_WAKELOCK_ACQUIRE = 150,
	GSA_MB_CMD_WAKELOCK_RELEASE = 151,

	/* App Loading */
	GSA_MB_CMD_LOAD_APP_PKG = 180,

	/* DSP management */
	GSA_MB_CMD_LOAD_DSP_FW_IMG = 240,
	GSA_MB_CMD_DSP_CMD = 241,
	GSA_MB_CMD_UNLOAD_DSP_FW_IMG = 242,
};

/**
 * enum img_loader_args - parameter layout for image loading mbox commands
 * @IMG_LOADER_HEADER_ADDR_LO_IDX: index of image header low address parameter
 * @IMG_LOADER_HEADER_ADDR_HI_IDX: index of image header high address parameter
 * @IMG_LOADER_BODY_ADDR_LO_IDX:   index of image body low address parameter
 * @IMG_LOADER_BODY_ADDR_HI_IDX:   index of image body high address parameter
 * @IMG_LOADER_ARG_CNT:            total number of parameters
 *
 * This layout is applicable for the fallowing mailbox commands:
 *     %GSA_MB_CMD_AUTH_IMG
 *     %GSA_MB_CMD_LOAD_FW_IMG
 *     %GSA_MB_CMD_LOAD_TPU_FW_IMG
 *     %GSA_MB_CMD_LOAD_AOC_FW_IMG
 */
enum img_loader_args {
	IMG_LOADER_HEADER_ADDR_LO_IDX = 0,
	IMG_LOADER_HEADER_ADDR_HI_IDX = 1,
	IMG_LOADER_BODY_ADDR_LO_IDX = 2,
	IMG_LOADER_BODY_ADDR_HI_IDX = 3,
	IMG_LOADER_ARGC = 4,
};

enum gsc_tpm_datagram_args {
	GSC_TPM_CMD_IDX = 0,
	GSC_TPM_LEN_IDX,
	GSC_TPM_ADDR_LO_IDX,
	GSC_TPM_ADDR_HI_IDX,
	GSC_TPM_ARGC,
};

/**
 * enum kdn_data_req_args - parameters layout for KDN related calls
 * @KDN_DATA_BUF_ADDR_LO_IDX: index of low word of KDN data buffer address
 * @KDN_DATA_BUF_ADDR_HI_IDX: index of high word of KDN data buffer address
 * @KDN_DATA_BUF_SIZE_IDX: index of KDN data buffer size parameter
 * @KDN_DATA_LEN_IDX: index of KDN data length parameter
 * @KDN_OPTION_IDX: index of KDN request option parameter
 * @KDN_REQ_ARGC: total number of parameters expected by KDN service
 */
enum kdn_data_req_args {
	KDN_DATA_BUF_ADDR_LO_IDX = 0,
	KDN_DATA_BUF_ADDR_HI_IDX,
	KDN_DATA_BUF_SIZE_IDX,
	KDN_DATA_LEN_IDX,
	KDN_OPTION_IDX,
	KDN_REQ_ARGC,
};

/**
 * enum kdn_data_rsp_args - parameters layout for KDN response calls
 * @KDN_RSP_DATA_LEN_IDX: number of bytes returned in KDN data buffer by GSA
 * @KDN_RSP_ARGC: total number of parameters expected by KDN service
 */
enum kdn_data_rsp_args {
	KDN_RSP_DATA_LEN_IDX = 0,
	KDN_RSP_ARGC,
};

/**
 * enum kdn_set_mode_req_args - parameter layout for KDN set mode command
 * @KDN_SET_OP_MODE_MODE_IDX: index of operating mode parameter
 * @KDN_SET_OP_MODE_UFS_DESCR_IDX: index of UFS descriptor format parameter
 * @KDN_SET_OP_MODE_ARGC: total number of parameters
 */
enum kdn_set_op_mode_req_args {
	KDN_SET_OP_MODE_MODE_IDX = 0,
	KDN_SET_OP_MODE_UFS_DESCR_IDX = 1,
	KDN_SET_OP_MODE_ARGC,
};

/**
 * enum sjtag_data_req_args - parameters layout for SJTAG related calls
 * @SJTAG_DATA_BUF_ADDR_LO_IDX: index of low word of SJTAG data buffer address
 * @SJTAG_DATA_BUF_ADDR_HI_IDX: index of high word of SJTAG data buffer address
 * @SJTAG_DATA_BUF_SIZE_IDX: index of SJTAG data buffer size parameter
 * @SJTAG_DATA_LEN_IDX: index of SJTAG data length parameter
 * @SJTAG_DATA_REQ_ARGC: total number of parameters expected by SJTAG service
 */
enum sjtag_data_req_args {
	SJTAG_DATA_BUF_ADDR_LO_IDX = 0,
	SJTAG_DATA_BUF_ADDR_HI_IDX,
	SJTAG_DATA_BUF_SIZE_IDX,
	SJTAG_DATA_LEN_IDX,
	SJTAG_DATA_REQ_ARGC,
};

/**
 * enum sjtag_data_rsp_args - parameters layout for SJTAG response calls
 * @SJTAG_DATA_RSP_STATUS_IDX: index of status word returned by SJTAG service
 * @SJTAG_DATA_RSP_DATA_LEN_IDX: index of parameters contining number of bytes
 *                               returned in SJTAG data buffer by GSA
 * @SJTAG_DATA_RSP_DATA_ARGC: total number of parameters returned by SJTAG
 *                            data request
 */
enum sjtag_data_rsp_args {
	SJTAG_DATA_RSP_STATUS_IDX = 0,
	SJTAG_DATA_RSP_DATA_LEN_IDX,
	SJTAG_DATA_RSP_ARGC,
};

/**
 * enum sjtag_status_rsp_args - parameter layout for SJTAG get status command
 * @SJTAG_STATUS_RSP_DEBUG_ALLOWED_IDX: Index of debug allowed parameter
 * @SJTAG_STATUS_RSP_HW_STATUS_IDX: index of hardware status parameter
 * @SJTAG_STATUS_RSP_DEBUG_TIME_IDX: index of DEBUG time parameter
 */
enum sjtag_status_rsp_args {
	SJTAG_STATUS_RSP_DEBUG_ALLOWED_IDX = 0,
	SJTAG_STATUS_RSP_HW_STATUS_IDX = 1,
	SJTAG_STATUS_RSP_DEBUG_TIME_IDX = 2,
	SJTAG_STATUS_RSP_ARGC,
};

/**
 * enum app_pkg_load_req_args - parameters layout for APP package request
 * @APP_PKG_ADDR_LO_IDX: index of low word of APP package address
 * @APP_PKG_ADDR_HI_IDX: index of high word of APP package address
 * @APP_PKG_SIZE_IDX: index of App package size parameter
 * @LOAD_APP_REQ_ARGC: total number of parameters expected by apploader
 *                     service
 */
enum app_pkg_load_req_args {
	APP_PKG_ADDR_LO_IDX = 0,
	APP_PKG_ADDR_HI_IDX,
	APP_PKG_SIZE_IDX,
	APP_PKG_LOAD_REQ_ARGC,
};

struct gsa_mbox;

struct gsa_mbox *gsa_mbox_init(struct platform_device *pdev);

int gsa_send_mbox_cmd(struct gsa_mbox *mb, u32 cmd,
		      u32 *req_args, u32 req_argc,
		      u32 *rsp_args, u32 rsp_argc);

#endif /* __LINUX_GSA_MBOX_H */
