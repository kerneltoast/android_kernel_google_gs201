/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __LINUX_HWMGR_IPC_H
#define __LINUX_HWMGR_IPC_H

#include <linux/types.h>

#define HWMGR_TPU_PORT "com.android.trusty.gsa.hwmgr.tpu"
#define HWMGR_AOC_PORT "com.android.trusty.gsa.hwmgr.aoc"
#define HWMGR_DSP_PORT "com.android.trusty.gsa.hwmgr.dsp"

enum hwmgr_cmd {
    HWMGR_CMD_RESP = (1U << 31),
    HWMGH_CMD_RESERVED = 0,
    HWMGR_CMD_STATE_CMD,
    HWMGR_CMD_UNLOAD_IMG,
    HWMGR_CMD_GET_IMG_CONFIG,
};

struct hwmgr_req_hdr {
    u32 cmd;
};

struct hwmgr_rsp_hdr {
    u32 cmd;
    u32 err;
};

struct hwmgr_state_cmd_req {
    u32 cmd;
};

struct hwmgr_state_cmd_rsp {
    u32 state;
};

#endif

