/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License Version 2 as published
 * by the Free Software Foundation.
 *
 * Header for BTS Bus Traffic Shaper
 *
 * Includes Register information to control BTS devices
 *
 */

#ifndef __REGS_BTSGS101_H__
#define __REGS_BTSGS101_H__

#define CON 0x000
#define TIMEOUT 0x010
#define RCON 0x020
#define RBLK_UPPER 0x024
#define RBLK_UPPER_FULL 0x02C
#define RBLK_UPPER_BUSY 0x030
#define RBLK_UPPER_MAX 0x034
#define WCON 0x040
#define WBLK_UPPER 0x044
#define WBLK_UPPER_FULL 0x04C
#define WBLK_UPPER_BUSY 0x050
#define WBLK_UPPER_MAX 0x054
#define CORE_QOS_EN 0x4
#define TIMEOUT_R0  0x008
#define TIMEOUT_R1  0x00C
#define TIMEOUT_W0  0x010
#define TIMEOUT_W1  0x014

#define AXQOS_BYPASS 8
#define AXQOS_VAL 12

#define SCIQOS_EN 0
#define SCIQOS_R 2
#define SCIQOS_W 0

#define AXQOS_ONOFF 0
#define BLOCK_UPPER 0
#define BLOCK_UPPER0 0
#define BLOCK_UPPER1 16
#define TIMEOUT_CNT_R 0
#define TIMEOUT_CNT_W 16
#define QURGENT_EN 23
#define EX_QURGENT_EN 7
#define BLOCKING_EN 0
#define TIMEOUT_CNT_VC0 0
#define TIMEOUT_CNT_VC1 8
#define TIMEOUT_CNT_VC2 16
#define TIMEOUT_CNT_VC3 24

#define RMO_PORT_0 0
#define RMO_PORT_1 16
#define WMO_PORT_0 8
#define WMO_PORT_1 24

#define SCI_CTRL 0x0000
#define CRP_CTL3_0 0x10
#define CRP_CTL3_1 0x38
#define CRP_CTL3_2 0x60
#define CRP_CTL3_3 0x88
#define TH_IMM_R_0 0x0100
#define TH_IMM_W_0 0x0180
#define TH_HIGH_R_0 0x0200
#define TH_HIGH_W_0 0x0280

#define HIGH_THRESHOLD_SHIFT 24
#define MID_THRESHOLD_SHIFT 16

#define DEFAULT_QBUSY_TH 0x4

#define SMC_SCHEDCTL_BUNDLE_CTRL4 0x0

#define DEFAULT_QMAX_RD_TH 0x60
#define DEFAULT_QMAX_WR_TH 0x30

#define QMAX_THRESHOLD_R 0x0050
#define QMAX_THRESHOLD_W 0x0054

#endif /* __REGS_BTSGS101_H__ */
