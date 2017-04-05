/*
 * linux/drivers/gpu/exynos/g2d/g2d_regs.h
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * Register Definitions for Samsung Graphics 2D Hardware
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __G2D_REGS_H__
#define __G2D_REGS_H__

/* General Registers */
#define G2D_SOFT_RESET_REG			0x000
#define G2D_INTEN_REG				0x004
#define G2D_INTC_PEND_REG			0x00c
#define G2D_FIFO_STAT_REG			0x010
#define G2D_VERSION_INFO_REG			0x014
#define G2D_AXI_MODE_REG			0x01c
#define G2D_COMP_DEBUG_ADDR_REG			0x0f0
#define G2D_COMP_DEBUG_DATA_REG			0x0f4

/* Job Manager Registers */
#define G2D_JOB_INT_ID_REG			0x09C
#define G2D_JOB_HEADER_REG			0x080
#define G2D_JOB_BASEADDR_REG			0x084
#define G2D_JOB_SFRNUM_REG			0x088
#define G2D_JOB_PUSH_REG			0x08C
#define G2D_JOB_KILL_REG			0x090
#define G2D_JOB_PUSHKILL_STATE_REG		0x094
#define G2D_JOB_EMPTYSLOT_NUM_REG		0x098
#define G2D_JOB_ID_REG				0x009
#define G2D_JOB_ID0_STATE_REG			0x0A0
#define G2D_JOB_IDn_STATE_REG(n)	(G2D_JOB_ID0_STATE_REG + ((n) * 0x4))

/* G2D command Registers */
#define G2D_BITBLT_START_REG			0x100
#define G2D_BITBLT_COMMAND_REG			0x104
#define G2D_LAYER_UPDATE_REG			0x108

/* HWFC related Registers */
#define G2D_HWFC_CAPTURE_IDX_REG		0x8000
#define G2D_HWFC_ENCODING_IDX_REG		0x8004

/* Fields of G2D_INTEN_REG */
#define G2D_BLIT_INT_ENABLE			((1 << 0) | (7 << 16))
#define G2D_ERR_INT_ENABLE			(7 << 16)

/* Fields of G2D_INTC_PEND_REG */
#define G2D_BLIT_INT_FLAG			((1 << 0) | (7 << 16))
#define G2D_ERR_INT_FLAG			(7 << 16)

/* Fields of G2D_BITBLT_START_REG */
#define G2D_START_BITBLT			(1 << 0)

/* Fields of G2D_JOB_HEADER_REG */
#define G2D_JOB_HEADER_DATA(p, id)	((((p) & 0x3) << 4) | ((id) & 0xF))

/* Fields of G2D_JOB_PUSH_REG */
#define G2D_JOBPUSH_INT_ENABLE			0x1

/* Fields of G2D_JOB_IDn_STATE_REG */
#define G2D_JOB_STATE_DONE			0x0
#define G2D_JOB_STATE_QUEUEING			0x1
#define G2D_JOB_STATE_SUSPENDING		0x2
#define G2D_JOB_STATE_RUNNING			0x3
#define G2D_JOB_STATE_MASK			0x3

/* Fields of G2D_SOFT_RESET_REG */
#define G2D_SFR_CLEAR				(1 << 2)
#define G2D_GLOBAL_RESET			(1 << 1)
#define G2D_SOFT_RESET				(1 << 0)

#endif /* __G2D_REGS_H__ */
