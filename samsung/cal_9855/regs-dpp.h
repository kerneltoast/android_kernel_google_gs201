/* SPDX-License-Identifier: GPL-2.0-only
 *
 * cal_9855/regs-dpp.h
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register definition file for Samsung Display Pre-Processor driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _REGS_DPP_9855_H
#define _REGS_DPP_9855_H

/*
 *-------------------------------------------------------------------
 * DPU_DMA: RDMA(L0~L5) SFR list
 * base address : 0x1C0B_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000
 *-------------------------------------------------------------------
 */

// new field in RDMA_IN_CTRL_0 register for 9855
#define IDMA_SBWC_LOSSLESS			(0 << 14)

// new CBP_PLANE_MO register for 9855
#define CBP_PLANE_MO				(0x0E0C)
#define CBP_OUTSTAND_P3(_v)			((_v) << 24)
#define CBP_OUTSTAND_P3_MASK			(0xFF << 0)
#define CBP_OUTSTAND_P2(_v)			((_v) << 16)
#define CBP_OUTSTAND_P2_MASK			(0xFF << 0)
#define CBP_OUTSTAND_P1(_v)			((_v) << 8)
#define CBP_OUTSTAND_P1_MASK			(0xFF << 0)
#define CBP_OUTSTAND_P0(_v)			((_v) << 0)
#define CBP_OUTSTAND_P0_MASK			(0xFF << 0)


/*
 *-------------------------------------------------------------------
 * DPU_DMA: RCD(L8~L9) SFR list
 * base address : 0x1C0B_0000
 * < Layer.offset >
 *  L8      L9
 *  0x8000  0x9000
 *-------------------------------------------------------------------
 */

#define RCD_ENABLE				(0x0000)
#define RCD_SRESET				(1 << 8)
#define RCD_SFR_UPDATE_FORCE			(1 << 4)
#define RCD_OP_STATUS				(1 << 2)
#define RCD_INST_OFF_PEND			(1 << 1)

#define RCD_IRQ					(0x0004)
#define RCD_RECOVERY_TRG_IRQ			(1 << 22)
#define RCD_CONFIG_ERR_IRQ			(1 << 21)
#define RCD_INST_OFF_DONE			(1 << 20)
#define RCD_READ_SLAVE_ERR_IRQ			(1 << 19)
#define RCD_DEADLOCK_IRQ			(1 << 17)
#define RCD_FRAME_DONE_IRQ			(1 << 16)
#define RCD_ALL_IRQ_CLEAR			(0x7B << 16)
#define RCD_RECOVERY_TRG_MASK			(1 << 7)
#define RCD_CONFIG_ERR_MASK			(1 << 6)
#define RCD_INST_OFF_DONE_MASK			(1 << 5)
#define RCD_READ_SLAVE_ERR_MASK			(1 << 4)
#define RCD_DEADLOCK_MASK			(1 << 2)
#define RCD_FRAME_DONE_MASK			(1 << 1)
#define RCD_ALL_IRQ_MASK			(0x7B << 1)
#define RCD_IRQ_ENABLE				(1 << 0)

#define RCD_IN_CTRL_0				(0x0008)
#define RCD_IC_MAX(_v)				((_v) << 16)
#define RCD_IC_MAX_MASK				(0xFF << 16)
#define RCD_BLOCK_EN				(1 << 0)
#define RCD_SRC_WIDTH				(0x0010)
#define RCD_SRC_HEIGHT				(0x0014)
#define RCD_SRC_OFFSET				(0x0018)
#define RCD_IMG_SIZE				(0x001C)
#define RCD_IMG_HEIGHT(_v)			((_v) << 16)
#define RCD_IMG_HEIGHT_MASK			(0x3FFF << 16)
#define RCD_IMG_WIDTH(_v)			((_v) << 0)
#define RCD_IMG_WIDTH_MASK			(0x3FFF << 0)

#define RCD_BLOCK_OFFSET			(0x0020)
#define RCD_BLK_OFFSET_Y(_v)			((_v) << 16)
#define RCD_BLK_OFFSET_X(_v)			((_v) << 0)
#define RCD_SRC_OFFSET_Y(_v)			((_v) << 16)
#define RCD_SRC_OFFSET_Y_MASK			(0x3FFF << 16)
#define RCD_SRC_OFFSET_X(_v)			((_v) << 0)
#define RCD_SRC_OFFSET_X_MASK			(0x3FFF << 0)
#define RCD_BLOCK_SIZE				(0x0024)
#define RCD_BLK_HEIGHT(_v)			((_v) << 16)
#define RCD_BLK_WIDTH(_v)			((_v) << 0)
#define RCD_BLOCK_VALUE				(0x0028)
#define RCD_BASEADDR_P0				(0x0040)
#define RCD_RECOVERY_CTRL			(0x0070)
#define RCD_DEADLOCK_CTRL			(0x0100)
#define RCD_DEADLOCK_NUM(_v)			((_v) << 1)
#define RCD_DEADLOCK_NUM_MASK			(0x7FFFFFFF << 1)
#define RCD_DEADLOCK_NUM_EN			(1 << 0)
#define RCD_BUS_CTRL				(0x0110)
#define RCD_LLC_CTRL				(0x0114)
#define RCD_PERF_CTRL				(0x0120)
#define RCD_QOS_LUT_LOW				(0x0130)
#define RCD_QOS_LUT_HIGH			(0x0134)
#define RCD_DYNAMIC_GATING_EN			(0x0140)
#define RCD_SRAM_CG_EN				(1 << 31)
#define RCD_DG_EN_ALL				(0x7FFFFFFF << 0)
#define RCD_MST_SECURITY			(0x0200)
#define RCD_SLV_SECURITY			(0x0204)
#define RCD_DEBUG_CTRL				(0x0300)
#define RCD_DEBUG_DATA				(0x0304)
#define RCD_IN_REQ_DEST				(0x0308)
#define RCD_PSLV_ERR_CTRL			(0x030C)


/*
 *-------------------------------------------------------------------
 * DPU_DMA: WDMA(L12) SFR list
 * base address : 0x1C0B_0000
 * < Layer.offset >
 *  L12
 *  0xC000
 *-------------------------------------------------------------------
 */

// new field in WDMA_OUT_CTRL_0 register for 9855
#define ODMA_SBWC_LOSSLESS			(0 << 14)


/*
 *-------------------------------------------------------------------
 * DPU_DMA: CGC(L14, L15) SFR List
 * base address : 0x1C0B_0000
 * < Layer.offset >
 *  L14     L15
 *  0xE000  0xF000
 *-------------------------------------------------------------------
 */

#define CGC_ENABLE				(0x0000)
#define CGC_START_SET_0				(1 << 0)
#define CGC_START_SET_0_MASK			(1 << 0)
#define CGC_IRQ					(0x0004)
#define CGC_CONFIG_ERR_IRQ			(1 << 21)
#define CGC_READ_SLAVE_ERROR			(1 << 19)
#define CGC_STATUS_DEADLOCK_IRQ			(1 << 17)
#define CGC_STATUS_FRAMEDONE_IRQ		(1 << 16)
#define CGC_ALL_IRQ_MASK			(0x2B << 1)
#define CGC_IRQ_ENABLE_MASK			(1 << 0)

#define CGC_IN_CTRL				(0x0008)
#define CGC_BASE_ADDR_SET_0			(0x0040)
#define CGC_BASE_ADDR_SET_1			(0x0044)

#define CGC_DEADLOCK_CTRL			(0x0100)
#define CGC_DEADLOCK_NUM(_v)			((_v) << 1)
#define CGC_DEADLOCK_NUM_MASK			(0x7FFFFFFF << 1)
#define CGC_DEADLOCK_NUM_EN			(1 << 0)

#define CGC_BUS_CTRL				(0x0110)
#define CGC_LLC_CTRL				(0x0114)
#define CGC_PERF_CTRL				(0x0120)
#define CGC_QOS_LUT_LOW				(0x0130)
#define CGC_QOS_LUT_HIGH			(0x0134)
#define CGC_DYNAMIC_GATIN_EN			(0x0140)
#define CGC_MST_SECURITY			(0x0200)
#define CGC_SLV_SECURITY			(0x0204)
#define CGC_DEBUG_CTRL				(0x0300)
#define CGC_DEBUG_DATA				(0x0304)
#define CGC_PSLV_ERR_CTRL			(0x030C)


/*
 *-------------------------------------------------------------------
 * DPP_COMMON(L0~L5, L12) SFR list
 * base address : 0x1C0D_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5      L12
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000  0xC000
 *-------------------------------------------------------------------
 */


/*
 *-------------------------------------------------------------------
 * DPP_HDR(L0~L5) SFR list
 * base address : 0x1C0E_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000
 *-------------------------------------------------------------------
 */

#endif /* _REGS_DPP_9855_H */
