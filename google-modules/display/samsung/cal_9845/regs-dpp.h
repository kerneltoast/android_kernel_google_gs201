/* SPDX-License-Identifier: GPL-2.0-only
 *
 * cal_9845/regs-dpp.h
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
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

#ifndef _REGS_DPP_H
#define _REGS_DPP_H

/*
 *-------------------------------------------------------------------
 * DPU_DMA: RDMA(L0~L5) SFR list
 * base address : 0x1C0B_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000
 *-------------------------------------------------------------------
 */

/* SHADOW: 0x400 ~ 0x800 */
#define DMA_SHD_OFFSET				(0x0400)

#define RDMA_ENABLE				(0x0000)
#define IDMA_ASSIGNED_MO(_v)			((_v) << 24)
#define IDMA_ASSIGNED_MO_MASK			(0xFF << 24)
#define IDMA_SRESET				(1 << 8)
#define IDMA_SFR_UPDATE_FORCE			(1 << 4)
#define IDMA_OP_STATUS				(1 << 2)
#define OP_STATUS_IDLE				(0)
#define OP_STATUS_BUSY				(1)
#define IDMA_INST_OFF_PEND			(1 << 1)
#define INST_OFF_PEND				(1)
#define INST_OFF_NOT_PEND			(0)

#define RDMA_IRQ				(0x0004)
#define IDMA_AXI_ADDR_ERR_IRQ			(1 << 26)
#define IDMA_LB_CONFLICT_IRQ			(1 << 25)
#define IDMA_MO_CONFLICT_IRQ			(1 << 24)
#define IDMA_FBC_ERR_IRQ			(1 << 23)
#define IDMA_RECOVERY_TRG_IRQ			(1 << 22)
#define IDMA_CONFIG_ERR_IRQ			(1 << 21)
#define IDMA_INST_OFF_DONE			(1 << 20)
#define IDMA_READ_SLAVE_ERR_IRQ			(1 << 19)
#define IDMA_DEADLOCK_IRQ			(1 << 17)
#define IDMA_FRAME_DONE_IRQ			(1 << 16)
#define IDMA_ALL_IRQ_CLEAR			(0x7FB << 16)
#define IDMA_AXI_ADDR_ERR_IRQ_MASK		(1 << 11)
#define IDMA_LB_CONFLICT_MASK			(1 << 10)
#define IDMA_MO_CONFLICT_MASK			(1 << 9)
#define IDMA_SBWC_ERR_MASK			(1 << 8)
#define IDMA_RECOVERY_TRG_MASK			(1 << 7)
#define IDMA_CONFIG_ERR_MASK			(1 << 6)
#define IDMA_INST_OFF_DONE_MASK			(1 << 5)
#define IDMA_READ_SLAVE_ERR_MASK		(1 << 4)
#define IDMA_DEADLOCK_MASK			(1 << 2)
#define IDMA_FRAME_DONE_MASK			(1 << 1)
#define IDMA_ALL_IRQ_MASK			(0x7FB << 1)
#define IDMA_IRQ_ENABLE				(1 << 0)


/* defined for common part of driver only */
#define IDMA_RECOVERY_START_IRQ			IDMA_RECOVERY_TRG_IRQ
#define IDMA_READ_SLAVE_ERROR			IDMA_READ_SLAVE_ERR_IRQ
#define IDMA_STATUS_DEADLOCK_IRQ		IDMA_DEADLOCK_IRQ
#define IDMA_STATUS_FRAMEDONE_IRQ		IDMA_FRAME_DONE_IRQ
#define IDMA_AFBC_CONFLICT_IRQ			IDMA_LB_CONFLICT_IRQ

#define RDMA_IN_CTRL_0				(0x0008)
#define IDMA_ALPHA(_v)				((_v) << 24)
#define IDMA_ALPHA_MASK				(0xFF << 24)
#define IDMA_IC_MAX(_v)				((_v) << 16)
#define IDMA_IC_MAX_MASK			(0xFF << 16)
#define IDMA_SBWC_LOSSY				(1 << 14)
#define IDMA_IMG_FORMAT(_v)			((_v) << 8)
#define IDMA_IMG_FORMAT_MASK			(0x3F << 8)

/*
 * Below RGB format name is different from user's manual.
 * Big endian is used for RGB naming in user's manual.
 * However, SFR definition related to RGB format uses
 * little endian because HAL_PIXEL format and drm fourcc use
 * little endian.
 */
#define IDMA_IMG_FORMAT_BGRA8888		(0)
#define IDMA_IMG_FORMAT_RGBA8888		(1)
#define IDMA_IMG_FORMAT_ABGR8888		(2)
#define IDMA_IMG_FORMAT_ARGB8888		(3)
#define IDMA_IMG_FORMAT_BGRX8888		(4)
#define IDMA_IMG_FORMAT_RGBX8888		(5)
#define IDMA_IMG_FORMAT_XBGR8888		(6)
#define IDMA_IMG_FORMAT_XRGB8888		(7)
#define IDMA_IMG_FORMAT_BGR565			(8)
#define IDMA_IMG_FORMAT_RGB565			(9)
#define IDMA_IMG_FORMAT_BGRA1010102		(16)
#define IDMA_IMG_FORMAT_RGBA1010102		(17)
#define IDMA_IMG_FORMAT_ABGR2101010		(18)
#define IDMA_IMG_FORMAT_ARGB2101010		(19)

#define IDMA_IMG_FORMAT_NV21			(24)
#define IDMA_IMG_FORMAT_NV12			(25)
#define IDMA_IMG_FORMAT_YVU420_8P2		(26)
#define IDMA_IMG_FORMAT_YUV420_8P2		(27)
#define IDMA_IMG_FORMAT_YVU420_P010		(28)
#define IDMA_IMG_FORMAT_YUV420_P010		(29)

#define IDMA_IMG_FORMAT_NV61			(56)
#define IDMA_IMG_FORMAT_NV16			(57)
#define IDMA_IMG_FORMAT_YVU422_8P2		(58)
#define IDMA_IMG_FORMAT_YUV422_8P2		(59)
#define IDMA_IMG_FORMAT_YVU422_P210		(60)
#define IDMA_IMG_FORMAT_YUV422_P210		(61)
#define IDMA_ROT(_v)				((_v) << 4)
#define IDMA_ROT_MASK				(0x7 << 4)
#define IDMA_ROT_X_FLIP				(1 << 4)
#define IDMA_ROT_Y_FLIP				(2 << 4)
#define IDMA_ROT_180				(3 << 4)
#define IDMA_ROT_90				(4 << 4)
#define IDMA_ROT_90_X_FLIP			(5 << 4)
#define IDMA_ROT_90_Y_FLIP			(6 << 4)
#define IDMA_ROT_270				(7 << 4)
#define IDMA_FLIP(_v)				((_v) << 4)
#define IDMA_FLIP_MASK				(0x3 << 4)
#define IDMA_CSET_EN				(1 << 3) // must keep as '0'
#define IDMA_SBWC_EN				(1 << 2)
#define IDMA_AFBC_EN				(1 << 1)
#define IDMA_BLOCK_EN				(1 << 0)

#define RDMA_IN_CTRL_1				(0x000C)
#define IDMA_SPLIT_8K_LR			(1 << 1)
#define IDMA_SPLIT_8K_EN			(1 << 0)

#define RDMA_SRC_SIZE				(0x0010)
#define IDMA_SRC_HEIGHT(_v)			((_v) << 16)
#define IDMA_SRC_HEIGHT_MASK			(0xFFFF << 16)
#define IDMA_SRC_WIDTH(_v)			((_v) << 0)
#define IDMA_SRC_WIDTH_MASK			(0xFFFF << 0)

#define RDMA_SRC_OFFSET				(0x0014)
#define IDMA_SRC_OFFSET_Y(_v)			((_v) << 16)
#define IDMA_SRC_OFFSET_Y_MASK			(0x3FFF << 16)
#define IDMA_SRC_OFFSET_X(_v)			((_v) << 0)
#define IDMA_SRC_OFFSET_X_MASK			(0x3FFF << 0)

#define RDMA_IMG_SIZE				(0x0018)
#define IDMA_IMG_HEIGHT(_v)			((_v) << 16)
#define IDMA_IMG_HEIGHT_MASK			(0x3FFF << 16)
#define IDMA_IMG_WIDTH(_v)			((_v) << 0)
#define IDMA_IMG_WIDTH_MASK			(0x3FFF << 0)

#define RDMA_BLOCK_OFFSET			(0x0020)
#define IDMA_BLK_OFFSET_Y(_v)			((_v) << 16)
#define IDMA_BLK_OFFSET_Y_MASK			(0x3FFF << 16)
#define IDMA_BLK_OFFSET_X(_v)			((_v) << 0)
#define IDMA_BLK_OFFSET_X_MASK			(0x3FFF << 0)

#define RDMA_BLOCK_SIZE				(0x0024)
#define IDMA_BLK_HEIGHT(_v)			((_v) << 16)
#define IDMA_BLK_HEIGHT_MASK			(0x3FFF << 16)
#define IDMA_BLK_WIDTH(_v)			((_v) << 0)
#define IDMA_BLK_WIDTH_MASK			(0x3FFF << 0)

#define RDMA_BASEADDR_Y8			(0x0040)
#define RDMA_BASEADDR_C8			(0x0044)
#define RDMA_BASEADDR_Y2			(0x0048)
#define RDMA_BASEADDR_C2			(0x004C)

#define RDMA_SRC_STRIDE_0			(0x0050)
#define IDMA_STRIDE_3_SEL			(1 << 23)
#define IDMA_STRIDE_2_SEL			(1 << 22)
#define IDMA_STRIDE_1_SEL			(1 << 21)
#define IDMA_STRIDE_0_SEL			(1 << 20)
#define IDMA_STRIDE_SEL(_v)			((_v) << 20)
#define IDMA_STRIDE_SEL_MASK			(0xF << 20)
#define IDMA_CHROM_STRIDE_SEL			(1 << 16)
#define IDMA_CHROM_STRIDE(_v)			((_v) << 0)
#define IDMA_CHROM_STRIDE_MASK			(0xFFFF << 0)

#define RDMA_SRC_STRIDE_1			(0x0054)
#define IDMA_STRIDE_1(_v)			((_v) << 16)
#define IDMA_STRIDE_1_MASK			(0xFFFF << 16)
#define IDMA_STRIDE_0(_v)			((_v) << 0)
#define IDMA_STRIDE_0_MASK			(0xFFFF << 0)

#define RDMA_SRC_STRIDE_2			(0x0058)
#define IDMA_STRIDE_3(_v)			((_v) << 16)
#define IDMA_STRIDE_3_MASK			(0xFFFF << 16)
#define IDMA_STRIDE_2(_v)			((_v) << 0)
#define IDMA_STRIDE_2_MASK			(0xFFFF << 0)

#define RDMA_AFBC_PARAM				(0x0060)
/* [128 x n] 3: 384 byte */
#define IDMA_AFBC_BLK_BYTENUM(_v)		((_v) << 4)
#define IDMA_AFBC_BLK_BYTENUM_MASK		(0xF << 4)
#define IDMA_AFBC_BLK_SIZE(_v)			((_v) << 0)
#define IDMA_AFBC_BLK_SIZE_MASK			(0x3 << 0)
#define IDMA_AFBC_BLK_SIZE_16_16		(0)
#define IDMA_AFBC_BLK_SIZE_32_8			(1)
#define IDMA_AFBC_BLK_SIZE_64_4			(2)

#define RDMA_SBWC_PARAM				(0x0064)
#define IDMA_CRC_EN				(1 << 16)
/* [32 x n] 2: 64 byte */
#define IDMA_CHM_BLK_BYTENUM(_v)		((_v) << 8)
#define IDMA_CHM_BLK_BYTENUM_MASK		(0xF << 8)
#define IDMA_LUM_BLK_BYTENUM(_v)		((_v) << 4)
#define IDMA_LUM_BLK_BYTENUM_MASK		(0xF << 4)
/* only valid 32x4 */
#define IDMA_CHM_BLK_SIZE(_v)			((_v) << 2)
#define IDMA_CHM_BLK_SIZE_MASK			(0x3 << 2)
#define IDMA_CHM_BLK_SIZE_32_4			(0)
#define IDMA_LUM_BLK_SIZE(_v)			((_v) << 0)
#define IDMA_LUM_BLK_SIZE_MASK			(0x3 << 0)
#define IDMA_CHM_BLK_SIZE_32_4			(0)

#define RDMA_CSET_PARAM				(0x0068)
#define IDMA_GB_BASE(_v)			((_v) << 0)
#define IDMA_GB_BASE_MASK			(0x1F << 0)

#define RDMA_RECOVERY_CTRL			(0x0070)
#define IDMA_RECOVERY_NUM(_v)			((_v) << 1)
#define IDMA_RECOVERY_NUM_MASK			(0x7FFFFFFF << 1)
#define IDMA_RECOVERY_EN			(1 << 0)

#define RDMA_DEADLOCK_CTRL			(0x0100)
#define IDMA_DEADLOCK_NUM(_v)			((_v) << 1)
#define IDMA_DEADLOCK_NUM_MASK			(0x7FFFFFFF << 1)
#define IDMA_DEADLOCK_NUM_EN			(1 << 0)

#define RDMA_BUS_CTRL				(0x0110)
#define IDMA_ARCACHE_P3				((_v) << 12)
#define IDMA_ARCACHE_P3_MASK			(0xF << 12)
#define IDMA_ARCACHE_P2				((_v) << 8)
#define IDMA_ARCACHE_P2_MASK			(0xF << 8)
#define IDMA_ARCACHE_P1				((_v) << 4)
#define IDMA_ARCACHE_P1_MASK			(0xF << 4)
#define IDMA_ARCACHE_P0				((_v) << 0)
#define IDMA_ARCACHE_P0_MASK			(0xF << 0)

#define RDMA_LLC_CTRL				(0x0114)
#define IDMA_DATA_SAHRE_TYPE_P3(_v)		((_v) << 28)
#define IDMA_DATA_SAHRE_TYPE_P3_MASK		(0x3 << 28)
#define IDMA_LLC_HINT_P3(_v)			((_v) << 24)
#define IDMA_LLC_HINT_P3_MASK			(0x7 << 24)
#define IDMA_DATA_SAHRE_TYPE_P2(_v)		((_v) << 20)
#define IDMA_DATA_SAHRE_TYPE_P2_MASK		(0x3 << 20)
#define IDMA_LLC_HINT_P2(_v)			((_v) << 16)
#define IDMA_LLC_HINT_P2_MASK			(0x7 << 16)
#define IDMA_DATA_SAHRE_TYPE_P1(_v)		((_v) << 12)
#define IDMA_DATA_SAHRE_TYPE_P1_MASK		(0x3 << 12)
#define IDMA_LLC_HINT_P1(_v)			((_v) << 8)
#define IDMA_LLC_HINT_P1_MASK			(0x7 << 8)
#define IDMA_DATA_SAHRE_TYPE_P0(_v)		((_v) << 4)
#define IDMA_DATA_SAHRE_TYPE_P0_MASK		(0x3 << 4)
#define IDMA_LLC_HINT_P0(_v)			((_v) << 0)
#define IDMA_LLC_HINT_P0_MASK			(0x7 << 0)

#define RDMA_PERF_CTRL				(0x0120)
#define IDMA_DEGRADATION_TIME(_v)		((_v) << 16)
#define IDMA_DEGRADATION_TIME_MASK		(0xFFFF << 16)
#define IDMA_IC_MAX_DEG(_v)			((_v) << 4)
#define IDMA_IC_MAX_DEG_MASK			(0xFF << 4)
#define IDMA_DEGRADATION_EN			(1 << 0)

/* _n: [0,7], _v: [0x0, 0xF] */
#define RDMA_QOS_LUT_LOW			(0x0130)
#define RDMA_QOS_LUT_HIGH			(0x0134)
#define IDMA_QOS_LUT(_n, _v)			((_v) << (4*(_n)))
#define IDMA_QOS_LUT_MASK(_n)			(0xF << (4*(_n)))

#define RDMA_DYNAMIC_GATING_EN			(0x0140)
#define IDMA_SRAM_CG_EN				(1 << 31)
#define IDMA_DG_EN(_n, _v)			((_v) << (_n))
#define IDMA_DG_EN_MASK(_n)			(1 << (_n))
#define IDMA_DG_EN_ALL				(0x7FFFFFFF << 0)

#define RDMA_MST_SECURITY			(0x0200)
#define RDMA_SLV_SECURITY			(0x0204)

#define RDMA_DEBUG_CTRL				(0x0300)
#define IDMA_DEBUG_SEL(_v)			((_v) << 16)
#define IDMA_DEBUG_EN				(0x1 << 0)

#define RDMA_DEBUG_DATA				(0x0304)

/* 0: AXI, 3: Pattern */
#define RDMA_IN_REQ_DEST			(0x0308)
#define IDMA_REQ_DEST_SEL(_v)			((_v) << 0)
#define IDMA_REQ_DEST_SEL_MASK			(0x3 << 0)

#define RDMA_PSLV_ERR_CTRL			(0x030c)
#define IDMA_PSLVERR_CTRL			(1 << 0)

#define RDMA_DEBUG_ADDR_Y8			(0x0310)
#define RDMA_DEBUG_ADDR_C8			(0x0314)
#define RDMA_DEBUG_ADDR_Y2			(0x0318)
#define RDMA_DEBUG_ADDR_C2			(0x031C)

#define RDMA_DEBUG_ADDR_CTRL			(0x0320)
#define IDMA_DEBUG_EN_ADDR_C2			(1 << 3)
#define IDMA_DEBUG_EN_ADDR_Y2			(1 << 2)
#define IDMA_DEBUG_EN_ADDR_C8			(1 << 1)
#define IDMA_DEBUG_EN_ADDR_Y8			(1 << 0)


#define RDMA_DEBUG_ADDR_ERR			(0x0730)
#define IDMA_ERR_ADDR_C2			(1 << 3)
#define IDMA_ERR_ADDR_Y2			(1 << 2)
#define IDMA_ERR_ADDR_C8			(1 << 1)
#define IDMA_ERR_ADDR_Y8			(1 << 0)
#define IDMA_ERR_ADDR_GET(_v)			(((_v) >> 0) & 0xF)

#define RDMA_CONFIG_ERR_STATUS			(0x0740)
#define IDMA_CFG_ERR_ROTATION			(1 << 21)
#define IDMA_CFG_ERR_IMG_HEIGHT_ROTATION	(1 << 20)
#define IDMA_CFG_ERR_AFBC			(1 << 18)
#define IDMA_CFG_ERR_SBWC			(1 << 17)
#define IDMA_CFG_ERR_BLOCK			(1 << 16)
#define IDMA_CFG_ERR_FORMAT			(1 << 15)
#define IDMA_CFG_ERR_STRIDE3			(1 << 14)
#define IDMA_CFG_ERR_STRIDE2			(1 << 13)
#define IDMA_CFG_ERR_STRIDE1			(1 << 12)
#define IDMA_CFG_ERR_STRIDE0			(1 << 11)
#define IDMA_CFG_ERR_CHROM_STRIDE		(1 << 10)
#define IDMA_CFG_ERR_BASE_ADDR_C2		(1 << 9)
#define IDMA_CFG_ERR_BASE_ADDR_Y2		(1 << 8)
#define IDMA_CFG_ERR_BASE_ADDR_C8		(1 << 7)
#define IDMA_CFG_ERR_BASE_ADDR_Y8		(1 << 6)
#define IDMA_CFG_ERR_SRC_OFFSET_Y		(1 << 5)
#define IDMA_CFG_ERR_SRC_OFFSET_X		(1 << 4)
#define IDMA_CFG_ERR_IMG_HEIGHT			(1 << 3)
#define IDMA_CFG_ERR_IMG_WIDTH			(1 << 2)
#define IDMA_CFG_ERR_SRC_HEIGHT			(1 << 1)
#define IDMA_CFG_ERR_SRC_WIDTH			(1 << 0)
#define IDMA_CFG_ERR_GET(_v)			(((_v) >> 0) & 0x3FFFFF)

/*
 * NEW : 0x0D00 ~ 0x0D28
 * SHADOW_OFFSET : 0x0D80
 */
#define RDMA_BWL_ENABLE				(0x0D00)
#define IDMA_LIMIT_EN_HDR			(1 << 4)
#define IDMA_LIMIT_EN_PLD			(1 << 0)

/* number of cycles in 1us : ACLK base */
#define RDMA_BWL_FREQ				(0x0D04)
#define IDMA_NUM_CYCLES(_v)			((_v) << 0)
#define IDMA_NUM_CYCLES_MASK			(0xFFF << 0)

#define RDMA_BWL_LIMIT_CTRL_PLD			(0x0D10)
#define IDMA_AVG_BW_LIMIT(_v)			((_v) << 16)
#define IDMA_AVG_BW_LIMIT_MASK			(0xFFFF << 16)
#define IDMA_SLOT_BW_LIMIT(_v)			((_v) << 0)
#define IDMA_SLOT_BW_LIMIT_MASK			(0xFFFF << 0)

#define RDMA_BWL_COMPENSATE_CTRL_PLD		(0x0D14)
#define IDMA_COMPENSATION_LIMIT(_v)		((_v) << 16)
#define IDMA_COMPENSATION_LIMIT_MASK		(0xFFFF << 16)
#define IDMA_PERIOD(_v)				((_v) << 0)
#define IDMA_PERIOD_MASK			(0xFFF << 0)

#define RDMA_BWL_LIMIT_CTRL_HDR			(0x0D20)
/* same field with RDMA_BWL_LIMIT_CTRL_PLD */

#define RDMA_BWL_COMPENSATE_CTRL_HDR		(0x0D24)
/* same field with RDMA_BWL_COMPENSATE_CTRL_PLD */


/* L0,2,4 only : Global is moved to one layer of each port */
#define CBP_ENABLE				(0x0E00)
#define CBP_PREFETCH_ON				(1 << 0)

#define CBP_SIZE				(0x0E04)
#define CBP_PREFETCH_SIZE(_v)			((_v) << 0)
#define CBP_PREFETCH_SIZE_MASK			(0x1F << 0)

#define CBP_DIST				(0x0E08)
#define CBP_DISTANCE(_v)			((_v) << 0)
#define CBP_DISTANCE_MASK			(0xFFFF << 0)

#define CBP_TOTAL_MO				(0x0E10)
#define CBP_OUTSTAND_TOTAL(_v)			((_v) << 0)
#define CBP_OUTSTAND_TOTAL_MASK			(0xFF << 0)


#define GLB_DPU_DMA_VERSION			(0x0F00)
#define GLB_DPU_DMA_VERSION_GET(_v)		(((_v) >> 0) & 0xffffffff)

#define GLB_QCH_EN				(0x0F0C)
#define GLB_DPU_QCH_EN				(1 << 0)

#define GLB_SWRST				(0x0F10)
#define GLB_DPU_ALL_SWRST			(1 << 0)

#define GLB_GLB_CGEN				(0x0F14)
#define GLB_DPU_INT_CGEN(_v)			((_v) << 0)
#define GLB_DPU_INT_CGEN_MASK			(0x7FFFFFFF << 0)

#define GLB_TEST_PATTERN0_3			(0x0F20)
#define GLB_TEST_PATTERN0_2			(0x0F24)
#define GLB_TEST_PATTERN0_1			(0x0F28)
#define GLB_TEST_PATTERN0_0			(0x0F2C)
#define GLB_TEST_PATTERN1_3			(0x0F30)
#define GLB_TEST_PATTERN1_2			(0x0F34)
#define GLB_TEST_PATTERN1_1			(0x0F38)
#define GLB_TEST_PATTERN1_0			(0x0F3C)

#define GLB_DEBUG_CTRL				(0x0F80)
#define GLB_DEBUG_SEL(_v)			((_v) << 16)
#define GLB_DEBUG_EN				(1 << 0)

#define GLB_DEBUG_DATA				(0x0F84)


/*
 *-------------------------------------------------------------------
 * DPU_DMA: WDMA(L12) SFR list
 * base address : 0x1C0B_0000
 * < Layer.offset >
 *  L12
 *  0xC000
 *-------------------------------------------------------------------
 */
#define WDMA_ENABLE				(0x0000)
#define ODMA_SRESET				(1 << 24)
#define ODMA_SHD_UPDATE_FORCE			(1 << 4)
#define ODMA_OP_STATUS				(1 << 2)
#define ODMA_INST_OFF_PEND			(1 << 1)

#define WDMA_IRQ				(0x0004)
#define ODMA_CONFIG_ERR_IRQ			(1 << 28)
#define ODMA_INST_OFF_DONE_IRQ			(1 << 20)
#define ODMA_WRITE_SLAVE_ERR_IRQ		(1 << 19)
#define ODMA_DEADLOCK_IRQ			(1 << 17)
#define ODMA_FRAME_DONE_IRQ			(1 << 16)
#define ODMA_ALL_IRQ_CLEAR			(0x101B << 16)

/* defined for common part of driver only */
#define ODMA_WRITE_SLAVE_ERROR			ODMA_WRITE_SLAVE_ERR_IRQ
#define ODMA_STATUS_FRAMEDONE_IRQ		ODMA_FRAME_DONE_IRQ
#define ODMA_STATUS_DEADLOCK_IRQ		ODMA_DEADLOCK_IRQ

#define ODMA_CONFIG_ERR_MASK			(1 << 13)
#define ODMA_INST_OFF_DONE_MASK			(1 << 5)
#define ODMA_WRITE_SLAVE_ERR_MASK		(1 << 4)
#define ODMA_DEADLOCK_MASK			(1 << 2)
#define ODMA_FRAME_DONE_MASK			(1 << 1)
#define ODMA_ALL_IRQ_MASK			(0x101B << 1)
#define ODMA_IRQ_ENABLE				(1 << 0)

#define WDMA_OUT_CTRL_0				(0x0008)
#define ODMA_ALPHA(_v)				((_v) << 24)
#define ODMA_ALPHA_MASK				(0xFF << 24)
#define ODMA_IC_MAX(_v)				((_v) << 16)
#define ODMA_IC_MAX_MASK			(0xFF << 16)
#define ODMA_SBWC_LOSSY				(1 << 14)
#define ODMA_IMG_FORMAT(_v)			((_v) << 8)
#define ODMA_IMG_FORMAT_MASK			(0x3F << 8)
#define ODMA_CSET_EN				(1 << 3) // must keep as '0'
#define ODMA_SBWC_EN				(1 << 2)
#define ODMA_AFBC_EN				(1 << 1)

#define WDMA_DST_SIZE				(0x0010)
#define ODMA_DST_HEIGHT(_v)			((_v) << 16)
#define ODMA_DST_HEIGHT_MASK			(0x3FFF << 16)
#define ODMA_DST_WIDTH(_v)			((_v) << 0)
#define ODMA_DST_WIDTH_MASK			(0xFFFF << 0)

#define WDMA_DST_OFFSET				(0x0014)
#define ODMA_DST_OFFSET_Y(_v)			((_v) << 16)
#define ODMA_DST_OFFSET_Y_MASK			(0x1FFF << 16)
#define ODMA_DST_OFFSET_X(_v)			((_v) << 0)
#define ODMA_DST_OFFSET_X_MASK			(0x1FFF << 0)

#define WDMA_IMG_SIZE				(0x0018)
#define ODMA_IMG_HEIGHT(_v)			((_v) << 16)
#define ODMA_IMG_HEIGHT_MASK			(0x1FFF << 16)
#define ODMA_IMG_WIDTH(_v)			((_v) << 0)
#define ODMA_IMG_WIDTH_MASK			(0x1FFF << 0)

#define WDMA_BASEADDR_Y8			(0x0040)
#define WDMA_BASEADDR_C8			(0x0044)
#define WDMA_BASEADDR_Y2			(0x0048)
#define WDMA_BASEADDR_C2			(0x004C)

#define WDMA_STRIDE_0				(0x0050)
#define ODMA_STRIDE_3_SEL			(1 << 23)
#define ODMA_STRIDE_2_SEL			(1 << 22)
#define ODMA_STRIDE_1_SEL			(1 << 21)
#define ODMA_STRIDE_0_SEL			(1 << 20)
#define ODMA_STRIDE_SEL(_v)			((_v) << 20)
#define ODMA_STRIDE_SEL_MASK			(0xF << 20)
#define ODMA_CHROM_STRIDE_SEL			(1 << 16)
#define ODMA_CHROM_STRIDE(_v)			((_v) << 0)
#define ODMA_CHROM_STRIDE_MASK			(0xFFFF << 0)

#define WDMA_STRIDE_1				(0x0054)
#define ODMA_STRIDE_1(_v)			((_v) << 16)
#define ODMA_STRIDE_1_MASK			(0xFFFF << 16)
#define ODMA_STRIDE_0(_v)			((_v) << 0)
#define ODMA_STRIDE_0_MASK			(0xFFFF << 0)

#define WDMA_STRIDE_2				(0x0058)
#define ODMA_STRIDE_3(_v)			((_v) << 16)
#define ODMA_STRIDE_3_MASK			(0xFFFF << 16)
#define ODMA_STRIDE_2(_v)			((_v) << 0)
#define ODMA_STRIDE_2_MASK			(0xFFFF << 0)

#define WDMA_AFBC_PARAM				(0x0060)
#define ODMA_AFBC_BLK_SIZE(_v)			((_v) << 0)
#define ODMA_AFBC_BLK_SIZE_MASK			(0x3 << 0)
#define ODMA_AFBC_BLK_SIZE_16_16		(0)
#define ODMA_AFBC_BLK_SIZE_32_8			(1)
#define ODMA_AFBC_BLK_SIZE_64_4			(2)

#define WDMA_SBWC_PARAM				(0x0064)
#define ODMA_CRC_EN				(1 << 16)
/* [32 x n] 2: 64 byte */
#define ODMA_CHM_BLK_BYTENUM(_v)		((_v) << 8)
#define ODMA_CHM_BLK_BYTENUM_MASK		(0xF << 8)
#define ODMA_LUM_BLK_BYTENUM(_v)		((_v) << 4)
#define ODMA_LUM_BLK_BYTENUM_MASK		(0xF << 4)
/* only valid 32x4 */
#define ODMA_CHM_BLK_SIZE(_v)			((_v) << 2)
#define ODMA_CHM_BLK_SIZE_MASK			(0x3 << 2)
#define ODMA_CHM_BLK_SIZE_32_4			(0)
#define ODMA_LUM_BLK_SIZE(_v)			((_v) << 0)
#define ODMA_LUM_BLK_SIZE_MASK			(0x3 << 0)
#define ODMA_CHM_BLK_SIZE_32_4			(0)

#define WDMA_CSET_PARAM				(0x0068)
#define ODMA_GB_BASE(_v)			((_v) << 0)
#define ODMA_GB_BASE_MASK			(0x1f << 0)

#define WDMA_DEADLOCK_CTRL			(0x0100)
#define ODMA_DEADLOCK_NUM(_v)			((_v) << 1)
#define ODMA_DEADLOCK_NUM_MASK			(0x7FFFFFFF << 1)
#define ODMA_DEADLOCK_NUM_EN			(1 << 0)

#define WDMA_BUS_CTRL				(0x0110)
#define ODMA_AWCACHE_P3				((_v) << 12)
#define ODMA_AWCACHE_P3_MASK			(0xF << 12)
#define ODMA_AWCACHE_P2				((_v) << 8)
#define ODMA_AWCACHE_P2_MASK			(0xF << 8)
#define ODMA_AWCACHE_P1				((_v) << 4)
#define ODMA_AWCACHE_P1_MASK			(0xF << 4)
#define ODMA_AWCACHE_P0				((_v) << 0)
#define ODMA_AWCACHE_P0_MASK			(0xF << 0)

#define WDMA_LLC_CTRL				(0x0114)
#define ODMA_DATA_SAHRE_TYPE_P3(_v)		((_v) << 28)
#define ODMA_DATA_SAHRE_TYPE_P3_MASK		(0x3 << 28)
#define ODMA_LLC_HINT_P3(_v)			((_v) << 24)
#define ODMA_LLC_HINT_P3_MASK			(0x7 << 24)
#define ODMA_DATA_SAHRE_TYPE_P2(_v)		((_v) << 20)
#define ODMA_DATA_SAHRE_TYPE_P2_MASK		(0x3 << 20)
#define ODMA_LLC_HINT_P2(_v)			((_v) << 16)
#define ODMA_LLC_HINT_P2_MASK			(0x7 << 16)
#define ODMA_DATA_SAHRE_TYPE_P1(_v)		((_v) << 12)
#define ODMA_DATA_SAHRE_TYPE_P1_MASK		(0x3 << 12)
#define ODMA_LLC_HINT_P1(_v)			((_v) << 8)
#define ODMA_LLC_HINT_P1_MASK			(0x7 << 8)
#define ODMA_DATA_SAHRE_TYPE_P0(_v)		((_v) << 4)
#define ODMA_DATA_SAHRE_TYPE_P0_MASK		(0x3 << 4)
#define ODMA_LLC_HINT_P0(_v)			((_v) << 0)
#define ODMA_LLC_HINT_P0_MASK			(0x7 << 0)

#define WDMA_PERF_CTRL				(0x0120)
#define ODMA_DEGRADATION_TIME(_v)		((_v) << 16)
#define ODMA_DEGRADATION_TIME_MASK		(0xFFFF << 16)
#define ODMA_IC_MAX_DEG(_v)			((_v) << 4)
#define ODMA_IC_MAX_DEG_MASK			(0xFF << 4)
#define ODMA_DEGRADATION_EN			(1 << 0)

/* _n: [0,7], _v: [0x0, 0xF] */
#define WDMA_QOS_LUT_LOW			(0x0130)
#define WDMA_QOS_LUT_HIGH			(0x0134)
#define ODMA_QOS_LUT(_n, _v)			((_v) << (4*(_n)))
#define ODMA_QOS_LUT_MASK(_n)			(0xF << (4*(_n)))

#define WDMA_DYNAMIC_GATING_EN			(0x0140)
#define ODMA_SRAM_CG_EN				(1 << 31)
#define ODMA_DG_EN(_n, _v)			((_v) << (_n))
#define ODMA_DG_EN_MASK(_n)			(1 << (_n))
#define ODMA_DG_EN_ALL				(0x7FFFFFFF << 0)

#define WDMA_MST_SECURITY			(0x0200)
#define WDMA_SLV_SECURITY			(0x0204)

#define WDMA_DEBUG_CTRL				(0x0300)
#define ODMA_DEBUG_SEL(_v)			((_v) << 16)
#define ODMA_DEBUG_EN				(0x1 << 0)

#define WDMA_DEBUG_DATA				(0x0304)

#define WDMA_PSLV_ERR_CTRL			(0x030C)
#define ODMA_PSLVERR_CTRL			(1 << 0)

#define WDMA_DEBUG_ADDR_ERR			(0x0730)
#define ODMA_ERR_ADDR_C2			(1 << 3)
#define ODMA_ERR_ADDR_Y2			(1 << 2)
#define ODMA_ERR_ADDR_C8			(1 << 1)
#define ODMA_ERR_ADDR_Y8			(1 << 0)
#define ODMA_ERR_ADDR_GET(_v)			(((_v) >> 0) & 0xF)

#define WDMA_CONFIG_ERR_STATUS			(0x0740)
#define ODMA_CFG_ERR_STRIDE1			(1 << 12)
#define ODMA_CFG_ERR_STRIDE0			(1 << 11)
#define ODMA_CFG_ERR_CHROM_STRIDE		(1 << 10)
#define ODMA_CFG_ERR_BASE_ADDR_C2		(1 << 9)
#define ODMA_CFG_ERR_BASE_ADDR_Y2		(1 << 8)
#define ODMA_CFG_ERR_BASE_ADDR_C8		(1 << 7)
#define ODMA_CFG_ERR_BASE_ADDR_Y8		(1 << 6)
#define ODMA_CFG_ERR_DST_OFFSET_Y		(1 << 5)
#define ODMA_CFG_ERR_DST_OFFSET_X		(1 << 4)
#define ODMA_CFG_ERR_IMG_HEIGHT			(1 << 3)
#define ODMA_CFG_ERR_IMG_WIDTH			(1 << 2)
#define ODMA_CFG_ERR_DST_HEIGHT			(1 << 1)
#define ODMA_CFG_ERR_DST_WIDTH			(1 << 0)
#define ODMA_CFG_ERR_GET(_v)			(((_v) >> 0) & 0x3FFFFF)


/*
 * NEW : 0x0D00 ~ 0x0D28
 * SHADOW_OFFSET : 0x0D80
 */
#define WDMA_BWL_ENABLE				(0x0D00)
#define ODMA_LIMIT_EN_HDR			(1 << 4)
#define ODMA_LIMIT_EN_PLD			(1 << 0)

/* number of cycles in 1us : ACLK base */
#define WDMA_BWL_FREQ				(0x0D04)
#define ODMA_NUM_CYCLES(_v)			((_v) << 0)
#define ODMA_NUM_CYCLES_MASK			(0xFFF << 0)

#define WDMA_BWL_LIMIT_CTRL_PLD			(0x0D10)
#define ODMA_AVG_BW_LIMIT(_v)			((_v) << 16)
#define ODMA_AVG_BW_LIMIT_MASK			(0xFFFF << 16)
#define ODMA_SLOT_BW_LIMIT(_v)			((_v) << 0)
#define ODMA_SLOT_BW_LIMIT_MASK			(0xFFFF << 0)

#define WDMA_BWL_COMPENSATE_CTRL_PLD		(0x0D14)
#define ODMA_COMPENSATION_LIMIT(_v)		((_v) << 16)
#define ODMA_COMPENSATION_LIMIT_MASK		(0xFFFF << 16)
#define ODMA_PERIOD(_v)				((_v) << 0)
#define ODMA_PERIOD_MASK			(0xFFF << 0)

#define WDMA_BWL_LIMIT_CTRL_HDR			(0x0D20)
/* same field with WDMA_BWL_LIMIT_CTRL_PLD */

#define WDMA_BWL_COMPENSATE_CTRL_HDR		(0x0D24)
/* same field with WDMA_BWL_COMPENSATE_CTRL_PLD */


/*
 *-------------------------------------------------------------------
 * DPP_COMMON(L0~L5, L12) SFR list
 * base address : 0x1C0D_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5      L12
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000  0xC000
 *-------------------------------------------------------------------
 */
#define DPP_COM_SHD_OFFSET			(0x0100)
#define DPP_SCL_SHD_OFFSET			(0x0400)

#define DPP_COM_VERSION				(0x0000)
#define DPP_COM_VERSION_GET(_v)			(((_v) >> 0) & 0xffffffff)

#define DPP_COM_SWRST_CON			(0x0004)
#define DPP_SRESET				(1 << 0)

#define DPP_COM_QCH_CON				(0x0008)
#define DPP_QACTIVE				(1 << 0)

#define DPP_COM_PSLVERR_CON			(0x000c)
#define DPP_PSLVERR_EN				(1 << 0)

#define DPP_COM_IRQ_CON				(0x0010)
#define DPP_IRQ_EN				(1 << 0)

#define DPP_COM_IRQ_MASK			(0x0014)
#define DPP_CFG_ERROR_MASK			(1 << 1)
#define DPP_FRM_DONE_MASK			(1 << 0)
#define DPP_ALL_IRQ_MASK			(0x3 << 0)

#define DPP_COM_IRQ_STATUS			(0x0018)
#define DPP_CFG_ERROR_IRQ			(1 << 1)
#define DPP_FRM_DONE_IRQ			(1 << 0)
#define DPP_ALL_IRQ_CLEAR			(0x3 << 0)

#define DPP_COM_CFG_ERROR_STATUS		(0x001C)
#define DPP_CFG_ERR_SCL_POS			(1 << 4)
#define DPP_CFG_ERR_SCL_RATIO			(1 << 3)
#define DPP_CFG_ERR_ODD_SIZE			(1 << 2)
#define DPP_CFG_ERR_MAX_SIZE			(1 << 1)
#define DPP_CFG_ERR_MIN_SIZE			(1 << 0)

#define DPP_COM_LC_CON				(0x0020)
#define DPP_LC_CAPTURE_MASK			(0x1 << 2)
#define DPP_LC_MODE(_V)				((_V) << 1)
#define DPP_LC_MODE_MASK			(0x1 << 1)
#define DPP_LC_EN(_v)				((_v) << 0)
#define DPP_LC_EN_MASK				(0x1 << 0)

#define DPP_COM_LC_STATUS			(0x0024)
#define DPP_LC_COUNTER_GET(_v)			(((_v) >> 0) & 0xFFFFFFFF)

#define DPP_COM_DBG_CON				(0x0028)
#define DPP_DBG_SEL(_v)				((_v) << 16)
#define DPP_DBG_EN				(1 << 0)

#define DPP_COM_DBG_STATUS			(0x002C)

#define DPP_COM_OP_STATUS			(0x0030)
#define DPP_OP_STATUS				(1 << 0)

#define DPP_COM_TZPC				(0x0034)
#define DPP_TZPC				(1 << 0)

#define DPP_COM_IO_CON				(0x0038)
#define DPP_ALPHA_SEL(_v)			((_v) << 7)
#define DPP_ALPHA_SEL_MASK			(0x1 << 7)
#define DPP_BPC_MODE(_v)			((_v) << 6)
#define DPP_BPC_MODE_8BPC			(0 << 6)
#define DPP_BPC_MODE_10BPC			(1 << 6)
#define DPP_BPC_MODE_MASK			(0x1 << 6)
#define DPP_IMG_FORMAT(_v)			((_v) << 0)
#define DPP_IMG_FORMAT_MASK			(0x7 << 0)
#define DPP_IMG_FORMAT_ARGB8888			(0 << 0)
#define DPP_IMG_FORMAT_ARGB8101010		(1 << 0)
#define DPP_IMG_FORMAT_YUV420_8P		(2 << 0)
#define DPP_IMG_FORMAT_YUV420_P010		(3 << 0)
#define DPP_IMG_FORMAT_YUV420_8P2		(4 << 0)
#define DPP_IMG_FORMAT_YUV422_8P		(5 << 0)
#define DPP_IMG_FORMAT_YUV422_P210		(6 << 0)
#define DPP_IMG_FORMAT_YUV422_8P2		(7 << 0)

#define DPP_COM_IMG_SIZE			(0x003C)
#define DPP_IMG_HEIGHT(_v)			((_v) << 16)
#define DPP_IMG_HEIGHT_MASK			(0x3FFF << 16)
#define DPP_IMG_WIDTH(_v)			((_v) << 0)
#define DPP_IMG_WIDTH_MASK			(0x3FFF << 0)

#define DPP_COM_CSC_CON				(0x0040)
#define DPP_CSC_TYPE(_v)			((_v) << 2)
#define DPP_CSC_TYPE_MASK			DPP_CSC_TYPE(3)
#define DPP_CSC_TYPE_BT601			DPP_CSC_TYPE(0)
#define DPP_CSC_TYPE_BT709			DPP_CSC_TYPE(1)
#define DPP_CSC_TYPE_BT2020			DPP_CSC_TYPE(2)
#define DPP_CSC_TYPE_DCI_P3			DPP_CSC_TYPE(3)
#define DPP_CSC_RANGE(_v)			((_v) << 1)
#define DPP_CSC_RANGE_MASK			DPP_CSC_RANGE(1)
#define DPP_CSC_RANGE_LIMITED			DPP_CSC_RANGE(0)
#define DPP_CSC_RANGE_FULL			DPP_CSC_RANGE(1)
#define DPP_CSC_MODE(_v)			((_v) << 0)
#define DPP_CSC_MODE_MASK			DPP_CSC_MODE(1)
#define DPP_CSC_MODE_HARDWIRED			DPP_CSC_MODE(0)
#define DPP_CSC_MODE_CUSTOMIZED			DPP_CSC_MODE(1)

/*
 * (00-01-02) : Reg0.L-Reg0.H-Reg1.L
 * (10-11-12) : Reg1.H-Reg2.L-Reg2.H
 * (20-21-22) : Reg3.L-Reg3.H-Reg4.L
 */
#define DPP_COM_CSC_COEF0			(0x0044)
#define DPP_COM_CSC_COEF1			(0x0048)
#define DPP_COM_CSC_COEF2			(0x004C)
#define DPP_COM_CSC_COEF3			(0x0050)
#define DPP_COM_CSC_COEF4			(0x0054)
#define DPP_CSC_COEF_H(_v)			((_v) << 16)
#define DPP_CSC_COEF_H_MASK			(0xFFFF << 16)
#define DPP_CSC_COEF_L(_v)			((_v) << 0)
#define DPP_CSC_COEF_L_MASK			(0xFFFF << 0)
#define DPP_CSC_COEF_XX(_n, _v)			((_v) << (0 + (16 * (_n))))
#define DPP_CSC_COEF_XX_MASK(_n)		(0xFFF << (0 + (16 * (_n))))

#define DPP_COM_HDR_CON				(0x0058)
#define DPP_MULT_EN				(1 << 0)

#define DPP_COM_DITH_CON			(0x005C)
#define DPP_DITH_MASK_SEL			(1 << 1)
#define DPP_DITH_MASK_SPIN			(1 << 0)

#define DPP_COM_SUB_CON				(0x0060)
#define DPP_UV_OFFSET_Y(_v)			((_v) << 4)
#define DPP_UV_OFFSET_Y_MASK			(0x7 << 4)
#define DPP_UV_OFFSET_X(_v)			((_v) << 0)
#define DPP_UV_OFFSET_X_MASK			(0x7 << 0)

/* SCL: L1,3,5 only */
#define DPP_SCL_SCALED_IMG_SIZE			(0x0200)
#define DPP_SCALED_IMG_HEIGHT(_v)		((_v) << 16)
#define DPP_SCALED_IMG_HEIGHT_MASK		(0x3FFF << 16)
#define DPP_SCALED_IMG_WIDTH(_v)		((_v) << 0)
#define DPP_SCALED_IMG_WIDTH_MASK		(0x3FFF << 0)

#define DPP_SCL_MAIN_H_RATIO			(0x0204)
#define DPP_H_RATIO(_v)				((_v) << 0)
#define DPP_H_RATIO_MASK			(0xFFFFFF << 0)

#define DPP_SCL_MAIN_V_RATIO			(0x0208)
#define DPP_V_RATIO(_v)				((_v) << 0)
#define DPP_V_RATIO_MASK			(0xFFFFFF << 0)

#define DPP_SCL_Y_VCOEF_0A			(0x0210)
#define DPP_SCL_Y_HCOEF_0A			(0x02A0)
#define DPP_SCL_C_VCOEF_0A			(0x03C0)
#define DPP_SCL_C_HCOEF_0A			(0x0450)
#define DPP_SCL_COEF(_v)			((_v) << 0)
#define DPP_SCL_COEF_MASK			(0x7FF << 0)
#define DPP_H_COEF(n, s, x)			(0x2A0 + (n) * 0x4 +	\
						(s) * 0x24 + (x) * 0x1B0)
#define DPP_V_COEF(n, s, x)			(0x210 + (n) * 0x4 +	\
						(s) * 0x24 + (x) * 0x1B0)

#define DPP_SCL_YHPOSITION			(0x0570)
#define DPP_SCL_YVPOSITION			(0x0574)
#define DPP_SCL_CHPOSITION			(0x0578)
#define DPP_SCL_CVPOSITION			(0x057C)
#define DPP_POS_I(_v)				((_v) << 20)
#define DPP_POS_I_MASK				(0xFFF << 20)
#define DPP_POS_I_GET(_v)			(((_v) >> 20) & 0xFFF)
#define DPP_POS_F(_v)				((_v) << 0)
#define DPP_POS_F_MASK				(0xFFFFF << 0)
#define DPP_POS_F_GET(_v)			(((_v) >> 0) & 0xFFFFF)

/* 0x0580 ~ 0x059C : ASHE */
#define DPP_SCL_ASHE_CON			(0x0580)
// ASHE_PARAM


/*
 *-------------------------------------------------------------------
 * DPP_HDR(L0~L5) SFR list
 * base address : 0x1C0E_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000
 *-------------------------------------------------------------------
 */
#define DPP_VGRF_HDR_CON		(0x0600)
#define DPP_TM_ON(_v)			((_v) << 3)
#define DPP_TM_ON_MASK			(0x1 << 3)
#define DPP_GM_ON(_v)			((_v) << 2)
#define DPP_GM_ON_MASK			(0x1 << 2)
#define DPP_EOTF_ON(_v)			((_v) << 1)
#define DPP_EOTF_ON_MASK		(0x1 << 1)
#define DPP_HDR_ON(_v)			((_v) << 0)
#define DPP_HDR_ON_MASK			(0x1 << 0)

/* EOTF */
#define DPP_HDR_EOTF_X_AXIS_ADDR(_n)	(((_n) / 2) * (0x4) + (0x610))
#define DPP_HDR_EOTF_X_AXIS_VAL(_n, _v)	\
	(((_n) % (2)) ? (((_v) & 0x3FFF) << 16) : (((_v) & 0x3FFF) << 0))

#define DPP_HDR_EOTF_Y_AXIS_ADDR(_n)	(((_n) / 2) * (0x4) + (0x694))
#define DPP_HDR_EOTF_Y_AXIS_VAL(_n, _v)	\
	(((_n) % (2)) ? (((_v) & 0x3FFF) << 16) : (((_v) & 0x3FFF) << 0))

#define DPP_HDR_EOTF_MASK(_n)		(((_n) % 2) ?	\
					(0x3FFF << 16) : (0x3FFF << 0))

/* GM */
#define DPP_HDR_GM_COEF_ADDR(_n)	((_n) * (0x4) + (0x720))
#define DPP_HDR_GM_COEF_MASK	(0x1FFFF << 0)

/* TM */
#define DPP_HDR_TM_X_AXIS_ADDR(_n)	(((_n) / 2) * (0x4) + (0x750))
#define DPP_HDR_TM_X_AXIS_VAL(_n, _v)	\
	(((_n) % (2)) ? (((_v) & 0x3FFF) << 16) : (((_v) & 0x3FFF) << 0))

#define DPP_HDR_TM_Y_AXIS_ADDR(_n)	(((_n) / 2) * (0x4) + (0x794))
#define DPP_HDR_TM_Y_AXIS_VAL(_n, _v)	\
	(((_n) % (2)) ? (((_v) & 0x3FFF) << 16) : (((_v) & 0x3FFF) << 0))

#define DPP_HDR_TM_MASK(_n)		(((_n) % 2) ?	\
					(0x3FFF << 16) : (0x3FFF << 0))

#define DPP_VGRF_HDR_EOTF_X_AXIS_0	(0x0610)
#define DPP_VGRF_HDR_EOTF_X_AXIS_1	(0x0614)
#define DPP_VGRF_HDR_EOTF_X_AXIS_2	(0x0618)
#define DPP_VGRF_HDR_EOTF_X_AXIS_3	(0x061C)
#define DPP_VGRF_HDR_EOTF_X_AXIS_4	(0x0620)
#define DPP_VGRF_HDR_EOTF_X_AXIS_5	(0x0624)
#define DPP_VGRF_HDR_EOTF_X_AXIS_6	(0x0628)
#define DPP_VGRF_HDR_EOTF_X_AXIS_7	(0x062C)
#define DPP_VGRF_HDR_EOTF_X_AXIS_8	(0x0630)
#define DPP_VGRF_HDR_EOTF_X_AXIS_9	(0x0634)
#define DPP_VGRF_HDR_EOTF_X_AXIS_10	(0x0638)
#define DPP_VGRF_HDR_EOTF_X_AXIS_11	(0x063C)
#define DPP_VGRF_HDR_EOTF_X_AXIS_12	(0x0640)
#define DPP_VGRF_HDR_EOTF_X_AXIS_13	(0x0644)
#define DPP_VGRF_HDR_EOTF_X_AXIS_14	(0x0648)
#define DPP_VGRF_HDR_EOTF_X_AXIS_15	(0x064C)
#define DPP_VGRF_HDR_EOTF_X_AXIS_16	(0x0650)
#define DPP_VGRF_HDR_EOTF_X_AXIS_17	(0x0654)
#define DPP_VGRF_HDR_EOTF_X_AXIS_18	(0x0658)
#define DPP_VGRF_HDR_EOTF_X_AXIS_19	(0x065C)
#define DPP_VGRF_HDR_EOTF_X_AXIS_20	(0x0660)
#define DPP_VGRF_HDR_EOTF_X_AXIS_21	(0x0664)
#define DPP_VGRF_HDR_EOTF_X_AXIS_22	(0x0668)
#define DPP_VGRF_HDR_EOTF_X_AXIS_23	(0x066C)
#define DPP_VGRF_HDR_EOTF_X_AXIS_24	(0x0670)
#define DPP_VGRF_HDR_EOTF_X_AXIS_25	(0x0674)
#define DPP_VGRF_HDR_EOTF_X_AXIS_26	(0x0678)
#define DPP_VGRF_HDR_EOTF_X_AXIS_27	(0x067C)
#define DPP_VGRF_HDR_EOTF_X_AXIS_28	(0x0680)
#define DPP_VGRF_HDR_EOTF_X_AXIS_29	(0x0684)
#define DPP_VGRF_HDR_EOTF_X_AXIS_30	(0x0688)
#define DPP_VGRF_HDR_EOTF_X_AXIS_31	(0x068C)
#define DPP_VGRF_HDR_EOTF_X_AXIS_32	(0x0690)

#define DPP_VGRF_HDR_EOTF_Y_AXIS_0	(0x0694)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_1	(0x0698)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_2	(0x069C)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_3	(0x06A0)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_4	(0x06A4)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_5	(0x06A8)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_6	(0x06AC)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_7	(0x06B0)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_8	(0x06B4)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_9	(0x06B8)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_10	(0x06BC)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_11	(0x06C0)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_12	(0x06C4)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_13	(0x06C8)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_14	(0x06CC)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_15	(0x06D0)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_16	(0x06D4)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_17	(0x06D8)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_18	(0x06DC)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_19	(0x06E0)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_20	(0x06E4)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_21	(0x06E8)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_22	(0x06EC)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_23	(0x06F0)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_24	(0x06F4)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_25	(0x06F8)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_26	(0x06FC)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_27	(0x0700)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_28	(0x0704)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_29	(0x0708)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_30	(0x070C)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_31	(0x0710)
#define DPP_VGRF_HDR_EOTF_Y_AXIS_32	(0x0714)

#define DPP_VGRF_HDR_GM_COEF_0_0	(0x0720)
#define DPP_VGRF_HDR_GM_COEF_0_1	(0x0724)
#define DPP_VGRF_HDR_GM_COEF_0_2	(0x0728)
#define DPP_VGRF_HDR_GM_COEF_1_0	(0x072C)
#define DPP_VGRF_HDR_GM_COEF_1_1	(0x0730)
#define DPP_VGRF_HDR_GM_COEF_1_2	(0x0734)
#define DPP_VGRF_HDR_GM_COEF_2_0	(0x0738)
#define DPP_VGRF_HDR_GM_COEF_2_1	(0x073C)
#define DPP_VGRF_HDR_GM_COEF_2_2	(0x0740)

#define DPP_VGRF_HDR_TM_X_AXIS_0	(0x0750)
#define DPP_VGRF_HDR_TM_X_AXIS_1	(0x0754)
#define DPP_VGRF_HDR_TM_X_AXIS_2	(0x0758)
#define DPP_VGRF_HDR_TM_X_AXIS_3	(0x075C)
#define DPP_VGRF_HDR_TM_X_AXIS_4	(0x0760)
#define DPP_VGRF_HDR_TM_X_AXIS_5	(0x0764)
#define DPP_VGRF_HDR_TM_X_AXIS_6	(0x0768)
#define DPP_VGRF_HDR_TM_X_AXIS_7	(0x076C)
#define DPP_VGRF_HDR_TM_X_AXIS_8	(0x0770)
#define DPP_VGRF_HDR_TM_X_AXIS_9	(0x0774)
#define DPP_VGRF_HDR_TM_X_AXIS_10	(0x0778)
#define DPP_VGRF_HDR_TM_X_AXIS_11	(0x077C)
#define DPP_VGRF_HDR_TM_X_AXIS_12	(0x0780)
#define DPP_VGRF_HDR_TM_X_AXIS_13	(0x0784)
#define DPP_VGRF_HDR_TM_X_AXIS_14	(0x0788)
#define DPP_VGRF_HDR_TM_X_AXIS_15	(0x078C)
#define DPP_VGRF_HDR_TM_X_AXIS_16	(0x0790)

#define DPP_VGRF_HDR_TM_Y_AXIS_0	(0x0794)
#define DPP_VGRF_HDR_TM_Y_AXIS_1	(0x0798)
#define DPP_VGRF_HDR_TM_Y_AXIS_2	(0x079C)
#define DPP_VGRF_HDR_TM_Y_AXIS_3	(0x07A0)
#define DPP_VGRF_HDR_TM_Y_AXIS_4	(0x07A4)
#define DPP_VGRF_HDR_TM_Y_AXIS_5	(0x07A8)
#define DPP_VGRF_HDR_TM_Y_AXIS_6	(0x07AC)
#define DPP_VGRF_HDR_TM_Y_AXIS_7	(0x07B0)
#define DPP_VGRF_HDR_TM_Y_AXIS_8	(0x07B4)
#define DPP_VGRF_HDR_TM_Y_AXIS_9	(0x07B8)
#define DPP_VGRF_HDR_TM_Y_AXIS_10	(0x07BC)
#define DPP_VGRF_HDR_TM_Y_AXIS_11	(0x07C0)
#define DPP_VGRF_HDR_TM_Y_AXIS_12	(0x07C4)
#define DPP_VGRF_HDR_TM_Y_AXIS_13	(0x07C8)
#define DPP_VGRF_HDR_TM_Y_AXIS_14	(0x07CC)
#define DPP_VGRF_HDR_TM_Y_AXIS_15	(0x07D0)
#define DPP_VGRF_HDR_TM_Y_AXIS_16	(0x07D4)

#endif /* _REGS_DPP_H */
