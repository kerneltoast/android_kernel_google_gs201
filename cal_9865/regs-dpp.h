/* SPDX-License-Identifier: GPL-2.0-only
 *
 * dpu30/cal_9865/regs-dpp.h
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

#ifndef DPP_REGS_H_
#define DPP_REGS_H_

/*
 *-------------------------------------------------------------------
 * RDMA(L0~L6) SFR list
 * base address(DPUF0) : 0x1990_0000
 * base address(DPUF1) : 0x19D0_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5	    L6
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000  0x6000
 *-------------------------------------------------------------------
 */

/* SHADOW: 0x400 ~ 0x800 */
#define DMA_SHD_OFFSET				0x0400

#define RDMA_ENABLE				0x0000
#define IDMA_ASSIGNED_MO(_v)			((_v) << 24)
#define IDMA_ASSIGNED_MO_MASK			(0xFFU << 24)
#define IDMA_SRESET				(1 << 8)
#define IDMA_SFR_UPDATE_FORCE			(1 << 4)
#define IDMA_OP_STATUS				(1 << 2)
#define OP_STATUS_IDLE				(0)
#define OP_STATUS_BUSY				(1)
#define IDMA_INST_OFF_PEND			(1 << 1)
#define INST_OFF_PEND				(1)
#define INST_OFF_NOT_PEND			(0)

#define RDMA_IRQ				0x0004
#define IDMA_VOTF_ERR_IRQ			(1 << 27)
#define IDMA_AXI_ADDR_ERR_IRQ			(1 << 26)
#define IDMA_MO_CONFLICT_IRQ			(1 << 24)
#define IDMA_FBC_ERR_IRQ			(1 << 23)
#define IDMA_RECOVERY_TRG_IRQ			(1 << 22)
#define IDMA_CONFIG_ERR_IRQ			(1 << 21)
#define IDMA_INST_OFF_DONE			(1 << 20)
#define IDMA_READ_SLAVE_ERR_IRQ			(1 << 19)
#define IDMA_DEADLOCK_IRQ			(1 << 17)
#define IDMA_FRAME_DONE_IRQ			(1 << 16)
#define IDMA_ALL_IRQ_CLEAR			(0xDFB << 16)
#define IDMA_VOTF_ERR_MASK			(1 << 12)
#define IDMA_AXI_ADDR_ERR_IRQ_MASK		(1 << 11)
#define IDMA_MO_CONFLICT_MASK			(1 << 9)
#define IDMA_FBC_ERR_MASK			(1 << 8)
#define IDMA_RECOVERY_TRG_MASK			(1 << 7)
#define IDMA_CONFIG_ERR_MASK			(1 << 6)
#define IDMA_INST_OFF_DONE_MASK			(1 << 5)
#define IDMA_READ_SLAVE_ERR_MASK		(1 << 4)
#define IDMA_DEADLOCK_MASK			(1 << 2)
#define IDMA_FRAME_DONE_MASK			(1 << 1)
#define IDMA_ALL_IRQ_MASK			(0xDFB << 1)
#define IDMA_IRQ_ENABLE				(1 << 0)


/* defined for common part of driver only */
#define IDMA_RECOVERY_START_IRQ			IDMA_RECOVERY_TRG_IRQ
#define IDMA_READ_SLAVE_ERROR			IDMA_READ_SLAVE_ERR_IRQ
#define IDMA_STATUS_DEADLOCK_IRQ		IDMA_DEADLOCK_IRQ
#define IDMA_STATUS_FRAMEDONE_IRQ		IDMA_FRAME_DONE_IRQ

#define RDMA_IN_CTRL_0				0x0008
#define IDMA_ALPHA(_v)				((_v) << 24)
#define IDMA_ALPHA_MASK				(0xFFU << 24)
#define IDMA_IC_MAX(_v)				((_v) << 16)
#define IDMA_IC_MAX_MASK			(0xff << 16)
#define IDMA_SBWC_LOSSY				(1 << 14)
#define IDMA_IMG_FORMAT(_v)			((_v) << 8)
#define IDMA_IMG_FORMAT_MASK			(0x3f << 8)

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
#define IDMA_IMG_FORMAT_ARGB_FP16		(32)
#define IDMA_IMG_FORMAT_ABGR_FP16		(33)
#define IDMA_IMG_FORMAT_RGBA_FP16		(34)
#define IDMA_IMG_FORMAT_BGRA_FP16		(35)
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
#define IDMA_AFBC_EN				(1 << 3)
#define IDMA_SBWC_EN				(1 << 2)
#define IDMA_SAJC_EN				(1 << 1)
#define IDMA_BLOCK_EN				(1 << 0)

#define RDMA_SRC_WIDTH				0x0010
#define IDMA_SRC_WIDTH(_v)			((_v) << 0)
#define IDMA_SRC_WIDTH_MASK			(0xFFFFFF << 0)

#define RDMA_SRC_HEIGHT				0x0014
#define IDMA_SRC_HEIGHT(_v)			((_v) << 0)
#define IDMA_SRC_HEIGHT_MASK			(0xFFFFFF << 0)

#define RDMA_SRC_OFFSET				0x0018
#define IDMA_SRC_OFFSET_Y(_v)			((_v) << 16)
#define IDMA_SRC_OFFSET_Y_MASK			(0xFFFF << 16)
#define IDMA_SRC_OFFSET_X(_v)			((_v) << 0)
#define IDMA_SRC_OFFSET_X_MASK			(0xFFFF << 0)

#define RDMA_IMG_SIZE				0x001C
#define IDMA_IMG_HEIGHT(_v)			((_v) << 16)
#define IDMA_IMG_HEIGHT_MASK			(0xFFFF << 16)
#define IDMA_IMG_WIDTH(_v)			((_v) << 0)
#define IDMA_IMG_WIDTH_MASK			(0xFFFF << 0)

#define RDMA_BLOCK_OFFSET			0x0020
#define IDMA_BLK_OFFSET_Y(_v)			((_v) << 16)
#define IDMA_BLK_OFFSET_Y_MASK			(0x3FFF << 16)
#define IDMA_BLK_OFFSET_X(_v)			((_v) << 0)
#define IDMA_BLK_OFFSET_X_MASK			(0x3FFF << 0)

#define RDMA_BLOCK_SIZE				0x0024
#define IDMA_BLK_HEIGHT(_v)			((_v) << 16)
#define IDMA_BLK_HEIGHT_MASK			(0x3FFF << 16)
#define IDMA_BLK_WIDTH(_v)			((_v) << 0)
#define IDMA_BLK_WIDTH_MASK			(0x3FFF << 0)

#define RDMA_BASEADDR_P0			0x0040
#define RDMA_BASEADDR_P1			0x0044
#define RDMA_BASEADDR_P2			0x0048
#define RDMA_BASEADDR_P3			0x004C

#define RDMA_SRC_STRIDE_0			0x0050
#define IDMA_STRIDE_0_SEL			(1 << 31)
#define IDMA_STRIDE_0(_v)			((_v) << 0)
#define IDMA_STRIDE_0_MASK			(0xffffff << 0)

#define RDMA_SRC_STRIDE_1			0x0054
#define IDMA_STRIDE_1_SEL			(1 << 31)
#define IDMA_STRIDE_1(_v)			((_v) << 0)
#define IDMA_STRIDE_1_MASK			(0xffffff << 0)

#define RDMA_SRC_STRIDE_2			0x0058
#define IDMA_STRIDE_2_SEL			(1 << 31)
#define IDMA_STRIDE_2(_v)			((_v) << 0)
#define IDMA_STRIDE_2_MASK			(0xffffff << 0)

#define RDMA_SRC_STRIDE_3			0x005C
#define IDMA_STRIDE_3_SEL			(1 << 31)
#define IDMA_STRIDE_3(_v)			((_v) << 0)
#define IDMA_STRIDE_3_MASK			(0xffffff << 0)

#define RDMA_SBWC_PARAM				0x0064
#define IDMA_LOSSY_COMP_MODE			(1 << 16)
/* [32 x n] 2: 64 byte */
#define IDMA_CHM_BLK_BYTENUM(_v)		((_v) << 8)
#define IDMA_CHM_BLK_BYTENUM_MASK		(0xf << 8)
#define IDMA_LUM_BLK_BYTENUM(_v)		((_v) << 4)
#define IDMA_LUM_BLK_BYTENUM_MASK		(0xf << 4)
/* only valid 32x4 */
#define IDMA_CHM_BLK_SIZE(_v)			((_v) << 2)
#define IDMA_CHM_BLK_SIZE_MASK			(0x3 << 2)
#define IDMA_CHM_BLK_SIZE_32_4			(0)
#define IDMA_LUM_BLK_SIZE(_v)			((_v) << 0)
#define IDMA_LUM_BLK_SIZE_MASK			(0x3 << 0)
#define IDMA_CHM_BLK_SIZE_32_4			(0)

#define RDMA_AFBC_PARAM				0x0070
/* [128 x n] 3: 384 byte */
#define IDMA_AFBC_DECODE_SIZE(_v)		((_v) << 4)
#define IDMA_AFBC_DECODE_SIZE_MASK		(0xf << 4)
#define IDMA_AFBC_BLK_SIZE(_v)			((_v) << 0)
#define IDMA_AFBC_BLK_SIZE_MASK			(0x3 << 0)
#define IDMA_AFBC_BLK_SIZE_16_16		(0)
#define IDMA_AFBC_BLK_SIZE_32_8			(1)
#define IDMA_AFBC_BLK_SIZE_64_4			(2)

#define RDMA_RECOVERY_CTRL			0x0080
#define IDMA_RECOVERY_NUM(_v)			((_v) << 1)
#define IDMA_RECOVERY_NUM_MASK			(0x7FFFFFFFU << 1)
#define IDMA_RECOVERY_EN			(1 << 0)

#define RDMA_DEADLOCK_CTRL			0x0100
#define IDMA_DEADLOCK_NUM(_v)			((_v) << 1)
#define IDMA_DEADLOCK_NUM_MASK			(0x7FFFFFFFU << 1)
#define IDMA_DEADLOCK_NUM_EN			(1 << 0)

#define RDMA_BUS_CTRL				0x0110
#define IDMA_ARCACHE_P3				((_v) << 12)
#define IDMA_ARCACHE_P3_MASK			(0xf << 12)
#define IDMA_ARCACHE_P2				((_v) << 8)
#define IDMA_ARCACHE_P2_MASK			(0xf << 8)
#define IDMA_ARCACHE_P1				((_v) << 4)
#define IDMA_ARCACHE_P1_MASK			(0xf << 4)
#define IDMA_ARCACHE_P0				((_v) << 0)
#define IDMA_ARCACHE_P0_MASK			(0xf << 0)

#define RDMA_LLC_CTRL				0x0114
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

#define RDMA_PERF_CTRL				0x0120
#define IDMA_DEGRADATION_TIME(_v)		((_v) << 16)
#define IDMA_DEGRADATION_TIME_MASK		(0xFFFF << 16)
#define IDMA_IC_MAX_DEG(_v)			((_v) << 4)
#define IDMA_IC_MAX_DEG_MASK			(0xFF << 4)
#define IDMA_DEGRADATION_EN			(1 << 0)

/* _n: [0,7], _v: [0x0, 0xF] */
#define RDMA_QOS_LUT_LOW			0x0130
#define RDMA_QOS_LUT_HIGH			0x0134
#define IDMA_QOS_LUT(_n, _v)			((_v) << (4*(_n)))
#define IDMA_QOS_LUT_MASK(_n)			(0xF << (4*(_n)))

#define RDMA_DYNAMIC_GATING_EN			0x0140
#define IDMA_SRAM_CG_EN				(1U << 31)
#define IDMA_DG_EN(_n, _v)			((_v) << (_n))
#define IDMA_DG_EN_MASK(_n)			(1 << (_n))
#define IDMA_DG_EN_ALL				(0x7FFFFFFF << 0)

#define RDMA_MST_SECURITY			0x200
#define RDMA_MST_SECURITY_MASK			(0x1 << 0)
#define RDMA_SLV_SECURITY			0x204

#define RDMA_DEBUG_CTRL				0x0300
#define IDMA_DEBUG_SEL(_v)			((_v) << 16)
#define IDMA_DEBUG_EN				(0x1 << 0)

#define RDMA_DEBUG_DATA				0x0304

/* 0: AXI, 3: Pattern */
#define RDMA_IN_REQ_DEST			0x0308
#define IDMA_REQ_DEST_SEL(_v)			((_v) << 0)
#define IDMA_REQ_DEST_SEL_MASK			(0x3 << 0)

#define RDMA_PSLV_ERR_CTRL			0x030c
#define IDMA_PSLVERR_CTRL			(1 << 0)

#define RDMA_DEBUG_ADDR_P0			0x0310
#define RDMA_DEBUG_ADDR_P1			0x0314
#define RDMA_DEBUG_ADDR_P2			0x0318
#define RDMA_DEBUG_ADDR_P3			0x031C

#define RDMA_DEBUG_ADDR_CTRL			0x0320
#define IDMA_DEBUG_EN_ADDR_P3			(1 << 3)
#define IDMA_DEBUG_EN_ADDR_P2			(1 << 2)
#define IDMA_DEBUG_EN_ADDR_P1			(1 << 1)
#define IDMA_DEBUG_EN_ADDR_P0			(1 << 0)

#define RDMA_DEBUG_ADDR_ERR			0x0730
#define IDMA_ERR_ADDR_C2			(1 << 3)
#define IDMA_ERR_ADDR_Y2			(1 << 2)
#define IDMA_ERR_ADDR_C8			(1 << 1)
#define IDMA_ERR_ADDR_Y8			(1 << 0)
#define IDMA_ERR_ADDR_GET(_v)			(((_v) >> 0) & 0xF)

#define RDMA_CONFIG_ERR_STATUS			0x0740
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
#define RDMA_BWL_ENABLE				0xd00
#define IDMA_LIMIT_EN_HDR			(1 << 4)
#define IDMA_LIMIT_EN_PLD			(1 << 0)

/* number of cycles in 1us : ACLK base */
#define RDMA_BWL_FREQ				0xd04
#define IDMA_NUM_CYCLES(_v)			((_v) << 0)
#define IDMA_NUM_CYCLES_MASK			(0xfff << 0)

#define RDMA_BWL_LIMIT_CTRL_PLD			0xd10
#define IDMA_AVG_BW_LIMIT(_v)			((_v) << 16)
#define IDMA_AVG_BW_LIMIT_MASK			(0xffff << 16)
#define IDMA_SLOT_BW_LIMIT(_v)			((_v) << 0)
#define IDMA_SLOT_BW_LIMIT_MASK			(0xffff << 0)

#define RDMA_BWL_COMPENSATE_CTRL_PLD		0xd14
#define IDMA_COMPENSATION_LIMIT(_v)		((_v) << 16)
#define IDMA_COMPENSATION_LIMIT_MASK		(0xffff << 16)
#define IDMA_PERIOD(_v)				((_v) << 0)
#define IDMA_PERIOD_MASK			(0xfff << 0)

#define RDMA_BWL_LIMIT_CTRL_HDR			0xd20
/* same field with RDMA_BWL_LIMIT_CTRL_PLD */

#define RDMA_BWL_COMPENSATE_CTRL_HDR		0xd24
/* same field with RDMA_BWL_COMPENSATE_CTRL_PLD */


/* L0,2,4 only : Global is moved to one layer of each port */
#define CBP_ENABLE				0xe00
#define CBP_PREFETCH_ON				(1 << 0)

#define CBP_SIZE				0xe04
#define CBP_PREFETCH_SIZE(_v)			((_v) << 0)
#define CBP_PREFETCH_SIZE_MASK			(0x1f << 0)

#define CBP_DIST				0xe08
#define CBP_DISTANCE(_v)			((_v) << 0)
#define CBP_DISTANCE_MASK			(0xffff << 0)

#define CBP_TOTAL_MO				0xe10
#define CBP_OUTSTAND_TOTAL(_v)			((_v) << 0)
#define CBP_OUTSTAND_TOTAL_MASK			(0xff << 0)


#define GLB_DPU_DMA_VERSION			0xf00
#define GLB_DPU_VERSION				0x06010000

#define GLB_QCH_EN				0xf0c
#define GLB_DPU_QCH_EN				(1 << 0)

#define GLB_SWRST				0xf10
#define GLB_DPU_ALL_SWRST			(1 << 0)

#define GLB_GLB_CGEN				0xf14
#define GLB_DPU_INT_CGEN(_v)			((_v) << 0)
#define GLB_DPU_INT_CGEN_MASK			(0x7FFFFFFF << 0)

#define GLB_TEST_PATTERN0_3			0xf20
#define GLB_TEST_PATTERN0_2			0xf24
#define GLB_TEST_PATTERN0_1			0xf28
#define GLB_TEST_PATTERN0_0			0xf2c
#define GLB_TEST_PATTERN1_3			0xf30
#define GLB_TEST_PATTERN1_2			0xf34
#define GLB_TEST_PATTERN1_1			0xf38
#define GLB_TEST_PATTERN1_0			0xf3c

#define GLB_DEBUG_CTRL				0xf80
#define GLB_DEBUG_SEL(_v)			((_v) << 16)
#define GLB_DEBUG_EN				(0x1 << 0)

#define GLB_DEBUG_DATA				0xf84



/*
 *-------------------------------------------------------------------
 * WDMA SFR list
 * base address : 0x1990_8000
 * base address : 0x19D0_8000
 *-------------------------------------------------------------------
 */
#define WDMA_ENABLE				0x0000
#define ODMA_SRESET				(1 << 8)
#define ODMA_SHD_UPDATE_FORCE			(1 << 4)
#define ODMA_OP_STATUS				(1 << 2)
#define ODMA_INST_OFF_PEND			(1 << 1)

#define WDMA_IRQ				0x0004
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

#define WDMA_OUT_CTRL_0				0x0008
#define ODMA_ALPHA(_v)				((_v) << 24)
#define ODMA_ALPHA_MASK				(0xFFU << 24)
#define ODMA_IC_MAX(_v)				((_v) << 16)
#define ODMA_IC_MAX_MASK			(0xff << 16)
#define ODMA_SBWC_LOSSY				(1 << 14)
#define ODMA_IMG_FORMAT(_v)			((_v) << 8)
#define ODMA_IMG_FORMAT_MASK			(0x3f << 8)
#define ODMA_SBWC_EN				(1 << 2)
#define ODMA_SAJC_EN				(1 << 1)

#define WDMA_DST_WIDTH				0x0010
#define ODMA_DST_WIDTH(_v)			((_v) << 0)
#define ODMA_DST_WIDTH_MASK			(0xFFFFFF << 0)

#define WDMA_DST_HEIGHT				0x0014
#define ODMA_DST_HEIGHT(_v)			((_v) << 0)
#define ODMA_DST_HEIGHT_MASK			(0xFFFFFF << 0)

#define WDMA_DST_OFFSET				0x0018
#define ODMA_DST_OFFSET_Y(_v)			((_v) << 16)
#define ODMA_DST_OFFSET_Y_MASK			(0x1FFF << 16)
#define ODMA_DST_OFFSET_X(_v)			((_v) << 0)
#define ODMA_DST_OFFSET_X_MASK			(0x1FFF << 0)

#define WDMA_IMG_SIZE				0x001C
#define ODMA_IMG_HEIGHT(_v)			((_v) << 16)
#define ODMA_IMG_HEIGHT_MASK			(0x1FFF << 16)
#define ODMA_IMG_WIDTH(_v)			((_v) << 0)
#define ODMA_IMG_WIDTH_MASK			(0x1FFF << 0)

#define WDMA_BASEADDR_P0			0x0040
#define WDMA_BASEADDR_P1			0x0044
#define WDMA_BASEADDR_P2			0x0048
#define WDMA_BASEADDR_P3			0x004C

#define WDMA_STRIDE_0				0x0050
#define ODMA_STRIDE_0(_v)			((_v) << 0)
#define ODMA_STRIDE_0_MASK			(0xffffff << 0)

#define WDMA_STRIDE_1				0x0054
#define ODMA_STRIDE_1(_v)			((_v) << 0)
#define ODMA_STRIDE_1_MASK			(0xffff << 0)

#define WDMA_STRIDE_2				0x0058
#define ODMA_STRIDE_2(_v)			((_v) << 0)
#define ODMA_STRIDE_2_MASK			(0xffffff << 0)

#define WDMA_STRIDE_3				0x005C
#define ODMA_STRIDE_3(_v)			((_v) << 0)
#define ODMA_STRIDE_3_MASK			(0xffffff << 0)

#define WDMA_SBWC_PARAM				0x0064
#define ODMA_LOSSY_COMP_MODE			(1 << 16)
/* [32 x n] 2: 64 byte */
#define ODMA_CHM_BLK_BYTENUM(_v)		((_v) << 8)
#define ODMA_CHM_BLK_BYTENUM_MASK		(0xf << 8)
#define ODMA_LUM_BLK_BYTENUM(_v)		((_v) << 4)
#define ODMA_LUM_BLK_BYTENUM_MASK		(0xf << 4)
/* only valid 32x4 */
#define ODMA_CHM_BLK_SIZE(_v)			((_v) << 2)
#define ODMA_CHM_BLK_SIZE_MASK			(0x3 << 2)
#define ODMA_CHM_BLK_SIZE_32_4			(0)
#define ODMA_LUM_BLK_SIZE(_v)			((_v) << 0)
#define ODMA_LUM_BLK_SIZE_MASK			(0x3 << 0)
#define ODMA_CHM_BLK_SIZE_32_4			(0)

#define WDMA_DEADLOCK_CTRL			0x0100
#define ODMA_DEADLOCK_NUM(_v)			((_v) << 1)
#define ODMA_DEADLOCK_NUM_MASK			(0x7fffffff << 1)
#define ODMA_DEADLOCK_NUM_EN			(1 << 0)

#define WDMA_BUS_CTRL				0x0110
#define ODMA_AWCACHE_P3				((_v) << 12)
#define ODMA_AWCACHE_P3_MASK			(0xf << 12)
#define ODMA_AWCACHE_P2				((_v) << 8)
#define ODMA_AWCACHE_P2_MASK			(0xf << 8)
#define ODMA_AWCACHE_P1				((_v) << 4)
#define ODMA_AWCACHE_P1_MASK			(0xf << 4)
#define ODMA_AWCACHE_P0				((_v) << 0)
#define ODMA_AWCACHE_P0_MASK			(0xf << 0)

#define WDMA_LLC_CTRL				0x0114
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

#define WDMA_PERF_CTRL				0x0120
#define ODMA_DEGRADATION_TIME(_v)		((_v) << 16)
#define ODMA_DEGRADATION_TIME_MASK		(0xFFFF << 16)
#define ODMA_IC_MAX_DEG(_v)			((_v) << 4)
#define ODMA_IC_MAX_DEG_MASK			(0xFF << 4)
#define ODMA_DEGRADATION_EN			(1 << 0)

/* _n: [0,7], _v: [0x0, 0xF] */
#define WDMA_QOS_LUT_LOW			0x0130
#define WDMA_QOS_LUT_HIGH			0x0134
#define ODMA_QOS_LUT(_n, _v)			((_v) << (4*(_n)))
#define ODMA_QOS_LUT_MASK(_n)			(0xF << (4*(_n)))

#define WDMA_DYNAMIC_GATING_EN			0x0140
#define ODMA_SRAM_CG_EN				(1U << 31)
#define ODMA_DG_EN(_n, _v)			((_v) << (_n))
#define ODMA_DG_EN_MASK(_n)			(1 << (_n))
#define ODMA_DG_EN_ALL				(0x7FFFFFFF << 0)

#define WDMA_MST_SECURITY			0x200
#define WDMA_SLV_SECURITY			0x204

#define WDMA_DEBUG_CTRL				0x0300
#define ODMA_DEBUG_SEL(_v)			((_v) << 16)
#define ODMA_DEBUG_EN				(0x1 << 0)

#define WDMA_DEBUG_DATA				0x0304

#define WDMA_PSLV_ERR_CTRL			0x030c
#define ODMA_PSLVERR_CTRL			(1 << 0)


#define WDMA_DEBUG_ADDR_ERR			0x0730
#define ODMA_ERR_ADDR_C2			(1 << 3)
#define ODMA_ERR_ADDR_Y2			(1 << 2)
#define ODMA_ERR_ADDR_C8			(1 << 1)
#define ODMA_ERR_ADDR_Y8			(1 << 0)
#define ODMA_ERR_ADDR_GET(_v)			(((_v) >> 0) & 0xF)

#define WDMA_CONFIG_ERR_STATUS			0x0740
#define ODMA_CFG_ERR_STRIDE1			(1 << 12)
#define ODMA_CFG_ERR_STRIDE0			(1 << 11)
#define ODMA_CFG_ERR_CHROM_STRIDE		(1 << 10)
#define ODMA_CFG_ERR_BASE_ADDR_P3		(1 << 9)
#define ODMA_CFG_ERR_BASE_ADDR_P2		(1 << 8)
#define ODMA_CFG_ERR_BASE_ADDR_P1		(1 << 7)
#define ODMA_CFG_ERR_BASE_ADDR_P0		(1 << 6)
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
#define WDMA_BWL_ENABLE				0xd00
#define ODMA_LIMIT_EN_HDR			(1 << 4)
#define ODMA_LIMIT_EN_PLD			(1 << 0)

/* number of cycles in 1us : ACLK base */
#define WDMA_BWL_FREQ				0xd04
#define ODMA_NUM_CYCLES(_v)			((_v) << 0)
#define ODMA_NUM_CYCLES_MASK			(0xfff << 0)

#define WDMA_BWL_LIMIT_CTRL_PLD			0xd10
#define ODMA_AVG_BW_LIMIT(_v)			((_v) << 16)
#define ODMA_AVG_BW_LIMIT_MASK			(0xffff << 16)
#define ODMA_SLOT_BW_LIMIT(_v)			((_v) << 0)
#define ODMA_SLOT_BW_LIMIT_MASK			(0xffff << 0)

#define WDMA_BWL_COMPENSATE_CTRL_PLD		0xd14
#define ODMA_COMPENSATION_LIMIT(_v)		((_v) << 16)
#define ODMA_COMPENSATION_LIMIT_MASK		(0xffff << 16)
#define ODMA_PERIOD(_v)				((_v) << 0)
#define ODMA_PERIOD_MASK			(0xfff << 0)

#define WDMA_BWL_LIMIT_CTRL_HDR			0xd20
/* same field with WDMA_BWL_LIMIT_CTRL_PLD */

#define WDMA_BWL_COMPENSATE_CTRL_HDR		0xd24
/* same field with WDMA_BWL_COMPENSATE_CTRL_PLD */

/*
 *-------------------------------------------------------------------
 * DPU_DMA: CGC SFR List
 * base address(DPUF0) : 0x1990_0000
 * base address(DPUF1) : 0x19D0_0000
 * < Layer.offset >
 *  CGC
 *  0xE000
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
#define CGC_IN_CTRL_1				(0x000C)
#define START_EN_SET0(_v)			((_v) << 0)
#define START_EN_SET0_MASK			(1 << 0)
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
 * DPU_DMA: RCD SFR list
 * RCD0 base address : 0x1990_C000
 * RCD1 base address : 0x19D0_C000
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
#define RCD_SRC_OFFSET_Y(_v)			((_v) << 16)
#define RCD_SRC_OFFSET_Y_MASK			(0x3FFF << 16)
#define RCD_SRC_OFFSET_X(_v)			((_v) << 0)
#define RCD_SRC_OFFSET_X_MASK			(0x3FFF << 0)

#define RCD_IMG_SIZE				(0x001C)
#define RCD_IMG_HEIGHT(_v)			((_v) << 16)
#define RCD_IMG_HEIGHT_MASK			(0x3FFF << 16)
#define RCD_IMG_WIDTH(_v)			((_v) << 0)
#define RCD_IMG_WIDTH_MASK			(0x3FFF << 0)

#define RCD_BLOCK_OFFSET			(0x0020)
#define RCD_BLK_OFFSET_Y(_v)			((_v) << 16)
#define RCD_BLK_OFFSET_X(_v)			((_v) << 0)

#define RCD_BLOCK_SIZE				(0x0024)
#define RCD_BLK_HEIGHT(_v)			((_v) << 16)
#define RCD_BLK_WIDTH(_v)			((_v) << 0)

#define RCD_BLOCK_VALUE				(0x0028)
#define RCD_BASEADDR_P0				(0x0040)
#define RCD_RECOVERY_CTRL			(0x0070)
#define RCD_DEADLOCK_CTRL			(0x0100)
#define RCD_DEADLOCK_NUM(_v)			((_v) << 1)
#define RCD_DEADLOCK_NUM_MASK			(0x7FFFFFFFU << 1)
#define RCD_DEADLOCK_NUM_EN			(1 << 0)

#define RCD_BUS_CTRL				(0x0110)
#define RCD_LLC_CTRL				(0x0114)
#define RCD_PERF_CTRL				(0x0120)
#define RCD_QOS_LUT_LOW				(0x0130)
#define RCD_QOS_LUT_HIGH			(0x0134)
#define RCD_DYNAMIC_GATING_EN			(0x0140)
#define RCD_SRAM_CG_EN				(1U << 31)
#define RCD_DG_EN_ALL				(0x7FFFFFFF << 0)
#define RCD_MST_SECURITY			(0x0200)
#define RCD_SLV_SECURITY			(0x0204)
#define RCD_DEBUG_CTRL				(0x0300)
#define RCD_DEBUG_DATA				(0x0304)
#define RCD_IN_REQ_DEST				(0x0308)
#define RCD_PSLV_ERR_CTRL			(0x030C)

/*
 *-------------------------------------------------------------------
 * DPP(L0~L6, WB) SFR list
 * base address : 0x1993_0000
 * base address : 0x19D3_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5      L6	    WB
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000  0x6000  0x8000
 *-------------------------------------------------------------------
 */
#define DPP_COM_SHD_OFFSET			0x0100

#define DPP_COM_VERSION				0x0000
#define DPP_VERSION				0x07020000

#define DPP_COM_SWRST_CON			0x0004
#define DPP_SRESET				(1 << 0)

#define DPP_COM_QCH_CON				0x0008
#define DPP_QACTIVE				(1 << 0)

#define DPP_COM_PSLVERR_CON			0x000c
#define DPP_PSLVERR_EN				(1 << 0)

#define DPP_COM_IRQ_CON				0x0010
#define DPP_IRQ_EN				(1 << 0)

#define DPP_COM_IRQ_MASK			0x0014
#define DPP_CFG_ERROR_MASK			(1 << 1)
#define DPP_FRM_DONE_MASK			(1 << 0)
#define DPP_ALL_IRQ_MASK			(0x3 << 0)

#define DPP_COM_IRQ_STATUS			0x0018
#define DPP_CFG_ERROR_IRQ			(1 << 1)
#define DPP_FRM_DONE_IRQ			(1 << 0)
#define DPP_ALL_IRQ_CLEAR			(0x3 << 0)

#define DPP_COM_CFG_ERROR_STATUS		0x001c
#define DPP_CFG_ERR_SCL_POS			(1 << 4)
#define DPP_CFG_ERR_SCL_RATIO			(1 << 3)
#define DPP_CFG_ERR_ODD_SIZE			(1 << 2)
#define DPP_CFG_ERR_MAX_SIZE			(1 << 1)
#define DPP_CFG_ERR_MIN_SIZE			(1 << 0)

#define DPP_COM_DBG_CON				0x0028
#define DPP_DBG_SEL(_v)				((_v) << 16)
#define DPP_DBG_EN				(1 << 0)

#define DPP_COM_DBG_STATUS			0x002c

#define DPP_COM_OP_STATUS			0x0030
#define DPP_OP_STATUS				(1 << 0)

#define DPP_COM_TZPC				0x0034
#define DPP_TZPC				(1 << 0)

#define DPP_COM_IO_CON				0x0038
#define DPP_ALPHA_SEL(_v)			((_v) << 7)
#define DPP_ALPHA_SEL_MASK			(1 << 7)
#define DPP_BPC_MODE(_v)			((_v) << 6)
#define DPP_BPC_MODE_MASK			(1 << 6)
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

#define DPP_COM_IMG_SIZE			0x003c
#define DPP_IMG_HEIGHT(_v)			((_v) << 16)
#define DPP_IMG_HEIGHT_MASK			(0x3FFF << 16)
#define DPP_IMG_WIDTH(_v)			((_v) << 0)
#define DPP_IMG_WIDTH_MASK			(0x3FFF << 0)

#define DPP_COM_CSC_CON				0x0040
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
#define DPP_COM_CSC_COEF0			0x0044
#define DPP_COM_CSC_COEF1			0x0048
#define DPP_COM_CSC_COEF2			0x004c
#define DPP_COM_CSC_COEF3			0x0050
#define DPP_COM_CSC_COEF4			0x0054
#define DPP_CSC_COEF_H(_v)			((_v) << 16)
#define DPP_CSC_COEF_H_MASK			(0xFFFFU << 16)
#define DPP_CSC_COEF_L(_v)			((_v) << 0)
#define DPP_CSC_COEF_L_MASK			(0xFFFF << 0)
#define DPP_CSC_COEF_XX(_n, _v)			((_v) << (0 + (16 * (_n))))
#define DPP_CSC_COEF_XX_MASK(_n)		(0xFFF << (0 + (16 * (_n))))

#define DPP_COM_SUB_CON				0x0060
#define DPP_UV_OFFSET_Y(_v)			((_v) << 4)
#define DPP_UV_OFFSET_Y_MASK			(0x7 << 4)
#define DPP_UV_OFFSET_X(_v)			((_v) << 0)
#define DPP_UV_OFFSET_X_MASK			(0x7 << 0)

#define DPP_COM_SUB_CON2			0x0064
#define DPP_X_ODD				(1 << 16)
#define DPP_Y_ODD				(1 << 0)

/* SCL */
#define DPP_COM_SCL_CTRL                        0x0080
#define DPP_SCL_COEF_SEL(_v)                    ((_v) << 4)
#define DPP_SCL_COEF_SEL_MASK                   (0x3 << 4)
#define DPP_SCL_ENABLE(_v)                      ((_v) << 0)
#define DPP_SCL_ENABLE_MASK                     (0x1 << 0)

#define DPP_COM_SCL_SCALED_IMG_SIZE             0x0084
#define DPP_SCL_SCALED_IMG_HEIGHT(_v)           ((_v) << 16)
#define DPP_SCL_SCALED_IMG_HEIGHT_MASK          (01FFF << 16)
#define DPP_SCL_SCALED_IMG_WIDTH(_v)            ((_v) << 0)
#define DPP_SCL_SCALED_IMG_WIDTH_MASK           (0x1FFF << 0)

#define DPP_COM_SCL_H_RATIO                0x0088
#define DPP_SCL_H_RATIO(_v)                     ((_v) << 0)
#define DPP_SCL_H_RATIO_MASK                    (0x7FFFFF << 0)

#define DPP_COM_SCL_V_RATIO                0x008C
#define DPP_SCL_V_RATIO(_v)                     ((_v) << 0)
#define DPP_SCL_V_RATIO_MASK                    (0x7FFFFF << 0)

/* Initial Phase : S11.20 */
#define DPP_COM_SCL_HPOSITION                   0x0090
#define DPP_SCL_HPOS(_v)                        ((_v) << 0)
#define DPP_SCL_HPOS_MASK                       (0xFFFFFFFF << 0)

#define DPP_COM_SCL_VPOSITION                   0x0094
#define DPP_SCL_VPOS(_v)                        ((_v) << 0)
#define DPP_SCL_VPOS_MASK                       (0xFFFFFFFF << 0)

#define DPP_POS_I(_v)                           ((_v) << 20)
#define DPP_POS_I_MASK                          (0xFFF << 20)
#define DPP_POS_I_GET(_v)                       (((_v) >> 20) & 0xFFF)
#define DPP_POS_F(_v)                           ((_v) << 0)
#define DPP_POS_F_MASK                          (0xFFFFF << 0)
#define DPP_POS_F_GET(_v)                       (((_v) >> 0) & 0xFFFFF)


#define DPP_COM_SCL_STATUS                      0x0098
#define DPP_SCL_SCALING_STATUS_GET(_v)          (((_v) >> 0) & 0x3)
#define SCL_STATUS_IDLE                         (0)
#define SCL_STATUS_QUEUED                       (1)
#define SCL_STATUS_BUSY                         (3)

/*
 *-------------------------------------------------------------------
 * DPP_SCL_COEF(0~3) + ALPHA SFR list
 * base address : 0x1994_0000(DPUF0_DPP1), 0x19D4_0000(DPUF1_DPP1)
 * < Coef.offset >
 *  COEF0   COEF1   COEF2   COEF3
 *  0x0000  0x1000  0x2000  0x3000
 *-------------------------------------------------------------------
 */
#define DPP_SCL_SHD_OFFSET                      0x0200 /* SCL_COEF */

/* coef_id = [0,1,2,3] */
#define SCL_COEF_OFFS(_id)                      (0x0000 + 0x1000 * (_id))

#define DPP_SCL_Y_VCOEF_0A(_id)                 (SCL_COEF_OFFS(_id) + 0x0000)
#define DPP_SCL_Y_HCOEF_0A(_id)                 (SCL_COEF_OFFS(_id) + 0x0090)
#define DPP_SCL_COEF(_v)                        ((_v) << 0)
#define DPP_SCL_COEF_MASK                       (0x7FF << 0)
#define DPP_H_COEF(id, n, s)                    ((SCL_COEF_OFFS(id) + 0x0090) \
                                                + (n) * 0x4 + (s) * 0x24)
#define DPP_V_COEF(id, n, s)                    ((SCL_COEF_OFFS(id) + 0x0000) \
                                                + (n) * 0x4 + (s) * 0x24)

/* core_id = [0,1] */
#define SCL_DBG_OFFS(_id)                       (0x4000 + 0x1000 * (_id))

#define DPP_SCL_CORE_OP_STATUS(_id)             (SCL_DBG_OFFS(_id) + 0x0000)
#define SCL_CORE_OP_LAYER_GET(_v)               (((_v) >> 4) & 0x7)
#define SCL_CORE_OP_STATUS_GET(_v)              (((_v) >> 0) & 0x1)
#define SCL_CORE_OP_STATUS_IDLE                 (0)
#define SCL_CORE_OP_STATUS_BUSY                 (1)

#define DPP_SCL_CORE_DBG_CON(_id)               (SCL_DBG_OFFS(_id) + 0x0004)
#define SCL_CORE_DBG_EN(_v)                     ((_v) << 0)
#define SCL_CORE_DBG_EN_MASK                    (1 << 0)

#define DPP_SCL_CORE_DBG_WR_POS(_id)            (SCL_DBG_OFFS(_id) + 0x0008)
#define WR_VSCL_YPOS_GET(_v)                    (((_v) >> 16) & 0x3FFF)
#define WR_VSCL_HPOS_GET(_v)                    (((_v) >> 0) & 0x3FFF)

#define DPP_SCL_CORE_DBG_VSCL_POS(_id)          (SCL_DBG_OFFS(_id) + 0x000C)
#define VSCL_YPOS_GET(_v)                       (((_v) >> 16) & 0x3FFF)
#define VSCL_HPOS_GET(_v)                       (((_v) >> 0) & 0x3FFF)

#define DPP_SCL_CORE_DBG_HSCL_POS(_id)          (SCL_DBG_OFFS(_id) + 0x0010)
#define HSCL_YPOS_GET(_v)                       (((_v) >> 16) & 0x3FFF)
#define HSCL_HPOS_GET(_v)                       (((_v) >> 0) & 0x3FFF)

/*
 *-------------------------------------------------------------------
 * SRAMCON(L0~L6, D, G) SFR list

 * base address : DPUF0: 0x1995_0000
 *		  DPUF1: 0x19D5_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5      L12
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000  0xC000
 *-------------------------------------------------------------------
 */

#define SRAMC_L_COM_SHD_OFFSET                  (0x0800)

#define SRAMC_L_COM_TZPC                        (0x0008)
#define SRAMC_TZPC                              (1 << 0)

#define SRAMC_L_COM_PSLVERR_CON                 (0x000C)
#define SRAMC_PSLVERR_EN                        (1 << 0)

#define SRAMC_L_COM_MODE_REG                    (0x0010)
#define SRAMC_SCL_ALPHA_ENABLE(_v)              (((_v) & 0x1) << 13)
#define SRAMC_SCL_ALPHA_ENABLE_MASK             (0x1 << 13)
#define SRAMC_SCL_ENABLE(_v)                    (((_v) & 0x1) << 12)
#define SRAMC_SCL_ENABLE_MASK                   (0x1 << 12)
#define SRAMC_COMP_YUV_MODE(_v)                 (((_v) & 0x1) << 10)
#define SRAMC_COMP_YUV_MODE_MASK                (0x1 << 10)
#define SRAMC_COMP_ENABLE(_v)                   (((_v) & 0x1) << 8)
#define SRAMC_COMP_ENABLE_MASK                  (0x1 << 8)
#define SRAMC_ROT_ENABLE(_v)                    (((_v) & 0x1) << 4)
#define SRAMC_ROT_ENABLE_MASK                   (0x1 << 4)
#define SRAMC_YUV_MODE(_v)                      (((_v) & 0x1) << 2)
#define SRAMC_YUV_MODE_MASK                     (0x1 << 2)
#define SRAMC_FORMAT(_v)                        (((_v) & 0x3) << 0)
#define SRAMC_FORMAT_MASK                       (0x3 << 0)
#define SRAMC_FMT_RGB32BIT                      (0)     /* ARGB8888, AR */
#define SRAMC_FMT_RGB16BIT                      (1)     /* ARGB4444, RG */
#define SRAMC_FMT_YUV08BIT                      (2)     /* YUV 8bit */
#define SRAMC_FMT_YUV10BIT                      (3)     /* YUV 16bit */

#define SRAMC_L_COM_DST_POSITION_REG            (0x0014)
#define SRAMC_DST_BOTTOM(_v)                    ((_v) << 16)
#define SRAMC_DST_BOTTOM_MASK                   (0x3FFF << 16)
#define SRAMC_DST_TOP(_v)                       ((_v) << 0)
#define SRAMC_DST_TOP_MASK                      (0x3FFF << 0)

/*
 *-------------------------------------------------------------------
 * HDR_COMM(L0~L7) SFR list
 * base address : 0x19D6_0000(DPUF0_HDR_COMM), 0x1AF6_0000(DPUF1_HDR_COMM)
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5      L6      L7
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000  0x6000  0x7000
 *-------------------------------------------------------------------
 */
#define LSI_COMM_SHD_OFFSET			0x0800

#define LSI_COMM_LC_CON				0x0000
#define COMM_LC_CAPTURE_MASK			(1 << 2)
#define COMM_LC_MODE(_v)			((_v) << 1)
#define COMM_LC_MODE_MASK			(1 << 1)
#define COMM_LC_EN(_v)				((_v) << 0)
#define COMM_LC_EN_MASK				(1 << 0)

#define LSI_COMM_DBG_CON			0x0004
#define COMM_DBG_EN(_v)				((_v) << 0)
#define COMM_DBG_EN_MASK			(1 << 0)

#define LSI_COMM_IO_CON				0x000c
#define COMM_BPC_MODE(_v)			((_v) << 6)
#define COMM_BPC_MODE_MASK			(1 << 6)
#define COMM_IMG_FORMAT(_v)			((_v) << 0)
#define COMM_IMG_FORMAT_MASK			(0x7 << 0)
#define COMM_IMG_FORMAT_ARGB8888		(0 << 0)
#define COMM_IMG_FORMAT_ARGB8101010		(1 << 0)
#define COMM_IMG_FORMAT_YUV420_8P		(2 << 0)
#define COMM_IMG_FORMAT_YUV420_P010		(3 << 0)
#define COMM_IMG_FORMAT_YUV422_8P		(5 << 0)
#define COMM_IMG_FORMAT_YUV422_P210		(6 << 0)

#define LSI_COMM_HDR_CON			0x0010
#define COMM_MULT_EN(_v)			((_v) << 0)
#define COMM_MULT_EN_MASK			(1 << 0)

#define LSI_COMM_DITH_CON			0x0014
#define COMM_DITH_EN(_v)			((_v) << 4)
#define COMM_DITH_EN_MASK			(0x3 << 4)
#define COMM_DITH_MASK_SEL(_v)			((_v) << 1)
#define COMM_DITH_MASK_SEL_MASK			(0x7 << 1)
#define COMM_DITH_MASK_SPIN(_v)			((_v) << 0)
#define COMM_DITH_MASK_SPIN_MASK		(0x1 << 0)

#define LSI_COMM_PSLVERR_CON			0x0018
#define COMM_PSLVERR_EN				(1 << 0)

#define LSI_COMM_SIZE				0x0020
#define COMM_SFR_VSIZE(_v)			((_v) << 16)
#define COMM_SFR_VSIZE_MASK			(0x1FFF << 16)
#define COMM_SFR_HSIZE(_v)			((_v) << 0)
#define COMM_SFR_HSIZE_MASK			(0x1FFF << 0)

#define LSI_COMM_DITH_LINE_CNT			0x0054
#define COMM_SFR_LINE_CNT			(0x1FFF << 0)
#define COMM_SFR_LINE_CNT_GET(_v)		(((_v) >> 0) & 0x1FFF)

#define LSI_COMM_DITH_DBG_DATA			0x0058

#define LSI_COMM_MCD_BYPASS			0x005C
#define COMM_MCD_BYPASS(_v)			((_v) << 0)
#define COMM_MCD_BYPASS_MASK			(1 << 0)

#endif
