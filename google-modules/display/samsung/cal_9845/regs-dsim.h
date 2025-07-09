/* SPDX-License-Identifier: GPL-2.0-only
 *
 * cal_9845/regs-dsim.h
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register definition file for Samsung MIPI-DSIM driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _REGS_SYSREG_DISP_H
#define _REGS_SYSREG_DISP_H

#define DISP_DPU_MIPI_PHY_CON			(0x0008)
/* _v : [0,1] */
#define SEL_RESET_DPHY_MASK(_v)			(0x1 << (4 + (_v)))
#define M_RESETN_M1_MASK			(0x1 << 1)
#define M_RESETN_M0_MASK			(0x1 << 0)

#endif /* _REGS_SYSREG_DISP_H */


#ifndef _REGS_DSIM_H
#define _REGS_DSIM_H

/*
 *-------------------------------------------------------------------
 * MIPI DSIM  registers
 * DSIM0's base address : 0x1C2C_0000
 * DSIM1's base address : 0x1C2D_0000
 *-------------------------------------------------------------------
 */

#define DSIM_VERSION					(0x0000)
#define DSIM_VERSION_GET(_v)				(((_v) >> 0) & 0xffffffff)

#define DSIM_SWRST					(0x0004)
#define DSIM_DPHY_RST					(1 << 16)
#define DSIM_SWRST_FUNCRST				(1 << 8)
#define DSIM_SWRST_RESET				(1 << 0)

#define DSIM_LINK_STATUS0				(0x0008)
#define DSIM_LINK_STATUS0_VIDEO_MODE_STATUS_GET(_x)	((_x >> 24) & 0x1)
#define DSIM_LINK_STATUS0_VM_LINE_CNT_GET(_x)		((_x >> 0) & 0x1FFF)

#define DSIM_LINK_STATUS1				(0x000C)
#define DSIM_LINK_STATUS1_CMD_MODE_STATUS_GET(_x)	((_x >> 26) & 0x1)
#define DSIM_LINK_STATUS1_CMD_LOCK_STATUS_GET(_x)	((_x >> 25) & 0x1)
#define DSIM_LINK_STATUS1_CMD_TRANSF_CNT_GET(_x)	((_x >> 0) & 0x1FFFFFF)
#define DSIM_STATUS_IDLE				(0)
#define DSIM_STATUS_ACTIVE				(1)

#define DSIM_LINK_STATUS2				(0x0010)

#define DSIM_LINK_STATUS3				(0x0014)
#define DSIM_LINK_STATUS3_PLL_STABLE_GET(_x)		((_x >> 0) & 0x1)
#define DSIM_LINK_STATUS3_PLL_STABLE			(1 << 0)

#define DSIM_MIPI_STATUS				(0x0018)
#define DSIM_MIPI_STATUS_FRM_PROCESSING			(1 << 29)
#define DSIM_MIPI_STATUS_FRM_DONE			(1 << 28)
#define DSIM_MIPI_STATUS_SHADOW_REG_UP_EN		(1 << 25)
#define DSIM_MIPI_STATUS_SHADOW_REG_UP_DONE		(1 << 24)
#define DSIM_MIPI_STATUS_INSTANT_OFF_REQ		(1 << 21)
#define DSIM_MIPI_STATUS_INSTANT_OFF_ACK		(1 << 20)
#define DSIM_MIPI_STATUS_TE				(1 << 0)

#define DSIM_DPHY_STATUS				(0x001C)
#define DSIM_DPHY_STATUS_TX_READY_HSCLK			(1 << 10)
#define DSIM_DPHY_STATUS_ULPS_CLK			(1 << 9)
#define DSIM_DPHY_STATUS_STOPSTATE_CLK			(1 << 8)
#define DSIM_DPHY_STATUS_ULPS_DATA_LANE_GET(x)		(((x) >> 4) & 0xF)
#define DSIM_DPHY_STATUS_ULPS_DAT(_x)			(((_x) & 0xF) << 4)
#define DSIM_DPHY_STATUS_STOPSTATE_DAT(_x)		(((_x) & 0xF) << 0)

#define DSIM_CLK_CTRL					(0x0020)
#define DSIM_CLK_CTRL_CLOCK_SEL				(1 << 26)
#define DSIM_CLK_CTRL_NONCONT_CLOCK_LANE		(1 << 25)
#define DSIM_CLK_CTRL_CLKLANE_ONOFF			(1 << 24)
#define DSIM_CLK_CTRL_TX_REQUEST_HSCLK			(1 << 20)
#define DSIM_CLK_CTRL_WORDCLK_EN			(1 << 17)
#define DSIM_CLK_CTRL_ESCCLK_EN				(1 << 16)
#define DSIM_CLK_CTRL_LANE_ESCCLK_EN(_x)		((_x) << 8)
#define DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK		(0x1F << 8)
#define DSIM_CLK_CTRL_ESC_PRESCALER(_x)			((_x) << 0)
#define DSIM_CLK_CTRL_ESC_PRESCALER_MASK		(0xFF << 0)

#define DSIM_DESKEW_CTRL				(0x0024)
#define DSIM_DESKEW_CTRL_HW_EN				(1 << 15)
#define DSIM_DESKEW_CTRL_HW_POSITION(_x)		((_x) << 14)
#define DSIM_DESKEW_CTRL_HW_POSITION_MASK		(0x1 << 14)
#define DSIM_DESKEW_CTRL_HW_INTERVAL(_x)		((_x) << 2)
#define DSIM_DESKEW_CTRL_HW_INTERVAL_MASK		(0xFFF << 2)
#define DSIM_DESKEW_CTRL_HW_INIT			(1 << 1)
#define DSIM_DESKEW_CTRL_SW_SEND			(1 << 0)

/* Time out register */
#define DSIM_TIMEOUT					(0x0028)
#define DSIM_TIMEOUT_BTA_TOUT(_x)			((_x) << 16)
#define DSIM_TIMEOUT_BTA_TOUT_MASK			(0xFFFF << 16)
#define DSIM_TIMEOUT_LPRX_TOUT(_x)			((_x) << 0)
#define DSIM_TIMEOUT_LPRX_TOUT_MASK			(0xFFFF << 0)

/* Escape mode register */
#define DSIM_ESCMODE					(0x002C)
#define DSIM_ESCMODE_STOP_STATE_CNT(_x)			((_x) << 21)
#define DSIM_ESCMODE_STOP_STATE_CNT_MASK		(0x7FF << 21)
#define DSIM_ESCMODE_FORCE_STOP_STATE			(1 << 20)
#define DSIM_ESCMODE_FORCE_BTA				(1 << 16)
#define DSIM_ESCMODE_CMD_LPDT				(1 << 7)
#define DSIM_ESCMODE_TRIGGER_RST			(1 << 4)
#define DSIM_ESCMODE_TX_ULPS_DATA			(1 << 3)
#define DSIM_ESCMODE_TX_ULPS_DATA_EXIT			(1 << 2)
#define DSIM_ESCMODE_TX_ULPS_CLK			(1 << 1)
#define DSIM_ESCMODE_TX_ULPS_CLK_EXIT			(1 << 0)

#define DSIM_NUM_OF_TRANSFER				(0x0030)
#define DSIM_NUM_OF_TRANSFER_PER_FRAME(_x)		((_x) << 0)
#define DSIM_NUM_OF_TRANSFER_PER_FRAME_MASK		(0xFFFFF << 0)
#define DSIM_NUM_OF_TRANSFER_PER_FRAME_GET(x)		(((x) >> 0) & 0xFFFFF)

#define DSIM_UNDERRUN_CTRL				(0x0034)
#define DSIM_UNDERRUN_CTRL_CM_UNDERRUN_LP_REF(_x)	((_x) << 0)
#define DSIM_UNDERRUN_CTRL_CM_UNDERRUN_LP_REF_MASK	(0xFFFF << 0)

#define DSIM_THRESHOLD					(0x0038)
#define DSIM_THRESHOLD_LEVEL(_x)			((_x) << 0)
#define DSIM_THRESHOLD_LEVEL_MASK			(0xFFFF << 0)

/* Display image resolution register */
#define DSIM_RESOL					(0x003C)
#define DSIM_RESOL_VRESOL(x)				(((x) & 0x1FFF) << 16)
#define DSIM_RESOL_VRESOL_MASK				(0x1FFF << 16)
#define DSIM_RESOL_HRESOL(x)				(((x) & 0x1FFF) << 0)
#define DSIM_RESOL_HRESOL_MASK				(0x1FFF << 0)
#define DSIM_RESOL_LINEVAL_GET(_x)			(((_x) >> 16) & 0x1FFF)
#define DSIM_RESOL_HOZVAL_GET(_x)			(((_x) >> 0) & 0x1FFF)

/* Main display Hporch register */
#define DSIM_HPORCH					(0x0044)
#define DSIM_HPORCH_HFP(_x)				((_x) << 16)
#define DSIM_HPORCH_HFP_MASK				(0xFFFF << 16)
#define DSIM_HPORCH_HBP(_x)				((_x) << 0)
#define DSIM_HPORCH_HBP_MASK				(0xFFFF << 0)

/* Main display sync area register */
#define DSIM_SYNC					(0x0048)
#define DSIM_SYNC_VSA(_x)				((_x) << 16)
#define DSIM_SYNC_VSA_MASK				(0xFF << 16)
#define DSIM_SYNC_HSA(_x)				((_x) << 0)
#define DSIM_SYNC_HSA_MASK				(0xFFFF << 0)

/* Configuration register */
#define DSIM_CONFIG					(0x004C)
#define DSIM_CONFIG_PLL_CLOCK_GATING			(1 << 30)
#define DSIM_CONFIG_PLL_SLEEP				(1 << 29)
#define DSIM_CONFIG_PHY_SELECTION			(1 << 28)
#define DSIM_CONFIG_SYNC_INFORM				(1 << 27)
#define DSIM_CONFIG_BURST_MODE				(1 << 26)
#define DSIM_CONFIG_LP_FORCE_EN				(1 << 24)
#define DSIM_CONFIG_HSE_DISABLE				(1 << 23)
#define DSIM_CONFIG_HFP_DISABLE				(1 << 22)
#define DSIM_CONFIG_HBP_DISABLE				(1 << 21)
#define DSIM_CONFIG_HSA_DISABLE				(1 << 20)
#define DSIM_CONFIG_CPRS_EN				(1 << 19)
#define DSIM_CONFIG_VIDEO_MODE				(1 << 18)
#define DSIM_CONFIG_DISPLAY_MODE_GET(_x)		(((_x) >> 18) & 0x1)
#define DSIM_CONFIG_VC_ID(_x)				((_x) << 15)
#define DSIM_CONFIG_VC_ID_MASK				(0x3 << 15)
#define DSIM_CONFIG_PIXEL_FORMAT(_x)			((_x) << 9)
#define DSIM_CONFIG_PIXEL_FORMAT_MASK			(0x3F << 9)
#define DSIM_CONFIG_PER_FRAME_READ_EN(_x)		((_x) << 8)
#define DSIM_CONFIG_PER_FRAME_READ_EN_MASK		(0x1 << 8)
#define DSIM_CONFIG_EOTP_EN(_x)				((_x) << 7)
#define DSIM_CONFIG_EOTP_EN_MASK			(0x1 << 7)
#define DSIM_CONFIG_NUM_OF_DATA_LANE(_x)		((_x) << 5)
#define DSIM_CONFIG_NUM_OF_DATA_LANE_MASK		(0x3 << 5)
#define DSIM_CONFIG_LANES_EN(_x)			(((_x) & 0x1F) << 0)
#define DSIM_CONFIG_CLK_LANES_EN			(1 << 0)

/* Interrupt source register */
#define DSIM_INTSRC					(0x0050)
#define DSIM_INTSRC_PLL_STABLE				(1 << 31)
#define DSIM_INTSRC_SW_RST_RELEASE			(1 << 30)
#define DSIM_INTSRC_SFR_PL_FIFO_EMPTY			(1 << 29)
#define DSIM_INTSRC_SFR_PH_FIFO_EMPTY			(1 << 28)
#define DSIM_INTSRC_SFR_PH_FIFO_OVERFLOW		(1 << 27)
#define DSIM_INTSRC_SW_DESKEW_DONE			(1 << 26)
#define DSIM_INTSRC_BUS_TURN_OVER			(1 << 25)
#define DSIM_INTSRC_FRAME_DONE				(1 << 24)
#define DSIM_INTSRC_INVALID_SFR_VALUE			(1 << 23)
#define DSIM_INTSRC_ABNORMAL_CMD_ST			(1 << 22)
#define DSIM_INTSRC_LPRX_TOUT				(1 << 21)
#define DSIM_INTSRC_BTA_TOUT				(1 << 20)
#define DSIM_INTSRC_UNDER_RUN				(1 << 19)
#define DSIM_INTSRC_RX_DATA_DONE			(1 << 18)
#define DSIM_INTSRC_RX_TE				(1 << 17)
#define DSIM_INTSRC_RX_ACK				(1 << 16)
#define DSIM_INTSRC_ERR_RX_ECC				(1 << 15)
#define DSIM_INTSRC_RX_CRC				(1 << 14)
#define DSIM_INTSRC_VT_STATUS				(1 << 13)

/* Interrupt mask register */
#define DSIM_INTMSK					(0x0054)
#define DSIM_INTMSK_PLL_STABLE				(1 << 31)
#define DSIM_INTMSK_SW_RST_RELEASE			(1 << 30)
#define DSIM_INTMSK_SFR_PL_FIFO_EMPTY			(1 << 29)
#define DSIM_INTMSK_SFR_PH_FIFO_EMPTY			(1 << 28)
#define DSIM_INTMSK_SFR_PH_FIFO_OVERFLOW		(1 << 27)
#define DSIM_INTMSK_SW_DESKEW_DONE			(1 << 26)
#define DSIM_INTMSK_BUS_TURN_OVER			(1 << 25)
#define DSIM_INTMSK_FRAME_DONE				(1 << 24)
#define DSIM_INTMSK_INVALID_SFR_VALUE			(1 << 23)
#define DSIM_INTMSK_ABNRMAL_CMD_ST			(1 << 22)
#define DSIM_INTMSK_LPRX_TOUT				(1 << 21)
#define DSIM_INTMSK_BTA_TOUT				(1 << 20)
#define DSIM_INTMSK_UNDER_RUN				(1 << 19)
#define DSIM_INTMSK_RX_DATA_DONE			(1 << 18)
#define DSIM_INTMSK_RX_TE				(1 << 17)
#define DSIM_INTMSK_RX_ACK				(1 << 16)
#define DSIM_INTMSK_ERR_RX_ECC				(1 << 15)
#define DSIM_INTMSK_RX_CRC				(1 << 14)
#define DSIM_INTMSK_VT_STATUS				(1 << 13)

/* Packet Header FIFO register */
#define DSIM_PKTHDR					(0x0058)
#define DSIM_PKTHDR_BTA_TYPE(_x)			((_x) << 24)
#define DSIM_PKTHDR_DATA1(_x)				((_x) << 16)
#define DSIM_PKTHDR_DATA0(_x)				((_x) << 8)
#define DSIM_PKTHDR_ID(_x)				((_x) << 0)
#define DSIM_PKTHDR_DATA				(0x1FFFFFF << 0)

/* Payload FIFO register */
#define DSIM_PAYLOAD					(0x005C)

/* Read FIFO register */
#define DSIM_RXFIFO					(0x0060)

/* SFR control Register for Stanby & Shadow */
#define DSIM_SFR_CTRL					(0x0064)
#define DSIM_SFR_CTRL_SHADOW_REG_READ_EN		(1 << 1)
#define DSIM_SFR_CTRL_SHADOW_EN				(1 << 0)

/* FIFO status and control register */
#define DSIM_FIFOCTRL					(0x0068)
#define DSIM_FIFOCTRL_NUMBER_OF_PH_SFR(_x)		(((_x) & 0x3F) << 16)
#define DSIM_FIFOCTRL_NUMBER_OF_PH_SFR_GET(_x)		(((_x) >> 16) & 0x3F)
#define DSIM_FIFOCTRL_EMPTY_RX				(1 << 12)
#define DSIM_FIFOCTRL_FULL_PH_SFR			(1 << 11)
#define DSIM_FIFOCTRL_EMPTY_PH_SFR			(1 << 10)
#define DSIM_FIFOCTRL_FULL_PL_SFR			(1 << 9)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR			(1 << 8)

#define DSIM_LP_SCATTER					(0x006C)
#define DSIM_LP_SCATTER_PATTERN(_x)			((_x) << 16)
#define DSIM_LP_SCATTER_PATTERN_MASK			(0xFFFF << 16)
#define DSIM_LP_SCATTER_EN				(1 << 0)

#define DSIM_S3D_CTRL					(0x0070)
#define DSIM_S3D_CTRL_3D_PRESENT			(1 << 11)
#define DSIM_S3D_CTRL_3D_ORDER				(1 << 5)
#define DSIM_S3D_CTRL_3D_VSYNC				(1 << 4)
#define DSIM_S3D_CTRL_3D_FORMAT(_x)			(((_x) & 0x3) << 2)
#define DSIM_S3D_CTRL_3D_FORMAT_GET(_x)			(((_x) >> 2) & 0x3)
#define DSIM_S3D_CTRL_3D_MODE(_x)			(((_x) & 0x3) << 0)
#define DSIM_S3D_CTRL_3D_MODE_GET(_x)			(((_x) >> 0) & 0x3)

/* Multi slice setting register */
#define DSIM_CPRS_CTRL					(0x0074)
#define DSIM_CPRS_CTRL_MULI_SLICE_PACKET(_x)		((_x) << 3)
#define DSIM_CPRS_CTRL_MULI_SLICE_PACKET_MASK		(0x1 << 3)
#define DSIM_CPRS_CTRL_NUM_OF_SLICE(_x)			((_x) << 0)
#define DSIM_CPRS_CTRL_NUM_OF_SLICE_MASK		(0x7 << 0)
#define DSIM_CPRS_CTRL_NUM_OF_SLICE_GET(_x)		(((_x) >> 0) & 0x7)

/* Slice01 size register */
#define DSIM_SLICE01					(0x0078)
#define DSIM_SLICE01_SIZE_OF_SLICE1(_x)			((_x) << 16)
#define DSIM_SLICE01_SIZE_OF_SLICE1_MASK		(0x1FFF << 16)
#define DSIM_SLICE01_SIZE_OF_SLICE1_GET(_x)		(((_x) >> 16) & 0x1FFF)
#define DSIM_SLICE01_SIZE_OF_SLICE0(_x)			((_x) << 0)
#define DSIM_SLICE01_SIZE_OF_SLICE0_MASK		(0x1FFF << 0)
#define DSIM_SLICE01_SIZE_OF_SLICE0_GET(_x)		(((_x) >> 0) & 0x1fff)

/* Slice23 size register */
#define DSIM_SLICE23					(0x007C)
#define DSIM_SLICE23_SIZE_OF_SLICE3(_x)			((_x) << 16)
#define DSIM_SLICE23_SIZE_OF_SLICE3_MASK		(0x1FFF << 16)
#define DSIM_SLICE23_SIZE_OF_SLICE3_GET(_x)		(((_x) >> 16) & 0x1FFF)
#define DSIM_SLICE23_SIZE_OF_SLICE2(_x)			((_x) << 0)
#define DSIM_SLICE23_SIZE_OF_SLICE2_MASK		(0x1FFF << 0)
#define DSIM_SLICE23_SIZE_OF_SLICE2_GET(_x)		(((_x) >> 0) & 0x1FFF)

/* Command configuration register */
#define DSIM_CMD_CONFIG					(0x0080)
#define DSIM_CMD_CONFIG_PKT_GO_RDY			(1 << 17)
#define DSIM_CMD_CONFIG_PKT_GO_EN			(1 << 16)
#define DSIM_CMD_CONFIG_MULTI_CMD_PKT_EN		(1 << 8)
#define DSIM_CMD_CONFIG_MULTI_PKT_CNT(_x)		((_x) << 0)
#define DSIM_CMD_CONFIG_MULTI_PKT_CNT_MASK		(0x3F << 0)

/* TE based command register */
#define DSIM_CMD_TE_CTRL0				(0x0084)
#define DSIM_CMD_TE_CTRL0_TIME_STABLE_VFP(_x)		((_x) << 0)
#define DSIM_CMD_TE_CTRL0_TIME_STABLE_VFP_MASK		(0xFFFF << 0)

/* TE based command register */
#define DSIM_CMD_TE_CTRL1				(0x0088)
#define DSIM_CMD_TE_CTRL1_TIME_TE_PROTECT_ON(_x)	((_x) << 16)
#define DSIM_CMD_TE_CTRL1_TIME_TE_PROTECT_ON_MASK	(0xFFFF << 16)
#define DSIM_CMD_TE_CTRL1_TIME_TE_TOUT(_x)		((_x) << 0)
#define DSIM_CMD_TE_CTRL1_TIME_TE_TOUT_MASK		(0xFFFF << 0)

/* Command Mode Status register */
#define DSIM_CMD_STATUS					(0x008C)
#define	DSIM_CMD_STATUS_ABNORMAL_CAUSE_ST_GET(x)	(((x) >> 0) & 0xFF)

/* Video Timer register for Video Mode */
#define DSIM_VIDEO_TIMER				(0x0090)
#define DSIM_VIDEO_TIMER_COMPENSATE(_x)			((_x) << 8)
#define DSIM_VIDEO_TIMER_COMPENSATE_MASK		(0xFFFFFF << 8)
#define DSIM_VIDEO_TIMER_VSTATUS_INTR_SEL(_x)		((_x) << 1)
#define DSIM_VIDEO_TIMER_VSTATUS_INTR_SEL_MASK		(0x3 << 1)
#define DSIM_VIDEO_TIMER_SYNC_MODE			(1 << 0)

/* BIST generation register */
#define	DSIM_BIST_CTRL0					(0x0094)
#define	DSIM_BIST_CTRL0_BIST_TE_INTERVAL(_x)		((_x) << 8)
#define	DSIM_BIST_CTRL0_BIST_TE_INTERVAL_MASK		(0xFFFFFF << 8)
#define	DSIM_BIST_CTRL0_BIST_PTRN_MOVE_EN		(1 << 4)
#define	DSIM_BIST_CTRL0_BIST_PTRN_MODE(_x)		((_x) << 1)
#define	DSIM_BIST_CTRL0_BIST_PTRN_MODE_MASK		(0x7 << 1)
#define	DSIM_BIST_CTRL0_BIST_EN				(1 << 0)

/* BIST generation register */
#define	DSIM_BIST_CTRL1					(0x0098)
#define	DSIM_BIST_CTRL1_BIST_PTRN_PRBS7_SEED(_x)	((_x) << 24)
#define	DSIM_BIST_CTRL1_BIST_PTRN_PRBS7_SEED_MASK	(0x7F << 24)
#define	DSIM_BIST_CTRL1_BIST_PTRN_USER_R(_x)		((_x) << 16)
#define	DSIM_BIST_CTRL1_BIST_PTRN_USER_R_MASK		(0XFF << 16)
#define	DSIM_BIST_CTRL1_BIST_PTRN_USER_G(_x)		((_x) << 8)
#define	DSIM_BIST_CTRL1_BIST_PTRN_USER_G_MASK		(0xFF << 8)
#define	DSIM_BIST_CTRL1_BIST_PTRN_USER_B(_x)		((_x) << 0)
#define	DSIM_BIST_CTRL1_BIST_PTRN_USER_B_MASK		(0xFF << 0)

/* DSIM to CSIS loopback register */
#define	DSIM_CSIS_LB					(0x009C)
#define DSIM_CSIS_LB_1BYTEPPI_MODE			(1 << 9)
#define	DSIM_CSIS_LB_CSIS_LB_EN				(1 << 8)
#define DSIM_CSIS_LB_CSIS_PH(_x)			((_x) << 0)
#define DSIM_CSIS_LB_CSIS_PH_MASK			(0xFF << 0)

#define DSIM_PLL_CTRL					(0x00A0)
#define DSIM_PHY_CTRL					(0x00B0)
#define DSIM_PHY_TIMING					(0x00D0)

/* IF CRC registers */
#define DSIM_IF_CRC_CTRL0				(0x00DC)
#define DSIM_IF_CRC_FAIL				(1 << 16)
#define DSIM_IF_CRC_PASS				(1 << 12)
#define DSIM_IF_CRC_VALID				(1 << 8)
#define DSIM_IF_CRC_CMP_MODE				(1 << 4)
#define DSIM_IF_CRC_CLEAR				(1 << 1)
#define DSIM_IF_CRC_EN					(1 << 0)

#define DSIM_IF_CRC_CTRL1				(0x00E0)
#define DSIM_IF_CRC_REF_R(_x)				((_x) << 16)
#define	DSIM_IF_CRC_RESULT_R_MASK			(0xFFFF << 0)
#define DSIM_IF_CRC_RESULT_R_GET(_x)			(((_x) >> 0) & 0xFFFF)

#define DSIM_IF_CRC_CTRL2				(0x00E4)
#define DSIM_IF_CRC_REF_G(_x)				((_x) << 16)
#define	DSIM_IF_CRC_RESULT_G_MASK			(0xFFFF << 0)
#define DSIM_IF_CRC_RESULT_G_GET(_x)			(((_x) >> 0) & 0xFFFF)

#define DSIM_IF_CRC_CTRL3				(0x00E8)
#define DSIM_IF_CRC_REF_B(_x)				((_x) << 16)
#define	DSIM_IF_CRC_RESULT_B_MASK			(0xFFFF << 0)
#define DSIM_IF_CRC_RESULT_B_GET(_x)			(((_x) >> 0) & 0xFFFF)

/* SA CRC registers */
#define DSIM_SA_CRC_CTRL0				(0x00EC)
#define DSIM_SA_CRC_FAIL				(1 << 16)
#define DSIM_SA_CRC_PASS				(1 << 12)
#define DSIM_SA_CRC_VALID				(1 << 8)
#define DSIM_SA_CRC_CMP_MODE				(1 << 4)
#define DSIM_SA_CRC_CLEAR				(1 << 1)
#define DSIM_SA_CRC_EN					(1 << 0)

#define DSIM_SA_CRC_CTRL1				(0x00F0)
#define DSIM_SA_CRC_REF_LN0(_x)				((_x) << 16)
#define	DSIM_SA_CRC_RESULT_LN0_MASK			(0xFFFF << 0)
#define DSIM_SA_CRC_RESULT_LN0_GET(_x)			(((_x) >> 0) & 0xFFFF)

#define DSIM_SA_CRC_CTRL2				(0x00F4)
#define DSIM_SA_CRC_REF_LN1(_x)				((_x) << 16)
#define	DSIM_SA_CRC_RESULT_LN1_MASK			(0xFFFF << 0)
#define DSIM_SA_CRC_RESULT_LN1_GET(_x)			(((_x) >> 0) & 0xFFFF)

#define DSIM_SA_CRC_CTRL3				(0x00F8)
#define DSIM_SA_CRC_REF_LN2(_x)				((_x) << 16)
#define	DSIM_SA_CRC_RESULT_LN2_MASK			(0xFFFF << 0)
#define DSIM_SA_CRC_RESULT_LN2_GET(_x)			(((_x) >> 0) & 0xFFFF)

#define DSIM_SA_CRC_CTRL4				(0x00FC)
#define DSIM_SA_CRC_REF_LN3(_x)				((_x) << 16)
#define	DSIM_SA_CRC_RESULT_LN3_MASK			(0xFFFF << 0)
#define DSIM_SA_CRC_RESULT_LN3_GET(_x)			(((_x) >> 0) & 0xFFFF)

#define DSIM_PLL_GUARD_TIMER				(0x0100)

/* Main display Vporch register */
#define DSIM_VPORCH					(0x0104)
#define DSIM_VPORCH_VFP_TOTAL(_x)			((_x) << 16)
#define DSIM_VPORCH_VFP_TOTAL_MASK			(0xFFFF << 16)
#define DSIM_VPORCH_VBP(_x)				((_x) << 0)
#define DSIM_VPORCH_VBP_MASK				(0xFFFF << 0)

#define DSIM_VFP_DETAIL					(0x0108)
#define DSIM_VPORCH_VFP_CMD_ALLOW(_x)			((_x) << 16)
#define DSIM_VPORCH_VFP_CMD_ALLOW_MASK			(0xFFFF << 16)
#define DSIM_VPORCH_STABLE_VFP(_x)			((_x) << 0)
#define DSIM_VPORCH_STABLE_VFP_MASK			(0xFFFF << 0)

#define DSIM_OPTION_SUITE				(0x010C)
#define	DSIM_OPTION_SUITE_SYNC_MODE_EN_MASK		(0x1 << 8)
#define	DSIM_OPTION_SUITE_OPT_VT_COND_MASK		(0x1 << 7)
#define	DSIM_OPTION_SUITE_OPT_FD_COND_MASK		(0x1 << 6)
#define	DSIM_OPTION_SUITE_OPT_KEEP_VFP_MASK		(0x1 << 5)
#define	DSIM_OPTION_SUITE_OPT_ALIVE_MODE_MASK		(0x1 << 4)
#define	DSIM_OPTION_SUITE_OPT_CM_EXT_CMD_ALLOW_MASK	(0x1 << 3)
#define	DSIM_OPTION_SUITE_OPT_USER_EXPIRE_VFP_MASK	(0x1 << 2)
#define	DSIM_OPTION_SUITE_EMIRROR_EN_MASK		(0x1 << 1)
#define	DSIM_OPTION_SUITE_CFG_UPDT_EN_MASK		(0x1 << 0)

#define DSIM_VT_HTIMING0				(0x0110)
#define DSIM_VT_HTIMING0_HSA_PERIOD(_x)			((_x) << 16)
#define DSIM_VT_HTIMING0_HACT_PERIOD(_x)		((_x) << 0)

#define DSIM_VT_HTIMING1				(0X0114)
#define DSIM_VT_HTIMING1_HFP_PERIOD(_x)			((_x) << 16)
#define DSIM_VT_HTIMING1_HBP_PERIOD(_x)			((_x) << 0)

#define DSIM_SYNCPKTHDR					(0X0118)
#define DSIM_SYNCPKTHDR_SYNC_PKT_HEADER(_x)		((_x) << 0)
#define DSIM_SYNCPKTHDR_SYNC_PKT_HEADER_MASK		(0xFFFFFF << 0)

#define DSIM_FIFO_STATUS				(0x011C)
#define DSIM_FIFO_CMD_PH_FIFO_REMAIN(_x)		((_x) << 16)
#define DSIM_FIFO_CMD_PH_FIFO_REMAIN_MASK		(0x3F << 16)
#define DSIM_FIFO_CMD_PL_FIFO_REMAIN(_x)		((_x) << 0)
#define DSIM_FIFO_CMD_PL_FIFO_REMAIN_MASK		(0xFFFF << 0)


/*
 *-------------------------------------------------------------------
 * MIPI DCPHY  registers
 * base address : 0x1C2E_0000
 * < offset >
 *  BIAS    MASTER0   MASTER1
 *  0x0000  0x0100    0x0900
 *-------------------------------------------------------------------
 */

/* DCPHY BIAS setting */
#define DSIM_PHY_BIAS_CON(_id)			(0x0000 + (4 * (_id)))

/* DCPHY MASTER Offsets */
#define DCPHY_M0_M4S0				(0x0100)
#define DCPHY_M1_M4S0				(0x0900)

/* DCPHY MASTER PLL setting */
#define DSIM_PHY_PLL_CON(_id)			(0x0000 + (4 * (_id)))
#define DSIM_PHY_PLL_CON0			(0x0000)
#define DSIM_PHY_PLL_CON1			(0x0004)
#define DSIM_PHY_PLL_CON2			(0x0008)
#define DSIM_PHY_PLL_CON3			(0x000C)
#define DSIM_PHY_PLL_CON4			(0x0010)
#define DSIM_PHY_PLL_CON5			(0x0014)
#define DSIM_PHY_PLL_CON6			(0x0018)
#define DSIM_PHY_PLL_CON7			(0x001C)
#define DSIM_PHY_PLL_CON8			(0x0020)

/* PLL_CON0 */
#define DSIM_PHY_PLL_EN(_x)			(((_x) & 0x1) << 12)
#define DSIM_PHY_PLL_EN_MASK			(0x1 << 12)
#define DSIM_PHY_PMS_S(_x)			(((_x) & 0x7) << 8)
#define DSIM_PHY_PMS_S_MASK			(0x7 << 8)
#define DSIM_PHY_PMS_P(_x)			(((_x) & 0x3F) << 0)
#define DSIM_PHY_PMS_P_MASK			(0x3F << 0)
/* PLL_CON1 */
#define DSIM_PHY_PMS_K(_x)			(((_x) & 0xFFFF) << 0)
#define DSIM_PHY_PMS_K_MASK			(0xFFFF << 0)
/* PLL_CON2 */
#define DSIM_PHY_USE_SDW_MASK			(0x1 << 15)
#define DSIM_PHY_M_ESCREF_EN			(1 << 14)
#define DSIM_PHY_DITHER_FOUT_MASK		(0x1 << 13)
#define DSIM_PHY_DITHER_FEED_EN_MASK		(0x1 << 12)
#define DSIM_PHY_PMS_M(_x)			(((_x) & 0x3FF) << 0)
#define DSIM_PHY_PMS_M_MASK			(0x3FF << 0)
/* PLL_CON3 */
#define DSIM_PHY_DITHER_MRR(_x)			(((_x) & 0x3F) << 8)
#define DSIM_PHY_DITHER_MRR_MASK		(0x3F << 8)
#define DSIM_PHY_DITHER_MFR(_x)			(((_x) & 0xFF) << 0)
#define DSIM_PHY_DITHER_MFR_MASK		(0xFF << 0)
/* PLL_CON4 */
#define DSIM_PHY_DITHER_RSEL(_x)		(((_x) & 0xF) << 12)
#define DSIM_PHY_DITHER_RSEL_MASK		(0xF << 12)
#define DSIM_PHY_DITHER_EN			(0x1 << 11)
#define DSIM_PHY_DITHER_FSEL(_x)		(((_x) & 0x1) << 10)
#define DSIM_PHY_DITHER_FSEL_MASK		(0x1 << 10)
#define DSIM_PHY_DITHER_BYPASS			(0x1 << 9)
#define DSIM_PHY_DITHER_AFC_ENB(_x)		(((_x) & 0x1) << 8)
#define DSIM_PHY_DITHER_AFC_ENB_MASK		(0x1 << 8)
#define DSIM_PHY_DITHER_EXTAFC(_x)		(((_x) & 0x1F) << 0)
#define DSIM_PHY_DITHER_EXTAFC_MASK		(0x1F << 0)
/* PLL_CON5 */
#define DSIM_PHY_DITHER_ICP(_x)			(((_x) & 0x3) << 4)
#define DSIM_PHY_DITHER_ICP_MASK		(0x3 << 4)
#define DSIM_PHY_DITHER_SEL_PF(_x)		(((_x) & 0x3) << 0)
#define DSIM_PHY_DITHER_SEL_PF_MASK		(0x3 << 0)
/* PLL_CON6 */
/*
 * WCLK_BUF_SFT_CNT = Roundup((Word Clock Period) / 38.46 + 2)
 */
#define DSIM_PHY_WCLK_BUF_SFT_CNT(_x)		(((_x) & 0xF) << 8)
#define DSIM_PHY_WCLK_BUF_SFT_CNT_MASK		(0xF << 8)
/* PLL_CON7 */
#define DSIM_PHY_PLL_LOCK_CNT(_x)		(((_x) & 0xFFFF) << 0)
#define DSIM_PHY_PLL_LOCK_CNT_MASK		(0xFFFF << 0)
/* PLL_CON8 */
#define DSIM_PHY_PLL_STB_CNT(x)			((x) << 0)
#define DSIM_PHY_PLL_STB_CNT_MASK		(0xFFFF << 0)

/* PLL_STAT0 */
#define DSIM_PHY_PLL_STAT0			(0x0040)
#define DSIM_PHY_PLL_LOCK_GET(x)		(((x) >> 0) & 0x1)

/* master clock lane General Control Register : GNR */
#define DSIM_PHY_MC_GNR_CON(_id)		(0x0200 + (4 * (_id)))
#define DSIM_PHY_MC_GNR_CON0			(0x0200)
#define DSIM_PHY_MC_GNR_CON1			(0x0204)

/* GNR0 */
#define DSIM_PHY_PHY_READY			(0x1 << 1)
#define DSIM_PHY_PHY_READY_GET(x)		(((x) >> 1) & 0x1)
#define DSIM_PHY_PHY_ENABLE			(1 << 0)
/* GNR1 */
#define DSIM_PHY_T_PHY_READY(_x)		(((_x) & 0xFFFF) << 0)
#define DSIM_PHY_T_PHY_READY_MASK		(0xFFFF << 0)


/* master clock lane Analog Block Control Register : ANA */
#define DSIM_PHY_MC_ANA_CON(_id)		(0x0208 + (4 * (_id)))
#define DSIM_PHY_MC_ANA_CON0			(0x0208)
#define DSIM_PHY_MC_ANA_CON1			(0x020C)
#define DSIM_PHY_MC_ANA_CON2			(0x0210)

/* ANA_CON0 */
#define DSIM_PHY_EDGE_CON_EN			(1 << 8)
#define DSIM_PHY_RES_UP(_x)			(((_x) & 0xF) << 4)
#define DSIM_PHY_RES_UP_MASK			(0xF << 4)
#define DSIM_PHY_RES_DN(_x)			(((_x) & 0xF) << 0)
#define DSIM_PHY_RES_DN_MASK			(0xF << 0)
/* ANA_CON1 */
#define DSIM_PHY_DPDN_SWAP(_x)			(((_x) & 0x1) << 12)
#define DSIM_PHY_DPDN_SWAP_MASK			(0x1 << 12)
/* ANA_CON2 */

/* master clock lane setting */
#define DSIM_PHY_MC_TIME_CON0			(0x0230)
#define DSIM_PHY_MC_TIME_CON1			(0x0234)
#define DSIM_PHY_MC_TIME_CON2			(0x0238)
#define DSIM_PHY_MC_TIME_CON3			(0x023C)
#define DSIM_PHY_MC_TIME_CON4			(0x0240)
#define DSIM_PHY_MC_DATA_CON0			(0x0244)
#define DSIM_PHY_MC_DESKEW_CON0			(0x0250)

/*
 * master data lane setting : D0 ~ D3
 * D0~D2 : COMBO
 * D3    : DPHY
 */
#define DSIM_PHY_MD_GNR_CON0(_x)		(0x0300 + (0x100 * (_x)))
#define DSIM_PHY_MD_GNR_CON1(_x)		(0x0304 + (0x100 * (_x)))
#define DSIM_PHY_MD_ANA_CON0(_x)		(0x0308 + (0x100 * (_x)))
#define DSIM_PHY_MD_ANA_CON1(_x)		(0x030C + (0x100 * (_x)))
#define DSIM_PHY_MD_ANA_CON2(_x)		(0x0310 + (0x100 * (_x)))
#define DSIM_PHY_MD_ANA_CON3(_x)		(0x0314 + (0x100 * (_x)))
#define DSIM_PHY_MD_TIME_CON0(_x)		(0x0330 + (0x100 * (_x)))
#define DSIM_PHY_MD_TIME_CON1(_x)		(0x0334 + (0x100 * (_x)))
#define DSIM_PHY_MD_TIME_CON2(_x)		(0x0338 + (0x100 * (_x)))
#define DSIM_PHY_MD_TIME_CON3(_x)		(0x033C + (0x100 * (_x)))
#define DSIM_PHY_MD_TIME_CON4(_x)		(0x0340 + (0x100 * (_x)))
#define DSIM_PHY_MD_DATA_CON0(_x)		(0x0344 + (0x100 * (_x)))

/* master data lane(COMBO) setting : D0 */
#define DSIM_PHY_MD0_TIME_CON0			(0x0330)
#define DSIM_PHY_MD0_TIME_CON1			(0x0334)
#define DSIM_PHY_MD0_TIME_CON2			(0x0338)
#define DSIM_PHY_MD0_TIME_CON3			(0x033C)
#define DSIM_PHY_MD0_TIME_CON4			(0x0340)
#define DSIM_PHY_MD0_DATA_CON0			(0x0344)

/* master data lane(COMBO) setting : D1 */
#define DSIM_PHY_MD1_TIME_CON0			(0x0430)
#define DSIM_PHY_MD1_TIME_CON1			(0x0434)
#define DSIM_PHY_MD1_TIME_CON2			(0x0438)
#define DSIM_PHY_MD1_TIME_CON3			(0x043C)
#define DSIM_PHY_MD1_TIME_CON4			(0x0440)
#define DSIM_PHY_MD1_DATA_CON0			(0x0444)

/* master data lane(COMBO) setting : D2 */
#define DSIM_PHY_MD2_TIME_CON0			(0x0530)
#define DSIM_PHY_MD2_TIME_CON1			(0x0534)
#define DSIM_PHY_MD2_TIME_CON2			(0x0538)
#define DSIM_PHY_MD2_TIME_CON3			(0x053C)
#define DSIM_PHY_MD2_TIME_CON4			(0x0540)
#define DSIM_PHY_MD2_DATA_CON0			(0x0544)

/* master data lane setting : D3 */
#define DSIM_PHY_MD3_TIME_CON0			(0x0630)
#define DSIM_PHY_MD3_TIME_CON1			(0x0634)
#define DSIM_PHY_MD3_TIME_CON2			(0x0638)
#define DSIM_PHY_MD3_TIME_CON3			(0x063C)
#define DSIM_PHY_MD3_TIME_CON4			(0x0640)
#define DSIM_PHY_MD3_DATA_CON0			(0x0644)


/* macros for DPHY timing controls */
/* MC/MD_TIME_CON0 */
#define DSIM_PHY_HSTX_CLK_SEL(_x)		(((_x) & 0x1) << 12)
#define DSIM_PHY_TLPX(_x)			(((_x) & 0xFF) << 4)
#define DSIM_PHY_TLPX_MASK			(0xFF << 4)
/* MD only */
#define DSIM_PHY_TLP_EXIT_SKEW(_x)		(((_x) & 0x3) << 2)
#define DSIM_PHY_TLP_EXIT_SKEW_MASK		(0x3 << 2)
#define DSIM_PHY_TLP_ENTRY_SKEW(_x)		(((_x) & 0x3) << 0)
#define DSIM_PHY_TLP_ENTRY_SKEW_MASK		(0x3 << 0)

/* MC/MD_TIME_CON1 */
#define DSIM_PHY_TCLK_ZERO(_x)			(((_x) & 0xFF) << 8)
#define DSIM_PHY_TCLK_ZERO_MASK			(0xFF << 8)
#define DSIM_PHY_TCLK_PREPARE(_x)		(((_x) & 0xFF) << 0)
#define DSIM_PHY_TCLK_PREPARE_MASK		(0xFF << 0)
/* MD case */
#define DSIM_PHY_THS_ZERO(_x)			(((_x) & 0xFF) << 8)
#define DSIM_PHY_THS_ZERO_MASK			(0xFF << 8)
#define DSIM_PHY_THS_PREPARE(_x)		(((_x) & 0xFF) << 0)
#define DSIM_PHY_THS_PREPARE_MASK		(0xFF << 0)

/* MC/MD_TIME_CON2 */
#define DSIM_PHY_THS_EXIT(_x)			(((_x) & 0xFF) << 8)
#define DSIM_PHY_THS_EXIT_MASK			(0xFF << 8)
/* MC case */
#define DSIM_PHY_TCLK_TRAIL(_x)			(((_x) & 0xFF) << 0)
#define DSIM_PHY_TCLK_TRAIL_MASK		(0xFF << 0)
/* MD case */
#define DSIM_PHY_THS_TRAIL(_x)			(((_x) & 0xFF) << 0)
#define DSIM_PHY_THS_TRAIL_MASK			(0xFF << 0)

/* MC_TIME_CON3 */
#define DSIM_PHY_TCLK_POST(_x)			(((_x) & 0xFF) << 0)
#define DSIM_PHY_TCLK_POST_MASK			(0xFF << 0)
/* MD_TIME_CON3 */
#define DSIM_PHY_TTA_GET(_x)			(((_x) & 0xF) << 4)
#define DSIM_PHY_TTA_GET_MASK			(0xF << 4)
#define DSIM_PHY_TTA_GO(_x)			(((_x) & 0xF) << 0)
#define DSIM_PHY_TTA_GO_MASK			(0xF << 0)

/* MC/MD_TIME_CON4 */
#define DSIM_PHY_ULPS_EXIT(_x)			(((_x) & 0x3FF) << 0)
#define DSIM_PHY_ULPS_EXIT_MASK			(0x3FF << 0)

/* MC_DATA_CON0 */
#define DSIM_PHY_CLK_INV			(1 << 1)
/* MD_DATA_CON0 */
#define DSIM_PHY_DATA_INV			(1 << 1)

/* MC_DESKEW_CON0 */
#define DSIM_PHY_SKEWCAL_RUN_TIME(_x)		(((_x) & 0xF) << 12)
#define DSIM_PHY_SKEWCAL_RUN_TIME_MASK		(0xF << 12)
#define DSIM_PHY_SKEWCAL_INIT_RUN_TIME(_x)	(((_x) & 0xF) << 8)
#define DSIM_PHY_SKEWCAL_INIT_RUN_TIME_MASK	(0xF << 8)
#define DSIM_PHY_SKEWCAL_INIT_WAIT_TIME(_x)	(((_x) & 0xF) << 4)
#define DSIM_PHY_SKEWCAL_INIT_WAIT_TIME_MASK	(0xF << 4)
#define DSIM_PHY_SKEWCAL_EN			(1 << 0)

#endif /* _REGS_DSIM_H */
