/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register definition file for Samsung DisplayPort driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _REGS_DP_H
#define _REGS_DP_H

/* DP_LINK base address : 0x110A0000 */

/* Pro DP Reference Clock is 19.2MHz. It should be set 20MHz */
#define OSC_CLK				20 // MHz Unit

/* System Common Registers */
#define SYSTEM_DEVICE_VERSION		(0x0000)

#define SYSTEM_SW_RESET_CONTROL		(0x0004)
#define SW_RESET			(1 << 0)
#define SW_RESET_GET(_v)		(((_v) >> 0) & 0x1)
#define SW_RESET_MASK			(0x1 << 0)

#define SYSTEM_CLK_CONTROL		(0x0008)
#define GFMUX_STATUS_TXCLK_GET(_v)	(((_v) >> 8) & 0x3)
#define GFMUX_STATUS_TXCLK_MASK		(0x3 << 8)
#define TXCLK_SEL_OSCCLK		(0 << 5)
#define TXCLK_SEL_TXCLK			(1 << 5)
#define TXCLK_SEL_MASK			(0x1 << 5)
#define TXCLK_SEL_MODE_BY_PLLLOCK	(0 << 4)
#define TXCLK_SEL_MODE_BY_TXCLK_SEL	(1 << 4)
#define TXCLK_SEL_MODE_MASK		(0x1 << 4)

#define SYSTEM_MAIN_LINK_LANE_COUNT	(0x0010)
#define LANE_COUNT_GET(_v)		(((_v) >> 0) & 0x7)
#define LANE_COUNT_SET(_v)		(((_v)&0x7) << 0)
#define LANE_COUNT_SET_MASK		(0x7 << 0)

#define SYSTEM_SW_FUNCTION_ENABLE	(0x0014)
#define SW_FUNC_EN_SET(_v)		(((_v)&0x1) << 0)
#define SW_FUNC_EN_MASK			(0x1 << 0)

#define SYSTEM_COMMON_FUNCTION_ENABLE	(0x0018)
#define HDCP22_FUNC_EN_SET(_v)		(((_v)&0x1) << 4)
#define HDCP22_FUNC_EN_MASK		(0x1 << 4)
#define HDCP13_FUNC_EN_SET(_v)		(((_v)&0x1) << 3)
#define HDCP13_FUNC_EN_MASK		(0x1 << 3)
#define GTC_FUNC_EN_SET(_v)		(((_v)&0x1) << 2)
#define GTC_FUNC_EN_MASK		(0x1 << 2)
#define PCS_FUNC_EN_SET(_v)		(((_v)&0x1) << 1)
#define PCS_FUNC_EN_MASK		(0x1 << 1)
#define AUX_FUNC_EN_SET(_v)		(((_v)&0x1) << 0)
#define AUX_FUNC_EN_MASK		(0x1 << 0)

#define SYSTEM_SST1_FUNCTION_ENABLE	(0x001C)
#define SST1_LH_PWR_ON_STATUS_GET(_v)	(((_v) >> 5) & 0x1)
#define SST1_LH_PWR_ON_STATUS_MASK	(0x1 << 5)
#define SST1_LH_PWR_ON_SET(_v)		(((_v)&0x1) << 4)
#define SST1_LH_PWR_ON_MASK		(0x1 << 4)
#define SST1_AUDIO_FIFO_FUNC_EN_SET(_v) (((_v)&0x1) << 2)
#define SST1_AUDIO_FIFO_FUNC_EN_MASK	(0x1 << 2)
#define SST1_AUDIO_FUNC_EN_SET(_v)	(((_v)&0x1) << 1)
#define SST1_AUDIO_FUNC_EN_MASK		(0x1 << 1)
#define SST1_VIDEO_FUNC_EN_SET(_v)	(((_v)&0x1) << 0)
#define SST1_VIDEO_FUNC_EN_MASK		(0x1 << 0)

#define SYSTEM_MISC_CONTROL		(0x0024)
#define MISC_CTRL_EN_SET(_v)		(((_v)&0x1) << 1)
#define MISC_CTRL_EN_MASK		(0x1 << 1)
#define HDCP_HPD_RST_SET(_v)		(((_v)&0x1) << 0)
#define HDCP_HPD_RST_MASK		(0x1 << 0)

#define SYSTEM_HPD_CONTROL		(0x0028)
#define HPD_HDCP_GET(_v)		(((_v) >> 30) & 0x1)
#define HPD_HDCP_MASK			(0x1 << 30)
#define HPD_DEGLITCH_COUNT_SET(_v)	(((_v)&0x3FFF) << 16)
#define HPD_DEGLITCH_COUNT_MASK		(0x3FFF << 16)
#define HPD_STATUS_GET(_v)		(((_v) >> 8) & 0x1)
#define HPD_STATUS_MASK			(0x1 << 8)
#define HPD_EVENT_UNPLUG		(1 << 7)
#define HPD_EVENT_UNPLUG_MASK		(0x1 << 7)
#define HPD_EVENT_PLUG			(1 << 6)
#define HPD_EVENT_PLUG_MASK		(0x1 << 6)
#define HPD_EVENT_IRQ			(1 << 5)
#define HPD_EVENT_IRQ_MASK		(0x1 << 5)
#define HPD_EVENT_CTRL_EN_BY_HOST	(1 << 4)
#define HPD_EVENT_CTRL_EN_BY_LOGIC	(0 << 4)
#define HPD_EVENT_CTRL_EN_MASK		(0x1 << 4)
#define HPD_FORCE_SET(_v)		(((_v)&0x1) << 1)
#define HPD_FORCE_MASK			(0x1 << 1)
#define HPD_FORCE_EN_SET(_v)		(((_v)&0x1) << 0)
#define HPD_FORCE_EN_MASK		(0x1 << 0)

#define SYSTEM_PLL_LOCK_CONTROL		(0x002C)
#define PLL_LOCK_STATUS_GET(_v)		(((_v) >> 4) & 0x1)
#define PLL_LOCK_STATUS_MASK		(0x1 << 4)
#define PLL_LOCK_FORCE_PLL_LOCK		(1 << 3)
#define PLL_LOCK_FORCE_PLL_NONLOCK	(0 << 3)
#define PLL_LOCK_FORCE_MASK		(0x1 << 3)
#define PLL_LOCK_FORCE_BY_SFR		(1 << 2)
#define PLL_LOCK_FORCE_BY_PLL		(0 << 2)
#define PLL_LOCK_FORCE_EN_MASK		(0x1 << 2)

/* OSC Clock Control Registers */
#define OSC_CLK_DIV_HPD			(0x0050)
#define HPD_EVENT_CLK_COUNT_SET(_v)                                            \
	((((_v)*2000) & 0x7FFFF) << 0) // (OSC(Hz) * 2) / 10^3
#define HPD_EVENT_CLK_COUNT_MASK (0x7FFFF << 0)

#define OSC_CLK_DIV_HDCP_10US	 (0x0054)
#define I2C_GEN10US_TIMER_SET(_v)                                              \
	((((_v)*10 - 1) & 0xFFF) << 0) // (OSC(Hz) / 10^5) - 1
#define I2C_GEN10US_TIMER_MASK (0xFFF << 0)

#define OSC_CLK_DIV_GTC_1MS    (0x0058)
#define GTC_1MS_OSC_CLK_COUNT_SET(_v)                                          \
	((((_v)*1000 - 1) & 0x3FFFF) << 0) // (OSC(Hz) / 10^3) - 1
#define GTC_1MS_OSC_CLK_COUNT_MASK (0x3FFFF << 0)

#define OSC_CLK_DIV_AUX_1US	   (0x005C)
#define AUX_1US_OSC_CLK_COUNT_SET(_v)                                          \
	((((_v)-1) & 0xFF) << 0) // ((OSC(Hz) / 10^6) - 1
#define AUX_1US_OSC_CLK_COUNT_MASK (0xFF << 0)

#define OSC_CLK_DIV_AUX_MAN_UI	   (0x0060)
#define AUX_MAN_UI_OSC_CLK_COUNT_SET(_v)                                       \
	((((((_v)*5) / 10) - 1) & 0xFF) << 0) // ((OSC(Hz) * 5 / 10^7) - 1
#define AUX_MAN_UI_OSC_CLK_COUNT_MASK (0xFF << 0)

#define OSC_CLK_DIV_AUX_10US	      (0x0064)
#define AUX_10US_OSC_CLK_COUNT_SET(_v)                                         \
	((((_v)*10) & 0xFFF) << 0) // OSC(Hz) / 10^5
#define AUX_10US_OSC_CLK_COUNT_MASK	(0xFFF << 0)

/* System Interrupt Registers */
#define SYSTEM_INTERRUPT_CONTROL	(0x0100)
#define SW_INTR_CTRL			(0x1 << 1)
#define INTR_POL			(0x1 << 0)

#define SYSTEM_INTERRUPT_REQUEST	(0x0104)
#define IRQ_PCS_GET(_v)			(((_v) >> 6) & 0x1)
#define IRQ_STR1_GET(_v)		(((_v) >> 1) & 0x1)
#define IRQ_COMMON_GET(_v)		(((_v) >> 0) & 0x1)

#define SYSTEM_IRQ_COMMON_STATUS	(0x0108)
#define HDCP_ENC_EN_CHG			(0x1 << 22)
#define HDCP_LINK_CHK_FAIL		(0x1 << 21)
#define HDCP_R0_CHECK_FLAG		(0x1 << 20)
#define HDCP_BKSV_RDY			(0x1 << 19)
#define HDCP_SHA_DONE			(0x1 << 18)
#define HDCP_AUTH_STATE_CHG		(0x1 << 17)
#define HDCP_AUTH_DONE			(0x1 << 16)
#define HPD_IRQ				(0x1 << 11)
#define HPD_CHG				(0x1 << 10)
#define HPD_LOST			(0x1 << 9)
#define HPD_PLUG			(0x1 << 8)
#define AUX_REPLY_RECEIVED		(0x1 << 5)
#define AUX_ERR				(0x1 << 4)
#define PLL_LOCK_CHG			(0x1 << 1)
#define SW_INTR				(0x1 << 0)

#define SYSTEM_IRQ_COMMON_STATUS_MASK	(0x010C)
#define HDCP_ENC_EN_CHG_MASK		(0x1 << 22)
#define HDCP_LINK_CHK_FAIL_MASK		(0x1 << 21)
#define HDCP_R0_CHECK_FLAG_MASK		(0x1 << 20)
#define HDCP_BKSV_RDY_MASK		(0x1 << 19)
#define HDCP_SHA_DONE_MASK		(0x1 << 18)
#define HDCP_AUTH_STATE_CHG_MASK	(0x1 << 17)
#define HDCP_AUTH_DONE_MASK		(0x1 << 16)
#define HPD_IRQ_MASK			(0x1 << 11)
#define HPD_CHG_MASK			(0x1 << 10)
#define HPD_LOST_MASK			(0x1 << 9)
#define HPD_PLUG_MASK			(0x1 << 8)
#define AUX_REPLY_RECEIVED_MASK		(0x1 << 5)
#define AUX_ERR_MASK			(0x1 << 4)
#define PLL_LOCK_CHG_MASK		(0x1 << 1)
#define SW_INTR_MASK			(0x1 << 0)

/* System Debug Registers */
#define SYSTEM_DEBUG			(0x0200)
#define AUX_HPD_CONTROL			(0x1 << 2)
#define CLKGATE_DISABLE			(0x1 << 1)

#define SYSTEM_DEBUG_LH_PCH		(0x0204)
#define SST1_LH_PSTATE			(0x1 << 4)
#define SST1_LH_PCH_FSM_STATE		(0xF << 0)

#define SYSTEM_DEBUG_LH_RST_CTRL	(0x0208)
#define SST1_LH_RST_CTRL		(0x1 << 0)

#define SYSTEM_DEBUG_AUDIO_RST_CTRL	(0x020C)
#define SST1_AUDIO_RST_CTRL		(0x1 << 0)

#define SYSTEM_DEBUG_HDCP_RST_CTRL	(0x0210)
#define HDCP_22_RST_CTRL		(0x1 << 0)

/* AUX Control Registers */
#define AUX_CONTROL			(0x1000)
#define AUX_POWER_DOWN			(1 << 16)
#define AUX_POWER_DOWN_MASK		(0x1 << 16)
#define AUX_REPLY_TIMER_MODE_600US	(0 << 12)
#define AUX_REPLY_TIMER_MODE_800US	(1 << 12)
#define AUX_REPLY_TIMER_MODE_1000US	(2 << 12)
#define AUX_REPLY_TIMER_MODE_1800US	(3 << 12)
#define AUX_REPLY_TIMER_MODE_MASK	(0x3 << 12)
#define AUX_RETRY_TIMER_SET(_v)		(((_v)&0x7) << 8)
#define AUX_RETRY_TIMER_MASK		(0x7 << 8)
#define AUX_PN_NORMAL			(0 << 1)
#define AUX_PN_INVERT			(1 << 1)
#define AUX_PN_INV_MASK			(0x1 << 1)
#define REG_MODE_AUTO			(0 << 0)
#define REG_MODE_MANCHESTER		(1 << 0)
#define REG_MODE_SEL_MASK		(0x1 << 0)

#define AUX_TRANSACTION_START		(0x1004)
#define AUX_TRAN_START			(1 << 0)
#define AUX_TRAN_START_GET(_v)		(((_v) >> 0) & 0x1)
#define AUX_TRAN_START_MASK		(0x1 << 0)

#define AUX_BUFFER_CLEAR		(0x1008)
#define AUX_BUF_CLR			(1 << 0)
#define AUX_BUF_CLR_MASK		(0x1 << 0)

#define AUX_ADDR_ONLY_COMMAND		(0x100C)
#define ADDR_ONLY_CMD			(1 << 0)
#define NORMAL_AUX_CMD			(0 << 0)
#define ADDR_ONLY_CMD_MASK		(0x1 << 0)

#define AUX_REQUEST_CONTROL		(0x1010)
#define REQ_COMM_SET(_v)		(((_v)&0xF) << 28)
#define REQ_COMM_MASK			(0xF << 28)
#define REQ_ADDR_SET(_v)		(((_v)&0xFFFFF) << 8)
#define REQ_ADDR_MASK			(0xFFFFF << 8)
#define REQ_LENGTH_SET(_v)		(((_v)&0x3F) << 0)
#define REQ_LENGTH_MASK			(0x3F << 0)

#define AUX_COMMAND_CONTROL		(0x1014)
#define DEFER_CTRL_EN_SET(_v)		(((_v)&0x1) << 8)
#define DEFER_CTRL_EN_MASK		(0x1 << 8)
#define DEFER_COUNT_SET(_v)		(((_v)&0x7F) << 0)
#define DEFER_COUNT_MASK		(0x7F << 0)

#define AUX_MONITOR_1			(0x1018)
#define AUX_BUF_DATA_COUNT_GET(_v)	(((_v) >> 24) & 0x7F)
#define AUX_BUF_DATA_COUNT_MASK		(0x7F << 24)
#define AUX_DETECTED_PERIOD_MON_GET(_v) (((_v) >> 12) & 0x1FF)
#define AUX_DETECTED_PERIOD_MON_MASK	(0x1FF << 12)
#define AUX_CMD_STATUS_GET(_v)		(((_v) >> 8) & 0xF)
#define AUX_CMD_STATUS_MASK		(0xF << 8)
#define AUX_RX_COMM_GET(_v)		(((_v) >> 4) & 0xF)
#define AUX_RX_COMM_MASK		(0xF << 4)
#define AUX_LAST_MODE_GET(_v)		(((_v) >> 3) & 0x1)
#define AUX_LAST_MODE_MASK		(0x1 << 3)
#define AUX_BUSY_GET(_v)		(((_v) >> 2) & 0x1)
#define AUX_BUSY_MASK			(0x1 << 2)
#define AUX_REQ_WAIT_GRANT_MASK		(0x1 << 1)
#define AUX_REQ_SIGNAL_MASK		(0x1 << 0)

#define AUX_MONITOR_2			(0x101C)
#define AUX_ERROR_NUMBER_GET(_v)	(((_v) >> 0) & 0xFF)
#define AUX_ERROR_NUMBER_MASK		(0xFF << 0)

#define AUX_TX_DATA_SET(_v)		(0x1030 + (((_v) / 4) * 4))
#define TX_DATA_SET(_d, _v)		((_d) << (((_v) % 4) * 8))
#define TX_DATA_MASK(_v)		(0xFF << (((_v) % 4) * 8))

#define AUX_TX_DATA_SET0		(0x1030)
#define AUX_TX_DATA_SET1		(0x1034)
#define AUX_TX_DATA_SET2		(0x1038)
#define AUX_TX_DATA_SET3		(0x103C)

#define AUX_RX_DATA_SET(_v)		(0x1040 + (((_v) / 4) * 4))
#define RX_DATA_GET(_d, _v)		(((_d) >> (((_v) % 4) * 8)) & 0xFF)
#define RX_DATA_MASK(_v)		(0xFF << (((_v) % 4) * 8))

#define AUX_RX_DATA_SET0		(0x1040)
#define AUX_RX_DATA_SET1		(0x1044)
#define AUX_RX_DATA_SET2		(0x1048)
#define AUX_RX_DATA_SET3		(0x104C)

#define AUX_RETRY_COUNTER_CLEAR		(0x1050)
#define RETRY_COUNTER_CLEAR		(0x1 << 0)

/* GTC (Global Time Code) Control Registers */
#define GTC_CONTROL			(0x1100)
#define GTC_DELTA_ADJ_EN		(0x1 << 2)
#define IMPL_OPTION			(0x1 << 1)
#define GTC_TX_MASTER			(0x1 << 0)

#define GTC_WORK_ENABLE			(0x1104)
#define GTC_WORK_EN			(0x1 << 0)

#define GTC_TIME_CONTROL		(0x1108)
#define TIME_PERIOD_SEL			(0x3 << 28)
#define TIME_PERIOD_FRACTIONAL		(0xFFFFF << 8)
#define TIME_PERIOD_INT1		(0xF << 4)
#define TIME_PERIOD_INT2		(0xF << 0)

#define GTC_ATTEMPT_CONTROL		(0x110C)
#define GTC_STATE_CHANGE_CTRL		(0x1 << 8)
#define NUM_GTC_ATTEMPT			(0xFF << 0)

#define GTC_TX_VALUE_MONITOR		(0x1110)
#define GTC_TX_VAL			(0xFFFFFFFF << 0)

#define GTC_RX_VALUE_MONITOR		(0x1114)
#define GTC_RX_VAL			(0xFFFFFFFF << 0)

#define GTC_STATUS_MONITOR		(0x1118)
#define FREQ_ADJ_10_3			(0xFF << 8)
#define FREQ_ADJ_2_0			(0x7 << 5)
#define TXGTC_LOCK_DONE_FLAG		(0x1 << 1)
#define RXGTC_LOCK_DONE_FLAG		(0x1 << 0)

#define AUX_GTC_DEBUG			(0x1200)
#define DEBUG_STATE_SEL			(0xFF << 8)
#define DEBUG_STATE			(0xFF << 0)

/* PCS (Scrambler/Encoder/FEC) Control Registers */
#define PCS_CONTROL			(0x3000)
#define FEC_READY			(0x1 << 9)
#define FEC_FUNC_EN			(0x1 << 8)
#define LINK_TRAINING_PATTERN_SET(_v)	(((_v)&0x7) << 4)
#define LINK_TRAINING_PATTERN_MASK	(0x7 << 4)
#define BYTE_SWAP			(0x1 << 3)
#define BIT_SWAP			(0x1 << 2)
#define SCRAMBLE_RESET_VALUE		(0x1 << 1)
#define SCRAMBLE_BYPASS_SET(_v)		(((_v)&0x1) << 0)
#define SCRAMBLE_BYPASS_MASK		(0x1 << 0)

enum dp_logical_lane {
	LOGICAL_LANE_0 = 0,
	LOGICAL_LANE_1,
	LOGICAL_LANE_2,
	LOGICAL_LANE_3,
};

#define PCS_LANE_CONTROL			   (0x3004)
#define LANE_DATA_INV_EN			   (0x1 << 20)
#define LANE3_DATA_INV				   (0x1 << 19)
#define LANE2_DATA_INV				   (0x1 << 18)
#define LANE1_DATA_INV				   (0x1 << 17)
#define LANE0_DATA_INV				   (0x1 << 16)
#define LANE3_MAP_SET(_v)			   (((_v)&0x3) << 12)
#define LANE3_MAP_MASK				   (0x3 << 12)
#define LANE2_MAP_SET(_v)			   (((_v)&0x3) << 8)
#define LANE2_MAP_MASK				   (0x3 << 8)
#define LANE1_MAP_SET(_v)			   (((_v)&0x3) << 4)
#define LANE1_MAP_MASK				   (0x3 << 4)
#define LANE0_MAP_SET(_v)			   (((_v)&0x3) << 0)
#define LANE0_MAP_MASK				   (0x3 << 0)

#define PCS_TEST_PATTERN_CONTROL		   (0x3008)
#define TEST_PATTERN				   (0x3F << 8)
#define LINK_QUALITY_PATTERN_SET(_v)		   (((_v)&0x7) << 0)
#define LINK_QUALITY_PATTERN_MASK		   (0x7 << 0)

#define PCS_TEST_PATTERN_SET0			   (0x300C)
#define TEST_80BIT_PATTERN_SET0			   (0xFFFFFFFF << 0)

#define PCS_TEST_PATTERN_SET1			   (0x3010)
#define TEST_80BIT_PATTERN_SET1			   (0xFFFFFFFF << 0)

#define PCS_TEST_PATTERN_SET2			   (0x3014)
#define TEST_80BIT_PATTERN_SET2			   (0xFFFF << 0)

#define PCS_DEBUG_CONTROL			   (0x3018)
#define FEC_FLIP_CDADJ_CODES_CASE4		   (0x1 << 6)
#define FEC_FLIP_CDADJ_CODES_CASE2		   (0x1 << 5)
#define FEC_DECODE_EN_4TH_SEL			   (0x1 << 4)
#define FEC_DECODE_DIS_4TH_SEL			   (0x1 << 3)
#define PRBS7_OPTION				   (0x1 << 2)
#define DISABLE_AUTO_RESET_ENCODE		   (0x1 << 1)
#define PRBS31_EN				   (0x1 << 0)

#define PCS_HBR2_EYE_SR_CONTROL			   (0x3020)
#define HBR2_EYE_SR_CTRL			   (0x3 << 16)
#define HBR2_EYE_SR_COUNT			   (0xFFFF << 0)

#define PCS_HBR3_CONTROL			   (0x3024)
#define HBR3_TPS4_PATTERN_CNT			   (0xFFFF << 0)

#define PCS_SA_CRC_CONTROL_1			   (0x3100)
#define SA_CRC_CLEAR				   (0x1 << 13)
#define SA_CRC_SW_COMPARE			   (0x1 << 12)
#define SA_CRC_LN3_PASS				   (0x1 << 11)
#define SA_CRC_LN2_PASS				   (0x1 << 10)
#define SA_CRC_LN1_PASS				   (0x1 << 9)
#define SA_CRC_LN0_PASS				   (0x1 << 8)
#define SA_CRC_LN3_FAIL				   (0x1 << 7)
#define SA_CRC_LN2_FAIL				   (0x1 << 6)
#define SA_CRC_LN1_FAIL				   (0x1 << 5)
#define SA_CRC_LN0_FAIL				   (0x1 << 4)
#define SA_CRC_LANE_3_ENABLE			   (0x1 << 3)
#define SA_CRC_LANE_2_ENABLE			   (0x1 << 2)
#define SA_CRC_LANE_1_ENABLE			   (0x1 << 1)
#define SA_CRC_LANE_0_ENABLE			   (0x1 << 0)

#define PCS_SA_CRC_CONTROL_2			   (0x3104)
#define SA_CRC_LN0_REF				   (0xFFFF << 16)
#define SA_CRC_LN0_RESULT			   (0xFFFF << 0)

#define PCS_SA_CRC_CONTROL_3			   (0x3108)
#define SA_CRC_LN1_REF				   (0xFFFF << 16)
#define SA_CRC_LN1_RESULT			   (0xFFFF << 0)

#define PCS_SA_CRC_CONTROL_4			   (0x310C)
#define SA_CRC_LN2_REF				   (0xFFFF << 16)
#define SA_CRC_LN2_RESULT			   (0xFFFF << 0)

#define PCS_SA_CRC_CONTROL_5			   (0x3110)
#define SA_CRC_LN3_REF				   (0xFFFF << 16)
#define SA_CRC_LN3_RESULT			   (0xFFFF << 0)

#define PCS_SCRAM_ENCODE_SELF_CHK		   (0x3114)
#define ENCODE_FAIL_FLAG_CLEAR			   (0x1 << 20)
#define ENCODE_TEST_ENABLE_3			   (0x1 << 19)
#define ENCODE_TEST_ENABLE_2			   (0x1 << 18)
#define ENCODE_TEST_ENABLE_1			   (0x1 << 17)
#define ENCODE_TEST_ENABLE_0			   (0x1 << 16)
#define SCRAM_FAIL_FLAG_CLEAR			   (0x1 << 4)
#define SCRAM_TEST_ENABLE_3			   (0x1 << 3)
#define SCRAM_TEST_ENABLE_2			   (0x1 << 2)
#define SCRAM_TEST_ENABLE_1			   (0x1 << 1)
#define SCRAM_TEST_ENABLE_0			   (0x1 << 0)

#define PCS_SCRAM_ENCODE_FAIL_INTP		   (0x3118)
#define ENCODE_TEST_FAIL_3_MASK			   (0x1 << 23)
#define ENCODE_TEST_FAIL_2_MASK			   (0x1 << 22)
#define ENCODE_TEST_FAIL_1_MASK			   (0x1 << 21)
#define ENCODE_TEST_FAIL_0_MASK			   (0x1 << 20)
#define ENCODE_TEST_FAIL_3			   (0x1 << 19)
#define ENCODE_TEST_FAIL_2			   (0x1 << 18)
#define ENCODE_TEST_FAIL_1			   (0x1 << 17)
#define ENCODE_TEST_FAIL_0			   (0x1 << 16)
#define SCRAM_TEST_FAIL_3_MASK			   (0x1 << 7)
#define SCRAM_TEST_FAIL_2_MASK			   (0x1 << 6)
#define SCRAM_TEST_FAIL_1_MASK			   (0x1 << 5)
#define SCRAM_TEST_FAIL_0_MASK			   (0x1 << 4)
#define SCRAM_TEST_FAIL_3			   (0x1 << 3)
#define SCRAM_TEST_FAIL_2			   (0x1 << 2)
#define SCRAM_TEST_FAIL_1			   (0x1 << 1)
#define SCRAM_TEST_FAIL_0			   (0x1 << 0)

/* HDCP Control Registers */
// HDCP 1.3
#define HDCP13_STATUS				   (0x4000)
#define REAUTH_REQUEST				   (0x1 << 7)
#define AUTH_FAIL				   (0x1 << 6)
#define HW_1ST_AUTHEN_PASS			   (0x1 << 5)
#define BKSV_VALID				   (0x1 << 3)
#define ENCRYPT					   (0x1 << 2)
#define HW_AUTHEN_PASS				   (0x1 << 1)
#define AKSV_VALID				   (0x1 << 0)

#define HDCP13_CONTROL_0			   (0x4004)
#define SW_STORE_AN				   (0x1 << 7)
#define SW_RX_REPEATER				   (0x1 << 6)
#define HW_RE_AUTHEN				   (0x1 << 5)
#define SW_AUTH_OK				   (0x1 << 4)
#define HW_AUTH_EN				   (0x1 << 3)
#define HDCP13_ENC_EN				   (0x1 << 2)
#define HW_1ST_PART_ATHENTICATION_EN		   (0x1 << 1)
#define HW_2ND_PART_ATHENTICATION_EN		   (0x1 << 0)

#define HDCP13_CONTROL_1			   (0x4008)
#define CONTROL_RESERVED			   (0xF << 4)
#define DPCD_REV_1_2				   (0x1 << 3)
#define HW_AUTH_POLLING_MODE			   (0x1 << 1)
#define HDCP_INT				   (0x1 << 0)

#define HDCP13_AKSV_0				   (0x4010)
#define AKSV0					   (0xFFFFFFFF << 0)

#define HDCP13_AKSV_1				   (0x4014)
#define AKSV1					   (0xFF << 0)

#define HDCP13_AN_0				   (0x4018)
#define AN0					   (0xFFFFFFFF << 0)

#define HDCP13_AN_1				   (0x401C)
#define AN1					   (0xFFFFFFFF << 0)

#define HDCP13_BKSV_0				   (0x4020)
#define BKSV0					   (0xFFFFFFFF << 0)

#define HDCP13_BKSV_1				   (0x4024)
#define BKSV1					   (0xFF << 0)

#define HDCP13_R0				   (0x4028)
#define R0					   (0xFFFF << 0)

#define HDCP13_BCAPS				   (0x4030)
#define BCAPS					   (0xFF << 0)

#define HDCP13_BINFO				   (0x4034)
#define BINFO					   (0xFFFF << 0)

#define HDCP13_DEBUG_CONTROL			   (0x4038)
#define CHECK_KSV				   (0x1 << 2)
#define REVOCATION_CHK_DONE			   (0x1 << 1)
#define HW_SKIP_RPT_ZERO_DEV			   (0x1 << 0)

#define HDCP13_KEY_MODE				   (0x403C)
#define KEY_MODE				   (0x1 << 6)

#define HDCP13_AUTH_DBG				   (0x4040)
#define DDC_STATE				   (0x7 << 5)
#define AUTH_STATE				   (0x1F << 0)

#define HDCP13_ENC_DBG				   (0x4044)
#define ENC_STATE				   (0x7 << 3)

#define HDCP13_AM0_0				   (0x4048)
#define AM0_0					   (0xFFFFFFFF << 0)

#define HDCP13_AM0_1				   (0x404C)
#define AM0_1					   (0xFFFFFFFF << 0)

#define HDCP13_KEY_VALID_STATUS			   (0x4050)
#define KEY_VALID_SYNC_IN_I2C_CLK		   (0x1 << 0)

#define HDCP13_WAIT_R0_TIME			   (0x4054)
#define HW_WRITE_AKSV_WAIT			   (0xFF << 0)

#define HDCP13_LINK_CHK_TIME			   (0x4058)
#define LINK_CHK_TIMER				   (0xFF << 0)

#define HDCP13_REPEATER_READY_WAIT_TIME		   (0x405C)
#define HW_RPTR_RDY_TIMER			   (0xFF << 0)

#define HDCP13_READY_POLL_TIME			   (0x4060)
#define POLLING_TIMER_TH			   (0xFF << 0)

#define HDCP13_HIDDEN				   (0x4064)
#define DERIVE_NEXT_AN				   (0x1 << 2)
#define FEC_PM_HOLD_LINK_LINE_BOUNDARY_COUNTER	   (0x1 << 1)
#define FEC_PM_HOLD_CIPHER			   (0x1 << 0)

#define HDCP13_STREAM_ID_ENCRYPTION_CONTROL	   (0x4068)
#define STRM_ID_ENC_UPDATE			   (0x1 << 7)
#define STRM_ID_ENC				   (0x7F << 0)

// HDCP 2.2
#define HDCP22_SYS_EN				   (0x4400)
#define SYSTEM_ENABLE				   (0x1 << 0)

#define HDCP22_CONTROL				   (0x4404)
#define HDCP22_BYPASS_MODE			   (0x1 << 1)
#define HDCP22_ENC_EN				   (0x1 << 0)

#define HDCP22_STREAM_TYPE			   (0x4454)
#define STREAM_TYPE				   (0x1 << 0)

#define HDCP22_LVP				   (0x4460)
#define LINK_VERIFICATION_PATTERN		   (0xFFFF << 0)

#define HDCP22_LVP_GEN				   (0x4464)
#define LVP_GEN					   (0x1 << 0)

#define HDCP22_LVP_CNT_KEEP			   (0x4468)
#define LVP_COUNT_KEEP_ENABLE			   (0x1 << 0)

#define HDCP22_LANE_DECODE_CTRL			   (0x4470)
#define ENHANCED_FRAMING_MODE			   (0x1 << 3)
#define LVP_EN_DECODE_ENABLE			   (0x1 << 2)
#define ENCRYPTION_SIGNAL_DECODE_ENABLE		   (0x1 << 1)
#define LANE_DECODE_ENABLE			   (0x1 << 0)

#define HDCP22_SR_VALUE				   (0x4480)
#define SR_VALUE				   (0xFF << 0)

#define HDCP22_CP_VALUE				   (0x4484)
#define CP_VALUE				   (0xFF << 0)

#define HDCP22_BF_VALUE				   (0x4488)
#define BF_VALUE				   (0xFF << 0)

#define HDCP22_BS_VALUE				   (0x448C)
#define BS_VALUE				   (0xFF << 0)

#define HDCP22_RIV_XOR				   (0x4490)
#define RIV_XOR_LOCATION			   (0x1 << 0)

#define HDCP22_RIV_0				   (0x4500)
#define RIV_KEY_0				   (0xFFFFFFFF << 0)

#define HDCP22_RIV_1				   (0x4504)
#define RIV_KEY_1				   (0xFFFFFFFF << 0)

#define HDCP22_KS_KEY_0_SECURE			   (0x4800)
#define KS_KEY_0				   (0xFFFFFFFF << 0)

#define HDCP22_KS_KEY_1_SECURE			   (0x4804)
#define KS_KEY_1				   (0xFFFFFFFF << 0)

#define HDCP22_KS_KEY_2_SECURE			   (0x4808)
#define KS_KEY_2				   (0xFFFFFFFF << 0)

#define HDCP22_KS_KEY_3_SECURE			   (0x480C)
#define KS_KEY_3				   (0xFFFFFFFF << 0)

#define HDCP22_AES_KEY_0_SECURE			   (0x4810)
#define AES_PAIRING_KEY_0			   (0xFFFFFFFF << 0)

#define HDCP22_AES_KEY_1_SECURE			   (0x4814)
#define AES_PAIRING_KEY_1			   (0xFFFFFFFF << 0)

#define HDCP22_AES_KEY_2_SECURE			   (0x4818)
#define AES_PAIRING_KEY_2			   (0xFFFFFFFF << 0)

#define HDCP22_AES_KEY_3_SECURE			   (0x481C)
#define AES_PAIRING_KEY_3			   (0xFFFFFFFF << 0)

/* SST (Single-Stream Transport) Control Registers */
// SST1 Common
#define SST1_MAIN_CONTROL			   (0x5000)
#define AUDIO_MODE_SET(_v)			   (((_v)&0x1) << 10)
#define AUDIO_MODE_MASK				   (0x1 << 10)
#define VIDEO_MODE_SET(_v)			   (((_v)&0x1) << 6)
#define VIDEO_MODE_MASK				   (0x1 << 6)
#define ENHANCED_MODE_SET(_v)			   (((_v)&0x1) << 5)
#define ENHANCED_MODE_MASK			   (0x1 << 5)
#define ODD_TU_CONTROL				   (0x1 << 4)

#define SST1_MAIN_FIFO_CONTROL			   (0x5004)
#define CLEAR_IFCRC				   (0x1 << 4)
#define CLEAR_AUDIO_1_FIFO_SET(_v)		   (((_v)&0x1) << 3)
#define CLEAR_AUDIO_1_FIFO_MASK			   (0x1 << 3)
#define CLEAR_PIXEL_MAPPING_FIFO		   (0x1 << 2)
#define CLEAR_MAPI_FIFO				   (0x1 << 1)
#define CLEAR_GL_DATA_FIFO			   (0x1 << 0)

#define SST1_GNS_CONTROL			   (0x5008)
#define RS_CTRL					   (0x1 << 0)

#define SST1_SR_CONTROL				   (0x500C)
#define SR_COUNT_RESET_VALUE			   (0x1FF << 16)
#define SR_REPLACE_BS_COUNT			   (0x1F << 4)
#define SR_START_CTRL				   (0x1 << 1)
#define SR_REPLACE_BS_EN			   (0x1 << 0)

// SST1 Interrupts
#define SST1_INTERRUPT_MONITOR			   (0x5020)
#define INT_STATE				   (0x1 << 0)

#define SST1_INTERRUPT_STATUS_SET0		   (0x5024)
#define VSC_SDP_TX_INCOMPLETE			   (0x1 << 9)
#define MAPI_FIFO_UNDER_FLOW			   (0x1 << 8)
#define VSYNC_DET				   (0x1 << 7)

#define SST1_INTERRUPT_STATUS_SET1		   (0x5028)
#define AFIFO_UNDER				   (0x1 << 7)
#define AFIFO_OVER				   (0x1 << 6)

#define SST1_INTERRUPT_MASK_SET0		   (0x502C)
#define VSC_SDP_TX_INCOMPLETE_MASK		   (0x1 << 9)
#define MAPI_FIFO_UNDER_FLOW_MASK		   (0x1 << 8)
#define VSYNC_DET_MASK				   (0x1 << 7)

#define SST1_INTERRUPT_MASK_SET1		   (0x5030)
#define AFIFO_UNDER_MASK			   (0x1 << 7)
#define AFIFO_OVER_MASK				   (0x1 << 6)

// SST1 Video M/N Value
#define SST1_MVID_MASTER_MODE			   (0x5044)
#define SST1_NVID_MASTER_MODE			   (0x5048)
#define SST1_MVID_SFR_CONFIGURE			   (0x504C)
#define SST1_NVID_SFR_CONFIGURE			   (0x5050)
#define MNVID_SFR_CONFIG_MASK			   (0xFFFFFF << 0)

// SST1 Audio M/N Value
#define SST1_MAUD_MASTER_MODE			   (0x505C)
#define SST1_NAUD_MASTER_MODE			   (0x5060)
#define SST1_MAUD_SFR_CONFIGURE			   (0x5064)
#define SST1_NAUD_SFR_CONFIGURE			   (0x5068)
#define MNAUD_SFR_CONFIG_MASK			   (0xFFFFFF << 0)

#define SST1_NARROW_BLANK_CONTROL		   (0x506C)
#define NARROW_BLANK_EN				   (0x1 << 1)
#define VIDEO_FIFO_FLUSH_DISABLE		   (0x1 << 0)

// SST1 Active Symbol SW Control
#define SST1_ACTIVE_SYMBOL_INTEGER_FEC_OFF	   (0x5080)
#define SST1_ACTIVE_SYMBOL_FRACTION_FEC_OFF	   (0x5084)
#define SST1_ACTIVE_SYMBOL_THRESHOLD_FEC_OFF	   (0x5088)
#define SST1_ACTIVE_SYMBOL_THRESHOLD_SEL_FEC_OFF   (0x508C)
#define SST1_ACTIVE_SYMBOL_INTEGER_FEC_ON	   (0x5090)
#define SST1_ACTIVE_SYMBOL_FRACTION_FEC_ON	   (0x5094)
#define SST1_ACTIVE_SYMBOL_THRESHOLD_FEC_ON	   (0x5098)
#define SST1_ACTIVE_SYMBOL_THRESHOLD_SEL_FEC_ON	   (0x509C)
#define ACTIVE_SYMBOL_INTEGER_SET(_v)		   (((_v)&0x3F) << 0)
#define ACTIVE_SYMBOL_INTEGER_MASK		   (0x3F << 0)
#define ACTIVE_SYMBOL_FRACTION_SET(_v)		   (((_v)&0x3FFFFFFF) << 0)
#define ACTIVE_SYMBOL_FRACTION_MASK		   (0x3FFFFFFF << 0)
#define ACTIVE_SYMBOL_THRESHOLD_SET(_v)		   (((_v)&0xF) << 0)
#define ACTIVE_SYMBOL_THRESHOLD_MASK		   (0xF << 0)
#define ACTIVE_SYMBOL_THRESHOLD_SEL_SET(_v)	   (((_v)&0x1) << 0)
#define ACTIVE_SYMBOL_THRESHOLD_SEL_MASK	   (0x1 << 0)

#define SST1_FEC_DISABLE_SEND_CONTROL		   (0x5100)
#define FEC_DISABLE_SEND_CONTROL		   (0x1 << 0)

#define SST1_HDR_PPS_SDP_SEND_CONTROL		   (0x5104)
#define HDR_PPS_SDP_SEND_CONTRL			   (0x1 << 0)

// SST1 Video Control
#define SST1_VIDEO_CONTROL			   (0x5400)
#define STRM_VALID_MON				   (0x1 << 10)
#define STRM_VALID_FORCE_SET(_v)		   (((_v)&0x1) << 9)
#define STRM_VALID_FORCE_MASK			   (0x1 << 9)
#define STRM_VALID_CTRL_SET(_v)			   (((_v)&0x1) << 8)
#define STRM_VALID_CTRL_MASK			   (0x1 << 8)
#define DYNAMIC_RANGE_MODE_SET(_v)		   (((_v)&0x1) << 7)
#define DYNAMIC_RANGE_MODE_MASK			   (0x1 << 7)
#define BPC_SET(_v)				   (((_v)&0x7) << 4)
#define BPC_MASK				   (0x7 << 4)
#define COLOR_FORMAT_SET(_v)			   (((_v)&0x3) << 2)
#define COLOR_FORMAT_MASK			   (0x3 << 2)
#define VSYNC_POLARITY_SET(_v)			   (((_v)&0x1) << 1)
#define VSYNC_POLARITY_MASK			   (0x1 << 1)
#define HSYNC_POLARITY_SET(_v)			   (((_v)&0x1) << 0)
#define HSYNC_POLARITY_MASK			   (0x1 << 0)

#define SST1_VIDEO_ENABLE			   (0x5404)
#define VIDEO_EN_SET(_v)			   (((_v)&0x1) << 0)
#define VIDEO_EN_MASK				   (0x1 << 0)

#define SST1_VIDEO_MASTER_TIMING_GEN		   (0x5408)
#define VIDEO_MASTER_TIME_GEN_SET(_v)		   (((_v)&0x1) << 0)
#define VIDEO_MASTER_TIME_GEN_MASK		   (0x1 << 0)

#define SST1_VIDEO_MUTE				   (0x540C)
#define VIDEO_MUTE				   (0x1 << 0)

// SST1 Video Timing
#define SST1_VIDEO_HORIZONTAL_TOTAL_PIXELS	   (0x5414)
#define SST1_VIDEO_VERTICAL_TOTAL_PIXELS	   (0x5418)
#define SST1_VIDEO_HORIZONTAL_FRONT_PORCH	   (0x541C)
#define SST1_VIDEO_HORIZONTAL_BACK_PORCH	   (0x5420)
#define SST1_VIDEO_HORIZONTAL_ACTIVE		   (0x5424)
#define SST1_VIDEO_VERTICAL_FRONT_PORCH		   (0x5428)
#define SST1_VIDEO_VERTICAL_BACK_PORCH		   (0x542C)
#define SST1_VIDEO_VERTICAL_ACTIVE		   (0x5430)

#define SST1_VIDEO_DSC_STREAM_CONTROL_0		   (0x5434)
#define DSC_ENABLE				   (0x1 << 4)
#define SLICE_COUNT_PER_LINE			   (0x7 << 0)

#define SST1_VIDEO_DSC_STREAM_CONTROL_1		   (0x5438)
#define SST1_VIDEO_DSC_STREAM_CONTROL_2		   (0x543C)
#define CHUNK_SIZE_ODD				   (0xFFFF << 16)
#define CHUNK_SIZE_EVEN				   (0xFFFF << 0)

// SST1 Video BIST Control
#define SST1_VIDEO_BIST_CONTROL			   (0x5450)
#define CTS_BIST_EN_SET(_v)			   (((_v)&0x1) << 17)
#define CTS_BIST_EN_MASK			   (0x1 << 17)
#define CTS_BIST_TYPE_SET(_v)			   (((_v)&0x3) << 15)
#define CTS_BIST_TYPE_MASK			   (0x3 << 15)
#define BIST_PRBS7_SEED				   (0x7F << 8)
#define BIST_USER_DATA_EN			   (0x1 << 4)
#define BIST_EN_SET(_v)				   (((_v)&0x1) << 3)
#define BIST_EN_MASK				   (0x1 << 3)
#define BIST_WIDTH_SET(_v)			   (((_v)&0x1) << 2)
#define BIST_WIDTH_MASK				   (0x1 << 2)
#define BIST_TYPE_SET(_v)			   (((_v)&0x3) << 0)
#define BIST_TYPE_MASK				   (0x3 << 0)

#define SST1_VIDEO_BIST_USER_DATA_R		   (0x5454)
#define BIST_USER_DATA_R			   (0x3FF << 0)

#define SST1_VIDEO_BIST_USER_DATA_G		   (0x5458)
#define BIST_USER_DATA_G			   (0x3FF << 0)

#define SST1_VIDEO_BIST_USER_DATA_B		   (0x545C)
#define BIST_USER_DATA_B			   (0x3FF << 0)

// SST1 Video Debug
#define SST1_VIDEO_DEBUG_FSM_STATE		   (0x5460)
#define DATA_PACK_FSM_STATE			   (0x3F << 16)
#define LINE_FSM_STATE				   (0x7 << 8)
#define PIXEL_FSM_STATE				   (0x7 << 0)

#define SST1_VIDEO_DEBUG_MAPI			   (0x5464)
#define MAPI_UNDERFLOW_STATUS			   (0x1 << 0)

#define SST1_VIDEO_DEBUG_HOR_BLANK_AUD_BW_ADJ	   (0x546C)
#define HOR_BLANK_AUD_BW_ADJ			   (0x1 << 0)

// SST1 Security CRC
#define SST1_SEC_CRC_CONTROL_1			   (0x5500)
#define SST1_SEC_CRC_CONTROL_2			   (0x5504)
#define SST1_SEC_CRC_CONTROL_3			   (0x5508)
#define SEC_CRC_RGB_RESULT			   (0xFFFF << 0)

#define SST1_SEC_CRC_CONTROL_4			   (0x550C)
#define SEC_CRC_PASS				   (0x1 << 5)
#define SEC_CRC_FAIL				   (0x1 << 4)
#define SEC_CRC_FAIL_MASK			   (0x1 << 3)
#define CLEAR_SEC_CRC_LFSR			   (0x1 << 2)
#define SEC_CRC_CLEAR				   (0x1 << 1)
#define SEC_CRC_ENABLE				   (0x1 << 0)

#define SST1_MIN_BLANKING_DUMMY_COUNT		   (0x5510)
#define MIN_BLANKING_DUMMY_COUNT		   (0x3FFF << 0)

// SST1 Audio Control
#define SST1_AUDIO_CONTROL			   (0x5800)
#define DMA_REQ_GEN_EN_SET(_v)			   (((_v)&0x1) << 30)
#define DMA_REQ_GEN_EN_MASK			   (0x1 << 30)
#define SW_AUD_CODING_TYPE			   (0x7 << 27)
#define AUD_DMA_IF_LTNCY_TRG_MODE		   (0x1 << 26)
#define AUD_DMA_IF_MODE_CONFIG			   (0x1 << 25)
#define AUD_ODD_CHANNEL_DUMMY			   (0x1 << 24)
#define AUD_M_VALUE_CMP_SPD_MASTER		   (0x7 << 21)
#define DMA_BURST_SEL_SET(_v)			   (((_v)&0x7) << 18)
#define DMA_BURST_SEL_MASK			   (0x7 << 18)
#define AUDIO_BIT_MAPPING_TYPE_SET(_v)		   (((_v)&0x3) << 16)
#define AUDIO_BIT_MAPPING_TYPE_MASK		   (0x3 << 16)
#define PCM_SIZE_SET(_v)			   (((_v)&0x3) << 13)
#define PCM_SIZE_MASK				   (0x3 << 13)
#define AUDIO_CH_STATUS_SAME_SET(_v)		   (((_v)&0x1) << 5)
#define AUDIO_CH_STATUS_SAME_MASK		   (0x1 << 5)
#define AUD_GTC_CHST_EN				   (0x1 << 1)

#define SST1_AUDIO_ENABLE			   (0x5804)
#define AUDIO_EN_GET(_v)			   (((_v) >> 0) & 0x1)
#define AUDIO_EN_SET(_v)			   (((_v)&0x1) << 0)
#define AUDIO_EN_MASK				   (0x1 << 0)

#define SST1_AUDIO_MASTER_TIMING_GEN		   (0x5808)
#define AUDIO_MASTER_TIME_GEN_SET(_v)		   (((_v)&0x1) << 0)
#define AUDIO_MASTER_TIME_GEN_MASK		   (0x1 << 0)

#define SST1_AUDIO_DMA_REQUEST_LATENCY_CONFIG	   (0x580C)
#define AUD_DMA_ACK_STATUS			   (0x1 << 21)
#define AUD_DMA_FORCE_ACK			   (0x1 << 20)
#define AUD_DMA_FORCE_ACK_SEL			   (0x1 << 19)
#define AUD_DMA_REQ_STATUS			   (0x1 << 18)
#define AUD_DMA_FORCE_REQ_VAL			   (0x1 << 17)
#define AUD_DMA_FORCE_REQ_SEL			   (0x1 << 16)
#define MASTER_DMA_REQ_LTNCY_CONFIG		   (0xFFFF << 0)

#define SST1_AUDIO_MUTE_CONTROL			   (0x5810)
#define AUD_MUTE_UNDRUN_EN			   (0x1 << 5)
#define AUD_MUTE_OVFLOW_EN			   (0x1 << 4)
#define AUD_MUTE_CLKCHG_EN			   (0x1 << 1)

#define SST1_AUDIO_MARGIN_CONTROL		   (0x5814)
#define FORCE_AUDIO_MARGIN			   (0x1 << 16)
#define AUDIO_MARGIN				   (0x1FFF << 0)

#define SST1_AUDIO_DATA_WRITE_FIFO		   (0x5818)
#define AUDIO_DATA_FIFO				   (0xFFFFFFFF << 0)

#define SST1_AUDIO_GTC_CONTROL			   (0x5824)
#define AUD_GTC_DELTA				   (0xFFFFFFFF << 0)

#define SST1_AUDIO_GTC_VALID_BIT_CONTROL	   (0x5828)
#define AUDIO_GTC_VALID_CONTROL			   (0x1 << 1)
#define AUDIO_GTC_VALID				   (0x1 << 0)

#define SST1_AUDIO_3DLPCM_PACKET_WAIT_TIMER	   (0x582C)
#define AUDIO_3D_PKT_WAIT_TIMER			   (0x3F << 0)

// SST1 Audio BIST Control
#define SST1_AUDIO_BIST_CONTROL			   (0x5830)
#define SIN_AMPL_SET(_v)			   (((_v)&0xF) << 4)
#define SIN_AMPL_MASK				   (0xF << 4)
#define AUD_BIST_EN_SET(_v)			   (((_v)&0x1) << 0)
#define AUD_BIST_EN_MASK			   (0x1 << 0)

#define SST1_AUDIO_BIST_CHANNEL_STATUS_SET0	   (0x5834)
#define CHNL_BIT1				   (0x3 << 30)
#define CLK_ACCUR_SET(_v)			   (((_v)&0x3) << 28)
#define CLK_ACCUR_MASK				   (0x3 << 28)
#define FS_FREQ_SET(_v)				   (((_v)&0xF) << 24)
#define FS_FREQ_MASK				   (0xF << 24)
#define CH_NUM_SET(_v)				   (((_v)&0xF) << 20)
#define CH_NUM_MASK				   (0xF << 20)
#define SOURCE_NUM_SET(_v)			   (((_v)&0xF) << 16)
#define SOURCE_NUM_MASK				   (0xF << 16)
#define CAT_CODE				   (0xFF << 8)
#define MODE					   (0x3 << 6)
#define PCM_MODE				   (0x7 << 3)
#define SW_CPRGT				   (0x1 << 2)
#define NON_PCM					   (0x1 << 1)
#define PROF_APP				   (0x1 << 0)

#define SST1_AUDIO_BIST_CHANNEL_STATUS_SET1	   (0x5838)
#define CHNL_BIT2				   (0xF << 4)
#define WORD_LENGTH_SET(_v)			   (((_v)&0x7) << 1)
#define WORD_LENGTH_MASK			   (0x7 << 1)
#define WORD_MAX_SET(_v)			   (((_v)&0x1) << 0)
#define WORD_MAX_MASK				   (0x1 << 0)

// SST1 Audio Channel Control
#define SST1_AUDIO_BUFFER_CONTROL		   (0x583C)
#define MASTER_AUDIO_INIT_BUFFER_THRD		   (0x7F << 24)
#define MASTER_AUDIO_BUFFER_THRD		   (0x3F << 18)
#define MASTER_AUDIO_BUFFER_EMPTY_INT_MASK	   (0x1 << 17)
#define MASTER_AUDIO_CHANNEL_COUNT_SET(_v)	   (((_v)&0x1F) << 12)
#define MASTER_AUDIO_CHANNEL_COUNT_MASK		   (0x1F << 12)
#define MASTER_AUDIO_BUFFER_LEVEL		   (0x7F << 5)
#define AUD_DMA_NOISE_INT_MASK			   (0x1 << 4)
#define AUD_DMA_NOISE_INT			   (0x1 << 3)
#define AUD_DMA_NOISE_INT_EN			   (0x1 << 2)
#define MASTER_AUDIO_BUFFER_EMPTY_INT		   (0x1 << 1)
#define MASTER_AUDIO_BUFFER_EMPTY_INT_EN	   (0x1 << 0)

#define SST1_AUDIO_CHANNEL_1_4_REMAP		   (0x5840)
#define AUD_CH_04_REMAP_SET(_v)			   (((_v)&0x3F) << 24)
#define AUD_CH_04_REMAP_MASK			   (0x3F << 24)
#define AUD_CH_03_REMAP_SET(_v)			   (((_v)&0x3F) << 16)
#define AUD_CH_03_REMAP_MASK			   (0x3F << 16)
#define AUD_CH_02_REMAP_SET(_v)			   (((_v)&0x3F) << 8)
#define AUD_CH_02_REMAP_MASK			   (0x3F << 8)
#define AUD_CH_01_REMAP_SET(_v)			   (((_v)&0x3F) << 0)
#define AUD_CH_01_REMAP_MASK			   (0x3F << 0)

#define SST1_AUDIO_CHANNEL_5_8_REMAP		   (0x5844)
#define AUD_CH_08_REMAP_SET(_v)			   (((_v)&0x3F) << 24)
#define AUD_CH_08_REMAP_MASK			   (0x3F << 24)
#define AUD_CH_07_REMAP_SET(_v)			   (((_v)&0x3F) << 16)
#define AUD_CH_07_REMAP_MASK			   (0x3F << 16)
#define AUD_CH_06_REMAP_SET(_v)			   (((_v)&0x3F) << 8)
#define AUD_CH_06_REMAP_MASK			   (0x3F << 8)
#define AUD_CH_05_REMAP_SET(_v)			   (((_v)&0x3F) << 0)
#define AUD_CH_05_REMAP_MASK			   (0x3F << 0)

#define SST1_AUDIO_CHANNEL_9_12_REMAP		   (0x5848)
#define AUD_CH_12_REMAP				   (0x3F << 24)
#define AUD_CH_11_REMAP				   (0x3F << 16)
#define AUD_CH_10_REMAP				   (0x3F << 8)
#define AUD_CH_09_REMAP				   (0x3F << 0)

#define SST1_AUDIO_CHANNEL_13_16_REMAP		   (0x584C)
#define AUD_CH_16_REMAP				   (0x3F << 24)
#define AUD_CH_15_REMAP				   (0x3F << 16)
#define AUD_CH_14_REMAP				   (0x3F << 8)
#define AUD_CH_13_REMAP				   (0x3F << 0)

#define SST1_AUDIO_CHANNEL_17_20_REMAP		   (0x5850)
#define AUD_CH_20_REMAP				   (0x3F << 24)
#define AUD_CH_19_REMAP				   (0x3F << 16)
#define AUD_CH_18_REMAP				   (0x3F << 8)
#define AUD_CH_17_REMAP				   (0x3F << 0)

#define SST1_AUDIO_CHANNEL_21_24_REMAP		   (0x5854)
#define AUD_CH_24_REMAP				   (0x3F << 24)
#define AUD_CH_23_REMAP				   (0x3F << 16)
#define AUD_CH_22_REMAP				   (0x3F << 8)
#define AUD_CH_21_REMAP				   (0x3F << 0)

#define SST1_AUDIO_CHANNEL_25_28_REMAP		   (0x5858)
#define AUD_CH_28_REMAP				   (0x3F << 24)
#define AUD_CH_27_REMAP				   (0x3F << 16)
#define AUD_CH_26_REMAP				   (0x3F << 8)
#define AUD_CH_25_REMAP				   (0x3F << 0)

#define SST1_AUDIO_CHANNEL_29_32_REMAP		   (0x585C)
#define AUD_CH_32_REMAP				   (0x3F << 24)
#define AUD_CH_31_REMAP				   (0x3F << 16)
#define AUD_CH_30_REMAP				   (0x3F << 8)
#define AUD_CH_29_REMAP				   (0x3F << 0)

#define SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_0	   (0x5860)
#define MASTER_AUD_GP0_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP0_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP0_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP0_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_1	   (0x5864)
#define MASTER_AUD_GP0_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_3_4_STATUS_CTRL_0	   (0x5868)
#define MASTER_AUD_GP1_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP1_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP1_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP1_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_3_4_STATUS_CTRL_1	   (0x586C)
#define MASTER_AUD_GP1_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_5_6_STATUS_CTRL_0	   (0x5870)
#define MASTER_AUD_GP2_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP2_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP2_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP2_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_5_6_STATUS_CTRL_1	   (0x5874)
#define MASTER_AUD_GP2_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_7_8_STATUS_CTRL_0	   (0x5878)
#define MASTER_AUD_GP3_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP3_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP3_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP3_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_7_8_STATUS_CTRL_1	   (0x587C)
#define MASTER_AUD_GP3_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_9_10_STATUS_CTRL_0	   (0x5880)
#define MASTER_AUD_GP4_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP4_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP4_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP4_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_9_10_STATUS_CTRL_1	   (0x5884)
#define MASTER_AUD_GP4_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_11_12_STATUS_CTRL_0	   (0x5888)
#define MASTER_AUD_GP5_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP5_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP5_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP5_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_11_12_STATUS_CTRL_1	   (0x588C)
#define MASTER_AUD_GP5_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_13_14_STATUS_CTRL_0	   (0x5890)
#define MASTER_AUD_GP6_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP6_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP6_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP6_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_13_14_STATUS_CTRL_1	   (0x5894)
#define MASTER_AUD_GP6_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_15_16_STATUS_CTRL_0	   (0x5898)
#define MASTER_AUD_GP7_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP7_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP7_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP7_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_15_16_STATUS_CTRL_1	   (0x589C)
#define MASTER_AUD_GP7_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_17_18_STATUS_CTRL_0	   (0x58A0)
#define MASTER_AUD_GP8_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP8_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP8_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP8_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_17_18_STATUS_CTRL_1	   (0x58A4)
#define MASTER_AUD_GP8_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_19_20_STATUS_CTRL_0	   (0x58A8)
#define MASTER_AUD_GP9_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP9_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP9_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP9_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_19_20_STATUS_CTRL_1	   (0x58AC)
#define MASTER_AUD_GP9_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_21_22_STATUS_CTRL_0	   (0x58B0)
#define MASTER_AUD_GP10_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP10_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP10_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP10_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_21_22_STATUS_CTRL_1	   (0x58B4)
#define MASTER_AUD_GP10_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_23_24_STATUS_CTRL_0	   (0x58B8)
#define MASTER_AUD_GP11_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP11_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP11_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP11_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_23_24_STATUS_CTRL_1	   (0x58BC)
#define MASTER_AUD_GP11_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_25_26_STATUS_CTRL_0	   (0x58C0)
#define MASTER_AUD_GP12_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP12_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP12_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP12_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_25_26_STATUS_CTRL_1	   (0x58C4)
#define MASTER_AUD_GP12_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_27_28_STATUS_CTRL_0	   (0x58C8)
#define MASTER_AUD_GP13_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP13_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP13_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP13_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_27_28_STATUS_CTRL_1	   (0x58CC)
#define MASTER_AUD_GP13_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_29_30_STATUS_CTRL_0	   (0x58D0)
#define MASTER_AUD_GP14_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP14_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP14_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP14_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_29_30_STATUS_CTRL_1	   (0x58D4)
#define MASTER_AUD_GP14_STA_4			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_31_32_STATUS_CTRL_0	   (0x58D8)
#define MASTER_AUD_GP15_STA_3			   (0xFF << 24)
#define MASTER_AUD_GP15_STA_2			   (0xFF << 16)
#define MASTER_AUD_GP15_STA_1			   (0xFF << 8)
#define MASTER_AUD_GP15_STA_0			   (0xFF << 0)

#define SST1_AUDIO_CHANNEL_31_32_STATUS_CTRL_1	   (0x58DC)
#define MASTER_AUD_GP15_STA_4			   (0xFF << 0)

// SST1 Stream Interface CRC
#define SST1_STREAM_IF_CRC_CONTROL_1		   (0x58E0)
#define IF_CRC_CLEAR				   (0x1 << 13)
#define IF_CRC_PASS				   (0x1 << 12)
#define IF_CRC_FAIL				   (0x1 << 8)
#define IF_CRC_SW_COMPARE			   (0x1 << 4)
#define IF_CRC_EN				   (0x1 << 0)

#define SST1_STREAM_IF_CRC_CONTROL_2		   (0x58E4)
#define IF_CRC_R_REF				   (0xFFFF << 16)
#define IF_CRC_R_RESULT				   (0xFFFF << 0)

#define SST1_STREAM_IF_CRC_CONTROL_3		   (0x58E8)
#define IF_CRC_G_REF				   (0xFFFF << 16)
#define IF_CRC_G_RESULT				   (0xFFFF << 0)

#define SST1_STREAM_IF_CRC_CONTROL_4		   (0x58EC)
#define IF_CRC_B_REF				   (0xFFFF << 16)
#define IF_CRC_B_RESULT				   (0xFFFF << 0)

// SST1 Audio Debug
#define SST1_AUDIO_DEBUG_MARGIN_CONTROL		   (0x5900)
#define AUDIO_DEBUG_MARGIN_EN			   (0x1 << 6)
#define AUDIO_DEBUG_MARGIN_VAL			   (0x3F << 0)

#define SST1_AUDIO_DEBUG_BLOCK_CNT_CONTROL	   (0x5904)
#define AUDIO_DEBUG_BLOCK_CNT			   (0xFF << 1)
#define AUDIO_DEBUG_BLOCK_CNT_RESET		   (0x1 << 0)

#define SST1_AUDIO_DEBUG_MUTE_FALL_BACK		   (0x5908)
#define AUDIO_DEBUG_MUTE_FALL_BACK		   (0x1 << 0)

// SST1 InfoFrame
#define SST1_SDP_SPLITTING_CONTROL		   (0x5C00)
#define AUD_SPLIT_DEBUG_CTRL_VBLANK		   (0x1 << 1)
#define SDP_SPLITTING_EN			   (0x1 << 0)

#define SST1_INFOFRAME_UPDATE_CONTROL		   (0x5C04)
#define HDR_INFO_UPDATE_SET(_v)			   (((_v)&0x1) << 4)
#define HDR_INFO_UPDATE_MASK			   (0x1 << 4)
#define AUDIO_INFO_UPDATE_SET(_v)		   (((_v)&0x1) << 3)
#define AUDIO_INFO_UPDATE_MASK			   (0x1 << 3)
#define AVI_INFO_UPDATE_SET(_v)			   (((_v)&0x1) << 2)
#define AVI_INFO_UPDATE_MASK			   (0x1 << 2)
#define MPEG_INFO_UPDATE_SET(_v)		   (((_v)&0x1) << 1)
#define MPEG_INFO_UPDATE_MASK			   (0x1 << 1)
#define SPD_INFO_UPDATE_SET(_v)			   (((_v)&0x1) << 0)
#define SPD_INFO_UPDATE_MASK			   (0x1 << 0)

#define SST1_INFOFRAME_SEND_CONTROL		   (0x5C08)
#define HDR_INFO_SEND_SET(_v)			   (((_v)&0x1) << 4)
#define HDR_INFO_SEND_MASK			   (0x1 << 4)
#define AUDIO_INFO_SEND_SET(_v)			   (((_v)&0x1) << 3)
#define AUDIO_INFO_SEND_MASK			   (0x1 << 3)
#define AVI_INFO_SEND_SET(_v)			   (((_v)&0x1) << 2)
#define AVI_INFO_SEND_MASK			   (0x1 << 2)
#define MPEG_INFO_SEND_SET(_v)			   (((_v)&0x1) << 1)
#define MPEG_INFO_SEND_MASK			   (0x1 << 1)
#define SPD_INFO_SEND_SET(_v)			   (((_v)&0x1) << 0)
#define SPD_INFO_SEND_MASK			   (0x1 << 0)

#define SST1_INFOFRAME_SDP_VERSION_CONTROL	   (0x5C0C)
#define INFOFRAME_SDP_HB3_SEL			   (0x1 << 1)
#define INFOFRAME_SDP_VERSION_SEL		   (0x1 << 0)

#define SST1_INFOFRAME_SPD_PACKET_TYPE		   (0x5C10)
#define SPD_TYPE_SET(_v)			   (((_v)&0xFF) << 0)
#define SPD_TYPE_MASK				   (0xFF << 0)

#define SST1_INFOFRAME_SPD_REUSE_PACKET_CONTROL	   (0x5C14)
#define SPD_REUSE_EN				   (0x1 << 0)

#define SST1_PPS_SDP_CONTROL			   (0x5C20)
#define PPS_SDP_CHANGE_STATUS			   (0x1 << 2)
#define PPS_SDP_FRAME_SEND_ENABLE		   (0x1 << 1)
#define PPS_SDP_UPDATE				   (0x1 << 0)

#define SST1_VSC_SDP_CONTROL_1			   (0x5C24)
#define VSC_TOTAL_BYTES_IN_SDP			   (0xFFF << 8)
#define VSC_CHANGE_STATUS			   (0x1 << 5)
#define VSC_FORCE_PACKET_MARGIN			   (0x1 << 4)
#define VSC_FIX_PACKET_SEQUENCE			   (0x1 << 3)
#define VSC_EXT_VESA_CEA			   (0x1 << 2)
#define VSC_SDP_FRAME_SEND_ENABLE		   (0x1 << 1)
#define VSC_SDP_UPDATE				   (0x1 << 0)

#define SST1_VSC_SDP_CONTROL_2			   (0x5C28)
#define VSC_SETUP_TIME				   (0xFFF << 20)
#define VSC_PACKET_MARGIN			   (0x1FFF << 0)

#define SST1_MST_WAIT_TIMER_CONTROL_1		   (0x5C2C)
#define AUDIO_WAIT_TIMER			   (0x1FFF << 16)
#define INFOFRAME_WAIT_TIMER			   (0x1FFF << 0)

#define SST1_MST_WAIT_TIMER_CONTROL_2		   (0x5C30)
#define PPS_WAIT_TIMER				   (0x1FFF << 16)
#define VSC_PACKET_WAIT_TIMER			   (0x1FFF << 0)

#define SST1_MST_WAIT_TIMER_CONTROL_3		   (0x5C34)
#define WAIT_TIMER_AFTER_ACTIVE_DATA		   (0x1FFF << 0)

#define MAX_AVI_INFOFRAME_DATA_BYTE_SIZE	   13 // 13 Bytes: Byte 1 ~ Byte 13
#define SST1_INFOFRAME_AVI_PACKET_DATA_SET0	   (0x5C40)
#define SST1_INFOFRAME_AVI_PACKET_DATA_SET1	   (0x5C44)
#define SST1_INFOFRAME_AVI_PACKET_DATA_SET2	   (0x5C48)
#define SST1_INFOFRAME_AVI_PACKET_DATA_SET3	   (0x5C4C)

#define MAX_AUDIO_INFOFRAME_DATA_BYTE_SIZE	   10 // 10 Bytes: Byte 1 ~ Byte 10
#define SST1_INFOFRAME_AUDIO_PACKET_DATA_SET0	   (0x5C50)
#define SST1_INFOFRAME_AUDIO_PACKET_DATA_SET1	   (0x5C54)
#define SST1_INFOFRAME_AUDIO_PACKET_DATA_SET2	   (0x5C58)

#define MAX_SPD_INFOFRAME_DATA_BYTE_SIZE	   25 // 25 Bytes: Byte 1 ~ Byte 25
#define SST1_INFOFRAME_SPD_PACKET_DATA_SET0	   (0x5C60)
#define SST1_INFOFRAME_SPD_PACKET_DATA_SET1	   (0x5C64)
#define SST1_INFOFRAME_SPD_PACKET_DATA_SET2	   (0x5C68)
#define SST1_INFOFRAME_SPD_PACKET_DATA_SET3	   (0x5C6C)
#define SST1_INFOFRAME_SPD_PACKET_DATA_SET4	   (0x5C70)
#define SST1_INFOFRAME_SPD_PACKET_DATA_SET5	   (0x5C74)
#define SST1_INFOFRAME_SPD_PACKET_DATA_SET6	   (0x5C78)

#define MAX_MPEG_INFOFRAME_DATA_BYTE_SIZE	   10 // 10 Bytes: Byte 1 ~ Byte 10
#define SST1_INFOFRAME_MPEG_PACKET_DATA_SET0	   (0x5C80)
#define SST1_INFOFRAME_MPEG_PACKET_DATA_SET1	   (0x5C84)
#define SST1_INFOFRAME_MPEG_PACKET_DATA_SET2	   (0x5C88)

#define MAX_SPD_REUSE_INFOFRAME_DATA_BYTE_SIZE	   4 // 4 Bytes: Byte 0 ~ Byte 3
#define SST1_INFOFRAME_SPD_REUSE_PACKET_HEADER_SET (0x5C90)
#define SST1_INFOFRAME_SPD_REUSE_PACKET_PARITY_SET (0x5C94)

#define SST1_INFOFRAME_HDR_PACKET_DATA_SET_0	   (0x5CA0)
#define SST1_INFOFRAME_HDR_PACKET_DATA_SET_1	   (0x5CA4)
#define SST1_INFOFRAME_HDR_PACKET_DATA_SET_2	   (0x5CA8)
#define SST1_INFOFRAME_HDR_PACKET_DATA_SET_3	   (0x5CAC)
#define SST1_INFOFRAME_HDR_PACKET_DATA_SET_4	   (0x5CB0)
#define SST1_INFOFRAME_HDR_PACKET_DATA_SET_5	   (0x5CB4)
#define SST1_INFOFRAME_HDR_PACKET_DATA_SET_6	   (0x5CB8)
#define SST1_INFOFRAME_HDR_PACKET_DATA_SET_7	   (0x5CBC)

#define SST1_PPS_SDP_PAYLOAD_0			   (0x5D00)
#define SST1_PPS_SDP_PAYLOAD_1			   (0x5D04)
#define SST1_PPS_SDP_PAYLOAD_2			   (0x5D08)
#define SST1_PPS_SDP_PAYLOAD_3			   (0x5D0C)
#define SST1_PPS_SDP_PAYLOAD_4			   (0x5D10)
#define SST1_PPS_SDP_PAYLOAD_5			   (0x5D14)
#define SST1_PPS_SDP_PAYLOAD_6			   (0x5D18)
#define SST1_PPS_SDP_PAYLOAD_7			   (0x5D1C)
#define SST1_PPS_SDP_PAYLOAD_8			   (0x5D20)
#define SST1_PPS_SDP_PAYLOAD_9			   (0x5D24)
#define SST1_PPS_SDP_PAYLOAD_10			   (0x5D28)
#define SST1_PPS_SDP_PAYLOAD_11			   (0x5D2C)
#define SST1_PPS_SDP_PAYLOAD_12			   (0x5D30)
#define SST1_PPS_SDP_PAYLOAD_13			   (0x5D34)
#define SST1_PPS_SDP_PAYLOAD_14			   (0x5D38)
#define SST1_PPS_SDP_PAYLOAD_15			   (0x5D3C)
#define SST1_PPS_SDP_PAYLOAD_16			   (0x5D40)
#define SST1_PPS_SDP_PAYLOAD_17			   (0x5D44)
#define SST1_PPS_SDP_PAYLOAD_18			   (0x5D48)
#define SST1_PPS_SDP_PAYLOAD_19			   (0x5D4C)
#define SST1_PPS_SDP_PAYLOAD_20			   (0x5D50)
#define SST1_PPS_SDP_PAYLOAD_21			   (0x5D54)
#define SST1_PPS_SDP_PAYLOAD_22			   (0x5D58)
#define SST1_PPS_SDP_PAYLOAD_23			   (0x5D5C)
#define SST1_PPS_SDP_PAYLOAD_24			   (0x5D60)
#define SST1_PPS_SDP_PAYLOAD_25			   (0x5D64)
#define SST1_PPS_SDP_PAYLOAD_26			   (0x5D68)
#define SST1_PPS_SDP_PAYLOAD_27			   (0x5D6C)
#define SST1_PPS_SDP_PAYLOAD_28			   (0x5D70)
#define SST1_PPS_SDP_PAYLOAD_29			   (0x5D74)
#define SST1_PPS_SDP_PAYLOAD_30			   (0x5D78)
#define SST1_PPS_SDP_PAYLOAD_31			   (0x5D7C)

#define SST1_VSC_SDP_DATA_PAYLOAD_FIFO		   (0x5D80)
#define VSC_SDP_DATA_PAYLOAD_FIFO		   (0xFFFFFFFF << 0)

#endif /* _REGS_DP_H */
