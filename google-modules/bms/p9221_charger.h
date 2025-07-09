/* SPDX-License-Identifier: GPL-2.0 */
/*
 * P9221 Wireless Charger Driver
 *
 * Copyright (C) 2017 Google, LLC.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#ifndef __P9221_CHARGER_H__
#define __P9221_CHARGER_H__

#include <linux/gpio.h>
#include <linux/crc8.h>
#include <misc/gvotable.h>
#include "gbms_power_supply.h"

#define P9221_WLC_VOTER				"WLC_VOTER"
#define P9221_USER_VOTER			"WLC_USER_VOTER"
#define P9221_OCP_VOTER				"OCP_VOTER"
#define DCIN_AICL_VOTER				"DCIN_AICL_VOTER"
#define P9382A_RTX_VOTER			"RTX_VOTER"
#define THERMAL_DAEMON_VOTER			"THERMAL_DAEMON_VOTER"
#define HPP_DC_ICL_VOTER			"HPP_VOTER"
#define DD_VOTER				"DD_VOTER"
#define AUTH_DC_ICL_VOTER			"AUTH_VOTER"
#define CPOUT_EN_VOTER				"CPOUT_EN_VOTER"
#define P9221_RAMP_VOTER			"WLC_RAMP_VOTER"
#define P9221_HPP_VOTER				"EPP_HPP_VOTER"
#define P9221_ALIGN_VOTER			"WLC_ALIGN_VOTER"
#define VOTABLE_DC_AVAIL			"DC_AVAIL"
#define P9221_ALIGN_VOTER			"WLC_ALIGN_VOTER"
#define WLC_MFG_GOOGLE				0x72
#define WLC_MFG_108_FOR_GOOGLE			0x108
#define P9221_DC_ICL_BPP_UA			700000
#define P9221_DC_ICL_BPP_RAMP_DEFAULT_UA	900000
#define P9221_DC_ICL_BPP_RAMP_DELAY_DEFAULT_MS	(7 * 60 * 1000)  /* 7 mins */
#define P9221_DC_ICL_EPP_UA			1100000
#define P9221_DC_ICL_HPP_UA			500000
#define P9221_DC_ICL_RTX_UA			600000
#define P9XXX_SW_RAMP_ICL_START_UA		125000
#define P9XXX_SW_RAMP_ICL_STEP_UA		100000
#define P9XXX_CDMODE_ENABLE_ICL_UA		200000
#define P9XXX_PROPMODE_ENABLE_ICL_UA		125000
#define P9221_AUTH_DC_ICL_UA_500		500000
#define P9221_EPP_THRESHOLD_UV			7000000
#define P9221_MAX_VOUT_SET_MV_DEFAULT		9000
#define P9221_VOUT_SET_MIN_MV			3500
#define P9221_VOUT_SET_MAX_MV			29000
#define P9221_RX_ILIM_MIN_MA			200
#define P9221_RX_ILIM_MAX_MA			1600
#define P9222_RX_ILIM_MIN_MA			100
#define P9222_RX_ILIM_MAX_MA			1500
#define P9382A_RTX_ICL_MAX_MA			1350
#define P9221R5_OVER_CHECK_NUM			3

#define P9412_VOUT_SET_MIN_MV			3520
#define P9412_VOUT_SET_MAX_MV			20000
#define P9412_RX_ILIM_MAX_MA			1900

#define P9221_TX_TIMEOUT_MS			(20 * 1000)
#define P9221_DCIN_TIMEOUT_MS			(1 * 1000)
#define P9221_DCIN_WAIT_CNT			4
#define P9221_CHARGE_STATS_TIMEOUT_MS		(10 * 1000)
#define P9221_VRECT_TIMEOUT_MS			(2 * 1000)
#define P9221_ALIGN_TIMEOUT_MS			(2 * 1000)
#define P9221_ALIGN_DELAY_MS			100
#define P9221_NOTIFIER_DELAY_MS			100
#define P9221_DCIN_PON_DELAY_MS			250
#define P9221R5_ILIM_MAX_UA			(1600 * 1000)

#define P9221_CHECK_NP_DELAY_MS		50
#define P9221_NEG_POWER_5W		(5 * 2)
#define P9221_NEG_POWER_10W		(10 * 2)
#define P9221_PTMC_EPP_TX_1912		0x32
#define P9221_PTMC_EPP_TX_4191		0x50
#define P9221_PTMC_EPP_TX_1801		0x28
#define P9221_PTMC_EPP_TX_2356		0x2356
#define P9221_PTMC_EPP_TX_2767		0x005D

#define P9222_RX_CALIBRATION_LIGHT_LOAD	0x5831
#define P9222_LIGHT_LOAD_VALUE		0x0C

#define P9221_DCIN_RETRY_DELAY_MS	50

#define P9XXX_DC_ICL_EPP_1000		1000000
#define P9XXX_DC_ICL_EPP_750		750000
#define P9XXX_DC_ICL_EPP_100		100000
#define P9XXX_NEG_POWER_10W		(10 * 2)
#define P9XXX_NEG_POWER_11W		(11 * 2)
#define P9XXX_TX_GUAR_PWR_12W		(12 * 2)
#define P9XXX_TX_GUAR_PWR_15W		(15 * 2)
#define P9382_RTX_TIMEOUT_MS		(2 * 1000)
#define WLCDC_DEBOUNCE_TIME_S		400
#define WLCDC_AUTH_CHECK_S		15
#define WLCDC_AUTH_CHECK_INTERVAL_MS	(2 * 1000)
#define WLCDC_AUTH_CHECK_INIT_DELAY_MS	(6 * 1000)
#define P9XXX_DC_ICL_GPP_UA		1500000
#define DET_READY_DEBOUNCE_MS		(3 * 1000)

#define I2C_LOG_NUM			128

/*
 * P9221 common registers
 */
#define P9221_CHIP_ID_REG			0x00
#define P9221_CHIP_ID				0x9220
#define P9221_CHIP_REVISION_REG			0x02
#define P9221_CUSTOMER_ID_REG			0x03
#define P9221R5_CUSTOMER_ID_VAL			0x05
#define P9221_OTP_FW_MAJOR_REV_REG		0x04
#define P9221_OTP_FW_MINOR_REV_REG		0x06
#define P9221_OTP_FW_DATE_REG			0x08
#define P9221_OTP_FW_DATE_SIZE			12
#define P9221_OTP_FW_TIME_REG			0x14
#define P9221_OTP_FW_TIME_SIZE			8
#define P9221_SRAM_FW_MAJOR_REV_REG		0x1C
#define P9221_SRAM_FW_MINOR_REV_REG		0x1E
#define P9221_SRAM_FW_DATE_REG			0x20
#define P9221_SRAM_FW_DATE_SIZE			12
#define P9221_SRAM_FW_TIME_REG			0x2C
#define P9221_SRAM_FW_TIME_SIZE			8
#define P9221_STATUS_REG			0x34
#define P9221_INT_REG				0x36
#define P9221_INT_MASK				0xF7
#define P9221_INT_ENABLE_REG			0x38
#define P9221_COM_REG				0x4E


/*
 * P9221R5 unique registers
 */
#define P9221R5_INT_CLEAR_REG			0x3A
#define P9221R5_VOUT_SET_REG			0x3C
#define P9221R5_ILIM_SET_REG			0x3D
#define P9221R5_ILIM_SET_MAX			0x0E	/* 0x0E = 1.6A */
#define P9221R5_CHARGE_STAT_REG			0x3E
#define P9221R5_EPT_REG				0x3F
#define P9221R5_VRECT_REG			0x40
#define P9221R5_VOUT_REG			0x42
#define P9221R5_IOUT_REG			0x44
#define P9221R5_OP_FREQ_REG			0x48
#define P9221R5_SYSTEM_MODE_REG			0x4C
#define P9221R5_COM_CHAN_RESET_REG		0x50
#define P9221R5_COM_CHAN_SEND_SIZE_REG		0x58
#define P9221R5_COM_CHAN_SEND_IDX_REG		0x59
#define P9221R5_COM_CHAN_RECV_SIZE_REG		0x5A
#define P9221R5_COM_CHAN_RECV_IDX_REG		0x5B
#define P9221R5_VRECT_ADC_REG			0x60
#define P9221R5_VOUT_ADC_REG			0x62
#define P9221R5_VOUT_ADC_MASK			0xFFF
#define P9221R5_IOUT_ADC_REG			0x64
#define P9221R5_IOUT_ADC_MASK			0xFFF
#define P9221R5_DIE_TEMP_ADC_REG		0x66
#define P9221R5_DIE_TEMP_ADC_MASK		0xFFF
#define P9221R5_AC_PERIOD_REG			0x68
#define P9221R5_TX_PINGFREQ_REG			0x6A
#define P9221R5_EXT_TEMP_REG			0x6C
#define P9221R5_EXT_TEMP_MASK			0xFFF
#define P9221R5_FOD_REG				0x70
#define P9221R5_NUM_FOD				16
#define P9221R5_DEBUG_REG			0x80
#define P9221R5_EPP_Q_FACTOR_REG		0x83
#define P9221R5_EPP_TX_GUARANTEED_POWER_REG	0x84
#define P9221R5_EPP_TX_POTENTIAL_POWER_REG	0x85
#define P9221R5_EPP_TX_CAPABILITY_FLAGS_REG	0x86
#define P9221R5_EPP_TX_CAPABILITY_FLAGS_AR	BIT(6)
#define P9221R5_EPP_RENEGOTIATION_REG		0x87
#define P9221R5_EPP_CUR_RPP_HEADER_REG		0x88
#define P9221R5_EPP_CUR_NEGOTIATED_POWER_REG	0x89
#define P9221R5_EPP_CUR_MAXIMUM_POWER_REG	0x8A
#define P9221R5_EPP_CUR_FSK_MODULATION_REG	0x8B
#define P9221R5_EPP_REQ_RPP_HEADER_REG		0x8C
#define P9221R5_EPP_REQ_NEGOTIATED_POWER_REG	0x8D
#define P9221R5_EPP_REQ_MAXIMUM_POWER_REG	0x8E
#define P9221R5_EPP_REQ_FSK_MODULATION_REG	0x8F
#define P9221R5_VRECT_TARGET_REG		0x90
#define P9221R5_VRECT_KNEE_REG			0x92
#define P9221R5_VRECT_CORRECTION_FACTOR_REG	0x93
#define P9221R5_VRECT_MAX_CORRECTION_FACTOR_REG	0x94
#define P9221R5_VRECT_MIN_CORRECTION_FACTOR_REG	0x96
#define P9221R5_FOD_SECTION_REG			0x99
#define P9221R5_VRECT_ADJ_REG			0x9E
#define P9221R5_ALIGN_X_ADC_REG			0xA0
#define P9221R5_ALIGN_Y_ADC_REG			0xA1
#define P9221R5_ASK_MODULATION_DEPTH_REG	0xA2
#define P9221R5_OVSET_REG			0xA3
#define P9221R5_OVSET_MASK			0x7
#define P9221R5_EPP_TX_SPEC_REV_REG		0xA9
#define P9221R5_EPP_TX_MFG_CODE_REG		0xAA
#define P9221R5_GP0_RESET_VOLT_REG		0xAC
#define P9221R5_GP1_RESET_VOLT_REG		0xAE
#define P9221R5_GP2_RESET_VOLT_REG		0xB0
#define P9221R5_GP3_RESET_VOLT_REG		0xB2
#define P9221R5_PROP_TX_ID_REG			0xB4
#define P9221R5_PROP_TX_ID_SIZE			4
#define P9221R5_DATA_SEND_BUF_START		0x100
#define P9221R5_DATA_SEND_BUF_SIZE		0x80
#define P9221R5_DATA_RECV_BUF_START		0x180
#define P9221R5_DATA_RECV_BUF_SIZE		0x80
#define P9221R5_MAX_PP_BUF_SIZE			16
#define P9221R5_LAST_REG			0x1FF

/*
 * System Mode Mask (R5+/0x4C)
 */
#define P9221R5_SYSTEM_MODE_EXTENDED_MASK	(1 << 3)

/*
 * Com Channel Commands
 */
#define P9221R5_COM_CHAN_CCRESET		BIT(7)
#define P9221_COM_CHAN_RETRIES			5

/*
 * End of Power packet types
 */
#define P9221_EOP_UNKNOWN			0x00
#define P9221_EOP_EOC				0x01
#define P9221_EOP_INTERNAL_FAULT		0x02
#define P9221_EOP_OVER_TEMP			0x03
#define P9221_EOP_OVER_VOLT			0x04
#define P9221_EOP_OVER_CURRENT			0x05
#define P9221_EOP_BATT_FAIL			0x06
#define P9221_EOP_RECONFIG			0x07
#define P9221_EOP_NO_RESPONSE			0x08
#define P9221_EOP_NEGOTIATION_FAIL		0x0A
#define P9221_EOP_RESTART_POWER			0x0B
#define P9221_EOP_REVERT_TO_BPP			0xF3

/*
 * Command flags
 */
#define P9221R5_COM_RENEGOTIATE			P9221_COM_RENEGOTIATE
#define P9221R5_COM_SWITCH2RAM			P9221_COM_SWITCH_TO_RAM_MASK
#define P9221R5_COM_CLRINT			P9221_COM_CLEAR_INT_MASK
#define P9221R5_COM_SENDCSP			P9221_COM_SEND_CHG_STAT_MASK
#define P9221R5_COM_SENDEPT			P9221_COM_SEND_EOP_MASK
#define P9221R5_COM_LDOTGL			P9221_COM_LDO_TOGGLE
#define P9221R5_COM_CCACTIVATE			BIT(0)

#define P9221_COM_RENEGOTIATE			BIT(7)
#define P9221_COM_SWITCH_TO_RAM_MASK		BIT(6)
#define P9221_COM_CLEAR_INT_MASK		BIT(5)
#define P9221_COM_SEND_CHG_STAT_MASK		BIT(4)
#define P9221_COM_SEND_EOP_MASK			BIT(3)
#define P9221_COM_LDO_TOGGLE			BIT(1)

/*
 * Interrupt/Status flags for P9221
 */
#define P9221_STAT_VOUT				BIT(7)
#define P9221_STAT_VRECT			BIT(6)
#define P9221_STAT_ACMISSING			BIT(5)
#define P9221_STAT_OV_TEMP			BIT(2)
#define P9221_STAT_OV_VOLT			BIT(1)
#define P9221_STAT_OV_CURRENT			BIT(0)
#define P9221_STAT_LIMIT_MASK			(P9221_STAT_OV_TEMP | \
						 P9221_STAT_OV_VOLT | \
						 P9221_STAT_OV_CURRENT)
/*
 * Interrupt/Status flags for P9221R5
 */
#define P9221R5_STAT_CCRESET			BIT(12)
#define P9221R5_STAT_CCERROR			BIT(11)
#define P9221R5_STAT_PPRCVD			BIT(10)
#define P9221R5_STAT_CCDATARCVD			BIT(9)
#define P9221R5_STAT_CCSENDBUSY			BIT(8)
#define P9221R5_STAT_VOUTCHANGED		BIT(7)
#define P9221R5_STAT_VRECTON			BIT(6)
#define P9221R5_STAT_MODECHANGED		BIT(5)
#define P9221R5_STAT_UV				BIT(3)
#define P9221R5_STAT_OVT			BIT(2)
#define P9221R5_STAT_OVV			BIT(1)
#define P9221R5_STAT_OVC			BIT(0)
#define P9221R5_STAT_MASK			0x1FFF
#define P9221R5_STAT_CC_MASK			(P9221R5_STAT_CCRESET | \
						 P9221R5_STAT_PPRCVD | \
						 P9221R5_STAT_CCERROR | \
						 P9221R5_STAT_CCDATARCVD | \
						 P9221R5_STAT_CCSENDBUSY)
#define P9221R5_STAT_LIMIT_MASK			(P9221R5_STAT_UV | \
						 P9221R5_STAT_OVV | \
						 P9221R5_STAT_OVT | \
						 P9221R5_STAT_OVC)

/*
 * P9221R5_SYSTEM_MODE_REG bits.
 */
#define P9221R5_MODE_RAMCODE			BIT(6)
#define P9221R5_MODE_EEPROMCODE			BIT(5)
#define P9221R5_MODE_EXTENDED			BIT(3)
#define P9221R5_MODE_WPCMODE			BIT(0)

/*
 * P9222 unique registers
 */
#define P9222_CHIP_ID				0x9222
#define P9222RE_SYSTEM_MODE_REG			0x3F
#define P9222RE_CHARGE_STAT_REG			0x4E
#define P9222RE_EPT_REG				0x4F
#define P9222RE_VOUT_REG			0x50
#define P9222RE_VOUT_SET_REG			0x52
#define P9222RE_VRECT_REG			0x54
#define P9222RE_IOUT_REG			0x58
#define P9222RE_DIE_TEMP_REG			0x5A
#define P9222RE_OP_FREQ_REG			0x5C
#define P9222RE_TX_PINGFREQ_REG			0x5E
#define P9222RE_ILIM_SET_REG			0x60
#define P9222RE_COM_REG				0x62
#define P9222RE_ASK_MOD_REG			0x7F
#define P9222RE_FOD_REG				0x84
#define P9222RE_COM_CHAN_RECV_SIZE_REG		0x98
#define P9222RE_EPP_TX_GUARANTEED_POWER_REG	0xB4
#define P9222RE_EPP_REQ_NEGOTIATED_POWER_REG	0xBD
#define P9222RE_EPP_REQ_MAXIMUM_POWER_REG	0xBE
#define P9222RE_EPP_Q_FACTOR_REG		0xD2
#define P9222RE_RESONANCE_FREQ_REG		0xD3
#define P9222RE_TX_MFG_CODE_REG			0x106
#define P9222RE_PROP_TX_ID_REG			0x118
#define P9222RE_DIE_TEMP_ADC_REG		0x12A
#define P9222RE_FREQ_LIMIT_REG			0x14A
#define P9222RE_COM_PACKET_TYPE_ADDR		0x600
#define P9222RE_COM_CHAN_SEND_SIZE_REG		0x601
#define P9222RE_DATA_BUF_START			0x604
#define P9222RE_DATA_BUF_SIZE			0x100
#define P9222RE_PP_SEND_BUF_START		0x64
#define P9222RE_PP_RECV_BUF_START		0x6C

#define P9222RE_COM_CCACTIVATE			BIT(9)

#define P9222_MOT_REG				0x152
#define P9222_MOT_70PCT				0x0E
#define P9222_MOT_60PCT				0x0C
#define P9222_MOT_50PCT				0x0A
#define P9222_MOT_40PCT				0x08
#define P9222_MOT_30PCT				0x06

/*
 * P9222 SYSTEM_MODE_REG bits
 */
#define P9222_SYS_OP_MODE_WPC_BASIC		BIT(5)
#define P9222_SYS_OP_MODE_WPC_EXTD		BIT(6)

#define P9222_VOUT_SET_MIN_MV			3500
#define P9222_VOUT_SET_MAX_MV			12500
#define P9222_NEG_POWER_10W			10000

/*
 * Interrupt/Status flags for P9222
 */
#define P9222_STAT_CCERROR			BIT(0)
#define P9222_STAT_OVT				BIT(2)
#define P9222_STAT_OVC				BIT(3)
#define P9222_STAT_OVV				BIT(4)
#define P9222_EXTENDED_MODE			BIT(12)
#define P9222_STAT_PPRCVD			BIT(15)

/*
 * P9382 unique registers
 */
#define P9382A_I2C_ADDRESS			0x3b

#define P9382A_CHIP_ID				0x9381 /* FIXME: b/146316852 */
#define P9382A_DATA_SEND_BUF_START		0x130
#define P9382A_DATA_RECV_BUF_START		0x1B0

#define P9382A_STATUS_REG			0x34
#define P9382A_CHARGE_STAT_REG			0x3E
#define P9382A_ILIM_SET_REG			0x4A
#define P9382A_TRX_ENABLE_REG			0x69
#define P9382A_TX_INHIBIT			0x3

#define P9382A_MODE_TXMODE			BIT(2)
#define P9382A_MODE_MASK			0xFF

#define P9382_PROP_TX_ID_REG			0xC4
#define P9382_EPP_TX_MFG_CODE_REG		0xBA
#define P9382A_FW_REV_25			0x25

/*
 * Interrupt/Status flags for P9382
 */
#define P9382_STAT_HARD_OCP			BIT(1)
#define P9382_STAT_TXCONFLICT			BIT(3)
#define P9382_STAT_CSP				BIT(4)
#define P9382_STAT_TXUVLO			BIT(6)
#define P9382_STAT_RXCONNECTED			BIT(10)
#define P9382_STAT_TXUNDERPOWER			BIT(12)
#define P9382_STAT_TXFOD			BIT(13)
#define P9382_STAT_RTX_MASK			(P9221R5_STAT_LIMIT_MASK | \
						 P9221R5_STAT_MODECHANGED | \
						 P9221R5_STAT_VOUTCHANGED | \
						 P9382_STAT_TXCONFLICT | \
						 P9382_STAT_CSP | \
						 P9382_STAT_TXUVLO | \
						 P9382_STAT_RXCONNECTED | \
						 P9382_STAT_TXUNDERPOWER | \
						 P9382_STAT_TXFOD)
/*
 * Send communication message
 */
#define P9382A_COM_PACKET_TYPE_ADDR		0x100
#define P9382A_COM_CHAN_SEND_SIZE_REG		0x101
#define BIDI_COM_PACKET_TYPE			0x98
#define PROPRIETARY_PACKET_TYPE			0x80
#define CHANNEL_RESET_PACKET_TYPE		0xA8
#define FAST_SERIAL_ID_HEADER			0x4F
#define FAST_SERIAL_ID_SIZE			4
#define ACCESSORY_TYPE_MASK			0x7
#define CHARGE_STATUS_PACKET_HEADER		0x48
#define CHARGE_STATUS_PACKET_SIZE		4
#define PP_TYPE_POWER_CONTROL			0x08
#define PP_SUBTYPE_SOC				0x10
#define ACCESSORY_TYPE_PHONE			BIT(2)
#define AICL_ENABLED				BIT(7)
#define TX_ACCESSORY_TYPE			(ACCESSORY_TYPE_PHONE | \
						 AICL_ENABLED)
#define TXID_SEND_DELAY_MS			(1 * 1000)
#define TXID_SEND_AGAIN_DELAY_MS		(300 * 1000)
#define TXSOC_SEND_DELAY_MS			(5 * 1000)

#define COM_BUSY_MAX				10
/*
 * P9412 unique registers
 */
#define P9412_CHIP_ID				0x9412

/* P9221R5_SYSTEM_MODE_REG(0x4C) values for P9412 */
#define P9XXX_SYS_OP_MODE_AC_MISSING		0x00 /* AC Missing */
#define P9XXX_SYS_OP_MODE_WPC_BASIC		0x01 /* WPC Basic Protocol */
#define P9XXX_SYS_OP_MODE_WPC_EXTD		0x02 /* WPC Extended Protocol */
#define P9XXX_SYS_OP_MODE_PROPRIETARY		0x03 /* Renesas Prop. Protocol */
#define P9XXX_SYS_OP_MODE_TX_MODE		0x08 /* TX Mode */
#define P9XXX_SYS_OP_MODE_TX_FOD		0x09 /* TX FOD (Stop) */

#define P9412_MODE_MASK				0xFF
#define RA9530_MODE_MASK			0x0F

#define P9412_TX_CMD_REG			0x4D
#define P9412_TX_I_API_LIM_REG			0x56
#define P9412_ALIGN_X_REG			0xB0 /* 1 byte 8 bit raw */
#define P9412_ALIGN_Y_REG			0xB1 /* 1 byte 8 bit raw */
#define P9412_EPP_CAL_STATE_REG			0xB8
#define P9412_WPC_SPEC_REV_REG			0xB9
#define P9412_PROP_TX_POTEN_PWR_REG		0xC4
#define P9412_PROP_REQ_PWR_REG			0xC5
#define P9412_PROP_CURR_PWR_REG			0xC6
#define P9412_PROP_MODE_PWR_STEP_REG		0xC7
#define P9412_PROP_MODE_STATUS_REG		0xC8
#define P9412_PROP_MODE_ERR_STS_REG		0xC9
#define P9412_VOUT_SET_REG			0x6C /* 2 byte 10 mV */
#define P9412_DIE_TEMP_REG			0x46 /* 2 byte in C */
#define P9412_V5P0AP_SWITCH_REG			0x81
#define V5P0AP_SWITCH_EN			BIT(7)
#define P9412_VCPOUT_VOL_REG			0x10C

#define P9412_CMFET_L_REG			0xF4
#define P9412_CDMODE_STS_REG			0x100
#define P9412_CDMODE_REQ_REG			0x101
#define P9412_HIVOUT_CMFET_REG			0x11B
#define P9412_COM_CHAN_RESET_REG		0x13F
#define P9412_COM_PACKET_TYPE_ADDR		0x800
#define P9412_COM_CHAN_SEND_SIZE_REG		0x801
#define P9412_COM_CHAN_SEND_IDX_REG		0x142
#define P9412_COM_CHAN_RECV_SIZE_REG		0x144
#define P9412_COM_CHAN_RECV_IDX_REG		0x146
#define P9412_COM_CHAN_STATUS_REG		0x148
#define P9412_PROP_TX_ID_REG			0x154

#define P9412_DATA_BUF_START			0x804
#define P9412_DATA_BUF_SIZE			0x7FC /* 2044 bytes */
#define P9412_PP_SEND_BUF_START			0x50
#define P9412_PP_RECV_BUF_START			0x58

#define P9412_RN_MAX_POLL_ATTEMPTS		5
#define P9412_RN_DELAY_MS			50
#define P9412_RN_STATUS_DONE			BIT(1)
#define P9412_RN_STATUS_ERROR			BIT(2)

#define P9XXX_INVALID_REG			0xFFFF
#define P9XXX_INT_CLEAR_MASK			0xFFFF

#define P9412_TX_CMD_TX_MODE_EN			BIT(7)
/* For Cap Div mode register */
#define CDMODE_BYPASS_MODE			BIT(0)
#define CDMODE_CAP_DIV_MODE			BIT(1)
/* For cmd register */
#define INIT_CAP_DIV_CMD			BIT(6)
#define PROP_MODE_EN_CMD			BIT(8)
#define PROP_REQ_PWR_CMD			BIT(9)
#define P9412_COM_CCACTIVATE			BIT(10)
/* For tx cmd register */
#define P9412_CMD_TXMODE_EXIT			BIT(9)
/* For INT status register */
#define P9412_STAT_PPRCVD			BIT(15)
#define P9412_CDMODE_ERROR_INT			BIT(14)
#define P9412_PROP_MODE_STAT_INT		BIT(12)
#define P9412_CDMODE_CHANGE_INT			BIT(11)
#define P9412_STAT_CCSENDBUSY			BIT(8)
#define P9412_STAT_CCDATARCVD			BIT(9)
#define P9412_STAT_CCERROR			BIT(0)
#define P9412_STAT_OVV				BIT(4)
#define P9412_STAT_OVC				BIT(3)
#define P9412_STAT_OVT				BIT(2)
#define P9412_STAT_OCP_PING			BIT(12)
#define P9412_STAT_RXCONNECTED			BIT(11)
#define P9412_STAT_PPPSENT			BIT(9)
#define P9412_STAT_CSP				BIT(10)
#define P9412_STAT_TXCONFLICT			BIT(1)
/* EPT code */
#define EPT_END_OF_CHARGE			BIT(0)
/* EPP calibration state */
#define P9412_CAL_STATE_1			BIT(1)
#define P9412_CAL_STATE_2			BIT(3)
#define P9412_EPP_CAL_STATE_MASK		(P9412_CAL_STATE_1 | \
						 P9412_CAL_STATE_2)
/* Rx Communication Modulation FET(CMFET) */
#define P9412_CMFET_DISABLE_ALL			(0xF0) /* CM-A/B-1/2: REG[7:4]=0b1111 */
#define P9412_CMFET_DEFAULT			(0x30) /* REG[7:4]=0b0011 */
#define P9412_CMFET_2_COMM			(0xC0) /* REG[7:4]=0b1100 */
#define P9412_CMFET_ENABLE_ALL			(0x00)
#define RA9530_CMFET_DISABLE_ALL		(0x00)
#define RA9530_CMFET_COM_1_2			(0xC0)
#define RA9530_CMFET_COM_A_B			(0x30)
#define RA9530_CMFET_COM_1_B			(0x60)
#define RA9530_CMFET_ENABLE_ALL			(0xF0)

#define P9221_CRC8_POLYNOMIAL			0x07    /* (x^8) + x^2 + x + 1 */
#define P9412_ADT_TYPE_AUTH			0x02

#define P9XXX_CHARGER_FEATURE_CACHE_SIZE	32
#define HPP_MODE_PWR_REQUIRE			23000
#define EPP_MODE_REQ_PWR			15000
#define WAIT_PROP_IRQ_MS			3000

#define RTX_RESET_COUNT_MAX			3
#define P9XXX_WPC_REV_13			0x13

/* p9412 AP BOOST PING register */
#define P9412_APBSTPING_REG			0xF0
#define P9412_APBSTCONTROL_REG			0xF1
#define P9412_APBSTPING_7V			BIT(0)
#define P9412_TXOCP_REG				0xA0
#define P9412_TXOCP_1400MA			1400
#define P9412_PLIM_REG				0x3D
#define P9412_PLIM_1200MA			0x0E
#define P9412_I_API_Limit			0x56
#define P9412_I_API_Limit_1350MA		1350
#define P9412_I_API_Hys				0x82
#define P9412_I_API_Hys_08			0x6A
#define P9412_MIN_FREQ_PER			0x94
#define P9412_MIN_FREQ_PER_120			1000	/* 120000/120 */
#define P9412_TX_FOD_THRSH_REG			0xD4
#define P9412_TX_FOD_THRSH_1600			0x640

#define P9412_MOT_REG				0xD0
#define P9412_MOT_40PCT				0x10
#define P9412_MOT_65PCT				0x1A
#define P9412_MOT_30PCT                         0x0C

#define P9412_HPP_FOD_SETS			8

#define P9XXX_IOP_MFG_NUM			8

/* RA9530 */

#define RA9530_CHIP_ID				0x9530
#define RA9530_DATA_BUF_SIZE			0x400 /* 1024 bytes */
#define RA9530_DATA_BUF_READ_START		0x800 /* 0x800 ~ 0xBFF */
#define RA9530_DATA_BUF_WRITE_START		0x400 /* 0x400 ~ 0x7FF */
#define RA9530_ILIM_REG				0x2C0 /* 0x400 ~ 0x7FF */
#define RA9530_TXOCP_1400MA			1400
#define RA9530_I_API_Limit_1350MA		1350
#define RA9530_TX_FOD_THRSH_1600		0x640
#define RA9530_CC_WRITE_TYPE_REG		0x3F0
#define RA9530_CC_TYPE_MASK			0xF8
#define RA9530_CC_TYPE_SHIFT			3
#define RA9530_CC_WRITE_SIZE_REG		0x3F2
#define RA9530_CC_WRITE_INDEX_REG		0x3F4
#define RA9530_CC_READ_SIZE_REG			0x3FA
#define RA9530_CC_READ_INDEX_REG		0x3FC
#define RA9530_BIDI_COM_PACKET_TYPE		0x13
#define RA9530_PROP_MODE_PWR_STEP		6
#define RA9530_RX_ILIM_MAX_MA			2600
#define RA9530_EPP_CAL_STATE_MASK		0x2F
#define RA9530_V21_EPP_CAL_STATE_MASK		0xF
#define RA9530_EPP_CALIBRATED_STATE		0x20
#define RA9530_PLIM_REG				0x2B8
#define RA9530_PLIM_900MA			0x384
#define RA9530_FREQ_PER_120			RA9530_KHZ_TO_REG(120) /* 60000/120 - 1 */
#define RA9530_FREQ_PER_105			RA9530_KHZ_TO_REG(105)
#define RA9530_HB_PING_FREQ_REG			0x98
#define RA9530_FB_MIN_FREQ_REG			0x94
#define RA9530_HB_MIN_FREQ_REG			0xA8
#define RA9530_TX_FB_HB_REG			0x1A0
#define RA9530_ILIM_MAX_UA			(1700 * 1000)
#define RA9530_ILIM_500UA			(500 * 1000)
#define P9XXX_OP_DUTY_REG			0xA6
#define P9XXX_TX_CUR_PWR_REG			0xAC
#define P9XXX_RX_CUR_PWR_REG			0xCE
#define RA9530_CMFET_REG			0xF4
#define RA9530_EPP_RF_REG			0x27C
#define RA9530_FW_REV_22			0x22
#define RA9530_OVSET_REG			0xB3
#define RA9530_OVSET_13V			4
#define RA9530_OVSET_16_7V			2
#define RA9530_OVSET_24_7V			6

#define OVSET_BPP				0
#define OVSET_EPP				1
#define OVSET_HPP				2

/* Features */
typedef enum {
    WLCF_DISABLE_ALL_FEATURE     = 0x00,
    WLCF_DREAM_ALIGN             = 0x01,
    WLCF_DREAM_DEFEND            = 0x02,
    WLCF_FAST_CHARGE             = 0x04,
    WLCF_CHARGE_15W              = 0x08,
} wlc_features_t;

/* for DD */
#define TXID_TYPE_MASK			0xFF000000 /* bit[24-31] */
#define TXID_TYPE_SHIFT			24
#define TXID_DD_TYPE			0xE0
#define TXID_DD_TYPE2			0xA0
#define P9221_POWER_MITIGATE_DELAY_MS   (10 * 1000)
#define P9221_FOD_MAX_TIMES             3
/* for RTx */
#define TXID_PHONE_TYPE_MASK            0xFE
#define TXID_PHONE_TYPE_CC              0x5


enum p9221_align_mfg_chk_state {
	ALIGN_MFG_FAILED = -1,
	ALIGN_MFG_CHECKING,
	ALIGN_MFG_PASSED,
};

enum p9xxx_chk_rp {
	RP_NOTSET = -1,
	RP_CHECKING,
	RP_DONE,
};

#define WLC_SOC_STATS_LEN      101

struct p9221_soc_data {
	ktime_t last_update;
	int elapsed_time;
	int pout_min;
	int pout_max;
	int of_freq;
	int alignment;
	int vrect;
	int iout;
	int die_temp;
	int sys_mode;
	long pout_sum;
};

struct p9221_charge_stats {
	struct mutex stats_lock;
	ktime_t start_time;
	struct p9221_soc_data soc_data[WLC_SOC_STATS_LEN];
	int adapter_type;
	int cur_soc;
	int volt_conf;
	int cur_conf;
	int of_freq;
	int last_soc;

	u32 adapter_capabilities[5];
	u32 receiver_state[2];
};

struct p9221_charger_feature_entry {
	u64 quickid;
	u64 features;
	u32 last_use;
};

struct p9221_charger_feature {
	struct mutex	feat_lock;

	struct p9221_charger_feature_entry entries[P9XXX_CHARGER_FEATURE_CACHE_SIZE];
	int num_entries;
	u32 age;

	u64 session_features;
	bool session_valid;
};

struct p9221_charger_cc_data_lock {
	bool cc_use;
	ktime_t cc_rcv_at;
};

struct p9221_fod_data {
	int num;
	u8 fod[P9221R5_NUM_FOD];
};

struct p9221_charger_platform_data {
	int				irq_gpio;
	int				irq_int;
	u64				irq_flag;
	int				irq_det_gpio;
	int				irq_det_int;
	int				qien_gpio;
	int				ldo_en_gpio;
	int				slct_gpio;
	int				slct_value;
	int				ben_gpio;
	int				ext_ben_gpio;
	int				switch_gpio;
	int				boost_gpio;
	int				dc_switch_gpio;
	int				wcin_inlim_en_gpio;
	int				qi_vbus_en;
	int				qi_vbus_en_act_low;
	int				wlc_en;
	int				wlc_en_act_low;
	int				max_vout_mv;
	int				epp_vout_mv;
	u8				fod[P9221R5_NUM_FOD];
	u8				fod_lv[P9221R5_NUM_FOD];
	u8				fod_epp[P9221R5_NUM_FOD];
	u8				fod_gpp[P9221R5_NUM_FOD];
	u8				fod_epp_comp[P9221R5_NUM_FOD];
	u8				fod_epp_iop[P9221R5_NUM_FOD];
	u8				fod_hpp[P9221R5_NUM_FOD];
	u8				fod_hpp_hv[P9221R5_NUM_FOD];
	struct p9221_fod_data		hpp_fods[P9412_HPP_FOD_SETS];
	int				fod_num;
	int				fod_lv_num;
	int				fod_epp_num;
	int				fod_gpp_num;
	int				fod_epp_comp_num;
	int				fod_epp_iop_num;
	int				fod_hpp_num;
	int				fod_hpp_hv_num;
	bool				fod_fsw;
	int				fod_fsw_high;
	int				fod_fsw_low;
	u16				fod_iop_mfg[P9XXX_IOP_MFG_NUM];
	int				fod_iop_mfg_num;
	int				q_value;
	int				rf_value;
	int				tx_4191q;
	int				tx_1801q;
	int				tx_2356q;
	u32				tx_2767_icl;
	int				epp_rp_value;
	int				epp_rp_low_value;
	int				needs_dcin_reset;
	int				nb_alignment_freq;
	int				*alignment_freq;
	u32				alignment_scalar;
	u32				alignment_hysteresis;
	u32				icl_ramp_delay_ms;
	u16				chip_id;
	bool				has_wlc_dc;
	bool				gpp_enhanced;
	bool				enable_15w;
	bool				has_rtx;
	bool				has_rtx_gpio;
	u32				power_mitigate_threshold;
	u32				power_mitigate_ac_threshold;
	u32				alignment_scalar_low_current;
	u32				alignment_scalar_high_current;
	u32				alignment_offset_low_current;
	u32				alignment_offset_high_current;
	u32				alignment_current_threshold;
	bool				feat_compat_mode;
	bool				apbst_en;
	bool				has_sw_ramp;
	bool				hw_ocp_det;
	bool				needs_align_check;
	/* phone type for tx_id*/
	u8				phone_type;
	u32				epp_icl;		/* EPP default ICL ua*/
	u32				gpp_icl;		/* GPP 15W ICL ua */
	u32				bpp_icl;		/* BPP default ICL ua */
	u32				bpp_lv_icl;		/* BPP ICL with lower Vout */
	u32				bpp_icl_ramp_ua;	/* BPP ramp ICL */
	u32				bpp_lv_icl_ramp_ua;	/* BPP ramp ICL with lower Vout */
	u32				dc_icl_gpp;		/* DC_ICL for GPP */
	/* calibrate light load */
	bool				light_load;
	int				nb_hpp_fod_vol;
	int				*hpp_fod_vol;
	bool				disable_align;
	bool				ll_vout_not_set;
	int				align_delta;
	bool				disable_repeat_eop;
	u32				hpp_neg_pwr;
	u32				epp_neg_pwr;
	u32				wait_prop_irq_ms;

	bool				bpp_cep_on_dl;
	bool				hda_tz_wlc;
	u32				set_iop_vout_bpp;
	u32				set_iop_vout_epp;
	u32				lowest_fsw_khz;
	u32				gpp_cmfet;
	bool				freq_108_disable_ramp;
	bool				magsafe_optimized;
	u8				ask_mod_fet;
	u32				freq_109_icl;
	u32				freq_109_vout;
};

struct p9221_charger_ints_bit {
	/* Rx mode */
	u16				over_curr_bit;
	u16				over_volt_bit;
	u16				over_temp_bit;
	u16				over_uv_bit;
	u16				mode_changed_bit;
	u16				vrecton_bit;
	u16				vout_changed_bit;
	u16				cc_send_busy_bit;
	u16				cc_data_rcvd_bit;
	u16				pp_rcvd_bit;
	u16				cc_error_bit;
	u16				cc_reset_bit;
	u16				propmode_stat_bit;
	u16				cdmode_change_bit;
	u16				cdmode_err_bit;
	u16				stat_limit_mask;
	u16				stat_cc_mask;
	u16				prop_mode_mask;
	u16				extended_mode_bit;
	/* Tx mode */
	u16				hard_ocp_bit;
	u16				tx_conflict_bit;
	u16				csp_bit;
	u16				rx_connected_bit;
	u16				tx_fod_bit;
	u16				tx_underpower_bit;
	u16				tx_uvlo_bit;
	u16				pppsent_bit;
	u16				ocp_ping_bit;
	u16				stat_rtx_mask;
};

struct p9221_charger_data {
	struct i2c_client		*client;
	struct p9221_charger_platform_data *pdata;
	struct p9221_charger_ints_bit	ints;
	struct power_supply		*wc_psy;
	struct power_supply		*dc_psy;
	struct power_supply		*batt_psy;
	struct gvotable_election	*dc_icl_votable;
	struct gvotable_election	*msc_last_votable;
	struct gvotable_election	*dc_suspend_votable;
	struct gvotable_election	*tx_icl_votable;
	struct gvotable_election	*disable_dcin_en_votable;
	struct gvotable_election	*chg_mode_votable;
	struct gvotable_election	*wlc_disable_votable;
	struct gvotable_election	*csi_status_votable;
	struct gvotable_election	*csi_type_votable;
	struct gvotable_election	*point_full_ui_soc_votable;
	struct gvotable_election	*dc_avail_votable;
	struct gvotable_election	*hda_tz_votable;
	struct gvotable_election	*bcl_wlc_votable;
	struct gvotable_election	*wlc_spoof_votable;
	struct gvotable_election	*fan_level_votable;
	struct notifier_block		nb;
	struct mutex			io_lock;
	struct mutex			cmd_lock;
	struct mutex			fod_lock;
	struct device			*dev;
	struct delayed_work		notifier_work;
	struct delayed_work		charge_stats_hda_work;
	struct delayed_work		dcin_work;
	struct delayed_work		stop_online_spoof_work;
	struct delayed_work		change_det_status_work;
	struct delayed_work		align_work;
	struct delayed_work		tx_work;
	struct delayed_work		icl_ramp_work;
	struct delayed_work		txid_work;
	struct delayed_work		rtx_work;
	struct delayed_work		power_mitigation_work;
	struct delayed_work		auth_dc_icl_work;
	struct delayed_work		soc_work;
	struct delayed_work		chk_rp_work;
	struct delayed_work		chk_rtx_ocp_work;
	struct delayed_work		chk_fod_work;
	struct delayed_work		set_rf_work;
	struct work_struct		uevent_work;
	struct work_struct		calibration_work;
	struct work_struct		rtx_disable_work;
	struct work_struct		rtx_reset_work;
	struct alarm			icl_ramp_alarm;
	struct alarm			auth_dc_icl_alarm;
	struct timer_list		vrect_timer;
	struct timer_list		align_timer;
	struct bin_attribute		bin;
	struct logbuffer		*log;
	struct logbuffer		*rtx_log;
	struct dentry			*debug_entry;
	struct p9221_charger_feature	chg_features;
	struct p9221_charger_cc_data_lock	cc_data_lock;
	struct wakeup_source		*align_ws;
	struct wakeup_source		*det_status_ws;
	u16				chip_id;
	int				online;
	bool				enabled;
	u16				addr;
	u8				count;
	u8				cust_id;
	int				ben_state;
	u8				pp_buf[P9221R5_MAX_PP_BUF_SIZE];
	char				pp_buf_str[P9221R5_MAX_PP_BUF_SIZE * 3 + 1];
	bool				pp_buf_valid;
	u8				*rx_buf;
	size_t				rx_buf_size;
	u16				rx_len;
	bool				rx_done;
	u8				*tx_buf;
	char				fast_id_str[FAST_SERIAL_ID_SIZE * 3 + 1];
	size_t				tx_buf_size;
	u32				tx_id;
	u8				tx_id_str[(sizeof(u32) * 2) + 1];
	u16				tx_len;
	u16				auth_type;
	bool				tx_done;
	bool				tx_busy;
	u32				com_busy;
	bool				check_np;
	bool				check_dc;
	bool				check_det;
	int				dcin_waitcnt;
	int				last_capacity;
	bool				resume_complete;
	bool				icl_ramp;
	u32				icl_ramp_ua;
	u32				icl_ramp_alt_ua;
	bool				fake_force_epp;
	bool				force_bpp;
	u32				dc_icl_epp_neg;
	u32				dc_icl_bpp;
	int				align;
	int				align_count;
	int				alignment;
	u8				alignment_str[(sizeof(u32) * 3) + 1];
	int				alignment_last;
	enum p9221_align_mfg_chk_state  alignment_capable;
	int				mfg_check_count;
	u16				mfg;
	int				alignment_time;
	u32				dc_icl_epp;
	u32				current_filtered;
	u32				current_sample_cnt;
	bool				log_current_filtered;
	struct delayed_work		dcin_pon_work;
	bool				is_mfg_google;
	u8				ptmc_id_str[(sizeof(u16) * 2) + 1];
	u32				aicl_delay_ms;
	u32				aicl_icl_ua;
	u32				rtx_csp;
	int				rtx_err;
	int				rtx_reset_cnt;
	int				rtx_ocp_chk_ms;
	int				rtx_total_delay;
	int				rtx_gpio_state;
	u16				rtx_ocp;
	u16				rtx_api_limit;
	u16				rtx_fb_freq_low_limit;
	u16				rtx_hb_freq_low_limit;
	u16				rtx_hb_ping_freq;
	u16				rtx_fod_thrsh;
	u16				ra9530_rtx_plim;
	bool				chg_on_rtx;
	bool				chg_on_cc_rtx;
	bool				is_rtx_mode;
	bool				prop_mode_en;
	bool				negotiation_complete;
	bool				no_fod;
	u32				de_q_value;
	u16				fw_rev;
	u32				wlc_ocp;
	u8				wlc_ovp;
	struct mutex			stats_lock;
	struct p9221_charge_stats	chg_data;
	u32				mitigate_threshold;
	u32				fod_cnt;
	bool				trigger_power_mitigation;
	bool				wait_for_online;
	struct mutex			rtx_lock;
	struct mutex			rtx_gpio_lock;
	bool				rtx_wakelock;
	ktime_t				online_at;
	bool				p9412_gpio_ctl;
	bool				auth_delay;
	struct mutex			auth_lock;
	int 				ll_bpp_cep;
	int				last_disable;
	ktime_t				irq_at;
	int				renego_state;
	struct mutex			renego_lock;
	bool				send_eop;
	wait_queue_head_t		ccreset_wq;
	bool				cc_reset_pending;
	bool				set_auth_icl;
	int				send_txid_cnt;
	bool				sw_ramp_done;
	bool				hpp_hv;
	int				hpp_fod_level;
	int				fod_mode;
	enum p9xxx_chk_rp		check_rp;
	bool				extended_int_recv;
	int				align_delta;
	bool				online_spoof;
	bool				det_status;
	int				det_on_debounce;
	int				det_off_debounce;
	bool				votable_init_done;
	u32				trigger_dd;
	u32				low_neg_pwr_icl;
	int				enable_i2c_debug;
	char				*i2c_txdebug_buf;
	char				*i2c_rxdebug_buf;
	struct mutex			irq_det_lock;
	struct mutex			icl_lock;
	int				fan_last_level;

#if IS_ENABLED(CONFIG_GPIOLIB)
	struct gpio_chip gpio;
#endif

	/* WLC DC when available */
	u32 				wlc_dc_voltage_now;
	u32 				wlc_dc_current_now;
	bool				wlc_dc_enabled;
	u8				wlc_dc_comcap;
	u8				wlc_default_comcap;
	u8				wlc_dd_comcap;
	u8				wlc_disable_comcap;
	bool				chg_mode_off;
	u32				de_hpp_neg_pwr;
	u32				de_epp_neg_pwr;
	u32				de_wait_prop_irq_ms;

	u16				reg_tx_id_addr;
	u16				reg_tx_mfg_code_addr;
	u16				reg_packet_type_addr;
	u16				set_cmd_ccactivate_bit;
	u16				reg_set_pp_buf_addr;
	u16				reg_get_pp_buf_addr;
	u16				reg_set_fod_addr;
	u16				reg_q_factor_addr;
	u16				reg_rf_value_addr;
	u16				reg_csp_addr;
	u16				reg_light_load_addr;
	u16				reg_mot_addr;
	u16				reg_cmfet_addr;
	u16				reg_hivout_cmfet_addr;
	u16				reg_epp_tx_guarpwr_addr;
	u16				reg_freq_limit_addr;
	u16				reg_ask_mod_fet_addr;

	int (*reg_read_n)(struct p9221_charger_data *chgr, u16 reg,
			  void *buf, size_t n);
	int (*reg_read_8)(struct p9221_charger_data *chgr, u16 reg,
			u8 *val);
	int (*reg_read_16)(struct p9221_charger_data *chgr, u16 reg,
			   u16 *val);
	int (*reg_write_n)(struct p9221_charger_data *charger, u16 reg,
			   const void *buf, size_t n);
	int (*reg_write_8)(struct p9221_charger_data *charger, u16 reg,
			   u8 val);
	int (*reg_write_16)(struct p9221_charger_data *charger, u16 reg,
			    u16 val);

	int (*chip_set_data_buf)(struct p9221_charger_data *chgr,
				 const u8 data[], size_t len);
	int (*chip_get_data_buf)(struct p9221_charger_data *chgr,
				 u8 data[], size_t len);
	int (*chip_get_cc_recv_size)(struct p9221_charger_data *chgr,
				     size_t *len);
	int (*chip_set_cc_send_size)(struct p9221_charger_data *chgr,
				     size_t len);
	int (*chip_send_ccreset)(struct p9221_charger_data *chgr);
	int (*chip_send_eop)(struct p9221_charger_data *chgr, u8 reason);
	int (*chip_get_align_x)(struct p9221_charger_data *chgr, u8 *x);
	int (*chip_get_align_y)(struct p9221_charger_data *chgr, u8 *y);

	int (*chip_get_vout)(struct p9221_charger_data *chgr, u32 *mv);
	int (*chip_get_iout)(struct p9221_charger_data *chgr, u32 *ma);
	int (*chip_get_op_freq)(struct p9221_charger_data *chgr, u32 *khz);
	int (*chip_get_ping_freq)(struct p9221_charger_data *chgr, u32 *khz);
	int (*chip_get_op_duty)(struct p9221_charger_data *chgr, u32 *duty);
	int (*chip_get_op_bridge)(struct p9221_charger_data *chgr, u8 *hf);
	int (*chip_get_tx_pwr)(struct p9221_charger_data *chgr, u16 *pwr);
	int (*chip_get_rx_pwr)(struct p9221_charger_data *chgr, u16 *pwr);
	int (*chip_get_vcpout)(struct p9221_charger_data *chgr, u32 *mv);
	int (*chip_set_cmd)(struct p9221_charger_data *chgr, u16 cmd);
	int (*chip_get_rx_ilim)(struct p9221_charger_data *chgr, u32 *ma);
	int (*chip_set_rx_ilim)(struct p9221_charger_data *chgr, u32 ma);
	int (*chip_get_tx_ilim)(struct p9221_charger_data *chgr, u32 *ma);
	int (*chip_set_tx_ilim)(struct p9221_charger_data *chgr, u32 ma);
	int (*chip_get_die_temp)(struct p9221_charger_data *chgr, int *mc);
	int (*chip_get_vout_max)(struct p9221_charger_data *chgr, u32 *mv);
	int (*chip_set_vout_max)(struct p9221_charger_data *chgr, u32 mv);
	int (*chip_get_vrect)(struct p9221_charger_data *chgr, u32 *mv);
	int (*chip_get_sys_mode)(struct p9221_charger_data *chgr, u8 *mode);
	int (*chip_get_tx_epp_guarpwr)(struct p9221_charger_data *chgr, u32 *tx_guarpwr);

	int (*chip_tx_mode_en)(struct p9221_charger_data *chgr, bool en);
	int (*chip_renegotiate_pwr)(struct p9221_charger_data *chrg);
	int (*chip_prop_mode_en)(struct p9221_charger_data *chgr, int req_pwr);
	void (*chip_check_neg_power)(struct p9221_charger_data *chgr);
	int (*chip_send_txid)(struct p9221_charger_data *chgr);
	int (*chip_send_csp_in_txmode)(struct p9221_charger_data *chgr, u8 stat);
	int (*chip_capdiv_en)(struct p9221_charger_data *chgr, u8 mode);
	bool (*chip_is_calibrated)(struct p9221_charger_data *chgr);
	void (*chip_set_cmfet)(struct p9221_charger_data *chgr);
	int (*chip_set_bpp_icl)(struct p9221_charger_data *chgr);
	void (*chip_magsafe_optimized)(struct p9221_charger_data *chgr);
	bool (*chip_is_vout_on)(struct p9221_charger_data *chgr);
	void (*chip_set_ovp)(struct p9221_charger_data *chgr, u8 mode);
};

u8 p9221_crc8(u8 *pdata, size_t nbytes, u8 crc);
bool p9221_is_epp(struct p9221_charger_data *charger);
bool p9xxx_is_capdiv_en(struct p9221_charger_data *charger);
int p9221_wlc_disable(struct p9221_charger_data *charger, int disable, u8 reason);
int p9221_set_auth_dc_icl(struct p9221_charger_data *charger, bool enable);
int p9xxx_sw_ramp_icl(struct p9221_charger_data *charger, const int icl_target);
int p9xxx_gpio_set_value(struct p9221_charger_data *charger, int gpio, int value);
bool is_ping_freq_fixed_at(struct p9221_charger_data *charger, u32 khz);

void p9xxx_gpio_init(struct p9221_charger_data *charger);
extern int p9221_chip_init_funcs(struct p9221_charger_data *charger,
				 u16 chip_id);
extern void p9221_chip_init_params(struct p9221_charger_data *charger,
				   u16 chip_id);
extern void p9221_chip_init_interrupt_bits(struct p9221_charger_data *charger,
					   u16 chip_id);

#define RTX_BEN_DISABLED	0
#define RTX_BEN_ON		1
#define RTX_BEN_ENABLED		2
#define FACTORY_BOOST_ENABLED	9

enum p9382_rtx_state {
	RTX_NOTSUPPORTED = 0,
	RTX_AVAILABLE,
	RTX_ACTIVE,
	RTX_DISABLED,
};

enum p9382_rtx_err {
	RTX_NO_ERROR = 0,
	RTX_BATT_LOW,
	RTX_OVER_TEMP,
	RTX_TX_CONFLICT,
	RTX_HARD_OCP,
	RTX_CHRG_NOT_SUP,
};

#define RTX_BATT_LOW_BIT		BIT(0)
#define RTX_OVER_TEMP_BIT		BIT(1)
#define RTX_TX_CONFLICT_BIT		BIT(2)
#define RTX_HARD_OCP_BIT		BIT(3)
#define RTX_CHRG_NOTSUP_BIT		BIT(4)

enum p9xxx_rtx_gpio_state {
	RTX_WAIT = 0,
	RTX_READY,
	RTX_NOT_SUPP,
	RTX_RETRY,
};

enum p9xxx_renego_state {
	P9XXX_AVAILABLE = 0,
	P9XXX_SEND_DATA,
	P9XXX_ENABLE_PROPMODE,
};

#define P9221_MA_TO_UA(ma)((ma) * 1000)
#define P9221_UA_TO_MA(ua) ((ua) / 1000)
#define P9221_MV_TO_UV(mv) ((mv) * 1000)
#define P9221_UV_TO_MV(uv) ((uv) / 1000)
#define P9221_KHZ_TO_HZ(khz) ((khz) * 1000)
#define P9221_HZ_TO_KHZ(khz) ((khz) / 1000)
#define P9221_C_TO_MILLIC(c) ((c) * 1000)
#define P9221_MILLIC_TO_C(mc) ((mc) / 1000)
#define P9221_MILLIC_TO_DECIC(mc) ((mc) / 100)
#define P9412_MW_TO_HW(mw) (((mw) * 2) / 1000) /* mw -> 0.5 W units */
#define P9412_HW_TO_MW(hw) (((hw) / 2) * 1000) /* 0.5 W units -> mw */
#define RA9530_KHZ_TO_REG(khz) ((60000 / khz) - 1)
#define get_boot_msec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_MSEC)

#define p9xxx_chip_get_tx_id(chgr, id) (chgr->reg_tx_id_addr < 0 ? \
      -ENOTSUPP : chgr->reg_read_n(chgr, chgr->reg_tx_id_addr, id, sizeof(*id)))
#define p9xxx_chip_get_tx_mfg_code(chgr, code) (chgr->reg_tx_mfg_code_addr < 0 ? \
      -ENOTSUPP : chgr->reg_read_n(chgr, chgr->reg_tx_mfg_code_addr, code, sizeof(*code)))
#define p9xxx_chip_set_pp_buf(chgr, data, len) (chgr->reg_set_pp_buf_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_n(chgr, chgr->reg_set_pp_buf_addr, data, len))
#define p9xxx_chip_get_pp_buf(chgr, data, len) (chgr->reg_get_pp_buf_addr == 0 ? \
      -ENOTSUPP : chgr->reg_read_n(chgr, chgr->reg_get_pp_buf_addr, data, len))
#define p9xxx_chip_set_fod_reg(chgr, data, len) (chgr->reg_set_fod_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_n(chgr, chgr->reg_set_fod_addr, data, len))
#define p9xxx_chip_get_fod_reg(chgr, data, len) (chgr->reg_set_fod_addr == 0 ? \
      -ENOTSUPP : chgr->reg_read_n(chgr, chgr->reg_set_fod_addr, data, len))
#define p9xxx_chip_set_q_factor_reg(chgr, data) (chgr->reg_q_factor_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_8(chgr, chgr->reg_q_factor_addr, data))
#define p9xxx_chip_set_resonance_freq_reg(chgr, data) (chgr->reg_rf_value_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_8(chgr, chgr->reg_rf_value_addr, data))
#define p9xxx_chip_set_light_load_reg(chgr, data) (chgr->reg_light_load_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_8(chgr, chgr->reg_light_load_addr, data))
#define p9xxx_chip_set_mot_reg(chgr, data) (chgr->reg_mot_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_8(chgr, chgr->reg_mot_addr, data))
#define p9xxx_chip_set_cmfet_reg(chgr, data) (chgr->reg_cmfet_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_8(chgr, chgr->reg_cmfet_addr, data))
#define p9xxx_chip_set_hivout_cmfet_reg(chgr, data) (chgr->reg_hivout_cmfet_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_8(chgr, chgr->reg_hivout_cmfet_addr, data))
#define p9xxx_chip_set_freq_limit(chgr, data) (chgr->reg_freq_limit_addr == 0 ? \
      -ENOTSUPP : chgr->reg_write_16(chgr, chgr->reg_freq_limit_addr, data))
#define p9xxx_chip_set_ask_mod_fet(chgr, data) ((chgr->reg_ask_mod_fet_addr == 0 || data == 0) ? \
      -ENOTSUPP : chgr->reg_write_8(chgr, chgr->reg_ask_mod_fet_addr, data))
#define logbuffer_prlog(p, fmt, ...)     \
      gbms_logbuffer_prlog(p, LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG, fmt, ##__VA_ARGS__)
#endif /* __P9221_CHARGER_H__ */
