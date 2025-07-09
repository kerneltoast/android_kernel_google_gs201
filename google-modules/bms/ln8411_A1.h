/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for LN8411 Direct charger
 * Based on PCA9468 driver
 */

#ifndef _LN8411_A1_H_
#define _LN8411_A1_H_

#define LN8411_MANUFACTURER             "Cirrus Logic, Inc."
#define LN8411_MODEL_NAME               "LN8411"

#define LN8411_REG_BITS                 8
#define LN8411_VAL_BITS                 8

#define LN8411_DEVICE_ID                0x0

#define LN8411_VBAT_OVP                 0x1
#define LN8411_VBAT_OVP_DIS             BIT(7)
#define LN8411_VBAT_OVP_DG_SET          BIT(6)
#define LN8411_VBAT_OVP_DG_DFLT         0
#define LN8411_VBAT_OVP_DG_80US         LN8411_VBAT_OVP_DG_SET
#define LN8411_VBAT_OVP_MASK            BIT(5)
#define LN8411_VBAT_OVP_SET_MASK        (LN8411_VBAT_OVP_DIS | \
					 LN8411_VBAT_OVP_DG_SET | \
					 LN8411_VBAT_OVP_MASK)
#define LN8411_VBAT_OVP_CFG_MASK        GENMASK(4, 0)
#define LN8411_VBAT_OVP_OFFSET_UV       4450000
#define LN8411_VBAT_OVP_STEP_UV         25000
#define LN8411_VBAT_OVP_MIN_UV          LN8411_VBAT_OVP_OFFSET_UV
#define LN8411_VBAT_OVP_MAX_UV          5225000
#define LN8411_VBAT_OVP_DFLT_UV         4450000

#define LN8411_IBAT_OCP                 0x2
#define LN8411_IBAT_OCP_DIS             BIT(7)
#define LN8411_IBAT_OCP_MASK            BIT(5)
#define LN8411_IBAT_OCP_SET_MASK        (LN8411_IBAT_OCP_DIS | \
					 LN8411_IBAT_OCP_MASK)
#define LN8411_IBAT_OCP_CFG_MASK        GENMASK(3, 0)
#define LN8411_IBAT_OCP_OFFSET_UA       10000000
#define LN8411_IBAT_OCP_STEP_UA         500000
#define LN8411_IBAT_OCP_MIN_UA          LN8411_IBAT_OCP_OFFSET_UA
#define LN8411_IBAT_OCP_MAX_UA          17500000
#define LN8411_IBAT_OCP_DFLT_UA         12000000

#define LN8411_VUSB_OVP                 0x3
#define LN8411_VUSB_OVP_DG_SET          BIT(7)
#define LN8411_VUSB_OVP_MASK            BIT(6)
#define LN8411_VUSB_OVP_SET_MASK        (LN8411_VUSB_OVP_DG_SET | LN8411_VUSB_OVP_MASK)
#define LN8411_VUSB_OVP_DFLT            0xf
#define LN8411_VUSB_OVP_DFLT_UV         6500000
#define LN8411_VUSB_OVP_STEP_UV         1000000
#define LN8411_VUSB_OVP_OFFSET_UV       11000000
#define LN8411_VUSB_OVP_MIN_UV          LN8411_VUSB_OVP_OFFSET_UV
#define LN8411_VUSB_OVP_MAX_UV          25000000
#define LN8411_VUSB_OVP_CFG_MASK        GENMASK(3, 0)

#define LN8411_VWPC_OVP                 0x4
#define LN8411_VWPC_OVP_DG_SET          BIT(7)
#define LN8411_VWPC_OVP_MASK            BIT(6)
#define LN8411_VWPC_OVP_SET_MASK        (LN8411_VWPC_OVP_DG_SET | LN8411_VWPC_OVP_MASK)
#define LN8411_VWPC_OVP_DFLT            0xf
#define LN8411_VWPC_OVP_DFLT_UV         6500000
#define LN8411_VWPC_OVP_STEP_UV         1000000
#define LN8411_VWPC_OVP_OFFSET_UV       11000000
#define LN8411_VWPC_OVP_MIN_UV          LN8411_VWPC_OVP_OFFSET_UV
#define LN8411_VWPC_OVP_MAX_UV          25000000
#define LN8411_VWPC_OVP_CFG_MASK        GENMASK(3, 0)

#define LN8411_IBUS_OCP                 0x5
#define LN8411_IBUS_OCP_DIS             BIT(7)
#define LN8411_IBUS_OCP_MASK            BIT(5)
#define LN8411_IBUS_OCP_SET_MASK        (LN8411_IBUS_OCP_DIS | LN8411_IBUS_OCP_MASK)
#define LN8411_IBUS_OCP_CFG_MASK        GENMASK(3, 0)
#define LN8411_IBUS_OCP_OFFSET_UA       2500000
#define LN8411_IBUS_OCP_STEP_UA         250000
#define LN8411_IBUS_OCP_MIN_UA          LN8411_IBUS_OCP_OFFSET_UA
#define LN8411_IBUS_OCP_MAX_UA          6250000
#define LN8411_IBUS_OCP_DLFT_UA         3500000

#define LN8411_IBUS_UCP                 0x6
#define LN8411_IBUS_UCP_DIS             BIT(7)
#define LN8411_IBUS_UCP_RISE_MASK       BIT(3)
#define LN8411_IBUS_UCP_FALL_MASK       BIT(1)
#define LN8411_IBUS_UCP_SET_MASK        (LN8411_IBUS_UCP_DIS | \
					 LN8411_IBUS_UCP_RISE_MASK | \
					 LN8411_IBUS_UCP_FALL_MASK)

#define LN8411_PMID2OUT_OVP             0x7
#define LN8411_PMID2OUT_OVP_DIS         BIT(7)
#define LN8411_PMID2OUT_OVP_MASK        BIT(4)
#define LN8411_PMID2OUT_OVP_DFLT        0x2
#define LN8411_PMID2OUT_OVP_CFG         GENMASK(2, 0)
#define LN8411_PMID2OUT_OVP_SET_MASK    (LN8411_PMID2OUT_OVP_DIS | \
					 LN8411_PMID2OUT_OVP_MASK | \
					 LN8411_PMID2OUT_OVP_CFG)

#define LN8411_PMID2OUT_UVP             0x8
#define LN8411_PMID2OUT_UVP_DIS         BIT(7)
#define LN8411_PMID2OUT_UVP_MASK        BIT(4)
#define LN8411_PMID2OUT_UVP_DFLT        0x1
#define LN8411_PMID2OUT_UVP_CFG         GENMASK(2, 0)
#define LN8411_PMID2OUT_UVP_SET_MASK    (LN8411_PMID2OUT_UVP_DIS | \
					 LN8411_PMID2OUT_UVP_MASK | \
					 LN8411_PMID2OUT_UVP_CFG)

#define LN8411_CONVERTER                0x9
#define LN8411_POR_FLAG                 BIT(7)
#define LN8411_ACRB_WPC_STAT            BIT(6)
#define LN8411_ACRB_USB_STAT            BIT(5)
#define LN8411_VBUS_ERRORLO_STAT        BIT(4)
#define LN8411_VBUS_ERRORHI_STAT        BIT(3)
#define LN8411_QB_ON_STAT               BIT(2)
#define LN8411_CP_SWITCHING_STAT        BIT(1)
#define LN8411_PIN_DIAG_FAIL_FLAG       BIT(0)

#define LN8411_CTRL1                    0xa
#define LN8411_CP_EN                    BIT(7)
#define LN8411_QB_EN                    BIT(6)
#define LN8411_ACDRV_MANUAL_EN          BIT(5)
#define LN8411_WPCGATE_EN               BIT(4)
#define LN8411_OVPGATE_EN               BIT(3)
#define LN8411_VBUS_PD_EN               BIT(2)
#define LN8411_VWPC_PD_EN               BIT(1)
#define LN8411_VUSB_PD_EN               BIT(0)
#define LN8411_PD_EN_MASK		GENMASK(2, 0)

#define LN8411_CTRL2                    0xb
#define LN8411_FSW_SET_MASK             GENMASK(7, 3)
#define LN8411_FSW_DFLT                 0x6
#define LN8411_FSW_DFLT_HZ             390000
#define LN8411_FSW_SHIFT                4
#define LN8411_FREQ_SHIFT               BIT(1)
#define LN8411_NUM_FREQ_VAL             32

#define LN8411_CTRL3                    0xc
#define LN8411_VBUS_INRANGE_DET_DIS     BIT(7)
#define LN8411_WD_TIMEOUT_CFG_MASK      GENMASK(2, 0)
#define LN8411_WD_TIMEOUT_DIS           0x0
#define LN8411_NUM_WD_MS_VAL            8

#define LN8411_CTRL4                    0xd
#define LN8411_SYNC_FUNCTION_EN         BIT(7)
#define LN8411_SYNC_MASTER_EN           BIT(6)
#define LN8411_SYNC_MASK		(LN8411_SYNC_FUNCTION_EN | LN8411_SYNC_MASTER_EN)
#define LN8411_VBUS_OVP_SET             BIT(5)
#define LN8411_SET_IBAT_SNS_RES         BIT(4)
#define LN8411_TSBAT_EN_PIN		BIT(3)
#define LN8411_MODE_MASK                GENMASK(2, 0)

#define LN8411_CTRL5                    0xe
#define LN8411_OVPGATE_STAT		BIT(7)
#define LN8411_WPCGATE_STAT		BIT(6)

#define LN8411_INT_STAT                 0xf
#define LN8411_VOUT_INSERT_STAT         BIT(3)
#define LN8411_VWPC_INSERT_STAT         BIT(1)
#define LN8411_VUSB_INSERT_STAT         BIT(0)

#define LN8411_INT_FLAG                 0x10
#define LN8411_VOUT_INSERT_FLAG         BIT(3)
#define LN8411_VWPC_INSERT_FLAG         BIT(1)
#define LN8411_VUSB_INSERT_FLAG         BIT(0)

#define LN8411_INT_MASK                 0x11
#define LN8411_VOUT_INSERT_MASK         BIT(3)
#define LN8411_VWPC_INSERT_MASK         BIT(1)
#define LN8411_VUSB_INSERT_MASK         BIT(0)

#define LN8411_FLT_FLAG                 0x12
#define LN8411_TSBAT_FLT_FLAG           BIT(7)
#define LN8411_TSHUT_FLAG               BIT(6)
#define LN8411_WD_TIMEOUT_FLAG          BIT(4)
#define LN8411_CONV_OCP_FLAG            BIT(3)
#define LN8411_VBUS_OVP_FLAG            BIT(1)
#define LN8411_VOUT_OVP_FLAG            BIT(0)

#define LN8411_FLT_MASK                 0x13
#define LN8411_TSBAT_FLT_MASK           BIT(7)
#define LN8411_TSHUT_MASK               BIT(6)
#define LN8411_WD_TIMEOUT_MASK          BIT(4)
#define LN8411_CONV_OCP_MASK            BIT(3)
#define LN8411_VBUS_OVP_MASK            BIT(1)
#define LN8411_VOUT_OVP_MASK            BIT(0)

#define LN8411_ADC_CTRL                 0x14
#define LN8411_ADC_EN                   BIT(7)
#define LN8411_ADC_RATE                 BIT(6)
#define LN8411_ADC_DONE_MASK            BIT(3)
#define LN8411_IBUS_ADC_DIS             BIT(0)

#define LN8411_ADC_FN_DISABLE1          0x15
#define LN8411_VBUS_ADC_DIS             BIT(7)
#define LN8411_VUSB_ADC_DIS             BIT(6)
#define LN8411_VWPC_ADC_DIS             BIT(5)
#define LN8411_VOUT_ADC_DIS             BIT(4)
#define LN8411_VBAT_ADC_DIS             BIT(3)
#define LN8411_IBAT_ADC_DIS             BIT(2)
#define LN8411_TSBAT_ADC_DIS            BIT(1)
#define LN8411_TDIE_ADC_DIS             BIT(0)

#define LN8411_IBUS_ADC1                0x16
#define LN8411_IBUS_ADC_STEP_UA         1563
#define LN8411_IBUS_ADC0                0x17

#define LN8411_VBUS_ADC1                0x18
#define LN8411_VBUS_ADC_STEP_UV         6250
#define LN8411_VBUS_ADC0                0x19

#define LN8411_VUSB_ADC1                0x1a
#define LN8411_VUSB_ADC_STEP_UV         6250
#define LN8411_VUSB_ADC0                0x1b

#define LN8411_VWPC_ADC1                0x1c
#define LN8411_VWPC_ADC_STEP_UV         6250
#define LN8411_VWPC_ADC0                0x1d

#define LN8411_VOUT_ADC1                0x1e
#define LN8411_VOUT_ADC_STEP_UV         1250
#define LN8411_VOUT_ADC0                0x1f

#define LN8411_VBAT_ADC1                0x20
#define LN8411_VBAT_ADC_STEP_UV         1250
#define LN8411_VBAT_ADC0                0x21

#define LN8411_IBAT_ADC1                0x22
#define LN8411_IBAT_ADC_STEP_UA         6250
#define LN8411_IBAT_ADC0                0x23

#define LN8411_TSBAT_ADC1               0x24
#define LN8411_TSBAT_ADC0               0x25

#define LN8411_TDIE_ADC0                0x26
#define LN8411_TDIE_STEP_DECIC          5
#define LN8411_TDIE_ADC1                0x27

#define LN8411_COMP_FLAG0               0x2a
#define LN8411_SYNC_CLK_MISSING_FLAG    BIT(7)
#define LN8411_IBUS_UCP_RISE_FLAG       BIT(6)
#define LN8411_IBUS_UCP_FALL_FLAG       BIT(5)
#define LN8411_IBUS_OCP_FLAG            BIT(4)
#define LN8411_VWPC_OVP_FLAG            BIT(3)
#define LN8411_VUSB_OVP_FLAG            BIT(2)
#define LN8411_IBAT_OCP_FLAG            BIT(1)
#define LN8411_VBAT_OVP_FLAG            BIT(0)

#define LN8411_COMP_STAT1               0x2b
#define LN8411_CFLY_SHORTED_SW_FLAG     BIT(7)
#define LN8411_IBUS_OC_ALARM_FLAG       BIT(6)
#define LN8411_IBAT_OC_ALARM_FLAG       BIT(5)
#define LN8411_VBAT_OV_ALARM_FLAG       BIT(4)

#define LN8411_COMP_FLAG1               0x2c
#define LN8411_REV_IBUS_FLAG            BIT(7)
#define LN8411_MODE_ACTIVE_FLAG         BIT(6)
#define LN8411_MODE_STANDBY_FLAG        BIT(5)
#define LN8411_MODE_SHUTDOWN_FLAG       BIT(4)
#define LN8411_TSBAT_ALARM_FLAG         BIT(3)
#define LN8411_ADC_DONE_FLAG            BIT(2)
#define LN8411_PMID2OUT_UVP_FLAG        BIT(1)
#define LN8411_PMID2OUT_OVP_FLAG        BIT(0)

#define LN8411_ADC_CTRL2		0x2d

#define LN8411_CFG_1			0x31
#define LN8411_DEVICE_MODE		BIT(3)
#define LN8411_INT_MASK_2		0x32
#define LN8411_PAUSE_INT_UPDATE		BIT(7)
#define LN8411_CLEAR_INT		BIT(6)


#define LN8411_LION_CTRL		0x40

#define LN8411_REG_49			0x49
#define LN8411_REVERT_LSNS		BIT(7)

#define LN8411_TEST_MODE_CTRL		0x56
#define LN8411_SOFT_RESET_REQ		BIT(0)

#define LN8411_CFG_10			0x5A

#define LN8411_BC_STS_C			0x62
#define LN8411_CHIP_REV_MASK		GENMASK(7,4)
#define LN8411_CHIP_REV_SHIFT		4

#define LN8411_ALARM_CTRL		0x74
#define VBAT_ALARM_CFG			BIT(7)

#define LN8411_ADC_CFG_2                0x76
#define LN8411_CLEAR_LATCHED_STS	BIT(7)
#define LN8411_PAUSE_ADC_UPDATES        BIT(5)

#define LN8411_SC_DITHER_CTRL		0x78

#define LN8411_LION_COMP_CTRL_1		0x79
#define LN8411_PMID_SWITCH_OK_DIS	BIT(3)

#define LN8411_LION_COMP_CTRL_2		0x7a
#define LN8411_VWPC_UVP_DIS		BIT(3)
#define LN8411_VUSB_UVP_DIS		BIT(2)
#define LN8411_VBUS_UVP_DIS		(LN8411_VWPC_UVP_DIS | \
					 LN8411_VUSB_UVP_DIS)

#define LN8411_LION_COMP_CTRL_4		0x7c

#define LN8411_OVPGATE_CTRL_0		0x89
#define LN8411_OVPFETDR_V_CFG		BIT(6)

#define LN8411_SYS_STS                  0x98
#define LN8411_PMID_SWITCH_OK_STS       BIT(7)
#define LN8411_INFET_OK_STS		BIT(6)
#define LN8411_SWITCHING41_ACTIVE_STS	BIT(5)
#define LN8411_SWITCHING31_ACTIVE_STS   BIT(4)
#define LN8411_SWITCHING21_ACTIVE_STS   BIT(3)
#define LN8411_BYPASS_ACTIVE_STS        BIT(2)
#define LN8411_ACTIVE_STS               (LN8411_SWITCHING41_ACTIVE_STS | \
					 LN8411_SWITCHING31_ACTIVE_STS | \
					 LN8411_SWITCHING21_ACTIVE_STS | \
					 LN8411_BYPASS_ACTIVE_STS)
#define LN8411_STANDBY_STS              BIT(1)
#define LN8411_SHUTDOWN_STS             BIT(0)

#define LN8411_SAFETY_STS		0x99
#define LN8411_TEMP_FAULT_DETECTED	BIT(7)
#define LN8411_TEMP_MAX_STS		BIT(6)
#define LN8411_TSBAT_ALARM_STS		BIT(5)
#define LN8411_TSBAT_SHUTDOWN_STS	BIT(3)
#define LN8411_REV_IBUS_STS		BIT(1)
#define LN8411_REV_IBUS_LATCHED		BIT(0)

#define LN8411_FAULT1_STS		0x9a
#define LN8411_WATCHDOG_TIMER_STS	BIT(7)
#define LN8411_VOLT_FAULT_DETECTED	BIT(6)
#define LN8411_V_SHORT_STS		BIT(5)
#define LN8411_VBUS_OV_FIXED_STS	BIT(4)
#define LN8411_PMID2OUT_OV_STS		BIT(3)
#define LN8411_PMID2OUT_UV_STS		BIT(2)
#define LN8411_SYS_LPM_STS		BIT(1)
#define LN8411_INFET_OFF_DET_STS	BIT(0)

#define LN8411_FAULT2_STS		0x9b
#define LN8411_VOUT_OV_STS		BIT(4)
#define LN8411_VWPC_OV_STS		BIT(3)
#define LN8411_VWPC_UV_STS		BIT(2)
#define LN8411_VUSB_OV_STS		BIT(1)
#define LN8411_VUSB_UV_STS		BIT(0)

#define LN8411_FAULT3_STS		0x9c
#define LN8411_IBUS_OC_DETECTED		BIT(7)
#define LN8411_IBAT_OC_DETECTED		BIT(6)
#define LN8411_VBAT_OV_STS		BIT(5)
#define LN8411_IBUS_ALARM_STS		BIT(4)
#define LN8411_IBAT_ALARM_STS		BIT(3)
#define LN8411_VBAT_ALARM_STS		BIT(2)
#define LN8411_CFLY_SHORT_DETECTED	BIT(1)
#define LN8411_SC_OUT_SHORT_DETECTED	BIT(0)

#define LN8411_OTG_MIN_UV		3500000
#define LN8411_OTG_MIN_1TO2_MIN_UV	(LN8411_OTG_MIN_UV * 2)
#define LN8411_OTG_MIN_1TO4_MIN_UV	(LN8411_OTG_MIN_UV * 4)

#endif /* _LN8411_A1_H_ */