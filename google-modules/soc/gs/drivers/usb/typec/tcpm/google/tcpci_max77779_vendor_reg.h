// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2019 Google LLC
 *
 */

#ifndef __TCPCI_MAX77759_VENDOR_REG_H
#define __TCPCI_MAX77759_VENDOR_REG_H

#define CHG_CNFG_00                              0xB9
#define MODE_MASK                               GENMASK(2, 0)
#define MODE_BUCK_ON                            0x4
#define MODE_BOOST_ON                           0xA
#define MODE_OFF                                0x0

#define VNDR_ALRT                               BIT(15)
#define VNDR_ALRT_H                             BIT(7)

/* TCPC ALERT bits */
#define UNMASK_VNDR_ALRT                        BIT(15)
#define UNMASK_RX_BUFF_OVRFL                    BIT(10)
#define UNMASK_FAULT_STAT                       BIT(9)

#define TCPC_CC_STATE_WTRSEL			3

#define TCPC_EXTENDED_STATUS                    0x20
#define TCPC_EXTENDED_STATUS_VSAFE0V            BIT(0)

#define TCPC_VENDOR_ALERT                       0x80
#define TCPC_VENDOR_ALERT2			0x81
#define TCPC_VENDOR_ALERT_MASK			0x82

#define TCPC_VENDOR_ALERT_MASK2			0x83
#define MSK_FLASH_ADCINT			BIT(1)

#define VENDOR_CC_STATUS1			0x84
#define VENDOR_CC_STATUS2			0x85
#define CC1_VUFP_RD0P5				BIT(1)
#define CC2_VUFP_RD0P5				BIT(5)
#define TCPC_VENDOR_SARADC_STATUS		0x89

#define TCPC_VENDOR_VCON_CTRL			0x8b
#define VCNILIM_MASK				GENMASK(2, 0)
#define VCNILIM_300_MA				0x2

#define TCPC_VENDOR_CC_CTRL1			0x8c
#define CCCONNDRY				BIT(7)
#define RDOPENDIS				BIT(6)
#define CCCOMPEN				BIT(5)
#define CCSNKEXITEN				BIT(4)
#define CCLPDRPCYCLE_MASK			GENMASK(3, 2)
#define CCDRPPHASE				GENMASK(1, 0)

#define TCPC_VENDOR_CC_CTRL2			0x8d
#define CCOVPDIS				BIT(7)
#define CCULPSRCSEL				GENMASK(6, 5)
#define CCULPSRC_025UA				0
#define CCULPSRC_050UA				(1 << 5)
#define CCULPSRC_1UA				(2 << 5)
#define CCULPSRC_5UA				(3 << 5)
#define CCLPMODESEL_MASK			GENMASK(4, 3)
#define LOW_POWER_MODE_DISABLE			0
#define ULTRA_LOW_POWER_MODE			(1 << 3)
#define AUTO_ULTRA_LOW_POWER_MODE		(3 << 3)
#define CCRPCTRL_MASK				GENMASK(2, 0)
#define TCPCI_CONTROL				0
#define UA_1_SRC				1
#define UA_5_SRC				2
#define UA_80_SRC				3
#define UA_180_SRC				4
#define UA_330_SRC				5
#define RFU					6
#define	OPEN					7

#define TCPC_VENDOR_CC_CTRL3			0x8e
#define CCWTRDEB_MASK				GENMASK(7, 6)
#define CCWTRDEB_SHIFT				6
#define CCWTRDEB_1MS				1
#define CCWTRSEL_MASK				GENMASK(5, 3)
#define CCWTRSEL_SHIFT				3
#define CCWTRSEL_1V				0x4
#define CCWTRSEL_0_6V				0x2
#define CCLADDERDIS				BIT(2)
#define SBU_DET_EN				BIT(1)
#define WTRCYCLE_MASK				BIT(0)
#define WTRCYCLE_SHIFT				0
#define WTRCYCLE_2_4_S				0
#define WTRCYCLE_4_8_S				1

#define TCPC_VENDOR_ADC_CTRL1			0x91
#define ADCINSEL_MASK				GENMASK(7, 5)
#define ADC_CHANNEL_OFFSET			5
#define ADCEN					BIT(0)

#define TCPC_VENDOR_EXTBST_CTRL			0x92
#define EXT_BST_EN				BIT(0)

#define TCPC_VENDOR_USBSW_CTRL			0x93
#define USBSW_CONNECT				0x9
#define USBSW_DISCONNECT			0

#define TCPC_VENDOR_SBUSW_CTRL			0x94
#define SBU1SW_MAP				GENMASK(2, 0)
#define FLADC_SELECT_SBU1			0x4
#define SBU2SW_MAP				GENMASK(5, 3)
#define FLADC_SELECT_SBU2			(4 << 3)
#define SBUSW_SERIAL_UART			0x12
#define SBUSW_PATH_1				0x9

#define TCPC_VENDOR_SBU_CTRL1			0x96
#define SBUOVPDIS				BIT(7)
#define SBUULPSRCSEL				GENMASK(6, 5)
#define SBUULPSRC_025UA				0
#define SBUULPSRC_050UA				(1 << 5)
#define SBUULPSRC_1UA				(2 << 5)
#define SBUCOMPEN				BIT(4)
#define SBUCLAMPDIS				BIT(3)
#define SBURPCTRL_ULP_EN			BIT(2)

#endif /* __TCPCI_MAX77759_VENDOR_REG_H */
