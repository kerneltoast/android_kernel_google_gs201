/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2022 Samsung Electronics Co., Ltd.
 */
#ifndef _UFS_CAL_TABLE_
#define _UFS_CAL_TABLE_

#define UFS_CAL_VER 0
#include "ufs-cal-if.h"

struct ufs_cal_phy_cfg {
	u32 mib;
	u32 addr;
	u32 val;
	u32 flg;
	u32 lyr;
	u8 board;
};

enum {
	PHY_CFG_NONE = 0,
	PHY_PCS_COMN,
	PHY_PCS_RXTX,
	PHY_PMA_COMN,
	PHY_PMA_TRSV,
	PHY_PLL_WAIT,
	PHY_CDR_WAIT,
	PHY_CDR_AFC_WAIT,
	UNIPRO_STD_MIB,
	UNIPRO_DBG_MIB,
	UNIPRO_DBG_APB,

	PHY_PCS_RX,
	PHY_PCS_TX,
	PHY_PCS_RX_PRD,
	PHY_PCS_TX_PRD,
	UNIPRO_DBG_PRD,
	PHY_PMA_TRSV_LANE1_SQ_OFF,
	PHY_PMA_TRSV_SQ,
	COMMON_WAIT,

	PHY_PCS_RX_LR_PRD,
	PHY_PCS_TX_LR_PRD,
	PHY_PCS_RX_PRD_ROUND_OFF,
	PHY_PCS_TX_PRD_ROUND_OFF,
	UNIPRO_ADAPT_LENGTH,
	PHY_EMB_CDR_WAIT,
	PHY_EMB_CAL_WAIT,

	HCI_AH8_THIBERN,
	HCI_AH8_REFCLKGATINGTING,
	HCI_AH8_ACTIVE_LANE,
	HCI_AH8_CMN,
	HCI_AH8_TRSV,
	HCI_STD,
};

enum {
	PMD_PWM_G1 = 0,
	PMD_PWM_G2,
	PMD_PWM_G3,
	PMD_PWM_G4,
	PMD_PWM_G5,
	PMD_PWM,

	PMD_HS_G1,
	PMD_HS_G2,
	PMD_HS_G3,
	PMD_HS_G4,
	PMD_HS,

	PMD_ALL,
};

#undef USE_38_4_MHZ
#define USE_38_4_MHZ	 /* 38.4MHz */

static struct ufs_cal_phy_cfg init_cfg_evt0[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x44, 0x00, PMD_ALL, UNIPRO_DBG_PRD, BRD_ALL},

	{0x200, 0x2800, 0x40, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#ifdef USE_38_4_MHZ /*38.4MHz*/
	{0x202, 0x2808, 0x22, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#else/*26MHz*/
	{0x202, 0x2808, 0x12, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#endif
	{0x12, 0x2048, 0x00, PMD_ALL, PHY_PCS_RX_PRD_ROUND_OFF, BRD_ALL},
	{0xAA, 0x22A8, 0x00, PMD_ALL, PHY_PCS_TX_PRD_ROUND_OFF, BRD_ALL},
	{0xA9, 0x22A4, 0x02, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0xAB, 0x22AC, 0x00, PMD_ALL, PHY_PCS_TX_LR_PRD, BRD_ALL},
	{0x11, 0x2044, 0x00, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x1B, 0x206C, 0x00, PMD_ALL, PHY_PCS_RX_LR_PRD, BRD_ALL},
	{0x2F, 0x20BC, 0x79, PMD_ALL, PHY_PCS_RX, BRD_ALL},

	{0x84, 0x2210, 0x01, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x04, 0x2010, 0x01, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0x25, 0x2094, 0xF6, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x7F, 0x21FC, 0x00, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0x200, 0x2800, 0x0, PMD_ALL, PHY_PCS_COMN, BRD_ALL},

	{0x155E, 0x3178, 0x0, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x3000, 0x5000, 0x0, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x3001, 0x5004, 0x1, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x4021, 0x6084, 0x1, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x4020, 0x6080, 0x1, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},

	{0x0000, 0x140, 0x08, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x014, 0x19, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x02C, 0x44, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x030, 0xC4, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x034, 0xC3, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x03C, 0x88, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x058, 0x1A, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x064, 0x04, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x150, 0x88, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x19C, 0x4C, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x1A0, 0x4C, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x804, 0x44, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x808, 0x44, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x80C, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x810, 0x18, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x814, 0xC0, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x81C, 0x1C, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xBB0, 0x8C, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9F0, 0xD0, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xA20, 0xFA, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA24, 0x60, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x8D0, 0x30, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8E4, 0x05, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8F4, 0x05, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x934, 0x1A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x938, 0x12, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x93C, 0x5E, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x964, 0x2A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x980, 0x54, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x998, 0x54, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x9CC, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9D0, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xAAC, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAB0, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x140, 0x0C, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x140, 0x00, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0xC74, 0x01, PMD_ALL, PHY_EMB_CAL_WAIT, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg init_cfg_evt1[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x44, 0x00, PMD_ALL, UNIPRO_DBG_PRD, BRD_ALL},

	{0x200, 0x2800, 0x40, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#ifdef USE_38_4_MHZ /*38.4MHz*/
	{0x202, 0x2808, 0x22, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#else/*26MHz*/
	{0x202, 0x2808, 0x12, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#endif
	{0x12, 0x2048, 0x00, PMD_ALL, PHY_PCS_RX_PRD_ROUND_OFF, BRD_ALL},
	{0xAA, 0x22A8, 0x00, PMD_ALL, PHY_PCS_TX_PRD_ROUND_OFF, BRD_ALL},
	{0xA9, 0x22A4, 0x02, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0xAB, 0x22AC, 0x00, PMD_ALL, PHY_PCS_TX_LR_PRD, BRD_ALL},
	{0x11, 0x2044, 0x00, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x1B, 0x206C, 0x00, PMD_ALL, PHY_PCS_RX_LR_PRD, BRD_ALL},
	{0x2F, 0x20BC, 0x79, PMD_ALL, PHY_PCS_RX, BRD_ALL},

	{0x84, 0x2210, 0x01, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x04, 0x2010, 0x01, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0x25, 0x2094, 0xF6, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x7F, 0x21FC, 0x00, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0x200, 0x2800, 0x0, PMD_ALL, PHY_PCS_COMN, BRD_ALL},

	{0x155E, 0x3178, 0x0, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x3000, 0x5000, 0x0, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x3001, 0x5004, 0x1, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x4021, 0x6084, 0x1, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x4020, 0x6080, 0x1, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},

	{0x0000, 0x140, 0x08, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x014, 0x19, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x02C, 0x44, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x030, 0xC4, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x034, 0xC3, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x03C, 0x88, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x058, 0x1A, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x064, 0x04, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x150, 0x88, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x19C, 0x4C, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x1A0, 0x4C, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x804, 0x44, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x808, 0x44, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x80C, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x810, 0x18, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x814, 0xC0, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x81C, 0x1C, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xBB0, 0x8C, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9F0, 0xD0, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xA20, 0xFA, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA24, 0x60, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x8D0, 0x30, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8E4, 0x05, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8F4, 0x05, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x934, 0x1A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x938, 0x12, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x93C, 0x5E, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x964, 0x2A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x980, 0x54, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x998, 0x54, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x9CC, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9D0, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xAAC, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAB0, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x140, 0x0C, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x140, 0x00, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0xC74, 0x01, PMD_ALL, PHY_EMB_CAL_WAIT, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_init_cfg_evt0[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x15D2, 0x3348, 0x0, PMD_ALL, UNIPRO_ADAPT_LENGTH, BRD_ALL},
	{0x15D3, 0x334C, 0x0, PMD_ALL, UNIPRO_ADAPT_LENGTH, BRD_ALL},
	{0x9529, 0x38A4, 0x01, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},
	{0x15A4, 0x3290, 0x3E8, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x9529, 0x38A4, 0x00, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_init_cfg_evt1[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x15D2, 0x3348, 0x0, PMD_ALL, UNIPRO_ADAPT_LENGTH, BRD_ALL},
	{0x15D3, 0x334C, 0x0, PMD_ALL, UNIPRO_ADAPT_LENGTH, BRD_ALL},
	{0x9529, 0x38A4, 0x01, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},
	{0x15A4, 0x3290, 0x3E8, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x9529, 0x38A4, 0x00, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg calib_of_pwm[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x2041, 0x4104, 8064, PMD_PWM, UNIPRO_STD_MIB, BRD_ALL},
	{0x2042, 0x4108, 28224, PMD_PWM, UNIPRO_STD_MIB, BRD_ALL},
	{0x2043, 0x410C, 20160, PMD_PWM, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B0, 0x32C0, 12000, PMD_PWM, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B1, 0x32C4, 32000, PMD_PWM, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B2, 0x32C8, 16000, PMD_PWM, UNIPRO_STD_MIB, BRD_ALL},

	{0x0000, 0x7888, 8064, PMD_PWM, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x788C, 28224, PMD_PWM, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x7890, 20160, PMD_PWM, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78B8, 12000, PMD_PWM, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78BC, 32000, PMD_PWM, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78C0, 16000, PMD_PWM, UNIPRO_DBG_APB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_calib_of_pwm[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg calib_of_hs_rate_a[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x15D4, 0x3350, 0x1, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},

	{0x2041, 0x4104, 8064, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x2042, 0x4108, 28224, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x2043, 0x410C, 20160, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B0, 0x32C0, 12000, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B1, 0x32C4, 32000, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B2, 0x32C8, 16000, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},

	{0x0000, 0x7888, 8064, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x788C, 28224, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x7890, 20160, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78B8, 12000, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78BC, 32000, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78C0, 16000, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_calib_of_hs_rate_a[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg calib_of_hs_rate_b[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x15D4, 0x3350, 0x1, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},

	{0x2041, 0x4104, 8064, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x2042, 0x4108, 28224, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x2043, 0x410C, 20160, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B0, 0x32C0, 12000, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B1, 0x32C4, 32000, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},
	{0x15B2, 0x32C8, 16000, PMD_HS, UNIPRO_STD_MIB, BRD_ALL},

	{0x0000, 0x7888, 8064, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x788C, 28224, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x7890, 20160, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78B8, 12000, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78BC, 32000, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},
	{0x0000, 0x78C0, 16000, PMD_HS, UNIPRO_DBG_APB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_calib_of_hs_rate_b[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_ah8_cfg_evt0[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x1604, 0x00, PMD_HS, HCI_AH8_THIBERN, BRD_ALL},
	{0x0000, 0x1608, 0x00, PMD_HS, HCI_AH8_REFCLKGATINGTING, BRD_ALL},
	{0x0000, 0x161C, 0x00, PMD_HS, HCI_AH8_ACTIVE_LANE, BRD_ALL},
	{0x0000, 0x1624, 0x35B60, PMD_HS, HCI_AH8_CMN, BRD_ALL},

	{0x0000, 0x1620, 0x120013, PMD_HS, HCI_AH8_CMN, BRD_ALL},

	{0x0000, 0x1710, 0x000009F4, PMD_HS, HCI_AH8_TRSV, BRD_ALL},
	{0x0000, 0x1750, 0x00000008, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1714, 0x00000A00, PMD_HS, HCI_AH8_TRSV, BRD_ALL},
	{0x0000, 0x1754, 0x0000003A, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1718, 0x00000000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1758, 0x00000051, PMD_HS, HCI_AH8_CMN, BRD_ALL},

	{0x0000, 0x1790, 0x00000000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x17D0, 0x00000011, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1794, 0x401E0000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x17D4, 0x00000000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1798, 0x000009F4, PMD_HS, HCI_AH8_TRSV, BRD_ALL},
	{0x0000, 0x17D8, 0x00000000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x179C, 0x00000A00, PMD_HS, HCI_AH8_TRSV, BRD_ALL},
	{0x0000, 0x17DC, 0x00000030, PMD_HS, HCI_AH8_CMN, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_ah8_cfg_evt1[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x1604, 0x00, PMD_HS, HCI_AH8_THIBERN, BRD_ALL},
	{0x0000, 0x1608, 0x00, PMD_HS, HCI_AH8_REFCLKGATINGTING, BRD_ALL},
	{0x0000, 0x161C, 0x00, PMD_HS, HCI_AH8_ACTIVE_LANE, BRD_ALL},
	{0x0000, 0x1624, 0x35B60, PMD_HS, HCI_AH8_CMN, BRD_ALL},

	{0x0000, 0x1620, 0x120013, PMD_HS, HCI_AH8_CMN, BRD_ALL},

	{0x0000, 0x1710, 0x000009F4, PMD_HS, HCI_AH8_TRSV, BRD_ALL},
	{0x0000, 0x1750, 0x00000008, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1714, 0x00000A00, PMD_HS, HCI_AH8_TRSV, BRD_ALL},
	{0x0000, 0x1754, 0x0000003A, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1718, 0x00000000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1758, 0x00000051, PMD_HS, HCI_AH8_CMN, BRD_ALL},

	{0x0000, 0x1790, 0x00000000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x17D0, 0x00000011, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1794, 0x401E0000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x17D4, 0x00000000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x1798, 0x000009F4, PMD_HS, HCI_AH8_TRSV, BRD_ALL},
	{0x0000, 0x17D8, 0x00000000, PMD_HS, HCI_AH8_CMN, BRD_ALL},
	{0x0000, 0x179C, 0x00000A00, PMD_HS, HCI_AH8_TRSV, BRD_ALL},
	{0x0000, 0x17DC, 0x00000030, PMD_HS, HCI_AH8_CMN, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg lane1_sq_off[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x9F4, 0x08, PMD_ALL, PHY_PMA_TRSV_LANE1_SQ_OFF, BRD_ALL},
	{0x0000, 0xA00, 0x3A, PMD_ALL, PHY_PMA_TRSV_LANE1_SQ_OFF, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_h8_enter[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x9F4, 0x08, PMD_ALL, PHY_PMA_TRSV_SQ, BRD_ALL},
	{0x0000, 0xA00, 0x3A, PMD_ALL, PHY_PMA_TRSV_SQ, BRD_ALL},
	{0x0000, 0x000, 0x51, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0xB64, 0x30, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xB64, 0x33, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg pre_h8_exit[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x000, 0x11, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x000, 0x0A, PMD_ALL, COMMON_WAIT, BRD_ALL},
	{0x0000, 0x9F4, 0x00, PMD_ALL, PHY_PMA_TRSV_SQ, BRD_ALL},
	{0x0000, 0xA00, 0x30, PMD_ALL, PHY_PMA_TRSV_SQ, BRD_ALL},

	{0x0000, 0xB64, 0x32, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xB64, 0x22, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg loopback_init[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xB44, 0xA3, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA24, 0x24, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x844, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA14, 0x58, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg loopback_set_1[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xB44, 0xAB, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg loopback_set_2[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xA14, 0x70, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x920, 0x1C, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg eom_prepare[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x870, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xB8C, 0xE3, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8C8, 0x33, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x91C, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xB90, 0x14, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x870, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg init_cfg_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_init_cfg_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg calib_of_pwm_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_calib_of_pwm_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg calib_of_hs_rate_a_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_calib_of_hs_rate_a_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg calib_of_hs_rate_b_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_calib_of_hs_rate_b_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg lane1_sq_off_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_h8_enter_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg pre_h8_exit_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg loopback_init_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg loopback_set_1_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg loopback_set_2_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};
#endif	/* _UFS_CAL_TABLE_ */
