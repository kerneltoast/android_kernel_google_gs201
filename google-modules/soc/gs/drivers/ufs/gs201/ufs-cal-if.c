/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * UFS Host Controller driver for Exynos specific extensions
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Kiwoong <kwmad.kim@samsung.com>
 */

#include <linux/io.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include "ufs-vs-mmio.h"
#include "ufs-cal-if.h"

/*
 * CAL table, project specifics
 *
 * This is supposed to be in here, i.e.
 * right before definitions for version check.
 *
 * DO NOT MOVE THIS TO ANYWHERE RANDOMLY !!!
 */
#include "ufs-cal.h"

/*
 * UFS CAL just requires some things that don't have an impact on
 * external components, such as macros, not functions.
 * The requirement list is below: readl, writel, udelay
 */

#define NUM_OF_UFS_HOST	2

enum {
	PA_HS_MODE_A	= 1,
	PA_HS_MODE_B	= 2,
};

enum {
	FAST_MODE	= 1,
	SLOW_MODE	= 2,
	FASTAUTO_MODE	= 4,
	SLOWAUTO_MODE	= 5,
	UNCHANGED	= 7,
};

enum {
	TX_LANE_0 = 0,
	RX_LANE_0 = 4,
};

#define TX_LINE_RESET_TIME		3200
#define RX_LINE_RESET_DETECT_TIME	1000

#define DELAY_PERIOD_IN_US		40		/* 40us */
#define TIMEOUT_IN_US			(40 * 1000)	/* 40ms */

#define UNIP_DL_ERROR_IRQ_MASK		0x4844	/* shadow of DL error */
#define PA_ERROR_IND_RECEIVED		BIT(15)

#define PHY_PMA_LANE_OFFSET		0x800
#define PHY_PMA_COMN_ADDR(reg)			(reg)
#define PHY_PMA_TRSV_ADDR(reg, lane)	((reg) + (PHY_PMA_LANE_OFFSET * (lane)))

#define UNIP_COMP_AXI_AUX_FIELD			0x040
#define __WSTRB					(0xF << 24)
#define __SEL_IDX(L)				((L) & 0xFFFF)

/*
 * private data
 */
static struct ufs_cal_param *ufs_cal[NUM_OF_UFS_HOST];

static const struct ufs_cal_phy_cfg init_cfg_evt0[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x44, 0x00, PMD_ALL, UNIPRO_DBG_PRD, BRD_ALL},

	{0x200, 0x2800, 0x40, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#if (USE_UFS_REFCLK == USE_19_2_MHZ) //UFS Reference CLK = 19.2MHz
	{0x202, 0x2808, 0x2, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#elif (USE_UFS_REFCLK == USE_38_4_MHZ)  //UFS Reference CLK = 38.4MHz
	{0x202, 0x2808, 0x2, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
	{0x0000, 0x0A4, 0x22, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
#else //UFS Reference CLK = 26MHz
	{0x202, 0x2808, 0x12, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#endif
	{0x12, 0x2048, 0x00, PMD_ALL, PHY_PCS_RX_PRD_ROUND_OFF, BRD_ALL},
	{0xAA, 0x22A8, 0x00, PMD_ALL, PHY_PCS_TX_PRD_ROUND_OFF, BRD_ALL},
	{0xA9, 0x22A4, 0x02, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0xAB, 0x22AC, 0x00, PMD_ALL, PHY_PCS_TX_LR_PRD, BRD_ALL},
	{0x11, 0x2044, 0x00, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x1B, 0x206C, 0x00, PMD_ALL, PHY_PCS_RX_LR_PRD, BRD_ALL},
	{0x2F, 0x20BC, 0x69, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x76, 0x21D8, 0x03, PMD_ALL, PHY_PCS_RX, BRD_ZEBU},
	{0x9E, 0x2278, 0x03, PMD_ALL, PHY_PCS_RX, BRD_ZEBU},
	{0x9F, 0x227C, 0x03, PMD_ALL, PHY_PCS_RX, BRD_ZEBU},

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

	{0xA006, 0x4818, 0x80000000, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},

	{0x0000, 0x10C, 0x10, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x0F0, 0x14, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x118, 0x48, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x800, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x804, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x808, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x80C, 0x0A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x810, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x814, 0x11, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x81C, 0x0C, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xB84, 0xC0, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x8B4, 0xB8, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x8D0, 0x60, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x8E0, 0x13, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8E4, 0x48, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8E8, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8EC, 0x25, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8F0, 0x2A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8F4, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8F8, 0x13, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8FC, 0x13, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x900, 0x4A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x90C, 0x40, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x910, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x974, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x978, 0x3F, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x97C, 0xFF, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x9CC, 0x33, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9D0, 0x50, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xA10, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA14, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xA88, 0x04, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9F4, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xBE8, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xA18, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA1C, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA20, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA24, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xACC, 0x04, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xAD8, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xADC, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAE0, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAE4, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAE8, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAEC, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAF0, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAF4, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAF8, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xB90, 0x1A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xBB4, 0x25, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9A4, 0x1A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xBD0, 0x2F, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xD2C, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD30, 0x23, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD34, 0x23, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD38, 0x45, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD3C, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD40, 0x31, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD44, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD48, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD4C, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD50, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x10C, 0x18, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x10C, 0x00, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0xCE0, 0x08, PMD_ALL, PHY_EMB_CAL_WAIT, BRD_ALL},
	{0xA006, 0x4818, 0x0, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg init_cfg_evt1[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x44, 0x00, PMD_ALL, UNIPRO_DBG_PRD, BRD_ALL},

	{0x200, 0x2800, 0x40, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#if (USE_UFS_REFCLK == USE_19_2_MHZ) //UFS Reference CLK = 19.2MHz
	{0x202, 0x2808, 0x2, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#elif (USE_UFS_REFCLK == USE_38_4_MHZ)  //UFS Reference CLK = 38.4MHz
	{0x202, 0x2808, 0x2, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
	{0x0000, 0x0A4, 0x22, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
#else //UFS Reference CLK = 26MHz
	{0x202, 0x2808, 0x12, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
#endif
	{0x12, 0x2048, 0x00, PMD_ALL, PHY_PCS_RX_PRD_ROUND_OFF, BRD_ALL},
	{0xAA, 0x22A8, 0x00, PMD_ALL, PHY_PCS_TX_PRD_ROUND_OFF, BRD_ALL},
	{0xA9, 0x22A4, 0x02, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0xAB, 0x22AC, 0x00, PMD_ALL, PHY_PCS_TX_LR_PRD, BRD_ALL},
	{0x11, 0x2044, 0x00, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x1B, 0x206C, 0x00, PMD_ALL, PHY_PCS_RX_LR_PRD, BRD_ALL},
	{0x2F, 0x20BC, 0x69, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x76, 0x21D8, 0x03, PMD_ALL, PHY_PCS_RX, BRD_ZEBU},
	{0x9E, 0x2278, 0x03, PMD_ALL, PHY_PCS_RX, BRD_ZEBU},
	{0x9F, 0x227C, 0x03, PMD_ALL, PHY_PCS_RX, BRD_ZEBU},

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

	{0xA006, 0x4818, 0x80000000, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},

	{0x0000, 0x10C, 0x10, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x0F0, 0x14, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x118, 0x48, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x800, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x804, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x808, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x80C, 0x0A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x810, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x814, 0x11, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x81C, 0x0C, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xB84, 0xC0, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8B4, 0xB8, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x8D0, 0x60, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8E0, 0x13, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8E4, 0x48, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8E8, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8EC, 0x25, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8F0, 0x2A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8F4, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8F8, 0x13, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8FC, 0x13, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x900, 0x4A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x90C, 0x40, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x910, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x974, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x978, 0x3F, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x97C, 0xFF, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x9CC, 0x33, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9D0, 0x50, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xA10, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA14, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xA88, 0x04, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9F4, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xBE8, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xA18, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA1C, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA20, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA24, 0x03, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xACC, 0x04, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xAD8, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xADC, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAE0, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAE4, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAE8, 0x0B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAEC, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAF0, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAF4, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xAF8, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xB90, 0x1A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xBB4, 0x25, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9A4, 0x1A, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xBD0, 0x2F, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xD2C, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD30, 0x23, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD34, 0x23, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD38, 0x45, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD3C, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD40, 0x31, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD44, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD48, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD4C, 0x00, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xD50, 0x01, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0x10C, 0x18, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x10C, 0x00, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0xCE0, 0x08, PMD_ALL, PHY_EMB_CAL_WAIT, BRD_ALL},

	{0xA006, 0x4818, 0x0, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_init_cfg_evt0[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x15D2, 0x3348, 0x0, PMD_ALL, UNIPRO_ADAPT_LENGTH, BRD_ALL},
	{0x15D3, 0x334C, 0x0, PMD_ALL, UNIPRO_ADAPT_LENGTH, BRD_ALL},

	{0x9529, 0x38A4, 0x01, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},
	{0x15A4, 0x3290, 0x3E8, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x9529, 0x38A4, 0x00, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_init_cfg_evt1[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x15D2, 0x3348, 0x0, PMD_ALL, UNIPRO_ADAPT_LENGTH, BRD_ALL},
	{0x15D3, 0x334C, 0x0, PMD_ALL, UNIPRO_ADAPT_LENGTH, BRD_ALL},

	{0x9529, 0x38A4, 0x01, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},
	{0x15A4, 0x3290, 0x3E8, PMD_ALL, UNIPRO_STD_MIB, BRD_ALL},
	{0x9529, 0x38A4, 0x00, PMD_ALL, UNIPRO_DBG_MIB, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg calib_of_pwm[] = {
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

static const struct ufs_cal_phy_cfg post_calib_of_pwm[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x20, 0x60, PMD_PWM, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x888, 0x08, PMD_PWM, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x918, 0x01, PMD_PWM, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg calib_of_hs_rate_a[] = {
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

	{0x0000, 0xDA4, 0x11, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x918, 0x03, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_calib_of_hs_rate_a[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xCE4, 0x08, PMD_HS, PHY_EMB_CDR_WAIT, BRD_ALL},
	{0x0000, 0x918, 0x01, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg calib_of_hs_rate_b[] = {
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

	{0x0000, 0xDA4, 0x11, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x918, 0x03, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_calib_of_hs_rate_b[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xCE4, 0x08, PMD_HS, PHY_EMB_CDR_WAIT, BRD_ALL^BRD_ZEBU},
	{0x0000, 0x918, 0x01, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg lane1_sq_off[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x988, 0x08, PMD_ALL, PHY_PMA_TRSV_LANE1_SQ_OFF, BRD_ALL},
	{0x0000, 0x994, 0x0A, PMD_ALL, PHY_PMA_TRSV_LANE1_SQ_OFF, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_h8_enter[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x988, 0x08, PMD_ALL, PHY_PMA_TRSV_SQ, BRD_ALL},
	{0x0000, 0x994, 0x0A, PMD_ALL, PHY_PMA_TRSV_SQ, BRD_ALL},
	{0x0000, 0x04, 0x08, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x00, 0x86, PMD_ALL, PHY_PMA_COMN, BRD_ALL},

	{0x0000, 0x20, 0x60, PMD_HS, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x888, 0x08, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x918, 0x01, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg pre_h8_exit[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x00, 0xC6, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x04, 0x0C, PMD_ALL, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x988, 0x00, PMD_ALL, PHY_PMA_TRSV_SQ, BRD_ALL},
	{0x0000, 0x994, 0x00, PMD_ALL, PHY_PMA_TRSV_SQ, BRD_ALL},

	{0x0000, 0x20, 0xE0, PMD_HS, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x918, 0x03, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x888, 0x18, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xCE0, 0x02, PMD_HS, PHY_EMB_CDR_WAIT, BRD_ZEBU},
	{0x0000, 0xCE4, 0x08, PMD_HS, PHY_EMB_CDR_WAIT, BRD_ALL^BRD_ZEBU},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg loopback_init[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xBB4, 0x23, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x868, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9A8, 0xA1, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9AC, 0x40, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg loopback_set_1[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xBB4, 0x2B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x888, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg loopback_set_2[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x9BC, 0x52, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9A8, 0xA7, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8AC, 0xC3, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg eom_prepare[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xBC0, 0x00, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xA88, 0x05, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x93C, 0x0F, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	/* per gear */
	{0x0000, 0x940, 0x4F, PMD_HS_G4, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x940, 0x2F, PMD_HS_G3, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x940, 0x1F, PMD_HS_G2, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x940, 0x0F, PMD_HS_G1, PHY_PMA_TRSV, BRD_ALL},

	{0x0000, 0xB64, 0xE3, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xB68, 0x04, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0xB6C, 0x00, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg init_cfg_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_init_cfg_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg calib_of_pwm_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_calib_of_pwm_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg calib_of_hs_rate_a_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_calib_of_hs_rate_a_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg calib_of_hs_rate_b_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_calib_of_hs_rate_b_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg lane1_sq_off_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg post_h8_enter_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg pre_h8_exit_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg loopback_init_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg loopback_set_1_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static const struct ufs_cal_phy_cfg loopback_set_2_card[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

const u32 ufs_s_eom_repeat[GEAR_MAX + 1] = {
	0, EOM_RTY_G1, EOM_RTY_G2, EOM_RTY_G3, EOM_RTY_G4
};

/*
 * inline functions
 */
static inline bool is_pwr_mode_hs(u8 m)
{
	return m == FAST_MODE || m == FASTAUTO_MODE;
}

static inline bool is_pwr_mode_pwm(u8 m)
{
	return m == SLOW_MODE || m == SLOWAUTO_MODE;
}

static inline u32 __get_mclk_period(struct ufs_cal_param *p)
{
	return 1000000000U / p->mclk_rate;
}

static inline u32 __get_mclk_period_unipro_18(struct ufs_cal_param *p)
{
	u64 val = 16 * 1000 * 1000000UL;

	return div_u64(val, p->mclk_rate);
}

static inline u32 __get_mclk_period_rnd_off(struct ufs_cal_param *p)
{
	/* assume that mclk_rate is supposed to be unsigned */
	return DIV_ROUND_CLOSEST(1000000000UL, p->mclk_rate);
}

/*
 * This function returns how many ticks is required to line reset
 * for predefined time value.
 */
static inline u32 __get_line_reset_ticks(struct ufs_cal_param *p,
					 u32 time_in_us)
{
	u64 val = (u64)p->mclk_rate * time_in_us;

	return div_u64(val, 1000000U);
}

static inline enum ufs_cal_errno __match_board_by_cfg(u8 board, u8 cfg_board)
{
	enum ufs_cal_errno match = UFS_CAL_ERROR;

	if (board & cfg_board)
		match = UFS_CAL_NO_ERROR;

	return match;
}

static enum ufs_cal_errno __match_mode_by_cfg(struct uic_pwr_mode *pmd,
					      int mode)
{
	enum ufs_cal_errno match;

	if (mode == PMD_ALL) {
		match = UFS_CAL_NO_ERROR;
	} else if (is_pwr_mode_hs(pmd->mode) && mode >= PMD_HS_G1 &&
		   mode <= PMD_HS) {
		match = UFS_CAL_NO_ERROR;
		if (mode != PMD_HS && pmd->gear != (mode - PMD_HS_G1 + 1))
			match = UFS_CAL_ERROR;
	} else if (is_pwr_mode_pwm(pmd->mode) && mode >= PMD_PWM_G1 &&
		   mode <= PMD_PWM) {
		match = UFS_CAL_NO_ERROR;
		if (mode != PMD_PWM && pmd->gear != (mode - PMD_PWM_G1 + 1))
			match = UFS_CAL_ERROR;
	} else {
		/* invalid lanes */
		match = UFS_CAL_ERROR;
	}

	return match;
}

static enum ufs_cal_errno ufs_cal_wait_pll_lock(struct ufs_vs_handle *handle,
						u32 addr, u32 mask)
{
	u32 attempts = TIMEOUT_IN_US / DELAY_PERIOD_IN_US;
	u32 reg;

	while (attempts--) {
		reg = pma_readl(handle, PHY_PMA_COMN_ADDR(addr));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);
	}

	return UFS_CAL_ERROR;
}

static enum ufs_cal_errno ufs_cal_wait_cdr_lock(struct ufs_vs_handle *handle,
						u32 addr, u32 mask, int lane)
{
	u32 attempts = TIMEOUT_IN_US / DELAY_PERIOD_IN_US;
	u32 reg;

	while (attempts--) {
		reg = pma_readl(handle, PHY_PMA_TRSV_ADDR(addr, lane));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);
	}

	return UFS_CAL_ERROR;
}

static enum ufs_cal_errno ufs30_cal_wait_cdr_lock(struct ufs_vs_handle *handle,
						  u32 addr, u32 mask, int lane)
{
	u32 reg;
	u32 i;

	for (i = 0; i < 100; i++) {
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		reg = pma_readl(handle, PHY_PMA_TRSV_ADDR(addr, lane));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		pma_writel(handle, 0x10, PHY_PMA_TRSV_ADDR(0x888, lane));
		pma_writel(handle, 0x18, PHY_PMA_TRSV_ADDR(0x888, lane));
	}

	return UFS_CAL_ERROR;
}

static enum ufs_cal_errno
ufs_cal_wait_cdr_afc_check(struct ufs_vs_handle *handle, u32 addr, u32 mask,
			   int lane)
{
	u32 i;

	for (i = 0; i < 100; i++) {
		u32 reg = 0;

		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		reg = pma_readl(handle, PHY_PMA_TRSV_ADDR(addr, lane));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		pma_writel(handle, 0x7F, PHY_PMA_TRSV_ADDR(0xF0, lane));
		pma_writel(handle, 0xFF, PHY_PMA_TRSV_ADDR(0xF0, lane));
	}

	return UFS_CAL_ERROR;
}

static enum ufs_cal_errno ufs30_cal_done_wait(struct ufs_vs_handle *handle,
					      u32 addr, u32 mask, int lane)
{
	u32 i;

	for (i = 0; i < 100; i++) {
		u32 reg = 0;

		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		reg = pma_readl(handle, PHY_PMA_TRSV_ADDR(addr, lane));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
	}

	return UFS_CAL_NO_ERROR;
}

static inline void __set_pcs(struct ufs_vs_handle *handle,
			     u8 lane, u32 offset, u32 value)
{
	unipro_writel(handle, __WSTRB | __SEL_IDX(lane),
		      UNIP_COMP_AXI_AUX_FIELD);
	unipro_writel(handle, value, offset);
	unipro_writel(handle, __WSTRB, UNIP_COMP_AXI_AUX_FIELD);
}

static enum ufs_cal_errno __config_uic(struct ufs_vs_handle *handle, u8 lane,
				       const struct ufs_cal_phy_cfg *cfg,
				       struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	u32 value, ticks;

	switch (cfg->lyr) {
	/* unipro */
	case UNIPRO_STD_MIB:
	case UNIPRO_DBG_MIB:
		unipro_writel(handle, cfg->val, cfg->addr);
		break;
	case UNIPRO_ADAPT_LENGTH:
		value = unipro_readl(handle, cfg->addr);
		if (value & 0x80) {
			if ((value & 0x7F) < 2)
				unipro_writel(handle, 0x82, cfg->addr);
		} else if (((value + 1) & 0x3)) {
			value |= 0x3;
			unipro_writel(handle, value, cfg->addr);
		}
		break;
	case UNIPRO_DBG_PRD:
		unipro_writel(handle, p->mclk_period_unipro_18, cfg->addr);
		break;
	case UNIPRO_DBG_APB:
		unipro_writel(handle, cfg->val, cfg->addr);
		break;

	/* pcs */
	case PHY_PCS_COMN:
		unipro_writel(handle, cfg->val, cfg->addr);
		break;
	case PHY_PCS_RX:
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr, cfg->val);
		break;
	case PHY_PCS_TX:
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr, cfg->val);
		break;
	case PHY_PCS_RX_PRD:
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr, p->mclk_period);
		break;
	case PHY_PCS_TX_PRD:
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr, p->mclk_period);
		break;
	case PHY_PCS_RX_PRD_ROUND_OFF:
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr,
			  p->mclk_period_rnd_off);
		break;
	case PHY_PCS_TX_PRD_ROUND_OFF:
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr,
			  p->mclk_period_rnd_off);
		break;
	case PHY_PCS_RX_LR_PRD:
		ticks = __get_line_reset_ticks(p, RX_LINE_RESET_DETECT_TIME);
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr,
			  (ticks >> 16) & 0xFF);
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr + 4,
			  (ticks >> 8) & 0xFF);
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr + 8,
			  (ticks >> 0) & 0xFF);
		break;
	case PHY_PCS_TX_LR_PRD:
		ticks = __get_line_reset_ticks(p, TX_LINE_RESET_TIME);
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr,
			  (ticks >> 16) & 0xFF);
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr + 4,
			  (ticks  >> 8) & 0xFF);
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr + 8,
			  (ticks >> 0) & 0xFF);
		break;

	/* pma */
	case PHY_PMA_COMN:
		pma_writel(handle, cfg->val, PHY_PMA_COMN_ADDR(cfg->addr));
		break;
	case PHY_PMA_TRSV:
		pma_writel(handle, cfg->val,
			   PHY_PMA_TRSV_ADDR(cfg->addr, lane));
		break;
	case PHY_PLL_WAIT:
		if (ufs_cal_wait_pll_lock(handle, cfg->addr,
					cfg->val) == UFS_CAL_ERROR)
			ret = UFS_CAL_TIMEOUT;
		break;
	case PHY_CDR_WAIT:
		/* after gear change */
		if (ufs_cal_wait_cdr_lock(handle, cfg->addr,
					cfg->val, lane) == UFS_CAL_ERROR)
			ret = UFS_CAL_TIMEOUT;
		break;
	case PHY_EMB_CDR_WAIT:
		/* after gear change */
		if (ufs30_cal_wait_cdr_lock(p->handle, cfg->addr,
					cfg->val, lane) == UFS_CAL_ERROR)
			ret = UFS_CAL_TIMEOUT;
		break;
		/* after gear change */
	case PHY_CDR_AFC_WAIT:
		if (p->tbl == HOST_CARD) {
			if (ufs_cal_wait_cdr_afc_check(p->handle, cfg->addr,
					cfg->val, lane) == UFS_CAL_ERROR)
				ret = UFS_CAL_TIMEOUT;
		}
		break;
	case PHY_EMB_CAL_WAIT:
		if (ufs30_cal_done_wait(p->handle, cfg->addr,
					cfg->val, lane) == UFS_CAL_ERROR)
			ret = UFS_CAL_TIMEOUT;
		break;
	case COMMON_WAIT:
		if (handle->udelay)
			handle->udelay(cfg->val);
		break;
	case PHY_PMA_TRSV_SQ:
		/* for hibern8 time */
		pma_writel(handle, cfg->val,
				PHY_PMA_TRSV_ADDR(cfg->addr, lane));
		break;
	case PHY_PMA_TRSV_LANE1_SQ_OFF:
		/* for hibern8 time */
		pma_writel(handle, cfg->val,
				PHY_PMA_TRSV_ADDR(cfg->addr, lane));
		break;
	default:
		break;
	}

	return ret;
}

static enum ufs_cal_errno do_cal_config_uic(int i, struct ufs_cal_param *p,
					const struct ufs_cal_phy_cfg *cfg,
					struct uic_pwr_mode *pmd)
{
	struct ufs_vs_handle *handle = p->handle;

	if (p->board && __match_board_by_cfg(p->board, cfg->board) ==
							UFS_CAL_ERROR)
		return UFS_CAL_NO_ERROR;

	if (pmd && __match_mode_by_cfg(pmd, cfg->flg) == UFS_CAL_ERROR)
		return UFS_CAL_NO_ERROR;

	if (i == 0)
		goto skip_base;

	switch (cfg->lyr) {
	case PHY_PCS_COMN:
	case UNIPRO_STD_MIB:
	case UNIPRO_DBG_MIB:
	case UNIPRO_ADAPT_LENGTH:
	case UNIPRO_DBG_PRD:
	case PHY_PMA_COMN:
	case UNIPRO_DBG_APB:
	case PHY_PLL_WAIT:
	case COMMON_WAIT:
		return UFS_CAL_NO_ERROR;
	default:
		break;
	}
skip_base:
	if (i < p->active_rx_lane)
		goto skip_rx_lane;

	switch (cfg->lyr) {
	case PHY_CDR_WAIT:
	case PHY_EMB_CDR_WAIT:
	case PHY_CDR_AFC_WAIT:
		return UFS_CAL_NO_ERROR;
	default:
		break;
	}
skip_rx_lane:
	if (i < p->connected_rx_lane && cfg->lyr == PHY_PMA_TRSV_LANE1_SQ_OFF)
		return UFS_CAL_NO_ERROR;

	if (i >= p->connected_rx_lane && cfg->lyr == PHY_PMA_TRSV_SQ)
		return UFS_CAL_NO_ERROR;

	return __config_uic(handle, i, cfg, p);
}

static enum ufs_cal_errno ufs_cal_config_uic(struct ufs_cal_param *p,
					     const struct ufs_cal_phy_cfg *cfg,
					     struct uic_pwr_mode *pmd)
{
	enum ufs_cal_errno ret = UFS_CAL_INV_ARG;
	u8 i = 0;

	if (!cfg)
		goto out;

	ret = UFS_CAL_NO_ERROR;
	for (; cfg->lyr != PHY_CFG_NONE; cfg++) {
		for (i = 0; i < p->available_lane; i++) {
			ret = do_cal_config_uic(i, p, cfg, pmd);
			if (ret != UFS_CAL_NO_ERROR)
				goto out;
		}
	}
out:
	return ret;
}

/*
 * public functions
 */
enum ufs_cal_errno ufs_cal_loopback_init(struct ufs_cal_param *p)
{
	const struct ufs_cal_phy_cfg *cfg;

	cfg = p->tbl == HOST_CARD ? loopback_init_card : loopback_init;
	return ufs_cal_config_uic(p, cfg, NULL);
}

enum ufs_cal_errno ufs_cal_loopback_set_1(struct ufs_cal_param *p)
{
	const struct ufs_cal_phy_cfg *cfg;

	cfg = p->tbl == HOST_CARD ? loopback_set_1_card : loopback_set_1;
	return ufs_cal_config_uic(p, cfg, NULL);
}

enum ufs_cal_errno ufs_cal_loopback_set_2(struct ufs_cal_param *p)
{
	const struct ufs_cal_phy_cfg *cfg;

	cfg = p->tbl == HOST_CARD ? loopback_set_2_card : loopback_set_2;
	return ufs_cal_config_uic(p, cfg, NULL);
}

enum ufs_cal_errno ufs_cal_post_h8_enter(struct ufs_cal_param *p)
{
	const struct ufs_cal_phy_cfg *cfg;

	cfg = p->tbl == HOST_CARD ? post_h8_enter_card : post_h8_enter;
	return ufs_cal_config_uic(p, cfg, p->pmd);
}

enum ufs_cal_errno ufs_cal_pre_h8_exit(struct ufs_cal_param *p)
{
	const struct ufs_cal_phy_cfg *cfg;

	cfg = p->tbl == HOST_CARD ? pre_h8_exit_card : pre_h8_exit;
	return ufs_cal_config_uic(p, cfg, p->pmd);
}

/*
 * This currently uses only SLOW_MODE and FAST_MODE.
 * If you want others, you should modify this function.
 */
enum ufs_cal_errno ufs_cal_pre_pmc(struct ufs_cal_param *p)
{
	const struct ufs_cal_phy_cfg *cfg;
	struct ufs_vs_handle *handle = p->handle;
	u32 dl_error;

	/* block PA_ERROR_IND_RECEIVED */
	dl_error = unipro_readl(handle, UNIP_DL_ERROR_IRQ_MASK) |
						PA_ERROR_IND_RECEIVED;
	unipro_writel(handle, dl_error, UNIP_DL_ERROR_IRQ_MASK);

	if (p->pmd->mode == SLOW_MODE || p->pmd->mode == SLOWAUTO_MODE)
		cfg = (p->tbl == HOST_CARD) ? calib_of_pwm_card : calib_of_pwm;
	else if (p->pmd->hs_series == PA_HS_MODE_B)
		cfg = (p->tbl == HOST_CARD) ? calib_of_hs_rate_b_card :
							calib_of_hs_rate_b;
	else if (p->pmd->hs_series == PA_HS_MODE_A)
		cfg = (p->tbl == HOST_CARD) ? calib_of_hs_rate_a_card :
							calib_of_hs_rate_a;
	else
		return UFS_CAL_INV_ARG;

	return  ufs_cal_config_uic(p, cfg, p->pmd);
}

/*
 * This currently uses only SLOW_MODE and FAST_MODE.
 * If you want others, you should modify this function.
 */
enum ufs_cal_errno ufs_cal_post_pmc(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	const struct ufs_cal_phy_cfg *cfg;

	if (p->pmd->mode == SLOWAUTO_MODE || p->pmd->mode == SLOW_MODE)
		cfg = (p->tbl == HOST_CARD) ? post_calib_of_pwm_card :
					post_calib_of_pwm;
	else if (p->pmd->hs_series == PA_HS_MODE_B)
		cfg = (p->tbl == HOST_CARD) ? post_calib_of_hs_rate_b_card :
					post_calib_of_hs_rate_b;
	else if (p->pmd->hs_series == PA_HS_MODE_A)
		cfg = (p->tbl == HOST_CARD) ? post_calib_of_hs_rate_a_card :
					post_calib_of_hs_rate_a;
	else
		return UFS_CAL_INV_ARG;

	ret = ufs_cal_config_uic(p, cfg, p->pmd);

	return ret;
}

enum ufs_cal_errno ufs_cal_post_link(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	const struct ufs_cal_phy_cfg *cfg;

	switch (p->max_gear) {
	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
		if (p->evt_ver == 0)
			cfg = (p->tbl == HOST_CARD) ? post_init_cfg_card :
						post_init_cfg_evt0;
		else
			cfg = (p->tbl == HOST_CARD) ? post_init_cfg_card :
						post_init_cfg_evt1;
		break;
	default:
		ret = UFS_CAL_INV_ARG;
		break;
	}

	if (ret)
		return ret;

	ret = ufs_cal_config_uic(p, cfg, NULL);

	/*
	 * If a number of target lanes is 1 and a host's
	 * a number of available lanes is 2,
	 * you should turn off phy power of lane #1.
	 *
	 * This must be modified when a number of available lanes
	 * would grow in the future.
	 */
	if (ret == UFS_CAL_NO_ERROR) {
		if (p->available_lane == 2 && p->connected_rx_lane == 1) {
			cfg = (p->tbl == HOST_CARD) ?
				lane1_sq_off_card : lane1_sq_off;
			ret = ufs_cal_config_uic(p, cfg, NULL);
		}
	}

	/* eom */
	p->eom_sz = EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX *
		ufs_s_eom_repeat[p->max_gear];

	return ret;
}

enum ufs_cal_errno ufs_cal_pre_link(struct ufs_cal_param *p)
{
	const struct ufs_cal_phy_cfg *cfg;

	/* preset mclk periods */
	p->mclk_period = __get_mclk_period(p);
	p->mclk_period_rnd_off = __get_mclk_period_rnd_off(p);
	p->mclk_period_unipro_18 = __get_mclk_period_unipro_18(p);

	if (p->evt_ver == 0)
		cfg = (p->tbl == HOST_CARD) ? init_cfg_card : init_cfg_evt0;
	else
		cfg = (p->tbl == HOST_CARD) ? init_cfg_card : init_cfg_evt1;

	return ufs_cal_config_uic(p, cfg, NULL);
}

static enum ufs_cal_errno ufs_cal_eom_prepare(struct ufs_cal_param *p)
{
	return ufs_cal_config_uic(p, eom_prepare, p->pmd);
}

static u32 ufs_cal_get_eom_err_cnt(struct ufs_vs_handle *handle, u32 lane_loop)
{
	return (pma_readl(handle,
		PHY_PMA_TRSV_ADDR(0xD20, lane_loop)) << 16) +
		(pma_readl(handle, PHY_PMA_TRSV_ADDR(0xD24, lane_loop)) << 8) +
		(pma_readl(handle, PHY_PMA_TRSV_ADDR(0xD28, lane_loop)));
}

static void ufs_cal_sweep_get_eom_data(struct ufs_vs_handle *handle, u32 *cnt,
				       struct ufs_cal_param *p, u32 lane,
				       u32 repeat)
{
	u32 phase, vref;
	u32 errors;
	struct ufs_eom_result_s *data = p->eom[lane];

	for (phase = 0; phase < EOM_PH_SEL_MAX; phase++) {
		pma_writel(handle, phase, PHY_PMA_TRSV_ADDR(0xB78, lane));

		for (vref = 0; vref < EOM_DEF_VREF_MAX; vref++) {
			pma_writel(handle, 0x18,
				   PHY_PMA_TRSV_ADDR(0xB5C, lane));
			pma_writel(handle, vref,
				   PHY_PMA_TRSV_ADDR(0xB74, lane));
			pma_writel(handle, 0x19,
				   PHY_PMA_TRSV_ADDR(0xB5C, lane));

			errors = ufs_cal_get_eom_err_cnt(handle, lane);

			if (handle->udelay)
				handle->udelay(1);

			data[*cnt].v_phase =
					phase + (repeat * EOM_PH_SEL_MAX);
			data[*cnt].v_vref = vref;
			data[*cnt].v_err = errors;
			(*cnt)++;
		}
	}
}

enum ufs_cal_errno ufs_cal_eom(struct ufs_cal_param *p)
{
	u32 repeat, lane;

	ufs_cal_eom_prepare(p);

	repeat = (p->max_gear <= GEAR_MAX) ? ufs_s_eom_repeat[p->max_gear] : 0;
	if (repeat == 0)
		return UFS_CAL_ERROR;
	if (repeat > EOM_RTY_MAX)
		return UFS_CAL_INV_CONF;

	for (lane = 0; lane < p->available_lane; lane++) {
		u32 cnt = 0;
		u32 i;

		for (i = 0; i < repeat; i++)
			ufs_cal_sweep_get_eom_data(p->handle, &cnt, p, lane, i);
	}
	return UFS_CAL_NO_ERROR;
}

enum ufs_cal_errno ufs_cal_init(struct ufs_cal_param *p, int idx)
{
	/*
	 * Return if innput index is greater than
	 * the maximum that cal supports
	 */
	if (idx >= NUM_OF_UFS_HOST)
		return UFS_CAL_INV_ARG;

	ufs_cal[idx] = p;

	return UFS_CAL_NO_ERROR;
}
