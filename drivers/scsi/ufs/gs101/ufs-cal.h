/* SPDX-License-Identifier: GPL-2.0-only */
//
// UFS Host Controller driver for Exynos specific extensions
//
// Copyright (C) 2013-2014 Samsung Electronics Co., Ltd.
//
// Authors:
//	Kiwoong <kwmad.kim@samsung.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
#ifndef _GS101_UFS_CAL_H
#define _GS101_UFS_CAL_H

/* To display */
#define UFS_CAL_TABLE_VER 2
/* If not matched with ufs-cal-if, compling would fail */
#define UFS_CAL_TABLE_COMPAT_IF_VER 4

static const struct ufs_cal_phy_cfg init_cfg_evt0[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x44, 0x00, PMD_ALL, UNIPRO_DBG_PRD, BRD_ALL},

	{0x200, 0x2800, 0x40, PMD_ALL, PHY_PCS_COMN, BRD_ALL},
	{0x12, 0x2048, 0x00, PMD_ALL, PHY_PCS_RX_PRD_ROUND_OFF, BRD_ALL},
	{0xAA, 0x22A8, 0x00, PMD_ALL, PHY_PCS_TX_PRD_ROUND_OFF, BRD_ALL},
	{0xA9, 0x22A4, 0x02, PMD_ALL, PHY_PCS_TX, BRD_ALL},
	{0xAB, 0x22AC, 0x00, PMD_ALL, PHY_PCS_TX_LR_PRD, BRD_ALL},
	{0x11, 0x2044, 0x00, PMD_ALL, PHY_PCS_RX, BRD_ALL},
	{0x1B, 0x206C, 0x00, PMD_ALL, PHY_PCS_RX_LR_PRD, BRD_ALL},
	{0x2F, 0x20BC, 0x79, PMD_ALL, PHY_PCS_RX, BRD_ALL},

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
	{0x0000, 0x20, 0x60, PMD_PWM, PHY_PMA_COMN, BRD_ALL},
	{0x0000, 0x888, 0x08, PMD_PWM, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x918, 0x01, PMD_PWM, PHY_PMA_TRSV, BRD_ALL},

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

	{0x0000, 0xDA4, 0x11, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x918, 0x03, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_calib_of_hs_rate_a[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xCE0, 0x02, PMD_HS, PHY_EMB_CDR_WAIT, BRD_ZEBU},
	{0x0000, 0xCE4, 0x08, PMD_HS, PHY_EMB_CDR_WAIT, BRD_ALL^BRD_ZEBU},
	{0x0000, 0x918, 0x01, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

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

	{0x0000, 0xDA4, 0x11, PMD_HS, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x918, 0x03, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_calib_of_hs_rate_b[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xCE0, 0x02, PMD_HS, PHY_EMB_CDR_WAIT, BRD_ZEBU},
	{0x0000, 0xCE4, 0x08, PMD_HS, PHY_EMB_CDR_WAIT, BRD_ALL^BRD_ZEBU},
	{0x0000, 0x918, 0x01, PMD_HS, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg lane1_sq_off[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x988, 0x08, PMD_ALL, PHY_PMA_TRSV_LANE1_SQ_OFF, BRD_ALL},
	{0x0000, 0x994, 0x0A, PMD_ALL, PHY_PMA_TRSV_LANE1_SQ_OFF, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg post_h8_enter[] = {
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

static struct ufs_cal_phy_cfg pre_h8_exit[] = {
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

static struct ufs_cal_phy_cfg loopback_init[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xBB4, 0x23, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x868, 0x02, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9A8, 0xA1, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9AC, 0x40, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg loopback_set_1[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0xBB4, 0x2B, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x888, 0x06, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg loopback_set_2[] = {
	/* mib(just to monitor), sfr offset, value, .. */
	{0x0000, 0x9BC, 0x52, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x9A8, 0xA7, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},
	{0x0000, 0x8AC, 0xC3, PMD_ALL, PHY_PMA_TRSV, BRD_ALL},

	{0, 0, 0, 0, PHY_CFG_NONE, BRD_ALL}
};

static struct ufs_cal_phy_cfg eom_prepare[] = {
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
#endif	/* _GS101_UFS_CAL_H */
