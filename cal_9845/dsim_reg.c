// SPDX-License-Identifier: GPL-2.0-only
/*
 * cal_9845/dsim_reg.c
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register access functions for Samsung EXYNOS SoC MIPI-DSI Master driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "regs-dsim.h"
#include <dsim_cal.h>
#include <cal_config.h>

static struct cal_regs_desc regs_desc[REGS_DSIM_TYPE_MAX][MAX_DSI_CNT];

#define dsim_regs_desc(id)				\
	(&regs_desc[REGS_DSIM_DSI][id])
#define dsim_read(id, offset)				\
	cal_read(dsim_regs_desc(id), offset)
#define dsim_write(id, offset, val)			\
	cal_write(dsim_regs_desc(id), offset, val)
#define dsim_read_mask(id, offset, mask)		\
	cal_read_mask(dsim_regs_desc(id), offset, mask)
#define dsim_write_mask(id, offset, val, mask)		\
	cal_write_mask(dsim_regs_desc(id), offset, val, mask)

#define dphy_regs_desc(id)				\
	(&regs_desc[REGS_DSIM_PHY][id])
#define dsim_phy_read(id, offset)			\
	cal_read(dphy_regs_desc(id), offset)
#define dsim_phy_write(id, offset, val)			\
	cal_write(dphy_regs_desc(id), offset, val)
#define dsim_phy_read_mask(id, offset, mask)		\
	cal_read_mask(dphy_regs_desc(id), offset, mask)
#define dsim_phy_write_mask(id, offset, val, mask)	\
	cal_write_mask(dphy_regs_desc(id), offset, val, mask)

#define dphy_bias_regs_desc(id)				\
	(&regs_desc[REGS_DSIM_PHY_BIAS][id])
#define dsim_phy_extra_write(id, offset, val)		\
	cal_write(dphy_bias_regs_desc(id), offset, val)
#define dsim_phy_extra_read_mask(id, offset, mask)       \
	cal_read_mask(dphy_bias_regs_desc(id), offset, mask)
#define dsim_phy_extra_write_mask(id, offset, val, mask) \
	cal_write_mask(dphy_bias_regs_desc(id), offset, val, mask);

#define sys_regs_desc(id)				\
	(&regs_desc[REGS_DSIM_SYS][id])
#define dsim_sys_read(id, offset)			\
	cal_read(sys_regs_desc(id), offset)
#define dsim_sys_write(id, offset, val)			\
	cal_write(sys_regs_desc(id), offset, val)
#define dsim_sys_write_mask(id, offset, val, mask)	\
	cal_write_mask(sys_regs_desc(id), offset, val, mask)

/* dsim version */
#define DSIM_VER_EVT0			0x02040000
#define DSIM_VER_EVT1			0x02050000

/* These definitions are need to guide from AP team */
#define DSIM_STOP_STATE_CNT		0xA
#define DSIM_BTA_TIMEOUT		0xff
#define DSIM_LP_RX_TIMEOUT		0xffff
#define DSIM_MULTI_PACKET_CNT		0xffff
#define DSIM_PLL_STABLE_TIME		0x682A
#define DSIM_PH_FIFOCTRL_THRESHOLD	32 /* 1 ~ 32 */

#define PLL_SLEEP_CNT_MULT		450
#define PLL_SLEEP_CNT_MARGIN_RATIO	0	/* 10% ~ 20% */
#define PLL_SLEEP_CNT_MARGIN		(PLL_SLEEP_CNT_MULT *	\
					PLL_SLEEP_CNT_MARGIN_RATIO / 100)

/* If below values depend on panel. These values wil be move to panel file.
 * And these values are valid in case of video mode only.
 */
#define DSIM_CMD_ALLOW_VALUE		4
#define DSIM_STABLE_VFP_VALUE		2
#define TE_MARGIN			5	/* 5% */

#define PLL_LOCK_CNT_MUL		500
#define PLL_LOCK_CNT_MARGIN_RATIO	0	/* 10% ~ 20% */
#define PLL_LOCK_CNT_MARGIN		(PLL_LOCK_CNT_MUL *	\
					PLL_LOCK_CNT_MARGIN_RATIO / 100)

#if !defined(CONFIG_BOARD_EMULATOR)
static DEFINE_MUTEX(dphy_lock);
#endif

static const u32 DSIM_PHY_BIAS_CON_VAL[] = {
	0x00000010,
	0x00000110,
	0x00003223,
	0x00000000,
	0x00000000,
};

static const u32 DSIM_PHY_PLL_CON_VAL[] = {
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000500,
	0x0000008E, /* Need to check default value of PLL_CON6 */
	0x00001450,
	0x00000A28,
};

static const u32 DSIM_PHY_MC_GNR_CON_VAL[] = {
	0x00000000,
	0x00001450,
};

static const u32 DSIM_PHY_MC_ANA_CON_VAL[] = {
	/* EDGE_CON[14:12] DPHY=3'b111, CPHY=3'b001 */
	0x00007033, /* Need to check default value of MC_ANA_CON0 */
	0x00000000,
	0x00000002,
};

/* same value in all master data lane */
static const u32 DSIM_PHY_MD_GNR_CON_VAL[] = {
	0x00000000,
	0x00001450,
};
/* Need to check reg_cnt */
static const u32 DSIM_PHY_MD_ANA_CON_VAL[] = {
	0x00007033,
	0x00000000,
	0x00000000,
	0x00000000,
};

/* DPHY timing table */
/* below table have to be changed to meet MK DPHY spec*/
static const u32 dphy_timing[][10] = {
	/* bps, clk_prepare, clk_zero, clk_post, clk_trail,
	 * hs_prepare, hs_zero, hs_trail, lpx, hs_exit
	 */
	{2500, 12, 43, 11, 11, 11, 20, 10, 9, 17},
	{2490, 12, 43, 11, 11, 11, 20, 10, 9, 17},
	{2480, 12, 42, 11, 11, 11, 20, 10, 9, 17},
	{2470, 11, 43, 11, 11, 11, 20, 10, 9, 16},
	{2460, 11, 43, 11, 11, 11, 20, 10, 9, 16},
	{2450, 11, 43, 11, 11, 11, 20, 10, 9, 16},
	{2440, 11, 42, 11, 11, 11, 19, 10, 9, 16},
	{2430, 11, 42, 11, 11, 11, 19, 10, 9, 16},
	{2420, 11, 42, 11, 10, 11, 19, 10, 9, 16},
	{2410, 11, 42, 11, 10, 11, 19, 10, 9, 16},
	{2400, 11, 41, 10, 10, 11, 19, 10, 8, 16},
	{2390, 11, 41, 10, 10, 11, 19, 10, 8, 16},
	{2380, 11, 41, 10, 10, 11, 19, 9, 8, 16},
	{2370, 11, 41, 10, 10, 11, 18, 9, 8, 16},
	{2360, 11, 41, 10, 10, 11, 18, 9, 8, 16},
	{2350, 11, 40, 10, 10, 11, 18, 9, 8, 16},
	{2340, 11, 40, 10, 10, 11, 18, 9, 8, 16},
	{2330, 11, 40, 10, 10, 10, 19, 9, 8, 16},
	{2320, 11, 40, 10, 10, 10, 19, 9, 8, 15},
	{2310, 11, 39, 10, 10, 10, 19, 9, 8, 15},
	{2300, 11, 39, 10, 10, 10, 18, 9, 8, 15},
	{2290, 11, 39, 10, 10, 10, 18, 9, 8, 15},
	{2280, 11, 39, 10, 10, 10, 18, 9, 8, 15},
	{2270, 10, 39, 10, 10, 10, 18, 9, 8, 15},
	{2260, 10, 39, 10, 10, 10, 18, 9, 8, 15},
	{2250, 10, 39, 10, 10, 10, 18, 9, 8, 15},
	{2240, 10, 39, 10, 10, 10, 18, 9, 8, 15},
	{2230, 10, 38, 10, 10, 10, 18, 9, 8, 15},
	{2220, 10, 38, 10, 10, 10, 17, 9, 8, 15},
	{2210, 10, 38, 10, 10, 10, 17, 9, 8, 15},
	{2200, 10, 38, 9, 10, 10, 17, 9, 8, 15},
	{2190, 10, 38, 9, 9, 10, 17, 9, 8, 15},
	{2180, 10, 37, 9, 9, 10, 17, 9, 8, 14},
	{2170, 10, 37, 9, 9, 10, 17, 9, 8, 14},
	{2160, 10, 37, 9, 9, 10, 17, 9, 8, 14},
	{2150, 10, 37, 9, 9, 10, 16, 8, 8, 14},
	{2140, 10, 36, 9, 9, 10, 16, 8, 8, 14},
	{2130, 10, 36, 9, 9, 10, 16, 8, 7, 14},
	{2120, 10, 36, 9, 9, 9, 17, 8, 7, 14},
	{2110, 10, 36, 9, 9, 9, 17, 8, 7, 14},
	{2100, 10, 35, 9, 9, 9, 17, 8, 7, 14},
	{2090, 10, 35, 9, 9, 9, 17, 8, 7, 14},
	{2080, 9, 36, 9, 9, 9, 16, 8, 7, 14},
	{2070, 9, 36, 9, 9, 9, 16, 8, 7, 14},
	{2060, 9, 35, 9, 9, 9, 16, 8, 7, 14},
	{2050, 9, 35, 9, 9, 9, 16, 8, 7, 14},
	{2040, 9, 35, 9, 9, 9, 16, 8, 7, 14},
	{2030, 9, 35, 9, 9, 9, 16, 8, 7, 13},
	{2020, 9, 35, 9, 9, 9, 16, 8, 7, 13},
	{2010, 9, 34, 9, 9, 9, 15, 8, 7, 13},
	{2000, 9, 34, 8, 9, 9, 15, 8, 7, 13},
	{1990, 9, 34, 8, 9, 9, 15, 8, 7, 13},
	{1980, 9, 34, 8, 9, 9, 15, 8, 7, 13},
	{1970, 9, 33, 8, 9, 9, 15, 8, 7, 13},
	{1960, 9, 33, 8, 9, 9, 15, 8, 7, 13},
	{1950, 9, 33, 8, 8, 9, 15, 8, 7, 13},
	{1940, 9, 33, 8, 8, 9, 15, 8, 7, 13},
	{1930, 9, 32, 8, 8, 9, 14, 8, 7, 13},
	{1920, 9, 32, 8, 8, 9, 14, 8, 7, 13},
	{1910, 9, 32, 8, 8, 8, 15, 7, 7, 13},
	{1900, 9, 32, 8, 8, 8, 15, 7, 7, 13},
	{1890, 9, 31, 8, 8, 8, 15, 7, 7, 12},
	{1880, 8, 32, 8, 8, 8, 15, 7, 7, 12},
	{1870, 8, 32, 8, 8, 8, 15, 7, 7, 12},
	{1860, 8, 32, 8, 8, 8, 14, 7, 6, 12},
	{1850, 8, 32, 8, 8, 8, 14, 7, 6, 12},
	{1840, 8, 31, 8, 8, 8, 14, 7, 6, 12},
	{1830, 8, 31, 8, 8, 8, 14, 7, 6, 12},
	{1820, 8, 31, 8, 8, 8, 14, 7, 6, 12},
	{1810, 8, 31, 8, 8, 8, 14, 7, 6, 12},
	{1800, 8, 30, 7, 8, 8, 14, 7, 6, 12},
	{1790, 8, 30, 7, 8, 8, 13, 7, 6, 12},
	{1780, 8, 30, 7, 8, 8, 13, 7, 6, 12},
	{1770, 8, 30, 7, 8, 8, 13, 7, 6, 12},
	{1760, 8, 29, 7, 8, 8, 13, 7, 6, 12},
	{1750, 8, 29, 7, 8, 8, 13, 7, 6, 12},
	{1740, 8, 29, 7, 8, 8, 13, 7, 6, 11},
	{1730, 8, 29, 7, 8, 8, 13, 7, 6, 11},
	{1720, 8, 29, 7, 7, 8, 13, 7, 6, 11},
	{1710, 8, 28, 7, 7, 8, 12, 7, 6, 11},
	{1700, 8, 28, 7, 7, 7, 13, 7, 6, 11},
	{1690, 8, 28, 7, 7, 7, 13, 7, 6, 11},
	{1680, 7, 29, 7, 7, 7, 13, 6, 6, 11},
	{1670, 7, 28, 7, 7, 7, 13, 6, 6, 11},
	{1660, 7, 28, 7, 7, 7, 13, 6, 6, 11},
	{1650, 7, 28, 7, 7, 7, 13, 6, 6, 11},
	{1640, 7, 28, 7, 7, 7, 12, 6, 6, 11},
	{1630, 7, 27, 7, 7, 7, 12, 6, 6, 11},
	{1620, 7, 27, 7, 7, 7, 12, 6, 6, 11},
	{1610, 7, 27, 7, 7, 7, 12, 6, 6, 11},
	{1600, 7, 27, 6, 7, 7, 12, 6, 5, 10},
	{1590, 7, 26, 6, 7, 7, 12, 6, 5, 10},
	{1580, 7, 26, 6, 7, 7, 12, 6, 5, 10},
	{1570, 7, 26, 6, 7, 7, 11, 6, 5, 10},
	{1560, 7, 26, 6, 7, 7, 11, 6, 5, 10},
	{1550, 7, 26, 6, 7, 7, 11, 6, 5, 10},
	{1540, 7, 25, 6, 7, 7, 11, 6, 5, 10},
	{1530, 7, 25, 6, 7, 7, 11, 6, 5, 10},
	{1520, 7, 25, 6, 7, 7, 11, 6, 5, 10},
	{1510, 7, 25, 6, 7, 7, 11, 6, 5, 10},
	{1500, 7, 24, 6, 7, 7, 10, 6, 5, 10},
	{1490, 59, 25, 6, 77, 59, 10, 70, 44, 9},
	{1480, 59, 24, 6, 76, 58, 10, 70, 44, 9},
	{1470, 58, 24, 6, 76, 58, 10, 69, 44, 9},
	{1460, 58, 24, 6, 76, 58, 10, 69, 43, 9},
	{1450, 58, 24, 6, 75, 57, 10, 68, 43, 9},
	{1440, 57, 24, 6, 75, 57, 10, 68, 43, 9},
	{1430, 57, 23, 6, 75, 56, 10, 68, 42, 8},
	{1420, 56, 23, 6, 74, 56, 9, 67, 42, 8},
	{1410, 56, 23, 6, 74, 56, 9, 67, 42, 8},
	{1400, 56, 23, 5, 74, 55, 9, 67, 41, 8},
	{1390, 55, 23, 5, 73, 55, 9, 66, 41, 8},
	{1380, 55, 23, 5, 73, 54, 9, 66, 41, 8},
	{1370, 54, 22, 5, 72, 54, 9, 66, 41, 8},
	{1360, 54, 22, 5, 72, 54, 9, 65, 40, 8},
	{1350, 54, 22, 5, 72, 53, 9, 65, 40, 8},
	{1340, 53, 22, 5, 71, 53, 9, 65, 40, 8},
	{1330, 53, 22, 5, 71, 53, 9, 64, 39, 8},
	{1320, 52, 22, 5, 71, 52, 8, 64, 39, 8},
	{1310, 52, 21, 5, 70, 52, 8, 64, 39, 8},
	{1300, 51, 21, 5, 70, 51, 8, 63, 38, 8},
	{1290, 51, 21, 5, 70, 51, 8, 63, 38, 7},
	{1280, 51, 21, 5, 69, 51, 8, 63, 38, 7},
	{1270, 50, 21, 5, 69, 50, 8, 62, 38, 7},
	{1260, 50, 20, 5, 69, 50, 8, 62, 37, 7},
	{1250, 49, 20, 5, 68, 49, 8, 62, 37, 7},
	{1240, 49, 20, 5, 68, 49, 8, 61, 37, 7},
	{1230, 49, 20, 5, 68, 49, 8, 61, 36, 7},
	{1220, 48, 20, 5, 67, 48, 8, 61, 36, 7},
	{1210, 48, 19, 5, 67, 48, 7, 60, 36, 7},
	{1200, 47, 19, 4, 67, 48, 7, 60, 35, 7},
	{1190, 47, 19, 4, 66, 47, 7, 60, 35, 7},
	{1180, 47, 19, 4, 66, 47, 7, 59, 35, 7},
	{1170, 46, 19, 4, 66, 46, 7, 59, 35, 7},
	{1160, 46, 18, 4, 65, 46, 7, 59, 34, 7},
	{1150, 45, 18, 4, 65, 46, 7, 58, 34, 7},
	{1140, 45, 18, 4, 65, 45, 7, 58, 34, 6},
	{1130, 45, 18, 4, 64, 45, 7, 58, 33, 6},
	{1120, 44, 18, 4, 64, 44, 7, 57, 33, 6},
	{1110, 44, 18, 4, 64, 44, 7, 57, 33, 6},
	{1100, 43, 17, 4, 63, 44, 6, 57, 32, 6},
	{1090, 43, 17, 4, 63, 43, 6, 56, 32, 6},
	{1080, 43, 17, 4, 63, 43, 6, 56, 32, 6},
	{1070, 42, 17, 4, 62, 43, 6, 56, 32, 6},
	{1060, 42, 17, 4, 62, 42, 6, 55, 31, 6},
	{1050, 41, 17, 4, 62, 42, 6, 55, 31, 6},
	{1040, 41, 16, 4, 61, 41, 6, 54, 31, 6},
	{1030, 41, 16, 4, 61, 41, 6, 54, 30, 6},
	{1020, 40, 16, 4, 61, 41, 6, 54, 30, 6},
	{1010, 40, 16, 4, 60, 40, 6, 53, 30, 6},
	{1000, 39, 16, 3, 60, 40, 6, 53, 29, 5},
	{990, 39, 15, 3, 60, 39, 6, 53, 29, 5},
	{980, 39, 15, 3, 59, 39, 5, 52, 29, 5},
	{970, 38, 15, 3, 59, 39, 5, 52, 29, 5},
	{960, 38, 15, 3, 59, 38, 5, 52, 28, 5},
	{950, 37, 15, 3, 58, 38, 5, 51, 28, 5},
	{940, 37, 14, 3, 58, 38, 5, 51, 28, 5},
	{930, 37, 14, 3, 57, 37, 5, 51, 27, 5},
	{920, 36, 14, 3, 57, 37, 5, 50, 27, 5},
	{910, 36, 14, 3, 57, 36, 5, 50, 27, 5},
	{900, 35, 14, 3, 56, 36, 5, 50, 26, 5},
	{890, 35, 14, 3, 56, 36, 5, 49, 26, 5},
	{880, 35, 13, 3, 56, 35, 5, 49, 26, 5},
	{870, 34, 13, 3, 55, 35, 4, 49, 26, 5},
	{860, 34, 13, 3, 55, 35, 4, 48, 25, 5},
	{850, 33, 13, 3, 55, 34, 4, 48, 25, 4},
	{840, 33, 13, 3, 54, 34, 4, 48, 25, 4},
	{830, 33, 12, 3, 54, 33, 4, 47, 24, 4},
	{820, 32, 12, 3, 54, 33, 4, 47, 24, 4},
	{810, 32, 12, 3, 53, 33, 4, 47, 24, 4},
	{800, 31, 12, 2, 53, 32, 4, 46, 23, 4},
	{790, 31, 12, 2, 53, 32, 4, 46, 23, 4},
	{780, 30, 12, 2, 52, 31, 4, 46, 23, 4},
	{770, 30, 11, 2, 52, 31, 4, 45, 23, 4},
	{760, 30, 11, 2, 52, 31, 3, 45, 22, 4},
	{750, 29, 11, 2, 51, 30, 3, 45, 22, 4},
	{740, 29, 11, 2, 51, 30, 3, 44, 22, 4},
	{730, 28, 11, 2, 51, 30, 3, 44, 21, 4},
	{720, 28, 10, 2, 50, 29, 3, 44, 21, 4},
	{710, 28, 10, 2, 50, 29, 3, 43, 21, 4},
	{700, 27, 10, 2, 50, 28, 3, 43, 20, 3},
	{690, 27, 10, 2, 49, 28, 3, 43, 20, 3},
	{680, 26, 10, 2, 49, 28, 3, 42, 20, 3},
	{670, 26, 10, 2, 49, 27, 3, 42, 20, 3},
	{660, 26, 9, 2, 48, 27, 3, 42, 19, 3},
	{650, 25, 9, 2, 48, 26, 3, 41, 19, 3},
	{640, 25, 9, 2, 48, 26, 2, 41, 19, 3},
	{630, 24, 9, 2, 47, 26, 2, 40, 18, 3},
	{620, 24, 9, 2, 47, 25, 2, 40, 18, 3},
	{610, 24, 8, 2, 47, 25, 2, 40, 18, 3},
	{600, 23, 8, 1, 46, 25, 2, 39, 17, 3},
	{590, 23, 8, 1, 46, 24, 2, 39, 17, 3},
	{580, 22, 8, 1, 46, 24, 2, 39, 17, 3},
	{570, 22, 8, 1, 45, 23, 2, 38, 17, 3},
	{560, 22, 7, 1, 45, 23, 2, 38, 16, 2},
	{550, 21, 7, 1, 45, 23, 2, 38, 16, 2},
	{540, 21, 7, 1, 44, 22, 2, 37, 16, 2},
	{530, 20, 7, 1, 44, 22, 1, 37, 15, 2},
	{520, 20, 7, 1, 43, 21, 1, 37, 15, 2},
	{510, 20, 6, 1, 43, 21, 1, 36, 15, 2},
	{500, 19, 6, 1, 43, 21, 1, 36, 14, 2},
	{490, 19, 6, 1, 42, 20, 1, 36, 14, 2},
	{480, 18, 6, 1, 42, 20, 1, 35, 14, 2},
	{470, 18, 6, 1, 42, 20, 1, 35, 14, 2},
	{460, 18, 6, 1, 41, 19, 1, 35, 13, 2},
	{450, 17, 5, 1, 41, 19, 1, 34, 13, 2},
	{440, 17, 5, 1, 41, 18, 1, 34, 13, 2},
	{430, 16, 5, 1, 40, 18, 0, 34, 12, 2},
	{420, 16, 5, 1, 40, 18, 0, 33, 12, 2},
	{410, 16, 5, 1, 40, 17, 0, 33, 12, 1},
	{400, 15, 5, 0, 39, 17, 0, 33, 11, 1}
};

static const u32 b_dphyctl[14] = {
	0x0af, 0x0c8, 0x0e1, 0x0fa,			/* esc 7 ~ 10 */
	0x113, 0x12c, 0x145, 0x15e, 0x177,	/* esc 11 ~ 15 */
	0x190, 0x1a9, 0x1c2, 0x1db, 0x1f4	/* esc 16 ~ 20 */
};

/**************************** DPHY CAL functions ******************************/
static u32 dsim_reg_get_version(u32 id)
{
	u32 version;

	version = dsim_read(id, DSIM_VERSION);

	return version;
}

static void dsim_reg_set_dphy_dither_en(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON4, val, DSIM_PHY_DITHER_EN);
}

#ifdef DPHY_LOOP
void dsim_reg_set_dphy_loop_back_test(u32 id)
{
	dsim_phy_write_mask(id, 0x0370, 1, (0x3 << 0));
	dsim_phy_write_mask(id, 0x0470, 1, (0x3 << 0));
	dsim_phy_write_mask(id, 0x0570, 1, (0x3 << 0));
	dsim_phy_write_mask(id, 0x0670, 1, (0x3 << 0));
	dsim_phy_write_mask(id, 0x0770, 1, (0x3 << 0));
}

static void dsim_reg_set_dphy_loop_test(u32 id)
{
	dsim_phy_write_mask(id, 0x0374, ~0, (1 << 3));
	dsim_phy_write_mask(id, 0x0474, ~0, (1 << 3));
	dsim_phy_write_mask(id, 0x0574, ~0, (1 << 3));
	dsim_phy_write_mask(id, 0x0674, ~0, (1 << 3));
	dsim_phy_write_mask(id, 0x0774, ~0, (1 << 3));

	dsim_phy_write_mask(id, 0x0374, 0x6, (0x7 << 0));
	dsim_phy_write_mask(id, 0x0474, 0x6, (0x7 << 0));
	dsim_phy_write_mask(id, 0x0574, 0x6, (0x7 << 0));
	dsim_phy_write_mask(id, 0x0674, 0x6, (0x7 << 0));
	dsim_phy_write_mask(id, 0x0774, 0x6, (0x7 << 0));

	dsim_phy_write_mask(id, 0x037c, 0x2, (0xffff << 0));
	dsim_phy_write_mask(id, 0x047c, 0x2, (0xffff << 0));
	dsim_phy_write_mask(id, 0x057c, 0x2, (0xffff << 0));
	dsim_phy_write_mask(id, 0x067c, 0x2, (0xffff << 0));
	dsim_phy_write_mask(id, 0x077c, 0x2, (0xffff << 0));
}
#endif

static void dsim_reg_set_dphy_wclk_buf_sft(u32 id, u32 cnt)
{
	u32 val = DSIM_PHY_WCLK_BUF_SFT_CNT(cnt);

	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON6, val,
			DSIM_PHY_WCLK_BUF_SFT_CNT_MASK);
}

/* DPHY setting */
static void dsim_reg_set_pll_freq(u32 id, u32 p, u32 m, u32 s, u32 k)
{
	u32 val, mask;

	/* K value */
	val = DSIM_PHY_PMS_K(k);
	dsim_phy_write(id, DSIM_PHY_PLL_CON1, val);

	/* P & S value */
	val = DSIM_PHY_PMS_P(p) | DSIM_PHY_PMS_S(s);
	mask = DSIM_PHY_PMS_P_MASK | DSIM_PHY_PMS_S_MASK;
	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON0, val, mask);

	/* M value */
	val = DSIM_PHY_PMS_M(m);
	mask = DSIM_PHY_PMS_M_MASK;
	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON2, val, mask);
}

static void dsim_reg_set_dphy_timing_values(u32 id,
			struct dphy_timing_value *t, u32 hsmode)
{
	u32 val;
	u32 hs_en, skewcal_en;
	u32 i;

	/* HS mode setting */
	if (hsmode) {
		/* under 1500Mbps : don't need SKEWCAL enable */
		hs_en = 1;
		skewcal_en = 0;
	} else {
		/* above 1500Mbps : need SKEWCAL enable */
		hs_en = 0;
		skewcal_en = 1;
	}

	/* clock lane setting */
	val = DSIM_PHY_ULPS_EXIT(t->b_dphyctl);
	dsim_phy_write(id, DSIM_PHY_MC_TIME_CON4, val);

	val = DSIM_PHY_HSTX_CLK_SEL(hs_en) | DSIM_PHY_TLPX(t->lpx);
	dsim_phy_write(id, DSIM_PHY_MC_TIME_CON0, val);

	/* skew cal implementation : disable */
	val = skewcal_en;
	dsim_phy_write(id, DSIM_PHY_MC_DESKEW_CON0, val);
	/* add 'run|init_run|wait_run time' if skewcal is enabled */

	val = DSIM_PHY_TCLK_PREPARE(t->clk_prepare) |
		DSIM_PHY_TCLK_ZERO(t->clk_zero);
	dsim_phy_write(id, DSIM_PHY_MC_TIME_CON1, val);

	val = DSIM_PHY_THS_EXIT(t->hs_exit) | DSIM_PHY_TCLK_TRAIL(t->clk_trail);
	dsim_phy_write(id, DSIM_PHY_MC_TIME_CON2, val);

	val = DSIM_PHY_TCLK_POST(t->clk_post);
	dsim_phy_write(id, DSIM_PHY_MC_TIME_CON3, val);

	/* add other clock lane setting if necessary */

	/* data lane setting : D0 ~ D3 */
	for (i = 0; i < 4; i++) {
		val = DSIM_PHY_ULPS_EXIT(t->b_dphyctl);
		dsim_phy_write(id, DSIM_PHY_MD_TIME_CON4(i), val);

		val = DSIM_PHY_HSTX_CLK_SEL(hs_en) | DSIM_PHY_TLPX(t->lpx) |
			DSIM_PHY_TLP_EXIT_SKEW(0) | DSIM_PHY_TLP_ENTRY_SKEW(0);
		dsim_phy_write(id, DSIM_PHY_MD_TIME_CON0(i), val);

		/* skew cal implementation later */
		val = DSIM_PHY_THS_PREPARE(t->hs_prepare) |
			DSIM_PHY_THS_ZERO(t->hs_zero);
		dsim_phy_write(id, DSIM_PHY_MD_TIME_CON1(i), val);

		val = DSIM_PHY_THS_EXIT(t->hs_exit) |
			DSIM_PHY_THS_TRAIL(t->hs_trail);
		dsim_phy_write(id, DSIM_PHY_MD_TIME_CON2(i), val);

		val = DSIM_PHY_TTA_GET(3) | DSIM_PHY_TTA_GO(0);
		dsim_phy_write(id, DSIM_PHY_MD_TIME_CON3(i), val);

		/* add other clock lane setting if necessary */
	}
}

static void dsim_reg_set_dphy_param_dither(u32 id, const struct stdphy_pms *dphy_pms)
{
	u32 val, mask;

	/* MFR & MRR*/
	val = DSIM_PHY_DITHER_MFR(dphy_pms->mfr)
		| DSIM_PHY_DITHER_MRR(dphy_pms->mrr);
	dsim_phy_write(id, DSIM_PHY_PLL_CON3, val);

	/* SEL_PF & ICP */
	val = DSIM_PHY_DITHER_SEL_PF(dphy_pms->sel_pf)
		| DSIM_PHY_DITHER_ICP(dphy_pms->icp);
	mask = DSIM_PHY_DITHER_SEL_PF_MASK | DSIM_PHY_DITHER_ICP_MASK;
	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON5, val, mask);

	/* AFC_ENB & EXTAFC & FSEL * RSEL*/
	val = (DSIM_PHY_DITHER_AFC_ENB((dphy_pms->afc_enb) ? ~0 : 0))
		| DSIM_PHY_DITHER_EXTAFC(dphy_pms->extafc)
		| DSIM_PHY_DITHER_FSEL(((dphy_pms->fsel) ? ~0 : 0))
		| DSIM_PHY_DITHER_RSEL(dphy_pms->rsel);
	mask = DSIM_PHY_DITHER_AFC_ENB_MASK | DSIM_PHY_DITHER_EXTAFC_MASK
		| DSIM_PHY_DITHER_FSEL_MASK | DSIM_PHY_DITHER_RSEL_MASK;
	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON4, val, mask);

	/* FEED_EN */
	val = ((dphy_pms->feed_en) ? ~0 : 0) | ((dphy_pms->fout_mask) ? ~0 : 0);
	mask = DSIM_PHY_DITHER_FEED_EN_MASK | DSIM_PHY_DITHER_FOUT_MASK;
	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON2, val, mask);
}

/* BIAS Block Control Register */
static void dsim_reg_set_bias_con(u32 id, const u32 *blk_ctl)
{
	u32 i;

	for (i = 0; i < 5; i++)
		dsim_phy_extra_write(id, DSIM_PHY_BIAS_CON(i), blk_ctl[i]);
}

/* PLL Control Register */
static void dsim_reg_set_pll_con(u32 id, const u32 *blk_ctl)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(DSIM_PHY_PLL_CON_VAL); i++)
		dsim_phy_write(id, DSIM_PHY_PLL_CON(i), blk_ctl[i]);
}

/* Master Clock Lane General Control Register */
static void dsim_reg_set_mc_gnr_con(u32 id, const u32 *blk_ctl)
{
	u32 i;

	for (i = 0; i < 2; i++)
		dsim_phy_write(id, DSIM_PHY_MC_GNR_CON(i), blk_ctl[i]);
}

/* Master Clock Lane Analog Block Control Register */
static void dsim_reg_set_mc_ana_con(u32 id, const u32 *blk_ctl)
{
	u32 i;
	u32 reg_cnt = 2;

	for (i = 0; i < reg_cnt; i++)
		dsim_phy_write(id, DSIM_PHY_MC_ANA_CON(i), blk_ctl[i]);
}

/* Master Data Lane General Control Register */
static void dsim_reg_set_md_gnr_con(u32 id, const u32 *blk_ctl)
{
	u32 i;

	for (i = 0; i < MAX_DSIM_DATALANE_CNT; i++) {
		dsim_phy_write(id, DSIM_PHY_MD_GNR_CON0(i), blk_ctl[0]);
		dsim_phy_write(id, DSIM_PHY_MD_GNR_CON1(i), blk_ctl[1]);
	}
}

/* Master Data Lane Analog Block Control Register */
static void dsim_reg_set_md_ana_con(u32 id, const u32 *blk_ctl)
{
	u32 i;

	for (i = 0; i < MAX_DSIM_DATALANE_CNT; i++) {
		dsim_phy_write(id, DSIM_PHY_MD_ANA_CON0(i), blk_ctl[0]);
		dsim_phy_write(id, DSIM_PHY_MD_ANA_CON1(i), blk_ctl[1]);
		dsim_phy_write(id, DSIM_PHY_MD_ANA_CON2(i), blk_ctl[2]);
		dsim_phy_write(id, DSIM_PHY_MD_ANA_CON3(i), blk_ctl[3]);
	}
}

#ifdef DPDN_INV_SWAP
void dsim_reg_set_inv_dpdn(u32 id, u32 inv_clk, u32 inv_data[4])
{
	u32 i;
	u32 val, mask;

	val = inv_clk ? (DSIM_PHY_CLK_INV) : 0;
	mask = DSIM_PHY_CLK_INV;
	dsim_phy_write_mask(id, DSIM_PHY_MC_DATA_CON0, val, mask);

	for (i = 0; i < MAX_DSIM_DATALANE_CNT; i++) {
		val = inv_data[i] ? (DSIM_PHY_DATA_INV) : 0;
		mask = DSIM_PHY_DATA_INV;
		dsim_phy_write_mask(id,  DSIM_PHY_MD_DATA_CON0(i), val, mask);
	}
}

static void dsim_reg_set_dpdn_swap(u32 id, u32 clk_swap)
{
	u32 val, mask;

	val = DSIM_PHY_DPDN_SWAP(clk_swap);
	mask = DSIM_PHY_DPDN_SWAP_MASK;
	dsim_phy_write_mask(id, DSIM_PHY_MC_ANA_CON1, val, mask);
}
#endif

/******************* DSIM CAL functions *************************/
static void dsim_reg_sw_reset(u32 id)
{
	u32 val;
	int ret;

	dsim_write_mask(id, DSIM_SWRST, ~0, DSIM_SWRST_RESET);

	ret = readl_poll_timeout_atomic(dsim_regs_desc(id)->regs + DSIM_SWRST,
			val, !(val & DSIM_SWRST_RESET), 10, 2000);
	if (ret)
		cal_log_err(id, "%s is timeout.\n", __func__);
}

static void dsim_reg_set_config_options(u32 id, u32 lane,
		u32 eotp_en, u32 per_frame_read, u32 pix_format, u32 vc_id)
{
	u32 val, mask;

	val = DSIM_CONFIG_NUM_OF_DATA_LANE(lane) | DSIM_CONFIG_EOTP_EN(eotp_en)
		| DSIM_CONFIG_PER_FRAME_READ_EN(per_frame_read)
		| DSIM_CONFIG_PIXEL_FORMAT(pix_format)
		| DSIM_CONFIG_VC_ID(vc_id);

	mask = DSIM_CONFIG_NUM_OF_DATA_LANE_MASK | DSIM_CONFIG_EOTP_EN_MASK
		| DSIM_CONFIG_PER_FRAME_READ_EN_MASK
		| DSIM_CONFIG_PIXEL_FORMAT_MASK
		| DSIM_CONFIG_VC_ID_MASK;

	dsim_write_mask(id, DSIM_CONFIG, val, mask);
}

static void dsim_reg_enable_lane(u32 id, u32 lane, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_LANES_EN(lane));
}

static int dsim_reg_wait_phy_ready_clane(u32 id, bool en)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(
			dphy_regs_desc(id)->regs + DSIM_PHY_MC_GNR_CON0,
			val, (en == DSIM_PHY_PHY_READY_GET(val)), 10, 2000);
	if (ret) {
		cal_log_err(id, "PHY clock lane is not ready[timeout]\n");
		return ret;
	}

	return 0;
}

static int dsim_reg_wait_phy_ready_dlane(u32 id, u32 lane_id, bool en)
{
	u32 val, reg_id;
	int ret;

	reg_id = DSIM_PHY_MD_GNR_CON0(lane_id);

	ret = readl_poll_timeout_atomic(dphy_regs_desc(id)->regs + reg_id, val,
			(en == DSIM_PHY_PHY_READY_GET(val)), 10, 2000);
	if (ret) {
		cal_log_err(id, "PHY clock lane is not ready[timeout]\n");
		return ret;
	}

	return 0;
}

static int dsim_reg_enable_lane_phy(u32 id, u32 lane, bool en)
{
	u32 i, lane_cnt = 0;
	u32 reg_id;
	u32 ret = 0;
	u32 val = en ? ~0 : 0;

	/* check enabled data lane count */
	lane_cnt = hweight32(lane);

	/*
	 * [step1] enable phy_enable
	 */

	/* (1.1) clock lane on|off */
	reg_id = DSIM_PHY_MC_GNR_CON0;
	dsim_phy_write(id, reg_id, val);

	/* (1.2) data lane on|off */
	for (i = 0; i < lane_cnt; i++) {
		reg_id = DSIM_PHY_MD_GNR_CON0(i);
		dsim_phy_write(id, reg_id, val);
	}

	/*
	 * [step2] wait for phy_ready
	 */

	/* (2.1) check ready of clock lane */
	if (dsim_reg_wait_phy_ready_clane(id, en))
		ret++;

	/* (2.2) check ready of data lanes (index : from '1') */
	for (i = 0; i < lane_cnt; i++)
		if (dsim_reg_wait_phy_ready_dlane(id, i, en))
			ret++;

	if (ret) {
		cal_log_err(id, "Error to enable PHY lane(err=%d)\n", ret);
		return -EBUSY;
	} else
		return 0;
}

static void dsim_reg_pll_stable_time(u32 id, u32 lock_cnt)
{
	u32 val;

	val = DSIM_PHY_PLL_LOCK_CNT(lock_cnt);
	dsim_phy_write(id, DSIM_PHY_PLL_CON7, val);
}

static void dsim_reg_set_pll(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON0, val, DSIM_PHY_PLL_EN_MASK);
}

static bool dsim_reg_get_pll_en(u32 id)
{
	u32 val = dsim_phy_read(id, DSIM_PHY_PLL_CON0);

	return !!(val & DSIM_PHY_PLL_EN_MASK);
}

bool dsim_reg_is_pll_stable(u32 id)
{
	u32 val, pll_lock;

	val = dsim_phy_read(id, DSIM_PHY_PLL_STAT0);
	pll_lock = DSIM_PHY_PLL_LOCK_GET(val);

	return pll_lock != 0;
}

static int dsim_reg_enable_pll(u32 id, u32 en)
{
	u32 val;
	int ret;

	if (en)
		dsim_reg_clear_int(id, DSIM_INTSRC_PLL_STABLE);

	dsim_reg_set_pll(id, en);
	if (en ^ dsim_reg_get_pll_en(id)) {
		WARN(1, "dsim%u: PLL_EN is not %s\n",
				id, en ? "enable" : "disable");
		return -EINVAL;
	}
	ret = readl_poll_timeout_atomic(
			dphy_regs_desc(id)->regs + DSIM_PHY_PLL_STAT0,
			val, en == DSIM_PHY_PLL_LOCK_GET(val), 10, 2000);
	if (ret) {
		cal_log_err(id, "PHY %s failed[timeout]\n",
				en ? "enable" : "disable");
		return ret;
	}

	return 0;
}

static void dsim_reg_set_esc_clk_en(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CLK_CTRL, val, DSIM_CLK_CTRL_ESCCLK_EN);
}

static void dsim_reg_set_esc_clk_prescaler(u32 id, u32 en, u32 p)
{
	u32 val = en ? DSIM_CLK_CTRL_ESCCLK_EN : 0;
	u32 mask = DSIM_CLK_CTRL_ESCCLK_EN | DSIM_CLK_CTRL_ESC_PRESCALER_MASK;

	val |= DSIM_CLK_CTRL_ESC_PRESCALER(p);
	dsim_write_mask(id, DSIM_CLK_CTRL, val, mask);
}

static void dsim_reg_set_esc_clk_on_lane(u32 id, u32 en, u32 lane)
{
	u32 val;

	lane = (lane >> 1) | (1 << 4);

	val = en ? DSIM_CLK_CTRL_LANE_ESCCLK_EN(lane) : 0;
	dsim_write_mask(id, DSIM_CLK_CTRL, val,
				DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK);
}

static void dsim_reg_set_stop_state_cnt(u32 id)
{
	u32 val = DSIM_ESCMODE_STOP_STATE_CNT(DSIM_STOP_STATE_CNT);

	dsim_write_mask(id, DSIM_ESCMODE, val,
				DSIM_ESCMODE_STOP_STATE_CNT_MASK);
}

static void dsim_reg_set_timeout(u32 id)
{
	u32 val = DSIM_TIMEOUT_BTA_TOUT(DSIM_BTA_TIMEOUT)
		| DSIM_TIMEOUT_LPRX_TOUT(DSIM_LP_RX_TIMEOUT);

	dsim_write(id, DSIM_TIMEOUT, val);
}

static void dsim_reg_disable_hsa(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_HSA_DISABLE);
}

static void dsim_reg_disable_hbp(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_HBP_DISABLE);
}

static void dsim_reg_disable_hfp(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_HFP_DISABLE);
}

static void dsim_reg_disable_hse(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_HSE_DISABLE);
}

static void dsim_reg_set_burst_mode(u32 id, u32 burst)
{
	u32 val = burst ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_BURST_MODE);
}

static void dsim_reg_set_sync_inform(u32 id, u32 inform)
{
	u32 val = inform ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_SYNC_INFORM);
}

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
static void dsim_reg_set_pll_sleep_enable(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_PLL_SLEEP);
}
#endif

static void dsim_reg_set_vporch(u32 id, u32 vbp, u32 vfp)
{
	u32 val;

	val = DSIM_VPORCH_VFP_TOTAL(vfp) | DSIM_VPORCH_VBP(vbp);
	dsim_write(id, DSIM_VPORCH, val);
}

static void dsim_reg_set_vfp_detail(u32 id, u32 cmd_allow, u32 stable_vfp)
{
	u32 val = DSIM_VPORCH_VFP_CMD_ALLOW(cmd_allow)
		| DSIM_VPORCH_STABLE_VFP(stable_vfp);
	dsim_write(id, DSIM_VFP_DETAIL, val);
}

static void dsim_reg_set_hporch(u32 id, u32 hbp, u32 hfp)
{
	u32 val;

	val = DSIM_HPORCH_HFP(hfp) | DSIM_HPORCH_HBP(hbp);
	dsim_write(id, DSIM_HPORCH, val);
}

static void dsim_reg_set_sync_area(u32 id, u32 vsa, u32 hsa)
{
	u32 val = DSIM_SYNC_VSA(vsa) | DSIM_SYNC_HSA(hsa);

	dsim_write(id, DSIM_SYNC, val);
}

static void dsim_reg_set_resol(u32 id, struct dsim_reg_config *config)
{
	u32 height, width, val;

	width = config->p_timing.hactive;
	if (config->dsc.enabled)
		width = get_comp_dsc_width(&config->dsc) *
					config->dsc.slice_count;

	height = config->p_timing.vactive;

	val = DSIM_RESOL_VRESOL(height) | DSIM_RESOL_HRESOL(width);

	dsim_write(id, DSIM_RESOL, val);
}

static void dsim_reg_set_vresol(u32 id, u32 vresol)
{
	u32 val = DSIM_RESOL_VRESOL(vresol);

	dsim_write_mask(id, DSIM_RESOL, val, DSIM_RESOL_VRESOL_MASK);
}

static void dsim_reg_set_hresol(u32 id, u32 hresol,
		struct dsim_reg_config *config)
{
	u32 width = hresol;
	u32 val;

	if (config->dsc.enabled)
		width = get_comp_dsc_width(&config->dsc) *
					config->dsc.slice_count;

	val = DSIM_RESOL_HRESOL(width);

	dsim_write_mask(id, DSIM_RESOL, val, DSIM_RESOL_HRESOL_MASK);
}

static void dsim_reg_set_porch(u32 id, struct dsim_reg_config *config)
{
	if (config->mode == DSIM_VIDEO_MODE) {
		dsim_reg_set_vporch(id, config->p_timing.vbp,
				config->p_timing.vfp);
		dsim_reg_set_vfp_detail(id,
				DSIM_CMD_ALLOW_VALUE,
				DSIM_STABLE_VFP_VALUE);
		dsim_reg_set_hporch(id, config->p_timing.hfp,
				config->p_timing.hbp);
		dsim_reg_set_sync_area(id, config->p_timing.vsa,
				config->p_timing.hsa);
	}
}

static void dsim_reg_set_vt_htiming0(u32 id, u32 hsa_period, u32 hact_period)
{
	u32 val = DSIM_VT_HTIMING0_HSA_PERIOD(hsa_period)
		| DSIM_VT_HTIMING0_HACT_PERIOD(hact_period);

	dsim_write(id, DSIM_VT_HTIMING0, val);
}

static void dsim_reg_set_vt_htiming1(u32 id, u32 hfp_period, u32 hbp_period)
{
	u32 val = DSIM_VT_HTIMING1_HFP_PERIOD(hfp_period)
		| DSIM_VT_HTIMING1_HBP_PERIOD(hbp_period);

	dsim_write(id, DSIM_VT_HTIMING1, val);
}

static void dsim_reg_set_hperiod(u32 id, const struct dsim_reg_config *config,
		struct dsim_clks *clks)
{
	u32 vclk,  wclk;
	u32 hblank, vblank;
	u32 width, height;
	u32 hact_period, hsa_period, hbp_period, hfp_period;

	if (config->dsc.enabled)
		width = config->p_timing.hactive / 3;
	else
		width = config->p_timing.hactive;

	height = config->p_timing.vactive;

	if (config->mode == DSIM_VIDEO_MODE) {
		hblank = config->p_timing.hsa + config->p_timing.hbp +
			config->p_timing.hfp;
		vblank = config->p_timing.vsa + config->p_timing.vbp +
			config->p_timing.vfp;
		vclk = DIV_ROUND_CLOSEST((width + hblank) * (height + vblank) *
				config->p_timing.vrefresh, 1000000);
		wclk = DIV_ROUND_CLOSEST(clks->hs_clk, 16);

		/* round calculation to reduce fps error */
		hact_period = DIV_ROUND_CLOSEST(width * wclk, vclk);
		hsa_period = DIV_ROUND_CLOSEST(config->p_timing.hsa * wclk,
				vclk);
		hbp_period = DIV_ROUND_CLOSEST(config->p_timing.hbp * wclk,
				vclk);
		hfp_period = DIV_ROUND_CLOSEST(config->p_timing.hfp * wclk,
				vclk);

		dsim_reg_set_vt_htiming0(id, hsa_period, hact_period);
		dsim_reg_set_vt_htiming1(id, hfp_period, hbp_period);
	}
}

static void dsim_reg_set_video_mode(u32 id, u32 mode)
{
	u32 val = mode ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_VIDEO_MODE);
}

static int dsim_reg_wait_idle_status(u32 id, u32 is_vm)
{
	u32 val;
	int ret;

	if (!is_vm)
		return 0;

	ret = readl_poll_timeout_atomic(
			dsim_regs_desc(id)->regs + DSIM_LINK_STATUS0, val,
			!DSIM_LINK_STATUS0_VIDEO_MODE_STATUS_GET(val), 10,
			2000);
	if (ret) {
		cal_log_err(id, "dsim%d wait timeout idle status\n", id);
		return ret;
	}

	return 0;
}

/* 0 = command, 1 = video mode */
static u32 dsim_reg_get_display_mode(u32 id)
{
	u32 val;

	val = dsim_read(id, DSIM_CONFIG);
	return DSIM_CONFIG_DISPLAY_MODE_GET(val);
}

static void dsim_reg_enable_dsc(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CONFIG, val, DSIM_CONFIG_CPRS_EN);
}

static void dsim_reg_set_cprl_ctrl(u32 id, struct dsim_reg_config *config)
{
	u32 multi_slice = 1;
	u32 val;

	/* if multi-slice(2~4 slices) DSC compression is used in video mode
	 * MULTI_SLICE_PACKET configuration must be matched
	 * to DDI's configuration
	 */
	if (config->mode == DSIM_COMMAND_MODE)
		multi_slice = 1;
	else if (config->mode == DSIM_VIDEO_MODE)
		multi_slice = config->dsc.slice_count > 1 ? 1 : 0;

	/* if MULTI_SLICE_PACKET is enabled,
	 * only one packet header is transferred
	 * for multi slice
	 */
	val = DSIM_CPRS_CTRL_MULI_SLICE_PACKET(multi_slice)
		| DSIM_CPRS_CTRL_NUM_OF_SLICE(config->dsc.slice_count);

	dsim_write(id, DSIM_CPRS_CTRL, val);
}

static void dsim_reg_set_num_of_slice(u32 id, u32 num_of_slice)
{
	u32 val = DSIM_CPRS_CTRL_NUM_OF_SLICE(num_of_slice);

	dsim_write_mask(id, DSIM_CPRS_CTRL, val,
				DSIM_CPRS_CTRL_NUM_OF_SLICE_MASK);
}

static void dsim_reg_get_num_of_slice(u32 id, u32 *num_of_slice)
{
	u32 val = dsim_read(id, DSIM_CPRS_CTRL);

	*num_of_slice = DSIM_CPRS_CTRL_NUM_OF_SLICE_GET(val);
}

static void dsim_reg_set_multi_slice(u32 id, struct dsim_reg_config *config)
{
	bool multi_slice = true;
	u32 val;

	/* if multi-slice(2~4 slices) DSC compression is used in video mode
	 * MULTI_SLICE_PACKET configuration must be matched
	 * to DDI's configuration
	 */
	if (config->mode == DSIM_VIDEO_MODE)
		multi_slice = config->dsc.slice_count > 1;

	/* if MULTI_SLICE_PACKET is enabled,
	 * only one packet header is transferred
	 * for multi slice
	 */
	val = multi_slice ? ~0 : 0;
	dsim_write_mask(id, DSIM_CPRS_CTRL, val,
				DSIM_CPRS_CTRL_MULI_SLICE_PACKET_MASK);
}

static void dsim_reg_set_size_of_slice(u32 id, struct dsim_reg_config *config)
{
	u32 slice_w = config->dsc.slice_width;
	u32 val_01 = 0;
	u32 val_23 = 0;

	if (config->dsc.slice_count == 4) {
		val_01 = DSIM_SLICE01_SIZE_OF_SLICE1(slice_w) |
			DSIM_SLICE01_SIZE_OF_SLICE0(slice_w);
		val_23 = DSIM_SLICE23_SIZE_OF_SLICE3(slice_w) |
			DSIM_SLICE23_SIZE_OF_SLICE2(slice_w);

		dsim_write(id, DSIM_SLICE01, val_01);
		dsim_write(id, DSIM_SLICE23, val_23);
	} else if (config->dsc.slice_count == 2) {
		val_01 = DSIM_SLICE01_SIZE_OF_SLICE1(slice_w) |
			DSIM_SLICE01_SIZE_OF_SLICE0(slice_w);

		dsim_write(id, DSIM_SLICE01, val_01);
	} else if (config->dsc.slice_count == 1) {
		val_01 = DSIM_SLICE01_SIZE_OF_SLICE0(slice_w);

		dsim_write(id, DSIM_SLICE01, val_01);
	} else {
		cal_log_err(id, "not supported slice mode dsc(%d), slice(%d)\n",
				config->dsc.dsc_count, config->dsc.slice_count);
	}
}

static void dsim_reg_print_size_of_slice(u32 id)
{
	u32 val;
	u32 slice0_w, slice1_w, slice2_w, slice3_w;

	val = dsim_read(id, DSIM_SLICE01);
	slice0_w = DSIM_SLICE01_SIZE_OF_SLICE0_GET(val);
	slice1_w = DSIM_SLICE01_SIZE_OF_SLICE1_GET(val);

	val = dsim_read(id, DSIM_SLICE23);
	slice2_w = DSIM_SLICE23_SIZE_OF_SLICE2_GET(val);
	slice3_w = DSIM_SLICE23_SIZE_OF_SLICE3_GET(val);

	cal_log_debug(id, "slice0 w(%d) slice1 w(%d) slice2 w(%d) slice3(%d)\n",
			slice0_w, slice1_w, slice2_w, slice3_w);
}

static void dsim_reg_set_multi_packet_count(u32 id, u32 multipacketcnt)
{
	u32 val = DSIM_CMD_CONFIG_MULTI_PKT_CNT(multipacketcnt);

	dsim_write_mask(id, DSIM_CMD_CONFIG, val,
				DSIM_CMD_CONFIG_MULTI_PKT_CNT_MASK);
}

static void dsim_reg_set_cmd_te_ctrl0(u32 id, u32 stablevfp)
{
	u32 val = DSIM_CMD_TE_CTRL0_TIME_STABLE_VFP(stablevfp);

	dsim_write(id, DSIM_CMD_TE_CTRL0, val);
}

static void dsim_reg_set_cmd_te_ctrl1(u32 id, u32 teprotecton, u32 tetout)
{
	u32 val = DSIM_CMD_TE_CTRL1_TIME_TE_PROTECT_ON(teprotecton)
		| DSIM_CMD_TE_CTRL1_TIME_TE_TOUT(tetout);

	dsim_write(id, DSIM_CMD_TE_CTRL1, val);
}

static void dsim_reg_get_cmd_timer(unsigned int fps, u32 hs_clk,
		unsigned int *te_protect, unsigned int *te_timeout)
{
	*te_protect = hs_clk * (100 - TE_MARGIN) * 100 / fps / 16;
	*te_timeout = hs_clk * (100 + TE_MARGIN * 2) * 100 / fps / 16;
}

static void dsim_reg_set_cmd_ctrl(u32 id, const struct dsim_reg_config *config,
						struct dsim_clks *clks)
{
	unsigned int time_stable_vfp;
	unsigned int time_te_protect_on;
	unsigned int time_te_tout;

	if (config->dsc.enabled)
		time_stable_vfp = config->p_timing.hactive *
			DSIM_STABLE_VFP_VALUE / 100;
	else
		time_stable_vfp = config->p_timing.hactive *
				DSIM_STABLE_VFP_VALUE * 3 / 100;

	dsim_reg_get_cmd_timer(config->p_timing.vrefresh, clks->hs_clk,
		&time_te_protect_on, &time_te_tout);
	dsim_reg_set_cmd_te_ctrl0(id, time_stable_vfp);
	dsim_reg_set_cmd_te_ctrl1(id, time_te_protect_on, time_te_tout);
}

static void dsim_reg_enable_noncont_clock(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CLK_CTRL, val,
				DSIM_CLK_CTRL_NONCONT_CLOCK_LANE);
}

static void dsim_reg_enable_clocklane(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CLK_CTRL, val,
				DSIM_CLK_CTRL_CLKLANE_ONOFF);
}

void dsim_reg_enable_packetgo(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CMD_CONFIG, val, DSIM_CMD_CONFIG_PKT_GO_EN);
}

void dsim_reg_ready_packetgo(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CMD_CONFIG, val, DSIM_CMD_CONFIG_PKT_GO_RDY);
}

static void dsim_reg_enable_multi_cmd_packet(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CMD_CONFIG, val,
				DSIM_CMD_CONFIG_MULTI_CMD_PKT_EN);
}

/*
 * 0 = Disable shadow update (applied to Operating_SFR directly)
 * 1 = Enable shadow update (applied to Operating_SFR based on protocol)
 */
static void dsim_reg_enable_shadow(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_SFR_CTRL, val,
				DSIM_SFR_CTRL_SHADOW_EN);
}

static void dsim_reg_set_link_clock(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CLK_CTRL, val, DSIM_CLK_CTRL_CLOCK_SEL);
}

static void dsim_reg_enable_hs_clock(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CLK_CTRL, val, DSIM_CLK_CTRL_TX_REQUEST_HSCLK);
}

static void dsim_reg_enable_word_clock(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_CLK_CTRL, val, DSIM_CLK_CTRL_WORDCLK_EN);
}

static int dsim_reg_wait_hs_clk_ready(u32 id)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(
			dsim_regs_desc(id)->regs + DSIM_DPHY_STATUS,
			val, (val & DSIM_DPHY_STATUS_TX_READY_HSCLK), 10,
			2000);
	if (ret) {
		cal_log_err(id, "DSI Master is not HS state.\n");
		return ret;
	}

	return 0;
}

static int dsim_reg_wait_hs_clk_disable(u32 id)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(
			dsim_regs_desc(id)->regs + DSIM_DPHY_STATUS,
			val, (val & DSIM_DPHY_STATUS_STOPSTATE_CLK), 10, 2000);
	if (ret) {
		cal_log_err(id, "DSI Master clock isn't disabled.\n");
		return ret;
	}

	return 0;
}

static void dsim_reg_enter_ulps(u32 id, u32 enter)
{
	u32 val = enter ? ~0 : 0;
	u32 mask = DSIM_ESCMODE_TX_ULPS_CLK | DSIM_ESCMODE_TX_ULPS_DATA;

	dsim_write_mask(id, DSIM_ESCMODE, val, mask);
}

static void dsim_reg_exit_ulps(u32 id, u32 exit)
{
	u32 val = exit ? ~0 : 0;
	u32 mask = DSIM_ESCMODE_TX_ULPS_CLK_EXIT |
			DSIM_ESCMODE_TX_ULPS_DATA_EXIT;

	dsim_write_mask(id, DSIM_ESCMODE, val, mask);
}

static void dsim_reg_set_num_of_transfer(u32 id, u32 num_of_transfer)
{
	u32 val = DSIM_NUM_OF_TRANSFER_PER_FRAME(num_of_transfer);

	dsim_write(id, DSIM_NUM_OF_TRANSFER, val);
}

static void dsim_reg_set_deskew_hw(u32 id, u32 interval, u32 position)
{
	u32 val = DSIM_DESKEW_CTRL_HW_INTERVAL(interval)
		| DSIM_DESKEW_CTRL_HW_POSITION(position);
	u32 mask = DSIM_DESKEW_CTRL_HW_INTERVAL_MASK
		| DSIM_DESKEW_CTRL_HW_POSITION_MASK;

	dsim_write_mask(id, DSIM_DESKEW_CTRL, val, mask);
}

static void dsim_reg_enable_deskew_hw_enable(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_DESKEW_CTRL, val, DSIM_DESKEW_CTRL_HW_EN);
}

static void dsim_reg_set_cm_underrun_lp_ref(u32 id, u32 lp_ref)
{
	u32 val = DSIM_UNDERRUN_CTRL_CM_UNDERRUN_LP_REF(lp_ref);

	dsim_write(id, DSIM_UNDERRUN_CTRL, val);
}

static void dsim_reg_set_threshold(u32 id, u32 threshold)
{
	u32 val = DSIM_THRESHOLD_LEVEL(threshold);

	dsim_write(id, DSIM_THRESHOLD, val);

}

static void dsim_reg_set_vt_compensate(u32 id, u32 compensate)
{
	u32 val = DSIM_VIDEO_TIMER_COMPENSATE(compensate);

	if (dsim_reg_get_version(id) == DSIM_VER_EVT1)
		return;

	dsim_write_mask(id, DSIM_VIDEO_TIMER, val,
			DSIM_VIDEO_TIMER_COMPENSATE_MASK);
}

static void dsim_reg_set_vstatus_int(u32 id, u32 vstatus)
{
	u32 val = DSIM_VIDEO_TIMER_VSTATUS_INTR_SEL(vstatus);

	dsim_write_mask(id, DSIM_VIDEO_TIMER, val,
			DSIM_VIDEO_TIMER_VSTATUS_INTR_SEL_MASK);
}

static void dsim_reg_set_vt_sync_mode(u32 id, bool sync_mode)
{
	dsim_write_mask(id, DSIM_VIDEO_TIMER, sync_mode,
			DSIM_VIDEO_TIMER_SYNC_MODE);
}

static void dsim_reg_set_bist_te_interval(u32 id, u32 interval)
{
	u32 val = DSIM_BIST_CTRL0_BIST_TE_INTERVAL(interval);

	dsim_write_mask(id, DSIM_BIST_CTRL0, val,
			DSIM_BIST_CTRL0_BIST_TE_INTERVAL_MASK);
}

static void dsim_reg_set_bist_mode(u32 id, u32 bist_mode)
{
	u32 val = DSIM_BIST_CTRL0_BIST_PTRN_MODE(bist_mode);

	dsim_write_mask(id, DSIM_BIST_CTRL0, val,
			DSIM_BIST_CTRL0_BIST_PTRN_MODE_MASK);
}

static void dsim_reg_enable_bist_pattern_move(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_BIST_CTRL0, val,
			DSIM_BIST_CTRL0_BIST_PTRN_MOVE_EN);
}

static void dsim_reg_enable_bist(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_BIST_CTRL0, val, DSIM_BIST_CTRL0_BIST_EN);
}

static void dsim_set_hw_deskew(u32 id, u32 en)
{
	u32 hw_interval = 1;

	if (en) {
		/* 0 : VBP first line, 1 : VFP last line*/
		dsim_reg_set_deskew_hw(id, hw_interval, 0);
		dsim_reg_enable_deskew_hw_enable(id, en);
	} else {
		dsim_reg_enable_deskew_hw_enable(id, en);
	}
}

static int dsim_reg_wait_enter_ulps_state(u32 id, u32 lanes)
{
	u32 val, data_lanes;
	int ret;

	data_lanes = lanes >> DSIM_LANE_CLOCK;
	ret = readl_poll_timeout_atomic(
			dsim_regs_desc(id)->regs + DSIM_DPHY_STATUS, val,
			(DSIM_DPHY_STATUS_ULPS_DATA_LANE_GET(val) == data_lanes)
			&& (val & DSIM_DPHY_STATUS_ULPS_CLK), 10, 2000);
	if (ret) {
		cal_log_debug(id, "DSI Master is ULPS state.\n");
		return ret;
	}

	return 0;
}

static int dsim_reg_wait_exit_ulps_state(u32 id)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(
			dsim_regs_desc(id)->regs + DSIM_DPHY_STATUS, val,
			!(DSIM_DPHY_STATUS_ULPS_DATA_LANE_GET(val)) &&
			!(val & DSIM_DPHY_STATUS_ULPS_CLK), 10, 2000);
	if (ret) {
		cal_log_err(id, "DSI Master is not stop state.\n");
		return ret;
	}

	return 0;
}

static int dsim_reg_get_dphy_timing(u32 id, u32 hs_clk, u32 esc_clk,
		struct dphy_timing_value *t)
{
	int i;

	i = ARRAY_SIZE(dphy_timing) - 1;

	for (; i >= 0; i--) {
		if (dphy_timing[i][0] < hs_clk) {
			continue;
		} else {
			t->bps = hs_clk;
			t->clk_prepare = dphy_timing[i][1];
			t->clk_zero = dphy_timing[i][2];
			t->clk_post = dphy_timing[i][3];
			t->clk_trail = dphy_timing[i][4];
			t->hs_prepare = dphy_timing[i][5];
			t->hs_zero = dphy_timing[i][6];
			t->hs_trail = dphy_timing[i][7];
			t->lpx = dphy_timing[i][8];
			t->hs_exit = dphy_timing[i][9];
			break;
		}
	}

	if (i < 0) {
		cal_log_err(id, "can't find proper dphy timing(%u Mhz)\n",
				hs_clk);
		return -EINVAL;
	}

	cal_log_debug(id, "bps(%u) clk_prepare(%u) clk_zero(%u) clk_post(%u)\n",
			t->bps, t->clk_prepare, t->clk_zero, t->clk_post);
	cal_log_debug(id, "clk_trail(%u) hs_prepare(%u) hs_zero(%u)\n",
			t->clk_trail, t->hs_prepare, t->hs_zero);
	cal_log_debug(id, "hs_trail(%u) lpx(%u) hs_exit(%u)\n",
			t->hs_trail, t->lpx, t->hs_exit);

	if ((esc_clk > 20) || (esc_clk < 7)) {
		cal_log_err(id, "%u Mhz can't be used as escape clock\n",
				esc_clk);
		return -EINVAL;
	}

	t->b_dphyctl = b_dphyctl[esc_clk - 7];
	cal_log_debug(id, "b_dphyctl(%u)\n", t->b_dphyctl);

	return 0;
}

static void dsim_reg_set_config(u32 id, struct dsim_reg_config *config,
						struct dsim_clks *clks)
{
	u32 threshold;
	u32 num_of_slice;
	u32 num_of_transfer;
	int idx;

	/* shadow read disable */
	dsim_reg_enable_shadow_read(id, 1);

	if (config->mode == DSIM_VIDEO_MODE)
		dsim_reg_enable_clocklane(id, 0);
	else
		dsim_reg_enable_noncont_clock(id, 1);

	dsim_set_hw_deskew(id, 0); /* second param is to control enable bit */

	/* set bta & lpdr timeout vlaues */
	dsim_reg_set_timeout(id);

	dsim_reg_set_cmd_transfer_mode(id, 0);
	dsim_reg_set_stop_state_cnt(id);

	if (config->mode == DSIM_COMMAND_MODE) {
		/* DSU_MODE_1 is used in stead of 1 in MCD */
		idx = config->mres_mode;
		dsim_reg_set_cm_underrun_lp_ref(id,
				config->cmd_underrun_cnt[idx]);
	}

	threshold = config->p_timing.hactive;
	/* threshold is set as 1H. 1H is compressed width in case of DSC */
	if (config->dsc.enabled)
		threshold = get_comp_dsc_width(&config->dsc) *
					config->dsc.slice_count;

	dsim_reg_set_threshold(id, threshold);

	dsim_reg_set_resol(id, config);
	dsim_reg_set_porch(id, config);

	if (config->mode == DSIM_COMMAND_MODE) {
		if (config->dsc.enabled)
			/* use 1-line transfer only */
			num_of_transfer = config->p_timing.vactive;
		else
			num_of_transfer = config->p_timing.hactive *
				config->p_timing.vactive / threshold;

		dsim_reg_set_num_of_transfer(id, num_of_transfer);
	} else {
		num_of_transfer = config->p_timing.hactive *
			config->p_timing.vactive / threshold;
		dsim_reg_set_num_of_transfer(id, num_of_transfer);
	}

	/* set number of lanes, eotp enable, per_frame_read, pixformat, vc_id */
	dsim_reg_set_config_options(id, config->data_lane_cnt - 1, 1, 0,
			DSIM_PIXEL_FORMAT_RGB24, 0);

	/* CPSR & VIDEO MODE can be set when shadow enable on */
	/* shadow enable */
	dsim_reg_enable_shadow(id, 1);
	if (config->mode == DSIM_VIDEO_MODE) {
		dsim_reg_set_video_mode(id, DSIM_CONFIG_VIDEO_MODE);
		dsim_reg_set_hperiod(id, config, clks);
		cal_log_debug(id, "%s: video mode set\n", __func__);
	} else {
		dsim_reg_set_video_mode(id, 0);
		cal_log_debug(id, "%s: command mode set\n", __func__);
	}

	dsim_reg_enable_dsc(id, config->dsc.enabled);

	if (config->mode == DSIM_VIDEO_MODE) {
		dsim_reg_disable_hsa(id, 0);
		dsim_reg_disable_hbp(id, 0);
		dsim_reg_disable_hfp(id, 1);
		dsim_reg_disable_hse(id, 0);
		dsim_reg_set_burst_mode(id, 1);
		dsim_reg_set_sync_inform(id, 0);
		dsim_reg_enable_clocklane(id, 0);
	}

	if (config->dsc.enabled) {
		cal_log_debug(id, "%s: dsc configuration is set\n", __func__);
		dsim_reg_set_cprl_ctrl(id, config);
		dsim_reg_set_size_of_slice(id, config);

		dsim_reg_get_num_of_slice(id, &num_of_slice);
		cal_log_debug(id, "number of DSC slice(%d)\n", num_of_slice);
		dsim_reg_print_size_of_slice(id);
	}

	if (config->mode == DSIM_VIDEO_MODE) {
		dsim_reg_set_multi_packet_count(id, DSIM_PH_FIFOCTRL_THRESHOLD);
		dsim_reg_enable_multi_cmd_packet(id, 0);
	}
	dsim_reg_enable_packetgo(id, 0);

	if (config->mode == DSIM_COMMAND_MODE) {
		dsim_reg_set_cmd_ctrl(id, config, clks);
	} else if (config->mode == DSIM_VIDEO_MODE) {
		dsim_reg_set_vt_compensate(id, config->vt_compensation);
		dsim_reg_set_vstatus_int(id, DSIM_VSYNC);
		dsim_reg_set_vt_sync_mode(id, config->dual_dsi);
	}
}

static void dsim_reg_diag_apply(u32 id, u32 num_dphy_diag,
				struct dsim_dphy_diag *dphy_diags)
{
	int ret;
	u32 diag_ix, reg_ix, mask;
	struct dsim_dphy_diag *diag;

	for (diag_ix = 0; diag_ix < num_dphy_diag; ++diag_ix) {
		diag = &dphy_diags[diag_ix];
		if (!diag->override)
			continue;
		ret = dsim_dphy_diag_mask_from_range(diag->bit_start,
						     diag->bit_end, &mask);
		if (ret < 0)
			continue;
		if (diag->reg_base == REGS_DSIM_PHY) {
			for (reg_ix = 0; reg_ix < diag->num_reg; ++reg_ix) {
				dsim_phy_write_mask(id,
						    diag->reg_offset[reg_ix],
						    diag->user_value, mask);
			}
		} else if (diag->reg_base == REGS_DSIM_PHY_BIAS) {
			for (reg_ix = 0; reg_ix < diag->num_reg; ++reg_ix) {
				dsim_phy_extra_write_mask(
					id, diag->reg_offset[reg_ix],
					diag->user_value, mask);
			}
		}
	}
}

/*
 * configure and set DPHY PLL, byte clock, escape clock and hs clock
 *	- PMS value have to be optained by using PMS Gen.
 *      tool (MSC_PLL_WIZARD2_00.exe)
 *	- PLL out is source clock of HS clock
 *	- byte clock = HS clock / 16
 *	- calculate divider of escape clock using requested escape clock
 *	  from driver
 *	- DPHY PLL, byte clock, escape clock are enabled.
 *	- HS clock will be enabled another function.
 *
 * Parameters
 *	- hs_clk : in/out parameter.
 *		in :  requested hs clock. out : calculated hs clock
 *	- esc_clk : in/out parameter.
 *		in : requested escape clock. out : calculated escape clock
 *	- word_clk : out parameter. byte clock = hs clock / 16
 */

static int dsim_reg_set_clocks(u32 id, const struct dsim_reg_config *config,
			       struct dsim_clks *clks, u32 en)
{
	unsigned int esc_div;
	struct dsim_pll_param pll;
	struct dphy_timing_value t;
	u32 pll_lock_cnt;
	const struct stdphy_pms *dphy_pms;
	int ret = 0;
	u32 hsmode = 0;
#ifdef DPDN_INV_SWAP
	u32 inv_data[4] = {0, };
#endif

	if (en) {
		if (!config) {
			cal_log_err(id, "%s invalid config (null)\n", __func__);
			return -EINVAL;
		}

		/*
		 * Do not need to set clocks related with PLL,
		 * if DPHY_PLL is already stabled because of LCD_ON_UBOOT.
		 */
		if (dsim_reg_is_pll_stable(id)) {
			cal_log_info(id, "DPHY PLL is already stable\n");
			return -EBUSY;
		}

		/*
		 * set p, m, s to DPHY PLL
		 * PMS value has to be optained by PMS calculation tool
		 * released to customer
		 */
		dphy_pms = &config->dphy_pms;
		pll.p = dphy_pms->p;
		pll.m = dphy_pms->m;
		pll.s = dphy_pms->s;
		pll.k = dphy_pms->k;

		/* get word clock */
		/* clks ->hs_clk is from DT */
		clks->word_clk = clks->hs_clk / 16;
		cal_log_debug(id, "word clock is %u MHz\n", clks->word_clk);

		/* requeseted escape clock */
		cal_log_debug(id, "requested escape clock %u MHz\n",
				clks->esc_clk);

		/* escape clock divider */
		esc_div = clks->word_clk / clks->esc_clk;

		/* adjust escape clock */
		if ((clks->word_clk / esc_div) > clks->esc_clk)
			esc_div += 1;

		/* adjusted escape clock */
		clks->esc_clk = clks->word_clk / esc_div;
		cal_log_debug(id, "escape clock divider is 0x%x\n", esc_div);
		cal_log_debug(id, "escape clock is %u MHz\n", clks->esc_clk);

		/* set BIAS ctrl : default value */
		dsim_reg_set_bias_con(id, DSIM_PHY_BIAS_CON_VAL);

		/* set PLL ctrl : default value */
		dsim_reg_set_pll_con(id, DSIM_PHY_PLL_CON_VAL);

		if (clks->hs_clk < 1500)
			hsmode = 1;

		dsim_reg_set_esc_clk_prescaler(id, 1, esc_div);
		/* get DPHY timing values using hs clock and escape clock */
		dsim_reg_get_dphy_timing(id, clks->hs_clk, clks->esc_clk, &t);
		dsim_reg_set_dphy_timing_values(id, &t, hsmode);
		/* check dither sequence */
		if (dphy_pms->dither_en) {
			dsim_reg_set_dphy_param_dither(id, dphy_pms);
			dsim_reg_set_dphy_dither_en(id, 1);
		}

		/* set clock lane General Control Register control */
		dsim_reg_set_mc_gnr_con(id, DSIM_PHY_MC_GNR_CON_VAL);

		/* set clock lane Analog Block Control Register control */
		dsim_reg_set_mc_ana_con(id, DSIM_PHY_MC_ANA_CON_VAL);

#ifdef DPDN_INV_SWAP
		dsim_reg_set_dpdn_swap(id, 1);
#endif

		/* set data lane General Control Register control */
		dsim_reg_set_md_gnr_con(id, DSIM_PHY_MD_GNR_CON_VAL);

#ifdef DPDN_INV_SWAP
		inv_data[0] = 0;
		inv_data[1] = 1;
		inv_data[2] = 1;
		inv_data[3] = 0;
		dsim_reg_set_inv_dpdn(id, 0, inv_data);
#endif

		/* set data lane Analog Block Control Register control */
		dsim_reg_set_md_ana_con(id, DSIM_PHY_MD_ANA_CON_VAL);

		/* set PMSK on PHY */
		dsim_reg_set_pll_freq(id, pll.p, pll.m, pll.s, pll.k);

		/*set wclk buf sft cnt */
		dsim_reg_set_dphy_wclk_buf_sft(id, 3);

		/* set PLL's lock time (lock_cnt)
		 * PLL lock cnt setting guide
		 * PLL_LOCK_CNT_MUL = 500
		 * PLL_LOCK_CNT_MARGIN = 10 (10%)
		 * PLL lock time = PLL_LOCK_CNT_MUL * Tp
		 * Tp = 1 / (OSC clk / pll_p)
		 * PLL lock cnt = PLL lock time * OSC clk
		 */
		pll_lock_cnt = (PLL_LOCK_CNT_MUL + PLL_LOCK_CNT_MARGIN) * pll.p;
		dsim_reg_pll_stable_time(id, pll_lock_cnt);

#ifdef DPHY_LOOP
		dsim_reg_set_dphy_loop_test(id);
#endif
		dsim_reg_diag_apply(id, config->num_dphy_diags, config->dphy_diags);
		/* enable PLL */
		ret = dsim_reg_enable_pll(id, 1);
	} else {
		/* check disable PHY timing */
		/* TBD */
		dsim_reg_set_esc_clk_prescaler(id, 0, 0xff);

		/* check dither sequence */
		if (config != NULL && config->dphy_pms.dither_en)
			dsim_reg_set_dphy_dither_en(id, 0);

		dsim_reg_enable_pll(id, 0);
	}

	return ret;
}

static int dsim_reg_set_lanes(u32 id, u32 lanes, u32 en)
{
	/* LINK lanes */
	dsim_reg_enable_lane(id, lanes, en);

	return 0;
}

static int dsim_reg_set_lanes_dphy(u32 id, u32 lanes, bool en)
{
	/* PHY lanes */
	if (dsim_reg_enable_lane_phy(id, (lanes >> 1), en))
		return -EBUSY;

	return 0;
}

static u32 dsim_reg_is_noncont_clk_enabled(u32 id)
{
	int ret;

	ret = dsim_read_mask(id, DSIM_CLK_CTRL,
					DSIM_CLK_CTRL_NONCONT_CLOCK_LANE);
	return ret;
}

static int dsim_reg_set_hs_clock(u32 id, u32 en)
{
	int reg = 0;
	int is_noncont = dsim_reg_is_noncont_clk_enabled(id);

	if (en) {
		dsim_reg_enable_hs_clock(id, 1);
		if (!is_noncont)
			reg = dsim_reg_wait_hs_clk_ready(id);
	} else {
		dsim_reg_enable_hs_clock(id, 0);
		reg = dsim_reg_wait_hs_clk_disable(id);
	}
	return reg;
}

static void dsim_reg_set_int(u32 id, u32 en)
{
	u32 val = en ? 0 : ~0;
	u32 mask;

	/*
	 * TODO: underrun irq will be unmasked in the future.
	 * underrun irq(dsim_reg_set_config) is ignored in zebu emulator.
	 * it's not meaningful
	 */
	mask = DSIM_INTMSK_SW_RST_RELEASE | DSIM_INTMSK_SFR_PL_FIFO_EMPTY |
		DSIM_INTMSK_SFR_PH_FIFO_EMPTY |
		DSIM_INTMSK_FRAME_DONE | DSIM_INTMSK_INVALID_SFR_VALUE |
		DSIM_INTMSK_UNDER_RUN | DSIM_INTMSK_RX_DATA_DONE |
		DSIM_INTMSK_ERR_RX_ECC;

	dsim_write_mask(id, DSIM_INTMSK, val, mask);
}

/*
 * enter or exit ulps mode
 *
 * Parameter
 *	1 : enter ULPS mode
 *	0 : exit ULPS mode
 */
static int dsim_reg_set_ulps(u32 id, u32 en, u32 lanes)
{
	int ret = 0;

	if (en) {
		/* Enable ULPS clock and data lane */
		dsim_reg_enter_ulps(id, 1);

		/* Check ULPS request for data lane */
		ret = dsim_reg_wait_enter_ulps_state(id, lanes);
		if (ret)
			return ret;

	} else {
		/* Exit ULPS clock and data lane */
		dsim_reg_exit_ulps(id, 1);

		ret = dsim_reg_wait_exit_ulps_state(id);
		if (ret)
			return ret;

		/* wait at least 1ms : Twakeup time for MARK1 state  */
		udelay(1000);

		/* Clear ULPS exit request */
		dsim_reg_exit_ulps(id, 0);

		/* Clear ULPS enter request */
		dsim_reg_enter_ulps(id, 0);
	}

	return ret;
}

/*
 * enter or exit ulps mode for LSI DDI
 *
 * Parameter
 *	1 : enter ULPS mode
 *	0 : exit ULPS mode
 * assume that disp block power is off after ulps mode enter
 */
static int dsim_reg_set_smddi_ulps(u32 id, u32 en, u32 lanes)
{
	int ret = 0;

	if (en) {
		/* Enable ULPS clock and data lane */
		dsim_reg_enter_ulps(id, 1);

		/* Check ULPS request for data lane */
		ret = dsim_reg_wait_enter_ulps_state(id, lanes);
		if (ret)
			return ret;
		/* Clear ULPS enter request */
		dsim_reg_enter_ulps(id, 0);
	} else {
		/* Enable ULPS clock and data lane */
		dsim_reg_enter_ulps(id, 1);

		/* Check ULPS request for data lane */
		ret = dsim_reg_wait_enter_ulps_state(id, lanes);
		if (ret)
			return ret;

		/* Exit ULPS clock and data lane */
		dsim_reg_exit_ulps(id, 1);

		ret = dsim_reg_wait_exit_ulps_state(id);
		if (ret)
			return ret;

		/* wait at least 1ms : Twakeup time for MARK1 state */
		udelay(100);

		/* Clear ULPS enter request */
		dsim_reg_enter_ulps(id, 0);

		/* Clear ULPS exit request */
		dsim_reg_exit_ulps(id, 0);
	}

	return ret;
}

static int dsim_reg_set_ulps_by_ddi(u32 id, u32 ddi_type, u32 lanes, u32 en)
{
	int ret;

	switch (ddi_type) {
	case TYPE_OF_SM_DDI:
		ret = dsim_reg_set_smddi_ulps(id, en, lanes);
		break;
	case TYPE_OF_MAGNA_DDI:
		cal_log_err(id, "The ddi(%d) doesn't support ULPS\n", ddi_type);
		ret = -EINVAL;
		break;
	case TYPE_OF_NORMAL_DDI:
	default:
		ret = dsim_reg_set_ulps(id, en, lanes);
		break;
	}

	if (ret < 0)
		cal_log_err(id, "failed to %s ULPS", en ? "enter" : "exit");

	return ret;
}

static void dsim_reg_set_dphy_shadow_update_req(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_OPTION_SUITE, val,
			DSIM_OPTION_SUITE_CFG_UPDT_EN_MASK);
}

static void dsim_reg_set_dphy_use_shadow(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON2, val,
			DSIM_PHY_USE_SDW_MASK);
}

static void dsim_reg_set_dphy_pll_stable_cnt(u32 id, u32 cnt)
{
	u32 val = DSIM_PHY_PLL_STB_CNT(cnt);

	dsim_phy_write_mask(id, DSIM_PHY_PLL_CON8, val,
			DSIM_PHY_PLL_STB_CNT_MASK);
}

void dsim_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		enum dsim_regs_type type, unsigned int id)
{
	cal_regs_desc_check(type, id, REGS_DSIM_TYPE_MAX, MAX_DSI_CNT);
	cal_regs_desc_set(regs_desc, regs, start, name, type, id);
}

static void dpu_sysreg_select_dphy_rst_control(u32 id, u32 sel)
{
	u32 phy_num = id;
	u32 val = sel ? ~0 : 0;
	u32 mask = SEL_RESET_DPHY_MASK(phy_num);

	dsim_sys_write_mask(id, DISP_DPU_MIPI_PHY_CON, val, mask);
	cal_log_debug(id, "%s: phy_con_sel val=0x%x", __func__,
			dsim_sys_read(id, DISP_DPU_MIPI_PHY_CON));
}

#if !defined(CONFIG_BOARD_EMULATOR)
static void dpu_sysreg_dphy_reset(u32 id, u32 rst)
{
	u32 val = rst ? ~0 : 0;
	u32 mask = id ? M_RESETN_M1_MASK : M_RESETN_M0_MASK;

	dsim_sys_write_mask(id, DISP_DPU_MIPI_PHY_CON, val, mask);
}
#endif

/******************** EXPORTED DSIM CAL APIs ********************/

void dsim_reg_init(u32 id, struct dsim_reg_config *config,
		struct dsim_clks *clks, bool panel_ctrl)
{
	u32 lanes;

	/* DPHY reset control from SYSREG(0) */
	dpu_sysreg_select_dphy_rst_control(id, 0);

	lanes = DSIM_LANE_CLOCK | GENMASK(config->data_lane_cnt, 1);

	/* choose OSC_CLK */
	dsim_reg_set_link_clock(id, 0);

	dsim_reg_sw_reset(id);

	dsim_reg_set_lanes(id, lanes, 1);

	dsim_reg_set_esc_clk_on_lane(id, 1, lanes);

	dsim_reg_enable_word_clock(id, 1);

#if !defined(CONFIG_BOARD_EMULATOR)
	mutex_lock(&dphy_lock);
	/* Enable DPHY reset : DPHY reset start */
	dpu_sysreg_dphy_reset(id, 0);
#endif

#if !defined(CONFIG_BOARD_EMULATOR)
	dsim_reg_set_clocks(id, config, clks, 1);
	dsim_reg_set_lanes_dphy(id, lanes, true);
	dpu_sysreg_dphy_reset(id, 1); /* Release DPHY reset */
	mutex_unlock(&dphy_lock);
#endif

	dsim_reg_set_link_clock(id, 1);	/* Selection to word clock */

	dsim_reg_set_config(id, config, clks);

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
	dsim_reg_set_pll_sleep_enable(id, true); /* PHY pll sleep enable */
#endif
}

/* Set clocks and lanes and HS ready */
void dsim_reg_start(u32 id)
{
	dsim_reg_set_hs_clock(id, 1);
	dsim_reg_set_int(id, 1);
	dsim_reg_clear_int(id, 0xffffffff);
}

/* Unset clocks and lanes and stop_state */
int dsim_reg_stop(u32 id, u32 lanes)
{
	int err = 0;
	u32 is_vm;

	/* 0. wait the IDLE status */
	is_vm = dsim_reg_get_display_mode(id);
	err = dsim_reg_wait_idle_status(id, is_vm);
	if (err < 0)
		cal_log_err(id, "DSIM status is not IDLE!\n");

	dsim_reg_clear_int(id, 0xffffffff);
	/* disable interrupts */
	dsim_reg_set_int(id, 0);

	/* first disable HS clock */
	if (dsim_reg_set_hs_clock(id, 0) < 0)
		cal_log_err(id, "CLK lane doesn't be switched to LP mode\n");

	/* 1. clock selection : OSC */
	dsim_reg_set_link_clock(id, 0);

#if !defined(CONFIG_BOARD_EMULATOR)
	/* 2. master resetn */
	dpu_sysreg_dphy_reset(id, 0);
	/* 3. disable lane */
	dsim_reg_set_lanes_dphy(id, lanes, false);
#endif

	/* 4. turn off WORDCLK and ESCCLK */
	dsim_reg_set_esc_clk_on_lane(id, 0, lanes);
	dsim_reg_set_esc_clk_en(id, 0);

#if !defined(CONFIG_BOARD_EMULATOR)
	/* 5. disable PLL */
	dsim_reg_set_clocks(id, NULL, NULL, 0);
#endif

	if (err == 0)
		dsim_reg_sw_reset(id);

	return err;
}

/* Update DSIM reg for rr changed */
void dsim_reg_set_rr_config(u32 id, const struct dsim_reg_config *config,
		struct dsim_clks *clks)
{
	int idx;
	if (config->mode == DSIM_COMMAND_MODE) {
		idx = config->mres_mode;
		dsim_reg_set_cm_underrun_lp_ref(id, config->cmd_underrun_cnt[idx]);
		dsim_reg_set_cmd_ctrl(id, config, clks);
	} else if (config->mode == DSIM_VIDEO_MODE) {
		dsim_reg_set_hperiod(id, config, clks);
	}
}

/* Exit ULPS mode and set clocks and lanes */
int dsim_reg_exit_ulps_and_start(u32 id, u32 ddi_type, u32 lanes)
{
	int ret = 0;

	/* try to exit ULPS mode. The sequence is depends on DDI type */
	ret = dsim_reg_set_ulps_by_ddi(id, ddi_type, lanes, 0);
	dsim_reg_start(id);
	return ret;
}

/* Unset clocks and lanes and enter ULPS mode */
int dsim_reg_stop_and_enter_ulps(u32 id, u32 ddi_type, u32 lanes)
{
	int ret = 0;
	u32 is_vm;

	dsim_reg_clear_int(id, 0xffffffff);
	/* disable interrupts */
	dsim_reg_set_int(id, 0);

	/* 1. turn off clk lane & wait for stopstate_clk */
	ret = dsim_reg_set_hs_clock(id, 0);
	if (ret < 0)
		cal_log_err(id, "CLK lane doesn't be switched to LP mode\n");

	/* 2. enter to ULPS & wait for ulps state of clk and data */
	dsim_reg_set_ulps_by_ddi(id, ddi_type, lanes, 1);

	/* 3. sequence for BLK_DPU off */
	/* 3.1 wait idle */
	is_vm = dsim_reg_get_display_mode(id);
	ret = dsim_reg_wait_idle_status(id, is_vm);
	if (ret < 0)
		cal_log_err(id, "%s : DSIM_status is not IDLE!\n", __func__);
	/* 3.2 OSC clock */
	dsim_reg_set_link_clock(id, 0);
	/* 3.3 off DPHY */
	dsim_reg_set_lanes_dphy(id, lanes, false);
	dsim_reg_set_clocks(id, NULL, NULL, 0);
	/* 3.4 sw reset */
	dsim_reg_sw_reset(id);

	return ret;
}

int dsim_reg_get_int_and_clear(u32 id)
{
	u32 val;

	val = dsim_read(id, DSIM_INTSRC);
	dsim_reg_clear_int(id, val);

	return val;
}

void dsim_reg_clear_int(u32 id, u32 int_src)
{
	dsim_write(id, DSIM_INTSRC, int_src);
}

/*
 * @di: Data Identifier
 * @d0: Data0
 * @d1: Data1
 * @bta: Bus Turn Around
 */
void dsim_reg_wr_tx_header(u32 id, u8 di, u8 d0, u8 d1, bool bta)
{
	dsim_write(id, DSIM_PKTHDR, DSIM_PKTHDR_BTA_TYPE(bta) |
			DSIM_PKTHDR_ID(di) | DSIM_PKTHDR_DATA0(d0) |
			DSIM_PKTHDR_DATA1(d1));
}

void dsim_reg_wr_tx_payload(u32 id, u32 payload)
{
	dsim_write(id, DSIM_PAYLOAD, payload);
}

u32 dsim_reg_header_fifo_is_empty(u32 id)
{
	return dsim_read_mask(id, DSIM_FIFOCTRL, DSIM_FIFOCTRL_EMPTY_PH_SFR);
}

u32 dsim_reg_get_ph_cnt(u32 id)
{
	return DSIM_FIFOCTRL_NUMBER_OF_PH_SFR_GET(dsim_read(id, DSIM_FIFOCTRL));
}

u32 dsim_reg_payload_fifo_is_empty(u32 id)
{
	return dsim_read_mask(id, DSIM_FIFOCTRL, DSIM_FIFOCTRL_EMPTY_PL_SFR);
}

bool dsim_reg_has_pend_cmd(u32 id)
{
	return !dsim_reg_header_fifo_is_empty(id) ||
		!dsim_reg_payload_fifo_is_empty(id);
}

u32 dsim_reg_get_rx_fifo(u32 id)
{
	return dsim_read(id, DSIM_RXFIFO);
}

u32 dsim_reg_rx_fifo_is_empty(u32 id)
{
	return dsim_read_mask(id, DSIM_FIFOCTRL, DSIM_FIFOCTRL_EMPTY_RX);
}

int dsim_reg_rx_err_handler(u32 id, u32 rx_fifo)
{
	int ret = 0;
	u32 err_bit = rx_fifo >> 8; /* Error_Range [23:8] */

	if ((err_bit & MIPI_DSI_ERR_BIT_MASK) == 0) {
		cal_log_debug(id, "Non error reporting format (rx_fifo=0x%x)\n",
				rx_fifo);
		return ret;
	}

	/* Parse error report bit*/
	if (err_bit & MIPI_DSI_ERR_SOT)
		cal_log_err(id, "SoT error!\n");
	if (err_bit & MIPI_DSI_ERR_SOT_SYNC)
		cal_log_err(id, "SoT sync error!\n");
	if (err_bit & MIPI_DSI_ERR_EOT_SYNC)
		cal_log_err(id, "EoT error!\n");
	if (err_bit & MIPI_DSI_ERR_ESCAPE_MODE_ENTRY_CMD)
		cal_log_err(id, "Escape mode entry command error!\n");
	if (err_bit & MIPI_DSI_ERR_LOW_POWER_TRANSMIT_SYNC)
		cal_log_err(id, "Low-power transmit sync error!\n");
	if (err_bit & MIPI_DSI_ERR_HS_RECEIVE_TIMEOUT)
		cal_log_err(id, "HS receive timeout error!\n");
	if (err_bit & MIPI_DSI_ERR_FALSE_CONTROL)
		cal_log_err(id, "False control error!\n");
	if (err_bit & MIPI_DSI_ERR_ECC_SINGLE_BIT)
		cal_log_err(id, "ECC error, single-bit(detect and correct)\n");
	if (err_bit & MIPI_DSI_ERR_ECC_MULTI_BIT)
		cal_log_err(id, "ECC error, multi-bit(detect, not correct)\n");
	if (err_bit & MIPI_DSI_ERR_CHECKSUM)
		cal_log_err(id, "Checksum error(long packet only)!\n");
	if (err_bit & MIPI_DSI_ERR_DATA_TYPE_NOT_RECOGNIZED)
		cal_log_err(id, "DSI data type not recognized!\n");
	if (err_bit & MIPI_DSI_ERR_VCHANNEL_ID_INVALID)
		cal_log_err(id, "DSI VC ID invalid!\n");
	if (err_bit & MIPI_DSI_ERR_INVALID_TRANSMIT_LENGTH)
		cal_log_err(id, "Invalid transmission length!\n");

	cal_log_err(id, "(rx_fifo=0x%x) Check DPHY values about HS clk.\n",
			rx_fifo);
	return -EINVAL;
}

/*
 * 0 = Updated Register : operating
 * 1 = Shadow Register  : programming
 */
void dsim_reg_enable_shadow_read(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dsim_write_mask(id, DSIM_SFR_CTRL, val,
				DSIM_SFR_CTRL_SHADOW_REG_READ_EN);
}

void dsim_reg_function_reset(u32 id)
{
	u32 val;
	int ret;

	dsim_write_mask(id, DSIM_SWRST, ~0, DSIM_SWRST_FUNCRST);
	ret = readl_poll_timeout_atomic(dsim_regs_desc(id)->regs + DSIM_SWRST,
			val, !(val & DSIM_SWRST_FUNCRST), 10, 2000);
	if (ret)
		cal_log_err(id, "dsim%d function reset timeout\n", id);
}

/* Set porch and resolution to support Partial update */
void dsim_reg_set_partial_update(u32 id, struct dsim_reg_config *config)
{
	dsim_reg_set_vresol(id, config->p_timing.vactive);
	dsim_reg_set_hresol(id, config->p_timing.hactive, config);
	dsim_reg_set_porch(id, config);
	dsim_reg_set_num_of_transfer(id, config->p_timing.vactive);
}

void dsim_reg_set_mres(u32 id, struct dsim_reg_config *config)
{
	u32 threshold;
	u32 num_of_slice;
	u32 num_of_transfer;
	int idx;

	if (config->mode != DSIM_COMMAND_MODE) {
		cal_log_err(id, "mode[%d] doesn't support multi resolution\n",
				config->mode);
		return;
	}

	idx = config->mres_mode;
	dsim_reg_set_cm_underrun_lp_ref(id, config->cmd_underrun_cnt[idx]);

	if (config->dsc.enabled) {
		threshold = get_comp_dsc_width(&config->dsc) *
					config->dsc.slice_count;
		/* use 1-line transfer only */
		num_of_transfer = config->p_timing.vactive;
	} else {
		threshold = config->p_timing.hactive;
		num_of_transfer = config->p_timing.hactive *
			config->p_timing.vactive / threshold;
	}

	dsim_reg_set_threshold(id, threshold);
	dsim_reg_set_vresol(id, config->p_timing.vactive);
	dsim_reg_set_hresol(id, config->p_timing.hactive, config);
	dsim_reg_set_porch(id, config);
	dsim_reg_set_num_of_transfer(id, num_of_transfer);

	dsim_reg_enable_dsc(id, config->dsc.enabled);
	if (config->dsc.enabled) {
		cal_log_debug(id, "%s: dsc configuration is set\n", __func__);
		dsim_reg_set_num_of_slice(id, config->dsc.slice_count);
		dsim_reg_set_multi_slice(id, config); /* multi slice */
		dsim_reg_set_size_of_slice(id, config);

		dsim_reg_get_num_of_slice(id, &num_of_slice);
		cal_log_debug(id, "number of DSC slice(%d)\n", num_of_slice);
		dsim_reg_print_size_of_slice(id);
	}
}

void dsim_reg_set_bist(u32 id, bool en, u32 mode)
{
	if (en) {
		dsim_reg_set_bist_te_interval(id, 4505);
		dsim_reg_set_bist_mode(id, mode);
		dsim_reg_enable_bist_pattern_move(id, true);
	}

	dsim_reg_enable_bist(id, en);
}

void dsim_reg_set_cmd_transfer_mode(u32 id, u32 lp)
{
	u32 val = lp ? ~0 : 0;

	dsim_write_mask(id, DSIM_ESCMODE, val, DSIM_ESCMODE_CMD_LPDT);
}

/* Only for command mode */
void dsim_reg_set_dphy_freq_hopping(u32 id, u32 p, u32 m, u32 k, u32 en)
{
	u32 val, mask;
	u32 pll_stable_cnt = (PLL_SLEEP_CNT_MULT + PLL_SLEEP_CNT_MARGIN) * p;

	if (en) {
#if defined(CONFIG_EXYNOS_PLL_SLEEP)
		dsim_reg_set_pll_sleep_enable(id, false);
#endif
		dsim_reg_set_dphy_use_shadow(id, 1);
		dsim_reg_set_dphy_pll_stable_cnt(id, pll_stable_cnt);

		/* M value */
		val = DSIM_PHY_PMS_M(m);
		mask = DSIM_PHY_PMS_M_MASK;
		dsim_phy_write_mask(id, DSIM_PHY_PLL_CON2, val, mask);

		/* K value */
		val = DSIM_PHY_PMS_K(k);
		mask = DSIM_PHY_PMS_K_MASK;
		dsim_phy_write_mask(id, DSIM_PHY_PLL_CON1, val, mask);

		dsim_reg_set_dphy_shadow_update_req(id, 1);
	} else {
		dsim_reg_set_dphy_use_shadow(id, 0);
		dsim_reg_set_dphy_shadow_update_req(id, 0);
#if defined(CONFIG_EXYNOS_PLL_SLEEP)
		dsim_reg_set_pll_sleep_enable(id, true);
#endif
	}
}

#ifndef CONFIG_BOARD_EMULATOR
static void __dphy_dump(struct drm_printer *p, u32 id, struct dsim_regs *regs)
{
	cal_drm_printf(p, id, "=== DSIM %d DPHY SFR DUMP ===\n", id);
	/* DPHY dump */
	/* BIAS */
	cal_drm_printf(p, id, "-[BIAS]-\n");
	dpu_print_hex_dump(p, regs->phy_regs_ex,
			regs->phy_regs_ex + 0x0000, 0x14);
	/* PLL */
	cal_drm_printf(p, id, "-[PLL : offset + 0x100]-\n");
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x0000, 0x44);
	/* MC */
	cal_drm_printf(p, id, "-[MC : offset + 0x100]-\n");
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x0200, 0x48);
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x02E0, 0x10);
	/* MD0 */
	cal_drm_printf(p, id, "-[CMD 0 : offset + 0x100]-\n");
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x0300, 0x70);
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x03E0, 0x40);
	/* MD1 */
	cal_drm_printf(p, id, "-[CMD 1 : offset + 0x100]-\n");
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x0400, 0x70);
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x04C0, 0x40);
	/* MD2 */
	cal_drm_printf(p, id, "-[CMD 2 : offset + 0x100]-\n");
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x0500, 0x70);
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x05C0, 0x40);
	/* MD3 */
	cal_drm_printf(p, id, "-[MD 3 : offset + 0x100]-\n");
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x0600, 0x48);
	dpu_print_hex_dump(p, regs->phy_regs, regs->phy_regs + 0x06C0, 0x20);
}
#else
static inline void __dphy_dump(struct drm_printer *p, u32 id, struct dsim_regs *regs) { }
#endif

void __dsim_dump(struct drm_printer *p, u32 id, struct dsim_regs *regs)
{
	/* change to updated register read mode (meaning: SHADOW in DECON) */
	cal_drm_printf(p, id, "=== DSIM %d LINK SFR DUMP(applied to hw) ===\n", id);
	dsim_reg_enable_shadow_read(id, 0);
	dpu_print_hex_dump(p, regs->regs, regs->regs + 0x0000, 0x124);

	__dphy_dump(p, id, regs);

	/* restore to avoid size mismatch (possible config error at DECON) */
	cal_drm_printf(p, id, "=== DSIM %d LINK SFR DUMP ===\n", id);
	dsim_reg_enable_shadow_read(id, 1);
	dpu_print_hex_dump(p, regs->regs, regs->regs + 0x0000, 0x124);
}

int dsim_dphy_diag_mask_from_range(uint8_t start, uint8_t end, uint32_t *mask)
{
	if (start > end || end >= 32) {
		pr_err("%s: invalid bit range [%u, %u]\n", __func__, start,
		       end);
		return -EINVAL;
	}

	*mask = (((uint32_t)1 << (end - start + 1)) - 1) << start;
	return 0;
}

u32 diag_dsim_dphy_reg_read_mask(u32 id, u16 offset, u32 mask)
{
	return dsim_phy_read_mask(id, offset, mask);
}

u32 diag_dsim_dphy_extra_reg_read_mask(u32 id, u16 offset, u32 mask)
{
	return dsim_phy_extra_read_mask(id, offset, mask);
}

void dsim_reg_set_drm_write_protected(u32 id, bool write_protected)
{
	cal_set_write_protected(dsim_regs_desc(id), write_protected);
}
