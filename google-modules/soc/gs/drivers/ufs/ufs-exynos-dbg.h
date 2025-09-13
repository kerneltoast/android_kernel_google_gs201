/* SPDX-License-Identifier: GPL-2.0-only */
//
// UFS Host Controller driver for Exynos specific extensions
//
// Copyright (C) 2022 Samsung Electronics Co., Ltd.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.

#ifndef _UFS_EXYNOS_DBG_H_
#define _UFS_EXYNOS_DBG_H_

enum {
	LOG_STD_HCI_SFR = 0xFFFFFFF0,
	LOG_VS_HCI_SFR,
	LOG_FMP_SFR,
	LOG_UNIPRO_SFR,
	LOG_PMA_SFR,
	LOG_INVALID,
};

enum {
	SFR_VAL_H_0_FIRST = 0,
	SFR_VAL_H_0,
	SFR_VAL_NUM,
};

enum {
	ATTR_VAL_H_0_L_0_FIRST = 0,
	ATTR_VAL_H_0_L_1_FIRST,
	ATTR_VAL_H_0_L_0,
	ATTR_VAL_H_0_L_1,
	ATTR_VAL_NUM,
};

enum {
	DBG_ATTR_UNIPRO = 0xEFFFFFF0,
	DBG_ATTR_PCS_CMN,
	DBG_ATTR_PCS_TX,
	DBG_ATTR_PCS_RX,
	DBG_ATTR_INVALID,
};

struct exynos_ufs_sfr_log {
	const char *name;
	const u32 offset;
	u32 val[SFR_VAL_NUM];
};

struct exynos_ufs_attr_log {
	const u32 mib;
	const u32 offset;
	/* 0: lane0, 1: lane1 */
	u32 val[ATTR_VAL_NUM];
};

#endif /* _UFS_EXYNOS_DBG_H_ */
