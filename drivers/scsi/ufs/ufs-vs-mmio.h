/* SPDX-License-Identifier: GPL-2.0-only */
//
// Copyright (C) 2019 Samsung Electronics Co., Ltd.
//
// Authors:
//	Kiwoong <kwmad.kim@samsung.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
#ifndef _UFS_VS_HANDLE_H_
#define _UFS_VS_HANDLE_H_

/* If not matched with ufs-cal-if, compling would fail */
#define UFS_VS_MMIO_VER 1

struct ufs_vs_handle {
	void *std;	/* Need to care conditions */
	void *hci;
	void *ufsp;
	void *unipro;
	void *pma;
	void *cport;
	void (*udelay)(u32 us);
	void *private;
};

/* Hardware */
#define UIC_ARG_MIB_SEL(attr, sel)		((((attr) & 0xFFFF) << 16) |\
						((sel) & 0xFFFF))
#define UIC_ARG_MIB(attr)			UIC_ARG_MIB_SEL(attr, 0)
#define PCS_CMN_OFFSET(ofs)			(0x2800 + ((ofs) - 0x200) * 4)
#define PCS_TRSV_OFFSET(ofs)			(0x2000 + (ofs) * 4)
#define PHY_PMA_COMN_ADDR(reg)			(reg)
#define PHY_PMA_TRSV_ADDR(reg, lane)		((reg) + (0x400 * (lane)))

#define UNIP_COMP_AXI_AUX_FIELD			0x040
#define __WSTRB					(0xF << 24)
#define __SEL_IDX(L)				((L) & 0xFFFF)

enum {
	TX_LANE_0 = 0,
	TX_LANE_1 = 1,
	TX_LANE_2 = 2,
	TX_LANE_3 = 3,
	RX_LANE_0 = 4,
	RX_LANE_1 = 5,
	RX_LANE_2 = 6,
	RX_LANE_3 = 7,
};

#define EXYNOS_UFS_MMIO_FUNC(name)					\
static inline void name##_writel(struct ufs_vs_handle *handle,		\
		u32 val, u32 ofs)					\
{									\
	writel(val, handle->name + ofs);				\
}									\
static inline u32 name##_readl(struct ufs_vs_handle *handle, u32 ofs)	\
{									\
	return readl(handle->name + ofs);				\
}

EXYNOS_UFS_MMIO_FUNC(std);
EXYNOS_UFS_MMIO_FUNC(hci);
EXYNOS_UFS_MMIO_FUNC(ufsp);
EXYNOS_UFS_MMIO_FUNC(unipro);
EXYNOS_UFS_MMIO_FUNC(pma);
EXYNOS_UFS_MMIO_FUNC(cport);

#endif /* _UFS_VS_HANDLE_H_ */
