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
#ifndef _GS101_UFS_VS_MMIO_H
#define _GS101_UFS_VS_MMIO_H

/* If not matched with ufs-cal-if, compling would fail */
#define UFS_VS_MMIO_VER 2

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

#define EXYNOS_UFS_MMIO_FUNC(name)					\
static inline void name##_writel(struct ufs_vs_handle *handle, u32 val, u32 ofs)	\
{									\
	writel(val, (u8 *)handle->name + ofs);				\
}									\
static inline u32 name##_readl(struct ufs_vs_handle *handle, u32 ofs)	\
{									\
	return readl((u8 *)handle->name + ofs);				\
}

EXYNOS_UFS_MMIO_FUNC(std);
EXYNOS_UFS_MMIO_FUNC(hci);
EXYNOS_UFS_MMIO_FUNC(ufsp);
EXYNOS_UFS_MMIO_FUNC(unipro);
EXYNOS_UFS_MMIO_FUNC(cport);

#if defined(__UFS_CAL_FW__)

#undef CLKSTOP_CTRL
#undef MPHY_APBCLK_STOP

#define	CLKSTOP_CTRL		0x11B0
#define	MPHY_APBCLK_STOP	(1<<3)

static inline void pma_writel(struct ufs_vs_handle *handle, u32 val, u32 ofs)
{
	u32 clkstop_ctrl = readl((u8 *)handle->hci + CLKSTOP_CTRL);

	writel(clkstop_ctrl & ~MPHY_APBCLK_STOP,
	       (u8 *)handle->hci + CLKSTOP_CTRL);
	writel(val, (u8 *)handle->pma + ofs);
	writel(clkstop_ctrl | MPHY_APBCLK_STOP,
	       (u8 *)handle->hci + CLKSTOP_CTRL);
}

static inline u32 pma_readl(struct ufs_vs_handle *handle, u32 ofs)
{
	u32 clkstop_ctrl = readl((u8 *)handle->hci + CLKSTOP_CTRL);
	u32 val;

	writel(clkstop_ctrl & ~MPHY_APBCLK_STOP,
	       (u8 *)handle->hci + CLKSTOP_CTRL);
	val = readl((u8 *)handle->pma + ofs);
	writel(clkstop_ctrl | MPHY_APBCLK_STOP,
	       (u8 *)handle->hci + CLKSTOP_CTRL);
	return val;
}
#else
EXYNOS_UFS_MMIO_FUNC(pma);
#endif

#endif /* _GS101_UFS_VS_MMIO_H */
