/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Kiwoong <kwmad.kim@samsung.com>
 */

#ifndef _GS201_UFS_VS_MMIO_H
#define _GS201_UFS_VS_MMIO_H

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

static inline void std_writel(struct ufs_vs_handle *handle, u32 val, u32 ofs)
{
	writel(val, handle->std + ofs);
}

static inline u32 std_readl(struct ufs_vs_handle *handle, u32 ofs)
{
	return readl(handle->std + ofs);
}

static inline void hci_writel(struct ufs_vs_handle *handle, u32 val, u32 ofs)
{
	writel(val, handle->hci + ofs);
}

static inline u32 hci_readl(struct ufs_vs_handle *handle, u32 ofs)
{
	return readl(handle->hci + ofs);
}

static inline void unipro_writel(struct ufs_vs_handle *handle, u32 val, u32 ofs)
{
	writel(val, handle->unipro + ofs);
}

static inline u32 unipro_readl(struct ufs_vs_handle *handle, u32 ofs)
{
	return readl(handle->unipro + ofs);
}

static inline void cport_writel(struct ufs_vs_handle *handle, u32 val, u32 ofs)
{
	writel(val, handle->cport + ofs);
}

static inline u32 cport_readl(struct ufs_vs_handle *handle, u32 ofs)
{
	return readl(handle->cport + ofs);
}

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
static inline void pma_writel(struct ufs_vs_handle *handle, u32 val, u32 ofs)
{
	writel(val, handle->pma + ofs);
}

static inline u32 pma_readl(struct ufs_vs_handle *handle, u32 ofs)
{
	return readl(handle->pma + ofs);
}
#endif

#endif /* _GS201_UFS_VS_MMIO_H */
