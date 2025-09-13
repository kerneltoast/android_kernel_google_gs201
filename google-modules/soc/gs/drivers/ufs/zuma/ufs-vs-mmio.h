/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2022 Samsung Electronics Co., Ltd.
 */
#ifndef _UFS_VS_HANDLE_H_
#define _UFS_VS_HANDLE_H_

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

#undef MISC_CAL
#undef MPHY_APBCLK_CAL

#define	MISC_CAL		0x11B4
#define	MPHY_APBCLK_CAL	(1 << 10)

static inline void pma_writel(struct ufs_vs_handle *handle, u32 val, u32 ofs)
{
	u32 clkstop_ctrl = readl(handle->hci + MISC_CAL);

	writel(clkstop_ctrl & ~MPHY_APBCLK_CAL, handle->hci + MISC_CAL);
	writel(val, handle->pma + ofs);
	writel(clkstop_ctrl | MPHY_APBCLK_CAL, handle->hci + MISC_CAL);
}

static inline u32 pma_readl(struct ufs_vs_handle *handle, u32 ofs)
{
	u32 clkstop_ctrl = readl(handle->hci + MISC_CAL);
	u32 val;

	writel(clkstop_ctrl & ~MPHY_APBCLK_CAL, handle->hci + MISC_CAL);
	DSB;
	val = readl(handle->pma + ofs);
	DSB;
	writel(clkstop_ctrl | MPHY_APBCLK_CAL, handle->hci + MISC_CAL);
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

#endif /* _UFS_VS_HANDLE_H_ */
