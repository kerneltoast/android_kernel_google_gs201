// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/media/platform/exynos/mfc/mfc_slc.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
#include <../../../../drivers/soc/google/pt/pt.h>
#endif

#include "mfc_slc.h"

#include "mfc_reg_api.h"

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
void mfc_slc_enable(struct mfc_dev *dev)
{
	int i;

	mfc_dev_debug_enter();

	if (slc_disable)
		return;

	/*
	 * SSMT ALLOCATE_OVERRIDE set to BYPASS
	 * Cache hint is applied on its own by 3 step below.
	 * 1) set AxCACHE(0x404) in SYSREG
	 * 2) set AXI_xxx_SLC in SFRs
	 * 3) Firmware control
	 */
	MFC_SYSREG_WRITEL(0xeeee, 0x404);
	/* Stream ID used from 0 to 15, and reserved up to max 63 */
	for (i = 0; i < 16; i++) {
		MFC_SSMT0_WRITEL(0x80000000, 0x600 + 0x4 * i);
		MFC_SSMT0_WRITEL(0x80000000, 0x800 + 0x4 * i);
		MFC_SSMT1_WRITEL(0x80000000, 0x600 + 0x4 * i);
		MFC_SSMT1_WRITEL(0x80000000, 0x800 + 0x4 * i);
	}

	dev->ptid = pt_client_enable(dev->pt_handle, 0);

	/*
	 * SSMT PID settings for internal buffers
	 * stream AXI ID: 4, 6, 7, 8, 9, 13
	 * READ : base + 0x000 + (0x4 * ID)
	 * WRITE: base + 0x200 + (0x4 * ID)
	 */
	MFC_SSMT0_WRITEL(dev->ptid, 0x4 * 4);
	MFC_SSMT0_WRITEL(dev->ptid, 0x4 * 6);
	MFC_SSMT0_WRITEL(dev->ptid, 0x4 * 7);
	MFC_SSMT0_WRITEL(dev->ptid, 0x4 * 8);
	MFC_SSMT0_WRITEL(dev->ptid, 0x4 * 9);
	MFC_SSMT0_WRITEL(dev->ptid, 0x4 * 13);
	MFC_SSMT0_WRITEL(dev->ptid, 0x200 + 0x4 * 4);
	MFC_SSMT0_WRITEL(dev->ptid, 0x200 + 0x4 * 6);
	MFC_SSMT0_WRITEL(dev->ptid, 0x200 + 0x4 * 7);
	MFC_SSMT0_WRITEL(dev->ptid, 0x200 + 0x4 * 8);
	MFC_SSMT0_WRITEL(dev->ptid, 0x200 + 0x4 * 9);
	MFC_SSMT0_WRITEL(dev->ptid, 0x200 + 0x4 * 13);

	dev->slc_on_status = 1;
	mfc_dev_info("[SLC] enabled ptid: %d\n", dev->ptid);
	MFC_TRACE_DEV("[SLC] enabled\n");

	mfc_dev_debug_leave();
}

void mfc_slc_disable(struct mfc_dev *dev)
{
	mfc_dev_debug_enter();

	pt_client_disable(dev->pt_handle, 0);
	dev->slc_on_status = 0;
	mfc_dev_info("[SLC] disabled\n");
	MFC_TRACE_DEV("[SLC] disabled\n");

	mfc_dev_debug_leave();
}

void mfc_slc_flush(struct mfc_dev *dev)
{
	mfc_dev_debug_enter();

	if (slc_disable)
		return;

	mfc_slc_disable(dev);
	mfc_slc_enable(dev);

	mfc_dev_debug(2, "[SLC] flushed\n");
	MFC_TRACE_DEV("[SLC] flushed\n");

	mfc_dev_debug_leave();
}

void mfc_pt_resize_callback(void *data, int id, size_t resize_allocated)
{
	struct mfc_dev *dev = (struct mfc_dev *)data;

	if (resize_allocated < 512 * 1024) {
		mfc_dev_info("[SLC] available SLC size(%ld) is too small\n",
				resize_allocated);
		mfc_slc_disable(dev);
	}
}

void mfc_client_pt_register(struct mfc_dev *dev)
{

	mfc_dev_debug_enter();

	dev->pt_handle = pt_client_register(dev->device->of_node, (void *)dev,
		mfc_pt_resize_callback);
	if (!IS_ERR(dev->pt_handle)) {
		dev->has_slc = 1;
		mfc_dev_info("[SLC] PT Client Register success\n");
	} else {
		dev->pt_handle = NULL;
		dev->has_slc = 0;
		mfc_dev_info("[SLC] PT Client Register fail\n");
	}

	mfc_dev_debug_leave();
}

void mfc_client_pt_unregister(struct mfc_dev *dev)
{
	mfc_dev_debug_enter();

	if (dev->pt_handle) {
		dev->has_slc = 0;
		pt_client_unregister(dev->pt_handle);

		mfc_dev_info("[SLC] PT Client Unregister.\n");
	}

	mfc_dev_debug_leave();
}
#endif
