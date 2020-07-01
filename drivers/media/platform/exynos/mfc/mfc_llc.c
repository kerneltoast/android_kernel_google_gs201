// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/media/platform/exynos/mfc/mfc_llc.c
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifdef MFC_USES_LLC

#include <linux/module.h>
#include <soc/samsung/exynos-sci.h>

#include "mfc_llc.h"

void mfc_llc_enable(struct mfc_dev *dev)
{
	mfc_dev_debug_enter();

	if (llc_disable)
		return;

	llc_region_alloc(LLC_REGION_MFC_DISPLAY, 1);
	dev->llc_on_status = 1;
	mfc_dev_info("[LLC] enabled\n");
	MFC_TRACE_DEV("[LLC] enabled\n");

	mfc_dev_debug_leave();
}

void mfc_llc_disable(struct mfc_dev *dev)
{
	mfc_dev_debug_enter();

	llc_region_alloc(LLC_REGION_MFC_DISPLAY, 0);
	dev->llc_on_status = 0;
	mfc_dev_info("[LLC] disabled\n");
	MFC_TRACE_DEV("[LLC] disabled\n");

	mfc_dev_debug_leave();
}

void mfc_llc_flush(struct mfc_dev *dev)
{
	mfc_dev_debug_enter();

	if (llc_disable)
		return;

	llc_flush(MFC_DISPLAY_WAY);
	mfc_dev_debug(2, "[LLC] flushed\n");
	MFC_TRACE_DEV("[LLC] flushed\n");

	mfc_dev_debug_leave();
}
#endif
