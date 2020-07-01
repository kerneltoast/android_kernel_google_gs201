/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * drivers/media/platform/exynos/mfc/mfc_slc.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_SLC_H
#define __MFC_SLC_H __FILE__

#include "mfc_common.h"

#ifdef CONFIG_SLC_PARTITION_MANAGER
void mfc_slc_enable(struct mfc_dev *dev);
void mfc_slc_disable(struct mfc_dev *dev);
void mfc_slc_flush(struct mfc_dev *dev);
void mfc_pt_resize_callback(void *data, int id, size_t resize_allocated);
#else
#define mfc_slc_enable(dev)	do {} while (0)
#define mfc_slc_disable(dev)	do {} while (0)
#define mfc_slc_flush(dev)	do {} while (0)
#endif

#endif /* __MFC_SLC_H */
