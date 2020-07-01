/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * drivers/media/platform/exynos/mfc/mfc_llc.h
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_LLC_H
#define __MFC_LLC_H __FILE__

#include "mfc_common.h"

/*
 * If SoC supports the LLC for MFC,
 * we should add definition the MFC_USES_LLC such as below.
 * #define MFC_USES_LLC
 */

#ifdef MFC_USES_LLC
void mfc_llc_enable(struct mfc_dev *dev);
void mfc_llc_disable(struct mfc_dev *dev);
void mfc_llc_flush(struct mfc_dev *dev);
#else
#define mfc_llc_enable(dev)	do {} while (0)
#define mfc_llc_disable(dev)	do {} while (0)
#define mfc_llc_flush(dev)	do {} while (0)
#endif

#endif /* __MFC_LLC_H */
