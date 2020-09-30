/*
 * drivers/media/platform/exynos/mfc/mfc_core_meerkat.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_MEERKAT_H
#define __MFC_CORE_MEERKAT_H __FILE__

#include "mfc_common.h"

void mfc_dump_state(struct mfc_dev *dev);
void mfc_core_meerkat_worker(struct work_struct *work);

#endif /* __MFC_CORE_MEERKAT_H */
