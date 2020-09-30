/*
 * drivers/media/platform/exynos/mfc/mfc_core_intlock.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_INTLOCK_H
#define __MFC_CORE_INTLOCK_H __FILE__

#include "mfc_common.h"

int mfc_get_core_intlock(struct mfc_core_ctx *core_ctx);
void mfc_release_core_intlock(struct mfc_core_ctx *core_ctx);

#endif /* __MFC_CORE_INTLOCK_H */
