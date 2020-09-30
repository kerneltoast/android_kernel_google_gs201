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
 * we should enable from defconfig.
 */

#if IS_ENABLED(CONFIG_MFC_USES_LLC)
void mfc_llc_enable(struct mfc_core *core);
void mfc_llc_disable(struct mfc_core *core);
void mfc_llc_flush(struct mfc_core *core);
void mfc_llc_update_size(struct mfc_core *core, bool sizeup);
void mfc_llc_handle_resol(struct mfc_core *core, struct mfc_ctx *ctx);
#else
#define mfc_llc_enable(core)			do {} while (0)
#define mfc_llc_disable(core)			do {} while (0)
#define mfc_llc_flush(core)			do {} while (0)
#define mfc_llc_update_size(core, sizeup)	do {} while (0)
#define mfc_llc_handle_resol(core, ctx)		do {} while (0)
#endif

#endif /* __MFC_LLC_H */
