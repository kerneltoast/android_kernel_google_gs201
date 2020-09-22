/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 */

#ifndef __MFC_SLC_H
#define __MFC_SLC_H __FILE__

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
#include <soc/google/pt.h>
#endif

#include "mfc_common.h"

#define MFC_SLC_CMD_ATTR	0x4055100

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)

#define MFC_SLC_CMD_SYSREG_AX_CACHE		0xeeee
#define MFC_SLC_CMD_SSMT_AXI_XXX_SLC	0x80000000

void mfc_slc_enable(struct mfc_core *core);
void mfc_slc_disable(struct mfc_core *core);
void mfc_slc_flush(struct mfc_core *core);
void mfc_pt_resize_callback(void *data, int id, size_t resize_allocated);

void mfc_client_pt_register(struct mfc_core *core);
void mfc_client_pt_unregister(struct mfc_core *core);
#else
#define mfc_slc_enable(core)	do {} while (0)
#define mfc_slc_disable(core)	do {} while (0)
#define mfc_slc_flush(core)	do {} while (0)

#define mfc_client_pt_register(core) do {} while (0)
#define mfc_client_pt_unregister(core) do {} while (0)
#endif

#endif /* __MFC_SLC_H */
