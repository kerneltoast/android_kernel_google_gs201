/*
 * drivers/media/platform/exynos/mfc/mfc_core_run.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_RUN_H
#define __MFC_CORE_RUN_H __FILE__

#include "mfc_common.h"

int mfc_core_run_init_hw(struct mfc_core *core, int is_drm);
void mfc_core_run_deinit_hw(struct mfc_core *core);

int mfc_core_run_sleep(struct mfc_core *core);
int mfc_core_run_wakeup(struct mfc_core *core);

int mfc_core_run_dec_init(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_run_dec_frame(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_run_dec_last_frames(struct mfc_core *core, struct mfc_ctx *ctx);

int mfc_core_run_enc_init(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_run_enc_frame(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_run_enc_last_frames(struct mfc_core *core, struct mfc_ctx *ctx);

#endif /* __MFC_CORE_RUN_H */
