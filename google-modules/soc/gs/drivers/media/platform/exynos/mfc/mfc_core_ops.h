/*
 * drivers/media/platform/exynos/mfc/mfc_core_ops.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_OPS_H
#define __MFC_CORE_OPS_H __FILE__

#include "mfc_common.h"

int mfc_core_instance_init(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_instance_deinit(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_instance_open(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_instance_cache_flush(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_instance_move_to(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_instance_move_from(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_instance_csd_parsing(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_instance_dpb_flush(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_instance_init_buf(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_instance_q_flush(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_instance_finishing(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_request_work(struct mfc_core *core, enum mfc_request_work work,
		struct mfc_ctx *ctx);
#if IS_ENABLED(CONFIG_EXYNOS_S2MPU)
int mfc_release_verify_fw(struct mfc_core *core);
#else
static inline int mfc_release_verify_fw(struct mfc_core *core) {return 0;}
#endif

#endif /* __MFC_CORE_OPS_H */
