/*
 * drivers/media/platform/exynos/mfc/mfc_core_cmd.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_CMD_H
#define __MFC_CORE_CMD_H __FILE__

#include "mfc_common.h"

void mfc_core_cmd_sys_init(struct mfc_core *core,
				enum mfc_buf_usage_type buf_type);
void mfc_core_cmd_sleep(struct mfc_core *core);
void mfc_core_cmd_wakeup(struct mfc_core *core);

void mfc_core_cmd_open_inst(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_cmd_close_inst(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_cmd_abort_inst(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_cmd_move_inst(struct mfc_core *core, struct mfc_ctx *ctx);

void mfc_core_cmd_dpb_flush(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_cmd_cache_flush(struct mfc_core *core);

void mfc_core_cmd_dec_seq_header(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_cmd_enc_seq_header(struct mfc_core *core, struct mfc_ctx *ctx);

int mfc_core_cmd_dec_init_buffers(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_cmd_enc_init_buffers(struct mfc_core *core, struct mfc_ctx *ctx);

int mfc_core_cmd_dec_one_frame(struct mfc_core *core, struct mfc_ctx *ctx,
		int last_frame, int src_index);
void mfc_core_cmd_enc_one_frame(struct mfc_core *core, struct mfc_ctx *ctx,
		int last_frame);

#endif /* __MFC_CORE_CMD_H */
