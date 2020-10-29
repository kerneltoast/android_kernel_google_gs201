/*
 * drivers/media/platform/exynos/mfc/mfc_buf.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_BUF_H
#define __MFC_BUF_H __FILE__

#include "mfc_common.h"

#define MFC_SECURE_FW_BASE	0x40000000

/* Memory allocation */
int mfc_alloc_common_context(struct mfc_core *core);
void mfc_release_common_context(struct mfc_core *core);

int mfc_alloc_instance_context(struct mfc_core_ctx *core_ctx);
void mfc_release_instance_context(struct mfc_core_ctx *core_ctx);

int mfc_alloc_codec_buffers(struct mfc_core_ctx *core_ctx);
void mfc_release_codec_buffers(struct mfc_core_ctx *core_ctx);
int mfc_alloc_scratch_buffer(struct mfc_core_ctx *core_ctx);
void mfc_release_scratch_buffer(struct mfc_core_ctx *core_ctx);

int mfc_alloc_enc_roi_buffer(struct mfc_core_ctx *core_ctx);
void mfc_release_enc_roi_buffer(struct mfc_core_ctx *core_ctx);

int mfc_otf_alloc_stream_buf(struct mfc_ctx *ctx);
void mfc_otf_release_stream_buf(struct mfc_ctx *ctx);

int mfc_alloc_metadata_buffer(struct mfc_ctx *ctx);
void mfc_release_metadata_buffer(struct mfc_ctx *ctx);

int mfc_alloc_firmware(struct mfc_core *core);
int mfc_load_firmware(struct mfc_core *core, struct mfc_special_buf *fw_buf,
		const u8 *fw_data, size_t fw_size);
int mfc_request_load_firmware(struct mfc_core *core, struct mfc_special_buf *fw_buf);
int mfc_release_firmware(struct mfc_core *core);

int mfc_alloc_dbg_info_buffer(struct mfc_core *core);
void mfc_release_dbg_info_buffer(struct mfc_core *core);

#endif /* __MFC_BUF_H */
