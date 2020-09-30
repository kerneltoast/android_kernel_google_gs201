/*
 * drivers/media/platform/exynos/mfc/mfc_core_otf.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_OTF_H
#define __MFC_CORE_OTF_H __FILE__

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
#include <media/exynos_tsmux.h>
#endif
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
#include <media/exynos_repeater.h>
#endif

#include "mfc_common.h"

#define MFC_OTF_DEFAULT_SCRATCH_SIZE	81920
#define MFC_OTF_DEFAULT_DPB_COUNT	3

extern struct mfc_dev *g_mfc_dev;

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
extern int repeater_request_buffer(struct shared_buffer_info *info, int owner);
extern int repeater_register_encode_cb(
	int (*repeater_encode_cb)(int, int, struct repeater_encoding_param *param));
extern int repeater_encoding_done(int encoding_ret);
#endif

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
extern int tsmux_packetize(struct packetizing_param *param);
extern int tsmux_encoding_start(int32_t index);
extern int tsmux_encoding_end(void);
extern void tsmux_sfr_dump(void);
extern void tsmux_set_es_size(unsigned int size);
#endif

enum hwfc_err {
	HWFC_ERR_NONE			= 0,
	HWFC_ERR_TSMUX			= 1,
	HWFC_ERR_MFC			= 2,
	HWFC_ERR_MFC_NOT_PREPARED	= 3,
	HWFC_ERR_MFC_TIMEOUT		= 4,
	HWFC_ERR_MFC_NOT_ENABLED		= 5,
};

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
int mfc_hwfc_encode(int buf_index, int job_id, struct repeater_encoding_param *param);
#endif

int mfc_core_otf_create(struct mfc_ctx *ctx);
void mfc_core_otf_destroy(struct mfc_ctx *ctx);
int mfc_core_otf_init(struct mfc_ctx *ctx);
void mfc_core_otf_deinit(struct mfc_ctx *ctx);
int mfc_core_otf_ctx_ready_set_bit(struct mfc_core_ctx *core_ctx, struct mfc_bits *data);
int mfc_core_otf_ctx_ready_clear_bit(struct mfc_core_ctx *core_ctx, struct mfc_bits *data);
int mfc_core_otf_run_enc_init(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_otf_run_enc_frame(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_otf_handle_seq(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_otf_handle_stream(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_otf_handle_error(struct mfc_core *core, struct mfc_ctx *ctx,
	unsigned int reason, unsigned int err);
void mfc_core_otf_path_test(struct mfc_ctx *ctx);

#endif /* __MFC_CORE_OTF_H  */
