/*
 * drivers/media/platform/exynos/mfc/mfc_core_isr.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_ISR_H
#define __MFC_CORE_ISR_H __FILE__

#include <linux/interrupt.h>

#include "mfc_common.h"

#include "mfc_utils.h"

irqreturn_t mfc_core_top_half_irq(int irq, void *priv);
irqreturn_t mfc_core_irq(int irq, void *priv);
void mfc_core_handle_error(struct mfc_core *core);

static inline enum vb2_buffer_state __mfc_get_buf_state(unsigned int err)
{
	switch (err) {
	case MFC_REG_ERR_NO_KEY_FRAME:
	case MFC_REG_ERR_NO_VALID_SEQ_HDR:
	case MFC_REG_ERR_NO_VALID_PIC_HDR:
	case MFC_REG_ERR_NO_VALID_REF_FOR_SKIP:
	case MFC_REG_ERR_UNSUPPORTED_FEATURE:
	case MFC_REG_ERR_UNSUPPORTED_RESOLUTION:
	case MFC_REG_ERR_HEADER_NOT_FOUND:
	case MFC_REG_ERR_INVALID_NAL_TYPE:
	case MFC_REG_ERR_SEQUENCE_HEADER_ERROR:
	case MFC_REG_ERR_PICTURE_HEADER_ERROR:
	case MFC_REG_ERR_SLICE_HEADER_ERROR:
	case MFC_REG_ERR_MISSING_FIRST_FIELD:
	case MFC_REG_ERR_SLICE_COUNT_IS_OVER_ASO:
	case MFC_REG_ERR_TILE_HEADER_ERROR:
	case MFC_REG_ERR_MAX_VIEW_NUM_OVER:
	case MFC_REG_ERR_MFC_TIMEOUT:
		return VB2_BUF_STATE_ERROR;
	default:
		return VB2_BUF_STATE_DONE;
	}
}

static inline int __mfc_core_is_err_condition(unsigned int err)
{
	switch (err) {
	case MFC_REG_ERR_NO_AVAILABLE_DPB:
	case MFC_REG_ERR_INSUFFICIENT_DPB_SIZE:
	case MFC_REG_ERR_INSUFFICIENT_NUM_DPB:
	case MFC_REG_ERR_INSUFFICIENT_MV_BUF_SIZE:
	case MFC_REG_ERR_INSUFFICIENT_SCRATCH_BUF_SIZE:
	case MFC_REG_ERR_UNDEFINED_EXCEPTION:
		return 1;
	default:
		return 0;
	}
}

static inline void mfc_handle_force_change_status(struct mfc_core_ctx *core_ctx)
{
	struct mfc_ctx *ctx = core_ctx->ctx;

	if (core_ctx->state != MFCINST_ABORT && core_ctx->state != MFCINST_HEAD_PARSED &&
			core_ctx->state != MFCINST_RES_CHANGE_FLUSH) {
		mfc_change_state(core_ctx, MFCINST_RUNNING);
		if (IS_SWITCH_SINGLE_MODE(ctx)) {
			mfc_change_op_mode(ctx, MFC_OP_TWO_MODE2);
			mfc_debug(2, "[2CORE] reset 2core op_mode: %d\n", ctx->op_mode);
		}
	}
}

#endif /* __MFC_CORE_ISR_H */
