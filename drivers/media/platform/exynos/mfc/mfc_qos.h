/*
 * drivers/media/platform/exynos/mfc/mfc_qos.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_QOS_H
#define __MFC_QOS_H __FILE__

#include "mfc_common.h"

#define MFC_MIN_FPS			(30000)
#define MFC_MAX_FPS			(480000)
#define DEC_DEFAULT_FPS			(240000)
#define ENC_DEFAULT_FPS			(240000)
#define ENC_DEFAULT_CAM_CAPTURE_FPS	(60000)

void mfc_qos_update_framerate(struct mfc_ctx *ctx, u32 bytesused);
void mfc_qos_update_last_framerate(struct mfc_ctx *ctx, u64 timestamp);

static inline int __mfc_timespec64_compare(const struct timespec64 *lhs, const struct timespec64 *rhs)
{
	if (lhs->tv_sec < rhs->tv_sec)
		return -1;
	if (lhs->tv_sec > rhs->tv_sec)
		return 1;
	return lhs->tv_nsec - rhs->tv_nsec;
}

static inline void mfc_qos_reset_framerate(struct mfc_ctx *ctx)
{
	if (ctx->type == MFCINST_DECODER)
		ctx->framerate = DEC_DEFAULT_FPS;
	else if (ctx->type == MFCINST_ENCODER)
		ctx->framerate = ENC_DEFAULT_FPS;
}

static inline void mfc_qos_reset_last_framerate(struct mfc_ctx *ctx)
{
	ctx->last_framerate = 0;
}

static inline void mfc_qos_set_framerate(struct mfc_ctx *ctx, int rate)
{
	ctx->framerate = rate;
}

static inline int mfc_qos_get_framerate(struct mfc_ctx *ctx)
{
	return ctx->framerate;
}

#endif /* __MFC_QOS_H */
