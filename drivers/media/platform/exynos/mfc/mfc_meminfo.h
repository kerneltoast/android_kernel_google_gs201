/*
 * drivers/media/platform/exynos/mfc/mfc_meminfo.h
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_MEMINFO_H
#define __MFC_MEMINFO_H __FILE__

#include "mfc_common.h"

void mfc_meminfo_add_inbuf(struct mfc_ctx *ctx, struct vb2_buffer *vb);
void mfc_meminfo_add_outbuf(struct mfc_ctx *ctx, struct vb2_buffer *vb);

void mfc_meminfo_cleanup_inbuf_q(struct mfc_ctx *ctx);
void mfc_meminfo_cleanup_outbuf_q(struct mfc_ctx *ctx);

int mfc_meminfo_get_dev(struct mfc_dev *dev);
//int mfc_meminfo_get_ctx(struct mfc_ctx *ctx);
#endif /* __MFC_MEMINFO_H */
