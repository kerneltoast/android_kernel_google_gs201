/*
 * drivers/media/platform/exynos/mfc/mfc_mem.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_MEM_H
#define __MFC_MEM_H __FILE__

#if IS_ENABLED(CONFIG_MFC_USE_DMABUF_CONTAINER)
#include <linux/dma-buf-container.h>
#endif

#include "mfc_common.h"

/* Offset base used to differentiate between CAPTURE and OUTPUT
*  while mmaping */
#define DST_QUEUE_OFF_BASE      (TASK_SIZE / 2)

static inline dma_addr_t mfc_mem_get_daddr_vb(
	struct vb2_buffer *vb, unsigned int n)
{
	dma_addr_t addr = 0;

	addr = vb2_dma_sg_plane_dma_addr(vb, n);
	WARN_ON((addr == 0) || IS_ERR_VALUE(addr));

	return addr;
}

#if IS_ENABLED(CONFIG_MFC_USE_DMABUF_CONTAINER)
static inline int mfc_bufcon_get_buf_count(struct dma_buf *dmabuf)
{
	return dmabuf_container_get_count(dmabuf);
}
#else
static inline int mfc_bufcon_get_buf_count(struct dma_buf *dmabuf)
{
	return -1;
}
#endif

static inline void mfc_print_dpb_table(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_buf *mfc_buf = NULL;
	unsigned long flags;
	int i, found = 0, in_nal_q = 0;

	mfc_debug(3, "[DPB] dynamic_used: %#lx, queued: %#lx, table_used: %#lx\n",
		dec->dynamic_used, dec->queued_dpb, dec->dpb_table_used);
	for (i = 0; i < MFC_MAX_DPBS; i++) {
		found = 0;
		in_nal_q = 0;
		spin_lock_irqsave(&ctx->buf_queue_lock, flags);
		list_for_each_entry(mfc_buf, &ctx->dst_buf_queue.head, list) {
			if (i == mfc_buf->dpb_index) {
				found = 1;
				break;
			}
		}
		if (!found) {
			list_for_each_entry(mfc_buf,
					&ctx->dst_buf_nal_queue.head, list) {
				if (i == mfc_buf->dpb_index) {
					found = 1;
					in_nal_q = 1;
					break;
				}
			}
		}
		spin_unlock_irqrestore(&ctx->buf_queue_lock, flags);
		mfc_debug(3, "[%d] dpb [%d] %#010llx %#010llx (%s, %s, %s%s)\n",
				i, found ? mfc_buf->vb.vb2_buf.index : -1,
				dec->dpb[i].addr[0], dec->dpb[i].addr[1],
				dec->dpb[i].mapcnt ? "map" : "unmap",
				dec->dpb[i].ref ? "ref" : "free",
				dec->dpb[i].queued ? "Q" : "DQ",
				in_nal_q ? " in NALQ" : "");
	}
}

struct vb2_mem_ops *mfc_mem_ops(void);

void mfc_mem_set_cacheable(bool cacheable);
void mfc_mem_clean(struct mfc_dev *dev,
			struct mfc_special_buf *specail_buf,
			off_t offset, size_t size);
void mfc_mem_invalidate(struct mfc_dev *dev,
			struct mfc_special_buf *specail_buf,
			off_t offset, size_t size);

int mfc_mem_get_user_shared_handle(struct mfc_ctx *ctx,
		struct mfc_user_shared_handle *handle);
void mfc_mem_cleanup_user_shared_handle(struct mfc_ctx *ctx,
		struct mfc_user_shared_handle *handle);

int mfc_mem_ion_alloc(struct mfc_dev *dev,
		struct mfc_special_buf *special_buf);
void mfc_mem_ion_free(struct mfc_dev *dev,
		struct mfc_special_buf *special_buf);

void mfc_bufcon_put_daddr(struct mfc_ctx *ctx, struct mfc_buf *mfc_buf, int plane);
#if IS_ENABLED(CONFIG_MFC_USE_DMABUF_CONTAINER)
int mfc_bufcon_get_daddr(struct mfc_ctx *ctx, struct mfc_buf *mfc_buf,
			struct dma_buf *bufcon_dmabuf, int plane);
#else
int mfc_bufcon_get_daddr(struct mfc_ctx *ctx, struct mfc_buf *mfc_buf,
			struct dma_buf *bufcon_dmabuf, int plane)
{
	return 0;
}
#endif
void mfc_put_iovmm(struct mfc_ctx *ctx, struct dpb_table *dpb,
			int num_planes, int index);
void mfc_get_iovmm(struct mfc_ctx *ctx, struct vb2_buffer *vb,
			struct dpb_table *dpb);
void mfc_clear_iovmm(struct mfc_ctx *ctx, struct dpb_table *dpb,
			int num_planes, int index);
void mfc_cleanup_iovmm(struct mfc_ctx *ctx);
void mfc_cleanup_iovmm_except_used(struct mfc_ctx *ctx);
#endif /* __MFC_MEM_H */
