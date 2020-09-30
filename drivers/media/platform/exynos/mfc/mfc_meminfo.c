/*
 * drivers/media/platform/exynos/mfc/mfc_meminfo.c
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "mfc_meminfo.h"

#include "mfc_rm.h"

#include "mfc_queue.h"

void __mfc_meminfo_add_buf(struct mfc_ctx *ctx, struct mfc_buf_queue *queue, struct vb2_buffer *vb)
{
	struct mfc_buf *buf = vb_to_mfc_buf(vb);
	struct mfc_mem *mfc_mem = NULL;
	unsigned long flags;
	int found = 0, plane;

	spin_lock_irqsave(&ctx->meminfo_queue_lock, flags);
	list_for_each_entry(mfc_mem, &queue->head, list) {
		if (mfc_mem->addr == buf->addr[0][0])
			found = 1;
	}
	spin_unlock_irqrestore(&ctx->meminfo_queue_lock, flags);

	if (!found) {
		mfc_mem = kzalloc(sizeof(struct mfc_mem), GFP_KERNEL);
		if (!mfc_mem)
			return;

		mfc_mem->addr = buf->addr[0][0];
		for (plane = 0; plane < vb->num_planes; ++plane) {
			mfc_mem->size += vb->planes[plane].length;
			mfc_debug(3, "plane[%d] size %u, mfc_mem size %zu\n", plane,
					vb->planes[plane].length, mfc_mem->size);
		}

		spin_lock_irqsave(&ctx->meminfo_queue_lock, flags);
		list_add_tail(&mfc_mem->list, &queue->head);
		queue->count++;
		spin_unlock_irqrestore(&ctx->meminfo_queue_lock, flags);
	}
}

void mfc_meminfo_add_inbuf(struct mfc_ctx *ctx, struct vb2_buffer *vb)
{
	struct mfc_buf_queue *queue  = &ctx->meminfo_inbuf_q;

	__mfc_meminfo_add_buf(ctx, queue, vb);
	mfc_debug(3, "[MEMINFO] input buffer count (%d)\n", queue->count);
}

void mfc_meminfo_add_outbuf(struct mfc_ctx *ctx, struct vb2_buffer *vb)
{
	struct mfc_buf_queue *queue  = &ctx->meminfo_outbuf_q;

	__mfc_meminfo_add_buf(ctx, queue, vb);
	mfc_debug(3, "[MEMINFO] output buffer count (%d)\n", queue->count);
}

void __mfc_meminfo_cleanup_queue(struct mfc_ctx *ctx, struct mfc_buf_queue *queue)
{
	struct mfc_mem *mfc_mem, *temp;
	unsigned long flags;

	mfc_debug(3, "[MEMINFO] clean up queue (%d)\n", queue->count);

	spin_lock_irqsave(&ctx->meminfo_queue_lock, flags);
	list_for_each_entry_safe(mfc_mem, temp, &queue->head, list) {
		list_del(&mfc_mem->list);
		kfree(mfc_mem);
	}

	mfc_init_queue(queue);
	spin_unlock_irqrestore(&ctx->meminfo_queue_lock, flags);
}

void mfc_meminfo_cleanup_inbuf_q(struct mfc_ctx *ctx)
{
	mfc_debug(3, "[MEMINFO] clean up inbuf_q\n");
	__mfc_meminfo_cleanup_queue(ctx, &ctx->meminfo_inbuf_q);
}

void mfc_meminfo_cleanup_outbuf_q(struct mfc_ctx *ctx)
{
	mfc_debug(3, "[MEMINFO] clean up outbuf_q\n");
	__mfc_meminfo_cleanup_queue(ctx, &ctx->meminfo_outbuf_q);
}

int mfc_meminfo_get_dev(struct mfc_dev *dev)
{
	struct mfc_ctx_buf_size *buf_size;
	int num = 0, i;

	dev->meminfo[num].type = MFC_MEMINFO_FW;
	dev->meminfo[num].name = "normal fw";
	dev->meminfo[num].count = 1;
	dev->meminfo[num].size = dev->variant->buf_size->firmware_code;
	dev->meminfo[num].total = dev->variant->buf_size->firmware_code;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	++num;
	dev->meminfo[num].type = MFC_MEMINFO_FW;
	dev->meminfo[num].name = "secure fw";
	dev->meminfo[num].count = 1;
	dev->meminfo[num].size = dev->variant->buf_size->firmware_code;
	dev->meminfo[num].total = dev->variant->buf_size->firmware_code;
#endif

	++num;
	dev->meminfo[num].type = MFC_MEMINFO_INTERNAL;
	dev->meminfo[num].name = "common ctx";
	dev->meminfo[num].count = 1;
	buf_size = dev->variant->buf_size->ctx_buf;
	dev->meminfo[num].size = buf_size->dev_ctx;
	dev->meminfo[num].total = buf_size->dev_ctx;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	++num;
	dev->meminfo[num].type = MFC_MEMINFO_INTERNAL;
	dev->meminfo[num].name = "secure common ctx";
	dev->meminfo[num].count = 1;
	buf_size = dev->variant->buf_size->ctx_buf;
	dev->meminfo[num].size = buf_size->dev_ctx;
	dev->meminfo[num].total = buf_size->dev_ctx;
#endif

	/*
	 * The dev total memory size is stored in dev->meminfo[6].
	 * If the number of dev->meminfo is over 6,
	 * MFC_MEMINFO_DEV_ALL should be changed.
	 */
	dev->meminfo[MFC_MEMINFO_DEV_ALL].total = 0;
	for (i = 0; i <= num; i++)
		dev->meminfo[MFC_MEMINFO_DEV_ALL].total += dev->meminfo[i].total;

	return ++num;
}

int __mfc_meminfo_get_dpb(struct mfc_ctx *ctx, int num)
{
	struct mfc_dec *dec = ctx->dec_priv;
	int i, cnt = 0;
	size_t size = 0, total = 0;

	mutex_lock(&dec->dpb_mutex);
	for (i = 0; i < MFC_MAX_DPBS; i++) {
		if (dec->dpb[i].mapcnt) {
			cnt++;
			total+= dec->dpb[i].size;
			if (size == 0)
				size = dec->dpb[i].size;
			mfc_debug(3, "dpb size [%d] %zu (%zx) / %zu\n",
					i, size, size, total);
		}
	}
	mutex_unlock(&dec->dpb_mutex);

	num++;
	ctx->meminfo[num].type = MFC_MEMINFO_OUTPUT;
	ctx->meminfo[num].name = "output";
	ctx->meminfo[num].count = cnt;
	ctx->meminfo[num].size = size;
	ctx->meminfo[num].total = total;

	return num;
}

#if 0
int mfc_meminfo_get_ctx(struct mfc_ctx *ctx)
{
	struct mfc_mem *mfc_mem = NULL;
	unsigned long flags;
	int num = 0, i;

	ctx->meminfo[num].type = MFC_MEMINFO_INTERNAL;
	ctx->meminfo[num].name = "instance buf";
	ctx->meminfo[num].count = 1;
	ctx->meminfo[num].size = ctx->instance_ctx_buf.size;
	ctx->meminfo[num].total = ctx->instance_ctx_buf.size;

	++num;
	ctx->meminfo[num].type = MFC_MEMINFO_INTERNAL;
	ctx->meminfo[num].name = "codec buf";
	ctx->meminfo[num].count = 1;
	ctx->meminfo[num].size = ctx->codec_buf.size;
	ctx->meminfo[num].total = ctx->codec_buf.size;

	if (ctx->type == MFCINST_ENCODER) {
		++num;
		ctx->meminfo[num].type = MFC_MEMINFO_INTERNAL;
		ctx->meminfo[num].name = "roi buf";
		ctx->meminfo[num].count = MFC_MAX_EXTRA_BUF;
		ctx->meminfo[num].size = ctx->enc_priv->roi_buf[0].size;
		ctx->meminfo[num].total =
			ctx->enc_priv->roi_buf[0].size * MFC_MAX_EXTRA_BUF;
	}


	/* get input buffer info */
	spin_lock_irqsave(&ctx->meminfo_queue_lock, flags);
	mfc_mem = list_entry(ctx->meminfo_inbuf_q.head.next, struct mfc_mem, list);
	spin_unlock_irqrestore(&ctx->meminfo_queue_lock, flags);
	if (mfc_mem != NULL) {
		++num;
		ctx->meminfo[num].type = MFC_MEMINFO_INPUT;
		ctx->meminfo[num].name = "input";
		ctx->meminfo[num].count = ctx->meminfo_inbuf_q.count;
		ctx->meminfo[num].size = mfc_mem->size;
		ctx->meminfo[num].total =
			mfc_mem->size * ctx->meminfo_inbuf_q.count;
	} else {
		mfc_ctx_err("list_entry is NULL\n");
	}

	/* get output buffer info */
	if (ctx->type == MFCINST_DECODER) {
		num = __mfc_meminfo_get_dpb(ctx, num);
	} else {
		spin_lock_irqsave(&ctx->meminfo_queue_lock, flags);
		mfc_mem = list_entry(ctx->meminfo_outbuf_q.head.next, struct mfc_mem, list);
		spin_unlock_irqrestore(&ctx->meminfo_queue_lock, flags);
		if (mfc_mem != 0) {
			++num;
			ctx->meminfo[num].type = MFC_MEMINFO_OUTPUT;
			ctx->meminfo[num].name = "output";
			ctx->meminfo[num].count = ctx->meminfo_outbuf_q.count;
			ctx->meminfo[num].size = mfc_mem->size;
			ctx->meminfo[num].total =
				mfc_mem->size * ctx->meminfo_outbuf_q.count;
		}
	}

	/* clear meminfo summary size */
	for (i = 0; i <= MFC_MEMINFO_CTX_MAX; i++)
		ctx->meminfo_size[i] = 0;

	/* update meminfo summary size */
	for (i = 0; i <= num; i++) {
		ctx->meminfo_size[MFC_MEMINFO_CTX_ALL] += ctx->meminfo[i].total;
		ctx->meminfo_size[MFC_MEMINFO_CTX_MAX] += ctx->meminfo[i].total;
		ctx->meminfo_size[ctx->meminfo[i].type] += ctx->meminfo[i].total;

		if (ctx->meminfo[i].type == MFC_MEMINFO_OUTPUT && ctx->type == MFCINST_DECODER) {
			ctx->meminfo_size[MFC_MEMINFO_CTX_MAX] -= ctx->meminfo[i].total;
			ctx->meminfo_size[MFC_MEMINFO_CTX_MAX] +=
				(ctx->dpb_count + MFC_NUM_EXTRA_DPB) * ctx->meminfo[i].size;
		}
	}

	return ++num;
}
#endif
