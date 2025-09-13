/*
 * drivers/media/platform/exynos/mfc/mfc_core_reg_api.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>

#include "mfc_core_reg_api.h"

void mfc_core_enc_save_regression_result(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;

	if (regression_option & MFC_TEST_ENC_QP)
		dev->regression_val[dev->regression_cnt++] = MFC_CORE_READL(0xE004) & 0xFF;
	if (regression_option & MFC_TEST_DEFAULT) {
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_enc_slice_type();
		dev->regression_val[dev->regression_cnt++] = MFC_CORE_READL(0x609C);
		dev->regression_val[dev->regression_cnt++] = MFC_CORE_READL(0x2A54);
		dev->regression_val[dev->regression_cnt++] = MFC_CORE_READL(0x5080);
		dev->regression_val[dev->regression_cnt++] = (MFC_CORE_READL(0xE004) >> 8) & 0x3;
		dev->regression_val[dev->regression_cnt++] = 0xDEADC0DE;
	}
}

void mfc_core_dec_save_regression_result(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;

	if (regression_option & MFC_TEST_DEC_PER_FRAME)
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_dec_temporal_id();
	if (regression_option & MFC_TEST_DEFAULT) {
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_img_width();
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_img_height();
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_chroma_format();
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_profile();
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_level();
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_aspect_ratio();
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_luma_bit_depth_minus8();
		dev->regression_val[dev->regression_cnt++] = mfc_core_get_chroma_bit_depth_minus8();
		dev->regression_val[dev->regression_cnt++] = 0xDEADC0DE;
	}
}

void mfc_core_dbg_enable(struct mfc_core *core)
{
	mfc_core_debug(2, "MFC debug info enable\n");
	MFC_CORE_WRITEL(0x1, MFC_REG_DBG_INFO_ENABLE);
}

void mfc_core_dbg_disable(struct mfc_core *core)
{
	mfc_core_debug(2, "MFC debug info disable\n");
	MFC_CORE_WRITEL(0x0, MFC_REG_DBG_INFO_ENABLE);
}

void mfc_core_dbg_set_addr(struct mfc_core *core)
{
	struct mfc_ctx_buf_size *buf_size = core->dev->variant->buf_size->ctx_buf;

	memset((void *)core->dbg_info_buf.vaddr, 0, buf_size->dbg_info_buf);

	MFC_CORE_WRITEL(core->dbg_info_buf.daddr, MFC_REG_DBG_BUFFER_ADDR);
	MFC_CORE_WRITEL(buf_size->dbg_info_buf, MFC_REG_DBG_BUFFER_SIZE);
}

void mfc_core_otf_set_frame_addr(struct mfc_core *core, struct mfc_ctx *ctx,
		int num_planes)
{
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_buf_addr *buf_addr = &handle->otf_buf_addr;
	int index = handle->otf_buf_index;
	int i;

	for (i = 0; i < num_planes; i++) {
		mfc_debug(2, "[OTF][FRAME] set frame buffer[%d], 0x%08llx)\n",
				i, buf_addr->otf_daddr[index][i]);
		MFC_CORE_WRITEL(buf_addr->otf_daddr[index][i],
				MFC_REG_E_SOURCE_FIRST_ADDR + (i * 4));
	}
}

void mfc_core_otf_set_stream_size(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int size)
{
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_debug *debug = &handle->otf_debug;
	struct mfc_special_buf *buf;

	mfc_debug(2, "[OTF] set stream buffer full size, %u\n", size);
	MFC_CORE_WRITEL(size, MFC_REG_E_STREAM_BUFFER_SIZE);

	if (otf_dump && !ctx->is_drm) {
		buf = &debug->stream_buf[debug->frame_cnt];
		mfc_debug(2, "[OTF] set stream addr for debugging\n");
		mfc_debug(2, "[OTF][STREAM] buf[%d] daddr: 0x%08llx\n",
				debug->frame_cnt, buf->daddr);
		MFC_CORE_WRITEL(buf->daddr, MFC_REG_E_STREAM_BUFFER_ADDR);
	}
}

void mfc_core_otf_set_hwfc_index(struct mfc_core *core, struct mfc_ctx *ctx,
		int job_id)
{
	mfc_debug(2, "[OTF] set hwfc index, %d\n", job_id);
	HWFC_WRITEL(job_id);
}

void mfc_core_otf_set_votf_index(struct mfc_core *core, struct mfc_ctx *ctx,
		int job_id)
{
	mfc_debug(2, "[OTF] set vOTF index, %d\n", job_id);
	MFC_CORE_WRITEL(job_id, MFC_REG_E_SOURCE_VOTF_BUF_INDEX);
}

unsigned int mfc_get_frame_error_type(struct mfc_ctx *ctx, unsigned int err)
{
	struct mfc_dev *dev = ctx->dev;

	if (!err) {
		mfc_debug(4, "[FRAME] there is no error frame\n");
		return MFC_ERR_FRAME_NO_ERR;
	}

	if ((IS_VC1_RCV_DEC(ctx) && (mfc_get_warn(err) == MFC_REG_ERR_SYNC_POINT_NOT_RECEIVED))
			|| (mfc_get_warn(err) == MFC_REG_ERR_BROKEN_LINK)) {
		mfc_debug(2, "[FRAME] Broken frame error (%d)\n", mfc_get_warn(err));
		return MFC_ERR_FRAME_BROKEN;
	} else if (mfc_get_warn(err) == MFC_REG_ERR_SYNC_POINT_NOT_RECEIVED) {
		mfc_debug(2, "[FRAME] Sync point frame error (%d), type %d\n",
				mfc_get_warn(err), dev->pdata->display_err_type);
		return dev->pdata->display_err_type;
	}

	mfc_debug(2, "[FRAME] Concealment frame error (%d)\n", mfc_get_warn(err));
	return MFC_ERR_FRAME_CONCEALMENT;
}

/* Set decoding frame buffer */
int mfc_core_set_dec_codec_buffers(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	unsigned int i;
	size_t frame_size_mv;
	dma_addr_t buf_addr;
	int buf_size;
	int align_gap;
	struct mfc_raw_info *raw;
	unsigned int reg = 0;
	unsigned int av1_static_buf_size = 0;

	raw = &ctx->raw_buf;
	buf_addr = core_ctx->codec_buf.daddr;
	buf_size = core_ctx->codec_buf.size;

	mfc_debug(2, "[MEMINFO] codec buf 0x%llx size: %d\n", buf_addr, buf_size);
	mfc_debug(2, "Total DPB COUNT: %d, display delay: %d\n",
			dec->total_dpb_count, dec->display_delay);

	/* set decoder DPB size, stride */
	MFC_CORE_WRITEL(dec->total_dpb_count, MFC_REG_D_NUM_DPB);
	for (i = 0; i < raw->num_planes; i++) {
		mfc_debug(2, "[FRAME] buf[%d] size: %d, stride: %d\n",
				i, raw->plane_size[i], raw->stride[i]);
		MFC_CORE_WRITEL(raw->plane_size[i], MFC_REG_D_FIRST_PLANE_DPB_SIZE + (i * 4));
		MFC_CORE_WRITEL(ctx->raw_buf.stride[i],
				MFC_REG_D_FIRST_PLANE_DPB_STRIDE_SIZE + (i * 4));
		if (IS_2BIT_NEED(ctx)) {
			MFC_CORE_WRITEL(raw->stride_2bits[i], MFC_REG_D_FIRST_PLANE_2BIT_DPB_STRIDE_SIZE + (i * 4));
			MFC_CORE_WRITEL(raw->plane_size_2bits[i], MFC_REG_D_FIRST_PLANE_2BIT_DPB_SIZE + (i * 4));
			mfc_debug(2, "[FRAME]%s%s 2bits buf[%d] size: %d, stride: %d\n",
					(ctx->is_10bit ? "[10BIT]" : ""),
					(ctx->is_sbwc ? "[SBWC]" : ""),
					i, raw->plane_size_2bits[i], raw->stride_2bits[i]);
		}
	}

	/* set codec buffers */
	MFC_CORE_WRITEL(buf_addr, MFC_REG_D_SCRATCH_BUFFER_ADDR);
	MFC_CORE_WRITEL(ctx->scratch_buf_size, MFC_REG_D_SCRATCH_BUFFER_SIZE);
	buf_addr += ctx->scratch_buf_size;
	buf_size -= ctx->scratch_buf_size;

	if (IS_H264_DEC(ctx) || IS_H264_MVC_DEC(ctx) || IS_HEVC_DEC(ctx) || IS_BPG_DEC(ctx) || IS_AV1_DEC(ctx))
		MFC_CORE_WRITEL(ctx->mv_size, MFC_REG_D_MV_BUFFER_SIZE);

	if (IS_VP9_DEC(ctx)) {
		MFC_CORE_WRITEL(buf_addr, MFC_REG_D_STATIC_BUFFER_ADDR);
		MFC_CORE_WRITEL(DEC_STATIC_BUFFER_SIZE, MFC_REG_D_STATIC_BUFFER_SIZE);
		buf_addr += DEC_STATIC_BUFFER_SIZE;
		buf_size -= DEC_STATIC_BUFFER_SIZE;
	} else if (IS_AV1_DEC(ctx)) {
		av1_static_buf_size = DEC_AV1_STATIC_BUFFER_SIZE(ctx->img_width, ctx->img_height);
		MFC_CORE_WRITEL(buf_addr, MFC_REG_D_STATIC_BUFFER_ADDR);
		MFC_CORE_WRITEL(av1_static_buf_size, MFC_REG_D_STATIC_BUFFER_SIZE);
		buf_addr += av1_static_buf_size;
		buf_size -= av1_static_buf_size;
	}

	if (IS_MPEG4_DEC(ctx) && dec->loop_filter_mpeg4) {
		mfc_debug(2, "Add DPB for loop filter of MPEG4\n");
		for (i = 0; i < NUM_MPEG4_LF_BUF; i++) {
			MFC_CORE_WRITEL(buf_addr, MFC_REG_D_POST_FILTER_LUMA_DPB0 + (4 * i));
			buf_addr += ctx->loopfilter_luma_size;
			buf_size -= ctx->loopfilter_luma_size;

			MFC_CORE_WRITEL(buf_addr, MFC_REG_D_POST_FILTER_CHROMA_DPB0 + (4 * i));
			buf_addr += ctx->loopfilter_chroma_size;
			buf_size -= ctx->loopfilter_chroma_size;
		}
		reg |= ((dec->loop_filter_mpeg4 & MFC_REG_D_INIT_BUF_OPT_LF_CTRL_MASK)
				<< MFC_REG_D_INIT_BUF_OPT_LF_CTRL_SHIFT);
	}

	reg |= (0x1 << MFC_REG_D_INIT_BUF_OPT_DYNAMIC_DPB_SET_SHIFT);

	if (CODEC_NOT_CODED(ctx)) {
		reg |= (0x1 << MFC_REG_D_INIT_BUF_OPT_COPY_NOT_CODED_SHIFT);
		mfc_debug(2, "Notcoded frame copy mode start\n");
	}

	/* Enable 10bit Dithering when only display device is not support 10bit */
	if (dev->pdata->dithering_enable)
		reg |= (0x1 << MFC_REG_D_INIT_BUF_OPT_DITHERING_EN_SHIFT);
	if (ctx->is_10bit && !ctx->mem_type_10bit && !ctx->is_sbwc)
		/* 64byte align, It is vaid only for VP9 */
		reg |= (0x1 << MFC_REG_D_INIT_BUF_OPT_STRIDE_SIZE_ALIGN);
	else
		/* 16byte align, It is vaid only for VP9 */
		reg &= ~(0x1 << MFC_REG_D_INIT_BUF_OPT_STRIDE_SIZE_ALIGN);
	if (IS_VP9_DEC(ctx) && MFC_FEATURE_SUPPORT(dev, dev->pdata->vp9_stride_align)) {
		reg &= ~(0x3 << MFC_REG_D_INIT_BUF_OPT_STRIDE_SIZE_ALIGN);
		reg |= (0x2 << MFC_REG_D_INIT_BUF_OPT_STRIDE_SIZE_ALIGN);
	}
	reg &= ~(0x1 << MFC_REG_D_INIT_BUF_OPT_TWO_MODE_ENABLE_SHIFT);
	if (IS_MULTI_MODE(ctx)) {
		reg |= (1 << MFC_REG_D_INIT_BUF_OPT_TWO_MODE_ENABLE_SHIFT);
		mfc_debug(2, "[2CORE] two mode(op_mode: %d) enable\n",
				ctx->op_mode);
	}
	MFC_CORE_WRITEL(reg, MFC_REG_D_INIT_BUFFER_OPTIONS);

	mfc_debug(2, "[MEMINFO] codec buf remained size: %d\n", buf_size);
	if (buf_size < 0) {
		mfc_debug(2, "[MEMINFO] Not enough memory has been allocated\n");
		return -ENOMEM;
	}

	/* MV buffers */
	buf_addr = ctx->mv_buf.daddr;
	buf_size = ctx->mv_buf.size;
	mfc_debug(2, "[MEMINFO] MV buf 0x%llx size: %d\n", buf_addr, buf_size);

	frame_size_mv = ctx->mv_size;
	MFC_CORE_WRITEL(dec->mv_count, MFC_REG_D_NUM_MV);
	if (IS_H264_DEC(ctx) || IS_H264_MVC_DEC(ctx) || IS_HEVC_DEC(ctx) || IS_BPG_DEC(ctx) || IS_AV1_DEC(ctx)) {
		for (i = 0; i < dec->mv_count; i++) {
			align_gap = buf_addr;
			buf_addr = ALIGN(buf_addr, 16);
			align_gap = buf_addr - align_gap;
			buf_size -= align_gap;

			MFC_CORE_WRITEL(buf_addr, MFC_REG_D_MV_BUFFER0 + i * 4);
			buf_addr += frame_size_mv;
			buf_size -= frame_size_mv;
		}
	}

	mfc_debug(2, "[MEMINFO] MV buf remained size: %d\n", buf_size);
	if (buf_size < 0) {
		mfc_debug(2, "[MEMINFO] Not enough memory has been allocated\n");
		return -ENOMEM;
	}

	return 0;
}

/* Set encoding ref & codec buffer */
int mfc_core_set_enc_codec_buffers(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_enc *enc = ctx->enc_priv;
	dma_addr_t buf_addr;
	int buf_size;
	int i;

	mfc_debug_enter();

	buf_addr = core_ctx->codec_buf.daddr;
	buf_size = core_ctx->codec_buf.size;

	mfc_debug(2, "[MEMINFO] codec buf 0x%llx, size: %d\n", buf_addr, buf_size);
	mfc_debug(2, "DPB COUNT: %d\n", ctx->dpb_count);

	MFC_CORE_WRITEL(buf_addr, MFC_REG_E_SCRATCH_BUFFER_ADDR);
	MFC_CORE_WRITEL(ctx->scratch_buf_size, MFC_REG_E_SCRATCH_BUFFER_SIZE);
	buf_addr += ctx->scratch_buf_size;
	buf_size -= ctx->scratch_buf_size;

	/* start address of per buffer is aligned */
	for (i = 0; i < ctx->dpb_count; i++) {
		MFC_CORE_WRITEL(buf_addr, MFC_REG_E_LUMA_DPB + (4 * i));
		buf_addr += enc->luma_dpb_size;
		buf_size -= enc->luma_dpb_size;
	}
	for (i = 0; i < ctx->dpb_count; i++) {
		MFC_CORE_WRITEL(buf_addr, MFC_REG_E_CHROMA_DPB + (4 * i));
		buf_addr += enc->chroma_dpb_size;
		buf_size -= enc->chroma_dpb_size;
	}
	for (i = 0; i < ctx->dpb_count; i++) {
		MFC_CORE_WRITEL(buf_addr, MFC_REG_E_ME_BUFFER + (4 * i));
		buf_addr += enc->me_buffer_size;
		buf_size -= enc->me_buffer_size;
	}

	MFC_CORE_WRITEL(buf_addr, MFC_REG_E_TMV_BUFFER0);
	buf_addr += enc->tmv_buffer_size >> 1;
	MFC_CORE_WRITEL(buf_addr, MFC_REG_E_TMV_BUFFER1);
	buf_addr += enc->tmv_buffer_size >> 1;
	buf_size -= enc->tmv_buffer_size;

	mfc_debug(2, "[MEMINFO] codec buf 0x%llx, remained size: %d\n", buf_addr, buf_size);
	if (buf_size < 0) {
		mfc_debug(2, "[MEMINFO] Not enough memory has been allocated\n");
		return -ENOMEM;
	}

	mfc_debug_leave();

	return 0;
}

/* Set registers for decoding stream buffer */
int mfc_core_set_dec_stream_buffer(struct mfc_core *core, struct mfc_ctx *ctx,
		struct mfc_buf *mfc_buf, unsigned int start_num_byte,
		unsigned int strm_size)
{
	unsigned int cpb_buf_size = 0;
	dma_addr_t addr;
	size_t dbuf_size;
	struct vb2_buffer *vb = &mfc_buf->vb.vb2_buf;
	int index = -1;

	mfc_debug_enter();

	if (mfc_buf) {
		dbuf_size = vb->planes[0].dbuf->size;
		cpb_buf_size = ALIGN(strm_size + 511, STREAM_BUF_ALIGN);
		index = vb->index;
		addr = mfc_buf->addr[0][0];
		if (dbuf_size < cpb_buf_size) {
			mfc_ctx_info("Decrease buffer size: %u -> %zu\n",
					cpb_buf_size, dbuf_size);
			cpb_buf_size = dbuf_size;
		}
		mfc_debug(2, "[BUFINFO] ctx[%d] set src index: %d(%d), addr: 0x%08llx\n",
				ctx->num, index, mfc_buf->src_index, addr);
	} else {
		addr = 0;
	}

	mfc_debug(2, "[STREAM] strm_size: %#x(%d), buf_size: %u, offset: %u\n",
			strm_size, strm_size, cpb_buf_size, start_num_byte);

	if (strm_size == 0)
		mfc_ctx_info("stream size is 0\n");

	MFC_CORE_WRITEL(strm_size, MFC_REG_D_STREAM_DATA_SIZE);
	MFC_CORE_WRITEL(addr, MFC_REG_D_CPB_BUFFER_ADDR);
	MFC_CORE_WRITEL(cpb_buf_size, MFC_REG_D_CPB_BUFFER_SIZE);
	MFC_CORE_WRITEL(start_num_byte, MFC_REG_D_CPB_BUFFER_OFFSET);
	ctx->last_src_addr = addr;

	mfc_debug_leave();
	return 0;
}

void mfc_core_set_enc_frame_buffer(struct mfc_core *core, struct mfc_ctx *ctx,
		struct mfc_buf *mfc_buf, int num_planes)
{
	dma_addr_t addr[3] = { 0, 0, 0 };
	dma_addr_t addr_2bit[2] = { 0, 0 };
	int index, i;

	if (!mfc_buf) {
		mfc_debug(3, "enc zero buffer set\n");
		goto buffer_set;
	}

	index = mfc_buf->vb.vb2_buf.index;
	if (mfc_buf->num_valid_bufs > 0) {
		for (i = 0; i < num_planes; i++) {
			addr[i] = mfc_buf->addr[mfc_buf->next_index][i];
			mfc_debug(2, "[BUFCON][BUFINFO] ctx[%d] set src index:%d, batch[%d], addr[%d]: 0x%08llx\n",
					ctx->num, index, mfc_buf->next_index, i, addr[i]);
		}
		mfc_buf->next_index++;
	} else {
		for (i = 0; i < num_planes; i++) {
			addr[i] = mfc_buf->addr[0][i];
			mfc_debug(2, "[BUFINFO] ctx[%d] set src index:%d, addr[%d]: 0x%08llx\n",
					ctx->num, index, i, addr[i]);
		}
	}

buffer_set:
	for (i = 0; i < num_planes; i++)
		MFC_CORE_WRITEL(addr[i], MFC_REG_E_SOURCE_FIRST_ADDR + (i * 4));

	if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV12M_S10B ||
		ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV21M_S10B) {
		addr_2bit[0] = addr[0] + NV12N_10B_Y_8B_SIZE(ctx->img_width, ctx->img_height);
		addr_2bit[1] = addr[1] + NV12N_10B_CBCR_8B_SIZE(ctx->img_width, ctx->img_height);

		for (i = 0; i < num_planes; i++) {
			MFC_CORE_WRITEL(addr_2bit[i], MFC_REG_E_SOURCE_FIRST_2BIT_ADDR + (i * 4));
			mfc_debug(2, "[BUFINFO][10BIT] ctx[%d] set src 2bit addr[%d]: 0x%08llx\n",
					ctx->num, i, addr_2bit[i]);
		}
	} else if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV16M_S10B ||
		ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV61M_S10B) {
		addr_2bit[0] = addr[0] + NV16M_Y_SIZE(ctx->img_width, ctx->img_height);
		addr_2bit[1] = addr[1] + NV16M_CBCR_SIZE(ctx->img_width, ctx->img_height);

		for (i = 0; i < num_planes; i++) {
			MFC_CORE_WRITEL(addr_2bit[i], MFC_REG_E_SOURCE_FIRST_2BIT_ADDR + (i * 4));
			mfc_debug(2, "[BUFINFO][10BIT] ctx[%d] set src 2bit addr[%d]: 0x%08llx\n",
					ctx->num, i, addr_2bit[i]);
		}
	} else if (ctx->is_sbwc && !ctx->is_10bit) {
		addr_2bit[0] = addr[0] + SBWC_8B_Y_SIZE(ctx->img_width, ctx->img_height);
		addr_2bit[1] = addr[1] + SBWC_8B_CBCR_SIZE(ctx->img_width, ctx->img_height);

		for (i = 0; i < num_planes; i++) {
			MFC_CORE_WRITEL(addr_2bit[i], MFC_REG_E_SOURCE_FIRST_2BIT_ADDR + (i * 4));
			mfc_debug(2, "[BUFINFO][SBWC] ctx[%d] set src header addr[%d]: 0x%08llx\n",
				ctx->num, i, addr_2bit[i]);
		}
	} else if (ctx->is_sbwc && ctx->is_10bit) {
		addr_2bit[0] = addr[0] + SBWC_10B_Y_SIZE(ctx->img_width, ctx->img_height);
		addr_2bit[1] = addr[1] + SBWC_10B_CBCR_SIZE(ctx->img_width, ctx->img_height);

		for (i = 0; i < num_planes; i++) {
			MFC_CORE_WRITEL(addr_2bit[i], MFC_REG_E_SOURCE_FIRST_2BIT_ADDR + (i * 4));
			mfc_debug(2, "[BUFINFO][10BIT][SBWC] ctx[%d] set src header addr[%d]: 0x%08llx\n",
				ctx->num, i, addr_2bit[i]);
		}
	}
}

/* Set registers for encoding stream buffer */
int mfc_core_set_enc_stream_buffer(struct mfc_core *core, struct mfc_ctx *ctx,
		struct mfc_buf *mfc_buf)
{
	dma_addr_t addr = 0;
	unsigned int size = 0, offset = 0, index = -1;

	if (mfc_buf) {
		index = mfc_buf->vb.vb2_buf.index;
		addr = mfc_buf->addr[0][0];
		offset = mfc_buf->vb.vb2_buf.planes[0].data_offset;
		size = (unsigned int)vb2_plane_size(&mfc_buf->vb.vb2_buf, 0);
		size = ALIGN(size, STREAM_BUF_ALIGN);
	} else {
		/*
		 * When LAST_SEQ of B frame encoding
		 * if there is no output buffer, set addr and size with 0xffffffff
		 * and then FW returns COMPLETE_SEQ.
		 */
		addr = 0xffffffff;
		size = 0xffffffff;
	}

	MFC_CORE_WRITEL(addr, MFC_REG_E_STREAM_BUFFER_ADDR); /* 16B align */
	MFC_CORE_WRITEL(size, MFC_REG_E_STREAM_BUFFER_SIZE);
	MFC_CORE_WRITEL(offset, MFC_REG_E_STREAM_BUFFER_OFFSET);

	mfc_debug(2, "[BUFINFO] ctx[%d] set dst index: %d, addr: 0x%08llx\n",
			ctx->num, index, addr);
	mfc_debug(2, "[STREAM] buf_size: %u, offset: %d\n", size, offset);

	return 0;
}

void mfc_core_get_enc_frame_buffer(struct mfc_core *core, struct mfc_ctx *ctx,
		dma_addr_t addr[], int num_planes)
{
	unsigned long enc_recon_y_addr, enc_recon_c_addr;
	int i, addr_offset;

	addr_offset = MFC_REG_E_ENCODED_SOURCE_FIRST_ADDR;

	for (i = 0; i < num_planes; i++)
		addr[i] = MFC_CORE_READL(addr_offset + (i * 4));

	enc_recon_y_addr = MFC_CORE_READL(MFC_REG_E_RECON_LUMA_DPB_ADDR);
	enc_recon_c_addr = MFC_CORE_READL(MFC_REG_E_RECON_CHROMA_DPB_ADDR);

	mfc_debug(2, "[MEMINFO] recon y: 0x%08lx c: 0x%08lx\n",
			enc_recon_y_addr, enc_recon_c_addr);
}

void mfc_core_set_enc_stride(struct mfc_core *core, struct mfc_ctx *ctx)
{
	int i;

	for (i = 0; i < ctx->raw_buf.num_planes; i++) {
		MFC_CORE_WRITEL(ctx->raw_buf.stride[i],
				MFC_REG_E_SOURCE_FIRST_STRIDE + (i * 4));
		mfc_debug(2, "[FRAME] enc src plane[%d] stride: %d\n",
				i, ctx->raw_buf.stride[i]);
		if (IS_2BIT_NEED(ctx)) {
			MFC_CORE_WRITEL(ctx->raw_buf.stride_2bits[0],
					MFC_REG_E_SOURCE_FIRST_2BIT_STRIDE + (i * 4));

			mfc_debug(2, "[FRAME] enc src plane[%d] 2bit stride: %d\n",
					i, ctx->raw_buf.stride_2bits[i]);
		}
	}
}

int mfc_core_set_dynamic_dpb(struct mfc_core *core, struct mfc_ctx *ctx,
		struct mfc_buf *dst_mb)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_raw_info *raw = &ctx->raw_buf;
	int dst_index;
	int i;

	dst_index = dst_mb->dpb_index;

	/* for debugging about black bar detection */
	if ((MFC_FEATURE_SUPPORT(dev, dev->pdata->black_bar) && dec->detect_black_bar) ||
			(feature_option & MFC_OPTION_BLACK_BAR_ENABLE)) {
		for (i = 0; i < raw->num_planes; i++) {
			dec->frame_vaddr[i][dec->frame_cnt] = vb2_plane_vaddr(&dst_mb->vb.vb2_buf, i);
			dec->frame_daddr[i][dec->frame_cnt] = dst_mb->addr[0][i];
			dec->frame_size[i][dec->frame_cnt] = raw->plane_size[i];
			dec->index[i][dec->frame_cnt] = dst_index;
			dec->fd[i][dec->frame_cnt] = dst_mb->vb.vb2_buf.planes[0].m.fd;
		}
		dec->frame_cnt++;
		if (dec->frame_cnt >= 30)
			dec->frame_cnt = 0;
	}

	for (i = 0; i < raw->num_planes; i++) {
		MFC_CORE_WRITEL(raw->plane_size[i],
				MFC_REG_D_FIRST_PLANE_DPB_SIZE + i * 4);
		MFC_CORE_WRITEL(dst_mb->addr[0][i],
				MFC_REG_D_FIRST_PLANE_DPB0 + (i * 0x100 + dst_index * 4));
		ctx->last_dst_addr[i] = dst_mb->addr[0][i];
		if (IS_2BIT_NEED(ctx))
			MFC_CORE_WRITEL(raw->plane_size_2bits[i],
					MFC_REG_D_FIRST_PLANE_2BIT_DPB_SIZE + (i * 4));
		mfc_debug(2, "[BUFINFO][DPB] ctx[%d] set dst index: [%d][%d], addr[%d]: 0x%08llx, fd: %d\n",
				ctx->num, dst_mb->vb.vb2_buf.index, dst_mb->dpb_index,
				i, dst_mb->addr[0][i], dst_mb->vb.vb2_buf.planes[0].m.fd);
	}

	MFC_TRACE_CORE_CTX("Set dst[%d] fd: %d, %#llx / used %#lx\n",
			dst_index, dst_mb->vb.vb2_buf.planes[0].m.fd,
			dst_mb->addr[0][0], dec->dynamic_used);

	return 0;
}

void mfc_core_get_img_size(struct mfc_core *core, struct mfc_ctx *ctx,
		enum mfc_get_img_size img_size)
{
	struct mfc_dec *dec = ctx->dec_priv;
	unsigned int w, h;
	int i;

	w = ctx->img_width;
	h = ctx->img_height;

	ctx->img_width = mfc_core_get_img_width();
	ctx->img_height = mfc_core_get_img_height();
	ctx->crop_width = ctx->img_width;
	ctx->crop_height = ctx->img_height;

	for (i = 0; i < ctx->dst_fmt->num_planes; i++) {
		ctx->raw_buf.stride[i] = mfc_core_get_stride_size(i);
		if (IS_2BIT_NEED(ctx))
			ctx->raw_buf.stride_2bits[i] = mfc_core_get_stride_size_2bit(i);
	}
	mfc_debug(2, "[FRAME][DRC] resolution changed, %dx%d => %dx%d (stride: %d)\n", w, h,
			ctx->img_width, ctx->img_height, ctx->raw_buf.stride[0]);

	if (img_size == MFC_GET_RESOL_SIZE) {
		dec->disp_drc.width[dec->disp_drc.push_idx] = ctx->img_width;
		dec->disp_drc.height[dec->disp_drc.push_idx] = ctx->img_height;
		dec->disp_drc.disp_res_change = ++dec->disp_drc.disp_res_change % MFC_MAX_DRC_FRAME;
		mfc_debug(3, "[DRC] disp_res_change[%d] count %d\n",
				dec->disp_drc.push_idx, dec->disp_drc.disp_res_change);
		dec->disp_drc.push_idx = ++dec->disp_drc.push_idx % MFC_MAX_DRC_FRAME;
	} else if (img_size == MFC_GET_RESOL_DPB_SIZE) {
		ctx->scratch_buf_size = mfc_core_get_scratch_size();
		for (i = 0; i < ctx->dst_fmt->num_planes; i++) {
			ctx->min_dpb_size[i] = mfc_core_get_min_dpb_size(i);
			if (IS_2BIT_NEED(ctx))
				ctx->min_dpb_size_2bits[i] = mfc_core_get_min_dpb_size_2bit(i);
		}

		mfc_debug(2, "[FRAME][DRC] DPB count %d, min_dpb_size %d(%#x) min_dpb_size_2bits %d scratch %zu(%#zx)\n",
			ctx->dpb_count, ctx->min_dpb_size[0], ctx->min_dpb_size[0], ctx->min_dpb_size_2bits[0],
			ctx->scratch_buf_size, ctx->scratch_buf_size);
	}
}

static void __mfc_enc_check_sbwc_option(struct mfc_ctx *ctx, unsigned int *sbwc)
{
	struct mfc_enc *enc = ctx->enc_priv;

	/*
	 * compressor option for encoder
	 * - feature_option enable and SBWC format: apply in input source and DPB (0)
	 * - feature_option enable and not SBWC format: apply only in DPB (2)
	 * - feature_option disable and SBWC format: apply only in input source (1)
	 * - feature_option disable and not SBWC format: no SBWC
	 */
	if (!(feature_option & MFC_OPTION_RECON_SBWC_DISABLE)) {
		if (*sbwc == 1) {
			enc->sbwc_option = 0;
			mfc_debug(2, "[SBWC] apply in input source and DPB\n");
		} else {
			enc->sbwc_option = 2;
			*sbwc = 1;
			mfc_debug(2, "[SBWC] enable SBWC and apply SBWC only in DPB\n");
		}
	} else {
		if (*sbwc == 1) {
			enc->sbwc_option = 1;
			mfc_debug(2, "[SBWC] apply SBWC only in input source\n");
		} else {
			enc->sbwc_option = 0;
			mfc_debug(2, "[SBWC] no SBWC\n");
		}
	}
}

void mfc_core_set_pixel_format(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int format)
{
	struct mfc_dev *dev = ctx->dev;
	unsigned int reg = 0;
	unsigned int pix_val;
	unsigned int sbwc = 0, align = 0;

	if (dev->pdata->P010_decoding)
		ctx->mem_type_10bit = 1;

	switch (format) {
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12N:
	case V4L2_PIX_FMT_NV12MT_16X16:
	case V4L2_PIX_FMT_NV16M:
		pix_val = 0;
		break;
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV61M:
		pix_val = 1;
		break;
	case V4L2_PIX_FMT_YVU420M:
	case V4L2_PIX_FMT_YVU420N:
		pix_val = 2;
		break;
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420N:
		pix_val = 3;
		break;
	/* For 10bit direct set */
	case V4L2_PIX_FMT_NV12N_10B:
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV16M_S10B:
		ctx->mem_type_10bit = 0;
		pix_val = 0;
		break;
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV12N_P010:
	case V4L2_PIX_FMT_NV16M_P210:
		ctx->mem_type_10bit = 1;
		pix_val = 0;
		break;
	case V4L2_PIX_FMT_NV21M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
		ctx->mem_type_10bit = 0;
		pix_val = 1;
		break;
	case V4L2_PIX_FMT_NV21M_P010:
	case V4L2_PIX_FMT_NV61M_P210:
		ctx->mem_type_10bit = 1;
		pix_val = 1;
		break;
	case V4L2_PIX_FMT_ARGB32:
		pix_val = 8;
		break;
	case V4L2_PIX_FMT_RGB24:
		pix_val = 9;
		break;
	case V4L2_PIX_FMT_RGB565:
		pix_val = 10;
		break;
	case V4L2_PIX_FMT_RGB32:
		pix_val = 11;
		break;
	case V4L2_PIX_FMT_RGB32X:
		pix_val = 12;
		break;
	case V4L2_PIX_FMT_BGR32:
		pix_val = 13;
		break;
	/* for compress format (SBWC) */
	case V4L2_PIX_FMT_NV12M_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
	case V4L2_PIX_FMT_NV21M_SBWC_8B:
	case V4L2_PIX_FMT_NV12M_SBWCL_8B:
	case V4L2_PIX_FMT_NV12N_SBWCL_8B:
	case V4L2_PIX_FMT_NV12M_SBWCL_10B:
	case V4L2_PIX_FMT_NV12N_SBWCL_10B:
		pix_val = 0;
		sbwc = 1;
		break;
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
	case V4L2_PIX_FMT_NV21M_SBWC_10B:
		pix_val = 0;
		sbwc = 1;
		align = 1;
		break;
	default:
		pix_val = 0;
		break;
	}
	mfc_set_bits(reg, 0xf, 0, pix_val);

	/* for YUV format */
	if (pix_val < 4)
		mfc_set_bits(reg, 0x3, 4, ctx->mem_type_10bit);

	if (dev->pdata->support_sbwc) {
		if (ctx->type == MFCINST_ENCODER && pix_val < 4 &&
				!(ctx->src_fmt->type & MFC_FMT_422))
			__mfc_enc_check_sbwc_option(ctx, &sbwc);

		mfc_set_bits(reg, 0x1, 9, sbwc);
		mfc_set_bits(reg, 0x1, 10, align);
	}

	MFC_CORE_WRITEL(reg, MFC_REG_PIXEL_FORMAT);
	mfc_debug(2, "[FRAME] pix format: %d, mem_type_10bit: %d, sbwc: %d, align: %d,(reg: %#x)\n",
			pix_val, ctx->mem_type_10bit, sbwc, align, reg);
}

void mfc_core_print_hdr_plus_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct hdr10_plus_meta *sei_meta)
{
	int num_distribution;
	int i, j;

	if (ctx->type == MFCINST_DECODER)
		mfc_debug(5, "[HDR+] ================= Decoder metadata =================\n");
	else
		mfc_debug(5, "[HDR+] ================= Encoder metadata =================\n");

	mfc_debug(5, "[HDR+] valid: %#x\n", sei_meta->valid);
	mfc_debug(5, "[HDR+] itu t35 country_code: %#x, provider_code %#x, oriented_code: %#x\n",
			sei_meta->t35_country_code, sei_meta->t35_terminal_provider_code,
			sei_meta->t35_terminal_provider_oriented_code);
	mfc_debug(5, "[HDR+] application identifier: %#x, version: %#x, num_windows: %#x\n",
			sei_meta->application_identifier, sei_meta->application_version,
			sei_meta->num_windows);
	mfc_debug(5, "[HDR+] target_maximum_luminance: %#x, target_actual_peak_luminance_flag %#x\n",
			sei_meta->target_maximum_luminance,
			sei_meta->target_actual_peak_luminance_flag);
	mfc_debug(5, "[HDR+] mastering_actual_peak_luminance_flag %#x\n",
			sei_meta->mastering_actual_peak_luminance_flag);

	for (i = 0; i < sei_meta->num_windows; i++) {
		mfc_debug(5, "[HDR+] -------- window[%d] info --------\n", i);
		for (j = 0; j < HDR_MAX_SCL; j++)
			mfc_debug(5, "[HDR+] maxscl[%d] %#x\n", j,
					sei_meta->win_info[i].maxscl[j]);
		mfc_debug(5, "[HDR+] average_maxrgb %#x, num_distribution_maxrgb_percentiles %#x\n",
				sei_meta->win_info[i].average_maxrgb,
				sei_meta->win_info[i].num_distribution_maxrgb_percentiles);
		num_distribution = sei_meta->win_info[i].num_distribution_maxrgb_percentiles;
		for (j = 0; j < num_distribution; j++)
			mfc_debug(5, "[HDR+] percentages[%d] %#x, percentiles[%d] %#x\n", j,
					sei_meta->win_info[i].distribution_maxrgb_percentages[j], j,
					sei_meta->win_info[i].distribution_maxrgb_percentiles[j]);

		mfc_debug(5, "[HDR+] fraction_bright_pixels %#x, tone_mapping_flag %#x\n",
				sei_meta->win_info[i].fraction_bright_pixels,
				sei_meta->win_info[i].tone_mapping_flag);
		if (sei_meta->win_info[i].tone_mapping_flag) {
			mfc_debug(5, "[HDR+] knee point x %#x, knee point y %#x\n",
					sei_meta->win_info[i].knee_point_x,
					sei_meta->win_info[i].knee_point_y);
			mfc_debug(5, "[HDR+] num_bezier_curve_anchors %#x\n",
					sei_meta->win_info[i].num_bezier_curve_anchors);
			for (j = 0; j < HDR_MAX_BEZIER_CURVES / 3; j++)
				mfc_debug(5, "[HDR+] anchors[%d] %#x, [%d] %#x, [%d] %#x\n",
						j * 3,
						sei_meta->win_info[i].bezier_curve_anchors[j * 3],
						j * 3 + 1,
						sei_meta->win_info[i].bezier_curve_anchors[j * 3 + 1],
						j * 3 + 2,
						sei_meta->win_info[i].bezier_curve_anchors[j * 3 + 2]);
		}
		mfc_debug(5, "[HDR+] color_saturation mapping_flag %#x, weight: %#x\n",
				sei_meta->win_info[i].color_saturation_mapping_flag,
				sei_meta->win_info[i].color_saturation_weight);
	}

	mfc_debug(5, "[HDR+] ================================================================\n");

	/* Do not need to dump register */
	if (core->nal_q_handle)
		if (core->nal_q_handle->nal_q_state == NAL_Q_STATE_STARTED)
			return;

	if (ctx->type == MFCINST_DECODER)
		print_hex_dump(KERN_ERR, "[HDR+] ", DUMP_PREFIX_OFFSET, 32, 4,
				core->regs_base + MFC_REG_D_ST_2094_40_SEI_0, 0x78, false);
	else
		print_hex_dump(KERN_ERR, "[HDR+] ", DUMP_PREFIX_OFFSET, 32, 4,
				core->regs_base + MFC_REG_E_ST_2094_40_SEI_0, 0x78, false);
}

void mfc_core_get_hdr_plus_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct hdr10_plus_meta *sei_meta)
{
	struct mfc_dev *dev = ctx->dev;
	unsigned int upper_value, lower_value;
	int num_win, num_distribution;
	int i, j;

	sei_meta->valid = 1;

	/* iru_t_t35 */
	sei_meta->t35_country_code = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_0) & 0xFF;
	sei_meta->t35_terminal_provider_code = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_0) >> 8 & 0xFF;
	upper_value = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_0) >> 24 & 0xFF;
	lower_value = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_1) & 0xFF;
	sei_meta->t35_terminal_provider_oriented_code = (upper_value << 8) | lower_value;

	/* application */
	sei_meta->application_identifier = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_1) >> 8 & 0xFF;
	sei_meta->application_version = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_1) >> 16 & 0xFF;

	/* window information */
	sei_meta->num_windows = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_1) >> 24 & 0x3;
	num_win = sei_meta->num_windows;
	if (num_win > dev->pdata->max_hdr_win) {
		mfc_ctx_err("[HDR+] num_window(%d) is exceeded supported max_num_window(%d)\n",
				num_win, dev->pdata->max_hdr_win);
		num_win = dev->pdata->max_hdr_win;
		sei_meta->num_windows = num_win;
	}

	/* luminance */
	sei_meta->target_maximum_luminance = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_2) & 0x7FFFFFF;
	sei_meta->target_actual_peak_luminance_flag = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_2) >> 27 & 0x1;
	sei_meta->mastering_actual_peak_luminance_flag = MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_22) >> 10 & 0x1;

	/* per window setting */
	for (i = 0; i < num_win; i++) {
		/* scl */
		for (j = 0; j < HDR_MAX_SCL; j++) {
			sei_meta->win_info[i].maxscl[j] =
				MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_3 + (4 * j)) & 0x1FFFF;
		}
		sei_meta->win_info[i].average_maxrgb =
			MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_6) & 0x1FFFF;

		/* distribution */
		sei_meta->win_info[i].num_distribution_maxrgb_percentiles =
			MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_6) >> 17 & 0xF;
		num_distribution = sei_meta->win_info[i].num_distribution_maxrgb_percentiles;
		for (j = 0; j < num_distribution; j++) {
			sei_meta->win_info[i].distribution_maxrgb_percentages[j] =
				MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_7 + (4 * j)) & 0x7F;
			sei_meta->win_info[i].distribution_maxrgb_percentiles[j] =
				MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_7 + (4 * j)) >> 7 & 0x1FFFF;
		}

		/* bright pixels */
		sei_meta->win_info[i].fraction_bright_pixels =
			MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_22) & 0x3FF;

		/* tone mapping */
		sei_meta->win_info[i].tone_mapping_flag =
			MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_22) >> 11 & 0x1;
		if (sei_meta->win_info[i].tone_mapping_flag) {
			sei_meta->win_info[i].knee_point_x =
				MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_23) & 0xFFF;
			sei_meta->win_info[i].knee_point_y =
				MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_23) >> 12 & 0xFFF;
			sei_meta->win_info[i].num_bezier_curve_anchors =
				MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_23) >> 24 & 0xF;
			for (j = 0; j < HDR_MAX_BEZIER_CURVES / 3; j++) {
				sei_meta->win_info[i].bezier_curve_anchors[j * 3] =
					MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_24 + (4 * j)) & 0x3FF;
				sei_meta->win_info[i].bezier_curve_anchors[j * 3 + 1] =
					MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_24 + (4 * j)) >> 10 & 0x3FF;
				sei_meta->win_info[i].bezier_curve_anchors[j * 3 + 2] =
					MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_24 + (4 * j)) >> 20 & 0x3FF;
			}
		}

		/* color saturation */
		sei_meta->win_info[i].color_saturation_mapping_flag =
			MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_29) & 0x1;
		if (sei_meta->win_info[i].color_saturation_mapping_flag)
			sei_meta->win_info[i].color_saturation_weight =
				MFC_CORE_READL(MFC_REG_D_ST_2094_40_SEI_29) >> 1 & 0x3F;
	}

	if (debug_level >= 5)
		mfc_core_print_hdr_plus_info(core, ctx, sei_meta);
}

void mfc_core_set_hdr_plus_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct hdr10_plus_meta *sei_meta)
{
	struct mfc_dev *dev = ctx->dev;
	unsigned int reg = 0;
	int num_win, num_distribution;
	int i, j;

	reg = MFC_CORE_READL(MFC_REG_E_HEVC_NAL_CONTROL);
	reg &= ~(0x1 << 6);
	reg |= ((sei_meta->valid & 0x1) << 6);
	MFC_CORE_WRITEL(reg, MFC_REG_E_HEVC_NAL_CONTROL);

	/* iru_t_t35 */
	reg = 0;
	reg |= (sei_meta->t35_country_code & 0xFF);
	reg |= ((sei_meta->t35_terminal_provider_code & 0xFF) << 8);
	reg |= (((sei_meta->t35_terminal_provider_oriented_code >> 8) & 0xFF) << 24);
	MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_0);

	/* window information */
	num_win = (sei_meta->num_windows & 0x3);
	if (!num_win || (num_win > dev->pdata->max_hdr_win)) {
		mfc_debug(3, "[HDR+] num_window is only supported till %d\n",
				dev->pdata->max_hdr_win);
		num_win = dev->pdata->max_hdr_win;
		sei_meta->num_windows = num_win;
	}

	/* application */
	reg = 0;
	reg |= (sei_meta->t35_terminal_provider_oriented_code & 0xFF);
	reg |= ((sei_meta->application_identifier & 0xFF) << 8);
	reg |= ((sei_meta->application_version & 0xFF) << 16);
	reg |= ((sei_meta->num_windows & 0x3) << 24);
	MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_1);

	/* luminance */
	reg = 0;
	reg |= (sei_meta->target_maximum_luminance & 0x7FFFFFF);
	reg |= ((sei_meta->target_actual_peak_luminance_flag & 0x1) << 27);
	MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_2);

	/* per window setting */
	for (i = 0; i < num_win; i++) {
		/* scl */
		for (j = 0; j < HDR_MAX_SCL; j++) {
			reg = (sei_meta->win_info[i].maxscl[j] & 0x1FFFF);
			MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_3 + (4 * j));
		}

		/* distribution */
		reg = 0;
		reg |= (sei_meta->win_info[i].average_maxrgb & 0x1FFFF);
		reg |= ((sei_meta->win_info[i].num_distribution_maxrgb_percentiles & 0xF) << 17);
		MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_6);
		num_distribution = (sei_meta->win_info[i].num_distribution_maxrgb_percentiles & 0xF);
		for (j = 0; j < num_distribution; j++) {
			reg = 0;
			reg |= (sei_meta->win_info[i].distribution_maxrgb_percentages[j] & 0x7F);
			reg |= ((sei_meta->win_info[i].distribution_maxrgb_percentiles[j] & 0x1FFFF) << 7);
			MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_7 + (4 * j));
		}

		/* bright pixels, luminance */
		reg = 0;
		reg |= (sei_meta->win_info[i].fraction_bright_pixels & 0x3FF);
		reg |= ((sei_meta->mastering_actual_peak_luminance_flag & 0x1) << 10);

		/* tone mapping */
		reg |= ((sei_meta->win_info[i].tone_mapping_flag & 0x1) << 11);
		MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_22);
		if (sei_meta->win_info[i].tone_mapping_flag & 0x1) {
			reg = 0;
			reg |= (sei_meta->win_info[i].knee_point_x & 0xFFF);
			reg |= ((sei_meta->win_info[i].knee_point_y & 0xFFF) << 12);
			reg |= ((sei_meta->win_info[i].num_bezier_curve_anchors & 0xF) << 24);
			MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_23);
			for (j = 0; j < HDR_MAX_BEZIER_CURVES / 3; j++) {
				reg = 0;
				reg |= (sei_meta->win_info[i].bezier_curve_anchors[j * 3] & 0x3FF);
				reg |= ((sei_meta->win_info[i].bezier_curve_anchors[j * 3 + 1] & 0x3FF) << 10);
				reg |= ((sei_meta->win_info[i].bezier_curve_anchors[j * 3 + 2] & 0x3FF) << 20);
				MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_24 + (4 * j));
			}

		}

		/* color saturation */
		if (sei_meta->win_info[i].color_saturation_mapping_flag & 0x1) {
			reg = 0;
			reg |= (sei_meta->win_info[i].color_saturation_mapping_flag & 0x1);
			reg |= ((sei_meta->win_info[i].color_saturation_weight & 0x3F) << 1);
			MFC_CORE_WRITEL(reg, MFC_REG_E_ST_2094_40_SEI_29);
		}
	}

	if (debug_level >= 5)
		mfc_core_print_hdr_plus_info(core, ctx, sei_meta);
}

void mfc_core_set_dec_metadata_buffer(struct mfc_core *core, struct mfc_ctx *ctx)
{
	MFC_CORE_WRITEL(ctx->metadata_buf.daddr, MFC_REG_D_METADATA_BUFFER_ADDR);
	MFC_CORE_WRITEL(ctx->metadata_buf.size, MFC_REG_D_METADATA_BUFFER_SIZE);

	mfc_debug(2, "[MEMINFO] metadata buf %pad size: %zu\n",
			&ctx->metadata_buf.daddr, ctx->metadata_buf.size);
}

void mfc_core_get_dec_metadata_sei_nal(struct mfc_core *core, struct mfc_ctx *ctx, int index)
{
	struct mfc_dec *dec = ctx->dec_priv;
	dma_addr_t buf_addr;
	dma_addr_t offset;
	unsigned int *sei_addr = NULL;
	unsigned int *addr;
	int buf_size, sei_size;
	int i;

	addr = HDR10_PLUS_ADDR(dec->hdr10_plus_full, index);

	if (ctx->is_drm) {
		/* When DRM, read metadata from register */
		sei_size = MFC_CORE_READL(MFC_REG_D_PAYLOAD_SIZE);
		sei_size += MFC_META_SEI_NAL_SIZE_OFFSET;
		if (sei_size == 0)
			mfc_ctx_err("[HDR+][DRM] sei size is zero\n");

		for (i = 0; i < (__ALIGN_UP(sei_size, 4) / 4); i++)
			addr[i] = MFC_CORE_READL(MFC_REG_D_PAYLOAD_SIZE + (i * 4));
		if (hdr_dump == 1) {
			mfc_ctx_err("[HDR+][DUMP][DRM] DRV data (idx %d)....\n", index);
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4, addr,
					sei_size, false);
		}
		return;
	}

	buf_addr = MFC_CORE_READL(MFC_REG_D_METADATA_ADDR_SEI_NAL);
	buf_size = MFC_CORE_READL(MFC_REG_D_METADATA_SIZE_SEI_NAL);
	if (!buf_addr) {
		mfc_ctx_err("[META] The metadata address is NULL\n");
		return;
	}

	offset = buf_addr - ctx->metadata_buf.daddr;
	if (offset < 0) {
		mfc_ctx_err("[HDR+][META] The metadata offset %pad is wrong\n", &offset);
		return;
	}

	/* SEI data - 0x0: payload type, 0x4: payload size, 0x8: payload data */
	sei_addr = ctx->metadata_buf.vaddr + offset + MFC_META_SEI_NAL_SIZE_OFFSET;
	sei_size = *sei_addr;

	/* If there is other SEI data, need to use it for purpose */
	if (sei_size != (buf_size - MFC_META_SEI_NAL_PAYLOAD_OFFSET))
		mfc_ctx_err("[HDR+][META] There is another SEI data (%d / %d)\n",
				sei_size, buf_size);

	/* HAL needs SEI data size info "size(4 bytes) + SEI data" */
	sei_size += MFC_META_SEI_NAL_SIZE_OFFSET;
	mfc_debug(2, "[HDR+][META] copy metadata offset %pad size: %d / %d\n",
			&offset, sei_size, buf_size);

	memcpy(addr, sei_addr, sei_size);
	if (hdr_dump == 1) {
		mfc_ctx_err("[HDR+][DUMP] F/W data (offset %pad)....\n", &offset);
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4,
				&ctx->metadata_buf.vaddr + offset,
				buf_size, false);
		mfc_ctx_err("[HDR+][DUMP] DRV data (idx %d)....\n", index);
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4, addr,
				sei_size, false);
	}
}

static void __mfc_core_print_av1_array(struct mfc_ctx *ctx, void *buf, int num)
{
	struct mfc_dec *dec = ctx->dec_priv;
	int i, ret, idx = 0;
	char *input = (char *)buf;
	char *pbuf_ptr = &dec->av1_film_grain_info_data[0];

	ret = snprintf(pbuf_ptr + idx, (128 - idx), "[FILMGR][Decimal]  ");
	idx += ret;

	for (i = 0; i < num; i++) {
		if (((i != 0) && ((i % 5) == 0)) || idx > 127) {
			dec->av1_film_grain_info_data[127] = 0;
			mfc_debug(5, "%s\n", dec->av1_film_grain_info_data);
			memset(dec->av1_film_grain_info_data, 0, sizeof(dec->av1_film_grain_info_data));
			pbuf_ptr = &dec->av1_film_grain_info_data[0];
			idx = 0;

			ret = snprintf(pbuf_ptr + idx, (128 - idx), "[FILMGR][Decimal]  ");
			idx += ret;
		}
		ret = snprintf(pbuf_ptr + idx, (128 - idx), "[%02d] %03d,  ", i, input[i]);
		idx += ret;
	}

	if (num != 0)
		mfc_debug(5, "%s\n", dec->av1_film_grain_info_data);
	else
		mfc_debug(5, "[FILMGR] -\n");
}

void mfc_core_print_av1_film_grain_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct av1_film_grain_meta *sei_meta)
{
	mfc_debug(5, "[FILMGR] ================= Decoder metadata =================\n");

	mfc_debug(5, "[FILMGR] apply_grain: %d, grain_seed %d\n",
		sei_meta->apply_grain, sei_meta->grain_seed);
	mfc_debug(5, "[FILMGR] num_y_points: %d, num_cb_points: %d, num_cr_points: %d\n",
		sei_meta->num_y_points, sei_meta->num_cb_points, sei_meta->num_cr_points);
	mfc_debug(5, "[FILMGR] chroma_scaling_from_luma: %d, grain_scaling_minus_8: %d\n",
		sei_meta->chroma_scaling_from_luma, sei_meta->grain_scaling_minus_8);
	mfc_debug(5, "[FILMGR] ar_coeff_lag: %d, ar_coeff_shift_minus_6: %d\n",
		sei_meta->ar_coeff_lag, sei_meta->ar_coeff_shift_minus_6);
	mfc_debug(5, "[FILMGR] cb_mult: %d, cb_luma_mult: %d, cb_offset: %d\n",
		sei_meta->cb_mult, sei_meta->cb_luma_mult, sei_meta->cb_offset);
	mfc_debug(5, "[FILMGR] cr_mult: %d, cr_luma_mult: %d, cr_offset: %d\n",
		sei_meta->cr_mult, sei_meta->cr_luma_mult, sei_meta->cr_offset);
	mfc_debug(5, "[FILMGR] grain_scale_shift: %d, overlap_flag: %d\n",
		sei_meta->grain_scale_shift, sei_meta->overlap_flag);
	mfc_debug(5, "[FILMGR] clip_to_restricted_range: %d, mc_identity: %d\n",
		sei_meta->clip_to_restricted_range, sei_meta->mc_identity);

	mfc_debug(5, "[FILMGR] DUMP point_y_value[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->point_y_value[0],
		sei_meta->num_y_points);
	mfc_debug(5, "[FILMGR] DUMP point_y_scaling[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->point_y_scaling[0],
		sei_meta->num_y_points);
	mfc_debug(5, "[FILMGR] DUMP point_cb_value[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->point_cb_value[0],
		sei_meta->num_cb_points);
	mfc_debug(5, "[FILMGR] DUMP point_cb_scaling[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->point_cb_scaling[0],
		sei_meta->num_cb_points);
	mfc_debug(5, "[FILMGR] DUMP point_cr_value[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->point_cr_value[0],
		sei_meta->num_cr_points);
	mfc_debug(5, "[FILMGR] DUMP point_cr_scaling[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->point_cr_scaling[0],
		sei_meta->num_cr_points);

	mfc_debug(5, "[FILMGR] DUMP ar_coeffs_y_plus_128[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->ar_coeffs_y_plus_128[0],
		AV1_FG_LUM_AR_COEF_SIZE);
	mfc_debug(5, "[FILMGR] DUMP ar_coeffs_cb_plus_128[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->ar_coeffs_cb_plus_128[0],
		AV1_FG_CHR_AR_COEF_SIZE);
	mfc_debug(5, "[FILMGR] DUMP ar_coeffs_cr_plus_128[] ---\n");
	__mfc_core_print_av1_array(ctx, (void *)&sei_meta->ar_coeffs_cr_plus_128[0],
		AV1_FG_CHR_AR_COEF_SIZE);

	mfc_debug(5, "[FILMGR] ====================================================\n");
	/* Do not need to dump register */
	if (core->nal_q_handle)
		if (core->nal_q_handle->nal_q_state == NAL_Q_STATE_STARTED)
			return;

	print_hex_dump(KERN_ERR, "[FILMGR] ", DUMP_PREFIX_OFFSET, 32, 4,
			core->regs_base + MFC_REG_D_FILM_GRAIN_0, 0xB0, false);
}

/*
 *
 * D_FILM_GRAIN_1 [15:8]  = POINT_Y_VALUE_0
 * D_FILM_GRAIN_1 [23:16] = POINT_Y_VALUE_1
 * D_FILM_GRAIN_1 [24:31] = POINT_Y_VALUE_2
 * D_FILM_GRAIN_2 [7:0]   = POINT_Y_VALUE_3
 *
 * av1_bitmask_shift[] has L-shift value.
 *
 */
static void __get_av1_point_value_info(struct mfc_core *core, struct mfc_ctx *ctx,
	unsigned char *sei_meta, int num, unsigned int start_addr)
{
	int i;
	unsigned int reg = 0;

	mfc_debug(5, "[FILMGR] GET point Value info START\n");
	reg = MFC_CORE_READL(start_addr);
	mfc_debug(5, "[FILMGR] GET point value addr: %#x, val: %#x\n", start_addr, reg);
	for (i = 1; i <= num; i++) {
		if ((i % 4) == 0) {
			reg = MFC_CORE_READL(start_addr + i);
			mfc_debug(5, "[FILMGR] GET point value addr: %#x, val: %#x\n", start_addr + i, reg);
		}
		sei_meta[i - 1] = (reg >> av1_bitmask_shift[i]) & 0xFF;
	}
}

/*
 *
 * D_FILM_GRAIN_5 [7:0]   = POINT_Y_SCALING_0
 * D_FILM_GRAIN_5 [15:8]  = POINT_Y_SCALING_1
 * D_FILM_GRAIN_5 [23:16] = POINT_Y_SCALING_2
 * D_FILM_GRAIN_5 [24:31] = POINT_Y_SCALING_3
 * D_FILM_GRAIN_6 [7:0]   = POINT_Y_SCALING_4
 *
 * av1_bitmask_shift[] has L-shift value.
 *
 */
static void __get_av1_point_scaling_info(struct mfc_core *core, struct mfc_ctx *ctx,
	char *sei_meta, int num, unsigned int start_addr)
{
	int i;
	unsigned int reg = 0;

	mfc_debug(5, "[FILMGR] GET scaling Value info START\n");
	for (i = 0; i < num; i++) {
		if ((i % 4) == 0) {
			reg = MFC_CORE_READL(start_addr + i);
			mfc_debug(5, "[FILMGR] GET scaling value addr: %#x, val: %#x\n", start_addr + i, reg);
		}
		sei_meta[i] = (reg >> av1_bitmask_shift[i]) & 0xFF;
	}
}

static void __get_av1_coeffs_info(struct mfc_core *core, struct mfc_ctx *ctx,
	char *sei_meta, int num, unsigned int start_addr)
{
	int i, j;
	unsigned int reg = 0;

	mfc_debug(5, "[FILMGR] GET coeffs Value info START\n");
	for (i = 0; i < num; i++) {
		j = i % 4;
		if (j == 0) {
			reg = MFC_CORE_READL(start_addr + i);
			mfc_debug(5, "[FILMGR] GET coeffes value addr: %#x, val: %#x\n", start_addr + i, reg);
		}
		sei_meta[i] = (reg >> av1_bitmask_shift[j]) & 0xFF;
	}
}

void mfc_core_get_av1_film_grain_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct av1_film_grain_meta *sei_meta)
{
	unsigned int reg = 0;

	/* from the SFR */
	reg = MFC_CORE_READL(MFC_REG_D_FILM_GRAIN_0);
	mfc_debug(5, "[FILMGR] D_FILM_GRAIN_[0], val: %#x\n", reg);
	sei_meta->apply_grain = reg & 0x1;
	sei_meta->grain_seed = (reg >> 1) & 0xFFFF;

	reg = MFC_CORE_READL(MFC_REG_D_FILM_GRAIN_1);
	mfc_debug(5, "[FILMGR] D_FILM_GRAIN_[1], val: %#x\n", reg);
	sei_meta->num_y_points = reg & 0xF;

	reg = MFC_CORE_READL(MFC_REG_D_FILM_GRAIN_9);
	mfc_debug(5, "[FILMGR] D_FILM_GRAIN_[9], val: %#x\n", reg);
	sei_meta->num_cb_points = reg & 0xF;

	reg = MFC_CORE_READL(MFC_REG_D_FILM_GRAIN_15);
	mfc_debug(5, "[FILMGR] D_FILM_GRAIN_[15], val: %#x\n", reg);
	sei_meta->num_cr_points = reg & 0xF;

	reg = MFC_CORE_READL(MFC_REG_D_FILM_GRAIN_8);
	mfc_debug(5, "[FILMGR] D_FILM_GRAIN_[8], val: %#x\n", reg);
	sei_meta->chroma_scaling_from_luma = (reg >> 16) & 0x1;

	reg = MFC_CORE_READL(MFC_REG_D_FILM_GRAIN_21);
	mfc_debug(5, "[FILMGR] D_FILM_GRAIN_[21], val: %#x\n", reg);
	sei_meta->grain_scaling_minus_8 = reg & 0x3;
	sei_meta->ar_coeff_lag = (reg >> 0x2) & 0x3;

	reg = MFC_CORE_READL(MFC_REG_D_FILM_GRAIN_42);
	mfc_debug(5, "[FILMGR] D_FILM_GRAIN_[42], val: %#x\n", reg);
	sei_meta->ar_coeff_shift_minus_6 = reg & 0x3;
	sei_meta->grain_scale_shift = (reg >> 2) & 0x3;
	sei_meta->cb_mult = (reg >> 4) & 0xFF;
	sei_meta->cb_luma_mult = (reg >> 12) & 0xFF;
	sei_meta->cb_offset = (reg >> 20) & 0x1FF;

	reg = MFC_CORE_READL(MFC_REG_D_FILM_GRAIN_43);
	mfc_debug(5, "[FILMGR] D_FILM_GRAIN_[43], val: %#x\n", reg);
	sei_meta->cr_mult = reg & 0xFF;
	sei_meta->cr_luma_mult = (reg >> 8) & 0xFF;
	sei_meta->cr_offset = (reg >> 16) & 0x1FF;
	sei_meta->overlap_flag = (reg >> 25) & 0x1;
	sei_meta->clip_to_restricted_range = (reg >> 26) & 0x1;
	sei_meta->mc_identity = (reg >> 27) & 0x1;


	__get_av1_point_value_info(core, ctx, &sei_meta->point_y_value[0],
		AV1_FG_LUM_POS_SIZE, MFC_REG_D_FILM_GRAIN_1);
	__get_av1_point_value_info(core, ctx, &sei_meta->point_cb_value[0],
		AV1_FG_CHR_POS_SIZE, MFC_REG_D_FILM_GRAIN_9);
	__get_av1_point_value_info(core, ctx, &sei_meta->point_cr_value[0],
		AV1_FG_CHR_POS_SIZE, MFC_REG_D_FILM_GRAIN_15);

	__get_av1_point_scaling_info(core, ctx, &sei_meta->point_y_scaling[0],
		AV1_FG_LUM_POS_SIZE, MFC_REG_D_FILM_GRAIN_5);
	__get_av1_point_scaling_info(core, ctx, &sei_meta->point_cb_scaling[0],
		AV1_FG_CHR_POS_SIZE, MFC_REG_D_FILM_GRAIN_12);
	__get_av1_point_scaling_info(core, ctx, &sei_meta->point_cr_scaling[0],
		AV1_FG_CHR_POS_SIZE, MFC_REG_D_FILM_GRAIN_18);

	__get_av1_coeffs_info(core, ctx, &sei_meta->ar_coeffs_y_plus_128[0],
		AV1_FG_LUM_AR_COEF_SIZE, MFC_REG_D_FILM_GRAIN_22);
	__get_av1_coeffs_info(core, ctx, &sei_meta->ar_coeffs_cb_plus_128[0],
		AV1_FG_CHR_AR_COEF_SIZE, MFC_REG_D_FILM_GRAIN_28);
	__get_av1_coeffs_info(core, ctx, &sei_meta->ar_coeffs_cr_plus_128[0],
		AV1_FG_CHR_AR_COEF_SIZE, MFC_REG_D_FILM_GRAIN_35);

	if (debug_level >= 5)
		mfc_core_print_av1_film_grain_info(core, ctx, sei_meta);

}
