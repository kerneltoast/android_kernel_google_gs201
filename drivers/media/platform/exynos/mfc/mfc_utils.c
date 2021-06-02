/*
 * drivers/media/platform/exynos/mfc/mfc_utils.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/soc/samsung/exynos-smc.h>

#include "mfc_utils.h"
#include "mfc_qos.h"
#include "mfc_mem.h"
#include "mfc_queue.h"

void mfc_core_handle_error(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_ctx *ctx;
	struct mfc_core_ctx *core_ctx;
	int i;

	mfc_core_err("[MSR] >>>>>>>> MFC CORE is Error state <<<<<<<<\n");
	mfc_core_change_state(core, MFCCORE_ERROR);

	mutex_lock(&dev->mfc_mutex);
	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		if (!core->core_ctx[i])
			continue;
		/* TODO: need to check two core mode */
		core_ctx = core->core_ctx[i];
		mfc_change_state(core_ctx, MFCINST_ERROR);
		ctx = core_ctx->ctx;

		/* Mark all dst buffers as having an error */
		mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->dst_buf_queue);
		/* Mark all src buffers as having an error */
		mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->src_buf_ready_queue);
		mfc_cleanup_queue(&ctx->buf_queue_lock, &core_ctx->src_buf_queue);
	}
	mutex_unlock(&dev->mfc_mutex);
}

int mfc_check_vb_with_fmt(struct mfc_fmt *fmt, struct vb2_buffer *vb)
{
	struct mfc_ctx *ctx = vb->vb2_queue->drv_priv;

	if (!fmt)
		return -EINVAL;

	if (fmt->mem_planes != vb->num_planes) {
		mfc_ctx_err("plane number is different (%d != %d)\n",
				fmt->mem_planes, vb->num_planes);
		return -EINVAL;
	}

	return 0;
}

void mfc_set_linear_stride_size(struct mfc_ctx *ctx, struct mfc_fmt *fmt)
{
	struct mfc_raw_info *raw;
	int stride_align, stride, stride_type;

	raw = &ctx->raw_buf;
	if (ctx->type == MFCINST_DECODER) {
		stride_align = ctx->dev->pdata->stride_align;
		stride = ALIGN(ctx->img_width, stride_align);
		stride_type = ctx->dev->pdata->stride_type;
	} else {
		/* Encoder stride uses the user stride */
		stride_align = 16;
		stride_type = 0;
		stride = ctx->buf_stride;
		if (stride % stride_align != 0) {
			mfc_ctx_err("[FRAME] MFC needs multiple of %dByte alignment for stride (%d)\n",
					stride_align, stride);
			stride = ALIGN(ctx->img_width, stride_align);
		}
	}

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420N:
	case V4L2_PIX_FMT_YVU420M:
		raw->stride[0] = stride;
		raw->stride[1] = ALIGN(stride >> 1, 16);
		raw->stride[2] = ALIGN(stride >> 1, 16);
		break;
	case V4L2_PIX_FMT_NV12MT_16X16:
	case V4L2_PIX_FMT_NV12MT:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12N:
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61M:
		raw->stride[0] = stride;
		raw->stride[1] = stride;
		raw->stride[2] = 0;
		break;
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV12N_10B:
	case V4L2_PIX_FMT_NV21M_S10B:
	case V4L2_PIX_FMT_NV16M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
		raw->stride[0] = S10B_8B_STRIDE(ctx->img_width);
		raw->stride[1] = S10B_8B_STRIDE(ctx->img_width);
		raw->stride[2] = 0;
		raw->stride_2bits[0] = S10B_2B_STRIDE(ctx->img_width);
		raw->stride_2bits[1] = S10B_2B_STRIDE(ctx->img_width);
		raw->stride_2bits[2] = 0;
		break;
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV21M_P010:
	case V4L2_PIX_FMT_NV61M_P210:
	case V4L2_PIX_FMT_NV16M_P210:
		if (ctx->dev->pdata->stride_type) {
			raw->stride[0] = ALIGN(ctx->img_width * 2, stride_align);
			raw->stride[1] = ALIGN(ctx->img_width * 2, stride_align);
		} else {
			raw->stride[0] = stride * 2;
			raw->stride[1] = stride * 2;
		}
		raw->stride[2] = 0;
		raw->stride_2bits[0] = 0;
		raw->stride_2bits[1] = 0;
		raw->stride_2bits[2] = 0;
		break;
	case V4L2_PIX_FMT_RGB24:
		ctx->rgb_bpp = 24;
		raw->stride[0] = stride * (ctx->rgb_bpp / 8);
		raw->stride[1] = 0;
		raw->stride[2] = 0;
		break;
	case V4L2_PIX_FMT_RGB565:
		ctx->rgb_bpp = 16;
		raw->stride[0] = stride * (ctx->rgb_bpp / 8);
		raw->stride[1] = 0;
		raw->stride[2] = 0;
		break;
	case V4L2_PIX_FMT_RGB32X:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_ARGB32:
	case V4L2_PIX_FMT_RGB32:
		ctx->rgb_bpp = 32;
		raw->stride[0] = stride * (ctx->rgb_bpp / 8);
		raw->stride[1] = 0;
		raw->stride[2] = 0;
		break;
	/* for compress format (SBWC) */
	case V4L2_PIX_FMT_NV12M_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		raw->stride[0] = SBWC_8B_STRIDE(ctx->img_width);
		raw->stride[1] = SBWC_8B_STRIDE(ctx->img_width);
		raw->stride[2] = 0;
		raw->stride_2bits[0] = SBWC_HEADER_STRIDE(ctx->img_width);
		raw->stride_2bits[1] = SBWC_HEADER_STRIDE(ctx->img_width);
		raw->stride_2bits[2] = 0;
		mfc_debug(2, "[SBWC] 8B stride [0] %d [1] %d header [0] %d [1] %d\n",
				raw->stride[0], raw->stride[1], raw->stride_2bits[0], raw->stride_2bits[1]);
		break;
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		raw->stride[0] = SBWC_10B_STRIDE(ctx->img_width);
		raw->stride[1] = SBWC_10B_STRIDE(ctx->img_width);
		raw->stride[2] = 0;
		raw->stride_2bits[0] = SBWC_HEADER_STRIDE(ctx->img_width);
		raw->stride_2bits[1] = SBWC_HEADER_STRIDE(ctx->img_width);
		raw->stride_2bits[2] = 0;
		mfc_debug(2, "[SBWC] 10B stride [0] %d [1] %d header [0] %d [1] %d\n",
				raw->stride[0], raw->stride[1], raw->stride_2bits[0], raw->stride_2bits[1]);
		break;
	/* for compress lossy format (SBWCL) */
	case V4L2_PIX_FMT_NV12M_SBWCL_8B:
		raw->stride[0] = SBWCL_8B_STRIDE(ctx->img_width, ctx->sbwcl_ratio);
		raw->stride[1] = SBWCL_8B_STRIDE(ctx->img_width, ctx->sbwcl_ratio);
		raw->stride[2] = 0;
		raw->stride_2bits[0] = 0;
		raw->stride_2bits[1] = 0;
		raw->stride_2bits[2] = 0;
		mfc_debug(2, "[SBWCL] 8B stride [0] %d [1] %d header [0] %d [1] %d\n",
				raw->stride[0], raw->stride[1], raw->stride_2bits[0], raw->stride_2bits[1]);
		break;
	case V4L2_PIX_FMT_NV12M_SBWCL_10B:
		raw->stride[0] = SBWCL_10B_STRIDE(ctx->img_width, ctx->sbwcl_ratio);
		raw->stride[1] = SBWCL_10B_STRIDE(ctx->img_width, ctx->sbwcl_ratio);
		raw->stride[2] = 0;
		raw->stride_2bits[0] = 0;
		raw->stride_2bits[1] = 0;
		raw->stride_2bits[2] = 0;
		mfc_debug(2, "[SBWCL] 10B stride [0] %d [1] %d header [0] %d [1] %d\n",
				raw->stride[0], raw->stride[1], raw->stride_2bits[0], raw->stride_2bits[1]);
		break;
	default:
		mfc_ctx_err("Invalid pixelformat : %s\n", fmt->name);
		break;
	}
}

void mfc_dec_calc_dpb_size(struct mfc_ctx *ctx)
{
	struct mfc_raw_info *raw;
	int i;
	int extra = MFC_LINEAR_BUF_SIZE;

	mfc_set_linear_stride_size(ctx, ctx->dst_fmt);

	raw = &ctx->raw_buf;
	raw->total_plane_size = 0;

	for (i = 0; i < raw->num_planes; i++) {
		raw->plane_size[i] = 0;
		raw->plane_size_2bits[i] = 0;
	}

	switch (ctx->dst_fmt->fourcc) {
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV21M_S10B:
		raw->plane_size[0] = NV12M_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = NV12M_CBCR_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = NV12M_Y_2B_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = NV12M_CBCR_2B_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) / 2 + extra;
		break;
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV21M_P010:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) / 2 + extra;
		break;
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YVU420M:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) / 2 + extra;
		raw->plane_size[2] = raw->stride[2] * ALIGN(ctx->img_height, 16) / 2 + extra;
		break;
	case V4L2_PIX_FMT_NV16M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
		raw->plane_size[0] = NV16M_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = NV16M_CBCR_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = NV16M_Y_2B_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = NV16M_CBCR_2B_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61M:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) + extra;
		break;
	case V4L2_PIX_FMT_NV16M_P210:
	case V4L2_PIX_FMT_NV61M_P210:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) + extra;
		break;
	/* non-contiguous single fd format */
	case V4L2_PIX_FMT_NV12N_10B:
		raw->plane_size[0] = NV12N_10B_Y_8B_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = NV12N_10B_CBCR_8B_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = NV12N_10B_Y_2B_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = NV12N_10B_CBCR_2B_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12N:
		raw->plane_size[0] = NV12N_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = NV12N_CBCR_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_YUV420N:
		raw->plane_size[0] = YUV420N_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = YUV420N_CB_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[2] = YUV420N_CR_SIZE(ctx->img_width, ctx->img_height);
		break;
	/* for compress format (SBWC) */
	case V4L2_PIX_FMT_NV12M_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		raw->plane_size[0] = SBWC_8B_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = SBWC_8B_CBCR_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = SBWC_8B_Y_HEADER_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = SBWC_8B_CBCR_HEADER_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		raw->plane_size[0] = SBWC_10B_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = SBWC_10B_CBCR_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = SBWC_10B_Y_HEADER_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = SBWC_10B_CBCR_HEADER_SIZE(ctx->img_width, ctx->img_height);
		break;
	default:
		mfc_ctx_err("Invalid pixelformat : %s\n", ctx->dst_fmt->name);
		break;
	}

	/*
	 * In case of 10bit,
	 * we do not update to min dpb size.
	 * Because min size may be different from the 10bit mem_type be used.
	 *
	 * If SBWC is turned off by driver,
	 * min dpb size should not be updated.
	 * Because the format is changed, the calculated value based on SBWC can't be used.
	 */
	for (i = 0; i < raw->num_planes; i++) {
		if (!ctx->is_10bit && !ctx->sbwc_disabled
				&& (raw->plane_size[i] < ctx->min_dpb_size[i])) {
			mfc_ctx_info("[FRAME] plane[%d] size is changed %d -> %d\n",
					i, raw->plane_size[i], ctx->min_dpb_size[i]);
			raw->plane_size[i] = ctx->min_dpb_size[i];
		}
		if (IS_2BIT_NEED(ctx) &&
				(raw->plane_size_2bits[i] < ctx->min_dpb_size_2bits[i])) {
			mfc_ctx_info("[FRAME] 2bit plane[%d] size is changed %d -> %d\n",
					i, raw->plane_size_2bits[i], ctx->min_dpb_size_2bits[i]);
			raw->plane_size_2bits[i] = ctx->min_dpb_size_2bits[i];
		}
	}

	for (i = 0; i < raw->num_planes; i++) {
		raw->total_plane_size += raw->plane_size[i];
		mfc_debug(2, "[FRAME] Plane[%d] size = %d, stride = %d\n",
			i, raw->plane_size[i], raw->stride[i]);
	}
	if (IS_2BIT_NEED(ctx)) {
		for (i = 0; i < raw->num_planes; i++) {
			raw->total_plane_size += raw->plane_size_2bits[i];
			mfc_debug(2, "[FRAME]%s%s Plane[%d] 2bit size = %d, stride = %d\n",
					(ctx->is_10bit ? "[10BIT]" : ""),
					(ctx->is_sbwc ? "[SBWC]" : ""),
					i, raw->plane_size_2bits[i],
					raw->stride_2bits[i]);
		}
	}
	mfc_debug(2, "[FRAME] total plane size: %d\n", raw->total_plane_size);

	if (IS_H264_DEC(ctx) || IS_H264_MVC_DEC(ctx)) {
		ctx->mv_size = DEC_MV_SIZE_MB(ctx->img_width, ctx->img_height);
		ctx->mv_size = ALIGN(ctx->mv_size, 32);
	} else if (IS_HEVC_DEC(ctx) || IS_BPG_DEC(ctx)) {
		ctx->mv_size = DEC_HEVC_MV_SIZE(ctx->img_width, ctx->img_height);
		ctx->mv_size = ALIGN(ctx->mv_size, 32);
	} else if (IS_AV1_DEC(ctx)) {
		ctx->mv_size = DEC_AV1_MV_SIZE(ctx->img_width, ctx->img_height);
		ctx->mv_size = ALIGN(ctx->mv_size, 32);
	} else {
		ctx->mv_size = 0;
	}
}

void mfc_enc_calc_src_size(struct mfc_ctx *ctx)
{
	struct mfc_raw_info *raw;
	int i, extra;

	mfc_set_linear_stride_size(ctx, ctx->src_fmt);

	raw = &ctx->raw_buf;
	raw->total_plane_size = 0;
	extra = MFC_LINEAR_BUF_SIZE;

	for (i = 0; i < raw->num_planes; i++) {
		raw->plane_size[i] = 0;
		raw->plane_size_2bits[i] = 0;
	}

	switch (ctx->src_fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420N:
	case V4L2_PIX_FMT_YVU420M:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) / 2 + extra;
		raw->plane_size[2] = raw->stride[2] * ALIGN(ctx->img_height, 16) / 2 + extra;
		break;
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV21M_S10B:
		raw->plane_size[0] = NV12M_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = NV12M_CBCR_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = NV12M_Y_2B_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = NV12M_CBCR_2B_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12N:
		raw->plane_size[0] = NV12N_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = NV12N_CBCR_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12MT_16X16:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) / 2 + extra;
		break;
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV21M_P010:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) / 2 + extra;
		break;
	case V4L2_PIX_FMT_NV16M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
		raw->plane_size[0] = NV16M_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = NV16M_CBCR_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = NV16M_Y_2B_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = NV16M_CBCR_2B_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61M:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) + extra;
		break;
	case V4L2_PIX_FMT_NV16M_P210:
	case V4L2_PIX_FMT_NV61M_P210:
		raw->plane_size[0] = raw->stride[0] * ALIGN(ctx->img_height, 16) + extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(ctx->img_height, 16) + extra;
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB32X:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_ARGB32:
	case V4L2_PIX_FMT_RGB32:
		raw->plane_size[0] = raw->stride[0] * ctx->img_height + extra;
		break;
	/* for compress format (SBWC) */
	case V4L2_PIX_FMT_NV12M_SBWC_8B:
	case V4L2_PIX_FMT_NV21M_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		raw->plane_size[0] = SBWC_8B_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = SBWC_8B_CBCR_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = SBWC_8B_Y_HEADER_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = SBWC_8B_CBCR_HEADER_SIZE(ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV21M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		raw->plane_size[0] = SBWC_10B_Y_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size[1] = SBWC_10B_CBCR_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[0] = SBWC_10B_Y_HEADER_SIZE(ctx->img_width, ctx->img_height);
		raw->plane_size_2bits[1] = SBWC_10B_CBCR_HEADER_SIZE(ctx->img_width, ctx->img_height);
		break;
	/* for compress lossy format (SBWCL) */
	case V4L2_PIX_FMT_NV12M_SBWCL_8B:
	case V4L2_PIX_FMT_NV12N_SBWCL_8B:
		raw->plane_size[0] = SBWCL_8B_Y_SIZE(ctx->img_width, ctx->img_height, ctx->sbwcl_ratio);
		raw->plane_size[1] = SBWCL_8B_CBCR_SIZE(ctx->img_width, ctx->img_height, ctx->sbwcl_ratio);
		raw->plane_size_2bits[0] = 0;
		raw->plane_size_2bits[1] = 0;
		break;
	case V4L2_PIX_FMT_NV12M_SBWCL_10B:
	case V4L2_PIX_FMT_NV12N_SBWCL_10B:
		raw->plane_size[0] = SBWCL_10B_Y_SIZE(ctx->img_width, ctx->img_height, ctx->sbwcl_ratio);
		raw->plane_size[1] = SBWCL_10B_CBCR_SIZE(ctx->img_width, ctx->img_height, ctx->sbwcl_ratio);
		raw->plane_size_2bits[0] = 0;
		raw->plane_size_2bits[1] = 0;
		break;
	default:
		mfc_ctx_err("Invalid pixel format(%d)\n", ctx->src_fmt->fourcc);
		break;
	}

	for (i = 0; i < raw->num_planes; i++) {
		if (raw->plane_size[i] < ctx->min_dpb_size[i])
			mfc_ctx_info("[FRAME] plane[%d] size %d / min size %d\n",
					i, raw->plane_size[i], ctx->min_dpb_size[i]);
	}

	for (i = 0; i < raw->num_planes; i++) {
		raw->total_plane_size += raw->plane_size[i];
		mfc_debug(2, "[FRAME] Plane[%d] size = %d, stride = %d\n",
			i, raw->plane_size[i], raw->stride[i]);
	}
	if (IS_2BIT_NEED(ctx)) {
		for (i = 0; i < raw->num_planes; i++) {
			raw->total_plane_size += raw->plane_size_2bits[i];
			mfc_debug(2, "[FRAME]%s%s Plane[%d] 2bit size = %d, stride = %d\n",
					(ctx->is_10bit ? "[10BIT]" : ""),
					(ctx->is_sbwc ? "[SBWC]" : ""),
					i, raw->plane_size_2bits[i],
					raw->stride_2bits[i]);
		}
	}

	mfc_debug(2, "[FRAME] total plane size: %d\n", raw->total_plane_size);
}

void mfc_calc_base_addr(struct mfc_ctx *ctx, struct vb2_buffer *vb,
				struct mfc_fmt *fmt)
{
	struct mfc_buf *buf = vb_to_mfc_buf(vb);
	dma_addr_t start_raw;
	int i;

	start_raw = mfc_mem_get_daddr_vb(vb, 0);

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_NV12N:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = NV12N_CBCR_BASE(start_raw, ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12N_10B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = NV12N_10B_CBCR_BASE(start_raw, ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_YUV420N:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = YUV420N_CB_BASE(start_raw, ctx->img_width, ctx->img_height);
		buf->addr[0][2] = YUV420N_CR_BASE(start_raw, ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = SBWC_8B_CBCR_BASE(start_raw, ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = SBWC_10B_CBCR_BASE(start_raw, ctx->img_width, ctx->img_height);
		break;
	case V4L2_PIX_FMT_NV12N_SBWCL_8B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = SBWCL_8B_CBCR_BASE(start_raw, ctx->img_width, ctx->img_height, ctx->sbwcl_ratio);
		break;
	case V4L2_PIX_FMT_NV12N_SBWCL_10B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = SBWCL_10B_CBCR_BASE(start_raw, ctx->img_width, ctx->img_height, ctx->sbwcl_ratio);
		break;
	default:
		for (i = 0; i < fmt->mem_planes; i++)
			buf->addr[0][i] = mfc_mem_get_daddr_vb(vb, i);
		break;
	}
}

void mfc_core_meerkat_tick(struct timer_list *t)
{
	struct mfc_core *core = from_timer(core, t, meerkat_timer);

	mfc_core_debug(5, "meerkat is ticking!\n");

	if (core->state == MFCCORE_ERROR) {
		atomic_set(&core->meerkat_tick_running, 0);
		atomic_set(&core->meerkat_tick_cnt, 0);
		mfc_core_info("[MSR] meerkat timer is now stopped! It's Error state\n");
		return;
	}

	if (atomic_read(&core->meerkat_tick_running))
		atomic_inc(&core->meerkat_tick_cnt);
	else
		atomic_set(&core->meerkat_tick_cnt, 0);

	if (atomic_read(&core->meerkat_tick_cnt) >= MEERKAT_TICK_CNT_TO_START_MEERKAT) {
		/* This means that hw is busy and no interrupts were
		 * generated by hw for the Nth time of running this
		 * meerkat timer. This usually means a serious hw
		 * error. Now it is time to kill all instances and
		 * reset the MFC. */
		mfc_core_err("[%d] Time out during waiting for HW\n",
				atomic_read(&core->meerkat_tick_cnt));
		queue_work(core->meerkat_wq, &core->meerkat_work);
	}

	mod_timer(&core->meerkat_timer, jiffies + msecs_to_jiffies(MEERKAT_TICK_INTERVAL));
}

void mfc_core_meerkat_start_tick(struct mfc_core *core)
{
	if (atomic_read(&core->meerkat_tick_running)) {
		mfc_core_debug(2, "meerkat timer was already started!\n");
	} else {
		mfc_core_debug(2, "meerkat timer is now started!\n");
		atomic_set(&core->meerkat_tick_running, 1);
	}

	/* Reset the timeout meerkat */
	atomic_set(&core->meerkat_tick_cnt, 0);
}

void mfc_core_meerkat_stop_tick(struct mfc_core *core)
{
	if (atomic_read(&core->meerkat_tick_running)) {
		mfc_core_debug(2, "meerkat timer is now stopped!\n");
		atomic_set(&core->meerkat_tick_running, 0);
	} else {
		mfc_core_debug(2, "meerkat timer was already stopped!\n");
	}

	/* Reset the timeout meerkat */
	atomic_set(&core->meerkat_tick_cnt, 0);
}

void mfc_core_meerkat_reset_tick(struct mfc_core *core)
{
	mfc_core_debug(2, "meerkat timer reset!\n");

	/* Reset the timeout meerkat */
	atomic_set(&core->meerkat_tick_cnt, 0);
}

void mfc_core_idle_checker(struct timer_list *t)
{
	struct mfc_core *core = from_timer(core, t, mfc_idle_timer);
	struct mfc_dev *dev = core->dev;

	mfc_core_debug(5, "[MFCIDLE] MFC HW idle checker is ticking!\n");

	if (perf_boost_mode) {
		mfc_core_info("[QoS][BOOST][MFCIDLE] skip control\n");
		return;
	}

	if (dev->move_ctx_cnt) {
		MFC_TRACE_RM("[MFCIDLE] migration working\n");
		mfc_core_idle_checker_start_tick(core);
		return;
	}

	if (atomic_read(&core->qos_req_cur) == 0) {
		mfc_core_debug(6, "[MFCIDLE] MFC QoS not started yet\n");
		mfc_core_idle_checker_start_tick(core);
		return;
	}

	if (atomic_read(&core->hw_run_cnt)) {
		mfc_core_idle_checker_start_tick(core);
		return;
	}

	if (atomic_read(&core->dev->queued_cnt)) {
		mfc_core_idle_checker_start_tick(core);
		return;
	}

#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
	mfc_core_change_idle_mode(core, MFC_IDLE_MODE_RUNNING);
	queue_work(core->mfc_idle_wq, &core->mfc_idle_work);
#endif
}
