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

#include <linux/smc.h>

#include "mfc_utils.h"
#include "mfc_qos.h"
#include "mfc_mem.h"

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

static int __mfc_calc_plane(int width, int height, int is_tiled)
{
	int mbX, mbY;

	mbX = (width + 15)/16;
	mbY = (height + 15)/16;

	/* Alignment for interlaced processing */
	if (is_tiled)
		mbY = (mbY + 1) / 2 * 2;

	return (mbX * 16) * (mbY * 16);
}

void mfc_set_linear_stride_size(struct mfc_ctx *ctx, struct mfc_fmt *fmt)
{
	struct mfc_raw_info *raw;
	int i;

	raw = &ctx->raw_buf;

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420N:
	case V4L2_PIX_FMT_YVU420M:
		raw->stride[0] = ALIGN(ctx->img_width, 16);
		if ((ctx->buf_stride > raw->stride[0]) &&
				(ctx->buf_stride % 16 == 0)) {
			mfc_debug(2, "[FRAME] using user stride(%d) not HW stride(%d)\n",
					ctx->buf_stride, raw->stride[0]);
			raw->stride[0] = ctx->buf_stride;
		}
		raw->stride[1] = ALIGN(raw->stride[0] >> 1, 16);
		raw->stride[2] = ALIGN(raw->stride[0] >> 1, 16);
		break;
	case V4L2_PIX_FMT_NV12MT_16X16:
	case V4L2_PIX_FMT_NV12MT:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12N:
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61M:
		raw->stride[0] = ALIGN(ctx->img_width, 16);
		raw->stride[1] = ALIGN(ctx->img_width, 16);
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
		raw->stride[0] = ALIGN(ctx->img_width, 16) * 2;
		raw->stride[1] = ALIGN(ctx->img_width, 16) * 2;
		raw->stride[2] = 0;
		raw->stride_2bits[0] = 0;
		raw->stride_2bits[1] = 0;
		raw->stride_2bits[2] = 0;
		break;
	case V4L2_PIX_FMT_RGB24:
		ctx->rgb_bpp = 24;
		raw->stride[0] = ALIGN(ctx->img_width, 16) * (ctx->rgb_bpp / 8);
		raw->stride[1] = 0;
		raw->stride[2] = 0;
		break;
	case V4L2_PIX_FMT_RGB565:
		ctx->rgb_bpp = 16;
		raw->stride[0] = ALIGN(ctx->img_width, 16) * (ctx->rgb_bpp / 8);
		raw->stride[1] = 0;
		raw->stride[2] = 0;
		break;
	case V4L2_PIX_FMT_RGB32X:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_ARGB32:
	case V4L2_PIX_FMT_RGB32:
		ctx->rgb_bpp = 32;
		raw->stride[0] = ALIGN(ctx->img_width, 16) * (ctx->rgb_bpp / 8);
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
				raw->stride[0], raw->stride[1],
				raw->stride_2bits[0], raw->stride_2bits[1]);
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
				raw->stride[0], raw->stride[1],
				raw->stride_2bits[0], raw->stride_2bits[1]);
		break;
	/* for compress lossy format (SBWCL) */
	case V4L2_PIX_FMT_NV12M_SBWCL_8B:
		raw->stride[0] = SBWCL_8B_STRIDE(ctx->img_width,
						ctx->sbwcl_ratio);
		raw->stride[1] = SBWCL_8B_STRIDE(ctx->img_width,
						ctx->sbwcl_ratio);
		raw->stride[2] = 0;
		raw->stride_2bits[0] = 0;
		raw->stride_2bits[1] = 0;
		raw->stride_2bits[2] = 0;
		mfc_debug(2, "[SBWCL] 8B stride [0] %d [1] %d header [0] %d [1] %d\n",
				raw->stride[0], raw->stride[1],
				raw->stride_2bits[0], raw->stride_2bits[1]);
		break;
	case V4L2_PIX_FMT_NV12M_SBWCL_10B:
		raw->stride[0] = SBWCL_10B_STRIDE(ctx->img_width,
				ctx->sbwcl_ratio);
		raw->stride[1] = SBWCL_10B_STRIDE(ctx->img_width,
				ctx->sbwcl_ratio);
		raw->stride[2] = 0;
		raw->stride_2bits[0] = 0;
		raw->stride_2bits[1] = 0;
		raw->stride_2bits[2] = 0;
		mfc_debug(2, "[SBWCL] 10B stride [0] %d [1] %d header [0] %d [1] %d\n",
				raw->stride[0], raw->stride[1],
				raw->stride_2bits[0], raw->stride_2bits[1]);
		break;
	case V4L2_PIX_FMT_NV12M_AFBC_8B:
		raw->stride[0] = AFBC_8B_STRIDE(ctx->img_width);
		raw->stride[1] = AFBC_8B_STRIDE(ctx->img_width);
		raw->stride[2] = 0;
		mfc_debug(2, "[AFBC] 8B stride %d\n", raw->stride[0]);
		break;
	case V4L2_PIX_FMT_NV12M_AFBC_10B:
		raw->stride[0] = AFBC_10B_STRIDE(ctx->img_width);
		raw->stride[1] = AFBC_10B_STRIDE(ctx->img_width);
		raw->stride[2] = 0;
		mfc_debug(2, "[AFBC] 10B stride %d\n", raw->stride[0]);
		break;
	default:
		mfc_ctx_err("Invalid pixelformat : %s\n", fmt->name);
		break;
	}

	/* Decoder needs multiple of 16 alignment for stride */
	if (ctx->type == MFCINST_DECODER) {
		for (i = 0; i < 3; i++)
			raw->stride[i] =
				ALIGN(raw->stride[i], 16);
	}
}

void mfc_dec_calc_dpb_size(struct mfc_ctx *ctx)
{
	struct mfc_raw_info *raw;
	int i;
	int extra = MFC_LINEAR_BUF_SIZE;
	int width = ctx->img_width, height = ctx->img_height;

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
		raw->plane_size[0] = NV12M_Y_SIZE(width, height);
		raw->plane_size[1] = NV12M_CBCR_SIZE(width, height);
		raw->plane_size_2bits[0] = NV12M_Y_2B_SIZE(width, height);
		raw->plane_size_2bits[1] = NV12M_CBCR_2B_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		raw->plane_size[0] =
			__mfc_calc_plane(width, height, 0) + extra;
		raw->plane_size[1] =
			__mfc_calc_plane(width, height, 0) / 2 + extra;
		break;
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV21M_P010:
		raw->plane_size[0] =
			__mfc_calc_plane(width, height, 0) * 2 + extra;
		raw->plane_size[1] =
			__mfc_calc_plane(width, height, 0) + extra;
		break;
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YVU420M:
		raw->plane_size[0] = raw->stride[0] * ALIGN(height, 16)
								+ extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(height, 16) / 2
								+ extra;
		raw->plane_size[2] = raw->stride[2] * ALIGN(height, 16) / 2
								+ extra;
		break;
	case V4L2_PIX_FMT_NV16M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
		raw->plane_size[0] = NV16M_Y_SIZE(width, height);
		raw->plane_size[1] = NV16M_CBCR_SIZE(width, height);
		raw->plane_size_2bits[0] = NV16M_Y_2B_SIZE(width, height);
		raw->plane_size_2bits[1] = NV16M_CBCR_2B_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61M:
		raw->plane_size[0] = __mfc_calc_plane(width, height, 0) + extra;
		raw->plane_size[1] = __mfc_calc_plane(width, height, 0) + extra;
		break;
	case V4L2_PIX_FMT_NV16M_P210:
	case V4L2_PIX_FMT_NV61M_P210:
		raw->plane_size[0] =
			__mfc_calc_plane(width, height, 0) * 2 + extra;
		raw->plane_size[1] =
			__mfc_calc_plane(width, height, 0) * 2 + extra;
		break;
	/* non-contiguous single fd format */
	case V4L2_PIX_FMT_NV12N_10B:
		raw->plane_size[0] = NV12N_10B_Y_8B_SIZE(width, height);
		raw->plane_size[1] = NV12N_10B_CBCR_8B_SIZE(width, height);
		raw->plane_size_2bits[0] =
			NV12N_10B_Y_2B_SIZE(width, height);
		raw->plane_size_2bits[1] =
			NV12N_10B_CBCR_2B_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV12N:
		raw->plane_size[0] = NV12N_Y_SIZE(width, height);
		raw->plane_size[1] = NV12N_CBCR_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_YUV420N:
		raw->plane_size[0] = YUV420N_Y_SIZE(width, height);
		raw->plane_size[1] = YUV420N_CB_SIZE(width, height);
		raw->plane_size[2] = YUV420N_CR_SIZE(width, height);
		break;
	/* for compress format (SBWC) */
	case V4L2_PIX_FMT_NV12M_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		raw->plane_size[0] = SBWC_8B_Y_SIZE(width, height);
		raw->plane_size[1] = SBWC_8B_CBCR_SIZE(width, height);
		raw->plane_size_2bits[0] =
			SBWC_8B_Y_HEADER_SIZE(width, height);
		raw->plane_size_2bits[1] =
			SBWC_8B_CBCR_HEADER_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		raw->plane_size[0] = SBWC_10B_Y_SIZE(width, height);
		raw->plane_size[1] = SBWC_10B_CBCR_SIZE(width, height);
		raw->plane_size_2bits[0] =
			SBWC_10B_Y_HEADER_SIZE(width, height);
		raw->plane_size_2bits[1] =
			SBWC_10B_CBCR_HEADER_SIZE(width, height);
		break;
	default:
		mfc_ctx_err("Invalid pixelformat : %s\n", ctx->dst_fmt->name);
		break;
	}

	/*
	 * In case of 10bit,
	 * we do not update to min dpb size.
	 * Because min size may be different from the 10bit mem_type be used.
	 */
	for (i = 0; i < raw->num_planes; i++) {
		if (!ctx->is_10bit &&
				(raw->plane_size[i] < ctx->min_dpb_size[i])) {
			mfc_ctx_info("[FRAME] plane[%d] size is changed %d -> %d\n",
					i, raw->plane_size[i], ctx->min_dpb_size[i]);
			raw->plane_size[i] = ctx->min_dpb_size[i];
		}
		if (IS_2BIT_NEED(ctx) &&
				(raw->plane_size_2bits[i] <
				 ctx->min_dpb_size_2bits[i])) {
			mfc_ctx_info("[FRAME] 2bit plane[%d] size is changed %d -> %d\n",
					i, raw->plane_size_2bits[i],
					ctx->min_dpb_size_2bits[i]);
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
	} else {
		ctx->mv_size = 0;
	}
}

void mfc_enc_calc_src_size(struct mfc_ctx *ctx)
{
	struct mfc_raw_info *raw;
	unsigned int mb_width, mb_height, default_size;
	int i, extra;
	int width = ctx->img_width, height = ctx->img_height;

	mfc_set_linear_stride_size(ctx, ctx->src_fmt);

	raw = &ctx->raw_buf;
	raw->total_plane_size = 0;
	mb_width = WIDTH_MB(width);
	mb_height = HEIGHT_MB(height);
	extra = MFC_LINEAR_BUF_SIZE;
	default_size = mb_width * mb_height * 256;

	for (i = 0; i < raw->num_planes; i++) {
		raw->plane_size[i] = 0;
		raw->plane_size_2bits[i] = 0;
	}

	switch (ctx->src_fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420N:
	case V4L2_PIX_FMT_YVU420M:
		raw->plane_size[0] = raw->stride[0] * ALIGN(height, 16)
								+ extra;
		raw->plane_size[1] = raw->stride[1] * ALIGN(height, 16) / 2
								+ extra;
		raw->plane_size[2] = raw->stride[2] * ALIGN(height, 16) / 2
								+ extra;
		break;
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV21M_S10B:
		raw->plane_size[0] = NV12M_Y_SIZE(width, height);
		raw->plane_size[1] = NV12M_CBCR_SIZE(width, height);
		raw->plane_size_2bits[0] = NV12M_Y_2B_SIZE(width, height);
		raw->plane_size_2bits[1] = NV12M_CBCR_2B_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV12N:
		raw->plane_size[0] = NV12N_Y_SIZE(width, height);
		raw->plane_size[1] = NV12N_CBCR_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV12MT_16X16:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		raw->plane_size[0] = ALIGN(default_size, 256) + extra;
		raw->plane_size[1] = ALIGN(default_size / 2, 256) + extra;
		break;
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV21M_P010:
		raw->plane_size[0] = ALIGN(default_size, 256) * 2 + extra;
		raw->plane_size[1] = ALIGN(default_size, 256) + extra;
		break;
	case V4L2_PIX_FMT_NV16M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
		raw->plane_size[0] = NV16M_Y_SIZE(width, height);
		raw->plane_size[1] = NV16M_CBCR_SIZE(width, height);
		raw->plane_size_2bits[0] = NV16M_Y_2B_SIZE(width, height);
		raw->plane_size_2bits[1] = NV16M_CBCR_2B_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61M:
		raw->plane_size[0] = ALIGN(default_size, 256) + extra;
		raw->plane_size[1] = ALIGN(default_size, 256) + extra;
		break;
	case V4L2_PIX_FMT_NV16M_P210:
	case V4L2_PIX_FMT_NV61M_P210:
		raw->plane_size[0] = ALIGN(default_size, 256) * 2 + extra;
		raw->plane_size[1] = ALIGN(default_size, 256) * 2 + extra;
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB32X:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_ARGB32:
	case V4L2_PIX_FMT_RGB32:
		raw->plane_size[0] = raw->stride[0] * height + extra;
		break;
	/* for compress format (SBWC) */
	case V4L2_PIX_FMT_NV12M_SBWC_8B:
	case V4L2_PIX_FMT_NV21M_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		raw->plane_size[0] = SBWC_8B_Y_SIZE(width, height);
		raw->plane_size[1] = SBWC_8B_CBCR_SIZE(width, height);
		raw->plane_size_2bits[0] =
			SBWC_8B_Y_HEADER_SIZE(width, height);
		raw->plane_size_2bits[1] =
			SBWC_8B_CBCR_HEADER_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV21M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		raw->plane_size[0] = SBWC_10B_Y_SIZE(width, height);
		raw->plane_size[1] = SBWC_10B_CBCR_SIZE(width, height);
		raw->plane_size_2bits[0] =
			SBWC_10B_Y_HEADER_SIZE(width, height);
		raw->plane_size_2bits[1] =
			SBWC_10B_CBCR_HEADER_SIZE(width, height);
		break;
	/* for compress lossy format (SBWCL) */
	case V4L2_PIX_FMT_NV12M_SBWCL_8B:
	case V4L2_PIX_FMT_NV12N_SBWCL_8B:
		raw->plane_size[0] = SBWCL_8B_Y_SIZE(ctx->img_width,
				ctx->img_height, ctx->sbwcl_ratio);
		raw->plane_size[1] = SBWCL_8B_CBCR_SIZE(ctx->img_width,
				ctx->img_height, ctx->sbwcl_ratio);
		raw->plane_size_2bits[0] = 0;
		raw->plane_size_2bits[1] = 0;
		break;
	case V4L2_PIX_FMT_NV12M_SBWCL_10B:
	case V4L2_PIX_FMT_NV12N_SBWCL_10B:
		raw->plane_size[0] = SBWCL_10B_Y_SIZE(ctx->img_width,
				ctx->img_height, ctx->sbwcl_ratio);
		raw->plane_size[1] = SBWCL_10B_CBCR_SIZE(ctx->img_width,
				ctx->img_height, ctx->sbwcl_ratio);
		raw->plane_size_2bits[0] = 0;
		raw->plane_size_2bits[1] = 0;
		break;
	case V4L2_PIX_FMT_NV12M_AFBC_8B:
		raw->plane_size[0] = AFBC_8B_Y_SIZE(width, height);
		break;
	case V4L2_PIX_FMT_NV12M_AFBC_10B:
		raw->plane_size[0] = AFBC_10B_Y_SIZE(width, height);
		break;
	default:
		mfc_ctx_err("Invalid pixel format(%d)\n", ctx->src_fmt->fourcc);
		break;
	}

	for (i = 0; i < raw->num_planes; i++) {
		if (raw->plane_size[i] < ctx->min_dpb_size[i])
			mfc_ctx_info("[FRAME] plane[%d] size %d / min size %d\n",
					i, raw->plane_size[i],
					ctx->min_dpb_size[i]);
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
	int width = ctx->img_width, height = ctx->img_height;

	start_raw = mfc_mem_get_daddr_vb(vb, 0);

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_NV12N:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = NV12N_CBCR_BASE(start_raw, width, height);
		break;
	case V4L2_PIX_FMT_NV12N_10B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = NV12N_10B_CBCR_BASE(start_raw, width, height);
		break;
	case V4L2_PIX_FMT_YUV420N:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = YUV420N_CB_BASE(start_raw, width, height);
		buf->addr[0][2] = YUV420N_CR_BASE(start_raw, width, height);
		break;
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = SBWC_8B_CBCR_BASE(start_raw, width, height);
		break;
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = SBWC_10B_CBCR_BASE(start_raw, width, height);
		break;
	case V4L2_PIX_FMT_NV12N_SBWCL_8B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = SBWCL_8B_CBCR_BASE(start_raw, ctx->img_width,
				ctx->img_height, ctx->sbwcl_ratio);
		break;
	case V4L2_PIX_FMT_NV12N_SBWCL_10B:
		buf->addr[0][0] = start_raw;
		buf->addr[0][1] = SBWCL_10B_CBCR_BASE(start_raw, ctx->img_width,
				ctx->img_height, ctx->sbwcl_ratio);
		break;
	default:
		for (i = 0; i < fmt->mem_planes; i++)
			buf->addr[0][i] = mfc_mem_get_daddr_vb(vb, i);
		break;
	}
}

void mfc_watchdog_tick(struct timer_list *t)
{
	struct mfc_dev *dev = from_timer(dev, t, watchdog_timer);

	mfc_dev_debug(5, "watchdog is ticking!\n");

	if (atomic_read(&dev->watchdog_tick_running))
		atomic_inc(&dev->watchdog_tick_cnt);
	else
		atomic_set(&dev->watchdog_tick_cnt, 0);

	if (atomic_read(&dev->watchdog_tick_cnt) >= WATCHDOG_TICK_CNT_TO_START_WATCHDOG) {
		/* This means that hw is busy and no interrupts were
		 * generated by hw for the Nth time of running this
		 * watchdog timer. This usually means a serious hw
		 * error. Now it is time to kill all instances and
		 * reset the MFC. */
		mfc_dev_err("[%d] Time out during waiting for HW\n",
				atomic_read(&dev->watchdog_tick_cnt));
		queue_work(dev->watchdog_wq, &dev->watchdog_work);
	}

	mod_timer(&dev->watchdog_timer, jiffies + msecs_to_jiffies(WATCHDOG_TICK_INTERVAL));
}

void mfc_watchdog_start_tick(struct mfc_dev *dev)
{
	if (atomic_read(&dev->watchdog_tick_running)) {
		mfc_dev_debug(2, "watchdog timer was already started!\n");
	} else {
		mfc_dev_debug(2, "watchdog timer is now started!\n");
		atomic_set(&dev->watchdog_tick_running, 1);
	}

	/* Reset the timeout watchdog */
	atomic_set(&dev->watchdog_tick_cnt, 0);
}

void mfc_watchdog_stop_tick(struct mfc_dev *dev)
{
	if (atomic_read(&dev->watchdog_tick_running)) {
		mfc_dev_debug(2, "watchdog timer is now stopped!\n");
		atomic_set(&dev->watchdog_tick_running, 0);
	} else {
		mfc_dev_debug(2, "watchdog timer was already stopped!\n");
	}

	/* Reset the timeout watchdog */
	atomic_set(&dev->watchdog_tick_cnt, 0);
}

void mfc_watchdog_reset_tick(struct mfc_dev *dev)
{
	mfc_dev_debug(2, "watchdog timer reset!\n");

	/* Reset the timeout watchdog */
	atomic_set(&dev->watchdog_tick_cnt, 0);
}

void mfc_idle_checker(struct timer_list *t)
{
	struct mfc_dev *dev = from_timer(dev, t, mfc_idle_timer);

	mfc_dev_debug(5, "[MFCIDLE] MFC HW idle checker is ticking!\n");

	if (perf_boost_mode) {
		mfc_dev_info("[QoS][BOOST][MFCIDLE] skip control\n");
		return;
	}

	if (atomic_read(&dev->qos_req_cur) == 0) {
		mfc_dev_debug(6, "[MFCIDLE] MFC QoS not started yet\n");
		mfc_idle_checker_start_tick(dev);
		return;
	}

	if (atomic_read(&dev->hw_run_cnt)) {
		mfc_idle_checker_start_tick(dev);
		return;
	}

	if (atomic_read(&dev->queued_cnt)) {
		mfc_idle_checker_start_tick(dev);
		return;
	}

#ifdef MFC_USE_BUS_DEVFREQ
	mfc_change_idle_mode(dev, MFC_IDLE_MODE_RUNNING);
	queue_work(dev->mfc_idle_wq, &dev->mfc_idle_work);
#endif
}
