/*
 * drivers/media/platform/exynos/mfc/mfc_dec_v4l2.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/vmalloc.h>

#include "mfc_dec_v4l2.h"
#include "mfc_dec_internal.h"
#include "mfc_rm.h"

#include "mfc_core_hwlock.h"

#include "mfc_sync.h"
#include "mfc_llc.h"
#include "mfc_slc.h"

#include "mfc_qos.h"
#include "mfc_queue.h"
#include "mfc_utils.h"
#include "mfc_buf.h"
#include "mfc_mem.h"

#define MAX_FRAME_SIZE		(2*1024*1024)

/* Find selected format description */
static struct mfc_fmt *__mfc_dec_find_format(struct mfc_ctx *ctx,
		unsigned int pixelformat)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_fmt *fmt = NULL;
	unsigned long i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (dec_formats[i].fourcc == pixelformat) {
			fmt = (struct mfc_fmt *)&dec_formats[i];
			break;
		}
	}

	if (fmt && !dev->pdata->support_10bit && (fmt->type & MFC_FMT_10BIT)) {
		mfc_ctx_err("[FRAME] 10bit is not supported\n");
		fmt = NULL;
	}
	if (fmt && !dev->pdata->support_422 && (fmt->type & MFC_FMT_422)) {
		mfc_ctx_err("[FRAME] 422 is not supported\n");
		fmt = NULL;
	}
	if (fmt && !dev->pdata->support_sbwc && (fmt->type & MFC_FMT_SBWC)) {
		mfc_ctx_err("[FRAME] SBWC is not supported\n");
		fmt = NULL;
	}
	if (fmt && !dev->pdata->support_av1_dec && (fmt->codec_mode == MFC_REG_CODEC_AV1_DEC)) {
		mfc_ctx_err("[STREAM] AV1 Decoder is not supported\n");
		fmt = NULL;
	}

	return fmt;
}

static struct v4l2_queryctrl *__mfc_dec_get_ctrl(int id)
{
	unsigned long i;

	for (i = 0; i < NUM_CTRLS; ++i)
		if (id == controls[i].id)
			return &controls[i];

	return NULL;
}

/* Check whether a ctrl value if correct */
static int __mfc_dec_check_ctrl_val(struct mfc_ctx *ctx, struct v4l2_control *ctrl)
{
	struct v4l2_queryctrl *c;

	c = __mfc_dec_get_ctrl(ctrl->id);
	if (!c) {
		mfc_ctx_err("[CTRLS] not supported control id (%#x)\n", ctrl->id);
		return -EINVAL;
	}

	if (ctrl->value < c->minimum || ctrl->value > c->maximum
		|| (c->step != 0 && ctrl->value % c->step != 0)) {
		mfc_ctx_err("[CTRLS][%s] id: %#x, invalid value (%d)\n",
				c->name, ctrl->id, ctrl->value);
		return -ERANGE;
	}

	mfc_debug(5, "[CTRLS][%s] id: %#x, value: %d (%#x)\n",
			c->name, ctrl->id, ctrl->value, ctrl->value);
	return 0;
}

/* Query capabilities of the device */
static int mfc_dec_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strncpy(cap->driver, "MFC", sizeof(cap->driver) - 1);
	strncpy(cap->card, "decoder", sizeof(cap->card) - 1);

	return 0;
}

static int __mfc_dec_enum_fmt(struct mfc_dev *dev, struct v4l2_fmtdesc *f,
		unsigned int type)
{
	struct mfc_fmt *fmt;
	unsigned long i, j = 0;

	for (i = 0; i < NUM_FORMATS; ++i) {
		if (!(dec_formats[i].type & type))
			continue;
		if (!dev->pdata->support_10bit && (dec_formats[i].type & MFC_FMT_10BIT))
			continue;
		if (!dev->pdata->support_422 && (dec_formats[i].type & MFC_FMT_422))
			continue;
		if (!dev->pdata->support_sbwc && (dec_formats[i].type & MFC_FMT_SBWC))
			continue;

		if (j == f->index) {
			fmt = &dec_formats[i];
			strlcpy(f->description, fmt->name,
				sizeof(f->description));
			f->pixelformat = fmt->fourcc;

			return 0;
		}

		++j;
	}

	return -EINVAL;
}

static int mfc_dec_enum_fmt_vid_cap_mplane(struct file *file, void *pirv,
		struct v4l2_fmtdesc *f)
{
	struct mfc_dev *dev = video_drvdata(file);

	return __mfc_dec_enum_fmt(dev, f, MFC_FMT_FRAME);
}

static int mfc_dec_enum_fmt_vid_out_mplane(struct file *file, void *prov,
		struct v4l2_fmtdesc *f)
{
	struct mfc_dev *dev = video_drvdata(file);

	return __mfc_dec_enum_fmt(dev, f, MFC_FMT_STREAM);
}

static void __mfc_dec_fix_10bit_memtype(struct mfc_ctx *ctx, unsigned int format)
{
	struct mfc_dev *dev = ctx->dev;

	switch (format) {
	case V4L2_PIX_FMT_NV12N_10B:
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV16M_S10B:
	case V4L2_PIX_FMT_NV21M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
		ctx->mem_type_10bit = 0;
		break;
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV12N_P010:
	case V4L2_PIX_FMT_NV16M_P210:
	case V4L2_PIX_FMT_NV21M_P010:
	case V4L2_PIX_FMT_NV61M_P210:
	case V4L2_PIX_FMT_NV21M_SBWC_8B:
	case V4L2_PIX_FMT_NV21M_SBWC_10B:
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		ctx->mem_type_10bit = 1;
		break;
	default:
		if (dev->pdata->P010_decoding)
			ctx->mem_type_10bit = 1;
		else
			ctx->mem_type_10bit = 0;
		break;
	}
	mfc_debug(2, "[10BIT] mem_type is %s\n", ctx->mem_type_10bit ? "P010" : "8+2");
}

static void __mfc_dec_change_format_sbwc_8bit(struct mfc_ctx *ctx)
{
	u32 org_fmt = ctx->dst_fmt->fourcc;

	switch (org_fmt) {
	case V4L2_PIX_FMT_NV12M_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
	case V4L2_PIX_FMT_NV21M_SBWC_8B:
		/* It is right format */
		break;
	case V4L2_PIX_FMT_NV12N:
	case V4L2_PIX_FMT_NV12N_P010:
	case V4L2_PIX_FMT_NV12N_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		/* change to single plane format */
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12N_SBWC_8B);
		break;
	default:
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12M_SBWC_8B);
		break;
	}
}

static void __mfc_dec_change_format_sbwc_10bit(struct mfc_ctx *ctx)
{
	u32 org_fmt = ctx->dst_fmt->fourcc;

	switch (org_fmt) {
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
	case V4L2_PIX_FMT_NV21M_SBWC_10B:
		/* It is right format */
		break;
	case V4L2_PIX_FMT_NV12N:
	case V4L2_PIX_FMT_NV12N_P010:
	case V4L2_PIX_FMT_NV12N_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		/* change to single plane format */
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12N_SBWC_10B);
		break;
	default:
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12M_SBWC_10B);
		break;
	}
}

static void __mfc_dec_change_format_8bit(struct mfc_ctx *ctx)
{
	u32 org_fmt = ctx->dst_fmt->fourcc;

	switch (org_fmt) {
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12N:
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420N:
	case V4L2_PIX_FMT_YVU420M:
	case V4L2_PIX_FMT_YVU420N:
		/* It is right format */
		break;
	case V4L2_PIX_FMT_NV12N_P010:
	case V4L2_PIX_FMT_NV12N_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		/* change to single plane format */
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12N);
		break;
	case V4L2_PIX_FMT_NV61M:
	case V4L2_PIX_FMT_NV61M_P210:
	case V4L2_PIX_FMT_NV61M_S10B:
	case V4L2_PIX_FMT_NV21M_P010:
	case V4L2_PIX_FMT_NV21M_S10B:
	case V4L2_PIX_FMT_NV21M_SBWC_8B:
	case V4L2_PIX_FMT_NV21M_SBWC_10B:
		/* change to CrCb order format */
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV21M);
		break;
	default:
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12M);
		break;
	}
}

static void __mfc_dec_change_format_8bit_422(struct mfc_ctx *ctx)
{
	u32 org_fmt = ctx->dst_fmt->fourcc;

	switch (org_fmt) {
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61M:
		/* It is right format */
		break;
	case V4L2_PIX_FMT_NV61M_P210:
	case V4L2_PIX_FMT_NV61M_S10B:
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV21M_P010:
	case V4L2_PIX_FMT_NV21M_S10B:
	case V4L2_PIX_FMT_NV21M_SBWC_8B:
	case V4L2_PIX_FMT_NV21M_SBWC_10B:
	case V4L2_PIX_FMT_YVU420M:
	case V4L2_PIX_FMT_YVU420N:
		/* change to CrCb order format */
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV61M);
		break;
	default:
		ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV16M);
		break;
	}
}

static void __mfc_dec_change_format_10bit(struct mfc_ctx *ctx)
{
	u32 org_fmt = ctx->dst_fmt->fourcc;

	if (ctx->dev->pdata->P010_decoding) {
		switch (org_fmt) {
		case V4L2_PIX_FMT_NV12M_P010:
		case V4L2_PIX_FMT_NV12N_P010:
		case V4L2_PIX_FMT_NV21M_P010:
			/* It is right format */
			break;
		case V4L2_PIX_FMT_NV12N:
		case V4L2_PIX_FMT_NV12N_10B:
		case V4L2_PIX_FMT_NV12N_SBWC_8B:
		case V4L2_PIX_FMT_NV12N_SBWC_10B:
		case V4L2_PIX_FMT_YVU420N:
			/* change to single plane format */
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12N_P010);
			break;
		case V4L2_PIX_FMT_NV61M:
		case V4L2_PIX_FMT_NV61M_P210:
		case V4L2_PIX_FMT_NV61M_S10B:
		case V4L2_PIX_FMT_NV21M:
		case V4L2_PIX_FMT_NV21M_S10B:
		case V4L2_PIX_FMT_NV21M_SBWC_8B:
		case V4L2_PIX_FMT_NV21M_SBWC_10B:
		case V4L2_PIX_FMT_YVU420M:
			/* change to CrCb order format */
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV21M_P010);
			break;
		default:
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12M_P010);
			break;
		}
	} else {
		switch (org_fmt) {
		case V4L2_PIX_FMT_NV12M_S10B:
		case V4L2_PIX_FMT_NV12N_10B:
		case V4L2_PIX_FMT_NV21M_S10B:
			/* It is right format */
			break;
		case V4L2_PIX_FMT_NV12N:
		case V4L2_PIX_FMT_NV12N_P010:
		case V4L2_PIX_FMT_NV12N_SBWC_8B:
		case V4L2_PIX_FMT_NV12N_SBWC_10B:
		case V4L2_PIX_FMT_YVU420N:
			/* change to single plane format */
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12N_10B);
			break;
		case V4L2_PIX_FMT_NV61M:
		case V4L2_PIX_FMT_NV61M_P210:
		case V4L2_PIX_FMT_NV61M_S10B:
		case V4L2_PIX_FMT_NV21M:
		case V4L2_PIX_FMT_NV21M_P010:
		case V4L2_PIX_FMT_NV21M_SBWC_8B:
		case V4L2_PIX_FMT_NV21M_SBWC_10B:
		case V4L2_PIX_FMT_YVU420M:
			/* change to CrCb order format */
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV21M_S10B);
			break;
		default:
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV12M_S10B);
			break;
		}
	}
}

static void __mfc_dec_change_format_10bit_422(struct mfc_ctx *ctx)
{
	u32 org_fmt = ctx->dst_fmt->fourcc;

	if (ctx->dev->pdata->P010_decoding) {
		switch (org_fmt) {
		case V4L2_PIX_FMT_NV16M_P210:
		case V4L2_PIX_FMT_NV61M_P210:
			/* It is right format */
			break;
		case V4L2_PIX_FMT_NV61M:
		case V4L2_PIX_FMT_NV61M_S10B:
		case V4L2_PIX_FMT_NV21M:
		case V4L2_PIX_FMT_NV21M_P010:
		case V4L2_PIX_FMT_NV21M_S10B:
		case V4L2_PIX_FMT_NV21M_SBWC_8B:
		case V4L2_PIX_FMT_NV21M_SBWC_10B:
		case V4L2_PIX_FMT_YVU420M:
		case V4L2_PIX_FMT_YVU420N:
			/* change to CrCb order format */
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV61M_P210);
			break;
		default:
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV16M_P210);
			break;
		}
	} else {
		switch (org_fmt) {
		case V4L2_PIX_FMT_NV16M_S10B:
		case V4L2_PIX_FMT_NV61M_S10B:
			/* It is right format */
			break;
		case V4L2_PIX_FMT_NV61M:
		case V4L2_PIX_FMT_NV61M_P210:
		case V4L2_PIX_FMT_NV21M:
		case V4L2_PIX_FMT_NV21M_P010:
		case V4L2_PIX_FMT_NV21M_S10B:
		case V4L2_PIX_FMT_NV21M_SBWC_8B:
		case V4L2_PIX_FMT_NV21M_SBWC_10B:
		case V4L2_PIX_FMT_YVU420M:
		case V4L2_PIX_FMT_YVU420N:
			/* change to CrCb order format */
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV61M_S10B);
			break;
		default:
			ctx->dst_fmt = __mfc_dec_find_format(ctx, V4L2_PIX_FMT_NV16M_S10B);
			break;
		}
	}
}

static void __mfc_dec_change_format_sbwc(struct mfc_ctx *ctx)
{
	u32 org_fmt = ctx->dst_fmt->fourcc;

	if (ctx->is_10bit)
		__mfc_dec_change_format_sbwc_10bit(ctx);
	else
		__mfc_dec_change_format_sbwc_8bit(ctx);

	ctx->raw_buf.num_planes = ctx->dst_fmt->num_planes;
	if (org_fmt != ctx->dst_fmt->fourcc)
		mfc_ctx_info("[FRAME][SBWC] format is changed to %s\n", ctx->dst_fmt->name);
}

static void __mfc_dec_change_format(struct mfc_ctx *ctx)
{
	u32 org_fmt = ctx->dst_fmt->fourcc;

	if (ctx->is_10bit && ctx->is_422)
		__mfc_dec_change_format_10bit_422(ctx);
	else if (ctx->is_10bit && !ctx->is_422)
		__mfc_dec_change_format_10bit(ctx);
	else if (!ctx->is_10bit && ctx->is_422)
		__mfc_dec_change_format_8bit_422(ctx);
	else
		__mfc_dec_change_format_8bit(ctx);

	ctx->raw_buf.num_planes = ctx->dst_fmt->num_planes;
	if (org_fmt != ctx->dst_fmt->fourcc)
		mfc_ctx_info("[FRAME] format is changed to %s\n", ctx->dst_fmt->name);
}

static void __mfc_dec_uncomp_format(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	u32 org_fmt = ctx->dst_fmt->fourcc;
	u32 uncomp_fmt = 0;

	uncomp_fmt = mfc_get_uncomp_format(ctx, org_fmt);
	if (uncomp_fmt) {
		dec->uncomp_fmt = __mfc_dec_find_format(ctx, uncomp_fmt);
		if (dec->uncomp_fmt)
			mfc_debug(2, "[SBWC] Uncompressed format is %s\n",
					dec->uncomp_fmt->name);
	}
}

static void __mfc_dec_update_pix_format(struct mfc_ctx *ctx, struct v4l2_format *f)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct mfc_raw_info *raw;
	int i;

	raw = &ctx->raw_buf;

	pix_fmt_mp->width = ctx->img_width;
	pix_fmt_mp->height = ctx->img_height;
	pix_fmt_mp->num_planes = ctx->dst_fmt->mem_planes;

	if (dec->is_interlaced)
		pix_fmt_mp->field = V4L2_FIELD_INTERLACED;
	else
		pix_fmt_mp->field = V4L2_FIELD_NONE;

	pix_fmt_mp->pixelformat = ctx->dst_fmt->fourcc;
	for (i = 0; i < ctx->dst_fmt->mem_planes; i++) {
		pix_fmt_mp->plane_fmt[i].bytesperline = raw->stride[i];
		if (ctx->dst_fmt->mem_planes == 1) {
			pix_fmt_mp->plane_fmt[i].sizeimage = raw->total_plane_size;
		} else {
			if (IS_2BIT_NEED(ctx))
				pix_fmt_mp->plane_fmt[i].sizeimage = raw->plane_size[i]
					+ raw->plane_size_2bits[i];
			else
				pix_fmt_mp->plane_fmt[i].sizeimage = raw->plane_size[i];
		}
	}
}

/* Get format */
static int mfc_dec_g_fmt_vid_cap_mplane(struct file *file, void *priv,
						struct v4l2_format *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_raw_info *raw;
	int ret;

	mfc_debug_enter();

	/* During g_fmt, context information is need to for only maincore */
	core = mfc_get_main_core_wait(dev, ctx);
	core_ctx = core->core_ctx[ctx->num];

	mfc_debug(2, "dec dst g_fmt, state: %d wait_state: %d\n",
			core_ctx->state, ctx->wait_state);
	MFC_TRACE_CTX("** DEC g_fmt(state:%d wait_state:%d)\n",
			core_ctx->state, ctx->wait_state);

	mutex_lock(&ctx->drc_wait_mutex);
	if (dec->disp_drc.disp_res_change) {
		__mfc_dec_update_pix_format(ctx, f);
		mutex_unlock(&ctx->drc_wait_mutex);
		return 0;
	}
	mutex_unlock(&ctx->drc_wait_mutex);

	if (core_ctx->state == MFCINST_GOT_INST ||
	    core_ctx->state == MFCINST_RES_CHANGE_INIT ||
	    core_ctx->state == MFCINST_RES_CHANGE_FLUSH ||
	    core_ctx->state == MFCINST_RES_CHANGE_END) {
		/* If there is no source buffer to parsing, we can't SEQ_START */
		mutex_lock(&ctx->drc_wait_mutex);
		if (((ctx->wait_state & WAIT_G_FMT) != 0) &&
			mfc_is_queue_count_same(&ctx->buf_queue_lock,
				&ctx->src_buf_ready_queue, 0) &&
			mfc_is_queue_count_same(&ctx->buf_queue_lock,
				&core_ctx->src_buf_queue, 0)) {
			mfc_ctx_err("There is no source buffer to parsing, keep previous resolution\n");
			mutex_unlock(&ctx->drc_wait_mutex);
			return -EAGAIN;
		}
		mutex_unlock(&ctx->drc_wait_mutex);

		/*
		 * If the MFC is parsing the header,
		 * so wait until it is finished.
		 */
		ret = mfc_wait_for_done_core_ctx(core_ctx, MFC_REG_R2H_CMD_SEQ_DONE_RET);
		if (ret) {
			if (core_ctx->int_err == MFC_REG_ERR_UNSUPPORTED_FEATURE) {
				mfc_ctx_err("header parsing failed by unsupported feature\n");
				return -EINVAL;
			}
			mfc_ctx_err("header parsing failed\n");
			return -EAGAIN;
		}
	}

	if (core_ctx->state >= MFCINST_HEAD_PARSED &&
	    core_ctx->state < MFCINST_ABORT) {
		/* This is run on CAPTURE (decode output) */
		if (IS_MULTI_MODE(ctx)) {
			mfc_ctx_info("[2CORE] start the subcore\n");
			if (mfc_rm_instance_setup(dev, ctx)) {
				mfc_ctx_err("[2CORE] failed to setup subcore\n");
				return -EAGAIN;
			}
		}

		/*
		 * The format should be changed according to various conditions.
		 * 1. compress (SBWC or not)
		 * 2. bit depth (8bit or 10bit)
		 * 3. chroma order (CbCr or CrCb)
		 * 4. SoC supported 10bit type (P010/P210 or 8+2)
		 * 5. component in memory (multi or single)
		 */
		if (ctx->is_sbwc) {
			__mfc_dec_change_format_sbwc(ctx);
			__mfc_dec_uncomp_format(ctx);
		} else {
			__mfc_dec_change_format(ctx);
		}

		if (ctx->is_10bit)
			__mfc_dec_fix_10bit_memtype(ctx, ctx->dst_fmt->fourcc);

		raw = &ctx->raw_buf;
		/* Width and height are set to the dimensions
		   of the movie, the buffer is bigger and
		   further processing stages should crop to this
		   rectangle. */
		mfc_dec_calc_dpb_size(ctx, &ctx->raw_buf, ctx->dst_fmt);

		if (IS_LOW_MEM) {
			unsigned int dpb_size;
			/*
			 * If total memory requirement is too big for this device,
			 * then it returns error.
			 * DPB size : Total plane size * the number of DPBs
			 * 5: the number of extra DPBs
			 * 3: the number of DPBs for Android framework
			 * 600MB: being used to return an error,
			 * when 8K resolution video clip is being tried to be decoded
			 */
			dpb_size = (ctx->raw_buf.total_plane_size *
					(ctx->dpb_count + MFC_EXTRA_DPB + 3));
			if (dpb_size > SZ_600M) {
				mfc_ctx_info("required memory size is too big (%dx%d, dpb: %d)\n",
						ctx->img_width, ctx->img_height, ctx->dpb_count);
				return -EINVAL;
			}
		}

		__mfc_dec_update_pix_format(ctx, f);
	}

	mutex_lock(&ctx->drc_wait_mutex);
	if ((ctx->wait_state & WAIT_G_FMT) != 0) {
		ctx->wait_state &= ~(WAIT_G_FMT);
		mfc_debug(2, "clear WAIT_G_FMT %d\n", ctx->wait_state);
		MFC_TRACE_CTX("** DEC clear WAIT_G_FMT(wait_state %d)\n", ctx->wait_state);
	}
	mutex_unlock(&ctx->drc_wait_mutex);

	mfc_debug_leave();

	return 0;
}

static int mfc_dec_g_fmt_vid_out_mplane(struct file *file, void *priv,
						struct v4l2_format *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dec *dec = ctx->dec_priv;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;

	mfc_debug_enter();

	mfc_debug(4, "dec src g_fmt\n");

	/* This is run on OUTPUT
	   The buffer contains compressed image
	   so width and height have no meaning */
	pix_fmt_mp->width = 0;
	pix_fmt_mp->height = 0;
	pix_fmt_mp->field = V4L2_FIELD_NONE;
	pix_fmt_mp->plane_fmt[0].bytesperline = dec->src_buf_size;
	pix_fmt_mp->plane_fmt[0].sizeimage = dec->src_buf_size;
	pix_fmt_mp->pixelformat = ctx->src_fmt->fourcc;
	pix_fmt_mp->num_planes = ctx->src_fmt->mem_planes;

	mfc_debug_leave();

	return 0;
}

/* Try format */
static int mfc_dec_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;

	fmt = __mfc_dec_find_format(ctx, pix_fmt_mp->pixelformat);
	if (!fmt) {
		mfc_ctx_err("Unsupported format for %s\n",
				V4L2_TYPE_IS_OUTPUT(f->type) ? "source" : "destination");
		return -EINVAL;
	}

	return 0;
}

/* Set format */
static int mfc_dec_s_fmt_vid_cap_mplane(struct file *file, void *priv,
							struct v4l2_format *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct mfc_fmt *fmt = NULL;

	mfc_debug_enter();

	if (ctx->vq_dst.streaming) {
		mfc_ctx_err("queue busy\n");
		return -EBUSY;
	}

	fmt = __mfc_dec_find_format(ctx, pix_fmt_mp->pixelformat);
	if (!fmt) {
		mfc_ctx_err("Unsupported format for destination\n");
		return -EINVAL;
	}
	ctx->dst_fmt = fmt;

	ctx->raw_buf.num_planes = ctx->dst_fmt->num_planes;
	mfc_ctx_info("[FRAME] dec dst pixelformat : %s\n", ctx->dst_fmt->name);

	mfc_debug_leave();

	return 0;
}

static int mfc_dec_s_fmt_vid_out_mplane(struct file *file, void *priv,
							struct v4l2_format *f)
{
	struct mfc_dev *dev = video_drvdata(file);
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core *core;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct mfc_fmt *fmt = NULL;
	int ret = 0;

	mfc_debug_enter();

	if (ctx->vq_src.streaming) {
		mfc_ctx_err("queue busy\n");
		return -EBUSY;
	}

	fmt = __mfc_dec_find_format(ctx, pix_fmt_mp->pixelformat);
	if (!fmt) {
		mfc_ctx_err("Unsupported format for source\n");
		return -EINVAL;
	}
	ctx->src_fmt = fmt;

	ctx->codec_mode = ctx->src_fmt->codec_mode;
	mfc_ctx_info("[STREAM] Dec src codec(%d): %s\n",
			ctx->codec_mode, ctx->src_fmt->name);

	ctx->pix_format = pix_fmt_mp->pixelformat;
	if ((pix_fmt_mp->width > 0) && (pix_fmt_mp->height > 0)) {
		ctx->img_height = pix_fmt_mp->height;
		ctx->img_width = pix_fmt_mp->width;
	}

	/* As this buffer will contain compressed data, the size is set
	 * to the maximum size. */
	if (pix_fmt_mp->plane_fmt[0].sizeimage)
		dec->src_buf_size = pix_fmt_mp->plane_fmt[0].sizeimage;
	else
		dec->src_buf_size = MAX_FRAME_SIZE;
	mfc_debug(2, "[STREAM] sizeimage: %d\n", pix_fmt_mp->plane_fmt[0].sizeimage);
	pix_fmt_mp->plane_fmt[0].bytesperline = 0;

	core = mfc_get_main_core_wait(dev, ctx);
	atomic_inc(&core->during_idle_resume);
	/* Trigger idle resume if core is in the idle mode for starting NAL_Q */
	mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

	ret = mfc_rm_instance_open(dev, ctx);
	if (ret)
		mfc_ctx_err("Failed to instance open\n");

	atomic_dec(&core->during_idle_resume);
	mfc_debug_leave();

	return ret;
}

/* Reqeust buffers */
static int mfc_dec_reqbufs(struct file *file, void *priv,
		struct v4l2_requestbuffers *reqbufs)
{
	struct mfc_dev *dev = video_drvdata(file);
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	int i, ret = 0;

	mfc_debug_enter();

	if (reqbufs->memory != V4L2_MEMORY_DMABUF) {
		mfc_ctx_err("Not supported memory type (%d)\n", reqbufs->memory);
		return -EINVAL;
	}

	if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "dec src reqbuf(%d)\n", reqbufs->count);
		/* Can only request buffers after
		   an instance has been opened.*/
		if (mfc_rm_query_state(ctx, EQUAL, MFCINST_GOT_INST)) {
			if (reqbufs->count == 0) {
				ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
				ctx->output_state = QUEUE_FREE;
				return ret;
			}

			/* Decoding */
			if (ctx->output_state != QUEUE_FREE) {
				mfc_ctx_err("Bufs have already been requested\n");
				return -EINVAL;
			}

			ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
			if (ret) {
				mfc_ctx_err("vb2_reqbufs on src failed\n");
				return ret;
			}

			ctx->output_state = QUEUE_BUFS_REQUESTED;
		}
	} else if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(4, "dec dst reqbuf(%d)\n", reqbufs->count);
		if (reqbufs->count == 0) {
			ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);

			if (!dec->inter_res_change) {
				for (i = 0; i < MFC_CORE_TYPE_NUM; i++) {
					if (ctx->op_core_num[i] == MFC_CORE_INVALID)
						break;
					core = dev->core[ctx->op_core_num[i]];

					if (core->has_llc && core->llc_on_status)
						mfc_llc_flush(core);
					if (core->has_slc && core->slc_on_status)
						mfc_slc_flush(core, ctx);

					core_ctx = core->core_ctx[ctx->num];
					mfc_release_codec_buffers(core_ctx);
				}
			}
			ctx->capture_state = QUEUE_FREE;
			return ret;
		}

		if (ctx->capture_state != QUEUE_FREE) {
			mfc_ctx_err("Bufs have already been requested\n");
			return -EINVAL;
		}

		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret) {
			mfc_ctx_err("vb2_reqbufs on capture failed\n");
			return ret;
		}

		if (reqbufs->count < ctx->dpb_count) {
			mfc_ctx_err("Not enough buffers allocated\n");
			reqbufs->count = 0;
			vb2_reqbufs(&ctx->vq_dst, reqbufs);
			return -ENOMEM;
		}

		dec->total_dpb_count = reqbufs->count;

		if (!dec->inter_res_change) {
			for (i = 0; i < MFC_CORE_TYPE_NUM; i++) {
				if (ctx->op_core_num[i] == MFC_CORE_INVALID)
					break;

				core = dev->core[ctx->op_core_num[i]];
				core_ctx = core->core_ctx[ctx->num];
				ret = mfc_alloc_codec_buffers(core_ctx);
				if (ret) {
					mfc_ctx_err("Failed to allocate decoding buffers\n");
					reqbufs->count = 0;
					vb2_reqbufs(&ctx->vq_dst, reqbufs);
					return -ENOMEM;
				}
			}
		}

		ctx->capture_state = QUEUE_BUFS_REQUESTED;

		mfc_rm_request_work(dev, MFC_WORK_TRY, ctx);
	}

	mfc_debug_leave();

	return ret;
}

/* Query buffer */
static int mfc_dec_querybuf(struct file *file, void *priv,
						   struct v4l2_buffer *buf)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	int ret;

	mfc_debug_enter();

	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(4, "dec dst querybuf\n");
		ret = vb2_querybuf(&ctx->vq_dst, buf);
		if (ret != 0) {
			mfc_ctx_err("dec dst: error in vb2_querybuf()\n");
			return ret;
		}
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "dec src querybuf\n");
		ret = vb2_querybuf(&ctx->vq_src, buf);
		if (ret != 0) {
			mfc_ctx_err("dec src: error in vb2_querybuf()\n");
			return ret;
		}
	} else {
		mfc_ctx_err("invalid buf type (%d)\n", buf->type);
		return -EINVAL;
	}

	mfc_debug_leave();

	return ret;
}

/* Queue a buffer */
static int mfc_dec_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	int ret = -EINVAL;

	mfc_debug_enter();

	if (mfc_rm_query_state(ctx, EQUAL_OR, MFCINST_ERROR)) {
		mfc_ctx_err("Call on QBUF after unrecoverable error\n");
		return -EIO;
	}

	if (!V4L2_TYPE_IS_MULTIPLANAR(buf->type)) {
		mfc_ctx_err("Invalid V4L2 Buffer for driver: type(%d)\n", buf->type);
		return -EINVAL;
	}

	if (!buf->length) {
		mfc_ctx_err("multiplanar but length is zero\n");
		return -EIO;
	}

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "dec src buf[%d] Q\n", buf->index);
		if (buf->m.planes[0].bytesused > buf->m.planes[0].length) {
			mfc_ctx_err("data size (%d) must be less than "
					"buffer size(%d)\n",
					buf->m.planes[0].bytesused,
					buf->m.planes[0].length);
			return -EIO;
		}

		mfc_qos_update_framerate(ctx, buf->m.planes[0].bytesused);
		mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

		if (!buf->m.planes[0].bytesused) {
			buf->m.planes[0].bytesused = buf->m.planes[0].length;
			mfc_debug(2, "Src size zero, changed to buf size %d\n",
					buf->m.planes[0].bytesused);
		} else {
			mfc_debug(2, "Src size = %d\n", buf->m.planes[0].bytesused);
		}

		ret = vb2_qbuf(&ctx->vq_src, NULL, buf);
	} else {
		mfc_debug(4, "dec dst buf[%d] Q\n", buf->index);
		mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);
		ret = vb2_qbuf(&ctx->vq_dst, NULL, buf);
	}

	atomic_inc(&dev->queued_cnt);

	mfc_debug_leave();
	return ret;
}

/* Dequeue a buffer */
static int mfc_dec_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dec *dec = ctx->dec_priv;
	struct dec_dpb_ref_info *dstBuf, *srcBuf;
	struct hdr10_plus_meta *dst_sei_meta, *src_sei_meta;
	struct av1_film_grain_meta *dst_av1_sei_meta, *src_av1_sei_meta;
	unsigned int *dst_sei_full, *src_sei_full;
	int ret;
	int ncount = 0;

	mfc_debug_enter();

	if (mfc_rm_query_state(ctx, EQUAL, MFCINST_ERROR)) {
		mfc_ctx_err("Call on DQBUF after unrecoverable error\n");
		return -EIO;
	}

	if (!V4L2_TYPE_IS_MULTIPLANAR(buf->type)) {
		mfc_ctx_err("Invalid V4L2 Buffer for driver: type(%d)\n", buf->type);
		return -EINVAL;
	}

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = vb2_dqbuf(&ctx->vq_src, buf, file->f_flags & O_NONBLOCK);
		mfc_debug(4, "dec src buf[%d] DQ\n", buf->index);
	} else {
		ret = vb2_dqbuf(&ctx->vq_dst, buf, file->f_flags & O_NONBLOCK);
		mfc_debug(4, "dec dst buf[%d] DQ\n", buf->index);

		if (buf->index >= MFC_MAX_BUFFERS) {
			mfc_ctx_err("buffer index[%d] range over\n", buf->index);
			return -EINVAL;
		}

		/* Memcpy from dec->ref_info to shared memory */
		if (dec->ref_info) {
			srcBuf = &dec->ref_info[buf->index];
			for (ncount = 0; ncount < MFC_MAX_BUFFERS; ncount++) {
				if (srcBuf->dpb[ncount].fd[0] == MFC_INFO_INIT_FD)
					break;
				mfc_debug(2, "[REFINFO] DQ index[%d] Released FD = %d\n",
						buf->index, srcBuf->dpb[ncount].fd[0]);
			}

			if (dec->sh_handle_dpb.vaddr != NULL) {
				dstBuf = (struct dec_dpb_ref_info *)
					dec->sh_handle_dpb.vaddr + buf->index;
				memcpy(dstBuf, srcBuf, sizeof(struct dec_dpb_ref_info));
				dstBuf->index = buf->index;
			}
		}

		/* Memcpy from dec->hdr10_plus_info to shared memory */
		if (dec->hdr10_plus_full) {
			src_sei_full = HDR10_PLUS_ADDR(dec->hdr10_plus_full, buf->index);
			if (dec->sh_handle_hdr.vaddr != NULL) {
				dst_sei_full = HDR10_PLUS_ADDR(dec->sh_handle_hdr.vaddr, buf->index);
				memcpy(dst_sei_full, src_sei_full, HDR10_PLUS_DATA_SIZE);
				if (hdr_dump == 1) {
					mfc_ctx_err("[HDR+][DUMP] SH_HANDLE data (idx %d)....\n",
							buf->index);
					print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4,
							dst_sei_full, 68, false);
				}
			}
		} else if (dec->hdr10_plus_info) {
			src_sei_meta = &dec->hdr10_plus_info[buf->index];
			if (dec->sh_handle_hdr.vaddr != NULL) {
				dst_sei_meta = (struct hdr10_plus_meta *)
					dec->sh_handle_hdr.vaddr + buf->index;
				memcpy(dst_sei_meta, src_sei_meta, sizeof(struct hdr10_plus_meta));
			}
		}

		/* Memcpy from dec->av1_film_grain_meta to shared memory */
		if (dec->av1_film_grain_info) {
			src_av1_sei_meta = &dec->av1_film_grain_info[buf->index];
			if (dec->sh_handle_av1_film_grain.vaddr != NULL) {
				dst_av1_sei_meta = (struct av1_film_grain_meta *)
					dec->sh_handle_av1_film_grain.vaddr + buf->index;
				memcpy(dst_av1_sei_meta, src_av1_sei_meta, sizeof(struct av1_film_grain_meta));
			}
		}
	}
	mfc_debug_leave();
	return ret;
}

/* Stream on */
static int mfc_dec_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	int ret = -EINVAL;

	mfc_debug_enter();

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "dec src streamon\n");
		ret = vb2_streamon(&ctx->vq_src, type);
	} else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(4, "dec dst streamon\n");
		ret = vb2_streamon(&ctx->vq_dst, type);
		if (!ret)
			mfc_rm_qos_control(ctx, MFC_QOS_ON);
	} else {
		mfc_ctx_err("unknown v4l2 buffer type\n");
	}

	mfc_debug(2, "src: %d, dst: %d, dpb_count = %d\n",
		  mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->src_buf_ready_queue),
		  mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->dst_buf_queue),
		  ctx->dpb_count);

	mfc_debug_leave();

	return ret;
}

/* Stream off, which equals to a pause */
static int mfc_dec_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	int ret = -EINVAL;

	mfc_debug_enter();

	core = mfc_get_main_core_wait(dev, ctx);
	atomic_inc(&core->during_idle_resume);
	/* Trigger idle resume if core is in the idle mode for stopping NAL_Q */
	mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "dec src streamoff\n");
		mfc_qos_reset_last_framerate(ctx);

		ret = vb2_streamoff(&ctx->vq_src, type);
	} else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(4, "dec dst streamoff\n");
		ret = vb2_streamoff(&ctx->vq_dst, type);
		if (!ret)
			mfc_rm_qos_control(ctx, MFC_QOS_OFF);
	} else {
		mfc_ctx_err("unknown v4l2 buffer type\n");
	}

	atomic_dec(&core->during_idle_resume);
	mfc_debug_leave();

	return ret;
}

/* Query a ctrl */
static int mfc_dec_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct v4l2_queryctrl *c;

	c = __mfc_dec_get_ctrl(qc->id);
	if (!c) {
		mfc_ctx_err("[CTRLS] not supported control id (%#x)\n", qc->id);
		return -EINVAL;
	}

	*qc = *c;
	return 0;
}

static int __mfc_dec_ext_info(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	int val = 0;

	val |= DEC_SET_DYNAMIC_DPB;
	val |= DEC_SET_C2_INTERFACE;
	val |= DEC_SET_BUF_FLAG_CTRL;
	val |= DEC_SET_FRAME_ERR_TYPE;
	val |= DEC_SET_OPERATING_FPS;
	val |= DEC_SET_PRIORITY;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->skype))
		val |= DEC_SET_SKYPE_FLAG;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus))
		val |= DEC_SET_HDR10_PLUS;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus_full))
		val |= DEC_SET_HDR10_PLUS_FULL;

	mfc_debug(5, "[CTRLS] ext info val: %#x\n", val);

	return val;
}

/* Get ctrl */
static int __mfc_dec_get_ctrl_val(struct mfc_ctx *ctx, struct v4l2_control *ctrl)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_ctx_ctrl *ctx_ctrl;
	int found = 0;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER:
		ctrl->value = dec->loop_filter_mpeg4;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY:
		ctrl->value = dec->display_delay;
		break;
	case V4L2_CID_CACHEABLE:
		mfc_debug(5, "it is supported only V4L2_MEMORY_MMAP\n");
		break;
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		/* These context information is need to for only maincore */
		core = mfc_get_main_core_wait(dev, ctx);
		core_ctx = core->core_ctx[ctx->num];

		if (core_ctx->state >= MFCINST_HEAD_PARSED &&
				core_ctx->state < MFCINST_ABORT) {
			ctrl->value = ctx->dpb_count;
			break;
		} else if (core_ctx->state != MFCINST_INIT) {
			mfc_ctx_err("Decoding not initialised\n");
			return -EINVAL;
		}

		/* Should wait for the header to be parsed */
		if (mfc_wait_for_done_core_ctx(core_ctx,
					MFC_REG_R2H_CMD_SEQ_DONE_RET)) {
			mfc_core_cleanup_work_bit_and_try_run(core_ctx);
			return -EIO;
		}

		if (core_ctx->state >= MFCINST_HEAD_PARSED &&
		    core_ctx->state < MFCINST_ABORT) {
			ctrl->value = ctx->dpb_count;
		} else {
			mfc_ctx_err("Decoding not initialised\n");
			return -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE:
		ctrl->value = dec->slice_enable;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_PACKED_PB:
		/* Not used */
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_CRC_ENABLE:
		ctrl->value = dec->crc_enable;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_CHECK_STATE:
		core = mfc_get_main_core_wait(dev, ctx);
		core_ctx = core->core_ctx[ctx->num];
		if (ctx->is_dpb_realloc &&
			mfc_rm_query_state(ctx, EQUAL, MFCINST_HEAD_PARSED))
			ctrl->value = MFCSTATE_DEC_S3D_REALLOC;
		/* TODO SAY: DRC is not supported yet in multi core mode */
		else if (core_ctx->state == MFCINST_RES_CHANGE_FLUSH
				|| core_ctx->state == MFCINST_RES_CHANGE_END
				|| core_ctx->state == MFCINST_HEAD_PARSED
				|| dec->inter_res_change)
			ctrl->value = MFCSTATE_DEC_RES_DETECT;
		else if (mfc_rm_query_state(ctx, EQUAL, MFCINST_FINISHING))
			ctrl->value = MFCSTATE_DEC_TERMINATING;
		else
			ctrl->value = MFCSTATE_PROCESSING;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_SEI_FRAME_PACKING:
		ctrl->value = dec->sei_parse;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_I_FRAME_DECODING:
		ctrl->value = dec->idr_decoding;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_RATE:
		ctrl->value = mfc_qos_get_framerate(ctx);
		break;
	case V4L2_CID_MPEG_MFC_GET_VERSION_INFO:
		ctrl->value = dev->pdata->ip_ver;
		break;
	case V4L2_CID_MPEG_VIDEO_QOS_RATIO:
		ctrl->value = ctx->qos_ratio;
		break;
	case V4L2_CID_MPEG_MFC_SET_DYNAMIC_DPB_MODE:
		ctrl->value = dec->is_dynamic_dpb;
		break;
	case V4L2_CID_MPEG_MFC_GET_EXT_INFO:
		ctrl->value = __mfc_dec_ext_info(ctx);
		break;
	case V4L2_CID_MPEG_MFC_GET_10BIT_INFO:
		ctrl->value = ctx->is_10bit;
		break;
	case V4L2_CID_MPEG_MFC_GET_DRIVER_INFO:
		ctrl->value = MFC_DRIVER_INFO;
		break;
	case V4L2_CID_MPEG_VIDEO_UNCOMP_FMT:
		if (dec->uncomp_fmt)
			ctrl->value = dec->uncomp_fmt->fourcc;
		else
			ctrl->value = 0;
		break;
	case V4L2_CID_MPEG_VIDEO_GET_DISPLAY_DELAY:
		/* These context information is need to for only maincore */
		core = mfc_get_main_core_wait(dev, ctx);
		core_ctx = core->core_ctx[ctx->num];
		if (core_ctx->state >= MFCINST_HEAD_PARSED) {
			ctrl->value = dec->frame_display_delay;
		} else {
			mfc_ctx_err("display delay information not parsed yet\n");
			return -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_MFC_AV1_FILM_GRAIN_PRESENT:
		ctrl->value = dec->av1_film_grain_present;
		break;
	default:
		list_for_each_entry(ctx_ctrl, &ctx->ctrls, list) {
			if (!(ctx_ctrl->type & MFC_CTRL_TYPE_GET))
				continue;

			if (ctx_ctrl->id == ctrl->id) {
				if (ctx_ctrl->get.has_new) {
					ctx_ctrl->get.has_new = 0;
					ctrl->value = ctx_ctrl->get.val;
				} else {
					mfc_debug(5, "[CTRLS] Control value "
							"is not up to date: "
							"0x%08x\n", ctrl->id);
					return -EINVAL;
				}

				found = 1;
				break;
			}
		}

		if (!found) {
			mfc_ctx_err("Invalid control: 0x%08x\n", ctrl->id);
			return -EINVAL;
		}
		break;
	}

	mfc_debug(5, "[CTRLS] get id: %#x, value: %d\n", ctrl->id, ctrl->value);

	return 0;
}

/* Get a ctrl */
static int mfc_dec_g_ctrl(struct file *file, void *priv,
			struct v4l2_control *ctrl)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	int ret = 0;

	mfc_debug_enter();
	ret = __mfc_dec_get_ctrl_val(ctx, ctrl);
	mfc_debug_leave();

	return ret;
}

/* Set a ctrl */
static int mfc_dec_s_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_ctx_ctrl *ctx_ctrl;
	int ret = 0;
	int found = 0;

	mfc_debug_enter();

	ret = __mfc_dec_check_ctrl_val(ctx, ctrl);
	if (ret)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER:
		dec->loop_filter_mpeg4 = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY:
		dec->display_delay = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE:
		dec->slice_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_PACKED_PB:
		/* Not used */
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_CRC_ENABLE:
		dec->crc_enable = ctrl->value;
		break;
	case V4L2_CID_CACHEABLE:
		mfc_debug(5, "it is supported only V4L2_MEMORY_MMAP\n");
		break;
	case V4L2_CID_MPEG_VIDEO_H264_SEI_FRAME_PACKING:
		dec->sei_parse = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_I_FRAME_DECODING:
		dec->idr_decoding = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_IMMEDIATE_DISPLAY:
		dec->immediate_display = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_DECODING_TIMESTAMP_MODE:
		dec->is_dts_mode = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_WAIT_DECODING_START:
		mutex_lock(&ctx->drc_wait_mutex);
		ctx->wait_state = ctrl->value;
		mutex_unlock(&ctx->drc_wait_mutex);
		break;
	case V4L2_CID_MPEG_MFC_SET_DUAL_DPB_MODE:
		mfc_ctx_err("[DPB] not supported CID: 0x%x\n", ctrl->id);
		break;
	case V4L2_CID_MPEG_VIDEO_QOS_RATIO:
		ctx->qos_ratio = ctrl->value;
		mfc_ctx_info("[QoS] set %d qos_ratio\n", ctrl->value);
		break;
	case V4L2_CID_MPEG_MFC_SET_DYNAMIC_DPB_MODE:
		dec->is_dynamic_dpb = ctrl->value;
		if (dec->is_dynamic_dpb == 0)
			mfc_ctx_err("[DPB] is_dynamic_dpb is 0. it has to be enabled\n");
		break;
	case V4L2_CID_MPEG_MFC_SET_USER_SHARED_HANDLE:
		if (dec->sh_handle_dpb.fd == -1) {
			dec->sh_handle_dpb.fd = ctrl->value;
			if (mfc_mem_get_user_shared_handle(ctx, &dec->sh_handle_dpb, "DPB"))
				return -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_MFC_SET_BUF_PROCESS_TYPE:
		ctx->buf_process_type = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_BLACK_BAR_DETECT:
		dec->detect_black_bar = ctrl->value;
		if (IS_BLACKBAR_OFF(ctx)) {
			mfc_ctx_info("[BLACKBAR] black bar detection doesn't work\n");
			dec->detect_black_bar = 0;
		}
		break;
	case V4L2_CID_MPEG_MFC_HDR_USER_SHARED_HANDLE:
		dec->sh_handle_hdr.fd = ctrl->value;
		if (MFC_FEATURE_SUPPORT(ctx->dev, ctx->dev->pdata->hdr10_plus_full))
			dec->sh_handle_hdr.data_size =
				HDR10_PLUS_DATA_SIZE * MFC_MAX_BUFFERS;
		else
			dec->sh_handle_hdr.data_size =
				sizeof(struct hdr10_plus_meta) * MFC_MAX_BUFFERS;
		if (mfc_mem_get_user_shared_handle(ctx, &dec->sh_handle_hdr, "HDR10+"))
			dec->sh_handle_hdr.fd = -1;
		break;
	case V4L2_CID_MPEG_MFC_AV1_FILM_GRAIN_USER_SHARED_HANDLE:
		dec->sh_handle_av1_film_grain.fd = ctrl->value;
		dec->sh_handle_av1_film_grain.data_size =
			sizeof(struct av1_film_grain_meta) * MFC_MAX_BUFFERS;
		if (mfc_mem_get_user_shared_handle(ctx, &dec->sh_handle_av1_film_grain, "FILM_G"))
			dec->sh_handle_av1_film_grain.fd = -1;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODING_ORDER:
		dec->decoding_order = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SKIP_LAZY_UNMAP:
		ctx->skip_lazy_unmap = ctrl->value;
		mfc_debug(2, "[LAZY_UNMAP] lazy unmap %s\n",
				ctx->skip_lazy_unmap ? "disable" : "enable");
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_RATE:
		ctx->operating_framerate = ctrl->value;
		mfc_rm_update_real_time(ctx);
		mfc_debug(2, "[QoS] user set the operating frame rate: %d\n", ctrl->value);
		break;
	case V4L2_CID_MPEG_VIDEO_PRIORITY:
		ctx->prio = ctrl->value;
		mfc_rm_update_real_time(ctx);
		mfc_debug(2, "[PRIO] user set priority: %d\n", ctrl->value);
		break;
	default:
		list_for_each_entry(ctx_ctrl, &ctx->ctrls, list) {
			if (!(ctx_ctrl->type & MFC_CTRL_TYPE_SET))
				continue;

			if (ctx_ctrl->id == ctrl->id) {
				ctx_ctrl->set.has_new = 1;
				ctx_ctrl->set.val = ctrl->value;

				found = 1;
				break;
			}
		}

		if (!found) {
			mfc_ctx_err("Invalid control: 0x%08x\n", ctrl->id);
			return -EINVAL;
		}
		break;
	}

	mfc_debug_leave();

	return 0;
}

static void __mfc_dec_update_disp_res(struct mfc_ctx *ctx, struct v4l2_selection *s)
{
	struct mfc_dec *dec = ctx->dec_priv;

	s->r.left = 0;
	s->r.top = 0;
	s->r.width = dec->disp_drc.width[dec->disp_drc.pop_idx];
	s->r.height = dec->disp_drc.height[dec->disp_drc.pop_idx];
	mfc_debug(2, "[FRAME] Composing info: w=%d h=%d\n", s->r.width, s->r.height);

	dec->disp_drc.disp_res_change--;
	mfc_debug(3, "[DRC] disp_res_change[%d] count %d\n",
			dec->disp_drc.pop_idx, dec->disp_drc.disp_res_change);
	dec->disp_drc.pop_idx = ++dec->disp_drc.pop_idx % MFC_MAX_DRC_FRAME;

	if (!dec->disp_drc.disp_res_change) {
		dec->disp_drc.push_idx = 0;
		dec->disp_drc.pop_idx = 0;
	}

	/*
	 * Do not clear WAIT_G_FMT except RUNNING state
	 * because the resolution change (DRC) case uses WAIT_G_FMT
	 */
	if (mfc_rm_query_state(ctx, EQUAL, MFCINST_RUNNING)
			&& (ctx->wait_state & WAIT_G_FMT) != 0) {
		ctx->wait_state &= ~(WAIT_G_FMT);
		mfc_debug(2, "clear WAIT_G_FMT %d\n", ctx->wait_state);
		MFC_TRACE_CTX("** DEC clear WAIT_G_FMT(wait_state %d)\n", ctx->wait_state);
	}
}

/* Get cropping information */
static int mfc_dec_g_selection(struct file *file, void *priv,
		struct v4l2_selection *s)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	struct mfc_dec *dec = ctx->dec_priv;

	mfc_debug_enter();

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	core = mfc_get_main_core_wait(dev, ctx);
	core_ctx = core->core_ctx[ctx->num];

	mutex_lock(&ctx->drc_wait_mutex);
	if (dec->disp_drc.disp_res_change) {
		__mfc_dec_update_disp_res(ctx, s);
		mutex_unlock(&ctx->drc_wait_mutex);
		return 0;
	}
	mutex_unlock(&ctx->drc_wait_mutex);

	if (!ready_to_get_crop(core_ctx)) {
		mfc_ctx_err("ready to get compose failed\n");
		return -EINVAL;
	}

	if (mfc_rm_query_state(ctx, EQUAL, MFCINST_RUNNING)
			&& dec->detect_black_bar && dec->black_bar_updated) {
		s->r.left = dec->black_bar.left;
		s->r.top = dec->black_bar.top;
		s->r.width = dec->black_bar.width;
		s->r.height = dec->black_bar.height;
		mfc_debug(2, "[FRAME][BLACKBAR] Cropping info: l=%d t=%d w=%d h=%d\n",
				dec->black_bar.left,
				dec->black_bar.top,
				dec->black_bar.width,
				dec->black_bar.height);
	} else {
		if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_H264 ||
			ctx->src_fmt->fourcc == V4L2_PIX_FMT_HEVC ||
			ctx->src_fmt->fourcc == V4L2_PIX_FMT_BPG) {
			s->r.left = dec->cr_left;
			s->r.top = dec->cr_top;
			s->r.width = ctx->img_width - dec->cr_left - dec->cr_right;
			s->r.height = ctx->img_height - dec->cr_top - dec->cr_bot;
			mfc_debug(2, "[FRAME] Composing info: l=%d t=%d "
					"w=%d h=%d (r=%d b=%d fw=%d fh=%d)\n",
					s->r.left, s->r.top,
					s->r.width, s->r.height,
					dec->cr_right, dec->cr_bot,
					ctx->img_width, ctx->img_height);
		} else {
			s->r.left = 0;
			s->r.top = 0;
			s->r.width = ctx->img_width;
			s->r.height = ctx->img_height;
			mfc_debug(2, "[FRAME] Composing info: w=%d h=%d fw=%d fh=%d\n",
					s->r.width, s->r.height,
					ctx->img_width, ctx->img_height);
		}
	}

	mfc_debug_leave();
	return 0;
}

static int mfc_dec_g_ext_ctrls(struct file *file, void *priv,
			struct v4l2_ext_controls *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct v4l2_ext_control *ext_ctrl;
	struct v4l2_control ctrl;
	int i;
	int ret = 0;

	if (f->which != V4L2_CTRL_CLASS_CODEC)
		return -EINVAL;

	for (i = 0; i < f->count; i++) {
		ext_ctrl = (f->controls + i);

		ctrl.id = ext_ctrl->id;

		ret = __mfc_dec_get_ctrl_val(ctx, &ctrl);
		if (ret == 0) {
			ext_ctrl->value = ctrl.value;
		} else {
			f->error_idx = i;
			break;
		}

		mfc_debug(5, "[CTRLS][%d] id: %#x, value: %d\n",
				i, ext_ctrl->id, ext_ctrl->value);
	}

	return ret;
}

/* v4l2_ioctl_ops */
static const struct v4l2_ioctl_ops mfc_dec_ioctl_ops = {
	.vidioc_querycap		= mfc_dec_querycap,
	.vidioc_enum_fmt_vid_cap	= mfc_dec_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out	= mfc_dec_enum_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= mfc_dec_g_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_out_mplane	= mfc_dec_g_fmt_vid_out_mplane,
	.vidioc_try_fmt_vid_cap_mplane	= mfc_dec_try_fmt,
	.vidioc_try_fmt_vid_out_mplane	= mfc_dec_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= mfc_dec_s_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_out_mplane	= mfc_dec_s_fmt_vid_out_mplane,
	.vidioc_reqbufs			= mfc_dec_reqbufs,
	.vidioc_querybuf		= mfc_dec_querybuf,
	.vidioc_qbuf			= mfc_dec_qbuf,
	.vidioc_dqbuf			= mfc_dec_dqbuf,
	.vidioc_streamon		= mfc_dec_streamon,
	.vidioc_streamoff		= mfc_dec_streamoff,
	.vidioc_queryctrl		= mfc_dec_queryctrl,
	.vidioc_g_ctrl			= mfc_dec_g_ctrl,
	.vidioc_s_ctrl			= mfc_dec_s_ctrl,
	.vidioc_g_selection		= mfc_dec_g_selection,
	.vidioc_g_ext_ctrls		= mfc_dec_g_ext_ctrls,
};

const struct v4l2_ioctl_ops *mfc_get_dec_v4l2_ioctl_ops(void)
{
	return &mfc_dec_ioctl_ops;
}
