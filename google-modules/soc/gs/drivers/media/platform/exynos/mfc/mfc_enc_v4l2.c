/*
 * drivers/media/platform/exynos/mfc/mfc_enc_v4l2.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "mfc_enc_v4l2.h"
#include "mfc_enc_internal.h"
#include "mfc_rm.h"

#include "mfc_core_otf.h"
#include "mfc_sync.h"
#include "mfc_llc.h"
#include "mfc_slc.h"

#include "mfc_qos.h"
#include "mfc_queue.h"
#include "mfc_utils.h"
#include "mfc_buf.h"
#include "mfc_mem.h"

static struct mfc_fmt *__mfc_enc_find_format(struct mfc_ctx *ctx,
		unsigned int pixelformat)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_fmt *fmt = NULL;
	unsigned long i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (enc_formats[i].fourcc == pixelformat) {
			fmt = (struct mfc_fmt *)&enc_formats[i];
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
	if (fmt && !dev->pdata->support_rgb && (fmt->type & MFC_FMT_RGB)) {
		mfc_ctx_err("[FRAME] RGB is not supported\n");
		fmt = NULL;
	}
	if (fmt && !dev->pdata->support_sbwc && (fmt->type & MFC_FMT_SBWC)) {
		mfc_ctx_err("[FRAME] SBWC is not supported\n");
		fmt = NULL;
	}
	if (fmt && !dev->pdata->support_sbwcl && (fmt->type & MFC_FMT_SBWCL)) {
		mfc_ctx_err("[FRAME] SBWC lossy is not supported\n");
		fmt = NULL;
	}

	return fmt;
}

static void __mfc_enc_uncomp_format(struct mfc_ctx *ctx)
{
	struct mfc_enc *enc = ctx->enc_priv;
	u32 org_fmt = ctx->src_fmt->fourcc;
	u32 uncomp_fmt = 0;

	uncomp_fmt = mfc_get_uncomp_format(ctx, org_fmt);
	if (uncomp_fmt) {
		enc->uncomp_fmt = __mfc_enc_find_format(ctx, uncomp_fmt);
		if (enc->uncomp_fmt)
			mfc_debug(2, "[SBWC] Uncompressed format is %s\n", enc->uncomp_fmt->name);
	}
}

static struct v4l2_queryctrl *__mfc_enc_get_ctrl(int id)
{
	unsigned long i;

	for (i = 0; i < NUM_CTRLS; ++i)
		if (id == controls[i].id)
			return &controls[i];
	return NULL;
}

static int __mfc_enc_check_ctrl_val(struct mfc_ctx *ctx, struct v4l2_control *ctrl)
{
	struct v4l2_queryctrl *c;
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_enc_params *p = &enc->params;

	c = __mfc_enc_get_ctrl(ctrl->id);
	if (!c) {
		mfc_ctx_err("[CTRLS] not supported control id (%#x)\n", ctrl->id);
		return -EINVAL;
	}

	if ((ctrl->id == V4L2_CID_MPEG_VIDEO_GOP_SIZE) && (ctrl->value > c->maximum)) {
		mfc_ctx_info("GOP_SIZE is changed to max(%d -> %d)\n",
                                ctrl->value, c->maximum);
		ctrl->value = c->maximum;
	}

	if (ctrl->id == V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER) {
		if ((ctrl->value & ~(1 << 16)) < c->minimum
				|| (ctrl->value & ~(1 << 16)) > c->maximum
				|| (c->step != 0 && (ctrl->value & ~(1 << 16)) % c->step != 0)) {
			mfc_ctx_err("[CTRLS][%s] Invalid control id: %#x, value: %d (%#x)\n",
					c->name, ctrl->id, ctrl->value, ctrl->value);
			return -ERANGE;
		} else {
			return 0;
		}
	}

	if (ctrl->id == V4L2_CID_MPEG_MFC_H264_BASE_PRIORITY) {
		if (ctrl->value < c->minimum
				|| (ctrl->value + p->num_hier_max_layer - 1) > c->maximum
				|| (c->step != 0 && (ctrl->value & ~(1 << 16)) % c->step != 0)) {
			mfc_ctx_err("[CTRLS][%s] Invalid control id: %#x, value: %d (%#x)\n",
					c->name, ctrl->id, ctrl->value, ctrl->value);
			return -ERANGE;
		} else {
			return 0;
		}
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

static inline int __mfc_enc_h264_profile(struct mfc_ctx *ctx, int profile)
{
	int ret = 0;

	switch (profile) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		ret = MFC_REG_E_PROFILE_H264_MAIN;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
		ret = MFC_REG_E_PROFILE_H264_HIGH;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
		ret = MFC_REG_E_PROFILE_H264_BASELINE;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
		ret = MFC_REG_E_PROFILE_H264_CONSTRAINED_BASELINE;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH:
		ret = MFC_REG_E_PROFILE_H264_CONSTRAINED_HIGH;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/* Query capabilities of the device */
static int mfc_enc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strncpy(cap->driver, "MFC", sizeof(cap->driver) - 1);
	strncpy(cap->card, "encoder", sizeof(cap->card) - 1);

	return 0;
}

static int __mfc_enc_enum_fmt(struct mfc_dev *dev, struct v4l2_fmtdesc *f,
		unsigned int type)
{
	struct mfc_fmt *fmt;
	unsigned long i, j = 0;

	for (i = 0; i < NUM_FORMATS; ++i) {
		if (!(enc_formats[i].type & type))
			continue;
		if (!dev->pdata->support_10bit && (enc_formats[i].type & MFC_FMT_10BIT))
			continue;
		if (!dev->pdata->support_422 && (enc_formats[i].type & MFC_FMT_422))
			continue;
		if (!dev->pdata->support_rgb && (enc_formats[i].type & MFC_FMT_RGB))
			continue;
		if (!dev->pdata->support_sbwc && (enc_formats[i].type & MFC_FMT_SBWC))
			continue;

		if (j == f->index) {
			fmt = &enc_formats[i];
			strlcpy(f->description, fmt->name,
				sizeof(f->description));
			f->pixelformat = fmt->fourcc;

			return 0;
		}

		++j;
	}

	return -EINVAL;
}

static int mfc_enc_enum_fmt_vid_cap_mplane(struct file *file, void *pirv,
		struct v4l2_fmtdesc *f)
{
	struct mfc_dev *dev = video_drvdata(file);

	return __mfc_enc_enum_fmt(dev, f, MFC_FMT_STREAM);
}

static int mfc_enc_enum_fmt_vid_out_mplane(struct file *file, void *prov,
		struct v4l2_fmtdesc *f)
{
	struct mfc_dev *dev = video_drvdata(file);

	return __mfc_enc_enum_fmt(dev, f, MFC_FMT_FRAME);
}

static int mfc_enc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_enc *enc = ctx->enc_priv;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct mfc_raw_info *raw;
	int i;

	mfc_debug_enter();

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(4, "enc dst g_fmt\n");
		/* This is run on output (encoder dest) */
		pix_fmt_mp->width = 0;
		pix_fmt_mp->height = 0;
		pix_fmt_mp->field = V4L2_FIELD_NONE;
		pix_fmt_mp->pixelformat = ctx->dst_fmt->fourcc;
		pix_fmt_mp->num_planes = ctx->dst_fmt->mem_planes;

		pix_fmt_mp->plane_fmt[0].bytesperline = enc->dst_buf_size;
		pix_fmt_mp->plane_fmt[0].sizeimage = enc->dst_buf_size;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "enc src g_fmt\n");
		/* This is run on capture (encoder src) */
		raw = &ctx->raw_buf;

		pix_fmt_mp->width = ctx->img_width;
		pix_fmt_mp->height = ctx->img_height;
		pix_fmt_mp->field = V4L2_FIELD_NONE;
		pix_fmt_mp->pixelformat = ctx->src_fmt->fourcc;
		pix_fmt_mp->num_planes = ctx->src_fmt->mem_planes;
		for (i = 0; i < ctx->src_fmt->mem_planes; i++) {
			pix_fmt_mp->plane_fmt[i].bytesperline = raw->stride[i];
			pix_fmt_mp->plane_fmt[i].sizeimage = raw->plane_size[i];
		}
	} else {
		mfc_ctx_err("invalid buf type (%d)\n", f->type);
		return -EINVAL;
	}

	mfc_debug_leave();

	return 0;
}

static int mfc_enc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;

	fmt = __mfc_enc_find_format(ctx, pix_fmt_mp->pixelformat);
	if (!fmt) {
		mfc_ctx_err("Unsupported format for %s\n",
				V4L2_TYPE_IS_OUTPUT(f->type) ? "source" : "destination");
		return -EINVAL;
	}

	return 0;
}

static void __mfc_enc_check_format(struct mfc_ctx *ctx)
{
	ctx->is_422 = 0;
	ctx->is_10bit = 0;
	ctx->is_sbwc = 0;
	ctx->rgb_bpp = 0;

	switch (ctx->src_fmt->fourcc) {
	case V4L2_PIX_FMT_NV16M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
	case V4L2_PIX_FMT_NV16M_P210:
	case V4L2_PIX_FMT_NV61M_P210:
		mfc_debug(2, "[FRAME][10BIT] is 422 and 10bit format\n");
		ctx->is_10bit = 1;
		ctx->is_422 = 1;
		break;
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV61M:
		mfc_debug(2, "[FRAME] is 422 format\n");
		ctx->is_422 = 1;
		break;
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV12N_P010:
	case V4L2_PIX_FMT_NV21M_S10B:
	case V4L2_PIX_FMT_NV21M_P010:
		mfc_debug(2, "[FRAME][10BIT] is 10bit format\n");
		ctx->is_10bit = 1;
		break;
	case V4L2_PIX_FMT_NV12M_SBWC_8B:
	case V4L2_PIX_FMT_NV21M_SBWC_8B:
	case V4L2_PIX_FMT_NV12N_SBWC_8B:
		mfc_debug(2, "[FRAME][SBWC] is SBWC 8bit format\n");
		ctx->is_sbwc = 1;
		break;
	case V4L2_PIX_FMT_NV12M_SBWC_10B:
	case V4L2_PIX_FMT_NV21M_SBWC_10B:
	case V4L2_PIX_FMT_NV12N_SBWC_10B:
		mfc_debug(2, "[FRAME][SBWC] is SBWC 10bit format\n");
		ctx->is_10bit = 1;
		ctx->is_sbwc = 1;
		break;
	case V4L2_PIX_FMT_NV12M_SBWCL_8B:
	case V4L2_PIX_FMT_NV12N_SBWCL_8B:
		mfc_debug(2, "[FRAME][SBWC] is SBWC Lossy 8bit format\n");
		ctx->is_sbwc_lossy = 1;
		break;
	case V4L2_PIX_FMT_NV12M_SBWCL_10B:
	case V4L2_PIX_FMT_NV12N_SBWCL_10B:
		mfc_debug(2, "[FRAME][SBWC] is SBWC Lossy 10bit format\n");
		ctx->is_10bit = 1;
		ctx->is_sbwc_lossy = 1;
		break;
	case V4L2_PIX_FMT_RGB24:
		ctx->rgb_bpp = 24;
		break;
	case V4L2_PIX_FMT_RGB565:
		ctx->rgb_bpp = 16;
		break;
	case V4L2_PIX_FMT_RGB32X:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_ARGB32:
	case V4L2_PIX_FMT_RGB32:
		ctx->rgb_bpp = 32;
		break;
	default:
		break;
	}
	mfc_debug(2, "[FRAME] 10bit: %d, 422: %d, rgb: %d, sbwc: %d lossy: %d\n",
			ctx->is_10bit, ctx->is_422, ctx->rgb_bpp, ctx->is_sbwc, ctx->is_sbwc_lossy);
}

static int __mfc_enc_check_resolution(struct mfc_ctx *ctx)
{
	int max_width = 0, max_height = 0, min_width = 0, min_height = 0, swap_check = 0;

	/* Check max resolution */
	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_HEVC_ENC:
		if (ctx->is_422) {
			max_width = 65536;
			max_height = 8192;
			swap_check = 1;
		} else {
			max_width = 8192;
			max_height = 8192;
		}
		break;
	case MFC_REG_CODEC_BPG_ENC:
		max_width = 65536;
		max_height = 8192;
		swap_check = 1;
		break;
	case MFC_REG_CODEC_H264_ENC:
	case MFC_REG_CODEC_VP8_ENC:
		max_width = 8192;
		max_height = 8192;
		break;
	case MFC_REG_CODEC_VP9_ENC:
		max_width = 4096;
		max_height = 8192;
		break;
	case MFC_REG_CODEC_MPEG4_ENC:
		max_width = 2048;
		max_height = 2048;
		break;
	case MFC_REG_CODEC_H263_ENC:
		max_width = 2048;
		max_height = 1152;
		break;
	default:
		mfc_ctx_err("Not supported codec(%d)\n", ctx->codec_mode);
		return -EINVAL;
	}

	if (swap_check) {
		if (!((ctx->crop_width < max_width && ctx->crop_height < max_height) ||
				(ctx->crop_width < max_height && ctx->crop_height < max_width))) {
			mfc_ctx_err("Resolution is too big(%dx%d > %dxi%d or %dx%d\n",
					ctx->crop_width, ctx->crop_height, max_width, max_height,
					max_height, max_width);
			return -EINVAL;
		}
	} else {
		if (ctx->crop_width > max_width || ctx->crop_height > max_height) {
			mfc_ctx_err("Resolution is too big(%dx%d > %dx%d)\n",
					ctx->crop_width, ctx->crop_height, max_width, max_height);
			return -EINVAL;
		}
	}

	/* Check min resolution */
	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_HEVC_ENC:
	case MFC_REG_CODEC_BPG_ENC:
	case MFC_REG_CODEC_VP9_ENC:
		min_width = 64;
		min_height = 64;
		break;
	case MFC_REG_CODEC_H264_ENC:
	case MFC_REG_CODEC_VP8_ENC:
	case MFC_REG_CODEC_MPEG4_ENC:
	case MFC_REG_CODEC_H263_ENC:
		min_width = 32;
		min_height = 32;
		break;
	default:
		mfc_ctx_err("Not supported codec(%d)\n", ctx->codec_mode);
		return -EINVAL;
	}

	if (ctx->crop_width < min_width || ctx->crop_height < min_height) {
		mfc_ctx_err("Resolution is too small(%dx%d < %dx%d)\n",
				ctx->crop_width, ctx->crop_height, min_width, min_height);
		return -EINVAL;
	}

	return 0;
}

static int mfc_enc_s_fmt_vid_cap_mplane(struct file *file, void *priv,
							struct v4l2_format *f)
{
	struct mfc_dev *dev = video_drvdata(file);
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_core *core;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct mfc_fmt *fmt = NULL;
	int ret = 0;

	mfc_debug_enter();

	if (ctx->vq_dst.streaming) {
		mfc_ctx_err("dst queue busy\n");
		return -EBUSY;
	}

	fmt = __mfc_enc_find_format(ctx, pix_fmt_mp->pixelformat);
	if (!fmt) {
		mfc_ctx_err("Unsupported format for destination\n");
		return -EINVAL;
	}
	ctx->dst_fmt = fmt;

	ctx->codec_mode = ctx->dst_fmt->codec_mode;
	mfc_ctx_info("[STREAM] Enc dst codec(%d) : %s\n",
			ctx->codec_mode, ctx->dst_fmt->name);

	if (ctx->otf_handle) {
		if (ctx->dst_fmt->fourcc != V4L2_PIX_FMT_H264 &&
				ctx->dst_fmt->fourcc != V4L2_PIX_FMT_HEVC) {
			mfc_ctx_err("[OTF] only H.264 and HEVC is supported\n");
			return -EINVAL;
		}
		ret = mfc_core_otf_init(ctx);
		if (ret) {
			mfc_ctx_err("[OTF] otf_init failed\n");
			mfc_core_otf_destroy(ctx);
			/* This should be no error return when VTS test case */
			if (ret == -EFAULT)
				return 0;
			return -EINVAL;
		}

		/*
		 * It takes a long time to allocate secure buffer,
		 * so it is allocated here and use default size.
		 */
		ctx->dpb_count = MFC_OTF_DEFAULT_DPB_COUNT;
		ctx->scratch_buf_size = MFC_OTF_DEFAULT_SCRATCH_SIZE;
		enc->sbwc_option = 2;
		core = mfc_get_main_core_wait(dev, ctx);
		if (!core) {
			mfc_ctx_err("There is no maincore\n");
			return -EINVAL;
		}

		if (mfc_alloc_codec_buffers(core->core_ctx[ctx->num])) {
			mfc_ctx_err("[OTF] Failed to allocate encoding buffers\n");
			return -EINVAL;
		}
	}

	if (__mfc_enc_check_resolution(ctx)) {
		mfc_ctx_err("Unsupported resolution\n");
		return -EINVAL;
	}

	enc->dst_buf_size = pix_fmt_mp->plane_fmt[0].sizeimage;
	pix_fmt_mp->plane_fmt[0].bytesperline = 0;

	core = mfc_get_main_core_wait(dev, ctx);
	atomic_inc(&core->during_idle_resume);
	/* Trigger idle resume if core is in the idle mode for starting NAL_Q */
	mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

	ret = mfc_rm_instance_open(dev, ctx);
	if (ret) {
		mfc_ctx_err("Failed to instance open\n");
		return ret;
	}

	atomic_dec(&core->during_idle_resume);
	mfc_debug_leave();
	return 0;
}

static int __mfc_enc_check_sbwcl(struct mfc_ctx *ctx, u8 pix_flag)
{
	int ret = 0;

	ctx->sbwcl_ratio = 0;
	if (ctx->is_sbwc_lossy && !ctx->is_10bit) {
		if (pix_flag == MFC_FMT_FLAG_SBWCL_50) {
			ctx->sbwcl_ratio = 50;
		} else if (pix_flag == MFC_FMT_FLAG_SBWCL_75) {
			ctx->sbwcl_ratio = 75;
		} else {
			ret = -EINVAL;
			mfc_ctx_err("sbwc ratio is wrong %#x\n", pix_flag);
		}
	} else if (ctx->is_sbwc_lossy && ctx->is_10bit) {
		if (pix_flag == MFC_FMT_FLAG_SBWCL_60) {
			ctx->sbwcl_ratio = 60;
		} else if (pix_flag == MFC_FMT_FLAG_SBWCL_80) {
			ctx->sbwcl_ratio = 80;
		} else {
			ret = -EINVAL;
			mfc_ctx_err("sbwc ratio is wrong %#x\n", pix_flag);
		}
	} else {
		ret = -EINVAL;
		mfc_ctx_err("This is not SBWC lossy format %s\n", ctx->src_fmt->name);
	}
	mfc_debug(2, "SBWC Lossy ratio is %d\n", ctx->sbwcl_ratio);
	return ret;
}

static int mfc_enc_s_fmt_vid_out_mplane(struct file *file, void *priv,
							struct v4l2_format *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	struct mfc_fmt *prev_src_fmt = NULL;
	struct mfc_fmt *fmt = NULL;
	int ret = 0;
	int i;

	mfc_debug_enter();

	if (ctx->vq_src.streaming) {
		mfc_ctx_err("src queue busy\n");
		return -EBUSY;
	}

	if (ctx->otf_handle) {
		mfc_ctx_info("[OTF] skip source s_fmt\n");
		return 0;
	}

	/* Backup previous format */
	prev_src_fmt = ctx->src_fmt;
	fmt = __mfc_enc_find_format(ctx, pix_fmt_mp->pixelformat);
	if (!fmt) {
		mfc_ctx_err("Unsupported format for source\n");
		return -EINVAL;
	}
	ctx->src_fmt = fmt;

	if (ctx->src_fmt->mem_planes != pix_fmt_mp->num_planes) {
		mfc_ctx_err("[FRAME] enc src plane number is different (%d != %d)\n",
				ctx->src_fmt->mem_planes, pix_fmt_mp->num_planes);
		return -EINVAL;
	}

	ctx->raw_buf.num_planes = ctx->src_fmt->num_planes;
	ctx->img_width = pix_fmt_mp->width;
	ctx->img_height = pix_fmt_mp->height;
	for (i = 0; i < ctx->src_fmt->mem_planes; i++)
		ctx->bytesperline[i] = pix_fmt_mp->plane_fmt[i].bytesperline;

	__mfc_enc_check_format(ctx);

	if (ctx->is_sbwc_lossy && __mfc_enc_check_sbwcl(ctx, pix_fmt_mp->flags))
		return -EINVAL;

	/* Encoder works only single core */
	core = mfc_get_main_core_wait(dev, ctx);
	core_ctx = core->core_ctx[ctx->num];

	/* Dynamic Resolution & Format Changes */
	if (core_ctx->state == MFCINST_FINISHED) {
		mfc_change_state(core_ctx, MFCINST_GOT_INST);
		if (ctx->src_fmt->fourcc != prev_src_fmt->fourcc)
			mfc_ctx_info("[DFC] Enc Dynamic Format Changed %s -> %s\n",
					prev_src_fmt->name, ctx->src_fmt->name);
		else
			mfc_ctx_info("[DRC] Enc Dynamic Resolution Changed\n");

		if (core_ctx->codec_buffer_allocated) {
			mfc_debug(2, "[DRC] Release previous codec buffer\n");

			if (core->has_llc && core->llc_on_status)
				mfc_llc_flush(core);

			if (core->has_slc && core->slc_on_status)
				mfc_slc_flush(core, ctx);

			mfc_release_codec_buffers(core_ctx);
			ret = mfc_alloc_codec_buffers(core_ctx);
			if (ret)
				mfc_err("[DRC] Failed to allocate encoding buffers\n");
		}
	}

	mfc_ctx_info("[FRAME] enc src pixelformat : %s\n", ctx->src_fmt->name);
	mfc_ctx_info("[FRAME] resolution w: %d, h: %d, Y stride: %d, C stride: %d (mb: %ld)\n",
			pix_fmt_mp->width, pix_fmt_mp->height,
			ctx->bytesperline[0], ctx->bytesperline[1], ctx->weighted_mb);

	/*
	 * It should be keep till buffer size and stride was calculated.
	 * And it can be changed to real encoding size, if user call the s_crop.
	 */
	ctx->crop_width = ctx->img_width;
	ctx->crop_height = ctx->img_height;
	mfc_enc_calc_src_size(ctx);

	if (ctx->is_sbwc)
		__mfc_enc_uncomp_format(ctx);

	ctx->output_state = QUEUE_FREE;

	mfc_debug_leave();
	return 0;
}

static int mfc_enc_g_selection(struct file *file, void *fh,
		struct v4l2_selection *s)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);

	mfc_debug_enter();

	s->r.left = ctx->crop_left;
	s->r.top = ctx->crop_top;
	s->r.width = ctx->crop_width;
	s->r.height = ctx->crop_height;

	mfc_debug(2, "[FRAME] enc crop w: %d, h: %d, offset l: %d t: %d\n",
			ctx->crop_width, ctx->crop_height, ctx->crop_left, ctx->crop_top);

	mfc_debug_leave();

	return 0;
}

static int mfc_enc_s_selection(struct file *file, void *priv,
		struct v4l2_selection *s)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);

	mfc_debug_enter();

	if (!V4L2_TYPE_IS_OUTPUT(s->type)) {
		mfc_ctx_err("not supported type (It can only in the source)\n");
		return -EINVAL;
	}

	if (s->r.left < 0 || s->r.top < 0) {
		mfc_ctx_err("[FRAME] crop position is negative\n");
		return -EINVAL;
	}

	if ((s->r.height > ctx->img_height) || (s->r.top > ctx->img_height) ||
			(s->r.width > ctx->img_width) || (s->r.left > ctx->img_width) ||
			(s->r.left > (ctx->img_width - s->r.width)) ||
			(s->r.top > (ctx->img_height - s->r.height))) {
		mfc_ctx_err("[FRAME] Out of crop range: (%d,%d,%d,%d) from %dx%d\n",
				s->r.left, s->r.top, s->r.width, s->r.height,
				ctx->img_width, ctx->img_height);
		return -EINVAL;
	}

	ctx->crop_top = s->r.top;
	ctx->crop_left = s->r.left;
	ctx->crop_height = s->r.height;
	ctx->crop_width = s->r.width;

	mfc_debug(3, "[FRAME] enc original: %dx%d, crop: %dx%d, offset l: %d t: %d\n",
			ctx->img_width, ctx->img_height,
			ctx->crop_width, ctx->crop_height,
			ctx->crop_left, ctx->crop_top);

	return 0;
}

static int mfc_enc_reqbufs(struct file *file, void *priv,
					  struct v4l2_requestbuffers *reqbufs)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	int ret = 0;

	mfc_debug_enter();

	if (reqbufs->memory != V4L2_MEMORY_DMABUF) {
		mfc_ctx_err("Not supported memory type (%d)\n", reqbufs->memory);
		return -EINVAL;
	}

	if (ctx->otf_handle) {
		mfc_ctx_info("[OTF] skip reqbufs\n");
		return 0;
	}

	if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(4, "enc dst reqbuf(%d)\n", reqbufs->count);
		if (reqbufs->count == 0) {
			ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
			ctx->capture_state = QUEUE_FREE;
			return ret;
		}

		if (ctx->capture_state != QUEUE_FREE) {
			mfc_ctx_err("invalid capture state: %d\n", ctx->capture_state);
			return -EINVAL;
		}

		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret) {
			mfc_ctx_err("error in vb2_reqbufs() for E(D)\n");
			return ret;
		}

		ctx->capture_state = QUEUE_BUFS_REQUESTED;
	} else if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "enc src reqbuf(%d)\n", reqbufs->count);
		if (reqbufs->count == 0) {
			ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
			ctx->output_state = QUEUE_FREE;
			return ret;
		}

		if (ctx->output_state != QUEUE_FREE) {
			mfc_ctx_err("invalid output state: %d\n", ctx->output_state);
			return -EINVAL;
		}

		ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
		if (ret) {
			mfc_ctx_err("error in vb2_reqbufs() for E(S)\n");
			return ret;
		}

		ctx->output_state = QUEUE_BUFS_REQUESTED;
	} else {
		mfc_ctx_err("invalid buf type (%d)\n", reqbufs->type);
		return -EINVAL;
	}

	mfc_debug_leave();

	return ret;
}

static int mfc_enc_querybuf(struct file *file, void *priv,
						   struct v4l2_buffer *buf)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	int ret = 0;

	mfc_debug_enter();

	if (ctx->otf_handle) {
		mfc_ctx_info("[OTF] skip source querybuf\n");
		return 0;
	}

	/* Encoder works only single core */
	core = mfc_get_main_core_wait(dev, ctx);
	core_ctx = core->core_ctx[ctx->num];

	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(4, "enc dst querybuf, state: %d\n", core_ctx->state);
		ret = vb2_querybuf(&ctx->vq_dst, buf);
		if (ret != 0) {
			mfc_ctx_err("enc dst: error in vb2_querybuf()\n");
			return ret;
		}
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "enc src querybuf, state: %d\n", core_ctx->state);
		ret = vb2_querybuf(&ctx->vq_src, buf);
		if (ret != 0) {
			mfc_ctx_err("enc src: error in vb2_querybuf()\n");
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
static int mfc_enc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	int i, ret = -EINVAL;

	mfc_debug_enter();

	if (ctx->otf_handle) {
		mfc_ctx_info("[OTF] skip qbuf\n");
		return 0;
	}

	/* Encoder works only single core */
	core = mfc_get_main_core_wait(dev, ctx);
	core_ctx = core->core_ctx[ctx->num];

	if (core_ctx->state == MFCINST_ERROR) {
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
		mfc_debug(4, "enc src buf[%d] Q\n", buf->index);
		if (ctx->src_fmt->mem_planes != buf->length) {
			mfc_ctx_err("number of memory container miss-match between Src planes(%d) and buffer length(%d)\n",
					ctx->src_fmt->mem_planes, buf->length);
			return -EINVAL;
		}

		for (i = 0; i < ctx->src_fmt->mem_planes; i++) {
			if (!buf->m.planes[i].bytesused) {
				mfc_debug(2, "[FRAME] enc src[%d] size zero, "
						"changed to buf size %d\n",
						i, buf->m.planes[i].length);
				buf->m.planes[i].bytesused = buf->m.planes[i].length;
			} else {
				mfc_debug(2, "[FRAME] enc src[%d] size %d\n",
						i, buf->m.planes[i].bytesused);
			}
		}

		ret = vb2_qbuf(&ctx->vq_src, NULL, buf);
	} else {
		mfc_debug(4, "enc dst buf[%d] Q\n", buf->index);
		ret = vb2_qbuf(&ctx->vq_dst, NULL, buf);
	}

	atomic_inc(&dev->queued_cnt);

	mfc_debug_leave();
	return ret;
}

/* Dequeue a buffer */
static int mfc_enc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	int ret;

	mfc_debug_enter();

	if (ctx->otf_handle) {
		mfc_ctx_info("[OTF] skip dqbuf\n");
		return 0;
	}

	/* Encoder works only single core */
	core = mfc_get_main_core_wait(dev, ctx);
	core_ctx = core->core_ctx[ctx->num];

	if (core_ctx->state == MFCINST_ERROR) {
		mfc_ctx_err("Call on DQBUF after unrecoverable error\n");
		return -EIO;
	}

	if (!V4L2_TYPE_IS_MULTIPLANAR(buf->type)) {
		mfc_ctx_err("Invalid V4L2 Buffer for driver: type(%d)\n", buf->type);
		return -EINVAL;
	}

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = vb2_dqbuf(&ctx->vq_src, buf, file->f_flags & O_NONBLOCK);
		mfc_debug(4, "enc src buf[%d] DQ\n", buf->index);
	} else {
		ret = vb2_dqbuf(&ctx->vq_dst, buf, file->f_flags & O_NONBLOCK);
		mfc_debug(4, "enc dst buf[%d] DQ\n", buf->index);
	}
	mfc_debug_leave();
	return ret;
}

/* Stream on */
static int mfc_enc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	int ret = -EINVAL;

	mfc_debug_enter();

	if (ctx->otf_handle) {
		mfc_ctx_info("[OTF] skip streamon\n");
		return 0;
	}

	if (!V4L2_TYPE_IS_MULTIPLANAR(type)) {
		mfc_ctx_err("Invalid V4L2 Buffer for driver: type(%d)\n", type);
		return -EINVAL;
	}

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "enc src streamon\n");
		ret = vb2_streamon(&ctx->vq_src, type);
		if (!ret)
			mfc_rm_qos_control(ctx, MFC_QOS_ON);
	} else {
		mfc_debug(4, "enc dst streamon\n");
		ret = vb2_streamon(&ctx->vq_dst, type);
	}

	mfc_debug(2, "src: %d, dst: %d, dpb_count = %d\n",
		  mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->src_buf_ready_queue),
		  mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->dst_buf_queue),
		  ctx->dpb_count);
	mfc_debug_leave();
	return ret;
}

/* Stream off, which equals to a pause */
static int mfc_enc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	int ret = -EINVAL;;

	mfc_debug_enter();

	if (ctx->otf_handle) {
		mfc_ctx_info("[OTF] skip streamoff\n");
		return 0;
	}

	if (!V4L2_TYPE_IS_MULTIPLANAR(type)) {
		mfc_ctx_err("Invalid V4L2 Buffer for driver: type(%d)\n", type);
		return -EINVAL;
	}

	core = mfc_get_main_core_wait(dev, ctx);
	atomic_inc(&core->during_idle_resume);
	/* Trigger idle resume if core is in the idle mode for stopping NAL_Q */
	mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "enc src streamoff\n");
		mfc_qos_reset_last_framerate(ctx);

		ret = vb2_streamoff(&ctx->vq_src, type);
		if (!ret)
			mfc_rm_qos_control(ctx, MFC_QOS_OFF);
	} else {
		mfc_debug(4, "enc dst streamoff\n");
		ret = vb2_streamoff(&ctx->vq_dst, type);
	}

	atomic_dec(&core->during_idle_resume);
	mfc_debug_leave();

	return ret;
}

/* Query a ctrl */
static int mfc_enc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	struct mfc_dev *dev = video_drvdata(file);
	struct v4l2_queryctrl *c;

	c = __mfc_enc_get_ctrl(qc->id);
	if (!c) {
		mfc_dev_err("[CTRLS] not supported control id (%#x)\n", qc->id);
		return -EINVAL;
	}

	*qc = *c;
	return 0;
}

static int __mfc_enc_ext_info(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core = mfc_get_main_core_wait(dev, ctx);
	int val = 0;

	val |= ENC_SET_SPARE_SIZE;
	val |= ENC_SET_TEMP_SVC_CH;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->skype))
		val |= ENC_SET_SKYPE_FLAG;

	val |= ENC_SET_ROI_CONTROL;
	val |= ENC_SET_QP_BOUND_PB;
	val |= ENC_SET_FIXED_SLICE;
	val |= ENC_SET_PVC_MODE;
	val |= ENC_SET_RATIO_OF_INTRA;
	val |= ENC_SET_DROP_CONTROL;
	val |= ENC_SET_CHROMA_QP_CONTROL;
	val |= ENC_SET_BUF_FLAG_CTRL;
	val |= ENC_SET_OPERATING_FPS;
	val |= ENC_SET_GOP_CTRL;
	val |= ENC_SET_PRIORITY;
	val |= ENC_SET_QPE_TWO_PASS_ENABLE;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->color_aspect_enc))
		val |= ENC_SET_COLOR_ASPECT;

	val |= ENC_SET_HP_BITRATE_CONTROL;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->static_info_enc))
		val |= ENC_SET_STATIC_INFO;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus))
		val |= ENC_SET_HDR10_PLUS;

	if (dev->pdata->support_422)
		val |= ENC_SET_VP9_PROFILE_LEVEL;

	if (core->core_pdata->gdc_votf_base)
		val |= ENC_SET_GDC_VOTF;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->average_qp))
		val |= ENC_SET_AVERAGE_QP;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->mv_search_mode))
		val |= ENC_SET_MV_SEARCH_MODE;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->enc_capability))
		val |= ENC_SET_CAPABILITY;

	mfc_debug(5, "[CTRLS] ext info val: %#x\n", val);

	return val;
}

static int __mfc_enc_get_ctrl_val(struct mfc_ctx *ctx, struct v4l2_control *ctrl)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_ctx_ctrl *ctx_ctrl;
	int ret = 0;
	int found = 0;

	switch (ctrl->id) {
	case V4L2_CID_CACHEABLE:
		mfc_debug(5, "it is supported only V4L2_MEMORY_MMAP\n");
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_STREAM_SIZE:
		ctrl->value = enc->dst_buf_size;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TYPE:
		ctrl->value = enc->frame_type;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_CHECK_STATE:
		ctrl->value = MFCSTATE_PROCESSING;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG:
	case V4L2_CID_MPEG_MFC51_VIDEO_LUMA_ADDR:
	case V4L2_CID_MPEG_MFC51_VIDEO_CHROMA_ADDR:
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_STATUS:
	case V4L2_CID_MPEG_VIDEO_SRC_BUF_FLAG:
	case V4L2_CID_MPEG_VIDEO_DST_BUF_FLAG:
	case V4L2_CID_MPEG_VIDEO_AVERAGE_QP:
	case V4L2_CID_MPEG_VIDEO_SUM_SKIP_MB:
	case V4L2_CID_MPEG_VIDEO_SUM_INTRA_MB:
	case V4L2_CID_MPEG_VIDEO_SUM_ZERO_MV_MB:
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
					if (ctrl->id == V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG)
						ctrl->value = IGNORE_TAG;
					else
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
	case V4L2_CID_MPEG_MFC_GET_VERSION_INFO:
		ctrl->value = dev->pdata->ip_ver;
		break;
	case V4L2_CID_MPEG_MFC_GET_DRIVER_INFO:
		ctrl->value = MFC_DRIVER_INFO;
		break;
	case V4L2_CID_MPEG_MFC_GET_EXTRA_BUFFER_SIZE:
		ctrl->value = MFC_LINEAR_BUF_SIZE;
		break;
	case V4L2_CID_MPEG_VIDEO_QOS_RATIO:
		ctrl->value = ctx->qos_ratio;
		break;
	case V4L2_CID_MPEG_MFC_GET_EXT_INFO:
		ctrl->value = __mfc_enc_ext_info(ctx);
		break;
	case V4L2_CID_MPEG_VIDEO_BPG_HEADER_SIZE:
		ctrl->value = enc->header_size;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_RATE:
		ctrl->value = mfc_qos_get_framerate(ctx);
		break;
	default:
		mfc_ctx_err("Invalid control: 0x%08x\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	mfc_debug(5, "[CTRLS] get id: %#x, value: %d\n", ctrl->id, ctrl->value);

	return ret;
}

static int mfc_enc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	int ret = 0;

	mfc_debug_enter();
	ret = __mfc_enc_get_ctrl_val(ctx, ctrl);
	mfc_debug_leave();

	return ret;
}

static inline int __mfc_enc_h264_level(enum v4l2_mpeg_video_h264_level lvl)
{
	static unsigned int t[V4L2_MPEG_VIDEO_H264_LEVEL_6_0 + 1] = {
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1_0   */ 10,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1B    */ 9,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1_1   */ 11,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1_2   */ 12,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1_3   */ 13,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_2_0   */ 20,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_2_1   */ 21,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_2_2   */ 22,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_3_0   */ 30,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_3_1   */ 31,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_3_2   */ 32,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_4_0   */ 40,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_4_1   */ 41,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_4_2   */ 42,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_5_0   */ 50,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_5_1   */ 51,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_5_2   */ 52,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_6_0   */ 60,
	};
	return t[lvl];
}

static inline int __mfc_enc_mpeg4_level(enum v4l2_mpeg_video_mpeg4_level lvl)
{
	static unsigned int t[V4L2_MPEG_VIDEO_MPEG4_LEVEL_6 + 1] = {
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_0	     */ 0,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_0B, Simple    */ 9,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_1	     */ 1,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_2	     */ 2,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_3	     */ 3,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_3B, Advanced  */ 7,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_4	     */ 4,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_5	     */ 5,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_6,  Simple    */ 6,
	};
	return t[lvl];
}

static inline int __mfc_enc_vui_sar_idc(enum v4l2_mpeg_video_h264_vui_sar_idc sar)
{
	static unsigned int t[V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED + 1] = {
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED     */ 0,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_1x1             */ 1,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_12x11           */ 2,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_10x11           */ 3,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_16x11           */ 4,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_40x33           */ 5,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_24x11           */ 6,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_20x11           */ 7,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_32x11           */ 8,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_80x33           */ 9,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_18x11           */ 10,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_15x11           */ 11,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_64x33           */ 12,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_160x99          */ 13,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_4x3             */ 14,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_3x2             */ 15,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_2x1             */ 16,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED        */ 255,
	};
	return t[sar];
}

static int __mfc_enc_get_roi(struct mfc_ctx *ctx, int value)
{
	struct mfc_enc *enc = ctx->enc_priv;
	int index = 0;

	if (enc->sh_handle_roi.fd == -1) {
		enc->sh_handle_roi.fd = value;
		if (mfc_mem_get_user_shared_handle(ctx, &enc->sh_handle_roi, "ROI"))
			return -EINVAL;
	}
	index = enc->roi_index;

	/* Copy the ROI info from shared buf */
	memcpy(&enc->roi_info[index], enc->sh_handle_roi.vaddr,
			sizeof(struct mfc_enc_roi_info));
	if (enc->roi_info[index].size > enc->roi_buf[index].size) {
		mfc_ctx_err("[MEMINFO][ROI] roi info size %d is over\n",
				enc->roi_info[index].size);
		return -EINVAL;
	}

	/* Copy the ROI map buffer from user's map buf */
	if (copy_from_user(enc->roi_buf[index].vaddr,
				enc->roi_info[index].addr,
				enc->roi_info[index].size))
		return -EFAULT;

	return 0;
}

static int __mfc_enc_set_param(struct mfc_ctx *ctx, struct v4l2_control *ctrl)
{
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_enc_params *p = &enc->params;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_CACHEABLE:
		mfc_debug(5, "it is supported only V4L2_MEMORY_MMAP\n");
		break;
	case V4L2_CID_MPEG_VIDEO_QOS_RATIO:
		ctx->qos_ratio = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_PRIORITY:
		ctx->prio = ctrl->value;
		mfc_rm_update_real_time(ctx);
		mfc_debug(2, "[PRIO] user set priority: %d\n", ctrl->value);
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		p->gop_size = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		p->slice_mode =
			(enum v4l2_mpeg_video_multi_slice_mode)ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		p->slice_mb = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES:
		p->slice_bit = ctrl->value * 8;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB_ROW:
		p->slice_mb_row = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB:
		p->intra_refresh_mb = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_PADDING:
		p->pad = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_PADDING_YUV:
		p->pad_luma = (ctrl->value >> 16) & 0xff;
		p->pad_cb = (ctrl->value >> 8) & 0xff;
		p->pad_cr = (ctrl->value >> 0) & 0xff;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		p->rc_frame = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		p->rc_bitrate = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_RC_REACTION_COEFF:
		p->rc_reaction_coeff = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE:
		enc->force_frame_type = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VBV_SIZE:
		p->vbv_buf_size = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		p->seq_hdr_mode =
			(enum v4l2_mpeg_video_header_mode)(ctrl->value);
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE:
		p->frame_skip_mode =
			(enum v4l2_mpeg_mfc51_video_frame_skip_mode)
			(ctrl->value);
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_RC_FIXED_TARGET_BIT:
		p->fixed_target_bit = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_B_FRAMES:
		p->num_b_frame = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		p->codec.h264.profile =
		__mfc_enc_h264_profile(ctx, (enum v4l2_mpeg_video_h264_profile)(ctrl->value));
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		p->codec.h264.level =
		__mfc_enc_h264_level((enum v4l2_mpeg_video_h264_level)(ctrl->value));
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_INTERLACE:
		p->codec.h264.interlace = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE:
		p->codec.h264.loop_filter_mode =
		(enum v4l2_mpeg_video_h264_loop_filter_mode)(ctrl->value);
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA:
		p->codec.h264.loop_filter_alpha = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA:
		p->codec.h264.loop_filter_beta = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		p->codec.h264.entropy_mode =
			(enum v4l2_mpeg_video_h264_entropy_mode)(ctrl->value);
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_NUM_REF_PIC_FOR_P:
		p->num_refs_for_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM:
		p->codec.h264._8x8_transform = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		p->rc_mb = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_RC_FRAME_RATE:
		p->rc_framerate = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		p->codec.h264.rc_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		p->codec.h264.rc_min_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		p->codec.h264.rc_max_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP_P:
		p->codec.h264.rc_min_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP_P:
		p->codec.h264.rc_max_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP_B:
		p->codec.h264.rc_min_qp_b = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP_B:
		p->codec.h264.rc_max_qp_b = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_DARK:
		p->codec.h264.rc_mb_dark = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_SMOOTH:
		p->codec.h264.rc_mb_smooth = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_STATIC:
		p->codec.h264.rc_mb_static = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_ACTIVITY:
		p->codec.h264.rc_mb_activity = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		p->codec.h264.rc_p_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP:
		p->codec.h264.rc_b_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE:
		p->codec.h264.ar_vui = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC:
		p->codec.h264.ar_vui_idc =
		__mfc_enc_vui_sar_idc((enum v4l2_mpeg_video_h264_vui_sar_idc)(ctrl->value));
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH:
		p->codec.h264.ext_sar_width = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT:
		p->codec.h264.ext_sar_height = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_CLOSURE:
		p->codec.h264.open_gop = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		p->codec.h264.open_gop_size = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING:
		p->codec.h264.hier_qp_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_TYPE:
		p->codec.h264.hier_qp_type =
		(enum v4l2_mpeg_video_h264_hierarchical_coding_type)(ctrl->value);
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER:
		p->codec.h264.num_hier_layer = ctrl->value & 0x7;
		p->codec.h264.hier_ref_type = (ctrl->value >> 16) & 0x1;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_QP:
		p->codec.h264.hier_qp_layer[(ctrl->value >> 16) & 0x7]
			= ctrl->value & 0xFF;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_BIT0:
		p->codec.h264.hier_bit_layer[0] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_BIT1:
		p->codec.h264.hier_bit_layer[1] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_BIT2:
		p->codec.h264.hier_bit_layer[2] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_BIT3:
		p->codec.h264.hier_bit_layer[3] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_BIT4:
		p->codec.h264.hier_bit_layer[4] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_BIT5:
		p->codec.h264.hier_bit_layer[5] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_BIT6:
		p->codec.h264.hier_bit_layer[6] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_SEI_FRAME_PACKING:
		p->codec.h264.sei_gen_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_SEI_FP_CURRENT_FRAME_0:
		p->codec.h264.sei_fp_curr_frame_0 = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE:
		p->codec.h264.sei_fp_arrangement_type = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_FMO:
		p->codec.h264.fmo_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_FMO_MAP_TYPE:
		switch ((enum v4l2_mpeg_video_h264_fmo_map_type)(ctrl->value)) {
		case V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_INTERLEAVED_SLICES:
		case V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_SCATTERED_SLICES:
		case V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_RASTER_SCAN:
		case V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_WIPE_SCAN:
			p->codec.h264.fmo_slice_map_type = ctrl->value;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_FMO_SLICE_GROUP:
		p->codec.h264.fmo_slice_num_grp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_FMO_RUN_LENGTH:
		p->codec.h264.fmo_run_length[(ctrl->value >> 30) & 0x3]
			= ctrl->value & 0x3FFFFFFF;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_FMO_CHANGE_DIRECTION:
		p->codec.h264.fmo_sg_dir = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_FMO_CHANGE_RATE:
		p->codec.h264.fmo_sg_rate = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_ASO:
		p->codec.h264.aso_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_ASO_SLICE_ORDER:
		p->codec.h264.aso_slice_order[(ctrl->value >> 18) & 0x7]
			&= ~(0xFF << (((ctrl->value >> 16) & 0x3) << 3));
		p->codec.h264.aso_slice_order[(ctrl->value >> 18) & 0x7]
			|= (ctrl->value & 0xFF) << \
				(((ctrl->value >> 16) & 0x3) << 3);
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PREPEND_SPSPPS_TO_IDR:
		p->codec.h264.prepend_sps_pps_to_idr = ctrl->value ? 1 : 0;
		break;
	case V4L2_CID_MPEG_MFC_H264_ENABLE_LTR:
		p->codec.h264.enable_ltr = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC_H264_NUM_OF_LTR:
		p->codec.h264.num_of_ltr = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC_H264_BASE_PRIORITY:
		p->codec.h264.base_priority = ctrl->value;
		p->codec.h264.set_priority = 1;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
		switch ((enum v4l2_mpeg_video_mpeg4_profile)(ctrl->value)) {
		case V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE:
			p->codec.mpeg4.profile =
				MFC_REG_E_PROFILE_MPEG4_SIMPLE;
			break;
		case V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE:
			p->codec.mpeg4.profile =
			MFC_REG_E_PROFILE_MPEG4_ADVANCED_SIMPLE;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
		p->codec.mpeg4.level =
		__mfc_enc_mpeg4_level((enum v4l2_mpeg_video_mpeg4_level)(ctrl->value));
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP:
		p->codec.mpeg4.rc_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP:
		p->codec.mpeg4.rc_min_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP:
		p->codec.mpeg4.rc_max_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP_P:
		p->codec.mpeg4.rc_min_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP_P:
		p->codec.mpeg4.rc_max_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP_B:
		p->codec.mpeg4.rc_min_qp_b = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP_B:
		p->codec.mpeg4.rc_max_qp_b = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_QPEL:
		p->codec.mpeg4.quarter_pixel = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP:
		p->codec.mpeg4.rc_p_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_B_FRAME_QP:
		p->codec.mpeg4.rc_b_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_MPEG4_VOP_TIME_RES:
		p->rc_framerate = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_MPEG4_VOP_FRM_DELTA:
		p->codec.mpeg4.vop_frm_delta = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H263_RC_FRAME_RATE:
		p->rc_framerate = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H263_I_FRAME_QP:
		p->codec.mpeg4.rc_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H263_MIN_QP:
		p->codec.mpeg4.rc_min_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H263_MAX_QP:
		p->codec.mpeg4.rc_max_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H263_MIN_QP_P:
		p->codec.mpeg4.rc_min_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H263_MAX_QP_P:
		p->codec.mpeg4.rc_max_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_H263_P_FRAME_QP:
		p->codec.mpeg4.rc_p_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_VERSION:
		p->codec.vp8.vp8_version = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_RC_FRAME_RATE:
		p->rc_framerate = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_MIN_QP:
		p->codec.vp8.rc_min_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_MAX_QP:
		p->codec.vp8.rc_max_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_MIN_QP_P:
		p->codec.vp8.rc_min_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_MAX_QP_P:
		p->codec.vp8.rc_max_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_I_FRAME_QP:
		p->codec.vp8.rc_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_P_FRAME_QP:
		p->codec.vp8.rc_p_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_NUM_OF_PARTITIONS:
		p->codec.vp8.vp8_numberofpartitions = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_FILTER_LEVEL:
		p->codec.vp8.vp8_filterlevel = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_FILTER_SHARPNESS:
		p->codec.vp8.vp8_filtersharpness = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_GOLDEN_FRAMESEL:
		p->codec.vp8.vp8_goldenframesel = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_GF_REFRESH_PERIOD:
		p->codec.vp8.vp8_gfrefreshperiod = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_HIERARCHY_QP_ENABLE:
		p->codec.vp8.hier_qp_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_HIERARCHY_QP_LAYER0:
		p->codec.vp8.hier_qp_layer[(ctrl->value >> 16) & 0x3]
			= ctrl->value & 0xFF;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_HIERARCHY_QP_LAYER1:
		p->codec.vp8.hier_qp_layer[(ctrl->value >> 16) & 0x3]
			= ctrl->value & 0xFF;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_HIERARCHY_QP_LAYER2:
		p->codec.vp8.hier_qp_layer[(ctrl->value >> 16) & 0x3]
			= ctrl->value & 0xFF;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_HIERARCHICAL_CODING_LAYER_BIT0:
		p->codec.vp8.hier_bit_layer[0] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_HIERARCHICAL_CODING_LAYER_BIT1:
		p->codec.vp8.hier_bit_layer[1] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_HIERARCHICAL_CODING_LAYER_BIT2:
		p->codec.vp8.hier_bit_layer[2] = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_REF_NUMBER_FOR_PFRAMES:
		p->num_refs_for_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_DISABLE_INTRA_MD4X4:
		p->codec.vp8.intra_4x4mode_disable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC70_VIDEO_VP8_NUM_TEMPORAL_LAYER:
		p->codec.vp8.num_hier_layer = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_VERSION:
		p->codec.vp9.profile = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_RC_FRAME_RATE:
		p->rc_framerate = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_MIN_QP:
		p->codec.vp9.rc_min_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_MAX_QP:
		p->codec.vp9.rc_max_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_MIN_QP_P:
		p->codec.vp9.rc_min_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_MAX_QP_P:
		p->codec.vp9.rc_max_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_I_FRAME_QP:
		p->codec.vp9.rc_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_P_FRAME_QP:
		p->codec.vp9.rc_p_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_GOLDEN_FRAMESEL:
		p->codec.vp9.vp9_goldenframesel = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_GF_REFRESH_PERIOD:
		p->codec.vp9.vp9_gfrefreshperiod = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_HIERARCHY_QP_ENABLE:
		p->codec.vp9.hier_qp_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_HIERARCHICAL_CODING_LAYER_QP:
		p->codec.vp9.hier_qp_layer[(ctrl->value >> 16) & 0x3]
			= ctrl->value & 0xFF;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_HIERARCHICAL_CODING_LAYER_BIT0:
		p->codec.vp9.hier_bit_layer[0] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_HIERARCHICAL_CODING_LAYER_BIT1:
		p->codec.vp9.hier_bit_layer[1] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_HIERARCHICAL_CODING_LAYER_BIT2:
		p->codec.vp9.hier_bit_layer[2] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_REF_NUMBER_FOR_PFRAMES:
		p->num_refs_for_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_HIERARCHICAL_CODING_LAYER:
		p->codec.vp9.num_hier_layer = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_MAX_PARTITION_DEPTH:
		p->codec.vp9.max_partition_depth = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_DISABLE_INTRA_PU_SPLIT:
		p->codec.vp9.intra_pu_split_disable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_DISABLE_IVF_HEADER:
		p->ivf_header_disable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
		p->codec.vp9.profile = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_LEVEL:
		p->codec.vp9.level = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP:
		p->codec.hevc.rc_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP:
		p->codec.hevc.rc_p_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP:
		p->codec.hevc.rc_b_frame_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_RC_FRAME_RATE:
		p->rc_framerate = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
		p->codec.hevc.rc_min_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
		p->codec.hevc.rc_max_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP_P:
		p->codec.hevc.rc_min_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP_P:
		p->codec.hevc.rc_max_qp_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP_B:
		p->codec.hevc.rc_min_qp_b = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP_B:
		p->codec.hevc.rc_max_qp_b = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
		p->codec.hevc.level = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		p->codec.hevc.profile = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_ADAPTIVE_RC_DARK:
		p->codec.hevc.rc_lcu_dark = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_ADAPTIVE_RC_SMOOTH:
		p->codec.hevc.rc_lcu_smooth = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_ADAPTIVE_RC_STATIC:
		p->codec.hevc.rc_lcu_static = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_ADAPTIVE_RC_ACTIVITY:
		p->codec.hevc.rc_lcu_activity = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_TIER_FLAG:
		p->codec.hevc.tier_flag = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_MAX_PARTITION_DEPTH:
		p->codec.hevc.max_partition_depth = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_REF_NUMBER_FOR_PFRAMES:
		p->num_refs_for_p = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_REFRESH_TYPE:
		p->codec.hevc.refreshtype = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_CONST_INTRA_PRED_ENABLE:
		p->codec.hevc.const_intra_period_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_LOSSLESS_CU_ENABLE:
		p->codec.hevc.lossless_cu_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_WAVEFRONT_ENABLE:
		p->codec.hevc.wavefront_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_LF_DISABLE:
		p->codec.hevc.loopfilter_disable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_LF_SLICE_BOUNDARY:
		p->codec.hevc.loopfilter_across = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_LTR_ENABLE:
		p->codec.hevc.enable_ltr = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_QP_ENABLE:
		p->codec.hevc.hier_qp_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_TYPE:
		p->codec.hevc.hier_qp_type =
		(enum v4l2_mpeg_video_hevc_hier_coding_type)(ctrl->value);
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER:
		p->codec.hevc.num_hier_layer = ctrl->value & 0x7;
		p->codec.hevc.hier_ref_type = (ctrl->value >> 16) & 0x1;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_QP:
		p->codec.hevc.hier_qp_layer[(ctrl->value >> 16) & 0x7]
			= ctrl->value & 0xFF;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_BIT0:
		p->codec.hevc.hier_bit_layer[0] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_BIT1:
		p->codec.hevc.hier_bit_layer[1] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_BIT2:
		p->codec.hevc.hier_bit_layer[2] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_BIT3:
		p->codec.hevc.hier_bit_layer[3] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_BIT4:
		p->codec.hevc.hier_bit_layer[4] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_BIT5:
		p->codec.hevc.hier_bit_layer[5] = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_BIT6:
		p->codec.hevc.hier_bit_layer[6] = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_GENERAL_PB_ENABLE:
		p->codec.hevc.general_pb_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_TEMPORAL_ID_ENABLE:
		p->codec.hevc.temporal_id_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_STRONG_SMOTHING_FLAG:
		p->codec.hevc.strong_intra_smooth = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_DISABLE_INTRA_PU_SPLIT:
		p->codec.hevc.intra_pu_split_disable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_DISABLE_TMV_PREDICTION:
		p->codec.hevc.tmv_prediction_disable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_MAX_NUM_MERGE_MV_MINUS1:
		p->codec.hevc.max_num_merge_mv = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_WITHOUT_STARTCODE_ENABLE:
		p->codec.hevc.encoding_nostartcode_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_REFRESH_PERIOD:
		p->codec.hevc.refreshperiod = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_LF_BETA_OFFSET_DIV2:
		p->codec.hevc.lf_beta_offset_div2 = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_LF_TC_OFFSET_DIV2:
		p->codec.hevc.lf_tc_offset_div2 = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_SIZE_OF_LENGTH_FIELD:
		p->codec.hevc.size_of_length_field = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_USER_REF:
		p->codec.hevc.user_ref = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_STORE_REF:
		p->codec.hevc.store_ref = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_ROI_ENABLE:
		p->roi_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC_H264_VUI_RESTRICTION_ENABLE:
		p->codec.h264.vui_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_PREPEND_SPSPPS_TO_IDR:
		p->codec.hevc.prepend_sps_pps_to_idr = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC_CONFIG_QP_ENABLE:
		p->dynamic_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC_CONFIG_QP:
		p->config_qp = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_RC_PVC_ENABLE:
		/* It is valid for H.264, HEVC, VP8, VP9 */
		p->rc_pvc = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_TEMPORAL_SHORTTERM_MAX_LAYER:
		p->num_hier_max_layer = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC90_VIDEO_HEVC_SIGN_DATA_HIDING:
		break;
	case V4L2_CID_MPEG_VIDEO_WEIGHTED_ENABLE:
		p->weighted_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_BPG_THUMBNAIL_SIZE:
		/* It should be included when NAL_START */
		p->codec.bpg.thumb_size = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_BPG_EXIF_SIZE:
		/* It should be included when NAL_START */
		p->codec.bpg.exif_size = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_RATIO_OF_INTRA:
		p->ratio_intra = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_FULL_RANGE_FLAG:
		p->check_color_range = 1;
		p->color_range = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_COLOUR_PRIMARIES:
		p->colour_primaries = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_TRANSFER_CHARACTERISTICS:
		p->transfer_characteristics = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MATRIX_COEFFICIENTS:
		p->matrix_coefficients = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_HIERARCHICAL_BITRATE_CTRL:
		p->hier_bitrate_ctrl = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_STATIC_INFO_ENABLE:
		p->static_info_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SEI_MAX_PIC_AVERAGE_LIGHT:
		p->max_pic_average_light = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SEI_MAX_CONTENT_LIGHT:
		p->max_content_light = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SEI_MAX_DISPLAY_LUMINANCE:
		p->max_display_luminance = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SEI_MIN_DISPLAY_LUMINANCE:
		p->min_display_luminance = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SEI_WHITE_POINT:
		p->white_point = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SEI_DISPLAY_PRIMARIES_0:
		p->display_primaries_0 = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SEI_DISPLAY_PRIMARIES_1:
		p->display_primaries_1 = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_SEI_DISPLAY_PRIMARIES_2:
		p->display_primaries_2 = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_DROP_CONTROL:
		p->drop_control = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_CHROMA_QP_OFFSET_CB:
		p->chroma_qp_offset_cb = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_CHROMA_QP_OFFSET_CR:
		p->chroma_qp_offset_cr = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC_HDR_USER_SHARED_HANDLE:
		if (enc->sh_handle_hdr.fd == -1) {
			enc->sh_handle_hdr.fd = ctrl->value;
			if (mfc_mem_get_user_shared_handle(ctx, &enc->sh_handle_hdr, "HDR10+"))
				enc->sh_handle_hdr.fd = -1;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_GDC_VOTF:
		ctx->gdc_votf = ctrl->value;
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
	case V4L2_CID_MPEG_VIDEO_MV_SEARCH_MODE:
		p->mv_search_mode = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_HOR_POSITION_L0:
		p->mv_hor_pos_l0 = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_HOR_POSITION_L1:
		p->mv_hor_pos_l1 = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_VER_POSITION_L0:
		p->mv_ver_pos_l0 = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_VER_POSITION_L1:
		p->mv_ver_pos_l1 = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_CTRL:
		p->gop_ctrl = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MIN_QUALITY:
		p->min_quality_mode = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_WP_TWO_PASS_ENABLE:
		p->wp_two_pass_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_ADAPTIVE_GOP_ENABLE:
		p->adaptive_gop_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_HOR_RANGE:
		p->mv_hor_range = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_VER_RANGE:
		p->mv_ver_range = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC_H264_SUB_GOP_ENABLE:
		p->codec.h264.sub_gop_enable = ctrl->value;
		break;
	case V4L2_CID_MPEG_MFC_HEVC_SUB_GOP_ENABLE:
		p->codec.hevc.sub_gop_enable = ctrl->value;
		fallthrough;
	case V4L2_CID_MPEG_MFC_QPE_TWO_PASS_ENABLE:
		p->qpe_two_pass_enable = ctrl->value;
		enc->nal_q_disable_for_qpe_two_pass = ctrl->value;
		mfc_debug(2, "[QPE TWO PASS] qpe two pass %s\n",
				p->qpe_two_pass_enable ? "enable" : "disable");
		break;
	/* These are stored in specific variables */
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_CH:
	case V4L2_CID_MPEG_VIDEO_VP9_HIERARCHICAL_CODING_LAYER_CH:
	case V4L2_CID_MPEG_VIDEO_VP8_HIERARCHICAL_CODING_LAYER_CH:
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_CH:
	/* These require control per buffer */
	case V4L2_CID_MPEG_VIDEO_YSUM:
	case V4L2_CID_MPEG_VIDEO_ROI_CONTROL:
	case V4L2_CID_MPEG_MFC_H264_USE_LTR:
	case V4L2_CID_MPEG_MFC_H264_MARK_LTR:
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG:
	case V4L2_CID_MPEG_MFC_H264_SUB_GOP_TYPE:
	case V4L2_CID_MPEG_MFC_HEVC_SUB_GOP_TYPE:
	case V4L2_CID_MPEG_VIDEO_SRC_BUF_FLAG:
	case V4L2_CID_MPEG_VIDEO_DST_BUF_FLAG:
		break;
	default:
		mfc_ctx_err("Invalid control: 0x%08x\n", ctrl->id);
		ret = -EINVAL;
	}

	return ret;
}

static int __mfc_enc_set_ctrl_val(struct mfc_ctx *ctx, struct v4l2_control *ctrl)
{
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_enc_params *p = &enc->params;
	struct mfc_ctx_ctrl *ctx_ctrl;
	int ret = 0;
	int found = 0;

	/* update parameter value */
	ret = __mfc_enc_set_param(ctx, ctrl);
	if (ret)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_CACHEABLE:
	case V4L2_CID_MPEG_VIDEO_QOS_RATIO:
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_H263_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_VP8_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_VP9_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_H263_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_VP8_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_VP9_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP_P:
	case V4L2_CID_MPEG_VIDEO_H263_MAX_QP_P:
	case V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP_P:
	case V4L2_CID_MPEG_VIDEO_VP8_MAX_QP_P:
	case V4L2_CID_MPEG_VIDEO_VP9_MAX_QP_P:
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP_P:
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP_P:
	case V4L2_CID_MPEG_VIDEO_H263_MIN_QP_P:
	case V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP_P:
	case V4L2_CID_MPEG_VIDEO_VP8_MIN_QP_P:
	case V4L2_CID_MPEG_VIDEO_VP9_MIN_QP_P:
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP_P:
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP_B:
	case V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP_B:
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP_B:
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP_B:
	case V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP_B:
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP_B:
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG:
	case V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE:
	case V4L2_CID_MPEG_MFC51_VIDEO_I_PERIOD_CH:
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_RATE_CH:
	case V4L2_CID_MPEG_MFC51_VIDEO_BIT_RATE_CH:
	case V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_CH:
	case V4L2_CID_MPEG_VIDEO_VP8_HIERARCHICAL_CODING_LAYER_CH:
	case V4L2_CID_MPEG_VIDEO_VP9_HIERARCHICAL_CODING_LAYER_CH:
	case V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_CH:
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
	case V4L2_CID_MPEG_MFC_H264_MARK_LTR:
	case V4L2_CID_MPEG_MFC_H264_USE_LTR:
	case V4L2_CID_MPEG_MFC_H264_BASE_PRIORITY:
	case V4L2_CID_MPEG_MFC_CONFIG_QP:
	case V4L2_CID_MPEG_VIDEO_ROI_CONTROL:
	case V4L2_CID_MPEG_VIDEO_YSUM:
	case V4L2_CID_MPEG_VIDEO_RATIO_OF_INTRA:
	case V4L2_CID_MPEG_VIDEO_DROP_CONTROL:
	case V4L2_CID_MPEG_VIDEO_MV_HOR_POSITION_L0:
	case V4L2_CID_MPEG_VIDEO_MV_HOR_POSITION_L1:
	case V4L2_CID_MPEG_VIDEO_MV_VER_POSITION_L0:
	case V4L2_CID_MPEG_VIDEO_MV_VER_POSITION_L1:
	case V4L2_CID_MPEG_MFC_H264_SUB_GOP_TYPE:
	case V4L2_CID_MPEG_MFC_HEVC_SUB_GOP_TYPE:
	case V4L2_CID_MPEG_VIDEO_SRC_BUF_FLAG:
	case V4L2_CID_MPEG_VIDEO_DST_BUF_FLAG:
		list_for_each_entry(ctx_ctrl, &ctx->ctrls, list) {
			if (!(ctx_ctrl->type & MFC_CTRL_TYPE_SET))
				continue;

			if (ctx_ctrl->id == ctrl->id) {
				ctx_ctrl->set.has_new = 1;
				ctx_ctrl->set.val = ctrl->value;
				if ((ctx_ctrl->id == \
					V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_CH) ||
					(ctx_ctrl->id == \
					V4L2_CID_MPEG_VIDEO_VP8_HIERARCHICAL_CODING_LAYER_CH) ||
					(ctx_ctrl->id == \
					V4L2_CID_MPEG_VIDEO_VP9_HIERARCHICAL_CODING_LAYER_CH) ||
					(ctx_ctrl->id == \
					V4L2_CID_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_LAYER_CH)) {
					if (enc->sh_handle_svc.fd == -1) {
						enc->sh_handle_svc.fd = ctrl->value;
						if (mfc_mem_get_user_shared_handle(ctx,
									&enc->sh_handle_svc, "SVC"))
							return -EINVAL;
					}
				}
				if (ctx_ctrl->id == V4L2_CID_MPEG_MFC51_VIDEO_I_PERIOD_CH &&
						p->i_frm_ctrl_mode) {
					if (!p->gop_ctrl)
						ctx_ctrl->set.val = ctx_ctrl->set.val *
							(p->num_b_frame + 1);
					if (ctx_ctrl->set.val >= 0x3FFFFFFF) {
						mfc_ctx_info("I frame interval is bigger than max: %d\n",
								ctx_ctrl->set.val);
						ctx_ctrl->set.val = 0x3FFFFFFF;
					}
				}
				if (ctx_ctrl->id == V4L2_CID_MPEG_VIDEO_H264_LEVEL)
					ctx_ctrl->set.val = __mfc_enc_h264_level((enum v4l2_mpeg_video_h264_level)(ctrl->value));
				if (ctx_ctrl->id == V4L2_CID_MPEG_VIDEO_H264_PROFILE)
					ctx_ctrl->set.val = __mfc_enc_h264_profile(ctx, (enum v4l2_mpeg_video_h264_profile)(ctrl->value));
				if (ctx_ctrl->id == V4L2_CID_MPEG_VIDEO_ROI_CONTROL) {
					ret = __mfc_enc_get_roi(ctx, ctrl->value);
					if (ret)
						return ret;
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
	default:
		break;
	}

	return ret;
}

static int mfc_enc_s_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	int ret = 0;

	mfc_debug_enter();

	ret = __mfc_enc_check_ctrl_val(ctx, ctrl);
	if (ret != 0)
		return ret;

	ret = __mfc_enc_set_ctrl_val(ctx, ctrl);

	mfc_debug_leave();

	return ret;
}

static int mfc_enc_g_ext_ctrls(struct file *file, void *priv,
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

		ret = __mfc_enc_get_ctrl_val(ctx, &ctrl);
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

static int mfc_enc_s_ext_ctrls(struct file *file, void *priv,
				struct v4l2_ext_controls *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct v4l2_ext_control *ext_ctrl;
	struct v4l2_control ctrl;
	int i;
	int ret = 0;

	mfc_debug_enter();

	if (f->which != V4L2_CTRL_CLASS_CODEC)
		return -EINVAL;

	for (i = 0; i < f->count; i++) {
		ext_ctrl = (f->controls + i);

		ctrl.id = ext_ctrl->id;
		ctrl.value = ext_ctrl->value;

		ret = __mfc_enc_check_ctrl_val(ctx, &ctrl);
		if (ret != 0) {
			f->error_idx = i;
			break;
		}

		ret = __mfc_enc_set_param(ctx, &ctrl);
		if (ret != 0) {
			f->error_idx = i;
			break;
		}
	}

	mfc_debug_leave();

	return ret;
}

static int mfc_enc_try_ext_ctrls(struct file *file, void *priv,
				struct v4l2_ext_controls *f)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct v4l2_ext_control *ext_ctrl;
	struct v4l2_control ctrl;
	int i;
	int ret = 0;

	mfc_debug_enter();

	if (f->which != V4L2_CTRL_CLASS_CODEC)
		return -EINVAL;

	for (i = 0; i < f->count; i++) {
		ext_ctrl = (f->controls + i);

		ctrl.id = ext_ctrl->id;
		ctrl.value = ext_ctrl->value;

		ret = __mfc_enc_check_ctrl_val(ctx, &ctrl);
		if (ret != 0) {
			f->error_idx = i;
			break;
		}
	}

	mfc_debug_leave();

	return ret;
}

static const struct v4l2_ioctl_ops mfc_enc_ioctl_ops = {
	.vidioc_querycap		= mfc_enc_querycap,
	.vidioc_enum_fmt_vid_cap	= mfc_enc_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out	= mfc_enc_enum_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= mfc_enc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= mfc_enc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= mfc_enc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane	= mfc_enc_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= mfc_enc_s_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_out_mplane	= mfc_enc_s_fmt_vid_out_mplane,
	.vidioc_g_selection		= mfc_enc_g_selection,
	.vidioc_s_selection		= mfc_enc_s_selection,
	.vidioc_reqbufs			= mfc_enc_reqbufs,
	.vidioc_querybuf		= mfc_enc_querybuf,
	.vidioc_qbuf			= mfc_enc_qbuf,
	.vidioc_dqbuf			= mfc_enc_dqbuf,
	.vidioc_streamon		= mfc_enc_streamon,
	.vidioc_streamoff		= mfc_enc_streamoff,
	.vidioc_queryctrl		= mfc_enc_queryctrl,
	.vidioc_g_ctrl			= mfc_enc_g_ctrl,
	.vidioc_s_ctrl			= mfc_enc_s_ctrl,
	.vidioc_g_ext_ctrls		= mfc_enc_g_ext_ctrls,
	.vidioc_s_ext_ctrls		= mfc_enc_s_ext_ctrls,
	.vidioc_try_ext_ctrls		= mfc_enc_try_ext_ctrls,
};

const struct v4l2_ioctl_ops *mfc_get_enc_v4l2_ioctl_ops(void)
{
	return &mfc_enc_ioctl_ops;
}
