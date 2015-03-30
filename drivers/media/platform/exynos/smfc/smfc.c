/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * The main source file of Samsung Exynos SMFC Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/exynos_iovmm.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-sg.h>

#include "smfc.h"

/* SMFC SPECIFIC DEVICE CAPABILITIES */
/* set if H/W supports for decompression */
#define V4L2_CAP_EXYNOS_JPEG_DECOMPRESSION		0x0100
/* set if H/W can compress dual images */
#define V4L2_CAP_EXYNOS_JPEG_B2B_COMPRESSION		0x0200
/* set if H/W supports for Hardware Flow Control */
#define V4L2_CAP_EXYNOS_JPEG_HWFC			0x0400
/* set if H/W supports for HWFC on internal buffers */
#define V4L2_CAP_EXYNOS_JPEG_HWFC_EMBEDDED		0x0800
/* set if H/W has a register to configure stream buffer size */
#define V4L2_CAP_EXYNOS_JPEG_MAX_STREAMSIZE  		0x1000
/* set if H/W does not have 128-bit alignment constraint for stream base */
#define V4L2_CAP_EXYNOS_JPEG_NO_STREAMBASE_ALIGN	0x2000
/* set if H/W does not have 128-bit alignment constraint for image base */
#define V4L2_CAP_EXYNOS_JPEG_NO_IMAGEBASE_ALIGN		0x4000

#define SMFC_DEFAULT_OUTPUT_FORMAT	(&smfc_image_formats[1])
#define SMFC_DEFAULT_CAPTURE_FORMAT	(&smfc_image_formats[0])

const struct smfc_image_format smfc_image_formats[] = {
	{
		/* JPEG should be the first format */
		.description	= "Baseline JPEG(Sequential DCT)",
		.v4l2_pixfmt	= V4L2_PIX_FMT_JPEG,
		.regcfg		= 0, /* Chagneable accroding to chroma factor */
		.bpp_buf	= {0}, /* undeterministic */
		.bpp_pix	= {0}, /* undeterministic */
		.num_planes	= 1,
		.num_buffers	= 1,
		.chroma_hfactor	= 1, /* dummy chroma subsampling factor */
		.chroma_vfactor	= 1, /* These will not affect to H/W config. */
	}, {
		/* YUYV is the default format */
		.description	= "YUV4:2:2 1-Plane 16BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_YUYV,
		.regcfg		= SMFC_IMGFMT_YUYV,
		.bpp_buf	= {16},
		.bpp_pix	= {16},
		.num_planes	= 1,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 1,
	}, {
		.description	= "YUV4:2:2 1-Plane 16BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_YVYU,
		.regcfg		= SMFC_IMGFMT_YVYU,
		.bpp_buf	= {16},
		.bpp_pix	= {16},
		.num_planes	= 1,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 1,
	}, {
		.description	= "YUV4:2:2 2-Plane 16BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_NV16,
		.regcfg		= SMFC_IMGFMT_NV16,
		.bpp_buf	= {16},
		.bpp_pix	= {8, 8, 0},
		.num_planes	= 2,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 1,
	}, {
		.description	= "YUV4:2:2 2-Plane 16BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_NV61,
		.regcfg		= SMFC_IMGFMT_NV61,
		.bpp_buf	= {16},
		.bpp_pix	= {8, 8, 0},
		.num_planes	= 2,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 1,
	}, {
		.description	= "YUV4:2:2 3-Plane 16BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_YUV422P,
		.regcfg		= SMFC_IMGFMT_YUV422,
		.bpp_buf	= {16},
		.bpp_pix	= {8, 2, 2},
		.num_planes	= 3,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 1,
	}, {
		.description	= "YUV4:2:0 2-Plane(Cb/Cr) 12BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_NV12,
		.regcfg		= SMFC_IMGFMT_NV12,
		.bpp_buf	= {12},
		.bpp_pix	= {8, 4, 0},
		.num_planes	= 2,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 2,
	}, {
		.description	= "YUV4:2:0 2-Plane(Cr/Cb) 12BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_NV21,
		.regcfg		= SMFC_IMGFMT_NV21,
		.bpp_buf	= {12},
		.bpp_pix	= {8, 4, 0},
		.num_planes	= 2,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 2,
	}, {
		.description	= "YUV4:2:0 3-Plane 12BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_YUV420,
		.regcfg		= SMFC_IMGFMT_YUV420P,
		.bpp_buf	= {12},
		.bpp_pix	= {8, 2, 2},
		.num_planes	= 3,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 2,
	}, {
		.description	= "YVU4:2:0 3-Plane 12BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_YVU420,
		.regcfg		= SMFC_IMGFMT_YVU420P,
		.bpp_buf	= {12},
		.bpp_pix	= {8, 2, 2},
		.num_planes	= 3,
		.num_buffers	= 1,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 2,
	}, {
		.description	= "YUV4:4:4 3-Plane 16BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_YUV444,
		.regcfg		= SMFC_IMGFMT_YUV444,
		.bpp_buf	= {24},
		.bpp_pix	= {8, 8, 8},
		.num_planes	= 3,
		.num_buffers	= 1,
		.chroma_hfactor	= 1,
		.chroma_vfactor	= 1,
	}, {
		.description	= "RGB565 LE 16BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_RGB565,
		.regcfg		= SMFC_IMGFMT_RGB565,
		.bpp_buf	= {16},
		.bpp_pix	= {16},
		.num_planes	= 1,
		.num_buffers	= 1,
	}, {
		.description	= "RGB565 BE 16BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_RGB565X,
		.regcfg		= SMFC_IMGFMT_BGR565,
		.bpp_buf	= {16},
		.bpp_pix	= {16},
		.num_planes	= 1,
		.num_buffers	= 1,
	}, {
		.description	= "RGB888 LE 24BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_RGB24,
		.regcfg		= SMFC_IMGFMT_RGB24,
		.bpp_buf	= {24},
		.bpp_pix	= {24},
		.num_planes	= 1,
		.num_buffers	= 1,
	}, {
		.description	= "RGB888 BE 24BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_BGR24,
		.regcfg		= SMFC_IMGFMT_BGR24,
		.bpp_buf	= {24},
		.bpp_pix	= {24},
		.num_planes	= 1,
		.num_buffers	= 1,
	}, {
		.description	= "ABGR8888 LE 32BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_RGB32,
		.regcfg		= SMFC_IMGFMT_RGBA32,
		.bpp_pix	= {32},
		.num_planes	= 1,
		.num_buffers	= 1,
	}, {
		.description	= "ARGB8888 BE 32BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_BGR32,
		.regcfg		= SMFC_IMGFMT_BGRA32,
		.bpp_pix	= {32},
		.num_planes	= 1,
		.num_buffers	= 1,
	},
	/* multi-planar formats must be at the last becuse of VIDOC_ENUM_FMT */
	{
		.description	= "YUV4:2:0 2-MPlane(Cb/Cr) 8+4BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_NV12M,
		.regcfg		= SMFC_IMGFMT_NV12,
		.bpp_buf	= {8, 4},
		.bpp_pix	= {8, 4},
		.num_planes	= 2,
		.num_buffers	= 2,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 2,
	}, {
		.description	= "YUV4:2:0 2-MPlane(Cr/Cb) 8+4BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_NV21M,
		.regcfg		= SMFC_IMGFMT_NV21,
		.bpp_buf	= {8, 4},
		.bpp_pix	= {8, 4},
		.num_planes	= 2,
		.num_buffers	= 2,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 2,
	}, {
		.description	= "YUV4:2:0 3-MPlane 8+2+2BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_YUV420M,
		.regcfg		= SMFC_IMGFMT_YUV420P,
		.bpp_buf	= {8, 2, 2},
		.bpp_pix	= {8, 2, 2},
		.num_planes	= 3,
		.num_buffers	= 3,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 2,
	}, {
		.description	= "YVU4:2:0 3-MPlane 8+2+2BPP",
		.v4l2_pixfmt	= V4L2_PIX_FMT_YVU420M,
		.regcfg		= SMFC_IMGFMT_YVU420P,
		.bpp_buf	= {8, 2, 2},
		.bpp_pix	= {8, 2, 2},
		.num_planes	= 3,
		.num_buffers	= 3,
		.chroma_hfactor	= 2,
		.chroma_vfactor	= 2,
	}
};

static const struct smfc_image_format *smfc_find_format(
				struct smfc_dev *smfc, __u32 v4l2_pixfmt)
{
	size_t i;
	for (i = 0; i < ARRAY_SIZE(smfc_image_formats); i++)
		if (smfc_image_formats[i].v4l2_pixfmt == v4l2_pixfmt)
			return &smfc_image_formats[i];

	dev_warn(smfc->dev, "Pixel format '%08X' not found, YUYV is forced.\n",
		v4l2_pixfmt);

	return V4L2_TYPE_IS_OUTPUT(v4l2_pixfmt) ? SMFC_DEFAULT_OUTPUT_FORMAT
						: SMFC_DEFAULT_CAPTURE_FORMAT;
}

static const char *buf_type_name(__u32 type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? "capture" : "output";
}

static irqreturn_t exynos_smfc_irq_handler(int irq, void *priv)
{
	struct smfc_dev *smfc = priv;
	struct smfc_ctx *ctx = v4l2_m2m_get_curr_priv(smfc->m2mdev);
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	struct vb2_v4l2_buffer *vb_capture;

	vb_capture = v4l2_m2m_dst_buf_remove(ctx->m2mctx);

	if (smfc_hwstatus_okay(smfc)) {
		vb2_set_plane_payload(&vb_capture->vb2_buf, 0,
				      smfc_get_streamsize(smfc));
	} else {
		state = VB2_BUF_STATE_ERROR;
		smfc_hwconfigure_reset(smfc);
	}

	if (!IS_ERR(smfc->clk_gate)) {
		clk_disable(smfc->clk_gate);
		if (!IS_ERR(smfc->clk_gate2))
			clk_disable(smfc->clk_gate2);
	}

	pm_runtime_put(smfc->dev);

	v4l2_m2m_buf_done(v4l2_m2m_src_buf_remove(ctx->m2mctx), state);
	v4l2_m2m_buf_done(vb_capture, state);
	v4l2_m2m_job_finish(smfc->m2mdev, ctx->m2mctx);

	return IRQ_HANDLED;
}

static int smfc_vb2_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
				unsigned int *num_planes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vq);
	unsigned int i;

	if (smfc_is_compressed_type(ctx, vq->type)) {
		/*
		 * SMFC is able to stop compression if the target buffer is not
		 * enough. Therefore, it is not required to configure larger
		 * buffer for compression.
		 */
		sizes[0] = PAGE_SIZE;
		*num_planes = 1;
		alloc_devs[i] = ctx->smfc->dev;
	} else {
		unsigned int i;
		*num_planes = ctx->img_fmt->num_buffers;
		for (i = 0; i < *num_planes; i++) {
			sizes[i] = ctx->width * ctx->height;
			sizes[i] = (sizes[i] * ctx->img_fmt->bpp_buf[i]) / 8;
			alloc_devs[i] = ctx->smfc->dev;
		}
	}

	return 0;
}

static int smfc_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	unsigned int i;

	if (!smfc_is_compressed_type(ctx, vb->vb2_queue->type)) {
		unsigned long payload = ctx->width * ctx->height;
		for (i = 0; i < ctx->img_fmt->num_buffers; i++) {
			unsigned long planebytes;
			planebytes = (payload * ctx->img_fmt->bpp_buf[i]) / 8;
			if (vb2_get_plane_payload(vb, i) < planebytes) {
				dev_err(ctx->smfc->dev,
				"Too small bytes_used[%lu]=%u (req.:%lu)\n",
				vb2_get_plane_payload(vb, i), i, planebytes);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static void smfc_vb2_buf_finish(struct vb2_buffer *vb)
{
}

static void smfc_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->m2mctx, to_vb2_v4l2_buffer(vb));
}

static void smfc_vb2_lock(struct vb2_queue *vq)
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vq);
	mutex_lock(&ctx->smfc->video_device_mutex);
}

static void smfc_vb2_unlock(struct vb2_queue *vq)
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vq);
	mutex_unlock(&ctx->smfc->video_device_mutex);
}

static struct vb2_ops smfc_vb2_ops = {
	.queue_setup	= smfc_vb2_queue_setup,
	.buf_prepare	= smfc_vb2_buf_prepare,
	.buf_finish	= smfc_vb2_buf_finish,
	.buf_queue	= smfc_vb2_buf_queue,
	.wait_finish	= smfc_vb2_lock,
	.wait_prepare	= smfc_vb2_unlock,
};

static int smfc_queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct smfc_ctx *ctx = priv;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->ops = &smfc_vb2_ops;
	src_vq->mem_ops = &vb2_dma_sg_memops;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->ops = &smfc_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_sg_memops;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	return vb2_queue_init(dst_vq);
}

static int exynos_smfc_open(struct file *filp)
{
	struct smfc_dev *smfc = video_drvdata(filp);
	struct smfc_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(smfc->dev, "Failed to allocate smfc_ctx");
		return -ENOMEM;
	}

	ctx->m2mctx = v4l2_m2m_ctx_init(smfc->m2mdev, ctx, smfc_queue_init);
	if (IS_ERR(ctx->m2mctx)) {
		ret = PTR_ERR(ctx->m2mctx);
		dev_err(smfc->dev, "Failed(%d) to init m2m_ctx\n", ret);
		goto err_m2m_ctx_init;
	}


	v4l2_fh_init(&ctx->v4l2_fh, smfc->videodev);
	v4l2_fh_add(&ctx->v4l2_fh);

	filp->private_data = &ctx->v4l2_fh;

	if (!IS_ERR(smfc->clk_gate)) {
		ret = clk_prepare(smfc->clk_gate);
		if (!ret && !IS_ERR(smfc->clk_gate2))
			ret = clk_prepare(smfc->clk_gate2);
		if (ret) {
			clk_unprepare(smfc->clk_gate);
			dev_err(smfc->dev,
				"Failed(%d) to prepare gate clocks\n", ret);
			goto err_clk;
		}
	}

	/*
	 * default mode: compression
	 * default image format: YUYV
	 * default size: 16x8
	 * default chroma subsampling for JPEG: YUV422
	 * default quality factor for compression: 96
	 */
	ctx->img_fmt = SMFC_DEFAULT_OUTPUT_FORMAT;
	ctx->width = SMFC_MIN_WIDTH << ctx->img_fmt->chroma_hfactor;
	ctx->height = SMFC_MIN_HEIGHT << ctx->img_fmt->chroma_vfactor;
	ctx->chroma_hfactor = ctx->img_fmt->chroma_hfactor;
	ctx->chroma_vfactor = ctx->img_fmt->chroma_vfactor;
	ctx->flags |= SMFC_CTX_COMPRESS;
	ctx->quality_factor = 96;

	ctx->smfc = smfc;

	return 0;
err_clk:
	v4l2_fh_del(&ctx->v4l2_fh);
	v4l2_fh_exit(&ctx->v4l2_fh);
	v4l2_m2m_ctx_release(ctx->m2mctx);
err_m2m_ctx_init:
	kfree(ctx);
	return ret;
}

static int exynos_smfc_release(struct file *filp)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(filp->private_data);

	v4l2_fh_del(&ctx->v4l2_fh);
	v4l2_m2m_ctx_release(ctx->m2mctx);

	if (!IS_ERR(ctx->smfc->clk_gate)) {
		clk_unprepare(ctx->smfc->clk_gate);
		if (!IS_ERR(ctx->smfc->clk_gate2))
			clk_unprepare(ctx->smfc->clk_gate2);
	}

	kfree(ctx);

	return 0;
}

static unsigned int exynos_smfc_poll(struct file *filp,
				     struct poll_table_struct *wait)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(filp->private_data);
	return v4l2_m2m_poll(filp, ctx->m2mctx, wait);
}

static int exynos_smfc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(filp->private_data);
	return v4l2_m2m_mmap(filp, ctx->m2mctx, vma);
}

static const struct v4l2_file_operations smfc_v4l2_fops = {
	.owner		= THIS_MODULE,
	.open		= exynos_smfc_open,
	.release	= exynos_smfc_release,
	.poll		= exynos_smfc_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= exynos_smfc_mmap,
};

static int smfc_v4l2_querycap(struct file *filp, void *fh,
			     struct v4l2_capability *cap)
{
	struct smfc_dev *smfc = v4l2_fh_to_smfc_ctx(fh)->smfc;

	strncpy(cap->driver, MODULE_NAME, sizeof(cap->driver));
	strncpy(cap->bus_info, dev_name(smfc->dev), sizeof(cap->bus_info));
	scnprintf(cap->card, sizeof(cap->card), "Still MFC %02x.%02x.%04x",
			(smfc->hwver >> 24) & 0xFF, (smfc->hwver >> 16) & 0xFF,
			smfc->hwver & 0xFFFF);

	cap->driver[sizeof(cap->driver) - 1] = '\0';
	cap->card[sizeof(cap->card) - 1] = '\0';
	cap->bus_info[sizeof(cap->bus_info) - 1] = '\0';

	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE
				| V4L2_CAP_VIDEO_M2M;
	cap->capabilities |= V4L2_CAP_DEVICE_CAPS;

	cap->device_caps = V4L2_CAP_EXYNOS_JPEG_B2B_COMPRESSION;
	cap->device_caps |= V4L2_CAP_EXYNOS_JPEG_HWFC;
	cap->device_caps |= V4L2_CAP_EXYNOS_JPEG_MAX_STREAMSIZE;
	cap->device_caps |= V4L2_CAP_EXYNOS_JPEG_NO_STREAMBASE_ALIGN;
	cap->device_caps |= V4L2_CAP_EXYNOS_JPEG_NO_IMAGEBASE_ALIGN;

	return 0;
}

static int smfc_v4l2_enum_fmt_mplane(struct file *filp, void *fh,
				     struct v4l2_fmtdesc *f)
{
	const struct smfc_image_format *fmt;

	if (f->index >= ARRAY_SIZE(smfc_image_formats))
		return -EINVAL;

	fmt = &smfc_image_formats[f->index];
	strncpy(f->description, fmt->description, sizeof(f->description));
	f->description[sizeof(f->description) - 1] = '\0';
	f->pixelformat = fmt->v4l2_pixfmt;
	if (fmt->bpp_buf[0] == 0)
		f->flags = V4L2_FMT_FLAG_COMPRESSED;

	return 0;
}

static int smfc_v4l2_enum_fmt(struct file *filp, void *fh,
			      struct v4l2_fmtdesc *f)
{
	int ret = smfc_v4l2_enum_fmt_mplane(filp, fh, f);
	if (ret < 0)
		return ret;

	return smfc_image_formats[f->index].num_buffers > 1 ? -EINVAL : 0;
}

static int smfc_v4l2_g_fmt_mplane(struct file *filp, void *fh,
				  struct v4l2_format *f)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(fh);
	f->fmt.pix_mp.width = ctx->width;
	f->fmt.pix_mp.height = ctx->height;

	if (!smfc_is_compressed_type(ctx, f->type)) {
		int i;
		/* uncompressed image */
		f->fmt.pix_mp.pixelformat = ctx->img_fmt->v4l2_pixfmt;
		f->fmt.pix_mp.num_planes = ctx->img_fmt->num_buffers;
		for (i = 0; i < ctx->img_fmt->num_buffers; i++) {
			f->fmt.pix_mp.plane_fmt[i].bytesperline =
				(f->fmt.pix_mp.width *
					 ctx->img_fmt->bpp_buf[i]) / 8;
			f->fmt.pix_mp.plane_fmt[i].sizeimage =
					f->fmt.pix_mp.plane_fmt[i].bytesperline;
			f->fmt.pix_mp.plane_fmt[i].sizeimage *=
							f->fmt.pix_mp.height;
		}
	} else {
		f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_JPEG;
		f->fmt.pix_mp.num_planes = 1;
		f->fmt.pix_mp.plane_fmt[0].bytesperline = 0;
		f->fmt.pix_mp.plane_fmt[0].sizeimage = 0;
	}

	f->fmt.pix_mp.field = 0;
	f->fmt.pix_mp.colorspace = 0;

	return 0;
}

static int smfc_v4l2_g_fmt(struct file *filp, void *fh, struct v4l2_format *f)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(fh);

	f->fmt.pix.width = ctx->width;
	f->fmt.pix.height = ctx->height;

	if (!smfc_is_compressed_type(ctx, f->type)) {
		if (ctx->img_fmt->num_buffers > 1) {
			dev_err(ctx->smfc->dev,
				"Current format is a multi-planar format\n");
			return -EINVAL;
		}

		/* uncompressed image */
		f->fmt.pix.pixelformat = ctx->img_fmt->v4l2_pixfmt;
		f->fmt.pix.bytesperline =
			(f->fmt.pix.width * ctx->img_fmt->bpp_buf[0]) / 8;
		f->fmt.pix.sizeimage = f->fmt.pix.bytesperline;
		f->fmt.pix.sizeimage *= f->fmt.pix.height;
	} else {
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
		f->fmt.pix.bytesperline = 0;
		f->fmt.pix.sizeimage = 0;
	}

	f->fmt.pix.field = 0;
	f->fmt.pix.colorspace = 0;

	return 0;
}

static bool smfc_check_image_size(struct device *dev, __u32 type,
				  const struct smfc_image_format *smfc_fmt,
				  __u32 width, __u32 height)
{
	__u32 min_width = SMFC_MIN_WIDTH << smfc_fmt->chroma_hfactor;
	__u32 min_height = SMFC_MIN_WIDTH << smfc_fmt->chroma_vfactor;

	if ((width < min_width) || (height < min_height)) {
		dev_warn(dev, "Too small image size(%ux%u) for '%s'\n",
				width, height, buf_type_name(type));
		return false;
	}

	if ((width > SMFC_MAX_WIDTH) || (height > SMFC_MAX_HEIGHT)) {
		dev_warn(dev, "Too large image size(%ux%u) for '%s'\n",
				width, height, buf_type_name(type));
		return false;
	}

	if (((width % smfc_fmt->chroma_hfactor) != 0) ||
		((height % smfc_fmt->chroma_vfactor) != 0)) {
		dev_err(dev, "Invalid size %ux%u for format '%s'\n",
			width, height, smfc_fmt->description);
		return false;
	}

	return true;
}

static bool smfc_v4l2_init_fmt_mplane(const struct smfc_ctx *ctx,
			const struct smfc_image_format *smfc_fmt,
			__u32 type, struct v4l2_pix_format_mplane *pix_mp)
{
	unsigned int i;

	if (!smfc_check_image_size(ctx->smfc->dev, type,
				smfc_fmt, pix_mp->width, pix_mp->height))
		return false;
	/* informs the user that the format might be changed to the default */
	pix_mp->pixelformat = smfc_fmt->v4l2_pixfmt;
	/* JPEG format has zero in smfc_fmt->bpp_buf[0] */
	for (i = 0; i < smfc_fmt->num_buffers; i++) {
		pix_mp->plane_fmt[i].bytesperline =
				(pix_mp->width * smfc_fmt->bpp_buf[i]) / 8;
		pix_mp->plane_fmt[i].sizeimage =
				pix_mp->plane_fmt[i].bytesperline;
		pix_mp->plane_fmt[i].sizeimage *= pix_mp->height;
	}

	pix_mp->field = 0;
	pix_mp->num_planes = smfc_fmt->num_buffers;

	return true;
}

static int smfc_v4l2_try_fmt_mplane(struct file *filp, void *fh,
				  struct v4l2_format *f)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(fh);
	const struct smfc_image_format *smfc_fmt =
			smfc_find_format(ctx->smfc, f->fmt.pix_mp.pixelformat);

	return smfc_v4l2_init_fmt_mplane(
			ctx, smfc_fmt, f->type, &f->fmt.pix_mp) ? 0 : -EINVAL;
}

static int smfc_v4l2_check_s_fmt(struct smfc_ctx *ctx,
				 const struct smfc_image_format *smfc_fmt,
				 __u32 type)
{
	struct vb2_queue *thisvq = v4l2_m2m_get_vq(ctx->m2mctx, type);
	struct vb2_queue *othervq = v4l2_m2m_get_vq(ctx->m2mctx,
			V4L2_TYPE_IS_OUTPUT(type) ?
				V4L2_BUF_TYPE_VIDEO_CAPTURE :
				V4L2_BUF_TYPE_VIDEO_OUTPUT);
	u32 flags;

	if (thisvq->num_buffers > 0) {
		dev_err(ctx->smfc->dev,
			"S_FMT after REQBUFS is not allowed\n");
		return -EBUSY;
	}

	flags = smfc_config_ctxflag(ctx, SMFC_CTX_COMPRESS,
			is_jpeg(smfc_fmt) != V4L2_TYPE_IS_OUTPUT(type));

	if (othervq->num_buffers > 0) { /* REQBUFSed on other vq */
		if ((flags & SMFC_CTX_COMPRESS) !=
					(ctx->flags & SMFC_CTX_COMPRESS)) {
			dev_err(ctx->smfc->dev,
				"Changing mode is prohibited after reqbufs\n");
			ctx->flags = flags;
			return -EBUSY;
		}
	}

	/* reset the buf type of vq with the given buf type */
	thisvq->type = type;

	return 0;
}



static int smfc_v4l2_s_fmt_mplane(struct file *filp, void *fh,
				    struct v4l2_format *f)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(fh);
	const struct smfc_image_format *smfc_fmt =
			smfc_find_format(ctx->smfc, f->fmt.pix_mp.pixelformat);
	int ret = smfc_v4l2_check_s_fmt(ctx, smfc_fmt, f->type);
	if (ret)
		return ret;

	if (!smfc_v4l2_init_fmt_mplane(ctx, smfc_fmt, f->type, &f->fmt.pix_mp))
		return -EINVAL;

	ctx->width = f->fmt.pix_mp.width;
	ctx->height = f->fmt.pix_mp.height;

	if (f->fmt.pix_mp.pixelformat != V4L2_PIX_FMT_JPEG)
		ctx->img_fmt = smfc_fmt;

	return 0;
}

static bool smfc_v4l2_init_fmt(const struct smfc_ctx *ctx,
				const struct smfc_image_format *smfc_fmt,
				__u32 type, struct v4l2_pix_format *pix)
{
	BUG_ON(V4L2_TYPE_IS_MULTIPLANAR(pix->pixelformat));

	if (!smfc_check_image_size(ctx->smfc->dev, type,
				smfc_fmt, pix->width, pix->height))
		return false;
	/* informs the user that the format might be changed to the default */
	pix->pixelformat = smfc_fmt->v4l2_pixfmt;
	/* JPEG format has zero in smfc_fmt->bpp_buf[0] */
	pix->bytesperline = (pix->width *  smfc_fmt->bpp_buf[0]) / 8;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = 0;

	return true;
}

static int smfc_v4l2_try_fmt(struct file *filp, void *fh, struct v4l2_format *f)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(fh);
	const struct smfc_image_format *smfc_fmt =
			smfc_find_format(ctx->smfc, f->fmt.pix.pixelformat);

	return smfc_v4l2_init_fmt(ctx, smfc_fmt, f->type, &f->fmt.pix)
								? 0 : -EINVAL;
}

static int smfc_v4l2_s_fmt(struct file *filp, void *fh, struct v4l2_format *f)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(fh);
	const struct smfc_image_format *smfc_fmt =
			smfc_find_format(ctx->smfc, f->fmt.pix.pixelformat);
	int ret = smfc_v4l2_check_s_fmt(ctx, smfc_fmt, f->type);
	if (ret)
		return ret;

	if (!smfc_v4l2_init_fmt(ctx, smfc_fmt, f->type, &f->fmt.pix))
		return -EINVAL;

	ctx->width = f->fmt.pix.width;
	ctx->height = f->fmt.pix.height;

	if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_JPEG)
		ctx->img_fmt = smfc_fmt;

	return 0;
}

static int smfc_v4l2_reqbufs(struct file *filp, void *fh,
			     struct v4l2_requestbuffers *reqbufs)
{
	return v4l2_m2m_reqbufs(filp, v4l2_fh_to_smfc_ctx(fh)->m2mctx, reqbufs);
}

static int smfc_v4l2_querybuf(struct file *filp, void *fh,
			      struct v4l2_buffer *buf)
{
	return v4l2_m2m_querybuf(filp, v4l2_fh_to_smfc_ctx(fh)->m2mctx, buf);
}

static int smfc_v4l2_qbuf(struct file *filp, void *fh, struct v4l2_buffer *buf)
{
	return v4l2_m2m_qbuf(filp, v4l2_fh_to_smfc_ctx(fh)->m2mctx, buf);
}

static int smfc_v4l2_dqbuf(struct file *filp, void *fh, struct v4l2_buffer *buf)
{
	return v4l2_m2m_dqbuf(filp, v4l2_fh_to_smfc_ctx(fh)->m2mctx, buf);
}

static int smfc_v4l2_streamon(struct file *filp, void *fh,
			      enum v4l2_buf_type type)
{
	return v4l2_m2m_streamon(filp, v4l2_fh_to_smfc_ctx(fh)->m2mctx, type);
}

static int smfc_v4l2_streamoff(struct file *filp, void *fh,
			       enum v4l2_buf_type type)
{
	return v4l2_m2m_streamoff(filp, v4l2_fh_to_smfc_ctx(fh)->m2mctx, type);
}

static const struct v4l2_ioctl_ops smfc_v4l2_ioctl_ops = {
	.vidioc_querycap		= smfc_v4l2_querycap,
	.vidioc_enum_fmt_vid_cap	= smfc_v4l2_enum_fmt,
	.vidioc_enum_fmt_vid_out	= smfc_v4l2_enum_fmt,
	.vidioc_enum_fmt_vid_cap_mplane	= smfc_v4l2_enum_fmt_mplane,
	.vidioc_enum_fmt_vid_out_mplane	= smfc_v4l2_enum_fmt_mplane,
	.vidioc_g_fmt_vid_cap		= smfc_v4l2_g_fmt,
	.vidioc_g_fmt_vid_out		= smfc_v4l2_g_fmt,
	.vidioc_g_fmt_vid_cap_mplane	= smfc_v4l2_g_fmt_mplane,
	.vidioc_g_fmt_vid_out_mplane	= smfc_v4l2_g_fmt_mplane,
	.vidioc_try_fmt_vid_cap		= smfc_v4l2_try_fmt,
	.vidioc_try_fmt_vid_out		= smfc_v4l2_try_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= smfc_v4l2_try_fmt_mplane,
	.vidioc_try_fmt_vid_out_mplane	= smfc_v4l2_try_fmt_mplane,
	.vidioc_s_fmt_vid_cap		= smfc_v4l2_s_fmt,
	.vidioc_s_fmt_vid_out		= smfc_v4l2_s_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= smfc_v4l2_s_fmt_mplane,
	.vidioc_s_fmt_vid_out_mplane	= smfc_v4l2_s_fmt_mplane,
	.vidioc_reqbufs			= smfc_v4l2_reqbufs,
	.vidioc_querybuf		= smfc_v4l2_querybuf,
	.vidioc_qbuf			= smfc_v4l2_qbuf,
	.vidioc_dqbuf			= smfc_v4l2_dqbuf,
	.vidioc_streamon		= smfc_v4l2_streamon,
	.vidioc_streamoff		= smfc_v4l2_streamoff,
};

static void smfc_m2m_device_run(void *priv)
{
	struct smfc_ctx *ctx = priv;
	int ret;

	ret = in_irq() ? pm_runtime_get(ctx->smfc->dev) :
			 pm_runtime_get_sync(ctx->smfc->dev);
	if (ret < 0) {
		pr_err("Failed to enable power\n");
		/* TODO: error current frame */
	}

	if (!IS_ERR(ctx->smfc->clk_gate)) {
		ret = clk_enable(ctx->smfc->clk_gate);
		if (!ret && !IS_ERR(ctx->smfc->clk_gate2)) {
			ret = clk_enable(ctx->smfc->clk_gate2);
			if (ret)
				clk_disable(ctx->smfc->clk_gate);
		}
	}

	if (ret < 0) {
		dev_err(ctx->smfc->dev, "Failed to enable clocks\n");
		pm_runtime_put(ctx->smfc->dev);
		/* TODO: error current frame */
	}

	smfc_hwconfigure_reset(ctx->smfc);
	smfc_hwconfigure_tables(ctx);
	smfc_hwconfigure_image(ctx);
	smfc_hwconfigure_start(ctx);
}

static void smfc_m2m_job_abort(void *priv)
{
	/* TODO: aborting next job */
}

static struct v4l2_m2m_ops smfc_m2m_ops = {
	.device_run	= smfc_m2m_device_run,
	.job_abort	= smfc_m2m_job_abort,
};

static int smfc_init_v4l2(struct device *dev, struct smfc_dev *smfc)
{
	int ret;
	size_t str_len;

	strncpy(smfc->v4l2_dev.name, "exynos-hwjpeg",
		sizeof(smfc->v4l2_dev.name) - 1);
	smfc->v4l2_dev.name[sizeof(smfc->v4l2_dev.name) - 1] = '\0';

	ret = v4l2_device_register(dev, &smfc->v4l2_dev);
	if (ret) {
		dev_err(dev, "Failed to register v4l2 device\n");
		return ret;
	}

	smfc->videodev = video_device_alloc();
	if (!smfc->videodev) {
		dev_err(dev, "Failed to allocate video_device");
		ret = -ENOMEM;
		goto err_videodev_alloc;
	}

	str_len = sizeof(smfc->videodev->name);
	if (smfc->device_id < 0) {
		strncpy(smfc->videodev->name, MODULE_NAME, str_len);
		smfc->videodev->name[str_len - 1] = '\0';
	} else {
		scnprintf(smfc->videodev->name, str_len,
			  "%s.%d", MODULE_NAME, smfc->device_id);
	}

	mutex_init(&smfc->video_device_mutex);

	smfc->videodev->fops		= &smfc_v4l2_fops;
	smfc->videodev->ioctl_ops	= &smfc_v4l2_ioctl_ops;
	smfc->videodev->release		= video_device_release;
	smfc->videodev->lock		= &smfc->video_device_mutex;
	smfc->videodev->vfl_dir		= VFL_DIR_M2M;
	smfc->videodev->v4l2_dev	= &smfc->v4l2_dev;

	video_set_drvdata(smfc->videodev, smfc);

	smfc->m2mdev = v4l2_m2m_init(&smfc_m2m_ops);
	if (IS_ERR(smfc->m2mdev)) {
		ret = PTR_ERR(smfc->m2mdev);
		dev_err(dev, "Failed(%d) to create v4l2_m2m_device\n", ret);
		goto err_m2m_init;
	}

	/* TODO: promote the magic number 12 in public */
	ret = video_register_device(smfc->videodev, VFL_TYPE_GRABBER, 12);
	if (ret < 0) {
		dev_err(dev, "Failed(%d) to register video_device[%d]\n",
			ret, 12);
		goto err_register;
	}

	return 0;

err_register:
	v4l2_m2m_release(smfc->m2mdev);
err_m2m_init:
	video_device_release(smfc->videodev);
err_videodev_alloc:
	v4l2_device_unregister(&smfc->v4l2_dev);
	return ret;
}

static int smfc_init_clock(struct device *dev, struct smfc_dev *smfc)
{
	smfc->clk_gate = devm_clk_get(dev, "gate");
	if (IS_ERR(smfc->clk_gate)) {
		if (PTR_ERR(smfc->clk_gate) != -ENOENT) {
			dev_err(dev, "Failed(%ld) to get 'gate' clock",
				PTR_ERR(smfc->clk_gate));
			return PTR_ERR(smfc->clk_gate);
		}

		dev_info(dev, "'gate' clock is note present\n");
		smfc->clk_gate2 = ERR_PTR(-ENOENT);
		return 0;
	}

	smfc->clk_gate2 = devm_clk_get(dev, "gate2");
	if (IS_ERR(smfc->clk_gate2) && (PTR_ERR(smfc->clk_gate2) != -ENOENT)) {
		dev_err(dev, "Failed(%ld) to get 'gate2' clock\n",
				PTR_ERR(smfc->clk_gate2));
		clk_put(smfc->clk_gate);
		return PTR_ERR(smfc->clk_gate2);
	}

	return 0;
}

static int smfc_find_hw_version(struct device *dev, struct smfc_dev *smfc)
{
	int ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "Failed(%d) to get the local power\n", ret);
		return ret;
	}

	if (!IS_ERR(smfc->clk_gate)) {
		ret = clk_prepare_enable(smfc->clk_gate);
		if (!ret && !IS_ERR(smfc->clk_gate2))
			ret = clk_prepare_enable(smfc->clk_gate2);
		if (ret) {
			clk_disable_unprepare(smfc->clk_gate);
			dev_err(dev, "Failed(%d) to get gate clocks\n", ret);
			goto err_clk;
		}
	}

	if (ret >= 0) {
		smfc->hwver = readl(smfc->reg + REG_IP_VERSION_NUMBER);
		if (!IS_ERR(smfc->clk_gate)) {
			clk_disable_unprepare(smfc->clk_gate);
			if (!IS_ERR(smfc->clk_gate2))
				clk_disable_unprepare(smfc->clk_gate2);
		}
	}

err_clk:
	pm_runtime_put(dev);

	return ret;
}

static int exynos_smfc_probe(struct platform_device *pdev)
{
	struct smfc_dev *smfc;
	struct resource *res;
	int ret;

	smfc = devm_kzalloc(&pdev->dev, sizeof(*smfc), GFP_KERNEL);
	if (!smfc) {
		dev_err(&pdev->dev, "Failed to get allocate drvdata");
		return -ENOMEM;
	}

	smfc->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smfc->reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(smfc->reg))
		return PTR_ERR(smfc->reg);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Failed to get IRQ resource");
		return -ENOENT;
	}

	ret = devm_request_irq(&pdev->dev, res->start, exynos_smfc_irq_handler,
				0, pdev->name, smfc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to install IRQ handler");
		return ret;
	}

	ret = smfc_init_clock(&pdev->dev, smfc);
	if (ret)
		return ret;

	smfc->device_id = of_alias_get_id(pdev->dev.of_node, "jpeg");
	if (smfc->device_id < 0) {
		dev_info(&pdev->dev,
			"device ID is not declared: unique device\n");
		smfc->device_id = -1;
	}

	pm_runtime_enable(&pdev->dev);

	ret = smfc_find_hw_version(&pdev->dev, smfc);
	if (ret < 0)
		return ret;

	ret = smfc_init_v4l2(&pdev->dev, smfc);
	if (ret < 0)
		return ret;

	ret = iovmm_activate(&pdev->dev);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, smfc);

	dev_info(&pdev->dev, "Probed H/W Version: %02x.%02x.%04x\n",
			(smfc->hwver >> 24) & 0xFF, (smfc->hwver >> 16) & 0xFF,
			smfc->hwver & 0xFFFF);
	return 0;
}

static void smfc_deinit_clock(struct smfc_dev *smfc)
{
	if (!IS_ERR(smfc->clk_gate2))
		clk_put(smfc->clk_gate2);
	if (!IS_ERR(smfc->clk_gate))
		clk_put(smfc->clk_gate);
}

static int exynos_smfc_remove(struct platform_device *pdev)
{
	struct smfc_dev *smfc = platform_get_drvdata(pdev);

	smfc_deinit_clock(smfc);

	return 0;
}

static const struct of_device_id exynos_smfc_match[] = {
	{
		.compatible = "samsung,exynos-jpeg",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_smfc_match);

#ifdef CONFIG_PM_SLEEP
static int smfc_suspend(struct device *dev)
{
	return 0;
}

static int smfc_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM
static int smfc_runtime_resume(struct device *dev)
{
	return 0;
}

static int smfc_runtime_suspend(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops exynos_smfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(smfc_suspend, smfc_resume)
	SET_RUNTIME_PM_OPS(NULL, smfc_runtime_resume, smfc_runtime_suspend)
};

static struct platform_driver exynos_smfc_driver = {
	.probe		= exynos_smfc_probe,
	.remove		= exynos_smfc_remove,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &exynos_smfc_pm_ops,
		.of_match_table = of_match_ptr(exynos_smfc_match),
	}
};

module_platform_driver(exynos_smfc_driver);

MODULE_AUTHOR("Cho KyongHo <pullip.cho@samsung.com>");
MODULE_DESCRIPTION("Exynos Still MFC(JPEG) V4L2 Driver");
MODULE_LICENSE("GPL");
