/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * The main header file of Samsung Exynos SMFC Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MEDIA_EXYNOS_SMFC_H_
#define _MEDIA_EXYNOS_SMFC_H_

#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>

#include "smfc-regs.h"

#define MODULE_NAME	"exynos-jpeg"

struct device;
struct video_device;

struct smfc_image_format {
	const char	*description;
	__u32		v4l2_pixfmt;
	u32		regcfg;
	unsigned char	bpp_buf[3];
	unsigned char	bpp_pix[3];
	unsigned char	num_planes;
	unsigned char	num_buffers;
	unsigned char	chroma_hfactor;
	unsigned char	chroma_vfactor;
};

static inline bool is_jpeg(const struct smfc_image_format *fmt)
{
	return fmt->bpp_buf[0] == 0;
}

struct smfc_dev {
	struct v4l2_device v4l2_dev;
	struct video_device *videodev;
	struct v4l2_m2m_dev *m2mdev;
	struct device *dev;
	void __iomem *reg;
	struct mutex video_device_mutex;
	int device_id;
	u32 hwver;

	struct clk *clk_gate;
	struct clk *clk_gate2; /* available if clk_gate is valid */
};

#define SMFC_CTX_COMPRESS	(1 << 0)

struct smfc_ctx {
	struct v4l2_fh v4l2_fh;
	struct v4l2_ctrl_handler v4l2_ctrlhdlr;
	struct smfc_dev *smfc;
	struct v4l2_m2m_ctx *m2mctx;
	u32 flags;
	/* uncomressed image description */
	const struct smfc_image_format *img_fmt;
	__u32 width;
	__u32 height;
	/* JPEG chroma subsampling factors */
	unsigned char chroma_hfactor;
	unsigned char chroma_vfactor;
	unsigned char restart_interval;
	unsigned char quality_factor;
};

static inline struct smfc_ctx *v4l2_fh_to_smfc_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct smfc_ctx, v4l2_fh);
}

/* return the previous flag */
static inline u32 smfc_config_ctxflag(struct smfc_ctx *ctx, u32 flag, bool set)
{
	u32 prevflags = ctx->flags;
	ctx->flags = set ? ctx->flags | flag : ctx->flags & ~flag;
	return prevflags;
}

static inline bool smfc_is_compressed_type(struct smfc_ctx *ctx, __u32 type)
{
	return !(ctx->flags & SMFC_CTX_COMPRESS) == V4L2_TYPE_IS_OUTPUT(type);
}

/* H/W Configuration */
void smfc_hwconfigure_tables(struct smfc_ctx *ctx);
void smfc_hwconfigure_image(struct smfc_ctx *ctx);
void smfc_hwconfigure_start(struct smfc_ctx *ctx);
bool smfc_hwstatus_okay(struct smfc_dev *smfc);
void smfc_hwconfigure_reset(struct smfc_dev *smfc);
static inline u32 smfc_get_streamsize(struct smfc_dev *smfc)
{
	return __raw_readl(smfc->reg + REG_MAIN_STREAM_SIZE);
}

#endif /* _MEDIA_EXYNOS_SMFC_H_ */
