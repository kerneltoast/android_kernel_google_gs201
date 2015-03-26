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

struct smfc_ctx {
	struct v4l2_fh v4l2_fh;
	struct smfc_dev *smfc;
	struct v4l2_m2m_ctx *m2mctx;
};

static inline struct smfc_ctx *v4l2_fh_to_smfc_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct smfc_ctx, v4l2_fh);
}

#endif /* _MEDIA_EXYNOS_SMFC_H_ */
