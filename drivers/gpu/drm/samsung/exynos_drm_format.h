/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Format Header file for Exynos DPU driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_FORMAT_H__
#define __EXYNOS_FORMAT_H__

#include <linux/types.h>
#include <linux/kernel.h>

#define DPU_UNDEF_BITS_DEPTH		0xabcd

enum dpu_pixel_format {
	/* RGB32 */
	DPU_PIXEL_FORMAT_ARGB_8888 = 0,
	DPU_PIXEL_FORMAT_ABGR_8888,
	DPU_PIXEL_FORMAT_RGBA_8888,
	DPU_PIXEL_FORMAT_BGRA_8888,
	DPU_PIXEL_FORMAT_XRGB_8888,
	DPU_PIXEL_FORMAT_XBGR_8888,
	DPU_PIXEL_FORMAT_RGBX_8888,
	DPU_PIXEL_FORMAT_BGRX_8888,
	/* RGB16 */
	DPU_PIXEL_FORMAT_RGB_565,
	DPU_PIXEL_FORMAT_BGR_565,
	/* RGB32, 10 bits per channel */
	DPU_PIXEL_FORMAT_ARGB_2101010,
	DPU_PIXEL_FORMAT_ABGR_2101010,
	DPU_PIXEL_FORMAT_RGBA_1010102,
	DPU_PIXEL_FORMAT_BGRA_1010102,

	/* YUV420 2 plane */
	DPU_PIXEL_FORMAT_NV12,
	/* YVU420 2 plane */
	DPU_PIXEL_FORMAT_NV21,
	/* YUV420, 4 plane, 10 bpc, packed 8P2 vendor specific format */
	DPU_PIXEL_FORMAT_YUV420_8P2,
	/* YUV420, 2 plane, 10 bpc, planar format */
	DPU_PIXEL_FORMAT_YUV420_P010,

	DPU_PIXEL_FORMAT_MAX,
};

enum dpu_colorspace {
	DPU_COLORSPACE_RGB,
	DPU_COLORSPACE_YUV420,
	DPU_COLORSPACE_YUV422,
};

struct dpu_fmt {
	const char *name;
	enum dpu_pixel_format fmt; /* user-interfaced color format */
	u32 dma_fmt;		   /* applied color format to DPU_DMA(In) */
	u32 dpp_fmt;		   /* applied color format to DPP(Out) */
	u8 bpp;			   /* bits per pixel */
	u8 padding;		   /* padding bits per pixel */
	u8 bpc;			   /* bits per each color component */
	u8 num_planes;		   /* plane(s) count of color format */
	u8 num_meta_planes;	   /* number of meta plane(s) */
	u8 len_alpha;		   /* length of alpha bits */
	enum dpu_colorspace cs;
};

/* format */
#define IS_YUV420(f)		((f)->cs == DPU_COLORSPACE_YUV420)
#define IS_YUV422(f)		((f)->cs == DPU_COLORSPACE_YUV422)
#define IS_YUV(f)		(((f)->cs == DPU_COLORSPACE_YUV420) ||	\
				((f)->cs == DPU_COLORSPACE_YUV422))
#define IS_YUV10(f)		(IS_YUV(f) && ((f)->bpc == 10))
#define IS_RGB32(f)	\
	(((f)->cs == DPU_COLORSPACE_RGB) && (((f)->bpp + (f)->padding) == 32))
#define IS_10BPC(f)		((f)->bpc == 10)
#define IS_OPAQUE(f)		((f)->len_alpha == 0)

const struct dpu_fmt *dpu_find_fmt_info(enum dpu_pixel_format fmt);
enum dpu_pixel_format convert_drm_format(u32 drm_format);

#endif /* __EXYNOS_FORMAT_H__ */
