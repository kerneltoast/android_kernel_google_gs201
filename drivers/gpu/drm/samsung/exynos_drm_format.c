// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * DPU format file for Samsung EXYNOS DPU driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "cal_9820/regs-dpp.h"
#include "cal_9820/dpp_cal.h"
#include "exynos_drm_format.h"
#include <drm/drmP.h>

static const struct dpu_fmt dpu_formats_list[] = {
	{
		.name = "ARGB8888",
		.fmt = DPU_PIXEL_FORMAT_ARGB_8888,
		.dma_fmt = IDMA_IMG_FORMAT_ARGB8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 32,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 8,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "ABGR8888",
		.fmt = DPU_PIXEL_FORMAT_ABGR_8888,
		.dma_fmt = IDMA_IMG_FORMAT_ABGR8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 32,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 8,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "RGBA8888",
		.fmt = DPU_PIXEL_FORMAT_RGBA_8888,
		.dma_fmt = IDMA_IMG_FORMAT_RGBA8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 32,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 8,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "BGRA8888",
		.fmt = DPU_PIXEL_FORMAT_BGRA_8888,
		.dma_fmt = IDMA_IMG_FORMAT_BGRA8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 32,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 8,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "XRGB8888",
		.fmt = DPU_PIXEL_FORMAT_XRGB_8888,
		.dma_fmt = IDMA_IMG_FORMAT_XRGB8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 24,
		.padding = 8,
		.bpc = 8,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "XBGR8888",
		.fmt = DPU_PIXEL_FORMAT_XBGR_8888,
		.dma_fmt = IDMA_IMG_FORMAT_XBGR8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 24,
		.padding = 8,
		.bpc = 8,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "RGBX8888",
		.fmt = DPU_PIXEL_FORMAT_RGBX_8888,
		.dma_fmt = IDMA_IMG_FORMAT_RGBX8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 24,
		.padding = 8,
		.bpc = 8,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "BGRX8888",
		.fmt = DPU_PIXEL_FORMAT_BGRX_8888,
		.dma_fmt = IDMA_IMG_FORMAT_BGRX8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 24,
		.padding = 8,
		.bpc = 8,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "RGB565",
		.fmt = DPU_PIXEL_FORMAT_RGB_565,
		.dma_fmt = IDMA_IMG_FORMAT_RGB565,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 16,
		.padding = 0,
		.bpc = (u8)DPU_UNDEF_BITS_DEPTH,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "BGR565",
		.fmt = DPU_PIXEL_FORMAT_BGR_565,
		.dma_fmt = IDMA_IMG_FORMAT_BGR565,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 16,
		.padding = 0,
		.bpc = (u8)DPU_UNDEF_BITS_DEPTH,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "ARGB2101010",
		.fmt = DPU_PIXEL_FORMAT_ARGB_2101010,
		.dma_fmt = IDMA_IMG_FORMAT_ARGB2101010,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8101010,
		.bpp = 32,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 1,
		.len_alpha = 2,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "ABGR2101010",
		.fmt = DPU_PIXEL_FORMAT_ABGR_2101010,
		.dma_fmt = IDMA_IMG_FORMAT_ABGR2101010,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8101010,
		.bpp = 32,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 1,
		.len_alpha = 2,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "RGBA1010102",
		.fmt = DPU_PIXEL_FORMAT_RGBA_1010102,
		.dma_fmt = IDMA_IMG_FORMAT_RGBA1010102,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8101010,
		.bpp = 32,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 1,
		.len_alpha = 2,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "BGRA1010102",
		.fmt = DPU_PIXEL_FORMAT_BGRA_1010102,
		.dma_fmt = IDMA_IMG_FORMAT_BGRA1010102,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8101010,
		.bpp = 32,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.num_buffers = 1,
		.num_meta_planes = 1,
		.len_alpha = 2,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "YVU422_3P",	/* not support */
		.fmt = DPU_PIXEL_FORMAT_YVU422_3P,
		.dma_fmt = 0,
		.dpp_fmt = 0,
		.bpp = 16,
		.padding = 0,
		.bpc = 8,
		.num_planes = 3,
		.num_buffers = 3,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV422,
	}, {
		.name = "NV12",
		.fmt = DPU_PIXEL_FORMAT_NV12,
		.dma_fmt = IDMA_IMG_FORMAT_YVU420_2P,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 2,
		.num_buffers = 2,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV21",
		.fmt = DPU_PIXEL_FORMAT_NV21,
		.dma_fmt = IDMA_IMG_FORMAT_YUV420_2P,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 2,
		.num_buffers = 2,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV12M",
		.fmt = DPU_PIXEL_FORMAT_NV12M,
		.dma_fmt = IDMA_IMG_FORMAT_YVU420_2P,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 2,
		.num_buffers = 2,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV21M",
		.fmt = DPU_PIXEL_FORMAT_NV21M,
		.dma_fmt = IDMA_IMG_FORMAT_YUV420_2P,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 2,
		.num_buffers = 2,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "YUV420",	/* not support */
		.fmt = DPU_PIXEL_FORMAT_YUV420,
		.dma_fmt = 0,
		.dpp_fmt = 0,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 3,
		.num_buffers = 3,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "YVU420",	/* not support */
		.fmt = DPU_PIXEL_FORMAT_YVU420,
		.dma_fmt = 0,
		.dpp_fmt = 0,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 3,
		.num_buffers = 3,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "YUV420M",	/* not support */
		.fmt = DPU_PIXEL_FORMAT_YUV420M,
		.dma_fmt = 0,
		.dpp_fmt = 0,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 3,
		.num_buffers = 3,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "YVU420M",	/* not support */
		.fmt = DPU_PIXEL_FORMAT_YVU420M,
		.dma_fmt = 0,
		.dpp_fmt = 0,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 3,
		.num_buffers = 3,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV12N",
		.fmt = DPU_PIXEL_FORMAT_NV12N,
		.dma_fmt = IDMA_IMG_FORMAT_YVU420_2P,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 2,
		.num_buffers = 1,
		.num_meta_planes = 0,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV12N_10B",
		.fmt = DPU_PIXEL_FORMAT_NV12N_10B,
		.dma_fmt = IDMA_IMG_FORMAT_YVU420_8P2,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P2,
		.bpp = 15,
		.padding = 0,
		.bpc = 10,
		.num_planes = 4,
		.num_buffers = 1,
		.num_meta_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV12M_P010",
		.fmt = DPU_PIXEL_FORMAT_NV12M_P010,
		.dma_fmt = IDMA_IMG_FORMAT_YUV420_P010,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_P010,
		.bpp = 15,
		.padding = 9,
		.bpc = 10,
		.num_planes = 2,
		.num_buffers = 2,
		.num_meta_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV21M_P010",
		.fmt = DPU_PIXEL_FORMAT_NV21M_P010,
		.dma_fmt = IDMA_IMG_FORMAT_YVU420_P010,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_P010,
		.bpp = 15,
		.padding = 9,
		.bpc = 10,
		.num_planes = 2,
		.num_buffers = 2,
		.num_meta_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV12M_S10B",
		.fmt = DPU_PIXEL_FORMAT_NV12M_S10B,
		.dma_fmt = IDMA_IMG_FORMAT_YVU420_8P2,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P2,
		.bpp = 15,
		.padding = 0,
		.bpc = 10,
		.num_planes = 4,
		.num_buffers = 2,
		.num_meta_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV21M_S10B",
		.fmt = DPU_PIXEL_FORMAT_NV21M_S10B,
		.dma_fmt = IDMA_IMG_FORMAT_YUV420_8P2,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P2,
		.bpp = 15,
		.padding = 0,
		.bpc = 10,
		.num_planes = 4,
		.num_buffers = 2,
		.num_meta_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV12_P010",
		.fmt = DPU_PIXEL_FORMAT_NV12_P010,
		.dma_fmt = IDMA_IMG_FORMAT_YUV420_P010,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_P010,
		.bpp = 15,
		.padding = 9,
		.bpc = 10,
		.num_planes = 2,
		.num_buffers = 1,
		.num_meta_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	},
};

const struct dpu_fmt *dpu_find_fmt_info(enum dpu_pixel_format fmt)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu_formats_list); i++)
		if (dpu_formats_list[i].fmt == fmt)
			return &dpu_formats_list[i];

	DRM_INFO("%s: can't find format(%d) in supported format list\n",
			__func__, fmt);
	return NULL;
}

enum dpu_pixel_format convert_drm_format(u32 drm_format)
{
	switch (drm_format) {
	case DRM_FORMAT_ARGB8888:	return DPU_PIXEL_FORMAT_ARGB_8888;
	case DRM_FORMAT_ABGR8888:	return DPU_PIXEL_FORMAT_ABGR_8888;
	case DRM_FORMAT_RGBA8888:	return DPU_PIXEL_FORMAT_RGBA_8888;
	case DRM_FORMAT_BGRA8888:	return DPU_PIXEL_FORMAT_BGRA_8888;
	case DRM_FORMAT_XRGB8888:	return DPU_PIXEL_FORMAT_XRGB_8888;
	case DRM_FORMAT_XBGR8888:	return DPU_PIXEL_FORMAT_XBGR_8888;
	case DRM_FORMAT_RGBX8888:	return DPU_PIXEL_FORMAT_RGBX_8888;
	case DRM_FORMAT_BGRX8888:	return DPU_PIXEL_FORMAT_BGRX_8888;
	case DRM_FORMAT_RGB565:		return DPU_PIXEL_FORMAT_RGB_565;
	case DRM_FORMAT_BGR565:		return DPU_PIXEL_FORMAT_BGR_565;
	case DRM_FORMAT_ARGB2101010:	return DPU_PIXEL_FORMAT_ARGB_2101010;
	case DRM_FORMAT_ABGR2101010:	return DPU_PIXEL_FORMAT_ABGR_2101010;
	case DRM_FORMAT_RGBA1010102:	return DPU_PIXEL_FORMAT_RGBA_1010102;
	case DRM_FORMAT_BGRA1010102:	return DPU_PIXEL_FORMAT_BGRA_1010102;
	case DRM_FORMAT_NV12:		return DPU_PIXEL_FORMAT_NV12;
	case DRM_FORMAT_NV21:		return DPU_PIXEL_FORMAT_NV21;
	default:
		return DPU_PIXEL_FORMAT_MAX;
	}
}
