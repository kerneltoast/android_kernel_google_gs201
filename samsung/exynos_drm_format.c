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

#include <drm/drm_print.h>
#include <uapi/drm/drm_fourcc.h>

#include <dpp_cal.h>
#include <regs-dpp.h>

#include "exynos_drm_format.h"

static const struct dpu_fmt dpu_formats_list[] = {
	{
		.name = "C8",
		.fmt = DRM_FORMAT_C8,
		.bpp = 8,
		.padding = 0,
		.num_planes = 1,
		.len_alpha = 0,
	}, {
		.name = "ARGB8888",
		.fmt = DRM_FORMAT_ARGB8888,
		.dma_fmt = IDMA_IMG_FORMAT_ARGB8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 32,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 8,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "ABGR8888",
		.fmt = DRM_FORMAT_ABGR8888,
		.dma_fmt = IDMA_IMG_FORMAT_ABGR8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 32,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 8,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "RGBA8888",
		.fmt = DRM_FORMAT_RGBA8888,
		.dma_fmt = IDMA_IMG_FORMAT_RGBA8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 32,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 8,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "BGRA8888",
		.fmt = DRM_FORMAT_BGRA8888,
		.dma_fmt = IDMA_IMG_FORMAT_BGRA8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 32,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 8,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "XRGB8888",
		.fmt = DRM_FORMAT_XRGB8888,
		.dma_fmt = IDMA_IMG_FORMAT_XRGB8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 24,
		.padding = 8,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "XBGR8888",
		.fmt = DRM_FORMAT_XBGR8888,
		.dma_fmt = IDMA_IMG_FORMAT_XBGR8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 24,
		.padding = 8,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "RGBX8888",
		.fmt = DRM_FORMAT_RGBX8888,
		.dma_fmt = IDMA_IMG_FORMAT_RGBX8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 24,
		.padding = 8,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "BGRX8888",
		.fmt = DRM_FORMAT_BGRX8888,
		.dma_fmt = IDMA_IMG_FORMAT_BGRX8888,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 24,
		.padding = 8,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "RGB565",
		.fmt = DRM_FORMAT_RGB565,
		.dma_fmt = IDMA_IMG_FORMAT_RGB565,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 16,
		.padding = 0,
		.bpc = (u8)DPU_UNDEF_BITS_DEPTH,
		.num_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "BGR565",
		.fmt = DRM_FORMAT_BGR565,
		.dma_fmt = IDMA_IMG_FORMAT_BGR565,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8888,
		.bpp = 16,
		.padding = 0,
		.bpc = (u8)DPU_UNDEF_BITS_DEPTH,
		.num_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "ARGB2101010",
		.fmt = DRM_FORMAT_ARGB2101010,
		.dma_fmt = IDMA_IMG_FORMAT_ARGB2101010,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8101010,
		.bpp = 32,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.len_alpha = 2,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "ABGR2101010",
		.fmt = DRM_FORMAT_ABGR2101010,
		.dma_fmt = IDMA_IMG_FORMAT_ABGR2101010,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8101010,
		.bpp = 32,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.len_alpha = 2,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "RGBA1010102",
		.fmt = DRM_FORMAT_RGBA1010102,
		.dma_fmt = IDMA_IMG_FORMAT_RGBA1010102,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8101010,
		.bpp = 32,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.len_alpha = 2,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "BGRA1010102",
		.fmt = DRM_FORMAT_BGRA1010102,
		.dma_fmt = IDMA_IMG_FORMAT_BGRA1010102,
		.dpp_fmt = DPP_IMG_FORMAT_ARGB8101010,
		.bpp = 32,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.len_alpha = 2,
		.cs = DPU_COLORSPACE_RGB,
	}, {
		.name = "NV12",
		.fmt = DRM_FORMAT_NV12,
		.dma_fmt = IDMA_IMG_FORMAT_NV12,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 2,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV21",
		.fmt = DRM_FORMAT_NV21,
		.dma_fmt = IDMA_IMG_FORMAT_NV21,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 2,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "P010",
		.fmt = DRM_FORMAT_P010,
		.dma_fmt = IDMA_IMG_FORMAT_YUV420_P010,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_P010,
		.bpp = 15,
		.padding = 9,
		.bpc = 10,
		.num_planes = 2,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "Y010_8P2",
		.fmt = DRM_FORMAT_Y010,
		.dma_fmt = IDMA_IMG_FORMAT_YUV420_8P2,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P2,
		.bpp = 15,
		.padding = 0,
		.bpc = 10,
		.num_planes = 4,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
	}, {
		.name = "NV12_AFBC",
		.fmt = DRM_FORMAT_YUV420_8BIT,
		.dma_fmt = IDMA_IMG_FORMAT_NV12,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_8P,
		.bpp = 12,
		.padding = 0,
		.bpc = 8,
		.num_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
        }, {
		.name = "P010_AFBC",
		.fmt = DRM_FORMAT_YUV420_10BIT,
		.dma_fmt = IDMA_IMG_FORMAT_YUV420_P010,
		.dpp_fmt = DPP_IMG_FORMAT_YUV420_P010,
		.bpp = 15,
		.padding = 0,
		.bpc = 10,
		.num_planes = 1,
		.len_alpha = 0,
		.cs = DPU_COLORSPACE_YUV420,
        },
};

const struct dpu_fmt *dpu_find_fmt_info(u32 fmt)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu_formats_list); i++)
		if (dpu_formats_list[i].fmt == fmt)
			return &dpu_formats_list[i];

	DRM_INFO("%s: can't find format(%d) in supported format list\n",
			__func__, fmt);

	return NULL;
}
