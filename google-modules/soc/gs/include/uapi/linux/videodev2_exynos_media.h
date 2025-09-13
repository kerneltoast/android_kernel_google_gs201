/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Video for Linux Two header file for Exynos
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This header file contains several v4l2 APIs to be proposed to v4l2
 * community and until being accepted, will be used restrictly for Exynos.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_VIDEODEV2_EXYNOS_MEDIA_H
#define __LINUX_VIDEODEV2_EXYNOS_MEDIA_H

#include <linux/videodev2.h>

/*
 *	C O N T R O L S
 */
/* CID base for Exynos controls (USER_CLASS) */
#define V4L2_CID_EXYNOS_BASE		(V4L2_CTRL_CLASS_USER | 0x2000)

/* cacheable configuration */
#define V4L2_CID_CACHEABLE		(V4L2_CID_EXYNOS_BASE + 10)

/* for color space conversion equation selection */
#define V4L2_CID_CSC_EQ_MODE		(V4L2_CID_EXYNOS_BASE + 100)
#define V4L2_CID_CSC_EQ			(V4L2_CID_EXYNOS_BASE + 101)
#define V4L2_CID_CSC_RANGE		(V4L2_CID_EXYNOS_BASE + 102)

/* for DRM playback scenario */
#define V4L2_CID_CONTENT_PROTECTION	(V4L2_CID_EXYNOS_BASE + 201)

/*
 *	V I D E O   I M A G E   F O R M A T
 */
/* 1 plane -- one Y, one Cb + Cr interleaved, non contiguous  */
#define V4L2_PIX_FMT_NV12N		v4l2_fourcc('N', 'N', '1', '2')
#define V4L2_PIX_FMT_NV12NT		v4l2_fourcc('T', 'N', '1', '2')

/* 1 plane -- one Y, one Cb, one Cr, non contiguous */
#define V4L2_PIX_FMT_YUV420N		v4l2_fourcc('Y', 'N', '1', '2')
#define V4L2_PIX_FMT_YVU420N		v4l2_fourcc('Y', 'N', '2', '1')

/* 1 plane -- 8bit Y, 2bit Y, 8bit Cb + Cr interleaved, 2bit Cb + Cr interleaved, non contiguous */
#define V4L2_PIX_FMT_NV12N_10B		v4l2_fourcc('B', 'N', '1', '2')
#define V4L2_PIX_FMT_NV12M_S10B		v4l2_fourcc('B', 'M', '1', '2')
#define V4L2_PIX_FMT_NV21M_S10B		v4l2_fourcc('B', 'M', '2', '1')
#define V4L2_PIX_FMT_NV16M_S10B		v4l2_fourcc('B', 'M', '1', '6')
#define V4L2_PIX_FMT_NV61M_S10B		v4l2_fourcc('B', 'M', '6', '1')
#define V4L2_PIX_FMT_NV12M_P010		v4l2_fourcc('P', 'M', '1', '2')
#define V4L2_PIX_FMT_NV21M_P010		v4l2_fourcc('P', 'M', '2', '1')
#define V4L2_PIX_FMT_NV16M_P210		v4l2_fourcc('P', 'M', '1', '6')
#define V4L2_PIX_FMT_NV61M_P210		v4l2_fourcc('P', 'M', '6', '1')

#define V4L2_PIX_FMT_NV12N_P010		v4l2_fourcc('N', 'P', '1', '2')
#define V4L2_PIX_FMT_NV12_P010		v4l2_fourcc('P', 'N', '1', '2')

/* 12 Y/CbCr 4:2:0 SBWC */
#define V4L2_PIX_FMT_NV12M_SBWC_8B	v4l2_fourcc('M', '1', 'S', '8')
#define V4L2_PIX_FMT_NV12M_SBWC_10B	v4l2_fourcc('M', '1', 'S', '1')

/* 21 Y/CrCb 4:2:0 SBWC */
#define V4L2_PIX_FMT_NV21M_SBWC_8B	v4l2_fourcc('M', '2', 'S', '8')
#define V4L2_PIX_FMT_NV21M_SBWC_10B	v4l2_fourcc('M', '2', 'S', '1')

/* 12 Y/CbCr 4:2:0 SBWC single */
#define V4L2_PIX_FMT_NV12N_SBWC_8B	v4l2_fourcc('N', '1', 'S', '8')
#define V4L2_PIX_FMT_NV12N_SBWC_10B	v4l2_fourcc('N', '1', 'S', '1')

/* 12 Y/CbCr 4:2:0 SBWC Lossy */
#define V4L2_PIX_FMT_NV12M_SBWCL_8B	v4l2_fourcc('M', '1', 'L', '8')
#define V4L2_PIX_FMT_NV12M_SBWCL_10B	v4l2_fourcc('M', '1', 'L', '1')

/* 12 Y/CbCr 4:2:0 SBWC Lossy single */
#define V4L2_PIX_FMT_NV12N_SBWCL_8B	v4l2_fourcc('N', '1', 'L', '8')
#define V4L2_PIX_FMT_NV12N_SBWCL_10B	v4l2_fourcc('N', '1', 'L', '1')

/* 12 Y/CbCr 4:2:0 AFBC */
#define V4L2_PIX_FMT_NV12M_AFBC_8B	v4l2_fourcc('M', '1', 'A', '8')
#define V4L2_PIX_FMT_NV12M_AFBC_10B	v4l2_fourcc('M', '1', 'A', '1')

/* helper macros */
#ifndef __ALIGN_UP
#define __ALIGN_UP(x, a)		(((x) + ((a) - 1)) & ~((a) - 1))
#endif

#define NV12N_STRIDE(w)			(__ALIGN_UP((w), 64))
#define NV12N_Y_SIZE(w, h)		(NV12N_STRIDE(w) * __ALIGN_UP((h), 16))
#define NV12N_CBCR_SIZE(w, h)		(NV12N_STRIDE(w) * __ALIGN_UP((h), 16) / 2)
#define NV12N_CBCR_BASE(base, w, h)		\
	((base) + NV12N_Y_SIZE((w), (h)))
#define NV12N_10B_Y_8B_SIZE(w, h)		\
	(__ALIGN_UP((w), 64) * __ALIGN_UP((h), 16) + 256)
#define NV12N_10B_Y_2B_SIZE(w, h)		\
	((__ALIGN_UP((w) / 4, 16) * __ALIGN_UP((h), 16) + 64))
#define NV12N_10B_CBCR_8B_SIZE(w, h)		\
(__ALIGN_UP((__ALIGN_UP((w), 64) * (__ALIGN_UP((h), 16) / 2) + 256), 16))
#define NV12N_10B_CBCR_2B_SIZE(w, h)		\
	((__ALIGN_UP((w) / 4, 16) * (__ALIGN_UP((h), 16) / 2) + 64))
#define NV12N_10B_CBCR_BASE(base, w, h)		\
	((base) + NV12N_10B_Y_8B_SIZE((w), (h)) + NV12N_10B_Y_2B_SIZE((w), (h)))

#define YUV420N_Y_SIZE(w, h)			\
	(__ALIGN_UP((w), 16) * __ALIGN_UP((h), 16) + 256)
#define YUV420N_CB_SIZE(w, h)			\
(__ALIGN_UP((__ALIGN_UP((w) / 2, 16) * (__ALIGN_UP((h), 16) / 2) + 256), 16))
#define YUV420N_CR_SIZE(w, h)			\
(__ALIGN_UP((__ALIGN_UP((w) / 2, 16) * (__ALIGN_UP((h), 16) / 2) + 256), 16))
#define YUV420N_CB_BASE(base, w, h)	((base) + YUV420N_Y_SIZE((w), (h)))
#define YUV420N_CR_BASE(base, w, h)		\
	(YUV420N_CB_BASE((base), (w), (h)) + YUV420N_CB_SIZE((w), (h)))

#define NV12M_Y_SIZE(w, h)			\
	(__ALIGN_UP((w), 64) * __ALIGN_UP((h), 16) + 256)
#define NV12M_CBCR_SIZE(w, h)			\
	((__ALIGN_UP((w), 64) * __ALIGN_UP((h), 16) / 2) + 256)
#define NV12M_Y_2B_SIZE(w, h)			\
	(__ALIGN_UP((w / 4), 16) * __ALIGN_UP((h), 16) + 256)
#define NV12M_CBCR_2B_SIZE(w, h)		\
	((__ALIGN_UP((w / 4), 16) * __ALIGN_UP((h), 16) / 2) + 256)

#define NV16M_Y_SIZE(w, h)			\
	(__ALIGN_UP((w), 64) * __ALIGN_UP((h), 16) + 256)
#define NV16M_CBCR_SIZE(w, h)			\
	(__ALIGN_UP((w), 64) * __ALIGN_UP((h), 16) + 256)
#define NV16M_Y_2B_SIZE(w, h)			\
	(__ALIGN_UP((w / 4), 16) * __ALIGN_UP((h), 16) + 256)
#define NV16M_CBCR_2B_SIZE(w, h)		\
	(__ALIGN_UP((w / 4), 16) * __ALIGN_UP((h), 16) + 256)

#define S10B_8B_STRIDE(w)		(__ALIGN_UP((w), 64))
#define S10B_2B_STRIDE(w)		(__ALIGN_UP(((w + 3) / 4), 16))

/* Compress format */

/* SBWC */
#define __COUNT_BLOCKS(x, a)		(((x) + ((a) - 1)) / (a))

#define SBWC_HEADER_STRIDE_ALIGN	16
#define SBWC_PAYLOAD_STRIDE_ALIGN	64

#define SBWC_BLOCK_WIDTH		32
#define SBWC_BLOCK_HEIGHT		4

#define SBWC_ALIGNED_H(h, a)		__ALIGN_UP((h), a)

#define SBWC_H_BLOCKS(w)		__COUNT_BLOCKS((w), SBWC_BLOCK_WIDTH)

#define SBWC_8B_STRIDE(w)		(__ALIGN_UP((8 / 2) *			\
						SBWC_BLOCK_WIDTH,		\
						SBWC_PAYLOAD_STRIDE_ALIGN) *	\
						SBWC_H_BLOCKS(w))
#define SBWC_10B_STRIDE(w)		(__ALIGN_UP((10 / 2) *			\
						SBWC_BLOCK_WIDTH,		\
						SBWC_PAYLOAD_STRIDE_ALIGN) *	\
						SBWC_H_BLOCKS(w))
#define SBWC_HEADER_STRIDE(w)		(__ALIGN_UP(__COUNT_BLOCKS(w,		\
						SBWC_BLOCK_WIDTH * 2),		\
						SBWC_HEADER_STRIDE_ALIGN))

#define SBWC_Y_VSTRIDE_BLOCKS(h, a)	__COUNT_BLOCKS(SBWC_ALIGNED_H(h, a),	\
						SBWC_BLOCK_HEIGHT)
#define SBWC_CBCR_VSTRIDE_BLOCKS(h, a)	__COUNT_BLOCKS(SBWC_ALIGNED_H(h, a) / 2,\
						SBWC_BLOCK_HEIGHT)

/* Height aligned to 16 for H.264 */
#define SBWC_8B_Y_SIZE(w, h)		((SBWC_8B_STRIDE(w) *			\
					  SBWC_Y_VSTRIDE_BLOCKS(h, 16)) + 64)
#define SBWC_8B_CBCR_SIZE(w, h)		((SBWC_8B_STRIDE(w) *			\
					  SBWC_CBCR_VSTRIDE_BLOCKS(h, 16)) + 64)

#define SBWC_8B_Y_HEADER_SIZE(w, h)	((SBWC_HEADER_STRIDE(w) *		\
					  SBWC_Y_VSTRIDE_BLOCKS(h, 16)) + 256)

#define SBWC_8B_CBCR_HEADER_SIZE(w, h)	((SBWC_HEADER_STRIDE(w) *		\
					  SBWC_CBCR_VSTRIDE_BLOCKS(h, 16)) + 128)

/* Height aligned to 8 for H.265 and VP9 */
#define SBWC_10B_Y_SIZE(w, h)		((SBWC_10B_STRIDE(w) *			\
					  SBWC_Y_VSTRIDE_BLOCKS(h, 8)) + 64)
#define SBWC_10B_CBCR_SIZE(w, h)	((SBWC_10B_STRIDE(w) *			\
					  SBWC_CBCR_VSTRIDE_BLOCKS(h, 8)) + 64)
#define SBWC_10B_Y_HEADER_SIZE(w, h)	((SBWC_HEADER_STRIDE(w) *		\
					  SBWC_Y_VSTRIDE_BLOCKS(h, 8)) + 256)
#define SBWC_10B_CBCR_HEADER_SIZE(w, h)	((SBWC_HEADER_STRIDE(w) *		\
					  SBWC_CBCR_VSTRIDE_BLOCKS(h, 8)) + 128)

/* SBWC - single fd */
#define SBWC_8B_CBCR_BASE(base, w, h)					\
	((base) + SBWC_8B_Y_SIZE(w, h) + SBWC_8B_Y_HEADER_SIZE(w, h))
#define SBWC_10B_CBCR_BASE(base, w, h)					\
	((base) + SBWC_10B_Y_SIZE(w, h) + SBWC_10B_Y_HEADER_SIZE(w, h))

/* SBWC Lossy */
#define SBWCL_8B_STRIDE(w, r)	(((128 * (r)) / 100) * (((w) + 31) / 32))
#define SBWCL_10B_STRIDE(w, r)	(((160 * (r)) / 100) * (((w) + 31) / 32))

#define SBWCL_8B_Y_SIZE(w, h, r)					\
	((SBWCL_8B_STRIDE(w, r) * ((__ALIGN_UP((h), 16) + 3) / 4)) + 64)
#define SBWCL_8B_CBCR_SIZE(w, h, r)					\
	((SBWCL_8B_STRIDE(w, r) * (((__ALIGN_UP((h), 16) / 2) + 3) / 4)) + 64)

#define SBWCL_10B_Y_SIZE(w, h, r)					\
	((SBWCL_10B_STRIDE(w, r) * ((__ALIGN_UP((h), 16) + 3) / 4)) + 64)
#define SBWCL_10B_CBCR_SIZE(w, h, r)					\
	((SBWCL_10B_STRIDE(w, r) * (((__ALIGN_UP((h), 16) / 2) + 3) / 4)) + 64)

#define SBWCL_8B_CBCR_BASE(base, w, h, r)				\
	((base) + SBWCL_8B_Y_SIZE(w, h, r))
#define SBWCL_10B_CBCR_BASE(base, w, h, r)				\
	((base) + SBWCL_10B_Y_SIZE(w, h, r))

/* AFBC */
#define AFBC_8B_STRIDE(w)		__ALIGN_UP(w, 16)
#define AFBC_10B_STRIDE(w)		__ALIGN_UP(w * 2, 16)

#define AFBC_8B_Y_SIZE(w, h)							\
	((((((w) + 31) / 32) * (((h) + 7) / 8) * 16 + 127) / 128) * 128 +	\
	(((w) + 31) / 32) * (((h) + 7) / 8) * 384)
#define AFBC_10B_Y_SIZE(w, h)							\
	((((((w) + 31) / 32) * (((h) + 7) / 8) * 16 + 127) / 128) * 128 +	\
	(((w) + 31) / 32) * (((h) + 7) / 8) * 512)
#endif /* __LINUX_VIDEODEV2_EXYNOS_MEDIA_H */
