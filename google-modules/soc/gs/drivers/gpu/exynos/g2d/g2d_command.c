// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 */

#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/device.h>

#include "g2d.h"
#include "g2d_task.h"
#include "g2d_uapi.h"
#include "g2d_command.h"
#include "g2d_regs.h"
#include "g2d_format.h"

static u32 layer_crop_width(struct g2d_layer *layer)
{
	return layer->commands[G2DSFR_IMG_RIGHT].value - layer->commands[G2DSFR_IMG_LEFT].value;
}

static u32 layer_crop_height(struct g2d_layer *layer)
{
	return layer->commands[G2DSFR_IMG_BOTTOM].value - layer->commands[G2DSFR_IMG_TOP].value;
}

static u32 layer_width(struct g2d_layer *layer)
{
	return layer->commands[G2DSFR_IMG_WIDTH].value;
}

static u32 layer_height(struct g2d_layer *layer)
{
	return layer->commands[G2DSFR_IMG_HEIGHT].value;
}

enum {
	TASK_REG_SOFT_RESET,
	TASK_REG_SECURE_MODE,
	TASK_REG_LAYER_UPDATE,

	TASK_REG_COUNT
};

/*
 * G2D_SECURE_LAYER_REG and G2D_LAYER_UPDATE_REG are updated in
 * g2d_prepare_source() and g2d_prepare_target().
 */
static struct g2d_reg g2d_setup_commands[TASK_REG_COUNT] = {
	{G2D_SOFT_RESET_REG,   0x00000004}, /* CoreSFRClear */
	{G2D_SECURE_MODE_REG,  0x00000000},
	{G2D_LAYER_UPDATE_REG, 0x00000000},
};

void g2d_init_commands(struct g2d_task *task)
{
	memcpy(page_address(task->cmd_page), &g2d_setup_commands, sizeof(g2d_setup_commands));
	task->sec.cmd_count = ARRAY_SIZE(g2d_setup_commands);
}

static void g2d_set_taskctl_commands(struct g2d_task *task)
{
	struct g2d_reg *regs = (struct g2d_reg *)page_address(task->cmd_page);
	struct g2d_layer *layer;
	u32 rot = 0;
	u32 n_rot = 0;
	u32 size = 0; /* Size doesn't cause overflow */
	u32 width = layer_width(&task->target);
	u32 height = layer_height(&task->target);
	int i;
	bool vertical;

	for (i = 0; i < task->num_source; i++) {
		layer = &task->source[i];
		size = layer_crop_width(layer) * layer_crop_height(layer);

		if (layer->commands[G2DSFR_SRC_ROTATE].value & 1)
			rot += size;
		else
			n_rot += size;
	}

	vertical = (rot > n_rot) ? true : false;

	if (vertical) {
		u32 mode = task->target.commands[G2DSFR_IMG_COLORMODE].value;

		regs[task->sec.cmd_count].offset = G2D_TILE_DIRECTION_ORDER_REG;
		regs[task->sec.cmd_count].value = G2D_TILE_DIRECTION_VERTICAL;

		if (!IS_HWFC(task->flags) &&
		    (IS_YUV420(mode) || IS_YUV422_2P(mode)))
			regs[task->sec.cmd_count].value |=
					G2D_TILE_DIRECTION_ZORDER;

		task->sec.cmd_count++;
	}

	/*
	 * Divide the entire destination in half by verital or horizontal,
	 * and let the H/W work in parallel.
	 * split index is half the width or height divided by 16
	 */
	regs[task->sec.cmd_count].offset = G2D_DST_SPLIT_TILE_IDX_REG;
	if (vertical && !IS_HWFC(task->flags) && height > width)
		regs[task->sec.cmd_count].value = ((height / 2) >> 4);
	else
		regs[task->sec.cmd_count].value =
			((width / 2) >> 4) | G2D_DST_SPLIT_TILE_IDX_VFLAG;

	task->sec.cmd_count++;
}

static void g2d_set_hwfc_commands(struct g2d_task *task)
{
	struct g2d_reg *regs = (struct g2d_reg *)page_address(task->cmd_page);

	regs[task->sec.cmd_count].offset = G2D_HWFC_CAPTURE_IDX_REG;
	regs[task->sec.cmd_count].value = IS_HWFC(task->flags) ?
			G2D_HWFC_CAPTURE_HWFC_JOB : 0;
	regs[task->sec.cmd_count].value |= g2d_task_id(task);

	task->sec.cmd_count++;
}

static void g2d_set_start_commands(struct g2d_task *task)
{
	bool self_prot = task->g2d_dev->caps & G2D_DEVICE_CAPS_SELF_PROTECTION;
	struct g2d_reg *regs = page_address(task->cmd_page);

	if (!self_prot && IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION))
		return;

	/*
	 * Number of commands should be multiple of 8.
	 * If it is not, then pad dummy commands with no side effect.
	 */
	while ((task->sec.cmd_count & 7) != 0) {
		regs[task->sec.cmd_count].offset = G2D_LAYER_UPDATE_REG;
		regs[task->sec.cmd_count].value =
					regs[TASK_REG_LAYER_UPDATE].value;
		task->sec.cmd_count++;
	}
}

void g2d_complete_commands(struct g2d_task *task)
{
	g2d_set_taskctl_commands(task);

	g2d_set_hwfc_commands(task);

	g2d_set_start_commands(task);

	BUG_ON(task->sec.cmd_count > G2D_MAX_COMMAND);
}

static const struct g2d_fmt g2d_formats_common[] = {
	{
		.name		= "ARGB8888",
		.fmtvalue	= G2D_FMT_ARGB8888,	/* [31:0] ARGB */
		.bpp		= { 32 },
		.num_planes	= 1,
	}, {
		.name		= "ABGR8888",
		.fmtvalue	= G2D_FMT_ABGR8888,	/* [31:0] ABGR */
		.bpp		= { 32 },
		.num_planes	= 1,
	}, {
		.name		= "XBGR8888",
		.fmtvalue	= G2D_FMT_XBGR8888,	/* [31:0] XBGR */
		.bpp		= { 32 },
		.num_planes	= 1,
	}, {
		.name		= "XRGB8888",
		.fmtvalue	= G2D_FMT_XRGB8888,	/* [31:0] XBGR */
		.bpp		= { 32 },
		.num_planes	= 1,
	}, {
		.name		= "RGB888",
		.fmtvalue	= G2D_FMT_RGB888,	/* [23:0] RGB */
		.bpp		= { 24 },
		.num_planes	= 1,
	}, {
		.name		= "ARGB4444",
		.fmtvalue	= G2D_FMT_ARGB4444,	/* [15:0] ARGB */
		.bpp		= { 16 },
		.num_planes	= 1,
	}, {
		.name		= "ARGB1555",
		.fmtvalue	= G2D_FMT_ARGB1555,	/* [15:0] ARGB */
		.bpp		= { 16 },
		.num_planes	= 1,
	}, {
		.name		= "RGB565",
		.fmtvalue	= G2D_FMT_RGB565,	/* [15:0] RGB */
		.bpp		= { 16 },
		.num_planes	= 1,
	}, {
		.name		= "BGR565",
		.fmtvalue	= G2D_FMT_BGR565,	/* [15:0] BGR */
		.bpp		= { 16 },
		.num_planes	= 1,
	}, {
		.name		= "NV12",
		.fmtvalue	= G2D_FMT_NV12,
		.bpp		= { 8, 4 },
		.num_planes	= 2,
	}, {
		.name		= "NV21",
		.fmtvalue	= G2D_FMT_NV21,
		.bpp		= { 8, 4 },
		.num_planes	= 2,
	}, {
		.name		= "YUYV",
		.fmtvalue	= G2D_FMT_YUYV,
		.bpp		= { 16 },
		.num_planes	= 1,
	}, {
		.name		= "YVYU",
		.fmtvalue	= G2D_FMT_YVYU,
		.bpp		= { 16 },
		.num_planes	= 1,
	}, {
		.name		= "UYVY",
		.fmtvalue	= G2D_FMT_UYVY,
		.bpp		= { 16 },
		.num_planes	= 1,
	}, {
		.name		= "VYUY",
		.fmtvalue	= G2D_FMT_VYUY,
		.bpp		= { 16 },
		.num_planes	= 1,
	}, {
		.name		= "NV16",
		.fmtvalue	= G2D_FMT_NV16,
		.bpp		= { 8, 8 },
		.num_planes	= 2,
	}, {
		.name		= "NV61",
		.fmtvalue	= G2D_FMT_NV61,
		.bpp		= { 8, 8 },
		.num_planes	= 2,
	}, {
		.name		= "ABGR2101010",
		.fmtvalue	= G2D_FMT_ABGR2101010,
		.bpp		= { 32 },
		.num_planes	= 1,
	}, {
		.name		= "ABGR1010102",
		.fmtvalue	= G2D_FMT_ABGR1010102,
		.bpp		= { 32 },
		.num_planes	= 1,
	}, {
		.name		= "XBGR2101010",
		.fmtvalue	= G2D_FMT_XBGR2101010,
		.bpp		= { 32 },
		.num_planes	= 1,
	}, {
		.name		= "YUV420P",
		.fmtvalue	= G2D_FMT_YUV420P,
		.bpp		= { 8, 2, 2},
		.num_planes	= 3,
	}, {
		.name		= "YV12",
		.fmtvalue	= G2D_FMT_YV12,
		.bpp		= { 8, 2, 2},
		.num_planes	= 3,
	},
};

static const struct g2d_fmt g2d_formats_9810[] = {
	 {
		.name		= "NV12_P010",
		.fmtvalue	= G2D_FMT_NV12_P010_9810,
		.bpp		= { 16, 8},
		.num_planes	= 2,
	}, {
		.name		= "NV21_P010",
		.fmtvalue	= G2D_FMT_NV21_P010_9810,
		.bpp		= { 16, 8},
		.num_planes	= 2,
	},
};

static const struct g2d_fmt g2d_formats_9820[] = {
	{
		.name		= "NV12_P010",
		.fmtvalue	= G2D_FMT_NV12_P010_9820,
		.bpp		= { 16, 8},
		.num_planes	= 2,
	}, {
		.name		= "NV21_P010",
		.fmtvalue	= G2D_FMT_NV21_P010_9820,
		.bpp		= { 16, 8},
		.num_planes	= 2,
	}, {
		.name		= "NV16_P210",
		.fmtvalue	= G2D_FMT_NV16_P210_9820,
		.bpp		= { 16, 16},
		.num_planes	= 2,
	}, {
		.name		= "NV61_P210",
		.fmtvalue	= G2D_FMT_NV61_P210_9820,
		.bpp		= { 16, 16},
		.num_planes	= 2,
	},
};

const struct g2d_fmt *g2d_find_format(u32 fmtval, unsigned long devcaps)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(g2d_formats_common); i++)
		if (g2d_formats_common[i].fmtvalue == G2D_IMGFMT(fmtval))
			return &g2d_formats_common[i];

	if (!(devcaps & G2D_DEVICE_CAPS_YUV_BITDEPTH)) {
		for (i = 0; i < ARRAY_SIZE(g2d_formats_9810); i++)
			if (g2d_formats_9810[i].fmtvalue == G2D_IMGFMT(fmtval))
				return &g2d_formats_9810[i];
	} else {
		for (i = 0; i < ARRAY_SIZE(g2d_formats_9820); i++)
			if (g2d_formats_9820[i].fmtvalue == G2D_IMGFMT(fmtval))
				return &g2d_formats_9820[i];
	}

	WARN(1, "Unknown format %#x with device caps %#lx", fmtval, devcaps);
	return NULL;
}

/*
 * Buffer stride alignment and padding restriction of MFC
 * YCbCr420 semi-planar 8+2 layout:
 *    Y8 -> Y2 -> C8 -> C2
 * 8 bit segments:
 *  - width stride: 64 bytes
 *  - height stride: 16 pixels
 * 2 bit segments:
 *  - width stride: 16 bytes
 *  - height stride: 16 pixels
 *  - padding: 64 bytes
 */
#ifndef __ALIGN_UP
#define __ALIGN_UP(x, a)		(((x) + ((a) - 1)) & ~((a) - 1))
#endif

#define NV12_MFC_Y_PAYLOAD(w, h)	(__ALIGN_UP(w, 64) * __ALIGN_UP(h, 16))
#define NV12_MFC_Y_PAYLOAD_PAD(w, h)	(NV12_MFC_Y_PAYLOAD(w, h))
#define NV12_MFC_C_PAYLOAD(w, h)	(__ALIGN_UP((w), 64) * __ALIGN_UP((h), 16) / 2)
#define NV12_MFC_CBASE(base, w, h)	((base) + NV12_MFC_Y_PAYLOAD_PAD(w, h))

static size_t nv12_mfc_payload(u32 w, u32 h)
{
	return NV12_MFC_Y_PAYLOAD_PAD(w, h) + NV12_MFC_C_PAYLOAD(w, h);
}

static unsigned char src_base_reg_offset[3] = {0x1C, 0x80, 0x84};
static unsigned char dst_base_reg_offset[3] = {0x00, 0x50, 0x54};

static unsigned char src_afbc_reg_offset[2] = {0x64, 0x68};
static unsigned char dst_afbc_reg_offset[2] = {0x30, 0x34};

/* {PAYLOAD_BASE, HEADER_BASE, PLANE3_BASE(C_PAYLOAD), PLANE2_BASE(C_HEADER)} */
static unsigned char src_sbwc_reg_offset[4] = {0x68, 0x64, 0x84, 0x80};
static unsigned char dst_sbwc_reg_offset[4] = {0x34, 0x30, 0x54, 0x50};

#define BASE_REG_OFFSET(base, offsets, buf_idx)	((base) + (offsets)[(buf_idx)])

#define REG_SRC_CHROMA_STRIDE 0xAC
#define REG_DST_CHROMA_STRIDE 0x60

#define AFBCV1_ALIGN_LEN_SRC (64 / 4)
#define AFBCV1_ALIGN_LEN 64
#define AFBCV12_ALIGN_LEN 128

#define AFBC_HEADER_BLOCK_LEN 16
/* SQBLOCK: 16x16 Sqaure superblock */
#define AFBC_SQBLOCK_WIDTH 16
#define AFBC_SQBLOCK_HEIGHT 16
/* LSBLOCK: 32x8 Landscape superblock */
#define AFBC_LSBLOCK_WIDTH 32
#define AFBC_LSBLOCK_HEIGHT 8

static u32 get_afbc_width_align(int layer_flags)
{
	bool landscape = !!(layer_flags & G2D_LAYERFLAG_AFBC_LANDSCAPE);

	return landscape ? AFBC_LSBLOCK_WIDTH : AFBC_SQBLOCK_WIDTH;
}

static u32 get_afbc_height_align(int layer_flags)
{
	bool landscape = !!(layer_flags & G2D_LAYERFLAG_AFBC_LANDSCAPE);

	return landscape ? AFBC_LSBLOCK_HEIGHT : AFBC_SQBLOCK_HEIGHT;
}

static inline bool caps_has_afbcv12(unsigned long caps)
{
	return !!(caps & G2D_DEVICE_CAPS_AFBC_V12);
}

static inline bool caps_has_align64(unsigned long caps)
{
	return !!(caps & G2D_DEVICE_CAPS_COMP_ALIGN_64);
}

static inline uint32_t afbc_header_len(unsigned long caps,
				       u32 nr_h_blocks, u32 nr_v_blocks)
{
	unsigned int align = AFBCV12_ALIGN_LEN;
	u32 len = nr_h_blocks * nr_v_blocks * AFBC_HEADER_BLOCK_LEN;
	/*
	 * AFBC v1.x requires alignment of 16-byte for both of the header and
	 * the payload address. It is safe to make the address of payload
	 * aligned by AFBC_ALIGN_LEN(128) because the AFBC decoders can find the
	 * offset of the payload from the header but we need to use 16-byte
	 * alignment when we check the buffer length for the compatibility with
	 * the legacy applications that work with AFBC 1.x
	 */
	if (!caps_has_afbcv12(caps))
		align = AFBCV1_ALIGN_LEN;
	return ALIGN(len, align);
}

static size_t afbc_payload_len(u32 width, u32 height, const struct g2d_fmt *fmt)
{
	size_t bpp = 0;
	unsigned int i;

	for (i = 0; i < fmt->num_planes; i++)
		bpp += fmt->bpp[i];

	if (fmt->fmtvalue & G2D_FMT_10) { // 10bit
		/*
		 * bitdepth treated by AFBC payload is not the same as the
		 * bitdepth of P010(24) and P210(32).
		 * Rather the required bitdepth values seem effective bitdepth:
		 * P010 - 15(round up to 16), P210 - 20
		 */
		bpp = ALIGN(bpp * 10 / 16, 4);
	}

	return width * height * bpp / BITS_PER_BYTE;
}

static size_t afbc_buffer_len(unsigned long caps, int layer_flags,
			      struct g2d_reg cmd[], const struct g2d_fmt *fmt)
{
	u32 walign = get_afbc_width_align(layer_flags);
	u32 halign = get_afbc_height_align(layer_flags);
	u32 width = ALIGN(cmd[G2DSFR_IMG_WIDTH].value, walign);
	u32 height = ALIGN(cmd[G2DSFR_IMG_HEIGHT].value, halign);
	size_t len = afbc_header_len(caps, width / walign, height / halign);

	return len + afbc_payload_len(width, height, fmt);
}

static u32 afbc_header_len_only(unsigned long caps, int layer_flags,
				struct g2d_reg cmd[])
{
	u32 walign = get_afbc_width_align(layer_flags);
	u32 halign = get_afbc_height_align(layer_flags);
	u32 width = ALIGN(cmd[G2DSFR_IMG_WIDTH].value, walign);
	u32 height = ALIGN(cmd[G2DSFR_IMG_HEIGHT].value, halign);

	return afbc_header_len(caps, width / walign, height / halign);
}

static inline bool is_afbc_aligned(u32 width, u32 height, bool dst,
				   int layer_flags, unsigned long caps)
{
	u32 walign = get_afbc_width_align(layer_flags);
	u32 halign = get_afbc_height_align(layer_flags);
	/*
	 * G2D with AFBCv1 allows multiple of 4 for the source image widths.
	 * Also, 32x8 superblock is supported from AFBC v1.2.
	 */
	if (!dst && !caps_has_afbcv12(caps))
		halign = 4;
	if (!IS_ALIGNED(width, walign))
		return false;
	if (!IS_ALIGNED(height, halign))
		return false;
	return true;
}

#define SBWC_BLOCK_WIDTH 32
#define SBWC_BLOCK_HEIGHT 4

#define SBWC_HEADER_ALIGN 16
#define SBWC_PAYLOAD_ALIGN 32

#define IS_SBWCL(sbwc) ((sbwc) & 1)
#define SBWCL_BLOCK_SIZE(sbwc) ((((sbwc) >> 4) & 0xF) << 5)

static u32 get_sbwc_stride(unsigned long caps, u32 stride, u32 align)
{
	if (caps_has_align64(caps))
		align = max(64U, align);

	return ALIGN(stride, align);
}

static u32 get_sbwc_stride_header(unsigned long caps, u32 width)
{
	/* length of a header is 2 bytes per a block */
	u32 stride = width / SBWC_BLOCK_WIDTH / 2;

	return get_sbwc_stride(caps, stride, SBWC_HEADER_ALIGN);
}

static u32 get_sbwc_stride_payload(unsigned long caps, u32 width, u32 bitdepth)
{
	/* width is aligned by SBWC_BLOCK_WIDTH */
	u32 stride = width * SBWC_BLOCK_HEIGHT * bitdepth / BITS_PER_BYTE;

	return get_sbwc_stride(caps, stride, SBWC_PAYLOAD_ALIGN);
}

static u32 get_sbwc_lossy_stride_payload(u32 width, u32 blocksize)
{
	u32 stride = (width / SBWC_BLOCK_WIDTH) * blocksize;

	return ALIGN(stride, SBWC_PAYLOAD_ALIGN);
}

static u32 get_sbwc_lossy_payload_y_basis(u32 width,
		u32 height, u32 blocksize)
{
	u32 size = get_sbwc_lossy_stride_payload(width, blocksize);

	return size * height / SBWC_BLOCK_HEIGHT;
}

static u32 get_sbwc_lossy_payload_c_basis(u32 colormode,
		u32 width, u32 height, u32 blocksize)
{
	u32 size = get_sbwc_lossy_stride_payload(width, blocksize);

	if (IS_YUV420(colormode))
		height = ALIGN(height, 8);
	return size * height / SBWC_BLOCK_HEIGHT;
}

static u32 get_sbwc_lossy_y_size(u32 width, u32 height, u32 blocksize)
{
	 return get_sbwc_lossy_payload_y_basis(width, height, blocksize);
}

static u32 get_sbwc_lossy_c_size(u32 colormode,
		u32 width, u32 height, u32 blocksize)
{
	u32 size = get_sbwc_lossy_payload_c_basis(colormode, width, height, blocksize);

	if (IS_YUV420(colormode))
		size = size / 2;
	return size;
}


/*
 * Buffer stride alignment and padding restriction of MFC
 * YCbCr semi-planar SBWC layout:
 *    Y payload -> Y header -> CbCr payload -> CbCr header
 *
 * payload segments:
 *  - padding : 64 bytes
 *  - height : 8 pixel
 * header segments:
 *  - padding : 256 bytes
 *  - height : 8 pixel
 */
#define MFC_SBWC_PAYLOAD_PAD	64
#define MFC_SBWC_HEADER_PAD	256
#define MFC_SBWC_HEIGHT_ALIGN	8

static u32 get_sbwc_payload_basis(unsigned long caps, u32 colormode,
				  u32 width, u32 height, u32 bitdepth)
{
	u32 size = get_sbwc_stride_payload(caps, width, bitdepth);

	if (IS_YUV420(colormode))
		height = ALIGN(height, MFC_SBWC_HEIGHT_ALIGN);

	return size * height / SBWC_BLOCK_HEIGHT;
}

static u32 get_sbwc_payload_y_size(unsigned long caps, u32 colormode,
				   u32 width, u32 height, u32 bitdepth)
{
	u32 size = get_sbwc_payload_basis(caps, colormode,
					  width, height, bitdepth);

	if (IS_YUV420(colormode))
		size += MFC_SBWC_PAYLOAD_PAD;

	return size;
}

static u32 get_sbwc_payload_c_size(unsigned long caps, u32 colormode,
				   u32 width, u32 height, u32 bitdepth)
{
	u32 size = get_sbwc_payload_basis(caps, colormode,
					  width, height, bitdepth);
	/* chroma of ycbcr420 is vertically subsampled */
	if (IS_YUV420(colormode))
		size = (size / 2) + MFC_SBWC_PAYLOAD_PAD;

	return size;
}

static u32 get_sbwc_header_size(unsigned long caps, u32 colormode,
				u32 width, u32 height)
{
	u32 size = get_sbwc_stride_header(caps, width);

	if (IS_YUV420(colormode))
		height = ALIGN(height, MFC_SBWC_HEIGHT_ALIGN);

	return size * height / SBWC_BLOCK_HEIGHT;
}

static u32 get_sbwc_header_y_size(unsigned long caps, u32 colormode,
				  u32 width, u32 height)
{
	u32 size = get_sbwc_header_size(caps, colormode, width, height);

	if (IS_YUV420(colormode))
		size += MFC_SBWC_HEADER_PAD;

	return size;
}

static u32 get_sbwc_header_c_size(unsigned long caps, u32 colormode,
				  u32 width, u32 height)
{
	u32 size = get_sbwc_header_size(caps, colormode, width, height);

	if (IS_YUV420(colormode))
		size = (size / 2) + (MFC_SBWC_HEADER_PAD / 2);

	return size;
}

static u32 get_sbwc_y_size(unsigned long caps, u32 colormode,
			   u32 width, u32 height, u32 bitdepth)
{
	return get_sbwc_payload_y_size(caps, colormode, width, height, bitdepth)
	       + get_sbwc_header_y_size(caps, colormode, width, height);
}

static u32 get_sbwc_c_size(unsigned long caps, u32 colormode,
			   u32 width, u32 height, u32 bitdepth)
{
	return get_sbwc_payload_c_size(caps, colormode, width, height, bitdepth)
	       + get_sbwc_header_c_size(caps, colormode, width, height);
}

unsigned int g2d_get_payload_index(struct g2d_reg cmd[], const struct g2d_fmt *fmt,
				   unsigned int idx, unsigned int buffer_count,
				   unsigned long caps, u32 flags, bool dst)
{
	u32 width = cmd[G2DSFR_IMG_WIDTH].value;
	u32 height = cmd[G2DSFR_IMG_BOTTOM].value;
	unsigned int colormode = cmd[G2DSFR_IMG_COLORMODE].value;

	BUG_ON(!IS_YUV(cmd[G2DSFR_IMG_COLORMODE].value));

	if (IS_SBWC(colormode)) {
		unsigned int dep = IS_YUV_P10(colormode,
				caps & G2D_DEVICE_CAPS_YUV_BITDEPTH) ? 10 : 8;
		u32 sbwc = dst ? cmd[G2DSFR_DST_SBWCINFO].value :
			cmd[G2DSFR_SRC_SBWCINFO].value;
		unsigned int blocksize = SBWCL_BLOCK_SIZE(sbwc);

		if (IS_SBWCL(sbwc)) {
			if (idx == 0)
				return get_sbwc_lossy_y_size(width, height, blocksize);
			return get_sbwc_lossy_c_size(colormode, width,
					height, blocksize);
		}
		if (idx == 0)
			return get_sbwc_y_size(caps, colormode,
					       width, height, dep);
		return get_sbwc_c_size(caps, colormode, width, height, dep);
	}

	if (!IS_YUV420P(fmt->fmtvalue) && idx > 0)
		return ALIGN(ALIGN(width / 2, 16) * height / 2, 16);

	return ((cmd[G2DSFR_IMG_WIDTH].value * fmt->bpp[idx]) / 8) * cmd[G2DSFR_IMG_BOTTOM].value;
}

static size_t g2d_get_ycbcr_payload(const struct g2d_fmt *fmt, u32 flags,
				    u32 mode, u32 width, u32 height)
{
	bool mfc_stride = flags & G2D_LAYERFLAG_MFC_STRIDE;
	size_t pixcount = width * height;
	size_t payload = 0;

	if (mfc_stride && IS_YUV420(mode)) {
		payload = nv12_mfc_payload(width, height);
	} else if (IS_YUV420P(fmt->fmtvalue)) {
		u32 stride = ALIGN(width / 2, 16);

		payload = (pixcount * fmt->bpp[0]) / 8;
		payload += ALIGN(stride * height / 2, 16) * 2;
	} else {
		unsigned int i;

		payload = (pixcount * fmt->bpp[0]) / 8;
		for (i = 1; i < fmt->num_planes; i++) {
			/* base address needs to be 16-byte aligned */
			payload = ALIGN(payload, 16);
			payload += (pixcount * fmt->bpp[i]) / 8;
		}
	}

	return payload;
}

size_t g2d_get_payload(struct g2d_reg cmd[], const struct g2d_fmt *fmt,
		       u32 flags, unsigned long cap, bool dst)
{
	size_t payload = 0;
	u32 mode = cmd[G2DSFR_IMG_COLORMODE].value;
	u32 width = cmd[G2DSFR_IMG_WIDTH].value;
	u32 height = cmd[G2DSFR_IMG_BOTTOM].value;

	if (IS_SBWC(mode)) {
		unsigned int dep = IS_YUV_P10(mode,
				cap & G2D_DEVICE_CAPS_YUV_BITDEPTH) ? 10 : 8;
		u32 sbwc = dst ? cmd[G2DSFR_DST_SBWCINFO].value :
			cmd[G2DSFR_SRC_SBWCINFO].value;
		unsigned int blocksize = SBWCL_BLOCK_SIZE(sbwc);

		if (IS_SBWCL(sbwc)) {
			return get_sbwc_lossy_y_size(width, height, blocksize) +
				get_sbwc_lossy_c_size(mode, width,
						 height, blocksize);
		}
		return get_sbwc_y_size(cap, mode, width, height, dep) +
			get_sbwc_c_size(cap, mode, width, height, dep);
	} else if (IS_AFBC(mode)) {
		payload = afbc_buffer_len(cap, flags, cmd, fmt);
	} else if (IS_YUV(mode)) {
		payload = g2d_get_ycbcr_payload(fmt, flags,
						mode, width, height);
	} else {
		payload = cmd[G2DSFR_IMG_STRIDE].value *
				cmd[G2DSFR_IMG_BOTTOM].value;
	}

	return payload;
}

static bool check_width_height(struct g2d_device *g2d_dev, u32 value)
{
	return (value > 0) && (value <= G2D_MAX_SIZE);
}

#define G2D_SRC_FMTS ((BIT(G2D_FMT_IDX_MAX) - 1) & ~(\
			BIT(G2D_FMT_IDX_RESERVED) | BIT(G2D_FMT_IDX_8) |\
			BIT(G2D_FMT_IDX_YUV420P) | BIT(G2D_FMT_IDX_1010102)))
#define G2D_DST_FMTS ((BIT(G2D_FMT_IDX_MAX) - 1) & ~(\
			BIT(G2D_FMT_IDX_RESERVED) | BIT(G2D_FMT_IDX_8) |\
			BIT(G2D_FMT_IDX_1010102)))
#define G2D_RGB_FMTS (BIT(G2D_FMT_IDX_MAX_RGB) - 1)

#define G2D_AFBCV1_FMTS (BIT(G2D_FMT_IDX_8888) | BIT(G2D_FMT_IDX_565) |\
			 BIT(G2D_FMT_IDX_4444) | BIT(G2D_FMT_IDX_1555))
#define G2D_SBWC_FMTS (BIT(G2D_FMT_IDX_YUV420SP) | BIT(G2D_FMT_IDX_YUV422SP))
#define G2D_AFBCV1_2_FMTS (G2D_AFBCV1_FMTS | G2D_SBWC_FMTS |\
			   BIT(G2D_FMT_IDX_2101010))

/* 8bpp(grayscale) format is not supported */
static bool check_srccolor_mode(struct g2d_device *g2d_dev, u32 value)
{
	u32 fmt = (value & G2D_DATAFMT_MASK) >> G2D_DATAFMT_SHIFT;
	u32 available;

	if (IS_AFBC(value) && IS_SBWC(value))
		return false;

	if (IS_AFBC(value))
		available = caps_has_afbcv12(g2d_dev->caps) ? G2D_AFBCV1_2_FMTS
							    : G2D_AFBCV1_FMTS;
	else if (IS_SBWC(value))
		available = G2D_SBWC_FMTS;
	else
		available = g2d_dev->fmts_src ? g2d_dev->fmts_src
					      : G2D_SRC_FMTS;

	return !!(BIT(fmt) & available);
}

static bool check_dstcolor_mode(struct g2d_device *g2d_dev, u32 value)
{
	u32 fmt = (value & G2D_DATAFMT_MASK) >> G2D_DATAFMT_SHIFT;
	u32 mode = value & (G2D_DATAFORMAT_AFBC | G2D_DATAFORMAT_UORDER |
			    G2D_DATAFORMAT_SBWC);
	u32 available;

	/* SBWC, AFBC and UORDER should not be set together */
	if (mode & (mode - 1))
		return false;

	if (IS_AFBC(value))
		available = caps_has_afbcv12(g2d_dev->caps) ? G2D_AFBCV1_2_FMTS
							    : G2D_AFBCV1_FMTS;
	else if (IS_SBWC(value))
		available = G2D_SBWC_FMTS;
	else if (IS_UORDER(value))
		available = G2D_RGB_FMTS;
	else
		available = g2d_dev->fmts_dst ? g2d_dev->fmts_dst
					      : G2D_DST_FMTS;

	return !!(BIT(fmt) & available);
}

static bool check_blend_mode(struct g2d_device *g2d_dev, u32 value)
{
	int i = 0;

	for (i = 0; i < 2; i++) { /* for each source and destination */
		if ((value & 0xF) > 7) /* Coeff */
			return false;
		value >>= 4;
		if ((value & 0x3) > 2) /* CoeffSA */
			return false;
		value >>= 2;
		if ((value & 0x3) > 2) /* CoeffDA */
			return false;
		value >>= 2;
	}

	return true;
}

static bool check_scale_control(struct g2d_device *g2d_dev, u32 value)
{
	if (!(g2d_dev->caps & G2D_DEVICE_CAPS_POLYFILTER))
		return ((value >> 4) == 0) && (value != 3);
	return !value || ((value & 3) == 3);
}

struct command_checker {
	const char *cmdname;
	u32 offset;
	u32 mask;
	bool (*checker)(struct g2d_device *g2d_dev, u32 value);
};

static struct command_checker source_command_checker[G2DSFR_SRC_FIELD_COUNT] = {
	{"STRIDE",	0x0020, 0x0001FFFF, NULL,},
	{"COLORMODE",	0x0028, 0x335FFFFF, check_srccolor_mode,},
	{"LEFT",	0x002C, 0x00001FFF, NULL,},
	{"TOP",		0x0030, 0x00001FFF, NULL,},
	{"RIGHT",	0x0034, 0x00003FFF, check_width_height,},
	{"BOTTOM",	0x0038, 0x00003FFF, check_width_height,},
	{"WIDTH",	0x0070, 0x00003FFF, check_width_height,},
	{"HEIGHT",	0x0074, 0x00003FFF, check_width_height,},
	{"COMMAND",	0x0000, 0x03100003, NULL,},
	{"SELECT",	0x0004, 0x00000001, NULL,},
	{"ROTATE",	0x0008, 0x00000031, NULL,},
	{"DSTLEFT",	0x000C, 0x00001FFF, NULL,},
	{"DSTTOP",	0x0010, 0x00001FFF, NULL,},
	{"DSTRIGHT",	0x0014, 0x00003FFF, check_width_height,},
	{"DSTBOTTOM",	0x0018, 0x00003FFF, check_width_height,},
	{"SCALECONTROL", 0x048, 0x00000033, check_scale_control,},
	{"XSCALE",	0x004C, 0x3FFFFFFF, NULL,},
	{"YSCALE",	0x0050, 0x3FFFFFFF, NULL,},
	{"XPHASE",	0x0054, 0x0000FFFF, NULL,},
	{"YPHASE",	0x0058, 0x0000FFFF, NULL,},
	{"COLOR",	0x005C, 0xFFFFFFFF, NULL,},
	{"ALPHA",	0x0060, 0xFFFFFFFF, NULL,},
	{"BLEND",	0x003C, 0x0035FFFF, check_blend_mode,},
	{"YCBCRMODE",	0x0088, 0x10000017, NULL,},
	{"HDRMODE",	0x0090, 0x000011B3, NULL,},
	{"YHEADERSTRIDE",  0x00A0, 0x00003FFF, NULL,},
	{"YPAYLOADSTRIDE", 0x00A4, 0x0000FFFF, NULL,},
	{"CHEADERSTRIDE",  0x00A8, 0x00003FFF, NULL,},
	{"CPAYLOADSTRIDE", 0x00AC, 0x0000FFFF, NULL,},
	{"SBWCINFO",    0x00B0, 0x000100F1, NULL,},
};

static struct command_checker target_command_checker[G2DSFR_DST_FIELD_COUNT] = {
	/* BASE OFFSET: 0x0120 */
	{"STRIDE",	0x0004, 0x0001FFFF, NULL,},
	{"COLORMODE",	0x000C, 0x337FFFFF, check_dstcolor_mode,},
	{"LEFT",	0x0010, 0x00001FFF, NULL,},
	{"TOP",		0x0014, 0x00001FFF, NULL,},
	{"RIGHT",	0x0018, 0x00003FFF, check_width_height,},
	{"BOTTOM",	0x001C, 0x00003FFF, check_width_height,},
	{"WIDTH",	0x0040, 0x00003FFF, check_width_height,},
	{"HEIGHT",	0x0044, 0x00003FFF, check_width_height,},
	/* TODO: check csc */
	{"YCBCRMODE",	0x0058, 0x0000F714, NULL,},
	{"YHEADERSTRIDE",  0x0070, 0x00003FFF, NULL,},
	{"YPAYLOADSTRIDE", 0x0074, 0x0000FFFF, NULL,},
	{"CHEADERSTRIDE",  0x0078, 0x00003FFF, NULL,},
	{"CPAYLOADSTRIDE", 0x007C, 0x0000FFFF, NULL,},
	{"SBWCINFO",    0x0090, 0x000100F1, NULL,},
};

#define TARGET_OFFSET		0x120
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
#define LAYER_OFFSET(idx)	((2 + (idx)) << 8)
#define LAYER_UPDATE_INDEX	1
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#define LAYER_OFFSET(idx)       ((3 + (idx)) << 8)
#define LAYER_UPDATE_INDEX	2
#endif

static int g2d_copy_commands(struct g2d_device *g2d_dev, int index,
			     struct g2d_reg regs[], __u32 cmd[],
			     struct command_checker checker[],
			     unsigned int num_cmds)
{
	unsigned int base = (index < 0) ? TARGET_OFFSET : LAYER_OFFSET(index);
	int i;

	for (i = 0; i < num_cmds; i++) {
		if (((cmd[i] & ~checker[i].mask) != 0) ||
		    (checker[i].checker && !checker[i].checker(g2d_dev, cmd[i]))) {
			perrfndev(g2d_dev, "Invalid %s[%d] SFR '%s' value %#x",
				  (index < 0) ? "target" : "source",
				  index, checker[i].cmdname, cmd[i]);
			perrdev(g2d_dev, "mask %#x", checker[i].mask);
			return -EINVAL;
		}

		regs[i].offset = base + checker[i].offset;
		regs[i].value = cmd[i];
	}

	return num_cmds;
}

static bool g2d_validate_image_dimension(struct g2d_device *g2d_dev,
					 u32 width, u32 height,
					 u32 left, u32 top,
					 u32 right, u32 bottom)
{
	if (left >= right || top >= bottom ||
	    width < (right - left) || height < (bottom - top)) {
		perrfndev(g2d_dev, "Invalid dimension [%ux%u, %ux%u) / %ux%u",
			  left, top, right, bottom, width, height);
		return false;
	}

	return true;
}

#define IS_EVEN(value) (((value) & 1) == 0)

static bool g2d_global_clip_okay(unsigned long caps, struct g2d_task *task)
{
	struct g2d_reg *l0 = task->source[0].commands;
	struct g2d_reg *dst = task->target.commands;

	return (l0[G2DSFR_SRC_DSTLEFT].value == 0) &&
	       (l0[G2DSFR_SRC_DSTTOP].value == 0) &&
	       (l0[G2DSFR_SRC_DSTRIGHT].value == dst[G2DSFR_IMG_WIDTH].value) &&
	       (l0[G2DSFR_SRC_DSTBOTTOM].value == dst[G2DSFR_IMG_HEIGHT].value);
}

/*
 * Offsets of G2DSFR_IMG_RIGHT/TOP/BOTTOM from G2DSFR_IMG_LEFT are the same with
 * the offsets from G2DSFR_SRC_DSTRIGHT/TOP/BOTTOM from G2DSFR_SRC_DSTLEFT,
 * respectively.
 */
enum {
	G2D_CLIP_LEFT = G2DSFR_IMG_LEFT - G2DSFR_IMG_LEFT,
	G2D_CLIP_RIGHT = G2DSFR_IMG_RIGHT - G2DSFR_IMG_LEFT,
	G2D_CLIP_TOP = G2DSFR_IMG_TOP - G2DSFR_IMG_LEFT,
	G2D_CLIP_BOTTOM = G2DSFR_IMG_BOTTOM - G2DSFR_IMG_LEFT,
};

static bool g2d_validate_clip_region(unsigned long caps, int layer_flags,
				     u32 mode, struct g2d_reg cmds[])
{
	u32 hori = cmds[G2D_CLIP_LEFT].value | cmds[G2D_CLIP_RIGHT].value;
	u32 vert = cmds[G2D_CLIP_TOP].value | cmds[G2D_CLIP_BOTTOM].value;

	if (IS_AFBC(mode)) {
		if (!is_afbc_aligned(hori, vert, true, layer_flags, caps))
			return false;
	} else if (IS_SBWC(mode)) {
		if (!IS_SBWC_WIDTH_ALIGNED(hori))
			return false;
		if (IS_YUV420(mode) && !IS_SBWC_HEIGHT_420_ALIGNED(vert))
			return false;
		if (!IS_SBWC_HEIGHT_ALIGNED(vert))
			return false;
	} else if (IS_YUV(mode)) {
		/*
		 * DST clip region has the alignment restrictions
		 * accroding to the chroma subsampling
		 */
		if (!IS_EVEN(hori))
			return false;
		if (IS_YUV420(mode) && !IS_EVEN(vert))
			return false;
	}

	return true;
}

static const char *g2d_mode_comp_string(u32 colormode)
{
	if (IS_AFBC(colormode))
		return "AFBC";
	if (IS_SBWC(colormode))
		return "SBWC";
	return "";
}

static bool g2d_validate_image_format(struct g2d_device *g2d_dev, struct g2d_task *task,
				      struct g2d_layer *layer, bool dst)
{
	struct g2d_reg *commands = layer->commands;
	u32 stride = commands[G2DSFR_IMG_STRIDE].value;
	u32 mode   = commands[G2DSFR_IMG_COLORMODE].value;
	u32 width  = commands[G2DSFR_IMG_WIDTH].value;
	u32 height = commands[G2DSFR_IMG_HEIGHT].value;
	u32 left   = commands[G2DSFR_IMG_LEFT].value;
	u32 right  = commands[G2DSFR_IMG_RIGHT].value;
	u32 top    = commands[G2DSFR_IMG_TOP].value;
	u32 bottom = commands[G2DSFR_IMG_BOTTOM].value;
	u32 byte_per_pix = 0;
	const struct g2d_fmt *fmt;

	if (IS_AFBC(mode) && !dst) {
		width = ALIGN(width, get_afbc_width_align(layer->flags));
		height = ALIGN(height, get_afbc_height_align(layer->flags));
	}

	if (!g2d_validate_image_dimension(g2d_dev, width, height, left, top, right, bottom))
		return false;

	fmt = g2d_find_format(mode, g2d_dev->caps);
	if (!fmt)
		return false;

	byte_per_pix = fmt->bpp[0] / 8;

	if (stride) {
		int err = 0;

		if (IS_SBWC(mode) || IS_AFBC(mode) || IS_YUV(mode))
			err |= 1 << 1;
		if (IS_UORDER(mode) && stride != ALIGN(width * byte_per_pix, 16))
			err |= 1 << 2;
		if (stride < width * byte_per_pix)
			err |= 1 << 4;
		if (stride > G2D_MAX_SIZE * byte_per_pix)
			err |= 1 << 5;

		if (err) {
			perrfndev(g2d_dev,
				  "wrong(%#x) stride %u mode %#x size %ux%u",
				  err, stride, mode, width, height);
			return false;
		}
	} else if (IS_YUV(mode)) {
		/* TODO: Y8 handling if required */
		if (!IS_EVEN(width) || (!IS_YUV422(mode) && !IS_EVEN(height)))
			goto err_align;
	} else if (!IS_AFBC(mode)) {
		perrfndev(g2d_dev, "Non AFBC requires valid stride");
		return false;
	}

	if (IS_HWFC(task->flags)) {
		if ((dst && IS_AFBC(mode)) || IS_UORDER(mode)) {
			perrfndev(g2d_dev, "Invalid HWFC format with %s",
				  IS_AFBC(mode) ? "AFBC" : "UORDER");
			return false;
		}
		if (dst && (width != (right - left) || height != (bottom - top)))
			goto err_align;
	}

	if (IS_SBWC(mode)) {
		int dep = IS_YUV_P10(mode, g2d_dev->caps & G2D_DEVICE_CAPS_YUV_BITDEPTH) ? 10 : 8;
		u32 offset = dst ? G2DSFR_DST_Y_HEADER_STRIDE : G2DSFR_SRC_Y_HEADER_STRIDE;
		int i;
		u32 sbwc = dst ? commands[G2DSFR_DST_SBWCINFO].value :
			commands[G2DSFR_SRC_SBWCINFO].value;
		u32 hdr_strd;
		u32 pld_strd;
		unsigned int blocksize = SBWCL_BLOCK_SIZE(sbwc);

		if (IS_SBWCL(sbwc)) {
			hdr_strd = 0;
			pld_strd = get_sbwc_lossy_stride_payload(width, blocksize);
		} else {
			hdr_strd = get_sbwc_stride_header(g2d_dev->caps, width);
			pld_strd = get_sbwc_stride_payload(g2d_dev->caps, width, dep);
		}
		/* Stride is in the order : header, payload, header, payload */
		for (i = 0; i < 4; i += 2) {
			if (commands[offset + i].value != hdr_strd ||
			    commands[offset + i + 1].value != pld_strd) {
				perrfndev(g2d_dev,
					"Bad stride %u, %u for w %u mode %x",
					commands[offset + i].value,
					commands[offset + i + 1].value,
					width, mode);

				return false;
			}
		}
		if (IS_SBWCL(sbwc)) {
			if ((dep == 8) && (blocksize == 128)) {
				perrfndev(g2d_dev,
						"Bad SBWC Lossy (mode %x, info %x)",
						mode, sbwc);
				return false;
			}
			if (!(g2d_dev->caps & G2D_DEVICE_CAPS_SBWC_LOSSY)) {
				perrfndev(g2d_dev,
						"SBWC Lossy is not supported");
				return false;
			}
		}
		if (!(g2d_dev->caps & G2D_DEVICE_CAPS_SBWC)) {
			perrfndev(g2d_dev, "SBWC format is not supported");
			return false;
		}

		if (!IS_SBWC_WIDTH_ALIGNED(width) || !IS_SBWC_HEIGHT_ALIGNED(height))
			goto err_align;
		if (dst && (width != (right - left) || height != (bottom - top)))
			goto err_align;
	}

	if (IS_AFBC(mode) && !is_afbc_aligned(width, height, dst, layer->flags, g2d_dev->caps))
		goto err_align;

	if (!dst) {
		return true;
	}

	if (!g2d_validate_clip_region(g2d_dev->caps, layer->flags, mode,
				      &commands[G2DSFR_IMG_LEFT]))
		goto err_align;

	return true;
err_align:
	perrfndev(g2d_dev,
		  "Unaligned size %ux%u or crop [%ux%u, %ux%u) for %s %s %s",
		  commands[G2DSFR_IMG_WIDTH].value,
		  commands[G2DSFR_IMG_HEIGHT].value,
		  commands[G2DSFR_IMG_LEFT].value,
		  commands[G2DSFR_IMG_TOP].value,
		  commands[G2DSFR_IMG_RIGHT].value,
		  commands[G2DSFR_IMG_BOTTOM].value,
		  g2d_mode_comp_string(mode), fmt->name,
		  IS_HWFC(task->flags) ? "HWFC" : "");
	return false;
}

bool g2d_validate_source_commands(struct g2d_device *g2d_dev,
				  struct g2d_task *task,
				  unsigned int i, struct g2d_layer *source,
				  struct g2d_layer *target)
{
	u32 colormode = source->commands[G2DSFR_IMG_COLORMODE].value;
	u32 dst_mode = target->commands[G2DSFR_IMG_COLORMODE].value;
	u32 width, height;

	if (!g2d_validate_image_format(g2d_dev, task, source, false)) {
		perrfndev(g2d_dev, "Failed to validate source[%d] commands", i);
		return false;
	}

	if ((source->flags & G2D_LAYERFLAG_COLORFILL) != 0 && !IS_RGB(colormode)) {
		perrfndev(g2d_dev, "Image type should be RGB for Solid color layer:");
		perrdev(g2d_dev, "\tindex: %d, flags %#x, colormode: %#010x",
			i, source->flags, colormode);
		return false;
	}

	width = target->commands[G2DSFR_IMG_WIDTH].value;
	height = target->commands[G2DSFR_IMG_HEIGHT].value;

	if (IS_AFBC(colormode)) {
		width = ALIGN(width, get_afbc_width_align(source->flags));
		height = ALIGN(height, get_afbc_height_align(source->flags));
	}

	if (!g2d_validate_image_dimension(g2d_dev, width, height,
					  source->commands[G2DSFR_SRC_DSTLEFT].value,
					  source->commands[G2DSFR_SRC_DSTTOP].value,
					  source->commands[G2DSFR_SRC_DSTRIGHT].value,
					  source->commands[G2DSFR_SRC_DSTBOTTOM].value)) {
		perrfndev(g2d_dev, "Window of source[%d] floods the target", i);
		return false;
	}

	if (g2d_global_clip_okay(g2d_dev->caps, task))
		return true;

	if (!g2d_validate_clip_region(g2d_dev->caps, source->flags, dst_mode,
				      &source->commands[G2DSFR_SRC_DSTLEFT])) {
		perrfndev(g2d_dev, "Unaligned sink region [%ux%u, %ux%u)",
			  source->commands[G2DSFR_SRC_DSTLEFT].value,
			  source->commands[G2DSFR_SRC_DSTTOP].value,
			  source->commands[G2DSFR_SRC_DSTRIGHT].value,
			  source->commands[G2DSFR_SRC_DSTBOTTOM].value);
		return false;
	}

	return true;
}

bool g2d_validate_target_commands(struct g2d_device *g2d_dev,
				  struct g2d_task *task)
{
	if (!g2d_validate_image_format(g2d_dev, task, &task->target, true)) {
		perrfndev(g2d_dev, "Failed to validate target commands");
		return false;
	}

	return true;
}

/*
 * List of extra command
 *
 * {0x2000, 0x208C}, SRC CSC Coefficients
 * {0x2100, 0x2120}, DST CSC Coefficients
 * {0x3000, 0x3100}, HDR EOTF Coefficients
 * {0x3200, 0x3300}, Degamma Coefficients
 * {0x3400, 0x3420}, HDR Gamut Mapping Coefficients
 * {0x3500, 0x3520}, Degamma 2.2 Coefficients
 * {0x3600, 0x3680}, HDR Tone Mapping Coefficients
 * {0x3700, 0x3780}, Degamma Tone Mapping Coefficients
 * {0x5000, 0x5C40}, SET 0,1,2 Coefficients
 * {0x6000, 0x6FAC}, Coeficients for polyphase filter[0-3]
 */
static u16 extra_valid_range[2] = {0x2000, 0x8000}; // {0x2000, 0x8000]

static bool g2d_validate_extra_command(struct g2d_device *g2d_dev,
				       struct g2d_reg extra[],
				       unsigned int num_regs)
{
	unsigned int n;

	for (n = 0; n < num_regs; n++) {
		if (extra[n].offset < extra_valid_range[0] ||
		    extra[n].offset >= extra_valid_range[1]) {
			perrfndev(g2d_dev, "wrong offset %#x @ extra cmd[%d]", extra[n].offset, n);
			return false;
		}
	}

	return true;
}


#define G2D_MAX_IMAGE_COMMAND	\
	((G2D_MAX_IMAGES * G2DSFR_SRC_FIELD_COUNT) + G2DSFR_DST_FIELD_COUNT)
/*
 * Maximum of number of register set by driver.
 *
 * 3 initial commands
 * 17 sets of 4 address of source and destinaion.
 * 2 of Task control register
 * 1 of HW flow control register
 * 1 of Secure layer register
 */
#define G2D_TASK_COMMAND	(3 + (4 * (G2D_MAX_IMAGES + 1)) + 2 + 1 + 1)

// Sum of taskctl, layer, extra must not exceed G2D_MAX_COMMAND
#define G2D_MAX_EXTRA_COMMAND \
	(G2D_MAX_COMMAND - G2D_MAX_IMAGE_COMMAND - G2D_TASK_COMMAND)

int g2d_import_commands(struct g2d_device *g2d_dev, struct g2d_task *task,
			struct g2d_task_data *data, unsigned int num_sources,
			struct g2d_reg *cmdaddr)
{
	struct g2d_commands *cmds = &data->commands;
	u32 tgtcmds[G2DSFR_DST_FIELD_COUNT];
	unsigned int i;
	int copied;

	if (cmds->num_extra_regs > G2D_MAX_EXTRA_COMMAND) {
		perrfndev(g2d_dev, "Too many coefficient reigsters %d",
			  cmds->num_extra_regs);
		return -EINVAL;
	}

	cmdaddr += task->sec.cmd_count;

	if (copy_from_user(tgtcmds, cmds->target, sizeof(tgtcmds))) {
		perrfndev(g2d_dev, "Failed to get target commands");
		return -EFAULT;
	}
	copied = g2d_copy_commands(g2d_dev, -1, cmdaddr, tgtcmds,
				   target_command_checker, G2DSFR_DST_FIELD_COUNT);
	if (copied < 0)
		return -EINVAL;

	task->target.commands = cmdaddr;

	cmdaddr += copied;
	task->sec.cmd_count += copied;

	for (i = 0; i < num_sources; i++) {
		u32 srccmds[G2DSFR_SRC_FIELD_COUNT];

		if (copy_from_user(srccmds, cmds->source[i], sizeof(srccmds))) {
			perrfndev(g2d_dev, "Failed to get src[%d] commands", i);
			return -EFAULT;
		}

		copied = g2d_copy_commands(g2d_dev, i, cmdaddr, srccmds,
					   source_command_checker, G2DSFR_SRC_FIELD_COUNT);
		if (copied < 0)
			return -EINVAL;

		task->source[i].commands = cmdaddr;
		cmdaddr += copied;
		task->sec.cmd_count += copied;
	}

	if (copy_from_user(cmdaddr, cmds->extra,
			   cmds->num_extra_regs * sizeof(struct g2d_reg))) {
		perrfndev(g2d_dev, "Failed to get %u extra commands",
			  cmds->num_extra_regs);
		return -EFAULT;
	}

	if (!g2d_validate_extra_command(g2d_dev, cmdaddr, cmds->num_extra_regs))
		return -EINVAL;

	task->sec.cmd_count += cmds->num_extra_regs;

	return 0;
}

static unsigned int g2d_set_image_buffer(struct g2d_task *task,
					 struct g2d_layer *layer, u32 colormode,
					 u32 base)
{
	const struct g2d_fmt *fmt = g2d_find_format(colormode, task->g2d_dev->caps);
	struct g2d_reg *reg = (struct g2d_reg *)page_address(task->cmd_page);
	unsigned int cmd_count = task->sec.cmd_count;
	unsigned char *offsets = (base == TARGET_OFFSET) ?
				dst_base_reg_offset : src_base_reg_offset;
	u32 width = layer_width(layer);
	u32 height = layer_height(layer);
	unsigned int i;

	if (!fmt)
		return 0;

	if (!IS_ALIGNED(layer->buffer[0].dma_addr, 4)) {
		perrfndev(task->g2d_dev, "Plane 0 address isn't aligned by 4.");
		return 0;
	}

	for (i = 0; i < layer->num_buffers; i++) {
		reg[cmd_count + i].offset = BASE_REG_OFFSET(base, offsets, i);
		reg[cmd_count + i].value = layer->buffer[i].dma_addr;
	}

	if (layer->num_buffers == fmt->num_planes)
		return cmd_count + layer->num_buffers;

	BUG_ON(layer->num_buffers != 1);

	/* address of plane 0 is set in the above for() */
	if (!!(layer->flags & G2D_LAYERFLAG_MFC_STRIDE) &&
	    IS_YUV420(fmt->fmtvalue)) {
		/* Consider MFC padding and stride only for NV12 */
		reg[cmd_count + 1].offset = BASE_REG_OFFSET(base, offsets, 1);
		reg[cmd_count + 1].value =
				NV12_MFC_CBASE(layer->buffer[0].dma_addr,
					       width, height);
		return cmd_count + fmt->num_planes;
	}

	reg[cmd_count + 1].offset = BASE_REG_OFFSET(base, offsets, 1);
	reg[cmd_count + 1].value = width * height * fmt->bpp[0];
	reg[cmd_count + 1].value /= BITS_PER_BYTE;
	reg[cmd_count + 1].value += reg[cmd_count + 0].value;
	reg[cmd_count + 1].value = ALIGN(reg[cmd_count + 1].value, 16);

	if (fmt->num_planes == 3) {
		/* YCbCr420 planar */
		u32 yv12_stride = ALIGN(width / 2, 16);

		reg[cmd_count + 2].offset = BASE_REG_OFFSET(base, offsets, 2);
		reg[cmd_count + 2].value = yv12_stride * height / 2;
		reg[cmd_count + 2].value += reg[cmd_count + 1].value;
		reg[cmd_count + 2].value = ALIGN(reg[cmd_count + 2].value, 16);

		/* YCbCr420 planar needs to set chroma stride additionally */
		reg[cmd_count + 3].offset = base;
		if (base == TARGET_OFFSET)
			reg[cmd_count + 3].offset += REG_DST_CHROMA_STRIDE;
		else
			reg[cmd_count + 3].offset += REG_SRC_CHROMA_STRIDE;
		reg[cmd_count + 3].value = yv12_stride;

		cmd_count++;
	}

	return cmd_count + fmt->num_planes;
}

static u32 get_sbwc_c_base(unsigned long caps, struct g2d_layer *layer,
			   u32 bitdepth, u32 mode)
{
	u32 width = layer_width(layer);
	u32 height = layer_height(layer);

	if (layer->num_buffers == 2)
		return layer->buffer[1].dma_addr;

	return layer->buffer[0].dma_addr +
		get_sbwc_payload_y_size(caps, mode, width, height, bitdepth) +
		get_sbwc_header_y_size(caps, mode, width, height);
}

static u32 get_sbwc_lossy_c_base(struct g2d_layer *layer,
		u32 blocksize, u32 mode)
{
	u32 width = layer_width(layer);
	u32 height = layer_height(layer);

	if (layer->num_buffers == 2)
		return layer->buffer[1].dma_addr;
	return layer->buffer[0].dma_addr +
		get_sbwc_lossy_y_size(width, height, blocksize);
}

/*
 * SBWC buffer layout:
 * Single buffer: luma payload, luma header, chroma payload, chroma header
 * Dual buffer: Buffer[0]: luma payload, chroma payload
 *              Buffer[1]: luma header, chroma header
 */
static unsigned int g2d_set_sbwc_buffer(struct g2d_task *task,
					struct g2d_layer *layer, u32 colormode,
					u32 base)
{
	struct g2d_reg *reg = (struct g2d_reg *)page_address(task->cmd_page);
	unsigned char *offsets = (base == TARGET_OFFSET) ?
				dst_sbwc_reg_offset : src_sbwc_reg_offset;
	unsigned int sbwc = (base == TARGET_OFFSET) ?
		layer->commands[G2DSFR_DST_SBWCINFO].value :
		layer->commands[G2DSFR_SRC_SBWCINFO].value;
	unsigned int blocksize = SBWCL_BLOCK_SIZE(sbwc);

	u32 w = layer_width(layer);
	u32 h = layer_height(layer);
	u32 align = 32;
	unsigned long caps = task->g2d_dev->caps;
	unsigned int cmd_cnt = task->sec.cmd_count;
	unsigned int dep;

	if (caps_has_align64(caps))
		align = 64;

	if (layer->num_buffers > 2) {
		perrfndev(task->g2d_dev, "invalid number of SBWC buffers %d",
			  layer->num_buffers);
		return 0;
	}

	dep = IS_YUV_P10(colormode, task->g2d_dev->caps &
			G2D_DEVICE_CAPS_YUV_BITDEPTH) ? 10 : 8;

	if (!IS_ALIGNED(layer->buffer[0].dma_addr, align)) {
		perrfndev(task->g2d_dev, "SBWC base %#llx is not aligned %u",
			  layer->buffer[0].dma_addr, align);
		return 0;
	}

	reg[cmd_cnt + 0].offset = BASE_REG_OFFSET(base, offsets, 0);
	reg[cmd_cnt + 0].value = layer->buffer[0].dma_addr;

	if (IS_SBWCL(sbwc)) {
		reg[cmd_cnt + 1].offset = BASE_REG_OFFSET(base, offsets, 1);
		reg[cmd_cnt + 1].value = reg[cmd_cnt + 0].value + 0;

		reg[cmd_cnt + 2].offset = BASE_REG_OFFSET(base, offsets, 2);
		reg[cmd_cnt + 2].value = get_sbwc_lossy_c_base(layer, blocksize, colormode);

		reg[cmd_cnt + 3].offset = BASE_REG_OFFSET(base, offsets, 3);
		reg[cmd_cnt + 3].value = reg[cmd_cnt + 2].value + 0;
	} else {
		reg[cmd_cnt + 1].offset = BASE_REG_OFFSET(base, offsets, 1);
		reg[cmd_cnt + 1].value = reg[cmd_cnt + 0].value +
			get_sbwc_payload_y_size(caps, colormode,
					w, h, dep);

		reg[cmd_cnt + 2].offset = BASE_REG_OFFSET(base, offsets, 2);
		reg[cmd_cnt + 2].value = get_sbwc_c_base(caps, layer, dep, colormode);

		reg[cmd_cnt + 3].offset = BASE_REG_OFFSET(base, offsets, 3);
		reg[cmd_cnt + 3].value = reg[cmd_cnt + 2].value +
			get_sbwc_payload_c_size(caps, colormode,
					w, h, dep);
	}
	return cmd_cnt + 4;
}

static unsigned int g2d_set_afbc_buffer(struct g2d_task *task,
					struct g2d_layer *layer,
					u32 base_offset)
{
	struct g2d_reg *reg = (struct g2d_reg *)page_address(task->cmd_page);
	u32 align;
	unsigned char *reg_offset = (base_offset == TARGET_OFFSET) ?
				dst_afbc_reg_offset : src_afbc_reg_offset;
	unsigned long caps = task->g2d_dev->caps;

	if (caps_has_afbcv12(caps))
		align = AFBCV12_ALIGN_LEN;
	else if ((base_offset == TARGET_OFFSET) || caps_has_align64(caps))
		align = AFBCV1_ALIGN_LEN;
	else
		align = AFBCV1_ALIGN_LEN_SRC;

	if (!IS_ALIGNED(layer->buffer[0].dma_addr, align)) {
		perrfndev(task->g2d_dev, "AFBC base %#llx is not aligned by %u",
			  layer->buffer[0].dma_addr, align);
		return 0;
	}

	reg[task->sec.cmd_count].offset = base_offset + reg_offset[0];
	reg[task->sec.cmd_count].value = layer->buffer[0].dma_addr;
	reg[task->sec.cmd_count + 1].offset = base_offset + reg_offset[1];
	if (base_offset == TARGET_OFFSET)
		reg[task->sec.cmd_count + 1].value =
			ALIGN(afbc_header_len_only(caps, layer->flags,
						   layer->commands)
				+ layer->buffer[0].dma_addr, align);
	else
		reg[task->sec.cmd_count + 1].value = layer->buffer[0].dma_addr;

	return task->sec.cmd_count + 2;
}

static bool g2d_prepare_layer(struct g2d_task *task, struct g2d_layer *layer,
			      u32 base)
{
	u32 colormode = layer->commands[G2DSFR_IMG_COLORMODE].value;
	unsigned int cmd_count;

	if (IS_AFBC(colormode))
		cmd_count = g2d_set_afbc_buffer(task, layer, base);
	else if (IS_SBWC(colormode))
		cmd_count = g2d_set_sbwc_buffer(task, layer, colormode, base);
	else
		cmd_count = g2d_set_image_buffer(task, layer, colormode, base);

	task->sec.cmd_count = cmd_count;

	return task->sec.cmd_count != 0;
}

bool g2d_prepare_source(struct g2d_task *task,
			struct g2d_layer *layer, int index)
{
	struct g2d_reg *reg = (struct g2d_reg *)page_address(task->cmd_page);

	reg[TASK_REG_LAYER_UPDATE].value |= LAYER_UPDATE_INDEX << index;

	if ((layer->flags & G2D_LAYERFLAG_COLORFILL) != 0)
		return true;

	return g2d_prepare_layer(task, layer, LAYER_OFFSET(index));
}

bool g2d_prepare_target(struct g2d_task *task)
{
	return g2d_prepare_layer(task, &task->target, TARGET_OFFSET);
}
