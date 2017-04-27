/*
 * linux/drivers/gpu/exynos/g2d/g2d_command.c
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/device.h>

#include "g2d.h"
#include "g2d_task.h"
#include "g2d_uapi.h"
#include "g2d_command.h"

#define NV12N_Y_SIZE(w, h)	(ALIGN((w), 16) * ALIGN((h), 16) + 256)
#define NV12N_CBCR_SIZE(w, h)		\
		(ALIGN((ALIGN((w), 16) * (ALIGN((h), 16) / 2) + 256), 16))
#define NV12N_CBCR_BASE(base, w, h)	((base) + NV12N_Y_SIZE((w), (h)))

#define NV12N10B_Y_2B_SIZE(w, h)    ((ALIGN((w) / 4, 16) * ALIGN((h), 16) + 64))
#define NV12N10B_CBCR_2B_SIZE(w, h)     \
			((ALIGN((w) / 4, 16) * (ALIGN((h), 16) / 2) + 64))
#define NV12N10B_CBCR_BASE(base, w, h)	\
		((base) + NV12N_Y_SIZE((w), (h)) + NV12N10B_Y_2B_SIZE((w), (h)))

static const struct g2d_fmt g2d_formats[] = {
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
		.name		= "NV12_8+2",
		.fmtvalue	= G2D_FMT_NV12_82,
		.bpp		= { 8, 2, 4, 1 },
		.num_planes	= 4,
	}, {
		.name		= "NV21_8+2",
		.fmtvalue	= G2D_FMT_NV21_82,
		.bpp		= { 8, 2, 4, 1 },
		.num_planes	= 4,
	}, {
		.name		= "NV12_P010",
		.fmtvalue	= G2D_FMT_NV12_P010,
		.bpp		= { 16, 8},
		.num_planes	= 2,
	}, {
		.name		= "NV21N_P010",
		.fmtvalue	= G2D_FMT_NV21_P010,
		.bpp		= { 16, 8},
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
	},
};

const struct g2d_fmt *g2d_find_format(u32 fmtval)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(g2d_formats); i++)
		if (g2d_formats[i].fmtvalue == G2D_IMGFMT(fmtval))
			return &g2d_formats[i];

	return NULL;
}

#define layer_width(layer)	((layer)->commands[G2DSFR_IMG_WIDTH].value)
#define layer_height(layer)	((layer)->commands[G2DSFR_IMG_HEIGHT].value)
#define layer_pixelcount(layer)	(layer_width(layer) * layer_height(layer))
#define YUV82_BASE_ALIGNED(addr, idx) IS_ALIGNED((addr), 32 >> (idx / 2))
#define YUV82_BASE_ALIGN(addr, idx)   ALIGN((addr), 32 >> (idx / 2))

static unsigned char src_base_reg_offset[4] = {0x1C, 0x80, 0x64, 0x68};
static unsigned char src_base_reg_offset_yuv82[4] = {0x1C, 0x64, 0x80, 0x68};
static unsigned char dst_base_reg_offset[4] = {0x00, 0x50, 0x30, 0x34};
#define BASE_REG_OFFSET(base, offsets, buf_idx)	((base) + (offsets)[(buf_idx)])

#define AFBC_HEADER_SIZE(cmd)						\
		((ALIGN((cmd)[G2DSFR_IMG_WIDTH].value, 16) / 16) *	\
		 (ALIGN((cmd)[G2DSFR_IMG_HEIGHT].value, 16) / 16) * 16)

size_t g2d_get_payload_index(struct g2d_reg cmd[], const struct g2d_fmt *fmt,
			     unsigned int idx)
{
	/*
	 * TODO: consider NV12N and similar image formats
	 *       with alignment restriction
	 */
	BUG_ON(!IS_YUV(cmd[G2DSFR_IMG_COLORMODE].value));

	return ((cmd[G2DSFR_IMG_WIDTH].value * fmt->bpp[idx]) / 8) *
						cmd[G2DSFR_IMG_BOTTOM].value;
}

size_t g2d_get_payload(struct g2d_reg cmd[], const struct g2d_fmt *fmt,
		       u32 flags)
{
	/*
	 * TODO: consider NV12N and similar image formats
	 *       with alignment restriction
	 */
	size_t payload;
	u32 mode = cmd[G2DSFR_IMG_COLORMODE].value;
	u32 width = cmd[G2DSFR_IMG_WIDTH].value;
	u32 height = cmd[G2DSFR_IMG_BOTTOM].value;
	size_t pixcount = width * height;

	if (IS_YUV420_82(mode)) {
		payload  = YUV82_BASE_ALIGN((pixcount * fmt->bpp[0]) / 8, 1);
		payload += YUV82_BASE_ALIGN((pixcount * fmt->bpp[1]) / 8, 2);
		payload += YUV82_BASE_ALIGN((pixcount * fmt->bpp[2]) / 8, 3);
		payload += (pixcount * fmt->bpp[3]) / 8;
	} else if (IS_YUV(mode)) {
		unsigned int i;

		payload = 0;
		if (((flags & G2D_LAYERFLAG_MFC_STRIDE) != 0) &&
				(fmt->fmtvalue == G2D_FMT_NV12)) {
			payload += NV12N_Y_SIZE(width, height);
			payload += NV12N_CBCR_SIZE(width, height);
		} else {
			for (i = 0; i < fmt->num_planes; i++)
				payload += (pixcount * fmt->bpp[i]) / 8;
		}
	} else if (IS_AFBC(mode)) {
		payload = AFBC_HEADER_SIZE(cmd) + (pixcount * fmt->bpp[0]) / 8;
	} else {
		payload = cmd[G2DSFR_IMG_STRIDE].value *
				cmd[G2DSFR_IMG_BOTTOM].value;
	}

	return payload;
}

static bool check_width_height(u32 value)
{
	return (value > 0) && (value <= G2D_MAX_SIZE);
}

/* 8bpp(grayscale) format is not supported */
static bool check_srccolor_mode(u32 value)
{
	u32 fmt = ((value) & G2D_DATAFMT_MASK) >> G2D_DATAFMT_SHIFT;

	if ((fmt > 13) || (fmt == 6) || (fmt == 7) || (fmt == 9))
		return false;

	if (IS_YUV(value) && (value & G2D_DATAFORMAT_AFBC))
		return false;

	return true;
}

static bool check_dstcolor_mode(u32 value)
{
	u32 fmt = ((value) & G2D_DATAFMT_MASK) >> G2D_DATAFMT_SHIFT;

	/* src + YCbCr420 3p, - YCbCr420 2p 8.2 */
	if ((fmt > 12) || (fmt == 6) || (fmt == 7))
		return false;

	/* AFBC and UORDER shoult not be set together */
	if ((value & (G2D_DATAFORMAT_AFBC | G2D_DATAFORMAT_UORDER)) ==
			(G2D_DATAFORMAT_AFBC | G2D_DATAFORMAT_UORDER))
		return false;

	if (IS_YUV(value) &&
			(value & (G2D_DATAFORMAT_AFBC | G2D_DATAFORMAT_UORDER)))
		return false;

	return true;
}

static bool check_blend_mode(u32 value)
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

static bool check_scale_control(u32 value)
{
	return value != 3;
}

struct command_checker {
	const char *cmdname;
	u32 offset;
	u32 mask;
	bool (*checker)(u32 value);
};

static struct command_checker source_command_checker[G2DSFR_SRC_FIELD_COUNT] = {
	{"STRIDE",	0x0020, 0x0001FFFF, NULL,},
	{"COLORMODE",	0x0028, 0x031FFFFF, check_srccolor_mode,},
	{"LEFT",	0x002C, 0x00001FFF, NULL,},
	{"TOP",		0x0030, 0x00001FFF, NULL,},
	{"RIGHT",	0x0034, 0x00003FFF, check_width_height,},
	{"BOTTOM",	0x0038, 0x00003FFF, check_width_height,},
	{"WIDTH",	0x0070, 0x00003FFF, check_width_height,},
	{"HEIGHT",	0x0074, 0x00003FFF, check_width_height,},
	{"COMMAND",	0x0000, 0x03100001, NULL,},
	{"SELECT",	0x0004, 0x00000001, NULL,},
	{"ROTATE",	0x0008, 0x00000031, NULL,},
	{"DSTLEFT",	0x000C, 0x00001FFF, NULL,},
	{"DSTTOP",	0x0010, 0x00001FFF, NULL,},
	{"DSTRIGHT",	0x0014, 0x00003FFF, check_width_height,},
	{"DSTBOTTOM",	0x0018, 0x00003FFF, check_width_height,},
	{"SCALECONTROL", 0x048, 0x00000003, check_scale_control,},
	{"XSCALE",	0x004C, 0x3FFFFFFF, NULL,},
	{"YSCALE",	0x0050, 0x3FFFFFFF, NULL,},
	{"XPHASE",	0x0054, 0x0000FFFF, NULL,},
	{"YPHASE",	0x0058, 0x0000FFFF, NULL,},
	{"COLOR",	0x005C, 0xFFFFFFFF, NULL,},
	{"ALPHA",	0x0060, 0xFFFFFFFF, NULL,},
	{"BLEND",	0x003C, 0x0035FFFF, check_blend_mode,},
	{"YCBCRMODE",	0x0088, 0x10000013, NULL,},
	{"HDRMODE",	0x0090, 0x000011B3, NULL,},
};

static struct command_checker target_command_checker[G2DSFR_DST_FIELD_COUNT] = {
	/* BASE OFFSET: 0x0120 */
	{"STRIDE",	0x0004, 0x0001FFFF, NULL,},
	{"COLORMODE",	0x000C, 0x033FFFFF, check_dstcolor_mode,},
	{"LEFT",	0x0010, 0x00001FFF, NULL,},
	{"TOP",		0x0014, 0x00001FFF, NULL,},
	{"RIGHT",	0x0018, 0x00003FFF, check_width_height,},
	{"BOTTOM",	0x001C, 0x00003FFF, check_width_height,},
	{"WIDTH",	0x0040, 0x00003FFF, check_width_height,},
	{"HEIGHT",	0x0044, 0x00003FFF, check_width_height,},
	/* TODO: check csc */
	{"YCBCRMODE",	0x0058, 0x0000F714, NULL,},
};

static u16 extra_cmd_range[][2] = { /* {first, last} */
	{0x2000, 0x208C}, /* SRC CSC Coefficients */
	{0x2100, 0x2120}, /* DST CSC Coefficients */
	{0x3000, 0x3100}, /* HDR EOTF Coefficients */
	{0x3200, 0x3300}, /* Degamma Coefficients */
	{0x3400, 0x3420}, /* HDR Gamut Mapping Coefficients */
	{0x3500, 0x3520}, /* Degamma 2.2 Coefficients */
	{0x3600, 0x3680}, /* HDR Tone Mapping Coefficients */
	{0x3700, 0x3780}, /* Degamma Tone Mapping Coefficients */
};

#define TARGET_OFFSET		0x120
#define LAYER_OFFSET(idx)	((2 + (idx)) << 8)

static int g2d_copy_commands(struct g2d_device *g2d_dev, int index,
			      struct g2d_reg regs[], __u32 cmd[],
			      struct command_checker checker[],
			      unsigned int num_cmds)
{
	unsigned int base = (index < 0) ? TARGET_OFFSET : LAYER_OFFSET(index);
	int i;

	for (i = 0; i < num_cmds; i++) {
		if (((cmd[i] & ~checker[i].mask) != 0) ||
				(checker[i].checker &&
					!checker[i].checker(cmd[i]))) {
			dev_err(g2d_dev->dev,
				"%s: Invalid %s[%d] SFR '%s' value %#x\n",
				__func__, (index < 0) ? "target" : "source",
				index, checker[i].cmdname, cmd[i]);
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
	if ((left >= right) || (top >= bottom) ||
			(width < (right - left)) || (height < (bottom - top))) {
		dev_err(g2d_dev->dev,
			"%s: Invalid dimension [%ux%u, %ux%u) / %ux%u\n",
			__func__, left, top, right, bottom, width, height);
		return false;
	}

	return true;
}

#define IS_EVEN(value) (((value) & 1) == 0)

static bool g2d_validate_image_format(struct g2d_device *g2d_dev,
				      struct g2d_reg commands[], bool dst)
{
	struct device *dev = g2d_dev->dev;
	u32 stride = commands[G2DSFR_IMG_STRIDE].value;
	u32 mode   = commands[G2DSFR_IMG_COLORMODE].value;
	u32 width  = commands[G2DSFR_IMG_WIDTH].value;
	u32 height = commands[G2DSFR_IMG_HEIGHT].value;
	u32 Bpp = 0;
	const struct g2d_fmt *fmt;

	if (IS_AFBC(mode) && !dst) {
		width++;
		height++;
	}

	if (!g2d_validate_image_dimension(g2d_dev, width, height,
					commands[G2DSFR_IMG_LEFT].value,
					commands[G2DSFR_IMG_TOP].value,
					commands[G2DSFR_IMG_RIGHT].value,
					commands[G2DSFR_IMG_BOTTOM].value))
		return false;

	fmt = g2d_find_format(mode);
	if (fmt == NULL) {
		dev_err(dev, "%s: Color mode %#x is not supported\n",
			__func__, mode);
		return false;
	}

	Bpp = fmt->bpp[0] / 8;

	if (stride) {
		int err = 0;

		if (IS_AFBC(mode) || IS_YUV(mode))
			err |= 1 << 1;
		if (IS_UORDER(mode) & (stride != ALIGN(width * Bpp, 16)))
			err |= 1 << 2;
		if (stride < (width * Bpp))
			err |= 1 << 4;
		if (stride > (G2D_MAX_SIZE * Bpp))
			err |= 1 << 5;

		if (err) {
			dev_err(dev,
			"%s: Invalid[%#x] stride %u with mode %#x size %ux%u\n",
				__func__, err, stride, mode, width, height);
			return false;
		}
	} else if (IS_YUV(mode)) {
		/* TODO: Y8 handling if required */
		if (!IS_EVEN(width) || (!IS_YUV422(mode) && !IS_EVEN(height)))
			goto err_align;
	} else if (!IS_AFBC(mode)) {
		dev_err(dev, "%s: Non AFBC RGB requires valid stride\n",
			__func__);
		return false;
	}

	if (!dst) {
		if (IS_AFBC(mode) && (!IS_AFBC_WIDTH_ALIGNED(width) ||
					!IS_AFBC_HEIGHT_ALIGNED(height)))
			goto err_align;

		if (IS_YUV420_82(mode) && !IS_ALIGNED(width, 64))
			goto err_align;

		return true;
	}

	width = commands[G2DSFR_IMG_LEFT].value |
				commands[G2DSFR_IMG_RIGHT].value;
	height = commands[G2DSFR_IMG_TOP].value |
					commands[G2DSFR_IMG_BOTTOM].value;

	if (IS_AFBC(mode) && !IS_AFBC_WIDTH_ALIGNED(width | height))
		goto err_align;

	if (IS_YUV(mode)) {
		/*
		 * DST clip region has the alignment restrictions
		 * accroding to the chroma subsampling
		 */
		if (!IS_EVEN(width) || (!IS_YUV422(mode) && !IS_EVEN(height)))
			goto err_align;
	}

	return true;
err_align:
	dev_err(dev,
		"%s: Unaligned size %ux%u or crop [%ux%u, %ux%u) for %s %s\n",
		__func__,
		commands[G2DSFR_IMG_WIDTH].value,
		commands[G2DSFR_IMG_HEIGHT].value,
		commands[G2DSFR_IMG_LEFT].value,
		commands[G2DSFR_IMG_TOP].value,
		commands[G2DSFR_IMG_RIGHT].value,
		commands[G2DSFR_IMG_BOTTOM].value,
		IS_AFBC(mode) ? "AFBC" :
			IS_YUV422(mode) ? "YUV422" : "YUV20",
		IS_YUV420_82(mode) ? "8+2" : "");

	return false;
}

bool g2d_validate_source_commands(struct g2d_device *g2d_dev,
				  unsigned int i, struct g2d_layer *source,
				  struct g2d_layer *target)
{
	u32 colormode = source->commands[G2DSFR_IMG_COLORMODE].value;
	u32 width, height;

	if (!g2d_validate_image_format(g2d_dev, source->commands, false)) {
		dev_err(g2d_dev->dev,
			"%s: Failed to validate source[%d] commands\n",
			__func__, i);
		return false;
	}

	if (((source->flags & G2D_LAYERFLAG_COLORFILL) != 0) &&
							!IS_RGB(colormode)) {
		dev_err(g2d_dev->dev,
			"%s: Image type should be RGB for Solid color layer\n",
			__func__);
		dev_err(g2d_dev->dev,
			"%s: layer index: %d, flags %#x, colormode: %#010x\n",
			__func__, i, source->flags, colormode);
		return false;
	}

	width = target->commands[G2DSFR_IMG_WIDTH].value;
	height = target->commands[G2DSFR_IMG_HEIGHT].value;

	if (IS_AFBC(colormode)) {
		width++;
		height++;
	}

	if (!g2d_validate_image_dimension(g2d_dev, width, height,
				source->commands[G2DSFR_SRC_DSTLEFT].value,
				source->commands[G2DSFR_SRC_DSTTOP].value,
				source->commands[G2DSFR_SRC_DSTRIGHT].value,
				source->commands[G2DSFR_SRC_DSTBOTTOM].value)) {
		dev_err(g2d_dev->dev,
			"%s: Window of source[%d] floods the target\n",
			__func__, i);
		return false;
	}

	return true;
}

bool g2d_validate_target_commands(struct g2d_device *g2d_dev,
				  struct g2d_task *task)
{
	if (!g2d_validate_image_format(g2d_dev, task->target.commands, true)) {
		dev_err(g2d_dev->dev,
			"%s: Failed to validate target commands\n", __func__);
		return false;
	}

	return true;
}

static bool g2d_validate_extra_command(struct g2d_device *g2d_dev,
				       struct g2d_reg extra[],
				       unsigned int num_regs)
{
	unsigned int n, i;
	/*
	 * TODO: NxM loop ==> total 2008 comparison are required in maximum:
	 * Consider if we can make it simple with a single range [0x2000, 4000)
	 */
	for (n = 0; n < num_regs; n++) {
		for (i = 0; i < ARRAY_SIZE(extra_cmd_range); i++) {
			if ((extra[n].offset >= extra_cmd_range[i][0]) &&
				(extra[n].offset <= extra_cmd_range[i][1]))
				break;
		}

		if (i == ARRAY_SIZE(extra_cmd_range)) {
			dev_err(g2d_dev->dev,
				"%s: Invalid offset %#x @ extra command[%d]\n",
				__func__, extra[n].offset, n);
			return false;
		}
	}

	return true;
}

int g2d_import_commands(struct g2d_device *g2d_dev, struct g2d_task *task,
			struct g2d_task_data *data, unsigned int num_sources)
{
	struct device *dev = g2d_dev->dev;
	struct g2d_reg *cmdaddr = page_address(task->cmd_page);
	struct g2d_commands *cmds = &data->commands;
	unsigned int i;
	int copied;

	task->cmd_count = 0;

	copied = g2d_copy_commands(g2d_dev, -1, cmdaddr, cmds->target,
				target_command_checker, G2DSFR_DST_FIELD_COUNT);
	if (copied < 0)
		return -EINVAL;

	task->target.commands = cmdaddr;

	cmdaddr += copied;
	task->cmd_count += copied;

	for (i = 0; i < num_sources; i++) {
		u32 srccmds[G2DSFR_SRC_FIELD_COUNT];

		if (copy_from_user(srccmds, cmds->source[i], sizeof(srccmds))) {
			dev_err(dev, "%s: Failed to get source[%d] commands\n",
				__func__, i);
			return -EFAULT;
		}

		copied = g2d_copy_commands(g2d_dev, i, cmdaddr, srccmds,
				source_command_checker, G2DSFR_SRC_FIELD_COUNT);
		if (copied < 0)
			return -EINVAL;

		task->source[i].commands = cmdaddr;
		cmdaddr += copied;
		task->cmd_count += copied;
	}

	if (copy_from_user(cmdaddr, cmds->extra,
			   cmds->num_extra_regs * sizeof(struct g2d_reg))) {
		dev_err(dev, "%s: Failed to get %u extra commands\n", __func__,
			cmds->num_extra_regs);
		return -EFAULT;
	}

	if (!g2d_validate_extra_command(g2d_dev, cmdaddr, cmds->num_extra_regs))
		return -EINVAL;

	task->cmd_count += cmds->num_extra_regs;

	return 0;
}

static unsigned int g2d_set_image_buffer(struct g2d_task *task,
					 struct g2d_layer *layer, u32 colormode,
					 unsigned char offsets[], u32 base)
{
	const struct g2d_fmt *fmt = g2d_find_format(colormode);
	struct g2d_reg *reg = (struct g2d_reg *)page_address(task->cmd_page);
	unsigned int cmd_count = task->cmd_count;
	unsigned int i;
	dma_addr_t addr;

	if (fmt->num_planes == 4) {
		unsigned int nbufs = min_t(unsigned int,
					   layer->num_buffers, fmt->num_planes);

		for (i = 0; i < nbufs; i++) {
			if (!YUV82_BASE_ALIGNED(layer->buffer[i].dma_addr, i)) {
				dev_err(task->g2d_dev->dev,
				"%s: Plane %d Addr isn't aligned for YUV 8+2\n",
					__func__, i);
				return 0;
			}
		}
	} else if (!IS_ALIGNED(layer->buffer[0].dma_addr, 4)) {
		dev_err(task->g2d_dev->dev,
			"%s: Plane 0 address isn't aligned by 4.\n", __func__);
		return 0;
	}

	for (i = 0; i < layer->num_buffers; i++) {
		reg[cmd_count].offset = BASE_REG_OFFSET(base, offsets, i);
		reg[cmd_count].value = layer->buffer[i].dma_addr;
		cmd_count++;
	}

	if (layer->num_buffers == fmt->num_planes)
		return cmd_count;

	if (fmt->num_planes == 2) {
		/* YCbCr semi-planar in a single buffer */
		reg[cmd_count].offset = BASE_REG_OFFSET(base, offsets, 1);
		if (((layer->flags & G2D_LAYERFLAG_MFC_STRIDE) != 0) &&
					(fmt->fmtvalue == G2D_FMT_NV12)) {
			reg[cmd_count].value =
				NV12N_CBCR_BASE(layer->buffer[0].dma_addr,
						layer_width(layer),
						layer_height(layer));
		} else {
			reg[cmd_count].value = layer_pixelcount(layer);
			reg[cmd_count].value *= fmt->bpp[0] / 8;
			reg[cmd_count].value += layer->buffer[0].dma_addr;
			reg[cmd_count].value = ALIGN(reg[cmd_count].value, 2);
		}
		cmd_count++;
		return cmd_count;
	}

	addr = layer->buffer[0].dma_addr;
	/* YCbCr semi-planar 8+2 in a single buffer */
	for (i = 1; i < 4; i++) {
		addr += (layer_pixelcount(layer) * fmt->bpp[i - 1]) / 8;
		addr = YUV82_BASE_ALIGN(addr, 32);
		reg[cmd_count].value = addr;
		reg[cmd_count].offset = BASE_REG_OFFSET(base, offsets, i);
		cmd_count++;
	}

	return cmd_count;
}

static unsigned int g2d_set_afbc_buffer(struct g2d_task *task,
					struct g2d_layer *layer,
					u32 base_offset)
{
	struct g2d_reg *reg = (struct g2d_reg *)page_address(task->cmd_page);
	u32 align = (base_offset == TARGET_OFFSET) ? 64 : 16;
	unsigned char *reg_offset = (base_offset == TARGET_OFFSET) ?
				dst_base_reg_offset : src_base_reg_offset;

	if (!IS_ALIGNED(layer->buffer[0].dma_addr, align)) {
		dev_err(task->g2d_dev->dev,
			"%s: AFBC base %#llx is not aligned by %u\n",
			__func__, layer->buffer[0].dma_addr, align);
		return 0;
	}

	reg[task->cmd_count].offset = base_offset + reg_offset[2];
	reg[task->cmd_count].value = layer->buffer[0].dma_addr;
	reg[task->cmd_count + 1].offset = base_offset + reg_offset[3];
	if (base_offset == TARGET_OFFSET)
		reg[task->cmd_count + 1].value =
			ALIGN(AFBC_HEADER_SIZE(layer->commands)
					+ layer->buffer[0].dma_addr, align);
	else
		reg[task->cmd_count + 1].value = layer->buffer[0].dma_addr;

	return task->cmd_count + 2;
}

bool g2d_prepare_source(struct g2d_task *task,
			struct g2d_layer *layer, int index)
{
	u32 colormode = layer->commands[G2DSFR_IMG_COLORMODE].value;
	unsigned char *offsets = IS_YUV420_82(colormode) ?
				src_base_reg_offset_yuv82 : src_base_reg_offset;

	task->cmd_count = ((colormode & G2D_DATAFORMAT_AFBC) != 0)
			? g2d_set_afbc_buffer(task, layer, LAYER_OFFSET(index))
			: g2d_set_image_buffer(task, layer, colormode,
				offsets, LAYER_OFFSET(index));
	/*
	 * It is alright to set task->cmd_count to 0
	 * because this task is to be discarded.
	 */
	return task->cmd_count != 0;
}

bool g2d_prepare_target(struct g2d_task *task)
{
	u32 colormode = task->target.commands[G2DSFR_IMG_COLORMODE].value;

	task->cmd_count = ((colormode & G2D_DATAFORMAT_AFBC) != 0)
			? g2d_set_afbc_buffer(task, &task->target,
					      TARGET_OFFSET)
			: g2d_set_image_buffer(task, &task->target, colormode,
					dst_base_reg_offset, TARGET_OFFSET);

	return task->cmd_count != 0;
}
