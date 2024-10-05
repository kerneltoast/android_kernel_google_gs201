// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_dpp.c
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Authors:
 *	Seong-gyu Park <seongyu.park@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt)  "%s: " fmt, __func__

#include <drm/exynos_drm.h>
#include <drm/drm_atomic.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fourcc_gs101.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <uapi/linux/videodev2_exynos_media.h>
#include <linux/dma-buf.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <linux/dma-heap.h>

#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
#include <soc/google/exynos-devfreq.h>
#if IS_ENABLED(CONFIG_SOC_GS101)
#include <dt-bindings/soc/google/gs101-devfreq.h>
#elif IS_ENABLED(CONFIG_SOC_GS201)
#include <dt-bindings/soc/google/gs201-devfreq.h>
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#include <dt-bindings/soc/google/zuma-devfreq.h>
#endif
#endif


#include <hdr_cal.h>
#include <regs-dpp.h>
#include <soc/google/debug-snapshot.h>

#include "exynos_drm_decon.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_dpp.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_format.h"

#define dpp_drm_printf(p, dpp, fmt, ...) \
drm_printf(p, "%s[%d]: "fmt, dpp->dev->driver->name, dpp->id, ##__VA_ARGS__)

#define dpp_info(dpp, fmt, ...)	\
pr_info("%s[%d]: "fmt, dpp->dev->driver->name, dpp->id, ##__VA_ARGS__)

#define dpp_warn(dpp, fmt, ...)	\
pr_warn("%s[%d]: "fmt, dpp->dev->driver->name, dpp->id, ##__VA_ARGS__)

#define dpp_err(dpp, fmt, ...)	\
pr_err("%s[%d]: "fmt, dpp->dev->driver->name, dpp->id, ##__VA_ARGS__)

#define dpp_debug(dpp, fmt, ...)	\
pr_debug("%s[%d]: "fmt, dpp->dev->driver->name, dpp->id, ##__VA_ARGS__)

#define P010_Y_SIZE(w, h)		((w) * (h) * 2)
#define P010_CBCR_SIZE(w, h)		((w) * (h))
#define P010_CBCR_BASE(base, w, h)	((base) + P010_Y_SIZE((w), (h)))

#define IN_RANGE(val, min, max)					\
	(((min) > 0 && (min) < (max) &&				\
	  (val) >= (min) && (val) <= (max)) ? true : false)

static const uint32_t dpp_gf_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_RGBA1010102,
	DRM_FORMAT_BGRA1010102,
#ifdef CONFIG_SOC_ZUMA
	DRM_FORMAT_ARGB16161616F,
	DRM_FORMAT_ABGR16161616F,
#endif

};

/* TODO: NV12M, NV12N, NV12_P010, ... modifier? */
static const uint32_t dpp_vg_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_RGBA1010102,
	DRM_FORMAT_BGRA1010102,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV61,
	DRM_FORMAT_P010,
	DRM_FORMAT_YUV420_8BIT,
	DRM_FORMAT_YUV420_10BIT,
#ifdef CONFIG_SOC_ZUMA
	DRM_FORMAT_ARGB16161616F,
	DRM_FORMAT_ABGR16161616F,
#endif

};

static const uint32_t rcd_alpha_formats[] = {
	DRM_FORMAT_C8,
};

static const uint64_t dpp_format_modifiers[] = {
	DRM_FORMAT_MOD_ARM_AFBC(0),
	DRM_FORMAT_MOD_SAMSUNG_SBWC(0),
	DRM_FORMAT_MOD_INVALID,
};

const struct dpp_restriction dpp_drv_data = {
	.src_f_w.min = 16,
	.src_f_w.max = 65534,
	.src_f_w.align = 1,
	.src_f_h.min = 16,
	.src_f_h.max = 8190,
	.src_f_h.align = 1,
	.src_w.min = 16,
	.src_w.max = 4096,
	.src_w.align = 1,
	.src_h.min = 16,
	.src_h.max = 4096,
	.src_h.align = 1,
	.src_x_align = 1,
	.src_y_align = 1,

	.dst_f_w.min = 16,
	.dst_f_w.max = 8190,
	.dst_f_w.align = 1,
	.dst_f_h.min = 16,
	.dst_f_h.max = 8190,
	.dst_f_h.align = 1,
	.dst_w.min = 16,
	.dst_w.max = 4096,
	.dst_w.align = 1,
	.dst_h.min = 16,
	.dst_h.max = 4096,
	.dst_h.align = 1,
	.dst_x_align = 1,
	.dst_y_align = 1,

	.blk_w.min = 4,
	.blk_w.max = 4096,
	.blk_w.align = 1,
	.blk_h.min = 4,
	.blk_h.max = 4096,
	.blk_h.align = 1,
	.blk_x_align = 1,
	.blk_y_align = 1,

	.src_h_rot_max = 2160,
};

static const struct of_device_id dpp_of_match[] = {
	{
		.compatible = "samsung,exynos-dpp",
		.data = &dpp_drv_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, dpp_of_match);

static inline const char *get_comp_type_str(enum dpp_comp_type type)
{
	if (type == COMP_TYPE_AFBC)
		return "AFBC";
	else if (type == COMP_TYPE_SBWC)
		return "SBWC";
	else
		return "";
}

void dpp_dump_buffer(struct drm_printer *p, struct dpp_device *dpp)
{
	const struct drm_plane_state *plane_state;
	struct exynos_drm_plane_state *exynos_plane_state;
	struct drm_framebuffer *fb, *old_fb;
	void *vaddr;

	if (dpp->state != DPP_STATE_ON) {
		dpp_drm_printf(p, dpp, "dpp state is off\n");
		return;
	}

	if (dpp->win_config.comp_type == COMP_TYPE_NONE) {
		dpp_drm_printf(p, dpp, "buffer doesn't have compressed data\n");
		return;
	}

	if (dpp->protection) {
		dpp_drm_printf(p, dpp, "dpp is protected\n");
		return;
	}

	plane_state = dpp->plane.base.state;
	if (!plane_state || !plane_state->fb) {
		dpp_drm_printf(p, dpp, "framebuffer not found\n");
		return;
	}

	fb = plane_state->fb;
	drm_framebuffer_get(fb);

	vaddr = exynos_drm_fb_to_vaddr(fb);
	if (vaddr) {
		dpp_drm_printf(p, dpp,
				"=== buffer dump[%s:%s]: dpp%d dma addr 0x%pad, vaddr 0x%p ===\n",
				get_comp_type_str(dpp->win_config.comp_type),
				get_comp_src_name(dpp->comp_src),
				dpp->id, &dpp->win_config.addr[0], vaddr);
		dpu_print_hex_dump(p, NULL, vaddr, 256);
	} else {
		dpp_drm_printf(p, dpp, "unable to find vaddr\n");
	}

	drm_framebuffer_put(fb);

	exynos_plane_state = to_exynos_plane_state(plane_state);
	old_fb = exynos_plane_state->old_fb;
	if (old_fb && old_fb != fb) {
		drm_framebuffer_get(old_fb);
		vaddr = exynos_drm_fb_to_vaddr(old_fb);
		if (vaddr) {
			dma_addr_t dma_addr = exynos_drm_fb_dma_addr(old_fb, 0);
			u64 comp_src = old_fb->modifier & AFBC_FORMAT_MOD_SOURCE_MASK;
			u64 comp_type;

			if (has_all_bits(DRM_FORMAT_MOD_ARM_AFBC(0), fb->modifier))
				comp_type = COMP_TYPE_AFBC;
			else if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0), fb->modifier))
				comp_type = COMP_TYPE_SBWC;
			else
				comp_type = COMP_TYPE_NONE;

			dpp_drm_printf(p, dpp,
				"=== old buffer dump[%s:%s]: dma addr 0x%pad, vaddr 0x%p ===\n",
				get_comp_type_str(comp_type),
				get_comp_src_name(comp_src),
				&dma_addr, vaddr);
			dpu_print_hex_dump(p, NULL, vaddr, 256);
		} else {
			dpp_drm_printf(p, dpp, "unable to find vaddr for old buffer\n");
		}
		drm_framebuffer_put(old_fb);
	}
}

void dpp_dump(struct drm_printer *p, struct dpp_device *dpp)
{
	if (dpp->state != DPP_STATE_ON) {
		dpp_drm_printf(p, dpp, "dpp state is off\n");
		return;
	}
	__dpp_dump(p, dpp->id, dpp->regs.dpp_base_regs, dpp->regs.dma_base_regs,
		   dpp->regs.sramc_base_regs, dpp->regs.hdr_comm_base_regs,
		   dpp->regs.hdr_base_regs, dpp->attr);
}

void rcd_dump(struct drm_printer *p, struct dpp_device *dpp)
{
	if (dpp->state != DPP_STATE_ON) {
		dpp_drm_printf(p, dpp, "rcd state is off\n");
		return;
	}
	__rcd_dump(p, dpp->id, dpp->regs.dpp_base_regs, dpp->regs.dma_base_regs,
			dpp->attr);
}

void cgc_dump(struct drm_printer *p, struct exynos_dma *dma)
{
	__cgc_dump(p, dma->id, dma->regs);
}

static bool is_fmt_fp16(const struct dpu_fmt *fmt_info)
{

	if (fmt_info && (fmt_info->fmt == DRM_FORMAT_ARGB16161616F ||
			fmt_info->fmt == DRM_FORMAT_ABGR16161616F))
		return true;

	return false;
}

bool dpp_need_enable_hdr(const struct dpp_device *dpp)
{
	bool hdr_en;

	if (unlikely(!dpp))
		return false;

	hdr_en = (dpp->hdr.state.eotf_lut || dpp->hdr.state.oetf_lut ||
			dpp->hdr.state.gm || dpp->hdr.state.tm);
	if (hdr_en) {
		/*
		 * In ZUMA, when enabling HDR, need to enable EOTF or FP16/FP16_CVT. Otherwise HDR
		 * engine canʼt work normally cause DPU stuck.
		 */
		const struct dpu_fmt *fmt_info = dpu_find_fmt_info(dpp->win_config.format);

		if (!is_fmt_fp16(fmt_info) && !dpp->hdr.state.eotf_lut) {
			printk_ratelimited(
				"DPP%u: when enabling HDR, EOTF_EN need to be set(GM %s TM %s OETF %s)\n",
				dpp->id,
				dpp->hdr.state.gm ? "on" : "off",
				dpp->hdr.state.tm ? "on" : "off",
				dpp->hdr.state.oetf_lut ? "on" : "off");
#if IS_ENABLED(CONFIG_SOC_ZUMA)
			hdr_en = false;
#endif
		} else if (is_fmt_fp16(fmt_info) &&
				!(dpp->hdr.fp16_en || dpp->hdr.fp16_cvt_en)) {
			printk_ratelimited(
				"DPP%u: when enabling HDR for FP16, FP16 or CVT need to be set(GM %s TM %s OETF %s)\n",
				dpp->id,
				dpp->hdr.state.gm ? "on" : "off",
				dpp->hdr.state.tm ? "on" : "off",
				dpp->hdr.state.oetf_lut ? "on" : "off");
#if IS_ENABLED(CONFIG_SOC_ZUMA)
			hdr_en = false;
#endif
		}
	}

	return hdr_en;
}

static dma_addr_t dpp_alloc_map_buf_test(void)
{
	struct dma_heap *dma_heap;
	struct dma_buf *buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sg_table;
	size_t size;
	struct iosys_map map;
	dma_addr_t dma_addr;
	struct decon_device *decon = get_decon_drvdata(0);
	struct drm_device *drm_dev = decon->drm_dev;
	struct exynos_drm_private *priv = drm_to_exynos_dev(drm_dev);
	int ret;

	size = PAGE_ALIGN(1440 * 3040 * 4);
	dma_heap = dma_heap_find("system");
	if (!dma_heap) {
		pr_err("Failed to find DMA-BUF system heap\n");
		return -EINVAL;
	}

	buf = dma_heap_buffer_alloc(dma_heap, size, O_RDWR, 0);
	dma_heap_put(dma_heap);
	if (IS_ERR(buf)) {
		pr_err("Failed to allocate %#zx bytes from DMA-BUF system heap\n", size);
		return PTR_ERR(buf);
	}

	ret = dma_buf_vmap(buf, &map);
	if (ret) {
		pr_err("failed to vmap buffer\n");
		dma_buf_put(buf);
		return -EINVAL;
	}

	memset(map.vaddr, 0x80, size);
	dma_buf_vunmap(buf, &map);

	/* mapping buffer for translating to DVA */
	attachment = dma_buf_attach(buf, priv->iommu_client);
	if (IS_ERR_OR_NULL(attachment)) {
		pr_err("failed to attach dma_buf\n");
		dma_buf_put(buf);
		return -EINVAL;
	}

	sg_table = dma_buf_map_attachment(attachment, DMA_TO_DEVICE);
	if (IS_ERR_OR_NULL(sg_table)) {
		pr_err("failed to map attachment\n");
		dma_buf_put(buf);
		return -EINVAL;
	}

	dma_addr = sg_dma_address(sg_table->sgl);
	if (IS_ERR_VALUE(dma_addr)) {
		pr_err("failed to map iovmm\n");
		dma_buf_put(buf);
		return -EINVAL;
	}

	return dma_addr;
}

__maybe_unused
static void dpp_test_fixed_config_params(struct dpp_params_info *config, u32 w,
		u32 h)
{
	config->src.x = 0;
	config->src.y = 0;
	config->src.w = w;
	config->src.h = h;
	config->src.f_w = w;
	config->src.f_h = h;

	config->dst.x = 0;
	config->dst.y = 0;
	config->dst.w = w;
	config->dst.h = h;
	/* TODO: This hard coded value will be changed */
	config->dst.f_w = w;
	config->dst.f_h = h;

	config->rot = 0; /* no rotation */
	config->comp_type = COMP_TYPE_NONE;
	config->format = DRM_FORMAT_BGRA8888;

	/* TODO: how to handle ? ... I don't know ... */
	config->addr[0] = dpp_alloc_map_buf_test();

	config->max_luminance = 0;
	config->min_luminance = 0;
	config->y_hd_y2_stride = 0;
	config->y_pl_c2_stride = 0;

	config->h_ratio = mult_frac(1 << 20, config->src.w, config->dst.w);
	config->v_ratio = mult_frac(1 << 20, config->src.h, config->dst.h);

	config->is_block = false;
#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
	config->rcv_num = exynos_devfreq_get_domain_freq(DEVFREQ_DISP) ? : 0x7FFFFFFF;
#else
	config->rcv_num = 0x7FFFFFFF;
#endif
}

static int dpp_convert_plane_state_to_config(struct dpp_params_info *config,
				const struct exynos_drm_plane_state *state,
				const struct drm_display_mode *mode)
{
	struct drm_framebuffer *fb = state->base.fb;
	unsigned int simplified_rot;
	unsigned long long comp_blk_size;

	pr_debug("mode(%dx%d)\n", mode->hdisplay, mode->vdisplay);
	config->src.x = state->base.src.x1 >> 16;
	config->src.y = state->base.src.y1 >> 16;
	config->src.w = drm_rect_width(&state->base.src) >> 16;
	config->src.h = drm_rect_height(&state->base.src) >> 16;
	config->src.f_w = fb->width;
	config->src.f_h = fb->height;

	config->dst.x = state->base.dst.x1;
	config->dst.y = state->base.dst.y1;
	config->dst.w = drm_rect_width(&state->base.dst);
	config->dst.h = drm_rect_height(&state->base.dst);
	config->dst.f_w = mode->hdisplay;
	config->dst.f_h = mode->vdisplay;
	config->rot = 0;
	simplified_rot = drm_rotation_simplify(state->base.rotation,
			DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
			DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);
	if (simplified_rot & DRM_MODE_ROTATE_90)
		config->rot |= DPP_ROT;
	if (simplified_rot & DRM_MODE_REFLECT_X)
		config->rot |= DPP_X_FLIP;
	if (simplified_rot & DRM_MODE_REFLECT_Y)
		config->rot |= DPP_Y_FLIP;

	if (has_all_bits(DRM_FORMAT_MOD_ARM_AFBC(0), fb->modifier)) {
		config->comp_type = COMP_TYPE_AFBC;
		comp_blk_size = AFBC_BLOCK_SIZE_GET(fb->modifier);
		switch (comp_blk_size) {
		case AFBC_FORMAT_MOD_BLOCK_SIZE_16x16:
			config->blk_size = AFBC_BLK_16x16;
			break;
		case AFBC_FORMAT_MOD_BLOCK_SIZE_32x8:
			config->blk_size = AFBC_BLK_32x8;
			break;
		case AFBC_FORMAT_MOD_BLOCK_SIZE_64x4:
			config->blk_size = AFBC_BLK_64x4;
			break;
		default:
			pr_err("AFBC Block Size(%llu) is not supported\n", comp_blk_size);
			return -EINVAL;
		}
	} else if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0), fb->modifier)) {
		config->comp_type = COMP_TYPE_SBWC;
		comp_blk_size = SBWC_BLOCK_SIZE_GET(fb->modifier);
		switch (comp_blk_size) {
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x2:
			config->blk_size = SBWC_BLK_32x2;
			break;
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x3:
			config->blk_size = SBWC_BLK_32x3;
			break;
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x4:
			config->blk_size = SBWC_BLK_32x4;
			break;
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x5:
			config->blk_size = SBWC_BLK_32x5;
			break;
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x6:
			config->blk_size = SBWC_BLK_32x6;
			break;
		default:
			pr_err("SBWC Block Size(%llu) is not supported\n", comp_blk_size);
			return -EINVAL;
		}
	} else {
		config->comp_type = COMP_TYPE_NONE;
	}

	config->is_lossy = has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0) |
			SBWC_FORMAT_MOD_LOSSY, fb->modifier) ? true : false;

	config->format = fb->format->format;
	config->standard = state->standard;
	config->transfer = state->transfer;
	config->range = state->range;
	config->max_luminance = state->max_luminance;
	config->min_luminance = state->min_luminance;
	config->y_hd_y2_stride = 0;
	config->y_pl_c2_stride = 0;
	config->c_hd_stride = 0;
	config->c_pl_stride = 0;

	config->addr[0] = exynos_drm_fb_dma_addr(fb, 0);
	config->addr[1] = exynos_drm_fb_dma_addr(fb, 1);
	if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_YUV_8_2_SPLIT, fb->modifier)) {
		config->addr[2] = config->addr[0] +
			NV12N_10B_Y_8B_SIZE(fb->width, fb->height);
		config->addr[3] = config->addr[1] +
			NV12N_10B_CBCR_8B_SIZE(fb->width, fb->height);
	} else if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0), fb->modifier)) {
		const struct dpu_fmt *fmt_info =
			dpu_find_fmt_info(config->format);
		bool is_10bpc = IS_10BPC(fmt_info);
		/* Luminance header */
		config->addr[0] += Y_PL_SIZE_SBWC(config->src.f_w,
				config->src.f_h, is_10bpc);
		config->y_hd_y2_stride = HD_STRIDE_SIZE_SBWC(config->src.f_w);

		/* Luminance payload */
		config->addr[1] = exynos_drm_fb_dma_addr(fb, 0);
		config->y_pl_c2_stride = PL_STRIDE_SIZE_SBWC(config->src.f_w,
				is_10bpc);

		/* Chrominance header */
		config->addr[2] = exynos_drm_fb_dma_addr(fb, 1) +
			UV_PL_SIZE_SBWC(config->src.f_w, config->src.f_h,
					is_10bpc);
		config->c_hd_stride = HD_STRIDE_SIZE_SBWC(config->src.f_w);

		/* Chrominance payload */
		config->addr[3] = exynos_drm_fb_dma_addr(fb, 1);
		config->c_pl_stride = PL_STRIDE_SIZE_SBWC(config->src.f_w,
				is_10bpc);
	} else {
		config->addr[2] = exynos_drm_fb_dma_addr(fb, 2);
		config->addr[3] = exynos_drm_fb_dma_addr(fb, 3);
	}

	if (config->rot & DPP_ROT) {
		config->h_ratio = mult_frac(1 << 20, config->src.h, config->dst.w);
		config->v_ratio = mult_frac(1 << 20, config->src.w, config->dst.h);
	} else {
		config->h_ratio = mult_frac(1 << 20, config->src.w, config->dst.w);
		config->v_ratio = mult_frac(1 << 20, config->src.h, config->dst.h);
	}

	if ((config->h_ratio != (1 << 20)) || (config->v_ratio != (1 << 20)))
		config->is_scale = true;
	else
		config->is_scale = false;

	if (state->block) {
		struct decon_win_rect *block = (struct decon_win_rect *)state->block->data;
		config->block.x = block->x;
		config->block.y = block->y;
		config->block.w = block->w;
		config->block.h = block->h;
		config->is_block = config->block.w > 0 && config->block.h > 0;
	} else {
		config->is_block = false;
	}
#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
	config->rcv_num = exynos_devfreq_get_domain_freq(DEVFREQ_DISP) ? : 0x7FFFFFFF;
#else
	config->rcv_num = 0x7FFFFFFF;
#endif

	return 0;
}

static void __dpp_enable(struct dpp_device *dpp)
{
	if (dpp->state == DPP_STATE_ON)
		return;

	dpp_reg_init(dpp->id, dpp->attr);

	dpp->state = DPP_STATE_ON;
	enable_irq(dpp->dma_irq);
	if (test_bit(DPP_ATTR_DPP, &dpp->attr))
		enable_irq(dpp->dpp_irq);

	dpp_debug(dpp, "enabled\n");
}

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)

static void set_resource_protection(bool dpu_protected)
{
	u32 decon_id;
	u32 dqe_id;
	u32 dsim_id;

	pr_debug("%s DPU protection status changed to %d", __func__, dpu_protected);
	for (decon_id = 0; decon_id < MAX_DECON_CNT; ++decon_id)
		decon_reg_set_drm_write_protected(decon_id, dpu_protected);
	for (dqe_id = 0; dqe_id < REGS_DQE_ID_MAX; ++dqe_id)
		dqe_reg_set_drm_write_protected(dqe_id, dpu_protected);
	for (dsim_id = 0; dsim_id < MAX_DSI_CNT; ++dsim_id)
		dsim_reg_set_drm_write_protected(dsim_id, dpu_protected);
}

static void update_secured_dpp_mask(struct dpp_device *dpp_to_change, bool enable) {
	const struct drm_device *drm_dev;
	struct exynos_drm_private *private;
	u32 dpp_mask = BIT(dpp_to_change->id);

	drm_dev = dpp_to_change->plane.base.dev;
	private = drm_to_exynos_dev(drm_dev);
	if (enable)
		private->secured_dpp_mask |= dpp_mask;
	else
		private->secured_dpp_mask &= ~dpp_mask;
}

static bool update_resource_protection(struct dpp_device *dpp_to_change, bool enable)
{
	const struct drm_device *drm_dev;
	const struct exynos_drm_private *private;
	u32 dpp_mask = BIT(dpp_to_change->id);
	u32 secure_mask;

	drm_dev = dpp_to_change->plane.base.dev;
	private = drm_to_exynos_dev(drm_dev);
	secure_mask = private->secured_dpp_mask;

	if (enable)
		secure_mask |= dpp_mask;
	else
		secure_mask &= ~dpp_mask;

	if ((secure_mask && !private->secured_dpp_mask) ||
				(!secure_mask && private->secured_dpp_mask)) {
		set_resource_protection(secure_mask);
		return true;
	}

	/* resource protection flag is not changed */
	return false;
}

static int set_protection(struct dpp_device *dpp, uint64_t modifier)
{
	bool protection;
	u32 protection_id;
	int ret = 0;
	bool res_protection_changed = false;
#if defined(CONFIG_SOC_ZUMA)
	static const u32 protection_ids[] = { PROT_L0,	PROT_L1,  PROT_L2,  PROT_L3, PROT_L4,
					      PROT_L5,	PROT_L6,  PROT_L8,  PROT_L9, PROT_L10,
					      PROT_L11, PROT_L12, PROT_L13, PROT_L14 };
#else
	static const u32 protection_ids[] = { PROT_L0, PROT_L1, PROT_L2,
					PROT_L3, PROT_L4, PROT_L5, PROT_L12 };
#endif

	protection = (modifier & DRM_FORMAT_MOD_PROTECTION) != 0;
	if (dpp->protection == protection)
		return ret;

	if (dpp->id >= ARRAY_SIZE(protection_ids)) {
		dpp_err(dpp, "failed to get protection id(%u)\n", dpp->id);
		return -EINVAL;
	}

        /* Forward some register update to el3 if transit to protection mode */

	if (protection)
		res_protection_changed = update_resource_protection(dpp, true);

	protection_id = protection_ids[dpp->id];
	ret = exynos_smc(SMC_PROTECTION_SET, 0, protection_id,
			(protection ? SMC_PROTECTION_ENABLE :
			SMC_PROTECTION_DISABLE));
	WARN(dma_reg_is_mst_security_enabled(dpp->id, &dpp->rdma_mst_security) != !protection,
						"dpp[%u] mst_security: %#x\n",
						dpp->id, dpp->rdma_mst_security);
	DPU_EVENT_LOG(DPU_EVT_DPP_SET_PROTECTION, dpp->decon_id, dpp);
	if (ret) {
		dpp_err(dpp, "failed to %s protection(ch:%u, ret:%d)\n",
				protection ? "enable" : "disable", dpp->id, ret);
		if (protection && res_protection_changed)
			set_resource_protection(false);
		return ret;
	}
	/* Stop forwarding registers update to el3 if transit to none protection mode */
	if (!protection)
		update_resource_protection(dpp, false);
	update_secured_dpp_mask(dpp, protection);

	dpp->protection = protection;

	dpp_debug(dpp, "ch:%u, en:%d\n", dpp->id, protection);

	return ret;
}
#else
static inline int
set_protection(struct dpp_device *dpp, uint64_t modifier) { return 0; }
#endif

static void __dpp_disable(struct dpp_device *dpp)
{
	if (dpp->state == DPP_STATE_OFF)
		return;

	if (dpp->hdr.state.eotf_lut) {
		dpp->hdr.state.eotf_lut = NULL;
		hdr_reg_set_eotf_lut(dpp->id, NULL);
	}

	if (dpp->hdr.state.oetf_lut) {
		dpp->hdr.state.oetf_lut = NULL;
		hdr_reg_set_oetf_lut(dpp->id, NULL);
	}

	if (dpp->hdr.state.gm) {
		dpp->hdr.state.gm = NULL;
		hdr_reg_set_gm(dpp->id, NULL);
	}

	if (dpp->hdr.state.tm) {
		dpp->hdr.state.tm = NULL;
		hdr_reg_set_tm(dpp->id, NULL);
	}

	if (test_bit(DPP_ATTR_DPP, &dpp->attr))
		disable_irq_nosync(dpp->dpp_irq);
	disable_irq_nosync(dpp->dma_irq);

	dpp_reg_deinit(dpp->id, false, dpp->attr);

	set_protection(dpp, 0);
	dpp->state = DPP_STATE_OFF;
	dpp->decon_id = -1;

	dpp_debug(dpp, "disabled\n");
}

static int dpp_disable(struct dpp_device *this_dpp)
{
	__dpp_disable(this_dpp);

	return 0;
}

static int dpp_check_scale(struct dpp_device *dpp,
			struct dpp_params_info *config)
{
	struct dpp_restriction *res;
	struct decon_frame *src, *dst;
	u32 src_w, src_h;

	res = &dpp->restriction;
	src = &config->src;
	dst = &config->dst;

	if (config->rot & DPP_ROT) {
		src_w = src->h;
		src_h = src->w;
	} else {
		src_w = src->w;
		src_h = src->h;
	}

	/* If scaling is not requested, it doesn't need to check limitation */
	if ((src_w == dst->w) && (src_h == dst->h))
		return 0;

#if !defined(CONFIG_SOC_ZUMA)
	/* Scaling is requested. need to check limitation */
	if (!test_bit(DPP_ATTR_CSC, &dpp->attr)) {
		dpp_err(dpp, "not support CSC\n");
		return -ENOTSUPP;
	}
#endif

	if ((src_w > dst->w * res->scale_down) ||
			(src_h > dst->h * res->scale_down)) {
		dpp_err(dpp, "not support under 1/%dx scale-down\n",
				res->scale_down);
		return -ENOTSUPP;
	}

	if ((src_w * res->scale_up < dst->w) ||
			(src_h * res->scale_up < dst->h)) {
		dpp_err(dpp, "not support over %dx scale-up\n", res->scale_up);
		return -ENOTSUPP;
	}

	return 0;
}

static int dpp_check_size(struct dpp_device *dpp,
			struct dpp_params_info *config)
{
	struct decon_frame *src, *dst;
	const struct dpu_fmt *fmt_info;
	struct dpp_restriction *res;
	u32 mul = 1; /* factor to multiply alignment */
	u32 src_h_max;

	fmt_info = dpu_find_fmt_info(config->format);

	if (IS_YUV(fmt_info))
		mul = 2;

	res = &dpp->restriction;
	src = &config->src;
	dst = &config->dst;

	if (config->rot & DPP_ROT)
		src_h_max = res->src_h_rot_max;
	else
		src_h_max = res->src_h.max;

	/* check alignment */
	if (!IS_ALIGNED(src->x, res->src_x_align * mul) ||
			!IS_ALIGNED(src->y, res->src_y_align * mul) ||
			!IS_ALIGNED(src->w, res->src_w.align * mul) ||
			!IS_ALIGNED(src->h, res->src_h.align * mul) ||
			!IS_ALIGNED(src->f_w, res->src_f_w.align * mul) ||
			!IS_ALIGNED(src->f_h, res->src_f_h.align * mul)) {
		dpp_err(dpp, "not supported source alignment\n");
		return -ENOTSUPP;
	}

	if (!IS_ALIGNED(dst->x, res->dst_x_align) ||
			!IS_ALIGNED(dst->y, res->dst_y_align) ||
			!IS_ALIGNED(dst->w, res->dst_w.align) ||
			!IS_ALIGNED(dst->h, res->dst_h.align) ||
			!IS_ALIGNED(dst->f_w, res->dst_f_w.align) ||
			!IS_ALIGNED(dst->f_h, res->dst_f_h.align)) {
		dpp_err(dpp, "not supported destination alignment\n");
		return -ENOTSUPP;
	}

	/* check range */
	if (!IN_RANGE(src->w, res->src_w.min * mul, res->src_w.max) ||
			!IN_RANGE(src->h, res->src_h.min * mul, src_h_max) ||
			!IN_RANGE(src->f_w, res->src_f_w.min * mul,
						res->src_f_w.max) ||
			!IN_RANGE(src->f_h, res->src_f_h.min,
						res->src_f_h.max)) {
		dpp_err(dpp, "not supported source size range\n");
		return -ENOTSUPP;
	}

	if (!IN_RANGE(dst->w, res->dst_w.min, res->dst_w.max) ||
			!IN_RANGE(dst->h, res->dst_h.min, res->dst_h.max) ||
			!IN_RANGE(dst->f_w, res->dst_f_w.min,
						res->dst_f_w.max) ||
			!IN_RANGE(dst->f_h, res->dst_f_h.min,
						res->dst_f_h.max)) {
		dpp_err(dpp, "not supported destination size range\n");
		return -ENOTSUPP;
	}

	return 0;
}

static int dpp_check_dst_size(struct dpp_device *dpp,
			struct dpp_params_info *config)
{
	struct decon_frame *dst;
	struct dpp_restriction *res;

	res = &dpp->restriction;
	dst = &config->dst;

	/* check range */
	if (!IN_RANGE(dst->w, res->dst_w.min, res->dst_w.max) ||
			!IN_RANGE(dst->h, res->dst_h.min, res->dst_h.max) ||
			!IN_RANGE(dst->f_w, res->dst_f_w.min,
						res->dst_f_w.max) ||
			!IN_RANGE(dst->f_h, res->dst_f_h.min,
						res->dst_f_h.max)) {
		dpp_err(dpp, "not supported destination size range\n");
		return -ENOTSUPP;
	}

	return 0;
}

static int dpp_check(struct dpp_device *dpp,
		const struct exynos_drm_plane_state *state)
{
	struct dpp_params_info config;
	const struct dpu_fmt *fmt_info;
	const struct drm_plane_state *plane_state = &state->base;
	const struct drm_crtc_state *crtc_state =
			drm_atomic_get_new_crtc_state(plane_state->state,
							plane_state->crtc);
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	const struct drm_framebuffer *fb = state->base.fb;
	int ret = 0;

	dpp_debug(dpp, "+\n");

	memset(&config, 0, sizeof(struct dpp_params_info));

	ret = dpp_convert_plane_state_to_config(&config, state, mode);
	if (ret)
		return ret;

	if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_COLORMAP, fb->modifier)) {
		if (dpp_check_dst_size(dpp, &config))
			goto err;

		return 0;
	}

	if (dpp_check_scale(dpp, &config))
		goto err;

	if (dpp_check_size(dpp, &config))
		goto err;

	fmt_info = dpu_find_fmt_info(config.format);
	if ((config.rot & DPP_ROT) && (!IS_YUV420(fmt_info))) {
		dpp_err(dpp, "support rotation only for YUV420 format\n");
		goto err;
	}

	if (!test_bit(DPP_ATTR_AFBC, &dpp->attr) &&
			(config.comp_type == COMP_TYPE_AFBC)) {
		dpp_err(dpp, "not support AFBC\n");
		goto err;
	}

	if (__dpp_check(dpp->id, &config, dpp->attr))
		goto err;

	dpp_debug(dpp, "-\n");

	return 0;

err:
	dpp_err(dpp, "src[%d %d %d %d %d %d] dst[%d %d %d %d %d %d] fmt[%d]\n",
			config.src.x, config.src.y, config.src.w, config.src.h,
			config.src.f_w, config.src.f_h,
			config.dst.x, config.dst.y, config.dst.w, config.dst.h,
			config.dst.f_w, config.dst.f_h,
			config.format);
	dpp_err(dpp, "rot[0x%x] comp_type[%d]\n", config.rot, config.comp_type);

	return -ENOTSUPP;
}

static void
exynos_eotf_update(struct dpp_device *dpp, struct exynos_drm_plane_state *state)
{
	struct eotf_debug_override *eotf = &dpp->hdr.eotf;
	struct exynos_debug_info *info = &eotf->info;
	struct drm_printer p = drm_info_printer(dpp->dev);

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->hdr_state.eotf_lut = &eotf->force_lut;

	if (dpp->hdr.state.eotf_lut != state->hdr_state.eotf_lut || info->dirty) {
		hdr_reg_set_eotf_lut(dpp->id, state->hdr_state.eotf_lut);
		dpp->hdr.state.eotf_lut = state->hdr_state.eotf_lut;
		info->dirty = false;
	}

	if (info->verbose)
		hdr_reg_print_eotf_lut(dpp->id, &p);
}

static void
exynos_oetf_update(struct dpp_device *dpp, struct exynos_drm_plane_state *state)
{
	struct oetf_debug_override *oetf = &dpp->hdr.oetf;
	struct exynos_debug_info *info = &oetf->info;
	struct drm_printer p = drm_info_printer(dpp->dev);

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->hdr_state.oetf_lut = &oetf->force_lut;

	if (dpp->hdr.state.oetf_lut != state->hdr_state.oetf_lut || info->dirty) {
		hdr_reg_set_oetf_lut(dpp->id, state->hdr_state.oetf_lut);
		dpp->hdr.state.oetf_lut = state->hdr_state.oetf_lut;
		info->dirty = false;
	}

	if (info->verbose)
		hdr_reg_print_oetf_lut(dpp->id, &p);
}

static void
exynos_gm_update(struct dpp_device *dpp, struct exynos_drm_plane_state *state)
{
	struct gm_debug_override *gm = &dpp->hdr.gm;
	struct exynos_debug_info *info = &gm->info;
	struct drm_printer p = drm_info_printer(dpp->dev);

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->hdr_state.gm = &gm->force_data;

	if (dpp->hdr.state.gm != state->hdr_state.gm || info->dirty) {
		hdr_reg_set_gm(dpp->id, state->hdr_state.gm);
		dpp->hdr.state.gm = state->hdr_state.gm;
		info->dirty = false;
	}

	if (info->verbose)
		hdr_reg_print_gm(dpp->id, &p);
}

static void
exynos_tm_update(struct dpp_device *dpp, struct exynos_drm_plane_state *state)
{
	struct tm_debug_override *tm = &dpp->hdr.tm;
	struct exynos_debug_info *info = &tm->info;
	struct drm_printer p = drm_info_printer(dpp->dev);

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->hdr_state.tm = &tm->force_data;

	if (dpp->hdr.state.tm != state->hdr_state.tm || info->dirty) {
		hdr_reg_set_tm(dpp->id, state->hdr_state.tm);
		dpp->hdr.state.tm = state->hdr_state.tm;
		info->dirty = false;
	}

	if (info->verbose)
		hdr_reg_print_tm(dpp->id, &p);
}

static void dpp_hdr_update(struct dpp_device *dpp,
				struct exynos_drm_plane_state *state)
{
	bool enable = false;

	hdr_reg_set_fp16(dpp->id, dpp->hdr.fp16_en, dpp->hdr.fp16_cvt_en);
	exynos_eotf_update(dpp, state);
	exynos_oetf_update(dpp, state);
	exynos_gm_update(dpp, state);
	exynos_tm_update(dpp, state);

	enable = dpp_need_enable_hdr(dpp);

	hdr_reg_set_hdr(dpp->id, enable);
}

static int dpp_update(struct dpp_device *dpp,
			struct exynos_drm_plane_state *state)
{
	struct dpp_params_info *config = &dpp->win_config;
	const struct drm_plane_state *plane_state = &state->base;
	const struct drm_crtc_state *crtc_state = plane_state->crtc->state;
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	const struct exynos_drm_crtc_state *exynos_crtc_state =
					to_exynos_crtc_state(crtc_state);
	int ret = 0;

	dpp_debug(dpp, "+\n");

	__dpp_enable(dpp);

	ret = dpp_convert_plane_state_to_config(config, state, mode);
	if (ret)
		return ret;

	config->in_bpc = exynos_crtc_state->in_bpc == 8 ? DPP_BPC_8 : DPP_BPC_10;
	dpp_debug(dpp, "in/force bpc(%d/%d)\n", exynos_crtc_state->in_bpc,
			exynos_crtc_state->force_bpc);

	if (test_bit(DPP_ATTR_HDR, &dpp->attr))
		dpp_hdr_update(dpp, state);

	set_protection(dpp, plane_state->fb->modifier);

	dpp_reg_configure_params(dpp->id, config, dpp->attr);

	dpp_debug(dpp, "-\n");

	return 0;
}

static int dpp_bind(struct device *dev, struct device *master, void *data)
{
	struct dpp_device *dpp = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_plane_config plane_config;
	int ret = 0;
	int id = dpp->id;

	dpp_debug(dpp, "+\n");

	memset(&plane_config, 0, sizeof(plane_config));

	plane_config.pixel_formats = dpp->pixel_formats;
	plane_config.num_pixel_formats = dpp->num_pixel_formats;
	plane_config.format_modifiers = dpp->format_modifiers;
	plane_config.zpos = id;
	plane_config.type = get_decon_drvdata(id) ? DRM_PLANE_TYPE_PRIMARY :
		DRM_PLANE_TYPE_OVERLAY;

	if (dpp->is_support & DPP_SUPPORT_AFBC)
		plane_config.capabilities |=
			EXYNOS_DRM_PLANE_CAP_AFBC;

	ret = exynos_plane_init(drm_dev, &dpp->plane, id,
			&plane_config);
	if (ret)
		return ret;

	dpp_debug(dpp, "-\n");

	return 0;
}


static void dpp_unbind(struct device *dev, struct device *master, void *data)
{
	struct dpp_device *dpp = dev_get_drvdata(dev);

	if (dpp->state == DPP_STATE_ON)
		dpp_disable(dpp);
}

static const struct component_ops exynos_dpp_component_ops = {
	.bind	= dpp_bind,
	.unbind	= dpp_unbind,
};

static void dpp_print_restriction(struct dpp_device *dpp)
{
	struct dpp_restriction *res = &dpp->restriction;

	dpp_info(dpp, "src_f_w[%d %d %d] src_f_h[%d %d %d]\n",
			res->src_f_w.min, res->src_f_w.max, res->src_f_w.align,
			res->src_f_h.min, res->src_f_h.max, res->src_f_h.align);
	dpp_info(dpp, "src_w[%d %d %d] src_h[%d %d %d] src_x_y_align[%d %d]\n",
			res->src_w.min, res->src_w.max, res->src_w.align,
			res->src_h.min, res->src_h.max, res->src_h.align,
			res->src_x_align, res->src_y_align);

	dpp_info(dpp, "dst_f_w[%d %d %d] dst_f_h[%d %d %d]\n",
			res->dst_f_w.min, res->dst_f_w.max, res->dst_f_w.align,
			res->dst_f_h.min, res->dst_f_h.max, res->dst_f_h.align);
	dpp_info(dpp, "dst_w[%d %d %d] dst_h[%d %d %d] dst_x_y_align[%d %d]\n",
			res->dst_w.min, res->dst_w.max, res->dst_w.align,
			res->dst_h.min, res->dst_h.max, res->dst_h.align,
			res->dst_x_align, res->dst_y_align);

	dpp_info(dpp, "blk_w[%d %d %d] blk_h[%d %d %d] blk_x_y_align[%d %d]\n",
			res->blk_w.min, res->blk_w.max, res->blk_w.align,
			res->blk_h.min, res->blk_h.max, res->blk_h.align,
			res->blk_x_align, res->blk_y_align);

	dpp_info(dpp, "src_h_rot_max[%d]\n", res->src_h_rot_max);

	dpp_info(dpp, "max scale up(%dx), down(1/%dx) ratio\n", res->scale_up,
			res->scale_down);
}

static int exynos_dpp_parse_dt(struct dpp_device *dpp, struct device_node *np)
{
	int ret = 0;
	struct dpp_restriction *res = &dpp->restriction;

	ret = of_property_read_u32(np, "dpp,id", &dpp->id);
	if (ret < 0)
		goto fail;

	of_property_read_u32(np, "attr", (u32 *)&dpp->attr);
	of_property_read_u32(np, "port", &dpp->port);

	if (of_property_read_bool(np, "dpp,video"))
		dpp->is_support |= DPP_SUPPORT_VIDEO;

	if (dpp->is_support & DPP_SUPPORT_VIDEO) {
		dpp->pixel_formats = dpp_vg_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(dpp_vg_formats);
	} else if (test_bit(DPP_ATTR_RCD, &dpp->attr)) {
		dpp->pixel_formats = rcd_alpha_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(rcd_alpha_formats);
	} else {
		dpp->pixel_formats = dpp_gf_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(dpp_gf_formats);
	}

	dpp->format_modifiers = dpp_format_modifiers;

	of_property_read_u32(np, "scale_down", (u32 *)&res->scale_down);
	of_property_read_u32(np, "scale_up", (u32 *)&res->scale_up);

	dpp_info(dpp, "attr(0x%lx), port(%d)\n", dpp->attr, dpp->port);

	dpp_print_restriction(dpp);

	return 0;
fail:
	return ret;
}

static irqreturn_t dpp_irq_handler(int irq, void *priv)
{
	struct dpp_device *dpp = priv;

	spin_lock(&dpp->slock);
	if (dpp->state == DPP_STATE_OFF)
		goto irq_end;

	dpp_reg_get_irq_and_clear(dpp->id);

irq_end:
	spin_unlock(&dpp->slock);
	return IRQ_HANDLED;
}

static irqreturn_t dma_irq_handler(int irq, void *priv)
{
	struct dpp_device *dpp = priv;
	u32 irqs;
	const char *str_comp;
	bool dump = false;
	static unsigned long last_dumptime;

	spin_lock(&dpp->dma_slock);
	if (dpp->state == DPP_STATE_OFF)
		goto irq_end;

	/* IDMA case */
	irqs = idma_reg_get_irq_and_clear(dpp->id);

	if (irqs & IDMA_RECOVERY_START_IRQ) {
		DPU_EVENT_LOG(DPU_EVT_DMA_RECOVERY, dpp->decon_id, dpp);
		dpp->recovery_cnt++;
		str_comp = get_comp_src_name(dpp->comp_src);
		dpp_info(dpp, "recovery start(0x%x) cnt(%d) src(%s)\n", irqs,
				dpp->recovery_cnt, str_comp);
	}

	/*
	 * TODO: Normally, DMA framedone occurs before DPP framedone.
	 * But DMA framedone can occur in case of AFBC crop mode
	 */
	if (irqs & IDMA_STATUS_FRAMEDONE_IRQ) {
		DPU_EVENT_LOG(DPU_EVT_DPP_FRAMEDONE, dpp->decon_id,
				dpp);
	}

	if (irqs & IDMA_FBC_ERR_IRQ) {
		DPU_EVENT_LOG(DPU_EVT_IDMA_FBC_ERROR, dpp->decon_id, dpp);
		dump = true;
	}

	if (irqs & IDMA_READ_SLAVE_ERROR) {
		DPU_EVENT_LOG(DPU_EVT_IDMA_READ_SLAVE_ERROR, dpp->decon_id, dpp);
		dump = true;
	}

	if (irqs & IDMA_STATUS_DEADLOCK_IRQ) {
		DPU_EVENT_LOG(DPU_EVT_IDMA_DEADLOCK, dpp->decon_id, dpp);
		dump = true;
	}

	if (irqs & IDMA_CONFIG_ERR_IRQ) {
		DPU_EVENT_LOG(DPU_EVT_IDMA_CFG_ERROR, dpp->decon_id, dpp);
		dump = true;
	}

	if (dump && time_after(jiffies, last_dumptime + msecs_to_jiffies(5000))) {
		struct decon_device *decon = get_decon_drvdata(dpp->decon_id);
		if (decon) {
			if (decon_dump_ignore(DPU_EVT_CONDITION_IDMA_ERROR))
				decon_dump_event_condition(decon,
					DPU_EVT_CONDITION_IDMA_ERROR_COMPACT);
			else
				decon_dump_all(decon, DPU_EVT_CONDITION_IDMA_ERROR, true);
			last_dumptime = jiffies;

			if ((irqs & IDMA_STATUS_DEADLOCK_IRQ) &&
				(decon->config.mode.op_mode == DECON_COMMAND_MODE)) {
				char idma_dl_msg[40];

				scnprintf(idma_dl_msg, sizeof(idma_dl_msg),
					"dpp[%u] deadlock in decon[%u]\n", dpp->id, dpp->decon_id);
				dbg_snapshot_emergency_reboot(idma_dl_msg);
			}
		}
	}

irq_end:

	spin_unlock(&dpp->dma_slock);
	return IRQ_HANDLED;
}

static irqreturn_t cgc_irq_handler(int irq, void *priv)
{
	struct decon_device *decon = priv;
	struct exynos_dma *dma = decon->cgc_dma;
	u32 irqs;

	spin_lock(&dma->dma_slock);

	irqs = cgc_reg_get_irq_and_clear(dma->id);

	if (irqs & IDMA_STATUS_FRAMEDONE_IRQ)
		DPU_EVENT_LOG(DPU_EVT_CGC_FRAMEDONE, decon->id, NULL);

	spin_unlock(&dma->dma_slock);
	return IRQ_HANDLED;
}

static int dpp_init_resources(struct dpp_device *dpp)
{
	struct resource res;
	struct device *dev = dpp->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev;
	int i, ret = 0;

	pdev = container_of(dev, struct platform_device, dev);

	i = of_property_match_string(np, "reg-names", "dma");
	if (of_address_to_resource(np, i, &res)) {
		dpp_err(dpp, "failed to get dma resource\n");
		return -EINVAL;
	}
	dpp->regs.dma_base_regs = of_iomap(np, i);
	if (!dpp->regs.dma_base_regs) {
		dpp_err(dpp, "failed to remap DPU_DMA SFR region\n");
		return -EINVAL;
	}
	dpp_regs_desc_init(dpp->regs.dma_base_regs, res.start, "dma", REGS_DMA, dpp->id);

	dpp->dma_irq = of_irq_get_byname(np, "dma");
	dpp_info(dpp, "dma irq no = %d\n", dpp->dma_irq);
	ret = devm_request_irq(dev, dpp->dma_irq, dma_irq_handler, 0,
			pdev->name, dpp);
	if (ret) {
		dpp_err(dpp, "failed to install DPU DMA irq\n");
		return -EINVAL;
	}
	disable_irq(dpp->dma_irq);

	if (test_bit(DPP_ATTR_DPP, &dpp->attr)) {
		i = of_property_match_string(np, "reg-names", "dpp");
		if (of_address_to_resource(np, i, &res)) {
			dpp_err(dpp, "failed to get dpp resource\n");
			return -EINVAL;
		}
		dpp->regs.dpp_base_regs = of_iomap(np, i);
		if (!dpp->regs.dpp_base_regs) {
			dpp_err(dpp, "failed to remap DPP SFR region\n");
			return -EINVAL;
		}
		dpp_regs_desc_init(dpp->regs.dpp_base_regs, res.start, "dpp", REGS_DPP,
				dpp->id);

		dpp->dpp_irq = of_irq_get_byname(np, "dpp");
		dpp_info(dpp, "dpp irq no = %d\n", dpp->dpp_irq);
		ret = devm_request_irq(dev, dpp->dpp_irq, dpp_irq_handler, 0,
				pdev->name, dpp);
		if (ret) {
			dpp_err(dpp, "failed to install DPP irq\n");
			return -EINVAL;
		}
		disable_irq(dpp->dpp_irq);
	}

	if (test_bit(DPP_ATTR_SCL_COEF, &dpp->attr)) {
		i = of_property_match_string(np, "reg-names", "scl_coef");
		if (of_address_to_resource(np, i, &res)) {
			dpp_err(dpp, "failed to get scl_coef resource\n");
			return -EINVAL;
		}
		dpp->regs.scl_coef_base_regs = of_iomap(np, i);
		if (!dpp->regs.scl_coef_base_regs) {
			dpp_err(dpp, "failed to remap SCL COEF SFR region\n");
			return -EINVAL;
		}
		dpp_regs_desc_init(dpp->regs.scl_coef_base_regs, res.start, "scl_coef", REGS_SCL_COEF,
				dpp->id);
	}

	if (test_bit(DPP_ATTR_SRAMC, &dpp->attr)) {
		i = of_property_match_string(np, "reg-names", "sramc");
		if (of_address_to_resource(np, i, &res)) {
			dpp_err(dpp, "failed to get sramc resource\n");
			return -EINVAL;
		}
		dpp->regs.sramc_base_regs = of_iomap(np, i);
		if (!dpp->regs.sramc_base_regs) {
			dpp_err(dpp, "failed to remap SRAMC SFR region\n");
			return -EINVAL;
		}
		dpp_regs_desc_init(dpp->regs.sramc_base_regs, res.start, "sramc", REGS_SRAMC,
				dpp->id);
	}

	if (test_bit(DPP_ATTR_HDR, &dpp->attr) ||
			test_bit(DPP_ATTR_HDR10_PLUS, &dpp->attr)) {
		i = of_property_match_string(np, "reg-names", "hdr");
		if (of_address_to_resource(np, i, &res)) {
			dpp_err(dpp, "failed to get hdr resource\n");
			return -EINVAL;
		}
		dpp->regs.hdr_base_regs = of_iomap(np, i);
		if (!dpp->regs.hdr_base_regs) {
			dpp_err(dpp, "failed to remap HDR SFR region\n");
			return -EINVAL;
		}
		hdr_regs_desc_init(dpp->regs.hdr_base_regs, res.start, "hdr", dpp->id);
	}

	if (test_bit(DPP_ATTR_HDR_COMM, &dpp->attr)) {
		i = of_property_match_string(np, "reg-names", "hdr_comm");
		if (of_address_to_resource(np, i, &res)) {
			dpp_err(dpp, "failed to get hdr comm resource\n");
			return -EINVAL;
		}
		dpp->regs.hdr_comm_base_regs = of_iomap(np, i);
		if (!dpp->regs.hdr_comm_base_regs) {
			dpp_err(dpp, "failed to remap HDR COMM SFR region\n");
			return -EINVAL;
		}
		dpp_regs_desc_init(dpp->regs.hdr_comm_base_regs, res.start, "hdr_comm",
				   REGS_HDR_COMM, dpp->id);
	}
	ret = __dpp_init_resources(dpp);

	return ret;
}

struct exynos_dma *exynos_cgc_dma_register(struct decon_device *decon)
{
	struct device *dev = decon->dev;
	struct device_node *np = dev->of_node;
	struct exynos_dma *dma;
	struct platform_device *pdev;
	int i, ret = 0;
	struct resource res;

	pdev = container_of(dev, struct platform_device, dev);

	i = of_property_match_string(np, "reg-names", "cgc-dma");
	if (i < 0) {
		pr_debug("cgc-dma is not supported\n");
		return NULL;
	}

	if (of_address_to_resource(np, i, &res)) {
		pr_err("failed to get cgc dma resource\n");
		return NULL;
	}

	dma = devm_kzalloc(dev, sizeof(struct exynos_dma), GFP_KERNEL);
	if (!dma)
		return NULL;

	dma->regs = of_iomap(np, i);
	if (IS_ERR(dma->regs)) {
		pr_err("failed to remap cgc-dma registers\n");
		return NULL;
	}

	ret = of_property_read_u32(np, "cgc-dma,id", &dma->id);
	if (ret < 0) {
		pr_err("failed to get cgc-dma id\n");
		return NULL;
	}

	dpp_regs_desc_init(dma->regs, res.start, "cgc-dma", REGS_DMA, dma->id);

	spin_lock_init(&dma->dma_slock);
	dma->dma_irq = of_irq_get_byname(np, "cgc-dma");
	ret = devm_request_irq(dev, dma->dma_irq, cgc_irq_handler, 0,
			pdev->name, decon);
	if (ret) {
		pr_err("failed to install CGC DMA irq\n");
		return NULL;
	}

	pr_debug("cgc-dma is supported\n");

	return dma;
}

static int dpp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct dpp_device *dpp;
	const struct dpp_restriction *restriction;

	dpp = devm_kzalloc(dev, sizeof(struct dpp_device), GFP_KERNEL);
	if (!dpp)
		return -ENOMEM;

	restriction = of_device_get_match_data(dev);
	memcpy(&dpp->restriction, restriction, sizeof(*restriction));

	dpp->dev = dev;
	ret = exynos_dpp_parse_dt(dpp, dev->of_node);
	if (ret)
		goto fail;

	spin_lock_init(&dpp->slock);
	spin_lock_init(&dpp->dma_slock);

	dpp->state = DPP_STATE_OFF;

	ret = dpp_init_resources(dpp);
	if (ret)
		goto fail;

	dpp->check = dpp_check;
	dpp->update = dpp_update;
	dpp->disable = dpp_disable;
	/* dpp is not connected decon now */
	dpp->decon_id = -1;

	platform_set_drvdata(pdev, dpp);

	dpp_info(dpp, "successfully probed");

	return component_add(dev, &exynos_dpp_component_ops);

fail:
	dpp_err(dpp, "probe failed");

	return ret;
}

static int dpp_remove(struct platform_device *pdev)
{
	struct dpp_device *dpp = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &exynos_dpp_component_ops);

	if (test_bit(DPP_ATTR_HDR, &dpp->attr) ||
			test_bit(DPP_ATTR_HDR10_PLUS, &dpp->attr))
		iounmap(dpp->regs.hdr_base_regs);
	if (test_bit(DPP_ATTR_DPP, &dpp->attr))
		iounmap(dpp->regs.dpp_base_regs);
	if (test_bit(DPP_ATTR_SRAMC, &dpp->attr))
		iounmap(dpp->regs.sramc_base_regs);
	if (test_bit(DPP_ATTR_HDR_COMM, &dpp->attr))
		iounmap(dpp->regs.hdr_comm_base_regs);
	iounmap(dpp->regs.dma_base_regs);

	return 0;
}

struct platform_driver dpp_driver = {
	.probe = dpp_probe,
	.remove = dpp_remove,
	.driver = {
		   .name = "exynos-dpp",
		   .owner = THIS_MODULE,
		   .of_match_table = dpp_of_match,
	},
};

#ifdef CONFIG_OF
static int of_device_match(struct device *dev, const void *data)
{
	return dev->of_node == data;
}

struct dpp_device *of_find_dpp_by_node(struct device_node *np)
{
	struct device *dev;

	dev = bus_find_device(&platform_bus_type, NULL, np, of_device_match);

	return dev ? dev_get_drvdata(dev) : NULL;
}
EXPORT_SYMBOL_GPL(of_find_dpp_by_node);
#endif

MODULE_AUTHOR("Seong-gyu Park <seongyu.park@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC Display Pre Processor");
MODULE_LICENSE("GPL v2");
