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
#include <drm/drmP.h>
#include <drm/exynos_drm.h>
#include <drm/drm_atomic.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <linux/videodev2_exynos_media.h>
#include <linux/ion_exynos.h>
#include <linux/dma-buf.h>
#include <linux/smc.h>

#include <exynos_drm_fb.h>
#include <exynos_drm_dpp.h>
#include <exynos_drm_dsim.h>
#include <exynos_drm_format.h>
#include <exynos_drm_decon.h>

#include <regs-dpp.h>
#include <hdr_cal.h>

static int dpp_log_level = 6;

#define dpp_info(dpp, fmt, ...)						\
	do {								\
		if (dpp_log_level >= 6) {				\
			DRM_INFO("%s[%d]: "fmt, dpp->dev->driver->name,	\
					dpp->id, ##__VA_ARGS__);	\
		}							\
	} while (0)

#define dpp_warn(dpp, fmt, ...)						\
	do {								\
		if (dpp_log_level >= 4) {				\
			DRM_WARN("%s[%d]: "fmt, dpp->dev->driver->name,	\
					dpp->id, ##__VA_ARGS__);	\
		}							\
	} while (0)

#define dpp_err(dpp, fmt, ...)						\
	do {								\
		if (dpp_log_level >= 3) {				\
			DRM_ERROR("%s[%d]: "fmt, dpp->dev->driver->name,\
					dpp->id, ##__VA_ARGS__);	\
		}							\
	} while (0)

#define dpp_dbg(dpp, fmt, ...)						\
	do {								\
		if (dpp_log_level >= 7) {				\
			DRM_INFO("%s[%d]: "fmt, dpp->dev->driver->name,	\
					dpp->id, ##__VA_ARGS__);	\
		}							\
	} while (0)

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
	}, {
	},
};

void dpp_dump(struct dpp_device *dpp)
{
	if (dpp->state != DPP_STATE_ON) {
		dpp_info(dpp, "dpp state is off\n");
		return;
	}
	__dpp_dump(dpp->id, dpp->regs.dpp_base_regs, dpp->regs.dma_base_regs,
			dpp->attr);
}

static dma_addr_t dpp_alloc_map_buf_test(void)
{
	struct dma_buf *buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sg_table;
	size_t size;
	void *vaddr;
	dma_addr_t dma_addr;
	struct decon_device *decon = get_decon_drvdata(0);
	struct drm_device *drm_dev = decon->drm_dev;
	struct exynos_drm_private *priv = drm_dev->dev_private;

	size = PAGE_ALIGN(1440 * 3040 * 4);
	buf = ion_alloc_dmabuf("ion_system_heap", size, 0);
	if (IS_ERR(buf)) {
		pr_err("failed to allocate test buffer memory\n");
		return PTR_ERR(buf);
	}

	vaddr = dma_buf_vmap(buf);
	memset(vaddr, 0x80, size);
	dma_buf_vunmap(buf, vaddr);

	/* mapping buffer for translating to DVA */
	attachment = dma_buf_attach(buf, priv->iommu_client);
	if (IS_ERR_OR_NULL(attachment)) {
		pr_err("failed to attach dma_buf\n");
		return -EINVAL;
	}

	sg_table = dma_buf_map_attachment(attachment, DMA_TO_DEVICE);
	if (IS_ERR_OR_NULL(sg_table)) {
		pr_err("failed to map attachment\n");
		return -EINVAL;
	}

	dma_addr = ion_iovmm_map(attachment, 0, size, DMA_TO_DEVICE, 0);
	if (IS_ERR_VALUE(dma_addr)) {
		pr_err("failed to map iovmm\n");
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

	config->h_ratio = (config->src.w << 20) / config->dst.w;
	config->v_ratio = (config->src.h << 20) / config->dst.h;

	/* TODO: scaling will be implemented later */
	config->is_scale = false;
	/* TODO: blocking mode will be implemented later */
	config->is_block = false;
	/* TODO: very big count.. recovery will be not working... */
	config->rcv_num = 0x7FFFFFFF;
}

static void dpp_convert_plane_state_to_config(struct dpp_params_info *config,
				const struct exynos_drm_plane_state *state,
				const struct drm_display_mode *mode)
{
	struct drm_framebuffer *fb = state->base.fb;
	unsigned int simplified_rot;

	pr_debug("%s: mode(%dx%d)\n", __func__, mode->hdisplay, mode->vdisplay);

	config->src.x = state->base.src_x >> 16;
	config->src.y = state->base.src_y >> 16;
	config->src.w = state->base.src_w >> 16;
	config->src.h = state->base.src_h >> 16;
	config->src.f_w = fb->width;
	config->src.f_h = fb->height;

	config->dst.x = state->crtc.x;
	config->dst.y = state->crtc.y;
	config->dst.w = state->crtc.w;
	config->dst.h = state->crtc.h;
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
	} else if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0), fb->modifier)) {
		config->comp_type = COMP_TYPE_SBWC;
		config->blk_size = SBWC_BLOCK_SIZE_GET(fb->modifier);
	} else {
		config->comp_type = COMP_TYPE_NONE;
	}

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
		config->h_ratio = (config->src.h << 20) / config->dst.w;
		config->v_ratio = (config->src.w << 20) / config->dst.h;
	} else {
		config->h_ratio = (config->src.w << 20) / config->dst.w;
		config->v_ratio = (config->src.h << 20) / config->dst.h;
	}

	/* TODO: scaling will be implemented later */
	config->is_scale = false;
	/* TODO: blocking mode will be implemented later */
	config->is_block = false;
	/* TODO: very big count.. recovery will be not working... */
	config->rcv_num = 0x7FFFFFFF;
}

static void __dpp_enable(struct dpp_device *dpp)
{
	if (dpp->state == DPP_STATE_ON)
		return;

	dpp_reg_init(dpp->id, dpp->attr);

	dpp->state = DPP_STATE_ON;
	enable_irq(dpp->dma_irq);
	enable_irq(dpp->dpp_irq);

	dpp_dbg(dpp, "enabled\n");
}

static int set_protection(struct dpp_device *dpp, uint64_t modifier)
{
	bool protection;
	u32 protection_id;
	int ret = 0;
	static const u32 protection_ids[] = { PROT_L0, PROT_L1, PROT_L2,
					PROT_L3, PROT_L4, PROT_L5, PROT_L12 };

	protection = (modifier & DRM_FORMAT_MOD_PROTECTION) != 0;
	if (dpp->protection == protection)
		return ret;

	if (dpp->id >= ARRAY_SIZE(protection_ids)) {
		pr_err("%s: failed to get protection id(%u)\n", __func__,
				dpp->id);
		return -EINVAL;
	}
	protection_id = protection_ids[dpp->id];

	ret = exynos_smc(SMC_PROTECTION_SET, 0, protection_id,
			(protection ? SMC_PROTECTION_ENABLE :
			SMC_PROTECTION_DISABLE));
	if (ret) {
		pr_err("%s: failed to %s protection(ch:%u, ret:%d)\n", __func__,
				protection ? "enable" : "disable", dpp->id,
				ret);
		return ret;
	}
	dpp->protection = protection;

	pr_debug("%s: ch:%u, en:%d\n", __func__, dpp->id, protection);

	return ret;
}

static void __dpp_disable(struct dpp_device *dpp)
{
	if (dpp->state == DPP_STATE_OFF)
		return;

	disable_irq(dpp->dpp_irq);
	disable_irq(dpp->dma_irq);

	dpp_reg_deinit(dpp->id, false, dpp->attr);

	dpp->hdr_state.eotf_lut = NULL;
	dpp->hdr_state.oetf_lut = NULL;
	dpp->hdr_state.gm = NULL;
	dpp->hdr_state.tm = NULL;

	set_protection(dpp, 0);
	dpp->state = DPP_STATE_OFF;

	dpp_dbg(dpp, "disabled\n");
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

	/* Scaling is requested. need to check limitation */
	if (!test_bit(DPP_ATTR_CSC, &dpp->attr)) {
		dpp_err(dpp, "not support CSC\n");
		return -ENOTSUPP;
	}

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

	dpp_dbg(dpp, "%s +\n", __func__);

	memset(&config, 0, sizeof(struct dpp_params_info));

	dpp_convert_plane_state_to_config(&config, state, mode);

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

	dpp_dbg(dpp, "%s -\n", __func__);

	return 0;

err:
	dpp_err(dpp, "src[%d %d %d %d %d %d] dst[%d %d %d %d %d %d] format[%d]\n",
			config.src.x, config.src.y, config.src.w, config.src.h,
			config.src.f_w, config.src.f_h,
			config.dst.x, config.dst.y, config.dst.w, config.dst.h,
			config.dst.f_w, config.dst.f_h,
			config.format);
	dpp_err(dpp, "rot[0x%x] comp_type[%d]\n", config.rot, config.comp_type);

	return -ENOTSUPP;
}

static void dpp_hdr_update(struct dpp_device *dpp,
			const struct exynos_drm_plane_state *state)
{
	bool enable = false;

	if (dpp->hdr_state.eotf_lut != state->hdr_state.eotf_lut) {
		hdr_reg_set_eotf_lut(dpp->id, state->hdr_state.eotf_lut);
		dpp->hdr_state.eotf_lut = state->hdr_state.eotf_lut;
		enable = true;
	}

	if (dpp->hdr_state.oetf_lut != state->hdr_state.oetf_lut) {
		hdr_reg_set_oetf_lut(dpp->id, state->hdr_state.oetf_lut);
		dpp->hdr_state.oetf_lut = state->hdr_state.oetf_lut;
		enable = true;
	}

	if (dpp->hdr_state.gm != state->hdr_state.gm) {
		hdr_reg_set_gm(dpp->id, state->hdr_state.gm);
		dpp->hdr_state.gm = state->hdr_state.gm;
		enable = true;
	}

	if (dpp->hdr_state.tm != state->hdr_state.tm) {
		hdr_reg_set_tm(dpp->id, state->hdr_state.tm);
		dpp->hdr_state.tm = state->hdr_state.tm;
		enable = true;
	}

	hdr_reg_set_hdr(dpp->id, enable);
}

static int dpp_update(struct dpp_device *dpp,
			const struct exynos_drm_plane_state *state)
{
	struct dpp_params_info *config = &dpp->win_config;
	const struct drm_plane_state *plane_state = &state->base;
	const struct drm_crtc_state *crtc_state = plane_state->crtc->state;
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;

	dpp_dbg(dpp, "%s +\n", __func__);

	__dpp_enable(dpp);

	dpp_convert_plane_state_to_config(config, state, mode);

	dpp_hdr_update(dpp, state);
	set_protection(dpp, plane_state->fb->modifier);

	dpp_reg_configure_params(dpp->id, config, dpp->attr);

	dpp_dbg(dpp, "%s -\n", __func__);

	return 0;
}

static int dpp_bind(struct device *dev, struct device *master, void *data)
{
	struct dpp_device *dpp = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_plane_config plane_config;
	int ret = 0;
	int id = dpp->id;

	dpp_dbg(dpp, "%s +\n", __func__);

	memset(&plane_config, 0, sizeof(plane_config));

	plane_config.pixel_formats = dpp->pixel_formats;
	plane_config.num_pixel_formats = dpp->num_pixel_formats;
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

	dpp_dbg(dpp, "%s -\n", __func__);

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
	} else {
		dpp->pixel_formats = dpp_gf_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(dpp_gf_formats);
	}

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
	u32 dpp_irq = 0;

	spin_lock(&dpp->slock);
	if (dpp->state == DPP_STATE_OFF)
		goto irq_end;

	dpp_irq = dpp_reg_get_irq_and_clear(dpp->id);

irq_end:
	spin_unlock(&dpp->slock);
	return IRQ_HANDLED;
}

static irqreturn_t dma_irq_handler(int irq, void *priv)
{
	struct dpp_device *dpp = priv;
	u32 irqs;
	const char *str_comp;

	spin_lock(&dpp->dma_slock);
	if (dpp->state == DPP_STATE_OFF)
		goto irq_end;

	if (test_bit(DPP_ATTR_ODMA, &dpp->attr)) { /* ODMA case */
		irqs = odma_reg_get_irq_and_clear(dpp->id);

		if (irqs & ODMA_STATUS_FRAMEDONE_IRQ) {
			dpp_dbg(dpp, "dpp%d framedone irq occurs\n", dpp->id);
			DPU_EVENT_LOG(DPU_EVT_DPP_FRAMEDONE, dpp->decon_id,
					dpp);
			goto irq_end;
		}
	} else { /* IDMA case */
		irqs = idma_reg_get_irq_and_clear(dpp->id);

		if (irqs & IDMA_RECOVERY_START_IRQ) {
			DPU_EVENT_LOG(DPU_EVT_DMA_RECOVERY, dpp->decon_id, dpp);
			dpp->recovery_cnt++;
			str_comp = get_comp_src_name(dpp->comp_src);
			dpp_info(dpp, "recovery start(0x%x) cnt(%d) src(%s)\n",
					irqs, dpp->recovery_cnt, str_comp);
			goto irq_end;
		}

		/*
		 * TODO: Normally, DMA framedone occurs before DPP framedone.
		 * But DMA framedone can occur in case of AFBC crop mode
		 */
		if (irqs & IDMA_STATUS_FRAMEDONE_IRQ) {
			DPU_EVENT_LOG(DPU_EVT_DPP_FRAMEDONE, dpp->decon_id,
					dpp);
			goto irq_end;
		}
	}

irq_end:

	spin_unlock(&dpp->dma_slock);
	return IRQ_HANDLED;
}

static int dpp_init_resources(struct dpp_device *dpp)
{
	struct device *dev = dpp->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev;
	int ret = 0;
	int idx;

	pdev = container_of(dev, struct platform_device, dev);

	idx = of_property_match_string(np, "reg-names", "dma");
	dpp->regs.dma_base_regs = of_iomap(np, idx);
	if (!dpp->regs.dma_base_regs) {
		pr_err("failed to remap DPU_DMA SFR region\n");
		return -EINVAL;
	}
	dpp_regs_desc_init(dpp->regs.dma_base_regs, "dma", REGS_DMA, dpp->id);

	idx = of_property_match_string(np, "interrupts-names", "dma");
	dpp->dma_irq = of_irq_get(np, idx);
	dpp_info(dpp, "dma irq no = %lld\n", dpp->dma_irq);
	ret = devm_request_irq(dev, dpp->dma_irq, dma_irq_handler, 0,
			pdev->name, dpp);
	if (ret) {
		dpp_err(dpp, "failed to install DPU DMA irq\n");
		return -EINVAL;
	}
	disable_irq(dpp->dma_irq);

	if (test_bit(DPP_ATTR_DPP, &dpp->attr) ||
			test_bit(DPP_ATTR_WBMUX, &dpp->attr)) {
		idx = of_property_match_string(np, "reg-names", "dpp");
		dpp->regs.dpp_base_regs = of_iomap(np, idx);
		if (!dpp->regs.dpp_base_regs) {
			pr_err("failed to remap DPP SFR region\n");
			return -EINVAL;
		}
		dpp_regs_desc_init(dpp->regs.dpp_base_regs, "dpp", REGS_DPP,
				dpp->id);
	}

	if (test_bit(DPP_ATTR_DPP, &dpp->attr)) {
		idx = of_property_match_string(np, "interrupts-names", "dpp");
		dpp->dpp_irq = of_irq_get(np, idx);
		dpp_info(dpp, "dpp irq no = %lld\n", dpp->dpp_irq);
		ret = devm_request_irq(dev, dpp->dpp_irq, dpp_irq_handler, 0,
				pdev->name, dpp);
		if (ret) {
			dpp_err(dpp, "failed to install DPP irq\n");
			return -EINVAL;
		}
		disable_irq(dpp->dpp_irq);
	}

	if (test_bit(DPP_ATTR_HDR, &dpp->attr) ||
			test_bit(DPP_ATTR_HDR10_PLUS, &dpp->attr)) {
		idx = of_property_match_string(np, "reg-names", "hdr");
		dpp->regs.hdr_base_regs = of_iomap(np, idx);
		if (!dpp->regs.hdr_base_regs) {
			pr_err("failed to remap HDR SFR region\n");
			return -EINVAL;
		}
		hdr_regs_desc_init(dpp->regs.hdr_base_regs, "hdr", dpp->id);
	}

	ret = __dpp_init_resources(dpp);

	return ret;
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

	dpp_info(dpp, "dpp%d successfully probed", dpp->id);

	return component_add(dev, &exynos_dpp_component_ops);

fail:
	dpp_err(dpp, "dpp%d probe failed", dpp->id);

	return ret;
}

static int dpp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &exynos_dpp_component_ops);

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
static int of_device_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

struct dpp_device *of_find_dpp_by_node(struct device_node *np)
{
	struct device *dev;

	dev = bus_find_device(&platform_bus_type, NULL, np, of_device_match);

	return dev ? dev_get_drvdata(dev) : NULL;
}
EXPORT_SYMBOL(of_find_dpp_by_node);
#endif

MODULE_AUTHOR("Seong-gyu Park <seongyu.park@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC Display Pre Processor");
MODULE_LICENSE("GPL v2");
