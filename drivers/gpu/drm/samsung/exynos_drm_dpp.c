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

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <linux/videodev2_exynos_media.h>
#include <linux/ion_exynos.h>
#include <linux/dma-buf.h>

#include "exynos_drm_fb.h"
#include "exynos_drm_dpp.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_format.h"
#include "exynos_drm_decon.h"
#include "cal_9820/regs-dpp.h"

static int dpp_log_level = 7;

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
};

static const struct of_device_id dpp_of_match[] = {
	{ .compatible = "samsung,exynos-dpp", },
	{},
};

static dma_addr_t dpp_alloc_map_buf_test(void)
{
	struct dma_buf *buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sg_table;
	size_t size;
	void *vaddr;
	dma_addr_t dma_addr;
	struct dsim_device *dsim;

	dsim = dsim_drvdata[0];

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
	attachment = dma_buf_attach(buf, dsim->dev);
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
	config->is_comp = false;
	config->format = DPU_PIXEL_FORMAT_BGRA_8888;

	/* TODO: how to handle ? ... I don't know ... */
	config->addr[0] = dpp_alloc_map_buf_test();

	config->eq_mode = 0;
	config->hdr = 0;
	config->max_luminance = 0;
	config->min_luminance = 0;
	config->y_2b_strd = 0;
	config->c_2b_strd = 0;

	config->h_ratio = (config->src.w << 20) / config->dst.w;
	config->v_ratio = (config->dst.h << 20) / config->dst.h;

	/* TODO: scaling will be implemented later */
	config->is_scale = false;
	/* TODO: blocking mode will be implemented later */
	config->is_block = false;
	/* TODO: very big count.. recovery will be not working... */
	config->rcv_num = 0x7FFFFFFF;
}

static void dpp_convert_plane_state_to_config(struct dpp_params_info *config,
				const struct exynos_drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->base.fb;

	config->src.x = state->src.x;
	config->src.y = state->src.y;
	config->src.w = state->src.w;
	config->src.h = state->src.h;
	config->src.f_w = fb->width;
	config->src.f_h = fb->height;

	config->dst.x = state->crtc.x;
	config->dst.y = state->crtc.y;
	config->dst.w = state->crtc.w;
	config->dst.h = state->crtc.h;
	/* TODO: This hard coded value will be changed */
	config->dst.f_w = 1440;
	config->dst.f_h = 3040;

	config->rot = 0; /* no rotation */
	config->is_comp = state->afbc;
	config->format = convert_drm_format(fb->format->format);

	/* TODO: how to handle ? ... I don't know ... */
	config->addr[0] = exynos_drm_fb_dma_addr(fb, 0);
	config->addr[1] = exynos_drm_fb_dma_addr(fb, 1);
	config->addr[2] = exynos_drm_fb_dma_addr(fb, 2);
	config->addr[3] = exynos_drm_fb_dma_addr(fb, 3);

	config->eq_mode = 0;
	config->hdr = 0;
	config->max_luminance = 0;
	config->min_luminance = 0;
	config->y_2b_strd = 0;
	config->c_2b_strd = 0;

	if (config->format == DPU_PIXEL_FORMAT_NV12N)
		config->addr[1] = NV12N_CBCR_BASE(config->addr[0],
				config->src.f_w, config->src.f_h);

	if (config->format == DPU_PIXEL_FORMAT_NV12_P010)
		config->addr[1] = P010_CBCR_BASE(config->addr[0],
				config->src.f_w, config->src.f_h);

	if (config->format == DPU_PIXEL_FORMAT_NV12M_S10B ||
			config->format == DPU_PIXEL_FORMAT_NV21M_S10B) {
		config->addr[2] = config->addr[0] +
			NV12M_Y_SIZE(config->src.f_w, config->src.f_h);
		config->addr[3] = config->addr[1] +
			NV12M_CBCR_SIZE(config->src.f_w, config->src.f_h);
		config->y_2b_strd = S10B_2B_STRIDE(config->src.f_w);
		config->c_2b_strd = S10B_2B_STRIDE(config->src.f_w);
	}

	if (config->format == DPU_PIXEL_FORMAT_NV12N_10B) {
		config->addr[1] = NV12N_10B_CBCR_BASE(config->addr[0],
				config->src.f_w, config->src.f_h);
		config->addr[2] = config->addr[0] +
			NV12N_10B_Y_8B_SIZE(config->src.f_w, config->src.f_h);
		config->addr[3] = config->addr[1] +
			NV12N_10B_CBCR_8B_SIZE(config->src.f_w,
					config->src.f_h);
		config->y_2b_strd = S10B_2B_STRIDE(config->src.f_w);
		config->c_2b_strd = S10B_2B_STRIDE(config->src.f_w);
	}

	if (config->format == DPU_PIXEL_FORMAT_NV16M_S10B ||
			config->format == DPU_PIXEL_FORMAT_NV61M_S10B) {
		config->addr[2] = config->addr[0] +
			NV16M_Y_SIZE(config->src.f_w, config->src.f_h);
		config->addr[3] = config->addr[1] +
			NV16M_CBCR_SIZE(config->src.f_w, config->src.f_h);
		config->y_2b_strd = S10B_2B_STRIDE(config->src.f_w);
		config->c_2b_strd = S10B_2B_STRIDE(config->src.f_w);
	}

	config->h_ratio = (config->src.w << 20) / config->dst.w;
	config->v_ratio = (config->dst.h << 20) / config->dst.h;

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

	enable_irq(dpp->dma_irq);
	enable_irq(dpp->dpp_irq);

	dpp->state = DPP_STATE_ON;

	dpp_info(dpp, "enabled\n");
}

static void __dpp_disable(struct dpp_device *dpp)
{
	if (dpp->state == DPP_STATE_OFF)
		return;

	disable_irq(dpp->dpp_irq);
	disable_irq(dpp->dma_irq);

	dpp_reg_deinit(dpp->id, false, dpp->attr);

	dpp->state = DPP_STATE_OFF;

	dpp_info(dpp, "disabled\n");
}

static int dpp_disable(struct dpp_device *this_dpp)
{
	__dpp_disable(this_dpp);

	return 0;
}

static int dpp_check(struct dpp_device *this_dpp,
			const struct exynos_drm_plane_state *state)
{
	return 0;
}

static int dpp_update(struct dpp_device *this_dpp,
			const struct exynos_drm_plane_state *state)
{
	struct dpp_params_info *config = &this_dpp->win_config;

	dpp_dbg(this_dpp, "+\n");

	__dpp_enable(this_dpp);

	dpp_convert_plane_state_to_config(config, state);

	dpp_reg_configure_params(this_dpp->id, config, this_dpp->attr);

	dpp_dbg(this_dpp, "-\n");

	return 0;
}

static int dpp_bind(struct device *dev, struct device *master, void *data)
{
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

static int __init exynos_dpp_parse_dt(struct dpp_device *dpp,
		struct device_node *np)
{
	int ret = 0;

	ret = of_property_read_u32(np, "dpp,id", &dpp->id);
	if (ret < 0)
		goto fail;

	of_property_read_u32(np, "attr", (u32 *)&dpp->attr);
	of_property_read_u32(np, "port", &dpp->port);

	if (of_property_read_bool(np, "dpp,afbc"))
		dpp->is_support |= DPP_SUPPORT_AFBC;
	if (of_property_read_bool(np, "dpp,video"))
		dpp->is_support |= DPP_SUPPORT_VIDEO;

	if (dpp->is_support & DPP_SUPPORT_VIDEO) {
		dpp->pixel_formats = dpp_vg_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(dpp_vg_formats);
	} else {
		dpp->pixel_formats = dpp_gf_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(dpp_gf_formats);
	}

	dpp_info(dpp, "attr(0x%lx), port(%d)\n", dpp->attr, dpp->port);

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

	spin_lock(&dpp->dma_slock);
	if (dpp->state == DPP_STATE_OFF)
		goto irq_end;

	if (test_bit(DPP_ATTR_ODMA, &dpp->attr)) { /* ODMA case */
		irqs = odma_reg_get_irq_and_clear(dpp->id);

		if ((irqs & ODMA_WRITE_SLAVE_ERROR) ||
			       (irqs & ODMA_STATUS_DEADLOCK_IRQ)) {
			dpp_err(dpp, "odma%d error irq occur(0x%x)\n",
					dpp->id, irqs);
			goto irq_end;
		}
		if (irqs & ODMA_STATUS_FRAMEDONE_IRQ) {
			dpp_dbg(dpp, "dpp%d framedone irq occurs\n", dpp->id);
			DPU_EVENT_LOG(DPU_EVT_DPP_FRAMEDONE, dpp->decon_id,
					dpp);
			goto irq_end;
		}
	} else { /* IDMA case */
		irqs = idma_reg_get_irq_and_clear(dpp->id);

		if (irqs & IDMA_RECOVERY_START_IRQ) {
			dpp_info(dpp, "dma%d recovery start(0x%x)..\n",
					dpp->id, irqs);
			goto irq_end;
		}
		if ((irqs & IDMA_AFBC_TIMEOUT_IRQ) ||
				(irqs & IDMA_READ_SLAVE_ERROR) ||
				(irqs & IDMA_STATUS_DEADLOCK_IRQ)) {
			dpp_err(dpp, "dma%d error irq occur(0x%x)\n",
					dpp->id, irqs);
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

#if defined(CONFIG_SOC_EXYNOS9820)
		/* TODO: SoC dependency will be removed */
		if (irqs & IDMA_AFBC_CONFLICT_IRQ) {
			dpp_err(dpp, "dma%d AFBC conflict irq occurs\n",
					dpp->id);
			goto irq_end;
		}
#endif
	}

irq_end:

	spin_unlock(&dpp->dma_slock);
	return IRQ_HANDLED;
}

static int dpp_init_resources(struct dpp_device *dpp)
{
	struct resource *res;
	struct device *dev = dpp->dev;
	struct platform_device *pdev;
	int ret;

	pdev = container_of(dev, struct platform_device, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dpp_err(dpp, "failed to get mem resource\n");
		return -ENOENT;
	}

	dpp->regs.dma_base_regs = devm_ioremap_resource(dev, res);
	if (!dpp->regs.dma_base_regs) {
		dpp_err(dpp, "failed to remap DPU_DMA SFR region\n");
		return -EINVAL;
	}
	dpp_regs_desc_init(dpp->regs.dma_base_regs, "dma", REGS_DMA, dpp->id);
	dpp_info(dpp, "dma res: start(0x%x), end(0x%x), vir(0x%llx)\n",
			(u32)res->start, (u32)res->end,
			(u64)dpp->regs.dma_base_regs);

	/* DPP0 channel can only access common area of DPU_DMA */
	if (dpp->id == 0) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (!res) {
			dpp_err(dpp, "failed to get mem resource\n");
			return -ENOENT;
		}

		dpp->regs.dma_common_base_regs = devm_ioremap_resource(dev,
				res);
		if (!dpp->regs.dma_common_base_regs) {
			dpp_err(dpp, "failed to map DPU_DMA COMMON SFR\n");
			return -EINVAL;
		}
		dpp_regs_desc_init(dpp->regs.dma_common_base_regs, "dma_common",
				REGS_DMA_COMMON, dpp->id);
		dpp_info(dpp, "dma common res start:0x%x end:0x%x vir:0x%llx\n",
				(u32)res->start, (u32)res->end,
				(u64)dpp->regs.dma_common_base_regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dpp_err(dpp, "failed to get dpu dma irq resource\n");
		return -ENOENT;
	}
	dpp_info(dpp, "dma irq no = %lld\n", res->start);

	dpp->dma_irq = res->start;
	ret = devm_request_irq(dev, res->start, dma_irq_handler, 0,
			pdev->name, dpp);
	if (ret) {
		dpp_err(dpp, "failed to install DPU DMA irq\n");
		return -EINVAL;
	}
	disable_irq(dpp->dma_irq);

	if (test_bit(DPP_ATTR_DPP, &dpp->attr) ||
			test_bit(DPP_ATTR_WBMUX, &dpp->attr)) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res) {
			dpp_err(dpp, "failed to get mem resource\n");
			return -ENOENT;
		}

		dpp->regs.dpp_base_regs = devm_ioremap_resource(dev, res);
		if (!dpp->regs.dpp_base_regs) {
			dpp_err(dpp, "failed to remap DPP SFR region\n");
			return -EINVAL;
		}
		dpp_regs_desc_init(dpp->regs.dpp_base_regs, "dpp", REGS_DPP,
				dpp->id);
		dpp_info(dpp, "dpp res: start(0x%x), end(0x%x), vir(0x%llx)\n",
				(u32)res->start, (u32)res->end,
				(u64)dpp->regs.dpp_base_regs);
	}

	if (test_bit(DPP_ATTR_DPP, &dpp->attr)) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
		if (!res) {
			dpp_err(dpp, "failed to get dpp irq resource\n");
			return -ENOENT;
		}
		dpp_info(dpp, "dpp irq no = %lld\n", res->start);

		dpp->dpp_irq = res->start;
		ret = devm_request_irq(dev, res->start, dpp_irq_handler, 0,
				pdev->name, dpp);
		if (ret) {
			dpp_err(dpp, "failed to install DPP irq\n");
			return -EINVAL;
		}
		disable_irq(dpp->dpp_irq);
	}

	return 0;
}

static int dpp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct dpp_device *dpp;

	dpp = devm_kzalloc(dev, sizeof(struct dpp_device), GFP_KERNEL);
	if (!dpp)
		return -ENOMEM;

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

	dpp_info(dpp, "%s successfully probed", dpp_name[dpp->id]);

	return component_add(dev, &exynos_dpp_component_ops);

fail:
	dpp_err(dpp, "%s probe failed", dpp_name[dpp->id]);

	return ret;
}

static int dpp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &exynos_dpp_component_ops);

	return 0;
}

static int dpp_suspend(struct device *dev)
{
	struct dpp_device *dpp = dev_get_drvdata(dev);

	dpp_info(dpp, "%s suspended\n", dpp_name[dpp->id]);

	return 0;
}

static int dpp_resume(struct device *dev)
{
	struct dpp_device *dpp = dev_get_drvdata(dev);

	dpp_info(dpp, "%s resumed\n", dpp_name[dpp->id]);

	return 0;
}

static const struct dev_pm_ops dpp_pm_ops = {
	SET_RUNTIME_PM_OPS(dpp_suspend, dpp_resume, NULL)
};


struct platform_driver dpp_driver = {
	.probe = dpp_probe,
	.remove = dpp_remove,
	.driver = {
		   .name = "exynos-dpp",
		   .owner = THIS_MODULE,
		   .pm = &dpp_pm_ops,
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
