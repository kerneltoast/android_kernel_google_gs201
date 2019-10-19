// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_decon.c
 *
 * Copyright (C) 2018 Samsung Electronics Co.Ltd
 * Authors:
 *	Hyung-jun Kim <hyungjun07.kim@samsung.com>
 *	Seong-gyu Park <seongyu.park@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <drm/drmP.h>
#include <drm/exynos_drm.h>

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>
#include <linux/console.h>

#include <video/videomode.h>

#include <exynos_drm_crtc.h>
#include <exynos_drm_plane.h>
#include <exynos_drm_dpp.h>
#include <exynos_drm_dsim.h>
#include <exynos_drm_drv.h>
#include <exynos_drm_fb.h>
#include <exynos_drm_decon.h>

#include <decon_cal.h>
#include <regs-decon.h>

struct decon_device *decon_drvdata[MAX_DECON_CNT];

static int decon_log_level = 6;

#define decon_info(decon, fmt, ...)					   \
	do {								   \
		if (decon_log_level >= 6) {				   \
			DRM_INFO("%s[%d]: "fmt, decon->dev->driver->name,  \
					decon->id, ##__VA_ARGS__);	   \
		}							   \
	} while (0)

#define decon_warn(decon, fmt, ...)					   \
	do {								   \
		if (decon_log_level >= 4) {				   \
			DRM_WARN("%s[%d]: "fmt, decon->dev->driver->name,  \
					decon->id, ##__VA_ARGS__);	   \
		}							   \
	} while (0)

#define decon_err(decon, fmt, ...)					   \
	do {								   \
		if (decon_log_level >= 3) {				   \
			DRM_ERROR("%s[%d]: "fmt, decon->dev->driver->name, \
					decon->id, ##__VA_ARGS__);	   \
		}							   \
	} while (0)

#define decon_dbg(decon, fmt, ...)					   \
	do {								   \
		if (decon_log_level >= 7) {				   \
			DRM_INFO("%s[%d]: "fmt, decon->dev->driver->name,  \
					decon->id, ##__VA_ARGS__);	   \
		}							   \
	} while (0)

#define SHADOW_UPDATE_TIMEOUT_US	(300 * USEC_PER_MSEC) /* 300ms */

static const struct of_device_id decon_driver_dt_match[] = {
	{.compatible = "samsung,exynos-decon"},
	{},
};
MODULE_DEVICE_TABLE(of, decon_driver_dt_match);

void decon_dump(struct decon_device *decon)
{
	int acquired = console_trylock();
	void __iomem *base_regs = get_decon_drvdata(0)->regs.base_addr;
	int i;

	if (decon->state != DECON_STATE_ON) {
		decon_info(decon, "DECON%d is disabled, state(%d)\n", decon->id,
				decon->state);
		goto err;
	}

	__decon_dump(decon->id, decon->regs.base_addr, base_regs,
			decon->config.dsc.enabled);

	for (i = 0; i < decon->dpp_cnt; ++i)
		dpp_dump(decon->dpp[i]);

err:
	if (acquired)
		console_unlock();
}

static inline u32 win_start_pos(int x, int y)
{
	return (WIN_STRPTR_Y_F(y) | WIN_STRPTR_X_F(x));
}

static inline u32 win_end_pos(int x, int y,  u32 xres, u32 yres)
{
	return (WIN_ENDPTR_Y_F(y + yres - 1) | WIN_ENDPTR_X_F(x + xres - 1));
}

/* ARGB value */
#define COLOR_MAP_VALUE			0x00340080

static void decon_set_color_map(struct decon_device *decon, u32 win_id,
						u32 hactive, u32 vactive)
{
	struct decon_window_regs win_info;

	decon_dbg(decon, "%s +\n", __func__);

	memset(&win_info, 0, sizeof(struct decon_window_regs));
	win_info.start_pos = win_start_pos(0, 0);
	win_info.end_pos = win_end_pos(0, 0, hactive, vactive);
	win_info.start_time = 0;
	win_info.colormap = 0x000000; /* black */
	win_info.blend = DECON_BLENDING_NONE;
	decon_reg_set_window_control(decon->id, win_id, &win_info, true);
	decon_reg_update_req_window(decon->id, win_id);

	decon_dbg(decon, "%s -\n", __func__);
}

static int decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	/* TODO : need to write code completely */
	decon_dbg(decon, "%s\n", __func__);

	return 0;
}

static void decon_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	/* TODO : need to write code completely */
	decon_dbg(decon, "%s\n", __func__);

}

static void decon_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	decon_dbg(decon, "%s +\n", __func__);
	decon_reg_wait_update_done_and_mask(decon->id, &decon->config.mode,
			SHADOW_UPDATE_TIMEOUT_US);

	/* consumed crtc event completion of drm_atomic_helper_commit_hw_done */
	exynos_crtc_handle_event(crtc);
	decon_dbg(decon, "%s -\n", __func__);
}

static void decon_update_plane(struct exynos_drm_crtc *crtc,
			       struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
				to_exynos_plane_state(plane->base.state);
	struct dpp_device *dpp = plane_to_dpp(plane);
	struct decon_device *decon = crtc->ctx;
	struct decon_window_regs win_info;
	unsigned int zpos;
	struct exynos_drm_fb *exynos_fb;
	bool colormap = false;

	decon_dbg(decon, "%s +\n", __func__);

	if (state->base.fb) {
		exynos_fb = container_of(state->base.fb, struct exynos_drm_fb,
				fb);
		colormap = exynos_fb->exynos_buf[0]->colormap;
	}

	memset(&win_info, 0, sizeof(struct decon_window_regs));
	win_info.start_pos = win_start_pos(state->crtc.x, state->crtc.y);
	win_info.end_pos = win_end_pos(state->crtc.x, state->crtc.y,
			state->crtc.w, state->crtc.h);
	win_info.start_time = 0;

	win_info.ch = dpp->id; /* DPP's id is DPP channel number */

	win_info.plane_alpha = state->alpha;
	win_info.blend = state->blend_mode;

	win_info.colormap = state->color;

	zpos = state->base.zpos;
	decon_reg_set_window_control(decon->id, zpos, &win_info, colormap);

	if (!colormap) {
		dpp->decon_id = decon->id;
		dpp->update(dpp, state);
		dpp->is_win_connected = true;
	} else {
		dpp->is_win_connected = false;
	}

	dpp->win_id = zpos;

	decon_dbg(decon, "plane idx[%d]: alpha(0x%x) blend_mode(%d) color(%s:0x%x)\n",
			drm_plane_index(&plane->base), state->alpha,
			state->blend_mode,
			colormap ? "enable" : "disable", state->color);
	decon_dbg(decon, "%s -\n", __func__);
}

static void decon_disable_plane(struct exynos_drm_crtc *crtc,
				struct exynos_drm_plane *plane)
{
	struct decon_device *decon = crtc->ctx;
	struct dpp_device *dpp = plane_to_dpp(plane);

	decon_dbg(decon, "%s +\n", __func__);

	/*
	 * When disabling the plane, previously connected window(zpos) should
	 * be disabled not newly requested zpos(window).
	 */
	decon_reg_set_win_enable(decon->id, dpp->win_id, 0);

	if (dpp->is_win_connected) {
		dpp->decon_id = decon->id;
		dpp->disable(dpp);
	}

	decon_dbg(decon, "plane idx[%d] win_id(%d) is_win_connected(%d)\n",
			drm_plane_index(&plane->base), dpp->win_id,
			dpp->is_win_connected);
	decon_dbg(decon, "%s -\n", __func__);
}

static void decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	decon_dbg(decon, "%s +\n", __func__);
	decon_reg_all_win_shadow_update_req(decon->id);
	decon_reg_start(decon->id, &decon->config);
	DPU_EVENT_LOG(DPU_EVT_DECON_TRIG_UNMASK, decon->id, decon);
	decon_dbg(decon, "%s -\n", __func__);
}

static void decon_print_config_info(struct decon_device *decon)
{
	char *str_output = NULL;
	char *str_trigger = NULL;

	if (decon->config.mode.trig_mode == DECON_HW_TRIG)
		str_trigger = "hw trigger.";
	else if (decon->config.mode.trig_mode == DECON_SW_TRIG)
		str_trigger = "sw trigger.";
	if (decon->config.mode.op_mode == DECON_VIDEO_MODE)
		str_trigger = "";

	if (decon->config.out_type & DECON_OUT_DSI0)
		str_output = "DSI0";
	else if  (decon->config.out_type & DECON_OUT_DSI1)
		str_output = "DSI1";
	else if  (decon->config.out_type & DECON_OUT_DP0)
		str_output = "DP0";
	else if  (decon->config.out_type & DECON_OUT_DP1)
		str_output = "DP1";

	decon_info(decon, "%s mode. %s %s output.(%dx%d@%dhz)\n",
			decon->config.mode.op_mode ? "command" : "video",
			str_trigger, str_output,
			decon->config.image_width, decon->config.image_height,
			decon->bts.fps);

	decon_info(decon, "DSC: en(%d), cnt(%d), slice: cnt(%d), size(%dx%d)\n",
			decon->config.dsc.enabled, decon->config.dsc.dsc_count,
			decon->config.dsc.slice_count,
			decon->config.dsc.slice_width,
			decon->config.dsc.slice_height);
}

static void decon_set_te_pinctrl(struct decon_device *decon, bool en)
{
	int ret;

	if ((decon->config.mode.op_mode != DECON_MIPI_COMMAND_MODE) ||
			(decon->config.mode.trig_mode != DECON_HW_TRIG))
		return;

	if (!decon->res.pinctrl || !decon->res.te_on)
		return;

	ret = pinctrl_select_state(decon->res.pinctrl,
			en ? decon->res.te_on : decon->res.te_off);
	if (ret)
		decon_err(decon, "failed to control decon TE(%d)\n", en);
}

static void decon_enable_irqs(struct decon_device *decon)
{
	decon_reg_set_interrupts(decon->id, 1);

	enable_irq(decon->irq_fs);
	enable_irq(decon->irq_fd);
	enable_irq(decon->irq_ext);
	if ((decon->config.mode.op_mode == DECON_MIPI_COMMAND_MODE) &&
			(decon->config.mode.trig_mode == DECON_HW_TRIG))
		enable_irq(decon->irq_te);
}

static void decon_enable(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;
	int i;

	if (decon->state == DECON_STATE_ON) {
		decon_info(decon, "decon%d already enabled(%d)\n",
				decon->id, decon->state);
		return;
	}

	decon_info(decon, "%s +\n", __func__);

	pm_runtime_get_sync(decon->dev);

	decon_set_te_pinctrl(decon, true);

	decon_reg_init(decon->id, &decon->config);

	if (decon->config.mode.op_mode == DECON_MIPI_COMMAND_MODE)
		decon_set_color_map(decon, 0, decon->config.image_width,
				decon->config.image_height);

	decon_enable_irqs(decon);

	decon_print_config_info(decon);

	for (i = 0; i < MAX_PLANE; ++i)
		decon->dpp[i]->win_id = 0xFF;

	decon->state = DECON_STATE_ON;

	DPU_EVENT_LOG(DPU_EVT_DECON_ENABLED, decon->id, decon);

	decon_info(decon, "%s -\n", __func__);
}

static void decon_disable_irqs(struct decon_device *decon)
{
	disable_irq(decon->irq_fs);
	disable_irq(decon->irq_fd);
	disable_irq(decon->irq_ext);
	decon_reg_set_interrupts(decon->id, 0);
	if ((decon->config.mode.op_mode == DECON_MIPI_COMMAND_MODE) &&
			(decon->config.mode.trig_mode == DECON_HW_TRIG))
		disable_irq(decon->irq_te);
}

static void decon_disable(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	decon_info(decon, "%s +\n", __func__);

	decon_disable_irqs(decon);

	decon_reg_stop(decon->id, &decon->config, true, decon->bts.fps);

	decon_set_te_pinctrl(decon, false);

	decon->state = DECON_STATE_OFF;

	pm_runtime_put_sync(decon->dev);

	DPU_EVENT_LOG(DPU_EVT_DECON_DISABLED, decon->id, decon);

	decon_info(decon, "%s -\n", __func__);
}

static const struct exynos_drm_crtc_ops decon_crtc_ops = {
	.enable = decon_enable,
	.disable = decon_disable,
	.enable_vblank = decon_enable_vblank,
	.disable_vblank = decon_disable_vblank,
	.atomic_begin = decon_atomic_begin,
	.update_plane = decon_update_plane,
	.disable_plane = decon_disable_plane,
	.atomic_flush = decon_atomic_flush,
};

static int decon_bind(struct device *dev, struct device *master, void *data)
{
	struct decon_device *decon = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct drm_plane *default_plane;
	struct exynos_drm_plane_config plane_config;
	int i, ret = 0;

	decon->drm_dev = drm_dev;

	/* plane initialization in DPP channel order */
	for (i = 0; i < decon->dpp_cnt; ++i) {
		struct dpp_device *dpp = decon->dpp[i];

		if (!dpp)
			continue;

		memset(&plane_config, 0, sizeof(plane_config));

		plane_config.pixel_formats = dpp->pixel_formats;
		plane_config.num_pixel_formats = dpp->num_pixel_formats;
		plane_config.zpos = i;
		plane_config.type = (i == 0) ? DRM_PLANE_TYPE_PRIMARY :
						DRM_PLANE_TYPE_OVERLAY;
		if (dpp->is_support & DPP_SUPPORT_AFBC)
			plane_config.capabilities |= EXYNOS_DRM_PLANE_CAP_AFBC;

		ret = exynos_plane_init(drm_dev, &dpp->plane, i, &plane_config);
		if (ret)
			return ret;
	}

	default_plane = &decon->dpp[0]->plane.base;

	decon->crtc = exynos_drm_crtc_create(drm_dev, default_plane,
			decon->con_type, &decon_crtc_ops, decon);
	if (IS_ERR(decon->crtc))
		return PTR_ERR(decon->crtc);

	decon_dbg(decon, "%s -\n", __func__);
	return 0;
}

static void decon_unbind(struct device *dev, struct device *master,
			void *data)
{
	struct decon_device *decon = dev_get_drvdata(dev);

	decon_dbg(decon, "%s +\n", __func__);
	decon_disable(decon->crtc);
	decon_dbg(decon, "%s -\n", __func__);
}

static const struct component_ops decon_component_ops = {
	.bind	= decon_bind,
	.unbind = decon_unbind,
};

static irqreturn_t decon_irq_handler(int irq, void *dev_data)
{
	struct decon_device *decon = dev_data;
	u32 irq_sts_reg;
	u32 ext_irq = 0;

	spin_lock(&decon->slock);
	if (decon->state != DECON_STATE_ON)
		goto irq_end;

	irq_sts_reg = decon_reg_get_interrupt_and_clear(decon->id, &ext_irq);
	decon_dbg(decon, "%s: irq_sts_reg = %x, ext_irq = %x\n", __func__,
			irq_sts_reg, ext_irq);

	if (irq_sts_reg & DPU_FRAME_START_INT_PEND) {
		DPU_EVENT_LOG(DPU_EVT_DECON_FRAMESTART, decon->id, decon);
		decon_dbg(decon, "%s: frame start\n", __func__);
		if (decon->config.mode.op_mode == DECON_VIDEO_MODE)
			drm_crtc_handle_vblank(&decon->crtc->base);
	}

	if (irq_sts_reg & DPU_FRAME_DONE_INT_PEND) {
		DPU_EVENT_LOG(DPU_EVT_DECON_FRAMEDONE, decon->id, decon);
		decon_dbg(decon, "%s: frame done\n", __func__);
	}

	if (ext_irq & DPU_RESOURCE_CONFLICT_INT_PEND)
		decon_dbg(decon, "%s: resource conflict\n", __func__);

	if (ext_irq & DPU_TIME_OUT_INT_PEND) {
		decon_err(decon, "%s: timeout irq occurs\n", __func__);
		decon_dump(decon);
		WARN_ON(1);
	}

irq_end:
	spin_unlock(&decon->slock);
	return IRQ_HANDLED;
}

static int decon_parse_dt(struct decon_device *decon, struct device_node *np)
{
	struct device_node *dsc_np;
	struct device_node *dpp_np = NULL;
	struct property *prop;
	const __be32 *cur;
	u32 val;
	int ret = 0, i;
	int dpp_id;

	of_property_read_u32(np, "decon,id", &decon->id);

	ret = of_property_read_u32(np, "max_win", &decon->win_cnt);
	if (ret) {
		decon_err(decon, "failed to parse max windows count\n");
		return ret;
	}

	ret = of_property_read_u32(np, "op_mode", &decon->config.mode.op_mode);
	if (ret) {
		decon_err(decon, "failed to parse operation mode(%d)\n", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "trig_mode",
			&decon->config.mode.trig_mode);
	if (ret) {
		decon_err(decon, "failed to parse trigger mode(%d)\n", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "out_type", &decon->config.out_type);
	if (ret) {
		decon_err(decon, "failed to parse output type(%d)\n", ret);
		return ret;
	}

	if (of_property_read_u32(np, "ppc", (u32 *)&decon->bts.ppc))
		decon->bts.ppc = 2UL;
	decon_info(decon, "PPC(%llu)\n", decon->bts.ppc);

	if (of_property_read_u32(np, "line_mem_cnt",
				(u32 *)&decon->bts.line_mem_cnt)) {
		decon->bts.line_mem_cnt = 4UL;
		decon_warn(decon, "WARN: line memory cnt is not defined in DT.\n");
	}
	decon_info(decon, "line memory cnt(%d)\n", decon->bts.line_mem_cnt);

	if (of_property_read_u32(np, "cycle_per_line",
				(u32 *)&decon->bts.cycle_per_line)) {
		decon->bts.cycle_per_line = 8UL;
		decon_warn(decon, "WARN: cycle per line is not defined in DT.\n");
	}
	decon_info(decon, "cycle per line(%d)\n", decon->bts.cycle_per_line);

	if (of_property_read_u32(np, "decon_cnt", &decon->decon_cnt)) {
		decon->decon_cnt = 1;
		decon_warn(decon, "WARN: decon count is not defined in DT.\n");
	}
	decon_info(decon, "decon count(%d)\n", decon->decon_cnt);

	dsc_np = of_parse_phandle(np, "dsc-config", 0);
	if (!dsc_np) {
		decon->config.dsc.enabled = false;
	} else {
		decon->config.dsc.enabled = true;
		of_property_read_u32(dsc_np, "dsc_count",
				&decon->config.dsc.dsc_count);
		of_property_read_u32(dsc_np, "slice_count",
				&decon->config.dsc.slice_count);
		of_property_read_u32(dsc_np, "slice_height",
				&decon->config.dsc.slice_height);
		of_node_put(dsc_np);
	}

	if (decon->config.out_type == DECON_OUT_DSI)
		decon->config.mode.dsi_mode = DSI_MODE_DUAL_DSI;
	else
		decon->config.mode.dsi_mode = DSI_MODE_SINGLE;

	decon->dpp_cnt = of_count_phandle_with_args(np, "dpps", NULL);
	for (i = 0; i < decon->dpp_cnt; ++i) {
		dpp_np = of_parse_phandle(np, "dpps", i);
		if (!dpp_np) {
			decon_err(decon, "can't find dpp%d node\n", i);
			return -EINVAL;
		}

		decon->dpp[i] = of_find_dpp_by_node(dpp_np);
		if (!decon->dpp[i]) {
			decon_err(decon, "can't find dpp%d structure\n", i);
			return -EINVAL;
		}

		dpp_id = decon->dpp[i]->id;
		decon_info(decon, "found dpp%d(%s)\n", i, dpp_name[dpp_id]);

		if (dpp_np)
			of_node_put(dpp_np);
	}

	of_property_for_each_u32(np, "connector", prop, cur, val)
		decon->con_type |= val;

	return 0;
}

static int decon_remap_regs(struct decon_device *decon)
{
	struct resource *res;
	struct device *dev = decon->dev;
	struct device_node *np;
	struct platform_device *pdev;

	pdev = container_of(dev, struct platform_device, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	decon->regs.base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(decon->regs.base_addr)) {
		DRM_DEV_ERROR(decon->dev, "ioremap failed!\n");
		return PTR_ERR(decon->regs.base_addr);
	}
	decon_regs_desc_init(decon->regs.base_addr, "decon", REGS_DECON,
			decon->id);

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9-disp_ss");
	if (IS_ERR_OR_NULL(np)) {
		DRM_DEV_ERROR(decon->dev, "failed to find disp_ss node");
		return PTR_ERR(np);
	}
	decon->regs.ss_regs = of_iomap(np, 0);
	if (!decon->regs.ss_regs) {
		DRM_DEV_ERROR(decon->dev, "failed to map sysreg-disp address.");
		return -ENOMEM;
	}
	decon_regs_desc_init(decon->regs.ss_regs, "decon-ss", REGS_DECON_SYS,
			decon->id);

	return 0;
}

static irqreturn_t decon_te_irq_handler(int irq, void *dev_id)
{
	struct decon_device *decon = dev_id;

	if (decon->state != DECON_STATE_ON)
		goto end;

	DPU_EVENT_LOG(DPU_EVT_TE_INTERRUPT, decon->id, NULL);

	if (decon->config.mode.op_mode == DECON_MIPI_COMMAND_MODE)
		drm_crtc_handle_vblank(&decon->crtc->base);

end:
	return IRQ_HANDLED;
}

static int decon_register_irqs(struct decon_device *decon)
{
	struct device *dev = decon->dev;
	struct platform_device *pdev;
	struct resource *res;
	int ret = 0;
	int gpio;

	pdev = container_of(dev, struct platform_device, dev);

	/* 1: FRAME START */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	decon->irq_fs = res->start;
	ret = devm_request_irq(dev, res->start, decon_irq_handler,
			0, pdev->name, decon);
	if (ret) {
		decon_err(decon, "failed to install FRAME START irq\n");
		return ret;
	}
	disable_irq(decon->irq_fs);

	/* 2: FRAME DONE */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	decon->irq_fd = res->start;
	ret = devm_request_irq(dev, res->start, decon_irq_handler,
			0, pdev->name, decon);
	if (ret) {
		decon_err(decon, "failed to install FRAME DONE irq\n");
		return ret;
	}
	disable_irq(decon->irq_fd);

	/* 3: EXTRA: resource conflict, timeout and error irq */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 2);
	decon->irq_ext = res->start;
	ret = devm_request_irq(dev, res->start, decon_irq_handler,
			0, pdev->name, decon);
	if (ret) {
		decon_err(decon, "failed to install EXTRA irq\n");
		return ret;
	}
	disable_irq(decon->irq_ext);

	/* Get IRQ resource and register IRQ handler. */
	if (of_get_property(dev->of_node, "gpios", NULL) != NULL) {
		gpio = of_get_gpio(dev->of_node, 0);
		if (gpio < 0) {
			decon_err(decon, "failed to get proper gpio number\n");
			return -EINVAL;
		}
	} else {
		decon_info(decon, "failed to find TE gpio node from device tree\n");
		return 0;
	}

	decon->irq_te = gpio_to_irq(gpio);

	decon_info(decon, "%s: TE irq number(%d)\n", __func__, decon->irq_te);
	ret = devm_request_irq(dev, decon->irq_te, decon_te_irq_handler,
			IRQF_TRIGGER_RISING, pdev->name, decon);
	disable_irq(decon->irq_te);

	return ret;
}

static int decon_get_pinctrl(struct decon_device *decon)
{
	int ret = 0;

	if ((decon->config.mode.op_mode != DECON_MIPI_COMMAND_MODE) ||
			(decon->config.mode.trig_mode != DECON_HW_TRIG)) {
		decon_warn(decon, "doesn't need pinctrl\n");
		return 0;
	}

	decon->res.pinctrl = devm_pinctrl_get(decon->dev);
	if (IS_ERR(decon->res.pinctrl)) {
		decon_err(decon, "failed to get pinctrl\n");
		ret = PTR_ERR(decon->res.pinctrl);
		decon->res.pinctrl = NULL;
		goto err;
	}

	decon->res.te_on = pinctrl_lookup_state(decon->res.pinctrl, "hw_te_on");
	if (IS_ERR(decon->res.te_on)) {
		decon_err(decon, "failed to get hw_te_on pin state\n");
		ret = PTR_ERR(decon->res.te_on);
		decon->res.te_on = NULL;
		goto err;
	}
	decon->res.te_off = pinctrl_lookup_state(decon->res.pinctrl,
			"hw_te_off");
	if (IS_ERR(decon->res.te_off)) {
		decon_err(decon, "failed to get hw_te_off pin state\n");
		ret = PTR_ERR(decon->res.te_off);
		decon->res.te_off = NULL;
		goto err;
	}

err:
	return ret;
}

static int decon_get_clock(struct decon_device *decon)
{
	decon->res.aclk = devm_clk_get(decon->dev, "aclk");
	if (IS_ERR_OR_NULL(decon->res.aclk)) {
		decon_err(decon, "failed to get aclk\n");
		return PTR_ERR(decon->res.aclk);
	}

	return 0;
}

static int decon_init_resources(struct decon_device *decon)
{
	int ret = 0;

	ret = decon_remap_regs(decon);
	if (ret)
		goto err;

	ret = decon_register_irqs(decon);
	if (ret)
		goto err;

	ret = decon_get_pinctrl(decon);
	if (ret)
		goto err;

	ret = decon_get_clock(decon);
	if (ret)
		goto err;
err:
	return ret;
}

static int decon_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct decon_device *decon;
	struct device *dev = &pdev->dev;

	decon = devm_kzalloc(dev, sizeof(struct decon_device), GFP_KERNEL);
	if (!decon)
		return -ENOMEM;

	decon->dev = dev;

	ret = decon_parse_dt(decon, dev->of_node);
	if (ret)
		goto err;

	decon_drvdata[decon->id] = decon;

	spin_lock_init(&decon->slock);

#if defined(CONFIG_EXYNOS_BTS)
	decon->bts.ops = &dpu_bts_control;
	decon->bts.ops->bts_init(decon);
#endif

	decon->state = DECON_STATE_OFF;
	pm_runtime_enable(decon->dev);

	ret = decon_init_resources(decon);
	if (ret)
		goto err;

	/* set drvdata */
	platform_set_drvdata(pdev, decon);

	ret = dpu_init_debug(decon);
	if (ret)
		goto err;

	ret = component_add(dev, &decon_component_ops);
	if (ret)
		goto err;

	dev_info(dev, "successfully probed");

err:
	return ret;
}

static int decon_remove(struct platform_device *pdev)
{
	struct decon_device *decon;

	decon = platform_get_drvdata(pdev);
#if defined(CONFIG_EXYNOS_BTS)
	decon->bts.ops->bts_deinit(decon);
#endif
	dpu_deinit_debug(decon);
	component_del(&pdev->dev, &decon_component_ops);

	return 0;
}

#ifdef CONFIG_PM
static int decon_suspend(struct device *dev)
{
	struct decon_device *decon = dev_get_drvdata(dev);

	clk_disable_unprepare(decon->res.aclk);

	decon_info(decon, "suspended\n");

	return 0;
}

static int decon_resume(struct device *dev)
{
	struct decon_device *decon = dev_get_drvdata(dev);

	clk_prepare_enable(decon->res.aclk);

	decon_info(decon, "resumed\n");

	return 0;
}
#endif

static const struct dev_pm_ops decon_pm_ops = {
	SET_RUNTIME_PM_OPS(decon_suspend, decon_resume, NULL)
};

struct platform_driver decon_driver = {
	.probe		= decon_probe,
	.remove		= decon_remove,
	.driver		= {
		.name	= "exynos-decon",
		.pm	= &decon_pm_ops,
		.of_match_table = decon_driver_dt_match,
	},
};

MODULE_AUTHOR("Hyung-jun Kim <hyungjun07.kim@samsung.com>");
MODULE_AUTHOR("Seong-gyu Park <seongyu.park@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC Display and Enhancement Controller");
MODULE_LICENSE("GPL v2");
