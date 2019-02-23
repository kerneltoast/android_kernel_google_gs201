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

#include <video/videomode.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_plane.h"
#include "exynos_drm_dpp.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_decon.h"

#include "cal_9820/decon_cal.h"
#include "cal_9820/regs-decon.h"

struct decon_device *decon_drvdata[MAX_DECON_CNT];

static int decon_log_level = 7;

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

#define plane_to_decon_win(this)	\
	container_of(this, struct decon_win, plane)

#define for_each_window(decon, i)	\
	for ((i) = 0; (i) < decon->win_cnt; (i)++)

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

__maybe_unused
static void decon_set_color_map(struct decon_device *decon,
						u32 hactive, u32 vactive)
{
	struct decon_win *window = &decon->win[5];
	struct decon_window_regs win_info;

	decon_dbg(decon, "%s +\n", __func__);

	memset(&win_info, 0, sizeof(struct decon_window_regs));
	win_info.start_pos = win_start_pos(0, 0);
	win_info.end_pos = win_end_pos(0, 0, hactive, vactive);
	win_info.start_time = 0;
	win_info.colormap = 0x00FF00;
	win_info.blend = DECON_BLENDING_NONE;
	decon_reg_set_window_control(decon->id, window->idx, &win_info, true);
	decon_reg_update_req_window(decon->id, window->idx);

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
	struct decon_device *decon = crtc->ctx;
	struct decon_win *window = plane_to_decon_win(plane);
	struct decon_window_regs win_info;

	decon_dbg(decon, "%s +\n", __func__);
	memset(&win_info, 0, sizeof(struct decon_window_regs));
	win_info.start_pos = win_start_pos(state->crtc.x, state->crtc.y);
	win_info.end_pos = win_end_pos(state->crtc.x, state->crtc.y,
			state->crtc.w, state->crtc.h);
	win_info.start_time = 0;
	win_info.ch = window->dpp->id;
	/* TODO: alpha blending will be configurable in the future */
	win_info.plane_alpha = 0xff; /* opaque temporarily*/
	win_info.blend = DECON_BLENDING_NONE;
	decon_reg_set_window_control(decon->id, window->idx, &win_info, false);

	/* TODO: This will be activated after dpp driver is complete. */
	window->dpp->update(window->dpp, state);

	/*
	 * TODO: need to consider of updating windows timing.
	 * It needs to update all windows at the same time
	 * If not, hw state(enable or disable) of each windows
	 * can be mis-matched.
	 */
	decon_reg_update_req_window(decon->id, window->idx);
	decon_dbg(decon, "%s -\n", __func__);
}

static void decon_disable_plane(struct exynos_drm_crtc *crtc,
				struct exynos_drm_plane *plane)
{
	struct decon_device *decon = crtc->ctx;
	struct decon_win *window = plane_to_decon_win(plane);

	decon_dbg(decon, "%s +\n", __func__);
	decon_reg_set_win_enable(decon->id, window->idx, 0);

	window->dpp->disable(window->dpp);

	decon_reg_update_req_window(decon->id, window->idx);
	decon_dbg(decon, "%s -\n", __func__);
}

static void decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon_device *decon = crtc->ctx;

	decon_dbg(decon, "%s +\n", __func__);
	decon_reg_start(decon->id, &decon->config);
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

	decon_info(decon, "%s mode. %s %s output\n",
			decon->config.mode.op_mode ? "command" : "video",
			str_trigger, str_output);
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

	if (decon->state == DECON_STATE_ON) {
		decon_info(decon, "decon%d already enabled(%d)\n",
				decon->id, decon->state);
		return;
	}

	decon_dbg(decon, "%s +\n", __func__);

	pm_runtime_get_sync(decon->dev);

	decon_set_te_pinctrl(decon, true);

	decon_reg_init(decon->id, &decon->config);
	/*
	 * TODO: need to check whether decon operation has problem or not
	 * without window configuration.
	 * And VIDEO mode operation ?
	 */
	decon_reg_start(decon->id, &decon->config);

	decon_enable_irqs(decon);

	decon_print_config_info(decon);

	decon->state = DECON_STATE_ON;
	decon_info(decon, "enabled crtc[%d] = %dx%d-%dHz\n", crtc->base.base.id,
			decon->config.image_width, decon->config.image_height,
			crtc->base.mode.vrefresh);
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

	decon_disable_irqs(decon);

	/* TODO: 60 means 60 fps.. this will be fixed */
	decon_reg_stop(decon->id, &decon->config, true, 60);

	decon_set_te_pinctrl(decon, false);

	decon->state = DECON_STATE_OFF;

	pm_runtime_put_sync(decon->dev);

	decon_info(decon, "disabled! crtc[%d]\n", crtc->base.base.id);
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
	int i, ret = 0;

	decon->drm_dev = drm_dev;

	for_each_window(decon, i) {
		struct decon_win *win = &decon->win[i];
		struct dpp_device *dpp = decon->win[i].dpp;

		if (!dpp)
			continue;

		win->plane_config.pixel_formats = dpp->pixel_formats;
		win->plane_config.num_pixel_formats = dpp->num_pixel_formats;
		win->plane_config.zpos = i;
		win->plane_config.type = (i == 0) ? DRM_PLANE_TYPE_PRIMARY :
						DRM_PLANE_TYPE_OVERLAY;
		if (dpp->is_support & DPP_SUPPORT_AFBC)
			win->plane_config.capabilities |=
				EXYNOS_DRM_PLANE_CAP_AFBC;

		ret = exynos_plane_init(drm_dev, &win->plane, i,
				&win->plane_config);
		if (ret)
			return ret;
	}

	default_plane = &decon->win[DEFAULT_WIN].plane.base;

	decon->crtc = exynos_drm_crtc_create(drm_dev, default_plane,
			decon->con_type, &decon_crtc_ops, decon);

	if (IS_ERR(decon->crtc))
		return PTR_ERR(decon->crtc);

	return 0;
}

static void decon_unbind(struct device *dev, struct device *master,
			void *data)
{
	struct decon_device *decon = dev_get_drvdata(dev);

	decon_disable(decon->crtc);
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
		decon_dbg(decon, "%s: DECON%d Frame start\n", __func__,
				decon->id);
		if (decon->config.mode.op_mode == DECON_VIDEO_MODE)
			drm_crtc_handle_vblank(&decon->crtc->base);
	}

	if (irq_sts_reg & DPU_FRAME_DONE_INT_PEND)
		decon_dbg(decon, "%s: DECON%d Frame Done\n", __func__,
				decon->id);

	if (ext_irq & DPU_RESOURCE_CONFLICT_INT_PEND)
		decon_dbg(decon, "%s: DECON%d resource conflict\n", __func__,
				decon->id);

	if (ext_irq & DPU_TIME_OUT_INT_PEND) {
		decon_err(decon, "%s: DECON%d timeout irq occurs\n", __func__,
				decon->id);
		WARN_ON(1);
	}

irq_end:
	spin_unlock(&decon->slock);
	return IRQ_HANDLED;
}

static int exynos_panel_calc_slice_width(u32 dsc_cnt, u32 slice_num, u32 xres)
{
	u32 slice_width;
	u32 width_eff;
	u32 slice_width_byte_unit, comp_slice_width_byte_unit;
	u32 comp_slice_width_pixel_unit;
	u32 compressed_slice_w = 0;
	u32 i, j;

	if (dsc_cnt == 2)
		width_eff = xres >> 1;
	else
		width_eff = xres;

	if (slice_num / dsc_cnt == 2)
		slice_width = width_eff >> 1;
	else
		slice_width = width_eff;

	/* 3bytes per pixel */
	slice_width_byte_unit = slice_width * 3;
	/* integer value, /3 for 1/3 compression */
	comp_slice_width_byte_unit = slice_width_byte_unit / 3;
	/* integer value, /3 for pixel unit */
	comp_slice_width_pixel_unit = comp_slice_width_byte_unit / 3;

	i = comp_slice_width_byte_unit % 3;
	j = comp_slice_width_pixel_unit % 2;

	if (i == 0 && j == 0) {
		compressed_slice_w = comp_slice_width_pixel_unit;
	} else if (i == 0 && j != 0) {
		compressed_slice_w = comp_slice_width_pixel_unit + 1;
	} else if (i != 0) {
		while (1) {
			comp_slice_width_pixel_unit++;
			j = comp_slice_width_pixel_unit % 2;
			if (j == 0)
				break;
		}
		compressed_slice_w = comp_slice_width_pixel_unit;
	}

	return compressed_slice_w;
}

static int decon_parse_dt(struct decon_device *decon, struct device_node *np)
{
	struct property *prop;
	const __be32 *cur;
	u32 val;
	int ret = 0, i;

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

	/* TODO: This will be modified in the future */
	decon->config.image_width = 1440;
	decon->config.image_height = 3040;
	decon->config.dsc.en = 1;
	decon->config.dsc.cnt = 2;
	decon->config.dsc.slice_num = 2;
	decon->config.dsc.slice_h = 40;
	decon->config.dsc.enc_sw = exynos_panel_calc_slice_width(
			decon->config.dsc.cnt, decon->config.dsc.slice_num,
			decon->config.image_width);

	if (decon->config.out_type == DECON_OUT_DSI)
		decon->config.mode.dsi_mode = DSI_MODE_DUAL_DSI;
	else
		decon->config.mode.dsi_mode = DSI_MODE_SINGLE;

	for_each_window(decon, i) {
		struct device_node *dpp_np = NULL;
		struct dpp_device *dpp;

		dpp_np = of_parse_phandle(np, "dpps", i);
		if (!dpp_np)
			goto next;

		dpp = of_find_dpp_by_node(dpp_np);
		if (!dpp)
			goto next;

		decon->win[i].dpp = dpp;
		decon->win[i].idx = i;

		DRM_INFO("window%d is map to %s\n", i, dpp_name[dpp->id]);

next:
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

	if (decon->config.mode.op_mode == DECON_MIPI_COMMAND_MODE) {
		drm_crtc_handle_vblank(&decon->crtc->base);
		decon_reg_set_trigger(decon->id, &decon->config.mode,
				DECON_TRIG_DISABLE);
	}

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

	decon->state = DECON_STATE_OFF;
	pm_runtime_enable(decon->dev);

	ret = decon_init_resources(decon);
	if (ret)
		goto err;

	/* set drvdata */
	platform_set_drvdata(pdev, decon);

	ret = component_add(dev, &decon_component_ops);
	if (ret)
		goto err;

	dev_info(dev, "successfully probed");

err:
	return ret;
}

static int decon_remove(struct platform_device *pdev)
{
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
