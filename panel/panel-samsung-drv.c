// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based Samsung common panel driver.
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <uapi/linux/sched/types.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <video/mipi_display.h>

#define CREATE_TRACE_POINTS
#include <trace/dpu_trace.h>
#include "../exynos_drm_connector.h"
#include "../exynos_drm_decon.h"
#include "../exynos_drm_dsim.h"
#include "../exynos_drm_dqe.h"
#include "panel-samsung-drv.h"

#define PANEL_ID_REG		0xA1
#define PANEL_ID_LEN		7
#define PANEL_ID_OFFSET		6
#define PANEL_ID_READ_SIZE	(PANEL_ID_LEN + PANEL_ID_OFFSET)
#define PANEL_SLSI_DDIC_ID_REG	0xD6
#define PANEL_SLSI_DDIC_ID_LEN	5
#define PROJECT_CODE_MAX	5

#ifndef DISPLAY_PANEL_INDEX_PRIMARY
#define DISPLAY_PANEL_INDEX_PRIMARY 0
#endif
#ifndef DISPLAY_PANEL_INDEX_SECONDARY
#define DISPLAY_PANEL_INDEX_SECONDARY 1
#endif

static const char ext_info_regs[] = { 0xDA, 0xDB, 0xDC, 0xA1 };

static const char * const disp_state_str[] = {
	[DISPLAY_STATE_ON] = "On",
	[DISPLAY_STATE_HBM] = "HBM",
	[DISPLAY_STATE_LP] = "LP",
	[DISPLAY_STATE_OFF] = "Off",
};

#define EXT_INFO_SIZE ARRAY_SIZE(ext_info_regs)

#define exynos_connector_to_panel(c)					\
	container_of((c), struct exynos_panel, exynos_connector)

#define bridge_to_exynos_panel(b) \
	container_of((b), struct exynos_panel, bridge)

static void exynos_panel_set_backlight_state(struct exynos_panel *ctx,
					enum exynos_panel_state panel_state);
static ssize_t exynos_panel_parse_byte_buf(char *input_str, size_t input_len,
					   const char **out_buf);
static int parse_u32_buf(char *src, size_t src_len, u32 *out, size_t out_len);
static void panel_update_local_hbm_locked(struct exynos_panel *ctx);
static void exynos_panel_check_mipi_sync_timing(struct drm_crtc *crtc,
					 const struct exynos_panel_mode *current_mode,
					 const struct exynos_panel_mode *target_mode,
					 struct exynos_panel *ctx);
static void exynos_panel_post_power_on(struct exynos_panel *ctx);
static void exynos_panel_pre_power_off(struct exynos_panel *ctx);

inline void exynos_panel_msleep(u32 delay_ms)
{
	dsim_trace_msleep(delay_ms);
}
EXPORT_SYMBOL_GPL(exynos_panel_msleep);
static void exynos_panel_node_attach(struct exynos_drm_connector *exynos_connector);

static inline bool is_backlight_off_state(const struct backlight_device *bl)
{
	return (bl->props.state & BL_STATE_STANDBY) != 0;
}

static inline bool is_backlight_lp_state(const struct backlight_device *bl)
{
	return (bl->props.state & BL_STATE_LP) != 0;
}

static inline enum display_state get_current_display_state_locked(struct exynos_panel *ctx)
{
	if (is_backlight_off_state(ctx->bl)) {
		return DISPLAY_STATE_OFF;
	} else if (is_backlight_lp_state(ctx->bl)) {
		return DISPLAY_STATE_LP;
	} else if (IS_HBM_ON(ctx->hbm_mode)) {
		return DISPLAY_STATE_HBM;
	} else {
		return DISPLAY_STATE_ON;
	}
}

static struct drm_crtc *get_exynos_panel_connector_crtc(struct exynos_panel *ctx)
{
	struct drm_crtc *crtc = NULL;

	mutex_lock(&ctx->crtc_lock);
	crtc = ctx->crtc;
	mutex_unlock(&ctx->crtc_lock);

	return crtc;
}

int exynos_panel_configure_te2_edges(struct exynos_panel *ctx,
				     u32 *timings, bool lp_mode)
{
	struct te2_mode_data *data;
	const u32 *t;
	int i;

	if (!ctx || !timings)
		return -EINVAL;

	t = timings;

	for_each_te2_timing(ctx, lp_mode, data, i) {
		data->timing.rising_edge = t[0];
		data->timing.falling_edge = t[1];
		t += 2;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_configure_te2_edges);

ssize_t exynos_panel_get_te2_edges(struct exynos_panel *ctx,
				   char *buf, bool lp_mode)
{
	struct te2_mode_data *data;
	size_t len = 0;
	int i;

	if (!ctx)
		return -EINVAL;

	for_each_te2_timing(ctx, lp_mode, data, i) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%ux%u@%d",
				 data->mode->hdisplay, data->mode->vdisplay,
				 drm_mode_vrefresh(data->mode));

		if (data->binned_lp)
			len += scnprintf(buf + len, PAGE_SIZE - len, "-lp_%s",
					 data->binned_lp->name);

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 " rising %u falling %u\n",
				 data->timing.rising_edge,
				 data->timing.falling_edge);
	}

	return len;
}
EXPORT_SYMBOL_GPL(exynos_panel_get_te2_edges);

int exynos_panel_get_current_mode_te2(struct exynos_panel *ctx,
				      struct exynos_panel_te2_timing *timing)
{
	struct te2_mode_data *data;
	const struct drm_display_mode *mode;
	u32 bl_th = 0;
	bool lp_mode;
	int i;

	if (!ctx)
		return -EINVAL;

	if (!ctx->current_mode)
		return -EAGAIN;

	mode = &ctx->current_mode->mode;
	lp_mode = ctx->current_mode->exynos_mode.is_lp_mode;

	if (lp_mode && !ctx->desc->num_binned_lp) {
		dev_warn(ctx->dev, "Missing LP mode command set\n");
		return -EINVAL;
	}

	if (lp_mode && !ctx->current_binned_lp)
		return -EAGAIN;

	if (ctx->current_binned_lp)
		bl_th = ctx->current_binned_lp->bl_threshold;

	for_each_te2_timing(ctx, lp_mode, data, i) {
		if (data->mode != mode)
			continue;

		if (data->binned_lp && data->binned_lp->bl_threshold != bl_th)
			continue;

		timing->rising_edge = data->timing.rising_edge;
		timing->falling_edge = data->timing.falling_edge;

		dev_dbg(ctx->dev,
			"found TE2 timing %s at %dHz: rising %u falling %u\n",
			!lp_mode ? "normal" : "LP", drm_mode_vrefresh(mode),
			timing->rising_edge, timing->falling_edge);

		return 0;
	}

	dev_warn(ctx->dev, "failed to find %s TE2 timing at %dHz\n",
		 !lp_mode ? "normal" : "LP", drm_mode_vrefresh(mode));

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(exynos_panel_get_current_mode_te2);

static void exynos_panel_update_te2(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!is_panel_active(ctx) || !funcs || !funcs->update_te2)
		return;

	funcs->update_te2(ctx);
}

static int exynos_panel_parse_gpios(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;
	int ret;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		dev_info(ctx->dev, "no reset/enable pins on emulator\n");
		return 0;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "failed to get reset-gpios %ld",
				PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		ctx->enable_gpio = NULL;

	ret = of_property_read_u32(dev->of_node, "vddd_gpio_fixed_level",
				   &ctx->vddd_gpio_fixed_level);
	if (ret) {
		ctx->vddd_gpio_fixed_level = GPIO_LEVEL_UNSPECIFIED;
	} else if (ctx->vddd_gpio_fixed_level > GPIO_LEVEL_HIGH) {
		dev_warn(ctx->dev, "ignore vddd_gpio_fixed_level value %u\n",
			 ctx->vddd_gpio_fixed_level);
		ctx->vddd_gpio_fixed_level = GPIO_LEVEL_UNSPECIFIED;
	}

	ctx->vddd_gpio = devm_gpiod_get(
		dev, "vddd",
		(ctx->vddd_gpio_fixed_level == GPIO_LEVEL_LOW ? GPIOD_OUT_LOW : GPIOD_OUT_HIGH));
	if (IS_ERR(ctx->vddd_gpio))
		ctx->vddd_gpio = NULL;

	ctx->ready_signal.gpio = of_get_named_gpio_flags(dev->of_node, "rdy-gpios", 0,
							 &ctx->ready_signal.gpio_flags);

	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}

static int exynos_panel_parse_regulators(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;
	struct regulator *reg;
	int ret;

	ctx->vddi = devm_regulator_get(dev, "vddi");
	if (IS_ERR(ctx->vddi)) {
		dev_warn(ctx->dev, "failed to get panel vddi.\n");
		return -EPROBE_DEFER;
	}

	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci)) {
		dev_warn(ctx->dev, "failed to get panel vci.\n");
		return -EPROBE_DEFER;
	}

	reg = devm_regulator_get_optional(dev, "vddd");
	if (!PTR_ERR_OR_ZERO(reg)) {
		pr_info("panel vddd found\n");
		ctx->vddd = reg;
	}

	ret = of_property_read_u32(dev->of_node, "vddd-normal-microvolt", &ctx->vddd_normal_uV);
	if (ret)
		ctx->vddd_normal_uV = 0;

	ret = of_property_read_u32(dev->of_node, "vddd-lp-microvolt", &ctx->vddd_lp_uV);
	if (ret) {
		ctx->vddd_lp_uV = 0;
		if (ctx->vddd_normal_uV != 0) {
			pr_warn("ignore vddd normal %u\n", ctx->vddd_normal_uV);
			ctx->vddd_normal_uV = 0;
		}
	} else {
		ctx->post_vddd_lp = of_property_read_bool(dev->of_node, "post-vddd-lp");
	}

	reg = devm_regulator_get_optional(dev, "vddr_en");
	if (!PTR_ERR_OR_ZERO(reg)) {
		dev_dbg(ctx->dev, "panel vddr_en found\n");
		ctx->vddr_en = reg;
	}

	reg = devm_regulator_get_optional(dev, "vddr");
	if (!PTR_ERR_OR_ZERO(reg)) {
		dev_dbg(ctx->dev, "panel vddr found\n");
		ctx->vddr = reg;
	}

	return 0;
}

int exynos_panel_read_id(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[PANEL_ID_READ_SIZE];
	int ret;

	ret = mipi_dsi_dcs_read(dsi, ctx->desc->panel_id_reg ? : PANEL_ID_REG,
				buf, PANEL_ID_READ_SIZE);
	if (ret != PANEL_ID_READ_SIZE) {
		dev_warn(ctx->dev, "Unable to read panel id (%d)\n", ret);
		return ret;
	}

	exynos_bin2hex(buf + PANEL_ID_OFFSET, PANEL_ID_LEN,
		       ctx->panel_id, sizeof(ctx->panel_id));

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_read_id);

int exynos_panel_read_ddic_id(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[PANEL_SLSI_DDIC_ID_LEN] = {0};
	int ret;

	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF0, 0x5A, 0x5A);
	ret = mipi_dsi_dcs_read(dsi, PANEL_SLSI_DDIC_ID_REG,
				buf, PANEL_SLSI_DDIC_ID_LEN);
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xF0, 0xA5, 0xA5);
	if (ret != PANEL_SLSI_DDIC_ID_LEN) {
		dev_warn(ctx->dev, "Unable to read DDIC id (%d)\n", ret);
		return ret;
	}

	exynos_bin2hex(buf, PANEL_SLSI_DDIC_ID_LEN,
				ctx->panel_id, sizeof(ctx->panel_id));
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_read_ddic_id);

static int exynos_panel_read_extinfo(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[EXT_INFO_SIZE];
	int i, ret;

	/* extinfo already set, skip reading */
	if (ctx->panel_extinfo[0] != '\0')
		return 0;

	for (i = 0; i < EXT_INFO_SIZE; i++) {
		ret = mipi_dsi_dcs_read(dsi, ext_info_regs[i], buf + i, 1);
		if (ret != 1) {
			dev_warn(ctx->dev,
				 "Unable to read panel extinfo (0x%x: %d)\n",
				 ext_info_regs[i], ret);
			return ret;
		}

	}
	exynos_bin2hex(buf, i, ctx->panel_extinfo, sizeof(ctx->panel_extinfo));

	return 0;
}

void exynos_panel_get_panel_rev(struct exynos_panel *ctx, u8 rev)
{
	switch (rev) {
	case 0:
		ctx->panel_rev = PANEL_REV_PROTO1;
		break;
	case 1:
		ctx->panel_rev = PANEL_REV_PROTO1_1;
		break;
	case 2:
		ctx->panel_rev = PANEL_REV_PROTO1_2;
		break;
	case 8:
		ctx->panel_rev = PANEL_REV_EVT1;
		break;
	case 9:
		ctx->panel_rev = PANEL_REV_EVT1_1;
		break;
	case 0xA:
		ctx->panel_rev = PANEL_REV_EVT1_2;
		break;
	case 0xC:
		ctx->panel_rev = PANEL_REV_DVT1;
		break;
	case 0xD:
		ctx->panel_rev = PANEL_REV_DVT1_1;
		break;
	case 0x10:
		ctx->panel_rev = PANEL_REV_PVT;
		break;
	case 0x14:
		ctx->panel_rev = PANEL_REV_MP;
		break;
	default:
		dev_warn(ctx->dev,
			 "unknown rev from panel (0x%x), default to latest\n",
			 rev);
		ctx->panel_rev = PANEL_REV_LATEST;
		return;
	}

	dev_info(ctx->dev, "panel_rev: 0x%x\n", ctx->panel_rev);
}
EXPORT_SYMBOL_GPL(exynos_panel_get_panel_rev);

void exynos_panel_get_revision_by_module_ids(struct exynos_panel *ctx, u32 module_id)
{
	int i;
	u32 rev;

	if (!ctx->desc || !ctx->desc->module_ids) {
		dev_err(ctx->dev, "panel revision array is not defined!\n");
		return;
	}

	for (i = 0; i < ctx->desc->num_module_ids; i++) {
		if (ctx->desc->module_ids[i].module_id == module_id) {
			rev = ctx->desc->module_ids[i].revision;
			dev_info(ctx->dev, "module id: 0x%x, revision: 0x%x\n", module_id, rev);
			ctx->panel_rev = rev;
			return;
		}
	}

	dev_warn(ctx->dev, "Unknown rev from panel (0x%x), default to latest\n", module_id);
	ctx->panel_rev = PANEL_REV_LATEST;
}
EXPORT_SYMBOL_GPL(exynos_panel_get_revision_by_module_ids);

void exynos_panel_model_init(struct exynos_panel *ctx, const char* project, u8 extra_info)
{

	u8 vendor_info;
	u8 panel_rev;

	if (ctx->panel_extinfo[0] == '\0' || ctx->panel_rev == 0 || !project)
		return;

	if (strlen(project) > PROJECT_CODE_MAX) {
		dev_err(ctx->dev, "Project Code '%s' is longer than maximum %d charactrers\n",
			project, PROJECT_CODE_MAX);
		return;
	}

	vendor_info  = hex_to_bin(ctx->panel_extinfo[1]) & 0x0f;
	panel_rev = __builtin_ctz(ctx->panel_rev);

	/*
	 * Panel Model Format:
	 * [Project Code]-[Vendor Info][Panel Revision]-[Extra Info]
	 */
	scnprintf(ctx->panel_model, PANEL_MODEL_MAX, "%s-%01X%02X-%02X",
			project, vendor_info, panel_rev, extra_info);
}
EXPORT_SYMBOL_GPL(exynos_panel_model_init);

int exynos_panel_init(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (ctx->initialized)
		return 0;

	ret = exynos_panel_read_extinfo(ctx);
	if (!ret)
		ctx->initialized = true;

	if (!ctx->panel_rev && funcs && funcs->get_panel_rev) {
		u32 id;

		if (kstrtou32(ctx->panel_extinfo, 16, &id)) {
			dev_warn(ctx->dev,
				 "failed to get panel extinfo, default to latest\n");
			ctx->panel_rev = PANEL_REV_LATEST;
		} else {
			/* reverse here to match the id order read from bootloader */
			funcs->get_panel_rev(ctx, __builtin_bswap32(id));
		}
	} else if (!ctx->panel_rev) {
		dev_warn(ctx->dev,
			 "unable to get panel rev, default to latest\n");
		ctx->panel_rev = PANEL_REV_LATEST;
	}

	if (funcs && funcs->read_id)
		ret = funcs->read_id(ctx);
	else
		ret = exynos_panel_read_id(ctx);
	if (ret)
		return ret;

	if (funcs && funcs->panel_init)
		funcs->panel_init(ctx);

	if (funcs && funcs->run_normal_mode_work) {
		dev_info(ctx->dev, "%s: schedule normal_mode_work\n", __func__);
		schedule_delayed_work(&ctx->normal_mode_work,
				      msecs_to_jiffies(ctx->normal_mode_work_delay_ms));
	}

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_panel_init);

static int exynos_panel_wait_ready(struct exynos_panel *ctx)
{
	ktime_t start_time, end_time;
	int rdy_active_val;
	unsigned int timeout_ms;
	bool panel_ready_detected = false;

	/* For the project doesn't have the ready pin signal, return immediately. */
	if (ctx->ready_signal.irq < 0)
		return 0;

	DPU_ATRACE_BEGIN(__func__);

	start_time = ktime_get();
	reinit_completion(&ctx->ready_signal.detected);
	enable_irq(ctx->ready_signal.irq);

	/* expected ready pin active value */
	rdy_active_val = (ctx->ready_signal.gpio_flags & OF_GPIO_ACTIVE_LOW) == 0;
	timeout_ms = (ctx->desc && ctx->desc->rdy_timeout_ms) ? ctx->desc->rdy_timeout_ms :
								PANEL_READY_TIMEOUT_MS;

	/* Check if the ready pin is already ready. If not just wait for the interrupt. */
	if (gpio_get_value(ctx->ready_signal.gpio) == rdy_active_val ||
	    wait_for_completion_timeout(&ctx->ready_signal.detected, msecs_to_jiffies(timeout_ms)) >
		    0)
		panel_ready_detected = true;

	disable_irq(ctx->ready_signal.irq);
	end_time = ktime_get();

	if (panel_ready_detected) {
		dev_dbg(ctx->dev, "panel ready detected: %lldms\n",
			ktime_sub_ns(end_time, start_time) / 1000);
	} else {
		dev_err(ctx->dev, "wait panel ready timeout: %ums\n", timeout_ms);
	}

	DPU_ATRACE_END(__func__);

	return panel_ready_detected ? 0 : -ETIMEDOUT;
}

int exynos_panel_reset(struct exynos_panel *ctx)
{
	int ret;
	int delay;
	const int *timing_ms = ctx->desc->reset_timing_ms;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR) || IS_ERR_OR_NULL(ctx->reset_gpio))
		return 0;

	delay = timing_ms[PANEL_RESET_TIMING_HIGH] ?: 5;
	if (delay > 0) {
		gpiod_set_value(ctx->reset_gpio, 1);
		dev_dbg(ctx->dev, "reset=H, delay: %dms\n", delay);
		delay *= 1000;
		usleep_range(delay, delay + 10);
	}

	gpiod_set_value(ctx->reset_gpio, 0);
	delay = timing_ms[PANEL_RESET_TIMING_LOW] ?: 5;
	dev_dbg(ctx->dev, "reset=L, delay: %dms\n", delay);
	delay *= 1000;
	usleep_range(delay, delay + 10);

	gpiod_set_value(ctx->reset_gpio, 1);
	delay = timing_ms[PANEL_RESET_TIMING_INIT] ?: 10;
	dev_dbg(ctx->dev, "reset=H, delay: %dms\n", delay);
	delay *= 1000;
	usleep_range(delay, delay + 10);

	dev_dbg(ctx->dev, "%s -\n", __func__);

	ret = exynos_panel_wait_ready(ctx);
	if (ret)
		return ret;

	ctx->is_brightness_initialized = false;
	exynos_panel_init(ctx);

	exynos_panel_post_power_on(ctx);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_reset);

static void _exynos_panel_set_vddd_voltage(struct exynos_panel *ctx, bool is_lp)
{
	if (!IS_ERR_OR_NULL(ctx->vddd_gpio)) {
		bool gpio_level = is_lp ? GPIO_LEVEL_LOW : GPIO_LEVEL_HIGH;

		if (ctx->vddd_gpio_fixed_level != GPIO_LEVEL_UNSPECIFIED)
			gpio_level = ctx->vddd_gpio_fixed_level;
		gpiod_set_value(ctx->vddd_gpio, gpio_level);
		dev_dbg(ctx->dev, "%s: is_lp: %d, vddd_gpio: %d\n", __func__, is_lp, gpio_level);
	} else {
		u32 uv = is_lp ? ctx->vddd_lp_uV : ctx->vddd_normal_uV;

		if (!uv || !ctx->vddd)
			return;

		if (regulator_set_voltage(ctx->vddd, uv, uv))
			dev_err(ctx->dev, "failed to set vddd at %u uV\n", uv);
	}
}

static int _exynos_panel_reg_ctrl(struct exynos_panel *ctx,
	const struct panel_reg_ctrl *reg_ctrl, bool enable)
{
	struct regulator *panel_reg[PANEL_REG_ID_MAX] = {
		[PANEL_REG_ID_VCI] = ctx->vci,
		[PANEL_REG_ID_VDDD] = ctx->vddd,
		[PANEL_REG_ID_VDDI] = ctx->vddi,
		[PANEL_REG_ID_VDDR_EN] = ctx->vddr_en,
		[PANEL_REG_ID_VDDR] = ctx->vddr,
	};
	u32 i;

	for (i = 0; i < PANEL_REG_COUNT; i++) {
		enum panel_reg_id id = reg_ctrl[i].id;
		u32 delay_ms = reg_ctrl[i].post_delay_ms;
		int ret;
		struct regulator *reg;

		if (!IS_VALID_PANEL_REG_ID(id))
			return 0;

		reg = panel_reg[id];
		if (!reg) {
			dev_dbg(ctx->dev, "no valid regulator found id=%d\n", id);
			continue;
		}
		ret = enable ? regulator_enable(reg) : regulator_disable(reg);
		if (ret) {
			dev_err(ctx->dev, "failed to %s regulator id=%d\n",
				enable ? "enable" : "disable", id);
			return ret;
		}

		if (delay_ms)
			usleep_range(delay_ms * 1000, delay_ms * 1000 + 10);
		dev_dbg(ctx->dev, "%s regulator id=%d with post_delay=%d ms\n",
			enable ? "enable" : "disable", id, delay_ms);
	}
	return 0;
}

static int _exynos_panel_set_power(struct exynos_panel *ctx, bool on)
{
	const struct panel_reg_ctrl default_ctrl_disable[PANEL_REG_COUNT] = {
		{PANEL_REG_ID_VDDR, 0},
		{PANEL_REG_ID_VDDR_EN, 0},
		{PANEL_REG_ID_VDDD, 0},
		{PANEL_REG_ID_VDDI, 0},
		{PANEL_REG_ID_VCI, 0},
	};
	const struct panel_reg_ctrl default_ctrl_enable[PANEL_REG_COUNT] = {
		{PANEL_REG_ID_VDDI, 5},
		{PANEL_REG_ID_VDDD, 0},
		{PANEL_REG_ID_VCI, 0},
		{PANEL_REG_ID_VDDR_EN, 2},
		{PANEL_REG_ID_VDDR, 0},

	};
	const struct panel_reg_ctrl *reg_ctrl;

	if (on) {
		if (!IS_ERR_OR_NULL(ctx->enable_gpio)) {
			gpiod_set_value(ctx->enable_gpio, 1);
			usleep_range(10000, 11000);
		}
		reg_ctrl = IS_VALID_PANEL_REG_ID(ctx->desc->reg_ctrl_enable[0].id) ?
			ctx->desc->reg_ctrl_enable : default_ctrl_enable;
	} else {
		exynos_panel_pre_power_off(ctx);
		if (!IS_ERR_OR_NULL(ctx->reset_gpio))
			gpiod_set_value(ctx->reset_gpio, 0);
		if (!IS_ERR_OR_NULL(ctx->enable_gpio))
			gpiod_set_value(ctx->enable_gpio, 0);
		reg_ctrl = IS_VALID_PANEL_REG_ID(ctx->desc->reg_ctrl_disable[0].id) ?
			ctx->desc->reg_ctrl_disable : default_ctrl_disable;
	}

	return _exynos_panel_reg_ctrl(ctx, reg_ctrl, on);
}

int exynos_panel_set_power(struct exynos_panel *ctx, bool on)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return 0;

	if (funcs && funcs->set_power)
		ret = funcs->set_power(ctx, on);
	else
		ret = _exynos_panel_set_power(ctx, on);

	if (ret) {
		dev_err(ctx->dev, "failed to set power: ret %d \n", ret);
		return ret;
	}

	ctx->bl->props.power = on ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_set_power);

static void exynos_panel_post_power_on(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return;

	if (funcs && funcs->post_power_on)
		ret = funcs->post_power_on(ctx);
	else if (!IS_VALID_PANEL_REG_ID(ctx->desc->reg_ctrl_post_enable[0].id))
		return;
	else
		ret = _exynos_panel_reg_ctrl(ctx, ctx->desc->reg_ctrl_post_enable, true);

	if (ret)
		dev_err(ctx->dev, "failed to set post power on: ret %d\n", ret);
	else
		dev_dbg(ctx->dev, "set post power on\n");
}

static void exynos_panel_pre_power_off(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return;

	if (funcs && funcs->pre_power_off)
		ret = funcs->pre_power_off(ctx);
	else if (!IS_VALID_PANEL_REG_ID(ctx->desc->reg_ctrl_pre_disable[0].id))
		return;
	else
		ret = _exynos_panel_reg_ctrl(ctx, ctx->desc->reg_ctrl_pre_disable, false);

	if (ret)
		dev_err(ctx->dev, "failed to set pre power off: ret %d\n", ret);
	else
		dev_dbg(ctx->dev, "set pre power off\n");
}

static void exynos_panel_handoff(struct exynos_panel *ctx)
{
	ctx->enabled = gpiod_get_raw_value(ctx->reset_gpio) > 0;
	_exynos_panel_set_vddd_voltage(ctx, false);
	if (ctx->enabled) {
		dev_info(ctx->dev, "panel enabled at boot\n");
		ctx->panel_state = PANEL_STATE_HANDOFF;
		ctx->is_brightness_initialized = true;
		exynos_panel_set_power(ctx, true);
		/* We don't do panel reset while booting, so call post power here */
		exynos_panel_post_power_on(ctx);
	} else {
		ctx->panel_state = PANEL_STATE_UNINITIALIZED;
		gpiod_direction_output(ctx->reset_gpio, 0);
	}

	if (ctx->desc && ctx->desc->num_modes > 0 &&
		ctx->panel_state == PANEL_STATE_HANDOFF) {
		int i;

		for (i = 0; i < ctx->desc->num_modes; i++) {
			const struct exynos_panel_mode *pmode;

			pmode = &ctx->desc->modes[i];
			if (pmode->mode.type & DRM_MODE_TYPE_PREFERRED) {
				ctx->current_mode = pmode;
				break;
			}
		}
		if (ctx->current_mode == NULL) {
			ctx->current_mode = &ctx->desc->modes[0];
			i = 0;
		}
		dev_info(ctx->dev, "set default panel mode[%d]: %s\n",
			i, ctx->current_mode->mode.name[0] ?
			ctx->current_mode->mode.name : "NA");
	}
}

static int exynos_panel_parse_dt(struct exynos_panel *ctx)
{
	int ret = 0;
	u32 orientation = DRM_MODE_PANEL_ORIENTATION_NORMAL;

	if (IS_ERR_OR_NULL(ctx->dev->of_node)) {
		dev_err(ctx->dev, "no device tree information of exynos panel\n");
		return -EINVAL;
	}

	ret = exynos_panel_parse_gpios(ctx);
	if (ret)
		goto err;

	if (ctx->desc && ctx->desc->exynos_panel_func &&
				ctx->desc->exynos_panel_func->parse_regulators)
		ret = ctx->desc->exynos_panel_func->parse_regulators(ctx);
	else
		ret = exynos_panel_parse_regulators(ctx);
	if (ret)
		goto err;

	ctx->touch_dev = of_parse_phandle(ctx->dev->of_node, "touch", 0);

	of_property_read_u32(ctx->dev->of_node, "orientation", &orientation);
	if (orientation > DRM_MODE_PANEL_ORIENTATION_RIGHT_UP) {
		dev_warn(ctx->dev, "invalid display orientation %d\n", orientation);
		orientation = DRM_MODE_PANEL_ORIENTATION_NORMAL;
	}
	ctx->orientation = orientation;

err:
	return ret;
}

static irqreturn_t panel_rdy_irq_handler(int irq, void *dev_data)
{
	struct exynos_panel *ctx = dev_data;

	if (!ctx)
		return IRQ_HANDLED;

	complete_all(&ctx->ready_signal.detected);

	return IRQ_HANDLED;
}

static int exynos_panel_register_irqs(struct exynos_panel *ctx)
{
	struct platform_device *pdev;
	int ret = 0, irq;

	pdev = container_of(ctx->dev, struct platform_device, dev);

	/* 1: Panel ready */
	init_completion(&ctx->ready_signal.detected);
	if (gpio_is_valid(ctx->ready_signal.gpio)) {
		ret = gpio_to_irq(ctx->ready_signal.gpio);
		if (ret < 0) {
			dev_err(ctx->dev, "Failed to get irq number for rdy_gpio: %d\n", ret);
			return ret;
		}

		irq = ret;
		irq_set_status_flags(irq, IRQ_DISABLE_UNLAZY);
		ret = devm_request_irq(ctx->dev, irq, panel_rdy_irq_handler,
				       (ctx->ready_signal.gpio_flags & OF_GPIO_ACTIVE_LOW ?
						IRQF_TRIGGER_FALLING :
						IRQF_TRIGGER_RISING),
				       pdev->name, ctx);
		if (ret) {
			dev_err(ctx->dev, "Request rdy irq number(%d) failed: %d\n", irq, ret);
			return ret;
		}
		disable_irq(irq);

		ctx->ready_signal.irq = irq;
		dev_info(ctx->dev, "Request rdy irq number(%d) okay (%sING_TRIGGER)\n", irq,
			 (ctx->ready_signal.gpio_flags & OF_GPIO_ACTIVE_LOW ? "FALL" : "RIS"));
	} else {
		/* Panel doesn't have the ready pin */
		ctx->ready_signal.irq = -ENOENT;
		dev_dbg(ctx->dev, "No rdy-gpios specified for panel, skip irq registration\n");
	}

	return ret;
}

static void exynos_panel_mode_set_name(struct drm_display_mode *mode)
{
	scnprintf(mode->name, DRM_DISPLAY_MODE_LEN, "%ux%ux%d@%d",
		  mode->hdisplay, mode->vdisplay,
		  drm_mode_vrefresh(mode), exynos_drm_mode_te_freq(mode));
}

const struct exynos_panel_mode *exynos_panel_get_mode(struct exynos_panel *ctx,
							     const struct drm_display_mode *mode)
{
	const struct exynos_panel_mode *pmode;
	int i;

	for (i = 0; i < ctx->desc->num_modes; i++) {
		pmode = &ctx->desc->modes[i];

		if (drm_mode_equal(&pmode->mode, mode))
			return pmode;
	}

	pmode = ctx->desc->lp_mode;
	if (pmode) {
		const size_t count = ctx->desc->lp_mode_count ? : 1;

		for (i = 0; i < count; i++, pmode++)
			if (drm_mode_equal(&pmode->mode, mode))
				return pmode;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(exynos_panel_get_mode);

int exynos_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	struct drm_display_mode *preferred_mode = NULL;
	int i;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	for (i = 0; i < ctx->desc->num_modes; i++) {
		const struct exynos_panel_mode *pmode = &ctx->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &pmode->mode);
		if (!mode)
			return -ENOMEM;

		if (!mode->name[0])
			exynos_panel_mode_set_name(mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);

		dev_dbg(ctx->dev, "added display mode: %s\n", mode->name);

		if (!preferred_mode || (mode->type & DRM_MODE_TYPE_PREFERRED))
			preferred_mode = mode;
	}

	if (preferred_mode) {
		dev_dbg(ctx->dev, "preferred display mode: %s\n", preferred_mode->name);
		preferred_mode->type |= DRM_MODE_TYPE_PREFERRED;
		connector->display_info.width_mm = preferred_mode->width_mm;
		connector->display_info.height_mm = preferred_mode->height_mm;
	}

	dev_dbg(ctx->dev, "%s -\n", __func__);

	return i;
}
EXPORT_SYMBOL_GPL(exynos_panel_get_modes);

static void _exynos_panel_disable_normal_feat_locked(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	bool is_lhbm_enabled = !is_local_hbm_disabled(ctx);
	bool is_hbm_enabled = IS_HBM_ON(ctx->hbm_mode);

	if (is_lhbm_enabled && funcs && funcs->set_local_hbm_mode) {
		ctx->hbm.local_hbm.requested_state = LOCAL_HBM_DISABLED;
		panel_update_local_hbm_locked(ctx);
		/* restore the state while calling restore function */
		ctx->hbm.local_hbm.requested_state = LOCAL_HBM_ENABLED;
	}
	/* TODO: restore hbm if needed */
	if (is_hbm_enabled && funcs && funcs->set_hbm_mode)
		funcs->set_hbm_mode(ctx, HBM_OFF);

	if (!is_lhbm_enabled && !is_hbm_enabled)
		return;

	dev_warn(ctx->dev,
		"%s: unexpected lhbm(%d) or hbm(%d) @ %s, force off to avoid unpredictable issue\n",
		__func__, is_lhbm_enabled, is_hbm_enabled, (!ctx->enabled) ? "OFF" : "ON or LP");
}

int exynos_panel_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	ctx->enabled = false;
	ctx->dimming_on = false;
	ctx->self_refresh_active = false;
	ctx->panel_idle_vrefresh = 0;
	ctx->current_binned_lp = NULL;
	ctx->cabc_mode = CABC_OFF;
	ctx->ssc_en = false;

	mutex_lock(&ctx->mode_lock);
	_exynos_panel_disable_normal_feat_locked(ctx);
	exynos_panel_send_cmd_set(ctx, ctx->desc->off_cmd_set);
	mutex_unlock(&ctx->mode_lock);
	dev_dbg(ctx->dev, "%s\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_disable);

int exynos_panel_unprepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	exynos_panel_set_power(ctx, false);
	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_unprepare);

int exynos_panel_prepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	exynos_panel_set_power(ctx, true);
	dev_dbg(ctx->dev, "%s -\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_prepare);

void exynos_panel_send_cmd_set_flags(struct exynos_panel *ctx,
				     const struct exynos_dsi_cmd_set *cmd_set, u32 flags)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const struct exynos_dsi_cmd *c;
	const struct exynos_dsi_cmd *last_cmd = NULL;
	const u32 async_mask = PANEL_CMD_SET_BATCH | PANEL_CMD_SET_QUEUE;
	u16 dsi_flags = 0;

	if (!cmd_set || !cmd_set->num_cmd)
		return;

	/* shouldn't have both queue and batch set together */
	WARN_ON((flags & async_mask) == async_mask);

	if (flags & PANEL_CMD_SET_IGNORE_VBLANK)
		dsi_flags |= EXYNOS_DSI_MSG_IGNORE_VBLANK;

	/* if not batched or queued, all commands should be sent out immediately */
	if (flags & async_mask)
		dsi_flags |= EXYNOS_DSI_MSG_QUEUE;

	c = &cmd_set->cmds[cmd_set->num_cmd - 1];
	if (!c->panel_rev) {
		last_cmd = c;
	} else {
		for (; c >= cmd_set->cmds; c--) {
			if (c->panel_rev & ctx->panel_rev) {
				last_cmd = c;
				break;
			}
		}
	}

	/* no commands to transfer */
	if (!last_cmd)
		return;

	for (c = cmd_set->cmds; c <= last_cmd; c++) {
		u32 delay_ms = c->delay_ms;

		if (ctx->panel_rev && !(c->panel_rev & ctx->panel_rev))
			continue;

		if ((c == last_cmd) && !(flags & PANEL_CMD_SET_QUEUE))
			dsi_flags &= ~EXYNOS_DSI_MSG_QUEUE;

		host_to_dsi(dsi->host)->tx_delay_ms = delay_ms;
		exynos_dsi_dcs_write_buffer(dsi, c->cmd, c->cmd_len, dsi_flags);
		if (delay_ms)
			usleep_range(delay_ms * 1000, delay_ms * 1000 + 10);
	}
}
EXPORT_SYMBOL_GPL(exynos_panel_send_cmd_set_flags);

void exynos_panel_set_lp_mode(struct exynos_panel *ctx, const struct exynos_panel_mode *pmode)
{
	exynos_panel_send_cmd_set(ctx, ctx->desc->lp_cmd_set);

	dev_info(ctx->dev, "enter %dhz LP mode\n", drm_mode_vrefresh(&pmode->mode));
}
EXPORT_SYMBOL_GPL(exynos_panel_set_lp_mode);

void exynos_panel_set_binned_lp(struct exynos_panel *ctx, const u16 brightness)
{
	int i;
	const struct exynos_binned_lp *binned_lp;
	struct backlight_device *bl = ctx->bl;
	bool is_lp_state;
	enum exynos_panel_state panel_state;

	for (i = 0; i < ctx->desc->num_binned_lp; i++) {
		binned_lp = &ctx->desc->binned_lp[i];
		if (brightness <= binned_lp->bl_threshold)
			break;
	}
	if (i == ctx->desc->num_binned_lp)
		return;

	mutex_lock(&ctx->bl_state_lock);
	is_lp_state = is_backlight_lp_state(bl);
	mutex_unlock(&ctx->bl_state_lock);

	mutex_lock(&ctx->lp_state_lock);

	if (is_lp_state && ctx->current_binned_lp &&
	    binned_lp->bl_threshold == ctx->current_binned_lp->bl_threshold) {
		mutex_unlock(&ctx->lp_state_lock);
		return;
	}

	exynos_panel_send_cmd_set(ctx, &binned_lp->cmd_set);

	ctx->current_binned_lp = binned_lp;
	dev_dbg(ctx->dev, "enter lp_%s\n", ctx->current_binned_lp->name);

	mutex_unlock(&ctx->lp_state_lock);

	panel_state = !binned_lp->bl_threshold ? PANEL_STATE_BLANK : PANEL_STATE_LP;
	exynos_panel_set_backlight_state(ctx, panel_state);

	if (bl)
		sysfs_notify(&bl->dev.kobj, NULL, "lp_state");

	if (panel_state == PANEL_STATE_LP)
		exynos_panel_update_te2(ctx);
}
EXPORT_SYMBOL_GPL(exynos_panel_set_binned_lp);

int exynos_panel_init_brightness(struct exynos_panel_desc *desc,
				const struct exynos_brightness_configuration *configs,
				u32 num_configs, u32 panel_rev)
{
	const struct exynos_brightness_configuration *matched_config;
	int i;

	if (!desc || !configs)
		return -EINVAL;

	matched_config = configs;

	if (panel_rev) {
		for (i = 0; i < num_configs; i++, configs++) {
			if (configs->panel_rev & panel_rev) {
				matched_config = configs;
				break;
			}
		}
	}

	desc->max_brightness = matched_config->brt_capability.hbm.level.max;
	desc->min_brightness = matched_config->brt_capability.normal.level.min;
	desc->dft_brightness = matched_config->dft_brightness,
	desc->brt_capability = &(matched_config->brt_capability);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_panel_init_brightness);

int exynos_panel_set_brightness(struct exynos_panel *exynos_panel, u16 br)
{
	u16 brightness;

	if (exynos_panel->current_mode->exynos_mode.is_lp_mode) {
		const struct exynos_panel_funcs *funcs;

		funcs = exynos_panel->desc->exynos_panel_func;
		if (funcs && funcs->set_binned_lp)
			funcs->set_binned_lp(exynos_panel, br);
		return 0;
	}

	brightness = (br & 0xff) << 8 | br >> 8;

	return exynos_dcs_set_brightness(exynos_panel, brightness);
}
EXPORT_SYMBOL_GPL(exynos_panel_set_brightness);

static int exynos_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

u16 exynos_panel_get_brightness(struct exynos_panel *exynos_panel)
{
	return exynos_get_brightness(exynos_panel->bl);
}
EXPORT_SYMBOL_GPL(exynos_panel_get_brightness);

static int exynos_bl_find_range(struct exynos_panel *ctx,
				int brightness, u32 *range)
{
	u32 i;

	if (!brightness) {
		*range = 0;
		return 0;
	}

	mutex_lock(&ctx->bl_state_lock);
	if (!ctx->bl_notifier.num_ranges) {
		mutex_unlock(&ctx->bl_state_lock);
		return -EOPNOTSUPP;
	}

	for (i = 0; i < ctx->bl_notifier.num_ranges; i++) {
		if (brightness <= ctx->bl_notifier.ranges[i])
			break;
	}
	mutex_unlock(&ctx->bl_state_lock);

	*range = i + 1;
	return 0;
}

/**
 * exynos_panel_get_state_str - get readable string for panel state
 * @state: panel state enum
 *
 * convert enum exynos_panel_state into readable panel state string.
 */
static const char *exynos_panel_get_state_str(enum exynos_panel_state state)
{
	static const char *state_str[PANEL_STATE_COUNT] = {
		[PANEL_STATE_UNINITIALIZED] = "UN-INIT",
		[PANEL_STATE_HANDOFF] = "HANDOFF",
		[PANEL_STATE_HANDOFF_MODESET] = "HANDOFF-MODESET",
		[PANEL_STATE_OFF] = "OFF",
		[PANEL_STATE_NORMAL] = "ON",
		[PANEL_STATE_LP] = "LP",
		[PANEL_STATE_MODESET] = "MODESET",
		[PANEL_STATE_BLANK] = "BLANK",
	};

	if (state >= PANEL_STATE_COUNT)
		return "UNKNOWN";
	return state_str[state];
}

static int exynos_update_status(struct backlight_device *bl)
{
	struct exynos_panel *ctx = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int min_brightness = ctx->desc->min_brightness ? : 1;
	u32 bl_range = 0;

	if (!is_panel_active(ctx)) {
		dev_dbg(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	if (!ctx->is_brightness_initialized) {
		if (brightness == 0)
			return 0;
		ctx->is_brightness_initialized = true;
	}
	/* check if backlight is forced off */
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	min_brightness =
		ctx->desc->lower_min_brightness ? ctx->desc->lower_min_brightness : min_brightness;
	if (brightness && brightness < min_brightness)
		brightness = min_brightness;

	dev_info(ctx->dev, "req: %d, br: %d\n", bl->props.brightness,
		brightness);

	mutex_lock(&ctx->mode_lock);
	if (ctx->panel.backlight && !ctx->bl_ctrl_dcs) {
		backlight_device_set_brightness(ctx->panel.backlight,
			brightness);
	} else if (ctx->desc->exynos_panel_func) {
		const struct exynos_panel_funcs *funcs =
			ctx->desc->exynos_panel_func;

		if (funcs->set_brightness)
			funcs->set_brightness(ctx, brightness);
	} else {
		exynos_dcs_set_brightness(ctx, brightness);
	}

	if (!ctx->hbm_mode &&
	    !exynos_bl_find_range(ctx, brightness, &bl_range) &&
	    bl_range != ctx->bl_notifier.current_range) {
		ctx->bl_notifier.current_range = bl_range;
		schedule_work(&ctx->notify_brightness_changed_work);
		dev_dbg(ctx->dev, "bl range is changed to %d\n", bl_range);
	}
	mutex_unlock(&ctx->mode_lock);
	return 0;
}

static const struct backlight_ops exynos_backlight_ops = {
	.get_brightness = exynos_get_brightness,
	.update_status = exynos_update_status,
};

static ssize_t serial_number_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	if (!strcmp(ctx->panel_id, ""))
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_id);
}

static ssize_t panel_extinfo_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_extinfo);
}

static ssize_t panel_name_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const char *p;

	/* filter priority info in the dsi device name */
	p = strstr(dsi->name, ":");
	if (!p)
		p = dsi->name;
	else
		p++;

	return snprintf(buf, PAGE_SIZE, "%s\n", p);
}

static ssize_t panel_model_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_model);
}

static ssize_t gamma_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs;
	size_t len, ret;
	char *input_buf;
	const char *out_buf;

	if (!is_panel_active(ctx))
		return -EPERM;

	funcs = ctx->desc->exynos_panel_func;
	if (!funcs || !funcs->gamma_store)
		return -EOPNOTSUPP;

	if (!strncmp(buf, DEFAULT_GAMMA_STR, strlen(DEFAULT_GAMMA_STR))) {
		if (!funcs->restore_native_gamma)
			return -EOPNOTSUPP;
		else
			ret = funcs->restore_native_gamma(ctx);

		return ret ? : count;
	}

	input_buf = kstrndup(buf, count, GFP_KERNEL);
	if (!input_buf)
		return -ENOMEM;

	len = exynos_panel_parse_byte_buf(input_buf, count, &out_buf);
	kfree(input_buf);
	if (len <= 0)
		return len;

	ret = funcs->gamma_store(ctx, out_buf, len);
	kfree(out_buf);

	return ret ? : count;
}

static ssize_t set_te2_timing(struct exynos_panel *ctx, size_t count,
			      const char *buf, bool lp_mode)
{
	char *buf_dup;
	ssize_t type_len, data_len;
	u32 timing[MAX_TE2_TYPE * 2] = {0};
	const struct exynos_panel_funcs *funcs;

	if (!is_panel_active(ctx))
		return -EPERM;

	funcs = ctx->desc->exynos_panel_func;
	if (!funcs || !funcs->configure_te2_edges || !funcs->update_te2)
		return -EINVAL;

	if (!count)
		return -EINVAL;

	buf_dup = kstrndup(buf, count, GFP_KERNEL);
	if (!buf_dup)
		return -ENOMEM;

	type_len = exynos_get_te2_type_len(ctx, lp_mode);
	data_len = parse_u32_buf(buf_dup, count + 1, timing, type_len * 2);
	if (data_len != type_len * 2) {
		dev_warn(ctx->dev,
			 "invalid number of TE2 %s timing: expected %ld but actual %ld\n",
			 lp_mode ? "LP" : "normal",
			 type_len * 2, data_len);
		kfree(buf_dup);
		return -EINVAL;
	}

	mutex_lock(&ctx->mode_lock);
	funcs->configure_te2_edges(ctx, timing, lp_mode);
	exynos_panel_update_te2(ctx);
	mutex_unlock(&ctx->mode_lock);

	kfree(buf_dup);

	return count;
}

static ssize_t get_te2_timing(struct exynos_panel *ctx, char *buf, bool lp_mode)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	size_t len;

	if (!funcs || !funcs->get_te2_edges)
		return -EPERM;

	mutex_lock(&ctx->mode_lock);
	len = funcs->get_te2_edges(ctx, buf, lp_mode);
	mutex_unlock(&ctx->mode_lock);

	return len;
}

static ssize_t te2_timing_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!is_panel_initialized(ctx))
		return -EAGAIN;

	ret = set_te2_timing(ctx, count, buf, false);
	if (ret < 0)
		dev_err(ctx->dev,
			"failed to set normal mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_timing_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!is_panel_initialized(ctx))
		return -EAGAIN;

	ret = get_te2_timing(ctx, buf, false);
	if (ret < 0)
		dev_err(ctx->dev,
			"failed to get normal mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_lp_timing_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!is_panel_initialized(ctx))
		return -EAGAIN;

	ret = set_te2_timing(ctx, count, buf, true);
	if (ret < 0)
		dev_err(ctx->dev,
			"failed to set LP mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_lp_timing_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!ctx->initialized)
		return -EAGAIN;

	ret = get_te2_timing(ctx, buf, true);
	if (ret < 0)
		dev_err(ctx->dev,
			"failed to get LP mode TE2 timing: ret %ld\n", ret);

	return ret;
}

unsigned int panel_get_idle_time_delta(struct exynos_panel *ctx)
{
	const ktime_t now = ktime_get();
	const enum exynos_panel_idle_mode idle_mode = (ctx->current_mode) ?
					ctx->current_mode->idle_mode : IDLE_MODE_UNSUPPORTED;
	unsigned int delta_ms = UINT_MAX;

	if (idle_mode == IDLE_MODE_ON_INACTIVITY) {
		delta_ms = ktime_ms_delta(now, ctx->last_mode_set_ts);
	} else if (idle_mode == IDLE_MODE_ON_SELF_REFRESH) {
		const ktime_t ts = max3(ctx->last_self_refresh_active_ts,
					ctx->last_mode_set_ts, ctx->last_panel_idle_set_ts);

		delta_ms = ktime_ms_delta(now, ts);
	} else {
		dev_dbg(ctx->dev, "%s: unsupported idle mode %d", __func__, idle_mode);
	}

	return delta_ms;
}
EXPORT_SYMBOL_GPL(panel_get_idle_time_delta);

static bool panel_idle_queue_delayed_work(struct exynos_panel *ctx)
{
	const unsigned int delta_ms = panel_get_idle_time_delta(ctx);

	if (delta_ms < ctx->idle_delay_ms) {
		const unsigned int delay_ms = ctx->idle_delay_ms - delta_ms;

		dev_dbg(ctx->dev, "%s: last mode %ums ago, schedule idle in %ums\n",
			__func__, delta_ms, delay_ms);

		mod_delayed_work(system_highpri_wq, &ctx->idle_work,
					msecs_to_jiffies(delay_ms));
		return true;
	}

	return false;
}

static void panel_update_idle_mode_locked(struct exynos_panel *ctx, bool allow_delay_update)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	WARN_ON(!mutex_is_locked(&ctx->mode_lock));

	if (unlikely(!ctx->current_mode || !funcs))
		return;

	if (!is_panel_active(ctx) || !funcs->set_self_refresh)
		return;

	if (ctx->idle_delay_ms && ctx->self_refresh_active && panel_idle_queue_delayed_work(ctx))
		return;

	if (!ctx->self_refresh_active && allow_delay_update) {
		// delay update idle mode to next commit
		ctx->panel_update_idle_mode_pending = true;
		return;
	}

	ctx->panel_update_idle_mode_pending = false;
	if (delayed_work_pending(&ctx->idle_work)) {
		dev_dbg(ctx->dev, "%s: cancelling delayed idle work\n", __func__);
		cancel_delayed_work(&ctx->idle_work);
	}

	if (funcs->set_self_refresh(ctx, ctx->self_refresh_active)) {
		exynos_panel_update_te2(ctx);
		ctx->last_self_refresh_active_ts = ktime_get();
	}
}

static void panel_idle_work(struct work_struct *work)
{
	struct exynos_panel *ctx = container_of(work, struct exynos_panel, idle_work.work);

	dev_dbg(ctx->dev, "%s\n", __func__);

	mutex_lock(&ctx->mode_lock);
	panel_update_idle_mode_locked(ctx, false);
	mutex_unlock(&ctx->mode_lock);
}

static ssize_t panel_idle_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	char str_trace[] = "panel_idle_store_0";
	u8 idx_str_en = strlen(str_trace) - 1;
	bool idle_enabled;
	int ret;

	ret = kstrtobool(buf, &idle_enabled);
	if (ret) {
		dev_err(dev, "invalid panel idle value\n");
		return ret;
	}

	str_trace[idx_str_en] = idle_enabled ? '1' : '0';
	DPU_ATRACE_BEGIN(str_trace);
	mutex_lock(&ctx->mode_lock);
	if (idle_enabled != ctx->panel_idle_enabled) {
		ctx->panel_idle_enabled = idle_enabled;

		if (idle_enabled)
			ctx->last_panel_idle_set_ts = ktime_get();

		panel_update_idle_mode_locked(ctx, true);
	}
	mutex_unlock(&ctx->mode_lock);
	DPU_ATRACE_END(str_trace);

	return count;
}

static ssize_t panel_idle_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->panel_idle_enabled);
}

static ssize_t panel_need_handle_idle_exit_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	bool idle_handle_exit;
	int ret;

	ret = kstrtobool(buf, &idle_handle_exit);
	if (ret) {
		dev_err(dev, "invalid panel idle handle exit value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->panel_need_handle_idle_exit = idle_handle_exit;
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t panel_need_handle_idle_exit_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->panel_need_handle_idle_exit);
}

static ssize_t min_vrefresh_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int min_vrefresh;
	int ret;

	ret = kstrtoint(buf, 0, &min_vrefresh);
	if (ret) {
		dev_err(dev, "invalid min vrefresh value\n");
		return ret;
	}

	DPU_ATRACE_BEGIN(__func__);
	mutex_lock(&ctx->mode_lock);
	if (ctx->min_vrefresh != min_vrefresh) {
		ctx->min_vrefresh = min_vrefresh;
		panel_update_idle_mode_locked(ctx, true);
	}
	mutex_unlock(&ctx->mode_lock);
	DPU_ATRACE_END(__func__);

	return count;
}

static ssize_t min_vrefresh_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->min_vrefresh);
}

static ssize_t idle_delay_ms_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	u32 idle_delay_ms;
	int ret;

	ret = kstrtou32(buf, 0, &idle_delay_ms);
	if (ret) {
		dev_err(dev, "invalid idle delay ms\n");
		return ret;
	}

	DPU_ATRACE_BEGIN(__func__);
	mutex_lock(&ctx->mode_lock);
	if (ctx->idle_delay_ms != idle_delay_ms) {
		ctx->idle_delay_ms = idle_delay_ms;
		panel_update_idle_mode_locked(ctx, true);
	}
	mutex_unlock(&ctx->mode_lock);
	DPU_ATRACE_END(__func__);

	return count;
}

static ssize_t idle_delay_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->idle_delay_ms);
}

static ssize_t force_power_on_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	bool force_on;
	int ret;

	ret = kstrtobool(buf, &force_on);
	if (ret) {
		dev_err(dev, "invalid force_power_on value\n");
		return ret;
	}

	drm_modeset_lock(&ctx->bridge.base.lock, NULL);
	if (force_on && ctx->panel_state == PANEL_STATE_OFF) {
		drm_panel_prepare(&ctx->panel);
		ctx->panel_state = PANEL_STATE_BLANK;
	}

	ctx->force_power_on = force_on;
	drm_modeset_unlock(&ctx->bridge.base.lock);

	return count;
}

static ssize_t force_power_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->force_power_on);
}

static ssize_t osc2_clk_khz_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	unsigned int osc2_clk_khz;
	int ret;

	if (!funcs || !funcs->set_osc2_clk_khz)
		return -EOPNOTSUPP;

	ret = kstrtou32(buf, 0, &osc2_clk_khz);
	if (ret) {
		dev_err(dev, "invalid osc2 clock value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	if (osc2_clk_khz != ctx->osc2_clk_khz)
		funcs->set_osc2_clk_khz(ctx, osc2_clk_khz);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t osc2_clk_khz_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctx->osc2_clk_khz);
}

static ssize_t available_osc2_clk_khz_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	ssize_t len;

	if (!funcs || !funcs->list_osc2_clk_khz)
		return -EPERM;

	len = funcs->list_osc2_clk_khz(ctx, buf);
	if (len < 0)
		dev_err(dev, "failed to list OSC2 clocks (%ld)\n", len);

	return len;
}

static int exynos_panel_set_op_hz(struct exynos_panel *ctx, unsigned int hz)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret = 0;

	if (!is_panel_initialized(ctx))
		return -EAGAIN;

	if (!funcs || !funcs->set_op_hz)
		return -EINVAL;

	mutex_lock(&ctx->mode_lock);
	if (ctx->op_hz != hz) {
		ret = funcs->set_op_hz(ctx, hz);
		if (ret) {
			dev_err(ctx->dev, "failed to set op rate: %u Hz\n", hz);
		} else {
			DPU_ATRACE_BEGIN("notify_op_hz");
			blocking_notifier_call_chain(&ctx->notifier_head,
						     EXYNOS_PANEL_NOTIFIER_SET_OP_HZ, &ctx->op_hz);
			DPU_ATRACE_END("notify_op_hz");
			sysfs_notify(&ctx->dev->kobj, NULL, "op_hz");
		}
	} else {
		dev_dbg(ctx->dev, "%s: skip the same op rate: %u Hz\n", __func__, hz);
	}
	mutex_unlock(&ctx->mode_lock);

	return ret;
}

static ssize_t op_hz_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;
	u32 hz;

	if (!count)
		return -EINVAL;

	ret = kstrtou32(buf, 0, &hz);
	if (ret) {
		dev_err(ctx->dev, "invalid op_hz value\n");
		return ret;
	}

	ret = exynos_panel_set_op_hz(ctx, hz);
	if (ret)
		return ret;

	return count;
}

static ssize_t op_hz_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!is_panel_initialized(ctx))
		return -EAGAIN;

	if (!funcs || !funcs->set_op_hz)
		return -EINVAL;

	dev_dbg(ctx->dev, "%s: %u\n", __func__, ctx->op_hz);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctx->op_hz);
}

static ssize_t refresh_rate_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_mode *current_mode;
	int rr = -1;

	mutex_lock(&ctx->mode_lock);
	current_mode = ctx->current_mode;
	if (current_mode != NULL)
		rr = drm_mode_vrefresh(&current_mode->mode);
	mutex_unlock(&ctx->mode_lock);

	return scnprintf(buf, PAGE_SIZE, "%d\n", rr);
}

static ssize_t refresh_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	ssize_t ret;
	u32 ctrl;

	if (!count)
		return -EINVAL;

	if (!is_panel_initialized(ctx))
		return -EAGAIN;

	if (!is_panel_active(ctx))
		return -EPERM;

	if (!funcs || !funcs->refresh_ctrl)
		return -EINVAL;

	ret = kstrtou32(buf, 0, &ctrl);
	if (ret || !(ctrl & PANEL_REFRESH_CTRL_MASK)) {
		return -EINVAL;
	}

	mutex_lock(&ctx->mode_lock);
	funcs->refresh_ctrl(ctx, ctrl);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t error_count_te_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	u32 count;
	mutex_lock(&ctx->mode_lock);
	count = ctx->error_count_te;
	mutex_unlock(&ctx->mode_lock);

	return scnprintf(buf, PAGE_SIZE, "%u\n", count);
}

static ssize_t error_count_unknown_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	u32 count;
	mutex_lock(&ctx->mode_lock);
	count = ctx->error_count_unknown;
	mutex_unlock(&ctx->mode_lock);

	return scnprintf(buf, PAGE_SIZE, "%u\n", count);
}

static ssize_t panel_pwr_vreg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!funcs || !funcs->get_pwr_vreg)
		return -EOPNOTSUPP;

	mutex_lock(&ctx->mode_lock);
	funcs->get_pwr_vreg(ctx, buf, PAGE_SIZE);
	mutex_unlock(&ctx->mode_lock);

	return strlcat(buf, "\n", PAGE_SIZE);
}

static inline int get_disp_stats_time_state_idx(struct exynos_panel *ctx,
		enum display_state state, int vrefresh, struct display_resolution res)
{
	struct display_stats *stats = &ctx->disp_stats;
	int i;
	int vrefresh_idx = -1, res_idx = -1, time_state_idx;
	int *vrefresh_range;
	size_t max_vrefresh_range_count;

	if (!stats->time_in_state[state].available_count) {
		dev_err(ctx->dev, "time state does not support %s\n",
			disp_state_str[state]);
		return -1;
	}

	if (state == DISPLAY_STATE_OFF)
		return 0;

	if (state == DISPLAY_STATE_LP) {
		vrefresh_range = stats->lp_vrefresh_range;
		max_vrefresh_range_count = stats->lp_vrefresh_range_count;
	} else {
		/* ON, HBM */
		vrefresh_range = stats->vrefresh_range;
		max_vrefresh_range_count = stats->vrefresh_range_count;
	}

	for (i = 0; i < stats->res_table_count; i++) {
		if (stats->res_table[i].hdisplay == res.hdisplay &&
			stats->res_table[i].vdisplay == res.vdisplay) {
			res_idx = i;
			break;
		}
	}

	if (res_idx < 0) {
		dev_err(ctx->dev, "time state does not support %ux%u on %s\n",
			res.hdisplay, res.vdisplay, disp_state_str[state]);
		return -1;
	}

	for (i = 0; i < max_vrefresh_range_count; i++) {
		if (vrefresh_range[i] == vrefresh) {
			vrefresh_idx = i;
			break;
		}
	}

	if (vrefresh_idx < 0) {
		dev_err(ctx->dev, "time state does not support %dhz on %s\n",
			vrefresh, disp_state_str[state]);
		return -1;
	}

	time_state_idx = res_idx * max_vrefresh_range_count + vrefresh_idx;
	if (time_state_idx >= stats->time_in_state[state].available_count) {
		dev_err(ctx->dev, "time state does not support %ux%u@%d on %s state\n",
			res.hdisplay, res.vdisplay, vrefresh, disp_state_str[state]);
		return -1;
	}

	return time_state_idx;
}

static ssize_t time_in_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	struct display_stats *stats = &ctx->disp_stats;
	int state, vrefresh_idx, res_idx, time_state_idx;
	u64 time, delta_ms;
	ssize_t len = 0;

	if (!stats->initialized)
		return -ENODEV;

	mutex_lock(&stats->lock);
	delta_ms = ktime_ms_delta(ktime_get_boottime(), stats->last_update);
	for (state = 0; state < DISPLAY_STATE_MAX; state++) {
		int *vrefresh_range;
		size_t vrefresh_range_count;

		if (!stats->time_in_state[state].available_count)
			continue;

		if (state == DISPLAY_STATE_OFF) {
			time = stats->time_in_state[state].time[0];
			if (stats->last_state == state)
				time += delta_ms;
			if (time) {
				len += sysfs_emit_at(buf, len, "%d 0 0 0 %llu\n",
				DISPLAY_STATE_OFF, time);
			}
			continue;
		}

		if (state == DISPLAY_STATE_LP) {
			vrefresh_range = stats->lp_vrefresh_range;
			vrefresh_range_count = stats->lp_vrefresh_range_count;
		} else {
			vrefresh_range = stats->vrefresh_range;
			vrefresh_range_count = stats->vrefresh_range_count;
		}
		for (res_idx = 0; res_idx < stats->res_table_count; res_idx++) {
			for (vrefresh_idx = 0; vrefresh_idx < vrefresh_range_count;
					vrefresh_idx++) {
				int vrefresh;

				vrefresh = vrefresh_range[vrefresh_idx];
				time_state_idx = get_disp_stats_time_state_idx(
					ctx, state, vrefresh, stats->res_table[res_idx]);
				if (time_state_idx < 0)
					continue;

				time = stats->time_in_state[state].time[time_state_idx];
				if (state == stats->last_state &&
					time_state_idx == stats->last_time_state_idx) {
					time += delta_ms;
				}
				if (!time)
					continue;

				len += sysfs_emit_at(buf, len,
					"%d %u %u %d %llu\n", state,
					stats->res_table[res_idx].hdisplay,
					stats->res_table[res_idx].vdisplay,
					vrefresh,
					time);
			}
		}
	}

	mutex_unlock(&stats->lock);
	return len;
}

static ssize_t available_disp_stats_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	struct display_stats *stats = &ctx->disp_stats;
	int state, vrefresh_idx, res_idx;
	ssize_t len = 0;

	if (!stats->initialized)
		return -ENODEV;

	mutex_lock(&stats->lock);
	for (state = 0; state < DISPLAY_STATE_MAX; state++) {
		int *vrefresh_range;
		size_t vrefresh_range_count;

		if (!stats->time_in_state[state].available_count)
			continue;

		if (state == DISPLAY_STATE_OFF) {
			len += sysfs_emit_at(buf, len, "%d 0 0 0\n", state);
			continue;
		}

		if (state == DISPLAY_STATE_LP) {
			vrefresh_range = stats->lp_vrefresh_range;
			vrefresh_range_count = stats->lp_vrefresh_range_count;
		} else {
			vrefresh_range = stats->vrefresh_range;
			vrefresh_range_count = stats->vrefresh_range_count;
		}
		for (res_idx = 0; res_idx < stats->res_table_count; res_idx++) {
			for (vrefresh_idx = 0; vrefresh_idx < vrefresh_range_count;
					vrefresh_idx++) {
				u16 hdisplay, vdisplay;
				int vrefresh;

				hdisplay = stats->res_table[res_idx].hdisplay;
				vdisplay = stats->res_table[res_idx].vdisplay;
				vrefresh = vrefresh_range[vrefresh_idx];
				len += sysfs_emit_at(buf, len, "%d %u %u %d\n",
						state, hdisplay, vdisplay, vrefresh);
			}
		}
	}

	mutex_unlock(&stats->lock);
	return len;
}

static ssize_t power_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int err;
	u8 power_mode;

	if (!is_panel_active(ctx)) {
		dev_warn(dev, "%s: panel is not enabled\n", __func__);
		return -EPERM;
	}

	mutex_lock(&ctx->mode_lock);
	err = mipi_dsi_dcs_read(dsi, MIPI_DCS_GET_POWER_MODE, &power_mode, sizeof(power_mode));
	mutex_unlock(&ctx->mode_lock);
	if (err <= 0) {
		if (err == 0)
			err = -ENODATA;
		dev_warn(dev, "Unable to read power mode register (%#02x: %d)\n",
			MIPI_DCS_GET_POWER_MODE, err);
		return err;
	}

	power_mode &= (MIPI_DSI_DCS_POWER_MODE_DISPLAY |
	    MIPI_DSI_DCS_POWER_MODE_NORMAL | MIPI_DSI_DCS_POWER_MODE_SLEEP);

	return sysfs_emit(buf, "%#02x\n", power_mode);
}

static ssize_t power_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	enum display_state state;

	mutex_lock(&ctx->bl_state_lock);
	state = get_current_display_state_locked(ctx);
	mutex_unlock(&ctx->bl_state_lock);

	return sysfs_emit(buf, "%s\n", disp_state_str[state]);
}

static DEVICE_ATTR_RO(serial_number);
static DEVICE_ATTR_RO(panel_extinfo);
static DEVICE_ATTR_RO(panel_name);
static DEVICE_ATTR_RO(panel_model);
static DEVICE_ATTR_WO(gamma);
static DEVICE_ATTR_RW(te2_timing);
static DEVICE_ATTR_RW(te2_lp_timing);
static DEVICE_ATTR_RW(panel_idle);
static DEVICE_ATTR_RW(panel_need_handle_idle_exit);
static DEVICE_ATTR_RW(min_vrefresh);
static DEVICE_ATTR_RW(idle_delay_ms);
static DEVICE_ATTR_RW(force_power_on);
static DEVICE_ATTR_RW(osc2_clk_khz);
static DEVICE_ATTR_RO(available_osc2_clk_khz);
static DEVICE_ATTR_RW(op_hz);
static DEVICE_ATTR_RO(refresh_rate);
static DEVICE_ATTR_WO(refresh_ctrl);
static DEVICE_ATTR_RO(error_count_te);
static DEVICE_ATTR_RO(error_count_unknown);
static DEVICE_ATTR_RO(panel_pwr_vreg);
static DEVICE_ATTR_RO(time_in_state);
static DEVICE_ATTR_RO(available_disp_stats);
static DEVICE_ATTR_RO(power_mode);
static DEVICE_ATTR_RO(power_state);

static const struct attribute *panel_attrs[] = {
	&dev_attr_serial_number.attr,
	&dev_attr_panel_extinfo.attr,
	&dev_attr_panel_name.attr,
	&dev_attr_panel_model.attr,
	&dev_attr_gamma.attr,
	&dev_attr_te2_timing.attr,
	&dev_attr_te2_lp_timing.attr,
	&dev_attr_panel_idle.attr,
	&dev_attr_panel_need_handle_idle_exit.attr,
	&dev_attr_min_vrefresh.attr,
	&dev_attr_idle_delay_ms.attr,
	&dev_attr_force_power_on.attr,
	&dev_attr_osc2_clk_khz.attr,
	&dev_attr_available_osc2_clk_khz.attr,
	&dev_attr_op_hz.attr,
	&dev_attr_refresh_rate.attr,
	&dev_attr_refresh_ctrl.attr,
	&dev_attr_error_count_te.attr,
	&dev_attr_error_count_unknown.attr,
	&dev_attr_panel_pwr_vreg.attr,
	&dev_attr_time_in_state.attr,
	&dev_attr_available_disp_stats.attr,
	&dev_attr_power_mode.attr,
	&dev_attr_power_state.attr,
	NULL
};

static void exynos_panel_connector_print_state(struct drm_printer *p,
					       const struct exynos_drm_connector_state *state)
{
	const struct exynos_drm_connector *exynos_connector =
		to_exynos_connector(state->base.connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	const struct exynos_panel_desc *desc = ctx->desc;
	int ret;

	ret = mutex_lock_interruptible(&ctx->mode_lock);
	if (ret)
		return;

	drm_printf(p, "\tpanel_state: %s\n", exynos_panel_get_state_str(ctx->panel_state));
	drm_printf(p, "\tidle: %s (%s)\n",
		   ctx->panel_idle_vrefresh ? "active" : "inactive",
		   ctx->panel_idle_enabled ? "enabled" : "disabled");

	if (ctx->current_mode) {
		const struct drm_display_mode *m = &ctx->current_mode->mode;

		drm_printf(p, " \tcurrent mode: %s te@%d\n", m->name, exynos_drm_mode_te_freq(m));
	}
	drm_printf(p, "\text_info: %s\n", ctx->panel_extinfo);
	drm_printf(p, "\tluminance: [%u, %u] avg: %u\n",
		   desc->min_luminance, desc->max_luminance,
		   desc->max_avg_luminance);
	drm_printf(p, "\thdr_formats: 0x%x\n", desc->hdr_formats);
	drm_printf(p, "\thbm_mode: %u\n", ctx->hbm_mode);
	drm_printf(p, "\tdimming_on: %s\n", ctx->dimming_on ? "true" : "false");
	drm_printf(p, "\tis_partial: %s\n", desc->is_partial ? "true" : "false");

	mutex_unlock(&ctx->mode_lock);
}

/**
 * is_umode_lp_compatible - check switching between provided modes can be seamless during LP
 * @pmode: initial display mode
 * @umode: target display mode
 *
 * Returns true if the switch to target mode can be seamless during LP
 */
static inline bool is_umode_lp_compatible(const struct exynos_panel_mode *pmode,
					  const struct drm_mode_modeinfo *umode)
{
	return pmode->mode.vdisplay == umode->vdisplay && pmode->mode.hdisplay == umode->hdisplay;
}

static int exynos_panel_get_lp_mode(struct exynos_drm_connector *exynos_conn,
				    const struct exynos_drm_connector_state *exynos_state,
				    uint64_t *val)
{
	const struct drm_connector_state *conn_state = &exynos_state->base;
	const struct drm_crtc_state *crtc_state = conn_state->crtc ? conn_state->crtc->state : NULL;
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_conn);
	struct drm_property_blob *blob = ctx->lp_mode_blob;
	const struct exynos_panel_mode *cur_mode;
	struct drm_mode_modeinfo umode;

	if (crtc_state)
		cur_mode = exynos_panel_get_mode(ctx, &crtc_state->mode);
	else
		cur_mode = READ_ONCE(ctx->current_mode);

	if (unlikely(!ctx->desc->lp_mode))
		return -EINVAL;

	if (blob) {
		if (!cur_mode || is_umode_lp_compatible(cur_mode, blob->data)) {
			dev_dbg(ctx->dev, "%s: returning existing lp mode blob\n", __func__);
			*val = blob->base.id;
			return 0;
		}
		ctx->lp_mode_blob = NULL;
		drm_property_blob_put(blob);
	}

	/* when mode count is 0, assume driver is only providing single LP mode */
	if (ctx->desc->lp_mode_count <= 1 || !cur_mode) {
		dev_dbg(ctx->dev, "%s: only single LP mode available\n", __func__);
		drm_mode_convert_to_umode(&umode, &ctx->desc->lp_mode->mode);
	} else {
		int i;

		for (i = 0; i < ctx->desc->lp_mode_count; i++) {
			const struct exynos_panel_mode *lp_mode = &ctx->desc->lp_mode[i];

			drm_mode_convert_to_umode(&umode, &lp_mode->mode);

			if (is_umode_lp_compatible(cur_mode, &umode)) {
				dev_dbg(ctx->dev, "%s: found lp mode: %s for mode:%s\n", __func__,
					lp_mode->mode.name, cur_mode->mode.name);
				break;
			}
		}

		if (i == ctx->desc->lp_mode_count) {
			dev_warn(ctx->dev, "%s: unable to find compatible LP mode for mode: %s\n",
				 __func__, cur_mode->mode.name);
			return -ENOENT;
		}
	}

	blob = drm_property_create_blob(exynos_conn->base.dev, sizeof(umode), &umode);
	if (IS_ERR(blob))
		return PTR_ERR(blob);

	ctx->lp_mode_blob = blob;
	*val = blob->base.id;

	return 0;
}

static int exynos_panel_connector_get_property(
				   struct exynos_drm_connector *exynos_connector,
				   const struct exynos_drm_connector_state *exynos_state,
				   struct drm_property *property,
				   uint64_t *val)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(exynos_connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);

	if (property == p->brightness_level) {
		*val = exynos_state->brightness_level;
		dev_dbg(ctx->dev, "%s: brt(%llu)\n", __func__, *val);
	} else if (property == p->global_hbm_mode) {
		*val = exynos_state->global_hbm_mode;
		dev_dbg(ctx->dev, "%s: global_hbm_mode(%llu)\n", __func__, *val);
	}  else if (property == p->local_hbm_on) {
		*val = exynos_state->local_hbm_on;
		dev_dbg(ctx->dev, "%s: local_hbm_on(%s)\n", __func__, *val ? "true" : "false");
	} else if (property == p->dimming_on) {
		*val = exynos_state->dimming_on;
		dev_dbg(ctx->dev, "%s: dimming_on(%s)\n", __func__, *val ? "true" : "false");
	} else if (property == p->operation_rate) {
		*val = exynos_state->operation_rate;
		dev_dbg(ctx->dev, "%s: operation_rate(%llu)\n", __func__, *val);
	} else if (property == p->lp_mode) {
		return exynos_panel_get_lp_mode(exynos_connector, exynos_state, val);
	} else if (property == p->mipi_sync) {
		*val = exynos_state->mipi_sync;
		dev_dbg(ctx->dev, "%s: mipi_sync(0x%llx)\n", __func__, *val);
	} else
		return -EINVAL;

	return 0;
}

static int exynos_panel_connector_set_property(
				   struct exynos_drm_connector *exynos_connector,
				   struct exynos_drm_connector_state *exynos_state,
				   struct drm_property *property,
				   uint64_t val)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(exynos_connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);

	if (property == p->brightness_level) {
		exynos_state->pending_update_flags |= HBM_FLAG_BL_UPDATE;
		exynos_state->brightness_level = val;
		dev_dbg(ctx->dev, "%s: brt(%u)\n", __func__, exynos_state->brightness_level);
	} else if (property == p->global_hbm_mode) {
		exynos_state->pending_update_flags |= HBM_FLAG_GHBM_UPDATE;
		exynos_state->global_hbm_mode = val;
		dev_dbg(ctx->dev, "%s: global_hbm_mode(%u)\n", __func__,
			 exynos_state->global_hbm_mode);
	} else if (property == p->local_hbm_on) {
		exynos_state->pending_update_flags |= HBM_FLAG_LHBM_UPDATE;
		exynos_state->local_hbm_on = val;
		dev_dbg(ctx->dev, "%s: local_hbm_on(%s)\n", __func__,
			 exynos_state->local_hbm_on ? "true" : "false");
	} else if (property == p->dimming_on) {
		exynos_state->pending_update_flags |= HBM_FLAG_DIMMING_UPDATE;
		exynos_state->dimming_on = val;
		dev_dbg(ctx->dev, "%s: dimming_on(%s)\n", __func__,
			 exynos_state->dimming_on ? "true" : "false");
	} else if (property == p->operation_rate) {
		exynos_state->pending_update_flags |= HBM_FLAG_OP_RATE_UPDATE;
		exynos_state->operation_rate = val;
		exynos_state->update_operation_rate_to_bts = true;
		dev_dbg(ctx->dev, "%s: operation_rate(%u)\n", __func__, exynos_state->operation_rate);
	} else if (property == p->mipi_sync) {
		exynos_state->mipi_sync = val;
		dev_dbg(ctx->dev, "%s: mipi_sync(0x%lx)\n", __func__, exynos_state->mipi_sync);
	} else
		return -EINVAL;

	return 0;
}

static int exynos_panel_connector_late_register(struct exynos_drm_connector *exynos_connector)
{
	exynos_panel_node_attach(exynos_connector);
	return 0;
}

static const struct exynos_drm_connector_funcs exynos_panel_connector_funcs = {
	.atomic_print_state = exynos_panel_connector_print_state,
	.atomic_get_property = exynos_panel_connector_get_property,
	.atomic_set_property = exynos_panel_connector_set_property,
	.late_register = exynos_panel_connector_late_register,
};

static void exynos_panel_set_dimming(struct exynos_panel *ctx, bool dimming_on)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!funcs || !funcs->set_dimming_on)
		return;

	mutex_lock(&ctx->mode_lock);
	if (dimming_on != ctx->dimming_on) {
		funcs->set_dimming_on(ctx, dimming_on);
		panel_update_idle_mode_locked(ctx, false);
	}
	mutex_unlock(&ctx->mode_lock);
}

static void exynos_panel_set_cabc(struct exynos_panel *ctx, enum exynos_cabc_mode cabc_mode)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!funcs || !funcs->set_cabc_mode)
		return;

	mutex_lock(&ctx->mode_lock);
	if (cabc_mode != ctx->cabc_mode)
		funcs->set_cabc_mode(ctx, cabc_mode);

	mutex_unlock(&ctx->mode_lock);
}

static void exynos_panel_lhbm_on_delay_frames(struct drm_crtc *crtc,
						struct exynos_panel *ctx)
{
	u64 last_vblank_cnt = ctx->hbm.local_hbm.last_lp_vblank_cnt;

	if (!ctx->desc->lhbm_on_delay_frames || !last_vblank_cnt)
		return;

	DPU_ATRACE_BEGIN("lhbm_on_delay_frames");
	if (crtc && !drm_crtc_vblank_get(crtc)) {
		int retry = ctx->desc->lhbm_on_delay_frames;

		do {
			u32 diff = 0;
			u64 cur_vblank_cnt = drm_crtc_vblank_count(crtc);

			if (cur_vblank_cnt > last_vblank_cnt)
				diff = cur_vblank_cnt - last_vblank_cnt;

			if (diff < ctx->desc->lhbm_on_delay_frames)
				drm_crtc_wait_one_vblank(crtc);
			else
				break;
		} while (--retry);
		drm_crtc_vblank_put(crtc);
	}
	ctx->hbm.local_hbm.last_lp_vblank_cnt = 0;
	DPU_ATRACE_END("lhbm_on_delay_frames");
}

static void exynos_panel_set_atc_config(struct exynos_panel *ctx,
					const struct decon_device *decon,
					struct exynos_drm_crtc_state *exynos_crtc_state,
					bool enable)
{
	decon->dqe->force_atc_config.dirty = true;
	decon->dqe->force_atc_config.en = enable;
	dev_info(ctx->dev, "set atc config %d\n", enable);
	exynos_atc_update(decon->dqe, &exynos_crtc_state->dqe);
}

static void exynos_panel_pre_commit_properties(
				struct exynos_panel *ctx,
				struct exynos_drm_connector_state *conn_state)
{
	const struct exynos_panel_funcs *exynos_panel_func = ctx->desc->exynos_panel_func;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	bool mipi_sync;
	bool ghbm_updated = false;
	const unsigned int normal_feat_flags = HBM_FLAG_GHBM_UPDATE | HBM_FLAG_LHBM_UPDATE;

	if (!conn_state->pending_update_flags)
		return;

	DPU_ATRACE_BEGIN(__func__);
	mipi_sync = conn_state->mipi_sync &
		(MIPI_CMD_SYNC_LHBM | MIPI_CMD_SYNC_GHBM | MIPI_CMD_SYNC_BL);

	if ((conn_state->pending_update_flags & normal_feat_flags) &&
		ctx->current_mode->exynos_mode.is_lp_mode) {
		dev_warn(ctx->dev,
			 "%s: skip LHBM/GHBM updates during lp mode, pending_update_flags(0x%x)\n",
			 __func__, conn_state->pending_update_flags);
		conn_state->pending_update_flags &= ~normal_feat_flags;
	}

	if (mipi_sync) {
		dev_info(ctx->dev, "%s: mipi_sync(0x%lx) pending_update_flags(0x%x)\n", __func__,
			 conn_state->mipi_sync, conn_state->pending_update_flags);
		if (conn_state->mipi_sync & MIPI_CMD_SYNC_LHBM)
			exynos_panel_lhbm_on_delay_frames(conn_state->base.crtc, ctx);

		exynos_panel_check_mipi_sync_timing(conn_state->base.crtc,
						    ctx->current_mode, ctx->current_mode, ctx);

		exynos_dsi_dcs_write_buffer_force_batch_begin(dsi);
	}

	if ((conn_state->pending_update_flags & HBM_FLAG_GHBM_UPDATE) &&
		exynos_panel_func && exynos_panel_func->set_hbm_mode &&
		(ctx->hbm_mode != conn_state->global_hbm_mode)) {
		DPU_ATRACE_BEGIN("set_hbm");
		mutex_lock(&ctx->mode_lock);
		exynos_panel_func->set_hbm_mode(ctx, conn_state->global_hbm_mode);
		notify_panel_mode_changed(ctx, false);
		mutex_unlock(&ctx->mode_lock);
		DPU_ATRACE_END("set_hbm");
		ghbm_updated = true;
	}

	if ((conn_state->pending_update_flags & HBM_FLAG_BL_UPDATE) &&
		(ctx->bl->props.brightness != conn_state->brightness_level)) {
		DPU_ATRACE_BEGIN("set_bl");
		ctx->bl->props.brightness = conn_state->brightness_level;
		backlight_update_status(ctx->bl);
		DPU_ATRACE_END("set_bl");
	}

	if ((conn_state->pending_update_flags & HBM_FLAG_LHBM_UPDATE) && exynos_panel_func &&
	    exynos_panel_func->set_local_hbm_mode) {
		DPU_ATRACE_BEGIN("set_lhbm");
		dev_info(ctx->dev, "%s: set LHBM to %d\n", __func__,
			conn_state->local_hbm_on);
		mutex_lock(&ctx->mode_lock);
		ctx->hbm.local_hbm.requested_state = conn_state->local_hbm_on ? LOCAL_HBM_ENABLED :
										LOCAL_HBM_DISABLED;
		panel_update_local_hbm_locked(ctx);
		mutex_unlock(&ctx->mode_lock);
		DPU_ATRACE_END("set_lhbm");
	}

	if ((conn_state->pending_update_flags & HBM_FLAG_DIMMING_UPDATE) &&
		exynos_panel_func && exynos_panel_func->set_dimming_on &&
		(ctx->dimming_on != conn_state->dimming_on)) {
		DPU_ATRACE_BEGIN("set_dimming");
		exynos_panel_set_dimming(ctx, conn_state->dimming_on);
		DPU_ATRACE_END("set_dimming");
	}

	if ((conn_state->pending_update_flags & HBM_FLAG_OP_RATE_UPDATE) && exynos_panel_func &&
	    exynos_panel_func->set_op_hz) {
		const struct decon_device *decon = to_exynos_crtc(conn_state->base.crtc)->ctx;
		const struct drm_connector_state *drm_conn_state = &conn_state->base;
		const struct drm_crtc_state *crtc_state =
				drm_conn_state->crtc ? drm_conn_state->crtc->state : NULL;

		/* disable atc before operation rate switch if it's enabled */
		if (unlikely(!decon || !decon->dqe || !crtc_state)) {
			dev_warn(ctx->dev, "unable to disable atc for op\n");
			ctx->atc_need_enabled = false;
		} else if (decon->dqe->force_atc_config.en != true) {
			ctx->atc_need_enabled = false;
		} else if (ctx->desc->keep_atc_on_for_op) {
			dev_dbg(ctx->dev, "keep atc on for op\n");
		} else {
			exynos_panel_set_atc_config(ctx, decon,
						    to_exynos_crtc_state(crtc_state),
						    false);
			ctx->atc_need_enabled = true;
		}

		DPU_ATRACE_BEGIN("set_op_hz");
		dev_info(ctx->dev, "%s: set op_hz to %d\n", __func__,
			 conn_state->operation_rate);
		exynos_panel_set_op_hz(ctx, conn_state->operation_rate);
		DPU_ATRACE_END("set_op_hz");
	}

	if (mipi_sync)
		exynos_dsi_dcs_write_buffer_force_batch_end(dsi);

	if (((MIPI_CMD_SYNC_GHBM | MIPI_CMD_SYNC_BL) & conn_state->mipi_sync)
	    && !(MIPI_CMD_SYNC_LHBM & conn_state->mipi_sync)
	    && ctx->desc->dbv_extra_frame) {
		/**
		 * panel needs one extra VSYNC period to apply GHBM/dbv. The frame
		 * update should be delayed.
		 */
		DPU_ATRACE_BEGIN("dbv_wait");
		if (!drm_crtc_vblank_get(conn_state->base.crtc)) {
			drm_crtc_wait_one_vblank(conn_state->base.crtc);
			drm_crtc_vblank_put(conn_state->base.crtc);
		} else {
			pr_warn("%s failed to get vblank for dbv wait\n", __func__);
		}
		DPU_ATRACE_END("dbv_wait");
	}

	if (ghbm_updated)
		sysfs_notify(&ctx->bl->dev.kobj, NULL, "hbm_mode");

	DPU_ATRACE_END(__func__);
}

static void exynos_panel_connector_atomic_pre_commit(
				struct exynos_drm_connector *exynos_connector,
			    struct exynos_drm_connector_state *exynos_old_state,
			    struct exynos_drm_connector_state *exynos_new_state)
{
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);

	exynos_panel_pre_commit_properties(ctx, exynos_new_state);

	mutex_lock(&ctx->mode_lock);
	if (ctx->panel_update_idle_mode_pending)
		panel_update_idle_mode_locked(ctx, false);
	mutex_unlock(&ctx->mode_lock);
}

static void exynos_panel_connector_atomic_commit(
				struct exynos_drm_connector *exynos_connector,
			    struct exynos_drm_connector_state *exynos_old_state,
			    struct exynos_drm_connector_state *exynos_new_state)
{
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	struct dsim_device *dsim = host_to_dsi(to_mipi_dsi_device(ctx->dev)->host);
	const struct exynos_panel_funcs *exynos_panel_func = ctx->desc->exynos_panel_func;

	if (!exynos_panel_func)
		return;

	mutex_lock(&ctx->mode_lock);
	if (exynos_panel_func->commit_done && ctx->current_mode)
		exynos_panel_func->commit_done(ctx);
	mutex_unlock(&ctx->mode_lock);

	ctx->last_commit_ts = ktime_get();

	/*
	 * TODO: Identify other kinds of errors and ensure detection is debounced
	 *	 correctly
	 */
	if (exynos_old_state->is_recovering && dsim->config.mode == DSIM_COMMAND_MODE) {
		mutex_lock(&ctx->mode_lock);
		ctx->error_count_te++;
		sysfs_notify(&ctx->dev->kobj, NULL, "error_count_te");
		mutex_unlock(&ctx->mode_lock);
	}

	if (exynos_old_state->is_recovering &&
	    ctx->hbm.local_hbm.requested_state == LOCAL_HBM_ENABLED) {
		dev_info(ctx->dev, "%s: doing lhbm recovery\n", __func__);

		mutex_lock(&ctx->mode_lock);
		panel_update_local_hbm_locked(ctx);
		mutex_unlock(&ctx->mode_lock);

		exynos_old_state->is_recovering = false;
	}
}

static const struct exynos_drm_connector_helper_funcs exynos_panel_connector_helper_funcs = {
	.atomic_pre_commit = exynos_panel_connector_atomic_pre_commit,
	.atomic_commit = exynos_panel_connector_atomic_commit,
};

static int exynos_drm_connector_modes(struct drm_connector *connector)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	int ret;

	ret = drm_panel_get_modes(&ctx->panel, connector);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to get panel display modes\n");
		return ret;
	}

	return ret;
}


static void exynos_drm_connector_attach_touch(struct exynos_panel *ctx,
					      const struct drm_connector_state *connector_state)
{
	struct drm_encoder *encoder = connector_state->best_encoder;
	struct drm_bridge *bridge;

	if (!encoder) {
		dev_warn(ctx->dev, "%s encoder is null\n", __func__);
		return;
	}

	bridge = of_drm_find_bridge(ctx->touch_dev);
	if (!bridge || bridge->dev)
		return;

	drm_bridge_attach(encoder, bridge, &ctx->bridge, 0);
	dev_info(ctx->dev, "attach bridge %p to encoder %p\n", bridge, encoder);
}

/*
 * Check whether transition to new mode can be done seamlessly without having
 * to turn display off before mode change. This is currently only possible if
 * only clocks/refresh rate is changing
 */
static bool exynos_panel_is_mode_seamless(const struct exynos_panel *ctx,
					  const struct exynos_panel_mode *mode)
{
	const struct exynos_panel_funcs *funcs;

	funcs = ctx->desc->exynos_panel_func;
	if (!funcs || !funcs->is_mode_seamless)
		return false;

	return funcs->is_mode_seamless(ctx, mode);
}

static void exynos_panel_set_partial(struct exynos_display_partial *partial,
			const struct exynos_panel_mode *pmode, bool is_partial)
{
	const struct exynos_display_dsc *dsc = &pmode->exynos_mode.dsc;
	const struct drm_display_mode *mode = &pmode->mode;

	partial->enabled = is_partial;
	if (!partial->enabled)
		return;

	if (dsc->enabled) {
		partial->min_width = DIV_ROUND_UP(mode->hdisplay, dsc->slice_count);
		partial->min_height = dsc->slice_height;
	} else {
		partial->min_width = MIN_WIN_BLOCK_WIDTH;
		partial->min_height = MIN_WIN_BLOCK_HEIGHT;
	}
}

static int exynos_drm_connector_check_mode(struct exynos_panel *ctx,
					   struct drm_connector_state *connector_state,
					   struct drm_crtc_state *crtc_state)
{
	struct exynos_drm_connector_state *exynos_connector_state =
		to_exynos_connector_state(connector_state);
	const struct exynos_panel_mode *pmode =
		exynos_panel_get_mode(ctx, &crtc_state->mode);
	bool is_video_mode;

	if (!pmode) {
		dev_warn(ctx->dev, "invalid mode %s\n", crtc_state->mode.name);
		return -EINVAL;
	}
	is_video_mode = (pmode->exynos_mode.mode_flags & MIPI_DSI_MODE_VIDEO) != 0;

	/* self refresh is only supported in command mode */
	connector_state->self_refresh_aware = !is_video_mode;

	if (crtc_state->connectors_changed || !is_panel_active(ctx))
		exynos_connector_state->seamless_possible = false;
	else
		exynos_connector_state->seamless_possible =
			exynos_panel_is_mode_seamless(ctx, pmode);

	exynos_connector_state->exynos_mode = pmode->exynos_mode;
	exynos_panel_set_partial(&exynos_connector_state->partial, pmode,
			ctx->desc->is_partial);

	return 0;
}

/*
 * this atomic check is called before adjusted mode is populated, this can be used to check only
 * connector state (without adjusted mode), or to decide if modeset may be required
 */
static int exynos_drm_connector_atomic_check(struct drm_connector *connector,
					     struct drm_atomic_state *state)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	struct drm_connector_state *old_conn_state, *new_conn_state, *conn_state;

	old_conn_state = drm_atomic_get_old_connector_state(state, connector);
	new_conn_state = drm_atomic_get_new_connector_state(state, connector);

	if (new_conn_state->crtc)
		conn_state = new_conn_state;
	else if (old_conn_state->crtc)
		conn_state = old_conn_state;
	else
		return 0; /* connector is/was unused */

	if (ctx->touch_dev)
		exynos_drm_connector_attach_touch(ctx, conn_state);

	return 0;
}

static const struct drm_connector_helper_funcs exynos_connector_helper_funcs = {
	.atomic_check = exynos_drm_connector_atomic_check,
	.get_modes = exynos_drm_connector_modes,
};

#ifdef CONFIG_DEBUG_FS

static u8 panel_get_cmd_type(const struct exynos_dsi_cmd *cmd)
{
	if (cmd->type)
		return cmd->type;

	switch (cmd->cmd_len) {
	case 0:
		return -EINVAL;
	case 1:
		return MIPI_DSI_DCS_SHORT_WRITE;
	case 2:
		return MIPI_DSI_DCS_SHORT_WRITE_PARAM;
	default:
		return MIPI_DSI_DCS_LONG_WRITE;
	}
}

static int panel_cmdset_show(struct seq_file *m, void *data)
{
	const struct exynos_dsi_cmd_set *cmdset = m->private;
	const struct exynos_dsi_cmd *cmd;
	u8 type;
	int i;

	for (i = 0; i < cmdset->num_cmd; i++) {
		cmd = &cmdset->cmds[i];

		type = panel_get_cmd_type(cmd);
		seq_printf(m, "0x%02x ", type);
		seq_hex_dump(m, "\t", DUMP_PREFIX_NONE, 16, 1, cmd->cmd, cmd->cmd_len, false);

		if (cmd->delay_ms)
			seq_printf(m, "wait \t%dms\n", cmd->delay_ms);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(panel_cmdset);

void exynos_panel_debugfs_create_cmdset(struct exynos_panel *ctx,
					struct dentry *parent,
					const struct exynos_dsi_cmd_set *cmdset,
					const char *name)

{
	if (!cmdset)
		return;

	debugfs_create_file(name, 0600, parent, (void *)cmdset, &panel_cmdset_fops);
}
EXPORT_SYMBOL_GPL(exynos_panel_debugfs_create_cmdset);

static int panel_gamma_show(struct seq_file *m, void *data)
{
	struct exynos_panel *ctx = m->private;
	const struct exynos_panel_funcs *funcs;
	const struct drm_display_mode *mode;
	int i;

	funcs = ctx->desc->exynos_panel_func;
	for_each_display_mode(i, mode, ctx) {
		seq_printf(m, "\n=== %dhz Mode Gamma ===\n", drm_mode_vrefresh(mode));
		funcs->print_gamma(m, mode);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(panel_gamma);

static int panel_debugfs_add(struct exynos_panel *ctx, struct dentry *parent)
{
	const struct exynos_panel_desc *desc = ctx->desc;
	const struct exynos_panel_funcs *funcs = desc->exynos_panel_func;
	struct dentry *cmdset_root;

	debugfs_create_u32("rev", 0600, parent, &ctx->panel_rev);
	debugfs_create_bool("lhbm_postwork_disabled", 0600, parent,
			    &ctx->hbm.local_hbm.post_work_disabled);
	debugfs_create_u32("normal_mode_work_delay_ms", 0600, parent,
			   &ctx->normal_mode_work_delay_ms);

	if (!funcs)
		return -EINVAL;

	if (funcs->print_gamma)
		debugfs_create_file("gamma", 0600, parent, ctx, &panel_gamma_fops);

	cmdset_root = debugfs_create_dir("cmdsets", parent);
	if (!cmdset_root) {
		dev_err(ctx->dev, "can't create cmdset dir\n");
		return -EFAULT;
	}

	exynos_panel_debugfs_create_cmdset(ctx, cmdset_root, desc->off_cmd_set, "off");

	if (desc->lp_mode) {
		struct dentry *lpd;
		int i;

		if (desc->binned_lp) {
			lpd = debugfs_create_dir("lp", cmdset_root);
			if (!lpd) {
				dev_err(ctx->dev, "can't create lp dir\n");
				return -EFAULT;
			}

			for (i = 0; i < desc->num_binned_lp; i++) {
				const struct exynos_binned_lp *b = &desc->binned_lp[i];

				exynos_panel_debugfs_create_cmdset(ctx, lpd, &b->cmd_set, b->name);
			}
		} else {
			lpd = cmdset_root;
		}
		exynos_panel_debugfs_create_cmdset(ctx, lpd, desc->lp_cmd_set, "lp_entry");
	}

	return 0;
}

static ssize_t exynos_dsi_dcs_transfer(struct mipi_dsi_device *dsi, u8 type,
				     const void *data, size_t len, u16 flags)
{
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	bool is_last;
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.tx_buf = data,
		.tx_len = len,
		.type = type,
	};

	if (!ops || !ops->transfer)
		return -ENOSYS;

	msg.flags = flags;

	is_last = ((flags & EXYNOS_DSI_MSG_QUEUE) == 0) || (flags & EXYNOS_DSI_MSG_FORCE_FLUSH);
	trace_dsi_tx(msg.type, msg.tx_buf, msg.tx_len, is_last, 0);
	if (dsi->mode_flags & MIPI_DSI_MODE_LPM)
		msg.flags |= MIPI_DSI_MSG_USE_LPM;

	return ops->transfer(dsi->host, &msg);
}

int exynos_dcs_write_delay(struct exynos_panel *ctx, const void *data, size_t len,
			   u32 delay_ms)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	host_to_dsi(dsi->host)->tx_delay_ms = delay_ms;

	return exynos_dcs_write(ctx, data, len);
}
EXPORT_SYMBOL_GPL(exynos_dcs_write_delay);

ssize_t exynos_dsi_dcs_write_buffer(struct mipi_dsi_device *dsi,
				  const void *data, size_t len, u16 flags)
{
	u8 type;

	switch (len) {
	case 0:
		/* allow flag only messages to dsim */
		type = 0;
		break;

	case 1:
		type = MIPI_DSI_DCS_SHORT_WRITE;
		break;

	case 2:
		type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		break;

	default:
		type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}

	return exynos_dsi_dcs_transfer(dsi, type, data, len, flags);
}
EXPORT_SYMBOL_GPL(exynos_dsi_dcs_write_buffer);

static int exynos_dsi_name_show(struct seq_file *m, void *data)
{
	struct mipi_dsi_device *dsi = m->private;

	seq_puts(m, dsi->name);
	seq_putc(m, '\n');

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(exynos_dsi_name);

static ssize_t parse_byte_buf(u8 *out, size_t len, char *src)
{
	const char *skip = "\n ";
	size_t i = 0;
	int rc = 0;
	char *s;

	while (src && !rc && i < len) {
		s = strsep(&src, skip);
		if (*s != '\0') {
			rc = kstrtou8(s, 16, out + i);
			i++;
		}
	}

	return rc ? : i;
}

static ssize_t exynos_panel_parse_byte_buf(char *input_str, size_t input_len,
					   const char **out_buf)
{
	size_t len = (input_len + 1) / 2;
	size_t rc;
	char *out;

	out = kzalloc(len, GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	rc = parse_byte_buf(out, len, input_str);
	if (rc <= 0) {
		kfree(out);
		return rc;
	}

	*out_buf = out;

	return rc;
}

struct exynos_dsi_reg_data {
	struct mipi_dsi_device *dsi;
	u8 address;
	u8 type;
	u16 flags;
	size_t count;
};

static ssize_t exynos_dsi_payload_write(struct file *file,
			       const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct exynos_dsi_reg_data *reg_data = m->private;
	char *buf;
	char *payload;
	size_t len;
	int ret;

	buf = memdup_user_nul(user_buf, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	/* calculate length for worst case (1 digit per byte + whitespace) */
	len = (count + 1) / 2;
	payload = kmalloc(len, GFP_KERNEL);
	if (!payload) {
		kfree(buf);
		return -ENOMEM;
	}

	ret = parse_byte_buf(payload, len, buf);
	if (ret <= 0) {
		ret = -EINVAL;
	} else if (reg_data->type) {
		ret = exynos_dsi_dcs_transfer(reg_data->dsi, reg_data->type,
					    payload, ret, reg_data->flags);
	} else {
		ret = exynos_dsi_dcs_write_buffer(reg_data->dsi, payload, ret,
						reg_data->flags);
	}

	kfree(buf);
	kfree(payload);

	return ret ? : count;
}

static int exynos_dsi_payload_show(struct seq_file *m, void *data)
{
	struct exynos_dsi_reg_data *reg_data = m->private;
	char *buf;
	ssize_t rc;

	if (!reg_data->count)
		return -EINVAL;

	buf = kmalloc(reg_data->count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	rc = mipi_dsi_dcs_read(reg_data->dsi, reg_data->address, buf,
			       reg_data->count);
	if (rc > 0) {
		seq_hex_dump(m, "", DUMP_PREFIX_NONE, 16, 1, buf, rc, false);
		rc = 0;
	} else if (rc == 0) {
		pr_debug("no response back\n");
	}
	kfree(buf);

	return 0;
}

static int exynos_dsi_payload_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_dsi_payload_show, inode->i_private);
}

static const struct file_operations exynos_dsi_payload_fops = {
	.owner		= THIS_MODULE,
	.open		= exynos_dsi_payload_open,
	.write		= exynos_dsi_payload_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int exynos_reset_panel(struct exynos_panel *ctx)
{
	if (!ctx) {
		pr_debug("reset_panel: exynos_panel not exist\n");
		return -EPERM;
	}

	if (IS_ERR_OR_NULL(ctx->reset_gpio)) {
		pr_debug("reset_panel: reset_gpio is invalid\n");
		return -EPERM;
	}

	gpiod_set_value(ctx->reset_gpio, 0);
	pr_info("reset_panel: pull reset_gpio to low to reset panel\n");

	return 0;
}

static ssize_t exynos_debugfs_reset_panel(struct file *file,
			       const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	bool reset_panel;
	int ret;
	struct mipi_dsi_device *dsi = file->private_data;
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!is_panel_active(ctx))
		return -EPERM;

	ret = kstrtobool_from_user(user_buf, count, &reset_panel);
	if (ret)
		return ret;

	if (reset_panel) {
		ret = exynos_reset_panel(ctx);
		if (ret) {
			pr_debug("reset_panel: reset panel failed\n");
			return ret;
		}
	}

	return count;
}

static const struct file_operations exynos_reset_panel_fops = {
	.open = simple_open,
	.write = exynos_debugfs_reset_panel,
};

static int exynos_dsi_debugfs_add(struct mipi_dsi_device *dsi,
				  struct dentry *parent)
{
	struct dentry *reg_root;
	struct exynos_dsi_reg_data *reg_data;

	reg_root = debugfs_create_dir("reg", parent);
	if (!reg_root)
		return -EFAULT;

	reg_data = devm_kzalloc(&dsi->dev, sizeof(*reg_data), GFP_KERNEL);
	if (!reg_data)
		return -ENOMEM;

	reg_data->dsi = dsi;

	debugfs_create_u8("address", 0600, reg_root, &reg_data->address);
	debugfs_create_u8("type", 0600, reg_root, &reg_data->type);
	debugfs_create_size_t("count", 0600, reg_root, &reg_data->count);
	debugfs_create_u16("flags", 0600, reg_root, &reg_data->flags);
	debugfs_create_file("payload", 0600, reg_root, reg_data,
			    &exynos_dsi_payload_fops);

	debugfs_create_file("name", 0600, parent, dsi, &exynos_dsi_name_fops);
	debugfs_create_file("reset_panel",0200, parent, dsi, &exynos_reset_panel_fops);

	return 0;
}
#else
static int panel_debugfs_add(struct exynos_panel *ctx, struct dentry *parent)
{
	return 0;
}

static int exynos_dsi_debugfs_add(struct mipi_dsi_device *dsi,
			 struct dentry *parent)
{
	return 0;
}
#endif

static ssize_t hbm_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	const struct exynos_panel_mode *pmode;
	u32 hbm_mode;
	int ret;

	if (!funcs || !funcs->set_hbm_mode) {
		dev_err(ctx->dev, "HBM is not supported\n");
		return -ENOTSUPP;
	}

	mutex_lock(&ctx->mode_lock);
	pmode = ctx->current_mode;

	if (!is_panel_active(ctx) || !pmode) {
		dev_err(ctx->dev, "panel is not enabled\n");
		ret = -EPERM;
		goto unlock;
	}

	if (pmode->exynos_mode.is_lp_mode) {
		dev_dbg(ctx->dev, "hbm unsupported in LP mode\n");
		ret = -EPERM;
		goto unlock;
	}

	ret = kstrtouint(buf, 0, &hbm_mode);
	if (ret || (hbm_mode >= HBM_STATE_MAX)) {
		dev_err(ctx->dev, "invalid hbm_mode value\n");
		goto unlock;
	}

	if (hbm_mode != ctx->hbm_mode) {
		funcs->set_hbm_mode(ctx, hbm_mode);
		notify_panel_mode_changed(ctx, false);
	}

unlock:
	mutex_unlock(&ctx->mode_lock);

	return ret ? : count;
}

static ssize_t hbm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctx->hbm_mode);
}

static DEVICE_ATTR_RW(hbm_mode);

static ssize_t cabc_mode_store(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	u32 cabc_mode;
	int ret;

	if (!is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	ret = kstrtouint(buf, 0, &cabc_mode);
	if (ret || (cabc_mode > CABC_MOVIE_MODE)) {
		dev_err(ctx->dev, "invalid cabc_mode value");
		return -EINVAL;
	}

	exynos_panel_set_cabc(ctx, cabc_mode);

	return count;
}

static ssize_t cabc_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	const char *mode;

	switch (ctx->cabc_mode) {
	case CABC_OFF:
		mode = "OFF";
		break;
	case CABC_UI_MODE:
		mode = "UI";
		break;
	case CABC_STILL_MODE:
		mode = "STILL";
		break;
	case CABC_MOVIE_MODE:
		mode = "MOVIE";
		break;
	default:
		dev_err(ctx->dev, "unknown CABC mode : %d\n", ctx->cabc_mode);
		return -EINVAL;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", mode);
}

static DEVICE_ATTR_RW(cabc_mode);

static ssize_t dimming_on_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	bool dimming_on;
	int ret;

	if (!is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	ret = kstrtobool(buf, &dimming_on);
	if (ret) {
		dev_err(ctx->dev, "invalid dimming_on value\n");
		return ret;
	}

	exynos_panel_set_dimming(ctx, dimming_on);

	return count;
}

static ssize_t dimming_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->dimming_on);
}
static DEVICE_ATTR_RW(dimming_on);

static ssize_t local_hbm_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	bool local_hbm_en;
	int ret;
	struct drm_crtc *crtc = get_exynos_panel_connector_crtc(ctx);

	if (!is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	if (!funcs || !funcs->set_local_hbm_mode) {
		dev_err(ctx->dev, "Local HBM is not supported\n");
		return -ENOTSUPP;
	}

	ret = kstrtobool(buf, &local_hbm_en);
	if (ret) {
		dev_err(ctx->dev, "invalid local_hbm_mode value\n");
		return ret;
	}

	if (crtc && !drm_crtc_vblank_get(crtc)) {
		struct drm_vblank_crtc vblank = crtc->dev->vblank[crtc->index];
		u32 delay_us = vblank.framedur_ns / 2000;

		drm_crtc_wait_one_vblank(crtc);
		drm_crtc_vblank_put(crtc);
		/* wait for 0.5 frame to send to ensure it is done in one frame */
		usleep_range(delay_us, delay_us + 10);
	}

	dev_info(ctx->dev, "%s: set LHBM to %d\n", __func__, local_hbm_en);
	mutex_lock(&ctx->mode_lock);
	ctx->hbm.local_hbm.requested_state = local_hbm_en;
	panel_update_local_hbm_locked(ctx);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t local_hbm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->hbm.local_hbm.effective_state);
}
static DEVICE_ATTR_RW(local_hbm_mode);

static ssize_t local_hbm_max_timeout_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	int ret;

	ret = kstrtou32(buf, 0, &ctx->hbm.local_hbm.max_timeout_ms);
	if (ret) {
		dev_err(ctx->dev, "invalid local_hbm_max_timeout_ms value\n");
		return ret;
	}

	return count;
}

static ssize_t local_hbm_max_timeout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->hbm.local_hbm.max_timeout_ms);
}

static DEVICE_ATTR_RW(local_hbm_max_timeout);

static ssize_t state_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	enum display_state state;
	int rc, ret_cnt;

	mutex_lock(&ctx->bl_state_lock);
	state = get_current_display_state_locked(ctx);
	mutex_unlock(&ctx->bl_state_lock);

	ret_cnt = scnprintf(buf, PAGE_SIZE, "%s\n", disp_state_str[state]);
	rc = ret_cnt;

	if (rc > 0 && state != DISPLAY_STATE_OFF) {
		const struct exynos_panel_mode *pmode;

		mutex_lock(&ctx->mode_lock);
		pmode = ctx->current_mode;
		mutex_unlock(&ctx->mode_lock);
		if (pmode) {
			/* overwrite \n and continue the string */
			const u8 str_len = ret_cnt - 1;

			ret_cnt = scnprintf(buf + str_len, PAGE_SIZE - str_len,
				      ": %ux%u@%d\n",
				      pmode->mode.hdisplay, pmode->mode.vdisplay,
				      exynos_get_actual_vrefresh(ctx));
			if (ret_cnt > 0)
				rc = str_len + ret_cnt;
		}
	}

	dev_dbg(ctx->dev, "%s: %s\n", __func__, rc > 0 ? buf : "");

	return rc;
}

static DEVICE_ATTR_RO(state);

static ssize_t lp_state_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	int rc;

	mutex_lock(&ctx->bl_state_lock);

	if (!is_backlight_lp_state(bl)) {
		dev_warn(ctx->dev, "panel is not in LP mode\n");
		mutex_unlock(&ctx->bl_state_lock);
		return -EPERM;
	}

	if (!ctx->current_binned_lp) {
		dev_warn(ctx->dev, "LP state is null\n");
		mutex_unlock(&ctx->bl_state_lock);
		return -EINVAL;
	}

	mutex_lock(&ctx->lp_state_lock);
	rc = scnprintf(buf, PAGE_SIZE, "%s\n", ctx->current_binned_lp->name);
	mutex_unlock(&ctx->lp_state_lock);

	mutex_unlock(&ctx->bl_state_lock);

	dev_dbg(ctx->dev, "%s: %s\n", __func__, buf);

	return rc;
}

static DEVICE_ATTR_RO(lp_state);

static ssize_t te2_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	const struct exynos_panel_mode *pmode;
	int rc = 0;

	if (!is_panel_active(ctx))
		return -EPERM;

	mutex_lock(&ctx->mode_lock);
	pmode = ctx->current_mode;
	mutex_unlock(&ctx->mode_lock);
	if (pmode) {
		bool fixed = ctx->te2.option == TE2_OPT_FIXED;
		bool lp_mode = pmode->exynos_mode.is_lp_mode;
		int vrefresh;

		if (fixed)
			vrefresh = lp_mode ? FIXED_TE2_VREFRESH_LP : FIXED_TE2_VREFRESH_NORMAL;
		else
			vrefresh = exynos_get_actual_vrefresh(ctx);

		rc = scnprintf(buf, PAGE_SIZE, "%s-te2@%d\n",
			       fixed ? "fixed" : "changeable", vrefresh);
	}

	dev_dbg(ctx->dev, "%s: %s\n", __func__, rc > 0 ? buf : "");

	return rc;
}

static DEVICE_ATTR_RO(te2_state);

static ssize_t dim_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->desc->lower_min_brightness);
}

static DEVICE_ATTR_RO(dim_brightness);

static int parse_u32_buf(char *src, size_t src_len, u32 *out, size_t out_len)
{
	int rc = 0, cnt = 0;
	char *str;
	const char *delim = " ";

	if (!src || !src_len || !out || !out_len)
		return -EINVAL;

	/* src_len is the length of src including null character '\0' */
	if (strnlen(src, src_len) == src_len)
		return -EINVAL;

	for (str = strsep(&src, delim); str != NULL; str = strsep(&src, delim)) {
		rc = kstrtou32(str, 0, out + cnt);
		if (rc)
			return -EINVAL;

		cnt++;

		if (out_len == cnt)
			break;
	}

	return cnt;
}

static ssize_t als_table_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	ssize_t bl_num_ranges;
	char *buf_dup;
	u32 ranges[MAX_BL_RANGES] = {0};
	u32 i;

	if (count == 0)
		return -EINVAL;

	buf_dup = kstrndup(buf, count, GFP_KERNEL);
	if (!buf_dup)
		return -ENOMEM;

	if (strlen(buf_dup) != count) {
		kfree(buf_dup);
		return -EINVAL;
	}

	bl_num_ranges = parse_u32_buf(buf_dup, count + 1,
				      ranges, MAX_BL_RANGES);
	if (bl_num_ranges < 0 || bl_num_ranges > MAX_BL_RANGES) {
		dev_warn(ctx->dev, "exceed max number of bl range\n");
		kfree(buf_dup);
		return -EINVAL;
	}

	mutex_lock(&ctx->bl_state_lock);

	ctx->bl_notifier.num_ranges = bl_num_ranges;
	for (i = 0; i < ctx->bl_notifier.num_ranges; i++)
		ctx->bl_notifier.ranges[i] = ranges[i];

	mutex_unlock(&ctx->bl_state_lock);

	kfree(buf_dup);

	return count;
}

static ssize_t als_table_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	ssize_t rc = 0;
	size_t len = 0;
	u32 i = 0;

	mutex_lock(&ctx->bl_state_lock);

	for (i = 0; i < ctx->bl_notifier.num_ranges; i++) {
		rc = scnprintf(buf + len, PAGE_SIZE - len,
			       "%u ", ctx->bl_notifier.ranges[i]);
		if (rc < 0) {
			mutex_unlock(&ctx->bl_state_lock);
			return -EINVAL;
		}

		len += rc;
	}

	mutex_unlock(&ctx->bl_state_lock);

	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}

static DEVICE_ATTR_RW(als_table);

static ssize_t ssc_en_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	bool enabled;
	int ret;

	if (!funcs || !funcs->set_ssc_en)
		return -ENOTSUPP;

	if (!is_panel_active(ctx)) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	ret = kstrtobool(buf, &enabled);
	if (ret) {
		dev_err(ctx->dev, "invalid ssc_mode value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	if (enabled != ctx->ssc_en) {
		funcs->set_ssc_en(ctx, enabled);
	}
	mutex_unlock(&ctx->mode_lock);
	return count;
}

static ssize_t ssc_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->ssc_en);
}
static DEVICE_ATTR_RW(ssc_en);

static ssize_t acl_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	ssize_t ret;
	u32 acl_mode;

	if (!funcs || !funcs->set_acl_mode)
		return -ENOTSUPP;

	if (!is_panel_initialized(ctx))
		return -EAGAIN;


	ret = kstrtouint(buf, 0, &acl_mode);
	if (ret || (acl_mode > ACL_ENHANCED)) {
		dev_err(dev, "invalid acl mode\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->acl_mode = acl_mode;
	funcs->set_acl_mode(ctx, acl_mode);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t acl_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!is_panel_initialized(ctx))
		return -EAGAIN;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->acl_mode);
}

static DEVICE_ATTR_RW(acl_mode);

static ssize_t allow_wakeup_by_state_change_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	int ret;

	ret = kstrtobool(buf, &ctx->allow_wakeup_by_state_change);
	if (ret) {
		dev_err(ctx->dev, "invalid allow wakeup by state change value\n");
		return ret;
	}

	return count;
}

static ssize_t allow_wakeup_by_state_change_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return sysfs_emit_at(buf, 0, "%d\n", ctx->allow_wakeup_by_state_change);
}

static DEVICE_ATTR_RW(allow_wakeup_by_state_change);

static struct attribute *bl_device_attrs[] = {
	&dev_attr_hbm_mode.attr,
	&dev_attr_dimming_on.attr,
	&dev_attr_local_hbm_mode.attr,
	&dev_attr_local_hbm_max_timeout.attr,
	&dev_attr_dim_brightness.attr,
	&dev_attr_state.attr,
	&dev_attr_lp_state.attr,
	&dev_attr_te2_state.attr,
	&dev_attr_als_table.attr,
	&dev_attr_allow_wakeup_by_state_change.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bl_device);

static int exynos_panel_attach_brightness_capability(struct exynos_drm_connector *exynos_conn,
				const struct brightness_capability *brt_capability)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(exynos_conn);
	struct drm_property_blob *blob;

	blob = drm_property_create_blob(exynos_conn->base.dev,
				 sizeof(struct brightness_capability),
				 brt_capability);
	if (IS_ERR(blob))
		return PTR_ERR(blob);
	drm_object_attach_property(&exynos_conn->base.base, p->brightness_capability, blob->base.id);

	return 0;
}

static unsigned long get_backlight_state_from_panel(struct backlight_device *bl,
					enum exynos_panel_state panel_state)
{
	unsigned long state = bl->props.state;

	switch (panel_state) {
	case PANEL_STATE_NORMAL:
		state &= ~(BL_STATE_STANDBY | BL_STATE_LP);
		break;
	case PANEL_STATE_LP:
		state &= ~(BL_STATE_STANDBY);
		state |= BL_STATE_LP;
		break;
	case PANEL_STATE_MODESET: /* no change */
		break;
	case PANEL_STATE_OFF:
	case PANEL_STATE_BLANK:
	default:
		state &= ~(BL_STATE_LP);
		state |= BL_STATE_STANDBY;
		break;
	}

	return state;
}

static void exynos_panel_set_backlight_state(struct exynos_panel *ctx,
					enum exynos_panel_state panel_state)
{
	struct backlight_device *bl = ctx->bl;
	unsigned long state;
	bool state_changed = false;

	if (!bl)
		return;

	mutex_lock(&ctx->bl_state_lock);

	state = get_backlight_state_from_panel(bl, panel_state);
	if (state != bl->props.state) {
		bl->props.state = state;
		state_changed = true;
	}

	mutex_unlock(&ctx->bl_state_lock);

	if (state_changed) {
		notify_panel_mode_changed(ctx, false);
		dev_info(ctx->dev, "panel: %s | bl: brightness@%u, state@0x%x\n",
			 exynos_panel_get_state_str(panel_state), bl->props.brightness,
			 bl->props.state);
	}
}

static int exynos_panel_attach_properties(struct exynos_panel *ctx)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(&ctx->exynos_connector);
	struct drm_mode_object *obj = &ctx->exynos_connector.base.base;
	const struct exynos_panel_desc *desc = ctx->desc;
	int ret = 0;

	if (!p || !desc)
		return -ENOENT;

	drm_object_attach_property(obj, p->min_luminance, desc->min_luminance);
	drm_object_attach_property(obj, p->max_luminance, desc->max_luminance);
	drm_object_attach_property(obj, p->max_avg_luminance, desc->max_avg_luminance);
	drm_object_attach_property(obj, p->hdr_formats, desc->hdr_formats);
	drm_object_attach_property(obj, p->brightness_level, 0);
	drm_object_attach_property(obj, p->global_hbm_mode, 0);
	drm_object_attach_property(obj, p->local_hbm_on, 0);
	drm_object_attach_property(obj, p->dimming_on, 0);
	drm_object_attach_property(obj, p->mipi_sync, MIPI_CMD_SYNC_NONE);
	drm_object_attach_property(obj, p->is_partial, desc->is_partial);
	drm_object_attach_property(obj, p->panel_idle_support, desc->is_panel_idle_supported);
	drm_object_attach_property(obj, p->panel_orientation, ctx->orientation);
	drm_object_attach_property(obj, p->rr_switch_duration, desc->rr_switch_duration);
	drm_object_attach_property(obj, p->operation_rate, 0);
	drm_object_attach_property(obj, p->refresh_on_lp, desc->refresh_on_lp);

	if (desc->brt_capability) {
		ret = exynos_panel_attach_brightness_capability(&ctx->exynos_connector,
				desc->brt_capability);
		if (ret)
			dev_err(ctx->dev, "Failed to attach brightness capability (%d)\n", ret);
	}

	if (desc->lp_mode)
		drm_object_attach_property(obj, p->lp_mode, 0);

	return ret;
}

static const char *exynos_panel_get_sysfs_name(struct exynos_panel *ctx)
{
	switch (ctx->panel_index) {
	case DISPLAY_PANEL_INDEX_PRIMARY:
		return "primary-panel";
	case DISPLAY_PANEL_INDEX_SECONDARY:
		return "secondary-panel";
	default:
		dev_err(ctx->dev, "unsupported panel_index %d\n", ctx->panel_index);
		return "primary-panel";
	}
}

static void exynos_panel_debugfs_init(struct drm_bridge *bridge,
				      struct dentry *root)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct dentry *panel_root;

	panel_root = debugfs_create_dir("panel", root);
	panel_debugfs_add(ctx, panel_root);
	if (ctx->desc->panel_func->debugfs_init)
		ctx->desc->panel_func->debugfs_init(&ctx->panel, root);

	exynos_dsi_debugfs_add(to_mipi_dsi_device(ctx->dev), panel_root);
}

static void exynos_panel_node_attach(struct exynos_drm_connector *exynos_connector)
{
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	struct drm_connector *connector = &exynos_connector->base;
	const char *sysfs_name = exynos_panel_get_sysfs_name(ctx);
	struct drm_bridge *bridge = &ctx->bridge;
	int ret;

	ret = sysfs_create_link(&connector->kdev->kobj, &ctx->dev->kobj,
				"panel");
	if (ret)
		dev_warn(ctx->dev, "unable to link panel sysfs (%d)\n", ret);

	exynos_panel_debugfs_init(bridge, connector->debugfs_entry);

	ret = sysfs_create_link(&bridge->dev->dev->kobj, &ctx->dev->kobj, sysfs_name);
	if (ret)
		dev_warn(ctx->dev, "unable to link %s sysfs (%d)\n", sysfs_name, ret);
	else
		dev_dbg(ctx->dev, "succeed to link %s sysfs\n", sysfs_name);
}

static int exynos_panel_bridge_attach(struct drm_bridge *bridge,
				      enum drm_bridge_attach_flags flags)
{
	struct drm_device *dev = bridge->dev;
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_connector *connector = &ctx->exynos_connector.base;
	int ret;

	ret = exynos_drm_connector_init(dev, &ctx->exynos_connector,
					&exynos_panel_connector_funcs,
					&exynos_panel_connector_helper_funcs,
					DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		dev_err(ctx->dev, "failed to initialize connector with drm\n");
		return ret;
	}

	ret = exynos_panel_attach_properties(ctx);
	if (ret) {
		dev_err(ctx->dev, "failed to attach connector properties\n");
		return ret;
	}

	drm_connector_helper_add(connector, &exynos_connector_helper_funcs);

	drm_connector_attach_encoder(connector, bridge->encoder);
	connector->funcs->reset(connector);
	connector->status = connector_status_connected;
	if (ctx->desc->exynos_panel_func && ctx->desc->exynos_panel_func->commit_done)
		ctx->exynos_connector.needs_commit = true;

	drm_kms_helper_hotplug_event(connector->dev);

	return 0;
}

static void exynos_panel_bridge_detach(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	const char *sysfs_name = exynos_panel_get_sysfs_name(ctx);

	sysfs_remove_link(&bridge->dev->dev->kobj, sysfs_name);
}

static void exynos_panel_bridge_enable(struct drm_bridge *bridge,
				       struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct dsim_device *dsim = host_to_dsi(to_mipi_dsi_device(ctx->dev)->host);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	bool need_update_backlight = false;
	bool is_active;
	const bool is_lp_mode = ctx->current_mode &&
				ctx->current_mode->exynos_mode.is_lp_mode;

	DPU_ATRACE_BEGIN(__func__);

	if (ctx->exynos_connector.base.state) {
		mutex_lock(&ctx->crtc_lock);
		ctx->crtc = ctx->exynos_connector.base.state->crtc;
		mutex_unlock(&ctx->crtc_lock);
	}

	mutex_lock(&ctx->mode_lock);
	if (ctx->panel_state == PANEL_STATE_HANDOFF) {
		is_active = !exynos_panel_init(ctx);
	} else if (ctx->panel_state == PANEL_STATE_HANDOFF_MODESET) {
		if (!exynos_panel_init(ctx)) {
			ctx->panel_state = PANEL_STATE_MODESET;
			mutex_unlock(&ctx->mode_lock);
			drm_panel_disable(&ctx->panel);
			mutex_lock(&ctx->mode_lock);
		}
		is_active = false;
	} else {
		is_active = is_panel_active(ctx);
	}

	/* avoid turning on panel again if already enabled (ex. while booting or self refresh) */
	if (!is_active) {
		drm_panel_enable(&ctx->panel);
		need_update_backlight = true;
	}
	ctx->panel_state = is_lp_mode ? PANEL_STATE_LP : PANEL_STATE_NORMAL;

	if (funcs && funcs->update_ffc &&
	    (!ctx->self_refresh_active || dsim->clk_param.hs_clk_changed)) {
		funcs->update_ffc(ctx, dsim->clk_param.hs_clk);
		dsim->clk_param.hs_clk_changed = false;
	}

	if (ctx->self_refresh_active) {
		dev_dbg(ctx->dev, "self refresh state : %s\n", __func__);

		ctx->self_refresh_active = false;
		panel_update_idle_mode_locked(ctx, false);
	} else {
		exynos_panel_set_backlight_state(ctx, ctx->panel_state);

		/* For the case of OFF->AOD, TE2 will be updated in backlight_update_status */
		if (ctx->panel_state == PANEL_STATE_NORMAL)
			exynos_panel_update_te2(ctx);
	}

	if (is_lp_mode) {
		if (funcs && funcs->set_post_lp_mode)
			funcs->set_post_lp_mode(ctx);
	}
	mutex_unlock(&ctx->mode_lock);

	if (need_update_backlight && ctx->bl)
		backlight_update_status(ctx->bl);

	if (!is_active && ctx->desc->exynos_panel_func &&
	    ctx->desc->exynos_panel_func->run_normal_mode_work) {
		dev_info(ctx->dev, "%s: schedule normal_mode_work\n", __func__);
		schedule_delayed_work(&ctx->normal_mode_work,
				      msecs_to_jiffies(ctx->normal_mode_work_delay_ms));
	}

	DPU_ATRACE_END(__func__);
}

/*
 * this atomic check is called after adjusted mode is populated, so it's safe to modify
 * adjusted_mode if needed at this point
 */
static int exynos_panel_bridge_atomic_check(struct drm_bridge *bridge,
					    struct drm_bridge_state *bridge_state,
					    struct drm_crtc_state *new_crtc_state,
					    struct drm_connector_state *conn_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_atomic_state *state = new_crtc_state->state;
	struct exynos_drm_connector_state *exynos_conn_state =
						to_exynos_connector_state(conn_state);
	const struct drm_display_mode *current_mode = &ctx->current_mode->mode;
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (unlikely(!new_crtc_state))
		return 0;

	if (unlikely(!current_mode)) {
		dev_warn(ctx->dev, "%s: failed to get current mode, skip mode check\n", __func__);
	} else {
		struct drm_display_mode *target_mode = &new_crtc_state->adjusted_mode;
		int current_vrefresh = drm_mode_vrefresh(current_mode);
		int target_vrefresh = drm_mode_vrefresh(target_mode);
		int current_bts_fps = exynos_drm_mode_bts_fps(current_mode,
			ctx->current_mode->exynos_mode.min_bts_fps);
		int target_bts_fps = exynos_drm_mode_bts_fps(target_mode,
			exynos_conn_state->exynos_mode.min_bts_fps);

		int clock;

		if (current_mode->hdisplay != target_mode->hdisplay &&
		    current_mode->vdisplay != target_mode->vdisplay) {
			if (current_vrefresh != target_vrefresh ||
				current_bts_fps != target_bts_fps) {
				/*
				 * While switching resolution and refresh rate (from high to low) in
				 * the same commit, the frame transfer time will become longer due to
				 * BTS update. In the case, frame done time may cross to the next
				 * vsync, which will hit DDIC’s constraint and cause the noises. Keep
				 * the current BTS (higher one) for a few frames to avoid the problem.
				 */
				if (current_bts_fps > target_bts_fps) {
					target_mode->clock = exynos_bts_fps_to_drm_mode_clock(
							target_mode, current_bts_fps);
					if (target_mode->clock != new_crtc_state->mode.clock) {
						new_crtc_state->mode_changed = true;
						dev_dbg(ctx->dev,
							"%s: keep mode (%s) clock %dhz on rrs\n",
							__func__, target_mode->name,
							current_bts_fps);
					}
					clock = target_mode->clock;
				}

				ctx->mode_in_progress = MODE_RES_AND_RR_IN_PROGRESS;
			} else {
				ctx->mode_in_progress = MODE_RES_IN_PROGRESS;
			}
		} else {
			if (ctx->mode_in_progress == MODE_RES_AND_RR_IN_PROGRESS &&
			    new_crtc_state->adjusted_mode.clock != new_crtc_state->mode.clock) {
				new_crtc_state->mode_changed = true;
				new_crtc_state->adjusted_mode.clock = new_crtc_state->mode.clock;
				clock = new_crtc_state->mode.clock;
				dev_dbg(ctx->dev, "%s: restore mode (%s) clock after rrs\n",
					__func__, new_crtc_state->mode.name);
			}

			if ((current_vrefresh != target_vrefresh) ||
					(current_bts_fps != target_bts_fps))
				ctx->mode_in_progress = MODE_RR_IN_PROGRESS;
			else
				ctx->mode_in_progress = MODE_DONE;
		}

		if (current_mode->hdisplay != target_mode->hdisplay ||
		    current_mode->vdisplay != target_mode->vdisplay ||
		    current_vrefresh != target_vrefresh ||
		    current_bts_fps != target_bts_fps)
			dev_dbg(ctx->dev,
				"%s: current %s(bts %d), target %s(bts %d), type %d\n",
				__func__, current_mode->name, current_bts_fps,
				target_mode->name, target_bts_fps, ctx->mode_in_progress);

		/*
		 * We may transfer the frame for the first TE after switching to higher
		 * op_hz. In this case, the DDIC read speed will become higher while the
		 * the DPU write speed will remain the same, so underruns would happen.
		 * Use higher BTS can avoid the issue. Also consider the clock from RRS
		 * and select the higher one.
		 */
		if ((exynos_conn_state->pending_update_flags & HBM_FLAG_OP_RATE_UPDATE) &&
		    exynos_conn_state->operation_rate > ctx->op_hz) {
			target_mode->clock =
				exynos_bts_fps_to_drm_mode_clock(target_mode, ctx->peak_bts_fps);
			/* use the higher clock to avoid underruns */
			if (target_mode->clock < clock)
				target_mode->clock = clock;

			if (target_mode->clock != new_crtc_state->mode.clock) {
				new_crtc_state->mode_changed = true;
				ctx->boosted_for_op_hz = true;
				dev_dbg(ctx->dev, "%s: raise mode clock %dhz on op_hz %d\n",
					__func__, ctx->peak_bts_fps,
					exynos_conn_state->operation_rate);
			}
		} else if (ctx->boosted_for_op_hz &&
			   new_crtc_state->adjusted_mode.clock != new_crtc_state->mode.clock) {
			new_crtc_state->mode_changed = true;
			ctx->boosted_for_op_hz = false;
			/* use the higher clock to avoid underruns */
			if (new_crtc_state->mode.clock < clock)
				new_crtc_state->adjusted_mode.clock = clock;
			else
				new_crtc_state->adjusted_mode.clock = new_crtc_state->mode.clock;

			dev_dbg(ctx->dev, "%s: restore mode clock after op_hz\n", __func__);
		}

		/* enable atc if it's disabled before */
		if (ctx->atc_need_enabled) {
			const struct decon_device *decon =
					to_exynos_crtc(exynos_conn_state->base.crtc)->ctx;

			if (unlikely(!decon || !decon->dqe))
				dev_warn(ctx->dev, "unable to enable atc for op\n");
			else if (decon->dqe->force_atc_config.en == false)
				exynos_panel_set_atc_config(ctx, decon,
							    to_exynos_crtc_state(new_crtc_state),
							    true);

			/* always clear the flag to avoid keeping retrying */
			ctx->atc_need_enabled = false;
		}
	}

	if (funcs && funcs->atomic_check) {
		ret = funcs->atomic_check(ctx, state);
		if (ret)
			return ret;
	}

	if (!drm_atomic_crtc_needs_modeset(new_crtc_state))
		return 0;

	if (ctx->panel_state == PANEL_STATE_HANDOFF) {
		struct drm_crtc_state *old_crtc_state =
			drm_atomic_get_old_crtc_state(state, new_crtc_state->crtc);

		if (!old_crtc_state->enable)
			old_crtc_state->self_refresh_active = true;
	}

	return exynos_drm_connector_check_mode(ctx, conn_state, new_crtc_state);
}

static void exynos_panel_bridge_pre_enable(struct drm_bridge *bridge,
					   struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (ctx->panel_state == PANEL_STATE_BLANK) {
		const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

		if (funcs && funcs->panel_reset)
			funcs->panel_reset(ctx);
	} else if (!is_panel_enabled(ctx)) {
		drm_panel_prepare(&ctx->panel);
	}
}

static void exynos_panel_bridge_disable(struct drm_bridge *bridge,
					struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	const struct drm_connector_state *conn_state = ctx->exynos_connector.base.state;
	struct exynos_drm_connector_state *exynos_conn_state =
		to_exynos_connector_state(conn_state);
	struct drm_crtc_state *crtc_state = !conn_state->crtc ? NULL : conn_state->crtc->state;
	const bool self_refresh_active = crtc_state && crtc_state->self_refresh_active;

	DPU_ATRACE_BEGIN(__func__);

	if (self_refresh_active && !exynos_conn_state->blanked_mode) {
		struct dsim_device *dsim = host_to_dsi(to_mipi_dsi_device(ctx->dev)->host);
		const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

		mutex_lock(&ctx->mode_lock);
		dev_dbg(ctx->dev, "self refresh state : %s\n", __func__);

		ctx->self_refresh_active = true;
		panel_update_idle_mode_locked(ctx, false);
		if (ctx->post_vddd_lp && ctx->need_post_vddd_lp) {
			_exynos_panel_set_vddd_voltage(ctx, true);
			ctx->need_post_vddd_lp = false;
		}

		if (funcs && funcs->pre_update_ffc &&
		    (dsim->clk_param.hs_clk_changed || dsim->clk_param.pending_hs_clk))
			funcs->pre_update_ffc(ctx);
		mutex_unlock(&ctx->mode_lock);
	} else {
		if (exynos_conn_state->blanked_mode) {
			/* blanked mode takes precedence over normal modeset */
			ctx->panel_state = PANEL_STATE_BLANK;
		} else if (crtc_state && crtc_state->mode_changed &&
			   drm_atomic_crtc_effectively_active(crtc_state)) {
			ctx->panel_state = PANEL_STATE_MODESET;
		} else if (ctx->force_power_on) {
			/* force blank state instead of power off */
			ctx->panel_state = PANEL_STATE_BLANK;
		} else {
			ctx->panel_state = PANEL_STATE_OFF;
			ctx->mode_in_progress = MODE_DONE;

			if (ctx->desc->exynos_panel_func &&
			    ctx->desc->exynos_panel_func->run_normal_mode_work) {
				dev_info(ctx->dev, "%s: cancel normal_mode_work\n", __func__);
				cancel_delayed_work(&ctx->normal_mode_work);
			}

			/* clear the flag since display will be off */
			ctx->atc_need_enabled = false;
		}

		drm_panel_disable(&ctx->panel);

		mutex_lock(&ctx->crtc_lock);
		ctx->crtc = NULL;
		mutex_unlock(&ctx->crtc_lock);
	}

	DPU_ATRACE_END(__func__);
}

static void exynos_panel_bridge_post_disable(struct drm_bridge *bridge,
					     struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	/* fully power off only if panel is in full off mode */
	if (!is_panel_enabled(ctx))
		drm_panel_unprepare(&ctx->panel);

	exynos_panel_set_backlight_state(ctx, ctx->panel_state);
}

/* Get the VSYNC start time within a TE period */
static u64 exynos_panel_vsync_start_time_us(u32 te_us, u32 te_period_us)
{
	/* Approximate the VSYNC start time with TE falling edge. */
	if (te_us > 0 && te_us < te_period_us)
		return te_us * 105 / 100; /* add 5% for variation */

	/* Approximate the TE falling edge with 55% TE width */
	return te_period_us * 55 / 100;
}

int exynos_panel_wait_for_vblank(struct exynos_panel *ctx)
{
	struct drm_crtc *crtc = NULL;

	if (ctx->exynos_connector.base.state)
		crtc = ctx->exynos_connector.base.state->crtc;

	if (crtc && !drm_crtc_vblank_get(crtc)) {
		drm_crtc_wait_one_vblank(crtc);
		drm_crtc_vblank_put(crtc);
		return 0;
	}

	WARN_ON(1);
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(exynos_panel_wait_for_vblank);

void exynos_panel_wait_for_vsync_done(struct exynos_panel *ctx, u32 te_us, u32 period_us)
{
	u32 delay_us;

	DPU_ATRACE_BEGIN(__func__);
	if (unlikely(exynos_panel_wait_for_vblank(ctx))) {
		delay_us = period_us + 1000;
		usleep_range(delay_us, delay_us + 10);
		DPU_ATRACE_END(__func__);
		return;
	}

	delay_us = exynos_panel_vsync_start_time_us(te_us, period_us);
	usleep_range(delay_us, delay_us + 10);
	DPU_ATRACE_END(__func__);
}
EXPORT_SYMBOL_GPL(exynos_panel_wait_for_vsync_done);

int exynos_panel_register_notifier(struct drm_connector *connector, struct notifier_block *nb)
{
	int retval;

	if (is_exynos_drm_connector(connector)) {
		struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
		struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);

		retval = blocking_notifier_chain_register(&ctx->notifier_head, nb);
		if (retval != 0)
			dev_warn(ctx->dev, "register notifier failed(%d)\n", retval);
		else
			blocking_notifier_call_chain(&ctx->notifier_head,
						     EXYNOS_PANEL_NOTIFIER_SET_OP_HZ, &ctx->op_hz);
	} else {
		dev_warn(connector->kdev,
			 "register notifier failed(unexpected type of connector)\n");
		retval = -EINVAL;
	}

	return retval;
}
EXPORT_SYMBOL_GPL(exynos_panel_register_notifier);

int exynos_panel_unregister_notifier(struct drm_connector *connector, struct notifier_block *nb)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);

	return blocking_notifier_chain_unregister(&ctx->notifier_head, nb);
}
EXPORT_SYMBOL_GPL(exynos_panel_unregister_notifier);

static u32 get_rr_switch_applied_te_count(struct exynos_panel *ctx)
{
	/* New refresh rate should take effect immediately after exiting AOD mode */
	if (ctx->last_rr_switch_ts == ctx->last_lp_exit_ts)
		return 1;

	/* New rr will take effect at the first vsync after sending rr command, but
	 * we only know te rising ts. The worse case, new rr take effect at 2nd TE.
	 */
	return 2;
}

static bool is_last_rr_applied(struct exynos_panel *ctx, ktime_t last_te)
{
	s64 rr_switch_delta_us;
	u32 te_period_before_rr_switch_us;
	u32 rr_switch_applied_te_count;

	if (last_te == 0)
		return false;

	rr_switch_delta_us = ktime_us_delta(last_te, ctx->last_rr_switch_ts);
	te_period_before_rr_switch_us = ctx->last_rr != 0 ? USEC_PER_SEC / ctx->last_rr : 0;
	rr_switch_applied_te_count = get_rr_switch_applied_te_count(ctx);

	if (rr_switch_delta_us > ((rr_switch_applied_te_count - 1) *
			te_period_before_rr_switch_us))
		return true;

	return false;
}

/* avoid accumulate te varaince cause predicted value is not precision enough */
#define ACCEPTABLE_TE_PERIOD_DETLA_NS		(3 * NSEC_PER_SEC)
#define ACCEPTABLE_TE_RR_SWITCH_DELTA_US	(500)
static ktime_t exynos_panel_te_ts_prediction(struct exynos_panel *ctx, ktime_t last_te,
					     u32 te_period_us)
{
	s64 rr_switch_delta_us, te_last_rr_switch_delta_us;
	u32 te_period_before_rr_switch_us;
	u32 rr_switch_applied_te_count;
	s64 te_period_delta_ns;

	if (last_te == 0)
		return 0;

	rr_switch_delta_us = ktime_us_delta(last_te, ctx->last_rr_switch_ts);
	te_period_before_rr_switch_us = ctx->last_rr != 0 ? USEC_PER_SEC / ctx->last_rr : 0;
	rr_switch_applied_te_count = get_rr_switch_applied_te_count(ctx);

	if (rr_switch_delta_us < 0 && te_period_before_rr_switch_us != 0) {
		/* last know TE ts is before sending last rr switch */
		ktime_t last_te_before_rr_switch, now;
		s64 since_last_te_us;
		s64 accumlated_te_period_delta_ns;

		/* donʼt predict if last rr switch ts too close to te */
		te_last_rr_switch_delta_us =  (-rr_switch_delta_us % te_period_before_rr_switch_us);
		if (te_last_rr_switch_delta_us >
			(te_period_before_rr_switch_us - ACCEPTABLE_TE_RR_SWITCH_DELTA_US) ||
			te_last_rr_switch_delta_us < ACCEPTABLE_TE_RR_SWITCH_DELTA_US) {
			return 0;
		}

		te_period_delta_ns = (-rr_switch_delta_us / te_period_before_rr_switch_us) *
					te_period_before_rr_switch_us * NSEC_PER_USEC;
		if (te_period_delta_ns < ACCEPTABLE_TE_PERIOD_DETLA_NS) {
			/* try to get last TE ts before sending last rr switch command */
			ktime_t first_te_after_rr_switch;

			last_te_before_rr_switch = last_te + te_period_delta_ns;
			now = ktime_get();
			since_last_te_us = ktime_us_delta(now, last_te_before_rr_switch);
			if (since_last_te_us < te_period_before_rr_switch_us) {
				/* now and last predict te is in the same te */
				return last_te_before_rr_switch;
			}

			first_te_after_rr_switch =
				last_te_before_rr_switch + te_period_before_rr_switch_us;

			if (rr_switch_applied_te_count == 1) {
				since_last_te_us = ktime_us_delta(now, first_te_after_rr_switch);
				accumlated_te_period_delta_ns = te_period_delta_ns;
				te_period_delta_ns =
					(since_last_te_us / te_period_us) *
					te_period_us * NSEC_PER_USEC;
				accumlated_te_period_delta_ns += te_period_delta_ns;
				if (accumlated_te_period_delta_ns < ACCEPTABLE_TE_PERIOD_DETLA_NS)
					return (first_te_after_rr_switch + te_period_delta_ns);
			} else {
				return first_te_after_rr_switch;
			}
		}
	} else if (is_last_rr_applied(ctx, last_te)) {
		/* new rr has already taken effect at last know TE ts */
		ktime_t now;
		s64 since_last_te_us;

		now = ktime_get();
		since_last_te_us = ktime_us_delta(now, last_te);
		te_period_delta_ns =
			(since_last_te_us / te_period_us) * te_period_us * NSEC_PER_USEC;

		if (te_period_delta_ns < ACCEPTABLE_TE_PERIOD_DETLA_NS)
			return (last_te + te_period_delta_ns);
	}

	return 0;
}

static void exynos_panel_check_mipi_sync_timing(struct drm_crtc *crtc,
						const struct exynos_panel_mode *current_mode,
						const struct exynos_panel_mode *target_mode,
						struct exynos_panel *ctx)
{
	u32 te_period_us;
	u32 te_usec;
	int retry;
	u64 left, right;
	bool vblank_taken = false;
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	const struct decon_device *decon = to_exynos_crtc(crtc)->ctx;
	bool is_rr_sent_at_te_high;

	if (WARN_ON(!current_mode))
		return;

	DPU_ATRACE_BEGIN("mipi_time_window");
	te_period_us = USEC_PER_SEC / exynos_drm_mode_te_freq(&current_mode->mode);

	if (funcs && funcs->get_te_usec)
		te_usec = funcs->get_te_usec(ctx, current_mode);
	else
		te_usec = current_mode->exynos_mode.te_usec;
	pr_debug("%s: check mode_set timing enter. te_period_us %u, te_usec %u\n", __func__,
		 te_period_us, te_usec);

	if (funcs && funcs->rr_need_te_high)
		is_rr_sent_at_te_high = funcs->rr_need_te_high(ctx, target_mode);
	else
		is_rr_sent_at_te_high = false;
	/*
	 * Safe time window to send RR (refresh rate) command illustrated below.
	 *
	 * When is_rr_sent_at_te_high is false, it makes RR command are sent in TE low
	 * to make RR switch and scanout happen in the same VSYNC period because the frame
	 * content might be adjusted specific to this RR.
	 *
	 * An estimation is [55% * TE_duration, TE_duration - 1ms] before driver has the
	 * accurate TE pulse width (VSYNC rising is a bit ahead of TE falling edge).
	 *
	 *         -->|     |<-- safe time window to send RR
	 *
	 *        +----+     +----+     +-+
	 *        |    |     |    |     | |
	 * TE   --+    +-----+    +-----+ +---
	 *               RR  SCANOUT
	 *
	 *            |          |       |
	 *            |          |       |
	 * VSYNC------+----------+-------+----
	 *            RR1        RR2
	 *
	 * When is_rr_sent_at_te_high is true, it makes RR switch commands are sent in TE
	 * high(skip frame) to makes RR switch happens prior to scanout. This is requested
	 * from specific DDIC to avoid transition flicker. This should not be used for TE
	 * pulse width is very short case.
	 *
	 * An estimation is [0.5ms, 55% * TE_duration - 1ms] before driver has the accurate
	 * TE pulse width (VSYNC rising is a bit ahead of TE falling edge).
	 *
	 *                -->|    |<-- safe time window to send RR
	 *
	 *        +----+     +----+     +-+
	 *        |    |     |    |     | |
	 * TE   --+    +-----+    +-----+ +---
	 *                     RR       SCANOUT
	 *
	 *            |          |       |
	 *            |          |       |
	 * VSYNC------+----------+-------+----
	 *            RR1        RR2
	 *
	 */
	retry = te_period_us / USEC_PER_MSEC + 1;

	do {
		u32 cur_te_period_us = te_period_us;
		u32 cur_te_usec = te_usec;
		ktime_t last_te = 0, now;
		s64 since_last_te_us;
		u64 vblank_counter;
		bool last_rr_applied;

		vblank_counter = drm_crtc_vblank_count_and_time(crtc, &last_te);
		now = ktime_get();
		since_last_te_us = ktime_us_delta(now, last_te);
		if (!vblank_taken) {
			ktime_t predicted_te = exynos_panel_te_ts_prediction(ctx, last_te,
				USEC_PER_SEC / exynos_drm_mode_te_freq(&current_mode->mode));
			if (predicted_te) {
				DPU_ATRACE_BEGIN("predicted_te");
				last_te = predicted_te;
				since_last_te_us = ktime_us_delta(now, last_te);
				DPU_ATRACE_INT("predicted_te_delta_us", (int)since_last_te_us);
				DPU_ATRACE_END("predicted_te");
			}
		}
		/**
		 * If a refresh rate switch happens right before last_te. last TE width could be for
		 * new rr or for old rr depending on if last rr is sent at TE high or low.
		 * If the refresh rate switch happens after last_te, last TE width won't change.
		 */
		last_rr_applied = is_last_rr_applied(ctx, last_te);
		if (ctx->last_rr != 0 && ((vblank_counter - ctx->last_rr_te_counter <= 1 &&
					   ctx->last_rr_te_gpio_value == 0 && !last_rr_applied) ||
					  ktime_after(ctx->last_rr_switch_ts, last_te))) {
			cur_te_period_us = USEC_PER_SEC / ctx->last_rr;
			cur_te_usec = ctx->last_rr_te_usec;
		}

		if (!is_rr_sent_at_te_high) {
			left = exynos_panel_vsync_start_time_us(cur_te_usec, cur_te_period_us);
			right = cur_te_period_us - USEC_PER_MSEC;
		} else {
			if (atomic_read(&decon->frame_transfer_pending)) {
				DPU_ATRACE_BEGIN("wait_frame_transfer_done");
				usleep_range(USEC_PER_MSEC, USEC_PER_MSEC + 100);
				DPU_ATRACE_END("wait_frame_transfer_done");
				continue;
			}
			left = USEC_PER_MSEC * 0.5;
			right = exynos_panel_vsync_start_time_us(cur_te_usec, cur_te_period_us)
					- USEC_PER_MSEC;
		}

		pr_debug(
			"%s: rr-te: %lld, te-now: %lld, time window [%llu, %llu] te/pulse: %u/%u\n",
			__func__, ktime_us_delta(last_te, ctx->last_rr_switch_ts),
			ktime_us_delta(now, last_te), left, right, cur_te_period_us, cur_te_usec);

		/* Only use the most recent TE as a reference point if it's not obsolete */
		if (since_last_te_us > cur_te_period_us) {
			DPU_ATRACE_BEGIN("time_window_wait_crtc");
			if (vblank_taken || !drm_crtc_vblank_get(crtc)) {
				drm_crtc_wait_one_vblank(crtc);
				vblank_taken = true;
			} else {
				pr_warn("%s failed to get vblank for ref point.\n", __func__);
			}
			DPU_ATRACE_END("time_window_wait_crtc");
			continue;
		}

		if (since_last_te_us <= right) {
			if (since_last_te_us < left) {
				u32 delay_us = left - since_last_te_us;

				DPU_ATRACE_BEGIN("time_window_wait_te_state");
				usleep_range(delay_us, delay_us + 100);
				DPU_ATRACE_END("time_window_wait_te_state");
				/*
				 * if a mode switch happens, a TE signal might
				 * happen during the sleep. need to re-sync
				 */
				continue;
			}
			break;
		}

		/* retry in 1ms */
		usleep_range(USEC_PER_MSEC, USEC_PER_MSEC + 100);
	} while (--retry > 0);

	if (vblank_taken)
		drm_crtc_vblank_put(crtc);

	pr_debug("%s: check mode_set timing exit.\n", __func__);
	DPU_ATRACE_END("mipi_time_window");
}

static bool panel_update_local_hbm_notimeout_locked(struct exynos_panel *ctx)
{
	const struct exynos_panel_mode *pmode;
	struct local_hbm *lhbm = &ctx->hbm.local_hbm;

	if (!ctx->desc->exynos_panel_func->set_local_hbm_mode)
		return false;

	if (!is_local_hbm_disabled(ctx) == !!lhbm->requested_state)
		return false;

	pmode = ctx->current_mode;
	if (unlikely(pmode == NULL)) {
		dev_err(ctx->dev, "%s: unknown current mode\n", __func__);
		return false;
	}

	if (lhbm->requested_state && !ctx->desc->no_lhbm_rr_constraints) {
		const int vrefresh = drm_mode_vrefresh(&pmode->mode);
		/* only allow to turn on LHBM at peak refresh rate to comply with HW constraint */
		if (ctx->peak_vrefresh && vrefresh != ctx->peak_vrefresh) {
			dev_err(ctx->dev, "unexpected mode `%s` while enabling LHBM, give up\n",
				pmode->mode.name);
			return false;
		}
	}

	if (is_local_hbm_post_enabling_supported(ctx)) {
		if (lhbm->requested_state) {
			lhbm->en_cmd_ts = ktime_get();
			kthread_queue_work(&lhbm->worker, &lhbm->post_work);
		} else {
			/*
			 * post_work also holds mode_lock. Release the lock
			 * before finishing post_work to avoid deadlock.
			 */
			mutex_unlock(&ctx->mode_lock);
			kthread_cancel_work_sync(&lhbm->post_work);
			mutex_lock(&ctx->mode_lock);
		}
	}

	dev_dbg(ctx->dev, "%s: requested %d, effective %d\n", __func__,
		lhbm->requested_state, lhbm->effective_state);
	lhbm->effective_state =
		(lhbm->requested_state && is_local_hbm_post_enabling_supported(ctx)) ?
		LOCAL_HBM_ENABLING : lhbm->requested_state;

	DPU_ATRACE_BEGIN(__func__);
	ctx->desc->exynos_panel_func->set_local_hbm_mode(ctx, lhbm->effective_state);
	sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
	DPU_ATRACE_END(__func__);

	return true;
}

static void panel_update_local_hbm_locked(struct exynos_panel *ctx)
{
	if (ctx->hbm.local_hbm.requested_state) {
		/* reset timeout timer if re-enabling lhbm */
		if (!is_local_hbm_disabled(ctx)) {
			mod_delayed_work(ctx->hbm.wq, &ctx->hbm.local_hbm.timeout_work,
					 msecs_to_jiffies(ctx->hbm.local_hbm.max_timeout_ms));
			return;
		}
		if (!panel_update_local_hbm_notimeout_locked(ctx))
			return;
		queue_delayed_work(ctx->hbm.wq, &ctx->hbm.local_hbm.timeout_work,
				   msecs_to_jiffies(ctx->hbm.local_hbm.max_timeout_ms));
	} else {
		cancel_delayed_work(&ctx->hbm.local_hbm.timeout_work);
		panel_update_local_hbm_notimeout_locked(ctx);
	}
}

static void exynos_panel_bridge_mode_set(struct drm_bridge *bridge,
				  const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct drm_connector_state *connector_state = ctx->exynos_connector.base.state;
	struct drm_crtc *crtc = connector_state->crtc;
	struct exynos_drm_connector_state *exynos_connector_state =
				      to_exynos_connector_state(connector_state);
	const struct exynos_panel_mode *pmode = exynos_panel_get_mode(ctx, mode);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	const struct exynos_panel_mode *old_mode;
	bool need_update_backlight = false;
	bool come_out_lp_mode = false;

	if (WARN_ON(!pmode))
		return;

	mutex_lock(&ctx->mode_lock);
	old_mode = ctx->current_mode;

	if (old_mode == pmode) {
		mutex_unlock(&ctx->mode_lock);
		return;
	}

	if (ctx->panel_state == PANEL_STATE_HANDOFF) {
		dev_warn(ctx->dev, "mode change at boot to %s\n", adjusted_mode->name);
		ctx->panel_state = PANEL_STATE_HANDOFF_MODESET;
	}

	dev_dbg(ctx->dev, "changing display mode to %s, lp %d, vrr %d\n",
		mode->name, pmode->exynos_mode.is_lp_mode, is_vrr_mode(pmode));

	dsi->mode_flags = pmode->exynos_mode.mode_flags;
	ctx->last_mode_set_ts = ktime_get();

	DPU_ATRACE_BEGIN(__func__);
	if (funcs) {
		const bool is_active = is_panel_active(ctx);
		const bool was_lp_mode = old_mode && old_mode->exynos_mode.is_lp_mode;
		const bool is_lp_mode = pmode->exynos_mode.is_lp_mode;
		bool state_changed = false;

		if (is_lp_mode && funcs->set_lp_mode) {
			if (is_active) {
				_exynos_panel_disable_normal_feat_locked(ctx);
				funcs->set_lp_mode(ctx, pmode);
				ctx->panel_state = PANEL_STATE_LP;
				need_update_backlight = true;
			}
			if (!ctx->post_vddd_lp)
				_exynos_panel_set_vddd_voltage(ctx, true);
			else
				ctx->need_post_vddd_lp = true;
		} else if (was_lp_mode && !is_lp_mode) {
			ctx->need_post_vddd_lp = false;
			_exynos_panel_set_vddd_voltage(ctx, false);
			if (is_active && funcs->set_nolp_mode) {
				funcs->set_nolp_mode(ctx, pmode);
				ctx->panel_state = PANEL_STATE_NORMAL;
				need_update_backlight = true;
				state_changed = true;
				come_out_lp_mode = true;

				if (ctx->desc->lhbm_on_delay_frames &&
					(crtc && !drm_crtc_vblank_get(crtc))) {
					ctx->hbm.local_hbm.last_lp_vblank_cnt =
							drm_crtc_vblank_count(crtc);
					drm_crtc_vblank_put(crtc);
				}
			}
			ctx->current_binned_lp = NULL;
		} else if (funcs->mode_set) {
			if ((MIPI_CMD_SYNC_REFRESH_RATE & exynos_connector_state->mipi_sync) &&
					is_active && old_mode)
				exynos_panel_check_mipi_sync_timing(crtc, old_mode, pmode, ctx);

			if (is_active) {
				if (!is_local_hbm_disabled(ctx) &&
				    !ctx->desc->no_lhbm_rr_constraints)
					dev_warn(ctx->dev,
						"do mode change (`%s`) unexpectedly when LHBM is ON\n",
						pmode->mode.name);

				funcs->mode_set(ctx, pmode);
				state_changed = true;
			} else if (ctx->mode_in_progress == MODE_RES_IN_PROGRESS ||
				ctx->mode_in_progress == MODE_RES_AND_RR_IN_PROGRESS) {
				dev_dbg(ctx->dev,
					"defer mode %s switch until enabling panel\n",
					pmode->mode.name);
			} else {
				dev_warn(ctx->dev,
					"skip mode %s switch when panel isn't active, mode_in_progress=%d\n",
					pmode->mode.name, ctx->mode_in_progress);
			}
		}

		ctx->current_mode = pmode;

		if (state_changed) {
			if (was_lp_mode)
				exynos_panel_set_backlight_state(
					ctx, is_active ? PANEL_STATE_NORMAL : PANEL_STATE_OFF);
			else if (ctx->bl)
				notify_panel_mode_changed(ctx, false);

			if (!is_lp_mode)
				exynos_panel_update_te2(ctx);
		}
	} else {
		ctx->current_mode = pmode;
	}

	if (old_mode &&
		(drm_mode_vrefresh(&pmode->mode) != drm_mode_vrefresh(&old_mode->mode) ||
		 exynos_drm_mode_te_freq(&pmode->mode) !=
			exynos_drm_mode_te_freq(&old_mode->mode))) {
		/* save the context in order to predict TE width in
		 * exynos_panel_check_mipi_sync_timing
		 */
		ctx->last_rr_switch_ts = ktime_get();
		ctx->last_rr = exynos_drm_mode_te_freq(&old_mode->mode);
		ctx->last_rr_te_gpio_value = gpio_get_value(exynos_connector_state->te_gpio);
		ctx->last_rr_te_counter = drm_crtc_vblank_count(crtc);
		if (ctx->desc->exynos_panel_func && ctx->desc->exynos_panel_func->get_te_usec)
			ctx->last_rr_te_usec =
				ctx->desc->exynos_panel_func->get_te_usec(ctx, old_mode);
		else
			ctx->last_rr_te_usec = old_mode->exynos_mode.te_usec;
		if (come_out_lp_mode)
			ctx->last_lp_exit_ts = ctx->last_rr_switch_ts;
		sysfs_notify(&ctx->dev->kobj, NULL, "refresh_rate");
	}

	if (pmode->exynos_mode.is_lp_mode && funcs && funcs->set_post_lp_mode)
		funcs->set_post_lp_mode(ctx);

	mutex_unlock(&ctx->mode_lock);

	if (need_update_backlight && ctx->bl)
		backlight_update_status(ctx->bl);

	/* we don't run normal_mode_work in LP mode */
	if (funcs && funcs->run_normal_mode_work) {
		if (pmode->exynos_mode.is_lp_mode) {
			dev_info(ctx->dev, "%s: cancel normal_mode_work while entering LP mode\n",
				 __func__);
			cancel_delayed_work(&ctx->normal_mode_work);
		} else if (old_mode && old_mode->exynos_mode.is_lp_mode &&
			   ctx->panel_state == PANEL_STATE_NORMAL) {
			dev_info(ctx->dev, "%s: schedule normal_mode_work while exiting LP mode\n",
				 __func__);
			schedule_delayed_work(&ctx->normal_mode_work,
					      msecs_to_jiffies(ctx->normal_mode_work_delay_ms));
		}
	}

	DPU_ATRACE_INT("panel_fps", drm_mode_vrefresh(mode));
	DPU_ATRACE_END(__func__);
}

static void local_hbm_timeout_work(struct work_struct *work)
{
	struct exynos_panel *ctx =
			 container_of(work, struct exynos_panel, hbm.local_hbm.timeout_work.work);

	dev_dbg(ctx->dev, "%s\n", __func__);

	dev_info(ctx->dev, "%s: turn off LHBM\n", __func__);
	mutex_lock(&ctx->mode_lock);
	ctx->hbm.local_hbm.requested_state = LOCAL_HBM_DISABLED;
	panel_update_local_hbm_notimeout_locked(ctx);
	mutex_unlock(&ctx->mode_lock);
}

static void usleep_since_ts(ktime_t ts, u32 offset_us)
{
	u32 us = ktime_us_delta(ktime_get(), ts);

	if (offset_us <= us)
		return;
	us = offset_us - us;
	usleep_range(us, us + 10);
}

static void local_hbm_wait_vblank_and_delay(struct exynos_panel *ctx, struct drm_crtc *crtc,
					    int frames, u32 offset_us)
{
	struct local_hbm *lhbm = &ctx->hbm.local_hbm;

	frames -= lhbm->frame_index;
	while (frames > 0) {
		ktime_t now;

		drm_crtc_wait_one_vblank(crtc);
		now = ktime_get();
		if (lhbm->frame_index == 0)
			lhbm->next_vblank_ts = now;
		lhbm->frame_index++;
		frames--;
		lhbm->last_vblank_ts = now;
	}

	usleep_since_ts(lhbm->last_vblank_ts, offset_us);
}

static void local_hbm_wait_and_send_post_cmd(struct exynos_panel *ctx, struct drm_crtc *crtc)
{
	const u32 per_frame_us = get_current_frame_duration_us(ctx);
	u32 frames = ctx->desc->lhbm_post_cmd_delay_frames;
	struct local_hbm *lhbm = &ctx->hbm.local_hbm;

	if (frames == 0)
		return;

	if (crtc)
		/* wait for 0.5 frame time to ensure panel internal scanout or vsync has started */
		local_hbm_wait_vblank_and_delay(ctx, crtc, frames, per_frame_us / 2);
	else
		/* align with the time of sending enabling cmd */
		usleep_since_ts(lhbm->en_cmd_ts, per_frame_us * frames);

	dev_dbg(ctx->dev, "%s: delay(us): %lld(EN), %lld(TE)\n", __func__,
		ktime_us_delta(ktime_get(), lhbm->en_cmd_ts),
		lhbm->next_vblank_ts ? ktime_us_delta(ktime_get(), lhbm->next_vblank_ts) : 0);
	if (ctx->desc->exynos_panel_func->set_local_hbm_mode_post) {
		mutex_lock(&ctx->mode_lock);
		ctx->desc->exynos_panel_func->set_local_hbm_mode_post(ctx);
		mutex_unlock(&ctx->mode_lock);
	}
}

static void local_hbm_wait_and_notify_effectiveness(struct exynos_panel *ctx, struct drm_crtc *crtc)
{
	const u32 per_frame_us = get_current_frame_duration_us(ctx);
	const u32 offset_us = per_frame_us * 4 / 5;
	u32 frames = ctx->desc->lhbm_effective_delay_frames;
	struct local_hbm *lhbm = &ctx->hbm.local_hbm;

	if (frames == 0)
		return;

	if (crtc)
		/* wait for 0.8 frame time to ensure finishing LHBM spot scanout */
		local_hbm_wait_vblank_and_delay(ctx, crtc, frames, offset_us);
	else
		/* take worst case (cmd sent immediately after last vsync) into account */
		usleep_since_ts(lhbm->en_cmd_ts, per_frame_us * frames + offset_us);

	dev_dbg(ctx->dev, "%s: delay(us): %lld(EN), %lld(TE)\n", __func__,
		ktime_us_delta(ktime_get(), lhbm->en_cmd_ts),
		lhbm->next_vblank_ts ? ktime_us_delta(ktime_get(), lhbm->next_vblank_ts) : 0);
	if (lhbm->effective_state == LOCAL_HBM_ENABLING) {
		lhbm->effective_state = LOCAL_HBM_ENABLED;
		sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
	} else {
		dev_warn(ctx->dev, "%s: LHBM state = %d before becoming effective\n", __func__,
			 lhbm->effective_state);
	}
}

static void local_hbm_post_work(struct kthread_work *work)
{
	struct exynos_panel *ctx = container_of(work, struct exynos_panel, hbm.local_hbm.post_work);
	const struct exynos_panel_desc *desc = ctx->desc;
	struct drm_crtc *crtc = get_exynos_panel_connector_crtc(ctx);
	DPU_ATRACE_BEGIN(__func__);
	if (crtc && drm_crtc_vblank_get(crtc))
		crtc = NULL;
	ctx->hbm.local_hbm.next_vblank_ts = 0;
	ctx->hbm.local_hbm.frame_index = 0;
	/* TODO: delay time might be inaccurate if refresh rate changes around here */
	if (desc->lhbm_post_cmd_delay_frames <= desc->lhbm_effective_delay_frames) {
		local_hbm_wait_and_send_post_cmd(ctx, crtc);
		local_hbm_wait_and_notify_effectiveness(ctx, crtc);
	} else {
		local_hbm_wait_and_notify_effectiveness(ctx, crtc);
		local_hbm_wait_and_send_post_cmd(ctx, crtc);
	}
	if (crtc)
		drm_crtc_vblank_put(crtc);
	DPU_ATRACE_END(__func__);
}

static void exynos_panel_normal_mode_work(struct work_struct *work)
{
	struct exynos_panel *ctx = container_of(work, struct exynos_panel, normal_mode_work.work);

	dev_info(ctx->dev, "%s\n", __func__);
	mutex_lock(&ctx->mode_lock);
	ctx->desc->exynos_panel_func->run_normal_mode_work(ctx);
	mutex_unlock(&ctx->mode_lock);
	schedule_delayed_work(&ctx->normal_mode_work,
			      msecs_to_jiffies(ctx->normal_mode_work_delay_ms));
}

static void hbm_data_init(struct exynos_panel *ctx)
{
	ctx->hbm.local_hbm.gamma_para_ready = false;
	ctx->hbm.local_hbm.max_timeout_ms = LOCAL_HBM_MAX_TIMEOUT_MS;
	ctx->hbm.local_hbm.requested_state = LOCAL_HBM_DISABLED;
	ctx->hbm.local_hbm.effective_state = LOCAL_HBM_DISABLED;
	ctx->hbm.wq = create_singlethread_workqueue("hbm_workq");
	if (!ctx->hbm.wq)
		dev_err(ctx->dev, "failed to create hbm workq!\n");
	else {
		INIT_DELAYED_WORK(&ctx->hbm.local_hbm.timeout_work, local_hbm_timeout_work);
	}

	if (is_local_hbm_post_enabling_supported(ctx)) {
		kthread_init_worker(&ctx->hbm.local_hbm.worker);
		ctx->hbm.local_hbm.thread =
			kthread_run(kthread_worker_fn, &ctx->hbm.local_hbm.worker, "lhbm_kthread");
		if (IS_ERR(ctx->hbm.local_hbm.thread))
			dev_err(ctx->dev, "failed to run display lhbm kthread\n");
		else {
			struct sched_param param = {
				.sched_priority = 2, // MAX_RT_PRIO - 1,
			};
			sched_setscheduler_nocheck(ctx->hbm.local_hbm.thread, SCHED_FIFO, &param);
			kthread_init_work(&ctx->hbm.local_hbm.post_work, local_hbm_post_work);
		}
	}
}

static void exynos_panel_te2_init(struct exynos_panel *ctx)
{
	struct te2_mode_data *data;
	const struct exynos_binned_lp *binned_lp;
	int i, j;
	int lp_idx = ctx->desc->num_modes;
	int lp_mode_count = ctx->desc->lp_mode_count ? : 1;
	int mode_count = ctx->desc->num_modes + lp_mode_count * (ctx->desc->num_binned_lp - 1);

	for (i = 0; i < ctx->desc->num_modes; i++) {
		const struct exynos_panel_mode *pmode = &ctx->desc->modes[i];

		data = &ctx->te2.mode_data[i];
		data->mode = &pmode->mode;
		data->timing.rising_edge = pmode->te2_timing.rising_edge;
		data->timing.falling_edge = pmode->te2_timing.falling_edge;
	}

	for (i = 0; i < lp_mode_count; i++) {
		int lp_mode_offset = lp_idx + i * (ctx->desc->num_binned_lp - 1);

		for_each_exynos_binned_lp(j, binned_lp, ctx) {
			int idx;

			/* ignore the first binned entry (off) */
			if (j == 0)
				continue;

			idx = lp_mode_offset + j - 1;
			if (idx >= mode_count) {
				dev_warn(ctx->dev,
					 "idx %d exceeds mode size %d\n", idx, mode_count);
				return;
			}

			data = &ctx->te2.mode_data[idx];
			data->mode = &ctx->desc->lp_mode[i].mode;
			data->binned_lp = binned_lp;
			data->timing.rising_edge =
					binned_lp->te2_timing.rising_edge;
			data->timing.falling_edge =
					binned_lp->te2_timing.falling_edge;
		}
	}

	ctx->te2.option = TE2_OPT_CHANGEABLE;
}

static const struct drm_bridge_funcs exynos_panel_bridge_funcs = {
	.attach = exynos_panel_bridge_attach,
	.detach = exynos_panel_bridge_detach,
	.atomic_check = exynos_panel_bridge_atomic_check,
	.atomic_pre_enable = exynos_panel_bridge_pre_enable,
	.atomic_enable = exynos_panel_bridge_enable,
	.atomic_disable = exynos_panel_bridge_disable,
	.atomic_post_disable = exynos_panel_bridge_post_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.mode_set = exynos_panel_bridge_mode_set,
};

#ifdef CONFIG_OF
static void devm_backlight_release(void *data)
{
	struct backlight_device *bd = data;

	if (bd)
		put_device(&bd->dev);
}

static int exynos_panel_of_backlight(struct exynos_panel *ctx)
{
	struct device *dev;
	struct device_node *np;
	struct backlight_device *bd;
	int ret;

	dev = ctx->panel.dev;
	if (!dev)
		return -EINVAL;

	if (!dev->of_node)
		return 0;

	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (!np)
		return 0;

	bd = of_find_backlight_by_node(np);
	of_node_put(np);
	if (IS_ERR_OR_NULL(bd))
		return -EPROBE_DEFER;
	ctx->panel.backlight = bd;
	ret = devm_add_action(dev, devm_backlight_release, bd);
	if (ret) {
		put_device(&bd->dev);
		return ret;
	}
	ctx->bl_ctrl_dcs = of_property_read_bool(dev->of_node, "bl-ctrl-dcs");
	dev_info(ctx->dev, "succeed to register devtree backlight phandle\n");
	return 0;
}
#else
static int exynos_panel_of_backlight(struct exynos_panel *ctx)
{
	return 0;
}
#endif

static void exynos_panel_check_mode_clock(struct exynos_panel *ctx,
					  const struct drm_display_mode *mode)
{
	int clk_hz = mode->htotal * mode->vtotal * drm_mode_vrefresh(mode);
	int clk_khz = DIV_ROUND_CLOSEST(clk_hz, 1000);

	if (clk_khz == mode->clock) {
		/* clock should be divisible by 1000 to get an accurate value */
		if (!(clk_hz % 1000))
			dev_info(ctx->dev, "mode %s, clock %dkhz\n", mode->name, clk_khz);
		else
			dev_warn(ctx->dev, "mode %s, clock %dkhz is invalid!\n",
				 mode->name, clk_khz);
	} else {
		dev_warn(ctx->dev, "mode %s, clock %dkhz is different from calculation %dkhz!\n",
			 mode->name, mode->clock, clk_khz);
	}
}

static size_t disp_stats_update_vrefresh_range(const int vrefresh,
						const size_t current_count, int *vrefresh_range)
{
	int i;
	size_t count = 0;

	for (i = 0; i < MAX_VREFRESH_RANGES; i++) {
		if (i == current_count) {
			vrefresh_range[i] = vrefresh;
			count++;
			break;
		} else if (vrefresh_range[i] == vrefresh) {
			break;
		}
	}

	return count;
}

static void disp_stats_init(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	struct display_stats *stats = &ctx->disp_stats;
	enum display_state init_state;
	int i, j, time_state_idx;
	size_t available_count;

	if (ctx->desc->resolution_table) {
		stats->res_table_count = ctx->desc->resolution_table_count;
		if (stats->res_table_count > MAX_RESOLUTION_TABLES) {
			dev_warn(ctx->dev, "exceed max count of resolution table\n");
			stats->res_table_count = MAX_RESOLUTION_TABLES;
		}
		for (i = 0; i < stats->res_table_count; i++)
			stats->res_table[i] = ctx->desc->resolution_table[i];
	} else {
		dev_dbg(ctx->dev, "find available resolution from modes\n");
		for (i = 0; i < ctx->desc->num_modes; i++) {
			const struct exynos_panel_mode *pmode = &ctx->desc->modes[i];

			for (j = 0; j < MAX_RESOLUTION_TABLES; j++) {
				struct display_resolution *res =  &stats->res_table[j];

				if (j == stats->res_table_count) {
					res->hdisplay = pmode->mode.hdisplay;
					res->vdisplay = pmode->mode.vdisplay;
					stats->res_table_count++;
					break;
				} else if ((res->hdisplay == pmode->mode.hdisplay) &&
					(res->vdisplay == pmode->mode.vdisplay)) {
					break;
				}
			}
		}
	}

	if (ctx->desc->vrefresh_range) {
		stats->vrefresh_range_count = ctx->desc->vrefresh_range_count;
		if (stats->vrefresh_range_count > MAX_VREFRESH_RANGES) {
			dev_warn(ctx->dev, "exceed max count of vrefresh range\n");
			stats->vrefresh_range_count = MAX_VREFRESH_RANGES;
		}
		for (i = 0; i < stats->vrefresh_range_count; i++)
			stats->vrefresh_range[i] = ctx->desc->vrefresh_range[i];
	} else {
		dev_dbg(ctx->dev, "find available vrefresh from modes\n");
		for (i = 0; i < ctx->desc->num_modes; i++) {
			const struct exynos_panel_mode *pmode = &ctx->desc->modes[i];
			const int vrefresh = drm_mode_vrefresh(&pmode->mode);
			int *vrefresh_range = stats->vrefresh_range;

			stats->vrefresh_range_count +=
				disp_stats_update_vrefresh_range(vrefresh,
					stats->vrefresh_range_count, vrefresh_range);
		}
	}
	available_count = stats->res_table_count *
				stats->vrefresh_range_count;
	// DISPLAY_STATE_ON
	stats->time_in_state[DISPLAY_STATE_ON].available_count =
			available_count;

	// DISPLAY_STATE_HBM
	if (funcs && funcs->set_hbm_mode)
		stats->time_in_state[DISPLAY_STATE_HBM].available_count =
			available_count;

	// DISPLAY_STATE_LP
	if (ctx->desc->lp_vrefresh_range) {
		stats->lp_vrefresh_range_count = ctx->desc->lp_vrefresh_range_count;
		if (stats->lp_vrefresh_range_count > MAX_VREFRESH_RANGES) {
			dev_warn(ctx->dev, "exceed max count of lp vrefresh range\n");
			stats->lp_vrefresh_range_count = MAX_VREFRESH_RANGES;
		}
		for (i = 0; i < stats->lp_vrefresh_range_count; i++)
			stats->lp_vrefresh_range[i] = ctx->desc->lp_vrefresh_range[i];
	} else if (ctx->desc->lp_mode) {
		const size_t lp_mode_count = ctx->desc->lp_mode_count ? : 1;

		dev_dbg(ctx->dev, "find available lp vrefresh from lp modes\n");
		for (i = 0; i < lp_mode_count; i++) {
			const struct exynos_panel_mode *pmode = &ctx->desc->lp_mode[i];
			const int vrefresh = drm_mode_vrefresh(&pmode->mode);
			int *vrefresh_range = stats->lp_vrefresh_range;

			stats->lp_vrefresh_range_count +=
				disp_stats_update_vrefresh_range(vrefresh,
					stats->lp_vrefresh_range_count, vrefresh_range);
		}
	}
	stats->time_in_state[DISPLAY_STATE_LP].available_count =
		stats->res_table_count * stats->lp_vrefresh_range_count;

	stats->time_in_state[DISPLAY_STATE_OFF].available_count = 1;

	/* setting init display mode */
	if (is_panel_enabled(ctx) && ctx->current_mode) {
		struct display_resolution init_res;
		int init_vrefresh;

		init_state = DISPLAY_STATE_ON;
		init_res.hdisplay = ctx->current_mode->mode.hdisplay;
		init_res.vdisplay = ctx->current_mode->mode.vdisplay;
		init_vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);
		time_state_idx = get_disp_stats_time_state_idx(ctx, init_state,
				init_vrefresh, init_res);
		if (time_state_idx < 0) {
			init_state = DISPLAY_STATE_OFF;
			time_state_idx = 0;
			dev_dbg(ctx->dev, "time state init mode: OFF");
		} else {
			dev_dbg(ctx->dev, "time state init mode: %s",
				ctx->current_mode->mode.name[0] ?
				ctx->current_mode->mode.name : "NA");
		}
	} else {
		init_state = DISPLAY_STATE_OFF;
		time_state_idx = 0;
		dev_dbg(ctx->dev, "time state init mode: OFF");
	}

	stats->last_state = init_state;
	stats->last_time_state_idx = time_state_idx;

	/* allocate memory for time state */
	for (i = 0; i < DISPLAY_STATE_MAX; i++) {
		struct display_time_state *t = &stats->time_in_state[i];

		if (t->available_count) {
			t->time = devm_kcalloc(ctx->dev,
				t->available_count, sizeof(*t->time),
				GFP_KERNEL);
			if (!t->time)
				goto free_time_state;
		}
	}

	mutex_init(&stats->lock);
	stats->initialized = true;

	return;

free_time_state:
	for (i = 0; i < DISPLAY_STATE_MAX; i++) {
		if (stats->time_in_state[i].time) {
			devm_kfree(ctx->dev, stats->time_in_state[i].time);
			stats->time_in_state[i].time = NULL;
		}
	}
}

static int disp_stats_update_state(struct exynos_panel *ctx)
{
	struct display_stats *stats = &ctx->disp_stats;
	struct display_resolution cur_res;
	enum display_state cur_state, last_state;
	int cur_vrefresh, cur_time_state_idx, last_time_state_idx;
	u64 cur_time, delta_ms;

	if (!stats->initialized)
		return -1;

	mutex_lock(&ctx->bl_state_lock);
	cur_state = get_current_display_state_locked(ctx);
	mutex_unlock(&ctx->bl_state_lock);

	mutex_lock(&ctx->mode_lock);
	if (unlikely(!ctx->current_mode)) {
		dev_warn(ctx->dev, "%s: current mode is null\n", __func__);
		mutex_unlock(&ctx->mode_lock);
		return -1;
	}
	cur_vrefresh = exynos_get_actual_vrefresh(ctx);
	cur_res.hdisplay = ctx->current_mode->mode.hdisplay;
	cur_res.vdisplay = ctx->current_mode->mode.vdisplay;
	mutex_unlock(&ctx->mode_lock);

	mutex_lock(&stats->lock);
	cur_time = ktime_get_boottime();
	delta_ms = ktime_ms_delta(cur_time, stats->last_update);
	cur_time_state_idx = get_disp_stats_time_state_idx(ctx, cur_state, cur_vrefresh,
			cur_res);

	if (cur_time_state_idx < 0) {
		dev_err(ctx->dev, "%s: fail to find time stats idx for %ux%u@%d\n",
			__func__, cur_res.hdisplay, cur_res.vdisplay, cur_vrefresh);
		mutex_unlock(&stats->lock);
		return -1;
	}

	last_state = stats->last_state;
	last_time_state_idx = stats->last_time_state_idx;
	stats->time_in_state[last_state].time[last_time_state_idx] +=
		delta_ms;
	stats->last_time_state_idx = cur_time_state_idx;
	stats->last_state = cur_state;
	stats->last_update = cur_time;

	mutex_unlock(&stats->lock);

	return 0;
}


static void notify_panel_mode_changed_worker(struct work_struct *work)
{
	struct notify_state_change *notify_state_change =
		container_of(work, struct notify_state_change, work);
	struct exynos_panel *ctx =
		container_of(notify_state_change, struct exynos_panel, notify_panel_mode_changed);
	enum display_state power_state;

	disp_stats_update_state(ctx);

	if (ctx->allow_wakeup_by_state_change && notify_state_change->abort_suspend) {
		if (unlikely(!notify_state_change->ws))
			dev_warn(ctx->dev, "wakeup source creation was unsuccessful\n");
		else
			pm_wakeup_ws_event(notify_state_change->ws, 0, true);
	}

	sysfs_notify(&ctx->bl->dev.kobj, NULL, "state");

	mutex_lock(&ctx->bl_state_lock);
	power_state = get_current_display_state_locked(ctx);
	mutex_unlock(&ctx->bl_state_lock);

	/* Avoid spurious notifications */
	if (power_state != ctx->notified_power_state) {
		sysfs_notify(&ctx->dev->kobj, NULL, "power_state");
		ctx->notified_power_state = power_state;
	}
}

static void notify_brightness_changed_worker(struct work_struct *work)
{
	struct exynos_panel *ctx =
		container_of(work, struct exynos_panel, notify_brightness_changed_work);

	sysfs_notify(&ctx->bl->dev.kobj, NULL, "brightness");
}

/**
 * parse_panel_index() - Resolves display index from dsi name
 * @dsi: dsi device corresponding to panel
 *
 * Assuming the dsi name follows the form "<idx>:<panel_name>...",
 * resolves the idx value into a valid panel_index
 *
 * Return: panel_index on success, -1 on error
 */
static int parse_panel_index(const struct mipi_dsi_device *dsi)
{
	if (dsi->name[1] != ':' || dsi->name[0] == '0')
		return DISPLAY_PANEL_INDEX_PRIMARY;
	else if (dsi->name[0] == '1')
		return DISPLAY_PANEL_INDEX_SECONDARY;
	else
		return -EINVAL;
}

int exynos_panel_common_init(struct mipi_dsi_device *dsi,
				struct exynos_panel *ctx)
{
	struct device *dev = &dsi->dev;
	int ret = 0;
	char name[32];
	const struct exynos_panel_funcs *exynos_panel_func;
	int i;
	u32 id = host_to_dsi(dsi->host)->panel_id;

	dev_dbg(dev, "%s +\n", __func__);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc) {
		dev_err(dev, "No device match found, exiting init\n");
		return -EINVAL;
	}

	dsi->lanes = ctx->desc->data_lane_cnt;
	dsi->format = MIPI_DSI_FMT_RGB888;

	ctx->panel_index = parse_panel_index(dsi);
	if (ctx->panel_index < 0) {
		dev_err(dev, "Invalid panel panel_index parsed from dsi name %s\n", dsi->name);
		return -EINVAL;
	}

	ret = exynos_panel_parse_dt(ctx);
	if (ret)
		return ret;

	ret = exynos_panel_register_irqs(ctx);
	if (ret)
		return ret;

	exynos_panel_func = ctx->desc->exynos_panel_func;

	if (id != INVALID_PANEL_ID) {
		exynos_bin2hex(&id, EXT_INFO_SIZE, ctx->panel_extinfo, sizeof(ctx->panel_extinfo));

		if (exynos_panel_func && exynos_panel_func->get_panel_rev)
			exynos_panel_func->get_panel_rev(ctx, id);
	}
	else
		dev_dbg(ctx->dev, "Invalid panel id passed from bootloader");

	if (exynos_panel_func && exynos_panel_func->panel_config) {
		ret = exynos_panel_func ->panel_config(ctx);
		if (ret) {
			dev_err(ctx->dev, "failed to configure panel settings\n");
			return ret;
		}
	}

	if (ctx->panel_model[0] == '\0')
		scnprintf(ctx->panel_model, PANEL_MODEL_MAX, "Common Panel");

	scnprintf(name, sizeof(name), "panel%d-backlight", ctx->panel_index);
	ctx->bl = devm_backlight_device_register(ctx->dev, name, dev,
			ctx, &exynos_backlight_ops, NULL);
	if (IS_ERR(ctx->bl)) {
		dev_err(ctx->dev, "failed to register backlight device\n");
		return PTR_ERR(ctx->bl);
	}

	ctx->notify_panel_mode_changed.ws = wakeup_source_register(dev, name);
	if (!ctx->notify_panel_mode_changed.ws)
		dev_warn(ctx->dev, "failed to register `%s` wakeup source\n", name);

	ctx->bl->props.max_brightness = ctx->desc->max_brightness;
	ctx->bl->props.brightness = ctx->desc->dft_brightness;

	if (exynos_panel_func && (exynos_panel_func->set_hbm_mode
				  || exynos_panel_func->set_local_hbm_mode))
		hbm_data_init(ctx);

	if (exynos_panel_func && exynos_panel_func->get_te2_edges &&
	    exynos_panel_func->configure_te2_edges &&
	    exynos_panel_func->update_te2)
		exynos_panel_te2_init(ctx);

	if (ctx->desc->bl_num_ranges) {
		ctx->bl_notifier.num_ranges = ctx->desc->bl_num_ranges;
		if (ctx->bl_notifier.num_ranges > MAX_BL_RANGES) {
			dev_warn(ctx->dev, "exceed max number of bl range\n");
			ctx->bl_notifier.num_ranges = MAX_BL_RANGES;
		}

		for (i = 0; i < ctx->bl_notifier.num_ranges; i++)
			ctx->bl_notifier.ranges[i] = ctx->desc->bl_range[i];
	}

	for (i = 0; i < ctx->desc->num_modes; i++) {
		const struct exynos_panel_mode *pmode = &ctx->desc->modes[i];
		const int vrefresh = drm_mode_vrefresh(&pmode->mode);
		const int bts_fps = exynos_drm_mode_bts_fps(&pmode->mode,
			pmode->exynos_mode.min_bts_fps);

		if (ctx->peak_vrefresh < vrefresh)
			ctx->peak_vrefresh = vrefresh;

		if (ctx->peak_bts_fps < bts_fps)
			ctx->peak_bts_fps = bts_fps;

		exynos_panel_check_mode_clock(ctx, &pmode->mode);
	}

	for (i = 0; i < ctx->desc->lp_mode_count; i++) {
		const struct exynos_panel_mode *lp_mode = &ctx->desc->lp_mode[i];

		exynos_panel_check_mode_clock(ctx, &lp_mode->mode);
	}

	ctx->panel_idle_enabled = exynos_panel_func && exynos_panel_func->set_self_refresh != NULL;
	INIT_DELAYED_WORK(&ctx->idle_work, panel_idle_work);

	ctx->notified_power_state = DISPLAY_STATE_MAX;
	INIT_WORK(&ctx->notify_panel_mode_changed.work, notify_panel_mode_changed_worker);
	INIT_WORK(&ctx->notify_brightness_changed_work, notify_brightness_changed_worker);

	if (exynos_panel_func && exynos_panel_func->run_normal_mode_work &&
	    ctx->desc->normal_mode_work_delay_ms) {
		ctx->normal_mode_work_delay_ms = ctx->desc->normal_mode_work_delay_ms;
		INIT_DELAYED_WORK(&ctx->normal_mode_work, exynos_panel_normal_mode_work);
	}

	BLOCKING_INIT_NOTIFIER_HEAD(&ctx->notifier_head);

	if (ctx->desc->default_dsi_hs_clk_mbps)
		ctx->dsi_hs_clk_mbps = ctx->desc->default_dsi_hs_clk_mbps;

	mutex_init(&ctx->mode_lock);
	mutex_init(&ctx->crtc_lock);
	mutex_init(&ctx->bl_state_lock);
	mutex_init(&ctx->lp_state_lock);

	drm_panel_init(&ctx->panel, dev, ctx->desc->panel_func, DRM_MODE_CONNECTOR_DSI);

	ret = exynos_panel_of_backlight(ctx);
	if (ret) {
		dev_err(ctx->dev, "failed to register devtree backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_add(&ctx->panel);

	ctx->bridge.funcs = &exynos_panel_bridge_funcs;
#ifdef CONFIG_OF
	ctx->bridge.of_node = ctx->dev->of_node;
#endif
	drm_bridge_add(&ctx->bridge);

	ret = sysfs_create_files(&dev->kobj, panel_attrs);
	if (ret)
		pr_warn("unable to add panel sysfs files (%d)\n", ret);

	ret = sysfs_create_groups(&ctx->bl->dev.kobj, bl_device_groups);
	if (ret)
		dev_err(ctx->dev, "unable to create bl_device_groups groups\n");

	if (exynos_panel_func && exynos_panel_func->set_cabc_mode) {
		ret = sysfs_create_file(&ctx->bl->dev.kobj, &dev_attr_cabc_mode.attr);
		if (ret)
			dev_err(ctx->dev, "unable to create cabc_mode\n");
	}
	if (exynos_panel_func && exynos_panel_func->set_acl_mode) {
		/* set acl mode true as default */
		ctx->acl_mode = true;
		ret = sysfs_create_file(&ctx->bl->dev.kobj, &dev_attr_acl_mode.attr);
		if (ret)
			dev_err(ctx->dev, "unable to create acl_mode\n");
	}
	if (exynos_panel_func && exynos_panel_func->set_ssc_en) {
		dev_info(ctx->dev, "create ssc_en sysfs node\n");
		ret = sysfs_create_file(&ctx->bl->dev.kobj, &dev_attr_ssc_en.attr);
		if (ret)
			dev_err(ctx->dev, "unable to create ssc_en\n");
	}

	ctx->mode_in_progress = MODE_DONE;

	exynos_panel_handoff(ctx);

	disp_stats_init(ctx);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		goto err_panel;

	dev_info(ctx->dev, "samsung common panel driver has been probed\n");

	return 0;

err_panel:
	drm_bridge_remove(&ctx->bridge);
	drm_panel_remove(&ctx->panel);
	dev_err(ctx->dev, "failed to probe samsung panel driver(%d)\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_panel_common_init);

int exynos_panel_probe(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx;

	ctx = devm_kzalloc(&dsi->dev, sizeof(struct exynos_panel), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	return exynos_panel_common_init(dsi, ctx);
}
EXPORT_SYMBOL_GPL(exynos_panel_probe);

void exynos_panel_remove(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	drm_bridge_remove(&ctx->bridge);

	sysfs_remove_groups(&ctx->bl->dev.kobj, bl_device_groups);
	sysfs_remove_file(&ctx->bl->dev.kobj, &dev_attr_cabc_mode.attr);
	sysfs_remove_file(&ctx->bl->dev.kobj, &dev_attr_acl_mode.attr);
	sysfs_remove_file(&ctx->bl->dev.kobj, &dev_attr_ssc_en.attr);
	devm_backlight_device_unregister(ctx->dev, ctx->bl);
}
EXPORT_SYMBOL_GPL(exynos_panel_remove);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung common panel driver");
MODULE_LICENSE("GPL");
