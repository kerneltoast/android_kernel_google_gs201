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

#include <trace/dpu_trace.h>
#include "../exynos_drm_connector.h"
#include "../exynos_drm_dsim.h"
#include "panel-samsung-drv.h"

#define PANEL_ID_REG		0xA1
#define PANEL_ID_LEN		7
#define PANEL_ID_OFFSET		6
#define PANEL_ID_READ_SIZE	(PANEL_ID_LEN + PANEL_ID_OFFSET)
#define PANEL_SLSI_DDIC_ID_REG	0xD6
#define PANEL_SLSI_DDIC_ID_LEN	5

static const char ext_info_regs[] = { 0xDA, 0xDB, 0xDC };
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
static const struct exynos_panel_mode *exynos_panel_get_mode(struct exynos_panel *ctx,
							     const struct drm_display_mode *mode);
static void panel_update_local_hbm_locked(struct exynos_panel *ctx, bool enable);
static void exynos_panel_check_mipi_sync_timing(struct drm_crtc *crtc,
					 const struct exynos_panel_mode *current_mode,
					 struct exynos_panel *ctx);

static inline bool is_backlight_off_state(const struct backlight_device *bl)
{
	return (bl->props.state & BL_STATE_STANDBY) != 0;
}

static inline bool is_backlight_lp_state(const struct backlight_device *bl)
{
	return (bl->props.state & BL_STATE_LP) != 0;
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
EXPORT_SYMBOL(exynos_panel_configure_te2_edges);

ssize_t exynos_panel_get_te2_edges(struct exynos_panel *ctx,
				   char *buf, bool lp_mode)
{
	struct te2_mode_data *data;
	size_t len = 0;
	int i;

	if (!ctx)
		return -EINVAL;

	for_each_te2_timing(ctx, lp_mode, data, i) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%dx%d@%d",
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
EXPORT_SYMBOL(exynos_panel_get_te2_edges);

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
EXPORT_SYMBOL(exynos_panel_get_current_mode_te2);

static void exynos_panel_update_te2(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!is_panel_active(ctx) || !funcs || !funcs->update_te2)
		return;

	funcs->update_te2(ctx);

	if (ctx->bl)
		te2_state_changed(ctx->bl);
}

static int exynos_panel_parse_gpios(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;

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
EXPORT_SYMBOL(exynos_panel_read_id);

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
EXPORT_SYMBOL(exynos_panel_read_ddic_id);

static int exynos_panel_read_extinfo(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[EXT_INFO_SIZE];
	int i, ret;

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
	default:
		dev_warn(ctx->dev,
			 "unknown rev from panel (0x%x), default to latest\n",
			 rev);
		ctx->panel_rev = PANEL_REV_LATEST;
		return;
	}

	dev_info(ctx->dev, "panel_rev: 0x%x\n", ctx->panel_rev);
}
EXPORT_SYMBOL(exynos_panel_get_panel_rev);

int exynos_panel_init(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (ctx->initialized)
		return 0;

	ret = exynos_panel_read_extinfo(ctx);
	if (!ret)
		ctx->initialized = true;

	if (funcs && funcs->get_panel_rev) {
		u32 id;

		if (kstrtou32(ctx->panel_extinfo, 16, &id)) {
			dev_warn(ctx->dev,
				 "failed to get panel extinfo, default to latest\n");
			ctx->panel_rev = PANEL_REV_LATEST;
		} else {
			funcs->get_panel_rev(ctx, id);
		}
	} else {
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

	return ret;
}
EXPORT_SYMBOL(exynos_panel_init);

void exynos_panel_reset(struct exynos_panel *ctx)
{
	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return;

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);

	dev_dbg(ctx->dev, "%s -\n", __func__);

	ctx->is_brightness_initialized = false;
	exynos_panel_init(ctx);
}
EXPORT_SYMBOL(exynos_panel_reset);

static void _exynos_panel_set_vddd_voltage(struct exynos_panel *ctx, bool is_lp)
{
	u32 uv = is_lp ? ctx->vddd_lp_uV : ctx->vddd_normal_uV;

	if (!uv || !ctx->vddd)
		return;
	if (regulator_set_voltage(ctx->vddd, uv, uv))
		dev_err(ctx->dev, "failed to set vddd at %u uV\n", uv);
}

static int _exynos_panel_set_power(struct exynos_panel *ctx, bool on)
{
	int ret;

	if (on) {
		if (ctx->enable_gpio) {
			gpiod_set_value(ctx->enable_gpio, 1);
			usleep_range(10000, 11000);
		}

		if (ctx->vddi) {
			ret = regulator_enable(ctx->vddi);
			if (ret) {
				dev_err(ctx->dev, "vddi enable failed\n");
				return ret;
			}
			usleep_range(5000, 6000);
		}

		if (ctx->vddd) {
			ret = regulator_enable(ctx->vddd);
			if (ret) {
				dev_err(ctx->dev, "vddd enable failed\n");
				return ret;
			}
		}

		if (ctx->vci) {
			ret = regulator_enable(ctx->vci);
			if (ret) {
				dev_err(ctx->dev, "vci enable failed\n");
				return ret;
			}
		}

		if (ctx->vddr_en) {
			ret = regulator_enable(ctx->vddr_en);
			if (ret) {
				dev_err(ctx->dev, "vddr_en enable failed\n");
				return ret;
			}
			usleep_range(2 * 1000, 2 * 1000 + 10);
		}

		if (ctx->vddr) {
			ret = regulator_enable(ctx->vddr);
			if (ret) {
				dev_err(ctx->dev, "vddr enable failed\n");
				return ret;
			}
		}
	} else {
		gpiod_set_value(ctx->reset_gpio, 0);
		if (ctx->enable_gpio)
			gpiod_set_value(ctx->enable_gpio, 0);

		if (ctx->vddr) {
			ret = regulator_disable(ctx->vddr);
			if (ret) {
				dev_err(ctx->dev, "vddr disable failed\n");
				return ret;
			}
		}

		if (ctx->vddr_en) {
			ret = regulator_disable(ctx->vddr_en);
			if (ret) {
				dev_err(ctx->dev, "vddr_en disable failed\n");
				return ret;
			}
		}

		if (ctx->vddd) {
			ret = regulator_disable(ctx->vddd);
			if (ret) {
				dev_err(ctx->dev, "vddd disable failed\n");
				return ret;
			}
		}

		if (ctx->vddi) {
			ret = regulator_disable(ctx->vddi);
			if (ret) {
				dev_err(ctx->dev, "vddi disable failed\n");
				return ret;
			}
		}

		if (ctx->vci > 0) {
			ret = regulator_disable(ctx->vci);
			if (ret) {
				dev_err(ctx->dev, "vci disable failed\n");
				return ret;
			}
		}
	}

	return 0;
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
EXPORT_SYMBOL(exynos_panel_set_power);

static void exynos_panel_handoff(struct exynos_panel *ctx)
{
	ctx->enabled = gpiod_get_raw_value(ctx->reset_gpio) > 0;
	_exynos_panel_set_vddd_voltage(ctx, false);
	if (ctx->enabled) {
		dev_info(ctx->dev, "panel enabled at boot\n");
		ctx->panel_state = PANEL_STATE_HANDOFF;
		ctx->is_brightness_initialized = true;
		exynos_panel_set_power(ctx, true);
	} else {
		ctx->panel_state = PANEL_STATE_UNINITIALIZED;
		gpiod_direction_output(ctx->reset_gpio, 0);
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

static void exynos_panel_mode_set_name(struct drm_display_mode *mode)
{
	scnprintf(mode->name, DRM_DISPLAY_MODE_LEN, "%dx%dx%d",
		  mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode));
}

int exynos_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	struct drm_display_mode *preferred_mode = NULL;
	const struct exynos_panel_mode *current_mode = ctx->current_mode;
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

		if (!preferred_mode || (mode->type & DRM_MODE_TYPE_PREFERRED)) {
			preferred_mode = mode;
			/* if enabled at boot, assume preferred mode was set */
			if ((ctx->panel_state == PANEL_STATE_HANDOFF) && !current_mode)
				ctx->current_mode = pmode;
		}
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
EXPORT_SYMBOL(exynos_panel_get_modes);

int exynos_panel_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_funcs *exynos_panel_func;

	ctx->enabled = false;
	ctx->hbm_mode = HBM_OFF;
	ctx->dimming_on = false;
	ctx->self_refresh_active = false;
	ctx->panel_idle_vrefresh = 0;
	ctx->current_binned_lp = NULL;
	ctx->cabc_mode = CABC_OFF;
	ctx->current_cabc_mode = CABC_OFF;

	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (exynos_panel_func) {
		if (exynos_panel_func->set_local_hbm_mode) {
			cancel_delayed_work_sync(&ctx->hbm.local_hbm.timeout_work);
			ctx->hbm.local_hbm.state = LOCAL_HBM_DISABLED;
			sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
		}
	}

	mutex_lock(&ctx->mode_lock);
	exynos_panel_send_cmd_set(ctx, ctx->desc->off_cmd_set);
	mutex_unlock(&ctx->mode_lock);
	dev_dbg(ctx->dev, "%s\n", __func__);
	return 0;
}
EXPORT_SYMBOL(exynos_panel_disable);

int exynos_panel_unprepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	exynos_panel_set_power(ctx, false);
	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}
EXPORT_SYMBOL(exynos_panel_unprepare);

int exynos_panel_prepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	exynos_panel_set_power(ctx, true);
	dev_dbg(ctx->dev, "%s -\n", __func__);

	return 0;
}
EXPORT_SYMBOL(exynos_panel_prepare);

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
	if (!(flags & async_mask))
		dsi_flags |= MIPI_DSI_MSG_LASTCOMMAND;

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
			dsi_flags |= MIPI_DSI_MSG_LASTCOMMAND;

		exynos_dsi_dcs_write_buffer(dsi, c->cmd, c->cmd_len, dsi_flags);
		if (delay_ms)
			usleep_range(delay_ms * 1000, delay_ms * 1000 + 10);
	}
}
EXPORT_SYMBOL(exynos_panel_send_cmd_set_flags);

void exynos_panel_set_lp_mode(struct exynos_panel *ctx, const struct exynos_panel_mode *pmode)
{
	exynos_panel_send_cmd_set(ctx, ctx->desc->lp_cmd_set);

	dev_info(ctx->dev, "enter %dhz LP mode\n", drm_mode_vrefresh(&pmode->mode));
}
EXPORT_SYMBOL(exynos_panel_set_lp_mode);

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
EXPORT_SYMBOL(exynos_panel_set_binned_lp);

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
EXPORT_SYMBOL(exynos_panel_set_brightness);

static int exynos_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static void exynos_panel_set_cabc(struct exynos_panel *ctx, enum exynos_cabc_mode cabc_mode)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	struct backlight_device *bl = ctx->bl;
	u8 mode;
	bool force_off = (bl->props.brightness <= ctx->desc->min_brightness);

	if (!funcs || !funcs->set_cabc_mode)
		return;

	/* force off will not change the cabc_mode node */
	mode = !force_off ? cabc_mode : CABC_OFF;
	if (ctx->current_cabc_mode != mode) {
		funcs->set_cabc_mode(ctx, mode);
		ctx->current_cabc_mode = mode;
	}
	ctx->cabc_mode = cabc_mode;
	dev_dbg(ctx->dev, "set cabc mode: %d, force_off: %d\n", cabc_mode, force_off);
}

static int exynos_bl_find_range(struct exynos_panel *ctx,
				int brightness, u32 *range)
{
	u32 i;

	if (!ctx->bl_notifier.num_ranges)
		return -EOPNOTSUPP;

	mutex_lock(&ctx->bl_state_lock);

	for (i = 0; i < ctx->bl_notifier.num_ranges; i++) {
		if (brightness <= ctx->bl_notifier.ranges[i]) {
			*range = i;
			mutex_unlock(&ctx->bl_state_lock);

			return 0;
		}
	}

	mutex_unlock(&ctx->bl_state_lock);

	dev_warn(ctx->dev, "failed to find bl range\n");

	return -EINVAL;
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
	    exynos_bl_find_range(ctx, brightness, &bl_range) >= 0 &&
	    bl_range != ctx->bl_notifier.current_range) {
		ctx->bl_notifier.current_range = bl_range;

		sysfs_notify(&ctx->bl->dev.kobj, NULL, "brightness");

		dev_dbg(ctx->dev, "bl range is changed to %d\n",
			ctx->bl_notifier.current_range);
	}

	if (ctx->cabc_mode && brightness)
		exynos_panel_set_cabc(ctx, ctx->cabc_mode);
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
EXPORT_SYMBOL(panel_get_idle_time_delta);

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

static void panel_update_idle_mode_locked(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	WARN_ON(!mutex_is_locked(&ctx->mode_lock));

	if (unlikely(!ctx->current_mode || !funcs))
		return;

	if (!is_panel_active(ctx) || !funcs->set_self_refresh)
		return;

	if (ctx->idle_delay_ms && ctx->self_refresh_active && panel_idle_queue_delayed_work(ctx))
		return;

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
	panel_update_idle_mode_locked(ctx);
	mutex_unlock(&ctx->mode_lock);
}

static ssize_t panel_idle_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	bool idle_enabled;
	int ret;

	ret = kstrtobool(buf, &idle_enabled);
	if (ret) {
		dev_err(dev, "invalid panel idle value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	if (idle_enabled != ctx->panel_idle_enabled) {
		ctx->panel_idle_enabled = idle_enabled;

		if (idle_enabled)
			ctx->last_panel_idle_set_ts = ktime_get();

		panel_update_idle_mode_locked(ctx);
	}
	mutex_unlock(&ctx->mode_lock);

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

	mutex_lock(&ctx->mode_lock);
	ctx->min_vrefresh = min_vrefresh;
	panel_update_idle_mode_locked(ctx);
	mutex_unlock(&ctx->mode_lock);

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

	mutex_lock(&ctx->mode_lock);
	ctx->idle_delay_ms = idle_delay_ms;
	panel_update_idle_mode_locked(ctx);
	mutex_unlock(&ctx->mode_lock);

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

static DEVICE_ATTR_RO(serial_number);
static DEVICE_ATTR_RO(panel_extinfo);
static DEVICE_ATTR_RO(panel_name);
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

static const struct attribute *panel_attrs[] = {
	&dev_attr_serial_number.attr,
	&dev_attr_panel_extinfo.attr,
	&dev_attr_panel_name.attr,
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

		drm_printf(p, " \tcurrent mode: %dx%d@%d\n", m->hdisplay,
			   m->vdisplay, drm_mode_vrefresh(m));
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
	} else if (property == p->mipi_sync) {
		exynos_state->mipi_sync = val;
		dev_dbg(ctx->dev, "%s: mipi_sync(0x%lx)\n", __func__, exynos_state->mipi_sync);
	} else
		return -EINVAL;

	return 0;
}

static const struct exynos_drm_connector_funcs exynos_panel_connector_funcs = {
	.atomic_print_state = exynos_panel_connector_print_state,
	.atomic_get_property = exynos_panel_connector_get_property,
	.atomic_set_property = exynos_panel_connector_set_property,
};

static void exynos_panel_set_dimming(struct exynos_panel *ctx, bool dimming_on)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!funcs || !funcs->set_dimming_on)
		return;

	mutex_lock(&ctx->mode_lock);
	if (dimming_on != ctx->dimming_on) {
		funcs->set_dimming_on(ctx, dimming_on);
		panel_update_idle_mode_locked(ctx);
	}
	mutex_unlock(&ctx->mode_lock);
}

static void exynos_panel_pre_commit_properties(
				struct exynos_panel *ctx,
				struct exynos_drm_connector_state *conn_state)
{
	const struct exynos_panel_funcs *exynos_panel_func = ctx->desc->exynos_panel_func;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	bool mipi_sync;
	bool ghbm_updated = false;

	if (!conn_state->pending_update_flags)
		return;

	DPU_ATRACE_BEGIN(__func__);
	mipi_sync = conn_state->mipi_sync &
		(MIPI_CMD_SYNC_LHBM | MIPI_CMD_SYNC_GHBM | MIPI_CMD_SYNC_BL);

	if ((conn_state->mipi_sync & (MIPI_CMD_SYNC_LHBM | MIPI_CMD_SYNC_GHBM)) &&
		ctx->current_mode->exynos_mode.is_lp_mode) {
		dev_warn(ctx->dev,
			 "%s: skip LHBM/GHBM updates during lp mode, pending_update_flags(0x%x)\n",
			 __func__, conn_state->pending_update_flags);
		conn_state->pending_update_flags &= ~(HBM_FLAG_LHBM_UPDATE | HBM_FLAG_GHBM_UPDATE);
	}

	if (mipi_sync) {
		dev_info(ctx->dev, "%s: mipi_sync(0x%lx) pending_update_flags(0x%x)\n", __func__,
			 conn_state->mipi_sync, conn_state->pending_update_flags);
		exynos_panel_check_mipi_sync_timing(conn_state->base.crtc,
						    ctx->current_mode, ctx);
		exynos_dsi_dcs_write_buffer_force_batch_begin(dsi);
	}

	if ((conn_state->pending_update_flags & HBM_FLAG_GHBM_UPDATE) &&
		exynos_panel_func && exynos_panel_func->set_hbm_mode &&
		(ctx->hbm_mode != conn_state->global_hbm_mode)) {
		DPU_ATRACE_BEGIN("set_hbm");
		mutex_lock(&ctx->mode_lock);
		exynos_panel_func->set_hbm_mode(ctx, conn_state->global_hbm_mode);
		backlight_state_changed(ctx->bl);
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
		panel_update_local_hbm_locked(ctx, conn_state->local_hbm_on);
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
}

static void exynos_panel_connector_atomic_commit(
				struct exynos_drm_connector *exynos_connector,
			    struct exynos_drm_connector_state *exynos_old_state,
			    struct exynos_drm_connector_state *exynos_new_state)
{
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	const struct exynos_panel_funcs *exynos_panel_func = ctx->desc->exynos_panel_func;

	if (!exynos_panel_func)
		return;

	mutex_lock(&ctx->mode_lock);
	if (exynos_panel_func->commit_done && !ctx->current_mode->exynos_mode.is_lp_mode)
		exynos_panel_func->commit_done(ctx);
	mutex_unlock(&ctx->mode_lock);

	ctx->last_commit_ts = ktime_get();
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

static const struct exynos_panel_mode *exynos_panel_get_mode(struct exynos_panel *ctx,
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
		dev_warn(ctx->dev, "invalid mode %s\n", pmode->mode.name);
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
EXPORT_SYMBOL(exynos_panel_debugfs_create_cmdset);

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
	struct dentry *root;

	debugfs_create_u32("rev", 0600, parent, &ctx->panel_rev);

	if (!funcs)
		return -EINVAL;

	if (funcs->print_gamma)
		debugfs_create_file("gamma", 0600, parent, ctx, &panel_gamma_fops);

	root = debugfs_create_dir("cmdsets", ctx->debugfs_entry);
	if (!root) {
		dev_err(ctx->dev, "can't create cmdset dir\n");
		return -EFAULT;
	}
	ctx->debugfs_cmdset_entry = root;

	exynos_panel_debugfs_create_cmdset(ctx, root, desc->off_cmd_set, "off");

	if (desc->lp_mode) {
		struct dentry *lpd;
		int i;

		if (desc->binned_lp) {
			lpd = debugfs_create_dir("lp", root);
			if (!lpd) {
				dev_err(ctx->dev, "can't create lp dir\n");
				return -EFAULT;
			}

			for (i = 0; i < desc->num_binned_lp; i++) {
				const struct exynos_binned_lp *b = &desc->binned_lp[i];

				exynos_panel_debugfs_create_cmdset(ctx, lpd, &b->cmd_set, b->name);
			}
		} else {
			lpd = root;
		}
		exynos_panel_debugfs_create_cmdset(ctx, lpd, desc->lp_cmd_set, "lp_entry");
	}

	return 0;
}

static ssize_t exynos_dsi_dcs_transfer(struct mipi_dsi_device *dsi, u8 type,
				     const void *data, size_t len, u16 flags)
{
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.tx_buf = data,
		.tx_len = len,
		.type = type,
	};

	if (!ops || !ops->transfer)
		return -ENOSYS;

	msg.flags = flags;
	if (dsi->mode_flags & MIPI_DSI_MODE_LPM)
		msg.flags |= MIPI_DSI_MSG_USE_LPM;

	return ops->transfer(dsi->host, &msg);
}

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
EXPORT_SYMBOL(exynos_dsi_dcs_write_buffer);

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

static ssize_t exynos_debugfs_op_hz_write(struct file *file,
			       const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	uint32_t hz;
	int ret;
	struct seq_file *m = file->private_data;
	struct mipi_dsi_device *dsi = m->private;
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs;

	if (!is_panel_active(ctx))
		return -EPERM;

	funcs = ctx->desc->exynos_panel_func;
	if (!count || !funcs || !funcs->set_op_hz)
		return -EINVAL;

	ret = kstrtou32_from_user(user_buf, count, 0, &hz);
	if (ret) {
		dev_err(ctx->dev, "invalid op rate value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ret = funcs->set_op_hz(ctx, hz);
	mutex_unlock(&ctx->mode_lock);
	if (ret) {
		dev_err(ctx->dev, "failed to set op rate: %d Hz\n", hz);
		return ret;
	}

	return count;
}

static int exynos_debugfs_op_hz_show(struct seq_file *m, void *data)
{
	struct mipi_dsi_device *dsi = m->private;
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs;

	if (!is_panel_active(ctx))
		return -EPERM;

	funcs = ctx->desc->exynos_panel_func;
	if (!funcs || !funcs->set_op_hz)
		return -EINVAL;

	seq_printf(m, "%d\n", ctx->op_hz);
	return 0;
}

static int exynos_debugfs_op_hz_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_debugfs_op_hz_show, inode->i_private);
}

static const struct file_operations exynos_op_hz_fops = {
	.owner = THIS_MODULE,
	.open = exynos_debugfs_op_hz_open,
	.write = exynos_debugfs_op_hz_write,
	.read = seq_read,
};

static int exynos_dsi_debugfs_add(struct mipi_dsi_device *dsi,
			 struct dentry *parent)
{
	struct dentry *reg_root;
	struct exynos_dsi_reg_data *reg_data;
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	reg_root = debugfs_create_dir("reg", parent);
	if (!reg_root)
		return -EFAULT;

	reg_data = devm_kzalloc(&dsi->dev, sizeof(*reg_data), GFP_KERNEL);
	if (!reg_data)
		return -ENOMEM;

	reg_data->dsi = dsi;
	reg_data->flags = MIPI_DSI_MSG_LASTCOMMAND;

	debugfs_create_u8("address", 0600, reg_root, &reg_data->address);
	debugfs_create_u8("type", 0600, reg_root, &reg_data->type);
	debugfs_create_size_t("count", 0600, reg_root, &reg_data->count);
	debugfs_create_u16("flags", 0600, reg_root, &reg_data->flags);
	debugfs_create_file("payload", 0600, reg_root, reg_data,
			    &exynos_dsi_payload_fops);

	debugfs_create_file("name", 0600, parent, dsi, &exynos_dsi_name_fops);
	debugfs_create_file("reset_panel",0200, parent, dsi, &exynos_reset_panel_fops);

	if (ctx && ctx->desc->exynos_panel_func &&
		ctx->desc->exynos_panel_func->set_op_hz)
		debugfs_create_file("op_hz",0600, parent, dsi, &exynos_op_hz_fops);

	return 0;
}

static int exynos_debugfs_panel_add(struct exynos_panel *ctx, struct dentry *parent)
{
	struct dentry *root;

	if (!parent)
		return -EINVAL;

	root = debugfs_create_dir("panel", parent);
	if (!root)
		return -EPERM;

	ctx->debugfs_entry = root;

	return 0;
}

static void exynos_debugfs_panel_remove(struct exynos_panel *ctx)
{
	if (!ctx->debugfs_entry)
		return;

	debugfs_remove_recursive(ctx->debugfs_entry);

	ctx->debugfs_entry = NULL;
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

static int exynos_debugfs_panel_add(struct exynos_panel *ctx, struct dentry *parent)
{
	return 0;
}

static void exynos_debugfs_panel_remove(struct exynos_panel *ctx)
{
	return;
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
		backlight_state_changed(bd);
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

	mutex_lock(&ctx->mode_lock);
	exynos_panel_set_cabc(ctx, cabc_mode);
	mutex_unlock(&ctx->mode_lock);

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

static struct drm_crtc *get_exynos_panel_connector_crtc(struct exynos_panel *ctx)
{
	struct drm_mode_config *config;
	struct drm_crtc *crtc = NULL;

	config = &ctx->exynos_connector.base.dev->mode_config;
	drm_modeset_lock(&config->connection_mutex, NULL);
	if (ctx->exynos_connector.base.state)
		crtc = ctx->exynos_connector.base.state->crtc;
	drm_modeset_unlock(&config->connection_mutex);
	return crtc;
}

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
	panel_update_local_hbm_locked(ctx, local_hbm_en);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t local_hbm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->hbm.local_hbm.state);
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
	bool show_mode = true;
	const char *statestr;
	int rc, ret_cnt;

	mutex_lock(&ctx->bl_state_lock);

	if (is_backlight_off_state(bl)) {
		statestr = "Off";
		show_mode = false;
	} else if (is_backlight_lp_state(bl)) {
		statestr = "LP";
	} else if (IS_HBM_ON(ctx->hbm_mode)) {
		statestr = IS_HBM_ON_IRC_OFF(ctx->hbm_mode) ?
				"HBM IRC_OFF" : "HBM";
	} else {
		statestr = "On";
	}

	mutex_unlock(&ctx->bl_state_lock);

	ret_cnt = scnprintf(buf, PAGE_SIZE, "%s\n", statestr);
	rc = ret_cnt;

	if (rc > 0 && show_mode) {
		const struct exynos_panel_mode *pmode;

		mutex_lock(&ctx->mode_lock);
		pmode = ctx->current_mode;
		mutex_unlock(&ctx->mode_lock);
		if (pmode) {
			/* overwrite \n and continue the string */
			const u8 str_len = ret_cnt - 1;

			ret_cnt = scnprintf(buf + str_len, PAGE_SIZE - str_len,
				      ": %dx%d@%d\n",
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
		backlight_state_changed(bl);
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
	drm_object_attach_property(obj, p->mipi_sync, 0);
	drm_object_attach_property(obj, p->is_partial, desc->is_partial);
	drm_object_attach_property(obj, p->panel_idle_support, desc->is_panel_idle_supported);
	drm_object_attach_property(obj, p->panel_orientation, ctx->orientation);
	drm_object_attach_property(obj, p->vrr_switch_duration, desc->vrr_switch_duration);

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
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const char *p = !IS_ERR(dsi) ? dsi->name : NULL;

	if (p == NULL || p[1] != ':' || p[0] == '0')
		return "primary-panel";
	if (p[0] == '1')
		return "secondary-panel";

	dev_err(ctx->dev, "unsupported dsi device name %s\n", dsi->name);
	return "primary-panel";
}

static int exynos_panel_bridge_attach(struct drm_bridge *bridge,
				      enum drm_bridge_attach_flags flags)
{
	struct drm_device *dev = bridge->dev;
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_connector *connector = &ctx->exynos_connector.base;
	const char *sysfs_name = exynos_panel_get_sysfs_name(ctx);
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

	drm_connector_register(connector);

	drm_connector_attach_encoder(connector, bridge->encoder);
	connector->funcs->reset(connector);
	connector->status = connector_status_connected;
	if (ctx->desc->exynos_panel_func && ctx->desc->exynos_panel_func->commit_done)
		ctx->exynos_connector.needs_commit = true;

	ret = sysfs_create_link(&connector->kdev->kobj, &ctx->dev->kobj,
				"panel");
	if (ret)
		dev_warn(ctx->dev, "unable to link panel sysfs (%d)\n", ret);

	exynos_debugfs_panel_add(ctx, connector->debugfs_entry);
	exynos_dsi_debugfs_add(to_mipi_dsi_device(ctx->dev), ctx->debugfs_entry);
	panel_debugfs_add(ctx, ctx->debugfs_entry);

	drm_kms_helper_hotplug_event(connector->dev);


	ret = sysfs_create_link(&bridge->dev->dev->kobj, &ctx->dev->kobj, sysfs_name);
	if (ret)
		dev_warn(ctx->dev, "unable to link %s sysfs (%d)\n", sysfs_name, ret);
	else
		dev_dbg(ctx->dev, "succeed to link %s sysfs\n", sysfs_name);

	return 0;
}

static void exynos_panel_bridge_detach(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_connector *connector = &ctx->exynos_connector.base;
	const char *sysfs_name = exynos_panel_get_sysfs_name(ctx);

	sysfs_remove_link(&bridge->dev->dev->kobj, sysfs_name);

	exynos_debugfs_panel_remove(ctx);
	sysfs_remove_link(&connector->kdev->kobj, "panel");
	drm_connector_unregister(connector);
	drm_connector_cleanup(&ctx->exynos_connector.base);
}

static void exynos_panel_bridge_enable(struct drm_bridge *bridge,
				       struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	bool need_update_backlight = false;
	bool is_active;
	const bool is_lp_mode = ctx->current_mode &&
				ctx->current_mode->exynos_mode.is_lp_mode;

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

	if (ctx->self_refresh_active) {
		dev_dbg(ctx->dev, "self refresh state : %s\n", __func__);

		ctx->self_refresh_active = false;
		panel_update_idle_mode_locked(ctx);
	} else {
		exynos_panel_set_backlight_state(ctx, ctx->panel_state);

		/* For the case of OFF->AOD, TE2 will be updated in backlight_update_status */
		if (ctx->panel_state == PANEL_STATE_NORMAL)
			exynos_panel_update_te2(ctx);

		if (bridge->encoder) {
			struct dsim_device *dsim = encoder_to_dsim(bridge->encoder);

			/* Enable error flag detection for the primary dsi */
			if (dsim->irq_err_fg >= 0)
				enable_irq(dsim->irq_err_fg);
		}
	}
	mutex_unlock(&ctx->mode_lock);

	if (need_update_backlight && ctx->bl)
		backlight_update_status(ctx->bl);
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
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (unlikely(!new_crtc_state))
		return 0;

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

	if (self_refresh_active && !exynos_conn_state->blanked_mode) {
		mutex_lock(&ctx->mode_lock);
		dev_dbg(ctx->dev, "self refresh state : %s\n", __func__);

		ctx->self_refresh_active = true;
		panel_update_idle_mode_locked(ctx);
		mutex_unlock(&ctx->mode_lock);
	} else {
		if (bridge->encoder) {
			struct dsim_device *dsim = encoder_to_dsim(bridge->encoder);

			if (dsim->irq_err_fg >= 0)
				disable_irq_nosync(dsim->irq_err_fg);
		}

		if (exynos_conn_state->blanked_mode) {
			/* blanked mode takes precedence over normal modeset */
			ctx->panel_state = PANEL_STATE_BLANK;
		} else if (crtc_state && crtc_state->mode_changed &&
		    drm_atomic_crtc_effectively_active(crtc_state)) {
			if (ctx->desc->delay_dsc_reg_init_us) {
				struct exynos_display_mode *exynos_mode =
							&exynos_conn_state->exynos_mode;

				exynos_mode->dsc.delay_reg_init_us =
							ctx->desc->delay_dsc_reg_init_us;
			}

			ctx->panel_state = PANEL_STATE_MODESET;
		} else if (ctx->force_power_on) {
			/* force blank state instead of power off */
			ctx->panel_state = PANEL_STATE_BLANK;
		} else {
			ctx->panel_state = PANEL_STATE_OFF;
		}

		drm_panel_disable(&ctx->panel);
	}
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
EXPORT_SYMBOL(exynos_panel_wait_for_vblank);

void exynos_panel_wait_for_vsync_done(struct exynos_panel *ctx, u32 te_us, u32 period_us)
{
	u32 delay_us;

	if (unlikely(exynos_panel_wait_for_vblank(ctx))) {
		delay_us = period_us + 1000;
		usleep_range(delay_us, delay_us + 10);
		return;
	}

	delay_us = exynos_panel_vsync_start_time_us(te_us, period_us);
	usleep_range(delay_us, delay_us + 10);
}
EXPORT_SYMBOL(exynos_panel_wait_for_vsync_done);

/* avoid accumulate te varaince cause predicted value is not accurate enough */
#define ACCEPTABLE_TE_PERIOD_DETLA_NS	(3 * NSEC_PER_SEC)
static ktime_t exynos_panel_te_ts_prediction(struct exynos_panel *ctx, ktime_t last_te,
					     s64 since_last_te_us, u32 te_period_us)
{
	const struct exynos_panel_desc *desc = ctx->desc;
	s64 rr_switch_delta_us;
	u32 te_period_before_rr_switch_us;
	u32 rr_switch_applied_duration;
	s64 te_period_delta_ns;

	if (!desc || last_te == 0)
		return 0;

	rr_switch_delta_us = ktime_us_delta(last_te, ctx->last_rr_switch_ts);
	te_period_before_rr_switch_us = ctx->last_rr != 0 ? USEC_PER_SEC / ctx->last_rr : 0;

	if (ctx->last_rr_switch_ts == ctx->last_lp_exit_ts) {
		/* new refresh rate should take effect at first vsync after exiting AOD mode */
		rr_switch_applied_duration = 0;
	} else {
		/* If vrr_switch_duration is not set that mean we don't know the new refresh rate
		 * take effect timing. It may take effect at first or second vsync after sending
		 * rr switch commands.
		 */
		rr_switch_applied_duration = desc->vrr_switch_duration != 0 ?
						(desc->vrr_switch_duration - 1) : 1;
	}

	if (rr_switch_delta_us < 0 && te_period_before_rr_switch_us != 0) {
		ktime_t first_te_after_rr_switch;

		te_period_delta_ns = ((-rr_switch_delta_us / te_period_before_rr_switch_us) + 1) *
					te_period_before_rr_switch_us * NSEC_PER_USEC;

		if (te_period_delta_ns < ACCEPTABLE_TE_PERIOD_DETLA_NS) {
			first_te_after_rr_switch = last_te + te_period_delta_ns;
			rr_switch_delta_us =
				ktime_us_delta(first_te_after_rr_switch, ctx->last_rr_switch_ts);
		}
	}

	if (rr_switch_delta_us > (rr_switch_applied_duration * te_period_before_rr_switch_us)) {
		te_period_delta_ns =
			(since_last_te_us / te_period_us) * te_period_us * NSEC_PER_USEC;

		if (te_period_delta_ns < ACCEPTABLE_TE_PERIOD_DETLA_NS)
			return (last_te + te_period_delta_ns);
	}

	return 0;
}

static void exynos_panel_check_mipi_sync_timing(struct drm_crtc *crtc,
						const struct exynos_panel_mode *current_mode,
						struct exynos_panel *ctx)
{
	u32 te_period_us;
	int retry;
	u64 left, right;
	bool vblank_taken = false;
	bool rr_applied = false;

	if (WARN_ON(!current_mode))
		return;

	DPU_ATRACE_BEGIN("mipi_time_window");
	te_period_us = USEC_PER_SEC / drm_mode_vrefresh(&current_mode->mode);
	pr_debug("%s: check mode_set timing enter. te %d\n", __func__, te_period_us);

	/*
	 * Safe time window to send RR (refresh rate) command illustrated below. RR switch
	 * and scanout need to happen in the same VSYNC period because the frame content might
	 * be adjusted specific to this RR.
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
	 */
	left = exynos_panel_vsync_start_time_us(current_mode->exynos_mode.te_usec, te_period_us);
	right = te_period_us - USEC_PER_MSEC;
	/* check for next TE every 1ms */
	retry = te_period_us / USEC_PER_MSEC + 1;

	do {
		ktime_t last_te = 0, now, predicted_te;
		s64 since_last_te_us;
		s64 rr_switch_delta_us;
		s64 te_period_before_rr_switch_us;

		drm_crtc_vblank_count_and_time(crtc, &last_te);
		now = ktime_get();
		since_last_te_us = ktime_us_delta(now, last_te);
		/* Need a vblank as a reference point */
		predicted_te =
			exynos_panel_te_ts_prediction(ctx, last_te, since_last_te_us, te_period_us);
		if (predicted_te) {
			DPU_ATRACE_BEGIN("predicted_te");
			last_te = predicted_te;
			since_last_te_us = ktime_us_delta(now, last_te);
			rr_applied = true;
			DPU_ATRACE_END("predicted_te");
		} else if (since_last_te_us > te_period_us) {
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

		/**
		 * If a refresh rate switch happens right before last_te. last_te is not reliable
		 * because it could be for the new refresh rate or the old refresh rate.
		 * Wait another vblank in this case.
		 */
		rr_switch_delta_us = ktime_us_delta(last_te, ctx->last_rr_switch_ts);
		te_period_before_rr_switch_us = ctx->last_rr != 0 ? USEC_PER_SEC / ctx->last_rr : 0;
		if (rr_switch_delta_us > 0 && rr_switch_delta_us < te_period_before_rr_switch_us &&
				!rr_applied) {
			DPU_ATRACE_BEGIN("time_window_wait_crtc2");
			if (vblank_taken || !drm_crtc_vblank_get(crtc)) {
				drm_crtc_wait_one_vblank(crtc);
				vblank_taken = true;
			} else {
				pr_warn("%s failed to get vblank for rr wait\n", __func__);
			}
			DPU_ATRACE_END("time_window_wait_crtc2");
			continue;
		}

		if (since_last_te_us <= right) {
			if (since_last_te_us < left) {
				u32 delay_us = left - since_last_te_us;

				DPU_ATRACE_BEGIN("time_window_wait");
				usleep_range(delay_us, delay_us + 100);
				DPU_ATRACE_END("time_window_wait");
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

static bool panel_update_local_hbm_notimeout(struct exynos_panel *ctx, bool enable)
{
	const struct exynos_panel_mode *pmode;
	struct local_hbm *lhbm = &ctx->hbm.local_hbm;

	if (!ctx->desc->exynos_panel_func->set_local_hbm_mode)
		return false;

	if (!is_local_hbm_disabled(ctx) == enable)
		return false;

	pmode = ctx->current_mode;
	if (unlikely(pmode == NULL)) {
		dev_err(ctx->dev, "%s: unknown current mode\n", __func__);
		return false;
	}

	if (enable && !ctx->desc->no_lhbm_rr_constraints) {
		const int vrefresh = drm_mode_vrefresh(&pmode->mode);
		/* only allow to turn on LHBM at peak refresh rate to comply with HW constraint */
		if (ctx->peak_vrefresh && vrefresh != ctx->peak_vrefresh) {
			dev_err(ctx->dev, "unexpected mode `%s` while enabling LHBM, give up\n",
				pmode->mode.name);
			return false;
		}
	}

	if (is_local_hbm_post_enabling_supported(ctx)) {
		if (enable) {
			lhbm->en_cmd_ts = ktime_get();
			kthread_queue_work(&lhbm->worker, &lhbm->post_work);
		} else {
			kthread_cancel_work_sync(&lhbm->post_work);
		}
	}

	DPU_ATRACE_BEGIN(__func__);
	lhbm->state = enable ? (is_local_hbm_post_enabling_supported(ctx) ? LOCAL_HBM_ENABLING :
									    LOCAL_HBM_ENABLED) :
			       LOCAL_HBM_DISABLED;
	ctx->desc->exynos_panel_func->set_local_hbm_mode(ctx, enable);
	sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
	DPU_ATRACE_END(__func__);

	return true;
}

static void panel_update_local_hbm_locked(struct exynos_panel *ctx, bool enable)
{
	if (enable) {
		/* reset timeout timer if re-enabling lhbm */
		if (!is_local_hbm_disabled(ctx)) {
			mod_delayed_work(ctx->hbm.wq, &ctx->hbm.local_hbm.timeout_work,
					 msecs_to_jiffies(ctx->hbm.local_hbm.max_timeout_ms));
			return;
		}

		if (!panel_update_local_hbm_notimeout(ctx, true))
			return;
		queue_delayed_work(ctx->hbm.wq, &ctx->hbm.local_hbm.timeout_work,
				   msecs_to_jiffies(ctx->hbm.local_hbm.max_timeout_ms));
	} else {
		cancel_delayed_work(&ctx->hbm.local_hbm.timeout_work);
		panel_update_local_hbm_notimeout(ctx, false);
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

	dev_dbg(ctx->dev, "changing display mode to %dx%d@%d\n",
		pmode->mode.hdisplay, pmode->mode.vdisplay, drm_mode_vrefresh(&pmode->mode));

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
				if (!is_local_hbm_disabled(ctx) && funcs->set_local_hbm_mode) {
					dev_warn(ctx->dev,
						"LHBM is on when switching to LP mode(%s), turn off LHBM first\n",
						pmode->mode.name);
					panel_update_local_hbm_locked(ctx, false);
				}
				funcs->set_lp_mode(ctx, pmode);
				ctx->panel_state = PANEL_STATE_LP;
				need_update_backlight = true;
			}
			_exynos_panel_set_vddd_voltage(ctx, true);
		} else if (was_lp_mode && !is_lp_mode) {
			_exynos_panel_set_vddd_voltage(ctx, false);
			if (is_active && funcs->set_nolp_mode) {
				funcs->set_nolp_mode(ctx, pmode);
				ctx->panel_state = PANEL_STATE_NORMAL;
				need_update_backlight = true;
				state_changed = true;
				come_out_lp_mode = true;
			}
			ctx->current_binned_lp = NULL;
		} else if (funcs->mode_set) {
			if ((MIPI_CMD_SYNC_REFRESH_RATE & exynos_connector_state->mipi_sync) &&
					is_active && old_mode)
				exynos_panel_check_mipi_sync_timing(crtc, old_mode, ctx);

			if (is_active) {
				if (!is_local_hbm_disabled(ctx) &&
				    !ctx->desc->no_lhbm_rr_constraints)
					dev_warn(ctx->dev,
						"do mode change (`%s`) unexpectedly when LHBM is ON\n",
						pmode->mode.name);

				funcs->mode_set(ctx, pmode);
				state_changed = true;
			} else {
				dev_warn(ctx->dev,
					"don't do mode change (`%s`) when panel isn't in interactive mode\n",
					pmode->mode.name);
			}
		}

		ctx->current_mode = pmode;

		if (state_changed) {
			if (was_lp_mode)
				exynos_panel_set_backlight_state(
					ctx, is_active ? PANEL_STATE_NORMAL : PANEL_STATE_OFF);
			else if (ctx->bl)
				backlight_state_changed(ctx->bl);

			if (!is_lp_mode)
				exynos_panel_update_te2(ctx);
		}
	} else {
		ctx->current_mode = pmode;
	}

	if (old_mode && drm_mode_vrefresh(&pmode->mode) != drm_mode_vrefresh(&old_mode->mode)) {
		ctx->last_rr_switch_ts = ktime_get();
		ctx->last_rr = drm_mode_vrefresh(&old_mode->mode);
		if (come_out_lp_mode)
			ctx->last_lp_exit_ts = ctx->last_rr_switch_ts;
	}

	mutex_unlock(&ctx->mode_lock);

	if (need_update_backlight && ctx->bl)
		backlight_update_status(ctx->bl);

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
	panel_update_local_hbm_notimeout(ctx, false);
	mutex_unlock(&ctx->mode_lock);
	sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
}

static void local_hbm_wait_and_notify_effectiveness(struct exynos_panel *ctx,
						     struct drm_crtc *crtc, u32 frames)
{
	const u32 per_frame_us = get_current_frame_duration_us(ctx);

	if (frames == 0)
		return;

	if (crtc) {
		u32 i;

		for (i = 0; i < frames; i++) {
			drm_crtc_wait_one_vblank(crtc);
			if (ctx->hbm.local_hbm.next_vblank_ts == 0)
				ctx->hbm.local_hbm.next_vblank_ts = ktime_get();
		}
	} else {
		u32 delay_us = ktime_us_delta(ktime_get(), ctx->hbm.local_hbm.en_cmd_ts);
		int remaining_us = (per_frame_us * frames) - delay_us;

		if (remaining_us > 0)
			usleep_range(remaining_us, remaining_us + 10);
	}
	/* wait for 0.8 frame time to ensure finishing LHBM spot scanout */
	usleep_range(per_frame_us * 4 / 5, (per_frame_us * 4 / 5) + 10);
	dev_dbg(ctx->dev, "%s: effectiveness delay(us): %lld(EN), %lld(TE)\n", __func__,
		ktime_us_delta(ktime_get(), ctx->hbm.local_hbm.en_cmd_ts),
		ctx->hbm.local_hbm.next_vblank_ts ?
			ktime_us_delta(ktime_get(), ctx->hbm.local_hbm.next_vblank_ts) :
			0);
	if (ctx->hbm.local_hbm.state == LOCAL_HBM_ENABLING) {
		ctx->hbm.local_hbm.state = LOCAL_HBM_ENABLED;
		sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
	} else {
		dev_warn(ctx->dev, "%s: LHBM state = %d before becoming effective\n", __func__,
			 ctx->hbm.local_hbm.state);
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

	/* TODO: delay time might be inaccurate if refresh rate changes around here */
	local_hbm_wait_and_notify_effectiveness(ctx, crtc, desc->lhbm_effective_delay_frames);

	if (crtc)
		drm_crtc_vblank_put(crtc);
	DPU_ATRACE_END(__func__);
}

static void hbm_data_init(struct exynos_panel *ctx)
{
	ctx->hbm.local_hbm.gamma_para_ready = false;
	ctx->hbm.local_hbm.max_timeout_ms = LOCAL_HBM_MAX_TIMEOUT_MS;
	ctx->hbm.local_hbm.state = LOCAL_HBM_DISABLED;
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

int exynos_panel_common_init(struct mipi_dsi_device *dsi,
				struct exynos_panel *ctx)
{
	static atomic_t panel_index = ATOMIC_INIT(-1);
	struct device *dev = &dsi->dev;
	int ret = 0;
	char name[32];
	const struct exynos_panel_funcs *exynos_panel_func;
	int i;

	dev_dbg(dev, "%s +\n", __func__);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);

	dsi->lanes = ctx->desc->data_lane_cnt;
	dsi->format = MIPI_DSI_FMT_RGB888;

	ret = exynos_panel_parse_dt(ctx);
	if (ret)
		return ret;

	scnprintf(name, sizeof(name), "panel%d-backlight", atomic_inc_return(&panel_index));

	ctx->bl = devm_backlight_device_register(ctx->dev, name, dev,
			ctx, &exynos_backlight_ops, NULL);
	if (IS_ERR(ctx->bl)) {
		dev_err(ctx->dev, "failed to register backlight device\n");
		return PTR_ERR(ctx->bl);
	}
	ctx->bl->props.max_brightness = ctx->desc->max_brightness;
	ctx->bl->props.brightness = ctx->desc->dft_brightness;

	exynos_panel_func = ctx->desc->exynos_panel_func;
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

		if (ctx->peak_vrefresh < vrefresh)
			ctx->peak_vrefresh = vrefresh;
	}

	ctx->panel_idle_enabled = exynos_panel_func && exynos_panel_func->set_self_refresh != NULL;
	INIT_DELAYED_WORK(&ctx->idle_work, panel_idle_work);

	mutex_init(&ctx->mode_lock);
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
	exynos_panel_handoff(ctx);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		goto err_panel;

	dev_info(ctx->dev, "samsung common panel driver has been probed\n");

	return 0;

err_panel:
	drm_panel_remove(&ctx->panel);
	dev_err(ctx->dev, "failed to probe samsung panel driver(%d)\n", ret);

	return ret;
}
EXPORT_SYMBOL(exynos_panel_common_init);

int exynos_panel_probe(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx;

	ctx = devm_kzalloc(&dsi->dev, sizeof(struct exynos_panel), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	return exynos_panel_common_init(dsi, ctx);
}
EXPORT_SYMBOL(exynos_panel_probe);

int exynos_panel_remove(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	drm_bridge_remove(&ctx->bridge);

	sysfs_remove_groups(&ctx->bl->dev.kobj, bl_device_groups);
	sysfs_remove_file(&ctx->bl->dev.kobj, &dev_attr_cabc_mode.attr);
	devm_backlight_device_unregister(ctx->dev, ctx->bl);

	return 0;
}
EXPORT_SYMBOL(exynos_panel_remove);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung common panel driver");
MODULE_LICENSE("GPL");
