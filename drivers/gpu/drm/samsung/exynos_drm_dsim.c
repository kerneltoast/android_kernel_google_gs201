// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_dsi.h
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Authors:
 *	Donghwa Lee <dh09.lee@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt)  "%s: " fmt, __func__

#include <asm/unaligned.h>

#include <drm/drmP.h>
#include <drm/drm_of.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_modes.h>
#include <drm/exynos_display_common.h>

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/component.h>
#include <linux/iommu.h>

#include <video/mipi_display.h>

#if defined(CONFIG_CPU_IDLE)
#include <soc/samsung/exynos-cpupm.h>
#endif

#include <dt-bindings/soc/google/gs101-devfreq.h>
#include <soc/samsung/exynos-devfreq.h>

#include <exynos_drm_crtc.h>
#include <exynos_drm_dsim.h>
#include <exynos_drm_decon.h>

#include <regs-dsim.h>

struct dsim_device *dsim_drvdata[MAX_DSI_CNT];

static char panel_name[64];
module_param_string(panel_name, panel_name, sizeof(panel_name), 0644);
MODULE_PARM_DESC(panel_name, "preferred panel name");

#define dsim_info(dsim, fmt, ...)	\
pr_info("%s[%d]: "fmt, dsim->dev->driver->name, dsim->id, ##__VA_ARGS__)

#define dsim_warn(dsim, fmt, ...)	\
pr_warn("%s[%d]: "fmt, dsim->dev->driver->name, dsim->id, ##__VA_ARGS__)

#define dsim_err(dsim, fmt, ...)	\
pr_err("%s[%d]: "fmt, dsim->dev->driver->name, dsim->id, ##__VA_ARGS__)

#define dsim_debug(dsim, fmt, ...)	\
pr_debug("%s[%d]: "fmt, dsim->dev->driver->name, dsim->id, ##__VA_ARGS__)

#define host_to_dsi(host) container_of(host, struct dsim_device, dsi_host)

#define DSIM_ESCAPE_CLK_20MHZ	20

//#define DSIM_BIST

static inline struct dsim_device *encoder_to_dsim(struct drm_encoder *e)
{
	return container_of(e, struct dsim_device, encoder);
}

static const struct of_device_id dsim_of_match[] = {
	{ .compatible = "samsung,exynos-dsim",
	  .data = NULL },
	{ }
};
MODULE_DEVICE_TABLE(of, dsim_of_match);

static void dsim_dump(struct dsim_device *dsim)
{
	struct dsim_regs regs;

	dsim_info(dsim, "=== DSIM SFR DUMP ===\n");

	regs.regs = dsim->res.regs;
	regs.ss_regs = dsim->res.ss_reg_base;
	regs.phy_regs = dsim->res.phy_regs;
	regs.phy_regs_ex = dsim->res.phy_regs_ex;
	__dsim_dump(dsim->id, &regs);
}

static int dsim_phy_power_on(struct dsim_device *dsim)
{
	int ret;

	dsim_debug(dsim, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return 0;

	ret = phy_power_on(dsim->res.phy);
	if (ret) {
		dsim_err(dsim, "failed to enable dphy(%d)\n", ret);
		return ret;
	}
	if (dsim->res.phy_ex) {
		ret = phy_power_on(dsim->res.phy_ex);
		if (ret) {
			dsim_err(dsim, "failed to enable ext dphy(%d)\n", ret);
			return ret;
		}
	}

	dsim_debug(dsim, "%s -\n", __func__);

	return 0;
}

static int dsim_phy_power_off(struct dsim_device *dsim)
{
	int ret;

	dsim_debug(dsim, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return 0;

	ret = phy_power_off(dsim->res.phy);
	if (ret) {
		dsim_err(dsim, "failed to disable dphy(%d)\n", ret);
		return ret;
	}
	if (dsim->res.phy_ex) {
		ret = phy_power_off(dsim->res.phy_ex);
		if (ret) {
			dsim_err(dsim, "failed to disable ext dphy(%d)\n", ret);
			return ret;
		}
	}

	dsim_debug(dsim, "%s -\n", __func__);

	return 0;
}

void dsim_exit_ulps(struct dsim_device *dsim)
{
	dsim_debug(dsim, "%s +\n", __func__);

	if (dsim->state != DSIM_STATE_ULPS)
		return;

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	dsim_phy_power_on(dsim);

	dsim_reg_init(dsim->id, &dsim->config, &dsim->clk_param, false);
	dsim_reg_exit_ulps_and_start(dsim->id, 0, 0x1F);

	dsim->state = DSIM_STATE_HSCLKEN;
	enable_irq(dsim->irq);

	dsim_debug(dsim, "%s -\n", __func__);
}

static void dsim_enable(struct drm_encoder *encoder)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	const struct decon_device *decon = dsim_get_decon(dsim);

	if (dsim->state == DSIM_STATE_HSCLKEN) {
		dsim_info(dsim, "already enabled(%d)\n", dsim->state);
		return;
	}

	dsim_debug(dsim, "%s +\n", __func__);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	dsim_phy_power_on(dsim);

	dsim_reg_init(dsim->id, &dsim->config, &dsim->clk_param, true);
	dsim_reg_start(dsim->id);

	/* TODO: dsi start: enable irq, sfr configuration */
	dsim->state = DSIM_STATE_HSCLKEN;
	enable_irq(dsim->irq);

#if defined(DSIM_BIST)
	dsim_reg_set_bist(dsim->id, true, DSIM_GRAY_GRADATION);
	dsim_dump(dsim);
#endif

	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_ENABLED, decon->id, dsim);

	dsim_debug(dsim, "%s -\n", __func__);
}

void dsim_enter_ulps(struct dsim_device *dsim)
{
	if (dsim->state != DSIM_STATE_HSCLKEN)
		return;

	dsim_debug(dsim, "%s +\n", __func__);

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	dsim->state = DSIM_STATE_ULPS;
	mutex_unlock(&dsim->cmd_lock);

	disable_irq(dsim->irq);
	dsim_reg_stop_and_enter_ulps(dsim->id, 0, 0x1F);

	dsim_phy_power_off(dsim);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	dsim_debug(dsim, "%s -\n", __func__);
}

static void dsim_disable(struct drm_encoder *encoder)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	const struct decon_device *decon = dsim_get_decon(dsim);

	if (dsim->state == DSIM_STATE_SUSPEND) {
		dsim_info(dsim, "already disabled(%d)\n", dsim->state);
		return;
	}

	dsim_debug(dsim, "%s +\n", __func__);

	/* TODO: 0x1F will be changed */
	dsim_reg_stop(dsim->id, 0x1F);
	disable_irq(dsim->irq);

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	del_timer(&dsim->cmd_timer);
	dsim->state = DSIM_STATE_SUSPEND;
	mutex_unlock(&dsim->cmd_lock);

	dsim_phy_power_off(dsim);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_DISABLED, decon->id, dsim);

	dsim_debug(dsim, "%s -\n", __func__);
}

static void dsim_modes_release(struct dsim_pll_params *pll_params)
{
	if (pll_params->params) {
		unsigned int i;

		for (i = 0; i < pll_params->num_modes; i++)
			kfree(pll_params->params[i]);
		kfree(pll_params->params);
	}
	kfree(pll_params);
}

static const struct dsim_pll_param *
dsim_get_clock_mode(const struct dsim_device *dsim,
		    const struct drm_display_mode *mode)
{
	int i;
	const struct dsim_pll_params *pll_params = dsim->pll_params;

	for (i = 0; i < pll_params->num_modes; i++) {
		const struct dsim_pll_param *p = pll_params->params[i];

		if (!strcmp(mode->name, p->name))
			return p;
	}

	return NULL;
}

static int dsim_set_clock_mode(struct dsim_device *dsim,
			       const struct drm_display_mode *mode)
{
	const struct dsim_pll_param *p = dsim_get_clock_mode(dsim, mode);

	if (!p)
		return -ENOENT;

	dsim->config.dphy_pms.p = p->p;
	dsim->config.dphy_pms.m = p->m;
	dsim->config.dphy_pms.s = p->s;
	dsim->config.dphy_pms.k = p->k;

	dsim->config.dphy_pms.mfr = p->mfr;
	dsim->config.dphy_pms.mrr = p->mrr;
	dsim->config.dphy_pms.sel_pf = p->sel_pf;
	dsim->config.dphy_pms.icp = p->icp;
	dsim->config.dphy_pms.afc_enb = p->afc_enb;
	dsim->config.dphy_pms.extafc = p->extafc;
	dsim->config.dphy_pms.feed_en = p->feed_en;
	dsim->config.dphy_pms.fsel = p->fsel;
	dsim->config.dphy_pms.fout_mask = p->fout_mask;
	dsim->config.dphy_pms.rsel = p->rsel;
	dsim->config.dphy_pms.dither_en = p->dither_en;

	dsim->clk_param.hs_clk = p->pll_freq;
	dsim->clk_param.esc_clk = p->esc_freq;

	dsim_debug(dsim, "found proper pll parameter\n");
	dsim_debug(dsim, "\t%s(p:0x%x,m:0x%x,s:0x%x,k:0x%x)\n", p->name,
		 dsim->config.dphy_pms.p, dsim->config.dphy_pms.m,
		 dsim->config.dphy_pms.s, dsim->config.dphy_pms.k);

	dsim_debug(dsim, "\t%s(hs:%d,esc:%d)\n", p->name, dsim->clk_param.hs_clk,
		 dsim->clk_param.esc_clk);

	dsim->config.cmd_underrun_cnt[0] = p->cmd_underrun_cnt;

	return 0;
}

static int dsim_of_parse_modes(struct device_node *entry,
		struct dsim_pll_param *pll_param)
{
	u32 res[14];
	int cnt;

	memset(pll_param, 0, sizeof(*pll_param));

	of_property_read_string(entry, "mode-name",
			(const char **)&pll_param->name);

	cnt = of_property_count_u32_elems(entry, "pmsk");
	if (cnt != 4 && cnt != 14) {
		pr_err("mode %s has wrong pmsk elements number %d\n",
				pll_param->name, cnt);
		return -EINVAL;
	}

	/* TODO: how dsi dither handle ? */
	of_property_read_u32_array(entry, "pmsk", res, cnt);
	pll_param->dither_en = false;
	pll_param->p = res[0];
	pll_param->m = res[1];
	pll_param->s = res[2];
	pll_param->k = res[3];
	if (cnt == 14) {
		pll_param->mfr = res[4];
		pll_param->mrr = res[5];
		pll_param->sel_pf = res[6];
		pll_param->icp = res[7];
		pll_param->afc_enb = res[8];
		pll_param->extafc = res[9];
		pll_param->feed_en = res[10];
		pll_param->fsel = res[11];
		pll_param->fout_mask = res[12];
		pll_param->rsel = res[13];
		pll_param->dither_en = true;
	}

	of_property_read_u32(entry, "hs-clk", &pll_param->pll_freq);
	of_property_read_u32(entry, "esc-clk", &pll_param->esc_freq);
	of_property_read_u32(entry, "cmd_underrun_cnt",
			&pll_param->cmd_underrun_cnt);

	return 0;
}

static struct dsim_pll_params *dsim_of_get_clock_mode(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct device_node *np, *mode_np, *entry;
	struct dsim_pll_params *pll_params;

	np = of_parse_phandle(dev->of_node, "dsim_mode", 0);
	if (!np) {
		dsim_err(dsim, "could not get dsi modes\n");
		return NULL;
	}

	mode_np = of_get_child_by_name(np, "dsim-modes");
	if (!mode_np) {
		dsim_err(dsim, "%pOF: could not find dsim-modes node\n", np);
		goto getnode_fail;
	}

	pll_params = kzalloc(sizeof(*pll_params), GFP_KERNEL);
	if (!pll_params)
		goto getmode_fail;

	entry = of_get_next_child(mode_np, NULL);
	if (!entry) {
		dsim_err(dsim, "could not find child node of dsim-modes");
		goto getchild_fail;
	}

	pll_params->num_modes = of_get_child_count(mode_np);
	if (pll_params->num_modes == 0) {
		dsim_err(dsim, "%pOF: no modes specified\n", np);
		goto getchild_fail;
	}

	pll_params->params = kzalloc(sizeof(struct dsim_pll_param *) *
				pll_params->num_modes, GFP_KERNEL);
	if (!pll_params->params)
		goto getchild_fail;

	pll_params->num_modes = 0;

	for_each_child_of_node(mode_np, entry) {
		struct dsim_pll_param *pll_param;

		pll_param = kzalloc(sizeof(*pll_param), GFP_KERNEL);
		if (!pll_param)
			goto getpll_fail;

		if (dsim_of_parse_modes(entry, pll_param) < 0) {
			kfree(pll_param);
			continue;
		}

		pll_params->params[pll_params->num_modes] = pll_param;
		pll_params->num_modes++;
	}

	of_node_put(np);
	of_node_put(mode_np);
	of_node_put(entry);

	return pll_params;

getpll_fail:
	of_node_put(entry);
getchild_fail:
	dsim_modes_release(pll_params);
getmode_fail:
	of_node_put(mode_np);
getnode_fail:
	of_node_put(np);
	return NULL;
}

static void dsim_adjust_video_timing(struct dsim_device *dsim,
				     const struct drm_display_mode *mode)
{
	struct videomode vm;
	struct dpu_panel_timing *p_timing = &dsim->config.p_timing;

	drm_display_mode_to_videomode(mode, &vm);

	p_timing->vactive = vm.vactive;
	p_timing->vfp = vm.vfront_porch;
	p_timing->vbp = vm.vback_porch;
	p_timing->vsa = vm.vsync_len;

	p_timing->hactive = vm.hactive;
	p_timing->hfp = vm.hfront_porch;
	p_timing->hbp = vm.hback_porch;
	p_timing->hsa = vm.hsync_len;

	dsim_set_clock_mode(dsim, mode);
}

static void dsim_update_config_for_mode(struct dsim_device *dsim,
					const struct drm_display_mode *mode)
{
	const struct exynos_display_mode *mode_priv =
					drm_mode_to_exynos(mode);

	if (!mode_priv)
		return;

	dsim->config.mode = (mode_priv->mode_flags & MIPI_DSI_MODE_VIDEO) ?
				DSIM_VIDEO_MODE : DSIM_COMMAND_MODE;

	dsim->config.data_lane_cnt = dsim->lanes;
	/* TODO: This hard coded information will be defined in device tree */
	dsim->config.mres_mode = 0;

	dsim->config.dsc.enabled = mode_priv->dsc.enabled;
	if (dsim->config.dsc.enabled) {
		dsim->config.dsc.dsc_count = mode_priv->dsc.dsc_count;
		dsim->config.dsc.slice_count = mode_priv->dsc.slice_count;
		dsim->config.dsc.slice_height = mode_priv->dsc.slice_height;
		dsim->config.dsc.slice_width = DIV_ROUND_UP(
				dsim->config.p_timing.hactive,
				dsim->config.dsc.slice_count);
	}

	dsim_info(dsim, "dsim mode %s dsc is %s [%d %d %d %d]\n",
			dsim->config.mode == DSIM_VIDEO_MODE ? "video" : "cmd",
			dsim->config.dsc.enabled ? "enabled" : "disabled",
			dsim->config.dsc.dsc_count,
			dsim->config.dsc.slice_count,
			dsim->config.dsc.slice_width,
			dsim->config.dsc.slice_height);
}

static void dsim_set_display_mode(struct dsim_device *dsim,
				  const struct drm_display_mode *mode)
{
	dsim_adjust_video_timing(dsim, mode);

	dsim_update_config_for_mode(dsim, mode);
}

static void dsim_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);

	dsim_set_display_mode(dsim, adjusted_mode);
}

static enum drm_mode_status dsim_mode_valid(struct drm_encoder *encoder,
					    const struct drm_display_mode *mode)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);

	if (!dsim_get_clock_mode(dsim, mode))
		return MODE_NOMODE;

	return MODE_OK;
}

static int dsim_atomic_check(struct drm_encoder *encoder,
			     struct drm_crtc_state *crtc_state,
			     struct drm_connector_state *state)
{
	struct drm_display_mode *mode;
	struct dsim_device *dsim = encoder_to_dsim(encoder);

	if (!crtc_state->mode_changed)
		return 0;

	list_for_each_entry(mode, &state->connector->modes, head) {
		if (drm_mode_equal(mode, &crtc_state->adjusted_mode)) {
			crtc_state->adjusted_mode.private = mode->private;
			crtc_state->adjusted_mode.private_flags =
			    mode->private_flags;

			return 0;
		}
	}

	dsim_warn(dsim, "%s: failed to find the display mode\n", __func__);
	return -EINVAL;
}

static const struct drm_encoder_helper_funcs dsim_encoder_helper_funcs = {
	.mode_valid = dsim_mode_valid,
	.mode_set = dsim_mode_set,
	.enable = dsim_enable,
	.disable = dsim_disable,
	.atomic_check = dsim_atomic_check,
};

static const struct drm_encoder_funcs dsim_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int dsim_add_mipi_dsi_device(struct dsim_device *dsim)
{
	struct mipi_dsi_device_info info = { };
	struct device_node *node;
	const char *name;

	dsim_debug(dsim, "preferred panel is %s\n", panel_name);

	for_each_available_child_of_node(dsim->dsi_host.dev->of_node, node) {
		/* panel w/ reg node will be added in mipi_dsi_host_register */
		if (of_find_property(node, "reg", NULL))
			continue;

		if (of_property_read_u32(node, "channel", &info.channel))
			continue;

		name = of_get_property(node, "label", NULL);
		if (name && !strcmp(name, panel_name)) {
			strlcpy(info.type, name, sizeof(info.type));
			info.node = of_node_get(node);
			mipi_dsi_device_register_full(&dsim->dsi_host, &info);

			return 0;
		}
	}

	return -ENODEV;
}

static int dsim_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct drm_device *drm_dev = data;
	int ret = 0;

	dsim_debug(dsim, "%s +\n", __func__);

	drm_encoder_init(drm_dev, encoder, &dsim_encoder_funcs,
			 DRM_MODE_ENCODER_DSI, NULL);
	drm_encoder_helper_add(encoder, &dsim_encoder_helper_funcs);

	encoder->possible_crtcs = exynos_drm_get_possible_crtcs(encoder,
							dsim->output_type);
	if (!encoder->possible_crtcs) {
		dsim_err(dsim, "failed to get possible crtc, ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return -ENOTSUPP;
	}

	/* add the dsi device for the detected panel */
	dsim_add_mipi_dsi_device(dsim);

	ret = mipi_dsi_host_register(&dsim->dsi_host);

	dsim_debug(dsim, "%s -\n", __func__);

	return ret;
}

static void dsim_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dsim_device *dsim = encoder_to_dsim(encoder);

	dsim_debug(dsim, "%s +\n", __func__);
	if (dsim->pll_params)
		dsim_modes_release(dsim->pll_params);

	mipi_dsi_host_unregister(&dsim->dsi_host);
}

static const struct component_ops dsim_component_ops = {
	.bind	= dsim_bind,
	.unbind	= dsim_unbind,
};

static int dsim_parse_dt(struct dsim_device *dsim)
{
	struct device_node *np = dsim->dev->of_node;

	if (!np) {
		dsim_err(dsim, "no device tree information\n");
		return -ENOTSUPP;
	}

	of_property_read_u32(np, "dsim,id", &dsim->id);
	if (dsim->id < 0 || dsim->id >= MAX_DSI_CNT) {
		dsim_err(dsim, "wrong dsim id(%d)\n", dsim->id);
		return -ENODEV;
	}

	dsim->pll_params = dsim_of_get_clock_mode(dsim);

	return 0;
}

static int dsim_remap_regs(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct device_node *np = dev->of_node;
	int i, ret = 0;

	i = of_property_match_string(np, "reg-names", "dsi");
	dsim->res.regs = of_iomap(np, i);
	if (IS_ERR(dsim->res.regs)) {
		dsim_err(dsim, "failed to remap io region\n");
		ret = PTR_ERR(dsim->res.regs);
		goto err;
	}
	dsim_regs_desc_init(dsim->res.regs, "dsi", REGS_DSIM_DSI, dsim->id);

	i = of_property_match_string(np, "reg-names", "dphy");
	dsim->res.phy_regs = of_iomap(np, i);
	if (IS_ERR(dsim->res.phy_regs)) {
		dsim_err(dsim, "failed to remap io region\n");
		ret = PTR_ERR(dsim->res.regs);
		goto err_dsi;
	}
	dsim_regs_desc_init(dsim->res.phy_regs, "dphy", REGS_DSIM_PHY,
			dsim->id);

	i = of_property_match_string(np, "reg-names", "dphy-extra");
	dsim->res.phy_regs_ex = of_iomap(np, i);
	if (IS_ERR(dsim->res.phy_regs_ex))
		dsim_warn(dsim, "failed to remap io region. it's optional\n");
	dsim_regs_desc_init(dsim->res.phy_regs_ex, "dphy-extra",
			REGS_DSIM_PHY_BIAS, dsim->id);

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9-disp_ss");
	i = of_property_match_string(np, "reg-names", "sys");
	dsim->res.ss_reg_base = of_iomap(np, i);
	if (!dsim->res.ss_reg_base) {
		dsim_err(dsim, "failed to map sysreg-disp address.");
		ret = PTR_ERR(dsim->res.ss_reg_base);
		goto err_dphy_ext;
	}
	dsim_regs_desc_init(dsim->res.ss_reg_base, np->name, REGS_DSIM_SYS,
			dsim->id);

	return ret;

err_dphy_ext:
	iounmap(dsim->res.phy_regs_ex);
	iounmap(dsim->res.phy_regs);
err_dsi:
	iounmap(dsim->res.regs);
err:
	return ret;
}

static void dsim_underrun_info(struct dsim_device *dsim)
{
	dsim_info(dsim, "underrun irq occurs: MIF(%lu), INT(%lu), DISP(%lu)\n",
			exynos_devfreq_get_domain_freq(DEVFREQ_MIF),
			exynos_devfreq_get_domain_freq(DEVFREQ_INT),
			exynos_devfreq_get_domain_freq(DEVFREQ_DISP));
}

static irqreturn_t dsim_irq_handler(int irq, void *dev_id)
{
	struct dsim_device *dsim = dev_id;
	struct drm_crtc *crtc = dsim->encoder.crtc;
	const struct decon_device *decon = dsim_get_decon(dsim);
	unsigned int int_src;

	spin_lock(&dsim->slock);

	dsim_debug(dsim, "%s +\n", __func__);

	if (dsim->state != DSIM_STATE_HSCLKEN) {
		dsim_info(dsim, "dsim power is off state(0x%x)\n", dsim->state);
		spin_unlock(&dsim->slock);
		return IRQ_HANDLED;
	}

	int_src = dsim_reg_get_int_and_clear(dsim->id);
	if (int_src & DSIM_INTSRC_SFR_PH_FIFO_EMPTY) {
		del_timer(&dsim->cmd_timer);
		complete(&dsim->ph_wr_comp);
		dsim_debug(dsim, "PH_FIFO_EMPTY irq occurs\n");
	}
	if (int_src & DSIM_INTSRC_RX_DATA_DONE)
		complete(&dsim->rd_comp);
	if (int_src & DSIM_INTSRC_FRAME_DONE) {
		dsim_debug(dsim, "framedone irq occurs\n");
		if (decon)
			DPU_EVENT_LOG(DPU_EVT_DSIM_FRAMEDONE, decon->id, NULL);
	}
	if (int_src & DSIM_INTSRC_ERR_RX_ECC)
		dsim_err(dsim, "RX ECC Multibit error was detected!\n");

	if (int_src & DSIM_INTSRC_UNDER_RUN) {
		dsim_underrun_info(dsim);
		if (decon)
			DPU_EVENT_LOG(DPU_EVT_DSIM_UNDERRUN, decon->id, dsim);
	}

	if (int_src & DSIM_INTSRC_VT_STATUS) {
		dsim_debug(dsim, "vt_status irq occurs\n");
		if ((dsim->config.mode == DSIM_VIDEO_MODE) && crtc)
			drm_crtc_handle_vblank(crtc);
	}

	spin_unlock(&dsim->slock);

	return IRQ_HANDLED;
}

static int dsim_register_irq(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev;
	int ret = 0;

	pdev = container_of(dev, struct platform_device, dev);

	dsim->irq = of_irq_get_byname(np, "dsim");
	ret = devm_request_irq(dsim->dev, dsim->irq, dsim_irq_handler, 0,
			pdev->name, dsim);
	if (ret) {
		dsim_err(dsim, "failed to install DSIM irq\n");
		return -EINVAL;
	}
	disable_irq(dsim->irq);

	return 0;
}

static int dsim_get_phys(struct dsim_device *dsim)
{
	dsim->res.phy = devm_phy_get(dsim->dev, "dsim_dphy");
	if (IS_ERR(dsim->res.phy)) {
		dsim_err(dsim, "failed to get dsim phy\n");
		return PTR_ERR(dsim->res.phy);
	}

	dsim->res.phy_ex = devm_phy_get(dsim->dev, "dsim_dphy_extra");
	if (IS_ERR(dsim->res.phy_ex)) {
		dsim_warn(dsim, "failed to get dsim extra phy\n");
		dsim->res.phy_ex = NULL;
	}

	return 0;
}

static int dsim_init_resources(struct dsim_device *dsim)
{
	int ret = 0;

	ret = dsim_remap_regs(dsim);
	if (ret)
		goto err;

	ret = dsim_register_irq(dsim);
	if (ret)
		goto err;

	ret = dsim_get_phys(dsim);
	if (ret)
		goto err;

err:
	return ret;
}

static int dsim_host_attach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct dsim_device *dsim = host_to_dsi(host);
	struct drm_bridge *bridge;
	int ret;

	dsim_debug(dsim, "%s +\n", __func__);

	dsim->lanes = device->lanes;
	dsim->format = device->format;
	dsim->mode_flags = device->mode_flags;

	bridge = of_drm_find_bridge(device->dev.of_node);
	if (!bridge) {
		struct drm_panel *panel;

		panel = of_drm_find_panel(device->dev.of_node);
		if (IS_ERR(panel)) {
			dsim_err(dsim, "failed to find panel\n");
			return PTR_ERR(panel);
		}

		bridge = devm_drm_panel_bridge_add(host->dev, panel,
						   DRM_MODE_CONNECTOR_DSI);
		if (IS_ERR(bridge)) {
			dsim_err(dsim, "failed to create panel bridge\n");
			return PTR_ERR(bridge);
		}
	}

	ret = drm_bridge_attach(&dsim->encoder, bridge, dsim->encoder.bridge);
	if (ret)
		dsim_err(dsim, "Unable to attach panel bridge\n");
	else
		dsim->panel_bridge = bridge;

	dsim_debug(dsim, "%s -\n", __func__);

	return ret;
}

static int dsim_host_detach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct dsim_device *dsim = host_to_dsi(host);

	dsim_info(dsim, "%s +\n", __func__);

	dsim_disable(&dsim->encoder);
	if (dsim->panel_bridge) {
		struct drm_bridge *bridge = dsim->panel_bridge;

		if (bridge->funcs && bridge->funcs->detach)
			bridge->funcs->detach(bridge);
		dsim->panel_bridge = NULL;
	}

	dsim_info(dsim, "%s -\n", __func__);
	return 0;
}

static void dsim_cmd_fail_detector(struct timer_list *arg)
{
	struct dsim_device *dsim = from_timer(dsim, arg, cmd_timer);

	dsim_debug(dsim, "%s +\n", __func__);

	if (dsim->state != DSIM_STATE_HSCLKEN) {
		dsim_err(dsim, "%s: DSIM is not ready. state(%d)\n", __func__,
				dsim->state);
		goto exit;
	}

	/* If already FIFO empty even though the timer is no pending */
	if (!timer_pending(&dsim->cmd_timer)
			&& dsim_reg_header_fifo_is_empty(dsim->id)) {
		reinit_completion(&dsim->ph_wr_comp);
		dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
		goto exit;
	}

exit:
	dsim_debug(dsim, "%s -\n", __func__);
}

static int dsim_wait_for_cmd_fifo_empty(struct dsim_device *dsim,
		bool must_wait)
{
	int ret = 0;

	if (!must_wait) {
		/* timer is running, but already command is transferred */
		if (dsim_reg_header_fifo_is_empty(dsim->id))
			del_timer(&dsim->cmd_timer);

		dsim_debug(dsim, "Doesn't need to wait fifo_completion\n");
		return ret;
	}

	del_timer(&dsim->cmd_timer);
	dsim_debug(dsim, "Waiting for fifo_completion...\n");

	if (!wait_for_completion_timeout(&dsim->ph_wr_comp, MIPI_WR_TIMEOUT)) {
		if (dsim_reg_header_fifo_is_empty(dsim->id)) {
			reinit_completion(&dsim->ph_wr_comp);
			dsim_reg_clear_int(dsim->id,
					DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
			return 0;
		}
		ret = -ETIMEDOUT;
	}

	if ((dsim->state == DSIM_STATE_HSCLKEN) && (ret == -ETIMEDOUT))
		dsim_err(dsim, "have timed out\n");

	return ret;
}

static void dsim_long_data_wr(struct dsim_device *dsim, unsigned long d0,
		u32 d1)
{
	unsigned int data_cnt = 0, payload = 0;

	/* in case that data count is more then 4 */
	for (data_cnt = 0; data_cnt < d1; data_cnt += 4) {
		/*
		 * after sending 4bytes per one time,
		 * send remainder data less then 4.
		 */
		if ((d1 - data_cnt) < 4) {
			if ((d1 - data_cnt) == 3) {
				payload = *(u8 *)(d0 + data_cnt) |
				    (*(u8 *)(d0 + (data_cnt + 1))) << 8 |
					(*(u8 *)(d0 + (data_cnt + 2))) << 16;
			dsim_debug(dsim, "count = 3 payload = %x, %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)));
			} else if ((d1 - data_cnt) == 2) {
				payload = *(u8 *)(d0 + data_cnt) |
					(*(u8 *)(d0 + (data_cnt + 1))) << 8;
			dsim_debug(dsim, "count = 2 payload = %x, %x %x\n",
				payload,
				*(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)));
			} else if ((d1 - data_cnt) == 1) {
				payload = *(u8 *)(d0 + data_cnt);
			}

			dsim_reg_wr_tx_payload(dsim->id, payload);
		/* send 4bytes per one time. */
		} else {
			payload = *(u8 *)(d0 + data_cnt) |
				(*(u8 *)(d0 + (data_cnt + 1))) << 8 |
				(*(u8 *)(d0 + (data_cnt + 2))) << 16 |
				(*(u8 *)(d0 + (data_cnt + 3))) << 24;

			dsim_debug(dsim, "count = 4 payload = %x, %x %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)),
				*(u8 *)(d0 + (data_cnt + 3)));

			dsim_reg_wr_tx_payload(dsim->id, payload);
		}
	}
}

static bool dsim_fifo_empty_needed(struct dsim_device *dsim,
		unsigned int data_id, unsigned long data0)
{
	/* read case or partial update command */
	if (data_id == MIPI_DSI_DCS_READ
			|| data0 == MIPI_DCS_SET_COLUMN_ADDRESS
			|| data0 == MIPI_DCS_SET_PAGE_ADDRESS) {
		dsim_debug(dsim, "id:%d, data=%ld\n", data_id, data0);
		return true;
	}

	/* Check a FIFO level whether writable or not */
	if (!dsim_reg_is_writable_fifo_state(dsim->id))
		return true;

	return false;
}

static int dsim_write_data(struct dsim_device *dsim, u32 id, unsigned long d0,
		u32 d1)
{
	int ret = 0;
	bool must_wait = true;
	const struct decon_device *decon = dsim_get_decon(dsim);

	mutex_lock(&dsim->cmd_lock);
	if (dsim->state != DSIM_STATE_HSCLKEN) {
		dsim_err(dsim, "Not ready(%d)\n", dsim->state);
		ret = -EINVAL;
		goto err_exit;
	}

	if (decon)
		DPU_EVENT_LOG_CMD(decon->id, dsim, id, d0);

	reinit_completion(&dsim->ph_wr_comp);
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);

	/* Run write-fail dectector */
	mod_timer(&dsim->cmd_timer, jiffies + MIPI_WR_TIMEOUT);

	switch (id) {
	/* short packet types of packet types for command. */
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
	case MIPI_DSI_DCS_COMPRESSION_MODE:
	case MIPI_DSI_COLOR_MODE_OFF:
	case MIPI_DSI_COLOR_MODE_ON:
	case MIPI_DSI_SHUTDOWN_PERIPHERAL:
	case MIPI_DSI_TURN_ON_PERIPHERAL:
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, false);
		must_wait = dsim_fifo_empty_needed(dsim, id, d0);
		break;

	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
	case MIPI_DSI_DCS_READ:
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, true);
		must_wait = dsim_fifo_empty_needed(dsim, id, d0);
		break;

	/* long packet types of packet types for command. */
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_PPS_LONG_WRITE:
		dsim_long_data_wr(dsim, d0, d1);
		dsim_reg_wr_tx_header(dsim->id, id, d1 & 0xff,
				(d1 & 0xff00) >> 8, false);
		must_wait = dsim_fifo_empty_needed(dsim, id, *(u8 *)d0);
		break;

	default:
		dsim_info(dsim, "data id %x is not supported.\n", id);
		ret = -EINVAL;
	}

	ret = dsim_wait_for_cmd_fifo_empty(dsim, must_wait);
	if (ret < 0)
		dsim_err(dsim, "ID(%d): DSIM cmd wr timeout 0x%lx\n", id, d0);

err_exit:
	mutex_unlock(&dsim->cmd_lock);

	return ret;
}

static int dsim_wr_data(struct dsim_device *dsim, u32 type, const u8 data[],
		u32 len)
{
	u32 t;
	int ret = 0;

	switch (len) {
	case 0:
		return -EINVAL;
	case 1:
		t = type ? type : MIPI_DSI_DCS_SHORT_WRITE;
		ret = dsim_write_data(dsim, t, (unsigned long)data[0], 0);
		break;
	case 2:
		t = type ? type : MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		ret = dsim_write_data(dsim, t, (unsigned long)data[0],
				(u32)data[1]);
		break;
	default:
		t = type ? type : MIPI_DSI_DCS_LONG_WRITE;
		ret = dsim_write_data(dsim, t, (unsigned long)data, len);
		break;
	}

	return ret;
}

#define DSIM_RX_PHK_HEADER_SIZE	4
static int dsim_read_data(struct dsim_device *dsim, u32 id, u32 addr, u32 cnt,
		u8 *buf)
{
	u32 rx_fifo, rx_size = 0;
	int i = 0, ret = 0;

	if (dsim->state != DSIM_STATE_HSCLKEN) {
		dsim_err(dsim, "Not ready(%d)\n", dsim->state);
		return -EINVAL;
	}

	if (cnt > DSIM_RX_FIFO_MAX_DEPTH * 4 - DSIM_RX_PHK_HEADER_SIZE) {
		dsim_err(dsim, "requested rx size is wrong(%d)\n", cnt);
		return -EINVAL;
	}

	dsim_debug(dsim, "type[0x%x], cmd[0x%x], rx cnt[%d]\n", id, addr, cnt);

	/* Init RX FIFO before read and clear DSIM_INTSRC */
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_RX_DATA_DONE);

	reinit_completion(&dsim->rd_comp);

	/* Set the maximum packet size returned */
	dsim_write_data(dsim,
		MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, cnt, 0);

	/* Read request */
	if (id == MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM)
		dsim_write_data(dsim, id, addr & 0xff, (addr >> 8) & 0xff);
	else
		dsim_write_data(dsim, id, addr, 0);

	if (!wait_for_completion_timeout(&dsim->rd_comp, MIPI_RD_TIMEOUT)) {
		dsim_err(dsim, "read timeout\n");
		return -ETIMEDOUT;
	}

	mutex_lock(&dsim->cmd_lock);

	rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
	dsim_debug(dsim, "rx fifo:0x%8x, response:0x%x, rx_size:%d\n", rx_fifo,
		 rx_fifo & 0xff, rx_size);

	/* Parse the RX packet data types */
	switch (rx_fifo & 0xff) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		ret = dsim_reg_rx_err_handler(dsim->id, rx_fifo);
		if (ret < 0) {
			dsim_dump(dsim);
			goto exit;
		}
		break;
	case MIPI_DSI_RX_END_OF_TRANSMISSION:
		dsim_debug(dsim, "EoTp was received\n");
		break;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		buf[1] = (rx_fifo >> 16) & 0xff;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		buf[0] = (rx_fifo >> 8) & 0xff;
		dsim_debug(dsim, "short packet was received\n");
		rx_size = cnt;
		break;
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
		dsim_debug(dsim, "long packet was received\n");
		rx_size = (rx_fifo & 0x00ffff00) >> 8;

		while (i < rx_size) {
			const u32 rx_max =
				min_t(u32, rx_size, i + sizeof(rx_fifo));

			rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
			dsim_debug(dsim, "payload: 0x%x i=%d max=%d\n", rx_fifo,
					i, rx_max);
			for (; i < rx_max; i++, rx_fifo >>= 8)
				buf[i] = rx_fifo & 0xff;
		}
		break;
	default:
		dsim_err(dsim, "packet format is invaild.\n");
		dsim_dump(dsim);
		ret = -EBUSY;
		goto exit;
	}

	if (!dsim_reg_rx_fifo_is_empty(dsim->id)) {
		dsim_err(dsim, "RX FIFO is not empty\n");
		dsim_dump(dsim);
		ret = -EBUSY;
	} else  {
		ret = rx_size;
	}
exit:
	mutex_unlock(&dsim->cmd_lock);

	return ret;
}

static int dsim_rd_data(struct dsim_device *dsim, u32 type, const u8 tx_data[],
		u8 rx_data[], u32 rx_len)
{
	u32 cmd = 0;

	switch (type) {
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		cmd = tx_data[1] << 8;
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
		cmd |= tx_data[0];
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
		break;
	default:
		dsim_err(dsim, "Invalid rx type (%d)\n", type);
	}
	return dsim_read_data(dsim, type, cmd, rx_len, rx_data);
}

static ssize_t dsim_host_transfer(struct mipi_dsi_host *host,
			    const struct mipi_dsi_msg *msg)
{
	struct dsim_device *dsim = host_to_dsi(host);
	const struct decon_device *decon = dsim_get_decon(dsim);
	int ret;

	hibernation_block_exit(decon->hibernation);

	switch (msg->type) {
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		ret = dsim_rd_data(dsim, msg->type, msg->tx_buf,
				   msg->rx_buf, msg->rx_len);
		break;
	default:
		ret = dsim_wr_data(dsim, msg->type, msg->tx_buf, msg->tx_len);
		break;
	}

	hibernation_unblock(decon->hibernation);

	return ret;
}

/* TODO: Below operation will be registered after panel driver is created. */
static const struct mipi_dsi_host_ops dsim_host_ops = {
	.attach = dsim_host_attach,
	.detach = dsim_host_detach,
	.transfer = dsim_host_transfer,
};

static ssize_t bist_mode_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{

	struct dsim_device *dsim = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", dsim->bist_mode);
}

static ssize_t bist_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int rc;
	unsigned int bist_mode;
	bool bist_en;

	rc = kstrtouint(buf, 0, &bist_mode);
	if (rc < 0)
		return rc;

	/*
	 * BIST modes:
	 * 0: Disable, 1: Color Bar, 2: GRAY Gradient, 3: User-Defined,
	 * 4: Prbs7 Random
	 */
	if (bist_mode > DSIM_BIST_MODE_MAX) {
		dsim_err(dsim, "invalid bist mode\n");
		return -EINVAL;
	}

	bist_en = bist_mode > 0;

	if (bist_en && dsim->state == DSIM_STATE_SUSPEND)
		dsim_enable(&dsim->encoder);

	dsim_reg_set_bist(dsim->id, bist_en, bist_mode - 1);
	dsim->bist_mode = bist_mode;

	if (!bist_en && dsim->state == DSIM_STATE_HSCLKEN)
		dsim_disable(&dsim->encoder);

	dsim_info(dsim, "0:Disable 1:ColorBar 2:GRAY Gradient 3:UserDefined\n");
	dsim_info(dsim, "4:Prbs7 Random (%d)\n", dsim->bist_mode);

	return len;
}
static DEVICE_ATTR_RW(bist_mode);

static int dsim_probe(struct platform_device *pdev)
{
	struct dsim_device *dsim;
	int ret;

	dsim_info(dsim, "%s +\n", __func__);

	dsim = devm_kzalloc(&pdev->dev, sizeof(*dsim), GFP_KERNEL);
	if (!dsim)
		return -ENOMEM;

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(36));

	dsim->dsi_host.ops = &dsim_host_ops;
	dsim->dsi_host.dev = &pdev->dev;
	dsim->dev = &pdev->dev;

	ret = dsim_parse_dt(dsim);
	if (ret)
		goto err;

	dsim_drvdata[dsim->id] = dsim;

	dsim->output_type = (dsim->id == 0) ?
			EXYNOS_DISPLAY_TYPE_DSI0 : EXYNOS_DISPLAY_TYPE_DSI1;

	spin_lock_init(&dsim->slock);
	mutex_init(&dsim->cmd_lock);
	init_completion(&dsim->ph_wr_comp);
	init_completion(&dsim->rd_comp);

	ret = dsim_init_resources(dsim);
	if (ret)
		goto err;

	ret = device_create_file(dsim->dev, &dev_attr_bist_mode);
	if (ret < 0)
		dsim_err(dsim, "failed to add sysfs entries\n");

	platform_set_drvdata(pdev, &dsim->encoder);

	timer_setup(&dsim->cmd_timer, dsim_cmd_fail_detector, 0);

#if defined(CONFIG_CPU_IDLE)
	dsim->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
	dsim_info(dsim, "dsim idle_ip_index[%d]\n", dsim->idle_ip_index);
	if (dsim->idle_ip_index < 0)
		dsim_warn(dsim, "idle ip index is not provided\n");
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	dsim->state = DSIM_STATE_SUSPEND;
	pm_runtime_enable(dsim->dev);

	if (!IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		phy_init(dsim->res.phy);
		if (dsim->res.phy_ex)
			phy_init(dsim->res.phy_ex);
	}

	dsim_info(dsim, "driver has been probed.\n");
	return component_add(dsim->dev, &dsim_component_ops);

err:
	dsim_err(dsim, "failed to probe exynos dsim driver\n");
	return ret;
}

static int dsim_remove(struct platform_device *pdev)
{
	struct dsim_device *dsim = platform_get_drvdata(pdev);

	device_remove_file(dsim->dev, &dev_attr_bist_mode);
	pm_runtime_disable(&pdev->dev);

	component_del(&pdev->dev, &dsim_component_ops);

	iounmap(dsim->res.ss_reg_base);
	iounmap(dsim->res.phy_regs_ex);
	iounmap(dsim->res.phy_regs);
	iounmap(dsim->res.regs);

	return 0;
}

struct platform_driver dsim_driver = {
	.probe = dsim_probe,
	.remove = dsim_remove,
	.driver = {
		   .name = "exynos-dsim",
		   .owner = THIS_MODULE,
		   .of_match_table = dsim_of_match,
	},
};

MODULE_SOFTDEP("pre: phy-exynos-mipi");
MODULE_AUTHOR("Donghwa Lee <dh09.lee@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC MIPI DSI Master");
MODULE_LICENSE("GPL v2");
