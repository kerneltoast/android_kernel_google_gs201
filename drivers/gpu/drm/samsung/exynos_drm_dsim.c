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

#include <asm/unaligned.h>

#include <drm/drmP.h>
#include <drm/drm_of.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_atomic_helper.h>

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/component.h>
#include <linux/iommu.h>
#include <linux/exynos_iovmm.h>

#include <video/mipi_display.h>

#if defined(CONFIG_CPU_IDLE)
#include <soc/samsung/exynos-cpupm.h>
#endif

#include "exynos_drm_crtc.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_decon.h"

#include "cal_9820/regs-dsim.h"

struct dsim_device *dsim_drvdata[MAX_DSI_CNT];

int dsim_log_level = 7;

#define dsim_info(dsim, fmt, ...)					  \
	do {								  \
		if (dsim_log_level >= 6) {				  \
			DRM_INFO("%s[%d]: "fmt, dsim->dev->driver->name,  \
					dsim->id, ##__VA_ARGS__);	  \
		}							  \
	} while (0)

#define dsim_warn(dsim, fmt, ...)					  \
	do {								  \
		if (dsim_log_level >= 4) {				  \
			DRM_WARN("%s[%d]: "fmt, dsim->dev->driver->name,  \
					dsim->id, ##__VA_ARGS__);	  \
		}							  \
	} while (0)

#define dsim_err(dsim, fmt, ...)					  \
	do {								  \
		if (dsim_log_level >= 3) {				  \
			DRM_ERROR("%s[%d]: "fmt, dsim->dev->driver->name, \
					dsim->id, ##__VA_ARGS__);	  \
		}							  \
	} while (0)

#define dsim_dbg(dsim, fmt, ...)					  \
	do {								  \
		if (dsim_log_level >= 7) {				  \
			DRM_INFO("%s[%d]: "fmt, dsim->dev->driver->name,  \
					dsim->id, ##__VA_ARGS__);	  \
		}							  \
	} while (0)

#define host_to_dsi(host) container_of(host, struct dsim_device, dsi_host)
#define connector_to_dsi(c) container_of(c, struct dsim_device, connector)

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

/*
 * TODO: Currently, this function is only supported 'on' case, but
 * 'off' case will also be implemented in the future.
 */
int dsim_set_panel_power(int id, bool on)
{
	struct dsim_device *dsim = dsim_drvdata[id];
	int ret;

	ret = drm_panel_prepare(dsim->panel);
	if (ret)
		dsim_err(dsim, "failed to prepare dsim%d panel\n", dsim->id);

	return ret;
}

#if defined(DSIM_BIST)
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
#endif

static void dsim_enable(struct drm_encoder *encoder)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct decon_device *decon =
		(struct decon_device *)to_exynos_crtc(encoder->crtc)->ctx;
	int ret;

	if (dsim->state == DSIM_STATE_HSCLKEN) {
		dsim_info(dsim, "dsim%d already enabled(%d)\n",
				dsim->id, dsim->state);
		return;
	}

	dsim_dbg(dsim, "%s +\n", __func__);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	pm_runtime_get_sync(dsim->dev);

	dsim_reg_init(dsim->id, &dsim->config, &dsim->clk_param, true);
	dsim_reg_start(dsim->id);

	/* TODO: dsi start: enable irq, sfr configuration */
	dsim->state = DSIM_STATE_HSCLKEN;
	enable_irq(dsim->irq);

	/* panel enable */
	ret = drm_panel_enable(dsim->panel);
	if (ret < 0) {
		drm_panel_unprepare(dsim->panel);
		pm_runtime_put_sync(dsim->dev);
		dsim_err(dsim, "drm_panel_enable is failed(%d)\n", ret);
		return;
	}

#if defined(DSIM_BIST)
	dsim_reg_set_bist(dsim->id, true);
	dsim_dump(dsim);
#endif

	DPU_EVENT_LOG(DPU_EVT_DSIM_ENABLED, decon->id, dsim);
	dsim_dbg(dsim, "%s -\n", __func__);
}

static void dsim_disable(struct drm_encoder *encoder)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct decon_device *decon =
		(struct decon_device *)to_exynos_crtc(encoder->crtc)->ctx;
	int ret;

	if (dsim->state == DSIM_STATE_SUSPEND) {
		dsim_info(dsim, "dsim%d already disabled(%d)\n",
				dsim->id, dsim->state);
		return;
	}

	dsim_dbg(dsim, "%s +\n", __func__);

	ret = drm_panel_disable(dsim->panel);
	if (ret < 0) {
		dsim_err(dsim, "drm_panel_disable is failed(%d)\n", ret);
		return;
	}

	/* TODO: 0x1F will be changed */
	dsim_reg_stop(dsim->id, 0x1F);
	disable_irq(dsim->irq);

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	del_timer(&dsim->cmd_timer);
	dsim->state = DSIM_STATE_SUSPEND;
	mutex_unlock(&dsim->cmd_lock);

	ret = drm_panel_unprepare(dsim->panel);
	if (ret < 0) {
		dsim_err(dsim, "drm_panel_unprepare is failed(%d)\n", ret);
		return;
	}

	pm_runtime_put_sync(dsim->dev);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	DPU_EVENT_LOG(DPU_EVT_DSIM_DISABLED, decon->id, dsim);
	dsim_dbg(dsim, "%s -\n", __func__);
}

static enum drm_connector_status
dsim_detect(struct drm_connector *connector, bool force)
{
	return connector->status;
}

static void dsim_connector_destroy(struct drm_connector *connector)
{
}

static const struct drm_connector_funcs dsim_connector_funcs = {
	.detect = dsim_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = dsim_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int dsim_get_modes(struct drm_connector *connector)
{
	struct dsim_device *dsim = connector_to_dsi(connector);
	int ret = 0;

	ret = drm_panel_get_modes(dsim->panel);
	if (ret < 0) {
		dsim_err(dsim, "failed to get panel display modes\n");
		return ret;
	}

	return ret;
}

static const struct drm_connector_helper_funcs dsim_connector_helper_funcs = {
	.get_modes = dsim_get_modes,
};

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

static void dsim_get_clock_modes(struct dsim_device *dsim, char *name)
{
	int i;
	struct dsim_pll_params *pll_params = dsim->pll_params;

	for (i = 0; i < pll_params->num_modes; i++) {
		if (!strcmp(name, pll_params->params[i]->name)) {
			dsim->config.dphy_pms.p = pll_params->params[i]->p;
			dsim->config.dphy_pms.m = pll_params->params[i]->m;
			dsim->config.dphy_pms.s = pll_params->params[i]->s;
			dsim->config.dphy_pms.k = pll_params->params[i]->k;

			dsim->config.dphy_pms.mfr = pll_params->params[i]->mfr;
			dsim->config.dphy_pms.mrr = pll_params->params[i]->mrr;
			dsim->config.dphy_pms.sel_pf =
				pll_params->params[i]->sel_pf;
			dsim->config.dphy_pms.icp = pll_params->params[i]->icp;
			dsim->config.dphy_pms.afc_enb =
				pll_params->params[i]->afc_enb;
			dsim->config.dphy_pms.extafc =
				pll_params->params[i]->extafc;
			dsim->config.dphy_pms.feed_en =
				pll_params->params[i]->feed_en;
			dsim->config.dphy_pms.fsel =
				pll_params->params[i]->fsel;
			dsim->config.dphy_pms.fout_mask =
				pll_params->params[i]->fout_mask;
			dsim->config.dphy_pms.rsel =
				pll_params->params[i]->rsel;
			dsim->config.dphy_pms.dither_en =
				pll_params->params[i]->dither_en;

			dsim->clk_param.hs_clk =
				pll_params->params[i]->pll_freq;
			dsim->clk_param.esc_clk =
				pll_params->params[i]->esc_freq;
			dsim_dbg(dsim, "found proper pll parameter\n");
			dsim_dbg(dsim, "\t%s(p:0x%x,m:0x%x,s:0x%x,k:0x%x)\n",
					pll_params->params[i]->name,
					dsim->config.dphy_pms.p,
					dsim->config.dphy_pms.m,
					dsim->config.dphy_pms.s,
					dsim->config.dphy_pms.k);

			dsim_dbg(dsim, "\t%s(hs:%d,esc:%d)\n",
					pll_params->params[i]->name,
					dsim->clk_param.hs_clk,
					dsim->clk_param.esc_clk);


			dsim->config.cmd_underrun_cnt[i] =
				pll_params->params[i]->cmd_underrun_cnt;
			break;
		}
	}
}

static void dsim_of_parse_modes(struct device_node *entry,
		struct dsim_pll_param *pll_param)
{
	u32 res[14];
	int cnt;

	memset(pll_param, 0, sizeof(*pll_param));

	of_property_read_string(entry, "mode-name",
			(const char **)&pll_param->name);

	cnt = of_property_count_u32_elems(entry, "pmsk");

	/* TODO: how dsi dither handle ? */
	of_property_read_u32_array(entry, "pmsk", res, 14);
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

		dsim_of_parse_modes(entry, pll_param);
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
		struct videomode videomode, char *name)
{
	struct dpu_panel_timing *p_timing = &dsim->config.p_timing;

	p_timing->vactive = videomode.vactive;
	p_timing->vfp = videomode.vfront_porch;
	p_timing->vbp = videomode.vback_porch;
	p_timing->vsa = videomode.vsync_len;

	p_timing->hactive = videomode.hactive;
	p_timing->hfp = videomode.hfront_porch;
	p_timing->hbp = videomode.hback_porch;
	p_timing->hsa = videomode.hsync_len;

	dsim_get_clock_modes(dsim, name);
}

static void dsim_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct videomode vm = dsim->vm;

	drm_display_mode_to_videomode(adjusted_mode, &vm);

	dsim_adjust_video_timing(dsim, vm, adjusted_mode->name);
}

static const struct drm_encoder_helper_funcs dsim_encoder_helper_funcs = {
	.mode_set = dsim_mode_set,
	.enable = dsim_enable,
	.disable = dsim_disable,
};

static const struct drm_encoder_funcs dsim_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int dsim_create_connector(struct drm_encoder *encoder)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct drm_connector *connector = &dsim->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(encoder->dev, connector,
				 &dsim_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		dsim_err(dsim, "Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &dsim_connector_helper_funcs);
	drm_connector_register(connector);
	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static int dsim_bind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct drm_device *drm_dev = data;
	int ret = 0;

	dsim_dbg(dsim, "%s +\n", __func__);

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

	ret = dsim_create_connector(encoder);
	if (ret) {
		dsim_err(dsim, "failed to create connector ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	ret = mipi_dsi_host_register(&dsim->dsi_host);
	dsim_dbg(dsim, "%s -\n", __func__);

	return ret;
}

static void dsim_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dsim_device *dsim = encoder_to_dsim(encoder);

	dsim_dbg(dsim, "%s +\n", __func__);
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
	struct device_node *dsc_np;

	if (!np) {
		dsim_err(dsim, "no device tree information\n");
		return -ENOTSUPP;
	}

	of_property_read_u32(np, "dsim,id", &dsim->id);
	if (dsim->id < 0 || dsim->id >= MAX_DSI_CNT) {
		dsim_err(dsim, "wrong dsim id(%d)\n", dsim->id);
		return -ENODEV;
	}

	dsc_np = of_parse_phandle(np, "dsc-config", 0);
	if (!dsc_np) {
		dsim->config.dsc.enabled = false;
	} else {
		dsim->config.dsc.enabled = true;
		of_property_read_u32(dsc_np, "dsc_count",
				&dsim->config.dsc.dsc_count);
		of_property_read_u32(dsc_np, "slice_count",
				&dsim->config.dsc.slice_count);
		of_property_read_u32(dsc_np, "slice_height",
				&dsim->config.dsc.slice_height);
	}

	dsim->pll_params = dsim_of_get_clock_mode(dsim);

	return 0;
}

static int dsim_remap_regs(struct dsim_device *dsim)
{
	struct resource *res;
	struct device_node *np;
	struct device *dev = dsim->dev;
	struct platform_device *pdev;
	int ret = 0;

	pdev = container_of(dev, struct platform_device, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dsim_err(dsim, "failed to find dsim memory resource\n");
		return -EINVAL;
	}
	dsim->res.regs = devm_ioremap_resource(dsim->dev, res);
	if (IS_ERR(dsim->res.regs)) {
		dsim_err(dsim, "failed to remap io region\n");
		ret = PTR_ERR(dsim->res.regs);
		return -EINVAL;
	}

	dsim_regs_desc_init(dsim->res.regs, "dsi", REGS_DSIM_DSI, dsim->id);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dsim_err(dsim, "failed to find dphy memory resource\n");
		return -EINVAL;
	}
	dsim->res.phy_regs = devm_ioremap_resource(dsim->dev, res);
	if (IS_ERR(dsim->res.phy_regs)) {
		dsim_err(dsim, "failed to remap io region\n");
		ret = PTR_ERR(dsim->res.regs);
		return -EINVAL;
	}
	dsim_regs_desc_init(dsim->res.phy_regs, "dphy", REGS_DSIM_PHY,
			dsim->id);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res == NULL) {
		dsim_err(dsim, "failed to find dphy extra memory resource\n");
		return -EINVAL;
	}
	dsim->res.phy_regs_ex = devm_ioremap_resource(dsim->dev, res);
	if (IS_ERR(dsim->res.phy_regs_ex))
		dsim_warn(dsim, "failed to remap io region. it's optional\n");
	dsim_regs_desc_init(dsim->res.phy_regs_ex, "dphy-extra",
			REGS_DSIM_PHY_BIAS, dsim->id);

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9-disp_ss");
	if (IS_ERR_OR_NULL(np)) {
		dsim_err(dsim, "failed to find compatible node(sysreg-disp)");
		return PTR_ERR(np);
	}
	dsim->res.ss_reg_base = of_iomap(np, 0);
	if (!dsim->res.ss_reg_base) {
		dsim_err(dsim, "failed to map sysreg-disp address.");
		return -ENOMEM;
	}
	dsim_regs_desc_init(dsim->res.ss_reg_base, np->name, REGS_DSIM_SYS,
			dsim->id);

	return 0;
}

static irqreturn_t dsim_irq_handler(int irq, void *dev_id)
{
	struct dsim_device *dsim = dev_id;
	struct decon_device *decon =
		(struct decon_device *)to_exynos_crtc(dsim->encoder.crtc)->ctx;
	unsigned int int_src;

	spin_lock(&dsim->slock);

	dsim_dbg(dsim, "%s +\n", __func__);

	if (dsim->state == DSIM_STATE_SUSPEND) {
		dsim_info(dsim, "dsim power is off state(0x%x)\n", dsim->state);
		spin_unlock(&dsim->slock);
		return IRQ_HANDLED;
	}

	int_src = dsim_reg_get_int_and_clear(dsim->id);
	if (int_src & DSIM_INTSRC_SFR_PH_FIFO_EMPTY) {
		del_timer(&dsim->cmd_timer);
		complete(&dsim->ph_wr_comp);
		dsim_dbg(dsim, "dsim%d PH_FIFO_EMPTY irq occurs\n", dsim->id);
	}
	if (int_src & DSIM_INTSRC_RX_DATA_DONE)
		complete(&dsim->rd_comp);
	if (int_src & DSIM_INTSRC_FRAME_DONE)
		dsim_dbg(dsim, "dsim%d framedone irq occurs\n", dsim->id);
	if (int_src & DSIM_INTSRC_ERR_RX_ECC)
		dsim_err(dsim, "RX ECC Multibit error was detected!\n");

	if (int_src & DSIM_INTSRC_UNDER_RUN) {
		dsim_info(dsim, "dsim%d underrun irq occurs\n", dsim->id);
		DPU_EVENT_LOG(DPU_EVT_DSIM_UNDERRUN, decon->id, dsim);
	}

	if (int_src & DSIM_INTSRC_VT_STATUS) {
		dsim_dbg(dsim, "dsim%d vt_status irq occurs\n", dsim->id);
		/* This will be implemented for video mode in the future */
		//if (decon) {
			//decon->vsync.timestamp = ktime_get();
			//wake_up_interruptible_all(&decon->vsync.wait);
		//}
	}

	spin_unlock(&dsim->slock);

	return IRQ_HANDLED;
}

static int dsim_register_irq(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct platform_device *pdev;
	struct resource *res;
	int ret = 0;

	pdev = container_of(dev, struct platform_device, dev);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dsim_err(dsim, "failed to get irq resource\n");
		return -ENOENT;
	}

	dsim->irq = res->start;
	ret = devm_request_irq(dsim->dev, res->start,
			dsim_irq_handler, 0, pdev->name, dsim);
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

static int dsim_get_clock(struct dsim_device *dsim)
{
	dsim->res.aclk = devm_clk_get(dsim->dev, "aclk");
	if (IS_ERR_OR_NULL(dsim->res.aclk)) {
		dsim_err(dsim, "failed to get aclk\n");
		return PTR_ERR(dsim->res.aclk);
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

	ret = dsim_get_clock(dsim);
	if (ret)
		goto err;

err:
	return ret;
}

static void dsim_convert_cal_data(struct dsim_device *dsim)
{
	struct drm_display_mode *native_mode = &dsim->native_mode;
	struct videomode videomode;

	drm_display_mode_to_videomode(native_mode, &videomode);

	dsim_adjust_video_timing(dsim, videomode, native_mode->name);

	dsim->config.data_lane_cnt = dsim->lanes;
	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO)
		dsim->config.mode = DSIM_VIDEO_MODE;
	else
		dsim->config.mode = DSIM_COMMAND_MODE;

	/* TODO: This hard coded information will be defined in device tree */
	dsim->config.mres_mode = 0;
	dsim->config.dsc.slice_width = DIV_ROUND_UP(
			dsim->config.p_timing.hactive,
			dsim->config.dsc.slice_count);

	dsim_info(dsim, "dsc is %s [%d %d %d %d]\n",
			dsim->config.dsc.enabled ? "enabled" : "disabled",
			dsim->config.dsc.dsc_count,
			dsim->config.dsc.slice_count,
			dsim->config.dsc.slice_width,
			dsim->config.dsc.slice_height);
}

static int dsim_get_display_mode(struct dsim_device *dsim)
{
	struct drm_connector *connector = &dsim->connector;
	int ret;

	ret = drm_panel_get_modes(dsim->panel);
	if (ret < 0) {
		dsim_err(dsim, "failed to get panel display modes\n");
		return ret;
	}

	if (!list_empty(&connector->probed_modes)) {
		struct drm_display_mode *preferred_mode;

		list_for_each_entry(preferred_mode, &connector->probed_modes,
				head) {
			if (preferred_mode->type & DRM_MODE_TYPE_PREFERRED) {
				dsim->native_mode = *preferred_mode;
				dsim_convert_cal_data(dsim);
			}
		}
	} else {
		dsim->native_mode.clock = 0;
	}

	return 0;
}

static int dsim_host_attach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct dsim_device *dsim = host_to_dsi(host);
	struct drm_device *drm = dsim->connector.dev;
	int ret;

	dsim_info(dsim, "%s +\n", __func__);

	mutex_lock(&drm->mode_config.mutex);

	dsim->lanes = device->lanes;
	dsim->format = device->format;
	dsim->mode_flags = device->mode_flags;

	dsim->panel = of_drm_find_panel(device->dev.of_node);
	if (dsim->panel) {
		ret = drm_panel_attach(dsim->panel, &dsim->connector);
		if (ret < 0) {
			dsim_err(dsim, "failed to attach drm panel\n");
			mutex_unlock(&drm->mode_config.mutex);
			return ret;
		}
		dsim->connector.status = connector_status_connected;
	}

	ret = dsim_get_display_mode(dsim);
	if (ret < 0) {
		dsim_err(dsim, "failed to get panel display info\n");
		mutex_unlock(&drm->mode_config.mutex);
		return ret;
	}

	mutex_unlock(&drm->mode_config.mutex);

	if (drm->mode_config.poll_enabled)
		drm_kms_helper_hotplug_event(drm);

	dsim_info(dsim, "%s -\n", __func__);
	return 0;
}

static int dsim_host_detach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct dsim_device *dsim = host_to_dsi(host);
	struct drm_device *drm = dsim->connector.dev;

	dsim_info(dsim, "%s +\n", __func__);
	mutex_lock(&drm->mode_config.mutex);

	if (dsim->panel) {
		dsim_disable(&dsim->encoder);
		drm_panel_detach(dsim->panel);
		dsim->panel = NULL;
		dsim->connector.status = connector_status_disconnected;
	}

	mutex_unlock(&drm->mode_config.mutex);

	if (drm->mode_config.poll_enabled)
		drm_kms_helper_hotplug_event(drm);

	dsim_info(dsim, "%s -\n", __func__);
	return 0;
}

static void dsim_cmd_fail_detector(struct timer_list *arg)
{
	struct dsim_device *dsim = from_timer(dsim, arg, cmd_timer);

	dsim_dbg(dsim, "%s +\n", __func__);

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
	dsim_dbg(dsim, "%s -\n", __func__);
}

static int dsim_wait_for_cmd_fifo_empty(struct dsim_device *dsim,
		bool must_wait)
{
	int ret = 0;

	if (!must_wait) {
		/* timer is running, but already command is transferred */
		if (dsim_reg_header_fifo_is_empty(dsim->id))
			del_timer(&dsim->cmd_timer);

		dsim_dbg(dsim, "Doesn't need to wait fifo_completion\n");
		return ret;
	}

	del_timer(&dsim->cmd_timer);
	dsim_dbg(dsim, "%s Waiting for fifo_completion...\n", __func__);

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
		dsim_err(dsim, "%s have timed out\n", __func__);

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
			dsim_dbg(dsim, "count = 3 payload = %x, %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)));
			} else if ((d1 - data_cnt) == 2) {
				payload = *(u8 *)(d0 + data_cnt) |
					(*(u8 *)(d0 + (data_cnt + 1))) << 8;
			dsim_dbg(dsim, "count = 2 payload = %x, %x %x\n",
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

			dsim_dbg(dsim, "count = 4 payload = %x, %x %x %x %x\n",
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
		dsim_dbg(dsim, "%s: id:%d, data=%ld\n", __func__, data_id,
				data0);
		return true;
	}

	/* Check a FIFO level whether writable or not */
	if (!dsim_reg_is_writable_fifo_state(dsim->id))
		return true;

	return false;
}

int dsim_write_data(struct dsim_device *dsim, u32 id, unsigned long d0, u32 d1)
{
	int ret = 0;
	bool must_wait = true;
	struct decon_device *decon =
		(struct decon_device *)to_exynos_crtc(dsim->encoder.crtc)->ctx;

	mutex_lock(&dsim->cmd_lock);
	if (dsim->state != DSIM_STATE_HSCLKEN) {
		dsim_err(dsim, "dsim%d is not ready(%d)\n",
				dsim->id, dsim->state);
		ret = -EINVAL;
		goto err_exit;
	}

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

static ssize_t dsim_host_transfer(struct mipi_dsi_host *host,
			    const struct mipi_dsi_msg *msg)
{
	struct dsim_device *dsim = host_to_dsi(host);
	const u8 *data;

	data = msg->tx_buf;
	dsim_wr_data(dsim, msg->type, data, msg->tx_len);

	return 0;
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
	char temp[1];

	sprintf(temp, "%d\n", dsim->bist_mode);
	strcpy(buf, temp);

	return strlen(buf);
}

static ssize_t bist_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int rc;

	rc = kstrtouint(buf, 0, &dsim->bist_mode);
	if (rc < 0)
		return rc;

	/*
	 * 1: Color Bar, 2: GRAY Gradient, 3: User-Defined, 4: Prbs7 Random
	 */
	if (dsim->bist_mode > 0) {
		if (dsim->state == DSIM_STATE_SUSPEND)
			dsim_enable(&dsim->encoder);
		//dsim_reg_set_bist(dsi->id, dsi->bist_mode);
	} else {
		//dsim_reg_set_bist(dsi->id, dsi->bist_mode);
		if (dsim->state == DSIM_STATE_HSCLKEN)
			dsim_disable(&dsim->encoder);
	}

	dsim_info(dsim, "0:Disable 1:Color Bar 2:GRAY Gradient 3:User-Defined\n");
	dsim_info(dsim, "4:Prbs7 Random (%d)\n", dsim->bist_mode);

	return len;
}
static DEVICE_ATTR_RW(bist_mode);

int dpu_sysmmu_fault_handler(struct iommu_domain *domain,
	struct device *dev, unsigned long iova, int flags, void *token)
{
	pr_info("%s +\n", __func__);
	return 0;
}

static int dsim_probe(struct platform_device *pdev)
{
	struct dsim_device *dsim;
	int ret;

	pr_info("%s +\n", __func__);

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
		dsim_warn(dsim, "idle ip index is not provided for dsim%d\n",
				dsim->id);
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	dsim->state = DSIM_STATE_SUSPEND;
	pm_runtime_enable(dsim->dev);

	ret = iovmm_activate(dsim->dev);
	if (ret) {
		dsim_err(dsim, "failed to activate iovmm\n");
		goto err;
	}
	iovmm_set_fault_handler(dsim->dev, dpu_sysmmu_fault_handler, NULL);

	phy_init(dsim->res.phy);
	if (dsim->res.phy_ex)
		phy_init(dsim->res.phy_ex);

	dsim_info(dsim, "dsim%d driver has been probed.\n", dsim->id);
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

	return 0;
}

static int __maybe_unused dsim_suspend(struct device *dev)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	int ret;

	dsim_dbg(dsim, "%s +\n", __func__);

	ret = phy_power_off(dsim->res.phy);
	if (ret < 0) {
		dsim_err(dsim, "failed to disable phy(%d)\n", ret);
		return ret;
	}
	if (dsim->res.phy_ex) {
		ret = phy_power_off(dsim->res.phy_ex);
		if (ret < 0) {
			dsim_err(dsim, "failed to disable extra phy(%d)\n",
					ret);
			return ret;
		}
	}

	clk_disable_unprepare(dsim->res.aclk);

	dsim_dbg(dsim, "%s -\n", __func__);

	return 0;
}

static int __maybe_unused dsim_resume(struct device *dev)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	int ret;

	dsim_dbg(dsim, "%s +\n", __func__);

	clk_prepare_enable(dsim->res.aclk);

	ret = phy_power_on(dsim->res.phy);
	if (ret < 0) {
		dsim_err(dsim, "failed to enable phy(%d)\n", ret);
		return ret;
	}
	if (dsim->res.phy_ex) {
		ret = phy_power_on(dsim->res.phy_ex);
		if (ret < 0) {
			dsim_err(dsim, "failed to enable extra phy(%d)\n", ret);
			return ret;
		}
	}

	dsim_dbg(dsim, "%s -\n", __func__);

	return 0;
}

static const struct dev_pm_ops dsim_pm_ops = {
	SET_RUNTIME_PM_OPS(dsim_suspend, dsim_resume, NULL)
};

struct platform_driver dsim_driver = {
	.probe = dsim_probe,
	.remove = dsim_remove,
	.driver = {
		   .name = "exynos-dsim",
		   .owner = THIS_MODULE,
		   .pm = &dsim_pm_ops,
		   .of_match_table = dsim_of_match,
	},
};

MODULE_SOFTDEP("pre: phy-exynos-mipi");
MODULE_AUTHOR("Donghwa Lee <dh09.lee@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC MIPI DSI Master");
MODULE_LICENSE("GPL v2");
