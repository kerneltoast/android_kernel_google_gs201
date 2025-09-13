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

#include <drm/drm_of.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_vblank.h>

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/component.h>
#include <linux/iommu.h>

#include <video/mipi_display.h>

#if defined(CONFIG_CPU_IDLE)
#include <soc/google/exynos-cpupm.h>
#endif

#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
#include <soc/google/exynos-devfreq.h>
#if defined(CONFIG_SOC_GS101)
#include <dt-bindings/soc/google/gs101-devfreq.h>
#elif defined(CONFIG_SOC_GS201)
#include <dt-bindings/soc/google/gs201-devfreq.h>
#elif defined(CONFIG_SOC_ZUMA)
#include <dt-bindings/soc/google/zuma-devfreq.h>
#endif
#endif

#include <regs-dsim.h>

#include <trace/dpu_trace.h>

#include "exynos_drm_connector.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_dsim.h"

#if IS_ENABLED(CONFIG_QUEUE_DDIC_CMD_CALL_BACK)
#include "panel/panel-samsung-drv.h"
#endif

#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
#include "gs_drm/gs_drm_connector.h"
#define BRIDGE_PORT 0
#define BRIDGE_ENDPOINT 0
#endif

struct dsim_device *dsim_drvdata[MAX_DSI_CNT];

/*
 * This global mutex lock protects to initialize or de-initialize DSIM and DPHY
 * hardware when multi display is in operation
 */
DEFINE_MUTEX(g_dsim_lock);

#define PANEL_DRV_LEN 64
#define RETRY_READ_FIFO_MAX 10
#define PANEL_ID_LENGTH 4
#define LEGACY_PANEL_ID_LENGTH 3

static char panel_name[PANEL_DRV_LEN];
module_param_string(panel_name, panel_name, sizeof(panel_name), 0644);
MODULE_PARM_DESC(panel_name, "preferred panel name");

static char sec_panel_name[PANEL_DRV_LEN];
module_param_string(sec_panel_name, sec_panel_name, sizeof(sec_panel_name), 0644);
MODULE_PARM_DESC(sec_panel_name, "secondary panel name");

enum panel_priority_index {
	PANEL_PRIORITY_PRI_IDX = 0,
	PANEL_PRIORITY_SEC_IDX = 1,
};
static u8 panel_usage = {0};

#define dsim_info(dsim, fmt, ...)	\
pr_info("%s[%d]: "fmt, dsim->dev->driver->name, dsim->id, ##__VA_ARGS__)

#define dsim_warn(dsim, fmt, ...)	\
pr_warn("%s[%d]: "fmt, dsim->dev->driver->name, dsim->id, ##__VA_ARGS__)

#define dsim_err(dsim, fmt, ...)	\
pr_err("%s[%d]: "fmt, dsim->dev->driver->name, dsim->id, ##__VA_ARGS__)

#define dsim_debug(dsim, fmt, ...)	\
pr_debug("%s[%d]: "fmt, dsim->dev->driver->name, dsim->id, ##__VA_ARGS__)

#define DSIM_ESCAPE_CLK_20MHZ	20

//#define DSIM_BIST

#define DEFAULT_TE_IDLE_US              1000
#define DEFAULT_TE_VARIATION            1

static const struct of_device_id dsim_of_match[] = {
	{ .compatible = "samsung,exynos-dsim",
	  .data = NULL },
	{ }
};
MODULE_DEVICE_TABLE(of, dsim_of_match);

static int dsim_calc_underrun(const struct dsim_device *dsim, uint32_t hs_clock_mhz,
		uint32_t *underrun);
static int dsim_set_hs_clock(struct dsim_device *dsim, unsigned int hs_clock, bool apply_now);

inline void dsim_trace_msleep(u32 delay_ms)
{
	trace_msleep(delay_ms);
	usleep_range(delay_ms * 1000, delay_ms * 1000 + 10);
}
EXPORT_SYMBOL_GPL(dsim_trace_msleep);

static struct drm_crtc *drm_encoder_get_new_crtc(struct drm_encoder *encoder,
						 struct drm_atomic_state *state)
{
	struct drm_connector *connector;
	const struct drm_connector_state *conn_state;

	connector = drm_atomic_get_new_connector_for_encoder(state, encoder);
	if (!connector)
		return NULL;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state)
		return NULL;

	return conn_state->crtc;
}

static struct drm_crtc *drm_encoder_get_old_crtc(struct drm_encoder *encoder,
						 struct drm_atomic_state *state)
{
	struct drm_connector *connector;
	const struct drm_connector_state *conn_state;

	connector = drm_atomic_get_old_connector_for_encoder(state, encoder);
	if (!connector)
		return NULL;

	conn_state = drm_atomic_get_old_connector_state(state, connector);
	if (!conn_state)
		return NULL;

	return conn_state->crtc;
}

void dsim_dump(struct dsim_device *dsim, struct drm_printer *p)
{
	struct dsim_regs regs;
	struct drm_printer printer, *pointer;

	if (!p) {
		printer = is_console_enabled() ?
			drm_debug_printer("[drm]") : drm_info_printer(dsim->dev);
			pointer = &printer;
	} else {
		pointer = p;
	}

	drm_printf(pointer, "%s[%d]: === DSIM SFR DUMP ===\n",
		dsim->dev->driver->name, dsim->id);

	if (dsim->state != DSIM_STATE_HSCLKEN)
		return;

	regs.regs = dsim->res.regs;
	regs.ss_regs = dsim->res.ss_reg_base;
	regs.phy_regs = dsim->res.phy_regs;
	regs.phy_regs_ex = dsim->res.phy_regs_ex;
	__dsim_dump(pointer, dsim->id, &regs);
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

static void _dsim_exit_ulps_locked(struct dsim_device *dsim)
{
	const struct decon_device *decon = dsim_get_decon(dsim);

	WARN_ON(!mutex_is_locked(&dsim->state_lock));

	if (dsim->state != DSIM_STATE_ULPS)
		return;

	dsim_debug(dsim, "+\n");

	DPU_ATRACE_BEGIN(__func__);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	dsim_phy_power_on(dsim);

	mutex_lock(&g_dsim_lock);
	dsim_reg_init(dsim->id, &dsim->config, &dsim->clk_param, false);
	dsim_reg_exit_ulps_and_start(dsim->id, 0, 0x1F);
	mutex_unlock(&g_dsim_lock);

	dsim->state = DSIM_STATE_HSCLKEN;
	enable_irq(dsim->irq);

	dsim_debug(dsim, "-\n");
	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_ULPS_EXIT, decon->id, dsim);

	DPU_ATRACE_END(__func__);
}

static void dsim_set_te_pinctrl(struct dsim_device *dsim, bool en)
{
	int ret;

	if (!dsim->hw_trigger || !dsim->te_on || !dsim->te_off)
		return;

	ret = pinctrl_select_state(dsim->pinctrl, en ? dsim->te_on : dsim->te_off);
	if (ret)
		dsim_err(dsim, "failed to control decon TE(%d)\n", en);
}

static void _dsim_enable(struct dsim_device *dsim)
{
	const struct decon_device *decon = dsim_get_decon(dsim);
	struct dsim_device *sec_dsi;
	bool skip_init = false;

	pm_runtime_get_sync(dsim->dev);

	mutex_lock(&dsim->state_lock);
	if (dsim->state == DSIM_STATE_HSCLKEN) {
		mutex_unlock(&dsim->state_lock);
		pm_runtime_put_sync(dsim->dev);
		dsim_info(dsim, "already enabled(%d)\n", dsim->state);
		return;
	}

	dsim_debug(dsim, "%s +\n", __func__);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	dsim_phy_power_on(dsim);
	if (dsim->state == DSIM_STATE_HANDOVER) {
		skip_init = dsim_reg_is_pll_stable(dsim->id);
		dsim_info(dsim, "dsim handover. skip_init=%d\n", skip_init);
	}

	mutex_lock(&g_dsim_lock);

	if (!skip_init)
		dsim_reg_init(dsim->id, &dsim->config, &dsim->clk_param, true);

	dsim_reg_start(dsim->id);
	mutex_unlock(&g_dsim_lock);

	/* TODO: dsi start: enable irq, sfr configuration */
	dsim->state = DSIM_STATE_HSCLKEN;
	enable_irq(dsim->irq);
	mutex_unlock(&dsim->state_lock);

#if defined(DSIM_BIST)
	dsim_reg_set_bist(dsim->id, true, DSIM_GRAY_GRADATION);
	dsim_dump(dsim, NULL);
#endif

	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_ENABLED, decon->id, dsim);

	dsim_debug(dsim, "%s -\n", __func__);

	if (dsim->dual_dsi == DSIM_DUAL_DSI_MAIN) {
		sec_dsi = exynos_get_dual_dsi(DSIM_DUAL_DSI_SEC);
		if (sec_dsi)
			_dsim_enable(sec_dsi);
		else
			dsim_err(dsim, "could not get secondary dsi\n");
	}
}

static void dsim_encoder_enable(struct drm_encoder *encoder, struct drm_atomic_state *state)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct drm_crtc *crtc = drm_encoder_get_new_crtc(encoder, state);
	struct drm_crtc_state *old_crtc_state = drm_atomic_get_old_crtc_state(state, crtc);
	const struct decon_device *decon = to_exynos_crtc(crtc)->ctx;
	struct device *supplier = decon->dev;
	struct drm_connector *connector =
			drm_atomic_get_new_connector_for_encoder(state, encoder);

	dsim_debug(dsim, "current state: %d\n", dsim->state);

	if (connector) {
		struct drm_connector_state *new_conn_state =
				drm_atomic_get_new_connector_state(state, connector);
		struct drm_connector_state *old_conn_state =
				drm_atomic_get_old_connector_state(state, connector);

		if (new_conn_state) {
			struct dsim_connector_funcs *funcs = get_connector_funcs(new_conn_state);

			if (funcs && funcs->update_hs_clk) {
				funcs->update_hs_clk(dsim, new_conn_state);

				/* Ideally new clock should be applied while enabling DSIM */
				mutex_lock(&dsim->state_lock);
				dsim->clk_param.hs_clk_changed = false;
				mutex_unlock(&dsim->state_lock);
			}
		}

		if (old_conn_state && atomic_read(&decon->recovery.recovering)) {
			struct dsim_connector_funcs *funcs = get_connector_funcs(old_conn_state);

			if (funcs && funcs->set_state_recovering) {
				funcs->set_state_recovering(old_conn_state);
				dsim_debug(dsim, "%s: doing recovery\n", __func__);
			}
		}
	}

	if (dsim->dev_link && (dsim->dev_link->supplier != supplier)) {
		device_link_del(dsim->dev_link);
		dsim->dev_link = NULL;
	}

	if (!dsim->dev_link) {
		const u32 dl_flags = DL_FLAG_PM_RUNTIME | DL_FLAG_AUTOREMOVE_SUPPLIER;

		dsim->dev_link = device_link_add(dsim->dev, supplier, dl_flags);
		if (WARN(!dsim->dev_link, "unable to create dev link between decon/dsim\n"))
			return;
	}

	if (dsim->state == DSIM_STATE_SUSPEND || dsim->state == DSIM_STATE_HANDOVER) {
		_dsim_enable(dsim);
		dsim_set_te_pinctrl(dsim, 1);
	} else if (dsim->state == DSIM_STATE_BYPASS) {
		pm_runtime_set_suspended(dsim->dev);
		dsim->state = DSIM_STATE_SUSPEND;
		_dsim_enable(dsim);
	} else if (old_crtc_state->self_refresh_active) {
		/* get extra ref count dropped when going into self refresh */
		pm_runtime_get_sync(dsim->dev);
	} else {
		WARN(1, "unknown dsim state (%d)\n", dsim->state);
	}
}

static inline bool dsim_cmd_packetgo_is_enabled(const struct dsim_device *dsim)
{
	return dsim->total_pend_ph > 0;
}

static void __dsim_cmd_packetgo_enable_locked(struct dsim_device *dsim, bool en)
{
	if (en) {
		pm_runtime_get_sync(dsim->dev);
		dsim_debug(dsim, "enabling packetgo\n");
	}

	dsim_reg_enable_packetgo(dsim->id, en);
	if (!en) {
		pm_runtime_put(dsim->dev);

		dsim->total_pend_ph = 0;
		dsim->total_pend_pl = 0;
		dsim_debug(dsim, "packetgo disabled\n");
	}
}

static void __dsim_check_pend_cmd_locked(struct dsim_device *dsim)
{
	WARN_ON(!mutex_is_locked(&dsim->cmd_lock));

	if (WARN_ON(dsim_reg_has_pend_cmd(dsim->id)))
		dsim_dump(dsim, NULL);

	if (WARN(dsim_cmd_packetgo_is_enabled(dsim), "pending packets remaining ph(%u) pl(%u)\n",
		 dsim->total_pend_ph, dsim->total_pend_pl))
		__dsim_cmd_packetgo_enable_locked(dsim, false);
}

static void _dsim_enter_ulps_locked(struct dsim_device *dsim)
{
	const struct decon_device *decon = dsim_get_decon(dsim);

	WARN_ON(!mutex_is_locked(&dsim->state_lock));

	DPU_ATRACE_BEGIN(__func__);

	dsim_debug(dsim, "+\n");

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	__dsim_check_pend_cmd_locked(dsim);
	dsim->state = DSIM_STATE_ULPS;
	mutex_unlock(&dsim->cmd_lock);

	disable_irq(dsim->irq);
	mutex_lock(&g_dsim_lock);
	dsim_reg_stop_and_enter_ulps(dsim->id, 0, 0x1F);
	mutex_unlock(&g_dsim_lock);

	dsim_phy_power_off(dsim);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif
	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_ULPS_ENTER, decon->id, dsim);

	dsim_debug(dsim, "-\n");

	DPU_ATRACE_END(__func__);
}

static void _dsim_disable(struct dsim_device *dsim)
{
	const struct decon_device *decon = dsim_get_decon(dsim);
	struct dsim_device *sec_dsi;

	if (dsim->dual_dsi == DSIM_DUAL_DSI_MAIN) {
		sec_dsi = exynos_get_dual_dsi(DSIM_DUAL_DSI_SEC);
		if (sec_dsi)
			_dsim_disable(sec_dsi);
		else
			dsim_err(dsim, "could not get secondary dsi\n");
	}

	dsim_debug(dsim, "+\n");
	mutex_lock(&dsim->state_lock);
	if (dsim->state == DSIM_STATE_SUSPEND) {
		mutex_unlock(&dsim->state_lock);
		dsim_info(dsim, "already disabled(%d)\n", dsim->state);
		return;
	}

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	mutex_lock(&g_dsim_lock);
	/* TODO: 0x1F will be changed */
	dsim_reg_stop(dsim->id, 0x1F);
	mutex_unlock(&g_dsim_lock);
	disable_irq(dsim->irq);

	dsim->state = DSIM_STATE_SUSPEND;
	__dsim_check_pend_cmd_locked(dsim);

	dsim->force_batching = false;
	mutex_unlock(&dsim->cmd_lock);
	mutex_unlock(&dsim->state_lock);

	dsim_phy_power_off(dsim);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	pm_runtime_put_sync(dsim->dev);

	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_DISABLED, decon->id, dsim);

	dsim_debug(dsim, "-\n");
}

static void dsim_encoder_disable(struct drm_encoder *encoder, struct drm_atomic_state *state)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct drm_crtc *crtc = drm_encoder_get_old_crtc(encoder, state);
	bool self_refresh_active = false;
	bool was_in_self_refresh = false;
	u32 pending_hs_clk;

	if (crtc) {
		const struct drm_crtc_state *old_crtc_state =
			drm_atomic_get_old_crtc_state(state, crtc);
		const struct drm_crtc_state *new_crtc_state =
			drm_atomic_get_new_crtc_state(state, crtc);

		if (old_crtc_state && old_crtc_state->self_refresh_active)
			was_in_self_refresh = true;
		if (new_crtc_state && new_crtc_state->self_refresh_active)
			self_refresh_active = true;
	}

	dsim_debug(dsim, "state: %d self_refresh: %d->%d\n", dsim->state,
		   was_in_self_refresh, self_refresh_active);

	DPU_ATRACE_BEGIN(__func__);

	mutex_lock(&dsim->state_lock);
	pending_hs_clk = dsim->clk_param.pending_hs_clk;
	mutex_unlock(&dsim->state_lock);
	if (pending_hs_clk) {
		if (dsim_set_hs_clock(dsim, pending_hs_clk, false) < 0)
			dsim_warn(dsim, "failed to set pending_hs_clk=%u\n", pending_hs_clk);
		else
			dsim_info(dsim, "set pending_hs_clk=%u\n", pending_hs_clk);
	}

	if (self_refresh_active) {
		const struct exynos_drm_crtc_state *exynos_crtc_state =
			to_exynos_crtc_state(crtc->state);

		if (was_in_self_refresh) {
			WARN(1, "already in self refresh state. state:%d\n", dsim->state);
		} else if (exynos_crtc_state->bypass) {
			/* in bypass mode dsim should get fully disabled */
			_dsim_disable(dsim);
			dsim->state = DSIM_STATE_BYPASS;
		} else {
			/* during regular self refresh just need to go into runtime idle */
			pm_runtime_put_sync(dsim->dev);
		}
	} else {
		if (dsim->state == DSIM_STATE_BYPASS) {
			pm_runtime_set_suspended(dsim->dev);
			dsim->state = DSIM_STATE_SUSPEND;
		} else if (was_in_self_refresh) {
			/* get extra ref count dropped when going into self refresh */
			pm_runtime_get_sync(dsim->dev);

			dsim_debug(dsim, "disable right after self refresh. state:%d\n",
				  dsim->state);
		}

		_dsim_disable(dsim);

		dsim_set_te_pinctrl(dsim, 0);
	}

	DPU_ATRACE_END(__func__);
}

static struct dsim_pll_param *
dsim_get_clock_mode(const struct dsim_device *dsim,
		    const struct drm_display_mode *mode)
{
	int i;
	const size_t mlen = strnlen(mode->name, DRM_DISPLAY_MODE_LEN);
	struct dsim_pll_param *ret = NULL;
	size_t plen;

	if (!dsim->pll_params || !dsim->pll_params->num_modes)
		return NULL;

	for (i = 0; i < dsim->pll_params->num_modes; i++) {
		struct dsim_pll_param *p = dsim->pll_params->params[i];

		plen = strnlen(p->name, DRM_DISPLAY_MODE_LEN);

		if (!strncmp(mode->name, p->name, plen)) {
			ret = p;
			/*
			 * if it's not exact match, continue looking for exact
			 * match, use this as a fallback
			 */
			if (plen == mlen)
				break;
		}
	}

	return ret;
}

static void dsim_update_clock_config(struct dsim_device *dsim,
				     const struct dsim_pll_param *p)
{
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

	dsim_debug(dsim, "clk_param.hs_clk=%u, pll_freq=%u, pending_hs_clk=%u\n",
		dsim->clk_param.hs_clk, p->pll_freq, dsim->clk_param.pending_hs_clk);
	if (dsim->clk_param.hs_clk != p->pll_freq) {
		dsim->clk_param.hs_clk_changed = true;
		dsim->clk_param.pending_hs_clk = 0;
	} else if (dsim->clk_param.pending_hs_clk) {
		dsim->clk_param.hs_clk_changed = true;
	}
	dsim->clk_param.hs_clk = p->pll_freq;
	dsim->clk_param.esc_clk = p->esc_freq;
	if (dsim->clk_param.hs_clk_changed) {
		dsim_info(dsim, "hs_clk is changed to %u\n", p->pll_freq);
		DPU_ATRACE_INT("mipi_clk", p->pll_freq);
	}

	dsim_debug(dsim, "found proper pll parameter\n");
	dsim_debug(dsim, "\t%s(p:0x%x,m:0x%x,s:0x%x,k:0x%x)\n", p->name,
		 dsim->config.dphy_pms.p, dsim->config.dphy_pms.m,
		 dsim->config.dphy_pms.s, dsim->config.dphy_pms.k);

	dsim_debug(dsim, "\t%s(hs:%d,esc:%d)\n", p->name, dsim->clk_param.hs_clk,
		 dsim->clk_param.esc_clk);

	if (p->cmd_underrun_cnt) {
		dsim->config.cmd_underrun_cnt[0] = p->cmd_underrun_cnt;
	} else {
		dsim_warn(dsim, "cmd_underrun_cnt is not set correctly\n");
		WARN_ON(1);
	}

	dsim_debug(dsim, "\tunderrun_lp_ref 0x%x\n", dsim->config.cmd_underrun_cnt[0]);
}

static int dsim_set_clock_mode(struct dsim_device *dsim,
			       const struct drm_display_mode *mode)
{
	struct dsim_pll_param *p = dsim_get_clock_mode(dsim, mode);
	uint32_t underrun_cnt;

	if (!p)
		return -ENOENT;

	if (!dsim_calc_underrun(dsim, p->pll_freq, &underrun_cnt))
		p->cmd_underrun_cnt = underrun_cnt;

	dsim_update_clock_config(dsim, p);
	dsim->current_pll_param = p;

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

static struct dsim_allowed_hs_clks *dsim_of_get_allowed_hs_clks(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct device_node *np;
	struct dsim_allowed_hs_clks *clock_rates;

	np = of_parse_phandle(dev->of_node, "clock_rates", 0);
	if (!np) {
		dsim_warn(dsim, "failed to get clock_rates\n");
		return NULL;
	}

	clock_rates = devm_kzalloc(dsim->dev, sizeof(*clock_rates), GFP_KERNEL);
	if (!clock_rates) {
		of_node_put(np);
		return NULL;
	}

	clock_rates->num_clks = of_property_count_u32_elems(np, "clock-rates");
	if (clock_rates->num_clks <= 0) {
		dsim_warn(dsim, "clock-rates empty\n");
		goto read_node_fail;
	}

	clock_rates->hs_clks = devm_kzalloc(dsim->dev,
				sizeof(u32) * clock_rates->num_clks, GFP_KERNEL);
	if (!clock_rates->hs_clks)
		goto read_node_fail;
	if (of_property_read_u32_array(
			np, "clock-rates", clock_rates->hs_clks, clock_rates->num_clks) < 0) {
		dsim_err(dsim, "%s, failed to read clock-rates\n", __func__);
		devm_kfree(dsim->dev, clock_rates->hs_clks);
		goto read_node_fail;
	}

	for (int i = 0; i < clock_rates->num_clks; i++)
		dsim_debug(dsim, "allowed clk #%d: %u\n", i, clock_rates->hs_clks[i]);

	of_node_put(np);
	return clock_rates;

read_node_fail:
	of_node_put(np);
	devm_kfree(dsim->dev, clock_rates);
	return NULL;
}

static struct dsim_pll_features *dsim_of_get_pll_features(
		struct dsim_device *dsim, struct device_node *np)
{
	u64 range64[2];
	u32 range32[2];
	struct dsim_pll_features *pll_features;

	pll_features = devm_kzalloc(dsim->dev, sizeof(*pll_features), GFP_KERNEL);
	if (!pll_features)
		return NULL;

	if (of_property_read_u64(np, "pll-input", &pll_features->finput) < 0) {
		dsim_err(dsim, "%s failed to get pll-input\n", __func__);
		goto read_node_fail;
	}

	if (of_property_read_u64(np, "pll-optimum",
				 &pll_features->foptimum) < 0) {
		dsim_err(dsim, "%s failed to get pll-optimum\n", __func__);
		goto read_node_fail;
	}

	if (of_property_read_u64_array(np, "pll-out-range", range64, 2) < 0) {
		dsim_err(dsim, "%s failed to get pll-out-range\n", __func__);
		goto read_node_fail;
	}
	pll_features->fout_min = range64[0];
	pll_features->fout_max = range64[1];

	if (of_property_read_u64_array(np, "pll-vco-range", range64, 2) < 0) {
		dsim_err(dsim, "%s failed to get pll-vco-range\n", __func__);
		goto read_node_fail;
	}
	pll_features->fvco_min = range64[0];
	pll_features->fvco_max = range64[1];

	if (of_property_read_u32_array(np, "p-range", range32, 2) < 0) {
		dsim_err(dsim, "%s failed to get p-range\n", __func__);
		goto read_node_fail;
	}
	pll_features->p_min = range32[0];
	pll_features->p_max = range32[1];

	if (of_property_read_u32_array(np, "m-range", range32, 2) < 0) {
		dsim_err(dsim, "%s failed to get m-range\n", __func__);
		goto read_node_fail;
	}
	pll_features->m_min = range32[0];
	pll_features->m_max = range32[1];

	if (of_property_read_u32_array(np, "s-range", range32, 2) < 0) {
		dsim_err(dsim, "%s failed to get s-range\n", __func__);
		goto read_node_fail;
	}
	pll_features->s_min = range32[0];
	pll_features->s_max = range32[1];

	if (of_property_read_u32(np, "k-bits", &pll_features->k_bits) < 0) {
		dsim_err(dsim, "%s failed to get k-bits\n", __func__);
		goto read_node_fail;
	}

	dsim_debug(dsim, "pll features: input %llu, optimum%llu\n",
		  pll_features->finput, pll_features->foptimum);
	dsim_debug(dsim, "pll features: output(%llu, %llu)\n",
		  pll_features->fout_min, pll_features->fout_max);
	dsim_debug(dsim, "pll features: vco (%llu, %llu)\n",
		  pll_features->fvco_min, pll_features->fout_max);
	dsim_debug(dsim, "pll limits: p(%u, %u), m(%u, %u), s(%u, %u), k(%u)\n",
		  pll_features->p_min, pll_features->p_max,
		  pll_features->m_min, pll_features->m_max,
		  pll_features->s_min, pll_features->s_max,
		  pll_features->k_bits);

	return pll_features;

read_node_fail:
	kfree(pll_features);
	return NULL;
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
		goto err_put_np;
	}

	pll_params = devm_kzalloc(dsim->dev, sizeof(*pll_params), GFP_KERNEL);
	if (!pll_params)
		goto err_put_mode_np;

	entry = of_get_next_child(mode_np, NULL);
	if (!entry) {
		dsim_err(dsim, "could not find child node of dsim-modes");
		goto err_put_mode_np;
	}

	pll_params->num_modes = of_get_child_count(mode_np);
	if (pll_params->num_modes == 0) {
		dsim_err(dsim, "%pOF: no modes specified\n", np);
		goto err_put_mode_np;
	}

	pll_params->params = devm_kcalloc(dsim->dev, sizeof(struct dsim_pll_param *),
				pll_params->num_modes, GFP_KERNEL);
	if (!pll_params->params)
		goto err_put_mode_np;

	pll_params->num_modes = 0;

	for_each_child_of_node(mode_np, entry) {
		struct dsim_pll_param *pll_param;

		pll_param = devm_kzalloc(dsim->dev, sizeof(*pll_param), GFP_KERNEL);
		if (!pll_param)
			goto err_put_entry;

		if (dsim_of_parse_modes(entry, pll_param) < 0) {
			kfree(pll_param);
			continue;
		}

		pll_params->params[pll_params->num_modes] = pll_param;
		pll_params->num_modes++;
	}

	pll_params->features = dsim_of_get_pll_features(dsim, np);

	of_node_put(np);
	of_node_put(mode_np);
	of_node_put(entry);

	return pll_params;

err_put_entry:
	of_node_put(entry);
err_put_mode_np:
	of_node_put(mode_np);
err_put_np:
	of_node_put(np);
	return NULL;
}

static void dsim_restart(struct dsim_device *dsim)
{
	mutex_lock(&dsim->cmd_lock);
	mutex_lock(&g_dsim_lock);
	dsim_reg_stop(dsim->id, 0x1F);
	mutex_unlock(&g_dsim_lock);
	disable_irq(dsim->irq);

	mutex_lock(&g_dsim_lock);
	dsim_reg_init(dsim->id, &dsim->config, &dsim->clk_param, true);
	dsim_reg_start(dsim->id);
	mutex_unlock(&g_dsim_lock);
	enable_irq(dsim->irq);
	mutex_unlock(&dsim->cmd_lock);
}

#ifdef CONFIG_DEBUG_FS

static int dsim_of_parse_diag(struct device_node *np, struct dsim_dphy_diag *diag)
{
        int count;
        u8 bit_range[2];
        const char *reg_base = NULL;

        of_property_read_string(np, "reg-base", &reg_base);
        if (!strcmp(reg_base, "dphy")) {
                diag->reg_base = REGS_DSIM_PHY;
        } else if (!strcmp(reg_base, "dphy-extra")) {
                diag->reg_base = REGS_DSIM_PHY_BIAS;
        } else {
                pr_err("%s: invalid reg-base: %s\n", __func__, reg_base);
                return -EINVAL;
        }

        of_property_read_string(np, "diag-name", &diag->name);
        if (!diag->name || !diag->name[0]) {
                pr_err("%s: empty diag-name\n", __func__);
                return -EINVAL;
        }

        of_property_read_string(np, "desc", &diag->desc);
        of_property_read_string(np, "help", &diag->help);

        count = of_property_count_u16_elems(np, "reg-offset");
        if (count <= 0 || count > MAX_DIAG_REG_NUM) {
                pr_err("%s: wrong number of reg-offset: %d\n", __func__, count);
                return -ERANGE;
        }

        if (of_property_read_u16_array(np, "reg-offset", diag->reg_offset, count) < 0) {
                pr_err("%s: failed to read reg-offset\n", __func__);
                return -EINVAL;
        }
        diag->num_reg = count;

        if (of_property_read_u8_array(np, "bit-range", bit_range, 2) < 0) {
                pr_err("%s: failed to read bit-range\n", __func__);
                return -EINVAL;
        }

        if (bit_range[0] >= 32 || bit_range[1] >= 32) {
                pr_err("%s: invalid bit range %d, %d\n", __func__, bit_range[0], bit_range[1]);
                return -EINVAL;
        }
        if (bit_range[0] < bit_range[1]) {
                diag->bit_start = bit_range[0];
                diag->bit_end = bit_range[1];
        } else {
                diag->bit_start = bit_range[1];
                diag->bit_end = bit_range[0];
        }
        diag->read_only = of_property_read_bool(np, "read_only");

        return 0;
}

static void dsim_of_get_pll_diags(struct dsim_device *dsim)
{
	struct device_node *np, *entry;
	struct device *dev = dsim->dev;
        uint32_t index = 0;

	np = of_parse_phandle(dev->of_node, "dphy_diag", 0);
        dsim->config.num_dphy_diags = of_get_child_count(np);
        if (dsim->config.num_dphy_diags == 0) {
                goto nochild;
        }

	dsim->config.dphy_diags = devm_kzalloc(dsim->dev, sizeof(struct dsim_dphy_diag) *
					dsim->config.num_dphy_diags, GFP_KERNEL);
	if (!dsim->config.dphy_diags) {
                dsim_warn(dsim, "%s: no memory for %u diag items\n",
                          __func__, dsim->config.num_dphy_diags);
                dsim->config.num_dphy_diags = 0;
                goto nochild;
        }

        for_each_child_of_node(np, entry) {
                if (index >= dsim->config.num_dphy_diags) {
                      dsim_warn(dsim, "%s: diag parsing error with unexpected index %u\n",
                                __func__, index);
                      goto get_diag_fail;
                }

                if (dsim_of_parse_diag(entry, &dsim->config.dphy_diags[index]) < 0) {
                      dsim_warn(dsim, "%s: diag parsing error for item %u\n",
                                __func__, index);
                      goto get_diag_fail;
                }
                ++index;
        }
        return;

get_diag_fail:
        dsim->config.num_dphy_diags = 0;
        devm_kfree(dsim->dev, dsim->config.dphy_diags);
        dsim->config.dphy_diags = NULL;
nochild:
        return;
}

int dsim_dphy_diag_get_reg(struct dsim_device *dsim,
                           struct dsim_dphy_diag *diag, uint32_t *vals)
{
	int ret;
	uint32_t ix, mask, val;

	ret = dsim_dphy_diag_mask_from_range(diag->bit_start, diag->bit_end,
					     &mask);
	if (ret < 0)
		return ret;

	mutex_lock(&dsim->state_lock);
	if (dsim->state != DSIM_STATE_HSCLKEN) {
		ret = -ENODEV;
		goto out;
	}

	for (ix = 0; ix < diag->num_reg; ++ix) {
		if (diag->reg_base == REGS_DSIM_PHY_BIAS)
			val = diag_dsim_dphy_extra_reg_read_mask(
				dsim->id, diag->reg_offset[ix], mask);
		else if (diag->reg_base == REGS_DSIM_PHY)
			val = diag_dsim_dphy_reg_read_mask(
				dsim->id, diag->reg_offset[ix], mask);
		else {
			pr_err("%s: invalid reg_base %u\n", __func__,
			       diag->reg_base);
			goto out;
		}
		vals[ix] = val >> diag->bit_start;
	}
out:
	mutex_unlock(&dsim->state_lock);
	return ret;
}

int dsim_dphy_diag_set_reg(struct dsim_device *dsim,
                           struct dsim_dphy_diag *diag, uint32_t val)
{
	int ret;
	u32 mask;

	ret = dsim_dphy_diag_mask_from_range(diag->bit_start, diag->bit_end,
					     &mask);
	if (ret < 0)
		return ret;

	diag->override = true;
	diag->user_value = (val << diag->bit_start) & mask;

	mutex_lock(&dsim->state_lock);
	if (dsim->state != DSIM_STATE_HSCLKEN)
		goto out;

	/* restart dsim to apply new config */
	dsim_restart(dsim);

out:
	mutex_unlock(&dsim->state_lock);
	return ret;
}

#else

static void dsim_of_get_pll_diags(struct dsim_device * /* dsim */)
{
}

#endif

/* exynos connector funcs */

static void _update_config_timing(struct dsim_reg_config *config,
				  const struct drm_display_mode *mode, bool has_underrun_param,
				  unsigned int te_idle_us, unsigned int te_var,
				  unsigned int min_bts_fps)
{
	struct dpu_panel_timing *p_timing = &config->p_timing;
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	p_timing->vactive = vm.vactive;
	p_timing->vfp = vm.vfront_porch;
	p_timing->vbp = vm.vback_porch;
	p_timing->vsa = vm.vsync_len;

	p_timing->hactive = vm.hactive;
	if (config->dual_dsi)
		p_timing->hactive /= 2;
	p_timing->hfp = vm.hfront_porch;
	p_timing->hbp = vm.hback_porch;
	p_timing->hsa = vm.hsync_len;
	p_timing->vrefresh = config->mode == DSIM_VIDEO_MODE ?
		drm_mode_vrefresh(mode) : exynos_drm_mode_bts_fps(mode, min_bts_fps);
	if (has_underrun_param) {
		p_timing->te_idle_us = te_idle_us;
		p_timing->te_var = te_var;
	} else {
		p_timing->te_idle_us = DEFAULT_TE_IDLE_US;
		p_timing->te_var = DEFAULT_TE_VARIATION;
		pr_debug("%s: underrun_param for mode " DRM_MODE_FMT
			" not specified", __func__, DRM_MODE_ARG(mode));
	}
}

static void _update_config_mode_flags(struct dsim_reg_config *config, unsigned long mode_flags)
{
	/* TODO: This hard coded information will be defined in device tree */
	config->mres_mode = 0;
	config->mode = (mode_flags & MIPI_DSI_MODE_VIDEO) ? DSIM_VIDEO_MODE : DSIM_COMMAND_MODE;
}

static void _update_config_dsc(struct dsim_reg_config *config, unsigned int bpc, bool dsc_enabled,
			       unsigned int dsc_count, unsigned int slice_count,
			       unsigned int slice_height)

{
	config->bpp = bpc * 3;
	config->dsc.enabled = dsc_enabled;
	if (config->dsc.enabled) {
		config->dsc.dsc_count = dsc_count;
		config->dsc.slice_count = slice_count;
		config->dsc.slice_height = slice_height;
		config->dsc.slice_width = DIV_ROUND_UP(
				config->p_timing.hactive,
				config->dsc.slice_count);
	}
}

static void update_config_for_mode_exynos(struct dsim_reg_config *config,
					  const struct drm_display_mode *mode,
					  const struct drm_connector_state *conn_state)
{
	const struct exynos_drm_connector_state *exynos_conn_state =
		to_exynos_connector_state(conn_state);
	const struct exynos_display_mode *exynos_mode = &exynos_conn_state->exynos_mode;
	unsigned int te_idle_us =
		exynos_mode->underrun_param ? exynos_mode->underrun_param->te_idle_us : 0;
	unsigned int te_var = exynos_mode->underrun_param ? exynos_mode->underrun_param->te_var : 0;

	_update_config_timing(config, mode, exynos_mode->underrun_param ? true : false, te_idle_us,
			      te_var, exynos_mode->min_bts_fps);
	_update_config_mode_flags(config, exynos_mode->mode_flags);
	_update_config_dsc(config, exynos_mode->bpc, exynos_mode->dsc.enabled,
			   exynos_mode->dsc.dsc_count, exynos_mode->dsc.slice_count,
			   exynos_mode->dsc.slice_height);
}

static void dsim_set_display_mode(struct dsim_device *dsim, const struct drm_display_mode *mode,
				  const struct drm_connector_state *conn_state, bool sw_trigger)
{
	struct dsim_device *sec_dsi;
	struct dsim_connector_funcs *funcs = get_connector_funcs(conn_state);

	if (IS_ERR_OR_NULL(dsim->dsi_device))
		return;

	mutex_lock(&dsim->state_lock);
	dsim->config.data_lane_cnt = dsim->dsi_device->lanes;
	dsim->hw_trigger = !sw_trigger;

	dsim->config.dual_dsi = dsim->dual_dsi;

	if (funcs && funcs->update_config_for_mode)
		funcs->update_config_for_mode(&dsim->config, mode, conn_state);
	dsim_set_clock_mode(dsim, mode);

	if (dsim->state == DSIM_STATE_HSCLKEN)
		dsim_reg_set_rr_config(dsim->id, &dsim->config, &dsim->clk_param);

	dsim_debug(dsim, "dsim mode %s dsc is %s [%d %d %d %d]\n",
			dsim->config.mode == DSIM_VIDEO_MODE ? "video" : "cmd",
			dsim->config.dsc.enabled ? "enabled" : "disabled",
			dsim->config.dsc.dsc_count,
			dsim->config.dsc.slice_count,
			dsim->config.dsc.slice_width,
			dsim->config.dsc.slice_height);
	mutex_unlock(&dsim->state_lock);

	if (dsim->dual_dsi == DSIM_DUAL_DSI_MAIN) {
		sec_dsi = exynos_get_dual_dsi(DSIM_DUAL_DSI_SEC);
		if (sec_dsi) {
			sec_dsi->dsi_device = dsim->dsi_device;
			dsim_set_display_mode(sec_dsi, mode, conn_state, sw_trigger);
		} else
			dsim_err(dsim, "could not get main dsi\n");
	}
}

static void dsim_atomic_mode_set_exynos(struct dsim_device *dsim, struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	const struct exynos_drm_connector_state *exynos_conn_state =
		to_exynos_connector_state(conn_state);
	const struct exynos_display_mode *exynos_mode = &exynos_conn_state->exynos_mode;

	dsim_set_display_mode(dsim, &crtc_state->adjusted_mode, conn_state,
			      exynos_mode->sw_trigger);
}

static void dsim_atomic_mode_set(struct drm_encoder *encoder, struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	const struct dsim_connector_funcs *funcs = get_connector_funcs(conn_state);

	if (funcs && funcs->atomic_mode_set) {
		funcs->atomic_mode_set(dsim, crtc_state, conn_state);
	}
}

static enum drm_mode_status dsim_mode_valid(struct drm_encoder *encoder,
					    const struct drm_display_mode *mode)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);

	if (!dsim_get_clock_mode(dsim, mode))
		return MODE_NOMODE;

	return MODE_OK;
}

static void set_state_recovering_exynos(struct drm_connector_state *conn_state)
{
	struct exynos_drm_connector_state *exynos_conn_state;

	exynos_conn_state = to_exynos_connector_state(conn_state);
	exynos_conn_state->is_recovering = true;
}

/*
 * Check whether mode change can happean seamlessly from dsim perspective.
 * Seamless mode switch from dsim perspective can only happen if there's no
 * need to change dsim configuration.
 */
static bool dsim_mode_is_seamless(const struct dsim_device *dsim,
				  const struct drm_display_mode *mode,
				  const struct drm_connector_state *conn_state)
{
	struct dsim_reg_config new_config = dsim->config;
	struct dsim_connector_funcs *funcs = get_connector_funcs(conn_state);

	if (dsim->current_pll_param != dsim_get_clock_mode(dsim, mode)) {
		dsim_debug(dsim, "clock mode change not allowed seamlessly\n");
		return false;
	}

	if (funcs && funcs->update_config_for_mode) {
		funcs->update_config_for_mode(&new_config, mode, conn_state);
	}
	if (dsim->config.mode != new_config.mode) {
		dsim_debug(dsim, "op mode change not allowed seamlessly\n");
		return false;
	}

	if (memcmp(&dsim->config.dsc, &new_config.dsc, sizeof(new_config.dsc))) {
		dsim_debug(dsim, "dsc change not allowed seamlessly\n");
		return false;
	}

	return true;
}

static int dsim_atomic_check_exynos(const struct dsim_device *dsim,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	struct exynos_drm_connector_state *exynos_conn_state =
		to_exynos_connector_state(conn_state);
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;

	if (exynos_conn_state->seamless_possible &&
	    !dsim_mode_is_seamless(dsim, mode, conn_state)) {
		dsim_warn(dsim, "%s: seamless mode switch not supported for %s\n", __func__,
			  mode->name);
		exynos_conn_state->seamless_possible = false;
	}

	if (!exynos_conn_state->exynos_mode.sw_trigger) {
		if (!dsim->pinctrl) {
			dsim_err(dsim, "TE error: pinctrl not found\n");
			return -EINVAL;
		} else if ((dsim->te_gpio < 0) || (dsim->te_from >= MAX_DECON_TE_FROM_DDI)) {
			dsim_err(dsim, "invalid TE config for hw trigger mode\n");
			return -EINVAL;
		}

		exynos_conn_state->te_from = dsim->te_from;
		exynos_conn_state->te_gpio = dsim->te_gpio;
		if (dsim->tout_gpio >= 0)
			exynos_conn_state->tout_gpio = dsim->tout_gpio;
	}
	return 0;
}

static void update_hs_clk_exynos(struct dsim_device *dsim, struct drm_connector_state *conn_state)
{
}

static struct dsim_connector_funcs exynos_dsim_connector_funcs = {
	.atomic_check = &dsim_atomic_check_exynos,
	.atomic_mode_set = &dsim_atomic_mode_set_exynos,
	.set_state_recovering = &set_state_recovering_exynos,
	.update_config_for_mode = &update_config_for_mode_exynos,
	.update_hs_clk = &update_hs_clk_exynos,
};

static int dsim_atomic_check(struct drm_encoder *encoder,
			     struct drm_crtc_state *crtc_state,
			     struct drm_connector_state *connector_state)
{
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	const struct dsim_connector_funcs *funcs = get_connector_funcs(connector_state);

	if (funcs && funcs->update_hs_clk)
		funcs->update_hs_clk(dsim, connector_state);

	if (crtc_state->mode_changed) {
		if (funcs && funcs->atomic_check)
			return funcs->atomic_check(dsim, crtc_state, connector_state);
	}
	return 0;
}

#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
/* gs connector funcs */
static int dsim_atomic_check_gs(const struct dsim_device *dsim, struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state)
{
	struct gs_drm_connector_state *gs_conn_state = to_gs_connector_state(conn_state);
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;

	if (gs_conn_state->seamless_possible && !dsim_mode_is_seamless(dsim, mode, conn_state)) {
		dsim_warn(dsim, "%s: seamless mode switch not supported for %s\n", __func__,
			  mode->name);
		gs_conn_state->seamless_possible = false;
	}

	if (!gs_conn_state->gs_mode.sw_trigger) {
		if (!dsim->pinctrl) {
			dsim_err(dsim, "TE error: pinctrl not found\n");
			return -EINVAL;
		} else if ((dsim->te_gpio < 0) || (dsim->te_from >= MAX_DECON_TE_FROM_DDI)) {
			dsim_err(dsim, "invalid TE config for hw trigger mode\n");
			return -EINVAL;
		}

		gs_conn_state->te_from = dsim->te_from;
		gs_conn_state->te_gpio = dsim->te_gpio;
		if (dsim->tout_gpio >= 0)
			gs_conn_state->tout_gpio = dsim->tout_gpio;
	}
	return 0;
}

static void update_config_for_mode_gs(struct dsim_reg_config *config,
				      const struct drm_display_mode *mode,
				      const struct drm_connector_state *conn_state)
{
	const struct gs_drm_connector_state *gs_conn_state = to_gs_connector_state(conn_state);
	const struct gs_display_mode *gs_mode = &gs_conn_state->gs_mode;
	unsigned int te_idle_us = gs_mode->underrun_param ? gs_mode->underrun_param->te_idle_us : 0;
	unsigned int te_var = gs_mode->underrun_param ? gs_mode->underrun_param->te_var : 0;
	unsigned int slice_count = 0;
	unsigned int slice_height = 0;

	_update_config_timing(config, mode, gs_mode->underrun_param ? true : false, te_idle_us,
			      te_var, gs_mode->min_bts_fps);
	_update_config_mode_flags(config, gs_mode->mode_flags);
	if (gs_mode->dsc.cfg) {
		slice_count = gs_mode->dsc.cfg->slice_count;
		slice_height = gs_mode->dsc.cfg->slice_height;
	}
	_update_config_dsc(config, gs_mode->bpc, gs_mode->dsc.enabled, gs_mode->dsc.dsc_count,
			   slice_count, slice_height);
}

static void dsim_atomic_mode_set_gs(struct dsim_device *dsim, struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	const struct gs_drm_connector_state *gs_conn_state = to_gs_connector_state(conn_state);
	const struct gs_display_mode *gs_mode = &gs_conn_state->gs_mode;

	dsim_set_display_mode(dsim, &crtc_state->adjusted_mode, conn_state, gs_mode->sw_trigger);
}

static void set_state_recovering_gs(struct drm_connector_state *conn_state)
{
	struct gs_drm_connector_state *gs_conn_state;

	gs_conn_state = to_gs_connector_state(conn_state);
	gs_conn_state->is_recovering = true;
}

static void update_hs_clk_gs(struct dsim_device *dsim, struct drm_connector_state *conn_state)
{
	struct gs_drm_connector_state *gs_conn_state = to_gs_connector_state(conn_state);

	mutex_lock(&dsim->state_lock);
	gs_conn_state->dsi_hs_clk_mbps = dsim->clk_param.hs_clk;
	gs_conn_state->pending_dsi_hs_clk_mbps = dsim->clk_param.pending_hs_clk;
	gs_conn_state->dsi_hs_clk_changed = dsim->clk_param.hs_clk_changed;
	mutex_unlock(&dsim->state_lock);
}

static struct dsim_connector_funcs gs_dsim_connector_funcs = {
	.atomic_check = &dsim_atomic_check_gs,
	.atomic_mode_set = &dsim_atomic_mode_set_gs,
	.set_state_recovering = &set_state_recovering_gs,
	.update_config_for_mode = &update_config_for_mode_gs,
	.update_hs_clk = &update_hs_clk_gs,
};
#endif

struct dsim_connector_funcs *get_connector_funcs(const struct drm_connector_state *conn_state)
{
	if (is_exynos_drm_connector(conn_state->connector))
		return &exynos_dsim_connector_funcs;
#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
	else if (is_gs_drm_connector(conn_state->connector))
		return &gs_dsim_connector_funcs;
#endif
	else if (conn_state->connector)
		dev_warn(conn_state->connector->kdev, "%s Unsupported connector type\n", __func__);
	return NULL;
}

static const struct drm_encoder_helper_funcs dsim_encoder_helper_funcs = {
	.mode_valid = dsim_mode_valid,
	.atomic_mode_set = dsim_atomic_mode_set,
	.atomic_enable = dsim_encoder_enable,
	.atomic_disable = dsim_encoder_disable,
	.atomic_check = dsim_atomic_check,
};

#ifdef CONFIG_DEBUG_FS

static int dsim_encoder_late_register(struct drm_encoder *encoder)
{
        struct dsim_device *dsim = encoder_to_dsim(encoder);
        dsim_diag_create_debugfs(dsim);
        return 0;
}

static void dsim_encoder_early_unregister(struct drm_encoder *encoder)
{
        struct dsim_device *dsim = encoder_to_dsim(encoder);
        dsim_diag_remove_debugfs(dsim);
}

#else

static int dsim_encoder_late_register(struct drm_encoder * /* encoder */)
{
        return 0;
}

static void dsim_encoder_early_unregister(struct drm_encoder * /*encoder */)
{
}

#endif

static const struct drm_encoder_funcs dsim_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
        .late_register = dsim_encoder_late_register,
        .early_unregister = dsim_encoder_early_unregister,
};

#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
/**
 * dsim_has_graph_link_to_bridge() - Checks possible of graph link to connector
 * @dsim_dev: Pointer to device object associated with dsim device
 *
 * Specifically, this function checks whether ports exist from the dsim entry in
 * the device tree to a valid DT node, using the BRIDGE_PORT and BRIDGE_ENDPOINT
 * constants (default: port 0, endpoint 0). Does not check whether that DT node
 * describes a valid drm_connector device.
 *
 * Return: whether dsim has a potential of graph connection to a connector
 */
static bool dsim_has_graph_link_to_bridge(struct device *dsim_dev)
{
	struct device_node *remote;

	if (!of_graph_is_present(dsim_dev->of_node))
		return false;
	remote = of_graph_get_remote_node(dsim_dev->of_node, BRIDGE_PORT, BRIDGE_ENDPOINT);
	if (!remote)
		return false;
	return true;
}

/**
 * dsim_parse_remote_port() - Get reg property of remote port in a connector
 * @dsim_dev: Pointer to device object associated with dsim device
 *
 * Specifically, this function parses the reg property of the port linked from
 * the dsim entry in the device tree, using the BRIDGE_PORT and BRIDGE_ENDPOINT
 * constants (default: port 0, endpoint 0).
 *
 * Return: reg property of the remote port linked from the dsim entry
 */
static int dsim_parse_remote_port(struct device *dsim_dev)
{
	struct device_node *endpoint_node, *remote_endpoint_node;
	struct of_endpoint endpoint;

	/* Get endpoint node in dsim device */
	endpoint_node =
		of_graph_get_endpoint_by_regs(dsim_dev->of_node, BRIDGE_PORT, BRIDGE_ENDPOINT);
	if (!endpoint_node)
		return 0;

	/* Get remote-endpoint node linked to endpoint_node */
	remote_endpoint_node = of_graph_get_remote_endpoint(endpoint_node);
	of_node_put(endpoint_node);
	if (!remote_endpoint_node)
		return 0;

	/* Get endpoint data structure from remote_endpoint_node */
	of_graph_parse_endpoint(remote_endpoint_node, &endpoint);
	of_node_put(remote_endpoint_node);

	/* Return reg property of the port which endpoint belongs to */
	return endpoint.port;
}
#endif

static int dsim_add_mipi_dsi_device(struct dsim_device *dsim,
	const char *pname, enum panel_priority_index idx)
{
	struct mipi_dsi_device_info info = {
		.node = NULL,
	};
	struct device_node *node;
	const char *name;
	const char *dual_dsi;
	const char *p;
	u32 cmp_len;

	p = strchr(pname, '.');
	cmp_len = p ? min((ptrdiff_t)PANEL_DRV_LEN, p - pname) : strnlen(pname, PANEL_DRV_LEN);

	dsim_debug(dsim, "preferred panel is %.*s\n", cmp_len, pname);

#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
	if (dsim_has_graph_link_to_bridge(dsim->dev)) {
		/* If using gs_drm_connector, only need to change name;
		 * panel detection and name comparison happens in connector
		 */
		gs_connector_set_panel_name(pname, strnlen(pname, PANEL_DRV_LEN), idx);
		if (dsim_parse_remote_port(dsim->dev) >= 1)
			dsim->dual_dsi = DSIM_DUAL_DSI_SEC;
		/* if port == 0, then we can find out if it's actual dual dsi or not later */
		return 0;
	}
	/* implied else case: search for legacy panel below */
#endif
	for_each_available_child_of_node(dsim->dsi_host.dev->of_node, node) {
		bool found;

		/*
		 * panel w/ reg node will be added in mipi_dsi_host_register,
		 * abort panel detection in that case
		 */
		if (of_find_property(node, "reg", NULL)) {
			if (info.node)
				of_node_put(info.node);

			return -ENODEV;
		}

		/*
		 * We already detected panel we want but keep iterating
		 * in case there are devices with reg property
		 */
		if (info.node)
			continue;

		if (of_property_read_u32(node, "channel", &info.channel))
			continue;

		name = of_get_property(node, "label", NULL);
		if (!name)
			continue;

		/* if panel name is not specified pick the first device found */
		found = !strncmp(name, pname, cmp_len);
		if (pname[0] == '\0' || found) {
			/*
			 * The default is primary panel. If not, add priority
			 * index in the panel name, i.e. "priority-index:panel-name"
			 */
			if (found && idx > PANEL_PRIORITY_PRI_IDX)
				scnprintf(info.type, sizeof(info.type),
					"%d:%s", idx, name);
			else
				strlcpy(info.type, name, sizeof(info.type));
			info.node = of_node_get(node);
		}
	}

	if (info.node) {
		if (!of_property_read_string(info.node, "dual-dsi", &dual_dsi)) {
			if (!strcmp(dual_dsi, "main"))
				dsim->dual_dsi = DSIM_DUAL_DSI_MAIN;
			else if (!strcmp(dual_dsi, "sec"))
				dsim->dual_dsi = DSIM_DUAL_DSI_SEC;
			else
				dsim->dual_dsi = DSIM_DUAL_DSI_NONE;
		}
		if (dsim->dual_dsi != DSIM_DUAL_DSI_SEC)
			mipi_dsi_device_register_full(&dsim->dsi_host, &info);

		if (dsim->dual_dsi == DSIM_DUAL_DSI_NONE)
			panel_usage |= BIT(idx);
		return 0;
	}

	return -ENODEV;
}

static const char *dsim_get_panel_name(const struct dsim_device *dsim, const char *name, size_t len)
{
	const char *p = strchr(name, ':');
	size_t pos;
	int dsim_id;

	/* if colon is found expect format which includes intended intf
	 * (ex. dsim0:panel_name), otherwise fallback to panel name only */
	if (!p)
		return name;

	if ((len < 5) || strncmp(name, "dsim", 4))
		return NULL;

	dsim_id = name[4] - '0';
	if (dsim->id != dsim_id)
		return NULL;

	p++;
	pos = p - name;
	if (pos >= len)
		return NULL;

	return p;
}

static u32 dsim_get_panel_id(const char *name)
{
	char *p = strchr(name, '.');
	u8 panel_id[PANEL_ID_LENGTH] = {0};

	/* if period is found, expect 6 or 8 hex characters (ex. panel_name.000000),
	 * otherwise return an invalid panel ID */
	if (!p)
		return INVALID_PANEL_ID;

	p++;

	if ((strlen(p) == (PANEL_ID_LENGTH * 2) &&
		!hex2bin(panel_id, p, PANEL_ID_LENGTH)) ||
		(strlen(p) == (LEGACY_PANEL_ID_LENGTH * 2) &&
		!hex2bin(panel_id + 1, p, LEGACY_PANEL_ID_LENGTH))) {
		return __builtin_bswap32(*(u32*)panel_id);
	}

	return INVALID_PANEL_ID;
}

static int dsim_parse_panel_name(struct dsim_device *dsim)
{
	const char *name;
	enum panel_priority_index idx;

	name = dsim_get_panel_name(dsim, panel_name, PANEL_DRV_LEN);
	idx = PANEL_PRIORITY_PRI_IDX;
	if (name && !(panel_usage & BIT(idx))) {
		dsim->panel_id = dsim_get_panel_id(panel_name);
		dsim->panel_index = idx;
		return dsim_add_mipi_dsi_device(dsim, name, idx);
	}

	name = dsim_get_panel_name(dsim, sec_panel_name, PANEL_DRV_LEN);
	idx = PANEL_PRIORITY_SEC_IDX;
	if (name && name[0] && !(panel_usage & BIT(idx))) {
		dsim->panel_id = dsim_get_panel_id(sec_panel_name);
		dsim->panel_index = idx;
		return dsim_add_mipi_dsi_device(dsim, name, idx);
	}

	return -ENODEV;
}

static struct drm_bridge *dsim_find_bridge_legacy(struct mipi_dsi_device *device)
{
	struct drm_bridge *bridge;
	bridge = of_drm_find_bridge(device->dev.of_node);
	if (!bridge) {
		struct drm_panel *panel;

		panel = of_drm_find_panel(device->dev.of_node);
		if (IS_ERR(panel)) {
			dev_err(&device->dev, "failed to find panel\n");
			return (struct drm_bridge *)panel; /* IS_ERR */
		}

		bridge = devm_drm_panel_bridge_add_typed(&device->dev, panel,
							 DRM_MODE_CONNECTOR_DSI);
		return bridge;
	}
	return bridge;
}

static int dsim_attach_bridge(struct dsim_device *dsim)
{
	struct drm_bridge *bridge;
	struct device_node *np;
	int ret;

	np = dsim->dsi_device->dev.of_node;
	if (!np)
		return -ENOENT;

#if IS_ENABLED(CONFIG_GS_DRM_PANEL_UNIFIED)
	if (dsim_has_graph_link_to_bridge(dsim->dev)) {
		bridge = devm_drm_of_get_bridge(dsim->dev, dsim->dev->of_node, BRIDGE_PORT,
						BRIDGE_ENDPOINT);
		if (IS_ERR(bridge)) {
			return PTR_ERR(bridge);
		}
	} else {
		/* compiled for unified panel but not using gs_connector */
		bridge = dsim_find_bridge_legacy(dsim->dsi_device);
	}
#else
	bridge = dsim_find_bridge_legacy(dsim->dsi_device);
#endif
	if (IS_ERR(bridge)) {
		dsim_err(dsim, "failed to create panel bridge\n");
		return PTR_ERR(bridge);
	}

	ret = drm_bridge_attach(&dsim->encoder, bridge, NULL, 0);
	if (ret)
		dsim_err(dsim, "Unable to attach panel bridge\n");
	else
		dsim->panel_bridge = bridge;

	return ret;
}

static bool is_primary_panel(struct dsim_device *dsim)
{
	if (!dsim->dsi_device) {
		dsim_info(dsim, "no dsi_device\n");
		return false;
	}

	return (dsim->panel_index == 0);
}

static int dsim_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dsim_device *dsim = encoder_to_dsim(encoder);
	struct drm_device *drm_dev = data;
	static bool primary_attached;
	int i, ret = 0;

	dsim_debug(dsim, "%s +\n", __func__);

	/* ignore bind calls for dual dsi */
	if (dsim->dual_dsi == DSIM_DUAL_DSI_SEC)
		return 0;

	if (!dsim->dsi_device) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(dsim->dsi_device)) {
		int ret = PTR_ERR(dsim->dsi_device);

		/* ignore cases where there's no dsi device to be attached */
		return ret == -ENODEV ? 0 : ret;
	}

	if (of_property_read_bool(dsim->dsi_device->dev.of_node, "google,dual-dsi-panel")) {
		if (!exynos_get_dual_dsi(DSIM_DUAL_DSI_SEC))
			dsim_err(dsim, "google,dual-dsi-panel is set, but cannot get sec dsi\n");
		else
			dsim->dual_dsi = DSIM_DUAL_DSI_MAIN;
	}

	drm_encoder_init(drm_dev, encoder, &dsim_encoder_funcs,
			 DRM_MODE_ENCODER_DSI, NULL);
	drm_encoder_helper_add(encoder, &dsim_encoder_helper_funcs);

	encoder->possible_crtcs = exynos_drm_get_possible_crtcs(encoder,
							dsim->output_type);
	if (!encoder->possible_crtcs) {
		dsim_err(dsim, "failed to get possible crtc, ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		dsim->encoder_initialized = false;
		return -ENOTSUPP;
	}
	dsim->encoder_initialized = true;

	if (primary_attached || is_primary_panel(dsim)) {
		dsim_attach_bridge(dsim);
		primary_attached = true;

		for (i = 0; i < MAX_DSI_CNT; i++) {
			if (dsim_drvdata[i] && (dsim_drvdata[i]->id != dsim->id) &&
				dsim_drvdata[i]->panel_bridge == NULL &&
				dsim_drvdata[i]->encoder_initialized)
				dsim_attach_bridge(dsim_drvdata[i]);
		}
	}
	dsim_debug(dsim, "%s -\n", __func__);

	return ret;
}

static void dsim_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dsim_device *dsim = encoder_to_dsim(encoder);

	dsim_debug(dsim, "%s +\n", __func__);

	if (dsim->dual_dsi == DSIM_DUAL_DSI_SEC)
		return;
}

static const struct component_ops dsim_component_ops = {
	.bind	= dsim_bind,
	.unbind	= dsim_unbind,
};

static int dsim_parse_dt(struct dsim_device *dsim)
{
	struct device_node *np = dsim->dev->of_node;
	int ret;

	if (!np) {
		dsim_err(dsim, "no device tree information\n");
		return -ENOTSUPP;
	}

	of_property_read_u32(np, "dsim,id", &dsim->id);
	if (dsim->id < 0 || dsim->id >= MAX_DSI_CNT) {
		dsim_err(dsim, "wrong dsim id(%d)\n", dsim->id);
		return -ENODEV;
	}

	dsim->allowed_hs_clks = dsim_of_get_allowed_hs_clks(dsim);
	dsim->pll_params = dsim_of_get_clock_mode(dsim);
	dsim_of_get_pll_diags(dsim);

	ret = of_property_read_u32(np, "te_from", &dsim->te_from);
	if (ret) {
		dsim->te_from = MAX_DECON_TE_FROM_DDI;
		dsim_warn(dsim, "failed to get TE from DDI\n");
	}
	dsim_debug(dsim, "TE from DDI%d\n", dsim->te_from);

	if (!ret) {
		dsim->te_gpio = of_get_named_gpio(np, "te-gpio", 0);
		if (dsim->te_gpio < 0) {
			dsim_warn(dsim, "failed to get TE gpio\n");
			dsim->te_from = MAX_DECON_TE_FROM_DDI;
		}
	}

	dsim->tout_gpio = of_get_named_gpio(np, "tout-gpio", 0);
	if (dsim->tout_gpio < 0)
		dsim_warn(dsim, "failed to get TOUT (TE2) gpio\n");

	return 0;
}

static int dsim_remap_regs(struct dsim_device *dsim)
{
	struct resource res;
	struct device *dev = dsim->dev;
	struct device_node *np = dev->of_node;
	int i, ret = 0;

	i = of_property_match_string(np, "reg-names", "dsi");
	if (of_address_to_resource(np, i, &res)) {
		pr_err("failed to get dsi resource\n");
		goto err;
	}
	dsim->res.regs = ioremap(res.start, resource_size(&res));
	if (!dsim->res.regs) {
		dsim_err(dsim, "failed to remap io region\n");
		ret = -ENOMEM;
		goto err;
	}
	dsim_regs_desc_init(dsim->res.regs, res.start, "dsi", REGS_DSIM_DSI, dsim->id);

	i = of_property_match_string(np, "reg-names", "dphy");
	if (of_address_to_resource(np, i, &res)) {
		pr_err("failed to get dphy resource\n");
		goto err;
	}

	dsim->res.phy_regs = ioremap(res.start, resource_size(&res));
	if (!dsim->res.phy_regs) {
		dsim_err(dsim, "failed to remap io region\n");
		ret = -ENOMEM;
		goto err_dsi;
	}
	dsim_regs_desc_init(dsim->res.phy_regs, res.start, "dphy", REGS_DSIM_PHY,
			dsim->id);

	i = of_property_match_string(np, "reg-names", "dphy-extra");
	if (of_address_to_resource(np, i, &res)) {
		pr_err("failed to get dphy resource\n");
		goto err_dsi;
	}
	dsim->res.phy_regs_ex = ioremap(res.start, resource_size(&res));
	if (!dsim->res.phy_regs_ex)
		dsim_warn(dsim, "failed to remap io region. it's optional\n");
	dsim_regs_desc_init(dsim->res.phy_regs_ex, res.start, "dphy-extra",
			REGS_DSIM_PHY_BIAS, dsim->id);

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9-disp_ss");
	i = of_property_match_string(np, "reg-names", "sys");
	if (of_address_to_resource(np, i, &res)) {
		dsim_err(dsim, "failed to get sys resource\n");
		goto err_dphy_ext;
	}
	dsim->res.ss_reg_base = ioremap(res.start, resource_size(&res));
	if (!dsim->res.ss_reg_base) {
		dsim_err(dsim, "failed to map sysreg-disp address.");
		ret = -ENOMEM;
		goto err_dphy_ext;
	}
	dsim_regs_desc_init(dsim->res.ss_reg_base, res.start, np->name, REGS_DSIM_SYS,
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


#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
static void dsim_underrun_info(struct dsim_device *dsim, u32 underrun_cnt)
{
	printk_ratelimited("underrun irq occurs(%u): MIF(%lu, %d), INT(%lu, %d), DISP(%lu, %d)\n",
			underrun_cnt,
			exynos_devfreq_get_domain_freq(DEVFREQ_MIF),
			exynos_pm_qos_request(PM_QOS_BUS_THROUGHPUT),
			exynos_devfreq_get_domain_freq(DEVFREQ_INT),
			exynos_pm_qos_request(PM_QOS_DEVICE_THROUGHPUT),
			exynos_devfreq_get_domain_freq(DEVFREQ_DISP),
			exynos_pm_qos_request(PM_QOS_DISPLAY_THROUGHPUT));
}
#else
static void dsim_underrun_info(struct dsim_device *dsim, u32 underrun_cnt)
{
	printk_ratelimited("underrun irq occurs(%u)\n", underrun_cnt);
}
#endif

static irqreturn_t dsim_irq_handler(int irq, void *dev_id)
{
	struct dsim_device *dsim = dev_id;
	struct decon_device *decon = (struct decon_device *)dsim_get_decon(dsim);
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
		complete(&dsim->ph_wr_comp);
		dsim_debug(dsim, "PH_FIFO_EMPTY irq occurs\n");
	}

	if (int_src & DSIM_INTSRC_SFR_PL_FIFO_EMPTY) {
		complete(&dsim->pl_wr_comp);
		dsim_debug(dsim, "PL_FIFO_EMPTY irq occurs\n");
	}

	if (int_src & DSIM_INTSRC_RX_DATA_DONE)
		complete(&dsim->rd_comp);
	if (int_src & DSIM_INTSRC_FRAME_DONE) {
		dsim_debug(dsim, "framedone irq occurs\n");
		if (decon)
			DPU_EVENT_LOG(DPU_EVT_DSIM_FRAMEDONE, decon->id, NULL);
	}

	if (int_src & DSIM_INTSRC_RX_CRC) {
		dsim_err(dsim, "RX CRC error was detected!\n");
		if (decon)
			DPU_EVENT_LOG(DPU_EVT_DSIM_CRC, decon->id, NULL);
	}

	if (int_src & DSIM_INTSRC_ERR_RX_ECC) {
		dsim_err(dsim, "RX ECC Multibit error was detected!\n");
		if (decon)
			DPU_EVENT_LOG(DPU_EVT_DSIM_ECC, decon->id, NULL);
	}

	if (int_src & DSIM_INTSRC_UNDER_RUN) {
		DPU_ATRACE_INT("DPU_UNDERRUN", 1);
		if (decon) {
			static unsigned long last_dumptime;

			dsim_underrun_info(dsim, decon->d.underrun_cnt + 1);
			DPU_EVENT_LOG(DPU_EVT_DSIM_UNDERRUN, decon->id, NULL);
			if (time_after(jiffies, last_dumptime + msecs_to_jiffies(5000))) {
				decon_dump_event_condition(decon, DPU_EVT_CONDITION_UNDERRUN);
				last_dumptime = jiffies;
			}
		} else {
			dsim_underrun_info(dsim, 0);
		}

		DPU_ATRACE_INT("DPU_UNDERRUN", 0);
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
	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return 0;

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

	if (!IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		ret = dsim_get_phys(dsim);
		if (ret)
			goto err;
	}

err:
	return ret;
}

static int dsim_host_attach(struct mipi_dsi_host *host, struct mipi_dsi_device *device)
{
	struct dsim_device *dsim = host_to_dsi(host);
	int ret;

	dsim_debug(dsim, "%s +\n", __func__);

	dsim->dsi_device = device;

	ret = sysfs_create_link(&device->dev.kobj, &host->dev->kobj, "dsim");
	if (ret)
		dev_warn(&device->dev, "unable to link %s sysfs (%d)\n", "dsim", ret);

	dsim_debug(dsim, "%s -\n", __func__);

	return component_add(dsim->dev, &dsim_component_ops);
}

static int dsim_host_detach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct dsim_device *dsim = host_to_dsi(host);

	dsim_info(dsim, "%s +\n", __func__);

	if (dsim->state != DSIM_STATE_HANDOVER)
		_dsim_disable(dsim);

	if (dsim->panel_bridge) {
		struct drm_bridge *bridge = dsim->panel_bridge;

		if (bridge->funcs && bridge->funcs->detach)
			bridge->funcs->detach(bridge);
		dsim->panel_bridge = NULL;
	}
	dsim->dsi_device = NULL;

	sysfs_remove_link(&device->dev.kobj, "dsim");
	dsim_info(dsim, "%s -\n", __func__);
	return 0;
}

static int __dsim_wait_for_ph_fifo_empty(struct dsim_device *dsim)
{
	const struct decon_device *decon = dsim_get_decon(dsim);

	if (dsim_reg_header_fifo_is_empty(dsim->id)) {
		dsim_debug(dsim, "no need to wait for packet header fifo empty\n");
		return 0;
	}

	dsim_debug(dsim, "wait for packet header fifo empty\n");

	if (!wait_for_completion_timeout(&dsim->ph_wr_comp, MIPI_WR_TIMEOUT)) {
		if (dsim_reg_header_fifo_is_empty(dsim->id)) {
			dsim_warn(dsim, "timed out but header fifo was empty\n");
			dsim_reg_clear_int(dsim->id,
					DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
			return 0;
		}

		dsim_warn(dsim, "timeout: packet header fifo empty\n");
		if (decon) {
			DPU_EVENT_LOG(DPU_EVT_DSIM_PH_FIFO_TIMEOUT, decon->id,
					NULL);
			decon_dump_event_condition(decon,
						DPU_EVT_CONDITION_FIFO_TIMEOUT);
		}
		return -ETIMEDOUT;
	}

	return 0;
}

static int __dsim_wait_for_pl_fifo_empty(struct dsim_device *dsim)
{
	const struct decon_device *decon = dsim_get_decon(dsim);

	if (dsim_reg_payload_fifo_is_empty(dsim->id)) {
		dsim_debug(dsim, "no need to wait for payload fifo empty\n");
		return 0;
	}

	dsim_debug(dsim, "wait for packet payload fifo empty\n");

	if (!wait_for_completion_timeout(&dsim->pl_wr_comp, MIPI_WR_TIMEOUT)) {
		if (dsim_reg_payload_fifo_is_empty(dsim->id)) {
			dsim_warn(dsim, "timed out but payload fifo was empty\n");
			dsim_reg_clear_int(dsim->id,
					DSIM_INTSRC_SFR_PL_FIFO_EMPTY);
			return 0;
		}

		dsim_warn(dsim, "timeout: packet payload fifo empty\n");
		if (decon) {
			DPU_EVENT_LOG(DPU_EVT_DSIM_PL_FIFO_TIMEOUT, decon->id,
					NULL);
			decon_dump_event_condition(decon,
						DPU_EVT_CONDITION_FIFO_TIMEOUT);
		}
		return -ETIMEDOUT;
	}

	return 0;
}

static int dsim_wait_for_cmd_fifo_empty(struct dsim_device *dsim, bool is_long)
{
	int ret = 0;

	DPU_ATRACE_BEGIN(__func__);
	if (is_long)
		ret = __dsim_wait_for_pl_fifo_empty(dsim);

	ret |= __dsim_wait_for_ph_fifo_empty(dsim);

	if (ret) {
		dsim_warn(dsim, "failed on wait for cmd fifo empty (%d)\n", ret);
		dsim_dump(dsim, NULL);
	}

	DPU_ATRACE_END(__func__);
	return ret;
}

static void
dsim_write_payload(struct dsim_device *dsim, const u8* buf, size_t len)
{
	const u8 *p = buf;
	const u8 *end = buf + len;
	u32 payload;

	dsim_debug(dsim, "payload length(%lu)\n", len);

	while (p < end) {
		size_t pkt_size = min_t(size_t, 4, end - p);

		if (pkt_size >= 4)
			payload = p[0] | p[1] << 8 | p[2] << 16 | (u32)p[3] << 24;
		else if (pkt_size == 3)
			payload = p[0] | p[1] << 8 | p[2] << 16;
		else if (pkt_size == 2)
			payload = p[0] | p[1] << 8;
		else if (pkt_size == 1)
			payload = p[0];

		dsim_reg_wr_tx_payload(dsim->id, payload);
		dsim_debug(dsim, "payload: 0x%x\n", payload);

		p += pkt_size;
	}
}

static void __dsim_cmd_write_locked(struct dsim_device *dsim, const struct mipi_dsi_packet *packet)
{
	WARN_ON(!mutex_is_locked(&dsim->cmd_lock));

	DPU_ATRACE_BEGIN(__func__);
	if (packet->payload_length > 0)
		dsim_write_payload(dsim, packet->payload, packet->payload_length);
	dsim_reg_wr_tx_header(dsim->id, packet->header[0], packet->header[1], packet->header[2],
			      false);

	dsim_debug(dsim, "header(0x%x 0x%x 0x%x) size(%lu) ph fifo(%d)\n", packet->header[0],
		   packet->header[1], packet->header[2], packet->size,
		   dsim_reg_get_ph_cnt(dsim->id));
	DPU_ATRACE_END(__func__);
}

static void dsim_cmd_packetgo_queue_locked(struct dsim_device *dsim,
					   const struct mipi_dsi_packet *packet)
{
	DPU_ATRACE_BEGIN(__func__);
	/* if this is the first packet being queued, enable packet go feature */
	if (!dsim->total_pend_ph)
		__dsim_cmd_packetgo_enable_locked(dsim, true);

	dsim->total_pend_ph++;
	dsim->total_pend_pl += ALIGN(packet->payload_length, 4);

	__dsim_cmd_write_locked(dsim, packet);

	dsim_debug(dsim, "total pending packet header(%u) payload(%u)\n", dsim->total_pend_ph,
		   dsim->total_pend_pl);
	DPU_ATRACE_END(__func__);
}

static void __dsim_cmd_prepare(struct dsim_device *dsim)
{
	WARN_ON(!mutex_is_locked(&dsim->cmd_lock));

	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY | DSIM_INTSRC_SFR_PL_FIFO_EMPTY);

	reinit_completion(&dsim->ph_wr_comp);
	reinit_completion(&dsim->pl_wr_comp);
}

static void dsim_cmd_packetgo_ready_locked(struct dsim_device *dsim)
{
	/* this should only be called with pending packets */
	WARN_ON(!dsim->total_pend_ph);

	__dsim_cmd_prepare(dsim);

	dsim_reg_ready_packetgo(dsim->id, true);
	dsim_debug(dsim, "packet go ready (ph: %d, pl: %d)\n", dsim->total_pend_ph,
		   dsim->total_pend_pl);
}

static void dsim_write_single_cmd_locked(struct dsim_device *dsim,
					const struct mipi_dsi_packet *packet)
{
	WARN_ON(dsim_cmd_packetgo_is_enabled(dsim));

	dsim->total_pend_pl = ALIGN(packet->payload_length, 4);

	__dsim_cmd_prepare(dsim);

	__dsim_cmd_write_locked(dsim, packet);
}

/*
 *		      <-- ACTIVE -->
 * data transfer ---|-**************------------------|--
 *				       <-CMD ALLOW->
 * CMD LOCK       __|-----------------|_____________|---
 *
 * TE PROTECT       <------- TE PROTECT ON -------->
 *
 * ready allow	    <---- ready allow ---->|<-1ms->|
 *
 * It is important to set packet-go ready to high to send multiple commnads
 * within one vblank interval at the same time. Because as soon as it becomes
 * high, the piled commands will start transmission. The ready_allow_period is
 * defined so that the stacked commands would not be sent in two vblank
 * intervals. The ready_allow_period is set from start of vblank to end of
 * CMD ALLOW period - 1ms. The 1ms is enough time to send the stacked commands
 * at once.
 */
#define PKTGO_READY_MARGIN_NS	1000000
static void need_wait_vblank(struct dsim_device *dsim)
{
	const struct decon_device *decon = dsim_get_decon(dsim);
	struct drm_vblank_crtc *vblank;
	struct drm_crtc *crtc;
	ktime_t last_vblanktime, diff, cur_time;
	int ready_allow_period;

	if (!decon)
		return;

	crtc = &decon->crtc->base;
	if (!crtc)
		return;

	if (drm_crtc_vblank_get(crtc))
		return;

	vblank = &crtc->dev->vblank[crtc->index];
	ready_allow_period =
		mult_frac(vblank->framedur_ns, 95, 100) - PKTGO_READY_MARGIN_NS;

	drm_crtc_vblank_count_and_time(crtc, &last_vblanktime);
	cur_time = ktime_get();
	diff = ktime_sub_ns(cur_time, last_vblanktime);

	dsim_debug(dsim, "last(%lld) cur(%lld) diff(%lld) ready allow period(%d)\n",
			last_vblanktime, cur_time, diff, ready_allow_period);

	if (diff > ready_allow_period) {
		DPU_ATRACE_BEGIN("dsim_pktgo_wait_vblank");
		drm_crtc_wait_one_vblank(crtc);
		DPU_ATRACE_END("dsim_pktgo_wait_vblank");
	}
	drm_crtc_vblank_put(crtc);
}

#if IS_ENABLED(CONFIG_DSIM_LOGBUFF)
static void dsim_dump_cmd(struct dsim_device *dsim, const struct mipi_dsi_msg *msg, bool is_last)
{
	int tx_len = msg->tx_len;
	const u8 *tx_buf = msg->tx_buf;

	logbuffer_log(dsim->log, "last=%d delay=%d tx=", is_last, dsim->tx_delay_ms);
	while (tx_len > 0) {
		/* "%*phN" limits up to 64 bytes */
		int sz = min(tx_len, 64);

		logbuffer_log(dsim->log, "%*phN", sz, tx_buf);
		tx_buf += sz;
		tx_len -= sz;
	}
}
#endif

#define PL_FIFO_THRESHOLD	mult_frac(MAX_PL_FIFO, 75, 100) /* 75% */
#define IS_LAST(flags)		(((flags) & EXYNOS_DSI_MSG_QUEUE) == 0)
static int dsim_write_data_locked(struct dsim_device *dsim, const struct mipi_dsi_msg *msg)
{
	int ret = 0;
	const u16 flags = msg->flags;
	bool is_last = false;
	struct mipi_dsi_packet packet = { .size = 0 };

#if IS_ENABLED(CONFIG_QUEUE_DDIC_CMD_CALL_BACK)
	const struct drm_bridge *bridge = dsim->panel_bridge;
	struct exynos_panel *panel = bridge ?
		container_of((bridge), struct exynos_panel, bridge) : NULL;
	const struct exynos_panel_funcs *funcs = (panel && panel->desc) ?
		panel->desc->exynos_panel_func : NULL;
#endif

	WARN_ON(!mutex_is_locked(&dsim->cmd_lock));

	if (msg->tx_len > 0) {
		const u8 *tx_buf = msg->tx_buf;

		ret = mipi_dsi_create_packet(&packet, msg);
		if (ret) {
			dsim_err(dsim, "unable to create dsi packet (%d)\n", ret);
			return 0;
		}

		DPU_EVENT_LOG_CMD(dsim, msg->type, tx_buf[0], msg->tx_len);
	}
	DPU_ATRACE_BEGIN(__func__);

	if (dsim->config.mode == DSIM_VIDEO_MODE) {
		is_last = true;
		if (flags & (EXYNOS_DSI_MSG_FORCE_BATCH | EXYNOS_DSI_MSG_FORCE_FLUSH))
			dsim_warn(dsim, "force batching is attempted in video mode\n");
		if (packet.size)
			dsim_write_single_cmd_locked(dsim, &packet);
		goto trace_dsi_cmd;
	}

	if (flags & EXYNOS_DSI_MSG_FORCE_BATCH) {
		WARN_ON(dsim->force_batching);
		dsim->force_batching = true;
		goto err;
	}

	if (((dsim->total_pend_pl + packet.payload_length) > MAX_PL_FIFO) ||
	    (dsim->total_pend_ph >= MAX_PH_FIFO)) {
		dsim_err(dsim, "fifo would be full. ph(%u) pl(%lu) max(%d/%d)\n",
				dsim->total_pend_ph,
				dsim->total_pend_pl + msg->tx_len,
				MAX_PH_FIFO, MAX_PL_FIFO);
		ret = -EINVAL;
		goto err;
	}

	is_last = (IS_LAST(flags) && !dsim->force_batching) || (flags & EXYNOS_DSI_MSG_FORCE_FLUSH);

	if (flags & EXYNOS_DSI_MSG_FORCE_FLUSH) {
		dsim->force_batching = false;
		/* force batching should happen only with empty msg */
		WARN_ON(packet.size);
	}

	if (!is_last && packet.size &&
	    (((dsim->total_pend_ph + 1) >= MAX_PH_FIFO) ||
	     ((dsim->total_pend_pl + packet.payload_length) > PL_FIFO_THRESHOLD))) {
		dsim_warn(dsim, "warning. changed last command. pend pl/pl(%u,%u) new pl(%zu)\n",
			  dsim->total_pend_ph, dsim->total_pend_pl, packet.payload_length);
		is_last = true;
	}

	if (is_last) {
		if (dsim_cmd_packetgo_is_enabled(dsim)) {
			if (packet.size > 0)
				dsim_cmd_packetgo_queue_locked(dsim, &packet);

			if (!(flags & EXYNOS_DSI_MSG_IGNORE_VBLANK))
				need_wait_vblank(dsim);

			dsim_cmd_packetgo_ready_locked(dsim);
		} else if (packet.size > 0) {
			dsim_write_single_cmd_locked(dsim, &packet);
		}
	} else if (packet.size > 0) {
		dsim_cmd_packetgo_queue_locked(dsim, &packet);
	}

trace_dsi_cmd:
#if IS_ENABLED(CONFIG_DSIM_LOGBUFF)
	DPU_ATRACE_BEGIN("dsim_dump_cmd");
	dsim_dump_cmd(dsim, msg, is_last);
	DPU_ATRACE_END("dsim_dump_cmd");
#endif
	dsim_debug(dsim, "%s last command\n", is_last ? "" : "Not");
#if IS_ENABLED(CONFIG_QUEUE_DDIC_CMD_CALL_BACK)
	if (funcs && funcs->on_queue_ddic_cmd)
		funcs->on_queue_ddic_cmd(panel, msg, is_last);
#endif
err:
	trace_dsi_cmd_fifo_status(dsim->total_pend_ph, dsim->total_pend_pl);
	DPU_ATRACE_END(__func__);
	return ret ? : is_last;
}

static int
dsim_req_read_command(struct dsim_device *dsim, const struct mipi_dsi_msg *msg)
{
	struct mipi_dsi_packet packet;
	const u8 rx_len = msg->rx_len & 0xff;

	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
	reinit_completion(&dsim->ph_wr_comp);
	trace_dsi_tx(MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, &rx_len, 1, true, 0);
	/* set the maximum packet size returned */
	dsim_reg_wr_tx_header(dsim->id, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
			msg->rx_len, 0, false);

	/* read request */
	mipi_dsi_create_packet(&packet, msg);
	trace_dsi_tx(msg->type, msg->tx_buf, msg->tx_len, true, 0);
	dsim_reg_wr_tx_header(dsim->id, packet.header[0], packet.header[1],
						packet.header[2], true);

	return dsim_wait_for_cmd_fifo_empty(dsim, false);
}

static int
dsim_read_data(struct dsim_device *dsim, const struct mipi_dsi_msg *msg)
{
	u32 rx_fifo, rx_size = 0;
	int i = 0, ret = 0;
	u8 *rx_buf = msg->rx_buf;
	const u8 *tx_buf = msg->tx_buf;

	if (msg->rx_len > MAX_RX_FIFO) {
		dsim_err(dsim, "invalid rx len(%lu) max(%d)\n", msg->rx_len,
				MAX_RX_FIFO);
		return -EINVAL;
	}

	/* Init RX FIFO before read and clear DSIM_INTSRC */
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_RX_DATA_DONE);

	reinit_completion(&dsim->rd_comp);

	ret = dsim_req_read_command(dsim, msg);
	if (ret) {
		dsim_err(dsim, "failed to request dsi read command\n");
		return ret;
	}

	if (!wait_for_completion_timeout(&dsim->rd_comp, MIPI_RD_TIMEOUT)) {
		dsim_err(dsim, "read timeout\n");
		return -ETIMEDOUT;
	}

	rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
	dsim_debug(dsim, "rx fifo:0x%8x, response:0x%x, rx_len:%lu\n", rx_fifo,
		 rx_fifo & 0xff, msg->rx_len);

	/* Parse the RX packet data types */
	switch (rx_fifo & 0xff) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		ret = dsim_reg_rx_err_handler(dsim->id, rx_fifo);
		if (ret < 0) {
			dsim_dump(dsim, NULL);
			return ret;
		}
		break;
	case MIPI_DSI_RX_END_OF_TRANSMISSION:
		dsim_debug(dsim, "EoTp was received\n");
		break;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		WARN_ON(msg->rx_len > 2);
		rx_buf[1] = (rx_fifo >> 16) & 0xff;
		fallthrough;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		rx_buf[0] = (rx_fifo >> 8) & 0xff;
		dsim_debug(dsim, "short packet was received\n");
		rx_size = msg->rx_len;
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
				rx_buf[i] = rx_fifo & 0xff;
		}
		break;
	default:
		dsim_err(dsim, "packet format is invalid.\n");
		dsim_dump(dsim, NULL);
		return -EBUSY;
	}

	if (!dsim_reg_rx_fifo_is_empty(dsim->id)) {
		u32 retry_cnt = RETRY_READ_FIFO_MAX;

		dsim_warn(dsim, "RX FIFO is not empty: rx_size:%u, rx_len:%lu\n",
			rx_size, msg->rx_len);
		dsim_dump(dsim, NULL);
		do {
			rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
			dsim_info(dsim, "rx fifo:0x%8x, response:0x%x\n",
				rx_fifo, rx_fifo & 0xff);
		} while (!dsim_reg_rx_fifo_is_empty(dsim->id) && --retry_cnt);
	}

	trace_dsi_rx(tx_buf[0], rx_buf, rx_size);
	return rx_size;
}

static int dsim_cmd_flush_locked(struct dsim_device *dsim)
{
	int ret;

	ret = dsim_wait_for_cmd_fifo_empty(dsim, dsim->total_pend_pl > 0);
	if (dsim_cmd_packetgo_is_enabled(dsim))
		__dsim_cmd_packetgo_enable_locked(dsim, false);

	return ret;
}

static int
dsim_write_data_dual(struct dsim_device *dsim, const struct mipi_dsi_msg *msg)
{
	int ret;

	ret = pm_runtime_resume_and_get(dsim->dev);
	if (ret) {
		dsim_err(dsim, "runtime resume failed (%d). unable to transfer cmd\n", ret);
		return ret;
	}

	mutex_lock(&dsim->cmd_lock);

	ret = dsim_write_data_locked(dsim, msg);
	if (ret == 1) // is_last
		ret = dsim_cmd_flush_locked(dsim);

	mutex_unlock(&dsim->cmd_lock);

	pm_runtime_mark_last_busy(dsim->dev);
	pm_runtime_put_sync_autosuspend(dsim->dev);

	return ret;
}

static ssize_t dsim_host_transfer(struct mipi_dsi_host *host,
			    const struct mipi_dsi_msg *msg)
{
	struct dsim_device *dsim = host_to_dsi(host);
	struct dsim_device *sec_dsi;
	int ret;

	DPU_ATRACE_BEGIN(__func__);

	ret = pm_runtime_resume_and_get(dsim->dev);
	if (ret) {
		dsim_err(dsim, "runtime resume failed (%d). unable to transfer cmd\n", ret);
		return ret;
	}

	mutex_lock(&dsim->cmd_lock);
	if (WARN_ON(dsim->state != DSIM_STATE_HSCLKEN)) {
		ret = -EPERM;
		goto abort;
	}

	switch (msg->type) {
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		ret = dsim_read_data(dsim, msg);
		break;
	default:
		ret = dsim_write_data_locked(dsim, msg);
		if (dsim->dual_dsi == DSIM_DUAL_DSI_MAIN) {
			sec_dsi = exynos_get_dual_dsi(DSIM_DUAL_DSI_SEC);
			if (sec_dsi)
				dsim_write_data_dual(sec_dsi, msg);
			else
				dsim_err(dsim, "could not get secondary dsi\n");
		}
		if (ret == 1) // is_last
			ret = dsim_cmd_flush_locked(dsim);
		break;
	}

abort:
	mutex_unlock(&dsim->cmd_lock);

	pm_runtime_mark_last_busy(dsim->dev);
	pm_runtime_put_sync_autosuspend(dsim->dev);

	DPU_ATRACE_END(__func__);

	return ret;
}

/* TODO: Below operation will be registered after panel driver is created. */
static const struct mipi_dsi_host_ops dsim_host_ops = {
	.attach = dsim_host_attach,
	.detach = dsim_host_detach,
	.transfer = dsim_host_transfer,
};

static int dsim_calc_pmsk(struct dsim_pll_features *pll_features,
			 struct stdphy_pms *pms, unsigned int hs_clock_mhz)
{
	uint64_t hs_clock;
	uint64_t fvco, q;
	uint32_t p, m, s, k;

	p = DIV_ROUND_CLOSEST(pll_features->finput, pll_features->foptimum);
	if (p == 0)
		p = 1;
	if ((p < pll_features->p_min) || (p > pll_features->p_max)) {
		pr_err("%s: p %u is out of range (%u, %u)\n",
		       __func__, p, pll_features->p_min, pll_features->p_max);
		return -EINVAL;
	}

	hs_clock = (uint64_t) hs_clock_mhz * 1000000;
	if ((hs_clock < pll_features->fout_min) ||
			(hs_clock > pll_features->fout_max)) {
		pr_err("%s: hs clock %llu out of range\n", __func__, hs_clock);
		return -EINVAL;
	}

	/* find s: vco_min <= fout * 2 ^ s <= vco_max */
	for (s = 0, fvco = 0; fvco < pll_features->fvco_min; s++)
		fvco = hs_clock * (1 << s);
	--s;

	if (fvco > pll_features->fvco_max) {
		pr_err("%s: no proper s found\n", __func__);
		return -EINVAL;
	}
	if ((s < pll_features->s_min) || (s > pll_features->s_max)) {
		pr_err("%s: s %u is out of range (%u, %u)\n",
		       __func__, s, pll_features->s_min, pll_features->s_max);
		return -EINVAL;
	}

	/* (hs_clk * 2^s / 2) / (fin / p) = m + k / 2^k_bits */
	fvco >>= 1;
	q = fvco << (pll_features->k_bits + 1); /* 1 extra bit for roundup */
	q /= pll_features->finput / p;

	/* m is the integer part, k is the fraction part */
	m = q >> (pll_features->k_bits + 1);
	if ((m < pll_features->m_min) || (m > pll_features->m_max)) {
		pr_err("%s: m %u is out of range (%u, %u)\n",
		       __func__, m, pll_features->m_min, pll_features->m_max);
		return -EINVAL;
	}

	k = q & ((1 << (pll_features->k_bits + 1)) - 1);
	k = DIV_ROUND_UP(k, 2);

	/* k is two's complement integer */
	if (k & (1 << (pll_features->k_bits - 1)))
		m++;

	pms->p = p;
	pms->m = m;
	pms->s = s;
	pms->k = k;

	return 0;
}

static int dsim_calc_underrun(const struct dsim_device *dsim, uint32_t hs_clock_mhz,
		uint32_t *underrun)
{
	const struct dsim_reg_config *config = &dsim->config;
	uint32_t lanes = config->data_lane_cnt;
	uint32_t number_of_transfer;
	uint32_t w_threshold;
	uint64_t wclk;
	uint64_t max_frame_time;
	uint64_t frame_data;
	uint64_t packet_header;
	uint64_t min_frame_transfer_time;
	uint64_t max_lp_time;

	number_of_transfer = config->p_timing.vactive;
	w_threshold = config->p_timing.hactive;
	if (config->dsc.enabled)
		w_threshold /= 3;
	wclk = (uint64_t) hs_clock_mhz * 1000000 / 16;

	/* max time to transfer one frame, in the unit of nanosecond */
	max_frame_time = NSEC_PER_SEC * 100 /
		(config->p_timing.vrefresh * (100 + config->p_timing.te_var)) -
		NSEC_PER_USEC * config->p_timing.te_idle_us;
	/* one frame pixel data (bytes) */
	frame_data = number_of_transfer * w_threshold * config->bpp / 8;
	/* packet header (bytes) */
	packet_header = number_of_transfer * 7;
	/* minimum time to transfer one frame, in nanosecond */
	min_frame_transfer_time = (frame_data + packet_header) *
					NSEC_PER_SEC / (2 * lanes * wclk);

	if (max_frame_time < min_frame_transfer_time) {
		pr_err("%s: max frame time %llu < min frame time %llu\n",
			__func__, max_frame_time, min_frame_transfer_time);
		return -EINVAL;
	}

	max_lp_time = max_frame_time - min_frame_transfer_time;
	/* underrun unit is 100 wclk, round up */
	*underrun = (uint32_t) DIV_ROUND_UP(max_lp_time * wclk / NSEC_PER_SEC, 100);

	return 0;
}

static int dsim_set_hs_clock(struct dsim_device *dsim, unsigned int hs_clock, bool apply_now)
{
	int ret;
	struct stdphy_pms pms;
	uint32_t lp_underrun = 0;
	struct dsim_pll_param *pll_param;

	if (!dsim->pll_params || !dsim->pll_params->features)
		return -ENODEV;

	memset(&pms, 0, sizeof(pms));
	ret = dsim_calc_pmsk(dsim->pll_params->features, &pms, hs_clock);
	if (ret < 0) {
		dsim_err(dsim, "Failed to update pll for hsclk %u\n", hs_clock);
		return -EINVAL;
	}

	mutex_lock(&dsim->state_lock);
	ret = dsim_calc_underrun(dsim, hs_clock, &lp_underrun);
	if (ret < 0) {
		dsim_err(dsim, "Failed to update underrun\n");
		goto out;
	}

	pll_param = dsim->current_pll_param;
	if (!pll_param) {
		ret = -EAGAIN;
		goto out;
	}

	if (hs_clock == pll_param->pll_freq) {
		dsim_debug(dsim, "the same hs_clock=%u\n", hs_clock);
		goto out;
	}

	pll_param->pll_freq = hs_clock;
	pll_param->p = pms.p;
	pll_param->m = pms.m;
	pll_param->s = pms.s;
	pll_param->k = pms.k;
	pll_param->cmd_underrun_cnt = lp_underrun;
	dsim_update_clock_config(dsim, pll_param);

	if (!apply_now || dsim->state != DSIM_STATE_HSCLKEN)
		goto out;

	/* Restart dsim to apply new clock settings */
	dsim_restart(dsim);
out:
	mutex_unlock(&dsim->state_lock);

	return ret;
}

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
		_dsim_enable(dsim);

	dsim_reg_set_bist(dsim->id, bist_en, bist_mode - 1);
	dsim->bist_mode = bist_mode;

	if (!bist_en && dsim->state == DSIM_STATE_HSCLKEN)
		_dsim_disable(dsim);

	dsim_info(dsim, "0:Disable 1:ColorBar 2:GRAY Gradient 3:UserDefined\n");
	dsim_info(dsim, "4:Prbs7 Random (%d)\n", dsim->bist_mode);

	return len;
}
static DEVICE_ATTR_RW(bist_mode);

static ssize_t hs_clock_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dsim->clk_param.hs_clk);
}

static ssize_t hs_clock_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	struct dsim_pll_param *pll_param;
	int rc;
	unsigned int hs_clock;
	bool apply_now = false;

	char params[32];
	char *hs_clk_str;
	char *apply_now_str;
	char *p = params;

	mutex_lock(&dsim->state_lock);
	pll_param = dsim->current_pll_param;
	if (unlikely(!pll_param)) {
		dsim_warn(dsim, "%s: unable to get pll param\n", __func__);
		rc = -EAGAIN;
		goto out;
	}

	strlcpy(params, buf, sizeof(params));
	hs_clk_str = strsep(&p, " ");
	apply_now_str = strsep(&p, " ");

	if (apply_now_str) {
		rc = kstrtobool(apply_now_str, &apply_now);
		if (rc < 0)
			goto out;
	}

	rc = kstrtouint(hs_clk_str, 0, &hs_clock);
	if (rc < 0)
		goto out;

	/* ddr hs_clock unit: MHz */
	dsim_info(dsim, "%s: hs clock %u, apply now: %u\n", __func__, hs_clock, apply_now);

	if (dsim->allowed_hs_clks && dsim->allowed_hs_clks->hs_clks && !dsim->force_set_hs_clk) {
		bool hs_clock_allowed = false;

		for (int i = 0; i < dsim->allowed_hs_clks->num_clks; i++) {
			if (hs_clock == dsim->allowed_hs_clks->hs_clks[i]) {
				hs_clock_allowed = true;
				break;
			}
		}

		if (!hs_clock_allowed) {
			dsim_warn(dsim, "hs_clock=%u not in allowed_hs_clks\n", hs_clock);
			rc = -EINVAL;
			goto out;
		}
	}

	if (dsim->state != DSIM_STATE_HSCLKEN) {
		if (pll_param->pll_freq == hs_clock) {
			dsim_debug(dsim, "set the same hs_clock=%u while idle\n", hs_clock);
			dsim->clk_param.pending_hs_clk = 0;
		} else {
			dsim_info(dsim, "not in HS state, set pending_hs_clk=%u\n", hs_clock);
			dsim->clk_param.pending_hs_clk = hs_clock;
		}
		rc = len;
		goto out;
	} else {
		dsim->clk_param.pending_hs_clk = 0;
		if (pll_param->pll_freq == hs_clock) {
			dsim_debug(dsim, "set the same hs_clock=%u while active\n", hs_clock);
			dsim->clk_param.pending_hs_clk = 0;
			rc = len;
			goto out;
		}
	}
	mutex_unlock(&dsim->state_lock);

	rc = dsim_set_hs_clock(dsim, hs_clock, apply_now);
	if (rc < 0)
		return rc;

	return len;

out:
	mutex_unlock(&dsim->state_lock);
	return rc;
}
static DEVICE_ATTR_RW(hs_clock);

static int dsim_get_pinctrl(struct dsim_device *dsim)
{
	int ret = 0;

	dsim->pinctrl = devm_pinctrl_get(dsim->dev);
	if (IS_ERR(dsim->pinctrl)) {
		ret = PTR_ERR(dsim->pinctrl);
		dsim_debug(dsim, "failed to get pinctrl (%d)\n", ret);
		dsim->pinctrl = NULL;
		/* optional in video mode */
		return 0;
	}

	dsim->te_on = pinctrl_lookup_state(dsim->pinctrl, "hw_te_on");
	if (IS_ERR(dsim->te_on)) {
		dsim_err(dsim, "failed to get hw_te_on pin state\n");
		ret = PTR_ERR(dsim->te_on);
		dsim->te_on = NULL;
		goto err;
	}
	dsim->te_off = pinctrl_lookup_state(dsim->pinctrl, "hw_te_off");
	if (IS_ERR(dsim->te_off)) {
		dsim_err(dsim, "failed to get hw_te_off pin state\n");
		ret = PTR_ERR(dsim->te_off);
		dsim->te_off = NULL;
		goto err;
	}

err:
	return ret;
}

static int dsim_probe(struct platform_device *pdev)
{
	struct dsim_device *dsim;
	int ret;

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
	mutex_init(&dsim->state_lock);
	init_completion(&dsim->ph_wr_comp);
	init_completion(&dsim->pl_wr_comp);
	init_completion(&dsim->rd_comp);

	ret = dsim_init_resources(dsim);
	if (ret)
		goto err;

	if (!IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		ret = dsim_get_pinctrl(dsim);
		if (ret)
			goto err;
	}

	ret = device_create_file(dsim->dev, &dev_attr_bist_mode);
	if (ret < 0)
		dsim_err(dsim, "failed to add sysfs bist_mode entries\n");

	ret = device_create_file(dsim->dev, &dev_attr_hs_clock);
	if (ret < 0)
		dsim_err(dsim, "failed to add sysfs hs_clock entries\n");

	platform_set_drvdata(pdev, &dsim->encoder);

#if defined(CONFIG_CPU_IDLE)
	dsim->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
	dsim_info(dsim, "dsim idle_ip_index[%d]\n", dsim->idle_ip_index);
	if (dsim->idle_ip_index < 0)
		dsim_warn(dsim, "idle ip index is not provided\n");
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	/* parse the panel name to select the dsi device for the detected panel */
	ret = dsim_parse_panel_name(dsim);
	if (IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		dsim->state = DSIM_STATE_SUSPEND;
	} else if (ret) {
		dsim->state = DSIM_STATE_MISSING;
	} else {
		dsim->state = DSIM_STATE_HANDOVER;

		// TODO: get which panel is active from bootloader?

		pm_runtime_use_autosuspend(dsim->dev);
		pm_runtime_set_autosuspend_delay(dsim->dev, 20);
		pm_runtime_enable(dsim->dev);

		if (!IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
			phy_init(dsim->res.phy);
			if (dsim->res.phy_ex)
				phy_init(dsim->res.phy_ex);
		}
	}

	if (dsim->state == DSIM_STATE_MISSING || dsim->dual_dsi == DSIM_DUAL_DSI_SEC) {
		dsim->dsi_device = ERR_PTR(-ENODEV);

		ret = component_add(dsim->dev, &dsim_component_ops);
		if (ret < 0) {
			dsim_err(dsim, "unable to add dsim component\n");
			goto err;
		}
	} else {
		ret = mipi_dsi_host_register(&dsim->dsi_host);
		if (ret) {
			dsim_err(dsim, "unable to register dsi host\n");
			goto err;
		}
	}
#if IS_ENABLED(CONFIG_DSIM_LOGBUFF)
	if (dsim->id == 0)
		dsim->log = logbuffer_register("dsim0");
	else if (dsim->id == 1)
		dsim->log = logbuffer_register("dsim1");
	else
		dev_err(dsim->dev, "invalid dsim_id=%d for logbuffer registry\n", dsim->id);
	if (IS_ERR_OR_NULL(dsim->log)) {
		dev_err(dsim->dev, "logbuffer register failed\n");
		dsim->log = NULL;
	}
#endif
	dsim_info(dsim, "driver has been probed.\n");
	return 0;

err:
	dsim_err(dsim, "failed to probe exynos dsim driver\n");
	return ret;
}

static int dsim_remove(struct platform_device *pdev)
{
	struct dsim_device *dsim = platform_get_drvdata(pdev);

	device_remove_file(dsim->dev, &dev_attr_bist_mode);
	device_remove_file(dsim->dev, &dev_attr_hs_clock);
	pm_runtime_disable(&pdev->dev);

	if (dsim->state == DSIM_STATE_MISSING || dsim->dual_dsi == DSIM_DUAL_DSI_SEC) {
		component_del(&pdev->dev, &dsim_component_ops);
	} else {
		if (!IS_ERR_OR_NULL(dsim->dsi_device))
			component_del(&pdev->dev, &dsim_component_ops);
		mipi_dsi_host_unregister(&dsim->dsi_host);
	}

	iounmap(dsim->res.ss_reg_base);
	iounmap(dsim->res.phy_regs_ex);
	iounmap(dsim->res.phy_regs);
	iounmap(dsim->res.regs);
#if IS_ENABLED(CONFIG_DSIM_LOGBUFF)
	if (dsim->log) {
		logbuffer_unregister(dsim->log);
		dsim->log = NULL;
	}
#endif
	return 0;
}

#ifdef CONFIG_PM
static int dsim_runtime_suspend(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	const struct decon_device *decon = dsim_get_decon(dsim);

	DPU_ATRACE_BEGIN(__func__);

	mutex_lock(&dsim->state_lock);

	dsim_debug(dsim, "state: %d\n", dsim->state);
	if (dsim->state == DSIM_STATE_HSCLKEN)
		_dsim_enter_ulps_locked(dsim);

	dsim->suspend_state = dsim->state;
	mutex_unlock(&dsim->state_lock);
	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_RUNTIME_SUSPEND, decon->id, dsim);
	DPU_ATRACE_END(__func__);

	return 0;
}

static int dsim_runtime_resume(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	const struct decon_device *decon = dsim_get_decon(dsim);
	int ret = 0;

	DPU_ATRACE_BEGIN(__func__);

	mutex_lock(&dsim->state_lock);
	dsim_debug(dsim, "state: %d\n", dsim->state);

	if (dsim->state == DSIM_STATE_BYPASS)
		ret = -EPERM;
	else if (dsim->state == DSIM_STATE_ULPS)
		_dsim_exit_ulps_locked(dsim);

	dsim->suspend_state = dsim->state;
	mutex_unlock(&dsim->state_lock);

	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_RUNTIME_RESUME, decon->id, dsim);
	DPU_ATRACE_END(__func__);

	return ret;
}

static int dsim_suspend(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	const struct decon_device *decon = dsim_get_decon(dsim);

	mutex_lock(&dsim->state_lock);
	dsim->suspend_state = dsim->state;

	if (dsim->state == DSIM_STATE_HSCLKEN) {
		_dsim_enter_ulps_locked(dsim);
		dev->power.must_resume = true;
	}

	dsim_debug(dsim, "-\n");

	mutex_unlock(&dsim->state_lock);

	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_SUSPEND, decon->id, dsim);

	return 0;
}

static int dsim_resume(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	const struct decon_device *decon = dsim_get_decon(dsim);

	mutex_lock(&dsim->state_lock);
	if (dsim->suspend_state == DSIM_STATE_HSCLKEN)
		_dsim_exit_ulps_locked(dsim);

	dsim_debug(dsim, "-\n");

	mutex_unlock(&dsim->state_lock);

	if (decon)
		DPU_EVENT_LOG(DPU_EVT_DSIM_RESUME, decon->id, dsim);

	return 0;
}
#endif

static const struct dev_pm_ops dsim_pm_ops = {
	SET_RUNTIME_PM_OPS(dsim_runtime_suspend, dsim_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(dsim_suspend, dsim_resume)
};

struct platform_driver dsim_driver = {
	.probe = dsim_probe,
	.remove = dsim_remove,
	.driver = {
		   .name = "exynos-dsim",
		   .owner = THIS_MODULE,
		   .of_match_table = dsim_of_match,
		   .pm = &dsim_pm_ops,
	},
};

#ifndef CONFIG_BOARD_EMULATOR
MODULE_SOFTDEP("pre: phy-exynos-mipi");
#endif
MODULE_AUTHOR("Donghwa Lee <dh09.lee@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC MIPI DSI Master");
MODULE_LICENSE("GPL v2");
