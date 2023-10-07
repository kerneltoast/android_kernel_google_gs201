/* SPDX-License-Identifier: GPL-2.0-only
 *
 * linux/drivers/gpu/drm/samsung/exynos_drm_dsim.h
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Headef file for Samsung MIPI DSI Master driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_DSI_H__
#define __EXYNOS_DRM_DSI_H__

/* Add header */
#include <drm/drm_encoder.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_property.h>
#include <drm/drm_panel.h>
#include <video/videomode.h>

#include <dsim_cal.h>

#include "exynos_drm_drv.h"

enum dsim_state {
	DSIM_STATE_HSCLKEN,     /* dsim fully active */
	DSIM_STATE_ULPS,	/* low power state */
	DSIM_STATE_SUSPEND,	/* inactive */
	DSIM_STATE_BYPASS,	/* bypass mode, dsim shouldn't be used */
	DSIM_STATE_HANDOVER,
};

enum dsim_dual_dsi {
	DSIM_DUAL_DSI_NONE,
	DSIM_DUAL_DSI_MAIN,
	DSIM_DUAL_DSI_SEC,
};

struct dsim_pll_features {
	u64 finput;
	u64 foptimum;
	u64 fout_min, fout_max;
	u64 fvco_min, fvco_max;
	u32 p_min, p_max;
	u32 m_min, m_max;
	u32 s_min, s_max;
	u32 k_bits;
};

struct dsim_pll_params {
	unsigned int num_modes;
	struct dsim_pll_param **params;
	struct dsim_pll_features *features;
};

struct dsim_resources {
	void __iomem *regs;
	void __iomem *phy_regs;
	void __iomem *phy_regs_ex;
	void __iomem *ss_reg_base;
	struct phy *phy;
	struct phy *phy_ex;
};

struct dsim_device {
	struct drm_encoder encoder;
	struct mipi_dsi_host dsi_host;
	struct device *dev;
	struct drm_bridge *panel_bridge;
	struct mipi_dsi_device *dsi_device;
	struct device_link *dev_link;

	enum exynos_drm_output_type output_type;
	int te_from;
	int te_gpio;
	int err_fg_gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *te_on;
	struct pinctrl_state *te_off;
	bool hw_trigger;

	struct dsim_resources res;
	struct clk **clks;
	struct dsim_pll_params *pll_params;

#ifdef CONFIG_DEBUG_FS
        struct dentry *debugfs_entry;
#endif

	int irq;
	int irq_err_fg;
	int id;
	spinlock_t slock;
	struct mutex cmd_lock;
	struct mutex state_lock;
	struct completion ph_wr_comp;
	struct completion pl_wr_comp;
	struct completion rd_comp;

	enum dsim_state state;
	enum dsim_state suspend_state;

	/* set bist mode by sysfs */
	unsigned int bist_mode;

	/* FIXME: dsim cal structure */
	struct dsim_reg_config config;
	struct dsim_clks clk_param;

	struct dsim_pll_param *current_pll_param;

	int idle_ip_index;
	u8 total_pend_ph;
	u16 total_pend_pl;
	/* override message flag MIPI_DSI_MSG_LASTCOMMAND */
	bool force_batching;

	enum dsim_dual_dsi dual_dsi;
};

extern struct dsim_device *dsim_drvdata[MAX_DSI_CNT];

#define encoder_to_dsim(e) container_of(e, struct dsim_device, encoder)

#define MIPI_WR_TIMEOUT				msecs_to_jiffies(80)
#define MIPI_RD_TIMEOUT				msecs_to_jiffies(100)

struct decon_device;

static inline struct dsim_device *
exynos_get_dual_dsi(enum dsim_dual_dsi dual_dsi)
{
	int i;
	struct dsim_device *dsim = NULL;

	for (i = 0; i < MAX_DSI_CNT; i++) {
		dsim = dsim_drvdata[i];
		if (dsim->dual_dsi == dual_dsi)
			return dsim;
	}

	return NULL;
}

static inline const struct decon_device *
dsim_get_decon(const struct dsim_device *dsim)
{
	const struct drm_crtc *crtc = dsim->encoder.crtc;
	struct dsim_device *main_dsi;

	if (dsim->dual_dsi == DSIM_DUAL_DSI_SEC) {
		main_dsi = exynos_get_dual_dsi(DSIM_DUAL_DSI_MAIN);
		crtc = main_dsi->encoder.crtc;
	}

	if (!crtc)
		return NULL;

	return to_exynos_crtc(crtc)->ctx;
}

#ifdef CONFIG_DEBUG_FS
void dsim_diag_create_debugfs(struct dsim_device *dsim);
void dsim_diag_remove_debugfs(struct dsim_device *dsim);

int dsim_dphy_diag_get_reg(struct dsim_device *dsim,
                           struct dsim_dphy_diag *diag, uint32_t *vals);
int dsim_dphy_diag_set_reg(struct dsim_device *dsim,
                           struct dsim_dphy_diag *diag, uint32_t val);
#endif

#endif /* __EXYNOS_DRM_DSI_H__ */
