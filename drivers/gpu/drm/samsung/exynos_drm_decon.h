/* SPDX-License-Identifier: GPL-2.0-only
 *
 * linux/drivers/gpu/drm/samsung/exynos_drm_decon.h
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Headef file for Samsung MIPI DSI Master driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_DECON_H__
#define __EXYNOS_DRM_DECON_H__

#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/spinlock.h>

#include <drm/drm_device.h>

#include <video/videomode.h>

#include "exynos_drm_dpp.h"
#include "exynos_drm_drv.h"

#include "cal_9820/decon_cal.h"

enum decon_state {
	DECON_STATE_INIT = 0,
	DECON_STATE_ON,
	DECON_STATE_DOZE,
	DECON_STATE_HIBER,
	DECON_STATE_DOZE_SUSPEND,
	DECON_STATE_OFF,
	DECON_STATE_TUI,
};

struct decon_win {
	u32				idx;
	struct decon_win_config		config;
	struct exynos_drm_plane		plane;
	struct exynos_drm_plane_config	plane_config;
};

struct decon_resources {
	struct pinctrl *pinctrl;
	struct pinctrl_state *te_on;
	struct pinctrl_state *te_off;
	struct clk *aclk;
};

struct decon_device {
	u32				id;
	enum decon_state		state;
	struct decon_regs		regs;
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct exynos_drm_crtc		*crtc;
	/* window information saved in window number order */
	struct decon_win		win[MAX_WIN_PER_DECON];
	/* dpp information saved in dpp channel number order */
	struct dpp_device		*dpp[MAX_DPP_CNT];
	u32				win_cnt;
	enum exynos_drm_output_type	con_type;
	struct videomode		v_mode;
	struct decon_config		config;
	struct decon_resources		res;

	u32				irq_fs;	/* frame start irq number*/
	u32				irq_fd;	/* frame done irq number*/
	u32				irq_ext;/* extra irq number*/
	u32				irq_te;

	spinlock_t			slock;
};

#endif /* __EXYNOS_DRM_DECON_H__ */
