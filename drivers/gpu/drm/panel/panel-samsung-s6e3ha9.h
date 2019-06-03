/* SPDX-License-Identifier: GPL-2.0-only
 *
 * MIPI-DSI based s6e3ha9 AMOLED LCD panel driver header
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PANEL_SAMSUNG_S6E3HA9_
#define _PANEL_SAMSUNG_S6E3HA9_

#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>

#define MAX_REGULATORS		3
#define MAX_HDR_FORMATS		4

#define HDR_DOLBY_VISION	(1<<1)
#define HDR_HDR10		(1<<2)
#define HDR_HLG			(1<<3)

struct s6e3ha9_panel_desc {
	const struct drm_display_mode *mode;
	bool dsc_en;
	u32 dsc_slice_cnt;
	u32 dsc_slice_height;
	u32 data_lane_cnt;
	u32 hdr_formats; /* supported HDR formats bitmask */
	u32 max_luminance;
	u32 max_avg_luminance;
	u32 min_luminance;
};

struct s6e3ha9 {
	struct device *dev;
	struct drm_panel panel;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator *regulator[MAX_REGULATORS];

	const struct s6e3ha9_panel_desc *desc;
};

#endif /* _PANEL_SAMSUNG_S6E3HA9_ */
