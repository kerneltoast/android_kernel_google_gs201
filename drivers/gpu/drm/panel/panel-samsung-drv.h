/* SPDX-License-Identifier: GPL-2.0-only
 *
 * MIPI-DSI based Samsung common panel driver header
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PANEL_SAMSUNG_DRV_
#define _PANEL_SAMSUNG_DRV_

#include <linux/printk.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>


#define MAX_REGULATORS		3
#define MAX_HDR_FORMATS		4

#define HDR_DOLBY_VISION	BIT(1)
#define HDR_HDR10		BIT(2)
#define HDR_HLG			BIT(3)

extern int panel_log_level;

#define panel_info(ctx, fmt, ...)					\
	do {								\
		if (panel_log_level >= 6) {				\
			pr_info("%s: "fmt, ctx->dev->driver->name,	\
					##__VA_ARGS__);			\
		}							\
	} while (0)

#define panel_warn(ctx, fmt, ...)					\
	do {								\
		if (panel_log_level >= 4) {				\
			pr_warn("%s: "fmt, ctx->dev->driver->name,	\
					##__VA_ARGS__);			\
		}							\
	} while (0)

#define panel_err(ctx, fmt, ...)					\
	do {								\
		if (panel_log_level >= 3) {				\
			pr_err("%s: "fmt, ctx->dev->driver->name,	\
					##__VA_ARGS__);			\
		}							\
	} while (0)

#define panel_dbg(ctx, fmt, ...)					\
	do {								\
		if (panel_log_level >= 7) {				\
			pr_info("%s: "fmt, ctx->dev->driver->name,	\
					##__VA_ARGS__);			\
		}							\
	} while (0)

struct exynos_panel_desc {
	bool dsc_en;
	u32 dsc_slice_cnt;
	u32 dsc_slice_height;
	u32 data_lane_cnt;
	u32 hdr_formats; /* supported HDR formats bitmask */
	u32 max_luminance;
	u32 max_avg_luminance;
	u32 min_luminance;
	unsigned long mode_flags;
	const struct drm_display_mode *mode;
	const struct drm_panel_funcs *panel_func;
};

struct exynos_panel {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator *regulator[MAX_REGULATORS];
	const struct exynos_panel_desc *desc;
};

static inline int exynos_dcs_write(struct exynos_panel *ctx, const void *data,
		size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_write_buffer(dsi, data, len);
}

static inline int exynos_dcs_compression_mode(struct exynos_panel *ctx, u8 mode)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_compression_mode(dsi, mode);
}

#define EXYNOS_DCS_WRITE_SEQ(ctx, seq...) do {				\
	u8 d[] = { seq };						\
	int ret;							\
	ret = exynos_dcs_write(ctx, d, ARRAY_SIZE(d));			\
	if (ret < 0)							\
		panel_err(ctx, "failed to write cmd(%d)\n", ret);	\
} while (0)

#define EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, delay, seq...) do {		\
	EXYNOS_DCS_WRITE_SEQ(ctx, seq);					\
	msleep(delay);							\
} while (0)

#define EXYNOS_DCS_WRITE_TABLE(ctx, table) do {				\
	int ret;							\
	ret = exynos_dcs_write(ctx, table, ARRAY_SIZE(table));		\
	if (ret < 0)							\
		panel_err(ctx, "failed to write cmd(%d)\n", ret);	\
} while (0)

#define EXYNOS_PPS_LONG_WRITE(ctx, pps) do {				\
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);	\
	int ret;							\
	ret = mipi_dsi_pps_long_write(dsi, pps, ARRAY_SIZE(pps));	\
	if (ret < 0)							\
		panel_err(ctx, "failed to write cmd(%d)\n", ret);	\
} while (0)

int exynos_panel_get_modes(struct drm_panel *panel);
void exynos_panel_reset(struct exynos_panel *ctx);
int exynos_panel_set_power(struct exynos_panel *ctx, bool on);

extern const struct exynos_panel_desc samsung_emul;
extern const struct exynos_panel_desc samsung_s6e3ha8;
extern const struct exynos_panel_desc samsung_s6e3ha9;
extern const struct exynos_panel_desc samsung_s6e3hc2;

#endif /* _PANEL_SAMSUNG_DRV_ */
