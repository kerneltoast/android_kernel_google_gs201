// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based s6e3hc2 AMOLED LCD panel driver.
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/kthread.h>
#include <uapi/linux/sched/types.h>
#include <video/mipi_display.h>

#include "panel-samsung-drv.h"

static const unsigned char SEQ_PPS_WQHD[] = {
	// WQHD+ :1440x3040
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0B, 0xE0,
	0x05, 0xA0, 0x00, 0x28, 0x02, 0xD0, 0x02, 0xD0,
	0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x04, 0x6C,
	0x00, 0x0A, 0x00, 0x0C, 0x02, 0x77, 0x01, 0xE9,
	0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
	0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const unsigned char SEQ_PPS_FHD[] = {
	//FHD+ :1080x2340
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x24,
	0x04, 0x38, 0x00, 0x41, 0x02, 0x1C, 0x02, 0x1C,
	0x02, 0x00, 0x02, 0x0E, 0x00, 0x20, 0x06, 0x50,
	0x00, 0x07, 0x00, 0x0C, 0x01, 0x80, 0x01, 0x91,
	0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
	0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#define S6E3HC2_WRCTRLD_DIMMING_BIT    0x08
#define S6E3HC2_WRCTRLD_FRAME_RATE_BIT 0x10
#define S6E3HC2_WRCTRLD_BCTRL_BIT      0x20
#define S6E3HC2_WRCTRLD_HBM_BIT        0xC0

static const u8 unlock_cmd_f0[] = { 0xF0, 0x5A, 0x5A };
static const u8 lock_cmd_f0[]   = { 0xF0, 0xA5, 0xA5 };
static const u8 swire_refresh_global[] = { 0xB0, 0x3F, 0xB5 };
static const u8 swire_refresh_off[] = { 0xB5, 0x05 };
static const u8 swire_refresh_on[] = { 0xB5, 0x85 };
static const u8 aid_trans_global[] = { 0xB0, 0xA8, 0xB1 };
static const u8 aid_trans_off[] = { 0xB1, 0x00 };
static const u8 aid_trans_on[] = { 0xB1, 0x01 };
static const u8 hlpm_ctrl_global[] = { 0xB0, 0x05, 0xBB };
static const u8 hlpm_ctrl_50nit[] = { 0xBB, 0x00 };
static const u8 hlpm_ctrl_10nit[] = { 0xBB, 0x80 };
static const u8 hlpm_on_50nit[] = { 0x53, 0x02 };
static const u8 hlpm_on_10nit[] = { 0x53, 0x03 };
static const u8 gamma_aid_update[] = { 0xF7, 0x02 };
static const u8 display_off[] = { 0x28 };
static const u8 display_on[] = { 0x29 };
static const u8 sleep_in[] = { 0x10 };

static const struct exynos_dsi_cmd s6e3hc2_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 10),
	EXYNOS_DSI_CMD(sleep_in, 120),
};

static const struct exynos_dsi_cmd_set s6e3hc2_off_cmd_set = {
	.num_cmd = ARRAY_SIZE(s6e3hc2_off_cmds),
	.cmds = s6e3hc2_off_cmds
};

static const struct exynos_dsi_cmd s6e3hc2_lp_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 17),
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD(swire_refresh_global, 0),
	EXYNOS_DSI_CMD(swire_refresh_off, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 0),
};

static const struct exynos_dsi_cmd_set s6e3hc2_lp_cmd_set = {
	.num_cmd = ARRAY_SIZE(s6e3hc2_lp_cmds),
	.cmds = s6e3hc2_lp_cmds
};

static const struct exynos_dsi_cmd s6e3hc2_lp_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 0)
};

static const struct exynos_dsi_cmd s6e3hc2_lp_low_cmds[] = {
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD(hlpm_on_10nit, 17),
	EXYNOS_DSI_CMD(aid_trans_global, 0),
	EXYNOS_DSI_CMD(aid_trans_off, 0),
	EXYNOS_DSI_CMD(hlpm_ctrl_global, 0),
	EXYNOS_DSI_CMD(hlpm_ctrl_10nit, 0),
	EXYNOS_DSI_CMD(gamma_aid_update, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 0),
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_dsi_cmd s6e3hc2_lp_high_cmds[] = {
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD(hlpm_on_50nit, 17),
	EXYNOS_DSI_CMD(aid_trans_global, 0),
	EXYNOS_DSI_CMD(aid_trans_off, 0),
	EXYNOS_DSI_CMD(hlpm_ctrl_global, 0),
	EXYNOS_DSI_CMD(hlpm_ctrl_50nit, 0),
	EXYNOS_DSI_CMD(gamma_aid_update, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 0),
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_binned_lp s6e3hc2_binned_lp[] = {
	BINNED_LP_MODE("off",     0, s6e3hc2_lp_off_cmds),
	BINNED_LP_MODE("low",    40, s6e3hc2_lp_low_cmds),
	BINNED_LP_MODE("high", 1023, s6e3hc2_lp_high_cmds)
};

static const struct exynos_display_underrun_param fhd_underrun_param = {
	.te_idle_us = 1000,
	.te_var = 1,
};

static const struct exynos_panel_mode s6e3hc2_fhd_modes[] = {
	{
		.mode = {
			.name = "1080x2340x60",
			.clock = 164358,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32, // add hfp
			.hsync_end = 1080 + 32 + 12, // add hsa
			.htotal = 1080 + 32 + 12 + 26, // add hbp
			.vdisplay = 2340,
			.vsync_start = 2340 + 12, // add vfp
			.vsync_end = 2340 + 12 + 4, // add vsa
			.vtotal = 2340 + 12 + 4 + 26, // add vbp
			.flags = 0,
			.width_mm = 63,
			.height_mm = 137,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 65,
			},
			.underrun_param = &fhd_underrun_param,
		},
	},
	{
		.mode = {
			.name = "1080x2340x90",
			.clock = 246537,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32, // add hfp
			.hsync_end = 1080 + 32 + 12, // add hsa
			.htotal = 1080 + 32 + 12 + 26, // add hbp
			.vdisplay = 2340,
			.vsync_start = 2340 + 12, // add vfp
			.vsync_end = 2340 + 12 + 4, // add vsa
			.vtotal = 2340 + 12 + 4 + 26, // add vbp
			.flags = 0,
			.width_mm = 63,
			.height_mm = 137,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 65,
			},
			.underrun_param = &fhd_underrun_param,
		},
	},
};

#define CALI_GAMMA_HEADER_SIZE	3
#define S6E3HC2_GAMMA_BAND_LEN 45

/*
 * s6e3hc2_gamma_info - Information used to access gamma data on s6e3hc2.
 * @cmd: Command to use when writing/reading gamma from the DDIC.
 * @len: Total number of bytes to write/read from DDIC, including prefix_len.
 * @prefix_len: Number of bytes that precede gamma data when writing/reading
 *     from the DDIC. This is a subset of len.
 * @flash_offset: Address offset to use when reading from flash.
 */
struct s6e3hc2_gamma_info {
	u8 cmd;
	u32 len;
	u32 prefix_len;
	u32 flash_offset;
};

const struct s6e3hc2_gamma_info s6e3hc2_gamma_tables[] = {
	/* order of commands matter due to use of cmds grouping */
	{ 0xC8, S6E3HC2_GAMMA_BAND_LEN * 3, 0, 0x0000 },
	{ 0xC9, S6E3HC2_GAMMA_BAND_LEN * 4, 0, 0x0087 },
	{ 0xB3, 2 + S6E3HC2_GAMMA_BAND_LEN, 2, 0x013B },
};

#define S6E3HC2_NUM_GAMMA_TABLES ARRAY_SIZE(s6e3hc2_gamma_tables)

struct s6e3hc2_panel_data {
	u8 *gamma_data[S6E3HC2_NUM_GAMMA_TABLES];
	u8 *native_gamma_data[S6E3HC2_NUM_GAMMA_TABLES];
};

struct s6e3hc2_mode_data {
	const struct drm_display_mode *mode;
	struct s6e3hc2_panel_data *sdata;
};

struct s6e3hc2_panel {
	struct exynos_panel base;
	struct s6e3hc2_mode_data modes[ARRAY_SIZE(s6e3hc2_fhd_modes)];
	struct kthread_worker worker;
	struct task_struct *thread;
	struct kthread_work gamma_work;
	bool native_gamma_ready;
	u8 num_of_cali_gamma;
};

#define to_spanel(ctx) \
	container_of(ctx, struct s6e3hc2_panel, base)

static struct s6e3hc2_mode_data *s6e3hc2_get_mode_data(struct exynos_panel *ctx,
						const struct drm_display_mode *mode)
{
	struct s6e3hc2_panel *spanel;
	int i;

	if (unlikely(!ctx || !mode))
		return NULL;

	spanel = to_spanel(ctx);

	for (i = 0; i < ctx->desc->num_modes; i++) {
		if (spanel->modes[i].mode == mode)
			return &spanel->modes[i];
	}

	return NULL;
}

static void s6e3hc2_gamma_update(struct exynos_panel *ctx,
				 struct s6e3hc2_mode_data *mdata)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct s6e3hc2_panel_data *priv_data;
	int i;

	if (unlikely(!mdata))
		return;

	priv_data = mdata->sdata;
	if (unlikely(!priv_data))
		return;

	for (i = 0; i < S6E3HC2_NUM_GAMMA_TABLES; i++) {
		/* extra byte for the dsi command */
		const size_t len = s6e3hc2_gamma_tables[i].len + 1;
		const void *data = priv_data->gamma_data[i];

		if (WARN(!data, "Gamma table #%d not read\n", i))
			continue;

		if (IS_ERR_VALUE(mipi_dsi_dcs_write_buffer(dsi, data, len)))
			dev_warn(ctx->dev, "failed sending gamma cmd 0x%02x\n",
				s6e3hc2_gamma_tables[i].cmd);
	}
}

static int s6e3hc2_gamma_read_otp(struct exynos_panel *ctx,
				  struct s6e3hc2_panel_data *priv_data)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t rc;
	int i;

	for (i = 0; i < S6E3HC2_NUM_GAMMA_TABLES; i++) {
		const struct s6e3hc2_gamma_info *info =
			&s6e3hc2_gamma_tables[i];
		u8 *buf = priv_data->gamma_data[i];

		/* store cmd on first byte to send payload as is */
		*buf = info->cmd;
		buf++;

		rc = mipi_dsi_dcs_read(dsi, info->cmd, buf, info->len);
		if (rc != info->len)
			dev_warn(ctx->dev, "Only got %zd / %d bytes\n", rc, info->len);
	}

	return 0;
}

static int s6e3hc2_gamma_read_flash(struct exynos_panel *ctx,
				    struct s6e3hc2_panel_data *priv_data)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const u8 flash_mode_en[]  = { 0xF1, 0xF1, 0xA2 };
	const u8 flash_mode_dis[] = { 0xF1, 0xA5, 0xA5 };
	const u8 pgm_dis[]        = { 0xC0, 0x00 };
	const u8 pgm_en[]         = { 0xC0, 0x02 };
	const u8 exe_inst[]	  = { 0xC0, 0x03 };
	const u8 write_en[]       = { 0xC1,
		0x00, 0x00, 0x00, 0x06,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x05 };
	const u8 quad_en[]        = { 0xC1,
		0x00, 0x00, 0x00, 0x01,
		0x40, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x10 };
	ssize_t rc;
	int i, j;

	EXYNOS_DCS_WRITE_TABLE(ctx, flash_mode_en);
	EXYNOS_DCS_WRITE_TABLE(ctx, pgm_en);
	EXYNOS_DCS_WRITE_TABLE(ctx, write_en);
	EXYNOS_DCS_WRITE_TABLE(ctx, exe_inst);

	usleep_range(950, 1000);

	EXYNOS_DCS_WRITE_TABLE(ctx, quad_en);
	EXYNOS_DCS_WRITE_TABLE(ctx, exe_inst);

	msleep(30);

	for (i = 0; i < S6E3HC2_NUM_GAMMA_TABLES; i++) {
		const struct s6e3hc2_gamma_info *info;
		const u8 gpar_cmd[] = { 0xB0, 0x0B };
		u8 flash_rd[] = { 0xC1,
			0x00, 0x00, 0x00, 0x6B, 0x00, 0x00, 0x00, /*Read Inst*/
			0x0A, 0x00, 0x00,    /* Flash data Address : 0A0000h */
			0x00, 0x05,          /* Bit rate setting */
			0x01 };
		u32 offset;
		u8 *buf;

		info = &s6e3hc2_gamma_tables[i];
		offset = info->flash_offset;
		buf = priv_data->gamma_data[i];
		/* store cmd on first byte to send payload as is */
		*buf = info->cmd;
		buf++;

		for (j = info->prefix_len; j < info->len; j++, offset++) {
			u8 tmp[2];

			flash_rd[9] = (offset >> 8) & 0xFF;
			flash_rd[10] = offset & 0xFF;

			EXYNOS_DCS_WRITE_TABLE(ctx, flash_rd);
			EXYNOS_DCS_WRITE_TABLE(ctx, exe_inst);

			usleep_range(200, 250);

			EXYNOS_DCS_WRITE_TABLE(ctx, gpar_cmd);

			rc = mipi_dsi_dcs_read(dsi, 0xFB, tmp, sizeof(tmp));
			if (rc != 2)
				dev_warn(ctx->dev, "Only got %zd / 2 bytes\n", rc);

			dev_dbg(ctx->dev, "read flash offset %04x: %02X %02X\n",
				 offset, tmp[0], tmp[1]);
			buf[j] = tmp[1];
		}
	}

	EXYNOS_DCS_WRITE_TABLE(ctx, pgm_dis);
	EXYNOS_DCS_WRITE_TABLE(ctx, flash_mode_dis);

	return 0;
}

static int s6e3hc2_gamma_alloc_mode_memory(struct s6e3hc2_mode_data *mdata)
{
	struct s6e3hc2_panel_data *priv_data;
	size_t offset, native_offset, total_size;
	int i;
	u8 *buf, *native_buf;

	if (mdata->sdata)
		return 0;

	total_size = sizeof(*priv_data);

	for (i = 0; i < S6E3HC2_NUM_GAMMA_TABLES; i++)
		total_size += s6e3hc2_gamma_tables[i].len;
	/* add an extra byte for cmd */
	total_size += S6E3HC2_NUM_GAMMA_TABLES;

	/* hold native gamma */
	native_offset = total_size;
	total_size *= 2;

	priv_data = kmalloc(total_size, GFP_KERNEL);
	if (!priv_data)
		return -ENOMEM;

	/* use remaining data at the end of buffer */
	buf = (u8 *)(priv_data);
	offset = sizeof(*priv_data);
	native_buf = buf + native_offset;

	for (i = 0; i < S6E3HC2_NUM_GAMMA_TABLES; i++) {
		const size_t len = s6e3hc2_gamma_tables[i].len;

		priv_data->gamma_data[i] = buf + offset;
		priv_data->native_gamma_data[i] = native_buf + offset;
		/* reserve extra byte to hold cmd */
		offset += len + 1;
	}

	mdata->sdata = priv_data;

	return 0;
}

static int s6e3hc2_gamma_read_mode(struct exynos_panel *ctx,
				   const struct drm_display_mode *mode)
{
	struct s6e3hc2_panel_data *priv_data;
	struct s6e3hc2_mode_data *mdata = s6e3hc2_get_mode_data(ctx, mode);
	int rc;

	if (unlikely(!mdata))
		return -EINVAL;

	rc = s6e3hc2_gamma_alloc_mode_memory(mdata);
	if (rc)
		return rc;

	priv_data = mdata->sdata;

	switch (drm_mode_vrefresh(mode)) {
	case 60:
		rc = s6e3hc2_gamma_read_otp(ctx, priv_data);
		break;
	case 90:
		rc = s6e3hc2_gamma_read_flash(ctx, priv_data);
		break;
	default:
		dev_warn(ctx->dev, "Unknown refresh rate!\n");
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int find_gamma_data_for_refresh_rate(struct exynos_panel *ctx,
	u32 refresh_rate, u8 ***gamma_data)
{
	const struct drm_display_mode *mode;
	struct s6e3hc2_panel *spanel = to_spanel(ctx);
	int i;

	if (!gamma_data)
		return -EINVAL;

	for (i = 0; i < ctx->desc->num_modes; i++) {
		mode = spanel->modes[i].mode;
		if (unlikely(!mode))
			return -ENOMEM;

		if (drm_mode_vrefresh(mode) == refresh_rate) {
			struct s6e3hc2_panel_data *priv_data;

			priv_data = spanel->modes[i].sdata;
			if (unlikely(!priv_data))
				return -ENODATA;

			*gamma_data = priv_data->gamma_data;
			return 0;
		}
	}

	return -ENODATA;
}

/*
 * For some modes, gamma curves are located in registers addresses that require
 * an offset to read/write. Because we cannot access a register offset directly,
 * we must read the portion of the data that precedes the gamma curve data
 * itself ("prefix") as well. In such cases, we read the prefix + gamma curve
 * data from DDIC registers, and only gamma curve data from flash.
 *
 * This function looks for such gamma curves, and adjusts gamma data read from
 * flash to include the prefix read from registers. The result is that, for all
 * modes, wherever the gamma curves were read from (registers or flash), when
 * that gamma data is written back to registers the write includes the original
 * prefix.
 * In other words, when we write gamma data to registers, we do not modify
 * prefix data; we only modify gamma data.
 */
static int s6e3hc2_gamma_set_prefixes(struct exynos_panel *ctx)
{
	int i;
	int rc = 0;
	u8 **gamma_data_otp;
	u8 **gamma_data_flash;

	/*
	 * For s6e3hc2, 60Hz gamma curves are read from OTP and 90Hz
	 * gamma curves are read from flash.
	 */
	rc = find_gamma_data_for_refresh_rate(ctx, 60, &gamma_data_otp);
	if (rc) {
		dev_err(ctx->dev, "Error setting gamma prefix: no matching OTP mode, err %d\n",
			rc);
		return rc;
	}

	rc = find_gamma_data_for_refresh_rate(ctx, 90, &gamma_data_flash);
	if (rc) {
		dev_err(ctx->dev, "Error setting gamma prefix: no matching flash mode, err %d\n",
			rc);
		return rc;
	}

	for (i = 0; i < S6E3HC2_NUM_GAMMA_TABLES; i++) {
		const struct s6e3hc2_gamma_info *gamma_info =
			&s6e3hc2_gamma_tables[i];
		u8 *gamma_curve_otp = gamma_data_otp[i];
		u8 *gamma_curve_flash = gamma_data_flash[i];

		if (!gamma_info->prefix_len)
			continue;

		/* skip command byte */
		gamma_curve_otp++;
		gamma_curve_flash++;

		memcpy(gamma_curve_flash, gamma_curve_otp,
			gamma_info->prefix_len);
	}

	return rc;
}

static int s6e3hc2_copy_gamma_table(u8 **dest_gamma, u8 **src_gamma)
{
	int i;

	for (i = 0; i < S6E3HC2_NUM_GAMMA_TABLES; i++) {
		const size_t len = s6e3hc2_gamma_tables[i].len;

		memcpy(dest_gamma[i], src_gamma[i], len * sizeof(**src_gamma));
	}

	return 0;
}

static int s6e3hc2_gamma_read_tables(struct exynos_panel *ctx)
{
	struct s6e3hc2_panel *spanel = to_spanel(ctx);
	const struct drm_display_mode *mode;
	int i, rc = 0;

	if (spanel->native_gamma_ready)
		return 0;

	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);

	for_each_display_mode(i, mode, ctx) {
		rc = s6e3hc2_gamma_read_mode(ctx, mode);
		if (rc) {
			dev_err(ctx->dev, "Unable to read gamma for mode #%d\n", i);
			goto abort;
		}
	}

	rc = s6e3hc2_gamma_set_prefixes(ctx);
	if (rc) {
		dev_err(ctx->dev, "Unable to set gamma prefix\n");
		goto abort;
	}

	for_each_display_mode(i, mode, ctx) {
		struct s6e3hc2_mode_data *mdata = s6e3hc2_get_mode_data(ctx, mode);
		struct s6e3hc2_panel_data *priv_data = mdata->sdata;
		u8 **gamma_data = priv_data->gamma_data;
		u8 **native_gamma_data = priv_data->native_gamma_data;

		s6e3hc2_copy_gamma_table(native_gamma_data, gamma_data);
	}

	spanel->native_gamma_ready = true;
abort:
	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);

	return rc;
}

static void s6e3hc2_perform_switch(struct exynos_panel *ctx,
				   const struct drm_display_mode *mode)
{
	struct s6e3hc2_mode_data *mdata = s6e3hc2_get_mode_data(ctx, mode);

	if (unlikely(!mdata))
		return;

	s6e3hc2_gamma_update(ctx, mdata);
}

static void s6e3hc2_write_display_mode(struct exynos_panel *ctx,
				       const struct drm_display_mode *mode)
{
	u8 val = S6E3HC2_WRCTRLD_BCTRL_BIT;

	if (drm_mode_vrefresh(mode) == 90)
		val |= S6E3HC2_WRCTRLD_FRAME_RATE_BIT;

	if (IS_HBM_ON(ctx->hbm_mode))
		val |= S6E3HC2_WRCTRLD_HBM_BIT;

	dev_dbg(ctx->dev, "%s(wrctrld:0x%x, hbm: %s, refresh: %uhz)\n",
		__func__, val, IS_HBM_ON(ctx->hbm_mode) ? "on" : "off", drm_mode_vrefresh(mode));

	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	EXYNOS_DCS_WRITE_SEQ(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);

	s6e3hc2_perform_switch(ctx, mode);
	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);
}

static void s6e3hc2_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	if (!ctx->enabled)
		return;

	EXYNOS_DCS_WRITE_TABLE(ctx, display_off);
	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	EXYNOS_DCS_WRITE_TABLE(ctx, aid_trans_global);
	EXYNOS_DCS_WRITE_TABLE(ctx, aid_trans_on);

	s6e3hc2_write_display_mode(ctx, &pmode->mode);

	EXYNOS_DCS_WRITE_TABLE(ctx, swire_refresh_global);
	EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, 34, swire_refresh_on);
	EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, 17, lock_cmd_f0);
	EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	dev_info(ctx->dev, "exit LP mode\n");
}

static int s6e3hc2_common_pre_enable(struct exynos_panel *ctx)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xf0, 0x5a, 0x5a);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xfc, 0x5a, 0x5a);

	/* DSC related configuration */
	exynos_dcs_compression_mode(ctx, 0x1);

	EXYNOS_PPS_LONG_WRITE(ctx);
	/* sleep out: 120ms delay */
	EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, 120, 0x11);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x00, 0xB0, 0x8F, 0x09, 0x00, 0x00,
			0x00, 0x11, 0x01);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x1A, 0x1F, 0x00, 0x00, 0x00, 0x00);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x35); /* TE on */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xED, 0x44);

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x01, 0xB0, 0x91, 0x09);
#else
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x00, 0xB0, 0x9C, 0x09);
#endif
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xCE, 0x0D, 0x58, 0x14, 0x64, 0x38, 0xB8,
			0xF2, 0x03, 0x00, 0xFF, 0x02, 0x0A, 0x0A, 0x0A, 0x0A,
			0x0F, 0x23);

	s6e3hc2_write_display_mode(ctx, &pmode->mode);

	return 0;
}

static void s6e3hc2_common_post_enable(struct exynos_panel *ctx)
{
	const struct exynos_panel_mode *pmode;
	struct s6e3hc2_panel *spanel = to_spanel(ctx);

	dev_dbg(ctx->dev, "%s\n", __func__);

	ctx->enabled = true;

	pmode = ctx->current_mode;
	if (pmode->exynos_mode.is_lp_mode)
		exynos_panel_set_lp_mode(ctx, pmode);
	else
		EXYNOS_DCS_WRITE_SEQ(ctx, 0x29); /* display on */

	kthread_flush_work(&spanel->gamma_work);
	if (!spanel->native_gamma_ready)
		kthread_queue_work(&spanel->worker, &spanel->gamma_work);
}

static int s6e3hc2_wqhd_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;
	int ret;

	ctx = container_of(panel, struct exynos_panel, panel);

	ret = s6e3hc2_common_pre_enable(ctx);
	if (ret)
		return ret;

	s6e3hc2_common_post_enable(ctx);

	return 0;
}

static int s6e3hc2_fhd_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;
	int ret;

	ctx = container_of(panel, struct exynos_panel, panel);

	ret = s6e3hc2_common_pre_enable(ctx);
	if (ret)
		return ret;

	/* TSP HSYNC Enable*/
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x07, 0xB9);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x11, 0x03);

	/*TOUT Enable*/
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xFF, 0x00, 0x01);

	s6e3hc2_common_post_enable(ctx);

	return 0;
}

static int s6e3hc2_panel_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	struct s6e3hc2_panel *spanel = to_spanel(ctx);

	kthread_flush_work(&spanel->gamma_work);

	exynos_panel_disable(panel);

	return 0;
}

static void s6e3hc2_set_hbm_mode(struct exynos_panel *exynos_panel,
				enum exynos_hbm_mode mode)
{
	const struct exynos_panel_mode *pmode = exynos_panel->current_mode;

	exynos_panel->hbm_mode = mode;

	s6e3hc2_write_display_mode(exynos_panel, &pmode->mode);
}

static void s6e3hc2_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{
	s6e3hc2_write_display_mode(ctx, &pmode->mode);
}

static bool s6e3hc2_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	/* seamless mode switch is possible if only changing refresh rate */
	return drm_mode_equal_no_clocks(&ctx->current_mode->mode, &pmode->mode);
}

static void s6e3hc2_panel_init(struct exynos_panel *ctx)
{
	struct s6e3hc2_panel *spanel = to_spanel(ctx);

	if (!ctx->enabled)
		return;

	if (!spanel->native_gamma_ready)
		kthread_queue_work(&spanel->worker, &spanel->gamma_work);
}

static void s6e3hc2_print_gamma(struct seq_file *seq,
				const struct drm_display_mode *mode)
{
	struct exynos_panel *ctx = seq->private;
	struct s6e3hc2_panel *spanel = to_spanel(ctx);
	struct s6e3hc2_panel_data *priv_data;
	struct s6e3hc2_mode_data *mdata =
			s6e3hc2_get_mode_data(ctx, mode);
	int i, j;

	if (unlikely(!spanel->native_gamma_ready)) {
		seq_puts(seq, "s6e3hc2 panel data not ready\n");
		return;
	}

	if (unlikely(!mdata || !mdata->sdata)) {
		seq_puts(seq, "s6e3hc2 mode data not ready\n");
		return;
	}

	priv_data = mdata->sdata;
	for (i = 0; i < S6E3HC2_NUM_GAMMA_TABLES; i++) {
		const size_t len = s6e3hc2_gamma_tables[i].len;
		const u8 cmd = s6e3hc2_gamma_tables[i].cmd;
		const u8 *buf = priv_data->gamma_data[i] + 1;

		seq_printf(seq, "0x%02X:", cmd);
		for (j = 0; j < len; j++) {
			if (j && (j % 8) == 0)
				seq_puts(seq, "\n     ");
			seq_printf(seq, " %02X", buf[j]);
		}
		seq_puts(seq, "\n");
	}
}

static int s6e3hc2_restore_gamma_data(struct exynos_panel *ctx)
{
	struct s6e3hc2_panel *spanel = to_spanel(ctx);
	const struct drm_display_mode *mode;
	int i;

	for_each_display_mode(i, mode, ctx) {
		struct s6e3hc2_mode_data *mdata = s6e3hc2_get_mode_data(ctx, mode);
		struct s6e3hc2_panel_data *priv_data = mdata->sdata;
		u8 **gamma_data = priv_data->gamma_data;
		u8 **native_gamma_data = priv_data->native_gamma_data;

		s6e3hc2_copy_gamma_table(gamma_data, native_gamma_data);
	}

	spanel->num_of_cali_gamma = 0;

	return 0;
}

static int parse_payload_len(const u8 *payload, size_t len, size_t *out_size)
{
	size_t payload_len;

	if (!out_size || len <= CALI_GAMMA_HEADER_SIZE)
		return -EINVAL;

	*out_size = payload_len = (payload[1] << 8) + payload[2];

	return payload_len &&
		(payload_len + CALI_GAMMA_HEADER_SIZE <= len) ? 0 : -EINVAL;
}

/* ID (1 byte) | payload size (2 bytes) | payload */
static int s6e3hc2_check_gamma_payload(const u8 *src, size_t len)
{
	size_t total_len = 0;
	int num_payload = 0;

	if (!src)
		return -EINVAL;

	while (len > total_len) {
		const u8 *payload = src + total_len;
		size_t payload_len;
		int rc;

		rc = parse_payload_len(payload, len - total_len, &payload_len);
		if (rc)
			return -EINVAL;

		total_len += CALI_GAMMA_HEADER_SIZE + payload_len;
		num_payload++;
	}

	return num_payload;
}

static int s6e3hc2_overwrite_gamma_bands(struct exynos_panel *ctx, u8 **gamma_data,
					 const u8 *src, size_t len)
{
	int i, num_of_reg;
	size_t payload_len, read_len = 0;

	if (!gamma_data || !src)
		return -EINVAL;

	num_of_reg = s6e3hc2_check_gamma_payload(src, len);
	if (num_of_reg != (S6E3HC2_NUM_GAMMA_TABLES - 1)) {
		dev_err(ctx->dev, "Invalid gamma bands\n");
		return -EINVAL;
	}

	/* reg (1 byte) | band size (2 bytes) | band */
	for (i = 0; i < num_of_reg; i++) {
		const struct s6e3hc2_gamma_info *gamma_info =
			&s6e3hc2_gamma_tables[i];
		const u8 *payload = src + read_len;
		const u8 cmd = *payload;
		u8 *tmp_buf = gamma_data[i] + sizeof(cmd)
				+ gamma_info->prefix_len;

		parse_payload_len(payload, len - read_len, &payload_len);
		if (gamma_info->cmd != cmd ||
		    payload_len != (gamma_info->len - gamma_info->prefix_len)) {
			dev_err(ctx->dev, "Failed to overwrite 0x%02X reg\n", cmd);
			return -EINVAL;
		}

		memcpy(tmp_buf, payload + CALI_GAMMA_HEADER_SIZE, payload_len);
		read_len = CALI_GAMMA_HEADER_SIZE + payload_len;
	}

	return 0;
}

static ssize_t s6e3hc2_overwrite_gamma_data(struct exynos_panel *ctx,
					    const char *buf, size_t len)
{
	struct s6e3hc2_panel *spanel = to_spanel(ctx);
	const u8 *payload;
	u8 **old_gamma_data, count;
	size_t payload_len;
	int rc = 0, num_of_fps;
	const struct exynos_panel_mode *pmode;

	payload = buf;

	if (unlikely(!payload))
		return -EINVAL;

	num_of_fps = s6e3hc2_check_gamma_payload(payload, len);
	if (num_of_fps <= 0) {
		dev_err(ctx->dev, "Invalid gamma data\n");
		return -EINVAL;
	}

	/*
	 * 60Hz's gamma table and 90Hz's gamma table are the same size,
	 * so we won't recalculate payload_len.
	 */
	parse_payload_len(payload, len, &payload_len);

	/* FPS (1 byte) | gamma size (2 bytes) | gamma */
	for (count = 0; count < num_of_fps; count++) {
		const u8 cali_fps = *payload;
		rc = find_gamma_data_for_refresh_rate(ctx,
					cali_fps, &old_gamma_data);
		if (rc) {
			dev_err(ctx->dev, "Not support %ufps, err %d\n", cali_fps, rc);
			break;
		}
		payload += CALI_GAMMA_HEADER_SIZE;
		rc = s6e3hc2_overwrite_gamma_bands(ctx, old_gamma_data,
							payload, payload_len);
		if (rc) {
			dev_err(ctx->dev, "Failed to overwrite gamma\n");
			break;
		}
		payload += payload_len;
	}

	if (rc) {
		s6e3hc2_restore_gamma_data(ctx);
	} else {
		pmode = ctx->current_mode;
		s6e3hc2_perform_switch(ctx, &pmode->mode);
		spanel->num_of_cali_gamma = count;

		dev_dbg(ctx->dev, "Finished overwriting gamma\n");
	}

	return rc;
}

static ssize_t s6e3hc2_restore_native_gamma(struct exynos_panel *ctx)
{
	struct s6e3hc2_panel *spanel = to_spanel(ctx);
	const struct exynos_panel_mode *pmode;

	if (!spanel->num_of_cali_gamma)
		return -EINVAL;

	s6e3hc2_restore_gamma_data(ctx);

	pmode = ctx->current_mode;
	s6e3hc2_perform_switch(ctx, &pmode->mode);

	dev_dbg(ctx->dev, "Finished restore gamma\n");

	return 0;
}

static void s6e3hc2_gamma_work(struct kthread_work *work)
{
	struct s6e3hc2_panel *spanel =
		container_of(work, struct s6e3hc2_panel, gamma_work);
	struct exynos_panel *ctx = &spanel->base;

	s6e3hc2_gamma_read_tables(ctx);
}

static int s6e3hc2_panel_probe(struct mipi_dsi_device *dsi)
{
	struct s6e3hc2_panel *spanel;
	struct exynos_panel *ctx;
	const struct drm_display_mode *mode;
	int i, ret = 0;
	struct sched_param param = {
		.sched_priority = 16,
	};

	spanel = devm_kzalloc(&dsi->dev, sizeof(*spanel), GFP_KERNEL);
	if (!spanel)
		return -ENOMEM;

	ret = exynos_panel_common_init(dsi, &spanel->base);
	if (ret)
		return ret;

	ctx = &spanel->base;
	for_each_display_mode(i, mode, ctx) {
		spanel->modes[i].mode = mode;
	}

	kthread_init_worker(&spanel->worker);
	spanel->thread = kthread_run(kthread_worker_fn, &spanel->worker, "panel");
	if (IS_ERR_OR_NULL(spanel->thread))
		return -EFAULT;

	sched_setscheduler(spanel->thread, SCHED_FIFO, &param);
	kthread_init_work(&spanel->gamma_work, s6e3hc2_gamma_work);

	return 0;
}

static const struct exynos_display_underrun_param wqhd_underrun_param = {
	.te_idle_us = 1000,
	.te_var = 1,
};

static const u32 s6e3hc2_bl_range[] = {
	129, 157, 187, 1023,
};

static const struct exynos_panel_mode s6e3hc2_wqhd_modes[] = {
	{
		/* 1440x3040 @ 60Hz */
		.mode = {
			.name = "1440x3040x60",
			.clock = 266568,
			.hdisplay = 1440,
			.hsync_start = 1440 + 2,
			.hsync_end = 1440 + 2 + 2,
			.htotal = 1440 + 6 + 2 + 2,
			.vdisplay = 3040,
			.vsync_start = 3040 + 8,
			.vsync_end = 3040 + 8 + 1,
			.vtotal = 3040 + 8 + 1 + 15,
			.flags = 0,
			.width_mm = 69,
			.height_mm = 142,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 40,
			},
			.underrun_param = &wqhd_underrun_param,
		},
	},
	{
		/* 1440x3040 @ 90Hz */
		.mode = {
			.name = "1440x3040x90",
			.clock = 399852,
			.hdisplay = 1440,
			.hsync_start = 1440 + 2,
			.hsync_end = 1440 + 2 + 2,
			.htotal = 1440 + 6 + 2 + 2,
			.vdisplay = 3040,
			.vsync_start = 3040 + 8,
			.vsync_end = 3040 + 8 + 1,
			.vtotal = 3040 + 8 + 1 + 15,
			.flags = 0,
			.width_mm = 69,
			.height_mm = 142,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 40,
			},
			.underrun_param = &wqhd_underrun_param,
		},
	}
};

static const struct exynos_panel_mode s6e3hc2_wqhd_lp_mode = {
	.mode = {
		/* 1440x3040 @ 30Hz */
		.name = "1440x3040x30",
		.clock = 133284,
		.hdisplay = 1440,
		.hsync_start = 1440 + 2,
		.hsync_end = 1440 + 2 + 2,
		.htotal = 1440 + 6 + 2 + 2,
		.vdisplay = 3040,
		.vsync_start = 3040 + 8,
		.vsync_end = 3040 + 8 + 1,
		.vtotal = 3040 + 8 + 1 + 15,
		.flags = 0,
		.type = DRM_MODE_TYPE_DRIVER,
		.width_mm = 69,
		.height_mm = 142,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		.dsc = {
			.enabled = true,
			.dsc_count = 2,
			.slice_count = 2,
			.slice_height = 40,
		},
		.is_lp_mode = true,
	}
};

static const struct exynos_panel_mode s6e3hc2_fhd_lp_mode = {
	.mode = {
		/* 1080x2340 @ 30Hz */
		.name = "1080x2340x30",
		.clock = 82179,
		.hdisplay = 1080,
		.hsync_start = 1080 + 32, // add hfp
		.hsync_end = 1080 + 32 + 12, // add hsa
		.htotal = 1080 + 32 + 12 + 26, // add hbp
		.vdisplay = 2340,
		.vsync_start = 2340 + 12, // add vfp
		.vsync_end = 2340 + 12 + 4, // add vsa
		.vtotal = 2340 + 12 + 4 + 26, // add vbp
		.flags = 0,
		.type = DRM_MODE_TYPE_DRIVER,
		.width_mm = 63,
		.height_mm = 137,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		.dsc = {
			.enabled = true,
			.dsc_count = 2,
			.slice_count = 2,
			.slice_height = 65,
		},
		.is_lp_mode = true,
	},
};

static const struct drm_panel_funcs s6e3hc2_wqhd_drm_funcs = {
	.disable = s6e3hc2_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3hc2_wqhd_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct drm_panel_funcs s6e3hc2_fhd_drm_funcs = {
	.disable = s6e3hc2_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3hc2_fhd_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs s6e3hc2_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_lp_mode = exynos_panel_set_lp_mode,
	.set_nolp_mode = s6e3hc2_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.set_hbm_mode = s6e3hc2_set_hbm_mode,
	.is_mode_seamless = s6e3hc2_is_mode_seamless,
	.mode_set = s6e3hc2_mode_set,
	.panel_init = s6e3hc2_panel_init,
	.print_gamma = s6e3hc2_print_gamma,
	.gamma_store = s6e3hc2_overwrite_gamma_data,
	.restore_native_gamma = s6e3hc2_restore_native_gamma,
};

const struct brightness_capability s6e3hc2_brightness_capability = {
	.normal = {
		.nits = {
			.min = 2,
			.max = 450,
		},
		.level = {
			.min = 4,
			.max = 1023,
		},
		.percentage = {
			.min = 0,
			.max = 80,
		},
	},
	.hbm = {
		.nits = {
			.min = 600,
			.max = 600,
		},
		.level = { /* not valid for one-step hbm */
			.min = 1023,
			.max = 1023,
		},
		.percentage = {
			.min = 80,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc samsung_s6e3hc2_wqhd = {
	.dsc_pps = SEQ_PPS_WQHD,
	.dsc_pps_len = ARRAY_SIZE(SEQ_PPS_WQHD),
	.data_lane_cnt = 4,
	.max_brightness = 1023,
	.dft_brightness = 511,
	.brt_capability = &s6e3hc2_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 6000000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = s6e3hc2_bl_range,
	.bl_num_ranges = ARRAY_SIZE(s6e3hc2_bl_range),
	.modes = s6e3hc2_wqhd_modes,
	.num_modes = ARRAY_SIZE(s6e3hc2_wqhd_modes),
	.off_cmd_set = &s6e3hc2_off_cmd_set,
	.lp_mode = &s6e3hc2_wqhd_lp_mode,
	.lp_cmd_set = &s6e3hc2_lp_cmd_set,
	.binned_lp = s6e3hc2_binned_lp,
	.num_binned_lp = ARRAY_SIZE(s6e3hc2_binned_lp),
	.panel_func = &s6e3hc2_wqhd_drm_funcs,
	.exynos_panel_func = &s6e3hc2_exynos_funcs,
};

const struct exynos_panel_desc samsung_s6e3hc2_fhd = {
	.dsc_pps = SEQ_PPS_FHD,
	.dsc_pps_len = ARRAY_SIZE(SEQ_PPS_FHD),
	.data_lane_cnt = 4,
	.max_brightness = 1023,
	.dft_brightness = 511,
	.brt_capability = &s6e3hc2_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 6000000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = s6e3hc2_bl_range,
	.bl_num_ranges = ARRAY_SIZE(s6e3hc2_bl_range),
	.modes = s6e3hc2_fhd_modes,
	.num_modes = ARRAY_SIZE(s6e3hc2_fhd_modes),
	.off_cmd_set = &s6e3hc2_off_cmd_set,
	.lp_mode = &s6e3hc2_fhd_lp_mode,
	.lp_cmd_set = &s6e3hc2_lp_cmd_set,
	.binned_lp = s6e3hc2_binned_lp,
	.num_binned_lp = ARRAY_SIZE(s6e3hc2_binned_lp),
	.panel_func = &s6e3hc2_fhd_drm_funcs,
	.exynos_panel_func = &s6e3hc2_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,s6e3hc2", .data = &samsung_s6e3hc2_wqhd },
	{ .compatible = "samsung,s6e3hc2-fhd", .data = &samsung_s6e3hc2_fhd },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = s6e3hc2_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-s6e3hc2",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung s6e3hc2 panel driver");
MODULE_LICENSE("GPL");
