/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include "gs_panel/dcs_helper.h"

#include <linux/delay.h>
#include <linux/version.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>

#include <trace/panel_trace.h>

#include "gs_panel_internal.h"

/**
 * GS_DSI_MSG_FLAG_MASK - contains all supported dsi cmd msg flags
 * Used to explicitly control which dsi flags may be attached when sending cmdset
 */
#define GS_DSI_MSG_FLAG_MASK (GS_DSI_MSG_QUEUE | GS_DSI_MSG_IGNORE_VBLANK)

void gs_dsi_send_cmdset(struct mipi_dsi_device *dsi, const struct gs_dsi_cmdset *cmdset,
			u32 panel_rev)
{
	const struct gs_dsi_cmd *c;
	const struct gs_dsi_cmd *last_cmd = NULL;

	if (!cmdset || !cmdset->num_cmd)
		return;

	c = &cmdset->cmds[cmdset->num_cmd - 1];
	if (!c->panel_rev) {
		last_cmd = c;
	} else {
		for (; c >= cmdset->cmds; c--) {
			if (c->panel_rev & panel_rev) {
				last_cmd = c;
				break;
			}
			/* prevent undefined address even if it's not dereferenced */
			if (c == cmdset->cmds)
				break;
		}
	}

	/* no commands to transfer */
	if (!last_cmd)
		return;

	for (c = cmdset->cmds; c <= last_cmd; c++) {
		u16 dsi_flags = 0;
		u32 delay_ms = c->delay_ms;

		/* skip if not correct panel rev */
		if (panel_rev && !(c->panel_rev & panel_rev))
			continue;

		/* explicitly transfer flags */
		dsi_flags = c->flags & GS_DSI_MSG_FLAG_MASK;

		gs_dsi_dcs_write_buffer(dsi, c->cmd, c->cmd_len, dsi_flags);
		if (delay_ms)
			usleep_range(delay_ms * 1000, delay_ms * 1000 + 10);
	}
}
EXPORT_SYMBOL_GPL(gs_dsi_send_cmdset);

static void write_dcs_transfer_trace(size_t len, const u8 *data)
{
	static char tmp[48] = { 0 };
	switch (len) {
	case 1:
		snprintf(tmp, sizeof(tmp), "[%02X]", data[0]);
		break;
	case 2:
		snprintf(tmp, sizeof(tmp), "[%02X %02X]", data[0], data[1]);
		break;
	case 3:
		snprintf(tmp, sizeof(tmp), "[%02X %02X %02X]", data[0], data[1], data[2]);
		break;
	case 4:
		snprintf(tmp, sizeof(tmp), "[%02X %02X %02X %02X]",
			 data[0], data[1], data[2], data[3]);
		break;
	case 5:
		snprintf(tmp, sizeof(tmp), "[%02X %02X %02X %02X %02X]",
			 data[0], data[1], data[2], data[3], data[4]);
		break;
	case 6:
		snprintf(tmp, sizeof(tmp), "[%02X %02X %02X %02X %02X %02X]",
			 data[0], data[1], data[2], data[3], data[4], data[5]);
		break;
	case 7:
		snprintf(tmp, sizeof(tmp), "[%02X %02X %02X %02X %02X %02X %02X]",
			 data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
		break;
	case 8:
		snprintf(tmp, sizeof(tmp), "[%02X %02X %02X %02X %02X %02X %02X %02X]",
			 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
		break;
	default:
		snprintf(tmp, sizeof(tmp), "[%02X %02X %02X %02X %02X %02X %02X %02X ...]",
			 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
	}
	PANEL_ATRACE_INSTANT("dsi_dcs_transfer len:%zu msg:%s", len, tmp);
}

ssize_t gs_dsi_dcs_transfer(struct mipi_dsi_device *dsi, u8 type, const void *data, size_t len,
			    u16 flags)
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
	if (dsi->mode_flags & MIPI_DSI_MODE_LPM)
		msg.flags |= MIPI_DSI_MSG_USE_LPM;
	is_last = ((flags & GS_DSI_MSG_QUEUE) == 0) || ((flags & GS_DSI_MSG_FORCE_FLUSH) != 0);
	trace_dsi_tx(msg.type, msg.tx_buf, msg.tx_len, is_last, 0);
	if (trace_panel_write_generic_enabled()) {
		if (len)
			write_dcs_transfer_trace(len, (const u8 *)data);
		else
			PANEL_ATRACE_INSTANT("dsi_dcs_transfer len:0 flags:0x%04x", msg.flags);
	}

	return ops->transfer(dsi->host, &msg);
}

static void gs_dcs_write_print_err(struct device *dev, const void *cmd, size_t len, ssize_t ret)
{
	dev_err(dev, "failed to write cmd (%ld)\n", ret);
	print_hex_dump(KERN_ERR, "command: ", DUMP_PREFIX_NONE, 16, 1, cmd, len, false);
}

ssize_t gs_dsi_dcs_write_buffer(struct mipi_dsi_device *dsi, const void *data, size_t len,
				u16 flags)
{
	ssize_t ret;
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

	ret = gs_dsi_dcs_transfer(dsi, type, data, len, flags);
	if (ret < 0)
		gs_dcs_write_print_err(&dsi->dev, data, len, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(gs_dsi_dcs_write_buffer);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)) || IS_ENABLED(CONFIG_DRM_DISPLAY_DP_HELPER)
int gs_dcs_write_dsc_config(struct device *dev, const struct drm_dsc_config *dsc_cfg)
{
	struct drm_dsc_picture_parameter_set pps;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	int ret;

	drm_dsc_pps_payload_pack(&pps, dsc_cfg);
	trace_dsi_tx(MIPI_DSI_PICTURE_PARAMETER_SET, (const u8 *)&pps, sizeof(pps), true, 0);
	PANEL_ATRACE_INSTANT("dsi_dcs_transfer len:%zu msg:pps_config", sizeof(pps));
	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0) {
		dev_err(dev, "failed to write pps(%d)\n", ret);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(gs_dcs_write_dsc_config);
#endif
