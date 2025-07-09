/*
 * drivers/media/platform/exynos/mfc/mfc_core_hw_reg_api.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <trace/events/mfc.h>

#include "mfc_core_hw_reg_api.h"
#include "mfc_core_pm.h"

/* Reset the device */
void mfc_core_reg_clear(struct mfc_core *core)
{
	int i;

	/* Zero Initialization of MFC registers */
	MFC_CORE_WRITEL(0, MFC_REG_RISC2HOST_CMD);
	MFC_CORE_WRITEL(0, MFC_REG_HOST2RISC_CMD);
	MFC_CORE_WRITEL(0, MFC_REG_FW_VERSION);

	for (i = 0; i < MFC_REG_CLEAR_COUNT; i++)
		MFC_CORE_WRITEL(0, MFC_REG_CLEAR_BEGIN + (i*4));
}

void mfc_core_reset_mfc(struct mfc_core *core, enum mfc_buf_usage_type buf_type)
{
	mfc_core_debug_enter();

	if (!(core->dev->pdata->security_ctrl && (buf_type == MFCBUF_DRM))) {
		MFC_CORE_WRITEL(0x1FFF, MFC_REG_MFC_RESET);
		MFC_CORE_WRITEL(0, MFC_REG_MFC_RESET);
	}

	mfc_core_debug_leave();
}

void mfc_core_set_risc_base_addr(struct mfc_core *core,
					enum mfc_buf_usage_type buf_type)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_special_buf *fw_buf;

	fw_buf = &core->fw_buf;

	if (buf_type == MFCBUF_DRM)
		fw_buf = &core->drm_fw_buf;

	if (dev->pdata->security_ctrl) {
		if (buf_type == MFCBUF_DRM)
			mfc_core_debug(2, "[MEMINFO][F/W] do not set secure base address\n");
		else
			MFC_CORE_WRITEL(fw_buf->daddr, MFC_REG_RISC_NONSECURE_BASE_ADDR);
	} else {
		MFC_CORE_WRITEL(fw_buf->daddr, MFC_REG_RISC_BASE_ADDRESS);
	}

	mfc_core_debug(2, "[MEMINFO][F/W] %s Base Address : %#x\n",
			buf_type == MFCBUF_DRM ? "DRM" : "NORMAL", (u32)fw_buf->daddr);
	MFC_TRACE_CORE("%s F/W Base Address : %#x\n",
			buf_type == MFCBUF_DRM ? "DRM" : "NORMAL", (u32)fw_buf->daddr);
}

void mfc_core_cmd_host2risc(struct mfc_core *core, int cmd)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[core->curr_core_ctx];
	int ret = 0;

	mfc_core_debug(1, "Issue the command: %d%s\n",
			cmd, core->cache_flush_flag ? " with cache flush" : "");
	MFC_TRACE_CORE_CTX(">> CMD : %d, (dev:0x%lx, bits:%lx, owned:%d, wl:%d, trans:%d)\n",
			cmd, core->hwlock.dev, core->hwlock.bits, core->hwlock.owned_by_irq,
			core->hwlock.wl_count, core->hwlock.transfer_owner);
	MFC_TRACE_LOG_CORE("C%d", cmd);

	if (core->cache_flush_flag) {
		MFC_TRACE_CORE_CTX(">> CMD : 12 in FW\n");
		MFC_TRACE_LOG_CORE("C12FW");
	}

	trace_mfc_frame_start(core->curr_core_ctx, cmd, 0, 0);
	/* Reset RISC2HOST command except nal q stop command */
	if (cmd != MFC_REG_H2R_CMD_STOP_QUEUE)
		MFC_CORE_WRITEL(0x0, MFC_REG_RISC2HOST_CMD);

	if ((cmd != MFC_REG_H2R_CMD_NAL_QUEUE) && (cmd != MFC_REG_H2R_CMD_STOP_QUEUE)) {
		/* TODO: change to core */
		/* Start the timeout meerkat */
		mfc_core_meerkat_start_tick(core);
		if (cmd != MFC_REG_H2R_CMD_NAL_ABORT) {
			/* Check the fw status */
			ret = mfc_core_wait_fw_status(core);
			if (ret != 0) {
				mfc_err("Failed to wait firmware status\n");
				call_dop(core, dump_and_stop_always, core);
			}
		}
	}

	if (dbg_enable) {
		/* For FW debugging */
		mfc_core_dbg_set_addr(core);
		mfc_core_dbg_enable(core);
	}

	core->last_cmd = cmd;
	core->last_cmd_time = ktime_to_timespec64(ktime_get());

	/* Record if the command incurs cache flush */
	core->last_cmd_has_cache_flush =
		(cmd == MFC_REG_H2R_CMD_CACHE_FLUSH
		 || core->cache_flush_flag) ? 1 : 0;

	/* Issue the command */
	if (!core->cache_flush_flag)
		MFC_CORE_WRITEL(cmd, MFC_REG_HOST2RISC_CMD);
	else
		MFC_CORE_WRITEL((cmd | (1 << MFC_REG_H2R_CACHE_FLUSH_FLAG)),
				MFC_REG_HOST2RISC_CMD);
	core->cache_flush_flag = 0;

	MFC_CORE_WRITEL(0x1, MFC_REG_HOST2RISC_INT);
}

/* Check whether HW interrupt has occurred or not */
int mfc_core_check_risc2host(struct mfc_core *core)
{
	if (mfc_core_pm_get_pwr_ref_cnt(core) && mfc_core_pm_get_clk_ref_cnt(core)) {
		if (MFC_CORE_READL(MFC_REG_RISC2HOST_INT))
			return MFC_CORE_READL(MFC_REG_RISC2HOST_CMD);
		else
			return 0;
	}

	return 0;
}

void mfc_core_set_gdc_votf(struct mfc_core *core, struct mfc_ctx *ctx)
{
	unsigned int mfc_votf_base = (core->core_pdata->mfc_votf_base >> 16) & 0xFFFF;

	mfc_core_debug(2, "[vOTF] set GDC-MFC vOTF\n");

	/* vOTF enable */
	VOTF_WRITEL(0x1, 0x000C);
	VOTF_WRITEL(0x1, 0x0010);
	VOTF_WRITEL(mfc_votf_base, 0x0014);
	VOTF_WRITEL(0x1, 0x0028);
	VOTF_WRITEL(0x1, 0x0024);

	/* TRS0 setting */
	VOTF_WRITEL(0x3, 0x0304);
	VOTF_WRITEL(0x1, 0x0308);
	VOTF_WRITEL(0x2, 0x0314);
	VOTF_WRITEL(0x2, 0x0318);
	VOTF_WRITEL((ctx->crop_height + 31) / 32, 0x031C);
	VOTF_WRITEL(0x1, 0x0300);

	/* TRS1 setting */
	VOTF_WRITEL(0x3, 0x0304 + 0x2C);
	VOTF_WRITEL(0x1, 0x0308 + 0x2C);
	VOTF_WRITEL(0x2, 0x0314 + 0x2C);
	VOTF_WRITEL(0x2, 0x0318 + 0x2C);
	VOTF_WRITEL((ctx->crop_height + 31) / 32, 0x031C + 0x2C);
	VOTF_WRITEL(0x1, 0x0300 + 0x2C);
}

void mfc_core_set_dpu_votf(struct mfc_core *core, struct mfc_ctx *ctx)
{
	unsigned int mfc_votf_base = (core->core_pdata->mfc_votf_base >> 16) & 0xFFFF;
	unsigned int buf_cnt = ctx->otf_handle->otf_buf_info.buffer_count;
	unsigned int i;

	mfc_core_debug(2, "[vOTF] set DPU-MFC vOTF\n");

	/* vOTF enable */
	VOTF_WRITEL(0x1, 0x000C);
	VOTF_WRITEL(0x1, 0x0010);
	VOTF_WRITEL(mfc_votf_base, 0x0014);
	VOTF_WRITEL(0x1, 0x0028);
	VOTF_WRITEL(0x1, 0x0024);

	/* TRS setting */
	for (i = 0; i < buf_cnt; i++) {
		VOTF_WRITEL(0x1, 0x0300 + (i * 0x2c));
		VOTF_WRITEL(0x3, 0x0304 + (i * 0x2c));
		VOTF_WRITEL(0xFF, 0x0308 + (i * 0x2c));
		VOTF_WRITEL(0x0, 0x0310 + (i * 0x2c));
		VOTF_WRITEL(0x1, 0x0314 + (i * 0x2c));
		VOTF_WRITEL(0x1, 0x0318 + (i * 0x2c));
		VOTF_WRITEL((ctx->crop_height + 31 / 32), 0x031C + (i * 0x2c));
	}
}

void mfc_core_clear_votf(struct mfc_core *core)
{
	mfc_core_debug(2, "[vOTF] clear vOTF\n");

	/* vOTF disable, Do not clear 0x0010 */
	VOTF_WRITEL(0x0, 0x000C);
}
