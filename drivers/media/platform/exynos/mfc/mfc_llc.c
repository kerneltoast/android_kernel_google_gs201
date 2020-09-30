/*
 * drivers/media/platform/exynos/mfc/mfc_llc.c
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#if IS_ENABLED(CONFIG_MFC_USES_LLC)

#include <linux/module.h>
#include <soc/samsung/exynos-sci.h>

#include "mfc_llc.h"

void mfc_llc_enable(struct mfc_core *core)
{
	mfc_core_debug_enter();

	if (llc_disable)
		return;

	/* region_index, enable, cache way */
	llc_region_alloc(LLC_REGION_MFC0_INT + core->id, 1, 1);
	llc_region_alloc(LLC_REGION_MFC0_DPB + core->id, 1, 1);
	core->llc_on_status = 1;
	mfc_core_info("[LLC] enabled\n");
	MFC_TRACE_CORE("[LLC] enabled\n");

	mfc_core_debug_leave();
}

void mfc_llc_disable(struct mfc_core *core)
{
	mfc_core_debug_enter();

	llc_region_alloc(LLC_REGION_MFC0_INT + core->id, 0, 0);
	llc_region_alloc(LLC_REGION_MFC0_DPB + core->id, 0, 0);
	core->llc_on_status = 0;
	mfc_core_info("[LLC] disabled\n");
	MFC_TRACE_CORE("[LLC] disabled\n");

	mfc_core_debug_leave();
}

void mfc_llc_flush(struct mfc_core *core)
{
	mfc_core_debug_enter();

	if (llc_disable)
		return;

	if (!core->need_llc_flush)
		return;

	llc_flush(LLC_REGION_MFC0_INT + core->id);
	llc_flush(LLC_REGION_MFC0_DPB + core->id);
	mfc_core_debug(2, "[LLC] flushed\n");
	MFC_TRACE_CORE("[LLC] flushed\n");

	mfc_core_debug_leave();
}

void mfc_llc_update_size(struct mfc_core *core, bool sizeup)
{
	int way;

	mfc_core_debug_enter();

	if (llc_disable)
		return;

	/* When 8K resolution, the LLC size needs 2way (1MB) for DPB */
	way = sizeup ? 2 : 1;

	llc_region_alloc(LLC_REGION_MFC0_DPB + core->id, 0, 0);
	llc_region_alloc(LLC_REGION_MFC0_DPB + core->id, 1, way);

	mfc_core_debug(2, "[LLC] update LLC %dway for DPB\n", way);
	MFC_TRACE_CORE("[LLC] update LLC %dway for DPB\n", way);

	mfc_core_debug_leave();
}

void mfc_llc_handle_resol(struct mfc_core *core, struct mfc_ctx *ctx)
{
	if (IS_8K_RES(ctx)) {
		ctx->is_8k = 1;
		mfc_core_debug(2, "[LLC] is_8k %d\n", ctx->is_8k);
		mfc_llc_update_size(core, true);
	}

	if (core->num_inst == 1 && UNDER_FHD_RES(ctx)) {
		mfc_core_debug(2, "[LLC] disable LLC for under FHD\n");
		mfc_llc_disable(core);
	}
}
#endif
