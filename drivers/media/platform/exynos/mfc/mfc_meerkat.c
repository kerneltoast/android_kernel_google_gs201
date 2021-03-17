/*
 * drivers/media/platform/exynos/mfc/mfc_meerkat.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "mfc_meerkat.h"

#include "mfc_core_meerkat.h"

static void __mfc_dump_trace_longterm(struct mfc_dev *dev)
{
	int i, cnt, trace_cnt;

	dev_err(dev->device, "-----------dumping MFC trace long-term info-----------\n");

	trace_cnt = atomic_read(&dev->trace_ref_longterm);
	for (i = MFC_TRACE_COUNT_PRINT - 1; i >= 0; i--) {
		cnt = ((trace_cnt + MFC_TRACE_COUNT_MAX) - i) % MFC_TRACE_COUNT_MAX;
		dev_err(dev->device, "MFC trace longterm[%d]: time=%llu, str=%s", cnt,
				dev->mfc_trace_longterm[cnt].time, dev->mfc_trace_longterm[cnt].str);
	}
}

static void __mfc_dump_info_context(struct mfc_dev *dev)
{
	mfc_dump_state(dev);
	__mfc_dump_trace_longterm(dev);
}

static void __mfc_dump_info_and_stop_hw_debug(struct mfc_dev *dev)
{
	struct mfc_core *core = dev->core[0];

	if (!dev->pdata->debug_mode && !debug_mode_en)
		return;

	strlcpy(core->crash_info, dev->dev_crash_info, sizeof(core->crash_info));
	call_dop(core, dump_and_stop_debug_mode, core);
}

struct mfc_dump_ops mfc_dump_ops = {
	.dump_info_context		= __mfc_dump_info_context,
	.dump_and_stop_debug_mode	= __mfc_dump_info_and_stop_hw_debug,
};
