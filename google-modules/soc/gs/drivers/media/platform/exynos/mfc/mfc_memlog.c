/*
 * drivers/media/platform/exynos/mfc/mfc_memlog.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#if IS_ENABLED(CONFIG_EXYNOS_MEMORY_LOGGER)

#include "mfc_memlog.h"
#include "mfc_common.h"

static int mfc_memlog_file_completed(struct memlog_obj *obj, u32 flags)
{
	/* NOP */
	return 0;
}

static int mfc_memlog_status_notify(struct memlog_obj *obj, u32 flags)
{
	/* NOP */
	return 0;
}

static int mfc_memlog_level_notify(struct memlog_obj *obj, u32 flags)
{
	/* NOP */
	return 0;
}

static int mfc_memlog_enable_notify(struct memlog_obj *obj, u32 flags)
{
	/* NOP */
	return 0;
}

static const struct memlog_ops mfc_memlog_ops = {
	.file_ops_completed = mfc_memlog_file_completed,
	.log_status_notify = mfc_memlog_status_notify,
	.log_level_notify = mfc_memlog_level_notify,
	.log_enable_notify = mfc_memlog_enable_notify,
};

void mfc_dev_init_memlog(struct platform_device *pdev)
{
	struct mfc_dev *dev = platform_get_drvdata(pdev);
	struct mfc_dev_memlog *memlog = &dev->memlog;
	struct memlog *desc;
	struct memlog_obj *log_obj;
	int ret;

	ret = memlog_register("MFC", &pdev->dev, &desc);
	if (ret) {
		mfc_dev_err("failed to register dev memlog\n");
		return;
	}

	memlog->desc = desc;
	desc->ops = mfc_memlog_ops;

	log_obj = memlog_alloc_printf(desc,
					MEC_MEMLOG_LOG_SIZE,
					NULL,
					"log-mem",
					0);

	if (log_obj) {
		memlog->log_obj = log_obj;
		memlog->log_enable = 1;
	} else {
		mfc_dev_err("failed to alloc dev memlog memory for log\n");
		return;
	}
}

void mfc_core_init_memlog(struct platform_device *pdev)
{
	struct mfc_core *core = platform_get_drvdata(pdev);
	struct mfc_dev *dev = core->dev;
	struct mfc_core_memlog *memlog = &core->memlog;
	struct memlog *desc = dev->memlog.desc;
	struct memlog_obj *sfr_obj;
	/* dump 0xF000~0xFFF8 */
	int offset = 0xF000;

	snprintf(memlog->sfr_obj_name, sizeof(memlog->sfr_obj_name), "sfr-dmp%d", core->id);

	sfr_obj = memlog_alloc_dump(desc,
					MFC_MEMLOG_SFR_SIZE,
					core->mfc_mem->start + offset,
					true,
					NULL,
					memlog->sfr_obj_name);
	if (sfr_obj) {
		memlog->sfr_obj = sfr_obj;
		memlog->sfr_enable = 1;
	} else {
		mfc_core_err("failed to alloc core memlog for sfr dump\n");
		return;
	}
}

void mfc_dev_deinit_memlog(struct mfc_dev *dev)
{
	dev->memlog.log_enable = 0;
}

void mfc_core_deinit_memlog(struct mfc_core *core)
{
	core->memlog.sfr_enable = 0;
}
#endif
