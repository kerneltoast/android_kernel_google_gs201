/*
 * drivers/media/platform/exynos/mfc/mfc_memlog.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __MFC_MEMLOG_H
#define __MFC_MEMLOG_H __FILE__

#if IS_ENABLED(CONFIG_EXYNOS_MEMORY_LOGGER)
#include <linux/platform_device.h>
#include "mfc_common.h"

#define MFC_MEMLOG_SIZE		(3*1024*1024)
#define MFC_MEMLOG_SFR_SIZE	(4*1024)
#define MEC_MEMLOG_LOG_SIZE	((MFC_MEMLOG_SIZE) - (MFC_NUM_CORE)*(MFC_MEMLOG_SFR_SIZE))

void mfc_dev_init_memlog(struct platform_device *pdev);
void mfc_core_init_memlog(struct platform_device *pdev);
void mfc_dev_deinit_memlog(struct mfc_dev *dev);
void mfc_core_deinit_memlog(struct mfc_core *core);
#else
#define memlog_register(...)		do {} while (0)
#define memlog_alloc_file(...)		do {} while (0)
#define memlog_alloc_printf(...)	do {} while (0)
#define memlog_alloc_dump(...)		do {} while (0)
#define memlog_write_printf(...)	do {} while (0)
#define memlog_do_dump(...)		do {} while (0)
#define mfc_dev_init_memlog(pdev)	do {} while (0)
#define mfc_core_init_memlog(pdev)	do {} while (0)
#define mfc_dev_deinit_memlog(dev)	do {} while (0)
#define mfc_core_deinit_memlog(core)	do {} while (0)
#endif

#endif /* __MFC_MEMLOG_H */
