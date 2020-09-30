/*
 * drivers/media/platform/exynos/mfc/mfc_core_qos.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_QOS_H
#define __MFC_CORE_QOS_H __FILE__

#include "mfc_common.h"

#define MB_COUNT_PER_UHD_FRAME		32400
#define MAX_FPS_PER_UHD_FRAME		120
#define MIN_BW_PER_SEC			1

#define MFC_DRV_TIME			500

#define MFC_QOS_TABLE_TYPE_DEFAULT	0
#define MFC_QOS_TABLE_TYPE_ENCODER	1

#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
#define MFC_THROUGHPUT_OFFSET	(PM_QOS_MFC1_THROUGHPUT - PM_QOS_MFC_THROUGHPUT)
void mfc_core_perf_boost_enable(struct mfc_core *core);
void mfc_core_perf_boost_disable(struct mfc_core *core);
void mfc_core_qos_on(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_qos_off(struct mfc_core *core, struct mfc_ctx *ctx);
#else
#define mfc_core_perf_boost_enable(core)	do {} while (0)
#define mfc_core_perf_boost_disable(core)	do {} while (0)
#define mfc_core_qos_on(core, ctx)		do {} while (0)
#define mfc_core_qos_off(core, ctx)		do {} while (0)
#endif

void mfc_core_qos_idle_worker(struct work_struct *work);
bool mfc_core_qos_idle_trigger(struct mfc_core *core, struct mfc_ctx *ctx);

#endif /* __MFC_CORE_QOS_H */
