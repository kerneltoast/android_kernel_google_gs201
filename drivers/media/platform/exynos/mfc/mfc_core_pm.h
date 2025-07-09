/*
 * drivers/media/platform/exynos/mfc/mfc_core_pm.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_PM_H
#define __MFC_CORE_PM_H __FILE__

#include "mfc_common.h"

static inline int mfc_core_pm_get_pwr_ref_cnt(struct mfc_core *core)
{
	return atomic_read(&core->pm.pwr_ref);
}

static inline int mfc_core_pm_get_clk_ref_cnt(struct mfc_core *core)
{
	return atomic_read(&core->clk_ref);
}

void mfc_core_pm_init(struct mfc_core *core);
void mfc_core_pm_final(struct mfc_core *core);

void mfc_core_protection_on(struct mfc_core *core);
void mfc_core_protection_off(struct mfc_core *core);
int mfc_core_pm_clock_on(struct mfc_core *core);
void mfc_core_pm_clock_off(struct mfc_core *core);
int mfc_core_pm_power_on(struct mfc_core *core);
int mfc_core_pm_power_off(struct mfc_core *core);
void mfc_core_pm_idle_suspend(struct mfc_core *core);
void mfc_core_pm_idle_resume(struct mfc_core *core);

#endif /* __MFC_CORE_PM_H */
