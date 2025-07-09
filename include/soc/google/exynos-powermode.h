/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_POWERMODE_H
#define __EXYNOS_POWERMODE_H __FILE__
#include <soc/google/cal-if.h>

extern int exynos_prepare_sys_powerdown(enum sys_powerdown mode);
extern void exynos_wakeup_sys_powerdown(enum sys_powerdown mode,
		bool early_wakeup);
extern void exynos_prepare_cp_call(void);
extern void exynos_wakeup_cp_call(bool early_wakeup);
extern int exynos_rtc_wakeup(void);

extern int exynos_system_idle_enter(void);
extern void exynos_system_idle_exit(int cancel);

/**
 * external driver APIs
 */

#if IS_ENABLED(CONFIG_PINCTRL_EXYNOS_GS)
extern u64 exynos_get_eint_wake_mask(void);
#else
static inline u64 exynos_get_eint_wake_mask(void) { return ULLONG_MAX; }

#endif

#endif /* __EXYNOS_POWERMODE_H */
