/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_CPUPM_H
#define __EXYNOS_CPUPM_H __FILE__

enum {
	C2_ENTER,
	C2_EXIT,
	SICD_ENTER,
	SICD_EXIT,
};

enum {
	POWERMODE_TYPE_CLUSTER = 0,
	POWERMODE_TYPE_SYSTEM,
	POWERMODE_TYPE_END,
};

#if IS_ENABLED(CONFIG_EXYNOS_CPUPM)
extern int exynos_sicd_notifier_register(struct notifier_block *nb);
extern void exynos_update_ip_idle_status(int index, int idle);
extern int exynos_get_idle_ip_index(const char *name);
extern void disable_power_mode(int cpu, int type);
extern void enable_power_mode(int cpu, int type);
#if defined(CONFIG_SOC_GS101) || defined(CONFIG_SOC_GS201)
extern bool system_is_in_itmon;
#endif
#else
static inline int exynos_sicd_notifier_register(struct notifier_block *nb) { return 0; }
static inline void exynos_update_ip_idle_status(int index, int idle) { return; }
static inline int exynos_get_idle_ip_index(const char *name) { return 0; }
static inline void disable_power_mode(int cpu, int type) { return; }
static inline void enable_power_mode(int cpu, int type) { return; }
#endif

#define PMU_ALLOWED_C2		(1)
#define PMU_INFORM0		(0x800)

#endif /* __EXYNOS_CPUPM_H */
