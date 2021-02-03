/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * GS101 SoC CPU Hotplug CONTROL support
 *
 */

#ifndef __EXYNOS_CPU_HP_H
#define __EXYNOS_CPU_HP_H __FILE__

#define CPUHP_USER_NAME_LEN	16

#if IS_ENABLED(CONFIG_EXYNOS_CPUHP)
extern int exynos_cpuhp_unregister(char *name, struct cpumask mask);
extern int exynos_cpuhp_register(char *name, struct cpumask mask);
extern int exynos_cpuhp_request(char *name, struct cpumask mask);
#else
static inline int exynos_cpuhp_unregister(char *name, struct cpumask mask)
{ return -1; }
static inline int exynos_cpuhp_register(char *name, struct cpumask mask)
{ return -1; }
static inline int exynos_cpuhp_request(char *name, struct cpumask mask)
{ return -1; }
#endif

#endif /* __EXYNOS_CPU_HP_H */
