/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __EXYNOS_CPU_COOLING_H__
#define __EXYNOS_CPU_COOLING_H__

#include <linux/of.h>
#include <linux/cpufreq.h>

#if IS_ENABLED(CONFIG_EXYNOS_CPU_THERMAL)
struct thermal_cooling_device *
exynos_cpufreq_cooling_register(struct device_node *np, struct cpufreq_policy *policy);
#else
static inline struct thermal_cooling_device *
exynos_cpufreq_cooling_register(struct device_node *np, struct cpufreq_policy *policy)
{
	return NULL;
}
#endif
#endif /* __EXYNOS_CPU_COOLING_H__ */
