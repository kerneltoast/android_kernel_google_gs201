/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 Google LLC
 *
 */

#ifndef __EXYNOS_PMU_H
#define __EXYNOS_PMU_H __FILE__

#include <asm/smp_plat.h>

/**
 * struct exynos_cpu_power_ops
 *
 * CPU power control operations
 *
 * @power_up : Set cpu configuration register
 * @power_down : Clear cpu configuration register
 * @power_state : Show cpu status.
 *		Return true if cpu power on, otherwise return false.
 */
struct exynos_cpu_power_ops {
	void (*power_up)(unsigned int cpu);
	void (*power_down)(unsigned int cpu);
	int (*power_state)(unsigned int cpu);
	void (*cluster_up)(unsigned int cpu);
	void (*cluster_down)(unsigned int cpu);
	int (*cluster_state)(unsigned int cpu);
};
extern struct exynos_cpu_power_ops exynos_cpu;

#if defined(CONFIG_SOC_EXYNOS9820)
#define phy_cluster(cpu)	MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1)
#define phy_cpu(cpu)		MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 0)
#elif defined(CONFIG_SOC_GS101) || defined(CONFIG_SOC_GS201) || defined(CONFIG_SOC_ZUMA)
#define phy_cluster(cpu)	MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 2)
#define phy_cpu(cpu)		MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1)
#else
#error "Unknown CONFIG_SOC"
#endif

/**
 * The APIs to control the PMU
 */
#if IS_ENABLED(CONFIG_EXYNOS_PMU_IF)
extern int exynos_pmu_read(unsigned int offset,
			unsigned int *val);
extern int exynos_pmu_write(unsigned int offset,
			unsigned int val);
extern int exynos_pmu_update(unsigned int offset,
			unsigned int mask,
			unsigned int val);

extern void exynos_cpu_reset_enable(unsigned int cpu);
extern void exynos_cpu_reset_disable(unsigned int cpu);

#ifdef CONFIG_CP_PMUCAL
extern int exynos_check_cp_status(void);
#else
static inline int exynos_check_cp_status(void) { return 1; }
#endif

#else
static inline int exynos_pmu_read(unsigned int offset,
		unsigned int *val) {
	return 0;
}
static inline int exynos_pmu_write(unsigned int offset,
		unsigned int val) {
	return 0;
}
static inline int exynos_pmu_update(unsigned int offset, unsigned int mask,
		unsigned int val) {
	return 0;
}

static inline void exynos_cpu_reset_enable(unsigned int cpu) { return ; }
static inline void exynos_cpu_reset_disable(unsigned int cpu) { return ; }
static inline int exynos_check_cp_status(void) { return 1; }
#endif
#endif /* __EXYNOS_PMU_H */
