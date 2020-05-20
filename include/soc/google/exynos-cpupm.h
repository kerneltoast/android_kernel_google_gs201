/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_CPUPM_H
#define __EXYNOS_CPUPM_H __FILE__

extern int exynos_cpu_pm_enter(int cpu, int index);
extern void exynos_cpu_pm_exit(int cpu, int cancel);

enum {
	POWERMODE_TYPE_CLUSTER = 0,
	POWERMODE_TYPE_SYSTEM,
};

extern bool exynos_cpuhp_last_cpu(unsigned int cpu);

struct fix_idle_ip {
	/* name of fix-idle-ip */
	const char		*name;
	/* register index of fix-idle-ip */
	unsigned int		reg_index;
	/* non-idle count for cpuidle-profiler */
	unsigned int		count;
};

struct idle_ip {
	/* list of idle-ip */
	struct list_head	list;
	/* name of idle-ip */
	const char		*name;
	/* identity of idle-ip */
	unsigned int		index;
	/* non-idle count for cpuidle-profiler */
	unsigned int		count;
};

#ifdef CONFIG_CPU_IDLE
void exynos_update_ip_idle_status(int index, int idle);
int exynos_get_idle_ip_index(const char *name);
extern void disable_power_mode(int cpu, int type);
extern void enable_power_mode(int cpu, int type);
#else
static inline void exynos_update_ip_idle_status(int index, int idle)
{
}

static inline int exynos_get_idle_ip_index(const char *name)
{
	return 0;
}

static inline void disable_power_mode(int cpu, int type)
{
}

static inline void enable_power_mode(int cpu, int type)
{
}
#endif

#ifdef CONFIG_SMP
extern DEFINE_PER_CPU(bool, pending_ipi);
static inline bool is_IPI_pending(const struct cpumask *mask)
{
	unsigned int cpu;

	for_each_cpu_and(cpu, cpu_online_mask, mask) {
		if (per_cpu(pending_ipi, cpu))
			return true;
	}
	return false;
}
#else
static inline bool is_IPI_pending(const struct cpumask *mask)
{
	return false;
}
#endif
#endif /* __EXYNOS_CPUPM_H */
