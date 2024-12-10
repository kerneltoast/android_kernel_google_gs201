/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 Sultan Alsawaf <sultan@kerneltoast.com>.
 */
#ifndef _TENSOR_AIO_H_
#define _TENSOR_AIO_H_

struct exynos_cpufreq_domain;
struct rq;

#ifdef CONFIG_ARM_TENSOR_AIO_DEVFREQ
void tensor_aio_init_cpu_domain(struct exynos_cpufreq_domain *domain);
void tensor_aio_cpufreq_pressure(int cpu, unsigned int cap);
void tensor_aio_update_rq_clock(struct rq *rq);
#else
static inline void tensor_aio_init_cpu_domain(struct exynos_cpufreq_domain *domain)
{
}
static inline void tensor_aio_cpufreq_pressure(int cpu, unsigned int cap)
{
}
static inline void tensor_aio_update_rq_clock(struct rq *rq)
{
}
#endif

#endif /* _TENSOR_AIO_H_ */
