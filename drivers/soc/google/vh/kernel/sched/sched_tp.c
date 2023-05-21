/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/module.h>

#include <linux/sched.h>
#include <trace/events/sched.h>
#include <kernel/sched/sched.h>

#define CREATE_TRACE_POINTS
#include "sched_events.h"

EXPORT_TRACEPOINT_SYMBOL_GPL(cpumask_any_and_distribute);
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_find_energy_efficient_cpu);
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_select_task_rq_fair);
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_cpu_util_cfs);
EXPORT_TRACEPOINT_SYMBOL_GPL(sugov_util_update);
EXPORT_TRACEPOINT_SYMBOL_GPL(sugov_next_freq);
EXPORT_TRACEPOINT_SYMBOL_GPL(schedutil_cpu_util_clamp);
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_cpu_util_rt);
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_find_least_loaded_cpu);
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_select_task_rq_rt);
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_setscheduler_uclamp);
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_compute_energy);
static inline struct cfs_rq *get_group_cfs_rq(struct sched_entity *se)
{
#ifdef CONFIG_FAIR_GROUP_SCHED
	return se->my_q;
#else
	return NULL;
#endif
}

static inline struct cfs_rq *get_se_cfs_rq(struct sched_entity *se)
{
#ifdef CONFIG_FAIR_GROUP_SCHED
	return se->cfs_rq;
#else
	return NULL;
#endif
}

static void sched_pelt_rt(void *data, struct rq *rq)
{
	const struct sched_avg *avg = sched_trace_rq_avg_rt(rq);
	int cpu = sched_trace_rq_cpu(rq);

	if (!avg)
		return;

	trace_sched_pelt_rt(cpu, avg);
}

static void sched_pelt_dl(void *data, struct rq *rq)
{
	const struct sched_avg *avg = sched_trace_rq_avg_dl(rq);
	int cpu = sched_trace_rq_cpu(rq);

	if (!avg)
		return;

	trace_sched_pelt_dl(cpu, avg);
}

static void sched_pelt_irq(void *data, struct rq *rq)
{
	const struct sched_avg *avg = sched_trace_rq_avg_irq(rq);
	int cpu = sched_trace_rq_cpu(rq);

	if (!avg)
		return;

	trace_sched_pelt_irq(cpu, avg);
}

static void sched_cpu_capacity(void *data, struct rq *rq)
{
	trace_sched_cpu_capacity(rq);
}

static void sched_overutilized(void *data, struct root_domain *rd, bool overutilized)
{
	char span[SPAN_SIZE];

	cpumap_print_to_pagebuf(false, span, sched_trace_rd_span(rd));

	trace_sched_overutilized(overutilized, span);
}

static int sched_tp_init(void)
{
	register_trace_pelt_rt_tp(sched_pelt_rt, NULL);
	register_trace_pelt_dl_tp(sched_pelt_dl, NULL);
	register_trace_pelt_irq_tp(sched_pelt_irq, NULL);
	register_trace_sched_cpu_capacity_tp(sched_cpu_capacity, NULL);
	register_trace_sched_overutilized_tp(sched_overutilized, NULL);

	return 0;
}

static void sched_tp_finish(void)
{
	unregister_trace_pelt_rt_tp(sched_pelt_rt, NULL);
	unregister_trace_pelt_dl_tp(sched_pelt_dl, NULL);
	unregister_trace_pelt_irq_tp(sched_pelt_irq, NULL);
	unregister_trace_sched_cpu_capacity_tp(sched_cpu_capacity, NULL);
	unregister_trace_sched_overutilized_tp(sched_overutilized, NULL);
}


module_init(sched_tp_init);
module_exit(sched_tp_finish);

MODULE_LICENSE("GPL");
