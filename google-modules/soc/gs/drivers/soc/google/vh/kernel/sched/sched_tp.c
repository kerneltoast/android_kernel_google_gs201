/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/module.h>

#include <linux/sched.h>
#include <trace/events/sched.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/autogroup.h>
#include <kernel/sched/sched.h>

#define CREATE_TRACE_POINTS
#include "sched_events.h"

EXPORT_TRACEPOINT_SYMBOL_GPL(set_cpus_allowed_by_task);
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
EXPORT_TRACEPOINT_SYMBOL_GPL(sched_wakeup_task_attr);

static inline struct sched_avg *sched_trace_cfs_rq_avg(struct cfs_rq *cfs_rq)
{
#if IS_ENABLED(CONFIG_SMP)
	return cfs_rq ? &cfs_rq->avg : NULL;
#else
	return NULL;
#endif
}

static inline void cfs_rq_tg_path(struct cfs_rq *cfs_rq, char *path, int len)
{
	if (!path)
		return;

	if (cfs_rq && task_group_is_autogroup(cfs_rq->tg))
		autogroup_path(cfs_rq->tg, path, len);
	else if (cfs_rq && cfs_rq->tg->css.cgroup)
		cgroup_path(cfs_rq->tg->css.cgroup, path, len);
	else
		strlcpy(path, "(null)", len);
}

static inline char *sched_trace_cfs_rq_path(struct cfs_rq *cfs_rq, char *str, int len)
{
	if (!cfs_rq) {
		if (str)
			strlcpy(str, "(null)", len);
		else
			return NULL;
	}

	cfs_rq_tg_path(cfs_rq, str, len);
	return str;
}

static inline struct cfs_rq *get_group_cfs_rq(struct sched_entity *se)
{
#if IS_ENABLED(CONFIG_FAIR_GROUP_SCHED)
	return se->my_q;
#else
	return NULL;
#endif
}

static inline struct cfs_rq *get_se_cfs_rq(struct sched_entity *se)
{
#if IS_ENABLED(CONFIG_FAIR_GROUP_SCHED)
	return se->cfs_rq;
#else
	return NULL;
#endif
}

static inline int sched_trace_cfs_rq_cpu(struct cfs_rq *cfs_rq)
{
	return cfs_rq ? cpu_of(rq_of(cfs_rq)) : -1;
}

static inline struct sched_avg *sched_trace_rq_avg_rt(struct rq *rq)
{
#if IS_ENABLED(CONFIG_SMP)
	return rq ? &rq->avg_rt : NULL;
#else
	return NULL;
#endif
}

static inline struct sched_avg *sched_trace_rq_avg_dl(struct rq *rq)
{
#if IS_ENABLED(CONFIG_SMP)
	return rq ? &rq->avg_dl : NULL;
#else
	return NULL;
#endif
}

static inline struct sched_avg *sched_trace_rq_avg_irq(struct rq *rq)
{
#if IS_ENABLED(CONFIG_SMP) && IS_ENABLED(CONFIG_HAVE_SCHED_AVG_IRQ)
	return rq ? &rq->avg_irq : NULL;
#else
	return NULL;
#endif
}

static inline struct cpumask *sched_trace_rd_span(struct root_domain *rd)
{
#if IS_ENABLED(CONFIG_SMP)
	return rd ? rd->span : NULL;
#else
	return NULL;
#endif
}

static inline int sched_trace_rq_cpu(struct rq *rq)
{
	return rq ? cpu_of(rq) : -1;
}

static inline void _trace_cfs(struct cfs_rq *cfs_rq,
			      void (*trace_event)(int, char*,
						  const struct sched_avg*))
{
	const struct sched_avg *avg;
	char path[PATH_SIZE];
	int cpu;

	avg = sched_trace_cfs_rq_avg(cfs_rq);
	sched_trace_cfs_rq_path(cfs_rq, path, PATH_SIZE);
	cpu = sched_trace_cfs_rq_cpu(cfs_rq);

	trace_event(cpu, path, avg);
 }

static inline void _trace_se(struct sched_entity *se,
			     void (*trace_event)(int, char*, char*, int,
						 const struct sched_avg*))
{
	void *gcfs_rq = get_group_cfs_rq(se);
	void *cfs_rq = get_se_cfs_rq(se);
	struct task_struct *p;
	char path[PATH_SIZE];
	char *comm;
	pid_t pid;
	int cpu;

	sched_trace_cfs_rq_path(gcfs_rq, path, PATH_SIZE);
	cpu = sched_trace_cfs_rq_cpu(cfs_rq);

	p = gcfs_rq ? NULL : container_of(se, struct task_struct, se);
	comm = p ? p->comm : "(null)";
	pid = p ? p->pid : -1;

	trace_event(cpu, path, comm, pid, &se->avg);
}

static void sched_pelt_cfs(void *data, struct cfs_rq *cfs_rq)
{
	if (trace_sched_pelt_cfs_enabled())
		_trace_cfs(cfs_rq, trace_sched_pelt_cfs);
}

static void sched_pelt_rt(void *data, struct rq *rq)
{
	if (trace_sched_pelt_rt_enabled()) {
		const struct sched_avg *avg = sched_trace_rq_avg_rt(rq);
		int cpu = sched_trace_rq_cpu(rq);

		if (!avg)
			return;

		trace_sched_pelt_rt(cpu, avg);
	}
}

static void sched_pelt_dl(void *data, struct rq *rq)
{
	if (trace_sched_pelt_dl_enabled()) {
		const struct sched_avg *avg = sched_trace_rq_avg_dl(rq);
		int cpu = sched_trace_rq_cpu(rq);

		if (!avg)
			return;

		trace_sched_pelt_dl(cpu, avg);
	}
}

static void sched_pelt_irq(void *data, struct rq *rq)
{
	if (trace_sched_pelt_irq_enabled()){
		const struct sched_avg *avg = sched_trace_rq_avg_irq(rq);
		int cpu = sched_trace_rq_cpu(rq);

		if (!avg)
			return;

		trace_sched_pelt_irq(cpu, avg);
	}
}

static void sched_pelt_se(void *data, struct sched_entity *se)
{
	if (trace_sched_pelt_se_enabled())
		_trace_se(se, trace_sched_pelt_se);
}

static void sched_cpu_capacity(void *data, struct rq *rq)
{
	if (trace_sched_cpu_capacity_enabled())
		trace_sched_cpu_capacity(rq);
}

static void sched_overutilized(void *data, struct root_domain *rd, bool overutilized)
{
	if (trace_sched_overutilized_enabled()) {
		char span[SPAN_SIZE];

		cpumap_print_to_pagebuf(false, span, sched_trace_rd_span(rd));

		trace_sched_overutilized(overutilized, span);
	}
}

static void sched_util_est_cfs(void *data, struct cfs_rq *cfs_rq)
{
	if (trace_sched_util_est_cfs_enabled())
		_trace_cfs(cfs_rq, trace_sched_util_est_cfs);
}

static void sched_util_est_se(void *data, struct sched_entity *se)
{
	if (trace_sched_util_est_se_enabled())
		_trace_se(se, trace_sched_util_est_se);
}

static int sched_tp_init(void)
{
	register_trace_pelt_cfs_tp(sched_pelt_cfs, NULL);
	register_trace_pelt_rt_tp(sched_pelt_rt, NULL);
	register_trace_pelt_dl_tp(sched_pelt_dl, NULL);
	register_trace_pelt_irq_tp(sched_pelt_irq, NULL);
	register_trace_pelt_se_tp(sched_pelt_se, NULL);
	register_trace_sched_cpu_capacity_tp(sched_cpu_capacity, NULL);
	register_trace_sched_overutilized_tp(sched_overutilized, NULL);
	register_trace_sched_util_est_cfs_tp(sched_util_est_cfs, NULL);
	register_trace_sched_util_est_se_tp(sched_util_est_se, NULL);

	return 0;
}

static void sched_tp_finish(void)
{
	unregister_trace_pelt_cfs_tp(sched_pelt_cfs, NULL);
	unregister_trace_pelt_rt_tp(sched_pelt_rt, NULL);
	unregister_trace_pelt_dl_tp(sched_pelt_dl, NULL);
	unregister_trace_pelt_irq_tp(sched_pelt_irq, NULL);
	unregister_trace_pelt_se_tp(sched_pelt_se, NULL);
	unregister_trace_sched_cpu_capacity_tp(sched_cpu_capacity, NULL);
	unregister_trace_sched_overutilized_tp(sched_overutilized, NULL);
	unregister_trace_sched_util_est_cfs_tp(sched_util_est_cfs, NULL);
	unregister_trace_sched_util_est_se_tp(sched_util_est_se, NULL);
}


module_init(sched_tp_init);
module_exit(sched_tp_finish);

MODULE_LICENSE("GPL");
