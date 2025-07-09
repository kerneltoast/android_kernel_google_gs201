/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM sched

#define MAX_SPAN_SIZE		128

#if !defined(_SCHED_EVENTS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _SCHED_EVENTS_H

#define PATH_SIZE		64
#define __SPAN_SIZE		(round_up(NR_CPUS, 4)/4)
#define SPAN_SIZE		(__SPAN_SIZE > MAX_SPAN_SIZE ? MAX_SPAN_SIZE : __SPAN_SIZE)

#include <linux/tracepoint.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,6,0)
#define RBL_LOAD_ENTRY		rbl_load
#define RBL_LOAD_MEMBER		runnable_load_avg
#define RBL_LOAD_STR		"rbl_load"
#else
#define RBL_LOAD_ENTRY		runnable
#define RBL_LOAD_MEMBER		runnable_avg
#define RBL_LOAD_STR		"runnable"
#endif

#if !defined(_SCHED_CPU_CANDIDATES)
#define _SCHED_CPU_CANDIDATES
enum cpu_candidates {
	IDLE_FIT = 0,
	IDLE_UNFIT,
	UNIMPORTANT_FIT,
	UNIMPORTANT_UNFIT,
	MAX_SPARE_CAP,
	PACKING,
	IDLE_UNPREFERRED,
	MAX_SPARE_CAP_RUNNING_RT,
	CC_MAX
};
#endif

TRACE_EVENT(sched_pelt_cfs,

	TP_PROTO(int cpu, char *path, const struct sched_avg *avg),

	TP_ARGS(cpu, path, avg),

	TP_STRUCT__entry(
		__field(	int,		cpu			)
		__array(	char,		path,	PATH_SIZE	)
		__field(	unsigned long,	load			)
		__field(	unsigned long,	RBL_LOAD_ENTRY		)
		__field(	unsigned long,	util			)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		strlcpy(__entry->path, path, PATH_SIZE);
		__entry->load		= avg->load_avg;
		__entry->RBL_LOAD_ENTRY	= avg->RBL_LOAD_MEMBER;
		__entry->util		= avg->util_avg;
	),

	TP_printk("cpu=%d path=%s load=%lu " RBL_LOAD_STR "=%lu util=%lu",
		  __entry->cpu, __entry->path, __entry->load,
		  __entry->RBL_LOAD_ENTRY,__entry->util)
);

DECLARE_EVENT_CLASS(sched_pelt_rq_template,

	TP_PROTO(int cpu, const struct sched_avg *avg),

	TP_ARGS(cpu, avg),

	TP_STRUCT__entry(
		__field(	int,		cpu			)
		__field(	unsigned long,	load			)
		__field(	unsigned long,	RBL_LOAD_ENTRY		)
		__field(	unsigned long,	util			)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->load		= avg->load_avg;
		__entry->RBL_LOAD_ENTRY	= avg->RBL_LOAD_MEMBER;
		__entry->util		= avg->util_avg;
	),

	TP_printk("cpu=%d load=%lu " RBL_LOAD_STR "=%lu util=%lu",
		  __entry->cpu, __entry->load,
		  __entry->RBL_LOAD_ENTRY,__entry->util)
);

DEFINE_EVENT(sched_pelt_rq_template, sched_pelt_rt,
	TP_PROTO(int cpu, const struct sched_avg *avg),
	TP_ARGS(cpu, avg));

DEFINE_EVENT(sched_pelt_rq_template, sched_pelt_dl,
	TP_PROTO(int cpu, const struct sched_avg *avg),
	TP_ARGS(cpu, avg));

DEFINE_EVENT(sched_pelt_rq_template, sched_pelt_irq,
	TP_PROTO(int cpu, const struct sched_avg *avg),
	TP_ARGS(cpu, avg));

TRACE_EVENT(sched_cpu_capacity,

	TP_PROTO(struct rq *rq),

	TP_ARGS(rq),

	TP_STRUCT__entry(
		__field(int, cpu)
		__field(unsigned long, capacity)
		__field(unsigned long, capacity_orig)
	),

	TP_fast_assign(
		__entry->cpu			= rq->cpu;
		__entry->capacity		= rq->cpu_capacity;
		__entry->capacity_orig		= rq->cpu_capacity_orig;
	),

	TP_printk("cpu=%d capacity=%lu, capacity_orig=%lu",
		__entry->cpu, __entry->capacity, __entry->capacity_orig)
);

TRACE_EVENT(sched_pelt_se,

	TP_PROTO(int cpu, char *path, char *comm, int pid, const struct sched_avg *avg),

	TP_ARGS(cpu, path, comm, pid, avg),

	TP_STRUCT__entry(
		__field(	int,		cpu			)
		__array(	char,		path,	PATH_SIZE	)
		__array(	char,		comm,	TASK_COMM_LEN	)
		__field(	int,		pid			)
		__field(	unsigned long,	load			)
		__field(	unsigned long,	RBL_LOAD_ENTRY		)
		__field(	unsigned long,	util			)
		__field(	unsigned long long, update_time	        )
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		strlcpy(__entry->path, path, PATH_SIZE);
		strlcpy(__entry->comm, comm, TASK_COMM_LEN);
		__entry->pid		= pid;
		__entry->load		= avg->load_avg;
		__entry->RBL_LOAD_ENTRY	= avg->RBL_LOAD_MEMBER;
		__entry->util		= avg->util_avg;
		__entry->update_time    = avg->last_update_time;
	),

	TP_printk("cpu=%d path=%s comm=%s pid=%d load=%lu " RBL_LOAD_STR "=%lu util=%lu update_time=%llu",
		  __entry->cpu, __entry->path, __entry->comm, __entry->pid,
		  __entry->load, __entry->RBL_LOAD_ENTRY,__entry->util, __entry->update_time)
);

TRACE_EVENT(sched_overutilized,

	TP_PROTO(int overutilized, char *span),

	TP_ARGS(overutilized, span),

	TP_STRUCT__entry(
		__field(	int,		overutilized		)
		__array(	char,		span,	SPAN_SIZE	)
	),

	TP_fast_assign(
		__entry->overutilized	= overutilized;
		strlcpy(__entry->span, span, SPAN_SIZE);
	),

	TP_printk("overutilized=%d span=0x%s",
		  __entry->overutilized, __entry->span)
);

TRACE_EVENT(sched_util_est_se,

	TP_PROTO(int cpu, char *path, char *comm, int pid,
		 const struct sched_avg *avg),

	TP_ARGS(cpu, path, comm, pid, avg),

	TP_STRUCT__entry(
		__field(	int,		cpu			)
		__array(	char,		path,	PATH_SIZE	)
		__array(	char,		comm,	TASK_COMM_LEN	)
		__field(	int,		pid			)
		__field( 	unsigned int,	enqueued		)
		__field( 	unsigned int,	ewma			)
		__field(	unsigned long,	util			)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		strlcpy(__entry->path, path, PATH_SIZE);
		strlcpy(__entry->comm, comm, TASK_COMM_LEN);
		__entry->pid		= pid;
		__entry->enqueued	= avg->util_est.enqueued & ~UTIL_AVG_UNCHANGED;
		__entry->ewma		= avg->util_est.ewma;
		__entry->util		= avg->util_avg;
	),

	TP_printk("cpu=%d path=%s comm=%s pid=%d enqueued=%u ewma=%u util=%lu",
		  __entry->cpu, __entry->path, __entry->comm, __entry->pid,
		  __entry->enqueued, __entry->ewma, __entry->util)
);

TRACE_EVENT(sched_util_est_cfs,

	TP_PROTO(int cpu, char *path, const struct sched_avg *avg),

	TP_ARGS(cpu, path, avg),

	TP_STRUCT__entry(
		__field(	int,		cpu			)
		__array(	char,		path,	PATH_SIZE	)
		__field( 	unsigned int,	enqueued		)
		__field( 	unsigned int,	ewma			)
		__field(	unsigned long,	util			)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		strlcpy(__entry->path, path, PATH_SIZE);
		__entry->enqueued	= avg->util_est.enqueued & ~UTIL_AVG_UNCHANGED;
		__entry->ewma		= avg->util_est.ewma;
		__entry->util		= avg->util_avg;
	),

	TP_printk("cpu=%d path=%s enqueued=%u ewma=%u util=%lu",
		  __entry->cpu, __entry->path, __entry->enqueued,
		 __entry->ewma, __entry->util)
);

TRACE_EVENT(sched_compute_energy,

	TP_PROTO(struct task_struct *tsk, int dst_cpu, unsigned long energy),

	TP_ARGS(tsk, dst_cpu, energy),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t,	pid)
		__field(int,	dst_cpu)
		__field(unsigned long,	energy)
	),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid             = tsk->pid;
		__entry->dst_cpu         = dst_cpu;
		__entry->energy          = energy;
	),

	TP_printk("pid=%d comm=%s dst_cpu=%d, energy=%lu",
		__entry->pid, __entry->comm, __entry->dst_cpu, __entry->energy)
);

TRACE_EVENT(sched_setscheduler_uclamp,

	TP_PROTO(struct task_struct *tsk, int clamp_id, unsigned int value),

	TP_ARGS(tsk, clamp_id, value),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t,	pid)
		__field(int,	clamp_id)
		__field(unsigned int,	value)
	),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid             = tsk->pid;
		__entry->clamp_id        = clamp_id;
		__entry->value           = value;
	),

	TP_printk("pid=%d comm=%s clamp_id=%d, value=%u",
		__entry->pid, __entry->comm, __entry->clamp_id, __entry->value)
);

TRACE_EVENT(set_cpus_allowed_by_task,

	TP_PROTO(struct task_struct *tsk, cpumask_t *valid_mask, int dest_cpu),

	TP_ARGS(tsk, valid_mask, dest_cpu),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t,			pid)
		__field(pid_t,			tgid)
		__field(unsigned int,	state)
		__field(unsigned long,	valid_mask)
		__field(int,			dest_cpu)
	),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid             	= tsk->pid;
		__entry->tgid             	= tsk->tgid;
		__entry->state             	= tsk->__state;
		__entry->valid_mask       	= *valid_mask->bits;
		__entry->dest_cpu           = dest_cpu;
	),

	TP_printk("pid=%d tgid=%d comm=%s state=%04x, valid_mask=0x%lx, dest_cpu=%d",
		__entry->pid, __entry->tgid, __entry->comm, __entry->state,
		__entry->valid_mask, __entry->dest_cpu)
);


TRACE_EVENT(sched_find_energy_efficient_cpu,

	TP_PROTO(struct task_struct *tsk, bool prefer_idle, bool prefer_fit,
		 unsigned long task_importance, cpumask_t *candidate_mask, int best_energy_cpu),

	TP_ARGS(tsk, prefer_idle, prefer_fit, task_importance, candidate_mask, best_energy_cpu),

	TP_STRUCT__entry(
		__array(char,		comm, TASK_COMM_LEN)
		__field(pid_t,		pid)
		__field(bool,	        prefer_idle)
		__field(bool,	        prefer_fit)
		__field(unsigned long,	task_importance)
		__field(unsigned long,	idle_fit)
		__field(unsigned long,	idle_unfit)
		__field(unsigned long,	unimportant_fit)
		__field(unsigned long,	unimportant_unfit)
		__field(unsigned long,	packing)
		__field(unsigned long,	max_spare_cap)
		__field(unsigned long,	idle_unpreferred)
		__field(int,		best_energy_cpu)
		),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid               = tsk->pid;
		__entry->prefer_idle       = prefer_idle;
		__entry->prefer_fit        = prefer_fit;
		__entry->task_importance   = task_importance;
		__entry->idle_fit          = *candidate_mask[IDLE_FIT].bits;
		__entry->idle_unfit        = *candidate_mask[IDLE_UNFIT].bits;
		__entry->unimportant_fit   = *candidate_mask[UNIMPORTANT_FIT].bits;
		__entry->unimportant_unfit = *candidate_mask[UNIMPORTANT_UNFIT].bits;
		__entry->packing           = *candidate_mask[PACKING].bits;
		__entry->max_spare_cap     = *candidate_mask[MAX_SPARE_CAP].bits;
		__entry->idle_unpreferred  = *candidate_mask[IDLE_UNPREFERRED].bits;
		__entry->best_energy_cpu   = best_energy_cpu;
		),

	TP_printk("pid=%d comm=%s prefer_idle=%d prefer_fit=%d task_importance=%lu " \
		  "idle_fit=0x%lx idle_unfit=0x%lx unimportant_fit=0x%lx unimportant_unfit=0x%lx "\
		  "packing=0x%lx max_spare_cap=0x%lx idle_unpreferred=0x%lx best_energy_cpu=%d",
		  __entry->pid, __entry->comm, __entry->prefer_idle, __entry->prefer_fit,
		  __entry->task_importance, __entry->idle_fit, __entry->idle_unfit,
		  __entry->unimportant_fit, __entry->unimportant_unfit, __entry->packing,
		  __entry->max_spare_cap, __entry->idle_unpreferred, __entry->best_energy_cpu)
);

TRACE_EVENT(sched_wakeup_task_attr,

	TP_PROTO(struct task_struct *tsk, const cpumask_t *cpu_affinity,
		 unsigned long task_util, int uclamp_min, u64 vruntime),

	TP_ARGS(tsk, cpu_affinity, task_util, uclamp_min, vruntime),

	TP_STRUCT__entry(
		__field(pid_t,		pid)
		__field(unsigned long,  cpu_affinity)
		__field(unsigned long,	task_util)
		__field(unsigned long,	uclamp_min)
		__field(u64,	        vruntime)
		),

	TP_fast_assign(
		__entry->pid             = tsk->pid;
		__entry->cpu_affinity    = *cpu_affinity->bits;
		__entry->task_util       = task_util;
		__entry->uclamp_min      = uclamp_min;
		__entry->vruntime        = vruntime;
		),

	TP_printk("pid=%d cpu_affinity=0x%lx, task_util=%lu, uclamp.min=%lu vruntime=%Lu [ns]",
		  __entry->pid,  __entry->cpu_affinity, __entry->task_util, __entry->uclamp_min,
		  (unsigned long long)__entry->vruntime)
);

TRACE_EVENT(sched_select_task_rq_fair,

	TP_PROTO(struct task_struct *tsk, unsigned long task_util, bool sync_wakeup,
		 bool adpf, bool prefer_prev, bool sync_boost, int group,
		 int uclamp_min, int uclamp_max, int prev_cpu, int target_cpu),

	TP_ARGS(tsk, task_util, sync_wakeup, adpf, prefer_prev, sync_boost,
		group, uclamp_min, uclamp_max, prev_cpu, target_cpu),

	TP_STRUCT__entry(
		__array(char,		comm, TASK_COMM_LEN)
		__field(pid_t,		pid)
		__field(unsigned long,	task_util)
		__field(bool,		sync_wakeup)
		__field(bool,		adpf)
		__field(bool,		prefer_prev)
		__field(bool,		sync_boost)
		__field(int,		group)
		__field(unsigned long,	uclamp_min)
		__field(unsigned long,	uclamp_max)
		__field(int,		prev_cpu)
		__field(int,		target_cpu)
		),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid             = tsk->pid;
		__entry->task_util       = task_util;
		__entry->sync_wakeup     = sync_wakeup;
		__entry->adpf            = adpf;
		__entry->prefer_prev     = prefer_prev;
		__entry->sync_boost      = sync_boost;
		__entry->group           = group;
		__entry->uclamp_min      = uclamp_min;
		__entry->uclamp_max      = uclamp_max;
		__entry->prev_cpu        = prev_cpu;
		__entry->target_cpu      = target_cpu;
		),

	TP_printk("pid=%d comm=%s task_util=%lu sync_wakeup=%d adpf=%d prefer_prev=%d " \
		  "sync_boost=%d group=%d uclamp.min=%lu uclamp.max=%lu prev_cpu=%d target_cpu=%d",
		  __entry->pid, __entry->comm, __entry->task_util, __entry->sync_wakeup,
		  __entry->adpf, __entry->prefer_prev, __entry->sync_boost,
		  __entry->group, __entry->uclamp_min, __entry->uclamp_max,
		  __entry->prev_cpu, __entry->target_cpu)
);

TRACE_EVENT(sched_cpu_util_cfs,

	TP_PROTO(int cpu, bool idle_cpu, unsigned int exit_lat, unsigned long cpu_importance,
		 unsigned long cpu_util, unsigned long capacity, unsigned long wake_util,
		 unsigned long group_capacity, unsigned long wake_group_util,
		 long spare_cap, bool task_fits, bool grp_overutilized),

	TP_ARGS(cpu, idle_cpu, exit_lat, cpu_importance, cpu_util, capacity, wake_util,
		group_capacity, wake_group_util, spare_cap, task_fits, grp_overutilized),

	TP_STRUCT__entry(
		__field(unsigned int,	cpu)
		__field(bool,		idle_cpu)
		__field(unsigned int,	exit_lat)
		__field(unsigned int,	nr_running)
		__field(int,		active)
		__field(unsigned long,	cpu_importance)
		__field(unsigned long,	capacity_orig)
		__field(unsigned long,	cpu_util)
		__field(unsigned long,	capacity)
		__field(unsigned long,	wake_util)
		__field(unsigned long,	group_capacity)
		__field(unsigned long,	wake_group_util)
		__field(long,		spare_cap)
		__field(bool,		task_fits)
		__field(bool,           grp_overutilized)
	),

	TP_fast_assign(
		__entry->cpu                = cpu;
		__entry->idle_cpu           = idle_cpu;
		__entry->exit_lat           = exit_lat;
		__entry->nr_running         = cpu_rq(cpu)->nr_running;
		__entry->active             = cpu_active(cpu);
		__entry->cpu_importance     = cpu_importance;
		__entry->capacity_orig      = capacity_orig_of(cpu);
		__entry->cpu_util           = cpu_util;
		__entry->capacity           = capacity;
		__entry->wake_util          = wake_util;
		__entry->group_capacity     = group_capacity;
		__entry->wake_group_util    = wake_group_util;
		__entry->spare_cap          = spare_cap;
		__entry->task_fits          = task_fits;
		__entry->grp_overutilized   = grp_overutilized;
	),

	TP_printk("cpu=%d cpu_idle=%d exit_lat=%u nr_running=%d active=%d cpu_importance=%lu " \
		  "capacity_orig=%lu cpu_util=%lu capacity=%lu wake_util=%lu " \
		  "group_capacity=%lu wake_group_util=%lu spare_cap=%ld task_fits=%d " \
		  "grp_overutilized=%d",
		__entry->cpu, __entry->idle_cpu,  __entry->exit_lat, __entry->nr_running,
		__entry->active, __entry->cpu_importance, __entry->capacity_orig, __entry->cpu_util,
		__entry->capacity, __entry->wake_util, __entry->group_capacity,
		__entry->wake_group_util, __entry->spare_cap, __entry->task_fits,
		__entry->grp_overutilized)
);

TRACE_EVENT(sugov_util_update,
	    TP_PROTO(unsigned int cpu, unsigned long util, unsigned long max_cap,
		     unsigned int flags),
	    TP_ARGS(cpu, util, max_cap, flags),
	    TP_STRUCT__entry(
		    __field(unsigned int, cpu)
		    __field(unsigned long, util)
		    __field(unsigned long, max_cap)
		    __field(unsigned int, flags)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->util = util;
		    __entry->max_cap = max_cap;
		    __entry->flags = flags;
	    ),
	    TP_printk("cpu=%u util=%lu max_cap=%lu flags=0x%x",
		      __entry->cpu, __entry->util,
		      __entry->max_cap, __entry->flags)
);

TRACE_EVENT(sugov_next_freq,
	    TP_PROTO(unsigned int cpu, unsigned long util, unsigned long max,
		     unsigned int freq),
	    TP_ARGS(cpu, util, max, freq),
	    TP_STRUCT__entry(
		    __field(unsigned int, cpu)
		    __field(unsigned long, util)
		    __field(unsigned long, max)
		    __field(unsigned int, freq)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->util = util;
		    __entry->max = max;
		    __entry->freq = freq;
	    ),
	    TP_printk("cpu=%u util=%lu max=%lu freq=%u",
		      __entry->cpu,
		      __entry->util,
		      __entry->max,
		      __entry->freq)
);

TRACE_EVENT(schedutil_cpu_util_clamp,

	TP_PROTO(int cpu, unsigned long util_cfs, unsigned long util_rt, unsigned long util_clamp,
		 unsigned long util_max),

	TP_ARGS(cpu, util_cfs, util_rt, util_clamp, util_max),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned long,	util_cfs)
		__field(unsigned long,	util_rt)
		__field(unsigned long,	util_clamp)
		__field(unsigned long,	util_max)
	),

	TP_fast_assign(
		__entry->cpu                = cpu;
		__entry->util_cfs           = util_cfs;
		__entry->util_rt	    = util_rt;
		__entry->util_clamp         = util_clamp;
		__entry->util_max	    = util_max;
	),

	TP_printk("cpu=%d util_cfs=%ld util_rt=%ld util_clamp=%ld util_max=%ld",
		__entry->cpu, __entry->util_cfs, __entry->util_rt, __entry->util_clamp,
		__entry->util_max)
);

TRACE_EVENT(sched_cpu_util_rt,

	TP_PROTO(int cpu, unsigned long capacity_orig, unsigned long capacity, unsigned long util,
		 unsigned long exit_lat, unsigned long cpu_importance, bool task_fits,
		 bool task_fits_original, bool overutilized, bool is_idle),

	TP_ARGS(cpu, capacity_orig, capacity, util, exit_lat, cpu_importance, task_fits,
		task_fits_original, overutilized, is_idle),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned long,	capacity_orig)
		__field(unsigned long,	capacity)
		__field(unsigned long,	util)
		__field(unsigned long,	exit_lat)
		__field(unsigned long,	cpu_importance)
		__field(bool,		task_fits)
		__field(bool,		task_fits_original)
		__field(bool,		overutilized)
		__field(bool,		is_idle)
	),

	TP_fast_assign(
		__entry->cpu                = cpu;
		__entry->capacity_orig      = capacity_orig;
		__entry->capacity           = capacity;
		__entry->util	            = util;
		__entry->exit_lat           = exit_lat;
		__entry->cpu_importance	    = cpu_importance;
		__entry->task_fits	    = task_fits;
		__entry->task_fits_original = task_fits_original;
		__entry->overutilized	    = overutilized;
		__entry->is_idle	    = is_idle;
	),

	TP_printk("cpu=%d capacity_orig=%lu capacity=%lu util=%lu exit_lat=%lu cpu_importance=%lu "\
		  "task_fits=%d task_fits_original=%d overutilized=%d is_idle=%d",
		__entry->cpu, __entry->capacity_orig, __entry->capacity, __entry->util,
		__entry->exit_lat, __entry->cpu_importance, __entry->task_fits,
		__entry->task_fits_original, __entry->overutilized, __entry->is_idle)
);

TRACE_EVENT(sched_find_least_loaded_cpu,

	TP_PROTO(struct task_struct *tsk, int group, unsigned long uclamp_min,
		 unsigned long uclamp_max, bool prefer_fit, int prev_cpu, int best_cpu,
		 unsigned long lowest_mask, unsigned long backup_mask),

	TP_ARGS(tsk, group, uclamp_min, uclamp_max, prefer_fit, prev_cpu, best_cpu,
		lowest_mask, backup_mask),

	TP_STRUCT__entry(
		__array(char,		comm, TASK_COMM_LEN)
		__field(pid_t,		pid)
		__field(int,		group)
		__field(unsigned long,	uclamp_min)
		__field(unsigned long,	uclamp_max)
		__field(bool,		prefer_fit)
		__field(int,		prev_cpu)
		__field(int,		best_cpu)
		__field(unsigned long,	lowest_mask)
		__field(unsigned long,	backup_mask)
		),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid                     = tsk->pid;
		__entry->group                   = group;
		__entry->uclamp_min              = uclamp_min;
		__entry->uclamp_max              = uclamp_max;
		__entry->prefer_fit              = prefer_fit;
		__entry->prev_cpu                = prev_cpu;
		__entry->best_cpu                = best_cpu;
		__entry->lowest_mask             = lowest_mask;
		__entry->backup_mask             = backup_mask;
		),

	TP_printk("pid=%d comm=%s group=%d uclamp_min=%lu uclamp_max=%lu prefer_fit=%d " \
		"prev_cpu=%d best_cpu=%d lowest_mask=0x%lx backup_mask=0x%lx",
		__entry->pid, __entry->comm, __entry->group, __entry->uclamp_min,
		__entry->uclamp_max, __entry->prefer_fit, __entry->prev_cpu, __entry->best_cpu,
		__entry->lowest_mask, __entry->backup_mask)
);

TRACE_EVENT(sched_select_task_rq_rt,

	TP_PROTO(struct task_struct *tsk, unsigned long task_util, int prev_cpu, int target,
		 int new_cpu, bool sync_wakeup),

	TP_ARGS(tsk, task_util, prev_cpu, target, new_cpu, sync_wakeup),

	TP_STRUCT__entry(
		__array(char,		comm, TASK_COMM_LEN)
		__field(pid_t,		pid)
		__field(unsigned long,	task_util)
		__field(int,		prev_cpu)
		__field(int,		target)
		__field(int,		new_cpu)
		__field(bool,		sync_wakeup)
		),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid             = tsk->pid;
		__entry->task_util       = task_util;
		__entry->prev_cpu        = prev_cpu;
		__entry->target          = target;
		__entry->new_cpu         = new_cpu;
		__entry->sync_wakeup     = sync_wakeup;
		),

	TP_printk("pid=%d comm=%s task_util=%lu prev_cpu=%d target=%d new_cpu=%d sync_wakeup=%d",
		__entry->pid, __entry->comm, __entry->task_util, __entry->prev_cpu, __entry->target,
		__entry->new_cpu, __entry->sync_wakeup)
);

#endif /* _SCHED_EVENTS_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE sched_events
#include <trace/define_trace.h>
