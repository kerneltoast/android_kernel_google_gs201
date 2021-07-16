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
		__entry->enqueued	= avg->util_est.enqueued;
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
		__entry->enqueued	= avg->util_est.enqueued;
		__entry->ewma		= avg->util_est.ewma;
		__entry->util		= avg->util_avg;
	),

	TP_printk("cpu=%d path=%s enqueued=%u ewma=%u util=%lu",
		  __entry->cpu, __entry->path, __entry->enqueued,
		 __entry->ewma, __entry->util)
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

TRACE_EVENT(sched_find_best_target,

	TP_PROTO(struct task_struct *tsk, bool prefer_idle,
		 bool prefer_high_cap, bool prefer_prev, bool sync_boost,
		 unsigned long task_util, int start_cpu,
		 int best_idle, int best_active, int best_importance,
		 int backup, int target),

	TP_ARGS(tsk, prefer_idle, prefer_high_cap, prefer_prev, sync_boost, task_util,
		start_cpu, best_idle, best_active, best_importance, backup, target),

	TP_STRUCT__entry(
		__array(char,		comm, TASK_COMM_LEN)
		__field(pid_t,		pid)
		__field(unsigned long,	task_util)
		__field(bool,		prefer_idle)
		__field(bool,		prefer_high_cap)
		__field(bool,		prefer_prev)
		__field(bool,		sync_boost)
		__field(int,		start_cpu)
		__field(int,		best_idle)
		__field(int,		best_active)
		__field(int,		best_importance)
		__field(int,		backup)
		__field(int,		target)
		),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid             = tsk->pid;
		__entry->task_util       = task_util;
		__entry->prefer_idle     = prefer_idle;
		__entry->prefer_high_cap = prefer_high_cap;
		__entry->prefer_prev     = prefer_prev;
		__entry->sync_boost      = sync_boost;
		__entry->start_cpu       = start_cpu;
		__entry->best_idle       = best_idle;
		__entry->best_active     = best_active;
		__entry->best_importance = best_importance;
		__entry->backup          = backup;
		__entry->target          = target;
		),

	TP_printk("pid=%d comm=%s task_util=%lu prefer_idle=%d prefer_high_cap=%d prefer_prev=%d " \
		  "sync_boost=%d start_cpu=%d best_idle=%d best_active=%d best_importance=%d " \
		  "backup=%d target=%d",
		  __entry->pid, __entry->comm, __entry->task_util, __entry->prefer_idle,
		  __entry->prefer_high_cap, __entry->prefer_prev, __entry->sync_boost,
		  __entry->start_cpu, __entry->best_idle, __entry->best_active,
		  __entry->best_importance, __entry->backup, __entry->target)
);

TRACE_EVENT(sched_find_energy_efficient_cpu,

	TP_PROTO(struct task_struct *tsk, bool sync_wakeup,
		 int new_cpu, int best_energy_cpu, int prev_cpu,
		 int group, int uclamp_min, int uclamp_max),

	TP_ARGS(tsk, sync_wakeup, new_cpu, best_energy_cpu, prev_cpu,
		group, uclamp_min, uclamp_max),

	TP_STRUCT__entry(
		__array(char,		comm, TASK_COMM_LEN)
		__field(pid_t,		pid)
		__field(bool,		sync_wakeup)
		__field(int,		new_cpu)
		__field(int,		best_energy_cpu)
		__field(int,		prev_cpu)
		__field(int,		group)
		__field(int,		uclamp_min)
		__field(int,		uclamp_max)
		),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid             = tsk->pid;
		__entry->sync_wakeup     = sync_wakeup;
		__entry->new_cpu         = new_cpu;
		__entry->best_energy_cpu = best_energy_cpu;
		__entry->prev_cpu        = prev_cpu;
		__entry->group           = group;
		__entry->uclamp_min      = uclamp_min;
		__entry->uclamp_max      = uclamp_max;
		),

	TP_printk("pid=%d comm=%s sync_wakeup=%d new_cpu=%d best_energy_cpu=%d prev_cpu=%d " \
		  "group=%d uclamp.min=%d uclamp.max=%d",
		  __entry->pid, __entry->comm, __entry->sync_wakeup, __entry->new_cpu,
		  __entry->best_energy_cpu, __entry->prev_cpu,
		  __entry->group, __entry->uclamp_min, __entry->uclamp_max)
);

TRACE_EVENT(sched_cpu_util,

	TP_PROTO(int cpu, unsigned long cpu_util, unsigned long capacity_curr,
		 unsigned long capacity, unsigned long wake_util, bool idle_cpu,
		 unsigned long cpu_importance,
		 unsigned long group_capacity, unsigned long wake_group_util,
		 long spare_cap, unsigned long group_util, bool grp_overutilized),

	TP_ARGS(cpu, cpu_util, capacity_curr, capacity, wake_util, idle_cpu,
		cpu_importance, group_capacity, wake_group_util, spare_cap, group_util,
		grp_overutilized),

	TP_STRUCT__entry(
		__field(unsigned int,	cpu)
		__field(unsigned int,	nr_running)
		__field(unsigned long,	cpu_util)
		__field(unsigned long,	capacity_curr)
		__field(unsigned long,	capacity)
		__field(unsigned long,	wake_util)
		__field(unsigned long,	capacity_orig)
		__field(int,		    active)
		__field(bool,		    idle_cpu)
		__field(unsigned long,	cpu_importance)
		__field(unsigned long,	group_capacity)
		__field(unsigned long,	wake_group_util)
		__field(long,			spare_cap)
		__field(unsigned long,  group_util)
		__field(bool,           grp_overutilized)
	),

	TP_fast_assign(
		__entry->cpu                = cpu;
		__entry->nr_running         = cpu_rq(cpu)->nr_running;
		__entry->cpu_util           = cpu_util;
		__entry->capacity_curr      = capacity_curr;
		__entry->capacity           = capacity;
		__entry->wake_util          = wake_util;
		__entry->capacity_orig      = capacity_orig_of(cpu);
		__entry->active             = cpu_active(cpu);
		__entry->idle_cpu           = idle_cpu;
		__entry->cpu_importance     = cpu_importance;
		__entry->group_capacity     = group_capacity;
		__entry->wake_group_util    = wake_group_util;
		__entry->spare_cap          = spare_cap;
		__entry->group_util         = group_util;
		__entry->grp_overutilized   = grp_overutilized;
	),

	TP_printk("cpu=%d nr_running=%d cpu_util=%llu capacity_curr=%llu capacity=%llu " \
		  "wake_util=%llu capacity_orig=%u active=%u idle_cpu=%d " \
		  "cpu_importance=%llu group_capacity=%llu wake_group_util=%llu spare_cap=%ld " \
		  "group_util=%lu grp_overutilized=%d",
		__entry->cpu, __entry->nr_running, __entry->cpu_util, __entry->capacity_curr,
		__entry->capacity, __entry->wake_util, __entry->capacity_orig, __entry->active,
		__entry->idle_cpu, __entry->cpu_importance,
		__entry->group_capacity, __entry->wake_group_util, __entry->spare_cap,
		__entry->group_util, __entry->grp_overutilized)
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

TRACE_EVENT(sched_rt_cpu_util,

	TP_PROTO(int cpu, unsigned long capacity, unsigned long util, unsigned long exit_lat,
		 unsigned long cpu_importance),

	TP_ARGS(cpu, capacity, util, exit_lat, cpu_importance),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned long,	capacity)
		__field(unsigned long,	util)
		__field(unsigned long,	exit_lat)
		__field(unsigned long,	cpu_importance)
	),

	TP_fast_assign(
		__entry->cpu                = cpu;
		__entry->capacity           = capacity;
		__entry->util	            = util;
		__entry->exit_lat           = exit_lat;
		__entry->cpu_importance	    = cpu_importance;
	),

	TP_printk("cpu=%d capacity=%llu util=%llu exit_lat=%llu cpu_importance=%llu",
		__entry->cpu, __entry->capacity, __entry->util, __entry->exit_lat,
		__entry->cpu_importance)
);

TRACE_EVENT(sched_find_least_loaded_cpu,

	TP_PROTO(struct task_struct *tsk, int group, unsigned long uclamp_min,
		 unsigned long uclamp_max, bool check_cpu_overutilized, unsigned long min_cpu_util,
		 unsigned long min_cpu_capacity, unsigned int min_exit_lat, int prev_cpu,
		 int best_cpu, unsigned long lowest_mask, unsigned long backup_mask),

	TP_ARGS(tsk, group, uclamp_min, uclamp_max, check_cpu_overutilized, min_cpu_util,
		min_cpu_capacity, min_exit_lat, prev_cpu, best_cpu, lowest_mask, backup_mask),

	TP_STRUCT__entry(
		__array(char,		comm, TASK_COMM_LEN)
		__field(pid_t,		pid)
		__field(int,		group)
		__field(unsigned long,		uclamp_min)
		__field(unsigned long,		uclamp_max)
		__field(bool,		check_cpu_overutilized)
		__field(unsigned long,		min_cpu_util)
		__field(unsigned long,		min_cpu_capacity)
		__field(unsigned int,		min_exit_lat)
		__field(int,		prev_cpu)
		__field(int,		best_cpu)
		__field(unsigned long,		lowest_mask)
		__field(unsigned long,		backup_mask)
		),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid                     = tsk->pid;
		__entry->group                   = group;
		__entry->uclamp_min              = uclamp_min;
		__entry->uclamp_max              = uclamp_max;
		__entry->check_cpu_overutilized  = check_cpu_overutilized;
		__entry->min_cpu_util            = min_cpu_util;
		__entry->min_cpu_capacity        = min_cpu_capacity;
		__entry->min_exit_lat            = min_exit_lat;
		__entry->prev_cpu                = prev_cpu;
		__entry->best_cpu                = best_cpu;
		__entry->lowest_mask             = lowest_mask;
		__entry->backup_mask             = backup_mask;
		),

	TP_printk("pid=%d comm=%s group=%d uclamp_min=%llu uclamp_max=%llu " \
		"check_cpu_overutilized=%d min_cpu_util=%llu min_cpu_capacity=%llu " \
		"min_exit_lat=%u prev_cpu=%d best_cpu=%d lowest_mask=0x%x backup_mask=0x%x",
		__entry->pid, __entry->comm, __entry->group, __entry->uclamp_min,
		__entry->uclamp_max, __entry->check_cpu_overutilized, __entry->min_cpu_util,
		__entry->min_cpu_capacity, __entry->min_exit_lat, __entry->prev_cpu,
		__entry->best_cpu, __entry->lowest_mask, __entry->backup_mask)
);

TRACE_EVENT(sched_select_task_rq_rt,

	TP_PROTO(struct task_struct *tsk, int prev_cpu, int target, int new_cpu, bool sync_wakeup),

	TP_ARGS(tsk, prev_cpu, target, new_cpu, sync_wakeup),

	TP_STRUCT__entry(
		__array(char,		comm, TASK_COMM_LEN)
		__field(pid_t,		pid)
		__field(int,		prev_cpu)
		__field(int,		target)
		__field(int,		new_cpu)
		__field(bool,		sync_wakeup)
		),

	TP_fast_assign(
		memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		__entry->pid             = tsk->pid;
		__entry->prev_cpu        = prev_cpu;
		__entry->target          = target;
		__entry->new_cpu         = new_cpu;
		__entry->sync_wakeup     = sync_wakeup;
		),

	TP_printk("pid=%d comm=%s prev_cpu=%d target=%d new_cpu=%d sync_wakeup=%d",
		__entry->pid, __entry->comm, __entry->prev_cpu, __entry->target, __entry->new_cpu,
		__entry->sync_wakeup)
);

#endif /* _SCHED_EVENTS_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE sched_events
#include <trace/define_trace.h>
