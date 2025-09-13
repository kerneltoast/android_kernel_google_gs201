// SPDX-License-Identifier: GPL-2.0-only
/* metrics.c
 *
 * Support for Perf metrics
 *
 * Copyright 2022 Google LLC
 */
#define pr_fmt(fmt) KBUILD_MODNAME": " fmt
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/sort.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/sched/clock.h>
#include <linux/sched/cputime.h>
#include <trace/events/power.h>
#include <trace/events/irq.h>
#include <trace/hooks/suspend.h>
#include <trace/hooks/sched.h>
#include "perf_metrics.h"
#include <trace/hooks/systrace.h>
#include "../../include/sched.h"
#include <kernel/sched/sched.h>

struct irq_storm_data {
	atomic64_t storm_count;
	s64 max_storm_count;
	atomic64_t irq_storm_start;
};

struct resume_latency {
	u64 resume_start;
	u64 resume_end;
	spinlock_t resume_latency_stat_lock;
	s64 resume_count[RESUME_LATENCY_ARR_SIZE];
	u64 resume_latency_max_ms;
	u64 resume_latency_sum_ms;
	u64 resume_latency_threshold;
	bool display_warning;
};

struct long_irq {
	ktime_t softirq_start[CONFIG_VH_SCHED_MAX_CPU_NR][NR_SOFTIRQS];
	ktime_t irq_start[CONFIG_VH_SCHED_MAX_CPU_NR][MAX_IRQ_NUM];
	atomic64_t long_softirq_count;
	atomic64_t long_irq_count;
	atomic64_t long_softirq_count_arr[CONFIG_VH_SCHED_MAX_CPU_NR];
	atomic64_t long_irq_count_arr[CONFIG_VH_SCHED_MAX_CPU_NR];
	s64 long_softirq_arr[NR_SOFTIRQS];
	s64 long_irq_arr[MAX_IRQ_NUM];
	struct irq_storm_data irq_storms[MAX_IRQ_NUM];
	s64 long_softirq_threshold;
	s64 long_irq_threshold;
	s64 irq_storm_threshold_us;
	bool display_warning;
};

struct rt_runnable {
	u64 latency;
	char comm[TASK_COMM_LEN];
	char prev_comm[TASK_COMM_LEN];
	pid_t pid;
};

struct top_rt_runnable {
	struct rt_runnable rt_runnable[RT_RUNNABLE_ARR_SIZE];
	int min_idx;
	atomic64_t count;
};

struct irq_entry {
	int irq_num;
	s64 latency;
	s64 max_storm_count;
};

static struct resume_latency resume_latency_stats;
static struct long_irq long_irq_stat;

static DEFINE_PER_CPU(struct top_rt_runnable, top_rt_runnable);
static DEFINE_PER_CPU(spinlock_t, rt_runnable_lock);
unsigned long long_rt_runnable_threshold_ns = 1500000UL;
const unsigned long task_real_cap_window_ns = 16000000UL;

/*********************************************************************
 *                          HELPER FUNCTIONS                         *
 *********************************************************************/

static bool is_top_latency(u64 latency)
{
	struct top_rt_runnable trr;
	int cpu = raw_smp_processor_id();

	spin_lock(&per_cpu(rt_runnable_lock, cpu));
	trr = per_cpu(top_rt_runnable, cpu);
	spin_unlock(&per_cpu(rt_runnable_lock, cpu));

	return latency > trr.rt_runnable[trr.min_idx].latency;
}

static void update_min_latency(struct task_struct *prev, struct task_struct *next, u64 latency)
{
	u64 min_latency = -1LL;
	struct rt_runnable *rt_runnable;
	struct top_rt_runnable *trr;
	int min_idx, i, cpu;
	bool in_arr = false;

	cpu = raw_smp_processor_id();
	spin_lock(&per_cpu(rt_runnable_lock, cpu));
	trr = this_cpu_ptr(&top_rt_runnable);

	atomic64_inc(&(trr->count));

	/* Search if the pid is already in the top_runnable, if so, update it */
	for (i = 0; i < RT_RUNNABLE_ARR_SIZE; i++) {
		rt_runnable =  &trr->rt_runnable[i];
		if (rt_runnable->pid == next->pid) {
			in_arr = true;
			rt_runnable->latency = latency;
			break;
		}
	}
	if (!in_arr){
		rt_runnable = &trr->rt_runnable[trr->min_idx];
		rt_runnable->latency = latency;
		strlcpy(rt_runnable->comm, next->comm, TASK_COMM_LEN);
		strlcpy(rt_runnable->prev_comm, prev->comm, TASK_COMM_LEN);
		rt_runnable->pid = next->pid;
	}

	/* Update min entry among top_rt_runnable */
	for (i = 0; i < RT_RUNNABLE_ARR_SIZE; i++) {
		rt_runnable =  &trr->rt_runnable[i];
		if (rt_runnable->latency < min_latency) {
			min_latency = rt_runnable->latency;
			min_idx = i;
		}
	}

	trr->min_idx = min_idx;
	spin_unlock(&per_cpu(rt_runnable_lock, cpu));
}

static int irq_latency_cmp(const void *a, const void *b)
{
	return ((struct irq_entry *)b)->latency - ((struct irq_entry *)a)->latency;
}

static int irq_storm_count_cmp(const void *a, const void *b)
{
	return ((struct irq_entry *)b)->max_storm_count - ((struct irq_entry *)a)->max_storm_count;
}

static int runnable_latency_cmp(const void *a, const void *b)
{
	if (((struct rt_runnable *)a)->latency < ((struct rt_runnable *)b)->latency)
		return 1;
	else if (((struct rt_runnable *)a)->latency > ((struct rt_runnable *)b)->latency)
		return -1;
	else
		return 0;
}

/*********************************************************************
 *                          SYSTEM TRACE
 *********************************************************************/

static void vendor_hook_resume_begin(void *data, void *unused)
{
	resume_latency_stats.resume_start = ktime_get_mono_fast_ns();
}

static void vendor_hook_resume_end(void *data, void *unused)
{
	int resume_latency_index;
	u64 resume_latency_msec;
	/* Exit function when partial resumes */
	if (resume_latency_stats.resume_start == resume_latency_stats.resume_end)
		return;
	resume_latency_stats.resume_end = ktime_get_mono_fast_ns();
	resume_latency_msec = (resume_latency_stats.resume_end -
						resume_latency_stats.resume_start) / NSEC_PER_MSEC;
	pr_info("resume latency: %llu\n", resume_latency_msec);
	/* Exit function when partial resumes */
	if (resume_latency_stats.resume_end < resume_latency_stats.resume_start)
		return;
	if (resume_latency_stats.display_warning)
		WARN(resume_latency_msec >= resume_latency_stats.resume_latency_threshold,
				"Got a outlier resume latency: %llums\n", resume_latency_msec);
	spin_lock(&resume_latency_stats.resume_latency_stat_lock);
	if (resume_latency_msec < RESUME_LATENCY_BOUND_SMALL) {
		resume_latency_index = resume_latency_msec / RESUME_LATENCY_STEP_SMALL;
	} else if (resume_latency_msec < RESUME_LATENCY_BOUND_MID) {
		resume_latency_index = (resume_latency_msec - RESUME_LATENCY_BOUND_SMALL) /
						RESUME_LATENCY_STEP_MID + LATENCY_CNT_SMALL;
	} else if (resume_latency_msec < RESUME_LATENCY_BOUND_MAX) {
		resume_latency_index = (resume_latency_msec - RESUME_LATENCY_BOUND_MID) /
						RESUME_LATENCY_STEP_LARGE + LATENCY_CNT_SMALL +
						LATENCY_CNT_MID;
	} else {
		resume_latency_index = LATENCY_CNT_SMALL + LATENCY_CNT_MID + LATENCY_CNT_LARGE;
	}
	resume_latency_stats.resume_count[resume_latency_index]++;
	resume_latency_stats.resume_latency_sum_ms += resume_latency_msec;
	resume_latency_stats.resume_latency_max_ms = max(resume_latency_stats.resume_latency_max_ms,
						resume_latency_msec);
	spin_unlock(&resume_latency_stats.resume_latency_stat_lock);
	resume_latency_stats.resume_start = resume_latency_stats.resume_end;
}

static void hook_softirq_begin(void *data, unsigned int vec_nr)
{
	int cpu_num;
	cpu_num = raw_smp_processor_id();
	long_irq_stat.softirq_start[cpu_num][vec_nr] = ktime_get();
}

static void hook_softirq_end(void *data, unsigned int vec_nr)
{
	s64 irq_usec;
	int cpu_num;
	ktime_t softirq_end;
	s64 curr_max_irq;
	if (vec_nr >= NR_SOFTIRQS)
		return;
	cpu_num = raw_smp_processor_id();
	softirq_end = ktime_get();
	irq_usec = ktime_to_us(ktime_sub(softirq_end,
						long_irq_stat.softirq_start[cpu_num][vec_nr]));
	if (irq_usec >= long_irq_stat.long_softirq_threshold) {
		if (long_irq_stat.display_warning)
			WARN(1, "Got a long running softirq: SOFTIRQ %u in cpu: %d\n",
						vec_nr, cpu_num);
		atomic64_inc(&(long_irq_stat.long_softirq_count));
		atomic64_inc(&(long_irq_stat.long_softirq_count_arr[cpu_num]));
		if (trace_clock_set_rate_enabled()) {
			char trace_name[32] = {0};
			scnprintf(trace_name, sizeof(trace_name), "long_softirq_count_cpu%d",
							cpu_num);
			trace_clock_set_rate(trace_name,
				(unsigned int)
				atomic64_read(&long_irq_stat.long_softirq_count_arr[cpu_num]),
				cpu_num);
		}
	}
	do {
		curr_max_irq = long_irq_stat.long_softirq_arr[vec_nr];
		if (irq_usec < curr_max_irq)
			return;
	} while (cmpxchg64(&long_irq_stat.long_softirq_arr[vec_nr],
						curr_max_irq, irq_usec) != curr_max_irq);
}

static void hook_irq_begin(void *data, int irq, struct irqaction *action)
{
	int cpu_num;
	ktime_t irq_start;
	ktime_t prev_irq_start;
	s64 irq_start_diff_usec;
	s64 curr_storm_count;
	s64 curr_max_storm_count;
	if (irq >= MAX_IRQ_NUM)
		return;
	cpu_num = raw_smp_processor_id();


	prev_irq_start = atomic64_read(&(long_irq_stat.irq_storms[irq].irq_storm_start));

	irq_start = ktime_get();
	long_irq_stat.irq_start[cpu_num][irq] = irq_start;
	atomic64_set(&(long_irq_stat.irq_storms[irq].irq_storm_start), irq_start);

	irq_start_diff_usec = ktime_to_us(ktime_sub(irq_start, prev_irq_start));
	if (irq_start_diff_usec <= long_irq_stat.irq_storm_threshold_us) {
		atomic64_inc(&(long_irq_stat.irq_storms[irq].storm_count));
	} else {
		curr_storm_count = atomic64_read(&(long_irq_stat.irq_storms[irq].storm_count));
		do {
			curr_max_storm_count = long_irq_stat.irq_storms[irq].max_storm_count;
			if (curr_storm_count <= curr_max_storm_count)
				break;
		} while (cmpxchg64(&long_irq_stat.irq_storms[irq].max_storm_count,
			curr_max_storm_count, curr_storm_count) != curr_max_storm_count);
		atomic64_set(&(long_irq_stat.irq_storms[irq].storm_count), 0);
	}

	if (long_irq_stat.display_warning &&
		long_irq_stat.long_irq_arr[irq] >= long_irq_stat.long_irq_threshold) {
		char trace_name[32] = {0};
		scnprintf(trace_name, sizeof(trace_name), "long_irq_%d",
							irq);
		ATRACE_BEGIN(trace_name);
    }
}

static void hook_irq_end(void *data, int irq, struct irqaction *action, int ret)
{
	s64 irq_usec;
	int cpu_num;
	ktime_t irq_end;
	s64 curr_max_irq;
	if (irq >= MAX_IRQ_NUM)
		return;
	cpu_num = raw_smp_processor_id();
	irq_end = ktime_get();
	irq_usec = ktime_to_us(ktime_sub(irq_end,
				long_irq_stat.irq_start[cpu_num][irq]));
	if (long_irq_stat.display_warning &&
		long_irq_stat.long_irq_arr[irq] >= long_irq_stat.long_irq_threshold)
		ATRACE_END();
	if (irq_usec >= long_irq_stat.long_irq_threshold) {
		if (long_irq_stat.display_warning)
			WARN(1, "Got a long running hardirq: IRQ %d in cpu: %d\n", irq, cpu_num);
		atomic64_inc(&(long_irq_stat.long_irq_count));
		atomic64_inc(&(long_irq_stat.long_irq_count_arr[cpu_num]));
		if (trace_clock_set_rate_enabled()) {
			char trace_name[32] = {0};
			scnprintf(trace_name, sizeof(trace_name), "long_irq_count_cpu%d",
							cpu_num);
			trace_clock_set_rate(trace_name,
				(unsigned int)
				atomic64_read(&long_irq_stat.long_irq_count_arr[cpu_num]),
				cpu_num);
			scnprintf(trace_name, sizeof(trace_name), "irq_%d_last_dur", irq);
			trace_clock_set_rate(trace_name, (unsigned int)irq_usec, cpu_num);
		}
	}
	do {
		curr_max_irq = long_irq_stat.long_irq_arr[irq];
		if (irq_usec < curr_max_irq)
			break;
	} while (cmpxchg64(&long_irq_stat.long_irq_arr[irq],
						curr_max_irq, irq_usec) != curr_max_irq);
}

void vh_sched_wakeup_pixel_mod(void *data, struct task_struct *p)
{
	struct vendor_task_struct *vp;

	if (!rt_task(p))
		return;
	vp = get_vendor_task_struct(p);
	vp->runnable_start_ns = sched_clock();
}

void vh_sched_switch_pixel_mod(void *data, bool preempt,
		struct task_struct *prev,
		struct task_struct *next,
		unsigned int prev_state)
{
	struct vendor_task_struct *vnext, *vprev;
	u64 now, runnable_delta;

	now = sched_clock();
	vprev = get_vendor_task_struct(prev);

	if (vprev->adpf_adj) {
		update_task_real_cap(prev);
	}

	/*
	 * Update previous task's runnable_start_ns if it is in TASK_RUNNING state,
	 * which means it remains in rq. Otherwise, invalidate runnable_start_ns,
	 * given task is dequeued.
	 */
	if (task_is_running(prev) && rt_task(prev))
		vprev->runnable_start_ns = now;
	else
		vprev->runnable_start_ns = -1;

	vnext = get_vendor_task_struct(next);

	if (vnext->adpf_adj) {
		raw_spin_lock(&vnext->lock);
		if (vnext->real_cap_total_ns > task_real_cap_window_ns) {
			vnext->real_cap_avg = 0;
			vnext->real_cap_total_ns = 0;
		}
		vnext->real_cap_update_ns = now;
		raw_spin_unlock(&vnext->lock);
	}

	if (!rt_task(next) || vnext->runnable_start_ns > now)
		return;

	runnable_delta = now - vnext->runnable_start_ns;
	if (runnable_delta < long_rt_runnable_threshold_ns ||
		!is_top_latency(runnable_delta))
		return;

	update_min_latency(prev, next, runnable_delta);

}

/*******************************************************************
 *                       		SYSFS			   				   *
 *******************************************************************/

static ssize_t resume_latency_metrics_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	s64 lower_bound;
	s64 upper_bound;
	int index;
	ssize_t count = 0;
	count += sysfs_emit_at(buf, count, "Resume Latency Bucket Count: %d\n",
				RESUME_LATENCY_ARR_SIZE);
	count += sysfs_emit_at(buf, count, "Max Resume Latency: %llu\n",
				resume_latency_stats.resume_latency_max_ms);
	count += sysfs_emit_at(buf, count, "Sum Resume Latency: %llu\n",
				resume_latency_stats.resume_latency_sum_ms);
	for (index = 0; index < RESUME_LATENCY_ARR_SIZE; index++) {
		if (index < LATENCY_CNT_SMALL) {
			lower_bound = index * RESUME_LATENCY_STEP_SMALL;
			upper_bound = lower_bound + RESUME_LATENCY_STEP_SMALL;
			count += sysfs_emit_at(buf, count, "%lld - %lldms ====> %lld\n",
				lower_bound, upper_bound,
				resume_latency_stats.resume_count[index]);
		} else if (index < LATENCY_CNT_SMALL + LATENCY_CNT_MID) {
			lower_bound = RESUME_LATENCY_BOUND_SMALL + RESUME_LATENCY_STEP_MID *
				(index - LATENCY_CNT_SMALL);
			upper_bound = lower_bound + RESUME_LATENCY_STEP_MID;
			count += sysfs_emit_at(buf, count, "%lld - %lldms ====> %lld\n",
				lower_bound, upper_bound,
				resume_latency_stats.resume_count[index]);
		} else if (index < LATENCY_CNT_SMALL + LATENCY_CNT_MID + LATENCY_CNT_LARGE) {
			lower_bound = RESUME_LATENCY_BOUND_MID + RESUME_LATENCY_STEP_LARGE *
				(index - (LATENCY_CNT_SMALL + LATENCY_CNT_MID));
			upper_bound = lower_bound + RESUME_LATENCY_STEP_LARGE;
			count += sysfs_emit_at(buf, count, "%lld - %lldms ====> %lld\n",
				lower_bound, upper_bound,
				resume_latency_stats.resume_count[index]);
		} else {
			lower_bound = RESUME_LATENCY_BOUND_MAX;
			count += sysfs_emit_at(buf, count, "%lld - infms ====> %lld\n",
				lower_bound,
				resume_latency_stats.resume_count[index]);
		}
	}
	return count;
}

static ssize_t resume_latency_metrics_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	spin_lock(&resume_latency_stats.resume_latency_stat_lock);
	resume_latency_stats.resume_latency_max_ms = 0;
	resume_latency_stats.resume_latency_sum_ms = 0;
	memset(resume_latency_stats.resume_count, 0, RESUME_LATENCY_ARR_SIZE *
				sizeof(resume_latency_stats.resume_count[0]));
	spin_unlock(&resume_latency_stats.resume_latency_stat_lock);
	return count;
}

static ssize_t resume_latency_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "%llu\n", resume_latency_stats.resume_latency_threshold);
}

static ssize_t resume_latency_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	u64 new_threshold_ms;
	int err = kstrtou64(buf, 10, &new_threshold_ms);
	if (err)
		return err;
	resume_latency_stats.resume_latency_threshold = new_threshold_ms;
	return count;
}

static ssize_t resume_latency_display_warning_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "%d\n", resume_latency_stats.display_warning);
}

static ssize_t resume_latency_display_warning_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int err = kstrtobool(buf, &resume_latency_stats.display_warning);
	if (err)
		return err;
	return count;
}

static ssize_t long_irq_metrics_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t count = 0;
	int index;
	s64 latency;
	int irq_num;
	struct irq_entry *sorted_softirq_arr;
	struct irq_entry *sorted_irq_arr;
	sorted_softirq_arr = kmalloc(NR_SOFTIRQS * sizeof(struct irq_entry), GFP_KERNEL);
	if (!sorted_softirq_arr)
		return -ENOMEM;
	sorted_irq_arr = kmalloc(MAX_IRQ_NUM * sizeof(struct irq_entry), GFP_KERNEL);
	if (!sorted_irq_arr) {
		kfree(sorted_softirq_arr);
		return -ENOMEM;
	}
	count += sysfs_emit_at(buf, count, "long SOFTIRQ count: %lld\n",
				atomic64_read(&(long_irq_stat.long_softirq_count)));
	count += sysfs_emit_at(buf, count, "long SOFTIRQ detail (num, latency):\n");

	for (index = 0; index < NR_SOFTIRQS; index++) {
		sorted_softirq_arr[index].irq_num = index;
		sorted_softirq_arr[index].latency = long_irq_stat.long_softirq_arr[index];
	}
	sort(sorted_softirq_arr, NR_SOFTIRQS, sizeof(struct irq_entry), irq_latency_cmp, NULL);
	for (index = 0; index < NR_SOFTIRQS; index++) {
		latency = sorted_softirq_arr[index].latency;
		irq_num = sorted_softirq_arr[index].irq_num;
		if (latency > 0)
			count += sysfs_emit_at(buf, count,
				"%d %lld\n", irq_num, latency);
	}
	count += sysfs_emit_at(buf, count, "long IRQ count: %lld\n",
				atomic64_read(&(long_irq_stat.long_irq_count)));
	count += sysfs_emit_at(buf, count, "long IRQ detail (num, latency):\n");

	for (index = 0; index < MAX_IRQ_NUM; index++) {
		sorted_irq_arr[index].irq_num = index;
		sorted_irq_arr[index].latency = long_irq_stat.long_irq_arr[index];
	}
	sort(sorted_irq_arr, MAX_IRQ_NUM, sizeof(struct irq_entry), irq_latency_cmp, NULL);

	for (index = 0; index < IRQ_ARR_LIMIT; index++) {
		latency = sorted_irq_arr[index].latency;
		irq_num = sorted_irq_arr[index].irq_num;
		if (latency > 0)
			count += sysfs_emit_at(buf, count, "%d %lld\n", irq_num, latency);
	}

	kfree(sorted_softirq_arr);
	kfree(sorted_irq_arr);
	return count;
}

static ssize_t storm_irq_metrics_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t count = 0;
	int index;
	s64 storm_count;
	int irq_num;
	struct irq_entry *sorted_irq_arr;
	sorted_irq_arr = kmalloc(MAX_IRQ_NUM * sizeof(struct irq_entry), GFP_KERNEL);
	if (!sorted_irq_arr) {
		return -ENOMEM;
	}
	count += sysfs_emit_at(buf, count, "storm IRQ detail (num, storm_count):\n");
	for (index = 0; index < MAX_IRQ_NUM; index++) {
		sorted_irq_arr[index].irq_num = index;
		sorted_irq_arr[index].max_storm_count =
						long_irq_stat.irq_storms[index].max_storm_count;
	}
	sort(sorted_irq_arr, MAX_IRQ_NUM, sizeof(struct irq_entry), irq_storm_count_cmp, NULL);
	for (index = 0; index < IRQ_ARR_LIMIT; index++) {
		storm_count = sorted_irq_arr[index].max_storm_count;
		irq_num = sorted_irq_arr[index].irq_num;
		if (storm_count > 0)
			count += sysfs_emit_at(buf, count, "%d %lld\n", irq_num, storm_count);
	}

	kfree(sorted_irq_arr);
	return count;
}

static ssize_t softirq_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "%lld\n", long_irq_stat.long_softirq_threshold);
}

static ssize_t softirq_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	s64 new_threshold_us;
	int err = kstrtoll(buf, 10, &new_threshold_us);
	if (!err || new_threshold_us < 0) {
		return -EINVAL;
	}
	long_irq_stat.long_softirq_threshold = new_threshold_us;
	atomic64_set(&(long_irq_stat.long_softirq_count), 0);
	return count;
}

static ssize_t irq_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "%lld\n", long_irq_stat.long_irq_threshold);
}

static ssize_t irq_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	s64 new_threshold_us;
	int err = kstrtoll(buf, 10, &new_threshold_us);
	if (!err || new_threshold_us < 0) {
		return -EINVAL;
	}
	long_irq_stat.long_irq_threshold = new_threshold_us;
	atomic64_set(&(long_irq_stat.long_irq_count), 0);
	return count;
}

static ssize_t irq_storm_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "%lld\n", long_irq_stat.irq_storm_threshold_us);
}

static ssize_t irq_storm_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	s64 new_threshold_us;
	int err = kstrtoll(buf, 10, &new_threshold_us);
	if (!err || new_threshold_us < 0) {
		return -EINVAL;
	}
	long_irq_stat.irq_storm_threshold_us = new_threshold_us;
	return count;
}

static ssize_t irq_display_warning_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t count = 0;
	if (long_irq_stat.display_warning) {
		count += sysfs_emit_at(buf, count,"%s",
				"WARN is turned on\n");
	} else {
		count += sysfs_emit_at(buf, count,"%s",
				"WARN is turned off\n");
	}
	return count;
}

static ssize_t irq_display_warning_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int display_warn;
	int err = sscanf (buf, "%d", &display_warn);
	if (!err) {
		return count;
	}
	if (display_warn == 0) {
		long_irq_stat.display_warning = false;
	}
	if (display_warn == 1) {
		long_irq_stat.display_warning = true;
	}
	return count;
}

static ssize_t irq_stats_reset_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	memset(long_irq_stat.long_softirq_arr, 0, NR_SOFTIRQS *
		sizeof(long_irq_stat.long_softirq_arr[0]));
	memset(long_irq_stat.long_irq_arr, 0, MAX_IRQ_NUM *
		sizeof(long_irq_stat.long_irq_arr[0]));
	memset(long_irq_stat.irq_storms, 0, MAX_IRQ_NUM *
		sizeof(struct irq_storm_data));
	atomic64_set(&(long_irq_stat.long_irq_count), 0);
	atomic64_set(&(long_irq_stat.long_softirq_count), 0);
	return count;
}

static ssize_t long_runnable_metrics_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int cpu, i;
	ssize_t count = 0;
	struct top_rt_runnable trr;
	struct rt_runnable long_rt_runnable;
	struct rt_runnable sorted_trr[RT_RUNNABLE_ARR_SIZE];

	for_each_possible_cpu(cpu) {
		count += sysfs_emit_at(buf, count, "cpu %d\n",cpu);
		spin_lock(&per_cpu(rt_runnable_lock, cpu));
		trr = per_cpu(top_rt_runnable, cpu);
		spin_unlock(&per_cpu(rt_runnable_lock, cpu));
		count += sysfs_emit_at(buf, count, "LONG RT_RUNNABLE: %lld\n",
				atomic64_read(&(trr.count)));

		for (i = 0; i < RT_RUNNABLE_ARR_SIZE; i++) {
			long_rt_runnable = trr.rt_runnable[i];
			strlcpy(sorted_trr[i].comm, long_rt_runnable.comm, TASK_COMM_LEN);
			strlcpy(sorted_trr[i].prev_comm, long_rt_runnable.prev_comm,
				TASK_COMM_LEN);
			sorted_trr[i].latency = long_rt_runnable.latency;
		}
		sort(sorted_trr, RT_RUNNABLE_ARR_SIZE, sizeof(struct rt_runnable),
						runnable_latency_cmp, NULL);
		for (i = 0; i < RT_RUNNABLE_ARR_SIZE; i++) {
			count += sysfs_emit_at(buf, count, "%s %llu %s\n",
				(const char *)sorted_trr[i].comm,
				sorted_trr[i].latency,
				(const char *)sorted_trr[i].prev_comm);
		}
		count += sysfs_emit_at(buf, count, "\n");
	}
	return count;
}

static ssize_t runnable_stats_reset_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int cpu;
	struct top_rt_runnable *trr;

	for_each_possible_cpu(cpu) {
		spin_lock(&per_cpu(rt_runnable_lock, cpu));
		trr = &per_cpu(top_rt_runnable, cpu);
		atomic64_set(&(trr->count), 0);
		memset(trr->rt_runnable, 0, sizeof(struct rt_runnable) *
							RT_RUNNABLE_ARR_SIZE);
		spin_unlock(&per_cpu(rt_runnable_lock, cpu));
	}
	return count;
}

static ssize_t runnable_stats_enable_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	register_trace_sched_wakeup(vh_sched_wakeup_pixel_mod, NULL);
	register_trace_sched_switch(vh_sched_switch_pixel_mod, NULL);
	return count;
}

static ssize_t runnable_stats_disable_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	unregister_trace_sched_wakeup(vh_sched_wakeup_pixel_mod, NULL);
	unregister_trace_sched_switch(vh_sched_switch_pixel_mod, NULL);
	return count;
}

static struct kobj_attribute resume_latency_metrics_attr = __ATTR(resume_latency_metrics,
							  0664,
							  resume_latency_metrics_show,
							  resume_latency_metrics_store);
static struct kobj_attribute resume_latency_threshold_attr = __ATTR(threshold,
							  0664,
							  resume_latency_threshold_show,
							  resume_latency_threshold_store);
static struct kobj_attribute resume_latency_display_warning_attr = __ATTR(display_warning,
							  0664,
							  resume_latency_display_warning_show,
							  resume_latency_display_warning_store);
static struct kobj_attribute long_irq_metrics_attr = __ATTR(long_irq_metrics,
							  0444,
							  long_irq_metrics_show,
							  NULL);

static struct kobj_attribute storm_irq_metrics_attr = __ATTR(storm_irq_metrics,
							  0444,
							  storm_irq_metrics_show,
							  NULL);
static struct kobj_attribute softirq_threshold_attr = __ATTR(softirq_threshold,
							  0664,
							  softirq_threshold_show,
							  softirq_threshold_store);
static struct kobj_attribute irq_threshold_attr = __ATTR(irq_threshold,
							  0664,
							  irq_threshold_show,
							  irq_threshold_store);
static struct kobj_attribute irq_storm_threshold_attr = __ATTR(irq_storm_threshold,
							  0664,
							  irq_storm_threshold_show,
							  irq_storm_threshold_store);
static struct kobj_attribute irq_display_warning_attr = __ATTR(display_warning,
							  0664,
							  irq_display_warning_show,
							  irq_display_warning_store);
static struct kobj_attribute irq_stats_reset_attr = __ATTR(
							stats_reset,
							0200,
							NULL,
							irq_stats_reset_store);
static struct kobj_attribute long_runnable_metrics_attr = __ATTR(stats,
							  0444,
							  long_runnable_metrics_show,
							  NULL);

static struct kobj_attribute runnable_stats_reset_attr = __ATTR(
							stats_reset,
							0200,
							NULL,
							runnable_stats_reset_store);
static struct kobj_attribute runnable_stats_enable_attr = __ATTR(enable,
							  0200,
							  NULL,
							  runnable_stats_enable_store);
static struct kobj_attribute runnable_stats_disable_attr = __ATTR(disable,
							  0200,
							  NULL,
							  runnable_stats_disable_store);

static struct attribute *irq_attrs[] = {
	&long_irq_metrics_attr.attr,
	&storm_irq_metrics_attr.attr,
	&softirq_threshold_attr.attr,
	&irq_threshold_attr.attr,
	&irq_storm_threshold_attr.attr,
	&irq_display_warning_attr.attr,
	&irq_stats_reset_attr.attr,
	NULL
};

static const struct attribute_group irq_attr_group = {
	.attrs = irq_attrs,
	.name = "irq"
};

static struct attribute *resume_latency_attrs[] = {
	&resume_latency_metrics_attr.attr,
	&resume_latency_threshold_attr.attr,
	&resume_latency_display_warning_attr.attr,
	NULL
};

static const struct attribute_group resume_latency_attr_group = {
	.attrs = resume_latency_attrs,
	.name = "resume_latency"
};

static struct attribute *runnable_attrs[] = {
	&long_runnable_metrics_attr.attr,
	&runnable_stats_reset_attr.attr,
	&runnable_stats_enable_attr.attr,
	&runnable_stats_disable_attr.attr,
	NULL
};

static const struct attribute_group runnable_attr_group = {
	.attrs = runnable_attrs,
	.name = "runnable"
};


/*********************************************************************
 *                  		INITIALIZE DRIVER                        *
 *********************************************************************/

int perf_metrics_init(struct kobject *metrics_kobj)
{
	int cpu;
	int ret = 0;

	if (!metrics_kobj) {
		pr_err("metrics_kobj is not initialized\n");
		return -EINVAL;
	}
	if (sysfs_create_group(metrics_kobj, &resume_latency_attr_group)) {
		pr_err("failed to create resume_latency folder\n");
		return -ENOMEM;
	}
	if (sysfs_create_group(metrics_kobj, &irq_attr_group)) {
		pr_err("failed to create irq folder\n");
		return -ENOMEM;
	}
	if (sysfs_create_group(metrics_kobj, &runnable_attr_group)) {
		pr_err("failed to create runnable folder\n");
		return -ENOMEM;
	}

	spin_lock_init(&resume_latency_stats.resume_latency_stat_lock);
	resume_latency_stats.resume_latency_threshold = RESUME_LATENCY_DEFAULT_THRESHOLD;
	ret = register_trace_android_vh_early_resume_begin(
					vendor_hook_resume_begin, NULL);
	if (ret) {
		pr_err("Register resume begin vendor hook fail %d\n", ret);
		return ret;
	}
	ret = register_trace_android_vh_resume_end(
					vendor_hook_resume_end, NULL);
	if (ret) {
		pr_err("Register resume end vendor hook fail %d\n", ret);
		return ret;
	}

	long_irq_stat.long_softirq_threshold = 10000;
	long_irq_stat.long_irq_threshold = 500;
	long_irq_stat.irq_storm_threshold_us = 500;
	ret = register_trace_softirq_entry(hook_softirq_begin, NULL);
	if (ret) {
		pr_err("Register soft irq handler hook fail %d\n", ret);
		return ret;
	}
	ret = register_trace_softirq_exit(hook_softirq_end, NULL);
	if (ret) {
		pr_err("Register soft irq exit hook fail %d\n", ret);
		return ret;
	}
	ret = register_trace_irq_handler_entry(hook_irq_begin, NULL);
	if (ret) {
		pr_err("Register irq handler hook fail %d\n", ret);
		return ret;
	}
	ret = register_trace_irq_handler_exit(hook_irq_end, NULL);
	if (ret) {
		pr_err("Register irq exit hook fail %d\n", ret);
		return ret;
	}

	for_each_possible_cpu(cpu) {
		spin_lock_init(&per_cpu(rt_runnable_lock, cpu));
	}
	ret = register_trace_sched_wakeup(vh_sched_wakeup_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_sched_switch(vh_sched_switch_pixel_mod, NULL);
	if (ret)
		return ret;

	pr_info("perf_metrics driver initialized! :D\n");
	return ret;
}

