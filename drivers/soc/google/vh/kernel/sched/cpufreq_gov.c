// SPDX-License-Identifier: GPL-2.0
/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>

#include <linux/sched/cpufreq.h>
#include <trace/events/power.h>
#include <linux/perf_event.h>
#include <linux/jiffies.h>
#include <linux/pm_qos.h>
#include <uapi/linux/sched/types.h>

#include <soc/google/exynos_pm_qos.h>

#include <performance/gs_perf_mon/gs_perf_mon.h>
#include "../../../../../devfreq/google/governor_memlat.h"
#include "sched_events.h"
#include "sched_priv.h"

#if IS_ENABLED(CONFIG_PIXEL_EM)
#include "../../include/pixel_em.h"
#endif

#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

unsigned int __read_mostly sched_per_cpu_iowait_boost_max_value[CONFIG_VH_SCHED_MAX_CPU_NR] = {
	[0 ... CONFIG_VH_SCHED_MAX_CPU_NR - 1] = SCHED_CAPACITY_SCALE
};

DEFINE_PER_CPU(u64, dvfs_update_delay);
DEFINE_PER_CPU(unsigned long, response_time_mult);

struct sugov_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		down_rate_limit_scale_pow;
	unsigned int		response_time_ms;

	/* The field below for PMU poll */
	unsigned int		lcpi_threshold;
	unsigned int		spc_threshold;
	unsigned int		limit_frequency;
	bool			pmu_limit_enable;
};

struct sugov_policy {
	struct cpufreq_policy	*policy;

	struct sugov_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			min_rate_limit_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	unsigned int		down_rate_limit_scale_pow;
	unsigned int		freq_response_time_ms;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;
	unsigned int		prev_cached_raw_freq;

	/* The next fields are only needed if fast switch cannot be used: */
	struct			irq_work irq_work;
	struct			kthread_work work;
	struct			mutex work_lock;
	struct			kthread_worker worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;

	struct freq_qos_request	pmu_max_freq_req;
	bool			under_pmu_throttle;
	bool			relax_pmu_throttle;

#if IS_ENABLED(CONFIG_PIXEL_EM)
	struct pixel_em_profile *em_profile;
#endif
};

struct sugov_cpu {
	struct update_util_data	update_util;
	struct sugov_policy	*sg_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	unsigned long           util;
	unsigned long		bw_dl;
	unsigned long		max;

	/* The field below is for single-CPU policies only: */
#if IS_ENABLED(CONFIG_NO_HZ_COMMON)
	unsigned long		saved_idle_calls;
#endif
};

static cpumask_t pixel_sched_governor_mask = CPU_MASK_NONE;
static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);
DEFINE_PER_CPU(struct uclamp_stats, uclamp_stats);
static struct kthread_worker pmu_worker;
static struct kthread_work pmu_work;
static struct irq_work pmu_irq_work;
static DEFINE_SPINLOCK(pmu_poll_enable_lock);
static u64 pmu_poll_last_update;
static bool pmu_poll_cancelling;
static bool pmu_poll_in_progress;
extern bool pmu_poll_enabled;
extern unsigned int pmu_poll_time_ms;

static void pmu_poll_defer_work(u64 time);

#if IS_ENABLED(CONFIG_UCLAMP_TASK) && IS_ENABLED(CONFIG_FAIR_GROUP_SCHED)
extern unsigned long cpu_util_cfs_group_mod(int cpu);
#else
#define cpu_util_cfs_group_mod cpu_util_cfs
#endif

unsigned int map_scaling_freq(int cpu, unsigned int freq)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);

	return policy ? clamp(freq, policy->min, policy->max) : freq;
}

#if !IS_ENABLED(CONFIG_TICK_DRIVEN_LATGOV)
extern int get_ev_data(int cpu, unsigned long *inst, unsigned long *cyc,
			unsigned long *stall, unsigned long *l2_cachemiss,
			unsigned long *l3_cachemiss, unsigned long *mem_stall,
			unsigned long *l2_cache_wb, unsigned long *l3_cache_access,
			unsigned long *mem_count, unsigned long *cpu_freq);
#endif

/************************ Governor internals ***********************/
static inline bool sugov_em_profile_changed(struct sugov_policy *sg_policy)
{
#if IS_ENABLED(CONFIG_PIXEL_EM)
	struct pixel_em_profile **profile_ptr_snapshot;
	struct pixel_em_profile *profile;

	profile_ptr_snapshot = READ_ONCE(vendor_sched_pixel_em_profile);
	profile = READ_ONCE(*profile_ptr_snapshot);

	if (sg_policy->em_profile != profile) {
		sg_policy->em_profile = profile;
		return true;
	}
#endif

	return false;
}

static inline unsigned int
sugov_calc_freq_response_ms(struct sugov_policy *sg_policy)
{
	int cpu = cpumask_first(sg_policy->policy->cpus);
	unsigned long cap = arch_scale_cpu_capacity(cpu);

#if IS_ENABLED(CONFIG_PIXEL_EM)
	struct pixel_em_profile **profile_ptr_snapshot;
	struct pixel_em_profile *profile;

	profile_ptr_snapshot = READ_ONCE(vendor_sched_pixel_em_profile);
	profile = READ_ONCE(*profile_ptr_snapshot);
	if (profile) {
		struct pixel_em_cluster *cluster = profile->cpu_to_cluster[cpu];
		struct pixel_em_opp *sec_max_opp;

		if (!cluster || !cluster->num_opps)
			goto out;

		if (cluster->num_opps >= 2) {
			sec_max_opp = &cluster->opps[cluster->num_opps-2];
			cap = sec_max_opp->capacity + 1;
		} else {
			sec_max_opp = &cluster->opps[0];
			cap = sec_max_opp->capacity;
		}
	}
out:
#endif
	/*
	 * We will request max_freq as soon as util crosses the capacity at
	 * second highest frequency. So effectively our response time is the
	 * util at which we cross the cap@2nd_highest_freq.
	 *
	 * We need to export some functions from GKI to get the 2nd max
	 * frequency without pixel_em.
	 */
	return approximate_runtime(cap);
}

static inline void sugov_update_response_time_mult(struct sugov_policy *sg_policy,
						   bool reset_defaults)
{
	unsigned long mult;
	int cpu;

	if (reset_defaults) {
		unsigned int new_response_time_ms = sugov_calc_freq_response_ms(sg_policy);

		/*
		 * If user has requested a value that is different than the
		 * default leave it as-is to avoid races between setting the
		 * value and changing the em
		 */
		if (sg_policy->tunables->response_time_ms == sg_policy->freq_response_time_ms)
			sg_policy->tunables->response_time_ms = new_response_time_ms;

		sg_policy->freq_response_time_ms = new_response_time_ms;
	}

	mult = sg_policy->freq_response_time_ms * SCHED_CAPACITY_SCALE;
	mult /=	sg_policy->tunables->response_time_ms;

	if (SCHED_WARN_ON(!mult))
		mult = SCHED_CAPACITY_SCALE;

	for_each_cpu(cpu, sg_policy->policy->cpus)
		per_cpu(response_time_mult, cpu) = mult;
}

/*
 * Implements a headroom function which gives the utilization (or the tasks
 * extra CPU bandwidth) to grow. The goal is to use the outcome to select the
 * frequency. We don't want an exact frequency selection so that if the tasks
 * running on the CPU don't go to sleep, they'll grow in that additional
 * headroom until we do the next frequency update to a higher one.
 */
unsigned long __always_inline
apply_dvfs_headroom(unsigned long util, int cpu, bool tapered)
{

	if (static_branch_likely(&auto_dvfs_headroom_enable)) {
		u64 limit = per_cpu(dvfs_update_delay, cpu);

		/*
		 * Only apply a small headroom until the next freq request can
		 * be taken.
		 */
		return approximate_util_avg(util, limit);
	}

	if (tapered && static_branch_unlikely(&tapered_dvfs_headroom_enable)) {
		unsigned long capacity = capacity_orig_of(cpu);
		unsigned long headroom;

		if (util >= capacity)
			return util;

		/*
		 * Taper the boosting at e top end as these are expensive and
		 * we don't need that much of a big headroom as we approach max
		 * capacity
		 *
		 */
		headroom = (capacity - util);
		/* formula: headroom * (1.X - 1) == headroom * 0.X */
		headroom = headroom * (sched_dvfs_headroom[cpu] - SCHED_CAPACITY_SCALE) >> SCHED_CAPACITY_SHIFT;
		return util + headroom;
	}

	return util * sched_dvfs_headroom[cpu] >> SCHED_CAPACITY_SHIFT;
}

/*
 * Shrink or expand how long it takes to reach the maximum performance of the
 * policy.
 *
 * sg_policy->freq_response_time_ms is a constant value defined by PELT
 * HALFLIFE and the capacity of the policy (assuming HMP systems).
 *
 * sg_policy->tunables->response_time_ms is a user defined response time. By
 * setting it lower than sg_policy->freq_response_time_ms, the system will
 * respond faster to changes in util, which will result in reaching maximum
 * performance point quicker. By setting it higher, it'll slow down the amount
 * of time required to reach the maximum OPP.
 *
 * This should be applied when selecting the frequency.
 */
static inline unsigned long
sugov_apply_response_time(unsigned long util, int cpu)
{
	unsigned long mult;

	if (!static_branch_likely(&auto_dvfs_headroom_enable))
		return util;

	mult = per_cpu(response_time_mult, cpu) * util;

	return mult >> SCHED_CAPACITY_SHIFT;
}

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
static bool check_pmu_limit_conditions(u64 lcpi, u64 spc, struct sugov_policy *sg_policy)
{
	if (sg_policy->tunables->lcpi_threshold <= lcpi &&
	    sg_policy->tunables->spc_threshold <= spc)
		return true;

	return false;
}
#else
static bool check_pmu_limit_conditions(u64 spc, struct sugov_policy *sg_policy)
{
	if (sg_policy->tunables->spc_threshold <= spc)
		return true;

	return false;
}
#endif

static inline void trace_pmu_limit(struct sugov_policy *sg_policy)
{
	if (trace_clock_set_rate_enabled()) {
		char trace_name[32] = {0};
		scnprintf(trace_name, sizeof(trace_name), "pmu_limit_cpu%d",
			  sg_policy->policy->cpu);
		trace_clock_set_rate(trace_name, sg_policy->under_pmu_throttle ?
				     sg_policy->tunables->limit_frequency :
				     sg_policy->policy->cpuinfo.max_freq,
				     raw_smp_processor_id());
	}
}

static bool check_sg_policy_initialized(void)
{
	unsigned int cpu = 0;
	struct cpufreq_policy *policy = NULL;
	struct sugov_policy *sg_policy = NULL;

	if (cpumask_weight(&pixel_sched_governor_mask) != pixel_cpu_num)
		return false;

	while (cpu < pixel_cpu_num) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			pr_err("no cpufreq policy for cpu %d\n", cpu);
			cpufreq_cpu_put(policy);
			return false;
		}

		sg_policy = policy->governor_data;
		if (!sg_policy) {
			pr_err("no sugov policy for cpu %d\n", cpu);
			cpufreq_cpu_put(policy);
			return false;
		}

		cpu = cpumask_last(policy->related_cpus) + 1;
		cpufreq_cpu_put(policy);
	}

	return true;
}

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
void update_uclamp_stats(int cpu, u64 time)
{
	struct uclamp_stats *stats = &per_cpu(uclamp_stats, cpu);
	s64 delta_ns = time - stats->last_update_time;
	struct rq *rq = cpu_rq(cpu);
	unsigned long cpu_util = min(capacity_orig_of(cpu), cpu_util_cfs(rq) + cpu_util_rt(rq));
	unsigned long cpu_util_max_clamped = min(capacity_orig_of(cpu), cpu_util_cfs_group_mod(cpu) +
						 cpu_util_rt(rq));
	unsigned int uclamp_min = READ_ONCE(rq->uclamp[UCLAMP_MIN].value);
	unsigned int uclamp_max = READ_ONCE(rq->uclamp[UCLAMP_MAX].value);
	long util_diff_min, util_diff_max;
	unsigned long flags;

	if (delta_ns <= 0)
		return;

	spin_lock_irqsave(&stats->lock, flags);
	stats->last_update_time = time;

	if(rq->curr == rq->idle)
		goto out;

	if (stats->last_min_in_effect) {
		stats->effect_time_in_state_min[stats->last_uclamp_min_index] += delta_ns;
		stats->util_diff_min[stats->last_util_diff_min_index] += delta_ns;
	}

	if (stats->last_max_in_effect) {
		stats->effect_time_in_state_max[stats->last_uclamp_max_index] += delta_ns;
		stats->util_diff_max[stats->last_util_diff_max_index] += delta_ns;
	}

	stats->total_time += delta_ns;

	util_diff_min = (long)uclamp_min - (long)cpu_util;
	util_diff_max = (long)cpu_util - (long)cpu_util_max_clamped;

	/* uclamp min is in effect */
	if (util_diff_min > 0) {
		stats->last_min_in_effect = true;
		stats->last_util_diff_min_index = ((util_diff_min * 100) >> SCHED_CAPACITY_SHIFT)
						  / UCLAMP_STATS_STEP;
	} else
		stats->last_min_in_effect = false;

	/* uclamp max is in effect */
	if (util_diff_max > 0) {
		stats->last_max_in_effect = true;
		stats->last_util_diff_max_index = ((util_diff_max * 100) >> SCHED_CAPACITY_SHIFT)
						  / UCLAMP_STATS_STEP;
	} else
		stats->last_max_in_effect = false;

	stats->time_in_state_min[stats->last_uclamp_min_index] += delta_ns;
	stats->time_in_state_max[stats->last_uclamp_max_index] += delta_ns;
	stats->last_uclamp_min_index = (((uclamp_min + UCLAMP_STATS_STEP) *
				100) >> SCHED_CAPACITY_SHIFT) / UCLAMP_STATS_STEP;
	stats->last_uclamp_max_index = (((uclamp_max + UCLAMP_STATS_STEP) *
				100) >> SCHED_CAPACITY_SHIFT) / UCLAMP_STATS_STEP;
out:
	spin_unlock_irqrestore(&stats->lock, flags);
}

void reset_uclamp_stats(void)
{
	int i;

	for (i = 0; i < pixel_cpu_num; i++) {
		unsigned long flags;
		u64 time;
		struct rq_flags rf;
		struct uclamp_stats *stats = &per_cpu(uclamp_stats, i);

		rq_lock_irqsave(cpu_rq(i), &rf);
		update_rq_clock(cpu_rq(i));
		time = rq_clock(cpu_rq(i));
		rq_unlock_irqrestore(cpu_rq(i), &rf);

		spin_lock_irqsave(&stats->lock, flags);
		stats->last_min_in_effect = false;
		stats->last_max_in_effect = false;
		stats->last_uclamp_min_index = 0;
		stats->last_uclamp_max_index = UCLAMP_STATS_SLOTS - 1;
		stats->last_util_diff_min_index = 0;
		stats->last_util_diff_max_index = 0;
		memset(stats->util_diff_min, 0, sizeof(u64) * UCLAMP_STATS_SLOTS);
		memset(stats->util_diff_max, 0, sizeof(u64) * UCLAMP_STATS_SLOTS);
		stats->total_time = 0;
		stats->last_update_time = time;
		memset(stats->time_in_state_min, 0, sizeof(u64) * UCLAMP_STATS_SLOTS);
		memset(stats->time_in_state_max, 0, sizeof(u64) * UCLAMP_STATS_SLOTS);
		memset(stats->effect_time_in_state_min, 0, sizeof(u64) * UCLAMP_STATS_SLOTS);
		memset(stats->effect_time_in_state_max, 0, sizeof(u64) * UCLAMP_STATS_SLOTS);
		spin_unlock_irqrestore(&stats->lock, flags);
	}
}

void init_uclamp_stats(void)
{
	int i;

	for (i = 0; i < pixel_cpu_num; i++) {
		struct uclamp_stats *stats = &per_cpu(uclamp_stats, i);
		spin_lock_init(&stats->lock);
	}
	reset_uclamp_stats();
}
#endif

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	/*
	 * Since cpufreq_update_util() is called with rq->lock held for
	 * the @target_cpu, our per-CPU data is fully serialized.
	 *
	 * However, drivers cannot in general deal with cross-CPU
	 * requests, so while get_next_freq() will work, our
	 * sugov_update_commit() call may not for the fast switching platforms.
	 *
	 * Hence stop here for remote requests if they aren't supported
	 * by the hardware, as calculating the frequency is pointless if
	 * we cannot in fact act on it.
	 *
	 * This is needed on the slow switching platforms too to prevent CPUs
	 * going offline from leaving stale IRQ work items behind.
	 */
	if (!cpufreq_this_cpu_can_update(sg_policy->policy))
		return false;

	if (unlikely(sg_policy->limits_changed)) {
		sg_policy->limits_changed = false;
		sg_policy->need_freq_update = true;
		return true;
	}

	delta_ns = time - sg_policy->last_freq_update_time;

	return delta_ns >= sg_policy->min_rate_limit_ns;
}

static bool sugov_up_down_rate_limit(struct sugov_policy *sg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;
	unsigned long comp;
	int i;

	delta_ns = time - sg_policy->last_freq_update_time;

	if (next_freq > sg_policy->next_freq &&
	    delta_ns < sg_policy->up_rate_delay_ns)
			return true;

	/*
	 * TODO: consider using a table with ratio and rate limit defined
	 * Here consider the ratio of freq change e.g. selecting larger rate limit
	 * when freq changed dramatically and smaller rate limit for the opposite.
	 * here for simple, rate_limit = down_rate_delay_ns * new_freq / old_freq
	 * Also we are not going to update update_min_rate_limit_ns, so the minimal
	 * rate limit is still the min(down_rate_delay_ns, up_rate_delay_ns).
	 */
	comp = sg_policy->down_rate_delay_ns * next_freq;
	for (i = 0; i < sg_policy->down_rate_limit_scale_pow - 1; i++) {
		comp = comp / sg_policy->next_freq * next_freq;
	}
	if (next_freq < sg_policy->next_freq && delta_ns * sg_policy->next_freq < comp)
		return true;

	return false;
}

static bool sugov_update_next_freq(struct sugov_policy *sg_policy, u64 time,
				   unsigned int next_freq)
{
	bool ignore_rate_limit = sg_policy->need_freq_update;
	sg_policy->need_freq_update = false;

	if (sg_policy->next_freq == next_freq)
		return false;

	if (!ignore_rate_limit && sugov_up_down_rate_limit(sg_policy, time, next_freq)) {
		/* Restore cached freq as next_freq is not changed */
		sg_policy->cached_raw_freq = sg_policy->prev_cached_raw_freq;
		return false;
	}

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	return true;
}

static void sugov_deferred_update(struct sugov_policy *sg_policy)
{
	if (!sg_policy->work_in_progress) {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.

 * @sg_policy: schedutil policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * If the utilization is frequency-invariant, choose the new frequency to be
 * proportional to it, that is
 *
 * next_freq = C * max_freq * util / max
 *
 * Otherwise, approximate the would-be frequency-invariant utilization by
 * util_raw * (curr_freq / max_freq) which leads to
 *
 * next_freq = C * curr_freq * util_raw / max
 *
 * Take C = 1.25 for the frequency tipping point at (util / max) = 0.8.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = policy->cpuinfo.max_freq;

	freq = map_util_freq_pixel_mod(util, freq, max, policy->cpu);
	trace_sugov_next_freq(policy->cpu, util, max, freq);

	if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
		return sg_policy->next_freq;

	sg_policy->prev_cached_raw_freq = sg_policy->cached_raw_freq;
	sg_policy->cached_raw_freq = freq;

	freq = cpufreq_driver_resolve_freq(policy, freq);

	/* Workaround a bug in GKI where we can escape policy limits */
	freq = clamp_val(freq, policy->min, policy->max);

	return freq;
}

/*
 * This function computes an effective utilization for the given CPU, to be
 * used for frequency selection given the linear relation: f = u * f_max.
 *
 * The scheduler tracks the following metrics:
 *
 *   cpu_util_{cfs,rt,dl,irq}()
 *   cpu_bw_dl()
 *
 * Where the cfs,rt and dl util numbers are tracked with the same metric and
 * synchronized windows and are thus directly comparable.
 *
 * The cfs,rt,dl utilization are the running times measured with rq->clock_task
 * which excludes things like IRQ and steal-time. These latter are then accrued
 * in the irq utilization.
 *
 * The DL bandwidth number otoh is not a measured metric but a value computed
 * based on the task model parameters and gives the minimal utilization
 * required to meet deadlines.
 */
__always_inline
unsigned long schedutil_cpu_util_pixel_mod(int cpu, unsigned long util_cfs,
				 unsigned long max, enum cpu_util_type type,
				 struct task_struct *p)
{
	unsigned long dl_util, util, irq;
	struct rq *rq = cpu_rq(cpu);

	if (!uclamp_is_used() &&
	    type == FREQUENCY_UTIL && rt_rq_is_runnable(&rq->rt)) {
		return max;
	}

	/*
	 * Early check to see if IRQ/steal time saturates the CPU, can be
	 * because of inaccuracies in how we track these -- see
	 * update_irq_load_avg().
	 */
	irq = cpu_util_irq(rq);
	if (unlikely(irq >= max))
		return max;

	/*
	 * Because the time spend on RT/DL tasks is visible as 'lost' time to
	 * CFS tasks and we use the same metric to track the effective
	 * utilization (PELT windows are synchronized) we can directly add them
	 * to obtain the CPU's actual utilization.
	 *
	 * CFS and RT utilization can be boosted or capped, depending on
	 * utilization clamp constraints requested by currently RUNNABLE
	 * tasks.
	 * When there are no CFS RUNNABLE tasks, clamps are released and
	 * frequency will be gracefully reduced with the utilization decay.
	 */
	util = util_cfs + cpu_util_rt(rq);
	if (type == FREQUENCY_UTIL) {
		/*
		 * Speed up/slow down response timee first then apply DVFS
		 * headroom. We only want to do that for cfs+rt util.
		 */
		util = sugov_apply_response_time(util, cpu);
		util = apply_dvfs_headroom(util, cpu, true);
		util = uclamp_rq_util_with(rq, util, p);
		trace_schedutil_cpu_util_clamp(cpu, util_cfs, cpu_util_rt(rq), util, max);
	}

	dl_util = cpu_util_dl(rq);

	/*
	 * For frequency selection we do not make cpu_util_dl() a permanent part
	 * of this sum because we want to use cpu_bw_dl() later on, but we need
	 * to check if the CFS+RT+DL sum is saturated (ie. no idle time) such
	 * that we select f_max when there is no idle time.
	 *
	 * NOTE: numerical errors or stop class might cause us to not quite hit
	 * saturation when we should -- something for later.
	 */
	if (util + dl_util >= max)
		return max;

	/*
	 * OTOH, for energy computation we need the estimated running time, so
	 * include util_dl and ignore dl_bw.
	 */
	if (type == ENERGY_UTIL)
		util += dl_util;

	/*
	 * There is still idle time; further improve the number by using the
	 * irq metric. Because IRQ/steal time is hidden from the task clock we
	 * need to scale the task numbers:
	 *
	 *              max - irq
	 *   U' = irq + --------- * U
	 *                 max
	 *
	 * We don't need to apply dvfs headroom to scale_irq_capacity() as util
	 * (U) already got the headroom applied. Only the 'irq' part needs to
	 * be multiplied by the headroom.
	 */
	util = scale_irq_capacity(util, irq, max);
	util += type == FREQUENCY_UTIL ? apply_dvfs_headroom(irq, cpu, false) : irq;

	/*
	 * Bandwidth required by DEADLINE must always be granted while, for
	 * FAIR and RT, we use blocked utilization of IDLE CPUs as a mechanism
	 * to gracefully reduce the frequency when no tasks show up for longer
	 * periods of time.
	 *
	 * Ideally we would like to set bw_dl as min/guaranteed freq and util +
	 * bw_dl as requested freq. However, cpufreq is not yet ready for such
	 * an interface. So, we only do the latter for now.
	 */
	if (type == FREQUENCY_UTIL)
		util += apply_dvfs_headroom(cpu_bw_dl(rq), cpu, false);

	return min(max, util);
}

static void __always_inline
sugov_get_util(struct sugov_cpu *sg_cpu)
{
	struct rq *rq = cpu_rq(sg_cpu->cpu);
	unsigned long max = arch_scale_cpu_capacity(sg_cpu->cpu);

	sg_cpu->max = max;
	sg_cpu->bw_dl = cpu_bw_dl(rq);

	sg_cpu->util = schedutil_cpu_util_pixel_mod(sg_cpu->cpu,
			cpu_util_cfs_group_mod(sg_cpu->cpu), max, FREQUENCY_UTIL, NULL);
}

/**
 * sugov_iowait_reset() - Reset the IO boost status of a CPU.
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @set_iowait_boost: true if an IO boost has been requested
 *
 * The IO wait boost of a task is disabled after a tick since the last update
 * of a CPU. If a new IO wait boost is requested after more then a tick, then
 * we enable the boost starting from IOWAIT_BOOST_MIN, which improves energy
 * efficiency by ignoring sporadic wakeups from IO.
 */
static bool sugov_iowait_reset(struct sugov_cpu *sg_cpu, u64 time,
			       bool set_iowait_boost)
{
	s64 delta_ns = time - sg_cpu->last_update;

	/* Reset boost only if a tick has elapsed since last request */
	if (delta_ns <= TICK_NSEC)
		return false;

	sg_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	sg_cpu->iowait_boost_pending = set_iowait_boost;

	return true;
}

/**
 * sugov_iowait_boost() - Updates the IO boost status of a CPU.
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @flags: SCHED_CPUFREQ_IOWAIT if the task is waking up after an IO wait
 *
 * Each time a task wakes up after an IO operation, the CPU utilization can be
 * boosted to a certain utilization which doubles at each "frequent and
 * successive" wakeup from IO, ranging from IOWAIT_BOOST_MIN to the utilization
 * of the maximum OPP.
 *
 * To keep doubling, an IO boost has to be requested at least once per tick,
 * otherwise we restart from the utilization of the minimum OPP.
 */
static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
			       unsigned int flags)
{
	struct vendor_rq_struct *vrq = get_vendor_rq_struct(cpu_rq(sg_cpu->cpu));
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sg_cpu->iowait_boost &&
	    sugov_iowait_reset(sg_cpu, time, set_iowait_boost))
		return;

	/* Boost only tasks waking up after IO */
	if (!set_iowait_boost)
		return;

	/* Ensure boost doubles only one time at each request */
	if (sg_cpu->iowait_boost_pending)
		return;
	sg_cpu->iowait_boost_pending = true;

	/* Double the boost at each request */
	if (sg_cpu->iowait_boost) {
		sg_cpu->iowait_boost =
			min_t(unsigned int, sg_cpu->iowait_boost << 1,
			      sched_per_cpu_iowait_boost_max_value[sg_cpu->cpu]);
		return;
	}

	/* First wakeup after IO: start with minimum boost */
	sg_cpu->iowait_boost = IOWAIT_BOOST_MIN;

	/* Cater for a task with high iowait boost migrated to this CPU */
	sg_cpu->iowait_boost = max_t(unsigned long, sg_cpu->iowait_boost, vrq->iowait_boost);
}

/**
 * sugov_iowait_apply() - Apply the IO boost to a CPU.
 * @sg_cpu: the sugov data for the cpu to boost
 * @time: the update time from the caller
 *
 * A CPU running a task which woken up after an IO operation can have its
 * utilization boosted to speed up the completion of those IO operations.
 * The IO boost value is increased each time a task wakes up from IO, in
 * sugov_iowait_apply(), and it's instead decreased by this function,
 * each time an increase has not been requested (!iowait_boost_pending).
 *
 * A CPU which also appears to have been idle for at least one tick has also
 * its IO boost utilization reset.
 *
 * This mechanism is designed to boost high frequently IO waiting tasks, while
 * being more conservative on tasks which does sporadic IO operations.
 */
static void sugov_iowait_apply(struct sugov_cpu *sg_cpu, u64 time)
{
	s64 delta_ns = time - sg_cpu->last_update;
	unsigned long boost;

	/* No boost currently required */
	if (!sg_cpu->iowait_boost)
		return;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sugov_iowait_reset(sg_cpu, time, false))
		return;

	/* Reduce boost only if a 1ms has elapsed since last request */
	if (delta_ns <= NSEC_PER_MSEC)
		goto apply_boost;

	if (!sg_cpu->iowait_boost_pending) {
		/*
		 * No boost pending; reduce the boost value.
		 */
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
			sg_cpu->iowait_boost = 0;
			return;
		}
	}

apply_boost:
	sg_cpu->iowait_boost_pending = false;

	/*
	 * sg_cpu->util is already in capacity scale; convert iowait_boost
	 * into the same scale so we can compare.
	 */
	boost = (sg_cpu->iowait_boost * sg_cpu->max) >> SCHED_CAPACITY_SHIFT;
	boost = max(boost, sg_cpu->util);
	sg_cpu->util = uclamp_rq_util_with(cpu_rq(sg_cpu->cpu), boost, NULL);
}

#if IS_ENABLED(CONFIG_NO_HZ_COMMON)
static bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

/*
 * Make sugov_should_update_freq() ignore the rate limit when DL
 * has increased the utilization.
 */
static inline void ignore_dl_rate_limit(struct sugov_cpu *sg_cpu)
{
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_dl)
		sg_cpu->sg_policy->limits_changed = true;
}

#if IS_ENABLED(USE_UPDATE_SINGLE)
static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	unsigned int next_f;
	bool busy;

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
	update_uclamp_stats(sg_cpu->cpu, time);
#endif

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	pmu_poll_defer_work(time);

	ignore_dl_rate_limit(sg_cpu);

	if (!sugov_should_update_freq(sg_cpu->sg_policy, time))
		return;

	/* Limits may have changed, don't skip frequency update */
	busy = !sg_cpu->sg_policy->need_freq_update && sugov_cpu_is_busy(sg_cpu);

	sugov_get_util(sg_cpu);

	trace_sugov_util_update(sg_cpu->cpu, sg_cpu->util, sg_cpu->max, flags);

	sugov_iowait_apply(sg_cpu, time);
	next_f = get_next_freq(sg_cpu->sg_policy, sg_cpu->util, sg_cpu->max);

	/*
	 * Do not reduce the frequency if the CPU has not been idle
	 * recently, as the reduction is likely to be premature then.
	 */

	if (!uclamp_rq_is_capped(cpu_rq(sg_cpu->cpu)) &&
	    busy && next_f < sg_cpu->sg_policy->next_freq) {
		next_f = sg_cpu->sg_policy->next_freq;

		/* Reset cached freq as next_freq has changed */
		sg_cpu->sg_policy->cached_raw_freq = sg_cpu->sg_policy->prev_cached_raw_freq;
	}

	if (!sugov_update_next_freq(sg_cpu->sg_policy, time, next_f))
		return;

	/*
	 * This code runs under rq->lock for the target CPU, so it won't run
	 * concurrently on two different CPUs for the same target and it is not
	 * necessary to acquire the lock in the fast switch case.
	 */
	if (sg_cpu->sg_policy->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(sg_cpu->sg_policy->policy, next_f);
	} else {
		raw_spin_lock(&sg_cpu->sg_policy->update_lock);
		sugov_deferred_update(sg_cpu->sg_policy);
		raw_spin_unlock(&sg_cpu->sg_policy->update_lock);
	}
}
#endif

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;

		sugov_get_util(j_sg_cpu);
		sugov_iowait_apply(j_sg_cpu, time);
		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;

		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
		}
	}

	return get_next_freq(sg_policy, util, max);
}

static void update_avg_real_cap_cluster(struct cpufreq_policy *policy)
{
	unsigned int j;
	for_each_cpu(j, policy->cpus) {
		struct task_struct *curr = cpu_rq(j)->curr;
		struct vendor_task_struct *vcurr = get_vendor_task_struct(curr);
		if (vcurr->adpf_adj) {
			get_task_struct(curr);
			update_task_real_cap(curr);
			put_task_struct(curr);
		}
	}
}

static void
sugov_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	unsigned int next_f;

	raw_spin_lock(&sg_cpu->sg_policy->update_lock);

	sg_cpu->sg_policy->limits_changed |= flags & SCHED_PIXEL_FORCE_UPDATE;

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
	update_uclamp_stats(sg_cpu->cpu, time);
#endif

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	pmu_poll_defer_work(time);

	ignore_dl_rate_limit(sg_cpu);

	if (sugov_should_update_freq(sg_cpu->sg_policy, time)) {
		bool busy;

		next_f = sugov_next_freq_shared(sg_cpu, time);

		/* Limits may have changed, don't skip frequency update */
		busy = !sg_cpu->sg_policy->need_freq_update && sugov_cpu_is_busy(sg_cpu);

		/*
		 * Do not reduce the frequency if a single cpu policy has not
		 * been idle recently, as the reduction is likely to be
		 * premature then.
		 */
		if (static_branch_likely(&auto_dvfs_headroom_enable) &&
		    cpumask_weight(sg_cpu->sg_policy->policy->cpus) == 1 &&
		    !uclamp_rq_is_capped(cpu_rq(sg_cpu->cpu)) &&
		    busy && next_f < sg_cpu->sg_policy->next_freq) {
			next_f = sg_cpu->sg_policy->next_freq;

			/* Reset cached freq as next_freq has changed */
			sg_cpu->sg_policy->cached_raw_freq = sg_cpu->sg_policy->prev_cached_raw_freq;
		}

		if (!sugov_update_next_freq(sg_cpu->sg_policy, time, next_f))
			goto unlock;
		update_avg_real_cap_cluster(sg_cpu->sg_policy->policy);

		if (trace_sugov_util_update_enabled())
			trace_sugov_util_update(sg_cpu->cpu, sg_cpu->util, sg_cpu->max, flags);

		if (sg_cpu->sg_policy->policy->fast_switch_enabled)
			cpufreq_driver_fast_switch(sg_cpu->sg_policy->policy, next_f);
		else
			sugov_deferred_update(sg_cpu->sg_policy);
	}
unlock:
	raw_spin_unlock(&sg_cpu->sg_policy->update_lock);
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);
	unsigned int freq;
	unsigned long flags;
	bool relax_pmu_throttle;

	if (sugov_em_profile_changed(sg_policy))
		sugov_update_response_time_mult(sg_policy, true);

	/*
	 * Hold sg_policy->update_lock shortly to handle the case where:
	 * incase sg_policy->next_freq is read here, and then updated by
	 * sugov_deferred_update() just before work_in_progress is set to false
	 * here, we may miss queueing the new update.
	 *
	 * Note: If a work was queued after the update_lock is released,
	 * sugov_work() will just be called again by kthread_work code; and the
	 * request will be proceed before the sugov thread sleeps.
	 */
	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	freq = sg_policy->next_freq;
	sg_policy->work_in_progress = false;
	relax_pmu_throttle = sg_policy->relax_pmu_throttle;
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

	if (relax_pmu_throttle) {
		freq_qos_update_request(&sg_policy->pmu_max_freq_req,
					sg_policy->policy->cpuinfo.max_freq);

		sg_policy->under_pmu_throttle = false;
		sg_policy->relax_pmu_throttle = false;

		trace_pmu_limit(sg_policy);
	}

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);

	/*
	 * Check if the memory frequencies need to be updated. This
	 * is an opportunistic path for updating the memory dvfs
	 * governors.
	 */
	gs_perf_mon_update_clients();
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

int pmu_poll_enable(void)
{
	// Check pmu_poll_init finish successfully
	if (!pmu_work.func || !pmu_worker.task)
		return -EBUSY;

	// Check sg_policy for whole clusters are initialized correctly
	if (!check_sg_policy_initialized())
		return -EBUSY;

	spin_lock(&pmu_poll_enable_lock);

	if (pmu_poll_cancelling) {
		spin_unlock(&pmu_poll_enable_lock);
		return -EBUSY;
	}

	if (!pmu_poll_enabled) {
		/*
		 * If we initialize and clean up properly, this should never
		 * happen.
		 */
		if (WARN_ON(pmu_poll_in_progress))
			pmu_poll_in_progress = false;

		pmu_poll_enabled = true;
		pmu_poll_last_update = 0;
	}

	spin_unlock(&pmu_poll_enable_lock);

	return 0;
}

void pmu_poll_disable(void)
{
	unsigned int cpu = 0;
	struct cpufreq_policy *policy = NULL;
	struct sugov_policy *sg_policy = NULL;

	spin_lock(&pmu_poll_enable_lock);

	if (pmu_poll_enabled) {
		pmu_poll_enabled = false;

		irq_work_sync(&pmu_irq_work);

		/*
		 * We must temporarily drop the lock to cancel the pmu_work.
		 * pmu_poll_cancelling should block any potential attempt to
		 * enable pmu_poll while the lock is dropped.
		 *
		 * pmu_defer_work() should see pmu_poll_enabled === false and
		 * continue to be blocked/NOP.
		 */
		pmu_poll_cancelling = true;
		spin_unlock(&pmu_poll_enable_lock);
		kthread_cancel_work_sync(&pmu_work);

		while (cpu < pixel_cpu_num) {
			policy = cpufreq_cpu_get(cpu);
			sg_policy = policy->governor_data;

			if (sg_policy)
				freq_qos_update_request(&sg_policy->pmu_max_freq_req,
							policy->cpuinfo.max_freq);
			else
				pr_err("no sugov policy for cpu %d\n", cpu);

			cpu = cpumask_last(policy->related_cpus) + 1;
			cpufreq_cpu_put(policy);
		}

		spin_lock(&pmu_poll_enable_lock);
		pmu_poll_cancelling = false;
	}

	spin_unlock(&pmu_poll_enable_lock);
}

static void pmu_limit_work(struct kthread_work *work)
{
	int ret;
	unsigned int cpu = 0, ccpu;
	struct sugov_policy *sg_policy = NULL;
	struct cpufreq_policy *policy = NULL;
	u64 lcpi = 0, spc = 0;
	unsigned int next_max_freq;
	unsigned long inst, cyc, stall, l3_cachemiss, mem_stall;
	unsigned long cpu_freq;
	unsigned long flags;
	bool pmu_throttle = false;

#if IS_ENABLED(CONFIG_TICK_DRIVEN_LATGOV)
	struct gs_cpu_perf_data perf_data;
#else
	unsigned long l2_cache_wb, l2_cachemiss, l3_cache_access, mem_count;
#endif

	while (cpu < pixel_cpu_num) {
		policy = cpufreq_cpu_get(cpu);
		sg_policy = policy->governor_data;
		next_max_freq = policy->cpuinfo.max_freq;
		pmu_throttle = false;

		// If pmu_limit_enable is not set, or policy max is lower than pum limit freq,
		// such as under thermal throttling, we don't need to call freq_qos_update_request
		// unless it's currently under throttle.
		if (!sg_policy->tunables->pmu_limit_enable ||
		    policy->max < sg_policy->tunables->limit_frequency) {
			if (unlikely(sg_policy->under_pmu_throttle)) {
				goto update_next_max_freq;
			} else {
				cpu = cpumask_last(policy->related_cpus) + 1;
				cpufreq_cpu_put(policy);
				continue;
			}
		}

		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		sg_policy->under_pmu_throttle = false;
		sg_policy->relax_pmu_throttle = false;
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

		for_each_cpu(ccpu, policy->cpus) {
			if (!cpu_online(ccpu)) {
				pr_info_ratelimited("cpu %d is offline, pmu read fail\n", ccpu);
				goto update_next_max_freq;
			}

#if IS_ENABLED(CONFIG_TICK_DRIVEN_LATGOV)
			ret = gs_perf_mon_get_data(ccpu, &perf_data);
			if (ret) {
				sg_policy->tunables->pmu_limit_enable = false;
				pr_err_ratelimited("pmu ev_data read fail\n");
				goto update_next_max_freq;
			}

			cyc = perf_data.perf_ev_last_delta[PERF_CYCLE_IDX];
			cpu_freq = cyc / perf_data.time_delta_us;
			l3_cachemiss = perf_data.perf_ev_last_delta[PERF_L3_CACHE_MISS_IDX];
			inst = perf_data.perf_ev_last_delta[PERF_INST_IDX];
			mem_stall = perf_data.perf_ev_last_delta[PERF_STALL_BACKEND_MEM_IDX];
			stall = 0;
#else
			ret = get_ev_data(ccpu, &inst, &cyc, &stall, &l2_cachemiss,
					  &l3_cachemiss, &mem_stall, &l2_cache_wb,
					  &l3_cache_access, &mem_count, &cpu_freq);

			if (ret) {
				sg_policy->tunables->pmu_limit_enable = false;
				pr_err_ratelimited("pmu ev_data read fail\n");
				goto update_next_max_freq;
			}
#endif

			if (inst == 0 || cyc == 0) {
				pr_err_ratelimited("pmu read fail for cpu %d\n", ccpu);
				goto update_next_max_freq;
			}

			lcpi = l3_cachemiss * 1000 / inst;
			spc = mem_stall * 100 / cyc;

			if (trace_clock_set_rate_enabled()) {
				char trace_name[32] = {0};
				scnprintf(trace_name, sizeof(trace_name), "lcpi%d", ccpu);
				trace_clock_set_rate(trace_name, lcpi, raw_smp_processor_id());
				scnprintf(trace_name, sizeof(trace_name), "spc%d", ccpu);
				trace_clock_set_rate(trace_name, spc, raw_smp_processor_id());
			}

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
			if (!check_pmu_limit_conditions(lcpi, spc, sg_policy)) {
#else
			if (!check_pmu_limit_conditions(spc, sg_policy)) {
#endif
				goto update_next_max_freq;
			}
		}

		next_max_freq = sg_policy->tunables->limit_frequency;
		pmu_throttle = true;

update_next_max_freq:

		freq_qos_update_request(&sg_policy->pmu_max_freq_req, next_max_freq);

		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		sg_policy->under_pmu_throttle = pmu_throttle;
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

		trace_pmu_limit(sg_policy);
		cpu = cpumask_last(policy->related_cpus) + 1;
		cpufreq_cpu_put(policy);
	}

	pmu_poll_in_progress = false;

	return;
}

static void pmu_poll_defer_work(u64 time)
{
	u64 delta_ms;

	if (!spin_trylock(&pmu_poll_enable_lock))
		return;

	if (!pmu_poll_enabled)
		goto unlock;

	if (pmu_poll_in_progress)
		goto unlock;

	delta_ms = (time - pmu_poll_last_update) / NSEC_PER_MSEC;

	if (delta_ms > pmu_poll_time_ms) {
		pmu_poll_last_update = time;
		pmu_poll_in_progress = true;
		irq_work_queue(&pmu_irq_work);
	}

unlock:
	spin_unlock(&pmu_poll_enable_lock);
}

/************************** sysfs interface ************************/

static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);


static void update_min_rate_limit_ns(struct sugov_policy *sg_policy)
{
	mutex_lock(&min_rate_lock);
	sg_policy->min_rate_limit_ns = min(sg_policy->up_rate_delay_ns,
					   sg_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;
	int cpu;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->up_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);

		for_each_cpu(cpu, sg_policy->policy->cpus)
			per_cpu(dvfs_update_delay, cpu) = rate_limit_us;
	}

	return count;
}

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf,
					size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->down_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}

	return count;
}

static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

static ssize_t down_rate_limit_scale_pow_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->down_rate_limit_scale_pow);
}

static ssize_t down_rate_limit_scale_pow_store(struct gov_attr_set *attr_set, const char *buf,
					size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int down_rate_limit_scale_pow;

	if (kstrtouint(buf, 10, &down_rate_limit_scale_pow))
		return -EINVAL;

	if (!down_rate_limit_scale_pow)
		return -EINVAL;

	tunables->down_rate_limit_scale_pow = down_rate_limit_scale_pow;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_limit_scale_pow = down_rate_limit_scale_pow;
	}

	return count;
}

static struct governor_attr down_rate_limit_scale_pow = __ATTR_RW(down_rate_limit_scale_pow);

static ssize_t response_time_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->response_time_ms);
}

static ssize_t
response_time_ms_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	int response_time_ms;

	if (kstrtoint(buf, 10, &response_time_ms))
		return -EINVAL;

	tunables->response_time_ms = response_time_ms;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		if (sg_policy->tunables == tunables) {
			if (response_time_ms <= 0)
				tunables->response_time_ms = sg_policy->freq_response_time_ms;

			sugov_update_response_time_mult(sg_policy, sugov_em_profile_changed(sg_policy));
			break;
		}
	}

	return count;
}

static struct governor_attr response_time_ms = __ATTR_RW(response_time_ms);

static ssize_t response_time_ms_nom_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		if (sg_policy->tunables == tunables)
			break;

	return sprintf(buf, "%u\n", sg_policy->freq_response_time_ms);
}

static struct governor_attr response_time_ms_nom = __ATTR_RO(response_time_ms_nom);

static ssize_t lcpi_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sysfs_emit(buf, "%u\n", tunables->lcpi_threshold);
}

static ssize_t lcpi_threshold_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	tunables->lcpi_threshold = val;

	return count;
}
static struct governor_attr lcpi_threshold = __ATTR_RW(lcpi_threshold);


static ssize_t spc_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sysfs_emit(buf, "%u\n", tunables->spc_threshold);
}

static ssize_t spc_threshold_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	tunables->spc_threshold = val;

	return count;
}
static struct governor_attr spc_threshold = __ATTR_RW(spc_threshold);


static ssize_t limit_frequency_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sysfs_emit(buf, "%u\n", tunables->limit_frequency);
}

static ssize_t limit_frequency_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	struct cpufreq_policy *policy;
	int index;
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		if (sg_policy->tunables == tunables)
			break;

	policy = sg_policy->policy;

	index = cpufreq_frequency_table_target(policy, val, CPUFREQ_RELATION_H);
	tunables->limit_frequency = policy->freq_table[index].frequency;

	return count;
}
static struct governor_attr limit_frequency = __ATTR_RW(limit_frequency);

static ssize_t pmu_limit_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sysfs_emit(buf, "%s\n", tunables->pmu_limit_enable ? "true" : "false");
}

static ssize_t pmu_limit_enable_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	tunables->pmu_limit_enable = val;

	return count;
}
static struct governor_attr pmu_limit_enable = __ATTR_RW(pmu_limit_enable);

static struct attribute *sugov_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&down_rate_limit_scale_pow.attr,
	&response_time_ms.attr,
	&response_time_ms_nom.attr,

	// For PMU Limit
	&lcpi_threshold.attr,
	&spc_threshold.attr,
	&limit_frequency.attr,
	&pmu_limit_enable.attr,
	NULL
};
ATTRIBUTE_GROUPS(sugov);

static void sugov_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = to_gov_attr_set(kobj);

	kfree(to_sugov_tunables(attr_set));
}

static struct kobj_type sugov_tunables_ktype = {
	.default_groups = sugov_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &sugov_tunables_free,
};

/********************** cpufreq governor interface *********************/

struct cpufreq_governor sched_pixel_gov;

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	sg_policy->under_pmu_throttle = false;
	sg_policy->relax_pmu_throttle = false;
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}


static void pmu_poll_irq_work(struct irq_work *irq_work)
{
	kthread_queue_work(&pmu_worker, &pmu_work);
}

int pmu_poll_init(void)
{
	int ret = 0;
	struct task_struct *thread;
	struct sched_attr attr = {0};

	attr.sched_policy = SCHED_FIFO;
	attr.sched_priority = MAX_RT_PRIO / 2;

	init_irq_work(&pmu_irq_work, pmu_poll_irq_work);
	kthread_init_work(&pmu_work, pmu_limit_work);
	kthread_init_worker(&pmu_worker);
	thread = kthread_create(kthread_worker_fn, &pmu_worker, "sched_pmu_wq");
	if (IS_ERR(thread)) {
		pr_err("failed to create pmu thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	wake_up_process(thread);

	return ret;
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_attr attr;
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	memset(&attr, 0, sizeof(struct sched_attr));
	attr.sched_policy = SCHED_FIFO;
	attr.sched_priority = MAX_RT_PRIO / 2;

	kthread_init_work(&sg_policy->work, sugov_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	thread->dl.flags = SCHED_FLAG_SUGOV;
	sg_policy->thread = thread;

	if (cpumask_first(policy->related_cpus) < pixel_cluster_start_cpu[1])
		kthread_bind_mask(thread, policy->related_cpus);
	else
		kthread_bind_mask(thread, cpu_possible_mask);

	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void sugov_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	tunables->up_rate_limit_us = cpufreq_policy_transition_delay_us(policy);
	tunables->down_rate_limit_us = cpufreq_policy_transition_delay_us(policy);
	tunables->down_rate_limit_scale_pow = 1;
	tunables->response_time_ms = sugov_calc_freq_response_ms(sg_policy);
	tunables->pmu_limit_enable = false;
	tunables->lcpi_threshold = 1000;
	tunables->spc_threshold = 100;
	tunables->limit_frequency = policy->cpuinfo.max_freq;

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	sugov_update_response_time_mult(sg_policy, true);


	freq_qos_add_request(&policy->constraints, &sg_policy->pmu_max_freq_req,
			     FREQ_QOS_MAX, policy->cpuinfo.max_freq);

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   sched_pixel_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	sugov_clear_global_tunables();

stop_kthread:
	sugov_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	cpumask_andnot(&pixel_sched_governor_mask, &pixel_sched_governor_mask, policy->cpus);

	pmu_poll_disable();
	freq_qos_remove_request(&sg_policy->pmu_max_freq_req);
	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		sugov_clear_global_tunables();

	mutex_unlock(&global_tunables_lock);

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->up_rate_delay_ns =
		sg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns =
		sg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_ns(sg_policy);
	sg_policy->down_rate_limit_scale_pow =
		sg_policy->tunables->down_rate_limit_scale_pow;
	sg_policy->last_freq_update_time	= 0;
	sg_policy->next_freq			= 0;
	sg_policy->work_in_progress		= false;
	sg_policy->limits_changed		= false;
	sg_policy->need_freq_update		= false;
	sg_policy->cached_raw_freq		= 0;
	sg_policy->prev_cached_raw_freq		= 0;

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu			= cpu;
		sg_cpu->sg_policy		= sg_policy;

		per_cpu(dvfs_update_delay, cpu) = sg_policy->tunables->up_rate_limit_us;
	}

	cpumask_or(&pixel_sched_governor_mask, &pixel_sched_governor_mask, policy->cpus);

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

#if IS_ENABLED(USE_UPDATE_SINGLE)
		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							sugov_update_shared :
							sugov_update_single);
#else
		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util, sugov_update_shared);
#endif
	}

	return 0;
}

static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	cpumask_andnot(&pixel_sched_governor_mask, &pixel_sched_governor_mask, policy->cpus);

	for_each_cpu(cpu, policy->cpus) {
		cpufreq_remove_update_util_hook(cpu);
	}

	pmu_poll_disable();

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	sg_policy->limits_changed = true;
}

struct cpufreq_governor sched_pixel_gov = {
	.name			= "sched_pixel",
	.owner			= THIS_MODULE,
	.flags			= CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init			= sugov_init,
	.exit			= sugov_exit,
	.start			= sugov_start,
	.stop			= sugov_stop,
	.limits			= sugov_limits,
};
