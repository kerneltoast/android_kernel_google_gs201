// SPDX-License-Identifier: GPL-2.0-only
/* fair.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */
#include <kernel/sched/sched.h>
#include <kernel/sched/pelt.h>

#include "sched_priv.h"
#include "sched_events.h"
#include "../systrace.h"

#if IS_ENABLED(CONFIG_PIXEL_EM)
#include "../../include/pixel_em.h"
struct pixel_em_profile **vendor_sched_pixel_em_profile;
EXPORT_SYMBOL_GPL(vendor_sched_pixel_em_profile);
#endif

extern unsigned int vendor_sched_uclamp_threshold;
extern unsigned int vendor_sched_util_post_init_scale;
extern bool vendor_sched_npi_packing;

unsigned int sched_capacity_margin[CPU_NUM] = {
			[0 ... CPU_NUM-1] = DEF_UTIL_THRESHOLD };

struct vendor_group_property vg[VG_MAX];

extern struct vendor_group_list vendor_group_list[VG_MAX];

extern inline unsigned int uclamp_none(enum uclamp_id clamp_id);

unsigned long schedutil_cpu_util_pixel_mod(int cpu, unsigned long util_cfs,
				 unsigned long max, enum schedutil_type type,
				 struct task_struct *p);
unsigned int map_scaling_freq(int cpu, unsigned int freq);

/*****************************************************************************/
/*                       Upstream Code Section                               */
/*****************************************************************************/
/*
 * This part of code is copied from Android common GKI kernel and unmodified.
 * Any change for these functions in upstream GKI would require extensive review
 * to make proper adjustment in vendor hook.
 */
#if IS_ENABLED(CONFIG_FAIR_GROUP_SCHED)
static inline struct task_struct *task_of(struct sched_entity *se)
{
	SCHED_WARN_ON(!entity_is_task(se));
	return container_of(se, struct task_struct, se);
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	return se->cfs_rq;
}
#else
static inline struct task_struct *task_of(struct sched_entity *se)
{
	return container_of(se, struct task_struct, se);
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	struct rq *rq = task_rq(p);

	return &rq->cfs;
}
#endif

#if !IS_ENABLED(CONFIG_64BIT)
static inline u64 cfs_rq_last_update_time(struct cfs_rq *cfs_rq)
{
	u64 last_update_time_copy;
	u64 last_update_time;

	do {
		last_update_time_copy = cfs_rq->load_last_update_time_copy;
		smp_rmb();
		last_update_time = cfs_rq->avg.last_update_time;
	} while (last_update_time != last_update_time_copy);

	return last_update_time;
}
#else
static inline u64 cfs_rq_last_update_time(struct cfs_rq *cfs_rq)
{
	return cfs_rq->avg.last_update_time;
}
#endif

#if IS_ENABLED(CONFIG_UCLAMP_TASK)
static inline unsigned long uclamp_task_util(struct task_struct *p)
{
	return clamp(task_util_est(p),
		     uclamp_eff_value(p, UCLAMP_MIN),
		     uclamp_eff_value(p, UCLAMP_MAX));
}
#else
static inline unsigned long uclamp_task_util(struct task_struct *p)
{
	return task_util_est(p);
}
#endif

static inline unsigned long capacity_of(int cpu)
{
	return cpu_rq(cpu)->cpu_capacity;
}

/* Runqueue only has SCHED_IDLE tasks enqueued */
static int sched_idle_rq(struct rq *rq)
{
	return unlikely(rq->nr_running == rq->cfs.idle_h_nr_running &&
			rq->nr_running);
}

#ifdef CONFIG_SMP
int sched_cpu_idle(int cpu)
{
	return sched_idle_rq(cpu_rq(cpu));
}
#endif

static inline bool within_margin(int value, int margin)
{
	return ((unsigned int)(value + margin - 1) < (2 * margin - 1));
}

static inline void update_load_add(struct load_weight *lw, unsigned long inc)
{
	lw->weight += inc;
	lw->inv_weight = 0;
}

#define WMULT_CONST	(~0U)
#define WMULT_SHIFT	32

static void __update_inv_weight(struct load_weight *lw)
{
	unsigned long w;

	if (likely(lw->inv_weight))
		return;

	w = scale_load_down(lw->weight);

	if (BITS_PER_LONG > 32 && unlikely(w >= WMULT_CONST))
		lw->inv_weight = 1;
	else if (unlikely(!w))
		lw->inv_weight = WMULT_CONST;
	else
		lw->inv_weight = WMULT_CONST / w;
}

static u64 __calc_delta(u64 delta_exec, unsigned long weight, struct load_weight *lw)
{
	u64 fact = scale_load_down(weight);
	int shift = WMULT_SHIFT;

	__update_inv_weight(lw);

	if (unlikely(fact >> 32)) {
		while (fact >> 32) {
			fact >>= 1;
			shift--;
		}
	}

	fact = mul_u32_u32(fact, lw->inv_weight);

	while (fact >> 32) {
		fact >>= 1;
		shift--;
	}

	return mul_u64_u32_shr(delta_exec, fact, shift);
}

static u64 __sched_period(unsigned long nr_running);

#define for_each_sched_entity(se) \
		for (; se; se = se->parent)

static u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	u64 slice = __sched_period(cfs_rq->nr_running + !se->on_rq);

	for_each_sched_entity(se) {
		struct load_weight *load;
		struct load_weight lw;

		cfs_rq = cfs_rq_of(se);
		load = &cfs_rq->load;

		if (unlikely(!se->on_rq)) {
			lw = cfs_rq->load;

			update_load_add(&lw, se->load.weight);
			load = &lw;
		}
		slice = __calc_delta(slice, se->load.weight, load);
	}
	return slice;
}

static void set_next_buddy(struct sched_entity *se)
{
	if (entity_is_task(se) && unlikely(task_has_idle_policy(task_of(se))))
		return;

	for_each_sched_entity(se) {
		if (SCHED_WARN_ON(!se->on_rq))
			return;
		cfs_rq_of(se)->next = se;
	}
}

static inline struct task_group *css_tg(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct task_group, css) : NULL;
}

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */
bool get_prefer_high_cap(struct task_struct *p)
{
	return vg[get_vendor_group(p)].prefer_high_cap;
}

static inline bool get_task_spreading(struct task_struct *p)
{
	return vg[get_vendor_group(p)].task_spreading;
}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
static inline unsigned int get_task_group_throttle(struct task_struct *p)
{
	return vg[get_vendor_group(p)].group_throttle;
}

static inline unsigned int get_group_throttle(struct task_group *tg)
{
	return vg[get_vendor_task_group_struct(tg)->group].group_throttle;
}
#endif

void init_vendor_group_data(void)
{
	int i;

	for (i = 0; i < VG_MAX; i++) {
		INIT_LIST_HEAD(&vendor_group_list[i].list);
		raw_spin_lock_init(&vendor_group_list[i].lock);
		vendor_group_list[i].cur_iterator = NULL;
	}
}

#if defined(CONFIG_UCLAMP_TASK) && defined(CONFIG_FAIR_GROUP_SCHED)
static inline unsigned long cpu_util_cfs_group_mod_no_est(struct rq *rq)
{
	struct cfs_rq *cfs_rq, *pos;
	unsigned long util = 0, unclamped_util = 0;
	struct task_group *tg;
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	unsigned long scale_cpu = arch_scale_cpu_capacity(rq->cpu);
#endif

	// cpu_util_cfs = root_util - subgroup_util_sum + throttled_subgroup_util_sum
	for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos) {
		if (&rq->cfs != cfs_rq) {
			tg = cfs_rq->tg;
			unclamped_util += cfs_rq->avg.util_avg;
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
			util += min_t(unsigned long, READ_ONCE(cfs_rq->avg.util_avg),
				cap_scale(get_group_throttle(tg), scale_cpu));
#else
			util += READ_ONCE(cfs_rq->avg.util_avg);
#endif
		}
	}

	util += max_t(int, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util, 0);

	return util;
}

unsigned long cpu_util_cfs_group_mod(struct rq *rq)
{
	unsigned long util = cpu_util_cfs_group_mod_no_est(rq);

	if (sched_feat(UTIL_EST)) {
		// TODO: right now the limit of util_est is per task
		// consider to make it per group.
		util = max_t(unsigned long, util,
			     READ_ONCE(rq->cfs.avg.util_est.enqueued));
	}

	return util;
}
#else
#define cpu_util_cfs_group_mod cpu_util_cfs
#endif

unsigned long cpu_util(int cpu)
{
       struct rq *rq = cpu_rq(cpu);

       unsigned long util = cpu_util_cfs_group_mod(rq);

       return min_t(unsigned long, util, capacity_of(cpu));
}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
/* Similar to cpu_util_without but only count the task's group util contribution */
static unsigned long group_util_without(int cpu, struct task_struct *p, unsigned long max)
{
	unsigned long util = READ_ONCE(task_group(p)->cfs_rq[cpu]->avg.util_avg);

	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		goto out;

	lsub_positive(&util, task_util(p));

out:
	return min_t(unsigned long, util, max);

}
#endif

static unsigned long cpu_util_without_raw(int cpu, struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	unsigned long util;

	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		return cpu_util_cfs_group_mod(cpu_rq(cpu));

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = cpu_util_cfs_group_mod_no_est(cpu_rq(cpu));

	/* Discount task's util from CPU's util */
	lsub_positive(&util, task_util(p));

	/*
	 * Covered cases:
	 *
	 * a) if *p is the only task sleeping on this CPU, then:
	 *      cpu_util (== task_util) > util_est (== 0)
	 *    and thus we return:
	 *      cpu_util_without = (cpu_util - task_util) = 0
	 *
	 * b) if other tasks are SLEEPING on this CPU, which is now exiting
	 *    IDLE, then:
	 *      cpu_util >= task_util
	 *      cpu_util > util_est (== 0)
	 *    and thus we discount *p's blocked utilization to return:
	 *      cpu_util_without = (cpu_util - task_util) >= 0
	 *
	 * c) if other tasks are RUNNABLE on that CPU and
	 *      util_est > cpu_util
	 *    then we use util_est since it returns a more restrictive
	 *    estimation of the spare capacity on that CPU, by just
	 *    considering the expected utilization of tasks already
	 *    runnable on that CPU.
	 *
	 * Cases a) and b) are covered by the above code, while case c) is
	 * covered by the following code when estimated utilization is
	 * enabled.
	 */
	if (sched_feat(UTIL_EST)) {
		unsigned int estimated =
			READ_ONCE(cfs_rq->avg.util_est.enqueued);

		/*
		 * Despite the following checks we still have a small window
		 * for a possible race, when an execl's select_task_rq_fair()
		 * races with LB's detach_task():
		 *
		 *   detach_task()
		 *     p->on_rq = TASK_ON_RQ_MIGRATING;
		 *     ---------------------------------- A
		 *     deactivate_task()                   \
		 *       dequeue_task()                     + RaceTime
		 *         util_est_dequeue()              /
		 *     ---------------------------------- B
		 *
		 * The additional check on "current == p" it's required to
		 * properly fix the execl regression and it helps in further
		 * reducing the chances for the above race.
		 */
		if (unlikely(task_on_rq_queued(p) || current == p))
			lsub_positive(&estimated, _task_util_est(p));

		util = max_t(unsigned long, util, estimated);
	}

	return util;
}

struct vendor_group_property *get_vendor_group_property(enum vendor_group group)
{
	SCHED_WARN_ON(group < VG_SYSTEM || group >= VG_MAX);

	return &(vg[group]);
}

static bool task_fits_capacity(struct task_struct *p, int cpu,  bool sync_boost)
{
	unsigned long task_util;

	if (cpu >= MAX_CAPACITY_CPU)
		return true;

	if ((get_prefer_high_cap(p) || sync_boost) && cpu < MID_CAPACITY_CPU)
		return false;

	task_util = get_task_spreading(p) ? task_util_est(p) : uclamp_task_util(p);

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	/* clamp task utilization against its per-cpu group limit */
	task_util = min_t(unsigned long, task_util, cap_scale(get_task_group_throttle(p),
							arch_scale_cpu_capacity(cpu)));
#endif

	return capacity_of(cpu) * SCHED_CAPACITY_SCALE > task_util * sched_capacity_margin[cpu];
}

static inline bool cpu_is_in_target_set(struct task_struct *p, int cpu)
{
	int first_cpu, next_usable_cpu;

	if (get_prefer_high_cap(p)) {
		first_cpu = HIGH_CAPACITY_CPU;
	} else {
		first_cpu = MIN_CAPACITY_CPU;
	}

	next_usable_cpu = cpumask_next(first_cpu - 1, p->cpus_ptr);
	return cpu >= next_usable_cpu || next_usable_cpu >= nr_cpu_ids;
}

/**
 * cpu_is_idle - is a given CPU idle for enqueuing work.
 * @cpu: the CPU in question.
 *
 * Return: 1 if the CPU is currently idle. 0 otherwise.
 */
int cpu_is_idle(int cpu)
{
	if (available_idle_cpu(cpu) || sched_cpu_idle(cpu))
		return 1;

	return 0;
}

static unsigned long cpu_util_next(int cpu, struct task_struct *p, int dst_cpu)
{
	struct cfs_rq *cfs_rq, *pos;
	unsigned long util = 0, unclamped_util = 0;
	unsigned long util_est;
	struct task_group *tg;
	long delta = 0;
	struct rq *rq = cpu_rq(cpu);
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	unsigned long scale_cpu = arch_scale_cpu_capacity(cpu);
#endif

	if (task_cpu(p) == cpu && dst_cpu != cpu)
		delta = -task_util(p);
	else if (task_cpu(p) != cpu && dst_cpu == cpu)
		delta = task_util(p);

	// For leaf groups
	for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos) {
		if (&rq->cfs != cfs_rq) {
			unsigned long group_util = 0;
			tg = cfs_rq->tg;

			if (p->se.cfs_rq->tg == tg)
				group_util = max_t(int, READ_ONCE(cfs_rq->avg.util_avg) + delta, 0);
			else
				group_util = READ_ONCE(cfs_rq->avg.util_avg);

			unclamped_util += READ_ONCE(cfs_rq->avg.util_avg);
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
			util += min_t(unsigned long, group_util,
				cap_scale(get_group_throttle(tg), scale_cpu));
#else
			util += group_util;
#endif
		}
	}

	// For root group
	if (p->se.cfs_rq->tg == rq->cfs.tg)
		util = max_t(long, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util + util + delta,
				   0);
	else
		util = max_t(long, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util + util, 0);

	if (sched_feat(UTIL_EST)) {
		util_est = READ_ONCE(cfs_rq->avg.util_est.enqueued);

		if (dst_cpu == cpu)
			util_est += _task_util_est(p);

		util = max_t(unsigned long, util, util_est);
	}

	return min(util, capacity_of(cpu));
}

static inline unsigned long em_cpu_energy_pixel_mod(struct em_perf_domain *pd,
				unsigned long max_util, unsigned long sum_util)
{
	unsigned long freq, scale_cpu;
	struct em_perf_state *ps;
	int i, cpu;

	if (!sum_util)
		return 0;

	cpu = cpumask_first(to_cpumask(pd->cpus));

#if IS_ENABLED(CONFIG_PIXEL_EM)
	{
		struct pixel_em_profile **profile_ptr_snapshot;
		profile_ptr_snapshot = READ_ONCE(vendor_sched_pixel_em_profile);
		if (profile_ptr_snapshot) {
			struct pixel_em_profile *profile = READ_ONCE(*profile_ptr_snapshot);
			if (profile) {
				struct pixel_em_cluster *cluster = profile->cpu_to_cluster[cpu];
				struct pixel_em_opp *max_opp;
				struct pixel_em_opp *opp;

				max_opp = &cluster->opps[cluster->num_opps - 1];

				freq = map_util_freq_pixel_mod(max_util,
							       max_opp->freq,
							       max_opp->capacity,
							       cpu);
				freq = map_scaling_freq(cpu, freq);

				for (i = 0; i < cluster->num_opps; i++) {
					opp = &cluster->opps[i];
					if (opp->freq >= freq)
						break;
				}

				return opp->cost * sum_util / max_opp->capacity;
			}
		}
	}
#endif

	scale_cpu = arch_scale_cpu_capacity(cpu);
	ps = &pd->table[pd->nr_perf_states - 1];
	freq = map_util_freq_pixel_mod(max_util, ps->frequency, scale_cpu, cpu);
	freq = map_scaling_freq(cpu, freq);

	for (i = 0; i < pd->nr_perf_states; i++) {
		ps = &pd->table[i];
		if (ps->frequency >= freq)
			break;
	}

	return ps->cost * sum_util / scale_cpu;
}

static long
compute_energy(struct task_struct *p, int dst_cpu, struct perf_domain *pd)
{
	unsigned long max_util, util_cfs, cpu_util, cpu_cap;
	unsigned long sum_util, energy = 0;
	struct task_struct *tsk;
	int cpu;

	for (; pd; pd = pd->next) {
		struct cpumask *pd_mask = perf_domain_span(pd);

		/*
		 * The energy model mandates all the CPUs of a performance
		 * domain have the same capacity.
		 */
		cpu_cap = arch_scale_cpu_capacity(cpumask_first(pd_mask));
		max_util = sum_util = 0;

		/*
		 * The capacity state of CPUs of the current rd can be driven by
		 * CPUs of another rd if they belong to the same performance
		 * domain. So, account for the utilization of these CPUs too
		 * by masking pd with cpu_online_mask instead of the rd span.
		 *
		 * If an entire performance domain is outside of the current rd,
		 * it will not appear in its pd list and will not be accounted
		 * by compute_energy().
		 */
		for_each_cpu_and(cpu, pd_mask, cpu_online_mask) {
			util_cfs = cpu_util_next(cpu, p, dst_cpu);

			/*
			 * Busy time computation: utilization clamping is not
			 * required since the ratio (sum_util / cpu_capacity)
			 * is already enough to scale the EM reported power
			 * consumption at the (eventually clamped) cpu_capacity.
			 */
			sum_util += schedutil_cpu_util_pixel_mod(cpu, util_cfs, cpu_cap,
						       ENERGY_UTIL, NULL);

			/*
			 * Performance domain frequency: utilization clamping
			 * must be considered since it affects the selection
			 * of the performance domain frequency.
			 * NOTE: in case RT tasks are running, by default the
			 * FREQUENCY_UTIL's utilization can be max OPP.
			 */
			tsk = cpu == dst_cpu ? p : NULL;
			cpu_util = schedutil_cpu_util_pixel_mod(cpu, util_cfs, cpu_cap,
						      FREQUENCY_UTIL, tsk);
			max_util = max(max_util, cpu_util);
		}
		energy += em_cpu_energy_pixel_mod(pd->em_pd, max_util, sum_util);
	}

	trace_sched_compute_energy(p, dst_cpu, energy);
	return energy;
}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
/* If a task_group is over its group limit on a particular CPU with margin considered */
static inline bool group_overutilized(int cpu, struct task_group *tg)
{

	unsigned long group_capacity = cap_scale(get_group_throttle(tg),
					arch_scale_cpu_capacity(cpu));
	unsigned long group_util = READ_ONCE(tg->cfs_rq[cpu]->avg.util_avg);
	return cpu_overutilized(group_util, group_capacity, cpu);
}
#endif

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the original
 * functions.
 */

/*
 * To avoid export more and more sysctl from GKI kernel,
 * use following setting directly dumped from running Android
 * and also assume the setting is not changed in runtime.
 *
 * sysctl_sched
 * .sysctl_sched_latency                    : 10.000000
 * .sysctl_sched_min_granularity            : 3.000000
 * .sysctl_sched_wakeup_granularity         : 2.000000
 * .sysctl_sched_child_runs_first           : 0
 * .sysctl_sched_features                   : 33477435
 * .sysctl_sched_tunable_scaling            : 0 (none)
 */
#define sysctl_sched_min_granularity 3000000ULL

static u64 __sched_period(unsigned long nr_running)
{
	unsigned int sched_nr_latency = DIV_ROUND_UP(sysctl_sched_latency,
					sysctl_sched_min_granularity);

	if (unlikely(nr_running > sched_nr_latency))
		return nr_running * sysctl_sched_min_granularity;
	else
		return sysctl_sched_latency;
}

static bool cpu_is_better(int cpu, int best_cpu,
			  const unsigned int exit_lat[CPU_NUM],
			  unsigned long util, unsigned long l_util)
{
	/*
	 * Find the CPU with the lowest raw utilization ratio. A non-idle CPU or
	 * CPU with lower exit latency is preferred when utilization is equal.
	 */
	return util < l_util || (util == l_util &&
				 exit_lat[cpu] < exit_lat[best_cpu]);
}

static unsigned long cpu_util_ratio(struct task_struct *p,
				    const unsigned long cap[CPU_NUM],
				    const unsigned int exit_lat[CPU_NUM],
				    int cpu, int prev_cpu, int *best_cpu,
				    unsigned long *l_util)
{
	unsigned long util;

	/* Exclude @p from the CPU's utilization if this is the previous CPU */
	if (cpu == prev_cpu)
		util = cpu_util_without_raw(cpu, p);
	else
		util = cpu_util_cfs_group_mod(cpu_rq(cpu));
	util = util * SCHED_CAPACITY_SCALE / cap[cpu];
	if (cpu_is_better(cpu, *best_cpu, exit_lat, util, *l_util)) {
		*l_util = util;
		*best_cpu = cpu;
	}

	return util;
}

static int find_energy_efficient_cpu(struct task_struct *p, int prev_cpu,
				     const cpumask_t *valid_mask)
{
	unsigned long cap[CPU_NUM], cpu_util[CPU_NUM], energy[CPU_NUM] = {};
	unsigned long l_util = ULONG_MAX, p_util;
	cpumask_t allowed, candidates = {};
	struct cpuidle_state *idle_state;
	unsigned int exit_lat[CPU_NUM];
	struct perf_domain *pd;
	int i, best_cpu;
	struct rq *rq;

	/*
	 * If there aren't any valid CPUs which are active, then just return the
	 * first valid CPU since it's possible for certain types of tasks to run
	 * on !active CPUs.
	 */
	if (unlikely(!cpumask_and(&allowed, valid_mask, cpu_active_mask)))
		return cpumask_first(valid_mask);

	/* Compute the utilization for this task */
	p_util = get_task_spreading(p) ? task_util_est(p) : uclamp_task_util(p);

	/*
	 * Find the best-fitting CPU with the lowest total raw utilization
	 * ratio; i.e., the least relatively-loaded CPU. Note that although
	 * idle_get_state() requires an RCU read lock, an RCU read lock isn't
	 * needed because we're not preemptible and RCU-sched is unified with
	 * normal RCU. Therefore, non-preemptible contexts are implicitly
	 * RCU-safe.
	 *
	 * Iteration through @allowed is intended to go from the lowest-capacity
	 * cluster to the highest-capacity cluster in order to pack tasks onto
	 * lower-capacity clusters. If all cores in a higher-capacity cluster
	 * can idle, then it may be possible to enter a cluster idle state where
	 * the whole cluster goes into a deeper C-state, saving more power. This
	 * is generally moot for the lowest-capacity cluster though, since it
	 * typically contains the boot CPU and handles housekeeping, plus
	 * generally has the most cores, so it's less likely for it to enter
	 * cluster idle.
	 *
	 * Packing tasks onto lower-capacity clusters also improves overall
	 * single-threaded performance by reducing the load on higher-capacity
	 * CPUs, making them more available to heavy tasks.
	 */
	for_each_cpu(i, &allowed) {
		/*
		 * Get the current capacity of this CPU adjusted for thermal
		 * pressure as well as IRQ and RT-task time.
		 */
		cap[i] = capacity_of(i);

		/* Get the idle exit latency for this CPU if it's idle */
		rq = cpu_rq(i);
		idle_state = idle_get_state(rq);
		exit_lat[i] = idle_state ? idle_state->exit_latency : 0;

		/* Calculate the raw utilization ratio if this CPU fits */
		if (!cpu_overutilized(p_util, cap[i], i)) {
			cpu_util[i] = cpu_util_ratio(p, cap, exit_lat, i,
						     prev_cpu, &best_cpu,
						     &l_util);
			__cpumask_set_cpu(i, &candidates);
		}
	}

	/* If no CPU fits, then place the task on the least utilized CPU */
	if (l_util == ULONG_MAX) {
		for_each_cpu(i, &allowed)
			cpu_util_ratio(p, cap, exit_lat, i, prev_cpu, &best_cpu,
				       &l_util);
		goto check_prev;
	}

	/* Stop now if only one CPU fits */
	if (cpumask_weight(&candidates) == 1)
		return best_cpu;

	/*
	 * Quickly filter out CPUs with significantly higher utilization by
	 * comparing floor(sqrt(util)) for each candidate. This helps avoid
	 * CPUs which are quadratically more loaded than the least utilized CPU
	 * found earlier, and eliminates their heavy energy computations. These
	 * CPUs are a bad choice from a performance standpoint, so discard them.
	 */
	l_util = int_sqrt(l_util);
	__cpumask_clear_cpu(best_cpu, &candidates);
	for_each_cpu(i, &candidates) {
		if (int_sqrt(cpu_util[i]) > l_util)
			__cpumask_clear_cpu(i, &candidates);
	}

	/* Stop now if all other CPUs are obviously a bad choice */
	if (cpumask_empty(&candidates))
		return best_cpu;

	pd = rcu_dereference(rq->rd->pd);
	if (unlikely(!pd))
		goto check_prev;

	/*
	 * Search for an energy efficient alternative to @best_cpu. This
	 * intentionally iterates over the candidates in ascending order from
	 * the lowest-capacity cluster to the highest-capacity cluster; that
	 * way, CPUs from lower-capacity clusters are preferred when there are
	 * multiple CPU candidates available that have similar energy and
	 * performance attributes.
	 */
	energy[best_cpu] = int_sqrt(compute_energy(p, best_cpu, pd));
	for_each_cpu(i, &candidates) {
		/*
		 * Compare floor(sqrt(energy)) to ignore small differences in
		 * energy and prefer performance at the expense of slightly
		 * higher predicted energy. This also helps avoid bouncing tasks
		 * between different CPUs over very small energy differences,
		 * which hurts performance and can worsen energy.
		 */
		energy[i] = int_sqrt(compute_energy(p, i, pd));
		if (energy[i] > energy[best_cpu])
			continue;

		/*
		 * Use this CPU if it has either lower energy or equal energy
		 * with better performance.
		 */
		if (energy[i] < energy[best_cpu] ||
		    cpu_is_better(i, best_cpu, exit_lat, cpu_util[i],
				  cpu_util[best_cpu]))
			best_cpu = i;
	}

check_prev:
	/*
	 * If utilization, idle exit latency, and energy are equal between the
	 * previous CPU and the best CPU, prefer the previous CPU if it's part
	 * of the same cluster as the best CPU or a lower-capacity cluster. The
	 * previous CPU isn't preferred if it's part of a higher-capacity
	 * cluster in order to pack tasks into lower-capacity clusters.
	 *
	 * This check is at the end because there's no way to know which cluster
	 * the best CPU will belong to until the final best CPU is found.
	 */
	if (best_cpu != prev_cpu && cpumask_test_cpu(prev_cpu, &allowed) &&
	    cpu_util[prev_cpu] == cpu_util[best_cpu] &&
	    exit_lat[prev_cpu] == exit_lat[best_cpu] &&
	    energy[prev_cpu] == energy[best_cpu] &&
	    capacity_orig_of(prev_cpu) <= capacity_orig_of(best_cpu))
		best_cpu = prev_cpu;

	return best_cpu;
}

#if IS_ENABLED(CONFIG_PIXEL_EM)
void vh_arch_set_freq_scale_pixel_mod(void *data, const struct cpumask *cpus,
				      unsigned long freq,
				      unsigned long max, unsigned long *scale)
{
	int i;
	struct pixel_em_profile **profile_ptr_snapshot;
	profile_ptr_snapshot = READ_ONCE(vendor_sched_pixel_em_profile);
	if (profile_ptr_snapshot) {
		struct pixel_em_profile *profile = READ_ONCE(*profile_ptr_snapshot);
		if (profile) {
			struct pixel_em_cluster *cluster;
			struct pixel_em_opp *max_opp;
			struct pixel_em_opp *opp;

			cluster = profile->cpu_to_cluster[cpumask_first(cpus)];
			max_opp = &cluster->opps[cluster->num_opps - 1];

			for (i = 0; i < cluster->num_opps; i++) {
				opp = &cluster->opps[i];
				if (opp->freq >= freq)
					break;
			}

			*scale = (opp->capacity << SCHED_CAPACITY_SHIFT) /
				  max_opp->capacity;
		}
	}
}
EXPORT_SYMBOL_GPL(vh_arch_set_freq_scale_pixel_mod);
#endif

void rvh_set_iowait_pixel_mod(void *data, struct task_struct *p, int *should_iowait_boost)
{
	*should_iowait_boost = p->in_iowait && uclamp_boosted(p);
}

void rvh_cpu_overutilized_pixel_mod(void *data, int cpu, int *overutilized)
{
	*overutilized = cpu_overutilized(cpu_util(cpu), capacity_of(cpu), cpu);
}

unsigned long map_util_freq_pixel_mod(unsigned long util, unsigned long freq,
				      unsigned long cap, int cpu)
{
	return (freq * sched_capacity_margin[cpu] >> SCHED_CAPACITY_SHIFT) * util / cap;
}

static inline bool check_uclamp_threshold(struct task_struct *p, enum uclamp_id clamp_id)
{
	if (clamp_id == UCLAMP_MIN && !rt_task(p) &&
	    !get_vendor_task_struct(p)->uclamp_fork_reset &&
	    task_util_est(p) < vendor_sched_uclamp_threshold) {
		return true;
	}
	return false;
}

static inline struct uclamp_se
uclamp_tg_restrict_pixel_mod(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct uclamp_se uc_req = p->uclamp_req[clamp_id];
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_binder_task_struct *vbinder = get_vendor_binder_task_struct(p);

#ifdef CONFIG_UCLAMP_TASK_GROUP
	unsigned int tg_min, tg_max, vnd_min, vnd_max, value;

	// Task group restriction
	tg_min = task_group(p)->uclamp[UCLAMP_MIN].value;
	tg_max = task_group(p)->uclamp[UCLAMP_MAX].value;
	// Vendor group restriction
	vnd_min = vg[vp->group].uc_req[UCLAMP_MIN].value;
	vnd_max = vg[vp->group].uc_req[UCLAMP_MAX].value;

	value = uc_req.value;
	value = clamp(value, max(tg_min, vnd_min),  min(tg_max, vnd_max));

	// Inherited uclamp restriction
	if (vbinder->active)
		value = clamp(value, vbinder->uclamp[UCLAMP_MIN], vbinder->uclamp[UCLAMP_MAX]);

	// For uclamp min, if task has a valid per-task setting that is lower than or equal to its
	// group value, increase the final uclamp value by 1. This would have effect only on
	// importance metrics which is used in task placement, and little effect on cpufreq.
	if (clamp_id == UCLAMP_MIN && uc_req.value <= max(tg_min, vnd_min) && uc_req.user_defined
		&& value < SCHED_CAPACITY_SCALE)
		value = value + 1;

	// For low prio unthrottled task, reduce its uclamp.max by 1 which
	// would affect task importance in cpu_rq thus affect task placement.
	// It should have no effect in cpufreq.
	if (clamp_id == UCLAMP_MAX && p->prio > DEFAULT_PRIO)
		value = min_t(unsigned int, UCLAMP_BUCKET_DELTA * (UCLAMP_BUCKETS - 1) - 1, value);

	uc_req.value = value;
	uc_req.bucket_id = get_bucket_id(value);
#endif

	return uc_req;
}

void rvh_uclamp_eff_get_pixel_mod(void *data, struct task_struct *p, enum uclamp_id clamp_id,
				  struct uclamp_se *uclamp_max, struct uclamp_se *uclamp_eff,
				  int *ret)
{
	struct uclamp_se uc_req;

	*ret = 1;

	/* Apply threshold first. */
	if (check_uclamp_threshold(p, clamp_id)) {
		uclamp_eff->value = 0;
		uclamp_eff->bucket_id = 0;
		return;
	}

	uc_req = uclamp_tg_restrict_pixel_mod(p, clamp_id);

	/* System default restrictions always apply */
	if (unlikely(uc_req.value > uclamp_max->value)) {
		*uclamp_eff = *uclamp_max;
		return;
	}

	*uclamp_eff = uc_req;
	return;
}

void initialize_vendor_group_property(void)
{
	int i;
	unsigned int min_val = 0;
	unsigned int max_val = SCHED_CAPACITY_SCALE;

	for (i = 0; i < VG_MAX; i++) {
		vg[i].prefer_idle = false;
		vg[i].prefer_high_cap = false;
		vg[i].task_spreading = false;
		vg[i].group_throttle = max_val;
		vg[i].uc_req[UCLAMP_MIN].value = min_val;
		vg[i].uc_req[UCLAMP_MIN].bucket_id = get_bucket_id(min_val);
		vg[i].uc_req[UCLAMP_MIN].user_defined = false;
		vg[i].uc_req[UCLAMP_MAX].value = max_val;
		vg[i].uc_req[UCLAMP_MAX].bucket_id = get_bucket_id(max_val);
		vg[i].uc_req[UCLAMP_MAX].user_defined = false;
	}
}

void rvh_check_preempt_wakeup_pixel_mod(void *data, struct rq *rq, struct task_struct *p,
			bool *preempt, bool *nopreempt, int wake_flags, struct sched_entity *se,
			struct sched_entity *pse, int next_buddy_marked, unsigned int granularity)
{
	unsigned long ideal_runtime, delta_exec;

	if (entity_is_task(pse) || entity_is_task(se))
		return;

	ideal_runtime = sched_slice(cfs_rq_of(se), se);
	delta_exec = se->sum_exec_runtime - se->prev_sum_exec_runtime;
	/*
	 * If the current group has run enough time for its slice and the new
	 * group has bigger weight, go ahead and preempt.
	 */
	if (ideal_runtime <= delta_exec && se->load.weight < pse->load.weight) {
		if (!next_buddy_marked)
			set_next_buddy(pse);

		*preempt = true;
	}

}

void rvh_util_est_update_pixel_mod(void *data, struct cfs_rq *cfs_rq, struct task_struct *p,
				    bool task_sleep, int *ret)
{
	long last_ewma_diff;
	struct util_est ue;
	int cpu;
	unsigned long scale_cpu;

	*ret = 1;

	if (!sched_feat(UTIL_EST))
		return;

	/*
	 * Skip update of task's estimated utilization when the task has not
	 * yet completed an activation, e.g. being migrated.
	 */
	if (!task_sleep)
		return;

	/*
	 * If the PELT values haven't changed since enqueue time,
	 * skip the util_est update.
	 */
	ue = p->se.avg.util_est;
	if (ue.enqueued & UTIL_AVG_UNCHANGED)
		return;

	/*
	 * Reset EWMA on utilization increases, the moving average is used only
	 * to smooth utilization decreases.
	 */
	ue.enqueued = task_util(p);

	cpu = cpu_of(rq_of(cfs_rq));
	scale_cpu = arch_scale_cpu_capacity(cpu);
	// TODO: make util_est to sub cfs-rq and aggregate.
#ifdef CONFIG_UCLAMP_TASK
	// Currently util_est is done only in the root group
	// Current solution apply the clamp in the per-task level for simplicity.
	// However it may
	// 1) over grow by the group limit
	// 2) out of sync when task migrated between cgroups (cfs_rq)
	ue.enqueued = min((unsigned long)ue.enqueued, uclamp_eff_value(p, UCLAMP_MAX));
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	ue.enqueued = min_t(unsigned long, ue.enqueued,
			cap_scale(get_group_throttle(task_group(p)), scale_cpu));
#endif
#endif

	if (sched_feat(UTIL_EST_FASTUP)) {
		if (ue.ewma < ue.enqueued) {
			ue.ewma = ue.enqueued;
			goto done;
		}
	}

	/*
	 * Skip update of task's estimated utilization when its EWMA is
	 * already ~1% close to its last activation value.
	 */
	last_ewma_diff = ue.enqueued - ue.ewma;
	if (within_margin(last_ewma_diff, (SCHED_CAPACITY_SCALE / 100)))
		return;

	/*
	 * To avoid overestimation of actual task utilization, skip updates if
	 * we cannot grant there is idle time in this CPU.
	 */

	if (task_util(p) > capacity_orig_of(cpu))
		return;

	/*
	 * Update Task's estimated utilization
	 *
	 * When *p completes an activation we can consolidate another sample
	 * of the task size. This is done by storing the current PELT value
	 * as ue.enqueued and by using this value to update the Exponential
	 * Weighted Moving Average (EWMA):
	 *
	 *  ewma(t) = w *  task_util(p) + (1-w) * ewma(t-1)
	 *          = w *  task_util(p) +         ewma(t-1)  - w * ewma(t-1)
	 *          = w * (task_util(p) -         ewma(t-1)) +     ewma(t-1)
	 *          = w * (      last_ewma_diff            ) +     ewma(t-1)
	 *          = w * (last_ewma_diff  +  ewma(t-1) / w)
	 *
	 * Where 'w' is the weight of new samples, which is configured to be
	 * 0.25, thus making w=1/4 ( >>= UTIL_EST_WEIGHT_SHIFT)
	 */
	ue.ewma <<= UTIL_EST_WEIGHT_SHIFT;
	ue.ewma  += last_ewma_diff;
	ue.ewma >>= UTIL_EST_WEIGHT_SHIFT;
#ifdef CONFIG_UCLAMP_TASK
	ue.ewma = min((unsigned long)ue.ewma, uclamp_eff_value(p, UCLAMP_MAX));
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	ue.ewma = min_t(unsigned long, ue.ewma,
			cap_scale(get_group_throttle(task_group(p)), scale_cpu));
#endif
#endif
done:
	ue.enqueued |= UTIL_AVG_UNCHANGED;
	WRITE_ONCE(p->se.avg.util_est, ue);

	trace_sched_util_est_se_tp(&p->se);
}

void rvh_post_init_entity_util_avg_pixel_mod(void *data, struct sched_entity *se)
{
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	struct sched_avg *sa = &se->avg;
	long cpu_scale = arch_scale_cpu_capacity(cpu_of(rq_of(cfs_rq)));

	if (cfs_rq->avg.util_avg == 0) {
		sa->util_avg = vendor_sched_util_post_init_scale * cpu_scale / SCHED_CAPACITY_SCALE;
		sa->runnable_avg = sa->util_avg;
	}
}

void rvh_cpu_cgroup_online_pixel_mod(void *data, struct cgroup_subsys_state *css)
{
	struct vendor_task_group_struct *vtg;
	const char *name = css_tg(css)->css.cgroup->kn->name;

	vtg = get_vendor_task_group_struct(css_tg(css));

	if (strcmp(name, "system") == 0) {
		vtg->group = VG_SYSTEM;
	} else if (strcmp(name, "top-app") == 0) {
		vtg->group = VG_TOPAPP;
	} else if (strcmp(name, "foreground") == 0) {
		vtg->group = VG_FOREGROUND;
	} else if (strcmp(name, "camera-daemon") == 0) {
		vtg->group = VG_CAMERA;
	} else if (strcmp(name, "background") == 0) {
		vtg->group = VG_BACKGROUND;
	} else if (strcmp(name, "system-background") == 0) {
		vtg->group = VG_SYSTEM_BACKGROUND;
	} else if (strcmp(name, "nnapi-hal") == 0) {
		vtg->group = VG_NNAPI_HAL;
	} else if (strcmp(name, "rt") == 0) {
		vtg->group = VG_RT;
	} else if (strcmp(name, "dex2oat") == 0) {
		vtg->group = VG_DEX2OAT;
	} else {
		vtg->group = VG_SYSTEM;
	}
}

void vh_dup_task_struct_pixel_mod(void *data, struct task_struct *tsk, struct task_struct *orig)
{
	struct vendor_task_struct *v_tsk, *v_orig;
	struct vendor_binder_task_struct *vbinder;

	v_tsk = get_vendor_task_struct(tsk);
	vbinder = get_vendor_binder_task_struct(tsk);
	if (likely(orig)) {
		v_orig = get_vendor_task_struct(orig);
		v_tsk->group = v_orig->group;
	} else {
		v_tsk->group = VG_SYSTEM;
	}
	v_tsk->prefer_idle = false;
	INIT_LIST_HEAD(&v_tsk->node);
	raw_spin_lock_init(&v_tsk->lock);
	v_tsk->queued_to_list = false;

	vbinder->uclamp[UCLAMP_MIN] = uclamp_none(UCLAMP_MIN);
	vbinder->uclamp[UCLAMP_MAX] = uclamp_none(UCLAMP_MAX);
	vbinder->prefer_idle = false;
	vbinder->active = false;
}

void rvh_cpumask_any_and_distribute(void *data, struct task_struct *p,
	const struct cpumask *cpu_valid_mask,
	const struct cpumask *new_mask, int *dest_cpu)
{
	cpumask_t valid_mask;

	if (unlikely(!cpumask_and(&valid_mask, cpu_valid_mask, new_mask)))
		goto out;

	/* find a cpu again for the running/runnable/waking tasks if their
	 * current cpu are not allowed
	 */
	if ((p->on_cpu || p->state == TASK_WAKING || task_on_rq_queued(p)) &&
	    !cpumask_test_cpu(task_cpu(p), new_mask))
		*dest_cpu = find_energy_efficient_cpu(p, task_cpu(p),
						      &valid_mask);

out:
	trace_cpumask_any_and_distribute(p, &valid_mask, *dest_cpu);

	return;
}

void rvh_select_task_rq_fair_pixel_mod(void *data, struct task_struct *p, int prev_cpu, int sd_flag,
				       int wake_flags, int *target_cpu)
{
	int sync = (wake_flags & WF_SYNC) && !(current->flags & PF_EXITING);
	bool sync_wakeup = false, prefer_prev = false, sync_boost = false;
	int cpu;

	if (sd_flag == SD_BALANCE_EXEC) {
		*target_cpu = prev_cpu;
		goto out;
	}

	/* sync wake up */
	cpu = smp_processor_id();
	if (sync && cpu_rq(cpu)->nr_running == 1 && cpumask_test_cpu(cpu, p->cpus_ptr) &&
	     cpu_is_in_target_set(p, cpu) && task_fits_capacity(p, cpu, false)) {
		*target_cpu = cpu;
		sync_wakeup = true;
		goto out;
	}

	sync_boost = sync && cpu >= HIGH_CAPACITY_CPU;

	/* prefer prev cpu */
	if (cpumask_test_cpu(prev_cpu, p->cpus_ptr) && cpu_active(prev_cpu) &&
	    cpu_is_idle(prev_cpu) && task_fits_capacity(p, prev_cpu, sync_boost)) {

		struct cpuidle_state *idle_state;
		unsigned int exit_lat = UINT_MAX;

		rcu_read_lock();
		idle_state = idle_get_state(cpu_rq(prev_cpu));

		if (sched_cpu_idle(prev_cpu))
			exit_lat = 0;
		else if (idle_state)
			exit_lat = idle_state->exit_latency;

		rcu_read_unlock();

		if (exit_lat <= C1_EXIT_LATENCY) {
			prefer_prev = true;
			*target_cpu = prev_cpu;
			goto out;
		}
	}

out:
	if (*target_cpu == -1)
		*target_cpu = find_energy_efficient_cpu(p, prev_cpu,
							p->cpus_ptr);
	trace_sched_select_task_rq_fair(p, task_util_est(p),
					sync_wakeup, prefer_prev, sync_boost,
					get_vendor_group(p),
					uclamp_eff_value(p, UCLAMP_MIN),
					uclamp_eff_value(p, UCLAMP_MAX),
					prev_cpu, *target_cpu);
}

