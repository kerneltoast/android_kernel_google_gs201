/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/sched/clock.h>
#include "../../include/sched.h"
#include "binder_internal.h"
#include <asm/atomic.h>

#define UCLAMP_STATS_SLOTS  21
#define UCLAMP_STATS_STEP   (100 / (UCLAMP_STATS_SLOTS - 1))
#define DEF_UTIL_THRESHOLD  1280
#define DEF_UTIL_POST_INIT_SCALE  512
#define DEF_THERMAL_CAP_MARGIN  1536
#define C1_EXIT_LATENCY     1
#define THREAD_PRIORITY_TOP_APP_BOOST 110
#define THREAD_PRIORITY_BACKGROUND    130
#define THREAD_PRIORITY_LOWEST        139
#define LIST_QUEUED         0xa5a55a5a
#define LIST_NOT_QUEUED     0x5a5aa5a5
#define LIB_PATH_LENGTH 512
/*
 * For cpu running normal tasks, its uclamp.min will be 0 and uclamp.max will be 1024,
 * and the sum will be 1024. We use this as index that cpu is not running important tasks.
 */
#define DEFAULT_IMPRATANCE_THRESHOLD	1024

/*
 * Sets uclamp_max to the task based on the most efficient point of the CPU the
 * task is currently running on.
 */
#define AUTO_UCLAMP_MAX_MAGIC		-2

#define AUTO_UCLAMP_MAX_FLAG_TASK	BIT(0)
#define AUTO_UCLAMP_MAX_FLAG_GROUP	BIT(1)

#define UCLAMP_BUCKET_DELTA DIV_ROUND_CLOSEST(SCHED_CAPACITY_SCALE, UCLAMP_BUCKETS)

/*
 * Bit definition for sched qos features.
 */
#define SCHED_QOS_RAMPUP_MULTIPLIER_BIT	0
#define SCHED_QOS_PREFER_HIGH_CAP_BIT	1
#define SCHED_QOS_AUTO_UCLAMP_MAX_BIT	2
#define SCHED_QOS_PREEMPT_WAKEUP_BIT	3
#define SCHED_QOS_ADPF_BIT		4
#define SCHED_QOS_PREFER_IDLE_BIT	5
#define SCHED_QOS_PREFER_FIT_BIT	6
#define SCHED_QOS_BOOST_PRIO_BIT	7

/* Iterate thr' all leaf cfs_rq's on a runqueue */
#define for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos)			\
	list_for_each_entry_safe(cfs_rq, pos, &rq->leaf_cfs_rq_list,	\
				 leaf_cfs_rq_list)

#define get_bucket_id(__val)								      \
		min_t(unsigned int,							      \
		      __val / DIV_ROUND_CLOSEST(SCHED_CAPACITY_SCALE, UCLAMP_BUCKETS),	      \
		      UCLAMP_BUCKETS - 1)

extern unsigned int thermal_cap_margin[CONFIG_VH_SCHED_MAX_CPU_NR];
extern unsigned int sched_capacity_margin[CONFIG_VH_SCHED_MAX_CPU_NR];
extern unsigned int sched_auto_fits_capacity[CONFIG_VH_SCHED_MAX_CPU_NR];
extern unsigned int sched_dvfs_headroom[CONFIG_VH_SCHED_MAX_CPU_NR];
extern unsigned int sched_auto_uclamp_max[CONFIG_VH_SCHED_MAX_CPU_NR];
extern unsigned int sched_per_cpu_iowait_boost_max_value[CONFIG_VH_SCHED_MAX_CPU_NR];
extern unsigned int sched_per_task_iowait_boost_max_value;
extern unsigned int vendor_sched_adpf_rampup_multiplier;

extern int pixel_cpu_num;
extern int pixel_cluster_num;
extern int *pixel_cluster_start_cpu;
extern int *pixel_cluster_cpu_num;
extern int *pixel_cpu_to_cluster;
extern int *pixel_cluster_enabled;
extern unsigned int *pixel_cpd_exit_latency;
extern struct thermal_cap thermal_cap[CONFIG_VH_SCHED_MAX_CPU_NR];

extern unsigned int vh_sched_max_load_balance_interval;
extern unsigned int vh_sched_min_granularity_ns;
extern unsigned int vh_sched_wakeup_granularity_ns;
extern unsigned int vh_sched_latency_ns;

extern char boost_at_fork_task_name[LIB_PATH_LENGTH];
extern raw_spinlock_t boost_at_fork_task_name_lock;
extern unsigned long vendor_sched_boost_at_fork_value;

DECLARE_STATIC_KEY_FALSE(auto_migration_margins_enable);
DECLARE_STATIC_KEY_FALSE(auto_dvfs_headroom_enable);


unsigned long approximate_util_avg(unsigned long util, u64 delta);
u64 approximate_runtime(unsigned long util);
inline void __reset_task_affinity(struct task_struct *p);
bool should_boost_at_fork(struct task_struct *p);

#define cpu_overutilized(cap, max, cpu)	\
		((cap) * sched_capacity_margin[cpu] > (max) << SCHED_CAPACITY_SHIFT)

#define cap_scale(v, s) ((v)*(s) >> SCHED_CAPACITY_SHIFT)

static inline void update_auto_fits_capacity(void)
{
	int cpu;

	for (cpu = 0; cpu < pixel_cpu_num; cpu++) {
		u64 limit = approximate_runtime(capacity_orig_of(cpu)) * USEC_PER_MSEC;
		if (static_branch_likely(&auto_dvfs_headroom_enable))
			limit -= TICK_USEC;
		else
			limit = cap_scale(limit - TICK_USEC, capacity_orig_of(cpu));
		sched_auto_fits_capacity[cpu] = approximate_util_avg(0, limit);
	}
}

/*
 * The util will fit the capacity if it has enough headroom to grow within the
 * next tick - which is when any load balancing activity happens to do the
 * correction.
 *
 * If util stays within the capacity before tick has elapsed, then it should be
 * fine. If not, then a correction action must happen shortly after it starts
 * running, hence we treat it as !fit.
 *
 * Make sure to take invariance into account so tasks don't linger on smaller
 * cores which needs to run more on to achieve same utilization compared to big
 * core.
 */
static inline bool fits_capacity(unsigned long util, unsigned long capacity, int cpu)
{
	if (static_branch_likely(&auto_migration_margins_enable))
		return util < sched_auto_fits_capacity[cpu];
	else
		return !cpu_overutilized(util, capacity, cpu);
}

#define lsub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	*ptr -= min_t(typeof(*ptr), *ptr, _val);		\
} while (0)

#define sub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	typeof(*ptr) val = (_val);				\
	typeof(*ptr) res, var = READ_ONCE(*ptr);		\
	res = var - val;					\
	if (res > var)						\
		res = 0;					\
	WRITE_ONCE(*ptr, res);					\
} while (0)

#define __container_of(ptr, type, member) ({			\
	void *__mptr = (void *)(ptr);				\
	((type *)(__mptr - offsetof(type, member))); })

#define remove_from_vendor_group_list(__node, __group) do {			\
	unsigned long irqflags;							\
	raw_spin_lock_irqsave(&vendor_group_list[__group].lock, irqflags);	\
	if (__node == vendor_group_list[__group].cur_iterator)			\
		vendor_group_list[__group].cur_iterator = (__node)->prev;	\
	list_del_init(__node);							\
	raw_spin_unlock_irqrestore(&vendor_group_list[__group].lock, irqflags);	\
} while (0)

#define add_to_vendor_group_list(__node, __group) do {				\
	unsigned long irqflags;							\
	raw_spin_lock_irqsave(&vendor_group_list[__group].lock, irqflags);	\
	list_add_tail(__node, &vendor_group_list[__group].list);		\
	raw_spin_unlock_irqrestore(&vendor_group_list[__group].lock, irqflags);	\
} while (0)

struct vendor_group_property {
	bool prefer_idle;
	bool prefer_high_cap;
	bool task_spreading;
	bool auto_uclamp_max;
	bool auto_prefer_fit;
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	unsigned int group_throttle;
#endif
	cpumask_t group_cfs_skip_mask;
	cpumask_t preferred_idle_mask_low;
	cpumask_t preferred_idle_mask_mid;
	cpumask_t preferred_idle_mask_high;
	unsigned int uclamp_min_on_nice_low_value;
	unsigned int uclamp_min_on_nice_mid_value;
	unsigned int uclamp_min_on_nice_high_value;
	unsigned int uclamp_max_on_nice_low_value;
	unsigned int uclamp_max_on_nice_mid_value;
	unsigned int uclamp_max_on_nice_high_value;
	unsigned int uclamp_min_on_nice_low_prio;
	unsigned int uclamp_min_on_nice_mid_prio;
	unsigned int uclamp_min_on_nice_high_prio;
	unsigned int uclamp_max_on_nice_low_prio;
	unsigned int uclamp_max_on_nice_mid_prio;
	unsigned int uclamp_max_on_nice_high_prio;
	bool uclamp_min_on_nice_enable;
	bool uclamp_max_on_nice_enable;
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	enum utilization_group ug;
#endif
	struct uclamp_se uc_req[UCLAMP_CNT];
	unsigned int rampup_multiplier;
	bool disable_util_est;

	bool qos_adpf_enable;
	bool qos_prefer_idle_enable;
	bool qos_prefer_fit_enable;
	bool qos_boost_prio_enable;
	bool qos_preempt_wakeup_enable;
	bool qos_auto_uclamp_max_enable;
	bool qos_prefer_high_cap_enable;
	bool qos_rampup_multiplier_enable;

	bool disable_sched_setaffinity;
	bool use_batch_policy;
};

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
struct vendor_util_group_property {
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	unsigned int group_throttle;
#endif
	struct uclamp_se uc_req[UCLAMP_CNT];
};
#endif

struct uclamp_stats {
	spinlock_t lock;
	bool last_min_in_effect;
	bool last_max_in_effect;
	unsigned int last_uclamp_min_index;
	unsigned int last_uclamp_max_index;
	unsigned int last_util_diff_min_index;
	unsigned int last_util_diff_max_index;
	u64 util_diff_min[UCLAMP_STATS_SLOTS];
	u64 util_diff_max[UCLAMP_STATS_SLOTS];
	u64 total_time;
	u64 last_update_time;
	u64 time_in_state_min[UCLAMP_STATS_SLOTS];
	u64 time_in_state_max[UCLAMP_STATS_SLOTS];
	u64 effect_time_in_state_min[UCLAMP_STATS_SLOTS];
	u64 effect_time_in_state_max[UCLAMP_STATS_SLOTS];
};

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
struct vendor_cfs_util {
	raw_spinlock_t lock;
	struct sched_avg avg;
	unsigned long util_removed;
	unsigned long util_est;
};
#endif

struct vendor_group_list {
	struct list_head list;
	raw_spinlock_t lock;
	struct list_head *cur_iterator;
	struct mutex iter_mutex;
};

unsigned long apply_dvfs_headroom(unsigned long util, int cpu, bool tapered);
unsigned long map_util_freq_pixel_mod(unsigned long util, unsigned long freq,
				      unsigned long cap, int cpu);
void check_migrate_rt_task(struct rq *rq, struct task_struct *p);
void rvh_uclamp_eff_get_pixel_mod(void *data, struct task_struct *p, enum uclamp_id clamp_id,
				  struct uclamp_se *uclamp_max, struct uclamp_se *uclamp_eff,
				  int *ret);

enum vendor_group_attribute {
	VTA_TASK_GROUP,
	VTA_PROC_GROUP,
};

enum VENDOR_TUNABLE_TYPE {
	SCHED_CAPACITY_MARGIN,
	SCHED_AUTO_UCLAMP_MAX,
	SCHED_DVFS_HEADROOM,
	SCHED_IOWAIT_BOOST_MAX,
	THERMAL_CAP_MARGIN,
};

#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
struct vendor_task_group_struct {
	enum vendor_group group;
};

ANDROID_VENDOR_CHECK_SIZE_ALIGN(u64 android_vendor_data1[4], struct vendor_task_group_struct t);
#endif

extern bool vendor_sched_reduce_prefer_idle;
extern bool vendor_sched_auto_prefer_idle;
extern struct vendor_group_property vg[VG_MAX];

DECLARE_STATIC_KEY_FALSE(uclamp_min_filter_enable);
DECLARE_STATIC_KEY_FALSE(uclamp_max_filter_enable);

DECLARE_STATIC_KEY_FALSE(tapered_dvfs_headroom_enable);

DECLARE_STATIC_KEY_FALSE(enqueue_dequeue_ready);

DECLARE_STATIC_KEY_FALSE(skip_inefficient_opps_enable);

/*
 * Any governor that relies on util signal to drive DVFS, must populate these
 * percpu dvfs_update_delay variables.
 *
 * It should describe the rate/delay at which the governor sends DVFS freq
 * update to the hardware in us.
 */
DECLARE_PER_CPU(u64, dvfs_update_delay);

#define SCHED_PIXEL_FORCE_UPDATE		BIT(8)

/*****************************************************************************/
/*                       Upstream Code Section                               */
/*****************************************************************************/
/*
 * This part of code is copied from Android common GKI kernel and unmodified.
 * Any change for these functions in upstream GKI would require extensive review
 * to make proper adjustment in vendor hook.
 */
#define UTIL_EST_MARGIN (SCHED_CAPACITY_SCALE / 100)

extern struct uclamp_se uclamp_default[UCLAMP_CNT];

void set_next_buddy(struct sched_entity *se);

static inline unsigned long task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}

static inline unsigned long _task_util_est(struct task_struct *p)
{
	struct util_est ue = READ_ONCE(p->se.avg.util_est);

	return max(ue.ewma, (ue.enqueued & ~UTIL_AVG_UNCHANGED));
}

static inline unsigned long task_util_est(struct task_struct *p)
{
	return max(task_util(p), _task_util_est(p));
}

static inline unsigned long capacity_of(int cpu)
{
	return cpu_rq(cpu)->cpu_capacity;
}

static inline unsigned int uclamp_none(enum uclamp_id clamp_id)
{
	if (clamp_id == UCLAMP_MIN)
		return 0;
	return SCHED_CAPACITY_SCALE;
}

static inline void uclamp_se_set(struct uclamp_se *uc_se,
				 unsigned int value, bool user_defined)
{
	uc_se->value = value;
	uc_se->bucket_id = get_bucket_id(value);
	uc_se->user_defined = user_defined;
}

extern inline void uclamp_rq_inc_id(struct rq *rq, struct task_struct *p,
				    enum uclamp_id clamp_id);
extern inline void uclamp_rq_dec_id(struct rq *rq, struct task_struct *p,
				    enum uclamp_id clamp_id);

static inline void
uclamp_update_active(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct rq_flags rf;
	struct rq *rq;

	if (!uclamp_is_used())
		return;

	/*
	 * Lock the task and the rq where the task is (or was) queued.
	 *
	 * We might lock the (previous) rq of a !RUNNABLE task, but that's the
	 * price to pay to safely serialize util_{min,max} updates with
	 * enqueues, dequeues and migration operations.
	 * This is the same locking schema used by __set_cpus_allowed_ptr().
	 */
	rq = task_rq_lock(p, &rf);

	/*
	 * Setting the clamp bucket is serialized by task_rq_lock().
	 * If the task is not yet RUNNABLE and its task_struct is not
	 * affecting a valid clamp bucket, the next time it's enqueued,
	 * it will already see the updated clamp bucket value.
	 */
	if (p->uclamp[clamp_id].active) {
		uclamp_rq_dec_id(rq, p, clamp_id);
		uclamp_rq_inc_id(rq, p, clamp_id);

		if (clamp_id == UCLAMP_MAX && rq->uclamp_flags & UCLAMP_FLAG_IDLE)
			rq->uclamp_flags &= ~UCLAMP_FLAG_IDLE;
	}

	task_rq_unlock(rq, p, &rf);
}

static inline int util_fits_cpu(unsigned long util,
				unsigned long uclamp_min,
				unsigned long uclamp_max,
				int cpu)
{
	unsigned long capacity_orig, capacity_orig_thermal;
	unsigned long capacity = capacity_of(cpu);
	bool fits, uclamp_max_fits;

	/*
	 * Check if the real util fits without any uclamp boost/cap applied.
	 */
	fits = fits_capacity(util, capacity, cpu);

	if (!uclamp_is_used())
		return fits;

	/*
	 * We must use capacity_orig_of() for comparing against uclamp_min and
	 * uclamp_max. We only care about capacity pressure (by using
	 * capacity_of()) for comparing against the real util.
	 *
	 * If a task is boosted to 1024 for example, we don't want a tiny
	 * pressure to skew the check whether it fits a CPU or not.
	 *
	 * Similarly if a task is capped to capacity_orig_of(little_cpu), it
	 * should fit a little cpu even if there's some pressure.
	 *
	 * Only exception is for thermal pressure since it has a direct impact
	 * on available OPP of the system.
	 *
	 * We honour it for uclamp_min only as a drop in performance level
	 * could result in not getting the requested minimum performance level.
	 *
	 * For uclamp_max, we can tolerate a drop in performance level as the
	 * goal is to cap the task. So it's okay if it's getting less.
	 *
	 * In case of capacity inversion, which is not handled yet, we should
	 * honour the inverted capacity for both uclamp_min and uclamp_max all
	 * the time.
	 */
	capacity_orig = capacity_orig_of(cpu);
	capacity_orig_thermal = capacity_orig - arch_scale_thermal_pressure(cpu);

	/*
	 * We want to force a task to fit a cpu as implied by uclamp_max.
	 * But we do have some corner cases to cater for..
	 *
	 *
	 *                                 C=z
	 *   |                             ___
	 *   |                  C=y       |   |
	 *   |_ _ _ _ _ _ _ _ _ ___ _ _ _ | _ | _ _ _ _ _  uclamp_max
	 *   |      C=x        |   |      |   |
	 *   |      ___        |   |      |   |
	 *   |     |   |       |   |      |   |    (util somewhere in this region)
	 *   |     |   |       |   |      |   |
	 *   |     |   |       |   |      |   |
	 *   +----------------------------------------
	 *         cpu0        cpu1       cpu2
	 *
	 *   In the above example if a task is capped to a specific performance
	 *   point, y, then when:
	 *
	 *   * util = 80% of x then it does not fit on cpu0 and should migrate
	 *     to cpu1
	 *   * util = 80% of y then it is forced to fit on cpu1 to honour
	 *     uclamp_max request.
	 *
	 *   which is what we're enforcing here. A task always fits if
	 *   uclamp_max <= capacity_orig. But when uclamp_max > capacity_orig,
	 *   the normal upmigration rules should withhold still.
	 *
	 *   Only exception is when we are on max capacity, then we need to be
	 *   careful not to block overutilized state. This is so because:
	 *
	 *     1. There's no concept of capping at max_capacity! We can't go
	 *        beyond this performance level anyway.
	 *     2. The system is being saturated when we're operating near
	 *        max capacity, it doesn't make sense to block overutilized.
	 */
	uclamp_max_fits = (capacity_orig == SCHED_CAPACITY_SCALE) && (uclamp_max == SCHED_CAPACITY_SCALE);
	uclamp_max_fits = !uclamp_max_fits && (uclamp_max <= capacity_orig);
	uclamp_max_fits = uclamp_max_fits && (uclamp_max <= thermal_cap[cpu].uclamp_max);
	fits = fits || uclamp_max_fits;

	/*
	 *
	 *                                 C=z
	 *   |                             ___       (region a, capped, util >= uclamp_max)
	 *   |                  C=y       |   |
	 *   |_ _ _ _ _ _ _ _ _ ___ _ _ _ | _ | _ _ _ _ _ uclamp_max
	 *   |      C=x        |   |      |   |
	 *   |      ___        |   |      |   |      (region b, uclamp_min <= util <= uclamp_max)
	 *   |_ _ _|_ _|_ _ _ _| _ | _ _ _| _ | _ _ _ _ _ uclamp_min
	 *   |     |   |       |   |      |   |
	 *   |     |   |       |   |      |   |      (region c, boosted, util < uclamp_min)
	 *   +----------------------------------------
	 *         cpu0        cpu1       cpu2
	 *
	 * a) If util > uclamp_max, then we're capped, we don't care about
	 *    actual fitness value here. We only care if uclamp_max fits
	 *    capacity without taking margin/pressure into account.
	 *    See comment above.
	 *
	 * b) If uclamp_min <= util <= uclamp_max, then the normal
	 *    fits_capacity() rules apply. Except we need to ensure that we
	 *    enforce we remain within uclamp_max, see comment above.
	 *
	 * c) If util < uclamp_min, then we are boosted. Same as (b) but we
	 *    need to take into account the boosted value fits the CPU without
	 *    taking margin/pressure into account.
	 *
	 * Cases (a) and (b) are handled in the 'fits' variable already. We
	 * just need to consider an extra check for case (c) after ensuring we
	 * handle the case uclamp_min > uclamp_max.
	 */
	uclamp_min = min(uclamp_min, uclamp_max);
	if (util < uclamp_min && capacity_orig != SCHED_CAPACITY_SCALE)
		fits = fits && (uclamp_min <= capacity_orig_thermal);

	return fits;
}

static inline unsigned long
uclamp_eff_value_pixel_mod(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct uclamp_se uc_max = uclamp_default[clamp_id];
	struct uclamp_se uc_eff;
	int ret;

	/* Task currently refcounted: use back-annotated (effective) value */
	if (p->uclamp[clamp_id].active)
		return (unsigned long)p->uclamp[clamp_id].value;

	// This function will always return uc_eff
	rvh_uclamp_eff_get_pixel_mod(NULL, p, clamp_id, &uc_max, &uc_eff, &ret);

	return (unsigned long)uc_eff.value;
}

static inline bool uclamp_boosted_pixel_mod(struct task_struct *p)
{
	return uclamp_eff_value_pixel_mod(p, UCLAMP_MIN) > 0;
}

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
static inline struct vendor_task_group_struct *get_vendor_task_group_struct(struct task_group *tg)
{
	return (struct vendor_task_group_struct *)tg->android_vendor_data1;
}
#endif

struct vendor_rq_struct {
	raw_spinlock_t lock;
	unsigned long util_removed;
	unsigned long iowait_boost;
	atomic_t num_adpf_tasks;
};

ANDROID_VENDOR_CHECK_SIZE_ALIGN(u64 android_vendor_data1[96], struct vendor_rq_struct t);

static inline struct vendor_rq_struct *get_vendor_rq_struct(struct rq *rq)
{
	return (struct vendor_rq_struct *)rq->android_vendor_data1;
}

static inline bool get_adpf(struct task_struct *p, bool inherited)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_inheritance_struct *vi = get_vendor_inheritance_struct(p);

	if (inherited)
		return (vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_ADPF_BIT) || vi->adpf) &&
		       vg[vp->group].qos_adpf_enable;
	else
		return vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_ADPF_BIT) &&
		       vg[vp->group].qos_adpf_enable;
}

static inline bool is_binder_task(struct task_struct *p)
{
	return get_vendor_task_struct(p)->is_binder_task;
}

static inline bool should_auto_prefer_idle(struct task_struct *p, int group)
{
	if (group == VG_TOPAPP) {
		/*
		 * Binder Task
		 */
		if (is_binder_task(p))
			return true;
		/*
		 * Task of prio <= 120 and with possitive wake_q_count
		 */
		if (p->prio <= DEFAULT_PRIO && p->wake_q_count)
			return true;
		/*
		 * Task of prio <= 120 and waked up by another process
		 */
		if (current) {
			if (p->prio <= DEFAULT_PRIO && current->tgid != p->tgid)
				return true;
		}
	} else if (group == VG_FOREGROUND) {
		/*
		 * Binder Task
		 */
		if (is_binder_task(p))
			return true;
	}

	return false;
}

static inline bool get_prefer_idle(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_inheritance_struct *vi = get_vendor_inheritance_struct(p);

	// Always perfer idle for tasks with prefer_idle set explicitly.
	// In auto_prefer_idle case, only allow high prio tasks of the prefer_idle group,
	// or high prio task with wake_q_count value greater than 0 in top-app.
	if ((vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_PREFER_IDLE_BIT) || vi->prefer_idle) &&
	    vg[vp->group].qos_prefer_idle_enable)
		return true;
	else if (vendor_sched_auto_prefer_idle)
		return should_auto_prefer_idle(p, vp->group);
	else if (vendor_sched_reduce_prefer_idle)
		return (vg[vp->group].prefer_idle && p->prio <= DEFAULT_PRIO &&
			uclamp_eff_value_pixel_mod(p, UCLAMP_MAX) == SCHED_CAPACITY_SCALE);
	else
		return vg[vp->group].prefer_idle;
}

static inline bool get_prefer_fit(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_inheritance_struct *vi = get_vendor_inheritance_struct(p);

	return (vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_PREFER_FIT_BIT) ||
	       vi->prefer_fit) && vg[vp->group].qos_prefer_fit_enable;
}

static inline bool get_boost_prio(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);

	return (vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_BOOST_PRIO_BIT) &&
		vg[vp->group].qos_boost_prio_enable);
}

static inline bool get_preempt_wakeup(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_inheritance_struct *vi = get_vendor_inheritance_struct(p);

	return (vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_PREEMPT_WAKEUP_BIT) ||
	       vi->preempt_wakeup) && vg[vp->group].qos_preempt_wakeup_enable;
}

static inline bool get_auto_uclamp_max(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);

	return vg[vp->group].auto_uclamp_max ||
	       (vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_AUTO_UCLAMP_MAX_BIT) &&
		vg[vp->group].qos_auto_uclamp_max_enable);
}

static inline bool get_power_efficiency(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);

	return vp->sched_qos_profile == SCHED_QOS_POWER_EFFICIENCY;
}

static inline bool get_prefer_high_cap(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_inheritance_struct *vi = get_vendor_inheritance_struct(p);

	return vg[vp->group].prefer_high_cap || vp->auto_prefer_high_cap ||
	       ((vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_PREFER_HIGH_CAP_BIT) ||
		vi->prefer_high_cap) && vg[vp->group].qos_prefer_high_cap_enable);
}

static inline unsigned int get_rampup_multiplier(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	bool rampup_qos_user_defined =
		vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_RAMPUP_MULTIPLIER_BIT);

	if (!vg[vp->group].qos_rampup_multiplier_enable)
		return vg[vp->group].rampup_multiplier;

	if (get_adpf(p, true))
		return rampup_qos_user_defined
			? max(vp->rampup_multiplier, vendor_sched_adpf_rampup_multiplier)
			: vendor_sched_adpf_rampup_multiplier;
	else
		return rampup_qos_user_defined
			? vp->rampup_multiplier
			: vg[vp->group].rampup_multiplier;
}

static inline void set_auto_prefer_high_cap(struct task_struct *p, bool val)
{
	get_vendor_task_struct(p)->auto_prefer_high_cap = val;
}

static inline void init_vendor_inheritance_struct(struct vendor_inheritance_struct *vi)
{
	int i;

	for (i = 0; i < VI_MAX; i++) {
		vi->uclamp[i][UCLAMP_MIN] = uclamp_none(UCLAMP_MIN);
		vi->uclamp[i][UCLAMP_MAX] = uclamp_none(UCLAMP_MAX);
	}
	vi->adpf = 0;
	vi->prefer_idle = 0;
	vi->prefer_high_cap = 0;
	vi->prefer_fit = 0;
	vi->preempt_wakeup = 0;
}

/*
 * Returns true if task has privilege false otherwise.
 */
static inline bool check_cred(struct task_struct *p)
{
	const struct cred *cred, *tcred;
	bool ret = true;

	cred = current_cred();
	tcred = get_task_cred(p);
	if (!uid_eq(cred->euid, GLOBAL_ROOT_UID) &&
	    !uid_eq(cred->euid, tcred->uid) &&
	    !uid_eq(cred->euid, tcred->suid) &&
	    !ns_capable(tcred->user_ns, CAP_SYS_NICE)) {
		ret = false;
	}
	put_cred(tcred);
	return ret;
}

static inline void init_vendor_task_struct(struct vendor_task_struct *v_tsk)
{
	/* Guarantee everything is not random first, just in case */
	memset(v_tsk, 0, sizeof(struct vendor_task_struct));

	/* Then explicitly set what we expect init value to be */
	raw_spin_lock_init(&v_tsk->lock);
	v_tsk->group = VG_SYSTEM;
	v_tsk->direct_reclaim_ts = 0;
	INIT_LIST_HEAD(&v_tsk->node);
	v_tsk->queued_to_list = LIST_NOT_QUEUED;
	v_tsk->auto_prefer_high_cap = false;
	v_tsk->auto_uclamp_max_flags = 0;
	v_tsk->uclamp_filter.uclamp_min_ignored = 0;
	v_tsk->uclamp_filter.uclamp_max_ignored = 0;
	v_tsk->iowait_boost = 0;
	v_tsk->is_binder_task = false;
	v_tsk->runnable_start_ns = -1;
	v_tsk->delta_exec = 0;
	v_tsk->util_enqueued = 0;
	v_tsk->util_dequeued = 0;
	v_tsk->prev_util_dequeued = 0;
	v_tsk->ignore_util_est_update = false;
	v_tsk->rampup_multiplier = 1;
	v_tsk->sched_qos_profile = SCHED_QOS_NONE;
	v_tsk->sched_qos_user_defined_flag = 0;
	v_tsk->prev_sched_qos_user_defined_flag = 0;
	init_vendor_inheritance_struct(&v_tsk->vi);
	v_tsk->adpf_adj = 0;
	v_tsk->real_cap_avg = 0;
	v_tsk->real_cap_update_ns = 0;
	v_tsk->real_cap_total_ns = 0;
}

extern u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se);
extern unsigned int sysctl_sched_uclamp_min_filter_us;
extern unsigned int sysctl_sched_uclamp_max_filter_divider;
extern unsigned int sysctl_sched_uclamp_min_filter_rt;
extern unsigned int sysctl_sched_uclamp_max_filter_rt;

/*
 * Check if we can ignore uclamp_min requirement of a task. The goal is to
 * prevent small transient tasks from boosting frequency unnecessarily.
 *
 * Returns true if a task can finish its work within a specific threshold.
 *
 * We look at the immediate history of how long the task ran previously.
 * Converting task util_avg into runtime is not trivial and expensive
 * operations.
 */
static inline bool uclamp_can_ignore_uclamp_min(struct rq *rq,
						struct task_struct *p)
{
	struct cpufreq_policy *policy;
	struct sched_entity *se;
	unsigned long runtime;

	if (SCHED_WARN_ON(!uclamp_is_used()))
		return false;

	if (!static_branch_likely(&uclamp_min_filter_enable))
		return false;

	if (task_on_rq_migrating(p))
		return false;

	if (get_adpf(p, true))
		return false;

	if (p->in_iowait && uclamp_boosted_pixel_mod(p))
		return false;

	if (rt_task(p))
		return task_util(p) < sysctl_sched_uclamp_min_filter_rt;

	/*
	 * Based on previous runtime, we check that runtime is sufficiently
	 * larger than a threshold
	 *
	 *
	 *	runtime >= sysctl_sched_uclamp_min_filter_us
	 *
	 * There are 2 caveats:
	 *
	 * 1- When a task migrates on big.LITTLE system, the runtime will not
	 *    be representative then. But this would be one time off error.
	 *
	 * 2. runtime is not frequency invariant. See comment in
	 *    uclamp_can_ignore_uclamp_max()
	 *
	 */
	se = &p->se;
	runtime = se->sum_exec_runtime - se->prev_sum_exec_runtime;
	if (!runtime)
		return false;

	/*
	 * XXX: This can explode if the governor changes in the wrong moment.
	 * We need to create per cpu variables and access those instead. This
	 * will be addressed in the future.
	 */
	policy = cpufreq_cpu_get_raw(cpu_of(rq));
	if (!policy)
		return false;

	if (runtime >= sysctl_sched_uclamp_min_filter_us * 1000)
		return false;

	return true;
}

/*
 * Check if we can ignore uclamp_max requirement of a task. The goal is to
 * prevent small transient tasks that share the rq with other tasks that are
 * capped to lift the capping easily/unnecessarily, hence increase power
 * consumption.
 *
 * Returns true if a task can finish its work within a sched_slice() / divider.
 *
 * We look at the immediate history of how long the task ran previously.
 * Converting task util_avg into runtime or sched_slice() into capacity is not
 * trivial and is an expensive operations. In practice this simple approach
 * proved effective to address the common source of noise. If a task suddenly
 * becomes a busy task, we should detect that and lift the capping at tick, see
 * task_tick_uclamp().
 */
static inline bool uclamp_can_ignore_uclamp_max(struct rq *rq,
						struct task_struct *p)
{
	unsigned long uclamp_max, util;
	unsigned long runtime, slice;
	struct sched_entity *se;
	struct cfs_rq *cfs_rq;
	bool is_rt = rt_task(p);

	if (SCHED_WARN_ON(!uclamp_is_used()))
		return false;

	if (!static_branch_likely(&uclamp_max_filter_enable))
		return false;

	if (task_on_rq_migrating(p))
		return false;

	if (get_adpf(p, true))
		return false;

	if (p->in_iowait && uclamp_boosted_pixel_mod(p))
		return false;

	/*
	 * If util has crossed uclamp_max threshold, then we have to ensure
	 * this is always enforced.
	 */
	util = is_rt ? task_util(p) : task_util_est(p);
	uclamp_max = uclamp_eff_value_pixel_mod(p, UCLAMP_MAX);
	if (util >= uclamp_max)
		return false;

	if (is_rt)
		return util < sysctl_sched_uclamp_max_filter_rt;

	/*
	 * Based on previous runtime, we check the allowed sched_slice() of the
	 * task is large enough for this task to run without preemption.
	 *
	 *
	 *	runtime < sched_slice() / divider
	 *
	 * ==>
	 *
	 *	runtime * divider < sched_slice()
	 *
	 * There are 2 caveats:
	 *
	 * 1- When a task migrates on big.LITTLE system, the runtime will not
	 *    be representative then (not capacity invariant). But this would
	 *    be one time off error.
	 *
	 * 2. runtime is not frequency invariant either. If the
	 *    divider >= fmax/fmin we should be okay in general because that's
	 *    the worst case scenario of how much the runtime will be stretched
	 *    due to it being capped to minimum frequency but the rq should run
	 *    at max. The rule here is that the task should finish its work
	 *    within its sched_slice(). Without this runtime scaling there's a
	 *    small opportunity for the task to ping-pong between capped and
	 *    uncapped state.
	 *
	 */
	se = &p->se;

	runtime = se->sum_exec_runtime - se->prev_sum_exec_runtime;
	if (!runtime)
		return false;

	cfs_rq = cfs_rq_of(se);
	slice = sched_slice(cfs_rq, se);
	runtime *= sysctl_sched_uclamp_max_filter_divider;

	if (runtime >= slice)
		return false;

	return true;
}

static inline void uclamp_set_ignore_uclamp_min(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	vp->uclamp_filter.uclamp_min_ignored = 1;
}
static inline void uclamp_reset_ignore_uclamp_min(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	vp->uclamp_filter.uclamp_min_ignored = 0;
}
static inline void uclamp_set_ignore_uclamp_max(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	vp->uclamp_filter.uclamp_max_ignored = 1;
}
static inline void uclamp_reset_ignore_uclamp_max(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	vp->uclamp_filter.uclamp_max_ignored = 0;
}

static inline bool uclamp_is_ignore_uclamp_min(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	return vp->uclamp_filter.uclamp_min_ignored;
}
static inline bool uclamp_is_ignore_uclamp_max(struct task_struct *p)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	return vp->uclamp_filter.uclamp_max_ignored;
}

static inline bool apply_uclamp_filters(struct rq *rq, struct task_struct *p)
{
	int auto_uclamp_max = get_vendor_task_struct(p)->auto_uclamp_max_flags;
	unsigned long rq_uclamp_min = rq->uclamp[UCLAMP_MIN].value;
	unsigned long rq_uclamp_max = rq->uclamp[UCLAMP_MAX].value;
	bool force_cpufreq_update;

	/*
	 * For AUTO_UCLAMP_MAX_FLAG_GROUP the effective value should have been
	 * updated correctly already by uclamp_rq_inc_id() by GKI. But for
	 * per-task auto_uclamp_max we need to ensure we update
	 * p->uclamp_req[] to reflect the CPU we are currently running on.
	 */
	if (auto_uclamp_max & AUTO_UCLAMP_MAX_FLAG_TASK) {
		/* GKI has incremented it already, undo that */
		uclamp_rq_dec_id(rq, p, UCLAMP_MAX);

		/* update uclamp_max if set to auto */
		uclamp_se_set(&p->uclamp_req[UCLAMP_MAX],
			      sched_auto_uclamp_max[task_cpu(p)], true);

		/*
		 * re-apply uclamp_max applying the potentially new
		 * auto value
		 */
		uclamp_rq_inc_id(rq, p, UCLAMP_MAX);

		/* Reset clamp idle holding when there is one RUNNABLE task */
		if (rq->uclamp_flags & UCLAMP_FLAG_IDLE)
			rq->uclamp_flags &= ~UCLAMP_FLAG_IDLE;
	}

	/*
	 * We can't ignore uclamp_min or uclamp_max individually without side
	 * effects due to the way UCLAMP_FLAG_IDLE Is handled. It'll cause
	 * confusions and spit out warnings due to imbalances.
	 *
	 * If one of them needs to be ignored, then we assume the other must be
	 * ignored too.
	 *
	 * This should keep some implicit assumptions about how these values
	 * are inc/dec and how the flag is handled correct.
	 */
	if (uclamp_can_ignore_uclamp_min(rq, p) ||
	    uclamp_can_ignore_uclamp_max(rq, p)) {

		uclamp_set_ignore_uclamp_min(p);
		uclamp_set_ignore_uclamp_max(p);

		/* GKI has incremented it already, undo that */
		uclamp_rq_dec_id(rq, p, UCLAMP_MIN);
		uclamp_rq_dec_id(rq, p, UCLAMP_MAX);
	}

	/*
	 * Force cpufreq update if we filtered and the new rq eff value is
	 * smaller than it was at func entry.
	 */
	force_cpufreq_update = rq_uclamp_min > rq->uclamp[UCLAMP_MIN].value;
	force_cpufreq_update |= rq_uclamp_max > rq->uclamp[UCLAMP_MAX].value;

	return force_cpufreq_update;
}

static inline void inc_adpf_counter(struct task_struct *p, struct rq *rq)
{
	struct vendor_rq_struct *vrq;

	if (rt_task(p))
		return;

	vrq = get_vendor_rq_struct(rq);

	atomic_inc(&vrq->num_adpf_tasks);
	/*
	 * Tell the scheduler that this tasks really wants to run next
	 */
	set_next_buddy(&p->se);
}

static inline void dec_adpf_counter(struct task_struct *p, struct rq *rq)
{
	struct vendor_rq_struct *vrq = get_vendor_rq_struct(rq);

	if (rt_task(p))
		return;

	vrq = get_vendor_rq_struct(rq);

	/*
	 * An enqueue could have happened before our dequeue hook was
	 * registered, which can lead to imbalance.
	 *
	 * Make sure to never go below 0.
	 */
	atomic_dec_if_positive(&vrq->num_adpf_tasks);
}

static inline void update_adpf_counter(struct task_struct *p, bool old_adpf)
{
	lockdep_assert_rq_held(task_rq(p));

	if (task_on_rq_queued(p)) {
		if (old_adpf && !get_adpf(p, true))
			dec_adpf_counter(p, task_rq(p));
		else if (!old_adpf && get_adpf(p, true))
			inc_adpf_counter(p, task_rq(p));
	}
}

extern int vendor_sched_ug_bg_auto_prio;

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
extern struct vendor_cfs_util vendor_cfs_util[UG_MAX][CONFIG_VH_SCHED_MAX_CPU_NR];
static inline enum utilization_group get_utilization_group(struct task_struct *p, int group)
{
	if (vg[group].ug == UG_AUTO) {
		// Always consider task prio >= vendor_sched_ug_bg_auto_prio as background util
		if (p->prio >= vendor_sched_ug_bg_auto_prio)
			return UG_BG;

		return UG_FG;
	}

	return vg[group].ug;
}
#endif

/*
 * Counter the impact of utilization invariance which can slow down ramp-up
 * time when tasks become suddenly busy.
 *
 * It is only enabled when auto_dvfs_headroom is enabled.
 */
static inline void __update_util_est_invariance(struct rq *rq,
						struct task_struct *p,
						bool update_cfs_rq)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	unsigned long se_enqueued, cfs_rq_enqueued, new_util_est;
	unsigned long util = task_util(p);
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct sched_entity *se = &p->se;
	unsigned int rampup_multiplier;
	int __maybe_unused group;
	unsigned long irqflags;
	u64 dequeue_time_ns;
	u64 delta_exec;
	bool do_update;

	if (!static_branch_likely(&auto_dvfs_headroom_enable))
		return;

	if (!sched_feat(UTIL_EST))
		return;

	if (!fair_policy(p->policy) && !idle_policy(p->policy))
		return;

	rampup_multiplier = get_rampup_multiplier(p);

	if (unlikely(!rampup_multiplier))
		return;

	delta_exec = (se->sum_exec_runtime - vp->prev_sum_exec_runtime)/1000;
	delta_exec *= rampup_multiplier;

	vp->delta_exec += delta_exec;
	vp->prev_sum_exec_runtime = se->sum_exec_runtime;

	dequeue_time_ns = sched_clock() - vp->last_dequeue;
	new_util_est = approximate_util_avg(vp->util_enqueued, vp->delta_exec);

	/* Is the task util increasing? */
	do_update = util > vp->util_dequeued + UTIL_EST_MARGIN;

	/*
	 * Due to invariance util can be stuck at the same value for extended
	 * period of time. Check if this is the case and try to rampup quickly
	 * if it is. To avoid triggering the logic against higher util values
	 * that naturally can linger, check if new_util_est has actually grown
	 * too.
	 */
	do_update |= util == vp->prev_util &&
		dequeue_time_ns >= NSEC_PER_MSEC && new_util_est > vp->util_dequeued + UTIL_EST_MARGIN;

	if (util != vp->prev_util) {
		vp->last_dequeue = sched_clock();
		vp->prev_util = util;
	}

	if (!do_update)
		return;

	se_enqueued = READ_ONCE(se->avg.util_est.enqueued) & ~UTIL_AVG_UNCHANGED;
	se_enqueued = max_t(unsigned long, se->avg.util_est.ewma, se_enqueued);

	cfs_rq_enqueued = READ_ONCE(cfs_rq->avg.util_est.enqueued);

	if (update_cfs_rq) {
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
		group = get_utilization_group(p, get_vendor_group(p));
		raw_spin_lock_irqsave(&vendor_cfs_util[group][rq->cpu].lock, irqflags);
		lsub_positive(&vendor_cfs_util[group][rq->cpu].util_est, se_enqueued);
#endif
		lsub_positive(&cfs_rq_enqueued, se_enqueued);
	}

	WRITE_ONCE(se->avg.util_est.enqueued, new_util_est);
	trace_sched_util_est_se_tp(se);

	if (update_cfs_rq) {
		cfs_rq_enqueued += new_util_est;
		WRITE_ONCE(cfs_rq->avg.util_est.enqueued, cfs_rq_enqueued);
		trace_sched_util_est_cfs_tp(cfs_rq);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
		vendor_cfs_util[group][rq->cpu].util_est += new_util_est;
		raw_spin_unlock_irqrestore(&vendor_cfs_util[group][rq->cpu].lock, irqflags);
#endif
	}
}
