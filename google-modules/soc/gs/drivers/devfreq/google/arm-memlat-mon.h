#ifndef _ARM_MEMLAT_MON_H_
#define _ARM_MEMLAT_MON_H_

#include "governor_memlat.h"

enum mon_type {
	MEMLAT_CPU_GRP,
	MEMLAT_MON,
	NUM_MON_TYPES
};

struct event_data {
	struct perf_event *pevent;
	unsigned long prev_count;
	unsigned long last_delta;
	unsigned long total;
};

struct cpu_data {
	struct event_data pmu_evs[NUM_COMMON_EVS];
	struct event_data amu_evs[NUM_COMMON_EVS];
	unsigned long freq;
	unsigned long stall_pct;
	spinlock_t    pmu_lock;
	unsigned long inst;
	unsigned long cyc;
	unsigned long stall;
	unsigned long l2_cachemiss;
	unsigned long l3_cachemiss;
	unsigned long mem_stall;
	unsigned long l2_cache_wb;
	unsigned long l3_cache_access;
};

/**
 * struct memlat_mon - A specific consumer of cpu_grp generic counters.
 *
 * @is_active:                  Whether or not this mon is currently running
 *                              memlat.
 * @cpus:                       CPUs this mon votes on behalf of. Must be a
 *                              subset of @cpu_grp's CPUs. If no CPUs provided,
 *                              defaults to using all of @cpu_grp's CPUs.
 * @requested_update_ms:        The mon's desired polling rate. The lowest
 *                              @requested_update_ms of all mons determines
 *                              @cpu_grp's update_ms.
 * @hw:                         The memlat_hwmon struct corresponding to this
 *                              mon's specific memlat instance.
 * @governor_name:              Mon supported governor name.
 * @cpu_grp:                    The cpu_grp who owns this mon.
 */
struct memlat_mon {
	bool                    is_active;
	cpumask_t               cpus;
	unsigned int            requested_update_ms;
	struct memlat_hwmon     hw;
	bool                    update_dsu_df;

	struct memlat_cpu_grp   *cpu_grp;
};

/**
 * struct memlat_cpu_grp - A coordinator of both HW reads and devfreq updates
 * for one or more memlat_mons.
 *
 * @cpus:                       The CPUs this cpu_grp will read events from.
 * @common_ev_ids:              The event codes of the events all mons need.
 * @cpus_data:                  The cpus data array of length #cpus. Includes
 *                              event_data of all the events all mons need as
 *                              well as common computed cpu data like freq.
 * @last_update_ts:             Used to avoid redundant reads.
 * @last_ts_delta_us:           The time difference between the most recent
 *                              update and the one before that. Used to compute
 *                              effective frequency.
 * @work:                       The delayed_work used for handling updates.
 * @update_ms:                  The frequency with which @work triggers.
 * @num_mons:           The number of @mons for this cpu_grp.
 * @num_inited_mons:    The number of @mons who have probed.
 * @num_active_mons:    The number of @mons currently running
 *                              memlat.
 * @mons:                       All of the memlat_mon structs representing
 *                              the different voters who share this cpu_grp.
 * @mons_lock:          A lock used to protect the @mons.
 */
struct memlat_cpu_grp {
	struct                  list_head node;
	cpumask_t               cpus;
	unsigned int            pmu_ev_ids[NUM_COMMON_EVS];
	unsigned int            amu_ev_ids[NUM_COMMON_EVS];
	struct cpu_data         *cpus_data;
	ktime_t                 last_update_ts;
	unsigned long           last_ts_delta_us;

	struct delayed_work     work;
	unsigned int            update_ms;

	unsigned int            num_mons;
	unsigned int            num_inited_mons;
	unsigned int            num_active_mons;
	struct memlat_mon       *mons;
	struct mutex            mons_lock;
	bool                    initialized;
};

struct memlat_mon_spec {
	enum mon_type type;
};

#define to_cpu_data(cpu_grp, cpu) \
        (&cpu_grp->cpus_data[cpu - cpumask_first(&cpu_grp->cpus)])
#define to_pmu_evs(cpu_grp, cpu) \
        (cpu_grp->cpus_data[cpu - cpumask_first(&cpu_grp->cpus)].pmu_evs)
#define to_devstats(mon, cpu) \
        (&mon->hw.core_stats[cpu - cpumask_first(&mon->cpus)])
#define to_mon(hwmon) container_of(hwmon, struct memlat_mon, hw)

#endif  // _ARM_MEMLAT_MON_H_
