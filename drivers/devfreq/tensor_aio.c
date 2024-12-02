// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

#include <linux/cpufreq.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/of_platform.h>
#include <linux/perf/arm_pmuv3.h>
#include <linux/reboot.h>
#include <linux/units.h>
#include <linux/sched/cputime.h>
#include <soc/google/acpm/acpm_ipc.h>
#include <soc/google/cal-if.h>
#include <soc/google/ect_parser.h>
#include <soc/google/exynos_pm_qos.h>
#include <trace/hooks/cpuidle.h>
#include <trace/hooks/sched.h>
#include <sched.h>
#include "governor.h"

/* SoC-specific ACPM constant definitions */
#ifdef CONFIG_SOC_GS101
#include <dt-bindings/clock/gs101.h>
#elif defined(CONFIG_SOC_GS201)
#include <dt-bindings/clock/gs201.h>
#elif defined(CONFIG_SOC_ZUMA)
#include <dt-bindings/clock/zuma.h>
#endif

/* Poll memperfd about every 10 ms */
#define MEMPERFD_POLL_HZ (HZ / 100)

/*
 * The percentage of the previous MIF frequency that MIF should be set to when
 * a memory-invariant workload is detected. This controls how fast the MIF
 * frequency is dropped after a memory-intensive workload.
 */
#define MEMPERFD_DOWN_PCT 65

/*
 * The minimum sample time required to measure the cycle counters. This should
 * take into account the time needed to read the monotonic clock.
 */
static u64 cpu_min_sample_cntpct __read_mostly = 100 * NSEC_PER_USEC;

/* The name of our governor exposed to devfreq */
#define DEVFREQ_GOV_TENSOR_AIO "tensor_aio"

/* The PMU (Performance Monitor Unit) event statistics */
struct pmu_stat {
	u64 cpu_cyc;
	u64 mem_cyc;
	u64 cntpct;
};

struct cpu_pmu {
	raw_spinlock_t lock;
	struct pmu_stat cur;
	struct pmu_stat prev;
	struct pmu_stat sfd;
};

static DEFINE_PER_CPU(struct cpu_pmu, cpu_pmu_evs) = {
	.lock = __RAW_SPIN_LOCK_UNLOCKED(cpu_pmu_evs.lock)
};

enum exynos_dev {
#if defined(CONFIG_SOC_GS101) || defined(CONFIG_SOC_GS201)
	BO,
#else
	BCI,
	BW,
	DSU,
#endif
	CAM,
	DISP,
	INT,
	INTCAM,
	MFC,
	MIF,
	TNR,
	MAX_DEV
};

struct exynos_devfreq_data {
	struct exynos_pm_qos_request min_req;
	struct exynos_pm_qos_request umin_req;
	struct exynos_pm_qos_request umax_req;
	struct devfreq_dev_profile profile;
	struct devfreq *df;
	struct notifier_block min_nb;
	struct notifier_block max_nb;
	struct mutex min_nb_lock;
	struct mutex max_nb_lock;
	struct mutex nb_lock;
	/* The table is `unsigned long` just for devfreq; it's actually u32 */
	unsigned long *tbl;
	int qmin;
	int qmax;
	u32 nr_freqs;
	u32 min_freq;
	u32 max_freq;
	u32 cur_freq;
	u32 suspend_freq;
	u32 dfs_id;
	u32 devfreq_type;
	u32 ipc_chan_id;
	bool use_acpm;
	bool suspended;
};

static struct exynos_devfreq_data df_data[MAX_DEV];
#ifdef CONFIG_SOC_ZUMA
static struct exynos_devfreq_data *const bci = &df_data[BCI];
static struct exynos_devfreq_data *const dsu = &df_data[DSU];
#endif
static struct exynos_devfreq_data *const mif = &df_data[MIF];

struct {
	u32 mif_freq;
	u32 int_freq;
} __packed static *mif_int_map;
static int mif_int_cnt __read_mostly;

/* PPC register addresses from gs-ppc.c */
#define PPC_PMNC	0x0004
#define PPC_CNTENS	0x0008
#define PPC_PMCNT1	0x0038
#define PPC_CCNT	0x0048

/* PPC register values from gs-ppc.c */
#define PPC_CHVALUE	(BIT(31) | BIT(1) | BIT(0))
#define PPC_REGVALUE	BIT(24)
#define PPC_RESETALL	(BIT(2) | BIT(1))
#define PPC_GLBCNTEN	BIT(0)

struct ppc_reg {
	u32 off;
	u32 val;
};

static const struct ppc_reg ppc_init_cmd[] = {
	{ PPC_PMNC, PPC_REGVALUE | PPC_RESETALL },
	{ PPC_CNTENS, PPC_CHVALUE },
	{ PPC_PMNC, PPC_REGVALUE | PPC_GLBCNTEN }
};

static const struct ppc_reg ppc_exit_cmd[] = {
	{ PPC_PMNC, PPC_RESETALL }
};

static const struct ppc_reg ppc_stop_cmd[] = {
	{ PPC_PMNC, PPC_REGVALUE }
};

static const struct ppc_reg ppc_start_cmd[] = {
	{ PPC_PMNC, PPC_REGVALUE | PPC_RESETALL },
	{ PPC_PMNC, PPC_REGVALUE | PPC_GLBCNTEN }
};

struct um_group {
	void __iomem **va_base;
	u32 cnt;
	u32 target_load;
};

struct mif_um_ppc {
	struct um_group *grp;
	u32 grp_cnt;
} static mif_um;

static atomic_long_t last_run_jiffies = ATOMIC_INIT(0);
static DECLARE_SWAIT_QUEUE_HEAD(memperfd_waitq);
static DEFINE_PER_CPU_READ_MOSTLY(struct cpufreq_policy, cached_pol);
static unsigned int dsu_scale_factor __read_mostly __maybe_unused;
static bool in_reboot __read_mostly;
static int cpuhp_state;

/*
 * CNTPCT_EL0 arithmetic helpers to avoid overflowing a u64 when converting
 * between ticks and nanoseconds. This avoids needing mult_frac() in a hot path.
 */
static u64 cntpct_mult __read_mostly;
static u64 cntpct_div __read_mostly;
static u64 cntpct_rate __read_mostly;

static u64 cntpct_to_ns(u64 cntpct)
{
	return cntpct * cntpct_mult / cntpct_div;
}

static u64 ns_to_cntpct(u64 ns)
{
	return DIV_ROUND_UP_ULL(ns * cntpct_div, cntpct_mult);
}

static u64 cyc_per_cntpct_to_hz(u64 cyc, u64 cntpct)
{
	return mult_frac(cyc, cntpct_rate, cntpct);
}

static void calc_cntpct_arith(void)
{
	int cd;

	/*
	 * Calculate lossless arithmetic to convert between timer ticks and
	 * nanoseconds, extracting all common denominators up through 10.
	 */
	cntpct_rate = arch_timer_get_rate();
	cntpct_mult = NSEC_PER_SEC;
	cntpct_div = cntpct_rate;
	for (cd = 10; cd > 1; cd--) {
		while (!(cntpct_mult % cd) && !(cntpct_div % cd)) {
			cntpct_div /= cd;
			cntpct_mult /= cd;
		}
	}

	/* Compute all nanosecond time intervals in terms of CNTPCT_EL0 ticks */
	cpu_min_sample_cntpct = ns_to_cntpct(cpu_min_sample_cntpct);
}

enum pmu_events {
	CPU_CYCLES,
	STALL_BACKEND_MEM,
	PMU_EVT_MAX
};

static const u32 pmu_evt_id[PMU_EVT_MAX] = {
	[CPU_CYCLES] = ARMV8_PMUV3_PERFCTR_CPU_CYCLES,
	[STALL_BACKEND_MEM] = ARMV8_AMU_PERFCTR_STALL_BACKEND_MEM
};

struct cpu_pmu_evt {
	struct perf_event *pev[PMU_EVT_MAX];
};

static DEFINE_PER_CPU(struct cpu_pmu_evt, pevt_pcpu);

static struct perf_event *create_pev(struct perf_event_attr *attr, int cpu)
{
	return perf_event_create_kernel_counter(attr, cpu, NULL, NULL, NULL);
}

static void release_perf_events(int cpu)
{
	struct cpu_pmu_evt *cpev = &per_cpu(pevt_pcpu, cpu);
	int i;

	for (i = 0; i < PMU_EVT_MAX; i++) {
		if (IS_ERR(cpev->pev[i]))
			break;

		perf_event_release_kernel(cpev->pev[i]);
	}
}

static int create_perf_events(int cpu)
{
	struct cpu_pmu_evt *cpev = &per_cpu(pevt_pcpu, cpu);
	struct perf_event_attr attr = {
		.type = PERF_TYPE_RAW,
		.size = sizeof(attr),
		.pinned = 1,
		/*
		 * Request a long counter (i.e., 64-bit instead of 32-bit) by
		 * setting bit 0 in config1. See armv8pmu_event_is_64bit().
		 */
		.config1 = 0x1
	};
	int i;

	for (i = 0; i < PMU_EVT_MAX; i++) {
		attr.config = pmu_evt_id[i];
		/*
		 * Cortex-A55 and Cortex-A76 don't support STALL_BACKEND_MEM, so
		 * use STALL_BACKEND as a poor man's alternative. We divide this
		 * value by two in read_perf_event() in order to get a close
		 * guess of how many stalled cycles are due to memory.
		 */
		if (i == STALL_BACKEND_MEM &&
		    ((IS_ENABLED(CONFIG_SOC_GS101) && cpu < 6) ||
		     (IS_ENABLED(CONFIG_SOC_GS201) && cpu < 4)))
			attr.config = ARMV8_PMUV3_PERFCTR_STALL_BACKEND;
		cpev->pev[i] = create_pev(&attr, cpu);
		if (WARN_ON(IS_ERR(cpev->pev[i])))
			goto release_pevs;
	}

	return 0;

release_pevs:
	release_perf_events(cpu);
	return PTR_ERR(cpev->pev[i]);
}

static u64 read_perf_event(enum pmu_events evt)
{
	struct cpu_pmu_evt *cpev = this_cpu_ptr(&pevt_pcpu);
	struct perf_event *event = cpev->pev[evt];
	u64 value;

#ifdef CONFIG_SOC_ZUMA
	/* Read the AMU registers directly for better speed and precision */
	switch (evt) {
	case CPU_CYCLES:
		return read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0);
	case STALL_BACKEND_MEM:
		return read_sysreg_s(SYS_AMEVCNTR0_MEM_STALL);
	default:
		break;
	}
#endif

	/* Do a raw read of the PMU event to go as fast as possible */
	event->pmu->read(event);
	value = local64_read(&event->count);
#if defined(CONFIG_SOC_GS101) || defined(CONFIG_SOC_GS201)
	/*
	 * Divide STALL_BACKEND by two for Cortex-A55 and Cortex-A76 since it's
	 * an overshoot of stalled cycles due to memory.
	 */
	if (event->attr.config == ARMV8_PMUV3_PERFCTR_STALL_BACKEND)
		value /= 2;
#endif
	return value;
}

static void pmu_read_events(struct pmu_stat *stat)
{
	stat->cpu_cyc = read_perf_event(CPU_CYCLES);
	stat->mem_cyc = read_perf_event(STALL_BACKEND_MEM);
}

static void pmu_get_stats(struct pmu_stat *stat)
{
	pmu_read_events(stat);
	stat->cntpct = __arch_counter_get_cntpct();
}

static void kick_memperfd(void)
{
	unsigned long prev, now = jiffies;

	prev = atomic_long_read(&last_run_jiffies);
	if (time_before(now, prev + MEMPERFD_POLL_HZ))
		return;

	if (atomic_long_cmpxchg_relaxed(&last_run_jiffies, prev, now) != prev)
		return;

	/* Ensure the relaxed cmpxchg is ordered before the swait_active() */
	smp_acquire__after_ctrl_dep();
	if (swait_active(&memperfd_waitq))
		swake_up_one(&memperfd_waitq);
}

static u32 find_cpu_freq(struct cpufreq_policy *pol, u64 khz, u32 relation)
{
	struct cpufreq_frequency_table *tbl = pol->freq_table;
	int idx;

	/* Find the nearest frequency in the table with the chosen relation */
	idx = cpufreq_frequency_table_target(pol, khz, relation);
	return tbl[idx].frequency;
}

static void update_thermal_pressure(void)
{
#ifdef CONFIG_SOC_ZUMA
	unsigned int gs_tmu_throt_freq(int cpu);
	int cpu = raw_smp_processor_id();
	struct cpufreq_policy *pol = cpufreq_cpu_get_raw(cpu);
	unsigned long capped_freq;

	if (unlikely(!pol))
		return;

	/* Refresh the thermal pressure with the TMU's current throttle freq */
	capped_freq = min(gs_tmu_throt_freq(cpu), pol->max);
	arch_update_thermal_pressure(cpumask_of(cpu), capped_freq);
#endif
}

static void update_freq_scale(void)
{
	int cpu = raw_smp_processor_id();
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	struct pmu_stat cur, prev = pmu->cur, *sfd = &pmu->sfd;

	/* Check if enough time has passed to take a new sample */
	cur.cntpct = __arch_counter_get_cntpct();
	if ((cur.cntpct - prev.cntpct) >= cpu_min_sample_cntpct) {
		/* Update the PMU counters without rereading the current time */
		pmu_read_events(&cur);
		raw_spin_lock(&pmu->lock);
		pmu->cur = cur;
		raw_spin_unlock(&pmu->lock);

		/* Accumulate more data for calculating the CPU's frequency */
		sfd->cpu_cyc += cur.cpu_cyc - prev.cpu_cyc;
		sfd->cntpct += cur.cntpct - prev.cntpct;
	}

	/*
	 * Set the CPU frequency scale measured via counters if enough data is
	 * present. This excludes idle time because although the cycle counter
	 * stops incrementing while the CPU idles, the system timer doesn't.
	 */
	if (sfd->cntpct >= cpu_min_sample_cntpct) {
		struct cpufreq_policy *pol = &per_cpu(cached_pol, cpu);
		u64 freq, max_freq = pol->cpuinfo.max_freq;
		u64 ns = cntpct_to_ns(sfd->cntpct);

		/* Report the measured frequency and reset the stats */
		freq = min(max_freq, USEC_PER_SEC * sfd->cpu_cyc / ns);
		per_cpu(arch_freq_scale, cpu) =
			SCHED_CAPACITY_SCALE * freq / max_freq;
		sfd->cpu_cyc = sfd->cntpct = 0;
	}
}

/*
 * The scheduler tick is used as a passive way to collect statistics on all
 * CPUs. Collecting statistics with per-CPU timers would result in the cpuidle
 * governor predicting imminent wakeups and thus selecting a shallower idle
 * state, to the detriment of power consumption. When CPUs aren't active,
 * there's no need to collect any statistics, so memperfd is designed to only
 * run when there's CPU activity.
 */
static void tensor_aio_tick(void)
{
	update_thermal_pressure();
	update_freq_scale();
	kick_memperfd();
}

static struct scale_freq_data tensor_aio_sfd = {
	.source = SCALE_FREQ_SOURCE_ARCH,
	.set_freq_scale = tensor_aio_tick
};

static void tensor_aio_cpu_idle(int cpu, bool idle)
{
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	struct pmu_stat *sfd = &pmu->sfd;
	struct pmu_stat cur, prev;

	/* Don't race with reboot */
	if (unlikely(in_reboot))
		return;

	/* Don't race with CPU hotplug */
	if (unlikely(!cpu_active(cpu)))
		return;

	if (idle) {
		/* Update the current counters one last time before idling */
		prev = pmu->cur;
		pmu_get_stats(&cur);
		raw_spin_lock(&pmu->lock);
		pmu->cur = cur;
		raw_spin_unlock(&pmu->lock);

		/* Accumulate data for calculating the CPU's frequency */
		sfd->cpu_cyc += cur.cpu_cyc - prev.cpu_cyc;
		sfd->cntpct += cur.cntpct - prev.cntpct;
	} else {
		/*
		 * Update the counters upon exiting idle without accumulating
		 * frequency data, in order to disregard all statistics from the
		 * period when the CPU was idle. This is because the system
		 * timer keeps incrementing while the CPU is idle, while the
		 * cycle counter doesn't because the CPU clock is gated in idle.
		 */
		pmu_get_stats(&cur);
		raw_spin_lock(&pmu->lock);
		pmu->cur = cur;
		raw_spin_unlock(&pmu->lock);
	}
}

static void tensor_aio_idle_enter(void *data, int *state,
				  struct cpuidle_device *dev)
{
	tensor_aio_cpu_idle(raw_smp_processor_id(), true);
}

static void tensor_aio_idle_exit(void *data, int state,
				 struct cpuidle_device *dev)
{
	tensor_aio_cpu_idle(raw_smp_processor_id(), false);
}

static int memperf_cpuhp_up(unsigned int cpu)
{
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	int ret;

	ret = create_perf_events(cpu);
	if (ret)
		return ret;

	/*
	 * Update and reset the statistics for this CPU as it comes online. No
	 * need to disable interrupts since tensor_aio_tick() isn't running yet,
	 * so pmu->lock can't be acquired from hard IRQ context right now.
	 */
	raw_spin_lock(&pmu->lock);
	pmu_get_stats(&pmu->cur);
	pmu->prev = pmu->cur;
	raw_spin_unlock(&pmu->lock);

	/* Reset the sfd statistics */
	pmu->sfd.cpu_cyc = pmu->sfd.cntpct = 0;

	/* Install tensor_aio_tick() */
	topology_set_scale_freq_source(&tensor_aio_sfd, cpumask_of(cpu));
	return 0;
}

static int memperf_cpuhp_down(unsigned int cpu)
{
	/* Stop tensor_aio_tick() from running on this CPU anymore */
	topology_clear_scale_freq_source(SCALE_FREQ_SOURCE_ARCH,
					 cpumask_of(cpu));
	release_perf_events(cpu);
	return 0;
}

static void update_qos_req(struct exynos_pm_qos_request *req, int value)
{
	/* Only update if the request value is different */
	if (req->node.prio != value)
		exynos_pm_qos_update_request(req, value);
}

/* This closely mimics cpufreq_table_find_index_dh() */
static inline u32 find_freq_h(struct exynos_devfreq_data *data, u32 target)
{
	u32 best = -1, i;

	for (i = 0; i < data->nr_freqs; i++) {
		if (data->tbl[i] <= target)
			return data->tbl[i];

		best = data->tbl[i];
	}

	return best;
}

/* This closely mimics cpufreq_table_find_index_dc() */
static inline u32 find_freq_c(struct exynos_devfreq_data *data, u32 target)
{
	u32 best = -1, i;

	for (i = 0; i < data->nr_freqs; i++) {
		if (data->tbl[i] == target)
			return target;

		if (data->tbl[i] > target) {
			best = data->tbl[i];
			continue;
		}

		/* No frequencies found above the target frequency */
		if (best == -1)
			return data->tbl[i];

		/* Choose the closer frequency */
		if (best - target > target - data->tbl[i])
			return data->tbl[i];

		return best;
	}

	return best;
}

/* This closely mimics cpufreq_table_find_index_dl() */
static inline u32 find_index_l(struct exynos_devfreq_data *data, u32 target)
{
	u32 best = -1, i;

	for (i = 0; i < data->nr_freqs; i++) {
		if (data->tbl[i] == target)
			return i;

		if (data->tbl[i] > target) {
			best = i;
			continue;
		}

		/* No frequencies found above the target frequency */
		if (best == -1)
			return i;

		return best;
	}

	return best;
}

static inline u32 find_freq_l(struct exynos_devfreq_data *data, u32 target)
{
	return data->tbl[find_index_l(data, target)];
}

/* Write specified register tuples to all PPC registers */
static void ppc_write_regs(const struct ppc_reg *reg, size_t cnt)
{
	struct mif_um_ppc *um = &mif_um;
	int i, j, k;

	for (i = 0; i < um->grp_cnt; i++) {
		struct um_group *ug = &um->grp[i];

		for (j = 0; j < ug->cnt; j++) {
			for (k = 0; k < cnt; k++)
				__raw_writel(reg[k].val,
					     ug->va_base[j] + reg[k].off);
		}
	}

	/* Ensure all of the register writes finish before continuing */
	mb();
}

static void memperfd_init(void)
{
	/*
	 * Delete the arch's scale_freq_data callback to get rid of the
	 * duplicated work by the arch's callback, since we read the same
	 * values. This also lets the frequency invariance engine work on cores
	 * with an erratum that breaks the const cycles PMU counter, since we
	 * don't use const cycles. A new scale_freq_data callback is installed
	 * in memperf_cpuhp_up().
	 */
	topology_clear_scale_freq_source(SCALE_FREQ_SOURCE_ARCH,
					 cpu_possible_mask);

	/* Register the CPU hotplug notifier with calls to all online CPUs */
	cpuhp_state = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "memperf",
					memperf_cpuhp_up, memperf_cpuhp_down);
	BUG_ON(cpuhp_state <= 0);

	/* Precompute arithmetic to convert between ticks and nanoseconds */
	calc_cntpct_arith();

	/*
	 * Register the cpuidle callback for frequency-invariant counting needed
	 * to set the CPU frequency scale correctly in update_freq_scale().
	 */
	BUG_ON(register_trace_android_vh_cpu_idle_enter(tensor_aio_idle_enter,
							NULL));
	BUG_ON(register_trace_android_vh_cpu_idle_exit(tensor_aio_idle_exit,
						       NULL));

	/* Initialize and start the PPCs */
	ppc_write_regs(ppc_init_cmd, ARRAY_SIZE(ppc_init_cmd));
}

static u32 mif_cpu_vote(struct pmu_stat *stat, int cpu, u32 cur, u32 *dsu_vote)
{
	struct cpufreq_policy *pol = &per_cpu(cached_pol, cpu);
	u64 cpu_hz, mem_hz;
	u32 cpu_khz, vote;

	/*
	 * Estimate the CPU's average frequency in Hz for this period. Note that
	 * this measurement _includes_ time spent in idle, which is significant
	 * because although the cycle counter stops while the CPU is idle, the
	 * monotonic time source doesn't. Therefore, the CPU's time spent in
	 * idle while its clock is gated at 0 Hz is included in the measurement.
	 * For MIF voting, this attribute is helpful because MIF doesn't idle
	 * while the CPU is idle (unless SICD is entered), so scaling MIF up in
	 * order to satisfy the CPU's frequency-invariant load would cause MIF
	 * to burn more power than just letting the CPU burn a few extra cycles
	 * on memory stalls. And if the CPU is under heavy load, it won't have
	 * much idle time and thus MIF will be scaled up accordingly anyway.
	 */
	cpu_hz = cyc_per_cntpct_to_hz(stat->cpu_cyc, stat->cntpct);

	/* Now translate the estimation into the closest actual CPU frequency */
	cpu_khz = find_cpu_freq(pol, cpu_hz / HZ_PER_KHZ, CPUFREQ_RELATION_L);

	/*
	 * Vote for the lowest MIF (and DSU) frequencies when the CPU is running
	 * at its lowest frequency. DSU's lowest frequency is voted for via
	 * skipping the dsu_vote update at the end by just returning now.
	 */
	if (cpu_khz == pol->cpuinfo.min_freq)
		return mif->nr_freqs - 1;

	/* Calculate the Hz lost to memory stalls */
	mem_hz = cyc_per_cntpct_to_hz(stat->mem_cyc, stat->cntpct);

	/* Estimate the CPU's new kHz for a given MIF frequency index */
#define est_cpu_khz(idx, r) \
	find_cpu_freq(pol, ((mem_hz * mif->tbl[cur] / mif->tbl[idx]) +	\
			    cpu_hz - mem_hz) / HZ_PER_KHZ,		\
		      CPUFREQ_RELATION_##r)
	/*
	 * Check if it's possible to change the CPU's frequency by changing MIF
	 * frequency, which would result in fewer or greater stalled cycles.
	 * This is done by estimating what the CPU's frequency would be if the
	 * number of stalled cycles changed with a different MIF frequency.
	 */
	if (cur > 0 && est_cpu_khz(0, L) < cpu_khz) {
		/*
		 * CPU frequency can be lowered by increasing MIF frequency, so
		 * find the lowest MIF frequency needed to do such. This is done
		 * by iterating in ascending order of MIF frequency until the
		 * estimated CPU frequency is less than the current frequency.
		 * The closest frequency (CPUFREQ_RELATION_C) is used for the
		 * estimation in order to avoid splitting hairs and voting for a
		 * really high MIF frequency when it's not necessary.
		 */
		for (vote = cur - 1; vote > 0; vote--) {
			if (est_cpu_khz(vote, C) < cpu_khz)
				break;
		}
	} else if (cur < mif->nr_freqs - 1 &&
		   est_cpu_khz(mif->nr_freqs - 1, L) > cpu_khz) {
		/*
		 * Changing the MIF frequency could possibly raise the CPU's
		 * frequency, so find the lowest MIF frequency that won't do
		 * such. This is done by iterating in descending order of MIF
		 * frequency until the estimated CPU frequency is greater than
		 * the current CPU frequency. The upper bound frequency
		 * (CPUFREQ_RELATION_L) is used for the estimation in order to
		 * avoid voting for a MIF frequency that's too low and thus too
		 * close to causing the CPU's frequency to increase.
		 */
		for (vote = cur + 1; vote < mif->nr_freqs - 1; vote++) {
			if (est_cpu_khz(vote, L) > cpu_khz) {
				vote--;
				break;
			}
		}
	} else if (cpu_khz < pol->cpuinfo.max_freq) {
		/*
		 * The current workload is likely memory invariant, and
		 * therefore shouldn't be affected very much by a change in MIF
		 * frequency. Gracefully lower the MIF frequency.
		 */
		if (cur == mif->nr_freqs - 1) {
			/* Keep using the lowest MIF frequency */
			vote = cur;
		} else {
			vote = mif->tbl[cur] * MEMPERFD_DOWN_PCT / 100;
			vote = find_index_l(mif, vote);
			if (vote == cur)
				/* Drop MIF frequency by at least one step */
				vote++;
		}
	} else {
		/*
		 * The CPU is at its maximum frequency and isn't influenced by
		 * changes in MIF frequency, likely because the CPU's frequency
		 * cannot go any higher. Use the highest MIF frequency.
		 */
		vote = 0;
	}
#undef est_cpu_khz

#ifdef CONFIG_SOC_ZUMA
	/*
	 * Set DSU frequency to a factor of the highest CPU frequency, rounded
	 * up. The DSU technical reference manual from Arm recommends setting
	 * SCLK anywhere between 1:2 and 1:1 to the highest CPU frequency.
	 * Setting it too low increases CPU power consumption due to the CPU
	 * needing to use higher frequencies when a bump in DSU frequency
	 * could've sufficed, while setting it too high results in heavy DSU
	 * power consumption.
	 */
	*dsu_vote = max_t(u32, *dsu_vote, cpu_hz / dsu_scale_factor);
#endif

	return vote;
}

/* Performance Profiling Counter (PPC) magic from gs-ppc.c */
static u32 mif_ppc_vote(u32 cur_mif_khz, u32 *bus2_mif)
{
	struct mif_um_ppc *um = &mif_um;
	u32 h_vote = 0, vote;
	int i, j;

	/* Stop all of the PPCs first */
	ppc_write_regs(ppc_stop_cmd, ARRAY_SIZE(ppc_stop_cmd));

	/* Iterate over each group and its counters */
	for (i = 0; i < um->grp_cnt; i++) {
		struct um_group *ug = &um->grp[i];
		u32 ccnt, h_pmcnt1 = 0, pmcnt1;
		int h;

		/* Set this group's vote to zero in case there's no BUS2 vote */
		vote = 0;

		/* Find the highest PMCNT1 (busy time) among the group's PPCs */
		for (j = 0; j < ug->cnt; j++) {
			pmcnt1 = __raw_readl(ug->va_base[j] + PPC_PMCNT1);
			if (pmcnt1 > h_pmcnt1) {
				h_pmcnt1 = pmcnt1;
				h = j;
			}
		}

		/* Check if there was any busy time at all */
		if (!h_pmcnt1)
			continue;

		/* Find the total time (CCNT) for the highest busy-time PPC */
		ccnt = __raw_readl(ug->va_base[h] + PPC_CCNT);
		if (!ccnt)
			continue;

		/*
		 * Find the highest computed MIF frequency among the groups.
		 * This formula comes from governor_simpleinteractive.c.
		 */
		vote = (u64)cur_mif_khz * h_pmcnt1 * 1000 /
		       ((u64)ccnt * ug->target_load);
		if (vote > h_vote)
			h_vote = vote;
	}

	/* Reset and restart all of the PPCs */
	ppc_write_regs(ppc_start_cmd, ARRAY_SIZE(ppc_start_cmd));

	/* Only the last group's (BUS2) MIF vote is used to set the INT vote */
	*bus2_mif = vote;

	/* Return the highest MIF frequency vote */
	return h_vote;
}

/* Returns true if this device isn't voted to its lowest frequency */
static bool memperf_set_vote(struct exynos_devfreq_data *data, u32 new)
{
	update_qos_req(&data->min_req, new);
	return data->min_req.node.prio > data->tbl[data->nr_freqs - 1];
}

/* Returns true if memperfd should arm a timeout to vote down upon inactivity */
static bool memperf_work(void)
{
	u32 vote = mif->nr_freqs - 1, cur, dsu_vote = 0, bus2_mif;
	bool ret = false;
	cpumask_t cpus;
	int cpu;

	/* Only consider active CPUs */
	cpumask_copy(&cpus, cpu_active_mask);

	/* Get the current MIF freq, since something else could've raised it */
	cur = find_index_l(mif, READ_ONCE(mif->cur_freq));

	/* Gather updated memory stall statistics for all active CPUs */
	for_each_cpu(cpu, &cpus) {
		struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
		struct pmu_stat stat;
		u64 delta_cntpct;

		/* Calculate the delta for each statistic */
		raw_spin_lock_irq(&pmu->lock);
		delta_cntpct = pmu->cur.cntpct - pmu->prev.cntpct;
		if (delta_cntpct >= cpu_min_sample_cntpct) {
			stat.mem_cyc = pmu->cur.mem_cyc - pmu->prev.mem_cyc;
			stat.cpu_cyc = pmu->cur.cpu_cyc - pmu->prev.cpu_cyc;
			stat.cntpct = delta_cntpct;
		} else {
			/* Indicate that this CPU should be skipped */
			stat.cpu_cyc = 0;
		}
		raw_spin_unlock_irq(&pmu->lock);

		/*
		 * Skip CPUs with incomplete statistics, like CPUs that have
		 * been idle for a while and thus have had their tick suspended.
		 */
		if (!stat.cpu_cyc || !stat.mem_cyc || !stat.cntpct)
			continue;

		/* Find the highest MIF freq step required among all the CPUs */
		vote = min(vote, mif_cpu_vote(&stat, cpu, cur, &dsu_vote));
	}

	/* Find the highest PPC MIF vote and set the higher of the MIF votes */
	vote = max((u32)mif->tbl[vote], mif_ppc_vote(mif->tbl[cur], &bus2_mif));
	ret |= memperf_set_vote(mif, vote);

	/* Set the new INT vote using BUS2's MIF requirement */
	for (vote = mif_int_cnt - 1; vote > 0; vote--) {
		if (bus2_mif <= mif_int_map[vote].mif_freq)
			break;
	}
	ret |= memperf_set_vote(&df_data[INT], mif_int_map[vote].int_freq);

#ifdef CONFIG_SOC_ZUMA
	/* Set the new DSU vote */
	ret |= memperf_set_vote(dsu, dsu_vote);
#endif

	/*
	 * Reset the statistics for all CPUs by setting the start of the next
	 * sample window to the current counter values.
	 */
	for_each_cpu(cpu, &cpus) {
		struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);

		raw_spin_lock_irq(&pmu->lock);
		pmu->prev = pmu->cur;
		raw_spin_unlock_irq(&pmu->lock);
	}

	return ret;
}

static void memperfd_timeout(struct timer_list *t)
{
	/*
	 * Wake up memperfd so it can vote down to the lowest state. This is
	 * done in order to prevent MIF from staying at a higher frequency than
	 * necessary and never getting a chance to vote down just because there
	 * aren't any scheduler ticks, which is how memperfd is normally driven.
	 */
	kick_memperfd();
}

static void memperfd_wait_timeout(void)
{
	struct timer_list timer;

	/*
	 * Open code freezable_schedule_timeout_interruptible() in order to
	 * make the timer deferrable, so that it doesn't kick CPUs out of idle.
	 * Also, add the timer onto CPU0 since it's usually the least idle.
	 */
	timer_setup_on_stack(&timer, memperfd_timeout, TIMER_DEFERRABLE);
	timer.expires = jiffies + MEMPERFD_POLL_HZ + 1;
	add_timer_on(&timer, 0);
	schedule();
	del_singleshot_timer_sync(&timer);
	destroy_timer_on_stack(&timer);
}

static void memperfd_wait_for_kick(bool timeout)
{
	unsigned long prev_jiffies = jiffies;
	DECLARE_SWAITQUEUE(wait);

	/*
	 * Reset last_run_jiffies to now. prepare_to_swait_exclusive() executes
	 * a barrier that ensures the last_run_jiffies store is ordered before
	 * kick_memperfd() can observe `swait_active() == true`.
	 */
	atomic_long_set(&last_run_jiffies, prev_jiffies);
	while (1) {
		prepare_to_swait_exclusive(&memperfd_waitq, &wait,
					   TASK_IDLE | TASK_FREEZABLE);
		if (atomic_long_read(&last_run_jiffies) != prev_jiffies)
			break;
		if (timeout)
			memperfd_wait_timeout();
		else
			schedule();
	}
	finish_swait(&memperfd_waitq, &wait);
}

static int __noreturn memperf_thread(void *data)
{
	sched_set_fifo(current);
	memperfd_init();
	set_freezable();
	while (1)
		memperfd_wait_for_kick(memperf_work());
}

static void exynos_qos_notify(struct exynos_devfreq_data *data)
{
	u32 freq;

	/* Set the frequency to the floor of the current limits */
	mutex_lock(&data->nb_lock);
	freq = min(data->min_freq, data->max_freq);
	if (freq != data->cur_freq) {
		/* Pairs with memperfd and exynos_df_get_cur_freq() */
		WRITE_ONCE(data->cur_freq, freq);
		if (!data->suspended)
			cal_dfs_set_rate(data->dfs_id, freq);

#ifdef CONFIG_SOC_ZUMA
		/* Set BCI frequency 1:1 to DSU frequency */
		if (data == dsu)
			update_qos_req(&bci->min_req, find_freq_c(bci, freq));
#endif
	}
	mutex_unlock(&data->nb_lock);
}

static int exynos_qos_min_notifier(struct notifier_block *nb,
				   unsigned long value, void *unused)
{
	struct exynos_devfreq_data *data =
		container_of(nb, typeof(*data), min_nb);
	u32 freq = find_freq_l(data, value);

	mutex_lock(&data->min_nb_lock);
	data->min_freq = freq;
	exynos_qos_notify(data);
	mutex_unlock(&data->min_nb_lock);
	return NOTIFY_OK;
}

static int exynos_qos_max_notifier(struct notifier_block *nb,
				   unsigned long value, void *unused)
{
	struct exynos_devfreq_data *data =
		container_of(nb, typeof(*data), max_nb);
	u32 freq = find_freq_h(data, value);

	mutex_lock(&data->max_nb_lock);
	data->max_freq = freq;
	exynos_qos_notify(data);
	mutex_unlock(&data->max_nb_lock);
	return NOTIFY_OK;
}

static int exynos_df_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);

	*freq = READ_ONCE(data->cur_freq);
	return 0;
}

static int exynos_df_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	struct devfreq *df = data->df;
	u32 min, max;

	/* devfreq->profile->target() can be called in devfreq_add_device() */
	if (unlikely(!df))
		return 0;

	/* Update the user requested frequency limits */
	min = dev_pm_qos_read_value(df->dev.parent, DEV_PM_QOS_MIN_FREQUENCY);
	min = find_freq_l(data, min * HZ_PER_KHZ);
	update_qos_req(&data->umin_req, min);

	max = dev_pm_qos_read_value(df->dev.parent, DEV_PM_QOS_MAX_FREQUENCY);
	max = find_freq_h(data, max * HZ_PER_KHZ);
	update_qos_req(&data->umax_req, max);

	*freq = clamp_t(u32, *freq, min, max);
	return 0;
}

static int exynos_acpm_pm(struct exynos_devfreq_data *data, bool resume)
{
	/* ACPM IPC command magic numbers pulled from gs-devfreq.c */
	unsigned int cmd[4] = { data->devfreq_type, resume, 5, 2 };
	struct ipc_config config = { .cmd = cmd, .response = true };

	return acpm_ipc_send_data(data->ipc_chan_id, &config);
}

static int exynos_devfreq_pm(struct device *dev, bool resume)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret = 0;

	mutex_lock(&data->nb_lock);
	if (data->use_acpm) {
		ret = exynos_acpm_pm(data, resume);
		if (WARN_ON(ret))
			goto unlock;
	}

	if (data == mif) {
		if (resume)
			ppc_write_regs(ppc_init_cmd, ARRAY_SIZE(ppc_init_cmd));
		else
			ppc_write_regs(ppc_exit_cmd, ARRAY_SIZE(ppc_exit_cmd));
	}

	if (data->cur_freq != data->suspend_freq)
		cal_dfs_set_rate(data->dfs_id,
				 resume ? data->cur_freq : data->suspend_freq);
	data->suspended = !resume;
unlock:
	mutex_unlock(&data->nb_lock);
	return ret;
}

static int exynos_devfreq_suspend(struct device *dev)
{
	return exynos_devfreq_pm(dev, false);
}

static int exynos_devfreq_resume(struct device *dev)
{
	return exynos_devfreq_pm(dev, true);
}

static int exynos_devfreq_parse_ect(struct device *dev,
				    struct exynos_devfreq_data *data,
				    const char *dvfs_domain_name)
{
	struct ect_dvfs_domain *dvfs_domain;
	void *dvfs_block;
	int i;

	dvfs_block = ect_get_block(BLOCK_DVFS);
	if (!dvfs_block)
		return -ENODEV;

	dvfs_domain = ect_dvfs_get_domain(dvfs_block, (char *)dvfs_domain_name);
	if (!dvfs_domain)
		return -ENODEV;

	data->tbl = kcalloc(dvfs_domain->num_of_level, sizeof(*data->tbl),
			    GFP_KERNEL);
	if (!data->tbl)
		return -ENOMEM;

	for (i = 0; i < dvfs_domain->num_of_level; i++) {
		u32 freq = dvfs_domain->list_level[i].level;

		if (freq > data->max_freq)
			continue;

		if (freq < data->min_freq)
			break;

		data->tbl[data->nr_freqs++] = freq;
		dev_pm_opp_add(dev, freq, 0);
	}

	data->min_freq = data->tbl[data->nr_freqs - 1];
	data->max_freq = data->tbl[0];
	return 0;
}

static enum exynos_dev dfs_id_to_dev(unsigned int dfs_id)
{
#define CASE(dev) case ACPM_DVFS_##dev: return dev
	switch (dfs_id) {
#if defined(CONFIG_SOC_GS101) || defined(CONFIG_SOC_GS201)
	CASE(BO);
#else
	CASE(BCI);
	CASE(BW);
	CASE(DSU);
#endif
	CASE(CAM);
	CASE(DISP);
	CASE(INT);
	CASE(INTCAM);
	CASE(MFC);
	CASE(MIF);
	CASE(TNR);
	}
#undef CASE

	return MAX_DEV;
}

static bool cache_cpu_policy(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *cached = &per_cpu(cached_pol, cpu);

		/*
		 * Cache the CPU policy the first time it's read in order to use
		 * it for looking up invariant information later, like the
		 * frequency table and minimum/maximum limits.
		 */
		if (!cached->freq_table && cpufreq_get_policy(cached, cpu))
			return false;
	}

	return true;
}

static int exynos_ppc_init(struct device *dev, struct device_node *np)
{
	struct mif_um_ppc *um = &mif_um;
	int i, j, k = 0, ret = -ENOMEM;

	um->grp_cnt = of_property_count_u32_elems(np, "um_count");
	if (um->grp_cnt <= 0) {
		dev_err(dev, "No PPC list found\n");
		return -EINVAL;
	}

	um->grp = kmalloc_array(um->grp_cnt, sizeof(*um->grp), GFP_KERNEL);
	if (!um->grp)
		return -ENOMEM;

	for (i = 0; i < um->grp_cnt; i++) {
		struct um_group *ug = &um->grp[i];
		u32 pa;

		if (of_property_read_u32_index(np, "um_count", i, &ug->cnt) ||
		    of_property_read_u32_index(np, "target_load", i,
					       &ug->target_load))
			goto prop_error;

		ug->va_base = kmalloc_array(ug->cnt, sizeof(*ug->va_base),
					    GFP_KERNEL);
		if (!ug->va_base)
			goto free_mem;

		for (j = 0; j < ug->cnt; j++) {
			of_property_read_u32_index(np, "um_list", k++, &pa);
			ug->va_base[j] = ioremap(pa, SZ_4K);
		}
	}

	return 0;

prop_error:
	ret = -ENODEV;
free_mem:
	while (i--)
		kfree(um->grp[i].va_base);
	kfree(um->grp);
	return ret;
}

static int exynos_devfreq_probe(struct platform_device *pdev)
{
	struct exynos_devfreq_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const char *domain_name, *use_acpm;
	enum exynos_dev edev;
	unsigned int dfs_id;
	u32 freq_cfg[5];
	int ret;

	if (!cache_cpu_policy())
		return -EPROBE_DEFER;

	if (of_property_read_u32(np, "dfs_id", &dfs_id))
		return -ENODEV;

	edev = dfs_id_to_dev(dfs_id);
	if (edev == MAX_DEV)
		return -ENODEV;

	data = &df_data[edev];
	if (WARN_ON(data->tbl))
		return -ENODEV;

#ifdef CONFIG_SOC_ZUMA
	/*
	 * DSU depends on BCI, so BCI must be initialized first. And memperfd
	 * depends on DSU, so DSU must be initialized before MIF.
	 */
	if ((data == dsu && !bci->tbl) || (data == mif && !dsu->tbl))
		return -EPROBE_DEFER;

	if (data == mif) {
		unsigned int cpu, cmax = 0, dmax;

		/* Get the highest possible CPU frequency */
		for_each_possible_cpu(cpu) {
			struct cpufreq_policy *pol = &per_cpu(cached_pol, cpu);

			cmax = max(cmax, pol->cpuinfo.max_freq);
		}

		/* Get the 2nd to highest DSU frequency */
		dmax = dsu->tbl[1];

		/*
		 * Set the scale factor for determining DSU frequency from CPU
		 * frequency, while making sure it's possible for the highest
		 * possible CPU frequency to use the highest DSU frequency.
		 */
		dsu_scale_factor = (cmax * 1000 / dmax) - 1;
	}
#endif

	/* memperfd sets INT, so INT must be initialized before MIF */
	if (data == mif && !df_data[INT].tbl)
		return -EPROBE_DEFER;

	if (of_property_read_u32(np, "pm_qos_class", &data->qmin) ||
	    of_property_read_u32(np, "pm_qos_class_max", &data->qmax) ||
	    of_property_read_string(np, "devfreq_domain_name", &domain_name) ||
	    of_property_read_u32_array(np, "freq_info", freq_cfg, 5))
		return -ENODEV;

	data->min_freq = freq_cfg[3];
	data->max_freq = freq_cfg[4];
	ret = exynos_devfreq_parse_ect(dev, data, domain_name);
	if (ret)
		return -EPROBE_DEFER;

	data->dfs_id = dfs_id;
	data->suspend_freq = clamp(freq_cfg[2], data->min_freq, data->max_freq);
	data->use_acpm = !of_property_read_string(np, "use_acpm", &use_acpm) &&
			 !strcmp(use_acpm, "true");
	if (data->use_acpm &&
	    (of_property_read_u32(np, "devfreq_type", &data->devfreq_type) ||
	     of_property_read_u32(np, "acpm-ipc-channel", &data->ipc_chan_id)))
		return -ENODEV;

	mutex_init(&data->min_nb_lock);
	mutex_init(&data->max_nb_lock);
	mutex_init(&data->nb_lock);
	platform_set_drvdata(pdev, data);

	/* Add notifiers to propagate frequency updates to hardware */
	data->min_nb.notifier_call = exynos_qos_min_notifier;
	ret = exynos_pm_qos_add_notifier(data->qmin, &data->min_nb);
	if (ret)
		goto free_tbl;

	data->max_nb.notifier_call = exynos_qos_max_notifier;
	ret = exynos_pm_qos_add_notifier(data->qmax, &data->max_nb);
	if (ret)
		goto del_min_nb;

	/*
	 * Set the initial request after the notifier is added, so the notifier
	 * can propagate the request to hardware.
	 */
	exynos_pm_qos_add_request(&data->min_req, data->qmin, data->min_freq);

	/* Additional initialization specific to MIF */
	if (data == mif) {
		ret = of_property_count_u32_elems(np, "mif_int_map");
		if (ret <= 0 || ret % 2) {
			dev_err(dev, "No valid mif_int_map available\n");
			ret = -EINVAL;
			goto remove_req;
		}

		mif_int_map = kmalloc_array(ret, sizeof(u32), GFP_KERNEL);
		if (!mif_int_map) {
			ret = -ENOMEM;
			goto remove_req;
		}

		if (of_property_read_u32_array(np, "mif_int_map",
					       (u32 *)mif_int_map, ret)) {
			ret = -ENODEV;
			goto free_int_map;
		}

		mif_int_cnt = ret / 2;

		ret = exynos_ppc_init(dev, np);
		if (ret)
			goto free_int_map;

		BUG_ON(IS_ERR(kthread_run(memperf_thread, NULL, "memperfd")));
	}

	/* Initialize devfreq for this device for userspace limits control */
	data->profile.initial_freq = data->min_freq;
	data->profile.freq_table = data->tbl;
	data->profile.max_state = data->nr_freqs;
	data->profile.get_cur_freq = exynos_df_get_cur_freq;
	data->profile.target = exynos_df_target;

	exynos_pm_qos_add_request(&data->umin_req, data->qmin, data->min_freq);
	exynos_pm_qos_add_request(&data->umax_req, data->qmax, data->max_freq);
	data->df = devfreq_add_device(dev, &data->profile,
				      DEVFREQ_GOV_TENSOR_AIO, NULL);
	if (IS_ERR(data->df))
		dev_err(dev, "Failed to add devfreq, ret: %ld\n",
			PTR_ERR(data->df));

	dev_info(dev, "Registered device successfully\n");
	return 0;

free_int_map:
	kfree(mif_int_map);
remove_req:
	exynos_pm_qos_remove_request(&data->min_req);
	exynos_pm_qos_remove_notifier(data->qmax, &data->max_nb);
del_min_nb:
	exynos_pm_qos_remove_notifier(data->qmin, &data->min_nb);
free_tbl:
	kfree(data->tbl);
	return ret;
}

static const struct of_device_id exynos_devfreq_match[] = {
	{ .compatible = "samsung,exynos-devfreq" },
	{}
};

static const struct dev_pm_ops exynos_devfreq_pm_ops = {
	.suspend_late = exynos_devfreq_suspend,
	.resume_early = exynos_devfreq_resume
};

static struct platform_driver exynos_devfreq_driver = {
	.probe = exynos_devfreq_probe,
	.driver = {
		.name = "exynos-devfreq-driver",
		.pm = &exynos_devfreq_pm_ops,
		.of_match_table = exynos_devfreq_match
	}
};

static int exynos_devfreq_root_probe(struct platform_device *pdev)
{
	/* Probe each devfreq driver node under the root node */
	return of_platform_populate(pdev->dev.of_node, NULL, NULL, NULL);
}

static const struct of_device_id exynos_devfreq_root_match[] = {
	{ .compatible = "samsung,exynos-devfreq-root" },
	{}
};

static struct platform_driver exynos_devfreq_root_driver = {
	.probe = exynos_devfreq_root_probe,
	.driver = {
		.name = "exynos-devfreq-root",
		.of_match_table = exynos_devfreq_root_match
	}
};

static int memperf_reboot(struct notifier_block *notifier, unsigned long val,
			  void *cmd)
{
	/*
	 * Unregister tensor_aio_tick() on all CPUs in order to prevent PMU
	 * register access after kvm_reboot() runs. PMU registers must not be
	 * accessed after kvm_reboot() finishes; attempting to do so will fault.
	 *
	 * This also needs to kick all CPUs to ensure that the cpuidle hooks
	 * aren't running anymore. kick_all_cpus_sync() executes a full memory
	 * barrier before kicking all CPUs; after it finishes, it's guaranteed
	 * that the cpuidle hooks will observe `is_reboot == true` and thus
	 * won't attempt to read PMU registers anymore.
	 */
	in_reboot = true;
	kick_all_cpus_sync();
	topology_clear_scale_freq_source(SCALE_FREQ_SOURCE_ARCH,
					 cpu_possible_mask);
	cpuhp_remove_state_nocalls(cpuhp_state);
	return NOTIFY_OK;
}

/* Use the highest priority in order to run before kvm_reboot() */
static struct notifier_block memperf_reboot_nb = {
	.notifier_call = memperf_reboot,
	.priority = INT_MAX
};

static int devfreq_tensor_aio_target(struct devfreq *df, unsigned long *freq)
{
	*freq = DEVFREQ_MIN_FREQ;
	return 0;
}

static int devfreq_tensor_aio_handler(struct devfreq *df, unsigned int event,
				      void *data)
{
	return 0;
}

/* Dummy devfreq governor to prevent any other governor from being used */
static struct devfreq_governor devfreq_tensor_aio = {
	.name = DEVFREQ_GOV_TENSOR_AIO,
	.flags = DEVFREQ_GOV_FLAG_IMMUTABLE | DEVFREQ_GOV_FLAG_IRQ_DRIVEN,
	.get_target_freq = devfreq_tensor_aio_target,
	.event_handler = devfreq_tensor_aio_handler
};

static int __init exynos_devfreq_driver_init(void)
{
	int ret;

	ret = devfreq_add_governor(&devfreq_tensor_aio);
	if (WARN_ON(ret))
		return ret;

	ret = platform_driver_register(&exynos_devfreq_driver);
	if (WARN_ON(ret))
		goto remove_gov;

	ret = platform_driver_register(&exynos_devfreq_root_driver);
	if (WARN_ON(ret))
		goto unregister_df;

	ret = register_reboot_notifier(&memperf_reboot_nb);
	if (WARN_ON(ret))
		goto unregister_df_root;

	return 0;

unregister_df_root:
	platform_driver_unregister(&exynos_devfreq_root_driver);
unregister_df:
	platform_driver_unregister(&exynos_devfreq_driver);
remove_gov:
	devfreq_remove_governor(&devfreq_tensor_aio);
	return ret;
}
device_initcall(exynos_devfreq_driver_init);
