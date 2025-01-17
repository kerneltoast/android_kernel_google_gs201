// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024-2025 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

#include <linux/cpufreq.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/of_platform.h>
#include <linux/perf/arm_pmuv3.h>
#include <linux/reboot.h>
#include <linux/rtmutex.h>
#include <linux/units.h>
#include <linux/sched/cputime.h>
#include <cpufreq/exynos-acme.h>
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
 * The minimum sample time required to measure the performance counters. This
 * should take into account the resolution of the system timer. At Tensor's
 * timer rate of 24576000 Hz, a sample window of 3 us provides an error margin
 * of ~1.4%.
 */
static u64 cpu_min_sample_cntpct __read_mostly = 3 * NSEC_PER_USEC;

/*
 * The maximum amount of time allowed for a CPU frequency ramp up to latch
 * before reporting the entire CPU domain as throttled to the scheduler. This
 * helps recover performance lost due to the scheduler's lack of awareness of
 * varying transition latency, which can exceed 10 ms in some cases.
 */
static u64 cpu_ramp_up_lat_cntpct __read_mostly = 500 * NSEC_PER_USEC;

/*
 * Compare two CPU frequencies to see if they are sufficiently close, within ~5%
 * of each other by default. This mimics capacity_greater() in sched/fair.c,
 * with the intent being that if the real CPU frequency is close enough to the
 * target frequency then there's no need to inform the scheduler about it.
 */
#define cpu_freqs_similar(lower_freq, higher_freq) \
	((u64)(lower_freq) * 1078 >= (u64)(higher_freq) * 1024)

/* The name of our governor exposed to devfreq */
#define DEVFREQ_GOV_TENSOR_AIO "tensor_aio"

/* The PMU/AMU event stats. Order is assumed by the *pmu_read() functions. */
struct pmu_stat {
	u64 cntpct;
	u64 const_cyc;
	u64 cpu_cyc;
	u64 mem_cyc;
};

struct cpu_pmu {
	/*
	 * The cur_ptr array is passed as an argument to cmpxchg_double_local(),
	 * which is implemented with the CASP instruction on AAarch64.
	 *
	 * When FEAT_LSE2 isn't implemented, "The CASP instructions require
	 * alignment to the total size of the memory being accessed."
	 * - 'ARM DDI 0487K.a C3.2.12.4 ("Compare and Swap")'
	 *
	 * When FEAT_LSE2 is implemented, "If all the bytes of the memory access
	 * lie within a 16-byte quantity aligned to 16 bytes and are to Normal
	 * Inner Write-Back, Outer Write-Back Cacheable memory, an unaligned
	 * access is performed." Otherwise, an alignment fault may be generated.
	 * - 'ARM DDI 0487K.a B2.13.2.1.2 ("Load-Exclusive/ Store-Exclusive and
	 *    Atomic instructions")'
	 *
	 * Therefore, the 16-byte cur_ptr array must be aligned to 16 bytes,
	 * since it's a hard requirement for !FEAT_LSE2, and since it's needed
	 * for FEAT_LSE2 in order to avoid generating alignment faults which are
	 * fatal in illegal contexts such as the cpuidle callbacks.
	 */
	struct pmu_stat *cur_ptr[2] __aligned(16);
	struct pmu_stat cur[2];
	struct pmu_stat prev;
	struct sfd_data {
		raw_spinlock_t lock;
		u64 cpu_cyc;
		u64 const_cyc;
		bool stale;
	} sfd; /* Scale Frequency Data */
	struct htd_data {
		u64 start;
		u64 cpu_cyc;
		u64 const_cyc;
	} htd; /* Hardware Throttle Data */
};

static DEFINE_PER_CPU(struct cpu_pmu, cpu_pmu_evs) = {
	.sfd.lock = __RAW_SPIN_LOCK_UNLOCKED(cpu_pmu_evs.sfd.lock)
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

/*
 * A raw spin lock is used for some domains which don't require sleeping for a
 * frequency transition (i.e., to wait for a response from ACPM). This allows
 * exynos_pm_qos_update_request() to be called from atomic contexts for such
 * atomic-capable domains.
 */
union nb_lock_type {
	struct rt_mutex rt_mutex;
	raw_spinlock_t raw_spinlock;
};

struct exynos_devfreq_data {
	struct exynos_pm_qos_request min_req;
	struct exynos_pm_qos_request umin_req;
	struct exynos_pm_qos_request umax_req;
	struct devfreq_dev_profile profile;
	struct devfreq *df;
	struct notifier_block min_nb;
	struct notifier_block max_nb;
	void (*nb_lock_fn)(void *);
	void (*nb_unlock_fn)(void *);
	union nb_lock_type min_nb_lock;
	union nb_lock_type max_nb_lock;
	union nb_lock_type nb_lock;
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

/* Domains which are governed by memperfd, to be voted down upon quiescence */
static struct exynos_devfreq_data *const memperfd_domains[] = {
#ifdef CONFIG_SOC_ZUMA
	&df_data[DSU], /* DSU sets BCI indirectly */
#endif
	&df_data[INT],
	&df_data[MIF]
};

/*
 * Domains governed by memperfd which perform DVFS fast and atomically, so that
 * they can be voted down from atomic context (in schedule()) upon memperfd
 * quiescence.
 */
#ifdef CONFIG_SOC_ZUMA
#define domain_has_fast_dvfs(d) (d == BCI || d == DSU || d == INT || d == MIF)
#else
#define domain_has_fast_dvfs(d) (d == INT || d == MIF)
#endif

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
	void __iomem **all_regs;
	struct um_group *grp;
	u32 all_regs_cnt;
	u32 grp_cnt;
} static mif_um;

static bool memperfd_quiescent;
static unsigned int idle_task_cpus;
static DEFINE_RAW_SPINLOCK(idle_task_lock);
static atomic_long_t last_run_jiffies = ATOMIC_INIT(0);
static DECLARE_SWAIT_QUEUE_HEAD(memperfd_waitq);
static DEFINE_PER_CPU_READ_MOSTLY(struct cpufreq_policy, cached_pol);
static unsigned int dsu_scale_factor __read_mostly __maybe_unused;
static DEFINE_STATIC_KEY_FALSE(system_ready);
static int cpuhp_state;

/* CPUs which have updated statistics available for memperfd to consume */
static atomic_t stats_avail_cpus;
static_assert(NR_CPUS <= sizeof(stats_avail_cpus) * BITS_PER_BYTE);

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
	cpu_ramp_up_lat_cntpct = ns_to_cntpct(cpu_ramp_up_lat_cntpct);
}

enum cpu_throttle_src {
	CPU_CPUFREQ_THROTTLE,
	CPU_HW_THROTTLE,
#ifdef CONFIG_SOC_ZUMA
	CPU_TMU_THROTTLE,
#endif
	MAX_CPU_THROTTLE_SRCS
};

struct throt_data {
	struct list_head node;
	raw_spinlock_t throt_lock;
	raw_spinlock_t idle_cpu_lock;
	struct exynos_cpufreq_domain *domain;
	unsigned int cap[MAX_CPU_THROTTLE_SRCS];
	cpumask_t idle_cpus;
	u64 last_htd_cntpct;
	int cpu;
};

/*
 * domain_throt_data is guaranteed to be initialized for all CPUs before Tensor
 * AIO starts because tensor_aio_init_cpu_domain() is called during cpufreq init
 * in exynos-acme, and the cache_cpu_policy() check in our probe routine forces
 * probing to be deferred until all CPUs are registered with cpufreq.
 */
static DEFINE_PER_CPU_READ_MOSTLY(struct throt_data *, domain_throt_data);
static LIST_HEAD(domain_throt_list);

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

static __always_inline bool cpu_cortex_a55_or_a76(int cpu)
{
	return (IS_ENABLED(CONFIG_SOC_GS201) && cpu < 4) ||
	       (IS_ENABLED(CONFIG_SOC_GS101) && cpu < 6);
}

static __always_inline bool cpu_supports_amu_const(int cpu)
{
	/*
	 * Zumapro supports AMU const cycles on all cores, while
	 * zuma supports AMU const cycles on all but the Cortex-A510 cores due
	 * to ARM erratum 2457168.
	 */
	return IS_ENABLED(CONFIG_SOC_ZUMAPRO) ? true : cpu > 4;
}

static __always_inline unsigned long cpu_pmu_evt_en_mask(int cpu)
{
	/* Zumapro has fully-functioning AMU events */
	if (IS_ENABLED(CONFIG_SOC_ZUMAPRO))
		return 0;

	/*
	 * Zuma can read STALL_BACKEND_MEM via AMU but on the Cortex-X3 core
	 * there is a quirk where the AMU event always returns zero after a
	 * hotplug. This can be worked around by creating a perf event
	 * specifically for the STALL_BACKEND_MEM counter on the Cortex-X3. The
	 * perf event must be created and released across Cortex-X3 hotplugs,
	 * and then the AMU event will work correctly.
	 */
	if (IS_ENABLED(CONFIG_SOC_ZUMA))
		return cpu == 8 ? BIT(STALL_BACKEND_MEM) : 0;

	/* Gsx01 doesn't support AMU events at all */
	return BIT(CPU_CYCLES) | BIT(STALL_BACKEND_MEM);
}

static struct perf_event *create_pev(struct perf_event_attr *attr, int cpu)
{
	return perf_event_create_kernel_counter(attr, cpu, NULL, NULL, NULL);
}

static void release_perf_events(int cpu)
{
	struct cpu_pmu_evt *cpev = &per_cpu(pevt_pcpu, cpu);
	unsigned long en_mask = cpu_pmu_evt_en_mask(cpu);
	int i;

	for_each_set_bit(i, &en_mask, PMU_EVT_MAX) {
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
	unsigned long en_mask = cpu_pmu_evt_en_mask(cpu);
	int i;

	for_each_set_bit(i, &en_mask, PMU_EVT_MAX) {
		/*
		 * Cortex-A55 and Cortex-A76 don't support STALL_BACKEND_MEM, so
		 * use STALL_BACKEND as a poor man's alternative. The value from
		 * STALL_BACKEND is divided by two in gsx01_pmu_read() to get a
		 * close guess of how many stalled cycles are due to memory.
		 */
		if (i == STALL_BACKEND_MEM && cpu_cortex_a55_or_a76(cpu))
			attr.config = ARMV8_PMUV3_PERFCTR_STALL_BACKEND;
		else
			attr.config = pmu_evt_id[i];
		cpev->pev[i] = create_pev(&attr, cpu);
		if (WARN_ON(IS_ERR(cpev->pev[i])))
			goto release_pevs;
	}

	return 0;

release_pevs:
	release_perf_events(cpu);
	return PTR_ERR(cpev->pev[i]);
}

/*
 * These are the optimized functions for reading the PMU/AMU event counters for
 * each supported SoC.
 *
 * The generic timer counter (CNTPCT_EL0) is read directly for the lowest
 * possible latency incurred from reading the current time, as well as the
 * greatest precision since we can convert the number of ticks into nanoseconds
 * without sched_clock()'s approximation that aims to do the conversion as
 * quickly as possible at a loss of precision. The preceeding ISB prevents
 * speculative reads of the counter register, though is unnecessary when
 * CNTPCTSS_EL0 is available (which is indicated at runtime via ARM64_HAS_ECV).
 *
 * Next, the constant cycles counter is read. On CPUs which don't support the
 * AMU constant cycles event, the value from the generic timer counter is used
 * instead. The AMU constant cycles event is useful because it always stops
 * incrementing when the CPU is in WFE/WFI, which can be entered from a number
 * of places in the kernel besides cpuidle, such as __delay(). CPUs which don't
 * support AMU constant cycles only account for WFI from cpuidle; as a result,
 * the CPU frequency calculation for such CPUs may be lower than expected. This
 * is because of unaccounted WFE/WFI activity, since the generic timer counter
 * doesn't stop incrementing in WFE/WFI. This isn't typically a huge problem,
 * but it's worth noting.
 *
 * Then, the CPU cycles and memory stall cycles are read from AMU event counters
 * on CPUs which support AMU. On gsx01 where this isn't the case, these values
 * are read directly from the PMU event counter registers configured by the
 * PMUv3 driver. This is done directly in assembly rather than using the perf
 * event API in order to achieve the best possible accuracy, since any delays
 * between the counter reads injects inaccuracy into later calculations
 * performed on these values.
 *
 * A succeeding ISB ensures all counter register reads are complete before the
 * CPU proceeds. Any instruction reordering within the ISBs is of negligible
 * consequence, so no barriers are used in between the counter reads.
 *
 * These functions must be noinline in order to force the compiler to align them
 * to an L1 cache line. The goal is to fit all of the MRS instructions in each
 * function into the same cache line, to avoid timing discrepancies between one
 * MRS and another. A stall due to a cache line fill to get the next MRS can
 * influence calculations using these readings; e.g., if the current time is
 * read first and then the current number of CPU cycles is read next after a
 * stall due to fetching the instruction from beyond L1, the resulting CPU
 * frequency calculation from these figures would produce a higher result than
 * expected.
 *
 * None of these functions have stack-allocated variables. Therefore, the
 * prologue of each function may consist of up to two instructions: BTI and
 * PACIASP, from CONFIG_ARM64_BTI_KERNEL and CONFIG_ARM64_PTR_AUTH_KERNEL,
 * respectively. This leaves room for six instructions to be guaranteed to fit
 * within the same cache line as the prologue, which is enough to cover all MRS
 * instruction sequences for every SoC, including ISBs.
 *
 * The resulting counter values are stored into a `struct pmu_stat` using a
 * compound literal to encourage the compiler to reorder or coalesce the stores
 * as it sees fit, without being constrained by any explicit store ordering
 * declared at compile time (e.g., `stat->foo = foo; stat->bar = bar; ...`).
 */
static noinline void __aligned(L1_CACHE_BYTES)
zumapro_pmu_read(struct pmu_stat *stat)
{
	register u64 cntpct, const_cyc, cpu_cyc, mem_cyc;

	asm volatile("mrs %0, cntpctss_el0\n\t"
		     "mrs %1, amevcntr01_el0\n\t"
		     "mrs %2, amevcntr00_el0\n\t"
		     "mrs %3, amevcntr03_el0\n\t"
		     "isb"
		     : "=r" (cntpct), "=r" (const_cyc), "=r" (cpu_cyc),
		       "=r" (mem_cyc));

	*stat = (typeof(*stat)){ cntpct, const_cyc, cpu_cyc, mem_cyc };
}

static noinline void __aligned(L1_CACHE_BYTES)
zuma_with_const_pmu_read(struct pmu_stat *stat)
{
	register u64 cntpct, const_cyc, cpu_cyc, mem_cyc;

	asm volatile("isb\n\t"
		     "mrs %0, cntpct_el0\n\t"
		     "mrs %1, amevcntr01_el0\n\t"
		     "mrs %2, amevcntr00_el0\n\t"
		     "mrs %3, amevcntr03_el0\n\t"
		     "isb"
		     : "=r" (cntpct), "=r" (const_cyc), "=r" (cpu_cyc),
		       "=r" (mem_cyc));

	*stat = (typeof(*stat)){ cntpct, const_cyc, cpu_cyc, mem_cyc };
}

static noinline void __aligned(L1_CACHE_BYTES)
zuma_no_const_pmu_read(struct pmu_stat *stat)
{
	register u64 cntpct, cpu_cyc, mem_cyc;

	asm volatile("isb\n\t"
		     "mrs %0, cntpct_el0\n\t"
		     "mrs %1, amevcntr00_el0\n\t"
		     "mrs %2, amevcntr03_el0\n\t"
		     "isb"
		     : "=r" (cntpct), "=r" (cpu_cyc), "=r" (mem_cyc));

	*stat = (typeof(*stat)){ cntpct, cntpct, cpu_cyc, mem_cyc };
}

static noinline void __aligned(L1_CACHE_BYTES)
gsx01_pmu_read(struct pmu_stat *stat)
{
	register u64 cntpct, cpu_cyc, mem_cyc_h, mem_cyc_l;

	/*
	 * Since the perf event is registered on CPU hotplug, we always claim
	 * the first two PMU registers (PMEVCNTR0_EL0 and PMEVCNTR1_EL0), so we
	 * can safely assume these are always the correct registers for the
	 * STALL_BACKEND{_MEM} event. The PMU event count registers are 32 bits
	 * wide on gsx01, so the STALL_BACKEND{_MEM} event ends up being chained
	 * using two PMU registers because create_perf_events() requests a
	 * 64-bit count. The PMUv3 driver configures the PMU to store the lower
	 * 32 bits in PMEVCNTR0_EL0 and the upper 32 bits in PMEVCNTR1_EL0.
	 */
	asm volatile("isb\n\t"
		     "mrs %0, cntpct_el0\n\t"
		     "mrs %1, pmccntr_el0\n\t"
		     "mrs %2, pmevcntr0_el0\n\t"
		     "mrs %3, pmevcntr1_el0\n\t"
		     "isb"
		     : "=r" (cntpct), "=r" (cpu_cyc), "=r" (mem_cyc_l),
		       "=r" (mem_cyc_h));

	/*
	 * Combine the lower and upper bits of STALL_BACKEND{_MEM} together and
	 * then divide it by two for Cortex-A55 and Cortex-A76 since the
	 * STALL_BACKEND event is a superset of STALL_BACKEND_MEM.
	 */
	mem_cyc_l |= mem_cyc_h << 32;
	mem_cyc_l >>= cpu_cortex_a55_or_a76(raw_smp_processor_id());
	*stat = (typeof(*stat)){ cntpct, cntpct, cpu_cyc, mem_cyc_l };
}

static void pmu_get_stats(struct pmu_stat *stat)
{
	/* Read the stats using the matching assembly function */
	if (IS_ENABLED(CONFIG_SOC_ZUMAPRO)) {
		zumapro_pmu_read(stat);
	} else if (IS_ENABLED(CONFIG_SOC_ZUMA)) {
		if (cpu_supports_amu_const(raw_smp_processor_id()))
			zuma_with_const_pmu_read(stat);
		else
			zuma_no_const_pmu_read(stat);
	} else {
		gsx01_pmu_read(stat);
	}
}

/*
 * ARM DDI 0487K.a B2.2.1 ("Requirements for single-copy atomicity"):
 * "Reads that are generated by a Load Pair instruction that loads two
 *  general-purpose registers and are aligned to the size of the load to each
 *  register are treated as two single-copy atomic reads, one for each register
 *  being loaded."
 *
 * So using LDP to read the old values for the cmpxchg_double() loop guarantees
 * that they are _individually_ read without data races. This is *not* the same
 * as one atomic 128-bit load, but rather two atomic 64-bit loads, so it is
 * possible to observe an impossible combination of the two 64-bit quantities.
 */
#define pmu_read_cur_ptrs(pmu, val1, val2) \
	asm volatile("ldp %[v1], %[v2], %[v]"				\
		     : [v1] "=r" (val1), [v2] "=r" (val2)		\
		     : [v] "Q" (*(__uint128_t *)pmu->cur_ptr))

/*
 * Helpers to locklessly grab a pmu_stat pointer and put it back, with or
 * without having updated it. This is implemented by keeping pointers saved for
 * two pmu_stat structs and moving them around with a 128-bit cmpxchg, since
 * there is only one producer (a specific CPU) and one consumer (the memperfd
 * thread), so at most only two pmu_stat copies are needed at any given moment.
 *
 * The goal is to let memperfd always consume the newest pmu_stat data without
 * blocking CPUs from producing new pmu_stat data. Hence the lockless scheme
 * using cmpxchg_double_local(), which is just cmpxchg_double() without any
 * explicit memory ordering semantics.
 *
 * When a CPU updates the pmu_stat with fresh data, a pointer to the newest
 * pmu_stat is kept around for memperfd to read locklessly.
 *
 * pmu_get_cur_reader() gets the newest pmu_stat, which is always stored in
 * pmu->cur_ptr[0]. If it's NULL, then that means the CPU in question is
 * actively getting new data that isn't ready yet, and the old pmu_stat is
 * actually the newest pmu_stat from the previous pmu_get_cur_reader() sequence.
 * When this happens, the "old" pmu_stat located at pmu->cur_ptr[1] is returned.
 *
 * pmu_get_cur_writer() gets the older pmu_stat (pmu->cur_ptr[1]), so that it
 * can be overwritten with new data. If pmu->cur_ptr[1] is NULL then that means
 * memperfd is still busy reading the last pmu_stat and hasn't touched the
 * newest pmu_stat that was generated. When this happens, the newest pmu_stat is
 * returned so that it can be overwritten with even newer data.
 *
 * pmu_put_cur_reader() puts the reader's pmu_stat back into the older pmu_stat
 * slot if the slot is NULL, otherwise it puts it back into pmu->cur_ptr[0]
 * since it is still the newest pmu_stat.
 *
 * pmu_put_cur_writer() always puts the writer's pmu_stat back into the newest
 * pmu_stat slot, since it always has the newest data.
 */
#define __PMU_CMPXCHG_DBL_LOOP(new1_expr, new2_expr) \
	struct pmu_stat *old1, *old2, *new1, *new2;			\
	do {								\
		pmu_read_cur_ptrs(pmu, old1, old2);			\
		new1 = (new1_expr), new2 = (new2_expr);			\
	} while (!cmpxchg_double_local(&pmu->cur_ptr[0],		\
				       &pmu->cur_ptr[1],		\
				       old1, old2, new1, new2))
#define pmu_get_cur_reader(pmu) \
({									\
	__PMU_CMPXCHG_DBL_LOOP(NULL, old1 ? old2 : NULL);		\
	old1 ? old1 : old2;						\
})
#define pmu_get_cur_writer(pmu) \
({									\
	__PMU_CMPXCHG_DBL_LOOP(old2 ? old1 : NULL, NULL);		\
	old2 ? old2 : old1;						\
})
#define pmu_put_cur_reader(pmu, cur) \
({									\
	__PMU_CMPXCHG_DBL_LOOP(old2 ? cur : old1, old2 ? old2 : cur);	\
})
#define pmu_put_cur_writer(pmu, cur) \
({									\
	__PMU_CMPXCHG_DBL_LOOP(cur, old1 ? old1 : old2);		\
})

/*
 * Implement atomic_or() with release semantics, since AAarch64 implements such
 * an instruction with LSE. This is faster than smp_mb__before_atomic() paired
 * with a normal atomic_or() to achieve release semantics.
 */
static __always_inline void atomic_or_release(int i, atomic_t *v)
{
	asm volatile(__LSE_PREAMBLE "stsetl %w[i], %[v]"
		     : [v] "+Q" (v->counter)
		     : [i] "r" (i));
}

static __always_inline void pmu_update_stats(int cpu, struct cpu_pmu *pmu,
					     struct pmu_stat *cur,
					     struct pmu_stat *prev)
{
	bool prev_needs_reset = !(atomic_read(&stats_avail_cpus) & BIT(cpu));
	struct pmu_stat *cur_ptr;

	/* Get the previous stats if requested or memperfd needs them reset */
	if (prev || prev_needs_reset) {
		struct pmu_stat *ptr1, *ptr2;

		/*
		 * Since we only need to read the latest stats, and we are the
		 * only producer of new stats, no synchronization is needed. But
		 * the latest stat pointer may have been taken by memperfd, in
		 * which we can still find it knowing that it's not the pointer
		 * stored in ptr2.
		 */
		pmu_read_cur_ptrs(pmu, ptr1, ptr2);
		if (!ptr1)
			ptr1 = &pmu->cur[!(ptr2 - &pmu->cur[0])];
		if (prev)
			*prev = *ptr1;
		if (prev_needs_reset)
			pmu->prev = *ptr1;
	}

	pmu_get_stats(cur);

	/* Publish the updated stats */
	cur_ptr = pmu_get_cur_writer(pmu);
	*cur_ptr = *cur;
	pmu_put_cur_writer(pmu, cur_ptr);

	/*
	 * Make the updated stats visible to memperfd. Release semantics are
	 * needed to ensure the unordered cmpxchg_double() publishing the stats
	 * occurs before the update to `stats_avail_cpus`.
	 */
	if (prev_needs_reset)
		atomic_or_release(BIT(cpu), &stats_avail_cpus);
}

static void kick_memperfd(void)
{
	unsigned long prev, now = jiffies;

	/* Do not kick memperfd when it's quiescent */
	if (memperfd_quiescent)
		return;

	/* Do not kick memperfd for idle CPUs */
	if (is_idle_task(current))
		return;

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

void tensor_aio_init_cpu_domain(struct exynos_cpufreq_domain *domain)
{
	struct throt_data *t;
	int cpu;

	t = kmalloc(sizeof(*t), GFP_KERNEL | __GFP_NOFAIL);
	memset32(t->cap, UINT_MAX, ARRAY_SIZE(t->cap));
	raw_spin_lock_init(&t->throt_lock);
	raw_spin_lock_init(&t->idle_cpu_lock);
	cpumask_clear(&t->idle_cpus);
	t->cpu = cpumask_first(&domain->cpus);
	t->domain = domain;
	t->last_htd_cntpct = 0;

	/*
	 * Store a pointer to the clock domain's throttle data for each CPU for
	 * quick translation from a CPU number to its domain.
	 */
	for_each_cpu(cpu, &domain->cpus)
		per_cpu(domain_throt_data, cpu) = t;

	/* Make a list so each CPU can refresh the TMU throttle for all CPUs */
	list_add_tail(&t->node, &domain_throt_list);
}

/* Must be called with t->throt_lock held */
static void update_thermal_pressure(struct throt_data *t,
				    enum cpu_throttle_src src, unsigned int cap)
{
	unsigned int capped_freq = UINT_MAX;
	int i;

	/*
	 * Update the thermal pressure for the designated source if it's
	 * different, and then aggregate the thermal pressure applied by all
	 * sources. This updates all CPUs within the same clock domain.
	 */
	if (t->cap[src] == cap)
		return;

	t->cap[src] = cap;
	for (i = 0; i < ARRAY_SIZE(t->cap); i++) {
		if (t->cap[i] < capped_freq)
			capped_freq = t->cap[i];
	}
	arch_update_thermal_pressure(&t->domain->cpus, capped_freq);
}

void tensor_aio_cpufreq_pressure(int cpu, unsigned int cap)
{
	struct throt_data *t = per_cpu(domain_throt_data, cpu);
	unsigned long flags;

	/* Update the throttle set via cpufreq policy (e.g., via sysfs) */
	raw_spin_lock_irqsave(&t->throt_lock, flags);
	update_thermal_pressure(t, CPU_CPUFREQ_THROTTLE, cap);
	raw_spin_unlock_irqrestore(&t->throt_lock, flags);
}

static void reset_htd_data(struct htd_data *htd)
{
	htd->cpu_cyc = htd->const_cyc = htd->start = 0;
}

static void add_htd_data(struct htd_data *htd, const struct pmu_stat *cur,
			 const struct pmu_stat *prev)
{
	/* Record the starting time of this sample window */
	if (!htd->start)
		htd->start = cur->cntpct;

	/* Accumulate data for calculating the CPU's frequency */
	htd->cpu_cyc += cur->cpu_cyc - prev->cpu_cyc;
	htd->const_cyc += cur->const_cyc - prev->const_cyc;
}

#ifdef CONFIG_SOC_ZUMA
/* Must be called with t->throt_lock held */
static void update_tmu_throttle(struct throt_data *t)
{
	unsigned int gs_tmu_throt_freq(int cpu);
	unsigned int freq = gs_tmu_throt_freq(t->cpu);
	struct cpufreq_policy *pol = &per_cpu(cached_pol, t->cpu);

	/*
	 * Get the throttle reported by TMU under the domain lock to report the
	 * latest TMU throttle for all CPUs in the domain. Let freq be UINT_MAX
	 * when there's no throttling at all.
	 */
	if (freq >= pol->cpuinfo.max_freq)
		freq = UINT_MAX;

	update_thermal_pressure(t, CPU_TMU_THROTTLE, freq);
}

static void update_tmu_throttle_all(void)
{
	struct throt_data *t;

	/*
	 * Update the TMU throttle for all CPU domains. This is helpful for when
	 * all CPUs in a domain are idle and thus cannot do the update
	 * themselves, which can potentially leave the entire domain with a
	 * stale thermal pressure. As a result, the scheduler may not prefer
	 * waking such CPUs out of idle if it sees that they're throttled.
	 */
	list_for_each_entry(t, &domain_throt_list, node) {
		raw_spin_lock(&t->throt_lock);
		update_tmu_throttle(t);
		raw_spin_unlock(&t->throt_lock);
	}
}
#endif

static void update_cpu_hw_throttle(void)
{
	int cpu = raw_smp_processor_id();
	struct throt_data *t = per_cpu(domain_throt_data, cpu);
	struct cpufreq_policy *pol = &per_cpu(cached_pol, cpu);
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	struct htd_data *htd = &pmu->htd;
	struct exynos_acme_rate rate_info;
	u64 freq, max_freq, ns;

	/*
	 * Check that enough time has passed to measure the CPU's frequency. If
	 * not, it means that the CPU spent so much time idle during this jiffy
	 * that checking for hardware throttling is pointless; as such, the
	 * stats should be reset so that stale data is not carried forward.
	 */
	if (htd->const_cyc < cpu_min_sample_cntpct)
		goto reset_stats;

	/* Calculate the measured frequency */
	max_freq = pol->cpuinfo.max_freq;
	ns = cntpct_to_ns(htd->const_cyc);
	freq = min(max_freq, USEC_PER_SEC * htd->cpu_cyc / ns);

	/*
	 * It may take a while for a CPU frequency change to latch, or the
	 * hardware may have other intentions and opaquely refuse to switch a
	 * CPU domain to the governor's desired target frequency due to hardware
	 * throttling. This is evident by observing the real CPU frequency
	 * measured via the cycle counter: sometimes it takes several
	 * milliseconds for a frequency switch to latch, while other times under
	 * heavy load the target frequency won't latch indefinitely due to
	 * hardware throttling.
	 *
	 * Although the TMU does report its current throttle for a CPU domain,
	 * there is still some hardware throttle mechanism which isn't reported
	 * by the TMU. This unknown hardware throttling can only be detected by
	 * comparing the real CPU frequency to the CPU's target frequency.
	 *
	 * Therefore, when a CPU frequency switch to a higher frequency exceeds
	 * a specified latency threshold, tell the scheduler to assume that the
	 * respective CPU domain is throttled to the actual measured frequency.
	 * This helps mitigate misguided scheduling decisions from hurting
	 * performance, since the scheduler would be otherwise unaware that a
	 * CPU domain is throttled.
	 */
	exynos_acme_rate_info(t->domain, &rate_info);

	/*
	 * Assume the raw measured frequency is the same as the set frequency
	 * if they are either sufficiently close or the measured frequency is
	 * greater than the set frequency. Otherwide, round the measured
	 * frequency to the closest frequency step.
	 */
	if (cpu_freqs_similar(freq, rate_info.freq))
		freq = rate_info.freq;
	else
		freq = find_cpu_freq(pol, freq, CPUFREQ_RELATION_C);

	if (freq < rate_info.freq) {
		/*
		 * If the measured frequency is below the target frequency, it
		 * can be due to two reasons: unknown hardware throttling or
		 * high transition latency to the requested frequency. In the
		 * case of high transition latency, give the transition at least
		 * cpu_ramp_up_lat_cntpct timer ticks to latch before telling
		 * the scheduler that this CPU domain is throttled. This is done
		 * by blocking new throttle reports for sample windows that are
		 * older than the frequency latch deadline.
		 *
		 * As for unknown hardware throttling: the CPU is throttled
		 * extremely often depending on the instructions executed,
		 * possibly due to MPMM (Maximum Power Mitigation Mechanism).
		 * This throttling isn't reflected by the TMU's shared ACPM
		 * data, so the only way to detect it is by measuring the CPU's
		 * real frequency.
		 *
		 * Therefore, a measured frequency below the latched target
		 * frequency is reported to the scheduler as a throttle, which
		 * neatly covers slow frequency transitions unknown hardware
		 * throttling.
		 */
		if (htd->start < rate_info.set_time + cpu_ramp_up_lat_cntpct)
			goto reset_stats;
	} else {
		/* Reject sample windows older than the last rate switch */
		if (htd->start < rate_info.set_time)
			goto reset_stats;

		/*
		 * Notify exynos-acme that the requested rate is now "latched";
		 * i.e., that the measured frequency is either at or above the
		 * target frequency. This is done even if the measurement is
		 * higher than the target because we only care about knowing
		 * when the CPU is throttled.
		 */
		if (rate_info.set_time)
			exynos_acme_rate_latched(t->domain, &rate_info);

		/* Indicate there's no hardware throttle detected */
		freq = UINT_MAX;
	}

	/*
	 * Report the throttle detected by measuring the real frequency, unless
	 * there's a newer frequency measurement from another CPU in the domain.
	 */
	raw_spin_lock(&t->throt_lock);
	if (htd->start > t->last_htd_cntpct) {
		t->last_htd_cntpct = htd->start;
		update_thermal_pressure(t, CPU_HW_THROTTLE, freq);
	}
	raw_spin_unlock(&t->throt_lock);

reset_stats:
	reset_htd_data(htd);
}

/* The sfd helpers must be called with sfd->lock held */
static void reset_sfd_data(struct sfd_data *sfd)
{
	sfd->cpu_cyc = sfd->const_cyc = sfd->stale = 0;
}

static void add_sfd_data(struct sfd_data *sfd, const struct pmu_stat *cur,
			 const struct pmu_stat *prev)
{
	u64 delta_const_cyc = cur->const_cyc - prev->const_cyc;

	/*
	 * Check the delta since the last reading and ditch any stale readings
	 * if this sample window is sufficiently large.
	 */
	if (sfd->stale && delta_const_cyc >= cpu_min_sample_cntpct)
		reset_sfd_data(sfd);

	/* Accumulate data for calculating the CPU's frequency */
	sfd->cpu_cyc += cur->cpu_cyc - prev->cpu_cyc;
	sfd->const_cyc += delta_const_cyc;
}

static void update_freq_scale(int cpu, struct rq *rq, bool local_cpu)
{
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	struct sfd_data *sfd = &pmu->sfd;
	struct htd_data *htd = &pmu->htd;
	struct pmu_stat cur, prev;

	if (local_cpu) {
		pmu_update_stats(cpu, pmu, &cur, &prev);
		add_htd_data(htd, &cur, &prev);
	}

	/*
	 * Don't race with remote CPUs which may update the current CPU's
	 * runqueue clock and thus access sfd in parallel, and vice versa.
	 */
	raw_spin_lock(&sfd->lock);
	if (local_cpu)
		add_sfd_data(sfd, &cur, &prev);

	/*
	 * Set the CPU frequency scale measured via counters if enough data is
	 * present for the runqueue that's getting its clock updated (and thus
	 * about to use the frequency scale). This excludes idle time because
	 * although the cycle counter stops incrementing while the CPU idles,
	 * the system timer doesn't.
	 */
	if (rq->cpu == cpu) {
		if (sfd->const_cyc >= cpu_min_sample_cntpct) {
			struct cpufreq_policy *pol = &per_cpu(cached_pol, cpu);
			u64 freq, max_freq = pol->cpuinfo.max_freq;
			u64 ns = cntpct_to_ns(sfd->const_cyc);

			/* Report the measured frequency and reset the stats */
			freq = min(max_freq, USEC_PER_SEC * sfd->cpu_cyc / ns);
			per_cpu(arch_freq_scale, cpu) =
				SCHED_CAPACITY_SCALE * freq / max_freq;
			reset_sfd_data(sfd);
		} else if (sfd->const_cyc) {
			/*
			 * Track that the sfd statistics now contain stale data,
			 * since the frequency measurement won't perfectly
			 * correlate to the runqueue clock update window
			 * anymore. Keeping stale data for a previous window
			 * technically perpetuates this inaccuracy, but it is
			 * better than being unable to update the CPU frequency
			 * scale due to not having accumulated enough data. The
			 * stale data won't be used if the next window is long
			 * enough to compute the CPU's frequency.
			 */
			sfd->stale = true;
		}
	}
	raw_spin_unlock(&sfd->lock);

	/*
	 * Update the frequency scale data for the remote CPU when the updated
	 * runqueue doesn't belong to this CPU. This recursion is bounded.
	 */
	if (rq->cpu != cpu)
		update_freq_scale(rq->cpu, rq, false);
}

/*
 * Called from update_rq_clock(), just before update_rq_clock_task(). This way,
 * the CPU's frequency scale info has a chance to get updated just before it is
 * used by update_rq_clock_pelt() for computing load.
 */
void tensor_aio_update_rq_clock(struct rq *rq)
{
	int cpu = raw_smp_processor_id();

	/* Don't race with reboot or probe, since this isn't a vendor hook */
	if (!static_branch_unlikely(&system_ready))
		return;

	/* Don't race with CPU hotplug for this CPU or the runqueue's CPU */
	if (unlikely(!cpu_active(cpu) || !cpu_active(rq->cpu)))
		return;

	/*
	 * Update the local CPU's frequency scale info, even if the runqueue in
	 * question doesn't belong to the current CPU. This way, any runqueue
	 * clock updates for remote CPUs will have fresh counter data, for when
	 * the current CPU's runqueue is the one being updated remotely.
	 *
	 * This also handles updating the frequency scale info for the remote
	 * CPU if the runqueue is indeed remote.
	 *
	 * Although the measured CPU frequency is ignored by PELT for the idle
	 * task, measurements are still allowed inside the idle task so that IRQ
	 * load average can still be tracked accurately for interrupts which
	 * fire while the idle task runs. There is otherwise no point to
	 * measuring CPU frequency within the idle task. PELT only cares about
	 * precisely tracking non-idle tasks' runtime, which it does in terms of
	 * time a task consumed relative to CPU frequency, so that the scheduler
	 * can accurately calculate the load of each actual task.
	 */
	update_freq_scale(cpu, rq, true);
}

/*
 * Called from scheduler_tick() just before it updates the thermal load average.
 * Updates the measured hardware throttle of this CPU domain just before that
 * happens. update_cpu_hw_throttle() checks the starting time of the sample
 * window to ensure that only the latest measurements from a CPU in a CPU domain
 * are used.
 */
static void tensor_aio_tick_entry(void *data, struct rq *rq)
{
	update_cpu_hw_throttle();
}

/*
 * The scheduler tick is used to kick memperfd to evaluate statistics gathered
 * through the callback installed in update_rq_clock(). There's no scheduler
 * tick when CPUs aren't active, so memperfd is designed to only run when
 * there's CPU activity.
 */
static void tensor_aio_tick(void)
{
#ifdef CONFIG_SOC_ZUMA
	/*
	 * Update the TMU throttle for all CPU domains, so they don't get stuck
	 * appearing to the scheduler as throttled when they're all idle.
	 */
	update_tmu_throttle_all();
#endif

	/* Kick memperfd if it's time for it to run */
	kick_memperfd();
}

static struct scale_freq_data tensor_aio_sfd = {
	.source = SCALE_FREQ_SOURCE_ARCH,
	.set_freq_scale = tensor_aio_tick
};

static void set_cpu_hw_throttle_idle(int cpu, bool idle)
{
	struct throt_data *t = per_cpu(domain_throt_data, cpu);

	raw_spin_lock(&t->idle_cpu_lock);
	if (idle) {
		cpumask_set_cpu(cpu, &t->idle_cpus);

		/*
		 * Clear the measured hardware throttle for the CPU domain when
		 * all CPUs in the domain are idle.
		 */
		if (cpumask_equal(&t->idle_cpus, &t->domain->cpus)) {
			raw_spin_lock(&t->throt_lock);
			update_thermal_pressure(t, CPU_HW_THROTTLE, UINT_MAX);
			raw_spin_unlock(&t->throt_lock);
		}
	} else {
		cpumask_clear_cpu(cpu, &t->idle_cpus);
	}
	raw_spin_unlock(&t->idle_cpu_lock);
}

static void tensor_aio_cpu_idle(int cpu, bool idle)
{
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	struct sfd_data *sfd = &pmu->sfd;
	struct htd_data *htd = &pmu->htd;
	struct pmu_stat cur, prev;

	/* Don't race with reboot */
	if (!static_branch_unlikely(&system_ready))
		return;

	/* Don't race with CPU hotplug */
	if (unlikely(!cpu_active(cpu)))
		return;

	set_cpu_hw_throttle_idle(cpu, idle);

	if (idle) {
		/* Update the current counters one last time before idling */
		pmu_update_stats(cpu, pmu, &cur, &prev);

		/* Accumulate data for calculating the CPU's frequency */
		raw_spin_lock(&sfd->lock);
		add_sfd_data(sfd, &cur, &prev);
		raw_spin_unlock(&sfd->lock);
	} else {
		/*
		 * For CPUs which don't support AMU const cycles: update the
		 * counters upon exiting idle without accumulating frequency
		 * data, in order to disregard all statistics from the period
		 * when the CPU was idle. This is because the system timer keeps
		 * incrementing while the CPU is idle, while the cycle counter
		 * doesn't because the CPU clock is gated in idle. This isn't a
		 * problem for AMU const cycles because it *does* stop
		 * incrementing while the CPU is idle.
		 */
		if (!cpu_supports_amu_const(cpu))
			pmu_update_stats(cpu, pmu, &cur, NULL);

		/* Discard stale hardware throttle detection data */
		reset_htd_data(htd);
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
	struct sfd_data *sfd = &pmu->sfd;
	struct htd_data *htd = &pmu->htd;
	int ret;

	ret = create_perf_events(cpu);
	if (ret)
		return ret;

	/* Initialize the pointers to the saved CPU stats */
	pmu->cur_ptr[0] = &pmu->cur[0];
	pmu->cur_ptr[1] = &pmu->cur[1];

	/*
	 * Update and reset the statistics for this CPU as it comes online. No
	 * need to take any locks since `cpu_active(cpu) == false` (except in
	 * memperfd_init()), so no shared data can be accessed concurrently with
	 * the hotplug handler. Disabling IRQs when reading the PMU statistics
	 * is needed to prevent interrupts from firing during the measurement
	 * and thus skewing the data.
	 */
	local_irq_disable();
	pmu_get_stats(&pmu->cur[0]);
	local_irq_enable();
	pmu->prev = pmu->cur[0];
	reset_sfd_data(sfd);
	reset_htd_data(htd);

	/* Install tensor_aio_tick() */
	topology_set_scale_freq_source(&tensor_aio_sfd, cpumask_of(cpu));

	/* Clear the hardware throttle idle flag for this CPU */
	set_cpu_hw_throttle_idle(cpu, false);
	return 0;
}

static int memperf_cpuhp_down(unsigned int cpu)
{
	/* Stop tensor_aio_tick() from running on this CPU anymore */
	topology_clear_scale_freq_source(SCALE_FREQ_SOURCE_ARCH,
					 cpumask_of(cpu));
	release_perf_events(cpu);

	/*
	 * Set the hardware throttle idle flag for this CPU, so this CPU is
	 * considered idle insofar as the hardware throttle detection is
	 * concerned. IRQs must be disabled around set_cpu_hw_throttle_idle()
	 * because this may be the last non-idle CPU in the domain, in which
	 * case t->throt_lock would be taken, requiring IRQs to be disabled.
	 */
	local_irq_disable();
	set_cpu_hw_throttle_idle(cpu, true);
	local_irq_enable();
	return 0;
}

static void update_qos_req(struct exynos_pm_qos_request *req, int value)
{
	/* Only update if the request value is different */
	if (req->node.prio != value)
		exynos_pm_qos_update_request(req, value);
}

static void memperfd_quiesce(void)
{
	int i;

	/*
	 * Clear out all of memperfd's votes for the domains it governs. This is
	 * safe from atomic context because all of memperfd's domains have fast,
	 * atomic DVFS, and therefore use raw spin locks in their exynos_pm_qos
	 * notifier callbacks.
	 */
	for (i = 0; i < ARRAY_SIZE(memperfd_domains); i++) {
		struct exynos_devfreq_data *data = memperfd_domains[i];

		update_qos_req(&data->min_req, data->tbl[data->nr_freqs - 1]);
	}
	memperfd_quiescent = true;
}

static void memperfd_unquiesce(void)
{
	atomic_set(&stats_avail_cpus, 0);
	atomic_long_set(&last_run_jiffies, jiffies);
	memperfd_quiescent = false;
}

static void tensor_aio_idle_task_switch(bool entering)
{
	/*
	 * Track the number of CPUs within the idle task. When all CPUs enter
	 * the idle task, quiesce memperfd to save power. Once a CPU becomes
	 * active again, reset memperfd so it runs with fresh statistics.
	 */
	raw_spin_lock(&idle_task_lock);
	if (entering) {
		if (++idle_task_cpus == num_online_cpus())
			memperfd_quiesce();
	} else {
		if (idle_task_cpus-- == num_online_cpus())
			memperfd_unquiesce();
	}
	raw_spin_unlock(&idle_task_lock);
}

static void tensor_aio_schedule(void *data, unsigned int sched_mode,
				struct task_struct *prev_tsk,
				struct task_struct *next_tsk, struct rq *rq)
{
	/* Don't race with reboot */
	if (!static_branch_unlikely(&system_ready))
		return;

	/* Do nothing when there isn't a task switch */
	if (prev_tsk == next_tsk)
		return;

	/* Track transitions to/from the idle task */
	if (is_idle_task(next_tsk))
		tensor_aio_idle_task_switch(true);
	else if (is_idle_task(prev_tsk))
		tensor_aio_idle_task_switch(false);
}

static int tensor_aio_idle_init(void *unused)
{
	/* All CPUs are guaranteed to not be in the idle task right now */
	idle_task_cpus = 0;
	return 0;
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
	int i, j;

	for (i = 0; i < um->all_regs_cnt; i++) {
		for (j = 0; j < cnt; j++)
			__raw_writel(reg[j].val, um->all_regs[i] + reg[j].off);
	}

	/* Ensure all of the register writes finish before continuing */
	__iomb();
}

static void memperfd_init(void)
{
	/*
	 * Delete the arch's scale_freq_data callback to get rid of the
	 * duplicated work by the arch's callback, since we read the same
	 * values. This also lets the frequency invariance engine work on cores
	 * that lack the AMU const cycles counter, since we use a workaround for
	 * such CPUs by using cpuidle callbacks to deduct time spent in WFE/WFI,
	 * which is good enough despite not tracking WFE/WFI usage outside of
	 * cpuidle (such as WFE/WFI usage in __delay()).
	 *
	 * A new scale_freq_data callback is installed in memperf_cpuhp_up().
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

	/* Begin updating CPU scheduler statistics from update_rq_clock() */
	static_branch_enable(&system_ready);

	/* Initialize and start the PPCs */
	ppc_write_regs(ppc_init_cmd, ARRAY_SIZE(ppc_init_cmd));

	/* Install the scheduler tick entry hook to detect CPU HW throttling */
	BUG_ON(register_trace_android_rvh_tick_entry(tensor_aio_tick_entry,
						     NULL));

	/*
	 * Stop all online CPUs in order to register the idle-task CPUs tracker,
	 * so that it starts out with all CPUs non-idle and thus can keep an
	 * accurate count of the number of CPUs inside the idle task.
	 */
	BUG_ON(register_trace_android_rvh_schedule(tensor_aio_schedule, NULL));
	BUG_ON(stop_machine(tensor_aio_idle_init, NULL, NULL));
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
	 * while the CPU is idle, so scaling MIF up in order to satisfy the
	 * CPU's frequency-invariant load would cause MIF to burn more power
	 * than just letting the CPU burn a few extra cycles on memory stalls.
	 * And if the CPU is under heavy load, it won't have much idle time and
	 * thus MIF will be scaled up accordingly anyway.
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

	/* Ensure all of the PPC register reads finish before they're reset */
	__iomb();

	/* Reset and restart all of the PPCs */
	ppc_write_regs(ppc_start_cmd, ARRAY_SIZE(ppc_start_cmd));

	/* Only the last group's (BUS2) MIF vote is used to set the INT vote */
	*bus2_mif = vote;

	/* Return the highest MIF frequency vote */
	return h_vote;
}

static void memperfd_work(void)
{
	u32 vote = mif->nr_freqs - 1, cur, dsu_vote = 0, bus2_mif;
	unsigned long cpus;
	int cpu;

	/* Get the current MIF freq, since something else could've raised it */
	cur = find_index_l(mif, READ_ONCE(mif->cur_freq));

	/* Gather fresh memory stall statistics for all updated CPUs */
	cpus = (unsigned int)atomic_read_acquire(&stats_avail_cpus);
	for_each_cpu(cpu, to_cpumask(&cpus)) {
		struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
		struct pmu_stat *cur_ptr, stat;
		u64 delta_cntpct;

		/* Calculate the delta for each statistic */
		cur_ptr = pmu_get_cur_reader(pmu);
		delta_cntpct = cur_ptr->cntpct - pmu->prev.cntpct;
		if (delta_cntpct >= cpu_min_sample_cntpct) {
			stat.mem_cyc = cur_ptr->mem_cyc - pmu->prev.mem_cyc;
			stat.cpu_cyc = cur_ptr->cpu_cyc - pmu->prev.cpu_cyc;
			stat.cntpct = delta_cntpct;
		} else {
			/* Indicate that this CPU should be skipped */
			stat.cpu_cyc = 0;
		}
		pmu_put_cur_reader(pmu, cur_ptr);

		/* Skip CPUs that have been idle for a while */
		if (!stat.cpu_cyc || !stat.mem_cyc || !stat.cntpct)
			continue;

		/* Find the highest MIF freq step required among all the CPUs */
		vote = min(vote, mif_cpu_vote(&stat, cpu, cur, &dsu_vote));
	}

	/* Find the highest PPC MIF vote and set the higher of the MIF votes */
	vote = max((u32)mif->tbl[vote], mif_ppc_vote(mif->tbl[cur], &bus2_mif));
	update_qos_req(&mif->min_req, vote);

	/* Set the new INT vote using BUS2's MIF requirement */
	for (vote = mif_int_cnt - 1; vote > 0; vote--) {
		if (bus2_mif <= mif_int_map[vote].mif_freq)
			break;
	}
	update_qos_req(&df_data[INT].min_req, mif_int_map[vote].int_freq);

#ifdef CONFIG_SOC_ZUMA
	/* Set the new DSU vote */
	update_qos_req(&dsu->min_req, dsu_vote);
#endif

	/*
	 * Reset the statistics for all CPUs. This is done after all voting
	 * takes place to ensure that all future measurements occur with the
	 * updated votes; i.e., so that there isn't any stale data from before
	 * memperfd altered the MIF frequency. This intentionally discards any
	 * measurements that may have occurred between when the statistics were
	 * read in the loop above and now.
	 */
	atomic_set_release(&stats_avail_cpus, 0);
}

static void memperfd_wait_for_kick(void)
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
		schedule();
	}
	finish_swait(&memperfd_waitq, &wait);
}

static int __noreturn memperf_thread(void *data)
{
	sched_set_fifo(current);
	memperfd_init();
	set_freezable();
	while (1) {
		memperfd_wait_for_kick();
		memperfd_work();
	}
}

static void exynos_qos_notify(struct exynos_devfreq_data *data)
{
	u32 freq;

	/* Set the frequency to the floor of the current limits */
	data->nb_lock_fn(&data->nb_lock);
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
	data->nb_unlock_fn(&data->nb_lock);
}

static int exynos_qos_min_notifier(struct notifier_block *nb,
				   unsigned long value, void *unused)
{
	struct exynos_devfreq_data *data =
		container_of(nb, typeof(*data), min_nb);
	u32 freq = find_freq_l(data, value);

	data->nb_lock_fn(&data->min_nb_lock);
	data->min_freq = freq;
	exynos_qos_notify(data);
	data->nb_unlock_fn(&data->min_nb_lock);
	return NOTIFY_OK;
}

static int exynos_qos_max_notifier(struct notifier_block *nb,
				   unsigned long value, void *unused)
{
	struct exynos_devfreq_data *data =
		container_of(nb, typeof(*data), max_nb);
	u32 freq = find_freq_h(data, value);

	data->nb_lock_fn(&data->max_nb_lock);
	data->max_freq = freq;
	exynos_qos_notify(data);
	data->nb_unlock_fn(&data->max_nb_lock);
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

	data->nb_lock_fn(&data->nb_lock);
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
	data->nb_unlock_fn(&data->nb_lock);
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

		um->all_regs_cnt += ug->cnt;
	}

	/* Store all register addresses in a single array for fast lookup */
	um->all_regs = kmalloc_array(um->all_regs_cnt, sizeof(*um->all_regs),
				     GFP_KERNEL);
	if (!um->all_regs)
		goto free_mem;

	for (i = 0, k = 0; i < um->grp_cnt; i++) {
		struct um_group *ug = &um->grp[i];

		for (j = 0; j < ug->cnt; j++)
			um->all_regs[k++] = ug->va_base[j];
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

	if (domain_has_fast_dvfs(edev)) {
		raw_spin_lock_init(&data->min_nb_lock.raw_spinlock);
		raw_spin_lock_init(&data->max_nb_lock.raw_spinlock);
		raw_spin_lock_init(&data->nb_lock.raw_spinlock);
		data->nb_lock_fn = (void *)_raw_spin_lock;
		data->nb_unlock_fn = (void *)_raw_spin_unlock;
	} else {
		rt_mutex_init(&data->min_nb_lock.rt_mutex);
		rt_mutex_init(&data->max_nb_lock.rt_mutex);
		rt_mutex_init(&data->nb_lock.rt_mutex);
		data->nb_lock_fn = (void *)rt_mutex_lock;
		data->nb_unlock_fn = (void *)rt_mutex_unlock;
	}
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
	 * Kill tensor_aio_tick() on all CPUs to stop kicking memperfd, and
	 * disable `system_ready` to prevent further PMU register access
	 * after this. PMU registers must not be accessed after kvm_reboot()
	 * finishes; attempting to do so will fault.
	 *
	 * This also needs to kick all CPUs to ensure that the scheduler and
	 * cpuidle hooks aren't running anymore. This works because the hooks
	 * themselves are always called from IRQs-disabled context, so when the
	 * IPI kick goes through it means that all in-flight IRQs-disabled
	 * contexts are done executing. Thus, once kick_all_cpus_sync() returns,
	 * it is guaranteed that all hooks which may read PMU registers will
	 * observe `system_ready == false`.
	 */
	topology_clear_scale_freq_source(SCALE_FREQ_SOURCE_ARCH,
					 cpu_possible_mask);
	static_branch_disable(&system_ready);
	kick_all_cpus_sync();
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
