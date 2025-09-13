// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/errno.h>
#include <linux/suspend.h>
#include <linux/perf_event.h>
#include <linux/of.h>
#include <linux/cpu_pm.h>
#include <linux/sched/clock.h>
#include <linux/notifier.h>
#include <linux/panic_notifier.h>
#include <linux/smpboot.h>
#include <linux/hrtimer.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/time64.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <linux/perf/arm_pmuv3.h>

#include <soc/google/acpm_ipc_ctrl.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-ehld.h>
#include <soc/google/exynos-coresight.h>
#include <soc/google/debug-snapshot.h>
#include <soc/google/sjtag-driver.h>

#include "core_regs.h"

#undef DEBUG
#undef EHLD_TASK_SUPPORT

#ifdef DEBUG
#define ehld_debug(f, str...) pr_debug(str)
#define ehld_info(f, str...) pr_info(str)
#define ehld_err(f, str...) pr_err(str)
#else
#define ehld_debug(f, str...) do { if (f == 1) pr_debug(str); } while (0)
#define ehld_info(f, str...) do { if (f == 1) pr_info(str); } while (0)
#define ehld_err(f, str...)  do { if (f == 1) pr_err(str); } while (0)
#endif

#define EHLD_PCSR_RES0_MAGIC	(0x5cULL << 56)
#define EHLD_PCSR_SELF		(EHLD_PCSR_RES0_MAGIC | 0xbb0ce1a1d05e1fULL)
#define MSB_MASKING		(0x0000FF0000000000)
#define MSB_PADDING		(0xFFFFFF0000000000)
#define DBG_UNLOCK(base)	\
	do { isb(); __raw_writel(OSLOCK_MAGIC, base + DBGLAR); } while (0)
#define DBG_LOCK(base)		\
	do { __raw_writel(0x1, base + DBGLAR); isb(); } while (0)
#define DBG_OS_UNLOCK(base)	\
	do { isb(); __raw_writel(0, base + DBGOSLAR); } while (0)
#define DBG_OS_LOCK(base)	\
	do { __raw_writel(0x1, base + DBGOSLAR); isb(); } while (0)

#define ALIVE_FRC_FREQ_49_152_MHZ (49152000) /* 49.152 MHz crystal */

#if IS_ENABLED(CONFIG_HARDLOCKUP_WATCHDOG)
extern struct atomic_notifier_head hardlockup_notifier_list;
#endif

struct exynos_ehld_dump {
	unsigned int			core_pmu;
	unsigned int			dss_pmu;
	unsigned int			ehld_stat;
};

struct exynos_ehld_dbgc {
	unsigned int			support;
	unsigned int			enabled;
	unsigned int			interval;
	unsigned int			warn_count;
	bool				use_tick_timer;
};

struct exynos_ehld_main {
	raw_spinlock_t			lock;
	unsigned int			cs_base;
	int				enabled;
	bool				suspending;
	bool				pm_suspending;
	bool				sicd_suspending;
	bool				resuming;
	struct exynos_ehld_dbgc		dbgc;
	void __iomem			**sgi_base;
};

static struct exynos_ehld_main ehld_main = {
	.lock = __RAW_SPIN_LOCK_UNLOCKED(ehld_main.lock),
};

struct exynos_ehld_data {
	unsigned long long		time[NUM_TRACE];
	unsigned long long		alive_time[NUM_TRACE];
	unsigned long long		event[NUM_TRACE];
	unsigned long long		pmpcsr[NUM_TRACE];
	unsigned long			data_ptr;
};

struct exynos_ehld_ctrl {
	struct task_struct		*task;
	struct hrtimer			hrtimer;
	struct perf_event		*event;
	struct exynos_ehld_data		data;
	void __iomem			*dbg_base;
	int				ehld_running;  /* CPUHP state */
	int				ehld_cpupm;    /* CPUPM state */
	raw_spinlock_t			lock;
	bool				need_to_task;
	u32				cntr_shift;
};

static DEFINE_PER_CPU(struct exynos_ehld_ctrl, ehld_ctrl) = {
	.lock = __RAW_SPIN_LOCK_UNLOCKED(ehld_ctrl.lock),
};

static struct perf_event_attr exynos_ehld_attr = {
	.type           = PERF_TYPE_HARDWARE,
	.config         = PERF_COUNT_HW_INSTRUCTIONS,
	.size           = sizeof(struct perf_event_attr),
	.sample_period  = U32_MAX,
	.pinned         = 1,
	.disabled       = 1,
};

static void exynos_ehld_callback(struct perf_event *event,
			       struct perf_sample_data *data,
			       struct pt_regs *regs)
{
	event->hw.interrupts = 0;       /* don't throttle interrupts */
}

static void exynos_ehld_do_action(unsigned int cpu, unsigned int lockup_level)
{
	unsigned int val;
	int i;

	switch (lockup_level) {
	case EHLD_STAT_LOCKUP_WARN:
	case EHLD_STAT_LOCKUP_SW:
		exynos_ehld_event_raw_dump(cpu, true);
		break;
	case EHLD_STAT_LOCKUP_HW:
		exynos_ehld_event_raw_dump(cpu, true);
#if IS_ENABLED(CONFIG_HARDLOCKUP_WATCHDOG)
		atomic_notifier_call_chain(&hardlockup_notifier_list,
							0, (void *)&cpu);
#endif
		for_each_possible_cpu(i) {
			val = dbg_snapshot_get_core_pmu_val(i);
			ehld_info(1, "%s: cpu%u: pmu_val:%#x\n",
							__func__, i, val);
		}
		panic("Watchdog detected hard HANG on cpu %u by EHLD", cpu);
		break;
	}
}

void exynos_ehld_do_policy(void)
{
	unsigned long flags;
	unsigned int cpu;
	unsigned int warn = 0, lockup_hw = 0, lockup_sw = 0;

	raw_spin_lock_irqsave(&ehld_main.lock, flags);
	for_each_possible_cpu(cpu) {
		unsigned int val;
		struct exynos_ehld_ctrl *ctrl;

		val = dbg_snapshot_get_core_ehld_stat(cpu);
		if (val & EHLD_STAT_HANDLED_FLAG)
			continue;

		if (val == EHLD_STAT_NORMAL)
			continue;

		ehld_info(0, "%s: cpu%u: val:%x timer:%llx",
				__func__, cpu, val, arch_timer_read_counter());
		ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
		switch (val) {
		case EHLD_STAT_LOCKUP_WARN:
			warn |= BIT(cpu);
			break;
		case EHLD_STAT_LOCKUP_SW:
			lockup_sw |= BIT(cpu);
			break;
		case EHLD_STAT_LOCKUP_HW:
			lockup_hw |= BIT(cpu);
			break;
		default:
			break;
		}
		val |= EHLD_STAT_HANDLED_FLAG;
		dbg_snapshot_set_core_ehld_stat(val, cpu);
	}
	raw_spin_unlock_irqrestore(&ehld_main.lock, flags);

	for_each_possible_cpu(cpu) {
		if (warn & (1 << cpu)) {
			ehld_info(1, "%s: cpu%u is hardlockup warning",
								__func__, cpu);
			exynos_ehld_event_raw_update(cpu, false);
			exynos_ehld_do_action(cpu, EHLD_STAT_LOCKUP_WARN);
		}
		if (lockup_sw & (1 << cpu)) {
			exynos_ehld_event_raw_update(cpu, false);
			ehld_info(1, "%s: cpu%u is hardlockup by software",
								__func__, cpu);
			exynos_ehld_do_action(cpu, EHLD_STAT_LOCKUP_SW);
		}
		if (lockup_hw & (1 << cpu)) {
			exynos_ehld_event_raw_update(cpu, false);
			ehld_info(1, "%s: cpu%u is hardlockup by hardware",
								__func__, cpu);
			exynos_ehld_do_action(cpu, EHLD_STAT_LOCKUP_HW);
		}
	}
}

/*
 * This implementation uses the code in `drivers/perf/arm_pmuv3.c` to access low-level ARMv8 PMU
 * events. The `ARMV8_IDX_TO_COUNTER` is cloned to convert the `event->hw.idx` to the corresponding
 * ARMv8 counter ID.
 */
#define ARMV8_IDX_COUNTER0 1
#define ARMV8_IDX_TO_COUNTER(x) (((x) - ARMV8_IDX_COUNTER0) & ARMV8_PMU_COUNTER_MASK)

static u32 exynos_ehld_read_pmu_counter(void)
{
	unsigned int cpu = raw_smp_processor_id();
	struct exynos_ehld_ctrl *ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	struct perf_event *event = ctrl->event;

	write_sysreg(ARMV8_IDX_TO_COUNTER(event->hw.idx), pmselr_el0);
	isb();
	return read_sysreg(pmxevcntr_el0);
}

void exynos_ehld_value_raw_update(unsigned int cpu)
{
	u32 val;
	struct exynos_ehld_ctrl *ctrl = per_cpu_ptr(&ehld_ctrl, cpu);

	val = exynos_ehld_read_pmu_counter();

	dbg_snapshot_set_core_pmu_val(val + ctrl->cntr_shift, cpu);

	ehld_info(0, "%s: cpu%u: val:%x timer:%llx",
			__func__, cpu, val, cpu_clock(cpu));
}

static enum hrtimer_restart ehld_value_raw_hrtimer_fn(struct hrtimer *hrtimer)
{
	unsigned int cpu = raw_smp_processor_id();
	struct exynos_ehld_ctrl *ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	struct perf_event *event = ctrl->event;

	if (!event || !ehld_main.dbgc.support) {
		ehld_err(1, "@%s: cpu%u, HRTIMER is cancel\n", __func__, cpu);
		return HRTIMER_NORESTART;
	}

	if (!ehld_main.dbgc.enabled) {
		ehld_debug(1, "@%s: cpu%u, dbgc is not enabled, re-start\n",
								__func__, cpu);
		hrtimer_forward_now(hrtimer, ns_to_ktime(NSEC_PER_SEC / 2));
		return HRTIMER_RESTART;
	}

	if (event->state != PERF_EVENT_STATE_ACTIVE) {
		dbg_snapshot_set_core_pmu_val(EHLD_VAL_PM, cpu);
		ehld_err(1, "@%s: cpu%u, event state is not active: %d\n",
						__func__, cpu, event->state);
		return HRTIMER_NORESTART;
	}

	exynos_ehld_value_raw_update(cpu);
	exynos_ehld_do_policy();

	ehld_info(0, "@%s: cpu%u hrtimer is running\n", __func__, cpu);

	if (ehld_main.dbgc.interval > 0) {
		hrtimer_forward_now(hrtimer,
			ns_to_ktime(ehld_main.dbgc.interval * 1000 * 1000));
	} else {
		ehld_info(1, "@%s: cpu%u hrtimer interval is abnormal: %u\n",
				__func__, cpu, ehld_main.dbgc.interval);
		return HRTIMER_NORESTART;
	}

	return HRTIMER_RESTART;
}

void ehld_tick_sched_timer(void)
{
	unsigned int cpu = raw_smp_processor_id();
	struct exynos_ehld_ctrl *ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	struct perf_event *event = ctrl->event;

	if (!event || !ehld_main.dbgc.support || !ehld_main.dbgc.enabled)
		return;

	if (event->state != PERF_EVENT_STATE_ACTIVE) {
		dbg_snapshot_set_core_pmu_val(EHLD_VAL_PM, cpu);
		ehld_err(0, "@%s: cpu%u, event state is not active: %d\n",
				__func__, cpu, event->state);
	} else {
		exynos_ehld_value_raw_update(cpu);
		exynos_ehld_do_policy();
	}

	ehld_info(0, "@%s: cpu%u hrtimer is running\n", __func__, cpu);
}

static void exynos_ehld_start_cpu_hrtimer(void *data)
{
	struct hrtimer *hrtimer = (struct hrtimer *)data;
	u64 interval = ehld_main.dbgc.interval * 1000 * 1000;

	hrtimer_start(hrtimer, ns_to_ktime(interval), HRTIMER_MODE_REL_PINNED);
}

static int exynos_ehld_start_cpu(unsigned int cpu)
{
	struct exynos_ehld_ctrl *ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	struct perf_event *event = ctrl->event;
	struct hrtimer *hrtimer = &ctrl->hrtimer;

	/* during resume, need to handle cpu 0 here from cpu 1 context */
	if (ehld_main.suspending && cpu == 1)
		exynos_ehld_start_cpu(0);

	if (!event) {
		int ret;

		event = perf_event_create_kernel_counter(&exynos_ehld_attr,
							 cpu,
							 NULL,
							 exynos_ehld_callback,
							 NULL);
		if (IS_ERR(event)) {
			ehld_err(1, "@%s: cpu%u event make failed err: %ld\n",
						__func__, cpu, PTR_ERR(event));
			return PTR_ERR(event);
		}

		ehld_debug(1, "@%s: cpu%u event make success\n", __func__, cpu);
		ctrl->event = event;
		perf_event_enable(event);

		ret = adv_tracer_ehld_set_pmu_cntr_id(cpu, 1, ARMV8_IDX_TO_COUNTER(event->hw.idx));
		if (ret) {
			ehld_err(1, "@%s: cpu%u set_pmu_cntr_id failed: %d\n", __func__, cpu, ret);
			ctrl->event = NULL;
			perf_event_disable(event);
			perf_event_release_kernel(event);
			return ret;
		}
	}

	ctrl->ehld_running = 1;
	ctrl->ehld_cpupm = 0;
	ehld_debug(1, "@%s: cpu%u ehld running\n", __func__, cpu);

	if (!ehld_main.dbgc.support)
		return 0;

	if (!ehld_main.dbgc.use_tick_timer) {
		u64 interval = ehld_main.dbgc.interval * 1000 * 1000;

		ehld_debug(1, "@%s: cpu%u ehld running with hrtimer\n", __func__, cpu);
		hrtimer_init(hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		hrtimer->function = ehld_value_raw_hrtimer_fn;
		if (ehld_main.suspending && cpu == 0)
			smp_call_function_single(0, exynos_ehld_start_cpu_hrtimer, hrtimer, 0);
		else
			hrtimer_start(hrtimer, ns_to_ktime(interval), HRTIMER_MODE_REL_PINNED);
	} else {
		ehld_debug(1, "@%s: cpu%u ehld running with tick-timer\n", __func__, cpu);
	}

	return 0;
}

static int exynos_ehld_stop_cpu(unsigned int cpu)
{
	struct exynos_ehld_ctrl *ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	struct perf_event *event;
	struct hrtimer *hrtimer = &ctrl->hrtimer;

	/* during suspend, need to handle cpu 0 here from cpu 1 context */
	if (ehld_main.suspending && cpu == 1)
		exynos_ehld_stop_cpu(0);

	if (ehld_main.dbgc.support && !ehld_main.dbgc.use_tick_timer) {
		ehld_debug(1, "@%s: cpu%u hrtimer cancel\n", __func__, cpu);
		hrtimer_cancel(hrtimer);
	}

	ctrl->ehld_running = 0;
	ctrl->ehld_cpupm = 1;

	ehld_debug(1, "@%s: cpu%u ehld stopping\n", __func__, cpu);

	event = ctrl->event;
	if (event) {
		adv_tracer_ehld_set_pmu_cntr_id(cpu, 0, 0);
		ctrl->event = NULL;
		perf_event_disable(event);
		perf_event_release_kernel(event);
	}

	dbg_snapshot_set_core_pmu_val(EHLD_VAL_PM, cpu);

	return 0;
}

unsigned long long exynos_ehld_event_read_cpu(unsigned int cpu)
{
	struct exynos_ehld_ctrl *ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	struct perf_event *event = ctrl->event;
	unsigned long long total = 0;
	unsigned long long enabled, running;

	if (!in_irq() && event) {
		total = perf_event_read_value(event, &enabled, &running);
		ehld_info(0, "%s: cpu%u - enabled: %llu, running: %llu, total: %llu\n",
				__func__, cpu, enabled, running, total);
	}
	return total;
}

void exynos_ehld_event_raw_update(unsigned int cpu, bool update_val)
{
	struct exynos_ehld_ctrl *ctrl;
	struct exynos_ehld_data *data;
	unsigned long long val;
	unsigned long flags, count;

	ctrl = per_cpu_ptr(&ehld_ctrl, cpu);

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	data = &ctrl->data;
	count = ++data->data_ptr & (NUM_TRACE - 1);
	data->time[count] = cpu_clock(cpu);
	data->alive_time[count] = get_frc_time() * MSEC_PER_SEC / ALIVE_FRC_FREQ_49_152_MHZ;
	if (sjtag_is_locked() || cpu_is_offline(cpu) || !ctrl->ehld_running ||
	    ctrl->ehld_cpupm) {
		val = EHLD_VAL_PM;
		data->event[count] = val;
		data->pmpcsr[count] = 0;
	} else {
		DBG_UNLOCK(ctrl->dbg_base + PMU_OFFSET);
		/*
		 * Workaround: Need to read PMUPCSR twice to get valid
		 * PC values. The first read keeps returning 0xffffffff.
		 */
		val = __raw_readq(ctrl->dbg_base + PMU_OFFSET + PMUPCSR);
		val = __raw_readq(ctrl->dbg_base + PMU_OFFSET + PMUPCSR);
		if (MSB_MASKING == (MSB_MASKING & val))
			val |= MSB_PADDING;
		if (cpu == raw_smp_processor_id())
			val = EHLD_PCSR_SELF;
		data->pmpcsr[count] = val;
		val = __raw_readl(ctrl->dbg_base + PMU_OFFSET);
		data->event[count] = val;
		DBG_LOCK(ctrl->dbg_base + PMU_OFFSET);
	}
	if (ehld_main.dbgc.support && ehld_main.dbgc.enabled && update_val)
		dbg_snapshot_set_core_pmu_val(val, cpu);

	ehld_info(0, "%s: cpu%u: running:%x, offline:%ld\n",
			__func__, cpu,
			ctrl->ehld_running,
			cpu_is_offline(cpu));

	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
	ehld_info(0, "%s: cpu%u - time:%llu(%llu.%03llu), event:%#llx\n",
		__func__, cpu, data->time[count],
		data->alive_time[count] / MSEC_PER_SEC, data->alive_time[count] % MSEC_PER_SEC,
		data->event[count]);
}

void exynos_ehld_event_raw_update_allcpu(void)
{
	int cpu;

	for_each_possible_cpu(cpu)
		exynos_ehld_event_raw_update(cpu, true);
}

static void print_gicr_regs(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		ehld_err(1, "cpu%u: gicr: is-enabled:%08x, is-pending:%08x\n",
			 cpu,
			 __raw_readl(ehld_main.sgi_base[cpu] + GICR_ISENABLER0),
			 __raw_readl(ehld_main.sgi_base[cpu] + GICR_ISPENDR0));
}

static void print_ehld_header(void)
{
	ehld_err(1, "-------------------------------------------------------------------------------------------\n");
	ehld_err(1, "      Exynos Early Lockup Detector Information\n\n");
	ehld_err(1, "      CPU    NUM     TIME:sys(alive)                      Value                PC\n\n");
}

void exynos_ehld_event_raw_dump(unsigned int cpu, bool header)
{
	struct exynos_ehld_ctrl *ctrl;
	struct exynos_ehld_data *data;
	unsigned long flags, count;
	int i;
	char buf[128];

	if (sjtag_is_locked()) {
		ehld_err(1, "EHLD trace requires SJTAG authentication\n");
		return;
	}

	if (header) {
		print_gicr_regs();
		print_ehld_header();
	}

	ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	raw_spin_lock_irqsave(&ctrl->lock, flags);
	data = &ctrl->data;
	for (i = 0; i < NUM_TRACE; i++) {
		count = ++data->data_ptr % NUM_TRACE;
		if (i < NUM_TRACE_SKIP)
			continue;
		if (data->pmpcsr[count] == EHLD_PCSR_SELF) {
			strlcpy(buf, "(self)", sizeof(buf));
		} else {
			snprintf(buf, sizeof(buf), "%#016llx(%pS)",
				 data->pmpcsr[count], (void *)data->pmpcsr[count]);
		}
		ehld_err(1, "      %03u    %03d     %015llu(%010llu.%03llu)      %#015llx      %s\n",
					cpu, i + 1, data->time[count],
					data->alive_time[count] / MSEC_PER_SEC,
					data->alive_time[count] % MSEC_PER_SEC,
					data->event[count], buf);
	}
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
	ehld_err(1, "--------------------------------------------------------------------------\n");
}

void exynos_ehld_event_raw_dump_allcpu(void)
{
	unsigned int cpu;

	if (sjtag_is_locked()) {
		ehld_err(1, "EHLD trace requires SJTAG authentication\n");
		return;
	}

	print_gicr_regs();
	print_ehld_header();
	for_each_possible_cpu(cpu)
		exynos_ehld_event_raw_dump(cpu, false);
}

static int exynos_ehld_cpu_pm_exit(unsigned int cpu)
{
	struct exynos_ehld_ctrl *ctrl;
	unsigned long flags;

	ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	raw_spin_lock_irqsave(&ctrl->lock, flags);
	if (!ctrl->ehld_running)
		goto out;

	/*
	 * WAR for b/233011236
	 * Do not touch hrtimers in EHLD CPUPM callbacks
	 *
	 * if (ehld_main.dbgc.support && !ehld_main.dbgc.use_tick_timer &&
	 *     !ehld_main.suspending && !hrtimer_active(&ctrl->hrtimer)) {
	 *     hrtimer_start(&ctrl->hrtimer,
	 *                   ns_to_ktime(ehld_main.dbgc.interval * 1000 * 1000),
	 *                   HRTIMER_MODE_REL_PINNED);
	 * }
	 */

	/*
	 * When a CPU core exits PM, we cannot use a constant value
	 * (EHLD_VAL_PM_POST or zero) here as the wake-up value.
	 * This is because CPU enters and exits PM frequently, and
	 * debug core keeps observing the core PMU value frequently.
	 * A single constant value will look like the core is stuck.
	 *
	 * Instead, need to show progress. In the optimal world, we
	 * would just pull the PMU retired instruction counter here,
	 * just like the hrtimer callback does. But experimentation
	 * shows that PMU returns zero rather frequently here.
	 *
	 * Thus, we will need to use CPU clock as the initial value.
	 */
	dbg_snapshot_set_core_pmu_val(cpu_clock(cpu), cpu);
	ctrl->cntr_shift = cpu_clock(cpu);
	ctrl->ehld_cpupm = 0;
out:
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
	return 0;
}

static int exynos_ehld_cpu_pm_enter(unsigned int cpu)
{
	struct exynos_ehld_ctrl *ctrl;
	unsigned long flags;

	ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
	raw_spin_lock_irqsave(&ctrl->lock, flags);
	if (!ctrl->ehld_running)
		goto out;

	/*
	 * WAR for b/233011236
	 * Do not touch hrtimers in EHLD CPUPM callbacks
	 *
	 * if (ehld_main.dbgc.support && !ehld_main.dbgc.use_tick_timer &&
	 *     !ehld_main.suspending && hrtimer_active(&ctrl->hrtimer)) {
	 *     hrtimer_cancel(&ctrl->hrtimer);
	 * }
	 */

	dbg_snapshot_set_core_pmu_val(EHLD_VAL_PM_PREPARE, cpu);
	ctrl->ehld_cpupm = 1;
out:
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
	return 0;
}

int exynos_ehld_start(void)
{
	int cpu;

	cpus_read_lock();
	for_each_online_cpu(cpu)
		exynos_ehld_start_cpu(cpu);
	cpus_read_unlock();

	return 0;
}

void exynos_ehld_stop(void)
{
	int cpu;

	cpus_read_lock();
	for_each_online_cpu(cpu)
		exynos_ehld_stop_cpu(cpu);
	cpus_read_unlock();
}

void exynos_ehld_prepare_panic(void)
{
	if (ehld_main.dbgc.support) {
		ehld_main.dbgc.support = false;
		adv_tracer_ehld_set_enable(false);
	}
}

#if IS_ENABLED(CONFIG_HARDLOCKUP_WATCHDOG)
static int exynos_ehld_hardlockup_handler(struct notifier_block *nb,
					   unsigned long l, void *p)
{
	unsigned int i;

	exynos_ehld_event_raw_update_allcpu();
	exynos_ehld_event_raw_dump_allcpu();

	for_each_possible_cpu(i)
		ehld_info(1, "%s: cpu%u: pmu_val:%#x, ehld_stat:%#x\n",
			__func__, i,
			dbg_snapshot_get_core_pmu_val(i),
			dbg_snapshot_get_core_ehld_stat(i));

	return 0;
}

static struct notifier_block exynos_ehld_hardlockup_block = {
	.notifier_call = exynos_ehld_hardlockup_handler,
};

static void register_hardlockup_notifier_list(void)
{
	atomic_notifier_chain_register(&hardlockup_notifier_list,
					&exynos_ehld_hardlockup_block);
}
#else
static void register_hardlockup_notifier_list(void) {}
#endif

static int exynos_ehld_reboot_handler(struct notifier_block *nb,
					   unsigned long l, void *p)
{
	if (ehld_main.dbgc.support) {
		adv_tracer_ehld_set_enable(false);
		ehld_main.dbgc.support = false;
	}
	return 0;
}

static struct notifier_block exynos_ehld_reboot_block = {
	.notifier_call = exynos_ehld_reboot_handler,
};

static int exynos_ehld_panic_handler(struct notifier_block *nb,
					   unsigned long l, void *p)
{
	unsigned int i;

	exynos_ehld_event_raw_update_allcpu();
	exynos_ehld_event_raw_dump_allcpu();

	for_each_possible_cpu(i)
		printk("cpu%u: pmu_val:%#x, ehld_stat:%#x\n",
			i, dbg_snapshot_get_core_pmu_val(i),
			dbg_snapshot_get_core_ehld_stat(i));

	return 0;
}

static struct notifier_block exynos_ehld_panic_block = {
	.notifier_call = exynos_ehld_panic_handler,
};

static int exynos_ehld_c2_pm_enter_notifier(struct notifier_block *self,
						unsigned long action, void *v)
{
	unsigned int cpu = raw_smp_processor_id();

	switch (action) {
	case CPU_PM_ENTER:
		exynos_ehld_cpu_pm_enter(cpu);
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		break;
	case CPU_CLUSTER_PM_ENTER:
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
		break;
	}
	return NOTIFY_OK;
}

static int exynos_ehld_c2_pm_exit_notifier(struct notifier_block *self,
						unsigned long action, void *v)
{
	unsigned int cpu = raw_smp_processor_id();

	switch (action) {
	case CPU_PM_ENTER:
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		exynos_ehld_cpu_pm_exit(cpu);
		break;
	case CPU_CLUSTER_PM_ENTER:
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block exynos_ehld_c2_pm_enter_nb = {
	.notifier_call = exynos_ehld_c2_pm_enter_notifier,
	.priority = CORESIGHT_CPUPM_PRIORITY + 1,
};

static struct notifier_block exynos_ehld_c2_pm_exit_nb = {
	.notifier_call = exynos_ehld_c2_pm_exit_notifier,
	.priority = CORESIGHT_CPUPM_PRIORITY - 1,
};

static int exynos_ehld_pm_notifier(struct notifier_block *notifier,unsigned long pm_event, void *v)
{
	unsigned int cpu;
	unsigned long flags;

	/*
	 * We should control re-init / exit for all CPUs
	 * Originally all CPUs are controlled by cpuhp framework.
	 * But CPU0 is not controlled by cpuhp framework in exynos BSP.
	 * So mainline code of perf(kernel/cpu.c) for CPU0 is not called by cpuhp framework.
	 * As a result, it's OK to not control CPU0.
	 * CPU0 will be controlled by CPU_PM notifier call.
	 */

	if (!ehld_main.dbgc.support) {
		return NOTIFY_OK;
	}

	switch (pm_event) {

	case PM_SUSPEND_PREPARE:
		raw_spin_lock_irqsave(&ehld_main.lock, flags);

		ehld_main.pm_suspending = 1;

		if (1 == ehld_main.suspending) {
			raw_spin_unlock_irqrestore(&ehld_main.lock, flags);
			return NOTIFY_OK;
		}

		adv_tracer_ehld_set_enable(false);
		ehld_main.dbgc.enabled = false;

		ehld_main.suspending = 1;

		raw_spin_unlock_irqrestore(&ehld_main.lock, flags);
		break;

	case PM_POST_SUSPEND:
		raw_spin_lock_irqsave(&ehld_main.lock, flags);

		ehld_main.pm_suspending = 0;

		if (1 == ehld_main.sicd_suspending) {
			raw_spin_unlock_irqrestore(&ehld_main.lock, flags);
			return NOTIFY_OK;
		}

		for_each_possible_cpu(cpu) {
			dbg_snapshot_set_core_pmu_val (EHLD_VAL_PM_POST, cpu);
		}

		adv_tracer_ehld_set_enable(true);
		ehld_main.dbgc.enabled = true;

		ehld_main.suspending = 0;

		raw_spin_unlock_irqrestore(&ehld_main.lock, flags);
		break;

	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_ehld_pm_notifier_block = {
	.notifier_call = exynos_ehld_pm_notifier,
};



static int exynos_ehld_sicd_notifier(struct notifier_block *self, unsigned long sicd_event, void *v)
{
	unsigned int cpu = raw_smp_processor_id();

	unsigned long flags;

	if (!ehld_main.dbgc.support) {
		return NOTIFY_OK;
	}

	switch (sicd_event) {

	case SICD_ENTER:
		raw_spin_lock_irqsave(&ehld_main.lock, flags);

		ehld_main.sicd_suspending = 1;

		if (1 == ehld_main.suspending) {
			raw_spin_unlock_irqrestore(&ehld_main.lock, flags);
			return NOTIFY_OK;
		}

		exynos_ehld_cpu_pm_enter(cpu);
		adv_tracer_ehld_set_enable(false);
		ehld_main.dbgc.enabled = false;

		ehld_main.suspending = 1;

		raw_spin_unlock_irqrestore(&ehld_main.lock, flags);
		break;

	case SICD_EXIT:
		raw_spin_lock_irqsave(&ehld_main.lock, flags);

		ehld_main.sicd_suspending = 0;

		if (1 == ehld_main.pm_suspending) {
			raw_spin_unlock_irqrestore(&ehld_main.lock, flags);
			return NOTIFY_OK;
		}

		ehld_main.dbgc.enabled = true;

		ehld_main.suspending = 0;

		raw_spin_unlock_irqrestore(&ehld_main.lock, flags);
		break;

	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_ehld_sicd_notifier_block = {
	.notifier_call = exynos_ehld_sicd_notifier,
};


static int exynos_ehld_init_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *child;
	int ret = 0;
	unsigned int cpu = 0, offset, base;
	struct exynos_ehld_ctrl *ctrl;
	u32 val, i;
	struct property *prop;
	const __be32 *cur;

	if (of_property_read_u32(np, "cs_base", &base)) {
		ehld_info(1, "ehld: no cs_base addr in device tree\n");
		return -EINVAL;
	}
	ehld_main.cs_base = base;
	ehld_main.dbgc.use_tick_timer = true;
	ehld_main.sgi_base = devm_kcalloc(dev, num_possible_cpus(),
			sizeof(void *), GFP_KERNEL);
	if (!ehld_main.sgi_base)
		return -ENOMEM;

	i = 0;
	of_property_for_each_u32(np, "sgi_base", prop, cur, val) {
		if (i >= num_possible_cpus() || !val)
			return -EINVAL;
		ehld_main.sgi_base[i] = devm_ioremap(dev, val, SZ_4K);
		if (!ehld_main.sgi_base[i])
			return -ENOMEM;
		i++;
	}

	child = of_get_child_by_name(np, "dbgc");
	if (child) {
		ret = adv_tracer_ehld_init((void *)child);
		if (ret) {
			of_node_put(child);
			return ret;
		}

		if (!of_property_read_u32(child, "support", &base)) {
			ehld_main.dbgc.support = base;
			ehld_info(1, "Support %s dbgc\n",
					base ? "with" : "without");
			if (!of_property_read_u32(child, "interval", &base)) {
				ehld_main.dbgc.interval = base;
				ehld_info(1,
					"Support dbgc interval:%u ms\n", base);
			}
			if (!of_property_read_u32(child, "warn-count", &base)) {
				ehld_main.dbgc.warn_count = base;
				ehld_info(1,
					"Support dbgc warning count: %u\n",
					base);
			}
			if (!of_property_read_u32(child, "use-tick-timer",
								&base)) {
				ehld_main.dbgc.use_tick_timer = base;
				ehld_info(1,
					"Support using tick timer: %u\n",
					base);
			}
		}
		of_node_put(child);
	}

	for_each_possible_cpu(cpu) {
		char name[SZ_16];

		snprintf(name, sizeof(name), "cpu%u", cpu);
		child = of_get_child_by_name(np, name);

		if (!child) {
			ehld_err(1, "ehld: cpu%u's cs not found\n", cpu);
			return -EINVAL;
		}

		ret = of_property_read_u32(child, "dbg-offset", &offset);
		if (ret) {
			of_node_put(child);
			return -EINVAL;
		}

		ctrl = per_cpu_ptr(&ehld_ctrl, cpu);
		ctrl->dbg_base = ioremap(ehld_main.cs_base + offset, SZ_256K);

		if (!ctrl->dbg_base) {
			ehld_err(1, "ehld: cpu%u's dbg base not found\n", cpu);
			of_node_put(child);
			return -ENOMEM;
		}

		ehld_info(1, "ehld: cpu%u, dbg_base:%#x\n",
					cpu, ehld_main.cs_base + offset);

		of_node_put(child);
	}
	return ret;
}

static void do_smp_ehld_dump(void *data)
{
	int cpu = raw_smp_processor_id();
	struct exynos_ehld_dump *ptr = data;

	if (ptr) {
		ptr->core_pmu = exynos_ehld_read_pmu_counter();
		ptr->dss_pmu = dbg_snapshot_get_core_pmu_val(cpu);
		ptr->ehld_stat = dbg_snapshot_get_core_ehld_stat(cpu);
	}
}

static ssize_t raw_dump_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	int size = 0;
	struct exynos_ehld_dump dump;
	unsigned int cpu;

	for_each_online_cpu(cpu) {
		smp_call_function_single(cpu, do_smp_ehld_dump, &dump, 1);
		size += scnprintf(buf + size, PAGE_SIZE - size,
				  "cpu%u: core_pmu:%#08x, dss_pmu:%#08x, ehld_stat:%#08x\n",
				  cpu, dump.core_pmu, dump.dss_pmu, dump.ehld_stat);
	}

	return size;
}

static struct kobj_attribute raw_dump_attr = __ATTR_RO(raw_dump);

static struct bus_type ehld_subsys = {
	.name = "exynos-ehld",
	.dev_name = "exynos-ehld",
};

static struct attribute *ehld_sysfs_attrs[] = {
	&raw_dump_attr.attr,
	NULL,
};

static struct attribute_group ehld_sysfs_group = {
	.attrs = ehld_sysfs_attrs,
};

static const struct attribute_group *ehld_sysfs_groups[] = {
	&ehld_sysfs_group,
	NULL,
};

static int exynos_ehld_sysfs_init(void)
{
	return subsys_system_register(&ehld_subsys, ehld_sysfs_groups);
}

static int exynos_ehld_setup(void)
{
	int ret;

	/* register the PM and SICD notifiers */
	register_pm_notifier(&exynos_ehld_pm_notifier_block);
	exynos_sicd_notifier_register(&exynos_ehld_sicd_notifier_block);

	register_hardlockup_notifier_list();

	register_reboot_notifier(&exynos_ehld_reboot_block);
	atomic_notifier_chain_register(&panic_notifier_list, &exynos_ehld_panic_block);

	/* register cpu pm notifier for C2 */
	cpu_pm_register_notifier(&exynos_ehld_c2_pm_enter_nb);
	cpu_pm_register_notifier(&exynos_ehld_c2_pm_exit_nb);

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "exynos-ehld:online",
			exynos_ehld_start_cpu, exynos_ehld_stop_cpu);
	if (ret > 0)
		ret = 0;

	return ret;
}

static unsigned long noehld;
module_param(noehld, ulong, 0664);
MODULE_PARM_DESC(noehld, "disable EHLD by setting noehld=1");

static int ehld_probe(struct platform_device *pdev)
{
	int err, val;
	unsigned int cpu;
	u32 online_mask = 0;

	if (noehld == 1) {
		ehld_info(1, "ehld: disabled by module parameter: noehld=1\n");
		return 0;
	}

	err = exynos_ehld_init_dt(&pdev->dev);
	if (err) {
		ehld_err(1, "ehld: fail device tree for ehld:%d\n", err);
		return err;
	}

	err = exynos_ehld_setup();
	if (err) {
		ehld_err(1, "ehld: fail setup for ehld:%d\n", err);
		return err;
	}

	err = exynos_ehld_sysfs_init();
	if (err) {
		/* intentionally ignoring error */
		ehld_err(1, "ehld: fail init sysfs for ehld:%d\n", err);
	}

	for_each_online_cpu(cpu)
		online_mask |= BIT(cpu);

	if (ehld_main.dbgc.support) {
		val = adv_tracer_ehld_set_init_val(ehld_main.dbgc.interval,
						ehld_main.dbgc.warn_count,
						online_mask);
		if (val >= 0) {
			adv_tracer_ehld_set_enable(true);
			val = adv_tracer_ehld_get_enable();
			if (val >= 0)
				ehld_main.dbgc.enabled = val;
			else
				ehld_main.dbgc.enabled = false;
		}
	}
	return 0;
}

static const struct of_device_id ehld_dt_match[] = {
	{
		.compatible = "google,exynos-ehld",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ehld_dt_match);

static struct platform_driver ehld_driver = {
	.probe = ehld_probe,
	.driver = {
		.name = "ehld",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ehld_dt_match),
	},
};
module_platform_driver(ehld_driver);

MODULE_DESCRIPTION("Module for Detecting HW lockup at an early time");
MODULE_AUTHOR("Hosung Kim <hosung0.kim@samsung.com");
MODULE_LICENSE("GPL v2");
