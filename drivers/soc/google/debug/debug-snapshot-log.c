// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/clk-provider.h>
#include <linux/sched/clock.h>
#include <linux/ftrace.h>
#include <linux/kernel_stat.h>
#include <linux/irqnr.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/sysfs.h>

#include <asm/stacktrace.h>
#include <soc/google/debug-snapshot.h>
#include "debug-snapshot-local.h"

#include <trace/events/power.h>

struct dbg_snapshot_log_item dss_log_items[] = {
	[DSS_LOG_TASK_ID]	= {DSS_LOG_TASK,	{0, 0, 0, false}, },
	[DSS_LOG_WORK_ID]	= {DSS_LOG_WORK,	{0, 0, 0, false}, },
	[DSS_LOG_CPUIDLE_ID]	= {DSS_LOG_CPUIDLE,	{0, 0, 0, false}, },
	[DSS_LOG_SUSPEND_ID]	= {DSS_LOG_SUSPEND,	{0, 0, 0, false}, },
	[DSS_LOG_IRQ_ID]	= {DSS_LOG_IRQ,		{0, 0, 0, false}, },
	[DSS_LOG_HRTIMER_ID]	= {DSS_LOG_HRTIMER,	{0, 0, 0, false}, },
	[DSS_LOG_CLK_ID]	= {DSS_LOG_CLK,		{0, 0, 0, false}, },
	[DSS_LOG_PMU_ID]	= {DSS_LOG_PMU,		{0, 0, 0, false}, },
	[DSS_LOG_FREQ_ID]	= {DSS_LOG_FREQ,	{0, 0, 0, false}, },
	[DSS_LOG_DM_ID]		= {DSS_LOG_DM,		{0, 0, 0, false}, },
	[DSS_LOG_REGULATOR_ID]	= {DSS_LOG_REGULATOR,	{0, 0, 0, false}, },
	[DSS_LOG_THERMAL_ID]	= {DSS_LOG_THERMAL,	{0, 0, 0, false}, },
	[DSS_LOG_ACPM_ID]	= {DSS_LOG_ACPM,	{0, 0, 0, false}, },
	[DSS_LOG_PRINTK_ID]	= {DSS_LOG_PRINTK,	{0, 0, 0, false}, },
};

struct dbg_snapshot_suspend_diag_item dss_suspend_diag_items[] = {
	[DSS_SUSPEND_SYNC_FILESYSTEMS_ID]	= {"sync_filesystems", 3 * NSEC_PER_SEC},
	[DSS_SUSPEND_FREEZE_PROCESSES_ID]	= {"freeze_processes", NSEC_PER_SEC},
	[DSS_SUSPEND_SUSPEND_ENTER_ID]		= {"suspend_enter", NSEC_PER_SEC},
	[DSS_SUSPEND_DPM_PREPARE_ID]		= {"dpm_prepare", NSEC_PER_SEC},
	[DSS_SUSPEND_DPM_SUSPEND_ID]		= {"dpm_suspend", NSEC_PER_SEC},
	[DSS_SUSPEND_DPM_SUSPEND_LATE_ID]	= {"dpm_suspend_late", NSEC_PER_SEC},
	[DSS_SUSPEND_DPM_SUSPEND_NOIRQ_ID]	= {"dpm_suspend_noirq", NSEC_PER_SEC},
	[DSS_SUSPEND_CPU_OFF_ID]		= {"cpu_off", NSEC_PER_SEC},
	[DSS_SUSPEND_SYSCORE_SUSPEND_ID]	= {"syscore_suspend", NSEC_PER_SEC},
	[DSS_SUSPEND_MACHINE_SUSPEND_ID]	= {"machine_suspend", NSEC_PER_SEC},
	[DSS_SUSPEND_SYSCORE_RESUME_ID]		= {"syscore_resume", NSEC_PER_SEC},
	[DSS_SUSPEND_CPU_ON_ID]			= {"cpu_on", NSEC_PER_SEC},
	[DSS_SUSPEND_DPM_RESUME_NOIRQ_ID]	= {"dpm_resume_noirq", NSEC_PER_SEC},
	[DSS_SUSPEND_DPM_RESUME_EARLY_ID]	= {"dpm_resume_early", NSEC_PER_SEC},
	[DSS_SUSPEND_DPM_RESUME_ID]		= {"dpm_resume", NSEC_PER_SEC},
	[DSS_SUSPEND_DPM_COMPLETE_ID]		= {"dpm_complete", NSEC_PER_SEC},
	[DSS_SUSPEND_RESUME_CONSOLE_ID]		= {"resume_console", NSEC_PER_SEC},
	[DSS_SUSPEND_THAW_PROCESSES_ID]		= {"thaw_processes", NSEC_PER_SEC},
};

/*  Internal interface variable */
struct dbg_snapshot_log_misc dss_log_misc;
static char dss_freq_name[SZ_32][SZ_8];
static unsigned int dss_freq_size;

static struct dbg_snapshot_suspend_diag suspend_diag;

#define dss_get_log(item)						\
long dss_get_len_##item##_log(void) {					\
	return ARRAY_SIZE(dss_log->item);				\
}									\
EXPORT_SYMBOL_GPL(dss_get_len_##item##_log);				\
long dss_get_last_##item##_log_idx(void) {				\
	return (atomic_read(&dss_log_misc.item##_log_idx) - 1) &	\
			(dss_get_len_##item##_log() - 1);		\
}									\
EXPORT_SYMBOL_GPL(dss_get_last_##item##_log_idx);				\
long dss_get_first_##item##_log_idx(void) {				\
	return atomic_read(&dss_log_misc.item##_log_idx) &		\
			(dss_get_len_##item##_log() - 1);		\
}									\
EXPORT_SYMBOL_GPL(dss_get_first_##item##_log_idx);				\
struct item##_log *dss_get_last_##item##_log(void) {			\
	return &dss_log->item[dss_get_last_##item##_log_idx()];		\
}									\
EXPORT_SYMBOL_GPL(dss_get_last_##item##_log);				\
struct item##_log *dss_get_first_##item##_log(void) {			\
	return &dss_log->item[dss_get_first_##item##_log_idx()];	\
}									\
EXPORT_SYMBOL_GPL(dss_get_first_##item##_log);				\
struct item##_log *dss_get_##item##_log_by_idx(int idx) {		\
	if (idx < 0 || idx >= dss_get_len_##item##_log())		\
		return NULL;						\
	return &dss_log->item[idx];					\
}									\
EXPORT_SYMBOL_GPL(dss_get_##item##_log_by_idx);				\
struct item##_log *dss_get_##item##_log_iter(int idx) {			\
	if (idx < 0)							\
		idx = dss_get_len_##item##_log() - abs(idx);		\
	if (idx >= dss_get_len_##item##_log())				\
		idx -= dss_get_len_##item##_log();			\
	return &dss_log->item[idx];					\
}									\
EXPORT_SYMBOL_GPL(dss_get_##item##_log_iter);				\
unsigned long dss_get_vaddr_##item##_log(void) {			\
	return (unsigned long)dss_log->item;				\
}									\
EXPORT_SYMBOL_GPL(dss_get_vaddr_##item##_log)

#define dss_get_log_by_cpu(item)					\
long dss_get_len_##item##_log(void) {					\
	return ARRAY_SIZE(dss_log->item);				\
}									\
EXPORT_SYMBOL_GPL(dss_get_len_##item##_log);				\
long dss_get_len_##item##_log_by_cpu(int cpu) {				\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return -EINVAL;						\
	return ARRAY_SIZE(dss_log->item[cpu]);				\
}									\
EXPORT_SYMBOL_GPL(dss_get_len_##item##_log_by_cpu);				\
long dss_get_last_##item##_log_idx(int cpu) {				\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return -EINVAL;						\
	return (atomic_read(&dss_log_misc.item##_log_idx[cpu]) - 1) &	\
			(dss_get_len_##item##_log_by_cpu(cpu) - 1);	\
}									\
EXPORT_SYMBOL_GPL(dss_get_last_##item##_log_idx);				\
long dss_get_first_##item##_log_idx(int cpu) {				\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return -EINVAL;						\
	return atomic_read(&dss_log_misc.item##_log_idx[cpu]) &		\
			(dss_get_len_##item##_log_by_cpu(cpu) - 1);	\
}									\
EXPORT_SYMBOL_GPL(dss_get_first_##item##_log_idx);				\
struct item##_log *dss_get_last_##item##_log(int cpu) {			\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return NULL;						\
	return &dss_log->item[cpu][dss_get_last_##item##_log_idx(cpu)];	\
}									\
EXPORT_SYMBOL_GPL(dss_get_last_##item##_log);				\
struct item##_log *dss_get_first_##item##_log(int cpu) {		\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return NULL;						\
	return &dss_log->item[cpu][dss_get_first_##item##_log_idx(cpu)];\
}									\
EXPORT_SYMBOL_GPL(dss_get_first_##item##_log);				\
struct item##_log *dss_get_##item##_log_by_idx(int cpu, int idx) {	\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return NULL;						\
	if (idx < 0 || idx >= dss_get_len_##item##_log_by_cpu(cpu))	\
		return NULL;						\
	return &dss_log->item[cpu][idx];				\
}									\
EXPORT_SYMBOL_GPL(dss_get_##item##_log_by_idx);				\
struct item##_log *dss_get_##item##_log_by_cpu_iter(int cpu, int idx) {	\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return NULL;						\
	if (idx < 0)							\
		idx = dss_get_len_##item##_log_by_cpu(cpu) - abs(idx);	\
	if (idx >= dss_get_len_##item##_log_by_cpu(cpu))		\
		idx -= dss_get_len_##item##_log_by_cpu(cpu);		\
	return &dss_log->item[cpu][idx];				\
}									\
EXPORT_SYMBOL_GPL(dss_get_##item##_log_by_cpu_iter);			\
unsigned long dss_get_vaddr_##item##_log_by_cpu(int cpu) {		\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return 0;						\
	return (unsigned long)dss_log->item[cpu];			\
}									\
EXPORT_SYMBOL_GPL(dss_get_vaddr_##item##_log_by_cpu)

dss_get_log_by_cpu(task);
dss_get_log_by_cpu(work);
dss_get_log_by_cpu(cpuidle);
dss_get_log_by_cpu(freq);
dss_get_log(suspend);
dss_get_log_by_cpu(irq);
dss_get_log_by_cpu(hrtimer);
dss_get_log(clk);
dss_get_log(pmu);
dss_get_log(dm);
dss_get_log(regulator);
dss_get_log(thermal);
dss_get_log(acpm);
dss_get_log(print);

#define log_item_set_filed(id, log_name)				\
		log_item = &dss_log_items[DSS_LOG_##id##_ID];		\
		log_item->entry.vaddr = (size_t)(&dss_log->log_name[0]);\
		log_item->entry.paddr = item->entry.paddr +		\
					((size_t)&dss_log->log_name[0] -\
					(size_t)&dss_log->task[0]);	\
		log_item->entry.size = sizeof(dss_log->log_name);	\
		if (!log_item->entry.paddr || !log_item->entry.vaddr	\
				|| !log_item->entry.size)		\
			log_item->entry.enabled = false

static inline bool dbg_snapshot_is_log_item_enabled(int id)
{
	bool item_enabled = dss_items[DSS_ITEM_KEVENTS_ID].entry.enabled;
	bool log_enabled = dss_log_items[id].entry.enabled;

	return dbg_snapshot_get_enable() && item_enabled && log_enabled;
}

struct dbg_snapshot_log_item *dbg_snapshot_log_get_item_by_index(int index)
{
	if (index < 0 || index > ARRAY_SIZE(dss_log_items))
		return NULL;

	return  &dss_log_items[index];
}
EXPORT_SYMBOL_GPL(dbg_snapshot_log_get_item_by_index);

int dbg_snapshot_log_get_num_items(void)
{
	return ARRAY_SIZE(dss_log_items);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_log_get_num_items);

int dbg_snapshot_get_freq_idx(const char *name)
{
	unsigned int i;

	for (i = 0; i < dss_freq_size; i++) {
		if (!strcmp(name, dss_freq_name[i]))
			return i;
	}

	return -EFAULT;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_freq_idx);

void *dbg_snapshot_get_suspend_diag(void)
{
	return (void *)&suspend_diag;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_suspend_diag);

void dbg_snapshot_log_output(void)
{
	unsigned long i;

	pr_info("debug-snapshot-log physical / virtual memory layout:\n");
	for (i = 0; i < ARRAY_SIZE(dss_log_items); i++) {
		if (dss_log_items[i].entry.enabled)
			pr_info("%-12s: phys:0x%pa / virt:0x%pK / size:0x%zx / en:%d\n",
				dss_log_items[i].name,
				&dss_log_items[i].entry.paddr,
				(void *) dss_log_items[i].entry.vaddr,
				dss_log_items[i].entry.size,
				dss_log_items[i].entry.enabled);
	}
}

void dbg_snapshot_set_enable_log_item(const char *name, int en)
{
	struct dbg_snapshot_log_item *log_item;
	int i;

	if (!name)
		return;

	for (i = 0; i < ARRAY_SIZE(dss_log_items); i++) {
		log_item = &dss_log_items[i];
		if (log_item->name && !strcmp(name, log_item->name)) {
			log_item->entry.enabled = en;
			pr_info("log item - %s is %sabled\n", name, en ? "en" : "dis");
			break;
		}
	}
}

static void dbg_snapshot_handle_suspend_diag(unsigned long last_idx, unsigned long curr_idx)
{
	unsigned long idx = (last_idx + 1) & (dss_get_len_suspend_log() - 1);
	bool has_dev_pm_cb = (idx == curr_idx) ? false : true;
	long long delta_time = 0;
	int i;

	if (!has_dev_pm_cb) {
		delta_time = dss_log->suspend[curr_idx].time - dss_log->suspend[last_idx].time;
	} else {
		/* dev_pm_cb have been run by multi cores between last_idx and curr_idx
		 * so we can't use dss_log->suspend[curr_idx].time - dss_log->suspend[last_idx].time
		 * directly to determine delta time */
		while (idx != curr_idx) {
			delta_time += (((long long)dss_log->suspend[idx].delta_time_h << 32) |
					(long long)dss_log->suspend[idx].delta_time_l);
			idx = (idx + 1) & (dss_get_len_suspend_log() - 1);
		}
	}

	for (i = 0; i < ARRAY_SIZE(dss_suspend_diag_items); i++) {
		if (!strcmp(dss_log->suspend[curr_idx].log, dss_suspend_diag_items[i].action))
			break;
	}

	if (i == ARRAY_SIZE(dss_suspend_diag_items))
		return;

	if (delta_time < dss_suspend_diag_items[i].timeout)
		return;

	if (strlen(suspend_diag.action) == 0)
		goto crash;

	if (strcmp(suspend_diag.action, dss_log->suspend[curr_idx].log))
		return;

crash:
	suspend_diag.force_panic = 0x1;
	suspend_diag.timeout = dss_suspend_diag_items[i].timeout;
	panic("%s: %s%s(%ld) to %s%s(%ld) %stake %lld ns\n", __func__,
	      dss_log->suspend[last_idx].log ? dss_log->suspend[last_idx].log : "",
	      dss_log->suspend[last_idx].en == DSS_FLAG_IN ? " IN" : " OUT", last_idx,
	      dss_log->suspend[curr_idx].log ? dss_log->suspend[curr_idx].log : "",
	      dss_log->suspend[curr_idx].en == DSS_FLAG_IN ? " IN" : " OUT", curr_idx,
	      has_dev_pm_cb ? "callbacks " : "",
	      delta_time);
}

static unsigned long dbg_snapshot_suspend(const char *log, struct device *dev,
				int event, int en)
{
	unsigned long i = atomic_fetch_inc(&dss_log_misc.suspend_log_idx) &
		(ARRAY_SIZE(dss_log->suspend) - 1);

	dss_log->suspend[i].time = local_clock();
	dss_log->suspend[i].log = log ? log : NULL;
	dss_log->suspend[i].event = event;
	dss_log->suspend[i].dev = dev ? dev_name(dev) : "";
	dss_log->suspend[i].core = raw_smp_processor_id();
	dss_log->suspend[i].en = en;
	dss_log->suspend[i].delta_time_h = 0x0;
	dss_log->suspend[i].delta_time_l = 0x0;

	return i;
}

static void dbg_snapshot_suspend_resume(void *ignore, const char *action,
					int event, bool start)
{
	suspend_diag.curr_index =
		dbg_snapshot_suspend(action, NULL, event, start ? DSS_FLAG_IN : DSS_FLAG_OUT);

	if (!suspend_diag.enable)
		return;

	if (start || !action)
		goto backup;

	dbg_snapshot_handle_suspend_diag(suspend_diag.last_index, suspend_diag.curr_index);

backup:
	suspend_diag.last_index = suspend_diag.curr_index;
}

void dbg_snapshot_dev_pm_cb_start(void *ignore, struct device *dev,
					const char *info, int event)
{
	dbg_snapshot_suspend(info, dev, event, DSS_FLAG_IN);
}

void dbg_snapshot_dev_pm_cb_end(void *ignore, struct device *dev, int error)
{
	unsigned long i;
	unsigned long long end_time;
	long long delta_time;

	if (!suspend_diag.enable) {
		dbg_snapshot_suspend(NULL, dev, error, DSS_FLAG_OUT);
		return;
	}

	end_time = local_clock();

	i = dss_get_last_suspend_log_idx();
	while(i != dss_get_first_suspend_log_idx()) {
		if (dev && end_time >= dss_log->suspend[i].time &&
			dss_log->suspend[i].dev == dev_name(dev)) {
			delta_time = end_time - dss_log->suspend[i].time;
			dss_log->suspend[i].delta_time_h = (delta_time >> 32) & 0xFFFF;
			dss_log->suspend[i].delta_time_l = delta_time & 0xFFFFFFFF;
			break;
		}
		i = (i - 1) & (dss_get_len_suspend_log() - 1);
	}
}

void dbg_snapshot_cpuidle_mod(char *modes, unsigned int state, s64 diff, int en)
{
	int cpu = raw_smp_processor_id();
	unsigned int i;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_CPUIDLE_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.cpuidle_log_idx[cpu]) &
		(ARRAY_SIZE(dss_log->cpuidle[0]) - 1);
	dss_log->cpuidle[cpu][i].time = local_clock();
	dss_log->cpuidle[cpu][i].modes = modes;
	dss_log->cpuidle[cpu][i].state = state;
	dss_log->cpuidle[cpu][i].num_online_cpus = num_online_cpus();
	dss_log->cpuidle[cpu][i].delta = (int)diff;
	dss_log->cpuidle[cpu][i].en = en;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_cpuidle_mod);

void dbg_snapshot_regulator(unsigned long long timestamp, char *f_name,
			unsigned int addr, unsigned int volt,
			unsigned int rvolt, int en)
{
	unsigned long i;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_REGULATOR_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.regulator_log_idx) &
		(ARRAY_SIZE(dss_log->regulator) - 1);

	dss_log->regulator[i].time = local_clock();
	dss_log->regulator[i].cpu = raw_smp_processor_id();
	dss_log->regulator[i].acpm_time = timestamp;
	strncpy(dss_log->regulator[i].name, f_name,
			min_t(int, strlen(f_name), SZ_16 - 1));
	dss_log->regulator[i].reg = addr;
	dss_log->regulator[i].en = en;
	dss_log->regulator[i].voltage = volt;
	dss_log->regulator[i].raw_volt = rvolt;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_regulator);

void dbg_snapshot_thermal(struct exynos_tmu_data *data, unsigned int temp,
			 char *name, unsigned long long max_cooling)
{
	unsigned long i;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_THERMAL_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.thermal_log_idx) &
		(ARRAY_SIZE(dss_log->thermal) - 1);

	dss_log->thermal[i].time = local_clock();
	dss_log->thermal[i].cpu = raw_smp_processor_id();
	dss_log->thermal[i].data = data;
	dss_log->thermal[i].temp = temp;
	dss_log->thermal[i].cooling_device = name;
	dss_log->thermal[i].cooling_state = max_cooling;

}
EXPORT_SYMBOL_GPL(dbg_snapshot_thermal);

void dbg_snapshot_clk(struct clk_hw *clock, const char *func_name,
		     unsigned long arg, int mode)
{
	unsigned long i;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_CLK_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.clk_log_idx) &
		(ARRAY_SIZE(dss_log->clk) - 1);

	dss_log->clk[i].time = local_clock();
	dss_log->clk[i].mode = mode;
	dss_log->clk[i].arg = arg;
	dss_log->clk[i].clk = clock;
	dss_log->clk[i].f_name = func_name;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_clk);

void dbg_snapshot_pmu(int id, const char *func_name, int mode)
{
	unsigned long i;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_PMU_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.pmu_log_idx) &
		(ARRAY_SIZE(dss_log->pmu) - 1);

	dss_log->pmu[i].time = local_clock();
	dss_log->pmu[i].mode = mode;
	dss_log->pmu[i].id = id;
	dss_log->pmu[i].f_name = func_name;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_pmu);

void dbg_snapshot_freq(int type, unsigned long old_freq,
			unsigned long target_freq, int en)
{
	unsigned long i;

	if (unlikely(type < 0 || type > dss_freq_size))
		return;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_FREQ_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.freq_log_idx[type]) &
		(ARRAY_SIZE(dss_log->freq[0]) - 1);

	dss_log->freq[type][i].time = local_clock();
	dss_log->freq[type][i].cpu = raw_smp_processor_id();
	dss_log->freq[type][i].freq_name = dss_freq_name[type];
	dss_log->freq[type][i].old_freq = old_freq;
	dss_log->freq[type][i].target_freq = target_freq;
	dss_log->freq[type][i].en = en;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_freq);

void dbg_snapshot_dm(int type, unsigned long min, unsigned long max,
			s32 wait_t, s32 t)
{
	unsigned long i;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_DM_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.dm_log_idx) &
		(ARRAY_SIZE(dss_log->dm) - 1);

	dss_log->dm[i].time = local_clock();
	dss_log->dm[i].cpu = raw_smp_processor_id();
	dss_log->dm[i].dm_num = type;
	dss_log->dm[i].min_freq = min;
	dss_log->dm[i].max_freq = max;
	dss_log->dm[i].wait_dmt = wait_t;
	dss_log->dm[i].do_dmt = t;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_dm);

void dbg_snapshot_acpm(unsigned long long timestamp, const char *log,
			unsigned int data)
{
	unsigned long i;
	int len;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_ACPM_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.acpm_log_idx) &
		(ARRAY_SIZE(dss_log->acpm) - 1);

	dss_log->acpm[i].time = local_clock();
	dss_log->acpm[i].acpm_time = timestamp;
	len = sizeof(dss_log->acpm[i].log) - 1;
	strncpy(dss_log->acpm[i].log, log, len);
	dss_log->acpm[i].log[len] = '\0';
	dss_log->acpm[i].data = data;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_acpm);

void dbg_snapshot_printk(const char *fmt, ...)
{
	unsigned long i;
	va_list args;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_PRINTK_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.print_log_idx) &
		(ARRAY_SIZE(dss_log->print) - 1);

	va_start(args, fmt);
	vsnprintf(dss_log->print[i].log, sizeof(dss_log->print[i].log),
			fmt, args);
	va_end(args);

	dss_log->print[i].time = local_clock();
	dss_log->print[i].cpu = raw_smp_processor_id();
}
EXPORT_SYMBOL_GPL(dbg_snapshot_printk);

void dbg_snapshot_itmon_irq_received(void)
{
	if (!dss_items[DSS_ITEM_ITMON_ID].entry.enabled)
		return;

	dss_itmon->magic = DSS_ITMON_MAGIC_IRQ_RECEIVED;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_itmon_irq_received);

void dbg_snapshot_itmon_backup_log(const char *fmt, ...)
{
	static atomic_t len_logs_a = ATOMIC_INIT(0);
	size_t len_log, len_logs;
	va_list args;

	if (!dss_items[DSS_ITEM_ITMON_ID].entry.enabled)
		return;

	va_start(args, fmt);
	len_log = vsnprintf(NULL, 0, fmt, args);
	va_end(args);

	len_log += 1;
	len_logs = atomic_add_return(len_log, &len_logs_a);

	if (len_logs >= DSS_ITMON_LOG_MAX_LEN) {
		dss_items[DSS_ITEM_ITMON_ID].entry.enabled = false;
		return;
	}

	va_start(args, fmt);
	vsnprintf(dss_itmon->log + len_logs - len_log, len_log, fmt, args);
	va_end(args);

	*(dss_itmon->log + len_logs - 1) = ' ';
}
EXPORT_SYMBOL_GPL(dbg_snapshot_itmon_backup_log);

static inline void dbg_snapshot_get_sec(unsigned long long ts,
					unsigned long *sec, unsigned long *msec)
{
	*sec = ts / NSEC_PER_SEC;
	*msec = (ts % NSEC_PER_SEC) / USEC_PER_MSEC;
}

static void dbg_snapshot_print_last_irq(int cpu)
{
	struct dbg_snapshot_log_item *log_item = &dss_log_items[DSS_LOG_IRQ_ID];
	unsigned long idx, sec, msec;

	if (!log_item->entry.enabled)
		return;

	idx = dss_get_last_irq_log_idx(cpu);
	dbg_snapshot_get_sec(dss_log->irq[cpu][idx].time, &sec, &msec);

	pr_info("%-12s: [%4ld] %10lu.%06lu sec, %10s: %pS, %8s: %8d, %10s: %2d, %s\n",
			">>> irq", idx, sec, msec,
			"handler", dss_log->irq[cpu][idx].fn,
			"irq", dss_log->irq[cpu][idx].irq,
			"en", dss_log->irq[cpu][idx].en,
			(dss_log->irq[cpu][idx].en == 1) ? "[Mismatch]" : "");
}

static void dbg_snapshot_print_last_task(int cpu)
{
	struct dbg_snapshot_log_item *log_item = &dss_log_items[DSS_LOG_TASK_ID];
	unsigned long idx, sec, msec;
	struct task_struct *task;

	if (!log_item->entry.enabled)
		return;

	idx = dss_get_last_task_log_idx(cpu);
	dbg_snapshot_get_sec(dss_log->task[cpu][idx].time, &sec, &msec);
	task = dss_log->task[cpu][idx].task;

	pr_info("%-12s: [%4lu] %10lu.%06lu sec, %10s: %-16s, %8s: 0x%-16px, %10s: %16llu\n",
			">>> task", idx, sec, msec,
			"task_comm", (task) ? task->comm : "NULL",
			"task", task,
			"exec_start", (task) ? task->se.exec_start : 0);
}

static void dbg_snapshot_print_last_work(int cpu)
{
	struct dbg_snapshot_log_item *log_item = &dss_log_items[DSS_LOG_WORK_ID];
	unsigned long idx, sec, msec;

	if (!log_item->entry.enabled)
		return;

	idx = dss_get_last_work_log_idx(cpu);
	dbg_snapshot_get_sec(dss_log->work[cpu][idx].time, &sec, &msec);

	pr_info("%-12s: [%4lu] %10lu.%06lu sec, %10s: %pS, %3s: %3d %s\n",
			">>> work", idx, sec, msec,
			"work_fn", dss_log->work[cpu][idx].fn,
			"en", dss_log->work[cpu][idx].en,
			(dss_log->work[cpu][idx].en == 1) ? "[Mismatch]" : "");
}

static void dbg_snapshot_print_last_cpuidle(int cpu)
{
	struct dbg_snapshot_log_item *log_item = &dss_log_items[DSS_LOG_CPUIDLE_ID];
	unsigned long idx, sec, msec;

	if (!log_item->entry.enabled)
		return;

	idx = dss_get_last_cpuidle_log_idx(cpu);
	dbg_snapshot_get_sec(dss_log->cpuidle[cpu][idx].time, &sec, &msec);

	pr_info("%-12s: [%4lu] %10lu.%06lu sec, %10s: %s, %s: %d, %s: %d, %s: %d, %s: %d %s\n",
			">>> cpuidle", idx, sec, msec,
			"modes", dss_log->cpuidle[cpu][idx].modes,
			"state", dss_log->cpuidle[cpu][idx].state,
			"stay time", dss_log->cpuidle[cpu][idx].delta,
			"online_cpus", dss_log->cpuidle[cpu][idx].num_online_cpus,
			"en", dss_log->cpuidle[cpu][idx].en,
			(dss_log->cpuidle[cpu][idx].en == 1) ? "[Mismatch]" : "");
}

static void dbg_snapshot_print_lastinfo(void)
{
	int cpu;

	pr_info("<last info>\n");
	for (cpu = 0; cpu < DSS_NR_CPUS; cpu++) {
		pr_info("CPU ID: %d ----------------------------------\n", cpu);
		dbg_snapshot_print_last_task(cpu);
		dbg_snapshot_print_last_work(cpu);
		dbg_snapshot_print_last_irq(cpu);
		dbg_snapshot_print_last_cpuidle(cpu);
	}
}

static void dbg_snapshot_print_freqinfo(void)
{
	struct dbg_snapshot_log_item *log_item = &dss_log_items[DSS_LOG_FREQ_ID];
	unsigned long i, idx, sec, msec;
	unsigned long old_freq, target_freq;

	if (!log_item->entry.enabled)
		return;

	pr_info("\n<last freq info>\n");
	for (i = 0; i < dss_freq_size; i++) {
		if (!atomic_read(&dss_log_misc.freq_log_idx[i])) {
			pr_info("%10s: no information\n", dss_freq_name[i]);
			continue;
		}
		idx = dss_get_last_freq_log_idx(i);
		dbg_snapshot_get_sec(dss_log->freq[i][idx].time, &sec, &msec);
		old_freq = dss_log->freq[i][idx].old_freq;
		target_freq = dss_log->freq[i][idx].target_freq;
		pr_info("%10s[%4lu]: %10lu.%06lu sec, %12s: %5luMhz, %12s: %5luMhz, %3s: %d %s\n",
				dss_freq_name[i], idx, sec, msec,
				"old_freq", old_freq / 1000,
				"target_freq", target_freq / 1000,
				"en", dss_log->freq[i][idx].en,
				(dss_log->freq[i][idx].en == 1) ? "[Mismatch]" : "");
	}
}

#ifndef arch_irq_stat
#define arch_irq_stat() 0
#endif

static void dbg_snapshot_print_irq(void)
{
	int i, cpu;
	u64 sum = 0;

	for_each_possible_cpu(cpu)
		sum += kstat_cpu_irqs_sum(cpu);

	sum += arch_irq_stat();

	pr_info("<irq info>\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("sum irq : %llu", sum);
	pr_info("----------------------------------------------------------\n");

	for_each_irq_nr(i) {
		struct irq_data *data;
		struct irq_desc *desc;
		unsigned int irq_stat = 0;
		const char *name;

		data = irq_get_irq_data(i);
		if (!data)
			continue;

		desc = irq_data_to_desc(data);

		for_each_possible_cpu(cpu)
			irq_stat += *per_cpu_ptr(desc->kstat_irqs, cpu);

		if (!irq_stat)
			continue;

		if (desc->action && desc->action->name)
			name = desc->action->name;
		else
			name = "???";
		pr_info("irq-%-4d(hwirq-%-3d) : %8u %s\n",
			i, (int)desc->irq_data.hwirq, irq_stat, name);
	}
}

void dbg_snapshot_print_log_report(void)
{
	if (unlikely(!dbg_snapshot_get_enable()))
		return;

	pr_info("==========================================================\n");
	pr_info("Panic Report\n");
	pr_info("==========================================================\n");
	dbg_snapshot_print_lastinfo();
	dbg_snapshot_print_freqinfo();
	dbg_snapshot_print_irq();
	pr_info("==========================================================\n");
}

void dbg_snapshot_init_log(void)
{
	struct dbg_snapshot_item *item = &dss_items[DSS_ITEM_KEVENTS_ID];
	struct dbg_snapshot_log_item *log_item;

	log_item_set_filed(TASK, task);
	log_item_set_filed(WORK, work);
	log_item_set_filed(CPUIDLE, cpuidle);
	log_item_set_filed(IRQ, irq);
	log_item_set_filed(HRTIMER, hrtimer);
	log_item_set_filed(FREQ, freq);
	log_item_set_filed(SUSPEND, suspend);
	log_item_set_filed(CLK, clk);
	log_item_set_filed(PMU, pmu);
	log_item_set_filed(DM, dm);
	log_item_set_filed(REGULATOR, regulator);
	log_item_set_filed(THERMAL, thermal);
	log_item_set_filed(ACPM, acpm);
	log_item_set_filed(PRINTK, print);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);

	if (!ret)
		suspend_diag.enable = val;

	return count;
}

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%sable\n", !!suspend_diag.enable ? "en" : "dis");
}

static struct kobj_attribute suspend_diag_attr_enable = __ATTR_RW_MODE(enable, 0660);

static ssize_t timeout_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
					size_t count)
{
	int i;
	char item_action[32];
	char *item_timeout, *space;
	int action_size;
	unsigned long long val;

	/*
	 * Extract buf before the first space.
	 */
	space = strchr(buf, ' ');
	if (!space) {
		pr_warn("invalid parameters format in buffer [%s]!\n", buf);
		return -EINVAL;
	}

	action_size = space - buf + 1;
	item_timeout = space + 1;
	if (action_size > sizeof(item_action)) {
		pr_warn("invalid action parameter in buffer [%s]!\n", buf);
		return -EINVAL;
	}
	strlcpy(item_action, buf, action_size);

	if (kstrtoll(item_timeout, 10, &val)) {
		pr_warn("invalid timeout parameter in buffer [%s]!\n", buf);
		return -EINVAL;
	}

	if (!strcmp(item_action, "all")) {
		for (i = 0; i < ARRAY_SIZE(dss_suspend_diag_items); i++) {
			dss_suspend_diag_items[i].timeout = val;
		}
		return count;
	}

	for (i = 0; i < ARRAY_SIZE(dss_suspend_diag_items); i++) {
		if (!strcmp(item_action, dss_suspend_diag_items[i].action)) {
			dss_suspend_diag_items[i].timeout = val;
			return count;
		}
	}

	pr_warn("item action doesn't exist in default list [%s]!\n", item_action);
	return -EEXIST;
}

static ssize_t timeout_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;
	ssize_t size = 0;

	for (i = 0; i < ARRAY_SIZE(dss_suspend_diag_items); i++) {
		size += scnprintf(buf + size, PAGE_SIZE - size, "%s: %llu(ns)\n",
				  dss_suspend_diag_items[i].action,
				  dss_suspend_diag_items[i].timeout);
	}

	return size;
}

static struct kobj_attribute suspend_diag_attr_timeout = __ATTR_RW_MODE(timeout, 0660);

static ssize_t action_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
					size_t count)
{
	char *newline = NULL;

	strlcpy(suspend_diag.action, buf, sizeof(suspend_diag.action));
	newline = strchr(suspend_diag.action, '\n');
	if (newline)
		*newline = '\0';

	return count;
}

static ssize_t action_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", suspend_diag.action);
}

static struct kobj_attribute suspend_diag_attr_action = __ATTR_RW_MODE(action, 0660);

static struct attribute *suspend_diag_attrs[] = {
	&suspend_diag_attr_enable.attr,
	&suspend_diag_attr_timeout.attr,
	&suspend_diag_attr_action.attr,
	NULL
};

static const struct attribute_group suspend_diag_attr_group = {
	.attrs = suspend_diag_attrs,
	.name = "suspend_diag"
};

static void dbg_snapshot_add_suspend_diag_attributes(void)
{
	struct kobject *dbg_snapshot_kobj;

	dbg_snapshot_kobj = kobject_create_and_add("dbg_snapshot", kernel_kobj);
	if (!dbg_snapshot_kobj) {
		dev_emerg(dss_desc.dev, "cannot create kobj for dbg_snapshot!\n");
		return;
	}

	if (sysfs_create_group(dbg_snapshot_kobj, &suspend_diag_attr_group)) {
		dev_emerg(dss_desc.dev, "cannot create files in ../dbg_snapshot/suspend_diag!\n");
		kobject_put(dbg_snapshot_kobj);
	}
}

void dbg_snapshot_start_log(void)
{
	struct property *prop;
	const char *str;
	unsigned int i = 0;

	struct device_node *np = dss_desc.dev->of_node;

	if (dbg_snapshot_is_log_item_enabled(DSS_LOG_SUSPEND_ID)) {
		register_trace_suspend_resume(dbg_snapshot_suspend_resume, NULL);
		register_trace_device_pm_callback_start(dbg_snapshot_dev_pm_cb_start, NULL);
		register_trace_device_pm_callback_end(dbg_snapshot_dev_pm_cb_end, NULL);
		dbg_snapshot_add_suspend_diag_attributes();
	}
	dss_freq_size = of_property_count_strings(np, "freq_names");
	of_property_for_each_string(np, "freq_names", prop, str) {
		strlcpy(dss_freq_name[i++], str, sizeof(dss_freq_name) - 1);
	}

	dbg_snapshot_log_output();
}
