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

#include <asm/stacktrace.h>
#include <soc/google/debug-snapshot.h>
#include <soc/google/pixel-suspend-diag.h>
#include "debug-snapshot-local.h"

#include <trace/events/power.h>
#include <trace/events/sched.h>
#include <trace/events/workqueue.h>

struct dbg_snapshot_log_item dss_log_items[] = {
	[DSS_LOG_TASK_ID]	= {DSS_LOG_TASK,	{0, 0, 0, true}, },
	[DSS_LOG_WORK_ID]	= {DSS_LOG_WORK,	{0, 0, 0, true}, },
	[DSS_LOG_CPUIDLE_ID]	= {DSS_LOG_CPUIDLE,	{0, 0, 0, true}, },
	[DSS_LOG_SUSPEND_ID]	= {DSS_LOG_SUSPEND,	{0, 0, 0, true}, },
	[DSS_LOG_IRQ_ID]	= {DSS_LOG_IRQ,		{0, 0, 0, false}, },
	[DSS_LOG_HRTIMER_ID]	= {DSS_LOG_HRTIMER,	{0, 0, 0, false}, },
	[DSS_LOG_CLK_ID]	= {DSS_LOG_CLK,		{0, 0, 0, true}, },
	[DSS_LOG_PMU_ID]	= {DSS_LOG_PMU,		{0, 0, 0, true}, },
	[DSS_LOG_FREQ_ID]	= {DSS_LOG_FREQ,	{0, 0, 0, true}, },
	[DSS_LOG_DM_ID]		= {DSS_LOG_DM,		{0, 0, 0, true}, },
	[DSS_LOG_REGULATOR_ID]	= {DSS_LOG_REGULATOR,	{0, 0, 0, true}, },
	[DSS_LOG_THERMAL_ID]	= {DSS_LOG_THERMAL,	{0, 0, 0, true}, },
	[DSS_LOG_ACPM_ID]	= {DSS_LOG_ACPM,	{0, 0, 0, true}, },
	[DSS_LOG_PRINTK_ID]	= {DSS_LOG_PRINTK,	{0, 0, 0, true}, },
};

/*  Internal interface variable */
struct dbg_snapshot_log_misc dss_log_misc;
static char dss_freq_name[DSS_FREQ_MAX_SIZE][DSS_FREQ_MAX_NAME_SIZE];
static unsigned int dss_freq_size;

#define dss_get_log(item)						\
long dss_get_len_##item##_log(void) {					\
	return ARRAY_SIZE(dss_log->item);				\
}									\
long dss_get_last_##item##_log_idx(void) {				\
	return (atomic_read(&dss_log_misc.item##_log_idx) - 1) &	\
			(dss_get_len_##item##_log() - 1);		\
}									\
long dss_get_first_##item##_log_idx(void) {				\
	return atomic_read(&dss_log_misc.item##_log_idx) &		\
			(dss_get_len_##item##_log() - 1);		\
}									\
struct item##_log *dss_get_last_##item##_log(void) {			\
	return &dss_log->item[dss_get_last_##item##_log_idx()];		\
}									\
struct item##_log *dss_get_first_##item##_log(void) {			\
	return &dss_log->item[dss_get_first_##item##_log_idx()];	\
}									\
struct item##_log *dss_get_##item##_log_by_idx(int idx) {		\
	if (idx < 0 || idx >= dss_get_len_##item##_log())		\
		return NULL;						\
	return &dss_log->item[idx];					\
}									\
struct item##_log *dss_get_##item##_log_iter(int idx) {			\
	if (idx < 0)							\
		idx = dss_get_len_##item##_log() - abs(idx);		\
	if (idx >= dss_get_len_##item##_log())				\
		idx -= dss_get_len_##item##_log();			\
	return &dss_log->item[idx];					\
}									\
unsigned long dss_get_vaddr_##item##_log(void) {			\
	return (unsigned long)dss_log->item;				\
}									\

#define dss_get_log_by_cpu(item)					\
long dss_get_len_##item##_log(void) {					\
	return ARRAY_SIZE(dss_log->item);				\
}									\
long dss_get_len_##item##_log_by_cpu(int cpu) {				\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return -EINVAL;						\
	return ARRAY_SIZE(dss_log->item[cpu]);				\
}									\
long dss_get_last_##item##_log_idx(int cpu) {				\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return -EINVAL;						\
	return (atomic_read(&dss_log_misc.item##_log_idx[cpu]) - 1) &	\
			(dss_get_len_##item##_log_by_cpu(cpu) - 1);	\
}									\
long dss_get_first_##item##_log_idx(int cpu) {				\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return -EINVAL;						\
	return atomic_read(&dss_log_misc.item##_log_idx[cpu]) &		\
			(dss_get_len_##item##_log_by_cpu(cpu) - 1);	\
}									\
struct item##_log *dss_get_last_##item##_log(int cpu) {			\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return NULL;						\
	return &dss_log->item[cpu][dss_get_last_##item##_log_idx(cpu)];	\
}									\
struct item##_log *dss_get_first_##item##_log(int cpu) {		\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return NULL;						\
	return &dss_log->item[cpu][dss_get_first_##item##_log_idx(cpu)];\
}									\
struct item##_log *dss_get_##item##_log_by_idx(int cpu, int idx) {	\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return NULL;						\
	if (idx < 0 || idx >= dss_get_len_##item##_log_by_cpu(cpu))	\
		return NULL;						\
	return &dss_log->item[cpu][idx];				\
}									\
struct item##_log *dss_get_##item##_log_by_cpu_iter(int cpu, int idx) {	\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return NULL;						\
	if (idx < 0)							\
		idx = dss_get_len_##item##_log_by_cpu(cpu) - abs(idx);	\
	if (idx >= dss_get_len_##item##_log_by_cpu(cpu))		\
		idx -= dss_get_len_##item##_log_by_cpu(cpu);		\
	return &dss_log->item[cpu][idx];				\
}									\
unsigned long dss_get_vaddr_##item##_log_by_cpu(int cpu) {		\
	if (cpu < 0 || cpu >= dss_get_len_##item##_log())		\
		return 0;						\
	return (unsigned long)dss_log->item[cpu];			\
}

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

void dbg_snapshot_get_freq_name(char (*freq_names)[DSS_FREQ_MAX_NAME_SIZE])
{
	int i;

	for (i = 0; i < dss_freq_size; i++)
		strlcpy(freq_names[i], dss_freq_name[i], DSS_FREQ_MAX_NAME_SIZE);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_freq_name);

unsigned int dbg_snapshot_get_freq_size(void)
{
	return dss_freq_size;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_freq_size);

void dbg_snapshot_log_output(void)
{
	unsigned long i;

	pr_info("debug-snapshot-log physical / virtual memory layout:\n");
	for (i = 0; i < ARRAY_SIZE(dss_log_items); i++) {
		if (dss_log_items[i].entry.enabled)
			pr_info("%-12s: phys:%pa / virt:%pK / size:0x%zx / en:%d\n",
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

static unsigned long dbg_snapshot_suspend(const char *log, struct device *dev,
					  int event, int en)
{
	unsigned long i = atomic_fetch_inc(&dss_log_misc.suspend_log_idx) %
		ARRAY_SIZE(dss_log->suspend);

	dss_log->suspend[i].time = local_clock();
	if (log && dev && dev->driver && dev->driver->name && strlen(dev->driver->name)) {
		dss_log->suspend[i].log = dev->driver->name;
	} else {
		dss_log->suspend[i].log = log;
	}
	dss_log->suspend[i].event = event;
	dss_log->suspend[i].dev = dev ? dev_name(dev) : "";
	dss_log->suspend[i].core = raw_smp_processor_id();
	dss_log->suspend[i].en = en;
#if IS_ENABLED(CONFIG_PIXEL_SUSPEND_DIAG)
	dss_log->suspend[i].delta_time_h = 0x0;
	dss_log->suspend[i].delta_time_l = 0x0;
#endif

	return i;
}

static void dbg_snapshot_suspend_resume(void *ignore, const char *action,
					int event, bool start)
{
	unsigned long curr_index;

	curr_index = dbg_snapshot_suspend(action, NULL, event,
					  start ? DSS_FLAG_IN : DSS_FLAG_OUT);

	if (IS_ENABLED(CONFIG_PIXEL_SUSPEND_DIAG))
		pixel_suspend_diag_suspend_resume(dss_log, action, start, curr_index);
}

void dbg_snapshot_dev_pm_cb_start(void *ignore, struct device *dev,
					const char *info, int event)
{
	dbg_snapshot_suspend(info, dev, event, DSS_FLAG_IN);
}

void dbg_snapshot_dev_pm_cb_end(void *ignore, struct device *dev, int error)
{
	int ret = 0;

	if (IS_ENABLED(CONFIG_PIXEL_SUSPEND_DIAG))
		ret = pixel_suspend_diag_dev_pm_cb_end(dss_log, dss_get_first_suspend_log_idx(),
						       dss_get_last_suspend_log_idx(), dev);

	if (!ret)
		dbg_snapshot_suspend(NULL, dev, error, DSS_FLAG_OUT);
}

static void dbg_snapshot_task(int cpu, struct task_struct *v_task)
{
	unsigned long i;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_TASK_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.task_log_idx[cpu]) %
			     ARRAY_SIZE(dss_log->task[0]);
	dss_log->task[cpu][i].time = cpu_clock(cpu);
	dss_log->task[cpu][i].task = v_task;
	dss_log->task[cpu][i].pid = v_task->pid;
	dss_log->task[cpu][i].se_exec_start = v_task->se.exec_start;
	strncpy(dss_log->task[cpu][i].task_comm, v_task->comm, TASK_COMM_LEN - 1);
}

static void dbg_snapshot_sched_switch(void *ignore, bool preempt, struct task_struct *prev,
				      struct task_struct *next, unsigned int prev_state)
{
	dbg_snapshot_task(raw_smp_processor_id(), next);
}

void dbg_snapshot_work(work_func_t fn, int en)
{
	int cpu = raw_smp_processor_id();
	unsigned long i;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_WORK_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.work_log_idx[cpu]) %
			     ARRAY_SIZE(dss_log->work[0]);
	dss_log->work[cpu][i].time = cpu_clock(cpu);
	dss_log->work[cpu][i].fn = fn;
	dss_log->work[cpu][i].en = en;
}

static void dbg_snapshot_wq_start(void *ignore, struct work_struct *work)
{
	dbg_snapshot_work(work->func, DSS_FLAG_IN);
}

static void dbg_snapshot_wq_end(void *ignore, struct work_struct *work,
				work_func_t func)
{
	dbg_snapshot_work(func, DSS_FLAG_OUT);
}

void dbg_snapshot_cpuidle_mod(char *modes, unsigned int state, s64 diff, int en)
{
	int cpu = raw_smp_processor_id();
	unsigned int i;

	if (!dss_log)
		return;
	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_CPUIDLE_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.cpuidle_log_idx[cpu]) %
		ARRAY_SIZE(dss_log->cpuidle[0]);
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
	int ret = 0;

	if (!dss_log)
		return;
	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_REGULATOR_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.regulator_log_idx) %
		ARRAY_SIZE(dss_log->regulator);

	dss_log->regulator[i].time = local_clock();
	dss_log->regulator[i].cpu = raw_smp_processor_id();
	dss_log->regulator[i].acpm_time = timestamp;
	ret = strscpy(dss_log->regulator[i].name, f_name, sizeof(dss_log->regulator[i].name));
	if (ret)
		dss_log->regulator[i].name[sizeof(dss_log->regulator[i].name) - 1] = '\0';
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

	if (!dss_log)
		return;
	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_THERMAL_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.thermal_log_idx) %
		ARRAY_SIZE(dss_log->thermal);

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

	if (!dss_log)
		return;
	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_CLK_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.clk_log_idx) %
		ARRAY_SIZE(dss_log->clk);

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

	if (!dss_log)
		return;
	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_PMU_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.pmu_log_idx) %
		ARRAY_SIZE(dss_log->pmu);

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

	if (!dss_log)
		return;
	if (unlikely(type < 0 || type > dss_freq_size))
		return;

	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_FREQ_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.freq_log_idx[type]) %
		ARRAY_SIZE(dss_log->freq[0]);

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

	if (!dss_log)
		return;
	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_DM_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.dm_log_idx) %
		ARRAY_SIZE(dss_log->dm);

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

	if (!dss_log)
		return;
	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_ACPM_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.acpm_log_idx) %
		ARRAY_SIZE(dss_log->acpm);

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

	if (!dss_log)
		return;
	if (!dbg_snapshot_is_log_item_enabled(DSS_LOG_PRINTK_ID))
		return;

	i = atomic_fetch_inc(&dss_log_misc.print_log_idx) %
		ARRAY_SIZE(dss_log->print);

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
	if (!dss_itmon)
		return;
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

	if (!dss_itmon)
		return;
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

	if (!dss_log)
		return;
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

	if (!dss_log)
		return;
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

	if (!dss_log)
		return;
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

	if (!dss_log)
		return;
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
	unsigned int nr_cpu = dbg_snapshot_get_max_core_num();

	if (!dss_log)
		return;

	if (dss_log_items[DSS_LOG_TASK_ID].entry.enabled ||
	    dss_log_items[DSS_LOG_WORK_ID].entry.enabled ||
	    dss_log_items[DSS_LOG_IRQ_ID].entry.enabled ||
	    dss_log_items[DSS_LOG_CPUIDLE_ID].entry.enabled)
		goto print_last_info;

	return;

print_last_info:
	pr_info("<last info>\n");
	for (cpu = 0; cpu < nr_cpu; cpu++) {
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

	if (!dss_log)
		return;
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

#define IRQ_THRESHOLD 50

static void dbg_snapshot_print_irq(void)
{
	int i, cpu;
	u64 sum = 0;
	struct timespec64 tp;
	unsigned long long irq_filter = 0;
	ktime_get_boottime_ts64(&tp);
	irq_filter = tp.tv_sec * IRQ_THRESHOLD;

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

		if (!irq_stat || irq_stat < irq_filter)
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
	int i;

	if (!item->entry.enabled) {
		for (i = 0; i < ARRAY_SIZE(dss_log_items); i++)
			dss_log_items[i].entry.enabled = false;
		return;
	}

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

void dbg_snapshot_register_vh_log(void)
{
	if (dss_log_items[DSS_LOG_TASK_ID].entry.enabled) {
		if (register_trace_sched_switch(dbg_snapshot_sched_switch, NULL))
			pr_err("dss task log VH register failed\n");
	}

	if (dss_log_items[DSS_LOG_WORK_ID].entry.enabled) {
		if (register_trace_workqueue_execute_start(dbg_snapshot_wq_start, NULL))
			pr_err("dss wq start log VH register failed\n");

		if (register_trace_workqueue_execute_end(dbg_snapshot_wq_end, NULL))
			pr_err("dss wq end log VH register failed\n");
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
	}
	dss_freq_size = of_property_count_strings(np, "freq_names");
	of_property_for_each_string(np, "freq_names", prop, str) {
		if (i >= ARRAY_SIZE(dss_freq_name)) {
			pr_warn("%s: DT specified more 'freq_names' than we can handle (%u vs %zu)\n",
				__func__, dss_freq_size, ARRAY_SIZE(dss_freq_name));
			break;
		}
		strlcpy(dss_freq_name[i], str, sizeof(dss_freq_name[i]));
		++i;
	}

	dbg_snapshot_log_output();
}
