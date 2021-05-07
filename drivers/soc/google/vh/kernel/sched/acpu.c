// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2022 Google LLC
 */

#include <kernel/sched/sched.h>
#include <trace/events/power.h>
#include <uapi/linux/acpu.h>

#include "sched_priv.h"

struct acpu_freq_priv {
	u64 last_update_time_ns;
	u64 last_freq;
	u64 weighted_sum;
};
static DEFINE_PER_CPU(struct acpu_freq_priv, freq_stats);

struct acpu_idle_priv {
	u64 last_update_time_ns;
	u64 current_pid;
	u64 total_idle_time_ns;
};
static DEFINE_PER_CPU(struct acpu_idle_priv, idle_stats);

static struct acpu_stats acpu_stats[NR_CPUS];
#define ACPU_STATS_SIZE (num_possible_cpus() * sizeof(struct acpu_stats))

/*
 * get_total_idle_time() and get_weighted_sum_freq() can race with the updates
 * from the trace hooks, which may create noise when reading the acpu file from
 * userspace. However, the model should be able to 'filter' the flukes. We
 * really don't want to take locks in the trace hooks as they're called from
 * scheduler hot paths. A clever cmpxchg-based scheme might improve the
 * situation, but it's not obvious it'll make a big difference, so let's keep
 * it simple for now.
 */
static u64 get_total_idle_time(int cpu, u64 t)
{
	struct acpu_idle_priv *stats = per_cpu_ptr(&idle_stats, cpu);
	u64 res = READ_ONCE(stats->total_idle_time_ns);

	if (READ_ONCE(stats->current_pid))
		return res;

	return res + (t - min(t, READ_ONCE(stats->last_update_time_ns)));
}

static u64 get_weighted_sum_freq(int cpu, u64 t)
{
	struct acpu_freq_priv *stats = per_cpu_ptr(&freq_stats, cpu);
	u64 delta = t - min(t, READ_ONCE(stats->last_update_time_ns));
	u64 sum = READ_ONCE(stats->weighted_sum);

	return sum + delta * READ_ONCE(stats->last_freq);

}

static ssize_t read_acpu(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	u64 t = ktime_get_ns();
	ssize_t read;
	int i;

	if (p >= ACPU_STATS_SIZE)
		return 0;

	for_each_possible_cpu(i) {
		acpu_stats[i].total_idle_time_ns = get_total_idle_time(i, t);
		acpu_stats[i].weighted_sum_freq = get_weighted_sum_freq(i, t);
	}

	read = min((p + count), ACPU_STATS_SIZE) - p;
	if (copy_to_user(buf, ((void *)acpu_stats + p), read))
		return -EFAULT;
	*ppos += read;

	return read;
}

static void sched_switch_cb(void *data, bool preempt, struct task_struct *prev,
			    struct task_struct *next)
{
	struct acpu_idle_priv *stats = this_cpu_ptr(&idle_stats);
	u64 t = ktime_get_ns();
	u64 delta = t - stats->last_update_time_ns;

	WRITE_ONCE(stats->current_pid, (u64)next->pid);
	WRITE_ONCE(stats->last_update_time_ns, t);
	if (!prev->pid)
		WRITE_ONCE(stats->total_idle_time_ns, stats->total_idle_time_ns + delta);
}

static void cpu_frequency_cb(void *data, unsigned int freq, unsigned int cpu)
{
	struct acpu_freq_priv *stats = per_cpu_ptr(&freq_stats, cpu);
	u64 t = ktime_get_ns();
	u64 delta = t - stats->last_update_time_ns;

	WRITE_ONCE(stats->last_update_time_ns, t);
	WRITE_ONCE(stats->weighted_sum, stats->weighted_sum + delta * stats->last_freq);
	WRITE_ONCE(stats->last_freq, freq);
}

static const struct proc_ops acpu_proc_ops = {
	.proc_read	= read_acpu,
	.proc_lseek	= default_llseek,
};

int acpu_init(void)
{
	struct proc_dir_entry *entry;
	int ret;

	ret = register_trace_sched_switch(sched_switch_cb, NULL);
	if (ret)
		return ret;

	ret = register_trace_cpu_frequency(cpu_frequency_cb, NULL);
	if (ret)
		goto fail_tp;

	entry = proc_create("acpu_stats", 0444, vendor_sched, &acpu_proc_ops);
	if (!entry) {
		ret = -EINVAL;
		goto fail_procfs;
	}
	proc_set_size(entry, ACPU_STATS_SIZE);

	return 0;

fail_procfs:
	unregister_trace_cpu_frequency(cpu_frequency_cb, NULL);
fail_tp:
	unregister_trace_sched_switch(sched_switch_cb, NULL);

	return ret;
}
