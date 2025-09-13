// SPDX-License-Identifier: GPL-2.0-only
/* drivers/soc/google/kernel-top.c
 *
 * Copyright (C) 2018 Google, Inc.
 */

#define dev_fmt(fmt)	"TOP: " fmt

#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/threads.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <asm/irq_regs.h>
#include <linux/kernel-top.h>
#include <linux/sched/cputime.h>
#include <linux/sched/debug.h>
#include <linux/sched/signal.h>

#define NUM_BUSY_TASK_CHECK 5

struct kernel_top_context {
	struct device *owner;
	u64 *prev_tasktics_array;
	u64 *frame_tasktics_array;
	pid_t *curr_task_pid_array;
	pid_t top_task_pid_array[NUM_BUSY_TASK_CHECK];
	struct task_struct **task_ptr_array;
	struct kernel_cpustat curr_all_cpustat;
	struct kernel_cpustat prev_all_cpustat;
	u64 frame_cpustat_total;
};

/* Clone from fs/proc/stat.c. */
#ifdef arch_idle_time

static u64 ktop_get_idle_time(struct kernel_cpustat *kcs, int cpu)
{
	u64 idle;

	idle = kcs->cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);
	return idle;
}

static u64 get_iowait_time(struct kernel_cpustat *kcs, int cpu)
{
	u64 iowait;

	iowait = kcs->cpustat[CPUTIME_IOWAIT];
	if (cpu_online(cpu) && nr_iowait_cpu(cpu))
		iowait += arch_idle_time(cpu);
	return iowait;
}

#else

static u64 ktop_get_idle_time(struct kernel_cpustat *kcs, int cpu)
{
	u64 idle, idle_usecs = -1ULL;

	if (cpu_online(cpu))
		idle_usecs = get_cpu_idle_time_us(cpu, NULL);

	if (idle_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.idle */
		idle = kcs->cpustat[CPUTIME_IDLE];
	else
		idle = idle_usecs * NSEC_PER_USEC;

	return idle;
}

static u64 get_iowait_time(struct kernel_cpustat *kcs, int cpu)
{
	u64 iowait, iowait_usecs = -1ULL;

	if (cpu_online(cpu))
		iowait_usecs = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
		iowait = kcs->cpustat[CPUTIME_IOWAIT];
	else
		iowait = iowait_usecs * NSEC_PER_USEC;

	return iowait;
}

#endif

static void get_all_cpustat(struct kernel_cpustat *total_cpu_stat)
{
	int cpu;

	if (!total_cpu_stat)
		return;

	memset(total_cpu_stat, 0, sizeof(struct kernel_cpustat));

#ifdef CONFIG_SMP
	for_each_possible_cpu(cpu) {
		struct kernel_cpustat kcpustat;
		u64 *cpustat = kcpustat.cpustat;

		kcpustat_cpu_fetch(&kcpustat, cpu);

		total_cpu_stat->cpustat[CPUTIME_USER]		+= cpustat[CPUTIME_USER];
		total_cpu_stat->cpustat[CPUTIME_NICE]		+= cpustat[CPUTIME_NICE];
		total_cpu_stat->cpustat[CPUTIME_SYSTEM]		+= cpustat[CPUTIME_SYSTEM];
		total_cpu_stat->cpustat[CPUTIME_IDLE]		+= ktop_get_idle_time(&kcpustat,
										      cpu);
		total_cpu_stat->cpustat[CPUTIME_IOWAIT]		+= get_iowait_time(&kcpustat, cpu);
		total_cpu_stat->cpustat[CPUTIME_IRQ]		+= cpustat[CPUTIME_IRQ];
		total_cpu_stat->cpustat[CPUTIME_SOFTIRQ]	+= cpustat[CPUTIME_SOFTIRQ];
		total_cpu_stat->cpustat[CPUTIME_STEAL]		+= cpustat[CPUTIME_STEAL];
		total_cpu_stat->cpustat[CPUTIME_GUEST]		+= cpustat[CPUTIME_GUEST];
		total_cpu_stat->cpustat[CPUTIME_GUEST_NICE]	+= cpustat[CPUTIME_GUEST_NICE];
	}
#endif
}

static void sort_top_tasks(u64 *frame_tasktics_array,
	pid_t *curr_task_pid_array, int task_count, pid_t *result)
{
	int i = 0, j = 0, k = 0;
	int pid_checked = 0;
	pid_t p = 0;

	for (i = 0; i < NUM_BUSY_TASK_CHECK; i++) {
		result[i] = 0;
		/* Find the task which has the largest cputime in this frame */
		if (i == 0) {
			for (j = 0; j < task_count; j++) {
				p = curr_task_pid_array[j];

				if (frame_tasktics_array[result[i]] <
					frame_tasktics_array[p])
					result[i] = p;
			}
		} else {
			for (j = 0; j < task_count; j++) {
				p = curr_task_pid_array[j];
				for (k = 0; k < i; k++) {
					if (result[k] == p) {
						pid_checked = 1;
						break;
					}
				}
				if (pid_checked) {
					pid_checked = 0;
					continue;
				}
				if (frame_tasktics_array[result[i]] <
					frame_tasktics_array[p])
					result[i] = p;
			}
		}
	}
}

static u64 cal_frame_cpustat_total(struct kernel_cpustat curr_all_cpustat,
	struct kernel_cpustat prev_all_cpustat)
{
	u64 user_time = 0, system_time = 0, io_time = 0;
	u64 irq_time = 0, idle_time = 0;

	user_time = ((curr_all_cpustat.cpustat[CPUTIME_USER] +
				curr_all_cpustat.cpustat[CPUTIME_NICE]) -
				(prev_all_cpustat.cpustat[CPUTIME_USER] +
				prev_all_cpustat.cpustat[CPUTIME_NICE]));
	system_time = (curr_all_cpustat.cpustat[CPUTIME_SYSTEM] -
				prev_all_cpustat.cpustat[CPUTIME_SYSTEM]);
	io_time = (curr_all_cpustat.cpustat[CPUTIME_IOWAIT] -
				prev_all_cpustat.cpustat[CPUTIME_IOWAIT]);
	irq_time = ((curr_all_cpustat.cpustat[CPUTIME_IRQ] +
				curr_all_cpustat.cpustat[CPUTIME_SOFTIRQ]) -
				(prev_all_cpustat.cpustat[CPUTIME_IRQ] +
				prev_all_cpustat.cpustat[CPUTIME_SOFTIRQ]));
	idle_time = ((curr_all_cpustat.cpustat[CPUTIME_IDLE] >
				prev_all_cpustat.cpustat[CPUTIME_IDLE]) ?
				curr_all_cpustat.cpustat[CPUTIME_IDLE] -
				prev_all_cpustat.cpustat[CPUTIME_IDLE] : 0);
	idle_time += ((curr_all_cpustat.cpustat[CPUTIME_STEAL] +
				curr_all_cpustat.cpustat[CPUTIME_GUEST]) -
				(prev_all_cpustat.cpustat[CPUTIME_STEAL] +
				prev_all_cpustat.cpustat[CPUTIME_GUEST]));

	return (user_time + system_time + io_time + irq_time + idle_time);
}

static void kernel_top_cal(struct kernel_top_context *cxt)
{
	int task_count = 0;
	struct task_struct *tsk;

	/* Calculate each tasks tics in this time frame*/
	rcu_read_lock();
	for_each_process(tsk) {
		u64 utime, stime;
		thread_group_cputime_adjusted(tsk, &utime, &stime);
		if (tsk->pid < PID_MAX_DEFAULT) {
			u64 cur_tasktics = (utime + stime);

			cxt->frame_tasktics_array[tsk->pid] =
				cur_tasktics -
					cxt->prev_tasktics_array[tsk->pid];

			cxt->prev_tasktics_array[tsk->pid] = cur_tasktics;
			cxt->task_ptr_array[tsk->pid] = tsk;

			if (cxt->frame_tasktics_array[tsk->pid] > 0) {
				cxt->curr_task_pid_array[task_count] = tsk->pid;
				task_count++;
			}
		}
	}
	rcu_read_unlock();

	get_all_cpustat(&cxt->curr_all_cpustat);

	sort_top_tasks(cxt->frame_tasktics_array,
		cxt->curr_task_pid_array, task_count, cxt->top_task_pid_array);

	cxt->frame_cpustat_total =
		cal_frame_cpustat_total(cxt->curr_all_cpustat,
			cxt->prev_all_cpustat);
	memcpy(&cxt->prev_all_cpustat,
		&cxt->curr_all_cpustat, sizeof(struct kernel_cpustat));

}

static void kernel_top_show(struct kernel_top_context *cxt)
{
	pid_t top_n_pid = 0;
	int i;

	dev_info(cxt->owner, "CPU Usage     PID     Name\n");
	for (i = 0; i < NUM_BUSY_TASK_CHECK; i++) {
		if (cxt->frame_cpustat_total > 0) {
			top_n_pid = cxt->top_task_pid_array[i];
			dev_info(cxt->owner, "%8llu%%%8d     %s%10llu\n",
				cxt->frame_tasktics_array[top_n_pid] * 100 /
					cxt->frame_cpustat_total,
				top_n_pid, cxt->task_ptr_array[top_n_pid]->comm,
			nsec_to_clock_t(cxt->frame_tasktics_array[top_n_pid]));
		}
	}

	memset(cxt->frame_tasktics_array, 0, sizeof(u64) * PID_MAX_DEFAULT);
	memset(cxt->task_ptr_array, 0,
		sizeof(struct task_struct *) * PID_MAX_DEFAULT);
	memset(cxt->curr_task_pid_array, 0, sizeof(pid_t) * PID_MAX_DEFAULT);
}

void kernel_top_print(struct kernel_top_context *cxt)
{
	struct timespec64 ts;
	struct rtc_time tm;

	kernel_top_cal(cxt);
	kernel_top_show(cxt);

	ktime_get_real_ts64(&ts);
	rtc_time64_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	dev_info(cxt->owner, "Kernel Top Statistic done (%02d-%02d %02d:%02d:%02d)\n",
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}
EXPORT_SYMBOL_GPL(kernel_top_print);

int kernel_top_init(struct device *dev, struct kernel_top_context **pcxt)
{
	struct kernel_top_context *cxt;

	cxt = devm_kzalloc(dev, sizeof(*cxt), GFP_KERNEL);
	if (!cxt)
		return -ENOMEM;

	cxt->prev_tasktics_array = vmalloc(sizeof(u64) * PID_MAX_DEFAULT);
	if (!cxt->prev_tasktics_array)
		goto err_alloc;
	cxt->frame_tasktics_array = vmalloc(sizeof(u64) * PID_MAX_DEFAULT);
	if (!cxt->frame_tasktics_array)
		goto err_alloc;
	cxt->task_ptr_array = vmalloc(sizeof(struct task_struct *) * PID_MAX_DEFAULT);
	if (!cxt->task_ptr_array)
		goto err_alloc;
	cxt->curr_task_pid_array = vmalloc(sizeof(pid_t) * PID_MAX_DEFAULT);
	if (!cxt->curr_task_pid_array)
		goto err_alloc;

	*pcxt = cxt;
	cxt->owner = dev;
	kernel_top_reset(cxt);
	return 0;

err_alloc:
	kernel_top_destroy(cxt);

	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(kernel_top_init);

void kernel_top_reset(struct kernel_top_context *cxt)
{
	struct task_struct *tsk;
	struct timespec64 ts;
	struct rtc_time tm;

	memset(cxt->prev_tasktics_array, 0, sizeof(u64) * PID_MAX_DEFAULT);
	memset(cxt->frame_tasktics_array, 0, sizeof(u64) * PID_MAX_DEFAULT);
	memset(cxt->task_ptr_array, 0, sizeof(struct task_struct *) * PID_MAX_DEFAULT);
	memset(cxt->curr_task_pid_array, 0, sizeof(pid_t) * PID_MAX_DEFAULT);

	ktime_get_real_ts64(&ts);
	rtc_time64_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	dev_info(cxt->owner, "Kernel Top Statistic start (%02d-%02d %02d:%02d:%02d)\n",
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	get_all_cpustat(&cxt->curr_all_cpustat);
	memcpy(&cxt->prev_all_cpustat,
		&cxt->curr_all_cpustat, sizeof(struct kernel_cpustat));

	/* Calculate time in a process;
	 * the sum of user time (utime) and system time (stime)*/
	rcu_read_lock();
	for_each_process(tsk) {
		if (tsk->pid < PID_MAX_DEFAULT) {
			u64 utime, stime;
			thread_group_cputime_adjusted(tsk, &utime, &stime);
			cxt->prev_tasktics_array[tsk->pid] = stime + utime;
		}
	}
	rcu_read_unlock();
}
EXPORT_SYMBOL_GPL(kernel_top_reset);

void kernel_top_destroy(struct kernel_top_context *cxt)
{
	vfree(cxt->prev_tasktics_array);
	vfree(cxt->frame_tasktics_array);
	vfree(cxt->task_ptr_array);
	vfree(cxt->curr_task_pid_array);

	devm_kfree(cxt->owner, cxt);
}
EXPORT_SYMBOL_GPL(kernel_top_destroy);

MODULE_DESCRIPTION("Kernel-Top utils");
MODULE_LICENSE("GPL v2");
