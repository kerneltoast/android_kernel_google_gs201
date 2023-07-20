// SPDX-License-Identifier: GPL-2.0-only
/* core.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */
#include <linux/sched.h>
#include <kernel/sched/sched.h>

#define LIB_PATH_LENGTH 512
static char sched_lib_name[LIB_PATH_LENGTH];
unsigned int sched_lib_cpu_freq_cached_val;
unsigned int sched_lib_freq_cpumask;
unsigned int sched_lib_affinity_val;

static DEFINE_SPINLOCK(__sched_lib_name_lock);

ssize_t sched_lib_name_store(struct file *filp,
				const char __user *ubuffer, size_t count,
				loff_t *ppos)
{
	size_t null_idx = count > 0 ? count - 1 : 0;
	if (null_idx >= sizeof(sched_lib_name))
		return -EINVAL;

	spin_lock(&__sched_lib_name_lock);

	if (copy_from_user(sched_lib_name, ubuffer, count)) {
		spin_unlock(&__sched_lib_name_lock);
		return -EFAULT;
	}

	sched_lib_name[null_idx] = '\0';
	spin_unlock(&__sched_lib_name_lock);
	return count;
}

sched_lib_name_show(struct seq_file *m, void *v)
{
	spin_lock(&__sched_lib_name_lock);
	seq_printf(m, "%s\n", sched_lib_name);
	spin_unlock(&__sched_lib_name_lock);
	return 0;
}

static bool is_sched_lib_based_task_name(char const *sched_lib_name_list, char const *name)
{
	int length_name;
	int length_sched_lib_name_list;

	length_name = strlen(name);
	if (!length_name)
		return false;

	length_sched_lib_name_list = strnlen(sched_lib_name_list, LIB_PATH_LENGTH);
	if (!length_sched_lib_name_list)
		return false;

	if (!strnstr(sched_lib_name_list, name, length_sched_lib_name_list))
		return false;
	return true;
}


static inline bool is_sched_lib_based_task(struct task_struct *task)
{
	bool found = false;
	char *tmp_lib_name;
	struct task_struct *list_entry_task;

	/* Copy lib name list into place */
	spin_lock(&__sched_lib_name_lock);
	if (strnlen(sched_lib_name, LIB_PATH_LENGTH) == 0) {
		spin_unlock(&__sched_lib_name_lock);
		return false;
	}

	tmp_lib_name = kstrdup(sched_lib_name, GFP_KERNEL);
	spin_unlock(&__sched_lib_name_lock);
	if (!tmp_lib_name) {
		return false;
	}

	/* Check task name equal to any of the sched_lib_name list. */
	found = is_sched_lib_based_task_name(tmp_lib_name, task->comm);
	if (found)
		goto free_up_tmp_lib_name;

	/* Check task name of every thread in group */
	rcu_read_lock();
	for_each_thread(task, list_entry_task) {
		if (is_sched_lib_based_task_name(tmp_lib_name, list_entry_task->comm)) {
			found = true;
			break;
		}
	}
	rcu_read_unlock();
free_up_tmp_lib_name:
	kfree(tmp_lib_name);
	return found;
}

static bool is_sched_lib_based_app(pid_t pid)
{
	bool found;
	struct task_struct *p;

	if (strnlen(sched_lib_name, LIB_PATH_LENGTH) == 0)
		return false;

	rcu_read_lock();
	p = pid ? get_pid_task(find_vpid(pid), PIDTYPE_PID) : get_task_struct(current);
	rcu_read_unlock();
	if (!p)
		return false;

	found = is_sched_lib_based_task(p);

	put_task_struct(p);
	return found;
}

void android_vh_show_max_freq(void *unused, struct cpufreq_policy *policy,
				     unsigned int *max_freq)
{
	bool is_app;
	unsigned int the_bit;
	if ((!sched_lib_freq_cpumask) || (!sched_lib_cpu_freq_cached_val))
		return;
	the_bit = BIT(policy->cpu);
	if (!(the_bit & sched_lib_freq_cpumask))
		return;

	is_app = is_sched_lib_based_app(current->pid);
	if (is_app)
		*max_freq = sched_lib_cpu_freq_cached_val << 1;

	pr_debug("sched_lib show_max_freq returning %u, pid %d, is_app %s, cpu %d, bit %d\n",
		(*max_freq), current->pid, is_app ? "true" : "false", policy->cpu, the_bit);
}

void vh_sched_setaffinity_mod(void *data, struct task_struct *task,
				const struct cpumask *in_mask, int *skip)
{
	bool is_sched;
	if (!sched_lib_affinity_val)
		return;

	is_sched = is_sched_lib_based_task(task);
	if (is_sched)
		*skip = 1;

	pr_debug("sched_lib setaffinity task %5d, cpumask %*pb, skip %s, is_sched %s\n",
		task_pid_nr(task), cpumask_pr_args(in_mask), (*skip)?"True":"False",
		is_sched?"True":"False");
}

