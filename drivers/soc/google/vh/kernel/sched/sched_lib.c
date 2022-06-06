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
unsigned int sched_lib_freq_val;
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


static bool is_sched_lib_based_task(struct task_struct *task)
{
	const char *name = NULL;
	char *libname, *lib_list;
	struct vm_area_struct *vma;
	char path_buf[LIB_PATH_LENGTH];
	char *tmp_lib_name;
	bool found = false;
	struct mm_struct *mm;

	spin_lock(&__sched_lib_name_lock);
	if (strnlen(sched_lib_name, LIB_PATH_LENGTH) == 0) {
		spin_unlock(&__sched_lib_name_lock);
		return false;
	}

	tmp_lib_name = kmalloc(LIB_PATH_LENGTH, GFP_KERNEL);
	if (!tmp_lib_name) {
		spin_unlock(&__sched_lib_name_lock);
		return false;
	}

	/* Check if the task name equals any of the sched_lib_name list. */
	strlcpy(tmp_lib_name, sched_lib_name, LIB_PATH_LENGTH);
	spin_unlock(&__sched_lib_name_lock);

	lib_list = tmp_lib_name;
	while ((libname = strsep(&lib_list, ","))) {
		libname = skip_spaces(libname);
		name = task->comm;
		if (strnstr(name, libname,
			strnlen(name, LIB_PATH_LENGTH))) {
			found = true;
			goto put_task_struct;
		}
	}

	mm = get_task_mm(task);
	if (!mm) {
		goto put_task_struct;
	}

	down_read(&mm->mmap_lock);
	for (vma = mm->mmap; vma ; vma = vma->vm_next) {
		if (vma->vm_file && vma->vm_flags & VM_EXEC) {
			name = d_path(&vma->vm_file->f_path,
					path_buf, LIB_PATH_LENGTH);
			if (IS_ERR(name)) {
				goto release_sem;
			}

			//strlcpy(tmp_lib_name, sched_lib_name, LIB_PATH_LENGTH);
			lib_list = tmp_lib_name;
			while ((libname = strsep(&lib_list, ","))) {
				libname = skip_spaces(libname);
				if (strnstr(name, libname,
					strnlen(name, LIB_PATH_LENGTH))) {
					found = true;
					goto release_sem;
				}
			}
		}
	}

release_sem:
	up_read(&mm->mmap_lock);
	mmput(mm);
put_task_struct:
	kfree(tmp_lib_name);
	return found;
}

static bool is_sched_lib_based_app(pid_t pid)
{
	bool found = false;
	struct task_struct *p;

	if (strnlen(sched_lib_name, LIB_PATH_LENGTH) == 0)
		return false;

	rcu_read_lock();
	p = pid ? get_pid_task(find_vpid(pid), PIDTYPE_PID) : get_task_struct(current);
	rcu_read_unlock();
	if (!p) {
		return false;
	}

	found = is_sched_lib_based_task(p);

	put_task_struct(p);
	return found;
}

void android_vh_show_max_freq(void *unused, struct cpufreq_policy *policy,
				     unsigned int *max_freq)
{
	bool is_app = false;
	int the_bit = 0;
	if (!sched_lib_cpu_freq_cached_val)
		return;

	if (!(BIT(policy->cpu) & sched_lib_freq_val))
		return;

	is_app = is_sched_lib_based_app(current->pid);
	if (is_app) {
		*max_freq = sched_lib_cpu_freq_cached_val << 1;
	}

	the_bit = BIT(policy->cpu);
	pr_debug("sched_lib show_max_freq returning %u, pid %d, is_app %s, cpu %d, bit %d\n",
		(*max_freq), current->pid, is_app ? "true" : "false", policy->cpu, the_bit);
}

void vh_sched_setaffinity_mod(void *data, struct task_struct *task,
				const struct cpumask *in_mask, int *skip)
{
	bool is_sched = false;
	if (!sched_lib_affinity_val) {
		return;
	}

	is_sched = is_sched_lib_based_task(task);
	if (is_sched) {
		*skip = 1;
	}

	pr_debug("sched_lib setaffinity task %5d, cpumask %*pb, skip %s, is_sched %s\n",
		task_pid_nr(task), cpumask_pr_args(in_mask), (*skip)?"True":"False",
		is_sched?"True":"False");
}

