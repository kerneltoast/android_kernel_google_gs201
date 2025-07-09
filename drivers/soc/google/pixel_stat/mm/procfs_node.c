// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 Google LLC
 */
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/mm.h>

struct proc_dir_entry *vendor_mm;
EXPORT_SYMBOL_GPL(vendor_mm);

struct task_group {
	int oom_adj;
	int nr_task;
	unsigned long rss[NR_MM_COUNTERS];
	unsigned long pgtable_bytes;
};

/* Feel free to suggest better division for type */
#define MAX_OOM_ADJ  1000
static int group_oom_adj[] = {
	951, 901, 851, 801, 751, 701, 651, 601, 551, 501, 451, 401, 351, 301,
	251, 201,
	200,
	151, 101, 51, 1,
	0,
	-1000,
};

#define NUM_OOM_ADJ_GROUPS ARRAY_SIZE(group_oom_adj)

static void init_task_groups(struct task_group *group, int array_len)
{
	int i;

	memset(group, 0, sizeof(struct task_group) * array_len);
	for (i = 0; i < array_len; i++)
		group[i].oom_adj = group_oom_adj[i];
}

/*
 * The process p may have detached its own ->mm while exiting or through
 * kthread_use_mm(), but one or more of its subthreads may still have a valid
 * pointer.  Return p, or any of its subthreads with a valid ->mm, with
 * task_lock() held.
 */
struct task_struct *vendor_find_lock_task_mm(struct task_struct *p)
{
	struct task_struct *t;

	rcu_read_lock();

	for_each_thread(p, t) {
		task_lock(t);
		if (likely(t->mm))
			goto found;
		task_unlock(t);
	}
	t = NULL;
found:
	rcu_read_unlock();

	return t;
}

static void gather_memory_usage(struct task_struct *p,
				struct task_group *groups, int array_len)
{
	int i;
	struct mm_struct *mm;
	struct task_group *group = NULL;
	struct task_struct *task = vendor_find_lock_task_mm(p);

	if (!task) {
		/*
		 * All of p's threads have already detached their mm's. There's
		 * no need to report them; they can't be oom killed anyway.
		 */
		return;
	}

	if (task->signal->oom_score_adj > MAX_OOM_ADJ ||
	    task->signal->oom_score_adj < groups[array_len - 1].oom_adj) {
		printk(KERN_ERR "/proc/vendor_mm/memory_usage_by_oom_score: "
		       "omm_score_adj %d out of the range.\n",
		       task->signal->oom_score_adj);
		goto cleanup;
	}

	for (i = 0; i < array_len; i++) {
		if (groups[i].oom_adj <= task->signal->oom_score_adj) {
			group = &groups[i];
			break;
		}
	}

	mm = task->mm;
	for (i = 0; i < NR_MM_COUNTERS; i++)
		group->rss[i] += get_mm_counter(mm, i);
	group->pgtable_bytes += mm_pgtables_bytes(mm);
	group->nr_task++;

cleanup:
	task_unlock(task);
}

/* show value with "kB" without space or line feed */
static void show_pure_val_kb(struct seq_file *m, const char *s,
			     unsigned long num, unsigned int width)
{
	seq_put_decimal_ull_width(m, s, num << (PAGE_SHIFT - 10), width);
}


/* same as seq_put_decimal_ll() but returns the length */
static int seq_put_decimal_ll_with_length(struct seq_file *m, long long num) {
	int n = m->count;

	seq_put_decimal_ll(m, NULL, num);
	return m->count - n;
}

/* print range: ex "[200,250]   ", left-padding spaces to the width */
static void seq_put_range(struct seq_file *m, int start, int end, int width) {
	int n = 3;  /* for "[,]" */

	seq_putc(m, '[');
	n += seq_put_decimal_ll_with_length(m, (long long) start);
	seq_putc(m, ',');
	n += seq_put_decimal_ll_with_length(m, (long long) end);
	seq_putc(m, ']');

	while (n++ < width)
		seq_putc(m, ' ');
}

static int memory_usage_by_oom_score_proc_show(struct seq_file *m, void *v)
{
	struct task_struct *p;
	struct task_group groups[NUM_OOM_ADJ_GROUPS];
	int i;
	int prev_group_base = MAX_OOM_ADJ + 1;

	init_task_groups(groups, NUM_OOM_ADJ_GROUPS);

	rcu_read_lock();
	for_each_process(p)
		gather_memory_usage(p, groups, NUM_OOM_ADJ_GROUPS);
	rcu_read_unlock();

	/* header */
	seq_puts(m, "# oom_group  <nr_task > <file_rss_kb> <anon_rss_kb> "
		 "<pgtable_kb> <swap_ents_kb> <shmem_rss_kb>\n");

	for (i = 0; i < NUM_OOM_ADJ_GROUPS; i++) {
		/* group: format aligns to the header */
		seq_put_range(m, groups[i].oom_adj, prev_group_base - 1, 12);
		prev_group_base = groups[i].oom_adj;

		/* values: format aligns to the header */
		seq_put_decimal_ull_width(m, " ", groups[i].nr_task, 10);
		show_pure_val_kb(m, " ", groups[i].rss[MM_FILEPAGES], 13);
		show_pure_val_kb(m, " ", groups[i].rss[MM_ANONPAGES], 13);
		seq_put_decimal_ull_width(m, " ",
					  groups[i].pgtable_bytes >> 10, 12);
		show_pure_val_kb(m, " ", groups[i].rss[MM_SWAPENTS], 14);
		show_pure_val_kb(m, " ", groups[i].rss[MM_SHMEMPAGES], 14);
		seq_putc(m, '\n');
	}

	return 0;
}

int create_mm_procfs_node(void)
{
	vendor_mm = proc_mkdir("vendor_mm", NULL);
	if (!vendor_mm)
		return -ENOMEM;

	if (!proc_create_single("memory_usage_by_oom_score", 0, vendor_mm,
				memory_usage_by_oom_score_proc_show))
		pr_warn("unable to create memory_usage_by_oom_score");

	return 0;
}
