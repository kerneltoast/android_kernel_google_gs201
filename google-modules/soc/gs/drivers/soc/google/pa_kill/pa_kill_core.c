// SPDX-License-Identifier: GPL-2.0

#define pr_fmt(fmt) "pa_kill: " fmt

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>
#include <trace/hooks/systrace.h>

#include "pa_kill_sysfs.h"
#include "../vh/include/pixel_mm_hint.h"

#define K(x) ((x) << (PAGE_SHIFT-10))
#define MAX_DEMAND_PAGES (totalram_pages() / 4)
#define SWAP_COMP_RATIO 3

unsigned long target_pgfree;
unsigned int killable_min_oom_adj = 900;
atomic_long_t pa_kill_count = ATOMIC_LONG_INIT(0);
atomic_long_t pa_nr_done = ATOMIC_LONG_INIT(0);
/*
 * the pa_kill will stop after 1sec by default
 */
static unsigned long pa_kill_timeout = HZ;
static unsigned long initiated_jiffes;

static LIST_HEAD(pa_task_list);
static DECLARE_WAIT_QUEUE_HEAD(pa_kill_wait);
static DEFINE_MUTEX(pa_kill_lock);

unsigned long extra_free_kb = 100UL << 10;
unsigned int poll_interval_ms = 20;
int origin_watermark_scale_factor = 0;
int origin_vm_swappiness = 0;
bool movable_allowable = 0;

unsigned int nr_kill_thread;
#define DEFAULT_NR_KILL_THREAD 1

cpumask_t pa_task_cpu_affinity;

struct pa_task {
	struct task_struct *task;
	struct list_head list;
};

static unsigned long get_nr_freed(void)
{
	unsigned long events[NR_VM_EVENT_ITEMS];

	all_vm_events(events);
	return events[PGFREE];
}

/* protected by pa_kill_lock */
static void change_vm_knobs(void)
{
	static int boost_watermark_scale_factor = 1000;
	static int boost_vm_swappiness = 10;

	/*
	 * Temporarily disable LMKD watermark updates to prevent LMKD from reading
	 * a boosted watermark level before we boost the watermark level, which could
	 * result in aggressive killing.
	 * Paired with enable_update_lmkd_watermark_notify(), see restore_vm_knobs().
	 */
	disable_update_lmkd_watermark_notify();
	get_reclaim_params(&origin_watermark_scale_factor, &origin_vm_swappiness);
	set_reclaim_params(boost_watermark_scale_factor, boost_vm_swappiness);
}

/* protected by pa_kill_lock */
static void restore_vm_knobs(void)
{
	int wmf, swappines;

	/*
	 * Re-enable LMKD watermark notifications before restoring original VM parameters.
	 * This ensures LMKD can get the notifications and read the correct, original
	 * watermark values once they are restored by set_reclaim_params().
	 * Paired with disable_update_lmkd_watermark_notify() in change_vm_knobs().
	 */
	enable_update_lmkd_watermark_notify();
	get_reclaim_params(&wmf, &swappines);
	if (wmf != origin_watermark_scale_factor || swappines != origin_vm_swappiness) {
		set_reclaim_params(origin_watermark_scale_factor, origin_vm_swappiness);
		origin_watermark_scale_factor = 0;
		origin_vm_swappiness = 0;
	}
}

static struct zone *pa_next_zone(struct zone *zone)
{
	pg_data_t *pgdat = zone->zone_pgdat;

	if (zone < pgdat->node_zones + MAX_NR_ZONES - 1)
		zone++;
	else
		zone = NULL;
	return zone;
}

/*
 * Return the number of available pages after the kswapd high watermark.
 * This proactively wakes kswapd, anticipating the upcoming burst memory request.
 */
static unsigned long available_pages(void)
{
	long available_pages;

	struct pglist_data *pgdat;
	struct zone *zone;
	unsigned long unusable_free = 0;

	pgdat = NODE_DATA(first_online_node);
	for (zone = pgdat->node_zones; zone; zone = pa_next_zone(zone))
		unusable_free += zone->_watermark[WMARK_HIGH];

	available_pages = global_zone_page_state(NR_FREE_PAGES) - unusable_free;
	if (!movable_allowable)
		available_pages -= global_zone_page_state(NR_FREE_CMA_PAGES);

	if (available_pages < 0)
		available_pages = 0;

	return available_pages;
}

void reclaim_memory(unsigned long nr_demand_pages)
{
	mutex_lock(&pa_kill_lock);
	if (target_pgfree == 0) {
		long nr_available_pages = available_pages();

		/* system has enough free memory so no need to work */
		if (nr_available_pages >= nr_demand_pages) {
			ATRACE_BEGIN("enough memory");
			ATRACE_END();
			goto out;
		};

		/* Too much request */
		if (nr_demand_pages > MAX_DEMAND_PAGES) {
			pr_info("too much request %lu pages\n", nr_demand_pages);
			goto out;
		}
		/*
		 * Kill threads will run until PGFREE is greater than target_pgfree
		 */
		nr_demand_pages -= nr_available_pages;
		target_pgfree = get_nr_freed() + nr_demand_pages;

		/*
		 * Only change the knob at detecting pa_kill completed the job
		 */
		change_vm_knobs();
	} else {
		if (target_pgfree + nr_demand_pages > MAX_DEMAND_PAGES) {
			pr_info("too much accumulated request %lu pages current target_pgfree %lu pages\n",
				nr_demand_pages, target_pgfree);
			goto out;
		}
		/*
		 * If the kill is already triggered, just add up the extra memory to
		 * keep kill threads running.
		 */
		target_pgfree += nr_demand_pages;
	}

	/* set or extend timer */
	initiated_jiffes = jiffies;
	wake_up_all(&pa_kill_wait);
	atomic_long_inc(&pa_nr_done);
out:
	mutex_unlock(&pa_kill_lock);
}

static bool expired_pa_kill(void)
{
	return time_after(jiffies, initiated_jiffes + pa_kill_timeout);
}

static int pa_kill_thread(void *data)
{
	struct sched_attr attr = {
		.sched_policy = SCHED_NORMAL,
		.sched_nice = -10,
	};

	WARN_ON_ONCE(sched_setattr_nocheck(current, &attr) != 0);

	while (!kthread_should_stop()) {
		wait_event_idle(pa_kill_wait, target_pgfree || kthread_should_stop());

		if (kthread_should_stop())
			break;

		/*
		 * Wait poll_interval_ms right after woken up to see how kswapd works
		 * to avoid unnecessary kill and then on every attempt to avoid serial
		 * killing.
		 */
		schedule_timeout_idle(msecs_to_jiffies(poll_interval_ms));
		mutex_lock(&pa_kill_lock);
		if (get_nr_freed() >= target_pgfree || expired_pa_kill()) {
			/* Meet the target so ready to sleep */
			restore_vm_knobs();
			target_pgfree = 0;
			mutex_unlock(&pa_kill_lock);
			continue;
		}
		mutex_unlock(&pa_kill_lock);

		/*
		 * Give chance to kswapd to keep reclaming without kill since
		 * we have extrace_free_kb buffers until system goes direct
		 * reclaim.
		 */
		if (available_pages() > (extra_free_kb >> (PAGE_SHIFT - 10)))
			continue;

		try_to_trigger_lmkd_kill(MM_PA_KILL, (short)killable_min_oom_adj);
	}

	mutex_lock(&pa_kill_lock);
	restore_vm_knobs();
	target_pgfree = 0;
	mutex_unlock(&pa_kill_lock);

	return 0;
}

void pa_set_cpu_affinity(void)
{
	struct pa_task *pa_task;

	list_for_each_entry(pa_task, &pa_task_list, list)
		set_cpus_allowed_ptr(pa_task->task, &pa_task_cpu_affinity);
}

/* protected by sysfs_lock */
void destroy_kill_threads(void)
{
	struct pa_task *pa_task, *next;

	list_for_each_entry_safe(pa_task, next, &pa_task_list, list) {
		list_del(&pa_task->list);
		kthread_stop(pa_task->task);
		kfree(pa_task);
		nr_kill_thread--;
	}
}

/* protected by sysfs_lock */
int create_kill_threads(unsigned int nr_thread)
{
	int i;
	int err = 0;
	struct pa_task *pa_task;

	for (i = 0; i < nr_thread; i++) {
		struct task_struct *task = kthread_run(pa_kill_thread, NULL, "pa_kill");

		if (IS_ERR(task)) {
			pr_err("couldn't craete proactive kill task\n");
			err = PTR_ERR(task);
			goto cleanup;
		}

		pa_task = kmalloc(sizeof(*pa_task), GFP_KERNEL);
		if (!pa_task) {
			err = -ENOMEM;
			goto cleanup;
		}

		pa_task->task = task;
		list_add(&pa_task->list, &pa_task_list);
		nr_kill_thread++;
	}

	pa_set_cpu_affinity();
	return err;

cleanup:
	destroy_kill_threads();
	return err;
}

int __init pa_kill_init(void)
{
	int err;

	/* enable threads on every core by default */
	cpumask_setall(&pa_task_cpu_affinity);
	/* no need a sysfs_lock since sysfs isn't populated yet */
	err = create_kill_threads(DEFAULT_NR_KILL_THREAD);
	if (!err) {
		pa_kill_sysfs_init();
	}

	return 0;

}
module_init(pa_kill_init);

MODULE_LICENSE("GPL");
