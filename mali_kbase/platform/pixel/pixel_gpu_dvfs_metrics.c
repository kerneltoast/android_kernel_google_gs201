// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Linux includes */
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <linux/clk.h>
#include <trace/events/power.h>

/* SOC includes */
#if IS_ENABLED(CONFIG_CAL_IF)
#include <soc/google/cal-if.h>
#endif

/* Mali core includes */
#include <mali_kbase.h>
#include <backend/gpu/mali_kbase_pm_internal.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_dvfs.h"
#include "mali_power_gpu_frequency_trace.h"

static void *enumerate_gpu_clk(struct kbase_device *kbdev, unsigned int index)
{
	struct pixel_context *pc = kbdev->platform_context;

	if (index < GPU_DVFS_CLK_COUNT)
		return &(pc->dvfs.clks[index]);
	else
		return NULL;
}

static unsigned long get_gpu_clk_rate(struct kbase_device *kbdev, void *gpu_clk_handle)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_clk *clk = gpu_clk_handle;

	if (clk->index < GPU_DVFS_CLK_COUNT)
		return pc->dvfs.table[pc->dvfs.level_target].clk[clk->index] * 1000;

	WARN_ONCE(1, "Clock rate requested for invalid clock index: %u\n", clk->index);
	return 0;
}

static int gpu_clk_notifier_register(struct kbase_device *kbdev, void *gpu_clk_handle,
	struct notifier_block *nb)
{
	struct gpu_dvfs_clk *clk = gpu_clk_handle;

	return blocking_notifier_chain_register(&clk->notifier, nb);
}

static void gpu_clk_notifier_unregister(struct kbase_device *kbdev, void *gpu_clk_handle,
	struct notifier_block *nb)
{
	struct gpu_dvfs_clk *clk = gpu_clk_handle;

	blocking_notifier_chain_unregister(&clk->notifier, nb);
}

struct kbase_clk_rate_trace_op_conf pixel_clk_rate_trace_ops = {
	.get_gpu_clk_rate = get_gpu_clk_rate,
	.enumerate_gpu_clk = enumerate_gpu_clk,
	.gpu_clk_notifier_register = gpu_clk_notifier_register,
	.gpu_clk_notifier_unregister = gpu_clk_notifier_unregister,
};

/**
 * gpu_dvfs_metrics_trace_clock() - Emits trace events corresponding to a change in GPU clocks.
 *
 * @kbdev:       The &struct kbase_device for the GPU.
 * @old_level:   The level the GPU has just been moved from.
 * @new_level:   The level the GPU has just been moved to.
 * @power_state: The current GPU power state.
 */
static void gpu_dvfs_metrics_trace_clock(struct kbase_device *kbdev, int old_level, int new_level,
	bool power_state)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct kbase_gpu_clk_notifier_data nd;
	int c;
	int clks[GPU_DVFS_CLK_COUNT];

	for (c = 0; c < GPU_DVFS_CLK_COUNT; c++) {
		clks[c] = 0;

		if (power_state) {
			clks[c] = pc->dvfs.table[new_level].clk[c];

			nd.gpu_clk_handle = &(pc->dvfs.clks[c]);
			nd.old_rate = pc->dvfs.table[old_level].clk[c] * 1000;
			nd.new_rate = pc->dvfs.table[new_level].clk[c] * 1000;
			blocking_notifier_call_chain(&pc->dvfs.clks[c].notifier,
				POST_RATE_CHANGE, &nd);
		}

	}
}

/**
 * gpu_dvfs_metrics_uid_level_change() - Event for updating per-UID states when GPU clocks change
 *
 * @kbdev:       The &struct kbase_device for the GPU.
 * @event_time:  The time of the clock change event in nanoseconds.
 *
 * Called when the operating point is changing so that the per-UID time in state
 * data for active work can be updated. Note that this function need only be
 * called when the operating point is changing _and_ the GPU is powered on.
 * This is because no work will be active when the GPU is powered down.
 *
 * Context: Called in process context. Requires the dvfs.lock & dvfs.metrics.lock to be held.
 */
static void gpu_dvfs_metrics_uid_level_change(struct kbase_device *kbdev, u64 event_time)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_metrics_uid_stats *stats;
	int i;
	int const nr_slots = ARRAY_SIZE(pc->dvfs.metrics.work_uid_stats);

	lockdep_assert_held(&pc->dvfs.lock);
	lockdep_assert_held(&pc->dvfs.metrics.lock);

	for (i = 0; i < nr_slots; i++) {
		stats = pc->dvfs.metrics.work_uid_stats[i];
		if (stats && stats->period_start != event_time) {
			WARN_ON_ONCE(stats->period_start == 0);
			stats->tis_stats[pc->dvfs.level].time_total +=
				(event_time - stats->period_start);
			stats->period_start = event_time;
		}
	}
}

void gpu_dvfs_metrics_update(struct kbase_device *kbdev, int old_level, int new_level,
	bool power_state)
{
	struct pixel_context *pc = kbdev->platform_context;
	const u64 prev = pc->dvfs.metrics.last_time;
	u64 curr = ktime_get_ns();
	unsigned long flags;

	lockdep_assert_held(&pc->dvfs.lock);

	/* Grab the lock before modifying the entry upon a dvfs event. */
	spin_lock_irqsave(&pc->dvfs.metrics.lock, flags);

	if (pc->dvfs.metrics.last_power_state) {
		if (power_state) {
			/* Power state was ON and is not changing */
			if (old_level != new_level) {
				pc->dvfs.table[new_level].metrics.entry_count++;
				pc->dvfs.table[new_level].metrics.time_last_entry = curr;
				gpu_dvfs_metrics_transtab_entry(pc, old_level, new_level)++;
				gpu_dvfs_metrics_uid_level_change(kbdev, curr);
			}
		} else {
			/* Power status was ON and is turning OFF */
			pc->pm.power_off_metrics.entry_count++;
			pc->pm.power_off_metrics.time_last_entry = curr;
		}

		pc->dvfs.table[old_level].metrics.time_total += (curr - prev);
		pc->pm.power_on_metrics.time_total += (curr - prev);

	} else {
		if (power_state) {
			/* Power state was OFF and is turning ON */
			pc->pm.power_on_metrics.entry_count++;
			pc->pm.power_on_metrics.time_last_entry = curr;

			if (pc->dvfs.metrics.last_level != new_level) {
				/* Level was changed while the GPU was powered off, and that change
				 * is being reflected now.
				 */
				pc->dvfs.table[new_level].metrics.entry_count++;
				pc->dvfs.table[new_level].metrics.time_last_entry = curr;
				gpu_dvfs_metrics_transtab_entry(pc, old_level, new_level)++;
			}
		}

		pc->pm.power_off_metrics.time_total += (curr - prev);
	}

	pc->dvfs.metrics.last_power_state = power_state;
	pc->dvfs.metrics.last_time = curr;
	pc->dvfs.metrics.last_level = new_level;
	spin_unlock_irqrestore(&pc->dvfs.metrics.lock, flags);

	gpu_dvfs_metrics_trace_clock(kbdev, old_level, new_level, power_state);
}

/**
 * gpu_dvfs_create_uid_stats() - Allocates and initializes a per-UID stats block
 *
 * @pc:  The &struct pixel_context that is requesting the stats block.
 * @uid: The &kuid_t corresponding to the application that will be tracked.
 *
 * Return: Returns a pointer to the per-UID stats block, or an ERRPTR on failure.
 */
static struct gpu_dvfs_metrics_uid_stats *gpu_dvfs_create_uid_stats(struct pixel_context *pc,
	kuid_t uid)
{
	struct gpu_dvfs_metrics_uid_stats *ret;

	lockdep_assert_held(&pc->kbdev->kctx_list_lock);

	ret = kzalloc(sizeof(struct gpu_dvfs_metrics_uid_stats), GFP_KERNEL);
	if (ret == NULL)
		return ERR_PTR(-ENOMEM);

	ret->tis_stats = kzalloc(sizeof(struct gpu_dvfs_opp_metrics) * pc->dvfs.table_size,
		GFP_KERNEL);
	if (ret->tis_stats == NULL) {
		kfree(ret);
		return ERR_PTR(-ENOMEM);
	}

	ret->gpu_cycles_last = 0;
	ret->gpu_active_ns_last = 0;
	ret->timestamp_ns_last = ktime_get_ns();

	ret->uid = uid;

	return ret;
}

/**
 * gpu_dvfs_destroy_uid_stats() - Destroys a previously initializes per-UID stats block
 *
 * @stats:  The &struct gpu_dvfs_metrics_uid_stats that is to be destroyed
 *
 */
static void gpu_dvfs_destroy_uid_stats(struct gpu_dvfs_metrics_uid_stats *stats)
{
	kfree(stats->tis_stats);
	kfree(stats);
}

/* Kernel context callback management */

/**
 * gpu_dvfs_kctx_init() - Called when a kernel context is created
 *
 * @kctx: The &struct kbase_context that is being initialized
 *
 * This function is called when the GPU driver is initializing a new kernel context. This event is
 * used to set up data structures that will be used to track this context's usage of the GPU to
 * enable tracking of GPU usage on a per-UID basis.
 *
 * If data for the calling UID has already been created during the life of the GPU kernel driver,
 * the previously allocated stats structure is used allowing for persistent metrics for that UID.
 * If the UID has not been seen before, a new stats block is created and inserted into the list of
 * per-UID stats such that the list is sorted by UID.
 *
 * Return: Returns 0 on success, or an error code on failure.
 */
int gpu_dvfs_kctx_init(struct kbase_context *kctx)
{
	struct kbase_device *kbdev = kctx->kbdev;
	struct pixel_context *pc = kbdev->platform_context;
	struct pixel_platform_data *pd = kctx->platform_data;
	struct task_struct *task;
	struct pid *pid;
	kuid_t uid;
	u8 uid_hash;
	unsigned long flags;
	struct gpu_dvfs_metrics_uid_stats *entry;
	struct gpu_dvfs_metrics_uid_stats *stats = NULL;
	int ret = 0;

	/* Get UID from task_struct */
	pid = find_get_pid(kctx->kprcs->tgid);
	task = get_pid_task(pid, PIDTYPE_TGID);
	uid = task->cred->uid;
	put_task_struct(task);
	put_pid(pid);
	uid_hash = gpu_dvfs_hash_uid_stats(__kuid_val(uid));

	mutex_lock(&kbdev->kctx_list_lock);

	/*
	 * Search through the UIDs we have encountered previously, and either return an already
	 * created stats block, or create one and insert it to the table before returning it.
	 */
	stats = NULL;

	hash_for_each_possible(pc->dvfs.metrics.uid_stats_table, entry, uid_list_node, uid_hash) {
		if (uid_eq(entry->uid, uid)) {
			/* Already created */
			stats = entry;
			break;
		}
	}

	/* Create and add to the table */
	if (stats == NULL) {
		stats = gpu_dvfs_create_uid_stats(pc, uid);
		if (IS_ERR(stats)) {
			ret = PTR_ERR(stats);
			goto done;
		}

		/* Grab the lock before adding an entry: we can't guarantee tha the entry won't be
		 * read from another thread before its initialization in this thread due to
		 * possible compiler/CPU out of order optimizations.
		 */
		spin_lock_irqsave(&pc->dvfs.metrics.lock, flags);
		hash_add(pc->dvfs.metrics.uid_stats_table, &stats->uid_list_node, uid_hash);
		spin_unlock_irqrestore(&pc->dvfs.metrics.lock, flags);
	}

	stats->active_kctx_count++;

	/* Store a direct link in the kctx */
	pd->stats = stats;

done:
	mutex_unlock(&kbdev->kctx_list_lock);
	return ret;
}

/**
 * gpu_dvfs_kctx_term() - Called when a kernel context is terminated
 *
 * @kctx: The &struct kbase_context that is being terminated
 *
 * Since per-UID stats are retained for as long as the GPU kernel driver is loaded, we don't delete
 * the stats block, we only update that there is one fewer kernel context attached to it.
 */
void gpu_dvfs_kctx_term(struct kbase_context *kctx)
{
	struct kbase_device *kbdev = kctx->kbdev;
	struct pixel_platform_data *pd = kctx->platform_data;
	struct gpu_dvfs_metrics_uid_stats *stats = pd->stats;
	unsigned long flags;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	stats->active_kctx_count--;
	WARN_ON(stats->active_kctx_count < 0);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
}

int gpu_dvfs_metrics_init(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	int c;

	mutex_lock(&pc->dvfs.lock);
	spin_lock_init(&pc->dvfs.metrics.lock);

	pc->dvfs.metrics.last_time = ktime_get_ns();
	pc->dvfs.metrics.last_power_state = gpu_pm_get_power_state(kbdev);

	pc->dvfs.metrics.transtab = kzalloc(sizeof(int) * gpu_dvfs_metrics_transtab_size(pc),
		GFP_KERNEL);
	if (pc->dvfs.metrics.transtab == NULL)
		return -ENOMEM;

	pc->dvfs.table[pc->dvfs.level].metrics.entry_count++;
	pc->dvfs.table[pc->dvfs.level].metrics.time_last_entry =
		pc->dvfs.metrics.last_time;

	mutex_unlock(&pc->dvfs.lock);

	for (c = 0; c < GPU_DVFS_CLK_COUNT; c++)
		BLOCKING_INIT_NOTIFIER_HEAD(&pc->dvfs.clks[c].notifier);

	/* Initialize per-UID metrics */
	hash_init(pc->dvfs.metrics.uid_stats_table);

	memset(pc->dvfs.metrics.work_uid_stats, 0, sizeof(pc->dvfs.metrics.work_uid_stats));

	return 0;
}

void gpu_dvfs_metrics_term(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_metrics_uid_stats *entry;
	struct hlist_node *tmp;
	unsigned bkt;

	kfree(pc->dvfs.metrics.transtab);

	hash_for_each_safe(pc->dvfs.metrics.uid_stats_table, bkt, tmp, entry, uid_list_node) {
		hash_del(&entry->uid_list_node);
		gpu_dvfs_destroy_uid_stats(entry);
	}

}
