/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/oom.h>
#include <linux/swap.h>

#include "../../include/pixel_mm_hint.h"

#define K(x) ((x) << (PAGE_SHIFT-10))
#define SWAPPINESS_MAX 200

// atomic type for no tearing issue
static atomic_long_t mm_hint_enable = ATOMIC_INIT(0);
static atomic_long_t mm_hint_mode = ATOMIC_INIT(0);
static atomic_long_t min_file_cache_kb = ATOMIC_INIT(0);
static atomic_long_t critical_oom_score = ATOMIC_INIT(OOM_SCORE_ADJ_MAX);
static atomic_long_t critical_swappiness = ATOMIC_INIT(20);

void vh_vmscan_tune_swappiness(void *data, int *swappiness)
{
	enum mm_hint_mode hint = get_mm_hint_mode();
	bool file_cache_enough = is_file_cache_enough();

	if (hint == MM_HINT_NONE)
		return;

	if (file_cache_enough) {
		// speed up kswapd & direct reclaim cases
		*swappiness = 0;
		return;
	}

	if (!current_is_kswapd() && !file_cache_enough &&
		is_critical_process(current)) {
		/*
		 * only allow critical process to reclaim further
		 * when file cache is NOT enough for direct reclaim case.
		 */
		*swappiness = get_critical_swappiness();
	}
}

static int mm_hint_enable_set(const char *val, const struct kernel_param *kp)
{
	bool is_active;

	if (kstrtobool(val, &is_active)) {
		pr_err("%s: mm_hint_enable parse error", __func__);
		return -EINVAL;
	}

	atomic_long_set(&mm_hint_enable, is_active);

	return 0;
}

static int mm_hint_enable_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit_at(buf, 0, "%lu\n", atomic_long_read(&mm_hint_enable));
}

enum mm_hint_mode get_mm_hint_mode(void)
{
	if (atomic_long_read(&mm_hint_enable))
		return atomic_long_read(&mm_hint_mode);
	else
		return MM_HINT_NONE;
}
EXPORT_SYMBOL_GPL(get_mm_hint_mode);

static int mm_hint_mode_set(const char *val, const struct kernel_param *kp)
{
	unsigned long value;

	if (kstrtoul(val, 10, &value)) {
		pr_err("%s: mm_hint_mode parse error", __func__);
		return -EINVAL;
	}

	if (value < MM_HINT_NUM)
		atomic_long_set(&mm_hint_mode, value);
	else
		return -EINVAL;

	return 0;
}

static int mm_hint_mode_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit_at(buf, 0, "%lu\n", atomic_long_read(&mm_hint_mode));
}


bool is_file_cache_enough(void)
{
	unsigned long num_file_pages;

	num_file_pages = global_node_page_state(NR_ACTIVE_FILE) +
		    global_node_page_state(NR_INACTIVE_FILE);

	if (K(num_file_pages) > atomic_long_read(&min_file_cache_kb))
		return true;
	else
		return false;
}
EXPORT_SYMBOL_GPL(is_file_cache_enough);

static int min_file_cache_kb_set(const char *val, const struct kernel_param *kp)
{
	unsigned long value;

	if (kstrtoul(val, 10, &value)) {
		pr_err("%s: min_file_cache_kb parse error", __func__);
		return -EINVAL;
	}

	atomic_long_set(&min_file_cache_kb, value);

	return 0;
}

static int min_file_cache_kb_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit_at(buf, 0, "%lu\n", atomic_long_read(&min_file_cache_kb));
}

bool is_critical_process(struct task_struct *task)
{
	return (task->signal->oom_score_adj <= atomic_long_read(&critical_oom_score));
}
EXPORT_SYMBOL_GPL(is_critical_process);

static int critical_oom_score_set(const char *val, const struct kernel_param *kp)
{
	long oom;

	if (kstrtol(val, 10, &oom)) {
		pr_err("%s: critical_oom_score parse error", __func__);
		return -EINVAL;
	}

	if (oom > OOM_SCORE_ADJ_MAX || oom < OOM_SCORE_ADJ_MIN)
		return -EINVAL;

	atomic_long_set(&critical_oom_score, oom);

	return 0;
}

static int critical_oom_score_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit_at(buf, 0, "%ld\n", atomic_long_read(&critical_oom_score));
}

int get_critical_swappiness(void)
{
	return atomic_long_read(&critical_swappiness);
}
EXPORT_SYMBOL_GPL(get_critical_swappiness);

static int critical_swappiness_set(const char *val, const struct kernel_param *kp)
{
	unsigned long swappiness;

	if (kstrtoul(val, 10, &swappiness)) {
		pr_err("%s: critical_swappiness parse error", __func__);
		return -EINVAL;
	}

	if (swappiness > SWAPPINESS_MAX)
		return -EINVAL;

	atomic_long_set(&critical_swappiness, swappiness);

	return 0;
}

static int critical_swappiness_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit_at(buf, 0, "%ld\n", atomic_long_read(&critical_swappiness));
}

static const struct kernel_param_ops mm_hint_enable_ops = {
	.set = mm_hint_enable_set,
	.get = mm_hint_enable_get,
};

static const struct kernel_param_ops  mm_hint_mode_ops = {
	.set = mm_hint_mode_set,
	.get = mm_hint_mode_get,
};

static const struct kernel_param_ops min_file_cache_kb_ops = {
	.set = min_file_cache_kb_set,
	.get = min_file_cache_kb_get,
};

static const struct kernel_param_ops critical_oom_score_ops = {
	.set = critical_oom_score_set,
	.get = critical_oom_score_get,
};

static const struct kernel_param_ops critical_swappiness_ops = {
	.set = critical_swappiness_set,
	.get = critical_swappiness_get,
};

module_param_cb(mm_hint_enable, &mm_hint_enable_ops, NULL, 0644);
module_param_cb(mm_hint_mode, &mm_hint_mode_ops, NULL, 0644);
module_param_cb(min_file_cache_kb, &min_file_cache_kb_ops, NULL, 0644);
module_param_cb(critical_oom_score, &critical_oom_score_ops, NULL, 0644);
module_param_cb(critical_swappiness, &critical_swappiness_ops, NULL, 0644);
