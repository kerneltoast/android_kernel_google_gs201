/* SPDX-License-Identifier: GPL-2.0 */

#ifndef PIXEL_MM_HINT_H
#define PIXEL_MM_HINT_H

enum mm_hint_mode {
	MM_HINT_NONE,
	MM_HINT_APP_LAUNCH,
	MM_HINT_CAMERA_LAUNCH,
	MM_HINT_NUM
};

enum mm_kill_reason {
	MM_PA_KILL,
};

#if IS_ENABLED(CONFIG_VH_MM)
void vh_vmscan_tune_swappiness(void *data, int *swappiness);
void vh_update_lmkd_watermark(void *data, bool *skip);
enum mm_hint_mode get_mm_hint_mode(void);
bool is_file_cache_enough(void);
bool is_critical_process(struct task_struct *task);
int get_critical_swappiness(void);
void disable_update_lmkd_watermark_notify(void);
void enable_update_lmkd_watermark_notify(void);
int try_to_trigger_lmkd_kill(int reason, short min_oom_score_adj);
#else
static inline enum mm_hint_mode get_mm_hint_mode(void)
{
	return 0;
}

static inline bool is_file_cache_enough(void)
{
	return 0;
}

static inline bool is_critical_process(struct task_struct *task)
{
	return 0;
}

static inline int get_critical_swappiness(void)
{
	return 100;
}

static inline void disable_update_lmkd_watermark_notify(void)
{
}

static inline void enable_update_lmkd_watermark_notify(void)
{
}

int try_to_trigger_lmkd_kill(int reason, short min_oom_score_adj)
{
	return -EBUSY;
}
#endif

#endif	/* PIXEL_MM_HINT_H */