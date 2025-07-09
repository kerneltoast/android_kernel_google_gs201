/* SPDX-License-Identifier: GPL-2.0 */

#ifndef PIXEL_MM_HINT_H
#define PIXEL_MM_HINT_H

enum mm_hint_mode {
	MM_HINT_NONE,
	MM_HINT_APP_LAUNCH,
	MM_HINT_CAMERA_LAUNCH,
	MM_HINT_NUM
};

#if IS_ENABLED(CONFIG_VH_MM)
void vh_vmscan_tune_swappiness(void *data, int *swappiness);
enum mm_hint_mode get_mm_hint_mode(void);
bool is_file_cache_enough(void);
bool is_critical_process(struct task_struct *task);
int get_critical_swappiness(void);
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
#endif

#endif	/* PIXEL_MM_HINT_H */