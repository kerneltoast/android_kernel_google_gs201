/* SPDX-License-Identifier: GPL-2.0 */
#ifndef EXYNOS_EHLD__H
#define EXYNOS_EHLD__H

#define NUM_TRACE			(8)
#define NUM_TRACE_SKIP			(4)

#define EHLD_STAT_NORMAL		(0x0)
#define EHLD_STAT_LOCKUP_WARN		(0x1)
#define EHLD_STAT_LOCKUP_SW		(0x2)
#define EHLD_STAT_LOCKUP_HW		(0x3)
#define EHLD_STAT_HANDLED_FLAG		(0x80)

#define EHLD_VAL_INIT			(0xC0)
#define EHLD_VAL_PM_PREPARE		(0xC1)
#define EHLD_VAL_PM			(0xC2)
#define EHLD_VAL_PM_POST		(0xC3)
#define EHLD_VAL_NORMAL			(0xC4)

#if IS_ENABLED(CONFIG_EXYNOS_EHLD)
void exynos_ehld_event_raw_update(unsigned int cpu, bool update_val);
void exynos_ehld_event_raw_dump(unsigned int cpu, bool header);
void exynos_ehld_event_raw_update_allcpu(void);
void exynos_ehld_event_raw_dump_allcpu(void);
void exynos_ehld_do_policy(void);
int exynos_ehld_start(void);
void exynos_ehld_stop(void);
void ehld_tick_sched_timer(void);
void exynos_ehld_prepare_panic(void);

int adv_tracer_ehld_set_enable(int en);
int adv_tracer_ehld_set_interval(u32 interval);
int adv_tracer_ehld_set_warn_count(u32 count);
int adv_tracer_ehld_set_lockup_count(u32 count);
int adv_tracer_ehld_set_init_val(u32 interval, u32 count, u32 cpu_mask);
u32 adv_tracer_ehld_get_interval(void);
int adv_tracer_ehld_get_enable(void);
int adv_tracer_ehld_init(void *data);
int adv_tracer_ehld_remove(void);
int adv_tracer_ehld_set_pmu_cntr_id(int cpu, int en, int cntr_id);
#else
#define exynos_ehld_event_raw_update(a, b)		do { } while (0)
#define exynos_ehld_event_raw_dump(a, b)		do { } while (0)
#define exynos_ehld_event_raw_update_allcpu(void)	do { } while (0)
#define exynos_ehld_event_raw_dump_allcpu(void)		do { } while (0)
#define exynos_ehld_do_policy(void)			do { } while (0)
inline int exynos_ehld_start(void)
{
	return -1;
}
#define exynos_ehld_start(void)				do { } while (0)
#define exynos_ehld_stop(void)				do { } while (0)
#define ehld_tick_sched_timer(a)			do { } while (0)
#define exynos_ehld_prepare_panic(void)			do { } while (0)

inline int adv_tracer_ehld_set_enable(int en)
{
	return -1;
}
inline int adv_tracer_ehld_set_interval(u32 interval)
{
	return -1;
}
inline int adv_tracer_ehld_set_warn_count(u32 count)
{
	return -1;
}
inline int adv_tracer_ehld_set_lockup_count(u32 count)
{
	return -1;
}
inline u32 adv_tracer_ehld_get_interval(void)
{
	return -1;
}
inline int adv_tracer_ehld_get_enable(void)
{
	return -1;
}
inline int adv_tracer_ehld_init(void *data)
{
	return -1;
}
inline int adv_tracer_ehld_remove(void)
{
	return -1;
}
inline int adv_tracer_ehld_set_pmu_cntr_id(int cpu, int en, int cntr_id)
{
	return -1;
}
#endif
#endif
