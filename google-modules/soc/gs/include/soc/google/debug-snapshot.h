/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Exynos-SnapShot debugging framework for Exynos SoC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DEBUG_SNAPSHOT_H
#define DEBUG_SNAPSHOT_H

#include <dt-bindings/soc/google/debug-snapshot-def.h>
#include <asm/cputype.h>
#include <asm/barrier.h>
#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
#include <linux/sched/clock.h>

#define DSS_FREQ_MAX_SIZE		SZ_32
#define DSS_FREQ_MAX_NAME_SIZE		SZ_8

struct clk;
struct clk_hw;
struct exynos_tmu_data;
struct dbg_snapshot_helper_ops;

extern int dbg_snapshot_get_enable(void);
extern void dbg_snapshot_set_item_enable(const char *name, int en);
extern int dbg_snapshot_get_item_enable(const char *name);
extern unsigned int dbg_snapshot_get_item_size(const char *name);
extern unsigned int dbg_snapshot_get_item_paddr(const char *name);
extern unsigned long dbg_snapshot_get_item_vaddr(const char *name);
extern void dbg_snapshot_output(void);
extern void dbg_snapshot_log_output(void);
extern void dbg_snapshot_scratch_clear(void);
extern bool dbg_snapshot_is_scratch(void);
extern unsigned int dbg_snapshot_get_hardlockup_magic(int cpu);
extern unsigned long dbg_snapshot_get_last_pc(unsigned int cpu);
extern unsigned long dbg_snapshot_get_last_pc_paddr(void);
extern unsigned int dbg_snapshot_get_slcdump_base(void);
extern unsigned int dbg_snapshot_get_pre_slcdump_base(void);
extern unsigned int dbg_snapshot_get_max_core_num(void);

/* debug-snapshot-dpm functions */
extern bool dbg_snapshot_get_dpm_status(void);
extern bool dbg_snapshot_get_enabled_debug_kinfo(void);
extern void dbg_snapshot_do_dpm_policy(unsigned int policy, const char *str);
/* debug-snapshot-qd functions */
extern int dbg_snapshot_qd_add_region(void *v_entry, u32 attr);
extern bool dbg_snapshot_qd_enabled(void);
extern void dbg_snapshot_qd_dump_stack(u64 sp);
/*debug-snapshot-utils functions */
extern int dbg_snapshot_get_sjtag_status(void);
extern bool dbg_snapshot_get_reboot_status(void);
extern bool dbg_snapshot_get_panic_status(void);
extern void dbg_snapshot_set_core_cflush_stat(unsigned int val);
extern bool dbg_snapshot_get_warm_status(void);
extern void dbg_snapshot_ecc_dump(bool call_panic);
extern int dbg_snapshot_start_watchdog(int sec);
extern int dbg_snapshot_emergency_reboot(const char *str);
extern int dbg_snapshot_emergency_reboot_timeout(const char *str, int tick);
extern int dbg_snapshot_kick_watchdog(void);
extern unsigned int dbg_snapshot_get_val_offset(unsigned int offset);
extern void dbg_snapshot_set_val_offset(unsigned int val, unsigned int offset);
extern void dbg_snapshot_register_wdt_ops(int (*start)(bool, int, int),
					  int (*expire)(unsigned int, int), int (*stop)(int));
extern void dbg_snapshot_register_debug_ops(int (*halt)(void), int (*arraydump)(void),
					    int (*scandump)(void));
extern void dbg_snapshot_save_context(struct pt_regs *regs, bool stack_dump);
extern void cache_flush_all(void);
extern int dbg_snapshot_stop_all_cpus(void);
extern void dbg_snapshot_set_usb_otg(unsigned int val);
extern void dbg_snapshot_set_core_pmu_val(unsigned int val, unsigned int cpu);
extern unsigned int dbg_snapshot_get_core_pmu_val(unsigned int cpu);
extern void dbg_snapshot_set_core_ehld_stat(unsigned int val, unsigned int cpu);
extern unsigned int dbg_snapshot_get_core_ehld_stat(unsigned int cpu);

/* debug-snapshot-log functions */
extern int dbg_snapshot_get_freq_idx(const char *name);
extern void dbg_snapshot_get_freq_name(char (*freq_names)[DSS_FREQ_MAX_NAME_SIZE]);
extern unsigned int dbg_snapshot_get_freq_size(void);
extern void *dbg_snapshot_get_suspend_diag(void);

#define dbg_snapshot_get_timestamp()	local_clock()
extern void dbg_snapshot_cpuidle(char *modes, unsigned int state, s64 diff, int en);
extern void dbg_snapshot_cpuidle_mod(char *modes, unsigned int state, s64 diff, int en);
extern void dbg_snapshot_clk(struct clk_hw *clock, const char *func_name,
			    unsigned long arg, int mode);
extern void dbg_snapshot_regulator(unsigned long long timestamp, char *f_name,
				unsigned int addr, unsigned int volt,
				unsigned int rvolt, int en);
extern void dbg_snapshot_acpm(unsigned long long timestamp, const char *log,
				unsigned int data);
extern void dbg_snapshot_thermal(struct exynos_tmu_data *data, unsigned int temp,
				char *name, unsigned long long max_cooling);
extern void dbg_snapshot_pmu(int id, const char *func_name, int mode);
extern void dbg_snapshot_freq(int type, unsigned long old_freq,
				unsigned long target_freq, int en);
extern void dbg_snapshot_dm(int type, unsigned long min, unsigned long max,
				s32 wait_t, s32 t);
extern void dbg_snapshot_printk(const char *fmt, ...);
void dbg_snapshot_itmon_backup_log(const char *fmt, ...);
void dbg_snapshot_itmon_irq_received(void);

#define dss_extern_get_log_by_cpu(item)					\
extern long dss_get_len_##item##_log(void);				\
extern long dss_get_len_##item##_log_by_cpu(int cpu);			\
extern long dss_get_last_##item##_log_idx(int cpu);			\
extern long dss_get_first_##item##_log_idx(int cpu);			\
extern struct item##_log *dss_get_last_##item##_log(int cpu);		\
extern struct item##_log *dss_get_first_##item##_log(int cpu);		\
extern struct item##_log *dss_get_##item##_log_by_idx(int cpu, int idx);\
extern struct item##_log *dss_get_##item##_log_by_cpu_iter(int cpu, int idx);\
extern unsigned long dss_get_vaddr_##item##_log_by_cpu(int cpu)

#define dss_extern_get_log(item)					\
extern long dss_get_len_##item##_log(void);				\
extern long dss_get_last_##item##_log_idx(void);			\
extern long dss_get_first_##item##_log_idx(void);			\
extern struct item##_log *dss_get_last_##item##_log(void);		\
extern struct item##_log *dss_get_first_##item##_log(void);		\
extern struct item##_log *dss_get_##item##_log_by_idx(int idx);		\
extern struct item##_log *dss_get_##item##_log_iter(int idx);		\
extern unsigned long dss_get_vaddr_##item##_log(void)

static inline void dbg_snapshot_spin_func(void)
{
        while (1)
                wfi();
}
#else /* CONFIG_DEBUG_SNAPSHOT */
#define dbg_snapshot_get_timestamp()		(0)
#define dbg_snapshot_task(a, b)			do { } while (0)
#define dbg_snapshot_work(a, b, c, d)		do { } while (0)
#define dbg_snapshot_cpuidle(a, b, c, d)	do { } while (0)
#define dbg_snapshot_cpuidle_mod(a, b, c, d)	do { } while (0)
#define dbg_snapshot_acpm(a, b, c)		do { } while (0)
#define dbg_snapshot_regulator(a, b, c, d, e, f)	do { } while (0)
#define dbg_snapshot_thermal(a, b, c, d)	do { } while (0)
#define dbg_snapshot_irq(a, b, c, d, e)		do { } while (0)
#define dbg_snapshot_clk(a, b, c, d)		do { } while (0)
#define dbg_snapshot_pmu(a, b, c)		do { } while (0)
#define dbg_snapshot_freq(a, b, c, d)		do { } while (0)
#define dbg_snapshot_hrtimer(a, b, c, d)	do { } while (0)
#define dbg_snapshot_dm(a, b, c, d, e)		do { } while (0)
#define dbg_snapshot_printk(...)		do { } while (0)
#define dbg_snapshot_itmon_backup_log(a)	do { } while (0)
#define dbg_snapshot_itmon_irq_received(a)	do { } while (0)

#define dbg_snapshot_set_item_enable(a, b)	do { } while (0)
#define dbg_snapshot_output()			do { } while (0)
#define dbg_snapshot_log_output()		do { } while (0)
#define dbg_snapshot_scratch_clear()		do { } while (0)

#define dbg_snapshot_do_dpm_policy(a, b)	do { } while (0)
#define dbg_snapshot_qd_dump_stack(a)		do { } while (0)

#define dbg_snapshot_get_sjtag_status()		do { } while (0)
#define dbg_snapshot_panic_handler_safe()	do { } while (0)
#define dbg_snapshot_ecc_dump(a)		do { } while (0)
#define dbg_snapshot_register_wdt_ops(a, b, c)	do { } while (0)
#define dbg_snapshot_register_debug_ops(a, b, c)	do { } while (0)
#define dbg_snapshot_save_context(a, b)		do { } while (0)
#define cache_flush_all()			do { } while (0)

#define dbg_snapshot_set_core_pmu_val(a, b)	do { } while (0)
#define dbg_snapshot_get_core_pmu_val(a)	(0)
#define dbg_snapshot_set_core_ehld_stat(a, b)	do { } while (0)
#define dbg_snapshot_get_core_ehld_stat(a)	(0)

#define dbg_snapshot_set_val_offset(a)		do { } while (0)

#define dbg_snapshot_get_enable() 		(0)
#define dbg_snapshot_get_item_enable(a) 	(0)
#define dbg_snapshot_get_item_size(a)		(0)
#define dbg_snapshot_get_item_paddr(a) 		(0)
#define dbg_snapshot_get_item_vaddr(a) 		(0)
#define dbg_snapshot_is_scratch() 		(0)
#define dbg_snapshot_get_hardlockup_magic(a)	(0)
#define dbg_snapshot_get_last_pc(a)		(0)
#define dbg_snapshot_get_last_pc_paddr		(0)
#define dbg_snapshot_get_slcdump_base		(0)
#define dbg_snapshot_get_pre_slcdump_base	(0)
#define dbg_snapshot_get_max_core_num		(0)

#define dbg_snapshot_get_dpm_status() 		(0)
#define dbg_snapshot_qd_add_region(a, b)	(-1)
#define dbg_snapshot_qd_enabled()		(0)

#define dbg_snapshot_start_watchdog(a)		(-1)
#define dbg_snapshot_emergency_reboot(a)	(-1)
#define dbg_snapshot_emergency_reboot_timeout(a, b)	(-1)
#define dbg_snapshot_kick_watchdog()		(-1)
#define dbg_snapshot_get_val_offset(a)		(0)
#define dbg_snapshot_stop_all_cpus()		(-1)

#define dbg_snapshot_get_freq_idx(a)		(-1)
#define dbg_snapshot_get_freq_name(a)		(0)
#define dbg_snapshot_get_freq_size()		(0)

#define dss_extern_get_log_by_cpu(item)					\
static inline long dss_get_len_##item##_log(void) {			\
	return -1;							\
}									\
static inline long dss_get_len_##item##_log_by_cpu(int cpu) {		\
	return -1;							\
}									\
static inline long dss_get_last_##item##_log_idx(int cpu) {		\
	return -1;							\
}									\
static inline long dss_get_first_##item##_log_idx(int cpu) {		\
	return -1;							\
}									\
static inline struct item##_log *dss_get_last_##item##_log(int cpu) {	\
	return NULL;							\
}									\
static inline struct item##_log *dss_get_first_##item##_log(int cpu) {	\
	return NULL;							\
}									\
static inline struct item##_log *dss_get_##item##_log_by_idx(int cpu, int idx) { \
	return NULL;							\
}									\
static inline struct item##_log *dss_get_##item##_log_by_cpu_iter(int cpu, int idx) {\
	return NULL;							\
}									\
static inline unsigned long dss_get_vaddr_##item##_log_by_cpu(int cpu) {\
	return 0;							\
}

#define dss_extern_get_log(item)					\
static inline long dss_get_len_##item##_log(void) {			\
	return -1;							\
}									\
static inline long dss_get_last_##item##_log_idx(void) {		\
	return -1;							\
}									\
static inline long dss_get_first_##item##_log_idx(void) {		\
	return -1;							\
}									\
static inline struct item##_log *dss_get_last_##item##_log(void) {	\
	return NULL;							\
}									\
static inline struct item##_log *dss_get_first_##item##_log(void) {	\
	return NULL;							\
}									\
static inline struct item##_log *dss_get_##item##_log_by_idx(int idx) {	\
	return NULL;							\
}									\
static inline struct item##_log *dss_get_##item##_log_iter(int idx) {	\
	return NULL;							\
}									\
static inline unsigned long dss_get_vaddr_##item##_log(void) {		\
	return 0;							\
}

#define dbg_snapshot_spin_func()		do { } while (0)
#endif /* CONFIG_DEBUG_SNAPSHOT */

#define for_each_item_in_dss_by_cpu(item, cpu, start, len, direction)	\
	for (item = dss_get_##item##_log_by_cpu_iter(cpu, start);	\
		len && item;						\
		item = dss_get_##item##_log_by_cpu_iter(cpu,		\
				direction ? ++start : --start), --len)

#define dss_get_start_addr_of_log_by_cpu(cpu, item, vaddr)		\
	(vaddr ? dss_get_vaddr_##item##_log_by_cpu(cpu) :		\
		dss_get_vaddr_##item##_log_by_cpu(cpu) -		\
		dbg_snapshot_get_item_vaddr("log_kevents") +		\
		dbg_snapshot_get_item_paddr("log_kevents"))

#define for_each_item_in_dss(item, start, len, direction)		\
	for (item = dss_get_##item##_log_by_iter(start);		\
		len && item;						\
		item = dss_get_##item##_log_by_iter(			\
			direction ? ++start : --start), --len)

#define dss_get_start_addr_of_log(item, vaddr)				\
	(vaddr ? dss_get_vaddr_##item##_log() :				\
		dss_get_vaddr_##item##_log() -				\
		dbg_snapshot_get_item_vaddr("log_kevents") +		\
		dbg_snapshot_get_item_paddr("log_kevents"))

dss_extern_get_log_by_cpu(task);
dss_extern_get_log_by_cpu(work);
dss_extern_get_log_by_cpu(cpuidle);
dss_extern_get_log_by_cpu(freq);
dss_extern_get_log_by_cpu(irq);
dss_extern_get_log_by_cpu(hrtimer);
dss_extern_get_log(suspend);
dss_extern_get_log(clk);
dss_extern_get_log(pmu);
dss_extern_get_log(dm);
dss_extern_get_log(regulator);
dss_extern_get_log(thermal);
dss_extern_get_log(acpm);
dss_extern_get_log(print);

/**
 * dsslog_flag - added log information supported.
 * @DSS_FLAG_IN: Generally, marking into the function
 * @DSS_FLAG_ON: Generally, marking the status not in, not out
 * @DSS_FLAG_OUT: Generally, marking come out the function
 * @DSS_FLAG_SOFTIRQ: Marking to pass the softirq function
 * @DSS_FLAG_CALL_TIMER_FN: Marking to pass the timer function
 * @DSS_FLAG_SMP_CALL_FN: Marking to pass the smp call function
 */
enum dsslog_flag {
	DSS_FLAG_IN			= 1,
	DSS_FLAG_ON			= 2,
	DSS_FLAG_OUT			= 3,
	DSS_FLAG_SOFTIRQ		= 10000,
	DSS_FLAG_CALL_TIMER_FN		= 20000,
	DSS_FLAG_SMP_CALL_FN		= 30000,
};

enum dss_item_index {
	DSS_ITEM_HEADER_ID = 0,
	DSS_ITEM_KEVENTS_ID,
	DSS_ITEM_BCM_ID,
	DSS_ITEM_S2D_ID,
	DSS_ITEM_ARRDUMP_RESET_ID,
	DSS_ITEM_ARRDUMP_PANIC_ID,
	DSS_ITEM_SLCDUMP_ID,
	DSS_ITEM_PRE_SLCDUMP_ID,
	DSS_ITEM_ITMON_ID,
};

enum dss_log_item_indx {
	DSS_LOG_TASK_ID = 0,
	DSS_LOG_WORK_ID,
	DSS_LOG_CPUIDLE_ID,
	DSS_LOG_SUSPEND_ID,
	DSS_LOG_IRQ_ID,
	DSS_LOG_HRTIMER_ID,
	DSS_LOG_CLK_ID,
	DSS_LOG_PMU_ID,
	DSS_LOG_FREQ_ID,
	DSS_LOG_DM_ID,
	DSS_LOG_REGULATOR_ID,
	DSS_LOG_THERMAL_ID,
	DSS_LOG_ACPM_ID,
	DSS_LOG_PRINTK_ID,
};

struct dbg_snapshot_helper_ops {
	int (*start_watchdog)(bool reset, int timeout, int idx);
	int (*expire_watchdog)(unsigned int cnt, int idx);
	int (*stop_watchdog)(int idx);
	int (*stop_all_cpus)(void);
	int (*run_arraydump)(void);
	int (*run_scandump_mode)(void);
};

/**
 * remain comment to check CPU nick name
 *
#define ARM_CPU_PART_HERCULES		ARM_CPU_PART_CORTEX_A78
#define ARM_CPU_PART_HERA		ARM_CPU_PART_CORTEX_X1
#define ARM_CPU_PART_KLEIN		ARM_CPU_PART_CORTEX_A510
#define ARM_CPU_PART_MATTERHORN		ARM_CPU_PART_CORTEX_A710
#define ARM_CPU_PART_MATTERHORN_ELP	ARM_CPU_PART_CORTEX_X2
*/
#define ARM_CPU_PART_MAKALU		0xD4D
#define ARM_CPU_PART_MAKALU_ELP		0xD4E
#define ARM_CPU_PART_HAYDEN		0xD80
#define ARM_CPU_PART_HUNTER		0xD81
#define ARM_CPU_PART_HUNTER_ELP		0xD82

#define DSS_NR_CPUS_ZUMA		(9)
#define DSS_NR_CPUS_OTHERS		(8)
#if IS_ENABLED(CONFIG_SOC_ZUMA)
// ZUMA and ZUMAPRO: max(DSS_NR_CPUS_ZUMA, DSS_NR_CPUS_OTHERS)
#define DSS_NR_CPUS			DSS_NR_CPUS_ZUMA
#else
// GS101 and GS201
#define DSS_NR_CPUS			DSS_NR_CPUS_OTHERS
#endif

#endif
