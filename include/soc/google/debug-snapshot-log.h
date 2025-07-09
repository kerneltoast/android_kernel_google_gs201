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

#ifndef DEBUG_SNAPSHOT_LOG_H
#define DEBUG_SNAPSHOT_LOG_H

#include <linux/clk-provider.h>
#include <soc/google/debug-snapshot.h>

/*  Length domain */
#define DSS_LOG_STRING_LEN		SZ_128

#define DSS_DOMAIN_NUM			CONFIG_DEBUG_SNAPSHOT_FREQ_DOMAIN_NUM
#define DSS_LOG_MAX_NUM			CONFIG_DEBUG_SNAPSHOT_LOG_ITEM_SIZE
#define DSS_ITMON_LOG_MAX_LEN		(DSS_LOG_ITMON_SIZE - 4)
/* ASCII presentaion of IMMI (Itmon Magic Memory Initialized) */
#define DSS_ITMON_MAGIC_INITIALIZED	0x494D4D49
/* ASCII presentaion of IMIR (Itmon Magic Irq Received) */
#define DSS_ITMON_MAGIC_IRQ_RECEIVED	0x494D4952

#define DSS_CALLSTACK_MAX_NUM		3
#define TASK_COMM_LEN			16

struct task_log {
	unsigned long long time;
	struct task_struct *task;
	char task_comm[TASK_COMM_LEN];
	unsigned long se_exec_start;
	int pid;
};

struct work_log {
	unsigned long long time;
	work_func_t fn;
	int en;
};

struct cpuidle_log {
	unsigned long long time;
	char *modes;
	unsigned int state;
	u32 num_online_cpus;
	int delta;
	int en;
};

struct suspend_log {
	unsigned long long time;
	const char *log;
	const char *dev;
	int en;
	int event;
#if IS_ENABLED(CONFIG_PIXEL_SUSPEND_DIAG)
	short core;
	unsigned short delta_time_h;
	unsigned int delta_time_l;
#else
	int core;
#endif
};

struct irq_log {
	unsigned long long time;
	int irq;
	void *fn;
	struct irq_desc *desc;
	int en;
};

struct clk_log {
	unsigned long long time;
	struct clk_hw *clk;
	const char *f_name;
	int mode;
	unsigned long arg;
};

struct pmu_log {
	unsigned long long time;
	unsigned int id;
	const char *f_name;
	int mode;
};

struct freq_log {
	unsigned long long time;
	int cpu;
	const char *freq_name;
	unsigned long old_freq;
	unsigned long target_freq;
	int en;
};

struct dm_log {
	unsigned long long time;
	int cpu;
	int dm_num;
	unsigned long min_freq;
	unsigned long max_freq;
	s32 wait_dmt;
	s32 do_dmt;
};

struct hrtimer_log {
	unsigned long long time;
	s64 now;
	struct hrtimer *timer;
	void *fn;
	int en;
};

struct regulator_log {
	unsigned long long time;
	unsigned long long acpm_time;
	int cpu;
	char name[SZ_16];
	unsigned int reg;
	unsigned int voltage;
	unsigned int raw_volt;
	int en;
};

struct thermal_log {
	unsigned long long time;
	int cpu;
	struct exynos_tmu_data *data;
	unsigned int temp;
	char *cooling_device;
	unsigned long long cooling_state;
};

struct acpm_log {
	unsigned long long time;
	unsigned long long acpm_time;
	char log[9];
	unsigned int data;
};

struct print_log {
	unsigned long long time;
	int cpu;
	char log[DSS_LOG_STRING_LEN];
};

struct itmon_logs {
	u32 magic;
	char log[DSS_ITMON_LOG_MAX_LEN];
} __packed;

struct dbg_snapshot_log {
	struct task_log task[DSS_NR_CPUS][DSS_LOG_MAX_NUM];
	struct work_log work[DSS_NR_CPUS][DSS_LOG_MAX_NUM];
	struct cpuidle_log cpuidle[DSS_NR_CPUS][DSS_LOG_MAX_NUM];
	struct suspend_log suspend[DSS_LOG_MAX_NUM * 2];
	struct irq_log irq[DSS_NR_CPUS][DSS_LOG_MAX_NUM * 4];
	struct clk_log clk[DSS_LOG_MAX_NUM];
	struct pmu_log pmu[DSS_LOG_MAX_NUM];
	struct freq_log freq[DSS_DOMAIN_NUM][DSS_LOG_MAX_NUM / 2];
	struct dm_log dm[DSS_LOG_MAX_NUM];
	struct hrtimer_log hrtimer[DSS_NR_CPUS][DSS_LOG_MAX_NUM];
	struct regulator_log regulator[DSS_LOG_MAX_NUM];
	struct thermal_log thermal[DSS_LOG_MAX_NUM];
	struct acpm_log acpm[DSS_LOG_MAX_NUM];
	struct print_log print[DSS_LOG_MAX_NUM * 2];
};

struct dbg_snapshot_log_misc {
	atomic_t task_log_idx[DSS_NR_CPUS];
	atomic_t work_log_idx[DSS_NR_CPUS];
	atomic_t cpuidle_log_idx[DSS_NR_CPUS];
	atomic_t suspend_log_idx;
	atomic_t irq_log_idx[DSS_NR_CPUS];
	atomic_t hrtimer_log_idx[DSS_NR_CPUS];
	atomic_t clk_log_idx;
	atomic_t pmu_log_idx;
	atomic_t freq_log_idx[SZ_32];
	atomic_t dm_log_idx;
	atomic_t regulator_log_idx;
	atomic_t thermal_log_idx;
	atomic_t print_log_idx;
	atomic_t acpm_log_idx;
};
#endif
