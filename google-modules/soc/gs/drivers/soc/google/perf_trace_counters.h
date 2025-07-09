/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2013-2014,2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM perf_trace_counters

#if !defined(_PERF_TRACE_COUNTERS_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _PERF_TRACE_COUNTERS_H_

#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/perf_event.h>
#include <linux/sched.h>
#include <linux/tracepoint.h>

#if IS_ENABLED(CONFIG_GS_PERF_MON)
#include <performance/gs_perf_mon/gs_perf_mon.h>
#else
#include "../../devfreq/google/governor_memlat.h"
#endif

/*
 * Legacy AMU/PMU common event index
 * 0: STALL_IDX - skipped
 * 1: L2D_CACHE_REFILL_IDX - skipped
 * 2: STALL_BACKEND_MEM_IDX
 * 3: L3_CACHE_MISS_IDX
 * 4: INST_IDX
 * 5: CYCLE_IDX
 *
 * gs_perf_mon AMU/PMU common event index
 * 0: PERF_L2D_CACHE_REFILL_IDX - skipped
 * 1: PERF_STALL_BACKEND_MEM_IDX
 * 2: PERF_L3_CACHE_MISS_IDX
 * 3: PERF_INST_IDX
 * 4: PERF_CYCLE_IDX
 */
#define NUM_EVENTS 4
extern const unsigned int ev_idx[NUM_EVENTS];

extern int read_perf_event_local(int cpu, unsigned int event_id, u64 *count);

DECLARE_PER_CPU(u64[NUM_EVENTS], previous_cnts);
DECLARE_PER_CPU(struct perf_event *, perf_event);
TRACE_EVENT(sched_switch_with_ctrs,

	TP_PROTO(struct task_struct *prev),

	TP_ARGS(prev),

	TP_STRUCT__entry(
		__array(char,	prev_comm,	TASK_COMM_LEN)
		__field(pid_t,	prev_pid)
		__field(u32, cyc)
		__field(u32, inst)
		__field(u32, stallbm)
		__field(u32, l3dm)
	),

	TP_fast_assign(
		u32 cpu = raw_smp_processor_id();
		u32 i;
		u64 count;
		u64 total_cnt;
		u32 delta_cnts[NUM_EVENTS];

		memcpy(__entry->prev_comm, prev->comm, TASK_COMM_LEN);
		__entry->prev_pid	= prev->pid;

		/* Read the PMU event counts from performance monitor. */
		for (i = 0; i < NUM_EVENTS; i++) {
			total_cnt = read_perf_event_local(cpu, ev_idx[i],
				&count) ? 0 : count;
			delta_cnts[i] = (uint32_t) (total_cnt -
					per_cpu(previous_cnts[i], cpu));
			per_cpu(previous_cnts[i], cpu) = total_cnt;
		}

		__entry->inst = delta_cnts[0];
		__entry->cyc = delta_cnts[1];
		__entry->stallbm = delta_cnts[2];
		__entry->l3dm = delta_cnts[3];
	),

	TP_printk("prev_comm=%s, prev_pid=%d, CYC=%u, INST=%u, STALLBM=%u, L3DM=%u",
		__entry->prev_comm,
		__entry->prev_pid,
		__entry->cyc,
		__entry->inst,
		__entry->stallbm,
		__entry->l3dm)
);

#endif
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/soc/google
#define TRACE_INCLUDE_FILE perf_trace_counters
#include <trace/define_trace.h>
