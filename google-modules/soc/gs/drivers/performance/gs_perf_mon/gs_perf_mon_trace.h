/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2024 The Linux Foundation. All rights reserved.
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
#if !defined(_GS_PERF_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _GS_PERF_TRACE_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM gs_perf

#include <linux/tracepoint.h>
#include <linux/trace_events.h>
#include <performance/gs_perf_mon/gs_perf_mon.h>

TRACE_EVENT(gs_perf_mon,
	TP_PROTO(int cpu, struct gs_cpu_perf_data *cpu_data),

	TP_ARGS(cpu, cpu_data),

	TP_STRUCT__entry(
		__field(int,		cpu		)
		__field(unsigned long,	time_delta_us	)
		__field(unsigned long,	instructions	)
		__field(unsigned long,	cpu_cycles	)
		__field(unsigned long,	l3_cachemiss	)
		__field(unsigned long,	l2_cachemiss	)
		__field(unsigned long,	mem_stalls	)
	),

	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->time_delta_us = cpu_data->time_delta_us;
		__entry->instructions = cpu_data->perf_ev_last_delta[PERF_INST_IDX];
		__entry->cpu_cycles = cpu_data->perf_ev_last_delta[PERF_CYCLE_IDX];
		__entry->l3_cachemiss = cpu_data->perf_ev_last_delta[PERF_L3_CACHE_MISS_IDX];
		__entry->l2_cachemiss = cpu_data->perf_ev_last_delta[PERF_L2D_CACHE_REFILL_IDX];
		__entry->mem_stalls = cpu_data->perf_ev_last_delta[PERF_STALL_BACKEND_MEM_IDX];
	),

	TP_printk("cpu=%d, time_delta_us=%lu, inst=%lu, cpu_cycles=%lu, mem_stall_backend=%lu, "
			"l2_cachemiss=%lu, l3_cachemiss=%lu\n",
		__entry->cpu,
		__entry->time_delta_us,
		__entry->instructions,
		__entry->cpu_cycles,
		__entry->mem_stalls,
		__entry->l2_cachemiss,
		__entry->l3_cachemiss
	)
);

#endif /* _GS_PERF_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/performance/gs_perf_mon
#define TRACE_INCLUDE_FILE gs_perf_mon_trace
#include <trace/define_trace.h>