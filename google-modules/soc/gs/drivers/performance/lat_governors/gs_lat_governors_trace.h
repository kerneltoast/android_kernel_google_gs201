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
#if !defined(_GS_LATGOV_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _GS_LATGOV_TRACE_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM gs_perf

#include <linux/tracepoint.h>
#include <linux/trace_events.h>

TRACE_EVENT(gs_lat_governor,
	TP_PROTO(const char *name, int cpu, unsigned int ratio, unsigned int stall,
		unsigned long governor_freq, unsigned long effective_cpu_freq),

	TP_ARGS(name, cpu, ratio, stall, governor_freq, effective_cpu_freq),

	TP_STRUCT__entry(
		__string(name, 		name			)
		__field(int,		cpu			)
		__field(unsigned long,	ratio			)
		__field(unsigned long,	stall			)
		__field(unsigned long,	governor_freq		)
		__field(unsigned long,	cpufreq			)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->cpu = cpu;
		__entry->ratio = ratio;
		__entry->stall = stall;
		__entry->governor_freq = governor_freq;
		__entry->cpufreq = effective_cpu_freq;
	),

	TP_printk("device=%s, cpu=%d, ratio=%lu, "
		"stall=%lu, governor_freq=%lu, cpufreq=%lu\n",
		__get_str(name),
		__entry->cpu,
		__entry->ratio,
		__entry->stall,
		__entry->governor_freq,
		__entry->cpufreq
	)
);

#endif /* _GS_LATGOV_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/performance/lat_governors
#define TRACE_INCLUDE_FILE gs_lat_governors_trace
#include <trace/define_trace.h>