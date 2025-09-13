/* SPDX-License-Identifier: GPL-2.0 */
/*
 * G2D trace support.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM g2d

#if !defined(_G2D_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _G2D_TRACE_H_

#include <linux/tracepoint.h>

TRACE_EVENT(g2d_perf_update_qos,
	TP_PROTO(u64 rbw, u64 wbw, u32 devfreq),
	TP_ARGS(rbw, wbw, devfreq),
	TP_STRUCT__entry(
		__field(u64, rbw)
		__field(u64, wbw)
		__field(u32, devfreq)
	),
	TP_fast_assign(
		__entry->rbw = rbw;
		__entry->wbw = wbw;
		__entry->devfreq = devfreq;
	),
	TP_printk("rbw=%lld wbw=%lld devfreq=%d",
		__entry->rbw, __entry->wbw, __entry->devfreq)
);

TRACE_EVENT(tracing_mark_write,
	TP_PROTO(char type, int pid, const char *name, int value),
	TP_ARGS(type, pid, name, value),
	TP_STRUCT__entry(
		__field(char, type)
		__field(int, pid)
		__string(name, name)
		__field(int, value)
	),
	TP_fast_assign(
		__entry->type = type;
		__entry->pid = pid;
		__assign_str(name, name);
		__entry->value = value;
	),
	TP_printk("%c|%d|%s|%d",
		__entry->type, __entry->pid, __get_str(name), __entry->value)
);

#define G2D_ATRACE_INT_PID(name, value, pid) trace_tracing_mark_write('C', pid, name, value)
#define G2D_ATRACE_INT(name, value) G2D_ATRACE_INT_PID(name, value, current->tgid)
#define G2D_ATRACE_BEGIN(name) trace_tracing_mark_write('B', current->tgid, name, 0)
#define G2D_ATRACE_END() trace_tracing_mark_write('E', current->tgid, "", 0)

#endif /* _G2D_TRACE_H_ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE g2d_trace

/* This part must be outside protection */
#include <trace/define_trace.h>
