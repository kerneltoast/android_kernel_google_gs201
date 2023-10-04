/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DPU trace support
 *
 * Copyright (C) 2020 Google, Inc.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM dpu

#if !defined(_DPU_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _DPU_TRACE_H_

#include <linux/tracepoint.h>

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

#define DPU_ATRACE_INT_PID(name, value, pid) trace_tracing_mark_write('C', pid, name, value)
#define DPU_ATRACE_INT(name, value) DPU_ATRACE_INT_PID(name, value, current->tgid)
#define DPU_ATRACE_BEGIN(name) trace_tracing_mark_write('B', current->tgid, name, 0)
#define DPU_ATRACE_END(name) trace_tracing_mark_write('E', current->tgid, "", 0)

#endif /* _DPU_TRACE_H_ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/.

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE dpu_trace

/* This part must be outside protection */
#include <trace/define_trace.h>
