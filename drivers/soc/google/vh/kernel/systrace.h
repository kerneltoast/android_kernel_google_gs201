/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Systrace Kernel Tracing Integration
 *
 * Copyright (C) 2019 Google, Inc.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM systrace

#if !defined(_TRACE_SYSTRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SYSTRACE_H

#include <linux/tracepoint.h>

/* make declarations singleton at recursive inclusion path */
#if !defined(_SYSTRACE_RECURSIVE_DECLARATIONS)
#define _SYSTRACE_RECURSIVE_DECLARATIONS
enum {
	SYSTRACE_EVENT_BEGIN	= (1UL << 0),
	SYSTRACE_EVENT_END	= (1UL << 1),
	SYSTRACE_EVENT_INT64	= (1UL << 2),
};
#endif /* _SYSTRACE_RECURSIVE_DECLARATIONS */

#define systrace_event_type(flag)		\
	__print_flags(flag, "",			\
		{ SYSTRACE_EVENT_BEGIN,	"B" },	\
		{ SYSTRACE_EVENT_END,	"E" },	\
		{ SYSTRACE_EVENT_INT64,	"C" })

/*
 * To comply with systrace format: [BEC]|pid|name|value
 *
 * Note:
 * 0 can be recognized by catapult ftrace importer but should be deprecated.
 * once we get a new dedicated parsing tag like tracing_mark_write.
 */
TRACE_EVENT(0,
	TP_PROTO(int flag, int pid, const char *name, int64_t value),
	TP_ARGS(flag, pid, name, value),
	TP_STRUCT__entry(
		__field(int, flag)
		__field(int, pid)
		__string(name, name)
		__field(int64_t, value)
	),
	TP_fast_assign(
		__entry->flag = flag;
		__entry->pid = pid;
		__assign_str(name, name);
		__entry->value = value;
	),
	TP_printk("%s|%d|%s|%lld",
		systrace_event_type(__entry->flag),
		__entry->pid,
		__get_str(name),
		__entry->value
	)
);

#define __ATRACE_INT_PID(pid, name, value) \
	trace_0(SYSTRACE_EVENT_INT64,	\
		pid,			\
		name,			\
		value)

#define __ATRACE_BEGIN_PID(pid, name)	\
	trace_0(SYSTRACE_EVENT_BEGIN,	\
		pid,			\
		name,			\
		0)

#define __ATRACE_END_PID(pid, name)	\
	trace_0(SYSTRACE_EVENT_END,	\
		pid,			\
		name,			\
		0)

#define ATRACE_INT(name, value)		\
	__ATRACE_INT_PID(current->tgid, name, value)

#define ATRACE_BEGIN(name)		\
	__ATRACE_BEGIN_PID(current->tgid, name)

#define ATRACE_END()			\
	__ATRACE_END_PID(current->tgid, "")

#define ATRACE_BLOCK(name, block)	\
do {					\
	ATRACE_BEGIN(name);		\
	(block);			\
	ATRACE_END();			\
} while (0)

#endif /* _TRACE_SYSTRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE systrace
#include <trace/define_trace.h>
