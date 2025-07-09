/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * LWIS Trace Event
 *
 * Copyright (c) 2021 Google Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM lwis

#if !defined(_TRACE_LWIS_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_LWIS_H_

#include <linux/tracepoint.h>
#include "lwis_commands.h"
#include "lwis_device.h"

#define LWIS_DEVICE_NAME_ENTRY __array(char, lwis_name, LWIS_MAX_NAME_STRING_LEN)
#define LWIS_DEVICE_NAME_ASSIGN                                                                    \
	strscpy(__entry->lwis_name, lwis_dev->name, LWIS_MAX_NAME_STRING_LEN)
#define LWIS_TRACE_DEVICE_NAME __entry->lwis_name

TRACE_EVENT(tracing_mark_write,
	    TP_PROTO(struct lwis_device *lwis_dev, char type, int pid, const char *func_name,
		     int64_t value),
	    TP_ARGS(lwis_dev, type, pid, func_name, value),
	    TP_STRUCT__entry(LWIS_DEVICE_NAME_ENTRY __field(char, type) __field(int, pid)
				     __string(func_name, func_name) __field(int64_t, value)),
	    TP_fast_assign(LWIS_DEVICE_NAME_ASSIGN; __entry->type = type; __entry->pid = pid;
			   __assign_str(func_name, func_name); __entry->value = value;),
	    TP_printk("%c|%d|lwis-%s:%s|%lld", __entry->type, __entry->pid, LWIS_TRACE_DEVICE_NAME,
		      __get_str(func_name), __entry->value));

#define LWIS_ATRACE_BEGIN(lwis_dev, func_name)                                                     \
	trace_tracing_mark_write(lwis_dev, 'B', current->tgid, func_name, 0)
#define LWIS_ATRACE_FUNC_BEGIN(lwis_dev, func_name) LWIS_ATRACE_BEGIN(lwis_dev, func_name)

#define LWIS_ATRACE_END(lwis_dev, func_name)                                                       \
	trace_tracing_mark_write(lwis_dev, 'E', current->tgid, func_name, 0)
#define LWIS_ATRACE_FUNC_END(lwis_dev, func_name) LWIS_ATRACE_END(lwis_dev, func_name)

#define LWIS_ATRACE_FUNC_INT_BEGIN(lwis_dev, func_name, value)                                     \
	trace_tracing_mark_write(lwis_dev, 'B', current->tgid, func_name, value)

#define LWIS_ATRACE_FUNC_INT_END(lwis_dev, func_name, value)                                       \
	trace_tracing_mark_write(lwis_dev, 'E', current->tgid, func_name, value)

#define LWIS_ATRACE_INT_PID(lwis_dev, func_name, value, pid)                                       \
	trace_tracing_mark_write(lwis_dev, 'C', pid, func_name, value)
#define LWIS_ATRACE_INT(lwis_dev, func_name, value)                                                \
	LWIS_ATRACE_INT_PID(lwis_dev, func_name, value, current->tgid)

#endif /* _TRACE_LWIS_H_ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE lwis_trace

/* This part must be outside protection */
#include <trace/define_trace.h>
