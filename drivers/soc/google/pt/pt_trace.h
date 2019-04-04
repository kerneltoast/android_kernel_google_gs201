/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * pt_trace.h
 *
 * System cache management.
 *
 * Copyright 2019 Google LLC
 *
 * Author: cozette@google.com
 */

#if !defined(_PT_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _PT_TRACE_H_

#include "./pt.h"
#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM pt
#define TRACE_INCLUDE_FILE pt_trace

TRACE_EVENT(pt_enable,
		TP_PROTO(const char *node_name, const char *id_name,
			bool enable, u32 ptid),
		TP_ARGS(node_name, id_name, enable, ptid),

		TP_STRUCT__entry(
				__field(const char *, node_name)
				__field(const char *, id_name)
				__field(bool, enable)
				__field(u32, ptid)
				),

		TP_fast_assign(
				__entry->node_name = node_name;
				__entry->id_name = id_name;
				__entry->enable = enable;
				__entry->ptid = ptid;
				),

		TP_printk("Node %s %s %s %d",
				__entry->node_name,
				__entry->id_name,
				__entry->enable?"ENABLE":"DISABLE",
				(int)__entry->ptid)
);

TRACE_EVENT(pt_resize_callback,
		TP_PROTO(const char *node_name, const char *id_name,
			bool enter, u64 size, u32 ptid),
		TP_ARGS(node_name, id_name, enter, size, ptid),

		TP_STRUCT__entry(
				__field(const char *, node_name)
				__field(const char *, id_name)
				__field(bool, enter)
				__field(u64, size)
				__field(u32, ptid)
				),

		TP_fast_assign(
				__entry->node_name = node_name;
				__entry->id_name = id_name;
				__entry->enter = enter;
				__entry->size = size;
				__entry->ptid = ptid;
				),

		TP_printk("Node %s %s %s 0x%llx ptid %d",
				__entry->node_name,
				__entry->id_name,
				__entry->enter?"ENTER":"EXIT",
				__entry->size,
				(int)__entry->ptid)
);

#endif /* _PT_TRACE_H_ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/soc/google/pt

#include <trace/define_trace.h>
