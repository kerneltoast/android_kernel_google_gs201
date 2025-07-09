/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 */

#if !defined(_TRACE_MEMLAT_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MEMLAT_TRACE_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM power

#include <linux/tracepoint.h>
#include <linux/trace_events.h>

TRACE_EVENT(memlat_dev_meas,

	TP_PROTO(const char *name, unsigned int dev_id, unsigned long inst,
		 unsigned long mem, unsigned long freq, unsigned int stall,
		 unsigned long mem_stall, unsigned long l2_cachemiss, unsigned int ratio),

	TP_ARGS(name, dev_id, inst, mem, freq, stall, mem_stall, l2_cachemiss, ratio),

	TP_STRUCT__entry(
		__string(name, name)
		__field(unsigned int, dev_id)
		__field(unsigned long, inst)
		__field(unsigned long, mem)
		__field(unsigned long, freq)
		__field(unsigned int, stall)
		__field(unsigned long, mem_stall)
		__field(unsigned long, l2_cachemiss)
		__field(unsigned int, ratio)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->dev_id = dev_id;
		__entry->inst = inst;
		__entry->mem = mem;
		__entry->freq = freq;
		__entry->stall = stall;
		__entry->mem_stall = mem_stall;
		__entry->l2_cachemiss = l2_cachemiss;
		__entry->ratio = ratio;
	),

	TP_printk("dev: %s, id=%u, inst=%lu, mem=%lu, freq=%lu, stall=%u, mem_stall=%lu, l2_cachemiss=%lu, ratio=%u",
		__get_str(name),
		__entry->dev_id,
		__entry->inst,
		__entry->mem,
		__entry->freq,
		__entry->stall,
		__entry->mem_stall,
		__entry->l2_cachemiss,
		__entry->ratio)
);

TRACE_EVENT(memlat_dev_update,

	TP_PROTO(const char *name, unsigned int dev_id, unsigned long inst,
		 unsigned long mem, unsigned long freq, unsigned long mem_stall,
		 unsigned long l2_cachemiss, unsigned long vote),

	TP_ARGS(name, dev_id, inst, mem, freq, mem_stall, l2_cachemiss, vote),

	TP_STRUCT__entry(
		__string(name, name)
		__field(unsigned int, dev_id)
		__field(unsigned long, inst)
		__field(unsigned long, mem)
		__field(unsigned long, freq)
		__field(unsigned long, mem_stall)
		__field(unsigned long, l2_cachemiss)
		__field(unsigned long, vote)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->dev_id = dev_id;
		__entry->inst = inst;
		__entry->mem = mem;
		__entry->freq = freq;
		__entry->mem_stall = mem_stall;
		__entry->l2_cachemiss = l2_cachemiss;
		__entry->vote = vote;
	),

	TP_printk("dev: %s, id=%u, inst=%lu, mem=%lu, freq=%lu, mem_stall=%lu, l2_cachemiss=%lu, vote=%lu",
		__get_str(name),
		__entry->dev_id,
		__entry->inst,
		__entry->mem,
		__entry->freq,
		__entry->mem_stall,
		__entry->l2_cachemiss,
		__entry->vote)
);

#endif /* _TRACE_MEMLAT_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE governor_memlat_trace
#include <trace/define_trace.h>
