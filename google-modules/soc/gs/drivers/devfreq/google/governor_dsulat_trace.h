/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 */

#if !defined(_TRACE_DSULAT_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DSULAT_TRACE_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM power

#include <linux/tracepoint.h>
#include <linux/trace_events.h>

TRACE_EVENT(dsulat_dev_meas,

	TP_PROTO(const char *name, unsigned int dev_id, unsigned long inst,
		 unsigned long l2_cachemiss, unsigned long freq,
		 unsigned long l2_cache_wb, unsigned long l3_cache_access,
		 unsigned long wb_pct, unsigned long mem_stall, unsigned int ratio),

	TP_ARGS(name, dev_id, inst, l2_cachemiss, freq,  l2_cache_wb, l3_cache_access, wb_pct, mem_stall, ratio),

	TP_STRUCT__entry(
		__string(name, name)
		__field(unsigned int, dev_id)
		__field(unsigned long, inst)
		__field(unsigned long, l2_cachemiss)
		__field(unsigned long, freq)
		__field(unsigned long, l2_cache_wb)
		__field(unsigned long, l3_cache_access)
		__field(unsigned long, wb_pct)
		__field(unsigned long, mem_stall)
		__field(unsigned int, ratio)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->dev_id = dev_id;
		__entry->inst = inst;
		__entry->l2_cachemiss = l2_cachemiss;
		__entry->freq = freq;
		__entry->l2_cache_wb = l2_cache_wb;
		__entry->l3_cache_access = l3_cache_access;
		__entry->wb_pct = wb_pct;
		__entry->mem_stall = mem_stall;
		__entry->ratio = ratio;
	),

	TP_printk("dev: %s, id=%u, inst=%lu, l2_cachemiss=%lu, freq=%lu,  l2_cache_wb=%lu, l3_cache_access=%lu, wb_pct=%lu, mem_stall=%lu, ratio=%u",
		__get_str(name),
		__entry->dev_id,
		__entry->inst,
		__entry->l2_cachemiss,
		__entry->freq,
		__entry->l2_cache_wb,
		__entry->l3_cache_access,
		__entry->wb_pct,
		__entry->mem_stall,
		__entry->ratio)
);

TRACE_EVENT(dsulat_dev_update,

	TP_PROTO(const char *name, unsigned int dev_id, bool latency_mode, unsigned long inst,
		 unsigned long l2_cachemiss, unsigned long freq,
		 unsigned long l2_cache_wb, unsigned long l3_cache_access,
		 unsigned long wb_pct, unsigned long mem_stall, unsigned long vote),

	TP_ARGS(name, dev_id, latency_mode, inst, l2_cachemiss, freq, l2_cache_wb, l3_cache_access, wb_pct, mem_stall, vote),

	TP_STRUCT__entry(
		__string(name, name)
		__field(unsigned int, dev_id)
		__field(bool, latency_mode)
		__field(unsigned long, inst)
		__field(unsigned long, l2_cachemiss)
		__field(unsigned long, freq)
		__field(unsigned long, l2_cache_wb)
		__field(unsigned long, l3_cache_access)
		__field(unsigned long, wb_pct)
		__field(unsigned long, mem_stall)
		__field(unsigned long, vote)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->dev_id = dev_id;
		__entry->latency_mode = latency_mode;
		__entry->inst = inst;
		__entry->l2_cachemiss = l2_cachemiss;
		__entry->freq = freq;
		__entry->l2_cache_wb = l2_cache_wb;
		__entry->l3_cache_access = l3_cache_access;
		__entry->wb_pct = wb_pct;
		__entry->mem_stall = mem_stall;
		__entry->vote = vote;
	),

	TP_printk("dev: %s, id=%u, latency_mode=%d, inst=%lu, l2_cachemiss=%lu, freq=%lu, l2_cache_wb=%lu, l3_cache_access=%lu, wb_pct=%lu, mem_stall=%lu, vote=%lu",
		__get_str(name),
		__entry->dev_id,
		__entry->latency_mode,
		__entry->inst,
		__entry->l2_cachemiss,
		__entry->freq,
		__entry->l2_cache_wb,
		__entry->l3_cache_access,
		__entry->wb_pct,
		__entry->mem_stall,
		__entry->vote)
);

#endif /* _TRACE_DSULAT_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE governor_dsulat_trace
#include <trace/define_trace.h>
