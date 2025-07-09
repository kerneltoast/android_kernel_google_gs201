// SPDX-License-Identifier: GPL-2.0-only

#undef TRACE_SYSTEM
#define TRACE_SYSTEM pixel_mm

#if !defined(_PIXEL_MM_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _PIXEL_MM_TRACE_H

#include <linux/tracepoint.h>

/*
 * Called when the kswapd wakes up to do work. We're interested in the timestamp
 * of the event, and "unused" is here only because TP_PROTO() requires at least
 * one param.
 */
TRACE_EVENT(pixel_mm_kswapd_wake,
	TP_PROTO(int unused),
	TP_ARGS(unused),
	TP_STRUCT__entry(__field(int, unused)),
	TP_fast_assign(__entry->unused = unused),
	TP_printk("%s", "")
);

/*
 * Called after "kswapd_wake" above once the kswapd has finished reclaiming
 * pages (and is going to sleep).
 */
TRACE_EVENT(pixel_mm_kswapd_done,
	TP_PROTO(
		unsigned long delta_nr_scanned,
		unsigned long delta_nr_reclaimed
	),

	TP_ARGS(delta_nr_scanned, delta_nr_reclaimed),

	TP_STRUCT__entry(
		__field(unsigned long, delta_nr_scanned)
		__field(unsigned long, delta_nr_reclaimed)
	),

	TP_fast_assign(
		__entry->delta_nr_scanned = delta_nr_scanned;
		__entry->delta_nr_reclaimed = delta_nr_reclaimed;
	),

	TP_printk("delta_nr_scanned=%lu, delta_nr_reclaimed=%lu",
		__entry->delta_nr_scanned,
		__entry->delta_nr_reclaimed
	)
);

#endif /* _PIXEL_MM_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/soc/google/pixel_stat/mm
#define TRACE_INCLUDE_FILE pixel_mm_trace
#include <trace/define_trace.h>