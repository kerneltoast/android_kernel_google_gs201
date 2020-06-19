/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM zram

#if !defined(_TRACE_ZRAM_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ZRAM_H

#include <linux/types.h>
#include <linux/tracepoint.h>
#include <linux/mm.h>

TRACE_EVENT(zcomp_decompress_start,

	TP_PROTO(struct page *page, u32 index),

	TP_ARGS(page, index),

	TP_STRUCT__entry(
		__field( 	unsigned long,	pfn)
		__field(	u32,		index)
	),

	TP_fast_assign(
		__entry->pfn		= page_to_pfn(page);
		__entry->index		= index;
	),

	TP_printk("pfn=%lu index=%d",
			__entry->pfn,
			__entry->index)
);

TRACE_EVENT(zcomp_decompress_end,

	TP_PROTO(struct page *page, u32 index),

	TP_ARGS(page, index),

	TP_STRUCT__entry(
		__field( 	unsigned long,	pfn)
		__field(	u32,		index)
	),

	TP_fast_assign(
		__entry->pfn		= page_to_pfn(page);
		__entry->index		= index;
	),

	TP_printk("pfn=%lu index=%d",
			__entry->pfn,
			__entry->index)
);
#endif /* _TRACE_ZRAM_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
