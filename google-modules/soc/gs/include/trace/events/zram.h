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

#if IS_ENABLED(CONFIG_ZRAM_GS_WRITEBACK)
TRACE_EVENT(zram_read_from_bdev,

	TP_PROTO(struct zram *zram, unsigned long entry_element),

	TP_ARGS(zram, entry_element),

	TP_STRUCT__entry(
		__field(unsigned long,	nr_bd_read)
		__field(unsigned long,	entry_element)
	),

	TP_fast_assign(
		__entry->nr_bd_read	= zram_stat_read(zram, NR_BD_READ);
		__entry->entry_element	= entry_element;
	),

	TP_printk("nr_bd_read=%lu entry_element=%lu",
			__entry->nr_bd_read,
			__entry->entry_element)
);
#endif

#endif /* _TRACE_ZRAM_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
