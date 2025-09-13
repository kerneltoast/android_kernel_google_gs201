/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung DMA-BUF heap trace points.
 *
 * Copyright (C) 2021 Google LLC.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM dmabuf_heap

#if !defined(_DMABUF_HEAP_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _DMABUF_HEAP_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(dma_heap_stat,
	    TP_PROTO(unsigned long inode, long len,
		     unsigned long total_allocated),
	    TP_ARGS(inode, len, total_allocated),
	    TP_STRUCT__entry(
		__field(unsigned long, inode)
		__field(long, len)
		__field(unsigned long, total_allocated)
	    ),
	    TP_fast_assign(
		__entry->inode = inode;
		__entry->len = len;
		__entry->total_allocated = total_allocated;
	    ),
	    TP_printk("inode=%lu len=%ldB total_allocated=%luB",
		      __entry->inode,
		      __entry->len,
		      __entry->total_allocated)
);

#endif /* _DMABUF_HEAP_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE dmabuf_heap_trace
#include <trace/define_trace.h>
