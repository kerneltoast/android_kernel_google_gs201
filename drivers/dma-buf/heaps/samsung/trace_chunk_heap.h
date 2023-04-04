/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM chunk_heap

#if !defined(_TRACE_CHUNK_HEAP_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CHUNK_HEAP_H

#include <linux/tracepoint.h>
#include <linux/dma-buf.h>

TRACE_EVENT(chunk_heap_allocate,

	TP_PROTO(struct dma_buf *dmabuf, const char *name, unsigned long len,
		 unsigned long nr_chunks),

	TP_ARGS(dmabuf, name, len, nr_chunks),

	TP_STRUCT__entry(
		__field(struct dma_buf *, dmabuf)
		__string(name, name)
		__field(unsigned long, len)
		__field(unsigned long, nr_chunks)
	),

	TP_fast_assign(
		__entry->dmabuf		= dmabuf;
		__assign_str(name, name);
		__entry->len		= len;
		__entry->nr_chunks	= nr_chunks;
	),

	TP_printk("dmabuf=%p name=%s len=%lu nr_chunks=%lu",
		  __entry->dmabuf, __get_str(name), __entry->len,
		  __entry->nr_chunks)
);

#endif /* _TRACE_CHUNK_HEAP_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace_chunk_heap
#include <trace/define_trace.h>
