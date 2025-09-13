/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM bcl_exynos

#if !defined(_TRACE_EXYNOS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_BCL_EXYNOS_H

#include <linux/tracepoint.h>

TRACE_EVENT(bcl_irq_trigger,
	TP_PROTO(int id, int throttle, int cpu0_limit, int cpu1_limit,
	         int cpu2_limit, int tpu_limit, int gpu_limit, int voltage, int capacity),

	TP_ARGS(id, throttle, cpu0_limit, cpu1_limit, cpu2_limit, tpu_limit, gpu_limit,
	        voltage, capacity),

	TP_STRUCT__entry(
		__field(int, id)
		__field(int, throttle)
		__field(int, cpu0_limit)
		__field(int, cpu1_limit)
		__field(int, cpu2_limit)
		__field(int, tpu_limit)
		__field(int, gpu_limit)
		__field(int, voltage)
		__field(int, capacity)
	),

	TP_fast_assign(
		__entry->id = id;
		__entry->throttle = throttle;
		__entry->cpu0_limit = cpu0_limit;
		__entry->cpu1_limit = cpu1_limit;
		__entry->cpu2_limit = cpu2_limit;
		__entry->tpu_limit = tpu_limit;
		__entry->gpu_limit = gpu_limit;
		__entry->voltage = voltage;
		__entry->capacity = capacity;
	),

	TP_printk("bcl irq %d trig %d: cpu0=%d, cpu1=%d, cpu2=%d, tpu=%d, gpu=%d, volt=%d cap=%d",
		  __entry->id, __entry->throttle, __entry->cpu0_limit, __entry->cpu1_limit,
		  __entry->cpu2_limit, __entry->tpu_limit, __entry->gpu_limit,
		  __entry->voltage, __entry->capacity)
);

#endif /* _TRACE_BCL_EXYNOS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
