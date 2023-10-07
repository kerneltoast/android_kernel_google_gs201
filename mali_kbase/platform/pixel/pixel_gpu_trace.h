// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Google LLC.
 */

#ifndef _PIXEL_GPU_TRACE_H_
#define _PIXEL_GPU_TRACE_H_

#endif /* _PIXEL_GPU_TRACE_H_ */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mali

#if !defined(_TRACE_PIXEL_GPU_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_PIXEL_GPU_H

/* Linux includes */
#include <linux/tracepoint.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"

#define GPU_POWER_STATE_SYMBOLIC_STRINGS \
	{GPU_POWER_LEVEL_STACKS,	"STACKS"}, \
	{GPU_POWER_LEVEL_GLOBAL,	"GLOBAL"}, \
	{GPU_POWER_LEVEL_OFF,		"OFF"}

TRACE_EVENT(gpu_power_state,
	TP_PROTO(u64 change_ns, int from, int to),
	TP_ARGS(change_ns, from, to),
	TP_STRUCT__entry(
		__field(u64, change_ns)
		__field(int, from_state)
		__field(int, to_state)
	),
	TP_fast_assign(
		__entry->change_ns	= change_ns;
		__entry->from_state	= from;
		__entry->to_state	= to;
	),
	TP_printk("from=%s to=%s ns=%llu",
		__print_symbolic(__entry->from_state, GPU_POWER_STATE_SYMBOLIC_STRINGS),
		__print_symbolic(__entry->to_state, GPU_POWER_STATE_SYMBOLIC_STRINGS),
		__entry->change_ns
	)
);

#endif /* _TRACE_PIXEL_GPU_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef  TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE pixel_gpu_trace
#include <trace/define_trace.h>
