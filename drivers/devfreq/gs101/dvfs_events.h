/* SPDX-License-Identifier: GPL-2.0-only */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM devfreq

#if !defined(_DVFS_EVENTS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _DVFS_EVENTS_H

#include <linux/tracepoint.h>
#include <soc/google/exynos-devfreq.h>

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
TRACE_EVENT(dvfs_update_load,

	TP_PROTO(unsigned int freq, struct devfreq_alt_dvfs_data *alt_data),

	TP_ARGS(freq, alt_data),

	TP_STRUCT__entry(
		__field(unsigned int, freq)
		__field(unsigned long long, busy)
		__field(unsigned long long, total)
		__field(unsigned int, min_load)
		__field(unsigned int, max_load)
		__field(unsigned int, max_spent)
	),

	TP_fast_assign(
		__entry->freq = freq;
		__entry->busy = alt_data->busy;
		__entry->total = alt_data->total;
		__entry->min_load = alt_data->min_load;
		__entry->max_load = alt_data->max_load;
		__entry->max_spent = alt_data->max_spent;
	),

	TP_printk("freq=%u busy=%llu total=%llu min_load=%u max_load=%u max_spent=%u",
			__entry->freq, __entry->busy, __entry->total,
			__entry->min_load, __entry->max_load,
			__entry->max_spent)
);

TRACE_EVENT(dvfs_read_ppc,

	TP_PROTO(unsigned long long ccnt, unsigned long long pmcnt0,
		 unsigned long long pmcnt1, unsigned long pa_base),

	TP_ARGS(ccnt, pmcnt0, pmcnt1, pa_base),

	TP_STRUCT__entry(
		__field(unsigned long long, ccnt)
		__field(unsigned long long, pmcnt0)
		__field(unsigned long long, pmcnt1)
		__field(unsigned long, pa_base)
	),

	TP_fast_assign(
		__entry->ccnt = ccnt;
		__entry->pmcnt0 = pmcnt0;
		__entry->pmcnt1 = pmcnt1;
		__entry->pa_base = pa_base;
	),

	TP_printk("ppc_base=%lx ccnt=%llu pmcnt0=%llu pmcnt1=%llu",
		  __entry->pa_base, __entry->ccnt, __entry->pmcnt0,
		  __entry->pmcnt1)
);
#endif /* CONFIG_EXYNOS_ALT_DVFS  */

#endif /* _DVFS_EVENTS_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE dvfs_events
#include <trace/define_trace.h>
