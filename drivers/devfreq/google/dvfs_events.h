/* SPDX-License-Identifier: GPL-2.0-only */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM devfreq

#if !defined(_DVFS_EVENTS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _DVFS_EVENTS_H

#include <linux/tracepoint.h>
#include <soc/google/exynos-devfreq.h>

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
TRACE_EVENT(dvfs_update_load,

	TP_PROTO(unsigned int mif_freq, struct devfreq_alt_dvfs_data *alt_data,
		 unsigned int idx, unsigned int int_freq),

	TP_ARGS(mif_freq, alt_data, idx, int_freq),

	TP_STRUCT__entry(
		__field(unsigned int, mif_freq)
		__field(unsigned long long, busy)
		__field(unsigned long long, total)
		__field(unsigned int, min_load)
		__field(unsigned int, max_load)
		__field(unsigned int, max_spent)
		__field(unsigned int, idx)
		__field(unsigned int, int_freq)
	),

	TP_fast_assign(
		__entry->mif_freq = mif_freq;
		__entry->busy = alt_data->track[idx].busy;
		__entry->total = alt_data->track[idx].total;
		__entry->min_load = alt_data->track[idx].min_load;
		__entry->max_load = alt_data->track[idx].max_load;
		__entry->max_spent = alt_data->track[idx].max_spent;
		__entry->idx = idx;
		__entry->int_freq = int_freq;
	),

	TP_printk("mif_freq=%u busy=%llu total=%llu min_load=%u max_load=%u max_spent=%u idx=%u int_freq=%u",
			__entry->mif_freq, __entry->busy, __entry->total,
			__entry->min_load, __entry->max_load,
			__entry->max_spent, __entry->idx,
			__entry->int_freq)
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
