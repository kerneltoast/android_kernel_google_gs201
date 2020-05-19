/* SPDX-License-Identifier: GPL-2.0-only */
//
// IO performance mode with UFS
//
// Copyright (C) 2019 Samsung Electronics Co., Ltd.
//
// Authors:
//	Kiwoong <kwmad.kim@samsung.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
#undef TRACE_SYSTEM
#define TRACE_SYSTEM ufs_exynos_perf

#if !defined(_TRACE_UFS_EXYNOS_PERF_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_UFS_EXYNOS_PERF_H

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(ufs_exynos_perf_one_int,
		    TP_PROTO(int i),
		    TP_ARGS(i),
		    TP_STRUCT__entry(__field(int, i)),
		    TP_fast_assign(__entry->i = i;),
		    TP_printk("%d", __entry->i)
);

DECLARE_EVENT_CLASS(ufs_exynos_perf_three_int,
		    TP_PROTO(int i, int j, int k),
		    TP_ARGS(i, j, k),
		    TP_STRUCT__entry(__field(int,	i)
				     __field(int,	j)
				     __field(int,	k)
		    ),
		    TP_fast_assign(__entry->i = i;
				   __entry->j = j;
				   __entry->k = k;
		    ),
		    TP_printk("%d, %d, %d", __entry->i, __entry->j, __entry->k)
);

TRACE_EVENT(ufs_exynos_perf_stat,
	    TP_PROTO(bool is_cp_time, unsigned long long diff,
		     unsigned long period,
		     long long cp_time, long long cur_time,
		     unsigned long count, unsigned long th_count,
		     int locked, int active),

	    TP_ARGS(is_cp_time, diff, period, cp_time, cur_time,
		    count, th_count, locked, active),

	    TP_STRUCT__entry(__field(bool,	is_cp_time)
			     __field(unsigned long long,	diff)
			     __field(unsigned long,	period)
			     __field(long long,	cp_time)
			     __field(long long,	cur_time)
			     __field(unsigned long,	count)
			     __field(unsigned long,	th_count)
			     __field(int,	locked)
			     __field(int,	active)
	    ),

	    TP_fast_assign(__entry->is_cp_time = is_cp_time;
			   __entry->diff	   = diff;
			   __entry->period    = period;
			   __entry->cp_time   = cp_time;
			   __entry->cur_time  = cur_time;
			   __entry->count     = count;
			   __entry->th_count  = th_count;
			   __entry->locked    = locked;
			   __entry->active    = active;
	    ),

	    TP_printk("%d: %llu (%lu, %lld, %lld) (%lu, %lu) | %d, %d,",
		      __entry->is_cp_time,
		      __entry->diff,
		      __entry->period,
		      __entry->cp_time,
		      __entry->cur_time,
		      __entry->count,
		      __entry->th_count,
		      __entry->locked,
		      __entry->active)
	    );

	    DEFINE_EVENT(ufs_exynos_perf_one_int, ufs_exynos_perf_lock,
			 TP_PROTO(int i),
			 TP_ARGS(i)
	    );

	    DEFINE_EVENT(ufs_exynos_perf_three_int, ufs_exynos_perf_issue,
			 TP_PROTO(int is_big, int op, int len),
			 TP_ARGS(is_big, op, len)
	    );

#endif /* _TRACE_UFS_EXYNOS_PERF_H */

		/* This part must be outside protection */
#include <trace/define_trace.h>
