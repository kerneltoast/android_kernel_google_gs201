/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * Authors: Daeho Jeong <daehojeong@google.com>
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ufs_pixel

#if !defined(_TRACE_UFS_PIXEL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_UFS_PIXEL_H

#include <linux/tracepoint.h>

TRACE_EVENT(ufs_stats,
	TP_PROTO(struct pixel_ufs *ufs, u64 *avg_time),

	TP_ARGS(ufs, avg_time),

	TP_STRUCT__entry(
		__field(u64,	peak_read)
		__field(u64,	peak_write)
		__field(u64,	peak_flush)
		__field(u64,	peak_discard)
		__field(u64,	peak_qdepth)
		__field(u64,	avg_read)
		__field(u64,	avg_write)
		__field(u64,	avg_flush)
		__field(u64,	avg_discard)
		__field(u64,	r_rc_s)
		__field(u64,	r_tb_s)
		__field(u64,	w_rc_s)
		__field(u64,	w_tb_s)
		__field(u64,	r_rc_c)
		__field(u64,	r_tb_c)
		__field(u64,	w_rc_c)
		__field(u64,	w_tb_c)
		__field(u64,	r_rem)
		__field(u64,	w_rem)
	),

	TP_fast_assign(
		__entry->peak_read	= ufs->peak_reqs[REQ_TYPE_READ];
		__entry->peak_write	= ufs->peak_reqs[REQ_TYPE_WRITE];
		__entry->peak_flush	= ufs->peak_reqs[REQ_TYPE_FLUSH];
		__entry->peak_discard	= ufs->peak_reqs[REQ_TYPE_DISCARD];
		__entry->peak_qdepth	= ufs->peak_queue_depth;
		__entry->avg_read	= avg_time[REQ_TYPE_READ];
		__entry->avg_write	= avg_time[REQ_TYPE_WRITE];
		__entry->avg_flush	= avg_time[REQ_TYPE_FLUSH];
		__entry->avg_discard	= avg_time[REQ_TYPE_DISCARD];
		__entry->r_rc_s	= ufs->curr_io_stats.r_req_count_started
				- ufs->prev_io_stats.r_req_count_started;
		__entry->r_tb_s	= ufs->curr_io_stats.r_total_bytes_started
				- ufs->prev_io_stats.r_total_bytes_started;
		__entry->w_rc_s	= ufs->curr_io_stats.w_req_count_started
				- ufs->prev_io_stats.w_req_count_started;
		__entry->w_tb_s	= ufs->curr_io_stats.w_total_bytes_started
				- ufs->prev_io_stats.w_total_bytes_started;
		__entry->r_rc_c	= ufs->curr_io_stats.r_req_count_completed
				- ufs->prev_io_stats.r_req_count_completed;
		__entry->r_tb_c	= ufs->curr_io_stats.r_total_bytes_completed
				- ufs->prev_io_stats.r_total_bytes_completed;
		__entry->w_rc_c	= ufs->curr_io_stats.w_req_count_completed
				- ufs->prev_io_stats.w_req_count_completed;
		__entry->w_tb_c	= ufs->curr_io_stats.w_total_bytes_completed
				- ufs->prev_io_stats.w_total_bytes_completed;
		__entry->r_rem	= ufs->curr_io_stats.r_req_count_started
				- ufs->curr_io_stats.r_req_count_completed;
		__entry->w_rem	= ufs->curr_io_stats.w_req_count_started
				- ufs->curr_io_stats.w_req_count_completed;
	),

	TP_printk(
		"avg/max(us): read(%llu/%llu) write(%llu/%llu) "
		"flush(%llu/%llu) discard(%llu/%llu), "
		"started_bytes/count: read(%llu/%llu) write(%llu/%llu), "
		"completed_bytes/count: read(%llu/%llu) write(%llu/%llu), "
		"in-flight_read/write: %llu/%llu, peak_queue_depth: %llu",
		__entry->avg_read, __entry->peak_read,
		__entry->avg_write, __entry->peak_write,
		__entry->avg_flush, __entry->peak_flush,
		__entry->avg_discard, __entry->peak_discard,
		__entry->r_tb_s, __entry->r_rc_s,
		__entry->w_tb_s, __entry->w_rc_s,
		__entry->r_tb_c, __entry->r_rc_c,
		__entry->w_tb_c, __entry->w_rc_c,
		__entry->r_rem, __entry->w_rem, __entry->peak_qdepth
	)
);
#endif /* if !defined(_TRACE_UFS_PIXEL_H) || defined(TRACE_HEADER_MULTI_READ) */

/* This part must be outside protection */
#include <trace/define_trace.h>
