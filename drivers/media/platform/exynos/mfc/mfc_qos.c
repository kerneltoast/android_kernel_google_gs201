/*
 * drivers/media/platform/exynos/mfc/mfc_qos.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/sort.h>

#include "mfc_qos.h"

#define COL_FRAME_RATE		0
#define COL_FRAME_INTERVAL	1

#define MFC_MAX_INTERVAL	(2 * USEC_PER_SEC)

/*
 * A framerate table determines framerate by the interval(us) of each frame.
 * Framerate is not accurate, just rough value to seperate overload section.
 * Base line of each section are selected from middle value.
 * 25fps(40000us), 40fps(25000us), 80fps(12500us)
 * 144fps(6940us), 205fps(4860us), 320fps(3125us)
 *
 * interval(us) | 0         3125          4860          6940          12500         25000        40000
 * framerate    |    480fps   |    240fps   |    180fps   |    120fps   |    60fps    |    30fps   |	24fps
 */
static unsigned long framerate_table[][2] = {
	{  24000, 40000 },
	{  30000, 25000 },
	{  60000, 12500 },
	{ 120000,  6940 },
	{ 180000,  4860 },
	{ 240000,  3125 },
	{ 480000,     0 },
};

static inline unsigned long __mfc_qos_timespec64_diff(struct timespec64 *to,
					struct timespec64 *from)
{
	unsigned long interval_nsec = (to->tv_sec * NSEC_PER_SEC + to->tv_nsec)
		- (from->tv_sec * NSEC_PER_SEC + from->tv_nsec);

	if (interval_nsec <= 0)
		interval_nsec = MFC_MAX_INTERVAL * 1000;

	return interval_nsec / 1000;
}

static int __mfc_qos_ts_sort(const void *p0, const void *p1)
{
	const int *t0 = p0, *t1 = p1;

	/* ascending sort */
	if (*t0 < *t1)
		return -1;
	else if (*t0 > *t1)
		return 1;

	return 0;
}

static int __mfc_qos_get_ts_interval(struct mfc_ctx *ctx)
{
	int tmp[MAX_TIME_INDEX];
	int n, i, min;

	n = ctx->ts_is_full ? MAX_TIME_INDEX : ctx->ts_count;

	memcpy(&tmp[0], &ctx->ts_interval_array[0], n * sizeof(int));
	sort(tmp, n, sizeof(int), __mfc_qos_ts_sort, NULL);

	/* apply median filter for selecting ts interval */
	min = (n <= 2) ? tmp[0] : tmp[n / 2];

	if (debug_ts == 1) {
		mfc_ctx_info("==============[TS] interval (sort)==============\n");
		for (i = 0; i < n; i++)
			mfc_ctx_info("[TS] interval [%d] = %d\n", i, tmp[i]);
		mfc_ctx_info("[TS] get interval %d\n", min);
	}

	return min;
}

static unsigned long __mfc_qos_get_framerate_by_interval(int interval)
{
	unsigned long i;

	/* if the interval is too big (2sec), framerate set to 0 */
	if (interval > MFC_MAX_INTERVAL || interval < 0)
		return 0;

	for (i = 0; i < ARRAY_SIZE(framerate_table); i++) {
		if (interval > framerate_table[i][COL_FRAME_INTERVAL])
			return framerate_table[i][COL_FRAME_RATE];
	}

	return 0;
}

static int __mfc_qos_get_bps_section_by_bps(struct mfc_dev *dev, int Kbps)
{
	int i;

	if (Kbps > dev->pdata->max_Kbps[0]) {
		mfc_dev_debug(4, "[QoS] overspec bps %d > %d\n",
				Kbps, dev->pdata->max_Kbps[0]);
		return dev->pdata->num_mfc_freq - 1;
	}

	for (i = 0; i < dev->pdata->num_mfc_freq; i++) {
		if (Kbps <= dev->bitrate_table[i].bps_interval) {
			mfc_dev_debug(3, "[QoS] MFC freq lv%d, %dKHz is needed\n",
					i, dev->bitrate_table[i].mfc_freq);
			return i;
		}
	}

	return 0;
}

/* Return the minimum interval between previous and next entry */
static int __mfc_qos_get_interval(struct list_head *head, struct list_head *entry)
{
	unsigned long prev_interval = MFC_MAX_INTERVAL, next_interval = MFC_MAX_INTERVAL;
	struct mfc_timestamp *prev_ts, *next_ts, *curr_ts;
	int ret = 0;

	curr_ts = list_entry(entry, struct mfc_timestamp, list);

	if (entry->prev != head) {
		prev_ts = list_entry(entry->prev, struct mfc_timestamp, list);
		prev_interval = __mfc_qos_timespec64_diff(&curr_ts->timestamp,
							&prev_ts->timestamp);
		if (prev_interval > MFC_MAX_INTERVAL)
			prev_interval = MFC_MAX_INTERVAL;
	}

	if (entry->next != head) {
		next_ts = list_entry(entry->next, struct mfc_timestamp, list);
		next_interval = __mfc_qos_timespec64_diff(&next_ts->timestamp,
							&curr_ts->timestamp);
		if (next_interval > MFC_MAX_INTERVAL)
			next_interval = MFC_MAX_INTERVAL;
	}

	ret = (prev_interval < next_interval) ? prev_interval : next_interval;
	return ret;
}

static int __mfc_qos_add_timestamp(struct mfc_ctx *ctx,
			struct timespec64 *time, struct list_head *head)
{
	int replace_entry = 0;
	struct mfc_timestamp *curr_ts = &ctx->ts_array[ctx->ts_count];

	if (ctx->ts_is_full) {
		/* Replace the entry if list of array[ts_count] is same as entry */
		if (&curr_ts->list == head)
			replace_entry = 1;
		else
			list_del(&curr_ts->list);
	}

	memcpy(&curr_ts->timestamp, time, sizeof(*time));
	if (!replace_entry)
		list_add(&curr_ts->list, head);
	curr_ts->interval =
		__mfc_qos_get_interval(&ctx->ts_list, &curr_ts->list);
	curr_ts->index = ctx->ts_count;

	ctx->ts_interval_array[ctx->ts_count] = curr_ts->interval;
	ctx->ts_count++;

	if (ctx->ts_count == MAX_TIME_INDEX) {
		ctx->ts_is_full = 1;
		ctx->ts_count %= MAX_TIME_INDEX;
	}

	return 0;
}

static unsigned long __mfc_qos_get_fps_by_timestamp(struct mfc_ctx *ctx, struct timespec64 *time)
{
	struct list_head *head = &ctx->ts_list;
	struct mfc_timestamp *temp_ts;
	int found;
	int index = 0;
	int min_interval = MFC_MAX_INTERVAL;
	int time_diff;
	unsigned long max_framerate;

	if (IS_BUFFER_BATCH_MODE(ctx)) {
		if (debug_ts == 1)
			mfc_ctx_info("[BUFCON][TS] Keep framerate if buffer batch mode is used, %ldfps\n",
					ctx->framerate);
		return ctx->framerate;
	}

	if (list_empty(&ctx->ts_list)) {
		__mfc_qos_add_timestamp(ctx, time, &ctx->ts_list);
		return __mfc_qos_get_framerate_by_interval(0);
	} else {
		found = 0;
		list_for_each_entry_reverse(temp_ts, &ctx->ts_list, list) {
			time_diff = __mfc_timespec64_compare(time, &temp_ts->timestamp);
			if (time_diff == 0) {
				/* Do not add if same timestamp already exists */
				found = 1;
				break;
			} else if (time_diff > 0) {
				/* Add this after temp_ts */
				__mfc_qos_add_timestamp(ctx, time, &temp_ts->list);
				found = 1;
				break;
			}
		}

		if (!found)	/* Add this at first entry */
			__mfc_qos_add_timestamp(ctx, time, &ctx->ts_list);
	}

	min_interval = __mfc_qos_get_ts_interval(ctx);
	max_framerate = __mfc_qos_get_framerate_by_interval(min_interval);

	if (debug_ts == 1) {
		/* Debug info */
		mfc_ctx_info("===================[TS]===================\n");
		mfc_ctx_info("[TS] New timestamp = %lld.%09ld, count = %d\n",
			time->tv_sec, time->tv_nsec, ctx->ts_count);

		index = 0;
		list_for_each_entry(temp_ts, &ctx->ts_list, list) {
			mfc_ctx_info("[TS] [%d] timestamp [i:%d]: %lld.%09ld\n",
					index, temp_ts->index,
					temp_ts->timestamp.tv_sec,
					temp_ts->timestamp.tv_nsec);
			index++;
		}
		mfc_ctx_info("[TS] Min interval = %d, It is %ld fps\n",
				min_interval, max_framerate);
	}

	/* Calculation the last frame fps for drop control */
	if (ctx->type == MFCINST_ENCODER) {
		temp_ts = list_entry(head->prev, struct mfc_timestamp, list);
		if (temp_ts->interval > USEC_PER_SEC) {
			mfc_ctx_info("[TS] ts interval(%d) couldn't over 1sec(1fps)\n",
					temp_ts->interval);
			ctx->ts_last_interval = 0;
		} else {
			ctx->ts_last_interval = temp_ts->interval;
		}
	}

	if (!ctx->ts_is_full) {
		if (debug_ts == 1)
			mfc_ctx_info("[TS] ts doesn't full, keep %ld fps\n", ctx->framerate);
		return ctx->framerate;
	}

	if (IS_MULTI_MODE(ctx))
		max_framerate = max_framerate / ctx->dev->num_core;

	return max_framerate;
}

static int __mfc_qos_get_bps_section(struct mfc_ctx *ctx, u32 bytesused)
{
	struct mfc_dev *dev = ctx->dev;
	struct list_head *head = &ctx->bitrate_list;
	struct mfc_bitrate *temp_bitrate;
	struct mfc_bitrate *new_bitrate = &ctx->bitrate_array[ctx->bitrate_index];
	unsigned long sum_size = 0, avg_Kbits;
	int count = 0, bps_section = 0;

	if (ctx->bitrate_is_full) {
		temp_bitrate = list_entry(head->next, struct mfc_bitrate, list);
		list_del(&temp_bitrate->list);
	}

	new_bitrate->bytesused = bytesused;
	list_add_tail(&new_bitrate->list, head);

	list_for_each_entry(temp_bitrate, head, list) {
		mfc_debug(4, "[QoS][%d] strm_size %d\n", count, temp_bitrate->bytesused);
		sum_size += temp_bitrate->bytesused;
		count++;
	}

	avg_Kbits = ((sum_size * BITS_PER_BYTE) / count) / 1024;
	ctx->Kbps = (int)(avg_Kbits * (ctx->last_framerate / 1000));
	/* Standardization to high bitrate spec */
	if (!CODEC_HIGH_PERF(ctx))
		ctx->Kbps = dev->bps_ratio * ctx->Kbps;
	mfc_debug(3, "[QoS] %d Kbps, average %ld Kbits per frame\n", ctx->Kbps, avg_Kbits);

	ctx->bitrate_index++;
	if (ctx->bitrate_index == MAX_TIME_INDEX) {
		ctx->bitrate_is_full = 1;
		ctx->bitrate_index %= MAX_TIME_INDEX;
	}

	/*
	 * When there is a value of ts_is_full,
	 * we can trust fps(trusted fps calculated by timestamp diff).
	 * When fps information becomes reliable,
	 * we will start QoS handling by obtaining bps section.
	 */
	if (ctx->ts_is_full)
		bps_section = __mfc_qos_get_bps_section_by_bps(dev, ctx->Kbps);

	return bps_section;
}

void mfc_qos_update_framerate(struct mfc_ctx *ctx, u32 bytesused)
{
	int bps_section;

	/* 1) bitrate is updated */
	bps_section = __mfc_qos_get_bps_section(ctx, bytesused);
	if (ctx->last_bps_section != bps_section) {
		mfc_debug(2, "[QoS] bps section changed: %d -> %d\n",
				ctx->last_bps_section, bps_section);
		ctx->last_bps_section = bps_section;
		ctx->update_bitrate = true;
	}

	/* 2) There is operating framerate */
	if (ctx->operating_framerate) {
		if ((ctx->ts_is_full && (ctx->operating_framerate != ctx->framerate)) ||
			(!ctx->ts_is_full && (ctx->operating_framerate > ctx->framerate))) {
			mfc_debug(2, "[QoS] operating fps changed: %ld -> %ld\n",
					ctx->framerate, ctx->operating_framerate);
			ctx->framerate = ctx->operating_framerate;
			ctx->update_framerate = true;
		}
		return;
	}

	/* 3) check non-real-time */
	if (ctx->rt == MFC_NON_RT) {
		if (ctx->framerate < DEC_DEFAULT_FPS) {
			mfc_debug(2, "[QoS][PRIO] non real time fps changed: %ld -> %ld\n",
					ctx->framerate, (long)DEC_DEFAULT_FPS);
			ctx->framerate = DEC_DEFAULT_FPS;
			ctx->update_framerate = true;
		}
		return;
	}

	/* 4) framerate is updated */
	if (ctx->last_framerate != 0 && ctx->last_framerate != ctx->framerate) {
		mfc_debug(2, "[QoS] fps changed: %ld -> %ld, qos ratio: %d\n",
				ctx->framerate, ctx->last_framerate, ctx->qos_ratio);
		ctx->framerate = ctx->last_framerate;
		ctx->update_framerate = true;
	}
}

void mfc_qos_update_last_framerate(struct mfc_ctx *ctx, u64 timestamp)
{
	struct timespec64 time;

	time.tv_sec = timestamp / NSEC_PER_SEC;
	time.tv_nsec = (timestamp - (time.tv_sec * NSEC_PER_SEC));

	ctx->last_framerate = __mfc_qos_get_fps_by_timestamp(ctx, &time);
	if (ctx->last_framerate > MFC_MAX_FPS)
		ctx->last_framerate = MFC_MAX_FPS;
	ctx->last_framerate = (ctx->qos_ratio * ctx->last_framerate) / 100;
}
