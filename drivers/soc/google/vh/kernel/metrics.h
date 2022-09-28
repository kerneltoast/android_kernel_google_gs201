/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support for Perf metrics
 *
 * Copyright 2022 Google LLC
 */

#include <linux/interrupt.h>

#define RESUME_LATENCY_STEP_SMALL 10
#define RESUME_LATENCY_STEP_MID 50
#define RESUME_LATENCY_STEP_LARGE 100

#define RESUME_LATENCY_BOUND_SMALL 250
#define RESUME_LATENCY_BOUND_MID 500
#define RESUME_LATENCY_BOUND_MAX 1000

#define LATENCY_CNT_SMALL (RESUME_LATENCY_BOUND_SMALL / RESUME_LATENCY_STEP_SMALL)
#define LATENCY_CNT_MID ((RESUME_LATENCY_BOUND_MID - RESUME_LATENCY_BOUND_SMALL) / \
	RESUME_LATENCY_STEP_MID)
#define LATENCY_CNT_LARGE ((RESUME_LATENCY_BOUND_MAX - RESUME_LATENCY_BOUND_MID) / \
	RESUME_LATENCY_STEP_LARGE)
#define RESUME_LATENCY_ARR_SIZE (LATENCY_CNT_SMALL + LATENCY_CNT_MID + LATENCY_CNT_LARGE + 1)

struct  resume_latency {
	ktime_t resume_start;
	ktime_t resume_end;
	spinlock_t resume_latency_stat_lock;
	s64 resume_count[RESUME_LATENCY_ARR_SIZE];
	s64 resume_latency_max_ms;
	u64 resume_latency_sum_ms;
};
