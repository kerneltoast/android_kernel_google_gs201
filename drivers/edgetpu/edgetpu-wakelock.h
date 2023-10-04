/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Wakelock for the runtime to explicitly claim it's going to use the EdgeTPU
 * device.
 *
 * Copyright (C) 2021 Google, Inc.
 */
#ifndef __EDGETPU_WAKELOCK_H__
#define __EDGETPU_WAKELOCK_H__

#include <linux/err.h>
#include <linux/mutex.h>

#include "edgetpu-internal.h"

/*
 * See edgetpu_wakelock_alloc() for when this value is returned.
 * Define as an errno so we can use IS_ERR* macros.
 */
#define EDGETPU_NO_WAKELOCK ERR_PTR(-EOPNOTSUPP)

#define NO_WAKELOCK(wakelock) ((wakelock) == EDGETPU_NO_WAKELOCK)

/*
 * Events that could block the wakelock from being released.
 * Use inc_event() and dec_event() to increase and decrease the counters when
 * the event happens.
 *
 * Defined with X macros to support fetching event names from values.
 */
#define EDGETPU_WAKELOCK_EVENTS                                                \
	X(EDGETPU_WAKELOCK_EVENT_FULL_CSR, 0),                                 \
	X(EDGETPU_WAKELOCK_EVENT_MBOX_CSR, 1),                                 \
	X(EDGETPU_WAKELOCK_EVENT_CMD_QUEUE, 2),                                \
	X(EDGETPU_WAKELOCK_EVENT_RESP_QUEUE, 3),                               \
	X(EDGETPU_WAKELOCK_EVENT_EXT_MAILBOX, 4),                              \
	X(EDGETPU_WAKELOCK_EVENT_END, 5)

enum edgetpu_wakelock_event {
#define X(name, val) name = val
	EDGETPU_WAKELOCK_EVENTS
#undef X
};

struct edgetpu_wakelock {
	struct edgetpu_dev *etdev; /* only for logging */
	/* Protects every field below */
	struct mutex lock;
	/*
	 * The request counter, increments on "acquire" and decrements on
	 * "release".
	 */
	uint req_count;
	/*
	 * Events counter.
	 * release() would fail if one of the slots is not zero.
	 */
	uint event_count[EDGETPU_WAKELOCK_EVENT_END];
};

/*
 * Allocates and initializes a wakelock object.
 *
 * Returns the pointer on success, or NULL when out of memory.
 *
 * When the chipset doesn't support wakelock (judged by EDGETPU_HAS_WAKELOCK):
 *   Nothing is allocated and this function returns EDGETPU_NO_WAKELOCK.
 */
struct edgetpu_wakelock *edgetpu_wakelock_alloc(struct edgetpu_dev *etdev);
/* Frees the allocated wakelock. */
void edgetpu_wakelock_free(struct edgetpu_wakelock *wakelock);

/*
 * Increases the event counter of @evt by one.
 *
 * Returns true if the counter is increased successfully.
 * Returns false when one of the following errors happens:
 *   - the wakelock is released
 *   - integer overflow on the counter
 *
 * When the chipset doesn't support wakelock:
 *   Does nothing and returns true.
 */
bool edgetpu_wakelock_inc_event(struct edgetpu_wakelock *wakelock,
				enum edgetpu_wakelock_event evt);

/*
 * A version of the above where the caller holds the wakelock internal lock
 * by calling edgetpu_wakelock_lock.
 */
bool edgetpu_wakelock_inc_event_locked(struct edgetpu_wakelock *wakelock,
				       enum edgetpu_wakelock_event evt);
/*
 * Decreases the event counter of @evt by one.
 *
 * Returns true if the counter is decreased successfully.
 * Returns false when one of the following errors happens:
 *   - the counter is zero
 *
 * When the chipset doesn't support wakelock:
 *   Does nothing and returns true.
 */
bool edgetpu_wakelock_dec_event(struct edgetpu_wakelock *wakelock,
				enum edgetpu_wakelock_event evt);

/*
 * A version of the above where the caller holds the wakelock internal lock
 * by calling edgetpu_wakelock_lock.
 */
bool edgetpu_wakelock_dec_event_locked(struct edgetpu_wakelock *wakelock,
				       enum edgetpu_wakelock_event evt);

/*
 * Holds the internal lock of @wakelock. Fields in @wakelock are protected when
 * this lock is holding.
 *
 * Returns the non-negative request counter of @wakelock.
 *
 * Example:
 *   if (edgetpu_wakelock_lock(wakelock)) {
 *      <..works that need the state of wakelock unchanged..>
 *   }
 *   edgetpu_wakelock_unlock(wakelock);
 *
 * When the chipset doesn't support wakelock:
 *   Does nothing and returns 1.
 */
uint edgetpu_wakelock_lock(struct edgetpu_wakelock *wakelock);
void edgetpu_wakelock_unlock(struct edgetpu_wakelock *wakelock);

/*
 * Returns the request counter of @wakelock.
 *
 * Caller calls edgetpu_wakelock_lock() before calling this function.
 *
 * When the chipset doesn't support wakelock:
 *   Returns 1.
 */
static inline uint
edgetpu_wakelock_count_locked(struct edgetpu_wakelock *wakelock)
{
	return NO_WAKELOCK(wakelock) ? 1 : wakelock->req_count;
}

/*
 * Acquires the wakelock, increases @wakelock->req_count by one.
 *
 * This function should be surrounded by edgetpu_wakelock_lock() and
 * edgetpu_wakelock_unlock().
 *
 * Returns the value of request counter *before* being increased.
 * Returns -EOVERFLOW if the request counter would overflow after increment.
 *
 * When the chipset doesn't support wakelock:
 *   Does nothing and returns 1.
 */
int edgetpu_wakelock_acquire(struct edgetpu_wakelock *wakelock);
/*
 * Requests to release the wakelock, decreases @wakelock->req_count by one on
 * success.
 *
 * This function should be surrounded by edgetpu_wakelock_lock() and
 * edgetpu_wakelock_unlock().
 *
 * Returns the value of request counter *after* being decreased.
 * Returns -EINVAL if the request counter is already zero.
 * Returns -EAGAIN when there are events blocking wakelock from being released.
 *
 * When the chipset doesn't support wakelock:
 *   Does nothing and returns 1.
 */
int edgetpu_wakelock_release(struct edgetpu_wakelock *wakelock);

#endif /* __EDGETPU_WAKELOCK_H__ */
