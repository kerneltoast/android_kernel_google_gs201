// SPDX-License-Identifier: GPL-2.0
/*
 * Wakelock for the runtime to explicitly claim it's going to use the EdgeTPU
 * device.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-wakelock.h"

static const char *const event_name[] = {
#define X(name, _) #name
	EDGETPU_WAKELOCK_EVENTS
#undef X
};

/*
 * Loops through the events and warns if any event has a non-zero counter.
 * Returns true if at least one non-zero counter is found.
 *
 * Caller holds @wakelock->lock.
 */
static bool wakelock_warn_non_zero_event(struct edgetpu_wakelock *wakelock)
{
	int i;
	bool ret = false;

	for (i = 0; i < EDGETPU_WAKELOCK_EVENT_END; i++) {
		if (wakelock->event_count[i]) {
			ret = true;
			etdev_warn(wakelock->etdev,
				   "%s has non-zero counter=%d", event_name[i],
				   wakelock->event_count[i]);
		}
	}
	return ret;
}

struct edgetpu_wakelock *edgetpu_wakelock_alloc(struct edgetpu_dev *etdev)
{
#ifndef EDGETPU_HAS_WAKELOCK
	return EDGETPU_NO_WAKELOCK;
#else /* !EDGETPU_HAS_WAKELOCK */
	struct edgetpu_wakelock *wakelock =
		kzalloc(sizeof(*wakelock), GFP_KERNEL);

	if (!wakelock)
		return NULL;
	wakelock->etdev = etdev;
	mutex_init(&wakelock->lock);
	/* Initialize client wakelock state to "released" */
	wakelock->req_count = 0;
	return wakelock;
#endif /* EDGETPU_HAS_WAKELOCK */
}

void edgetpu_wakelock_free(struct edgetpu_wakelock *wakelock)
{
	if (IS_ERR_OR_NULL(wakelock))
		return;
	kfree(wakelock);
}

bool edgetpu_wakelock_inc_event_locked(struct edgetpu_wakelock *wakelock,
				       enum edgetpu_wakelock_event evt)
{
	bool ret = true;

	if (NO_WAKELOCK(wakelock))
		return true;
	if (!wakelock->req_count) {
		ret = false;
		etdev_warn(
			wakelock->etdev,
			"invalid increase event %d when wakelock is released",
			evt);
	} else {
		++wakelock->event_count[evt];
		/* integer overflow.. */
		if (unlikely(wakelock->event_count[evt] == 0)) {
			--wakelock->event_count[evt];
			ret = false;
			etdev_warn_once(wakelock->etdev,
					"int overflow on increasing event %d",
					evt);
		}
	}
	return ret;
}

bool edgetpu_wakelock_inc_event(struct edgetpu_wakelock *wakelock,
				enum edgetpu_wakelock_event evt)
{
	bool ret;

	if (NO_WAKELOCK(wakelock))
		return true;
	mutex_lock(&wakelock->lock);
	ret = edgetpu_wakelock_inc_event_locked(wakelock, evt);
	mutex_unlock(&wakelock->lock);
	return ret;
}

bool edgetpu_wakelock_dec_event_locked(struct edgetpu_wakelock *wakelock,
				       enum edgetpu_wakelock_event evt)
{
	bool ret = true;

	if (NO_WAKELOCK(wakelock))
		return true;
	if (!wakelock->event_count[evt]) {
		ret = false;
		etdev_warn(wakelock->etdev, "event %d unbalanced decreasing",
			   evt);
	} else {
		--wakelock->event_count[evt];
	}
	return ret;
}

bool edgetpu_wakelock_dec_event(struct edgetpu_wakelock *wakelock,
				enum edgetpu_wakelock_event evt)
{
	bool ret;

	if (NO_WAKELOCK(wakelock))
		return true;
	mutex_lock(&wakelock->lock);
	ret = edgetpu_wakelock_dec_event_locked(wakelock, evt);
	mutex_unlock(&wakelock->lock);
	return ret;
}

uint edgetpu_wakelock_lock(struct edgetpu_wakelock *wakelock)
{
	if (NO_WAKELOCK(wakelock))
		return 1;
	mutex_lock(&wakelock->lock);
	return wakelock->req_count;
}

void edgetpu_wakelock_unlock(struct edgetpu_wakelock *wakelock)
{
	if (!NO_WAKELOCK(wakelock))
		mutex_unlock(&wakelock->lock);
}

int edgetpu_wakelock_acquire(struct edgetpu_wakelock *wakelock)
{
	int ret;

	if (NO_WAKELOCK(wakelock))
		return 1;
	ret = wakelock->req_count++;
	/* integer overflow */
	if (unlikely(ret < 0)) {
		wakelock->req_count--;
		return -EOVERFLOW;
	}
	return ret;
}

int edgetpu_wakelock_release(struct edgetpu_wakelock *wakelock)
{
	if (NO_WAKELOCK(wakelock))
		return 1;
	if (!wakelock->req_count) {
		etdev_warn(wakelock->etdev, "invalid wakelock release");
		return -EINVAL;
	}
	/* only need to check events when this is the last reference */
	if (wakelock->req_count == 1 &&
	    wakelock_warn_non_zero_event(wakelock)) {
		etdev_warn(
			wakelock->etdev,
			"detected non-zero events, refusing wakelock release");
		return -EAGAIN;
	}

	return --wakelock->req_count;
}
