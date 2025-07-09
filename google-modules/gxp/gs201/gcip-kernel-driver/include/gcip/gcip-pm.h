/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Power management support for GCIP devices.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GCIP_PM_H__
#define __GCIP_PM_H__

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

struct gcip_pm {
	struct device *dev;
	/* Worker to handle async power down retry. */
	struct delayed_work power_down_work;

	/* Lock to protect the members listed below. */
	struct mutex lock;
	/* Power up counter. Protected by @lock */
	int count;
	/* Flag indicating a deferred power down is pending. Protected by @lock */
	bool power_down_pending;
	/* The worker to asynchronously call gcip_pm_put(). */
	struct work_struct put_async_work;

	/* Callbacks. See struct gcip_pm_args. */
	void *data;
	int (*after_create)(void *data);
	void (*before_destroy)(void *data);
	int (*power_up)(void *data);
	int (*power_down)(void *data);
};

struct gcip_pm_args {
	/* Device struct for logging. */
	struct device *dev;

	/* Private data for the callbacks listed below. */
	void *data;
	/*
	 * Device-specific power up.
	 * Called with @pm->lock held and nesting is handled at generic layer.
	 * The IP driver may reject power on for such conditions as thermal suspend in this
	 * callback.
	 */
	int (*power_up)(void *data);
	/*
	 * Device-specific power down.
	 * Called with @pm->lock held and nesting is handled at generic layer.
	 * Returning -EAGAIN will trigger a retry after GCIP_ASYNC_POWER_DOWN_RETRY_DELAY ms.
	 */
	int (*power_down)(void *data);
	/* Optional. For initial setup after the interface initialized. */
	int (*after_create)(void *data);
	/* Optional. For clean-up before the interface is destroyed. */
	void (*before_destroy)(void *data);
};

/* Allocates and initializes a power management interface for the GCIP device. */
struct gcip_pm *gcip_pm_create(const struct gcip_pm_args *args);

/* Destroys and frees the power management interface. */
void gcip_pm_destroy(struct gcip_pm *pm);

/*
 * These mimic the pm_runtime_{get|put} functions to keep a reference count of requests in order to
 * keep the device up and turn it off.
 * Note that we don't keep track of system suspend/resume state since the system power management
 * will respect the parent-child sequencing to use a bottom-up order to suspend devices and a
 * top-down order to resume devices. No one would have the ability to acquire or release a wakelock
 * when the device is suspending or resuming.
 */

/*
 * Increases @pm->count if the device is already powered on.
 *
 * Caller should call gcip_pm_put() to decrease @pm->count if this function returns 0.
 * If @blocking is true, it will wait until the ongoing power state transition finishes (i.e.,
 * gcip_pm_{get,put,shutdown} called by other thread returns) and then check the power state.
 * If @blocking is false, return -EAGAIN immediately when there is a ongoing power state transition.
 *
 * Returns 0 on success; otherwise -EAGAIN if the device is off or in power state transition when
 * @blocking is false.
 */
int gcip_pm_get_if_powered(struct gcip_pm *pm, bool blocking);

/*
 * Increases @pm->count and powers up the device if previous @pm->count was zero.
 *
 * Returns 0 on success; otherwise negative error values.
 */
int gcip_pm_get(struct gcip_pm *pm);

/*
 * Decreases @pm->count and powers off the device if @pm->count reaches zero.
 * If .power_down fails, async work will be scheduled to retry after
 * GCIP_ASYNC_POWER_DOWN_RETRY_DELAY ms.
 */
void gcip_pm_put(struct gcip_pm *pm);

/* Schedules an asynchronous job to execute gcip_pm_put(). */
void gcip_pm_put_async(struct gcip_pm *pm);

/* Flushes the pending pm_put work if any. */
void gcip_pm_flush_put_work(struct gcip_pm *pm);

/* Gets the power up counter. Note that this is checked without PM lock. */
int gcip_pm_get_count(struct gcip_pm *pm);

/* Checks if device is already on. Note that this is checked without PM lock. */
bool gcip_pm_is_powered(struct gcip_pm *pm);

/* Shuts down the device if @pm->count equals to 0 or @force is true. */
void gcip_pm_shutdown(struct gcip_pm *pm, bool force);

/* Make sure @pm->lock is held. */
static inline void gcip_pm_lockdep_assert_held(struct gcip_pm *pm)
{
	if (!pm)
		return;

	lockdep_assert_held(&pm->lock);
}

/*
 * Lock the PM lock.
 * Since all the PM requests will be blocked until gcip_pm_unlock is called, one should use the
 * gcip_pm_{get,get_if_powered,put} if possible and uses this only if a power state transition can
 * not be triggered, e.g., in a workqueue that will be canceled during power off or crash handler.
 */
static inline void gcip_pm_lock(struct gcip_pm *pm)
{
	if (!pm)
		return;

	mutex_lock(&pm->lock);
}

/*
 * Lock the PM lock.
 * Same as gcip_pm_lock, but returns 1 if the lock has been acquired successfully, and 0 on
 * contention.
 */
static inline int gcip_pm_trylock(struct gcip_pm *pm)
{
	if (!pm)
		return 1;

	return mutex_trylock(&pm->lock);
}

/* Unlock the PM lock. */
static inline void gcip_pm_unlock(struct gcip_pm *pm)
{
	if (!pm)
		return;

	lockdep_assert_held(&pm->lock);
	mutex_unlock(&pm->lock);
}

#endif /* __GCIP_PM_H__ */
