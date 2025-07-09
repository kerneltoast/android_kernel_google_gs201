// SPDX-License-Identifier: GPL-2.0-only
/*
 * Power management interface for GCIP devices.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <gcip/gcip-pm.h>

#define GCIP_ASYNC_POWER_DOWN_RETRY_DELAY 200 /* ms */

/* Caller must hold @pm->lock. */
static void gcip_pm_try_power_down(struct gcip_pm *pm)
{
	int ret;

	gcip_pm_lockdep_assert_held(pm);

	ret = pm->power_down(pm->data);

	if (ret == -EAGAIN) {
		dev_warn(pm->dev, "Power down request denied, retrying in %d ms\n",
			 GCIP_ASYNC_POWER_DOWN_RETRY_DELAY);
		pm->power_down_pending = true;
		schedule_delayed_work(&pm->power_down_work,
				      msecs_to_jiffies(GCIP_ASYNC_POWER_DOWN_RETRY_DELAY));
	} else {
		if (ret)
			dev_err(pm->dev, "Power down request failed (%d)\n", ret);
		pm->power_down_pending = false;
	}
}

/* Worker for async power down. */
static void gcip_pm_async_power_down_work(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct gcip_pm *pm = container_of(dwork, struct gcip_pm, power_down_work);

	mutex_lock(&pm->lock);

	if (pm->power_down_pending)
		gcip_pm_try_power_down(pm);
	else
		dev_info(pm->dev, "Delayed power down cancelled\n");

	mutex_unlock(&pm->lock);
}

/* Worker for async gcip_pm_put(). */
static void gcip_pm_async_put_work(struct work_struct *work)
{
	struct gcip_pm *pm = container_of(work, struct gcip_pm, put_async_work);

	gcip_pm_put(pm);
}

struct gcip_pm *gcip_pm_create(const struct gcip_pm_args *args)
{
	struct gcip_pm *pm;
	int ret;

	if (!args->dev || !args->power_up || !args->power_down)
		return ERR_PTR(-EINVAL);

	pm = devm_kzalloc(args->dev, sizeof(*pm), GFP_KERNEL);
	if (!pm)
		return ERR_PTR(-ENOMEM);

	pm->dev = args->dev;
	pm->data = args->data;
	pm->after_create = args->after_create;
	pm->before_destroy = args->before_destroy;
	pm->power_up = args->power_up;
	pm->power_down = args->power_down;

	mutex_init(&pm->lock);
	INIT_DELAYED_WORK(&pm->power_down_work, gcip_pm_async_power_down_work);
	INIT_WORK(&pm->put_async_work, gcip_pm_async_put_work);

	if (pm->after_create) {
		ret = pm->after_create(pm->data);
		if (ret) {
			devm_kfree(args->dev, pm);
			return ERR_PTR(ret);
		}
	}

	return pm;
}

void gcip_pm_destroy(struct gcip_pm *pm)
{
	if (!pm)
		return;

	gcip_pm_flush_put_work(pm);

	pm->power_down_pending = false;
	cancel_delayed_work_sync(&pm->power_down_work);

	if (pm->before_destroy)
		pm->before_destroy(pm->data);

	devm_kfree(pm->dev, pm);
}

/*
 * Increases the counter and calls the power_up callback.
 *
 * Returns zero on success.
 *
 * Caller holds pm->lock.
 */
static int gcip_pm_get_locked(struct gcip_pm *pm)
{
	int ret = 0;

	gcip_pm_lockdep_assert_held(pm);

	if (!pm->count) {
		if (pm->power_down_pending)
			pm->power_down_pending = false;
		else
			ret = pm->power_up(pm->data);
	}

	if (!ret)
		pm->count++;

	dev_dbg(pm->dev, "%s: %d\n", __func__, pm->count);

	return ret;
}

int gcip_pm_get_if_powered(struct gcip_pm *pm, bool blocking)
{
	int ret = -EAGAIN;

	if (!pm)
		return 0;

	/* Fast fails without holding the lock. */
	if (!pm->count)
		return ret;

	if (blocking)
		mutex_lock(&pm->lock);
	else if (!mutex_trylock(&pm->lock))
		return ret;

	if (pm->count)
		ret = gcip_pm_get_locked(pm);

	mutex_unlock(&pm->lock);

	return ret;
}

int gcip_pm_get(struct gcip_pm *pm)
{
	int ret;

	if (!pm)
		return 0;

	mutex_lock(&pm->lock);
	ret = gcip_pm_get_locked(pm);
	mutex_unlock(&pm->lock);

	return ret;
}

void gcip_pm_put(struct gcip_pm *pm)
{
	if (!pm)
		return;

	mutex_lock(&pm->lock);

	if (WARN_ON(!pm->count))
		goto unlock;

	if (!--pm->count) {
		pm->power_down_pending = true;
		gcip_pm_try_power_down(pm);
	}

	dev_dbg(pm->dev, "%s: %d\n", __func__, pm->count);

unlock:
	mutex_unlock(&pm->lock);
}

void gcip_pm_put_async(struct gcip_pm *pm)
{
	schedule_work(&pm->put_async_work);
}

void gcip_pm_flush_put_work(struct gcip_pm *pm)
{
	flush_work(&pm->put_async_work);
}

int gcip_pm_get_count(struct gcip_pm *pm)
{
	if (!pm)
		return 0;

	return pm->count;
}

bool gcip_pm_is_powered(struct gcip_pm *pm)
{
	/* Assumes powered-on in case of no power interface. */
	return pm ? gcip_pm_get_count(pm) > 0 : true;
}

void gcip_pm_shutdown(struct gcip_pm *pm, bool force)
{
	if (!pm)
		return;

	mutex_lock(&pm->lock);

	if (pm->count) {
		if (!force)
			goto unlock;
		dev_warn(pm->dev, "Force shutdown with power up count: %d", pm->count);
	}

	gcip_pm_try_power_down(pm);

unlock:
	mutex_unlock(&pm->lock);
}
