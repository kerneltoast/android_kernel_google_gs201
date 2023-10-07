// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU software WDT interface.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <asm/barrier.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-sw-watchdog.h"

static bool wdt_disable;
module_param(wdt_disable, bool, 0660);

/* Worker to execute action callback handler on watchdog bite. */
static void sw_wdt_handler_work(struct work_struct *work)
{
	struct edgetpu_sw_wdt_action_work *et_action_work =
		container_of(work, struct edgetpu_sw_wdt_action_work, work);

	if (et_action_work->edgetpu_sw_wdt_handler)
		et_action_work->edgetpu_sw_wdt_handler(et_action_work->data);
}

static void sw_wdt_start(struct edgetpu_sw_wdt *wdt)
{
	if (wdt->is_wdt_disabled) {
		etdev_dbg(wdt->etdev, "sw wdt disabled by module param");
		return;
	}
	etdev_dbg(wdt->etdev, "sw wdt: started\n");
	schedule_delayed_work(&wdt->dwork, wdt->hrtbeat_jiffs);
}

static void sw_wdt_stop(struct edgetpu_sw_wdt *wdt)
{
	etdev_dbg(wdt->etdev, "sw wdt: stopped\n");
	cancel_delayed_work_sync(&wdt->dwork);
}

static void sw_wdt_modify_rate(struct edgetpu_sw_wdt *wdt, unsigned long rate)
{
	if (rate == wdt->hrtbeat_jiffs)
		return;
	wdt->hrtbeat_jiffs = rate;
	/*
	 * Don't restart the work if we already encountered a firmware timeout.
	 */
	if (work_pending(&wdt->et_action_work.work))
		return;
	sw_wdt_stop(wdt);
	sw_wdt_start(wdt);
}

void edgetpu_watchdog_bite(struct edgetpu_dev *etdev, bool reset)
{
	if (!etdev->etdev_sw_wdt)
		return;
	/*
	 * Stop sw wdog delayed worker, to reduce chance this explicit call
	 * races with a sw wdog timeout.  May be in IRQ context, no sync,
	 * worker may already be active.  If we race with a sw wdog restart
	 * and need a chip reset, hopefully the P-channel reset will fail
	 * and the bigger hammer chip reset will kick in at that point.
	 */
	cancel_delayed_work(&etdev->etdev_sw_wdt->dwork);
	etdev_err(etdev, "watchdog %s", reset ? "reset" : "restart");
	etdev->reset_needed = reset;
	schedule_work(&etdev->etdev_sw_wdt->et_action_work.work);
}

/*
 * Ping the f/w for a response. Reschedule the work for next beat in case of f/w
 * is responded, or schedule a worker for action callback in case of TIMEOUT.
 */
static void sw_wdt_work(struct work_struct *work)
{
	int ret;
	struct delayed_work *dwork = to_delayed_work(work);
	struct edgetpu_sw_wdt *etdev_sw_wdt =
		container_of(dwork, struct edgetpu_sw_wdt, dwork);
	struct edgetpu_dev *etdev = etdev_sw_wdt->etdev;

	/* Ping f/w, and grab updated usage stats while we're at it. */
	etdev_dbg(etdev, "sw wdt: pinging firmware\n");
	ret = edgetpu_kci_update_usage(etdev);
	if (ret)
		etdev_dbg(etdev, "sw-watchdog ping resp:%d\n", ret);
	if (ret == -ETIMEDOUT) {
		etdev_err(etdev, "sw-watchdog response timed out\n");
		schedule_work(&etdev_sw_wdt->et_action_work.work);
	} else {
		/* reschedule to next beat. */
		schedule_delayed_work(dwork, etdev_sw_wdt->hrtbeat_jiffs);
	}
}

int edgetpu_sw_wdt_create(struct edgetpu_dev *etdev, unsigned long active_ms,
			  unsigned long dormant_ms)
{
	struct edgetpu_sw_wdt *etdev_sw_wdt;

	etdev_sw_wdt = kzalloc(sizeof(*etdev_sw_wdt), GFP_KERNEL);
	if (!etdev_sw_wdt)
		return -ENOMEM;

	etdev_sw_wdt->etdev = etdev;
	etdev_sw_wdt->hrtbeat_active = msecs_to_jiffies(active_ms);
	etdev_sw_wdt->hrtbeat_dormant = msecs_to_jiffies(dormant_ms);
	atomic_set(&etdev_sw_wdt->active_counter, 0);
	/* init to dormant rate */
	etdev_sw_wdt->hrtbeat_jiffs = etdev_sw_wdt->hrtbeat_dormant;
	INIT_DELAYED_WORK(&etdev_sw_wdt->dwork, sw_wdt_work);
	INIT_WORK(&etdev_sw_wdt->et_action_work.work, sw_wdt_handler_work);
	etdev_sw_wdt->is_wdt_disabled = wdt_disable;
	etdev->etdev_sw_wdt = etdev_sw_wdt;
	return 0;
}

int edgetpu_sw_wdt_start(struct edgetpu_dev *etdev)
{
	struct edgetpu_sw_wdt *wdt;

	/* to match edgetpu_sw_wdt_destroy() */
	smp_mb();
	wdt = etdev->etdev_sw_wdt;
	if (!wdt)
		return -EINVAL;
	if (!wdt->et_action_work.edgetpu_sw_wdt_handler)
		etdev_err(etdev, "sw wdt handler not set\n");
	sw_wdt_start(wdt);
	return 0;
}

void edgetpu_sw_wdt_stop(struct edgetpu_dev *etdev)
{
	struct edgetpu_sw_wdt *wdt;

	/* to match edgetpu_sw_wdt_destroy() */
	smp_mb();
	wdt = etdev->etdev_sw_wdt;
	if (!wdt)
		return;
	sw_wdt_stop(wdt);
}

void edgetpu_sw_wdt_destroy(struct edgetpu_dev *etdev)
{
	struct edgetpu_sw_wdt *wdt = etdev->etdev_sw_wdt;
	int counter;

	if (!wdt)
		return;
	etdev->etdev_sw_wdt = NULL;
	/*
	 * To ensure that etdev->etdev_sw_wdt is NULL so wdt_start() calls from other processes
	 * won't start the watchdog again.
	 */
	smp_mb();
	sw_wdt_stop(wdt);
	/* cancel and sync work due to watchdog bite to prevent UAF */
	cancel_work_sync(&wdt->et_action_work.work);
	counter = atomic_read(&wdt->active_counter);
	if (counter)
		etdev_warn(etdev, "Unbalanced WDT active counter: %d", counter);
	kfree(wdt);
}

void edgetpu_sw_wdt_set_handler(struct edgetpu_dev *etdev,
				void (*handler_cb)(void *), void *data)
{
	struct edgetpu_sw_wdt *et_sw_wdt = etdev->etdev_sw_wdt;

	if (!et_sw_wdt)
		return;
	et_sw_wdt->et_action_work.edgetpu_sw_wdt_handler = handler_cb;
	et_sw_wdt->et_action_work.data = data;
}

void edgetpu_sw_wdt_inc_active_ref(struct edgetpu_dev *etdev)
{
	struct edgetpu_sw_wdt *wdt = etdev->etdev_sw_wdt;

	if (!wdt)
		return;
	if (!atomic_fetch_inc(&wdt->active_counter))
		sw_wdt_modify_rate(wdt, wdt->hrtbeat_active);
}

void edgetpu_sw_wdt_dec_active_ref(struct edgetpu_dev *etdev)
{
	struct edgetpu_sw_wdt *wdt = etdev->etdev_sw_wdt;

	if (!wdt)
		return;
	if (atomic_fetch_dec(&wdt->active_counter) == 1)
		sw_wdt_modify_rate(wdt, wdt->hrtbeat_dormant);
}
