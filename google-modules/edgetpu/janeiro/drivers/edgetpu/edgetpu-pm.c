// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeTPU power management interface.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-pm.h"
#include "edgetpu-sw-watchdog.h"
#include "edgetpu-wakelock.h"

#if IS_ENABLED(CONFIG_EDGETPU_TEST)
#include "unittests/factory/fake-edgetpu-firmware.h"
#define SIM_PCHANNEL(etdev) fake_edgetpu_firmware_sim_pchannel(etdev)
#else
#define SIM_PCHANNEL(...)
#endif

#define EDGETPU_ASYNC_POWER_DOWN_RETRY_DELAY	200

struct edgetpu_pm_private {
	const struct edgetpu_pm_handlers *handlers;
	struct mutex lock;
	/* Power up counter. Protected by @lock */
	int power_up_count;
	/* Flag indicating a deferred power down is pending. Protected by @lock */
	bool power_down_pending;
	/* Worker to handle async power down retry */
	struct delayed_work power_down_work;
	/* Back pointer to parent struct */
	struct edgetpu_pm *etpm;
};

/*
 * Increases the counter and call the power_up callback.
 *
 * Returns zero on success.
 *
 * Caller holds etpm->p->lock.
 */
static int edgetpu_pm_get_locked(struct edgetpu_pm *etpm)
{
	int power_up_count = etpm->p->power_up_count++;
	int ret = 0;

	if (!power_up_count) {
		if (etpm->p->power_down_pending) {
			etpm->p->power_down_pending = false;
		} else {
			ret = etpm->p->handlers->power_up(etpm);
			if (!ret)
				edgetpu_mailbox_restore_active_mailbox_queues(etpm->etdev);
		}
	}
	if (ret)
		etpm->p->power_up_count--;
	etdev_dbg(etpm->etdev, "%s: %d\n", __func__, etpm->p->power_up_count);
	return ret;
}

int edgetpu_pm_trylock(struct edgetpu_pm *etpm)
{
	if (!etpm || !etpm->p->handlers || !etpm->p->handlers->power_up)
		return 1;
	return mutex_trylock(&etpm->p->lock);
}

void edgetpu_pm_unlock(struct edgetpu_pm *etpm)
{
	if (!etpm || !etpm->p->handlers || !etpm->p->handlers->power_up)
		return;
	mutex_unlock(&etpm->p->lock);
}

bool edgetpu_pm_get_if_powered(struct edgetpu_pm *etpm)
{
	bool ret;

	if (!etpm || !etpm->p->handlers || !etpm->p->handlers->power_up)
		return true;
	/* fast fail without holding the lock */
	if (!etpm->p->power_up_count)
		return false;
	mutex_lock(&etpm->p->lock);
	if (etpm->p->power_up_count)
		ret = !edgetpu_pm_get_locked(etpm);
	else
		ret = false;
	mutex_unlock(&etpm->p->lock);
	return ret;
}

int edgetpu_pm_get(struct edgetpu_pm *etpm)
{
	int ret;

	if (!etpm || !etpm->p->handlers || !etpm->p->handlers->power_up)
		return 0;

	mutex_lock(&etpm->p->lock);
	ret = edgetpu_pm_get_locked(etpm);
	mutex_unlock(&etpm->p->lock);

	return ret;
}

/* Caller must hold @etpm->p->lock */
static void edgetpu_pm_try_power_down(struct edgetpu_pm *etpm)
{
	int ret = etpm->p->handlers->power_down(etpm);

	if (ret == -EAGAIN) {
		etdev_warn(etpm->etdev, "Power down request denied. Retrying in %d ms\n",
			   EDGETPU_ASYNC_POWER_DOWN_RETRY_DELAY);
		etpm->p->power_down_pending = true;
		schedule_delayed_work(&etpm->p->power_down_work,
				      msecs_to_jiffies(EDGETPU_ASYNC_POWER_DOWN_RETRY_DELAY));
	} else {
		if (ret)
			etdev_warn(etpm->etdev, "Power down request failed (%d)\n", ret);
		etpm->p->power_down_pending = false;
	}
}

/* Worker for async power down */
static void edgetpu_pm_async_power_down_work(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct edgetpu_pm_private *p =
		container_of(dwork, struct edgetpu_pm_private, power_down_work);

	mutex_lock(&p->lock);
	etdev_info(p->etpm->etdev, "Delayed power down starting\n");
	if (p->power_down_pending)
		edgetpu_pm_try_power_down(p->etpm);
	else
		etdev_info(p->etpm->etdev, "Delayed power down cancelled\n");
	mutex_unlock(&p->lock);
}

void edgetpu_pm_put(struct edgetpu_pm *etpm)
{
	if (!etpm || !etpm->p->handlers || !etpm->p->handlers->power_down)
		return;
	mutex_lock(&etpm->p->lock);
	if (!etpm->p->power_up_count) {
		dev_err(etpm->etdev->dev, "Unbalanced pm_put");
		WARN_ON(1);
		mutex_unlock(&etpm->p->lock);
		return;
	}
	if (!--etpm->p->power_up_count) {
		edgetpu_sw_wdt_stop(etpm->etdev);
		edgetpu_pm_try_power_down(etpm);
	}
	etdev_dbg(etpm->etdev, "%s: %d\n", __func__, etpm->p->power_up_count);
	mutex_unlock(&etpm->p->lock);
}

int edgetpu_pm_create(struct edgetpu_dev *etdev,
		      const struct edgetpu_pm_handlers *handlers)
{
	int ret = 0;
	struct edgetpu_pm *etpm;

	if (etdev->pm) {
		dev_err(etdev->dev,
			"Refusing to replace existing PM interface\n");
		return -EEXIST;
	}

	etpm = kzalloc(sizeof(*etpm), GFP_KERNEL);
	if (!etpm)
		return -ENOMEM;

	etpm->p = kzalloc(sizeof(*etpm->p), GFP_KERNEL);
	if (!etpm->p) {
		ret = -ENOMEM;
		goto out_free_etpm;
	}

	INIT_DELAYED_WORK(&etpm->p->power_down_work, edgetpu_pm_async_power_down_work);
	etpm->p->etpm = etpm;
	etpm->p->handlers = handlers;
	etpm->etdev = etdev;

	mutex_init(&etpm->p->lock);

	if (handlers->after_create) {
		ret = handlers->after_create(etpm);
		if (ret) {
			ret = -EINVAL;
			goto out_free_etpm_p;
		}
	}
	etdev->pm = etpm;
	return 0;
out_free_etpm_p:
	kfree(etpm->p);
out_free_etpm:
	kfree(etpm);
	return ret;
}

void edgetpu_pm_destroy(struct edgetpu_dev *etdev)
{
	const struct edgetpu_pm_handlers *handlers;

	if (!etdev->pm)
		return;
	if (etdev->pm->p) {
		handlers = etdev->pm->p->handlers;
		etdev->pm->p->power_down_pending = false;
		cancel_delayed_work_sync(&etdev->pm->p->power_down_work);
		if (handlers && handlers->before_destroy)
			handlers->before_destroy(etdev->pm);
		kfree(etdev->pm->p);
	}
	kfree(etdev->pm);
	etdev->pm = NULL;
}

void edgetpu_pm_shutdown(struct edgetpu_dev *etdev, bool force)
{
	struct edgetpu_pm *etpm = etdev->pm;

	if (!etpm)
		return;

	mutex_lock(&etpm->p->lock);

	/* someone is using the device */
	if (etpm->p->power_up_count) {
		if (!force)
			goto unlock;
		else
			etdev_warn(etdev, "Leaving %d clients behind!\n",
				   etpm->p->power_up_count);
	}

	if (etpm->p->handlers && etpm->p->handlers->power_down)
		etpm->p->handlers->power_down(etpm);
unlock:
	mutex_unlock(&etpm->p->lock);
}

bool edgetpu_is_powered(struct edgetpu_dev *etdev)
{
	struct edgetpu_pm *etpm = etdev->pm;

	if (!etpm)
		/* Assume powered-on in case of no power interface. */
		return true;
	return etpm->p->power_up_count;
}

#define etdev_poll_power_state(etdev, val, cond)                               \
	({                                                                     \
		SIM_PCHANNEL(etdev);                                           \
		readl_relaxed_poll_timeout(                                    \
			etdev->regs.mem + EDGETPU_REG_POWER_CONTROL, val,      \
			cond, 1, EDGETPU_PCHANNEL_STATE_CHANGE_TIMEOUT);       \
	})

static int pchannel_state_change_request(struct edgetpu_dev *etdev, int state)
{
	int ret;
	u32 val;
	bool deny = false;

	/* P-channel state change request and handshake. */
	val = edgetpu_dev_read_32(etdev, EDGETPU_REG_POWER_CONTROL);
	/* Phase 1: Drive PREQ to 0 */
	if (val & PREQ) {
		edgetpu_dev_write_32(etdev, EDGETPU_REG_POWER_CONTROL,
				     val & ~(PREQ));
		ret = etdev_poll_power_state(etdev, val, (val & PACCEPT) == 0);
		if (ret) {
			etdev_err(etdev, "p-channel request timeout\n");
			return ret;
		}
	}
	/* Phase 2: Request state */
	edgetpu_dev_write_32(etdev, EDGETPU_REG_POWER_CONTROL,
			     (state << PSTATE_SHIFT) | PREQ);
	SIM_PCHANNEL(etdev);

	/* don't wait for state accept if STATE RUN */
	if (state == STATE_RUN)
		return 0;

	/* Phase 3: CPU acknowledgment */
	ret = etdev_poll_power_state(etdev, val,
				     (val & PACCEPT) || (val & PDENY));
	if (val & PDENY) {
		edgetpu_dev_write_32(etdev, EDGETPU_REG_POWER_CONTROL,
				     val & ~(state << PSTATE_SHIFT));
		etdev_dbg(etdev, "p-channel state change request denied\n");
		deny = true;
	}
	if (ret) {
		etdev_dbg(etdev, "p-channel state change request timeout\n");
		return ret;
	}
	/* Phase 4. Drive PREQ to 0 */
	edgetpu_dev_write_32(etdev, EDGETPU_REG_POWER_CONTROL, val & ~(PREQ));
	ret = etdev_poll_power_state(
		etdev, val, ((val & PACCEPT) == 0) && ((val & PDENY) == 0));

	return deny ? -EACCES : ret;
}

int edgetpu_pchannel_power_down(struct edgetpu_dev *etdev, bool wait_on_pactive)
{
	int ret;
	int tries = EDGETPU_PCHANNEL_STATE_CHANGE_RETRIES;
	u32 val;

	etdev_dbg(etdev, "Starting p-channel power down\n");
	edgetpu_sw_wdt_stop(etdev);
	ret = edgetpu_kci_shutdown(etdev->kci);
	if (ret) {
		etdev_err(etdev, "p-channel power down routing failed: %d",
			  ret);
		return ret;
	}
	if (wait_on_pactive) {
		/* wait for PACTIVE[1] goes low. */
		ret = etdev_poll_power_state(etdev, val, (val & PACTIVE) == 0);
		tries = 1;
	}
	if (ret)
		return ret;
	do {
		ret = pchannel_state_change_request(etdev, STATE_SHUTDOWN);
		tries--;
		/* Throttle the retry */
		if (tries && ret == -EACCES)
			usleep_range(EDGETPU_PCHANNEL_RETRY_DELAY_MIN,
				     EDGETPU_PCHANNEL_RETRY_DELAY_MAX);
	} while (ret && tries);

	if (ret)
		etdev_err(etdev, "p-channel shutdown state change failed: %d",
			  ret);

	return ret;
}

void edgetpu_pchannel_power_up(struct edgetpu_dev *etdev)
{
	pchannel_state_change_request(etdev, STATE_RUN);
}

#if IS_ENABLED(CONFIG_PM_SLEEP)

int edgetpu_pm_suspend(struct edgetpu_dev *etdev)
{
	struct edgetpu_pm *etpm = etdev->pm;
	struct edgetpu_list_device_client *lc;

	if (!etpm || !etpm->p->power_up_count)
		return 0;

	etdev_warn_ratelimited(
		etdev, "cannot suspend with power up count = %d\n",
		etpm->p->power_up_count);

	if (!mutex_trylock(&etdev->clients_lock))
		return -EAGAIN;
	for_each_list_device_client(etdev, lc) {
		if (NO_WAKELOCK(lc->client->wakelock) ||
		    !lc->client->wakelock->req_count)
			continue;
		etdev_warn_ratelimited(etdev,
				       "client pid %d tgid %d count %d\n",
				       lc->client->pid,
				       lc->client->tgid,
				       lc->client->wakelock->req_count);
	}
	mutex_unlock(&etdev->clients_lock);
	return -EAGAIN;
}

int edgetpu_pm_resume(struct edgetpu_dev *etdev)
{
	struct edgetpu_pm *etpm = etdev->pm;

	if (etpm && etpm->p->power_up_count)
		etdev_warn_ratelimited(etdev,
				       "resumed with power up count = %d\n",
				       etpm->p->power_up_count);

	return 0;
}

#endif /* IS_ENABLED(CONFIG_PM_SLEEP) */
