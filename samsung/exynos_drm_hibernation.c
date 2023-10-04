// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_hibernation.c
 *
 * Copyright (C) 2020 Samsung Electronics Co.Ltd
 * Authors:
 *	Jiun Yu <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/atomic.h>

#include <trace/dpu_trace.h>

#include <dqe_cal.h>

#include "exynos_drm_decon.h"
#include "exynos_drm_hibernation.h"
#include "exynos_drm_writeback.h"

#define HIBERNATION_ENTRY_MIN_TIME_MS		50
#define CAMERA_OPERATION_MASK	0xF

static bool is_camera_operating(struct exynos_hibernation *hiber)
{
	/* No need to check camera operation status. It depends on SoC */
	if (!hiber->cam_op_reg)
		return false;

	return (readl(hiber->cam_op_reg) & CAMERA_OPERATION_MASK);
}

static inline bool is_hibernation_enabled(struct exynos_hibernation *hiber)
{
	/* hibernation is only supported in command mode */
	return hiber && hiber->enabled && hiber->decon->config.mode.op_mode == DECON_COMMAND_MODE;
}

static inline bool is_hibernaton_blocked(struct exynos_hibernation *hiber)
{
	return (!is_hibernation_enabled(hiber)) ||
		(atomic_read(&hiber->block_cnt) > 0) ||
		(hiber->decon->state != DECON_STATE_ON);
}

static inline bool is_dqe_dimming_in_progress(struct decon_device *decon)
{
	return (decon->dqe &&
		dqe_reg_dimming_in_progress(decon->id));
}

static bool exynos_hibernation_check(struct exynos_hibernation *hiber)
{
	pr_debug("%s +\n", __func__);

	return (!is_camera_operating(hiber) &&
		!is_dqe_dimming_in_progress(hiber->decon));
}

static inline void hibernation_unblock(struct exynos_hibernation *hiber)
{
	WARN_ON(!atomic_add_unless(&hiber->block_cnt, -1, 0));
}

static int exynos_crtc_self_refresh_update(struct drm_crtc *crtc, bool enable, bool nonblock)
{
	struct drm_device *dev = crtc->dev;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_atomic_state *state;
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	int i, ret = 0;
	ktime_t start = ktime_get();

	drm_modeset_acquire_init(&ctx, 0);

	state = drm_atomic_state_alloc(dev);
	if (!state) {
		ret = -ENOMEM;
		goto out_drop_locks;
	}

retry:
	state->acquire_ctx = &ctx;

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		ret = PTR_ERR(crtc_state);
		goto out;
	}

	if (!crtc_state->enable) {
		pr_debug("%s: skipping due to crtc disabled\n", __func__);
		goto out;
	}

	/* if already in desired state we can skip */
	if (crtc->state->active == !enable) {
		pr_debug("%s: skipping due to (active=%d enable=%d)\n", __func__,
			 crtc->state->active, enable);
		goto out;
	}

	crtc_state->active = !enable;
	crtc_state->self_refresh_active = enable;
	if (!enable) {
		struct exynos_drm_crtc_state *exynos_state = to_exynos_crtc_state(crtc_state);

		exynos_state->hibernation_exit = true;
	}

	ret = drm_atomic_add_affected_connectors(state, crtc);
	if (ret)
		goto out;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		if (!conn_state->self_refresh_aware)
			goto out;
	}

	if (nonblock)
		ret = drm_atomic_nonblocking_commit(state);
	else
		ret = drm_atomic_commit(state);

	if (ret)
		goto out;

out:
	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		ret = drm_modeset_backoff(&ctx);
		if (!ret)
			goto retry;
	}

	drm_atomic_state_put(state);

out_drop_locks:
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	if (!ret) {
		ktime_t delta = ktime_sub(ktime_get(), start);

		pr_debug("%s: %s took %lldus\n", __func__, enable ? "on" : "off",
			 ktime_to_us(delta));
	} else if (ret == -EBUSY) {
		pr_debug("%s: aborted due to normal commit pending\n", __func__);
	} else {
		pr_warn("%s: unable to enter self refresh (%d)\n", __func__, ret);
	}

	return ret;
}

static int exynos_hibernation_enter(struct exynos_hibernation *hiber, bool nonblock)
{
	struct decon_device *decon = hiber->decon;
	int ret;

	pr_debug("%s +\n", __func__);

	DPU_ATRACE_BEGIN(__func__);
	ret = exynos_crtc_self_refresh_update(&decon->crtc->base, true, nonblock);
	DPU_ATRACE_END(__func__);

	return ret;
}

static int exynos_hibernation_exit(struct exynos_hibernation *hiber, bool nonblock)
{
	struct decon_device *decon = hiber->decon;
	int ret;

	DPU_ATRACE_BEGIN(__func__);
	ret = exynos_crtc_self_refresh_update(&decon->crtc->base, false, nonblock);
	DPU_ATRACE_END(__func__);

	pr_debug("%s: DPU power %s\n", __func__,
			pm_runtime_active(decon->dev) ? "on" : "off");

	return ret;
}

static bool exynos_hibernation_cancel(struct exynos_hibernation *hiber)
{
	struct decon_device *decon = hiber->decon;

	hibernation_block(hiber);

	/*
	 * Cancel and/or wait for finishing previous queued hibernation entry work. It only
	 * goes to sleep when work is currently executing. If not, there is no operation here.
	 */
	kthread_cancel_delayed_work_sync(&hiber->dwork);

	hibernation_unblock(hiber);

	pr_debug("%s: DPU power %s\n", __func__,
			pm_runtime_active(decon->dev) ? "on" : "off");

	return decon->state != DECON_STATE_HIBERNATION;
}

/**
 * hibernation_block_sync - block hibernation, cancel/wait for any pending hibernation work
 * @hiber: hibernation block ptr
 *
 * Return: %true if display is currently in hibernation, %false otherwise
 */
static bool hibernation_block_sync(struct exynos_hibernation *hiber)
{
	const struct exynos_hibernation_funcs *funcs;
	int block_cnt;
	bool ret;

	if (!hiber)
		return false;

	block_cnt = hibernation_block(hiber);

	if (!is_hibernation_enabled(hiber))
		return false;

	funcs = hiber->funcs;

	ret = !funcs || !funcs->cancel(hiber);

	pr_debug("%s: block_cnt(%d)\n", __func__, block_cnt);

	return ret;
}

static bool _hibernation_block_exit(struct exynos_hibernation *hiber, bool nonblock)
{
	bool hibernation_on;

	if (!hiber)
		return false;

	hibernation_on = hibernation_block_sync(hiber);

	if (hibernation_on && hiber->funcs)
		return !hiber->funcs->exit(hiber, nonblock);

	return hibernation_on;
}

void hibernation_block_exit(struct exynos_hibernation *hiber)
{
	_hibernation_block_exit(hiber, false);
}

void hibernation_unblock_enter(struct exynos_hibernation *hiber)
{
	if (!hiber)
		return;

	hibernation_unblock(hiber);

	if (!is_hibernaton_blocked(hiber))
		kthread_mod_delayed_work(&hiber->decon->worker, &hiber->dwork,
			msecs_to_jiffies(HIBERNATION_ENTRY_MIN_TIME_MS));

	pr_debug("%s: block_cnt(%d)\n", __func__, atomic_read(&hiber->block_cnt));
}

bool exynos_hibernation_async_exit(struct exynos_hibernation *hiber)
{
	bool hibernation_on = _hibernation_block_exit(hiber, true);

	hibernation_unblock_enter(hiber);

	return hibernation_on;
}

static const struct exynos_hibernation_funcs hibernation_funcs = {
	.check	= exynos_hibernation_check,
	.enter	= exynos_hibernation_enter,
	.exit	= exynos_hibernation_exit,
	.cancel	= exynos_hibernation_cancel,
};

static int _exynos_hibernation_run(struct exynos_hibernation *hibernation, bool nonblock)
{
	const struct exynos_hibernation_funcs *funcs = hibernation->funcs;
	struct decon_device *decon = hibernation->decon;
	int rc = 0;

	mutex_lock(&hibernation->lock);
	pr_debug("%s: decon state: %d +\n", __func__, decon->state);

	if (is_hibernaton_blocked(hibernation)) {
		if (decon->state == DECON_STATE_ON)
			rc = -EBUSY;
		goto ret;
	}

	/* If hibernation entry condition does NOT meet, try again later */
	if (!funcs->check(hibernation)) {
		rc = -EAGAIN;
		goto ret;
	}

	hibernation_block(hibernation);
	rc = funcs->enter(hibernation, nonblock);
	hibernation_unblock(hibernation);
ret:
	mutex_unlock(&hibernation->lock);

	pr_debug("%s: -\n", __func__);

	return rc;
}

static void exynos_hibernation_handler(struct kthread_work *work)
{
	struct exynos_hibernation *hibernation = container_of(work,
			struct exynos_hibernation, dwork.work);
	int rc;

	pr_debug("Display hibernation handler is called\n");

	rc = _exynos_hibernation_run(hibernation, true);
	if (rc == -EAGAIN)
		kthread_mod_delayed_work(&hibernation->decon->worker, &hibernation->dwork,
			msecs_to_jiffies(HIBERNATION_ENTRY_MIN_TIME_MS));
}

int exynos_hibernation_suspend(struct exynos_hibernation *hiber)
{
	int ret = 0;

	if (!hiber)
		return 0;

	/* cancel any scheduled delayed work, do it synchronously instead */
	kthread_cancel_delayed_work_sync(&hiber->dwork);

	/* make sure all work is complete (including async commits) */
	kthread_flush_worker(&hiber->decon->worker);

	if (atomic_read(&hiber->block_cnt) > 0)
		ret = -EBUSY;
	else if (hiber->decon->state == DECON_STATE_ON)
		ret = _exynos_hibernation_run(hiber, false);

	return ret;
}

struct exynos_hibernation *
exynos_hibernation_register(struct decon_device *decon)
{
	struct device_node *np, *cam_np;
	struct exynos_hibernation *hibernation;
	struct device *dev = decon->dev;

	np = dev->of_node;

	if (of_property_read_bool(np, "override-hibernation")) {
		pr_info("display hibernation is overridden\n");
		return NULL;
	}

	if (!of_property_read_bool(np, "hibernation")) {
		pr_info("display hibernation is not supported\n");
		return NULL;
	}

	hibernation = devm_kzalloc(dev, sizeof(struct exynos_hibernation),
			GFP_KERNEL);
	if (!hibernation)
		return NULL;

	cam_np = of_get_child_by_name(np, "camera-operation");
	if (!cam_np) {
		pr_info("doesn't need to get camera operation register\n");
		hibernation->cam_op_reg = NULL;
	} else {
		hibernation->cam_op_reg = of_iomap(cam_np, 0);
		if (!hibernation->cam_op_reg) {
			pr_err("failed to map camera operation register\n");
			kfree(hibernation);
			return NULL;
		}
	}

	hibernation->decon = decon;
	hibernation->funcs = &hibernation_funcs;
	hibernation->enabled = true;

	mutex_init(&hibernation->lock);

	atomic_set(&hibernation->block_cnt, 0);

	kthread_init_delayed_work(&hibernation->dwork, exynos_hibernation_handler);

	pr_info("display hibernation is supported\n");

	return hibernation;
}

void exynos_hibernation_destroy(struct exynos_hibernation *hiber)
{
	if (!is_hibernation_enabled(hiber))
		return;

	if (hiber->cam_op_reg)
		iounmap(hiber->cam_op_reg);
}
