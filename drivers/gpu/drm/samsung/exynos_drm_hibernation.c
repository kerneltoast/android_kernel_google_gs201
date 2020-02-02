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
#include <linux/kthread.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/sched/types.h>
#include <linux/err.h>
#include <linux/atomic.h>

#include <exynos_drm_hibernation.h>
#include <exynos_drm_decon.h>

#define CAMERA_OPERATION_MASK	0xF
static bool is_camera_operating(struct exynos_hibernation *hiber)
{
	/* No need to check camera operation status. It depends on SoC */
	if (!hiber->cam_op_reg)
		return false;

	return (readl(hiber->cam_op_reg) & CAMERA_OPERATION_MASK);
}

static void exynos_hibernation_trig_reset(struct exynos_hibernation *hiber)
{
	atomic_set(&hiber->trig_cnt, hiber->entry_cnt);
}

static bool exynos_hibernation_check(struct exynos_hibernation *hiber)
{
	pr_debug("%s +\n", __func__);

	return (!is_hibernaton_blocked(hiber) &&
		!is_camera_operating(hiber) &&
		atomic_dec_and_test(&hiber->trig_cnt));
}

static void exynos_hibernation_enter(struct exynos_hibernation *hiber)
{
	struct decon_device *decon = hiber->decon;
	struct dsim_device *dsim = decon_get_dsim(decon);

	pr_debug("%s +\n", __func__);

	if (!decon || !dsim)
		return;

	mutex_lock(&hiber->lock);
	hibernation_block(hiber);

	if (decon->state != DECON_STATE_ON)
		goto ret;

	DPU_EVENT_LOG(DPU_EVT_ENTER_HIBERNATION_IN, decon->id, NULL);

	decon_enter_hibernation(decon);
	dsim_enter_ulps(dsim);

	pm_runtime_put_sync(decon->dev);

	DPU_EVENT_LOG(DPU_EVT_ENTER_HIBERNATION_OUT, decon->id, NULL);

ret:
	hibernation_unblock(hiber);
	mutex_unlock(&hiber->lock);

	pr_debug("%s: DPU power %s -\n", __func__,
			pm_runtime_active(decon->dev) ? "on" : "off");
}

static void exynos_hibernation_exit(struct exynos_hibernation *hiber)
{
	struct decon_device *decon = hiber->decon;
	struct dsim_device *dsim = decon_get_dsim(decon);

	pr_debug("%s +\n", __func__);

	if (!decon || !dsim)
		return;

	hibernation_block(hiber);

	/*
	 * It waits for finishing previous queued hibernation entry work.
	 * It only goes to sleep when work is queued or executing. If not,
	 * there is no operation here.
	 */
	kthread_flush_work(&hiber->work);

	mutex_lock(&hiber->lock);

	if (decon->state != DECON_STATE_HIBERNATION)
		goto ret;

	DPU_EVENT_LOG(DPU_EVT_EXIT_HIBERNATION_IN, decon->id, NULL);

	pm_runtime_get_sync(decon->dev);

	dsim_exit_ulps(dsim);
	decon_exit_hibernation(decon);

	exynos_hibernation_trig_reset(hiber);

	DPU_EVENT_LOG(DPU_EVT_EXIT_HIBERNATION_OUT, decon->id, NULL);

ret:
	mutex_unlock(&hiber->lock);
	hibernation_unblock(hiber);

	pr_debug("%s: DPU power %s -\n", __func__,
			pm_runtime_active(decon->dev) ? "on" : "off");
}

void hibernation_block_exit(struct exynos_hibernation *hiber)
{
	const struct exynos_hibernation_funcs *funcs;

	if (!hiber)
		return;

	hibernation_block(hiber);

	funcs = hiber->funcs;
	if (funcs)
		funcs->exit(hiber);
}

static const struct exynos_hibernation_funcs hibernation_funcs = {
	.check	= exynos_hibernation_check,
	.enter	= exynos_hibernation_enter,
	.exit	= exynos_hibernation_exit,
};

static void exynos_hibernation_handler(struct kthread_work *work)
{
	struct exynos_hibernation *hibernation =
		container_of(work, struct exynos_hibernation, work);
	const struct exynos_hibernation_funcs *funcs = hibernation->funcs;

	pr_debug("Display hibernation handler is called(trig_cnt:%d)\n",
			atomic_read(&hibernation->trig_cnt));

	/* If hibernation entry condition does NOT meet, just return here */
	if (!funcs->check(hibernation))
		return;

	exynos_hibernation_trig_reset(hibernation);
	funcs->enter(hibernation);
}

struct exynos_hibernation *
exynos_hibernation_register(struct decon_device *decon)
{
	struct device_node *np, *cam_np;
	struct sched_param param;
	struct exynos_hibernation *hibernation;
	struct device *dev = decon->dev;

	np = dev->of_node;
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

	mutex_init(&hibernation->lock);

	hibernation->entry_cnt = EXYNOS_HIBERNATION_ENTRY_CNT;
	exynos_hibernation_trig_reset(hibernation);

	atomic_set(&hibernation->block_cnt, 0);

	kthread_init_worker(&hibernation->worker);
	hibernation->thread = kthread_run(kthread_worker_fn,
			&hibernation->worker, "display_hibernation");
	if (IS_ERR(hibernation->thread)) {
		pr_err("failed to run display hibernation thread\n");
		if (hibernation->cam_op_reg)
			iounmap(hibernation->cam_op_reg);
		kfree(hibernation);
		return NULL;
	}

	param.sched_priority = 20;
	sched_setscheduler_nocheck(hibernation->thread, SCHED_FIFO, &param);
	kthread_init_work(&hibernation->work, exynos_hibernation_handler);

	pr_info("display hibernation is supported\n");

	return hibernation;
}

void exynos_hibernation_destroy(struct exynos_hibernation *hiber)
{
	if (!hiber)
		return;

	if (hiber->thread)
		kthread_stop(hiber->thread);

	if (hiber->cam_op_reg)
		iounmap(hiber->cam_op_reg);
}
