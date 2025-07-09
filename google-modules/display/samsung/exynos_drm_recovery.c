// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_recovery.c
 *
 * Copyright (C) 2021 Samsung Electronics Co.Ltd
 * Authors:
 *	Jiun Yu <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define pr_fmt(fmt)  "[RECOVERY] %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/export.h>
#include <drm/drm_drv.h>
#include <drm/drm_device.h>
#include <drm/drm_modeset_lock.h>
#include "exynos_drm_crtc.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_recovery.h"

static void exynos_recovery_handler(struct work_struct *work)
{
	struct exynos_recovery *recovery = container_of(work,
					struct exynos_recovery, work);
	struct decon_device *decon = container_of(recovery, struct decon_device,
					recovery);
	struct drm_atomic_state *rcv_state;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc *crtc = &decon->crtc->base;
	int ret;

	pr_info("starting recovery...\n");

	drm_modeset_acquire_init(&ctx, 0);
	rcv_state = exynos_crtc_suspend(crtc, &ctx);
	if (!IS_ERR_OR_NULL(rcv_state)) {
		ret = exynos_crtc_resume(rcv_state, &ctx);
		drm_atomic_state_put(rcv_state);
	} else {
		ret = -EINVAL;
	}

	if (!ret) {
		recovery->count++;
		pr_info("recovery is successfully finished(%d)\n", recovery->count);
	} else {
		pr_err("Failed to recover display\n");
	}
	atomic_set(&recovery->recovering, 0);
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);
}

void exynos_recovery_register(struct decon_device *decon)
{
	struct exynos_recovery *recovery = &decon->recovery;

	INIT_WORK(&recovery->work, exynos_recovery_handler);
	recovery->count = 0;
	atomic_set(&recovery->recovering, 0);

	pr_info("ESD recovery is supported\n");
}
EXPORT_SYMBOL_GPL(exynos_recovery_register);
