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
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_uapi.h>
#include <drm/drm_modeset_lock.h>
#include "exynos_drm_decon.h"
#include "exynos_drm_recovery.h"

static struct drm_atomic_state *_duplicate_active_crtc_state(struct drm_crtc *crtc,
							     struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_device *dev = crtc->dev;
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	int err;

	state = drm_atomic_state_alloc(dev);
	if (!state)
		return ERR_PTR(-ENOMEM);

	state->acquire_ctx = ctx;

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		err = PTR_ERR(crtc_state);
		goto free_state;
	}

	if (!crtc_state->active) {
		pr_warn("crtc[%s]: skipping duplication of inactive crtc state\n", crtc->name);
		err = -EINVAL;
		goto free_state;
	}

	err = drm_atomic_add_affected_planes(state, crtc);
	if (err)
		goto free_state;

	err = drm_atomic_add_affected_connectors(state, crtc);
	if (err)
		goto free_state;

	/* clear the acquire context so that it isn't accidentally reused */
	state->acquire_ctx = NULL;

free_state:
	if (err < 0) {
		drm_atomic_state_put(state);
		state = ERR_PTR(err);
	}

	return state;
}

static void exynos_recovery_handler(struct work_struct *work)
{
	struct exynos_recovery *recovery = container_of(work,
					struct exynos_recovery, work);
	struct decon_device *decon = container_of(recovery, struct decon_device,
					recovery);
	struct drm_device *dev = decon->drm_dev;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_atomic_state *state, *rcv_state;
	struct drm_crtc_state *crtc_state;
	struct drm_crtc *crtc = &decon->crtc->base;
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	int ret, i;

	pr_info("starting recovery...\n");

	drm_modeset_acquire_init(&ctx, 0);

	rcv_state = _duplicate_active_crtc_state(crtc, &ctx);
	if (IS_ERR(rcv_state)) {
		ret = PTR_ERR(rcv_state);
		goto out_drop_locks;
	}

	state = drm_atomic_state_alloc(dev);
	if (!state) {
		drm_atomic_state_put(rcv_state);
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

	crtc_state->active = false;

	ret = drm_atomic_set_mode_prop_for_crtc(crtc_state, NULL);
	if (ret)
		goto out;

	ret = drm_atomic_add_affected_planes(state, crtc);
	if (ret)
		goto out;

	ret = drm_atomic_add_affected_connectors(state, crtc);
	if (ret)
		goto out;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		ret = drm_atomic_set_crtc_for_connector(conn_state, NULL);
		if (ret)
			goto out;
	}

	for_each_new_plane_in_state(state, plane, plane_state, i) {
		ret = drm_atomic_set_crtc_for_plane(plane_state, NULL);
		if (ret)
			goto out;

		drm_atomic_set_fb_for_plane(plane_state, NULL);
	}

	ret = drm_atomic_commit(state);
	if (ret)
		goto out;

	ret = drm_atomic_helper_commit_duplicated_state(rcv_state, &ctx);
	if (ret)
		goto out;

	recovery->count++;
	pr_info("recovery is successfully finished(%d)\n", recovery->count);

out:
	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		drm_atomic_state_clear(rcv_state);
		ret = drm_modeset_backoff(&ctx);
		if (!ret)
			goto retry;
	}

	drm_atomic_state_put(state);
	drm_atomic_state_put(rcv_state);

out_drop_locks:
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
EXPORT_SYMBOL(exynos_recovery_register);
