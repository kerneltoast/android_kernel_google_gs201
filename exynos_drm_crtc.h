/* SPDX-License-Identifier: GPL-2.0-only
 * exynos_drm_crtc.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _EXYNOS_DRM_CRTC_H_
#define _EXYNOS_DRM_CRTC_H_

#include "exynos_drm_drv.h"

#define HAL_COLOR_MODE_NATIVE				0
#define HAL_COLOR_MODE_STANDARD_BT601_625		1
#define HAL_COLOR_MODE_STANDARD_BT601_625_UNADJUSTED	2
#define HAL_COLOR_MODE_STANDARD_BT601_525		3
#define HAL_COLOR_MODE_STANDARD_BT601_525_UNADJUSTED	4
#define HAL_COLOR_MODE_STANDARD_BT709			5
#define HAL_COLOR_MODE_DCI_P3				6
#define HAL_COLOR_MODE_SRGB				7
#define HAL_COLOR_MODE_ADOBE_RGB			8
#define HAL_COLOR_MODE_DISPLAY_P3			9
#define HAL_COLOR_MODE_BT2020				10
#define HAL_COLOR_MODE_BT2100_PQ			11
#define HAL_COLOR_MODE_BT2100_HLG			12

/* supported bpc mode for crtc(DECON) */
#define EXYNOS_BPC_MODE_UNSPECIFIED	0
#define EXYNOS_BPC_MODE_8		1
#define EXYNOS_BPC_MODE_10		2

struct exynos_drm_crtc *exynos_drm_crtc_create(struct drm_device *drm_dev,
					struct drm_plane *plane,
					enum exynos_drm_output_type out_type,
					const struct exynos_drm_crtc_ops *ops,
					void *context);
void exynos_drm_crtc_wait_pending_update(struct exynos_drm_crtc *exynos_crtc);
void exynos_drm_crtc_finish_update(struct exynos_drm_crtc *exynos_crtc,
				   struct exynos_drm_plane *exynos_plane);

uint32_t exynos_drm_get_possible_crtcs(const struct drm_encoder *encoder,
		enum exynos_drm_output_type out_type);

/*
 * This function calls the crtc device(manager)'s te_handler() callback
 * to trigger to transfer video image at the tearing effect synchronization
 * signal.
 */
void exynos_drm_crtc_te_handler(struct drm_crtc *crtc);

void exynos_crtc_handle_event(struct exynos_drm_crtc *exynos_crtc);

void exynos_crtc_wait_for_flip_done(struct drm_atomic_state *old_state);

bool exynos_crtc_needs_disable(struct drm_crtc_state *old_state,
			struct drm_crtc_state *new_state);

void exynos_crtc_set_mode(struct drm_device *dev,
			struct drm_atomic_state *old_state);

struct drm_atomic_state
*exynos_duplicate_active_crtc_state(struct drm_crtc *crtc,
				struct drm_modeset_acquire_ctx *ctx);
struct drm_atomic_state
*exynos_crtc_suspend(struct drm_crtc *crtc,
		struct drm_modeset_acquire_ctx *ctx);
int exynos_crtc_resume(struct drm_atomic_state *state,
				struct drm_modeset_acquire_ctx *ctx);
#endif
