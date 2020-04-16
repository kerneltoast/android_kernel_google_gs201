/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Headef file for Display Quality Enhancer.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_DQE_H__
#define __EXYNOS_DRM_DQE_H__

#include <drm/samsung_drm.h>

struct decon_device;
struct exynos_dqe;
struct exynos_dqe_state;

struct exynos_dqe_funcs {
	void (*update)(struct exynos_dqe *dqe, struct exynos_dqe_state *state,
			u32 width, u32 height);
};

struct exynos_dqe_state {
	const struct gamma_matrix *gamma_matrix;
	const struct drm_color_lut *degamma_lut;
	const struct linear_matrix *linear_matrix;
	const struct cgc_lut *cgc_lut;
	struct drm_color_lut *regamma_lut;
};

struct exynos_dqe {
	void __iomem *regs;
	bool initialized;
	const struct exynos_dqe_funcs *funcs;
	struct exynos_dqe_state state;
};

void exynos_dqe_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state,
		u32 width, u32 height);
void exynos_dqe_reset(struct exynos_dqe *dqe);
struct exynos_dqe *exynos_dqe_register(struct decon_device *decon);

#endif /* __EXYNOS_DRM_DQE_H__ */
