/* SPDX-License-Identifier: GPL-2.0-only
 *
 * linux/drivers/gpu/drm/samsung/exynos_drm_recovery.h
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Headef file for recovery Feature.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_RECOVERY__
#define __EXYNOS_DRM_RECOVERY__

#include <linux/kthread.h>

struct decon_device;
struct exynos_recovery {
	struct work_struct work;
	int count;
	atomic_t recovering;
};

void exynos_recovery_register(struct decon_device *decon);

#endif /* __EXYNOS_DRM_RECOVERY__ */
