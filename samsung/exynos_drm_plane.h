/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __EXYNOS_DRM_PLANE_H__
#define __EXYNOS_DRM_PLANE_H__

#include <drm/drm_device.h>

#include "exynos_drm_drv.h"

#define EXYNOS_PLANE_ALPHA_MAX          0xff

enum {
	EXYNOS_STANDARD_UNSPECIFIED,
	EXYNOS_STANDARD_BT709,
	EXYNOS_STANDARD_BT601_625,
	EXYNOS_STANDARD_BT601_625_UNADJUSTED,
	EXYNOS_STANDARD_BT601_525,
	EXYNOS_STANDARD_BT601_525_UNADJUSTED,
	EXYNOS_STANDARD_BT2020,
	EXYNOS_STANDARD_BT2020_CONSTANT_LUMINANCE,
	EXYNOS_STANDARD_BT470M,
	EXYNOS_STANDARD_FILM,
	EXYNOS_STANDARD_DCI_P3,
	EXYNOS_STANDARD_ADOBE_RGB,
};

enum {
	EXYNOS_TRANSFER_UNSPECIFIED,
	EXYNOS_TRANSFER_LINEAR,
	EXYNOS_TRANSFER_SRGB,
	EXYNOS_TRANSFER_SMPTE_170M,
	EXYNOS_TRANSFER_GAMMA2_2,
	EXYNOS_TRANSFER_GAMMA2_6,
	EXYNOS_TRANSFER_GAMMA2_8,
	EXYNOS_TRANSFER_ST2084,
	EXYNOS_TRANSFER_HLG,
};

enum {
	EXYNOS_RANGE_UNSPECIFIED,
	EXYNOS_RANGE_FULL,
	EXYNOS_RANGE_LIMITED,
	EXYNOS_RANGE_EXTENDED,
};

#define plane_to_dpp(p)		container_of(p, struct dpp_device, plane)

int exynos_plane_init(struct drm_device *dev,
		      struct exynos_drm_plane *exynos_plane, unsigned int index,
		      const struct exynos_drm_plane_config *config);
int exynos_drm_debugfs_plane_add(struct exynos_drm_plane *exynos_plane);

#endif /* __EXYNOS_DRM_PLANE_H__ */
