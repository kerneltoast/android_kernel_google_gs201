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
#include <exynos_drm_drv.h>

#define EXYNOS_PLANE_ALPHA_MAX          0xff

#define STANDARD_SHIFT					 16
#define HAL_DATASPACE_STANDARD_MASK			(0x3F << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_UNSPECIFIED		 (0 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_BT709			 (1 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_BT601_625		 (2 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_BT601_625_UNADJUSTED	 (3 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_BT601_525		 (4 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_BT601_525_UNADJUSTED	 (5 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_BT2020			 (6 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_BT2020_CONSTANT_LUMINANCE (7 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_BT470M			 (8 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_FILM			 (9 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_DCI_P3			 (10 << STANDARD_SHIFT)
#define HAL_DATASPACE_STANDARD_ADOBE_RGB		 (11 << STANDARD_SHIFT)

#define TRANSFER_SHIFT					 22
#define HAL_DATASPACE_TRANSFER_MASK			(0x1F << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_UNSPECIFIED		 (0 << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_LINEAR			 (1 << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_SRGB			 (2 << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_SMPTE_170M		 (3 << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_GAMMA2_2			 (4 << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_GAMMA2_6			 (5 << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_GAMMA2_8			 (6 << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_ST2084			 (7 << TRANSFER_SHIFT)
#define HAL_DATASPACE_TRANSFER_HLG			 (8 << TRANSFER_SHIFT)

#define RANGE_SHIFT					 27
#define HAL_DATASPACE_RANGE_MASK			 (0x7 << RANGE_SHIFT)
#define HAL_DATASPACE_RANGE_UNSPECIFIED			 (0 << RANGE_SHIFT)
#define HAL_DATASPACE_RANGE_FULL			 (1 << RANGE_SHIFT)
#define HAL_DATASPACE_RANGE_LIMITED			 (2 << RANGE_SHIFT)
#define HAL_DATASPACE_RANGE_EXTENDED			 (3 << RANGE_SHIFT)

#define plane_to_dpp(p)		container_of(p, struct dpp_device, plane)

int exynos_plane_init(struct drm_device *dev,
		      struct exynos_drm_plane *exynos_plane, unsigned int index,
		      const struct exynos_drm_plane_config *config);

#endif /* __EXYNOS_DRM_PLANE_H__ */
