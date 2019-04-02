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

#define EXYNOS_DRM_BLEND_ALPHA_OPAQUE		0xff

#define EXYNOS_DRM_MODE_BLEND_PREMULTI		0
#define EXYNOS_DRM_MODE_BLEND_COVERAGE		1
#define EXYNOS_DRM_MODE_BLEND_PIXEL_NONE	2

#define plane_to_dpp(p)		container_of(p, struct dpp_device, plane)

int exynos_plane_init(struct drm_device *dev,
		      struct exynos_drm_plane *exynos_plane, unsigned int index,
		      const struct exynos_drm_plane_config *config);

#endif /* __EXYNOS_DRM_PLANE_H__ */
