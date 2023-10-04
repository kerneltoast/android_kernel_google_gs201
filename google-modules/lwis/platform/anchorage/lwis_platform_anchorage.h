/*
 * Google LWIS Anchorage Platform-Specific Functions
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_PLATFORM_ANCHORAGE_H_
#define LWIS_PLATFORM_ANCHORAGE_H_

#include <soc/google/exynos_pm_qos.h>

struct lwis_platform {
	struct exynos_pm_qos_request pm_qos_int_cam;
	struct exynos_pm_qos_request pm_qos_int;
	struct exynos_pm_qos_request pm_qos_cam;
	struct exynos_pm_qos_request pm_qos_mem;
	struct exynos_pm_qos_request pm_qos_tnr;
	/* struct exynos_pm_qos_request pm_qos_hpg; */
};

#endif /* LWIS_PLATFORM_ANCHORAGE_H_ */