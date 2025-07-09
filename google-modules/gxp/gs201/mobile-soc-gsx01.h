/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Header file for GSx01.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#ifndef __MOBILE_SOC_GSX01_H__
#define __MOBILE_SOC_GSX01_H__

#include <soc/google/exynos_pm_qos.h>

#include <gcip/gcip-slc.h>

#include "gxp-gsx01-ssmt.h"

struct gxp_soc_data {
	/* INT/MIF requests for memory bandwidth. */
	struct exynos_pm_qos_request int_min;
	struct exynos_pm_qos_request mif_min;
	struct exynos_pm_qos_request bci_min;
	struct exynos_pm_qos_request dsu_min;
	/* The Stream Security Mapping Table support. */
	struct gxp_ssmt ssmt;
	/* The SLC configurations. */
	struct gcip_slc slc;
};

#endif /* __MOBILE_SOC_GSX01_H__ */
