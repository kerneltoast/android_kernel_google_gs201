/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Platform thermal driver for GXP.
 *
 * Copyright (C) 2021-2023 Google LLC
 */
#ifndef __GXP_THERMAL_H__
#define __GXP_THERMAL_H__

#include <gcip/gcip-thermal.h>

#include "gxp-internal.h"

#define GXP_COOLING_NAME "gxp-cooling"

int gxp_thermal_init(struct gxp_dev *gxp);
void gxp_thermal_exit(struct gxp_dev *gxp);

#endif /* __GXP_THERMAL_H__ */
