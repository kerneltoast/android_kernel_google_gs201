/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Devfreq driver for GXP.
 *
 * Copyright (C) 2024 Google LLC
 */

#ifndef __GXP_DEVFREQ_H__
#define __GXP_DEVFREQ_H__

#include <gcip/gcip-devfreq.h>

#include "gxp-internal.h"

/**
 * gxp_devfreq_init() - API to initialize devfreq for the device. Should be called once per probe.
 *
 * @gxp: The GXP device to operate
 *
 * Return:
 * * 0       - Initialization finished successfully.
 * * -EEXIST - devfreq is already initialized.
 * * Error codes propagated by gcip_devfreq_create() on failure.
 */
int gxp_devfreq_init(struct gxp_dev *gxp);

/**
 * gxp_devfreq_exit() - API for removing devfreq for the device.
 *
 * @gxp: The GXP device to operate.
 */
void gxp_devfreq_exit(struct gxp_dev *gxp);

#endif /* __GXP_DEVFREQ_H__ */
