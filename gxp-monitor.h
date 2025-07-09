/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP abstract monitor interface for BPM and GEM.
 *
 * Copyright (C) 2024 Google LLC
 */

#ifndef __GXP_MONITOR_H__
#define __GXP_MONITOR_H__

#include <linux/platform_device.h>

#include "gxp-internal.h"

/**
 * gxp_monitor_set_count_read_data() - Set the counter to record the MCU read data transfer event.
 * @gxp: The gxp object which contains the CSR address information.
 *
 * Enable a counter of MCU and configure it to record the number of read data beats received.
 */
void gxp_monitor_set_count_read_data(struct gxp_dev *gxp);

/**
 * gxp_monitor_start() - Start the counter to record MCU events.
 * @gxp: The gxp object which contains the CSR address information.
 */
void gxp_monitor_start(struct gxp_dev *gxp);

/**
 * gxp_monitor_stop() - Stop the counter from recording MCU events.
 * @gxp: The gxp object which contains the CSR address information.
 */
void gxp_monitor_stop(struct gxp_dev *gxp);

/**
 * gxp_monitor_get_count_read_data() - Get the count of MCU read data transfer event.
 * @gxp: The gxp object which contains the CSR address information.
 *
 * Return: The number retrieved from the counter.
 */
u32 gxp_monitor_get_count_read_data(struct gxp_dev *gxp);

#endif /* __GXP_BPM_H__ */
