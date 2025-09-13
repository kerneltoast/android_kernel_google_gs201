/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Dynamic Power Managerment
 *
 * Copyright (c) 2020 Google, LLC
 */

#ifndef LWIS_DPM_DEVICE_H_
#define LWIS_DPM_DEVICE_H_

#include "lwis_commands.h"
#include "lwis_device.h"

/*
 *  struct lwis_dpm_device
 *  The device majorly control/handle requests from dpm clients.
 */
struct lwis_dpm_device {
	struct lwis_device base_dev;
};

/*
 *  lwis_dpm_update_clock: update specific clock setting on lwis device.
 *  clk_settings needs to be freed at the end of this function.
 */
int lwis_dpm_update_clock(struct lwis_device *lwis_dev, struct lwis_clk_setting *clk_settings,
			  size_t num_settings);

/*
 *  lwis_dpm_update_qos: update qos requirement from dpm client.
 */
int lwis_dpm_update_qos(struct lwis_device *lwis_dev, struct lwis_qos_setting_v3 *qos_setting);

/*
 *  lwis_dpm_read_clock: read current IP core clock for given lwis device.
 *  The unit is hz.
 */
uint32_t lwis_dpm_read_clock(struct lwis_device *lwis_dev);

int lwis_dpm_device_init(void);
int lwis_dpm_device_deinit(void);

#endif /* LWIS_DPM_DEVICE_H_ */
