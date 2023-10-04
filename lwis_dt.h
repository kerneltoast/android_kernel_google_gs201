/*
 * Google LWIS Device Tree Parser
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_DT_H_
#define LWIS_DT_H_

#include <linux/device.h>

#include "lwis_device.h"
#include "lwis_device_i2c.h"
#include "lwis_device_ioreg.h"
#include "lwis_device_top.h"

/*
 *  lwis_base_parse_dt: Parse device configurations based on device tree
 *  entries. This is being called by all types of devices.
 */
int lwis_base_parse_dt(struct lwis_device *lwis_dev);

/*
 *  lwis_i2c_device_parse_dt: Parse device configurations specifically for
 *  i2c devices.
 */
int lwis_i2c_device_parse_dt(struct lwis_i2c_device *i2c_dev);

/*
 *  lwis_ioreg_device_parse_dt: Parse device configurations specifically for
 *  IOREG devices.
 */
int lwis_ioreg_device_parse_dt(struct lwis_ioreg_device *ioreg_dev);

/*
 *  lwis_top_device_parse_dt: Parse device configurations specifically for
 *  top devices.
 */
int lwis_top_device_parse_dt(struct lwis_top_device *top_dev);

#endif /* LWIS_DT_H_ */
