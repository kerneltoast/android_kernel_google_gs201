/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS I2C Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_I2C_H_
#define LWIS_I2C_H_

#include <linux/i2c.h>
#include <linux/pinctrl/consumer.h>

#include "lwis_commands.h"
#include "lwis_device_i2c.h"

#define I2C_STATE_OFF_STRING "off_i2c"
#define I2C_STATE_ON_STRING "on_i2c"

/*
 *  lwis_i2c_set_state: Enable or disable the i2c device.
 *  NOTE: state_str must match the pinctrl-names defined in the i2c driver.
 *  Pinctrl states can be found in the device tree, look for the i2c entry and
 *  the state names are defined under "pinctrl-names".  Their corresponding
 *  functions are defined under "pinctrl-N".
 */
int lwis_i2c_set_state(struct lwis_i2c_device *i2c, const char *state_str);

/*
 *  lwis_i2c_io_entry_rw: Read/Write from i2c bus via io_entry request.
 *  The readback values will be stored in the entry.
 */
int lwis_i2c_io_entry_rw(struct lwis_i2c_device *i2c, struct lwis_io_entry *entry);

#endif /* LWIS_I2C_H_ */
