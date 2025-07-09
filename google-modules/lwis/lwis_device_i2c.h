/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS I2C Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_DEVICE_I2C_H_
#define LWIS_DEVICE_I2C_H_

#include <linux/i2c.h>
#include <linux/pinctrl/consumer.h>

#include "lwis_device.h"
#include "lwis_bus_manager.h"

#define MAX_I2C_LOCK_NUM 8

/*
 *  struct lwis_i2c_device
 *  "Derived" lwis_device struct, with added i2c related elements.
 */
struct lwis_i2c_device {
	struct lwis_device base_dev;
	int address;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct pinctrl *state_pinctrl;
	bool set_master_pinctrl_state;
	/* Group id for I2C lock */
	u32 i2c_lock_group_id;
	/* Mutex shared by the same group id's I2C devices */
	struct mutex *group_i2c_lock;
	struct lwis_bus_manager *i2c_bus_manager;
	int device_priority;
};

int lwis_i2c_device_init(void);
int lwis_i2c_device_deinit(void);

#if IS_ENABLED(CONFIG_INPUT_STMVL53L1)
/*
 * Module stmvl53l1 shares one i2c bus with some lwis i2c devices. And use the
 * two APIs in stmvl53l1 driver to well handle the enabling and disabling.
 */
extern bool is_shared_i2c_with_stmvl53l1(struct pinctrl *pinctrl);
extern int shared_i2c_set_state(struct device *dev, struct pinctrl *pinctrl, const char *state_str);
#endif

#endif /* LWIS_DEVICE_I2C_H_ */
