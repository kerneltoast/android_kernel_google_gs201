/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**  @file stmvl53l1-i2c.h
 * Linux kernel i2c/cci  wrapper for  ST VL53L1 sensor i2c interface
 **/

#ifndef STMVL53L1_I2C_H
#define STMVL53L1_I2C_H
#include <linux/types.h>
#include "stmvl53l1.h"

struct i2c_data {
	struct i2c_client *client;
	/** back link to driver for interrupt and clean-up */
	struct stmvl53l1_data *vl53l1_data;

	/* reference counter */
	struct kref ref;

	/*!< vio power control */
	int vio_gpio;
	struct regulator *vio;
	int vio_voltage;

	/*!< power enable gpio number
	 *
	 * if -1 no gpio if vdd not avl pwr is not controllable
	 */
	int pwren_gpio;

	/*!< xsdn reset (low active) gpio number to device
	 *
	 *  -1  mean none assume no "resetable"
	 */
	int xsdn_gpio;

	/*!< intr gpio number to device
	 *
	 *  intr is active/low negative edge by default
	 *
	 *  -1  mean none assume use polling
	 *  @warning if the dev tree and intr gpio is require please adapt code
	 */
	int intr_gpio;

	/*!< device boot i2c register address
	 *
	 * boot_reg is the value of device i2c address after it is bring out
	 * of reset.
	 */
	int boot_reg;

	/*!< is set if above irq gpio got acquired */
	struct i2d_data_flags_t {
		unsigned pwr_owned:1; /*!< set if pwren gpio is owned*/
		unsigned xsdn_owned:1; /*!< set if sxdn  gpio is owned*/
		unsigned intr_owned:1; /*!< set if intr  gpio is owned*/
		unsigned vio_owned:1; /*!< set if vio gpio is owned*/
		unsigned intr_started:1; /*!< set if irq is handle  */
	} io_flag;

	/** the irq vectore assigned to gpio
	 * -1 if no irq handle
	 */
	int irq;

	struct msgtctrl_t {
		unsigned unhandled_irq_vec:1;
	} msg_flag;

	/* pwren regulator */
	struct regulator *pwren_supply;
};

struct shared_i2c_data {
	struct kref refcount;
	/* pin control */
	struct pinctrl *pinctrl;
};

int stmvl53l1_init_i2c(void);
void __exit stmvl53l1_exit_i2c(void *arg);
int stmvl53l1_power_up_i2c(void *arg);
int stmvl53l1_power_down_i2c(void *arg);
void stmvl53l1_pinctrl_set_state(struct device *dev, struct pinctrl *pinctrl,
				 const char *state_str);
int stmvl53l1_reset_release_i2c(void *arg);
int stmvl53l1_reset_hold_i2c(void *arg);
void stmvl53l1_clean_up_i2c(void);
int stmvl53l1_start_intr(void *object, int *poll_mode);
void *stmvl53l1_get(void *arg);
int stmvl53l1_put(void *arg);

#endif /* STMVL53L1_I2C_H */
