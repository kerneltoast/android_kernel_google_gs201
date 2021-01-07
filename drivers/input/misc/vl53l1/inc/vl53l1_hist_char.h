/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#ifndef _VL53L1_HIST_CHAR_H_
#define _VL53L1_HIST_CHAR_H_

#include "vl53l1_platform.h"





VL53L1_Error VL53L1_set_calib_config(
	VL53L1_DEV Dev,
	uint8_t vcsel_delay__a0,
	uint8_t calib_1,
	uint8_t calib_2,
	uint8_t calib_3,
	uint8_t calib_2__a0,
	uint8_t spad_readout);




VL53L1_Error VL53L1_set_hist_calib_pulse_delay(
	VL53L1_DEV Dev,
	uint8_t calib_delay);




VL53L1_Error VL53L1_disable_calib_pulse_delay(
	VL53L1_DEV Dev);



#endif /* _VL53L1_HIST_CHAR_H_ */
