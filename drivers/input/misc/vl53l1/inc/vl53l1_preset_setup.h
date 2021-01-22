/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

#ifndef _VL53L1_PRESET_SETUP_H_
#define _VL53L1_PRESET_SETUP_H_


/* indexes for the bare driver tuning setting API function */
enum VL53L1_Tuning_t {
	VL53L1_TUNING_VERSION = 0,
	VL53L1_TUNING_PROXY_MIN,
	VL53L1_TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM,
	VL53L1_TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER,
	VL53L1_TUNING_MIN_AMBIENT_DMAX_VALID,
	VL53L1_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER,
	VL53L1_TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM,
	VL53L1_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT,
	VL53L1_TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN,
	VL53L1_TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET,
	VL53L1_TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR,
	VL53L1_TUNING_MAX_TUNABLE_KEY
};

/* default values for the tuning settings parameters */
#define TUNING_VERSION 0x0007

#define TUNING_PROXY_MIN -30 /* min distance in mm */
#define TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM 600
/* Target distance in mm for single target Xtalk */
#define TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER 50
/* Number of sample used for single target Xtalk */
#define TUNING_MIN_AMBIENT_DMAX_VALID 8
/* Minimum ambient level to state the Dmax returned by the device is valid */
#define TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER 10
/* Maximum loops to perform simple offset calibration */
#define TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM 600
/* Target distance in mm for target Xtalk from Bins method*/
#define TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT 3
/* Number of loops done during the simple offset calibration*/
#define TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR_DEFAULT 9
/* zero distance offset calibration non linear compensation default value */

/* The following settings are related to the fix for ticket EwokP #558410 */
#define TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN 24
/* Acceptance margin for the xtalk_shape bin_data sum computation */
#define TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET 50
/* Recovery value for Xtalk compensation plane offset in kcps */
/* 50 stands for ~0.10 kcps cover glass in 7.9 format */
/* End of settings related to the fix for ticket EwokP #558410 */


#endif /* _VL53L1_PRESET_SETUP_H_ */
