// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#include <stdio.h>
#include <stdlib.h>

#include "../inc/vl53l1_core.h"
#include "../inc/vl53l1_register_settings.h"
#include "../inc/vl53l1_hist_char.h"

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_HISTOGRAM, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_HISTOGRAM, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_HISTOGRAM,\
		status, fmt, ##__VA_ARGS__)


VL53L1_Error VL53L1_set_calib_config(
	VL53L1_DEV Dev,
	uint8_t vcsel_delay__a0,
	uint8_t calib_1,
	uint8_t calib_2,
	uint8_t calib_3,
	uint8_t calib_2__a0,
	uint8_t spad_readout)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t comms_buffer[3];

	LOG_FUNCTION_START("");



	status = VL53L1_enable_powerforce(Dev);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_disable_firmware(Dev);




	if (status == VL53L1_ERROR_NONE) {
		status = VL53L1_WrByte(
					Dev,
					VL53L1_RANGING_CORE__VCSEL_DELAY__A0,
					vcsel_delay__a0);
	}



	if (status == VL53L1_ERROR_NONE) {


		comms_buffer[0] = calib_1;
		comms_buffer[1] = calib_2;
		comms_buffer[2] = calib_3;

		status = VL53L1_WriteMulti(
					Dev,
					VL53L1_RANGING_CORE__CALIB_1,
					comms_buffer,
					3);
	}



	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_WrByte(
					Dev,
					VL53L1_RANGING_CORE__CALIB_2__A0,
					calib_2__a0);



	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_WrByte(
					Dev,
					VL53L1_RANGING_CORE__SPAD_READOUT,
					spad_readout);



	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_enable_firmware(Dev);

	LOG_FUNCTION_END(status);

	return status;
}



VL53L1_Error VL53L1_set_hist_calib_pulse_delay(
	VL53L1_DEV Dev,
	uint8_t calib_delay)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;

	LOG_FUNCTION_START("");

	status =
		VL53L1_set_calib_config(
			Dev,
			0x01,
			calib_delay,
			0x04,
			0x08,
			0x14,
			VL53L1_RANGING_CORE__SPAD_READOUT__CALIB_PULSES);

	LOG_FUNCTION_END(status);

	return status;
}


VL53L1_Error VL53L1_disable_calib_pulse_delay(
	VL53L1_DEV Dev)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;

	LOG_FUNCTION_START("");

	status =
		VL53L1_set_calib_config(
			Dev,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			VL53L1_RANGING_CORE__SPAD_READOUT__STANDARD);

	LOG_FUNCTION_END(status);

	return status;
}
