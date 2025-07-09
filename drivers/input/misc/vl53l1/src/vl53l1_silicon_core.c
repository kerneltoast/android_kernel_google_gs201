// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#include "../inc/vl53l1_ll_def.h"
#include "../inc/vl53l1_register_map.h"
#include "../inc/vl53l1_core.h"
#include "../inc/vl53l1_silicon_core.h"

#include "../vl53l1_platform.h"

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_CORE, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_CORE, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_CORE,\
			status, fmt, ##__VA_ARGS__)


VL53L1_Error VL53L1_is_firmware_ready_silicon(
	VL53L1_DEV Dev,
	uint8_t *pready)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;
	struct VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);

	uint8_t comms_buffer[5];

	LOG_FUNCTION_START("");



	status = VL53L1_ReadMulti(
				Dev,
				VL53L1_INTERRUPT_MANAGER__ENABLES,
				comms_buffer,
				5);

	if (status != VL53L1_ERROR_NONE)
		goto ENDFUNC;

	pdev->dbg_results.interrupt_manager__enables =
			comms_buffer[0];
	pdev->dbg_results.interrupt_manager__clear =
			comms_buffer[1];
	pdev->dbg_results.interrupt_manager__status =
			comms_buffer[2];
	pdev->dbg_results.mcu_to_host_bank__wr_access_en =
			comms_buffer[3];
	pdev->dbg_results.power_management__go1_reset_status =
			comms_buffer[4];

	if ((pdev->sys_ctrl.power_management__go1_power_force & 0x01)
			== 0x01) {

		if (((pdev->dbg_results.interrupt_manager__enables &
				0x1F) == 0x1F) &&
			((pdev->dbg_results.interrupt_manager__clear
					& 0x1F) == 0x1F))
			*pready = 0x01;
		else
			*pready = 0x00;

	} else {


		if ((pdev->dbg_results.power_management__go1_reset_status
				& 0x01) == 0x00)
			*pready = 0x01;
		else
			*pready = 0x00;
	}


ENDFUNC:
	LOG_FUNCTION_END(status);

	return status;
}
