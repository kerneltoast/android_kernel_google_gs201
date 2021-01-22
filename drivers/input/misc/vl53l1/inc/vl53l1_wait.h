/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_WAIT_H_
#define _VL53L1_WAIT_H_

#include "vl53l1_platform.h"





VL53L1_Error VL53L1_wait_for_boot_completion(
	VL53L1_DEV Dev);




VL53L1_Error VL53L1_wait_for_firmware_ready(
	VL53L1_DEV Dev);




VL53L1_Error VL53L1_wait_for_range_completion(
	VL53L1_DEV Dev);




VL53L1_Error VL53L1_wait_for_test_completion(
	VL53L1_DEV Dev);






VL53L1_Error VL53L1_is_boot_complete(
	VL53L1_DEV Dev,
	uint8_t *pready);



VL53L1_Error VL53L1_is_firmware_ready(
	VL53L1_DEV Dev,
	uint8_t *pready);




VL53L1_Error VL53L1_is_new_data_ready(
	VL53L1_DEV Dev,
	uint8_t *pready);






VL53L1_Error VL53L1_poll_for_boot_completion(
	VL53L1_DEV      Dev,
	uint32_t        timeout_ms);




VL53L1_Error VL53L1_poll_for_firmware_ready(
	VL53L1_DEV      Dev,
	uint32_t        timeout_ms);




VL53L1_Error VL53L1_poll_for_range_completion(
	VL53L1_DEV   Dev,
	uint32_t     timeout_ms);




#endif /* _VL53L1_WAIT_H_ */
