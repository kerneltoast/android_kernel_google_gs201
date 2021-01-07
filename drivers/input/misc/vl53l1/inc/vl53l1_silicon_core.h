/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_SILICON_CORE_H_
#define _VL53L1_SILICON_CORE_H_

#include "vl53l1_platform.h"





VL53L1_Error VL53L1_is_firmware_ready_silicon(
	VL53L1_DEV Dev,
	uint8_t *pready);



#endif /* _VL53L1_SILICON_CORE_H_ */
