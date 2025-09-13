/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_ZONE_PRESETS_H_
#define _VL53L1_ZONE_PRESETS_H_

#include "vl53l1_ll_def.h"





VL53L1_Error VL53L1_init_zone_config_structure(
	uint8_t x_off,
	uint8_t x_inc,
	uint8_t x_zones,
	uint8_t y_off,
	uint8_t y_inc,
	uint8_t y_zones,
	uint8_t width,
	uint8_t height,
	struct VL53L1_zone_config_t *pdata);




VL53L1_Error VL53L1_zone_preset_xtalk_planar(
	struct VL53L1_general_config_t *pgeneral,
	struct VL53L1_zone_config_t *pzone_cfg);



VL53L1_Error VL53L1_init_zone_config_histogram_bins(
	struct VL53L1_zone_config_t *pdata);


#endif /* _VL53L1_ZONE_PRESETS_H_ */
