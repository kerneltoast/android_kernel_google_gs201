// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#include "../inc/vl53l1_ll_def.h"
#include "../inc/vl53l1_ll_device.h"
#include "../inc/vl53l1_zone_presets.h"

#include "../vl53l1_platform_log.h"

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_CORE, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_CORE, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_CORE,\
			status, fmt, ##__VA_ARGS__)

VL53L1_Error VL53L1_init_zone_config_structure(
	uint8_t x_off,
	uint8_t x_inc,
	uint8_t x_zones,
	uint8_t y_off,
	uint8_t y_inc,
	uint8_t y_zones,
	uint8_t width,
	uint8_t height,
	struct VL53L1_zone_config_t *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint8_t x = 0;
	uint8_t y = 0;
	uint16_t i = 0;

	LOG_FUNCTION_START("");

	pdata->max_zones = VL53L1_MAX_USER_ZONES;

	i = 0;

	for (x = 0; x < x_zones; x++) {
		for (y = 0; y < y_zones; y++) {
			if (i < VL53L1_MAX_USER_ZONES) {
				pdata->active_zones = (uint8_t)i;
				pdata->user_zones[i].height = height;
				pdata->user_zones[i].width = width;
				pdata->user_zones[i].x_centre =
						x_off + (x * x_inc);
				pdata->user_zones[i].y_centre =
						y_off + (y * y_inc);
			}

			i++;
		}
	}

	status = VL53L1_init_zone_config_histogram_bins(pdata);

	LOG_FUNCTION_END(status);

	return status;
}

VL53L1_Error VL53L1_zone_preset_xtalk_planar(
	struct VL53L1_general_config_t *pgeneral,
	struct VL53L1_zone_config_t *pzone_cfg)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;

	LOG_FUNCTION_START("");

	pgeneral->global_config__stream_divider = 0x05;

	pzone_cfg->active_zones = 0x04;

	pzone_cfg->user_zones[0].height = 15;
	pzone_cfg->user_zones[0].width = 7;
	pzone_cfg->user_zones[0].x_centre = 4;
	pzone_cfg->user_zones[0].y_centre = 8;

	pzone_cfg->user_zones[1].height = 15;
	pzone_cfg->user_zones[1].width = 7;
	pzone_cfg->user_zones[1].x_centre = 12;
	pzone_cfg->user_zones[1].y_centre = 8;

	pzone_cfg->user_zones[2].height = 7;
	pzone_cfg->user_zones[2].width = 15;
	pzone_cfg->user_zones[2].x_centre = 8;
	pzone_cfg->user_zones[2].y_centre = 4;

	pzone_cfg->user_zones[3].height = 7;
	pzone_cfg->user_zones[3].width = 15;
	pzone_cfg->user_zones[3].x_centre = 8;
	pzone_cfg->user_zones[3].y_centre = 12;

	pzone_cfg->user_zones[4].height = 15;
	pzone_cfg->user_zones[4].width = 15;
	pzone_cfg->user_zones[4].x_centre = 8;
	pzone_cfg->user_zones[4].y_centre = 8;

	status = VL53L1_init_zone_config_histogram_bins(pzone_cfg);

	LOG_FUNCTION_END(status);

	return status;
}

VL53L1_Error VL53L1_init_zone_config_histogram_bins(
	struct VL53L1_zone_config_t *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint8_t i;

	LOG_FUNCTION_START("");

	for (i = 0; i < pdata->max_zones; i++)
		pdata->bin_config[i] = VL53L1_ZONECONFIG_BINCONFIG__LOWAMB;

	LOG_FUNCTION_END(status);

	return status;
}
