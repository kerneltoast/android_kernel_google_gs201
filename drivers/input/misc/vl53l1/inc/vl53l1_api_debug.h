/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_API_DEBUG_H_
#define _VL53L1_API_DEBUG_H_

#include "vl53l1_platform.h"
#include "vl53l1_nvm_structs.h"






VL53L1_Error VL53L1_decode_calibration_data_buffer(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_calibration_data_t *pdata);






VL53L1_Error VL53L1_get_nvm_debug_data(
	VL53L1_DEV Dev,
	struct VL53L1_decoded_nvm_data_t *pdata);



VL53L1_Error VL53L1_get_histogram_debug_data(
	VL53L1_DEV Dev,
	struct VL53L1_histogram_bin_data_t *pdata);






VL53L1_Error VL53L1_get_additional_data(
	VL53L1_DEV Dev,
	struct VL53L1_additional_data_t *pdata);






VL53L1_Error VL53L1_get_xtalk_debug_data(
	VL53L1_DEV Dev,
	struct VL53L1_xtalk_debug_data_t *pdata);




VL53L1_Error VL53L1_get_offset_debug_data(
	VL53L1_DEV Dev,
	struct VL53L1_offset_debug_data_t *pdata);

#ifdef VL53L1_LOG_ENABLE



void VL53L1_signed_fixed_point_sprintf(
	int32_t fp_value,
	uint8_t frac_bits,
	uint16_t buf_size,
	char *pbuffer);




void VL53L1_print_static_nvm_managed(
	struct VL53L1_static_nvm_managed_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_customer_nvm_managed(
	struct VL53L1_customer_nvm_managed_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_nvm_copy_data(
	struct VL53L1_nvm_copy_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_histogram_bin_data(
	struct VL53L1_histogram_bin_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_xtalk_histogram_data(
	struct VL53L1_xtalk_histogram_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_xtalk_histogram_shape_data(
	struct VL53L1_xtalk_histogram_shape_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_range_results(
	struct VL53L1_range_results_t *pdata,
	char *pprefix,
	uint32_t trace_flags);



void VL53L1_print_range_data(
	struct VL53L1_range_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_offset_range_results(
	struct VL53L1_offset_range_results_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_offset_range_data(
	struct VL53L1_offset_range_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_cal_peak_rate_map(
	struct VL53L1_cal_peak_rate_map_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_additional_offset_cal_data(
	struct VL53L1_additional_offset_cal_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);



void VL53L1_print_additional_data(
	struct VL53L1_additional_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_gain_calibration_data(
	struct VL53L1_gain_calibration_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_zone_calibration_data(
	struct VL53L1_zone_calibration_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_zone_calibration_results(
	struct VL53L1_zone_calibration_results_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_xtalk_range_results(
	struct VL53L1_xtalk_range_results_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_xtalk_range_data(
	struct VL53L1_xtalk_range_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_xtalk_calibration_results(
	struct VL53L1_xtalk_calibration_results_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_xtalk_config(
	struct VL53L1_xtalk_config_t *pdata,
	char *pprefix,
	uint32_t trace_flags);



void VL53L1_print_xtalk_extract_config(
	struct VL53L1_xtalkextract_config_t *pdata,
	char *pprefix,
	uint32_t trace_flags);



void VL53L1_print_zone_cal_config(
	struct VL53L1_zonecal_config_t *pdata,
	char *pprefix,
	uint32_t trace_flags);



void VL53L1_print_offset_cal_config(
	struct VL53L1_offsetcal_config_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_dmax_calibration_data(
	struct VL53L1_dmax_calibration_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_calibration_data(
	struct VL53L1_calibration_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_xtalk_debug_data(
	struct VL53L1_xtalk_debug_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);



void VL53L1_print_offset_debug_data(
	struct VL53L1_offset_debug_data_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_optical_centre(
	struct VL53L1_optical_centre_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_user_zone(
	struct VL53L1_user_zone_t *pdata,
	char *pprefix,
	uint32_t trace_flags);



void VL53L1_print_zone_config(
	struct VL53L1_zone_config_t *pdata,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_spad_rate_data(
	struct VL53L1_spad_rate_data_t *pspad_rates,
	char *pprefix,
	uint32_t trace_flags);




void VL53L1_print_spad_rate_map(
	struct VL53L1_spad_rate_data_t *pspad_rates,
	char *pprefix,
	uint32_t trace_flags);


#endif

#endif /* _VL53L1_API_DEBUG_H_ */
