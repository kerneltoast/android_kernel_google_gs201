/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_API_CALIBRATION_H_
#define _VL53L1_API_CALIBRATION_H_

#include "vl53l1_platform.h"

VL53L1_Error VL53L1_run_ref_spad_char(
	VL53L1_DEV Dev,
	VL53L1_Error *pcal_status);




VL53L1_Error VL53L1_run_device_test(
	VL53L1_DEV Dev,
	VL53L1_DeviceTestMode device_test_mode);




VL53L1_Error VL53L1_run_spad_rate_map(
	VL53L1_DEV Dev,
	VL53L1_DeviceTestMode device_test_mode,
	VL53L1_DeviceSscArray array_select,
	uint32_t ssc_config_timeout_us,
	struct VL53L1_spad_rate_data_t *pspad_rate_data);




VL53L1_Error VL53L1_run_xtalk_extraction(
	VL53L1_DEV Dev,
	VL53L1_Error *pcal_status);



VL53L1_Error VL53L1_get_and_avg_xtalk_samples(
	VL53L1_DEV Dev,
	uint8_t num_of_samples,
	uint8_t measurement_mode,
	int16_t xtalk_filter_thresh_max_mm,
	int16_t xtalk_filter_thresh_min_mm,
	uint16_t xtalk_max_valid_rate_kcps,
	uint8_t xtalk_result_id,
	uint8_t xtalk_histo_id,
	struct VL53L1_xtalk_range_results_t *pxtalk_results,
	struct VL53L1_histogram_bin_data_t *psum_histo,
	struct VL53L1_histogram_bin_data_t *pavg_histo);



VL53L1_Error VL53L1_run_offset_calibration(
	VL53L1_DEV Dev,
	int16_t cal_distance_mm,
	uint16_t cal_reflectance_pc,
	VL53L1_Error *pcal_status);




VL53L1_Error VL53L1_run_phasecal_average(
	VL53L1_DEV Dev,
	uint8_t measurement_mode,
	uint8_t phasecal_result__vcsel_start,
	uint16_t phasecal_num_of_samples,
	struct VL53L1_range_results_t *prange_results,
	uint16_t *pphasecal_result__reference_phase,
	uint16_t *pzero_distance_phase);




VL53L1_Error VL53L1_run_zone_calibration(
	VL53L1_DEV Dev,
	VL53L1_DevicePresetModes device_preset_mode,
	VL53L1_DeviceZonePreset zone_preset,
	struct VL53L1_zone_config_t *pzone_cfg,
	int16_t cal_distance_mm,
	uint16_t cal_reflectance_pc,
	VL53L1_Error *pcal_status);




void VL53L1_hist_xtalk_extract_data_init(
	struct VL53L1_hist_xtalk_extract_data_t *pxtalk_data);



VL53L1_Error VL53L1_hist_xtalk_extract_update(
	int16_t target_distance_mm,
	uint16_t target_width_oversize,
	struct VL53L1_histogram_bin_data_t *phist_bins,
	struct VL53L1_hist_xtalk_extract_data_t *pxtalk_data);



VL53L1_Error VL53L1_hist_xtalk_extract_fini(
	struct VL53L1_histogram_bin_data_t *phist_bins,
	struct VL53L1_hist_xtalk_extract_data_t *pxtalk_data,
	struct VL53L1_xtalk_calibration_results_t *pxtalk_cal,
	struct VL53L1_xtalk_histogram_shape_t *pxtalk_shape);




VL53L1_Error VL53L1_run_hist_xtalk_extraction(
	VL53L1_DEV Dev,
	int16_t cal_distance_mm,
	VL53L1_Error *pcal_status);

#endif /* _VL53L1_API_CALIBRATION_H_ */
