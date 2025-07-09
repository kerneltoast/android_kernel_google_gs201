/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_CORE_SUPPORT_H_
#define _VL53L1_CORE_SUPPORT_H_

#include "vl53l1_types.h"
#include "vl53l1_hist_structs.h"





uint32_t VL53L1_calc_pll_period_us(
	uint16_t fast_osc_frequency);





uint32_t VL53L1_duration_maths(
	uint32_t pll_period_us,
	uint32_t vcsel_parm_pclks,
	uint32_t window_vclks,
	uint32_t periods_elapsed_mclks);



uint32_t VL53L1_events_per_spad_maths(
	int32_t VL53L1_p_013,
	uint16_t num_spads,
	uint32_t duration);




uint32_t VL53L1_isqrt(
	uint32_t num);




void VL53L1_hist_calc_zero_distance_phase(
	struct VL53L1_histogram_bin_data_t *pdata);




void VL53L1_hist_estimate_ambient_from_thresholded_bins(
	int32_t ambient_threshold_sigma,
	struct VL53L1_histogram_bin_data_t *pdata);




void VL53L1_hist_remove_ambient_bins(
	struct VL53L1_histogram_bin_data_t *pdata);




uint32_t VL53L1_calc_pll_period_mm(
	uint16_t fast_osc_frequency);




uint16_t VL53L1_rate_maths(
	int32_t VL53L1_p_008,
	uint32_t time_us);




uint16_t VL53L1_rate_per_spad_maths(
	uint32_t frac_bits,
	uint32_t peak_count_rate,
	uint16_t num_spads,
	uint32_t max_output_value);




int32_t VL53L1_range_maths(
	uint16_t fast_osc_frequency,
	uint16_t VL53L1_p_017,
	uint16_t zero_distance_phase,
	uint8_t fractional_bits,
	int32_t gain_factor,
	int32_t range_offset_mm);




uint8_t VL53L1_decode_vcsel_period(
	uint8_t vcsel_period_reg);



void VL53L1_copy_xtalk_bin_data_to_histogram_data_struct(
		struct VL53L1_xtalk_histogram_shape_t *pxtalk,
		struct VL53L1_histogram_bin_data_t *phist);




void VL53L1_init_histogram_bin_data_struct(
	int32_t bin_value,
	uint16_t VL53L1_p_024,
	struct VL53L1_histogram_bin_data_t *pdata);




void VL53L1_decode_row_col(
	uint8_t spad_number,
	uint8_t *prow,
	uint8_t *pcol);




void VL53L1_hist_find_min_max_bin_values(
	struct VL53L1_histogram_bin_data_t *pdata);




void VL53L1_hist_estimate_ambient_from_ambient_bins(
	struct VL53L1_histogram_bin_data_t *pdata);

#endif /* _VL53L1_CORE_SUPPORT_H_ */
