/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_CORE_H_
#define _VL53L1_CORE_H_

#include "vl53l1_platform.h"
#include "vl53l1_core_support.h"

void VL53L1_init_version(
	VL53L1_DEV Dev);




void VL53L1_init_ll_driver_state(
	VL53L1_DEV Dev,
	VL53L1_DeviceState ll_state);




VL53L1_Error VL53L1_update_ll_driver_rd_state(
	VL53L1_DEV Dev);




VL53L1_Error VL53L1_check_ll_driver_rd_state(
	VL53L1_DEV Dev);




VL53L1_Error VL53L1_update_ll_driver_cfg_state(
	VL53L1_DEV Dev);




void VL53L1_copy_rtn_good_spads_to_buffer(
	struct VL53L1_nvm_copy_data_t *pdata,
	uint8_t *pbuffer);




void VL53L1_init_system_results(
	struct VL53L1_system_results_t *pdata);




void V53L1_init_zone_results_structure(
	uint8_t active_zones,
	struct VL53L1_zone_results_t *pdata);




void V53L1_init_zone_dss_configs(
	VL53L1_DEV Dev);




void VL53L1_init_histogram_config_structure(
	uint8_t even_bin0,
	uint8_t even_bin1,
	uint8_t even_bin2,
	uint8_t even_bin3,
	uint8_t even_bin4,
	uint8_t even_bin5,
	uint8_t odd_bin0,
	uint8_t odd_bin1,
	uint8_t odd_bin2,
	uint8_t odd_bin3,
	uint8_t odd_bin4,
	uint8_t odd_bin5,
	struct VL53L1_histogram_config_t *pdata);



void VL53L1_init_histogram_multizone_config_structure(
	uint8_t even_bin0,
	uint8_t even_bin1,
	uint8_t even_bin2,
	uint8_t even_bin3,
	uint8_t even_bin4,
	uint8_t even_bin5,
	uint8_t odd_bin0,
	uint8_t odd_bin1,
	uint8_t odd_bin2,
	uint8_t odd_bin3,
	uint8_t odd_bin4,
	uint8_t odd_bin5,
	struct VL53L1_histogram_config_t *pdata);




void VL53L1_init_xtalk_bin_data_struct(
	uint32_t bin_value,
	uint16_t VL53L1_p_024,
	struct VL53L1_xtalk_histogram_shape_t *pdata);




void VL53L1_i2c_encode_uint16_t(
	uint16_t ip_value,
	uint16_t count,
	uint8_t *pbuffer);




uint16_t VL53L1_i2c_decode_uint16_t(
	uint16_t count,
	uint8_t *pbuffer);




void VL53L1_i2c_encode_int16_t(
	int16_t ip_value,
	uint16_t count,
	uint8_t *pbuffer);




int16_t VL53L1_i2c_decode_int16_t(
	uint16_t count,
	uint8_t *pbuffer);




void VL53L1_i2c_encode_uint32_t(
	uint32_t ip_value,
	uint16_t count,
	uint8_t *pbuffer);




uint32_t VL53L1_i2c_decode_uint32_t(
	uint16_t count,
	uint8_t *pbuffer);




uint32_t VL53L1_i2c_decode_with_mask(
	uint16_t count,
	uint8_t *pbuffer,
	uint32_t bit_mask,
	uint32_t down_shift,
	uint32_t offset);




void VL53L1_i2c_encode_int32_t(
	int32_t ip_value,
	uint16_t count,
	uint8_t *pbuffer);




int32_t VL53L1_i2c_decode_int32_t(
	uint16_t count,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_start_test(
	VL53L1_DEV Dev,
	uint8_t test_mode__ctrl);




VL53L1_Error VL53L1_set_firmware_enable_register(
	VL53L1_DEV Dev,
	uint8_t value);




VL53L1_Error VL53L1_enable_firmware(
	VL53L1_DEV Dev);




VL53L1_Error VL53L1_disable_firmware(
	VL53L1_DEV Dev);




VL53L1_Error VL53L1_set_powerforce_register(
	VL53L1_DEV Dev,
	uint8_t value);





VL53L1_Error VL53L1_enable_powerforce(
	VL53L1_DEV Dev);



VL53L1_Error VL53L1_disable_powerforce(
	VL53L1_DEV Dev);





VL53L1_Error VL53L1_clear_interrupt(
	VL53L1_DEV Dev);





VL53L1_Error VL53L1_force_shadow_stream_count_to_zero(
	VL53L1_DEV Dev);




uint32_t VL53L1_calc_macro_period_us(
	uint16_t fast_osc_frequency,
	uint8_t VL53L1_p_009);




uint16_t VL53L1_calc_range_ignore_threshold(
	uint32_t central_rate,
	int16_t x_gradient,
	int16_t y_gradient,
	uint8_t rate_mult);




uint32_t VL53L1_calc_timeout_mclks(
	uint32_t timeout_us,
	uint32_t macro_period_us);



uint16_t VL53L1_calc_encoded_timeout(
	uint32_t timeout_us,
	uint32_t macro_period_us);




uint32_t VL53L1_calc_timeout_us(
	uint32_t timeout_mclks,
	uint32_t macro_period_us);



uint32_t VL53L1_calc_decoded_timeout_us(
	uint16_t timeout_encoded,
	uint32_t macro_period_us);




uint16_t VL53L1_encode_timeout(
	uint32_t timeout_mclks);




uint32_t VL53L1_decode_timeout(
	uint16_t encoded_timeout);




VL53L1_Error VL53L1_calc_timeout_register_values(
	uint32_t phasecal_config_timeout_us,
	uint32_t mm_config_timeout_us,
	uint32_t range_config_timeout_us,
	uint16_t fast_osc_frequency,
	struct VL53L1_general_config_t *pgeneral,
	struct VL53L1_timing_config_t *ptiming);




uint8_t VL53L1_encode_vcsel_period(
	uint8_t VL53L1_p_031);




uint32_t VL53L1_decode_unsigned_integer(
	uint8_t *pbuffer,
	uint8_t no_of_bytes);




void VL53L1_encode_unsigned_integer(
	uint32_t ip_value,
	uint8_t no_of_bytes,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_hist_copy_and_scale_ambient_info(
	struct VL53L1_zone_hist_info_t *pidata,
	struct VL53L1_histogram_bin_data_t *podata);




void VL53L1_hist_get_bin_sequence_config(
	VL53L1_DEV Dev,
	struct VL53L1_histogram_bin_data_t *pdata);




VL53L1_Error VL53L1_hist_phase_consistency_check(
	VL53L1_DEV Dev,
	struct VL53L1_zone_hist_info_t *phist_prev,
	struct VL53L1_zone_objects_t *prange_prev,
	struct VL53L1_range_results_t *prange_curr);







VL53L1_Error VL53L1_hist_events_consistency_check(
	uint8_t event_sigma,
	uint16_t min_effective_spad_count,
	struct VL53L1_zone_hist_info_t *phist_prev,
	struct VL53L1_object_data_t *prange_prev,
	struct VL53L1_range_data_t *prange_curr,
	int32_t *pevents_tolerance,
	int32_t *pevents_delta,
	VL53L1_DeviceError *prange_status);







VL53L1_Error VL53L1_hist_merged_pulse_check(
	int16_t min_max_tolerance_mm,
	struct VL53L1_range_data_t *pdata,
	VL53L1_DeviceError *prange_status);






VL53L1_Error VL53L1_hist_xmonitor_consistency_check(
	VL53L1_DEV Dev,
	struct VL53L1_zone_hist_info_t *phist_prev,
	struct VL53L1_zone_objects_t *prange_prev,
	struct VL53L1_range_data_t *prange_curr);






VL53L1_Error VL53L1_hist_wrap_dmax(
	struct VL53L1_hist_post_process_config_t *phistpostprocess,
	struct VL53L1_histogram_bin_data_t *pcurrent,
	int16_t *pwrap_dmax_mm);




void VL53L1_hist_combine_mm1_mm2_offsets(
	int16_t mm1_offset_mm,
	int16_t mm2_offset_mm,
	uint8_t encoded_mm_roi_centre,
	uint8_t encoded_mm_roi_size,
	uint8_t encoded_zone_centre,
	uint8_t encoded_zone_size,
	struct VL53L1_additional_offset_cal_data_t *pcal_data,
	uint8_t *pgood_spads,
	uint16_t aperture_attenuation,
	int16_t *prange_offset_mm);




VL53L1_Error VL53L1_hist_xtalk_extract_calc_window(
	int16_t target_distance_mm,
	uint16_t target_width_oversize,
	struct VL53L1_histogram_bin_data_t *phist_bins,
	struct VL53L1_hist_xtalk_extract_data_t *pxtalk_data);




VL53L1_Error VL53L1_hist_xtalk_extract_calc_event_sums(
	struct VL53L1_histogram_bin_data_t *phist_bins,
	struct VL53L1_hist_xtalk_extract_data_t *pxtalk_data);




VL53L1_Error VL53L1_hist_xtalk_extract_calc_rate_per_spad(
	struct VL53L1_hist_xtalk_extract_data_t *pxtalk_data);



VL53L1_Error VL53L1_hist_xtalk_extract_calc_shape(
	struct VL53L1_hist_xtalk_extract_data_t *pxtalk_data,
	struct VL53L1_xtalk_histogram_shape_t *pxtalk_shape);



VL53L1_Error VL53L1_hist_xtalk_shape_model(
	uint16_t events_per_bin,
	uint16_t pulse_centre,
	uint16_t pulse_width,
	struct VL53L1_xtalk_histogram_shape_t *pxtalk_shape);




uint16_t VL53L1_hist_xtalk_shape_model_interp(
	uint16_t events_per_bin,
	uint32_t phase_delta);




void VL53L1_spad_number_to_byte_bit_index(
	uint8_t spad_number,
	uint8_t *pbyte_index,
	uint8_t *pbit_index,
	uint8_t *pbit_mask);




void VL53L1_encode_row_col(
	uint8_t row,
	uint8_t col,
	uint8_t *pspad_number);




void VL53L1_decode_zone_size(
	uint8_t encoded_xy_size,
	uint8_t *pwidth,
	uint8_t *pheight);




void VL53L1_encode_zone_size(
	uint8_t width,
	uint8_t height,
	uint8_t *pencoded_xy_size);




void VL53L1_decode_zone_limits(
	uint8_t encoded_xy_centre,
	uint8_t encoded_xy_size,
	int16_t *px_ll,
	int16_t *py_ll,
	int16_t *px_ur,
	int16_t *py_ur);




uint8_t VL53L1_is_aperture_location(
	uint8_t row,
	uint8_t col);




void VL53L1_calc_max_effective_spads(
	uint8_t encoded_zone_centre,
	uint8_t encoded_zone_size,
	uint8_t *pgood_spads,
	uint16_t aperture_attenuation,
	uint16_t *pmax_effective_spads);




void VL53L1_calc_mm_effective_spads(
	uint8_t encoded_mm_roi_centre,
	uint8_t encoded_mm_roi_size,
	uint8_t encoded_zone_centre,
	uint8_t encoded_zone_size,
	uint8_t *pgood_spads,
	uint16_t aperture_attenuation,
	uint16_t *pmm_inner_effective_spads,
	uint16_t *pmm_outer_effective_spads);




void VL53L1_hist_copy_results_to_sys_and_core(
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_range_results_t *phist,
	struct VL53L1_system_results_t *psys,
	struct VL53L1_core_results_t *pcore);




VL53L1_Error VL53L1_sum_histogram_data(
		struct VL53L1_histogram_bin_data_t *phist_input,
		struct VL53L1_histogram_bin_data_t *phist_output);




VL53L1_Error VL53L1_avg_histogram_data(
		uint8_t no_of_samples,
		struct VL53L1_histogram_bin_data_t *phist_sum,
		struct VL53L1_histogram_bin_data_t *phist_avg);




VL53L1_Error VL53L1_save_cfg_data(
	VL53L1_DEV Dev);




VL53L1_Error VL53L1_dynamic_zone_update(
	VL53L1_DEV Dev,
	struct VL53L1_range_results_t *presults);




VL53L1_Error VL53L1_update_internal_stream_counters(
	VL53L1_DEV Dev,
	uint8_t external_stream_count,
	uint8_t *pinternal_stream_count,
	uint8_t *pinternal_stream_count_val);



VL53L1_Error VL53L1_multizone_hist_bins_update(
	VL53L1_DEV Dev);



VL53L1_Error VL53L1_set_histogram_multizone_initial_bin_config(
	struct VL53L1_zone_config_t *pzone_cfg,
	struct VL53L1_histogram_config_t *phist_cfg,
	struct VL53L1_histogram_config_t *pmulti_hist);



uint8_t	VL53L1_encode_GPIO_interrupt_config(
	struct VL53L1_GPIO_interrupt_config_t *pintconf);



struct VL53L1_GPIO_interrupt_config_t VL53L1_decode_GPIO_interrupt_config(
	uint8_t system__interrupt_config);



VL53L1_Error VL53L1_set_GPIO_distance_threshold(
	VL53L1_DEV Dev,
	uint16_t threshold_high,
	uint16_t threshold_low);



VL53L1_Error VL53L1_set_GPIO_rate_threshold(
	VL53L1_DEV Dev,
	uint16_t threshold_high,
	uint16_t threshold_low);



VL53L1_Error VL53L1_set_GPIO_thresholds_from_struct(
	VL53L1_DEV Dev,
	struct VL53L1_GPIO_interrupt_config_t *pintconf);





VL53L1_Error VL53L1_set_ref_spad_char_config(
	VL53L1_DEV Dev,
	uint8_t vcsel_period_a,
	uint32_t phasecal_timeout_us,
	uint16_t total_rate_target_mcps,
	uint16_t max_count_rate_rtn_limit_mcps,
	uint16_t min_count_rate_rtn_limit_mcps,
	uint16_t fast_osc_frequency);




VL53L1_Error VL53L1_set_ssc_config(
	VL53L1_DEV Dev,
	struct VL53L1_ssc_config_t *pssc_cfg,
	uint16_t fast_osc_frequency);




VL53L1_Error VL53L1_get_spad_rate_data(
	VL53L1_DEV Dev,
	struct VL53L1_spad_rate_data_t *pspad_rates);



uint32_t VL53L1_calc_crosstalk_plane_offset_with_margin(
		uint32_t plane_offset_kcps,
		int16_t margin_offset_kcps);



VL53L1_Error VL53L1_low_power_auto_data_init(
	VL53L1_DEV Dev);



VL53L1_Error VL53L1_low_power_auto_data_stop_range(
	VL53L1_DEV Dev
	);




VL53L1_Error VL53L1_dynamic_xtalk_correction_calc_required_samples(
	VL53L1_DEV Dev);



VL53L1_Error VL53L1_dynamic_xtalk_correction_calc_new_xtalk(
	VL53L1_DEV Dev,
	uint32_t xtalk_offset_out,
	struct VL53L1_smudge_corrector_config_t *pconfig,
	struct VL53L1_smudge_corrector_data_t *pout,
	uint8_t add_smudge,
	uint8_t soft_update);



VL53L1_Error VL53L1_dynamic_xtalk_correction_corrector(
	VL53L1_DEV Dev);



VL53L1_Error VL53L1_dynamic_xtalk_correction_data_init(
	VL53L1_DEV Dev);



VL53L1_Error VL53L1_dynamic_xtalk_correction_output_init(
	struct VL53L1_LLDriverResults_t *pres);



VL53L1_Error VL53L1_xtalk_cal_data_init(
	VL53L1_DEV Dev);



VL53L1_Error VL53L1_config_low_power_auto_mode(
	struct VL53L1_general_config_t *pgeneral,
	struct VL53L1_dynamic_config_t *pdynamic,
	struct VL53L1_low_power_auto_data_t *plpadata);



VL53L1_Error VL53L1_low_power_auto_setup_manual_calibration(
	VL53L1_DEV Dev);



VL53L1_Error VL53L1_low_power_auto_update_DSS(
	VL53L1_DEV Dev);


VL53L1_Error VL53L1_compute_histo_merge_nb(
	VL53L1_DEV Dev, uint8_t *histo_merge_nb);

#endif /* _VL53L1_CORE_H_ */
