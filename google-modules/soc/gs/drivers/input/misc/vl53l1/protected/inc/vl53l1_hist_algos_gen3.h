/* SPDX-License-Identifier: BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 Protected and is dual licensed,
 either 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

 ******************************************************************************

 'STMicroelectronics Proprietary license'

 ******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 ******************************************************************************
 */





#ifndef _VL53L1_HIST_ALGOS_GEN3_H_
#define _VL53L1_HIST_ALGOS_GEN3_H_

#include "vl53l1_types.h"

#include "vl53l1_hist_private_structs.h"
#include "vl53l1_dmax_private_structs.h"
#include "../../inc/vl53l1_ll_def.h"




void VL53L1_f_016(
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);






VL53L1_Error VL53L1_f_018(
	uint16_t ambient_threshold_events_scaler,
	int32_t ambient_threshold_sigma,
	int32_t min_ambient_threshold_events,
	uint8_t algo__crosstalk_compensation_enable,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_histogram_bin_data_t *pxtalk,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);






VL53L1_Error VL53L1_f_019(
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);




VL53L1_Error VL53L1_f_020(
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);




VL53L1_Error VL53L1_f_021(
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);




VL53L1_Error VL53L1_f_028(
	VL53L1_HistTargetOrder target_order,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);




VL53L1_Error VL53L1_f_022(
	uint8_t pulse_no,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);



VL53L1_Error VL53L1_f_027(
	uint8_t pulse_no,
	uint8_t clip_events,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);




VL53L1_Error VL53L1_f_030(
	int16_t VL53L1_p_022,
	int16_t VL53L1_p_026,
	uint8_t VL53L1_p_031,
	uint8_t clip_events,
	struct VL53L1_histogram_bin_data_t *pbins,
	uint32_t *pphase);




VL53L1_Error VL53L1_f_023(
	uint8_t pulse_no,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo,
	int32_t pad_value,
	struct VL53L1_histogram_bin_data_t *ppulse);




VL53L1_Error VL53L1_f_026(
	uint8_t bin,
	uint8_t sigma_estimator__sigma_ref_mm,
	uint8_t VL53L1_p_031,
	uint8_t VL53L1_p_055,
	uint8_t crosstalk_compensation_enable,
	struct VL53L1_histogram_bin_data_t *phist_data_ap,
	struct VL53L1_histogram_bin_data_t *phist_data_zp,
	struct VL53L1_histogram_bin_data_t *pxtalk_hist,
	uint16_t *psigma_est);




void VL53L1_f_029(
	uint8_t range_id,
	uint8_t valid_phase_low,
	uint8_t valid_phase_high,
	uint16_t sigma_thres,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_pulse_data_t *ppulse,
	struct VL53L1_range_data_t *pdata);



#endif /* _VL53L1_HIST_ALGOS_GEN3_H_ */
