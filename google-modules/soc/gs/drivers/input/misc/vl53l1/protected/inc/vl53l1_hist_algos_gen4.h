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





#ifndef _VL53L1_HIST_ALGOS_GEN4_H_
#define _VL53L1_HIST_ALGOS_GEN4_H_

#include "vl53l1_types.h"
#include "vl53l1_hist_private_structs.h"
#include "vl53l1_dmax_private_structs.h"
#include "../../inc/vl53l1_ll_def.h"





void VL53L1_f_032(
	struct VL53L1_hist_gen4_algo_filtered_data_t *palgo);




VL53L1_Error VL53L1_f_033(
	struct VL53L1_dmax_calibration_data_t *pdmax_cal,
	struct VL53L1_hist_gen3_dmax_config_t *pdmax_cfg,
	struct VL53L1_hist_post_process_config_t *ppost_cfg,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_histogram_bin_data_t *pxtalk,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo,
	struct VL53L1_hist_gen4_algo_filtered_data_t *pfiltered,
	struct VL53L1_hist_gen3_dmax_private_data_t *pdmax_algo,
	struct VL53L1_range_results_t *presults,
	uint8_t histo_merge_nb);





VL53L1_Error VL53L1_f_034(
	uint8_t pulse_no,
	struct VL53L1_histogram_bin_data_t *ppulse,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo,
	struct VL53L1_hist_gen4_algo_filtered_data_t *pfiltered);




VL53L1_Error VL53L1_f_035(
	uint8_t pulse_no,
	uint16_t noise_threshold,
	struct VL53L1_hist_gen4_algo_filtered_data_t *pfiltered,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo);




VL53L1_Error VL53L1_f_036(
	uint8_t bin,
	int32_t VL53L1_p_003,
	int32_t VL53L1_p_018,
	int32_t VL53L1_p_001,
	int32_t ax,
	int32_t bx,
	int32_t cx,
	int32_t VL53L1_p_004,
	uint8_t VL53L1_p_031,
	uint32_t *pmedian_phase);



#endif /* _VL53L1_HIST_ALGOS_GEN4_H_ */
