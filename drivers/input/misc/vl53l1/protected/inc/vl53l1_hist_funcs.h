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





#ifndef _VL53L1_HIST_FUNCS_H_
#define _VL53L1_HIST_FUNCS_H_

#include "vl53l1_types.h"
#include "../../inc/vl53l1_ll_def.h"





VL53L1_Error VL53L1_hist_process_data(
	struct VL53L1_dmax_calibration_data_t *pdmax_cal,
	struct VL53L1_hist_gen3_dmax_config_t *pdmax_cfg,
	struct VL53L1_hist_post_process_config_t *ppost_cfg,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_xtalk_histogram_data_t *pxtalk,
	uint8_t *pArea1,
	uint8_t *pArea2,
	struct VL53L1_range_results_t *presults,
	uint8_t *HistMergeNumber);




VL53L1_Error VL53L1_hist_ambient_dmax(
	uint16_t target_reflectance,
	struct VL53L1_dmax_calibration_data_t *pdmax_cal,
	struct VL53L1_hist_gen3_dmax_config_t *pdmax_cfg,
	struct VL53L1_histogram_bin_data_t *pbins,
	int16_t *pambient_dmax_mm);



#endif /* _VL53L1_HIST_FUNCS_H_ */
