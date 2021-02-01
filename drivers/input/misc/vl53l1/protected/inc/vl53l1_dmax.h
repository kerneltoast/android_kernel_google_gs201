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




#ifndef _VL53L1_DMAX_H_
#define _VL53L1_DMAX_H_

#include "vl53l1_types.h"
#include "vl53l1_dmax_private_structs.h"
#include "../../inc/vl53l1_hist_structs.h"
#include "../../inc/vl53l1_error_codes.h"




VL53L1_Error VL53L1_f_001(
	uint16_t target_reflectance,
	struct VL53L1_dmax_calibration_data_t *pcal,
	struct VL53L1_hist_gen3_dmax_config_t *pcfg,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_gen3_dmax_private_data_t *pdata,
	int16_t *pambient_dmax_mm);




uint32_t VL53L1_f_002(
	uint32_t events_threshold,
	uint32_t ref_signal_events,
	uint32_t ref_distance_mm,
	uint32_t signal_thresh_sigma);


#endif /* _VL53L1_DMAX_H_ */
