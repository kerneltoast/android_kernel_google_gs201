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





#ifndef _VL53L1_HIST_CORE_H_
#define _VL53L1_HIST_CORE_H_

#include "vl53l1_types.h"
#include "vl53l1_hist_private_structs.h"
#include "../../inc/vl53l1_ll_def.h"




void VL53L1_f_013(
	uint8_t VL53L1_p_018,
	uint8_t filter_woi,
	struct VL53L1_histogram_bin_data_t *pbins,
	int32_t *pa,
	int32_t *pb,
	int32_t *pc);




VL53L1_Error VL53L1_f_011(
	uint16_t vcsel_width,
	uint16_t fast_osc_frequency,
	uint32_t total_periods_elapsed,
	uint16_t VL53L1_p_006,
	struct VL53L1_range_data_t *pdata,
	uint8_t histo_merge_nb);




void VL53L1_f_012(
	uint16_t gain_factor,
	int16_t range_offset_mm,
	struct VL53L1_range_data_t *pdata);




void VL53L1_f_037(
	struct VL53L1_histogram_bin_data_t *pdata,
	int32_t ambient_estimate_counts_per_bin);




void VL53L1_f_004(
	struct VL53L1_histogram_bin_data_t *pxtalk,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_histogram_bin_data_t *pxtalk_realigned);



int8_t VL53L1_f_038(
	struct VL53L1_histogram_bin_data_t *pdata1,
	struct VL53L1_histogram_bin_data_t *pdata2);



VL53L1_Error VL53L1_f_039(
	struct VL53L1_histogram_bin_data_t *pidata,
	struct VL53L1_histogram_bin_data_t *podata);


#endif /* _VL53L1_HIST_CORE_H_ */
