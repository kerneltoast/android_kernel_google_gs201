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






#ifndef _VL53L1_XTALK_PRIVATE_STRUCTS_H_
#define _VL53L1_XTALK_PRIVATE_STRUCTS_H_

#include "vl53l1_types.h"
#include "../../inc/vl53l1_hist_structs.h"



#define VL53L1_D_012 4



struct VL53L1_xtalk_algo_data_t {
	uint32_t VL53L1_p_062[VL53L1_D_012];
	int16_t VL53L1_p_060;
	int16_t VL53L1_p_061;
	struct VL53L1_histogram_bin_data_t VL53L1_p_057;
	struct VL53L1_histogram_bin_data_t VL53L1_p_058;
	uint32_t VL53L1_p_059;
	uint32_t VL53L1_p_063[VL53L1_XTALK_HISTO_BINS];
};


#endif /* _VL53L1_XTALK_PRIVATE_STRUCTS_H_ */
