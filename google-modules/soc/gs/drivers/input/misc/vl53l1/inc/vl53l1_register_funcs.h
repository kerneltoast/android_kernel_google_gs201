/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_REGISTER_FUNCS_H_
#define _VL53L1_REGISTER_FUNCS_H_

#include "vl53l1_platform.h"





VL53L1_Error VL53L1_i2c_encode_static_nvm_managed(
	struct VL53L1_static_nvm_managed_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_static_nvm_managed(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_static_nvm_managed_t *pdata);




VL53L1_Error VL53L1_set_static_nvm_managed(
	VL53L1_DEV Dev,
	struct VL53L1_static_nvm_managed_t *pdata);




VL53L1_Error VL53L1_get_static_nvm_managed(
	VL53L1_DEV Dev,
	struct VL53L1_static_nvm_managed_t *pdata);




VL53L1_Error VL53L1_i2c_encode_customer_nvm_managed(
	struct VL53L1_customer_nvm_managed_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_customer_nvm_managed(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_customer_nvm_managed_t *pdata);




VL53L1_Error VL53L1_set_customer_nvm_managed(
	VL53L1_DEV Dev,
	struct VL53L1_customer_nvm_managed_t *pdata);




VL53L1_Error VL53L1_get_customer_nvm_managed(
	VL53L1_DEV Dev,
	struct VL53L1_customer_nvm_managed_t *pdata);




VL53L1_Error VL53L1_i2c_encode_static_config(
	struct VL53L1_static_config_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_static_config(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_static_config_t *pdata);




VL53L1_Error VL53L1_set_static_config(
	VL53L1_DEV Dev,
	struct VL53L1_static_config_t *pdata);




VL53L1_Error VL53L1_get_static_config(
	VL53L1_DEV Dev,
	struct VL53L1_static_config_t *pdata);




VL53L1_Error VL53L1_i2c_encode_general_config(
	struct VL53L1_general_config_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_general_config(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_general_config_t *pdata);




VL53L1_Error VL53L1_set_general_config(
	VL53L1_DEV Dev,
	struct VL53L1_general_config_t *pdata);




VL53L1_Error VL53L1_get_general_config(
	VL53L1_DEV Dev,
	struct VL53L1_general_config_t *pdata);




VL53L1_Error VL53L1_i2c_encode_timing_config(
	struct VL53L1_timing_config_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_timing_config(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_timing_config_t *pdata);




VL53L1_Error VL53L1_set_timing_config(
	VL53L1_DEV Dev,
	struct VL53L1_timing_config_t *pdata);




VL53L1_Error VL53L1_get_timing_config(
	VL53L1_DEV Dev,
	struct VL53L1_timing_config_t *pdata);




VL53L1_Error VL53L1_i2c_encode_dynamic_config(
	struct VL53L1_dynamic_config_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_dynamic_config(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_dynamic_config_t *pdata);




VL53L1_Error VL53L1_set_dynamic_config(
	VL53L1_DEV Dev,
	struct VL53L1_dynamic_config_t *pdata);




VL53L1_Error VL53L1_get_dynamic_config(
	VL53L1_DEV Dev,
	struct VL53L1_dynamic_config_t *pdata);




VL53L1_Error VL53L1_i2c_encode_system_control(
	struct VL53L1_system_control_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_system_control(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_system_control_t *pdata);




VL53L1_Error VL53L1_set_system_control(
	VL53L1_DEV Dev,
	struct VL53L1_system_control_t *pdata);




VL53L1_Error VL53L1_get_system_control(
	VL53L1_DEV Dev,
	struct VL53L1_system_control_t *pdata);




VL53L1_Error VL53L1_i2c_encode_system_results(
	struct VL53L1_system_results_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_system_results(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_system_results_t *pdata);




VL53L1_Error VL53L1_set_system_results(
	VL53L1_DEV Dev,
	struct VL53L1_system_results_t *pdata);




VL53L1_Error VL53L1_get_system_results(
	VL53L1_DEV Dev,
	struct VL53L1_system_results_t *pdata);




VL53L1_Error VL53L1_i2c_encode_core_results(
	struct VL53L1_core_results_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_core_results(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_core_results_t *pdata);




VL53L1_Error VL53L1_set_core_results(
	VL53L1_DEV Dev,
	struct VL53L1_core_results_t *pdata);




VL53L1_Error VL53L1_get_core_results(
	VL53L1_DEV Dev,
	struct VL53L1_core_results_t *pdata);




VL53L1_Error VL53L1_i2c_encode_debug_results(
	struct VL53L1_debug_results_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_debug_results(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_debug_results_t *pdata);




VL53L1_Error VL53L1_set_debug_results(
	VL53L1_DEV Dev,
	struct VL53L1_debug_results_t *pdata);




VL53L1_Error VL53L1_get_debug_results(
	VL53L1_DEV Dev,
	struct VL53L1_debug_results_t *pdata);




VL53L1_Error VL53L1_i2c_encode_nvm_copy_data(
	struct VL53L1_nvm_copy_data_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_nvm_copy_data(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_nvm_copy_data_t *pdata);




VL53L1_Error VL53L1_set_nvm_copy_data(
	VL53L1_DEV Dev,
	struct VL53L1_nvm_copy_data_t *pdata);




VL53L1_Error VL53L1_get_nvm_copy_data(
	VL53L1_DEV Dev,
	struct VL53L1_nvm_copy_data_t *pdata);




VL53L1_Error VL53L1_i2c_encode_prev_shadow_system_results(
	struct VL53L1_prev_shadow_system_results_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_prev_shadow_system_results(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_prev_shadow_system_results_t *pdata);




VL53L1_Error VL53L1_set_prev_shadow_system_results(
	VL53L1_DEV Dev,
	struct VL53L1_prev_shadow_system_results_t *pdata);




VL53L1_Error VL53L1_get_prev_shadow_system_results(
	VL53L1_DEV Dev,
	struct VL53L1_prev_shadow_system_results_t *pdata);




VL53L1_Error VL53L1_i2c_encode_prev_shadow_core_results(
	struct VL53L1_prev_shadow_core_results_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_prev_shadow_core_results(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_prev_shadow_core_results_t *pdata);




VL53L1_Error VL53L1_set_prev_shadow_core_results(
	VL53L1_DEV Dev,
	struct VL53L1_prev_shadow_core_results_t *pdata);




VL53L1_Error VL53L1_get_prev_shadow_core_results(
	VL53L1_DEV Dev,
	struct VL53L1_prev_shadow_core_results_t *pdata);




VL53L1_Error VL53L1_i2c_encode_patch_debug(
	struct VL53L1_patch_debug_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_patch_debug(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_patch_debug_t *pdata);




VL53L1_Error VL53L1_set_patch_debug(
	VL53L1_DEV Dev,
	struct VL53L1_patch_debug_t *pdata);




VL53L1_Error VL53L1_get_patch_debug(
	VL53L1_DEV Dev,
	struct VL53L1_patch_debug_t *pdata);




VL53L1_Error VL53L1_i2c_encode_gph_general_config(
	struct VL53L1_gph_general_config_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_gph_general_config(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_gph_general_config_t *pdata);




VL53L1_Error VL53L1_set_gph_general_config(
	VL53L1_DEV Dev,
	struct VL53L1_gph_general_config_t *pdata);




VL53L1_Error VL53L1_get_gph_general_config(
	VL53L1_DEV Dev,
	struct VL53L1_gph_general_config_t *pdata);




VL53L1_Error VL53L1_i2c_encode_gph_static_config(
	struct VL53L1_gph_static_config_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_gph_static_config(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_gph_static_config_t *pdata);




VL53L1_Error VL53L1_set_gph_static_config(
	VL53L1_DEV Dev,
	struct VL53L1_gph_static_config_t *pdata);




VL53L1_Error VL53L1_get_gph_static_config(
	VL53L1_DEV Dev,
	struct VL53L1_gph_static_config_t *pdata);




VL53L1_Error VL53L1_i2c_encode_gph_timing_config(
	struct VL53L1_gph_timing_config_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_gph_timing_config(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_gph_timing_config_t *pdata);




VL53L1_Error VL53L1_set_gph_timing_config(
	VL53L1_DEV Dev,
	struct VL53L1_gph_timing_config_t *pdata);




VL53L1_Error VL53L1_get_gph_timing_config(
	VL53L1_DEV Dev,
	struct VL53L1_gph_timing_config_t *pdata);




VL53L1_Error VL53L1_i2c_encode_fw_internal(
	struct VL53L1_fw_internal_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_fw_internal(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_fw_internal_t *pdata);




VL53L1_Error VL53L1_set_fw_internal(
	VL53L1_DEV Dev,
	struct VL53L1_fw_internal_t *pdata);




VL53L1_Error VL53L1_get_fw_internal(
	VL53L1_DEV Dev,
	struct VL53L1_fw_internal_t *pdata);




VL53L1_Error VL53L1_i2c_encode_patch_results(
	struct VL53L1_patch_results_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_patch_results(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_patch_results_t *pdata);




VL53L1_Error VL53L1_set_patch_results(
	VL53L1_DEV Dev,
	struct VL53L1_patch_results_t *pdata);




VL53L1_Error VL53L1_get_patch_results(
	VL53L1_DEV Dev,
	struct VL53L1_patch_results_t *pdata);




VL53L1_Error VL53L1_i2c_encode_shadow_system_results(
	struct VL53L1_shadow_system_results_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_shadow_system_results(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_shadow_system_results_t *pdata);




VL53L1_Error VL53L1_set_shadow_system_results(
	VL53L1_DEV Dev,
	struct VL53L1_shadow_system_results_t *pdata);




VL53L1_Error VL53L1_get_shadow_system_results(
	VL53L1_DEV Dev,
	struct VL53L1_shadow_system_results_t *pdata);




VL53L1_Error VL53L1_i2c_encode_shadow_core_results(
	struct VL53L1_shadow_core_results_t *pdata,
	uint16_t buf_size,
	uint8_t *pbuffer);




VL53L1_Error VL53L1_i2c_decode_shadow_core_results(
	uint16_t buf_size,
	uint8_t *pbuffer,
	struct VL53L1_shadow_core_results_t *pdata);




VL53L1_Error VL53L1_set_shadow_core_results(
	VL53L1_DEV Dev,
	struct VL53L1_shadow_core_results_t *pdata);




VL53L1_Error VL53L1_get_shadow_core_results(
	VL53L1_DEV Dev,
	struct VL53L1_shadow_core_results_t *pdata);



#endif /* _VL53L1_REGISTER_FUNCS_H_ */
