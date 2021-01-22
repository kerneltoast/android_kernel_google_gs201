// SPDX-License-Identifier: BSD-3-Clause
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




#include "vl53l1_types.h"
#include "vl53l1_platform_log.h"
#include "vl53l1_platform_user_defines.h"
#include "../../inc/vl53l1_core_support.h"
#include "../../inc/vl53l1_error_codes.h"

#include "../inc/vl53l1_dmax.h"

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_PROTECTED, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_PROTECTED, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_PROTECTED, \
	status, fmt, ##__VA_ARGS__)

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PROTECTED, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)


VL53L1_Error VL53L1_f_001(
	uint16_t target_reflectance,
	struct VL53L1_dmax_calibration_data_t *pcal,
	struct VL53L1_hist_gen3_dmax_config_t *pcfg,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_gen3_dmax_private_data_t *pdata,
	int16_t *pambient_dmax_mm)
{



	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint32_t pll_period_us = 0;
	uint32_t periods_elapsed = 0;

	uint32_t tmp32 = 0;
	uint64_t tmp64 = 0;

	uint32_t amb_thres_delta = 0;

	LOG_FUNCTION_START("");



	pdata->VL53L1_p_006 = 0x0000;
	pdata->VL53L1_p_033 = 0x0000;
	pdata->VL53L1_p_001 = 0x0000;
	pdata->VL53L1_p_012 = 0x0000;
	pdata->VL53L1_p_004 = 0x0000;
	pdata->VL53L1_p_034 = 0x0000;
	pdata->VL53L1_p_035 = 0;
	pdata->VL53L1_p_007 = 0;

	*pambient_dmax_mm = 0;


	if ((pbins->VL53L1_p_019 != 0) &&
		(pbins->total_periods_elapsed != 0)) {



		pll_period_us =
			VL53L1_calc_pll_period_us(pbins->VL53L1_p_019);



		periods_elapsed = pbins->total_periods_elapsed + 1;



		pdata->VL53L1_p_036 =
			VL53L1_duration_maths(
				pll_period_us,
				1<<4,
				VL53L1_RANGING_WINDOW_VCSEL_PERIODS,
				periods_elapsed);


		pdata->VL53L1_p_001 =
			VL53L1_rate_maths(
				pbins->VL53L1_p_004,
				pdata->VL53L1_p_036);



		pdata->VL53L1_p_033 =
			VL53L1_events_per_spad_maths(
				pbins->VL53L1_p_004,
				pbins->result__dss_actual_effective_spads,
				pdata->VL53L1_p_036);



		pdata->VL53L1_p_037 = pcfg->max_effective_spads;
		pdata->VL53L1_p_006 = pcfg->max_effective_spads;

		if (pdata->VL53L1_p_033 > 0) {
			tmp64 =
			(uint64_t)pcfg->dss_config__target_total_rate_mcps;
			tmp64 *= 1000;
			tmp64 <<= (11+1);
			tmp32 = pdata->VL53L1_p_033/2;
			tmp64 += (uint64_t)tmp32;
			tmp64 = do_division_u(tmp64,
				(uint64_t)pdata->VL53L1_p_033);

			if (tmp64 < (uint64_t)pcfg->max_effective_spads)
				pdata->VL53L1_p_006 = (uint16_t)tmp64;
		}
	}



	if ((pcal->ref__actual_effective_spads != 0) &&
		(pbins->VL53L1_p_019 != 0) &&
		(pcal->ref_reflectance_pc != 0) &&
		(pbins->total_periods_elapsed != 0)) {



		tmp64 = (uint64_t)pcal->ref__peak_signal_count_rate_mcps;
		tmp64 *= (1000 * 256);
		tmp32 = pcal->ref__actual_effective_spads/2;
		tmp64 += (uint64_t)tmp32;
		tmp64 = do_division_u(tmp64,
			(uint64_t)pcal->ref__actual_effective_spads);

		pdata->VL53L1_p_012 = (uint32_t)tmp64;
		pdata->VL53L1_p_012 <<= 4;



		tmp64 = (uint64_t)pdata->VL53L1_p_036;
		tmp64 *= (uint64_t)pdata->VL53L1_p_033;
		tmp64 *= (uint64_t)pdata->VL53L1_p_006;
		tmp64 += (1<<(11+7));
		tmp64 >>= (11+8);
		tmp64 += 500;
		tmp64 = do_division_u(tmp64, 1000);


		if (tmp64 > 0x00FFFFFF)
			tmp64 = 0x00FFFFFF;

		pdata->VL53L1_p_004 = (uint32_t)tmp64;



		tmp64 = (uint64_t)pdata->VL53L1_p_036;
		tmp64 *= (uint64_t)pdata->VL53L1_p_012;
		tmp64 *= (uint64_t)pdata->VL53L1_p_006;
		tmp64 += (1<<(11+7));
		tmp64 >>= (11+8);



		tmp64 *= ((uint64_t)target_reflectance *
				(uint64_t)pcal->coverglass_transmission);

		tmp64 += ((uint64_t)pcal->ref_reflectance_pc * 128);
		tmp64 = do_division_u(tmp64,
			((uint64_t)pcal->ref_reflectance_pc * 256));

		tmp64 += 500;
		tmp64 = do_division_u(tmp64, 1000);


		if (tmp64 > 0x00FFFFFF)
			tmp64 = 0x00FFFFFF;

		pdata->VL53L1_p_034 = (uint32_t)tmp64;



		tmp32 = VL53L1_isqrt(pdata->VL53L1_p_004 << 8);
		tmp32 *= (uint32_t)pcfg->ambient_thresh_sigma;



		if (pdata->VL53L1_p_004 <
			(uint32_t)pcfg->min_ambient_thresh_events) {

			amb_thres_delta =
				pcfg->min_ambient_thresh_events -
				(uint32_t)pdata->VL53L1_p_004;


			amb_thres_delta <<= 8;

			if (tmp32 < amb_thres_delta)
				tmp32 = amb_thres_delta;
		}



		pdata->VL53L1_p_007 =
			(int16_t)VL53L1_f_002(
				tmp32,
				pdata->VL53L1_p_034,
				(uint32_t)pcal->ref__distance_mm,
				(uint32_t)pcfg->signal_thresh_sigma);



		tmp32 = (uint32_t)pdata->VL53L1_p_034;
		tmp32 *= (uint32_t)pbins->vcsel_width;
		tmp32 += (1 << 3);
		tmp32 /= (1 << 4);

		pdata->VL53L1_p_035 =
			(int16_t)VL53L1_f_002(
				256 * (uint32_t)pcfg->signal_total_events_limit,
				tmp32,
				(uint32_t)pcal->ref__distance_mm,
				(uint32_t)pcfg->signal_thresh_sigma);




		if (pdata->VL53L1_p_035 < pdata->VL53L1_p_007)
			*pambient_dmax_mm = pdata->VL53L1_p_035;
		else
			*pambient_dmax_mm = pdata->VL53L1_p_007;

	}

	LOG_FUNCTION_END(status);

	return status;

}


uint32_t VL53L1_f_002(
	uint32_t events_threshold,
	uint32_t ref_signal_events,
	uint32_t ref_distance_mm,
	uint32_t signal_thresh_sigma)
{



	uint32_t tmp32 = 0;
	uint32_t range_mm = 0;

	tmp32 = 4 * events_threshold;



	tmp32 += ((uint32_t)signal_thresh_sigma *
			(uint32_t)signal_thresh_sigma);



	tmp32 = VL53L1_isqrt(tmp32);
	tmp32 += (uint32_t)signal_thresh_sigma;



	range_mm =
		(uint32_t)VL53L1_isqrt(ref_signal_events << 4);
	range_mm *= ref_distance_mm;

	if (tmp32 > 0) {
		range_mm += (tmp32);
		range_mm /= (2*tmp32);
	}

	return range_mm;

}
