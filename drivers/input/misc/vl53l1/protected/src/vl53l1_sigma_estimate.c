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

#include "../../inc/vl53l1_core_support.h"
#include "../../inc/vl53l1_error_codes.h"

#include "../inc/vl53l1_sigma_estimate.h"

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


uint16_t VL53L1_f_042(
		uint8_t sigma_estimator__effective_pulse_width_ns,
		uint8_t sigma_estimator__effective_ambient_width_ns,
		uint8_t sigma_estimator__sigma_ref_mm,
		struct VL53L1_range_data_t *pdata)
{



	uint16_t sigma_est = VL53L1_D_002;

	uint32_t tmp0 = 0;
	uint32_t tmp1 = 0;
	uint32_t tmp2 = 0;

	uint32_t sigma_est__rtn_array = 0;
	uint32_t sigma_est__ref_array = 0;

	LOG_FUNCTION_START("");

	if (pdata->peak_signal_count_rate_mcps > 0 &&
		pdata->VL53L1_p_013 > 0) {



		tmp0 = 100 *
			(uint32_t)sigma_estimator__effective_pulse_width_ns;



		tmp1 = ((uint32_t)sigma_estimator__effective_pulse_width_ns *
			100 *
			(uint32_t)sigma_estimator__effective_ambient_width_ns);

		tmp1 = (tmp1 +
			(uint32_t)pdata->peak_signal_count_rate_mcps/2) /
			(uint32_t)pdata->peak_signal_count_rate_mcps;



		sigma_est__rtn_array =
			VL53L1_f_043(tmp0, tmp1);



		sigma_est__rtn_array =
			((VL53L1_SPEED_OF_LIGHT_IN_AIR + 1000) / 2000) *
			sigma_est__rtn_array;



		tmp2 =
			VL53L1_isqrt(12 * (uint32_t)pdata->VL53L1_p_013);

		if (tmp2 > 0) {

			sigma_est__rtn_array =
					(sigma_est__rtn_array + tmp2 / 2) / tmp2;



			sigma_est__ref_array =
				100 * (uint32_t)sigma_estimator__sigma_ref_mm;

			sigma_est =
				(uint16_t)VL53L1_f_043(
						(uint32_t)sigma_est__ref_array,
						sigma_est__rtn_array);

		} else {
			sigma_est = VL53L1_D_002;
		}

	}

	pdata->VL53L1_p_005 = sigma_est;

	LOG_FUNCTION_END(0);

	return sigma_est;

}


uint16_t VL53L1_f_044(
	uint8_t sigma_estimator__effective_pulse_width_ns,
	uint8_t sigma_estimator__effective_ambient_width_ns,
	uint8_t sigma_estimator__sigma_ref_mm,
	struct VL53L1_range_data_t *pdata)
{


	uint16_t sigma_est = VL53L1_D_002;

	uint32_t eqn7 = 0;
	uint32_t sigma_est__ref_sq = 0;
	uint32_t sigma_est__rtn_sq = 0;

	uint64_t tmp0 = 0;
	uint64_t tmp1 = 0;

	LOG_FUNCTION_START("");

	if (pdata->peak_signal_count_rate_mcps > 0 &&
		pdata->VL53L1_p_013 > 0) {



		eqn7 = 4573 * 4573;
		eqn7 = eqn7 / (3 * (uint32_t)pdata->VL53L1_p_013);



		tmp0 = ((uint64_t)sigma_estimator__effective_pulse_width_ns)
				<< 8;



		tmp1 = ((uint64_t)pdata->ambient_count_rate_mcps *
			(uint64_t)sigma_estimator__effective_ambient_width_ns)
					<< 8;

		tmp1 = do_division_u(tmp1,
			(uint64_t)pdata->peak_signal_count_rate_mcps);



		tmp1 = 16 * (uint64_t)eqn7 * (tmp0 * tmp0 + tmp1 * tmp1);
		tmp1 = do_division_u(tmp1, (15625 * 15625));
		sigma_est__rtn_sq = (uint32_t)tmp1;



		sigma_est__ref_sq = ((uint32_t)sigma_estimator__sigma_ref_mm)
				<< 2;

		sigma_est__ref_sq = sigma_est__ref_sq * sigma_est__ref_sq;



		sigma_est = (uint16_t)VL53L1_isqrt(sigma_est__ref_sq +
				sigma_est__rtn_sq);

	}

	pdata->VL53L1_p_005 = sigma_est;

	LOG_FUNCTION_END(0);

	return sigma_est;

}



VL53L1_Error VL53L1_f_045(
	uint8_t sigma_estimator__sigma_ref_mm,
	uint32_t VL53L1_p_003,
	uint32_t VL53L1_p_018,
	uint32_t VL53L1_p_001,
	uint32_t a_zp,
	uint32_t c_zp,
	uint32_t bx,
	uint32_t ax_zp,
	uint32_t cx_zp,
	uint32_t VL53L1_p_004,
	uint16_t fast_osc_frequency,
	uint16_t *psigma_est)
{


	VL53L1_Error status = VL53L1_ERROR_DIVISION_BY_ZERO;
	uint32_t sigma_int = VL53L1_D_002;

	uint32_t pll_period_mm = 0;

	uint64_t tmp0 = 0;
	uint64_t tmp1 = 0;
	uint64_t b_minus_amb = 0;
	uint64_t VL53L1_p_041 = 0;

	*psigma_est = VL53L1_D_002;



	if (fast_osc_frequency != 0) {



		pll_period_mm = VL53L1_calc_pll_period_mm(fast_osc_frequency);


		pll_period_mm = (pll_period_mm + 0x02) >> 2;



		if (VL53L1_p_004 > VL53L1_p_018)
			b_minus_amb = (uint64_t)VL53L1_p_004 -
			(uint64_t)VL53L1_p_018;
		else
			b_minus_amb = (uint64_t)VL53L1_p_018 -
			(uint64_t)VL53L1_p_004;



		if (VL53L1_p_003 > VL53L1_p_001)
			VL53L1_p_041 = (uint64_t)VL53L1_p_003 -
			(uint64_t)VL53L1_p_001;
		else
			VL53L1_p_041 = (uint64_t)VL53L1_p_001 -
			(uint64_t)VL53L1_p_003;





		if (b_minus_amb != 0) {



			tmp0 = (uint64_t)pll_period_mm *
					(uint64_t)pll_period_mm;
			tmp0 = tmp0 * ((uint64_t)c_zp +
					(uint64_t)cx_zp + (uint64_t)a_zp +
					(uint64_t)ax_zp);
			tmp0 = do_division_u((tmp0 + (b_minus_amb >> 1)),
				b_minus_amb);



			tmp1 = (uint64_t)pll_period_mm *
					(uint64_t)pll_period_mm * VL53L1_p_041;
			tmp1 = do_division_u((tmp1 + (b_minus_amb >> 1)),
				b_minus_amb);

			tmp1 = tmp1 * VL53L1_p_041;
			tmp1 = do_division_u((tmp1 + (b_minus_amb >> 1)),
				b_minus_amb);

			tmp1 = tmp1 * ((uint64_t)VL53L1_p_018 + (uint64_t)bx +
					(uint64_t)VL53L1_p_004);
			tmp1 = do_division_u((tmp1 + (b_minus_amb >> 1)),
				b_minus_amb);



			tmp0 = tmp0 + tmp1;
			tmp0 = do_division_u((tmp0 + (b_minus_amb >> 1)),
				b_minus_amb);
			tmp0 = (tmp0 + 0x01) >> 2;



			tmp1 = (uint64_t)sigma_estimator__sigma_ref_mm << 2;
			tmp1 = tmp1 * tmp1;
			tmp0 = tmp0 + tmp1;



			if (tmp0 > 0xFFFFFFFF)
				tmp0 = 0xFFFFFFFF;

			sigma_int = VL53L1_isqrt((uint32_t)tmp0);



			if (sigma_int > VL53L1_D_002)
				*psigma_est =
				(uint16_t)VL53L1_D_002;
			else
				*psigma_est = (uint16_t)sigma_int;

			status = VL53L1_ERROR_NONE;
		}

	}

	return status;
}



VL53L1_Error VL53L1_f_014(
	uint8_t sigma_estimator__sigma_ref_mm,
	uint32_t VL53L1_p_003,
	uint32_t VL53L1_p_018,
	uint32_t VL53L1_p_001,
	uint32_t a_zp,
	uint32_t c_zp,
	uint32_t bx,
	uint32_t ax_zp,
	uint32_t cx_zp,
	uint32_t VL53L1_p_004,
	uint16_t fast_osc_frequency,
	uint16_t *psigma_est)
{


	VL53L1_Error status = VL53L1_ERROR_DIVISION_BY_ZERO;
	uint32_t sigma_int = VL53L1_D_002;

	uint32_t pll_period_mm = 0;

	uint64_t tmp0 = 0;
	uint64_t tmp1 = 0;
	uint64_t b_minus_amb = 0;
	uint64_t VL53L1_p_041 = 0;

	*psigma_est = VL53L1_D_002;



	if (fast_osc_frequency != 0) {



		pll_period_mm = VL53L1_calc_pll_period_mm(fast_osc_frequency);



		if (VL53L1_p_004 > VL53L1_p_018)
			b_minus_amb = (uint64_t)VL53L1_p_004 -
			(uint64_t)VL53L1_p_018;
		else
			b_minus_amb = (uint64_t)VL53L1_p_018 -
			(uint64_t)VL53L1_p_004;



		if (VL53L1_p_003 > VL53L1_p_001)
			VL53L1_p_041 = (uint64_t)VL53L1_p_003 -
			(uint64_t)VL53L1_p_001;
		else
			VL53L1_p_041 = (uint64_t)VL53L1_p_001 -
			(uint64_t)VL53L1_p_003;



		if (b_minus_amb != 0) {




			tmp0 = (uint64_t)VL53L1_p_018 + (uint64_t)bx +
					(uint64_t)VL53L1_p_004;
			if (tmp0 > VL53L1_D_003)
				tmp0 = VL53L1_D_003;



			tmp1 = (uint64_t)VL53L1_p_041 * (uint64_t)VL53L1_p_041;
			tmp1 = tmp1 << 8;


			if (tmp1 > VL53L1_D_004)
				tmp1 = VL53L1_D_004;


			tmp1 = do_division_u(tmp1, b_minus_amb);
			tmp1 = do_division_u(tmp1, b_minus_amb);


			if (tmp1 > (uint64_t)VL53L1_D_005)
				tmp1 = (uint64_t)VL53L1_D_005;


			tmp0 = tmp1 * tmp0;


			tmp1 = (uint64_t)c_zp + (uint64_t)cx_zp +
				(uint64_t)a_zp + (uint64_t)ax_zp;


			if (tmp1 > (uint64_t)VL53L1_D_003)
				tmp1 = (uint64_t)VL53L1_D_003;

			tmp1 = tmp1 << 8;


			tmp0 = tmp1 + tmp0;
			if (tmp0 > (uint64_t)VL53L1_D_006)
				tmp0 = (uint64_t)VL53L1_D_006;






			if (tmp0 > (uint64_t)VL53L1_D_007) {
				tmp0 = do_division_u(tmp0, b_minus_amb);
				tmp0 = tmp0 * pll_period_mm;
			} else {
				tmp0 = tmp0 * pll_period_mm;
				tmp0 = do_division_u(tmp0, b_minus_amb);
			}


			if (tmp0 > (uint64_t)VL53L1_D_006)
				tmp0 = (uint64_t)VL53L1_D_006;



			if (tmp0 > (uint64_t)VL53L1_D_007) {
				tmp0 = do_division_u(tmp0, b_minus_amb);
				tmp0 = do_division_u(tmp0, 4);
				tmp0 = tmp0 * pll_period_mm;
			} else {
				tmp0 = tmp0 * pll_period_mm;
				tmp0 = do_division_u(tmp0, b_minus_amb);
				tmp0 = do_division_u(tmp0, 4);
			}


			if (tmp0 > (uint64_t)VL53L1_D_006)
				tmp0 = (uint64_t)VL53L1_D_006;


			tmp0 = tmp0 >> 2;


			if (tmp0 > (uint64_t)VL53L1_D_007)
				tmp0 = (uint64_t)VL53L1_D_007;


			tmp1 = (uint64_t)sigma_estimator__sigma_ref_mm << 7;
			tmp1 = tmp1 * tmp1;
			tmp0 = tmp0 + tmp1;


			if (tmp0 > (uint64_t)VL53L1_D_007)
				tmp0 = (uint64_t)VL53L1_D_007;


			sigma_int = VL53L1_isqrt((uint32_t)tmp0);

			*psigma_est = (uint16_t)sigma_int;

			status = VL53L1_ERROR_NONE;
		}

	}

	return status;
}

uint32_t VL53L1_f_046(
	uint64_t VL53L1_p_003,
	uint32_t size
	)
{


	uint64_t next;
	uint64_t upper;
	uint64_t lower;
	uint32_t stepsize;
	uint32_t count;


	next = VL53L1_p_003;
	upper = 0;
	lower = 0;
	stepsize = size/2;
	count = 0;

	while (1) {
		upper = next >> stepsize;
		lower = next & ((1 << stepsize) - 1);

		if (upper != 0) {
			count += stepsize;
			next = upper;
		} else {
			next = lower;
		}

		stepsize = stepsize / 2;
		if (stepsize == 0)
			break;
	}

	return count;
}



uint32_t VL53L1_f_043(
	uint32_t VL53L1_p_003,
	uint32_t VL53L1_p_018)
{


	uint32_t res = 0;

	if (VL53L1_p_003 > 65535 || VL53L1_p_018 > 65535)
		res = 65535;
	else
		res = VL53L1_isqrt(VL53L1_p_003*VL53L1_p_003 +
			VL53L1_p_018*VL53L1_p_018);

	return res;
}
