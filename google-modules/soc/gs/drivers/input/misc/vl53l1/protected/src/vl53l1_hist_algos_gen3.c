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

#include "../inc/vl53l1_hist_core.h"
#include "../inc/vl53l1_hist_algos_gen3.h"
#include "../inc/vl53l1_sigma_estimate.h"
#include "../inc/vl53l1_dmax.h"

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_HISTOGRAM, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_HISTOGRAM, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_HISTOGRAM,\
	status, fmt, ##__VA_ARGS__)

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_HISTOGRAM, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)


void VL53L1_f_016(
	struct VL53L1_hist_gen3_algo_private_data_t *palgo)
{


	uint8_t lb = 0;

	palgo->VL53L1_p_023 = VL53L1_HISTOGRAM_BUFFER_SIZE;
	palgo->VL53L1_p_022 = 0;
	palgo->VL53L1_p_024 = 0;
	palgo->VL53L1_p_045 = 0;
	palgo->VL53L1_p_004 = 0;
	palgo->VL53L1_p_032 = 0;

	for (lb = palgo->VL53L1_p_022; lb < palgo->VL53L1_p_023; lb++) {
		palgo->VL53L1_p_043[lb] = 0;
		palgo->VL53L1_p_046[lb] = 0;
		palgo->VL53L1_p_047[lb] = 0;
		palgo->VL53L1_p_048[lb] = 0;
		palgo->VL53L1_p_008[lb] = 0;
	}

	palgo->VL53L1_p_049 = 0;
	palgo->VL53L1_p_050 = VL53L1_D_001;
	palgo->VL53L1_p_051 = 0;



	VL53L1_init_histogram_bin_data_struct(
		0,
		VL53L1_HISTOGRAM_BUFFER_SIZE,
		&(palgo->VL53L1_p_010));
	VL53L1_init_histogram_bin_data_struct(
		0,
		VL53L1_HISTOGRAM_BUFFER_SIZE,
		&(palgo->VL53L1_p_038));
	VL53L1_init_histogram_bin_data_struct(
		0,
		VL53L1_HISTOGRAM_BUFFER_SIZE,
		&(palgo->VL53L1_p_052));
	VL53L1_init_histogram_bin_data_struct(
		0,
		VL53L1_HISTOGRAM_BUFFER_SIZE,
		&(palgo->VL53L1_p_053));
	VL53L1_init_histogram_bin_data_struct(
		0,
		VL53L1_HISTOGRAM_BUFFER_SIZE,
		&(palgo->VL53L1_p_054));
}



VL53L1_Error VL53L1_f_018(
	uint16_t ambient_threshold_events_scaler,
	int32_t ambient_threshold_sigma,
	int32_t min_ambient_threshold_events,
	uint8_t algo__crosstalk_compensation_enable,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_histogram_bin_data_t *pxtalk,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo)
{



	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t lb = 0;
	uint8_t VL53L1_p_001 = 0;
	int64_t tmp = 0;
	int32_t amb_events = 0;
	int32_t VL53L1_p_008 = 0;
	int32_t samples = 0;

	LOG_FUNCTION_START("");


	palgo->VL53L1_p_023 = pbins->VL53L1_p_023;
	palgo->VL53L1_p_022 = pbins->VL53L1_p_022;
	palgo->VL53L1_p_024 = pbins->VL53L1_p_024;
	palgo->VL53L1_p_004 = pbins->VL53L1_p_004;



	palgo->VL53L1_p_031 =
			VL53L1_decode_vcsel_period(pbins->VL53L1_p_009);



	tmp = (int64_t)pbins->VL53L1_p_004;
	tmp *= (int64_t)ambient_threshold_events_scaler;
	tmp += 2048;
	tmp = do_division_s(tmp, 4096);
	amb_events = (int32_t)tmp;



	for (lb = 0; lb < pbins->VL53L1_p_024; lb++) {

		VL53L1_p_001 = lb >> 2;
		samples = (int32_t)pbins->bin_rep[VL53L1_p_001];

		if (samples > 0) {

			if (lb < pxtalk->VL53L1_p_024 &&
				algo__crosstalk_compensation_enable > 0)
				VL53L1_p_008 = samples * (amb_events +
					pxtalk->bin_data[lb]);
			else
				VL53L1_p_008 = samples * amb_events;

			VL53L1_p_008 = VL53L1_isqrt(VL53L1_p_008);

			VL53L1_p_008 += (samples/2);
			VL53L1_p_008 /= samples;
			VL53L1_p_008 *= ambient_threshold_sigma;
			VL53L1_p_008 += 8;
			VL53L1_p_008 /= 16;
			VL53L1_p_008 += amb_events;

			if (VL53L1_p_008 < min_ambient_threshold_events)
				VL53L1_p_008 = min_ambient_threshold_events;

			palgo->VL53L1_p_056[lb] = VL53L1_p_008;
			palgo->VL53L1_p_032 = VL53L1_p_008;
		}



	}



	palgo->VL53L1_p_045 = 0;

	for (lb = pbins->VL53L1_p_022; lb < pbins->VL53L1_p_024; lb++) {

		if (pbins->bin_data[lb] > palgo->VL53L1_p_056[lb]) {
			palgo->VL53L1_p_043[lb] = 1;
			palgo->VL53L1_p_046[lb] = 1;
			palgo->VL53L1_p_045++;
		} else {
			palgo->VL53L1_p_043[lb] = 0;
			palgo->VL53L1_p_046[lb] = 0;
		}
	}

	LOG_FUNCTION_END(status);

	return status;

}




VL53L1_Error VL53L1_f_019(
	struct VL53L1_hist_gen3_algo_private_data_t *palgo)
{



	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t found = 0;

	LOG_FUNCTION_START("");

	palgo->VL53L1_p_049 = 0;

	for (i = 0; i < palgo->VL53L1_p_031; i++) {

		j = (i + 1) % palgo->VL53L1_p_031;



		if (i < palgo->VL53L1_p_024 && j < palgo->VL53L1_p_024) {
			if (palgo->VL53L1_p_046[i] == 0 &&
				palgo->VL53L1_p_046[j] == 1 &&
				found == 0) {
				palgo->VL53L1_p_049 = i;
				found = 1;
			}
		}
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53L1_Error VL53L1_f_020(
	struct VL53L1_hist_gen3_algo_private_data_t *palgo)
{



	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t lb = 0;

	LOG_FUNCTION_START("");

	for (lb = palgo->VL53L1_p_049;
		lb < (palgo->VL53L1_p_049 +
		palgo->VL53L1_p_031);
		lb++) {



		i = lb % palgo->VL53L1_p_031;
		j = (lb + 1) % palgo->VL53L1_p_031;



		if (i < palgo->VL53L1_p_024 && j < palgo->VL53L1_p_024) {

			if (palgo->VL53L1_p_046[i] == 0 &&
				palgo->VL53L1_p_046[j] == 1)
				palgo->VL53L1_p_051++;

			if (palgo->VL53L1_p_046[i] > 0)
				palgo->VL53L1_p_047[i] = palgo->VL53L1_p_051;
			else
				palgo->VL53L1_p_047[i] = 0;
		}

	}


	if (palgo->VL53L1_p_051 > palgo->VL53L1_p_050)
		palgo->VL53L1_p_051 = palgo->VL53L1_p_050;

	LOG_FUNCTION_END(status);

	return status;

}


VL53L1_Error VL53L1_f_021(
	struct VL53L1_hist_gen3_algo_private_data_t *palgo)
{



	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t blb = 0;
	uint8_t pulse_no = 0;

	uint8_t max_filter_half_width = 0;

	struct VL53L1_hist_pulse_data_t *pdata;

	LOG_FUNCTION_START("");


	/* add boundary check for uint8_t VL53L1_p_031 */
	if (palgo->VL53L1_p_031 > 0)
		max_filter_half_width = palgo->VL53L1_p_031 - 1;
	max_filter_half_width = max_filter_half_width >> 1;

	for (blb = palgo->VL53L1_p_049;
		blb < (palgo->VL53L1_p_049 +
		palgo->VL53L1_p_031);
		blb++) {

		if (palgo->VL53L1_p_031 > VL53L1_HISTOGRAM_BUFFER_SIZE)
			return VL53L1_ERROR_INVALID_PARAMS;

		i = blb % palgo->VL53L1_p_031;
		j = (blb + 1) % palgo->VL53L1_p_031;



		if (i < palgo->VL53L1_p_024 &&
				j < palgo->VL53L1_p_024) {



			if (palgo->VL53L1_p_047[i] == 0 &&
					palgo->VL53L1_p_047[j] > 0) {

				pulse_no = palgo->VL53L1_p_047[j] - 1;

				if (palgo->VL53L1_p_050 > VL53L1_D_001)
					return VL53L1_ERROR_INVALID_PARAMS;

				if (pulse_no < palgo->VL53L1_p_050) {
					pdata =
					    &(palgo->VL53L1_p_002[pulse_no]);

					pdata->VL53L1_p_015 = blb;
					pdata->VL53L1_p_022 = blb + 1;
					pdata->VL53L1_p_025 = 0xFF;
					pdata->VL53L1_p_026 = 0;
					pdata->VL53L1_p_016 = 0;
				}
			}



			if (palgo->VL53L1_p_047[i] > 0
				&& palgo->VL53L1_p_047[j] == 0) {

				pulse_no = palgo->VL53L1_p_047[i] - 1;

				if (palgo->VL53L1_p_050 > VL53L1_D_001)
					return VL53L1_ERROR_INVALID_PARAMS;

				if (pulse_no < palgo->VL53L1_p_050) {
					pdata =
					    &(palgo->VL53L1_p_002[pulse_no]);

					pdata->VL53L1_p_026 = blb;
					pdata->VL53L1_p_016 = blb + 1;
					/* add overflow protector */
					if (pdata->VL53L1_p_022 >
					    (pdata->VL53L1_p_026 + 1))
						pdata->VL53L1_p_027 = 0;
					else
						pdata->VL53L1_p_027 =
						    (pdata->VL53L1_p_026 + 1) -
						    pdata->VL53L1_p_022;
					/* add overflow protector */
					if (pdata->VL53L1_p_015 >
					    (pdata->VL53L1_p_016 + 1))
						pdata->VL53L1_p_055 = 0;
					else
						pdata->VL53L1_p_055 =
						    (pdata->VL53L1_p_016 + 1) -
						    pdata->VL53L1_p_015;

					if (pdata->VL53L1_p_055 >
						max_filter_half_width)
						pdata->VL53L1_p_055 =
						max_filter_half_width;
				}

			}
		}
	}

	LOG_FUNCTION_END(status);

	return status;

}


VL53L1_Error VL53L1_f_028(
	VL53L1_HistTargetOrder target_order,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo)
{



	VL53L1_Error status = VL53L1_ERROR_NONE;

	struct VL53L1_hist_pulse_data_t tmp;
	struct VL53L1_hist_pulse_data_t *ptmp = &tmp;
	struct VL53L1_hist_pulse_data_t *p0;
	struct VL53L1_hist_pulse_data_t *p1;

	uint8_t i = 0;
	uint8_t swapped = 1;

	LOG_FUNCTION_START("");

	if (!(palgo->VL53L1_p_051 > 1))
		goto ENDFUNC;

	while (swapped > 0) {

		swapped = 0;

		for (i = 1; i < palgo->VL53L1_p_051; i++) {

			p0 = &(palgo->VL53L1_p_002[i-1]);
			p1 = &(palgo->VL53L1_p_002[i]);



			if (target_order ==
			    VL53L1_HIST_TARGET_ORDER__STRONGEST_FIRST) {

				if (p0->VL53L1_p_013 <
						p1->VL53L1_p_013) {



					memcpy(ptmp,
					p1, sizeof(struct VL53L1_hist_pulse_data_t));
					memcpy(p1,
					p0, sizeof(struct VL53L1_hist_pulse_data_t));
					memcpy(p0,
					ptmp, sizeof(struct VL53L1_hist_pulse_data_t));

					swapped = 1;
				}

			} else {

				if (p0->VL53L1_p_014 > p1->VL53L1_p_014) {



					memcpy(ptmp,
					p1, sizeof(struct VL53L1_hist_pulse_data_t));
					memcpy(p1,
					p0, sizeof(struct VL53L1_hist_pulse_data_t));
					memcpy(p0,
					ptmp, sizeof(struct VL53L1_hist_pulse_data_t));

					swapped = 1;
				}

			}
		}
	}

ENDFUNC:
	LOG_FUNCTION_END(status);

	return status;

}


VL53L1_Error VL53L1_f_022(
	uint8_t pulse_no,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint8_t i = 0;
	uint8_t lb = 0;

	struct VL53L1_hist_pulse_data_t *pdata = &(palgo->VL53L1_p_002[pulse_no]);

	LOG_FUNCTION_START("");



	pdata->VL53L1_p_021 = 0;
	pdata->VL53L1_p_020 = 0;

	for (lb = pdata->VL53L1_p_015; lb <= pdata->VL53L1_p_016; lb++) {
		i = lb % palgo->VL53L1_p_031;
		pdata->VL53L1_p_021 += pbins->bin_data[i];
		pdata->VL53L1_p_020 += palgo->VL53L1_p_004;
	}



	pdata->VL53L1_p_013 =
		pdata->VL53L1_p_021 - pdata->VL53L1_p_020;

	LOG_FUNCTION_END(status);

	return status;
}


VL53L1_Error VL53L1_f_027(
	uint8_t pulse_no,
	uint8_t clip_events,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint8_t i = 0;
	int16_t VL53L1_p_015 = 0;
	int16_t VL53L1_p_016 = 0;
	int16_t window_width = 0;
	uint32_t tmp_phase = 0;

	struct VL53L1_hist_pulse_data_t *pdata = &(palgo->VL53L1_p_002[pulse_no]);

	LOG_FUNCTION_START("");



	i = pdata->VL53L1_p_025 % palgo->VL53L1_p_031;

	VL53L1_p_015 = (int16_t)i;
	VL53L1_p_015 += (int16_t)pdata->VL53L1_p_015;
	VL53L1_p_015 -= (int16_t)pdata->VL53L1_p_025;

	VL53L1_p_016 = (int16_t)i;
	VL53L1_p_016 += (int16_t)pdata->VL53L1_p_016;
	VL53L1_p_016 -= (int16_t)pdata->VL53L1_p_025;


	window_width = VL53L1_p_016 - VL53L1_p_015;
	if (window_width > 3)
		window_width = 3;

	status =
		VL53L1_f_030(
			VL53L1_p_015,
			VL53L1_p_015 + window_width,
			palgo->VL53L1_p_031,
			clip_events,
			pbins,
			&(pdata->VL53L1_p_028));


	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_f_030(
				VL53L1_p_016 - window_width,
				VL53L1_p_016,
				palgo->VL53L1_p_031,
				clip_events,
				pbins,
				&(pdata->VL53L1_p_029));


	if (pdata->VL53L1_p_028 > pdata->VL53L1_p_029) {
		tmp_phase = pdata->VL53L1_p_028;
		pdata->VL53L1_p_028 = pdata->VL53L1_p_029;
		pdata->VL53L1_p_029 = tmp_phase;
	}


	if (pdata->VL53L1_p_014 < pdata->VL53L1_p_028)
		pdata->VL53L1_p_028 = pdata->VL53L1_p_014;


	if (pdata->VL53L1_p_014 > pdata->VL53L1_p_029)
		pdata->VL53L1_p_029 = pdata->VL53L1_p_014;

	LOG_FUNCTION_END(status);

	return status;
}


VL53L1_Error VL53L1_f_030(
	int16_t VL53L1_p_022,
	int16_t VL53L1_p_026,
	uint8_t VL53L1_p_031,
	uint8_t clip_events,
	struct VL53L1_histogram_bin_data_t *pbins,
	uint32_t *pphase)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;

	int16_t i = 0;
	int16_t lb = 0;

	int64_t VL53L1_p_008 = 0;
	int64_t event_sum = 0;
	int64_t weighted_sum = 0;

	LOG_FUNCTION_START("");

	*pphase = VL53L1_MAX_ALLOWED_PHASE;

	if (VL53L1_p_031 == 0)
		return VL53L1_ERROR_DIVISION_BY_ZERO;

	for (lb = VL53L1_p_022; lb <= VL53L1_p_026; lb++) {


		if (lb < 0)
			i = lb + (int16_t)VL53L1_p_031;
		else
			i = lb % (int16_t)VL53L1_p_031;

		if ((i >= 0) && (i < VL53L1_HISTOGRAM_BUFFER_SIZE)) {
			VL53L1_p_008 =
				(int64_t)pbins->bin_data[i] -
				(int64_t)pbins->VL53L1_p_004;

			if (clip_events > 0 && VL53L1_p_008 < 0)
				VL53L1_p_008 = 0;
			event_sum += VL53L1_p_008;
			weighted_sum +=
				(VL53L1_p_008 * (1024 + (2048*(int64_t)lb)));
		}

		trace_print(
			VL53L1_TRACE_LEVEL_INFO,
			"\tb = %5d : i = %5d : VL53L1_p_008 = %8d,",
			lb, i, VL53L1_p_008);

		trace_print(
			VL53L1_TRACE_LEVEL_INFO,
			" event_sum = %8d, weighted_sum = %8d\n",
			event_sum, weighted_sum);
	}

	if (event_sum > 0) {

		weighted_sum += do_division_s(event_sum, 2);
		weighted_sum = do_division_s(weighted_sum, event_sum);

		if (weighted_sum < 0)
			weighted_sum = 0;

		*pphase = (uint32_t)weighted_sum;
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53L1_Error VL53L1_f_023(
	uint8_t pulse_no,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo,
	int32_t pad_value,
	struct VL53L1_histogram_bin_data_t *ppulse)
{



	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint8_t i = 0;
	uint8_t lb = 0;

	struct VL53L1_hist_pulse_data_t *pdata = &(palgo->VL53L1_p_002[pulse_no]);

	LOG_FUNCTION_START("");



	memcpy(ppulse, pbins, sizeof(struct VL53L1_histogram_bin_data_t));



	for (lb = palgo->VL53L1_p_049;
		lb < (palgo->VL53L1_p_049 +
		palgo->VL53L1_p_031);
		lb++) {

		if (lb < pdata->VL53L1_p_015 || lb > pdata->VL53L1_p_016) {
			i = lb % palgo->VL53L1_p_031;
			if (i < ppulse->VL53L1_p_024)
				ppulse->bin_data[i] = pad_value;
		}
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53L1_Error VL53L1_f_026(
	uint8_t bin,
	uint8_t sigma_estimator__sigma_ref_mm,
	uint8_t VL53L1_p_031,
	uint8_t VL53L1_p_055,
	uint8_t crosstalk_compensation_enable,
	struct VL53L1_histogram_bin_data_t *phist_data_ap,
	struct VL53L1_histogram_bin_data_t *phist_data_zp,
	struct VL53L1_histogram_bin_data_t *pxtalk_hist,
	uint16_t *psigma_est)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_Error func_status = VL53L1_ERROR_NONE;

	uint8_t i = 0;
	int32_t VL53L1_p_003 = 0;
	int32_t VL53L1_p_018 = 0;
	int32_t VL53L1_p_001 = 0;
	int32_t a_zp = 0;
	int32_t c_zp = 0;
	int32_t ax = 0;
	int32_t bx = 0;
	int32_t cx = 0;


	if (VL53L1_p_031 == 0) {
		*psigma_est = 0xFFFF;
		return VL53L1_ERROR_DIVISION_BY_ZERO;
	}
	i = bin % VL53L1_p_031;



	VL53L1_f_013(
			i,
			VL53L1_p_055,
			phist_data_zp,
			&a_zp,
			&VL53L1_p_018,
			&c_zp);



	VL53L1_f_013(
			i,
			VL53L1_p_055,
			phist_data_ap,
			&VL53L1_p_003,
			&VL53L1_p_018,
			&VL53L1_p_001);

	if (crosstalk_compensation_enable > 0)
		VL53L1_f_013(
				i,
				VL53L1_p_055,
				pxtalk_hist,
				&ax,
				&bx,
				&cx);







	func_status =
		VL53L1_f_014(
			sigma_estimator__sigma_ref_mm,
			(uint32_t)VL53L1_p_003,
			(uint32_t)VL53L1_p_018,
			(uint32_t)VL53L1_p_001,
			(uint32_t)a_zp,
			(uint32_t)c_zp,
			(uint32_t)bx,
			(uint32_t)ax,
			(uint32_t)cx,
			(uint32_t)phist_data_ap->VL53L1_p_004,
			phist_data_ap->VL53L1_p_019,
			psigma_est);




	if (func_status == VL53L1_ERROR_DIVISION_BY_ZERO)
		*psigma_est = 0xFFFF;


	return status;
}


void VL53L1_f_029(
	uint8_t range_id,
	uint8_t valid_phase_low,
	uint8_t valid_phase_high,
	uint16_t sigma_thres,
	struct VL53L1_histogram_bin_data_t *pbins,
	struct VL53L1_hist_pulse_data_t *ppulse,
	struct VL53L1_range_data_t *pdata)
{

	uint16_t lower_phase_limit = 0;
	uint16_t upper_phase_limit = 0;



	pdata->range_id = range_id;
	pdata->time_stamp = 0;

	pdata->VL53L1_p_015 = ppulse->VL53L1_p_015;
	pdata->VL53L1_p_022 = ppulse->VL53L1_p_022;
	pdata->VL53L1_p_025 = ppulse->VL53L1_p_025;
	pdata->VL53L1_p_026 = ppulse->VL53L1_p_026;
	pdata->VL53L1_p_016 = ppulse->VL53L1_p_016;
	pdata->VL53L1_p_027 = ppulse->VL53L1_p_027;



	pdata->VL53L1_p_030 =
		(ppulse->VL53L1_p_016 + 1) - ppulse->VL53L1_p_015;



	pdata->zero_distance_phase = pbins->zero_distance_phase;
	pdata->VL53L1_p_005 = ppulse->VL53L1_p_005;
	pdata->VL53L1_p_028 = (uint16_t)ppulse->VL53L1_p_028;
	pdata->VL53L1_p_014 = (uint16_t)ppulse->VL53L1_p_014;
	pdata->VL53L1_p_029 = (uint16_t)ppulse->VL53L1_p_029;
	pdata->VL53L1_p_021 = (uint32_t)ppulse->VL53L1_p_021;
	pdata->VL53L1_p_013 = ppulse->VL53L1_p_013;
	pdata->VL53L1_p_020 = (uint32_t)ppulse->VL53L1_p_020;
	pdata->total_periods_elapsed = pbins->total_periods_elapsed;



	pdata->range_status = VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK;


	if (sigma_thres > 0 &&
		(uint32_t)ppulse->VL53L1_p_005 > ((uint32_t)sigma_thres << 5))
		pdata->range_status = VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK;



	lower_phase_limit = (uint8_t)valid_phase_low << 8;
	if (lower_phase_limit < pdata->zero_distance_phase)
		lower_phase_limit =
			pdata->zero_distance_phase -
			lower_phase_limit;
	else
		lower_phase_limit = 0;

	upper_phase_limit = (uint8_t)valid_phase_high << 8;
	upper_phase_limit += pbins->zero_distance_phase;

	if (pdata->VL53L1_p_014 < lower_phase_limit ||
		pdata->VL53L1_p_014 > upper_phase_limit)
		pdata->range_status = VL53L1_DEVICEERROR_RANGEPHASECHECK;

}
