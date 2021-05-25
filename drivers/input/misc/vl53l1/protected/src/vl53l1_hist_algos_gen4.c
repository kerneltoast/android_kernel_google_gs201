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
#include "../inc/vl53l1_hist_algos_gen4.h"
#include "../inc/vl53l1_sigma_estimate.h"
#include "../inc/vl53l1_dmax.h"
#ifdef __KERNEL__
#include <linux/math64.h>
#include <linux/kernel.h>
#endif

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_HISTOGRAM, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_HISTOGRAM, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_HISTOGRAM, \
	status, fmt, ##__VA_ARGS__)

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_HISTOGRAM, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)


void VL53L1_f_032(
	struct VL53L1_hist_gen4_algo_filtered_data_t *palgo)
{


	uint8_t lb = 0;

	palgo->VL53L1_p_023 = VL53L1_HISTOGRAM_BUFFER_SIZE;
	palgo->VL53L1_p_022 = 0;
	palgo->VL53L1_p_024 = 0;

	for (lb = palgo->VL53L1_p_022; lb < palgo->VL53L1_p_023; lb++) {
		palgo->VL53L1_p_003[lb] = 0;
		palgo->VL53L1_p_018[lb] = 0;
		palgo->VL53L1_p_001[lb] = 0;
		palgo->VL53L1_p_039[lb] = 0;
		palgo->VL53L1_p_040[lb] = 0;
		palgo->VL53L1_p_043[lb] = 0;
	}
}


VL53L1_Error VL53L1_f_033(
	struct VL53L1_dmax_calibration_data_t *pdmax_cal,
	struct VL53L1_hist_gen3_dmax_config_t *pdmax_cfg,
	struct VL53L1_hist_post_process_config_t *ppost_cfg,
	struct VL53L1_histogram_bin_data_t *pbins_input,
	struct VL53L1_histogram_bin_data_t *pxtalk,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo3,
	struct VL53L1_hist_gen4_algo_filtered_data_t *pfiltered,
	struct VL53L1_hist_gen3_dmax_private_data_t *pdmax_algo,
	struct VL53L1_range_results_t *presults,
	uint8_t histo_merge_nb)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;

	struct VL53L1_hist_pulse_data_t *ppulse_data;
	struct VL53L1_range_data_t *prange_data;

	uint8_t p = 0;
	struct VL53L1_histogram_bin_data_t *pB = &(palgo3->VL53L1_p_010);
	uint16_t enabled_spads_ninety_percent;
	int32_t amb_threshold_sigma;

	LOG_FUNCTION_START("");





	VL53L1_f_016(palgo3);



	memcpy(
		&(palgo3->VL53L1_p_010),
		pbins_input,
		sizeof(struct VL53L1_histogram_bin_data_t));



	presults->cfg_device_state = pbins_input->cfg_device_state;
	presults->rd_device_state = pbins_input->rd_device_state;
	presults->zone_id = pbins_input->zone_id;
	presults->stream_count = pbins_input->result__stream_count;
	presults->wrap_dmax_mm = 0;
	presults->max_results = VL53L1_MAX_RANGE_RESULTS;
	presults->active_results = 0;

	for (p = 0; p < VL53L1_MAX_AMBIENT_DMAX_VALUES; p++)
		presults->VL53L1_p_007[p] = 0;



	VL53L1_hist_calc_zero_distance_phase(&(palgo3->VL53L1_p_010));



	if (ppost_cfg->hist_amb_est_method ==
		VL53L1_HIST_AMB_EST_METHOD__THRESHOLDED_BINS)
		VL53L1_hist_estimate_ambient_from_thresholded_bins(
			(int32_t)ppost_cfg->ambient_thresh_sigma0,
			&(palgo3->VL53L1_p_010));
	else
		VL53L1_hist_estimate_ambient_from_ambient_bins(
				&(palgo3->VL53L1_p_010));


	VL53L1_hist_remove_ambient_bins(&(palgo3->VL53L1_p_010));


	if (ppost_cfg->algo__crosstalk_compensation_enable > 0)
		VL53L1_f_004(
				pxtalk,
				&(palgo3->VL53L1_p_010),
				&(palgo3->VL53L1_p_038));


	amb_threshold_sigma = ppost_cfg->ambient_thresh_sigma1;
	enabled_spads_ninety_percent =
		presults->fmt_total_enabled_spads * 230;
	if ((pbins_input->result__dss_actual_effective_spads <
	     enabled_spads_ninety_percent) &&
	    (presults->VL53L1_p_002[0].ambient_count_rate_mcps < (5 * 128))) {
		amb_threshold_sigma *= histo_merge_nb;
	}

	pdmax_cfg->ambient_thresh_sigma = amb_threshold_sigma;

	for (p = 0; p < VL53L1_MAX_AMBIENT_DMAX_VALUES; p++) {
		if (status == VL53L1_ERROR_NONE) {
			status =
			VL53L1_f_001(
				pdmax_cfg->target_reflectance_for_dmax_calc[p],
				pdmax_cal,
				pdmax_cfg,
				&(palgo3->VL53L1_p_010),
				pdmax_algo,
				&(presults->VL53L1_p_007[p]));
		}
	}





	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_f_018(
			ppost_cfg->ambient_thresh_events_scaler,
			(int32_t)amb_threshold_sigma,
			(int32_t)ppost_cfg->min_ambient_thresh_events,
			ppost_cfg->algo__crosstalk_compensation_enable,
			&(palgo3->VL53L1_p_010),
			&(palgo3->VL53L1_p_038),
			palgo3);





	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_f_019(palgo3);



	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_f_020(palgo3);



	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_f_021(palgo3);



	for (p = 0; p < palgo3->VL53L1_p_051; p++) {

		ppulse_data = &(palgo3->VL53L1_p_002[p]);



		if (status == VL53L1_ERROR_NONE)
			status =
				VL53L1_f_022(
					p,
					&(palgo3->VL53L1_p_010),
					palgo3);



		if (status == VL53L1_ERROR_NONE)
			status =
				VL53L1_f_023(
					p,
					&(palgo3->VL53L1_p_010),
					palgo3,
					pB->VL53L1_p_004,
					&(palgo3->VL53L1_p_052));



		if (status == VL53L1_ERROR_NONE) {
			status =
				VL53L1_f_023(
					p,
					&(palgo3->VL53L1_p_010),
					palgo3,
					0,
					&(palgo3->VL53L1_p_053));
		}



		if (status == VL53L1_ERROR_NONE) {
			status =
				VL53L1_f_023(
					p,
					&(palgo3->VL53L1_p_038),
					palgo3,
					0,
					&(palgo3->VL53L1_p_054));
		}



		if (status == VL53L1_ERROR_NONE)
			status =
				VL53L1_f_034(
					p,
					&(palgo3->VL53L1_p_052),
					palgo3,
					pfiltered);



		if (status == VL53L1_ERROR_NONE)
			status =
				VL53L1_f_035(
					p,
					ppost_cfg->noise_threshold,
					pfiltered,
					palgo3);

		if (status == VL53L1_ERROR_NONE)
			status =
			VL53L1_f_026(
			ppulse_data->VL53L1_p_025,
			ppost_cfg->sigma_estimator__sigma_ref_mm,
			palgo3->VL53L1_p_031,
			ppulse_data->VL53L1_p_055,
			ppost_cfg->algo__crosstalk_compensation_enable,
			&(palgo3->VL53L1_p_052),
			&(palgo3->VL53L1_p_053),
			&(palgo3->VL53L1_p_054),
			&(ppulse_data->VL53L1_p_005));



		if (status == VL53L1_ERROR_NONE)
			status =
				VL53L1_f_027(
					p,
					1,
					&(palgo3->VL53L1_p_010),
					palgo3);

	}



	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_f_028(
				ppost_cfg->hist_target_order,
				palgo3);



	for (p = 0; p < palgo3->VL53L1_p_051; p++) {

		ppulse_data = &(palgo3->VL53L1_p_002[p]);


		if (!(presults->active_results < presults->max_results))
			continue;




		if (ppulse_data->VL53L1_p_013 >
			ppost_cfg->signal_total_events_limit &&
			ppulse_data->VL53L1_p_025 < 0xFF) {

			prange_data =
			&(presults->VL53L1_p_002[presults->active_results]);

			if (status == VL53L1_ERROR_NONE)
				VL53L1_f_029(
						presults->active_results,
						ppost_cfg->valid_phase_low,
						ppost_cfg->valid_phase_high,
						ppost_cfg->sigma_thresh,
						&(palgo3->VL53L1_p_010),
						ppulse_data,
						prange_data);

			if (status == VL53L1_ERROR_NONE)
				status =
				VL53L1_f_011(
				pB->vcsel_width,
				pB->VL53L1_p_019,
				pB->total_periods_elapsed,
				pB->result__dss_actual_effective_spads,
				prange_data,
				histo_merge_nb);

			if (status == VL53L1_ERROR_NONE)
				VL53L1_f_012(
					ppost_cfg->gain_factor,
					ppost_cfg->range_offset_mm,
					prange_data);

			presults->active_results++;
		}

	}



	LOG_FUNCTION_END(status);

	return status;
}



VL53L1_Error VL53L1_f_034(
	uint8_t pulse_no,
	struct VL53L1_histogram_bin_data_t *ppulse,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo3,
	struct VL53L1_hist_gen4_algo_filtered_data_t *pfiltered)
{




	VL53L1_Error status = VL53L1_ERROR_NONE;

	struct VL53L1_hist_pulse_data_t *pdata = &(palgo3->VL53L1_p_002[pulse_no]);

	uint8_t lb = 0;
	uint8_t i = 0;
	int32_t suma = 0;
	int32_t sumb = 0;
	int32_t sumc = 0;

	LOG_FUNCTION_START("");

	pfiltered->VL53L1_p_023 = palgo3->VL53L1_p_023;
	pfiltered->VL53L1_p_022 = palgo3->VL53L1_p_022;
	pfiltered->VL53L1_p_024 = palgo3->VL53L1_p_024;



	for (lb = pdata->VL53L1_p_015; lb <= pdata->VL53L1_p_016; lb++) {

		i = lb % palgo3->VL53L1_p_031;


		VL53L1_f_013(
				i,
				pdata->VL53L1_p_055,
				ppulse,
				&suma,
				&sumb,
				&sumc);


		pfiltered->VL53L1_p_003[i] = suma;
		pfiltered->VL53L1_p_018[i] = sumb;
		pfiltered->VL53L1_p_001[i] = sumc;



		pfiltered->VL53L1_p_039[i] =
			(suma + sumb) -
			(sumc + palgo3->VL53L1_p_004);



		pfiltered->VL53L1_p_040[i] =
			(sumb + sumc) -
			(suma + palgo3->VL53L1_p_004);
	}

	return status;
}


VL53L1_Error VL53L1_f_035(
	uint8_t pulse_no,
	uint16_t noise_threshold,
	struct VL53L1_hist_gen4_algo_filtered_data_t *pfiltered,
	struct VL53L1_hist_gen3_algo_private_data_t *palgo3)
{



	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_Error func_status = VL53L1_ERROR_NONE;

	struct VL53L1_hist_pulse_data_t *pdata = &(palgo3->VL53L1_p_002[pulse_no]);

	uint8_t lb = 0;
	uint8_t i = 0;
	uint8_t j = 0;

	SUPPRESS_UNUSED_WARNING(noise_threshold);

	for (lb = pdata->VL53L1_p_015; lb < pdata->VL53L1_p_016; lb++) {

		i = lb % palgo3->VL53L1_p_031;
		j = (lb+1) % palgo3->VL53L1_p_031;

		if (i < palgo3->VL53L1_p_024 &&
			j < palgo3->VL53L1_p_024) {

			if (pfiltered->VL53L1_p_039[i] == 0 &&
				pfiltered->VL53L1_p_040[i] == 0)

				pfiltered->VL53L1_p_043[i] = 0;

			else if (pfiltered->VL53L1_p_039[i] >= 0 &&
					 pfiltered->VL53L1_p_040[i] >= 0)
				pfiltered->VL53L1_p_043[i] = 1;

			else if (pfiltered->VL53L1_p_039[i] < 0 &&
					 pfiltered->VL53L1_p_040[i] >= 0 &&
					 pfiltered->VL53L1_p_039[j] >= 0 &&
					 pfiltered->VL53L1_p_040[j] < 0)
				pfiltered->VL53L1_p_043[i] = 1;

			else
				pfiltered->VL53L1_p_043[i] = 0;


			if (pfiltered->VL53L1_p_043[i] > 0) {

				pdata->VL53L1_p_025 = lb;

				func_status =
					VL53L1_f_036(
					lb,
					pfiltered->VL53L1_p_003[i],
					pfiltered->VL53L1_p_018[i],
					pfiltered->VL53L1_p_001[i],
					0,
					0,
					0,
					palgo3->VL53L1_p_004,
					palgo3->VL53L1_p_031,
					&(pdata->VL53L1_p_014));

				if (func_status ==
					VL53L1_ERROR_DIVISION_BY_ZERO)
					pfiltered->VL53L1_p_043[i] = 0;

			}
		}
	}

	return status;
}


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
	uint32_t *pmean_phase)
{


	VL53L1_Error status = VL53L1_ERROR_DIVISION_BY_ZERO;

	int64_t mean_phase = VL53L1_MAX_ALLOWED_PHASE;
	int32_t mean_phase32;
	int64_t VL53L1_p_041 = 0;
	int64_t half_b_minus_amb = 0;


	VL53L1_p_041 = 4096 * ((int64_t)VL53L1_p_001 -
		(int64_t)cx - (int64_t)VL53L1_p_003 - (int64_t)ax);
	half_b_minus_amb = 4096 * ((int64_t)VL53L1_p_018 -
		(int64_t)bx - (int64_t)VL53L1_p_004);

	if (half_b_minus_amb != 0) {
		mean_phase = (4096 * VL53L1_p_041) + half_b_minus_amb;
		mean_phase = do_division_s(mean_phase, (half_b_minus_amb * 2));
		mean_phase += 2048;
		mean_phase += (4096 * (int64_t)bin);

		mean_phase = do_division_s((mean_phase + 1), 2);

		if (mean_phase < 0)
			mean_phase = 0;
		if (mean_phase > VL53L1_MAX_ALLOWED_PHASE)
			mean_phase = VL53L1_MAX_ALLOWED_PHASE;

		mean_phase32 = (int32_t)mean_phase;
		mean_phase32 = mean_phase32 %
			((int32_t)VL53L1_p_031 * 2048);
		mean_phase = mean_phase32;

		status = VL53L1_ERROR_NONE;
	}

	*pmean_phase = (uint32_t)mean_phase;

	return status;
}
