// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */






#include "../inc/vl53l1_api_core.h"
#include "../inc/vl53l1_api_strings.h"
#include "../inc/vl53l1_error_codes.h"
#include "../inc/vl53l1_error_strings.h"

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_API, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_API, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_API, status, fmt, \
			##__VA_ARGS__)


VL53L1_Error VL53L1_get_range_status_string(
	uint8_t RangeStatus,
	char *pRangeStatusString)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;

	LOG_FUNCTION_START("");

#ifdef VL53L1_USE_EMPTY_STRING
	SUPPRESS_UNUSED_WARNING(RangeStatus);
	VL53L1_COPYSTRING(pRangeStatusString, "");
#else
	switch (RangeStatus) {
	case 0:
		VL53L1_COPYSTRING(pRangeStatusString,
			VL53L1_STRING_RANGESTATUS_RANGEVALID);
		break;
	case 1:
		VL53L1_COPYSTRING(pRangeStatusString,
			VL53L1_STRING_RANGESTATUS_SIGMA);
		break;
	case 2:
		VL53L1_COPYSTRING(pRangeStatusString,
			VL53L1_STRING_RANGESTATUS_SIGNAL);
		break;
	case 3:
		VL53L1_COPYSTRING(pRangeStatusString,
			VL53L1_STRING_RANGESTATUS_MINRANGE);
		break;
	case 4:
		VL53L1_COPYSTRING(pRangeStatusString,
			VL53L1_STRING_RANGESTATUS_PHASE);
		break;
	case 5:
		VL53L1_COPYSTRING(pRangeStatusString,
			VL53L1_STRING_RANGESTATUS_HW);
		break;

	default:
		VL53L1_COPYSTRING(pRangeStatusString,
			VL53L1_STRING_RANGESTATUS_NONE);
	}
#endif

	LOG_FUNCTION_END(status);
	return status;
}


VL53L1_Error VL53L1_get_pal_state_string(
	VL53L1_State PalStateCode,
	char *pPalStateString)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;

	LOG_FUNCTION_START("");

#ifdef VL53L1_USE_EMPTY_STRING
	SUPPRESS_UNUSED_WARNING(PalStateCode);
	VL53L1_COPYSTRING(pPalStateString, "");
#else
	switch (PalStateCode) {
	case VL53L1_STATE_POWERDOWN:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_POWERDOWN);
		break;
	case VL53L1_STATE_WAIT_STATICINIT:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_WAIT_STATICINIT);
		break;
	case VL53L1_STATE_STANDBY:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_STANDBY);
		break;
	case VL53L1_STATE_IDLE:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_IDLE);
		break;
	case VL53L1_STATE_RUNNING:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_RUNNING);
		break;
	case VL53L1_STATE_RESET:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_RESET);
		break;
	case VL53L1_STATE_UNKNOWN:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_UNKNOWN);
		break;
	case VL53L1_STATE_ERROR:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_ERROR);
		break;

	default:
		VL53L1_COPYSTRING(pPalStateString,
			VL53L1_STRING_STATE_UNKNOWN);
	}
#endif

	LOG_FUNCTION_END(status);
	return status;
}

VL53L1_Error VL53L1_get_sequence_steps_info(
		VL53L1_SequenceStepId SequenceStepId,
		char *pSequenceStepsString)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;

	LOG_FUNCTION_START("");

#ifdef VL53L1_USE_EMPTY_STRING
	SUPPRESS_UNUSED_WARNING(SequenceStepId);
	VL53L1_COPYSTRING(pSequenceStepsString, "");
#else
	switch (SequenceStepId) {
	case VL53L1_SEQUENCESTEP_VHV:
		VL53L1_COPYSTRING(pSequenceStepsString,
				VL53L1_STRING_SEQUENCESTEP_VHV);
		break;
	case VL53L1_SEQUENCESTEP_PHASECAL:
		VL53L1_COPYSTRING(pSequenceStepsString,
				VL53L1_STRING_SEQUENCESTEP_PHASECAL);
		break;
	case VL53L1_SEQUENCESTEP_REFPHASE:
		VL53L1_COPYSTRING(pSequenceStepsString,
				VL53L1_STRING_SEQUENCESTEP_DSS1);
		break;
	case VL53L1_SEQUENCESTEP_DSS1:
		VL53L1_COPYSTRING(pSequenceStepsString,
				VL53L1_STRING_SEQUENCESTEP_DSS1);
		break;
	case VL53L1_SEQUENCESTEP_DSS2:
		VL53L1_COPYSTRING(pSequenceStepsString,
				VL53L1_STRING_SEQUENCESTEP_DSS2);
		break;
	case VL53L1_SEQUENCESTEP_MM1:
		VL53L1_COPYSTRING(pSequenceStepsString,
				VL53L1_STRING_SEQUENCESTEP_MM1);
		break;
	case VL53L1_SEQUENCESTEP_MM2:
		VL53L1_COPYSTRING(pSequenceStepsString,
				VL53L1_STRING_SEQUENCESTEP_MM2);
		break;
	case VL53L1_SEQUENCESTEP_RANGE:
		VL53L1_COPYSTRING(pSequenceStepsString,
				VL53L1_STRING_SEQUENCESTEP_RANGE);
		break;
	default:
		Status = VL53L1_ERROR_INVALID_PARAMS;
	}
#endif

	LOG_FUNCTION_END(Status);

	return Status;
}

VL53L1_Error VL53L1_get_limit_check_info(uint16_t LimitCheckId,
	char *pLimitCheckString)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;

	LOG_FUNCTION_START("");

#ifdef VL53L1_USE_EMPTY_STRING
	SUPPRESS_UNUSED_WARNING(LimitCheckId);
	VL53L1_COPYSTRING(pLimitCheckString, "");
#else
	switch (LimitCheckId) {
	case VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE:
		VL53L1_COPYSTRING(pLimitCheckString,
			VL53L1_STRING_CHECKENABLE_SIGMA_FINAL_RANGE);
		break;
	case VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
		VL53L1_COPYSTRING(pLimitCheckString,
			VL53L1_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);
		break;
	default:
		VL53L1_COPYSTRING(pLimitCheckString,
				VL53L1_STRING_UNKNOWN_ERROR_CODE);
	}
#endif

	LOG_FUNCTION_END(Status);
	return Status;
}
