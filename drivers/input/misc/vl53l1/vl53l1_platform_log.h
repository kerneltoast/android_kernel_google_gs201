/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file  vl53l1_platform_log.h
 *
 * @brief EwokPlus25 platform logging function definition
 */


#ifndef _VL53L1_PLATFORM_LOG_H_
#define _VL53L1_PLATFORM_LOG_H_

#include <linux/string.h>

#ifdef VL53L1_LOG_ENABLE
#include "vl53l1_platform_user_config.h"

#ifdef _MSC_VER
#define EWOKPLUS_EXPORTS __declspec(dllexport)
#else
#define EWOKPLUS_EXPORTS
#endif

#include "vl53l1_types.h"


#include <linux/time.h>

	/**
	 * @brief Set the level, output and specific functions for module
	 * logging.
	 *
	 *
	 * @param filename  - full path of output log file, NULL for print to
	 * stdout
	 *
	 * @param modules   - Module or None or All to trace
	 *                      VL53L1_TRACE_MODULE_NONE
	 *                      VL53L1_TRACE_MODULE_API
	 *                      VL53L1_TRACE_MODULE_CORE
	 *                      VL53L1_TRACE_MODULE_TUNING
	 *                      VL53L1_TRACE_MODULE_CHARACTERISATION
	 *                      VL53L1_TRACE_MODULE_PLATFORM
	 *                      VL53L1_TRACE_MODULE_ALL
	 *
	 * @param level     - trace level
	 *                      VL53L1_TRACE_LEVEL_NONE
	 *                      VL53L1_TRACE_LEVEL_ERRORS
	 *                      VL53L1_TRACE_LEVEL_WARNING
	 *                      VL53L1_TRACE_LEVEL_INFO
	 *                      VL53L1_TRACE_LEVEL_DEBUG
	 *                      VL53L1_TRACE_LEVEL_ALL
	 *                      VL53L1_TRACE_LEVEL_IGNORE
	 *
	 *  @param functions - function level to trace;
	 *                      VL53L1_TRACE_FUNCTION_NONE
	 *                      VL53L1_TRACE_FUNCTION_I2C
	 *                      VL53L1_TRACE_FUNCTION_ALL
	 *
	 * @return status - always VL53L1_ERROR_NONE
	 *
	 */

#define VL53L1_TRACE_LEVEL_NONE 0x00000000
#define VL53L1_TRACE_LEVEL_ERRORS 0x00000001
#define VL53L1_TRACE_LEVEL_WARNING 0x00000002
#define VL53L1_TRACE_LEVEL_INFO 0x00000004
#define VL53L1_TRACE_LEVEL_DEBUG 0x00000008
#define VL53L1_TRACE_LEVEL_ALL 0x00000010
#define VL53L1_TRACE_LEVEL_IGNORE 0x00000020

#define VL53L1_TRACE_FUNCTION_NONE 0x00000000
#define VL53L1_TRACE_FUNCTION_I2C 0x00000001
#define VL53L1_TRACE_FUNCTION_ALL 0x7fffffff

#define VL53L1_TRACE_MODULE_NONE 0x00000000
#define VL53L1_TRACE_MODULE_API 0x00000001
#define VL53L1_TRACE_MODULE_CORE 0x00000002
#define VL53L1_TRACE_MODULE_PROTECTED 0x00000004
#define VL53L1_TRACE_MODULE_HISTOGRAM 0x00000008
#define VL53L1_TRACE_MODULE_REGISTERS 0x00000010
#define VL53L1_TRACE_MODULE_PLATFORM 0x00000020
#define VL53L1_TRACE_MODULE_NVM 0x00000040
#define VL53L1_TRACE_MODULE_CALIBRATION_DATA 0x00000080
#define VL53L1_TRACE_MODULE_NVM_DATA 0x00000100
#define VL53L1_TRACE_MODULE_HISTOGRAM_DATA 0x00000200
#define VL53L1_TRACE_MODULE_RANGE_RESULTS_DATA 0x00000400
#define VL53L1_TRACE_MODULE_XTALK_DATA 0x00000800
#define VL53L1_TRACE_MODULE_OFFSET_DATA 0x00001000
#define VL53L1_TRACE_MODULE_DATA_INIT 0x00002000
#define VL53L1_TRACE_MODULE_REF_SPAD_CHAR 0x00004000
#define VL53L1_TRACE_MODULE_SPAD_RATE_MAP 0x00008000
#ifdef PAL_EXTENDED
#define VL53L1_TRACE_MODULE_SPAD 0x01000000
#define VL53L1_TRACE_MODULE_FMT 0x02000000
#define VL53L1_TRACE_MODULE_UTILS 0x04000000
#define VL53L1_TRACE_MODULE_BENCH_FUNCS 0x08000000
#endif
#define VL53L1_TRACE_MODULE_CUSTOMER_API 0x40000000
#define VL53L1_TRACE_MODULE_ALL 0x7fffffff

extern void log_trace_print(uint32_t module, uint32_t level,
	uint32_t function, const char *format, ...);

#define _LOG_TRACE_PRINT_FMT(module, level, function, format, ...) \
	log_trace_print(module, level, function, \
		KERN_INFO "<TRACE> " format, ##__VA_ARGS__)
#define _LOG_TRACE_PRINT(module, level, function, ...) \
	_LOG_TRACE_PRINT_FMT(module, level, function, ##__VA_ARGS__)
#define _LOG_FUNCTION_START(module, fmt, ...) \
	log_trace_print(module, VL53L1_TRACE_LEVEL_NONE, \
		VL53L1_TRACE_FUNCTION_ALL, \
		KERN_INFO "<START> %s "fmt"\n", __func__, ##__VA_ARGS__)
#define _LOG_FUNCTION_END(module, status, ...) \
	log_trace_print(module, VL53L1_TRACE_LEVEL_NONE, \
		VL53L1_TRACE_FUNCTION_ALL, \
		KERN_INFO "<END>   %s %d\n", __func__, status)
#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...) \
	log_trace_print(module, VL53L1_TRACE_LEVEL_NONE, \
		VL53L1_TRACE_FUNCTION_ALL, \
		KERN_INFO "<END  > %s %d"fmt"\n", __func__, status, \
		##__VA_ARGS__)
#define _LOG_GET_TRACE_FUNCTIONS() 0
#define _LOG_SET_TRACE_FUNCTIONS(functions)

#define _LOG_STRING_BUFFER(x) char x[VL53L1_MAX_STRING_LENGTH]


#else /* VL53L1_LOG_ENABLE - no logging */
#include "vl53l1_platform_user_config.h"

#define _LOG_TRACE_PRINT(module, level, function, ...)
#define _LOG_FUNCTION_START(module, fmt, ...)
#define _LOG_FUNCTION_END(module, status, ...)
#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...)
#define _LOG_GET_TRACE_FUNCTIONS() 0
#define _LOG_SET_TRACE_FUNCTIONS(functions)
#define _LOG_STRING_BUFFER(x)

#endif /* VL53L1_LOG_ENABLE */

#endif /* _VL53L1_PLATFORM_LOG_H_ */
