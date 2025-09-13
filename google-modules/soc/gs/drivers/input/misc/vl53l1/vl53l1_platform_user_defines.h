/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */


#ifndef _VL53L1_PLATFORM_USER_DEFINES_H_
#define _VL53L1_PLATFORM_USER_DEFINES_H_


#ifdef __KERNEL__
#include <linux/math64.h>
#endif

/**
 * @file   vl53l1_platform_user_defines.h
 *
 * @brief  All end user OS/platform/application definitions
 */


/**
 * @def do_division_u
 * @brief customer supplied division operation - 64-bit unsigned
 *
 * @param dividend      unsigned 64-bit numerator
 * @param divisor       unsigned 64-bit denominator
 */
#ifdef __KERNEL__
#define do_division_u(dividend, divisor) div64_u64(dividend, divisor)
#else
#define do_division_u(dividend, divisor) (dividend / divisor)
#endif

/**
 * @def do_division_s
 * @brief customer supplied division operation - 64-bit signed
 *
 * @param dividend      signed 64-bit numerator
 * @param divisor       signed 64-bit denominator
 */
#ifdef __KERNEL__
#define do_division_s(dividend, divisor) div64_s64(dividend, divisor)
#else
#define do_division_s(dividend, divisor) (dividend / divisor)
#endif

#define WARN_OVERRIDE_STATUS(__X__)\
	trace_print(VL53L1_TRACE_LEVEL_WARNING, #__X__)


#define DISABLE_WARNINGS()
#define ENABLE_WARNINGS()




#endif /* _VL53L1_PLATFORM_USER_DEFINES_H_ */
