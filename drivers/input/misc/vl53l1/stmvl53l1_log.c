// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file /stmvl53l1_module.c  vl53l1_module  ST VL53L1 linux kernel module
 *
 * This is implementation of low level driver trace support
 */
#include <linux/module.h>
#include <linux/printk.h>

#include "stmvl53l1.h"

#ifdef VL53L1_LOG_ENABLE

static bool trace_function;
static int trace_module;
static int trace_level;

module_param(trace_function, bool, 0644);
MODULE_PARM_DESC(trace_function,
	"allow tracing of low level function entry and exit");

module_param(trace_module, int, 0644);
MODULE_PARM_DESC(trace_module,
	"control tracing of low level per module");

module_param(trace_level, int, 0644);
MODULE_PARM_DESC(trace_level,
	"control tracing of low level per level");

void log_trace_print(uint32_t module, uint32_t level, uint32_t function,
	const char *format, ...)
{
	va_list args;

	if (function && !trace_function)
		return;

	if (!(module & trace_module))
		return;

	if (level > trace_level)
		return;

	va_start(args, format);
	vprintk(format, args);
	va_end(args);
}

#endif
