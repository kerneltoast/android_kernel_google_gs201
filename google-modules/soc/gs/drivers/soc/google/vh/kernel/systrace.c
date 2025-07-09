// SPDX-License-Identifier: GPL-2.0
/*
 * Systrace Kernel Tracing Integration
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/module.h>

#define CREATE_TRACE_POINTS
#include <trace/hooks/systrace.h>

EXPORT_TRACEPOINT_SYMBOL_GPL(0);

static int systrace_init(void)
{
	return 0;
}

static void systrace_exit(void)
{
}

module_init(systrace_init);
module_exit(systrace_exit);

MODULE_LICENSE("GPL");
