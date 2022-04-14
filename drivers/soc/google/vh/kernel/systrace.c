// SPDX-License-Identifier: GPL-2.0
/*
 * Systrace Kernel Tracing Integration
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/module.h>

#include <trace/events/power.h>
#define CREATE_TRACE_POINTS
#include "systrace.h"

EXPORT_TRACEPOINT_SYMBOL_GPL(0);

#define SUSPEND_CLOCK_STRING_LEN			(32)

static void note_suspend_resume(void *data, const char *action, int val, bool start)
{
	if (unlikely(trace_clock_set_rate_enabled())) {
		char clock_name[SUSPEND_CLOCK_STRING_LEN] = {0};
		scnprintf(clock_name, SUSPEND_CLOCK_STRING_LEN, "S-%s-%d", action, val);
		trace_clock_set_rate(clock_name, start, raw_smp_processor_id());
	}
}

static int systrace_init(void)
{
	int ret = register_trace_suspend_resume(note_suspend_resume, NULL);
	if (ret)
		return ret;

	return 0;
}

static void systrace_exit(void)
{
	unregister_trace_suspend_resume(note_suspend_resume, NULL);
}

module_init(systrace_init);
module_exit(systrace_exit);

MODULE_LICENSE("GPL");
