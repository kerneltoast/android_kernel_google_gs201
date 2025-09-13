// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/topology.h>
#include <trace/hooks/preemptirq.h>
#include <trace/events/sched.h>

extern void note_irq_disable(void *u1, unsigned long u2, unsigned long u3);
extern void test_irq_disable_long(void *u1, unsigned long u2, unsigned long u3);
extern void test_preempt_disable_long(void *u1, unsigned long u2, unsigned long u3);
extern void note_preempt_disable(void *u1, unsigned long u2, unsigned long u3);
extern void note_context_switch(void *u1, bool u2, struct task_struct *u3,
				struct task_struct *next, unsigned int prev_state);
extern int preemptirq_long_init(void);

static int vh_long_preemptirq_init(void)
{
	int ret = 0;

	ret = preemptirq_long_init();
	if (ret)
		return ret;

	ret = register_trace_android_rvh_irqs_disable(note_irq_disable, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_irqs_enable(test_irq_disable_long, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_preempt_disable(note_preempt_disable, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_preempt_enable(test_preempt_disable_long, NULL);
	if (ret)
		return ret;

	ret = register_trace_sched_switch(note_context_switch, NULL);
	if (ret)
		return ret;

	return ret;
}

module_init(vh_long_preemptirq_init);
MODULE_LICENSE("GPL v2");
