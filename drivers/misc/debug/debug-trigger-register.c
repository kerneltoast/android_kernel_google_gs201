// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Registration of SOC specific Pixel Debug Tests.
 *
 * Copyright (C) 2019 Google LLC
 */
#include <linux/printk.h>
#include <linux/debug-test.h>

struct debug_trigger soc_test_trigger;
EXPORT_SYMBOL(soc_test_trigger);

void debug_trigger_register(struct debug_trigger *soc_trigger, char *arch_name)
{
	pr_crit("DEBUG TEST: [%s] test triggers are registered!", arch_name);
	soc_test_trigger.hard_lockup = soc_trigger->hard_lockup;
	soc_test_trigger.cold_reset = soc_trigger->cold_reset;
	soc_test_trigger.watchdog_emergency_reset =
		soc_trigger->watchdog_emergency_reset;
}
