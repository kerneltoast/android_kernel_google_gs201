/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Interface to SOC specific Pixel Debug Tests
 *
 * Copyright (C) 2019 Google LLC
 */
#ifndef DEBUG_TEST_H
#define DEBUG_TEST_H

struct debug_trigger {
	void (*hard_lockup)(char *arg);
	void (*cold_reset)(char *arg);
	void (*watchdog_emergency_reset)(char *arg);
	void (*halt)(char *arg);
	void (*arraydump)(char *arg);
	void (*scandump)(char *arg);
};

#if IS_ENABLED(CONFIG_PIXEL_DEBUG_TEST)
extern void debug_trigger_register(struct debug_trigger *soc_trigger, char *arch_name);
#else
#define debug_trigger_register(a, b)	do { } while (0)
#endif

#endif /* DEBUG_TEST_H */
