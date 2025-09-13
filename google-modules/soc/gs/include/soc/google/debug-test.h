/* SPDX-License-Identifier: GPL-2.0-only */
/*
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
	void (*cacheflush)(char *arg);
	void (*cpucontext)(char *arg);
	void (*arraydump)(char *arg);
	void (*scandump)(char *arg);
	void (*el3_assert)(char *arg);
	void (*el3_panic)(char *arg);
	void (*ecc)(char *arg);
};

#if IS_ENABLED(CONFIG_PIXEL_DEBUG_TEST)
extern void debug_trigger_register(struct debug_trigger *soc_trigger, char *arch_name);
#else
static inline void debug_trigger_register(struct debug_trigger *soc_trigger, char *arch_name) {}
#endif

#endif /* DEBUG_TEST_H */
