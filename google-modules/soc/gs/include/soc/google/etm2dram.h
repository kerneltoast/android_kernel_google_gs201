/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Interfaces to control debugcore emt2dram plugin.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef _ETM2DRAM_H_
#define _ETM2DRAM_H_

#if IS_ENABLED(CONFIG_ETM2DRAM)
int etm2dram_delayed_start(int delay_secs);
int etm2dram_cancel_delayed_start(void);
#else
static inline int etm2dram_delayed_start(int delay_secs) { return 0; }
static inline int etm2dram_cancel_delayed_start(void) { return 0; }
#endif

#endif /* _ETM2DRAM_H_ */
