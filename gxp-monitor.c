// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP abstract monitor interface for BPM and GEM.
 *
 * Copyright (C) 2024 Google LLC
 */

#include <linux/platform_device.h>

#include "gxp-config.h"
#include "gxp-internal.h"
#include "gxp-monitor.h"

/* GEM has a higher priority than BPM if both are available. */
#if GXP_USE_GEM
/* Use the first counter in GEM to record events. */
#define GEM_COUNTER_ID 0
#include "gxp-gem.h"
#elif GXP_USE_BPM
#include "gxp-bpm.h"
#endif /* GXP_USE_GEM */

void gxp_monitor_set_count_read_data(struct gxp_dev *gxp)
{
#if GXP_USE_GEM
	gxp_gem_set_counter_config(gxp, GEM_COUNTER_ID, true, GEM_EVENT_READ_DATA);
#elif GXP_USE_BPM
	gxp_bpm_configure(gxp, GXP_REG_MCU_ID, INST_BPM_OFFSET, BPM_EVENT_READ_XFER);
#endif /* GXP_USE_GEM */
}

void gxp_monitor_start(struct gxp_dev *gxp)
{
#if GXP_USE_GEM
	gxp_gem_set_config(gxp, true);
#elif GXP_USE_BPM
	gxp_bpm_start(gxp, GXP_REG_MCU_ID);
#endif /* GXP_USE_GEM */
}

void gxp_monitor_stop(struct gxp_dev *gxp)
{
#if GXP_USE_GEM
	gxp_gem_set_config(gxp, false);
#elif GXP_USE_BPM
	gxp_bpm_stop(gxp, GXP_REG_MCU_ID);
#endif /* GXP_USE_GEM */
}

u32 gxp_monitor_get_count_read_data(struct gxp_dev *gxp)
{
#if GXP_USE_GEM
	return gxp_gem_get_counter_snapshot(gxp, GEM_COUNTER_ID);
#elif GXP_USE_BPM
	return gxp_bpm_read_counter(gxp, GXP_REG_MCU_ID, INST_BPM_OFFSET);
#else
	return 0;
#endif /* GXP_USE_GEM */
}
