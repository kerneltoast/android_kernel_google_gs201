/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Include all configuration files for Amalthea.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __AMALTHEA_CONFIG_H__
#define __AMALTHEA_CONFIG_H__

#include <linux/sizes.h>

#define GXP_DRIVER_NAME "gxp_platform"
#define DSP_FIRMWARE_DEFAULT_PREFIX "gxp_fw_core"

#define AUR_DVFS_DOMAIN 17

#define GXP_NUM_CORES 4
#define GXP_NUM_MAILBOXES GXP_NUM_CORES
#define GXP_NUM_WAKEUP_DOORBELLS GXP_NUM_CORES

/* The total size of the configuration region. */
#define GXP_SHARED_BUFFER_SIZE SZ_256K
/* Size of slice per VD. */
#define GXP_SHARED_SLICE_SIZE 0x9000 /* 36K */
/* At most GXP_NUM_CORES VDs can be supported on Amalthea. */
#define GXP_NUM_SHARED_SLICES GXP_NUM_CORES

/* Enable a partial LAP workaround for sync barrier access only. */
#define GXP_ENABLE_PARTIAL_LAP 1

#define GXP_HAS_MCU 0

#define GXP_HAS_BPM 1
#define GXP_HAS_GEM 0

#define GXP_HAS_CMU 1

#include "config-pwr-state.h"
#include "context.h"
#include "csrs.h"
#include "iova.h"
#include "lpm.h"
#include "mailbox-regs.h"

#endif /* __AMALTHEA_CONFIG_H__ */
