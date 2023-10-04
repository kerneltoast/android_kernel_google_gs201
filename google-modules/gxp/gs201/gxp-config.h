/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Include all configuration files for GXP.
 *
 * Copyright (C) 2020 Google LLC
 */

#ifndef __GXP_CONFIG_H__
#define __GXP_CONFIG_H__

#if IS_ENABLED(CONFIG_AMALTHEA)

#include "amalthea/config.h"

#else /* unknown */

#error "Unknown GXP config"

#endif /* unknown */

#if IS_ENABLED(CONFIG_GXP_GEM5)
#undef GXP_NUM_CORES
#define GXP_NUM_CORES 1
#endif

#define GXP_NUM_PREALLOCATED_DOMAINS GXP_NUM_CORES

#if defined(CONFIG_GXP_ZEBU) || defined(CONFIG_GXP_IP_ZEBU)
#define GXP_TIME_DELAY_FACTOR 20
#else
#define GXP_TIME_DELAY_FACTOR 1
#endif

#define DOORBELL_COUNT 32

#define SYNC_BARRIER_COUNT 16

/* Core address space starts at Inst_BPM block */
#define GXP_CORE_0_BASE GXP_REG_CORE_0_INST_BPM
#define GXP_CORE_SIZE (GXP_REG_CORE_1_INST_BPM - GXP_REG_CORE_0_INST_BPM)

/* LPM address space starts at lpm_version register */
#define GXP_LPM_BASE GXP_REG_LPM_VERSION
#define GXP_LPM_PSM_0_BASE GXP_REG_LPM_PSM_0
#define GXP_LPM_PSM_SIZE (GXP_REG_LPM_PSM_1 - GXP_REG_LPM_PSM_0)

#endif /* __GXP_CONFIG_H__ */
