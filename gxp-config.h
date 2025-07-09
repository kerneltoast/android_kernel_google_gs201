/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Include all configuration files for GXP.
 *
 * Copyright (C) 2020 Google LLC
 */

#ifndef __GXP_CONFIG_H__
#define __GXP_CONFIG_H__

#include <gcip/gcip-config.h>

#if IS_ENABLED(CONFIG_AMALTHEA)

#include "amalthea/config.h"

#else /* unknown */

#error "Unknown GXP config"

#endif /* unknown */

#if IS_ENABLED(CONFIG_GXP_ZEBU) || IS_ENABLED(CONFIG_GXP_GEM5)
#define GXP_TIME_DELAY_FACTOR 100
#elif IS_ENABLED(CONFIG_GXP_IP_ZEBU)
#define GXP_TIME_DELAY_FACTOR 500
#elif IS_ENABLED(CONFIG_GXP_TEST) && IS_ENABLED(CONFIG_DEBUG_KMEMLEAK)
#define GXP_TIME_DELAY_FACTOR 5
#else
#define GXP_TIME_DELAY_FACTOR 1
#endif

#define DOORBELL_COUNT 32

#define SYNC_BARRIER_COUNT 16

#ifndef GXP_ENABLE_PARTIAL_LAP
#define GXP_ENABLE_PARTIAL_LAP 0
#endif

#ifndef GXP_HAS_LAP
#define GXP_HAS_LAP 1
#endif

#ifndef GXP_HAS_MCU
#define GXP_HAS_MCU 1
#endif

#ifndef GXP_MMU_REQUIRE_ATTACH
#define GXP_MMU_REQUIRE_ATTACH 0
#endif

#ifndef GXP_HAS_GSA
#define GXP_HAS_GSA 1
#endif

#ifndef GXP_DUMP_INTERRUPT_POLARITY_REGISTER
#define GXP_DUMP_INTERRUPT_POLARITY_REGISTER 1
#endif

#ifndef GXP_ENABLE_DEBUG_DUMP
#define GXP_ENABLE_DEBUG_DUMP 1
#endif

#ifndef GXP_AUTO_PSM
#define GXP_AUTO_PSM 0
#endif

#define GXP_DEBUG_DUMP_IOVA_BASE (0xF5000000)
#define GXP_TELEMETRY_IOVA_BASE (0xF6000000)

/*
 * Only supports interop with TPU when
 * 1. Unit testing, or
 * 2. Production on Android (to exclude vanilla Linux for bringup) but not GEM5.
 */
#define HAS_TPU_EXT (!IS_ENABLED(CONFIG_GXP_GEM5) &&				\
		    !IS_ENABLED(CONFIG_GXP_IP_ZEBU) &&				\
		    !IS_ENABLED(CONFIG_GXP_ZEBU))

#define GXP_USE_GEM GXP_HAS_GEM
#define GXP_USE_BPM (!GXP_USE_GEM && GXP_HAS_BPM && GXP_HAS_MCU)

#endif /* __GXP_CONFIG_H__ */
