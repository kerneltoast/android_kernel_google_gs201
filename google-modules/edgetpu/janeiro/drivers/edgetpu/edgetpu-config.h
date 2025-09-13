/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Defines chipset dependent configuration.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#ifndef __EDGETPU_CONFIG_H__
#define __EDGETPU_CONFIG_H__

#if IS_ENABLED(CONFIG_JANEIRO)
#include "janeiro/config.h"

#else /* unknown */

#error "Unknown EdgeTPU config"

#endif /* unknown */

#define EDGETPU_DEFAULT_FIRMWARE_NAME "google/edgetpu-" DRIVER_NAME ".fw"
#define EDGETPU_TEST_FIRMWARE_NAME "google/edgetpu-" DRIVER_NAME "-test.fw"

#ifndef EDGETPU_NUM_CORES
#define EDGETPU_NUM_CORES 1
#endif

#endif /* __EDGETPU_CONFIG_H__ */
