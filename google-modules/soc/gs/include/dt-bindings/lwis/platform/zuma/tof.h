/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS ZUMA TOF Interrupt And Event Defines
 *
 * Copyright (c) 2022 Google, LLC
 */

#ifndef DT_BINDINGS_LWIS_PLATFORM_ZUMA_TOF_H_
#define DT_BINDINGS_LWIS_PLATFORM_ZUMA_TOF_H_

#include <dt-bindings/lwis/platform/common.h>

/* clang-format off */

#define TOF_IRQ_BASE (HW_EVENT_MASK + 0)

#define TOF_IRQ_DATA_POLLING 0

/* clang-format on */

#define LWIS_PLATFORM_EVENT_ID_TOF_DATA_POLLING EVENT_ID(TOF_IRQ_BASE, TOF_IRQ_DATA_POLLING)

#endif /* DT_BINDINGS_LWIS_PLATFORM_ZUMA_TOF_H_ */
