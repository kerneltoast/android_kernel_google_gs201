/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Amalthea context related macros.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __AMALTHEA_CONTEXT_H__
#define __AMALTHEA_CONTEXT_H__

/* The stream IDs used for each core. */
#define INST_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4) | (0 << 3))
#define DATA_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4) | (1 << 3))
#define IDMA_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4))

#endif /* __AMALTHEA_CONTEXT_H__ */
