/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2020 Google LLC
 *
 * Authors: Jaegeuk Kim <jaegeuk@google.com>
 */

#ifndef _UFS_PIXEL_H_
#define _UFS_PIXEL_H_

#include <asm/unaligned.h>
#include "ufshcd.h"

/* 1% lifetime C */
#define HEALTH_DESC_DEFAULT_PE_CYCLE		3000
#define HEALTH_DESC_PARAM_AVG_PE_CYCLE		0xD
#define HEALTH_DESC_PARAM_LIFE_TIME_EST_C	0x24
#endif
