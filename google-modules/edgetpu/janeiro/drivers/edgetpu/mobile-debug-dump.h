/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Module that defines structure to retrieve debug dump segments
 * specific to the family of EdgeTPUs for mobile devices.
 *
 * Copyright (C) 2021-2022 Google LLC
 */

#ifndef __MOBILE_DEBUG_DUMP_H__
#define __MOBILE_DEBUG_DUMP_H__

#include "edgetpu-debug-dump.h"

struct mobile_sscd_info {
	void *pdata; /* SSCD platform data */
	void *dev; /* SSCD platform device */
};

#endif /* __MOBILE_DEBUG_DUMP_H__ */
