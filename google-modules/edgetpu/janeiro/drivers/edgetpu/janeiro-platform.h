/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform device driver for the Google Edge TPU ML accelerator.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#ifndef __JANEIRO_PLATFORM_H__
#define __JANEIRO_PLATFORM_H__

#include "edgetpu-internal.h"
#include "edgetpu-mobile-platform.h"

#define to_janeiro_dev(etdev)                                                                      \
	container_of((to_mobile_dev(etdev)), struct janeiro_platform_dev, mobile_dev)

struct janeiro_platform_dev {
	struct edgetpu_mobile_platform_dev mobile_dev;
};

#endif /* __JANEIRO_PLATFORM_H__ */
