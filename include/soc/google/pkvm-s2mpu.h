/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __PKVM_S2MPU_H
#define __PKVM_S2MPU_H

#include <linux/device.h>

int pkvm_s2mpu_of_link(struct device *parent);

static inline bool pkvm_s2mpu_ready(struct device *dev)
{
	return !!platform_get_drvdata(to_platform_device(dev));
}

#endif	/* __PKVM_S2MPU_H */
