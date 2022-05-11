/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __PKVM_S2MPU_H
#define __PKVM_S2MPU_H

#include <linux/device.h>

/*
 * Parse the 's2mpus' DT property of 'parent' and create a device link
 * to all referenced S2MPU devices.
 */
int pkvm_s2mpu_of_link(struct device *parent);

/*
 * Parse the 's2mpu' DT property of 'parent' and return a pointer to
 * the referenced S2MPU device, or NULL if the property does not exist.
 */
struct device *pkvm_s2mpu_of_parse(struct device *parent);

int pkvm_s2mpu_suspend(struct device *dev);
int pkvm_s2mpu_resume(struct device *dev);

static inline bool pkvm_s2mpu_ready(struct device *dev)
{
	return !!platform_get_drvdata(to_platform_device(dev));
}

#endif	/* __PKVM_S2MPU_H */
