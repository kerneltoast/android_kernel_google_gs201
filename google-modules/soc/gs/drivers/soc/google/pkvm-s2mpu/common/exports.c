// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 - Google LLC
 */
#include "kvm_s2mpu.h"
#include <soc/google/pkvm-s2mpu.h>

int PER_DRIVER_FN(pkvm_s2mpu_of_link)(struct device *parent)
{
	return __pkvm_s2mpu_of_link(parent);
}
EXPORT_SYMBOL_GPL(PER_DRIVER_FN(pkvm_s2mpu_of_link));

struct device *PER_DRIVER_FN(pkvm_s2mpu_of_parse)(struct device *parent)
{
	return __pkvm_s2mpu_of_parse(parent);
}
EXPORT_SYMBOL_GPL(PER_DRIVER_FN(pkvm_s2mpu_of_parse));

int PER_DRIVER_FN(pkvm_s2mpu_suspend)(struct device *dev)
{
	return __pkvm_s2mpu_suspend(dev);
}
EXPORT_SYMBOL_GPL(PER_DRIVER_FN(pkvm_s2mpu_suspend));

int PER_DRIVER_FN(pkvm_s2mpu_resume)(struct device *dev)
{
	return __pkvm_s2mpu_resume(dev);
}
EXPORT_SYMBOL_GPL(PER_DRIVER_FN(pkvm_s2mpu_resume));
