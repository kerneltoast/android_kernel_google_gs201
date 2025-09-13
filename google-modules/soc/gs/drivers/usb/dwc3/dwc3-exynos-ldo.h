/* SPDX-License-Identifier: GPL-2.0 */
/*
 * exynos-otg.h - Samsung EXYNOS OTG Header
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 */

#ifndef __LINUX_USB_DWC3_EXYNOS_LDO_H
#define __LINUX_USB_DWC3_EXYNOS_LDO_H

#include <soc/google/exynos-pd_hsi0.h>

int exynos_usbdrd_s2mpu_manual_control(bool on);

#endif /* __LINUX_USB_DWC3_EXYNOS_LDO_H */
