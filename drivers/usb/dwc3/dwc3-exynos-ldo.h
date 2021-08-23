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

int exynos_usbdrd_ldo_manual_control(bool on);
int exynos_usbdrd_vdd_hsi_manual_control(bool on);
bool exynos_usbdrd_get_ldo_status(void);
bool exynos_usbdrd_get_vdd_hsi_status(void);

#endif /* __LINUX_USB_DWC3_EXYNOS_LDO_H */
