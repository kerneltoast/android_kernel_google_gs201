/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung EXYNOS SoC series USB PHY driver
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef _PHY_EXYNOS_EUSB_H_
#define _PHY_EXYNOS_EUSB_H_

extern void phy_exynos_eusb_reset(struct exynos_usbphy_info *info);
extern void phy_exynos_eusb_initiate(struct exynos_usbphy_info *info);
extern u8 phy_exynos_eusb_get_eusb_state(struct exynos_usbphy_info *info);
extern void phy_exynos_eusb_terminate(struct exynos_usbphy_info *info);

#endif
