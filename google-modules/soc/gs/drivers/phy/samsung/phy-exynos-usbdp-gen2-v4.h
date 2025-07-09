/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung EXYNOS SoC series USB DRD PHY driver
 *
 * Author: Sung-Hyun Na <sunghyun.na@samsung.com>
 *
 * Chip Abstraction Layer for USB PHY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PHY_EXYNOS_USBDP_GEN2_V4_H__
#define __PHY_EXYNOS_USBDP_GEN2_V4_H__

int phy_exynos_usbdp_g2_v4_enable(struct exynos_usbphy_info *info);
void phy_exynos_usbdp_g2_v4_disable(struct exynos_usbphy_info *info);
void phy_exynos_usbdp_g2_v4_tune(struct exynos_usbphy_info *info);
void phy_exynos_usbdp_g2_v4_eom(struct exynos_usbphy_info *info,
					struct usb_eom_result_s *eom_result, u32 cmn_rate);
int phy_exynos_usbdp_g2_v4_internal_loopback(struct exynos_usbphy_info *info, u32 cmn_rate);

#endif /* __PHY_EXYNOS_USBDP_GEN2_V4_H__ */
