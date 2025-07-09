/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung EXYNOS SoC series USB PHY driver
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef _PHY_EXYNOS_SNPS_USBDP_H_
#define _PHY_EXYNOS_SNPS_USBDP_H_

extern void phy_exynos_snps_usbdp_phy_initiate(struct exynos_usbphy_info *info);
extern int phy_exynos_snps_usbdp_phy_enable(struct exynos_usbphy_info *info);
extern void phy_exynos_snps_usbdp_phy_disable(struct exynos_usbphy_info *info);
extern void phy_exynos_snps_usbdp_tune_each(struct exynos_usbphy_info *info, char *name, int val);
extern void phy_exynos_snps_usbdp_tune_each_cr(struct exynos_usbphy_info *info, char *name, int val);
extern u16 phy_exynos_snps_usbdp_cr_read(struct exynos_usbphy_info *info, u16 addr);
extern void phy_exynos_snps_usbdp_cr_write(struct exynos_usbphy_info *info, u16 addr, u16 data);
extern void phy_exynos_snps_usbdp_tune(struct exynos_usbphy_info *info);

extern void phy_exynos_snps_usbdp_tca_set(struct exynos_usbphy_info *info, int mux, int low_power_en);
#endif
