/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Google LLC.
 */

#ifndef __EXYNOS_PD_HSI0_H
#define __EXYNOS_PD_HSI0_H

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct exynos_pd_hsi0_data {
	struct device	*dev;
	struct regulator *vdd_hsi;
	struct regulator *vdd_high;    //voltage is larger than 3v
	struct regulator *vdd_medium;  //voltage is range from 1v to 2v
	struct regulator *vdd_low;     //voltage is lower than 1v
};

#if IS_ENABLED(CONFIG_EXYNOS_PD_HSI0)
int exynos_pd_hsi0_ldo_manual_control(bool on);
bool exynos_pd_hsi0_get_ldo_status(void);
#if IS_ENABLED(CONFIG_PHY_EXYNOS_EUSB_REPEATER)
extern void eusb_repeater_update_usb_state(bool on);
#else
static inline void eusb_repeater_update_usb_state(bool on)
{
}
#endif
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
int exynos_pd_hsi0_vdd_hsi_manual_control(bool on);
#else
static inline int exynos_pd_hsi0_vdd_hsi_manual_control(bool on)
{
	return 0;
}
#endif
#else
static inline int exynos_pd_hsi0_ldo_manual_control(bool on)
{
	return 0;
}
static inline bool exynos_pd_hsi0_get_ldo_status(void)
{
	return true;
}
static inline int exynos_pd_hsi0_vdd_hsi_manual_control(bool on)
{
	return 0;
}
#endif
#endif  /* __EXYNOS_PD_HSI0_H */
