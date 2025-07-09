/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_PM_H
#define __EXYNOS_PM_H

#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <soc/google/cal-if.h>

#define EXYNOS_PM_PREFIX	"EXYNOS-PM:"

#if IS_ENABLED(CONFIG_ACPM_FLEXPMU_DBG)
u32 acpm_get_mifdn_count(void);
u32 acpm_get_apsocdn_count(void);
u32 acpm_get_early_wakeup_count(void);
int acpm_get_mif_request(void);
#else
static inline int acpm_get_mif_request(void) { return 0; }
static inline u32 acpm_get_mifdn_count(void) { return 0; }
static inline u32 acpm_get_apsocdn_count(void) { return 0; }
static inline u32 acpm_get_early_wakeup_count(void) { return 0; }
#endif

#define EINTMASK_ARR_SIZE (3)
#define BITMAP_SIZE (EINTMASK_ARR_SIZE * 32)

#if IS_ENABLED(CONFIG_PINCTRL_EXYNOS_GS)
extern u32 exynos_eint_wake_mask_array[EINTMASK_ARR_SIZE];
#else
u32 exynos_eint_wake_mask_array[EINTMASK_ARR_SIZE] = {~0U, ~0U, ~0U};
#endif

int register_pcie_is_connect(u32 (*func)(void));

struct wakeup_stat_name {
	const char *name[32];
};

struct exynos_pm_info {
	void __iomem *eint_base;	/* GPIO_ALIVE base to check wkup reason */
	void __iomem *eint_far_base;
	void __iomem *gic_base;		/* GICD_ISPENDRn base to check wkup reason */
	void __iomem *mbox_aoc;
	void __iomem *mbox_aocf1;
	unsigned int num_gic;		/* Total number of GIC sources */

	bool is_early_wakeup;
	unsigned int suspend_mode_idx;	/* power mode to be used in suspend scenario */
	unsigned int apdn_cnt_prev;	/* sleep apsoc down sequence prev count */
	unsigned int apdn_cnt;		/* sleep apsoc down sequence count */

	unsigned int num_wakeup_stat;	/* Total number of wakeup_stat */
	unsigned int *wakeup_stat_offset;

	unsigned int num_wakeup_int_en;
	unsigned int wakeup_stat_eint;
	unsigned int wakeup_stat_rtc;
	unsigned int *wakeup_int_en_offset;
	unsigned int *wakeup_int_en;

	unsigned int num_eint_wakeup_mask;
	unsigned int *eint_wakeup_mask_offset;

	void __iomem *vgpio2pmu_base;	/* SYSREG_VGPIO2PMU base */
	unsigned int vgpio_inten_offset;
	unsigned int vgpio_wakeup_inten;

	struct wakeup_source *ws;
	bool is_stay_awake;

	struct wakeup_stat_name *ws_names;

	bool is_pcieon_suspend;
	unsigned int pcieon_suspend_available;
	unsigned int pcieon_suspend_mode_idx;
	u32 (*pcie_is_connect)(void);
};

struct exynos_pm_dbg {
	u32 test_early_wakeup;
	u32 test_pcieon_suspend;

	unsigned int mifdn_early_wakeup_prev;
	unsigned int mifdn_early_wakeup_cnt;
	unsigned int mifdn_cnt_prev;
	unsigned int mifdn_cnt;
	unsigned int mif_req;
};

#endif /* __EXYNOS_PM_H */
