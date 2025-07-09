/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_PD_H
#define __EXYNOS_PD_H __FILE__

#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/atomic.h>

#include <linux/mfd/samsung/core.h>
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG)
#include <soc/google/exynos-bcm_dbg.h>
#endif

#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pd_hsi0.h>
#include <dt-bindings/power/exynos-power.h>

#if IS_ENABLED(CONFIG_SOC_ZUMA)
#define HSI0_CAL_PDID	0xB1380018
#else
#define HSI0_CAL_PDID	0xB1380008
#endif

struct exynos_pm_domain;

struct exynos_pd_stat {
	u64 on_count;
	ktime_t total_on_time;
	ktime_t last_on_time;
	ktime_t last_off_time;
};

struct exynos_pm_domain {
	struct generic_pm_domain genpd;
	char *name;
	unsigned int cal_pdid;
	struct device_node *of_node;
	int (*pd_control)(unsigned int cal_id, int on);
	int (*check_status)(struct exynos_pm_domain *pd);
	bool (*power_down_ok)(void);
	unsigned int bts;
	int devfreq_index;
	struct mutex access_lock;
	int idle_ip_index;
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG)
	struct exynos_bcm_pd_info *bcm;
#endif
	bool power_down_skipped;
	/* Total number of descendants needing sync, including self */
	atomic_t need_sync;
	bool turn_off_on_sync;
	unsigned int need_smc;
	bool skip_idle_ip;
	bool always_on;
	struct exynos_pd_stat pd_stat;
	struct exynos_pm_domain *parent;
};

#if IS_ENABLED(CONFIG_EXYNOS_PD)
struct exynos_pm_domain *exynos_pd_lookup_name(const char *domain_name);
int exynos_pd_status(struct exynos_pm_domain *pd);
int exynos_pd_power_on(struct exynos_pm_domain *pd);
int exynos_pd_power_off(struct exynos_pm_domain *pd);
int exynos_pd_get_pd_stat(struct exynos_pm_domain *pd,
			  struct exynos_pd_stat *s);
#else
static inline
struct exynos_pm_domain *exynos_pd_lookup_name(const char *domain_name)
{
	return NULL;
}

static inline int exynos_pd_status(struct exynos_pm_domain *pd)
{
	return 1;
}

static inline int exynos_pd_power_on(struct exynos_pm_domain *pd)
{
	return 0;
}

static inline int exynos_pd_power_off(struct exynos_pm_domain *pd)
{
	return 0;
}

static inline int exynos_pd_get_pd_stat(struct exynos_pm_domain *pd,
					struct exynos_pd_stat *s)
{
	return 1;
}
#endif

#if IS_ENABLED(CONFIG_USB_DWC3_EXYNOS_GS)
extern u32 dwc3_otg_is_connect(void);
extern void exynos_usbdrd_s2mpu_manual_control(bool on);
extern int exynos_usbdrd_set_s2mpu_pm_ops(int (*cb)(struct device *dev, bool on));
#else
static inline u32 dwc3_otg_is_connect(void)
{
	return 0;
}
static inline void exynos_usbdrd_s2mpu_manual_control(bool on)
{
	return;
}
static inline int exynos_usbdrd_set_s2mpu_pm_ops(int (*cb)(struct device *dev, bool on))
{
	return 0;
}
#endif

#endif /* __EXYNOS_PD_H */
