// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 Watchdog Timer Support
 *
 * Based on, softdog.c by Alan Cox,
 *     (c) Copyright 1996 Alan Cox <alan@lxorguk.ukuu.org.uk>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/debug-snapshot.h>
#include <linux/preempt.h>
#include <linux/of_address.h>
#include <soc/google/exynos-el3_mon.h>
#include <trace/hooks/sched.h>
#include <asm/stacktrace.h>
#include <linux/minmax.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/rcupdate.h>
#include <linux/pid.h>
#include <linux/sched/task.h>
#include <soc/google/etm2dram.h>

#define S3C2410_WTCON		0x00
#define S3C2410_WTDAT		0x04
#define S3C2410_WTCNT		0x08
#define S3C2410_WTCLRINT	0x0c
#define EXYNOS_WTMINCNT		0x10

#define S3C2410_WTCNT_MAXCNT	0xffff

#define S3C2410_WTCON_RSTEN	BIT(0)
#define S3C2410_WTCON_INTEN	BIT(2)
#define S3C2410_WTCON_ENABLE	BIT(5)

#define S3C2410_WTCON_DIV16	(0 << 3)
#define S3C2410_WTCON_DIV32	(1 << 3)
#define S3C2410_WTCON_DIV64	(2 << 3)
#define S3C2410_WTCON_DIV128	(3 << 3)

#define S3C2410_WTCON_MAXDIV	0x80

#define S3C2410_WTCON_PRESCALE(x)	((x) << 8)
#define S3C2410_WTCON_PRESCALE_MASK	(0xff << 8)
#define S3C2410_WTCON_PRESCALE_MAX	0xff

#define EXYNOS_WTCON_DBGACK_MASK	BIT(16)
#define EXYNOS_WTCON_WINDOWED_WD	BIT(20)

#define S3C2410_WATCHDOG_ATBOOT		(0)
#define S3C2410_WATCHDOG_DEFAULT_TIME	(15)

#define EXYNOS5_RST_STAT_REG_OFFSET		0x0404
#define EXYNOS_RST_STAT_REG_OFFSET		0x3B44
#define EXYNOS5_WDT_DISABLE_REG_OFFSET		0x0408
#define EXYNOS5_WDT_MASK_RESET_REG_OFFSET	0x040c
#define QUIRK_HAS_PMU_CONFIG			BIT(0)
#define QUIRK_HAS_RST_STAT			BIT(1)
#define QUIRK_HAS_WTCLRINT_REG			BIT(2)
#define QUIRK_HAS_DBGACK_BIT			BIT(3)
#define QUIRK_HAS_WTMINCNT_REG			BIT(4)

#define EXYNOS_CLUSTER0_NONCPU_INT_EN		(0x1244)
#define EXYNOS_CLUSTER1_NONCPU_INT_EN		(0x1444)
#define EXYNOS_CLUSTER2_NONCPU_INT_EN		(0x1644)
#define EXYNOS_CLUSTER0_NONCPU_OUT		(0x1220)
#define EXYNOS_CLUSTER1_NONCPU_OUT		(0x1420)

/* These quirks require that we have a PMU register map */
#define QUIRKS_HAVE_PMUREG			(QUIRK_HAS_PMU_CONFIG | \
						 QUIRK_HAS_RST_STAT)
#define MAX_WATCHDOG_CLUSTER_CNT		2
#define LITTLE_CLUSTER				0
#define BIG_CLUSTER				1
#define MULTISTAGE_WDT_RATIO			80
#define WINDOW_MULTIPLIER			2

#define PRINT_CPUS_LIMIT			(30)

static bool nowayout	= WATCHDOG_NOWAYOUT;
static int tmr_margin;
static int tmr_atboot	= S3C2410_WATCHDOG_ATBOOT;
static int soft_noboot;

module_param(tmr_margin,  int, 0);
module_param(tmr_atboot,  int, 0);
module_param(nowayout,   bool, 0);
module_param(soft_noboot, int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. (default="
		__MODULE_STRING(S3C2410_WATCHDOG_DEFAULT_TIME) ")");
MODULE_PARM_DESC(tmr_atboot,
		 "Watchdog is started at boot time if set to 1, default="
			__MODULE_STRING(S3C2410_WATCHDOG_ATBOOT));
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
MODULE_PARM_DESC(soft_noboot, "Watchdog action, set to 1 to ignore reboots, 0 to reboot (default 0)");

struct schedstat {
	u64 rt_load;
	u64 kcpustat_softirq;
	u64 kcpustat_irq;
};

struct s3c2410_wdt {
	struct device		*dev;
	struct clk		*rate_clock;
	struct clk		*gate_clock;
	void __iomem		*reg_base;
	unsigned int		count;
	/* lock to serialize watchdog operation */
	spinlock_t		lock;
	unsigned long		wtcon_save;
	unsigned long		wtdat_save;
	unsigned long		freq;
	struct watchdog_device	wdt_device;
	struct notifier_block	freq_transition;
	const struct s3c2410_wdt_variant *drv_data;
	struct regmap *pmureg;
	phys_addr_t pmu_alive_pa;
	unsigned int cluster;
	int use_multistage_wdt;
	unsigned int disable_reg_val;
	unsigned int mask_reset_reg_val;
	unsigned int noncpu_int_reg_val;
	unsigned int noncpu_out_reg_val;
	int in_suspend;
	int in_panic;
	pid_t dog_owner_pid;
	u64 pet_when;
	struct schedstat __percpu *schedstat;
};

/*
 * struct s3c2410_wdt_variant - Per-variant config data
 *
 * @disable_reg: Offset in pmureg for the register that disables the watchdog
 *  timer when corresponding processor is in reset state.
 * @mask_reset_reg: Offset in pmureg for the register that masks the watchdog reset request.
 * @mask_bit: Bit number for the cluster# watchdog timer in the disable_reg and the mask_reset_reg.
 * @cnt_en_bit: Bit number of the wathcdog counter en/disable in noncpu_out register.
 * @rst_stat_reg: Offset of reset status register in PMU.
 * @rst_stat_bit: Bit number in the rst_stat register indicating a watchdog reset.
 * @noncpu_int_en: Offset in pmureg for the register that en/disable watchdog reset interrupt.
 * @noncpu_out: Offset in pmureg for the register that en/disable watchdog counter.
 * @quirks: A bitfield of quirks.
 * @pmu_reset_func: callback for en/disable watchdog reset interrupt in PMU.
 * @pmu_count_en_func: callback for en/disable watchdog counter in PMU.
 * @auto_disable_func: callback for disable watchdog timer when corresponding processor is in reset
 *  state.
 */

struct s3c2410_wdt_variant {
	int disable_reg;
	int mask_reset_reg;
	int mask_bit;
	int cnt_en_bit;
	int rst_stat_reg;
	int rst_stat_bit;
	int noncpu_int_en;
	int noncpu_out;
	u32 quirks;
	int (*pmu_reset_func)(struct s3c2410_wdt *wdt, bool mask);
	int (*pmu_count_en_func)(struct s3c2410_wdt *wdt, bool mask);
	int (*auto_disable_func)(struct s3c2410_wdt *wdt, bool mask);
};

static struct s3c2410_wdt *s3c_wdt[MAX_WATCHDOG_CLUSTER_CNT];

static int s3c2410wdt_multistage_wdt_stop(void);
static int s3c2410wdt_multistage_wdt_start(void);
static int s3c2410wdt_multistage_set_heartbeat(struct s3c2410_wdt *wdt, int ratio);
static void s3c2410wdt_multistage_wdt_keepalive(void);
static int s3c2410wdt_get_multistage_index(void);

static int s3c2410wdt_mask_wdt_reset(struct s3c2410_wdt *wdt, bool mask);
static int s3c2410wdt_automatic_disable_wdt(struct s3c2410_wdt *wdt, bool mask);
static int s3c2410wdt_noncpu_int_en(struct s3c2410_wdt *wdt, bool mask);
static int s3c2410wdt_noncpu_out(struct s3c2410_wdt *wdt, bool mask);

static const struct s3c2410_wdt_variant drv_data_s3c2410 = {
	.pmu_reset_func = s3c2410wdt_mask_wdt_reset,
	.auto_disable_func = s3c2410wdt_automatic_disable_wdt,
	.quirks = 0
};

#ifdef CONFIG_OF
static const struct s3c2410_wdt_variant drv_data_s3c6410 = {
	.pmu_reset_func = s3c2410wdt_mask_wdt_reset,
	.auto_disable_func = s3c2410wdt_automatic_disable_wdt,
	.quirks = QUIRK_HAS_WTCLRINT_REG,
};

static const struct s3c2410_wdt_variant drv_data_exynos5250  = {
	.disable_reg = EXYNOS5_WDT_DISABLE_REG_OFFSET,
	.mask_reset_reg = EXYNOS5_WDT_MASK_RESET_REG_OFFSET,
	.mask_bit = 20,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 20,
	.pmu_reset_func = s3c2410wdt_mask_wdt_reset,
	.auto_disable_func = s3c2410wdt_automatic_disable_wdt,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG,
};

static const struct s3c2410_wdt_variant drv_data_exynos5420 = {
	.disable_reg = EXYNOS5_WDT_DISABLE_REG_OFFSET,
	.mask_reset_reg = EXYNOS5_WDT_MASK_RESET_REG_OFFSET,
	.mask_bit = 0,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 9,
	.pmu_reset_func = s3c2410wdt_mask_wdt_reset,
	.auto_disable_func = s3c2410wdt_automatic_disable_wdt,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG,
};

static const struct s3c2410_wdt_variant drv_data_exynos7 = {
	.disable_reg = EXYNOS5_WDT_DISABLE_REG_OFFSET,
	.mask_reset_reg = EXYNOS5_WDT_MASK_RESET_REG_OFFSET,
	.mask_bit = 23,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 23,	/* A57 WDTRESET */
	.pmu_reset_func = s3c2410wdt_mask_wdt_reset,
	.auto_disable_func = s3c2410wdt_automatic_disable_wdt,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG,
};

static const struct s3c2410_wdt_variant drv_data_exynos8 = {
	.disable_reg = EXYNOS5_WDT_DISABLE_REG_OFFSET,
	.mask_reset_reg = EXYNOS5_WDT_MASK_RESET_REG_OFFSET,
	.mask_bit = 24,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 24,	/* A53 WDTRESET */
	.pmu_reset_func = s3c2410wdt_mask_wdt_reset,
	.auto_disable_func = s3c2410wdt_automatic_disable_wdt,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG,
};

/* From Exynos9, PMU registers were changed.
 * MASK_RESET register was replaced by CLUSTERx_NONCPU_INT_EN register.
 * DISABLE register was removed and its value was always fixed by 0.
 */
static const struct s3c2410_wdt_variant drv_data_exynos9_v1 = {
	.noncpu_int_en = EXYNOS_CLUSTER0_NONCPU_INT_EN,
	.mask_bit = 2,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 24,	/* CLUSTER0 WDTRESET */
	.pmu_reset_func = s3c2410wdt_noncpu_int_en,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG,
};

static const struct s3c2410_wdt_variant drv_data_exynos9_v2 = {
	.noncpu_int_en = EXYNOS_CLUSTER2_NONCPU_INT_EN,
	.mask_bit = 2,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 23,	/* CLUSTER2 WDTRESET */
	.pmu_reset_func = s3c2410wdt_noncpu_int_en,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG,
};

static const struct s3c2410_wdt_variant drv_data_exynos9_v3 = {
	.noncpu_int_en = EXYNOS_CLUSTER0_NONCPU_INT_EN,
	.mask_bit = 2,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 24,	/* CLUSTER0 WDTRESET */
	.pmu_reset_func = s3c2410wdt_noncpu_int_en,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG |
		  QUIRK_HAS_DBGACK_BIT | QUIRK_HAS_WTMINCNT_REG,
};

static const struct s3c2410_wdt_variant drv_data_exynos9_v4 = {
	.noncpu_int_en = EXYNOS_CLUSTER2_NONCPU_INT_EN,
	.mask_bit = 2,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 23,	/* CLUSTER2 WDTRESET */
	.pmu_reset_func = s3c2410wdt_noncpu_int_en,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG |
		  QUIRK_HAS_DBGACK_BIT | QUIRK_HAS_WTMINCNT_REG,
};

static const struct s3c2410_wdt_variant drv_data_gs101_cl0 = {
	.noncpu_int_en = EXYNOS_CLUSTER0_NONCPU_INT_EN,
	.noncpu_out = EXYNOS_CLUSTER0_NONCPU_OUT,
	.mask_bit = 2,
	.cnt_en_bit = 8,
	.rst_stat_reg = EXYNOS_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 0,      /* CLUSTER0 WDTRESET */
	.pmu_reset_func = s3c2410wdt_noncpu_int_en,
	.pmu_count_en_func = s3c2410wdt_noncpu_out,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG |
		  QUIRK_HAS_DBGACK_BIT | QUIRK_HAS_WTMINCNT_REG,
};

static const struct s3c2410_wdt_variant drv_data_gs101_cl1 = {
	.noncpu_int_en = EXYNOS_CLUSTER1_NONCPU_INT_EN,
	.noncpu_out = EXYNOS_CLUSTER1_NONCPU_OUT,
	.mask_bit = 2,
	.cnt_en_bit = 7,
	.rst_stat_reg = EXYNOS_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 1,      /* CLUSTER1 WDTRESET */
	.pmu_reset_func = s3c2410wdt_noncpu_int_en,
	.pmu_count_en_func = s3c2410wdt_noncpu_out,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG |
		  QUIRK_HAS_DBGACK_BIT | QUIRK_HAS_WTMINCNT_REG,
};

static const struct s3c2410_wdt_variant drv_data_gs201_cl0 = {
	.noncpu_int_en = EXYNOS_CLUSTER0_NONCPU_INT_EN,
	.noncpu_out = EXYNOS_CLUSTER0_NONCPU_OUT,
	.mask_bit = 2,
	.cnt_en_bit = 8,
	.rst_stat_reg = EXYNOS_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 0,      /* CLUSTER0 WDTRESET */
	.pmu_reset_func = s3c2410wdt_noncpu_int_en,
	.pmu_count_en_func = s3c2410wdt_noncpu_out,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG |
		  QUIRK_HAS_DBGACK_BIT | QUIRK_HAS_WTMINCNT_REG,
};

static const struct s3c2410_wdt_variant drv_data_gs201_cl1 = {
	.noncpu_int_en = EXYNOS_CLUSTER1_NONCPU_INT_EN,
	.noncpu_out = EXYNOS_CLUSTER1_NONCPU_OUT,
	.mask_bit = 2,
	.cnt_en_bit = 7,
	.rst_stat_reg = EXYNOS_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 1,      /* CLUSTER1 WDTRESET */
	.pmu_reset_func = s3c2410wdt_noncpu_int_en,
	.pmu_count_en_func = s3c2410wdt_noncpu_out,
	.quirks = QUIRK_HAS_PMU_CONFIG | QUIRK_HAS_RST_STAT | QUIRK_HAS_WTCLRINT_REG |
		  QUIRK_HAS_DBGACK_BIT | QUIRK_HAS_WTMINCNT_REG,
};

static const struct of_device_id s3c2410_wdt_match[] = {
	{ .compatible = "samsung,s3c2410-wdt",
	  .data = &drv_data_s3c2410 },
	{ .compatible = "samsung,s3c6410-wdt",
	  .data = &drv_data_s3c6410 },
	{ .compatible = "samsung,exynos5250-wdt",
	  .data = &drv_data_exynos5250 },
	{ .compatible = "samsung,exynos5420-wdt",
	  .data = &drv_data_exynos5420 },
	{ .compatible = "samsung,exynos7-wdt",
	  .data = &drv_data_exynos7 },
	{ .compatible = "samsung,exynos8-wdt",
	  .data = &drv_data_exynos8 },
	{ .compatible = "samsung,exynos9-v1-wdt",
	  .data = &drv_data_exynos9_v1 },
	{ .compatible = "samsung,exynos9-v2-wdt",
	  .data = &drv_data_exynos9_v2 },
	{ .compatible = "samsung,exynos9-v3-wdt",
	  .data = &drv_data_exynos9_v3 },
	{ .compatible = "samsung,exynos9-v4-wdt",
	  .data = &drv_data_exynos9_v4 },
	{ .compatible = "google,gs101-cl0-wdt",
	  .data = &drv_data_gs101_cl0 },
	{ .compatible = "google,gs101-cl1-wdt",
	  .data = &drv_data_gs101_cl1 },
	{ .compatible = "google,gs201-cl0-wdt",
	  .data = &drv_data_gs201_cl0 },
	{ .compatible = "google,gs201-cl1-wdt",
	  .data = &drv_data_gs201_cl1 },
	{},
};
MODULE_DEVICE_TABLE(of, s3c2410_wdt_match);
#endif

static const struct platform_device_id s3c2410_wdt_ids[] = {
	{
		.name = "s3c2410-wdt",
		.driver_data = (unsigned long)&drv_data_s3c2410,
	},
	{}
};
MODULE_DEVICE_TABLE(platform, s3c2410_wdt_ids);

/* functions */

static inline unsigned int s3c2410wdt_max_timeout(struct clk *clock)
{
	unsigned long freq = clk_get_rate(clock);

	return S3C2410_WTCNT_MAXCNT / (freq / (S3C2410_WTCON_PRESCALE_MAX + 1)
				       / S3C2410_WTCON_MAXDIV);
}

static inline struct s3c2410_wdt *freq_to_wdt(struct notifier_block *nb)
{
	return container_of(nb, struct s3c2410_wdt, freq_transition);
}

static int s3c2410wdt_noncpu_int_en(struct s3c2410_wdt *wdt, bool mask)
{
	int ret;
	u32 mask_val = 1 << wdt->drv_data->mask_bit;
	u32 val = mask_val, reg_val = 0;

	/* No need to do anything if no PMU CONFIG needed */
	if (!(wdt->drv_data->quirks & QUIRK_HAS_PMU_CONFIG))
		return 0;

	/* If mask value is true, wdt interrupt disable */
	if (mask)
		val = 0;

	ret = exynos_pmu_update(wdt->drv_data->noncpu_int_en, mask_val, val);

	if (ret < 0) {
		dev_err(wdt->dev, "failed to update reg(%d)\n", ret);
		return ret;
	}

	ret = exynos_pmu_read(wdt->drv_data->noncpu_int_en, &reg_val);
	if (ret < 0) {
		dev_err(wdt->dev,
			"Couldn't get NONCPU_INT_EN register, ret = (%d)\n", ret);
		return ret;
	}

	wdt->noncpu_int_reg_val = reg_val;

	dev_info(wdt->dev, "NONCPU_INT_EN set %s done, val = %x, mask = %d\n",
		 val ? "true" : "false", reg_val, mask);

	return ret;
}

static int s3c2410wdt_noncpu_out(struct s3c2410_wdt *wdt, bool en)
{
	int ret;
	u32 cnt_en_val = 1 << wdt->drv_data->cnt_en_bit;
	u32 val = cnt_en_val, reg_val = 0;

	/* No need to do anything if no PMU CONFIG needed */
	if (!(wdt->drv_data->quirks & QUIRK_HAS_PMU_CONFIG))
		return 0;

	/* If en value is false, wdt counter disable */
	if (!en)
		val = 0;

	ret = exynos_pmu_update(wdt->drv_data->noncpu_out, cnt_en_val, val);
	if (ret < 0) {
		dev_err(wdt->dev, "failed to update reg(%d)\n", ret);
		return ret;
	}

	ret = exynos_pmu_read(wdt->drv_data->noncpu_out, &reg_val);
	if (ret < 0) {
		dev_err(wdt->dev, "Couldn't get NONCPU_OUT register, ret = (%d)\n", ret);
		return ret;
	}

	wdt->noncpu_out_reg_val = reg_val;

	dev_info(wdt->dev, "NONCPU_OUT set %s done, val = %x, en= %d\n",
		 val ? "true" : "false", reg_val, en);

	return ret;
}

static int s3c2410wdt_mask_wdt_reset(struct s3c2410_wdt *wdt, bool mask)
{
	int ret;
	u32 mask_val = 1 << wdt->drv_data->mask_bit;
	u32 val = 0, mask_reset_reg_val = 0, disable_reg_val = 0;

	/* No need to do anything if no PMU CONFIG needed */
	if (!(wdt->drv_data->quirks & QUIRK_HAS_PMU_CONFIG))
		return 0;

	if (mask)
		val = mask_val;

	ret = rmw_priv_reg(wdt->pmu_alive_pa + wdt->drv_data->mask_reset_reg, mask_val, val);

	if (ret < 0) {
		dev_err(wdt->dev, "failed to update reg(%d)\n", ret);
		return ret;
	}

	ret = regmap_read(wdt->pmureg, wdt->drv_data->mask_reset_reg, &mask_reset_reg_val);
	if (ret < 0) {
		dev_err(wdt->dev, "Couldn't get MASK_WDT_RESET register, ret = (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(wdt->pmureg, wdt->drv_data->disable_reg, &disable_reg_val);
	if (ret < 0) {
		dev_err(wdt->dev, "Couldn't get DISABLE_WDT register, ret = (%d)\n", ret);
		return ret;
	}

	dev_info(wdt->dev, "DISABLE_WDT reg val:  %x, MASK_WDT_RESET reg val: %x\n",
		 disable_reg_val, mask_reset_reg_val);

	wdt->disable_reg_val = disable_reg_val;
	wdt->mask_reset_reg_val = mask_reset_reg_val;

	dev_info(wdt->dev, "Mask_wdt_reset set %s done, mask = %d\n",
		 mask ? "true" : "false", mask);

	return ret;
}

static int s3c2410wdt_automatic_disable_wdt(struct s3c2410_wdt *wdt, bool mask)
{
	int ret;
	u32 mask_val = 1 << wdt->drv_data->mask_bit;
	u32 val = 0, mask_reset_reg_val = 0, disable_reg_val = 0;

	/* No need to do anything if no PMU CONFIG needed */
	if (!(wdt->drv_data->quirks & QUIRK_HAS_PMU_CONFIG))
		return 0;

	if (mask)
		val = mask_val;

	ret = rmw_priv_reg(wdt->pmu_alive_pa + wdt->drv_data->disable_reg, mask_val, val);
	if (ret < 0) {
		dev_err(wdt->dev, "failed to update reg(%d)\n", ret);
		return ret;
	}

	ret = regmap_read(wdt->pmureg, wdt->drv_data->mask_reset_reg, &mask_reset_reg_val);
	if (ret < 0) {
		dev_err(wdt->dev, "Couldn't get MASK_WDT_RESET register, ret = (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(wdt->pmureg, wdt->drv_data->disable_reg, &disable_reg_val);
	if (ret < 0) {
		dev_err(wdt->dev, "Couldn't get DISABLE_WDT register, ret = (%d)\n", ret);
		return ret;
	}

	dev_info(wdt->dev, "DISABLE_WDT reg val:  %x, MASK_WDT_RESET reg val: %x\n",
		 disable_reg_val, mask_reset_reg_val);

	wdt->disable_reg_val = disable_reg_val;
	wdt->mask_reset_reg_val = mask_reset_reg_val;

	dev_info(wdt->dev, "Automatic_wdt set %s done, mask = %d\n", mask ? "true" : "false", mask);

	return ret;
}

static void s3c2410wdt_mask_dbgack(struct s3c2410_wdt *wdt, bool mask)
{
	unsigned long wtcon;

	if (!(wdt->drv_data->quirks & QUIRK_HAS_DBGACK_BIT))
		return;

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	if (mask)
		wtcon |= EXYNOS_WTCON_DBGACK_MASK;
	else
		wtcon &= ~EXYNOS_WTCON_DBGACK_MASK;
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);
}

#define per_schedstat(wdt, cpu) per_cpu_ptr(wdt->schedstat, cpu)

static int s3c2410wdt_keepalive(struct watchdog_device *wdd)
{
	int cpu;
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags, old_wtcnt = 0, wtcnt = 0;

	preempt_disable();

	if (wdt->cluster == LITTLE_CLUSTER)
		s3c2410wdt_multistage_wdt_keepalive();

	old_wtcnt = readl(wdt->reg_base + S3C2410_WTCNT);

	spin_lock_irqsave(&wdt->lock, flags);
	writel(wdt->count, wdt->reg_base + S3C2410_WTCNT);
	spin_unlock_irqrestore(&wdt->lock, flags);

	wtcnt = readl(wdt->reg_base + S3C2410_WTCNT);
	dev_dbg(wdt->dev, "Watchdog cluster %u keepalive!, old_wtcnt = %lx, wtcnt = %lx\n",
		wdt->cluster, old_wtcnt, wtcnt);

	wdt->dog_owner_pid = task_active_pid_ns(current) == &init_pid_ns ? current->pid : 0;
	wdt->pet_when = sched_clock();
	if (wdt->schedstat) {
		for_each_possible_cpu(cpu) {
			struct schedstat *stat = per_schedstat(wdt, cpu);

			stat->rt_load = 0;
			stat->kcpustat_softirq = kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
			stat->kcpustat_irq = kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
		}
	}

	preempt_enable();

	return 0;
}

static struct task_struct *get_task_by_pid_in_init_ns(pid_t nr)
{
	struct task_struct *task;

	rcu_read_lock();
	task = pid_task(find_pid_ns(nr, &init_pid_ns), PIDTYPE_PID);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();

	return task;
}

static void print_wd_owner(struct s3c2410_wdt *wdt, const char *loglvl)
{
	const char *status = "unknown";
	u64 last_nsecs = 0;
	struct task_struct *task = NULL;

	if (wdt->dog_owner_pid)
		task = get_task_by_pid_in_init_ns(wdt->dog_owner_pid);

	if (!task) {
		dev_err(wdt->dev, "last petting info not found: pid = %d\n", wdt->dog_owner_pid);
		return;
	}

	dev_printk(loglvl, wdt->dev, "last petted by comm:%s (pid:%d state:%c cpu:%d)\n",
			task->comm, task->pid, task_state_to_char(task), task_cpu(task));

#ifdef CONFIG_SCHED_INFO
	if (task->state == TASK_RUNNING
			&& task->sched_info.last_queued == 0) {
		status = "running";
		last_nsecs = task->sched_info.last_arrival;
	} else if (task->state == TASK_RUNNING
			&& task->sched_info.last_queued != 0) {
		status = "queued";
		last_nsecs = task->sched_info.last_queued;
	} else if (task->state == TASK_INTERRUPTIBLE
			|| task->state == TASK_UNINTERRUPTIBLE) {
		status = "blocked";
		last_nsecs = task->sched_info.last_arrival;
	}
#endif /* CONFIG_SCHED_INFO */

	dev_printk(loglvl, wdt->dev, "  %s for %llu secs\n", status,
			(sched_clock() - last_nsecs) / NSEC_PER_SEC);

	if (task->state != TASK_RUNNING)
		dump_backtrace(NULL, task, loglvl);
	else
		dev_printk(loglvl, wdt->dev, "  (skip dump backtrace; task is running)\n");

	put_task_struct(task);
}

/* Returns proportion of dividend to divisor in range (0, 10]. */
static int proportion_in_1_digit(u64 dividend, u64 divisor)
{
	return max(min((int) div64_u64(dividend * 10, divisor), 9), 0);
}

void s3c2410wdt_print_schedstat(const char *loglvl)
{
	int cpu;
	char buf[max(NR_CPUS, PRINT_CPUS_LIMIT)];
	char *bufp;
	u64 duration;
	struct s3c2410_wdt *wdt = s3c_wdt[LITTLE_CLUSTER];

	if (s3c_wdt[LITTLE_CLUSTER])
		print_wd_owner(s3c_wdt[LITTLE_CLUSTER], loglvl);
	if (s3c_wdt[BIG_CLUSTER])
		print_wd_owner(s3c_wdt[BIG_CLUSTER], loglvl);

	if (!wdt || !wdt->schedstat) {
		dev_err(wdt->dev, "schedstat is invalid: %s not found",
				wdt ? "little-wdt.schedstat" : "little-wdt");
		return;
	}

	duration = sched_clock() - wdt->pet_when;

	dev_printk(loglvl, wdt->dev, "schedstat since last petting: %llu secs ago",
			duration / NSEC_PER_SEC);

	if (num_possible_cpus() > sizeof(buf)) {
		dev_err(wdt->dev, "not enough buffer to print schedstat\n");
		return;
	}

	bufp = &buf[0];
	for_each_possible_cpu(cpu)
		*bufp++ = cpu_online(cpu) ? '0' + cpu : '_';
	dev_printk(loglvl, wdt->dev, "\t   online cpu nr: %.*s\n", num_possible_cpus(), buf);

	bufp = &buf[0];
	for_each_possible_cpu(cpu) {
		u64 load = per_schedstat(wdt, cpu)->rt_load;
		*bufp++ = '0' + proportion_in_1_digit(load, duration);
	}
	dev_printk(loglvl, wdt->dev, "\trt tasks loading: %.*s\n", num_possible_cpus(), buf);

	bufp = &buf[0];
	for_each_possible_cpu(cpu) {
		u64 last = per_schedstat(wdt, cpu)->kcpustat_irq;
		u64 now = kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
		*bufp++ = '0' + proportion_in_1_digit(now - last, duration);
	}
	dev_printk(loglvl, wdt->dev, "\t     irq loading: %.*s\n", num_possible_cpus(), buf);

	bufp = &buf[0];
	for_each_possible_cpu(cpu) {
		u64 last = per_schedstat(wdt, cpu)->kcpustat_softirq;
		u64 now = kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
		*bufp++ = '0' + proportion_in_1_digit(now - last, duration);
	}
	dev_printk(loglvl, wdt->dev, "\t softirq loading: %.*s\n", num_possible_cpus(), buf);
}
EXPORT_SYMBOL_GPL(s3c2410wdt_print_schedstat);

static void __s3c2410wdt_stop(struct s3c2410_wdt *wdt)
{
	unsigned long wtcon;

	/* If Cluster 0 Watchdog is disabled, Multistage watchdog is also disabled */
	s3c2410wdt_multistage_wdt_stop();

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtcon &= ~(S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN);
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	dev_info(wdt->dev, "Watchdog cluster %u stop done, WTCON = %lx\n", wdt->cluster, wtcon);
}

static int s3c2410wdt_stop(struct watchdog_device *wdd)
{
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	__s3c2410wdt_stop(wdt);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return 0;
}

static int s3c2410wdt_stop_intclear(struct s3c2410_wdt *wdt)
{
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	__s3c2410wdt_stop(wdt);
	writel(1, wdt->reg_base + S3C2410_WTCLRINT);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return 0;
}

static int s3c2410wdt_start(struct watchdog_device *wdd)
{
	unsigned long wtcon, wtmincnt, flags;
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock_irqsave(&wdt->lock, flags);

	__s3c2410wdt_stop(wdt);

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtcon |= S3C2410_WTCON_ENABLE | S3C2410_WTCON_DIV128;

	if (soft_noboot) {
		wtcon |= S3C2410_WTCON_INTEN;
		wtcon &= ~S3C2410_WTCON_RSTEN;
	} else {
		wtcon &= ~S3C2410_WTCON_INTEN;
		wtcon |= S3C2410_WTCON_RSTEN;
	}

	dev_dbg(wdt->dev, "Starting watchdog: count=0x%08x, wtcon=%08lx\n",
		wdt->count, wtcon);

	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG) {
		wtcon |= EXYNOS_WTCON_WINDOWED_WD;
		wtmincnt = wdt->count * WINDOW_MULTIPLIER;
		writel(wtmincnt, wdt->reg_base + EXYNOS_WTMINCNT);
		dev_dbg(wdt->dev, "Starting watchdog: wtmincnt=%08lx\n", wtmincnt);
	}
	writel(wdt->count, wdt->reg_base + S3C2410_WTDAT);
	writel(wdt->count, wdt->reg_base + S3C2410_WTCNT);
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);
	s3c2410wdt_multistage_wdt_start();

	spin_unlock_irqrestore(&wdt->lock, flags);

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	dev_info(wdt->dev, "Watchdog cluster %u start, WTCON = %lx\n", wdt->cluster, wtcon);

	return 0;
}

static inline int s3c2410wdt_is_running(struct s3c2410_wdt *wdt)
{
	return readl(wdt->reg_base + S3C2410_WTCON) & S3C2410_WTCON_ENABLE;
}

static int s3c2410wdt_set_heartbeat(struct watchdog_device *wdd,
				    unsigned int timeout)
{
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long freq = wdt->freq;
	unsigned long count;
	unsigned int divisor = 1;
	unsigned long wtcon, wtmincnt;

	if (timeout < 1)
		return -EINVAL;

	freq = DIV_ROUND_UP(freq, 128);
	count = timeout * freq;

	dev_dbg(wdt->dev, "Heartbeat: count=%lu, timeout=%u, freq=%lu\n",
		count, timeout, freq);

	/*
	 * if the count is bigger than the watchdog register,
	 * then work out what we need to do (and if) we can
	 * actually make this value
	 */

	if (count >= 0x10000) {
		divisor = DIV_ROUND_UP(count, 0xffff);

		if (divisor > 0x100) {
			dev_err(wdt->dev, "timeout %d too big\n", timeout);
			divisor = 0x100;
		}
	}

	dev_dbg(wdt->dev, "Heartbeat: timeout=%u, divisor=%d, count=%lu (%08lx)\n",
		timeout, divisor, count, DIV_ROUND_UP(count, divisor));

	count = DIV_ROUND_UP(count, divisor);
	wdt->count = count;

	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG) {
		wtmincnt = count * WINDOW_MULTIPLIER;
		writel(wtmincnt, wdt->reg_base + EXYNOS_WTMINCNT);
		dev_dbg(wdt->dev, "Heartbeat: wtmincnt=%lx\n", wtmincnt);
	}

	/* update the pre-scaler */
	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtcon &= ~S3C2410_WTCON_PRESCALE_MASK;
	wtcon |= S3C2410_WTCON_PRESCALE(divisor - 1);

	writel(count, wdt->reg_base + S3C2410_WTDAT);
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);

	wdd->timeout = (count * divisor) / freq;
	s3c2410wdt_multistage_set_heartbeat(wdt, MULTISTAGE_WDT_RATIO);

	return 0;
}

static int s3c2410wdt_restart(struct watchdog_device *wdd, unsigned long action,
			      void *data)
{
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	void __iomem *wdt_base = wdt->reg_base;
	unsigned int wtcon;

	/* disable watchdog, to be safe  */
	writel(0, wdt_base + S3C2410_WTCON);

	/* put initial values into count and data */
	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG)
		writel(0x100, wdt_base + EXYNOS_WTMINCNT);
	writel(0x80, wdt_base + S3C2410_WTCNT);
	writel(0x80, wdt_base + S3C2410_WTDAT);

	/* set the watchdog to go and reset... */
	wtcon = S3C2410_WTCON_ENABLE | S3C2410_WTCON_DIV16 |
		S3C2410_WTCON_RSTEN | S3C2410_WTCON_PRESCALE(0x20);
	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG)
		wtcon |= EXYNOS_WTCON_WINDOWED_WD;

	writel(wtcon, wdt_base + S3C2410_WTCON);

	s3c2410wdt_multistage_wdt_start();

	/* wait for reset to assert... */
	mdelay(500);

	return 0;
}

#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE)

static const struct watchdog_info s3c2410_wdt_ident = {
	.options          =     OPTIONS,
	.firmware_version =	0,
	.identity         =	"S3C2410 Watchdog",
};

static const struct watchdog_ops s3c2410wdt_ops = {
	.owner = THIS_MODULE,
	.start = s3c2410wdt_start,
	.stop = s3c2410wdt_stop,
	.ping = s3c2410wdt_keepalive,
	.set_timeout = s3c2410wdt_set_heartbeat,
	.restart = s3c2410wdt_restart,
};

static const struct watchdog_device s3c2410_wdd = {
	.info = &s3c2410_wdt_ident,
	.ops = &s3c2410wdt_ops,
	.timeout = S3C2410_WATCHDOG_DEFAULT_TIME,
};

/* interrupt handler code */

static irqreturn_t s3c2410wdt_irq(int irqno, void *param)
{
	struct s3c2410_wdt *wdt = platform_get_drvdata(param);

	dev_info(wdt->dev, "watchdog timer expired (irq)\n");

	s3c2410wdt_keepalive(&wdt->wdt_device);

	if (wdt->drv_data->quirks & QUIRK_HAS_WTCLRINT_REG)
		writel(0x1, wdt->reg_base + S3C2410_WTCLRINT);

	return IRQ_HANDLED;
}

#ifdef CONFIG_ARM_S3C24XX_CPUFREQ

static int s3c2410wdt_cpufreq_transition(struct notifier_block *nb,
					 unsigned long val, void *data)
{
	int ret;
	struct s3c2410_wdt *wdt = freq_to_wdt(nb);

	if (!s3c2410wdt_is_running(wdt))
		goto done;

	if (val == CPUFREQ_PRECHANGE) {
		/* To ensure that over the change we don't cause the
		 * watchdog to trigger, we perform an keep-alive if
		 * the watchdog is running.
		 */

		s3c2410wdt_keepalive(&wdt->wdt_device);
	} else if (val == CPUFREQ_POSTCHANGE) {
		s3c2410wdt_stop(&wdt->wdt_device);

		ret = s3c2410wdt_set_heartbeat(&wdt->wdt_device, wdt->wdt_device.timeout);

		if (ret >= 0)
			s3c2410wdt_start(&wdt->wdt_device);
		else
			goto err;
	}

done:
	return 0;

err:
	dev_err(wdt->dev, "cannot set new value (%d) for timeout\n", wdt->wdt_device.timeout);
	return ret;
}

static inline int s3c2410wdt_cpufreq_register(struct s3c2410_wdt *wdt)
{
	wdt->freq_transition.notifier_call = s3c2410wdt_cpufreq_transition;

	return cpufreq_register_notifier(&wdt->freq_transition,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void s3c2410wdt_cpufreq_deregister(struct s3c2410_wdt *wdt)
{
	wdt->freq_transition.notifier_call = s3c2410wdt_cpufreq_transition;

	cpufreq_unregister_notifier(&wdt->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

#else

static inline int s3c2410wdt_cpufreq_register(struct s3c2410_wdt *wdt)
{
	return 0;
}

static inline void s3c2410wdt_cpufreq_deregister(struct s3c2410_wdt *wdt)
{
}
#endif

static inline unsigned int s3c2410wdt_get_bootstatus(struct s3c2410_wdt *wdt)
{
	unsigned int rst_stat;
	int ret;

	if (!(wdt->drv_data->quirks & QUIRK_HAS_RST_STAT))
		return 0;

	ret = regmap_read(wdt->pmureg, wdt->drv_data->rst_stat_reg, &rst_stat);
	if (ret)
		dev_warn(wdt->dev, "Couldn't get RST_STAT register\n");
	else if (rst_stat & BIT(wdt->drv_data->rst_stat_bit))
		return WDIOF_CARDRESET;

	return 0;
}

static inline const struct s3c2410_wdt_variant *
s3c2410_get_wdt_drv_data(struct platform_device *pdev)
{
	const struct s3c2410_wdt_variant *variant;

	variant = of_device_get_match_data(&pdev->dev);
	if (!variant) {
		/* Device matched by platform_device_id */
		variant = (struct s3c2410_wdt_variant *)
			   platform_get_device_id(pdev)->driver_data;
	}

	return variant;
}

int s3c2410wdt_set_emergency_stop(int index)
{
	struct s3c2410_wdt *wdt = s3c_wdt[index];

	if (!wdt)
		return -ENODEV;

	/* stop watchdog */
	dev_emerg(wdt->dev, "%s: watchdog is stopped\n", __func__);
	s3c2410wdt_stop(&wdt->wdt_device);
	return 0;
}
EXPORT_SYMBOL(s3c2410wdt_set_emergency_stop);

int s3c2410wdt_keepalive_emergency(bool reset, int index, unsigned int sec)
{
	struct s3c2410_wdt *wdt = s3c_wdt[index];

	if (!wdt)
		return -ENODEV;

	if (reset) {
		dev_emerg(wdt->dev, "watchdog reset is started to %dsecs\n", sec);
		s3c2410wdt_set_heartbeat(&wdt->wdt_device, sec);
		s3c2410wdt_start(&wdt->wdt_device);
		s3c2410wdt_multistage_wdt_stop();
	}
	s3c2410wdt_keepalive(&wdt->wdt_device);
	return 0;
}
EXPORT_SYMBOL(s3c2410wdt_keepalive_emergency);

static int s3c2410wdt_get_multistage_index(void)
{
	int i;

	for (i = 0; i < MAX_WATCHDOG_CLUSTER_CNT; i++) {
		if (s3c_wdt[i] && s3c_wdt[i]->use_multistage_wdt)
			return i;
	}

	return -ENODEV;
}

int s3c2410wdt_keepalive_common(void)
{
	struct s3c2410_wdt *wdt = s3c_wdt[LITTLE_CLUSTER];

	if (!wdt)
		return -ENODEV;

	return s3c2410wdt_keepalive(&wdt->wdt_device);
}
EXPORT_SYMBOL(s3c2410wdt_keepalive_common);

static int s3c2410wdt_multistage_set_heartbeat(struct s3c2410_wdt *wdt, int ratio)
{
	int index;
	unsigned int count;
	unsigned int wtcon, multi_wtcon, wtmincnt;

	index = s3c2410wdt_get_multistage_index();

	if (index < 0)
		return -ENODEV;

	/* set the pre-scaler */
	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtcon &= S3C2410_WTCON_PRESCALE_MASK;

	multi_wtcon = readl(s3c_wdt[index]->reg_base + S3C2410_WTCON);
	multi_wtcon &= ~S3C2410_WTCON_PRESCALE_MASK;
	multi_wtcon |= wtcon;

	count = (wdt->count * ratio) / 100;

	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG) {
		wtmincnt = count * WINDOW_MULTIPLIER;
		writel(wtmincnt, s3c_wdt[index]->reg_base + EXYNOS_WTMINCNT);
	}

	writel(count, s3c_wdt[index]->reg_base + S3C2410_WTDAT);
	writel(multi_wtcon, s3c_wdt[index]->reg_base + S3C2410_WTCON);
	s3c_wdt[index]->count = count;

	return 0;
}

static void s3c2410wdt_multistage_wdt_keepalive(void)
{
	int index;
	unsigned long flags, old_wtcnt = 0, wtcnt = 0;

	index = s3c2410wdt_get_multistage_index();

	if (index < 0)
		return;

	old_wtcnt = readl(s3c_wdt[index]->reg_base + S3C2410_WTCNT);

	spin_lock_irqsave(&s3c_wdt[index]->lock, flags);
	writel(s3c_wdt[index]->count, s3c_wdt[index]->reg_base + S3C2410_WTCNT);
	spin_unlock_irqrestore(&s3c_wdt[index]->lock, flags);

	wtcnt = readl(s3c_wdt[index]->reg_base + S3C2410_WTCNT);
	dev_info(s3c_wdt[index]->dev,
		 "Watchdog cluster %u keepalive!, old_wtcnt = %lx, wtcnt = %lx\n",
		 s3c_wdt[index]->cluster, old_wtcnt, wtcnt);

	etm2dram_delayed_start(s3c_wdt[index]->wdt_device.timeout / 4 * 3);
}

static int s3c2410wdt_multistage_wdt_stop(void)
{
	int index;
	unsigned long wtcon;

	index = s3c2410wdt_get_multistage_index();

	if (index < 0)
		return -ENODEV;

	wtcon = readl(s3c_wdt[index]->reg_base + S3C2410_WTCON);
	wtcon &= ~(S3C2410_WTCON_ENABLE | S3C2410_WTCON_INTEN);
	writel(wtcon, s3c_wdt[index]->reg_base + S3C2410_WTCON);

	dev_info(s3c_wdt[index]->dev, "%s: cluster %u stop done, WTCON %08lx\n",
		 __func__, s3c_wdt[index]->cluster, wtcon);

	etm2dram_cancel_delayed_start();
	return 0;
}

static int s3c2410wdt_multistage_wdt_start(void)
{
	int index;
	unsigned long wtcon;

	index = s3c2410wdt_get_multistage_index();

	if (index < 0)
		return -ENODEV;

	s3c2410wdt_multistage_wdt_stop();

	wtcon = readl(s3c_wdt[index]->reg_base + S3C2410_WTCON);
	wtcon |= S3C2410_WTCON_ENABLE | S3C2410_WTCON_DIV128;

	wtcon |= S3C2410_WTCON_INTEN;
	wtcon &= ~S3C2410_WTCON_RSTEN;

	if (s3c_wdt[index]->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG) {
		wtcon |= EXYNOS_WTCON_WINDOWED_WD;
		writel(s3c_wdt[index]->count * WINDOW_MULTIPLIER,
		       s3c_wdt[index]->reg_base + EXYNOS_WTMINCNT);
	}
	writel(s3c_wdt[index]->count, s3c_wdt[index]->reg_base + S3C2410_WTDAT);
	writel(s3c_wdt[index]->count, s3c_wdt[index]->reg_base + S3C2410_WTCNT);
	writel(wtcon, s3c_wdt[index]->reg_base + S3C2410_WTCON);

	dev_info(s3c_wdt[index]->dev, "%s: count=0x%08x, wtcon=%08lx\n",
		 __func__, s3c_wdt[index]->count, wtcon);

	return 0;
}

static struct wdt_panic_block {
	struct notifier_block nb_panic_block;
	struct s3c2410_wdt *wdt;
} wdt_block;

static int s3c2410wdt_panic_handler(struct notifier_block *nb,
				    unsigned long l, void *buf)
{
	struct wdt_panic_block *wdt_panic = (struct wdt_panic_block *)nb;
	struct s3c2410_wdt *wdt = wdt_panic->wdt;
	int i;
	unsigned int timeout;

	if (!wdt)
		return -ENODEV;

	timeout = wdt->wdt_device.timeout;
	for (i = 0; i < MAX_WATCHDOG_CLUSTER_CNT; i++)
		s3c_wdt[i]->in_panic = 1;

	/* We assumed that num_online_cpus() > 1 status is abnormal */
	if (num_online_cpus() > 1)
		timeout = 10;

	s3c2410wdt_keepalive_emergency(true, 0, timeout);

	return 0;
}

int s3c2410wdt_set_emergency_reset(unsigned int timeout_cnt, int index)
{
	struct s3c2410_wdt *wdt = s3c_wdt[index];
	unsigned int wtdat = 0;
	unsigned int wtcnt = wtdat + timeout_cnt;
	unsigned long wtcon;

	if (!wdt)
		return -ENODEV;

	/* emergency reset with wdt reset */
	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtcon |= S3C2410_WTCON_RSTEN | S3C2410_WTCON_ENABLE;

	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG)
		writel(wtdat * WINDOW_MULTIPLIER, wdt->reg_base + EXYNOS_WTMINCNT);
	writel(wtdat, wdt->reg_base + S3C2410_WTDAT);
	writel(wtcnt, wdt->reg_base + S3C2410_WTCNT);
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);

	return 0;
}
EXPORT_SYMBOL(s3c2410wdt_set_emergency_reset);

inline int s3c2410wdt_multistage_emergency_reset(unsigned int timeout_cnt)
{
	unsigned int wtdat = 0;
	unsigned int wtcnt = wtdat + timeout_cnt;
	unsigned long wtcon;
	int index;

	index = s3c2410wdt_get_multistage_index();

	if (index < 0)
		return -ENODEV;

	/* emergency reset with wdt reset */
	wtcon = readl(s3c_wdt[index]->reg_base + S3C2410_WTCON);
	wtcon |= S3C2410_WTCON_INTEN | S3C2410_WTCON_ENABLE;

	if (s3c_wdt[index]->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG)
		writel(wtdat * WINDOW_MULTIPLIER,
		       s3c_wdt[index]->reg_base + EXYNOS_WTMINCNT);
	writel(wtdat, s3c_wdt[index]->reg_base + S3C2410_WTDAT);
	writel(wtcnt, s3c_wdt[index]->reg_base + S3C2410_WTCNT);
	writel(wtcon, s3c_wdt[index]->reg_base + S3C2410_WTCON);

	return 0;
}

inline void s3c2410wdt_reset_confirm(unsigned long mtime, int index)
{
	struct s3c2410_wdt *wdt = s3c_wdt[index];
	unsigned int wtcon, wtdat, wtcnt, disable_reg, mask_reset_reg;
	unsigned long total_time = 0;
	int ret;

	if (!wdt)
		return;

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);

	dev_info(wdt->dev, "Current %s_cluster watchdog %sable, wtcon = %x\n",
		 index ? "Big" : "Little", (wtcon & S3C2410_WTCON_ENABLE) ? "en" : "dis", wtcon);

	ret = regmap_read(wdt->pmureg, wdt->drv_data->mask_reset_reg, &mask_reset_reg);
	if (ret) {
		dev_err(wdt->dev, "Couldn't get MASK_WDT_RESET register\n");
		return;
	}

	ret = regmap_read(wdt->pmureg, wdt->drv_data->disable_reg, &disable_reg);
	if (ret) {
		dev_err(wdt->dev, "Couldn't get DISABLE_WDT register\n");
		return;
	}

	/*  Fake watchdog bits in both registers must be cleared. */
	dev_info(wdt->dev, "DISABLE_WDT reg:  %x, MASK_WDT_RESET reg: %x\n",
		 disable_reg, mask_reset_reg);

	/* If watchdog is disabled, do not print wtcnt value. */
	if (!(wtcon & S3C2410_WTCON_ENABLE))
		return;

	do {
		/*
		 * It continues to print the wtcnt and wddat values
		 * until watchdog reset is taken.
		 */
		wtdat = readl(wdt->reg_base + S3C2410_WTDAT);
		wtcnt = readl(wdt->reg_base + S3C2410_WTCNT);
		dev_info(wdt->dev, "%lu milliseconds, wtdat = %u, wtcnt = %u",
			 total_time, wtdat, wtcnt);
		total_time += mtime;
		mdelay(mtime);
	} while (1);

	/* This function does not return. */
}
EXPORT_SYMBOL(s3c2410wdt_reset_confirm);

#ifdef CONFIG_PM_SLEEP
static int s3c2410wdt_dev_suspend(struct device *dev)
{
	struct s3c2410_wdt *wdt = dev_get_drvdata(dev);

	if (!wdt)
		return 0;

	/* little cluster wdt should excute syscore suspend */
	if (wdt->cluster == LITTLE_CLUSTER)
		return 0;

	wdt->in_suspend = 1;
	s3c2410wdt_keepalive(&wdt->wdt_device);
	/* Save watchdog state, and turn it off. */
	wdt->wtcon_save = readl(wdt->reg_base + S3C2410_WTCON);
	wdt->wtdat_save = readl(wdt->reg_base + S3C2410_WTDAT);

	return 0;
}

static int s3c2410wdt_dev_resume(struct device *dev)
{
	int ret = 0;
	unsigned int val, wtmincnt;
	struct s3c2410_wdt *wdt = dev_get_drvdata(dev);

	if (!wdt)
		return ret;

	/* little cluster wdt should excute syscore resume */
	if (wdt->cluster == LITTLE_CLUSTER)
		return ret;

	wdt->in_suspend = 0;
	if (wdt->drv_data->auto_disable_func) {
		ret = wdt->drv_data->auto_disable_func(wdt, false);
		if (ret < 0) {
			dev_info(wdt->dev, "automatic_dsiable fail");
			return ret;
		}
	}

	s3c2410wdt_stop_intclear(wdt);

	/* Restore watchdog state. */
	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG) {
		wtmincnt = wdt->wtdat_save * WINDOW_MULTIPLIER;
		writel(wtmincnt, wdt->reg_base + EXYNOS_WTMINCNT);
	}

	writel(wdt->wtdat_save, wdt->reg_base + S3C2410_WTDAT);
	writel(wdt->wtdat_save, wdt->reg_base + S3C2410_WTCNT); /* Reset count */
	writel(wdt->wtcon_save, wdt->reg_base + S3C2410_WTCON);
	if (!wdt->use_multistage_wdt && wdt->drv_data->pmu_reset_func) {
		ret = wdt->drv_data->pmu_reset_func(wdt, false);
		if (ret < 0) {
			dev_info(wdt->dev, "wdt reset mask fail");
			return ret;
		}
	}

	if (wdt->drv_data->pmu_count_en_func) {
		ret = wdt->drv_data->pmu_count_en_func(wdt, true);
		if (ret < 0) {
			dev_info(wdt->dev, "wdt count enable fail\n");
			return ret;
		}
	}

	val = readl(wdt->reg_base + S3C2410_WTCON);
	dev_info(wdt->dev, "watchdog %sabled, con: 0x%08x, dat: 0x%08x, cnt: 0x%08x\n",
		 (val & S3C2410_WTCON_ENABLE) ? "en" : "dis", val,
		 readl(wdt->reg_base + S3C2410_WTDAT),
		 readl(wdt->reg_base + S3C2410_WTCNT));

	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG)
		dev_info(wdt->dev, "windowed_wd %sabled, wtmincnt: 0x%08x\n",
			 (wdt->wtcon_save & EXYNOS_WTCON_WINDOWED_WD) ? "en" : "dis", wtmincnt);

	return ret;
}
#else
#define	s3c2410wdt_dev_suspend	NULL
#define	s3c2410wdt_dev_resume	NULL
#endif
static SIMPLE_DEV_PM_OPS(s3c_wdt_pm_ops, s3c2410wdt_dev_suspend, s3c2410wdt_dev_resume);

#ifdef CONFIG_PM
static int s3c2410wdt_syscore_suspend(void)
{
	struct s3c2410_wdt *wdt = s3c_wdt[LITTLE_CLUSTER];

	if (!wdt)
		return 0;

	wdt->in_suspend = 1;
	s3c2410wdt_keepalive(&wdt->wdt_device);
	/* Save watchdog state, and turn it off. */
	wdt->wtcon_save = readl(wdt->reg_base + S3C2410_WTCON);
	wdt->wtdat_save = readl(wdt->reg_base + S3C2410_WTDAT);

	return 0;
}

static void s3c2410wdt_syscore_resume(void)
{
	int ret;
	unsigned int val;
	struct s3c2410_wdt *wdt = s3c_wdt[LITTLE_CLUSTER];

	if (!wdt)
		return;

	wdt->in_suspend = 0;
	if (wdt->drv_data->auto_disable_func) {
		ret = wdt->drv_data->auto_disable_func(wdt, false);
		if (ret < 0) {
			dev_info(wdt->dev, "automatic_dsiable fail");
			return;
		}
	}

	s3c2410wdt_stop_intclear(wdt);
	/* Restore watchdog state. */
	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG)
		writel(wdt->wtdat_save * WINDOW_MULTIPLIER, wdt->reg_base + EXYNOS_WTMINCNT);
	writel(wdt->wtdat_save, wdt->reg_base + S3C2410_WTDAT);
	writel(wdt->wtdat_save, wdt->reg_base + S3C2410_WTCNT);/* Reset count */
	writel(wdt->wtcon_save, wdt->reg_base + S3C2410_WTCON);

	if (!wdt->use_multistage_wdt && wdt->drv_data->pmu_reset_func) {
		ret = wdt->drv_data->pmu_reset_func(wdt, false);
		if (ret < 0) {
			dev_info(wdt->dev, "pmu reset function fail");
			return;
		}
	}

	if (wdt->drv_data->pmu_count_en_func) {
		ret = wdt->drv_data->pmu_count_en_func(wdt, true);
		if (ret < 0) {
			dev_info(wdt->dev, "wdt count enable fail\n");
			return;
		}
	}

	val = readl(wdt->reg_base + S3C2410_WTCON);
	dev_info(wdt->dev, "watchdog %sabled, con: 0x%08x, dat: 0x%08x, cnt: 0x%08x\n",
		 (val & S3C2410_WTCON_ENABLE) ? "en" : "dis", val,
		 readl(wdt->reg_base + S3C2410_WTDAT),
		 readl(wdt->reg_base + S3C2410_WTCNT));
}

#else
#define s3c2410_wdt_syscore_suspend		NULL
#define s3c2410_wdt_syscore_resume		NULL
#endif

static struct syscore_ops s3c2410wdt_syscore_ops = {
	.suspend	= s3c2410wdt_syscore_suspend,
	.resume		= s3c2410wdt_syscore_resume,
};

static int s3c2410wdt_pm_notifier(struct notifier_block *notifier,
				  unsigned long pm_event, void *v)
{
	struct s3c2410_wdt *wdt = s3c_wdt[LITTLE_CLUSTER];

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		s3c2410wdt_keepalive(&wdt->wdt_device);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block s3c2410wdt_pm_nb = {
	.notifier_call = s3c2410wdt_pm_notifier,
	.priority = 0,
};

static void vh_scheduler_tick(void *data, struct rq *unused)
{
	struct s3c2410_wdt *wdt = data;

	if (!rt_prio(current->prio))
		return;

	this_cpu_ptr(wdt->schedstat)->rt_load += NSEC_PER_SEC / HZ;
}

static int s3c2410wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s3c2410_wdt *wdt;
	struct resource *wdt_irq;
	unsigned int wtcon, disable_reg_val = 0, mask_reset_reg_val = 0;
	unsigned int noncpu_int_reg_val = 0, noncpu_out_reg_val = 0;
	int started = 0;
	int ret;
	unsigned int cluster_index;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->dev = dev;
	spin_lock_init(&wdt->lock);
	wdt->wdt_device = s3c2410_wdd;

	if (of_property_read_u32(dev->of_node, "index", &cluster_index)) {
		dev_err(dev, "Watchdog cluster index lookup failed.\n");
		return -EINVAL;
	}
	dev_info(dev, "watchdog cluster%d probe\n", cluster_index);

	/* s3c_wdt[0] = Little cluster wdt, s3c_wdt[1] = Big cluster wdt */
	if (cluster_index >= MAX_WATCHDOG_CLUSTER_CNT) {
		dev_err(dev, "Watchdog index property too large.\n");
		return -EINVAL;
	}
	s3c_wdt[cluster_index] = wdt;
	wdt->cluster = cluster_index;

	wdt->drv_data = s3c2410_get_wdt_drv_data(pdev);

	if (of_find_property(dev->of_node, "use_multistage_wdt", NULL)) {
		/* If use multistage watchdog, value is 1*/
		wdt->use_multistage_wdt = true;
	} else {
		/* Not use Multistage watchdog */
		wdt->use_multistage_wdt = false;
		dev_info(dev, "It is not a multistage watchdog.\n");
	}

	if (wdt->drv_data->quirks & QUIRKS_HAVE_PMUREG) {
		struct device_node *syscon_np;
		struct resource res;

		wdt->pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
							      "samsung,syscon-phandle");
		if (IS_ERR(wdt->pmureg)) {
			dev_err(dev, "syscon regmap lookup failed.\n");
			return PTR_ERR(wdt->pmureg);
		}

		syscon_np = of_parse_phandle(dev->of_node, "samsung,syscon-phandle", 0);
		if (!syscon_np) {
			dev_err(dev, "syscon device node not found\n");
			return -EINVAL;
		}

		if (of_address_to_resource(syscon_np, 0, &res)) {
			dev_err(dev, "failed to get syscon base address\n");
			return -ENOMEM;
		}

		wdt->pmu_alive_pa = res.start;
	}
	if (wdt->drv_data->mask_reset_reg) {
		ret = regmap_read(wdt->pmureg, wdt->drv_data->mask_reset_reg, &mask_reset_reg_val);
		if (ret) {
			dev_err(wdt->dev, "Couldn't get MASK_WDT_RESET register\n");
			return PTR_ERR(wdt->pmureg);
		}

		dev_info(wdt->dev, "MASK_WDT_RESET reg val: %x\n", mask_reset_reg_val);
	}

	if (wdt->drv_data->disable_reg) {
		ret = regmap_read(wdt->pmureg, wdt->drv_data->disable_reg, &disable_reg_val);
		if (ret) {
			dev_err(wdt->dev, "Couldn't get DISABLE_WDT register\n");
			return PTR_ERR(wdt->pmureg);
		}

		dev_info(wdt->dev, "DISABLE_WDT reg val: %x\n", disable_reg_val);
	}

	if (wdt->drv_data->noncpu_int_en) {
		ret = regmap_read(wdt->pmureg, wdt->drv_data->noncpu_int_en, &noncpu_int_reg_val);
		if (ret) {
			dev_err(wdt->dev, "Couldn't get NONCPU_INT_EN register\n");
			return PTR_ERR(wdt->pmureg);
		}

		dev_info(wdt->dev, "NONCPU_INT_EN reg val: %x\n", noncpu_int_reg_val);
	}

	wdt->disable_reg_val = disable_reg_val;
	wdt->mask_reset_reg_val = mask_reset_reg_val;
	wdt->noncpu_int_reg_val = noncpu_int_reg_val;
	wdt->noncpu_out_reg_val = noncpu_out_reg_val;

	wdt_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!wdt_irq) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err;
	}

	/* get the memory region for the watchdog timer */
	wdt->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->reg_base)) {
		ret = PTR_ERR(wdt->reg_base);
		goto err;
	}

	dev_info(dev, "probe: mapped reg_base=%p\n", wdt->reg_base);

	wdt->rate_clock = devm_clk_get(dev, "rate_watchdog");
	if (IS_ERR(wdt->rate_clock)) {
		dev_err(dev, "failed to find watchdog rate clock source\n");
		ret = PTR_ERR(wdt->rate_clock);
		goto err;
	}
	wdt->freq = clk_get_rate(wdt->rate_clock);

	wdt->gate_clock = devm_clk_get(dev, "gate_watchdog");
	if (IS_ERR(wdt->gate_clock)) {
		dev_err(dev, "failed to find watchdog gate clock source\n");
		ret = PTR_ERR(wdt->gate_clock);
		goto err;
	}

	ret = clk_prepare_enable(wdt->gate_clock);
	if (ret < 0) {
		dev_err(dev, "failed to enable gate clock\n");
		return ret;
	}

	wdt->wdt_device.min_timeout = 1;
	wdt->wdt_device.max_timeout = s3c2410wdt_max_timeout(wdt->rate_clock);

	ret = s3c2410wdt_cpufreq_register(wdt);
	if (ret < 0) {
		dev_err(dev, "failed to register cpufreq\n");
		goto err_clk;
	}

	watchdog_set_drvdata(&wdt->wdt_device, wdt);

	/*
	 * see if we can actually set the requested timer margin,
	 * and if not, try the default value
	 */

	watchdog_init_timeout(&wdt->wdt_device, tmr_margin, dev);
	ret = s3c2410wdt_set_heartbeat(&wdt->wdt_device, wdt->wdt_device.timeout);
	if (ret) {
		started = s3c2410wdt_set_heartbeat(&wdt->wdt_device, S3C2410_WATCHDOG_DEFAULT_TIME);

		if (started == 0)
			dev_info(dev,
				 "tmr_margin value out of range, default %d used\n",
				 S3C2410_WATCHDOG_DEFAULT_TIME);
		else
			dev_info(dev, "default timer value is out of range, cannot start\n");
	}

	ret = devm_request_irq(dev, wdt_irq->start, s3c2410wdt_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_cpufreq;
	}

	watchdog_set_nowayout(&wdt->wdt_device, nowayout);
	watchdog_set_restart_priority(&wdt->wdt_device, 64);

	wdt->wdt_device.bootstatus = s3c2410wdt_get_bootstatus(wdt);
	wdt->wdt_device.parent = dev;

	if (cluster_index == LITTLE_CLUSTER) {
		ret = watchdog_register_device(&wdt->wdt_device);
		if (ret) {
			dev_err(dev, "cannot register watchdog (%d)\n", ret);
			goto err_cpufreq;
		}
	}

	if (wdt->drv_data->auto_disable_func) {
		ret = wdt->drv_data->auto_disable_func(wdt, false);
		if (ret < 0) {
			dev_info(wdt->dev, "automatic_disable fail");
			return 0;
		}
	}

	/* Prevent watchdog reset while setting */
	s3c2410wdt_stop_intclear(wdt);

	if (wdt->drv_data->pmu_count_en_func) {
		ret = wdt->drv_data->pmu_count_en_func(wdt, true);
		if (ret < 0)
			goto err_unregister;
	}

	if (!wdt->use_multistage_wdt && wdt->drv_data->pmu_reset_func) {
		ret = wdt->drv_data->pmu_reset_func(wdt, false);
		if (ret < 0)
			goto err_unregister;
	}

	if (wdt->cluster == LITTLE_CLUSTER) {
		wdt->schedstat = alloc_percpu(struct schedstat);
		if (!wdt->schedstat) {
			dev_err(dev, "cannot alloc percpu schedstat\n");
			goto err_unregister;
		}
		WARN_ON(register_trace_android_vh_scheduler_tick(vh_scheduler_tick, wdt));
	}

	s3c2410wdt_mask_dbgack(wdt, true);

	if (tmr_atboot && started == 0) {
		dev_info(dev, "starting watchdog timer\n");
		s3c2410wdt_start(&wdt->wdt_device);
	} else if (!tmr_atboot) {
		/*
		 * if we're not enabling the watchdog, then ensure it is
		 * disabled if it has been left running from the bootloader
		 * or other source
		 */

		s3c2410wdt_stop(&wdt->wdt_device);
	}

	platform_set_drvdata(pdev, wdt);

	/* print out a statement of readiness */

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	if (cluster_index == LITTLE_CLUSTER) {
		register_syscore_ops(&s3c2410wdt_syscore_ops);

		/* register panic handler for watchdog reset */
		wdt_block.nb_panic_block.notifier_call = s3c2410wdt_panic_handler;
		wdt_block.nb_panic_block.priority = 256;
		wdt_block.wdt = wdt;
		atomic_notifier_chain_register(&panic_notifier_list, &wdt_block.nb_panic_block);
		dbg_snapshot_register_wdt_ops(
				(void *)s3c2410wdt_keepalive_emergency,
				(void *)s3c2410wdt_set_emergency_reset,
				(void *)s3c2410wdt_set_emergency_stop);
		register_pm_notifier(&s3c2410wdt_pm_nb);
	}
	dev_info(dev, "watchdog cluster %d, %sactive, reset %sabled, irq %sabled\n", cluster_index,
		 (wtcon & S3C2410_WTCON_ENABLE) ?  "" : "in",
		 (wtcon & S3C2410_WTCON_RSTEN) ? "en" : "dis",
		 (wtcon & S3C2410_WTCON_INTEN) ? "en" : "dis");

	if (wdt->drv_data->quirks & QUIRK_HAS_DBGACK_BIT)
		dev_info(dev, "DBGACK %sabled\n",
			 (wtcon & EXYNOS_WTCON_DBGACK_MASK) ? "en" : "dis");

	if (wdt->drv_data->quirks & QUIRK_HAS_WTMINCNT_REG)
		dev_info(dev, "windowed watchdog %sabled, wtmincnt=%x\n",
			 (wtcon & EXYNOS_WTCON_WINDOWED_WD) ? "en" : "dis",
			 readl(wdt->reg_base + EXYNOS_WTMINCNT));

	pr_info("Multistage watchdog %sabled",
		wdt->use_multistage_wdt ? "en" : "dis");

	return 0;

 err_unregister:
	watchdog_unregister_device(&wdt->wdt_device);

 err_cpufreq:
	s3c2410wdt_cpufreq_deregister(wdt);

 err_clk:
	clk_disable_unprepare(wdt->gate_clock);
	wdt->rate_clock = NULL;
	wdt->gate_clock = NULL;
 err:
	return ret;
}

static int s3c2410wdt_remove(struct platform_device *dev)
{
	int ret = 0;
	struct s3c2410_wdt *wdt = platform_get_drvdata(dev);

	if (wdt->drv_data->pmu_reset_func)
		ret = wdt->drv_data->pmu_reset_func(wdt, true);
	if (ret < 0)
		return ret;

	watchdog_unregister_device(&wdt->wdt_device);

	s3c2410wdt_cpufreq_deregister(wdt);

	clk_disable_unprepare(wdt->rate_clock);
	wdt->rate_clock = NULL;
	wdt->gate_clock = NULL;

	unregister_pm_notifier(&s3c2410wdt_pm_nb);

	if (wdt->schedstat) {
		WARN_ON(unregister_trace_android_vh_scheduler_tick(vh_scheduler_tick, wdt));
		free_percpu(wdt->schedstat);
	}

	return ret;
}

static void s3c2410wdt_shutdown(struct platform_device *dev)
{
	struct s3c2410_wdt *wdt = platform_get_drvdata(dev);

#ifdef CONFIG_S3C2410_SHUTDOWN_REBOOT
	if (wdt->cluster == 0) {
		/* 0 and multistage are reconfigured altogether by this */
		dev_emerg(wdt->dev, "watchdog is still alive\n");
		s3c2410wdt_keepalive_emergency(true, 0, wdt->wdt_device.timeout);
	} else if (wdt->cluster != 0 && wdt->cluster != s3c2410wdt_get_multistage_index()) {
		s3c2410wdt_stop(&wdt->wdt_device);
	}
#else
	/* Only little cluster watchdog excute mask function */
	if (wdt->cluster == LITTLE_CLUSTER && wdt->drv_data->pmu_reset_func)
		wdt->drv_data->pmu_reset_func(wdt, true);

	s3c2410wdt_stop(&wdt->wdt_device);
#endif
}

static struct platform_driver s3c2410wdt_driver = {
	.probe		= s3c2410wdt_probe,
	.remove		= s3c2410wdt_remove,
	.shutdown	= s3c2410wdt_shutdown,
	.id_table	= s3c2410_wdt_ids,
	.driver		= {
		.name	= "s3c2410-wdt",
		.pm	= &s3c_wdt_pm_ops,
		.of_match_table	= of_match_ptr(s3c2410_wdt_match),
	},
};

module_platform_driver(s3c2410wdt_driver);

MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>, Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("S3C2410 Watchdog Device Driver");
MODULE_LICENSE("GPL");
