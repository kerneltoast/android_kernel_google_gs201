// SPDX-License-Identifier: GPL-2.0
/*
 * gs101_bcl.c gsoc101 bcl driver
 *
 * Copyright (c) 2020, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif
#include "../thermal_core.h"

/* This driver determines if HW was throttled due to SMPL/OCP */

#define CPUCL0_BASE (0x20c00000)
#define CPUCL1_BASE (0x20c10000)
#define CPUCL2_BASE (0x20c20000)
#define SYSREG_CPUCL0_BASE (0x20c40000)
#define CLUSTER0_GENERAL_CTRL_64 (0x1404)
#define CLKDIVSTEP (0x830)
#define CPUCL0_CLKDIVSTEP_STAT (0x83c)
#define CPUCL12_CLKDIVSTEP_STAT (0x848)
#define CLUSTER0_MPMM (0x1408)
#define CLUSTER0_PPM (0x140c)
#define MPMMEN_MASK (0xF << 20)
#define PPMEN_MASK (0x3 << 8)

enum sys_throttling_core { SYS_THROTTLING_MID_CORE, SYS_THROTTLING_BIG_CORE };

enum sys_throttling_switch { SYS_THROTTLING_DISABLED, SYS_THROTTLING_ENABLED };

enum sys_throttling_mode { SYS_THROTTLING_MPMM_MODE, SYS_THROTTLING_PPM_MODE };

struct gs101_bcl_dev {
	struct device *device;
	struct dentry *debug_entry;
	void __iomem *cpu0_mem;
	void __iomem *cpu1_mem;
	void __iomem *cpu2_mem;
	void __iomem *sysreg_cpucl0;
};

DEFINE_MUTEX(sysreg_lock);

static struct gs101_bcl_dev *gs101_bcl_device;

static int get_cpucl0_stat(void *data, u64 *val)
{
	unsigned int reg = 0;
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;

	reg = __raw_readl(bcl_dev->cpu0_mem + CPUCL0_CLKDIVSTEP_STAT);
	*val = (reg >> 16) & 0x0FFF;
	return 0;
}

static int reset_cpucl0_stat(void *data, u64 val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;

	if (val == 0)
		__raw_writel(0x1, bcl_dev->cpu0_mem + CLKDIVSTEP);
	else
		__raw_writel(0x107f, bcl_dev->cpu0_mem + CLKDIVSTEP);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpucl0_clkdivstep_stat_fops, get_cpucl0_stat,
			reset_cpucl0_stat, "%d\n");

static int get_cpucl2_stat(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;
	unsigned int reg = 0;

	reg = __raw_readl(bcl_dev->cpu2_mem + CPUCL12_CLKDIVSTEP_STAT);
	*val = (reg >> 16) & 0x0FFF;
	return 0;
}

static int reset_cpucl2_stat(void *data, u64 val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;

	if (val == 0)
		__raw_writel(0x1, bcl_dev->cpu2_mem + CLKDIVSTEP);
	else
		__raw_writel(0x107f, bcl_dev->cpu2_mem + CLKDIVSTEP);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpucl2_clkdivstep_stat_fops, get_cpucl2_stat,
			reset_cpucl2_stat, "%d\n");

static int get_cpucl1_stat(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;
	unsigned int reg = 0;

	reg = __raw_readl(bcl_dev->cpu1_mem + CPUCL12_CLKDIVSTEP_STAT);
	*val = (reg >> 16) & 0x0FFF;
	return 0;
}

static int reset_cpucl1_stat(void *data, u64 val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;

	if (val == 0)
		__raw_writel(0x1, bcl_dev->cpu1_mem + CLKDIVSTEP);
	else
		__raw_writel(0x107f, bcl_dev->cpu1_mem + CLKDIVSTEP);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpucl1_clkdivstep_stat_fops, get_cpucl1_stat,
			reset_cpucl1_stat, "%d\n");

static void gs101_set_sys_throttling(enum sys_throttling_core core,
				     enum sys_throttling_switch throttle_switch,
				     enum sys_throttling_mode mode)
{
	unsigned int reg, mask;
	void __iomem *addr;

	if (!gs101_bcl_device->sysreg_cpucl0) {
		pr_err("sysreg_cpucl0 ioremap not mapped\n");
		return;
	}
	mutex_lock(&sysreg_lock);
	addr = gs101_bcl_device->sysreg_cpucl0 +
	       ((mode == SYS_THROTTLING_PPM_MODE) ? CLUSTER0_PPM :
							  CLUSTER0_MPMM);
	reg = __raw_readl(addr);
	mask = (core == SYS_THROTTLING_BIG_CORE) ? (0x0F << 4) : 0x0F;
	if (throttle_switch == SYS_THROTTLING_ENABLED)
		reg |= mask;
	else
		reg &= ~mask;
	__raw_writel(reg, addr);
	mutex_unlock(&sysreg_lock);
}

static int gs101_enable_ppm_throttling(void *data, u64 val)
{
	unsigned int mode;

	pr_info("gs101: enable PPM throttling");

	mode = (val == 0) ? SYS_THROTTLING_DISABLED : SYS_THROTTLING_ENABLED;
	gs101_set_sys_throttling(SYS_THROTTLING_MID_CORE, mode,
				 SYS_THROTTLING_PPM_MODE);
	gs101_set_sys_throttling(SYS_THROTTLING_BIG_CORE, mode,
				 SYS_THROTTLING_PPM_MODE);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ppm_fops, NULL, gs101_enable_ppm_throttling, "%d\n");

static int gs101_enable_mpmm_throttling(void *data, u64 val)
{
	unsigned int mode;

	pr_info("gs101: enable MPMM throttling");

	mode = (val == 0) ? SYS_THROTTLING_DISABLED : SYS_THROTTLING_ENABLED;
	gs101_set_sys_throttling(SYS_THROTTLING_MID_CORE, mode,
				 SYS_THROTTLING_MPMM_MODE);
	gs101_set_sys_throttling(SYS_THROTTLING_BIG_CORE, mode,
				 SYS_THROTTLING_MPMM_MODE);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mpmm_fops, NULL, gs101_enable_mpmm_throttling, "%d\n");

static int google_gs101_bcl_probe(struct platform_device *pdev)
{
	unsigned int reg;

	gs101_bcl_device =
		devm_kzalloc(&pdev->dev, sizeof(*gs101_bcl_device), GFP_KERNEL);
	if (!gs101_bcl_device)
		return -ENOMEM;
	gs101_bcl_device->device = &pdev->dev;

	platform_set_drvdata(pdev, gs101_bcl_device);
	gs101_bcl_device->debug_entry = debugfs_create_dir("gs101-bcl", 0);
	if (IS_ERR_OR_NULL(gs101_bcl_device->debug_entry)) {
		gs101_bcl_device->debug_entry = NULL;
		return -EINVAL;
	}
	gs101_bcl_device->cpu0_mem = ioremap(CPUCL0_BASE, SZ_8K);
	if (!gs101_bcl_device->cpu0_mem) {
		pr_err("cpu0_mem ioremap failed\n");
		return -EIO;
	}
	gs101_bcl_device->cpu1_mem = ioremap(CPUCL1_BASE, SZ_8K);
	if (!gs101_bcl_device->cpu1_mem) {
		pr_err("cpu1_mem ioremap failed\n");
		return -EIO;
	}
	gs101_bcl_device->cpu2_mem = ioremap(CPUCL2_BASE, SZ_8K);
	if (!gs101_bcl_device->cpu2_mem) {
		pr_err("cpu2_mem ioremap failed\n");
		return -EIO;
	}
	gs101_bcl_device->sysreg_cpucl0 = ioremap(SYSREG_CPUCL0_BASE, SZ_8K);
	if (!gs101_bcl_device->sysreg_cpucl0) {
		pr_err("sysreg_cpucl0 ioremap failed\n");
		return -EIO;
	}

	debugfs_create_file("cpucl0_clkdiv_stat", 0644,
			    gs101_bcl_device->debug_entry, gs101_bcl_device,
			    &cpucl0_clkdivstep_stat_fops);
	debugfs_create_file("cpucl1_clkdiv_stat", 0644,
			    gs101_bcl_device->debug_entry, gs101_bcl_device,
			    &cpucl1_clkdivstep_stat_fops);
	debugfs_create_file("cpucl2_clkdiv_stat", 0644,
			    gs101_bcl_device->debug_entry, gs101_bcl_device,
			    &cpucl2_clkdivstep_stat_fops);
	debugfs_create_file("mpmm_throttle", 0644,
			    gs101_bcl_device->debug_entry, gs101_bcl_device,
			    &mpmm_fops);
	debugfs_create_file("ppm_throttle", 0644, gs101_bcl_device->debug_entry,
			    gs101_bcl_device, &ppm_fops);

	mutex_lock(&sysreg_lock);
	reg = __raw_readl(gs101_bcl_device->sysreg_cpucl0 +
			  CLUSTER0_GENERAL_CTRL_64);
	reg |= MPMMEN_MASK;
	__raw_writel(reg, gs101_bcl_device->sysreg_cpucl0 +
				  CLUSTER0_GENERAL_CTRL_64);
	reg = __raw_readl(gs101_bcl_device->sysreg_cpucl0 + CLUSTER0_PPM);
	reg |= PPMEN_MASK;
	__raw_writel(reg, gs101_bcl_device->sysreg_cpucl0 + CLUSTER0_PPM);
	mutex_unlock(&sysreg_lock);

	return 0;
}

static int google_gs101_bcl_remove(struct platform_device *pdev)
{
	debugfs_remove(gs101_bcl_device->debug_entry);
	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "google,gs101-bcl" },
	{},
};

static struct platform_driver gs101_bcl_driver = {
	.probe  = google_gs101_bcl_probe,
	.remove = google_gs101_bcl_remove,
	.driver = {
		.name           = "google,gs101_bcl",
		.owner          = THIS_MODULE,
		.of_match_table = match_table,
	},
	.probe = google_gs101_bcl_probe,
	.remove = google_gs101_bcl_remove,
};

static int __init google_gs101_bcl_init(void)
{
	int ret;

	ret = platform_driver_register(&gs101_bcl_driver);
	if (ret < 0) {
		pr_err("device registration failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static void __init google_gs101_bcl_exit(void)
{
	platform_driver_unregister(&gs101_bcl_driver);
	pr_info("unregistered platform driver: gs101_bcl\n");
}

module_init(google_gs101_bcl_init);
module_exit(google_gs101_bcl_exit);
MODULE_DESCRIPTION("Google Battery Current Limiter");
MODULE_AUTHOR("George Lee <geolee@google.com>");
MODULE_LICENSE("GPL");
