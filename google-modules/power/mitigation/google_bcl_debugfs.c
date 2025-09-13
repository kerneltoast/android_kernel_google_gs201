// SPDX-License-Identifier: GPL-2.0-only
/*
 * google_bcl_debugfs.c Google bcl debug fs driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include "bcl.h"
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

static const unsigned int xclkout_source[] = {
	XCLKOUT_SOURCE_CPU0,
	XCLKOUT_SOURCE_CPU1,
	XCLKOUT_SOURCE_CPU2,
	XCLKOUT_SOURCE_TPU,
	XCLKOUT_SOURCE_GPU
};

static int get_xclk(void *data, u64 *val, int idx)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;
	unsigned int reg;

	*val = 0;
	switch (idx) {
	case SUBSYSTEM_CPU0:
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		addr = bcl_dev->core_conf[idx].base_mem + CLKOUT;
		if (cpu_sfr_read(bcl_dev, idx, addr, &reg) == 0)
			*val = (u64) reg;
		break;
	case SUBSYSTEM_GPU:
	case SUBSYSTEM_TPU:
		*val = bcl_dev->core_conf[idx].clk_out;
		break;
	}

	return 0;
}

static int set_xclk(void *data, u64 val, int idx)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;

	switch (idx) {
	case SUBSYSTEM_CPU0:
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		addr = bcl_dev->core_conf[idx].base_mem + CLKOUT;
		cpu_sfr_write(bcl_dev, idx, addr, val);
		break;
	case SUBSYSTEM_GPU:
	case SUBSYSTEM_TPU:
	case SUBSYSTEM_AUR:
		bcl_dev->core_conf[idx].clk_out = val;
		break;
	}

	exynos_pmu_write(PMU_CLK_OUT, val ? xclkout_source[idx] : 0);
	return 0;
}

static int get_cpu0clk(void *data, u64 *val)
{
	return get_xclk(data, val, SUBSYSTEM_CPU0);
}

static int set_cpu0clk(void *data, u64 val)
{
	return set_xclk(data, val, SUBSYSTEM_CPU0);
}

static int get_cpu1clk(void *data, u64 *val)
{
	return get_xclk(data, val, SUBSYSTEM_CPU1);
}

static int set_cpu1clk(void *data, u64 val)
{
	return set_xclk(data, val, SUBSYSTEM_CPU1);
}

static int get_cpu2clk(void *data, u64 *val)
{
	return get_xclk(data, val, SUBSYSTEM_CPU2);
}

static int set_cpu2clk(void *data, u64 val)
{
	return set_xclk(data, val, SUBSYSTEM_CPU2);
}

static int get_gpuclk(void *data, u64 *val)
{
	return get_xclk(data, val, SUBSYSTEM_GPU);
}

static int set_gpuclk(void *data, u64 val)
{
	return set_xclk(data, val, SUBSYSTEM_GPU);
}

static int get_tpuclk(void *data, u64 *val)
{
	return get_xclk(data, val, SUBSYSTEM_TPU);
}

static int set_tpuclk(void *data, u64 val)
{
	return set_xclk(data, val, SUBSYSTEM_TPU);
}

static int get_modem_gpio1(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;

	*val = gpio_get_value(bcl_dev->modem_gpio1_pin);
	return 0;
}

static int set_modem_gpio1(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	gpio_set_value(bcl_dev->modem_gpio1_pin, val);
	return 0;
}

static int get_modem_gpio2(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;

	*val = gpio_get_value(bcl_dev->modem_gpio2_pin);
	return 0;
}

static int set_modem_gpio2(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	gpio_set_value(bcl_dev->modem_gpio2_pin, val);
	return 0;
}

static int get_add_perph(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;

	*val = (u64)bcl_dev->add_perph;
	return 0;
}

static int set_add_perph(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	if (val < 0 || val > SUBSYSTEM_SOURCE_MAX)
		return -EINVAL;

	bcl_dev->add_perph = (u8)val;
	return 0;
}

static int get_add_addr(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;

	*val = bcl_dev->add_addr;
	return 0;
}

static int set_add_addr(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	if (val < 0 || val > SZ_128)
		return -EINVAL;

	bcl_dev->add_addr = val;
	return 0;
}

static int get_add_data(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *read_addr;

	if (bcl_dev->add_addr > SZ_128)
		return -EINVAL;

	if (bcl_dev->add_perph > SUBSYSTEM_SOURCE_MAX)
		return -EINVAL;

	mutex_lock(&bcl_dev->sysreg_lock);
	if ((bcl_dev->add_perph < SUBSYSTEM_TPU) && (bcl_dev->add_perph != SUBSYSTEM_CPU0)) {
		if (!bcl_disable_power(bcl_dev, bcl_dev->add_perph)) {
			mutex_unlock(&bcl_dev->sysreg_lock);
			return 0;
		}
	}
	read_addr = bcl_dev->base_add_mem[bcl_dev->add_perph] + bcl_dev->add_addr;
	*val = __raw_readl(read_addr);
	if ((bcl_dev->add_perph < SUBSYSTEM_TPU) && (bcl_dev->add_perph != SUBSYSTEM_CPU0))
		bcl_enable_power(bcl_dev, bcl_dev->add_perph);
	mutex_unlock(&bcl_dev->sysreg_lock);

	return 0;
}

static int set_add_data(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *write_addr;

	if (bcl_dev->add_addr > SZ_128)
		return -EINVAL;

	if (bcl_dev->add_perph > SUBSYSTEM_SOURCE_MAX)
		return -EINVAL;

	if (!bcl_dev)
		return -ENOMEM;

	if (!bcl_dev->base_add_mem[bcl_dev->add_perph]) {
		pr_err("Error in ADD perph\n");
		return -ENOMEM;
	}

	mutex_lock(&bcl_dev->sysreg_lock);
	if ((bcl_dev->add_perph < SUBSYSTEM_TPU) && (bcl_dev->add_perph != SUBSYSTEM_CPU0)) {
		if (!bcl_disable_power(bcl_dev, bcl_dev->add_perph)) {
			mutex_unlock(&bcl_dev->sysreg_lock);
			return 0;
		}
	}
	write_addr = bcl_dev->base_add_mem[bcl_dev->add_perph] + bcl_dev->add_addr;
	__raw_writel(val, write_addr);
	if ((bcl_dev->add_perph < SUBSYSTEM_TPU) && (bcl_dev->add_perph != SUBSYSTEM_CPU0))
		bcl_enable_power(bcl_dev, bcl_dev->add_perph);
	mutex_unlock(&bcl_dev->sysreg_lock);
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(cpu0_clkout_fops, get_cpu0clk, set_cpu0clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(cpu1_clkout_fops, get_cpu1clk, set_cpu1clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(cpu2_clkout_fops, get_cpu2clk, set_cpu2clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(gpu_clkout_fops, get_gpuclk, set_gpuclk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(tpu_clkout_fops, get_tpuclk, set_tpuclk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(modem_gpio1_fops, get_modem_gpio1, set_modem_gpio1, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(modem_gpio2_fops, get_modem_gpio2, set_modem_gpio2, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(add_perph_fops, get_add_perph, set_add_perph, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(add_addr_fops, get_add_addr, set_add_addr, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(add_data_fops, get_add_data, set_add_data, "0x%llx\n");

void google_init_debugfs(struct bcl_device *bcl_dev)
{
	struct dentry *dentry_add;
	bcl_dev->debug_entry = debugfs_create_dir("google_bcl", 0);
	debugfs_create_file("cpu0_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu0_clkout_fops);
	debugfs_create_file("cpu1_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu1_clkout_fops);
	debugfs_create_file("cpu2_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu2_clkout_fops);
	debugfs_create_file("gpu_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &gpu_clkout_fops);
	debugfs_create_file("tpu_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &tpu_clkout_fops);
	debugfs_create_file("modem_gpio1", 0644, bcl_dev->debug_entry, bcl_dev, &modem_gpio1_fops);
	debugfs_create_file("modem_gpio2", 0644, bcl_dev->debug_entry, bcl_dev, &modem_gpio2_fops);
	dentry_add = debugfs_create_dir("add", bcl_dev->debug_entry);
	debugfs_create_file("perph", 0600, dentry_add, bcl_dev, &add_perph_fops);
	debugfs_create_file("addr", 0600, dentry_add, bcl_dev, &add_addr_fops);
	debugfs_create_file("data", 0600, dentry_add, bcl_dev, &add_data_fops);
}

