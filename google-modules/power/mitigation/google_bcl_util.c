// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_util.c Google bcl driver - Utility
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/mfd/samsung/s2mpg12-register.h>
#include <linux/mfd/samsung/s2mpg13-register.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
#include <linux/mfd/samsung/s2mpg1415.h>
#include <linux/mfd/samsung/s2mpg1415-register.h>
#endif

#include <soc/google/cal-if.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include "bcl.h"

const unsigned int subsystem_pmu[] = {
	PMU_ALIVE_CPU0_STATES,
	PMU_ALIVE_CPU1_STATES,
	PMU_ALIVE_CPU2_STATES,
	PMU_ALIVE_TPU_STATES,
	PMU_ALIVE_GPU_STATES,
	PMU_ALIVE_AUR_STATES
};

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
#define PMIC_MAIN_WRITE_REG(i2c, reg, val) s2mpg10_write_reg(i2c, reg, val)
#define PMIC_SUB_WRITE_REG(i2c, reg, val) s2mpg11_write_reg(i2c, reg, val)
#define PMIC_MAIN_READ_REG(i2c, reg, val) s2mpg10_read_reg(i2c, reg, val)
#define PMIC_SUB_READ_REG(i2c, reg, val) s2mpg11_read_reg(i2c, reg, val)
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
#define PMIC_MAIN_WRITE_REG(i2c, reg, val) s2mpg12_write_reg(i2c, reg, val)
#define PMIC_SUB_WRITE_REG(i2c, reg, val) s2mpg13_write_reg(i2c, reg, val)
#define PMIC_MAIN_READ_REG(i2c, reg, val) s2mpg12_read_reg(i2c, reg, val)
#define PMIC_SUB_READ_REG(i2c, reg, val) s2mpg13_read_reg(i2c, reg, val)
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
#define PMIC_MAIN_WRITE_REG(i2c, reg, val) s2mpg14_write_reg(i2c, reg, val)
#define PMIC_SUB_WRITE_REG(i2c, reg, val) s2mpg15_write_reg(i2c, reg, val)
#define PMIC_MAIN_READ_REG(i2c, reg, val) s2mpg14_read_reg(i2c, reg, val)
#define PMIC_SUB_READ_REG(i2c, reg, val) s2mpg15_read_reg(i2c, reg, val)
#endif

int meter_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value)
{
	switch (pmic) {
	case CORE_PMIC_SUB:
		return PMIC_SUB_WRITE_REG((bcl_dev)->sub_meter_i2c, reg, value);
	case CORE_PMIC_MAIN:
		return PMIC_MAIN_WRITE_REG((bcl_dev)->main_meter_i2c, reg, value);
	}
	return 0;
}

static int bcl_dev_cpu_notifier(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	int cpu = raw_smp_processor_id();
	struct bcl_device *bcl_dev;
	unsigned int i, bit_mask;
	int cpu_ind;

	if (action != CPU_PM_EXIT)
		return NOTIFY_OK;

	bcl_dev = container_of(nfb, struct bcl_device, cpu_nb);
	if (!bcl_dev)
		return -ENODEV;

	if (!bcl_dev->enabled)
		return -ENODEV;

	if (cpu < bcl_dev->cpu1_cluster)
		return NOTIFY_OK;

	if (cpu >= bcl_dev->cpu1_cluster && cpu < bcl_dev->cpu2_cluster)
		cpu_ind = MID_CLUSTER;
	else
		cpu_ind = BIG_CLUSTER;

	if (bcl_dev->cpu_buff_conf[cpu_ind].wr_update_rqd == 0 &&
	    bcl_dev->cpu_buff_conf[cpu_ind].rd_update_rqd == 0)
		return NOTIFY_OK;

	for (i = 0; i < CPU_BUFF_VALS_MAX; i++) {
		bit_mask = BIT(i);
		if (bcl_dev->cpu_buff_conf[cpu_ind].wr_update_rqd & bit_mask) {
			__raw_writel(bcl_dev->cpu_buff_conf[cpu_ind].buff[i],
				     bcl_dev->core_conf[SUBSYSTEM_CPU0 + cpu_ind].base_mem +
				     bcl_dev->cpu_buff_conf[cpu_ind].addr[i]);
			bcl_dev->cpu_buff_conf[cpu_ind].wr_update_rqd &= ~bit_mask;
		}
		if (bcl_dev->cpu_buff_conf[cpu_ind].rd_update_rqd & bit_mask) {
			bcl_dev->cpu_buff_conf[cpu_ind].buff[i] =
				__raw_readl(bcl_dev->core_conf[SUBSYSTEM_CPU0 + cpu_ind].base_mem +
					    bcl_dev->cpu_buff_conf[cpu_ind].addr[i]);
			bcl_dev->cpu_buff_conf[cpu_ind].rd_update_rqd &= ~bit_mask;
		}
	}

	return NOTIFY_OK;
}

int cpu_buff_read(struct bcl_device *bcl_dev, int cluster, unsigned int type, unsigned int *reg)
{
	if (cluster < SUBSYSTEM_CPU0 || cluster > SUBSYSTEM_CPU2)
		return -EINVAL;

	if (cluster == SUBSYSTEM_CPU0) {
		*reg = __raw_readl(bcl_dev->core_conf[SUBSYSTEM_CPU0].base_mem +
				   bcl_dev->cpu_buff_conf[LITTLE_CLUSTER].addr[type]);
		return 0;
	}

	*reg = bcl_dev->cpu_buff_conf[cluster].buff[type];
	bcl_dev->cpu_buff_conf[cluster].rd_update_rqd |= BIT(type);
	return 0;
}

int cpu_buff_write(struct bcl_device *bcl_dev, int cluster, unsigned int type, unsigned int val)
{
	if (cluster < SUBSYSTEM_CPU0 || cluster > SUBSYSTEM_CPU2)
		return -EINVAL;

	if (cluster == SUBSYSTEM_CPU0) {
		__raw_writel(val, bcl_dev->core_conf[SUBSYSTEM_CPU0].base_mem +
			     bcl_dev->cpu_buff_conf[LITTLE_CLUSTER].addr[type]);
		return 0;
	}

	bcl_dev->cpu_buff_conf[cluster].buff[type] = val;
	bcl_dev->cpu_buff_conf[cluster].wr_update_rqd |= BIT(type);
	return 0;
}

int cpu_sfr_write(struct bcl_device *bcl_dev, int idx, void __iomem *addr, unsigned int value)
{
	mutex_lock(&bcl_dev->cpu_ratio_lock);
	if (!bcl_disable_power(bcl_dev, idx)) {
		mutex_unlock(&bcl_dev->cpu_ratio_lock);
		return -EIO;
	}
	__raw_writel(value, addr);
	bcl_enable_power(bcl_dev, idx);
	mutex_unlock(&bcl_dev->cpu_ratio_lock);
	return 0;
}

int cpu_sfr_read(struct bcl_device *bcl_dev, int idx, void __iomem *addr, unsigned int *reg)
{
	mutex_lock(&bcl_dev->cpu_ratio_lock);
	if (!bcl_disable_power(bcl_dev, idx)) {
		mutex_unlock(&bcl_dev->cpu_ratio_lock);
		return -EIO;
	}
	*reg = __raw_readl(addr);
	bcl_enable_power(bcl_dev, idx);
	mutex_unlock(&bcl_dev->cpu_ratio_lock);

	return 0;
}

int meter_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value)
{
	switch (pmic) {
	case CORE_PMIC_SUB:
		return PMIC_SUB_READ_REG((bcl_dev)->sub_meter_i2c, reg, value);
	case CORE_PMIC_MAIN:
		return PMIC_MAIN_READ_REG((bcl_dev)->main_meter_i2c, reg, value);
	}
	return 0;
}

int pmic_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value)
{
	switch (pmic) {
	case CORE_PMIC_SUB:
		return PMIC_SUB_WRITE_REG((bcl_dev)->sub_pmic_i2c, reg, value);
	case CORE_PMIC_MAIN:
		return PMIC_MAIN_WRITE_REG((bcl_dev)->main_pmic_i2c, reg, value);
	case CORE_PMIC_MAIN_RTC:
		return PMIC_MAIN_WRITE_REG((bcl_dev)->main_rtc_i2c, reg, value);
	}
	return 0;
}

int pmic_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value)
{
	switch (pmic) {
	case CORE_PMIC_SUB:
		return PMIC_SUB_READ_REG((bcl_dev)->sub_pmic_i2c, reg, value);
	case CORE_PMIC_MAIN:
		return PMIC_MAIN_READ_REG((bcl_dev)->main_pmic_i2c, reg, value);
	case CORE_PMIC_MAIN_RTC:
		return PMIC_MAIN_READ_REG((bcl_dev)->main_rtc_i2c, reg, value);
	}
	return 0;
}

bool bcl_is_cluster_on(struct bcl_device *bcl_dev, int cluster)
{
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	unsigned int addr, value = 0;

	if (cluster < bcl_dev->cpu2_cluster) {
		addr = CLUSTER1_NONCPU_STATES;
		exynos_pmu_read(addr, &value);
		return value & BIT(4);
	}
	if (cluster == bcl_dev->cpu2_cluster) {
		addr = CLUSTER2_NONCPU_STATES;
		exynos_pmu_read(addr, &value);
		return value & BIT(4);
	}
	return false;
#endif
	return true;
}

bool bcl_is_subsystem_on(struct bcl_device *bcl_dev, unsigned int addr)
{
	unsigned int value;

	switch (addr) {
	case PMU_ALIVE_TPU_STATES:
	case PMU_ALIVE_GPU_STATES:
	case PMU_ALIVE_AUR_STATES:
		exynos_pmu_read(addr, &value);
		return !(value & BIT(7));
	case PMU_ALIVE_CPU0_STATES:
		return true;
	case PMU_ALIVE_CPU1_STATES:
		return bcl_is_cluster_on(bcl_dev, bcl_dev->cpu1_cluster);
	case PMU_ALIVE_CPU2_STATES:
		return bcl_is_cluster_on(bcl_dev, bcl_dev->cpu2_cluster);
	}
	return false;
}

bool bcl_disable_power(struct bcl_device *bcl_dev, int cluster)
{
	if (IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)) {
		int i;
		if (cluster == SUBSYSTEM_CPU1)
			for (i = bcl_dev->cpu1_cluster; i < bcl_dev->cpu2_cluster; i++)
				disable_power_mode(i, POWERMODE_TYPE_CLUSTER);
		else if (cluster == SUBSYSTEM_CPU2)
			disable_power_mode(bcl_dev->cpu2_cluster, POWERMODE_TYPE_CLUSTER);
	}
	return true;
}

bool bcl_enable_power(struct bcl_device *bcl_dev, int cluster)
{
	if (IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)) {
		int i;
		if (cluster == SUBSYSTEM_CPU1)
			for (i = bcl_dev->cpu1_cluster; i < bcl_dev->cpu2_cluster; i++)
				enable_power_mode(i, POWERMODE_TYPE_CLUSTER);
		else if (cluster == SUBSYSTEM_CPU2)
			enable_power_mode(bcl_dev->cpu2_cluster, POWERMODE_TYPE_CLUSTER);
	}
	return true;
}

int google_bcl_init_notifier(struct bcl_device *bcl_dev)
{
	int ret, i;

	for (i = LITTLE_CLUSTER; i < CPU_CLUSTER_MAX; i++) {
		bcl_dev->cpu_buff_conf[i].addr[CPU_BUFF_CON_HEAVY] =
			(i == LITTLE_CLUSTER) ? CPUCL0_CLKDIVSTEP_CON : CLKDIVSTEP_CON_HEAVY;
		bcl_dev->cpu_buff_conf[i].addr[CPU_BUFF_CON_LIGHT] =
			(i == LITTLE_CLUSTER) ? CPUCL0_CLKDIVSTEP_CON : CLKDIVSTEP_CON_LIGHT;
		bcl_dev->cpu_buff_conf[i].addr[CPU_BUFF_CLKDIVSTEP] = CLKDIVSTEP;
		bcl_dev->cpu_buff_conf[i].addr[CPU_BUFF_VDROOP_FLT] = VDROOP_FLT;
		bcl_dev->cpu_buff_conf[i].addr[CPU_BUFF_CLK_STATS] =
			(i == LITTLE_CLUSTER) ? CPUCL0_CLKDIVSTEP_STAT : CLKDIVSTEP_STAT;
		bcl_dev->cpu_buff_conf[i].rd_update_rqd = BIT(CPU_BUFF_VALS_MAX) - 1;
	}
	bcl_dev->cpu_nb.notifier_call = bcl_dev_cpu_notifier;
	ret = cpu_pm_register_notifier(&bcl_dev->cpu_nb);

	return ret;
}

