// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 */

#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/kallsyms.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/panic_notifier.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/sjtag-driver.h>

#include "core_regs.h"

#define SYS_READ(reg, val)	asm volatile("mrs %0, " #reg : "=r" (val))
#define SYS_WRITE(reg, val)	asm volatile("msr " #reg ", %0" :: "r" (val))

#define DBG_UNLOCK(base)	\
	do { isb(); __raw_writel(OSLOCK_MAGIC, base + DBGLAR); } while (0)
#define DBG_LOCK(base)		\
	do { __raw_writel(0x1, base + DBGLAR); isb(); } while (0)

#define DBG_REG_MAX_SIZE	(6)
#define DBG_BW_REG_MAX_SIZE	(24)
#define OS_LOCK_FLAG		(DBG_REG_MAX_SIZE - 1)
#define ITERATION		(5)
#define MSB_PADDING		(0xFFFFFF0000000000)
#define MSB_MASKING		(0x0000FF0000000000)

struct exynos_coresight_info {
	struct device *dev;
	void __iomem **dbg_base;
	void __iomem **cti_base;
	void __iomem **pmu_base;
	void __iomem **sgi_base;
	bool halt_enabled;
	void __iomem *gpr_base;
	unsigned int dbgack_mask;
	bool retention_enabled;
	unsigned long **dbg_reg;
	unsigned long bw_reg[DBG_BW_REG_MAX_SIZE];
};

static struct exynos_coresight_info *ecs_info;

#if IS_ENABLED(CONFIG_LOCKUP_DETECTOR_OTHER_CPU)
extern struct atomic_notifier_head hardlockup_notifier_list;
#endif

static inline void sys_os_lock(void)
{
	u64 val = 0x1;

	SYS_WRITE(OSLAR_EL1, val);
	isb();
}

static inline void sys_os_unlock(void)
{
	u64 val = 0x0;

	isb();
	SYS_WRITE(OSLAR_EL1, val);
}

static inline void dbg_os_lock(void __iomem *base)
{
	__raw_writel(0x1, base + DBGOSLAR);
	isb();
}

static inline void dbg_os_unlock(void __iomem *base)
{
	isb();
	__raw_writel(0x0, base + DBGOSLAR);
}

static inline bool is_power_up(int cpu)
{
	return __raw_readl(ecs_info->dbg_base[cpu] + DBGPRSR) & POWER_UP;
}

static inline bool is_reset_state(int cpu)
{
	return __raw_readl(ecs_info->dbg_base[cpu] + DBGPRSR) & RESET_STATE;
}

struct exynos_cs_pcsr {
	unsigned long pc;
	int ns;
	int el;
};
static struct exynos_cs_pcsr **exynos_cs_pc;

static inline int exynos_cs_get_cpu_part_num(int cpu)
{
	u32 midr = __raw_readl(ecs_info->dbg_base[cpu] + MIDR);

	return MIDR_PARTNUM(midr);
}

static inline bool have_pc_offset(void __iomem *base)
{
	 return !(__raw_readl(base + DBGDEVID1) & 0xf);
}

static int exynos_cs_get_pc(int cpu, int iter)
{
	void __iomem *dbg_base, *pmu_base;
	unsigned long val = 0, valHi;
	int ns = -1, el = -1;

	if (!ecs_info)
		return -ENODEV;

	dbg_base = ecs_info->dbg_base[cpu];
	pmu_base = ecs_info->pmu_base[cpu];
	if (!is_power_up(cpu)) {
		dev_err(ecs_info->dev, "CPU%d is power down. (PRSR:0x%x)\n",
			cpu, __raw_readl(dbg_base + DBGPRSR));
		return -EACCES;
	}

	if (is_reset_state(cpu)) {
		dev_err(ecs_info->dev, "Power on but reset state!\n");
		return -EACCES;
	}

	switch (exynos_cs_get_cpu_part_num(cpu)) {
	case ARM_CPU_PART_CORTEX_A53:
	case ARM_CPU_PART_CORTEX_A57:
		DBG_UNLOCK(dbg_base);
		dbg_os_unlock(dbg_base);

		val = __raw_readl(dbg_base + DBGPCSRlo);
		valHi = __raw_readl(dbg_base + DBGPCSRhi);

		val |= (valHi << 32L);
		if (have_pc_offset(dbg_base))
			val -= 0x8;
		if (MSB_MASKING == (MSB_MASKING & val))
			val |= MSB_PADDING;

		dbg_os_lock(dbg_base);
		DBG_LOCK(dbg_base);
		break;
	case ARM_CPU_PART_CORTEX_A55:
	case ARM_CPU_PART_CORTEX_A75:
	case ARM_CPU_PART_CORTEX_A76:
	case ARM_CPU_PART_CORTEX_A77:
	case ARM_CPU_PART_CORTEX_A78:
	case ARM_CPU_PART_CORTEX_A510:
	case ARM_CPU_PART_CORTEX_A710:
	case ARM_CPU_PART_CORTEX_X1:
	case ARM_CPU_PART_CORTEX_X2:
	case ARM_CPU_PART_MAKALU:
	case ARM_CPU_PART_MAKALU_ELP:
	case ARM_CPU_PART_HAYDEN:
	case ARM_CPU_PART_HUNTER:
	case ARM_CPU_PART_HUNTER_ELP:
		DBG_UNLOCK(dbg_base);
		dbg_os_unlock(dbg_base);
		DBG_UNLOCK(pmu_base);

		val = __raw_readq(pmu_base + PMUPCSR);
		val = __raw_readq(pmu_base + PMUPCSR);
		ns = (val >> 63L) & 0x1;
		el = (val >> 61L) & 0x3;
		if (MSB_MASKING == (MSB_MASKING & val))
			val |= MSB_PADDING;

		DBG_LOCK(pmu_base);
		break;
	default:
		break;
	}

	exynos_cs_pc[cpu][iter].pc = val;
	exynos_cs_pc[cpu][iter].ns = ns;
	exynos_cs_pc[cpu][iter].el = el;

	return 0;
}

#if IS_ENABLED(CONFIG_LOCKUP_DETECTOR_OTHER_CPU)
static int exynos_cs_lockup_handler(struct notifier_block *nb,
					unsigned long l, void *core)
{
	unsigned long val = 0, iter;
	unsigned int *cpu = (unsigned int *)core;
	int ret = 0;

	if (!ecs_info) {
		pr_err("exynos-coresight disabled\n");
		return 0;
	}

	if (sjtag_is_locked()) {
		dev_err(ecs_info->dev, "Coresight requires SJTAG auth\n");
		return 0;
	}

	dev_err(ecs_info->dev, "CPU[%d] GICR_ISPENDR0: 0x%08x\n", *cpu,
		__raw_readl(ecs_info->sgi_base[*cpu] + GICR_ISPENDR0));
	dev_err(ecs_info->dev, "CPU[%d] saved pc value\n", *cpu);
	for (iter = 0; iter < ITERATION; iter++) {
#if IS_ENABLED(CONFIG_EXYNOS_PMU_IF)
		if (!exynos_cpu.power_state(*cpu))
			continue;
#endif
		ret = exynos_cs_get_pc(*cpu, iter);
		if (ret < 0)
			continue;

		val = exynos_cs_pc[*cpu][iter].pc;
		dev_err(ecs_info->dev, "      0x%016zx : %pS\n", val, (void *)val);
	}

	return 0;
}

static struct notifier_block exynos_cs_lockup_nb = {
	.notifier_call = exynos_cs_lockup_handler,
};
#endif

static int exynos_cs_panic_handler(struct notifier_block *np,
				unsigned long l, void *msg)
{
	unsigned long flags, val;
	unsigned int cpu, iter, curr_cpu;

	if (!ecs_info) {
		pr_err("exynos-coresight disabled\n");
		return 0;
	}

	if (num_online_cpus() <= 1)
		return 0;

	if (sjtag_is_locked()) {
		dev_err(ecs_info->dev, "Coresight requires SJTAG auth\n");
		return 0;
	}

	local_irq_save(flags);
	curr_cpu = raw_smp_processor_id();
	for (iter = 0; iter < ITERATION; iter++) {
		for_each_possible_cpu(cpu) {
			exynos_cs_pc[cpu][iter].pc = 0;
			if (cpu == curr_cpu)
				continue;
#if IS_ENABLED(CONFIG_EXYNOS_PMU_IF)
			if (!exynos_cpu.power_state(cpu))
				continue;
#endif
			if (exynos_cs_get_pc(cpu, iter) < 0)
				continue;
		}
	}

	local_irq_restore(flags);
	for_each_possible_cpu(cpu) {
		dev_err(ecs_info->dev, "CPU[%d] GICR_ISPENDR0: 0x%08x\n", cpu,
			__raw_readl(ecs_info->sgi_base[cpu] + GICR_ISPENDR0));
		dev_err(ecs_info->dev, "CPU[%d] saved pc value\n", cpu);
		for (iter = 0; iter < ITERATION; iter++) {
			val = exynos_cs_pc[cpu][iter].pc;
			if (!val)
				continue;

			dev_err(ecs_info->dev, "    0x%016zx : %pS\n",
					val, (void *)val);
		}
	}
	return 0;
}

static struct notifier_block exynos_cs_panic_nb = {
	.notifier_call = exynos_cs_panic_handler,
};

static int exynos_cs_halt_enable(unsigned int cpu)
{
	if (ecs_info->halt_enabled && ecs_info->gpr_base) {
		__raw_writel(OSLOCK_MAGIC, ecs_info->cti_base[cpu] + DBGLAR);
		__raw_writel(CTICH0, ecs_info->cti_base[cpu] + CTIGATE);
		__raw_writel(CTICH0, ecs_info->cti_base[cpu] + CTIINEN(0));
		__raw_writel(CTICH0, ecs_info->cti_base[cpu] + CTIOUTEN(0));
		__raw_writel(0x1, ecs_info->cti_base[cpu]);
	}

	return 0;
}

static int exynos_cs_halt_disable(unsigned int cpu)
{
	if (ecs_info->halt_enabled && ecs_info->gpr_base)
		__raw_writel(0x0, ecs_info->cti_base[cpu]);

	return 0;
}

static int exynos_cs_halt_suspend(struct device *dev)
{
	int cpu;

	if (ecs_info->halt_enabled) {
		for_each_possible_cpu(cpu) {
			exynos_cs_halt_disable(cpu);
		}
	}

	return 0;
}

static int exynos_cs_halt_resume(struct device *dev)
{
	int cpu;

	if (ecs_info->halt_enabled) {
		for_each_possible_cpu(cpu) {
			exynos_cs_halt_enable(cpu);
		}
	}

	return 0;
}

static const struct dev_pm_ops cs_pm_ops = {
	.suspend = exynos_cs_halt_suspend,
	.resume = exynos_cs_halt_resume,
};

int exynos_cs_stop_cpus(void)
{
	int cpu;

	if (!ecs_info || !ecs_info->halt_enabled || !ecs_info->gpr_base) {
		pr_err("exynos-coresight-halt is disabled\n");
		return -ENODEV;
	}

	local_irq_disable();
	cpu = raw_smp_processor_id();

	dev_info(ecs_info->dev, "cpu%d calls HALT!\n", cpu);

	/* gen sw dbgack for mask wdt reset */
	__raw_writel(OSLOCK_MAGIC, ecs_info->gpr_base + DBGLAR);
	isb();
	__raw_writel(ecs_info->dbgack_mask, ecs_info->gpr_base);
	isb();

	__raw_writel(CTICH0, ecs_info->cti_base[cpu] + CTIAPPPULSE);

	mdelay(500);
	dev_info(ecs_info->dev, "cpu%d Resume!\n", cpu);

	return 0;
}

static int exynos_cs_suspend_cpu(unsigned int cpu)
{
	int idx = 0;
	void __iomem *base;

	if (!ecs_info->retention_enabled)
		return 0;

	pr_debug("%s: cpu %d\n", __func__, cpu);
	base = ecs_info->dbg_base[cpu];

	sys_os_lock();
	SYS_READ(DBGBVR0_EL1, ecs_info->bw_reg[idx++]); /* DBGBVR */
	SYS_READ(DBGBVR1_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBVR2_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBVR3_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBVR4_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBVR5_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBCR0_EL1, ecs_info->bw_reg[idx++]); /* DBGDCR */
	SYS_READ(DBGBCR1_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBCR2_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBCR3_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBCR4_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGBCR5_EL1, ecs_info->bw_reg[idx++]);

	SYS_READ(DBGWVR0_EL1, ecs_info->bw_reg[idx++]); /* DBGWVR */
	SYS_READ(DBGWVR1_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGWVR2_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGWVR3_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGWCR0_EL1, ecs_info->bw_reg[idx++]); /* DBGDCR */
	SYS_READ(DBGWCR1_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGWCR2_EL1, ecs_info->bw_reg[idx++]);
	SYS_READ(DBGWCR3_EL1, ecs_info->bw_reg[idx++]);

	idx = 0;
	SYS_READ(MDSCR_EL1, ecs_info->dbg_reg[cpu][idx++]);
	SYS_READ(OSECCR_EL1, ecs_info->dbg_reg[cpu][idx++]);
	SYS_READ(OSDTRTX_EL1, ecs_info->dbg_reg[cpu][idx++]);
	SYS_READ(OSDTRRX_EL1, ecs_info->dbg_reg[cpu][idx++]);
	SYS_READ(DBGCLAIMCLR_EL1, ecs_info->dbg_reg[cpu][idx++]);

	sys_os_unlock();

	pr_debug("%s: cpu %d done\n", __func__, cpu);

	return 0;
}

static int exynos_cs_resume_cpu(unsigned int cpu)
{
	int idx = 0;
	void __iomem *base;

	if (!ecs_info->retention_enabled)
		return 0;

	base = ecs_info->dbg_base[cpu];
	pr_debug("%s: cpu %d\n", __func__, cpu);

	sys_os_lock();

	SYS_WRITE(MDSCR_EL1, ecs_info->dbg_reg[cpu][idx++]);
	SYS_WRITE(OSECCR_EL1, ecs_info->dbg_reg[cpu][idx++]);
	SYS_WRITE(OSDTRTX_EL1, ecs_info->dbg_reg[cpu][idx++]);
	SYS_WRITE(OSDTRRX_EL1, ecs_info->dbg_reg[cpu][idx++]);
	SYS_WRITE(DBGCLAIMSET_EL1, ecs_info->dbg_reg[cpu][idx++]);

	idx = 0;
	SYS_WRITE(DBGBVR0_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBVR1_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBVR2_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBVR3_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBVR4_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBVR5_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBCR0_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBCR1_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBCR2_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBCR3_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBCR4_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGBCR5_EL1, ecs_info->bw_reg[idx++]);

	SYS_WRITE(DBGWVR0_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGWVR1_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGWVR2_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGWVR3_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGWCR0_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGWCR1_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGWCR2_EL1, ecs_info->bw_reg[idx++]);
	SYS_WRITE(DBGWCR3_EL1, ecs_info->bw_reg[idx++]);

	sys_os_unlock();

	pr_debug("%s: %d done\n", __func__, cpu);
	return 0;
}

static int exynos_cs_c2_notifier(struct notifier_block *self,
		unsigned long cmd, void *v)
{
	int cpu = raw_smp_processor_id();

	switch (cmd) {
	case CPU_PM_ENTER:
		exynos_cs_halt_disable(cpu);
		exynos_cs_suspend_cpu(cpu);
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		exynos_cs_resume_cpu(cpu);
		exynos_cs_halt_enable(cpu);
		break;
	case CPU_CLUSTER_PM_ENTER:
		exynos_cs_halt_disable(cpu);
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
		exynos_cs_halt_enable(cpu);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_cs_c2_nb = {
	.notifier_call = exynos_cs_c2_notifier,
};

static const struct of_device_id of_exynos_cs_matches[] __initconst= {
	{.compatible = "exynos,coresight"},
	{},
};

static int exynos_cs_parsing_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	const __be32 *cur;
	u32 val, i = 0;
	int ret;

	ecs_info->dbg_base = devm_kcalloc(dev, num_possible_cpus(),
			sizeof(void *), GFP_KERNEL);
	if (!ecs_info->dbg_base)
		return -ENOMEM;

	ecs_info->cti_base = devm_kcalloc(dev, num_possible_cpus(),
			sizeof(void *), GFP_KERNEL);
	if (!ecs_info->cti_base)
		return -ENOMEM;

	ecs_info->pmu_base = devm_kcalloc(dev, num_possible_cpus(),
			sizeof(void *), GFP_KERNEL);
	if (!ecs_info->pmu_base)
		return -ENOMEM;

	ecs_info->sgi_base = devm_kcalloc(dev, num_possible_cpus(),
			sizeof(void *), GFP_KERNEL);
	if (!ecs_info->sgi_base)
		return -ENOMEM;

	of_property_for_each_u32(np, "dbg_base", prop, cur, val) {
		if (i >= num_possible_cpus() || !val)
			return -EINVAL;

		ecs_info->dbg_base[i] = devm_ioremap(dev, val, SZ_4K);
		if (!ecs_info->dbg_base[i]) {
			dev_err(ecs_info->dev, "fail property %s(%d) ioremap\n",
					prop->name, i);
			return -ENOMEM;
		}
		i++;
	}

	i = 0;
	of_property_for_each_u32(np, "cti_base", prop, cur, val) {
		if (i >= num_possible_cpus() || !val)
			return -EINVAL;
		ecs_info->cti_base[i] = devm_ioremap(dev, val, SZ_4K);
		if (!ecs_info->cti_base[i]) {
			dev_err(ecs_info->dev, "fail property %s(%d) ioremap\n",
					prop->name, i);
			return -ENOMEM;
		}
		i++;
	}

	i = 0;
	of_property_for_each_u32(np, "pmu_base", prop, cur, val) {
		if (i >= num_possible_cpus() || !val)
			return -EINVAL;
		ecs_info->pmu_base[i] = devm_ioremap(dev, val, SZ_4K);
		if (!ecs_info->pmu_base[i]) {
			dev_err(ecs_info->dev, "fail property %s(%d) ioremap\n",
					prop->name, i);
			return -ENOMEM;
		}
		i++;
	}

	i = 0;
	of_property_for_each_u32(np, "sgi_base", prop, cur, val) {
		if (i >= num_possible_cpus() || !val)
			return -EINVAL;
		ecs_info->sgi_base[i] = devm_ioremap(dev, val, SZ_64K);
		if (!ecs_info->sgi_base[i]) {
			dev_err(ecs_info->dev, "fail property %s(%d) ioremap\n",
					prop->name, i);
			return -ENOMEM;
		}
		i++;
	}

	ret = of_property_read_u32(np, "halt", &val);
	if (ret || !val) {
		dev_err(ecs_info->dev, "disable halt function\n");
		goto retention;
	}
	ecs_info->halt_enabled = val;

	ret = of_property_read_u32(np, "gpr_base", &val);
	if (ret) {
		dev_err(ecs_info->dev, "no such gpr_base\n");
		return 0;
	}
	ecs_info->gpr_base = ioremap(val, SZ_4K);
	if (!ecs_info->gpr_base) {
		dev_err(ecs_info->dev, "failed gpr_base ioremap\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(np, "dbgack-mask", &val);
	if (ret) {
		dev_info(ecs_info->dev, "no such dbgack-mask\n");
		return 0;
	}
	ecs_info->dbgack_mask = val;
retention:
	ret = of_property_read_u32(np, "retention", &val);
	if (ret || !val) {
		dev_err(ecs_info->dev, "disable retention debug register\n");
		return 0;
	}
	ecs_info->retention_enabled = val;

	return 0;
}

static int exynos_coresight_probe(struct platform_device *pdev)
{
	int ret = 0, cpu;

	ecs_info = devm_kzalloc(&pdev->dev,
			sizeof(struct exynos_coresight_info), GFP_KERNEL);
	if (!ecs_info)
		return -ENOMEM;

	ecs_info->dev = &pdev->dev;
	ret = exynos_cs_parsing_dt(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s failed.\n", __func__);
		return ret;
	}

	ecs_info->dbg_reg = devm_kcalloc(&pdev->dev, num_possible_cpus(),
			sizeof(unsigned long *), GFP_KERNEL);
	if (!ecs_info->dbg_reg)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		ecs_info->dbg_reg[cpu] = devm_kcalloc(&pdev->dev, DBG_REG_MAX_SIZE,
				sizeof(unsigned long), GFP_KERNEL);
		if (!ecs_info->dbg_reg[cpu])
			return -ENOMEM;
	}

	exynos_cs_pc = devm_kcalloc(&pdev->dev, num_possible_cpus(),
			sizeof(struct exynos_cs_pcsr *), GFP_KERNEL);
	if (!exynos_cs_pc)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		exynos_cs_pc[cpu] = devm_kcalloc(&pdev->dev, ITERATION,
				sizeof(struct exynos_cs_pcsr), GFP_KERNEL);
		if (!exynos_cs_pc[cpu])
			return -ENOMEM;
	}


#if IS_ENABLED(CONFIG_LOCKUP_DETECTOR_OTHER_CPU)
	atomic_notifier_chain_register(&hardlockup_notifier_list,
			&exynos_cs_lockup_nb);
#endif

	cpus_read_lock();
	if (ecs_info->halt_enabled || ecs_info->retention_enabled) {
		if (ecs_info->halt_enabled) {
			if (!sjtag_is_locked()) {
				for_each_online_cpu(cpu) {
					exynos_cs_halt_enable(cpu);
				}
			}
			ret = cpuhp_setup_state_nocalls_cpuslocked(
					CPUHP_AP_ONLINE_DYN,
					"exynoscoresighthalt:online",
					exynos_cs_halt_enable,
					exynos_cs_halt_disable);
			if (ret < 0)
				goto err;
		}
		if (ecs_info->retention_enabled) {
			ret = cpuhp_setup_state_nocalls_cpuslocked(
					CPUHP_AP_ONLINE_DYN,
					"exynoscoresight:online",
					exynos_cs_resume_cpu,
					exynos_cs_suspend_cpu);
			if (ret < 0)
				goto err;
		}
		ret = cpu_pm_register_notifier(&exynos_cs_c2_nb);
		if (ret < 0)
			goto err;
	}
	cpus_read_unlock();

	atomic_notifier_chain_register(&panic_notifier_list,
			&exynos_cs_panic_nb);
	dbg_snapshot_register_debug_ops(exynos_cs_stop_cpus, NULL, NULL);
	dev_info(&pdev->dev, "%s Successful\n", __func__);
	return ret;

err:
	cpus_read_unlock();
	return ret;
}

static const struct of_device_id exynos_coresight_matches[] = {
	{ .compatible = "google,exynos-coresight", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_coresight_matches);

static struct platform_driver exynos_coresight_driver = {
	.probe		= exynos_coresight_probe,
	.driver		= {
		.name	= "exynos-coresight",
		.of_match_table	= of_match_ptr(exynos_coresight_matches),
		.pm = &cs_pm_ops,
	},
};
module_platform_driver(exynos_coresight_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos Coresight Driver");
