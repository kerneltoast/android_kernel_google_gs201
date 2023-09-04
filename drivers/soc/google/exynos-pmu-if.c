// SPDX-License-Identifier: GPL-2.0
/*
 * PMU (Power Management Unit) Register Access Support.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/smp.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <soc/google/exynos-pmu-if.h>
#include <linux/mod_devicetable.h>
#include <soc/google/exynos-el3_mon.h>

/**
 * "pmureg" has the mapped base address of PMU(Power Management Unit)
 */
static struct regmap *pmureg;
static phys_addr_t pmu_alive_pa;
static spinlock_t update_lock;

struct pmu_lock_data {
	raw_spinlock_t lock;
	unsigned long irqflags;
};

/* Atomic operation for PMU_ALIVE registers. (offset 0~0x3FFF)
 *  When the targer register can be accessed by multiple masters,
 *  This functions should be used.
 */
static inline void exynos_pmu_set_bit_atomic(unsigned int offset,
						unsigned int val)
{
	set_priv_reg(pmu_alive_pa + (offset | 0xc000), val);
}

static inline void exynos_pmu_clr_bit_atomic(unsigned int offset,
						unsigned int val)
{
	set_priv_reg(pmu_alive_pa + (offset | 0x8000), val);
}

static int exynos_pmu_update_bits(unsigned int offset, unsigned int mask, unsigned int val)
{
	return rmw_priv_reg(pmu_alive_pa + offset, mask, val);
}

/**
 * No driver refers the "pmureg" directly, through the only exported API.
 */
int exynos_pmu_read(unsigned int offset, unsigned int *val)
{
	return regmap_read(pmureg, offset, val);
}
EXPORT_SYMBOL(exynos_pmu_read);

int exynos_pmu_write(unsigned int offset, unsigned int val)
{
	return set_priv_reg(pmu_alive_pa + offset, val);
}
EXPORT_SYMBOL(exynos_pmu_write);

int exynos_pmu_update(unsigned int offset, unsigned int mask, unsigned int val)
{
	int i;
	unsigned long flags;

	if (offset > 0x3fff)
		return exynos_pmu_update_bits(offset, mask, val);

	spin_lock_irqsave(&update_lock, flags);

	for (i = 0; i < 32; i++) {
		if (mask & (1 << i)) {
			if (val & (1 << i))
				exynos_pmu_set_bit_atomic(offset, i);
			else
				exynos_pmu_clr_bit_atomic(offset, i);
		}
	}

	spin_unlock_irqrestore(&update_lock, flags);

	return 0;
}
EXPORT_SYMBOL(exynos_pmu_update);

#define PMU_CPU_CONFIG_BASE			0x1000
#define PMU_CPU_STATUS_BASE			0x1004
#define CPU_LOCAL_PWR_CFG			0x1

static int pmu_cpu_offset(unsigned int cpu)
{
	unsigned int offset = 0;

	switch (cpu) {
	case 0:
		offset = 0x0;
		break;
	case 1:
		offset = 0x80;
		break;
	case 2:
		offset = 0x100;
		break;
	case 3:
		offset = 0x180;
		break;
	case 4:
		offset = 0x300;
		break;
	case 5:
		offset = 0x380;
		break;
	case 6:
		offset = 0x500;
		break;
	case 7:
		offset = 0x580;
		break;
	default:
		pr_err("CPU index out-of-bound\n");
		WARN_ON(1);
		break;
	}
	return offset;
}

static void pmu_cpu_ctrl(unsigned int cpu, int enable)
{
	unsigned int offset;

	offset = pmu_cpu_offset(cpu);
	exynos_pmu_update_bits(PMU_CPU_CONFIG_BASE + offset,
			       CPU_LOCAL_PWR_CFG,
			       enable ? CPU_LOCAL_PWR_CFG : 0);
}

static int pmu_cpu_state(unsigned int cpu)
{
	unsigned int offset, val = 0;

	offset = pmu_cpu_offset(cpu);
	regmap_read(pmureg, PMU_CPU_STATUS_BASE + offset, &val);

	return ((val & CPU_LOCAL_PWR_CFG) == CPU_LOCAL_PWR_CFG);
}

#define CLUSTER_ADDR_OFFSET			0x8
#define PMU_NONCPU_CONFIG_BASE			0x2040
#define PMU_NONCPU_STATUS_BASE			0x2044
#define PMU_MEMORY_CLUSTER1_NONCPU_STATUS	0x2380
#define MEMORY_CLUSTER_ADDR_OFFSET		0x21C
#define NONCPU_LOCAL_PWR_CFG			0xF
#define SHARED_CACHE_LOCAL_PWR_CFG		0x1

static void pmu_cluster_ctrl(unsigned int cpu, int enable)
{
	unsigned int offset = 0;

	exynos_pmu_update_bits(PMU_NONCPU_CONFIG_BASE + offset,
			       NONCPU_LOCAL_PWR_CFG,
			       enable ? NONCPU_LOCAL_PWR_CFG : 0);
}

static bool pmu_noncpu_state(unsigned int cpu)
{
	unsigned int noncpu_stat = 0;
	unsigned int offset = 0;

	regmap_read(pmureg,
		PMU_NONCPU_STATUS_BASE + offset, &noncpu_stat);

	return !!(noncpu_stat & NONCPU_LOCAL_PWR_CFG);
}

static int pmu_shared_cache_state(unsigned int cpu)
{
	unsigned int shared_stat = 0;
	unsigned int offset = 0;

	regmap_read(pmureg,
		PMU_MEMORY_CLUSTER1_NONCPU_STATUS + offset, &shared_stat);

	return (shared_stat & SHARED_CACHE_LOCAL_PWR_CFG);
}

static void exynos_cpu_up(unsigned int cpu)
{
	pmu_cpu_ctrl(cpu, 1);
}

static void exynos_cpu_down(unsigned int cpu)
{
	pmu_cpu_ctrl(cpu, 0);
}

static int exynos_cpu_state(unsigned int cpu)
{
	return pmu_cpu_state(cpu);
}

static void exynos_cluster_up(unsigned int cpu)
{
	pmu_cluster_ctrl(cpu, false);
}

static void exynos_cluster_down(unsigned int cpu)
{
	pmu_cluster_ctrl(cpu, true);
}

static int exynos_cluster_state(unsigned int cpu)
{
	return pmu_shared_cache_state(cpu) &&
			pmu_noncpu_state(cpu);
}

struct exynos_cpu_power_ops exynos_cpu = {
	.power_up = exynos_cpu_up,
	.power_down = exynos_cpu_down,
	.power_state = exynos_cpu_state,
	.cluster_up = exynos_cluster_up,
	.cluster_down = exynos_cluster_down,
	.cluster_state = exynos_cluster_state,
};
EXPORT_SYMBOL_GPL(exynos_cpu);

#ifdef CONFIG_CP_PMUCAL
#define PMU_CP_STAT		0x0038
int exynos_check_cp_status(void)
{
	unsigned int val;

	exynos_pmu_read(PMU_CP_STAT, &val);

	return val;
}
#endif

static struct bus_type exynos_info_subsys = {
	.name = "exynos_info",
	.dev_name = "exynos_info",
};

#define NR_CPUS_PER_CLUSTER		4
static ssize_t core_status_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	ssize_t n = 0;
	int cpu;

	for_each_possible_cpu(cpu) {
		/*
		 * Each cluster has four cores.
		 * "cpu % NR_CPUS_PER_CLUSTER == 0" means that
		 * the cpu is a first one of each cluster.
		 */
		if (!(cpu % NR_CPUS_PER_CLUSTER)) {
			n += scnprintf(buf + n, PAGE_SIZE - n,
					"%s shared_cache : %d\n",
				(!cpu) ? "boot" : "nonboot",
				pmu_shared_cache_state(cpu));

			n += scnprintf(buf + n, PAGE_SIZE - n,
					"%s Noncpu : %d\n",
				(!cpu) ? "boot" : "nonboot",
				pmu_noncpu_state(cpu));
		}
		n += scnprintf(buf + n, PAGE_SIZE - n,
				"CPU%d : %d\n",
				cpu, pmu_cpu_state(cpu));
	}

	return n;
}

static struct kobj_attribute cs_attr =
	__ATTR(core_status, 0644, core_status_show, NULL);

static struct attribute *cs_sysfs_attrs[] = {
	&cs_attr.attr,
	NULL,
};

static struct attribute_group cs_sysfs_group = {
	.attrs = cs_sysfs_attrs,
};

static const struct attribute_group *cs_sysfs_groups[] = {
	&cs_sysfs_group,
	NULL,
};

static void exynos_regmap_lock(void *data)
__acquires(&regmap_lock)
{
	struct pmu_lock_data *plock = data;
	unsigned long flags;

	raw_spin_lock_irqsave(&plock->lock, flags);
	plock->irqflags = flags;
}

static void exynos_regmap_unlock(void *data)
__releases(&regmap_lock)
{
	struct pmu_lock_data *plock = data;

	raw_spin_unlock_irqrestore(&plock->lock, plock->irqflags);
}

static int exynos_pmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *syscon_np;
	struct pmu_lock_data *plock;
	struct resource *res;

	plock = kmalloc(sizeof(*plock), GFP_KERNEL);
	if (!plock)
		return -ENOMEM;

	raw_spin_lock_init(&plock->lock);
	syscon_np = of_parse_phandle(dev->of_node, "samsung,syscon-phandle", 0);
	pmureg = __syscon_node_to_regmap(syscon_np, exynos_regmap_lock,
					 exynos_regmap_unlock, plock);
	if (IS_ERR(pmureg)) {
		pr_err("Fail to get regmap of PMU\n");
		kfree(plock);
		return PTR_ERR(pmureg);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmu_alive");
	pmu_alive_pa = res->start;
	spin_lock_init(&update_lock);

	if (subsys_system_register(&exynos_info_subsys,
					cs_sysfs_groups))
		pr_err("Fail to register exynos_info subsys\n");

	dev_info(dev, "exynos_pmu_if probe\n");
	return 0;
}

static const struct of_device_id of_exynos_pmu_match[] = {
	{ .compatible = "samsung,exynos-pmu", },
	{ },
};
MODULE_DEVICE_TABLE(of, of_exynos_pmu_match);

static const struct platform_device_id exynos_pmu_ids[] = {
	{ "exynos-pmu", },
	{ }
};

static struct platform_driver exynos_pmu_if_driver = {
	.driver = {
		.name = "exynos-pmu-if",
		.of_match_table = of_exynos_pmu_match,
	},
	.probe		= exynos_pmu_probe,
	.id_table	= exynos_pmu_ids,
};

static int exynos_pmu_if_init(void)
{
	return platform_driver_register(&exynos_pmu_if_driver);
}
postcore_initcall_sync(exynos_pmu_if_init);

static void exynos_pmu_if_exit(void)
{
	return platform_driver_unregister(&exynos_pmu_if_driver);
}
module_exit(exynos_pmu_if_exit);

MODULE_LICENSE("GPL");
