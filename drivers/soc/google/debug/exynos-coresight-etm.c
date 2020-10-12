// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <asm/barrier.h>
#include <soc/google/debug-snapshot.h>

#include "core_regs.h"

#define CHANNEL		(0)
#define PORT		(1)
#define NONE		(-1)
#define ARR_SZ		(2)

#define etm_writel(base, val, off)	__raw_writel((val), (base) + (off))
#define etm_readl(base, off)		__raw_readl((base) + (off))

static inline void soft_lock(void __iomem *base)
{
	dsb(sy);
	isb();
	etm_writel(base, 0x0, LAR);
}

static inline void soft_unlock(void __iomem *base)
{
	etm_writel(base, OSLOCK_MAGIC, LAR);
	dsb(sy);
	isb();
}

struct etm_info {
	void __iomem	*base;
	u32		enabled;
	u32		f_port[ARR_SZ];
};

struct funnel_info {
	void __iomem	*base;
	u32		port_status;
};
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
struct etf_info {
	void __iomem	*base;
	u32		enabled;
	u32		f_port[ARR_SZ];
};
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
struct etr_info {
	void __iomem	*base;
	void __iomem	*sfr_base;
	u32		enabled;
	dma_addr_t	buf_addr;
	void		*buf_vaddr;
	u64		buf_pointer;
	u32		qch_offset;
	bool		hwacg;
};
#endif
struct exynos_etm_info {
	struct etm_info		*cpu;
	spinlock_t		trace_lock;
	struct device		*dev;
	u32			on;
	u32			boot_start;
	u32			etr_buf_size;

	u32			funnel_num;
	struct funnel_info	*funnel;
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	u32			etf_num;
	struct etf_info		*etf;
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	struct etr_info		etr;
#endif
};

static struct exynos_etm_info *ee_info;

static void exynos_etm_set_funnel_port(struct funnel_info *funnel,
				      int port, bool enabled)
{
	soft_unlock(funnel->base);
	funnel->port_status = etm_readl(funnel->base, FUNCTRL);
	if (enabled)
		funnel->port_status |= BIT(port);
	else
		funnel->port_status &= ~BIT(port);
	etm_writel(funnel->base, funnel->port_status, FUNCTRL);
	soft_lock(funnel->base);
}
static void exynos_etm_funnel_init(void)
{
	unsigned int i;
	struct funnel_info *funnel;

	for (i = 0; i < ee_info->funnel_num; i++) {
		funnel = &ee_info->funnel[i];
		spin_lock(&ee_info->trace_lock);
		soft_unlock(funnel->base);
		funnel->port_status = etm_readl(funnel->base, FUNCTRL);
		funnel->port_status = (funnel->port_status & 0x3ff) | 0x300;
		etm_writel(funnel->base, funnel->port_status, FUNCTRL);
		etm_writel(funnel->base, 0x0, FUNPRIORCTRL);
		soft_lock(funnel->base);
		spin_unlock(&ee_info->trace_lock);
	}
}

static void exynos_etm_funnel_close(void)
{
	unsigned int i;
	struct funnel_info *funnel;

	for (i = 0; i < ee_info->funnel_num; i++) {
		funnel = &ee_info->funnel[i];
		spin_lock(&ee_info->trace_lock);
		soft_unlock(funnel->base);
		etm_writel(funnel->base, 0x300, FUNCTRL);
		soft_lock(funnel->base);
		spin_unlock(&ee_info->trace_lock);
	}
}

#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
static void exynos_etm_etf_enable(void)
{
	unsigned int i, port, channel;
	struct etf_info *etf;
	struct funnel_info *funnel;

	for (i = 0; i < ee_info->etf_num; i++) {
		etf = &ee_info->etf[i];
		soft_unlock(etf->base);
		etm_writel(etf->base, 0x0, TMCCTL);
		etm_writel(etf->base, 0x800, TMCRSZ);
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
		etm_writel(etf->base, 0x2, TMCMODE);
#else
		etm_writel(etf->base, 0x0, TMCMODE);
#endif
		etm_writel(etf->base, 0x0, TMCTGR);
		etm_writel(etf->base, 0x0, TMCFFCR);
		etm_writel(etf->base, 0x1, TMCCTL);
		soft_lock(etf->base);

		if (etf->f_port[CHANNEL] == NONE)
			continue;

		channel = etf->f_port[CHANNEL];
		port = etf->f_port[PORT];
		funnel = &ee_info->funnel[channel];

		spin_lock(&ee_info->trace_lock);
		exynos_etm_set_funnel_port(funnel, port, true);
		spin_unlock(&ee_info->trace_lock);
	}
}

static void exynos_etm_etf_disable(void)
{
	unsigned int i, port, channel;
	struct etf_info *etf;
	struct funnel_info *funnel;

	for (i = 0; i < ee_info->etf_num; i++) {
		etf = &ee_info->etf[i];
		soft_unlock(etf->base);
		etm_writel(etf->base, 0x0, TMCCTL);
		soft_lock(etf->base);

		if (etf->f_port[CHANNEL] == NONE)
			continue;

		channel = etf->f_port[CHANNEL];
		port = etf->f_port[PORT];
		funnel = &ee_info->funnel[channel];

		spin_lock(&ee_info->trace_lock);
		exynos_etm_set_funnel_port(funnel, port, false);
		spin_unlock(&ee_info->trace_lock);
	}
}
#endif

#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
static void exynos_etm_etr_enable(void)
{
	struct etr_info *etr = &ee_info->etr;

	if (etr->hwacg)
		writel_relaxed(0x1, etr->sfr_base + etr->qch_offset);

	soft_unlock(etr->base);
	etm_writel(etr->base, 0x0, TMCCTL);
	etm_writel(etr->base, ee_info->etr_buf_size / 4, TMCRSZ);
	etm_writel(etr->base, 0x4, TMCTGR);
	etm_writel(etr->base, 0x0, TMCAXICTL);
	etm_writel(etr->base, etr->buf_addr & 0xffffffff, TMCDBALO);
	etm_writel(etr->base, etr->buf_addr >> 32, TMCDBALO);
	etm_writel(etr->base, 0x0, TMCDBAHI);
	etm_writel(etr->base, etr->buf_pointer, TMCRWP);
	etm_writel(etr->base, 0x0, TMCMODE);
	etm_writel(etr->base, 0x2001, TMCFFCR);
	etm_writel(etr->base, 0x1, TMCCTL);
	soft_lock(etr->base);
}

static void exynos_etm_etr_disable(void)
{
	struct etr_info *etr = &ee_info->etr;

	soft_unlock(etr->base);
	etm_writel(etr->base, 0x0, TMCCTL);
	etr->buf_pointer = etm_readl(etr->base, TMCRWP);
	soft_lock(etr->base);

	if (etr->hwacg)
		writel_relaxed(0x0, etr->sfr_base + etr->qch_offset);
}
#endif

static int exynos_etm_enable(unsigned int cpu)
{
	struct etm_info *info = &ee_info->cpu[cpu];
	struct funnel_info *funnel;
	unsigned int channel, port, val;

	if (!ee_info->on || info->enabled)
		return 0;

	soft_unlock(info->base);
	etm_writel(info->base, OSLOCK, ETMOSLAR);
	etm_writel(info->base, !ETM_EN, ETMCTLR);

	/* Main control and Configuration */
	etm_writel(info->base, TIMESTAMP, ETMCONFIG);
	etm_writel(info->base, PERIOD(8), ETMSYNCPR);

	val = etm_readl(info->base, ETMID3);
	val &= 0xfff;
	etm_writel(info->base, val, ETMCCCCTLR);
	etm_writel(info->base, cpu + 1, ETMTRACEIDR);

	etm_writel(info->base, 0x0, ETMEVENTCTL0R);
	etm_writel(info->base, 0x0, ETMEVENTCTL1R);

	etm_writel(info->base, ETM_EN, ETMCTLR);
	etm_writel(info->base, !OSLOCK, ETMOSLAR);

	channel = info->f_port[CHANNEL];
	port = info->f_port[PORT];
	funnel = &ee_info->funnel[channel];

	spin_lock(&ee_info->trace_lock);
	info->enabled = 1;
	exynos_etm_set_funnel_port(funnel, port, true);
	spin_unlock(&ee_info->trace_lock);

	return 0;
}

static int exynos_etm_disable(unsigned int cpu)
{
	struct etm_info *info = &ee_info->cpu[cpu];
	struct funnel_info *funnel;
	unsigned int channel, port;

	if (!ee_info->on || !info->enabled)
		return 0;

	channel = info->f_port[CHANNEL];
	port = info->f_port[PORT];
	funnel = &ee_info->funnel[channel];

	spin_lock(&ee_info->trace_lock);
	info->enabled = 0;
	exynos_etm_set_funnel_port(funnel, port, false);
	spin_unlock(&ee_info->trace_lock);

	soft_unlock(info->base);
	etm_writel(info->base, !ETM_EN, ETMCTLR);
	etm_writel(info->base, OSLOCK, ETMOSLAR);

	return 0;
}

static void exynos_etm_smp_enable(void *onoff)
{
	bool on = *(bool *)onoff;
	unsigned int cpu = raw_smp_processor_id();

	if (on)
		exynos_etm_enable(cpu);
	else
		exynos_etm_disable(cpu);
}

static ssize_t exynos_etm_print_info(char *buf)
{
	struct etm_info *info;
	struct funnel_info *funnel;
	struct etf_info *etf;
	unsigned long tmp, port_status, read_p;
	int i, channel, port, size = 0;

	size += scnprintf(buf + size, PAGE_SIZE - size,
			"\n---------------------------------------\n");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			" %-4s | %-10s | %-6s | %-4s | %-6s\n",
			"Core", "ETM status", "Funnel", "Port", "Status");

	for (i = 0; i < num_possible_cpus(); i++) {
		info = &ee_info->cpu[i];
		channel = info->f_port[CHANNEL];
		port = info->f_port[PORT];
		funnel = &ee_info->funnel[channel];
		spin_lock(&ee_info->trace_lock);
		port_status = (funnel->port_status >> port) & 0x1;
		spin_unlock(&ee_info->trace_lock);
		size += scnprintf(buf + size, PAGE_SIZE - size,
				" %-4d | %5sabled | %-6u | %-4d | %-6s\n",
				i, info->enabled ? "en" : "dis",
				channel, port, port_status ? "on" : "off");
	}
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"---------------------------------------\n");

	for (i = 0; i < ee_info->funnel_num; i++) {
		funnel = &ee_info->funnel[i];
		soft_unlock(funnel->base);
		tmp = etm_readl(funnel->base, FUNCTRL);
		soft_lock(funnel->base);
		size += scnprintf(buf + size, PAGE_SIZE - size,
				"FUNNEL%d Status : 0x%lx\n", i, tmp);
	}
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	for (i = 0; i < ee_info->etf_num; i++) {
		etf = &ee_info->etf[i];
		soft_unlock(etf->base);
		tmp = etm_readl(etf->base, TMCCTL);
		read_p = etm_readl(etf->base, TMCRWP);
		soft_lock(etf->base);
		size += scnprintf(buf + size, PAGE_SIZE - size,
				"ETF%d Status: %sabled, RWP Reg: 0x%px\n",
				i, tmp & 0x1 ? "en" : "dis", read_p);
	}
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	soft_unlock(ee_info->etr.base);
	tmp = etm_readl(ee_info->etr.base, TMCCTL);
	read_p = etm_readl(ee_info->etr.base, TMCRWP);
	soft_lock(ee_info->etr.base);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"ETR Status: %sabled, RWP Reg: 0x%px, Save RWP: 0x%px\n",
			tmp & 0x1 ? "en" : "dis", read_p, ee_info->etr.buf_pointer);
#endif
	return size;
}

void exynos_etm_trace_start(void)
{
	char *buf;

	buf = devm_kzalloc(ee_info->dev, PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return;

	exynos_etm_funnel_init();
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	exynos_etm_etf_enable();
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	exynos_etm_etr_enable();
#endif
	on_each_cpu(exynos_etm_smp_enable, &ee_info->on, 1);
	exynos_etm_print_info(buf);
	dev_info(ee_info->dev, "%s\n", buf);
	devm_kfree(ee_info->dev, buf);
}

void exynos_etm_trace_stop(void)
{
	char *buf;

	buf = devm_kzalloc(ee_info->dev, PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return;

	on_each_cpu(exynos_etm_smp_enable, &ee_info->on, 1);
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	exynos_etm_etr_disable();
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	exynos_etm_etf_disable();
#endif
	exynos_etm_funnel_close();
	exynos_etm_print_info(buf);
	dev_info(ee_info->dev, "%s\n", buf);
	devm_kfree(ee_info->dev, buf);
}

static int exynos_etm_c2_pm_notifier(struct notifier_block *self,
				     unsigned long action, void *v)
{
	int cpu = raw_smp_processor_id();

	if (!ee_info->on)
		return NOTIFY_OK;

	switch (action) {
	case CPU_PM_ENTER:
		exynos_etm_disable(cpu);
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		exynos_etm_enable(cpu);
		break;
	case CPU_CLUSTER_PM_ENTER:
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block exynos_etm_c2_pm_nb = {
	.notifier_call = exynos_etm_c2_pm_notifier,
};

static int exynos_etm_pm_notifier(struct notifier_block *notifier,
				  unsigned long pm_event, void *v)
{
	if (!ee_info->on)
		return NOTIFY_OK;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		exynos_etm_trace_stop();
		break;
	case PM_POST_SUSPEND:
		exynos_etm_trace_start();
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block exynos_etm_pm_nb = {
	.notifier_call = exynos_etm_pm_notifier,
};

static ssize_t trace_on_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int on;

	if (kstrtoint(buf, 10, &on))
		return count;

	ee_info->on = on;
	if (on)
		exynos_etm_trace_start();
	else
		exynos_etm_trace_stop();

	return count;
}

static ssize_t trace_on_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			ee_info->on ? "on" : "off");
}

static ssize_t etm_info_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return exynos_etm_print_info(buf);
}

DEVICE_ATTR_RO(trace_on);
DEVICE_ATTR_RO(etm_info);

static struct attribute *exynos_etm_sysfs_attrs[] = {
	&dev_attr_trace_on.attr,
	&dev_attr_etm_info.attr,
	NULL,
};

static struct attribute_group exynos_etm_sysfs_group = {
	.attrs = exynos_etm_sysfs_attrs,
	NULL,
};

static const struct attribute_group *exynos_etm_sysfs_groups[] = {
	&exynos_etm_sysfs_group,
	NULL,
};

static int exynos_etm_cs_etm_init_dt(struct device *dev)
{
	struct device_node *np, *etm_np = dev->of_node;
	unsigned int offset, cs_base;
	int i = 0;
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	u32 reg[2];
#endif
	ee_info = devm_kzalloc(dev, sizeof(struct exynos_etm_info),
			GFP_KERNEL);
	if (!ee_info)
		return -ENOMEM;

	ee_info->dev = dev;
	ee_info->cpu = devm_kcalloc(dev, num_possible_cpus(),
			sizeof(struct etm_info), GFP_KERNEL);
	if (!ee_info->cpu)
		return -ENOMEM;

	if (of_property_read_u32(etm_np, "funnel-num", &ee_info->funnel_num))
		return -EINVAL;

	ee_info->funnel = devm_kcalloc(dev, ee_info->funnel_num,
			sizeof(struct funnel_info), GFP_KERNEL);
	if (!ee_info->funnel)
		return -ENOMEM;
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	if (of_property_read_u32(etm_np, "etf-num", &ee_info->etf_num))
		return -EINVAL;

	ee_info->etf = devm_kcalloc(dev, ee_info->etf_num,
			sizeof(struct etf_info), GFP_KERNEL);
	if (!ee_info->etf)
		return -ENOMEM;
#endif
	if (of_property_read_u32(etm_np, "cs_base", &cs_base))
		return -EINVAL;

	np = etm_np;
	while ((np = of_find_node_by_type(np, "etm"))) {
		if (of_property_read_u32(np, "offset", &offset))
			return -EINVAL;

		ee_info->cpu[i].base = devm_ioremap(dev,
				cs_base + offset, SZ_4K);
		if (!ee_info->cpu[i].base)
			return -ENOMEM;

		if (of_property_read_u32_array(np, "funnel-port",
				ee_info->cpu[i].f_port, 2))
			ee_info->cpu[i].f_port[CHANNEL] = NONE;
		i++;
	}
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	i = 0;
	np = etm_np;
	while ((np = of_find_node_by_type(np, "etf"))) {
		if (of_property_read_u32(np, "offset", &offset))
			return -EINVAL;

		ee_info->etf[i].base = devm_ioremap(dev, cs_base + offset,
				SZ_1K);
		if (!ee_info->etf[i].base)
			return -ENOMEM;

		if (of_property_read_u32_array(np, "funnel-port",
				ee_info->etf[i].f_port, 2))
			ee_info->etf[i].f_port[CHANNEL] = NONE;
		i++;
	}
#endif
	i = 0;
	np = etm_np;
	while ((np = of_find_node_by_type(np, "funnel"))) {
		if (of_property_read_u32(np, "offset", &offset))
			return -EINVAL;

		ee_info->funnel[i].base = devm_ioremap(dev, cs_base + offset,
				SZ_256);

		if (!ee_info->funnel[i].base)
			return -ENOMEM;
		i++;
	}
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	np = of_find_node_by_type(etm_np, "etr");
	if (!np)
		return -EINVAL;
	if (of_property_read_u32(np, "offset", &offset))
		return -EINVAL;
	ee_info->etr.base = devm_ioremap(dev, cs_base + offset, SZ_1K);
	if (!ee_info->etr.base)
		return -ENOMEM;

	if (of_property_read_u32(np, "buf-size", &ee_info->etr_buf_size))
		return -EINVAL;

	dma_set_mask_and_coherent(ee_info->dev, DMA_BIT_MASK(36));
	ee_info->etr.buf_vaddr = dma_alloc_coherent(ee_info->dev,
		ee_info->etr_buf_size, &ee_info->etr.buf_addr, GFP_KERNEL);
	if (!ee_info->etr.buf_vaddr || !ee_info->etr.buf_addr)
		return -ENOMEM;

	if (of_property_read_u32(np, "qch-offset", &offset))
		return 0;

	ee_info->etr.qch_offset = offset;

	if (of_property_read_u32_array(np, "sfr_base", reg, 2))
		return -EINVAL;

	ee_info->etr.sfr_base = ioremap(reg[0], reg[1]);
	if (!ee_info->etr.sfr_base)
		return -EINVAL;

	ee_info->etr.hwacg = true;
#endif
	if (of_property_read_u32(etm_np, "boot-start", &ee_info->boot_start))
		return 0;

	return 0;
}

static int exynos_etm_probe(struct platform_device *pdev)
{
	int ret;

	ret = exynos_etm_cs_etm_init_dt(&pdev->dev);
	if (ret < 0)
		return ret;

	spin_lock_init(&ee_info->trace_lock);
	register_pm_notifier(&exynos_etm_pm_nb);
	cpuhp_setup_state_nocalls_cpuslocked(CPUHP_AP_ONLINE_DYN,
			"exynos-etm:online",
			exynos_etm_enable,
			exynos_etm_disable);
	cpu_pm_register_notifier(&exynos_etm_c2_pm_nb);

	if (ee_info->boot_start) {
		ee_info->on = true;
		exynos_etm_trace_start();
	}

	if (sysfs_create_groups(&pdev->dev.kobj, exynos_etm_sysfs_groups)) {
		dev_err(&pdev->dev, "fail to register exynos-etm sysfs\n");
		return -EFAULT;
	}

	dev_info(&pdev->dev, "%s successful\n", __func__);
	return 0;
}

static const struct of_device_id exynos_etm_matches[] = {
	{.compatible = "google,exynos-etm"},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_etm_matches);

static struct platform_driver exynos_etm_driver = {
	.probe		= exynos_etm_probe,
	.driver		= {
		.name	= "exynos-etm",
		.of_match_table	= of_match_ptr(exynos_etm_matches),
	},
};
module_platform_driver(exynos_etm_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos ETM Driver");
