// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 - Google LLC
 * Author: David Brazdil <dbrazdil@google.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>

#include <linux/kvm_host.h>
#include <asm/kvm_s2mpu.h>

#include <soc/google/pkvm-s2mpu.h>

#define S2MPU_NR_WAYS	4

struct s2mpu_data {
	struct device *dev;
	void __iomem *base;
	bool pkvm_registered;
	bool always_on;
};

struct s2mpu_mptc_entry {
	bool valid;
	u32 vid;
	u32 ppn;
	u32 others;
	u32 data;
};

static const struct of_device_id sysmmu_sync_of_match[];

static int nr_devs_total;
static atomic_t nr_devs_registered = ATOMIC_INIT(0);

static struct platform_device *__of_get_phandle_pdev(struct device *parent,
						     const char *prop, int index)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_parse_phandle(parent->of_node, prop, index);
	if (!np)
		return NULL;

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EINVAL);

	return pdev;
}

int pkvm_s2mpu_of_link(struct device *parent)
{
	struct platform_device *pdev;
	struct device_link *link;
	int i;

	/* Check that all S2MPUs have been initialized. */
	for (i = 0; (pdev = __of_get_phandle_pdev(parent, "s2mpus", i)); i++) {
		if (IS_ERR(pdev))
			return PTR_ERR(pdev);

		if (!pkvm_s2mpu_ready(&pdev->dev))
			return -EAGAIN;
	}

	/* Link all S2MPUs as suppliers to the parent. */
	for (i = 0; (pdev = __of_get_phandle_pdev(parent, "s2mpus", i)); i++) {
		if (IS_ERR(pdev))
			return PTR_ERR(pdev);

		link = device_link_add(/*consumer=*/parent, /*supplier=*/&pdev->dev,
				       DL_FLAG_AUTOREMOVE_CONSUMER | DL_FLAG_PM_RUNTIME);
		if (!link)
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pkvm_s2mpu_of_link);

struct device *pkvm_s2mpu_of_parse(struct device *parent)
{
	struct platform_device *pdev;

	pdev = __of_get_phandle_pdev(parent, "s2mpu", 0);
	if (IS_ERR_OR_NULL(pdev))
		return ERR_PTR(PTR_ERR(pdev));

	return &pdev->dev;
}
EXPORT_SYMBOL_GPL(pkvm_s2mpu_of_parse);

static const char *str_fault_direction(u32 fault_info)
{
	return (fault_info & FAULT_INFO_RW_BIT) ? "write" : "read";
}

static const char *str_fault_type(u32 fault_info)
{
	switch (FIELD_GET(FAULT_INFO_TYPE_MASK, fault_info)) {
	case FAULT_INFO_TYPE_MPTW:
		return "MPTW fault";
	case FAULT_INFO_TYPE_AP:
		return "access permission fault";
	case FAULT_INFO_TYPE_CONTEXT:
		return "context fault";
	default:
		return "unknown fault";
	}
}

static const char *str_l1entry_gran(u32 l1attr)
{
	if (!(l1attr & L1ENTRY_ATTR_L2TABLE_EN))
		return "1G";

	switch (FIELD_GET(L1ENTRY_ATTR_GRAN_MASK, l1attr)) {
	case L1ENTRY_ATTR_GRAN_4K:
		return "4K";
	case L1ENTRY_ATTR_GRAN_64K:
		return "64K";
	case L1ENTRY_ATTR_GRAN_2M:
		return "2M";
	default:
		return "invalid";
	}
}

static const char *str_l1entry_prot(u32 l1attr)
{
	if (l1attr & L1ENTRY_ATTR_L2TABLE_EN)
		return "??";

	switch (FIELD_GET(L1ENTRY_ATTR_PROT_MASK, l1attr)) {
	case MPT_PROT_NONE:
		return "0";
	case MPT_PROT_R:
		return "R";
	case MPT_PROT_W:
		return "W";
	case MPT_PROT_RW:
		return "RW";
	default:
		return "invalid";
	}
}

static struct s2mpu_mptc_entry read_mptc(void __iomem *base, u32 set, u32 way)
{
	struct s2mpu_mptc_entry entry;

	writel_relaxed(READ_MPTC(set, way), base + REG_NS_READ_MPTC);

	entry.ppn = readl_relaxed(base + REG_NS_READ_MPTC_TAG_PPN),
	entry.others = readl_relaxed(base + REG_NS_READ_MPTC_TAG_OTHERS),
	entry.data = readl_relaxed(base + REG_NS_READ_MPTC_DATA),

	entry.valid = FIELD_GET(READ_MPTC_TAG_OTHERS_VALID_BIT, entry.others);
	entry.vid = FIELD_GET(READ_MPTC_TAG_OTHERS_VID_MASK, entry.others);
	return entry;
}

static irqreturn_t s2mpu_irq_handler(int irq, void *ptr)
{
	struct s2mpu_data *data = ptr;
	struct device *dev = data->dev;
	unsigned int vid, gb;
	u32 vid_bmap, fault_info, fmpt, smpt, nr_sets, set, way, invalid;
	phys_addr_t fault_pa;
	struct s2mpu_mptc_entry mptc;
	irqreturn_t ret = IRQ_NONE;

	while ((vid_bmap = readl_relaxed(data->base + REG_NS_FAULT_STATUS))) {
		WARN_ON_ONCE(vid_bmap & (~ALL_VIDS_BITMAP));
		vid = __ffs(vid_bmap);

		fault_pa = hi_lo_readq_relaxed(data->base + REG_NS_FAULT_PA_HIGH_LOW(vid));
		fault_info = readl_relaxed(data->base + REG_NS_FAULT_INFO(vid));
		WARN_ON(FIELD_GET(FAULT_INFO_VID_MASK, fault_info) != vid);

		dev_err(dev, "============== S2MPU FAULT DETECTED ==============\n");
		dev_err(dev, "  PA=%pap, FAULT_INFO=0x%08x\n",
			&fault_pa, fault_info);
		dev_err(dev, "  DIRECTION: %s, TYPE: %s\n",
			str_fault_direction(fault_info),
			str_fault_type(fault_info));
		dev_err(dev, "  VID=%u, REQ_LENGTH=%lu, REQ_AXI_ID=%lu\n",
			vid,
			FIELD_GET(FAULT_INFO_LEN_MASK, fault_info),
			FIELD_GET(FAULT_INFO_ID_MASK, fault_info));

		for_each_gb(gb) {
			fmpt = readl_relaxed(data->base + REG_NS_L1ENTRY_ATTR(vid, gb));
			smpt = readl_relaxed(data->base + REG_NS_L1ENTRY_L2TABLE_ADDR(vid, gb));
			dev_err(dev, "  %uG: FMPT=%#x (%s, %s), SMPT=%#x\n",
				gb, fmpt, str_l1entry_gran(fmpt),
				str_l1entry_prot(fmpt), smpt);
		}

		dev_err(dev, "==================================================\n");

		writel_relaxed(BIT(vid), data->base + REG_NS_INTERRUPT_CLEAR);
		ret = IRQ_HANDLED;
	}

	dev_err(dev, "================== MPTC ENTRIES ==================\n");
	nr_sets = FIELD_GET(INFO_NUM_SET_MASK, readl_relaxed(data->base + REG_NS_INFO));
	for (invalid = 0, set = 0; set < nr_sets; set++) {
		for (way = 0; way < S2MPU_NR_WAYS; way++) {
			mptc = read_mptc(data->base, set, way);
			if (!mptc.valid) {
				invalid++;
				continue;
			}

			dev_err(dev,
				"  MPTC[set=%u, way=%u]={VID=%u, PPN=%#x, OTHERS=%#x, DATA=%#x}\n",
				set, way, mptc.vid, mptc.ppn, mptc.others, mptc.data);
		}
	}
	dev_err(dev, "  invalid entries: %u\n", invalid);
	dev_err(dev, "==================================================\n");

	return ret;
}

/*
 * Parse interrupt information from DT and if found, register IRQ handler.
 * This is considered optional and will not fail even if the initialization is
 * unsuccessful. In that case the IRQ will remain masked.
 */
static void s2mpu_probe_irq(struct platform_device *pdev, struct s2mpu_data *data)
{
	int ret, irq;

	irq = platform_get_irq_optional(pdev, 0);

	if (irq == -ENXIO)
		return; /* No IRQ specified. */

	if (irq < 0) {
		/* IRQ specified but failed to parse. */
		dev_err(data->dev, "failed to parse IRQ, IRQ not enabled");
		return;
	}

	ret = devm_request_irq(data->dev, irq, s2mpu_irq_handler, 0,
			       dev_name(data->dev), data);
	if (ret) {
		dev_err(&pdev->dev, "failed to register IRQ, IRQ not enabled");
		return;
	}
}

static struct s2mpu_data *s2mpu_dev_data(struct device *dev)
{
	return platform_get_drvdata(to_platform_device(dev));
}

int pkvm_s2mpu_suspend(struct device *dev)
{
	struct s2mpu_data *data = s2mpu_dev_data(dev);

	if (data->pkvm_registered && !data->always_on)
		return pkvm_iommu_suspend(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(pkvm_s2mpu_suspend);

int pkvm_s2mpu_resume(struct device *dev)
{
	struct s2mpu_data *data = s2mpu_dev_data(dev);

	if (data->pkvm_registered)
		return pkvm_iommu_resume(dev);

	writel_relaxed(0, data->base + REG_NS_CTRL0);
	return 0;
}
EXPORT_SYMBOL_GPL(pkvm_s2mpu_resume);

static int s2mpu_late_suspend(struct device *dev)
{
	struct s2mpu_data *data = s2mpu_dev_data(dev);

	/*
	 * Some always-on S2MPUs need to allow traffic while the CPU is asleep.
	 * Do not call pkvm_iommu_suspend() here because that would put them
	 * in a blocking state.
	 */
	if (data->always_on || pm_runtime_status_suspended(dev))
		return 0;

	dev->power.must_resume = true;
	return pkvm_s2mpu_suspend(dev);
}

static int s2mpu_late_resume(struct device *dev)
{
	/*
	 * Some always-on S2MPUs reset while the CPU is asleep. Call
	 * pkvm_iommu_resume() here regardless of always-on to reconfigure them.
	 */

	if (pm_runtime_status_suspended(dev))
		return 0;

	return pkvm_s2mpu_resume(dev);
}

static int sysmmu_sync_probe(struct device *parent)
{
	struct platform_device *pdev;
	struct resource *res;
	int i, ret;

	for (i = 0; (pdev = __of_get_phandle_pdev(parent, "sysmmu_syncs", i)); i++) {
		if (IS_ERR(pdev))
			return PTR_ERR(pdev);

		if (!of_match_device(sysmmu_sync_of_match, &pdev->dev)) {
			dev_err(parent, "%s is not sysmmu_sync compatible",
				dev_name(&pdev->dev));
			return -EINVAL;
		}

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&pdev->dev, "failed to parse 'reg'");
			return -EINVAL;
		}

		if (!devm_request_mem_region(&pdev->dev, res->start,
					     resource_size(res),
					     dev_name(&pdev->dev))) {
			dev_err(&pdev->dev, "failed to request mmio region");
			return -EINVAL;
		}

		ret = pkvm_iommu_sysmmu_sync_register(&pdev->dev, res->start,
						      parent);
		if (ret) {
			dev_err(&pdev->dev, "could not register: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int s2mpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct s2mpu_data *data;
	bool off_at_boot, has_pd;
	int ret, nr_devs;

	data = devm_kmalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to parse 'reg'");
		return -EINVAL;
	}

	/* devm_ioremap_resource internally calls devm_request_mem_region. */
	data->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->base)) {
		dev_err(dev, "could not ioremap resource: %ld", PTR_ERR(data->base));
		return PTR_ERR(data->base);
	}

	data->always_on = !!of_get_property(np, "always-on", NULL);
	off_at_boot = !!of_get_property(np, "off-at-boot", NULL);
	has_pd = !!of_get_property(np, "power-domains", NULL);

	/*
	 * Try to parse IRQ information. This is optional as it only affects
	 * runtime fault reporting, and therefore errors do not fail the whole
	 * driver initialization.
	 */
	s2mpu_probe_irq(pdev, data);

	ret = pkvm_iommu_s2mpu_register(dev, res->start);
	if (ret && ret != -ENODEV) {
		dev_err(dev, "could not register: %d\n", ret);
		return ret;
	}

	data->pkvm_registered = ret != -ENODEV;

	if (data->pkvm_registered) {
		ret = sysmmu_sync_probe(dev);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, data);
	nr_devs = atomic_inc_return(&nr_devs_registered);

	if (data->pkvm_registered)
		dev_info(dev, "registered with hypervisor [%d/%d]\n", nr_devs, nr_devs_total);
	else
		dev_warn(dev, "hypervisor disabled, control from kernel\n");

	if (data->pkvm_registered && nr_devs == nr_devs_total) {
		ret = pkvm_iommu_finalize();
		if (!ret)
			pr_info("list of devices successfully finalized\n");
		else
			pr_err("could not finalize: %d\n", ret);
	}

	/*
	 * Most S2MPUs are in an allow-all state at boot. Call the hypervisor
	 * to initialize the S2MPU to a blocking state. This corresponds to
	 * the state the hypervisor sets on suspend.
	 */
	if (!off_at_boot)
		WARN_ON(pkvm_s2mpu_suspend(dev));

	if (has_pd || data->always_on)
		pm_runtime_enable(dev);
	if (data->always_on)
		pm_runtime_get_sync(dev);

	return 0;
}

static const struct dev_pm_ops s2mpu_pm_ops = {
	SET_RUNTIME_PM_OPS(pkvm_s2mpu_suspend, pkvm_s2mpu_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(s2mpu_late_suspend, s2mpu_late_resume)
};

static const struct of_device_id sysmmu_sync_of_match[] = {
	{ .compatible = "google,sysmmu_sync" },
	{},
};

static const struct of_device_id s2mpu_of_match[] = {
	{ .compatible = "google,s2mpu" },
	{},
};

static struct platform_driver s2mpu_driver = {
	.probe = s2mpu_probe,
	.driver = {
		.name = "pkvm-s2mpu",
		.of_match_table = s2mpu_of_match,
		.pm = &s2mpu_pm_ops,
	},
};

static int s2mpu_driver_register(struct platform_driver *driver)
{
	struct device_node *np;

	for_each_matching_node(np, driver->driver.of_match_table)
		if (of_device_is_available(np))
			nr_devs_total++;
	pr_info("%d devices to be initialized\n", nr_devs_total);

	return platform_driver_register(driver);
}

module_driver(s2mpu_driver, s2mpu_driver_register, platform_driver_unregister);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Brazdil <dbrazdil@google.com>");
