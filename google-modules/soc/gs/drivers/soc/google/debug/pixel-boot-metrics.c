// SPDX-License-Identifier: GPL-2.0
/*
 * boot_metrics.c - Get boot metrics information
 *
 * Copyright 2021 Google LLC
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/sysfs.h>

#include "pixel-boot-metrics.h"

#define METRICS_ATTR_RO(_name, _phase1, _type1, _phase2, _type2)	\
	static ssize_t _name##_show(struct kobject *kobj,	\
			struct kobj_attribute *attr, char *buf)	\
	{	\
		u32 timestamp1 =	\
		boot_metrics_get(METRICS_PHASE_##_phase1, METRICS_##_phase1##_##_type1);	\
		u32 timestamp2 =	\
		boot_metrics_get(METRICS_PHASE_##_phase2, METRICS_##_phase2##_##_type2);	\
		\
		u32 interval = boot_metrics_calculate_interval(timestamp1, timestamp2);		\
		\
		return sysfs_emit(buf, "%u\n", interval);	\
	}	\
	static struct kobj_attribute metrics_attr_##_name = __ATTR_RO(_name)

/*
 * Metrics structures
 */
static struct metrics_header_t *metrics_header_info;

/*
 * Sysfs structures
 */
static struct kobject *boot_metrics_kobj;


static u32 boot_metrics_calculate_interval(u32 timestamp_before, u32 timestamp_after)
{
	/*
	 * The timestamps overflow at `METRICS_DATA_OVERFLOW`.
	 * They are not full-range 32-bit integers.
	 */

	/* To make sure it is larger than `timestamp_before`. */
	timestamp_after += METRICS_DATA_OVERFLOW;

	return (timestamp_after - timestamp_before) % METRICS_DATA_OVERFLOW;
}

/*
 * Get metrics header.
 */
static struct metrics_header_t *boot_metrics_get_metrics_header(void)
{
	return metrics_header_info;
}

/*
 * Get metrics.
 */
static u32 *boot_metrics_get_metrics(void)
{
	struct metrics_header_t *metrics_header = boot_metrics_get_metrics_header();

	return (u32 *)(uintptr_t)metrics_header->metrics;
}

/*
 * Get metrics maximum
 */
static u32 boot_metrics_get_maximum(void)
{
	return boot_metrics_get_metrics_header()->maximum;
}

/*
 * Get metrics by index
 */
static u32 boot_metrics_get_index_metrics(u32 index)
{
	if (index >= boot_metrics_get_maximum())
		return 0;

	return boot_metrics_get_metrics()[index];
}

/*
 * Get metrics phase type.
 */
static u32 metrics_phase_type_get(u32 index)
{
	return ((boot_metrics_get_index_metrics(index) >> METRICS_PHASE_TYPE_SHIFT) &
			METRICS_PHASE_TYPE_MASK);
}

/*
 * Get metrics type.
 */
static u32 metrics_type_get(u32 index)
{
	return ((boot_metrics_get_index_metrics(index) >> METRICS_TYPE_SHIFT) & METRICS_TYPE_MASK);
}

/*
 * Get metrics data by index
 */
static u32 metrics_data_get(u32 index)
{
	return (((boot_metrics_get_index_metrics(index) >> METRICS_DATA_SHIFT) & METRICS_DATA_MASK)
		<< METRICS_DATA_ALIGN);
}

/*
 * Get metrics data by phase and type.
 */
static u32 boot_metrics_get(u32 phase_type, u32 metrics_type)
{
	u32 index;

	if (!boot_metrics_get_metrics_header())
		return 0;

	for (index = 0; index < boot_metrics_get_maximum(); index++) {
		if (boot_metrics_get_index_metrics(index) == 0)
			break;

		if ((metrics_phase_type_get(index) == phase_type) &&
		    (metrics_type_get(index) == metrics_type)) {
			return metrics_data_get(index);
		}
	}

	return 0;
}

/*
 * Suspend sysfs
 */
/* el3pss: psci_system_suspend time */
METRICS_ATTR_RO(el3pss, EL3, PSCI_SYSTEM_SUSPEND_START, EL3, MON_SMC_SLEEP_START);
/* el3mss: mon_smc_sleep time */
METRICS_ATTR_RO(el3mss, EL3, MON_SMC_SLEEP_START, EL3, MON_SMC_SLEEP_END);
/* total: psci_system_suspend + mon_smc_sleep time */
METRICS_ATTR_RO(suspend_total, EL3, PSCI_SYSTEM_SUSPEND_START, EL3, MON_SMC_SLEEP_END);

static struct attribute *suspend_attrs[] = {
	&metrics_attr_el3pss.attr,
	&metrics_attr_el3mss.attr,
	&metrics_attr_suspend_total.attr,
	NULL
};

static const struct attribute_group suspend_attr_group = {
	.attrs = suspend_attrs,
	.name = "suspend"
};

/*
 * Resume sysfs
 */
/* bl1sg: bl1_sleep_go time */
METRICS_ATTR_RO(bl1sg, BL1, SLEEP_GO_START, PBL, SLEEP_GO_START);
#if defined(CONFIG_SOC_GS101)
/* pblsg: pbl_sleep_go time */
METRICS_ATTR_RO(pblsg, PBL, SLEEP_GO_START, EL3, SLEEP_GO_START);
/* el3sg: el3_sleep_go time */
METRICS_ATTR_RO(el3sg, EL3, SLEEP_GO_START, BL2, WARMBOOT_START);
/* bl2wb: bl2_warmboot time */
METRICS_ATTR_RO(bl2wb, BL2, WARMBOOT_START, EL3, MON_SMC_WARMBOOT_START);
/* el3wb: el3_warmboot time */
METRICS_ATTR_RO(el3wb, EL3, MON_SMC_WARMBOOT_START, EL3, MON_SMC_WARMBOOT_END);
/* total: bl1_sleep_go + pbl_sleep_go + el3_sleep_go + bl2_warmboot + el3_warmboot time */
METRICS_ATTR_RO(resume_total, BL1, SLEEP_GO_START, EL3, MON_SMC_WARMBOOT_END);
#else /* CONFIG_SOC_GS201 or Zuma */
/* pblsg: pbl_sleep_go time */
METRICS_ATTR_RO(pblsg, PBL, SLEEP_GO_START, EL3, MON_SMC_WARMBOOT_START);
/* el3wb: el3_warmboot time */
METRICS_ATTR_RO(el3wb, EL3, MON_SMC_WARMBOOT_START, EL3, MON_SMC_WARMBOOT_END);
/* total: bl1_sleep_go + pbl_sleep_go + el3_warmboot time */
METRICS_ATTR_RO(resume_total, BL1, SLEEP_GO_START, EL3, MON_SMC_WARMBOOT_END);
#endif

static struct attribute *resume_attrs[] = {
	&metrics_attr_bl1sg.attr,
	&metrics_attr_pblsg.attr,
#if defined(CONFIG_SOC_GS101)
	&metrics_attr_el3sg.attr,
	&metrics_attr_bl2wb.attr,
#endif
	&metrics_attr_el3wb.attr,
	&metrics_attr_resume_total.attr,
	NULL
};

static const struct attribute_group resume_attr_group = {
	.attrs = resume_attrs,
	.name = "resume"
};

static int boot_metrics_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	void __iomem *apc_sram_virt_mapping;
	u32 offset;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ns_sram_base");

	if (!res) {
		dev_err(dev, "failed to get memory resources for device sram\n");
		return 0;
	}

	apc_sram_virt_mapping = devm_ioremap_resource(dev, res);
	if (IS_ERR(apc_sram_virt_mapping)) {
		dev_err(dev, "failed to get apc sram virtual mapping %ld\n",
			PTR_ERR(apc_sram_virt_mapping));
		return 0;
	}

	if (of_property_read_u32(np, "offset", &offset)) {
		dev_err(&pdev->dev, "offset property value is missing\n");
		return 0;
	}

	if (offset + sizeof(struct metrics_header_t) > resource_size(res)) {
		dev_err(&pdev->dev, "unexpected offset %x or resource %pr properties\n",
			offset, res);
		return 0;
	}

	boot_metrics_kobj = kobject_create_and_add("pixel_boot_metrics", kernel_kobj);
	if (!boot_metrics_kobj) {
		dev_err(&pdev->dev, "cannot create kobj for pixel_boot_metrics!\n");
		return 0;
	}

	if (sysfs_create_group(boot_metrics_kobj, &suspend_attr_group)) {
		dev_err(&pdev->dev, "cannot create files in ../pixel_boot_metrics/suspend!\n");
		goto put_metrics_kobj;
	}

	if (sysfs_create_group(boot_metrics_kobj, &resume_attr_group)) {
		dev_err(&pdev->dev, "cannot create files in ../pixel_boot_metrics/resume!\n");
		goto remove_suspend_sysfs;
	}

	metrics_header_info = (struct metrics_header_t *)(apc_sram_virt_mapping + offset);

	return 0;

remove_suspend_sysfs:
	sysfs_remove_group(boot_metrics_kobj, &suspend_attr_group);
put_metrics_kobj:
	kobject_put(boot_metrics_kobj);

	return 0;
}

static const struct of_device_id boot_metrics_of_match[] = {
	{ .compatible	= "google,boot-metrics" },
	{},
};
MODULE_DEVICE_TABLE(of, boot_metrics_of_match);

static struct platform_driver boot_metrics_driver = {
	.probe = boot_metrics_probe,
	.driver = {
		.name = "boot-metrics",
		.of_match_table = of_match_ptr(boot_metrics_of_match),
	},
};
module_platform_driver(boot_metrics_driver);

MODULE_AUTHOR("Jone Chou <jonechou@google.com>");
MODULE_DESCRIPTION("Boot Metrics Driver");
MODULE_LICENSE("GPL v2");
