// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019 Google LLC
 *    Author: Hyunki Koo <hyunki00.koo@samsung.com>
 */

#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/bitrev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/module.h>

struct gs_chipid_variant {
	int product_ver;
	int unique_id_reg;
	int rev_reg;
	int main_rev_bit;
	int sub_rev_bit;
};

#define RAW_HEX_STR_SIZE 116
#define AP_HW_TUNE_HEX_STR_SIZE 64
#define AP_HW_TUNE_HEX_ARRAY_SIZE 32

/**
 * Struct gs_chipid_info
 * @soc_product_id: product id allocated to gs SoC
 * @soc_revision: revision of gs SoC
 */
struct gs_chipid_info {
	bool initialized;
	u32 product_id;
	u32 type;
	u32 revision;
	u32 main_rev;
	u32 sub_rev;
	u32 lot_id;
	char *lot_id2;
	u64 unique_id;
	char ap_hw_tune_str[AP_HW_TUNE_HEX_STR_SIZE];
	u8 ap_hw_tune_arr[AP_HW_TUNE_HEX_ARRAY_SIZE];
	char raw_str[RAW_HEX_STR_SIZE];
	struct gs_chipid_variant *drv_data;
	struct platform_device *pdev;
};

#define GS101_SOC_ID		0x09845000
#define SOC_MASK		0xFFFFF000
#define SOC_MASK_V2		0x00FFFFFF
#define SOC_TYPE_MASK		0x0000000F
#define LOTID_MASK		0x001FFFFF
#define REV_MASK		0xF

#define MAIN_REV_1		0x10
#define MAIN_REV_2		0x20
#define SUB_REV1		0x1
#define SUB_REV2		0x2

static struct gs_chipid_info gs_soc_info;

static const char *product_id_to_name(unsigned int product_id)
{
	const char *soc_name;
	unsigned int soc_id = product_id;

	switch (soc_id) {
	case GS101_SOC_ID:
		soc_name = "GS101";
		break;
	default:
		soc_name = "UNKNOWN";
	}
	return soc_name;
}

static const struct gs_chipid_variant drv_data_gs101 = {
	.product_ver = 1,
	.unique_id_reg = 0x04,
	.rev_reg = 0x10,
	.main_rev_bit = 0,
	.sub_rev_bit = 16,
};

static char lot_id[6];

static void chipid_dec_to_36(u32 in, u32 uniq_id1, char *p)
{
	u32 mod;
	u32 i;
	u32 val;

	for (i = 4; i >= 1; i--) {
		mod = in % 36;
		in /= 36;
		p[i] = (mod < 10) ? (mod + '0') : (mod - 10 + 'A');
	}

	val = (uniq_id1 >> 10) & 0x3;

	switch (val) {
	case 0:
		p[0] = 'N';
		break;
	case 1:
		p[0] = 'S';
		break;
	case 2:
		p[0] = 'A';
		break;
	case 3:
	default:
		break;
	}

	p[5] = 0;
}

/*
 *  sysfs implementation for gs-snapshot
 *  you can access the sysfs of gs-snapshot to /sys/devices/system/chip-id
 *  path.
 */
static struct bus_type chipid_subsys = {
	.name = "chip-id",
	.dev_name = "chip-id",
};

static ssize_t product_id_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%08X\n", gs_soc_info.product_id);
}

static ssize_t unique_id_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%010llX\n", gs_soc_info.unique_id);
}

static ssize_t lot_id_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%08X\n", gs_soc_info.lot_id);
}

static ssize_t lot_id2_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", gs_soc_info.lot_id2);
}

static ssize_t revision_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%08X\n", gs_soc_info.revision);
}

static ssize_t evt_ver_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "EVT%1X.%1X\n",
			 gs_soc_info.main_rev, gs_soc_info.sub_rev);
}

static ssize_t raw_str_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", gs_soc_info.raw_str);
}

static ssize_t ap_hw_tune_str_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", gs_soc_info.ap_hw_tune_str);
}

static DEVICE_ATTR_RO(product_id);
static DEVICE_ATTR_RO(unique_id);
static DEVICE_ATTR_RO(lot_id);
static DEVICE_ATTR_RO(lot_id2);
static DEVICE_ATTR_RO(revision);
static DEVICE_ATTR_RO(evt_ver);
static DEVICE_ATTR_RO(raw_str);
static DEVICE_ATTR_RO(ap_hw_tune_str);

static struct attribute *chipid_sysfs_attrs[] = {
	&dev_attr_product_id.attr,
	&dev_attr_unique_id.attr,
	&dev_attr_lot_id.attr,
	&dev_attr_lot_id2.attr,
	&dev_attr_revision.attr,
	&dev_attr_evt_ver.attr,
	&dev_attr_raw_str.attr,
	&dev_attr_ap_hw_tune_str.attr,
	NULL,
};

static struct attribute_group chipid_sysfs_group = {
	.attrs = chipid_sysfs_attrs,
};

static const struct attribute_group *chipid_sysfs_groups[] = {
	&chipid_sysfs_group,
	NULL,
};

static int chipid_sysfs_init(void)
{
	int ret = 0;

	ret = subsys_system_register(&chipid_subsys, chipid_sysfs_groups);
	if (ret)
		dev_err(&gs_soc_info.pdev->dev,
			"fail to register chip-id subsys\n");

	return ret;
}

u32 gs_chipid_get_type(void)
{
	if (!gs_soc_info.initialized)
		return -EPROBE_DEFER;

	return gs_soc_info.type;
}
EXPORT_SYMBOL_GPL(gs_chipid_get_type);

u32 gs_chipid_get_revision(void)
{
	if (!gs_soc_info.initialized)
		return -EPROBE_DEFER;

	return gs_soc_info.revision;
}
EXPORT_SYMBOL_GPL(gs_chipid_get_revision);

static void gs_chipid_get_chipid_info(void __iomem *reg)
{
	const struct gs_chipid_variant *data = gs_soc_info.drv_data;
	u64 val;
	u32 uniq_id0, uniq_id1;
	u32 temp;

	val = readl_relaxed(reg);

	switch (data->product_ver) {
	case 2:
		gs_soc_info.product_id = val & SOC_MASK_V2;
		break;
	case 1:
	default:
		gs_soc_info.product_id = val & SOC_MASK;
		gs_soc_info.type = val & SOC_TYPE_MASK;
		break;
	}

	gs_soc_info.main_rev = (val >> data->main_rev_bit) & REV_MASK;

	val = readl_relaxed(reg + data->rev_reg);
	gs_soc_info.sub_rev = (val >> data->sub_rev_bit) & REV_MASK;
	gs_soc_info.revision = (gs_soc_info.main_rev << 4)
	    | gs_soc_info.sub_rev;

	uniq_id0 = readl_relaxed(reg + data->unique_id_reg);
	uniq_id1 = readl_relaxed(reg + data->unique_id_reg + 4);
	val = (u64)uniq_id0 | ((u64)uniq_id1 << 32UL);
	gs_soc_info.unique_id = val;
	gs_soc_info.lot_id = val & LOTID_MASK;

	temp = bitrev32(uniq_id0);
	temp = (temp >> 11) & LOTID_MASK;
	chipid_dec_to_36(temp, uniq_id1, lot_id);
	gs_soc_info.lot_id2 = lot_id;
}

static void gs_chipid_get_raw_str(void __iomem *reg)
{
	u32 addr;
	u8 val;
	int str_pos = 0;

	for (addr = 0x4; addr < 0xA; addr++) {
		val = readb_relaxed(reg + addr);
		str_pos += scnprintf(gs_soc_info.raw_str + str_pos,
				     RAW_HEX_STR_SIZE - str_pos,
				     "%02x", val);
	}
	for (addr = 0xA000; addr < 0xA024; addr++) {
		val = readb_relaxed(reg + addr);
		str_pos += scnprintf(gs_soc_info.raw_str + str_pos,
				     RAW_HEX_STR_SIZE - str_pos,
				     "%02x", val);
	}
	for (addr = 0x9000; addr < 0x9010; addr++) {
		val = readb_relaxed(reg + addr);
		str_pos += scnprintf(gs_soc_info.raw_str + str_pos,
				     RAW_HEX_STR_SIZE - str_pos,
				     "%02x", val);
	}
}

static void gs_chipid_get_ap_hw_tune_str(void __iomem *reg)
{
	u32 addr;
	u8 val;
	int str_pos = 0;
	int arr_pos = 0;

	for (addr = 0xC300; addr < 0xC320; addr++) {
		val = readb_relaxed(reg + addr);
		str_pos += scnprintf(gs_soc_info.ap_hw_tune_str + str_pos,
				     AP_HW_TUNE_HEX_STR_SIZE - str_pos, "%02x",
				     val);
		if (arr_pos < ARRAY_SIZE(gs_soc_info.ap_hw_tune_arr))
			gs_soc_info.ap_hw_tune_arr[arr_pos++] = val;
	}
}

int gs_chipid_get_ap_hw_tune_array(const u8 **array)
{
	if (!gs_soc_info.initialized)
		return -EPROBE_DEFER;

	*array = gs_soc_info.ap_hw_tune_arr;
	return sizeof(gs_soc_info.ap_hw_tune_arr);
}
EXPORT_SYMBOL_GPL(gs_chipid_get_ap_hw_tune_array);

static const struct of_device_id of_gs_chipid_ids[] = {
	{
	 .compatible = "google,gs101-chipid",
	 .data = &drv_data_gs101,
	 },
	{},
};

/**
 *  gs_chipid_early_init: Early chipid initialization
 *  @dev: pointer to chipid device
 */
void gs_chipid_early_init(void)
{
	struct device_node *np;
	const struct of_device_id *match;
	void __iomem *reg;

	if (gs_soc_info.initialized)
		return;

	np = of_find_matching_node_and_match(NULL, of_gs_chipid_ids, &match);
	if (!np || !match)
		panic("%s, failed to find chipid node or match\n", __func__);

	gs_soc_info.drv_data = (struct gs_chipid_variant *)match->data;
	reg = of_iomap(np, 0);
	if (!reg)
		panic("%s: failed to map registers\n", __func__);

	gs_chipid_get_chipid_info(reg);
	gs_chipid_get_raw_str(reg);
	gs_chipid_get_ap_hw_tune_str(reg);
	iounmap(reg);
	gs_soc_info.initialized = true;
}

static int gs_chipid_probe(struct platform_device *pdev)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	int ret;

	gs_chipid_early_init();

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENODEV;

	soc_dev_attr->family = "Diablo";

	ret = of_property_read_string(of_root, "model", &soc_dev_attr->machine);
	if (ret)
		goto free_soc;

	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%u",
					   gs_soc_info.revision);
	if (!soc_dev_attr->revision)
		goto free_soc;

	soc_dev_attr->soc_id = product_id_to_name(gs_soc_info.product_id);
	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev))
		goto free_rev;

	dev_info(&pdev->dev, "CPU[%s]%s CPU_REV[0x%x] Detected\n",
		 product_id_to_name(gs_soc_info.product_id),
		 gs_soc_info.type ? "B0" : "", gs_soc_info.revision);
	gs_soc_info.pdev = pdev;

	chipid_sysfs_init();
	return 0;
free_rev:
	kfree(soc_dev_attr->revision);
free_soc:
	kfree(soc_dev_attr);
	return -EINVAL;
}

static struct platform_driver gs_chipid_driver = {
	.driver = {
		   .name = "gs-chipid",
		   .of_match_table = of_gs_chipid_ids,
		   },
	.probe = gs_chipid_probe,
};

module_platform_driver(gs_chipid_driver);

MODULE_DESCRIPTION("GS ChipID driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gs-chipid");
