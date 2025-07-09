/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023, Google Inc
 *
 * MAX77779 Scratch space management
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "max77779_regs.h"
#include "gbms_storage.h"
#include "max77779_sp.h"

/* hold lock on &data->page_lock */
static int max77779_sp_rd(uint8_t *buff, int addr, size_t count, struct regmap *regmap)
{
	const int page = addr / 256, offset = addr % 256;
	const int base = MAX77779_SP_DATA + ((offset & ~1) / 2);
	int ret = 0;

	/* todo: bulk of odd count, read across pages */
	if ((count > 2 && (count & 1)) || ((offset + count - 1) > 0xff) || page > 3)
		return -ERANGE;

	ret = regmap_write(regmap, MAX77779_SP_PAGE_CTRL, page);
	if (ret < 0)
		return ret;

	if (count > 2) {
		ret = regmap_raw_read(regmap, base, buff, count);
	} else if (count) {
		unsigned tmp = 0;

		/* one or two bytes, unaligned TODO: 2 bytes unaligned */
		ret = regmap_read(regmap, base, &tmp);
		if (ret < 0)
			return ret;

		if (count == 1) {
			if (offset & 1)
				*((uint8_t *)buff) = (tmp >> 8) & 0xFF;
			else
				*((uint8_t *)buff) = 0xFF & tmp;
		} else {
			*((uint16_t *)buff) = 0xFFFF & tmp;
		}
	}

	return ret;
}

/* hold lock on &data->page_lock */
static int max77779_sp_wr(const uint8_t *buff, int addr, size_t count, struct regmap *regmap)
{
	const int page = addr / 256, offset = addr % 256;
	const int base = MAX77779_SP_DATA + ((offset & ~1) / 2);
	unsigned tmp = 0;
	int ret = 0;

	/* todo: bulk of odd count, read across pages */
	if ((count > 2 && (count & 1)) || ((offset + count - 1) > 0xff) || page > 3)
		return -ERANGE;

	ret = regmap_write(regmap, MAX77779_SP_PAGE_CTRL, page);
	if (ret < 0)
		return ret;

	if (count > 2)
		return regmap_raw_write(regmap, base, buff, count);

	if (count == 1) {
		/* one or two bytes, unaligned TODO: 2 bytes unaligned */
		ret = regmap_read(regmap, base, &tmp);
		if (ret < 0)
			return ret;
		tmp &= 0xff << (!(offset & 1) * 8);
		tmp |= buff[0] << ((offset & 1) * 8);
	} else {
		tmp = ((uint16_t*)buff)[0];
	}

	return regmap_write(regmap, base, tmp);
}

static int max77779_sp_info(gbms_tag_t tag, size_t *addr, size_t size)
{
	switch (tag) {
	case GBMS_TAG_RS32:
		if (size && size > OPCODE_USER_SPACE_R_RES_LEN)
			return -EINVAL;
		*addr = RSBM_ADDR;
		break;
	case GBMS_TAG_RSBM:
		if (size && size > RS_TAG_LENGTH)
			return -EINVAL;
		*addr = RSBM_ADDR;
		break;
	case GBMS_TAG_RSBR:
		if (size && size > RS_TAG_LENGTH)
			return -EINVAL;
		*addr = RSBR_ADDR;
		break;
	case GBMS_TAG_SUFG:
		if (size && size > SU_TAG_LENGTH)
			return -EINVAL;
		*addr = SUFG_ADDR;
		break;
	case GBMS_TAG_RSOC:
		if (size && size > RSOC_TAG_LENGTH)
			return -EINVAL;
		*addr = RSOC_ADDR;
		break;
	case GBMS_TAG_FWHI:
		if (size && size > FWHI_TAG_LENGTH)
			return -EINVAL;
		*addr = FWHI_ADDR;
		break;
	case GBMS_TAG_FWSF:
		if (size && size > FWSF_TAG_LENGTH)
			return -EINVAL;
		*addr = FWSF_ADDR;
		break;
	case GBMS_TAG_MDLV:
		if (size && size > MDLV_TAG_LENGTH)
			return -EINVAL;
		*addr = MDLV_ADDR;
		break;
	default:
		return -ENOENT;
	}

	return 0;
}

static int max77779_sp_iter(int index, gbms_tag_t *tag, void *ptr)
{
	static gbms_tag_t keys[] = {GBMS_TAG_RS32, GBMS_TAG_RSBM, GBMS_TAG_RSBR,
				    GBMS_TAG_SUFG, GBMS_TAG_RSOC, GBMS_TAG_FWHI,
				    GBMS_TAG_FWSF, GBMS_TAG_MDLV};
	const int count = ARRAY_SIZE(keys);

	if (index >= 0 && index < count) {
		*tag = keys[index];
		return 0;
	}
	return -ENOENT;
}

static int max77779_sp_read(gbms_tag_t tag, void *buff, size_t size, void *ptr)
{
	struct max77779_sp_data *data = ptr;
	size_t addr;
	int ret;

	ret = max77779_sp_info(tag, &addr, size);
	if (ret < 0)
		return ret;

	mutex_lock(&data->page_lock);
	ret = max77779_sp_rd(buff, addr, size, data->regmap);
	mutex_unlock(&data->page_lock);

	return ret;
}

static int max77779_sp_write(gbms_tag_t tag, const void *buff, size_t size, void *ptr)
{
	struct max77779_sp_data *data = ptr;
	size_t addr;
	int ret;

	ret = max77779_sp_info(tag, &addr, size);
	if (ret < 0)
		return ret;

	mutex_lock(&data->page_lock);
	ret = max77779_sp_wr(buff, addr, size, data->regmap);
	mutex_unlock(&data->page_lock);

	return ret;
}

/* -- debug --------------------------------------------------------------- */
static int max77779_sp_debug_reg_read(void *d, u64 *val)
{
	struct max77779_sp_data *data = d;
	u8 reg = 0;
	int ret;

	ret = max77779_sp_rd(&reg, data->debug_reg_address, 1, data->regmap);
	if (ret)
		return ret;

	*val = reg;

	return 0;
}

static int max77779_sp_debug_reg_write(void *d, u64 val)
{
	struct max77779_sp_data *data = d;
	const u8 regval = val;

	return max77779_sp_wr(&regval, data->debug_reg_address, 1, data->regmap);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_reg_rw_fops, max77779_sp_debug_reg_read,
			max77779_sp_debug_reg_write, "%02llx\n");

#define DUMP_PAGES 1
static ssize_t registers_dump_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct max77779_sp_data *data = dev_get_drvdata(dev);
	u8 dump[256];
	int ret = 0, offset = 0, page, i;

	if (!data->regmap) {
		dev_err(dev, "Failed to read, no regmap\n");
		return -EIO;
	}

	mutex_lock(&data->page_lock);

	for (page = 0; page < DUMP_PAGES; page++) {
		const int addr = page * 256;

		ret = max77779_sp_rd(dump, addr, 256, data->regmap);
		if (ret < 0) {
			dev_err(dev, "[%s]: Failed to dump page:%d ret:%d\n", __func__, page, ret);
			goto unlock;
		}

		for (i = 0; i < 256; i++) {
			ret = sysfs_emit_at(buf, offset, "%02x: %02x\n", i + addr, dump[i]);
			if (!ret) {
				dev_err(dev, "[%s]: Not all registers printed. page:%d, last:%x\n",
					__func__, page, i - 1);
				goto unlock;
			}
			offset += ret;
		}
	}

unlock:
	mutex_unlock(&data->page_lock);
	return offset;
}
static DEVICE_ATTR_RO(registers_dump);

static struct gbms_storage_desc max77779_sp_dsc = {
	.write = max77779_sp_write,
	.read = max77779_sp_read,
	.iter = max77779_sp_iter,
};

bool max77779_sp_is_reg(struct device *dev, unsigned int reg)
{
	return (reg == MAX77779_SP_PAGE_CTRL) ||
	       (reg >= MAX77779_SP_DATA && reg <= MAX77779_SP_MAX_ADDR);
}
EXPORT_SYMBOL_GPL(max77779_sp_is_reg);

static int max77779_sp_dbg_init_fs(struct max77779_sp_data *data)
{
	data->de = debugfs_create_dir("max77779_sp", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_u32("address", 0600, data->de, &data->debug_reg_address);
	debugfs_create_file("data", 0600, data->de, data, &debug_reg_rw_fops);

	return 0;
}

/*
 * Initialization requirements
 * struct max77779_sp_data *data
 * - dev
 * - regmap
 */
int max77779_sp_init(struct max77779_sp_data *data)
{
	int ret, page;

	ret = regmap_read(data->regmap, MAX77779_SP_PAGE_CTRL, &page);
	if (ret) {
		dev_err(data->dev, "Unable to find scratchpad (%d)\n", ret);
		return ret;
	}

	mutex_init(&data->page_lock);

	if (!of_property_read_bool(data->dev->of_node, "max77779,no-storage")) {
		ret = gbms_storage_register(&max77779_sp_dsc, "max77779_sp", data);
		if (ret < 0)
			dev_warn(data->dev, "register failed, ret:%d\n", ret);
	}

	ret = max77779_sp_dbg_init_fs(data);
	if (ret < 0)
		dev_warn(data->dev, "Failed to initialize debug fs\n");

	ret = device_create_file(data->dev, &dev_attr_registers_dump);
	if (ret != 0)
		dev_warn(data->dev, "Failed to create registers_dump, ret=%d\n", ret);

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_sp_init);

void max77779_sp_remove(struct max77779_sp_data *data)
{
	if (data->de)
		debugfs_remove(data->de);
}
EXPORT_SYMBOL_GPL(max77779_sp_remove);

MODULE_DESCRIPTION("max77779 Scratch Driver");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
