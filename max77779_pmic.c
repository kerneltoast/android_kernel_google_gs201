// SPDX-License-Identifier: GPL-2.0-only
/*
 * max77779 pmic driver
 *
 * Copyright (C) 2023 Google, LLC.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/regmap.h>

#if IS_ENABLED(CONFIG_DEBUG_FS)
# include <linux/debugfs.h>
#endif

#include "google_bms.h"
#include "max77779_pmic.h"

#define MAX77779_PMIC_ID_VAL	0x79
#define MAX77779_PMIC_NUM_REGS (MAX77779_PMIC_GPIO_VGPI_CNFG - MAX77779_PMIC_ID + 1)

static inline int max77779_pmic_reg_read(struct regmap *regmap, uint8_t reg, uint8_t *val)
{
	int ret, ival;

	ret = regmap_read(regmap, reg, &ival);
	if (ret == 0)
		*val = 0xFF & ival;

	return ret;
}

static int max77779_pmic_reg_write(struct regmap *map, uint8_t reg, uint8_t val)
{
	return regmap_write(map, reg, val);
}

static int max77779_pmic_reg_update(struct regmap *map, uint8_t reg, uint8_t mask, uint8_t val)
{
	return regmap_update_bits(map, reg, mask, val);
}

static inline int max77779_pmic_readn(struct max77779_pmic_info *info,
				      int addr, u8 *val, int len)
{
	int rc;

	rc = regmap_bulk_read(info->regmap, addr, val, len);
	if (rc < 0)
		dev_warn(info->dev, "regmap_read failed for address %04x rc=%d\n",
			 addr, rc);

	return rc;
}

int max77779_external_pmic_reg_read(struct device *dev, uint8_t reg, uint8_t *val)
{
	struct max77779_pmic_info *info = dev_get_drvdata(dev);

	if (!info || !info->regmap)
		return -ENODEV;

	return max77779_pmic_reg_read(info->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(max77779_external_pmic_reg_read);

int max77779_external_pmic_reg_write(struct device *dev, uint8_t reg, uint8_t val)
{
	struct max77779_pmic_info *info = dev_get_drvdata(dev);

	if (!info || !info->regmap)
		return -ENODEV;

	return max77779_pmic_reg_write(info->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(max77779_external_pmic_reg_write);

int max77779_external_pmic_reg_update(struct device *dev, uint8_t reg, uint8_t msk, uint8_t val)
{
	struct max77779_pmic_info *info = dev_get_drvdata(dev);

	if (!info || !info->regmap)
		return -ENODEV;

	return max77779_pmic_reg_update(info->regmap, reg, msk, val);
}
EXPORT_SYMBOL_GPL(max77779_external_pmic_reg_update);

#ifdef CONFIG_DEBUG_FS
static int addr_write(void *d, u64 val)
{
	struct max77779_pmic_info *info = d;

	info->addr = val & 0xff;
	return 0;
}

static int addr_read(void *d, u64 *val)
{
	struct max77779_pmic_info *info = d;

	*val  = info->addr;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(addr_fops, addr_read, addr_write, "%llx\n");

static int data_write(void *d, u64 val)
{
	struct max77779_pmic_info *info = d;

	return max77779_pmic_reg_write(info->regmap, info->addr, (val & 0xff));
}

static int data_read(void *d, u64 *val)
{
	struct max77779_pmic_info *info = d;
	uint8_t rd_val;
	int ret;

	ret = max77779_pmic_reg_read(info->regmap, info->addr, &rd_val);
	*val = rd_val;

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(data_fops, data_read, data_write, "%llx\n");

static int dbg_init_fs(struct max77779_pmic_info *info)
{
	info->de = debugfs_create_dir("max77779_pmic", 0);
	if (!info->de)
		return -EINVAL;

	debugfs_create_file("addr", 0600, info->de, info, &addr_fops);

	debugfs_create_file("data", 0600, info->de, info, &data_fops);

	return 0;
}
static void dbg_remove_fs(struct max77779_pmic_info *info)
{
	debugfs_remove_recursive(info->de);
	info->de = NULL;
}
#else
static inline int dbg_init_fs(struct max77779_pmic_info *info)
{
	return 0;
}
static inline void dbg_remove_fs(struct max77779_pmic_info *info) {}
#endif

static ssize_t registers_dump_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct max77779_pmic_info *info = dev_get_drvdata(dev);
	static u8 *dump;
	int ret = 0, offset = 0, i;

	if (!info->regmap) {
		pr_err("Failed to read, no regmap\n");
		return -EIO;
	}

	mutex_lock(&info->reg_dump_lock);

	dump = kzalloc(MAX77779_PMIC_NUM_REGS * sizeof(u8), GFP_KERNEL);
	if (!dump) {
		dev_err(dev, "[%s]: Failed to allocate mem ret:%d\n", __func__, ret);
		goto unlock;
	}

	ret = max77779_pmic_readn(info, MAX77779_PMIC_ID, dump, MAX77779_PMIC_NUM_REGS);
	if (ret < 0) {
		dev_err(dev, "[%s]: Failed to dump ret:%d\n", __func__, ret);
		goto done;
	}

	for (i = 0; i < MAX77779_PMIC_NUM_REGS; i++) {
		u32 reg_address = i + MAX77779_PMIC_ID;

		if (!max77779_pmic_is_readable(dev, reg_address))
			continue;

		ret = sysfs_emit_at(buf, offset, "%02x: %02x\n", reg_address, dump[i]);
		if (!ret) {
			dev_err(dev, "[%s]: Not all registers printed. last:%x\n", __func__,
				reg_address - 1);
			break;
		}
		offset += ret;
	}

done:
	kfree(dump);
unlock:
	mutex_unlock(&info->reg_dump_lock);
	return offset;
}
static DEVICE_ATTR_RO(registers_dump);

bool max77779_pmic_is_readable(struct device *dev, unsigned int reg)
{
	switch(reg) {
	case MAX77779_PMIC_ID ... MAX77779_PMIC_OTP_REVISION:
	case MAX77779_PMIC_INTSRC_STS ... MAX77779_PMIC_INT_MASK:
	case MAX77779_PMIC_EVENT_CNT_CFG ... MAX77779_PMIC_EVENT_CNT_UVLO1:
	case MAX77779_PMIC_I2C_CNFG ... MAX77779_PMIC_SPMI_STS:
	case MAX77779_PMIC_SWRESET ... MAX77779_PMIC_CONTROL_FG:
	case MAX77779_PMIC_RISCV_DEVICE_ID ... MAX77779_PMIC_RISCV_FW_SUB_REV:
	case MAX77779_PMIC_RISCV_AP_DATAOUT1 ... MAX77779_PMIC_RISCV_AP_DATAOUT_OPCODE:
	case MAX77779_PMIC_RISCV_AP_DATAIN0 ... MAX77779_PMIC_RISCV_SysMsg:
	case MAX77779_PMIC_RISCV_COMMAND_HW:
	case MAX77779_PMIC_GPIO_SGPIO_INT ... MAX77779_PMIC_GPIO_VGPI_CNFG:
		return true;
	default:
		return false;
	}
}
EXPORT_SYMBOL_GPL(max77779_pmic_is_readable);

static const struct mfd_cell max77779_pmic_devs[] = {
	{
		.name = "max77779-pmic-irq",
		.of_compatible = "max77779-pmic-irq",
	},
	{
		.name = "max77779-pinctrl",
		.of_compatible = "max77779-pinctrl",
	},
	{
		.name = "max77779-pmic-sgpio",
		.of_compatible = "max77779-pmic-sgpio",
	},
};

/*
 * Initialization requirements
 * struct max77779_pmic_info *info
 * - dev
 * - regmap
 */
int max77779_pmic_init(struct max77779_pmic_info *info)
{
	uint8_t pmic_id;
	int err = 0;

	err = max77779_pmic_reg_read(info->regmap, MAX77779_PMIC_ID, &pmic_id);
	if (err) {
		dev_err(info->dev, "Unable to read Device ID (%d)\n", err);
		return err;
	} else if (MAX77779_PMIC_ID_VAL != pmic_id) {
		dev_err(info->dev, "Unsupported Device ID (%#02x)\n", pmic_id);
		return -ENODEV;
	}

	mfd_add_devices(info->dev, PLATFORM_DEVID_AUTO, max77779_pmic_devs,
			ARRAY_SIZE(max77779_pmic_devs), NULL, 0, NULL);

	dbg_init_fs(info);

	mutex_init(&info->reg_dump_lock);
	err = device_create_file(info->dev, &dev_attr_registers_dump);
	if (err != 0)
		dev_warn(info->dev, "Failed to create registers_dump, ret=%d\n", err);

	return err;
}
EXPORT_SYMBOL_GPL(max77779_pmic_init);

void max77779_pmic_remove(struct max77779_pmic_info *info)
{
	dbg_remove_fs(info);
}
EXPORT_SYMBOL_GPL(max77779_pmic_remove);

MODULE_DESCRIPTION("Maxim 77779 PMIC driver");
MODULE_AUTHOR("James Wylder <jwylder@google.com>");
MODULE_LICENSE("GPL");
