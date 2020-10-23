// SPDX-License-Identifier: GPL-2.0
/*
 * gs101_bcl.c gsoc101 bcl driver
 *
 * Copyright (c) 2020, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif
#include "../thermal_core.h"

/* This driver determines if HW was throttled due to SMPL/OCP */

#define CPUCL0_BASE (0x20c00000)
#define CPUCL1_BASE (0x20c10000)
#define CPUCL2_BASE (0x20c20000)
#define SYSREG_CPUCL0_BASE (0x20c40000)
#define CLUSTER0_GENERAL_CTRL_64 (0x1404)
#define CLKDIVSTEP (0x830)
#define CPUCL0_CLKDIVSTEP_STAT (0x83c)
#define CPUCL12_CLKDIVSTEP_STAT (0x848)
#define CLUSTER0_MPMM (0x1408)
#define CLUSTER0_PPM (0x140c)
#define MPMMEN_MASK (0xF << 21)
#define PPMEN_MASK (0x3 << 8)
#define PPMCTL_MASK (0xFF)
#define OCP_WARN_MASK (0x1F)
#define B3M_UPPER_LIMIT (7000)
#define B3M_LOWER_LIMIT (1688)
#define B3M_STEP (166)
#define B2M_UPPER_LIMIT (12000)
#define B2M_LOWER_LIMIT (4000)
#define B2M_STEP (250)
#define B10M_UPPER_LIMIT (10500)
#define B10M_LOWER_LIMIT (2500)
#define B10M_STEP (250)
#define B2S_UPPER_LIMIT (13200)
#define B2S_LOWER_LIMIT (5200)
#define B2S_STEP (250)
#define SMPL_UPPER_LIMIT (3300)
#define SMPL_LOWER_LIMIT (2600)

enum sys_throttling_core { SYS_THROTTLING_MID_CORE, SYS_THROTTLING_BIG_CORE };

enum sys_throttling_switch { SYS_THROTTLING_DISABLED, SYS_THROTTLING_ENABLED };

enum sys_throttling_mode { SYS_THROTTLING_MPMM_MODE, SYS_THROTTLING_PPM_MODE };

enum PMIC_REG { S2MPG10, S2MPG11 };

struct gs101_bcl_dev {
	struct device *device;
	struct dentry *debug_entry;
	void __iomem *cpu0_mem;
	void __iomem *cpu1_mem;
	void __iomem *cpu2_mem;
	void __iomem *sysreg_cpucl0;

	struct notifier_block psy_nb;
	struct delayed_work soc_eval_work;
	struct delayed_work mfd_init;
	int trip_high_temp;
	int trip_low_temp;
	int trip_val;
	struct mutex state_trans_lock;
	struct thermal_zone_device *tz_dev;
	struct thermal_zone_of_device_ops ops;
	void *iodev;

	struct i2c_client *s2mpg10_i2c;
	struct i2c_client *s2mpg11_i2c;
};


static const struct platform_device_id google_gs101_id_table[] = {
	{.name = "gs101-bcl-m",},
	{.name = "gs101-bcl-s",},
	{},
};

DEFINE_MUTEX(sysreg_lock);

static int gs101_bcl_set_soc(void *data, int low, int high)
{
	struct gs101_bcl_dev *gs101_bcl_device = (struct gs101_bcl_dev *)data;

	if (high == gs101_bcl_device->trip_high_temp)
		return 0;

	mutex_lock(&gs101_bcl_device->state_trans_lock);
	gs101_bcl_device->trip_low_temp = low;
	gs101_bcl_device->trip_high_temp = high;
	schedule_delayed_work(&gs101_bcl_device->soc_eval_work, 0);

	mutex_unlock(&gs101_bcl_device->state_trans_lock);
	return 0;
}

static int gs101_bcl_read_soc(void *data, int *val)
{
	static struct power_supply *batt_psy;
	union power_supply_propval ret = {
		0,
	};
	int err = 0;

	*val = 100;
	if (!batt_psy)
		batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		err = power_supply_get_property(
			batt_psy, POWER_SUPPLY_PROP_CAPACITY, &ret);
		if (err) {
			pr_err("battery percentage read error:%d\n", err);
			return err;
		}
		*val = 100 - ret.intval;
	}
	pr_debug("soc:%d\n", *val);

	return err;
}

static void gs101_bcl_evaluate_soc(struct work_struct *work)
{
	int battery_percentage_reverse;
	struct gs101_bcl_dev *gs101_bcl_device =
	    container_of(work, struct gs101_bcl_dev, soc_eval_work.work);

	if (gs101_bcl_read_soc(NULL, &battery_percentage_reverse))
		return;

	mutex_lock(&gs101_bcl_device->state_trans_lock);
	if ((battery_percentage_reverse < gs101_bcl_device->trip_high_temp) &&
		(battery_percentage_reverse > gs101_bcl_device->trip_low_temp))
		goto eval_exit;

	gs101_bcl_device->trip_val = battery_percentage_reverse;
	mutex_unlock(&gs101_bcl_device->state_trans_lock);
	thermal_zone_device_update(gs101_bcl_device->tz_dev,
				   THERMAL_EVENT_UNSPECIFIED);

	return;
eval_exit:
	mutex_unlock(&gs101_bcl_device->state_trans_lock);
}

static int battery_supply_callback(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(nb, struct gs101_bcl_dev, psy_nb);

	if (strcmp(psy->desc->name, "battery") == 0)
		schedule_delayed_work(&gs101_bcl_device->soc_eval_work, 0);

	return NOTIFY_OK;
}

static int gs101_bcl_soc_remove(struct gs101_bcl_dev *gs101_bcl_device)
{
	power_supply_unreg_notifier(&gs101_bcl_device->psy_nb);
	if (gs101_bcl_device->tz_dev)
		thermal_zone_of_sensor_unregister(gs101_bcl_device->device,
						  gs101_bcl_device->tz_dev);

	return 0;
}

static int get_cpucl0_stat(void *data, u64 *val)
{
	unsigned int reg = 0;
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;

	reg = __raw_readl(bcl_dev->cpu0_mem + CPUCL0_CLKDIVSTEP_STAT);
	*val = (reg >> 16) & 0x0FFF;
	return 0;
}

static int reset_cpucl0_stat(void *data, u64 val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;

	if (val == 0)
		__raw_writel(0x1, bcl_dev->cpu0_mem + CLKDIVSTEP);
	else
		__raw_writel(0x107f, bcl_dev->cpu0_mem + CLKDIVSTEP);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpucl0_clkdivstep_stat_fops, get_cpucl0_stat,
			reset_cpucl0_stat, "%d\n");

static int get_cpucl2_stat(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;
	unsigned int reg = 0;

	reg = __raw_readl(bcl_dev->cpu2_mem + CPUCL12_CLKDIVSTEP_STAT);
	*val = (reg >> 16) & 0x0FFF;
	return 0;
}

static int reset_cpucl2_stat(void *data, u64 val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;

	if (val == 0)
		__raw_writel(0x1, bcl_dev->cpu2_mem + CLKDIVSTEP);
	else
		__raw_writel(0x107f, bcl_dev->cpu2_mem + CLKDIVSTEP);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpucl2_clkdivstep_stat_fops, get_cpucl2_stat,
			reset_cpucl2_stat, "%d\n");

static int get_cpucl1_stat(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;
	unsigned int reg = 0;

	reg = __raw_readl(bcl_dev->cpu1_mem + CPUCL12_CLKDIVSTEP_STAT);
	*val = (reg >> 16) & 0x0FFF;
	return 0;
}

static int reset_cpucl1_stat(void *data, u64 val)
{
	struct gs101_bcl_dev *bcl_dev = (struct gs101_bcl_dev *)data;

	if (val == 0)
		__raw_writel(0x1, bcl_dev->cpu1_mem + CLKDIVSTEP);
	else
		__raw_writel(0x107f, bcl_dev->cpu1_mem + CLKDIVSTEP);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpucl1_clkdivstep_stat_fops, get_cpucl1_stat,
			reset_cpucl1_stat, "%d\n");

static int get_smpl_lvl(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;
	u8 value = 0;
	unsigned int smpl_warn_lvl;

	if (!bcl_dev->s2mpg10_i2c) {
		pr_err("S2MPG10 I2C not found.");
		return 0;
	}
	if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c,
			     S2MPG10_PM_SMPL_WARN_CTRL, &value)) {
		pr_err("S2MPG10 read SMPL_WARN_CTRL failed.");
		return 0;
	}
	value >>= S2MPG10_SMPL_WARN_LVL_SHIFT;
	smpl_warn_lvl = value * 100 + SMPL_LOWER_LIMIT;
	*val = smpl_warn_lvl;
	return 0;
}

static int set_smpl_lvl(void *data, u64 val)
{
	struct gs101_bcl_dev *bcl_dev = data;
	u8 value;
	int ret;

	if (val < SMPL_LOWER_LIMIT || val > SMPL_UPPER_LIMIT) {
		pr_err("SMPL_WARN LEVEL %d outside of range %d - %d mV.", val,
		       SMPL_LOWER_LIMIT, SMPL_UPPER_LIMIT);
		return -1;
	}

	if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c,
			     S2MPG10_PM_SMPL_WARN_CTRL, &value)) {
		pr_err("S2MPG10 read 0x%x failed.", S2MPG10_PM_SMPL_WARN_CTRL);
	}
	value |= ((val - SMPL_LOWER_LIMIT) / 100) << S2MPG10_SMPL_WARN_LVL_SHIFT;
	ret = s2mpg10_write_reg(bcl_dev->s2mpg10_i2c,
				S2MPG10_PM_SMPL_WARN_CTRL, value);

	if (ret)
		pr_err("i2c write error setting smpl_warn\n");

	return ret;

}

DEFINE_SIMPLE_ATTRIBUTE(smpl_lvl_fops, get_smpl_lvl,
			set_smpl_lvl, "%d\n");

static int get_ocp_lvl(void *data, u64 *val, u8 addr,
		       u8 pmic, u8 mask, u16 limit,
		       u8 step)
{
	struct gs101_bcl_dev *bcl_dev = data;
	u8 value = 0;
	unsigned int ocp_warn_lvl;

	if (pmic == S2MPG10) {
		if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, addr, &value)) {
			pr_err("S2MPG10 read 0x%x failed.", addr);
			return -1;
		}
	} else {
		if (s2mpg11_read_reg(bcl_dev->s2mpg11_i2c, addr, &value)) {
			pr_err("S2MPG11 read 0x%x failed.", addr);
			return -1;
		}
	}
	value &= mask;
	ocp_warn_lvl = limit - value * step;
	*val = ocp_warn_lvl;
	return 0;
}

static int set_ocp_lvl(void *data, u64 val, u8 addr, u8 pmic, u8 mask,
		       u16 llimit, u16 ulimit, u8 step)
{
	struct gs101_bcl_dev *bcl_dev = data;
	u8 value;
	int ret;

	if (val < llimit || val > ulimit) {
		pr_err("OCP_WARN LEVEL %d outside of range %d - %d mA.", val,
		       llimit, ulimit);
		return -1;
	}
	if (pmic == S2MPG10) {
		if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, addr, &value)) {
			pr_err("S2MPG10 read 0x%x failed.", addr);
		}
		value |= ((ulimit - val) / step) << S2MPG10_OCP_WARN_LVL_SHIFT;
		ret = s2mpg10_write_reg(bcl_dev->s2mpg10_i2c, addr, value);
	} else {
		if (s2mpg11_read_reg(bcl_dev->s2mpg11_i2c, addr, &value)) {
			pr_err("S2MPG11 read 0x%x failed.", addr);
		}
		value |= ((ulimit - val) / step) << S2MPG11_OCP_WARN_LVL_SHIFT;
		ret = s2mpg11_write_reg(bcl_dev->s2mpg11_i2c, addr, value);
	}

	if (ret)
		pr_err("i2c write error setting smpl_warn\n");

	return ret;
}

static int get_soft_cpu1_lvl(void *data, u64 *val)
{
	return get_ocp_lvl(data, val, S2MPG10_PM_B3M_SOFT_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B3M_UPPER_LIMIT, B3M_STEP);
}

static int set_soft_cpu1_lvl(void *data, u64 val)
{
	return set_ocp_lvl(data, val, S2MPG10_PM_B3M_SOFT_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B3M_LOWER_LIMIT, B3M_UPPER_LIMIT,
			   B3M_STEP);
}

DEFINE_SIMPLE_ATTRIBUTE(soft_cpu1_lvl_fops, get_soft_cpu1_lvl,
			set_soft_cpu1_lvl, "%d\n");

static int get_soft_cpu2_lvl(void *data, u64 *val)
{
	return get_ocp_lvl(data, val, S2MPG10_PM_B2M_SOFT_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B2M_UPPER_LIMIT, B2M_STEP);
}

static int set_soft_cpu2_lvl(void *data, u64 val)
{
	return set_ocp_lvl(data, val, S2MPG10_PM_B2M_SOFT_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B2M_LOWER_LIMIT, B2M_UPPER_LIMIT,
			   B2M_STEP);
}

DEFINE_SIMPLE_ATTRIBUTE(soft_cpu2_lvl_fops, get_soft_cpu2_lvl,
			set_soft_cpu2_lvl, "%d\n");

static int get_cpu1_lvl(void *data, u64 *val)
{
	return get_ocp_lvl(data, val, S2MPG10_PM_B3M_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B3M_UPPER_LIMIT, B3M_STEP);
}

static int set_cpu1_lvl(void *data, u64 val)
{
	return set_ocp_lvl(data, val, S2MPG10_PM_B3M_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B3M_LOWER_LIMIT, B3M_UPPER_LIMIT,
			   B3M_STEP);
}

DEFINE_SIMPLE_ATTRIBUTE(cpu1_lvl_fops, get_cpu1_lvl,
			set_cpu1_lvl, "%d\n");

static int get_cpu2_lvl(void *data, u64 *val)
{
	return get_ocp_lvl(data, val, S2MPG10_PM_B2M_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B2M_UPPER_LIMIT, B2M_STEP);
}

static int set_cpu2_lvl(void *data, u64 val)
{
	return set_ocp_lvl(data, val, S2MPG10_PM_B2M_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B2M_LOWER_LIMIT, B2M_UPPER_LIMIT,
			   B2M_STEP);
}

DEFINE_SIMPLE_ATTRIBUTE(cpu2_lvl_fops, get_cpu2_lvl,
			set_cpu2_lvl, "%d\n");

static int get_tpu_lvl(void *data, u64 *val)
{
	return get_ocp_lvl(data, val, S2MPG10_PM_B10M_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B10M_UPPER_LIMIT, B10M_STEP);
}

static int set_tpu_lvl(void *data, u64 val)
{
	return set_ocp_lvl(data, val, S2MPG10_PM_B10M_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B10M_LOWER_LIMIT, B10M_UPPER_LIMIT,
			   B10M_STEP);
}

DEFINE_SIMPLE_ATTRIBUTE(tpu_lvl_fops, get_tpu_lvl,
			set_tpu_lvl, "%d\n");

static int get_soft_tpu_lvl(void *data, u64 *val)
{
	return get_ocp_lvl(data, val, S2MPG10_PM_B10M_SOFT_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B10M_UPPER_LIMIT, B10M_STEP);
}

static int set_soft_tpu_lvl(void *data, u64 val)
{
	return set_ocp_lvl(data, val, S2MPG10_PM_B10M_SOFT_OCP_WARN, S2MPG10,
			   OCP_WARN_MASK, B10M_LOWER_LIMIT, B10M_UPPER_LIMIT,
			   B10M_STEP);
}

DEFINE_SIMPLE_ATTRIBUTE(soft_tpu_lvl_fops, get_soft_tpu_lvl,
			set_soft_tpu_lvl, "%d\n");

static int get_gpu_lvl(void *data, u64 *val)
{
	return get_ocp_lvl(data, val, S2MPG11_PM_B2S_OCP_WARN, S2MPG11,
			   OCP_WARN_MASK, B2S_UPPER_LIMIT, B2S_STEP);
}

static int set_gpu_lvl(void *data, u64 val)
{
	return set_ocp_lvl(data, val, S2MPG11_PM_B2S_OCP_WARN, S2MPG11,
			   OCP_WARN_MASK, B2S_LOWER_LIMIT, B2S_UPPER_LIMIT,
			   B2S_STEP);
}

DEFINE_SIMPLE_ATTRIBUTE(gpu_lvl_fops, get_gpu_lvl,
			set_gpu_lvl, "%d\n");

static int get_soft_gpu_lvl(void *data, u64 *val)
{
	return get_ocp_lvl(data, val, S2MPG11_PM_B2S_SOFT_OCP_WARN, S2MPG11,
			   OCP_WARN_MASK, B2S_UPPER_LIMIT, B2S_STEP);
}

static int set_soft_gpu_lvl(void *data, u64 val)
{
	return set_ocp_lvl(data, val, S2MPG11_PM_B2S_SOFT_OCP_WARN, S2MPG11,
			   OCP_WARN_MASK, B2S_LOWER_LIMIT, B2S_UPPER_LIMIT,
			   B2S_STEP);
}

DEFINE_SIMPLE_ATTRIBUTE(soft_gpu_lvl_fops, get_soft_gpu_lvl,
			set_soft_gpu_lvl, "%d\n");

static void gs101_set_ppm_throttling(struct gs101_bcl_dev *gs101_bcl_device,
				     enum sys_throttling_core core,
				     enum sys_throttling_switch throttle_switch)
{
	unsigned int reg, mask;
	void __iomem *addr;

	if (!gs101_bcl_device->sysreg_cpucl0) {
		pr_err("sysreg_cpucl0 ioremap not mapped\n");
		return;
	}
	mutex_lock(&sysreg_lock);
	addr = gs101_bcl_device->sysreg_cpucl0 + CLUSTER0_PPM;
	reg = __raw_readl(addr);
	mask = (core == SYS_THROTTLING_BIG_CORE) ? (0x01 << 8) : (0x01 << 9);
	/* 75% dispatch reduction */
	if (throttle_switch == SYS_THROTTLING_ENABLED) {
		reg |= mask;
		reg |= PPMCTL_MASK;
	} else {
		reg &= ~mask;
		reg &= ~(PPMCTL_MASK);
	}
	__raw_writel(reg, addr);
	mutex_unlock(&sysreg_lock);
}

static void
gs101_set_mpmm_throttling(struct gs101_bcl_dev *gs101_bcl_device,
			  enum sys_throttling_core core,
			  enum sys_throttling_switch throttle_switch)
{
	unsigned int reg, mask;
	void __iomem *addr;

	if (!gs101_bcl_device->sysreg_cpucl0) {
		pr_err("sysreg_cpucl0 ioremap not mapped\n");
		return;
	}
	mutex_lock(&sysreg_lock);
	addr = gs101_bcl_device->sysreg_cpucl0 + CLUSTER0_MPMM;
	reg = __raw_readl(addr);
	mask = (core == SYS_THROTTLING_BIG_CORE) ? (0x0F << 4) : 0x0F;
	if (throttle_switch == SYS_THROTTLING_ENABLED)
		reg &= ~mask;
	else
		reg |= mask;
	__raw_writel(reg, addr);
	mutex_unlock(&sysreg_lock);
}

static int gs101_enable_ppm_throttling(void *data, u64 val)
{
	unsigned int mode;

	pr_info("gs101: enable PPM throttling");

	mode = (val == 0) ? SYS_THROTTLING_DISABLED : SYS_THROTTLING_ENABLED;
	gs101_set_ppm_throttling(data, SYS_THROTTLING_MID_CORE, mode);
	gs101_set_ppm_throttling(data, SYS_THROTTLING_BIG_CORE, mode);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ppm_fops, NULL, gs101_enable_ppm_throttling, "%d\n");

static int gs101_enable_mpmm_throttling(void *data, u64 val)
{
	unsigned int mode;

	pr_info("gs101: enable MPMM throttling");

	mode = (val == 0) ? SYS_THROTTLING_DISABLED : SYS_THROTTLING_ENABLED;
	gs101_set_mpmm_throttling(data, SYS_THROTTLING_MID_CORE, mode);
	gs101_set_mpmm_throttling(data, SYS_THROTTLING_BIG_CORE, mode);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mpmm_fops, NULL, gs101_enable_mpmm_throttling, "%d\n");

static void gs101_bcl_mfd_init(struct work_struct *work)
{
	u8 val = 0;
	struct gs101_bcl_dev *gs101_bcl_device =
	    container_of(work, struct gs101_bcl_dev, mfd_init.work);
	struct s2mpg10_dev *s2mpg10;
	struct s2mpg11_dev *s2mpg11;

	if (strcmp(dev_name(gs101_bcl_device->device), google_gs101_id_table[0].name) == 0) {
		s2mpg10 = gs101_bcl_device->iodev;
		gs101_bcl_device->s2mpg10_i2c = s2mpg10->pmic;
		gs101_bcl_device->s2mpg11_i2c = 0x0;
		if (s2mpg10_read_reg(gs101_bcl_device->s2mpg10_i2c, S2MPG10_COMMON_CHIPID, &val)) {
			pr_debug ("S2MPG10 not loaded.\n");
			return;
		}
		debugfs_create_file("smpl_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &smpl_lvl_fops);

		debugfs_create_file("soft_ocp_cpu1_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &soft_cpu1_lvl_fops);

		debugfs_create_file("soft_ocp_cpu2_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &soft_cpu2_lvl_fops);

		debugfs_create_file("soft_ocp_tpu_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &soft_cpu2_lvl_fops);

		debugfs_create_file("ocp_cpu1_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpu1_lvl_fops);

		debugfs_create_file("ocp_cpu2_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpu2_lvl_fops);

		debugfs_create_file("ocp_tpu_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpu2_lvl_fops);
	}
	if (strcmp(dev_name(gs101_bcl_device->device), "gs101-bcl11") == 0) {
		s2mpg11 = gs101_bcl_device->iodev;
		gs101_bcl_device->s2mpg10_i2c = 0;
		gs101_bcl_device->s2mpg11_i2c = s2mpg11->pmic;
		if (s2mpg11_read_reg(gs101_bcl_device->s2mpg11_i2c, S2MPG11_COMMON_CHIPID, &val)) {
			pr_debug ("S2MPG11 not loaded.\n");
			return;
		}
		debugfs_create_file("soft_ocp_gpu_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &soft_gpu_lvl_fops);

		debugfs_create_file("ocp_gpu_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &gpu_lvl_fops);
	}
}

static int google_gs101_bcl_probe(struct platform_device *pdev)
{
	unsigned int reg;
	int ret = 0;
	struct gs101_bcl_dev *gs101_bcl_device;
	struct dentry *root;
	struct s2mpg10_dev *s2mpg10;
	struct s2mpg11_dev *s2mpg11;

	gs101_bcl_device =
		devm_kzalloc(&pdev->dev, sizeof(*gs101_bcl_device), GFP_KERNEL);
	if (!gs101_bcl_device)
		return -ENOMEM;
	gs101_bcl_device->device = &pdev->dev;
	gs101_bcl_device->iodev = dev_get_drvdata(pdev->dev.parent);

	platform_set_drvdata(pdev, gs101_bcl_device);
	root = debugfs_lookup("gs101-bcl", NULL);
	if (!root) {
		gs101_bcl_device->debug_entry = debugfs_create_dir("gs101-bcl", 0);
		if (IS_ERR_OR_NULL(gs101_bcl_device->debug_entry)) {
			gs101_bcl_device->debug_entry = NULL;
			return -EINVAL;
		}
		debugfs_create_file("cpucl0_clkdiv_stat", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpucl0_clkdivstep_stat_fops);
		debugfs_create_file("cpucl1_clkdiv_stat", 0644,
				    gs101_bcl_device->debug_entry, gs101_bcl_device,
				    &cpucl1_clkdivstep_stat_fops);
		debugfs_create_file("cpucl2_clkdiv_stat", 0644,
				    gs101_bcl_device->debug_entry, gs101_bcl_device,
				    &cpucl2_clkdivstep_stat_fops);
		debugfs_create_file("mpmm_throttle", 0644,
				    gs101_bcl_device->debug_entry, gs101_bcl_device,
				    &mpmm_fops);
		debugfs_create_file("ppm_throttle", 0644, gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &ppm_fops);

	} else
		gs101_bcl_device->debug_entry = root;
	gs101_bcl_device->cpu0_mem = ioremap(CPUCL0_BASE, SZ_8K);
	if (!gs101_bcl_device->cpu0_mem) {
		pr_err("cpu0_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	gs101_bcl_device->cpu1_mem = ioremap(CPUCL1_BASE, SZ_8K);
	if (!gs101_bcl_device->cpu1_mem) {
		pr_err("cpu1_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	gs101_bcl_device->cpu2_mem = ioremap(CPUCL2_BASE, SZ_8K);
	if (!gs101_bcl_device->cpu2_mem) {
		pr_err("cpu2_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	gs101_bcl_device->sysreg_cpucl0 = ioremap(SYSREG_CPUCL0_BASE, SZ_8K);
	if (!gs101_bcl_device->sysreg_cpucl0) {
		pr_err("sysreg_cpucl0 ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}

	mutex_lock(&sysreg_lock);
	reg = __raw_readl(gs101_bcl_device->sysreg_cpucl0 +
			  CLUSTER0_GENERAL_CTRL_64);
	reg |= MPMMEN_MASK;
	__raw_writel(reg, gs101_bcl_device->sysreg_cpucl0 +
				  CLUSTER0_GENERAL_CTRL_64);
	reg = __raw_readl(gs101_bcl_device->sysreg_cpucl0 + CLUSTER0_PPM);
	reg |= PPMEN_MASK;
	__raw_writel(reg, gs101_bcl_device->sysreg_cpucl0 + CLUSTER0_PPM);
	mutex_unlock(&sysreg_lock);
	gs101_set_ppm_throttling(gs101_bcl_device,
				 SYS_THROTTLING_MID_CORE,
				 SYS_THROTTLING_DISABLED);
	gs101_set_ppm_throttling(gs101_bcl_device,
				 SYS_THROTTLING_BIG_CORE,
				 SYS_THROTTLING_DISABLED);
	gs101_set_mpmm_throttling(gs101_bcl_device,
				  SYS_THROTTLING_MID_CORE,
				  SYS_THROTTLING_DISABLED);
	gs101_set_mpmm_throttling(gs101_bcl_device,
				  SYS_THROTTLING_BIG_CORE,
				  SYS_THROTTLING_DISABLED);

	mutex_init(&gs101_bcl_device->state_trans_lock);
	gs101_bcl_device->ops.get_temp = gs101_bcl_read_soc;
	gs101_bcl_device->ops.set_trips = gs101_bcl_set_soc;
	INIT_DELAYED_WORK(&gs101_bcl_device->mfd_init, gs101_bcl_mfd_init);

	if (strcmp(dev_name(&pdev->dev), google_gs101_id_table[0].name) == 0) {
		INIT_DELAYED_WORK(&gs101_bcl_device->soc_eval_work, gs101_bcl_evaluate_soc);
		gs101_bcl_device->psy_nb.notifier_call = battery_supply_callback;
		ret = power_supply_reg_notifier(&gs101_bcl_device->psy_nb);
		if (ret < 0) {
			pr_err("soc notifier registration error. defer. err:%d\n", ret);
			ret = -EPROBE_DEFER;
			goto bcl_soc_probe_exit;
		}
		gs101_bcl_device->tz_dev = thermal_zone_of_sensor_register(
			gs101_bcl_device->device, 0, gs101_bcl_device, &gs101_bcl_device->ops);
		if (IS_ERR(gs101_bcl_device->tz_dev)) {
			pr_err("soc TZ register failed. err:%ld\n",
			       PTR_ERR(gs101_bcl_device->tz_dev));
			ret = PTR_ERR(gs101_bcl_device->tz_dev);
			gs101_bcl_device->tz_dev = NULL;
			ret = -EPROBE_DEFER;
			goto bcl_soc_probe_exit;
		}
		thermal_zone_device_update(gs101_bcl_device->tz_dev, THERMAL_DEVICE_UP);
		schedule_delayed_work(&gs101_bcl_device->soc_eval_work, 0);
	}
	schedule_delayed_work(&gs101_bcl_device->mfd_init, 1000);

	return 0;

bcl_soc_probe_exit:
	gs101_bcl_soc_remove(gs101_bcl_device);
	return ret;
}

static int google_gs101_bcl_remove(struct platform_device *pdev)
{
	struct gs101_bcl_dev *gs101_bcl_device = platform_get_drvdata(pdev);

	gs101_bcl_soc_remove(gs101_bcl_device);
	debugfs_remove(gs101_bcl_device->debug_entry);
	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "google,gs101-bcl-m"},
	{ .compatible = "google,gs101-bcl-s"},
	{},
};

static struct platform_driver gs101_bcl_driver = {
	.probe  = google_gs101_bcl_probe,
	.remove = google_gs101_bcl_remove,
	.id_table = google_gs101_id_table,
	.driver = {
		.name           = "google,gs101-bcl",
		.owner          = THIS_MODULE,
		.of_match_table = match_table,
	},
};

module_platform_driver(gs101_bcl_driver);
MODULE_DESCRIPTION("Google Battery Current Limiter");
MODULE_AUTHOR("George Lee <geolee@google.com>");
MODULE_LICENSE("GPL");
