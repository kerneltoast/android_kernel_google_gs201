// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_sysfs.c Google bcl sysfs driver
 *
 * Copyright (c) 2022, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include "bcl.h"
#include "soc/google/debug-snapshot.h"
#include <linux/regulator/pmic_class.h>
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
#include <dt-bindings/interrupt-controller/zuma.h>
#include <linux/mfd/samsung/rtc-s2mpg14.h>
#include <linux/mfd/samsung/s2mpg14-register.h>
#include <linux/mfd/samsung/s2mpg15-register.h>
#include <max77779_regs.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
#include <dt-bindings/interrupt-controller/gs201.h>
#include <linux/mfd/samsung/rtc-s2mpg12.h>
#include <linux/mfd/samsung/s2mpg12-register.h>
#include <linux/mfd/samsung/s2mpg13-register.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
#include <dt-bindings/interrupt-controller/gs101.h>
#include <linux/mfd/samsung/rtc-s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#endif
#include <max77759_regs.h>
#include <max77779.h>
#include <max777x9_bcl.h>

const unsigned int clk_stats_offset[] = {
	CPUCL0_CLKDIVSTEP_STAT,
	CLKDIVSTEP_STAT,
	CLKDIVSTEP_STAT,
	TPU_CLKDIVSTEP_STAT,
	G3D_CLKDIVSTEP_STAT,
	AUR_CLKDIVSTEP_STAT,
};

static const char * const batt_irq_names[] = {
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	"uvlo1", "uvlo2", "batoilo", "batoilo2"
#else
	"uvlo1", "uvlo2", "batoilo"
#endif
};

static const char * const concurrent_pwrwarn_irq_names[] = {
	"none", "mmwave", "rffe"
};

static ssize_t safe_emit_bcl_cnt(char* buf, struct bcl_zone * zone) {
	if (!zone)
		return sysfs_emit(buf, "0\n");
	else
		return sysfs_emit(buf, "%d\n", atomic_read(&zone->bcl_cnt));
}

static ssize_t safe_emit_bcl_capacity(char* buf, struct bcl_zone * zone) {
	if (!zone)
		return sysfs_emit(buf, "0\n");
	else
		return sysfs_emit(buf, "%d\n", zone->bcl_stats.capacity);
}

static ssize_t safe_emit_bcl_voltage(char* buf, struct bcl_zone * zone) {
	if (!zone)
		return sysfs_emit(buf, "0\n");
	else
		return sysfs_emit(buf, "%d\n", zone->bcl_stats.voltage);
}

static ssize_t safe_emit_bcl_time(char* buf, struct bcl_zone * zone) {
	if (!zone)
		return sysfs_emit(buf, "0\n");
	else
		return sysfs_emit(buf, "%lld\n", zone->bcl_stats._time);
}

static ssize_t batoilo_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[BATOILO1]);
}

static DEVICE_ATTR_RO(batoilo_count);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t batoilo2_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[BATOILO2]);
}

static DEVICE_ATTR_RO(batoilo2_count);
#endif

static ssize_t vdroop2_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[UVLO2]);
}

static DEVICE_ATTR_RO(vdroop2_count);

static ssize_t vdroop1_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[UVLO1]);
}

static DEVICE_ATTR_RO(vdroop1_count);

static ssize_t smpl_warn_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[SMPL_WARN]);
}

static DEVICE_ATTR_RO(smpl_warn_count);

static ssize_t ocp_cpu1_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[OCP_WARN_CPUCL1]);
}

static DEVICE_ATTR_RO(ocp_cpu1_count);

static ssize_t ocp_cpu2_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[OCP_WARN_CPUCL2]);
}

static DEVICE_ATTR_RO(ocp_cpu2_count);

static ssize_t ocp_tpu_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[OCP_WARN_TPU]);
}

static DEVICE_ATTR_RO(ocp_tpu_count);

static ssize_t ocp_gpu_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[OCP_WARN_GPU]);
}

static DEVICE_ATTR_RO(ocp_gpu_count);

static ssize_t soft_ocp_cpu1_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[SOFT_OCP_WARN_CPUCL1]);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_count);

static ssize_t soft_ocp_cpu2_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[SOFT_OCP_WARN_CPUCL2]);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_count);

static ssize_t soft_ocp_tpu_count_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[SOFT_OCP_WARN_TPU]);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_count);

static ssize_t soft_ocp_gpu_count_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_cnt(buf, bcl_dev->zone[SOFT_OCP_WARN_GPU]);
}

static DEVICE_ATTR_RO(soft_ocp_gpu_count);

static ssize_t batoilo_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[BATOILO1]);
}

static DEVICE_ATTR_RO(batoilo_cap);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t batoilo2_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[BATOILO2]);
}

static DEVICE_ATTR_RO(batoilo2_cap);
#endif

static ssize_t vdroop2_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[UVLO2]);
}

static DEVICE_ATTR_RO(vdroop2_cap);

static ssize_t vdroop1_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[UVLO1]);
}

static DEVICE_ATTR_RO(vdroop1_cap);

static ssize_t smpl_warn_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[SMPL_WARN]);
}

static DEVICE_ATTR_RO(smpl_warn_cap);

static ssize_t ocp_cpu1_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[OCP_WARN_CPUCL1]);
}

static DEVICE_ATTR_RO(ocp_cpu1_cap);

static ssize_t ocp_cpu2_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[OCP_WARN_CPUCL2]);
}

static DEVICE_ATTR_RO(ocp_cpu2_cap);

static ssize_t ocp_tpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[OCP_WARN_TPU]);
}

static DEVICE_ATTR_RO(ocp_tpu_cap);

static ssize_t ocp_gpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[OCP_WARN_GPU]);
}

static DEVICE_ATTR_RO(ocp_gpu_cap);

static ssize_t soft_ocp_cpu1_cap_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[OCP_WARN_CPUCL1]);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_cap);

static ssize_t soft_ocp_cpu2_cap_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[SOFT_OCP_WARN_CPUCL2]);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_cap);

static ssize_t soft_ocp_tpu_cap_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[SOFT_OCP_WARN_TPU]);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_cap);

static ssize_t soft_ocp_gpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_capacity(buf, bcl_dev->zone[SOFT_OCP_WARN_GPU]);
}

static DEVICE_ATTR_RO(soft_ocp_gpu_cap);

static ssize_t batoilo_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[BATOILO1]);
}

static DEVICE_ATTR_RO(batoilo_volt);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t batoilo2_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[BATOILO2]);
}

static DEVICE_ATTR_RO(batoilo2_volt);
#endif

static ssize_t vdroop2_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[UVLO2]);
}

static DEVICE_ATTR_RO(vdroop2_volt);

static ssize_t vdroop1_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[UVLO1]);
}

static DEVICE_ATTR_RO(vdroop1_volt);

static ssize_t smpl_warn_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[SMPL_WARN]);
}

static DEVICE_ATTR_RO(smpl_warn_volt);

static ssize_t ocp_cpu1_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[OCP_WARN_CPUCL1]);
}

static DEVICE_ATTR_RO(ocp_cpu1_volt);

static ssize_t ocp_cpu2_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[OCP_WARN_CPUCL2]);
}

static DEVICE_ATTR_RO(ocp_cpu2_volt);

static ssize_t ocp_tpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[OCP_WARN_TPU]);
}

static DEVICE_ATTR_RO(ocp_tpu_volt);

static ssize_t ocp_gpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[OCP_WARN_GPU]);
}

static DEVICE_ATTR_RO(ocp_gpu_volt);

static ssize_t soft_ocp_cpu1_volt_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[SOFT_OCP_WARN_CPUCL1]);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_volt);

static ssize_t soft_ocp_cpu2_volt_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[SOFT_OCP_WARN_CPUCL2]);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_volt);

static ssize_t soft_ocp_tpu_volt_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[SOFT_OCP_WARN_TPU]);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_volt);

static ssize_t soft_ocp_gpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_voltage(buf, bcl_dev->zone[SOFT_OCP_WARN_GPU]);
}

static DEVICE_ATTR_RO(soft_ocp_gpu_volt);

static ssize_t batoilo_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[BATOILO1]);
}

static DEVICE_ATTR_RO(batoilo_time);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t batoilo2_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[BATOILO2]);
}

static DEVICE_ATTR_RO(batoilo2_time);
#endif

static ssize_t vdroop2_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[UVLO2]);
}

static DEVICE_ATTR_RO(vdroop2_time);

static ssize_t vdroop1_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[UVLO1]);
}

static DEVICE_ATTR_RO(vdroop1_time);

static ssize_t smpl_warn_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[SMPL_WARN]);
}

static DEVICE_ATTR_RO(smpl_warn_time);

static ssize_t ocp_cpu1_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[OCP_WARN_CPUCL1]);
}

static DEVICE_ATTR_RO(ocp_cpu1_time);

static ssize_t ocp_cpu2_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[OCP_WARN_CPUCL2]);
}

static DEVICE_ATTR_RO(ocp_cpu2_time);

static ssize_t ocp_tpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[OCP_WARN_TPU]);
}

static DEVICE_ATTR_RO(ocp_tpu_time);

static ssize_t ocp_gpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[OCP_WARN_GPU]);
}

static DEVICE_ATTR_RO(ocp_gpu_time);

static ssize_t soft_ocp_cpu1_time_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[SOFT_OCP_WARN_CPUCL1]);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_time);

static ssize_t soft_ocp_cpu2_time_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[SOFT_OCP_WARN_CPUCL2]);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_time);

static ssize_t soft_ocp_tpu_time_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[SOFT_OCP_WARN_TPU]);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_time);

static ssize_t soft_ocp_gpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	return safe_emit_bcl_time(buf, bcl_dev->zone[SOFT_OCP_WARN_GPU]);
}

static DEVICE_ATTR_RO(soft_ocp_gpu_time);

static ssize_t db_settings_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size, enum MPMM_SOURCE src)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	int value;

	if (kstrtouint(buf, 16, &value) < 0)
		return -EINVAL;

	if (src != BIG && src != MID)
		return -EINVAL;

	google_set_db(bcl_dev, value, src);

	return size;
}

static ssize_t db_settings_show(struct device *dev, struct device_attribute *attr,
				char *buf, enum MPMM_SOURCE src)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if ((!bcl_dev->sysreg_cpucl0) || (src == LITTLE) || (src == MPMMEN))
		return -EIO;

	return sysfs_emit(buf, "%#x\n", google_get_db(bcl_dev, src));
}

static ssize_t mid_db_settings_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t size)
{
	return db_settings_store(dev, attr, buf, size, MID);
}

static ssize_t mid_db_settings_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return db_settings_show(dev, attr, buf, MID);
}

static DEVICE_ATTR_RW(mid_db_settings);

static ssize_t big_db_settings_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t size)
{
	return db_settings_store(dev, attr, buf, size, BIG);
}

static ssize_t big_db_settings_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return db_settings_show(dev, attr, buf, BIG);
}

static DEVICE_ATTR_RW(big_db_settings);

static ssize_t enable_mitigation_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(bcl_dev->enabled));
}

static ssize_t enable_mitigation_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret, i;
	unsigned int reg;

	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;

	if (smp_load_acquire(&bcl_dev->enabled) == value)
		return size;

	/* Kernel filesystem serializes sysfs store callbacks */
	smp_store_release(&bcl_dev->enabled, value);
	if (value) {
		bcl_dev->core_conf[SUBSYSTEM_TPU].clkdivstep |= 0x1;
		bcl_dev->core_conf[SUBSYSTEM_GPU].clkdivstep |= 0x1;
		bcl_dev->core_conf[SUBSYSTEM_AUR].clkdivstep |= 0x1;

		for (i = SUBSYSTEM_CPU0; i <= SUBSYSTEM_CPU2; i++) {
			ret = cpu_buff_read(bcl_dev, i, CPU_BUFF_CLKDIVSTEP, &reg);
			if (ret < 0)
				return ret;

			ret = cpu_buff_write(bcl_dev, i, CPU_BUFF_CLKDIVSTEP, reg | 0x1);
			if (ret < 0)
				return ret;
		}
		for (i = 0; i < TRIGGERED_SOURCE_MAX; i++)
			if (bcl_dev->zone[i] && i != BATOILO)
				enable_irq(bcl_dev->zone[i]->bcl_irq);
	} else {
		bcl_dev->core_conf[SUBSYSTEM_TPU].clkdivstep &= ~(1 << 0);
		bcl_dev->core_conf[SUBSYSTEM_GPU].clkdivstep &= ~(1 << 0);
		bcl_dev->core_conf[SUBSYSTEM_AUR].clkdivstep &= ~(1 << 0);

		for (i = SUBSYSTEM_CPU0; i <= SUBSYSTEM_CPU2; i++) {
			ret = cpu_buff_read(bcl_dev, i, CPU_BUFF_CLKDIVSTEP, &reg);
			if (ret < 0)
				return ret;

			ret = cpu_buff_write(bcl_dev, i, CPU_BUFF_CLKDIVSTEP, reg & ~(1 << 0));
			if (ret < 0)
				return ret;
		}
		for (i = 0; i < TRIGGERED_SOURCE_MAX; i++)
			if (bcl_dev->zone[i] && i != BATOILO)
				disable_irq(bcl_dev->zone[i]->bcl_irq);
	}
	return size;
}

static DEVICE_ATTR_RW(enable_mitigation);

static ssize_t enable_rffe_mitigation_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->rffe_mitigation_enable);
}

static ssize_t enable_rffe_mitigation_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;

	if (bcl_dev->rffe_mitigation_enable == value)
		return size;

	bcl_dev->rffe_mitigation_enable = value;
	return size;
}

static DEVICE_ATTR_RW(enable_rffe_mitigation);

static ssize_t main_offsrc1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%#x\n", bcl_dev->main_offsrc1);
}

static DEVICE_ATTR_RO(main_offsrc1);

static ssize_t main_offsrc2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%#x\n", bcl_dev->main_offsrc2);
}

static DEVICE_ATTR_RO(main_offsrc2);

static ssize_t sub_offsrc1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%#x\n", bcl_dev->sub_offsrc1);
}

static DEVICE_ATTR_RO(sub_offsrc1);

static ssize_t sub_offsrc2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%#x\n", bcl_dev->sub_offsrc2);
}

static DEVICE_ATTR_RO(sub_offsrc2);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t evt_cnt_uvlo1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%i\n", bcl_dev->evt_cnt.uvlo1);
}

static DEVICE_ATTR_RO(evt_cnt_uvlo1);

static ssize_t evt_cnt_uvlo2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%i\n", bcl_dev->evt_cnt.uvlo2);
}

static DEVICE_ATTR_RO(evt_cnt_uvlo2);

static ssize_t evt_cnt_batoilo1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%i\n", bcl_dev->evt_cnt.batoilo1);
}

static DEVICE_ATTR_RO(evt_cnt_batoilo1);

static ssize_t evt_cnt_batoilo2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%i\n", bcl_dev->evt_cnt.batoilo2);
}

static DEVICE_ATTR_RO(evt_cnt_batoilo2);

static ssize_t evt_cnt_latest_uvlo1_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%i\n", bcl_dev->evt_cnt_latest.uvlo1);
}

static DEVICE_ATTR_RO(evt_cnt_latest_uvlo1);

static ssize_t evt_cnt_latest_uvlo2_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%i\n", bcl_dev->evt_cnt_latest.uvlo2);
}

static DEVICE_ATTR_RO(evt_cnt_latest_uvlo2);

static ssize_t evt_cnt_latest_batoilo1_show(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%i\n", bcl_dev->evt_cnt_latest.batoilo1);
}

static DEVICE_ATTR_RO(evt_cnt_latest_batoilo1);

static ssize_t evt_cnt_latest_batoilo2_show(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%i\n", bcl_dev->evt_cnt_latest.batoilo2);
}

static DEVICE_ATTR_RO(evt_cnt_latest_batoilo2);
#endif

static ssize_t pwronsrc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%#x\n", bcl_dev->pwronsrc);
}

static DEVICE_ATTR_RO(pwronsrc);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t last_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (bcl_dev->ifpmic != MAX77779)
		return -ENODEV;

	return sysfs_emit(buf, "%#x\n", bcl_dev->last_current);
}
static DEVICE_ATTR_RO(last_current);

static ssize_t vimon_buff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	int idx, ret;
	uint16_t rdback;
	ssize_t count = 0;

	if (bcl_dev->ifpmic != MAX77779)
		return -ENODEV;

	ret = bcl_vimon_read(bcl_dev);
	if (ret < 0)
		return -ENODEV;

	for (idx = 0; idx < ret / VIMON_BYTES_PER_ENTRY; idx++) {
		rdback = bcl_dev->vimon_intf.data[idx];
		count += sysfs_emit_at(buf, count, "%#x\n", rdback);
	}

	return count;
}
static DEVICE_ATTR_RO(vimon_buff);
#endif

static ssize_t ready_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(bcl_dev->enabled));
}
static DEVICE_ATTR_RO(ready);

static ssize_t ifpmic_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%s\n", bcl_dev->ifpmic == MAX77779 ? "max77779" : "max77759");
}
static DEVICE_ATTR_RO(ifpmic);

static struct attribute *instr_attrs[] = {
	&dev_attr_mid_db_settings.attr,
	&dev_attr_big_db_settings.attr,
	&dev_attr_enable_mitigation.attr,
	&dev_attr_enable_rffe_mitigation.attr,
	&dev_attr_main_offsrc1.attr,
	&dev_attr_main_offsrc2.attr,
	&dev_attr_sub_offsrc1.attr,
	&dev_attr_sub_offsrc2.attr,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_evt_cnt_uvlo1.attr,
	&dev_attr_evt_cnt_uvlo2.attr,
	&dev_attr_evt_cnt_batoilo1.attr,
	&dev_attr_evt_cnt_batoilo2.attr,
	&dev_attr_evt_cnt_latest_uvlo1.attr,
	&dev_attr_evt_cnt_latest_uvlo2.attr,
	&dev_attr_evt_cnt_latest_batoilo1.attr,
	&dev_attr_evt_cnt_latest_batoilo2.attr,
#endif
	&dev_attr_pwronsrc.attr,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_last_current.attr,
	&dev_attr_vimon_buff.attr,
#endif
	&dev_attr_ready.attr,
	&dev_attr_ifpmic.attr,
	NULL,
};

static const struct attribute_group instr_group = {
	.attrs = instr_attrs,
	.name = "instruction",
};

int uvlo_reg_read(struct device *dev, enum IFPMIC ifpmic, int triggered, unsigned int *val)
{
	int ret;
	uint8_t reg, regval;

	if (!dev)
		return -ENODEV;

	if (ifpmic == MAX77779) {
		reg = (triggered == UVLO1) ? MAX77779_SYS_UVLO1_CNFG_0 : MAX77779_SYS_UVLO2_CNFG_0;
		ret = max77779_external_chg_reg_read(dev, reg, &regval);
		if (ret < 0)
			return -EINVAL;
		if (triggered == UVLO1)
			*val = _max77779_sys_uvlo1_cnfg_0_sys_uvlo1_get(regval);
		else
			*val = _max77779_sys_uvlo2_cnfg_0_sys_uvlo2_get(regval);
	} else {
		reg = (triggered == UVLO1) ? MAX77759_CHG_CNFG_15 : MAX77759_CHG_CNFG_16;
		ret = max77759_external_reg_read(dev, reg, &regval);
		if (ret < 0)
			return -EINVAL;
		if (triggered == UVLO1)
			*val = _chg_cnfg_15_sys_uvlo1_get(regval);
		else
			*val = _chg_cnfg_16_sys_uvlo2_get(regval);
	}
	return ret;
}

static int uvlo_reg_write(struct device *dev, uint8_t val,
			  enum IFPMIC ifpmic, int triggered)
{
	int ret;
	uint8_t reg, regval;

	if (!dev)
		return -ENODEV;

	if (ifpmic == MAX77779) {
		reg = (triggered == UVLO1) ? MAX77779_SYS_UVLO1_CNFG_0 : MAX77779_SYS_UVLO2_CNFG_0;
		ret = max77779_external_chg_reg_read(dev, reg, &regval);
		if (ret < 0)
			return -EINVAL;
		if (triggered == UVLO1)
			regval = _max77779_sys_uvlo1_cnfg_0_sys_uvlo1_set(regval, val);
		else
			regval = _max77779_sys_uvlo2_cnfg_0_sys_uvlo2_set(regval, val);
		ret = max77779_external_chg_reg_write(dev, reg, regval);
		if (ret < 0)
			return -EINVAL;
	} else {
		reg = (triggered == UVLO1) ? MAX77759_CHG_CNFG_15 : MAX77759_CHG_CNFG_16;
		ret = max77759_external_reg_read(dev, reg, &regval);
		if (ret < 0)
			return -EINVAL;
		if (triggered == UVLO1)
			regval = _chg_cnfg_15_sys_uvlo1_set(regval, val);
		else
			regval = _chg_cnfg_16_sys_uvlo2_set(regval, val);
		ret = max77759_external_reg_write(dev, reg, regval);
		if (ret < 0)
			return -EINVAL;
	}
	return ret;
}

static ssize_t uvlo1_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int uvlo1_lvl;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[UVLO1])
		return -EIO;
	if (!bcl_dev->intf_pmic_dev)
		return -EBUSY;
	uvlo_reg_read(bcl_dev->intf_pmic_dev, bcl_dev->ifpmic, UVLO1, &uvlo1_lvl);
	bcl_dev->zone[UVLO1]->bcl_lvl = VD_BATTERY_VOLTAGE - VD_STEP * uvlo1_lvl +
			VD_LOWER_LIMIT - THERMAL_HYST_LEVEL;
	return sysfs_emit(buf, "%dmV\n", VD_STEP * uvlo1_lvl + VD_LOWER_LIMIT);
}

static ssize_t uvlo1_lvl_store(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	uint8_t lvl;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[UVLO1])
		return -EIO;
	if (value < VD_LOWER_LIMIT || value > VD_UPPER_LIMIT) {
		dev_err(bcl_dev->device, "UVLO1 %d outside of range %d - %d mV.", value,
			VD_LOWER_LIMIT, VD_UPPER_LIMIT);
		return -EINVAL;
	}
	if (!bcl_dev->intf_pmic_dev)
		return -EIO;
	lvl = (value - VD_LOWER_LIMIT) / VD_STEP;
	disable_irq(bcl_dev->zone[UVLO1]->bcl_irq);
	ret = uvlo_reg_write(bcl_dev->intf_pmic_dev, lvl, bcl_dev->ifpmic, UVLO1);
	enable_irq(bcl_dev->zone[UVLO1]->bcl_irq);
	if (ret)
		return ret;
	bcl_dev->zone[UVLO1]->bcl_lvl = VD_BATTERY_VOLTAGE - value - THERMAL_HYST_LEVEL;
	if (bcl_dev->zone[UVLO1]->tz) {
		bcl_dev->zone[UVLO1]->tz->trips[0].temperature = VD_BATTERY_VOLTAGE - value;
		thermal_zone_device_update(bcl_dev->zone[UVLO1]->tz, THERMAL_EVENT_UNSPECIFIED);
	}
	return size;

}

static DEVICE_ATTR_RW(uvlo1_lvl);

static ssize_t uvlo2_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int uvlo2_lvl;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[UVLO2])
		return sysfs_emit(buf, "disabled\n");
	if (!bcl_dev->intf_pmic_dev)
		return -EBUSY;
	uvlo_reg_read(bcl_dev->intf_pmic_dev, bcl_dev->ifpmic, UVLO2, &uvlo2_lvl);
	bcl_dev->zone[UVLO1]->bcl_lvl = VD_BATTERY_VOLTAGE - VD_STEP * uvlo2_lvl +
			VD_LOWER_LIMIT - THERMAL_HYST_LEVEL;
	return sysfs_emit(buf, "%dmV\n", VD_STEP * uvlo2_lvl + VD_LOWER_LIMIT);
}

static ssize_t uvlo2_lvl_store(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	uint8_t lvl;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[UVLO2]) {
		dev_err(bcl_dev->device, "UVLO2 is disabled\n");
		return -EIO;
	}
	if (value < VD_LOWER_LIMIT || value > VD_UPPER_LIMIT) {
		dev_err(bcl_dev->device, "UVLO2 %d outside of range %d - %d mV.", value,
			VD_LOWER_LIMIT, VD_UPPER_LIMIT);
		return -EINVAL;
	}
	if (!bcl_dev->intf_pmic_dev)
		return -EIO;
	lvl = (value - VD_LOWER_LIMIT) / VD_STEP;
	disable_irq(bcl_dev->zone[UVLO2]->bcl_irq);
	ret = uvlo_reg_write(bcl_dev->intf_pmic_dev, lvl, bcl_dev->ifpmic, UVLO2);
	enable_irq(bcl_dev->zone[UVLO2]->bcl_irq);
	if (ret)
		return ret;
	bcl_dev->zone[UVLO2]->bcl_lvl = VD_BATTERY_VOLTAGE - value - THERMAL_HYST_LEVEL;
	if (bcl_dev->zone[UVLO2]->tz) {
		bcl_dev->zone[UVLO2]->tz->trips[0].temperature = VD_BATTERY_VOLTAGE - value;
		thermal_zone_device_update(bcl_dev->zone[UVLO2]->tz, THERMAL_EVENT_UNSPECIFIED);
	}
	return size;
}

static DEVICE_ATTR_RW(uvlo2_lvl);

int batoilo_reg_read(struct device *dev, enum IFPMIC ifpmic, int oilo, unsigned int *val)
{
	int ret;
	uint8_t reg, regval;

	if (!dev)
		return -ENODEV;

	if (ifpmic == MAX77779) {
		reg = (oilo == BATOILO1) ? MAX77779_BAT_OILO1_CNFG_0 : MAX77779_BAT_OILO2_CNFG_0;
		ret = max77779_external_chg_reg_read(dev, reg, &regval);
		if (ret < 0)
			return -EINVAL;
		if (oilo == BATOILO1)
			*val = _max77779_bat_oilo1_cnfg_0_bat_oilo1_get(regval);
		else
			*val = _max77779_bat_oilo2_cnfg_0_bat_oilo2_get(regval);
	} else {
		reg = MAX77759_CHG_CNFG_14;
		ret = max77759_external_reg_read(dev, reg, &regval);
		if (ret < 0)
			return -EINVAL;
		*val = _chg_cnfg_14_bat_oilo_get(regval);
	}
	return ret;
}

static int batoilo_reg_write(struct device *dev, uint8_t val,
			     enum IFPMIC ifpmic, int oilo)
{
	int ret;
	uint8_t reg, regval;

	if (!dev)
		return -ENODEV;

	if (ifpmic == MAX77779) {
		reg = (oilo == BATOILO1) ? MAX77779_BAT_OILO1_CNFG_0 : MAX77779_BAT_OILO2_CNFG_0;
		ret = max77779_external_chg_reg_read(dev, reg, &regval);
		if (ret < 0)
			return -EINVAL;
		if (oilo == BATOILO1)
			regval = _max77779_bat_oilo1_cnfg_0_bat_oilo1_set(regval, val);
		else
			regval = _max77779_bat_oilo2_cnfg_0_bat_oilo2_set(regval, val);
		ret = max77779_external_chg_reg_write(dev, reg, regval);
		if (ret < 0)
			return -EINVAL;
	} else {
		reg = MAX77759_CHG_CNFG_14;
		ret = max77759_external_reg_read(dev, reg, &regval);
		if (ret < 0)
			return -EINVAL;
		regval = _chg_cnfg_14_bat_oilo_set(regval, val);
		ret = max77759_external_reg_write(dev, reg, regval);
		if (ret < 0)
			return -EINVAL;
	}
	return ret;
}

static ssize_t batoilo_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int batoilo1_lvl, lvl;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[BATOILO1])
		return -EIO;
	if (!bcl_dev->intf_pmic_dev)
		return -EBUSY;
	batoilo_reg_read(bcl_dev->intf_pmic_dev, bcl_dev->ifpmic, BATOILO1, &lvl);
	batoilo1_lvl = BO_STEP * lvl + bcl_dev->batt_irq_conf1.batoilo_lower_limit;
	bcl_dev->zone[BATOILO1]->bcl_lvl = batoilo1_lvl;
	return sysfs_emit(buf, "%umA\n", batoilo1_lvl);
}

static ssize_t batoilo_lvl_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value, lvl;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[BATOILO1])
		return -EIO;
	if (value < bcl_dev->batt_irq_conf1.batoilo_lower_limit ||
	    value > bcl_dev->batt_irq_conf1.batoilo_upper_limit) {
		dev_err(bcl_dev->device, "BATOILO1 %d outside of range %d - %d mA.", value,
			bcl_dev->batt_irq_conf1.batoilo_lower_limit,
			bcl_dev->batt_irq_conf1.batoilo_upper_limit);
		return -EINVAL;
	}
	lvl = (value - bcl_dev->batt_irq_conf1.batoilo_lower_limit) / BO_STEP;
	ret = batoilo_reg_write(bcl_dev->intf_pmic_dev, lvl, bcl_dev->ifpmic, BATOILO1);
	if (ret)
		return ret;
	bcl_dev->zone[BATOILO1]->bcl_lvl = value - THERMAL_HYST_LEVEL;
	if (bcl_dev->zone[BATOILO1]->tz) {
		bcl_dev->zone[BATOILO1]->tz->trips[0].temperature = value;
		thermal_zone_device_update(bcl_dev->zone[BATOILO1]->tz, THERMAL_EVENT_UNSPECIFIED);
	}
	return size;
}

static DEVICE_ATTR_RW(batoilo_lvl);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t batoilo2_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int batoilo2_lvl, lvl;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[BATOILO2])
		return -EIO;
	if (!bcl_dev->intf_pmic_dev)
		return -EBUSY;
	batoilo_reg_read(bcl_dev->intf_pmic_dev, bcl_dev->ifpmic, BATOILO2, &lvl);
	batoilo2_lvl = BO_STEP * lvl + bcl_dev->batt_irq_conf2.batoilo_lower_limit;
	bcl_dev->zone[BATOILO2]->bcl_lvl = batoilo2_lvl;
	return sysfs_emit(buf, "%umA\n", batoilo2_lvl);
}

static ssize_t batoilo2_lvl_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value, lvl;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[BATOILO2])
		return -EIO;
	if (value < bcl_dev->batt_irq_conf2.batoilo_lower_limit ||
	    value > bcl_dev->batt_irq_conf2.batoilo_upper_limit) {
		dev_err(bcl_dev->device, "BATOILO2 %d outside of range %d - %d mA.", value,
			bcl_dev->batt_irq_conf2.batoilo_lower_limit,
			bcl_dev->batt_irq_conf2.batoilo_upper_limit);
		return -EINVAL;
	}
	lvl = (value - bcl_dev->batt_irq_conf2.batoilo_lower_limit) / BO_STEP;
	ret = batoilo_reg_write(bcl_dev->intf_pmic_dev, lvl, bcl_dev->ifpmic, BATOILO2);
	if (ret)
		return ret;
	bcl_dev->zone[BATOILO2]->bcl_lvl = value - THERMAL_HYST_LEVEL;
	if (bcl_dev->zone[BATOILO2]->tz) {
		bcl_dev->zone[BATOILO2]->tz->trips[0].temperature = value;
		thermal_zone_device_update(bcl_dev->zone[BATOILO2]->tz, THERMAL_EVENT_UNSPECIFIED);
	}
	return size;
}

static DEVICE_ATTR_RW(batoilo2_lvl);
#endif

static ssize_t smpl_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u8 value = 0;
	unsigned int smpl_warn_lvl;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->main_pmic_i2c) {
		return -EBUSY;
	}
	pmic_read(CORE_PMIC_MAIN, bcl_dev, SMPL_WARN_CTRL, &value);
	value >>= SMPL_WARN_SHIFT;

	smpl_warn_lvl = value * 100 + SMPL_LOWER_LIMIT;
	return sysfs_emit(buf, "%umV\n", smpl_warn_lvl);
}

static ssize_t smpl_lvl_store(struct device *dev,
			      struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int val;
	u8 value;
	int ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (!bcl_dev)
		return -EIO;
	if (val < SMPL_LOWER_LIMIT || val > SMPL_UPPER_LIMIT) {
		dev_err(bcl_dev->device, "SMPL_WARN LEVEL %d outside of range %d - %d mV.", val,
			SMPL_LOWER_LIMIT, SMPL_UPPER_LIMIT);
		return -EINVAL;
	}
	if (!bcl_dev->main_pmic_i2c) {
		dev_err(bcl_dev->device, "MAIN I2C not found\n");
		return -EIO;
	}
	if (pmic_read(CORE_PMIC_MAIN, bcl_dev, SMPL_WARN_CTRL, &value)) {
		dev_err(bcl_dev->device, "S2MPG1415 read 0x%x failed.", SMPL_WARN_CTRL);
		return -EBUSY;
	}
	disable_irq(bcl_dev->zone[SMPL_WARN]->bcl_irq);
	value &= ~SMPL_WARN_MASK;
	value |= ((val - SMPL_LOWER_LIMIT) / 100) << SMPL_WARN_SHIFT;
	if (pmic_write(CORE_PMIC_MAIN, bcl_dev, SMPL_WARN_CTRL, value)) {
		dev_err(bcl_dev->device, "i2c write error setting smpl_warn\n");
		enable_irq(bcl_dev->zone[SMPL_WARN]->bcl_irq);
		return ret;
	}
	bcl_dev->zone[SMPL_WARN]->bcl_lvl = SMPL_BATTERY_VOLTAGE - val - THERMAL_HYST_LEVEL;

	if (bcl_dev->zone[SMPL_WARN]->tz) {
		bcl_dev->zone[SMPL_WARN]->tz->trips[0].temperature = SMPL_BATTERY_VOLTAGE - val;
		thermal_zone_device_update(bcl_dev->zone[SMPL_WARN]->tz, THERMAL_EVENT_UNSPECIFIED);
	}
	enable_irq(bcl_dev->zone[SMPL_WARN]->bcl_irq);

	return size;

}

static DEVICE_ATTR_RW(smpl_lvl);

static int get_ocp_lvl(struct bcl_device *bcl_dev, u64 *val, u8 addr, u8 pmic, u8 mask, u16 limit,
		       u16 step)
{
	u8 value = 0;
	unsigned int ocp_warn_lvl;

	if (!bcl_dev)
		return -EIO;
	if (pmic_read(pmic, bcl_dev, addr, &value)) {
		dev_err(bcl_dev->device, "S2MPG1415 read 0x%x failed.", addr);
		return -EBUSY;
	}
	value &= mask;
	ocp_warn_lvl = limit - value * step;
	*val = ocp_warn_lvl;
	return 0;
}

static int set_ocp_lvl(struct bcl_device *bcl_dev, u64 val, u8 addr, u8 pmic, u8 mask,
		       u16 llimit, u16 ulimit, u16 step, u8 id)
{
	u8 value;
	int ret;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->zone[id])
		return 0;
	if (val < llimit || val > ulimit) {
		dev_err(bcl_dev->device, "OCP_WARN LEVEL %llu outside of range %d - %d mA.", val,
		       llimit, ulimit);
		return -EBUSY;
	}
	if (pmic_read(pmic, bcl_dev, addr, &value)) {
		dev_err(bcl_dev->device, "S2MPG1415 read 0x%x failed.", addr);
		return -EBUSY;
	}
	disable_irq(bcl_dev->zone[id]->bcl_irq);
	value &= ~(OCP_WARN_MASK) << OCP_WARN_LVL_SHIFT;
	value |= ((ulimit - val) / step) << OCP_WARN_LVL_SHIFT;
	ret = pmic_write(pmic, bcl_dev, addr, value);
	if (!ret)
		bcl_dev->zone[id]->bcl_lvl = val - THERMAL_HYST_LEVEL;
	if (bcl_dev->zone[id]->tz) {
		bcl_dev->zone[id]->tz->trips[0].temperature = val;
		thermal_zone_device_update(bcl_dev->zone[id]->tz, THERMAL_EVENT_UNSPECIFIED);
	}
	enable_irq(bcl_dev->zone[id]->bcl_irq);

	return ret;
}

static ssize_t ocp_cpu1_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, CPU1_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                CPU1_UPPER_LIMIT, CPU1_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%llumA\n", val);

}

static ssize_t ocp_cpu1_lvl_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (set_ocp_lvl(bcl_dev, value, CPU1_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                CPU1_LOWER_LIMIT, CPU1_UPPER_LIMIT, CPU1_STEP, OCP_WARN_CPUCL1) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_cpu1_lvl);

static ssize_t ocp_cpu2_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, CPU2_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                CPU2_UPPER_LIMIT, CPU2_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%llumA\n", val);

}

static ssize_t ocp_cpu2_lvl_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (set_ocp_lvl(bcl_dev, value, CPU2_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                CPU2_LOWER_LIMIT, CPU2_UPPER_LIMIT, CPU2_STEP, OCP_WARN_CPUCL2) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_cpu2_lvl);

static ssize_t ocp_tpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, TPU_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                TPU_UPPER_LIMIT, TPU_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%llumA\n", val);

}

static ssize_t ocp_tpu_lvl_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (set_ocp_lvl(bcl_dev, value, TPU_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                TPU_LOWER_LIMIT, TPU_UPPER_LIMIT, TPU_STEP, OCP_WARN_TPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_tpu_lvl);

static ssize_t ocp_gpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, GPU_OCP_WARN, CORE_PMIC_SUB, OCP_WARN_MASK, GPU_UPPER_LIMIT,
			GPU_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%llumA\n", val);

}

static ssize_t ocp_gpu_lvl_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (set_ocp_lvl(bcl_dev, value, GPU_OCP_WARN, CORE_PMIC_SUB, OCP_WARN_MASK,
	                GPU_LOWER_LIMIT, GPU_UPPER_LIMIT, GPU_STEP, OCP_WARN_GPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_gpu_lvl);

static ssize_t soft_ocp_cpu1_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, SOFT_CPU1_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                CPU1_UPPER_LIMIT, CPU1_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%llumA\n", val);
}

static ssize_t soft_ocp_cpu1_lvl_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (set_ocp_lvl(bcl_dev, value, SOFT_CPU1_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                CPU1_LOWER_LIMIT, CPU1_UPPER_LIMIT, CPU1_STEP, SOFT_OCP_WARN_CPUCL1) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_cpu1_lvl);

static ssize_t soft_ocp_cpu2_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, SOFT_CPU2_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                CPU2_UPPER_LIMIT, CPU2_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%llumA\n", val);
}

static ssize_t soft_ocp_cpu2_lvl_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (set_ocp_lvl(bcl_dev, value, SOFT_CPU2_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                CPU2_LOWER_LIMIT, CPU2_UPPER_LIMIT, CPU2_STEP, SOFT_OCP_WARN_CPUCL2) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_cpu2_lvl);

static ssize_t soft_ocp_tpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, SOFT_TPU_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                TPU_UPPER_LIMIT, TPU_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%llumA\n", val);
}

static ssize_t soft_ocp_tpu_lvl_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (set_ocp_lvl(bcl_dev, value, SOFT_TPU_OCP_WARN, CORE_PMIC_MAIN, OCP_WARN_MASK,
	                TPU_LOWER_LIMIT, TPU_UPPER_LIMIT, TPU_STEP, SOFT_OCP_WARN_TPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_tpu_lvl);

static ssize_t soft_ocp_gpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, SOFT_GPU_OCP_WARN, CORE_PMIC_SUB, OCP_WARN_MASK,
	                GPU_UPPER_LIMIT, GPU_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%llumA\n", val);
}

static ssize_t soft_ocp_gpu_lvl_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (set_ocp_lvl(bcl_dev, value, SOFT_GPU_OCP_WARN, CORE_PMIC_SUB, OCP_WARN_MASK,
	                GPU_LOWER_LIMIT, GPU_UPPER_LIMIT, GPU_STEP, SOFT_OCP_WARN_GPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_gpu_lvl);

static struct attribute *triggered_lvl_attrs[] = {
	&dev_attr_uvlo1_lvl.attr,
	&dev_attr_uvlo2_lvl.attr,
	&dev_attr_batoilo_lvl.attr,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_batoilo2_lvl.attr,
#endif
	&dev_attr_smpl_lvl.attr,
	&dev_attr_ocp_cpu1_lvl.attr,
	&dev_attr_ocp_cpu2_lvl.attr,
	&dev_attr_ocp_tpu_lvl.attr,
	&dev_attr_ocp_gpu_lvl.attr,
	&dev_attr_soft_ocp_cpu1_lvl.attr,
	&dev_attr_soft_ocp_cpu2_lvl.attr,
	&dev_attr_soft_ocp_tpu_lvl.attr,
	&dev_attr_soft_ocp_gpu_lvl.attr,
	NULL,
};

static const struct attribute_group triggered_lvl_group = {
	.attrs = triggered_lvl_attrs,
	.name = "triggered_lvl",
};

static ssize_t bat_ktimer_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->bat_ktimer_en) {
		return -EIO;
	}
	return sysfs_emit(buf, "%ums\n", bcl_dev->bat_ktimer);
}

static ssize_t bat_ktimer_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;
	if (value < BAT_KTIMER_LIMIT_MS)
		return -EINVAL;

	bcl_dev->bat_ktimer = value;
	return size;
}

static DEVICE_ATTR_RW(bat_ktimer);

static ssize_t bat_ktimer_enable_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bat_ktimer_en);
}

static ssize_t bat_ktimer_enable_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	ret = kstrtobool(buf, &value);
	if (ret)
		return -EINVAL;

	bcl_dev->bat_ktimer_en = value;
	return size;
}

static DEVICE_ATTR_RW(bat_ktimer_enable);

static struct attribute *ktimer_attrs[] = {
	&dev_attr_bat_ktimer.attr,
	&dev_attr_bat_ktimer_enable.attr,
	NULL,
};

static const struct attribute_group ktimer_group = {
	.attrs = ktimer_attrs,
	.name = "batfet_kernel_timer",
};

static ssize_t clk_div_show(struct bcl_device *bcl_dev, int idx, char *buf)
{
	unsigned int reg = 0;
	int ret;

	if (!bcl_dev)
		return -EIO;
	switch (idx) {
	case SUBSYSTEM_TPU:
	case SUBSYSTEM_GPU:
	case SUBSYSTEM_AUR:
		return sysfs_emit(buf, "0x%x\n", bcl_dev->core_conf[idx].clkdivstep);
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		ret = cpu_buff_read(bcl_dev, idx, CPU_BUFF_CLKDIVSTEP, &reg);
		if (ret < 0)
			return ret;
		break;
	}
	return sysfs_emit(buf, "0x%x\n", reg);
}

static ssize_t clk_stats_show(struct bcl_device *bcl_dev, int idx, char *buf)
{
	unsigned int reg = 0;
	int ret;

	if (!bcl_dev)
		return -EIO;
	switch (idx) {
	case SUBSYSTEM_TPU:
	case SUBSYSTEM_GPU:
	case SUBSYSTEM_AUR:
		return sysfs_emit(buf, "0x%x\n", bcl_dev->core_conf[idx].clk_stats);
	case SUBSYSTEM_CPU0:
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		ret = cpu_buff_read(bcl_dev, idx, CPU_BUFF_CLK_STATS, &reg);
		if (ret < 0)
			return ret;
		break;
	}
	return sysfs_emit(buf, "0x%x\n", reg);
}
static ssize_t cpu0_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, SUBSYSTEM_CPU0, buf);
}

static ssize_t clk_div_store(struct bcl_device *bcl_dev, int idx,
			     const char *buf, size_t size)
{
	unsigned int value;
	int ret;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	if (!bcl_dev)
		return -EIO;
	switch (idx) {
	case SUBSYSTEM_TPU:
	case SUBSYSTEM_GPU:
	case SUBSYSTEM_AUR:
		return size;
	case SUBSYSTEM_CPU0:
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		ret = cpu_buff_write(bcl_dev, idx, CPU_BUFF_CLKDIVSTEP, value);
		if (ret < 0)
			return ret;
		break;
	}
	return size;
}

static ssize_t cpu0_clk_div_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, SUBSYSTEM_CPU0, buf, size);
}

static DEVICE_ATTR_RW(cpu0_clk_div);

static ssize_t cpu1_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, SUBSYSTEM_CPU1, buf);
}

static ssize_t cpu1_clk_div_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, SUBSYSTEM_CPU1, buf, size);
}

static DEVICE_ATTR_RW(cpu1_clk_div);

static ssize_t cpu2_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, SUBSYSTEM_CPU2, buf);
}

static ssize_t cpu2_clk_div_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, SUBSYSTEM_CPU2, buf, size);
}

static DEVICE_ATTR_RW(cpu2_clk_div);

static ssize_t tpu_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, SUBSYSTEM_TPU, buf);
}

static ssize_t tpu_clk_div_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, SUBSYSTEM_TPU, buf, size);
}

static DEVICE_ATTR_RW(tpu_clk_div);

static ssize_t aur_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, SUBSYSTEM_AUR, buf);
}

static ssize_t aur_clk_div_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, SUBSYSTEM_AUR, buf, size);
}

static DEVICE_ATTR_RW(aur_clk_div);

static ssize_t gpu_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, SUBSYSTEM_GPU, buf);
}

static ssize_t gpu_clk_div_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, SUBSYSTEM_GPU, buf, size);
}

static DEVICE_ATTR_RW(gpu_clk_div);

static struct attribute *clock_div_attrs[] = {
	&dev_attr_cpu0_clk_div.attr,
	&dev_attr_cpu1_clk_div.attr,
	&dev_attr_cpu2_clk_div.attr,
	&dev_attr_tpu_clk_div.attr,
	&dev_attr_gpu_clk_div.attr,
	&dev_attr_aur_clk_div.attr,
	NULL,
};

static const struct attribute_group clock_div_group = {
	.attrs = clock_div_attrs,
	.name = "clock_div",
};

static ssize_t clk_ratio_show(struct bcl_device *bcl_dev, enum RATIO_SOURCE idx, char *buf,
			      int sub_idx)
{
	unsigned int reg = 0;
	int ret;

	if (!bcl_dev)
		return -EIO;

	switch (idx) {
	case TPU_HEAVY:
	case GPU_HEAVY:
		return sysfs_emit(buf, "0x%x\n", bcl_dev->core_conf[sub_idx].con_heavy);
	case TPU_LIGHT:
	case GPU_LIGHT:
		return sysfs_emit(buf, "0x%x\n", bcl_dev->core_conf[sub_idx].con_light);
	case CPU0_CON:
	case CPU1_LIGHT:
	case CPU2_LIGHT:
		ret = cpu_buff_read(bcl_dev, sub_idx, CPU_BUFF_CON_LIGHT, &reg);
		if (ret < 0)
			return ret;
		break;
	case CPU1_HEAVY:
	case CPU2_HEAVY:
		ret = cpu_buff_read(bcl_dev, sub_idx, CPU_BUFF_CON_HEAVY, &reg);
		if (ret < 0)
			return ret;
		break;
	}
	return sysfs_emit(buf, "0x%x\n", reg);
}

static ssize_t clk_ratio_store(struct bcl_device *bcl_dev, enum RATIO_SOURCE idx,
			       const char *buf, size_t size, int sub_idx)
{
	unsigned int value;
	int ret;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	if (!bcl_dev)
		return -EIO;

	switch (idx) {
	case TPU_HEAVY:
	case GPU_HEAVY:
		bcl_dev->core_conf[sub_idx].con_heavy = value;
		return size;
	case TPU_LIGHT:
	case GPU_LIGHT:
		bcl_dev->core_conf[sub_idx].con_light = value;
		return size;
	case CPU0_CON:
	case CPU1_LIGHT:
	case CPU2_LIGHT:
		ret = cpu_buff_write(bcl_dev, sub_idx, CPU_BUFF_CON_LIGHT, value);
		if (ret < 0)
			return ret;
		break;
	case CPU1_HEAVY:
	case CPU2_HEAVY:
		ret = cpu_buff_write(bcl_dev, sub_idx, CPU_BUFF_CON_HEAVY, value);
		if (ret < 0)
			return ret;
		break;
	}
	return size;
}

static ssize_t cpu0_clk_ratio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU0_CON, buf, SUBSYSTEM_CPU0);
}

static ssize_t cpu0_clk_ratio_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU0_CON, buf, size, SUBSYSTEM_CPU0);
}

static DEVICE_ATTR_RW(cpu0_clk_ratio);

static ssize_t cpu1_heavy_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU1_HEAVY, buf, SUBSYSTEM_CPU1);
}

static ssize_t cpu1_heavy_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU1_HEAVY, buf, size, SUBSYSTEM_CPU1);
}

static DEVICE_ATTR_RW(cpu1_heavy_clk_ratio);

static ssize_t cpu2_heavy_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU2_HEAVY, buf, SUBSYSTEM_CPU2);
}

static ssize_t cpu2_heavy_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU2_HEAVY, buf, size, SUBSYSTEM_CPU2);
}

static DEVICE_ATTR_RW(cpu2_heavy_clk_ratio);

static ssize_t tpu_heavy_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, TPU_HEAVY, buf, SUBSYSTEM_TPU);
}

static ssize_t tpu_heavy_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, TPU_HEAVY, buf, size, SUBSYSTEM_TPU);
}

static DEVICE_ATTR_RW(tpu_heavy_clk_ratio);

static ssize_t gpu_heavy_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, GPU_HEAVY, buf, SUBSYSTEM_GPU);
}

static ssize_t gpu_heavy_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, GPU_HEAVY, buf, size, SUBSYSTEM_GPU);
}

static DEVICE_ATTR_RW(gpu_heavy_clk_ratio);

static ssize_t cpu1_light_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU1_LIGHT, buf, SUBSYSTEM_CPU1);
}

static ssize_t cpu1_light_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU1_LIGHT, buf, size, SUBSYSTEM_CPU1);
}

static DEVICE_ATTR_RW(cpu1_light_clk_ratio);

static ssize_t cpu2_light_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU2_LIGHT, buf, SUBSYSTEM_CPU2);
}

static ssize_t cpu2_light_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU2_LIGHT, buf, size, SUBSYSTEM_CPU2);
}

static DEVICE_ATTR_RW(cpu2_light_clk_ratio);

static ssize_t tpu_light_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, TPU_LIGHT, buf, SUBSYSTEM_TPU);
}

static ssize_t tpu_light_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, TPU_LIGHT, buf, size, SUBSYSTEM_TPU);
}

static DEVICE_ATTR_RW(tpu_light_clk_ratio);

static ssize_t gpu_light_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, GPU_LIGHT, buf, SUBSYSTEM_GPU);
}

static ssize_t gpu_light_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, GPU_LIGHT, buf, size, SUBSYSTEM_GPU);
}

static DEVICE_ATTR_RW(gpu_light_clk_ratio);

static struct attribute *clock_ratio_attrs[] = {
	&dev_attr_cpu0_clk_ratio.attr,
	&dev_attr_cpu1_heavy_clk_ratio.attr,
	&dev_attr_cpu2_heavy_clk_ratio.attr,
	&dev_attr_tpu_heavy_clk_ratio.attr,
	&dev_attr_gpu_heavy_clk_ratio.attr,
	&dev_attr_cpu1_light_clk_ratio.attr,
	&dev_attr_cpu2_light_clk_ratio.attr,
	&dev_attr_tpu_light_clk_ratio.attr,
	&dev_attr_gpu_light_clk_ratio.attr,
	NULL,
};

static const struct attribute_group clock_ratio_group = {
	.attrs = clock_ratio_attrs,
	.name = "clock_ratio",
};

static ssize_t cpu0_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, SUBSYSTEM_CPU0, buf);
}

static DEVICE_ATTR_RO(cpu0_clk_stats);

static ssize_t cpu1_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, SUBSYSTEM_CPU1, buf);
}

static DEVICE_ATTR_RO(cpu1_clk_stats);

static ssize_t cpu2_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, SUBSYSTEM_CPU2, buf);
}

static DEVICE_ATTR_RO(cpu2_clk_stats);

static ssize_t tpu_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, SUBSYSTEM_TPU, buf);
}

static DEVICE_ATTR_RO(tpu_clk_stats);

static ssize_t aur_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, SUBSYSTEM_AUR, buf);
}

static DEVICE_ATTR_RO(aur_clk_stats);

static ssize_t gpu_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, SUBSYSTEM_GPU, buf);
}

static DEVICE_ATTR_RO(gpu_clk_stats);

static ssize_t last_triggered_cnt(struct bcl_zone *zone, char *buf, int mode)
{
	if (!zone)
		return -ENODEV;
	if (mode >= 0 && mode < MAX_MITIGATION_MODE)
		return sysfs_emit(buf, "%d\n",
				  atomic_read(&zone->last_triggered.triggered_cnt[mode]));
	return sysfs_emit(buf, "0\n");
}

static ssize_t last_triggered_time(struct bcl_zone *zone, char *buf, int mode)
{
	if (!zone)
		return -ENODEV;
	if (mode >= 0 && mode < MAX_MITIGATION_MODE)
		return sysfs_emit(buf, "%lld\n", zone->last_triggered.triggered_time[mode]);
	return sysfs_emit(buf, "0\n");
}

static ssize_t last_triggered_uvlo1_heavy_cnt_show(struct device *dev,
						      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[UVLO1], buf, HEAVY);
}

static DEVICE_ATTR_RO(last_triggered_uvlo1_heavy_cnt);

static ssize_t last_triggered_uvlo1_medium_cnt_show(struct device *dev,
							     struct device_attribute *attr,
							     char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[UVLO1], buf, MEDIUM);
}

static DEVICE_ATTR_RO(last_triggered_uvlo1_medium_cnt);

static ssize_t last_triggered_uvlo1_light_cnt_show(struct device *dev,
						       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[UVLO1], buf, LIGHT);
}

static DEVICE_ATTR_RO(last_triggered_uvlo1_light_cnt);

static ssize_t last_triggered_uvlo1_start_cnt_show(struct device *dev,
						   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[UVLO1], buf, START);
}

static DEVICE_ATTR_RO(last_triggered_uvlo1_start_cnt);

static ssize_t last_triggered_uvlo1_heavy_time_show(struct device *dev,
						       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[UVLO1], buf, HEAVY);
}

static DEVICE_ATTR_RO(last_triggered_uvlo1_heavy_time);

static ssize_t last_triggered_uvlo1_medium_time_show(struct device *dev,
							      struct device_attribute *attr,
							      char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[UVLO1], buf, MEDIUM);
}

static DEVICE_ATTR_RO(last_triggered_uvlo1_medium_time);

static ssize_t last_triggered_uvlo1_light_time_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[UVLO1], buf, LIGHT);
}

static DEVICE_ATTR_RO(last_triggered_uvlo1_light_time);

static ssize_t last_triggered_uvlo1_start_time_show(struct device *dev,
						    struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[UVLO1], buf, START);
}

static DEVICE_ATTR_RO(last_triggered_uvlo1_start_time);

static ssize_t last_triggered_uvlo2_heavy_cnt_show(struct device *dev,
						      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[UVLO2], buf, HEAVY);
}

static DEVICE_ATTR_RO(last_triggered_uvlo2_heavy_cnt);

static ssize_t last_triggered_uvlo2_medium_cnt_show(struct device *dev,
							     struct device_attribute *attr,
							     char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[UVLO2], buf, MEDIUM);
}

static DEVICE_ATTR_RO(last_triggered_uvlo2_medium_cnt);

static ssize_t last_triggered_uvlo2_light_cnt_show(struct device *dev,
						       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[UVLO2], buf, LIGHT);
}

static DEVICE_ATTR_RO(last_triggered_uvlo2_light_cnt);

static ssize_t last_triggered_uvlo2_start_cnt_show(struct device *dev,
						   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[UVLO2], buf, START);
}

static DEVICE_ATTR_RO(last_triggered_uvlo2_start_cnt);

static ssize_t last_triggered_uvlo2_heavy_time_show(struct device *dev,
						       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[UVLO2], buf, HEAVY);
}

static DEVICE_ATTR_RO(last_triggered_uvlo2_heavy_time);

static ssize_t last_triggered_uvlo2_medium_time_show(struct device *dev,
							      struct device_attribute *attr,
							      char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[UVLO2], buf, MEDIUM);
}

static DEVICE_ATTR_RO(last_triggered_uvlo2_medium_time);

static ssize_t last_triggered_uvlo2_light_time_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[UVLO2], buf, LIGHT);
}

static DEVICE_ATTR_RO(last_triggered_uvlo2_light_time);

static ssize_t last_triggered_uvlo2_start_time_show(struct device *dev,
						    struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[UVLO2], buf, START);
}

static DEVICE_ATTR_RO(last_triggered_uvlo2_start_time);

static ssize_t last_triggered_batoilo2_heavy_cnt_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[BATOILO2], buf, HEAVY);
}

static DEVICE_ATTR_RO(last_triggered_batoilo2_heavy_cnt);

static ssize_t last_triggered_batoilo2_medium_cnt_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[BATOILO2], buf, MEDIUM);
}

static DEVICE_ATTR_RO(last_triggered_batoilo2_medium_cnt);

static ssize_t last_triggered_batoilo2_light_cnt_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[BATOILO2], buf, LIGHT);
}

static DEVICE_ATTR_RO(last_triggered_batoilo2_light_cnt);

static ssize_t last_triggered_batoilo2_start_cnt_show(struct device *dev,
						      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[BATOILO2], buf, START);
}

static DEVICE_ATTR_RO(last_triggered_batoilo2_start_cnt);

static ssize_t last_triggered_batoilo2_heavy_time_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[BATOILO2], buf, HEAVY);
}

static DEVICE_ATTR_RO(last_triggered_batoilo2_heavy_time);

static ssize_t last_triggered_batoilo2_medium_time_show(struct device *dev,
								 struct device_attribute *attr,
								 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[BATOILO2], buf, MEDIUM);
}

static DEVICE_ATTR_RO(last_triggered_batoilo2_medium_time);

static ssize_t last_triggered_batoilo2_light_time_show(struct device *dev,
							   struct device_attribute *attr,
							   char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[BATOILO2], buf, LIGHT);
}

static DEVICE_ATTR_RO(last_triggered_batoilo2_light_time);

static ssize_t last_triggered_batoilo2_start_time_show(struct device *dev,
						       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[BATOILO2], buf, START);
}

static DEVICE_ATTR_RO(last_triggered_batoilo2_start_time);

static ssize_t last_triggered_batoilo_heavy_cnt_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[BATOILO], buf, HEAVY);
}

static DEVICE_ATTR_RO(last_triggered_batoilo_heavy_cnt);

static ssize_t last_triggered_batoilo_medium_cnt_show(struct device *dev,
							       struct device_attribute *attr,
							       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[BATOILO], buf, MEDIUM);
}

static DEVICE_ATTR_RO(last_triggered_batoilo_medium_cnt);

static ssize_t last_triggered_batoilo_light_cnt_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[BATOILO], buf, LIGHT);
}

static DEVICE_ATTR_RO(last_triggered_batoilo_light_cnt);

static ssize_t last_triggered_batoilo_start_cnt_show(struct device *dev,
						     struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_cnt(bcl_dev->zone[BATOILO], buf, START);
}

static DEVICE_ATTR_RO(last_triggered_batoilo_start_cnt);

static ssize_t last_triggered_batoilo_heavy_time_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[BATOILO], buf, HEAVY);
}

static DEVICE_ATTR_RO(last_triggered_batoilo_heavy_time);

static ssize_t last_triggered_batoilo_medium_time_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[BATOILO], buf, MEDIUM);
}

static DEVICE_ATTR_RO(last_triggered_batoilo_medium_time);

static ssize_t last_triggered_batoilo_light_time_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[BATOILO], buf, LIGHT);
}

static DEVICE_ATTR_RO(last_triggered_batoilo_light_time);

static ssize_t last_triggered_batoilo_start_time_show(struct device *dev,
						      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return last_triggered_time(bcl_dev->zone[BATOILO], buf, START);
}

static DEVICE_ATTR_RO(last_triggered_batoilo_start_time);

static struct attribute *clock_stats_attrs[] = {
	&dev_attr_cpu0_clk_stats.attr,
	&dev_attr_cpu1_clk_stats.attr,
	&dev_attr_cpu2_clk_stats.attr,
	&dev_attr_tpu_clk_stats.attr,
	&dev_attr_gpu_clk_stats.attr,
	&dev_attr_aur_clk_stats.attr,
	NULL,
};

static const struct attribute_group clock_stats_group = {
	.attrs = clock_stats_attrs,
	.name = "clock_stats",
};

static struct attribute *triggered_count_attrs[] = {
	&dev_attr_smpl_warn_count.attr,
	&dev_attr_ocp_cpu1_count.attr,
	&dev_attr_ocp_cpu2_count.attr,
	&dev_attr_ocp_tpu_count.attr,
	&dev_attr_ocp_gpu_count.attr,
	&dev_attr_soft_ocp_cpu1_count.attr,
	&dev_attr_soft_ocp_cpu2_count.attr,
	&dev_attr_soft_ocp_tpu_count.attr,
	&dev_attr_soft_ocp_gpu_count.attr,
	&dev_attr_vdroop1_count.attr,
	&dev_attr_vdroop2_count.attr,
	&dev_attr_batoilo_count.attr,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_batoilo2_count.attr,
#endif
	NULL,
};

static const struct attribute_group triggered_count_group = {
	.attrs = triggered_count_attrs,
	.name = "last_triggered_count",
};

static struct attribute *triggered_time_attrs[] = {
	&dev_attr_smpl_warn_time.attr,
	&dev_attr_ocp_cpu1_time.attr,
	&dev_attr_ocp_cpu2_time.attr,
	&dev_attr_ocp_tpu_time.attr,
	&dev_attr_ocp_gpu_time.attr,
	&dev_attr_soft_ocp_cpu1_time.attr,
	&dev_attr_soft_ocp_cpu2_time.attr,
	&dev_attr_soft_ocp_tpu_time.attr,
	&dev_attr_soft_ocp_gpu_time.attr,
	&dev_attr_vdroop1_time.attr,
	&dev_attr_vdroop2_time.attr,
	&dev_attr_batoilo_time.attr,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_batoilo2_time.attr,
#endif
	NULL,
};

static const struct attribute_group triggered_timestamp_group = {
	.attrs = triggered_time_attrs,
	.name = "last_triggered_timestamp",
};

static struct attribute *triggered_cap_attrs[] = {
	&dev_attr_smpl_warn_cap.attr,
	&dev_attr_ocp_cpu1_cap.attr,
	&dev_attr_ocp_cpu2_cap.attr,
	&dev_attr_ocp_tpu_cap.attr,
	&dev_attr_ocp_gpu_cap.attr,
	&dev_attr_soft_ocp_cpu1_cap.attr,
	&dev_attr_soft_ocp_cpu2_cap.attr,
	&dev_attr_soft_ocp_tpu_cap.attr,
	&dev_attr_soft_ocp_gpu_cap.attr,
	&dev_attr_vdroop1_cap.attr,
	&dev_attr_vdroop2_cap.attr,
	&dev_attr_batoilo_cap.attr,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_batoilo2_cap.attr,
#endif
	NULL,
};

static const struct attribute_group triggered_capacity_group = {
	.attrs = triggered_cap_attrs,
	.name = "last_triggered_capacity",
};

static struct attribute *triggered_volt_attrs[] = {
	&dev_attr_smpl_warn_volt.attr,
	&dev_attr_ocp_cpu1_volt.attr,
	&dev_attr_ocp_cpu2_volt.attr,
	&dev_attr_ocp_tpu_volt.attr,
	&dev_attr_ocp_gpu_volt.attr,
	&dev_attr_soft_ocp_cpu1_volt.attr,
	&dev_attr_soft_ocp_cpu2_volt.attr,
	&dev_attr_soft_ocp_tpu_volt.attr,
	&dev_attr_soft_ocp_gpu_volt.attr,
	&dev_attr_vdroop1_volt.attr,
	&dev_attr_vdroop2_volt.attr,
	&dev_attr_batoilo_volt.attr,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_batoilo2_volt.attr,
#endif
	NULL,
};

static const struct attribute_group triggered_voltage_group = {
	.attrs = triggered_volt_attrs,
	.name = "last_triggered_voltage",
};

static struct attribute *last_triggered_mode_attrs[] = {
	&dev_attr_last_triggered_uvlo1_start_cnt.attr,
	&dev_attr_last_triggered_uvlo1_start_time.attr,
	&dev_attr_last_triggered_uvlo1_light_cnt.attr,
	&dev_attr_last_triggered_uvlo1_light_time.attr,
	&dev_attr_last_triggered_uvlo1_medium_cnt.attr,
	&dev_attr_last_triggered_uvlo1_medium_time.attr,
	&dev_attr_last_triggered_uvlo1_heavy_cnt.attr,
	&dev_attr_last_triggered_uvlo1_heavy_time.attr,
	&dev_attr_last_triggered_uvlo2_start_cnt.attr,
	&dev_attr_last_triggered_uvlo2_start_time.attr,
	&dev_attr_last_triggered_uvlo2_light_cnt.attr,
	&dev_attr_last_triggered_uvlo2_light_time.attr,
	&dev_attr_last_triggered_uvlo2_medium_cnt.attr,
	&dev_attr_last_triggered_uvlo2_medium_time.attr,
	&dev_attr_last_triggered_uvlo2_heavy_cnt.attr,
	&dev_attr_last_triggered_uvlo2_heavy_time.attr,
	&dev_attr_last_triggered_batoilo_start_cnt.attr,
	&dev_attr_last_triggered_batoilo_start_time.attr,
	&dev_attr_last_triggered_batoilo_light_cnt.attr,
	&dev_attr_last_triggered_batoilo_light_time.attr,
	&dev_attr_last_triggered_batoilo_medium_cnt.attr,
	&dev_attr_last_triggered_batoilo_medium_time.attr,
	&dev_attr_last_triggered_batoilo_heavy_cnt.attr,
	&dev_attr_last_triggered_batoilo_heavy_time.attr,
	&dev_attr_last_triggered_batoilo2_start_cnt.attr,
	&dev_attr_last_triggered_batoilo2_start_time.attr,
	&dev_attr_last_triggered_batoilo2_light_cnt.attr,
	&dev_attr_last_triggered_batoilo2_light_time.attr,
	&dev_attr_last_triggered_batoilo2_medium_cnt.attr,
	&dev_attr_last_triggered_batoilo2_medium_time.attr,
	&dev_attr_last_triggered_batoilo2_heavy_cnt.attr,
	&dev_attr_last_triggered_batoilo2_heavy_time.attr,
	NULL,
};

static const struct attribute_group last_triggered_mode_group = {
	.attrs = last_triggered_mode_attrs,
	.name = "last_triggered_mode",
};

static ssize_t vdroop_flt_show(struct bcl_device *bcl_dev, int idx, char *buf)
{
	unsigned int reg = 0;
	int ret;

	if (!bcl_dev)
		return -EIO;
	switch (idx) {
	case SUBSYSTEM_TPU:
	case SUBSYSTEM_GPU:
		return sysfs_emit(buf, "0x%x\n", bcl_dev->core_conf[idx].vdroop_flt);
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		ret = cpu_buff_read(bcl_dev, idx, CPU_BUFF_VDROOP_FLT, &reg);
		if (ret < 0)
			return ret;
		break;
	}
	return sysfs_emit(buf, "0x%x\n", reg);
}

static ssize_t vdroop_flt_store(struct bcl_device *bcl_dev, int idx,
				const char *buf, size_t size)
{
	unsigned int value;
	int ret;

	if (sscanf(buf, "0x%x", &value) != 1)
		return -EINVAL;

	if (!bcl_dev)
		return -EIO;
	switch (idx) {
	case SUBSYSTEM_TPU:
	case SUBSYSTEM_GPU:
		bcl_dev->core_conf[idx].vdroop_flt = value;
		return size;
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		ret = cpu_buff_write(bcl_dev, idx, CPU_BUFF_VDROOP_FLT, value);
		if (ret < 0)
			return ret;
	}
	return size;
}

static ssize_t cpu1_vdroop_flt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_show(bcl_dev, SUBSYSTEM_CPU1, buf);
}

static ssize_t cpu1_vdroop_flt_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_store(bcl_dev, SUBSYSTEM_CPU1, buf, size);
}

static DEVICE_ATTR_RW(cpu1_vdroop_flt);

static ssize_t cpu2_vdroop_flt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_show(bcl_dev, SUBSYSTEM_CPU2, buf);
}

static ssize_t cpu2_vdroop_flt_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_store(bcl_dev, SUBSYSTEM_CPU2, buf, size);
}

static DEVICE_ATTR_RW(cpu2_vdroop_flt);

static ssize_t tpu_vdroop_flt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_show(bcl_dev, SUBSYSTEM_TPU, buf);
}

static ssize_t tpu_vdroop_flt_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_store(bcl_dev, SUBSYSTEM_TPU, buf, size);
}

static DEVICE_ATTR_RW(tpu_vdroop_flt);

static ssize_t gpu_vdroop_flt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_show(bcl_dev, SUBSYSTEM_GPU, buf);
}

static ssize_t gpu_vdroop_flt_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_store(bcl_dev, SUBSYSTEM_GPU, buf, size);
}

static DEVICE_ATTR_RW(gpu_vdroop_flt);

static struct attribute *vdroop_flt_attrs[] = {
	&dev_attr_cpu1_vdroop_flt.attr,
	&dev_attr_cpu2_vdroop_flt.attr,
	&dev_attr_tpu_vdroop_flt.attr,
	&dev_attr_gpu_vdroop_flt.attr,
	NULL,
};

static const struct attribute_group vdroop_flt_group = {
	.attrs = vdroop_flt_attrs,
	.name = "vdroop_flt",
};

static ssize_t main_pwrwarn_threshold_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	int ret, idx;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif

	ret = sscanf(attr->attr.name, "main_pwrwarn_threshold%d", &idx);
	if (ret != 1)
		return -EINVAL;
	if (idx >= METER_CHANNEL_MAX || idx < 0)
		return -EINVAL;

	return sysfs_emit(buf, "%d=%lld\n", bcl_dev->main_setting[idx], bcl_dev->main_limit[idx]);
}

static ssize_t main_pwrwarn_threshold_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	int ret, idx, value;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;
	ret = sscanf(attr->attr.name, "main_pwrwarn_threshold%d", &idx);
	if (ret != 1)
		return -EINVAL;
	if (idx >= METER_CHANNEL_MAX || idx < 0)
		return -EINVAL;

	bcl_dev->main_setting[idx] = value;
	bcl_dev->main_limit[idx] = settings_to_current(bcl_dev, CORE_PMIC_MAIN, idx,
	                                               value << LPF_CURRENT_SHIFT);
	meter_write(CORE_PMIC_MAIN, bcl_dev, MAIN_METER_PWR_WARN0 + idx, value);

	return size;
}

static ssize_t sub_pwrwarn_threshold_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	int ret, idx;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif

	ret = sscanf(attr->attr.name, "sub_pwrwarn_threshold%d", &idx);
	if (ret != 1)
		return -EINVAL;
	if (idx >= METER_CHANNEL_MAX || idx < 0)
		return -EINVAL;

	return sysfs_emit(buf, "%d=%lld\n", bcl_dev->sub_setting[idx], bcl_dev->sub_limit[idx]);
}

static ssize_t sub_pwrwarn_threshold_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	int ret, idx, value;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;
	ret = sscanf(attr->attr.name, "sub_pwrwarn_threshold%d", &idx);
	if (ret != 1)
		return -EINVAL;
	if (idx >= METER_CHANNEL_MAX || idx < 0)
		return -EINVAL;

	bcl_dev->sub_setting[idx] = value;
	bcl_dev->sub_limit[idx] = settings_to_current(bcl_dev, CORE_PMIC_SUB, idx,
	                                              value << LPF_CURRENT_SHIFT);
	meter_write(CORE_PMIC_SUB, bcl_dev, SUB_METER_PWR_WARN0 + idx, value);

	return size;
}

#define DEVICE_PWRWARN_ATTR(_name, _num) \
	struct device_attribute attr_##_name##_num = \
		__ATTR(_name##_num, 0644, _name##_show, _name##_store)

static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 0);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 1);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 2);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 3);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 4);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 5);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 6);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 7);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 8);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 9);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 10);
static DEVICE_PWRWARN_ATTR(main_pwrwarn_threshold, 11);

static struct attribute *main_pwrwarn_attrs[] = {
	&attr_main_pwrwarn_threshold0.attr,
	&attr_main_pwrwarn_threshold1.attr,
	&attr_main_pwrwarn_threshold2.attr,
	&attr_main_pwrwarn_threshold3.attr,
	&attr_main_pwrwarn_threshold4.attr,
	&attr_main_pwrwarn_threshold5.attr,
	&attr_main_pwrwarn_threshold6.attr,
	&attr_main_pwrwarn_threshold7.attr,
	&attr_main_pwrwarn_threshold8.attr,
	&attr_main_pwrwarn_threshold9.attr,
	&attr_main_pwrwarn_threshold10.attr,
	&attr_main_pwrwarn_threshold11.attr,
	NULL,
};

static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 0);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 1);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 2);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 3);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 4);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 5);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 6);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 7);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 8);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 9);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 10);
static DEVICE_PWRWARN_ATTR(sub_pwrwarn_threshold, 11);

static struct attribute *sub_pwrwarn_attrs[] = {
	&attr_sub_pwrwarn_threshold0.attr,
	&attr_sub_pwrwarn_threshold1.attr,
	&attr_sub_pwrwarn_threshold2.attr,
	&attr_sub_pwrwarn_threshold3.attr,
	&attr_sub_pwrwarn_threshold4.attr,
	&attr_sub_pwrwarn_threshold5.attr,
	&attr_sub_pwrwarn_threshold6.attr,
	&attr_sub_pwrwarn_threshold7.attr,
	&attr_sub_pwrwarn_threshold8.attr,
	&attr_sub_pwrwarn_threshold9.attr,
	&attr_sub_pwrwarn_threshold10.attr,
	&attr_sub_pwrwarn_threshold11.attr,
	NULL,
};

static ssize_t qos_show(struct bcl_device *bcl_dev, int idx, char *buf)
{
	struct bcl_zone *zone;

	if (!bcl_dev)
		return -EIO;
	zone = bcl_dev->zone[idx];
	if ((!zone) || (!zone->bcl_qos))
		return -EIO;

	return sysfs_emit(buf, "CPU0,CPU1,CPU2,GPU,TPU\n%d,%d,%d,%d,%d\n",
			  zone->bcl_qos->cpu0_limit,
			  zone->bcl_qos->cpu1_limit,
			  zone->bcl_qos->cpu2_limit,
			  zone->bcl_qos->gpu_limit,
			  zone->bcl_qos->tpu_limit);
}

static ssize_t qos_store(struct bcl_device *bcl_dev, int idx, const char *buf, size_t size)
{
	unsigned int cpu0, cpu1, cpu2, gpu, tpu;
	struct bcl_zone *zone;

	if (sscanf(buf, "%d,%d,%d,%d,%d", &cpu0, &cpu1, &cpu2, &gpu, &tpu) != 5)
		return -EINVAL;
	if (!bcl_dev)
		return -EIO;
	zone = bcl_dev->zone[idx];
	if ((!zone) || (!zone->bcl_qos))
		return -EIO;
	zone->bcl_qos->cpu0_limit = cpu0;
	zone->bcl_qos->cpu1_limit = cpu1;
	zone->bcl_qos->cpu2_limit = cpu2;
	zone->bcl_qos->gpu_limit = gpu;
	zone->bcl_qos->tpu_limit = tpu;

	return size;
}

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t qos_batoilo2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, BATOILO2, buf);
}

static ssize_t qos_batoilo2_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, BATOILO2, buf, size);
}
#endif

static ssize_t qos_batoilo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, BATOILO, buf);
}

static ssize_t qos_batoilo_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, BATOILO, buf, size);
}

static ssize_t qos_vdroop1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, UVLO1, buf);
}

static ssize_t qos_vdroop1_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, UVLO1, buf, size);
}

static ssize_t qos_vdroop2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, UVLO2, buf);
}

static ssize_t qos_vdroop2_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, UVLO2, buf, size);
}

static ssize_t qos_smpl_warn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, SMPL_WARN, buf);
}

static ssize_t qos_smpl_warn_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, SMPL_WARN, buf, size);
}

static ssize_t qos_ocp_cpu2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, OCP_WARN_CPUCL2, buf);
}

static ssize_t qos_ocp_cpu2_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, OCP_WARN_CPUCL2, buf, size);
}

static ssize_t qos_ocp_cpu1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, OCP_WARN_CPUCL1, buf);
}

static ssize_t qos_ocp_cpu1_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, OCP_WARN_CPUCL1, buf, size);
}

static ssize_t qos_ocp_tpu_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, OCP_WARN_TPU, buf);
}

static ssize_t qos_ocp_tpu_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, OCP_WARN_TPU, buf, size);
}

static ssize_t qos_ocp_gpu_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_show(bcl_dev, OCP_WARN_GPU, buf);
}

static ssize_t qos_ocp_gpu_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return qos_store(bcl_dev, OCP_WARN_GPU, buf, size);
}

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static DEVICE_ATTR_RW(qos_batoilo2);
#endif
static DEVICE_ATTR_RW(qos_batoilo);
static DEVICE_ATTR_RW(qos_vdroop1);
static DEVICE_ATTR_RW(qos_vdroop2);
static DEVICE_ATTR_RW(qos_smpl_warn);
static DEVICE_ATTR_RW(qos_ocp_cpu2);
static DEVICE_ATTR_RW(qos_ocp_cpu1);
static DEVICE_ATTR_RW(qos_ocp_gpu);
static DEVICE_ATTR_RW(qos_ocp_tpu);

static struct attribute *qos_attrs[] = {
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_qos_batoilo2.attr,
#endif
	&dev_attr_qos_batoilo.attr,
	&dev_attr_qos_vdroop1.attr,
	&dev_attr_qos_vdroop2.attr,
	&dev_attr_qos_smpl_warn.attr,
	&dev_attr_qos_ocp_cpu2.attr,
	&dev_attr_qos_ocp_cpu1.attr,
	&dev_attr_qos_ocp_gpu.attr,
	&dev_attr_qos_ocp_tpu.attr,
	NULL,
};

static const struct attribute_group main_pwrwarn_group = {
	.attrs = main_pwrwarn_attrs,
	.name = "main_pwrwarn",
};

static const struct attribute_group sub_pwrwarn_group = {
	.attrs = sub_pwrwarn_attrs,
	.name = "sub_pwrwarn",
};

static ssize_t less_than_5ms_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int irq_count, batt_idx, pwrwarn_idx;
	ssize_t count = 0;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif

	for (batt_idx = 0; batt_idx < MAX_BCL_BATT_IRQ; batt_idx++) {
		for (pwrwarn_idx = 0; pwrwarn_idx < MAX_CONCURRENT_PWRWARN_IRQ; pwrwarn_idx++) {
			irq_count = atomic_read(&bcl_dev->ifpmic_irq_bins[batt_idx][pwrwarn_idx]
						.lt_5ms_count);
			count += scnprintf(buf + count, PAGE_SIZE - count,
						"%s + %s: %i\n",
						batt_irq_names[batt_idx],
						concurrent_pwrwarn_irq_names[pwrwarn_idx],
						irq_count);
		}
	}
	for (pwrwarn_idx = 0; pwrwarn_idx < METER_CHANNEL_MAX; pwrwarn_idx++) {
		irq_count = atomic_read(&bcl_dev->pwrwarn_main_irq_bins[pwrwarn_idx].lt_5ms_count);
		count += scnprintf(buf + count, PAGE_SIZE - count,
					"main CH%d[%s]: %i\n",
					pwrwarn_idx,
					bcl_dev->main_rail_names[pwrwarn_idx],
					irq_count);
	}
	for (pwrwarn_idx = 0; pwrwarn_idx < METER_CHANNEL_MAX; pwrwarn_idx++) {
		irq_count = atomic_read(&bcl_dev->pwrwarn_sub_irq_bins[pwrwarn_idx].lt_5ms_count);
		count += scnprintf(buf + count, PAGE_SIZE - count,
					"sub CH%d[%s]: %i\n",
					pwrwarn_idx,
					bcl_dev->sub_rail_names[pwrwarn_idx],
					irq_count);
	}
	return count;
}

static DEVICE_ATTR_RO(less_than_5ms_count);

static ssize_t between_5ms_to_10ms_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int irq_count, batt_idx, pwrwarn_idx;
	ssize_t count = 0;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif

	for (batt_idx = 0; batt_idx < MAX_BCL_BATT_IRQ; batt_idx++) {
		for (pwrwarn_idx = 0; pwrwarn_idx < MAX_CONCURRENT_PWRWARN_IRQ; pwrwarn_idx++) {
			irq_count = atomic_read(&bcl_dev->ifpmic_irq_bins[batt_idx][pwrwarn_idx]
						.bt_5ms_10ms_count);
			count += scnprintf(buf + count, PAGE_SIZE - count,
						"%s + %s: %i\n",
						batt_irq_names[batt_idx],
						concurrent_pwrwarn_irq_names[pwrwarn_idx],
						irq_count);
		}
	}
	for (pwrwarn_idx = 0; pwrwarn_idx < METER_CHANNEL_MAX; pwrwarn_idx++) {
		irq_count = atomic_read(&bcl_dev->pwrwarn_main_irq_bins[pwrwarn_idx]
					.bt_5ms_10ms_count);
		count += scnprintf(buf + count, PAGE_SIZE - count,
					"main CH%d[%s]: %i\n",
					pwrwarn_idx,
					bcl_dev->main_rail_names[pwrwarn_idx],
					irq_count);
	}
	for (pwrwarn_idx = 0; pwrwarn_idx < METER_CHANNEL_MAX; pwrwarn_idx++) {
		irq_count = atomic_read(&bcl_dev->pwrwarn_sub_irq_bins[pwrwarn_idx]
					.bt_5ms_10ms_count);
		count += scnprintf(buf + count, PAGE_SIZE - count,
					"sub CH%d[%s]: %i\n",
					pwrwarn_idx,
					bcl_dev->sub_rail_names[pwrwarn_idx],
					irq_count);
	}
	return count;
}

static DEVICE_ATTR_RO(between_5ms_to_10ms_count);

static ssize_t greater_than_10ms_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int irq_count, batt_idx, pwrwarn_idx;
	ssize_t count = 0;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif

	for (batt_idx = 0; batt_idx < MAX_BCL_BATT_IRQ; batt_idx++) {
		for (pwrwarn_idx = 0; pwrwarn_idx < MAX_CONCURRENT_PWRWARN_IRQ; pwrwarn_idx++) {
			irq_count = atomic_read(&bcl_dev->ifpmic_irq_bins[batt_idx][pwrwarn_idx]
						.gt_10ms_count);
			count += scnprintf(buf + count, PAGE_SIZE - count,
						"%s + %s: %i\n",
						batt_irq_names[batt_idx],
						concurrent_pwrwarn_irq_names[pwrwarn_idx],
						irq_count);
		}
	}
	for (pwrwarn_idx = 0; pwrwarn_idx < METER_CHANNEL_MAX; pwrwarn_idx++) {
		irq_count = atomic_read(&bcl_dev->pwrwarn_main_irq_bins[pwrwarn_idx].gt_10ms_count);
		count += scnprintf(buf + count, PAGE_SIZE - count,
					"main CH%d[%s]: %i\n",
					pwrwarn_idx,
					bcl_dev->main_rail_names[pwrwarn_idx],
					irq_count);
	}
	for (pwrwarn_idx = 0; pwrwarn_idx < METER_CHANNEL_MAX; pwrwarn_idx++) {
		irq_count = atomic_read(&bcl_dev->pwrwarn_sub_irq_bins[pwrwarn_idx].gt_10ms_count);
		count += scnprintf(buf + count, PAGE_SIZE - count,
					"sub CH%d[%s]: %i\n",
					pwrwarn_idx,
					bcl_dev->sub_rail_names[pwrwarn_idx],
					irq_count);
	}
	return count;
}

static DEVICE_ATTR_RO(greater_than_10ms_count);

static struct attribute *irq_dur_cnt_attrs[] = {
	&dev_attr_less_than_5ms_count.attr,
	&dev_attr_between_5ms_to_10ms_count.attr,
	&dev_attr_greater_than_10ms_count.attr,
	NULL,
};

static ssize_t disabled_store(struct bcl_zone *zone, bool disabled, size_t size)
{
	if (disabled && !zone->disabled) {
		zone->disabled = true;
		disable_irq(zone->bcl_irq);
	} else if (!disabled && zone->disabled) {
		zone->disabled = false;
		enable_irq(zone->bcl_irq);
	}
	return size;
}

static ssize_t uvlo1_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[UVLO1])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[UVLO1]->disabled);
}

static ssize_t uvlo1_disabled_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[UVLO1])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[UVLO1], value, size);
}

static ssize_t uvlo2_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[UVLO2])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[UVLO2]->disabled);
}

static ssize_t uvlo2_disabled_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[UVLO2])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[UVLO2], value, size);
}

static ssize_t batoilo_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[BATOILO])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[BATOILO]->disabled);
}

static ssize_t batoilo_disabled_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[BATOILO])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[BATOILO], value, size);
}

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static ssize_t batoilo2_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[BATOILO2])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[BATOILO2]->disabled);
}

static ssize_t batoilo2_disabled_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[BATOILO2])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[BATOILO2], value, size);
}
#endif

static ssize_t smpl_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[SMPL_WARN])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[SMPL_WARN]->disabled);
}

static ssize_t smpl_disabled_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[SMPL_WARN])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[SMPL_WARN], value, size);
}

static ssize_t ocp_cpu1_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[OCP_WARN_CPUCL1])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[OCP_WARN_CPUCL1]->disabled);
}

static ssize_t ocp_cpu1_disabled_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[OCP_WARN_CPUCL1])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[OCP_WARN_CPUCL1], value, size);
}

static ssize_t ocp_cpu2_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[OCP_WARN_CPUCL2])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[OCP_WARN_CPUCL2]->disabled);
}

static ssize_t ocp_cpu2_disabled_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[OCP_WARN_CPUCL2])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[OCP_WARN_CPUCL2], value, size);
}

static ssize_t ocp_tpu_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[OCP_WARN_TPU])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[OCP_WARN_TPU]->disabled);
}

static ssize_t ocp_tpu_disabled_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[OCP_WARN_TPU])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[OCP_WARN_TPU], value, size);
}

static ssize_t ocp_gpu_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[OCP_WARN_GPU])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[OCP_WARN_GPU]->disabled);
}

static ssize_t ocp_gpu_disabled_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[OCP_WARN_GPU])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[OCP_WARN_GPU], value, size);
}

static ssize_t soft_ocp_cpu1_disabled_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[SOFT_OCP_WARN_CPUCL1])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[SOFT_OCP_WARN_CPUCL1]->disabled);
}

static ssize_t soft_ocp_cpu1_disabled_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[SOFT_OCP_WARN_CPUCL1])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[SOFT_OCP_WARN_CPUCL1], value, size);
}

static ssize_t soft_ocp_cpu2_disabled_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[SOFT_OCP_WARN_CPUCL2])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[SOFT_OCP_WARN_CPUCL2]->disabled);
}

static ssize_t soft_ocp_cpu2_disabled_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[SOFT_OCP_WARN_CPUCL2])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[SOFT_OCP_WARN_CPUCL2], value, size);
}

static ssize_t soft_ocp_tpu_disabled_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[SOFT_OCP_WARN_TPU])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[SOFT_OCP_WARN_TPU]->disabled);
}

static ssize_t soft_ocp_tpu_disabled_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[SOFT_OCP_WARN_TPU])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[SOFT_OCP_WARN_TPU], value, size);
}

static ssize_t soft_ocp_gpu_disabled_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[SOFT_OCP_WARN_GPU])
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", bcl_dev->zone[SOFT_OCP_WARN_GPU]->disabled);
}

static ssize_t soft_ocp_gpu_disabled_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->zone[SOFT_OCP_WARN_GPU])
		return -ENODEV;
	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	return disabled_store(bcl_dev->zone[SOFT_OCP_WARN_GPU], value, size);
}

static DEVICE_ATTR_RW(uvlo1_disabled);
static DEVICE_ATTR_RW(uvlo2_disabled);
static DEVICE_ATTR_RW(batoilo_disabled);
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static DEVICE_ATTR_RW(batoilo2_disabled);
#endif
static DEVICE_ATTR_RW(smpl_disabled);
static DEVICE_ATTR_RW(ocp_cpu1_disabled);
static DEVICE_ATTR_RW(ocp_cpu2_disabled);
static DEVICE_ATTR_RW(ocp_tpu_disabled);
static DEVICE_ATTR_RW(ocp_gpu_disabled);
static DEVICE_ATTR_RW(soft_ocp_cpu1_disabled);
static DEVICE_ATTR_RW(soft_ocp_cpu2_disabled);
static DEVICE_ATTR_RW(soft_ocp_tpu_disabled);
static DEVICE_ATTR_RW(soft_ocp_gpu_disabled);

static struct attribute *irq_config_attrs[] = {
	&dev_attr_uvlo1_disabled.attr,
	&dev_attr_uvlo2_disabled.attr,
	&dev_attr_batoilo_disabled.attr,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	&dev_attr_batoilo2_disabled.attr,
#endif
	&dev_attr_smpl_disabled.attr,
	&dev_attr_ocp_cpu1_disabled.attr,
	&dev_attr_ocp_cpu2_disabled.attr,
	&dev_attr_ocp_tpu_disabled.attr,
	&dev_attr_ocp_gpu_disabled.attr,
	&dev_attr_soft_ocp_cpu1_disabled.attr,
	&dev_attr_soft_ocp_cpu2_disabled.attr,
	&dev_attr_soft_ocp_tpu_disabled.attr,
	&dev_attr_soft_ocp_gpu_disabled.attr,
	NULL,
};

int get_final_mitigation_module_ids(struct bcl_device *bcl_dev) {
	int mitigation_module_ids = atomic_read(&bcl_dev->mitigation_module_ids);
	int w = hweight32(mitigation_module_ids);

	if (w >= HEAVY_MITIGATION_MODULES_NUM || w == 0)
		mitigation_module_ids |= bcl_dev->non_monitored_mitigation_module_ids;

	return mitigation_module_ids;
}

static ssize_t uvlo1_triggered_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[UVLO1])
		return -ENODEV;
	return sysfs_emit(buf, "%d_%d\n", bcl_dev->zone[UVLO1]->current_state,
					  get_final_mitigation_module_ids(bcl_dev));
}

static ssize_t uvlo2_triggered_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[UVLO2])
		return -ENODEV;
	return sysfs_emit(buf, "%d_%d\n", bcl_dev->zone[UVLO2]->current_state,
					  get_final_mitigation_module_ids(bcl_dev));
}

static ssize_t oilo1_triggered_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[BATOILO])
		return -ENODEV;
	return sysfs_emit(buf, "%d_%d\n", bcl_dev->zone[BATOILO]->current_state,
					  get_final_mitigation_module_ids(bcl_dev));
}

static ssize_t oilo2_triggered_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[BATOILO2])
		return -ENODEV;
	return sysfs_emit(buf, "%d_%d\n", bcl_dev->zone[BATOILO2]->current_state,
					  get_final_mitigation_module_ids(bcl_dev));
}

static ssize_t smpl_triggered_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (!bcl_dev->zone[SMPL_WARN])
		return -ENODEV;
	return sysfs_emit(buf, "%d_%d\n", bcl_dev->zone[SMPL_WARN]->current_state,
					  get_final_mitigation_module_ids(bcl_dev));
}

static DEVICE_ATTR(oilo1_triggered, 0444, oilo1_triggered_show, NULL);
static DEVICE_ATTR(oilo2_triggered, 0444, oilo2_triggered_show, NULL);
static DEVICE_ATTR(uvlo1_triggered, 0444, uvlo1_triggered_show, NULL);
static DEVICE_ATTR(uvlo2_triggered, 0444, uvlo2_triggered_show, NULL);
static DEVICE_ATTR(smpl_triggered, 0444, smpl_triggered_show, NULL);

void bunch_mitigation_threshold_addr(struct bcl_mitigation_conf *mitigation_conf,
					unsigned int *addr[METER_CHANNEL_MAX]) {
	int i;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return;
#endif

	for (i = 0; i < METER_CHANNEL_MAX; i++)
		addr[i] = &mitigation_conf[i].threshold;
}

void bunch_mitigation_module_id_addr(struct bcl_mitigation_conf *mitigation_conf,
					unsigned int *addr[METER_CHANNEL_MAX]) {
	int i;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return;
#endif

	for (i = 0; i < METER_CHANNEL_MAX; i++)
		addr[i] = &mitigation_conf[i].module_id;
}

static ssize_t mitigation_show(unsigned int *addr[METER_CHANNEL_MAX], char *buf)
{
	int i, at = 0;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif

	for (i = 0; i < METER_CHANNEL_MAX; i++)
		at += sysfs_emit_at(buf, at, "%d,", *addr[i]);

	return at;
}

static ssize_t mitigation_store(unsigned int *addr[METER_CHANNEL_MAX],
					const char *buf, size_t size) {
	int i;
	unsigned int ch[METER_CHANNEL_MAX] = {0};
	char * const str = kstrndup(buf, size, GFP_KERNEL);
	char *sep_str = str;
	char *token = NULL;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	goto mitigation_store_exit;
#endif

	if (!sep_str)
		goto mitigation_store_exit;

	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		token = strsep(&sep_str, MITIGATION_INPUT_DELIM);
		if (!token || sscanf(token, "%d", &ch[i]) != 1)
			break;
		else
			*addr[i] = ch[i];
	}

mitigation_store_exit:
	kfree(str);

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	return -ENODEV;
#endif
	return size;
}

static ssize_t main_mitigation_threshold_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int *addr[METER_CHANNEL_MAX];

	bunch_mitigation_threshold_addr(bcl_dev->main_mitigation_conf, addr);

	return mitigation_show(addr, buf);
}

static ssize_t main_mitigation_threshold_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int *addr[METER_CHANNEL_MAX];

	bunch_mitigation_threshold_addr(bcl_dev->main_mitigation_conf, addr);

	return mitigation_store(addr, buf, size);
}

static ssize_t sub_mitigation_threshold_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int *addr[METER_CHANNEL_MAX];

	bunch_mitigation_threshold_addr(bcl_dev->sub_mitigation_conf, addr);

	return mitigation_show(addr, buf);
}

static ssize_t sub_mitigation_threshold_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int *addr[METER_CHANNEL_MAX];

	bunch_mitigation_threshold_addr(bcl_dev->sub_mitigation_conf, addr);

	return mitigation_store(addr, buf, size);
}

static ssize_t main_mitigation_module_id_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int *addr[METER_CHANNEL_MAX];

	bunch_mitigation_module_id_addr(bcl_dev->main_mitigation_conf, addr);

	return mitigation_show(addr, buf);
}

static ssize_t main_mitigation_module_id_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int *addr[METER_CHANNEL_MAX];

	bunch_mitigation_module_id_addr(bcl_dev->main_mitigation_conf, addr);

	return mitigation_store(addr, buf, size);
}

static ssize_t sub_mitigation_module_id_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int *addr[METER_CHANNEL_MAX];

	bunch_mitigation_module_id_addr(bcl_dev->sub_mitigation_conf, addr);

	return mitigation_show(addr, buf);
}

static ssize_t sub_mitigation_module_id_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int *addr[METER_CHANNEL_MAX];

	bunch_mitigation_module_id_addr(bcl_dev->sub_mitigation_conf, addr);

	return mitigation_store(addr, buf, size);
}

static DEVICE_ATTR_RW(main_mitigation_threshold);
static DEVICE_ATTR_RW(sub_mitigation_threshold);
static DEVICE_ATTR_RW(main_mitigation_module_id);
static DEVICE_ATTR_RW(sub_mitigation_module_id);

static struct attribute *mitigation_attrs[] = {
	&dev_attr_main_mitigation_threshold.attr,
	&dev_attr_sub_mitigation_threshold.attr,
	&dev_attr_main_mitigation_module_id.attr,
	&dev_attr_sub_mitigation_module_id.attr,
	NULL,
};

static struct attribute *triggered_state_sq_attrs[] = {
	&dev_attr_oilo1_triggered.attr,
	&dev_attr_uvlo1_triggered.attr,
	&dev_attr_uvlo2_triggered.attr,
	&dev_attr_smpl_triggered.attr,
	&dev_attr_oilo2_triggered.attr,
	NULL,
};

static struct attribute *triggered_state_mw_attrs[] = {
	&dev_attr_oilo1_triggered.attr,
	&dev_attr_uvlo1_triggered.attr,
	&dev_attr_uvlo2_triggered.attr,
	&dev_attr_smpl_triggered.attr,
	NULL,
};

static ssize_t triggered_idx_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_idx);
}

static DEVICE_ATTR(triggered_idx, 0444, triggered_idx_show, NULL);

static ssize_t enable_br_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->enabled_br_stats);
}

static ssize_t enable_br_stats_store(struct device *dev, struct device_attribute *attr,
				                          const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret;

	if (!bcl_dev->data_logging_initialized)
		return -EINVAL;

	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;

	bcl_dev->enabled_br_stats = value;

	return size;
}

static DEVICE_ATTR_RW(enable_br_stats);

static ssize_t trigger_br_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "");
}

static ssize_t trigger_br_stats_store(struct device *dev, struct device_attribute *attr,
				                          const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;

	if (!bcl_dev->data_logging_initialized)
		return -EINVAL;

	if (kstrtouint(buf, 16, &value) != 0 || value >= TRIGGERED_SOURCE_MAX)
		return -EINVAL;

	dev_dbg(bcl_dev->device, "Triggered: %d\n", value);
	google_bcl_start_data_logging(bcl_dev, value);

	return size;
}

static DEVICE_ATTR_RW(trigger_br_stats);

static ssize_t meter_channels_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", METER_CHANNEL_MAX);
}
static DEVICE_ATTR_RO(meter_channels);

static struct attribute *br_stats_attrs[] = {
	&dev_attr_triggered_idx.attr,
	&dev_attr_enable_br_stats.attr,
	&dev_attr_trigger_br_stats.attr,
	&dev_attr_meter_channels.attr,
	NULL,
};

static ssize_t br_stats_dump_read(struct file *filp,
				  struct kobject *kobj, struct bin_attribute *attr,
				  char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (off > bcl_dev->br_stats_size)
		return 0;
	if (off + count > bcl_dev->br_stats_size)
		count = bcl_dev->br_stats_size - off;

	memcpy(buf, (const void *)bcl_dev->br_stats + off, count);

	return count;
}

static struct bin_attribute br_stats_dump_attr = {
	.attr = { .name = "stats", .mode = 0444 },
	.read = br_stats_dump_read,
	.size = sizeof(struct brownout_stats),
};

static struct bin_attribute *br_stats_bin_attrs[] = {
	&br_stats_dump_attr,
	NULL,
};

static const struct attribute_group irq_dur_cnt_group = {
	.attrs = irq_dur_cnt_attrs,
	.name = "irq_dur_cnt",
};

static const struct attribute_group qos_group = {
	.attrs = qos_attrs,
	.name = "qos",
};

static const struct attribute_group br_stats_group = {
	.attrs = br_stats_attrs,
	.bin_attrs = br_stats_bin_attrs,
	.name = "br_stats",
};

static const struct attribute_group irq_config_group = {
	.attrs = irq_config_attrs,
	.name = "irq_config",
};

const struct attribute_group triggered_state_sq_group = {
	.attrs = triggered_state_sq_attrs,
	.name = "triggered_state",
};

const struct attribute_group triggered_state_mw_group = {
	.attrs = triggered_state_mw_attrs,
	.name = "triggered_state",
};

const struct attribute_group mitigation_group = {
	.attrs = mitigation_attrs,
	.name = "mitigation",
};


const struct attribute_group *mitigation_mw_groups[] = {
	&instr_group,
	&triggered_lvl_group,
	&clock_div_group,
	&clock_ratio_group,
	&clock_stats_group,
	&triggered_count_group,
	&triggered_timestamp_group,
	&triggered_capacity_group,
	&triggered_voltage_group,
	&vdroop_flt_group,
	&main_pwrwarn_group,
	&sub_pwrwarn_group,
	&irq_dur_cnt_group,
	&qos_group,
	&br_stats_group,
	&last_triggered_mode_group,
	&triggered_state_mw_group,
	&irq_config_group,
	NULL,
};

const struct attribute_group *mitigation_sq_groups[] = {
	&instr_group,
	&triggered_lvl_group,
	&clock_div_group,
	&clock_ratio_group,
	&clock_stats_group,
	&triggered_count_group,
	&triggered_timestamp_group,
	&triggered_capacity_group,
	&triggered_voltage_group,
	&vdroop_flt_group,
	&main_pwrwarn_group,
	&sub_pwrwarn_group,
	&irq_dur_cnt_group,
	&qos_group,
	&br_stats_group,
	&last_triggered_mode_group,
	&triggered_state_sq_group,
	&mitigation_group,
	&irq_config_group,
	&ktimer_group,
	NULL,
};
