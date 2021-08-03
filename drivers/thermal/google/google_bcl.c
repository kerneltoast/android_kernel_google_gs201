// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl.c Google bcl driver
 *
 * Copyright (c) 2020, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#include <linux/regulator/pmic_class.h>
#include <soc/google/bcl.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif
#include "../thermal_core.h"

/* This driver determines if HW was throttled due to SMPL/OCP */

#define CPUCL0_BASE (0x20c00000)
#define CPUCL1_BASE (0x20c10000)
#define CPUCL2_BASE (0x20c20000)
#define G3D_BASE (0x1c400000)
#define TPU_BASE (0x1cc00000)
#define SYSREG_CPUCL0_BASE (0x20c40000)
#define CLUSTER0_GENERAL_CTRL_64 (0x1404)
#define CLKDIVSTEP (0x830)
#define CPUCL0_CLKDIVSTEP_STAT (0x83c)
#define CPUCL0_CLKDIVSTEP_CON (0x838)
#define CPUCL12_CLKDIVSTEP_STAT (0x848)
#define CPUCL12_CLKDIVSTEP_CON_HEAVY (0x840)
#define CPUCL12_CLKDIVSTEP_CON_LIGHT (0x844)
#define G3D_CLKDIVSTEP_STAT (0x854)
#define TPU_CLKDIVSTEP_STAT (0x850)
#define CLUSTER0_MPMM (0x1408)
#define CLUSTER0_PPM (0x140c)
#define MPMMEN_MASK (0xF << 21)
#define PPMEN_MASK (0x3 << 8)
#define PPMCTL_MASK (0xFF)
#define OCP_WARN_MASK (0x1F)
#define SMPL_WARN_MASK (0xE0)
#define B3M_UPPER_LIMIT (7000)
#define B3M_LOWER_LIMIT (1688)
#define B3M_STEP (166)
#define B2M_UPPER_LIMIT (12000)
#define B2M_LOWER_LIMIT (4000)
#define B2M_STEP (250)
#define B10M_UPPER_LIMIT (10500)
#define B10M_LOWER_LIMIT (2500)
#define B10M_STEP (250)
#define B2S_UPPER_LIMIT (12000)
#define B2S_LOWER_LIMIT (4000)
#define B2S_STEP (250)
#define SMPL_BATTERY_VOLTAGE (4200)
#define SMPL_UPPER_LIMIT (3300)
#define SMPL_LOWER_LIMIT (2600)
#define SMPL_STEP (100)
#define SMPL_NUM_LVL (32)
#define THERMAL_IRQ_COUNTER_LIMIT (5)
#define THERMAL_HYST_LEVEL (100)
#define ACTIVE_HIGH (0x1)
#define ACTIVE_LOW (0x0)
#define THERMAL_DELAY_INIT_MS 5000
#define PMIC_OVERHEAT_UPPER_LIMIT (2000)
#define PMIC_120C_UPPER_LIMIT (1200)
#define PMIC_140C_UPPER_LIMIT (1400)
#define PMU_ALIVE_CPU0_OUT (0x1CA0)
#define PMU_ALIVE_CPU1_OUT (0x1D20)
#define PMU_ALIVE_CPU2_OUT (0x1DA0)
#define PMU_ALIVE_TPU_OUT (0x2920)
#define PMU_ALIVE_GPU_OUT (0x1E20)
#define ONE_SECOND 1000

static const char * const triggered_source[] = {
	[IRQ_SMPL_WARN] = "smpl_warn",
	[IRQ_PMIC_120C] = "pmic_120c",
	[IRQ_PMIC_140C] = "pmic_140c",
	[IRQ_PMIC_OVERHEAT] = "pmic_overheat",
	[IRQ_OCP_WARN_CPUCL1] = "ocp_cpu1",
	[IRQ_OCP_WARN_CPUCL2] = "ocp_cpu2",
	[IRQ_SOFT_OCP_WARN_CPUCL1] = "soft_ocp_cpu1",
	[IRQ_SOFT_OCP_WARN_CPUCL2] = "soft_ocp_cpu2",
	[IRQ_OCP_WARN_TPU] = "ocp_tpu",
	[IRQ_SOFT_OCP_WARN_TPU] = "soft_ocp_tpu",
	[IRQ_OCP_WARN_GPU] = "ocp_gpu",
	[IRQ_SOFT_OCP_WARN_GPU] = "soft_ocp_gpu"};

static const char * const clk_ratio_source[] = {
	"cpu0", "cpu1_heavy", "cpu2_heavy", "tpu_heavy", "gpu_heavy",
	"cpu1_light", "cpu2_light", "tpu_light", "gpu_light"
};

enum RATIO_SOURCE {
	CPU0_CON,
	CPU1_HEAVY,
	CPU2_HEAVY,
	TPU_HEAVY,
	GPU_HEAVY,
	CPU1_LIGHT,
	CPU2_LIGHT,
	TPU_LIGHT,
	GPU_LIGHT
};

static const char * const clk_stats_source[] = {
	"cpu0", "cpu1", "cpu2", "tpu", "gpu"
};

static const unsigned int clk_stats_offset[] = {
	CPUCL0_CLKDIVSTEP_STAT,
	CPUCL12_CLKDIVSTEP_STAT,
	CPUCL12_CLKDIVSTEP_STAT,
	TPU_CLKDIVSTEP_STAT,
	G3D_CLKDIVSTEP_STAT
};

enum SUBSYSTEM_SOURCE {
	CPU0,
	CPU1,
	CPU2,
	TPU,
	GPU,
	SUBSYSTEM_SOURCE_MAX,
};

static const unsigned int subsystem_pmu[] = {
	PMU_ALIVE_CPU0_OUT,
	PMU_ALIVE_CPU1_OUT,
	PMU_ALIVE_CPU2_OUT,
	PMU_ALIVE_TPU_OUT,
	PMU_ALIVE_GPU_OUT
};

static const struct platform_device_id google_id_table[] = {
	{.name = "google_mitigation",},
	{},
};

DEFINE_MUTEX(sysreg_lock);

static bool is_subsystem_on(unsigned int addr)
{
	unsigned int value;

	if ((addr == PMU_ALIVE_TPU_OUT) || (addr == PMU_ALIVE_GPU_OUT)) {
		exynos_pmu_read(addr, &value);
		return value & BIT(6);
	}
	return true;
}

static int triggered_read_level(void *data, int *val, int id)
{
	struct bcl_device *bcl_dev = data;

	if ((bcl_dev->triggered_counter[id] != 0) &&
	    (bcl_dev->triggered_counter[id] < THERMAL_IRQ_COUNTER_LIMIT)) {
		*val = bcl_dev->triggered_lvl[id] +
				THERMAL_HYST_LEVEL;
		bcl_dev->triggered_counter[id] += 1;
	} else {
		*val = bcl_dev->triggered_lvl[id];
		bcl_dev->triggered_counter[id] = 0;
	}
	return 0;
}

static struct power_supply *google_get_power_supply(struct bcl_device *bcl_dev)
{
	static struct power_supply *psy[2];
	static struct power_supply *batt_psy;
	int err = 0;

	batt_psy = NULL;
	err = power_supply_get_by_phandle_array(bcl_dev->device->of_node,
						"google,power-supply", psy,
						ARRAY_SIZE(psy));
	if (err > 0)
		batt_psy = psy[0];
	return batt_psy;
}

static void ocpsmpl_read_stats(struct bcl_device *bcl_dev,
			       struct ocpsmpl_stats *dst, struct power_supply *psy)
{
	union power_supply_propval ret = {0};
	int err = 0;

	dst->_time = ktime_to_ms(ktime_get());
	err = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &ret);
	if (err < 0)
		dst->capacity = -1;
	else {
		dst->capacity = ret.intval;
		bcl_dev->batt_psy_initialized = true;
	}
	err = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	if (err < 0)
		dst->voltage = -1;
	else {
		dst->voltage = ret.intval;
		bcl_dev->batt_psy_initialized = true;
	}

}

static irqreturn_t irq_handler(int irq, void *data, u8 idx)
{
	struct bcl_device *bcl_dev = data;

	if (bcl_dev->batt_psy_initialized) {
		atomic_inc(&bcl_dev->triggered_cnt[idx]);
		ocpsmpl_read_stats(bcl_dev, &bcl_dev->triggered_stats[idx], bcl_dev->batt_psy);
	}
	if (bcl_dev->triggered_counter[idx] == 0) {
		bcl_dev->triggered_counter[idx] += 1;
		queue_delayed_work(system_wq, &bcl_dev->triggered_irq_work[idx],
				   msecs_to_jiffies(ONE_SECOND));

		/* Minimize the amount of thermal update by only triggering
		 * update every ONE_SECOND.
		 */
		if (bcl_dev->triggered_tz_irq[idx])
			thermal_zone_device_update(bcl_dev->triggered_tz_irq[idx],
						   THERMAL_EVENT_UNSPECIFIED);
	}
	return IRQ_HANDLED;
}

static irqreturn_t google_smpl_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_SMPL_WARN);
}

static void google_smpl_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
	    container_of(work, struct bcl_device, triggered_irq_work[IRQ_SMPL_WARN].work);

	bcl_dev->triggered_counter[IRQ_SMPL_WARN] = 0;
}

static int smpl_warn_read_voltage(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_SMPL_WARN);
}

static const struct thermal_zone_of_device_ops google_smpl_warn_ops = {
	.get_temp = smpl_warn_read_voltage,
};

static void google_cpu1_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_OCP_WARN_CPUCL1].work);

	bcl_dev->triggered_counter[IRQ_OCP_WARN_CPUCL1] = 0;
}

static irqreturn_t google_cpu1_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_OCP_WARN_CPUCL1);
}

static int ocp_cpu1_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_OCP_WARN_CPUCL1);
}

static const struct thermal_zone_of_device_ops google_ocp_cpu1_ops = {
	.get_temp = ocp_cpu1_read_current,
};

static void google_cpu2_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_device =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_OCP_WARN_CPUCL2].work);

	bcl_device->triggered_counter[IRQ_OCP_WARN_CPUCL2] = 0;
}

static irqreturn_t google_cpu2_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_OCP_WARN_CPUCL2);
}

static int ocp_cpu2_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_OCP_WARN_CPUCL2);
}

static const struct thermal_zone_of_device_ops google_ocp_cpu2_ops = {
	.get_temp = ocp_cpu2_read_current,
};

static void google_soft_cpu1_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_SOFT_OCP_WARN_CPUCL1].work);

	bcl_dev->triggered_counter[IRQ_SOFT_OCP_WARN_CPUCL1] = 0;
}

static irqreturn_t google_soft_cpu1_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_SOFT_OCP_WARN_CPUCL1);
}

static int soft_ocp_cpu1_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_SOFT_OCP_WARN_CPUCL1);
}

static const struct thermal_zone_of_device_ops google_soft_ocp_cpu1_ops = {
	.get_temp = soft_ocp_cpu1_read_current,
};

static void google_soft_cpu2_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_SOFT_OCP_WARN_CPUCL2].work);

	bcl_dev->triggered_counter[IRQ_SOFT_OCP_WARN_CPUCL2] = 0;
}

static irqreturn_t google_soft_cpu2_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_SOFT_OCP_WARN_CPUCL2);
}

static int soft_ocp_cpu2_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_SOFT_OCP_WARN_CPUCL2);
}

static const struct thermal_zone_of_device_ops google_soft_ocp_cpu2_ops = {
	.get_temp = soft_ocp_cpu2_read_current,
};

static void google_tpu_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_OCP_WARN_TPU].work);

	bcl_dev->triggered_counter[IRQ_OCP_WARN_TPU] = 0;
}

static irqreturn_t google_tpu_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_OCP_WARN_TPU);
}

static int ocp_tpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_OCP_WARN_TPU);
}

static const struct thermal_zone_of_device_ops google_ocp_tpu_ops = {
	.get_temp = ocp_tpu_read_current,
};

static void google_soft_tpu_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_SOFT_OCP_WARN_TPU].work);

	bcl_dev->triggered_counter[IRQ_SOFT_OCP_WARN_TPU] = 0;
}

static irqreturn_t google_soft_tpu_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_SOFT_OCP_WARN_TPU);
}

static int soft_ocp_tpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_SOFT_OCP_WARN_TPU);
}

static const struct thermal_zone_of_device_ops google_soft_ocp_tpu_ops = {
	.get_temp = soft_ocp_tpu_read_current,
};

static void google_gpu_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_OCP_WARN_GPU].work);

	bcl_dev->triggered_counter[IRQ_OCP_WARN_GPU] = 0;
}

static irqreturn_t google_gpu_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_OCP_WARN_GPU);
}

static int ocp_gpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_OCP_WARN_GPU);
}

static const struct thermal_zone_of_device_ops google_ocp_gpu_ops = {
	.get_temp = ocp_gpu_read_current,
};

static void google_soft_gpu_warn_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_SOFT_OCP_WARN_GPU].work);

	bcl_dev->triggered_counter[IRQ_SOFT_OCP_WARN_GPU] = 0;
}

static irqreturn_t google_soft_gpu_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_SOFT_OCP_WARN_GPU);
}

static int soft_ocp_gpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_SOFT_OCP_WARN_GPU);
}

static const struct thermal_zone_of_device_ops google_soft_ocp_gpu_ops = {
	.get_temp = soft_ocp_gpu_read_current,
};

static void google_pmic_120c_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_PMIC_120C].work);

	bcl_dev->triggered_counter[IRQ_PMIC_120C] = 0;
}

static irqreturn_t google_pmic_120c_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_PMIC_120C);
}

static int pmic_120c_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_PMIC_120C);
}

static const struct thermal_zone_of_device_ops google_pmic_120c_ops = {
	.get_temp = pmic_120c_read_temp,
};

static void google_pmic_140c_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_PMIC_140C].work);

	bcl_dev->triggered_counter[IRQ_PMIC_140C] = 0;
}

static irqreturn_t google_pmic_140c_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_PMIC_140C);
}

static int pmic_140c_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_PMIC_140C);
}

static const struct thermal_zone_of_device_ops google_pmic_140c_ops = {
	.get_temp = pmic_140c_read_temp,
};

static void google_pmic_overheat_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev =
			container_of(work, struct bcl_device,
		   triggered_irq_work[IRQ_PMIC_OVERHEAT].work);

	bcl_dev->triggered_counter[IRQ_PMIC_OVERHEAT] = 0;
}

static irqreturn_t google_tsd_overheat_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, IRQ_PMIC_OVERHEAT);
}

static int tsd_overheat_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, IRQ_PMIC_OVERHEAT);
}

static const struct thermal_zone_of_device_ops google_pmic_overheat_ops = {
	.get_temp = tsd_overheat_read_temp,
};

static int google_bcl_set_soc(void *data, int low, int high)
{
	struct bcl_device *bcl_dev = data;

	if (high == bcl_dev->trip_high_temp)
		return 0;

	mutex_lock(&bcl_dev->state_trans_lock);
	bcl_dev->trip_low_temp = low;
	bcl_dev->trip_high_temp = high;
	schedule_delayed_work(&bcl_dev->soc_eval_work, 0);

	mutex_unlock(&bcl_dev->state_trans_lock);
	return 0;
}

static int google_bcl_read_soc(void *data, int *val)
{
	struct bcl_device *bcl_dev = data;
	union power_supply_propval ret = {
		0,
	};
	int err = 0;

	*val = 100;
	if (!bcl_dev->batt_psy)
		bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	if (bcl_dev->batt_psy) {
		err = power_supply_get_property(bcl_dev->batt_psy,
						POWER_SUPPLY_PROP_CAPACITY, &ret);
		if (err < 0) {
			dev_err(bcl_dev->device, "battery percentage read error:%d\n", err);
			return err;
		}
		bcl_dev->batt_psy_initialized = true;
		*val = 100 - ret.intval;
	}
	pr_debug("soc:%d\n", *val);

	return err;
}

static void google_bcl_evaluate_soc(struct work_struct *work)
{
	int battery_percentage_reverse;
	struct bcl_device *bcl_dev =
	    container_of(work, struct bcl_device, soc_eval_work.work);

	if (google_bcl_read_soc(bcl_dev, &battery_percentage_reverse))
		return;

	mutex_lock(&bcl_dev->state_trans_lock);
	if ((battery_percentage_reverse < bcl_dev->trip_high_temp) &&
		(battery_percentage_reverse > bcl_dev->trip_low_temp))
		goto eval_exit;

	bcl_dev->trip_val = battery_percentage_reverse;
	mutex_unlock(&bcl_dev->state_trans_lock);
	if (!bcl_dev->soc_tzd) {
		bcl_dev->soc_tzd = thermal_zone_of_sensor_register(bcl_dev->device, PMIC_SOC,
								   bcl_dev, &bcl_dev->soc_ops);
		if (IS_ERR(bcl_dev->soc_tzd)) {
			dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
				PTR_ERR(bcl_dev->soc_tzd));
			return;
		}
	}
	if (!IS_ERR(bcl_dev->soc_tzd))
		thermal_zone_device_update(bcl_dev->soc_tzd, THERMAL_EVENT_UNSPECIFIED);
	return;
eval_exit:
	mutex_unlock(&bcl_dev->state_trans_lock);
}

static int battery_supply_callback(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct bcl_device *bcl_dev =
			container_of(nb, struct bcl_device, psy_nb);
	struct power_supply *bcl_psy;

	if (!bcl_dev)
		return NOTIFY_OK;

	bcl_psy = bcl_dev->batt_psy;

	if (!bcl_psy || event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!strcmp(psy->desc->name, bcl_psy->desc->name))
		schedule_delayed_work(&bcl_dev->soc_eval_work, 0);

	return NOTIFY_OK;
}

static int google_bcl_remove_thermal(struct bcl_device *bcl_dev)
{
	int i = 0;

	power_supply_unreg_notifier(&bcl_dev->psy_nb);
	if (bcl_dev->soc_tzd)
		thermal_zone_of_sensor_unregister(bcl_dev->device, bcl_dev->soc_tzd);
	for (i = 0; i < IRQ_TRIGGERED_SOURCE_MAX; i++) {
		if (bcl_dev->triggered_tz_irq[i])
			thermal_zone_of_sensor_unregister(bcl_dev->device,
							  bcl_dev->triggered_tz_irq[i]);
	}

	return 0;
}

static ssize_t batoilo_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->if_triggered_cnt[BATOILO]));
}

static DEVICE_ATTR_RO(batoilo_count);

static ssize_t vdroop2_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->if_triggered_cnt[VDROOP2]));
}

static DEVICE_ATTR_RO(vdroop2_count);

static ssize_t vdroop1_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->if_triggered_cnt[VDROOP1]));
}

static DEVICE_ATTR_RO(vdroop1_count);

static ssize_t smpl_warn_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->triggered_cnt[IRQ_SMPL_WARN]));
}

static DEVICE_ATTR_RO(smpl_warn_count);

static ssize_t ocp_cpu1_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->triggered_cnt[IRQ_OCP_WARN_CPUCL1]));
}

static DEVICE_ATTR_RO(ocp_cpu1_count);

static ssize_t ocp_cpu2_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->triggered_cnt[IRQ_OCP_WARN_CPUCL2]));
}

static DEVICE_ATTR_RO(ocp_cpu2_count);

static ssize_t ocp_tpu_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->triggered_cnt[IRQ_OCP_WARN_TPU]));
}

static DEVICE_ATTR_RO(ocp_tpu_count);

static ssize_t ocp_gpu_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->triggered_cnt[IRQ_OCP_WARN_GPU]));
}

static DEVICE_ATTR_RO(ocp_gpu_count);

static ssize_t soft_ocp_cpu1_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  atomic_read(&bcl_dev->triggered_cnt[IRQ_SOFT_OCP_WARN_CPUCL1]));
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_count);

static ssize_t soft_ocp_cpu2_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  atomic_read(&bcl_dev->triggered_cnt[IRQ_SOFT_OCP_WARN_CPUCL2]));
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_count);

static ssize_t soft_ocp_tpu_count_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  atomic_read(&bcl_dev->triggered_cnt[IRQ_SOFT_OCP_WARN_TPU]));
}

static DEVICE_ATTR_RO(soft_ocp_tpu_count);

static ssize_t soft_ocp_gpu_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  atomic_read(&bcl_dev->triggered_cnt[IRQ_SOFT_OCP_WARN_GPU]));
}

static DEVICE_ATTR_RO(soft_ocp_gpu_count);

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
	NULL,
};

static ssize_t batoilo_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[BATOILO].capacity);
}

static DEVICE_ATTR_RO(batoilo_cap);

static ssize_t vdroop2_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[VDROOP2].capacity);
}

static DEVICE_ATTR_RO(vdroop2_cap);

static ssize_t vdroop1_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[VDROOP1].capacity);
}

static DEVICE_ATTR_RO(vdroop1_cap);

static ssize_t smpl_warn_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_SMPL_WARN].capacity);
}

static DEVICE_ATTR_RO(smpl_warn_cap);

static ssize_t ocp_cpu1_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_CPUCL1].capacity);
}

static DEVICE_ATTR_RO(ocp_cpu1_cap);

static ssize_t ocp_cpu2_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_CPUCL2].capacity);
}

static DEVICE_ATTR_RO(ocp_cpu2_cap);

static ssize_t ocp_tpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_TPU].capacity);
}

static DEVICE_ATTR_RO(ocp_tpu_cap);

static ssize_t ocp_gpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_GPU].capacity);
}

static DEVICE_ATTR_RO(ocp_gpu_cap);

static ssize_t soft_ocp_cpu1_cap_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_CPUCL1].capacity);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_cap);

static ssize_t soft_ocp_cpu2_cap_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_CPUCL2].capacity);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_cap);

static ssize_t soft_ocp_tpu_cap_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_TPU].capacity);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_cap);

static ssize_t soft_ocp_gpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_GPU].capacity);
}

static DEVICE_ATTR_RO(soft_ocp_gpu_cap);

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
	NULL,
};

static ssize_t batoilo_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[BATOILO].voltage);
}

static DEVICE_ATTR_RO(batoilo_volt);

static ssize_t vdroop2_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[VDROOP2].voltage);
}

static DEVICE_ATTR_RO(vdroop2_volt);

static ssize_t vdroop1_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[VDROOP1].voltage);
}

static DEVICE_ATTR_RO(vdroop1_volt);

static ssize_t smpl_warn_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_SMPL_WARN].voltage);
}

static DEVICE_ATTR_RO(smpl_warn_volt);

static ssize_t ocp_cpu1_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_CPUCL1].voltage);
}

static DEVICE_ATTR_RO(ocp_cpu1_volt);

static ssize_t ocp_cpu2_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_CPUCL2].voltage);
}

static DEVICE_ATTR_RO(ocp_cpu2_volt);

static ssize_t ocp_tpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_TPU].voltage);
}

static DEVICE_ATTR_RO(ocp_tpu_volt);

static ssize_t ocp_gpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_GPU].voltage);
}

static DEVICE_ATTR_RO(ocp_gpu_volt);

static ssize_t soft_ocp_cpu1_volt_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_CPUCL1].voltage);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_volt);

static ssize_t soft_ocp_cpu2_volt_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_CPUCL2].voltage);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_volt);

static ssize_t soft_ocp_tpu_volt_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_TPU].voltage);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_volt);

static ssize_t soft_ocp_gpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_GPU].voltage);
}

static DEVICE_ATTR_RO(soft_ocp_gpu_volt);

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
	NULL,
};

static ssize_t batoilo_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[BATOILO]._time);
}

static DEVICE_ATTR_RO(batoilo_time);

static ssize_t vdroop2_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[VDROOP2]._time);
}

static DEVICE_ATTR_RO(vdroop2_time);

static ssize_t vdroop1_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->if_triggered_stats[VDROOP1]._time);
}

static DEVICE_ATTR_RO(vdroop1_time);

static ssize_t smpl_warn_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_SMPL_WARN]._time);
}

static DEVICE_ATTR_RO(smpl_warn_time);

static ssize_t ocp_cpu1_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_CPUCL1]._time);
}

static DEVICE_ATTR_RO(ocp_cpu1_time);

static ssize_t ocp_cpu2_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_CPUCL2]._time);
}

static DEVICE_ATTR_RO(ocp_cpu2_time);

static ssize_t ocp_tpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_TPU]._time);
}

static DEVICE_ATTR_RO(ocp_tpu_time);

static ssize_t ocp_gpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->triggered_stats[IRQ_OCP_WARN_GPU]._time);
}

static DEVICE_ATTR_RO(ocp_gpu_time);

static ssize_t soft_ocp_cpu1_time_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_CPUCL1]._time);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_time);

static ssize_t soft_ocp_cpu2_time_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_CPUCL2]._time);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_time);

static ssize_t soft_ocp_tpu_time_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_TPU]._time);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_time);

static ssize_t soft_ocp_gpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n",
			  bcl_dev->triggered_stats[IRQ_SOFT_OCP_WARN_GPU]._time);
}

static DEVICE_ATTR_RO(soft_ocp_gpu_time);

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
	NULL,
};

static const struct attribute_group triggered_count_group = {
	.attrs = triggered_count_attrs,
	.name = "last_triggered_count",
};

static const struct attribute_group triggered_timestamp_group = {
	.attrs = triggered_time_attrs,
	.name = "last_triggered_timestamp",
};

static const struct attribute_group triggered_capacity_group = {
	.attrs = triggered_cap_attrs,
	.name = "last_triggered_capacity",
};

static const struct attribute_group triggered_voltage_group = {
	.attrs = triggered_volt_attrs,
	.name = "last_triggered_voltage",
};

static void __iomem *get_addr_by_subsystem(struct bcl_device *bcl_dev,
					   const char *subsystem)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(clk_stats_source); i++) {
		if (strcmp(subsystem, clk_stats_source[i]) == 0) {
			if (!is_subsystem_on(subsystem_pmu[i]))
				return NULL;
			return bcl_dev->base_mem[i] + CLKDIVSTEP;
		}
	}
	return NULL;
}

static ssize_t clk_div_show(struct bcl_device *bcl_dev, int idx, char *buf)
{
	unsigned int reg;
	void __iomem *addr;

	if (idx == TPU)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->tpu_clkdivstep);
	else if (idx == GPU)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->gpu_clkdivstep);

	addr = get_addr_by_subsystem(bcl_dev, clk_stats_source[idx]);
	if (addr == NULL)
		return sysfs_emit(buf, "off\n");
	reg = __raw_readl(addr);

	return sysfs_emit(buf, "0x%x\n", reg);
}

static ssize_t clk_stats_show(struct bcl_device *bcl_dev, int idx, char *buf)
{
	unsigned int reg;
	void __iomem *addr;

	if (idx == TPU)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->tpu_clk_stats);
	else if (idx == GPU)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->gpu_clk_stats);

	addr = get_addr_by_subsystem(bcl_dev, clk_stats_source[idx]);
	if (addr == NULL)
		return sysfs_emit(buf, "off\n");
	reg = __raw_readl(bcl_dev->base_mem[idx] + clk_stats_offset[idx]);

	return sysfs_emit(buf, "0x%x\n", reg);
}

static ssize_t clk_div_store(struct bcl_device *bcl_dev, int idx,
			     const char *buf, size_t size)
{
	void __iomem *addr;
	unsigned int value;
	int ret;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	addr = get_addr_by_subsystem(bcl_dev, clk_stats_source[idx]);

	if (addr == NULL) {
		dev_err(bcl_dev->device, "Address is NULL\n");
		return -EIO;
	}

	if (idx == TPU)
		bcl_dev->tpu_clkdivstep = value;
	else if (idx == GPU)
		bcl_dev->gpu_clkdivstep = value;
	else {
		mutex_lock(&bcl_dev->ratio_lock);
		__raw_writel(value, addr);
		mutex_unlock(&bcl_dev->ratio_lock);
	}

	return size;
}

static ssize_t cpu0_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, CPU0, buf);
}

static ssize_t cpu0_clk_div_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, CPU0, buf, size);
}

static DEVICE_ATTR_RW(cpu0_clk_div);

static ssize_t cpu1_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, CPU1, buf);
}

static ssize_t cpu1_clk_div_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, CPU1, buf, size);
}

static DEVICE_ATTR_RW(cpu1_clk_div);

static ssize_t cpu2_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, CPU2, buf);
}

static ssize_t cpu2_clk_div_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, CPU2, buf, size);
}

static DEVICE_ATTR_RW(cpu2_clk_div);

static ssize_t tpu_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, TPU, buf);
}

static ssize_t tpu_clk_div_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, TPU, buf, size);
}

static DEVICE_ATTR_RW(tpu_clk_div);

static ssize_t gpu_clk_div_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_show(bcl_dev, GPU, buf);
}

static ssize_t gpu_clk_div_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_div_store(bcl_dev, GPU, buf, size);
}

static DEVICE_ATTR_RW(gpu_clk_div);

static struct attribute *clock_div_attrs[] = {
	&dev_attr_cpu0_clk_div.attr,
	&dev_attr_cpu1_clk_div.attr,
	&dev_attr_cpu2_clk_div.attr,
	&dev_attr_tpu_clk_div.attr,
	&dev_attr_gpu_clk_div.attr,
	NULL,
};

static const struct attribute_group clock_div_group = {
	.attrs = clock_div_attrs,
	.name = "clock_div",
};

static ssize_t cpu0_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, CPU0, buf);
}

static DEVICE_ATTR_RO(cpu0_clk_stats);

static ssize_t cpu1_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, CPU1, buf);
}

static DEVICE_ATTR_RO(cpu1_clk_stats);

static ssize_t cpu2_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, CPU2, buf);
}

static DEVICE_ATTR_RO(cpu2_clk_stats);

static ssize_t tpu_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, TPU, buf);
}

static DEVICE_ATTR_RO(tpu_clk_stats);

static ssize_t gpu_clk_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_stats_show(bcl_dev, GPU, buf);
}

static DEVICE_ATTR_RO(gpu_clk_stats);

static struct attribute *clock_stats_attrs[] = {
	&dev_attr_cpu0_clk_stats.attr,
	&dev_attr_cpu1_clk_stats.attr,
	&dev_attr_cpu2_clk_stats.attr,
	&dev_attr_tpu_clk_stats.attr,
	&dev_attr_gpu_clk_stats.attr,
	NULL,
};

static const struct attribute_group clock_stats_group = {
	.attrs = clock_stats_attrs,
	.name = "clock_stats",
};

static void __iomem *get_addr_by_rail(struct bcl_device *bcl_dev, const char *rail_name)
{
	int i = 0, idx;

	for (i = 0; i < 9; i++) {
		if (strcmp(rail_name, clk_ratio_source[i]) == 0) {
			idx = i > 4 ? i - 4 : i;
			if (is_subsystem_on(subsystem_pmu[idx])) {
				if (idx == 0)
					return bcl_dev->base_mem[CPU0] + CPUCL0_CLKDIVSTEP_CON;
				if (i > 4)
					return bcl_dev->base_mem[idx] +
							CPUCL12_CLKDIVSTEP_CON_LIGHT;
				else
					return bcl_dev->base_mem[idx] +
							CPUCL12_CLKDIVSTEP_CON_HEAVY;
			} else
				return NULL;
		}
	}

	return NULL;
}

static ssize_t clk_ratio_show(struct bcl_device *bcl_dev, int idx, char *buf)
{
	unsigned int reg;
	void __iomem *addr;

	if (idx == TPU_HEAVY)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->tpu_con_heavy);
	else if (idx == TPU_LIGHT)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->tpu_con_light);
	else if (idx == GPU_LIGHT)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->gpu_con_light);
	else if (idx == GPU_HEAVY)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->gpu_con_heavy);

	addr = get_addr_by_rail(bcl_dev, clk_ratio_source[idx]);
	if (addr == NULL)
		return sysfs_emit(buf, "off\n");

	reg = __raw_readl(addr);
	return sysfs_emit(buf, "0x%x\n", reg);
}

static ssize_t clk_ratio_store(struct bcl_device *bcl_dev, int idx,
			       const char *buf, size_t size)
{
	void __iomem *addr;
	unsigned int value;
	int ret;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	addr = get_addr_by_rail(bcl_dev, clk_ratio_source[idx]);
	if (addr == NULL) {
		dev_err(bcl_dev->device, "Address is NULL\n");
		return -EIO;
	}
	if (idx == TPU_HEAVY)
		bcl_dev->tpu_con_heavy = value;
	else if (idx == GPU_HEAVY)
		bcl_dev->gpu_con_heavy = value;
	else if (idx == TPU_LIGHT)
		bcl_dev->tpu_con_light = value;
	else if (idx == GPU_LIGHT)
		bcl_dev->gpu_con_light = value;
	else {
		mutex_lock(&bcl_dev->ratio_lock);
		__raw_writel(value, addr);
		mutex_unlock(&bcl_dev->ratio_lock);
	}

	return size;
}

static ssize_t cpu0_clk_ratio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU0_CON, buf);
}

static ssize_t cpu0_clk_ratio_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU0_CON, buf, size);
}

static DEVICE_ATTR_RW(cpu0_clk_ratio);

static ssize_t cpu1_heavy_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU1_HEAVY, buf);
}

static ssize_t cpu1_heavy_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU1_HEAVY, buf, size);
}

static DEVICE_ATTR_RW(cpu1_heavy_clk_ratio);

static ssize_t cpu2_heavy_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU2_HEAVY, buf);
}

static ssize_t cpu2_heavy_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU2_HEAVY, buf, size);
}

static DEVICE_ATTR_RW(cpu2_heavy_clk_ratio);

static ssize_t tpu_heavy_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, TPU_HEAVY, buf);
}

static ssize_t tpu_heavy_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, TPU_HEAVY, buf, size);
}

static DEVICE_ATTR_RW(tpu_heavy_clk_ratio);

static ssize_t gpu_heavy_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, GPU_HEAVY, buf);
}

static ssize_t gpu_heavy_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, GPU_HEAVY, buf, size);
}

static DEVICE_ATTR_RW(gpu_heavy_clk_ratio);

static ssize_t cpu1_light_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU1_LIGHT, buf);
}

static ssize_t cpu1_light_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU1_LIGHT, buf, size);
}

static DEVICE_ATTR_RW(cpu1_light_clk_ratio);

static ssize_t cpu2_light_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, CPU2_LIGHT, buf);
}

static ssize_t cpu2_light_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, CPU2_LIGHT, buf, size);
}

static DEVICE_ATTR_RW(cpu2_light_clk_ratio);

static ssize_t tpu_light_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, TPU_LIGHT, buf);
}

static ssize_t tpu_light_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, TPU_LIGHT, buf, size);
}

static DEVICE_ATTR_RW(tpu_light_clk_ratio);

static ssize_t gpu_light_clk_ratio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_show(bcl_dev, GPU_LIGHT, buf);
}

static ssize_t gpu_light_clk_ratio_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return clk_ratio_store(bcl_dev, GPU_LIGHT, buf, size);
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

static ssize_t smpl_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u8 value = 0;
	unsigned int smpl_warn_lvl;

	if (!bcl_dev->s2mpg10_i2c) {
		return -EBUSY;
	}
	if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c,
			     S2MPG10_PM_SMPL_WARN_CTRL, &value)) {
		return -EINVAL;
	}
	value >>= S2MPG10_SMPL_WARN_LVL_SHIFT;
	smpl_warn_lvl = value * 100 + SMPL_LOWER_LIMIT;
	return sysfs_emit(buf, "%dmV\n", smpl_warn_lvl);
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

	if (val < SMPL_LOWER_LIMIT || val > SMPL_UPPER_LIMIT) {
		dev_err(bcl_dev->device, "SMPL_WARN LEVEL %d outside of range %d - %d mV.", val,
			SMPL_LOWER_LIMIT, SMPL_UPPER_LIMIT);
		return -EINVAL;
	}
	if (!bcl_dev->s2mpg10_i2c) {
		dev_err(bcl_dev->device, "S2MPG10 I2C not found\n");
		return -EIO;
	}
	if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, S2MPG10_PM_SMPL_WARN_CTRL, &value)) {
		dev_err(bcl_dev->device, "S2MPG10 read 0x%x failed.", S2MPG10_PM_SMPL_WARN_CTRL);
		return -EBUSY;
	}
	value &= ~SMPL_WARN_MASK;
	value |= ((val - SMPL_LOWER_LIMIT) / 100) << S2MPG10_SMPL_WARN_LVL_SHIFT;
	ret = s2mpg10_write_reg(bcl_dev->s2mpg10_i2c, S2MPG10_PM_SMPL_WARN_CTRL, value);

	if (ret) {
		dev_err(bcl_dev->device, "i2c write error setting smpl_warn\n");
		return ret;
	}
	bcl_dev->triggered_lvl[IRQ_SMPL_WARN] = SMPL_BATTERY_VOLTAGE - val -
			THERMAL_HYST_LEVEL;
	ret = bcl_dev->triggered_tz_irq[IRQ_SMPL_WARN]->ops->set_trip_temp(
			bcl_dev->triggered_tz_irq[IRQ_SMPL_WARN], 0,
			SMPL_BATTERY_VOLTAGE - val);
	if (ret)
		dev_err(bcl_dev->device, "Fail to set smpl_warn trip temp\n");

	return size;

}

static DEVICE_ATTR_RW(smpl_lvl);

static int get_ocp_lvl(struct bcl_device *bcl_dev, u64 *val, u8 addr,
		       u8 pmic, u8 mask, u16 limit,
		       u8 step)
{
	u8 value = 0;
	unsigned int ocp_warn_lvl;

	if (pmic == S2MPG10) {
		if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, addr, &value)) {
			dev_err(bcl_dev->device, "S2MPG10 read 0x%x failed.", addr);
			return -EBUSY;
		}
	} else {
		if (s2mpg11_read_reg(bcl_dev->s2mpg11_i2c, addr, &value)) {
			dev_err(bcl_dev->device, "S2MPG11 read 0x%x failed.", addr);
			return -EBUSY;
		}
	}
	value &= mask;
	ocp_warn_lvl = limit - value * step;
	*val = ocp_warn_lvl;
	return 0;
}

static int set_ocp_lvl(struct bcl_device *bcl_dev, u64 val, u8 addr, u8 pmic, u8 mask,
		       u16 llimit, u16 ulimit, u8 step, u8 id)
{
	u8 value;
	int ret;

	if (val < llimit || val > ulimit) {
		dev_err(bcl_dev->device, "OCP_WARN LEVEL %d outside of range %d - %d mA.", val,
		       llimit, ulimit);
		return -EBUSY;
	}
	if (pmic == S2MPG10) {
		mutex_lock(&bcl_dev->triggered_irq_lock[id]);
		if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, addr, &value)) {
			dev_err(bcl_dev->device, "S2MPG10 read 0x%x failed.", addr);
			mutex_unlock(&bcl_dev->triggered_irq_lock[id]);
			return -EBUSY;
		}
		value &= ~(OCP_WARN_MASK) << S2MPG10_OCP_WARN_LVL_SHIFT;
		value |= ((ulimit - val) / step) << S2MPG10_OCP_WARN_LVL_SHIFT;
		ret = s2mpg10_write_reg(bcl_dev->s2mpg10_i2c, addr, value);
		if (!ret) {
			bcl_dev->triggered_lvl[id] = val - THERMAL_HYST_LEVEL;
			ret = bcl_dev->triggered_tz_irq[id]->ops->set_trip_temp(
					bcl_dev->triggered_tz_irq[id], 0,
					val);
			if (ret)
				dev_err(bcl_dev->device, "Fail to set ocp_warn trip temp\n");
		}
		mutex_unlock(&bcl_dev->triggered_irq_lock[id]);
	} else {
		mutex_lock(&bcl_dev->triggered_irq_lock[id]);
		if (s2mpg11_read_reg(bcl_dev->s2mpg11_i2c, addr, &value)) {
			dev_err(bcl_dev->device, "S2MPG11 read 0x%x failed.", addr);
			mutex_unlock(&bcl_dev->triggered_irq_lock[id]);
			return -EBUSY;
		}
		value &= ~(OCP_WARN_MASK) << S2MPG10_OCP_WARN_LVL_SHIFT;
		value |= ((ulimit - val) / step) << S2MPG11_OCP_WARN_LVL_SHIFT;
		ret = s2mpg11_write_reg(bcl_dev->s2mpg11_i2c, addr, value);
		if (!ret) {
			bcl_dev->triggered_lvl[id] = val - THERMAL_HYST_LEVEL;
			ret = bcl_dev->triggered_tz_irq[id]->ops->set_trip_temp(
					bcl_dev->triggered_tz_irq[id], 0,
					val);
			if (ret)
				dev_err(bcl_dev->device, "Fail to set ocp_warn trip temp\n");
		}
		mutex_unlock(&bcl_dev->triggered_irq_lock[id]);
	}

	if (ret)
		dev_err(bcl_dev->device, "i2c write error setting smpl_warn\n");

	return ret;
}

static ssize_t ocp_cpu1_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, S2MPG10_PM_B3M_OCP_WARN, S2MPG10,
			OCP_WARN_MASK, B3M_UPPER_LIMIT, B3M_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%dmA\n", val);

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

	if (set_ocp_lvl(bcl_dev, value, S2MPG10_PM_B3M_OCP_WARN, S2MPG10, OCP_WARN_MASK,
			B3M_LOWER_LIMIT, B3M_UPPER_LIMIT, B3M_STEP, IRQ_OCP_WARN_CPUCL1) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_cpu1_lvl);

static ssize_t ocp_cpu2_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, S2MPG10_PM_B2M_OCP_WARN, S2MPG10,
			OCP_WARN_MASK, B2M_UPPER_LIMIT, B2M_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%dmA\n", val);

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

	if (set_ocp_lvl(bcl_dev, value, S2MPG10_PM_B2M_OCP_WARN, S2MPG10, OCP_WARN_MASK,
			B2M_LOWER_LIMIT, B2M_UPPER_LIMIT, B2M_STEP, IRQ_OCP_WARN_CPUCL2) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_cpu2_lvl);

static ssize_t ocp_tpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, S2MPG10_PM_B10M_OCP_WARN, S2MPG10,
			OCP_WARN_MASK, B10M_UPPER_LIMIT, B10M_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%dmA\n", val);

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

	if (set_ocp_lvl(bcl_dev, value, S2MPG10_PM_B10M_OCP_WARN, S2MPG10, OCP_WARN_MASK,
			B10M_LOWER_LIMIT, B10M_UPPER_LIMIT, B10M_STEP, IRQ_OCP_WARN_TPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_tpu_lvl);

static ssize_t ocp_gpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, S2MPG11_PM_B2S_OCP_WARN, S2MPG11,
			OCP_WARN_MASK, B2S_UPPER_LIMIT, B2S_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%dmA\n", val);

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

	if (set_ocp_lvl(bcl_dev, value, S2MPG11_PM_B2S_OCP_WARN, S2MPG11, OCP_WARN_MASK,
			B2S_LOWER_LIMIT, B2S_UPPER_LIMIT, B2S_STEP, IRQ_OCP_WARN_GPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_gpu_lvl);

static ssize_t soft_ocp_cpu1_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, S2MPG10_PM_B3M_SOFT_OCP_WARN, S2MPG10,
			OCP_WARN_MASK, B3M_UPPER_LIMIT, B3M_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%dmA\n", val);

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

	if (set_ocp_lvl(bcl_dev, value, S2MPG10_PM_B3M_SOFT_OCP_WARN, S2MPG10, OCP_WARN_MASK,
			B3M_LOWER_LIMIT, B3M_UPPER_LIMIT, B3M_STEP, IRQ_SOFT_OCP_WARN_CPUCL1) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_cpu1_lvl);

static ssize_t soft_ocp_cpu2_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, S2MPG10_PM_B2M_SOFT_OCP_WARN, S2MPG10,
			OCP_WARN_MASK, B2M_UPPER_LIMIT, B2M_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%dmA\n", val);

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

	if (set_ocp_lvl(bcl_dev, value, S2MPG10_PM_B2M_SOFT_OCP_WARN, S2MPG10, OCP_WARN_MASK,
			B2M_LOWER_LIMIT, B2M_UPPER_LIMIT, B2M_STEP, IRQ_SOFT_OCP_WARN_CPUCL2) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_cpu2_lvl);

static ssize_t soft_ocp_tpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, S2MPG10_PM_B10M_SOFT_OCP_WARN, S2MPG10,
			OCP_WARN_MASK, B10M_UPPER_LIMIT, B10M_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%dmA\n", val);

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

	if (set_ocp_lvl(bcl_dev, value, S2MPG10_PM_B10M_SOFT_OCP_WARN, S2MPG10, OCP_WARN_MASK,
			B10M_LOWER_LIMIT, B10M_UPPER_LIMIT, B10M_STEP, IRQ_SOFT_OCP_WARN_TPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_tpu_lvl);

static ssize_t soft_ocp_gpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, S2MPG11_PM_B2S_SOFT_OCP_WARN, S2MPG11,
			OCP_WARN_MASK, B2S_UPPER_LIMIT, B2S_STEP) < 0)
		return -EINVAL;
	return sysfs_emit(buf, "%dmA\n", val);

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

	if (set_ocp_lvl(bcl_dev, value, S2MPG11_PM_B2S_SOFT_OCP_WARN, S2MPG11, OCP_WARN_MASK,
			B2S_LOWER_LIMIT, B2S_UPPER_LIMIT, B2S_STEP, IRQ_SOFT_OCP_WARN_GPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_gpu_lvl);

static struct attribute *triggered_lvl_attrs[] = {
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

static ssize_t offsrc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%#x\n", bcl_dev->offsrc);
}

static DEVICE_ATTR_RO(offsrc);

static ssize_t pwronsrc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%#x\n", bcl_dev->pwronsrc);
}

static DEVICE_ATTR_RO(pwronsrc);

static ssize_t enable_mitigation_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->enabled);
}

static ssize_t enable_mitigation_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	bool value;
	int ret, i;
	void __iomem *addr;
	unsigned int reg;

	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;

	if (bcl_dev->enabled == value)
		return size;

	bcl_dev->enabled = value;
	if (bcl_dev->enabled) {
		bcl_dev->gpu_clkdivstep |= 0x1;
		bcl_dev->tpu_clkdivstep |= 0x1;
		for (i = 0; i < TPU; i++) {
			addr = bcl_dev->base_mem[i] + CLKDIVSTEP;
			mutex_lock(&bcl_dev->ratio_lock);
			reg = __raw_readl(addr);
			__raw_writel(reg | 0x1, addr);
			mutex_unlock(&bcl_dev->ratio_lock);
		}
	} else {
		bcl_dev->gpu_clkdivstep &= ~(1 << 0);
		bcl_dev->tpu_clkdivstep &= ~(1 << 0);
		for (i = 0; i < TPU; i++) {
			addr = bcl_dev->base_mem[i] + CLKDIVSTEP;
			mutex_lock(&bcl_dev->ratio_lock);
			reg = __raw_readl(addr);
			__raw_writel(reg & ~(1 << 0), addr);
			mutex_unlock(&bcl_dev->ratio_lock);
		}
	}
	return size;
}

static DEVICE_ATTR_RW(enable_mitigation);

struct bcl_device *google_retrieve_bcl_handle(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct bcl_device *bcl_dev;

	np = of_find_node_by_name(NULL, "google,mitigation");
	if (!np)
		return NULL;
	pdev = of_find_device_by_node(np);
	if (!pdev)
		return NULL;
	bcl_dev = platform_get_drvdata(pdev);
	if (!bcl_dev)
		return NULL;

	return bcl_dev;
}
EXPORT_SYMBOL_GPL(google_retrieve_bcl_handle);

int google_init_tpu_ratio(struct bcl_device *data)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;

	if (!data->sysreg_cpucl0)
		return -ENOMEM;

	if (!is_subsystem_on(subsystem_pmu[TPU]))
		return -EIO;

	mutex_lock(&data->ratio_lock);
	addr = data->base_mem[TPU] + CPUCL12_CLKDIVSTEP_CON_HEAVY;
	__raw_writel(data->tpu_con_heavy, addr);
	addr = data->base_mem[TPU] + CPUCL12_CLKDIVSTEP_CON_LIGHT;
	__raw_writel(data->tpu_con_light, addr);
	addr = data->base_mem[TPU] + CLKDIVSTEP;
	__raw_writel(data->tpu_clkdivstep, addr);
	data->tpu_clk_stats = __raw_readl(data->base_mem[TPU] + clk_stats_offset[TPU]);
	mutex_unlock(&data->ratio_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_init_tpu_ratio);

int google_init_gpu_ratio(struct bcl_device *data)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;

	if (!data->sysreg_cpucl0)
		return -ENOMEM;

	if (!is_subsystem_on(subsystem_pmu[GPU]))
		return -EIO;

	mutex_lock(&data->ratio_lock);
	addr = data->base_mem[GPU] + CPUCL12_CLKDIVSTEP_CON_HEAVY;
	__raw_writel(data->gpu_con_heavy, addr);
	addr = data->base_mem[GPU] + CPUCL12_CLKDIVSTEP_CON_LIGHT;
	__raw_writel(data->gpu_con_light, addr);
	addr = data->base_mem[GPU] + CLKDIVSTEP;
	__raw_writel(data->gpu_clkdivstep, addr);
	data->gpu_clk_stats = __raw_readl(data->base_mem[GPU] + clk_stats_offset[GPU]);
	mutex_unlock(&data->ratio_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_init_gpu_ratio);

unsigned int google_get_ppm(struct bcl_device *data)
{
	void __iomem *addr;
	unsigned int reg;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		pr_err("Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	addr = data->sysreg_cpucl0 + CLUSTER0_PPM;
	reg = __raw_readl(addr);
	mutex_unlock(&sysreg_lock);

	return reg;
}
EXPORT_SYMBOL_GPL(google_get_ppm);

unsigned int google_get_mpmm(struct bcl_device *data)
{
	void __iomem *addr;
	unsigned int reg;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		pr_err("Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	addr = data->sysreg_cpucl0 + CLUSTER0_MPMM;
	reg = __raw_readl(addr);
	mutex_unlock(&sysreg_lock);

	return reg;
}
EXPORT_SYMBOL_GPL(google_get_mpmm);

int google_set_ppm(struct bcl_device *data, unsigned int value)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		pr_err("Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	addr = data->sysreg_cpucl0 + CLUSTER0_PPM;
	__raw_writel(value, addr);
	mutex_unlock(&sysreg_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_set_ppm);

int google_set_mpmm(struct bcl_device *data, unsigned int value)
{
	void __iomem *addr;

	if (!data)
		return -ENOMEM;
	if (!data->sysreg_cpucl0) {
		pr_err("Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	mutex_lock(&sysreg_lock);
	addr = data->sysreg_cpucl0 + CLUSTER0_MPMM;
	__raw_writel(value, addr);
	mutex_unlock(&sysreg_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_set_mpmm);

static ssize_t mpmm_settings_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	void __iomem *addr;
	int value;
	int ret;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_MPMM;
	__raw_writel(value, addr);
	mutex_unlock(&sysreg_lock);

	return size;
}

static ssize_t mpmm_settings_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int reg = 0;
	void __iomem *addr;

	if (!bcl_dev->sysreg_cpucl0)
		return -EIO;

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_MPMM;
	reg = __raw_readl(addr);
	mutex_unlock(&sysreg_lock);
	return sysfs_emit(buf, "0x%x\n", reg);
}

static DEVICE_ATTR_RW(mpmm_settings);

static ssize_t ppm_settings_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	void __iomem *addr;
	int value;
	int ret;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM;
	__raw_writel(value, addr);
	mutex_unlock(&sysreg_lock);

	return size;
}

static ssize_t ppm_settings_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int reg = 0;
	void __iomem *addr;

	if (!bcl_dev->sysreg_cpucl0)
		return -EIO;

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM;
	reg = __raw_readl(addr);
	mutex_unlock(&sysreg_lock);
	return sysfs_emit(buf, "0x%x\n", reg);
}

static DEVICE_ATTR_RW(ppm_settings);

static struct attribute *instr_attrs[] = {
	&dev_attr_mpmm_settings.attr,
	&dev_attr_ppm_settings.attr,
	&dev_attr_enable_mitigation.attr,
	&dev_attr_offsrc.attr,
	&dev_attr_pwronsrc.attr,
	NULL,
};

static const struct attribute_group instr_group = {
	.attrs = instr_attrs,
	.name = "instruction",
};

static int google_bcl_register_irq(struct bcl_device *bcl_dev,
				  int id, irq_handler_t thread_fn,
				  struct device *dev,
				  const struct thermal_zone_of_device_ops *ops,
				  const char *devname, u32 intr_flag)
{
	int ret = 0;

	ret = devm_request_threaded_irq(dev,
					bcl_dev->triggered_irq[id],
					NULL, thread_fn,
					intr_flag | IRQF_ONESHOT,
					devname, bcl_dev);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ: %d: %d\n",
			bcl_dev->triggered_irq[id], ret);
		return ret;
	}
	if (!ops)
		return -EINVAL;

	if (id < IRQ_OCP_WARN_GPU)
		bcl_dev->triggered_tz_irq[id] = thermal_zone_of_sensor_register(dev, id, bcl_dev,
										ops);
	else if ((id >= IRQ_OCP_WARN_GPU) && (id < IRQ_PMIC_120C))
		bcl_dev->triggered_tz_irq[id] =
				thermal_zone_of_sensor_register(dev, id - IRQ_OCP_WARN_GPU,
								bcl_dev, ops);
	else
		bcl_dev->triggered_tz_irq[id] =
				thermal_zone_of_sensor_register(bcl_dev->device,
								id - IRQ_PMIC_120C + 1,
								bcl_dev, ops);
	if (IS_ERR(bcl_dev->triggered_tz_irq[id])) {
		dev_err(bcl_dev->device, "TZ register failed. %d, err:%ld\n", id,
			PTR_ERR(bcl_dev->triggered_tz_irq[id]));
	} else {
		thermal_zone_device_enable(bcl_dev->triggered_tz_irq[id]);
		thermal_zone_device_update(bcl_dev->triggered_tz_irq[id],
		   THERMAL_DEVICE_UP);
	}
	return ret;
}

static void google_set_throttling(struct bcl_device *bcl_dev)
{
	struct device_node *np = bcl_dev->device->of_node;
	int ret;
	u32 val, ppm_settings, mpmm_settings;
	void __iomem *addr;

	if (!bcl_dev->sysreg_cpucl0) {
		dev_err(bcl_dev->device, "sysreg_cpucl0 ioremap not mapped\n");
		return;
	}
	ret = of_property_read_u32(np, "ppm_settings", &val);
	ppm_settings = ret ? 0 : val;

	ret = of_property_read_u32(np, "mpmm_settings", &val);
	mpmm_settings = ret ? 0 : val;

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM;
	__raw_writel(ppm_settings, addr);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_MPMM;
	__raw_writel(mpmm_settings, addr);
	mutex_unlock(&sysreg_lock);

}

static int google_set_sub_pmic(struct bcl_device *bcl_dev)
{
	struct s2mpg11_platform_data *pdata_s2mpg11;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct s2mpg11_dev *s2mpg11 = NULL;
	struct i2c_client *i2c;
	u8 val = 0;
	int ret;

	p_np = of_parse_phandle(np, "google,sub-power", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find sub-power I2C\n");
			return -ENODEV;
		}
		s2mpg11 = i2c_get_clientdata(i2c);
	}
	of_node_put(p_np);
	if (!s2mpg11) {
		dev_err(bcl_dev->device, "S2MPG11 device not found\n");
		return -ENODEV;
	}
	pdata_s2mpg11 = dev_get_platdata(s2mpg11->dev);
	bcl_dev->s2mpg11_i2c = s2mpg11->pmic;
	bcl_dev->triggered_lvl[IRQ_OCP_WARN_GPU] = B2S_UPPER_LIMIT - THERMAL_HYST_LEVEL -
			(pdata_s2mpg11->b2_ocp_warn_lvl * B2S_STEP);
	bcl_dev->triggered_lvl[IRQ_SOFT_OCP_WARN_GPU] = B2S_UPPER_LIMIT - THERMAL_HYST_LEVEL -
			(pdata_s2mpg11->b2_soft_ocp_warn_lvl * B2S_STEP);
	bcl_dev->triggered_pin[IRQ_OCP_WARN_GPU] = pdata_s2mpg11->b2_ocp_warn_pin;
	bcl_dev->triggered_pin[IRQ_SOFT_OCP_WARN_GPU] = pdata_s2mpg11->b2_soft_ocp_warn_pin;
	bcl_dev->triggered_irq[IRQ_OCP_WARN_GPU] = gpio_to_irq(pdata_s2mpg11->b2_ocp_warn_pin);
	bcl_dev->triggered_irq[IRQ_SOFT_OCP_WARN_GPU] =
			gpio_to_irq(pdata_s2mpg11->b2_soft_ocp_warn_pin);
	if (s2mpg11_read_reg(bcl_dev->s2mpg11_i2c, S2MPG11_COMMON_CHIPID, &val)) {
		dev_err(bcl_dev->device, "Failed to read S2MPG11 chipid.\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_OCP_WARN_GPU,
				     google_gpu_ocp_warn_irq_handler,
				     s2mpg11->dev,
				     &google_ocp_gpu_ops, "GPU_OCP_IRQ", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_OCP_WARN_GPU);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_SOFT_OCP_WARN_GPU,
				     google_soft_gpu_ocp_warn_irq_handler,
				     s2mpg11->dev,
				     &google_soft_ocp_gpu_ops, "SOFT_GPU_OCP_IRQ",
				     IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_SOFT_OCP_WARN_GPU);
		return -ENODEV;
	}
	return 0;
}

static int google_set_main_pmic(struct bcl_device *bcl_dev)
{
	struct s2mpg10_platform_data *pdata_s2mpg10;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct s2mpg10_dev *s2mpg10 = NULL;
	struct i2c_client *i2c;
	bool bypass_smpl_warn = false;
	u8 val = 0;
	int ret;

	p_np = of_parse_phandle(np, "google,main-power", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find main-power I2C\n");
			return -ENODEV;
		}
		s2mpg10 = i2c_get_clientdata(i2c);
	}
	of_node_put(p_np);
	if (!s2mpg10) {
		dev_err(bcl_dev->device, "S2MPG10 device not found\n");
		return -ENODEV;
	}
	pdata_s2mpg10 = dev_get_platdata(s2mpg10->dev);
	/* request smpl_warn interrupt */
	if (!gpio_is_valid(pdata_s2mpg10->smpl_warn_pin)) {
		dev_err(bcl_dev->device, "smpl_warn GPIO NOT VALID\n");
		devm_free_irq(bcl_dev->device, bcl_dev->triggered_irq[IRQ_SMPL_WARN], bcl_dev);
		bypass_smpl_warn = true;
	}
	bcl_dev->s2mpg10_i2c = s2mpg10->pmic;
	/* clear S2MPG_10 information every boot */
	/* see b/166671802#comment34 and b/195455000 */
	s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, S2MPG10_PM_OFFSRC, &val);
	pr_info("S2MPG10 OFFSRC : %#x\n", val);
	bcl_dev->offsrc = val;
	s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, S2MPG10_PM_PWRONSRC, &val);
	pr_info("S2MPG10 PWRONSRC: %#x\n", val);
	bcl_dev->pwronsrc = val;
	s2mpg10_write_reg(bcl_dev->s2mpg10_i2c, S2MPG10_PM_OFFSRC, 0);
	s2mpg10_write_reg(bcl_dev->s2mpg10_i2c, S2MPG10_PM_PWRONSRC, 0);
	bcl_dev->triggered_irq[IRQ_SMPL_WARN] = gpio_to_irq(pdata_s2mpg10->smpl_warn_pin);
	irq_set_status_flags(bcl_dev->triggered_irq[IRQ_SMPL_WARN], IRQ_DISABLE_UNLAZY);
	bcl_dev->triggered_pin[IRQ_SMPL_WARN] = pdata_s2mpg10->smpl_warn_pin;
	bcl_dev->triggered_lvl[IRQ_SMPL_WARN] = SMPL_BATTERY_VOLTAGE -
			(pdata_s2mpg10->smpl_warn_lvl * SMPL_STEP + SMPL_LOWER_LIMIT);
	bcl_dev->triggered_lvl[IRQ_OCP_WARN_CPUCL1] = B3M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_s2mpg10->b3_ocp_warn_lvl * B3M_STEP);
	bcl_dev->triggered_lvl[IRQ_SOFT_OCP_WARN_CPUCL1] = B3M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_s2mpg10->b3_soft_ocp_warn_lvl * B3M_STEP);
	bcl_dev->triggered_lvl[IRQ_OCP_WARN_CPUCL2] = B2M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_s2mpg10->b2_ocp_warn_lvl * B2M_STEP);
	bcl_dev->triggered_lvl[IRQ_SOFT_OCP_WARN_CPUCL2] = B2M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_s2mpg10->b2_soft_ocp_warn_lvl * B2M_STEP);
	bcl_dev->triggered_lvl[IRQ_OCP_WARN_TPU] = B10M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_s2mpg10->b10_ocp_warn_lvl * B10M_STEP);
	bcl_dev->triggered_lvl[IRQ_SOFT_OCP_WARN_TPU] = B10M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_s2mpg10->b10_soft_ocp_warn_lvl * B10M_STEP);
	bcl_dev->triggered_lvl[IRQ_PMIC_120C] = PMIC_120C_UPPER_LIMIT - THERMAL_HYST_LEVEL;
	bcl_dev->triggered_lvl[IRQ_PMIC_140C] = PMIC_140C_UPPER_LIMIT - THERMAL_HYST_LEVEL;
	bcl_dev->triggered_lvl[IRQ_PMIC_OVERHEAT] = PMIC_OVERHEAT_UPPER_LIMIT - THERMAL_HYST_LEVEL;
	bcl_dev->triggered_pin[IRQ_OCP_WARN_CPUCL1] = pdata_s2mpg10->b3_ocp_warn_pin;
	bcl_dev->triggered_pin[IRQ_OCP_WARN_CPUCL2] = pdata_s2mpg10->b2_ocp_warn_pin;
	bcl_dev->triggered_pin[IRQ_SOFT_OCP_WARN_CPUCL1] = pdata_s2mpg10->b3_soft_ocp_warn_pin;
	bcl_dev->triggered_pin[IRQ_SOFT_OCP_WARN_CPUCL2] = pdata_s2mpg10->b2_soft_ocp_warn_pin;
	bcl_dev->triggered_pin[IRQ_OCP_WARN_TPU] = pdata_s2mpg10->b10_ocp_warn_pin;
	bcl_dev->triggered_pin[IRQ_SOFT_OCP_WARN_TPU] = pdata_s2mpg10->b10_soft_ocp_warn_pin;
	bcl_dev->triggered_irq[IRQ_OCP_WARN_CPUCL1] = gpio_to_irq(pdata_s2mpg10->b3_ocp_warn_pin);
	bcl_dev->triggered_irq[IRQ_OCP_WARN_CPUCL2] = gpio_to_irq(pdata_s2mpg10->b2_ocp_warn_pin);
	bcl_dev->triggered_irq[IRQ_SOFT_OCP_WARN_CPUCL1] =
			gpio_to_irq(pdata_s2mpg10->b3_soft_ocp_warn_pin);
	bcl_dev->triggered_irq[IRQ_SOFT_OCP_WARN_CPUCL2] =
			gpio_to_irq(pdata_s2mpg10->b2_soft_ocp_warn_pin);
	bcl_dev->triggered_irq[IRQ_OCP_WARN_TPU] = gpio_to_irq(pdata_s2mpg10->b10_ocp_warn_pin);
	bcl_dev->triggered_irq[IRQ_SOFT_OCP_WARN_TPU] =
			gpio_to_irq(pdata_s2mpg10->b10_soft_ocp_warn_pin);
	bcl_dev->triggered_irq[IRQ_PMIC_120C] = pdata_s2mpg10->irq_base + S2MPG10_IRQ_120C_INT3;
	bcl_dev->triggered_irq[IRQ_PMIC_140C] = pdata_s2mpg10->irq_base + S2MPG10_IRQ_140C_INT3;
	bcl_dev->triggered_irq[IRQ_PMIC_OVERHEAT] = pdata_s2mpg10->irq_base + S2MPG10_IRQ_TSD_INT3;
	if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, S2MPG10_COMMON_CHIPID, &val)) {
		dev_err(bcl_dev->device, "Failed to read S2MPG10 chipid.\n");
		return -ENODEV;
	}
	if (!bypass_smpl_warn) {
		ret = google_bcl_register_irq(bcl_dev,
					     IRQ_SMPL_WARN,
					     google_smpl_warn_irq_handler,
					     s2mpg10->dev,
					     &google_smpl_warn_ops,
					     "SMPL_WARN_IRQ", IRQF_TRIGGER_FALLING);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_SMPL_WARN);
			return -ENODEV;
		}
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_OCP_WARN_CPUCL1,
				     google_cpu1_ocp_warn_irq_handler,
				     s2mpg10->dev,
				     &google_ocp_cpu1_ops, "CPU1_OCP_IRQ", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_OCP_WARN_CPUCL1);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_OCP_WARN_CPUCL2,
				     google_cpu2_ocp_warn_irq_handler,
				     s2mpg10->dev,
				     &google_ocp_cpu2_ops, "CPU2_OCP_IRQ", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_OCP_WARN_CPUCL2);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_SOFT_OCP_WARN_CPUCL1,
				     google_soft_cpu1_ocp_warn_irq_handler,
				     s2mpg10->dev,
				     &google_soft_ocp_cpu1_ops, "SOFT_CPU1_OCP_IRQ",
				     IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_SOFT_OCP_WARN_CPUCL1);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_SOFT_OCP_WARN_CPUCL2,
				     google_soft_cpu2_ocp_warn_irq_handler,
				     s2mpg10->dev,
				     &google_soft_ocp_cpu2_ops, "SOFT_CPU2_OCP_IRQ",
				     IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_SOFT_OCP_WARN_CPUCL2);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_OCP_WARN_TPU,
				     google_tpu_ocp_warn_irq_handler,
				     s2mpg10->dev,
				     &google_ocp_tpu_ops, "TPU_OCP_IRQ", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_OCP_WARN_TPU);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_SOFT_OCP_WARN_TPU,
				     google_soft_tpu_ocp_warn_irq_handler,
				     s2mpg10->dev,
				     &google_soft_ocp_tpu_ops, "SOFT_TPU_OCP_IRQ",
				     IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail:%d\n", IRQ_SOFT_OCP_WARN_TPU);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_PMIC_120C,
				     google_pmic_120c_irq_handler,
				     bcl_dev->device,
				     &google_pmic_120c_ops, "PMIC_120C", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_pmic_register fail:%d\n", IRQ_PMIC_120C);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_PMIC_140C,
				     google_pmic_140c_irq_handler,
				     bcl_dev->device,
				     &google_pmic_140c_ops, "PMIC_140C", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_pmic_register fail:%d\n", IRQ_PMIC_140C);
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev,
				     IRQ_PMIC_OVERHEAT,
				     google_tsd_overheat_irq_handler,
				     bcl_dev->device,
				     &google_pmic_overheat_ops, "THERMAL_OVERHEAT",
				     IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_pmic_register fail:%d\n", IRQ_PMIC_OVERHEAT);
		return -ENODEV;
	}

	return 0;

}

const struct attribute_group *mitigation_groups[] = {
	&instr_group,
	&triggered_lvl_group,
	&clock_div_group,
	&clock_ratio_group,
	&clock_stats_group,
	&triggered_count_group,
	&triggered_timestamp_group,
	&triggered_capacity_group,
	&triggered_voltage_group,
	NULL,
};

static int google_init_fs(struct bcl_device *bcl_dev)
{
	bcl_dev->mitigation_dev = pmic_subdevice_create(NULL, mitigation_groups,
							bcl_dev, "mitigation");
	if (IS_ERR(bcl_dev->mitigation_dev))
		return -ENODEV;

	return 0;
}

static int google_bcl_probe(struct platform_device *pdev)
{
	unsigned int reg;
	int ret = 0, i;
	struct bcl_device *bcl_dev;
	u32 val;
	struct device_node *np;

	bcl_dev = devm_kzalloc(&pdev->dev, sizeof(*bcl_dev), GFP_KERNEL);
	if (!bcl_dev)
		return -ENOMEM;
	bcl_dev->device = &pdev->dev;
	bcl_dev->iodev = dev_get_drvdata(pdev->dev.parent);
	np = bcl_dev->device->of_node;
	bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	bcl_dev->soc_tzd = thermal_zone_of_sensor_register(bcl_dev->device, PMIC_SOC, bcl_dev,
							   &bcl_dev->soc_ops);
	if (IS_ERR(bcl_dev->soc_tzd)) {
		dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
			PTR_ERR(bcl_dev->soc_tzd));
		ret = PTR_ERR(bcl_dev->soc_tzd);
		bcl_dev->soc_tzd = NULL;
	}
	INIT_DELAYED_WORK(&bcl_dev->soc_eval_work, google_bcl_evaluate_soc);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_SMPL_WARN], google_smpl_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_OCP_WARN_CPUCL1], google_cpu1_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_SOFT_OCP_WARN_CPUCL1],
			  google_soft_cpu1_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_OCP_WARN_CPUCL2], google_cpu2_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_SOFT_OCP_WARN_CPUCL2],
			  google_soft_cpu2_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_OCP_WARN_TPU], google_tpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_SOFT_OCP_WARN_TPU],
			  google_soft_tpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_PMIC_120C], google_pmic_120c_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_PMIC_140C], google_pmic_140c_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_PMIC_OVERHEAT],
			  google_pmic_overheat_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_OCP_WARN_GPU], google_gpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->triggered_irq_work[IRQ_SOFT_OCP_WARN_GPU],
			  google_soft_gpu_warn_work);
	platform_set_drvdata(pdev, bcl_dev);
	bcl_dev->base_mem[CPU0] = devm_ioremap(bcl_dev->device, CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU0]) {
		dev_err(bcl_dev->device, "cpu0_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	bcl_dev->base_mem[CPU1] = devm_ioremap(bcl_dev->device, CPUCL1_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU1]) {
		dev_err(bcl_dev->device, "cpu1_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	bcl_dev->base_mem[CPU2] = devm_ioremap(bcl_dev->device, CPUCL2_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU2]) {
		dev_err(bcl_dev->device, "cpu2_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	bcl_dev->base_mem[TPU] = devm_ioremap(bcl_dev->device, TPU_BASE, SZ_8K);
	if (!bcl_dev->base_mem[TPU]) {
		dev_err(bcl_dev->device, "tpu_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	bcl_dev->base_mem[GPU] = devm_ioremap(bcl_dev->device, G3D_BASE, SZ_8K);
	if (!bcl_dev->base_mem[GPU]) {
		dev_err(bcl_dev->device, "gpu_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	bcl_dev->sysreg_cpucl0 = devm_ioremap(bcl_dev->device, SYSREG_CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->sysreg_cpucl0) {
		dev_err(bcl_dev->device, "sysreg_cpucl0 ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}

	mutex_lock(&sysreg_lock);
	reg = __raw_readl(bcl_dev->sysreg_cpucl0 + CLUSTER0_GENERAL_CTRL_64);
	reg |= MPMMEN_MASK;
	__raw_writel(reg, bcl_dev->sysreg_cpucl0 + CLUSTER0_GENERAL_CTRL_64);
	reg = __raw_readl(bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM);
	reg |= PPMEN_MASK;
	__raw_writel(reg, bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM);

	mutex_unlock(&sysreg_lock);
	mutex_init(&bcl_dev->state_trans_lock);
	mutex_init(&bcl_dev->ratio_lock);
	bcl_dev->soc_ops.get_temp = google_bcl_read_soc;
	bcl_dev->soc_ops.set_trips = google_bcl_set_soc;
	for (i = 0; i < IFPMIC_SENSOR_MAX; i++) {
		atomic_set(&bcl_dev->if_triggered_cnt[i], 0);
	}
	for (i = 0; i < IRQ_TRIGGERED_SOURCE_MAX; i++) {
		bcl_dev->triggered_counter[i] = 0;
		atomic_set(&bcl_dev->triggered_cnt[i], 0);
		mutex_init(&bcl_dev->triggered_irq_lock[i]);
	}
	if (!IS_ERR(bcl_dev->soc_tzd)) {
		bcl_dev->psy_nb.notifier_call = battery_supply_callback;
		ret = power_supply_reg_notifier(&bcl_dev->psy_nb);
		if (ret < 0)
			dev_err(bcl_dev->device,
				"soc notifier registration error. defer. err:%d\n", ret);
		thermal_zone_device_update(bcl_dev->soc_tzd, THERMAL_DEVICE_UP);
	}
	google_set_throttling(bcl_dev);
	google_set_main_pmic(bcl_dev);
	google_set_sub_pmic(bcl_dev);
	ret = of_property_read_u32(np, "tpu_con_heavy", &val);
	bcl_dev->tpu_con_heavy = ret ? 0 : val;
	ret = of_property_read_u32(np, "tpu_con_light", &val);
	bcl_dev->tpu_con_light = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_con_heavy", &val);
	bcl_dev->gpu_con_heavy = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_con_light", &val);
	bcl_dev->gpu_con_light = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_clkdivstep", &val);
	bcl_dev->gpu_clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "tpu_clkdivstep", &val);
	bcl_dev->tpu_clkdivstep = ret ? 0 : val;
	bcl_dev->batt_psy_initialized = false;

	ret = google_init_fs(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;
	bcl_dev->enabled = true;
	return 0;

bcl_soc_probe_exit:
	google_bcl_remove_thermal(bcl_dev);
	return ret;
}

static int google_bcl_remove(struct platform_device *pdev)
{
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	pmic_device_destroy(bcl_dev->mitigation_dev->devt);
	google_bcl_remove_thermal(bcl_dev);

	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "google,google-bcl"},
	{},
};

static struct platform_driver google_bcl_driver = {
	.probe  = google_bcl_probe,
	.remove = google_bcl_remove,
	.id_table = google_id_table,
	.driver = {
		.name           = "google_mitigation",
		.owner          = THIS_MODULE,
		.of_match_table = match_table,
	},
};

module_platform_driver(google_bcl_driver);
MODULE_DESCRIPTION("Google Battery Current Limiter");
MODULE_AUTHOR("George Lee <geolee@google.com>");
MODULE_LICENSE("GPL");
