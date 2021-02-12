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
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
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
#define BUF_SIZE 192
#define PMU_ALIVE_CPU1_OUT (0x1D20)
#define PMU_ALIVE_CPU2_OUT (0x1DA0)
#define PMU_ALIVE_TPU_OUT (0x2920)
#define PMU_ALIVE_GPU_OUT (0x1E20)

#define BCL_DEBUG_ATTRIBUTE(name, fn_read, fn_write) \
static const struct file_operations name = {	\
	.open	= simple_open,			\
	.llseek	= no_llseek,			\
	.read	= fn_read,			\
	.write	= fn_write,			\
}

enum PMIC_THERMAL_SENSOR {
	PMIC_SOC,
	PMIC_120C,
	PMIC_140C,
	PMIC_OVERHEAT,
	PMIC_THERMAL_SENSOR_MAX,
};

enum IRQ_SOURCE_S2MPG10 {
	IRQ_SMPL_WARN,
	IRQ_OCP_WARN_CPUCL1,
	IRQ_OCP_WARN_CPUCL2,
	IRQ_SOFT_OCP_WARN_CPUCL1,
	IRQ_SOFT_OCP_WARN_CPUCL2,
	IRQ_OCP_WARN_TPU,
	IRQ_SOFT_OCP_WARN_TPU,
	IRQ_PMIC_120C,
	IRQ_PMIC_140C,
	IRQ_PMIC_OVERHEAT,
	IRQ_SOURCE_S2MPG10_MAX,
};

enum IRQ_SOURCE_S2MPG11 {
	IRQ_OCP_WARN_GPU,
	IRQ_SOFT_OCP_WARN_GPU,
	IRQ_SOURCE_S2MPG11_MAX,
};

enum sys_throttling_switch {
	SYS_THROTTLING_DISABLED,
	SYS_THROTTLING_ENABLED,
	SYS_THROTTLING_GEAR0,
	SYS_THROTTLING_GEAR1,
	SYS_THROTTLING_GEAR2,
	SYS_THROTTLING_MAX,
};

enum PMIC_REG { S2MPG10, S2MPG11 };

struct gs101_bcl_dev {
	struct device *device;
	struct dentry *debug_entry;
	void __iomem *cpu0_mem;
	void __iomem *cpu1_mem;
	void __iomem *cpu2_mem;
	void __iomem *gpu_mem;
	void __iomem *tpu_mem;
	void __iomem *sysreg_cpucl0;

	struct notifier_block psy_nb;
	struct delayed_work soc_eval_work;
	struct delayed_work mfd_init;

	void *iodev;

	int trip_high_temp;
	int trip_low_temp;
	int trip_val;
	struct mutex state_trans_lock;
	struct mutex ratio_lock;
	struct thermal_zone_device *soc_tzd;
	struct thermal_zone_of_device_ops soc_ops;
	struct mutex s2mpg10_irq_lock[IRQ_SOURCE_S2MPG10_MAX];
	struct mutex s2mpg11_irq_lock[IRQ_SOURCE_S2MPG11_MAX];
	struct delayed_work s2mpg10_irq_work[IRQ_SOURCE_S2MPG10_MAX];
	struct delayed_work s2mpg11_irq_work[IRQ_SOURCE_S2MPG11_MAX];
	struct thermal_zone_device *s2mpg10_tz_irq[IRQ_SOURCE_S2MPG10_MAX];
	struct thermal_zone_device *s2mpg11_tz_irq[IRQ_SOURCE_S2MPG11_MAX];

	unsigned int s2mpg10_lvl[IRQ_SOURCE_S2MPG10_MAX];
	unsigned int s2mpg11_lvl[IRQ_SOURCE_S2MPG11_MAX];
	unsigned int s2mpg10_irq[IRQ_SOURCE_S2MPG10_MAX];
	unsigned int s2mpg11_irq[IRQ_SOURCE_S2MPG11_MAX];
	int s2mpg10_counter[IRQ_SOURCE_S2MPG10_MAX];
	int s2mpg11_counter[IRQ_SOURCE_S2MPG11_MAX];
	int s2mpg10_pin[IRQ_SOURCE_S2MPG10_MAX];
	int s2mpg11_pin[IRQ_SOURCE_S2MPG11_MAX];

	struct s2mpg10_dev *s2mpg10;
	struct s2mpg11_dev *s2mpg11;

	struct i2c_client *s2mpg10_i2c;
	struct i2c_client *s2mpg11_i2c;

	unsigned int s2mpg10_triggered_irq[IRQ_SOURCE_S2MPG10_MAX];
	unsigned int s2mpg11_triggered_irq[IRQ_SOURCE_S2MPG11_MAX];
};


static const struct platform_device_id google_gs101_id_table[] = {
	{.name = "gs101-bcl-m",},
	{.name = "gs101-bcl-s",},
	{},
};

DEFINE_MUTEX(sysreg_lock);

static bool is_subsystem_on(unsigned int addr)
{
	unsigned int value;

	exynos_pmu_read(addr, &value);
	return ((value & 0xF) == 0x4);
}

static int s2mpg10_read_level(void *data, int *val, int id)
{
	struct gs101_bcl_dev *gs101_bcl_device = data;

	if ((gs101_bcl_device->s2mpg10_counter[id] != 0) &&
	    (gs101_bcl_device->s2mpg10_counter[id] < THERMAL_IRQ_COUNTER_LIMIT)) {
		*val = gs101_bcl_device->s2mpg10_lvl[id] +
				THERMAL_HYST_LEVEL;
		gs101_bcl_device->s2mpg10_counter[id] += 1;
	} else {
		*val = gs101_bcl_device->s2mpg10_lvl[id];
		gs101_bcl_device->s2mpg10_counter[id] = 0;
	}
	return 0;
}

static int s2mpg11_read_level(void *data, int *val, int id)
{
	struct gs101_bcl_dev *gs101_bcl_device = data;

	if (gs101_bcl_device->s2mpg11_counter[id] != 0) {
		*val = gs101_bcl_device->s2mpg11_lvl[id] +
				THERMAL_HYST_LEVEL;
	} else {
		*val = gs101_bcl_device->s2mpg11_lvl[id];
	}
	return 0;
}

static void irq_work(struct gs101_bcl_dev *gs101_bcl_device, u8 active_pull, u8 idx, u8 pmic)
{
	int state = !active_pull;

	if (pmic == S2MPG10) {
		mutex_lock(&gs101_bcl_device->s2mpg10_irq_lock[idx]);
		state = gpio_get_value(gs101_bcl_device->s2mpg10_pin[idx]);
		if (state == active_pull) {
			gs101_bcl_device->s2mpg10_triggered_irq[idx] = 1;
			queue_delayed_work(system_wq, &gs101_bcl_device->s2mpg10_irq_work[idx],
			 msecs_to_jiffies(300));
		} else {
			gs101_bcl_device->s2mpg10_triggered_irq[idx] = 0;
			gs101_bcl_device->s2mpg10_counter[idx] = 0;
			enable_irq(gs101_bcl_device->s2mpg10_irq[idx]);
		}
		mutex_unlock(&gs101_bcl_device->s2mpg10_irq_lock[idx]);
	} else {
		mutex_lock(&gs101_bcl_device->s2mpg11_irq_lock[idx]);
		state = gpio_get_value(gs101_bcl_device->s2mpg11_pin[idx]);
		if (state == active_pull) {
			gs101_bcl_device->s2mpg11_triggered_irq[idx] = 1;
			queue_delayed_work(system_wq, &gs101_bcl_device->s2mpg11_irq_work[idx],
			 msecs_to_jiffies(300));
		} else {
			gs101_bcl_device->s2mpg11_triggered_irq[idx] = 0;
			gs101_bcl_device->s2mpg11_counter[idx] = 0;
			enable_irq(gs101_bcl_device->s2mpg11_irq[idx]);
		}
		mutex_unlock(&gs101_bcl_device->s2mpg11_irq_lock[idx]);
	}
}

static irqreturn_t irq_handler(int irq, void *data, u8 pmic, u8 idx, u8 active_pull)
{
	struct gs101_bcl_dev *gs101_bcl_device = data;

	if (pmic == S2MPG10) {
		mutex_lock(&gs101_bcl_device->s2mpg10_irq_lock[idx]);
		gs101_bcl_device->s2mpg10_triggered_irq[idx] = 1;
		disable_irq_nosync(gs101_bcl_device->s2mpg10_irq[idx]);
		queue_delayed_work(system_wq, &gs101_bcl_device->s2mpg10_irq_work[idx],
				   msecs_to_jiffies(300));
		mutex_unlock(&gs101_bcl_device->s2mpg10_irq_lock[idx]);
		pr_info_ratelimited("S2MPG10 IRQ : %d triggered\n", irq);
		if (gs101_bcl_device->s2mpg10_counter[idx] == 0) {
			gs101_bcl_device->s2mpg10_counter[idx] += 1;

			/* Minimize the amount of thermal update by only triggering
			 * update every THERMAL_IRQ_COUNTER_LIMIT IRQ triggered.
			 */
			if (gs101_bcl_device->s2mpg10_tz_irq[idx])
				thermal_zone_device_update(
						gs101_bcl_device->s2mpg10_tz_irq[idx],
						THERMAL_EVENT_UNSPECIFIED);
		}
	} else {
		mutex_lock(&gs101_bcl_device->s2mpg11_irq_lock[idx]);
		gs101_bcl_device->s2mpg11_triggered_irq[idx] = 1;
		disable_irq_nosync(gs101_bcl_device->s2mpg11_irq[idx]);
		queue_delayed_work(system_wq, &gs101_bcl_device->s2mpg11_irq_work[idx],
				   msecs_to_jiffies(300));
		mutex_unlock(&gs101_bcl_device->s2mpg11_irq_lock[idx]);
		pr_info_ratelimited("S2MPG11 IRQ : %d triggered\n", irq);
		if (gs101_bcl_device->s2mpg11_counter[idx] == 0) {
			gs101_bcl_device->s2mpg11_counter[idx] = 1;

			/* Minimize the amount of thermal update by only triggering
			 * update every THERMAL_IRQ_COUNTER_LIMIT IRQ triggered.
			 */
			if (gs101_bcl_device->s2mpg11_tz_irq[idx])
				thermal_zone_device_update(
						gs101_bcl_device->s2mpg11_tz_irq[idx],
						THERMAL_EVENT_UNSPECIFIED);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t gs101_smpl_warn_irq_handler(int irq, void *data)
{
	struct gs101_bcl_dev *gs101_bcl_device = data;

	if (!gs101_bcl_device)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_SMPL_WARN, ACTIVE_LOW);
}

static void gs101_smpl_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
	    container_of(work, struct gs101_bcl_dev, s2mpg10_irq_work[IRQ_SMPL_WARN].work);

	irq_work(gs101_bcl_device, ACTIVE_LOW, IRQ_SMPL_WARN, S2MPG10);
}

static int smpl_warn_read_voltage(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_SMPL_WARN);
}

static const struct thermal_zone_of_device_ops gs101_smpl_warn_ops = {
	.get_temp = smpl_warn_read_voltage,
};

static void gs101_cpu1_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_OCP_WARN_CPUCL1].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_OCP_WARN_CPUCL1, S2MPG10);
}

static irqreturn_t gs101_cpu1_ocp_warn_irq_handler(int irq, void *data)
{
	struct gs101_bcl_dev *gs101_bcl_device = data;

	if (!gs101_bcl_device)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_OCP_WARN_CPUCL1, ACTIVE_HIGH);
}

static int ocp_cpu1_read_current(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_OCP_WARN_CPUCL1);
}

static const struct thermal_zone_of_device_ops gs101_ocp_cpu1_ops = {
	.get_temp = ocp_cpu1_read_current,
};

static void gs101_cpu2_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_OCP_WARN_CPUCL2].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_OCP_WARN_CPUCL2, S2MPG10);
}

static irqreturn_t gs101_cpu2_ocp_warn_irq_handler(int irq, void *data)
{
	struct gs101_bcl_dev *gs101_bcl_device = data;

	if (!gs101_bcl_device)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_OCP_WARN_CPUCL2, ACTIVE_HIGH);
}

static int ocp_cpu2_read_current(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_OCP_WARN_CPUCL2);
}

static const struct thermal_zone_of_device_ops gs101_ocp_cpu2_ops = {
	.get_temp = ocp_cpu2_read_current,
};

static void gs101_soft_cpu1_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_SOFT_OCP_WARN_CPUCL1].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_SOFT_OCP_WARN_CPUCL1, S2MPG10);
}

static irqreturn_t gs101_soft_cpu1_ocp_warn_irq_handler(int irq, void *data)
{
	struct gs101_bcl_dev *gs101_bcl_device = data;

	if (!gs101_bcl_device)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_SOFT_OCP_WARN_CPUCL1, ACTIVE_HIGH);
}

static int soft_ocp_cpu1_read_current(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_SOFT_OCP_WARN_CPUCL1);
}

static const struct thermal_zone_of_device_ops gs101_soft_ocp_cpu1_ops = {
	.get_temp = soft_ocp_cpu1_read_current,
};

static void gs101_soft_cpu2_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_SOFT_OCP_WARN_CPUCL2].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_SOFT_OCP_WARN_CPUCL2, S2MPG10);
}

static irqreturn_t gs101_soft_cpu2_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_SOFT_OCP_WARN_CPUCL2, ACTIVE_HIGH);
}

static int soft_ocp_cpu2_read_current(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_SOFT_OCP_WARN_CPUCL2);
}

static const struct thermal_zone_of_device_ops gs101_soft_ocp_cpu2_ops = {
	.get_temp = soft_ocp_cpu2_read_current,
};

static void gs101_tpu_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_OCP_WARN_TPU].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_OCP_WARN_TPU, S2MPG10);
}

static irqreturn_t gs101_tpu_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_OCP_WARN_TPU, ACTIVE_HIGH);
}

static int ocp_tpu_read_current(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_OCP_WARN_TPU);
}

static const struct thermal_zone_of_device_ops gs101_ocp_tpu_ops = {
	.get_temp = ocp_tpu_read_current,
};

static void gs101_soft_tpu_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_SOFT_OCP_WARN_TPU].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_SOFT_OCP_WARN_TPU, S2MPG10);
}

static irqreturn_t gs101_soft_tpu_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_SOFT_OCP_WARN_TPU, ACTIVE_HIGH);
}

static int soft_ocp_tpu_read_current(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_SOFT_OCP_WARN_TPU);
}

static const struct thermal_zone_of_device_ops gs101_soft_ocp_tpu_ops = {
	.get_temp = soft_ocp_tpu_read_current,
};

static void gs101_gpu_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_OCP_WARN_GPU].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_OCP_WARN_GPU, S2MPG11);
}

static irqreturn_t gs101_gpu_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG11, IRQ_OCP_WARN_GPU, ACTIVE_HIGH);
}

static int ocp_gpu_read_current(void *data, int *val)
{
	return s2mpg11_read_level(data, val, IRQ_OCP_WARN_GPU);
}

static const struct thermal_zone_of_device_ops gs101_ocp_gpu_ops = {
	.get_temp = ocp_gpu_read_current,
};

static void gs101_soft_gpu_warn_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_SOFT_OCP_WARN_GPU].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_SOFT_OCP_WARN_GPU, S2MPG11);
}

static irqreturn_t gs101_soft_gpu_ocp_warn_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG11, IRQ_SOFT_OCP_WARN_GPU, ACTIVE_HIGH);
}

static int soft_ocp_gpu_read_current(void *data, int *val)
{
	return s2mpg11_read_level(data, val, IRQ_SOFT_OCP_WARN_GPU);
}

static const struct thermal_zone_of_device_ops gs101_soft_ocp_gpu_ops = {
	.get_temp = soft_ocp_gpu_read_current,
};

static void gs101_pmic_120c_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_PMIC_120C].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_PMIC_120C, S2MPG10);
}

static irqreturn_t gs101_pmic_120c_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_PMIC_120C, ACTIVE_HIGH);
}

static int pmic_120c_read_temp(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_PMIC_120C);
}

static const struct thermal_zone_of_device_ops gs101_pmic_120c_ops = {
	.get_temp = pmic_120c_read_temp,
};

static void gs101_pmic_140c_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_PMIC_140C].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_PMIC_140C, S2MPG10);
}

static irqreturn_t gs101_pmic_140c_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_PMIC_140C, ACTIVE_HIGH);
}

static int pmic_140c_read_temp(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_PMIC_140C);
}

static const struct thermal_zone_of_device_ops gs101_pmic_140c_ops = {
	.get_temp = pmic_140c_read_temp,
};

static void gs101_pmic_overheat_work(struct work_struct *work)
{
	struct gs101_bcl_dev *gs101_bcl_device =
			container_of(work, struct gs101_bcl_dev,
		   s2mpg10_irq_work[IRQ_PMIC_OVERHEAT].work);

	irq_work(gs101_bcl_device, ACTIVE_HIGH, IRQ_PMIC_OVERHEAT, S2MPG10);
}

static irqreturn_t gs101_tsd_overheat_irq_handler(int irq, void *data)
{
	if (!data)
		return IRQ_HANDLED;

	return irq_handler(irq, data, S2MPG10, IRQ_PMIC_OVERHEAT, ACTIVE_HIGH);
}

static int tsd_overheat_read_temp(void *data, int *val)
{
	return s2mpg10_read_level(data, val, IRQ_PMIC_OVERHEAT);
}

static const struct thermal_zone_of_device_ops gs101_pmic_overheat_ops = {
	.get_temp = tsd_overheat_read_temp,
};

static int gs101_bcl_set_soc(void *data, int low, int high)
{
	struct gs101_bcl_dev *gs101_bcl_device = data;

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
	thermal_zone_device_update(gs101_bcl_device->soc_tzd,
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
	if (gs101_bcl_device->soc_tzd)
		thermal_zone_of_sensor_unregister(gs101_bcl_device->device,
						  gs101_bcl_device->soc_tzd);

	return 0;
}

static ssize_t clk_ratio_get(struct gs101_bcl_dev *bcl_dev, char __user *buf, size_t count,
			     loff_t *ppos, void __iomem *addr, const char *label,
			     bool is_subsystem_on)
{
	char buff[BUF_SIZE];
	int len = 0;
	unsigned int reg = 0;
	unsigned int step_count, step_size, numerator, denominator;

	if (!is_subsystem_on) {

		len = scnprintf(buff, BUF_SIZE, "%s is off.\n\n", label);
		return simple_read_from_buffer(buf, count, ppos, buff, len);
	}

	reg = __raw_readl(addr);
	step_count = (reg >> 20) & 0xFFF;
	step_size = (reg >> 14) & 0x3F;
	numerator = (reg >> 6) & 0x3F;
	denominator = reg & 0x3F;

	len = scnprintf(buff, BUF_SIZE, "%s:0x%x\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s:\n%s%s\n\n",
			label, reg, "Step count [31:20]", step_count,
			"Step size [19:14]", step_size, "Numerator [11:6]", numerator,
			"Denominator [5:0]", denominator,
			"To set", "echo 0x(value) > ", label);

	return simple_read_from_buffer(buf, count, ppos, buff, len);
}

static ssize_t clk_ratio_set(struct gs101_bcl_dev *bcl_dev, struct file *filp,
			     const char __user *user_buf,
			     size_t count, loff_t *ppos, void __iomem *addr)
{
	int ret;
	char buf[16];
	unsigned int value;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (ret < 0)
		return ret;
	buf[ret] = 0;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	/* Denominator cannot be zero */
	if ((value & 0x3F) == 0)
		return -EINVAL;

	mutex_lock(&bcl_dev->ratio_lock);
	__raw_writel(value, addr);
	mutex_unlock(&bcl_dev->ratio_lock);

	return 0;
}

static ssize_t tpu_clk_ratio_light_get(struct file *filp, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->tpu_mem + CPUCL12_CLKDIVSTEP_CON_LIGHT,
			     "tpu_clkdiv_ratio_light", is_subsystem_on(PMU_ALIVE_TPU_OUT));
}

static ssize_t tpu_clk_ratio_light_set(struct file *filp, const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	if (!is_subsystem_on(PMU_ALIVE_TPU_OUT))
		return -EIO;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->tpu_mem + CPUCL12_CLKDIVSTEP_CON_LIGHT);
}

static ssize_t gpu_clk_ratio_light_get(struct file *filp, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->gpu_mem + CPUCL12_CLKDIVSTEP_CON_LIGHT,
			     "gpu_clkdiv_ratio_light", is_subsystem_on(PMU_ALIVE_GPU_OUT));
}

static ssize_t gpu_clk_ratio_light_set(struct file *filp, const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	if (!is_subsystem_on(PMU_ALIVE_GPU_OUT))
		return -EIO;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->gpu_mem + CPUCL12_CLKDIVSTEP_CON_LIGHT);
}

static ssize_t cpucl1_clk_ratio_light_get(struct file *filp, char __user *buf,
					  size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->cpu1_mem + CPUCL12_CLKDIVSTEP_CON_LIGHT,
			     "cpucl1_clkdiv_ratio_light", is_subsystem_on(PMU_ALIVE_CPU1_OUT));
}

static ssize_t cpucl1_clk_ratio_light_set(struct file *filp, const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	if (!is_subsystem_on(PMU_ALIVE_CPU1_OUT))
		return -EIO;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->cpu1_mem + CPUCL12_CLKDIVSTEP_CON_LIGHT);
}

static ssize_t cpucl2_clk_ratio_light_get(struct file *filp, char __user *buf,
					  size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->cpu2_mem + CPUCL12_CLKDIVSTEP_CON_LIGHT,
			     "cpucl2_clkdiv_ratio_light", is_subsystem_on(PMU_ALIVE_CPU2_OUT));
}

static ssize_t cpucl2_clk_ratio_light_set(struct file *filp, const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	if (!is_subsystem_on(PMU_ALIVE_CPU2_OUT))
		return -EIO;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->cpu2_mem + CPUCL12_CLKDIVSTEP_CON_LIGHT);
}

static ssize_t tpu_clk_ratio_get(struct file *filp, char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->tpu_mem + CPUCL12_CLKDIVSTEP_CON_HEAVY,
			     "tpu_clkdiv_ratio_heavy", is_subsystem_on(PMU_ALIVE_TPU_OUT));
}

static ssize_t tpu_clk_ratio_set(struct file *filp, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	if (!is_subsystem_on(PMU_ALIVE_TPU_OUT))
		return -EIO;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->tpu_mem + CPUCL12_CLKDIVSTEP_CON_HEAVY);
}

static ssize_t gpu_clk_ratio_get(struct file *filp, char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->gpu_mem + CPUCL12_CLKDIVSTEP_CON_HEAVY,
			     "gpu_clkdiv_ratio_heavy", is_subsystem_on(PMU_ALIVE_GPU_OUT));
}

static ssize_t gpu_clk_ratio_set(struct file *filp, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	if (!is_subsystem_on(PMU_ALIVE_GPU_OUT))
		return -EIO;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->gpu_mem + CPUCL12_CLKDIVSTEP_CON_HEAVY);
}

static ssize_t cpucl0_clk_ratio_get(struct file *filp, char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->cpu0_mem + CPUCL0_CLKDIVSTEP_CON,
			     "cpucl0_clkdiv_ratio_heavy", true);
}

static ssize_t cpucl0_clk_ratio_set(struct file *filp, const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->cpu0_mem + CPUCL0_CLKDIVSTEP_CON);
}

static ssize_t cpucl1_clk_ratio_get(struct file *filp, char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->cpu1_mem + CPUCL12_CLKDIVSTEP_CON_HEAVY,
			     "cpucl1_clkdiv_ratio_heavy", is_subsystem_on(PMU_ALIVE_CPU1_OUT));
}

static ssize_t cpucl1_clk_ratio_set(struct file *filp, const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	if (!is_subsystem_on(PMU_ALIVE_CPU1_OUT))
		return -EIO;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->cpu1_mem + CPUCL12_CLKDIVSTEP_CON_HEAVY);
}

static ssize_t cpucl2_clk_ratio_get(struct file *filp, char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return clk_ratio_get(bcl_dev, buf, count, ppos,
			     bcl_dev->cpu2_mem + CPUCL12_CLKDIVSTEP_CON_HEAVY,
			     "cpucl2_clkdiv_ratio_heavy", is_subsystem_on(PMU_ALIVE_CPU2_OUT));
}

static ssize_t cpucl2_clk_ratio_set(struct file *filp, const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	if (!is_subsystem_on(PMU_ALIVE_CPU2_OUT))
		return -EIO;

	return clk_ratio_set(bcl_dev, filp, user_buf, count, ppos,
			     bcl_dev->cpu2_mem + CPUCL12_CLKDIVSTEP_CON_HEAVY);
}

BCL_DEBUG_ATTRIBUTE(tpu_clk_ratio_light_fops, tpu_clk_ratio_light_get, tpu_clk_ratio_light_set);
BCL_DEBUG_ATTRIBUTE(gpu_clk_ratio_light_fops, gpu_clk_ratio_light_get, gpu_clk_ratio_light_set);
BCL_DEBUG_ATTRIBUTE(cpucl1_clk_ratio_light_fops, cpucl1_clk_ratio_light_get,
		    cpucl1_clk_ratio_light_set);
BCL_DEBUG_ATTRIBUTE(cpucl2_clk_ratio_light_fops, cpucl2_clk_ratio_light_get,
		    cpucl2_clk_ratio_light_set);
BCL_DEBUG_ATTRIBUTE(tpu_clk_ratio_fops, tpu_clk_ratio_get, tpu_clk_ratio_set);
BCL_DEBUG_ATTRIBUTE(gpu_clk_ratio_fops, gpu_clk_ratio_get, gpu_clk_ratio_set);
BCL_DEBUG_ATTRIBUTE(cpucl0_clk_ratio_fops, cpucl0_clk_ratio_get, cpucl0_clk_ratio_set);
BCL_DEBUG_ATTRIBUTE(cpucl1_clk_ratio_fops, cpucl1_clk_ratio_get, cpucl1_clk_ratio_set);
BCL_DEBUG_ATTRIBUTE(cpucl2_clk_ratio_fops, cpucl2_clk_ratio_get, cpucl2_clk_ratio_set);

static ssize_t reset_stats(const char __user *user_buf, size_t count, loff_t *ppos,
			   void __iomem *addr, bool is_subsystem_on)
{
	int ret;
	char buf[1];

	if (!is_subsystem_on)
		return -EIO;

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	if (buf[0] == '0')
		__raw_writel(0x107d, addr);
	else
		__raw_writel(0x107f, addr);
	return 0;
}

static ssize_t trigger_stats_get(struct gs101_bcl_dev *bcl_dev, char __user *buf, size_t count,
				 loff_t *ppos, void __iomem *addr, const char *label,
				 bool is_subsystem_on)
{
	char buff[BUF_SIZE];
	int len = 0;
	unsigned int reg = 0;
	unsigned int stepup_run, trig_overflow, trig_cnt_value, current_numerator;

	if (!is_subsystem_on) {

		len = scnprintf(buff, BUF_SIZE, "%s is off.\n\n", label);
		return simple_read_from_buffer(buf, count, ppos, buff, len);
	}

	reg = __raw_readl(addr);
	stepup_run = (reg >> 31) & 0x1;
	trig_overflow = (reg >> 29) & 0x1;
	trig_cnt_value = (reg >> 16) & 0xFFF;
	current_numerator = (reg >> 8) & 0x3F;

	len = scnprintf(buff, BUF_SIZE, "%s:0x%x\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s:\n%s%s\n\n",
			label, reg, "Stepup run [31]", stepup_run,
			"Trig CNT OFL [29]", trig_overflow, "Trig CNT [27:16]",
			trig_cnt_value, "Current Num. [14:8]", current_numerator,
			"To disable/enable", "echo 0/1 > ", label);

	return simple_read_from_buffer(buf, count, ppos, buff, len);
}

static ssize_t get_cpucl0_stat(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return trigger_stats_get(bcl_dev, buf, count, ppos,
				 bcl_dev->cpu0_mem + CPUCL0_CLKDIVSTEP_STAT,
				 "cpucl0_clkdiv_stat", true);
}

static ssize_t reset_cpucl0_stat(struct file *filp, const char __user *user_buf,
			     size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return reset_stats(user_buf, count, ppos, bcl_dev->cpu0_mem + CLKDIVSTEP, true);
}

static ssize_t get_cpucl2_stat(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return trigger_stats_get(bcl_dev, buf, count, ppos,
				 bcl_dev->cpu2_mem + CPUCL12_CLKDIVSTEP_STAT,
				 "cpucl2_clkdiv_stat", is_subsystem_on(PMU_ALIVE_CPU2_OUT));
}

static ssize_t reset_cpucl2_stat(struct file *filp, const char __user *user_buf,
			     size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return reset_stats(user_buf, count, ppos, bcl_dev->cpu2_mem + CLKDIVSTEP,
			   is_subsystem_on(PMU_ALIVE_CPU2_OUT));
}

static ssize_t get_cpucl1_stat(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return trigger_stats_get(bcl_dev, buf, count, ppos,
				 bcl_dev->cpu1_mem + CPUCL12_CLKDIVSTEP_STAT,
				 "cpucl1_clkdiv_stat", is_subsystem_on(PMU_ALIVE_CPU1_OUT));
}

static ssize_t reset_cpucl1_stat(struct file *filp, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return reset_stats(user_buf, count, ppos, bcl_dev->cpu1_mem + CLKDIVSTEP,
			   is_subsystem_on(PMU_ALIVE_CPU1_OUT));
}

static ssize_t get_gpu_stat(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return trigger_stats_get(bcl_dev, buf, count, ppos,
				 bcl_dev->gpu_mem + G3D_CLKDIVSTEP_STAT,
				 "gpu_clkdiv_stat", is_subsystem_on(PMU_ALIVE_GPU_OUT));
}

static ssize_t reset_gpu_stat(struct file *filp, const char __user *user_buf,
			  size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return reset_stats(user_buf, count, ppos, bcl_dev->gpu_mem + CLKDIVSTEP,
			   is_subsystem_on(PMU_ALIVE_GPU_OUT));
}

static ssize_t get_tpu_stat(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return trigger_stats_get(bcl_dev, buf, count, ppos,
				 bcl_dev->tpu_mem + TPU_CLKDIVSTEP_STAT,
				 "tpu_clkdiv_stat", is_subsystem_on(PMU_ALIVE_TPU_OUT));
}

static ssize_t reset_tpu_stat(struct file *filp, const char __user *user_buf,
			  size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;

	return reset_stats(user_buf, count, ppos, bcl_dev->tpu_mem + CLKDIVSTEP,
			   is_subsystem_on(PMU_ALIVE_TPU_OUT));
}

BCL_DEBUG_ATTRIBUTE(cpucl0_clkdivstep_stat_fops, get_cpucl0_stat, reset_cpucl0_stat);
BCL_DEBUG_ATTRIBUTE(cpucl1_clkdivstep_stat_fops, get_cpucl1_stat, reset_cpucl1_stat);
BCL_DEBUG_ATTRIBUTE(cpucl2_clkdivstep_stat_fops, get_cpucl2_stat, reset_cpucl2_stat);
BCL_DEBUG_ATTRIBUTE(gpu_clkdivstep_stat_fops, get_gpu_stat, reset_gpu_stat);
BCL_DEBUG_ATTRIBUTE(tpu_clkdivstep_stat_fops, get_tpu_stat, reset_tpu_stat);

static int get_smpl_warn(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg10_triggered_irq[IRQ_SMPL_WARN];
	return 0;
}

static int get_cpu1_ocp(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg10_triggered_irq[IRQ_OCP_WARN_CPUCL1];
	return 0;
}

static int get_cpu2_ocp(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg10_triggered_irq[IRQ_OCP_WARN_CPUCL2];
	return 0;
}

static int get_tpu_ocp(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg10_triggered_irq[IRQ_OCP_WARN_TPU];
	return 0;
}

static int get_gpu_ocp(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg11_triggered_irq[IRQ_OCP_WARN_GPU];
	return 0;
}

static int get_soft_cpu1_ocp(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg10_triggered_irq[IRQ_SOFT_OCP_WARN_CPUCL1];
	return 0;
}

static int get_soft_cpu2_ocp(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg10_triggered_irq[IRQ_SOFT_OCP_WARN_CPUCL2];
	return 0;
}

static int get_soft_tpu_ocp(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg10_triggered_irq[IRQ_SOFT_OCP_WARN_TPU];
	return 0;
}

static int get_soft_gpu_ocp(void *data, u64 *val)
{
	struct gs101_bcl_dev *bcl_dev = data;

	*val = bcl_dev->s2mpg11_triggered_irq[IRQ_SOFT_OCP_WARN_GPU];
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(smpl_triggered_fops, get_smpl_warn, NULL, "%d\n");
DEFINE_SIMPLE_ATTRIBUTE(soft_cpucl1_ocp_triggered_fops, get_soft_cpu1_ocp, NULL, "%d\n");
DEFINE_SIMPLE_ATTRIBUTE(soft_cpucl2_ocp_triggered_fops, get_soft_cpu2_ocp, NULL, "%d\n");
DEFINE_SIMPLE_ATTRIBUTE(soft_tpu_ocp_triggered_fops, get_soft_tpu_ocp, NULL, "%d\n");
DEFINE_SIMPLE_ATTRIBUTE(soft_gpu_ocp_triggered_fops, get_soft_gpu_ocp, NULL, "%d\n");
DEFINE_SIMPLE_ATTRIBUTE(cpucl1_ocp_triggered_fops, get_cpu1_ocp, NULL, "%d\n");
DEFINE_SIMPLE_ATTRIBUTE(cpucl2_ocp_triggered_fops, get_cpu2_ocp, NULL, "%d\n");
DEFINE_SIMPLE_ATTRIBUTE(tpu_ocp_triggered_fops, get_tpu_ocp, NULL, "%d\n");
DEFINE_SIMPLE_ATTRIBUTE(gpu_ocp_triggered_fops, get_gpu_ocp, NULL, "%d\n");

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
	else {
		bcl_dev->s2mpg10_lvl[IRQ_SMPL_WARN] = SMPL_BATTERY_VOLTAGE - val -
				THERMAL_HYST_LEVEL;
		ret = bcl_dev->s2mpg10_tz_irq[IRQ_SMPL_WARN]->ops->set_trip_temp(
				bcl_dev->s2mpg10_tz_irq[IRQ_SMPL_WARN], 0,
				SMPL_BATTERY_VOLTAGE - val);
		if (ret)
			pr_err("Fail to set smpl_warn trip temp\n");

	}

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
		       u16 llimit, u16 ulimit, u8 step, u8 id)
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
		mutex_lock(&bcl_dev->s2mpg10_irq_lock[id]);
		if (s2mpg10_read_reg(bcl_dev->s2mpg10_i2c, addr, &value)) {
			pr_err("S2MPG10 read 0x%x failed.", addr);
			mutex_unlock(&bcl_dev->s2mpg10_irq_lock[id]);
			return -1;
		}
		value &= ~(OCP_WARN_MASK) << S2MPG10_OCP_WARN_LVL_SHIFT;
		value |= ((ulimit - val) / step) << S2MPG10_OCP_WARN_LVL_SHIFT;
		ret = s2mpg10_write_reg(bcl_dev->s2mpg10_i2c, addr, value);
		if (!ret) {
			bcl_dev->s2mpg10_lvl[id] = val - THERMAL_HYST_LEVEL;
			ret = bcl_dev->s2mpg10_tz_irq[id]->ops->set_trip_temp(
					bcl_dev->s2mpg10_tz_irq[id], 0,
					val);
			if (ret)
				pr_err("Fail to set ocp_warn trip temp\n");
		}
		mutex_unlock(&bcl_dev->s2mpg10_irq_lock[id]);
	} else {
		mutex_lock(&bcl_dev->s2mpg11_irq_lock[id]);
		if (s2mpg11_read_reg(bcl_dev->s2mpg11_i2c, addr, &value)) {
			pr_err("S2MPG11 read 0x%x failed.", addr);
			mutex_unlock(&bcl_dev->s2mpg11_irq_lock[id]);
			return -1;
		}
		value &= ~(OCP_WARN_MASK) << S2MPG10_OCP_WARN_LVL_SHIFT;
		value |= ((ulimit - val) / step) << S2MPG11_OCP_WARN_LVL_SHIFT;
		ret = s2mpg11_write_reg(bcl_dev->s2mpg11_i2c, addr, value);
		if (!ret) {
			bcl_dev->s2mpg11_lvl[id] = val - THERMAL_HYST_LEVEL;
			ret = bcl_dev->s2mpg11_tz_irq[id]->ops->set_trip_temp(
					bcl_dev->s2mpg11_tz_irq[id], 0,
					val);
			if (ret)
				pr_err("Fail to set ocp_warn trip temp\n");
		}
		mutex_unlock(&bcl_dev->s2mpg11_irq_lock[id]);
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
			   B3M_STEP, IRQ_SOFT_OCP_WARN_CPUCL1);
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
			   B2M_STEP, IRQ_SOFT_OCP_WARN_CPUCL2);
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
			   B3M_STEP, IRQ_OCP_WARN_CPUCL1);
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
			   B2M_STEP, IRQ_OCP_WARN_CPUCL2);
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
			   B10M_STEP, IRQ_OCP_WARN_TPU);
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
			   B10M_STEP, IRQ_SOFT_OCP_WARN_TPU);
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
			   B2S_STEP, IRQ_OCP_WARN_GPU);
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
			   B2S_STEP, IRQ_SOFT_OCP_WARN_GPU);
}

DEFINE_SIMPLE_ATTRIBUTE(soft_gpu_lvl_fops, get_soft_gpu_lvl,
			set_soft_gpu_lvl, "%d\n");

static void gs101_set_ppm_throttling(struct gs101_bcl_dev *gs101_bcl_device,
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
	mask = 0x01 << 8;
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

static ssize_t gs101_mpmm_settings_set(struct file *filp, const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;
	void __iomem *addr;
	int value;
	char buf[16];
	int ret;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (ret < 0)
		return ret;
	buf[ret] = 0;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_MPMM;
	__raw_writel(value, addr);
	mutex_unlock(&sysreg_lock);

	return 0;
}

static const char *mpmm_gear_parse(unsigned int state)
{
	switch (state) {
	case 0x0:
		return "GEAR 0";
	case 0x1:
		return "GEAR 1";
	case 0x2:
		return "GEAR 2";
	case 0x3:
	default:
		return "DISABLED";
	}
}

static ssize_t gs101_mpmm_settings_get(struct file *filp, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;
	unsigned int reg = 0;
	unsigned int state_big7, state_big6;
	int len = 0;
	void __iomem *addr;
	char buff[BUF_SIZE];

	if (!bcl_dev->sysreg_cpucl0) {
		len = scnprintf(buff, BUF_SIZE, "sysreg_cpucl0 memory is not accessible.\n\n");
		return simple_read_from_buffer(buf, count, ppos, buff, len);
	}

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_MPMM;
	reg = __raw_readl(addr);
	mutex_unlock(&sysreg_lock);
	state_big7 = (reg >> 2) & 0x3;
	state_big6  = reg & 0x3;
	len = scnprintf(buff, BUF_SIZE,
			"0x%x\n%s: 0x%x,%s\n%s: 0x%x,%s\n%s:\n%s\n\n",
			reg, "MPMMSTATE_BIG7 [3:2]", state_big7, mpmm_gear_parse(state_big7),
			"MPMMSTATE_BIG6 [1:0]", state_big6, mpmm_gear_parse(state_big6),
			"To set", "echo 0x(value) > mpmm_settings");

	return simple_read_from_buffer(buf, count, ppos, buff, len);
}

static void
gs101_set_mpmm_throttling(struct gs101_bcl_dev *gs101_bcl_device,
			  enum sys_throttling_switch throttle_switch)
{
	unsigned int reg;
	void __iomem *addr;
	unsigned int settings;
	unsigned int sys_throttling_settings[SYS_THROTTLING_MAX] = {0x1F, 0x10, 0x10, 0x15, 0x1A};

	if (!gs101_bcl_device->sysreg_cpucl0) {
		pr_err("sysreg_cpucl0 ioremap not mapped\n");
		return;
	}
	mutex_lock(&sysreg_lock);
	addr = gs101_bcl_device->sysreg_cpucl0 + CLUSTER0_MPMM;
	reg = __raw_readl(addr);

	if ((throttle_switch < 0) || (throttle_switch > SYS_THROTTLING_GEAR2))
		settings = 0xF;
	else
		settings = sys_throttling_settings[throttle_switch];

	reg &= ~0x1F;
	reg |= settings;
	__raw_writel(reg, addr);
	mutex_unlock(&sysreg_lock);
}

static ssize_t gs101_ppm_settings_set(struct file *filp, const char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;
	void __iomem *addr;
	int value;
	char buf[16];
	int ret;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (ret < 0)
		return ret;
	buf[ret] = 0;

	ret = sscanf(buf, "0x%x", &value);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM;
	__raw_writel(value, addr);
	mutex_unlock(&sysreg_lock);

	return 0;
}

static ssize_t gs101_ppm_settings_get(struct file *filp, char __user *buf,
				      size_t count, loff_t *ppos)
{
	struct gs101_bcl_dev *bcl_dev = filp->private_data;
	unsigned int reg = 0;
	unsigned int ppmctl7, ppmctl6, ocp_en_big;
	int len = 0;
	void __iomem *addr;
	char buff[BUF_SIZE];

	if (!bcl_dev->sysreg_cpucl0) {
		len = scnprintf(buff, BUF_SIZE, "sysreg_cpucl0 memory is not accessible.\n\n");
		return simple_read_from_buffer(buf, count, ppos, buff, len);
	}

	mutex_lock(&sysreg_lock);
	addr = bcl_dev->sysreg_cpucl0 + CLUSTER0_PPM;
	reg = __raw_readl(addr);
	mutex_unlock(&sysreg_lock);
	ocp_en_big = test_bit(8, (unsigned long *)&reg);
	ppmctl7 = (reg >> 4) & 0xF;
	ppmctl6 = reg & 0xF;
	len = scnprintf(buff, BUF_SIZE,
			"0x%x\n%s: %d\n%s: 0x%x\n%s: 0x%x\n%s:\n%s\n\n",
			reg, "PPMOCP_EN_BIG [8]", ocp_en_big,
			"PPMCTL7 BIG1 [7:4]", ppmctl7, "PPMCTL6 BIG0 [3:0]", ppmctl6,
			"To set", "echo 0x(value) > ppm_settings");

	return simple_read_from_buffer(buf, count, ppos, buff, len);
}
BCL_DEBUG_ATTRIBUTE(mpmm_fops, gs101_mpmm_settings_get, gs101_mpmm_settings_set);
BCL_DEBUG_ATTRIBUTE(ppm_fops, gs101_ppm_settings_get, gs101_ppm_settings_set);

static int gs101_bcl_register_pmic_irq(struct gs101_bcl_dev *gs101_bcl_device,
				  int id, int sensor_id, irq_handler_t thread_fn,
				  struct device *dev,
				  const struct thermal_zone_of_device_ops *ops,
				  const char *devname, u8 pmic, u32 intr_flag)
{
	int ret = 0;

	ret = devm_request_threaded_irq(gs101_bcl_device->device,
					gs101_bcl_device->s2mpg10_irq[id],
					NULL, thread_fn,
					intr_flag | IRQF_ONESHOT,
					devname, gs101_bcl_device);
	if (ret < 0) {
		pr_err("Failed to request IRQ: %d: %d\n",
			gs101_bcl_device->s2mpg10_irq[id], ret);
		return ret;
	}
	if (ops) {
		gs101_bcl_device->s2mpg10_tz_irq[id] =
				thermal_zone_of_sensor_register(dev, sensor_id,
					gs101_bcl_device,
								ops);
		if (IS_ERR(gs101_bcl_device->s2mpg10_tz_irq[id])) {
			pr_err("PMIC TZ register failed. %d, err:%ld\n", id,
					PTR_ERR(gs101_bcl_device->s2mpg10_tz_irq[id]));
		} else {
			thermal_zone_device_enable(gs101_bcl_device->s2mpg10_tz_irq[id]);
			thermal_zone_device_update(gs101_bcl_device->s2mpg10_tz_irq[id],
			   THERMAL_DEVICE_UP);
		}
	}
	return ret;
}

static int gs101_bcl_register_irq(struct gs101_bcl_dev *gs101_bcl_device,
				  int id, irq_handler_t thread_fn,
				  struct device *dev,
				  const struct thermal_zone_of_device_ops *ops,
				  const char *devname, u8 pmic, u32 intr_flag)
{
	int ret = 0;

	if (pmic == S2MPG10) {
		ret = devm_request_threaded_irq(gs101_bcl_device->device,
						gs101_bcl_device->s2mpg10_irq[id],
						NULL, thread_fn,
						intr_flag | IRQF_ONESHOT,
						devname, gs101_bcl_device);
		if (ret < 0) {
			pr_err("Failed to request IRQ: %d: %d\n",
				gs101_bcl_device->s2mpg10_irq[id], ret);
			return ret;
		}
		if (ops) {
			gs101_bcl_device->s2mpg10_tz_irq[id] =
					thermal_zone_of_sensor_register(dev, id,
									gs101_bcl_device,
									ops);
			if (IS_ERR(gs101_bcl_device->s2mpg10_tz_irq[id])) {
				pr_err("TZ register failed. %d, err:%ld\n", id,
						PTR_ERR(gs101_bcl_device->s2mpg10_tz_irq[id]));
			} else {
				thermal_zone_device_enable(gs101_bcl_device->s2mpg10_tz_irq[id]);
				thermal_zone_device_update(gs101_bcl_device->s2mpg10_tz_irq[id],
				   THERMAL_DEVICE_UP);
			}
		}
	} else {
		ret = devm_request_threaded_irq(gs101_bcl_device->device,
						gs101_bcl_device->s2mpg11_irq[id],
						NULL, thread_fn,
						intr_flag | IRQF_ONESHOT,
						devname, gs101_bcl_device);
		if (ret < 0) {
			pr_err("Failed to request IRQ: %d: %d\n",
				gs101_bcl_device->s2mpg11_irq[id], ret);
			return ret;
		}
		if (ops) {
			gs101_bcl_device->s2mpg11_tz_irq[id] =
					thermal_zone_of_sensor_register(dev, id,
									gs101_bcl_device,
									ops);
			if (IS_ERR(gs101_bcl_device->s2mpg11_tz_irq[id])) {
				pr_err("TZ register failed. %d, err:%ld\n", id,
						PTR_ERR(gs101_bcl_device->s2mpg11_tz_irq[id]));
			} else {
				thermal_zone_device_enable(gs101_bcl_device->s2mpg11_tz_irq[id]);
				thermal_zone_device_update(gs101_bcl_device->s2mpg11_tz_irq[id],
				   THERMAL_DEVICE_UP);
			}
		}
	}
	return ret;
}

static void gs101_bcl_mfd_init(struct work_struct *work)
{
	u8 val = 0;
	bool bypass_smpl_warn = false;
	int ret;
	struct gs101_bcl_dev *gs101_bcl_device =
	    container_of(work, struct gs101_bcl_dev, mfd_init.work);
	struct s2mpg10_dev *s2mpg10;
	struct s2mpg11_dev *s2mpg11;
	struct s2mpg10_platform_data *pdata_s2mpg10;
	struct s2mpg11_platform_data *pdata_s2mpg11;

	if (strcmp(dev_name(gs101_bcl_device->device), google_gs101_id_table[0].name) == 0) {
		s2mpg10 = gs101_bcl_device->iodev;
		pdata_s2mpg10 = dev_get_platdata(s2mpg10->dev);
		/* request smpl_warn interrupt */
		if (!gpio_is_valid(pdata_s2mpg10->smpl_warn_pin)) {
			pr_err("smpl_warn GPIO NOT VALID\n");
			devm_free_irq(gs101_bcl_device->device,
				      gs101_bcl_device->s2mpg10_irq[IRQ_SMPL_WARN],
				      gs101_bcl_device);
			bypass_smpl_warn = true;
		}
		thermal_zone_device_update(gs101_bcl_device->soc_tzd, THERMAL_DEVICE_UP);
		schedule_delayed_work(&gs101_bcl_device->soc_eval_work, 0);
		gs101_bcl_device->s2mpg10_i2c = s2mpg10->pmic;
		gs101_bcl_device->s2mpg11_i2c = 0x0;
		gs101_bcl_device->s2mpg10_irq[IRQ_SMPL_WARN] =
				gpio_to_irq(pdata_s2mpg10->smpl_warn_pin);
		irq_set_status_flags(gs101_bcl_device->s2mpg10_irq[IRQ_SMPL_WARN],
				     IRQ_DISABLE_UNLAZY);
		gs101_bcl_device->s2mpg10_pin[IRQ_SMPL_WARN] = pdata_s2mpg10->smpl_warn_pin;
		gs101_bcl_device->s2mpg10_lvl[IRQ_SMPL_WARN] = SMPL_BATTERY_VOLTAGE -
				(pdata_s2mpg10->smpl_warn_lvl * SMPL_STEP +
				 SMPL_LOWER_LIMIT);
		gs101_bcl_device->s2mpg10_lvl[IRQ_OCP_WARN_CPUCL1] = B3M_UPPER_LIMIT -
				THERMAL_HYST_LEVEL - (pdata_s2mpg10->b3_ocp_warn_lvl * B3M_STEP);
		gs101_bcl_device->s2mpg10_lvl[IRQ_SOFT_OCP_WARN_CPUCL1] = B3M_UPPER_LIMIT -
				THERMAL_HYST_LEVEL -
				(pdata_s2mpg10->b3_soft_ocp_warn_lvl * B3M_STEP);
		gs101_bcl_device->s2mpg10_lvl[IRQ_OCP_WARN_CPUCL2] = B2M_UPPER_LIMIT -
				THERMAL_HYST_LEVEL - (pdata_s2mpg10->b2_ocp_warn_lvl * B2M_STEP);
		gs101_bcl_device->s2mpg10_lvl[IRQ_SOFT_OCP_WARN_CPUCL2] = B2M_UPPER_LIMIT -
				THERMAL_HYST_LEVEL -
				(pdata_s2mpg10->b2_soft_ocp_warn_lvl * B2M_STEP);
		gs101_bcl_device->s2mpg10_lvl[IRQ_OCP_WARN_TPU] = B10M_UPPER_LIMIT -
				THERMAL_HYST_LEVEL -
				(pdata_s2mpg10->b10_ocp_warn_lvl * B10M_STEP);
		gs101_bcl_device->s2mpg10_lvl[IRQ_SOFT_OCP_WARN_TPU] = B10M_UPPER_LIMIT -
				THERMAL_HYST_LEVEL -
				(pdata_s2mpg10->b10_soft_ocp_warn_lvl * B10M_STEP);
		gs101_bcl_device->s2mpg10_lvl[IRQ_PMIC_120C] = PMIC_120C_UPPER_LIMIT -
				THERMAL_HYST_LEVEL;
		gs101_bcl_device->s2mpg10_lvl[IRQ_PMIC_140C] = PMIC_140C_UPPER_LIMIT -
				THERMAL_HYST_LEVEL;
		gs101_bcl_device->s2mpg10_lvl[IRQ_PMIC_OVERHEAT] = PMIC_OVERHEAT_UPPER_LIMIT -
				THERMAL_HYST_LEVEL;
		gs101_bcl_device->s2mpg10_pin[IRQ_OCP_WARN_CPUCL1] =
				pdata_s2mpg10->b3_ocp_warn_pin;
		gs101_bcl_device->s2mpg10_pin[IRQ_OCP_WARN_CPUCL2] =
				pdata_s2mpg10->b2_ocp_warn_pin;
		gs101_bcl_device->s2mpg10_pin[IRQ_SOFT_OCP_WARN_CPUCL1] =
				pdata_s2mpg10->b3_soft_ocp_warn_pin;
		gs101_bcl_device->s2mpg10_pin[IRQ_SOFT_OCP_WARN_CPUCL2] =
				pdata_s2mpg10->b2_soft_ocp_warn_pin;
		gs101_bcl_device->s2mpg10_pin[IRQ_OCP_WARN_TPU] =
				pdata_s2mpg10->b10_ocp_warn_pin;
		gs101_bcl_device->s2mpg10_pin[IRQ_SOFT_OCP_WARN_TPU] =
				pdata_s2mpg10->b10_soft_ocp_warn_pin;
		gs101_bcl_device->s2mpg10_irq[IRQ_OCP_WARN_CPUCL1] =
				gpio_to_irq(pdata_s2mpg10->b3_ocp_warn_pin);
		gs101_bcl_device->s2mpg10_irq[IRQ_OCP_WARN_CPUCL2] =
				gpio_to_irq(pdata_s2mpg10->b2_ocp_warn_pin);
		gs101_bcl_device->s2mpg10_irq[IRQ_SOFT_OCP_WARN_CPUCL1] =
				gpio_to_irq(pdata_s2mpg10->b3_soft_ocp_warn_pin);
		gs101_bcl_device->s2mpg10_irq[IRQ_SOFT_OCP_WARN_CPUCL2] =
				gpio_to_irq(pdata_s2mpg10->b2_soft_ocp_warn_pin);
		gs101_bcl_device->s2mpg10_irq[IRQ_OCP_WARN_TPU] =
				gpio_to_irq(pdata_s2mpg10->b10_ocp_warn_pin);
		gs101_bcl_device->s2mpg10_irq[IRQ_SOFT_OCP_WARN_TPU] =
				gpio_to_irq(pdata_s2mpg10->b10_soft_ocp_warn_pin);
		gs101_bcl_device->s2mpg10_irq[IRQ_PMIC_120C] =
				pdata_s2mpg10->irq_base + S2MPG10_IRQ_120C_INT3;
		gs101_bcl_device->s2mpg10_irq[IRQ_PMIC_140C] =
				pdata_s2mpg10->irq_base + S2MPG10_IRQ_140C_INT3;
		gs101_bcl_device->s2mpg10_irq[IRQ_PMIC_OVERHEAT] =
				pdata_s2mpg10->irq_base + S2MPG10_IRQ_TSD_INT3;
		if (s2mpg10_read_reg(gs101_bcl_device->s2mpg10_i2c,
				     S2MPG10_COMMON_CHIPID, &val)) {
			pr_err("S2MPG10 not loaded.\n");
			return;
		}
		if (!bypass_smpl_warn) {
			ret = gs101_bcl_register_irq(gs101_bcl_device,
						     IRQ_SMPL_WARN,
						     gs101_smpl_warn_irq_handler,
						     s2mpg10->dev,
						     &gs101_smpl_warn_ops,
						     "SMPL_WARN_IRQ",
						     S2MPG10, IRQF_TRIGGER_LOW);
			if (ret < 0) {
				pr_err("bcl_register fail:%d\n", IRQ_SMPL_WARN);
				return;
			}
		}
		ret = gs101_bcl_register_irq(gs101_bcl_device,
					     IRQ_OCP_WARN_CPUCL1,
					     gs101_cpu1_ocp_warn_irq_handler,
					     s2mpg10->dev,
					     &gs101_ocp_cpu1_ops, "CPU1_OCP_IRQ",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_register fail:%d\n", IRQ_OCP_WARN_CPUCL1);
			return;
		}
		ret = gs101_bcl_register_irq(gs101_bcl_device,
					     IRQ_OCP_WARN_CPUCL2,
					     gs101_cpu2_ocp_warn_irq_handler,
					     s2mpg10->dev,
					     &gs101_ocp_cpu2_ops, "CPU2_OCP_IRQ",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_register fail:%d\n", IRQ_OCP_WARN_CPUCL2);
			return;
		}
		ret = gs101_bcl_register_irq(gs101_bcl_device,
					     IRQ_SOFT_OCP_WARN_CPUCL1,
					     gs101_soft_cpu1_ocp_warn_irq_handler,
					     s2mpg10->dev,
					     &gs101_soft_ocp_cpu1_ops, "SOFT_CPU1_OCP_IRQ",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_register fail:%d\n", IRQ_SOFT_OCP_WARN_CPUCL1);
			return;
		}
		ret = gs101_bcl_register_irq(gs101_bcl_device,
					     IRQ_SOFT_OCP_WARN_CPUCL2,
					     gs101_soft_cpu2_ocp_warn_irq_handler,
					     s2mpg10->dev,
					     &gs101_soft_ocp_cpu2_ops, "SOFT_CPU2_OCP_IRQ",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_register fail:%d\n", IRQ_SOFT_OCP_WARN_CPUCL2);
			return;
		}
		ret = gs101_bcl_register_irq(gs101_bcl_device,
					     IRQ_OCP_WARN_TPU,
					     gs101_tpu_ocp_warn_irq_handler,
					     s2mpg10->dev,
					     &gs101_ocp_tpu_ops, "TPU_OCP_IRQ",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_register fail:%d\n", IRQ_OCP_WARN_TPU);
			return;
		}
		ret = gs101_bcl_register_irq(gs101_bcl_device,
					     IRQ_SOFT_OCP_WARN_TPU,
					     gs101_soft_tpu_ocp_warn_irq_handler,
					     s2mpg10->dev,
					     &gs101_soft_ocp_tpu_ops, "SOFT_TPU_OCP_IRQ",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_register fail:%d\n", IRQ_SOFT_OCP_WARN_TPU);
			return;
		}
		ret = gs101_bcl_register_pmic_irq(gs101_bcl_device,
					     IRQ_PMIC_120C, PMIC_120C,
					     gs101_pmic_120c_irq_handler,
					     gs101_bcl_device->device,
					     &gs101_pmic_120c_ops, "PMIC_120C",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_pmic_register fail:%d\n", IRQ_PMIC_120C);
			return;
		}
		ret = gs101_bcl_register_pmic_irq(gs101_bcl_device,
					     IRQ_PMIC_140C, PMIC_140C,
					     gs101_pmic_140c_irq_handler,
					     gs101_bcl_device->device,
					     &gs101_pmic_140c_ops, "PMIC_140C",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_pmic_register fail:%d\n", IRQ_PMIC_140C);
			return;
		}
		ret = gs101_bcl_register_pmic_irq(gs101_bcl_device,
					     IRQ_PMIC_OVERHEAT, PMIC_OVERHEAT,
					     gs101_tsd_overheat_irq_handler,
					     gs101_bcl_device->device,
					     &gs101_pmic_overheat_ops, "THERMAL_OVERHEAT",
					     S2MPG10, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_pmic_register fail:%d\n", IRQ_PMIC_OVERHEAT);
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
				    gs101_bcl_device, &soft_tpu_lvl_fops);

		debugfs_create_file("ocp_cpu1_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpu1_lvl_fops);

		debugfs_create_file("ocp_cpu2_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpu2_lvl_fops);

		debugfs_create_file("ocp_tpu_lvl", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &tpu_lvl_fops);

	}
	if (strcmp(dev_name(gs101_bcl_device->device), google_gs101_id_table[1].name) == 0) {
		s2mpg11 = gs101_bcl_device->iodev;
		pdata_s2mpg11 = dev_get_platdata(s2mpg11->dev);
		gs101_bcl_device->s2mpg10_i2c = 0;
		gs101_bcl_device->s2mpg11_i2c = s2mpg11->pmic;
		gs101_bcl_device->s2mpg11_lvl[IRQ_OCP_WARN_GPU] = B2S_UPPER_LIMIT -
				THERMAL_HYST_LEVEL -
				(pdata_s2mpg11->b2_ocp_warn_lvl * B2S_STEP);
		gs101_bcl_device->s2mpg11_lvl[IRQ_SOFT_OCP_WARN_GPU] = B2S_UPPER_LIMIT -
				THERMAL_HYST_LEVEL -
				(pdata_s2mpg11->b2_soft_ocp_warn_lvl * B2S_STEP);
		gs101_bcl_device->s2mpg11_pin[IRQ_OCP_WARN_GPU] =
				pdata_s2mpg11->b2_ocp_warn_pin;
		gs101_bcl_device->s2mpg11_pin[IRQ_SOFT_OCP_WARN_GPU] =
				pdata_s2mpg11->b2_soft_ocp_warn_pin;
		gs101_bcl_device->s2mpg11_irq[IRQ_OCP_WARN_GPU] =
				gpio_to_irq(pdata_s2mpg11->b2_ocp_warn_pin);
		gs101_bcl_device->s2mpg11_irq[IRQ_SOFT_OCP_WARN_GPU] =
				gpio_to_irq(pdata_s2mpg11->b2_soft_ocp_warn_pin);
		if (s2mpg11_read_reg(gs101_bcl_device->s2mpg11_i2c,
				     S2MPG11_COMMON_CHIPID, &val)) {
			pr_err("S2MPG11 not loaded.\n");
			return;
		}
		ret = gs101_bcl_register_irq(gs101_bcl_device,
					     IRQ_OCP_WARN_GPU,
					     gs101_gpu_ocp_warn_irq_handler,
					     s2mpg11->dev,
					     &gs101_ocp_gpu_ops, "GPU_OCP_IRQ",
					     S2MPG11, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_register fail:%d\n", IRQ_OCP_WARN_GPU);
			return;
		}
		ret = gs101_bcl_register_irq(gs101_bcl_device,
					     IRQ_SOFT_OCP_WARN_GPU,
					     gs101_soft_gpu_ocp_warn_irq_handler,
					     s2mpg11->dev,
					     &gs101_soft_ocp_gpu_ops, "SOFT_GPU_OCP_IRQ",
					     S2MPG11, IRQF_TRIGGER_HIGH);
		if (ret < 0) {
			pr_err("bcl_register fail:%d\n", IRQ_SOFT_OCP_WARN_GPU);
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
	int ret = 0, i;
	struct gs101_bcl_dev *gs101_bcl_device;
	struct dentry *root;

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
		debugfs_create_file("gpu_clkdiv_stat", 0644,
				    gs101_bcl_device->debug_entry, gs101_bcl_device,
				    &gpu_clkdivstep_stat_fops);
		debugfs_create_file("tpu_clkdiv_stat", 0644,
				    gs101_bcl_device->debug_entry, gs101_bcl_device,
				    &tpu_clkdivstep_stat_fops);
		debugfs_create_file("mpmm_settings", 0644,
				    gs101_bcl_device->debug_entry, gs101_bcl_device, &mpmm_fops);
		debugfs_create_file("ppm_settings", 0644, gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &ppm_fops);
		debugfs_create_file("cpucl2_clkdiv_ratio_heavy", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpucl2_clk_ratio_fops);
		debugfs_create_file("cpucl1_clkdiv_ratio_heavy", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpucl1_clk_ratio_fops);
		debugfs_create_file("cpucl0_clkdiv_ratio_heavy", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpucl0_clk_ratio_fops);
		debugfs_create_file("gpu_clkdiv_ratio_heavy", 0644, gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &gpu_clk_ratio_fops);
		debugfs_create_file("tpu_clkdiv_ratio_heavy", 0644, gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &tpu_clk_ratio_fops);
		debugfs_create_file("cpucl2_clkdiv_ratio_light", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpucl2_clk_ratio_light_fops);
		debugfs_create_file("cpucl1_clkdiv_ratio_light", 0644,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpucl1_clk_ratio_light_fops);
		debugfs_create_file("gpu_clkdiv_ratio_light", 0644, gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &gpu_clk_ratio_light_fops);
		debugfs_create_file("tpu_clkdiv_ratio_light", 0644, gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &tpu_clk_ratio_light_fops);
		debugfs_create_file("smpl_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &smpl_triggered_fops);
		debugfs_create_file("cpu1_ocp_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpucl1_ocp_triggered_fops);
		debugfs_create_file("cpu2_ocp_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &cpucl2_ocp_triggered_fops);
		debugfs_create_file("tpu_ocp_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &tpu_ocp_triggered_fops);
		debugfs_create_file("gpu_ocp_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &gpu_ocp_triggered_fops);
		debugfs_create_file("soft_cpu1_ocp_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &soft_cpucl1_ocp_triggered_fops);
		debugfs_create_file("soft_cpu2_ocp_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &soft_cpucl2_ocp_triggered_fops);
		debugfs_create_file("soft_tpu_ocp_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &soft_tpu_ocp_triggered_fops);
		debugfs_create_file("soft_gpu_ocp_triggered", 0600,
				    gs101_bcl_device->debug_entry,
				    gs101_bcl_device, &soft_gpu_ocp_triggered_fops);

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
	gs101_bcl_device->gpu_mem = ioremap(G3D_BASE, SZ_8K);
	if (!gs101_bcl_device->gpu_mem) {
		pr_err("gpu_mem ioremap failed\n");
		ret = -EIO;
		goto bcl_soc_probe_exit;
	}
	gs101_bcl_device->tpu_mem = ioremap(TPU_BASE, SZ_8K);
	if (!gs101_bcl_device->tpu_mem) {
		pr_err("tpu_mem ioremap failed\n");
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
				 SYS_THROTTLING_DISABLED);
	gs101_set_mpmm_throttling(gs101_bcl_device,
				  SYS_THROTTLING_DISABLED);
	mutex_init(&gs101_bcl_device->state_trans_lock);
	mutex_init(&gs101_bcl_device->ratio_lock);
	gs101_bcl_device->soc_ops.get_temp = gs101_bcl_read_soc;
	gs101_bcl_device->soc_ops.set_trips = gs101_bcl_set_soc;
	for (i = 0; i < IRQ_SOURCE_S2MPG10_MAX; i++) {
		gs101_bcl_device->s2mpg10_counter[i] = 0;
		gs101_bcl_device->s2mpg10_triggered_irq[i] = 0;
		mutex_init(&gs101_bcl_device->s2mpg10_irq_lock[i]);
	}
	for (i = 0; i < IRQ_SOURCE_S2MPG11_MAX; i++) {
		gs101_bcl_device->s2mpg11_counter[i] = 0;
		gs101_bcl_device->s2mpg11_triggered_irq[i] = 0;
		mutex_init(&gs101_bcl_device->s2mpg11_irq_lock[i]);
	}
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
		gs101_bcl_device->soc_tzd = thermal_zone_of_sensor_register(
			gs101_bcl_device->device, PMIC_SOC, gs101_bcl_device,
			&gs101_bcl_device->soc_ops);
		if (IS_ERR(gs101_bcl_device->soc_tzd)) {
			pr_err("soc TZ register failed. err:%ld\n",
			       PTR_ERR(gs101_bcl_device->soc_tzd));
			ret = PTR_ERR(gs101_bcl_device->soc_tzd);
			gs101_bcl_device->soc_tzd = NULL;
			ret = -EPROBE_DEFER;
			goto bcl_soc_probe_exit;
		}
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_SMPL_WARN],
				  gs101_smpl_warn_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_OCP_WARN_CPUCL1],
				  gs101_cpu1_warn_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_SOFT_OCP_WARN_CPUCL1],
				  gs101_soft_cpu1_warn_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_OCP_WARN_CPUCL2],
				  gs101_cpu2_warn_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_SOFT_OCP_WARN_CPUCL2],
				  gs101_soft_cpu2_warn_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_OCP_WARN_TPU],
				  gs101_tpu_warn_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_SOFT_OCP_WARN_TPU],
				  gs101_soft_tpu_warn_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_PMIC_120C],
				  gs101_pmic_120c_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_PMIC_140C],
				  gs101_pmic_140c_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg10_irq_work[IRQ_PMIC_OVERHEAT],
				  gs101_pmic_overheat_work);
	}
	if (strcmp(dev_name(&pdev->dev), google_gs101_id_table[1].name) == 0) {
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg11_irq_work[IRQ_OCP_WARN_GPU],
				  gs101_gpu_warn_work);
		INIT_DELAYED_WORK(&gs101_bcl_device->s2mpg11_irq_work[IRQ_SOFT_OCP_WARN_GPU],
				  gs101_soft_gpu_warn_work);
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
