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
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#if IS_ENABLED(CONFIG_SOC_GS201)
#include <dt-bindings/interrupt-controller/gs201.h>
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg12-register.h>
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/mfd/samsung/s2mpg13-register.h>
#else
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#endif
#include <linux/regulator/pmic_class.h>
#include <soc/google/bcl.h>
#include <soc/google/odpm.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

/* consistency checks in google_bcl_register_callback() */
#define bcl_cb_uvlo_read(bcl, m, v) (((bcl)->pmic_ops && (bcl)->intf_pmic_i2c) ? \
	(bcl)->pmic_ops->cb_uvlo_read(bcl->intf_pmic_i2c, m, v) : -ENODEV)
#define bcl_cb_uvlo_write(bcl, m, v) (((bcl)->pmic_ops && (bcl)->intf_pmic_i2c) ? \
	(bcl)->pmic_ops->cb_uvlo_write(bcl->intf_pmic_i2c, m, v) : -ENODEV)
#define bcl_cb_batoilo_read(bcl, v) (((bcl)->pmic_ops && (bcl)->intf_pmic_i2c) ? \
	(bcl)->pmic_ops->cb_batoilo_read(bcl->intf_pmic_i2c, v) : -ENODEV)
#define bcl_cb_batoilo_write(bcl, v) (((bcl)->pmic_ops && (bcl)->intf_pmic_i2c) ? \
	(bcl)->pmic_ops->cb_batoilo_write(bcl->intf_pmic_i2c, v) : -ENODEV)
#define bcl_cb_vdroop_ok(bcl, v) (((bcl)->pmic_ops && (bcl)->intf_pmic_i2c) ? \
	(bcl)->pmic_ops->cb_get_vdroop_ok(bcl->intf_pmic_i2c, v) : -ENODEV)

/* helpers for UVLO1 and UVLO2 */
#define bcl_cb_uvlo1_read(bcl, v)	bcl_cb_uvlo_read(bcl, TS_UVLO1, v)
#define bcl_cb_uvlo1_write(bcl, v)	bcl_cb_uvlo_write(bcl, TS_UVLO1, v)
#define bcl_cb_uvlo2_read(bcl, v)	bcl_cb_uvlo_read(bcl, TS_UVLO2, v)
#define bcl_cb_uvlo2_write(bcl, v)	bcl_cb_uvlo_write(bcl, TS_UVLO2, v)

/* This driver determines if HW was throttled due to SMPL/OCP */

#define CPUCL0_BASE (0x20c00000)
#define CPUCL1_BASE (0x20c10000)
#define CPUCL2_BASE (0x20c20000)
#if IS_ENABLED(CONFIG_SOC_GS201)
#define G3D_BASE (0x27F00000)
#else
#define G3D_BASE (0x1c400000)
#endif
#define TPU_BASE (0x1cc00000)
#define SYSREG_CPUCL0_BASE (0x20c40000)
#define CLUSTER0_GENERAL_CTRL_64 (0x1404)
#define CLKDIVSTEP (0x830)
#define VDROOP_FLT (0x838)
#define CPUCL0_CLKDIVSTEP_STAT (0x83c)
#define CPUCL0_CLKDIVSTEP_CON (0x838)
#define CPUCL12_CLKDIVSTEP_STAT (0x848)
#define CPUCL12_CLKDIVSTEP_CON_HEAVY (0x840)
#define CPUCL12_CLKDIVSTEP_CON_LIGHT (0x844)
#define CLKOUT (0x810)
#define G3D_CLKDIVSTEP_STAT (0x854)
#define TPU_CLKDIVSTEP_STAT (0x850)
#define CLUSTER0_MPMM (0x1408)
#define CLUSTER0_PPM (0x140c)
#define MPMMEN_MASK (0xF << 21)
#define PPMEN_MASK (0x3 << 8)
#define PPMCTL_MASK (0xFF)
#define OCP_WARN_MASK (0x1F)
#define SMPL_WARN_MASK (0xE0)
#if IS_ENABLED(CONFIG_SOC_GS101)
#define B3M_UPPER_LIMIT (9600)
#define B3M_LOWER_LIMIT (3400)
#define B3M_STEP (200)
#define B2M_UPPER_LIMIT (14400)
#define B2M_LOWER_LIMIT (5100)
#define B2M_STEP (300)
#define B10M_UPPER_LIMIT (14400)
#define B10M_LOWER_LIMIT (5100)
#define B10M_STEP (300)
#define B2S_UPPER_LIMIT (14400)
#define B2S_LOWER_LIMIT (5100)
#define B2S_STEP (300)
#endif
#if IS_ENABLED(CONFIG_SOC_GS201)
#define B3M_UPPER_LIMIT (8000)
#define B3M_LOWER_LIMIT (2688)
#define B3M_STEP (166)
#define B2M_UPPER_LIMIT (12000)
#define B2M_LOWER_LIMIT (4000)
#define B2M_STEP (250)
#define B10M_UPPER_LIMIT (12000)
#define B10M_LOWER_LIMIT (4000)
#define B10M_STEP (250)
#define B2S_UPPER_LIMIT (12000)
#define B2S_LOWER_LIMIT (4000)
#define B2S_STEP (250)
#endif
#define SMPL_BATTERY_VOLTAGE (4200)
#define SMPL_UPPER_LIMIT (3300)
#define SMPL_LOWER_LIMIT (2600)
#define SMPL_STEP (100)
#define SMPL_NUM_LVL (32)
#define ACTIVE_HIGH (0x1)
#define ACTIVE_LOW (0x0)
#define THERMAL_DELAY_INIT_MS 1000
#define PMIC_OVERHEAT_UPPER_LIMIT (2000)
#define PMIC_120C_UPPER_LIMIT (1200)
#define PMIC_140C_UPPER_LIMIT (1400)
#define PMU_ALIVE_CPU0_OUT (0x1CA0)
#define PMU_ALIVE_CPU1_OUT (0x1D20)
#define PMU_ALIVE_CPU2_OUT (0x1DA0)
#define PMU_ALIVE_TPU_OUT (0x2920)
#define PMU_ALIVE_GPU_OUT (0x1E20)
#define PMU_CLK_OUT (0x3E80)
#define THRESHOLD_DELAY_MS 50

#if IS_ENABLED(CONFIG_SOC_GS201)
#define MAIN 			S2MPG12
#define SUB 			S2MPG13
#define SMPL_WARN_CTRL		S2MPG12_PM_SMPL_WARN_CTRL
#define SMPL_WARN_SHIFT		S2MPG12_SMPL_WARN_LVL_SHIFT
#define OCP_WARN_LVL_SHIFT	S2MPG12_OCP_WARN_LVL_SHIFT
#define B3M_OCP_WARN		S2MPG12_PM_B3M_OCP_WARN
#define B3M_SOFT_OCP_WARN	S2MPG12_PM_B3M_SOFT_OCP_WARN
#define B2M_OCP_WARN		S2MPG12_PM_B2M_OCP_WARN
#define B2M_SOFT_OCP_WARN	S2MPG12_PM_B2M_SOFT_OCP_WARN
#define B10M_OCP_WARN		S2MPG12_PM_B10M_OCP_WARN
#define B10M_SOFT_OCP_WARN	S2MPG12_PM_B10M_SOFT_OCP_WARN
#define B2S_OCP_WARN		S2MPG13_PM_B2S_OCP_WARN
#define B2S_SOFT_OCP_WARN	S2MPG13_PM_B2S_SOFT_OCP_WARN
#define MAIN_CHIPID		S2MPG12_COMMON_CHIPID
#define SUB_CHIPID		S2MPG13_COMMON_CHIPID
#define INT3_120C		S2MPG12_IRQ_120C_INT3;
#define INT3_140C		S2MPG12_IRQ_140C_INT3;
#define INT3_TSD		S2MPG12_IRQ_TSD_INT3;
#define S2MPG1X_WRITE(pmic, bcl_dev, ret, args...)                             \
	do {                                                                   \
		switch (pmic) {                                                \
		case SUB:                                                      \
			ret = s2mpg13_write_reg(bcl_dev->sub_pmic_i2c, args);  \
			break;						       \
		case MAIN:                                                     \
			ret = s2mpg12_write_reg(bcl_dev->main_pmic_i2c, args); \
			break;						       \
		}                                                              \
	} while (0)

#define S2MPG1X_READ(pmic, bcl_dev, ret, args...)                              \
	do {                                                                   \
		switch (pmic) {                                                \
		case SUB:                                                      \
			ret = s2mpg13_read_reg(bcl_dev->sub_pmic_i2c, args);   \
			break;						       \
		case MAIN:                                                     \
			ret = s2mpg12_read_reg(bcl_dev->main_pmic_i2c, args);  \
			break;						       \
		}                                                              \
	} while (0)
#else
#define MAIN 			S2MPG10
#define SUB 			S2MPG11
#define SMPL_WARN_CTRL		S2MPG10_PM_SMPL_WARN_CTRL
#define SMPL_WARN_SHIFT		S2MPG10_SMPL_WARN_LVL_SHIFT
#define OCP_WARN_LVL_SHIFT	S2MPG10_OCP_WARN_LVL_SHIFT
#define B3M_OCP_WARN		S2MPG10_PM_B3M_OCP_WARN
#define B3M_SOFT_OCP_WARN	S2MPG10_PM_B3M_SOFT_OCP_WARN
#define B2M_OCP_WARN		S2MPG10_PM_B2M_OCP_WARN
#define B2M_SOFT_OCP_WARN	S2MPG10_PM_B2M_SOFT_OCP_WARN
#define B10M_OCP_WARN		S2MPG10_PM_B10M_OCP_WARN
#define B10M_SOFT_OCP_WARN	S2MPG10_PM_B10M_SOFT_OCP_WARN
#define B2S_OCP_WARN		S2MPG11_PM_B2S_OCP_WARN
#define B2S_SOFT_OCP_WARN	S2MPG11_PM_B2S_SOFT_OCP_WARN
#define MAIN_CHIPID		S2MPG10_COMMON_CHIPID
#define SUB_CHIPID		S2MPG11_COMMON_CHIPID
#define INT3_120C		S2MPG10_IRQ_120C_INT3;
#define INT3_140C		S2MPG10_IRQ_140C_INT3;
#define INT3_TSD		S2MPG10_IRQ_TSD_INT3;
#define S2MPG1X_WRITE(pmic, bcl_dev, ret, args...)                             \
	do {                                                                   \
		switch (pmic) {                                                \
		case SUB:                                                      \
			ret = s2mpg11_write_reg(bcl_dev->sub_pmic_i2c, args);  \
			break;						       \
		case MAIN:                                                     \
			ret = s2mpg10_write_reg(bcl_dev->main_pmic_i2c, args); \
			break;						       \
		}                                                              \
	} while (0)

#define S2MPG1X_READ(pmic, bcl_dev, ret, args...)                              \
	do {                                                                   \
		switch (pmic) {                                                \
		case SUB:                                                      \
			ret = s2mpg11_read_reg(bcl_dev->sub_pmic_i2c, args);   \
			break;						       \
		case MAIN:                                                     \
			ret = s2mpg10_read_reg(bcl_dev->main_pmic_i2c, args);  \
			break;						       \
		}                                                              \
	} while (0)

#endif

static const char * const triggered_source[] = {
	[SMPL_WARN] = "smpl_warn",
	[PMIC_120C] = "pmic_120c",
	[PMIC_140C] = "pmic_140c",
	[PMIC_OVERHEAT] = "pmic_overheat",
	[OCP_WARN_CPUCL1] = "ocp_cpu1",
	[OCP_WARN_CPUCL2] = "ocp_cpu2",
	[SOFT_OCP_WARN_CPUCL1] = "soft_ocp_cpu1",
	[SOFT_OCP_WARN_CPUCL2] = "soft_ocp_cpu2",
	[OCP_WARN_TPU] = "ocp_tpu",
	[SOFT_OCP_WARN_TPU] = "soft_ocp_tpu",
	[OCP_WARN_GPU] = "ocp_gpu",
	[SOFT_OCP_WARN_GPU] = "soft_ocp_gpu"};

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

static int bcl_odpm_map(int id)
{
#if IS_ENABLED(CONFIG_SOC_GS201)
	switch (id) {
	case OCP_WARN_GPU:
	case SOFT_OCP_WARN_GPU:
		return BUCK4;
	case OCP_WARN_TPU:
	case SOFT_OCP_WARN_TPU:
		return BUCK10;
	case OCP_WARN_CPUCL1:
	case SOFT_OCP_WARN_CPUCL1:
		return BUCK3;
	case OCP_WARN_CPUCL2:
	case SOFT_OCP_WARN_CPUCL2:
	default:
		return BUCK2;
	}
#endif
#if IS_ENABLED(CONFIG_SOC_GS101)
	switch (id) {
	case OCP_WARN_GPU:
	case SOFT_OCP_WARN_GPU:
		return 6;
	case OCP_WARN_CPUCL1:
	case SOFT_OCP_WARN_CPUCL1:
		return 4;
	case OCP_WARN_CPUCL2:
	case SOFT_OCP_WARN_CPUCL2:
	default:
		return 3;
	}
#endif
	return BUCK2;
}

static int triggered_read_level(void *data, int *val, int id)
{
	struct bcl_device *bcl_dev = data;
	u64 micro_unit[ODPM_CHANNEL_MAX];
	u64 odpm_current = 0;
	bool state = true;
	int polarity = (id == SMPL_WARN) ? 0 : 1;
	int gpio_pin = bcl_dev->vdroop1_pin;
	int gpio_level = (id >= UVLO1 && id <= BATOILO) ? gpio_get_value(gpio_pin) :
			gpio_get_value(bcl_dev->bcl_pin[id]);

	if ((id >= UVLO2 && id <= BATOILO) && (bcl_dev->bcl_tz_cnt[id] == 0)) {
		if (bcl_cb_vdroop_ok(bcl_dev, &state) < 0) {
			*val = 0;
			if (bcl_dev->bcl_prev_lvl[id] != 0) {
				mod_delayed_work(system_unbound_wq, &bcl_dev->bcl_irq_work[id],
						 msecs_to_jiffies(THRESHOLD_DELAY_MS));
				bcl_dev->bcl_prev_lvl[id] = *val;
			}
			return -EINVAL;
		} else
			gpio_level = (state) ? 0 : 1;
	}
	/* Check polarity */
	if ((gpio_level == polarity) || (bcl_dev->bcl_tz_cnt[id] == 1)) {
		*val = bcl_dev->bcl_lvl[id] + THERMAL_HYST_LEVEL;
		mod_delayed_work(system_unbound_wq, &bcl_dev->bcl_irq_work[id],
				 msecs_to_jiffies(THRESHOLD_DELAY_MS));
		bcl_dev->bcl_prev_lvl[id] = *val;
		return 0;
	}
	if (id >= UVLO1 && id <= BATOILO) {
		/* Zero is applied in case bcl_lvl[id] has a different value */
		*val = 0;
		if (bcl_dev->bcl_prev_lvl[id] != 0) {
			mod_delayed_work(system_unbound_wq, &bcl_dev->bcl_irq_work[id],
					 msecs_to_jiffies(THRESHOLD_DELAY_MS));
			bcl_dev->bcl_prev_lvl[id] = 0;
		}
		return 0;
	}

	/* If IRQ not asserted, check ODPM */
	if ((id == OCP_WARN_GPU) || (id == SOFT_OCP_WARN_GPU)) {
		mutex_lock(&bcl_dev->sub_odpm->lock);
		odpm_get_lpf_values(bcl_dev->sub_odpm, S2MPG1X_METER_CURRENT, micro_unit);
		odpm_current = micro_unit[bcl_odpm_map(id)] / 1000;
		mutex_unlock(&bcl_dev->sub_odpm->lock);
	} else if ((id > SMPL_WARN) && (id < OCP_WARN_TPU)) {
		mutex_lock(&bcl_dev->main_odpm->lock);
		odpm_get_lpf_values(bcl_dev->main_odpm, S2MPG1X_METER_CURRENT, micro_unit);
		odpm_current = micro_unit[bcl_odpm_map(id)] / 1000;
		mutex_unlock(&bcl_dev->main_odpm->lock);
	}

	*val = 0;
	bcl_dev->bcl_tz_cnt[id] = 0;
	if (odpm_current > (bcl_dev->bcl_lvl[id] / bcl_dev->odpm_ratio))
		*val = bcl_dev->bcl_lvl[id] + THERMAL_HYST_LEVEL;
	if (bcl_dev->bcl_prev_lvl[id] != *val) {
		mod_delayed_work(system_unbound_wq, &bcl_dev->bcl_irq_work[id],
				 msecs_to_jiffies(THRESHOLD_DELAY_MS));
		bcl_dev->bcl_prev_lvl[id] = *val;
	}
	return 0;
}

static struct power_supply *google_get_power_supply(struct bcl_device *bcl_dev)
{
	static struct power_supply *psy[2];
	static struct power_supply *batt_psy;
	int err = 0;

	batt_psy = NULL;
	err = power_supply_get_by_phandle_array(bcl_dev->device->of_node, "google,power-supply",
						psy, ARRAY_SIZE(psy));
	if (err > 0)
		batt_psy = psy[0];
	return batt_psy;
}

static void ocpsmpl_read_stats(struct bcl_device *bcl_dev,
			       struct ocpsmpl_stats *dst, struct power_supply *psy)
{
	union power_supply_propval ret = {0};
	int err = 0;

	if (!psy)
		return;
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

static u8 irq_to_id(struct bcl_device *bcl_dev, int irq)
{
	int i;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (bcl_dev->bcl_irq[i] == irq)
			return i;
	}
	return 0;
}

static irqreturn_t irq_handler(int irq, void *data)
{
	struct bcl_device *bcl_dev = data;
	u8 idx;
	if (!bcl_dev)
		return IRQ_HANDLED;

	idx = irq_to_id(bcl_dev, irq);

	if (bcl_dev->batt_psy_initialized) {
		atomic_inc(&bcl_dev->bcl_cnt[idx]);
		ocpsmpl_read_stats(bcl_dev, &bcl_dev->bcl_stats[idx], bcl_dev->batt_psy);
	}
	if ((bcl_dev->bcl_tz[idx]) && (bcl_dev->bcl_tz_cnt[idx] == 0)) {
		bcl_dev->bcl_tz_cnt[idx] = 1;
		bcl_dev->bcl_tz[idx]->temperature = 0;
		bcl_dev->bcl_prev_lvl[idx] = 0;
		thermal_zone_device_update(bcl_dev->bcl_tz[idx], THERMAL_EVENT_UNSPECIFIED);
	}
	return IRQ_HANDLED;
}

static void google_warn_work(struct work_struct *work, int idx)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  bcl_irq_work[idx].work);

	bcl_dev->bcl_tz_cnt[idx] = 0;
	if (bcl_dev->bcl_tz[idx])
		thermal_zone_device_update(bcl_dev->bcl_tz[idx], THERMAL_EVENT_UNSPECIFIED);
}

static void google_smpl_warn_work(struct work_struct *work)
{
	google_warn_work(work, SMPL_WARN);
}

static int smpl_warn_read_voltage(void *data, int *val)
{
	return triggered_read_level(data, val, SMPL_WARN);
}

static int ocp_cpu1_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, OCP_WARN_CPUCL1);
}

static void google_cpu1_warn_work(struct work_struct *work)
{
	google_warn_work(work, OCP_WARN_CPUCL1);
}

static int ocp_cpu2_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, OCP_WARN_CPUCL2);
}

static void google_cpu2_warn_work(struct work_struct *work)
{
	google_warn_work(work, OCP_WARN_CPUCL2);
}

static int soft_ocp_cpu1_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, SOFT_OCP_WARN_CPUCL1);
}

static void google_soft_cpu1_warn_work(struct work_struct *work)
{
	google_warn_work(work, SOFT_OCP_WARN_CPUCL1);
}

static int soft_ocp_cpu2_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, SOFT_OCP_WARN_CPUCL2);
}

static void google_soft_cpu2_warn_work(struct work_struct *work)
{
	google_warn_work(work, SOFT_OCP_WARN_CPUCL2);
}

static int ocp_tpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, OCP_WARN_TPU);
}

static void google_tpu_warn_work(struct work_struct *work)
{
	google_warn_work(work, OCP_WARN_TPU);
}

static int soft_ocp_tpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, SOFT_OCP_WARN_TPU);
}

static void google_soft_tpu_warn_work(struct work_struct *work)
{
	google_warn_work(work, SOFT_OCP_WARN_TPU);
}

static int ocp_gpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, OCP_WARN_GPU);
}

static void google_gpu_warn_work(struct work_struct *work)
{
	google_warn_work(work, OCP_WARN_GPU);
}

static int soft_ocp_gpu_read_current(void *data, int *val)
{
	return triggered_read_level(data, val, SOFT_OCP_WARN_GPU);
}

static void google_soft_gpu_warn_work(struct work_struct *work)
{
	google_warn_work(work, SOFT_OCP_WARN_GPU);
}

static void google_pmic_120c_work(struct work_struct *work)
{
	google_warn_work(work, PMIC_120C);
}

static int pmic_120c_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, PMIC_120C);
}

static void google_pmic_140c_work(struct work_struct *work)
{
	google_warn_work(work, PMIC_140C);
}

static int pmic_140c_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, PMIC_140C);
}

static void google_pmic_overheat_work(struct work_struct *work)
{
	google_warn_work(work, PMIC_OVERHEAT);
}

static int tsd_overheat_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, PMIC_OVERHEAT);
}

static int google_bcl_uvlo1_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, UVLO1);
}

static int google_bcl_uvlo2_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, UVLO2);
}

static int google_bcl_batoilo_read_temp(void *data, int *val)
{
	return triggered_read_level(data, val, BATOILO);
}

static int google_bcl_set_soc(void *data, int low, int high)
{
	struct bcl_device *bcl_dev = data;

	if (high == bcl_dev->trip_high_temp)
		return 0;

	mutex_lock(&bcl_dev->state_trans_lock);
	bcl_dev->trip_low_temp = low;
	bcl_dev->trip_high_temp = high;
	schedule_delayed_work(&bcl_dev->bcl_irq_work[PMIC_SOC], 0);

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
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  bcl_irq_work[PMIC_SOC].work);

	if (google_bcl_read_soc(bcl_dev, &battery_percentage_reverse))
		return;

	mutex_lock(&bcl_dev->state_trans_lock);
	if ((battery_percentage_reverse < bcl_dev->trip_high_temp) &&
		(battery_percentage_reverse > bcl_dev->trip_low_temp))
		goto eval_exit;

	bcl_dev->trip_val = battery_percentage_reverse;
	mutex_unlock(&bcl_dev->state_trans_lock);
	if (!bcl_dev->bcl_tz[PMIC_SOC]) {
		bcl_dev->bcl_tz[PMIC_SOC] =
				thermal_zone_of_sensor_register(bcl_dev->device,
								PMIC_SOC, bcl_dev,
								&bcl_dev->bcl_ops[PMIC_SOC]);
		if (IS_ERR(bcl_dev->bcl_tz[PMIC_SOC])) {
			dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
				PTR_ERR(bcl_dev->bcl_tz[PMIC_SOC]));
			return;
		}
	}
	if (!IS_ERR(bcl_dev->bcl_tz[PMIC_SOC]))
		thermal_zone_device_update(bcl_dev->bcl_tz[PMIC_SOC], THERMAL_EVENT_UNSPECIFIED);
	return;
eval_exit:
	mutex_unlock(&bcl_dev->state_trans_lock);
}

static int battery_supply_callback(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct bcl_device *bcl_dev = container_of(nb, struct bcl_device, psy_nb);
	struct power_supply *bcl_psy;

	if (!bcl_dev)
		return NOTIFY_OK;

	bcl_psy = bcl_dev->batt_psy;

	if (!bcl_psy || event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!strcmp(psy->desc->name, bcl_psy->desc->name))
		schedule_delayed_work(&bcl_dev->bcl_irq_work[PMIC_SOC], 0);

	return NOTIFY_OK;
}

static int google_bcl_remove_thermal(struct bcl_device *bcl_dev)
{
	int i = 0;

	power_supply_unreg_notifier(&bcl_dev->psy_nb);
	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (bcl_dev->bcl_tz[i])
			thermal_zone_of_sensor_unregister(bcl_dev->device, bcl_dev->bcl_tz[i]);
	}

	return 0;
}

static ssize_t batoilo_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[BATOILO]));
}

static DEVICE_ATTR_RO(batoilo_count);

static ssize_t vdroop2_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[UVLO2]));
}

static DEVICE_ATTR_RO(vdroop2_count);

static ssize_t vdroop1_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[UVLO1]));
}

static DEVICE_ATTR_RO(vdroop1_count);

static ssize_t smpl_warn_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[SMPL_WARN]));
}

static DEVICE_ATTR_RO(smpl_warn_count);

static ssize_t ocp_cpu1_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[OCP_WARN_CPUCL1]));
}

static DEVICE_ATTR_RO(ocp_cpu1_count);

static ssize_t ocp_cpu2_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[OCP_WARN_CPUCL2]));
}

static DEVICE_ATTR_RO(ocp_cpu2_count);

static ssize_t ocp_tpu_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[OCP_WARN_TPU]));
}

static DEVICE_ATTR_RO(ocp_tpu_count);

static ssize_t ocp_gpu_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[OCP_WARN_GPU]));
}

static DEVICE_ATTR_RO(ocp_gpu_count);

static ssize_t soft_ocp_cpu1_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[SOFT_OCP_WARN_CPUCL1]));
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_count);

static ssize_t soft_ocp_cpu2_count_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[SOFT_OCP_WARN_CPUCL2]));
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_count);

static ssize_t soft_ocp_tpu_count_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[SOFT_OCP_WARN_TPU]));
}

static DEVICE_ATTR_RO(soft_ocp_tpu_count);

static ssize_t soft_ocp_gpu_count_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", atomic_read(&bcl_dev->bcl_cnt[SOFT_OCP_WARN_GPU]));
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

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[BATOILO].capacity);
}

static DEVICE_ATTR_RO(batoilo_cap);

static ssize_t vdroop2_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[UVLO2].capacity);
}

static DEVICE_ATTR_RO(vdroop2_cap);

static ssize_t vdroop1_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[UVLO1].capacity);
}

static DEVICE_ATTR_RO(vdroop1_cap);

static ssize_t smpl_warn_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SMPL_WARN].capacity);
}

static DEVICE_ATTR_RO(smpl_warn_cap);

static ssize_t ocp_cpu1_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[OCP_WARN_CPUCL1].capacity);
}

static DEVICE_ATTR_RO(ocp_cpu1_cap);

static ssize_t ocp_cpu2_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[OCP_WARN_CPUCL2].capacity);
}

static DEVICE_ATTR_RO(ocp_cpu2_cap);

static ssize_t ocp_tpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[OCP_WARN_TPU].capacity);
}

static DEVICE_ATTR_RO(ocp_tpu_cap);

static ssize_t ocp_gpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[OCP_WARN_GPU].capacity);
}

static DEVICE_ATTR_RO(ocp_gpu_cap);

static ssize_t soft_ocp_cpu1_cap_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_CPUCL1].capacity);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_cap);

static ssize_t soft_ocp_cpu2_cap_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_CPUCL2].capacity);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_cap);

static ssize_t soft_ocp_tpu_cap_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_TPU].capacity);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_cap);

static ssize_t soft_ocp_gpu_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_GPU].capacity);
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

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[BATOILO].voltage);
}

static DEVICE_ATTR_RO(batoilo_volt);

static ssize_t vdroop2_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[UVLO2].voltage);
}

static DEVICE_ATTR_RO(vdroop2_volt);

static ssize_t vdroop1_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[UVLO1].voltage);
}

static DEVICE_ATTR_RO(vdroop1_volt);

static ssize_t smpl_warn_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SMPL_WARN].voltage);
}

static DEVICE_ATTR_RO(smpl_warn_volt);

static ssize_t ocp_cpu1_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[OCP_WARN_CPUCL1].voltage);
}

static DEVICE_ATTR_RO(ocp_cpu1_volt);

static ssize_t ocp_cpu2_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[OCP_WARN_CPUCL2].voltage);
}

static DEVICE_ATTR_RO(ocp_cpu2_volt);

static ssize_t ocp_tpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[OCP_WARN_TPU].voltage);
}

static DEVICE_ATTR_RO(ocp_tpu_volt);

static ssize_t ocp_gpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[OCP_WARN_GPU].voltage);
}

static DEVICE_ATTR_RO(ocp_gpu_volt);

static ssize_t soft_ocp_cpu1_volt_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_CPUCL1].voltage);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_volt);

static ssize_t soft_ocp_cpu2_volt_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_CPUCL2].voltage);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_volt);

static ssize_t soft_ocp_tpu_volt_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_TPU].voltage);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_volt);

static ssize_t soft_ocp_gpu_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_GPU].voltage);
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

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[BATOILO]._time);
}

static DEVICE_ATTR_RO(batoilo_time);

static ssize_t vdroop2_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[UVLO2]._time);
}

static DEVICE_ATTR_RO(vdroop2_time);

static ssize_t vdroop1_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[UVLO1]._time);
}

static DEVICE_ATTR_RO(vdroop1_time);

static ssize_t smpl_warn_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[SMPL_WARN]._time);
}

static DEVICE_ATTR_RO(smpl_warn_time);

static ssize_t ocp_cpu1_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[OCP_WARN_CPUCL1]._time);
}

static DEVICE_ATTR_RO(ocp_cpu1_time);

static ssize_t ocp_cpu2_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[OCP_WARN_CPUCL2]._time);
}

static DEVICE_ATTR_RO(ocp_cpu2_time);

static ssize_t ocp_tpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[OCP_WARN_TPU]._time);
}

static DEVICE_ATTR_RO(ocp_tpu_time);

static ssize_t ocp_gpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[OCP_WARN_GPU]._time);
}

static DEVICE_ATTR_RO(ocp_gpu_time);

static ssize_t soft_ocp_cpu1_time_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_CPUCL1]._time);
}

static DEVICE_ATTR_RO(soft_ocp_cpu1_time);

static ssize_t soft_ocp_cpu2_time_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_CPUCL2]._time);
}

static DEVICE_ATTR_RO(soft_ocp_cpu2_time);

static ssize_t soft_ocp_tpu_time_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_TPU]._time);
}

static DEVICE_ATTR_RO(soft_ocp_tpu_time);

static ssize_t soft_ocp_gpu_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%lld\n", bcl_dev->bcl_stats[SOFT_OCP_WARN_GPU]._time);
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

	if (!bcl_dev)
		return -EIO;
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

	if (!bcl_dev)
		return -EIO;
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

static int google_bcl_init_clk_div(struct bcl_device *bcl_dev, int idx, unsigned int value)
{
	void __iomem *addr;

	if (!bcl_dev)
		return -EIO;
	addr = get_addr_by_subsystem(bcl_dev, clk_stats_source[idx]);
	if (addr == NULL)
		return -EINVAL;

	mutex_lock(&bcl_dev->ratio_lock);
	__raw_writel(value, addr);
	mutex_unlock(&bcl_dev->ratio_lock);

	return 0;
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

	if (!bcl_dev)
		return -EIO;
	if (idx == TPU)
		bcl_dev->tpu_clkdivstep = value;
	else if (idx == GPU)
		bcl_dev->gpu_clkdivstep = value;
	else {
		if (idx == CPU2)
			bcl_dev->cpu2_clkdivstep = value;
		else if (idx == CPU1)
			bcl_dev->cpu1_clkdivstep = value;
		else
			bcl_dev->cpu0_clkdivstep = value;

		addr = get_addr_by_subsystem(bcl_dev, clk_stats_source[idx]);
		if (addr == NULL) {
			dev_err(bcl_dev->device, "IDX %d: Address is NULL\n", idx);
			return -EIO;
		}
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

static ssize_t vdroop_flt_show(struct bcl_device *bcl_dev, int idx, char *buf)
{
	unsigned int reg;
	void __iomem *addr;

	if (!bcl_dev)
		return -EIO;
	if (idx == TPU)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->tpu_vdroop_flt);
	else if (idx == GPU)
		return sysfs_emit(buf, "0x%x\n", bcl_dev->gpu_vdroop_flt);
	else if (idx >= CPU1 && idx <= CPU2)
		addr = bcl_dev->base_mem[idx] + VDROOP_FLT;
	else
		return sysfs_emit(buf, "off\n");
	reg = __raw_readl(addr);

	return sysfs_emit(buf, "0x%x\n", reg);
}

static ssize_t vdroop_flt_store(struct bcl_device *bcl_dev, int idx,
				const char *buf, size_t size)
{
	void __iomem *addr;
	unsigned int value;

	if (sscanf(buf, "0x%x", &value) != 1)
		return -EINVAL;

	if (!bcl_dev)
		return -EIO;
	if (idx == TPU)
		bcl_dev->tpu_vdroop_flt = value;
	else if (idx == GPU)
		bcl_dev->gpu_vdroop_flt = value;
	else if (idx >= CPU1 && idx <= CPU2) {
		addr = bcl_dev->base_mem[idx] + VDROOP_FLT;
		mutex_lock(&bcl_dev->ratio_lock);
		__raw_writel(value, addr);
		mutex_unlock(&bcl_dev->ratio_lock);
	} else
		return -EINVAL;

	return size;
}

static ssize_t cpu1_vdroop_flt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_show(bcl_dev, CPU1, buf);
}

static ssize_t cpu1_vdroop_flt_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_store(bcl_dev, CPU1, buf, size);
}

static DEVICE_ATTR_RW(cpu1_vdroop_flt);

static ssize_t cpu2_vdroop_flt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_show(bcl_dev, CPU2, buf);
}

static ssize_t cpu2_vdroop_flt_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_store(bcl_dev, CPU2, buf, size);
}

static DEVICE_ATTR_RW(cpu2_vdroop_flt);

static ssize_t tpu_vdroop_flt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_show(bcl_dev, TPU, buf);
}

static ssize_t tpu_vdroop_flt_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_store(bcl_dev, TPU, buf, size);
}

static DEVICE_ATTR_RW(tpu_vdroop_flt);

static ssize_t gpu_vdroop_flt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_show(bcl_dev, GPU, buf);
}

static ssize_t gpu_vdroop_flt_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return vdroop_flt_store(bcl_dev, GPU, buf, size);
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

	if (!bcl_dev)
		return -EIO;
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

	if (!bcl_dev)
		return -EIO;
	if (idx == TPU_HEAVY)
		bcl_dev->tpu_con_heavy = value;
	else if (idx == GPU_HEAVY)
		bcl_dev->gpu_con_heavy = value;
	else if (idx == TPU_LIGHT)
		bcl_dev->tpu_con_light = value;
	else if (idx == GPU_LIGHT)
		bcl_dev->gpu_con_light = value;
	else {
		addr = get_addr_by_rail(bcl_dev, clk_ratio_source[idx]);
		if (addr == NULL) {
			dev_err(bcl_dev->device, "IDX %d: Address is NULL\n", idx);
			return -EIO;
		}
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

static ssize_t uvlo1_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int uvlo1_lvl;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->intf_pmic_i2c)
		return -EBUSY;
	if (bcl_cb_uvlo1_read(bcl_dev, &uvlo1_lvl) < 0)
		return -EINVAL;
	bcl_dev->bcl_lvl[UVLO1] = VD_BATTERY_VOLTAGE - uvlo1_lvl - THERMAL_HYST_LEVEL;
	return sysfs_emit(buf, "%dmV\n", uvlo1_lvl);
}

static ssize_t uvlo1_lvl_store(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (!bcl_dev)
		return -EIO;
	if (value < VD_LOWER_LIMIT || value > VD_UPPER_LIMIT) {
		dev_err(bcl_dev->device, "UVLO1 %d outside of range %d - %d mV.", value,
			VD_LOWER_LIMIT, VD_UPPER_LIMIT);
		return -EINVAL;
	}
	if (!bcl_dev->intf_pmic_i2c)
		return -EIO;
	if (bcl_cb_uvlo1_write(bcl_dev, value) < 0)
		return -EIO;
	bcl_dev->bcl_lvl[UVLO1] = VD_BATTERY_VOLTAGE - value - THERMAL_HYST_LEVEL;
	ret = bcl_dev->bcl_tz[UVLO1]->ops->set_trip_temp(bcl_dev->bcl_tz[UVLO1], 0,
							 VD_BATTERY_VOLTAGE - value);
	if (bcl_dev->bcl_tz[UVLO1])
		thermal_zone_device_update(bcl_dev->bcl_tz[UVLO1], THERMAL_EVENT_UNSPECIFIED);
	if (ret)
		dev_err(bcl_dev->device, "Fail to set sys_uvlo1 trip temp\n");
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
	if (!bcl_dev->intf_pmic_i2c)
		return -EBUSY;
	if (bcl_cb_uvlo2_read(bcl_dev, &uvlo2_lvl) < 0)
		return -EINVAL;
	bcl_dev->bcl_lvl[UVLO2] = VD_BATTERY_VOLTAGE - uvlo2_lvl - THERMAL_HYST_LEVEL;
	return sysfs_emit(buf, "%umV\n", uvlo2_lvl);
}

static ssize_t uvlo2_lvl_store(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (!bcl_dev)
		return -EIO;
	if (value < VD_LOWER_LIMIT || value > VD_UPPER_LIMIT) {
		dev_err(bcl_dev->device, "UVLO2 %d outside of range %d - %d mV.", value,
			VD_LOWER_LIMIT, VD_UPPER_LIMIT);
		return -EINVAL;
	}
	if (!bcl_dev->intf_pmic_i2c)
		return -EIO;
	if (bcl_cb_uvlo2_write(bcl_dev, value) < 0)
		return -EIO;
	bcl_dev->bcl_lvl[UVLO2] = VD_BATTERY_VOLTAGE - value - THERMAL_HYST_LEVEL;
	ret = bcl_dev->bcl_tz[UVLO2]->ops->set_trip_temp(bcl_dev->bcl_tz[UVLO2], 0,
							 VD_BATTERY_VOLTAGE - value);
	if (bcl_dev->bcl_tz[UVLO2])
		thermal_zone_device_update(bcl_dev->bcl_tz[UVLO2], THERMAL_EVENT_UNSPECIFIED);
	if (ret)
		dev_err(bcl_dev->device, "Fail to set sys_uvlo2 trip temp\n");
	return size;
}

static DEVICE_ATTR_RW(uvlo2_lvl);

static ssize_t batoilo_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int batoilo_lvl;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->intf_pmic_i2c)
		return -EBUSY;
	if (bcl_cb_batoilo_read(bcl_dev, &batoilo_lvl) < 0)
		return -EINVAL;
	bcl_dev->bcl_lvl[BATOILO] = batoilo_lvl;
	return sysfs_emit(buf, "%umA\n", batoilo_lvl);
}

static ssize_t batoilo_lvl_store(struct device *dev,
				 struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	unsigned int value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (!bcl_dev)
		return -EIO;
	if (value < BO_LOWER_LIMIT || value > BO_UPPER_LIMIT) {
		dev_err(bcl_dev->device, "BATOILO %d outside of range %d - %d mA.", value,
			BO_LOWER_LIMIT, BO_UPPER_LIMIT);
		return -EINVAL;
	}
	if (bcl_cb_batoilo_write(bcl_dev, value) < 0)
		return -EIO;
	bcl_dev->bcl_lvl[BATOILO] = value - THERMAL_HYST_LEVEL;
	ret = bcl_dev->bcl_tz[BATOILO]->ops->set_trip_temp(bcl_dev->bcl_tz[BATOILO], 0, value);
	if (bcl_dev->bcl_tz[BATOILO])
		thermal_zone_device_update(bcl_dev->bcl_tz[BATOILO], THERMAL_EVENT_UNSPECIFIED);
	if (ret)
		dev_err(bcl_dev->device, "Fail to set sys_uvlo2 trip temp\n");
	return size;
}

static DEVICE_ATTR_RW(batoilo_lvl);

static ssize_t smpl_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u8 value = 0;
	int ret;
	unsigned int smpl_warn_lvl;

	if (!bcl_dev)
		return -EIO;
	if (!bcl_dev->main_pmic_i2c) {
		return -EBUSY;
	}
	S2MPG1X_READ(MAIN, bcl_dev, ret, SMPL_WARN_CTRL, &value);
	if (ret)
		return -EINVAL;
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
	S2MPG1X_READ(MAIN, bcl_dev, ret, SMPL_WARN_CTRL, &value);
	if (ret) {
		dev_err(bcl_dev->device, "S2MPG1X read 0x%x failed.", SMPL_WARN_CTRL);
		return -EBUSY;
	}
	value &= ~SMPL_WARN_MASK;
	value |= ((val - SMPL_LOWER_LIMIT) / 100) << SMPL_WARN_SHIFT;
	S2MPG1X_WRITE(MAIN, bcl_dev, ret, SMPL_WARN_CTRL, value);

	if (ret) {
		dev_err(bcl_dev->device, "i2c write error setting smpl_warn\n");
		return ret;
	}
	bcl_dev->bcl_lvl[SMPL_WARN] = SMPL_BATTERY_VOLTAGE - val - THERMAL_HYST_LEVEL;
	ret = bcl_dev->bcl_tz[SMPL_WARN]->ops->set_trip_temp(bcl_dev->bcl_tz[SMPL_WARN], 0,
							     SMPL_BATTERY_VOLTAGE - val);
	if (ret)
		dev_err(bcl_dev->device, "Fail to set smpl_warn trip temp\n");
	if (bcl_dev->bcl_tz[SMPL_WARN])
		thermal_zone_device_update(bcl_dev->bcl_tz[SMPL_WARN], THERMAL_EVENT_UNSPECIFIED);

	return size;

}

static DEVICE_ATTR_RW(smpl_lvl);

static int get_ocp_lvl(struct bcl_device *bcl_dev, u64 *val, u8 addr, u8 pmic, u8 mask, u16 limit,
		       u16 step)
{
	u8 value = 0;
	int ret;
	unsigned int ocp_warn_lvl;

	if (!bcl_dev)
		return -EIO;
	S2MPG1X_READ(pmic, bcl_dev, ret, addr, &value);
	if (ret) {
		dev_err(bcl_dev->device, "S2MPG1X read 0x%x failed.", addr);
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
	if (val < llimit || val > ulimit) {
		dev_err(bcl_dev->device, "OCP_WARN LEVEL %llu outside of range %d - %d mA.", val,
		       llimit, ulimit);
		return -EBUSY;
	}
	mutex_lock(&bcl_dev->bcl_irq_lock[id]);
	S2MPG1X_READ(pmic, bcl_dev, ret, addr, &value);
	if (ret) {
		dev_err(bcl_dev->device, "S2MPG1X read 0x%x failed.", addr);
		mutex_unlock(&bcl_dev->bcl_irq_lock[id]);
		return -EBUSY;
	}
	value &= ~(OCP_WARN_MASK) << OCP_WARN_LVL_SHIFT;
	value |= ((ulimit - val) / step) << OCP_WARN_LVL_SHIFT;
	S2MPG1X_WRITE(pmic, bcl_dev, ret, addr, value);
	if (!ret) {
		bcl_dev->bcl_lvl[id] = val - THERMAL_HYST_LEVEL;
		ret = bcl_dev->bcl_tz[id]->ops->set_trip_temp(bcl_dev->bcl_tz[id], 0, val);
		if (ret)
			dev_err(bcl_dev->device, "Fail to set ocp_warn trip temp\n");
	}
	mutex_unlock(&bcl_dev->bcl_irq_lock[id]);
	if (bcl_dev->bcl_tz[id])
		thermal_zone_device_update(bcl_dev->bcl_tz[id], THERMAL_EVENT_UNSPECIFIED);

	return ret;
}

static ssize_t ocp_cpu1_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, B3M_OCP_WARN, MAIN, OCP_WARN_MASK, B3M_UPPER_LIMIT,
			B3M_STEP) < 0)
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

	if (set_ocp_lvl(bcl_dev, value, B3M_OCP_WARN, MAIN, OCP_WARN_MASK, B3M_LOWER_LIMIT,
			B3M_UPPER_LIMIT, B3M_STEP, OCP_WARN_CPUCL1) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_cpu1_lvl);

static ssize_t ocp_cpu2_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, B2M_OCP_WARN, MAIN, OCP_WARN_MASK, B2M_UPPER_LIMIT,
			B2M_STEP) < 0)
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

	if (set_ocp_lvl(bcl_dev, value, B2M_OCP_WARN, MAIN, OCP_WARN_MASK, B2M_LOWER_LIMIT,
			B2M_UPPER_LIMIT, B2M_STEP, OCP_WARN_CPUCL2) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_cpu2_lvl);

static ssize_t ocp_tpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, B10M_OCP_WARN, MAIN, OCP_WARN_MASK, B10M_UPPER_LIMIT,
			B10M_STEP) < 0)
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

	if (set_ocp_lvl(bcl_dev, value, B10M_OCP_WARN, MAIN, OCP_WARN_MASK, B10M_LOWER_LIMIT,
			B10M_UPPER_LIMIT, B10M_STEP, OCP_WARN_TPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_tpu_lvl);

static ssize_t ocp_gpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, B2S_OCP_WARN, SUB, OCP_WARN_MASK, B2S_UPPER_LIMIT,
			B2S_STEP) < 0)
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

	if (set_ocp_lvl(bcl_dev, value, B2S_OCP_WARN, SUB, OCP_WARN_MASK, B2S_LOWER_LIMIT,
			B2S_UPPER_LIMIT, B2S_STEP, OCP_WARN_GPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(ocp_gpu_lvl);

static ssize_t soft_ocp_cpu1_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, B3M_SOFT_OCP_WARN, MAIN, OCP_WARN_MASK, B3M_UPPER_LIMIT,
			B3M_STEP) < 0)
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

	if (set_ocp_lvl(bcl_dev, value, B3M_SOFT_OCP_WARN, MAIN, OCP_WARN_MASK, B3M_LOWER_LIMIT,
			B3M_UPPER_LIMIT, B3M_STEP, SOFT_OCP_WARN_CPUCL1) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_cpu1_lvl);

static ssize_t soft_ocp_cpu2_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, B2M_SOFT_OCP_WARN, MAIN, OCP_WARN_MASK, B2M_UPPER_LIMIT,
			B2M_STEP) < 0)
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

	if (set_ocp_lvl(bcl_dev, value, B2M_SOFT_OCP_WARN, MAIN, OCP_WARN_MASK, B2M_LOWER_LIMIT,
			B2M_UPPER_LIMIT, B2M_STEP, SOFT_OCP_WARN_CPUCL2) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_cpu2_lvl);

static ssize_t soft_ocp_tpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, B10M_SOFT_OCP_WARN, MAIN, OCP_WARN_MASK, B10M_UPPER_LIMIT,
			B10M_STEP) < 0)
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

	if (set_ocp_lvl(bcl_dev, value, B10M_SOFT_OCP_WARN, MAIN, OCP_WARN_MASK, B10M_LOWER_LIMIT,
			B10M_UPPER_LIMIT, B10M_STEP, SOFT_OCP_WARN_TPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_tpu_lvl);

static ssize_t soft_ocp_gpu_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);
	u64 val;

	if (get_ocp_lvl(bcl_dev, &val, B2S_SOFT_OCP_WARN, SUB, OCP_WARN_MASK, B2S_UPPER_LIMIT,
			B2S_STEP) < 0)
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

	if (set_ocp_lvl(bcl_dev, value, B2S_SOFT_OCP_WARN, SUB, OCP_WARN_MASK, B2S_LOWER_LIMIT,
			B2S_UPPER_LIMIT, B2S_STEP, SOFT_OCP_WARN_GPU) < 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR_RW(soft_ocp_gpu_lvl);

static struct attribute *triggered_lvl_attrs[] = {
	&dev_attr_uvlo1_lvl.attr,
	&dev_attr_uvlo2_lvl.attr,
	&dev_attr_batoilo_lvl.attr,
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

static ssize_t ready_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d\n", bcl_dev->ready);
}
static DEVICE_ATTR_RO(ready);

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

int google_bcl_register_ifpmic(struct bcl_device *bcl_dev,
			       const struct bcl_ifpmic_ops *pmic_ops)
{
	if (!bcl_dev)
		return -EIO;

	if (!pmic_ops || !pmic_ops->cb_get_vdroop_ok ||
	    !pmic_ops->cb_uvlo_read || !pmic_ops->cb_uvlo_write ||
	    !pmic_ops->cb_batoilo_read || !pmic_ops->cb_batoilo_write)
		return -EINVAL;

	bcl_dev->pmic_ops = pmic_ops;

	return 0;
}
EXPORT_SYMBOL_GPL(google_bcl_register_ifpmic);

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
	addr = data->base_mem[TPU] + VDROOP_FLT;
	__raw_writel(data->tpu_vdroop_flt, addr);
	addr = data->base_mem[TPU] + CLKOUT;
	__raw_writel(data->tpu_clk_out, addr);
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
	addr = data->base_mem[GPU] + VDROOP_FLT;
	__raw_writel(data->gpu_vdroop_flt, addr);
	addr = data->base_mem[GPU] + CLKOUT;
	__raw_writel(data->gpu_clk_out, addr);
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
	&dev_attr_ready.attr,
	NULL,
};

static const struct attribute_group instr_group = {
	.attrs = instr_attrs,
	.name = "instruction",
};

static int google_bcl_register_irq(struct bcl_device *bcl_dev, int id, const char *devname,
				   u32 intr_flag)
{
	int ret = 0;

	ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->bcl_irq[id], NULL, irq_handler,
					intr_flag | IRQF_ONESHOT, devname, bcl_dev);
	if (ret < 0) {
		dev_err(bcl_dev->device, "Failed to request IRQ: %d: %d\n", bcl_dev->bcl_irq[id],
			ret);
		return ret;
	}

	bcl_dev->bcl_tz[id] = thermal_zone_of_sensor_register(bcl_dev->device, id, bcl_dev,
							      &bcl_dev->bcl_ops[id]);
	if (IS_ERR(bcl_dev->bcl_tz[id])) {
		dev_err(bcl_dev->device, "TZ register failed. %d, err:%ld\n", id,
			PTR_ERR(bcl_dev->bcl_tz[id]));
	} else {
		thermal_zone_device_enable(bcl_dev->bcl_tz[id]);
		thermal_zone_device_update(bcl_dev->bcl_tz[id], THERMAL_DEVICE_UP);
	}
	return ret;
}

static int get_cpu0clk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;
	unsigned int value;

	addr = bcl_dev->base_mem[CPU0] + CLKOUT;
	*val = __raw_readl(addr);
	exynos_pmu_read(PMU_CLK_OUT, &value);
	pr_info("PMU CLK OUT: 0x%x\n", value);
	return 0;
}

static int set_cpu0clk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;

	addr = bcl_dev->base_mem[CPU0] + CLKOUT;
	__raw_writel(val, addr);
	if (val != 0)
		exynos_pmu_write(PMU_CLK_OUT, 0x3001);
	else
		exynos_pmu_write(PMU_CLK_OUT, 0x0);
	return 0;
}

static int get_cpu1clk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;
	unsigned int value;

	addr = bcl_dev->base_mem[CPU1] + CLKOUT;
	*val = __raw_readl(addr);
	exynos_pmu_read(PMU_CLK_OUT, &value);
	pr_info("PMU CLK OUT: 0x%x\n", value);
	return 0;
}

static int set_cpu1clk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;

	addr = bcl_dev->base_mem[CPU1] + CLKOUT;
	__raw_writel(val, addr);
	if (val != 0)
		exynos_pmu_write(PMU_CLK_OUT, 0x1101);
	else
		exynos_pmu_write(PMU_CLK_OUT, 0x0);
	return 0;
}

static int get_cpu2clk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;
	unsigned int value;

	addr = bcl_dev->base_mem[CPU2] + CLKOUT;
	*val = __raw_readl(addr);
	exynos_pmu_read(PMU_CLK_OUT, &value);
	pr_info("PMU CLK OUT: 0x%x\n", value);
	return 0;
}

static int set_cpu2clk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;
	void __iomem *addr;

	addr = bcl_dev->base_mem[CPU2] + CLKOUT;
	__raw_writel(val, addr);
	if (val != 0)
		exynos_pmu_write(PMU_CLK_OUT, 0x1201);
	else
		exynos_pmu_write(PMU_CLK_OUT, 0x0);
	return 0;
}

static int get_gpuclk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	unsigned int value;

	*val = bcl_dev->gpu_clk_out;
	exynos_pmu_read(PMU_CLK_OUT, &value);
	pr_info("PMU CLK OUT: 0x%x\n", value);
	return 0;
}

static int set_gpuclk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	bcl_dev->gpu_clk_out = val;
	if (val != 0)
		exynos_pmu_write(PMU_CLK_OUT, 0x1a01);
	else
		exynos_pmu_write(PMU_CLK_OUT, 0x0);
	return 0;
}

static int get_tpuclk(void *data, u64 *val)
{
	struct bcl_device *bcl_dev = data;
	unsigned int value;

	*val = bcl_dev->tpu_clk_out;
	exynos_pmu_read(PMU_CLK_OUT, &value);
	pr_info("PMU CLK OUT: 0x%x\n", value);
	return 0;
}

static int set_tpuclk(void *data, u64 val)
{
	struct bcl_device *bcl_dev = data;

	bcl_dev->tpu_clk_out = val;
	if (val != 0)
		exynos_pmu_write(PMU_CLK_OUT, 0x2E01);
	else
		exynos_pmu_write(PMU_CLK_OUT, 0x0);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpu0_clkout_fops, get_cpu0clk, set_cpu0clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(cpu1_clkout_fops, get_cpu1clk, set_cpu1clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(cpu2_clkout_fops, get_cpu2clk, set_cpu2clk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(gpu_clkout_fops, get_gpuclk, set_gpuclk, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(tpu_clkout_fops, get_tpuclk, set_tpuclk, "0x%llx\n");

static void google_init_debugfs(struct bcl_device *bcl_dev)
{
	bcl_dev->debug_entry = debugfs_create_dir("google_bcl", 0);
	debugfs_create_file("cpu0_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu0_clkout_fops);
	debugfs_create_file("cpu1_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu1_clkout_fops);
	debugfs_create_file("cpu2_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &cpu2_clkout_fops);
	debugfs_create_file("gpu_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &gpu_clkout_fops);
	debugfs_create_file("tpu_clk_out", 0644, bcl_dev->debug_entry, bcl_dev, &tpu_clkout_fops);
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
#if IS_ENABLED(CONFIG_SOC_GS201)
	struct s2mpg13_platform_data *pdata_sub;
	struct s2mpg13_dev *sub_dev = NULL;
#else
	struct s2mpg11_platform_data *pdata_sub;
	struct s2mpg11_dev *sub_dev = NULL;
#endif
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
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
		sub_dev = i2c_get_clientdata(i2c);
	}
	of_node_put(p_np);
	if (!sub_dev) {
		dev_err(bcl_dev->device, "SUB PMIC device not found\n");
		return -ENODEV;
	}
	pdata_sub = dev_get_platdata(sub_dev->dev);
	bcl_dev->sub_odpm = pdata_sub->meter;
	bcl_dev->sub_pmic_i2c = sub_dev->pmic;
	bcl_dev->sub_dev = sub_dev->dev;
	bcl_dev->bcl_lvl[OCP_WARN_GPU] = B2S_UPPER_LIMIT - THERMAL_HYST_LEVEL -
			(pdata_sub->b2_ocp_warn_lvl * B2S_STEP);
	bcl_dev->bcl_lvl[SOFT_OCP_WARN_GPU] = B2S_UPPER_LIMIT - THERMAL_HYST_LEVEL -
			(pdata_sub->b2_soft_ocp_warn_lvl * B2S_STEP);
	bcl_dev->bcl_pin[OCP_WARN_GPU] = pdata_sub->b2_ocp_warn_pin;
	bcl_dev->bcl_pin[SOFT_OCP_WARN_GPU] = pdata_sub->b2_soft_ocp_warn_pin;
	bcl_dev->bcl_irq[OCP_WARN_GPU] = gpio_to_irq(pdata_sub->b2_ocp_warn_pin);
	bcl_dev->bcl_irq[SOFT_OCP_WARN_GPU] = gpio_to_irq(pdata_sub->b2_soft_ocp_warn_pin);
	bcl_dev->bcl_ops[OCP_WARN_GPU].get_temp = ocp_gpu_read_current;
	bcl_dev->bcl_ops[SOFT_OCP_WARN_GPU].get_temp = soft_ocp_gpu_read_current;
	S2MPG1X_READ(SUB, bcl_dev, ret, SUB_CHIPID, &val);
	if (ret) {
		dev_err(bcl_dev->device, "Failed to read PMIC chipid.\n");
		return -ENODEV;
	}
#if IS_ENABLED(CONFIG_SOC_GS201)
	S2MPG1X_READ(SUB, bcl_dev, ret, S2MPG13_PM_OFFSRC, &val);
	pr_info("S2MPG13 OFFSRC: %#x\n", val);
	S2MPG1X_WRITE(SUB, bcl_dev, ret, S2MPG13_PM_OFFSRC, 0);
#endif

	ret = google_bcl_register_irq(bcl_dev, OCP_WARN_GPU, "GPU_OCP_IRQ", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: GPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, SOFT_OCP_WARN_GPU, "SOFT_GPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_GPU\n");
		return -ENODEV;
	}
	return 0;
}

static void google_bcl_uvlo1_irq_work(struct work_struct *work)
{
	google_warn_work(work, UVLO1);
}

static void google_bcl_uvlo2_irq_work(struct work_struct *work)
{
	google_warn_work(work, UVLO2);
}

static void google_bcl_batoilo_irq_work(struct work_struct *work)
{
	google_warn_work(work, BATOILO);
}

void google_bcl_irq_changed(struct bcl_device *bcl_dev, int index)
{
	if (!bcl_dev)
		return;
	atomic_inc(&bcl_dev->bcl_cnt[index]);
	ocpsmpl_read_stats(bcl_dev, &bcl_dev->bcl_stats[index], bcl_dev->batt_psy);
	if ((bcl_dev->bcl_tz[index]) && (bcl_dev->bcl_tz_cnt[index] == 0)) {
		bcl_dev->bcl_tz_cnt[index] = 1;
		bcl_dev->bcl_tz[index]->temperature = 0;
		thermal_zone_device_update(bcl_dev->bcl_tz[index], THERMAL_EVENT_UNSPECIFIED);
	}
}
EXPORT_SYMBOL_GPL(google_bcl_irq_changed);

static void google_set_intf_pmic_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device, init_work.work);
	int ret = 0;
	unsigned int uvlo1_lvl, uvlo2_lvl, batoilo_lvl;

	if (!bcl_dev->intf_pmic_i2c)
		goto retry_init_work;
	if (IS_ERR_OR_NULL(bcl_dev->pmic_ops) || IS_ERR_OR_NULL(bcl_dev->pmic_ops->cb_uvlo_read))
		goto retry_init_work;
	if (bcl_cb_uvlo1_read(bcl_dev, &uvlo1_lvl) < 0)
		goto retry_init_work;
	if (bcl_cb_uvlo2_read(bcl_dev, &uvlo2_lvl) < 0)
		goto retry_init_work;
	if (bcl_cb_batoilo_read(bcl_dev, &batoilo_lvl) < 0)
		goto retry_init_work;

	bcl_dev->batt_psy = google_get_power_supply(bcl_dev);
	bcl_dev->bcl_tz[PMIC_SOC] = thermal_zone_of_sensor_register(bcl_dev->device,
								    PMIC_SOC, bcl_dev,
								    &bcl_dev->bcl_ops[PMIC_SOC]);
	bcl_dev->bcl_ops[PMIC_SOC].get_temp = google_bcl_read_soc;
	bcl_dev->bcl_ops[PMIC_SOC].set_trips = google_bcl_set_soc;
	if (IS_ERR(bcl_dev->bcl_tz[PMIC_SOC])) {
		dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
			PTR_ERR(bcl_dev->bcl_tz[PMIC_SOC]));
		ret = PTR_ERR(bcl_dev->bcl_tz[PMIC_SOC]);
		bcl_dev->bcl_tz[PMIC_SOC]= NULL;
	} else {
		bcl_dev->psy_nb.notifier_call = battery_supply_callback;
		ret = power_supply_reg_notifier(&bcl_dev->psy_nb);
		if (ret < 0)
			dev_err(bcl_dev->device,
				"soc notifier registration error. defer. err:%d\n", ret);
		thermal_zone_device_update(bcl_dev->bcl_tz[PMIC_SOC], THERMAL_DEVICE_UP);
	}
	bcl_dev->batt_psy_initialized = false;

	bcl_dev->bcl_lvl[UVLO1] = VD_BATTERY_VOLTAGE - uvlo1_lvl - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_lvl[UVLO2] = VD_BATTERY_VOLTAGE - uvlo2_lvl - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_lvl[BATOILO] = batoilo_lvl - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_ops[UVLO1].get_temp = google_bcl_uvlo1_read_temp;
	bcl_dev->bcl_ops[UVLO2].get_temp = google_bcl_uvlo2_read_temp;
	bcl_dev->bcl_ops[BATOILO].get_temp = google_bcl_batoilo_read_temp;

	bcl_dev->bcl_tz[UVLO1] = thermal_zone_of_sensor_register(bcl_dev->device, UVLO1,
								 bcl_dev,
								 &bcl_dev->bcl_ops[UVLO1]);
	if (IS_ERR(bcl_dev->bcl_tz[UVLO1])) {
		dev_err(bcl_dev->device, "TZ register vdroop%d failed, err:%ld\n", UVLO1,
			PTR_ERR(bcl_dev->bcl_tz[UVLO1]));
	} else {
		thermal_zone_device_enable(bcl_dev->bcl_tz[UVLO1]);
		thermal_zone_device_update(bcl_dev->bcl_tz[UVLO1], THERMAL_DEVICE_UP);
	}
	bcl_dev->bcl_tz[UVLO2] = thermal_zone_of_sensor_register(bcl_dev->device, UVLO2,
								 bcl_dev,
								 &bcl_dev->bcl_ops[UVLO2]);
	if (IS_ERR(bcl_dev->bcl_tz[UVLO2])) {
		dev_err(bcl_dev->device, "TZ register vdroop%d failed, err:%ld\n", UVLO2,
			PTR_ERR(bcl_dev->bcl_tz[UVLO2]));
	} else {
		thermal_zone_device_enable(bcl_dev->bcl_tz[UVLO2]);
		thermal_zone_device_update(bcl_dev->bcl_tz[UVLO2], THERMAL_DEVICE_UP);
	}
	bcl_dev->bcl_tz[BATOILO] = thermal_zone_of_sensor_register(bcl_dev->device, BATOILO,
								   bcl_dev,
								   &bcl_dev->bcl_ops[BATOILO]);
	if (IS_ERR(bcl_dev->bcl_tz[BATOILO])) {
		dev_err(bcl_dev->device, "TZ register vdroop%d failed, err:%ld\n", BATOILO,
			PTR_ERR(bcl_dev->bcl_tz[BATOILO]));
	} else {
		thermal_zone_device_enable(bcl_dev->bcl_tz[BATOILO]);
		thermal_zone_device_update(bcl_dev->bcl_tz[BATOILO], THERMAL_DEVICE_UP);
	}

	bcl_dev->ready = true;
	return;

retry_init_work:
	schedule_delayed_work(&bcl_dev->init_work, msecs_to_jiffies(THERMAL_DELAY_INIT_MS));
}

static int google_set_intf_pmic(struct bcl_device *bcl_dev)
{
	int ret = 0;
	u8 val;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
#if IS_ENABLED(CONFIG_SOC_GS201)
	struct s2mpg12_platform_data *pdata_main;
#else
	struct s2mpg10_platform_data *pdata_main;
#endif
	p_np = of_parse_phandle(np, "google,charger", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find Charger I2C\n");
			return -ENODEV;
		}
		bcl_dev->intf_pmic_i2c = i2c;
	}
	of_node_put(p_np);
	if (!bcl_dev->intf_pmic_i2c) {
		dev_err(bcl_dev->device, "Interface PMIC device not found\n");
		return -ENODEV;
	}

	pdata_main = dev_get_platdata(bcl_dev->main_dev);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[PMIC_SOC], google_bcl_evaluate_soc);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[PMIC_120C], google_pmic_120c_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[PMIC_140C], google_pmic_140c_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[PMIC_OVERHEAT], google_pmic_overheat_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[UVLO1], google_bcl_uvlo1_irq_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[UVLO2], google_bcl_uvlo2_irq_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[BATOILO], google_bcl_batoilo_irq_work);
	bcl_dev->bcl_irq[PMIC_120C] = pdata_main->irq_base + INT3_120C;
	bcl_dev->bcl_irq[PMIC_140C] = pdata_main->irq_base + INT3_140C;
	bcl_dev->bcl_irq[PMIC_OVERHEAT] = pdata_main->irq_base + INT3_TSD;
	bcl_dev->bcl_ops[PMIC_120C].get_temp = pmic_120c_read_temp;
	bcl_dev->bcl_ops[PMIC_140C].get_temp = pmic_140c_read_temp;
	bcl_dev->bcl_ops[PMIC_OVERHEAT].get_temp = tsd_overheat_read_temp;
	S2MPG1X_READ(MAIN, bcl_dev, ret, MAIN_CHIPID, &val);
	if (ret) {
		dev_err(bcl_dev->device, "Failed to read MAIN chipid.\n");
		return -ENODEV;
	}
	bcl_dev->bcl_lvl[PMIC_120C] = PMIC_120C_UPPER_LIMIT - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_lvl[PMIC_140C] = PMIC_140C_UPPER_LIMIT - THERMAL_HYST_LEVEL;
	bcl_dev->bcl_lvl[PMIC_OVERHEAT] = PMIC_OVERHEAT_UPPER_LIMIT - THERMAL_HYST_LEVEL;

	ret = google_bcl_register_irq(bcl_dev, PMIC_120C, "PMIC_120C", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_120C\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, PMIC_140C, "PMIC_140C", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_140C\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, PMIC_OVERHEAT, "PMIC_OVERHEAT",
				      IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_OVERHEAT\n");
		return -ENODEV;
	}

	return 0;
}

static int google_set_main_pmic(struct bcl_device *bcl_dev)
{
#if IS_ENABLED(CONFIG_SOC_GS201)
	struct s2mpg12_platform_data *pdata_main;
	struct s2mpg12_dev *main_dev = NULL;
#else
	struct s2mpg10_platform_data *pdata_main;
	struct s2mpg10_dev *main_dev = NULL;
#endif
	u8 val;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	bool bypass_smpl_warn = false;
	int ret, i;

	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SMPL_WARN], google_smpl_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[OCP_WARN_TPU], google_tpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[OCP_WARN_GPU], google_gpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SOFT_OCP_WARN_TPU], google_soft_tpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SOFT_OCP_WARN_GPU], google_soft_gpu_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[OCP_WARN_CPUCL2], google_cpu2_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[OCP_WARN_CPUCL1], google_cpu1_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SOFT_OCP_WARN_CPUCL2], google_soft_cpu2_warn_work);
	INIT_DELAYED_WORK(&bcl_dev->bcl_irq_work[SOFT_OCP_WARN_CPUCL1], google_soft_cpu1_warn_work);

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		bcl_dev->bcl_tz_cnt[i] = 0;
		bcl_dev->bcl_prev_lvl[i] = 0;
		atomic_set(&bcl_dev->bcl_cnt[i], 0);
		mutex_init(&bcl_dev->bcl_irq_lock[i]);
	}
	p_np = of_parse_phandle(np, "google,main-power", 0);
	if (p_np) {
		i2c = of_find_i2c_device_by_node(p_np);
		if (!i2c) {
			dev_err(bcl_dev->device, "Cannot find main-power I2C\n");
			return -ENODEV;
		}
		main_dev = i2c_get_clientdata(i2c);
	}
	of_node_put(p_np);
	if (!main_dev) {
		dev_err(bcl_dev->device, "Main PMIC device not found\n");
		return -ENODEV;
	}
	pdata_main = dev_get_platdata(main_dev->dev);
	bcl_dev->main_odpm = pdata_main->meter;
	/* request smpl_warn interrupt */
	if (!gpio_is_valid(pdata_main->smpl_warn_pin)) {
		dev_err(bcl_dev->device, "smpl_warn GPIO NOT VALID\n");
		devm_free_irq(bcl_dev->device, bcl_dev->bcl_irq[SMPL_WARN], bcl_dev);
		bypass_smpl_warn = true;
	}
	bcl_dev->main_pmic_i2c = main_dev->pmic;
	bcl_dev->main_dev = main_dev->dev;
#if IS_ENABLED(CONFIG_SOC_GS201)
	/* clear S2MPG_12 information every boot */
	/* see b/215371539 */
	S2MPG1X_READ(MAIN, bcl_dev, ret, S2MPG12_PM_OFFSRC1, &val);
	pr_info("S2MPG12 OFFSRC1 : %#x\n", val);
	bcl_dev->offsrc = val;
	S2MPG1X_READ(MAIN, bcl_dev, ret, S2MPG12_PM_OFFSRC2, &val);
	pr_info("S2MPG12 OFFSRC2 : %#x\n", val);
	S2MPG1X_READ(MAIN, bcl_dev, ret, S2MPG12_PM_PWRONSRC, &val);
	pr_info("S2MPG12 PWRONSRC: %#x\n", val);
	bcl_dev->pwronsrc = val;
	S2MPG1X_WRITE(MAIN, bcl_dev, ret, S2MPG12_PM_OFFSRC1, 0);
	S2MPG1X_WRITE(MAIN, bcl_dev, ret, S2MPG12_PM_OFFSRC2, 0);
	S2MPG1X_WRITE(MAIN, bcl_dev, ret, S2MPG12_PM_PWRONSRC, 0);
#else
	bcl_dev->main_pmic_i2c = main_dev->pmic;
	/* clear S2MPG_10 information every boot */
	/* see b/166671802#comment34 and b/195455000 */
	S2MPG1X_READ(MAIN, bcl_dev, ret, S2MPG10_PM_OFFSRC, &val);
	pr_info("S2MPG10 OFFSRC : %#x\n", val);
	bcl_dev->offsrc = val;
	S2MPG1X_READ(MAIN, bcl_dev, ret, S2MPG10_PM_PWRONSRC, &val);
	pr_info("S2MPG10 PWRONSRC: %#x\n", val);
	bcl_dev->pwronsrc = val;
	S2MPG1X_WRITE(MAIN, bcl_dev, ret, S2MPG10_PM_OFFSRC, 0);
	S2MPG1X_WRITE(MAIN, bcl_dev, ret, S2MPG10_PM_PWRONSRC, 0);
#endif
	bcl_dev->bcl_irq[SMPL_WARN] = gpio_to_irq(pdata_main->smpl_warn_pin);
	irq_set_status_flags(bcl_dev->bcl_irq[SMPL_WARN], IRQ_DISABLE_UNLAZY);
	bcl_dev->bcl_pin[SMPL_WARN] = pdata_main->smpl_warn_pin;
	bcl_dev->bcl_lvl[SMPL_WARN] = SMPL_BATTERY_VOLTAGE -
			(pdata_main->smpl_warn_lvl * SMPL_STEP + SMPL_LOWER_LIMIT);
	bcl_dev->bcl_lvl[OCP_WARN_CPUCL1] = B3M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b3_ocp_warn_lvl * B3M_STEP);
	bcl_dev->bcl_lvl[SOFT_OCP_WARN_CPUCL1] = B3M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b3_soft_ocp_warn_lvl * B3M_STEP);
	bcl_dev->bcl_lvl[OCP_WARN_CPUCL2] = B2M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b2_ocp_warn_lvl * B2M_STEP);
	bcl_dev->bcl_lvl[SOFT_OCP_WARN_CPUCL2] = B2M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b2_soft_ocp_warn_lvl * B2M_STEP);
	bcl_dev->bcl_lvl[OCP_WARN_TPU] = B10M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b10_ocp_warn_lvl * B10M_STEP);
	bcl_dev->bcl_lvl[SOFT_OCP_WARN_TPU] = B10M_UPPER_LIMIT -
			THERMAL_HYST_LEVEL - (pdata_main->b10_soft_ocp_warn_lvl * B10M_STEP);
	bcl_dev->bcl_pin[OCP_WARN_CPUCL1] = pdata_main->b3_ocp_warn_pin;
	bcl_dev->bcl_pin[OCP_WARN_CPUCL2] = pdata_main->b2_ocp_warn_pin;
	bcl_dev->bcl_pin[SOFT_OCP_WARN_CPUCL1] = pdata_main->b3_soft_ocp_warn_pin;
	bcl_dev->bcl_pin[SOFT_OCP_WARN_CPUCL2] = pdata_main->b2_soft_ocp_warn_pin;
	bcl_dev->bcl_pin[OCP_WARN_TPU] = pdata_main->b10_ocp_warn_pin;
	bcl_dev->bcl_pin[SOFT_OCP_WARN_TPU] = pdata_main->b10_soft_ocp_warn_pin;
	bcl_dev->bcl_irq[OCP_WARN_CPUCL1] = gpio_to_irq(pdata_main->b3_ocp_warn_pin);
	bcl_dev->bcl_irq[OCP_WARN_CPUCL2] = gpio_to_irq(pdata_main->b2_ocp_warn_pin);
	bcl_dev->bcl_irq[SOFT_OCP_WARN_CPUCL1] = gpio_to_irq(pdata_main->b3_soft_ocp_warn_pin);
	bcl_dev->bcl_irq[SOFT_OCP_WARN_CPUCL2] = gpio_to_irq(pdata_main->b2_soft_ocp_warn_pin);
	bcl_dev->bcl_irq[OCP_WARN_TPU] = gpio_to_irq(pdata_main->b10_ocp_warn_pin);
	bcl_dev->bcl_irq[SOFT_OCP_WARN_TPU] = gpio_to_irq(pdata_main->b10_soft_ocp_warn_pin);
	bcl_dev->bcl_ops[SMPL_WARN].get_temp = smpl_warn_read_voltage;
	bcl_dev->bcl_ops[OCP_WARN_CPUCL1].get_temp = ocp_cpu1_read_current;
	bcl_dev->bcl_ops[OCP_WARN_CPUCL2].get_temp = ocp_cpu2_read_current;
	bcl_dev->bcl_ops[SOFT_OCP_WARN_CPUCL1].get_temp = soft_ocp_cpu1_read_current;
	bcl_dev->bcl_ops[SOFT_OCP_WARN_CPUCL2].get_temp = soft_ocp_cpu2_read_current;
	bcl_dev->bcl_ops[OCP_WARN_TPU].get_temp = ocp_tpu_read_current;
	bcl_dev->bcl_ops[SOFT_OCP_WARN_TPU].get_temp = soft_ocp_tpu_read_current;
	if (!bypass_smpl_warn) {
		ret = google_bcl_register_irq(bcl_dev, SMPL_WARN, "SMPL_WARN_IRQ",
					      IRQF_TRIGGER_FALLING);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: SMPL_WARN\n");
			return -ENODEV;
		}
	}
	ret = google_bcl_register_irq(bcl_dev, OCP_WARN_CPUCL1, "CPU1_OCP_IRQ",
				      IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, OCP_WARN_CPUCL2, "CPU2_OCP_IRQ",
				      IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, SOFT_OCP_WARN_CPUCL1, "SOFT_CPU1_OCP_IRQ",
				      IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL1\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, SOFT_OCP_WARN_CPUCL2, "SOFT_CPU2_OCP_IRQ",
				      IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL2\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, OCP_WARN_TPU, "TPU_OCP_IRQ", IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: TPU\n");
		return -ENODEV;
	}
	ret = google_bcl_register_irq(bcl_dev, SOFT_OCP_WARN_TPU, "SOFT_TPU_OCP_IRQ",
				      IRQF_TRIGGER_RISING);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_TPU\n");
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
	&vdroop_flt_group,
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

static int google_bcl_init_instruction(struct bcl_device *bcl_dev)
{
	unsigned int reg;

	if (!bcl_dev)
		return -EIO;

	bcl_dev->base_mem[CPU0] = devm_ioremap(bcl_dev->device, CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU0]) {
		dev_err(bcl_dev->device, "cpu0_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->base_mem[CPU1] = devm_ioremap(bcl_dev->device, CPUCL1_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU1]) {
		dev_err(bcl_dev->device, "cpu1_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->base_mem[CPU2] = devm_ioremap(bcl_dev->device, CPUCL2_BASE, SZ_8K);
	if (!bcl_dev->base_mem[CPU2]) {
		dev_err(bcl_dev->device, "cpu2_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->base_mem[TPU] = devm_ioremap(bcl_dev->device, TPU_BASE, SZ_8K);
	if (!bcl_dev->base_mem[TPU]) {
		dev_err(bcl_dev->device, "tpu_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->base_mem[GPU] = devm_ioremap(bcl_dev->device, G3D_BASE, SZ_8K);
	if (!bcl_dev->base_mem[GPU]) {
		dev_err(bcl_dev->device, "gpu_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->sysreg_cpucl0 = devm_ioremap(bcl_dev->device, SYSREG_CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->sysreg_cpucl0) {
		dev_err(bcl_dev->device, "sysreg_cpucl0 ioremap failed\n");
		return -EIO;
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

	return 0;
}

static void google_bcl_parse_dtree(struct bcl_device *bcl_dev)
{
	int ret;
	struct device_node *np = bcl_dev->device->of_node;
	u32 val;

	if (!bcl_dev) {
		dev_err(bcl_dev->device, "Cannot parse device tree\n");
		return;
	}
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
	ret = of_property_read_u32(np, "cpu2_clkdivstep", &val);
	bcl_dev->cpu2_clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu1_clkdivstep", &val);
	bcl_dev->cpu1_clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu0_clkdivstep", &val);
	bcl_dev->cpu0_clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "odpm_ratio", &val);
	bcl_dev->odpm_ratio = (ret || !val) ? 2 : val;
	bcl_dev->vdroop1_pin = of_get_gpio(np, 0);
	bcl_dev->vdroop2_pin = of_get_gpio(np, 1);
	if (google_bcl_init_clk_div(bcl_dev, CPU2, bcl_dev->cpu2_clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU2 Address is NULL\n");
	if (google_bcl_init_clk_div(bcl_dev, CPU1, bcl_dev->cpu1_clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU1 Address is NULL\n");
	if (google_bcl_init_clk_div(bcl_dev, CPU0, bcl_dev->cpu0_clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU0 Address is NULL\n");
}

static int google_bcl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bcl_device *bcl_dev;

	bcl_dev = devm_kzalloc(&pdev->dev, sizeof(*bcl_dev), GFP_KERNEL);
	if (!bcl_dev)
		return -ENOMEM;
	bcl_dev->device = &pdev->dev;

	INIT_DELAYED_WORK(&bcl_dev->init_work, google_set_intf_pmic_work);
	platform_set_drvdata(pdev, bcl_dev);

	bcl_dev->ready = false;
	ret = google_bcl_init_instruction(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	google_set_throttling(bcl_dev);
	google_set_main_pmic(bcl_dev);
	google_set_sub_pmic(bcl_dev);
	google_set_intf_pmic(bcl_dev);
	google_bcl_parse_dtree(bcl_dev);

	ret = google_init_fs(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;
	schedule_delayed_work(&bcl_dev->init_work, msecs_to_jiffies(THERMAL_DELAY_INIT_MS));
	bcl_dev->enabled = true;
	google_init_debugfs(bcl_dev);

	return 0;

bcl_soc_probe_exit:
	google_bcl_remove_thermal(bcl_dev);
	return ret;
}

static int google_bcl_remove(struct platform_device *pdev)
{
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	pmic_device_destroy(bcl_dev->mitigation_dev->devt);
	debugfs_remove_recursive(bcl_dev->debug_entry);
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

MODULE_SOFTDEP("pre: i2c-acpm");
MODULE_DESCRIPTION("Google Battery Current Limiter");
MODULE_AUTHOR("George Lee <geolee@google.com>");
MODULE_LICENSE("GPL");
