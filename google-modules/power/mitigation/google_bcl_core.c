// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_core.c Google bcl core driver
 *
 * Copyright (c) 2022, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
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
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
#include <dt-bindings/interrupt-controller/zuma.h>
#include <soc/google/odpm.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
#include <dt-bindings/interrupt-controller/gs101.h>
#include <soc/google/odpm-whi.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
#include <dt-bindings/interrupt-controller/gs201.h>
#include <soc/google/odpm-whi.h>
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg13.h>
#endif
#include <linux/regulator/pmic_class.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include "bcl.h"
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#include <max77759_regs.h>
#include <max77779.h>
#include <max77779_fg.h>
#include <max777x9_bcl.h>

static const struct platform_device_id google_id_table[] = {
	{.name = "google_mitigation",},
	{},
};

static const unsigned int xclkout_source[] = {
	XCLKOUT_SOURCE_CPU0,
	XCLKOUT_SOURCE_CPU1,
	XCLKOUT_SOURCE_CPU2,
	XCLKOUT_SOURCE_TPU,
	XCLKOUT_SOURCE_GPU
};

void update_irq_start_times(struct bcl_device *bcl_dev, int id);
void update_irq_end_times(struct bcl_device *bcl_dev, int id);
void pwrwarn_update_start_time(struct bcl_device *bcl_dev,
				int id, struct irq_duration_stats *bins,
				bool *pwr_warn_triggered,
				enum CONCURRENT_PWRWARN_IRQ bin_ind);
void pwrwarn_update_end_time(struct bcl_device *bcl_dev, int id,
				struct irq_duration_stats *bins,
				enum CONCURRENT_PWRWARN_IRQ bin_ind);
void trace_bcl_zone_stats(struct bcl_zone *zone, int value);

static struct power_supply *google_get_power_supply(struct bcl_device *bcl_dev, const char *str)
{
	static struct power_supply *psy[2];
	static struct power_supply *batt_psy;
	int err = 0;

	batt_psy = NULL;
	err = power_supply_get_by_phandle_array(bcl_dev->device->of_node, str,
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

static void update_tz(struct bcl_zone *zone, int idx, bool triggered)
{
	struct bcl_device *bcl_dev;

	bcl_dev = zone->parent;
	if (bcl_dev->ifpmic == MAX77779)
		return;
	if (triggered)
		zone->bcl_cur_lvl = zone->bcl_lvl + THERMAL_HYST_LEVEL;
	else
		zone->bcl_cur_lvl = 0;
	if (zone->tz && (zone->bcl_prev_lvl != zone->bcl_cur_lvl))
		thermal_zone_device_update(zone->tz, THERMAL_EVENT_UNSPECIFIED);
}

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static int evt_cnt_rd_and_clr(struct bcl_device *bcl_dev, int idx, bool update_evt_cnt)
{
	int ret;
	u8 reg, val;

	switch (idx) {
	case UVLO1:
		reg = MAX77779_PMIC_EVENT_CNT_UVLO0;
		break;
	case UVLO2:
		reg = MAX77779_PMIC_EVENT_CNT_UVLO1;
		break;
	case BATOILO1:
		reg = MAX77779_PMIC_EVENT_CNT_OILO0;
		break;
	case BATOILO2:
		reg = MAX77779_PMIC_EVENT_CNT_OILO1;
		break;
	}

	/* Read to clear register */
	ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_dev, reg, &val);
	if (ret < 0) {
		dev_err(bcl_dev->device, "evt_cnt_rd_and_clr: %d, fail\n", reg);
		return -ENODEV;
	}
	switch (idx) {
	case UVLO1:
		bcl_dev->evt_cnt_latest.uvlo1 = val;
		if (update_evt_cnt)
			bcl_dev->evt_cnt.uvlo1 = val;
		break;
	case UVLO2:
		bcl_dev->evt_cnt_latest.uvlo2 = val;
		if (update_evt_cnt)
			bcl_dev->evt_cnt.uvlo2 = val;
		break;
	case BATOILO1:
		bcl_dev->evt_cnt_latest.batoilo1 = val;
		if (update_evt_cnt)
			bcl_dev->evt_cnt.batoilo1 = val;
		break;
	case BATOILO2:
		bcl_dev->evt_cnt_latest.batoilo2 = val;
		if (update_evt_cnt)
			bcl_dev->evt_cnt.batoilo2 = val;
		break;
	}

	return 0;
}
#endif

static int google_bcl_wait_for_response_locked(struct bcl_zone *zone, int timeout_ms)
{
	struct bcl_device *bcl_dev = zone->parent;
	if (bcl_dev->ifpmic == MAX77759)
		return 0;
	reinit_completion(&zone->deassert);
	return wait_for_completion_timeout(&zone->deassert, msecs_to_jiffies(timeout_ms));
}

static irqreturn_t latched_irq_handler(int irq, void *data)
{
	struct bcl_zone *zone = data;
	struct bcl_device *bcl_dev;
	u8 idx;

	if (!zone || !zone->parent)
		return IRQ_HANDLED;

	idx = zone->idx;
	bcl_dev = zone->parent;
	if (!smp_load_acquire(&bcl_dev->enabled)) {
		if (zone->irq_type == IF_PMIC)
			bcl_cb_clr_irq(bcl_dev, idx);
		return IRQ_HANDLED;
	}
	queue_work(system_unbound_wq, &zone->irq_triggered_work);
	return IRQ_HANDLED;
}

static bool google_warn_check(struct bcl_zone *zone)
{
	struct bcl_device *bcl_dev;
	int gpio_level, assert = 0, ret;
	u8 regval;

	bcl_dev = zone->parent;
	if (zone->bcl_pin != NOT_USED) {
		gpio_level = gpio_get_value(zone->bcl_pin);
		return (gpio_level == zone->polarity);
	}
	if (bcl_dev->ifpmic == MAX77779) {
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_CHG_DETAILS_01, &regval);
		if (ret < 0) {
			dev_err(bcl_dev->device, "IRQ read: %d, fail\n", regval);
			return false;
		}
		assert = _max77779_chg_details_01_bat_dtls_get(regval);
		if (assert == BAT_DTLS_OILO_ASSERTED)
			return true;
	}
	return false;
}

static void google_bcl_release_throttling(struct bcl_zone *zone)
{
	struct bcl_device *bcl_dev;

	bcl_dev = zone->parent;
	zone->bcl_cur_lvl = 0;

	if (zone->bcl_qos)
		google_bcl_qos_update(zone, false);
	else if (zone->idx == BATOILO2 && bcl_dev->zone[BATOILO])
		google_bcl_qos_update(bcl_dev->zone[BATOILO], false);

	complete(&zone->deassert);
	trace_bcl_zone_stats(zone, 0);

	if (zone->irq_type == IF_PMIC) {
		bcl_cb_clr_irq(bcl_dev, zone->idx);
		update_irq_end_times(bcl_dev, zone->idx);
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
		if (zone->idx >= UVLO1 && zone->idx <= BATOILO2 && bcl_dev->ifpmic == MAX77779)
			evt_cnt_rd_and_clr(bcl_dev, zone->idx, false);
#endif
	}
	if (zone->idx == BATOILO && bcl_dev->config_modem)
		gpio_set_value(bcl_dev->modem_gpio2_pin, 0);
	update_tz(zone, zone->idx, false);

	if (bcl_dev->bat_ktimer_en && zone->idx == BATOILO) {
		hrtimer_cancel(&(bcl_dev->hr_timer));
		wakeup_source_unregister(bcl_dev->ws);
	}
}

static void google_warn_work(struct work_struct *work)
{
	struct bcl_zone *zone = container_of(work, struct bcl_zone, warn_work.work);
	struct bcl_device *bcl_dev;

	bcl_dev = zone->parent;
	if (!google_warn_check(zone)) {
		google_bcl_upstream_state(zone, DISABLED);
		google_bcl_release_throttling(zone);
	} else {
		zone->bcl_cur_lvl = zone->bcl_lvl + THERMAL_HYST_LEVEL;
		/* ODPM Read to kick off LIGHT module throttling */
		mod_delayed_work(bcl_dev->qos_update_wq, &zone->warn_work,
				 msecs_to_jiffies(TIMEOUT_5MS));
	}
	if (zone->tz)
		thermal_zone_device_update(zone->tz, THERMAL_EVENT_UNSPECIFIED);
}

static int google_bcl_set_soc(struct bcl_device *bcl_dev, int low, int high)
{
	if (IS_ERR_OR_NULL(bcl_dev) || IS_ERR_OR_NULL(bcl_dev->device))
		return 0;
	if (high == bcl_dev->trip_high_temp)
		return 0;

	bcl_dev->trip_low_temp = low;
	bcl_dev->trip_high_temp = high;
	schedule_delayed_work(&bcl_dev->soc_work, 0);

	return 0;
}

static int tz_bcl_set_soc(struct thermal_zone_device *tz, int low, int high)
{
	return google_bcl_set_soc(tz->devdata, low, high);
}

static int google_bcl_read_soc(struct bcl_device *bcl_dev, int *val)
{
	union power_supply_propval ret = {
		0,
	};
	int err = 0;
	*val = 100;

	if (IS_ERR_OR_NULL(bcl_dev) || IS_ERR_OR_NULL(bcl_dev->device))
		return 0;
	if (!smp_load_acquire(&bcl_dev->enabled))
		return 0;
	if (!bcl_dev->batt_psy)
		bcl_dev->batt_psy = google_get_power_supply(bcl_dev, PSY_NAME);
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

static int tz_bcl_read_soc(struct thermal_zone_device *tz, int *val)
{
	return google_bcl_read_soc(tz->devdata, val);
}

static void google_bcl_evaluate_soc(struct work_struct *work)
{
	int battery_percentage_reverse;
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  soc_work.work);

	if (google_bcl_read_soc(bcl_dev, &battery_percentage_reverse))
		return;

	if ((battery_percentage_reverse < bcl_dev->trip_high_temp) &&
		(battery_percentage_reverse > bcl_dev->trip_low_temp))
		return;

	bcl_dev->trip_val = battery_percentage_reverse;
	if (!bcl_dev->soc_tz) {
		bcl_dev->soc_tz = devm_thermal_of_zone_register(bcl_dev->device,
								PMIC_SOC, bcl_dev,
								&bcl_dev->soc_tz_ops);
		if (IS_ERR(bcl_dev->soc_tz)) {
			dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
				PTR_ERR(bcl_dev->soc_tz));
			return;
		}
	}
	if (!IS_ERR(bcl_dev->soc_tz))
		thermal_zone_device_update(bcl_dev->soc_tz, THERMAL_EVENT_UNSPECIFIED);
	return;
}

static int battery_supply_callback(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct bcl_device *bcl_dev = container_of(nb, struct bcl_device, psy_nb);
	struct power_supply *bcl_psy;

	if (IS_ERR_OR_NULL(bcl_dev))
		return NOTIFY_OK;

	bcl_psy = bcl_dev->batt_psy;

	if (!bcl_psy || event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!strcmp(psy->desc->name, bcl_psy->desc->name))
		schedule_delayed_work(&bcl_dev->soc_work, 0);

	return NOTIFY_OK;
}

/*
 * For PROBE_DEFER used.
 */
static int google_bcl_remove_thermal(struct bcl_device *bcl_dev)
{
	int i = 0;
	struct bcl_zone *zone;

	if (IS_ERR_OR_NULL(bcl_dev))
		return 0;
        if (bcl_dev->batt_psy_initialized)
		power_supply_unreg_notifier(&bcl_dev->psy_nb);
	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (!bcl_dev->zone[i])
			continue;
		zone = bcl_dev->zone[i];
		if (zone->irq_reg) {
			if ((bcl_dev->ifpmic == MAX77779) && (i == BATOILO))
				devm_free_irq(bcl_dev->device, bcl_dev->pmic_irq, bcl_dev);
			else
				devm_free_irq(bcl_dev->device, zone->bcl_irq, zone);
		}
		zone->irq_reg = false;
		if (zone->tz)
			devm_thermal_of_zone_unregister(bcl_dev->device, zone->tz);
		if (zone->irq_triggered_work.func != NULL)
			cancel_work_sync(&zone->irq_triggered_work);
		if (zone->warn_work.work.func != NULL)
			cancel_delayed_work_sync(&zone->warn_work);
		devm_kfree(bcl_dev->device, zone);
	}
	if (bcl_dev->setup_sub_odpm_work.work.func != NULL)
		cancel_delayed_work_sync(&bcl_dev->setup_sub_odpm_work);

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	if (bcl_dev->main_pwr_irq_work.work.func != NULL)
		cancel_delayed_work_sync(&bcl_dev->main_pwr_irq_work);
	if (bcl_dev->sub_pwr_irq_work.work.func != NULL)
		cancel_delayed_work_sync(&bcl_dev->sub_pwr_irq_work);
#endif
	google_bcl_remove_qos(bcl_dev);
	destroy_workqueue(bcl_dev->qos_update_wq);
	if (bcl_dev->soc_work.work.func != NULL)
		cancel_delayed_work_sync(&bcl_dev->soc_work);

	cpu_pm_unregister_notifier(&bcl_dev->cpu_nb);
	if (bcl_dev->non_monitored_module_ids != NULL)
		kfree(bcl_dev->non_monitored_module_ids);
	if (bcl_dev->rd_last_curr_work.work.func != NULL)
		cancel_delayed_work_sync(&bcl_dev->rd_last_curr_work);
	google_bcl_remove_data_logging(bcl_dev);
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	google_bcl_remove_votable(bcl_dev);
#endif
	mutex_destroy(&bcl_dev->cpu_ratio_lock);
	mutex_destroy(&bcl_dev->sysreg_lock);

	return 0;
}

static int google_bcl_init_clk_div(struct bcl_device *bcl_dev, int idx,
				   unsigned int value)
{
	if (IS_ERR_OR_NULL(bcl_dev))
		return -EIO;
	switch (idx) {
	case SUBSYSTEM_TPU:
	case SUBSYSTEM_GPU:
	case SUBSYSTEM_AUR:
		return -EIO;
	case SUBSYSTEM_CPU0:
	case SUBSYSTEM_CPU1:
	case SUBSYSTEM_CPU2:
		return cpu_buff_write(bcl_dev, idx, CPU_BUFF_CLKDIVSTEP, value);
	}
	return -EINVAL;
}

struct bcl_device *google_retrieve_bcl_handle(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct bcl_device *bcl_dev;

	np = of_find_node_by_name(NULL, "google,mitigation");
	if (!np || !virt_addr_valid(np) || !of_device_is_available(np))
		return NULL;
	pdev = of_find_device_by_node(np);
	if (!pdev)
		return NULL;
	bcl_dev = platform_get_drvdata(pdev);
	if (IS_ERR_OR_NULL(bcl_dev))
		return NULL;

	return bcl_dev;
}
EXPORT_SYMBOL_GPL(google_retrieve_bcl_handle);

static int google_init_ratio(struct bcl_device *data, enum SUBSYSTEM_SOURCE idx)
{
	void __iomem *addr;

	if (IS_ERR_OR_NULL(data->device))
		return -EIO;

	if (!smp_load_acquire(&data->enabled))
		return -EINVAL;

	if (!bcl_is_subsystem_on(data, subsystem_pmu[idx]))
		return -EIO;

	if (idx < SUBSYSTEM_TPU)
		return -EIO;

	if (idx != SUBSYSTEM_AUR) {
		addr = data->core_conf[idx].base_mem + CLKDIVSTEP_CON_HEAVY;
		__raw_writel(data->core_conf[idx].con_heavy, addr);
		addr = data->core_conf[idx].base_mem + CLKDIVSTEP_CON_LIGHT;
		__raw_writel(data->core_conf[idx].con_light, addr);
		addr = data->core_conf[idx].base_mem + VDROOP_FLT;
		__raw_writel(data->core_conf[idx].vdroop_flt, addr);
	}
	addr = data->core_conf[idx].base_mem + CLKDIVSTEP;
	__raw_writel(data->core_conf[idx].clkdivstep, addr);
	addr = data->core_conf[idx].base_mem + CLKOUT;
	__raw_writel(data->core_conf[idx].clk_out, addr);
	data->core_conf[idx].clk_stats = __raw_readl(data->core_conf[idx].base_mem +
						     clk_stats_offset[idx]);

	return 0;
}

int google_init_tpu_ratio(struct bcl_device *data)
{
	if (!IS_ERR_OR_NULL(data))
		return google_init_ratio(data, SUBSYSTEM_TPU);
	return 0;
}
EXPORT_SYMBOL_GPL(google_init_tpu_ratio);

int google_init_gpu_ratio(struct bcl_device *data)
{
	if (!IS_ERR_OR_NULL(data))
		return google_init_ratio(data, SUBSYSTEM_GPU);
	return 0;
}
EXPORT_SYMBOL_GPL(google_init_gpu_ratio);

int google_init_aur_ratio(struct bcl_device *data)
{
	if (!IS_ERR_OR_NULL(data))
		return google_init_ratio(data, SUBSYSTEM_AUR);
	return 0;
}
EXPORT_SYMBOL_GPL(google_init_aur_ratio);

unsigned int google_get_db(struct bcl_device *data, enum MPMM_SOURCE index)
{
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	void __iomem *addr;
	unsigned int reg;

	if (IS_ERR_OR_NULL(data))
		return -ENOMEM;
	if (!smp_load_acquire(&data->enabled))
		return -EINVAL;
	if (!data->sysreg_cpucl0) {
		dev_err(data->device, "Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	if (index == MID)
		addr = data->sysreg_cpucl0 + CLUSTER0_MID_DISPBLOCK;
	else if (index == BIG)
		addr = data->sysreg_cpucl0 + CLUSTER0_BIG_DISPBLOCK;
	else
		return -EINVAL;

	mutex_lock(&data->sysreg_lock);
	reg = __raw_readl(addr);
	mutex_unlock(&data->sysreg_lock);

	return reg;
#endif
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(google_get_db);

int google_set_db(struct bcl_device *data, unsigned int value, enum MPMM_SOURCE index)
{
	void __iomem *addr;

	if (IS_ERR_OR_NULL(data))
		return -ENOMEM;
	if (!smp_load_acquire(&data->enabled))
		return -EINVAL;
	if (!data->sysreg_cpucl0) {
		dev_err(data->device, "Error in sysreg_cpucl0\n");
		return -ENOMEM;
	}

	if (index == MID)
		addr = data->sysreg_cpucl0 + CLUSTER0_MID_DISPBLOCK;
	else if (index == BIG)
		addr = data->sysreg_cpucl0 + CLUSTER0_BIG_DISPBLOCK;
	else
		return -EINVAL;

	mutex_lock(&data->sysreg_lock);
	__raw_writel(value, addr);
	mutex_unlock(&data->sysreg_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(google_set_db);

int google_pwr_loop_trigger_mitigation(struct bcl_device *bcl_dev)
{
	/* TODO: b/356694140 - implement power reduction */
	return 0;
}

static void google_irq_triggered_work(struct work_struct *work)
{
	struct bcl_zone *zone = container_of(work, struct bcl_zone, irq_triggered_work);
	struct bcl_device *bcl_dev;
	u8 irq_val = 0;
	int idx;

	idx = zone->idx;
	bcl_dev = zone->parent;

	google_bcl_upstream_state(zone, START);

	if (zone->bcl_pin != NOT_USED) {
		if (bcl_dev->ifpmic == MAX77759 && idx >= UVLO2 && idx <= BATOILO2) {
			bcl_cb_get_irq(bcl_dev, &irq_val);
			if (irq_val == 0)
				return;
			idx = irq_val;
			zone = bcl_dev->zone[idx];
		}
		if (zone->irq_type == IF_PMIC)
			bcl_cb_clr_irq(bcl_dev, idx);
		if (gpio_get_value(zone->bcl_pin) == zone->polarity) {
			if (idx >= UVLO1 && idx <= BATOILO2) {
				atomic_inc(&zone->last_triggered.triggered_cnt[START]);
				zone->last_triggered.triggered_time[START] =
						ktime_to_ms(ktime_get());
			}
		} else {
			google_bcl_release_throttling(zone);
			return;
		}
	}

	if (zone->bcl_qos) {
		google_bcl_qos_update(zone, true);
		mod_delayed_work(bcl_dev->qos_update_wq, &zone->warn_work,
				 msecs_to_jiffies(TIMEOUT_5MS));
	}

	idx = zone->idx;
	bcl_dev = zone->parent;

	google_bcl_start_data_logging(bcl_dev, idx);

	/* LIGHT phase */
	google_bcl_upstream_state(zone, LIGHT);

	if (bcl_dev->batt_psy_initialized) {
		atomic_inc(&zone->bcl_cnt);
		ocpsmpl_read_stats(bcl_dev, &zone->bcl_stats, bcl_dev->batt_psy);
		update_tz(zone, idx, true);
	}

	trace_bcl_zone_stats(zone, 1);

	if (zone->irq_type == IF_PMIC) {
		update_irq_start_times(bcl_dev, idx);
	}

	if (idx == BATOILO && bcl_dev->config_modem)
		gpio_set_value(bcl_dev->modem_gpio2_pin, 1);

	if (google_bcl_wait_for_response_locked(zone, TIMEOUT_5MS) > 0)
		return;
	google_bcl_upstream_state(zone, MEDIUM);

	/* MEDIUM phase: b/300504518 */
	if (google_bcl_wait_for_response_locked(zone, TIMEOUT_5MS) > 0)
		return;
	google_bcl_upstream_state(zone, HEAVY);
	/* We most likely have to shutdown after this */

	/* Reset Mitigation module if we are still alive */
	atomic_set(&bcl_dev->mitigation_module_ids, 0);

	/* HEAVY phase */
	/* IRQ deasserted */
}

static irqreturn_t vdroop_irq_thread_fn(int irq, void *data)
{
	struct bcl_device *bcl_dev = data;
	struct bcl_zone *zone = NULL;

	if (IS_ERR_OR_NULL(bcl_dev))
		return IRQ_HANDLED;
	bcl_cb_clr_irq(bcl_dev, BATOILO);
	if (!smp_load_acquire(&bcl_dev->enabled))
		return IRQ_HANDLED;

	/* This is only BATOILO */
	zone = bcl_dev->zone[BATOILO];
	if (zone) {
		if (bcl_dev->bat_ktimer_en) {
			bcl_dev->ws = wakeup_source_register(NULL, "bcl_overcurrent_wake");
			hrtimer_start(&(bcl_dev->hr_timer),
				      ktime_set(bcl_dev->bat_ktimer / 1000,
						(bcl_dev->bat_ktimer % 1000) *
							1000000),
				      HRTIMER_MODE_REL);
		}
		atomic_inc(&zone->last_triggered.triggered_cnt[START]);
		zone->last_triggered.triggered_time[START] =
			ktime_to_ms(ktime_get());
		queue_work(system_unbound_wq, &zone->irq_triggered_work);
	}

	return IRQ_HANDLED;
}

static int google_bcl_register_zone(struct bcl_device *bcl_dev, int idx, const char *devname,
				    int pin, int lvl, int irq, int type)
{
	int ret = 0;
	struct bcl_zone *zone;
	bool to_conf = true;
	u32 default_intr_flag, latched_intr_flag;

	zone = devm_kzalloc(bcl_dev->device, sizeof(struct bcl_zone), GFP_KERNEL);

	if (!zone)
		return -ENOMEM;

	default_intr_flag = IRQF_ONESHOT | IRQF_NO_AUTOEN;
	init_completion(&zone->deassert);
	zone->idx = idx;
	zone->bcl_pin = pin;
	zone->bcl_irq = irq;
	zone->bcl_cur_lvl = 0;
	zone->bcl_prev_lvl = 0;
	zone->bcl_lvl = lvl;
	zone->parent = bcl_dev;
	zone->irq_type = type;
	zone->devname = devname;
	zone->disabled = true;
	zone->device = bcl_dev->device;
	atomic_set(&zone->bcl_cnt, 0);
	atomic_set(&zone->last_triggered.triggered_cnt[START], 0);
	atomic_set(&zone->last_triggered.triggered_cnt[LIGHT], 0);
	atomic_set(&zone->last_triggered.triggered_cnt[MEDIUM], 0);
	atomic_set(&zone->last_triggered.triggered_cnt[HEAVY], 0);
	INIT_WORK(&zone->irq_triggered_work, google_irq_triggered_work);
	INIT_DELAYED_WORK(&zone->warn_work, google_warn_work);
	if (idx == SMPL_WARN) {
		latched_intr_flag = IRQF_TRIGGER_FALLING;
		zone->polarity = 0;
	} else {
		latched_intr_flag = IRQF_TRIGGER_RISING;
		zone->polarity = 1;
	}
	latched_intr_flag = latched_intr_flag | default_intr_flag;
	if ((bcl_dev->ifpmic == MAX77779) && (idx == BATOILO)) {
		zone->bcl_pin = NOT_USED;
		if (!zone->irq_reg) {
			ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->pmic_irq, NULL,
							vdroop_irq_thread_fn,
							IRQF_TRIGGER_FALLING | IRQF_SHARED |
							IRQF_ONESHOT | IRQF_NO_THREAD, devname,
							bcl_dev);
			if (ret < 0) {
				dev_err(zone->device, "Failed to request l-IRQ: %d: %d\n",
					irq, ret);
				devm_kfree(bcl_dev->device, zone);
				return ret;
			}
			zone->bcl_irq = bcl_dev->pmic_irq;
			zone->irq_reg = true;
			zone->disabled = false;
			to_conf = false;
		}
	}
	if ((bcl_dev->ifpmic == MAX77759) && (idx == BATOILO))
		to_conf = false;
	if (to_conf) {
		ret = devm_request_threaded_irq(bcl_dev->device, zone->bcl_irq, NULL,
						latched_irq_handler,
						latched_intr_flag, devname, zone);

		if (ret < 0) {
			dev_err(zone->device, "Failed to request IRQ: %d: %d\n", irq, ret);
			devm_kfree(bcl_dev->device, zone);
			return ret;
		}
		zone->irq_reg = true;
	}
	bcl_dev->zone[idx] = zone;
	return ret;
}

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
static void main_pwrwarn_irq_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  main_pwr_irq_work.work);
	bool revisit_needed = false;
	int i;
	u32 micro_unit[ODPM_CHANNEL_MAX];
	u32 measurement;

	mutex_lock(&bcl_dev->main_odpm->lock);

	odpm_get_raw_lpf_values(bcl_dev->main_odpm, S2MPG1415_METER_CURRENT, micro_unit);
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		measurement = micro_unit[i] >> LPF_CURRENT_SHIFT;
		bcl_dev->main_pwr_warn_triggered[i] = (measurement > bcl_dev->main_setting[i]);
		if (!revisit_needed)
			revisit_needed = bcl_dev->main_pwr_warn_triggered[i];
		if ((!revisit_needed) && (i == bcl_dev->rffe_channel) && bcl_dev->config_modem)
			gpio_set_value(bcl_dev->modem_gpio1_pin, 0);
		if (!bcl_dev->main_pwr_warn_triggered[i])
			pwrwarn_update_end_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
						RFFE_BCL_BIN);
		else
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
							bcl_dev->main_pwr_warn_triggered,
							RFFE_BCL_BIN);
	}

	mutex_unlock(&bcl_dev->main_odpm->lock);

	if (revisit_needed)
		mod_delayed_work(system_unbound_wq, &bcl_dev->main_pwr_irq_work,
				 msecs_to_jiffies(PWRWARN_DELAY_MS));
}
#endif

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
static void sub_pwrwarn_irq_work(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  sub_pwr_irq_work.work);
	bool revisit_needed = false;
	int i;
	u32 micro_unit[ODPM_CHANNEL_MAX];
	u32 measurement;

	mutex_lock(&bcl_dev->sub_odpm->lock);

	odpm_get_raw_lpf_values(bcl_dev->sub_odpm, S2MPG1415_METER_CURRENT, micro_unit);
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		measurement = micro_unit[i] >> LPF_CURRENT_SHIFT;
		bcl_dev->sub_pwr_warn_triggered[i] = (measurement > bcl_dev->sub_setting[i]);
		if (!revisit_needed)
			revisit_needed = bcl_dev->sub_pwr_warn_triggered[i];
		if ((!revisit_needed) && (i == bcl_dev->rffe_channel) && bcl_dev->config_modem)
			gpio_set_value(bcl_dev->modem_gpio1_pin, 0);
		if (!bcl_dev->sub_pwr_warn_triggered[i])
			pwrwarn_update_end_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
						MMWAVE_BCL_BIN);
		else
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
							bcl_dev->sub_pwr_warn_triggered,
							MMWAVE_BCL_BIN);
	}

	mutex_unlock(&bcl_dev->sub_odpm->lock);

	if (revisit_needed)
		mod_delayed_work(system_unbound_wq, &bcl_dev->sub_pwr_irq_work,
				 msecs_to_jiffies(PWRWARN_DELAY_MS));
}
#endif

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
static irqreturn_t sub_pwr_warn_irq_handler(int irq, void *data)
{
	struct bcl_device *bcl_dev = data;
	int i;

	if (!smp_load_acquire(&bcl_dev->enabled))
		return IRQ_HANDLED;

	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		if (bcl_dev->sub_pwr_warn_irq[i] == irq) {
			bcl_dev->sub_pwr_warn_triggered[i] = 1;
			/* Check for Modem MMWAVE */
			if (i == bcl_dev->rffe_channel && bcl_dev->config_modem)
				gpio_set_value(bcl_dev->modem_gpio1_pin, 1);

			/* Setup Timer to clear the triggered */
			mod_delayed_work(system_unbound_wq, &bcl_dev->sub_pwr_irq_work,
					 msecs_to_jiffies(PWRWARN_DELAY_MS));
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_sub_irq_bins,
							bcl_dev->sub_pwr_warn_triggered,
							MMWAVE_BCL_BIN);
			break;
		}
	}

	return IRQ_HANDLED;
}
#endif

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
static irqreturn_t main_pwr_warn_irq_handler(int irq, void *data)
{
	struct bcl_device *bcl_dev = data;
	int i;

	if (!smp_load_acquire(&bcl_dev->enabled))
		return IRQ_HANDLED;

	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		if (bcl_dev->main_pwr_warn_irq[i] == irq) {
			bcl_dev->main_pwr_warn_triggered[i] = 1;
			/* Check for Modem RFFE */
			if (i == bcl_dev->rffe_channel && bcl_dev->config_modem)
				gpio_set_value(bcl_dev->modem_gpio1_pin, 1);

			/* Setup Timer to clear the triggered */
			mod_delayed_work(system_unbound_wq, &bcl_dev->main_pwr_irq_work,
					 msecs_to_jiffies(PWRWARN_DELAY_MS));
			pwrwarn_update_start_time(bcl_dev, i, bcl_dev->pwrwarn_main_irq_bins,
							bcl_dev->main_pwr_warn_triggered,
							RFFE_BCL_BIN);
			break;
		}
	}

	return IRQ_HANDLED;
}
#endif

static void google_bcl_setup_main_odpm(struct work_struct *work) {
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	struct s2mpg14_platform_data *pdata;
	int read;
	struct device_node *child;
	struct device_node *p_np;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	struct s2mpg12_platform_data *pdata;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	struct s2mpg10_platform_data *pdata;
#endif
	int i, rail_i;
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device, setup_main_odpm_work.work);

	pdata = dev_get_platdata(bcl_dev->main_dev);
	bcl_dev->main_odpm = pdata->meter;
	if (!bcl_dev->main_odpm) {
		dev_err(bcl_dev->device, "Main PMIC meter device not found\n");
		schedule_delayed_work(&bcl_dev->setup_main_odpm_work, msecs_to_jiffies(TIMEOUT_5S));
		return;
	}
	if (!smp_load_acquire(&bcl_dev->main_odpm->ready)) {
		dev_err(bcl_dev->device, "Main PMIC meter not initialized\n");
		schedule_delayed_work(&bcl_dev->setup_main_odpm_work, msecs_to_jiffies(TIMEOUT_5S));
		return;
	}
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		rail_i = bcl_dev->main_odpm->channels[i].rail_i;
		if (bcl_dev->main_odpm->chip.rails == NULL) {
			dev_err(bcl_dev->device, "Main PMIC Rail:%d not initialized\n", rail_i);
			schedule_delayed_work(&bcl_dev->setup_main_odpm_work, msecs_to_jiffies(TIMEOUT_5S));
			return;
		}
		bcl_dev->main_rail_names[i] = bcl_dev->main_odpm->chip.rails[rail_i].schematic_name;
	}
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	/* parse ODPM main limit */
	p_np = of_get_child_by_name(bcl_dev->device->of_node, "main_limit");
	if (p_np) {
		i = 0;
		for_each_child_of_node(p_np, child) {
			of_property_read_u32(child, "setting", &read);
			if (i < METER_CHANNEL_MAX) {
				bcl_dev->main_setting[i] = read;
				meter_write(CORE_PMIC_MAIN, bcl_dev,
				            S2MPG14_METER_PWR_WARN0 + i, read);
				bcl_dev->main_limit[i] =
				    settings_to_current(bcl_dev, CORE_PMIC_MAIN, i,
				                        read << LPF_CURRENT_SHIFT);
				i++;
			}
		}
	}
#endif

	dev_err(bcl_dev->device, "setup Main PMIC meter done\n");
}

static void google_bcl_setup_sub_odpm(struct work_struct *work) {
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	struct s2mpg15_platform_data *pdata;
	int read;
	struct device_node *child;
	struct device_node *p_np;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	struct s2mpg13_platform_data *pdata;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	struct s2mpg11_platform_data *pdata;
#endif
	int i, rail_i;
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device, setup_sub_odpm_work.work);

	pdata = dev_get_platdata(bcl_dev->sub_dev);
	bcl_dev->sub_odpm = pdata->meter;
	if (!bcl_dev->sub_odpm) {
		dev_err(bcl_dev->device, "Sub PMIC meter device not found\n");
		schedule_delayed_work(&bcl_dev->setup_sub_odpm_work, msecs_to_jiffies(TIMEOUT_5S));
		return;
	}
	if (!smp_load_acquire(&bcl_dev->sub_odpm->ready)) {
		dev_err(bcl_dev->device, "Sub PMIC meter not initialized\n");
		schedule_delayed_work(&bcl_dev->setup_sub_odpm_work, msecs_to_jiffies(TIMEOUT_5S));
		return;
	}
	for (i = 0; i < METER_CHANNEL_MAX; i++) {
		rail_i = bcl_dev->sub_odpm->channels[i].rail_i;
		if (bcl_dev->sub_odpm->chip.rails == NULL) {
			dev_err(bcl_dev->device, "Sub PMIC Rail:%d not initialized\n", rail_i);
			schedule_delayed_work(&bcl_dev->setup_sub_odpm_work, msecs_to_jiffies(TIMEOUT_5S));
			return;
		}
		bcl_dev->sub_rail_names[i] = bcl_dev->sub_odpm->chip.rails[rail_i].schematic_name;
	}
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	/* parse ODPM sub limit */
	p_np = of_get_child_by_name(bcl_dev->device->of_node, "sub_limit");
	if (p_np) {
		i = 0;
		for_each_child_of_node(p_np, child) {
			of_property_read_u32(child, "setting", &read);
			if (i < METER_CHANNEL_MAX) {
				bcl_dev->sub_setting[i] = read;
				meter_write(CORE_PMIC_SUB, bcl_dev,
				            S2MPG15_METER_PWR_WARN0 + i, read);
				bcl_dev->sub_limit[i] =
				    settings_to_current(bcl_dev, CORE_PMIC_SUB, i,
				                        read << LPF_CURRENT_SHIFT);
				i++;
			}
		}
	}
#endif

	dev_err(bcl_dev->device, "setup Sub PMIC meter done\n");
}

static int google_bcl_register_zones_sub(struct bcl_device *bcl_dev, void *pdata_sub)
{
	int ret;
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	struct s2mpg15_platform_data *pdata = pdata_sub;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	struct s2mpg13_platform_data *pdata = pdata_sub;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	struct s2mpg11_platform_data *pdata = pdata_sub;
#endif

	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_GPU, "ocp_gpu",
				       pdata->b2_ocp_warn_pin,
				       GPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata->b2_ocp_warn_lvl * GPU_STEP),
				       gpio_to_irq(pdata->b2_ocp_warn_pin),
				       CORE_SUB_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: GPU\n");
		return -ENODEV;
	}
	if (IS_ENABLED(CONFIG_SOC_ZUMAPRO))
		return 0;
	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_GPU, "soft_ocp_gpu",
				       pdata->b2_soft_ocp_warn_pin,
				       GPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (pdata->b2_soft_ocp_warn_lvl * GPU_STEP),
				       gpio_to_irq(pdata->b2_soft_ocp_warn_pin),
				       CORE_SUB_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_GPU\n");
		return -ENODEV;
	}
	return 0;
}

static int google_set_sub_pmic(struct bcl_device *bcl_dev)
{
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	struct s2mpg15_platform_data *pdata_sub;
	struct s2mpg15_dev *sub_dev = NULL;
	int i;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	struct s2mpg13_platform_data *pdata_sub;
	struct s2mpg13_dev *sub_dev = NULL;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	struct s2mpg11_platform_data *pdata_sub;
	struct s2mpg11_dev *sub_dev = NULL;
#endif
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	u8 val = 0;
	int ret;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	INIT_DELAYED_WORK(&bcl_dev->sub_pwr_irq_work, sub_pwrwarn_irq_work);
#endif

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
	if (!pdata_sub) {
		dev_err(bcl_dev->device, "SUB platform data not found\n");
		return -ENODEV;
	}

	bcl_dev->sub_meter_i2c = sub_dev->meter;
	bcl_dev->sub_irq_base = pdata_sub->irq_base;
	bcl_dev->sub_pmic_i2c = sub_dev->pmic;
	bcl_dev->sub_dev = sub_dev->dev;
	if (pmic_read(CORE_PMIC_SUB, bcl_dev, SUB_CHIPID, &val)) {
		dev_err(bcl_dev->device, "Failed to read PMIC chipid.\n");
		return -ENODEV;
	}
	pmic_read(CORE_PMIC_SUB, bcl_dev, SUB_OFFSRC1, &val);
	dev_info(bcl_dev->device, "SUB OFFSRC1 : %#x\n", val);
	bcl_dev->sub_offsrc1 = val;
	pmic_write(CORE_PMIC_SUB, bcl_dev, SUB_OFFSRC1, 0);
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	pmic_read(CORE_PMIC_SUB, bcl_dev, SUB_OFFSRC2, &val);
	dev_info(bcl_dev->device, "SUB OFFSRC2 : %#x\n", val);
	bcl_dev->sub_offsrc2 = val;
	pmic_write(CORE_PMIC_SUB, bcl_dev, SUB_OFFSRC2, 0);
#endif

	ret = google_bcl_register_zones_sub(bcl_dev, pdata_sub);
	if (ret < 0)
		return ret;

	INIT_DELAYED_WORK(&bcl_dev->setup_sub_odpm_work, google_bcl_setup_sub_odpm);
	schedule_delayed_work(&bcl_dev->setup_sub_odpm_work, msecs_to_jiffies(TIMEOUT_5MS));

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		bcl_dev->sub_pwr_warn_irq[i] =
				bcl_dev->sub_irq_base + S2MPG15_IRQ_PWR_WARN_CH0_INT5 + i;
		ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->sub_pwr_warn_irq[i],
						NULL, sub_pwr_warn_irq_handler, 0,
						bcl_dev->sub_rail_names[i], bcl_dev);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Failed to request PWR_WARN_CH%d IRQ: %d: %d\n",
				i, bcl_dev->sub_pwr_warn_irq[i], ret);
		}
	}
#endif

	return 0;
}

static int get_idx_from_zone(struct bcl_device *bcl_dev, const char *name)
{
	int i;
	struct bcl_zone *zone;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		zone = bcl_dev->zone[i];
		if (!zone)
			continue;
		if (!strcmp(name, zone->devname))
			return i;
	}
	return -EINVAL;
}

static void google_bcl_parse_qos(struct bcl_device *bcl_dev)
{
	struct device_node *np = bcl_dev->device->of_node;
	struct device_node *child;
	struct device_node *p_np;
	int idx;

	/* parse qos */
	p_np = of_get_child_by_name(np, "freq_qos");
	if (!p_np)
		return;
	for_each_child_of_node(p_np, child) {
		idx = get_idx_from_zone(bcl_dev, child->name);
		if (idx < 0)
			continue;
		bcl_dev->zone[idx]->bcl_qos = devm_kzalloc(bcl_dev->device,
		                                           sizeof(struct qos_throttle_limit),
		                                           GFP_KERNEL);
		if (of_property_read_u32(child, "cpucl0",
					 &bcl_dev->zone[idx]->bcl_qos->cpu0_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->cpu0_limit = INT_MAX;
		if (of_property_read_u32(child, "cpucl1",
					 &bcl_dev->zone[idx]->bcl_qos->cpu1_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->cpu1_limit = INT_MAX;
		if (of_property_read_u32(child, "cpucl2",
					 &bcl_dev->zone[idx]->bcl_qos->cpu2_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->cpu2_limit = INT_MAX;
		if (of_property_read_u32(child, "gpu",
					 &bcl_dev->zone[idx]->bcl_qos->gpu_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->gpu_limit = INT_MAX;
		if (of_property_read_u32(child, "tpu",
					 &bcl_dev->zone[idx]->bcl_qos->tpu_limit) != 0)
			bcl_dev->zone[idx]->bcl_qos->tpu_limit = INT_MAX;
	}
	bcl_dev->throttle = false;
}

static int google_bcl_update_last_curr(struct bcl_device *bcl_dev)
{
	int ret;
	u16 readout;

	bcl_dev->last_curr_rd_retry_cnt--;
	ret = max77779_external_fg_reg_read(bcl_dev->fg_pmic_dev,
					    MAX77779_FG_MaxMinCurr,
					    &readout);
	if (ret == -EAGAIN)
		return ret;

	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl read of last current failed: %d\n", ret);
		return ret;
	}

	readout &= MAX77779_FG_MaxMinCurr_MAXCURR_MASK;
	readout = readout >> MAX77779_FG_MaxMinCurr_MAXCURR_SHIFT;
	bcl_dev->last_current = readout;
	dev_dbg(bcl_dev->device, "LAST CURRENT: %#x\n", bcl_dev->last_current);

	return ret;
}

static void google_bcl_rd_last_curr(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device,
						  rd_last_curr_work.work);

	if (google_bcl_update_last_curr(bcl_dev) == -EAGAIN && bcl_dev->last_curr_rd_retry_cnt > 0)
		schedule_delayed_work(&bcl_dev->rd_last_curr_work,
				      msecs_to_jiffies(TIMEOUT_5S));
}

static int intf_pmic_init(struct bcl_device *bcl_dev)
{
	int ret;
	u8 val, retval;
	unsigned int uvlo1_lvl, uvlo2_lvl, batoilo_lvl, batoilo2_lvl, lvl;

	bcl_dev->batt_psy = google_get_power_supply(bcl_dev, PSY_NAME);
	batoilo_reg_read(bcl_dev->intf_pmic_dev, bcl_dev->ifpmic, BATOILO2, &lvl);
	batoilo2_lvl = BO_STEP * lvl + bcl_dev->batt_irq_conf1.batoilo_lower_limit;
	batoilo_reg_read(bcl_dev->intf_pmic_dev, bcl_dev->ifpmic, BATOILO1, &lvl);
	batoilo_lvl = BO_STEP * lvl + bcl_dev->batt_irq_conf1.batoilo_lower_limit;
	uvlo_reg_read(bcl_dev->intf_pmic_dev, bcl_dev->ifpmic, UVLO1, &uvlo1_lvl);
	uvlo_reg_read(bcl_dev->intf_pmic_dev, bcl_dev->ifpmic, UVLO2, &uvlo2_lvl);

	if (bcl_dev->ifpmic == MAX77759) {
		ret = google_bcl_register_zone(bcl_dev, UVLO1, "vdroop1", bcl_dev->vdroop1_pin,
				       	       VD_BATTERY_VOLTAGE - uvlo1_lvl - THERMAL_HYST_LEVEL,
				       	       gpio_to_irq(bcl_dev->vdroop1_pin), IF_PMIC);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: UVLO1\n");
			return -ENODEV;
		}
		ret = google_bcl_register_zone(bcl_dev, BATOILO1, "batoilo", bcl_dev->vdroop2_pin,
				       	       batoilo_lvl - THERMAL_HYST_LEVEL,
				       	       gpio_to_irq(bcl_dev->vdroop2_pin), IF_PMIC);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: BATOILO\n");
			return -ENODEV;
		}
		ret = google_bcl_register_zone(bcl_dev, UVLO2, "vdroop2", bcl_dev->vdroop2_pin,
				       	       VD_BATTERY_VOLTAGE - uvlo2_lvl - THERMAL_HYST_LEVEL,
				       	       gpio_to_irq(bcl_dev->vdroop2_pin), IF_PMIC);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: UVLO2\n");
			return -ENODEV;
		}

#if !IS_ENABLED(CONFIG_SOC_ZUMAPRO)
		/* Setup batoilo detect deglitch */
		ret = max77759_external_reg_read(bcl_dev->intf_pmic_dev, MAX77759_CHG_CNFG_17, &val);
		if (ret < 0)
			return -ENODEV;

		ret = max77759_external_reg_write(bcl_dev->intf_pmic_dev, MAX77759_CHG_CNFG_17,
					 (u8) val | (bcl_dev->batt_irq_conf1.batoilo_det << BATOILO_DET_SHIFT));
		if (ret < 0)
			return -EIO;
#endif
		bcl_cb_clr_irq(bcl_dev, UVLO1);
		bcl_cb_clr_irq(bcl_dev, UVLO2);
		bcl_cb_clr_irq(bcl_dev, BATOILO1);
	}
	if (bcl_dev->ifpmic == MAX77779) {
		ret = google_bcl_register_zone(bcl_dev, UVLO1, "vdroop1", bcl_dev->vdroop1_pin,
				       	       VD_BATTERY_VOLTAGE - uvlo1_lvl - THERMAL_HYST_LEVEL,
				       	       gpio_to_irq(bcl_dev->vdroop1_pin), IF_PMIC);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: UVLO1\n");
			return -ENODEV;
		}
		ret = google_bcl_register_zone(bcl_dev, BATOILO1, "batoilo", bcl_dev->vdroop2_pin,
				       	       batoilo_lvl - THERMAL_HYST_LEVEL,
				       	       gpio_to_irq(bcl_dev->vdroop2_pin), IF_PMIC);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: BATOILO\n");
			return -ENODEV;
		}
		ret = google_bcl_register_zone(bcl_dev, BATOILO2, "batoilo2", bcl_dev->vdroop2_pin,
					       batoilo2_lvl - THERMAL_HYST_LEVEL,
					       gpio_to_irq(bcl_dev->vdroop2_pin), IF_PMIC);
		if (ret < 0) {
			dev_err(bcl_dev->device, "bcl_register fail: BATOILO2\n");
			return -ENODEV;
		}
		/* Setup mitigation IRQ */
		ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_dev,
		                                       MAX77779_PMIC_VDROOP_INT_MASK,
		                                       bcl_dev->vdroop_int_mask);
		ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_dev,
		                                      MAX77779_PMIC_INTB_MASK, &retval);
		val = bcl_dev->intb_int_mask;
		retval = _max77779_pmic_intb_mask_vdroop_int_m_set(retval, val);
		ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_dev,
		                                       MAX77779_PMIC_INTB_MASK, retval);

		/* UVLO2 no VDROOP2 */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_SYS_UVLO2_CNFG_1, &val);
		val = _max77779_sys_uvlo2_cnfg_1_sys_uvlo2_vdrp2_en_set(val,
									bcl_dev->uvlo2_vdrp2_en);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
						      MAX77779_SYS_UVLO2_CNFG_1, val);
		val = _max77779_sys_uvlo2_cnfg_0_sys_uvlo2_set(val, bcl_dev->uvlo2_lvl);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_SYS_UVLO2_CNFG_0, val);
		/* UVLO1 = VDROOP1, 3.1V */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_SYS_UVLO1_CNFG_1, &val);
		val = _max77779_sys_uvlo1_cnfg_1_sys_uvlo1_vdrp1_en_set(val,
									bcl_dev->uvlo1_vdrp1_en);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_SYS_UVLO1_CNFG_1, val);
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_SYS_UVLO1_CNFG_0, &val);
		val = _max77779_sys_uvlo1_cnfg_0_sys_uvlo1_set(val,
							       bcl_dev->uvlo1_lvl);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_SYS_UVLO1_CNFG_0, val);

		/* BATOILO1 = VDROOP2, 36ms BATOILO1 BAT_OPEN */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_BAT_OILO1_CNFG_3, &val);
		val = _max77779_bat_oilo1_cnfg_3_bat_oilo1_vdrp1_en_set(val,
									bcl_dev->oilo1_vdrp1_en);
		val = _max77779_bat_oilo1_cnfg_3_bat_oilo1_vdrp2_en_set(val,
									bcl_dev->oilo1_vdrp2_en);
		val = _max77779_bat_oilo1_cnfg_3_bat_open_to_1_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_bat_open_to);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_BAT_OILO1_CNFG_3, val);

		/* BATOILO2 = VDROOP1/2, 12ms BATOILO2 BAT_OPEN */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_BAT_OILO2_CNFG_3, &val);
		val = _max77779_bat_oilo2_cnfg_3_bat_oilo2_vdrp1_en_set(val,
									bcl_dev->oilo2_vdrp1_en);
		val = _max77779_bat_oilo2_cnfg_3_bat_oilo2_vdrp2_en_set(val,
									bcl_dev->oilo2_vdrp2_en);
		val = _max77779_bat_oilo2_cnfg_3_bat_open_to_2_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_bat_open_to);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_BAT_OILO2_CNFG_3, val);

		/* BATOILO1 5A THRESHOLD */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_BAT_OILO1_CNFG_0, &val);
		val = _max77779_bat_oilo1_cnfg_0_bat_oilo1_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_trig_lvl);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_BAT_OILO1_CNFG_0, val);

		/* BATOILO2 8A THRESHOLD */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_BAT_OILO2_CNFG_0, &val);
		val = _max77779_bat_oilo2_cnfg_0_bat_oilo2_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_trig_lvl);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
						      MAX77779_BAT_OILO2_CNFG_0, val);

		/* BATOILO INT and VDROOP1 REL and DET */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_BAT_OILO1_CNFG_1, &val);
		val = _max77779_bat_oilo1_cnfg_1_bat_oilo1_rel_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_rel);
		val = _max77779_bat_oilo1_cnfg_1_bat_oilo1_det_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_BAT_OILO1_CNFG_1, val);

		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_BAT_OILO1_CNFG_2, &val);
		val = _max77779_bat_oilo1_cnfg_2_bat_oilo1_int_rel_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_int_rel);
		val = _max77779_bat_oilo1_cnfg_2_bat_oilo1_int_det_set(
						 val, bcl_dev->batt_irq_conf1.batoilo_int_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_BAT_OILO1_CNFG_2, val);

		/* BATOILO2 INT and VDROOP2 REL and DET */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_BAT_OILO2_CNFG_1, &val);
		val = _max77779_bat_oilo2_cnfg_1_bat_oilo2_rel_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_rel);
		val = _max77779_bat_oilo2_cnfg_1_bat_oilo2_det_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_BAT_OILO2_CNFG_1, val);

		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_BAT_OILO2_CNFG_2, &val);
		val = _max77779_bat_oilo2_cnfg_2_bat_oilo2_int_rel_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_int_rel);
		val = _max77779_bat_oilo2_cnfg_2_bat_oilo2_int_det_set(
						 val, bcl_dev->batt_irq_conf2.batoilo_int_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_BAT_OILO2_CNFG_2, val);

		/* UVLO1 INT and VDROOP1 REL and DET */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_SYS_UVLO1_CNFG_1, &val);
		val = _max77779_sys_uvlo1_cnfg_1_sys_uvlo1_rel_set(
						 val, bcl_dev->batt_irq_conf1.uvlo_rel);
		val = _max77779_sys_uvlo1_cnfg_1_sys_uvlo1_det_set(
						 val, bcl_dev->batt_irq_conf1.uvlo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_SYS_UVLO1_CNFG_1, val);

		/* UVLO2 INT and VDROOP1 REL and DET */
		ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
						     MAX77779_SYS_UVLO2_CNFG_1, &val);
		val = _max77779_sys_uvlo2_cnfg_1_sys_uvlo2_rel_set(
						 val, bcl_dev->batt_irq_conf2.uvlo_rel);
		val = _max77779_sys_uvlo2_cnfg_1_sys_uvlo2_det_set(
						 val, bcl_dev->batt_irq_conf2.uvlo_det);
		ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
		                                      MAX77779_SYS_UVLO2_CNFG_1, val);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
		/* Read, save, and clear event counters */
		evt_cnt_rd_and_clr(bcl_dev, UVLO1, true);
		evt_cnt_rd_and_clr(bcl_dev, UVLO2, true);
		evt_cnt_rd_and_clr(bcl_dev, BATOILO1, true);
		evt_cnt_rd_and_clr(bcl_dev, BATOILO2, true);

		/* Enable event counter if it is not enabled */
		ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_dev,
						 MAX77779_PMIC_EVENT_CNT_CFG, &retval);
		retval = _max77779_pmic_event_cnt_cfg_enable_set(
						 retval, bcl_dev->evt_cnt.enable);
		retval = _max77779_pmic_event_cnt_cfg_sample_rate_set(
						 retval, bcl_dev->evt_cnt.rate);
		ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_dev,
						  MAX77779_PMIC_EVENT_CNT_CFG, retval);
		bcl_cb_clr_irq(bcl_dev, UVLO1);
		bcl_cb_clr_irq(bcl_dev, UVLO2);
		bcl_cb_clr_irq(bcl_dev, BATOILO1);
		bcl_cb_clr_irq(bcl_dev, BATOILO2);
#endif
	}
	return ret;
}

static int google_set_intf_pmic(struct bcl_device *bcl_dev, struct platform_device *pdev)
{
	int ret = 0;
	u32 retval;
	struct device_node *np = bcl_dev->device->of_node;

	ret = of_property_read_u32(np, "google,ifpmic", &retval);
	bcl_dev->ifpmic = (retval == M77759) ? MAX77759 : MAX77779;

	bcl_dev->intf_pmic_dev = max77779_get_dev(bcl_dev->device, "google,charger");
	if (!bcl_dev->intf_pmic_dev) {
		dev_err(bcl_dev->device, "Cannot find Charger I2C\n");
		return -ENODEV;
	}

	if (bcl_dev->ifpmic == MAX77779) {
		ret = platform_get_irq(pdev, 0);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Failed to get irq: %d\n", ret);
			return -ENODEV;
		}
		bcl_dev->pmic_irq = ret;
	}

	INIT_DELAYED_WORK(&bcl_dev->rd_last_curr_work, google_bcl_rd_last_curr);

	if (np) {
		ret = of_property_read_u32(np, "batoilo_lower", &retval);
		bcl_dev->batt_irq_conf1.batoilo_lower_limit = ret ? BO_LOWER_LIMIT : retval;
		ret = of_property_read_u32(np, "batoilo_upper", &retval);
		bcl_dev->batt_irq_conf1.batoilo_upper_limit = ret ? BO_UPPER_LIMIT : retval;
		ret = of_property_read_u32(np, "batoilo2_lower", &retval);
		bcl_dev->batt_irq_conf2.batoilo_lower_limit = ret ? BO_LOWER_LIMIT : retval;
		ret = of_property_read_u32(np, "batoilo2_upper", &retval);
		bcl_dev->batt_irq_conf2.batoilo_upper_limit = ret ? BO_UPPER_LIMIT : retval;
		ret = of_property_read_u32(np, "batoilo_trig_lvl", &retval);
		retval = ret ? BO_LIMIT : retval;
		bcl_dev->batt_irq_conf1.batoilo_trig_lvl =
				(retval - bcl_dev->batt_irq_conf1.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo2_trig_lvl", &retval);
		retval = ret ? BO_LIMIT : retval;
		bcl_dev->batt_irq_conf2.batoilo_trig_lvl =
				(retval - bcl_dev->batt_irq_conf2.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo_otg_trig_lvl", &retval);
		bcl_dev->batt_irq_conf1.batoilo_otg_trig_lvl = ret ?
				bcl_dev->batt_irq_conf1.batoilo_trig_lvl :
				(retval - bcl_dev->batt_irq_conf1.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo2_otg_trig_lvl", &retval);
		bcl_dev->batt_irq_conf2.batoilo_otg_trig_lvl = ret ?
				bcl_dev->batt_irq_conf2.batoilo_trig_lvl :
				(retval - bcl_dev->batt_irq_conf2.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo_usb_trig_lvl", &retval);
		bcl_dev->batt_irq_conf1.batoilo_usb_trig_lvl = ret ?
				bcl_dev->batt_irq_conf1.batoilo_trig_lvl :
				(retval - bcl_dev->batt_irq_conf1.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo2_usb_trig_lvl", &retval);
		bcl_dev->batt_irq_conf2.batoilo_usb_trig_lvl = ret ?
				bcl_dev->batt_irq_conf2.batoilo_trig_lvl :
				(retval - bcl_dev->batt_irq_conf2.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo_wlc_trig_lvl", &retval);
		bcl_dev->batt_irq_conf1.batoilo_wlc_trig_lvl = ret ?
				bcl_dev->batt_irq_conf1.batoilo_trig_lvl :
				(retval - bcl_dev->batt_irq_conf1.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo2_wlc_trig_lvl", &retval);
		bcl_dev->batt_irq_conf2.batoilo_wlc_trig_lvl = ret ?
				bcl_dev->batt_irq_conf2.batoilo_trig_lvl :
				(retval - bcl_dev->batt_irq_conf2.batoilo_lower_limit) / BO_STEP;
		ret = of_property_read_u32(np, "batoilo_otg_bat_open_to", &retval);
		bcl_dev->batt_irq_conf1.batoilo_bat_otg_open_to = ret ?
				BO_BAT_OPEN_TO_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo_bat_open_to", &retval);
		bcl_dev->batt_irq_conf1.batoilo_bat_open_to = ret ? BO_BAT_OPEN_TO_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo2_bat_open_to", &retval);
		bcl_dev->batt_irq_conf2.batoilo_bat_open_to = ret ? BO_BAT_OPEN_TO_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo_rel", &retval);
		bcl_dev->batt_irq_conf1.batoilo_rel = ret ? BO_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo2_rel", &retval);
		bcl_dev->batt_irq_conf2.batoilo_rel = ret ? BO_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo_int_rel", &retval);
		bcl_dev->batt_irq_conf1.batoilo_int_rel = ret ? BO_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo2_int_rel", &retval);
		bcl_dev->batt_irq_conf2.batoilo_int_rel = ret ? BO_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo_det", &retval);
		bcl_dev->batt_irq_conf1.batoilo_det = ret ? BO_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo2_det", &retval);
		bcl_dev->batt_irq_conf2.batoilo_det = ret ? BO_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo_int_det", &retval);
		bcl_dev->batt_irq_conf1.batoilo_int_det = ret ? BO_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "batoilo2_int_det", &retval);
		bcl_dev->batt_irq_conf2.batoilo_int_det = ret ? BO_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "uvlo1_det", &retval);
		bcl_dev->batt_irq_conf1.uvlo_det = ret ? UV_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "uvlo2_det", &retval);
		bcl_dev->batt_irq_conf2.uvlo_det = ret ? UV_INT_REL_DEFAULT : retval;
		ret = of_property_read_u32(np, "uvlo1_rel", &retval);
		bcl_dev->batt_irq_conf1.uvlo_rel = ret ? UV_INT_DET_DEFAULT : retval;
		ret = of_property_read_u32(np, "uvlo2_rel", &retval);
		bcl_dev->batt_irq_conf2.uvlo_rel = ret ? UV_INT_DET_DEFAULT : retval;
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
		ret = of_property_read_u32(np, "evt_cnt_enable", &retval);
		bcl_dev->evt_cnt.enable = ret ? EVT_CNT_ENABLE_DEFAULT : retval;
		ret = of_property_read_u32(np, "evt_cnt_rate", &retval);
		bcl_dev->evt_cnt.rate = ret ? EVT_CNT_RATE_DEFAULT : retval;
#endif
		bcl_dev->usb_otg_conf = of_property_read_bool(np, "usb_otg_conf");
		bcl_dev->uvlo1_vdrp1_en = of_property_read_bool(np, "uvlo1_vdrp1_en");
		bcl_dev->uvlo1_vdrp2_en = of_property_read_bool(np, "uvlo1_vdrp2_en");
		bcl_dev->uvlo2_vdrp1_en = of_property_read_bool(np, "uvlo2_vdrp1_en");
		bcl_dev->uvlo2_vdrp2_en = of_property_read_bool(np, "uvlo2_vdrp2_en");
		bcl_dev->oilo1_vdrp1_en = of_property_read_bool(np, "oilo1_vdrp1_en");
		bcl_dev->oilo1_vdrp2_en = of_property_read_bool(np, "oilo1_vdrp2_en");
		bcl_dev->oilo2_vdrp1_en = of_property_read_bool(np, "oilo2_vdrp1_en");
		bcl_dev->oilo2_vdrp2_en = of_property_read_bool(np, "oilo2_vdrp2_en");
		bcl_dev->vimon_pwr_loop_en = of_property_read_bool(np, "vimon_pwr_loop_en");
		ret = of_property_read_u32(np, "uvlo1_lvl", &retval);
		bcl_dev->uvlo1_lvl = ret ? DEFAULT_SYS_UVLO1_LVL : retval;
		ret = of_property_read_u32(np, "uvlo2_lvl", &retval);
		bcl_dev->uvlo2_lvl = ret ? DEFAULT_SYS_UVLO2_LVL : retval;
		ret = of_property_read_u32(np, "vdroop_int_mask", &retval);
		bcl_dev->vdroop_int_mask = ret ? DEFAULT_VDROOP_INT_MASK : retval;
		ret = of_property_read_u32(np, "intb_int_mask", &retval);
		bcl_dev->intb_int_mask = ret ? DEFAULT_INTB_MASK : retval;
		ret = of_property_read_u32(np, "vimon_pwr_loop_cnt", &retval);
		bcl_dev->vimon_pwr_loop_cnt = ret ? DEFAULT_VIMON_PWR_LOOP_CNT : retval;
		ret = of_property_read_u32(np, "vimon_pwr_loop_thresh", &retval);
		bcl_dev->vimon_pwr_loop_thresh = ret ? DEFAULT_VIMON_PWR_LOOP_THRESH : retval;
		bcl_dev->bat_ktimer_en = of_property_read_bool(np, "bat_ktimer_en");
		ret = of_property_read_u32(np, "bat_ktimer", &retval);
		bcl_dev->bat_ktimer = ret ? BAT_KTIMER_LIMIT_MS : retval;
	}

	if (bcl_dev->ifpmic == MAX77779) {
		bcl_dev->irq_pmic_dev = max77779_get_dev(bcl_dev->device, "google,pmic");
		if (!bcl_dev->irq_pmic_dev) {
			dev_err(bcl_dev->device, "Cannot find PMIC bus\n");
			return -ENODEV;
		}

		bcl_dev->fg_pmic_dev = max77779_get_dev(bcl_dev->device, "google,power-supply");
		if (!bcl_dev->fg_pmic_dev) {
			dev_err(bcl_dev->device, "Cannot find google,power-supply\n");
			return -ENODEV;
		}

		/* Readout last current */
		bcl_dev->last_curr_rd_retry_cnt = LAST_CURR_RD_CNT_MAX;
		if (google_bcl_update_last_curr(bcl_dev) == -EAGAIN)
			schedule_delayed_work(&bcl_dev->rd_last_curr_work,
					      msecs_to_jiffies(TIMEOUT_5S));

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
		bcl_dev->vimon_dev = max77779_get_dev(bcl_dev->device, "google,vimon");
		if (!bcl_dev->vimon_dev) {
			dev_err(bcl_dev->device, "Cannot find max77779 vimon\n");
			return -ENODEV;
		}
		if (!bcl_dev->vimon_pwr_loop_en)
			return 0;
		ret = max77779_vimon_register_callback(bcl_dev);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Cannot register max77779 vimon\n");
			return -ENODEV;
		}
#endif
	}

	ret = intf_pmic_init(bcl_dev);
	if (ret < 0) {
		dev_err(bcl_dev->device, "Interface PMIC initialization err:%d\n", ret);
		return ret;
	}

	google_bcl_parse_qos(bcl_dev);
	if (google_bcl_setup_qos(bcl_dev) != 0) {
		dev_err(bcl_dev->device, "Cannot Initiate QOS\n");
		return -ENODEV;
	}

	return 0;
}

static int google_bcl_register_zones_main(struct bcl_device *bcl_dev, void *pdata_main)
{
	int ret;
	unsigned int ocp_cpu1_pin, ocp_cpu1_lvl;
	unsigned int ocp_cpu2_pin, ocp_cpu2_lvl;
	unsigned int ocp_tpu_pin, ocp_tpu_lvl;
	unsigned int soft_ocp_cpu1_pin, soft_ocp_cpu1_lvl;
	unsigned int soft_ocp_cpu2_pin, soft_ocp_cpu2_lvl;
	unsigned int soft_ocp_tpu_pin, soft_ocp_tpu_lvl;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	struct s2mpg14_platform_data *pdata = pdata_main;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	struct s2mpg12_platform_data *pdata = pdata_main;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	struct s2mpg10_platform_data *pdata = pdata_main;
#endif

	ocp_cpu2_pin = pdata->b2_ocp_warn_pin;
	ocp_cpu2_lvl = pdata->b2_ocp_warn_lvl;
	ocp_cpu1_pin = pdata->b3_ocp_warn_pin;
	ocp_cpu1_lvl = pdata->b3_ocp_warn_lvl;
	soft_ocp_cpu2_pin = pdata->b2_soft_ocp_warn_pin;
	soft_ocp_cpu2_lvl = pdata->b2_soft_ocp_warn_lvl;
	soft_ocp_cpu1_pin = pdata->b3_soft_ocp_warn_pin;
	soft_ocp_cpu1_lvl = pdata->b3_soft_ocp_warn_lvl;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	ocp_tpu_pin = pdata->b7_ocp_warn_pin;
	ocp_tpu_lvl = pdata->b7_ocp_warn_lvl;
	soft_ocp_tpu_pin = pdata->b7_soft_ocp_warn_pin;
	soft_ocp_tpu_lvl = pdata->b7_soft_ocp_warn_lvl;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	ocp_tpu_pin = pdata->b10_ocp_warn_pin;
	ocp_tpu_lvl = pdata->b10_ocp_warn_lvl;
	soft_ocp_tpu_pin = pdata->b10_soft_ocp_warn_pin;
	soft_ocp_tpu_lvl = pdata->b10_soft_ocp_warn_lvl;
#endif

	ret = google_bcl_register_zone(bcl_dev, SMPL_WARN, SMPL_ZONE_NAME,
				       pdata->smpl_warn_pin, SMPL_BATTERY_VOLTAGE -
				       (pdata->smpl_warn_lvl * SMPL_STEP + SMPL_LOWER_LIMIT),
				       gpio_to_irq(pdata->smpl_warn_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SMPL_WARN\n");
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_SOC_ZUMAPRO))
		return 0;

	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_CPUCL1, "ocp_cpu1",
				       ocp_cpu1_pin,
				       CPU1_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (ocp_cpu1_lvl * CPU1_STEP),
				       gpio_to_irq(ocp_cpu1_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL1\n");
		return -ENODEV;
	}

	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_CPUCL2, "ocp_cpu2",
				       ocp_cpu2_pin,
				       CPU2_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (ocp_cpu2_lvl * CPU2_STEP),
				       gpio_to_irq(ocp_cpu2_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: CPUCL2\n");
		return -ENODEV;
	}

	ret = google_bcl_register_zone(bcl_dev, OCP_WARN_TPU, "ocp_tpu",
				       ocp_tpu_pin,
				       TPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (ocp_tpu_lvl * TPU_STEP),
				       gpio_to_irq(ocp_tpu_pin), CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: TPU\n");
		return -ENODEV;
	}

	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_CPUCL1, "soft_ocp_cpu1",
				       soft_ocp_cpu1_pin,
				       CPU1_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (soft_ocp_cpu1_lvl * CPU1_STEP),
				       gpio_to_irq(soft_ocp_cpu1_pin),
				       CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL1\n");
		return -ENODEV;
	}

	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_CPUCL2, "soft_ocp_cpu2",
				       soft_ocp_cpu2_pin,
				       CPU2_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (soft_ocp_cpu2_lvl * CPU2_STEP),
				       gpio_to_irq(soft_ocp_cpu2_pin),
				       CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_CPUCL2\n");
		return -ENODEV;
	}

	ret = google_bcl_register_zone(bcl_dev, SOFT_OCP_WARN_TPU, "soft_ocp_tpu",
				       soft_ocp_tpu_pin,
				       TPU_UPPER_LIMIT - THERMAL_HYST_LEVEL -
				       (soft_ocp_tpu_lvl * TPU_STEP),
				       gpio_to_irq(soft_ocp_tpu_pin),
				       CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: SOFT_TPU\n");
		return -ENODEV;
	}

	ret = google_bcl_register_zone(bcl_dev, PMIC_120C, "PMIC_120C",
					0,
					PMIC_120C_UPPER_LIMIT - THERMAL_HYST_LEVEL,
					pdata->irq_base + INT3_120C,
					CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_120C\n");
		return -ENODEV;
	}

	ret = google_bcl_register_zone(bcl_dev, PMIC_140C, "PMIC_140C",
					0,
					PMIC_140C_UPPER_LIMIT - THERMAL_HYST_LEVEL,
					pdata->irq_base + INT3_140C,
					CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_140C\n");
		return -ENODEV;
	}

#if !IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	ret = google_bcl_register_zone(bcl_dev, PMIC_OVERHEAT, "PMIC_OVERHEAT",
					0,
					PMIC_OVERHEAT_UPPER_LIMIT - THERMAL_HYST_LEVEL,
					pdata->irq_base + INT3_TSD,
					CORE_MAIN_PMIC);
	if (ret < 0) {
		dev_err(bcl_dev->device, "bcl_register fail: PMIC_OVERHEAT\n");
		return -ENODEV;
	}
#endif
	return 0;
}

static int google_set_main_pmic(struct bcl_device *bcl_dev)
{
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	struct s2mpg14_platform_data *pdata_main;
	struct s2mpg14_dev *main_dev = NULL;
	int i;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	struct s2mpg12_platform_data *pdata_main;
	struct s2mpg12_dev *main_dev = NULL;
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
	struct s2mpg10_platform_data *pdata_main;
	struct s2mpg10_dev *main_dev = NULL;
#endif
	u8 val;
	struct device_node *p_np;
	struct device_node *np = bcl_dev->device->of_node;
	struct i2c_client *i2c;
	int ret;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	INIT_DELAYED_WORK(&bcl_dev->main_pwr_irq_work, main_pwrwarn_irq_work);
#endif

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
	if (!pdata_main) {
		dev_err(bcl_dev->device, "Main platform data not found\n");
		return -ENODEV;
	}

	bcl_dev->main_irq_base = pdata_main->irq_base;
	bcl_dev->main_pmic_i2c = main_dev->pmic;
	bcl_dev->main_rtc_i2c = main_dev->rtc;
	bcl_dev->main_meter_i2c = main_dev->meter;
	bcl_dev->main_dev = main_dev->dev;
	/* clear MAIN information every boot */
	/* see b/215371539 */
	pmic_read(CORE_PMIC_MAIN, bcl_dev, MAIN_OFFSRC1, &val);
	dev_info(bcl_dev->device, "MAIN OFFSRC1 : %#x\n", val);
	bcl_dev->main_offsrc1 = val;
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	pmic_read(CORE_PMIC_MAIN, bcl_dev, MAIN_OFFSRC2, &val);
	dev_info(bcl_dev->device, "MAIN OFFSRC2 : %#x\n", val);
	bcl_dev->main_offsrc2 = val;
	pmic_write(CORE_PMIC_MAIN_RTC, bcl_dev, RTC_SCRATCH1, 0);
#endif
	pmic_read(CORE_PMIC_MAIN, bcl_dev, MAIN_PWRONSRC, &val);
	dev_info(bcl_dev->device, "MAIN PWRONSRC: %#x\n", val);
	bcl_dev->pwronsrc = val;
	pmic_write(CORE_PMIC_MAIN, bcl_dev, MAIN_OFFSRC1, 0);
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	pmic_write(CORE_PMIC_MAIN, bcl_dev, MAIN_OFFSRC2, 0);
#endif
	pmic_write(CORE_PMIC_MAIN, bcl_dev, MAIN_PWRONSRC, 0);
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	/* SMPL_WARN = 3.0V */
	pmic_write(CORE_PMIC_MAIN, bcl_dev, S2MPG14_PM_SMPL_WARN_CTRL, bcl_dev->smpl_ctrl);
#endif

	ret = google_bcl_register_zones_main(bcl_dev, pdata_main);
	if (ret < 0)
		return ret;

	INIT_DELAYED_WORK(&bcl_dev->setup_main_odpm_work, google_bcl_setup_main_odpm);
	schedule_delayed_work(&bcl_dev->setup_main_odpm_work, msecs_to_jiffies(TIMEOUT_5MS));

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		bcl_dev->main_pwr_warn_irq[i] = bcl_dev->main_irq_base
				+ S2MPG14_IRQ_PWR_WARN_CH0_INT6 + i;
		ret = devm_request_threaded_irq(bcl_dev->device, bcl_dev->main_pwr_warn_irq[i],
						NULL, main_pwr_warn_irq_handler, 0,
						bcl_dev->main_rail_names[i], bcl_dev);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Failed to request PWR_WARN_CH%d IRQ: %d: %d\n",
				i, bcl_dev->main_pwr_warn_irq[i], ret);
		}
	}
#endif

	return 0;
}

extern const struct attribute_group *mitigation_mw_groups[];
extern const struct attribute_group *mitigation_sq_groups[];

static int google_init_fs(struct bcl_device *bcl_dev)
{
	if (bcl_dev->ifpmic == MAX77759)
		bcl_dev->mitigation_dev = pmic_subdevice_create(NULL, mitigation_mw_groups,
								bcl_dev, "mitigation");
	else
		bcl_dev->mitigation_dev = pmic_subdevice_create(NULL, mitigation_sq_groups,
								bcl_dev, "mitigation");
	if (IS_ERR(bcl_dev->mitigation_dev))
		return -ENODEV;

	return 0;
}

static void google_bcl_enable_vdroop_irq(struct bcl_device *bcl_dev)
{
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	void __iomem *gpio_alive;
	unsigned int reg;
#endif
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	gpio_alive = ioremap(GPIO_ALIVE_BASE, SZ_4K);
	reg = __raw_readl(gpio_alive + GPA9_CON);
	reg |= 0xFF0000;
	__raw_writel(0xFFFFF22, gpio_alive + GPA9_CON);
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	gpio_alive = ioremap(GPIO_ALIVE_BASE, SZ_4K);
	reg = __raw_readl(gpio_alive + GPA5_CON);
	reg |= 0xFF0000;
	__raw_writel(0xFFFFF22, gpio_alive + GPA5_CON);
#endif
}

static int google_bcl_init_instruction(struct bcl_device *bcl_dev)
{
	if (IS_ERR_OR_NULL(bcl_dev))
		return -EIO;

	bcl_dev->core_conf[SUBSYSTEM_CPU0].base_mem = devm_ioremap(bcl_dev->device,
	                                                           CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_CPU0].base_mem) {
		dev_err(bcl_dev->device, "cpu0_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_CPU1].base_mem = devm_ioremap(bcl_dev->device,
	                                                           CPUCL1_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_CPU1].base_mem) {
		dev_err(bcl_dev->device, "cpu1_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_CPU2].base_mem = devm_ioremap(bcl_dev->device,
	                                                           CPUCL2_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_CPU2].base_mem) {
		dev_err(bcl_dev->device, "cpu2_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_TPU].base_mem = devm_ioremap(bcl_dev->device,
	                                                          TPU_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_TPU].base_mem) {
		dev_err(bcl_dev->device, "tpu_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_GPU].base_mem = devm_ioremap(bcl_dev->device,
	                                                          G3D_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_GPU].base_mem) {
		dev_err(bcl_dev->device, "gpu_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->core_conf[SUBSYSTEM_AUR].base_mem = devm_ioremap(bcl_dev->device,
	                                                          AUR_BASE, SZ_8K);
	if (!bcl_dev->core_conf[SUBSYSTEM_AUR].base_mem) {
		dev_err(bcl_dev->device, "aur_mem ioremap failed\n");
		return -EIO;
	}
	bcl_dev->sysreg_cpucl0 = devm_ioremap(bcl_dev->device, SYSREG_CPUCL0_BASE, SZ_8K);
	if (!bcl_dev->sysreg_cpucl0) {
		dev_err(bcl_dev->device, "sysreg_cpucl0 ioremap failed\n");
		return -EIO;
	}

	mutex_init(&bcl_dev->sysreg_lock);
	mutex_init(&bcl_dev->cpu_ratio_lock);
	google_bcl_enable_vdroop_irq(bcl_dev);

	bcl_dev->base_add_mem[SUBSYSTEM_CPU0] = devm_ioremap(bcl_dev->device, ADD_CPUCL0, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_CPU0]) {
		dev_err(bcl_dev->device, "cpu0_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_CPU1] = devm_ioremap(bcl_dev->device, ADD_CPUCL1, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_CPU1]) {
		dev_err(bcl_dev->device, "cpu1_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_CPU2] = devm_ioremap(bcl_dev->device, ADD_CPUCL2, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_CPU2]) {
		dev_err(bcl_dev->device, "cpu2_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_TPU] = devm_ioremap(bcl_dev->device, ADD_TPU, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_TPU]) {
		dev_err(bcl_dev->device, "tpu_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_GPU] = devm_ioremap(bcl_dev->device, ADD_G3D, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_GPU]) {
		dev_err(bcl_dev->device, "gpu_add_mem ioremap failed\n");
		return -EIO;
	}

	bcl_dev->base_add_mem[SUBSYSTEM_AUR] = devm_ioremap(bcl_dev->device, ADD_AUR, SZ_128);
	if (!bcl_dev->base_add_mem[SUBSYSTEM_AUR]) {
		dev_err(bcl_dev->device, "aur_add_mem ioremap failed\n");
		return -EIO;
	}
	return 0;
}

u64 settings_to_current(struct bcl_device *bcl_dev, int pmic, int idx, u32 setting)
{
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	int rail_i;
	enum s2mpg1415_meter_muxsel muxsel;
	struct odpm_info *info;
	u64 raw_unit;
	u32 resolution;
	const char *rail_name;

	if (IS_ERR_OR_NULL(bcl_dev))
		return 0;
	if (pmic == CORE_PMIC_MAIN)
		info = bcl_dev->main_odpm;
	else
		info = bcl_dev->sub_odpm;

	if (!info)
		return 0;

	rail_i = info->channels[idx].rail_i;
	muxsel = info->chip.rails[rail_i].mux_select;
	if (pmic == CORE_PMIC_MAIN)
		rail_name = bcl_dev->main_rail_names[idx];
	else
		rail_name = bcl_dev->sub_rail_names[idx];

	if (!rail_name)
		return 0;

	if (strstr(rail_name, "VSYS") != NULL) {
			resolution = (u32) VSHUNT_MULTIPLIER * ((u64)EXTERNAL_RESOLUTION_VSHUNT) /
					info->chip.rails[rail_i].shunt_uohms;
	} else {
		if (pmic == CORE_PMIC_MAIN)
			resolution = s2mpg14_muxsel_to_current_resolution(muxsel);
		else
			resolution = s2mpg15_muxsel_to_current_resolution(muxsel);
	}
	raw_unit = (u64)setting * resolution;
	raw_unit = raw_unit * MILLI_TO_MICRO;
	return (u32)_IQ30_to_int(raw_unit);
#endif
	return 0;
}

static void irq_config(struct bcl_zone *zone, bool enabled)
{
	if (!zone)
		return;
	if (!enabled && !zone->disabled) {
		zone->disabled = true;
		disable_irq_nosync(zone->bcl_irq);
	} else if (enabled && zone->disabled && zone->irq_reg) {
		zone->disabled = false;
		if (zone->bcl_pin != NOT_USED)
			enable_irq(zone->bcl_irq);
	}
}

static void google_bcl_parse_irq_config(struct bcl_device *bcl_dev)
{
	struct device_node *np = bcl_dev->device->of_node;
	struct device_node *child;
        /* irq config */
	child = of_get_child_by_name(np, "irq_config");
	if (!child)
		return;
	irq_config(bcl_dev->zone[UVLO1], of_property_read_bool(child, "irq,uvlo1"));
	irq_config(bcl_dev->zone[UVLO2], of_property_read_bool(child, "irq,uvlo2"));
	/* This enables BATOILO2 as well */
	irq_config(bcl_dev->zone[SMPL_WARN], of_property_read_bool(child, "irq,smpl_warn"));
	irq_config(bcl_dev->zone[BATOILO2], of_property_read_bool(child, "irq,batoilo2"));
	if (bcl_dev->ifpmic == MAX77779)
		return;
	irq_config(bcl_dev->zone[BATOILO], of_property_read_bool(child, "irq,batoilo"));
	irq_config(bcl_dev->zone[OCP_WARN_CPUCL1], of_property_read_bool(child, "irq,ocp_cpu1"));
	irq_config(bcl_dev->zone[OCP_WARN_CPUCL2], of_property_read_bool(child, "irq,ocp_cpu2"));
	irq_config(bcl_dev->zone[OCP_WARN_TPU], of_property_read_bool(child, "irq,ocp_tpu"));
	irq_config(bcl_dev->zone[OCP_WARN_GPU], of_property_read_bool(child, "irq,ocp_gpu"));
	irq_config(bcl_dev->zone[SOFT_OCP_WARN_CPUCL1],
		   of_property_read_bool(child, "irq,soft_ocp_cpu1"));
	irq_config(bcl_dev->zone[SOFT_OCP_WARN_CPUCL2],
		   of_property_read_bool(child, "irq,soft_ocp_cpu2"));
	irq_config(bcl_dev->zone[SOFT_OCP_WARN_TPU],
		   of_property_read_bool(child, "irq,soft_ocp_tpu"));
	irq_config(bcl_dev->zone[SOFT_OCP_WARN_GPU],
		   of_property_read_bool(child, "irq,soft_ocp_gpu"));

}

static void google_bcl_clk_div(struct bcl_device *bcl_dev)
{
	if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU2,
				    bcl_dev->core_conf[SUBSYSTEM_CPU2].clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU2 Address is NULL\n");
	if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU1,
				    bcl_dev->core_conf[SUBSYSTEM_CPU1].clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU1 Address is NULL\n");
	if (google_bcl_init_clk_div(bcl_dev, SUBSYSTEM_CPU0,
	                            bcl_dev->core_conf[SUBSYSTEM_CPU0].clkdivstep) != 0)
		dev_err(bcl_dev->device, "CPU0 Address is NULL\n");
}

static void google_bcl_parse_clk_div_dtree(struct bcl_device *bcl_dev)
{
	int ret;
	struct device_node *np = bcl_dev->device->of_node;
	u32 val;

	if (IS_ERR_OR_NULL(bcl_dev)) {
		dev_err(bcl_dev->device, "Cannot parse device tree\n");
		return;
	}
	ret = of_property_read_u32(np, "tpu_con_heavy", &val);
	bcl_dev->core_conf[SUBSYSTEM_TPU].con_heavy = ret ? 0 : val;
	ret = of_property_read_u32(np, "tpu_con_light", &val);
	bcl_dev->core_conf[SUBSYSTEM_TPU].con_light = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_con_heavy", &val);
	bcl_dev->core_conf[SUBSYSTEM_GPU].con_heavy = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_con_light", &val);
	bcl_dev->core_conf[SUBSYSTEM_GPU].con_light = ret ? 0 : val;
	ret = of_property_read_u32(np, "gpu_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_GPU].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "tpu_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_TPU].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "aur_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_AUR].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu2_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_CPU2].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu1_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_CPU1].clkdivstep = ret ? 0 : val;
	ret = of_property_read_u32(np, "cpu0_clkdivstep", &val);
	bcl_dev->core_conf[SUBSYSTEM_CPU0].clkdivstep = ret ? 0 : val;
	bcl_dev->vdroop1_pin = of_get_gpio(np, 0);
	bcl_dev->vdroop2_pin = of_get_gpio(np, 1);
	bcl_dev->modem_gpio1_pin = of_get_gpio(np, 2);
	bcl_dev->modem_gpio2_pin = of_get_gpio(np, 3);
	ret = of_property_read_u32(np, "rffe_channel", &val);
	bcl_dev->rffe_channel = ret ? 11 : val;
	ret = of_property_read_u32(np, "cpu0_cluster", &val);
	bcl_dev->cpu0_cluster = ret ? CPU0_CLUSTER_MIN : val;
	ret = of_property_read_u32(np, "cpu1_cluster", &val);
	bcl_dev->cpu1_cluster = ret ? CPU1_CLUSTER_MIN : val;
	ret = of_property_read_u32(np, "cpu2_cluster", &val);
	bcl_dev->cpu2_cluster = ret ? CPU2_CLUSTER_MIN : val;
	ret = of_property_read_u32(np, "smpl_ctrl", &val);
	bcl_dev->smpl_ctrl = ret ? DEFAULT_SMPL : val;

	bcl_dev->qos_update_wq = create_singlethread_workqueue("bcl_qos_update");
}
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static void google_bcl_parse_dtree(struct bcl_device *bcl_dev)
{
	int ret, len, i;
	int read;
	struct device_node *child;
	struct device_node *p_np;
	struct device_node *np;

	if (IS_ERR_OR_NULL(bcl_dev)) {
		dev_err(bcl_dev->device, "Cannot parse device tree\n");
		return;
	}
	np = bcl_dev->device->of_node;

	/* parse ODPM main mitigation module */
	p_np = of_get_child_by_name(np, "main_mitigation");
	if (p_np) {
		i = 0;
		for_each_child_of_node(p_np, child) {
			if (i < METER_CHANNEL_MAX) {
				of_property_read_u32(child, "module_id", &read);
				bcl_dev->main_mitigation_conf[i].module_id = read;

				of_property_read_u32(child, "threshold", &read);
				bcl_dev->main_mitigation_conf[i].threshold = read;
				i++;
			}
		}
	}

	/* parse ODPM sub mitigation module */
	p_np = of_get_child_by_name(np, "sub_mitigation");
	if (p_np) {
		i = 0;
		for_each_child_of_node(p_np, child) {
			if (i < METER_CHANNEL_MAX) {
				of_property_read_u32(child, "module_id", &read);
				bcl_dev->sub_mitigation_conf[i].module_id = read;

				of_property_read_u32(child, "threshold", &read);
				bcl_dev->sub_mitigation_conf[i].threshold = read;
				i++;
			}
		}
	}

	/* parse and init non-monitored modules */
	bcl_dev->non_monitored_mitigation_module_ids = 0;
	len = 0;
	if (of_get_property(np, "non_monitored_module_ids", &len) && len >= sizeof(u32)) {
		len /= sizeof(u32);
		bcl_dev->non_monitored_module_ids = kmalloc(sizeof(u32) * len, GFP_KERNEL);
		if (bcl_dev->non_monitored_module_ids) {
			for (i = 0; i < len; i++) {
				ret = of_property_read_u32_index(np,
						 "non_monitored_module_ids",
						 i, &bcl_dev->non_monitored_module_ids[i]);
				if (ret) {
					dev_err(bcl_dev->device,
						"failed to read non_monitored_module_id_%d\n", i);
				}
				bcl_dev->non_monitored_mitigation_module_ids |=
					BIT(bcl_dev->non_monitored_module_ids[i]);
			}
		}
	}
}

static int google_bcl_configure_gpio(struct bcl_device *bcl_dev)
{
	struct pinctrl *bcl_pinctrl;
	struct pinctrl_state *batoilo_pinctrl_state, *rffe_pinctrl_state, *s_ocp_tpu_pinctrl_state;
	int ret;

	bcl_pinctrl = devm_pinctrl_get(bcl_dev->device);
	if (IS_ERR_OR_NULL(bcl_pinctrl)) {
		dev_err(bcl_dev->device, "Cannot find bcl_pinctrl!\n");
		return -EINVAL;
	}
	batoilo_pinctrl_state = pinctrl_lookup_state(bcl_pinctrl, "bcl-batoilo-modem");
	if (IS_ERR_OR_NULL(batoilo_pinctrl_state)) {
		dev_err(bcl_dev->device, "batoilo: pinctrl lookup state failed!\n");
		return -EINVAL;
	}
	rffe_pinctrl_state = pinctrl_lookup_state(bcl_pinctrl, "bcl-rffe-modem");
	if (IS_ERR_OR_NULL(rffe_pinctrl_state)) {
		dev_err(bcl_dev->device, "rffe: pinctrl lookup state failed!\n");
		return -EINVAL;
	}
	s_ocp_tpu_pinctrl_state = pinctrl_lookup_state(bcl_pinctrl, "bcl-s-ocp-tpu");
	if (IS_ERR_OR_NULL(s_ocp_tpu_pinctrl_state)) {
		dev_err(bcl_dev->device, "tpu: pinctrl lookup state failed!\n");
		return -EINVAL;
	}
	ret = pinctrl_select_state(bcl_pinctrl, batoilo_pinctrl_state);
	if (ret < 0) {
		dev_err(bcl_dev->device, "batoilo: pinctrl select state failed!!\n");
		return -EINVAL;
	}
	ret = pinctrl_select_state(bcl_pinctrl, rffe_pinctrl_state);
	if (ret < 0) {
		dev_err(bcl_dev->device, "rffe: pinctrl select state failed!!\n");
		return -EINVAL;
	}
	ret = pinctrl_select_state(bcl_pinctrl, s_ocp_tpu_pinctrl_state);
	if (ret < 0) {
		dev_err(bcl_dev->device, "tpu: pinctrl select state failed!!\n");
		return -EINVAL;
	}
	bcl_dev->config_modem = true;
	return 0;
}
#endif

static void google_bcl_init_power_supply(struct bcl_device *bcl_dev)
{
	int ret;

	INIT_DELAYED_WORK(&bcl_dev->soc_work, google_bcl_evaluate_soc);
	bcl_dev->batt_psy = google_get_power_supply(bcl_dev, PSY_NAME);
	bcl_dev->batt_psy_initialized = false;
	bcl_dev->psy_nb.notifier_call = battery_supply_callback;
	bcl_dev->otg_psy = google_get_power_supply(bcl_dev, PSY_OTG_NAME);
	ret = power_supply_reg_notifier(&bcl_dev->psy_nb);
	if (ret < 0)
		dev_err(bcl_dev->device, "soc notifier registration error. defer. err:%d\n", ret);
	else
		bcl_dev->batt_psy_initialized = true;
	bcl_dev->soc_tz_ops.get_temp = tz_bcl_read_soc;
	bcl_dev->soc_tz_ops.set_trips = tz_bcl_set_soc;
	bcl_dev->soc_tz = devm_thermal_of_zone_register(bcl_dev->device, PMIC_SOC, bcl_dev,
							&bcl_dev->soc_tz_ops);
	if (IS_ERR(bcl_dev->soc_tz)) {
		dev_err(bcl_dev->device, "soc TZ register failed. err:%ld\n",
			PTR_ERR(bcl_dev->soc_tz));
		ret = PTR_ERR(bcl_dev->soc_tz);
		bcl_dev->soc_tz = NULL;
	} else
		thermal_zone_device_update(bcl_dev->soc_tz, THERMAL_DEVICE_UP);
}

static enum hrtimer_restart bcl_hrtimer_irq_handler(struct hrtimer *timer)
{
	panic("Kernel panic:  Sustained overcurrent detected; battery protection triggered.\n");
	return HRTIMER_NORESTART;
}

static void google_bcl_setup_timer(struct bcl_device *bcl_dev)
{
	hrtimer_init(&(bcl_dev->hr_timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bcl_dev->hr_timer.function = bcl_hrtimer_irq_handler;
}

static int google_bcl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bcl_device *bcl_dev;

	bcl_dev = devm_kzalloc(&pdev->dev, sizeof(*bcl_dev), GFP_KERNEL);
	if (IS_ERR_OR_NULL(bcl_dev))
		return -ENOMEM;

	mutex_init(&bcl_dev->sysreg_lock);
	bcl_dev->device = &pdev->dev;
	platform_set_drvdata(pdev, bcl_dev);
	google_bcl_init_power_supply(bcl_dev);

	google_bcl_parse_clk_div_dtree(bcl_dev);
	ret = google_bcl_init_instruction(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	if (google_set_main_pmic(bcl_dev) < 0)
		goto bcl_soc_probe_exit;

	if (google_set_sub_pmic(bcl_dev) < 0)
		goto bcl_soc_probe_exit;

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	google_bcl_parse_dtree(bcl_dev);
	google_bcl_configure_gpio(bcl_dev);
#endif

	if (google_set_intf_pmic(bcl_dev, pdev) < 0)
		goto bcl_soc_probe_exit;

	google_init_debugfs(bcl_dev);

	ret = google_bcl_init_data_logging(bcl_dev);
	if (ret < 0)
		goto bcl_soc_probe_exit;

	/* br_stats no need to run without mitigation app */
	bcl_dev->enabled_br_stats = false;
	bcl_dev->triggered_idx = TRIGGERED_SOURCE_MAX;

	ret = google_init_fs(bcl_dev);
	if (ret < 0)
		goto debug_fs_removal;

	ret = google_bcl_init_notifier(bcl_dev);
	if (ret < 0)
		goto debug_init_fs;

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	google_bcl_setup_votable(bcl_dev);
#endif

	if(bcl_dev->bat_ktimer_en)
		google_bcl_setup_timer(bcl_dev);

	google_bcl_clk_div(bcl_dev);
	google_bcl_parse_irq_config(bcl_dev);

	smp_store_release(&bcl_dev->enabled, true);
	dev_info(bcl_dev->device, "BCL done\n");

	return 0;

debug_init_fs:
	pmic_device_destroy(bcl_dev->mitigation_dev->devt);
debug_fs_removal:
	debugfs_remove_recursive(bcl_dev->debug_entry);
bcl_soc_probe_exit:
	google_bcl_remove_thermal(bcl_dev);
	dev_err(bcl_dev->device, "BCL SW disabled.  Revert to HW mitigation\n");
	return 0;
}

static int google_bcl_remove(struct platform_device *pdev)
{
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	pmic_device_destroy(bcl_dev->mitigation_dev->devt);
	debugfs_remove_recursive(bcl_dev->debug_entry);
	cpu_pm_unregister_notifier(&bcl_dev->cpu_nb);
	google_bcl_remove_thermal(bcl_dev);
	wakeup_source_unregister(bcl_dev->ws);

	return 0;
}

static void google_bcl_shutdown(struct platform_device *pdev)
{
	struct bcl_device *bcl_dev = platform_get_drvdata(pdev);

	if (bcl_dev)
		power_supply_unreg_notifier(&bcl_dev->psy_nb);
}

static const struct of_device_id match_table[] = {
	{ .compatible = "google,google-bcl"},
	{},
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver google_bcl_driver = {
	.probe  = google_bcl_probe,
	.remove = google_bcl_remove,
	.shutdown = google_bcl_shutdown,
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
