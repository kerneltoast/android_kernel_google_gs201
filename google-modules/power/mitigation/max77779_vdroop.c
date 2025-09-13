/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <max77779.h>
#include <max77779_vimon.h>
#include <max777x9_bcl.h>
#include "bcl.h"

#define MAX77779_VIMON_BCL_CLIENT 0
#define MAX77779_VIMON_BCL_SAMPLE_COUNT 16
#define MAX77779_VIMON_CLIENT_TRIG_FOREVER -1

struct max77779_sample_data {
	uint16_t v_val;
	int16_t i_val;
} __packed;

int max77779_adjust_bat_open_to(struct bcl_device *bcl_dev, bool enable)
{
	int ret;
	u8 val;

	ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev,
					     MAX77779_BAT_OILO1_CNFG_3, &val);
	if (enable)
		val = _max77779_bat_oilo1_cnfg_3_bat_open_to_1_set(
				val, bcl_dev->batt_irq_conf1.batoilo_bat_otg_open_to);
	else
		val = _max77779_bat_oilo1_cnfg_3_bat_open_to_1_set(
				val, bcl_dev->batt_irq_conf1.batoilo_bat_open_to);
	ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev,
					      MAX77779_BAT_OILO1_CNFG_3, val);
	return ret;
}

int max77779_adjust_batoilo_lvl(struct bcl_device *bcl_dev, u8 lower_enable, u8 set_batoilo1_lvl,
                                u8 set_batoilo2_lvl)
{
	int ret;
	u8 val, batoilo1_lvl, batoilo2_lvl;

	if (lower_enable) {
		batoilo1_lvl = set_batoilo1_lvl;
		batoilo2_lvl = set_batoilo2_lvl;
	} else {
		batoilo1_lvl = bcl_dev->batt_irq_conf1.batoilo_trig_lvl;
		batoilo2_lvl = bcl_dev->batt_irq_conf2.batoilo_trig_lvl;
	}
	ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev, MAX77779_BAT_OILO1_CNFG_0, &val);
	if (ret < 0)
		return ret;
	val = _max77779_bat_oilo1_cnfg_0_bat_oilo1_set(val, batoilo1_lvl);
	ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev, MAX77779_BAT_OILO1_CNFG_0, val);
	if (ret < 0)
		return ret;
	ret = max77779_external_chg_reg_read(bcl_dev->intf_pmic_dev, MAX77779_BAT_OILO2_CNFG_0, &val);
	if (ret < 0)
		return ret;
	val = _max77779_bat_oilo2_cnfg_0_bat_oilo2_set(val, batoilo2_lvl);
	ret = max77779_external_chg_reg_write(bcl_dev->intf_pmic_dev, MAX77779_BAT_OILO2_CNFG_0, val);

	return ret;
}

int max77779_get_irq(struct bcl_device *bcl_dev, u8 *irq_val)
{
	u8 vdroop_int;
	u8 ret;
	u8 clr_bcl_irq_mask;

	clr_bcl_irq_mask = (MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK |
			    MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK |
			    MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_MASK |
			    MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK);
	ret = max77779_external_pmic_reg_read(bcl_dev->irq_pmic_dev,
		                              MAX77779_PMIC_VDROOP_INT,
					      &vdroop_int);
	if (ret < 0)
		return IRQ_NONE;
	if ((vdroop_int & clr_bcl_irq_mask) == 0)
		return IRQ_NONE;

	/* UVLO2 has the highest priority and then BATOILO, then UVLO1 */
	if (vdroop_int & MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK)
		*irq_val = UVLO2;
	else if (vdroop_int & MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK)
		*irq_val = BATOILO2;
	else if (vdroop_int & MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK)
		*irq_val = BATOILO1;
	else if (vdroop_int & MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_MASK)
		*irq_val = UVLO1;

	return ret;
}

int max77779_clr_irq(struct bcl_device *bcl_dev, int idx)
{
	u8 irq_val = 0;
	u8 chg_int = 0;
	int ret;

	if (idx != NOT_USED)
		irq_val = idx;
	else {
		if (max77779_get_irq(bcl_dev, &irq_val) != 0)
			return IRQ_NONE;
	}
	if (irq_val == UVLO2)
		chg_int = MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK |
				MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK |
				MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK;
	else if (irq_val == UVLO1)
		chg_int = MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_MASK;
	else if (irq_val == BATOILO1)
		chg_int = MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK |
				MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK |
				MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK;
	else if (irq_val == BATOILO2)
		chg_int = MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK |
				MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK |
				MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK;

	ret = max77779_external_pmic_reg_write(bcl_dev->irq_pmic_dev,
		                               MAX77779_PMIC_VDROOP_INT, chg_int);
	if (ret < 0)
		return IRQ_NONE;
	return ret;
}

int max77779_vimon_read(struct bcl_device *bcl_dev)
{
	int ret = 0;

	if (!IS_ENABLED(CONFIG_SOC_ZUMAPRO))
		return ret;

	ret = max77779_external_vimon_read_buffer(bcl_dev->vimon_dev, bcl_dev->vimon_intf.data,
						  &bcl_dev->vimon_intf.count, VIMON_BUF_SIZE);
	if (ret == 0)
		return bcl_dev->vimon_intf.count;
	return ret;
}

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
static void max77779_bcl_on_sample_ready(void *device, const enum vimon_trigger_source reason,
					 const u16 *buf, const size_t buf_size)
{
	struct bcl_device *bcl_dev = device;
	int i_max = 0;
	const int count = buf_size / sizeof(struct max77779_sample_data);
	const struct max77779_sample_data *sample = (const struct max77779_sample_data *)buf;
	int i, i_rdback;

	if (reason == VIMON_CLIENT_REQUEST)
		goto max77779_bcl_trigger_mitigation;

	for (i = 0; i < count; i++) {
		i_rdback = ((int64_t)sample->i_val * MAX77779_VIMON_NA_PER_LSB) /
			   MILLI_UNITS_TO_NANO_UNITS;

		i_max = max(i_max, i_rdback);
		sample++;
	}

	if (i_max <= bcl_dev->vimon_pwr_loop_thresh)
		return;

max77779_bcl_trigger_mitigation:
	google_pwr_loop_trigger_mitigation(bcl_dev);
}

static void max77779_bcl_on_sample_removed(void *device)
{
}

static bool max77779_bcl_extra_trigger(void *device, const uint16_t *buf, const size_t buf_size)
{
	struct bcl_device *bcl_dev = device;
	const int count = buf_size / sizeof(struct max77779_sample_data);
	const struct max77779_sample_data *sample = (const struct max77779_sample_data *)buf;
	int i, i_rdback;

	if (!bcl_dev->vimon_pwr_loop_en)
		return false;

	for (i = 0; i < count; i++) {
		i_rdback = ((int64_t)sample->i_val * MAX77779_VIMON_NA_PER_LSB) /
			   MILLI_UNITS_TO_NANO_UNITS;

		if (i_rdback > bcl_dev->vimon_pwr_loop_thresh)
			return true;

		sample++;
	}

	return false;
}

static struct vimon_client_callbacks max77779_vimon_bcl_client = {
	.on_sample_ready = max77779_bcl_on_sample_ready,
	.on_removed = max77779_bcl_on_sample_removed,
	.extra_trigger = max77779_bcl_extra_trigger,
};
#endif

int max77779_vimon_register_callback(struct bcl_device *bcl_dev)
{
	int ret = 0;

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	ret = vimon_register_callback(bcl_dev->vimon_dev, VIMON_BATOILO1_TRIGGER,
				      MAX77779_VIMON_CLIENT_TRIG_FOREVER, bcl_dev,
				      &max77779_vimon_bcl_client);
	if (ret)
		dev_err(bcl_dev->device, "bcl_vimon_client register callback failed %d\n", ret);
#endif

	return ret;
}
