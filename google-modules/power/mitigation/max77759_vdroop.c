/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <max77759_regs.h>
#include <max777x9_bcl.h>
#include "bcl.h"

int max77759_get_irq(struct bcl_device *bcl_dev, u8 *irq_val)
{
	u8 chg_int;
	u8 ret;
	u8 clr_bcl_irq_mask;

	clr_bcl_irq_mask = (MAX77759_CHG_INT2_BAT_OILO_I | MAX77759_CHG_INT2_SYS_UVLO1_I |
			    MAX77759_CHG_INT2_SYS_UVLO2_I);
	ret = max77759_external_reg_read(bcl_dev->intf_pmic_dev, MAX77759_CHG_INT2,
		                         &chg_int);
	if (ret < 0)
		return IRQ_NONE;
	if ((chg_int & clr_bcl_irq_mask) == 0)
		return IRQ_NONE;

	/* UVLO2 has the highest priority and then BATOILO, then UVLO1 */
	if (chg_int & MAX77759_CHG_INT2_SYS_UVLO2_I)
		*irq_val = UVLO2;
	else if (chg_int & MAX77759_CHG_INT2_BAT_OILO_I)
		*irq_val = BATOILO;
	else if (chg_int & MAX77759_CHG_INT2_SYS_UVLO1_I)
		*irq_val = UVLO1;

	return ret;
}

int max77759_clr_irq(struct bcl_device *bcl_dev, int idx)
{
	u8 irq_val = 0;
	u8 chg_int = 0;
	int ret;

	if (idx == NOT_USED)
		irq_val = idx;
	else {
		if (max77759_get_irq(bcl_dev, &irq_val) != 0)
			return IRQ_NONE;
	}
	if (irq_val == UVLO2)
		chg_int = MAX77759_CHG_INT2_SYS_UVLO2_I | MAX77759_CHG_INT2_BAT_OILO_I;
	else if (irq_val == UVLO1)
		chg_int = MAX77759_CHG_INT2_SYS_UVLO1_I;
	else if (irq_val == BATOILO)
		chg_int = MAX77759_CHG_INT2_SYS_UVLO2_I | MAX77759_CHG_INT2_BAT_OILO_I;

	ret = max77759_external_reg_write(bcl_dev->intf_pmic_dev, MAX77759_CHG_INT2,
		                          chg_int);
	if (ret < 0)
		return IRQ_NONE;
	return ret;

}

int max77759_adjust_batoilo_lvl(struct bcl_device *bcl_dev, u8 lower_enable, u8 set_batoilo1_lvl)
{
	int ret;
	u8 val, batoilo1_lvl;

	if (lower_enable)
		batoilo1_lvl = set_batoilo1_lvl;
	else
		batoilo1_lvl = bcl_dev->batt_irq_conf1.batoilo_trig_lvl;
	ret = max77759_external_reg_read(bcl_dev->intf_pmic_dev, MAX77759_CHG_CNFG_14, &val);
	if (ret < 0)
		return ret;
	val = _chg_cnfg_14_bat_oilo_set(val, batoilo1_lvl);
	return max77759_external_reg_write(bcl_dev->intf_pmic_dev, MAX77759_CHG_CNFG_14, val);
}

int max77759_vimon_read(struct bcl_device *bcl_dev)
{
	return 0;
}

int max77759_req_vimon_conv(struct bcl_device *bcl_dev, int idx)
{
	return 0;
}
