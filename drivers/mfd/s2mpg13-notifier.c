// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg13-notifier.c - Interrupt controller support for S2MPG13
 *
 * Copyright (C) 2020 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/mfd/samsung/s2mpg13-register.h>
#include <linux/notifier.h>
#include <linux/irq.h>

#ifndef TEST_DBG
#define TEST_DBG 0
#endif

static struct notifier_block sub_pmic_notifier;
static struct s2mpg13_dev *s2mpg13_global;
static u8 irq_reg[S2MPG13_IRQ_GROUP_NR] = {0};

/* BUCK 1~10, BUCK D, BUCK A, BUCK C */
static int s2mpg13_buck_ocp_cnt[S2MPG13_BUCK_MAX];

static void s2mpg13_buck_ocp_irq(struct s2mpg13_dev *s2mpg13, int buck)
{
	s2mpg13_buck_ocp_cnt[buck]++;

	dev_info(s2mpg13->dev, "%s: S2MPG13 BUCK[%d] OCP IRQ: %d\n",
		 __func__, buck + 1, s2mpg13_buck_ocp_cnt[buck]);
}

static const u8 s2mpg13_get_irq_mask[] = {
	/* OCP */
	[S2MPG13_IRQ_OCP_B1S_INT3] = S2MPG13_IRQ_OCP_B1S_MASK,
	[S2MPG13_IRQ_OCP_B2S_INT3] = S2MPG13_IRQ_OCP_B2S_MASK,
	[S2MPG13_IRQ_OCP_B3S_INT3] = S2MPG13_IRQ_OCP_B3S_MASK,
	[S2MPG13_IRQ_OCP_B4S_INT3] = S2MPG13_IRQ_OCP_B4S_MASK,
	[S2MPG13_IRQ_OCP_B5S_INT3] = S2MPG13_IRQ_OCP_B5S_MASK,
	[S2MPG13_IRQ_OCP_B6S_INT3] = S2MPG13_IRQ_OCP_B6S_MASK,
	[S2MPG13_IRQ_OCP_B7S_INT3] = S2MPG13_IRQ_OCP_B7S_MASK,
	[S2MPG13_IRQ_OCP_B8S_INT3] = S2MPG13_IRQ_OCP_B8S_MASK,
	[S2MPG13_IRQ_OCP_B9S_INT4] = S2MPG13_IRQ_OCP_B9S_MASK,
	[S2MPG13_IRQ_OCP_B10S_INT4] = S2MPG13_IRQ_OCP_B10S_MASK,
	[S2MPG13_IRQ_OCP_BDS_INT4] = S2MPG13_IRQ_OCP_BDS_MASK,
	[S2MPG13_IRQ_OCP_BAS_INT4] = S2MPG13_IRQ_OCP_BAS_MASK,
	[S2MPG13_IRQ_OCP_BCS_INT4] = S2MPG13_IRQ_OCP_BCS_MASK,
	[S2MPG13_IRQ_OCP_BBS_INT4] = S2MPG13_IRQ_OCP_BBS_MASK,
};

static void s2mpg13_call_interrupt(struct s2mpg13_dev *s2mpg13,
				   u8 pm_int1, u8 pm_int2, u8 pm_int3, u8 pm_int4,
		u8 meter_int1, u8 meter_int2, u8 meter_int3, u8 meter_int4)
{
	size_t i;
	u8 reg = 0;

	/* BUCK OCP interrupt */
	for (i = 0; i < S2MPG13_BUCK_MAX; i++) {
		reg = S2MPG13_IRQ_OCP_B1S_INT3 + i;

		if ((pm_int3 & s2mpg13_get_irq_mask[reg]) ||
		    (pm_int4 & s2mpg13_get_irq_mask[reg]))
			s2mpg13_buck_ocp_irq(s2mpg13, i);
	}

#if IS_ENABLED(TEST_DBG)
	/* BUCK-BOOST OCP interrupt */
	if (int4 & s2mpg13_get_irq_mask[S2MPG13_IRQ_OCP_BBS_INT4])
		s2mpg13_bb_ocp_irq(s2mpg13);
#endif
}

static void s2mpg13_irq_work_func(struct work_struct *work)
{
	pr_info("%s: sub pmic interrupt(0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx)\n",
		__func__, irq_reg[S2MPG13_IRQS_PMIC_INT1], irq_reg[S2MPG13_IRQS_PMIC_INT2],
		irq_reg[S2MPG13_IRQS_PMIC_INT3], irq_reg[S2MPG13_IRQS_PMIC_INT4],
		irq_reg[S2MPG13_IRQS_METER_INT1], irq_reg[S2MPG13_IRQS_METER_INT2],
		irq_reg[S2MPG13_IRQS_METER_INT3], irq_reg[S2MPG13_IRQS_METER_INT4]);
}

static int s2mpg13_notifier_handler(struct notifier_block *nb,
				    unsigned long action,
				    void *data)
{
	int ret;
	struct s2mpg13_dev *s2mpg13 = data;

	if (!s2mpg13) {
		dev_err(s2mpg13->dev, "fail to load dev.\n");
		return IRQ_HANDLED;
	}

	mutex_lock(&s2mpg13->irqlock);

	/* Read interrupt */
	ret = s2mpg13_bulk_read(s2mpg13->pmic, S2MPG13_PM_INT1,
				S2MPG13_NUM_IRQ_PMIC_REGS,
				&irq_reg[S2MPG13_IRQS_PMIC_INT1]);
	if (ret) {
		dev_err(s2mpg13->dev, "fail to read INT sources\n");
		mutex_unlock(&s2mpg13->irqlock);

		return IRQ_HANDLED;
	}

	/* Read interrupt */
	ret = s2mpg13_bulk_read(s2mpg13->pmic, S2MPG13_METER_INT1,
				S2MPG13_NUM_IRQ_METER_REGS,
				&irq_reg[S2MPG13_IRQS_METER_INT1]);
	if (ret) {
		dev_err(s2mpg13->dev, "fail to read INT sources\n");
		mutex_unlock(&s2mpg13->irqlock);

		return IRQ_HANDLED;
	}

	queue_delayed_work(s2mpg13->irq_wqueue, &s2mpg13->irq_work, 0);

	/* Call interrupt */
	s2mpg13_call_interrupt(s2mpg13, irq_reg[S2MPG13_IRQS_PMIC_INT1],
			       irq_reg[S2MPG13_IRQS_PMIC_INT2], irq_reg[S2MPG13_IRQS_PMIC_INT3],
			irq_reg[S2MPG13_IRQS_PMIC_INT4], irq_reg[S2MPG13_IRQS_METER_INT1],
			irq_reg[S2MPG13_IRQS_METER_INT2], irq_reg[S2MPG13_IRQS_METER_INT3],
			irq_reg[S2MPG13_IRQS_METER_INT4]);

	mutex_unlock(&s2mpg13->irqlock);

	return IRQ_HANDLED;
}

static BLOCKING_NOTIFIER_HEAD(s2mpg13_notifier);

static int s2mpg13_register_notifier(struct notifier_block *nb,
				     struct s2mpg13_dev *s2mpg13)
{
	int ret;

	ret = blocking_notifier_chain_register(&s2mpg13_notifier, nb);
	if (ret < 0)
		dev_err(s2mpg13->dev, "fail to register notifier\n");

	return ret;
}

void s2mpg13_call_notifier(void)
{
	blocking_notifier_call_chain(&s2mpg13_notifier, 0, s2mpg13_global);
}
EXPORT_SYMBOL(s2mpg13_call_notifier);

static const u8 s2mpg13_mask_reg[] = {
	[S2MPG13_IRQS_PMIC_INT1] = S2MPG13_PM_INT1M,
	[S2MPG13_IRQS_PMIC_INT2] = S2MPG13_PM_INT2M,
	[S2MPG13_IRQS_PMIC_INT3] = S2MPG13_PM_INT3M,
	[S2MPG13_IRQS_PMIC_INT4] = S2MPG13_PM_INT4M,
	[S2MPG13_IRQS_METER_INT1] = S2MPG13_METER_INT1M,
	[S2MPG13_IRQS_METER_INT2] = S2MPG13_METER_INT2M,
	[S2MPG13_IRQS_METER_INT3] = S2MPG13_METER_INT3M,
	[S2MPG13_IRQS_METER_INT4] = S2MPG13_METER_INT4M,
};

static int s2mpg13_unmask_interrupt(struct s2mpg13_dev *s2mpg13)
{
	int ret;
	/* unmask IRQM interrupt */
	ret = s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_COMMON_IBIM1,
				 0x00, S2MPG13_IRQSRC_MASK);
	if (ret)
		return -EIO;

	/* unmask BUCK1~10, BUCKD, BUCKA, BUCKD interrupt */
	ret = s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_PM_INT3M,
				 0x00, S2MPG13_IRQ_INT3M_MASK);
	if (ret)
		return -EIO;

	ret = s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_PM_INT4M,
				 S2MPG13_IRQ_INT4M_MASK, S2MPG13_IRQ_INT4M_MASK);
	if (ret)
		return -EIO;
#if IS_ENABLED(TEST_DBG)
	/* unmask BUCK BOOST */
	ret = s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_PM_INT4M,
				 S2MPG13_IRQ_OCP_BBS_MASK, S2MPG13_IRQ_OCP_BBS_MASK);
	if (ret)
		return -EIO;
#endif
	return 0;
}

static int s2mpg13_set_interrupt(struct s2mpg13_dev *s2mpg13)
{
	u8 i, val;
	int ret;

	/* Mask all the interrupt sources */
	for (i = 0; i < S2MPG13_IRQ_GROUP_NR; i++) {
		if (i < S2MPG13_IRQS_METER_INT1)
			ret = s2mpg13_write_reg(s2mpg13->pmic, s2mpg13_mask_reg[i], 0xFF);
		else
			ret = s2mpg13_write_reg(s2mpg13->meter, s2mpg13_mask_reg[i], 0xFF);

		if (ret)
			goto err;
	}

	ret = s2mpg13_update_reg(s2mpg13->i2c, S2MPG13_COMMON_IBIM1,
				 S2MPG13_IRQSRC_MASK, S2MPG13_IRQSRC_MASK);

	if (ret)
		goto err;

	/* Unmask interrupt sources */
	ret = s2mpg13_unmask_interrupt(s2mpg13);
	if (ret < 0) {
		dev_err(s2mpg13->dev, "Unmask interrupt fail\n");
		goto err;
	}

	/* Check unmask interrupt register */
	for (i = 0; i < S2MPG13_IRQ_GROUP_NR; i++) {
		if (i < S2MPG13_IRQS_METER_INT1)
			ret = s2mpg13_read_reg(s2mpg13->pmic, s2mpg13_mask_reg[i], &val);
		else
			ret = s2mpg13_read_reg(s2mpg13->meter, s2mpg13_mask_reg[i], &val);

		if (ret)
			goto err;

		dev_info(s2mpg13->dev, "INT%dM = 0x%02hhx\n", i + 1, val);
	}

	return 0;
err:
	return -1;
}

static int s2mpg13_set_wqueue(struct s2mpg13_dev *s2mpg13)
{
	s2mpg13->irq_wqueue = create_singlethread_workqueue("s2mpg13-wqueue");
	if (!s2mpg13->irq_wqueue) {
		dev_err(s2mpg13->dev, "fail to create workqueue\n");
		return -1;
	}

	INIT_DELAYED_WORK(&s2mpg13->irq_work, s2mpg13_irq_work_func);

	return 0;
}

static void s2mpg13_set_notifier(struct s2mpg13_dev *s2mpg13)
{
	sub_pmic_notifier.notifier_call = s2mpg13_notifier_handler;
	s2mpg13_register_notifier(&sub_pmic_notifier, s2mpg13);
}

int s2mpg13_notifier_init(struct s2mpg13_dev *s2mpg13)
{
	int ret;

	s2mpg13_global = s2mpg13;
	mutex_init(&s2mpg13->irqlock);

	/* Register notifier */
	s2mpg13_set_notifier(s2mpg13);

	/* Set workqueue */
	ret = s2mpg13_set_wqueue(s2mpg13);
	if (ret < 0) {
		dev_err(s2mpg13->dev, "s2mpg13_set_wqueue fail\n");
		return ret;
	}

	/* Set interrupt */
	ret = s2mpg13_set_interrupt(s2mpg13);
	if (ret < 0) {
		dev_err(s2mpg13->dev, "s2mpg13_set_interrupt fail\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg13_notifier_init);
