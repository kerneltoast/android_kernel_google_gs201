// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpg12-irq.c - Interrupt controller support for S2MPG12
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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/wakeup_reason.h>
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg12-register.h>

#define S2MPG12_IBI_CNT		4

static u8 irq_reg[S2MPG12_IRQ_GROUP_NR] = {0};

static const u8 s2mpg12_mask_reg[] = {
	/* TODO: Need to check other INTMASK */
	[S2MPG12_IRQS_PMIC_INT1] = S2MPG12_PM_INT1M,
	[S2MPG12_IRQS_PMIC_INT2] = S2MPG12_PM_INT2M,
	[S2MPG12_IRQS_PMIC_INT3] = S2MPG12_PM_INT3M,
	[S2MPG12_IRQS_PMIC_INT4] = S2MPG12_PM_INT4M,
	[S2MPG12_IRQS_PMIC_INT5] = S2MPG12_PM_INT5M,
	[S2MPG12_IRQS_METER_INT1] = S2MPG12_METER_INT1M,
	[S2MPG12_IRQS_METER_INT2] = S2MPG12_METER_INT2M,
};

static struct i2c_client *get_i2c(struct s2mpg12_dev *s2mpg12,
				  enum s2mpg12_irq_source src)
{
	switch (src) {
	case S2MPG12_IRQS_PMIC_INT1 ... S2MPG12_IRQS_PMIC_INT5:
		return s2mpg12->pmic;
	case S2MPG12_IRQS_METER_INT1 ... S2MPG12_IRQS_METER_INT2:
		return s2mpg12->meter;
	default:
		return ERR_PTR(-EINVAL);
	}
}

struct s2mpg12_irq_data {
	int mask;
	enum s2mpg12_irq_source group;
};

#define DECLARE_IRQ(idx, _group, _mask)	\
	[(idx)] = { .group = (_group), .mask = (_mask) }

static const struct s2mpg12_irq_data s2mpg12_irqs[] = {
	DECLARE_IRQ(S2MPG12_IRQ_PWRONF_INT1, S2MPG12_IRQS_PMIC_INT1, 1 << 0),
	DECLARE_IRQ(S2MPG12_IRQ_PWRONR_INT1, S2MPG12_IRQS_PMIC_INT1, 1 << 1),
	DECLARE_IRQ(S2MPG12_IRQ_JIGONBF_INT1, S2MPG12_IRQS_PMIC_INT1, 1 << 2),
	DECLARE_IRQ(S2MPG12_IRQ_JIGONBR_INT1, S2MPG12_IRQS_PMIC_INT1, 1 << 3),
	DECLARE_IRQ(S2MPG12_IRQ_ACOKBF_INT1, S2MPG12_IRQS_PMIC_INT1, 1 << 4),
	DECLARE_IRQ(S2MPG12_IRQ_ACOKBR_INT1, S2MPG12_IRQS_PMIC_INT1, 1 << 5),
	DECLARE_IRQ(S2MPG12_IRQ_PWRON1S_INT1, S2MPG12_IRQS_PMIC_INT1, 1 << 6),
	DECLARE_IRQ(S2MPG12_IRQ_MRB_INT1, S2MPG12_IRQS_PMIC_INT1, 1 << 7),

	DECLARE_IRQ(S2MPG12_IRQ_RTC60S_INT2, S2MPG12_IRQS_PMIC_INT2, 1 << 0),
	DECLARE_IRQ(S2MPG12_IRQ_RTCA1_INT2, S2MPG12_IRQS_PMIC_INT2, 1 << 1),
	DECLARE_IRQ(S2MPG12_IRQ_RTCA0_INT2, S2MPG12_IRQS_PMIC_INT2, 1 << 2),
	DECLARE_IRQ(S2MPG12_IRQ_RTC1S_INT2, S2MPG12_IRQS_PMIC_INT2, 1 << 3),
	DECLARE_IRQ(S2MPG12_IRQ_WTSR_COLDRST_INT2, S2MPG12_IRQS_PMIC_INT2, 1 << 4),
	DECLARE_IRQ(S2MPG12_IRQ_WTSR_INT2, S2MPG12_IRQS_PMIC_INT2, 1 << 5),
	DECLARE_IRQ(S2MPG12_IRQ_WRST_INT2, S2MPG12_IRQS_PMIC_INT2, 1 << 6),
	DECLARE_IRQ(S2MPG12_IRQ_SMPL_INT2, S2MPG12_IRQS_PMIC_INT2, 1 << 7),

	DECLARE_IRQ(S2MPG12_IRQ_120C_INT3, S2MPG12_IRQS_PMIC_INT3, 1 << 0),
	DECLARE_IRQ(S2MPG12_IRQ_140C_INT3, S2MPG12_IRQS_PMIC_INT3, 1 << 1),
	DECLARE_IRQ(S2MPG12_IRQ_TSD_INT3, S2MPG12_IRQS_PMIC_INT3, 1 << 2),
	DECLARE_IRQ(S2MPG12_IRQ_SCL_SOFTRST_INT3, S2MPG12_IRQS_PMIC_INT3, 1 << 3),
	DECLARE_IRQ(S2MPG12_IRQ_WLWP_ACC_INT3, S2MPG12_IRQS_PMIC_INT3, 1 << 7),

	DECLARE_IRQ(S2MPG12_IRQ_OCP_B1M_INT4, S2MPG12_IRQS_PMIC_INT4, 1 << 0),
	DECLARE_IRQ(S2MPG12_IRQ_OCP_B2M_INT4, S2MPG12_IRQS_PMIC_INT4, 1 << 1),
	DECLARE_IRQ(S2MPG12_IRQ_OCP_B3M_INT4, S2MPG12_IRQS_PMIC_INT4, 1 << 2),
	DECLARE_IRQ(S2MPG12_IRQ_OCP_B4M_INT4, S2MPG12_IRQS_PMIC_INT4, 1 << 3),
	DECLARE_IRQ(S2MPG12_IRQ_OCP_B5M_INT4, S2MPG12_IRQS_PMIC_INT4, 1 << 4),
	DECLARE_IRQ(S2MPG12_IRQ_OCP_B6M_INT4, S2MPG12_IRQS_PMIC_INT4, 1 << 5),
	DECLARE_IRQ(S2MPG12_IRQ_OCP_B7M_INT4, S2MPG12_IRQS_PMIC_INT4, 1 << 6),
	DECLARE_IRQ(S2MPG12_IRQ_OCP_B8M_INT4, S2MPG12_IRQS_PMIC_INT4, 1 << 7),

	DECLARE_IRQ(S2MPG12_IRQ_OCP_B9M_INT5, S2MPG12_IRQS_PMIC_INT5, 1 << 0),
	DECLARE_IRQ(S2MPG12_IRQ_OCP_B10M_INT5, S2MPG12_IRQS_PMIC_INT5, 1 << 1),
	DECLARE_IRQ(S2MPG12_IRQ_SMPL_TIMEOUT_INT5, S2MPG12_IRQS_PMIC_INT5, 1 << 5),
	DECLARE_IRQ(S2MPG12_IRQ_WTSR_TIMEOUT_INT5, S2MPG12_IRQS_PMIC_INT5, 1 << 6),

	DECLARE_IRQ(S2MPG12_IRQ_PMETER_OVERF_INT6, S2MPG12_IRQS_METER_INT1, 1 << 0),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH0_INT6, S2MPG12_IRQS_METER_INT1, 1 << 2),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH1_INT6, S2MPG12_IRQS_METER_INT1, 1 << 3),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH2_INT6, S2MPG12_IRQS_METER_INT1, 1 << 4),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH3_INT6, S2MPG12_IRQS_METER_INT1, 1 << 5),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH4_INT6, S2MPG12_IRQS_METER_INT1, 1 << 6),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH5_INT6, S2MPG12_IRQS_METER_INT1, 1 << 7),

	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH6_INT7, S2MPG12_IRQS_METER_INT2, 1 << 0),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH7_INT7, S2MPG12_IRQS_METER_INT2, 1 << 1),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH8_INT7, S2MPG12_IRQS_METER_INT2, 1 << 2),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH9_INT7, S2MPG12_IRQS_METER_INT2, 1 << 3),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH10_INT7, S2MPG12_IRQS_METER_INT2, 1 << 4),
	DECLARE_IRQ(S2MPG12_IRQ_PWR_WARN_CH11_INT7, S2MPG12_IRQS_METER_INT2, 1 << 5),
};

static void s2mpg12_irq_lock(struct irq_data *data)
{
	struct s2mpg12_dev *s2mpg12 = irq_get_chip_data(data->irq);

	mutex_lock(&s2mpg12->irqlock);
}

static void s2mpg12_irq_sync_unlock(struct irq_data *data)
{
	struct s2mpg12_dev *s2mpg12 = irq_get_chip_data(data->irq);
	int i;

	for (i = 0; i < S2MPG12_IRQ_GROUP_NR; i++) {
		u8 mask_reg = s2mpg12_mask_reg[i];
		struct i2c_client *i2c = get_i2c(s2mpg12, i);

		if (mask_reg == S2MPG12_REG_INVALID || IS_ERR_OR_NULL(i2c))
			continue;
		s2mpg12->irq_masks_cache[i] = s2mpg12->irq_masks_cur[i];

		s2mpg12_write_reg(i2c, s2mpg12_mask_reg[i],
				  s2mpg12->irq_masks_cur[i]);
	}

	mutex_unlock(&s2mpg12->irqlock);
}

static const inline struct s2mpg12_irq_data *
irq_to_s2mpg12_irq(struct s2mpg12_dev *s2mpg12, int irq)
{
	return &s2mpg12_irqs[irq - s2mpg12->irq_base];
}

static void s2mpg12_irq_mask(struct irq_data *data)
{
	struct s2mpg12_dev *s2mpg12 = irq_get_chip_data(data->irq);
	const struct s2mpg12_irq_data *irq_data =
		irq_to_s2mpg12_irq(s2mpg12, data->irq);

	if (irq_data->group >= S2MPG12_IRQ_GROUP_NR)
		return;

	s2mpg12->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void s2mpg12_irq_unmask(struct irq_data *data)
{
	struct s2mpg12_dev *s2mpg12 = irq_get_chip_data(data->irq);
	const struct s2mpg12_irq_data *irq_data =
		irq_to_s2mpg12_irq(s2mpg12, data->irq);

	if (irq_data->group >= S2MPG12_IRQ_GROUP_NR)
		return;

	s2mpg12->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static struct irq_chip s2mpg12_irq_chip = {
	.name = S2MPG12_MFD_DEV_NAME,
	.irq_bus_lock = s2mpg12_irq_lock,
	.irq_bus_sync_unlock = s2mpg12_irq_sync_unlock,
	.irq_mask = s2mpg12_irq_mask,
	.irq_unmask = s2mpg12_irq_unmask,
};

static void s2mpg12_report_irq(struct s2mpg12_dev *s2mpg12)
{
	int i;

	/* Apply masking */
	for (i = 0; i < S2MPG12_IRQ_GROUP_NR; i++)
		irq_reg[i] &= ~s2mpg12->irq_masks_cur[i];

	/* Report */
	for (i = 0; i < S2MPG12_IRQ_NR; i++) {
		if (irq_reg[s2mpg12_irqs[i].group] & s2mpg12_irqs[i].mask) {
			handle_nested_irq(s2mpg12->irq_base + i);
			log_threaded_irq_wakeup_reason(s2mpg12->irq_base + i,
						       s2mpg12->irq);
		}
	}
}

static int s2mpg12_power_key_detection(struct s2mpg12_dev *s2mpg12)
{
	int ret, i;
	u8 val;

	/* Determine falling/rising edge for PWR_ON key */
	if ((irq_reg[S2MPG12_IRQS_PMIC_INT1] & 0x3) != 0x3)
		return 0;

	ret = s2mpg12_read_reg(s2mpg12->pmic, S2MPG12_PM_STATUS1, &val);
	if (ret) {
		pr_err("%s: fail to read register\n", __func__);
		return 1;
	}
	irq_reg[S2MPG12_IRQS_PMIC_INT1] &= 0xFC;

	if (val & S2MPG12_STATUS1_PWRON) {
		irq_reg[S2MPG12_IRQS_PMIC_INT1] = S2MPG12_RISING_EDGE;
		s2mpg12_report_irq(s2mpg12);

		/* clear irq */
		for (i = 0; i < S2MPG12_IRQ_GROUP_NR; i++)
			irq_reg[i] &= 0x00;

		irq_reg[S2MPG12_IRQS_PMIC_INT1] = S2MPG12_FALLING_EDGE;
	} else {
		irq_reg[S2MPG12_IRQS_PMIC_INT1] = S2MPG12_FALLING_EDGE;
		s2mpg12_report_irq(s2mpg12);

		/* clear irq */
		for (i = 0; i < S2MPG12_IRQ_GROUP_NR; i++)
			irq_reg[i] &= 0x00;

		irq_reg[S2MPG12_IRQS_PMIC_INT1] = S2MPG12_RISING_EDGE;
	}

	return 0;
}

static void s2mpg12_irq_work_func(struct work_struct *work)
{
	pr_info("%s: main pmic interrupt(0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx, 0x%02hhx)\n",
		__func__, irq_reg[S2MPG12_IRQS_PMIC_INT1], irq_reg[S2MPG12_IRQS_PMIC_INT2],
		 irq_reg[S2MPG12_IRQS_PMIC_INT3], irq_reg[S2MPG12_IRQS_PMIC_INT4],
		 irq_reg[S2MPG12_IRQS_PMIC_INT5], irq_reg[S2MPG12_IRQS_METER_INT1],
		 irq_reg[S2MPG12_IRQS_METER_INT2]);
}

static irqreturn_t s2mpg12_irq_thread(int irq, void *data)
{
	struct s2mpg12_dev *s2mpg12 = data;
	u8 ibi_src[S2MPG12_IBI_CNT] = { 0 };
	u32 val, ibi;
	int i, ret;
	bool is_pm_irq = false;

	/* Clear interrupt pending bit */
	val = readl(s2mpg12->sysreg_pending);
	writel(val, s2mpg12->sysreg_pending);

	/* Read VGPIO_RX_MONITOR */
	val = readl(s2mpg12->mem_base);
	ibi = val;

	for (i = 0; i < S2MPG12_IBI_CNT; i++) {
		ibi_src[i] = val & 0xFF;
		val = (val >> 8);
	}

	/* notify Main PMIC */
	if (ibi_src[0] & S2MPG12_PMIC_M_MASK) {
		/* Read PMIC INT1 ~ INT5 */
		ret = s2mpg12_bulk_read(s2mpg12->pmic, S2MPG12_PM_INT1,
					S2MPG12_NUM_IRQ_PMIC_REGS,
					&irq_reg[S2MPG12_IRQS_PMIC_INT1]);
		if (ret) {
			dev_err(s2mpg12->dev, "%s Failed to read pmic interrupt: %d\n",
				S2MPG12_MFD_DEV_NAME, ret);
			return IRQ_HANDLED;
		}

		/* Read METER INT1 ~ INT2 */
		ret = s2mpg12_bulk_read(s2mpg12->meter, S2MPG12_METER_INT1,
					S2MPG12_NUM_IRQ_METER_REGS,
					&irq_reg[S2MPG12_IRQS_METER_INT1]);
		if (ret) {
			dev_err(s2mpg12->dev, "%s Failed to read pmic interrupt: %d\n",
				S2MPG12_MFD_DEV_NAME, ret);
			return IRQ_HANDLED;
		}

		queue_delayed_work(s2mpg12->irq_wqueue, &s2mpg12->irq_work, 0);

		/* Power-key W/A */
		ret = s2mpg12_power_key_detection(s2mpg12);
		if (ret)
			dev_err(s2mpg12->dev, "POWER-KEY detection error\n");

		/* Report IRQ */
		s2mpg12_report_irq(s2mpg12);
		is_pm_irq = true;
	}

	/* notify SUB PMIC */
	if (ibi_src[0] & S2MPG12_PMIC_S_MASK) {
		s2mpg13_call_notifier();
		is_pm_irq = true;
	}

	/* Log VGPIO2PMU_EINT wakeup reason with ibi */
	if (ibi != 0 && !is_pm_irq) {
		log_abnormal_wakeup_reason("VGPIO2PMU_EINT0x%08x", ibi);
	}

	return IRQ_HANDLED;
}

static int s2mpg12_set_wqueue(struct s2mpg12_dev *s2mpg12)
{
	s2mpg12->irq_wqueue = create_singlethread_workqueue("s2mpg12-wqueue");
	if (!s2mpg12->irq_wqueue) {
		pr_err("%s: fail to create workqueue\n", __func__);
		return 1;
	}

	INIT_DELAYED_WORK(&s2mpg12->irq_work, s2mpg12_irq_work_func);

	return 0;
}

int s2mpg12_irq_init(struct s2mpg12_dev *s2mpg12)
{
	int i;
	int ret;
	u8 i2c_data;
	int cur_irq;

	if (!s2mpg12->irq_base) {
		dev_err(s2mpg12->dev, "No interrupt base specified.\n");
		return 0;
	}

	mutex_init(&s2mpg12->irqlock);

	/* Set VGPIO Monitor */
	s2mpg12->mem_base = ioremap(VGPIO_I3C_BASE + VGPIO_MONITOR_ADDR, SZ_32);
	if (!s2mpg12->mem_base)
		pr_err("%s: fail to allocate VGPIO_MONITOR memory\n", __func__);

	/* Set VGPIO Monitor */
	s2mpg12->sysreg_pending = ioremap(SYSREG_VGPIO2AP + INTC0_IPEND, SZ_32);
	if (!s2mpg12->sysreg_pending)
		pr_err("%s: fail to allocate INTC0_IPEND memory\n", __func__);

	/* Set workqueue */
	s2mpg12_set_wqueue(s2mpg12);

	/* Mask individual interrupt sources */
	for (i = 0; i < S2MPG12_IRQ_GROUP_NR; i++) {
		struct i2c_client *i2c;

		s2mpg12->irq_masks_cur[i] = 0xff;
		s2mpg12->irq_masks_cache[i] = 0xff;

		i2c = get_i2c(s2mpg12, i);

		if (IS_ERR_OR_NULL(i2c))
			continue;
		if (s2mpg12_mask_reg[i] == S2MPG12_REG_INVALID)
			continue;

		s2mpg12_write_reg(i2c, s2mpg12_mask_reg[i], 0xff);
	}

	/* Register with genirq */
	for (i = 0; i < S2MPG12_IRQ_NR; i++) {
		cur_irq = i + s2mpg12->irq_base;
		irq_set_chip_data(cur_irq, s2mpg12);
		irq_set_chip_and_handler(cur_irq, &s2mpg12_irq_chip,
					 handle_level_irq);
		irq_set_nested_thread(cur_irq, 1);
#if IS_ENABLED(CONFIG_ARM)
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}

	s2mpg12_write_reg(s2mpg12->i2c, S2MPG12_COMMON_IBIM1, 0xff);
	/* Unmask s2mpg12 interrupt */
	ret = s2mpg12_read_reg(s2mpg12->i2c, S2MPG12_COMMON_IBIM1,
			       &i2c_data);
	if (ret) {
		dev_err(s2mpg12->dev, "%s fail to read intsrc mask reg\n",
			S2MPG12_MFD_DEV_NAME);
		return ret;
	}

	i2c_data &= ~(S2MPG12_IRQSRC_MASK); /* Unmask pmic interrupt */
	s2mpg12_write_reg(s2mpg12->i2c, S2MPG12_COMMON_IBIM1, i2c_data);

	dev_info(s2mpg12->dev, "%s S2MPG12_COMMON_IBIM1=0x%02x\n",
		 S2MPG12_MFD_DEV_NAME, i2c_data);

	s2mpg12->irq = irq_of_parse_and_map(s2mpg12->dev->of_node, 0);
	ret = request_threaded_irq(s2mpg12->irq, NULL, s2mpg12_irq_thread,
				   IRQF_ONESHOT,
				   "s2mpg12-irq", s2mpg12);

	if (ret) {
		dev_err(s2mpg12->dev, "Failed to request IRQ %d: %d\n",
			s2mpg12->irq, ret);
		destroy_workqueue(s2mpg12->irq_wqueue);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg12_irq_init);

void s2mpg12_irq_exit(struct s2mpg12_dev *s2mpg12)
{
	if (s2mpg12->irq)
		free_irq(s2mpg12->irq, s2mpg12);

	iounmap(s2mpg12->mem_base);

	cancel_delayed_work_sync(&s2mpg12->irq_work);
	destroy_workqueue(s2mpg12->irq_wqueue);
}
EXPORT_SYMBOL_GPL(s2mpg12_irq_exit);
