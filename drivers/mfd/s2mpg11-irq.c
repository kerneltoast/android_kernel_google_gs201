// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/mdf/s2mpg11-irq.c
 *
 * Copyright (C) 2016 Samsung Electronics Co.Ltd
 *
 *Interrupt controller support for S2MPG11
 */

#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/wakeup_reason.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg11-register.h>

static const u8 s2mpg11_mask_reg[] = {
	/* TODO: Need to check other INTMASK */
	[S2MPG11_PMIC_INT1] = S2MPG11_PM_INT1M,
	[S2MPG11_PMIC_INT2] = S2MPG11_PM_INT2M,
	[S2MPG11_PMIC_INT3] = S2MPG11_PM_INT3M,
	[S2MPG11_PMIC_INT4] = S2MPG11_PM_INT4M,
	[S2MPG11_PMIC_INT5] = S2MPG11_PM_INT5M,
	[S2MPG11_PMIC_INT6] = S2MPG11_PM_INT6M,
};

static struct i2c_client *get_i2c(struct s2mpg11_dev *s2mpg11,
				  enum s2mpg11_irq_source src)
{
	switch (src) {
	case S2MPG11_PMIC_INT1 ... S2MPG11_PMIC_INT6:
		return s2mpg11->pmic;
	default:
		return ERR_PTR(-EINVAL);
	}
}

struct s2mpg11_irq_data {
	int mask;
	enum s2mpg11_irq_source group;
};

#define DECLARE_IRQ(idx, _group, _mask)	\
	[(idx)] = { .group = (_group), .mask = (_mask) }

static const struct s2mpg11_irq_data s2mpg11_irqs[] = {
	DECLARE_IRQ(S2MPG11_IRQ_PWRONF_INT1, S2MPG11_PMIC_INT1, 1 << 0),
	DECLARE_IRQ(S2MPG11_IRQ_PWRONR_INT1, S2MPG11_PMIC_INT1, 1 << 1),
	DECLARE_IRQ(S2MPG11_IRQ_PIF_TIMEOUTS_INT1, S2MPG11_PMIC_INT1, 1 << 4),
	DECLARE_IRQ(S2MPG11_IRQ_WTSR_INT1, S2MPG11_PMIC_INT1, 1 << 5),
	DECLARE_IRQ(S2MPG11_IRQ_SPD_PARITY_ERR_INT1, S2MPG11_PMIC_INT1, 1 << 6),
	DECLARE_IRQ(S2MPG11_IRQ_SPD_ABNORMAL_STOP_INT1, S2MPG11_PMIC_INT1,
		    1 << 7),

	DECLARE_IRQ(S2MPG11_IRQ_INT120C_INT2, S2MPG11_PMIC_INT2, 1 << 0),
	DECLARE_IRQ(S2MPG11_IRQ_INT140C_INT2, S2MPG11_PMIC_INT2, 1 << 1),
	DECLARE_IRQ(S2MPG11_IRQ_TSD_INT2, S2MPG11_PMIC_INT2, 1 << 2),
	DECLARE_IRQ(S2MPG11_IRQ_UV_BB_INT2, S2MPG11_PMIC_INT2, 1 << 3),
	DECLARE_IRQ(S2MPG11_IRQ_BB_NTR_DET_INT2, S2MPG11_PMIC_INT2, 1 << 4),
	DECLARE_IRQ(S2MPG11_IRQ_WRST_INT2, S2MPG11_PMIC_INT2, 1 << 5),
	DECLARE_IRQ(S2MPG11_IRQ_NTC_CYCLE_DONE_INT2, S2MPG11_PMIC_INT2, 1 << 6),
	DECLARE_IRQ(S2MPG11_IRQ_PMETER_OVERF_INT2, S2MPG11_PMIC_INT2, 1 << 7),

	DECLARE_IRQ(S2MPG11_IRQ_OCP_B1S_INT3, S2MPG11_PMIC_INT3, 1 << 0),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_B2S_INT3, S2MPG11_PMIC_INT3, 1 << 1),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_B3S_INT3, S2MPG11_PMIC_INT3, 1 << 2),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_B4S_INT3, S2MPG11_PMIC_INT3, 1 << 3),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_B5S_INT3, S2MPG11_PMIC_INT3, 1 << 4),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_B6S_INT3, S2MPG11_PMIC_INT3, 1 << 5),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_B7S_INT3, S2MPG11_PMIC_INT3, 1 << 6),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_B8S_INT3, S2MPG11_PMIC_INT3, 1 << 7),

	DECLARE_IRQ(S2MPG11_IRQ_OCP_B9S_INT4, S2MPG11_PMIC_INT4, 1 << 0),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_B10S_INT4, S2MPG11_PMIC_INT4, 1 << 1),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_BDS_INT4, S2MPG11_PMIC_INT4, 1 << 2),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_BAS_INT4, S2MPG11_PMIC_INT4, 1 << 3),
	DECLARE_IRQ(S2MPG11_IRQ_OCP_BBS_INT4, S2MPG11_PMIC_INT4, 1 << 4),
	DECLARE_IRQ(S2MPG11_IRQ_WLWP_ACC_INT4, S2MPG11_PMIC_INT4, 1 << 5),
	DECLARE_IRQ(S2MPG11_IRQ_SPD_SRP_PKT_RST_INT4, S2MPG11_PMIC_INT4,
		    1 << 7),

	DECLARE_IRQ(S2MPG11_IRQ_PWR_WARN_CH1_INT5, S2MPG11_PMIC_INT5, 1 << 0),
	DECLARE_IRQ(S2MPG11_IRQ_PWR_WARN_CH2_INT5, S2MPG11_PMIC_INT5, 1 << 1),
	DECLARE_IRQ(S2MPG11_IRQ_PWR_WARN_CH3_INT5, S2MPG11_PMIC_INT5, 1 << 2),
	DECLARE_IRQ(S2MPG11_IRQ_PWR_WARN_CH4_INT5, S2MPG11_PMIC_INT5, 1 << 3),
	DECLARE_IRQ(S2MPG11_IRQ_PWR_WARN_CH5_INT5, S2MPG11_PMIC_INT5, 1 << 4),
	DECLARE_IRQ(S2MPG11_IRQ_PWR_WARN_CH6_INT5, S2MPG11_PMIC_INT5, 1 << 5),
	DECLARE_IRQ(S2MPG11_IRQ_PWR_WARN_CH7_INT5, S2MPG11_PMIC_INT5, 1 << 6),
	DECLARE_IRQ(S2MPG11_IRQ_PWR_WARN_CH8_INT5, S2MPG11_PMIC_INT5, 1 << 7),

	DECLARE_IRQ(S2MPG11_IRQ_NTC_WARN_CH1_INT6, S2MPG11_PMIC_INT6, 1 << 0),
	DECLARE_IRQ(S2MPG11_IRQ_NTC_WARN_CH2_INT6, S2MPG11_PMIC_INT6, 1 << 1),
	DECLARE_IRQ(S2MPG11_IRQ_NTC_WARN_CH3_INT6, S2MPG11_PMIC_INT6, 1 << 2),
	DECLARE_IRQ(S2MPG11_IRQ_NTC_WARN_CH4_INT6, S2MPG11_PMIC_INT6, 1 << 3),
	DECLARE_IRQ(S2MPG11_IRQ_NTC_WARN_CH5_INT6, S2MPG11_PMIC_INT6, 1 << 4),
	DECLARE_IRQ(S2MPG11_IRQ_NTC_WARN_CH6_INT6, S2MPG11_PMIC_INT6, 1 << 5),
	DECLARE_IRQ(S2MPG11_IRQ_NTC_WARN_CH7_INT6, S2MPG11_PMIC_INT6, 1 << 6),
	DECLARE_IRQ(S2MPG11_IRQ_NTC_WARN_CH8_INT6, S2MPG11_PMIC_INT6, 1 << 7),
};

static void s2mpg11_irq_lock(struct irq_data *data)
{
	struct s2mpg11_dev *s2mpg11 = irq_get_chip_data(data->irq);

	mutex_lock(&s2mpg11->irqlock);
}

static void s2mpg11_irq_sync_unlock(struct irq_data *data)
{
	struct s2mpg11_dev *s2mpg11 = irq_get_chip_data(data->irq);
	int i;

	for (i = 0; i < S2MPG11_IRQ_GROUP_NR; i++) {
		u8 mask_reg = s2mpg11_mask_reg[i];
		struct i2c_client *i2c = get_i2c(s2mpg11, i);

		if (mask_reg == S2MPG11_REG_INVALID || IS_ERR_OR_NULL(i2c))
			continue;
		s2mpg11->irq_masks_cache[i] = s2mpg11->irq_masks_cur[i];

		s2mpg11_write_reg(i2c, s2mpg11_mask_reg[i],
				  s2mpg11->irq_masks_cur[i]);
	}

	mutex_unlock(&s2mpg11->irqlock);
}

static const inline struct s2mpg11_irq_data *
irq_to_s2mpg11_irq(struct s2mpg11_dev *s2mpg11, int irq)
{
	return &s2mpg11_irqs[irq - s2mpg11->irq_base];
}

static void s2mpg11_irq_mask(struct irq_data *data)
{
	struct s2mpg11_dev *s2mpg11 = irq_get_chip_data(data->irq);
	const struct s2mpg11_irq_data *irq_data =
		irq_to_s2mpg11_irq(s2mpg11, data->irq);

	if (irq_data->group >= S2MPG11_IRQ_GROUP_NR)
		return;

	s2mpg11->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void s2mpg11_irq_unmask(struct irq_data *data)
{
	struct s2mpg11_dev *s2mpg11 = irq_get_chip_data(data->irq);
	const struct s2mpg11_irq_data *irq_data =
		irq_to_s2mpg11_irq(s2mpg11, data->irq);

	if (irq_data->group >= S2MPG11_IRQ_GROUP_NR)
		return;

	s2mpg11->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static struct irq_chip s2mpg11_irq_chip = {
	.name = S2MPG11_MFD_DEV_NAME,
	.irq_bus_lock = s2mpg11_irq_lock,
	.irq_bus_sync_unlock = s2mpg11_irq_sync_unlock,
	.irq_mask = s2mpg11_irq_mask,
	.irq_unmask = s2mpg11_irq_unmask,
};

static irqreturn_t s2mpg11_irq_thread(int irq, void *data)
{
	struct s2mpg11_dev *s2mpg11 = data;
	u8 irq_reg[S2MPG11_IRQ_GROUP_NR] = { 0 };
	u8 irq_src;
	int i, ret;

	pr_debug_ratelimited("%s: irq gpio pre-state(0x%02x)\n", __func__,
			     gpio_get_value(s2mpg11->irq_gpio));

	ret = s2mpg11_read_reg(s2mpg11->i2c, S2MPG11_COMMON_INT, &irq_src);
	if (ret) {
		pr_err("%s:%s Failed to read interrupt source: %d\n",
		       S2MPG11_MFD_DEV_NAME, __func__, ret);
		return IRQ_NONE;
	}

	pr_info_ratelimited("%s: interrupt source(0x%02x)\n", __func__,
			    irq_src);

	if (irq_src & S2MPG11_IRQSRC_PMIC) {
		/* PMIC_INT */
		ret = s2mpg11_bulk_read(s2mpg11->pmic, S2MPG11_PM_INT1,
					S2MPG11_NUM_IRQ_PMIC_REGS,
					&irq_reg[S2MPG11_PMIC_INT1]);
		if (ret) {
			pr_err("%s:%s Failed to read pmic interrupt: %d\n",
			       S2MPG11_MFD_DEV_NAME, __func__, ret);
			return IRQ_NONE;
		}

		pr_info_ratelimited("%s: pmic interrupt(0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
				    __func__, irq_reg[S2MPG11_PMIC_INT1],
				    irq_reg[S2MPG11_PMIC_INT2],
				    irq_reg[S2MPG11_PMIC_INT3],
				    irq_reg[S2MPG11_PMIC_INT4],
				    irq_reg[S2MPG11_PMIC_INT5],
				    irq_reg[S2MPG11_PMIC_INT6]);
	}

	/* Apply masking */
	for (i = 0; i < S2MPG11_IRQ_GROUP_NR; i++)
		irq_reg[i] &= ~s2mpg11->irq_masks_cur[i];

	/* Report */
	for (i = 0; i < S2MPG11_IRQ_NR; i++) {
		if (irq_reg[s2mpg11_irqs[i].group] & s2mpg11_irqs[i].mask) {
			handle_nested_irq(s2mpg11->irq_base + i);
			log_threaded_irq_wakeup_reason(s2mpg11->irq_base + i,
						       s2mpg11->irq);
		}
	}

	return IRQ_HANDLED;
}

int s2mpg11_irq_init(struct s2mpg11_dev *s2mpg11)
{
	int i;
	int ret;
	u8 i2c_data;
	int cur_irq;

	if (s2mpg11->irq_gpio < 0) {
		dev_warn(s2mpg11->dev, "No interrupt specified.\n");
		s2mpg11->irq_base = 0;
		return 0;
	}

	if (!s2mpg11->irq_base) {
		dev_err(s2mpg11->dev, "No interrupt base specified.\n");
		return 0;
	}

	mutex_init(&s2mpg11->irqlock);

	s2mpg11->irq = gpio_to_irq(s2mpg11->irq_gpio);
	pr_info("%s:%s irq=%d, irq->gpio=%d\n", S2MPG11_MFD_DEV_NAME, __func__,
		s2mpg11->irq, s2mpg11->irq_gpio);

	ret = gpio_request(s2mpg11->irq_gpio, "s2mpg11_irq");
	if (ret) {
		dev_err(s2mpg11->dev, "%s: failed requesting gpio %d\n",
			__func__, s2mpg11->irq_gpio);
		return ret;
	}
	gpio_direction_input(s2mpg11->irq_gpio);
	gpio_free(s2mpg11->irq_gpio);

	/* Mask individual interrupt sources */
	for (i = 0; i < S2MPG11_IRQ_GROUP_NR; i++) {
		struct i2c_client *i2c;

		s2mpg11->irq_masks_cur[i] = 0xff;
		s2mpg11->irq_masks_cache[i] = 0xff;

		i2c = get_i2c(s2mpg11, i);

		if (IS_ERR_OR_NULL(i2c))
			continue;
		if (s2mpg11_mask_reg[i] == S2MPG11_REG_INVALID)
			continue;

		s2mpg11_write_reg(i2c, s2mpg11_mask_reg[i], 0xff);
	}

	/* Register with genirq */
	for (i = 0; i < S2MPG11_IRQ_NR; i++) {
		cur_irq = i + s2mpg11->irq_base;
		irq_set_chip_data(cur_irq, s2mpg11);
		irq_set_chip_and_handler(cur_irq, &s2mpg11_irq_chip,
					 handle_level_irq);
		irq_set_nested_thread(cur_irq, 1);
#if IS_ENABLED(CONFIG_ARM)
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}

	s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_COMMON_INT_MASK, 0xff);
	/* Unmask s2mpg11 interrupt */
	ret = s2mpg11_read_reg(s2mpg11->i2c, S2MPG11_COMMON_INT_MASK,
			       &i2c_data);
	if (ret) {
		pr_err("%s:%s fail to read intsrc mask reg\n", S2MPG11_MFD_DEV_NAME,
		       __func__);
		return ret;
	}

	i2c_data &= ~(S2MPG11_IRQSRC_PMIC); /* Unmask pmic interrupt */
	s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_COMMON_INT_MASK, i2c_data);

	pr_info("%s:%s s2mpg11_COMMON_INT_MASK=0x%02x\n", S2MPG11_MFD_DEV_NAME,
		__func__, i2c_data);

	ret = request_threaded_irq(s2mpg11->irq, NULL, s2mpg11_irq_thread,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "s2mpg11-irq", s2mpg11);

	if (ret) {
		dev_err(s2mpg11->dev, "Failed to request IRQ %d: %d\n",
			s2mpg11->irq, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg11_irq_init);

void s2mpg11_irq_exit(struct s2mpg11_dev *s2mpg11)
{
	if (s2mpg11->irq)
		free_irq(s2mpg11->irq, s2mpg11);
}
EXPORT_SYMBOL_GPL(s2mpg11_irq_exit);
