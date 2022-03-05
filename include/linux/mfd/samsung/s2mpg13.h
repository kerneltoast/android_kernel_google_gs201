/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * s2mpg13.h
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * Driver for the s2mpg13
 */

#ifndef __S2MPG13_MFD_H__
#define __S2MPG13_MFD_H__
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/regmap.h>

#include "s2mpg13-meter.h"

#define S2MPG13_MFD_DEV_NAME "s2mpg13"

/**
 * sec_regulator_data - regulator data
 * @id: regulator id
 * @initdata: regulator init data (constraints, supplies, ...)
 */
struct s2mpg13_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

enum s2mpg13_irq_source {
	S2MPG13_IRQS_PMIC_INT1 = 0,
	S2MPG13_IRQS_PMIC_INT2,
	S2MPG13_IRQS_PMIC_INT3,
	S2MPG13_IRQS_PMIC_INT4,
	S2MPG13_IRQS_METER_INT1,
	S2MPG13_IRQS_METER_INT2,
	S2MPG13_IRQS_METER_INT3,
	S2MPG13_IRQS_METER_INT4,

	S2MPG13_IRQ_GROUP_NR,
};

#define S2MPG13_NUM_IRQ_PMIC_REGS 4
#define S2MPG13_NUM_IRQ_METER_REGS 4

enum s2mpg13_device_type {
	S2MPG13X,
};

enum s2mpg13_types {
	TYPE_S2MPG13,
};

struct s2mpg13_platform_data {
	/* Device Data */
	int device_type;

	/* IRQ */
	int irq_base;
	bool wakeup;

	/* VGPIO */
	u32 *sel_vgpio;

	/* Regulator */
	int num_regulators;
	struct s2mpg13_regulator_data *regulators;
	struct sec_opmode_data *opmode;

	unsigned int b2_ocp_warn_pin;
	unsigned int b2_ocp_warn_en;
	unsigned int b2_ocp_warn_cnt;
	unsigned int b2_ocp_warn_dvs_mask;
	unsigned int b2_ocp_warn_lvl;

	unsigned int b2_soft_ocp_warn_pin;
	unsigned int b2_soft_ocp_warn_en;
	unsigned int b2_soft_ocp_warn_cnt;
	unsigned int b2_soft_ocp_warn_dvs_mask;
	unsigned int b2_soft_ocp_warn_lvl;

	unsigned int buck_ocp_ctrl1;
	unsigned int buck_ocp_ctrl2;
	unsigned int buck_ocp_ctrl3;
	unsigned int buck_ocp_ctrl4;
	unsigned int buck_ocp_ctrl5;
	unsigned int buck_ocp_ctrl6;
	unsigned int buck_ocp_ctrl7;

	void *meter;
};

struct s2mpg13_dev {
	/* Device Data */
	struct device *dev;
	struct s2mpg13_platform_data *pdata;
	struct regmap *regmap;
	int device_type;
	int type;

	/* pmic VER/REV register */
	enum S2MPG13_pmic_rev pmic_rev; /* pmic Rev */

	/* I2C Client */
	struct i2c_client *i2c;
	struct i2c_client *pmic;
	struct i2c_client *meter;
	struct i2c_client *trim;
	struct i2c_client *gpio;
	struct i2c_client *wlwp;
	struct i2c_client *mt_trim;
	/* mutex for i2c */
	struct mutex i2c_lock;

	/* IRQ */
	int irq;
	int irq_base;
	bool wakeup;

	/* mutex for s2mpg13 irq handling */
	struct mutex irqlock;
	u8 irq_masks_cur[S2MPG13_IRQ_GROUP_NR];
	u8 irq_masks_cache[S2MPG13_IRQ_GROUP_NR];

	/* Work queue */
	struct workqueue_struct *irq_wqueue;
	struct delayed_work irq_work;
};

struct s2mpg13_pmic {
	struct s2mpg13_dev *iodev;
	struct i2c_client *i2c;

	/* mutex for s2mpg13 regulator */
	struct mutex lock;
	struct regulator_dev **rdev;
	unsigned int *opmode;
	int num_regulators;
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	struct device *dev;
	u16 read_addr;
#endif
};

void s2mpg13_call_notifier(void);
int s2mpg13_notifier_init(struct s2mpg13_dev *s2mpg13);

/* S2MPG13 shared i2c API function */
int s2mpg13_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest);
int s2mpg13_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf);
int s2mpg13_write_reg(struct i2c_client *i2c, u8 reg, u8 value);
int s2mpg13_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf);
int s2mpg13_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask);

#endif /* __S2MPG13_MFD_H__ */
