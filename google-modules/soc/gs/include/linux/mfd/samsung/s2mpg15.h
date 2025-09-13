/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * s2mpg15.h
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * Driver for the s2mpg15
 */

#ifndef __S2MPG15_MFD_H__
#define __S2MPG15_MFD_H__
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/regmap.h>

#include "s2mpg15-meter.h"

#define S2MPG15_MFD_DEV_NAME "s2mpg15"

/**
 * sec_regulator_data - regulator data
 * @id: regulator id
 * @initdata: regulator init data (constraints, supplies, ...)
 */
struct s2mpg15_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

enum s2mpg15_irq_source {
	S2MPG15_IRQS_PMIC_INT1 = 0,
	S2MPG15_IRQS_PMIC_INT2,
	S2MPG15_IRQS_PMIC_INT3,
	S2MPG15_IRQS_PMIC_INT4,
	S2MPG15_IRQS_METER_INT1,
	S2MPG15_IRQS_METER_INT2,
	S2MPG15_IRQS_METER_INT3,
	S2MPG15_IRQS_METER_INT4,

	S2MPG15_IRQ_GROUP_NR,
};

#define S2MPG15_NUM_IRQ_PMIC_REGS 4
#define S2MPG15_NUM_IRQ_METER_REGS 4

#define ACPM_BULK_READ_MAX_LIMIT 8

enum s2mpg15_device_type {
	S2MPG15X,
};

enum s2mpg15_types {
	TYPE_S2MPG15,
};

struct s2mpg15_platform_data {
	/* Device Data */
	int device_type;

	/* IRQ */
	int irq_base;
	bool wakeup;

	/* VGPIO */
	u32 *sel_vgpio;

	/* Regulator */
	int num_regulators;
	struct s2mpg15_regulator_data *regulators;
	struct sec_opmode_data *opmode;

	unsigned int b2_ocp_warn_pin;
	unsigned int b2_ocp_warn_en;
	unsigned int b2_ocp_warn_cnt;
	unsigned int b2_ocp_warn_dvs_mask;
	unsigned int b2_ocp_warn_lvl;
	unsigned int b2_ocp_warn_debounce_clk;

	unsigned int b2_soft_ocp_warn_pin;
	unsigned int b2_soft_ocp_warn_en;
	unsigned int b2_soft_ocp_warn_cnt;
	unsigned int b2_soft_ocp_warn_dvs_mask;
	unsigned int b2_soft_ocp_warn_lvl;
	unsigned int b2_soft_ocp_warn_debounce_clk;

	unsigned int buck_ocp_ctrl1;
	unsigned int buck_ocp_ctrl2;
	unsigned int buck_ocp_ctrl3;
	unsigned int buck_ocp_ctrl4;
	unsigned int buck_ocp_ctrl5;
	unsigned int buck_ocp_ctrl6;
	unsigned int buck_ocp_ctrl7;
	unsigned int buck_ocp_ctrl8;

	void *meter;
	int wtsr_en;
};

struct s2mpg15_dev {
	/* Device Data */
	struct device *dev;
	struct s2mpg15_platform_data *pdata;
	struct regmap *regmap;
	int device_type;
	int type;

	/* pmic VER/REV register */
	enum S2MPG15_pmic_rev pmic_rev; /* pmic Rev */

	/* I2C Client */
	struct i2c_client *i2c;
	struct i2c_client *pmic;
	struct i2c_client *meter;
	struct i2c_client *gpio;
	struct i2c_client *wlwp;
	struct i2c_client *mt_trim;
	struct i2c_client *pm_trim1;
	struct i2c_client *pm_trim2;
	/* mutex for i2c */
	struct mutex i2c_lock;

	/* IRQ */
	int irq;
	int irq_base;
	bool wakeup;

	/* mutex for s2mpg15 irq handling */
	struct mutex irqlock;
	u8 irq_masks_cur[S2MPG15_IRQ_GROUP_NR];
	u8 irq_masks_cache[S2MPG15_IRQ_GROUP_NR];

	/* Work queue */
	struct workqueue_struct *irq_wqueue;
	struct delayed_work irq_work;
};

struct s2mpg15_pmic {
	struct s2mpg15_dev *iodev;
	struct i2c_client *i2c;

	/* mutex for s2mpg15 regulator */
	struct mutex lock;
	struct regulator_dev **rdev;
	unsigned int *opmode;
	int num_regulators;
	int wtsr_en;
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	struct device *dev;
	u16 read_addr;
#endif
};

void s2mpg15_call_notifier(void);
int s2mpg15_notifier_init(struct s2mpg15_dev *s2mpg15);

/* S2MPG15 shared i2c API function */
int s2mpg15_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest);
int s2mpg15_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf);
int s2mpg15_write_reg(struct i2c_client *i2c, u8 reg, u8 value);
int s2mpg15_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf);
int s2mpg15_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask);
typedef void (*client_exit_callback)(void);

#if  IS_ENABLED(CONFIG_GSOC_PMIC_THERMAL_CAMBRIA)
extern bool s2mpg15_spmic_thermal_ready(void);
extern int s2mpg15_spmic_set_hw_lpf(bool);
#else
static inline bool s2mpg15_spmic_thermal_ready(void)
{
	return false;
}
static inline int s2mpg15_spmic_set_hw_lpf(bool enable)
{
	return -ENOSYS;
}
#endif /* IS_ENABLED(CONFIG_GSOC_PMIC_THERMAL_CAMBRIA)*/
#endif /* __S2MPG15_MFD_H__ */
