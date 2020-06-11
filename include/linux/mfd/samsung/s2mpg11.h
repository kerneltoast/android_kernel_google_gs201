/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/linux/mfd/samsung/s2mpg11.h
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * Driver for the s2mpg11
 */

#ifndef __S2MPG11_MFD_H__
#define __S2MPG11_MFD_H__
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "s2mpg11-meter.h"

#define S2MPG11_MFD_DEV_NAME "s2mpg11"

/**
 * sec_regulator_data - regulator data
 * @id: regulator id
 * @initdata: regulator init data (constraints, supplies, ...)
 */
struct s2mpg11_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

#ifndef __S2MPG1X_MFD_SEC_OPMODE__
#define __S2MPG1X_MFD_SEC_OPMODE__
/*
 * sec_opmode_data - regulator operation mode data
 * @id: regulator id
 * @mode: regulator operation mode
 */
struct sec_opmode_data {
	int id;
	unsigned int mode;
};

/*
 * samsung regulator operation mode
 * SEC_OPMODE_OFF	Regulator always OFF
 * SEC_OPMODE_ON	Regulator always ON
 * SEC_OPMODE_LOWPOWER  Regulator is on in low-power mode
 * SEC_OPMODE_SUSPEND   Regulator is changed by PWREN pin
 *			If PWREN is high, regulator is on
 *			If PWREN is low, regulator is off
 */
enum sec_opmode {
	SEC_OPMODE_OFF,
	SEC_OPMODE_SUSPEND,
	SEC_OPMODE_LOWPOWER,
	SEC_OPMODE_ON,
	SEC_OPMODE_TCXO = 0x2,
	SEC_OPMODE_MIF = 0x2,
};
#endif

enum s2mpg11_irq_source {
	S2MPG11_PMIC_INT1 = 0,
	S2MPG11_PMIC_INT2,
	S2MPG11_PMIC_INT3,
	S2MPG11_PMIC_INT4,
	S2MPG11_PMIC_INT5,
	S2MPG11_PMIC_INT6,

	S2MPG11_IRQ_GROUP_NR,
};

#define S2MPG11_NUM_IRQ_PMIC_REGS 6

enum s2mpg11_device_type {
	S2MPG11X,
};

enum s2mpg11_types {
	TYPE_S2MPG11,
};

struct s2mpg11_platform_data {
	/* IRQ */
	int irq_base;
	int irq_gpio;
	bool wakeup;

	int num_regulators;
	struct s2mpg11_regulator_data *regulators;
	struct sec_opmode_data *opmode;
	struct mfd_cell *sub_devices;
	int num_subdevs;

	int device_type;
	int buck_ramp_delay;

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

	bool use_i2c_speedy;
};

struct s2mpg11_dev {
	struct device *dev;
	struct i2c_client *i2c;
	struct i2c_client *pmic;
	struct i2c_client *meter;
	struct i2c_client *trim;

	/* mutex for s2mpg11 speedy read/write */
	struct mutex i2c_lock;
	int type;
	int device_type;
	int irq;
	int irq_base;
	int irq_gpio;
	bool wakeup;

	/* mutex for s2mpg11 irq handling */
	struct mutex irqlock;
	int irq_masks_cur[S2MPG11_IRQ_GROUP_NR];
	int irq_masks_cache[S2MPG11_IRQ_GROUP_NR];

	/* pmic VER/REV register */
	u8 pmic_rev; /* pmic Rev */

	struct s2mpg11_platform_data *pdata;
	struct regmap *regmap;
};

struct s2mpg11_pmic {
	struct s2mpg11_dev *iodev;
	struct i2c_client *i2c;

	/* mutex for s2mpg11 regulator */
	struct mutex lock;
	struct regulator_dev **rdev;
	unsigned int *opmode;
	int num_regulators;
	int *buck_ocp_irq;
	int gpu_ocp_warn_irq;
	int soft_gpu_ocp_warn_irq;
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	struct device *dev;
	u8 read_val;
	u8 read_addr;
#endif
};

int s2mpg11_irq_init(struct s2mpg11_dev *s2mpg11);
void s2mpg11_irq_exit(struct s2mpg11_dev *s2mpg11);

/* S2MPG11 shared i2c API function */
int s2mpg11_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest);
int s2mpg11_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf);
int s2mpg11_write_reg(struct i2c_client *i2c, u8 reg, u8 value);
int s2mpg11_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf);
int s2mpg11_write_word(struct i2c_client *i2c, u8 reg, u16 value);
int s2mpg11_read_word(struct i2c_client *i2c, u8 reg);
int s2mpg11_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask);

#endif /* __S2MPG11_MFD_H__ */
