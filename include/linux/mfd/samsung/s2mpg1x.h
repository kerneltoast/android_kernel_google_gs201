/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2021 Samsung Electronics
 *
 * header file including shared functions of s2mpg1x
 */

#ifndef __LINUX_MFD_S2MPG1X_H
#define __LINUX_MFD_S2MPG1X_H

enum s2mpg1x_id {
#if IS_ENABLED(CONFIG_SOC_GS101)
	ID_S2MPG10,
	ID_S2MPG11,
#elif IS_ENABLED(CONFIG_SOC_GS201)
	ID_S2MPG12,
	ID_S2MPG13,
#endif
	ID_COUNT,
};

#if IS_ENABLED(CONFIG_SOC_GS101)
#define SWITCH_ID_FUNC(id, func, args...)                                      \
	do {                                                                   \
		switch (id) {                                                  \
		case ID_S2MPG10:                                               \
			ret = s2mpg10_##func(args);                            \
			break;                                                 \
		case ID_S2MPG11:                                               \
			ret = s2mpg11_##func(args);                            \
			break;                                                 \
		default:                                                       \
			break;                                                 \
		}                                                              \
	} while (0)

#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>
#elif IS_ENABLED(CONFIG_SOC_GS201)
#define SWITCH_ID_FUNC(id, func, args...)                                      \
	do {                                                                   \
		switch (id) {                                                  \
		case ID_S2MPG12:                                               \
			ret = s2mpg12_##func(args);                            \
			break;                                                 \
		case ID_S2MPG13:                                               \
			ret = s2mpg13_##func(args);                            \
			break;                                                 \
		default:                                                       \
			break;                                                 \
		}                                                              \
	} while (0)
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg13.h>
#endif

static inline int s2mpg1x_update_reg(enum s2mpg1x_id id, struct i2c_client *i2c,
				     u8 reg, u8 val, u8 mask)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, update_reg, i2c, reg, val, mask);
	return ret;
}

static inline int s2mpg1x_read_reg(enum s2mpg1x_id id, struct i2c_client *i2c,
				   u8 reg, u8 *dest)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, read_reg, i2c, reg, dest);
	return ret;
}

static inline int s2mpg1x_write_reg(enum s2mpg1x_id id, struct i2c_client *i2c,
				    u8 reg, u8 val)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, write_reg, i2c, reg, val);
	return ret;
}

static inline int s2mpg1x_bulk_write(enum s2mpg1x_id id, struct i2c_client *i2c,
				     u8 reg, int count, u8 *buf)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, bulk_write, i2c, reg, count, buf);
	return ret;
}

static inline int s2mpg1x_bulk_read(enum s2mpg1x_id id, struct i2c_client *i2c,
				    u8 reg, int count, u8 *buf)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, bulk_read, i2c, reg, count, buf);
	return ret;
}

#endif /* __LINUX_MFD_S2MPG1X_H */
