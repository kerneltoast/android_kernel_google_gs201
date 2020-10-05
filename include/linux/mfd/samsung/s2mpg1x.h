/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/linux/mfd/samsung/s2mpg1x-meter.h
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * header file including shared functions of s2mpg1x
 */

#ifndef __LINUX_MFD_S2MPG1X_H
#define __LINUX_MFD_S2MPG1X_H

#include <linux/i2c.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>

enum s2mpg1x_id {
	ID_S2MPG10,
	ID_S2MPG11,
	ID_COUNT,
};

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

static inline int s2mpg1x_bulk_write(enum s2mpg1x_id id, struct i2c_client *i2c,
				     u8 reg, int count, u8 *buf)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, bulk_write, i2c, reg, count, buf);
	return ret;
}

#endif /* __LINUX_MFD_S2MPG1X_H */
