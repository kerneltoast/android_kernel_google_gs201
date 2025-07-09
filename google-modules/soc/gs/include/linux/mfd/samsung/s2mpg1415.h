/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2022 Samsung Electronics
 *
 * header file including shared functions of s2mpg1415
 */

#ifndef __LINUX_MFD_S2MPG1415_H
#define __LINUX_MFD_S2MPG1415_H

enum s2mpg1415_id {
	ID_S2MPG14,
	ID_S2MPG15,
	ID_COUNT,
};

#define SWITCH_ID_FUNC(id, func, args...)                                      \
	do {                                                                   \
		switch (id) {                                                  \
		case ID_S2MPG14:                                               \
			ret = s2mpg14_##func(args);                            \
			break;                                                 \
		case ID_S2MPG15:                                               \
			ret = s2mpg15_##func(args);                            \
			break;                                                 \
		default:                                                       \
			break;                                                 \
		}                                                              \
	} while (0)

#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mfd/samsung/s2mpg15.h>
static inline int s2mpg1415_update_reg(enum s2mpg1415_id id,
				       struct i2c_client *i2c,
				       u8 reg, u8 val, u8 mask)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, update_reg, i2c, reg, val, mask);
	return ret;
}

static inline int s2mpg1415_read_reg(enum s2mpg1415_id id,
				     struct i2c_client *i2c,
				     u8 reg, u8 *dest)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, read_reg, i2c, reg, dest);
	return ret;
}

static inline int s2mpg1415_write_reg(enum s2mpg1415_id id,
				      struct i2c_client *i2c,
				      u8 reg, u8 val)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, write_reg, i2c, reg, val);
	return ret;
}

static inline int s2mpg1415_bulk_write(enum s2mpg1415_id id,
				       struct i2c_client *i2c,
				       u8 reg, int count, u8 *buf)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, bulk_write, i2c, reg, count, buf);
	return ret;
}

static inline int s2mpg1415_bulk_read(enum s2mpg1415_id id,
				      struct i2c_client *i2c,
				      u8 reg, int count, u8 *buf)
{
	int ret = -1;

	SWITCH_ID_FUNC(id, bulk_read, i2c, reg, count, buf);
	return ret;
}

#endif /* __LINUX_MFD_S2MPG1415_H */
