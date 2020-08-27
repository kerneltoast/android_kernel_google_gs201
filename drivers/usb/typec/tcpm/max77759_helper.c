// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020, Google LLC
 *
 * MAX77759 helper
 */
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#define I2C_RETRY 3
#define I2C_RETRY_DELAY_USLEEP_MIN 1000
#define I2C_RETRY_DELAY_USLEEP_MAX 2000

#define MAX77759_RW(operation, rmap, reg, type, val)			\
{									\
	int ret, retry;							\
									\
	ret = regmap_raw_##operation(rmap, reg, val, sizeof(type));	\
									\
	for (retry = 0; ret && retry < I2C_RETRY; retry++) {		\
		usleep_range(I2C_RETRY_DELAY_USLEEP_MIN,		\
			     I2C_RETRY_DELAY_USLEEP_MAX);		\
		ret = regmap_raw_##operation(rmap, reg, val,		\
					     sizeof(type));		\
	}								\
									\
	if (ret)							\
		printk(KERN_ERR "MAX77759 i2c error:%d reg:%d\n",	\
		       ret, reg);					\
	return ret;							\
}

#define MAX77759_UPDATE_BITS(width, rmap, reg, mask, val)		\
{									\
	u##width status;						\
	int ret;							\
									\
	ret = max77759_read##width(rmap, reg, &status);			\
	if (ret < 0)							\
		return ret;						\
									\
	return max77759_write##width(rmap, reg, (status & ~mask) |	\
				      val);				\
}

int max77759_read16(struct regmap *regmap, unsigned int reg,
		    u16  *val)
{
	MAX77759_RW(read, regmap, reg, u16, val);
}
EXPORT_SYMBOL_GPL(max77759_read16);

int max77759_write16(struct regmap *regmap, unsigned int reg,
		     u16 val)
{
	MAX77759_RW(write, regmap, reg, u16, &val);
}
EXPORT_SYMBOL_GPL(max77759_write16);

int max77759_read8(struct regmap *regmap, unsigned int reg,
		   u8 *val)
{
	MAX77759_RW(read, regmap, reg, u8, val);
}
EXPORT_SYMBOL_GPL(max77759_read8);

int max77759_write8(struct regmap *regmap, unsigned int reg,
		    u8 val)
{
	MAX77759_RW(write, regmap, reg, u8, &val);
}
EXPORT_SYMBOL_GPL(max77759_write8);

int max77759_update_bits16(struct regmap *regmap, unsigned int reg, u16 mask,
			   u16 val)
{
	MAX77759_UPDATE_BITS(16, regmap, reg, mask, val);
}
EXPORT_SYMBOL_GPL(max77759_update_bits16);

int max77759_update_bits8(struct regmap *regmap, unsigned int reg, u8 mask,
			  u8 val)
{
	MAX77759_UPDATE_BITS(8, regmap, reg, mask, val);
}
EXPORT_SYMBOL_GPL(max77759_update_bits8);

MODULE_DESCRIPTION("MAX77759_HELPER Module");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL");
