// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019, Google LLC
 *
 * MAX77759 helper
 */
#ifndef __LINUX_REGMAP_MAX77759_H
#define __LINUX_REGMAP_MAX77759_H

#include <linux/regmap.h>

#define MAX77759_LOG_REGISTER(regmap, reg, log)			\
{									\
	u8 val;								\
	int ret;							\
									\
	ret = max77759_read8(regmap, reg, &val);			\
	if (ret >= 0)							\
		logbuffer_log(log, "%s:%d " #reg " val:%u", __func__,	\
			      __LINE__,	val);				\
}

int max77759_read16(struct regmap *regmap, unsigned int reg, u16 *val);
int max77759_write16(struct regmap *regmap, unsigned int reg, u16 val);
int max77759_read8(struct regmap *regmap, unsigned int reg, u8 *val);
int max77759_write8(struct regmap *regmap, unsigned int reg, u8 val);
int max77759_update_bits16(struct regmap *regmap, unsigned int reg,
			   u16 mask, u16 val);
int max77759_update_bits8(struct regmap *regmap, unsigned int reg, u8 mask,
			  u8 val);
#endif
