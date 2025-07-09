/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SW Support for MAX77779 IF-PMIC
 *
 * Copyright 2023 Google, LLC
 */
#ifndef MAX77779_PMIC
#define MAX77779_PMIC

#include <linux/device.h>

#include "max77779.h"

struct max77779_pmic_info {
	struct device		*dev;
	struct regmap		*regmap;
	struct mutex		reg_dump_lock;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct dentry		*de;
	unsigned int		addr;
#endif
};

bool max77779_pmic_is_readable(struct device *dev, unsigned int reg);
int max77779_pmic_init(struct max77779_pmic_info *info);
void max77779_pmic_remove(struct max77779_pmic_info *info);
#endif
