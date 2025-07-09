/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 Google, LLC
 *
 */

#ifndef MAX777X9_BCL_H_
#define MAX777X9_BCL_H_

#include <linux/i2c.h>

int max77759_external_reg_read(struct device *dev, uint8_t reg, uint8_t *val);
int max77759_external_reg_write(struct device *dev, uint8_t reg, uint8_t val);

#endif
