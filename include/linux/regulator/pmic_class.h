/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/linux/regulator/pmic_class.h
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 */
#ifndef PMIC_CLASS_H
#define PMIC_CLASS_H

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
struct device *pmic_device_create(void *drvdata, const char *fmt);
struct device *pmic_subdevice_create(struct device *parent, const struct attribute_group **groups,
				     void *drvdata, const char *fmt);
void pmic_device_destroy(dev_t devt);
#else
#define pmic_device_create(a, b) (-1)
#define pmic_device_destroy(a)                                            \
	do {                                                              \
	} while (0)
#endif

#endif /* PMIC_CLASS_H */
