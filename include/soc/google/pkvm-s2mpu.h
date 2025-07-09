/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __PKVM_S2MPU_H
#define __PKVM_S2MPU_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/irqreturn.h>

struct s2mpu_data {
	struct device *dev;
	void __iomem *base;
	bool pkvm_registered;
	bool always_on;
	bool has_sysmmu;
	bool has_pd;
	bool pm_ref;
};

/*
 * Parse the 's2mpus' DT property of 'parent' and create a device link
 * to all referenced S2MPU devices.
 */
int __pkvm_s2mpu_of_link(struct device *parent);
int pkvm_s2mpu_of_link(struct device *parent);
int pkvm_s2mpu_of_link_v9(struct device *parent);

/*
 * Parse the 's2mpu' DT property of 'parent' and return a pointer to
 * the referenced S2MPU device, or NULL if the property does not exist.
 */
struct device *__pkvm_s2mpu_of_parse(struct device *parent);
struct device *pkvm_s2mpu_of_parse(struct device *parent);
struct device *pkvm_s2mpu_of_parse_v9(struct device *parent);

int __pkvm_s2mpu_suspend(struct device *dev);
int pkvm_s2mpu_suspend(struct device *dev);
int pkvm_s2mpu_suspend_v9(struct device *dev);

int __pkvm_s2mpu_resume(struct device *dev);
int pkvm_s2mpu_resume(struct device *dev);
int pkvm_s2mpu_resume_v9(struct device *dev);


int pkvm_iommu_s2mpu_init(unsigned long token);
int pkvm_iommu_s2mpu_register(struct device *dev, phys_addr_t pa, u8 flags);
int pkvm_iommu_sysmmu_sync_register(struct device *dev, phys_addr_t pa,
				    struct device *parent);

static inline bool pkvm_s2mpu_ready(struct device *dev)
{
	return !!platform_get_drvdata(to_platform_device(dev));
}

irqreturn_t s2mpu_fault_handler(struct s2mpu_data *data, bool print_caches);

#endif	/* __PKVM_S2MPU_H */
