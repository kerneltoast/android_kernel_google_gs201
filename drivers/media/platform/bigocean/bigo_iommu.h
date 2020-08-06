/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _BIGO_IOMMU_H_
#define _BIGO_IOMMU_H_

#include <linux/dma-buf.h>
#include <linux/exynos_iovmm.h>
#include <linux/ion_exynos.h>

#include "bigo_priv.h"

void bigo_unmap_all(struct bigo_inst *inst);
int bigo_map(struct bigo_core *core, struct bigo_inst *inst,
	     struct bigo_ioc_mapping *mapping);
int bigo_unmap(struct bigo_inst *inst, struct bigo_ioc_mapping *mapping);
int bigo_iommu_fault_handler(struct iommu_domain *iodmn, struct device *device,
			     unsigned long addr, int id, void *param);

#endif //_BIGO_IOMMU_H_
