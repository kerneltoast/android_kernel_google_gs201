/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _BIGO_IOMMU_H_
#define _BIGO_IOMMU_H_

#include <linux/dma-buf.h>
#include <linux/iommu.h>

#include "bigo_priv.h"

void bigo_unmap_all(struct bigo_inst *inst);
int bigo_map(struct bigo_core *core, struct bigo_inst *inst,
	     struct bigo_ioc_mapping *mapping);
int bigo_unmap(struct bigo_inst *inst, struct bigo_ioc_mapping *mapping);
int bigo_dma_sync(struct bigo_buf_sync *sync);
int bigo_iommu_fault_handler(struct iommu_fault *fault, void *param);

#endif //_BIGO_IOMMU_H_
