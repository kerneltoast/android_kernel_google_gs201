/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#ifndef __LINK_DEVICE_PCIE_IOMMU_H__
#define __LINK_DEVICE_PCIE_IOMMU_H__

#include "link_device_memory.h"

void cpif_pcie_iommu_enable_regions(struct mem_link_device *mld);
int cpif_pcie_iommu_init(struct pktproc_queue *q);
void cpif_pcie_iommu_reset(struct pktproc_queue *q);

void *cpif_pcie_iommu_map_va(struct pktproc_queue *q, unsigned long src_pa,
			     u32 idx, u32 *map_cnt);
void cpif_pcie_iommu_try_ummap_va(struct pktproc_queue *q, unsigned long src_pa,
				  void *addr, u32 idx);

#endif /* __LINK_DEVICE_PCIE_IOMMU_H__ */
