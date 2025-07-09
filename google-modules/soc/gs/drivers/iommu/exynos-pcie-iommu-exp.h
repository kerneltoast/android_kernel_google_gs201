/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PCIe Exynos IOMMU driver header file
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 */

#ifndef _EXYNOS_PCIE_IOMMU_EXP_H_
#define _EXYNOS_PCIE_IOMMU_EXP_H_

int pcie_iommu_map(unsigned long iova, phys_addr_t paddr, size_t size,
		   int prot, int hsi_block_num);
size_t pcie_iommu_unmap(unsigned long iova, size_t size, int hsi_block_num);

void pcie_sysmmu_set_use_iocc(int hsi_block_num);
void pcie_sysmmu_enable(int hsi_block_num);
void pcie_sysmmu_disable(int hsi_block_num);
void pcie_sysmmu_all_buff_free(int hsi_block_num);
void print_pcie_sysmmu_tlb(int hsi_block_num);

#if IS_ENABLED(CONFIG_SOC_ZUMA)
void pcie_sysmmu_dump_registers(int hsi_block_num);
#endif

#endif /* _EXYNOS_PCIE_IOMMU_EXP_H_ */
