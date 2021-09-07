// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#include "link_device_pcie_iommu.h"

extern int pcie_iommu_map(unsigned long iova, phys_addr_t paddr, size_t size,
			  int prot, int ch_num);
extern size_t pcie_iommu_unmap(unsigned long iova, size_t size, int ch_num);

void cpif_pcie_iommu_enable_regions(struct mem_link_device *mld)
{
	static bool enabled_region;

	struct pktproc_adaptor *ppa = &mld->pktproc;
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;

	u32 cp_num = ld->mdm_data->cp_num;
	u32 shmem_idx;
	u32 size;
	int ret;

	if (enabled_region)
		return;

	for (shmem_idx = 0 ; shmem_idx < MAX_CP_SHMEM ; shmem_idx++) {
		if (shmem_idx == SHMEM_MSI)
			continue;

		if (shmem_idx == SHMEM_PKTPROC)
			size = ppa->buff_rgn_offset;
		else
			size = cp_shmem_get_size(cp_num, shmem_idx);

		if (cp_shmem_get_base(cp_num, shmem_idx)) {
			ret = pcie_iommu_map(cp_shmem_get_base(cp_num, shmem_idx),
					     cp_shmem_get_base(cp_num, shmem_idx),
					     size, 0, mc->pcie_ch_num);
			mif_info("pcie iommu idx:%d addr:0x%08lx size:0x%08x ret:%d\n",
				 shmem_idx, cp_shmem_get_base(cp_num, shmem_idx), size, ret);
		}
	}

	enabled_region = true;
}

int cpif_pcie_iommu_init(struct pktproc_queue *q)
{
	q->ioc.pf_buf = kvzalloc(sizeof(void *) * q->num_desc, GFP_KERNEL);
	if (!q->ioc.pf_buf)
		return -ENOMEM;

	return 0;
}

void cpif_pcie_iommu_deinit(struct pktproc_queue *q)
{
	kvfree(q->ioc.pf_buf);
}

void *cpif_pcie_iommu_map_va(struct pktproc_queue *q, unsigned long src_pa,
			     u32 idx, u32 *map_cnt)
{
	struct modem_ctl *mc = dev_get_drvdata(q->ppa->dev);
	struct cpif_pcie_iommu_ctrl *ioc = &q->ioc;
	const size_t pf_size = q->ppa->max_packet_size;
	void *addr_des, *addr_asc;

	/*
	 * Every page orders are compatible with IOMMU granularity.
	 * But the poped addrs are in descending order.
	 */
	addr_des = page_frag_alloc(&ioc->pf_cache, pf_size, GFP_ATOMIC);
	if (!addr_des) {
		mif_err_limited("failed to alloc page frag\n");
		return NULL;
	}

	/* Map the last page */
	*map_cnt = 0;
	if (ioc->map_page_va != ioc->pf_cache.va) {
		unsigned long map_size;
		int ret;

		if (!ioc->map_src_pa)
			goto set_map;

		map_size = page_size(virt_to_page(ioc->map_page_va));

		mif_debug("map idx:%u src_pa:0x%lX va:0x%p size:0x%lX\n",
			  ioc->map_idx, ioc->map_src_pa, ioc->map_page_va, map_size);

		ret = pcie_iommu_map(ioc->map_src_pa, virt_to_phys(ioc->map_page_va),
				     map_size, 0, mc->pcie_ch_num);
		if (ret) {
			mif_err("map failure idx:%u src_pa:0x%lX va:0x%p size:0x%X\n",
				ioc->map_idx, ioc->map_src_pa, ioc->map_page_va, map_size);
			return NULL;
		}

		/* Store the last mapping size */
		if (!idx)
			ioc->end_map_size = (u32)map_size;

		*map_cnt = circ_get_usage(q->num_desc, idx, ioc->map_idx);

set_map:
		ioc->map_src_pa = src_pa;
		ioc->map_page_va = ioc->pf_cache.va;
		ioc->map_idx = idx;
		ioc->pf_offset = 0;
	}

	/* Convert an address in accending order */
	addr_asc = ioc->pf_cache.va + ioc->pf_offset;

	ioc->pf_buf[idx] = addr_asc;
	ioc->pf_offset += pf_size;
	ioc->curr_fore = idx;

	/*
	 * The first buffer should be mapped with a new page.
	 * Drain the page frag cache at the last buffer.
	 */
	if (idx == (q->num_desc - 1)) {
		__page_frag_cache_drain(virt_to_page(ioc->pf_cache.va),
					ioc->pf_cache.pagecnt_bias);
		ioc->pf_cache.va = NULL;
	}

	return addr_asc;
}

void cpif_pcie_iommu_try_ummap_va(struct pktproc_queue *q, unsigned long src_pa,
				  void *addr, u32 idx)
{
	struct modem_ctl *mc = dev_get_drvdata(q->ppa->dev);
	struct cpif_pcie_iommu_ctrl *ioc = &q->ioc;
	struct page *p = virt_to_head_page(addr);
	u32 unmap_size;
	size_t ret;

	if (ioc->unmap_page == p)
		return;

	if (!ioc->unmap_src_pa)
		goto set_unmap;

	/*
	 * Cannot not ensure that ioc->unmap_page has a valid page addr.
	 * Use the variable to keep an addr only.
	 */

	if (!idx)
		unmap_size = ioc->end_map_size;
	else
		unmap_size = (u32)(src_pa - ioc->unmap_src_pa);

	mif_debug("unmap src_pa:0x%lX size:0x%X\n", ioc->unmap_src_pa, unmap_size);

	ret = pcie_iommu_unmap(ioc->unmap_src_pa, unmap_size, mc->pcie_ch_num);
	if (ret != unmap_size) {
		mif_err("invalid unmap size:0x%X expected:0x%X src_pa:0x%lX\n",
			ret, unmap_size, ioc->unmap_src_pa);
	}

set_unmap:
	ioc->unmap_src_pa = src_pa;
	ioc->unmap_page = p;
}
