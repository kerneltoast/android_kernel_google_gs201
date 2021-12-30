// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <net/sock.h>

#include "cpif_netrx_mng.h"

#define NETRX_POOL_PAGE_SIZE	32768
struct cpif_netrx_mng *cpif_create_netrx_mng(struct cpif_addr_pair *desc_addr_pair,
						u64 desc_size, u64 databuf_cp_pbase,
						u64 frag_size, u64 num_packet)
{
	struct cpif_netrx_mng *cm;
	u64 temp;
	u64 num_packet_per_page;
	u64 total_page_count;

	if (!desc_addr_pair) {
		mif_err("desc addr pair not given\n");
		return NULL;
	}

	cm = kvzalloc(sizeof(struct cpif_netrx_mng), GFP_KERNEL);
	if (cm == NULL)
		goto fail_cm;

	cm->frag_size = frag_size;
	cm->num_packet = num_packet;

	/* finding the least number of pages required for data map */
	num_packet_per_page = NETRX_POOL_PAGE_SIZE / frag_size;
	total_page_count = num_packet / num_packet_per_page + 1;
	/**
	 * total buffer size is calculated based on worst case. buffer
	 * composed of 4KB pages only
	 */
	cm->total_buf_size =  (num_packet + 100) * PAGE_SIZE;

	cm->desc_map = cpif_vmap_create(desc_addr_pair->cp_addr, desc_size, desc_size);
	if (!cm->desc_map)
		goto fail_vmap;
	cm->data_map = cpif_vmap_create(databuf_cp_pbase, cm->total_buf_size, frag_size);
	if (!cm->data_map) {
		cpif_vmap_free(cm->desc_map);
		goto fail_vmap;
	}

	/* map descriptor region in advance */
	temp = cpif_vmap_map_area(cm->desc_map, 0, 0, virt_to_phys(desc_addr_pair->ap_addr));
	if (temp != desc_addr_pair->cp_addr)
		goto fail;

	/* create recycling page array */
	cm->data_pool = cpif_page_pool_create(total_page_count, NETRX_POOL_PAGE_SIZE);
	if (unlikely(!cm->data_pool))
		goto fail;

	spin_lock_init(&cm->lock);

	/* initialize data address list */
	INIT_LIST_HEAD(&cm->data_addr_list);

	mif_info("netrx mng: num_packet: %llu frag_size: %llu total_buf_size: %llu\n",
		 cm->num_packet, cm->frag_size, cm->total_buf_size);
	mif_info("desc vmap: va_start: 0x%llX va_end: 0x%llX va_size: %llu\n",
		 cm->desc_map->va_start, cm->desc_map->va_end, cm->desc_map->va_size);
	mif_info("data vmap: va_start: 0x%llX va_end: 0x%llX va_size: %llu\n",
		 cm->data_map->va_start, cm->data_map->va_end, cm->data_map->va_size);
	mif_info("data_pool: num_pages: %d\n", cm->data_pool->rpage_arr_len);

	return cm;

fail:
	cpif_vmap_free(cm->desc_map);
	cpif_vmap_free(cm->data_map);

fail_vmap:
	kvfree(cm);

fail_cm:
	return NULL;
}
EXPORT_SYMBOL(cpif_create_netrx_mng);

void cpif_exit_netrx_mng(struct cpif_netrx_mng *cm)
{
	if (cm) {
		struct cpif_addr_pair *temp, *temp2;

		if (cm->data_pool)
			cpif_page_pool_delete(cm->data_pool);

		cpif_vmap_free(cm->desc_map);
		cpif_vmap_free(cm->data_map);
		list_for_each_entry_safe(temp, temp2, &cm->data_addr_list, addr_item) {
			list_del(&temp->addr_item);
			kfree(temp);
		}
		kvfree(cm);
	}
}
EXPORT_SYMBOL(cpif_exit_netrx_mng);

void cpif_init_netrx_mng(struct cpif_netrx_mng *cm)
{
	if (cm && cm->data_map && cm->data_pool) {
		struct cpif_addr_pair *temp, *temp2;

		list_for_each_entry_safe(temp, temp2, &cm->data_addr_list, addr_item)
			cpif_unmap_rx_buf(cm, temp->cp_addr, true);

		cpif_page_init_tmp_page(cm->data_pool);
	}
}
EXPORT_SYMBOL(cpif_init_netrx_mng);

struct cpif_addr_pair *cpif_map_rx_buf(struct cpif_netrx_mng *cm)
{
	struct page *page;
	void *data;
	u64 page_size, cp_addr;
	unsigned long flags;
	struct cpif_addr_pair *ret = NULL;
	bool used_tmp_alloc = false;

	spin_lock_irqsave(&cm->lock, flags);

	if (unlikely(!cm->data_map)) {
		mif_err_limited("data map is not created yet\n");
		goto done;
	}

	data = cpif_page_alloc(cm->data_pool, cm->frag_size, &used_tmp_alloc);
	if (!data) {
		mif_err_limited("failed to page alloc: return\n");
		goto done;
	}

	page = cpif_get_cur_page(cm->data_pool, used_tmp_alloc);
	page_size = cpif_cur_page_size(cm->data_pool, used_tmp_alloc);

	ret = kzalloc(sizeof(struct cpif_addr_pair), GFP_ATOMIC);
	if (!ret) {
		mif_err_limited("failed to kzalloc for addr_pair\n");
		goto done;
	}

	cp_addr = cpif_vmap_map_area(cm->data_map, page_to_phys(page),
				     page_size, virt_to_phys(data));
	if (!cp_addr) {  /* cp_addr cannot be allocated */
		mif_err_limited("failed to vmap and get cp_addr\n");
		kfree(ret);
		ret = NULL;
		goto done;
	}

	/* returns addr that cp is allowed to write */
	ret->cp_addr = cp_addr;
	ret->ap_addr = data;
	ret->page = page;
	ret->page_order = get_order(page_size);
	list_add_tail(&ret->addr_item, &cm->data_addr_list);

done:
	spin_unlock_irqrestore(&cm->lock, flags);

	return ret;
}
EXPORT_SYMBOL(cpif_map_rx_buf);

void *cpif_unmap_rx_buf(struct cpif_netrx_mng *cm, u64 cp_addr, bool free)
{
	unsigned long flags;
	u64 ap_paddr = 0;
	void *ap_addr = NULL;
	struct cpif_addr_pair *apair;

	spin_lock_irqsave(&cm->lock, flags);

	if (unlikely(!cm->data_map)) {
		mif_err_limited("data map does not exist\n");
		goto done;
	}

	if (cm->already_retrieved) {
		ap_addr = cm->already_retrieved;
		cm->already_retrieved = NULL;
		goto done;
	}

	ap_paddr = cpif_vmap_unmap_area(cm->data_map, cp_addr);
	if (unlikely(ap_paddr == 0)) {
		mif_err_limited("failed to receive ap_addr\n");
		goto done;
	}
	ap_addr = phys_to_virt(ap_paddr);

	apair = list_first_entry_or_null(&cm->data_addr_list, struct cpif_addr_pair, addr_item);
	if (unlikely(!apair) || ap_addr != apair->ap_addr) {
		mif_err_limited("ERR! ap_addr: %pK apair->ap_addr:%pK\n", ap_addr,
				apair ? apair->ap_addr : 0);
		ap_addr = NULL;
		goto done;
	}

	if (ap_addr && free) {
		__free_pages(apair->page, apair->page_order);
		ap_addr = NULL;
	}
	list_del(&apair->addr_item);
	kfree(apair);

done:
	spin_unlock_irqrestore(&cm->lock, flags);

	return ap_addr; /* returns NULL or unmapped AP virtual address */
}
EXPORT_SYMBOL(cpif_unmap_rx_buf);
