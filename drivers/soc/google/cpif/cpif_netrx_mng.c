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

struct cpif_netrx_mng *cpif_create_netrx_mng(struct cpif_addr_pair *desc_addr_pair,
						u64 desc_size, u64 databuf_cp_pbase,
						u64 max_packet_size, u64 num_packet)
{
	struct cpif_netrx_mng *cm;
	u64 temp;
	u64 num_packet_per_page;
	u64 total_page_count;

	u64 desc_cp_pbase = desc_addr_pair->cp_addr;
	u64 desc_ap_pbase = virt_to_phys(desc_addr_pair->ap_addr);

	if (desc_cp_pbase == 0 || desc_ap_pbase == 0 || desc_size == 0 ||
			databuf_cp_pbase == 0 || max_packet_size == 0 ||
			num_packet == 0) {
		mif_err("parameter ERR! 1: 0x%lX 2: 0x%lX 3: %d 4: 0x%lX 5: %d 6: %d\n",
				desc_cp_pbase, desc_ap_pbase, desc_size,
				databuf_cp_pbase, max_packet_size, num_packet);
		return NULL;
	}

	cm = kvzalloc(sizeof(struct cpif_netrx_mng), GFP_KERNEL);
	if (cm == NULL)
		goto fail_cm;

	cm->max_packet_size = max_packet_size;
	cm->num_packet = num_packet;

	/* deriving size of the data map : large enough to handle num_packet */
	num_packet_per_page = PAGE_FRAG_CACHE_MAX_SIZE / max_packet_size;
	total_page_count = num_packet / num_packet_per_page + 2;
	cm->total_buf_size =  PAGE_FRAG_CACHE_MAX_SIZE * total_page_count;

	cm->desc_map = cpif_vmap_create(desc_cp_pbase, desc_size, desc_size, 0);
	cm->data_map = cpif_vmap_create(databuf_cp_pbase, cm->total_buf_size,
					PAGE_FRAG_CACHE_MAX_SIZE, max_packet_size);
	if (!cm->desc_map || !cm->data_map) {
		if (cm->desc_map)
			cpif_vmap_free(cm->desc_map);
		goto fail_vmap;
	}

	/* map descriptor region in advance */
	temp = cpif_vmap_map_area(cm->desc_map, desc_ap_pbase, 0);
	if (temp != desc_cp_pbase)
		goto fail;

	/* create recycling page array */
	cm->data_pool = cpif_page_pool_create(total_page_count);
	if (unlikely(!cm->data_pool))
		goto fail;

	/* initialize data address list */
	INIT_LIST_HEAD(&cm->data_addr_list);

	mif_info("netrx mng: num_packet: %d max_packet_size: %d total_buf_size: %d\n",
			cm->num_packet, cm->max_packet_size, cm->total_buf_size);
	mif_info("desc vmap: va_start: 0x%lX va_end: 0x%lX va_size: %d\n",
		cm->desc_map->va_start, cm->desc_map->va_end, cm->desc_map->va_size);
	mif_info("data vmap: va_start: 0x%lX va_end: 0x%lX va_size: %d\n",
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
struct cpif_addr_pair *cpif_map_rx_buf(struct cpif_netrx_mng *cm)
{
	void *data, *page_base;
	struct cpif_addr_pair *ret = NULL;
	bool used_tmp_alloc = false;

	if (unlikely(!cm->data_map)) {
		mif_err_limited("data map is not created yet\n");
		goto done;
	}

	data = cpif_page_alloc(cm->data_pool, cm->max_packet_size, &used_tmp_alloc);
	if (!data) {
		mif_err_limited("failed to page alloc: return\n");
		goto done;
	}
	page_base = cpif_cur_page_base(cm->data_pool, used_tmp_alloc);

	ret = kzalloc(sizeof(struct cpif_addr_pair), GFP_ATOMIC);
	if (!ret)
		return NULL;
	ret->cp_addr = cpif_vmap_map_area(cm->data_map, virt_to_phys(page_base),
						virt_to_phys(data));
	if (ret->cp_addr == 0) {  /* cp_addr cannot be allocated */
		mif_err_limited("failed to vmap and get cp_addr\n");
		goto done;
	}

	/* returns addr that cp is allowed to write */
	ret->ap_addr = data;
	list_add_tail(&ret->addr_item, &cm->data_addr_list);
done:
	return ret;
}
EXPORT_SYMBOL(cpif_map_rx_buf);

void *cpif_unmap_rx_buf(struct cpif_netrx_mng *cm, u64 cp_addr, bool free)
{
	u64 ap_paddr = 0;
	void *ap_addr = NULL;
	struct cpif_addr_pair *apair;

	if (cm->data_map) {
		if (cm->already_retrieved) {
			ap_addr = cm->already_retrieved;
			cm->already_retrieved = NULL;
			return ap_addr;
		}

		ap_paddr = cpif_vmap_unmap_area(cm->data_map, cp_addr);
		if (unlikely(ap_paddr == 0)) {
			mif_err_limited("failed to receive ap_addr\n");
			return NULL;
		}
		ap_addr = phys_to_virt(ap_paddr);
	}

	if (ap_addr && free) {
		page_ref_dec(phys_to_page(ap_paddr & ~0x7FFF));
		ap_addr = NULL;
	}

	apair = list_first_entry_or_null(&cm->data_addr_list, struct cpif_addr_pair,
						addr_item);
	if (unlikely(!apair))
		mif_err_limited("failed to get addr pair from data addr list\n");
	else {
		list_del(&apair->addr_item);
		kfree(apair);
	}

	return ap_addr; /* returns NULL or unmapped AP virtual address */
}
EXPORT_SYMBOL(cpif_unmap_rx_buf);
