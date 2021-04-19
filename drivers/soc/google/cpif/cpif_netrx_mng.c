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

	cm = kzalloc(sizeof(struct cpif_netrx_mng), GFP_ATOMIC);
	if (cm == NULL)
		return NULL;

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
		kfree(cm);
		return NULL;
	}

	/* map descriptor region in advance */
	temp = cpif_vmap_map_area(cm->desc_map, desc_ap_pbase, 0);
	if (temp != desc_cp_pbase) {
		cpif_vmap_free(cm->desc_map);
		cpif_vmap_free(cm->data_map);
		kfree(cm);
		return NULL;
	}

	mif_info("netrx mng: num_packet: %d max_packet_size: %d total_buf_size: %d\n",
			cm->num_packet, cm->max_packet_size, cm->total_buf_size);
	mif_info("desc vmap: va_start: 0x%lX va_end: 0x%lX va_size: %d\n",
		cm->desc_map->va_start, cm->desc_map->va_end, cm->desc_map->va_size);
	mif_info("data vmap: va_start: 0x%lX va_end: 0x%lX va_size: %d\n",
		cm->data_map->va_start, cm->data_map->va_end, cm->data_map->va_size);

	return cm;
}
EXPORT_SYMBOL(cpif_create_netrx_mng);

void cpif_exit_netrx_mng(struct cpif_netrx_mng *cm)
{
	if (cm) {
		cpif_vmap_free(cm->desc_map);
		cpif_vmap_free(cm->data_map);
		kfree(cm);
	}
}
EXPORT_SYMBOL(cpif_exit_netrx_mng);

static DEFINE_PER_CPU(struct page_frag_cache, cpif_alloc_cache);

struct cpif_addr_pair cpif_map_rx_buf(struct cpif_netrx_mng *cm,
		unsigned int skb_padding_size)
{
	void *data;
	struct cpif_addr_pair ret = {0, 0};
	struct page_frag_cache *nc = this_cpu_ptr(&cpif_alloc_cache);
	gfp_t gfp_mask = GFP_ATOMIC;

	if (unlikely(!cm->data_map)) {
		mif_err_limited("data map is not created yet\n");
		goto done;
	}

	if (sk_memalloc_socks())
		gfp_mask |= __GFP_MEMALLOC;

	data = page_frag_alloc(nc, cm->max_packet_size, gfp_mask);
	if (unlikely(!data)) {
		mif_err_limited("page_frag_alloc failed\n");
		goto done;
	}

	if (unlikely(nc->size != PAGE_FRAG_CACHE_MAX_SIZE)) {
		mif_err_limited("page frag size is not 32KB, cannot control vmap\n");
		page_frag_free(nc->va);
		page_frag_free(data);
		goto done;
	}

	ret.cp_addr = cpif_vmap_map_area(cm->data_map, virt_to_phys(nc->va),
						virt_to_phys(data));
	if (ret.cp_addr == 0) {  /* cp_addr cannot be allocated */
		mif_err_limited("failed to vmap and get cp_addr\n");
		goto done;
	}

	/* returns addr that cp is allowed to write */
	ret.cp_addr += skb_padding_size;
	ret.ap_addr = (u8 *)data + skb_padding_size;
done:
	return ret;
}
EXPORT_SYMBOL(cpif_map_rx_buf);

void *cpif_unmap_rx_buf(struct cpif_netrx_mng *cm, u64 cp_addr, bool free)
{
	u64 ap_paddr = 0;
	void *ap_addr = NULL;

	if (cm->data_map) {
		ap_paddr = cpif_vmap_unmap_area(cm->data_map, cp_addr);
		if (unlikely(ap_paddr == 0)) {
			mif_err_limited("failed to receive ap_addr\n");
			return NULL;
		}
		ap_addr = phys_to_virt(ap_paddr);
	}

	if (ap_addr && free)
		page_frag_free(ap_addr);

	return ap_addr; /* returns NULL or unmapped AP virtual address */
}
EXPORT_SYMBOL(cpif_unmap_rx_buf);
