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

static void *cpif_alloc_tmp_page(struct cpif_netrx_mng *cm)
{
	struct netrx_page *tmp = cm->tmp_page;
	int ret;

	if (tmp->usable) {
		page_ref_inc(tmp->page);
	} else {
		/* new page is required */
		tmp->page = dev_alloc_pages(PAGE_FRAG_CACHE_MAX_ORDER);
		if (unlikely(!tmp->page)) {
			mif_err("failed to get page\n");
			return NULL;
		}
		cm->using_tmp_alloc = true;
		tmp->usable = true;
		tmp->offset = PAGE_FRAG_CACHE_MAX_SIZE - cm->max_packet_size;
	}

	ret = tmp->offset;
	tmp->offset -= cm->max_packet_size;
	if (tmp->offset < 0) { /* drained page, let netrx try recycle page next time */
		cm->using_tmp_alloc = false;
		tmp->usable = false;
	}

	return page_to_virt(tmp->page) + ret;
}

static void cpif_free_recycling_page_arr(struct netrx_page **rpage_arr, u32 num_page)
{
	int i;

	for (i = 0; i < num_page; i++) {
		struct netrx_page *cur = rpage_arr[i];

		if (!cur)
			continue;
		if (cur->page) {
			init_page_count(cur->page);
			__free_pages(cur->page, PAGE_FRAG_CACHE_MAX_ORDER);
		}
		kfree(cur);
	}
	kfree(rpage_arr);
}

static void *cpif_alloc_recycling_page(struct cpif_netrx_mng *cm)
{
	u32 idx = cm->rpage_arr_idx;
	struct netrx_page *cur = cm->recycling_page_arr[idx];
	u32 ret;

	if (cur->offset < 0) { /* this page cannot handle next packet */
		cur->usable = false;
		cur->offset = 0;
		if (++idx == cm->rpage_arr_len)
			cm->rpage_arr_idx = 0;
		else
			cm->rpage_arr_idx++;

		idx = cm->rpage_arr_idx;
		cur = cm->recycling_page_arr[idx];
	}

	if (page_ref_count(cur->page) == 1) { /* no one uses this page*/
		cur->offset = PAGE_FRAG_CACHE_MAX_SIZE - cm->max_packet_size;
		cur->usable = true;
		goto assign_page;
	}

	if (cur->usable == true) /* page is in use, but still has some space left */
		goto assign_page;

	/* else, the page is not ready to be used */
	mif_debug("cannot use the page: ref: %d usable: %d idx: %d\n",
			page_ref_count(cur->page), cur->usable, idx);

	return NULL;

assign_page:
	ret = cur->offset;
	cur->offset -= cm->max_packet_size;
	page_ref_inc(cur->page);

	return page_to_virt(cur->page) + ret;
}

static void *cpif_get_cur_recycling_page_base(struct cpif_netrx_mng *cm)
{
	return page_to_virt(cm->recycling_page_arr[cm->rpage_arr_idx]->page);
}

static struct netrx_page **cpif_create_recycling_page_arr(u32 num_page)
{
	int i;
	struct netrx_page **rpage_arr =
			kzalloc(sizeof(struct netrx_page *) * num_page, GFP_ATOMIC);

	if (!rpage_arr) {
		mif_err("failed to alloc recycling_page_arr\n");
		return NULL;
	}

	for (i = 0; i < num_page; i++) {
		struct netrx_page *cur = kzalloc(sizeof(struct netrx_page),
				GFP_ATOMIC);
		if (unlikely(!cur))
			goto fail;
		cur->page = dev_alloc_pages(PAGE_FRAG_CACHE_MAX_ORDER);
		if (unlikely(!cur->page)) {
			mif_err("failed to get page\n");
			cur->usable = false;
			goto fail;
		}
		cur->usable = true;
		cur->offset = 0;
		rpage_arr[i] = cur;
	}

	return rpage_arr;

fail:
	cpif_free_recycling_page_arr(rpage_arr, num_page);

	return NULL;
}

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
	cm->recycling_page_arr = cpif_create_recycling_page_arr(total_page_count * 2);
	if (unlikely(!cm->recycling_page_arr))
		goto fail;
	cm->rpage_arr_idx = 0;
	cm->rpage_arr_len = total_page_count * 2;

	/* create netrx page for temporary page allocation */
	cm->tmp_page = kzalloc(sizeof(struct netrx_page), GFP_ATOMIC);
	if (unlikely(!cm->tmp_page))
		goto fail;
	cm->tmp_page->offset = 0;
	cm->tmp_page->usable = false;

	mif_info("netrx mng: num_packet: %d max_packet_size: %d total_buf_size: %d\n",
			cm->num_packet, cm->max_packet_size, cm->total_buf_size);
	mif_info("desc vmap: va_start: 0x%lX va_end: 0x%lX va_size: %d\n",
		cm->desc_map->va_start, cm->desc_map->va_end, cm->desc_map->va_size);
	mif_info("data vmap: va_start: 0x%lX va_end: 0x%lX va_size: %d\n",
		cm->data_map->va_start, cm->data_map->va_end, cm->data_map->va_size);
	mif_info("recycling page arr: num_pages: %d\n", cm->rpage_arr_len);

	return cm;

fail:
	cpif_vmap_free(cm->desc_map);
	cpif_vmap_free(cm->data_map);

fail_vmap:
	kfree(cm);

fail_cm:
	return NULL;
}
EXPORT_SYMBOL(cpif_create_netrx_mng);

void cpif_exit_netrx_mng(struct cpif_netrx_mng *cm)
{
	if (cm) {
		cpif_free_recycling_page_arr(cm->recycling_page_arr, cm->rpage_arr_len);
		cpif_vmap_free(cm->desc_map);
		cpif_vmap_free(cm->data_map);
		kfree(cm);
	}
}
EXPORT_SYMBOL(cpif_exit_netrx_mng);

struct cpif_addr_pair cpif_map_rx_buf(struct cpif_netrx_mng *cm,
		unsigned int skb_padding_size)
{
	void *data, *page_base;
	struct cpif_addr_pair ret = {0, 0};

	if (unlikely(!cm->data_map)) {
		mif_err_limited("data map is not created yet\n");
		goto done;
	}

	if (!cm->using_tmp_alloc) {
		data = cpif_alloc_recycling_page(cm);
		if (data) {
			page_base = cpif_get_cur_recycling_page_base(cm);
			goto do_vmap;
		}
	}

	data = cpif_alloc_tmp_page(cm);
	if (!data) {
		mif_err_limited("failed to tmp page alloc: return\n");
		goto done;
	}
	page_base = page_to_virt(cm->tmp_page->page);

do_vmap:
	ret.cp_addr = cpif_vmap_map_area(cm->data_map, virt_to_phys(page_base),
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

	if (ap_addr && free) {
		page_ref_dec(phys_to_page(ap_paddr & ~0x7FFF));
		ap_addr = NULL;
	}

	return ap_addr; /* returns NULL or unmapped AP virtual address */
}
EXPORT_SYMBOL(cpif_unmap_rx_buf);
