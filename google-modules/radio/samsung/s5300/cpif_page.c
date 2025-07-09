// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#include <linux/slab.h>
#include "cpif_page.h"

void cpif_page_pool_delete(struct cpif_page_pool *pool)
{
	int i;
	struct cpif_page **rpage_arr = pool->recycling_page_arr;
	struct cpif_page *tmp_page = pool->tmp_page;

	if (rpage_arr) {
		for (i = 0; i < pool->rpage_arr_len; i++) {
			struct cpif_page *cur = rpage_arr[i];

			if (!cur)
				continue;
			if (cur->page) {
				init_page_count(cur->page);
				__free_pages(cur->page, pool->page_order);
			}
			kvfree(cur);
		}
		kvfree(rpage_arr);
	}

	if (tmp_page) {
		if (tmp_page->page) {
			init_page_count(tmp_page->page);
			__free_pages(tmp_page->page, get_order(pool->tmp_page_size));
		}
		kvfree(tmp_page);
	}

	kvfree(pool);
	pool = NULL;
}
EXPORT_SYMBOL(cpif_page_pool_delete);

void cpif_page_init_tmp_page(struct cpif_page_pool *pool)
{
	if (pool->tmp_page) {
		pool->tmp_page->usable = false;
		pool->tmp_page->offset = 0;
		if (pool->tmp_page->page) {
			__free_pages(pool->tmp_page->page, get_order(pool->tmp_page_size));
			pool->tmp_page->page = NULL;
		}
	}
}
EXPORT_SYMBOL(cpif_page_init_tmp_page);

struct cpif_page_pool *cpif_page_pool_create(u64 num_page, u64 page_size)
{
	int i;
	struct cpif_page_pool *pool;
	struct cpif_page **rpage_arr;
	struct cpif_page *tmp_page;

	pool = kvzalloc(sizeof(struct cpif_page_pool), GFP_KERNEL);
	if (unlikely(!pool)) {
		mif_err("failed to create page pool\n");
		return NULL;
	}

	num_page *= 2; /* reserve twice as large of the least required */
	rpage_arr = kvzalloc(sizeof(struct cpif_page *) * num_page, GFP_KERNEL);
	if (unlikely(!rpage_arr)) {
		mif_err("failed to alloc recycling_page_arr\n");
		goto fail;
	}

	pool->page_size = page_size;
	pool->page_order = get_order(page_size);

	mif_info("num_page: %llu page_size: %llu page_order: %llu\n",
			num_page, page_size, pool->page_order);

	for (i = 0; i < num_page; i++) {
		struct cpif_page *cur = kvzalloc(sizeof(struct cpif_page), GFP_KERNEL);

		if (unlikely(!cur)) {
			mif_err("failed to alloc cpif_page\n");
			goto fail;
		}
		cur->page = __dev_alloc_pages(GFP_KERNEL | CPIF_GFP_MASK, pool->page_order);
		if (unlikely(!cur->page)) {
			mif_err("failed to get page\n");
			cur->usable = false;
			goto fail;
		}
		cur->usable = true;
		cur->offset = 0;
		rpage_arr[i] = cur;
	}

	tmp_page = kvzalloc(sizeof(struct cpif_page), GFP_KERNEL);
	if (unlikely(!tmp_page)) {
		mif_err("failed to allocate temporary page\n");
		goto fail;
	}
	tmp_page->offset = 0;
	tmp_page->usable = false;

	pool->recycling_page_arr = rpage_arr;
	pool->tmp_page = tmp_page;
	pool->rpage_arr_idx = 0;
	pool->rpage_arr_len = num_page;

	return pool;

fail:
	cpif_page_pool_delete(pool);
	return NULL;
}
EXPORT_SYMBOL(cpif_page_pool_create);

struct page *cpif_get_cur_page(struct cpif_page_pool *pool, bool used_tmp_alloc)
{
	if (!used_tmp_alloc)
		return pool->recycling_page_arr[pool->rpage_arr_idx]->page;
	else
		return pool->tmp_page->page;
}
EXPORT_SYMBOL(cpif_get_cur_page);

u64 cpif_cur_page_size(struct cpif_page_pool *pool, bool used_tmp_alloc)
{
	if (!used_tmp_alloc)
		return pool->page_size;
	else
		return pool->tmp_page_size;
}
EXPORT_SYMBOL(cpif_cur_page_size);

#define RECYCLING_MAX_TRIAL	100
static void *cpif_alloc_recycling_page(struct cpif_page_pool *pool, u64 alloc_size)
{
	u32 ret;
	u32 idx = pool->rpage_arr_idx;
	struct cpif_page *cur = pool->recycling_page_arr[idx];
	int retry_count = RECYCLING_MAX_TRIAL / (pool->page_order + 1);

	if (cur->offset < 0) { /* this page cannot handle next packet */
		cur->usable = false;
		cur->offset = 0;
try_next_rpage:
		if (++idx == pool->rpage_arr_len)
			pool->rpage_arr_idx = 0;
		else
			pool->rpage_arr_idx++;

		idx = pool->rpage_arr_idx;
		cur = pool->recycling_page_arr[idx];
	}

	if (page_ref_count(cur->page) == 1) { /* no one uses this page */
		cur->offset = pool->page_size - alloc_size;
		cur->usable = true;
		goto assign_page;
	}

	if (cur->usable == true) /* page is in use, but still has some space left */
		goto assign_page;

	/* else, the page is not ready to be used, need to see next one */
	if (retry_count > 0) {
		retry_count--;
		goto try_next_rpage;
	}

	return NULL;

assign_page:
	ret = cur->offset;
	cur->offset -= alloc_size;
	page_ref_inc(cur->page);

	return page_to_virt(cur->page) + ret;
}

static void *cpif_alloc_tmp_page(struct cpif_page_pool *pool, u64 alloc_size)
{
	struct cpif_page *tmp = pool->tmp_page;
	int ret;

	/* new page is required */
	if (!tmp->usable) {
		u64 page_order = pool->page_order;
		struct page *new_pg = __dev_alloc_pages(GFP_ATOMIC | CPIF_GFP_MASK, page_order);

		if (unlikely(!new_pg)) {
			if (alloc_size > PAGE_SIZE) {
				mif_err_limited("cannot alloc page for size: %llu\n",
						alloc_size);
				return NULL;
			}
			/* try PAGE_SIZE */
			new_pg = __dev_alloc_pages(GFP_ATOMIC | CPIF_GFP_MASK, 0);
			if (unlikely(!new_pg)) {
				mif_err_limited("cannot alloc new page\n");
				return NULL;
			}
			page_order = 0;
		}

		if (tmp->page) /* unref, or possibly free the page */
			__free_pages(tmp->page, get_order(pool->tmp_page_size));
		tmp->page = new_pg;
		pool->tmp_page_size = PAGE_SIZE * (1 << page_order);
		pool->using_tmp_alloc = true;
		tmp->usable = true;
		tmp->offset = pool->tmp_page_size - alloc_size;
	}

	ret = tmp->offset;
	tmp->offset -= alloc_size;
	page_ref_inc(tmp->page);
	if (tmp->offset < 0) { /* drained page, let pool try recycle page next time */
		pool->using_tmp_alloc = false;
		tmp->usable = false;
	}

	return page_to_virt(tmp->page) + ret;
}

void *cpif_page_alloc(struct cpif_page_pool *pool, u64 alloc_size, bool *used_tmp_alloc)
{
	void *ret;

	if (alloc_size > pool->page_size) {
		mif_err_limited("requested size exceeds page size. r_size: %llu p_size: %llu\n",
				alloc_size, pool->page_size);
		return NULL;
	}

	if (!pool->using_tmp_alloc) {
		ret = cpif_alloc_recycling_page(pool, alloc_size);
		if (ret) {
			*used_tmp_alloc = false;
			goto done;
		}
	}

	mif_err_limited("cannot recycle page, alloc new one\n");
	ret = cpif_alloc_tmp_page(pool, alloc_size);
	if (!ret) {
		mif_err_limited("failed to tmp page alloc: return\n");
		goto done;
	}
	*used_tmp_alloc = true;

done:
	return ret;
}
EXPORT_SYMBOL(cpif_page_alloc);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung Page Recycling driver");
