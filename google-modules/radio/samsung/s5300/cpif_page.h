/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#ifndef __CPIF_RX_PAGE_H__
#define __CPIF_RX_PAGE_H__

#include "modem_prj.h"

#define CPIF_GFP_MASK	(__GFP_NOWARN | __GFP_NORETRY | __GFP_NOMEMALLOC)

struct cpif_page {
	struct page	*page;
	bool		usable;
	int		offset;
};

struct cpif_page_pool {
	u64			page_order;
	u64			page_size;
	u64			tmp_page_size;
	struct cpif_page	**recycling_page_arr;
	struct cpif_page	*tmp_page;
	u32			rpage_arr_idx;
	u32			rpage_arr_len;
	bool			using_tmp_alloc;
};

#if IS_ENABLED(CONFIG_CPIF_PAGE_RECYCLING)
void cpif_page_pool_delete(struct cpif_page_pool *pool);
void cpif_page_init_tmp_page(struct cpif_page_pool *pool);
struct cpif_page_pool *cpif_page_pool_create(u64 num_page, u64 page_size);
struct page *cpif_get_cur_page(struct cpif_page_pool *pool, bool used_tmp_alloc);
u64 cpif_cur_page_size(struct cpif_page_pool *pool, bool used_tmp_alloc);
void *cpif_page_alloc(struct cpif_page_pool *pool, u64 alloc_size, bool *used_tmp_alloc);
#else
static inline void cpif_page_pool_delete(struct cpif_page_pool *pool) { return; }
static inline void cpif_page_init_tmp_page(struct cpif_page_pool *pool) { return; }
static inline struct cpif_page_pool *cpif_page_pool_create(u64 num_page,
					u64 page_size) { return NULL; }
static inline struct page *cpif_get_cur_page(struct cpif_page_pool *pool,
				       bool used_tmp_alloc) { return NULL; }
static inline u64 cpif_cur_page_size(struct cpif_page_pool *pool, bool used_tmp_alloc)
					{ return 0; }
static inline void *cpif_page_alloc(struct cpif_page_pool *pool, u64 alloc_size,
				    bool *used_tmp_alloc) { return NULL; }
#endif

#endif /* __CPIF_RX_PAGE_H__ */
