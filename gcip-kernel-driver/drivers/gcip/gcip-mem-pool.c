// SPDX-License-Identifier: GPL-2.0-only
/*
 * A simple memory allocator to help allocating reserved memory pools.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/device.h>
#include <linux/genalloc.h>
#include <linux/log2.h>
#include <linux/types.h>

#include <gcip/gcip-mem-pool.h>

int gcip_mem_pool_init(struct gcip_mem_pool *pool, struct device *dev, unsigned long base_addr,
		       size_t size, size_t granule)
{
	int ret;

	if (!base_addr || granule == 0)
		return -EINVAL;
	if (base_addr % granule || size % granule)
		return -EINVAL;
	pool->gen_pool = gen_pool_create(ilog2(granule), -1);
	if (!pool->gen_pool) {
		dev_err(dev, "gcip memory pool allocate gen_pool failed");
		return -ENOMEM;
	}
	ret = gen_pool_add(pool->gen_pool, base_addr, size, -1);
	if (ret) {
		gen_pool_destroy(pool->gen_pool);
		pool->gen_pool = NULL;
		dev_err(dev, "gcip failed to add memory to mem pool: %d", ret);
		return ret;
	}
	pool->dev = dev;
	pool->granule = granule;
	pool->base_addr = base_addr;
	return 0;
}

void gcip_mem_pool_exit(struct gcip_mem_pool *pool)
{
	if (!pool->gen_pool)
		return;
	gen_pool_destroy(pool->gen_pool);
	pool->gen_pool = NULL;
}

unsigned long gcip_mem_pool_alloc(struct gcip_mem_pool *pool, size_t size)
{
	unsigned long addr;

	addr = gen_pool_alloc(pool->gen_pool, size);
	if (!addr)
		return 0;
	dev_dbg(pool->dev, "%s @ size = %#zx addr=%#lx", __func__, size, addr);
	return addr;
}

void gcip_mem_pool_free(struct gcip_mem_pool *pool, unsigned long addr, size_t size)
{
	dev_dbg(pool->dev, "%s @ size = %#zx addr=%#lx", __func__, size, addr);
	size = ALIGN(size, pool->granule);
	gen_pool_free(pool->gen_pool, addr, size);
}
