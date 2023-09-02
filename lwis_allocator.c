/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google LWIS Recycling Memory Allocator
 *
 * Copyright (c) 2021 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-allocator: " fmt

#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/preempt.h>
#include <linux/slab.h>
#include "lwis_allocator.h"
#include "lwis_commands.h"

static void allocator_block_pool_free_locked(struct lwis_device *lwis_dev,
					     struct lwis_allocator_block_pool *block_pool)
{
	struct lwis_allocator_block_mgr *block_mgr = lwis_dev->block_mgr;

	if (block_pool == NULL) {
		dev_err(lwis_dev->dev, "block_pool is NULL\n");
		return;
	}
	if (block_pool->in_use_count != 0 || block_pool->in_use != NULL) {
		dev_err(lwis_dev->dev, "block_pool %s still has %d block(s) in use\n",
			block_pool->name, block_pool->in_use_count);
	}

	while (block_pool->free != NULL) {
		struct lwis_allocator_block *curr;
		struct lwis_allocator_block *block;
		struct hlist_node *n;
		int i;

		curr = block_pool->free;
		hash_for_each_safe (block_mgr->allocated_blocks, i, n, block, node) {
			if (block->ptr == curr->ptr) {
				hash_del(&block->node);
			}
		}
		block_pool->free = curr->next;
		block_pool->free_count--;
		kvfree(curr->ptr);
		kfree(curr);
	}
}

static struct lwis_allocator_block *
allocator_free_block_get_locked(struct lwis_allocator_block_pool *block_pool)
{
	struct lwis_allocator_block *head;

	if (block_pool == NULL) {
		pr_err("block_pool is NULL\n");
		return NULL;
	}
	if (block_pool->free == NULL) {
		return NULL;
	}

	head = block_pool->free;
	block_pool->free = head->next;
	if (block_pool->free != NULL) {
		block_pool->free->prev = NULL;
	}
	block_pool->free_count--;

	head->next = block_pool->in_use;
	if (head->next != NULL) {
		head->next->prev = head;
	}
	block_pool->in_use = head;
	block_pool->in_use_count++;

	return head;
}

static void allocator_free_block_put_locked(struct lwis_allocator_block_pool *block_pool,
					    struct lwis_allocator_block *block)
{
	if (block_pool == NULL) {
		pr_err("block_pool is NULL\n");
		return;
	}
	if (block == NULL) {
		pr_err("block is NULL\n");
		return;
	}

	if (block->next != NULL) {
		block->next->prev = block->prev;
	}
	if (block->prev != NULL) {
		block->prev->next = block->next;
	} else {
		block_pool->in_use = block->next;
	}
	block_pool->in_use_count--;

	if (block_pool->free != NULL) {
		block_pool->free->prev = block;
	}
	block->next = block_pool->free;
	block->prev = NULL;
	block_pool->free = block;
	block_pool->free_count++;
}

static struct lwis_allocator_block_pool *
allocator_get_block_pool(struct lwis_allocator_block_mgr *block_mgr, int idx)
{
	struct lwis_allocator_block_pool *block_pool;

	switch (idx) {
	case 13:
		block_pool = &block_mgr->pool_8k;
		break;
	case 14:
		block_pool = &block_mgr->pool_16k;
		break;
	case 15:
		block_pool = &block_mgr->pool_32k;
		break;
	case 16:
		block_pool = &block_mgr->pool_64k;
		break;
	case 17:
		block_pool = &block_mgr->pool_128k;
		break;
	case 18:
		block_pool = &block_mgr->pool_256k;
		break;
	case 19:
		block_pool = &block_mgr->pool_512k;
		break;
	default:
		pr_err("size is not supportted\n");
		return NULL;
	}

	return block_pool;
}

int lwis_allocator_init(struct lwis_device *lwis_dev)
{
	struct lwis_allocator_block_mgr *block_mgr;

	if (lwis_dev == NULL) {
		dev_err(lwis_dev->dev, "lwis_dev is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&lwis_dev->client_lock);

	if (lwis_dev->block_mgr != NULL) {
		block_mgr = lwis_dev->block_mgr;
		block_mgr->ref_count++;
		mutex_unlock(&lwis_dev->client_lock);
		return 0;
	}

	block_mgr = kzalloc(sizeof(struct lwis_allocator_block_mgr), GFP_KERNEL);
	if (block_mgr == NULL) {
		dev_err(lwis_dev->dev, "Allocate block_mgr failed\n");
		mutex_unlock(&lwis_dev->client_lock);
		return -ENOMEM;
	}

	/* Initialize mutex */
	spin_lock_init(&block_mgr->lock);

	/* Empty hash table for allocated blocks */
	hash_init(block_mgr->allocated_blocks);

	/* Initialize block pools */
	strlcpy(block_mgr->pool_8k.name, "lwis-block-8k", LWIS_MAX_NAME_STRING_LEN);
	strlcpy(block_mgr->pool_16k.name, "lwis-block-16k", LWIS_MAX_NAME_STRING_LEN);
	strlcpy(block_mgr->pool_32k.name, "lwis-block-32k", LWIS_MAX_NAME_STRING_LEN);
	strlcpy(block_mgr->pool_64k.name, "lwis-block-64k", LWIS_MAX_NAME_STRING_LEN);
	strlcpy(block_mgr->pool_128k.name, "lwis-block-128k", LWIS_MAX_NAME_STRING_LEN);
	strlcpy(block_mgr->pool_256k.name, "lwis-block-256k", LWIS_MAX_NAME_STRING_LEN);
	strlcpy(block_mgr->pool_512k.name, "lwis-block-512k", LWIS_MAX_NAME_STRING_LEN);
	strlcpy(block_mgr->pool_large.name, "lwis-block-large", LWIS_MAX_NAME_STRING_LEN);

	/* Initialize reference count */
	block_mgr->ref_count = 1;

	lwis_dev->block_mgr = block_mgr;
	mutex_unlock(&lwis_dev->client_lock);
	return 0;
}

void lwis_allocator_release(struct lwis_device *lwis_dev)
{
	struct lwis_allocator_block_mgr *block_mgr;

	if (lwis_dev == NULL) {
		dev_err(lwis_dev->dev, "lwis_dev is NULL\n");
		return;
	}

	mutex_lock(&lwis_dev->client_lock);

	block_mgr = lwis_dev->block_mgr;
	if (block_mgr == NULL) {
		dev_err(lwis_dev->dev, "block_mgr is NULL\n");
		mutex_unlock(&lwis_dev->client_lock);
		return;
	}

	block_mgr->ref_count--;
	if (block_mgr->ref_count > 0) {
		mutex_unlock(&lwis_dev->client_lock);
		return;
	}

	allocator_block_pool_free_locked(lwis_dev, &block_mgr->pool_8k);
	allocator_block_pool_free_locked(lwis_dev, &block_mgr->pool_16k);
	allocator_block_pool_free_locked(lwis_dev, &block_mgr->pool_32k);
	allocator_block_pool_free_locked(lwis_dev, &block_mgr->pool_64k);
	allocator_block_pool_free_locked(lwis_dev, &block_mgr->pool_128k);
	allocator_block_pool_free_locked(lwis_dev, &block_mgr->pool_256k);
	allocator_block_pool_free_locked(lwis_dev, &block_mgr->pool_512k);

	kfree(block_mgr);
	lwis_dev->block_mgr = NULL;
	mutex_unlock(&lwis_dev->client_lock);
}

void *lwis_allocator_allocate(struct lwis_device *lwis_dev, size_t size)
{
	struct lwis_allocator_block_mgr *block_mgr;
	struct lwis_allocator_block_pool *block_pool;
	struct lwis_allocator_block *block;
	uint32_t idx;
	size_t block_size;
	unsigned long flags;

	if (lwis_dev == NULL) {
		dev_err(lwis_dev->dev, "lwis_dev is NULL\n");
		return NULL;
	}
	block_mgr = lwis_dev->block_mgr;
	if (block_mgr == NULL) {
		dev_err(lwis_dev->dev, "block_mgr is NULL\n");
		return NULL;
	}

	/*
	 * Linux already has slab allocator to cache the allocated memory within a page.
	 * The default page size is 4K. We can leverage linux's slab implementation for
	 * small size memory recycling.
	 */
	if (size <= 4 * 1024) {
		return kmalloc(size, GFP_KERNEL);
	}

	/*
	   fls() has better performance profile, it's currently used to mimic the
	   behavior of kmalloc_index().

	   kmalloc_index() return value as following:
	     if (size <=          8) return 3;
	     if (size <=         16) return 4;
	     if (size <=         32) return 5;
	     if (size <=         64) return 6;
	     if (size <=        128) return 7;
	     if (size <=        256) return 8;
	     if (size <=        512) return 9;
	     if (size <=       1024) return 10;
	     if (size <=   2 * 1024) return 11;
	     if (size <=   4 * 1024) return 12;
	     if (size <=   8 * 1024) return 13;
	     if (size <=  16 * 1024) return 14;
	     if (size <=  32 * 1024) return 15;
	     if (size <=  64 * 1024) return 16;
	     if (size <= 128 * 1024) return 17;
	     if (size <= 256 * 1024) return 18;
	     if (size <= 512 * 1024) return 19;
	     if (size <= 1024 * 1024) return 20;
	     if (size <=  2 * 1024 * 1024) return 21;
	     if (size <=  4 * 1024 * 1024) return 22;
	     if (size <=  8 * 1024 * 1024) return 23;
	     if (size <=  16 * 1024 * 1024) return 24;
	     if (size <=  32 * 1024 * 1024) return 25;
	*/
	idx = fls(size - 1);

	/*
	 * For the large size memory allocation, we usually use kvmalloc() to allocate
	 * the memory, but kvmalloc() does not take advantage of slab. For this case,
	 * we define several memory pools and recycle to use these memory blocks. For the
	 * size large than 512K, we do not have such use case yet. In current
	 * implementation, I do not cache it due to prevent keeping too much unused
	 * memory on hand.
	 */
	if (idx > 19) {
		block = kmalloc(sizeof(struct lwis_allocator_block), GFP_KERNEL);
		if (block == NULL) {
			dev_err(lwis_dev->dev, "Allocate failed\n");
			return NULL;
		}
		block->type = idx;
		block->next = NULL;
		block->prev = NULL;
		block->ptr = kvmalloc(size, GFP_KERNEL);
		if (block->ptr == NULL) {
			dev_err(lwis_dev->dev, "Allocate failed\n");
			kfree(block);
			return NULL;
		}
		spin_lock_irqsave(&block_mgr->lock, flags);
		block_mgr->pool_large.in_use_count++;
		hash_add(block_mgr->allocated_blocks, &block->node, (unsigned long long)block->ptr);
		spin_unlock_irqrestore(&block_mgr->lock, flags);
		return block->ptr;
	}

	block_pool = allocator_get_block_pool(block_mgr, idx);
	if (block_pool == NULL) {
		return NULL;
	}

	/* Try to get free block from recycling block pool */
	spin_lock_irqsave(&block_mgr->lock, flags);
	block = allocator_free_block_get_locked(block_pool);
	spin_unlock_irqrestore(&block_mgr->lock, flags);
	if (block != NULL) {
		return block->ptr;
	}

	/* Allocate new block */
	block = kmalloc(sizeof(struct lwis_allocator_block), GFP_KERNEL);
	if (block == NULL) {
		dev_err(lwis_dev->dev, "Allocate failed\n");
		return NULL;
	}
	block->type = idx;
	block->next = NULL;
	block->prev = NULL;
	block_size = 1 << idx;
	block->ptr = kvmalloc(block_size, GFP_KERNEL);
	if (block->ptr == NULL) {
		dev_err(lwis_dev->dev, "Allocate failed\n");
		kfree(block);
		return NULL;
	}

	spin_lock_irqsave(&block_mgr->lock, flags);
	block->next = block_pool->in_use;
	if (block->next != NULL) {
		block->next->prev = block;
	}
	block_pool->in_use = block;
	block_pool->in_use_count++;
	hash_add(block_mgr->allocated_blocks, &block->node, (unsigned long long)block->ptr);
	spin_unlock_irqrestore(&block_mgr->lock, flags);

	return block->ptr;
}

void lwis_allocator_free(struct lwis_device *lwis_dev, void *ptr)
{
	struct lwis_allocator_block_mgr *block_mgr;
	struct lwis_allocator_block_pool *block_pool;
	struct lwis_allocator_block *block = NULL;
	struct lwis_allocator_block *blk;
	unsigned long flags;

	if (lwis_dev == NULL || ptr == NULL) {
		dev_err(lwis_dev->dev, "input is NULL\n");
		return;
	}
	block_mgr = lwis_dev->block_mgr;
	if (block_mgr == NULL) {
		dev_err(lwis_dev->dev, "block_mgr is NULL\n");
		return;
	}
	hash_for_each_possible (block_mgr->allocated_blocks, blk, node, (unsigned long long)ptr) {
		if (blk->ptr == ptr) {
			block = blk;
			break;
		}
	}

	if (block == NULL) {
		kfree(ptr);
		return;
	}

	if (block->type > 19) {
		struct lwis_allocator_block *b;
		struct hlist_node *n;
		spin_lock_irqsave(&block_mgr->lock, flags);
		hash_for_each_possible_safe (block_mgr->allocated_blocks, b, n, node,
					     (unsigned long long)ptr) {
			if (b->ptr == block->ptr) {
				hash_del(&b->node);
				break;
			}
		}
		kvfree(block->ptr);
		kfree(block);
		block_mgr->pool_large.in_use_count--;
		spin_unlock_irqrestore(&block_mgr->lock, flags);
		return;
	}

	block_pool = allocator_get_block_pool(block_mgr, block->type);
	if (block_pool == NULL) {
		dev_err(lwis_dev->dev, "block type is invalid\n");
		return;
	}

	spin_lock_irqsave(&block_mgr->lock, flags);
	allocator_free_block_put_locked(block_pool, block);
	spin_unlock_irqrestore(&block_mgr->lock, flags);

	return;
}
