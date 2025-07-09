// SPDX-License-Identifier: GPL-2.0-only
/*
 * GCIP helpers for allocating memories.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <asm/page.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/mm_types.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <gcip/gcip-alloc-helper.h>

/*
 * Set @pages to the pages @mem represents.
 * @mem must be a pointer returned by vmalloc.
 *
 * Returns 0 on success, -ENOMEM when any page is NULL.
 */
static int gcip_vmalloc_to_pages(void *mem, size_t count, struct page **pages)
{
	size_t i = 0;

	while (count--) {
		pages[i] = vmalloc_to_page(mem);
		if (!pages[i])
			return -ENOMEM;
		i++;
		mem += PAGE_SIZE;
	}
	return 0;
}

struct sg_table *gcip_alloc_noncontiguous(struct device *dev, size_t size, gfp_t gfp)
{
	struct gcip_sgt_handle *sh = kmalloc(sizeof(*sh), gfp);
	void *mem;
	struct page **pages;
	size_t count;
	int ret;

	if (!sh)
		return NULL;

	size = PAGE_ALIGN(size);
	count = size >> PAGE_SHIFT;
	if (gfp & __GFP_ZERO)
		mem = vzalloc(size);
	else
		mem = vmalloc(size);
	if (!mem) {
		dev_err(dev, "GCIP noncontiguous alloc size=%#zx failed", size);
		goto err_free_sh;
	}

	pages = kvmalloc_array(count, sizeof(*pages), gfp);
	if (!pages)
		goto err_free_mem;

	if (gcip_vmalloc_to_pages(mem, count, pages)) {
		dev_err(dev, "convert memory to pages failed");
		goto err_free_pages;
	}

	ret = sg_alloc_table_from_pages(&sh->sgt, pages, count, 0, size, gfp);
	if (ret) {
		dev_err(dev, "alloc SG table with size=%#zx failed: %d", size, ret);
		goto err_free_pages;
	}

	kvfree(pages);
	sh->mem = mem;
	return &sh->sgt;

err_free_pages:
	kvfree(pages);
err_free_mem:
	vfree(mem);
err_free_sh:
	kfree(sh);
	return NULL;
}

void gcip_free_noncontiguous(struct sg_table *sgt)
{
	struct gcip_sgt_handle *sh = container_of(sgt, struct gcip_sgt_handle, sgt);

	sg_free_table(&sh->sgt);
	vfree(sh->mem);
	kfree(sh);
}
