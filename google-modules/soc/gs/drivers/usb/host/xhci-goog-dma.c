// SPDX-License-Identifier: GPL-2.0
/*
 * xhci-goog-dma.c - DMA coherent driver for xHCI.
 *
 * Copyright (c) 2024 Google LLC
 *
 * Author: Howard Yen <howardyen@google.com>
 */

#include <host/xhci.h>
#include <linux/bitmap.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include "xhci-goog-dma.h"

static inline dma_addr_t xhci_goog_dma_get_device_base(struct device *dev,
						       struct xhci_goog_dma_coherent_mem *mem)
{
	if (mem->use_dev_dma_pfn_offset)
		return phys_to_dma(dev, PFN_PHYS(mem->pfn_base));

	return mem->device_base;
}

static void *__xhci_goog_dma_alloc_from_coherent(struct device *dev,
						 struct xhci_goog_dma_coherent_mem *mem,
						 ssize_t size, dma_addr_t *dma_handle)
{
	int order = get_order(size);
	unsigned long flags;
	int pageno;
	void *ret;

	spin_lock_irqsave(&mem->spinlock, flags);

	if (unlikely(size > ((dma_addr_t)mem->size << PAGE_SHIFT)))
		goto err;

	pageno = bitmap_find_free_region(mem->bitmap, mem->size, order);
	if (unlikely(pageno < 0))
		goto err;

	/*
	 * Memory was found in the coherent area.
	 */
	*dma_handle = xhci_goog_dma_get_device_base(dev, mem) +
			((dma_addr_t)pageno << PAGE_SHIFT);
	ret = mem->virt_base + ((dma_addr_t)pageno << PAGE_SHIFT);
	spin_unlock_irqrestore(&mem->spinlock, flags);
	memset(ret, 0, size);
	return ret;
err:
	spin_unlock_irqrestore(&mem->spinlock, flags);
	return NULL;
}

static void *xhci_goog_dma_alloc_coherent(struct device *dev, size_t size,
					  dma_addr_t *dma_handle, gfp_t gfp,
					  unsigned long attrs)
{
	struct xhci_goog_dma_coherent_mem **dma_mem = NULL;
	void *ret = NULL;

	if (get_dma_coherent_mem)
		dma_mem = get_dma_coherent_mem(dev);
	if (!dma_mem)
		return NULL;

	ret = __xhci_goog_dma_alloc_from_coherent(dev, dma_mem[XHCI_GOOG_DMA_RMEM_SRAM], size,
						  dma_handle);
	if (!ret) {
		ret = __xhci_goog_dma_alloc_from_coherent(dev, dma_mem[XHCI_GOOG_DMA_RMEM_DRAM],
							  size, dma_handle);
	}

	return ret;
}

static int __xhci_goog_dma_free_coherent(struct xhci_goog_dma_coherent_mem *mem,
					 int order, void *vaddr)
{
	if (mem && vaddr >= mem->virt_base && vaddr <
		   (mem->virt_base + ((dma_addr_t)mem->size << PAGE_SHIFT))) {
		int page = (vaddr - mem->virt_base) >> PAGE_SHIFT;
		unsigned long flags;

		spin_lock_irqsave(&mem->spinlock, flags);
		bitmap_release_region(mem->bitmap, page, order);
		spin_unlock_irqrestore(&mem->spinlock, flags);
		return 1;
	}

	return 0;
}

static void xhci_goog_dma_free_coherent(struct device *dev, size_t size, void *vaddr,
					dma_addr_t dma_handle, unsigned long attrs)
{
	struct xhci_goog_dma_coherent_mem **dma_mem = NULL;
	int order = get_order(size);
	int ret = 0;

	if (get_dma_coherent_mem)
		dma_mem = get_dma_coherent_mem(dev);
	if (!dma_mem)
		return;

	ret = __xhci_goog_dma_free_coherent(dma_mem[XHCI_GOOG_DMA_RMEM_SRAM], order, vaddr);
	if (!ret)
		__xhci_goog_dma_free_coherent(dma_mem[XHCI_GOOG_DMA_RMEM_DRAM], order, vaddr);
}

static struct dma_map_ops xhci_goog_dma_ops;
static const struct dma_map_ops *dma_ops_temp;

void xhci_goog_setup_dma_ops(struct device *dev)
{
	const struct dma_map_ops *dma_ops = get_dma_ops(dev);

	if (dma_ops) {
		dev_dbg(dev, "Setup DMA ops\n");
		memcpy(&xhci_goog_dma_ops, dma_ops, sizeof(xhci_goog_dma_ops));
		xhci_goog_dma_ops.alloc = xhci_goog_dma_alloc_coherent;
		xhci_goog_dma_ops.free = xhci_goog_dma_free_coherent;
		set_dma_ops(dev, &xhci_goog_dma_ops);
		dma_ops_temp = dma_ops;
	} else {
		dev_err(dev, "No dma_ops\n");
	}
}
EXPORT_SYMBOL_GPL(xhci_goog_setup_dma_ops);

void xhci_goog_restore_dma_ops(struct device *dev)
{
	dev_dbg(dev, "Restore DMA ops\n");
	set_dma_ops(dev, dma_ops_temp);
}
EXPORT_SYMBOL_GPL(xhci_goog_restore_dma_ops);

static struct xhci_goog_dma_coherent_mem *xhci_goog_dma_init_coherent_memory(
	phys_addr_t phys_addr, dma_addr_t device_addr, size_t size, bool use_dma_pfn_offset)
{
	struct xhci_goog_dma_coherent_mem *dma_mem;
	int pages = size >> PAGE_SHIFT;
	int bitmap_size = BITS_TO_LONGS(pages) * sizeof(long);
	void *mem_base;

	if (!size)
		return ERR_PTR(-EINVAL);

	mem_base = memremap(phys_addr, size, MEMREMAP_WC);
	if (!mem_base)
		return ERR_PTR(-EINVAL);

	dma_mem = kzalloc(sizeof(struct xhci_goog_dma_coherent_mem), GFP_KERNEL);
	if (!dma_mem)
		goto out_unmap_membase;
	dma_mem->bitmap = kzalloc(bitmap_size, GFP_KERNEL);
	if (!dma_mem->bitmap)
		goto out_free_dma_mem;

	dma_mem->virt_base = mem_base;
	dma_mem->device_base = device_addr;
	dma_mem->pfn_base = PFN_DOWN(phys_addr);
	dma_mem->size = pages;
	dma_mem->use_dev_dma_pfn_offset = use_dma_pfn_offset;
	spin_lock_init(&dma_mem->spinlock);

	return dma_mem;

out_free_dma_mem:
	kfree(dma_mem);
out_unmap_membase:
	memunmap(mem_base);
	pr_err("Reserved memory: failed to init DMA memory pool at %pa, size %zd KiB\n",
	       &phys_addr, size / SZ_1K);
	return ERR_PTR(-ENOMEM);
}

int xhci_goog_register_get_cb(
	struct xhci_goog_dma_coherent_mem **(*get_dma_coherent_mem_cb)(struct device *dev))
{
	if (get_dma_coherent_mem_cb)
		get_dma_coherent_mem = get_dma_coherent_mem_cb;

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_goog_register_get_cb);

void xhci_goog_unregister_get_cb(void)
{
	if (get_dma_coherent_mem)
		get_dma_coherent_mem = NULL;
}
EXPORT_SYMBOL_GPL(xhci_goog_unregister_get_cb);

int xhci_goog_register_put_cb(void (*put_dma_coherent_mem_cb)(struct device *dev))
{
	if (put_dma_coherent_mem_cb)
		put_dma_coherent_mem = put_dma_coherent_mem_cb;

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_goog_register_put_cb);

void xhci_goog_unregister_put_cb(void)
{
	if (put_dma_coherent_mem)
		put_dma_coherent_mem = NULL;
}
EXPORT_SYMBOL_GPL(xhci_goog_unregister_put_cb);

static int xhci_goog_rmem_device_init(struct reserved_mem *rmem,
					    struct device *dev)
{
	struct xhci_goog_dma_coherent_mem **dma_mem = NULL;

	if (!rmem->priv) {
		struct xhci_goog_dma_coherent_mem *mem;

		mem = xhci_goog_dma_init_coherent_memory(rmem->base, rmem->base,
							 rmem->size, true);

		if (!mem)
			return PTR_ERR(mem);

		rmem->priv = mem;
	}

	if (get_dma_coherent_mem)
		dma_mem = get_dma_coherent_mem(dev);
	if (!dma_mem)
		return PTR_ERR(dma_mem);

	if (!dma_mem[XHCI_GOOG_DMA_RMEM_SRAM])
		dma_mem[XHCI_GOOG_DMA_RMEM_SRAM] = (struct xhci_goog_dma_coherent_mem *)rmem->priv;
	else if (!dma_mem[XHCI_GOOG_DMA_RMEM_DRAM])
		dma_mem[XHCI_GOOG_DMA_RMEM_DRAM] = (struct xhci_goog_dma_coherent_mem *)rmem->priv;
	else {
		dev_warn(dev, "Unexpected dma mem initialization\n");
		return -EEXIST;
	}

	return 0;
}

static void xhci_goog_rmem_device_release(struct reserved_mem *rmem,
					  struct device *dev)
{
	struct xhci_goog_dma_coherent_mem **dma_mem = NULL;
	int i;

	if (dev) {
		if (get_dma_coherent_mem)
			dma_mem = get_dma_coherent_mem(dev);
		if (!dma_mem)
			return;

		for (i = 0; i < XHCI_GOOG_DMA_RMEM_MAX; i++) {
			if (dma_mem[i] == rmem->priv) {
				dma_mem[i] = NULL;
				break;
			}
		}

		if (!dma_mem[XHCI_GOOG_DMA_RMEM_SRAM] && !dma_mem[XHCI_GOOG_DMA_RMEM_DRAM])
			if (put_dma_coherent_mem)
				put_dma_coherent_mem(dev);
	}
}

static const struct reserved_mem_ops xhci_goog_rmem_ops = {
	.device_init	= xhci_goog_rmem_device_init,
	.device_release	= xhci_goog_rmem_device_release,
};

int xhci_goog_rmem_setup_latecall(struct device *dev)
{
	struct device_node *np;
	struct reserved_mem *rmem;
	int i;

	for (i = 0; i < XHCI_GOOG_DMA_RMEM_MAX; i++) {
		np = of_parse_phandle(dev->of_node, "memory-region", i);
		if (!np) {
			dev_err(dev, "memory-region not found\n");
			break;
		}

		rmem = of_reserved_mem_lookup(np);
		if (!rmem) {
			dev_err(dev, "of_reserved_mem_lookup() failed\n");
			break;
		}

		rmem->ops = &xhci_goog_rmem_ops;
		dev_info(dev, "Reserved memory: created XHCI DMA memory pool at %pa, size %ld KiB\n",
			 &rmem->base, (unsigned long)rmem->size / SZ_1K);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_goog_rmem_setup_latecall);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("GOOG XHCI RMEM and DMA driver");
MODULE_AUTHOR("Howard Yen <howardyen@google.com>");
