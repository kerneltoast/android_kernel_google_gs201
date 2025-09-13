// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 Google LLC.
 *
 * Protected memory allocator driver for allocation and release of pages of
 * protected memory for use by Mali GPU device drivers.
 */

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/protected_memory_allocator.h>
#include <linux/slab.h>
#include <linux/version.h>

MODULE_IMPORT_NS(DMA_BUF);

#define MALI_PMA_DMA_HEAP_NAME "vframe-secure"
#define MALI_PMA_SLAB_SIZE (1 << 16)
#define MALI_PMA_SLAB_BLOCK_SIZE (PAGE_SIZE)
#define MALI_PMA_SLAB_BLOCK_COUNT \
	(MALI_PMA_SLAB_SIZE / MALI_PMA_SLAB_BLOCK_SIZE)
#define MALI_PMA_MAX_ALLOC_SIZE (MALI_PMA_SLAB_SIZE)

/**
 * struct mali_pma_dev - Structure for managing a Mali protected memory
 *                       allocator device.
 *
 * @pma_dev: The base protected memory allocator device.
 * @dev: The device for which to allocate protected memory.
 * @dma_heap: The DMA buffer heap from which to allocate protected memory.
 * @slab_list: List of allocated slabs of protected memory.
 * @slab_mutex: Mutex used to serialize access to the slab list.
 */
struct mali_pma_dev {
	struct protected_memory_allocator_device pma_dev;
	struct device *dev;
	struct dma_heap *dma_heap;
	struct list_head slab_list;
	struct mutex slab_mutex;
};

/**
 * struct mali_protected_memory_allocation - Structure for tracking a Mali
 *                                           protected memory allocation.
 *
 * @pma: The base protected memory allocation record.
 * @slab: Protected memory slab used for allocation.
 * @first_block_index: Index of first memory block allocated from the slab.
 * @block_count: Count of the number of blocks allocated from the slab.
 */
struct mali_protected_memory_allocation {
	struct protected_memory_allocation pma;
	struct mali_pma_slab *slab;
	int first_block_index;
	int block_count;
};

/**
 * struct mali_pma_slab - Structure for managing a slab of Mali protected
 *                        memory.
 *
 * @list_entry: Entry in slab list.
 * @base: Physical base address of slab memory.
 * @dma_buf: The DMA buffer allocated for the slab . A reference to the DMA
 *           buffer is held by this pointer.
 * @dma_attachment: The DMA buffer device attachment.
 * @dma_sg_table: The DMA buffer scatter/gather table.
 * @allocated_block_map: Bit map of allocated blocks in the slab.
 */
struct mali_pma_slab {
	struct list_head list_entry;
	phys_addr_t base;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *dma_attachment;
	struct sg_table *dma_sg_table;
	uint64_t allocated_block_map;
};
static_assert(8 * sizeof(((struct mali_pma_slab *) 0)->allocated_block_map) >=
		MALI_PMA_SLAB_BLOCK_COUNT);

static struct protected_memory_allocation *mali_pma_alloc_page(
	struct protected_memory_allocator_device *pma_dev,
	unsigned int order);

static phys_addr_t mali_pma_get_phys_addr(
	struct protected_memory_allocator_device *pma_dev,
	struct protected_memory_allocation *pma);

static void mali_pma_free_page(
	struct protected_memory_allocator_device *pma_dev,
	struct protected_memory_allocation *pma);

static bool mali_pma_slab_alloc(
	struct mali_pma_dev* mali_pma_dev,
	struct mali_protected_memory_allocation *mali_pma, size_t size);

static void mali_pma_slab_dealloc(
	struct mali_pma_dev* mali_pma_dev,
	struct mali_protected_memory_allocation *mali_pma);

static bool mali_pma_slab_find_available(
	struct mali_pma_dev* mali_pma_dev, size_t size,
	struct mali_pma_slab** p_slab, int* p_block_index);

static struct mali_pma_slab* mali_pma_slab_add(
	struct mali_pma_dev* mali_pma_dev);

static void mali_pma_slab_remove(
	struct mali_pma_dev* mali_pma_dev, struct mali_pma_slab* slab);

static int protected_memory_allocator_probe(struct platform_device *pdev);

static int protected_memory_allocator_remove(struct platform_device *pdev);

/**
 * mali_pma_alloc_page - Allocate protected memory pages
 *
 * @pma_dev: The protected memory allocator the request is being made
 *           through.
 * @order:   How many pages to allocate, as a base-2 logarithm.
 *
 * Return: Pointer to allocated memory, or NULL if allocation failed.
 */
static struct protected_memory_allocation *mali_pma_alloc_page(
	struct protected_memory_allocator_device *pma_dev,
	unsigned int order) {
	struct mali_pma_dev *mali_pma_dev;
	struct protected_memory_allocation* pma = NULL;
	struct mali_protected_memory_allocation *mali_pma;
	size_t alloc_size;
	bool succeeded = false;

	/* Get the Mali protected memory allocator device record. */
	mali_pma_dev = container_of(pma_dev, struct mali_pma_dev, pma_dev);

	/* Check requested size against the maximum size. */
	alloc_size = 1 << (PAGE_SHIFT + order);
	if (alloc_size > MALI_PMA_MAX_ALLOC_SIZE) {
		dev_err(mali_pma_dev->dev,
			"Protected memory allocation size %zu too big\n",
			alloc_size);
		goto out;
	}

	/* Allocate a Mali protected memory allocation record. */
	mali_pma = devm_kzalloc(
		mali_pma_dev->dev, sizeof(*mali_pma), GFP_KERNEL);
	if (!mali_pma) {
		dev_err(mali_pma_dev->dev,
			"Failed to allocate a Mali protected memory allocation "
			"record\n");
		goto out;
	}
	pma = &(mali_pma->pma);
	pma->order = order;

	/* Allocate Mali protected memory from a slab. */
	if (!mali_pma_slab_alloc(mali_pma_dev, mali_pma, alloc_size)) {
		dev_err(mali_pma_dev->dev,
			"Failed to allocate Mali protected memory.\n");
		goto out;
	}

	/* Mark the allocation as successful. */
	succeeded = true;

out:
	/* Clean up on error. */
	if (!succeeded) {
		if (pma) {
			mali_pma_free_page(pma_dev, pma);
			pma = NULL;
		}
	}

	return pma;
}

/**
 * mali_pma_get_phys_addr - Get the physical address of the protected memory
 *                          allocation
 *
 * @pma_dev: The protected memory allocator the request is being made
 *           through.
 * @pma:     The protected memory allocation whose physical address
 *           shall be retrieved
 *
 * Return: The physical address of the given allocation.
 */
static phys_addr_t mali_pma_get_phys_addr(
	struct protected_memory_allocator_device *pma_dev,
	struct protected_memory_allocation *pma) {
	return pma->pa;
}

/**
 * mali_pma_free_page - Free a page of memory
 *
 * @pma_dev: The protected memory allocator the request is being made
 *           through.
 * @pma:     The protected memory allocation to free.
 */
static void mali_pma_free_page(
	struct protected_memory_allocator_device *pma_dev,
	struct protected_memory_allocation *pma) {
	struct mali_pma_dev *mali_pma_dev;
	struct mali_protected_memory_allocation *mali_pma;

	/*
	 * Get the Mali protected memory allocator device record and allocation
	 * record.
	 */
	mali_pma_dev = container_of(pma_dev, struct mali_pma_dev, pma_dev);
	mali_pma =
		container_of(pma, struct mali_protected_memory_allocation, pma);

	/* Deallocate Mali protected memory from the slab. */
	mali_pma_slab_dealloc(mali_pma_dev, mali_pma);

	/* Deallocate the Mali protected memory allocation record. */
	devm_kfree(mali_pma_dev->dev, mali_pma);
}

/**
 * mali_pma_slab_alloc - Allocate protected memory from a slab
 *
 * @mali_pma_dev: Mali protected memory allocator device.
 * @mali_pma: Mali protected memory allocation record to hold the slab memory.
 * @size: Size in bytes of memory to allocate.
 *
 * Return: True if memory was successfully allocated.
 */
static bool mali_pma_slab_alloc(
	struct mali_pma_dev *mali_pma_dev,
	struct mali_protected_memory_allocation *mali_pma, size_t size) {
	struct mali_pma_slab *slab;
	int start_block;
	int block_count;
	bool succeeded = false;

	/* Lock the slab list. */
	mutex_lock(&(mali_pma_dev->slab_mutex));

	/*
	 * Try finding an existing slab from which to allocate. If none are
	 * available, add a new slab and allocate from it.
	 */
	if (!mali_pma_slab_find_available(
		mali_pma_dev, size, &slab, &start_block)) {
		slab = mali_pma_slab_add(mali_pma_dev);
		if (!slab) {
			goto out;
		}
		start_block = 0;
	}

	/* Allocate a contiguous set of blocks from the slab. */
	block_count = DIV_ROUND_UP(size, MALI_PMA_SLAB_BLOCK_SIZE);
	bitmap_set((unsigned long *) &(slab->allocated_block_map),
			start_block, block_count);

	/*
	 * Use the allocated slab memory for the Mali protected memory
	 * allocation.
	 */
	mali_pma->pma.pa =
		slab->base + (start_block * MALI_PMA_SLAB_BLOCK_SIZE);
	mali_pma->slab = slab;
	mali_pma->first_block_index = start_block;
	mali_pma->block_count = block_count;

	/* Mark the allocation as successful. */
	succeeded = true;

out:
	/* Unlock the slab list. */
	mutex_unlock(&(mali_pma_dev->slab_mutex));

	return succeeded;
}

/**
 * mali_pma_slab_dealloc - Deallocate protected memory from a slab
 *
 * @mali_pma_dev: Mali protected memory allocator device.
 * @mali_pma: Mali protected memory allocation record holding slab memory to
 *            deallocate.
 */
static void mali_pma_slab_dealloc(
	struct mali_pma_dev *mali_pma_dev,
	struct mali_protected_memory_allocation *mali_pma) {
	struct mali_pma_slab *slab;

	/* Lock the slab list. */
	mutex_lock(&(mali_pma_dev->slab_mutex));

	/* Get the slab. */
	slab = mali_pma->slab;

	/* Deallocate the slab. */
	if (slab != NULL) {
		/* Deallocate all the blocks in the slab. */
		bitmap_clear((unsigned long *) &(slab->allocated_block_map),
				mali_pma->first_block_index,
				mali_pma->block_count);

		/* If no slab blocks remain allocated, remove the slab. */
		if (bitmap_empty(
			(unsigned long *) &(slab->allocated_block_map),
			MALI_PMA_SLAB_BLOCK_COUNT)) {
			mali_pma_slab_remove(mali_pma_dev, slab);
		}
	}

	/* Unlock the slab list. */
	mutex_unlock(&(mali_pma_dev->slab_mutex));
}

/**
 * mali_pma_slab_find_available - Find a slab with available memory
 *
 * Must be called with the slab list mutex locked.
 *
 * @mali_pma_dev: Mali protected memory allocator device.
 * @size: Size in bytes of requested memory.
 * @p_slab: Returned slab with requested memory available.
 * @p_block_index: Returned starting block index of available memory.
 *
 * Return: True if a slab was found with the requested memory available.
 */
static bool mali_pma_slab_find_available(
	struct mali_pma_dev *mali_pma_dev, size_t size,
	struct mali_pma_slab **p_slab, int *p_block_index) {
	struct mali_pma_slab *slab;
	int block_count;
	int start_block;
	bool found = false;

	/* Ensure the slab list mutex is locked. */
	lockdep_assert_held(&(mali_pma_dev->slab_mutex));

	/* Search slabs for a contiguous set of blocks of the requested size. */
	block_count = DIV_ROUND_UP(size, MALI_PMA_SLAB_BLOCK_SIZE);
	list_for_each_entry(slab, &(mali_pma_dev->slab_list), list_entry) {
		start_block = bitmap_find_next_zero_area_off(
			(unsigned long *) &(slab->allocated_block_map),
			MALI_PMA_SLAB_BLOCK_COUNT, 0, block_count, 0, 0);
		if (start_block < MALI_PMA_SLAB_BLOCK_COUNT) {
			found = true;
			break;
		}
	}

	/* Return results if found. */
	if (found) {
		*p_slab = slab;
		*p_block_index = start_block;
	}

	return found;
}

/**
 * mali_pma_slab_add - Allocate and add a new slab
 *
 * Must be called with the slab list mutex locked.
 *
 * @mali_pma_dev: Mali protected memory allocator device.
 *
 * Return: Newly added slab.
 */
static struct mali_pma_slab *mali_pma_slab_add(
	struct mali_pma_dev *mali_pma_dev) {
	struct mali_pma_slab *slab = NULL;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *dma_attachment;
	struct sg_table *dma_sg_table;
	bool succeeded = false;

	/* Ensure the slab list mutex is locked. */
	lockdep_assert_held(&(mali_pma_dev->slab_mutex));

	/* Allocate and initialize a Mali protected memory slab record. */
	slab = devm_kzalloc(mali_pma_dev->dev, sizeof(*slab), GFP_KERNEL);
	if (!slab) {
		dev_err(mali_pma_dev->dev,
			"Failed to allocate a Mali protected memory slab.\n");
		goto out;
	}
	INIT_LIST_HEAD(&(slab->list_entry));

	/* Allocate a DMA buffer. */
	dma_buf = dma_heap_buffer_alloc(
		mali_pma_dev->dma_heap, MALI_PMA_SLAB_SIZE, O_RDWR, 0);
	if (IS_ERR(dma_buf)) {
		dev_err(mali_pma_dev->dev,
			"Failed to allocate a DMA buffer of size %d\n",
			MALI_PMA_SLAB_SIZE);
		goto out;
	}
	slab->dma_buf = dma_buf;

	/* Attach the device to the DMA buffer. */
	dma_attachment = dma_buf_attach(dma_buf, mali_pma_dev->dev);
	if (IS_ERR(dma_attachment)) {
		dev_err(mali_pma_dev->dev,
			"Failed to attach the device to the DMA buffer\n");
		goto out;
	}
	slab->dma_attachment = dma_attachment;

	/* Map the DMA buffer into the attached device address space. */
#if (KERNEL_VERSION(6, 1, 55) <= LINUX_VERSION_CODE)
	dma_sg_table =
		dma_buf_map_attachment_unlocked(dma_attachment, DMA_BIDIRECTIONAL);
#else
	dma_sg_table =
		dma_buf_map_attachment(dma_attachment, DMA_BIDIRECTIONAL);
#endif
	if (IS_ERR(dma_sg_table)) {
		dev_err(mali_pma_dev->dev, "Failed to map the DMA buffer\n");
		goto out;
	}
	slab->dma_sg_table = dma_sg_table;
	slab->base = page_to_phys(sg_page(dma_sg_table->sgl));

	/* Add the slab to the slab list. */
	list_add(&(slab->list_entry), &(mali_pma_dev->slab_list));

	/* Mark that the slab was successfully added. */
	succeeded = true;

out:
	/* Clean up on failure. */
	if (!succeeded && (slab != NULL)) {
		mali_pma_slab_remove(mali_pma_dev, slab);
		slab = NULL;
	}

	return slab;
}

/**
 * mali_pma_slab_remove - Remove and deallocate a slab
 *
 * Must be called with the slab list mutex locked.
 *
 * @mali_pma_dev: Mali protected memory allocator device.
 * @slab: Slab to remove and deallocate.
 */
static void mali_pma_slab_remove(
	struct mali_pma_dev *mali_pma_dev, struct mali_pma_slab *slab) {
	/* Ensure the slab list mutex is locked. */
	lockdep_assert_held(&(mali_pma_dev->slab_mutex));

	/* Free the Mali protected memory slab allocation. */
	if (slab->dma_sg_table) {
#if (KERNEL_VERSION(6, 1, 55) <= LINUX_VERSION_CODE)
		dma_buf_unmap_attachment_unlocked(
			slab->dma_attachment,
			slab->dma_sg_table, DMA_BIDIRECTIONAL);
#else
		dma_buf_unmap_attachment(
			slab->dma_attachment,
	 		slab->dma_sg_table, DMA_BIDIRECTIONAL);
#endif
	}
	if (slab->dma_attachment) {
		dma_buf_detach(slab->dma_buf, slab->dma_attachment);
	}
	if (slab->dma_buf) {
		dma_buf_put(slab->dma_buf);
	}

	/* Remove the slab from the slab list. */
	list_del(&(slab->list_entry));

	/* Deallocate the Mali protected memory slab record. */
	devm_kfree(mali_pma_dev->dev, slab);
}

/**
 * protected_memory_allocator_probe - Probe the protected memory allocator
 *                                    device
 *
 * @pdev: The platform device to probe.
 */
static int protected_memory_allocator_probe(struct platform_device *pdev)
{
	struct dma_heap *pma_heap;
	struct mali_pma_dev *mali_pma_dev;
	struct protected_memory_allocator_device *pma_dev;
	int ret = 0;

	/* Try locating a PMA heap, defer if not present (yet). */
	pma_heap = dma_heap_find(MALI_PMA_DMA_HEAP_NAME);
	if (!pma_heap) {
		dev_warn(&(pdev->dev),
			"Failed to find \"%s\" DMA buffer heap. Deferring.\n",
			MALI_PMA_DMA_HEAP_NAME);
		ret = -EPROBE_DEFER;
		goto out;
	}

	/* Create a Mali protected memory allocator device record. */
	mali_pma_dev = kzalloc(sizeof(*mali_pma_dev), GFP_KERNEL);
	if (!mali_pma_dev) {
		dev_err(&(pdev->dev),
			"Failed to create a Mali protected memory allocator "
			"device record\n");
		dma_heap_put(pma_heap);
		ret = -ENOMEM;
		goto out;
	}
	pma_dev = &(mali_pma_dev->pma_dev);
	platform_set_drvdata(pdev, pma_dev);

	/* Initialize the slab list. */
	INIT_LIST_HEAD(&(mali_pma_dev->slab_list));
	mutex_init(&(mali_pma_dev->slab_mutex));

	/* Configure the Mali protected memory allocator. */
	mali_pma_dev->dev = &(pdev->dev);
	pma_dev->owner = THIS_MODULE;
	pma_dev->ops.pma_alloc_page = mali_pma_alloc_page;
	pma_dev->ops.pma_get_phys_addr = mali_pma_get_phys_addr;
	pma_dev->ops.pma_free_page = mali_pma_free_page;

	/* Assign the DMA buffer heap. */
	mali_pma_dev->dma_heap = pma_heap;

	/* Log that the protected memory allocator was successfully probed. */
	dev_info(&(pdev->dev),
		"Protected memory allocator probed successfully\n");

out:
	return ret;
}

/**
 * protected_memory_allocator_remove - Remove the protected memory allocator
 *                                     device
 *
 * @pdev: The protected memory allocator platform device to remove.
 */
static int protected_memory_allocator_remove(struct platform_device *pdev)
{
	struct protected_memory_allocator_device *pma_dev;
	struct mali_pma_dev *mali_pma_dev;

	/* Get the Mali protected memory allocator device record. */
	pma_dev = platform_get_drvdata(pdev);
	if (!pma_dev) {
		return 0;
	}
	mali_pma_dev = container_of(pma_dev, struct mali_pma_dev, pma_dev);

	/* Warn if there are any outstanding protected memory slabs. */
	if (!list_empty(&(mali_pma_dev->slab_list))) {
		dev_warn(&(pdev->dev),
			"Some protected memory has been left allocated\n");
	}

	/* Release the DMA buffer heap. */
	if (mali_pma_dev->dma_heap) {
		dma_heap_put(mali_pma_dev->dma_heap);
	}

	/* Free the Mali protected memory allocator device record. */
	kfree(mali_pma_dev);

	return 0;
}

static const struct of_device_id protected_memory_allocator_dt_ids[] = {
	{ .compatible = "arm,protected-memory-allocator" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, protected_memory_allocator_dt_ids);

struct platform_driver protected_memory_allocator_driver = {
	.probe = protected_memory_allocator_probe,
	.remove = protected_memory_allocator_remove,
	.driver = {
		.name = "mali-pma",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(protected_memory_allocator_dt_ids),
		.suppress_bind_attrs = true,
	}
};

