// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU operations for BigOcean
 *
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/samsung-dma-mapping.h>
#include <uapi/linux/dma-buf.h>

#include "bigo_iommu.h"

static void bigo_unmap_one(struct bufinfo *binfo)
{
	dma_buf_unmap_attachment(binfo->attachment, binfo->sgt,
				 DMA_BIDIRECTIONAL);
	dma_buf_detach(binfo->dmabuf, binfo->attachment);
	dma_buf_put(binfo->dmabuf);
}

void bigo_unmap_all(struct bigo_inst *inst)
{
	struct bufinfo *curr, *next;

	mutex_lock(&inst->lock);
	list_for_each_entry_safe(curr, next, &inst->buffers, list) {
		list_del(&curr->list);
		bigo_unmap_one(curr);
		kfree(curr);
	}
	mutex_unlock(&inst->lock);
}

static int check_mapped_list(struct bigo_core *core, struct bigo_inst *inst,
			     struct bigo_ioc_mapping *mapping)
{
	int found = -1;
	struct bufinfo *binfo;

	mutex_lock(&inst->lock);
	list_for_each_entry(binfo, &inst->buffers, list) {
		/*
		 * TODO(vinaykalia@): Do we need to check for size,
		 * offset, etc?
		 */
		if (binfo->fd == mapping->fd) {
			mapping->iova = binfo->iova;
			found = 0;
			break;
		}
	}
	mutex_unlock(&inst->lock);

	return found;
}

static int add_to_mapped_list(struct bigo_core *core, struct bigo_inst *inst,
			      struct bigo_ioc_mapping *mapping)
{
	int rc = 0;
	struct bufinfo *binfo;

	binfo = kzalloc(sizeof(*binfo), GFP_KERNEL);
	if (!binfo)
		return -ENOMEM;
	binfo->dmabuf = dma_buf_get(mapping->fd);
	if (IS_ERR(binfo->dmabuf)) {
		rc = PTR_ERR(binfo->dmabuf);
		pr_err("failed to get dma buf(%d): %d\n", mapping->fd, rc);
		goto fail_buf_get;
	}

	binfo->attachment = dma_buf_attach(binfo->dmabuf, core->dev);
	if (IS_ERR(binfo->attachment)) {
		rc = PTR_ERR(binfo->attachment);
		pr_err("failed to dma_buf_attach: %d\n", rc);
		goto fail_attach;
	}

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	if (mapping->skip_cmo)
		binfo->attachment->dma_map_attrs |= DMA_ATTR_SKIP_CPU_SYNC;
#endif

	binfo->sgt = dma_buf_map_attachment(binfo->attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(binfo->sgt)) {
		rc = PTR_ERR(binfo->sgt);
		pr_err("failed to dma_buf_map_attachment: %d\n", rc);
		goto fail_map_attachment;
	}
	binfo->iova = sg_dma_address(binfo->sgt->sgl);
	binfo->fd = mapping->fd;
	binfo->size = mapping->size;
	binfo->offset = mapping->offset;
	mutex_lock(&inst->lock);
	list_add_tail(&binfo->list, &inst->buffers);
	mutex_unlock(&inst->lock);
	mapping->iova = binfo->iova;
	return rc;

fail_map_attachment:
	dma_buf_detach(binfo->dmabuf, binfo->attachment);
fail_attach:
	dma_buf_put(binfo->dmabuf);
fail_buf_get:
	kfree(binfo);
	return rc;
}

int bigo_map(struct bigo_core *core, struct bigo_inst *inst,
	     struct bigo_ioc_mapping *mapping)
{
	if (!check_mapped_list(core, inst, mapping))
		return 0;

	return add_to_mapped_list(core, inst, mapping);
}

int bigo_unmap(struct bigo_inst *inst, struct bigo_ioc_mapping *mapping)
{
	struct bufinfo *curr, *next;
	struct bufinfo *found = NULL;

	mutex_lock(&inst->lock);
	list_for_each_entry_safe(curr, next, &inst->buffers, list) {
		if (curr->fd == mapping->fd) {
			list_del(&curr->list);
			found = curr;
			break;
		}
	}
	mutex_unlock(&inst->lock);
	if (!found)
		return -ENOENT;

	bigo_unmap_one(found);
	kfree(found);
	return 0;
}

int bigo_dma_sync(struct bigo_buf_sync *sync)
{
	int ret;
	struct dma_buf *dmabuf;
	enum dma_data_direction direction;
	u64 flags;

	flags = sync->flags;
	if (flags & ~DMA_BUF_SYNC_VALID_FLAGS_MASK)
		return -EINVAL;

	switch (flags & DMA_BUF_SYNC_RW) {
	case DMA_BUF_SYNC_READ:
		direction = DMA_FROM_DEVICE;
		break;
	case DMA_BUF_SYNC_WRITE:
		direction = DMA_TO_DEVICE;
		break;
	case DMA_BUF_SYNC_RW:
		direction = DMA_BIDIRECTIONAL;
		break;
	default:
		return -EINVAL;
	}

	dmabuf = dma_buf_get(sync->fd);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		pr_err("failed to get dma buf(%d): %d\n", sync->fd, ret);
		return ret;
	}

	ret = -1;

	/* TODO: remove dependency on dmabuf internal API, see b/244200482 */
	if (flags & DMA_BUF_SYNC_END) {
		if (dmabuf->ops->end_cpu_access_partial)
			ret = dma_buf_end_cpu_access_partial(dmabuf, direction,
						sync->offset, sync->size);
		if (ret < 0)
			ret = dma_buf_end_cpu_access(dmabuf, direction);
	} else {
		if (dmabuf->ops->begin_cpu_access_partial)
			ret = dma_buf_begin_cpu_access_partial(dmabuf, direction,
						sync->offset, sync->size);
		if (ret < 0)
			ret = dma_buf_begin_cpu_access(dmabuf, direction);
	}
	dma_buf_put(dmabuf);

	return ret;
}

int bigo_iommu_fault_handler(struct iommu_fault *fault, void *param)
{
	struct bigo_core *core = (struct bigo_core*)param;
	struct bufinfo *binfo;
	struct bigo_inst *inst;

	/* Don't try to mutex_lock core->lock here since worker thread
	 * already has the lock */
	pr_info("mapped iova list:\n");
	list_for_each_entry(inst, &core->instances, list) {
		mutex_lock(&inst->lock);
		list_for_each_entry(binfo, &inst->buffers, list)
			pr_info("iova: 0x%llx size: %lu", binfo->iova, binfo->size);
		mutex_unlock(&inst->lock);
	}

	return NOTIFY_OK;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinay Kalia <vinaykalia@google.com>");
