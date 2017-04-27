/*
 * linux/drivers/gpu/exynos/g2d/g2d_uapi_process.c
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/iommu.h>
#include <linux/ion.h>
#include <linux/slab.h>
#include <linux/sched/mm.h>
#include <linux/exynos_iovmm.h>

#include "g2d.h"
#include "g2d_task.h"
#include "g2d_uapi_process.h"
#include "g2d_command.h"

static int g2d_prepare_buffer(struct g2d_device *g2d_dev,
			      struct g2d_layer *layer,
			      struct g2d_layer_data *data)
{
	struct device *dev = g2d_dev->dev;
	const struct g2d_fmt *fmt = NULL;
	struct g2d_reg *cmd = layer->commands;
	unsigned int payload;
	int i;

	fmt = g2d_find_format(cmd[G2DSFR_IMG_COLORMODE].value);

	BUG_ON(!fmt);

	if ((data->num_buffers > 1) && (data->num_buffers != fmt->num_planes)) {
		dev_err(dev, "%s: Invalid number of buffers %u for %s\n",
			__func__, data->num_buffers, fmt->name);
		return -EINVAL;
	}

	if (data->num_buffers > 1) {
		for (i = 0; i < data->num_buffers; i++) {
			payload = g2d_get_payload_index(cmd, fmt, i);
			if (data->buffer[i].length < payload) {
				dev_err(dev,
					"%s: Too small buffer[%d]: expected %uB"
					" for %ux%u(b%u)/%s but %uB is given\n",
					__func__, i, payload,
					cmd[G2DSFR_IMG_WIDTH].value,
					cmd[G2DSFR_IMG_HEIGHT].value,
					cmd[G2DSFR_IMG_BOTTOM].value,
					fmt->name, data->buffer[i].length);
				return -EINVAL;
			}

			layer->buffer[i].payload = payload;
		}
	} else {
		payload = g2d_get_payload(cmd, fmt, layer->flags);
		if (data->buffer[0].length < payload) {
			dev_err(dev, "%s: Too small buffer: expected %uB"
				" for %ux%u(bt %u)/%s but %uB is given\n",
				__func__, payload,
				cmd[G2DSFR_IMG_WIDTH].value,
				cmd[G2DSFR_IMG_HEIGHT].value,
				cmd[G2DSFR_IMG_BOTTOM].value,
				fmt->name, data->buffer[0].length);
			return -EINVAL;
		}

		layer->buffer[0].payload = payload;
	}

	layer->num_buffers = data->num_buffers;

	return 0;
}

static int g2d_get_dmabuf(struct g2d_task *task,
			  struct g2d_buffer *buffer,
			  struct g2d_buffer_data *data,
			  enum dma_data_direction dir)
{
	struct device *dev = task->g2d_dev->dev;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	int ret = -EINVAL;
	int prot = IOMMU_READ | IOMMU_CACHE;

	dmabuf = dma_buf_get(data->dmabuf.fd);
	if (IS_ERR(dmabuf)) {
		dev_err(dev, "%s: Failed to get dmabuf from fd %d\n",
			__func__, data->dmabuf.fd);
		return PTR_ERR(dmabuf);
	}

	if (dmabuf->size < data->dmabuf.offset) {
		dev_err(dev, "%s: too large offset %u for dmabuf of %zu\n",
			__func__, data->dmabuf.offset, dmabuf->size);
		goto err;
	}

	if ((dmabuf->size - data->dmabuf.offset) < buffer->payload) {
		dev_err(dev, "%s: too small dmabuf %zu/%u but reqiured %u\n",
			__func__, dmabuf->size,
			data->dmabuf.offset, buffer->payload);
		goto err;
	}

	if (dir != DMA_TO_DEVICE)
		prot |= IOMMU_WRITE;

	attachment = dma_buf_attach(dmabuf, dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		dev_err(dev,
			"%s: failed to attach to dmabuf (%d)\n", __func__, ret);
		goto err;
	}

	sgt = dma_buf_map_attachment(attachment, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		dev_err(dev, "%s: failed to map dmabuf (%d)\n", __func__, ret);
		goto err_map;
	}

	dma_addr = ion_iovmm_map(attachment, 0, buffer->payload, dir, prot);
	if (IS_ERR_VALUE(dma_addr)) {
		ret = (int)dma_addr;
		dev_err(dev, "%s: failed to iovmm map for dmabuf (%d)\n",
			__func__, ret);
		goto err_iovmmmap;
	}

	buffer->dmabuf.dmabuf = dmabuf;
	buffer->dmabuf.attachment = attachment;
	buffer->dmabuf.sgt = sgt;
	buffer->dmabuf.offset = data->dmabuf.offset;
	buffer->dma_addr = dma_addr + data->dmabuf.offset;

	return 0;
err_iovmmmap:
	dma_buf_unmap_attachment(attachment, sgt, dir);
err_map:
	dma_buf_detach(dmabuf, attachment);
err:
	dma_buf_put(dmabuf);
	return ret;
}

static int g2d_put_dmabuf(struct g2d_device *g2d_dev, struct g2d_buffer *buffer,
			  enum dma_data_direction dir)
{
	ion_iovmm_unmap(buffer->dmabuf.attachment,
			buffer->dma_addr - buffer->dmabuf.offset);
	dma_buf_unmap_attachment(buffer->dmabuf.attachment,
				 buffer->dmabuf.sgt, dir);
	dma_buf_detach(buffer->dmabuf.dmabuf, buffer->dmabuf.attachment);
	dma_buf_put(buffer->dmabuf.dmabuf);

	memset(buffer, 0, sizeof(*buffer));

	return 0;
}

#define is_vma_cached(vma)						\
	((pgprot_val(pgprot_noncached((vma)->vm_page_prot)) !=		\
			  pgprot_val((vma)->vm_page_prot)) &&		\
	   (pgprot_val(pgprot_writecombine((vma)->vm_page_prot)) !=	\
	    pgprot_val((vma)->vm_page_prot)))

static int g2d_get_userptr(struct g2d_task *task,
			   struct g2d_buffer *buffer,
			   struct g2d_buffer_data *data,
			   enum dma_data_direction dir)
{
	struct device *dev = task->g2d_dev->dev;
	unsigned long end = data->userptr + data->length;
	struct vm_area_struct *vma;
	struct vm_area_struct *tvma;
	struct mm_struct *mm;
	int ret = -EINVAL;
	int prot = IOMMU_READ;

	mm = get_task_mm(current);

	down_read(&mm->mmap_sem);

	vma = find_vma(mm, data->userptr);
	if (!vma || (data->userptr < vma->vm_start)) {
		dev_err(dev, "%s: invalid address %#lx\n",
			__func__, data->userptr);
		goto err_novma;
	}

	tvma = kmemdup(vma, sizeof(*vma), GFP_KERNEL);
	if (!tvma) {
		ret = -ENOMEM;
		goto err_novma;
	}

	if (dir != DMA_TO_DEVICE)
		prot |= IOMMU_WRITE;
	if (is_vma_cached(vma))
		prot |= IOMMU_CACHE;

	buffer->userptr.vma = tvma;

	tvma->vm_next = NULL;
	tvma->vm_prev = NULL;
	tvma->vm_mm = mm;

	while (end > vma->vm_end) {
		if (!!(vma->vm_flags & VM_PFNMAP)) {
			dev_err(dev, "%s: non-linear pfnmap is not supported\n",
				__func__);
			goto err_vma;
		}

		if ((vma->vm_next == NULL) ||
				(vma->vm_end != vma->vm_next->vm_start)) {
			dev_err(dev, "%s: invalid size %u bytes\n",
					__func__, data->length);
			goto err_vma;
		}

		vma = vma->vm_next;
		tvma->vm_next = kmemdup(vma, sizeof(*vma), GFP_KERNEL);
		if (tvma->vm_next == NULL) {
			dev_err(dev, "%s: failed to allocate vma\n", __func__);
			ret = -ENOMEM;
			goto err_vma;
		}
		tvma->vm_next->vm_prev = tvma;
		tvma->vm_next->vm_next = NULL;
		tvma = tvma->vm_next;
	}

	for (vma = buffer->userptr.vma; vma != NULL; vma = vma->vm_next) {
		if (vma->vm_file)
			get_file(vma->vm_file);
		if (vma->vm_ops && vma->vm_ops->open)
			vma->vm_ops->open(vma);
	}

	buffer->dma_addr = exynos_iovmm_map_userptr(dev, data->userptr,
						    data->length, prot);
	if (IS_ERR_VALUE(buffer->dma_addr)) {
		ret = (int)buffer->dma_addr;
		dev_err(dev, "%s: failed to iovmm map userptr\n", __func__);
		goto err_map;
	}

	up_read(&mm->mmap_sem);

	buffer->userptr.addr = data->userptr;

	return 0;
err_map:
	buffer->dma_addr = 0;

	for (vma = buffer->userptr.vma; vma != NULL; vma = vma->vm_next) {
		if (vma->vm_file)
			fput(vma->vm_file);
		if (vma->vm_ops && vma->vm_ops->close)
			vma->vm_ops->close(vma);
	}
err_vma:
	while (tvma) {
		vma = tvma;
		tvma = tvma->vm_prev;
		kfree(vma);
	}

	buffer->userptr.vma = NULL;
err_novma:
	up_read(&mm->mmap_sem);

	mmput(mm);

	return ret;
}

static int g2d_put_userptr(struct g2d_device *g2d_dev,
			struct g2d_buffer *buffer, enum dma_data_direction dir)
{
	struct vm_area_struct *vma = buffer->userptr.vma;
	struct mm_struct *mm;

	BUG_ON((vma == NULL) || (buffer->userptr.addr == 0));

	mm = vma->vm_mm;

	exynos_iovmm_unmap_userptr(g2d_dev->dev, buffer->dma_addr);

	/*
	 * Calling to vm_ops->close() actually does not need mmap_sem to be
	 * acquired but some device driver needs mmap_sem to be held.
	 */
	down_read(&mm->mmap_sem);

	while (vma) {
		struct vm_area_struct *tvma;

		if (vma->vm_ops && vma->vm_ops->close)
			vma->vm_ops->close(vma);

		if (vma->vm_file)
			fput(vma->vm_file);

		tvma = vma;
		vma = vma->vm_next;

		kfree(tvma);
	}

	up_read(&mm->mmap_sem);

	mmput(mm);

	memset(buffer, 0, sizeof(*buffer));

	return 0;
}

static int g2d_get_buffer(struct g2d_device *g2d_dev,
				struct g2d_layer *layer,
				struct g2d_layer_data *data,
				enum dma_data_direction dir)
{
	int ret = 0;
	unsigned int i;
	int (*get_func)(struct g2d_task *, struct g2d_buffer *,
			struct g2d_buffer_data *, enum dma_data_direction);
	int (*put_func)(struct g2d_device *, struct g2d_buffer *,
			enum dma_data_direction);

	if (layer->buffer_type == G2D_BUFTYPE_DMABUF) {
		get_func = g2d_get_dmabuf;
		put_func = g2d_put_dmabuf;
	} else if (layer->buffer_type == G2D_BUFTYPE_USERPTR) {
		get_func = g2d_get_userptr;
		put_func = g2d_put_userptr;
	} else {
		BUG();
	}

	for (i = 0; i < layer->num_buffers; i++) {
		ret = get_func(layer->task, &layer->buffer[i],
			       &data->buffer[i], dir);
		if (ret) {
			while (i-- > 0)
				put_func(g2d_dev, &layer->buffer[i], dir);
			return ret;
		}

		layer->buffer[i].length = data->buffer[i].length;
	}

	return 0;
}

static void g2d_put_buffer(struct g2d_device *g2d_dev,
			u32 buffer_type, struct g2d_buffer buffer[],
			unsigned int num_buffer, enum dma_data_direction dir)
{
	unsigned int i;
	int (*put_func)(struct g2d_device *, struct g2d_buffer *,
			enum dma_data_direction);

	switch (buffer_type) {
	case G2D_BUFTYPE_DMABUF:
		put_func = g2d_put_dmabuf;
		break;
	case G2D_BUFTYPE_USERPTR:
		put_func = g2d_put_userptr;
		break;
	case G2D_BUFTYPE_EMPTY:
		return;
	default:
		BUG();
	}

	for (i = 0; i < num_buffer; i++)
		put_func(g2d_dev, &buffer[i], dir);
}

static void g2d_put_image(struct g2d_device *g2d_dev, struct g2d_layer *layer,
			  enum dma_data_direction dir)
{
	g2d_put_buffer(g2d_dev, layer->buffer_type,
			layer->buffer, layer->num_buffers, dir);

	layer->buffer_type = G2D_BUFTYPE_NONE;
}

static int g2d_get_source(struct g2d_device *g2d_dev, struct g2d_task *task,
			  struct g2d_layer *layer, struct g2d_layer_data *data,
			  int index)
{
	struct device *dev = g2d_dev->dev;
	int ret;

	layer->flags = data->flags;
	layer->buffer_type = data->buffer_type;

	if (!G2D_BUFTYPE_VALID(layer->buffer_type)) {
		dev_err(dev, "%s: invalid buffer type %u specified\n",
			__func__, layer->buffer_type);
		return -EINVAL;
	}

	/* color fill has no buffer */
	if (layer->buffer_type == G2D_BUFTYPE_EMPTY) {
		if (layer->flags & G2D_LAYERFLAG_COLORFILL) {
			layer->num_buffers = 0;
			layer->flags &= ~G2D_LAYERFLAG_ACQUIRE_FENCE;
			return 0;
		}

		dev_err(dev, "%s: DMA layer %d has no buffer - flags: %#x\n",
			__func__, index, layer->flags);
		return -EINVAL;
	}

	if (!g2d_validate_source_commands(g2d_dev, index, layer, &task->target))
		return -EINVAL;

	ret = g2d_prepare_buffer(g2d_dev, layer, data);
	if (ret)
		return ret;

	ret = g2d_get_buffer(g2d_dev, layer, data, DMA_TO_DEVICE);
	if (ret)
		return ret;

	if (!g2d_prepare_source(task, layer, index)) {
		dev_err(dev, "%s: Failed to prepare source layer %d\n",
			__func__, index);
		g2d_put_buffer(g2d_dev, layer->buffer_type, layer->buffer,
				layer->num_buffers, DMA_TO_DEVICE);
		return -EINVAL;
	}

	return 0;
}

static int g2d_get_sources(struct g2d_device *g2d_dev, struct g2d_task *task,
			   struct g2d_layer_data __user *src)
{
	struct device *dev = g2d_dev->dev;
	unsigned int i;
	int ret;

	for (i = 0; i < task->num_source; i++, src++) {
		struct g2d_layer_data data;

		if (copy_from_user(&data, src, sizeof(*src))) {
			dev_err(dev,
				"%s: Failed to read source image data %d/%d\n",
				__func__, i, task->num_source);
			ret = -EFAULT;
			break;
		}

		ret = g2d_get_source(g2d_dev, task,
				     &task->source[i], &data, i);
		if (ret)
			break;
	}

	if (i < task->num_source) {
		while (i-- > 0)
			g2d_put_image(g2d_dev, &task->source[i], DMA_TO_DEVICE);
	}

	return ret;
}

static int g2d_get_target(struct g2d_device *g2d_dev,
			  struct g2d_task *task, struct g2d_layer_data *data)
{
	struct device *dev = g2d_dev->dev;
	struct g2d_layer *target = &task->target;
	int ret;

	target->flags = data->flags;
	target->buffer_type = data->buffer_type;

	if (!G2D_BUFTYPE_VALID(target->buffer_type)) {
		dev_err(dev,
			"%s: invalid buffer type %u specified for target\n",
			__func__, target->buffer_type);
		return -EINVAL;
	}

	if (target->buffer_type == G2D_BUFTYPE_EMPTY) {
		if (!!(task->flags & G2D_FLAG_HWFC)) {
			/* TODO: set the hwfc buffers from g2d_dev*/
		} else {
			dev_err(dev, "%s: target has no buffer - flags: %#x\n",
				__func__, task->flags);
			return -EINVAL;
		}
	}

	if (!g2d_validate_target_commands(g2d_dev, task))
		return -EINVAL;

	ret = g2d_prepare_buffer(g2d_dev, target, data);
	if (ret)
		return ret;

	ret = g2d_get_buffer(g2d_dev, target, data, DMA_FROM_DEVICE);
	if (ret)
		return ret;

	if (!g2d_prepare_target(task)) {
		dev_err(dev, "%s: Failed to prepare target layer\n", __func__);

		g2d_put_buffer(g2d_dev, target->buffer_type, target->buffer,
				target->num_buffers, DMA_FROM_DEVICE);
		return -EINVAL;
	}

	return 0;
}

int g2d_get_userdata(struct g2d_device *g2d_dev,
		     struct g2d_task *task, struct g2d_task_data *data)
{
	struct device *dev = g2d_dev->dev;
	int ret;

	/* invalid range check */
	if ((data->num_source < 1) || (data->num_source > G2D_MAX_IMAGES)) {
		dev_err(dev, "%s: Invalid number of source images %u\n",
			__func__, data->num_source);
		return -EINVAL;
	}

	if (data->priority > G2D_MAX_PRIORITY) {
		dev_err(dev, "%s: Invalid number of priority %u\n",
			__func__, data->priority);
		return -EINVAL;
	}

	task->flags = data->flags;
	task->priority = data->priority;
	task->num_source = data->num_source;

	ret = g2d_import_commands(g2d_dev, task, data, task->num_source);
	if (ret < 0)
		return ret;

	ret = g2d_get_target(g2d_dev, task, &data->target);
	if (ret)
		return ret;

	ret = g2d_get_sources(g2d_dev, task, data->source);
	if (ret)
		goto err_src;

	return 0;
err_src:
	g2d_put_image(g2d_dev, &task->target, DMA_FROM_DEVICE);

	return ret;
}

static void g2d_put_images(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	unsigned int i;

	for (i = 0; i < task->num_source; i++)
		g2d_put_image(g2d_dev, &task->source[i], DMA_TO_DEVICE);

	g2d_put_image(g2d_dev, &task->target, DMA_FROM_DEVICE);
	task->num_source = 0;
}

int g2d_wait_put_user(struct g2d_device *g2d_dev, struct g2d_task *task,
		      struct g2d_task_data __user *uptr, u32 userflag)
{
	u32 laptime_in_usec = (u32)ktime_us_delta(task->ktime_end,
						  task->ktime_begin);
	int ret;

	if (!g2d_task_wait_completion(task)) {
		userflag |= G2D_FLAG_ERROR;
		ret = put_user(userflag, &uptr->flags);
	} else {
		ret = put_user(laptime_in_usec, &uptr->laptime_in_usec);
	}

	g2d_put_images(g2d_dev, task);

	g2d_put_free_task(g2d_dev, task);

	return ret;
}
