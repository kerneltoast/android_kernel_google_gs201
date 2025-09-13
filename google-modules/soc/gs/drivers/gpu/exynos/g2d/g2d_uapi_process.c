// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 */

#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/sync_file.h>
#include <linux/slab.h>
#include <linux/sched/mm.h>
#include <linux/panic_notifier.h>
#include <media/frame_vector.h>

#include <asm/cacheflush.h>

#include "g2d.h"
#include "g2d_task.h"
#include "g2d_uapi_process.h"
#include "g2d_command.h"
#include "g2d_fence.h"
#include "g2d_regs.h"
#include "g2d_debug.h"

#define buferr_show(dev, i, payload, w, h, b, _mode, name, len)				   \
perrfndev(dev,										   \
	  "Too small buffer[%d]: expected %u for %ux%u(btm %u)/%s(mode %#x) but %u given", \
	  i, payload, w, h, b, name, _mode, len)

static int g2d_prepare_buffer(struct g2d_device *g2d_dev,
			      struct g2d_layer *layer,
			      struct g2d_layer_data *data, bool dst)
{
	const struct g2d_fmt *fmt = NULL;
	struct g2d_reg *cmd = layer->commands;
	unsigned int payload;
	int i;

	fmt = g2d_find_format(cmd[G2DSFR_IMG_COLORMODE].value, g2d_dev->caps);
	if (!fmt)
		return -EINVAL;

	if (data->num_buffers == 0) {
		perrfndev(g2d_dev, "Invalid number of buffer %u for %s",
			  data->num_buffers, fmt->name);
		return -EINVAL;
	}

	if (data->num_buffers > 1 && data->num_buffers != fmt->num_planes) {
		/* NV12 8+2 in two buffers is valid */
		if (fmt->num_planes != 4 || data->num_buffers != 2) {
			perrfndev(g2d_dev, "Invalid number of buffer %u for %s",
				  data->num_buffers, fmt->name);
			return -EINVAL;
		}
	}

	if (data->num_buffers > 1) {
		for (i = 0; i < data->num_buffers; i++) {
			payload = g2d_get_payload_index(cmd, fmt, i, data->num_buffers,
							g2d_dev->caps, layer->flags, dst);
			if (data->buffer[i].length < payload) {
				buferr_show(g2d_dev, i, payload,
					    cmd[G2DSFR_IMG_WIDTH].value,
					    cmd[G2DSFR_IMG_HEIGHT].value,
					    cmd[G2DSFR_IMG_BOTTOM].value,
					    cmd[G2DSFR_IMG_COLORMODE].value,
					    fmt->name, data->buffer[i].length);
				return -EINVAL;
			}

			layer->buffer[i].payload = payload;
		}
	} else {
		payload = (unsigned int)g2d_get_payload(cmd, fmt, layer->flags,
							g2d_dev->caps, dst);
		if (data->buffer[0].length < payload) {
			buferr_show(g2d_dev, 0, payload,
				    cmd[G2DSFR_IMG_WIDTH].value,
				    cmd[G2DSFR_IMG_HEIGHT].value,
				    cmd[G2DSFR_IMG_BOTTOM].value,
				    cmd[G2DSFR_IMG_COLORMODE].value,
				    fmt->name, data->buffer[0].length);
			return -EINVAL;
		}

		layer->buffer[0].payload = payload;
	}

	layer->num_buffers = data->num_buffers;

	return 0;
}

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
static struct dma_buf *g2d_get_hwfc_dmabuf(struct g2d_context *ctx,
					   struct g2d_task *task)
{
	struct dma_buf *dmabuf = ctx->hwfc_info->bufs[g2d_task_id(task)];

	get_dma_buf(dmabuf);

	return dmabuf;
}
#else
static struct dma_buf *g2d_get_hwfc_dmabuf(struct g2d_context *ctx,
					   struct g2d_task *task)
{
	perrfndev(task->g2d_dev, "HWFC without repeater is not supported.\n");
	return ERR_PTR(-EINVAL);
}

/* hwfc_get_valid_buffer is defined in the repeater driver */
static int hwfc_get_valid_buffer(int *buf_idx)
{
	return -ENOTSUPP;
}
#endif

static int g2d_get_dmabuf(struct g2d_task *task,
			  struct g2d_context *ctx,
			  struct g2d_buffer *buffer,
			  struct g2d_buffer_data *data,
			  enum dma_data_direction dir)
{
	struct g2d_device *g2d_dev = task->g2d_dev;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	int ret = -EINVAL;

	if (!IS_HWFC(task->flags) || dir == DMA_TO_DEVICE) {
		dmabuf = dma_buf_get(data->dmabuf.fd);
		if (IS_ERR(dmabuf)) {
			perrfndev(g2d_dev, "Failed to get dmabuf from fd %d",
				  data->dmabuf.fd);
			return PTR_ERR(dmabuf);
		}
	} else {
		dmabuf = g2d_get_hwfc_dmabuf(ctx, task);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
	}

	if (dmabuf->size < data->dmabuf.offset) {
		perrfndev(g2d_dev, "too large offset %u for dmabuf of %zu",
			  data->dmabuf.offset, dmabuf->size);
		goto err;
	}

	if ((dmabuf->size - data->dmabuf.offset) < buffer->payload) {
		perrfndev(g2d_dev, "too small dmabuf %zu/%u but reqiured %u",
			  dmabuf->size, data->dmabuf.offset, buffer->payload);
		goto err;
	}

	attachment = dma_buf_attach(dmabuf, g2d_dev->dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		perrfndev(g2d_dev, "failed to attach to dmabuf (%d)", ret);
		goto err;
	}

	sgt = dma_buf_map_attachment(attachment, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		perrfndev(g2d_dev, "failed to map dmabuf (%d)", ret);
		goto err_map;
	}

	buffer->dmabuf.dmabuf = dmabuf;
	buffer->dmabuf.attachment = attachment;
	buffer->dmabuf.offset = data->dmabuf.offset;
	buffer->dma_addr = sg_dma_address(sgt->sgl) + data->dmabuf.offset;
	buffer->sgt = sgt;

	return 0;
err_map:
	dma_buf_detach(dmabuf, attachment);
err:
	dma_buf_put(dmabuf);
	return ret;
}

static int g2d_put_dmabuf(struct g2d_device *g2d_dev, struct g2d_buffer *buffer,
			  enum dma_data_direction dir)
{
	dma_buf_unmap_attachment(buffer->dmabuf.attachment, buffer->sgt, dir);
	dma_buf_detach(buffer->dmabuf.dmabuf, buffer->dmabuf.attachment);
	dma_buf_put(buffer->dmabuf.dmabuf);

	memset(buffer, 0, sizeof(*buffer));

	return 0;
}

static int g2d_get_userptr(struct g2d_task *task,
			   struct g2d_context *ctx,
			   struct g2d_buffer *buffer,
			   struct g2d_buffer_data *data,
			   enum dma_data_direction dir)
{
	struct device *dev = task->g2d_dev->dev;
	unsigned long begin = PFN_DOWN(data->userptr);
	unsigned long end = PFN_UP(data->userptr + data->length);
	unsigned long page_off = data->userptr & ~PAGE_MASK;
	unsigned int nr_pages = (unsigned int)(end - begin);
	struct sg_table *sgt;
	struct frame_vector *vec;
	struct page **pages;
	int ret = -ENOMEM;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	vec = frame_vector_create(nr_pages);
	if (!vec)
		goto err_vector;

	ret = get_vaddr_frames(begin << PAGE_SHIFT, nr_pages, vec);
	if (ret < 0)
		goto err_get_frames;
	if (ret != (int)nr_pages) {
		ret = -EFAULT;
		goto err_nr_frames;
	}

	pages = frame_vector_pages(vec);
	if (IS_ERR(pages)) {
		ret = PTR_ERR(pages);
		goto err_nr_frames;
	}

	ret = sg_alloc_table_from_pages(sgt, pages, nr_pages, page_off, data->length, 0);
	if (ret)
		goto userptr_fail_sgtable;

	sgt->nents = dma_map_sg_attrs(dev, sgt->sgl, sgt->orig_nents, dir, 0);
	if (!sgt->nents)
		goto userptr_fail_map;

	buffer->userptr.addr = data->userptr;
	buffer->userptr.vec = vec;
	buffer->dma_addr = sg_dma_address(sgt->sgl);
	buffer->sgt = sgt;

	return 0;
userptr_fail_map:
	sg_free_table(sgt);
userptr_fail_sgtable:
err_nr_frames:
	put_vaddr_frames(vec);
err_get_frames:
	frame_vector_destroy(vec);
err_vector:
	kfree(sgt);
	return ret;
}

static int g2d_put_userptr(struct g2d_device *g2d_dev,
			   struct g2d_buffer *buffer,
			   enum dma_data_direction dir)
{
	dma_unmap_sg_attrs(g2d_dev->dev, buffer->sgt->sgl,
			   buffer->sgt->orig_nents, dir, 0);

	if (dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL) {
		struct scatterlist *sg;
		int i;

		for_each_sg(buffer->sgt->sgl, sg, buffer->sgt->nents, i)
			set_page_dirty_lock(sg_page(sg));
	}

	sg_free_table(buffer->sgt);
	put_vaddr_frames(buffer->userptr.vec);
	frame_vector_destroy(buffer->userptr.vec);
	kfree(buffer->sgt);

	memset(buffer, 0, sizeof(*buffer));

	return 0;
}

static int g2d_get_buffer(struct g2d_device *g2d_dev, struct g2d_context *ctx,
			  struct g2d_layer *layer, struct g2d_layer_data *data,
			  enum dma_data_direction dir)
{
	int ret = 0;
	unsigned int i;
	int (*get_func)(struct g2d_task *task, struct g2d_context *ctx,
			struct g2d_buffer *buf, struct g2d_buffer_data *data,
			enum dma_data_direction dir);
	int (*put_func)(struct g2d_device *dev, struct g2d_buffer *buf,
			enum dma_data_direction dir);

	if (layer->buffer_type == G2D_BUFTYPE_DMABUF) {
		get_func = g2d_get_dmabuf;
		put_func = g2d_put_dmabuf;
	} else if (layer->buffer_type == G2D_BUFTYPE_USERPTR) {
		get_func = g2d_get_userptr;
		put_func = g2d_put_userptr;
	} else {
		perrfndev(g2d_dev, "invalid buffer type %u specified", layer->buffer_type);
		return -EINVAL;
	}

	for (i = 0; i < layer->num_buffers; i++) {
		ret = get_func(layer->task, ctx, &layer->buffer[i], &data->buffer[i], dir);
		if (ret) {
			while (i-- > 0)
				put_func(g2d_dev, &layer->buffer[i], dir);
			return ret;
		}

		layer->buffer[i].length = data->buffer[i].length;
	}

	return 0;
}

static void g2d_put_buffer(struct g2d_device *g2d_dev, u32 buffer_type, struct g2d_buffer buffer[],
			   unsigned int num_buffer, enum dma_data_direction dir)
{
	unsigned int i;
	int (*put_func)(struct g2d_device *dev, struct g2d_buffer *buf,
			enum dma_data_direction dir);

	switch (buffer_type) {
	case G2D_BUFTYPE_DMABUF:
		put_func = g2d_put_dmabuf;
		break;
	case G2D_BUFTYPE_USERPTR:
		put_func = g2d_put_userptr;
		break;
	default:
		if (buffer_type != G2D_BUFTYPE_EMPTY)
			perrfndev(g2d_dev, "Unexpected buftype %d is ignored", buffer_type);
		return;
	}

	for (i = 0; i < num_buffer; i++)
		put_func(g2d_dev, &buffer[i], dir);
}

static void g2d_put_image(struct g2d_device *g2d_dev, struct g2d_layer *layer,
			  enum dma_data_direction dir)
{
	g2d_put_buffer(g2d_dev, layer->buffer_type, layer->buffer, layer->num_buffers, dir);

	if (layer->fence)
		dma_fence_remove_callback(layer->fence, &layer->fence_cb);
	dma_fence_put(layer->fence);

	layer->buffer_type = G2D_BUFTYPE_NONE;
}

#define IS_DST_SBWC(task) \
	IS_SBWC((task)->target.commands[G2DSFR_IMG_COLORMODE].value)

static int g2d_get_source(struct g2d_device *g2d_dev, struct g2d_task *task,
			  struct g2d_layer *layer, struct g2d_layer_data *data,
			  int index)
{
	int ret;

	layer->flags = data->flags;
	layer->buffer_type = data->buffer_type;
	layer->fence = NULL;

	if (!G2D_BUFTYPE_VALID(layer->buffer_type)) {
		perrfndev(g2d_dev, "invalid buffer type %u specified",
			  layer->buffer_type);
		return -EINVAL;
	}

	if (!g2d_validate_source_commands(g2d_dev, task, index, layer, &task->target))
		return -EINVAL;

	/* color fill has no buffer */
	if (layer->buffer_type == G2D_BUFTYPE_EMPTY) {
		if (layer->flags & G2D_LAYERFLAG_COLORFILL) {
			layer->num_buffers = 0;
			layer->flags &= ~G2D_LAYERFLAG_ACQUIRE_FENCE;
			/* g2d_prepare_source() always successes for colofill */
			g2d_prepare_source(task, layer, index);
			return 0;
		}

		perrfndev(g2d_dev, "DMA layer %d has no buffer - flags: %#x",
			  index, layer->flags);
		return -EINVAL;
	}

	if (index == 0 && (IS_HWFC(task->flags) || IS_DST_SBWC(task))) {
		perrfndev(g2d_dev, "Layer0 can be used as a constant layer %s",
			  IS_HWFC(task->flags) ? "HWFC" : "SBWC for encoding");
		return -EINVAL;
	}

	ret = g2d_prepare_buffer(g2d_dev, layer, data, false);
	if (ret)
		return ret;

	layer->fence = g2d_get_acquire_fence(g2d_dev, layer, data->fence);
	if (IS_ERR(layer->fence)) {
		perrfndev(g2d_dev, "Invalid fence fd %d on source[%d]", data->fence, index);
		return PTR_ERR(layer->fence);
	}

	ret = g2d_get_buffer(g2d_dev, NULL, layer, data, DMA_TO_DEVICE);
	if (ret)
		goto err_buffer;

	if (!g2d_prepare_source(task, layer, index)) {
		ret = -EINVAL;
		perrfndev(g2d_dev, "Failed to prepare source layer %d", index);
		goto err_prepare;
	}

	return 0;
err_prepare:
	g2d_put_buffer(g2d_dev, layer->buffer_type, layer->buffer,
		       layer->num_buffers, DMA_TO_DEVICE);
err_buffer:
	if (layer->fence)
		dma_fence_remove_callback(layer->fence, &layer->fence_cb);
	dma_fence_put(layer->fence); /* dma_fence_put() checkes NULL */

	return ret;
}

static int g2d_get_sources(struct g2d_device *g2d_dev, struct g2d_task *task,
			   struct g2d_layer_data __user *src)
{
	unsigned int i;
	int ret;

	for (i = 0; i < task->num_source; i++, src++) {
		struct g2d_layer_data data;

		if (copy_from_user(&data, src, sizeof(*src))) {
			perrfndev(g2d_dev, "Failed to read source image data %d/%d",
				  i, task->num_source);
			ret = -EFAULT;
			break;
		}

		ret = g2d_get_source(g2d_dev, task, &task->source[i], &data, i);
		if (ret)
			break;
	}

	if (i < task->num_source) {
		while (i-- > 0)
			g2d_put_image(g2d_dev, &task->source[i], DMA_TO_DEVICE);
	}

	return ret;
}

static int g2d_get_target(struct g2d_device *g2d_dev, struct g2d_context *ctx,
			  struct g2d_task *task, struct g2d_layer_data *data)
{
	struct g2d_layer *target = &task->target;
	int ret;

	target->flags = data->flags;
	target->fence = NULL;

	target->buffer_type = data->buffer_type;

	if (!G2D_BUFTYPE_VALID(target->buffer_type)) {
		perrfndev(g2d_dev, "invalid buffer type %u specified to target",
			  target->buffer_type);
		return -EINVAL;
	}

	if (IS_HWFC(task->flags)) {
		struct g2d_task *ptask;
		unsigned long flags;
		struct dma_buf *dmabuf;

		target->buffer_type = G2D_BUFTYPE_DMABUF;

		/*
		 * The index from repeater driver used on both buffer index and
		 * job id, and this index is managed by repeater driver to
		 * avoid overwriting the buffer index and job id while MFC is
		 * running.
		 */
		ret = hwfc_get_valid_buffer(&task->bufidx);
		if (ret < 0) {
			perrfndev(g2d_dev, "Failed to get buffer for HWFC");
			return ret;
		}

		spin_lock_irqsave(&task->g2d_dev->lock_task, flags);

		ptask = task->g2d_dev->tasks;

		while (ptask) {
			if (ptask == task) {
				ptask = ptask->next;
				continue;
			}
			if ((g2d_task_id(task) == task->bufidx) && !is_task_state_idle(ptask)) {
				perrfndev(g2d_dev, "The %d task is not idle", task->bufidx);

				spin_unlock_irqrestore(&task->g2d_dev->lock_task, flags);

				return -EINVAL;
			}
			ptask = ptask->next;
		}

		g2d_task_set_id(task, task->bufidx);

		spin_unlock_irqrestore(&task->g2d_dev->lock_task, flags);

		/*
		 * HWFC buffer is not given by the userspace. So, userspace does
		 * not fill buffer information. Let's initialize buffer
		 * information with the HWFC buffer.
		 */
		dmabuf = g2d_get_hwfc_dmabuf(ctx, task);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);

		data->num_buffers = 1;
		data->buffer[0].dmabuf.offset = 0;
		data->buffer[0].length = dmabuf->size;

		dma_buf_put(dmabuf);

		g2d_stamp_task(task, G2D_STAMP_STATE_HWFCBUF, task->bufidx);
	}

	if (target->buffer_type == G2D_BUFTYPE_EMPTY) {
		perrfndev(g2d_dev, "target has no buffer - flags: %#x",
			  task->flags);
		return -EINVAL;
	}

	if (!g2d_validate_target_commands(g2d_dev, task))
		return -EINVAL;

	ret = g2d_prepare_buffer(g2d_dev, target, data, true);
	if (ret)
		return ret;

	target->fence = g2d_get_acquire_fence(g2d_dev, target, data->fence);
	if (IS_ERR(target->fence)) {
		perrfndev(g2d_dev, "Invalid taret fence fd %d", data->fence);
		return PTR_ERR(target->fence);
	}

	ret = g2d_get_buffer(g2d_dev, ctx, target, data, DMA_FROM_DEVICE);
	if (ret)
		goto err_buffer;

	if (!g2d_prepare_target(task)) {
		ret = -EINVAL;
		perrfndev(g2d_dev, "Failed to prepare target layer");
		goto err_prepare;
	}

	return 0;
err_prepare:
	g2d_put_buffer(g2d_dev, target->buffer_type, target->buffer,
		       target->num_buffers, DMA_FROM_DEVICE);
err_buffer:
	if (target->fence)
		dma_fence_remove_callback(target->fence, &target->fence_cb);
	dma_fence_put(target->fence); /* dma_fence_put() checkes NULL */

	return ret;
}

static inline bool caps_has_afbcv12(unsigned long caps)
{
	return !!(caps & G2D_DEVICE_CAPS_AFBC_V12);
}

#define TARGET_OFFSET		0x120
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
#define LAYER_OFFSET(idx)	((2 + (idx)) << 8)
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#define LAYER_OFFSET(idx)       ((3 + (idx)) << 8)
#endif

enum {
	G2D_AFBC_REG_VALUE_BLK_16x16 = 0,
	G2D_AFBC_REG_VALUE_BLK_32x8 = 1,
};

static unsigned int update_afbc_supblock_size(struct g2d_task *task,
					      struct g2d_reg regs[])
{
	unsigned int i;
	unsigned int count = 0, index = task->sec.cmd_count;

	if (!caps_has_afbcv12(task->g2d_dev->caps))
		return 0;

	if (IS_AFBC(task->target.commands[G2DSFR_IMG_COLORMODE].value) &&
	    !!(task->target.flags & G2D_LAYERFLAG_AFBC_LANDSCAPE)) {
		regs[count + index].offset = TARGET_OFFSET + 0x94;
		regs[count + index].value = G2D_AFBC_REG_VALUE_BLK_32x8;
		count++;
	}

	for (i = 0; i < task->num_source; i++) {
		struct g2d_layer *layer = &task->source[i];

		if (IS_AFBC(layer->commands[G2DSFR_IMG_COLORMODE].value) &&
		    !!(layer->flags & G2D_LAYERFLAG_AFBC_LANDSCAPE)) {
			regs[count + index].offset = LAYER_OFFSET(i) + 0xB4;
			regs[count + index].value = G2D_AFBC_REG_VALUE_BLK_32x8;
			count++;
		}
	}

	return count;
}

int g2d_get_userdata(struct g2d_device *g2d_dev, struct g2d_context *ctx,
		     struct g2d_task *task, struct g2d_task_data *data)
{
	unsigned int i;
	int ret;
	struct g2d_reg *cmdaddr = page_address(task->cmd_page);

	/* invalid range check */
	if (data->num_source < 1 || data->num_source > g2d_dev->max_layers) {
		perrfndev(g2d_dev, "Invalid number of source images %u", data->num_source);
		return -EINVAL;
	}

	if (data->priority > G2D_MAX_PRIORITY) {
		perrfndev(g2d_dev, "Invalid number of priority %u", data->priority);
		return -EINVAL;
	}

	task->flags = data->flags;
	task->num_source = data->num_source;

	ret = g2d_import_commands(g2d_dev, task, data, task->num_source, cmdaddr);
	if (ret < 0)
		return ret;

	ret = g2d_get_target(g2d_dev, ctx, task, &data->target);
	if (ret)
		return ret;

	ret = g2d_get_sources(g2d_dev, task, data->source);
	if (ret)
		goto err_src;
	task->sec.cmd_count += update_afbc_supblock_size(task, cmdaddr);

	task->release_fence = g2d_create_release_fence(g2d_dev, task, data);
	if (IS_ERR(task->release_fence)) {
		ret = PTR_ERR(task->release_fence);
		goto err_fence;
	}

	return 0;
err_fence:
	for (i = 0; i < task->num_source; i++)
		g2d_put_image(g2d_dev, &task->source[i], DMA_TO_DEVICE);
err_src:
	g2d_put_image(g2d_dev, &task->target, DMA_FROM_DEVICE);

	return ret;
}

void g2d_put_images(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	unsigned int i;

	if (task->release_fence) {
		if (is_task_state_error(task))
			dma_fence_set_error(task->release_fence->fence, -EIO);

		dma_fence_signal(task->release_fence->fence);
		fput(task->release_fence->file);
	}

	for (i = 0; i < task->num_source; i++)
		g2d_put_image(g2d_dev, &task->source[i], DMA_TO_DEVICE);

	g2d_put_image(g2d_dev, &task->target, DMA_FROM_DEVICE);
	task->num_source = 0;
}

int g2d_wait_put_user(struct g2d_device *g2d_dev, struct g2d_task *task,
		      struct g2d_task_data __user *uptr, u32 userflag)
{
	int ret;

	if (!g2d_task_wait_completion(task)) {
		userflag |= G2D_FLAG_ERROR;
		ret = put_user(userflag, &uptr->flags);
	} else {
		u32 laptime_in_usec = (u32)ktime_us_delta(task->ktime_end,
						  task->ktime_begin);

		ret = put_user(laptime_in_usec, &uptr->laptime_in_usec);
	}

	g2d_put_images(g2d_dev, task);

	g2d_put_free_task(g2d_dev, task);

	return ret;
}
