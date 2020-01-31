// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_gem.c
 *
 * Copyright (C) 2019 Samsung Electronics Co.Ltd
 * Authors:
 *	Jiun Yu <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#define pr_fmt(fmt)  "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mm_types.h>
#include <linux/ion_exynos.h>
#include <exynos_drm_dsim.h>
#include <exynos_drm_gem.h>
#include <exynos_drm_dsim.h>

struct exynos_drm_gem *exynos_drm_gem_alloc(struct drm_device *dev,
					    size_t size, unsigned int flags)
{
	struct exynos_drm_gem *exynos_gem_obj;

	exynos_gem_obj = kzalloc(sizeof(*exynos_gem_obj), GFP_KERNEL);
	if (!exynos_gem_obj)
		return ERR_PTR(-ENOMEM);

	exynos_gem_obj->flags = flags;

	/* no need to release initialized private gem object */
	drm_gem_private_object_init(dev, &exynos_gem_obj->base, size);

	pr_debug("allocated %zu bytes with flags %#x\n", size, flags);

	return exynos_gem_obj;
}

struct drm_gem_object *
exynos_drm_gem_prime_import_sg_table(struct drm_device *dev,
				     struct dma_buf_attachment *attach,
				     struct sg_table *sgt)
{
	const unsigned long size = attach->dmabuf->size;
	struct exynos_drm_gem *exynos_gem_obj =
		exynos_drm_gem_alloc(dev, size, 0);

	if (IS_ERR(exynos_gem_obj))
		return ERR_CAST(exynos_gem_obj);

	exynos_gem_obj->sgt = sgt;
	exynos_gem_obj->dma_addr = ion_iovmm_map(attach, 0, size,
						 DMA_TO_DEVICE, 0);
	if (IS_ERR_VALUE(exynos_gem_obj->dma_addr)) {
		pr_err("Failed to allocate IOVM\n");
		kfree(exynos_gem_obj);
		return ERR_PTR(exynos_gem_obj->dma_addr);
	}

	pr_debug("dma_addr(0x%llx)\n", exynos_gem_obj->dma_addr);

	return &exynos_gem_obj->base;
}

static void exynos_drm_gem_unmap(struct exynos_drm_gem *exynos_gem_obj,
				 struct device *client)
{
	struct dma_buf_attachment *attach = exynos_gem_obj->base.import_attach;

	/* nothing to do for color map buffers */
	if (exynos_gem_obj->flags & EXYNOS_DRM_GEM_FLAG_COLORMAP)
		return;

	if (client)
		ion_iovmm_unmap(attach, exynos_gem_obj->dma_addr);

	if (exynos_gem_obj->kaddr)
		dma_buf_vunmap(attach->dmabuf, exynos_gem_obj->kaddr);

	pr_debug("unmapped the dma address and the attachment\n");
}

void exynos_drm_gem_free_object(struct drm_gem_object *obj)
{
	struct exynos_drm_gem *exynos_gem_obj = to_exynos_gem(obj);
	const struct exynos_drm_private *priv = obj->dev->dev_private;

	exynos_drm_gem_unmap(exynos_gem_obj, priv->iommu_client);

	if (obj->import_attach)
		drm_prime_gem_destroy(obj, exynos_gem_obj->sgt);

	drm_gem_object_release(&exynos_gem_obj->base);
	kfree(exynos_gem_obj);
}

static int exynos_drm_gem_create(struct drm_device *dev, struct drm_file *filep,
				 size_t size, unsigned int flags,
				 unsigned int *gem_handle)
{
	struct exynos_drm_gem *exynos_gem_obj;
	const char *heap_name = "ion_system_heap";
	struct dma_buf *dmabuf;
	struct drm_gem_object *obj;
	int ret;

	if (flags & EXYNOS_DRM_GEM_FLAG_COLORMAP) {
		pr_err("unsupported color map gem creation\n");
		return -EINVAL;
	}

	dmabuf = ion_alloc_dmabuf(heap_name, size, 0);
	if (IS_ERR_OR_NULL(dmabuf)) {
		pr_err("ION Failed to alloc %#zx bytes\n", size);
		return PTR_ERR(dmabuf);
	}

	obj = exynos_drm_gem_prime_import(dev, dmabuf);
	if (IS_ERR(obj)) {
		pr_err("Unable to import created ION buffer\n");
		ret = PTR_ERR(obj);
		goto err_import;
	}

	ret = drm_gem_handle_create(filep, obj, gem_handle);
	if (ret) {
		pr_err("Failed to create a handle of GEM\n");
		goto err_handle;
	}

	/* drop reference from import - handle holds it now. */
	drm_gem_object_unreference_unlocked(&exynos_gem_obj->base);

	return 0;
err_handle:
	exynos_drm_gem_free_object(obj);
err_import:
	dma_buf_put(dmabuf);

	return ret;
}

int exynos_drm_gem_dumb_create(struct drm_file *file_priv,
			       struct drm_device *dev,
			       struct drm_mode_create_dumb *args)
{
	unsigned int handle;
	int ret;

	args->pitch = args->width * DIV_ROUND_UP(args->bpp, 8);
	args->size = PAGE_ALIGN(args->pitch * args->height);

	ret = exynos_drm_gem_create(dev, file_priv, args->size,
				    0, &handle);
	if (ret) {
		pr_err("Failed to create dumb of %llu bytes (%ux%u/%ubpp)\n",
			  args->size, args->width, args->height, args->bpp);
		return ret;
	}

	args->handle = handle;

	return 0;
}

struct drm_gem_object *exynos_drm_gem_prime_import(struct drm_device *dev,
						   struct dma_buf *dma_buf)
{
	struct exynos_drm_private *priv = dev->dev_private;

	return drm_gem_prime_import_dev(dev, dma_buf, priv->iommu_client);
}

int exynos_drm_gem_mmap_object(struct exynos_drm_gem *exynos_gem_obj,
			       struct vm_area_struct *vma)
{
	struct dma_buf_attachment *attach = exynos_gem_obj->base.import_attach;
	int ret;


	if (unlikely(!attach)) {
		pr_err("Invalid mmap with empty attach!\n");
		return (-EINVAL);
	}

	ret = dma_buf_mmap(attach->dmabuf, vma, 0);
	if (ret)
		pr_err("Failed to mmap imported buffer: %d\n", ret);

	return ret;
}

int exynos_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	/* NOTE: drm_gem_mmap() always set the vm_page_prot to writecombine */
	ret = drm_gem_mmap(filp, vma);
	if (ret < 0)
		goto err;

	ret =
	    exynos_drm_gem_mmap_object(to_exynos_gem(vma->vm_private_data),
				       vma);
	if (ret)
		goto err_mmap;

	pr_debug("mmaped the offset %lu of size %lu to %#lx\n",
			 vma->vm_pgoff, vma->vm_end - vma->vm_start,
			 vma->vm_start);

	return 0;
err_mmap:
	/* release the resources referenced by drm_gem_mmap() */
	drm_gem_vm_close(vma);
err:
	pr_err("Failed to mmap with offset %lu.\n", vma->vm_pgoff);

	return ret;
}

static int exynos_drm_gem_offset(struct drm_device *dev, struct drm_file *filep,
				 unsigned int handle, uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret = 0;

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(filep, handle);
	if (!obj) {
		pr_err("Failed to lookup gem object from handle %u.\n",
			  handle);
		ret = -EINVAL;
		goto unlock;
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret) {
		pr_err("Failed to create mmap fake offset for handle %u\n",
			  handle);
		goto out;
	}

	*offset = drm_vma_node_offset_addr(&obj->vma_node);
out:
	drm_gem_object_unreference(obj);
unlock:
	mutex_unlock(&dev->struct_mutex);

	return ret;
}

int exynos_drm_gem_dumb_map_offset(struct drm_file *file_priv,
				   struct drm_device *dev, uint32_t handle,
				   uint64_t *offset)
{
	int ret;

	ret = exynos_drm_gem_offset(dev, file_priv, handle, offset);
	if (!ret)
		pr_debug("obtained fake mmap offset %llu from handle %u\n",
			      *offset, handle);

	return ret;
}

void exynos_drm_gem_print_info(struct drm_printer *p, unsigned int indent,
			       const struct drm_gem_object *obj)
{
	struct exynos_drm_gem *exynos_gem_obj = to_exynos_gem(obj);
	const char *buf_type = NULL;

	if (exynos_gem_obj->flags & EXYNOS_DRM_GEM_FLAG_COLORMAP)
		buf_type = "colormap";

	if (buf_type)
		drm_printf_indent(p, indent, "type=%s\n", buf_type);
}
