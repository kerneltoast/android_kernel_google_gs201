/*
 * drivers/media/platform/exynos/mfc/mfc_mem.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/property.h>
#include <linux/dma-buf.h>
#include <linux/iommu.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-heap.h>

#include "mfc_mem.h"

struct vb2_mem_ops *mfc_mem_ops(void)
{
	return (struct vb2_mem_ops *)&vb2_dma_sg_memops;
}

int mfc_mem_get_user_shared_handle(struct mfc_ctx *ctx,
	struct mfc_user_shared_handle *handle, char *name)
{
	struct iosys_map map;
	int ret = 0;

	handle->dma_buf = dma_buf_get(handle->fd);
	if (IS_ERR(handle->dma_buf)) {
		mfc_ctx_err("[MEMINFO][SH][%s] Failed to import fd\n", name);
		ret = PTR_ERR(handle->dma_buf);
		goto import_dma_fail;
	}

	if (handle->dma_buf->size < handle->data_size) {
		mfc_ctx_err("[MEMINFO][SH][%s] User-provided dma_buf size(%zu) is smaller than required size(%zu)\n",
				name, handle->dma_buf->size, handle->data_size);
		ret = -EINVAL;
		goto dma_buf_size_fail;
	}

	ret = dma_buf_vmap(handle->dma_buf, &map);
	if (ret) {
		mfc_ctx_err("Failed to get kernel virtual address\n");
		goto map_kernel_fail;
	}
	handle->vaddr = map.vaddr;

	mfc_debug(2, "[MEMINFO][SH][%s] shared handle fd: %d, vaddr: %pK, buf size: %zu, data size: %zu\n",
			name, handle->fd, handle->vaddr, handle->dma_buf->size, handle->data_size);

	return 0;

map_kernel_fail:
	handle->vaddr = NULL;
dma_buf_size_fail:
	dma_buf_put(handle->dma_buf);
import_dma_fail:
	handle->dma_buf = NULL;
	handle->fd = -1;
	return ret;
}

void mfc_mem_cleanup_user_shared_handle(struct mfc_ctx *ctx,
		struct mfc_user_shared_handle *handle)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(handle->vaddr);

	if (handle->vaddr)
		dma_buf_vunmap(handle->dma_buf, &map);
	if (handle->dma_buf)
		dma_buf_put(handle->dma_buf);

	handle->data_size = 0;
	handle->dma_buf = NULL;
	handle->vaddr = NULL;
	handle->fd = -1;
}

static int mfc_mem_dma_heap_alloc(struct mfc_dev *dev,
		struct mfc_special_buf *special_buf)
{
	struct dma_heap *dma_heap;
	const char *heapname;

	switch (special_buf->buftype) {
	case MFCBUF_DRM_FW:
		heapname = "mfc_fw-secure";
		break;
	case MFCBUF_NORMAL_FW:
	case MFCBUF_NORMAL:
		heapname = "system-uncached";
		break;
	case MFCBUF_DRM:
		heapname = "vframe-secure";
		break;
	default:
		return -EINVAL;
	}

	/* control by DMA heap API */
	dma_heap = dma_heap_find(heapname);
	if (!dma_heap) {
		mfc_dev_err("Failed to get DMA heap (name: %s)\n", heapname);
		goto err_dma_heap_find;
	}

	special_buf->dma_buf = dma_heap_buffer_alloc(dma_heap,
			special_buf->size, 0, 0);
	if (IS_ERR(special_buf->dma_buf)) {
		mfc_dev_err("Failed to allocate buffer (err %ld)\n",
				PTR_ERR(special_buf->dma_buf));
		goto err_dma_heap_alloc;
	}

	/* control by DMA buf API */
	special_buf->attachment = dma_buf_attach(special_buf->dma_buf,
					dev->device);
	if (IS_ERR(special_buf->attachment)) {
		mfc_dev_err("Failed to get dma_buf_attach (err %ld)\n",
				PTR_ERR(special_buf->attachment));
		goto err_attach;
	}

	special_buf->sgt = dma_buf_map_attachment(special_buf->attachment,
			DMA_BIDIRECTIONAL);
	if (IS_ERR(special_buf->sgt)) {
		mfc_dev_err("Failed to get sgt (err %ld)\n",
				PTR_ERR(special_buf->sgt));
		goto err_map;
	}

	special_buf->daddr = sg_dma_address(special_buf->sgt->sgl);
	if (IS_ERR_VALUE(special_buf->daddr)) {
		mfc_dev_err("Failed to get iova (err 0x%p)\n",
				&special_buf->daddr);
		goto err_daddr;
	}

	/* Can't map secure memory */
	if ((special_buf->buftype != MFCBUF_DRM) &&
	    (special_buf->buftype != MFCBUF_DRM_FW)) {
		struct iosys_map map;
		int ret;

		ret = dma_buf_vmap(special_buf->dma_buf, &map);
		if (ret) {
			mfc_dev_err("Failed to get kernel virtual adddress\n");
			goto err_vaddr;
		}
		special_buf->vaddr = map.vaddr;
	}

	special_buf->paddr = page_to_phys(sg_page(special_buf->sgt->sgl));

	dma_heap_put(dma_heap);

	return 0;
err_vaddr:
	special_buf->vaddr = NULL;
err_daddr:
	special_buf->daddr = 0;
	dma_buf_unmap_attachment(special_buf->attachment, special_buf->sgt,
				 DMA_BIDIRECTIONAL);
err_map:
	special_buf->sgt = NULL;
	dma_buf_detach(special_buf->dma_buf, special_buf->attachment);
err_attach:
	special_buf->attachment = NULL;
	dma_buf_put(special_buf->dma_buf);
err_dma_heap_alloc:
	dma_heap_put(dma_heap);
	special_buf->dma_buf = NULL;
err_dma_heap_find:
	return -ENOMEM;
}

void mfc_mem_dma_heap_free(struct mfc_special_buf *special_buf)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(special_buf->vaddr);

	if (special_buf->vaddr)
		dma_buf_vunmap(special_buf->dma_buf, &map);
	if (special_buf->sgt)
		dma_buf_unmap_attachment(special_buf->attachment,
					 special_buf->sgt, DMA_BIDIRECTIONAL);
	if (special_buf->attachment)
		dma_buf_detach(special_buf->dma_buf, special_buf->attachment);
	if (special_buf->dma_buf)
		dma_buf_put(special_buf->dma_buf);

	special_buf->dma_buf = NULL;
	special_buf->attachment = NULL;
	special_buf->sgt = NULL;
	special_buf->daddr = 0;
	special_buf->vaddr = NULL;

}

int mfc_mem_special_buf_alloc(struct mfc_dev *dev,
		struct mfc_special_buf *special_buf)
{
	int ret;

	switch (special_buf->buftype) {
	case MFCBUF_DRM_FW:
	case MFCBUF_NORMAL_FW:
	case MFCBUF_DRM:
	case MFCBUF_NORMAL:
		ret = mfc_mem_dma_heap_alloc(dev, special_buf);
		break;
	default:
		mfc_dev_err("not supported mfc mem type: %d\n", special_buf->buftype);
		return -EINVAL;
	}

	return ret;
}

void mfc_mem_special_buf_free(struct mfc_dev *dev, struct mfc_special_buf *special_buf)
{
	switch (special_buf->buftype) {
	case MFCBUF_DRM_FW:
	case MFCBUF_NORMAL_FW:
	case MFCBUF_DRM:
	case MFCBUF_NORMAL:
		mfc_mem_dma_heap_free(special_buf);
		break;
	default:
		break;
	}

	return;
}

void mfc_bufcon_put_daddr(struct mfc_ctx *ctx, struct mfc_buf *mfc_buf, int plane)
{
	int i;

	for (i = 0; i < mfc_buf->num_valid_bufs; i++) {
		if (mfc_buf->addr[i][plane]) {
			mfc_debug(4, "[BUFCON] put batch buf addr[%d][%d]: 0x%08llx\n",
					i, plane, mfc_buf->addr[i][plane]);
		}
		if (mfc_buf->attachments[i][plane])
			dma_buf_detach(mfc_buf->dmabufs[i][plane], mfc_buf->attachments[i][plane]);
		if (mfc_buf->dmabufs[i][plane])
			dma_buf_put(mfc_buf->dmabufs[i][plane]);

		mfc_buf->addr[i][plane] = 0;
		mfc_buf->attachments[i][plane] = NULL;
		mfc_buf->dmabufs[i][plane] = NULL;
	}
}

#if IS_ENABLED(CONFIG_MFC_USE_DMABUF_CONTAINER)
int mfc_bufcon_get_daddr(struct mfc_ctx *ctx, struct mfc_buf *mfc_buf,
					struct dma_buf *bufcon_dmabuf, int plane)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_raw_info *raw = &ctx->raw_buf;
	int i, j = 0;
	u32 mask;

	if (dmabuf_container_get_mask(bufcon_dmabuf, &mask)) {
		mfc_ctx_err("[BUFCON] it is not buffer container\n");
		return -1;
	}

	if (mask == 0) {
		mfc_ctx_err("[BUFCON] number of valid buffers is zero\n");
		return -1;
	}

	mfc_debug(3, "[BUFCON] bufcon mask info %#x\n", mask);

	for (i = 0; i < mfc_buf->num_bufs_in_batch; i++) {
		if ((mask & (1 << i)) == 0) {
			mfc_debug(4, "[BUFCON] unmasked buf[%d]\n", i);
			continue;
		}

		mfc_buf->dmabufs[j][plane] = dmabuf_container_get_buffer(bufcon_dmabuf, i);
		if (IS_ERR(mfc_buf->dmabufs[i][plane])) {
			snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
				"[BUFCON] Failed to get dma_buf (err %ld)",
				PTR_ERR(mfc_buf->dmabufs[i][plane]));
			mfc_ctx_err("%s", dev->dev_crash_info);
			call_dop(dev, dump_and_stop_debug_mode, dev);
			goto err_get_daddr;
		}

		mfc_buf->attachments[j][plane] = dma_buf_attach(mfc_buf->dmabufs[i][plane], dev->device);
		if (IS_ERR(mfc_buf->attachments[i][plane])) {
			snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
				"[BUFCON] Failed to get dma_buf_attach (err %ld)",
				PTR_ERR(mfc_buf->attachments[i][plane]));
			mfc_ctx_err("%s", dev->dev_crash_info);
			call_dop(dev, dump_and_stop_debug_mode, dev);
			goto err_get_daddr;
		}

		mfc_buf->addr[j][plane] = ion_iovmm_map(mfc_buf->attachments[i][plane], 0,
				raw->plane_size[plane], DMA_BIDIRECTIONAL, 0);
		if (IS_ERR_VALUE(mfc_buf->addr[i][plane])) {
			snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
				"[BUFCON] Failed to allocate iova (err %pa)",
				&mfc_buf->addr[i][plane]);
			mfc_ctx_err("%s", dev->dev_crash_info);
			call_dop(dev, dump_and_stop_debug_mode, dev);
			goto err_get_daddr;
		}

		mfc_debug(4, "[BUFCON] get batch buf addr[%d][%d]: 0x%08llx, size: %d\n",
				j, plane, mfc_buf->addr[j][plane], raw->plane_size[plane]);
		j++;
	}

	mfc_buf->num_valid_bufs = j;
	mfc_debug(3, "[BUFCON] batch buffer has %d buffers\n", mfc_buf->num_valid_bufs);

	return 0;

err_get_daddr:
	mfc_bufcon_put_daddr(ctx, mfc_buf, plane);
	return -1;
}
#endif

void mfc_put_iovmm(struct mfc_ctx *ctx, struct dpb_table *dpb, int num_planes, int index)
{
	struct mfc_dev *dev = ctx->dev;
	int i;

	MFC_TRACE_CTX("DPB[%d] fd: %d addr: %#llx put(%d)\n",
			index, dpb[index].fd[0], dpb[index].addr[0], dpb[index].mapcnt);

	for (i = 0; i < num_planes; i++) {
#if IS_ENABLED(CONFIG_MFC_USE_DMA_SKIP_LAZY_UNMAP)
		if (dpb[index].attach[i] && (dev->skip_lazy_unmap || ctx->skip_lazy_unmap)) {
			dpb[index].attach[i]->dma_map_attrs |= DMA_ATTR_SKIP_LAZY_UNMAP;
			mfc_debug(4, "[LAZY_UNMAP] skip for dst plane[%d]\n", i);
		}
#endif

		if (dpb[index].addr[i])
			mfc_debug(2, "[IOVMM] index %d buf[%d] fd: %d addr: %#llx\n",
					index, i, dpb[index].fd[i], dpb[index].addr[i]);
		if (dpb[index].sgt[i])
			dma_buf_unmap_attachment(dpb[index].attach[i], dpb[index].sgt[i],
					DMA_BIDIRECTIONAL);
		if (dpb[index].attach[i])
			dma_buf_detach(dpb[index].dmabufs[i], dpb[index].attach[i]);
		if (dpb[index].dmabufs[i])
			dma_buf_put(dpb[index].dmabufs[i]);

		dpb[index].fd[i] = -1;
		dpb[index].addr[i] = 0;
		dpb[index].attach[i] = NULL;
		dpb[index].dmabufs[i] = NULL;
		dpb[index].sgt[i] = NULL;
	}

	dpb[index].new_fd = -1;
	dpb[index].mapcnt--;
	mfc_debug(2, "[IOVMM] index %d mapcnt %d\n", index, dpb[index].mapcnt);

	if (dpb[index].mapcnt != 0) {
		snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
			"[IOVMM] DPB[%d] %#llx invalid mapcnt %d\n",
			index, dpb[index].addr[0], dpb[index].mapcnt);
		mfc_ctx_err("%s", dev->dev_crash_info);
		call_dop(dev, dump_and_stop_debug_mode, dev);
		dpb[index].mapcnt = 0;
	}
}

void mfc_get_iovmm(struct mfc_ctx *ctx, struct vb2_buffer *vb, struct dpb_table *dpb)
{
	struct mfc_dev *dev = ctx->dev;
	int i, mem_get_count = 0;
	struct mfc_buf *mfc_buf = vb_to_mfc_buf(vb);
	int index = mfc_buf->dpb_index;

	if (dpb[index].mapcnt != 0) {
		snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
			"[IOVMM] DPB[%d] %#llx invalid mapcnt %d\n",
			index, dpb[index].addr[0], dpb[index].mapcnt);
		mfc_ctx_err("%s", dev->dev_crash_info);
		call_dop(dev, dump_and_stop_debug_mode, dev);
	}

	for (i = 0; i < ctx->dst_fmt->mem_planes; i++) {
		mem_get_count++;

		dpb[index].fd[i] = vb->planes[i].m.fd;

		dpb[index].dmabufs[i] = dma_buf_get(vb->planes[i].m.fd);
		if (IS_ERR(dpb[index].dmabufs[i])) {
			mfc_ctx_err("[IOVMM] Failed to dma_buf_get (err %ld)\n",
					PTR_ERR(dpb[index].dmabufs[i]));
			dpb[index].dmabufs[i] = NULL;
			goto err_iovmm;
		}

		dpb[index].attach[i] = dma_buf_attach(dpb[index].dmabufs[i], dev->device);
		if (IS_ERR(dpb[index].attach[i])) {
			mfc_ctx_err("[IOVMM] Failed to get dma_buf_attach (err %ld)\n",
					PTR_ERR(dpb[index].attach[i]));
			dpb[index].attach[i] = NULL;
			goto err_iovmm;
		}

		dpb[index].sgt[i] = dma_buf_map_attachment(dpb[index].attach[i],
				DMA_BIDIRECTIONAL);
		if (IS_ERR(dpb[index].sgt[i])) {
			mfc_ctx_err("[IOVMM] Failed to get sgt (err %ld)\n",
					PTR_ERR(dpb[index].sgt[i]));
			dpb[index].sgt[i] = NULL;
			goto err_iovmm;
		}

		dpb[index].addr[i] = sg_dma_address(dpb[index].sgt[i]->sgl);
		if (IS_ERR_VALUE(dpb[index].addr[i])) {
			mfc_ctx_err("[IOVMM] Failed to get iova (err 0x%p)\n",
					&dpb[index].addr[i]);
			dpb[index].addr[i] = 0;
			goto err_iovmm;
		}

		mfc_debug(2, "[IOVMM] index %d buf[%d] fd: %d addr: %#llx\n",
				index, i, dpb[index].fd[i], dpb[index].addr[i]);
	}

	dpb[index].paddr = page_to_phys(sg_page(dpb[index].sgt[0]->sgl));
	mfc_debug(2, "[DPB] dpb index [%d][%d] paddr %#llx daddr %#llx\n",
			mfc_buf->vb.vb2_buf.index,
			index, dpb[index].paddr, dpb[index].addr[0]);

	dpb[index].mapcnt++;
	mfc_debug(2, "[IOVMM] index %d mapcnt %d\n", index, dpb[index].mapcnt);
	MFC_TRACE_CTX("DPB[%d] fd: %d addr: %#llx get(%d)\n",
			index, dpb[index].fd[0], dpb[index].addr[0], dpb[index].mapcnt);

	return;

err_iovmm:
	dpb[index].mapcnt++;
	mfc_put_iovmm(ctx, dpb, mem_get_count, index);
}

void mfc_init_dpb_table(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	int index, plane;

	for (index = 0; index < MFC_MAX_DPBS; index++) {
		for (plane = 0; plane < MFC_MAX_PLANES; plane++) {
			dec->dpb[index].fd[plane] = -1;
			dec->dpb[index].addr[plane] = 0;
			dec->dpb[index].attach[plane] = NULL;
			dec->dpb[index].dmabufs[plane] = NULL;
		}
		dec->dpb[index].new_fd = -1;
		dec->dpb[index].mapcnt = 0;
		dec->dpb[index].queued = 0;
	}
}

void mfc_cleanup_iovmm(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	int i;

	mutex_lock(&dec->dpb_mutex);

	for (i = 0; i < MFC_MAX_DPBS; i++) {
		dec->dpb[i].paddr = 0;
		dec->dpb[i].ref = 0;
		if (dec->dpb[i].mapcnt == 0) {
			continue;
		} else if (dec->dpb[i].mapcnt == 1) {
			mfc_put_iovmm(ctx, dec->dpb, ctx->dst_fmt->mem_planes, i);
		} else {
			snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
					"[IOVMM] DPB[%d] %pad invalid mapcnt %d\n",
					i, &dec->dpb[i].addr[0], dec->dpb[i].mapcnt);
			mfc_ctx_err("%s", dev->dev_crash_info);
			MFC_TRACE_CTX("%s", dev->dev_crash_info);
			call_dop(dev, dump_and_stop_debug_mode, dev);
		}
	}

	mutex_unlock(&dec->dpb_mutex);
}

void mfc_cleanup_iovmm_except_used(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	int i;

	mutex_lock(&dec->dpb_mutex);

	for (i = 0; i < MFC_MAX_DPBS; i++) {
		if (dec->dynamic_used & (1UL << i)) {
			continue;
		} else {
			dec->dpb[i].paddr = 0;
			dec->dpb[i].ref = 0;
			if (dec->dpb[i].mapcnt == 0) {
				continue;
			} else if (dec->dpb[i].mapcnt == 1) {
				dec->dpb_table_used &= ~(1UL << i);
				mfc_put_iovmm(ctx, dec->dpb, ctx->dst_fmt->mem_planes, i);
			} else {
				mfc_ctx_err("[IOVMM] DPB[%d] %pad invalid mapcnt %d\n",
						i, &dec->dpb[i].addr[0], dec->dpb[i].mapcnt);
				MFC_TRACE_CTX("DPB[%d] %pad invalid mapcnt %d\n",
						i, &dec->dpb[i].addr[0], dec->dpb[i].mapcnt);
			}
		}
	}

	mutex_unlock(&dec->dpb_mutex);
}

int mfc_iommu_map_firmware(struct mfc_core *core, struct mfc_special_buf *fw_buf)
{
	struct mfc_dev *dev = core->dev;
	struct device_node *node = core->device->of_node;
	dma_addr_t reserved_base;
	const __be32 *prop;

	prop = of_get_property(node, "samsung,iommu-reserved-map", NULL);
	if (!prop) {
		mfc_dev_err("No reserved F/W dma area\n");
		return -ENOENT;
	}

	reserved_base = of_read_number(prop, of_n_addr_cells(node));

	fw_buf->map_size = iommu_map_sg(core->domain, reserved_base,
			fw_buf->sgt->sgl,
			fw_buf->sgt->orig_nents,
			IOMMU_READ|IOMMU_WRITE);
	if (!fw_buf->map_size) {
		mfc_core_err("Failed to map iova (err VA: %pad, PA: %pap)\n",
				&reserved_base, &fw_buf->paddr);
		return -ENOMEM;
	}

	fw_buf->daddr = reserved_base;

	return 0;
}

int mfc_iommu_map_sfr(struct mfc_core *core)
{
	struct device_node *node = core->device->of_node;
	dma_addr_t reserved_base;
	const __be32 *prop;
	size_t reserved_size;
	int n_addr_cells = of_n_addr_cells(node);
	int n_size_cells = of_n_size_cells(node);
	int n_all_cells = n_addr_cells + n_size_cells;
	int i, cnt;

	prop = of_get_property(node, "samsung,iommu-identity-map", &cnt);
	if (!prop) {
		mfc_core_err("No reserved votf SFR area\n");
		return -ENOENT;
	}

	cnt /= sizeof(unsigned int);
	if (cnt % n_all_cells != 0) {
		mfc_core_err("Invalid number(%d) of values\n", cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i += n_all_cells) {
		reserved_base = of_read_number(prop + i, n_addr_cells);
		reserved_size = of_read_number(prop + i + n_addr_cells, n_size_cells);
		if (reserved_base == core->core_pdata->gdc_votf_base) {
			core->has_gdc_votf = 1;
			mfc_core_info("iommu mapped at GDC vOTF SFR %#llx ++ %#zx\n",
					reserved_base, reserved_size);
		} else if (reserved_base == core->core_pdata->dpu_votf_base) {
			core->has_dpu_votf = 1;
			mfc_core_info("iommu mapped at DPU vOTF SFR %#llx ++ %#zx\n",
					reserved_base, reserved_size);
		} else {
			mfc_core_err("iommu mapped at unknown SFR %#llx ++ %#zx\n",
					reserved_base, reserved_size);
			return -EINVAL;
		}
	}

	return 0;
}

void mfc_check_iova(struct mfc_dev *dev)
{
	struct mfc_platdata *pdata = dev->pdata;
	struct mfc_ctx *ctx;
	unsigned long total_iova = 0;

	if (!pdata->iova_threshold)
		return;

	/*
	 * The number of extra dpb is 8
	 * OMX: extra buffer 5, platform buffer 3
	 * Codec2: platform buffer 8
	 */
	list_for_each_entry(ctx, &dev->ctx_list, list)
		total_iova += (ctx->raw_buf.total_plane_size *
				(ctx->dpb_count + MFC_EXTRA_DPB + 3)) / 1024;

	if (total_iova > (pdata->iova_threshold * 1024))
		dev->skip_lazy_unmap = 1;
	else
		dev->skip_lazy_unmap = 0;

	mfc_dev_debug(2, "[LAZY_UNMAP] Now the IOVA for DPB is %lu/%uMB, LAZY_UNMAP %s\n",
			total_iova / 1024, pdata->iova_threshold,
			dev->skip_lazy_unmap ? "disable" : "enable");
}
