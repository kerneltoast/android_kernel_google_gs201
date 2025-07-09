// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeTPU support for dma-buf.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sync_file.h>
#include <linux/time64.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "edgetpu-device-group.h"
#include "edgetpu-dmabuf.h"
#include "edgetpu-dram.h"
#include "edgetpu-internal.h"
#include "edgetpu-mapping.h"
#include "edgetpu-mmu.h"
#include "edgetpu.h"

/*
 * Records objects for mapping a dma-buf to an edgetpu_dev.
 */
struct dmabuf_map_entry {
	struct dma_buf_attachment *attachment;
	/* SG table returned by dma_buf_map_attachment() */
	struct sg_table *sgt;
	/*
	 * The SG table that shrunk and condensed from @sgt with region [0, size), where @size is
	 * the size field in edgetpu_dmabuf_map which owns this entry.
	 */
	struct sg_table shrunk_sgt;
};

/*
 * Records the mapping and other fields needed for mapping a dma-buf to a device
 * group.
 */
struct edgetpu_dmabuf_map {
	struct edgetpu_mapping map;
	u64 size; /* size of this mapping in bytes */
	u32 mmu_flags;
	/*
	 * The array length of @dmabufs is
	 *   * 1, for normal dmabuf mappings
	 *   * number of devices in @group, for bulk mappings
	 */
	struct dma_buf **dmabufs;
	/*
	 * The length of array @entries will be
	 *   * 1, for non-mirrored normal mapping requests
	 *   * number of devices in @group, otherwise
	 */
	struct dmabuf_map_entry *entries;
	uint num_entries;
};

/*
 * edgetpu implementation of DMA fence
 *
 * @fence:		the base DMA fence
 * @lock:		spinlock protecting updates to @fence
 * @timeline_name:	name of the timeline associated with the fence
 * @group:		owning device group
 * @etfence_list:	global list of all edgetpu DMA fences
 * @group_list:		list of DMA fences owned by the same group
 *
 * It is likely timelines will become a separate object in the future,
 * but for now there's a unique named timeline associated with each fence.
 */
struct edgetpu_dma_fence {
	struct dma_fence fence;
	spinlock_t lock;
	char timeline_name[EDGETPU_SYNC_TIMELINE_NAME_LEN];
	struct edgetpu_device_group *group;
	struct list_head etfence_list;
	struct list_head group_list;
};

/* List of all edgetpu fence objects for debugging. */
static LIST_HEAD(etfence_list_head);
static DEFINE_SPINLOCK(etfence_list_lock);

static const struct dma_fence_ops edgetpu_dma_fence_ops;

static int etdev_add_translations(struct edgetpu_dev *etdev,
				  tpu_addr_t tpu_addr,
				  struct dmabuf_map_entry *entry,
				  u32 mmu_flags,
				  enum dma_data_direction dir,
				  enum edgetpu_context_id ctx_id)
{
	int prot = mmu_flag_to_iommu_prot(mmu_flags, etdev->dev, dir);
	uint i;
	u64 offset = 0;
	int ret;
	struct sg_table *sgt = &entry->shrunk_sgt;
	struct scatterlist *sg;

	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		ret = edgetpu_mmu_add_translation(etdev, tpu_addr + offset,
						  sg_dma_address(sg),
						  sg_dma_len(sg), prot, ctx_id);
		if (ret)
			goto rollback;
		offset += sg_dma_len(sg);
	}
	edgetpu_mmu_reserve(etdev, tpu_addr, offset);
	return 0;

rollback:
	edgetpu_mmu_remove_translation(etdev, tpu_addr, offset, ctx_id);
	return ret;
}

static void etdev_remove_translations(struct edgetpu_dev *etdev,
				      tpu_addr_t tpu_addr,
				      struct dmabuf_map_entry *entry,
				      enum edgetpu_context_id ctx_id)
{
	uint i;
	u64 offset = 0;
	struct sg_table *sgt = &entry->shrunk_sgt;
	struct scatterlist *sg;

	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		edgetpu_mmu_remove_translation(etdev, tpu_addr + offset,
					       sg_dma_len(sg), ctx_id);
		offset += sg_dma_len(sg);
	}
	edgetpu_mmu_free(etdev, tpu_addr, offset);
}

/*
 * Maps to the first entry in @dmap.
 *
 * If the first entry is not fetched (this could happen in bulk mappings),
 * a TPU VA is still allocated according to @dmap->mmu_flags but not mapped.
 *
 * Caller holds @group->lock.
 */
static int etdev_map_dmabuf(struct edgetpu_dev *etdev,
			    struct edgetpu_dmabuf_map *dmap,
			    tpu_addr_t *tpu_addr_p)
{
	struct edgetpu_device_group *group = dmap->map.priv;
	const enum edgetpu_context_id ctx_id =
		edgetpu_group_context_id_locked(group);
	struct dmabuf_map_entry *entry = &dmap->entries[0];
	tpu_addr_t tpu_addr;

	if (!entry->sgt) {
		/*
		 * This could only happen in bulk mappings, when the dmabuf for
		 * master device is not set (runtime passes EDGETPU_IGNORE_FD).
		 */
		tpu_addr =
			edgetpu_mmu_alloc(etdev, dmap->size, dmap->mmu_flags);
		if (!tpu_addr)
			return -ENOSPC;
	} else {
		tpu_addr =
			edgetpu_mmu_tpu_map_sgt(etdev, &entry->shrunk_sgt, dmap->map.dir,
						ctx_id, dmap->mmu_flags);
		if (!tpu_addr)
			return -ENOSPC;
	}

	*tpu_addr_p = tpu_addr;
	return 0;
}

/*
 * Reverts etdev_map_dmabuf().
 *
 * Caller holds @group->lock.
 */
static void etdev_unmap_dmabuf(struct edgetpu_dev *etdev,
			       struct edgetpu_dmabuf_map *dmap,
			       tpu_addr_t tpu_addr)
{
	struct edgetpu_device_group *group = dmap->map.priv;
	const enum edgetpu_context_id ctx_id =
		edgetpu_group_context_id_locked(group);
	struct dmabuf_map_entry *entry = &dmap->entries[0];

	if (!entry->sgt)
		edgetpu_mmu_free(etdev, tpu_addr, dmap->size);
	else
		edgetpu_mmu_tpu_unmap_sgt(etdev, tpu_addr, &entry->shrunk_sgt,
					  ctx_id);
}

/*
 * Handles mirrored mapping request.
 *
 * Caller holds @group->lock.
 */
static int group_map_dmabuf(struct edgetpu_device_group *group,
			    struct edgetpu_dmabuf_map *dmap,
			    tpu_addr_t *tpu_addr_p)
{
	const enum edgetpu_context_id ctx_id =
		edgetpu_group_context_id_locked(group);
	struct edgetpu_dev *etdev = group->etdev;
	tpu_addr_t tpu_addr;
	uint i;
	int ret;

	ret = etdev_map_dmabuf(etdev, dmap, &tpu_addr);
	if (ret)
		return ret;
	for (i = 1; i < group->n_clients; i++) {
		etdev = edgetpu_device_group_nth_etdev(group, i);
		if (!dmap->entries[i].sgt) {
			edgetpu_mmu_reserve(etdev, tpu_addr, dmap->size);
			continue;
		}
		ret = etdev_add_translations(etdev, tpu_addr, &dmap->entries[i],
					     dmap->mmu_flags, dmap->map.dir, ctx_id);
		if (ret)
			goto err_remove;
	}
	*tpu_addr_p = tpu_addr;
	return 0;

err_remove:
	while (i > 1) {
		i--;
		etdev = edgetpu_device_group_nth_etdev(group, i);
		if (!dmap->entries[i].sgt)
			edgetpu_mmu_free(etdev, tpu_addr, dmap->size);
		else
			etdev_remove_translations(etdev, tpu_addr,
						  &dmap->entries[i], ctx_id);
	}
	etdev_unmap_dmabuf(group->etdev, dmap, tpu_addr);

	return ret;
}

/*
 * Reverts group_map_dmabuf().
 *
 * Caller holds @group->lock.
 */
static void group_unmap_dmabuf(struct edgetpu_device_group *group,
			       struct edgetpu_dmabuf_map *dmap,
			       tpu_addr_t tpu_addr)
{
	const enum edgetpu_context_id ctx_id =
		edgetpu_group_context_id_locked(group);
	struct edgetpu_dev *etdev;
	uint i;

	for (i = 1; i < group->n_clients; i++) {
		etdev = edgetpu_device_group_nth_etdev(group, i);
		etdev_remove_translations(etdev, tpu_addr, &dmap->entries[i],
					  ctx_id);
	}
	etdev_unmap_dmabuf(group->etdev, dmap, tpu_addr);
}

/*
 * Clean resources recorded in @dmap.
 *
 * Caller holds the lock of group (map->priv) and ensures the group is in
 * the finalized state.
 */
static void dmabuf_map_callback_release(struct edgetpu_mapping *map)
{
	struct edgetpu_dmabuf_map *dmap =
		container_of(map, struct edgetpu_dmabuf_map, map);
	struct edgetpu_device_group *group = map->priv;
	const enum dma_data_direction dir = map->dir;
	tpu_addr_t tpu_addr = map->device_address;
	struct edgetpu_dev *etdev;
	uint i;

	if (tpu_addr) {
		if (IS_MIRRORED(map->flags)) {
			group_unmap_dmabuf(group, dmap, tpu_addr);
		} else {
			etdev = edgetpu_device_group_nth_etdev(group,
							       map->die_index);
			etdev_unmap_dmabuf(etdev, dmap, tpu_addr);
		}
	}
	for (i = 0; i < dmap->num_entries; i++) {
		struct dmabuf_map_entry *entry = &dmap->entries[i];

		sg_free_table(&entry->shrunk_sgt);
		if (entry->sgt)
			dma_buf_unmap_attachment(entry->attachment, entry->sgt,
						 dir);
		if (entry->attachment)
			dma_buf_detach(dmap->dmabufs[0], entry->attachment);
	}
	dma_buf_put(dmap->dmabufs[0]);
	edgetpu_device_group_put(group);
	kfree(dmap->dmabufs);
	kfree(dmap->entries);
	kfree(dmap);
}

static void entry_show_dma_addrs(struct dmabuf_map_entry *entry,
				 struct seq_file *s)
{
	struct sg_table *sgt = &entry->shrunk_sgt;

	if (sgt->nents == 1) {
		seq_printf(s, "%pad\n", &sg_dma_address(sgt->sgl));
	} else {
		uint i;
		struct scatterlist *sg = sgt->sgl;

		seq_puts(s, "[");
		for (i = 0; i < sgt->nents; i++) {
			if (i)
				seq_puts(s, ", ");
			seq_printf(s, "%pad", &sg_dma_address(sg));
			sg = sg_next(sg);
		}
		seq_puts(s, "]\n");
	}
}

static void dmabuf_map_callback_show(struct edgetpu_mapping *map,
				     struct seq_file *s)
{
	struct edgetpu_dmabuf_map *dmap =
		container_of(map, struct edgetpu_dmabuf_map, map);

	if (IS_MIRRORED(dmap->map.flags))
		seq_printf(s, "  <%s> mirrored: iova=%pad pages=%llu %s",
			   dmap->dmabufs[0]->exp_name, &map->device_address,
			   DIV_ROUND_UP(dmap->size, PAGE_SIZE),
			   edgetpu_dma_dir_rw_s(map->dir));
	else
		seq_printf(s, "  <%s> die %u: iova=%pad pages=%llu %s",
			   dmap->dmabufs[0]->exp_name, map->die_index, &map->device_address,
			   DIV_ROUND_UP(dmap->size, PAGE_SIZE),
			   edgetpu_dma_dir_rw_s(map->dir));

	edgetpu_device_dram_dmabuf_info_show(dmap->dmabufs[0], s);
	seq_puts(s, " dma=");
	entry_show_dma_addrs(&dmap->entries[0], s);
}

/*
 * Allocates and properly sets fields of an edgetpu_dmabuf_map.
 *
 * Returns the pointer on success, or NULL on failure.
 */
static struct edgetpu_dmabuf_map *
alloc_dmabuf_map(struct edgetpu_device_group *group, edgetpu_map_flag_t flags)
{
	struct edgetpu_dmabuf_map *dmap = kzalloc(sizeof(*dmap), GFP_KERNEL);
	struct edgetpu_mapping *map;
	uint n;

	if (!dmap)
		return NULL;
	if (IS_MIRRORED(flags))
		n = group->n_clients;
	else
		n = 1;
	dmap->entries = kcalloc(n, sizeof(*dmap->entries), GFP_KERNEL);
	if (!dmap->entries)
		goto err_free;
	dmap->dmabufs = kcalloc(1, sizeof(*dmap->dmabufs), GFP_KERNEL);
	if (!dmap->dmabufs)
		goto err_free;
	dmap->num_entries = n;
	dmap->mmu_flags = map_to_mmu_flags(flags) | EDGETPU_MMU_DMABUF;
	map = &dmap->map;
	map->flags = flags;
	map->dir = flags & EDGETPU_MAP_DIR_MASK;
	map->release = dmabuf_map_callback_release;
	map->show = dmabuf_map_callback_show;
	map->priv = edgetpu_device_group_get(group);
	return dmap;

err_free:
	kfree(dmap->dmabufs);
	kfree(dmap->entries);
	kfree(dmap);
	return NULL;
}

/*
 * Clean resources recorded in edgetpu_dmabuf_map, bulk version.
 *
 * Caller holds the lock of group (map->priv) and ensures the group is in
 * the finalized state.
 */
static void dmabuf_bulk_map_callback_release(struct edgetpu_mapping *map)
{
	struct edgetpu_dmabuf_map *bmap =
		container_of(map, struct edgetpu_dmabuf_map, map);
	struct edgetpu_device_group *group = map->priv;
	const enum dma_data_direction dir = map->dir;
	const tpu_addr_t tpu_addr = map->device_address;
	int i;

	if (tpu_addr)
		group_unmap_dmabuf(group, bmap, tpu_addr);
	for (i = 0; i < group->n_clients; i++) {
		struct dmabuf_map_entry *entry = &bmap->entries[i];

		sg_free_table(&entry->shrunk_sgt);
		if (entry->sgt)
			dma_buf_unmap_attachment(entry->attachment, entry->sgt, dir);
		if (entry->attachment)
			dma_buf_detach(bmap->dmabufs[i], entry->attachment);
		if (bmap->dmabufs[i])
			dma_buf_put(bmap->dmabufs[i]);
	}
	edgetpu_device_group_put(group);
	kfree(bmap->dmabufs);
	kfree(bmap->entries);
	kfree(bmap);
}

static void dmabuf_bulk_map_callback_show(struct edgetpu_mapping *map,
					  struct seq_file *s)
{
	struct edgetpu_dmabuf_map *bmap =
		container_of(map, struct edgetpu_dmabuf_map, map);
	int i;

	seq_printf(s, "  bulk: iova=%pad pages=%llu %s\n",
		   &map->device_address, DIV_ROUND_UP(bmap->size, PAGE_SIZE),
		   edgetpu_dma_dir_rw_s(map->dir));
	for (i = 0; i < bmap->num_entries; i++) {
		if (!bmap->dmabufs[i]) {
			seq_printf(s, "   %2d: ignored\n", i);
			continue;
		}
		seq_printf(s, "   %2d: <%s>", i, bmap->dmabufs[i]->exp_name);
		edgetpu_device_dram_dmabuf_info_show(bmap->dmabufs[i], s);
		seq_puts(s, " dma=");
		entry_show_dma_addrs(&bmap->entries[i], s);
	}
}

/*
 * Allocates and properly sets fields of an edgetpu_dmabuf_map, bulk version.
 *
 * Caller holds group->lock and checks @group is finalized.
 *
 * Returns the pointer on success, or NULL on failure.
 */
static struct edgetpu_dmabuf_map *
alloc_dmabuf_bulk_map(struct edgetpu_device_group *group,
		      edgetpu_map_flag_t flags)
{
	struct edgetpu_dmabuf_map *bmap = kzalloc(sizeof(*bmap), GFP_KERNEL);
	struct edgetpu_mapping *map;
	const uint n = group->n_clients;

	if (!bmap)
		return NULL;
	bmap->entries = kcalloc(n, sizeof(*bmap->entries), GFP_KERNEL);
	if (!bmap->entries)
		goto err_free;
	bmap->dmabufs = kcalloc(n, sizeof(*bmap->dmabufs), GFP_KERNEL);
	if (!bmap->dmabufs)
		goto err_free;
	/* mirroredness is ignored in bulk mappings */
	bmap->mmu_flags = map_to_mmu_flags(flags & ~EDGETPU_MAP_NONMIRRORED) |
			  EDGETPU_MMU_DMABUF | EDGETPU_MMU_VDG;
	bmap->num_entries = n;
	map = &bmap->map;
	map->die_index = ALL_DIES;
	map->flags = flags;
	map->dir = flags & EDGETPU_MAP_DIR_MASK;
	map->release = dmabuf_bulk_map_callback_release;
	map->show = dmabuf_bulk_map_callback_show;
	map->priv = edgetpu_device_group_get(group);
	return bmap;

err_free:
	kfree(bmap->dmabufs);
	kfree(bmap->entries);
	kfree(bmap);
	return NULL;
}

/*
 * Duplicates @sgt in region [0, @size) to @out.
 * Only duplicates the "page" parts in @sgt, DMA addresses and lengths are not
 * considered.
 */
static int dup_sgt_in_region(struct sg_table *sgt, u64 size, struct sg_table *out)
{
	uint n = 0;
	u64 cur_offset = 0;
	struct scatterlist *sg, *new_sg;
	int i;
	int ret;

	/* calculate the number of sg covered */
	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		size_t pg_len = sg->length + sg->offset;

		n++;
		if (size <= cur_offset + pg_len)
			break;
		cur_offset += pg_len;
	}
	ret = sg_alloc_table(out, n, GFP_KERNEL);
	if (ret)
		return ret;
	cur_offset = 0;
	new_sg = out->sgl;
	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		size_t pg_len = sg->length + sg->offset;
		struct page *page = sg_page(sg);
		unsigned int len = pg_len;
		u64 remain_size = size - cur_offset;

		if (remain_size < pg_len)
			len -= pg_len - remain_size;
		sg_set_page(new_sg, page, len, 0);
		new_sg = sg_next(new_sg);

		if (size <= cur_offset + pg_len)
			break;
		cur_offset += pg_len;
	}
	return 0;
}

/*
 * Copy the DMA addresses and lengths in region [0, @size) from
 * @sgt to @out.
 *
 * The DMA addresses will be condensed when possible.
 */
static void shrink_sgt_dma_in_region(struct sg_table *sgt, u64 size, struct sg_table *out)
{
	u64 cur_offset = 0;
	struct scatterlist *sg, *prv_sg = NULL, *cur_sg;

	cur_sg = out->sgl;
	out->nents = 0;
	for (sg = sgt->sgl; sg;
	     cur_offset += sg_dma_len(sg), sg = sg_next(sg)) {
		u64 remain_size = size - cur_offset;
		dma_addr_t dma;
		size_t len;

		dma = sg_dma_address(sg);
		len = sg_dma_len(sg);
		if (remain_size < sg_dma_len(sg))
			len -= sg_dma_len(sg) - remain_size;
		if (prv_sg &&
		    sg_dma_address(prv_sg) + sg_dma_len(prv_sg) == dma) {
			/* merge to previous sg */
			sg_dma_len(prv_sg) += len;
		} else {
			sg_dma_address(cur_sg) = dma;
			sg_dma_len(cur_sg) = len;
			prv_sg = cur_sg;
			cur_sg = sg_next(cur_sg);
			out->nents++;
		}
		if (remain_size <= sg_dma_len(sg))
			break;
	}
}

static int entry_set_shrunk_sgt(struct dmabuf_map_entry *entry, u64 size)
{
	int ret;

	ret = dup_sgt_in_region(entry->sgt, size, &entry->shrunk_sgt);
	if (ret)
		return ret;
	shrink_sgt_dma_in_region(entry->sgt, size, &entry->shrunk_sgt);
	return 0;
}

/*
 * Performs dma_buf_attach + dma_buf_map_attachment of @dmabuf to @etdev, and
 * sets @entry per the attaching result.
 *
 * Fields of @entry will be set on success.
 */
static int etdev_attach_dmabuf_to_entry(struct edgetpu_dev *etdev, struct dma_buf *dmabuf,
					struct dmabuf_map_entry *entry, u64 size,
					enum dma_data_direction dir)
{
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	int ret;

	attachment = dma_buf_attach(dmabuf, etdev->dev);
	if (IS_ERR(attachment))
		return PTR_ERR(attachment);
	sgt = dma_buf_map_attachment(attachment, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err_detach;
	}
	entry->attachment = attachment;
	entry->sgt = sgt;
	ret = entry_set_shrunk_sgt(entry, size);
	if (ret)
		goto err_unmap;

	return 0;

err_unmap:
	dma_buf_unmap_attachment(attachment, sgt, dir);
err_detach:
	dma_buf_detach(dmabuf, attachment);
	entry->sgt = NULL;
	entry->attachment = NULL;
	return ret;
}

int edgetpu_map_dmabuf(struct edgetpu_device_group *group,
		       struct edgetpu_map_dmabuf_ioctl *arg)
{
	int ret = -EINVAL;
	struct dma_buf *dmabuf;
	edgetpu_map_flag_t flags = arg->flags;
	u64 size;
	const enum dma_data_direction dir = map_flag_to_host_dma_dir(flags);
	struct edgetpu_dev *etdev;
	struct edgetpu_dmabuf_map *dmap;
	tpu_addr_t tpu_addr;
	uint i;

	if (!valid_dma_direction(dir)) {
		etdev_dbg(group->etdev, "%s: invalid direction %d\n", __func__, dir);
		return -EINVAL;
	}
	dmabuf = dma_buf_get(arg->dmabuf_fd);
	if (IS_ERR(dmabuf)) {
		etdev_dbg(group->etdev, "%s: dma_buf_get returns %ld\n",
			  __func__, PTR_ERR(dmabuf));
		return PTR_ERR(dmabuf);
	}

	dmap = alloc_dmabuf_map(group, flags);
	if (!dmap) {
		ret = -ENOMEM;
		goto err_put_dmabuf;
	}

	get_dma_buf(dmabuf);
	dmap->dmabufs[0] = dmabuf;
	dmap->map.map_size = dmap->size = size = dmabuf->size;
	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_finalized(group)) {
		ret = edgetpu_group_errno(group);
		etdev_dbg(group->etdev,
			  "%s: edgetpu_device_group_is_finalized returns %d\n",
			  __func__, ret);
		goto err_unlock_group;
	}
	if (IS_MIRRORED(flags)) {
		for (i = 0; i < group->n_clients; i++) {
			etdev = edgetpu_device_group_nth_etdev(group, i);
			ret = etdev_attach_dmabuf_to_entry(etdev, dmabuf, &dmap->entries[i], size,
							   dir);
			if (ret) {
				etdev_dbg(group->etdev,
					  "%s: etdev_attach_dmabuf_to_entry returns %d\n",
					  __func__, ret);
				goto err_unlock_group;
			}
		}
		ret = group_map_dmabuf(group, dmap, &tpu_addr);
		if (ret) {
			etdev_dbg(group->etdev,
				  "%s: group_map_dmabuf returns %d\n",
				  __func__, ret);
			goto err_unlock_group;
		}
		dmap->map.die_index = ALL_DIES;
	} else {
		etdev = edgetpu_device_group_nth_etdev(group, arg->die_index);
		if (!etdev) {
			etdev_dbg(group->etdev,
				  "%s: edgetpu_device_group_nth_etdev returns NULL\n",
				  __func__);
			ret = -EINVAL;
			goto err_unlock_group;
		}
		ret = etdev_attach_dmabuf_to_entry(etdev, dmabuf, &dmap->entries[0], size, dir);
		if (ret) {
			etdev_dbg(group->etdev,
				  "%s: etdev_attach_dmabuf_to_entry returns %d\n",
				  __func__, ret);
			goto err_unlock_group;
		}
		ret = etdev_map_dmabuf(etdev, dmap, &tpu_addr);
		if (ret) {
			etdev_dbg(group->etdev,
				  "%s: etdev_map_dmabuf returns %d\n",
				  __func__, ret);
			goto err_unlock_group;
		}
		dmap->map.die_index = arg->die_index;
	}
	dmap->map.device_address = tpu_addr;
	mutex_unlock(&group->lock);
	/* Save address before add to mapping tree, after which another thread can free it. */
	arg->device_address = dmap->map.device_address;
	ret = edgetpu_mapping_add(&group->dmabuf_mappings, &dmap->map);
	if (ret) {
		etdev_dbg(group->etdev, "%s: edgetpu_mapping_add returns %d\n",
			  __func__, ret);
		goto err_release_map;
	}
	dma_buf_put(dmabuf);
	return 0;

err_unlock_group:
	mutex_unlock(&group->lock);
err_release_map:
	/* also releases entries if they are set */
	dmabuf_map_callback_release(&dmap->map);
err_put_dmabuf:
	dma_buf_put(dmabuf);

	return ret;
}

int edgetpu_unmap_dmabuf(struct edgetpu_device_group *group, u32 die_index, tpu_addr_t tpu_addr)
{
	struct edgetpu_mapping_root *mappings = &group->dmabuf_mappings;
	struct edgetpu_mapping *map;

	edgetpu_mapping_lock(mappings);
	map = edgetpu_mapping_find_locked(mappings, die_index, tpu_addr);
	if (!map)
		map = edgetpu_mapping_find_locked(mappings, ALL_DIES, tpu_addr);
	/* the mapping is not found */
	if (!map) {
		edgetpu_mapping_unlock(mappings);
		etdev_err(group->etdev, "unmap group=%u tpu_addr=%pad not found",
			  group->workload_id, &tpu_addr);
		return -EINVAL;
	}
	edgetpu_mapping_unlink(mappings, map);
	edgetpu_mapping_unlock(mappings);
	/* use the callback to handle both normal and bulk requests */
	map->release(map);
	return 0;
}

int edgetpu_map_bulk_dmabuf(struct edgetpu_device_group *group,
			    struct edgetpu_map_bulk_dmabuf_ioctl *arg)
{
	const enum dma_data_direction dir = map_flag_to_host_dma_dir(arg->flags);
	int ret = -EINVAL;
	struct edgetpu_dmabuf_map *bmap;
	struct dma_buf *dmabuf;
	struct edgetpu_dev *etdev;
	tpu_addr_t tpu_addr;
	int i;

	if (!valid_dma_direction(dir) || arg->size == 0)
		return -EINVAL;
	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_finalized(group)) {
		ret = edgetpu_group_errno(group);
		goto err_unlock_group;
	}
	/* checks not all FDs are ignored */
	for (i = 0; i < group->n_clients; i++)
		if (arg->dmabuf_fds[i] != EDGETPU_IGNORE_FD)
			break;
	if (i == group->n_clients)
		goto err_unlock_group;
	bmap = alloc_dmabuf_bulk_map(group, arg->flags);
	if (!bmap) {
		ret = -ENOMEM;
		goto err_unlock_group;
	}
	for (i = 0; i < group->n_clients; i++) {
		if (arg->dmabuf_fds[i] != EDGETPU_IGNORE_FD) {
			dmabuf = dma_buf_get(arg->dmabuf_fds[i]);
			if (IS_ERR(dmabuf)) {
				ret = PTR_ERR(dmabuf);
				goto err_release_bmap;
			}
			if (arg->size > dmabuf->size)
				goto err_release_bmap;
			bmap->dmabufs[i] = dmabuf;
		}
	}
	bmap->size = PAGE_ALIGN(arg->size);
	for (i = 0; i < group->n_clients; i++) {
		if (!bmap->dmabufs[i])
			continue;
		etdev = edgetpu_device_group_nth_etdev(group, i);
		ret = etdev_attach_dmabuf_to_entry(etdev, bmap->dmabufs[i], &bmap->entries[i],
						   bmap->size, dir);
		if (ret)
			goto err_release_bmap;
	}
	ret = group_map_dmabuf(group, bmap, &tpu_addr);
	if (ret)
		goto err_release_bmap;
	mutex_unlock(&group->lock);
	bmap->map.device_address = tpu_addr;
	ret = edgetpu_mapping_add(&group->dmabuf_mappings, &bmap->map);
	if (ret)
		dmabuf_bulk_map_callback_release(&bmap->map);
	else
		arg->device_address = tpu_addr;
	return ret;

err_release_bmap:
	/* also releases entries if they are set */
	dmabuf_bulk_map_callback_release(&bmap->map);
err_unlock_group:
	mutex_unlock(&group->lock);
	return ret;
}

/* reverts edgetpu_map_bulk_dmabuf */
int edgetpu_unmap_bulk_dmabuf(struct edgetpu_device_group *group,
			      tpu_addr_t tpu_addr)
{
	return edgetpu_unmap_dmabuf(group, ALL_DIES, tpu_addr);
}

static struct edgetpu_dma_fence *to_etfence(struct dma_fence *fence)
{
	struct edgetpu_dma_fence *etfence;

	etfence = container_of(fence, struct edgetpu_dma_fence, fence);
	if (fence->ops != &edgetpu_dma_fence_ops)
		return NULL;

	return etfence;
}

static const char *edgetpu_dma_fence_get_driver_name(struct dma_fence *fence)
{
	return "edgetpu";
}

static const char *edgetpu_dma_fence_get_timeline_name(struct dma_fence *fence)
{
	struct edgetpu_dma_fence *etfence = to_etfence(fence);

	return etfence->timeline_name;
}

static void edgetpu_dma_fence_release(struct dma_fence *fence)
{
	struct edgetpu_dma_fence *etfence = to_etfence(fence);
	struct edgetpu_device_group *group;
	unsigned long flags;

	if (!etfence)
		return;

	spin_lock_irqsave(&etfence_list_lock, flags);
	list_del(&etfence->etfence_list);
	spin_unlock_irqrestore(&etfence_list_lock, flags);

	/* group might not yet be set if error at init time. */
	group = etfence->group;
	if (group) {
		mutex_lock(&group->lock);
		list_del(&etfence->group_list);
		mutex_unlock(&group->lock);
		/* Release this fence's reference on the owning group. */
		edgetpu_device_group_put(group);
	}

	kfree(etfence);
}

static bool edgetpu_dma_fence_enable_signaling(struct dma_fence *fence)
{
	return true;
}

static const struct dma_fence_ops edgetpu_dma_fence_ops = {
	.get_driver_name = edgetpu_dma_fence_get_driver_name,
	.get_timeline_name = edgetpu_dma_fence_get_timeline_name,
	.wait = dma_fence_default_wait,
	.enable_signaling = edgetpu_dma_fence_enable_signaling,
	.release = edgetpu_dma_fence_release,
};

#define SEQ_FMT "%llu"

int edgetpu_sync_fence_create(struct edgetpu_device_group *group,
			      struct edgetpu_create_sync_fence_data *datap)
{
	int fd = get_unused_fd_flags(O_CLOEXEC);
	int ret;
	struct edgetpu_dma_fence *etfence;
	struct sync_file *sync_file;
	unsigned long flags;

	if (fd < 0)
		return fd;
	etfence = kzalloc(sizeof(*etfence), GFP_KERNEL);
	if (!etfence) {
		ret = -ENOMEM;
		goto err_put_fd;
	}

	spin_lock_init(&etfence->lock);
	/*
	 * If sync_file_create() fails, fence release is called on dma_fence_put(). A valid
	 * list_head is needed for list_del().
	 */
	INIT_LIST_HEAD(&etfence->etfence_list);
	INIT_LIST_HEAD(&etfence->group_list);
	memcpy(&etfence->timeline_name, &datap->timeline_name,
	       EDGETPU_SYNC_TIMELINE_NAME_LEN - 1);

	dma_fence_init(&etfence->fence, &edgetpu_dma_fence_ops,
		       &etfence->lock, dma_fence_context_alloc(1),
		       datap->seqno);

	sync_file = sync_file_create(&etfence->fence);
	dma_fence_put(&etfence->fence);
	if (!sync_file) {
		ret = -ENOMEM;
		/* doesn't need kfree(etfence) here: dma_fence_put does it for us */
		goto err_put_fd;
	}

	spin_lock_irqsave(&etfence_list_lock, flags);
	list_add_tail(&etfence->etfence_list, &etfence_list_head);
	spin_unlock_irqrestore(&etfence_list_lock, flags);
	etfence->group = edgetpu_device_group_get(group);
	mutex_lock(&group->lock);
	list_add_tail(&etfence->group_list, &group->dma_fence_list);
	mutex_unlock(&group->lock);
	fd_install(fd, sync_file->file);
	datap->fence = fd;
	return 0;

err_put_fd:
	put_unused_fd(fd);
	return ret;
}

static int _edgetpu_sync_fence_signal(struct dma_fence *fence, int errno, bool ignore_signaled)
{
	int ret;

	spin_lock_irq(fence->lock);
	/* don't signal fence twice */
	if (unlikely(test_bit(DMA_FENCE_FLAG_SIGNALED_BIT, &fence->flags))) {
		ret = ignore_signaled ? 0 : -EINVAL;
		goto out_unlock;
	}
	pr_debug("%s: %s-%s%llu-" SEQ_FMT " errno=%d\n", __func__,
		 fence->ops->get_driver_name(fence),
		 fence->ops->get_timeline_name(fence), fence->context,
		 fence->seqno, errno);
	if (errno)
		dma_fence_set_error(fence, errno);
	ret = dma_fence_signal_locked(fence);

out_unlock:
	spin_unlock_irq(fence->lock);
	return ret;
}

int edgetpu_sync_fence_signal(struct edgetpu_signal_sync_fence_data *datap)
{
	struct dma_fence *fence;
	int errno;
	int ret;

	errno = datap->error;
	if (errno > 0)
		errno = -errno;
	if (errno < -MAX_ERRNO)
		return -EINVAL;

	fence = sync_file_get_fence(datap->fence);
	if (!fence)
		return -EINVAL;

	ret = _edgetpu_sync_fence_signal(fence, errno, false);
	dma_fence_put(fence);
	return ret;
}

/* Caller holds group lock. */
void edgetpu_sync_fence_group_shutdown(struct edgetpu_device_group *group)
{
	struct list_head *pos;
	int ret;

	lockdep_assert_held(&group->lock);
	list_for_each(pos, &group->dma_fence_list) {
		struct edgetpu_dma_fence *etfence =
			container_of(pos, struct edgetpu_dma_fence, group_list);
		struct dma_fence *fence = &etfence->fence;

		ret = _edgetpu_sync_fence_signal(fence, -EPIPE, true);
		if (ret)
			etdev_warn(group->etdev, "error %d signaling fence %s-%s %llu-" SEQ_FMT,
				   ret, fence->ops->get_driver_name(fence),
				   fence->ops->get_timeline_name(fence),
				   fence->context, fence->seqno);
	}
}

int edgetpu_sync_fence_status(struct edgetpu_sync_fence_status *datap)
{
	struct dma_fence *fence;

	fence = sync_file_get_fence(datap->fence);
	if (!fence)
		return -EINVAL;

	datap->status = dma_fence_get_status(fence);
	dma_fence_put(fence);
	return 0;
}

static const char *sync_status_str(int status)
{
	if (status < 0)
		return "error";

	if (status > 0)
		return "signaled";

	return "active";
}

int edgetpu_sync_fence_debugfs_show(struct seq_file *s, void *unused)
{
	struct list_head *pos;

	spin_lock_irq(&etfence_list_lock);
	list_for_each(pos, &etfence_list_head) {
		struct edgetpu_dma_fence *etfence =
			container_of(pos, struct edgetpu_dma_fence,
				     etfence_list);
		struct dma_fence *fence = &etfence->fence;

		spin_lock_irq(&etfence->lock);
		seq_printf(s, "%s-%s %llu-" SEQ_FMT " %s",
			   fence->ops->get_driver_name(fence),
			   fence->ops->get_timeline_name(fence),
			   fence->context, fence->seqno,
			   sync_status_str(dma_fence_get_status_locked(fence)));

		if (test_bit(DMA_FENCE_FLAG_TIMESTAMP_BIT, &fence->flags)) {
			struct timespec64 ts64 =
				ktime_to_timespec64(fence->timestamp);

			seq_printf(s, " @%lld.%09ld", (s64)ts64.tv_sec,
				   ts64.tv_nsec);
		}

		if (fence->error)
			seq_printf(s, " err=%d", fence->error);
		seq_printf(s, " group=%u\n", etfence->group->workload_id);
		spin_unlock_irq(&etfence->lock);
	}

	spin_unlock_irq(&etfence_list_lock);
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,16,0)
MODULE_IMPORT_NS(DMA_BUF);
#endif
