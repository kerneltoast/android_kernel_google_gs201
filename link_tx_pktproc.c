// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020-2021, Samsung Electronics.
 *
 */

#include <asm/cacheflush.h>
#include <linux/shm_ipc.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"
#if IS_ENABLED(CONFIG_EXYNOS_DIT)
#include "dit.h"
#endif

static int pktproc_send_pkt_to_cp(struct pktproc_queue_ul *q, struct sk_buff *skb)
{
	struct pktproc_q_info_ul *q_info = q->q_info;
	struct pktproc_desc_ul *desc;
	void *target_addr;
	int len = skb->len;
#if IS_ENABLED(CONFIG_EXYNOS_DIT)
	bool use_dit;
#endif
	u32 space;

	q->stat.total_cnt++;
	if (!pktproc_check_ul_q_active(q->ppa_ul, q->q_idx)) {
		mif_err_limited("Queue[%d] not activated\n", q->q_idx);
		q->stat.inactive_cnt++;
		return -EACCES;
	}

	/* avoid race with the done_ptr disordering */
	smp_rmb();

	space = circ_get_space(q->num_desc, q->done_ptr, q_info->rear_ptr);
	if (space < 1) {
		mif_err_limited("NOSPC Queue[%d] num_desc:%d fore:%d done:%d rear:%d\n",
				q->q_idx, q->num_desc, q_info->fore_ptr, q->done_ptr,
				q_info->rear_ptr);
		q->stat.buff_full_cnt++;
		return -ENOSPC;
	}

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
	use_dit = dit_check_dir_use_queue(DIT_DIR_TX, q->q_idx);
	if (!use_dit)
#endif
	{
		target_addr = (void *)(q->q_buff_vbase +
			(q->done_ptr * q->max_packet_size));
		skb_copy_from_linear_data(skb, target_addr, skb->len);
	}

	desc = &q->desc_ul[q->done_ptr];
	desc->sktbuf_point = q->buff_addr_cp + (q->done_ptr * q->max_packet_size);

	desc->data_size = skb->len;
	if (q->ppa_ul->padding_required)
		desc->data_size += CP_PADDING;
	desc->total_pkt_size = desc->data_size;
	desc->last_desc = 0;
	desc->seg_on = 0;
	desc->hw_set = 0;
	desc->lcid = skbpriv(skb)->sipc_ch;

	barrier();

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
	if (use_dit) {
		int ret;
		/* skb may not be valid after dit_enqueue is done */
		ret = dit_enqueue_src_desc_ring_skb(DIT_DIR_TX, skb);
		if (ret < 0) {
			mif_err_limited("Enqueue failed Queue[%d] done:%d ret:%d\n",
					q->q_idx, q->done_ptr, ret);
			q->stat.buff_full_cnt++;
			return ret;
		}
	}
#endif

	q->done_ptr = circ_new_ptr(q->num_desc, q->done_ptr, 1);

	/* ensure the done_ptr ordering */
	smp_mb();

	return len;
}

static int pktproc_set_end(struct pktproc_queue_ul *q, unsigned int desc_index,
		unsigned int prev_offset)
{
	struct pktproc_q_info_ul *q_info = q->q_info;
	struct pktproc_desc_ul *prev_desc;
	unsigned int prev_index;

	if (unlikely(desc_index >= q->num_desc))
		return -EINVAL;

	if (unlikely(circ_empty(q->done_ptr, q_info->rear_ptr)))
		return -EAGAIN;

	if (desc_index == q->q_info->fore_ptr)
		return -ERANGE;

	prev_index = circ_prev_ptr(q->num_desc, desc_index, prev_offset);

	prev_desc = &q->desc_ul[prev_index];
	prev_desc->last_desc = 1;

	q->stat.pass_cnt++;

	return 0;
}

static int pktproc_ul_update_fore_ptr(struct pktproc_queue_ul *q, u32 count)
{
	u32 offset = q->ppa_ul->cp_quota;
	unsigned int last_ptr;
	unsigned int fore_ptr;
	unsigned int i;
	int ret;

	last_ptr = q->q_info->fore_ptr;
	fore_ptr = circ_new_ptr(q->num_desc, last_ptr, count);

	if (q->ppa_ul->end_bit_owner == END_BIT_CP)
		goto set_fore;

	if (count < offset)
		goto set_last;

	for (i = 0; i < count - offset; i += offset) {
		last_ptr = circ_new_ptr(q->num_desc, last_ptr, offset);
		ret = pktproc_set_end(q, last_ptr, 1);
		if (ret) {
			mif_err_limited("set end failed. q_idx:%d, ret:%d\n", q->q_idx, ret);
			goto error;
		}
	}

set_last:
	ret = pktproc_set_end(q, fore_ptr, 1);
	if (ret) {
		mif_err_limited("set end failed. q_idx:%d, ret:%d\n", q->q_idx, ret);
		goto error;
	}

set_fore:
	q->q_info->fore_ptr = fore_ptr;

	/* ensure the fore_ptr ordering */
	smp_mb();

error:
	return 0;
}

/*
 * Debug
 */
static ssize_t region_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor_ul *ppa_ul = &mld->pktproc_ul;
	struct pktproc_info_ul *info_ul =
		(struct pktproc_info_ul *)ppa_ul->info_vbase;

	ssize_t count = 0;
	int i;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "CP base:0x%08llx\n", ppa_ul->cp_base);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "Num of queue:%d\n", ppa_ul->num_queue);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "HW cache coherency:%d\n",
			ppa_ul->use_hw_iocc);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");

	count += scnprintf(&buf[count], PAGE_SIZE - count, "End bit owner:%d\n",
			info_ul->end_bit_owner);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "CP quota:%d\n", info_ul->cp_quota);

	for (i = 0; i < ppa_ul->num_queue; i++) {
		struct pktproc_queue_ul *q = ppa_ul->q[i];

		if (!pktproc_check_ul_q_active(ppa_ul, q->q_idx)) {
			count += scnprintf(&buf[count], PAGE_SIZE - count,
					"Queue %d is not active\n", i);
			continue;
		}

		count += scnprintf(&buf[count], PAGE_SIZE - count, "Queue%d\n", i);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  num_desc:%d(0x%08x)\n",
				q->q_info->num_desc, q->q_info->num_desc);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  cp_desc_pbase:0x%08x\n",
				q->q_info->cp_desc_pbase);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  desc_size:0x%08x\n",
				q->desc_size);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  cp_buff_pbase:0x%08x\n",
				q->q_info->cp_buff_pbase);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  q_buff_size:0x%08x\n",
				q->q_buff_size);
	}

	return count;
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor_ul *ppa_ul = &mld->pktproc_ul;
	ssize_t count = 0;
	int i;

	for (i = 0; i < ppa_ul->num_queue; i++) {
		struct pktproc_queue_ul *q = ppa_ul->q[i];

		if (!pktproc_check_ul_q_active(ppa_ul, q->q_idx)) {
			count += scnprintf(&buf[count], PAGE_SIZE - count,
					"Queue %d is not active\n", i);
			continue;
		}

		count += scnprintf(&buf[count], PAGE_SIZE - count, "Queue%d\n", i);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  num_desc:%d\n",
				q->num_desc);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  fore/rear:%d/%d\n",
				q->q_info->fore_ptr, q->q_info->rear_ptr);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  pass:%lld\n",
				q->stat.pass_cnt);
		count += scnprintf(&buf[count], PAGE_SIZE - count,
				"  fail: buff_full:%lld inactive:%lld\n",
				q->stat.buff_full_cnt, q->stat.inactive_cnt);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  total:%lld\n",
				q->stat.total_cnt);
	}

	return count;
}

static DEVICE_ATTR_RO(region);
static DEVICE_ATTR_RO(status);

static struct attribute *pktproc_ul_attrs[] = {
	&dev_attr_region.attr,
	&dev_attr_status.attr,
	NULL,
};

static const struct attribute_group pktproc_ul_group = {
	.attrs = pktproc_ul_attrs,
	.name = "pktproc_ul",
};

/*
 * Initialize PktProc
 */
int pktproc_init_ul(struct pktproc_adaptor_ul *ppa_ul)
{
	int i;
	struct mem_link_device *mld;
	struct pktproc_info_ul *info;

	mld = container_of(ppa_ul, struct mem_link_device, pktproc_ul);

	if (!ppa_ul) {
		mif_err("ppa_ul is null\n");
		return -EPERM;
	}

	info = (struct pktproc_info_ul *)ppa_ul->info_vbase;

	if (info->end_bit_owner == END_BIT_AP && info->cp_quota <= 0) {
		mif_err("invalid cp quota: %d\n", info->cp_quota);
		return -EINVAL;
	}

	ppa_ul->cp_quota = info->cp_quota;
	ppa_ul->end_bit_owner = info->end_bit_owner;
	mif_info("CP quota set to %d\n", ppa_ul->cp_quota);

	if (!pktproc_check_support_ul(ppa_ul))
		return 0;


	for (i = 0; i < ppa_ul->num_queue; i++) {
		struct pktproc_queue_ul *q = ppa_ul->q[i];

		mif_info("PKTPROC UL Q%d\n", i);

		*q->fore_ptr = 0; /* sets q_info->fore_ptr to 0 */
		q->done_ptr = 0;
		*q->rear_ptr = 0; /* sets q_info->rear_ptr to 0 */

		if (mld->pktproc_use_36bit_addr) {
			q->q_info->cp_desc_pbase = q->cp_desc_pbase >> 4;
			q->q_info->cp_buff_pbase = q->cp_buff_pbase >> 4;
		} else {
			q->q_info->cp_desc_pbase = q->cp_desc_pbase;
			q->q_info->cp_buff_pbase = q->cp_buff_pbase;
		}

		q->q_info->num_desc = q->num_desc;

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
		if (dit_check_dir_use_queue(DIT_DIR_TX, q->q_idx))
			dit_reset_dst_wp_rp(DIT_DIR_TX);
#endif

		memset(&q->stat, 0, sizeof(struct pktproc_statistics_ul));

		atomic_set(&q->active, 1);
		atomic_set(&q->busy, 0);
		mif_info("num_desc:0x%08x cp_desc_pbase:0x%08llx cp_buff_pbase:0x%08llx\n",
			q->num_desc, q->cp_desc_pbase, q->cp_buff_pbase);
		mif_info("fore:%d rear:%d\n",
			q->q_info->fore_ptr, q->q_info->rear_ptr);
	}

	return 0;
}

/*
 * Create PktProc
 */
static int pktproc_get_info_ul(struct pktproc_adaptor_ul *ppa_ul,
		struct device_node *np)
{
	mif_dt_read_u64(np, "pktproc_cp_base", ppa_ul->cp_base);
	mif_dt_read_u32(np, "pktproc_ul_num_queue", ppa_ul->num_queue);
	if (ppa_ul->num_queue == 1 && !IS_ENABLED(CONFIG_CP_PKTPROC_UL_SINGLE_QUEUE)) {
		mif_err("Need to enable UL single queue config for single UL queue\n");
		panic("Need to enable UL single queue config for single UL queue.\n");
		return -EINVAL;
	}
	if (ppa_ul->num_queue > PKTPROC_UL_QUEUE_MAX)
		ppa_ul->num_queue = PKTPROC_UL_QUEUE_MAX;
	mif_dt_read_u32(np, "pktproc_ul_max_packet_size", ppa_ul->default_max_packet_size);
	mif_dt_read_u32_noerr(np, "pktproc_ul_hiprio_ack_only", ppa_ul->hiprio_ack_only);
	mif_dt_read_u32(np, "pktproc_ul_use_hw_iocc", ppa_ul->use_hw_iocc);
	mif_dt_read_u32(np, "pktproc_ul_info_rgn_cached", ppa_ul->info_rgn_cached);
	mif_dt_read_u32(np, "pktproc_ul_desc_rgn_cached", ppa_ul->desc_rgn_cached);
	mif_dt_read_u32(np, "pktproc_ul_buff_rgn_cached", ppa_ul->buff_rgn_cached);
	mif_dt_read_u32(np, "pktproc_ul_padding_required",
			ppa_ul->padding_required);
	mif_info("cp_base:0x%08llx num_queue:%d max_packet_size:%d hiprio_ack_only:%d iocc:%d\n",
		 ppa_ul->cp_base, ppa_ul->num_queue, ppa_ul->default_max_packet_size,
		 ppa_ul->hiprio_ack_only, ppa_ul->use_hw_iocc);
	mif_info("cached: %d/%d/%d padding_required:%d\n", ppa_ul->info_rgn_cached,
			ppa_ul->desc_rgn_cached, ppa_ul->buff_rgn_cached,
			ppa_ul->padding_required);

	mif_dt_read_u32(np, "pktproc_ul_info_rgn_offset",
			ppa_ul->info_rgn_offset);
	mif_dt_read_u32(np, "pktproc_ul_info_rgn_size",
			ppa_ul->info_rgn_size);
	mif_dt_read_u32(np, "pktproc_ul_desc_rgn_offset",
			ppa_ul->desc_rgn_offset);
	mif_dt_read_u32(np, "pktproc_ul_desc_rgn_size",
			ppa_ul->desc_rgn_size);
	mif_dt_read_u32(np, "pktproc_ul_buff_rgn_offset",
			ppa_ul->buff_rgn_offset);
	mif_dt_read_u32(np, "pktproc_ul_buff_rgn_size",
			ppa_ul->buff_rgn_size);
	mif_info("info_rgn 0x%08lx 0x%08lx desc_rgn 0x%08lx 0x%08lx buff_rgn 0x%08lx 0x%08lx\n",
		ppa_ul->info_rgn_offset, ppa_ul->info_rgn_size,	ppa_ul->desc_rgn_offset,
		ppa_ul->desc_rgn_size, ppa_ul->buff_rgn_offset, ppa_ul->buff_rgn_size);

	return 0;
}

int pktproc_create_ul(struct platform_device *pdev, struct mem_link_device *mld,
		unsigned long memaddr, u32 memsize)
{
	struct device_node *np = pdev->dev.of_node;
	struct pktproc_adaptor_ul *ppa_ul = &mld->pktproc_ul;
	struct pktproc_info_ul *ul_info;
	u32 buff_size, buff_size_by_q;
	u32 last_q_desc_offset;
	int i, ret;
#if IS_ENABLED(CONFIG_EXYNOS_CPIF_IOMMU)
	u64 temp;
#endif

	if (!np) {
		mif_err("of_node is null\n");
		return -EINVAL;
	}
	if (!ppa_ul) {
		mif_err("ppa_ul is null\n");
		return -EINVAL;
	}

	mif_dt_read_u32_noerr(np, "pktproc_ul_support", ppa_ul->support);
	if (!ppa_ul->support) {
		mif_info("pktproc_support_ul is 0. Just return\n");
		return 0;
	}

	/* Get info */
	ret = pktproc_get_info_ul(ppa_ul, np);
	if (ret != 0) {
		mif_err("pktproc_get_info_ul() error %d\n", ret);
		return ret;
	}

#if !IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOCC)
	if (!ppa_ul->use_hw_iocc && ppa_ul->info_rgn_cached) {
		mif_err("cannot support sw iocc based caching on info region\n");
		return -EINVAL;
	}

	if (!ppa_ul->use_hw_iocc && ppa_ul->desc_rgn_cached) {
		mif_err("cannot support sw iocc based caching on desc region\n");
		return -EINVAL;
	}

	if (!ppa_ul->use_hw_iocc && ppa_ul->buff_rgn_cached) {
		mif_err("cannot support sw iocc based caching on buff region\n");
		return -EINVAL;
	}
#endif

	/* Get base addr */
	mif_info("memaddr:0x%lx memsize:0x%08x\n", memaddr, memsize);

	if (ppa_ul->info_rgn_cached) {
		ppa_ul->info_vbase = phys_to_virt(memaddr + ppa_ul->info_rgn_offset);
	} else {
		ppa_ul->info_vbase = cp_shmem_get_nc_region(memaddr +
				ppa_ul->info_rgn_offset, ppa_ul->info_rgn_size);
		if (!ppa_ul->info_vbase) {
			mif_err("ppa->info_vbase error\n");
			ret = -ENOMEM;
			goto create_error;
		}
	}

#if IS_ENABLED(CONFIG_EXYNOS_CPIF_IOMMU)
	ppa_ul->desc_map = cpif_vmap_create(ppa_ul->cp_base + ppa_ul->desc_rgn_offset,
			ppa_ul->desc_rgn_size, ppa_ul->desc_rgn_size);
	if (unlikely(!ppa_ul->desc_map)) {
		mif_err("failed to create desc map for pktproc ul\n");
		ret = -ENOMEM;
		goto create_error;
	}

	ppa_ul->buff_map = cpif_vmap_create(ppa_ul->cp_base + ppa_ul->buff_rgn_offset,
			ppa_ul->buff_rgn_size, ppa_ul->buff_rgn_size);
	if (unlikely(!ppa_ul->buff_map)) {
		mif_err("failed to create buff map for pktproc ul\n");
		cpif_vmap_free(ppa_ul->desc_map);
		ret = -ENOMEM;
		goto create_error;
	}

	temp = cpif_vmap_map_area(ppa_ul->desc_map, 0, 0, memaddr + ppa_ul->desc_rgn_offset);
	if (temp != ppa_ul->cp_base + ppa_ul->desc_rgn_offset) {
		cpif_vmap_free(ppa_ul->desc_map);
		cpif_vmap_free(ppa_ul->buff_map);
		ret = -EINVAL;
		goto create_error;
	}

	temp = cpif_vmap_map_area(ppa_ul->buff_map, 0, 0, memaddr + ppa_ul->buff_rgn_offset);
	if (temp != ppa_ul->cp_base + ppa_ul->buff_rgn_offset) {
		cpif_vmap_free(ppa_ul->desc_map);
		cpif_vmap_free(ppa_ul->buff_map);
		ret = -EINVAL;
		goto create_error;
	}
#endif
	if (ppa_ul->desc_rgn_cached) {
		ppa_ul->desc_vbase = phys_to_virt(memaddr + ppa_ul->desc_rgn_offset);
	} else {
		ppa_ul->desc_vbase = cp_shmem_get_nc_region(memaddr +
				ppa_ul->desc_rgn_offset, ppa_ul->desc_rgn_size);
		if (!ppa_ul->desc_vbase) {
			mif_err("ppa->desc_vbase error\n");
			ret = -ENOMEM;
			goto create_error;
		}
	}

	memset(ppa_ul->info_vbase, 0, ppa_ul->info_rgn_size);
	memset(ppa_ul->desc_vbase, 0, ppa_ul->desc_rgn_size);

	mif_info("info + desc size:0x%08lx\n",
			ppa_ul->info_rgn_size + ppa_ul->desc_rgn_size);

	buff_size = ppa_ul->buff_rgn_size;
	buff_size_by_q = buff_size / ppa_ul->num_queue;
	if (ppa_ul->buff_rgn_cached) {
		ppa_ul->buff_vbase = phys_to_virt(memaddr + ppa_ul->buff_rgn_offset);
	} else {
		ppa_ul->buff_vbase = cp_shmem_get_nc_region(memaddr +
				ppa_ul->buff_rgn_offset, buff_size);
		if (!ppa_ul->buff_vbase) {
			mif_err("ppa->buff_vbase error\n");
			ret = -ENOMEM;
			goto create_error;
		}
	}

	mif_info("Total buffer size:0x%08x Queue:%d Size by queue:0x%08x\n",
					buff_size, ppa_ul->num_queue,
					buff_size_by_q);

	ul_info = (struct pktproc_info_ul *)ppa_ul->info_vbase;
	ul_info->num_queues = ppa_ul->num_queue;

	/* Create queue */
	last_q_desc_offset = 0;
	for (i = 0; i < ppa_ul->num_queue; i++) {
		struct pktproc_queue_ul *q;

		mif_info("Queue %d\n", i);

		ppa_ul->q[i] = kzalloc(sizeof(*ppa_ul->q[i]), GFP_ATOMIC);
		if (ppa_ul->q[i] == NULL) {
			ret = -ENOMEM;
			goto create_error;
		}
		q = ppa_ul->q[i];

		atomic_set(&q->active, 0);

		/* Info region */
		q->ul_info = ul_info;
		q->q_info = &q->ul_info->q_info[i];

		q->q_buff_vbase = ppa_ul->buff_vbase + (i * buff_size_by_q);
		q->cp_buff_pbase = ppa_ul->cp_base +
			ppa_ul->buff_rgn_offset + (i * buff_size_by_q);

		if (mld->pktproc_use_36bit_addr)
			q->q_info->cp_buff_pbase = q->cp_buff_pbase >> 4;
		else
			q->q_info->cp_buff_pbase = q->cp_buff_pbase;

		if (ppa_ul->num_queue > 1 && i == PKTPROC_UL_HIPRIO && ppa_ul->hiprio_ack_only) {
			struct link_device *ld = &mld->link_dev;

			ld->hiprio_ack_only = true;
			q->max_packet_size = HIPRIO_MAX_PACKET_SIZE;
		} else {
			q->max_packet_size = ppa_ul->default_max_packet_size;
		}

		q->q_buff_size = buff_size_by_q;
		q->num_desc = buff_size_by_q / q->max_packet_size;
		q->q_info->num_desc = q->num_desc;
		q->desc_ul = ppa_ul->desc_vbase + last_q_desc_offset;
		q->cp_desc_pbase = ppa_ul->cp_base +
			ppa_ul->desc_rgn_offset + last_q_desc_offset;
		if (mld->pktproc_use_36bit_addr)
			q->q_info->cp_desc_pbase = q->cp_desc_pbase >> 4;
		else
			q->q_info->cp_desc_pbase = q->cp_desc_pbase;
		q->desc_size = sizeof(struct pktproc_desc_ul) * q->num_desc;
		q->buff_addr_cp = ppa_ul->cp_base + ppa_ul->buff_rgn_offset +
			(i * buff_size_by_q);
		q->send_packet = pktproc_send_pkt_to_cp;
		q->update_fore_ptr = pktproc_ul_update_fore_ptr;

		if ((last_q_desc_offset + q->desc_size) > ppa_ul->desc_rgn_size) {
			mif_err("Descriptor overflow. 0x%08x + 0x%08x > 0x%08lx\n",
				last_q_desc_offset, q->desc_size, ppa_ul->desc_rgn_size);
			goto create_error;
		}

		spin_lock_init(&q->lock);

		q->q_idx = i;
		q->mld = mld;
		q->ppa_ul = ppa_ul;

		q->q_info->fore_ptr = 0;
		q->q_info->rear_ptr = 0;

		q->fore_ptr = &q->q_info->fore_ptr;
		q->rear_ptr = &q->q_info->rear_ptr;
		q->done_ptr = *q->fore_ptr;

		last_q_desc_offset += q->desc_size;

		mif_info("num_desc:%d desc_offset:0x%08llx desc_size:0x%08x max_packet_size: %d\n",
			q->num_desc, q->cp_desc_pbase, q->desc_size, q->max_packet_size);
		mif_info("buff_offset:0x%08llx buff_size:0x%08x\n",
			q->cp_buff_pbase, q->q_buff_size);

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
		if (ppa_ul->num_queue == 1 || q->q_idx == PKTPROC_UL_NORM) {
			ret = dit_set_pktproc_queue_num(DIT_DIR_TX, q->q_idx);
			if (ret)
				mif_err("dit_set_pktproc_queue_num() error:%d\n", ret);

			ret = dit_set_buf_size(DIT_DIR_TX, q->max_packet_size);
			if (ret)
				mif_err("dit_set_buf_size() error:%d\n", ret);

			ret = dit_set_pktproc_base(DIT_DIR_TX, memaddr + ppa_ul->buff_rgn_offset +
						   (q->q_idx * buff_size_by_q));
			if (ret)
				mif_err("dit_set_pktproc_base() error:%d\n", ret);

			ret = dit_set_desc_ring_len(DIT_DIR_TX, q->num_desc);
			if (ret)
				mif_err("dit_set_desc_ring_len() error:%d\n", ret);
		}
#endif
	}

	/* Debug */
	ret = sysfs_create_group(&pdev->dev.kobj, &pktproc_ul_group);
	if (ret != 0) {
		mif_err("sysfs_create_group() error %d\n", ret);
		goto create_error;
	}

	return 0;

create_error:
	for (i = 0; i < ppa_ul->num_queue; i++)
		kfree(ppa_ul->q[i]);

	if (!ppa_ul->info_rgn_cached && ppa_ul->info_vbase)
		vunmap(ppa_ul->info_vbase);
	if (!ppa_ul->desc_rgn_cached && ppa_ul->desc_vbase)
		vunmap(ppa_ul->desc_vbase);
	if (!ppa_ul->buff_rgn_cached && ppa_ul->buff_vbase)
		vunmap(ppa_ul->buff_vbase);

	return ret;
}
