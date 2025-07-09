// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/shm_ipc.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"

static int save_log_dump(struct io_device *iod, struct link_device *ld, u8 __iomem *base,
		size_t size)
{
	struct sk_buff *skb = NULL;
	size_t alloc_size = 0xE00;
	size_t copied = 0;
	int ret = 0;

	if (!base) {
		mif_err("base is null\n");
		return -EINVAL;
	}
	if (!size) {
		mif_err("size is 0\n");
		return -EINVAL;
	}

	while (copied < size) {
		if (size - copied < alloc_size)
			alloc_size =  size - copied;

		skb = alloc_skb(alloc_size, GFP_KERNEL);
		if (!skb) {
			skb_queue_purge(&iod->sk_rx_q);
			mif_err("alloc_skb() error\n");
			return -ENOMEM;
		}

		memcpy(skb_put(skb, alloc_size), base + copied, alloc_size);
		copied += alloc_size;

		skbpriv(skb)->iod = iod;
		skbpriv(skb)->ld = ld;
		skbpriv(skb)->lnk_hdr = false;
		skbpriv(skb)->sipc_ch = iod->ch;
		skbpriv(skb)->napi = NULL;

		ret = iod->recv_skb_single(iod, ld, skb);
		if (unlikely(ret < 0)) {
			struct modem_ctl *mc = ld->mc;

			mif_err_limited("%s: %s<-%s: %s->recv_skb fail (%d)\n",
					ld->name, iod->name, mc->name, iod->name, ret);
			dev_kfree_skb_any(skb);
			return ret;
		}
	}

	mif_info("Complete:%zu bytes\n", copied);

	return 0;
}

int cp_get_log_dump(struct io_device *iod, struct link_device *ld, unsigned long arg)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct modem_data *modem = ld->mdm_data;
	void __user *uarg = (void __user *)arg;
	struct cp_log_dump log_dump;
	u8 __iomem *base = NULL;
	u32 size = 0;
	int ret = 0;
	u32 cp_num;

	ret = copy_from_user(&log_dump, uarg, sizeof(log_dump));
	if (ret) {
		mif_err("copy_from_user() error:%d\n", ret);
		return ret;
	}
	log_dump.name[sizeof(log_dump.name) - 1] = '\0';
	mif_info("%s log name:%s index:%d\n", iod->name, log_dump.name, log_dump.idx);

	cp_num = ld->mdm_data->cp_num;
	switch (log_dump.idx) {
	case LOG_IDX_SHMEM:
		if (modem->legacy_raw_rx_buffer_cached) {
			base = phys_to_virt(cp_shmem_get_base(cp_num, SHMEM_IPC));
			size = cp_shmem_get_size(cp_num, SHMEM_IPC);
		} else {
			base = mld->base;
			size = mld->size;
		}

		break;

	case LOG_IDX_VSS:
		base = mld->vss_base;
		size = cp_shmem_get_size(cp_num, SHMEM_VSS);
		break;

	case LOG_IDX_ACPM:
		base = mld->acpm_base;
		size = mld->acpm_size;
		break;

#if IS_ENABLED(CONFIG_CP_BTL)
	case LOG_IDX_CP_BTL:
		if (!ld->mdm_data->btl.enabled) {
			mif_info("%s CP BTL is disabled\n", iod->name);
			return -EPERM;
		}
		base = ld->mdm_data->btl.mem.v_base;
		size = ld->mdm_data->btl.mem.size;
		break;
#endif

	case LOG_IDX_DATABUF_DL:
		base = phys_to_virt(cp_shmem_get_base(cp_num, SHMEM_PKTPROC));
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)
		size = mld->pktproc.buff_rgn_offset;
#else
		size = cp_shmem_get_size(cp_num, SHMEM_PKTPROC);
#endif
		break;

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	case LOG_IDX_DATABUF_UL:
		base = phys_to_virt(cp_shmem_get_base(cp_num, SHMEM_PKTPROC_UL));
		size = cp_shmem_get_size(cp_num, SHMEM_PKTPROC_UL);
		break;
#endif

	case LOG_IDX_L2B:
		base = phys_to_virt(cp_shmem_get_base(cp_num, SHMEM_L2B));
		size = cp_shmem_get_size(cp_num, SHMEM_L2B);
		break;

	case LOG_IDX_DDM:
		base = phys_to_virt(cp_shmem_get_base(cp_num, SHMEM_DDM));
		size = cp_shmem_get_size(cp_num, SHMEM_DDM);
		break;

	default:
		mif_err("%s: invalid index:%d\n", iod->name, log_dump.idx);
		return -EINVAL;
	}

	if (!base) {
		mif_err("base is null for %s\n", log_dump.name);
		return -EINVAL;
	}
	if (!size) {
		mif_err("size is 0 for %s\n", log_dump.name);
		return -EINVAL;
	}

	log_dump.size = size;
	mif_info("%s log size:%d\n", iod->name, log_dump.size);
	ret = copy_to_user(uarg, &log_dump, sizeof(log_dump));
	if (ret) {
		mif_err("copy_to_user() error:%d\n", ret);
		return ret;
	}

	return save_log_dump(iod, ld, base, size);
}
