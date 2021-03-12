// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#include "dit_common.h"
#include "dit_hal.h"

static struct dit_ctrl_t *dc;

static bool dit_check_nat_enabled(void)
{
	unsigned int ring_num;

	for (ring_num = DIT_DST_DESC_RING_1; ring_num < DIT_DST_DESC_RING_MAX; ring_num++) {
		if (dit_check_dst_ready(DIT_DIR_RX, ring_num) == 0)
			return true;
	}

	return false;
}

static void dit_check_clat_enabled_internal(struct io_device *iod, void *args)
{
	bool *enabled = (bool *)args;

	if (*enabled || !dc->ld->is_ps_ch(iod->ch))
		return;

	if (iod->clat_ndev)
		*enabled = true;
}

static bool dit_check_clat_enabled(void)
{
	bool enabled = false;

	if (unlikely(!dc->ld))
		return false;

	iodevs_for_each(dc->ld->msd, dit_check_clat_enabled_internal, &enabled);

	return enabled;
}

static int dit_reg_backup_restore_internal(bool backup, const u16 *offset,
					   const u16 *size, void **buf,
					   const unsigned int arr_len)
{
	unsigned long flags;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < arr_len; i++) {
		if (!buf[i]) {
			buf[i] = kvzalloc(size[i], GFP_KERNEL);
			if (!buf[i]) {
				ret = -ENOMEM;
				goto exit;
			}
		}

		spin_lock_irqsave(&dc->src_lock, flags);
		if (dit_is_kicked_any() || !dc->init_done) {
			ret = -EAGAIN;
			spin_unlock_irqrestore(&dc->src_lock, flags);
			goto exit;
		}

		if (backup)
			BACKUP_REG_VALUE(dc, buf[i], offset[i], size[i]);
		else
			RESTORE_REG_VALUE(dc, buf[i], offset[i], size[i]);
		spin_unlock_irqrestore(&dc->src_lock, flags);
	}

exit:
	/* reset buffer if failed to backup */
	if (unlikely(ret && backup)) {
		for (i = 0; i < arr_len; i++) {
			if (buf[i])
				memset(buf[i], 0, size[i]);
		}
	}

	return ret;
}

static int dit_reg_backup_restore(bool backup)
{
	/* NAT */
	static const u16 nat_offset[] = {
		DIT_REG_NAT_LOCAL_ADDR,
		DIT_REG_NAT_ETHERNET_DST_MAC_ADDR_0,
		DIT_REG_NAT_RX_PORT_TABLE_SLOT,
	};
	static const u16 nat_size[] = {
		(DIT_REG_NAT_LOCAL_ADDR_MAX * DIT_REG_NAT_LOCAL_INTERVAL),
		(DIT_REG_NAT_LOCAL_ADDR_MAX * DIT_REG_ETHERNET_MAC_INTERVAL),
		(DIT_REG_NAT_LOCAL_PORT_MAX * DIT_REG_NAT_LOCAL_INTERVAL),
	};
	static const unsigned int nat_len = ARRAY_SIZE(nat_offset);
	static void *nat_buf[ARRAY_SIZE(nat_offset)];

	/* CLAT */
	static const u16 clat_offset[] = {
		DIT_REG_CLAT_TX_FILTER,
		DIT_REG_CLAT_TX_PLAT_PREFIX_0,
		DIT_REG_CLAT_TX_CLAT_SRC_0,
	};
	static const u16 clat_size[] = {
		(DIT_REG_CLAT_ADDR_MAX * DIT_REG_CLAT_TX_FILTER_INTERVAL),
		(DIT_REG_CLAT_ADDR_MAX * DIT_REG_CLAT_TX_PLAT_PREFIX_INTERVAL),
		(DIT_REG_CLAT_ADDR_MAX * DIT_REG_CLAT_TX_CLAT_SRC_INTERVAL),
	};
	static const unsigned int clat_len = ARRAY_SIZE(clat_offset);
	static void *clat_buf[ARRAY_SIZE(clat_offset)];

	int ret = 0;

	if (unlikely(!dc))
		return -EPERM;

	/* NAT */
	if (dit_check_nat_enabled()) {
		ret = dit_reg_backup_restore_internal(backup, nat_offset,
						      nat_size, nat_buf, nat_len);
		if (ret)
			goto error;
	}

	/* CLAT */
	if (dit_check_clat_enabled()) {
		ret = dit_reg_backup_restore_internal(backup, clat_offset,
						      clat_size, clat_buf, clat_len);
		if (ret)
			goto error;
	}

	return 0;

error:
	mif_err("backup/restore failed is_backup:%d, ret:%d\n", backup, ret);

	return ret;
}

static int dit_do_suspend(void)
{
	int ret;

	ret = dit_reg_backup_restore(true);
	if (ret) {
		mif_err("reg backup failed ret:%d\n", ret);
		return ret;
	}

	ret = dit_init(NULL, DIT_INIT_DEINIT);
	if (ret) {
		mif_err("deinit failed ret:%d\n", ret);
		return ret;
	}

	return 0;
}

static int dit_do_resume(void)
{
	unsigned int dir;
	int ret;

	ret = dit_init(NULL, DIT_INIT_NORMAL);
	if (ret) {
		mif_err("init failed ret:%d\n", ret);
		for (dir = 0; dir < DIT_DIR_MAX; dir++) {
			if (dit_is_busy(dir))
				mif_err("busy (dir:%d)\n", dir);
		}
		return ret;
	}

	ret = dit_reg_backup_restore(false);
	if (ret) {
		mif_err("reg restore failed ret:%d\n", ret);
		return ret;
	}

	return 0;
}

static int dit_set_desc_filter_bypass(enum dit_direction dir, struct dit_src_desc *src_desc,
				      u8 *src, bool *is_upstream_pkt)
{
	struct net_device *upstream_netdev;
	bool bypass = true;
	u8 mask = 0;

	/*
	 * LRO start/end bit can be used for filter bypass
	 * 1/1: filtered
	 * 0/0: filter bypass
	 * dit does not support checksum yet
	 */
	cpif_set_bit(mask, DIT_DESC_C_END);
	cpif_set_bit(mask, DIT_DESC_C_START);

	if (dir != DIT_DIR_RX)
		goto out;

	/*
	 * check ipv6 for clat.
	 * port table does not have entries for tun device or ipv6.
	 * every ipv6 packets from any rmnet can see port table.
	 */
	if ((src[0] & 0xF0) == 0x60) {
		bypass = false;
		goto out;
	}

	/* check upstream netdev */
	upstream_netdev = dit_hal_get_dst_netdev(DIT_DST_DESC_RING_0);
	if (upstream_netdev) {
		struct io_device *iod = link_get_iod_with_channel(dc->ld, src_desc->ch_id);

		if (iod && iod->ndev == upstream_netdev) {
			*is_upstream_pkt = true;
			bypass = false;
			goto out;
		}
	}

out:
#if defined(DIT_DEBUG_LOW)
	if (dc->force_bypass == 1)
		bypass = true;
	else if (dc->force_bypass == 2)
		bypass = false;
#endif

	if (bypass)
		src_desc->control &= ~mask;
	else
		src_desc->control |= mask;

	return 0;
}

static void __dit_set_interrupt(void)
{
	static int irq_pending_bit[] = {
		RX_DST0_INT_PENDING_BIT, RX_DST1_INT_PENDING_BIT,
		RX_DST2_INT_PENDING_BIT, TX_DST0_INT_PENDING_BIT,
		ERR_INT_PENDING_BIT};
	static char const *irq_name[] = {
		"DIT-RxDst0", "DIT-RxDst1",
		"DIT-RxDst2", "DIT-Tx",
		"DIT-Err"};

	dc->irq_pending_bit = irq_pending_bit;
	dc->irq_name = irq_name;
	dc->irq_len = ARRAY_SIZE(irq_pending_bit);
}

int dit_ver_create(struct dit_ctrl_t *dc_ptr)
{
	if (unlikely(!dc_ptr))
		return -EPERM;

	dc = dc_ptr;

	__dit_set_interrupt();

	dc->set_desc_filter_bypass = dit_set_desc_filter_bypass;
	dc->do_suspend = dit_do_suspend;
	dc->do_resume = dit_do_resume;

	return 0;
}

