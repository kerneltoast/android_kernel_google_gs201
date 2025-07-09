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

	return 0;
}

