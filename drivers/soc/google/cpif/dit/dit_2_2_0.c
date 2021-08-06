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
static int upstream_ch = -1;

static int dit_get_reg_version(u32 *version)
{
	*version = READ_REG_VALUE(dc, DIT_REG_VERSION);

	return 0;
}

static void dit_set_reg_nat_interface(int ch)
{
	unsigned int part = 0, value = 0;
	unsigned int i, offset;
	unsigned long flags;

	if (ch >= 0) {
		part = (ch >> 5) & 0x7;
		value = 1 << (ch & 0x1F);
	}

	spin_lock_irqsave(&dc->src_lock, flags);
	for (i = 0; i < DIT_REG_NAT_INTERFACE_NUM_MAX; i++) {
		offset = i * DIT_REG_NAT_INTERFACE_NUM_INTERVAL;
		dit_enqueue_reg_value_with_ext_lock((i == part ? value : 0),
						    DIT_REG_NAT_INTERFACE_NUM + offset);
	}
	spin_unlock_irqrestore(&dc->src_lock, flags);
}

static void dit_set_reg_upstream_internal(struct io_device *iod, void *args)
{
	struct net_device *netdev = (struct net_device *)args;

	if (iod->ndev == netdev) {
		upstream_ch = (int)iod->ch;
		dit_set_reg_nat_interface(iod->ch);
	}
}

static int dit_set_reg_upstream(struct net_device *netdev)
{
	if (!netdev) {
		dit_set_reg_nat_interface(-1);
		upstream_ch = -1;
	} else {
		if (unlikely(!dc->ld))
			return -EINVAL;

		iodevs_for_each(dc->ld->msd, dit_set_reg_upstream_internal, netdev);
	}

	return 0;
}

static int dit_set_desc_filter_bypass(enum dit_direction dir, struct dit_src_desc *src_desc,
				      u8 *src, bool *is_upstream_pkt)
{
	bool bypass = false;
	u8 mask = 0;

	/*
	 * LRO start/end bit can be used for filter bypass
	 * 1/1: filtered
	 * 0/0: filter bypass
	 * dit does not support checksum yet
	 */
	cpif_set_bit(mask, DIT_DESC_C_END);
	cpif_set_bit(mask, DIT_DESC_C_START);

	if (dir != DIT_DIR_RX) {
		bypass = true;
		goto out;
	}

	/* ToDo: not need if HW supports UDP zero checksum.
	 * check if the packet will be filtered by DIT_REG_NAT_INTERFACE_NUM.
	 */
	if (upstream_ch >= 0 && src_desc->ch_id == upstream_ch)
		*is_upstream_pkt = true;

out:
#if defined(DIT_DEBUG_LOW)
	if (dc->force_bypass == 1)
		bypass = true;
#endif

	if (bypass) {
		src_desc->control &= ~mask;
	} else {
		src_desc->control |= mask;
		src_desc->interface = src_desc->ch_id;
	}

	return 0;
}

static int dit_set_src_desc_tail(enum dit_direction dir, struct dit_desc_info *desc_info,
				 unsigned int tail)
{
	phys_addr_t p_desc;
	u32 offset_lo = 0, offset_hi = 0, offset_en = 0;

	p_desc = desc_info->src_desc_ring_daddr + (sizeof(struct dit_src_desc) * tail);
	if (dir == DIT_DIR_TX) {
		offset_lo = DIT_REG_TX_SRC_A_TAIL_ADDR_0_TEMP;
		offset_hi = DIT_REG_TX_SRC_A_TAIL_ADDR_1_TEMP;
		offset_en = DIT_REG_TX_SRC_A_TAIL_VALID;
	} else {
		offset_lo = DIT_REG_RX_SRC_A_TAIL_ADDR_0_TEMP;
		offset_hi = DIT_REG_RX_SRC_A_TAIL_ADDR_1_TEMP;
		offset_en = DIT_REG_RX_SRC_A_TAIL_VALID;
	}

	WRITE_REG_PADDR_LO(dc, p_desc, offset_lo);
	WRITE_REG_PADDR_HI(dc, p_desc, offset_hi);
	WRITE_REG_VALUE(dc, 0x1, offset_en);

	return 0;
}

static int dit_do_init_desc(enum dit_direction dir)
{
	struct dit_desc_info *desc_info;
	phys_addr_t p_desc;

	/* dst01-dst03 is not used but hw checks the registers */
	u32 tx_offset_lo[] = {
		DIT_REG_TX_RING_START_ADDR_0_DST01, DIT_REG_TX_RING_START_ADDR_0_DST02,
		DIT_REG_TX_RING_START_ADDR_0_DST03,
		DIT_REG_NAT_TX_DESC_ADDR_0_DST01, DIT_REG_NAT_TX_DESC_ADDR_0_DST02,
		DIT_REG_NAT_TX_DESC_ADDR_0_DST03};
	u32 tx_offset_hi[] = {
		DIT_REG_TX_RING_START_ADDR_1_DST01, DIT_REG_TX_RING_START_ADDR_1_DST02,
		DIT_REG_TX_RING_START_ADDR_1_DST03,
		DIT_REG_NAT_TX_DESC_ADDR_1_DST01, DIT_REG_NAT_TX_DESC_ADDR_1_DST02,
		DIT_REG_NAT_TX_DESC_ADDR_1_DST03};
	u32 rx_offset_lo[] = {
		DIT_REG_RX_RING_START_ADDR_0_DST01, DIT_REG_RX_RING_START_ADDR_0_DST02,
		DIT_REG_RX_RING_START_ADDR_0_DST03,
		DIT_REG_NAT_RX_DESC_ADDR_0_DST01, DIT_REG_NAT_RX_DESC_ADDR_0_DST02,
		DIT_REG_NAT_RX_DESC_ADDR_0_DST03};
	u32 rx_offset_hi[] = {
		DIT_REG_RX_RING_START_ADDR_1_DST01, DIT_REG_RX_RING_START_ADDR_1_DST02,
		DIT_REG_RX_RING_START_ADDR_1_DST03,
		DIT_REG_NAT_RX_DESC_ADDR_1_DST01, DIT_REG_NAT_RX_DESC_ADDR_1_DST02,
		DIT_REG_NAT_RX_DESC_ADDR_1_DST03};

	u32 *offset_lo;
	u32 *offset_hi;
	unsigned int offset_len;
	unsigned int i;

	if (dir == DIT_DIR_TX) {
		offset_lo = tx_offset_lo;
		offset_hi = tx_offset_hi;
		offset_len = ARRAY_SIZE(tx_offset_lo);
	} else {
		offset_lo = rx_offset_lo;
		offset_hi = rx_offset_hi;
		offset_len = ARRAY_SIZE(rx_offset_lo);
	}

	desc_info = &dc->desc_info[dir];
	p_desc = desc_info->dst_desc_ring_daddr[DIT_DST_DESC_RING_0];

	for (i = 0; i < offset_len; i++) {
		WRITE_REG_PADDR_LO(dc, p_desc, offset_lo[i]);
		WRITE_REG_PADDR_HI(dc, p_desc, offset_hi[i]);
	}

	return 0;
}

static int dit_do_init_hw(void)
{
	WRITE_REG_VALUE(dc, BIT(RX_TTLDEC_EN_BIT), DIT_REG_NAT_TTLDEC_EN);
	WRITE_REG_VALUE(dc, BIT(TX_DST_DESC_RESET_BIT), DIT_REG_DST_DESC_RESET);
	WRITE_REG_VALUE(dc, BIT(RX_DST_DESC_RESET_BIT), DIT_REG_DST_DESC_RESET);
	dit_set_reg_upstream(NULL);

	return 0;
}

static void __dit_set_interrupt(void)
{
	static int irq_pending_bit[] = {
		RX_DST00_INT_PENDING_BIT, RX_DST1_INT_PENDING_BIT,
		RX_DST2_INT_PENDING_BIT, TX_DST00_INT_PENDING_BIT};
	static char const *irq_name[] = {
		"DIT-RxDst00", "DIT-RxDst1",
		"DIT-RxDst2", "DIT-Tx"};

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

	dc->get_reg_version = dit_get_reg_version;
	dc->set_reg_upstream = dit_set_reg_upstream;
	dc->set_desc_filter_bypass = dit_set_desc_filter_bypass;
	dc->set_src_desc_tail = dit_set_src_desc_tail;
	dc->do_init_desc = dit_do_init_desc;
	dc->do_init_hw = dit_do_init_hw;

	return 0;
}

