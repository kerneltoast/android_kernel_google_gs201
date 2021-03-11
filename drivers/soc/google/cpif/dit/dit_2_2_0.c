// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#include "dit_common.h"

static struct dit_ctrl_t *dc;

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

	if (iod->ndev == netdev)
		dit_set_reg_nat_interface(iod->ch);
}

static int dit_set_reg_upstream(struct net_device *netdev)
{
	if (!netdev) {
		dit_set_reg_nat_interface(-1);
	} else {
		if (unlikely(!dc->ld))
			return -EINVAL;

		iodevs_for_each(dc->ld->msd, dit_set_reg_upstream_internal, netdev);
	}

	return 0;
}

static int dit_do_init_hw(void)
{
	WRITE_REG_VALUE(dc, BIT(RX_TTLDEC_EN_BIT), DIT_REG_NAT_TTLDEC_EN);
	dit_set_reg_upstream(NULL);

	return 0;
}

static void __dit_set_interrupt(void)
{
	static int irq_pending_bit[] = {
		RX_DST00_INT_PENDING_BIT, RX_DST1_INT_PENDING_BIT,
		RX_DST2_INT_PENDING_BIT, TX_DST0_INT_PENDING_BIT};
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
	dc->do_init_hw = dit_do_init_hw;
	dc->do_suspend = dit_dummy;
	dc->do_resume = dit_dummy;

	return 0;
}

