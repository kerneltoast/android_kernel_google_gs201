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
	dc->do_suspend = dit_dummy;
	dc->do_resume = dit_dummy;

	return 0;
}

