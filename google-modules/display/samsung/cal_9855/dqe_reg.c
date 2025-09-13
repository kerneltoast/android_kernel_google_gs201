// SPDX-License-Identifier: GPL-2.0-only
/*
 * cal_9855/dqe_regs.c
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register access functions for Samsung Display Quality Enhancer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <dqe_cal.h>

#include "regs-dqe.h"
#include "../cal_9845/regs-dqe.h"

void dqe_reg_set_rcd_en_internal(u32 dqe_id, bool en)
{
	dqe_write(dqe_id, DQE_RCD, DQE_RCD_EN(en ? 1 : 0));
}

void dqe_reg_set_cgc_en_internal(u32 dqe_id, bool en)
{
	cgc_write_mask(dqe_id, DQE_CGC_CON, CGC_EN(en), CGC_EN_MASK);
}

void dqe_reg_set_cgc_coef_dma_req_internal(u32 dqe_id)
{
	cgc_write_mask(dqe_id, DQE_CGC_CON, CGC_COEF_DMA_REQ,
			CGC_COEF_DMA_REQ_MASK);
}

int dqe_reg_wait_cgc_dma_done_internal(u32 id, unsigned long timeout_us)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(dqe_regs_desc(id)->regs +
			cgc_offset(regs_dqe[id].version) + DQE_CGC_CON, val,
			!(val & CGC_COEF_DMA_REQ), 2, timeout_us);
	if (ret) {
		cal_log_err(id, "timeout of CGC COEF DMA request (0x%x)\n",
				!(val & CGC_COEF_DMA_REQ));
		return ret;
	}

	return 0;
}

void dqe_reg_set_histogram_pos_internal(u32 dqe_id, enum exynos_histogram_id hist_id,
					enum histogram_prog_pos pos)
{
	if (hist_id >= HISTOGRAM_MAX) {
		pr_warn("%s: invalid hist_id(%d)\n", __func__, hist_id);
		return;
	}

	pr_debug("%s: pos(%d)\n", __func__, pos);
	hist_write_mask(dqe_id, DQE_HIST, HIST_POS_SEL(pos), HIST_POS_SEL_MASK);
}
