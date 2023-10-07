// SPDX-License-Identifier: GPL-2.0-only
/*
 * cal_9855/decon_reg.c
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register access functions for Samsung EXYNOS DECON driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <decon_cal.h>

#include "regs-decon.h"
#include "decon_cal_internal.h"
#include "../cal_9845/regs-decon.h"

void decon_reg_set_rcd_enable_internal(u32 id, bool en)
{
	u32 val;

	val = en ? ENHANCE_RCD_ON : 0;
	decon_write_mask(id, DATA_PATH_CON_0, val, ENHANCE_RCD_ON);
}

void decon_reg_update_req_cgc_internal(u32 id)
{
	decon_write_mask(id, SHD_REG_UP_REQ, ~0, SHD_REG_UP_REQ_DQE_CGC);
}

