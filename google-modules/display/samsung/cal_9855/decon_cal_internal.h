/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS201 DECON CAL INTERNAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DECON_CAL_INTERNAL_H__
#define __SAMSUNG_DECON_CAL_INTERNAL_H__

#ifdef CONFIG_SOC_GS201
void decon_reg_set_rcd_enable_internal(u32 id, bool en);
void decon_reg_update_req_cgc_internal(u32 id);
#else
static void decon_reg_set_rcd_enable_internal(u32 id, bool en) {}
static void decon_reg_update_req_cgc_internal(u32 id) {}
#endif

#endif /* __SAMSUNG_DPP_CAL_INTERNAL_H__ */
