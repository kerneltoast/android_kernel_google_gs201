/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS101 HDR CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_HDR_CAL_H__
#define __SAMSUNG_HDR_CAL_H__

#include <drm/samsung_drm.h>

void hdr_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name, u32 id);
void hdr_reg_set_hdr(u32 id, bool en);
void hdr_reg_set_eotf_lut(u32 id, struct hdr_eotf_lut *lut);
void hdr_reg_set_oetf_lut(u32 id, struct hdr_oetf_lut *lut);
void hdr_reg_set_gm(u32 id, struct hdr_gm_data *data);
void hdr_reg_set_tm(u32 id, struct hdr_tm_data *tm);
void hdr_reg_print_eotf_lut(u32 id, struct drm_printer *p);
void hdr_reg_print_oetf_lut(u32 id, struct drm_printer *p);
void hdr_reg_print_gm(u32 id, struct drm_printer *p);
void hdr_reg_print_tm(u32 id, struct drm_printer *p);
#endif /* __SAMSUNG_HDR_CAL_H__ */
