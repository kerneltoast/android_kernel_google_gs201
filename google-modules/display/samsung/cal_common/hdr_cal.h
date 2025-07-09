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
#if defined(CONFIG_SOC_ZUMA)
void hdr_reg_set_eotf_lut(u32 id, struct hdr_eotf_lut_v2p2 *lut);
void hdr_reg_set_oetf_lut(u32 id, struct hdr_oetf_lut_v2p2 *lut);
void hdr_reg_set_tm(u32 id, struct hdr_tm_data_v2p2 *tm);
void hdr_reg_set_fp16(u32 id, bool fp16_en, bool fp16_cvt_en);
#else
void hdr_reg_set_eotf_lut(u32 id, struct hdr_eotf_lut *lut);
void hdr_reg_set_oetf_lut(u32 id, struct hdr_oetf_lut *lut);
void hdr_reg_set_tm(u32 id, struct hdr_tm_data *tm);
static inline void hdr_reg_set_fp16(u32 id, bool fp16_en, bool fp16_cvt_en) {}
#endif
void hdr_reg_set_gm(u32 id, struct hdr_gm_data *data);
void hdr_reg_print_eotf_lut(u32 id, struct drm_printer *p);
void hdr_reg_print_oetf_lut(u32 id, struct drm_printer *p);
void hdr_reg_print_gm(u32 id, struct drm_printer *p);
void hdr_reg_print_tm(u32 id, struct drm_printer *p);

#endif /* __SAMSUNG_HDR_CAL_H__ */
