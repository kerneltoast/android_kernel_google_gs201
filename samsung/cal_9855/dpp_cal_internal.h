/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS201 DPP CAL INTERNAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DPP_CAL_INTERNAL_H__
#define __SAMSUNG_DPP_CAL_INTERNAL_H__

#ifdef CONFIG_SOC_GS201
void rcd_reg_init(u32 id);
int rcd_reg_deinit(u32 id, bool reset, const unsigned long attr);
void rcd_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr);
u32 cgc_reg_get_irq_and_clear_internal(u32 id);
void cgc_reg_set_cgc_start_internal(u32 id);
void cgc_reg_set_config_internal(u32 id, bool en, dma_addr_t addr);
#else
static inline void rcd_reg_init(u32 id) {return;}
static inline int rcd_reg_deinit(u32 id, bool reset, const unsigned long attr) {return 0;}
static inline void rcd_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr) {return;}
static inline u32 cgc_reg_get_irq_and_clear_internal(u32 id) {return 0;}
static inline void cgc_reg_set_cgc_start_internal(u32 id) {return;}
static inline void cgc_reg_set_config_internal(u32 id, bool en, dma_addr_t addr) {return;}
#endif

#endif /* __SAMSUNG_DPP_CAL_INTERNAL_H__ */
