/* SPDX-License-Identifier: GPL-2.0-only
 *
 * panel common utility header
 *
 * Copyright (c) 2022 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PANEL_COMMON_H_
#define PANEL_COMMON_H_

u32 panel_cmn_calc_gamma_2_2_luminance(const u32 value, const u32 max_value, const u32 nit);
u32 panel_cmn_calc_linear_luminance(const u32 value, const u32 coef_x_1k, const int offset);

#endif /* PANEL_COMMON_H_ */
