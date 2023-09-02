/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */

#ifndef __DW3000_NFCC_COEX_CORE_H
#define __DW3000_NFCC_COEX_CORE_H

#include <linux/module.h>
#include "dw3000_nfcc_coex.h"
#include "dw3000.h"

extern unsigned dw3000_nfcc_coex_margin_dtu;

int dw3000_nfcc_coex_cancel_watchdog(struct dw3000 *dw);
int dw3000_nfcc_coex_spi1_avail(struct dw3000 *dw);
int dw3000_nfcc_coex_idle_timeout(struct dw3000 *dw);
void dw3000_nfcc_coex_init(struct dw3000 *dw);
int dw3000_nfcc_coex_enable(struct dw3000 *dw, u8 channel);
int dw3000_nfcc_coex_disable(struct dw3000 *dw);
int dw3000_nfcc_coex_configure(struct dw3000 *dw);

#endif /*  __DW3000_NFCC_COEX_CORE_H */
