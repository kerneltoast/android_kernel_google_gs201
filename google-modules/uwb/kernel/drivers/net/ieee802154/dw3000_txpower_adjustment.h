/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
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
#ifndef __DW3000_TXPOWER_ADJUSTMENT_H
#define __DW3000_TXPOWER_ADJUSTMENT_H

#include "dw3000_core.h"
#include "dw3000_core_reg.h"

static inline int dw3000_set_tx_power_register(struct dw3000 *dw, u32 value)
{
	return dw3000_reg_write32(dw, DW3000_TX_POWER_ID, 0, value);
}

int dw3000_adjust_tx_power(struct dw3000 *dw, int payload_bytes);

#endif /* __DW3000_TXPOWER_ADJUSTMENT_H */
