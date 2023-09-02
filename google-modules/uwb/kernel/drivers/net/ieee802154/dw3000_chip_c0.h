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
#ifndef __DW3000_CHIP_C0_H
#define __DW3000_CHIP_C0_H

/* Register PLL_COARSE_CODE */
#define DW3000_PLL_COARSE_CODE_ID 0x90004
#define DW3000_PLL_COARSE_CODE_LEN (4U)
#define DW3000_PLL_COARSE_CODE_MASK 0xFFFFFFFFUL
#define DW3000_PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_OFFSET (8U)
#define DW3000_PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_LEN (14U)
#define DW3000_PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_MASK 0x3fff00UL
#define DW3000_PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET (0U)
#define DW3000_PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_LEN (5U)
#define DW3000_PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK 0x1fU

/* Time to wait before reading the calibration status register
 * when a calibration from scratch is executed */
#define DW3000_C0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US (400)

/* RSSI constants */
#define DW3000_RSSI_OFFSET_PRF64_STS 121
#define DW3000_RSSI_OFFSET_PRF64_IPATOV 122
#define DW3000_RSSI_OFFSET_PRF16 114
#define DW3000_RSSI_CONSTANT \
	63 /* 3 * log2(2^21) because log2 used instead of log10 */

#endif /* __DW3000_CHIP_C0_H */
