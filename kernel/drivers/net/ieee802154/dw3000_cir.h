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
#ifndef __DW3000_CIR_H
#define __DW3000_CIR_H

#include <linux/completion.h>

#include "dw3000_core_reg.h"

/* Default count of CIR records stored in cir_data */
#define DW3000_DEFAULT_CIR_RECORD_COUNT 20

/**
 * struct dw3000_cir_record - Hold a cir data record
 * @real: real part of the cir record. Format 6.18 corresponding to sign:figure
 * @imag: imaginary part of the cir record. Format 6.18 corresponding to sign:figure
 */
struct dw3000_cir_record {
	u8 real[3];
	u8 imag[3];
} __attribute__((__packed__));

/**
 * struct dw3000_cir_data - Allow CIR memory records dumped through debugfs
 * @complete: synchronize producer and consumer
 * @mutex: ensure security of accesses to the instance of this structure
 * @ciaregs: array storing all CIA registers
 * @count: number of records in data member
 * @filter: kind of frame selected and accumulated in CIR
 * @ts: timestamp extracted from CIA registers
 * @utime: rx timestamp for the cir measurement
 * @fp_power1: first path power component f1
 * @fp_power2: first path power component f2
 * @fp_power3: first path power component f3
 * @offset: user defined offset
 * @fp_index: index of first path record in CIR register
 * @pdoa: pdoa raw value
 * @acc: number of symbols accumulated in CIR
 * @type: CIR type field
 * @dummy: store the dummy byte firstly received at each CIR memory reading
 * @data: table storing 'count' records read from CIR data memory
 */
struct dw3000_cir_data {
	struct completion complete;
	struct mutex mutex;
	__le32 ciaregs[DW3000_DB_DIAG_SET_LEN >> 2];
	unsigned int count;
	u32 filter;
	u64 ts;
	u64 utime;
	u32 fp_power1;
	u32 fp_power2;
	u32 fp_power3;
	s32 offset;
	u16 fp_index;
	u16 pdoa;
	u16 acc;
	u8 type;
	u8 dummy;
	struct dw3000_cir_record data[1];
};

#endif
