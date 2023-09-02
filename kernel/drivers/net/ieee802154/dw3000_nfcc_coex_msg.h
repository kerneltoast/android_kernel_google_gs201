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
#ifndef __DW3000_NFCC_COEX_MSG_H
#define __DW3000_NFCC_COEX_MSG_H

#include <linux/module.h>
#include "dw3000_nfcc_coex.h"
#include "dw3000.h"

#define TLV_MAX_NB_SLOTS 4
#define DW3000_NFCC_COEX_SESSION_ID_DEFAULT 0

/**
 * enum dw3000_nfcc_coex_tlv_type - TLVs types.
 *
 * @DW3000_NFCC_COEX_TLV_TYPE_UNSPEC: Invalid command.
 * @DW3000_NFCC_COEX_TLV_TYPE_SESSION_TIME0:
 *	Indicate start of UWB session in RCTU time unit.
 * @DW3000_NFCC_COEX_TLV_TYPE_SLOT_LIST:
 *	Indicate the requested next time slots.
 * @DW3000_NFCC_COEX_TLV_TYPE_TLV_UWBCNT_OFFS:
 *	Indicate the UWB clock offset in V1 protocol.
 * @DW3000_NFCC_COEX_TLV_TYPE_ERROR:
 *	Indicate error condition.
 * @DW3000_NFCC_COEX_TLV_TYPE_SLOT_LIST_UUS:
 *	Indicate the UWB clock offset in V2 protocol.
 * @DW3000_NFCC_COEX_TLV_TYPE_STOP_SESSION:
 *	Indicate the stop session in V3 protocol.
 */
enum dw3000_nfcc_coex_tlv_type {
	DW3000_NFCC_COEX_TLV_TYPE_UNSPEC,

	DW3000_NFCC_COEX_TLV_TYPE_SESSION_TIME0,
	DW3000_NFCC_COEX_TLV_TYPE_SLOT_LIST,
	DW3000_NFCC_COEX_TLV_TYPE_TLV_UWBCNT_OFFS,
	DW3000_NFCC_COEX_TLV_TYPE_ERROR,
	DW3000_NFCC_COEX_TLV_TYPE_SLOT_LIST_UUS,
	DW3000_NFCC_COEX_TLV_TYPE_STOP_SESSION
};

/**
 * struct dw3000_nfcc_coex_tlv_slot - TLV slot definition.
 */
struct dw3000_nfcc_coex_tlv_slot {
	/**
	 * @t_start_uus: Start date in 65536*RCTU (close to usec unit).
	 */
	u32 t_start_uus;
	/**
	 * @t_end_uus: End date in 65536*RCTU (close to usec unit).
	 */
	u32 t_end_uus;
};

/**
 * struct dw3000_nfcc_coex_tlv_slot_list - TLV slots.
 */
struct dw3000_nfcc_coex_tlv_slot_list {
	/**
	 * @nb_slots: Number of slots used.
	 */
	u8 nb_slots;
	/**
	 * @slots: array of start/end session time.
	 */
	struct dw3000_nfcc_coex_tlv_slot slots[TLV_MAX_NB_SLOTS];
} __attribute__((packed));

void dw3000_nfcc_coex_header_put(struct dw3000 *dw,
				 struct dw3000_nfcc_coex_buffer *buffer);
int dw3000_nfcc_coex_message_send(struct dw3000 *dw);
int dw3000_nfcc_coex_message_check(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer,
	struct dw3000_nfcc_coex_rx_msg_info *rx_msg_info);

#endif /* __DW3000_NFCC_COEX_MSG_H */
