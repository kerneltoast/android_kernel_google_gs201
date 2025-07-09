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
#ifndef __DW3000_NFCC_COEX_H
#define __DW3000_NFCC_COEX_H

#include <linux/module.h>
#include <net/vendor_cmd.h>

/* Main defines */
#define DW3000_NFCC_COEX_SIGNATURE_STR "QORVO"
#define DW3000_NFCC_COEX_SIGNATURE_LEN 5
#define DW3000_NFCC_COEX_MAX_NB_TLV 12

/* For TLV messages written by AP / read by NFCC,
 * the scratch memory region is SCRATCH_RAM[0-63]. */
#define DW3000_NFCC_COEX_MSG_OUT_OFFSET 0
#define DW3000_NFCC_COEX_MSG_OUT_SIZE 64
/* For TLV messages read by AP / written by NFCC,
 * the scratch memory region is SCRATCH_RAM[64-127]. */
#define DW3000_NFCC_COEX_MSG_IN_OFFSET 64
#define DW3000_NFCC_COEX_MSG_IN_SIZE 63

#define DW3000_NFCC_COEX_MSG_MAX_SIZE DW3000_NFCC_COEX_MSG_OUT_SIZE

/* MSG_HEADER_LEN is also the sizeof of dw3000_nfcc_coex_msg structure. */
#define MSG_HEADER_LEN (DW3000_NFCC_COEX_SIGNATURE_LEN + 3)
#define MSG_LEN(x) (MSG_HEADER_LEN + (x).tlvs_len)

#define DW3000_NFCC_COEX_UUS_PER_SYS_POWER 8 /* To use with right shift. */
#define DW3000_NFCC_COEX_DTU_PER_UUS_POWER 4 /* To use with left shift. */

/**
 * enum dw3000_nfcc_coex_send - Type of message to send.
 *
 * @DW3000_NFCC_COEX_SEND_CLK_SYNC: Clock sync message.
 * @DW3000_NFCC_COEX_SEND_CLK_OFFSET: Clock offset message.
 * @DW3000_NFCC_COEX_SEND_STOP: Stop message.
 */
enum dw3000_nfcc_coex_send {
	DW3000_NFCC_COEX_SEND_CLK_SYNC,
	DW3000_NFCC_COEX_SEND_CLK_OFFSET,
	DW3000_NFCC_COEX_SEND_STOP,
};

/**
 * struct dw3000_nfcc_coex_msg - Message read/write from/to scratch memory.
 */
struct dw3000_nfcc_coex_msg {
	/**
	 * @signature: String signature, example "QORVO".
	 */
	u8 signature[DW3000_NFCC_COEX_SIGNATURE_LEN];
	/**
	 * @ver_id: Version identifier.
	 */
	u8 ver_id;
	/**
	 * @seqnum: Sequence number.
	 */
	u8 seqnum;
	/**
	 * @nb_tlv: Number of TLV object in the body message.
	 */
	u8 nb_tlv;
	/**
	 * @tlvs: Body message. Its addr points to TLVs start.
	 */
	u8 tlvs[];
} __attribute__((packed));

/**
 * struct dw3000_nfcc_coex_buffer - Memory buffer to read/write a NFCC message.
 */
struct dw3000_nfcc_coex_buffer {
	/* Unamed union for structured access or raw access. */
	union {
		/**
		 * @raw: Byte memory.
		 */
		u8 raw[DW3000_NFCC_COEX_MSG_MAX_SIZE];
		/**
		 * @msg: nfcc_coex message.
		 */
		struct dw3000_nfcc_coex_msg msg;
	};
	/**
	 * @tlvs_len: Len of TLVs in bytes.
	 */
	u8 tlvs_len;
} __attribute__((packed));

/**
 * struct dw3000_nfcc_coex_rx_msg_info - Result of message parsed.
 */
struct dw3000_nfcc_coex_rx_msg_info {
	/**
	 * @next_timestamp_dtu: Next NFCC access requested.
	 */
	u32 next_timestamp_dtu;
	/**
	 * @next_duration_dtu: Next NFCC duration access.
	 */
	int next_duration_dtu;
	/**
	 * @next_slot_found: True when first next slot is found.
	 */
	bool next_slot_found;
};

/**
 * struct dw3000_nfcc_coex - NFCC coexistence context.
 */
struct dw3000_nfcc_coex {
	/**
	 * @access_info: Access information to provide to upper layer.
	 */
	struct llhw_vendor_cmd_nfcc_coex_get_access_info access_info;
	/**
	 * @session_time0_dtu: Timestamp used as reference between NFCC and AP.
	 */
	u32 session_time0_dtu;
	/**
	 * @access_start_dtu: start date of nfcc session in DTU.
	 */
	u32 access_start_dtu;
	/**
	 * @prev_offset_sys_time: Previous offset between local and decawave time.
	 */
	u32 prev_offset_sys_time;
	/**
	 * @original_channel: channel number to be restored after NFCC session.
	 */
	u8 original_channel;
	/**
	 * @rx_seq_num: Sequence number of last valid message check.
	 */
	u8 rx_seq_num;
	/**
	 * @tx_seq_num: Sequence message counter increased on outgoing message.
	 */
	u8 tx_seq_num;
	/**
	 * @enabled: True when nfcc coex is handling an access.
	 */
	bool enabled;
	/**
	 * @configured: True when nfcc coex is configured.
	 */
	bool configured;
	/**
	 * @send: Type of message to send.
	 */
	enum dw3000_nfcc_coex_send send;
	/**
	 * @first_rx_message: False after the first valid msg received.
	 */
	bool first_rx_message;
	/**
	 * @watchdog_timer: Watchdog timer to detect spi not bring back.
	 */
	struct timer_list watchdog_timer;
	/**
	 * @version: Protocol version to use.
	 */
	u8 version;
};

#endif /* __DW3000_NFCC_COEX_H */
