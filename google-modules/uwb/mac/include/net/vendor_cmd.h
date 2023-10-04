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

#ifndef NET_VENDOR_CMD_H
#define NET_VENDOR_CMD_H

#include <linux/types.h>
#include <net/mcps802154.h>

/**
 * enum llhw_vendor_cmd - Vendor command identifiers.
 * @LLHW_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS:
 *     NFCC Coex: Handle access.
 * @LLHW_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION:
 *     NFCC Coex: Get access information.
 * @LLHW_VENDOR_CMD_NFCC_COEX_STOP:
 *     NFCC Coex: Stop.
 * @LLHW_VENDOR_CMD_PCTT_SETUP_HW:
 *     PCTT: Setup hardware access.
 * @LLHW_VENDOR_CMD_PCTT_HANDLE_LOOPBACK:
 *     PCTT: Handle loop-back test.
 * @LLHW_VENDOR_CMD_PCTT_GET_LOOPBACK_INFO:
 *     PCTT: Get loop-back information.
 * @LLHW_VENDOR_CMD_PCTT_GET_FRAME_INFO:
 *     PCTT: Get the last received frame information.
 */
enum llhw_vendor_cmd {
	LLHW_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS,
	LLHW_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION,
	LLHW_VENDOR_CMD_NFCC_COEX_STOP,
	LLHW_VENDOR_CMD_PCTT_SETUP_HW,
	LLHW_VENDOR_CMD_PCTT_HANDLE_LOOPBACK,
	LLHW_VENDOR_CMD_PCTT_GET_LOOPBACK_INFO,
	LLHW_VENDOR_CMD_PCTT_GET_FRAME_INFO,
};

/**
 * struct llhw_vendor_cmd_nfcc_coex_handle_access - NFCC Coex: handle access
 * vendor command.
 */
struct llhw_vendor_cmd_nfcc_coex_handle_access {
	/**
	 * @start: True to start a new session.
	 */
	bool start;
	/**
	 * @timestamp_dtu:
	 *     Access start date. If this is a new session, this also defines
	 *     TIME0.
	 */
	u32 timestamp_dtu;
	/**
	 * @duration_dtu:
	 *     Duration of the access, or 0 if unknown (this is the case when
	 *     starting a new session).
	 */
	int duration_dtu;
	/**
	 * @chan: Channel number, 5 or 9.
	 */
	int chan;
	/**
	 * @version: Protocol version.
	 */
	int version;
};

/**
 * struct llhw_vendor_cmd_nfcc_coex_get_access_info - NFCC Coex: get access
 * info vendor command.
 */
struct llhw_vendor_cmd_nfcc_coex_get_access_info {
	/**
	 * @stop: If true, the NFCC did not give a next access.
	 */
	bool stop;
	/**
	 * @watchdog_timeout:
	 *     If true, Watchdog triggered before NFCC released the SPI.
	 */
	bool watchdog_timeout;
	/**
	 * @duration_dtu:
	 *     Effective duration of the access. If 0, the current date will be
	 *     read to continue.
	 */
	int duration_dtu;
	/**
	 * @next_timestamp_dtu: Next access date.
	 */
	u32 next_timestamp_dtu;
	/**
	 * @next_duration_dtu: Next access duration, or 0 if unknown.
	 */
	int next_duration_dtu;
};

/**
 * struct llhw_vendor_cmd_nfcc_coex_stop - NFCC Coex: stop
 * vendor command.
 */
struct llhw_vendor_cmd_nfcc_coex_stop {
	/**
	 * @timestamp_dtu:
	 *     Access date when the stop must be sent.
	 */
	u32 timestamp_dtu;
	/**
	 * @duration_dtu:
	 *     Duration of the access.
	 */
	int duration_dtu;
	/**
	 * @version: Protocol version.
	 */
	int version;
};

/**
 * struct llhw_vendor_cmd_pctt_setup_hw - PCTT: direct HW access
 * vendor command.
 */
struct llhw_vendor_cmd_pctt_setup_hw {
	/**
	 * @chan: Channel number, 5 or 9.
	 */
	int chan;
	/**
	 * @rframe_config: STS packet configuration for ranging frame.
	 */
	u8 rframe_config;
	/**
	 * @preamble_code_index: Specifies code index according to Table 16-7 in
	 * IEEE Std 802.15.4-2020 and Table 42 in IEEE Std 802.15.4z-2020
	 */
	u8 preamble_code_index;
	/**
	 * @sfd_id: Start of frame delimiter.
	 */
	u8 sfd_id;
	/**
	 * @psdu_data_rate: PSDU data rate.
	 */
	u8 psdu_data_rate;
	/**
	 * @preamble_duration: Preamble duration.
	 */
	u8 preamble_duration;
};

/**
 * struct llhw_vendor_cmd_pctt_handle_loopback - PCTT: handle loopback access.
 */
struct llhw_vendor_cmd_pctt_handle_loopback {
	/**
	 * @ant_set_id : antenna set index to use for transmit/receive.
	 */
	int ant_set_id;
	/**
	 * @rx_timeout_dtu: If negative, no timeout, if zero, use a default timeout
	 * value, else this is the timeout value in device time unit.
	 */
	int rx_timeout_dtu;
	/**
	 * @rx_frame_timeout_dtu: If no zero, timeout value for the full frame
	 * reception. This allow limiting the length of accepted frame. The
	 * timeout starts after rx_timeout_dtu value.
	 */
	int rx_frame_timeout_dtu;
	/**
	 * @data_payload: Array of payload to send during loopback test.
	 */
	const u8 *data_payload;
	/**
	 * @data_payload_len: Length of the payload array in byte.
	 */
	size_t data_payload_len;
};

/**
 * struct llhw_vendor_cmd_pctt_get_loopback_info - PCTT: get access
 * info vendor command.
 */
struct llhw_vendor_cmd_pctt_get_loopback_info {
	/**
	 * @skb: sk buffer containing received data.
	 */
	struct sk_buff *skb;
	/**
	 * @success: True when data sent match with received.
	 */
	bool success;
	/**
	 * @rssi: Received signal strength indication (RSSI),
	 * absolute value in Q1 fixed point format.
	 */
	int rssi;
	/**
	 * @rx_timestamp_rctu: RX timestamp in RCTU units.
	 */
	u64 rx_timestamp_rctu;
	/**
	 * @tx_timestamp_rctu: TX timestamp in RCTU units.
	 */
	u64 tx_timestamp_rctu;
};

/**
 * struct llhw_vendor_cmd_pctt_get_frame_info - PCTT: last received frame
 * information.
 */
struct llhw_vendor_cmd_pctt_get_frame_info {
	/**
	 * @skb: sk buffer containing received data.
	 */
	struct sk_buff *skb;
	/**
	 * @info: frame information.
	 */
	struct mcps802154_rx_frame_info info;
};

#endif /* NET_VENDOR_CMD_H */
