/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021-2022 Qorvo US, Inc.
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

#ifndef NET_MCPS802154_PCTT_SESSION_H
#define NET_MCPS802154_PCTT_SESSION_H

#include <linux/kernel.h>
#include <net/mcps802154_schedule.h>
#include <net/vendor_cmd.h>
#include <net/pctt_region_params.h>
#include <net/pctt_region_nl.h>

#define PCTT_PAYLOAD_MAX_LEN 4096

struct pctt_session_params {
	enum pctt_device_role device_role;
	__le16 short_addr;
	__le16 dst_short_addr;
	u8 rx_antenna_selection;
	u8 tx_antenna_selection;
	int slot_duration_dtu;
	int channel_number;
	int preamble_code_index;
	enum pctt_rframe_config rframe_config;
	enum pctt_preamble_duration preamble_duration;
	enum pctt_sfd_id sfd_id;
	enum pctt_number_of_sts_segments number_of_sts_segments;
	enum pctt_psdu_data_rate psdu_data_rate;
	enum pctt_mac_fcs_type mac_fcs_type;
	enum pctt_prf_mode prf_mode;
	enum pctt_phr_data_rate phr_data_rate;
	u8 tx_adaptive_payload_power;
	u32 sts_index;
	enum pctt_sts_length sts_length;
	/* Test specific parameters */
	u32 num_packets;
	int gap_duration_dtu;
	u32 t_start;
	u32 t_win;
	u8 randomize_psdu;
	u8 phr_ranging_bit;
	u32 rmarker_tx_start;
	u32 rmarker_rx_start;
	u8 sts_index_auto_incr;
	/* Data payload to put in TX test frame */
	u8 data_payload[PCTT_PAYLOAD_MAX_LEN];
	int data_payload_len;
};

/**
 * struct pctt_session - Session information.
 */
struct pctt_session {
	/**
	 * @params: Session parameters, mostly read only while the session is
	 * active.
	 */
	struct pctt_session_params params;
	/**
	 * @hrp_uwb_params: HRP UWB parameters, read only while the session is
	 * active.
	 */
	struct mcps802154_hrp_uwb_params hrp_uwb_params;
	/**
	 * @event_portid: Port identifier to use for notifications.
	 */
	u32 event_portid;
	/**
	 * @portid_set_once: True when portid have been set once.
	 *
	 * FIXME: To be delete with status notification!
	 */
	bool portid_set_once;
	/**
	 * @first_access: True on the first access.
	 */
	bool first_access;
	/**
	 * @first_rx_synchronized: True after the first successful reception.
	 */
	bool first_rx_synchronized;
	/**
	 * @stop_request: True to not start an another access.
	 */
	bool stop_request;
	/**
	 * @next_timestamp_dtu: next date for next frame.
	 */
	u32 next_timestamp_dtu;
	/**
	 * @setup_hw: setup hardware through a vendor command.
	 */
	struct llhw_vendor_cmd_pctt_setup_hw setup_hw;
	/**
	 * @state: UWB session state.
	 */
	enum pctt_session_state state;
	/**
	 * @cmd_id: test identifier in progress.
	 */
	enum pctt_id_attrs cmd_id;
	/**
	 * @test_on_going: True when test is in progress.
	 */
	bool test_on_going;
};

/* Forward declaration. */
struct pctt_local;

/**
 * pctt_session_init() - Initialize session parameters to default value.
 * @local: PCTT context.
 *
 * Return: 0 on success -errno otherwise.
 */
int pctt_session_init(struct pctt_local *local);

/**
 * pctt_session_deinit() - Declare session as not ready.
 * @local: PCTT context.
 *
 * Return: 0 on success -errno otherwise.
 */
int pctt_session_deinit(struct pctt_local *local);

/**
 * pctt_session_set_state() - Update PCTT state and send notification on change.
 * @local: PCTT context.
 * @new_state: New state to set.
 */
void pctt_session_set_state(struct pctt_local *local,
			    enum pctt_session_state new_state);

/**
 * pctt_session_start_test() - Start PCTT test.
 * @local: PCTT context.
 * @id: Test identifier requested.
 * @info: Request information.
 *
 * Return: 0 on success -errno otherwise.
 */
int pctt_session_start_test(struct pctt_local *local, enum pctt_id_attrs id,
			    const struct genl_info *info);

#endif /* NET_MCPS802154_PCTT_SESSION_H */
