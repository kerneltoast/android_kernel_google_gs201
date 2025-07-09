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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mcps802154_region_pctt

#if !defined(PCTT_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define PCTT_TRACE_H

#include <linux/tracepoint.h>
#include <net/pctt_region_nl.h>
#include <net/pctt_region_params.h>
#include "pctt_session.h"
#include "pctt_region.h"

/* clang-format off */
#define pctt_device_role_name(name)         \
	{ PCTT_DEVICE_ROLE_##name, #name }
#define PCTT_DEVICE_ROLE_SYMBOLS            \
	pctt_device_role_name(RESPONDER),   \
	pctt_device_role_name(INITIATOR)
TRACE_DEFINE_ENUM(PCTT_DEVICE_ROLE_RESPONDER);
TRACE_DEFINE_ENUM(PCTT_DEVICE_ROLE_INITIATOR);
#define PCTT_DEVICE_ROLE_ENTRY __field(enum pctt_device_role, device_role)
#define PCTT_DEVICE_ROLE_ASSIGN __entry->device_role = params->device_role
#define PCTT_DEVICE_ROLE_PR_FMT "device_role=%s"
#define PCTT_DEVICE_ROLE_PR_ARG \
	__print_symbolic(__entry->device_role, PCTT_DEVICE_ROLE_SYMBOLS)

#define pctt_rframe_config_name(name)         \
	{ PCTT_RFRAME_CONFIG_##name, #name }
#define PCTT_RFRAME_CONFIG_SYMBOLS            \
	pctt_rframe_config_name(SP0),         \
	pctt_rframe_config_name(SP1),         \
	pctt_rframe_config_name(SP2),         \
	pctt_rframe_config_name(SP3)
TRACE_DEFINE_ENUM(PCTT_RFRAME_CONFIG_SP0);
TRACE_DEFINE_ENUM(PCTT_RFRAME_CONFIG_SP1);
TRACE_DEFINE_ENUM(PCTT_RFRAME_CONFIG_SP2);
TRACE_DEFINE_ENUM(PCTT_RFRAME_CONFIG_SP3);
#define PCTT_RFRAME_CONFIG_ENTRY __field(enum pctt_rframe_config, rframe_config)
#define PCTT_RFRAME_CONFIG_ASSIGN __entry->rframe_config = params->rframe_config
#define PCTT_RFRAME_CONFIG_PR_FMT "rframe_config=%s"
#define PCTT_RFRAME_CONFIG_PR_ARG \
	__print_symbolic(__entry->rframe_config, PCTT_RFRAME_CONFIG_SYMBOLS)

#define pctt_prf_mode_name(name)         \
	{ PCTT_PRF_MODE_##name, #name }
#define PCTT_PRF_MODE_SYMBOLS              \
	pctt_prf_mode_name(BPRF),          \
	pctt_prf_mode_name(HPRF),          \
	pctt_prf_mode_name(HPRF_HIGH_RATE)
TRACE_DEFINE_ENUM(PCTT_PRF_MODE_BPRF);
TRACE_DEFINE_ENUM(PCTT_PRF_MODE_HPRF);
TRACE_DEFINE_ENUM(PCTT_PRF_MODE_HPRF_HIGH_RATE);
#define PCTT_PRF_MODE_ENTRY __field(enum pctt_prf_mode, prf_mode)
#define PCTT_PRF_MODE_ASSIGN __entry->prf_mode = params->prf_mode
#define PCTT_PRF_MODE_PR_FMT "prf_mode=%s"
#define PCTT_PRF_MODE_PR_ARG \
	__print_symbolic(__entry->prf_mode, PCTT_PRF_MODE_SYMBOLS)

#define pctt_preamble_duration_name(name)         \
	{ PCTT_PREAMBLE_DURATION_##name, #name }
#define PCTT_PREAMBLE_DURATION_SYMBOLS            \
	pctt_preamble_duration_name(32),          \
	pctt_preamble_duration_name(64)
TRACE_DEFINE_ENUM(PCTT_PREAMBLE_DURATION_32);
TRACE_DEFINE_ENUM(PCTT_PREAMBLE_DURATION_64);
#define PCTT_PREAMBLE_DURATION_ENTRY \
	__field(enum pctt_preamble_duration, preamble_duration)
#define PCTT_PREAMBLE_DURATION_ASSIGN \
	__entry->preamble_duration = params->preamble_duration
#define PCTT_PREAMBLE_DURATION_PR_FMT "preamble_duration=%s"
#define PCTT_PREAMBLE_DURATION_PR_ARG                \
	__print_symbolic(__entry->preamble_duration, \
			 PCTT_PREAMBLE_DURATION_SYMBOLS)

#define pctt_sfd_id_name(name)         \
	{ PCTT_SFD_ID_##name, #name }
#define PCTT_SFD_ID_SYMBOLS            \
	pctt_sfd_id_name(0),           \
	pctt_sfd_id_name(1),           \
	pctt_sfd_id_name(2),           \
	pctt_sfd_id_name(3),           \
	pctt_sfd_id_name(4)
TRACE_DEFINE_ENUM(PCTT_SFD_ID_0);
TRACE_DEFINE_ENUM(PCTT_SFD_ID_1);
TRACE_DEFINE_ENUM(PCTT_SFD_ID_2);
TRACE_DEFINE_ENUM(PCTT_SFD_ID_3);
TRACE_DEFINE_ENUM(PCTT_SFD_ID_4);
#define PCTT_SFD_ID_ENTRY __field(enum pctt_sfd_id, sfd_id)
#define PCTT_SFD_ID_ASSIGN __entry->sfd_id = params->sfd_id
#define PCTT_SFD_ID_PR_FMT "sfd_id=%s"
#define PCTT_SFD_ID_PR_ARG \
	__print_symbolic(__entry->sfd_id, PCTT_SFD_ID_SYMBOLS)

#define pctt_number_of_sts_segments_name(name)         \
	{ PCTT_NUMBER_OF_STS_SEGMENTS_##name, #name }
#define PCTT_NUMBER_OF_STS_SEGMENTS_SYMBOLS            \
	pctt_number_of_sts_segments_name(NONE),        \
	pctt_number_of_sts_segments_name(1_SEGMENT),   \
	pctt_number_of_sts_segments_name(2_SEGMENTS),  \
	pctt_number_of_sts_segments_name(3_SEGMENTS),  \
	pctt_number_of_sts_segments_name(4_SEGMENTS)
TRACE_DEFINE_ENUM(PCTT_NUMBER_OF_STS_SEGMENTS_NONE);
TRACE_DEFINE_ENUM(PCTT_NUMBER_OF_STS_SEGMENTS_1_SEGMENT);
TRACE_DEFINE_ENUM(PCTT_NUMBER_OF_STS_SEGMENTS_2_SEGMENTS);
TRACE_DEFINE_ENUM(PCTT_NUMBER_OF_STS_SEGMENTS_3_SEGMENTS);
TRACE_DEFINE_ENUM(PCTT_NUMBER_OF_STS_SEGMENTS_4_SEGMENTS);
#define PCTT_NUMBER_OF_STS_SEGMENTS_ENTRY \
	__field(enum pctt_number_of_sts_segments, number_of_sts_segments)
#define PCTT_NUMBER_OF_STS_SEGMENTS_ASSIGN \
	__entry->number_of_sts_segments = params->number_of_sts_segments
#define PCTT_NUMBER_OF_STS_SEGMENTS_PR_FMT "number_of_sts_segments=%s"
#define PCTT_NUMBER_OF_STS_SEGMENTS_PR_ARG                \
	__print_symbolic(__entry->number_of_sts_segments, \
			 PCTT_NUMBER_OF_STS_SEGMENTS_SYMBOLS)

#define pctt_psdu_data_rate_name(name)         \
	{ PCTT_PSDU_DATA_RATE_##name, #name }
#define PCTT_PSDU_DATA_RATE_SYMBOLS            \
	pctt_psdu_data_rate_name(6M81),        \
	pctt_psdu_data_rate_name(7M80),        \
	pctt_psdu_data_rate_name(27M2),        \
	pctt_psdu_data_rate_name(31M2)
TRACE_DEFINE_ENUM(PCTT_PSDU_DATA_RATE_6M81);
TRACE_DEFINE_ENUM(PCTT_PSDU_DATA_RATE_7M80);
TRACE_DEFINE_ENUM(PCTT_PSDU_DATA_RATE_27M2);
TRACE_DEFINE_ENUM(PCTT_PSDU_DATA_RATE_31M2);
#define PCTT_PSDU_DATA_RATE_ENTRY \
	__field(enum pctt_psdu_data_rate, psdu_data_rate)
#define PCTT_PSDU_DATA_RATE_ASSIGN \
	__entry->psdu_data_rate = params->psdu_data_rate
#define PCTT_PSDU_DATA_RATE_PR_FMT "psdu_data_rate=%s"
#define PCTT_PSDU_DATA_RATE_PR_ARG \
	__print_symbolic(__entry->psdu_data_rate, PCTT_PSDU_DATA_RATE_SYMBOLS)

#define pctt_phr_data_rate_name(name)         \
	{ PCTT_PHR_DATA_RATE##name, #name }
#define PCTT_PHR_DATA_RATE_SYMBOLS            \
	pctt_phr_data_rate_name(850k),        \
	pctt_phr_data_rate_name(6M81),        \
TRACE_DEFINE_ENUM(PCTT_PHR_DATA_RATE_850k);
TRACE_DEFINE_ENUM(PCTT_PHR_DATA_RATE_6M81);
#define PCTT_PHR_DATA_RATE_ENTRY \
	__field(enum pctt_phr_data_rate, phr_data_rate)
#define PCTT_PHR_DATA_RATE_ASSIGN \
	__entry->phr_data_rate = params->phr_data_rate
#define PCTT_PHR_DATA_RATE_PR_FMT "phr_data_rate=%s"
#define PCTT_PHR_DATA_RATE_PR_ARG \
	__print_symbolic(__entry->phr_data_rate, PCTT_PHR_DATA_RATE_SYMBOLS)

#define pctt_mac_fcs_type_name(name)         \
	{ PCTT_MAC_FCS_TYPE_##name, #name }
#define PCTT_MAC_FCS_TYPE_SYMBOLS            \
	pctt_mac_fcs_type_name(CRC_16),      \
	pctt_mac_fcs_type_name(CRC_32)
TRACE_DEFINE_ENUM(PCTT_MAC_FCS_TYPE_CRC_16);
TRACE_DEFINE_ENUM(PCTT_MAC_FCS_TYPE_CRC_32);
#define PCTT_MAC_FCS_TYPE_ENTRY __field(enum pctt_mac_fcs_type, mac_fcs_type)
#define PCTT_MAC_FCS_TYPE_ASSIGN __entry->mac_fcs_type = params->mac_fcs_type
#define PCTT_MAC_FCS_TYPE_PR_FMT "mac_fcs_type=%s"
#define PCTT_MAC_FCS_TYPE_PR_ARG \
	__print_symbolic(__entry->mac_fcs_type, PCTT_MAC_FCS_TYPE_SYMBOLS)

/* clang-format off */
#define pctt_status_ranging_name(name)                    \
	{ PCTT_STATUS_RANGING_##name, #name }
#define PCTT_STATUS_RANGING_SYMBOLS                       \
	pctt_status_ranging_name(INTERNAL_ERROR),         \
	pctt_status_ranging_name(SUCCESS),                \
	pctt_status_ranging_name(TX_FAILED),              \
	pctt_status_ranging_name(RX_TIMEOUT),             \
	pctt_status_ranging_name(RX_PHY_DEC_FAILED),      \
	pctt_status_ranging_name(RX_PHY_TOA_FAILED),      \
	pctt_status_ranging_name(RX_PHY_STS_FAILED),      \
	pctt_status_ranging_name(RX_MAC_DEC_FAILED),      \
	pctt_status_ranging_name(RX_MAC_IE_DEC_FAILED),   \
	pctt_status_ranging_name(RX_MAC_IE_MISSING)
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_INTERNAL_ERROR);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_SUCCESS);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_TX_FAILED);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_RX_TIMEOUT);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_RX_PHY_DEC_FAILED);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_RX_PHY_TOA_FAILED);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_RX_PHY_STS_FAILED);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_RX_MAC_DEC_FAILED);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_RX_MAC_IE_DEC_FAILED);
TRACE_DEFINE_ENUM(PCTT_STATUS_RANGING_RX_MAC_IE_MISSING);
#define PCTT_STATUS_RANGING_ENTRY \
	__field(enum pctt_status_ranging, status_ranging)
#define PCTT_STATUS_RANGING_ASSIGN __entry->status_ranging = status_ranging
#define PCTT_STATUS_RANGING_PR_FMT "status_ranging=%s"
#define PCTT_STATUS_RANGING_PR_ARG \
	__print_symbolic(__entry->status_ranging, PCTT_STATUS_RANGING_SYMBOLS)

#define pctt_session_state_name(name)         \
	{ PCTT_SESSION_STATE_##name, #name }
#define PCTT_SESSION_STATE_SYMBOLS            \
	pctt_session_state_name(INIT),        \
	pctt_session_state_name(DEINIT),      \
	pctt_session_state_name(ACTIVE),      \
	pctt_session_state_name(IDLE)
TRACE_DEFINE_ENUM(PCTT_SESSION_STATE_INIT);
TRACE_DEFINE_ENUM(PCTT_SESSION_STATE_DEINIT);
TRACE_DEFINE_ENUM(PCTT_SESSION_STATE_ACTIVE);
TRACE_DEFINE_ENUM(PCTT_SESSION_STATE_IDLE);
#define PCTT_SESSION_STATE_ENTRY(name) __field(enum pctt_session_state, name)
#define PCTT_SESSION_STATE_ASSIGN(name) __entry->name = name
#define PCTT_SESSION_STATE_PR_FMT(name) "name=%s"
#define PCTT_SESSION_STATE_PR_ARG(name) \
	__print_symbolic(__entry->name, PCTT_SESSION_STATE_SYMBOLS)

#define pctt_id_name(name)         \
	{ PCTT_ID_ATTR_##name, #name }
#define PCTT_ID_SYMBOLS              \
	pctt_id_name(UNSPEC),        \
	pctt_id_name(PERIODIC_TX),   \
	pctt_id_name(PER_RX),        \
	pctt_id_name(RX),            \
	pctt_id_name(LOOPBACK),      \
	pctt_id_name(SS_TWR),        \
	pctt_id_name(RX)
TRACE_DEFINE_ENUM(PCTT_ID_ATTR_UNSPEC);
TRACE_DEFINE_ENUM(PCTT_ID_ATTR_PERIODIC_TX);
TRACE_DEFINE_ENUM(PCTT_ID_ATTR_PER_RX);
TRACE_DEFINE_ENUM(PCTT_ID_ATTR_RX);
TRACE_DEFINE_ENUM(PCTT_ID_ATTR_LOOPBACK);
TRACE_DEFINE_ENUM(PCTT_ID_ATTR_SS_TWR);
TRACE_DEFINE_ENUM(PCTT_ID_ATTR_STOP_TEST);
#define PCTT_ID_ENTRY __field(enum pctt_id_attrs, cmd_id)
#define PCTT_ID_ASSIGN __entry->cmd_id = cmd_id
#define PCTT_ID_PR_FMT "cmd_id=%s"
#define PCTT_ID_PR_ARG \
	__print_symbolic(__entry->cmd_id, PCTT_ID_SYMBOLS)

TRACE_EVENT(
	region_pctt_session_start_test,
	TP_PROTO(enum pctt_id_attrs cmd_id,
		 const struct pctt_session_params *params),
	TP_ARGS(cmd_id, params),
	TP_STRUCT__entry(
		PCTT_ID_ENTRY
		PCTT_DEVICE_ROLE_ENTRY
		__field(__le16, short_addr)
		__field(__le16, dst_short_addr)
		__field(int, slot_duration_dtu)
		__field(int, channel_number)
		__field(int, preamble_code_index)
		PCTT_RFRAME_CONFIG_ENTRY
		PCTT_PRF_MODE_ENTRY
		PCTT_PREAMBLE_DURATION_ENTRY
		PCTT_SFD_ID_ENTRY
		PCTT_NUMBER_OF_STS_SEGMENTS_ENTRY
		PCTT_PSDU_DATA_RATE_ENTRY
		PCTT_MAC_FCS_TYPE_ENTRY
		__field(u8, tx_adaptive_payload_power)
		__field(uint32_t, sts_index)
	),
	TP_fast_assign(
		PCTT_ID_ASSIGN;
		PCTT_DEVICE_ROLE_ASSIGN;
		__entry->short_addr = params->short_addr;
		__entry->dst_short_addr = params->dst_short_addr;
		__entry->slot_duration_dtu = params->slot_duration_dtu;
		__entry->channel_number = params->channel_number;
		__entry->preamble_code_index = params->preamble_code_index;
		PCTT_RFRAME_CONFIG_ASSIGN;
		PCTT_PRF_MODE_ASSIGN;
		PCTT_PREAMBLE_DURATION_ASSIGN;
		PCTT_SFD_ID_ASSIGN;
		PCTT_NUMBER_OF_STS_SEGMENTS_ASSIGN;
		PCTT_PSDU_DATA_RATE_ASSIGN;
		PCTT_MAC_FCS_TYPE_ASSIGN;
		__entry->tx_adaptive_payload_power =
			params->tx_adaptive_payload_power;
		__entry->sts_index = params->sts_index;
	),
	TP_printk(
		PCTT_ID_PR_FMT " " PCTT_DEVICE_ROLE_PR_FMT " "
		"short_addr=0x%x "
		"dst_short_addr=0x%x slot_duration_dtu=%d "
		"channel_number=%d preamble_code_index=%d "
		PCTT_RFRAME_CONFIG_PR_FMT " "
		PCTT_PRF_MODE_PR_FMT " "
		PCTT_PREAMBLE_DURATION_PR_FMT " "
		PCTT_SFD_ID_PR_FMT " "
		PCTT_NUMBER_OF_STS_SEGMENTS_PR_FMT " "
		PCTT_PSDU_DATA_RATE_PR_FMT " "
		PCTT_MAC_FCS_TYPE_PR_FMT " "
		"tx_adaptive_payload_power=%u sts_index=%u",
		PCTT_ID_PR_ARG, PCTT_DEVICE_ROLE_PR_ARG,
		__entry->short_addr,
		__entry->dst_short_addr,
		__entry->slot_duration_dtu,
		__entry->channel_number,
		__entry->preamble_code_index,
		PCTT_RFRAME_CONFIG_PR_ARG,
		PCTT_PRF_MODE_PR_ARG,
		PCTT_PREAMBLE_DURATION_PR_ARG,
		PCTT_SFD_ID_PR_ARG,
		PCTT_NUMBER_OF_STS_SEGMENTS_PR_ARG,
		PCTT_PSDU_DATA_RATE_PR_ARG,
		PCTT_MAC_FCS_TYPE_PR_ARG,
		__entry->tx_adaptive_payload_power,
		__entry->sts_index
	)
);

TRACE_EVENT(region_pctt_session_set_state,
	    TP_PROTO(enum pctt_session_state old_state,
		     enum pctt_session_state new_state),
	    TP_ARGS(old_state, new_state),
	    TP_STRUCT__entry(
		PCTT_SESSION_STATE_ENTRY(old_state)
		PCTT_SESSION_STATE_ENTRY(new_state)
	    ),
	    TP_fast_assign(
		PCTT_SESSION_STATE_ASSIGN(old_state);
		PCTT_SESSION_STATE_ASSIGN(new_state);
	    ),
	    TP_printk(PCTT_SESSION_STATE_PR_FMT(old_state) " "
		      PCTT_SESSION_STATE_PR_FMT(new_state),
		      PCTT_SESSION_STATE_PR_ARG(old_state),
		      PCTT_SESSION_STATE_PR_ARG(new_state))
);

TRACE_EVENT(region_pctt_report_periodic_tx,
	    TP_PROTO(enum pctt_status_ranging status_ranging),
	    TP_ARGS(status_ranging),
	    TP_STRUCT__entry(
		PCTT_STATUS_RANGING_ENTRY
	    ),
	    TP_fast_assign(
		PCTT_STATUS_RANGING_ASSIGN;
	    ),
	    TP_printk(PCTT_STATUS_RANGING_PR_FMT,
		      PCTT_STATUS_RANGING_PR_ARG)
);

TRACE_EVENT(region_pctt_report_per_rx,
	    TP_PROTO(enum pctt_status_ranging status_ranging,
		     const struct pctt_test_per_rx_results *per_rx),
	    TP_ARGS(status_ranging, per_rx),
	    TP_STRUCT__entry(
		PCTT_STATUS_RANGING_ENTRY
		__field(u32, attempts)
		__field(u32, acq_detect)
		__field(u32, acq_reject)
		__field(u32, rx_fail)
		__field(u32, sync_cir_ready)
		__field(u32, sfd_fail)
		__field(u32, sfd_found)
		__field(u32, phr_dec_error)
		__field(u32, phr_bit_error)
		__field(u32, psdu_dec_error)
		__field(u32, psdu_bit_error)
		__field(u32, sts_found)
		__field(u32, eof)
		__field(u8, rssi)
	    ),
	    TP_fast_assign(
		PCTT_STATUS_RANGING_ASSIGN;
		__entry->attempts = per_rx->attempts;
		__entry->acq_detect = per_rx->acq_detect;
		__entry->acq_reject = per_rx->acq_reject;
		__entry->rx_fail = per_rx->rx_fail;
		__entry->sync_cir_ready = per_rx->sync_cir_ready;
		__entry->sfd_fail = per_rx->sfd_fail;
		__entry->sfd_found = per_rx->sfd_found;
		__entry->phr_dec_error = per_rx->phr_dec_error;
		__entry->phr_bit_error = per_rx->phr_bit_error;
		__entry->psdu_dec_error = per_rx->psdu_dec_error;
		__entry->psdu_bit_error = per_rx->psdu_bit_error;
		__entry->sts_found = per_rx->sts_found;
		__entry->eof = per_rx->eof;
		__entry->rssi = per_rx->rssi;
	    ),
	    TP_printk(PCTT_STATUS_RANGING_PR_FMT " "
		      "attempts=%u acq_detect=%u acq_reject=%u "
		      "rx_fail=%u sync_cir_ready=%u sfd_fail=%u "
		      "sfd_found=%u phr_dec_error=%u phr_bit_error=%u "
		      "psdu_dec_error=%u psdu_bit_error=%u sts_found=%u "
		      "eof=%u rssi=%u ",
		      PCTT_STATUS_RANGING_PR_ARG, __entry->attempts,
		      __entry->acq_detect, __entry->acq_reject,
		      __entry->rx_fail, __entry->sync_cir_ready,
		      __entry->sfd_fail, __entry->sfd_found,
		      __entry->phr_dec_error, __entry->phr_bit_error,
		      __entry->psdu_dec_error, __entry->psdu_bit_error,
		      __entry->sts_found, __entry->eof, __entry->rssi)
);

TRACE_EVENT(region_pctt_report_rx,
	    TP_PROTO(enum pctt_status_ranging status_ranging,
		     const struct pctt_test_rx_results *rx),
	    TP_ARGS(status_ranging, rx),
	    TP_STRUCT__entry(
		PCTT_STATUS_RANGING_ENTRY
		__field(u32, rx_done_ts_int)
		__field(u16, rx_done_ts_frac)
		__field(s16, aoa_azimuth)
		__field(s16, aoa_elevation)
		__field(u8, toa_gap)
		__field(u16, phr)
		__field(u8, rssi)
		__field(u16, psdu_data_len)
		__dynamic_array(u8, psdu_data, rx->psdu_data_len)
	    ),
	    TP_fast_assign(
		PCTT_STATUS_RANGING_ASSIGN;
		__entry->rx_done_ts_int = rx->rx_done_ts_int;
		__entry->rx_done_ts_frac = rx->rx_done_ts_frac;
		__entry->aoa_azimuth = rx->aoa_azimuth;
		__entry->aoa_elevation = rx->aoa_elevation;
		__entry->toa_gap = rx->toa_gap;
		__entry->phr = rx->phr;
		__entry->rssi = rx->rssi;
		__entry->psdu_data_len = rx->psdu_data_len;
		memcpy(__get_dynamic_array(psdu_data), rx->psdu_data,
		       __get_dynamic_array_len(psdu_data));

	    ),
	    TP_printk(PCTT_STATUS_RANGING_PR_FMT " "
		      "rx_done_ts_int=%u rx_done_ts_frac=%u "
		      "aoa_azimuth=%d aoa_elevation=%d toa_gap=%u "
		      "phr=%u rssi=%u  psdu_data_len=%u psdu_data=%s",
		      PCTT_STATUS_RANGING_PR_ARG, __entry->rx_done_ts_int,
		      __entry->rx_done_ts_frac, __entry->aoa_azimuth,
		      __entry->aoa_elevation, __entry->toa_gap, __entry->phr,
		      __entry->rssi, __entry->psdu_data_len,
		      __print_hex(__get_dynamic_array(psdu_data),
				  __get_dynamic_array_len(psdu_data)))
);

TRACE_EVENT(region_pctt_report_loopback,
	    TP_PROTO(enum pctt_status_ranging status_ranging),
	    TP_ARGS(status_ranging),
	    TP_STRUCT__entry(
		PCTT_STATUS_RANGING_ENTRY
	    ),
	    TP_fast_assign(
		PCTT_STATUS_RANGING_ASSIGN;
	    ),
	    TP_printk(PCTT_STATUS_RANGING_PR_FMT,
		      PCTT_STATUS_RANGING_PR_ARG)
);

TRACE_EVENT(region_pctt_report_ss_twr,
	    TP_PROTO(enum pctt_status_ranging status_ranging,
		     const struct pctt_test_ss_twr_results *ss_twr),
	    TP_ARGS(status_ranging, ss_twr),
	    TP_STRUCT__entry(
		PCTT_STATUS_RANGING_ENTRY
		__field(u32, measurement_rctu)
		__field(s16, pdoa_azimuth_deg_q7)
		__field(s16, pdoa_elevation_deg_q7)
		__field(u8, rssi)
		__field(s16, aoa_azimuth_deg_q7)
		__field(s16, aoa_elevation_deg_q7)
	    ),
	    TP_fast_assign(
		PCTT_STATUS_RANGING_ASSIGN;
		__entry->measurement_rctu = ss_twr->measurement_rctu;
		__entry->pdoa_azimuth_deg_q7 = ss_twr->pdoa_azimuth_deg_q7;
		__entry->pdoa_elevation_deg_q7 = ss_twr->pdoa_elevation_deg_q7;
		__entry->rssi = ss_twr->rssi;
		__entry->aoa_azimuth_deg_q7 = ss_twr->aoa_azimuth_deg_q7;
		__entry->aoa_elevation_deg_q7 = ss_twr->aoa_elevation_deg_q7;
	    ),
	    TP_printk(PCTT_STATUS_RANGING_PR_FMT " measurement_rctu=%u "
				  "pdoa_azimuth_deg_q7=%u  pdoa_elevation_deg_q7=%u "
				  "rssi=%u aoa_azimuth_deg_q7=%u aoa_elevation_deg_q7=%u",
		      PCTT_STATUS_RANGING_PR_ARG, __entry->measurement_rctu,
			  __entry->pdoa_azimuth_deg_q7, __entry->pdoa_elevation_deg_q7,
			  __entry->rssi, __entry->aoa_azimuth_deg_q7, __entry->aoa_elevation_deg_q7)
);

TRACE_EVENT(region_pctt_report_nla_put_failure,
	    TP_PROTO(enum pctt_id_attrs cmd_id),
	    TP_ARGS(cmd_id),
	    TP_STRUCT__entry(
		PCTT_ID_ENTRY
	    ),
	    TP_fast_assign(
		PCTT_ID_ASSIGN;
	    ),
	    TP_printk(PCTT_ID_PR_FMT,
		      PCTT_ID_PR_ARG)
);
/* clang-format on */

#endif /* !PCTT_TRACE_H || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE pctt_trace
#include <trace/define_trace.h>
