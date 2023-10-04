/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2022 Qorvo US, Inc.
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
#define TRACE_SYSTEM mcps802154_region_fira

#if !defined(FIRA_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define FIRA_TRACE_H

#include <linux/tracepoint.h>
#include "fira_session.h"
#include "net/fira_region_params.h"
#include <net/fira_region_nl.h>

/* clang-format off */

#define FIRA_SESSION_ENTRY __field(u32, session_id)
#define FIRA_SESSION_ASSIGN __entry->session_id = session->id
#define FIRA_SESSION_PR_FMT "session_id=%d"
#define FIRA_SESSION_PR_ARG __entry->session_id

#define FIRA_DEVICE_TYPE_SYMBOLS                      \
	{ FIRA_DEVICE_TYPE_CONTROLEE, "controlee" },  \
	{ FIRA_DEVICE_TYPE_CONTROLLER, "controller" }
TRACE_DEFINE_ENUM(FIRA_DEVICE_TYPE_CONTROLEE);
TRACE_DEFINE_ENUM(FIRA_DEVICE_TYPE_CONTROLLER);

#define FIRA_DEVICE_ROLE_SYMBOLS                      \
	{ FIRA_DEVICE_ROLE_RESPONDER, "responder" },  \
	{ FIRA_DEVICE_ROLE_INITIATOR, "initiator" }
TRACE_DEFINE_ENUM(FIRA_DEVICE_ROLE_RESPONDER);
TRACE_DEFINE_ENUM(FIRA_DEVICE_ROLE_INITIATOR);

#define FIRA_RANGING_ROUND_SYMBOLS                    \
	{ FIRA_RANGING_ROUND_USAGE_OWR, "OWR" },      \
	{ FIRA_RANGING_ROUND_USAGE_SSTWR, "SSTWR" },  \
	{ FIRA_RANGING_ROUND_USAGE_DSTWR, "DSTWR" }
TRACE_DEFINE_ENUM(FIRA_RANGING_ROUND_USAGE_OWR);
TRACE_DEFINE_ENUM(FIRA_RANGING_ROUND_USAGE_SSTWR);
TRACE_DEFINE_ENUM(FIRA_RANGING_ROUND_USAGE_DSTWR);

#define FIRA_MULTI_NODE_MODE_SYMBOLS                          \
	{ FIRA_MULTI_NODE_MODE_UNICAST, "UNICAST" },          \
	{ FIRA_MULTI_NODE_MODE_ONE_TO_MANY, "ONE_TO_MANY" },  \
	{ FIRA_MULTI_NODE_MODE_MANY_TO_MANY, "MANY_TO_MANY" }
TRACE_DEFINE_ENUM(FIRA_MULTI_NODE_MODE_UNICAST);
TRACE_DEFINE_ENUM(FIRA_MULTI_NODE_MODE_ONE_TO_MANY);
TRACE_DEFINE_ENUM(FIRA_MULTI_NODE_MODE_MANY_TO_MANY);

#define FIRA_RFRAME_CONFIG_SYMBOLS          \
	{ FIRA_RFRAME_CONFIG_SP0, "SP0" },  \
	{ FIRA_RFRAME_CONFIG_SP1, "SP1" },  \
	{ FIRA_RFRAME_CONFIG_SP2, "SP2" },  \
	{ FIRA_RFRAME_CONFIG_SP3, "SP3" }
TRACE_DEFINE_ENUM(FIRA_RFRAME_CONFIG_SP0);
TRACE_DEFINE_ENUM(FIRA_RFRAME_CONFIG_SP1);
TRACE_DEFINE_ENUM(FIRA_RFRAME_CONFIG_SP2);
TRACE_DEFINE_ENUM(FIRA_RFRAME_CONFIG_SP3);

#define FIRA_PREAMBULE_DURATION_SYMBOLS        \
	{ FIRA_PREAMBULE_DURATION_32, "32" },  \
	{ FIRA_PREAMBULE_DURATION_64, "64" }
TRACE_DEFINE_ENUM(FIRA_PREAMBULE_DURATION_32);
TRACE_DEFINE_ENUM(FIRA_PREAMBULE_DURATION_64);

#define FIRA_STS_SEGMENTS_SYMBOLS    		\
	{ FIRA_STS_SEGMENTS_0, "0" },	\
	{ FIRA_STS_SEGMENTS_1, "1" },  \
	{ FIRA_STS_SEGMENTS_2, "2" },  \
	{ FIRA_STS_SEGMENTS_3, "3" },  \
	{ FIRA_STS_SEGMENTS_4, "4" }
TRACE_DEFINE_ENUM(FIRA_STS_SEGMENTS_0);
TRACE_DEFINE_ENUM(FIRA_STS_SEGMENTS_1);
TRACE_DEFINE_ENUM(FIRA_STS_SEGMENTS_2);
TRACE_DEFINE_ENUM(FIRA_STS_SEGMENTS_3);
TRACE_DEFINE_ENUM(FIRA_STS_SEGMENTS_4);

#define FIRA_PSDU_DATA_RATE_SYMBOLS            \
	{ FIRA_PSDU_DATA_RATE_6M81, "6M81" },  \
	{ FIRA_PSDU_DATA_RATE_7M80, "7M80" },  \
	{ FIRA_PSDU_DATA_RATE_27M2, "27M2" },  \
	{ FIRA_PSDU_DATA_RATE_31M2, "31M2" }
TRACE_DEFINE_ENUM(FIRA_PSDU_DATA_RATE_6M81);
TRACE_DEFINE_ENUM(FIRA_PSDU_DATA_RATE_7M80);
TRACE_DEFINE_ENUM(FIRA_PSDU_DATA_RATE_27M2);
TRACE_DEFINE_ENUM(FIRA_PSDU_DATA_RATE_31M2);

#define FIRA_PHR_DATA_RATE_SYMBOLS            \
	{ FIRA_PHR_DATA_RATE_850K, "850k" },  \
	{ FIRA_PHR_DATA_RATE_6M81, "6M81" }
TRACE_DEFINE_ENUM(FIRA_PHR_DATA_RATE_850K);
TRACE_DEFINE_ENUM(FIRA_PHR_DATA_RATE_6M81);

#define FIRA_MAC_FCS_TYPE_CRC_SYMBOLS        \
	{ FIRA_MAC_FCS_TYPE_CRC_16, "16" },  \
	{ FIRA_MAC_FCS_TYPE_CRC_32, "32" }
TRACE_DEFINE_ENUM(FIRA_MAC_FCS_TYPE_CRC_16);
TRACE_DEFINE_ENUM(FIRA_MAC_FCS_TYPE_CRC_32);

#define FIRA_STS_MODE_SYMBOLS                  \
	{ FIRA_STS_MODE_STATIC, "static" },    \
	{ FIRA_STS_MODE_DYNAMIC, "dynamic" },  \
	{ FIRA_STS_MODE_DYNAMIC_INDIVIDUAL_KEY, "dynamic_individual_key" },  \
	{ FIRA_STS_MODE_PROVISIONED, "provisioned" },  \
	{ FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY, "provisioned_individual_key" }
TRACE_DEFINE_ENUM(FIRA_STS_MODE_STATIC);
TRACE_DEFINE_ENUM(FIRA_STS_MODE_DYNAMIC);
TRACE_DEFINE_ENUM(FIRA_STS_MODE_DYNAMIC_INDIVIDUAL_KEY);
TRACE_DEFINE_ENUM(FIRA_STS_MODE_PROVISIONED);
TRACE_DEFINE_ENUM(FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY);

#define FIRA_MESSAGE_TYPE                              \
	{ FIRA_MESSAGE_ID_RANGING_INITIATION, "RIM" }, \
	{ FIRA_MESSAGE_ID_RANGING_RESPONSE, "RRM" },   \
	{ FIRA_MESSAGE_ID_RANGING_FINAL, "RFM" },      \
	{ FIRA_MESSAGE_ID_CONTROL, "RCM" },            \
	{ FIRA_MESSAGE_ID_MEASUREMENT_REPORT, "MRM" }, \
	{ FIRA_MESSAGE_ID_RESULT_REPORT, "RRRM" },     \
	{ FIRA_MESSAGE_ID_CONTROL_UPDATE, "CMU" }
TRACE_DEFINE_ENUM(FIRA_MESSAGE_ID_RANGING_INITIATION);
TRACE_DEFINE_ENUM(FIRA_MESSAGE_ID_RANGING_RESPONSE);
TRACE_DEFINE_ENUM(FIRA_MESSAGE_ID_RANGING_FINAL);
TRACE_DEFINE_ENUM(FIRA_MESSAGE_ID_CONTROL);
TRACE_DEFINE_ENUM(FIRA_MESSAGE_ID_MEASUREMENT_REPORT);
TRACE_DEFINE_ENUM(FIRA_MESSAGE_ID_RESULT_REPORT);
TRACE_DEFINE_ENUM(FIRA_MESSAGE_ID_CONTROL_UPDATE);

#define mcps802154_rx_error_name(name)                               \
	{                                                            \
		MCPS802154_RX_ERROR_##name, #name                    \
	}
#define MCPS802154_RX_ERROR_SYMBOLS                                  \
	mcps802154_rx_error_name(NONE),                              \
	mcps802154_rx_error_name(TIMEOUT),                           \
	mcps802154_rx_error_name(BAD_CKSUM),                         \
	mcps802154_rx_error_name(UNCORRECTABLE),                     \
	mcps802154_rx_error_name(FILTERED),                          \
	mcps802154_rx_error_name(SFD_TIMEOUT),                       \
	mcps802154_rx_error_name(PHR_DECODE),                        \
	mcps802154_rx_error_name(OTHER)
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_NONE);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_TIMEOUT);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_BAD_CKSUM);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_UNCORRECTABLE);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_FILTERED);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_SFD_TIMEOUT);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_PHR_DECODE);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_OTHER);

#define mcps802154_tx_reason_name(name)                              \
	{                                                            \
		MCPS802154_ACCESS_TX_RETURN_REASON_##name, #name     \
	}
#define MCPS802154_TX_REASON_SYMBOLS                                 \
	mcps802154_tx_reason_name(CONSUMED),                         \
	mcps802154_tx_reason_name(FAILURE),                          \
	mcps802154_tx_reason_name(CANCEL)
TRACE_DEFINE_ENUM(MCPS802154_ACCESS_TX_RETURN_REASON_CONSUMED);
TRACE_DEFINE_ENUM(MCPS802154_ACCESS_TX_RETURN_REASON_FAILURE);
TRACE_DEFINE_ENUM(MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL);

#define FIRA_MEAS_SEQ_STEP_TYPE                                   \
	{ FIRA_MEASUREMENT_TYPE_RANGE, "range" },                   \
	{ FIRA_MEASUREMENT_TYPE_AOA, "AoA" },                       \
	{ FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH, "AoA Azimuth" },       \
	{ FIRA_MEASUREMENT_TYPE_AOA_ELEVATION, "AoA Elevation" },   \
	{ FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH_ELEVATION,              \
		"AoA Azimuth/Elevation" }
TRACE_DEFINE_ENUM(FIRA_MEASUREMENT_TYPE_RANGE);
TRACE_DEFINE_ENUM(FIRA_MEASUREMENT_TYPE_AOA);
TRACE_DEFINE_ENUM(FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH);
TRACE_DEFINE_ENUM(FIRA_MEASUREMENT_TYPE_AOA_ELEVATION);
TRACE_DEFINE_ENUM(FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH_ELEVATION);

#define fira_session_state_id_name(name)                             \
	{                                                            \
		FIRA_SESSION_STATE_ID_##name, #name                  \
	}
#define FIRA_SESSION_STATE_ID_SYMBOLS                                \
	fira_session_state_id_name(DEINIT),                          \
	fira_session_state_id_name(INIT),                            \
	fira_session_state_id_name(ACTIVE),                          \
	fira_session_state_id_name(IDLE)
TRACE_DEFINE_ENUM(FIRA_SESSION_STATE_ID_DEINIT);
TRACE_DEFINE_ENUM(FIRA_SESSION_STATE_ID_INIT);
TRACE_DEFINE_ENUM(FIRA_SESSION_STATE_ID_ACTIVE);
TRACE_DEFINE_ENUM(FIRA_SESSION_STATE_ID_IDLE);


#define fira_call_name(name)                                         \
	{                                                            \
		FIRA_CALL_##name, #name                              \
	}
#define FIRA_CALL_SYMBOLS                                            \
	fira_call_name(GET_CAPABILITIES),                            \
	fira_call_name(SESSION_INIT),                                \
	fira_call_name(SESSION_START),                               \
	fira_call_name(SESSION_STOP),                                \
	fira_call_name(SESSION_DEINIT),                              \
	fira_call_name(SESSION_SET_PARAMS),                          \
	fira_call_name(NEW_CONTROLEE),                               \
	fira_call_name(DEL_CONTROLEE),                               \
	fira_call_name(SESSION_NOTIFICATION),                        \
	fira_call_name(SESSION_GET_PARAMS),                          \
	fira_call_name(SESSION_GET_STATE),                           \
	fira_call_name(SESSION_GET_COUNT),                           \
	fira_call_name(SET_CONTROLEE),                               \
	fira_call_name(GET_CONTROLEES)
TRACE_DEFINE_ENUM(FIRA_CALL_GET_CAPABILITIES);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_INIT);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_START);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_STOP);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_DEINIT);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_SET_PARAMS);
TRACE_DEFINE_ENUM(FIRA_CALL_NEW_CONTROLEE);
TRACE_DEFINE_ENUM(FIRA_CALL_DEL_CONTROLEE);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_NOTIFICATION);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_GET_PARAMS);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_GET_STATE);
TRACE_DEFINE_ENUM(FIRA_CALL_SESSION_GET_COUNT);
TRACE_DEFINE_ENUM(FIRA_CALL_SET_CONTROLEE);
TRACE_DEFINE_ENUM(FIRA_CALL_GET_CONTROLEES);


TRACE_EVENT(region_fira_session_params,
	TP_PROTO(const struct fira_session *session,
		 const struct fira_session_params *params),
	TP_ARGS(session, params),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum fira_device_type, device_type)
		__field(enum fira_ranging_round_usage, ranging_round_usage)
		__field(enum fira_multi_node_mode, multi_node_mode)
		__field(__le16, controller_short_addr)
		__field(int, initiation_time_ms)
		__field(int, slot_duration_dtu)
		__field(int, block_duration_dtu)
		__field(u32, block_stride_len)
		__field(u32, max_number_of_measurements)
		__field(u32, max_rr_retry)
		__field(int, round_duration_slots)
		__field(bool, round_hopping)
		__field(u8, priority)
		__field(int, channel_number)
		__field(int, preamble_code_index)
		__field(enum fira_rframe_config, rframe_config)
		__field(enum fira_preambule_duration, preamble_duration)
		__field(enum fira_sfd_id, sfd_id)
		__field(enum fira_sts_segments, number_of_sts_segments)
		__field(enum fira_psdu_data_rate, psdu_data_rate)
		__field(enum fira_mac_fcs_type, mac_fcs_type)
		__field(enum fira_sts_mode, sts_config)
		__array(u8, vupper64, FIRA_VUPPER64_SIZE)
		__field(u32, session_key_len)
		__array(u8, session_key, FIRA_KEY_SIZE_MIN)
		__field(bool, key_rotation)
		__field(int, key_rotation_rate)
		__field(bool, aoa_result_req)
		__field(bool, report_tof)
		__field(bool, report_aoa_azimuth)
		__field(bool, report_aoa_elevation)
		__field(bool, report_aoa_fom)
		__field(bool, report_diagnostics)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->device_type = params->device_type;
		__entry->ranging_round_usage = params->ranging_round_usage;
		__entry->multi_node_mode = params->multi_node_mode;
		__entry->controller_short_addr = params->controller_short_addr;
		__entry->initiation_time_ms = params->initiation_time_ms;
		__entry->slot_duration_dtu = params->slot_duration_dtu;
		__entry->block_duration_dtu = params->block_duration_dtu;
		__entry->block_stride_len = params->block_stride_len;
		__entry->max_number_of_measurements = params->max_number_of_measurements;
		__entry->max_rr_retry = params->max_rr_retry;
		__entry->round_duration_slots = params->round_duration_slots;
		__entry->round_hopping = params->round_hopping;
		__entry->priority = params->priority;
		__entry->channel_number = params->channel_number;
		__entry->preamble_code_index = params->preamble_code_index;
		__entry->rframe_config = params->rframe_config;
		__entry->preamble_duration = params->preamble_duration;
		__entry->sfd_id = params->sfd_id;
		__entry->number_of_sts_segments = params->number_of_sts_segments;
		__entry->psdu_data_rate = params->psdu_data_rate;
		__entry->mac_fcs_type = params->mac_fcs_type;
		__entry->sts_config = params->sts_config;
		memcpy(__entry->vupper64, params->vupper64, sizeof(params->vupper64));
		__entry->session_key_len = params->session_key_len;
		memcpy(__entry->session_key, params->session_key, params->session_key_len);
		__entry->key_rotation = params->key_rotation;
		__entry->key_rotation_rate = params->key_rotation_rate;
		__entry->aoa_result_req = params->aoa_result_req;
		__entry->report_tof = params->report_tof;
		__entry->report_aoa_azimuth = params->report_aoa_azimuth;
		__entry->report_aoa_elevation = params->report_aoa_elevation;
		__entry->report_aoa_fom = params->report_aoa_fom;
		__entry->report_diagnostics = params->report_diagnostics;
		),
	TP_printk(FIRA_SESSION_PR_FMT " device_type=%s ranging_round_usage=%s multi_node_mode=%s "
		  "controller_short_addr=0x%x initiation_time_ms=%d slot_duration_dtu=%d "
		  "block_duration_dtu=%d block_stride_len=%d max_nb_of_measurements=%d "
		  "max_rr_retry=%d round_duration_slots=%d round_hopping=%d "
		  "priority=%d channel_number=%d preamble_code_index=%d rframe_config=%s "
		  "preamble_duration=%s sfd_id=%d number_of_sts_segments=%s psdu_data_rate=%s mac_fcs_type=%s "
		  "sts_config=%s vupper64=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x session_key_len=%d "
		  "session_key=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x "
		  "key_rotation=%d key_rotation_rate=%d aoa_result_req=%d report_tof=%d report_aoa_azimuth=%d "
		  "report_aoa_elevation=%d report_aoa_fom=%d diagnostics=%d",
		  FIRA_SESSION_PR_ARG,
		  __print_symbolic(__entry->device_type, FIRA_DEVICE_TYPE_SYMBOLS),
		  __print_symbolic(__entry->ranging_round_usage, FIRA_RANGING_ROUND_SYMBOLS),
		  __print_symbolic(__entry->multi_node_mode, FIRA_MULTI_NODE_MODE_SYMBOLS),
		  __entry->controller_short_addr,
		  __entry->initiation_time_ms,
		  __entry->slot_duration_dtu,
		  __entry->block_duration_dtu,
		  __entry->block_stride_len,
		  __entry->max_number_of_measurements,
		  __entry->max_rr_retry,
		  __entry->round_duration_slots,
		  __entry->round_hopping,
		  __entry->priority,
		  __entry->channel_number,
		  __entry->preamble_code_index,
		  __print_symbolic(__entry->rframe_config, FIRA_RFRAME_CONFIG_SYMBOLS),
		  __print_symbolic(__entry->preamble_duration, FIRA_PREAMBULE_DURATION_SYMBOLS),
		  __entry->sfd_id,
		  __print_symbolic(__entry->number_of_sts_segments, FIRA_STS_SEGMENTS_SYMBOLS),
		  __print_symbolic(__entry->psdu_data_rate, FIRA_PSDU_DATA_RATE_SYMBOLS),
		  __print_symbolic(__entry->mac_fcs_type, FIRA_MAC_FCS_TYPE_CRC_SYMBOLS),
		  __print_symbolic(__entry->sts_config, FIRA_STS_MODE_SYMBOLS),
		  __entry->vupper64[0], __entry->vupper64[1], __entry->vupper64[2], __entry->vupper64[3],
		  __entry->vupper64[4], __entry->vupper64[5], __entry->vupper64[6], __entry->vupper64[7],
		  __entry->session_key_len,
		  __entry->session_key[0], __entry->session_key[1], __entry->session_key[2], __entry->session_key[3],
		  __entry->session_key[4], __entry->session_key[5], __entry->session_key[6], __entry->session_key[7],
		  __entry->session_key[8], __entry->session_key[9], __entry->session_key[10], __entry->session_key[11],
		  __entry->session_key[12], __entry->session_key[13], __entry->session_key[14], __entry->session_key[15],
		  __entry->key_rotation,
		  __entry->key_rotation_rate,
		  __entry->aoa_result_req,
		  __entry->report_tof,
		  __entry->report_aoa_azimuth,
		  __entry->report_aoa_elevation,
		  __entry->report_aoa_fom,
		  __entry->report_diagnostics
		)
	);

TRACE_EVENT(region_fira_session_meas_seq_params,
	TP_PROTO(const struct fira_session *session,
		 const struct fira_measurement_sequence_step *step,
		 int index),
	TP_ARGS(session, step, index),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(int, index)
		__field(enum fira_measurement_type, type)
		__field(u8, n_measurements)
		__field(s8, rx_ant_set_nonranging)
		__field(s8, rx_ant_sets_ranging_0)
		__field(s8, rx_ant_sets_ranging_1)
		__field(s8, tx_ant_set_nonranging)
		__field(s8, tx_ant_set_ranging)
	),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->index = index;
		__entry->type = step->type;
		__entry->n_measurements = step->n_measurements;
		__entry->rx_ant_set_nonranging = step->rx_ant_set_nonranging;
		__entry->rx_ant_sets_ranging_0 = step->rx_ant_sets_ranging[0];
		__entry->rx_ant_sets_ranging_1 = step->rx_ant_sets_ranging[1];
		__entry->tx_ant_set_nonranging = step->tx_ant_set_nonranging;
		__entry->tx_ant_set_ranging = step->tx_ant_set_ranging;
		),
	TP_printk(FIRA_SESSION_PR_FMT " index=%d type=%s n_measurements=%d "
		  "rx_ant_set_nonranging=%d rx_ant_sets_ranging_0=%d "
		  "rx_ant_sets_ranging_1=%d tx_ant_set_nonranging=%d "
		  "tx_ant_set_ranging=%d",
		  FIRA_SESSION_PR_ARG,
		__entry->index,
		__print_symbolic(__entry->type, FIRA_MEAS_SEQ_STEP_TYPE),
		__entry->n_measurements,
		__entry->rx_ant_set_nonranging,
		__entry->rx_ant_sets_ranging_0,
		__entry->rx_ant_sets_ranging_1,
		__entry->tx_ant_set_nonranging,
		__entry->tx_ant_set_ranging)
	);

TRACE_EVENT(region_fira_session_control,
	TP_PROTO(const struct fira_local *local,
		 int session_id, enum fira_call call_id),
	TP_ARGS(local, session_id, call_id),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum fira_call, call_id)
		),
	TP_fast_assign(
		__entry->session_id = session_id;
		__entry->call_id = call_id;
		),
	TP_printk(FIRA_SESSION_PR_FMT " call_id=%s",
		FIRA_SESSION_PR_ARG,
		__print_symbolic(__entry->call_id, FIRA_CALL_SYMBOLS)
		)
	);

TRACE_EVENT(
	region_fira_session_fsm_active_get_demand_return,
	TP_PROTO(const struct fira_local *local,
		 const struct fira_session *session,
		 const struct fira_session_demand *fsd),
	TP_ARGS(local, session, fsd),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(u32, block_start_dtu)
		__field(u32, timestamp_dtu)
		__field(int, max_duration_dtu)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->block_start_dtu = fsd->block_start_dtu;
		__entry->timestamp_dtu = fsd->timestamp_dtu;
		__entry->max_duration_dtu = fsd->max_duration_dtu;
		),
	TP_printk(FIRA_SESSION_PR_FMT " block_start_dtu=%#x "
		  "timestamp_dtu=%#x max_duration_dtu=%d",
		  FIRA_SESSION_PR_ARG,
		  __entry->block_start_dtu,
		  __entry->timestamp_dtu,
		  __entry->max_duration_dtu
		)
	);

TRACE_EVENT(
	region_fira_get_access_controller,
	TP_PROTO(const struct fira_local *local,
		 const struct fira_session *session,
		 const struct fira_session_demand *fsd),
	TP_ARGS(local, session, fsd),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(int, block_index)
		__field(int, add_blocks)
		__field(int, round_index)
		__field(int, block_stride_len)
		__field(u32, block_start_dtu)
		__field(u32, timestamp_dtu)
		__field(int, max_duration_dtu)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->block_index = session->block_index;
		__entry->add_blocks = fsd->add_blocks;
		__entry->round_index = fsd->round_index;
		__entry->block_stride_len = session->block_stride_len;
		__entry->block_start_dtu = fsd->block_start_dtu;
		__entry->timestamp_dtu = fsd->timestamp_dtu;
		__entry->max_duration_dtu = fsd->max_duration_dtu;
		),
	TP_printk(FIRA_SESSION_PR_FMT " block_index=%d add_blocks=%d "
		  "round_index=%d block_stride_len=%d block_start_dtu=%#x "
		  "timestamp_dtu=%#x max_duration_dtu=%d",
		  FIRA_SESSION_PR_ARG,
		  __entry->block_index,
		  __entry->add_blocks,
		  __entry->round_index,
		  __entry->block_stride_len,
		  __entry->block_start_dtu,
		  __entry->timestamp_dtu,
		  __entry->max_duration_dtu
		)
	);

TRACE_EVENT(
	region_fira_get_access_controlee,
	TP_PROTO(const struct fira_local *local,
		 const struct fira_session *session,
		 const struct fira_session_demand *fsd),
	TP_ARGS(local, session, fsd),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(int, block_index)
		__field(int, add_blocks)
		__field(int, round_index)
		__field(u32, block_start_dtu)
		__field(u32, timestamp_dtu)
		__field(int, max_duration_dtu)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->block_index = session->block_index;
		__entry->add_blocks = fsd->add_blocks;
		__entry->round_index = fsd->round_index;
		__entry->block_start_dtu = fsd->block_start_dtu;
		__entry->timestamp_dtu = fsd->timestamp_dtu;
		__entry->max_duration_dtu = fsd->max_duration_dtu;
		),
	TP_printk(FIRA_SESSION_PR_FMT " block_index=%d add_blocks=%d "
		  "round_index=%d block_start_dtu=%#x timestamp_dtu=%#x "
		  "max_duration_dtu=%d",
		  FIRA_SESSION_PR_ARG,
		  __entry->block_index,
		  __entry->add_blocks,
		  __entry->round_index,
		  __entry->block_start_dtu,
		  __entry->timestamp_dtu,
		  __entry->max_duration_dtu
		)
	);

TRACE_EVENT(region_fira_rx_frame,
	TP_PROTO(const struct fira_session *session,
		 enum fira_message_id message_id,
		 enum mcps802154_rx_error_type error),
	TP_ARGS(session, message_id, error),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum fira_message_id, message_id)
		__field(enum mcps802154_rx_error_type, error)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->message_id = message_id;
		__entry->error = error;
		),
	TP_printk(FIRA_SESSION_PR_FMT " message_id=%s error=%s",
		FIRA_SESSION_PR_ARG,
		__print_symbolic(__entry->message_id, FIRA_MESSAGE_TYPE),
		__print_symbolic(__entry->error, MCPS802154_RX_ERROR_SYMBOLS)
		)
	);

TRACE_EVENT(region_fira_rx_frame_control,
	TP_PROTO(const struct fira_local *local,
		 const struct fira_session *session,
		 int left_duration_dtu, int n_slots),
	TP_ARGS(local, session, left_duration_dtu, n_slots),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(u32, block_start_dtu)
		__field(int, block_index)
		__field(int, round_index)
		__field(bool, stop_inband)
		__field(int, left_duration_dtu)
		__field(int, n_slots)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->block_start_dtu = session->block_start_dtu;
		__entry->block_index = session->block_index;
		__entry->round_index = session->round_index;
		__entry->stop_inband = session->stop_inband;
		__entry->left_duration_dtu = left_duration_dtu;
		__entry->n_slots = n_slots;
		),
	TP_printk(FIRA_SESSION_PR_FMT " block_start_dtu=%#x block_index=%d "
		  "round_index=%d stop_inband=%s left_duration_dtu=%d n_slots=%d",
		FIRA_SESSION_PR_ARG,
		__entry->block_start_dtu,
		__entry->block_index,
		__entry->round_index,
		__entry->stop_inband ? "true": "false",
		__entry->left_duration_dtu,
		__entry->n_slots
		)
	);

TRACE_EVENT(region_fira_tx_get_frame,
	TP_PROTO(const struct fira_session *session,
		 enum fira_message_id message_id),
	TP_ARGS(session, message_id),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum fira_message_id, message_id)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->message_id = message_id;
		),
	TP_printk(FIRA_SESSION_PR_FMT " message_id=%s",
		FIRA_SESSION_PR_ARG,
		__print_symbolic(__entry->message_id, FIRA_MESSAGE_TYPE)
		)
	);

TRACE_EVENT(region_fira_tx_return,
	TP_PROTO(const struct fira_session *session,
		 enum mcps802154_access_tx_return_reason reason),
	TP_ARGS(session, reason),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum mcps802154_access_tx_return_reason, reason)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->reason = reason;
		),
	TP_printk(FIRA_SESSION_PR_FMT " reason=%s",
		FIRA_SESSION_PR_ARG,
		  __print_symbolic(__entry->reason,
				   MCPS802154_TX_REASON_SYMBOLS)
		)
	);

TRACE_EVENT(
	region_fira_session_fsm_change_state,
	TP_PROTO(const struct fira_session *session, enum fira_session_state_id new_state_id),
	TP_ARGS(session, new_state_id),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum fira_session_state_id, new_state_id)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->new_state_id = new_state_id;
		),
	TP_printk(FIRA_SESSION_PR_FMT " new_state_id=%s",
		  FIRA_SESSION_PR_ARG,
		  __print_symbolic(__entry->new_state_id,
				   FIRA_SESSION_STATE_ID_SYMBOLS)
		)
	);

TRACE_EVENT(
	region_fira_access_done,
	TP_PROTO(const struct fira_local *local,
		 const struct fira_session *session,
		 int access_duration_dtu, bool error),
	TP_ARGS(local, session, access_duration_dtu, error),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(int, access_duration_dtu)
		__field(bool, error)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->access_duration_dtu = access_duration_dtu;
		__entry->error = error;
		),
	TP_printk(FIRA_SESSION_PR_FMT " access_duration_dtu=%d error=%s",
		  FIRA_SESSION_PR_ARG,
		  __entry->access_duration_dtu,
		  __entry->error ? "true": "false"
		)
	);

TRACE_EVENT(
	region_fira_is_controlee_synchronised,
	TP_PROTO(const struct fira_session *session,
		 int drift_ppm, int rx_margin_ppm),
	TP_ARGS(session, drift_ppm, rx_margin_ppm),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(int, block_index_sync)
		__field(int, drift_ppm)
		__field(int, rx_margin_ppm)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->block_index_sync = session->controlee.block_index_sync;
		__entry->drift_ppm = drift_ppm;
		__entry->rx_margin_ppm = rx_margin_ppm;
		),
	TP_printk(FIRA_SESSION_PR_FMT " block_index_sync=%d drift_ppm=%d rx_margin_ppm=%d",
		  FIRA_SESSION_PR_ARG,
		  __entry->block_index_sync,
		  __entry->drift_ppm,
		  __entry->rx_margin_ppm
		)
	);

TRACE_EVENT(
	region_fira_session_report,
	TP_PROTO(const struct fira_session *session,
		 const struct fira_report_info *report_info),
	TP_ARGS(session, report_info),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(int, sequence_number)
		__field(int, block_index)
		__field(int, n_ranging_data)
		__field(int, n_stopped_controlees)
		__field(int, n_slots)
		__field(bool, stopped)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->sequence_number = session->sequence_number;
		__entry->block_index = session->block_index;
		__entry->n_ranging_data = report_info->n_ranging_data;
		__entry->n_stopped_controlees = report_info->n_stopped_controlees;
		__entry->n_slots = report_info->n_slots;
		__entry->stopped = report_info->stopped;
		),
	TP_printk(FIRA_SESSION_PR_FMT " sequence_number=%d block_index=%d "
		  "n_ranging_data=%d n_stopped_controlees=%d n_slots=%d stopped=%s",
		  FIRA_SESSION_PR_ARG,
		  __entry->sequence_number,
		  __entry->block_index,
		  __entry->n_ranging_data,
		  __entry->n_stopped_controlees,
		  __entry->n_slots,
		  __entry->stopped ? "true": "false"
		)
	);

TRACE_EVENT(fira_nondeferred_not_supported,
	TP_PROTO(const struct fira_session *session),
	TP_ARGS(session),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		),
	TP_printk(FIRA_SESSION_PR_FMT,
		  FIRA_SESSION_PR_ARG
		)
	);

#endif /* !FIRA_TRACE_H || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE fira_trace
#include <trace/define_trace.h>
