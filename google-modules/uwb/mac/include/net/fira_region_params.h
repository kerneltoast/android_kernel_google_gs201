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

#ifndef NET_FIRA_REGION_PARAMS_H
#define NET_FIRA_REGION_PARAMS_H

#include <linux/types.h>

#define FIRA_VUPPER64_SIZE 8
#define FIRA_STS_VUPPER64_OFFSET 8
#define FIRA_KEY_SIZE_MAX 16
#define FIRA_KEY_SIZE_MIN 16
#define FIRA_CONTROLEES_MAX 8
#define FIRA_RX_ANTENNA_PAIR_INVALID 0xff
/*
 * In BPRF, frame is at most 127
 * 127 - (MHR + HIE + HT + PIE_Header + V_OUI + MIC + CRC)
 */
#define FIRA_DATA_PAYLOAD_SIZE_MAX 84

/* From UCI spec v1.1.0 (converted to mm).
 * Need a 2mm margin to avoid errors when converting to and from RCTU */
#define FIRA_RANGE_DATA_NTF_PROXIMITY_FAR_MM_DEFAULT 200002

/*
 * FIRA_SESSION_DATA_NTF_LOWER_/UPPER_BOUND_AOA default values :
 * Azimuth in rad_2pi_q16 : -32768 / 32767 (equal to -180 / ~180 degrees)
 * Elevation in rad_2pi_q16 : -16384 / 16384 (equal to -90 / 90 degrees)
 */
#define FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI_DEFAULT -32768
#define FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI_DEFAULT 32767
#define FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI_DEFAULT -16384
#define FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI_DEFAULT 16384

/**
 * enum fira_device_type - Type of a device.
 * @FIRA_DEVICE_TYPE_CONTROLEE: The device is a controlee.
 * @FIRA_DEVICE_TYPE_CONTROLLER: The device is a controller.
 */
enum fira_device_type {
	FIRA_DEVICE_TYPE_CONTROLEE,
	FIRA_DEVICE_TYPE_CONTROLLER,
};

/**
 * enum fira_device_role - **[NOT IMPLEMENTED]**  Role played by a device.
 * @FIRA_DEVICE_ROLE_RESPONDER: The device acts as a responder.
 * @FIRA_DEVICE_ROLE_INITIATOR: The device acts as an initiator.
 *
 * Current implementation does not support decorrelation between the
 * device's role and the device's type. The controller is always
 * the initiator and the controlee is always the responder.
 *
 * This enum is not used in the current implementation.
 */
enum fira_device_role {
	FIRA_DEVICE_ROLE_RESPONDER,
	FIRA_DEVICE_ROLE_INITIATOR,
};

/**
 * enum fira_ranging_round_usage - Ranging mode.
 * @FIRA_RANGING_ROUND_USAGE_OWR: One Way Ranging mode (unused, not in FiRa 1.1).
 * @FIRA_RANGING_ROUND_USAGE_SSTWR: Single-Sided Two Way Ranging mode.
 * @FIRA_RANGING_ROUND_USAGE_DSTWR: Dual-Sided Two Way Ranging mode.
 */
enum fira_ranging_round_usage {
	FIRA_RANGING_ROUND_USAGE_OWR,
	FIRA_RANGING_ROUND_USAGE_SSTWR,
	FIRA_RANGING_ROUND_USAGE_DSTWR,
};

/**
 * enum fira_multi_node_mode - Multi-node mode.
 * @FIRA_MULTI_NODE_MODE_UNICAST: Ranging between one initiator and one
 * responder.
 * @FIRA_MULTI_NODE_MODE_ONE_TO_MANY: Ranging between one initiator and
 * multiple responders.
 * @FIRA_MULTI_NODE_MODE_MANY_TO_MANY: Ranging between multiple initiators
 * and multiple responders.
 */
enum fira_multi_node_mode {
	FIRA_MULTI_NODE_MODE_UNICAST,
	FIRA_MULTI_NODE_MODE_ONE_TO_MANY,
	FIRA_MULTI_NODE_MODE_MANY_TO_MANY,
};

/**
 * enum fira_measurement_report - **[NOT IMPLEMENTED]**  Transmission of a
 * Ranging Measurement Report Message (MRM) option.
 * @FIRA_MEASUREMENT_REPORT_AT_RESPONDER: The initiator emits a MRM.
 * @FIRA_MEASUREMENT_REPORT_AT_INITIATOR: The responder emits a MRM.
 *
 * In the current implementation measurement report is always available at
 * responder.
 *
 * This enum is not used in the current implementation.
 */
enum fira_measurement_report {
	FIRA_MEASUREMENT_REPORT_AT_RESPONDER,
	FIRA_MEASUREMENT_REPORT_AT_INITIATOR,
};

/**
 * enum fira_embedded_mode - **[NOT IMPLEMENTED]** Message embedding
 * behaviour.
 * @FIRA_EMBEDDED_MODE_DEFERRED: Ranging messages do not embed control messages.
 * Additional messages are required.
 * @FIRA_EMBEDDED_MODE_NON_DEFERRED: Ranging messages embed control messages.
 *
 * The current implementation only supports deferred mode.
 *
 * This enum is not used in the current implementation.
 */
enum fira_embedded_mode {
	FIRA_EMBEDDED_MODE_DEFERRED,
	FIRA_EMBEDDED_MODE_NON_DEFERRED,
};

/**
 * enum fira_rframe_config - Rframe configuration used to transmit/receive
 * ranging messages.
 * @FIRA_RFRAME_CONFIG_SP0: Use SP0 mode.
 * @FIRA_RFRAME_CONFIG_SP1: Use SP1 mode.
 * @FIRA_RFRAME_CONFIG_SP2: RFU
 * @FIRA_RFRAME_CONFIG_SP3: Use SP3 mode.
 */
enum fira_rframe_config {
	FIRA_RFRAME_CONFIG_SP0,
	FIRA_RFRAME_CONFIG_SP1,
	FIRA_RFRAME_CONFIG_SP2,
	FIRA_RFRAME_CONFIG_SP3,
};

/**
 * enum fira_prf_mode - Pulse Repetition Frequency mode
 * @FIRA_PRF_MODE_BPRF: Base Pulse Repetition Frequency.
 * @FIRA_PRF_MODE_HPRF: Higher Pulse Repetition Frequency.
 * @FIRA_PRF_MODE_HPRF_HIGH_RATE: Higher Pulse Repetition Frequency allows
 * high data rate (27.2 Mbps and 31.2 Mbps).
 *
 * This enum is not used in the current implementation.
 */
enum fira_prf_mode {
	FIRA_PRF_MODE_BPRF,
	FIRA_PRF_MODE_HPRF,
	FIRA_PRF_MODE_HPRF_HIGH_RATE,
};

/**
 * enum fira_preambule_duration - Duration of preamble in symbols.
 * @FIRA_PREAMBULE_DURATION_32: 32 symbols duration.
 * @FIRA_PREAMBULE_DURATION_64: 64 symbols duration.
 */
enum fira_preambule_duration {
	FIRA_PREAMBULE_DURATION_32,
	FIRA_PREAMBULE_DURATION_64,
};

/**
 * enum fira_sfd_id - Start-of-frame delimiter.
 * @FIRA_SFD_ID_0: Delimiter is [0 +1 0 –1 +1 0 0 –1]
 * @FIRA_SFD_ID_1: Delimiter is [ –1 –1 +1 –1 ]
 * @FIRA_SFD_ID_2: Delimiter is [ –1 –1 –1 +1 –1 –1 +1 –1 ]
 * @FIRA_SFD_ID_3: Delimiter is
 * [ –1 –1 –1 –1 –1 +1 +1 –1 –1 +1 –1 +1 –1 –1 +1 –1 ]
 * @FIRA_SFD_ID_4: Delimiter is
 * [ –1 –1 –1 –1 –1 –1 –1 +1 –1 –1 +1 –1 –1 +1 –1 +1 –1 +1
 * –1 –1 –1 +1 +1 –1 –1 –1 +1 –1 +1 +1 –1 –1 ]
 */
enum fira_sfd_id {
	FIRA_SFD_ID_0,
	FIRA_SFD_ID_1,
	FIRA_SFD_ID_2,
	FIRA_SFD_ID_3,
	FIRA_SFD_ID_4,
};

/**
 * enum fira_sts_segments - Number of STS segments.
 * @FIRA_STS_SEGMENTS_0: No STS Segment (Rframe config SP0).
 * @FIRA_STS_SEGMENTS_1: 1 STS Segment.
 * @FIRA_STS_SEGMENTS_2: 2 STS Segments.
 * @FIRA_STS_SEGMENTS_3: 3 STS Segments.
 * @FIRA_STS_SEGMENTS_4: 4 STS Segments.
 */
enum fira_sts_segments {
	FIRA_STS_SEGMENTS_0,
	FIRA_STS_SEGMENTS_1,
	FIRA_STS_SEGMENTS_2,
	FIRA_STS_SEGMENTS_3,
	FIRA_STS_SEGMENTS_4,
};

/**
 * enum fira_psdu_data_rate - Data rate used to exchange PSDUs.
 * @FIRA_PSDU_DATA_RATE_6M81: 6.8Mb/s rate.
 * @FIRA_PSDU_DATA_RATE_7M80: 7.8Mb/s rate.
 * @FIRA_PSDU_DATA_RATE_27M2: 27.2Mb/s rate.
 * @FIRA_PSDU_DATA_RATE_31M2: 31.2Mb/s rate.
 */
enum fira_psdu_data_rate {
	FIRA_PSDU_DATA_RATE_6M81,
	FIRA_PSDU_DATA_RATE_7M80,
	FIRA_PSDU_DATA_RATE_27M2,
	FIRA_PSDU_DATA_RATE_31M2,
};

/**
 * enum fira_phr_data_rate - Data rate used to exchange PHR.
 * @FIRA_PHR_DATA_RATE_850K: 850kb/s rate.
 * @FIRA_PHR_DATA_RATE_6M81: 6.8Mb/s rate.
 *
 * This enum is not used in the current implementation.
 */
enum fira_phr_data_rate {
	FIRA_PHR_DATA_RATE_850K,
	FIRA_PHR_DATA_RATE_6M81,
};

/**
 * enum fira_mac_fcs_type - Length of Frame Check Sequence.
 * @FIRA_MAC_FCS_TYPE_CRC_16: 2 bytes sequence.
 * @FIRA_MAC_FCS_TYPE_CRC_32: 4 bytes sequence.
 */
enum fira_mac_fcs_type {
	FIRA_MAC_FCS_TYPE_CRC_16,
	FIRA_MAC_FCS_TYPE_CRC_32,
};

/**
 * enum fira_rssi_report_type - Mode used to sum up individual frames RSSI
 * in report.
 * @FIRA_RSSI_REPORT_NONE: No RSSI value in report.
 * @FIRA_RSSI_REPORT_MINIMUM: Report minimum RSSI
 * @FIRA_RSSI_REPORT_AVERAGE: Report average RSSI
 */
enum fira_rssi_report_type {
	FIRA_RSSI_REPORT_NONE,
	FIRA_RSSI_REPORT_MINIMUM,
	FIRA_RSSI_REPORT_AVERAGE,
};

/**
 * enum fira_sts_mode - Scrambled Timestamp Sequence modes.
 *
 * @FIRA_STS_MODE_STATIC: Static STS mode.
 * @FIRA_STS_MODE_DYNAMIC: Use a dynamic STS mode.
 * @FIRA_STS_MODE_DYNAMIC_INDIVIDUAL_KEY: Use a dynamic STS mode
 * with individual controlee key.
 * @FIRA_STS_MODE_PROVISIONED: Use a provisioned STS mode.
 * @FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY: Use a provisioned STS
 * mode with individual controlee key.
 */
enum fira_sts_mode {
	FIRA_STS_MODE_STATIC = 0,
	FIRA_STS_MODE_DYNAMIC = 1,
	FIRA_STS_MODE_DYNAMIC_INDIVIDUAL_KEY = 2,
	FIRA_STS_MODE_PROVISIONED = 3,
	FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY = 4,
};

/*
 * Get the capabilities bitfield value corresponding to given STS mode.
 */
#define STS_CAP(mode) (1 << (FIRA_STS_MODE_##mode))

/**
 * enum fira_ranging_status - The ranging status.
 * @FIRA_STATUS_RANGING_INTERNAL_ERROR: Implementation specific error.
 * @FIRA_STATUS_RANGING_SUCCESS: Ranging info are valid.
 * @FIRA_STATUS_RANGING_TX_FAILED: Failed to transmit UWB packet.
 * @FIRA_STATUS_RANGING_RX_TIMEOUT: No UWB packet detected by the receiver.
 * @FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED: UWB packet channel decoding error.
 * @FIRA_STATUS_RANGING_RX_PHY_TOA_FAILED: Failed to detect time of arrival of
 * the UWB packet from CIR samples.
 * @FIRA_STATUS_RANGING_RX_PHY_STS_FAILED: UWB packet STS segment mismatch.
 * @FIRA_STATUS_RANGING_RX_MAC_DEC_FAILED: MAC CRC or syntax error.
 * @FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED: IE syntax error.
 * @FIRA_STATUS_RANGING_RX_MAC_IE_MISSING: Expected IE missing in the packet.
 */
enum fira_ranging_status {
	FIRA_STATUS_RANGING_INTERNAL_ERROR = -1,
	FIRA_STATUS_RANGING_SUCCESS = 0,
	FIRA_STATUS_RANGING_TX_FAILED = 1,
	FIRA_STATUS_RANGING_RX_TIMEOUT = 2,
	FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED = 3,
	FIRA_STATUS_RANGING_RX_PHY_TOA_FAILED = 4,
	FIRA_STATUS_RANGING_RX_PHY_STS_FAILED = 5,
	FIRA_STATUS_RANGING_RX_MAC_DEC_FAILED = 6,
	FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED = 7,
	FIRA_STATUS_RANGING_RX_MAC_IE_MISSING = 8,
};

/**
 * enum fira_session_state - Session state.
 * @FIRA_SESSION_STATE_INIT: Initial state, session is not ready yet.
 * @FIRA_SESSION_STATE_DEINIT: Session does not exist.
 * @FIRA_SESSION_STATE_ACTIVE: Session is currently active.
 * @FIRA_SESSION_STATE_IDLE: Session is ready to start, but not currently
 * active.
 */
enum fira_session_state {
	FIRA_SESSION_STATE_INIT,
	FIRA_SESSION_STATE_DEINIT,
	FIRA_SESSION_STATE_ACTIVE,
	FIRA_SESSION_STATE_IDLE,
};

/*
 * The maximum number of steps a measurement sequence can contain.
 */
#define FIRA_MEASUREMENT_SEQUENCE_STEP_MAX 10

/**
 * enum fira_measurement_type - The different type of available measurements.
 * @FIRA_MEASUREMENT_TYPE_RANGE: Measure only range.
 * @FIRA_MEASUREMENT_TYPE_AOA: Measure range + unspecified AoA.
 * @FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH: Measure range + azimuth.
 * @FIRA_MEASUREMENT_TYPE_AOA_ELEVATION: Measure range + elevation.
 * @FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH_ELEVATION: Measure range+azimuth+elevation.
 * @__FIRA_MEASUREMENT_TYPE_AFTER_LAST: Internal use.
 */
enum fira_measurement_type {
	FIRA_MEASUREMENT_TYPE_RANGE = 0,
	FIRA_MEASUREMENT_TYPE_AOA,
	FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH,
	FIRA_MEASUREMENT_TYPE_AOA_ELEVATION,
	FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH_ELEVATION,
	__FIRA_MEASUREMENT_TYPE_AFTER_LAST,
};

/**
 * enum fira_ranging_diagnostics_frame_report_flags - Activation flags for different frame diagnostics information.
 * @FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_NONE: No specific frame diagnostic report requested.
 * @FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_RSSIS: Report RSSI in frame diagnostics.
 * @FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_AOAS: Report AOA in frame diagnostics.
 * @FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_CIRS: Report CIR in frame diagnostics.
 * @__FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_AFTER_LAST: Internal use.
 */
enum fira_ranging_diagnostics_frame_report_flags {
	FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_NONE = 0,
	FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_RSSIS = 1 << 0,
	FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_AOAS = 1 << 1,
	FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_CIRS = 1 << 2,
	__FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_AFTER_LAST = 1 << 31,
};

/**
 * enum fira_sts_length - Number of symbols in a STS segment.
 * @FIRA_STS_LENGTH_32: The STS length is 32 symbols.
 * @FIRA_STS_LENGTH_64: The STS length is 64 symbols.
 * @FIRA_STS_LENGTH_128: The STS length is 128 symbols.
 */
enum fira_sts_length {
	FIRA_STS_LENGTH_32 = 0,
	FIRA_STS_LENGTH_64 = 1,
	FIRA_STS_LENGTH_128 = 2,
};

/**
 * enum fira_range_data_ntf_config - Configure range data notification.
 * @FIRA_RANGE_DATA_NTF_DISABLED: Do not report range data.
 * @FIRA_RANGE_DATA_NTF_ALWAYS: Report range data.
 * @FIRA_RANGE_DATA_NTF_CONFIG_PROXIMITY: Report range data if it is within
 * proximity range defined by proximity parameters
 * defined by proximity parameters (RANGE_DATA_NTF_PROXIMITY_NEAR/FAR).
 * @FIRA_RANGE_DATA_NTF_CONFIG_AOA: Report range data in AoA upper and lower bound.
 * defined by AOA parameters (FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_UPPER/
 * LOWER_BOUND_AOA_AZIMUTH/ELEVATION)
 * @FIRA_RANGE_DATA_NTF_CONFIG_PROXIMITY_AND_AOA: Report range data in AoA upper
 * and lower bound as well as in proximity range.
 * @FIRA_RANGE_DATA_NTF_CONFIG_PROXIMITY_CROSSING: Same as
 * FIRA_RANGE_DATA_NTF_CONFIG_PROXIMITY, but issues notification on crossing of
 * boundaries. As for now, same notif is sent for "enter" and "exit" events.
 * @FIRA_RANGE_DATA_NTF_CONFIG_AOA_CROSSING:  Same as
 * FIRA_RANGE_DATA_NTF_CONFIG_AOA, but issues notification on crossing of boundaries.
 * As for now, same notif is sent for "enter" and "exit" events.
 * @FIRA_RANGE_DATA_NTF_CONFIG_PROXIMITY_AND_AOA_CROSSING:  Same as
 * FIRA_RANGE_DATA_NTF_CONFIG_PROXIMITY_AND_AOA, but issues notification on crossing of
 * As for now, same notif is sent for "enter" and "exit" events.
 * @FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA_CROSSING:  Same as
 * FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA, but issues notification on crossing of
 * boundaries. As for now, same notif is sent for "enter" and "exit" events.
 */
enum fira_range_data_ntf_config {
	FIRA_RANGE_DATA_NTF_DISABLED = 0x00,
	FIRA_RANGE_DATA_NTF_ALWAYS = 0x01,
	FIRA_RANGE_DATA_NTF_PROXIMITY = 0x02,
	FIRA_RANGE_DATA_NTF_AOA = 0x03,
	FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA = 0x04,
	FIRA_RANGE_DATA_NTF_PROXIMITY_CROSSING = 0x05,
	FIRA_RANGE_DATA_NTF_AOA_CROSSING = 0x06,
	FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA_CROSSING = 0x07,
};

#endif /* NET_FIRA_REGION_PARAMS_H */
