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

#ifndef NET_FIRA_REGION_H
#define NET_FIRA_REGION_H

#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/math64.h>
#include <net/mcps802154_schedule.h>

#include "net/fira_region_params.h"

#define FIRA_SLOT_DURATION_RSTU_DEFAULT 2400
#define FIRA_BLOCK_DURATION_MS_DEFAULT 200
#define FIRA_ROUND_DURATION_SLOTS_DEFAULT 30
#define FIRA_MAX_RR_RETRY_DEFAULT 0
#define FIRA_PRIORITY_MAX 100
#define FIRA_PRIORITY_DEFAULT 50
#define FIRA_IN_BAND_TERMINATION_ATTEMPT_COUNT_MAX 10
#define FIRA_IN_BAND_TERMINATION_ATTEMPT_COUNT_MIN 1
#define FIRA_BOOLEAN_MAX 1
#define FIRA_BLOCK_STRIDE_LEN_MAX 255
#define FIRA_FRAMES_MAX (3 + 3 * FIRA_CONTROLEES_MAX)
#define FIRA_CONTROLEE_FRAMES_MAX (3 + 3 + 1)
/* IEEE 802.15.4z 2020 section 6.9.7.2 */
#define UWB_BLOCK_DURATION_MARGIN_PPM 100
/* FiRa Tx should arrive between 0 and 10 us, always add 2 us. */
#define FIRA_TX_MARGIN_US 2

/*
 * FIRA_SESSION_DATA_NTF_LOWER_/UPPER_BOUND_AOA min/max :
 * Azimuth in rad_2pi_q16 : -32768 / 32767 (equal to -180 / ~180 degrees)
 * Elevation in rad_2pi_q16 : -16384 / 16384 (equal to -90 / 90 degrees)
 */
#define FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI_MIN -32768
#define FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI_MAX 32767
#define FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI_MIN -32768
#define FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI_MAX 32767
#define FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI_MIN -16384
#define FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI_MAX 16384
#define FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI_MIN -16384
#define FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI_MAX 16384

/**
 * enum fira_message_id - Message identifiers, used in internal state and in
 * messages.
 * @FIRA_MESSAGE_ID_RANGING_INITIATION: Initial ranging message.
 * @FIRA_MESSAGE_ID_RANGING_RESPONSE: Response ranging message.
 * @FIRA_MESSAGE_ID_RANGING_FINAL: Final ranging message, only for DS-TWR.
 * @FIRA_MESSAGE_ID_CONTROL: Control message, sent by the controller.
 * @FIRA_MESSAGE_ID_MEASUREMENT_REPORT: Deferred report of ranging measures.
 * @FIRA_MESSAGE_ID_RESULT_REPORT: Report computed ranging result.
 * @FIRA_MESSAGE_ID_CONTROL_UPDATE: Message to change hopping.
 * @FIRA_MESSAGE_ID_RFRAME_MAX: Maximum identifier of message transmitted using
 * an RFRAME.
 * @FIRA_MESSAGE_ID_MAX: Maximum message identifier.
 */
enum fira_message_id {
	FIRA_MESSAGE_ID_RANGING_INITIATION = 0,
	FIRA_MESSAGE_ID_RANGING_RESPONSE = 1,
	FIRA_MESSAGE_ID_RANGING_FINAL = 2,
	FIRA_MESSAGE_ID_CONTROL = 3,
	FIRA_MESSAGE_ID_MEASUREMENT_REPORT = 4,
	FIRA_MESSAGE_ID_RESULT_REPORT = 5,
	FIRA_MESSAGE_ID_CONTROL_UPDATE = 6,
	FIRA_MESSAGE_ID_RFRAME_MAX = FIRA_MESSAGE_ID_RANGING_FINAL,
	FIRA_MESSAGE_ID_MAX = FIRA_MESSAGE_ID_CONTROL_UPDATE,
};

/**
 * struct fira_diagnostic - Diagnostic result.
 */
struct fira_diagnostic {
	/**
	 * @rssis_q1: Received signal strength indication (RSSI), absolute value
	 * in Q1 fixed point format, unit is dBm.
	 */
	u8 rssis_q1[MCPS802154_RSSIS_N_MAX];
	/**
	 * @n_rssis: The number of RSSI in the array below.
	 */
	size_t n_rssis;
	/**
	 * @aoas: Angle of arrival, ordered by increasing measurement type.
	 */
	struct mcps802154_rx_aoa_measurements
		aoas[MCPS802154_RX_AOA_MEASUREMENTS_MAX];
	/**
	 * @n_aoas: Number of angle of arrival.
	 */
	size_t n_aoas;
	/**
	 * @cirs: CIR for different parts of the frame.
	 *
	 * Set by low-level driver, must be kept valid until next received
	 * frame.
	 */
	struct mcps802154_rx_cir *cirs;
	/**
	 * @n_cirs: Number of parts of CIR.
	 */
	size_t n_cirs;
};

/**
 * struct fira_slot - Information on an active slot.
 */
struct fira_slot {
	/**
	 * @index: Index of this slot, add it to the block STS index to get the
	 * slot STS index. Note: there can be holes for a controlee as only
	 * relevant slots are recorded.
	 */
	int index;
	/**
	 * @controller_tx: True if Tx is performed by the controller.
	 */
	bool controller_tx;
	/**
	 * @ranging_index: Index of the ranging in the ranging information
	 * table, -1 if none.
	 */
	int ranging_index;
	/**
	 * @message_id: Identifier of the message exchanged in this slot.
	 */
	enum fira_message_id message_id;
	/**
	 * @tx_ant_set: Tx antenna set.
	 */
	int tx_ant_set;
	/**
	 * @rx_ant_set: Rx antenna set.
	 */
	int rx_ant_set;
	/**
	 * @controlee: Controlee.
	 */
	struct fira_controlee *controlee;
};

/**
 * struct fira_local_aoa_info - Ranging AoA information.
 */
struct fira_local_aoa_info {
	/**
	 * @pdoa_2pi: Phase Difference of Arrival.
	 */
	s16 pdoa_2pi;
	/**
	 * @aoa_2pi: Angle of Arrival.
	 */
	s16 aoa_2pi;
	/**
	 * @aoa_fom: Figure of merit of the AoA.
	 */
	u8 aoa_fom;
	/**
	 * @rx_ant_set: Antenna set index.
	 */
	u8 rx_ant_set;
	/**
	 * @present: true if AoA information is present.
	 */
	bool present;
};

/**
 * enum fira_range_data_ntf_status - Device (controller or controlee)
 * status, used for range_data_ntf.
 * @FIRA_RANGE_DATA_NTF_NONE: Undetermined, no ranging data for this
 * device yet, or N/A (not applicable).
 * @FIRA_RANGE_DATA_NTF_IN: Last ranging data for this device
 * were inside given boudaries.
 * @FIRA_RANGE_DATA_NTF_OUT: Last ranging data for this device
 * were outside given boudaries.
 * @FIRA_RANGE_DATA_NTF_ERROR: Last ranging round(s) for this device
 * failed (timeout, error, ...). No info about a previous state or N/A.
 * @FIRA_RANGE_DATA_NTF_IN_ERROR: Last ranging round(s) for this device
 * failed (timeout, error, ...). Previous data were inside given boudaries.
 * @FIRA_RANGE_DATA_NTF_OUT_ERROR: Last ranging round(s) for this device
 * failed (timeout, error, ...). Previous data were inside given boudaries.
*/
enum fira_range_data_ntf_status {
	FIRA_RANGE_DATA_NTF_NONE,
	FIRA_RANGE_DATA_NTF_IN,
	FIRA_RANGE_DATA_NTF_OUT,
	FIRA_RANGE_DATA_NTF_ERROR,
	FIRA_RANGE_DATA_NTF_IN_ERROR,
	FIRA_RANGE_DATA_NTF_OUT_ERROR,
};

/**
 * struct fira_ranging_info - Ranging information.
 */
struct fira_ranging_info {
	/**
	 * @timestamps_rctu: Timestamps of the ranging messages.
	 */
	u64 timestamps_rctu[FIRA_MESSAGE_ID_RFRAME_MAX + 1];
	/**
	 * @tof_rctu: Computed Time of Flight.
	 */
	int tof_rctu;
	/**
	* @local_aoa: Local ranging AoA information.
	*/
	struct fira_local_aoa_info local_aoa;
	/**
	 * @local_aoa_azimuth: Azimuth ranging AoA information.
	 */
	struct fira_local_aoa_info local_aoa_azimuth;
	/**
	 * @local_aoa_elevation: Elevation ranging AoA information.
	 */
	struct fira_local_aoa_info local_aoa_elevation;
	/**
	 * @remote_aoa_azimuth_2pi: Remote azimuth AoA.
	 */
	s16 remote_aoa_azimuth_2pi;
	/**
	 * @remote_aoa_elevation_pi: Remote elevation AoA.
	 */
	s16 remote_aoa_elevation_pi;
	/**
	 * @remote_aoa_azimuth_fom: Remote azimuth FoM.
	 */
	u8 remote_aoa_azimuth_fom;
	/**
	 * @remote_aoa_elevation_fom: Remote elevation FoM.
	 */
	u8 remote_aoa_elevation_fom;
	/**
	 * @rx_rssis: RSSI value measured for individual Rx frames.
	 */
	u8 rx_rssis[FIRA_MESSAGE_ID_MAX + 1];
	/**
	 * @n_rx_rssis: Number of Rx RSSI saved.
	 */
	int n_rx_rssis;
	/**
	 * @short_addr: Peer short address.
	 */
	__le16 short_addr;
	/**
	 * @status: Success or failure reason.
	 */
	enum fira_ranging_status status;
	/**
	 * @slot_index: In case of failure, the slot index where it has occured.
	 */
	u8 slot_index;
	/**
	 * @tof_present: true if time of flight information is present.
	 */
	bool tof_present;
	/**
	 * @remote_aoa_azimuth_present: true if azimuth AoA information is present.
	 */
	bool remote_aoa_azimuth_present;
	/**
	 * @remote_aoa_elevation_present: true if elevation AoA information is present.
	 */
	bool remote_aoa_elevation_present;
	/**
	 * @remote_aoa_fom_present: true if FoM AoA is present.
	 */
	bool remote_aoa_fom_present;
	/**
	 * @clock_offset_present: true if the driver provided clock_offset info.
	 */
	bool clock_offset_present;
	/**
	 * @clock_offset_q26: clock offset value, as signed Q26, if present.
	 */
	s16 clock_offset_q26;
	/**
	 * @data_payload: Custom data payload.
	 */
	u8 data_payload[FIRA_DATA_PAYLOAD_SIZE_MAX];
	/**
	 * @data_payload_len: Custom data payload length.
	 */
	int data_payload_len;
	/**
	 * @rx_ctx: Pointer to the current rx_ctx context controlee.
	 */
	void *rx_ctx;
	/**
	 * @range_data_ntf_status: range_data_ntf status of the remote device.
	 */
	enum fira_range_data_ntf_status range_data_ntf_status;
	/**
	 * @notify: if true, add this ranging to the notification report.
	 */
	bool notify;
};

/**
 * struct fira_local - Local context.
 */
struct fira_local {
	/**
	 * @region: Region instance returned to MCPS.
	 */
	struct mcps802154_region region;
	/**
	 * @llhw: Low-level device pointer.
	 */
	struct mcps802154_llhw *llhw;
	/**
	 * @access: Access returned to MCPS.
	 */
	struct mcps802154_access access;
	/**
	 * @report_queue: Queue of report frame to be processed.
	 */
	struct sk_buff_head report_queue;
	/**
	 * @report_work: Process work of report event.
	 */
	struct work_struct report_work;
	/**
	 * @frames: Access frames referenced from access.
	 */
	struct mcps802154_access_frame frames[FIRA_FRAMES_MAX];
	/**
	 * @sts_params: STS parameters for access frames.
	 */
	struct mcps802154_sts_params sts_params[FIRA_FRAMES_MAX];
	/**
	 * @channel: Channel parameters for access.
	 */
	struct mcps802154_channel channel;
	/**
	 * @inactive_sessions: List of inactive sessions.
	 */
	struct list_head inactive_sessions;
	/**
	 * @active_sessions: List of active sessions.
	 */
	struct list_head active_sessions;
	/**
	 * @current_session: Pointer to the current session.
	 */
	struct fira_session *current_session;
	/**
	 * @src_short_addr: Source address for the current session (actually
	 * never put as a source address in a frame, but used for control
	 * message).
	 */
	__le16 src_short_addr;
	/**
	 * @dst_short_addr: Destination address for the current session. When
	 * controller, this is broadcast or the address of the only controlee.
	 * When controlee, this is the address of the controller.
	 */
	__le16 dst_short_addr;
	/**
	 * @block_duration_rx_margin_ppm: Block duration rx margin for
	 * controlees.
	 */
	int block_duration_rx_margin_ppm;
	/**
	 * @slots: Descriptions of each active slots for the current session.
	 * When controller, this is filled when the access is requested. When
	 * controlee, the first slot is filled when the access is requested and
	 * the other slots are filled when the control message is received.
	 */
	struct fira_slot slots[FIRA_FRAMES_MAX];
	/**
	 * @ranging_info: Information on ranging for the current session. Index
	 * in the table is determined by the order of the ranging messages.
	 * First ranging exchange is put at index 0. When a message is shared
	 * between several exchanges, its information is stored at index 0.
	 * Reset when access is requested.
	 */
	struct fira_ranging_info ranging_info[FIRA_CONTROLEES_MAX];
	/**
	 * @n_ranging_info: Number of element in the ranging information table.
	 */
	int n_ranging_info;
	/**
	 * @n_ranging_valid: Number of valid ranging in the current ranging
	 * information table.
	 */
	int n_ranging_valid;
	/**
	 * @diagnostics: Diagnostic collected for each slot.
	 */
	struct fira_diagnostic diagnostics[FIRA_FRAMES_MAX];
	/**
	 * @stopped_controlees: Short addresses of the stopped controlees for
	 * which an element must be added to the Device Management List of
	 * the control message.
	 */
	__le16 stopped_controlees[FIRA_CONTROLEES_MAX];
	/**
	 * @n_stopped_controlees: Number of elements in the stopped controlees .
	 */
	int n_stopped_controlees;
};

static const s64 speed_of_light_mm_per_s = 299702547000ull;

static inline s64 fira_rctu_to_mm(s64 rctu_freq_hz, s32 rctu)
{
	s64 temp = speed_of_light_mm_per_s * rctu + rctu_freq_hz / 2;
	return div64_s64(temp, rctu_freq_hz);
}

static inline s64 fira_mm_to_rctu(struct fira_local *local, s32 mm)
{
	s64 temp = (s64)mm * local->llhw->dtu_freq_hz * local->llhw->dtu_rctu +
		   speed_of_light_mm_per_s / 2;
	return div64_s64(temp, speed_of_light_mm_per_s);
}

static inline struct fira_local *
region_to_local(struct mcps802154_region *region)
{
	return container_of(region, struct fira_local, region);
}

static inline struct fira_local *
access_to_local(struct mcps802154_access *access)
{
	return container_of(access, struct fira_local, access);
}

/**
 * fira_get_session_by_session_id() - Get a session by its identifier.
 * @local: FiRa context.
 * @session_id: Session identifier.
 *
 * Return: The session or NULL if not found.
 */
struct fira_session *fira_get_session_by_session_id(struct fira_local *local,
						    u32 session_id);

/**
 * fira_check_all_missed_ranging() - Check missed ranging round for all active
 * session except the recent.
 * @local: FiRa context.
 * @recent_session: FiRa session to not check in active list.
 * @timestamp_dtu: Timestamp used to trig (or not) a report of ranging failure.
 */
void fira_check_all_missed_ranging(struct fira_local *local,
				   const struct fira_session *recent_session,
				   u32 timestamp_dtu);

#endif /* NET_FIRA_REGION_H */
