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

#ifndef NET_MCPS802154_FIRA_SESSION_H
#define NET_MCPS802154_FIRA_SESSION_H

#include "fira_session_fsm.h"
#include "fira_region.h"
#include "fira_sts.h"
#include "fira_crypto.h"
#include "fira_round_hopping_crypto_impl.h"

/**
 * enum fira_controlee_state - State of controlee.
 * @FIRA_CONTROLEE_STATE_PENDING_RUN: The controlee will be set to running state
 * at the end of round.
 * @FIRA_CONTROLEE_STATE_RUNNING: The controlee is running.
 * @FIRA_CONTROLEE_STATE_PENDING_STOP: The controlee will be set to stopping
 * state at the end of round.
 * @FIRA_CONTROLEE_STATE_STOPPING: The controlee is stopping.
 * @FIRA_CONTROLEE_STATE_PENDING_DEL: The controlee will be set to deleting
 * state at the end of round.
 * @FIRA_CONTROLEE_STATE_DELETING: The controlee is being deleted.
 */
enum fira_controlee_state {
	FIRA_CONTROLEE_STATE_PENDING_RUN,
	FIRA_CONTROLEE_STATE_RUNNING,
	FIRA_CONTROLEE_STATE_PENDING_STOP,
	FIRA_CONTROLEE_STATE_STOPPING,
	FIRA_CONTROLEE_STATE_PENDING_DEL,
	FIRA_CONTROLEE_STATE_DELETING,
};

/**
 * struct fira_controlee - Represent a controlee.
 */
struct fira_controlee {
	/**
	 * @sub_session_id: Sub-session ID for the controlee device.
	 */
	__u32 sub_session_id;
	/**
	 * @short_addr: Short address of the controlee.
	 */
	__le16 short_addr;
	/**
	 * @sub_session_key_len: Length of the sub-session key used by
	 * the controlee.
	 */
	__u8 sub_session_key_len;
	/**
	 * @sub_session_key: Sub-session key used by the controlee.
	 */
	u8 sub_session_key[FIRA_KEY_SIZE_MAX];
	/**
	 * @sub_session: Is the controlee using a sub-session.
	 */
	bool sub_session;
	/**
	 * @state: Current state of the controlee.
	 */
	enum fira_controlee_state state;
	/**
	 * @range_data_ntf_status: range_data_ntf status of the controlee.
	 */
	enum fira_range_data_ntf_status range_data_ntf_status;
	/*
	* @crypto: Crypto related variables in the sub-session. Only valid if the current device
	 * is a controller/initiator and the sts_config is FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY.
	 */
	struct fira_crypto *crypto;
	/**
	 * @entry: Entry in list of controlees.
	 */
	struct list_head entry;
};

struct fira_measurement_sequence_step {
	enum fira_measurement_type type;
	u8 n_measurements;
	s8 rx_ant_set_nonranging;
	s8 rx_ant_sets_ranging[2];
	s8 tx_ant_set_nonranging;
	s8 tx_ant_set_ranging;
};

struct fira_measurement_sequence {
	struct fira_measurement_sequence_step
		steps[FIRA_MEASUREMENT_SEQUENCE_STEP_MAX];
	size_t n_steps;
};

struct fira_session_params {
	/* Main parameters. */
	enum fira_device_type device_type;
	enum fira_ranging_round_usage ranging_round_usage;
	enum fira_multi_node_mode multi_node_mode;
	__le16 short_addr;
	__le16 controller_short_addr;
	/* Timings parameters. */
	int initiation_time_ms;
	int slot_duration_dtu;
	int block_duration_dtu;
	int round_duration_slots;
	/* Behaviour parameters. */
	u32 block_stride_len;
	u32 max_number_of_measurements;
	u32 max_rr_retry;
	bool round_hopping;
	u8 priority;
	bool result_report_phase;
	/* Radio. */
	int channel_number;
	int preamble_code_index;
	enum fira_rframe_config rframe_config;
	enum fira_preambule_duration preamble_duration;
	enum fira_sfd_id sfd_id;
	enum fira_sts_segments number_of_sts_segments;
	enum fira_psdu_data_rate psdu_data_rate;
	enum fira_mac_fcs_type mac_fcs_type;
	enum fira_prf_mode prf_mode;
	enum fira_phr_data_rate phr_data_rate;
	/* STS and crypto. */
	enum fira_sts_mode sts_config;
	u8 vupper64[FIRA_VUPPER64_SIZE];
	u8 session_key_len;
	u8 session_key[FIRA_KEY_SIZE_MAX];
	u32 sub_session_id;
	u8 sub_session_key_len;
	u8 sub_session_key[FIRA_KEY_SIZE_MAX];
	bool key_rotation;
	u8 key_rotation_rate;
	bool aoa_result_req;
	bool report_tof;
	bool report_aoa_azimuth;
	bool report_aoa_elevation;
	bool report_aoa_fom;
	enum fira_rssi_report_type report_rssi;
	struct fira_measurement_sequence meas_seq;
	u32 data_vendor_oui;
	u8 data_payload[FIRA_DATA_PAYLOAD_SIZE_MAX];
	u32 data_payload_seq;
	int data_payload_len;
	bool report_diagnostics;
	enum fira_ranging_diagnostics_frame_report_flags diagnostic_report_flags;
	/* Misc */
	enum fira_sts_length sts_length;
	enum fira_range_data_ntf_config range_data_ntf_config;
	u32 range_data_ntf_proximity_near_rctu;
	u32 range_data_ntf_proximity_far_rctu;
	s16 range_data_ntf_lower_bound_aoa_azimuth_2pi;
	s16 range_data_ntf_upper_bound_aoa_azimuth_2pi;
	s16 range_data_ntf_lower_bound_aoa_elevation_2pi;
	s16 range_data_ntf_upper_bound_aoa_elevation_2pi;
};

/**
 * struct fira_session - Session information.
 */
struct fira_session {
	/**
	 * @id: Session identifier.
	 */
	u32 id;
	/**
	 * @sequence_number: Session notification counter.
	 */
	u32 sequence_number;
	/**
	 * @entry: Entry in list of sessions.
	 */
	struct list_head entry;
	/**
	 * @state: State of the session.
	 */
	const struct fira_session_fsm_state *state;
	/**
	 * @params: Session parameters, mostly read only while the session is
	 * active.
	 */
	struct fira_session_params params;
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
	 * @block_start_valid: True when block_start_dtu is valid.
	 * It's false on the first access wo initiation delay.
	 */
	bool block_start_valid;
	/**
	 * @block_start_dtu: Block start timestamp in dtu of the last
	 * get_access.
	 */
	u32 block_start_dtu;
	/**
	 * @next_access_timestamp_dtu: Next access timestamp in dtu.
	 * It's equal to block_start_dtu when the hopping is disabled.
	 * Otherwise it's beyond the block_start_dtu.
	 * It's updated after each good or missed ranging round.
	 */
	u32 next_access_timestamp_dtu;
	/**
	 * @last_access_timestamp_dtu: Last timestamp where the session got
	 * the access.
	 * It's used only on session's election, when a negotiation between
	 * two session fails.
	 */
	u32 last_access_timestamp_dtu;
	/**
	 * @block_index: Block index used on the last access.
	 */
	u32 block_index;
	/**
	 * @block_stride_len: Stride length indicates how many ranging blocks
	 * will be skipped.
	 * The value is updated at the beginning of an access.
	 */
	int block_stride_len;
	/**
	 * @round_index: Round index used on the last access.
	 */
	int round_index;
	/**
	 * @next_round_index: Next round index a announced in measurement
	 * report message.
	 */
	int next_round_index;
	/**
	* @stop_request: Session has been requested to stop.
	*/
	bool stop_request;
	/**
	 * @stop_inband: Session has been requested to stop by controller. This
	 * field is only used for controlee sessions.
	 */
	bool stop_inband;
	/**
	 * @stop_no_response: Session has been requested to stop because ranging
	 * failed for max_rr_retry consecutive rounds.
	 */
	bool stop_no_response;
	/**
	 * @n_ranging_round_retry: Number of ranging round failed.
	 * Counter reset on ranging round success.
	 */
	int n_ranging_round_retry;

	/**
	 * @round_hopping_sequence: Round hopping sequence generation context.
	 */
	struct fira_round_hopping_sequence round_hopping_sequence;
	/**
	 * @controlee: Group of persistent variable(s) used when session
	 * is a controlee.
	 */
	struct {
		/**
		 * @synchronised: Whether a controlee session was synchronised.
		 */
		bool synchronised;
		/**
		 * @block_index_sync: Last block index received.
		 */
		int block_index_sync;
		/**
		 * @hopping_mode: True when hopping is enabled on last
		 * measurement frame.
		 */
		bool hopping_mode;
		/**
		 * @next_round_index_valid: True when the next round index
		 * is present in measurement report frame.
		 */
		bool next_round_index_valid;
		/**
		* @ctlr_range_data_ntf_status: range_data_ntf status of the
		* controller.
		*/
		enum fira_range_data_ntf_status ctlr_range_data_ntf_status;
		 /*
		 * @responder_specific_crypto: crypto related variables in the sub-session.
		 * Only valid if the sts_config is FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY.
		 */
		struct fira_crypto *responder_specific_crypto;
	} controlee;
	/**
	 * @controller: Group of persistent variable(s) used when session
	 * is a controller.
	 */
	struct {
		/**
		 * @next_block_index: Next block index built on get access with
		 * next round index.
		 * It's only to avoid to rebuild the next round index on next
		 * access, when this last occur in time as block index will
		 * match.
		 */
		u32 next_block_index;
	} controller;
	/**
	 * @data_payload: Local context for data_payload feature.
	 */
	struct {
		/**
		 * @seq: Sequence number of last sent data.
		 */
		u32 seq;
		/**
		 * @sent: True when data have been send during ranging round.
		 */
		bool sent;
	} data_payload;
	/**
	 * @current_controlees: Current list of controlees.
	 */
	struct list_head current_controlees;
	/**
	 * @n_current_controlees: Number of elements in the list of current
	 * controlees.
	 */
	size_t n_current_controlees;
	/**
	 * @measurements: Measurement configurations which influence diagnostics.
	 */
	struct {
		/**
		 * @sequence: Copy of the meas_seq parameter on get_access
		 * event.
		 */
		struct fira_measurement_sequence sequence;
		/**
		 * @index: Index of the step in sequence array.
		 */
		int index;
		/**
		 * @n_achieved: Number of measurements done inside a step.
		 */
		int n_achieved;
		/**
		 * @n_total_achieved: Total number of measurements done.
		 */
		int n_total_achieved;
		/**
		 * @reset: True when new parameters have to be retrieved.
		 */
		bool reset;
	} measurements;
	/**
	 * @rx_ctx: Custom rx context for all controlees.
	 */
	void *rx_ctx[FIRA_CONTROLEES_MAX];
	/**
	 * @crypto: crypto related variables in a session.
	 */
	struct fira_crypto *crypto;
	/**
	 * @sts: sts related variables.
	 */
	struct {
		/**
		 * @last_rotation_block_index: index to the last block where the
		 * rotation occurred.
		 */
		u32 last_rotation_block_index;
	} sts;
	/*
	 * @last_error: last error that occurred during the active session.
	 */
	int last_error;
};

/**
 * struct fira_session_demand - Next access information for one FiRa session.
 */
struct fira_session_demand {
	/**
	 * @block_start_dtu: Block start in dtu.
	 */
	u32 block_start_dtu;
	/**
	 * @timestamp_dtu: Access timestamp in dtu.
	 */
	u32 timestamp_dtu;
	/**
	 * @max_duration_dtu: Maximum duration for the access.
	 */
	int max_duration_dtu;
	/**
	 * @add_blocks: Number of block to add.
	 */
	int add_blocks;
	/**
	 * @round_index: Round index to apply for the access.
	 */
	int round_index;
	/**
	 * @rx_timeout_dtu: timeout to apply when first frame of the controlee.
	 */
	int rx_timeout_dtu;
};

/**
 * struct fira_report_info - Report information for all peer.
 */
struct fira_report_info {
	/**
	 * @ranging_data: Base address of ranging data per peer, or null
	 * pointer.
	 */
	struct fira_ranging_info *ranging_data;
	/**
	 * @n_ranging_data: Number of entry in ranging_data above.
	 */
	size_t n_ranging_data;
	/**
	 * @stopped_controlees: NULL, or short address of all stopped controlees.
	 */
	const __le16 *stopped_controlees;
	/**
	 * @n_stopped_controlees: Number of controlees stopped in array above.
	 */
	size_t n_stopped_controlees;
	/**
	 * @diagnostics: Array of diagnostic collected per slots.
	 */
	const struct fira_diagnostic *diagnostics;
	/**
	 * @slots: Array of information slots.
	 */
	const struct fira_slot *slots;
	/**
	 * @n_slots: Number of slots above.
	 */
	size_t n_slots;
	/**
	 * @stopped: True when the session is stopped.
	 */
	bool stopped;
};

/**
 * fira_session_new() - Create a new session.
 * @local: FiRa context.
 * @session_id: Session identifier, must be unique.
 *
 * Return: The new session or NULL on error.
 */
struct fira_session *fira_session_new(struct fira_local *local, u32 session_id);

/**
 * fira_session_free() - Remove a session.
 * @local: FiRa context.
 * @session: Session to remove, must be inactive.
 */
void fira_session_free(struct fira_local *local, struct fira_session *session);

/**
 * fira_session_set_controlees() - Set controlees.
 * @local: FiRa context.
 * @session: Session.
 * @controlees: List of controlees.
 * @slot_duration_us: Duration of a FiRa slot in us (according to session config).
 * @n_controlees: Number of controlees.
 *
 * Return: 0 or error.
 */
int fira_session_set_controlees(struct fira_local *local,
				struct fira_session *session,
				struct list_head *controlees,
				int slot_duration_us, int n_controlees);

/**
 * fira_session_new_controlee() - Add new controlee.
 * @session: Session.
 * @controlee: Controlee to add.
 * @slot_duration_us: Duration of a FiRa slot in us (according to session
 *  config).
 * @active_session: True if controlee addition should be postponed as ranging
 *  session is active.
 *
 * If succeed, function as taken ownership of the structure and removed it
 * from the original list.
 *
 * Return: 0 or error.
 */
int fira_session_new_controlee(struct fira_local *local,
				struct fira_session *session,
				struct fira_controlee *controlee,
				int slot_duration_us,
				bool active_session);

/**
 * fira_session_restart_controlees() - Restart controlee and erase pending del.
 * @session: FiRa session context.
 *
 * Return: Number of controlee removed.
 */
void fira_session_restart_controlees(struct fira_session *session);

/**
 * fira_session_del_controlee() - Delete controlee.
 * @session: Session.
 * @controlee: Controlee to delete.
 * @active_session: True if controlee deletion should be postponed as ranging
 *  session is active.
 *
 * If succeed, function has removed given controlee from its current list and
 * freed it.
 *
 * Return: 0 or error.
 */
int fira_session_del_controlee(struct fira_session *session,
				struct fira_controlee *controlee,
				bool active_session);

/**
 * fira_session_get_controlee() - Get controlee info from short address.
 * @session: Session.
 * @short_addr: Short address of the controlee.
 *
 * Return: The corresponding controlee object or NULL.
 */
static inline struct fira_controlee *
fira_session_get_controlee(struct fira_session *session, u16 short_addr)
{
        struct fira_controlee *controlee;
        list_for_each_entry (controlee, &session->current_controlees, entry) {
                if (controlee->short_addr == short_addr)
                        return controlee;
        }
        return NULL;
}

/**
 * fira_session_stop_controlees() - Stop controlees.
 * @session: Session.
 */
void fira_session_stop_controlees(struct fira_session *session);

/**
 * fira_session_controlees_running_count() - Get the number of running controlees.
 * @session: Session.
 *
 * Return: Number of running controlees.
 */
int fira_session_controlees_running_count(const struct fira_session *session);

/**
 * fira_session_update_controlees() - Update controlee's states.
 * @local: FiRa context.
 * @session: Session to test.
 */
void fira_session_update_controlees(struct fira_local *local,
				    struct fira_session *session);

/**
 * fira_session_is_ready() - Test whether a session is ready to be started.
 * @local: FiRa context.
 * @session: Session to test.
 *
 * Return: true if the session can be started.
 */
bool fira_session_is_ready(const struct fira_local *local,
			   const struct fira_session *session);

/**
 * fira_session_get_meas_seq_step() - Get current measurement step.
 * @session: Session.
 *
 * Return: Current Measurement Sequence step for given session.
 */
static inline const struct fira_measurement_sequence_step *
fira_session_get_meas_seq_step(const struct fira_session *session)
{
	const struct fira_measurement_sequence *seq =
		&session->measurements.sequence;

	return &seq->steps[session->measurements.index];
}

/**
 * fira_session_get_rx_ant_set() - Get Rx antenna set for a given message ID.
 * @session: Session.
 * @message_id: Message ID of FiRa frame.
 *
 * Return: Adequate antenna set id for given frame and session parameters.
 */
static inline int
fira_session_get_rx_ant_set(const struct fira_session *session,
			    enum fira_message_id message_id)
{
	const struct fira_session_params *params = &session->params;
	const struct fira_measurement_sequence_step *step =
		fira_session_get_meas_seq_step(session);

	if (message_id > FIRA_MESSAGE_ID_RFRAME_MAX)
		return step->rx_ant_set_nonranging;

	/* TODO: replace this test by device_role == FIRA_DEVICE_ROLE_INITIATOR
	* as soon as this feature is supported */
	if (params->device_type == FIRA_DEVICE_TYPE_CONTROLLER)
		return step->rx_ant_sets_ranging[0];
	else
		switch (step->type) {
		case FIRA_MEASUREMENT_TYPE_RANGE:
		case FIRA_MEASUREMENT_TYPE_AOA:
		case FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH:
		case FIRA_MEASUREMENT_TYPE_AOA_ELEVATION:
			return step->rx_ant_sets_ranging[0];
		case FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH_ELEVATION:
			return step->rx_ant_sets_ranging
				[message_id == FIRA_MESSAGE_ID_RANGING_FINAL];
		default:
			return -1;
		}
	return -1;
}

/**
 * fira_session_set_range_data_ntf_status() - Update range_data_ntf_status
 * for a given ranging.
 * @session: FiRa session.
 * @ranging_info: ranging data to be evaluated.
 */
void fira_session_set_range_data_ntf_status(
	const struct fira_session *session,
	struct fira_ranging_info *ranging_info);

/**
 * fira_session_report() - Report state change and ranging result for a session.
 * @local: FiRa context.
 * @session: Session to report.
 * @report_info: report information to exploit for the reporting.
 */
void fira_session_report(struct fira_local *local, struct fira_session *session,
			 struct fira_report_info *report_info);

/**
 * fira_session_controlee_active() - Return whether the controlee is currently active.
 * @controlee: Controlee.
 *
 * Return: True if the controlee is currently active.
 */
static inline bool
fira_session_controlee_active(struct fira_controlee *controlee)
{
	switch (controlee->state) {
	case FIRA_CONTROLEE_STATE_RUNNING:
	case FIRA_CONTROLEE_STATE_PENDING_STOP:
	case FIRA_CONTROLEE_STATE_PENDING_DEL:
		return true;
	default:
		return false;
	}
}

#endif /* NET_MCPS802154_FIRA_SESSION_H */
