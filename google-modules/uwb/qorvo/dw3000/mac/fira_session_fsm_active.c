/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2022 Qorvo US, Inc.
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

#include <net/mcps802154_frame.h>
#include <net/fira_region_nl.h>
#include <linux/errno.h>
#include <linux/math64.h>

#include "fira_round_hopping_sequence.h"
#include "fira_session_fsm_init.h"
#include "fira_session_fsm_idle.h"
#include "fira_session_fsm_active.h"
#include "fira_session.h"
#include "fira_trace.h"
#include "warn_return.h"

/**
 * list_move_to_active() - Move from inactive list to active list.
 * @local: FiRa context.
 * @session: Session context.
 */
static void list_move_to_active(struct fira_local *local,
				struct fira_session *session)
{
	struct list_head *position = &local->active_sessions;
	struct fira_session *tmp;

	/*
	 * Search the position to maintain a list sorted from highest to
	 * lowest priority. And for the same priority keep the call
	 * order (moved to the tail).
	 * Highest value of priority is the highest priority.
	 * Range of priority is between: 0 to FIRA_PRIORITY_MAX.
	 */
	list_for_each_entry (tmp, &local->active_sessions, entry) {
		if (session->params.priority <= tmp->params.priority)
			position = &tmp->entry;
		else
			break;
	}
	list_move(&session->entry, position);
}

/**
 * get_channel() - Retrieve the channel to applied on the access.
 * @local: FiRa context.
 * @session: Session context.
 *
 * Return: The channel.
 */
static const struct mcps802154_channel *
get_channel(struct fira_local *local, const struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;

	if (params->channel_number || params->preamble_code_index) {
		const struct mcps802154_channel *channel =
			mcps802154_get_current_channel(local->llhw);

		local->channel = *channel;
		if (params->channel_number)
			local->channel.channel = params->channel_number;
		if (params->preamble_code_index)
			local->channel.preamble_code =
				params->preamble_code_index;
		return &local->channel;
	}
	return NULL;
}

/**
 * get_round_index() - Return the round index for a specific block index.
 * @session: Session context.
 * @block_index: Block index.
 *
 * Return: Round index freshly computed or the round index saved.
 */
static int get_round_index(const struct fira_session *session, int block_index)
{
	const struct fira_session_params *params = &session->params;
	int expected_block_index;

	switch (params->device_type) {
	default:
	case FIRA_DEVICE_TYPE_CONTROLLER:
		if (!params->round_hopping)
			return 0;
		/*
		 * Avoid to rebuild the round_index.
		 * The condition is true on first get_access too.
		 */
		if (session->controller.next_block_index == block_index)
			return session->next_round_index;
		break;
	case FIRA_DEVICE_TYPE_CONTROLEE:
		if (!session->controlee.hopping_mode)
			return 0;
		/*
		 * Return the round index received, only when the block index
		 * match with expected block index.
		 */
		expected_block_index = session->controlee.block_index_sync +
				       session->block_stride_len + 1;
		if (expected_block_index == block_index &&
		    session->controlee.next_round_index_valid)
			return session->next_round_index;
		break;
	}
	return fira_round_hopping_sequence_get(session, block_index);
}

/**
 * get_rx_margin_duration_dtu() - Build the maximum margin tolerance for Rx.
 * @local: FiRa context.
 * @session: Session context.
 *
 * Return: Duration to apply on first and Rx frame of controlee's access.
 */
static int get_rx_margin_duration_dtu(const struct fira_local *local,
				      const struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	s64 duration_dtu = (s64)(session->block_stride_len + 1) *
			   params->block_duration_dtu;
	/*
	 * TODO: Unit test should be able to predic timestamp.
	 * - Replace 'local->block_duration_rx_margin_ppm by'
	 *   UWB_BLOCK_DURATION_MARGIN_PPM
	 * - Remove 'local' from args.
	 */
	return div64_s64(duration_dtu * local->block_duration_rx_margin_ppm,
			 1000000);
}

/**
 * get_next_access_timestamp_dtu() - Build the next access timestamp.
 * @local: FiRa context.
 * @session: Session context.
 *
 * Return: Timestamp in dtu.
 */
static u32 get_next_access_timestamp_dtu(const struct fira_local *local,
					 const struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	int add_blocks = session->block_stride_len + 1;
	int next_block_index = session->block_index + add_blocks;
	int next_round_index = get_round_index(session, next_block_index);
	int round_duration_dtu =
		params->round_duration_slots * params->slot_duration_dtu;
	u32 next_block_start_dtu = session->block_start_dtu +
				   add_blocks * params->block_duration_dtu +
				   next_round_index * round_duration_dtu;

	switch (params->device_type) {
	default:
	case FIRA_DEVICE_TYPE_CONTROLLER:
		return next_block_start_dtu;
	case FIRA_DEVICE_TYPE_CONTROLEE:
		return next_block_start_dtu -
		       get_rx_margin_duration_dtu(local, session);
	}
}

/**
 * is_controlee_synchronised() - Answer to the question of the synchronisation
 * status.
 * @local: FiRa context.
 * @session: Session context.
 *
 * Return: True when the controlee session is still synchronized.
 */
static bool is_controlee_synchronised(const struct fira_local *local,
				      const struct fira_session *session)
{
#define FIRA_DRIFT_TOLERANCE_PPM 30
	const struct fira_session_params *params = &session->params;
	int n_unsync_blocks;
	s64 unsync_duration_dtu;
	int drift_ppm, rx_margin_ppm;

	if (session->controlee.synchronised) {
		n_unsync_blocks = session->block_index -
				  session->controlee.block_index_sync;
		unsync_duration_dtu =
			n_unsync_blocks * params->block_duration_dtu;
		drift_ppm = div64_s64(unsync_duration_dtu *
					      FIRA_DRIFT_TOLERANCE_PPM,
				      1000000);
		rx_margin_ppm = get_rx_margin_duration_dtu(local, session);

		trace_region_fira_is_controlee_synchronised(session, drift_ppm,
							    rx_margin_ppm);
		if (drift_ppm <= rx_margin_ppm)
			return true;
	}
	return false;
}

/**
 * is_stopped() - Is the session stopped?
 * @session: Session context.
 *
 * Return: True when the session is stopped, false otherwise.
 */
static bool is_stopped(struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	int nb_controlee = fira_session_controlees_running_count(session);

	if (params->max_rr_retry &&
	    session->n_ranging_round_retry >= params->max_rr_retry)
		session->stop_no_response = true;

	return (session->stop_request && !nb_controlee) ||
	       session->stop_inband || session->stop_no_response;
}

/**
 * forward_to_next_ranging() - Update the session to forward to next ranging.
 * @session: Session context.
 * @n_ranging: Number of ranging (forward).
 */
static void forward_to_next_ranging(struct fira_session *session, int n_ranging)
{
	const struct fira_session_params *params = &session->params;
	int blocks_per_ranging = session->block_stride_len + 1;
	int add_blocks = n_ranging * blocks_per_ranging;
	int duration_dtu = add_blocks * params->block_duration_dtu;

	session->block_index += add_blocks;
	session->block_start_dtu += duration_dtu;
	session->n_ranging_round_retry += n_ranging;
}

/**
 * ranging_round_done() - Update controlee and notify the upper layer and rotate crypto keys.
 * @local: FiRa context.
 * @session: Session context.
 * @report_info: Report information to forward fira_session_report.
 */
static void ranging_round_done(struct fira_local *local,
			       struct fira_session *session,
			       struct fira_report_info *report_info)
{
	const struct fira_session_params *params = &session->params;

	session->next_access_timestamp_dtu =
		get_next_access_timestamp_dtu(local, session);
	report_info->stopped = is_stopped(session);

	switch (params->device_type) {
	default:
	case FIRA_DEVICE_TYPE_CONTROLLER:
		/* Update controlee's states between two ranging round. */
		fira_session_update_controlees(local, session);
		fira_sts_rotate_keys(session);
		break;
	case FIRA_DEVICE_TYPE_CONTROLEE:
		/* Did the controlee's access lose the synchronisation? */
		session->controlee.synchronised =
			is_controlee_synchronised(local, session);
		if (session->controlee.synchronised)
			fira_sts_rotate_keys(session);
		break;
	}

	fira_session_report(local, session, report_info);

	if (report_info->stopped) {
		fira_session_fsm_change_state(local, session,
					      &fira_session_fsm_idle);
	} else {
		forward_to_next_ranging(session, 1);
	}
}

static void fira_session_fsm_active_enter(struct fira_local *local,
					  struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;

	session->stop_request = false;
	session->stop_inband = false;
	session->stop_no_response = false;
	session->measurements.n_total_achieved = 0;
	session->block_stride_len = params->block_stride_len;
	session->controlee.synchronised = false;
	session->controlee.hopping_mode = false;
	session->controlee.next_round_index_valid = false;
	session->controlee.block_index_sync = 0;
	session->round_index = 0;
	/*
	 * Initialize to 1 when initiation_time_ms is 0,
	 * because first add_blocks built will be 0.
	 */
	session->n_ranging_round_retry = params->initiation_time_ms ? 0 : 1;

	list_move_to_active(local, session);
}

static void fira_session_fsm_active_leave(struct fira_local *local,
					  struct fira_session *session)
{
	fira_sts_deinit(session);
	list_move(&session->entry, &local->inactive_sessions);
	fira_session_restart_controlees(session);
}

static int
fira_session_fsm_active_check_parameters(const struct fira_session *session,
					 struct nlattr **attrs)
{
	const struct fira_session_params *params = &session->params;
	enum fira_session_param_attrs i;

	for (i = FIRA_SESSION_PARAM_ATTR_UNSPEC + 1;
	     i <= FIRA_SESSION_PARAM_ATTR_MAX; i++) {
		const struct nlattr *attr = attrs[i];

		if (!attr)
			/* Attribute not provided. */
			continue;

		switch (i) {
		case FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE:
		case FIRA_SESSION_PARAM_ATTR_DATA_PAYLOAD:
		case FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_CONFIG:
		case FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_PROXIMITY_NEAR_MM:
		case FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_PROXIMITY_FAR_MM:
		case FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI:
		case FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI:
		case FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI:
		case FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI:					/* Allowed for all device type. */
			break;
		case FIRA_SESSION_PARAM_ATTR_BLOCK_STRIDE_LENGTH:
			/* Allowed only for controller. */
			if (params->device_type == FIRA_DEVICE_TYPE_CONTROLLER)
				continue;
			return -EBUSY;
		default:
			return -EBUSY;
		}
	}
	return 0;
}

static void
fira_session_fsm_active_parameters_updated(struct fira_local *local,
					   struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	int i;

	if (session->measurements.reset) {
		for (i = 0; i < params->meas_seq.n_steps; i++) {
			const struct fira_measurement_sequence_step *step;

			step = &params->meas_seq.steps[i];
			trace_region_fira_session_meas_seq_params(session, step,
								  i);
		}
	}
}

static int fira_session_fsm_active_start(struct fira_local *local,
					 struct fira_session *session,
					 const struct genl_info *info)
{
	/* Already started. */
	return 0;
}

static int fira_session_fsm_active_stop(struct fira_local *local,
					struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	struct fira_report_info report_info = {
		.stopped = true,
	};

	switch (params->device_type) {
	default:
	case FIRA_DEVICE_TYPE_CONTROLLER:
		if (local->current_session == NULL) {
			session->stop_request = true;
			/*
			 * FIXME/BUG:
			 * In unit test, the stop_tx_frame_error (or rx),
			 * stop the current access and trig a broken event.
			 * Then the TearDown request a session_stop, but
			 * there is still more than one controlee running.
			 *
			 * Normally the controller must do an access to
			 * announce a stop of all controlees.
			 * But here, there is a missing mechanism, as:
			 *  - notify_stop is not called,
			 *  - error is only a boolean in access_done.
			 *    And the error boolean is True on -ETIME.
			 *
			 * And as the current_session equal to NULL is a
			 * normal behavior in multi-region. We have a bug.
			 */
			fira_session_report(local, session, &report_info);
			fira_session_fsm_change_state(local, session,
						      &fira_session_fsm_idle);
		} else if (session->n_current_controlees) {
			/*
			 * A ranging round to announced all controlee
			 * stopped is required.
			 */
			fira_session_stop_controlees(session);
			session->stop_request = true;
		} else if (local->current_session != session) {
			session->stop_request = true;
			fira_session_report(local, session, &report_info);
			fira_session_fsm_change_state(local, session,
						      &fira_session_fsm_idle);
		}
		break;
	case FIRA_DEVICE_TYPE_CONTROLEE:
		session->stop_request = true;
		if (local->current_session != session) {
			fira_session_report(local, session, &report_info);
			fira_session_fsm_change_state(local, session,
						      &fira_session_fsm_idle);
		}
		break;
	}
	mcps802154_reschedule(local->llhw);
	return 0;
}

static int
fira_session_fsm_active_get_demand(const struct fira_local *local,
				   const struct fira_session *session,
				   u32 next_timestamp_dtu, int max_duration_dtu,
				   struct fira_session_demand *session_demand)
{
	const struct fira_session_params *params = &session->params;
	int round_duration_dtu =
		params->round_duration_slots * params->slot_duration_dtu;
	u32 block_start_dtu;
	u32 timestamp_dtu;
	u32 duration_dtu;
	int round_index = 0;
	u32 block_index;
	int add_blocks = 0;
	int rx_timeout_dtu = 0;
	int slot_count;

	/* First, determine two dates: block_start_dtu and timestamp_dtu. */
	if (!is_before_dtu(session->next_access_timestamp_dtu,
			   next_timestamp_dtu)) {
		/*
		 * block_start_dtu is set in the future or present.
		 * It's happen mainly when initiation_time_ms is not zero.
		 */
		timestamp_dtu = block_start_dtu = session->block_start_dtu;
	} else {
		/*
		 * block start is in the past, we have to evaluate the
		 * new block start dtu.
		 * It's could be the same with a controlee not synchronized.
		 *
		 * Example of time graph of what's could happen:
		 *
		 * -------x----------------x----------------x-------
		 * #x - 1 | Block Index #x |    #x + 1      | #x + 2
		 * -------x----------------x------x---------x-------> Time
		 *        |<--------------------->|
		 *     Block         |            |
		 *     start         |            next_timestamp_dtu
		 *                   |
		 * duration_from_block_start_dtu
		 *
		 * In the graph example, one block is missed, but it's could be
		 * more or less(controlee).
		 */
		int duration_from_block_start_dtu =
			next_timestamp_dtu - session->block_start_dtu;

		if (params->device_type == FIRA_DEVICE_TYPE_CONTROLEE &&
		    !session->controlee.synchronised) {
			/*
			 * With a controlee not synchronized, consider the
			 * block as missed when there is no more left duration
			 * in the current block.
			 *
			 *      block
			 *      start       next_timestamp_dtu
			 *        |                 |
			 * -------x-----------------x---x------
			 * #x - 1 |       #x        :   | #x + 1
			 * -------x-----------------x---x-----> Time
			 *        |                 |
			 *        |<--------------->|
			 *                  |
			 *                  |
			 *   add_blocks = Time / block_duration
			 */
			add_blocks = duration_from_block_start_dtu /
				     params->block_duration_dtu;
		} else {
			int blocks_per_ranging = session->block_stride_len + 1;

			/*
			 * With a controller or a controlee synchronized,
			 * consider a block started as a missed block.
			 */
			add_blocks = (duration_from_block_start_dtu +
				      params->block_duration_dtu - 1) /
				     params->block_duration_dtu;
			/*
			 * Block stride feature announced/received in last
			 * access.
			 */
			if (session->block_stride_len) {
				int n = add_blocks % blocks_per_ranging;

				/*
				 * Add more block(s) to reach block stride
				 * modulo.
				 */
				if (n)
					add_blocks += blocks_per_ranging - n;
			}
		}

		/* Compute block start dtu. 'add_blocks' can be zero. */
		block_start_dtu = session->block_start_dtu +
				  add_blocks * params->block_duration_dtu;
		/* Determine the access timestamp. */
		if (is_before_dtu(block_start_dtu, next_timestamp_dtu))
			/*
			 * Only the controlee not synchronized can have its
			 * next access timestamp_dtu in the future of the
			 * block start.
			 *
			 * block_start_dtu
			 *        |
			 * -------x-----------------x----------
			 * #x - 1 | Block index #x  | #x + 1
			 * -------x------x----------x----------> Time
			 *               |
			 *      next_timestamp_dtu
			 */
			timestamp_dtu = next_timestamp_dtu;
		else
			timestamp_dtu = block_start_dtu;
	}

	/*
	 * As block_start_dtu is updated with new timestamp in the future,
	 * or still in the past (controlee), then other variables will be
	 * build to fill the session_demand output.
	 *
	 * In other words, locale variables can have a new values which
	 * represent the next(future) block/access/index/...
	 * Or keep +/- the same values.
	 */
	block_index = session->block_index + add_blocks;
	switch (params->device_type) {
	default:
	case FIRA_DEVICE_TYPE_CONTROLLER:
		slot_count = fira_session_get_slot_count(session);
		round_index = get_round_index(session, block_index);
		timestamp_dtu =
			block_start_dtu + round_index * round_duration_dtu;
		duration_dtu = slot_count * params->slot_duration_dtu;
		if (max_duration_dtu &&
		    is_before_dtu(next_timestamp_dtu + max_duration_dtu,
				  timestamp_dtu + duration_dtu))
			/* No way to start an access. */
			return 0;
		break;
	case FIRA_DEVICE_TYPE_CONTROLEE:
		if (session->controlee.synchronised) {
			int margin_less, margin_more;

			/*
			 * Time graph to illustrate the controlee access
			 * and its synchronization.
			 *
			 *    Next          Timestamp
			 *  timestamp     without margin
			 *      |             |
			 *  ----x--------x----x----x-----> Time
			 *               |<---|--->|
			 *      Rx enabled         Rx timeout
			 *    @timestamp_dtu
			 *
			 * rx_margin is the maximum margin accepted.
			 */
			round_index = get_round_index(session, block_index);
			timestamp_dtu += round_index * round_duration_dtu;
			margin_less = margin_more =
				get_rx_margin_duration_dtu(local, session);
			if (timestamp_dtu - next_timestamp_dtu < margin_less)
				/*
				 * Avoid to build a timestamp_dtu which is in
				 * the past of next_timestamp_dtu.
				 */
				margin_less =
					timestamp_dtu - next_timestamp_dtu;
			timestamp_dtu -= margin_less;
			rx_timeout_dtu = margin_less + margin_more;
			duration_dtu = round_duration_dtu + margin_less;
			if (max_duration_dtu &&
			    is_before_dtu(next_timestamp_dtu + max_duration_dtu,
					  timestamp_dtu + duration_dtu))
				/* No way to start an access. */
				return 0;
		} else {
			int unsync_max_duration_dtu =
				params->block_duration_dtu +
				params->slot_duration_dtu;

			/*
			 * A controlee not synchronized is allowed to start/end
			 * anywhere in the block to find the controller.
			 * But the session continue to work with block duration
			 * to provide:
			 *  - Regular reporting.
			 *  - Time-sharing in multi-session/multi-region.
			 *
			 * Time graph:
			 *
			 *           unsync_max_duration_dtu
			 *       |<----------------------------->|
			 *       |                               |
			 * --+---x-----------------------|-------x------>
			 *   |        Block #x           |  Block #x + 1
			 * --+---x-----------------------|---x---x------> Time
			 *       |<------------------------->|<->|
			 *             block duration         slot duration
			 *
			 * The unsync duration is bigger than the block, to
			 * listen the medium for one block min. But to avoid
			 * to be in late on the next access, we must add one
			 * slot.
			 */
			if (max_duration_dtu &&
			    is_before_dtu(next_timestamp_dtu + max_duration_dtu,
					  timestamp_dtu +
						  params->slot_duration_dtu))
				/* No way to start an access. */
				return 0;
			else if (!max_duration_dtu ||
				 is_before_dtu(timestamp_dtu +
						       unsync_max_duration_dtu,
					       next_timestamp_dtu +
						       max_duration_dtu))
				/* Maximum access granted. */
				duration_dtu = unsync_max_duration_dtu;
			else
				/* Adjusted access duration. */
				duration_dtu = next_timestamp_dtu +
					       max_duration_dtu - timestamp_dtu;

			/*
			 * 'rx_timeout_dtu' is set to allow the reception
			 * of the control frame close to the end of the
			 * access, and so be synchronized for next block.
			 *
			 * But if the control message is received
			 * at the end of access, the other frames
			 * will be dropped to respect the duration_dtu.
			 * See: rx control frame.
			 */
			rx_timeout_dtu =
				duration_dtu - params->slot_duration_dtu;
		}
		break;
	}

	/*
	 * Update the session demand (output):
	 * - rx_timeout_dtu: Used only by the controlee.
	 *
	 * On the get_access, the session_demand will be applied
	 * to the session. Otherwise the session_demand is dropped.
	 *
	 * In a way, session_demand represent the next access.
	 */
	*session_demand = (struct fira_session_demand){
		.block_start_dtu = block_start_dtu,
		.timestamp_dtu = timestamp_dtu,
		.max_duration_dtu = duration_dtu,
		.add_blocks = add_blocks,
		.rx_timeout_dtu = rx_timeout_dtu,
		.round_index = round_index,
	};
	trace_region_fira_session_fsm_active_get_demand_return(local, session,
							       session_demand);
	return 1;
}

static struct mcps802154_access *
fira_session_fsm_active_get_access(struct fira_local *local,
				   struct fira_session *session,
				   const struct fira_session_demand *fsd)
{
	const struct fira_session_params *params = &session->params;
	const struct mcps802154_hrp_uwb_params *hrp = &session->hrp_uwb_params;
	struct mcps802154_access *access = &local->access;
	int blocks_per_ranging;

	/*
	 *      ,     ,
	 *     (\____/)              Important:
	 *      (_oo_)
	 *        (O)        It's almost forbidden to update session
	 *      __||__    \) content for a controlee.
	 *   []/______\[] /
	 *   / \______/ \/   Because, the session can change on control
	 *  /    /__\        frame reception (static STS only).
	 * (\   /____\
	 */
	local->current_session = session;

	/*
	 * Update common access fields for controlee and controller.
	 * hrp must stay const, see 'Important' above.
	 */
	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->frames = local->frames;
	access->n_frames = 0;
	access->channel = get_channel(local, session);
	access->hrp_uwb_params = hrp;

	/*
	 * For the ranging round failure counter, consider these rounds as
	 * failed. And reset the counter in the access_done if success.
	 */
	blocks_per_ranging = session->block_stride_len + 1;
	session->n_ranging_round_retry += fsd->add_blocks / blocks_per_ranging;

	/* Continue to 'device type' access. */
	if (params->device_type == FIRA_DEVICE_TYPE_CONTROLLER)
		return fira_get_access_controller(local, fsd);
	return fira_get_access_controlee(local, fsd);
}

static void fira_session_fsm_active_access_done(struct fira_local *local,
						struct fira_session *session,
						bool error)
{
	const struct fira_session_params *params = &session->params;
	const struct fira_measurement_sequence_step *step;
	struct fira_report_info report_info = {
		.ranging_data = local->ranging_info,
		.n_ranging_data = local->n_ranging_info,
		.stopped_controlees = local->stopped_controlees,
		.n_stopped_controlees = local->n_stopped_controlees,
		.diagnostics = local->diagnostics,
		.slots = local->slots,
		.n_slots = local->access.n_frames,
	};
	struct fira_ranging_info *ri;
	int i;

	/* Update local. */
	local->current_session = NULL;

	if (error) {
		/*
		 * FIXME:
		 * Why corrupt all status, the last slot_index is not
		 * enough?
		 * TODO: Proposal:
		 *  - Set INTERNAL_ERROR on status during the get_access.
		 *  - Update status on tx_return/rx_frame.
		 *  - Update testu which expect the wrong status.
		 */
		for (i = 0; i < local->n_ranging_info; i++) {
			ri = &local->ranging_info[i];
			ri->status = FIRA_STATUS_RANGING_INTERNAL_ERROR;
		}
	} else {
		for (i = 0; i < local->n_ranging_info; i++) {
			ri = &local->ranging_info[i];
			if (ri->status != FIRA_STATUS_RANGING_SUCCESS)
				break;
		}
		/* Reset ranging round failure counter. */
		if (i == local->n_ranging_info)
			session->n_ranging_round_retry = 0;
	}

	session->measurements.n_achieved++;
	session->measurements.n_total_achieved++;
	step = fira_session_get_meas_seq_step(session);
	if (session->measurements.reset) {
		/* Copy new measurement sequence. */
		session->measurements.sequence = params->meas_seq;
		session->measurements.index = 0;
		session->measurements.n_achieved = 0;
		session->measurements.reset = false;
	} else if (session->measurements.n_achieved >= step->n_measurements) {
		struct fira_measurement_sequence *seq =
			&session->measurements.sequence;

		session->measurements.n_achieved = 0;
		session->measurements.index++;
		session->measurements.index %= seq->n_steps;
	}
	/* Check max number of measurements. */
	if (params->max_number_of_measurements &&
	    session->measurements.n_total_achieved >=
		    params->max_number_of_measurements) {
		session->stop_request = true;
	}

	ranging_round_done(local, session, &report_info);
}

static void
fira_session_fsm_active_check_missed_ranging(struct fira_local *local,
					     struct fira_session *session,
					     u32 timestamp_dtu)
{
	const struct fira_session_params *params = &session->params;
	int next_block_start_dtu =
		session->block_start_dtu + params->block_duration_dtu;
	bool is_missed_ranging_round = false;

	/*
	 * Example of possible timings (without hopping):
	 *
	 *                          check(timestamp_dtu)
	 *          Ok        Miss      Miss  |
	 * Session: [--]      [--]      [--]  |   [--]
	 *    ------x---------x---------------x--------> Time
	 *          |         |
	 * block_start_dtu    next_access_timestamp_dtu
	 *
	 * Tips:
	 * - 'session->block_start_dtu' is the block start of the last access.
	 * - 'session->next_access_timestamp_dtu' value can be:
	 *   - Next block start when hopping is disabled.
	 *   - Beyond the next block start when hopping is enabled.
	 * - When the session haven't been check since a long time,
	 *   many blocks could be missed.
	 */

	/* First, determine if there is missed ranging round. */
	if (params->device_type == FIRA_DEVICE_TYPE_CONTROLEE &&
	    !session->controlee.synchronised) {
		/* Consider the block as missed when next block is reached. */
		if (!is_before_dtu(timestamp_dtu, next_block_start_dtu))
			is_missed_ranging_round = true;
	} else if (is_before_dtu(session->next_access_timestamp_dtu,
				 timestamp_dtu)) {
		/* A late is not accepted here. */
		is_missed_ranging_round = true;
	}

	/* Compute the number of missed ranging. */
	if (is_missed_ranging_round) {
		int blocks_per_ranging = session->block_stride_len + 1;
		int add_blocks = 0;

		/* Drift probably due to multi-session or multi-region. */
		if (is_before_dtu(next_block_start_dtu, timestamp_dtu))
			add_blocks = (timestamp_dtu - next_block_start_dtu) /
				     params->block_duration_dtu;
		if (add_blocks >= blocks_per_ranging) {
			int n_ranging_failed = add_blocks / blocks_per_ranging;

			if (params->max_rr_retry &&
			    session->n_ranging_round_retry + n_ranging_failed >
				    params->max_rr_retry) {
				/*
				 * Avoid to set a block index bigger than the
				 * max ranging round retry in the report.
				 */
				n_ranging_failed =
					params->max_rr_retry -
					session->n_ranging_round_retry;
			}
			forward_to_next_ranging(session, n_ranging_failed);
		}
	}

	/* Finally, do the missed ranging round report. */
	if (is_missed_ranging_round) {
		struct fira_report_info report_info = {};
		__le16 *pend_del;
		struct fira_ranging_info *ri;
		int j, k;
		struct fira_controlee *controlee;

		/*
		 *           \\\||||||////
		 *            \\  ~ ~  //
		 *             (  @ @  )
		 * _________ oOOo-(_)-oOOo________________________________
		 * WARN_RETURN_VOID_ON: Because the 'local' information will
		 * be used until the end of this bloc.
		 * So this function must not be called during an access,
		 * to avoid to use a shared memory already used by current
		 * session.
		 * ________________Oooo.__________________________________
		 *       .oooO     (   )
		 *        (   )     ) /
		 *         \ (     (_/
		 *          \_)
		 */
		WARN_RETURN_VOID_ON(local->current_session);
		/* Build a missed ranging round report. */
		report_info.ranging_data = local->ranging_info;
		switch (params->device_type) {
		default:
		case FIRA_DEVICE_TYPE_CONTROLLER:
			pend_del = local->stopped_controlees;
			j = k = 0;
			list_for_each_entry (controlee,
					     &session->current_controlees,
					     entry) {
				switch (controlee->state) {
				case FIRA_CONTROLEE_STATE_RUNNING:
				case FIRA_CONTROLEE_STATE_PENDING_STOP:
				case FIRA_CONTROLEE_STATE_PENDING_DEL:
					ri = &local->ranging_info[j];
					*ri = (struct fira_ranging_info){
						.short_addr =
							controlee->short_addr,
						.status =
							FIRA_STATUS_RANGING_TX_FAILED,
					};
					j++;
					break;
				default:
					pend_del[k++] = controlee->short_addr;
					break;
				}
			}
			report_info.stopped_controlees = pend_del;
			report_info.n_stopped_controlees = k,
			report_info.n_ranging_data = j;
			break;
		case FIRA_DEVICE_TYPE_CONTROLEE:
			ri = &local->ranging_info[0];
			*ri = (struct fira_ranging_info){
				.short_addr = params->controller_short_addr,
				.status = FIRA_STATUS_RANGING_RX_TIMEOUT,
			};
			report_info.n_ranging_data = 1;
			break;
		}
		ranging_round_done(local, session, &report_info);
	}
}

const struct fira_session_fsm_state fira_session_fsm_active = {
	.id = FIRA_SESSION_STATE_ID_ACTIVE,
	.enter = fira_session_fsm_active_enter,
	.leave = fira_session_fsm_active_leave,
	.check_parameters = fira_session_fsm_active_check_parameters,
	.parameters_updated = fira_session_fsm_active_parameters_updated,
	.start = fira_session_fsm_active_start,
	.stop = fira_session_fsm_active_stop,
	.get_demand = fira_session_fsm_active_get_demand,
	.get_access = fira_session_fsm_active_get_access,
	.access_done = fira_session_fsm_active_access_done,
	.check_missed_ranging = fira_session_fsm_active_check_missed_ranging,
};
