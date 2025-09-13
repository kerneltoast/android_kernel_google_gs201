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

#include <asm/unaligned.h>

#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/string.h>

#include <net/fira_region_nl.h>
#include <net/mcps802154_frame.h>

#include "fira_session.h"
#include "fira_access.h"
#include "fira_region_call.h"
#include "fira_trace.h"
#include "fira_sts.h"

static const struct nla_policy fira_call_nla_policy[FIRA_CALL_ATTR_MAX + 1] = {
	[FIRA_CALL_ATTR_SESSION_ID] = { .type = NLA_U32 },
	[FIRA_CALL_ATTR_SESSION_PARAMS] = { .type = NLA_NESTED },
	[FIRA_CALL_ATTR_CONTROLEES] = { .type = NLA_NESTED_ARRAY },
};

static const struct nla_policy fira_session_param_nla_policy[FIRA_SESSION_PARAM_ATTR_MAX +
							     1] = {
	[FIRA_SESSION_PARAM_ATTR_DEVICE_TYPE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_DEVICE_TYPE_CONTROLLER),
	[FIRA_SESSION_PARAM_ATTR_DEVICE_ROLE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_DEVICE_ROLE_INITIATOR),
	[FIRA_SESSION_PARAM_ATTR_RANGING_ROUND_USAGE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_RANGING_ROUND_USAGE_DSTWR),
	[FIRA_SESSION_PARAM_ATTR_MULTI_NODE_MODE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_MULTI_NODE_MODE_MANY_TO_MANY),
	[FIRA_SESSION_PARAM_ATTR_SHORT_ADDR] = { .type = NLA_U16 },
	[FIRA_SESSION_PARAM_ATTR_DESTINATION_SHORT_ADDR] = { .type = NLA_U16 },
	[FIRA_SESSION_PARAM_ATTR_INITIATION_TIME_MS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_SLOT_DURATION_RSTU] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_BLOCK_DURATION_MS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_ROUND_DURATION_SLOTS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_BLOCK_STRIDE_LENGTH] =
		NLA_POLICY_MAX(NLA_U32, FIRA_BLOCK_STRIDE_LEN_MAX),
	[FIRA_SESSION_PARAM_ATTR_MAX_NUMBER_OF_MEASUREMENTS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_MAX_RR_RETRY] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_ROUND_HOPPING] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_PRIORITY] =
		NLA_POLICY_MAX(NLA_U8, FIRA_PRIORITY_MAX),
	[FIRA_SESSION_PARAM_ATTR_RESULT_REPORT_PHASE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_MR_AT_INITIATOR] =
		NLA_POLICY_MAX(NLA_U8, FIRA_MEASUREMENT_REPORT_AT_INITIATOR),
	[FIRA_SESSION_PARAM_ATTR_EMBEDDED_MODE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_EMBEDDED_MODE_NON_DEFERRED),
	[FIRA_SESSION_PARAM_ATTR_IN_BAND_TERMINATION_ATTEMPT_COUNT] =
		NLA_POLICY_RANGE(NLA_U32,
				 FIRA_IN_BAND_TERMINATION_ATTEMPT_COUNT_MIN,
				 FIRA_IN_BAND_TERMINATION_ATTEMPT_COUNT_MAX),
	[FIRA_SESSION_PARAM_ATTR_CHANNEL_NUMBER] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_PREAMBLE_CODE_INDEX] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_RFRAME_CONFIG] =
		NLA_POLICY_MAX(NLA_U8, FIRA_RFRAME_CONFIG_SP3),
	[FIRA_SESSION_PARAM_ATTR_PRF_MODE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_PRF_MODE_HPRF_HIGH_RATE),
	[FIRA_SESSION_PARAM_ATTR_PREAMBLE_DURATION] =
		NLA_POLICY_MAX(NLA_U8, FIRA_PREAMBULE_DURATION_64),
	[FIRA_SESSION_PARAM_ATTR_SFD_ID] =
		NLA_POLICY_MAX(NLA_U8, FIRA_SFD_ID_4),
	[FIRA_SESSION_PARAM_ATTR_NUMBER_OF_STS_SEGMENTS] =
		NLA_POLICY_MAX(NLA_U8, FIRA_STS_SEGMENTS_4),
	[FIRA_SESSION_PARAM_ATTR_PSDU_DATA_RATE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_PSDU_DATA_RATE_31M2),
	[FIRA_SESSION_PARAM_ATTR_BPRF_PHR_DATA_RATE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_PHR_DATA_RATE_6M81),
	[FIRA_SESSION_PARAM_ATTR_MAC_FCS_TYPE] =
		NLA_POLICY_MAX(NLA_U8, FIRA_MAC_FCS_TYPE_CRC_32),
	[FIRA_SESSION_PARAM_ATTR_TX_ADAPTIVE_PAYLOAD_POWER] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE] = { .type = NLA_NESTED_ARRAY },
	[FIRA_SESSION_PARAM_ATTR_STS_CONFIG] =
		NLA_POLICY_MAX(NLA_U8, FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY),
	[FIRA_SESSION_PARAM_ATTR_SUB_SESSION_ID] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_VUPPER64] =
		NLA_POLICY_EXACT_LEN(FIRA_VUPPER64_SIZE),
	[FIRA_SESSION_PARAM_ATTR_SESSION_KEY] = {
		.type = NLA_BINARY, .len = FIRA_KEY_SIZE_MAX, },
	[FIRA_SESSION_PARAM_ATTR_SUB_SESSION_KEY] = {
		.type = NLA_BINARY, .len = FIRA_KEY_SIZE_MAX, },
	[FIRA_SESSION_PARAM_ATTR_KEY_ROTATION] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_KEY_ROTATION_RATE] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_AOA_RESULT_REQ] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_REPORT_TOF] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_AZIMUTH] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_ELEVATION] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_FOM] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_REPORT_RSSI] =
		NLA_POLICY_MAX(NLA_U8, FIRA_RSSI_REPORT_AVERAGE),
	[FIRA_SESSION_PARAM_ATTR_DATA_VENDOR_OUI] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_DATA_PAYLOAD] = {
		.type = NLA_BINARY, .len = FIRA_DATA_PAYLOAD_SIZE_MAX, },
	[FIRA_SESSION_PARAM_ATTR_DIAGNOSTICS] =
		NLA_POLICY_MAX(NLA_U8, FIRA_BOOLEAN_MAX),
	[FIRA_SESSION_PARAM_ATTR_DIAGNOSTICS_FRAME_REPORTS_FIELDS] = {.type = NLA_U32},
	[FIRA_SESSION_PARAM_ATTR_STS_LENGTH] =
		NLA_POLICY_MAX(NLA_U8, FIRA_STS_LENGTH_128),
	[FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_CONFIG] =
		NLA_POLICY_MAX(NLA_U8, FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA_CROSSING),
	[FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_PROXIMITY_NEAR_MM] =
		{ .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_PROXIMITY_FAR_MM] =
		{ .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI] =
                NLA_POLICY_RANGE(NLA_S16,
                                 FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI_MIN,
                                 FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI_MAX),
        [FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI] =
                NLA_POLICY_RANGE(NLA_S16,
                                 FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI_MIN,
                                 FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI_MAX),
        [FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI] =
                NLA_POLICY_RANGE(NLA_S16,
                                 FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI_MIN,
                                 FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI_MAX),
        [FIRA_SESSION_PARAM_ATTR_RANGE_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI] =
                NLA_POLICY_RANGE(NLA_S16,
                                 FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI_MIN,
                                 FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI_MAX),
};

/**
 * fira_get_state_by_session_id() - Get state of the session.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 *
 * Return: current session state.
 */
static enum fira_session_state_id
fira_get_state_by_session_id(struct fira_local *local, u32 session_id)
{
	struct fira_session *session =
		fira_get_session_by_session_id(local, session_id);

	if (session)
		return fira_session_get_state_id(session);
	return FIRA_SESSION_STATE_ID_DEINIT;
}

/**
 * fira_session_init() - Initialize FiRa session.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 *
 * Return: 0 or error.
 */
static int fira_session_init(struct fira_local *local, u32 session_id)
{
	struct fira_session *session =
		fira_get_session_by_session_id(local, session_id);

	if (session)
		return -EBUSY;

	session = fira_session_new(local, session_id);
	if (!session)
		return -ENOMEM;

	return 0;
}

/**
 * fira_session_start() - Start FiRa session.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_start(struct fira_local *local, u32 session_id,
			      const struct genl_info *info)
{
	struct fira_session *session;

	session = fira_get_session_by_session_id(local, session_id);
	if (!session)
		return -ENOENT;

	return fira_session_fsm_start(local, session, info);
}

/**
 * fira_session_stop() - Stop FiRa session.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 *
 * Return: 0 or error.
 */
static int fira_session_stop(struct fira_local *local, u32 session_id)
{
	struct fira_session *session;

	session = fira_get_session_by_session_id(local, session_id);
	if (!session)
		return -ENOENT;

	return fira_session_fsm_stop(local, session);
}

/**
 * fira_session_deinit() - Deinitialize FiRa session.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 *
 * Return: 0 or error.
 */
static int fira_session_deinit(struct fira_local *local, u32 session_id)
{
	struct fira_session *session =
		fira_get_session_by_session_id(local, session_id);

	if (!session)
		return -ENOENT;
	if (fira_session_is_active(session))
		return -EBUSY;

	fira_session_free(local, session);
	return 0;
}

/**
 * fira_session_params_set_measurement_sequence_step() - Retrieve a
 * measurement sequence step from a NL message and store it in the session
 * parameters.
 * @params: The parameters contained in the NL message.
 * @info: NL context info.
 * @step: The step where to store the information.
 *
 * Return: 0 or error.
 */
static int fira_session_params_set_measurement_sequence_step(
	const struct nlattr *params, const struct genl_info *info,
	struct fira_measurement_sequence_step *step)
{
#define STEP_ATTR(x) FIRA_SESSION_PARAM_MEAS_SEQ_STEP_ATTR_##x
#define ASR_ATTR(x) \
	FIRA_SESSION_PARAM_MEAS_SEQ_STEP_RX_ANT_SETS_RANGING_ATTR_##x
	static const struct nla_policy meas_seq_step_policy[STEP_ATTR(MAX) +
							    1] = {
		[STEP_ATTR(MEASUREMENT_TYPE)] = { .type = NLA_U8 },
		[STEP_ATTR(N_MEASUREMENTS)] = { .type = NLA_U8 },
		[STEP_ATTR(RX_ANT_SET_NONRANGING)] = { .type = NLA_U8 },
		[STEP_ATTR(RX_ANT_SETS_RANGING)] = { .type = NLA_NESTED },
		[STEP_ATTR(TX_ANT_SET_NONRANGING)] = { .type = NLA_U8 },
		[STEP_ATTR(TX_ANT_SET_RANGING)] = { .type = NLA_U8 },
	};
	static const struct nla_policy
		rx_ant_sets_ranging_policy[ASR_ATTR(MAX) + 1] = {
			[ASR_ATTR(0)] = { .type = NLA_U8 },
			[ASR_ATTR(1)] = { .type = NLA_U8 },
		};
	struct nlattr *step_attrs[STEP_ATTR(MAX) + 1];
	struct nlattr *rx_ant_sets_attrs[ASR_ATTR(MAX) + 1];
	int r = 0;
	u8 n_measurements = 0;

	enum fira_measurement_type type = __FIRA_MEASUREMENT_TYPE_AFTER_LAST;

	r = nla_parse_nested(step_attrs, STEP_ATTR(MAX), params,
			     meas_seq_step_policy, info->extack);
	/* LCOV_EXCL_START */
	if (r)
		return r;
	/* LCOV_EXCL_STOP */

	memset(step, 0, sizeof(struct fira_measurement_sequence_step));
	if (!step_attrs[STEP_ATTR(MEASUREMENT_TYPE)] ||
	    !step_attrs[STEP_ATTR(N_MEASUREMENTS)])
		return -EINVAL;

	type = nla_get_u8(step_attrs[STEP_ATTR(MEASUREMENT_TYPE)]);
	if (type >= __FIRA_MEASUREMENT_TYPE_AFTER_LAST)
		return -EINVAL;
	step->type = type;

	n_measurements = nla_get_u8(step_attrs[STEP_ATTR(N_MEASUREMENTS)]);
	if (n_measurements == 0)
		return -EINVAL;
	step->n_measurements = n_measurements;

#define GET_ANTENNA(nl_attr, ant_set) \
	(ant_set) = !(nl_attr) ? -1 : nla_get_u8(nl_attr);

	GET_ANTENNA(step_attrs[STEP_ATTR(RX_ANT_SET_NONRANGING)],
		    step->rx_ant_set_nonranging);
	GET_ANTENNA(step_attrs[STEP_ATTR(TX_ANT_SET_NONRANGING)],
		    step->tx_ant_set_nonranging);
	GET_ANTENNA(step_attrs[STEP_ATTR(TX_ANT_SET_RANGING)],
		    step->tx_ant_set_ranging);

	if (!step_attrs[STEP_ATTR(RX_ANT_SETS_RANGING)])
		return -EINVAL;

	r = nla_parse_nested(rx_ant_sets_attrs, ASR_ATTR(MAX),
			     step_attrs[STEP_ATTR(RX_ANT_SETS_RANGING)],
			     rx_ant_sets_ranging_policy, info->extack);
	/* LCOV_EXCL_START */
	if (r)
		return r;
	/* LCOV_EXCL_STOP */

	GET_ANTENNA(rx_ant_sets_attrs[ASR_ATTR(0)],
		    step->rx_ant_sets_ranging[0]);
	GET_ANTENNA(rx_ant_sets_attrs[ASR_ATTR(1)],
		    step->rx_ant_sets_ranging[1]);

#undef GET_ANTENNA
#undef STEP_ATTR
#undef ASR_ATTR

	if (step->rx_ant_sets_ranging[0] == -1)
		step->rx_ant_sets_ranging[0] = 0;
	if (step->rx_ant_sets_ranging[1] == -1)
		step->rx_ant_sets_ranging[1] = 0;
	if (step->rx_ant_set_nonranging == -1)
		step->rx_ant_set_nonranging = step->rx_ant_sets_ranging[0];
	if (step->tx_ant_set_ranging == -1)
		step->tx_ant_set_ranging = 0;
	if (step->tx_ant_set_nonranging == -1)
		step->tx_ant_set_nonranging = step->tx_ant_set_ranging;

	return 0;
}

/**
 * fira_session_params_set_measurement_sequence() - Retrieve the measurement
 * schedule from a NL message and store it in the session parameters.
 * @params: The parameters contained in the NL message.
 * @info: NL context info.
 * @meas_seq: The measurement sequence where to store the information.
 *
 * Return: 0 or error.
 */
static int fira_session_params_set_measurement_sequence(
	const struct nlattr *params, const struct genl_info *info,
	struct fira_measurement_sequence *meas_seq)
{
	struct nlattr *request;
	int r, rem = 0;
	size_t n_steps = 0;

	nla_for_each_nested (request, params, rem) {
		if (n_steps >= FIRA_MEASUREMENT_SEQUENCE_STEP_MAX)
			return -EINVAL;
		r = fira_session_params_set_measurement_sequence_step(
			request, info, &meas_seq->steps[n_steps]);
		if (r)
			return r;
		n_steps++;
	}
	if (!n_steps)
		return -EINVAL;
	meas_seq->n_steps = n_steps;
	return 0;
}

/**
 * fira_session_set_parameters() - Set FiRa session parameters.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 * @params: Nested attribute containing session parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_set_parameters(struct fira_local *local, u32 session_id,
				       const struct nlattr *params,
				       const struct genl_info *info)
{
	struct nlattr *attrs[FIRA_SESSION_PARAM_ATTR_MAX + 1];
	struct fira_session *session;
	struct fira_session_params *p;
	struct fira_measurement_sequence meas_seq = {};
	int r;

	if (!params)
		return -EINVAL;

	session = fira_get_session_by_session_id(local, session_id);
	if (!session)
		return -ENOENT;

	r = nla_parse_nested(attrs, FIRA_SESSION_PARAM_ATTR_MAX, params,
			     fira_session_param_nla_policy, info->extack);
	if (r)
		return r;
	/* Check attribute validity. */
	if (attrs[FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE]) {
		r = fira_session_params_set_measurement_sequence(
			attrs[FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE],
			info, &meas_seq);
		if (r)
			return r;
	}
	r = fira_session_fsm_check_parameters(session, attrs);
	if (r)
		return r;

	p = &session->params;
#define P(attr, member, type, conv)                                     \
	do {                                                            \
		int x;                                                  \
		if (attrs[FIRA_SESSION_PARAM_ATTR_##attr]) {            \
			x = nla_get_##type(                             \
				attrs[FIRA_SESSION_PARAM_ATTR_##attr]); \
			p->member = conv;                               \
		}                                                       \
	} while (0)
#define PMEMCPY(attr, member)                                             \
	do {                                                              \
		if (attrs[FIRA_SESSION_PARAM_ATTR_##attr]) {              \
			struct nlattr *attr =                             \
				attrs[FIRA_SESSION_PARAM_ATTR_##attr];    \
			memcpy(p->member, nla_data(attr), nla_len(attr)); \
		}                                                         \
	} while (0)
#define PMEMNCPY(attr, member, size)                                   \
	do {                                                           \
		if (attrs[FIRA_SESSION_PARAM_ATTR_##attr]) {           \
			struct nlattr *attr =                          \
				attrs[FIRA_SESSION_PARAM_ATTR_##attr]; \
			int len = nla_len(attr);                       \
			memcpy(p->member, nla_data(attr), len);        \
			p->size = len;                                 \
		}                                                      \
	} while (0)
	/* Main session parameters. */
	P(DEVICE_TYPE, device_type, u8, x);
	P(RANGING_ROUND_USAGE, ranging_round_usage, u8, x);
	P(MULTI_NODE_MODE, multi_node_mode, u8, x);
	P(SHORT_ADDR, short_addr, u16, x);
	P(DESTINATION_SHORT_ADDR, controller_short_addr, u16, x);
	/* Timings parameters. */
	P(INITIATION_TIME_MS, initiation_time_ms, u32, x);
	P(SLOT_DURATION_RSTU, slot_duration_dtu, u32,
	  x * local->llhw->rstu_dtu);
	P(BLOCK_DURATION_MS, block_duration_dtu, u32,
	  x * (local->llhw->dtu_freq_hz / 1000));
	P(ROUND_DURATION_SLOTS, round_duration_slots, u32, x);
	/* Behaviour parameters. */
	P(BLOCK_STRIDE_LENGTH, block_stride_len, u32, x);
	P(MAX_NUMBER_OF_MEASUREMENTS, max_number_of_measurements, u32, x);
	P(MAX_RR_RETRY, max_rr_retry, u32, x);
	P(ROUND_HOPPING, round_hopping, u8, !!x);
	P(PRIORITY, priority, u8, x);
	P(RESULT_REPORT_PHASE, result_report_phase, u8, !!x);
	/* Radio parameters. */
	P(CHANNEL_NUMBER, channel_number, u8, x);
	P(PREAMBLE_CODE_INDEX, preamble_code_index, u8, x);
	P(RFRAME_CONFIG, rframe_config, u8, x);
	P(PREAMBLE_DURATION, preamble_duration, u8, x);
	P(SFD_ID, sfd_id, u8, x);
	P(NUMBER_OF_STS_SEGMENTS, number_of_sts_segments, u8, x);
	P(PSDU_DATA_RATE, psdu_data_rate, u8, x);
	P(MAC_FCS_TYPE, mac_fcs_type, u8, x);
	P(PRF_MODE, prf_mode, u8, x);
	P(BPRF_PHR_DATA_RATE, phr_data_rate, u8, x);
	/* Measurement Sequence */
	if (attrs[FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE]) {
		p->meas_seq = meas_seq;
		session->measurements.reset = true;
	}
	/* STS and crypto parameters. */
	P(STS_CONFIG, sts_config, u8, x);
	PMEMCPY(VUPPER64, vupper64);
	if (attrs[FIRA_SESSION_PARAM_ATTR_SESSION_KEY]) {
		struct nlattr *attr =
			attrs[FIRA_SESSION_PARAM_ATTR_SESSION_KEY];
		memcpy(p->session_key, nla_data(attr), nla_len(attr));
		p->session_key_len = nla_len(attr);
	}
	P(SUB_SESSION_ID, sub_session_id, u32, x);
	if (attrs[FIRA_SESSION_PARAM_ATTR_SUB_SESSION_KEY]) {
		struct nlattr *attr =
			attrs[FIRA_SESSION_PARAM_ATTR_SUB_SESSION_KEY];
		memcpy(p->sub_session_key, nla_data(attr), nla_len(attr));
		p->sub_session_key_len = nla_len(attr);
	}
	P(KEY_ROTATION, key_rotation, u8, !!x);
	P(KEY_ROTATION_RATE, key_rotation_rate, u8, x);
	/* Report parameters. */
	P(AOA_RESULT_REQ, aoa_result_req, u8, !!x);
	P(REPORT_TOF, report_tof, u8, !!x);
	P(REPORT_AOA_AZIMUTH, report_aoa_azimuth, u8, !!x);
	P(REPORT_AOA_ELEVATION, report_aoa_elevation, u8, !!x);
	P(REPORT_AOA_FOM, report_aoa_fom, u8, !!x);
	P(REPORT_RSSI, report_rssi, u8, x);
	/* Custom data */
	P(DATA_VENDOR_OUI, data_vendor_oui, u32, x);

	PMEMNCPY(DATA_PAYLOAD, data_payload, data_payload_len);
	/* Increment payload sequence number if a new data is received. */
	if (attrs[FIRA_SESSION_PARAM_ATTR_DATA_PAYLOAD])
		p->data_payload_seq++;
	/* Diagnostics */
	P(DIAGNOSTICS, report_diagnostics, u8, x);
	P(DIAGNOSTICS_FRAME_REPORTS_FIELDS, diagnostic_report_flags, u32, x);
	/* Misc */
	P(STS_LENGTH, sts_length, u8, x);
	P(RANGE_DATA_NTF_CONFIG, range_data_ntf_config, u8, x);
	P(RANGE_DATA_NTF_PROXIMITY_NEAR_MM, range_data_ntf_proximity_near_rctu,
						u32, fira_mm_to_rctu(local, x));
	P(RANGE_DATA_NTF_PROXIMITY_FAR_MM, range_data_ntf_proximity_far_rctu,
						u32, fira_mm_to_rctu(local, x));
	P(RANGE_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI,
			range_data_ntf_lower_bound_aoa_azimuth_2pi, s16, x);
	P(RANGE_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI,
			range_data_ntf_upper_bound_aoa_azimuth_2pi, s16, x);
	P(RANGE_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI,
			range_data_ntf_lower_bound_aoa_elevation_2pi, s16, x);
	P(RANGE_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI,
			range_data_ntf_upper_bound_aoa_elevation_2pi, s16, x);
#undef PMEMNCPY
#undef PMEMCPY
#undef P

	fira_session_fsm_parameters_updated(local, session);
	return 0;
}

/**
 * fira_session_get_state() - Get state of the session.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 *
 * Return: 0 or error.
 */
static int fira_session_get_state(struct fira_local *local, u32 session_id)
{
	struct sk_buff *msg;
	enum fira_session_state_id state;

	state = fira_get_state_by_session_id(local, session_id);

	msg = mcps802154_region_call_alloc_reply_skb(
		local->llhw, &local->region, FIRA_CALL_SESSION_GET_STATE,
		NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_ID, session_id))
		goto nla_put_failure;

	if (nla_put_u8(msg, FIRA_CALL_ATTR_SESSION_STATE, state))
		goto nla_put_failure;

	return mcps802154_region_call_reply(local->llhw, msg);

nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

/**
 * fira_session_params_get_measurement_sequence_step() - Retrieve a
 * measurement sequence step in a NL message.
 * @step: The measurement sequence step to add to the message.
 * @msg_buf: NL message buffer.
 * @idx: Index of the step.
 *
 * Return: 0 or error.
 */
static int fira_session_params_get_measurement_sequence_step(
	const struct fira_measurement_sequence_step *step,
	struct sk_buff *msg_buf, size_t idx)
{
#define P(attr, member, type, conv)                      \
	do {                                             \
		type x = member;                         \
		if (nla_put_##type(msg_buf, attr, conv)) \
			return -ENOBUFS;                 \
	} while (0)
#define STEP_ATTR(x) FIRA_SESSION_PARAM_MEAS_SEQ_STEP_ATTR_##x
#define ASR_ATTR(x) \
	FIRA_SESSION_PARAM_MEAS_SEQ_STEP_RX_ANT_SETS_RANGING_ATTR_##x

	struct nlattr *step_params = NULL;
	struct nlattr *rx_ant_sets_ranging_params = NULL;

	step_params = nla_nest_start(msg_buf, idx);

	if (!step_params)
		return -ENOBUFS;
	P(STEP_ATTR(MEASUREMENT_TYPE), step->type, u8, x);
	P(STEP_ATTR(N_MEASUREMENTS), step->n_measurements, u8, x);
	P(STEP_ATTR(RX_ANT_SET_NONRANGING), step->rx_ant_set_nonranging, u8, x);

	rx_ant_sets_ranging_params =
		nla_nest_start(msg_buf, STEP_ATTR(RX_ANT_SETS_RANGING));
	if (!rx_ant_sets_ranging_params)
		return -ENOBUFS;

	P(ASR_ATTR(0), step->rx_ant_sets_ranging[0], u8, x);
	P(ASR_ATTR(1), step->rx_ant_sets_ranging[1], u8, x);

	nla_nest_end(msg_buf, rx_ant_sets_ranging_params);

	P(STEP_ATTR(TX_ANT_SET_NONRANGING), step->tx_ant_set_nonranging, u8, x);
	P(STEP_ATTR(TX_ANT_SET_RANGING), step->tx_ant_set_ranging, u8, x);

	nla_nest_end(msg_buf, step_params);

#undef STEP_ATTR
#undef ASR_ATTR
#undef P
	return 0;
}

/**
 * fira_session_params_get_measurement_sequence() - Retrieve a
 * measurement sequence in a NL message.
 * @meas_seq: The measurement sequence to add to the message.
 * @msg_buf: NL message buffer.
 *
 * Return: 0 or error.
 */
static int fira_session_params_get_measurement_sequence(
	const struct fira_measurement_sequence *meas_seq,
	struct sk_buff *msg_buf)
{
	struct nlattr *meas_seq_params = NULL;
	size_t i;

	meas_seq_params = nla_nest_start(
		msg_buf, FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE);
	if (!meas_seq_params)
		return -ENOBUFS;

	for (i = 0; i < meas_seq->n_steps; ++i) {
		int r = 0;
		r = fira_session_params_get_measurement_sequence_step(
			meas_seq->steps + i, msg_buf, i);
		if (r)
			return r;
	}

	nla_nest_end(msg_buf, meas_seq_params);

	return 0;
}

/**
 * fira_session_get_parameters() - Get FiRa session parameters.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 *
 * Return: 0 or error.
 */
static int fira_session_get_parameters(struct fira_local *local, u32 session_id)
{
	const struct fira_session *session;
	const struct fira_session_params *p;
	struct sk_buff *msg;
	struct nlattr *params;

	session = fira_get_session_by_session_id(local, session_id);
	if (!session)
		return -ENOENT;

	p = &session->params;
	msg = mcps802154_region_call_alloc_reply_skb(
		local->llhw, &local->region, FIRA_CALL_SESSION_GET_PARAMS,
		NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_ID, session->id))
		goto nla_put_failure;

	params = nla_nest_start(msg, FIRA_CALL_ATTR_SESSION_PARAMS);
	if (!params)
		goto nla_put_failure;

#define P(attr, member, type, conv)                                            \
	do {                                                                   \
		type x = p->member;                                            \
		if (nla_put_##type(msg, FIRA_SESSION_PARAM_ATTR_##attr, conv)) \
			goto nla_put_failure;                                  \
	} while (0)
#define PMEMCPY(attr, member)                                    \
	do {                                                     \
		if (nla_put(msg, FIRA_SESSION_PARAM_ATTR_##attr, \
			    sizeof(p->member), p->member))       \
			goto nla_put_failure;                    \
	} while (0)
	/* Main session parameters. */
	P(DEVICE_TYPE, device_type, u8, x);
	P(RANGING_ROUND_USAGE, ranging_round_usage, u8, x);
	P(MULTI_NODE_MODE, multi_node_mode, u8, x);
	if (p->short_addr != IEEE802154_ADDR_SHORT_BROADCAST)
		P(SHORT_ADDR, short_addr, u16, x);
	if (p->controller_short_addr != IEEE802154_ADDR_SHORT_BROADCAST)
		P(DESTINATION_SHORT_ADDR, controller_short_addr, u16, x);
	/* Timings parameters. */
	P(INITIATION_TIME_MS, initiation_time_ms, u32, x);
	P(SLOT_DURATION_RSTU, slot_duration_dtu, u32,
	  x / local->llhw->rstu_dtu);
	P(BLOCK_DURATION_MS, block_duration_dtu, u32,
	  x / (local->llhw->dtu_freq_hz / 1000));
	P(ROUND_DURATION_SLOTS, round_duration_slots, u32, x);
	/* Behaviour parameters. */
	P(BLOCK_STRIDE_LENGTH, block_stride_len, u32, x);
	P(MAX_NUMBER_OF_MEASUREMENTS, max_number_of_measurements, u32, x);
	P(MAX_RR_RETRY, max_rr_retry, u32, x);
	P(ROUND_HOPPING, round_hopping, u8, !!x);
	P(PRIORITY, priority, u8, x);
	P(RESULT_REPORT_PHASE, result_report_phase, u8, !!x);
	/* Radio parameters. */
	if (p->channel_number)
		P(CHANNEL_NUMBER, channel_number, u8, x);
	if (p->preamble_code_index)
		P(PREAMBLE_CODE_INDEX, preamble_code_index, u8, x);
	P(RFRAME_CONFIG, rframe_config, u8, x);
	P(PREAMBLE_DURATION, preamble_duration, u8, x);
	P(SFD_ID, sfd_id, u8, x);
	P(NUMBER_OF_STS_SEGMENTS, number_of_sts_segments, u8, x);
	P(PSDU_DATA_RATE, psdu_data_rate, u8, x);
	P(PRF_MODE, prf_mode, u8, x);
	P(BPRF_PHR_DATA_RATE, phr_data_rate, u8, x);
	P(MAC_FCS_TYPE, mac_fcs_type, u8, x);
	/* Measurement Sequence */
	if (fira_session_params_get_measurement_sequence(
		    &session->measurements.sequence, msg))
		goto nla_put_failure;
	/* STS and crypto parameters. */
	PMEMCPY(VUPPER64, vupper64);
	P(SUB_SESSION_ID, sub_session_id, u32, x);
	P(STS_CONFIG, sts_config, u8, x);
	P(KEY_ROTATION, key_rotation, u8, x);
	P(KEY_ROTATION_RATE, key_rotation_rate, u8, x);
	/* Report parameters. */
	P(AOA_RESULT_REQ, aoa_result_req, u8, !!x);
	P(REPORT_TOF, report_tof, u8, !!x);
	P(REPORT_AOA_AZIMUTH, report_aoa_azimuth, u8, !!x);
	P(REPORT_AOA_ELEVATION, report_aoa_elevation, u8, !!x);
	P(REPORT_AOA_FOM, report_aoa_fom, u8, !!x);
	P(REPORT_RSSI, report_rssi, u8, !!x);
	/* Custom data */
	if (p->data_vendor_oui)
		P(DATA_VENDOR_OUI, data_vendor_oui, u32, x);
	/* Diagnostics */
	P(DIAGNOSTICS, report_diagnostics, u8, x);
	P(DIAGNOSTICS_FRAME_REPORTS_FIELDS, diagnostic_report_flags, u32, x);
	/* Misc */
	P(STS_LENGTH, sts_length, u8, x);
	P(RANGE_DATA_NTF_CONFIG, range_data_ntf_config, u8, x);
	P(RANGE_DATA_NTF_PROXIMITY_NEAR_MM, range_data_ntf_proximity_near_rctu,
	  u32, fira_rctu_to_mm((s64)local->llhw->dtu_freq_hz * local->llhw->dtu_rctu, x));
	P(RANGE_DATA_NTF_PROXIMITY_FAR_MM, range_data_ntf_proximity_far_rctu,
	  u32, fira_rctu_to_mm((s64)local->llhw->dtu_freq_hz * local->llhw->dtu_rctu, x));
	P(RANGE_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI,
	  range_data_ntf_lower_bound_aoa_azimuth_2pi, s16, x);
	P(RANGE_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI,
	  range_data_ntf_upper_bound_aoa_azimuth_2pi, s16, x);
	P(RANGE_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI,
	  range_data_ntf_lower_bound_aoa_elevation_2pi, s16, x);
	P(RANGE_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI,
	  range_data_ntf_upper_bound_aoa_elevation_2pi, s16, x);
#undef P
#undef PMEMCPY

	nla_nest_end(msg, params);

	return mcps802154_region_call_reply(local->llhw, msg);
nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

/**
 * fira_manage_controlees() - Manage controlees.
 * @local: FiRa context.
 * @call_id: FiRa call id.
 * @session_id: FiRa session id.
 * @params: Nested attribute containing controlee parameters.
 * @info: Request information.
 * Return: 0 or error.
 */
static int fira_manage_controlees(struct fira_local *local,
				  enum fira_call call_id, u32 session_id,
				  const struct nlattr *params,
				  const struct genl_info *info)
{
	static const struct nla_policy new_controlee_nla_policy[FIRA_CALL_CONTROLEE_ATTR_MAX +
								1] = {
		[FIRA_CALL_CONTROLEE_ATTR_SHORT_ADDR] = { .type = NLA_U16 },
		[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID] = { .type = NLA_U32 },
		[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY] = { .type = NLA_BINARY,
							       .len = FIRA_KEY_SIZE_MAX },
	};
	struct nlattr *request;
	struct nlattr *attrs[FIRA_CALL_CONTROLEE_ATTR_MAX + 1];
	int r, rem, i, slot_duration_us, n_controlees = 0;
	struct fira_session *session;
	struct fira_controlee *controlee = NULL, *tmp_controlee;
	bool is_active;
	struct list_head controlees;

	if (!params)
		return -EINVAL;

	session = fira_get_session_by_session_id(local, session_id);
	if (!session)
		return -ENOENT;

	INIT_LIST_HEAD(&controlees);

	nla_for_each_nested (request, params, rem) {
		if (n_controlees >= FIRA_CONTROLEES_MAX) {
			r = -EINVAL;
			goto end;
		}

		r = nla_parse_nested(attrs, FIRA_CALL_CONTROLEE_ATTR_MAX,
				     request, new_controlee_nla_policy,
				     info->extack);
		if (r)
			goto end;

		controlee = kzalloc(sizeof(struct fira_controlee), GFP_KERNEL);
		if (!controlee) {
			r = -ENOMEM;
			goto end;
		}

		if (!attrs[FIRA_CALL_CONTROLEE_ATTR_SHORT_ADDR] ||
		    (!attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID] &&
		     attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY])) {
			kfree(controlee);
			r = -EINVAL;
			goto end;
		}

		controlee->short_addr = nla_get_le16(
			attrs[FIRA_CALL_CONTROLEE_ATTR_SHORT_ADDR]);
		if (attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID]) {
			if (call_id == FIRA_CALL_DEL_CONTROLEE) {
				kfree(controlee);
				r = -EINVAL;
				goto end;
			}
			controlee->sub_session = true;
			controlee->sub_session_id = nla_get_u32(
				attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID]);
			if (attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY]) {
				memcpy(controlee->sub_session_key,
				       nla_data(
					       attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY]),
				       nla_len(attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY]));
				controlee->sub_session_key_len = nla_len(
					attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY]);
			}
		} else {
			controlee->sub_session = false;
		}
		controlee->range_data_ntf_status = FIRA_RANGE_DATA_NTF_NONE;
		controlee->state = FIRA_CONTROLEE_STATE_RUNNING;
		/* Check and reject a duplication of short_addr. */
		list_for_each_entry (tmp_controlee, &controlees, entry) {
			if (controlee->short_addr ==
			    tmp_controlee->short_addr) {
				kfree(controlee);
				r = -EINVAL;
				goto end;
			}
		}
		list_add_tail(&controlee->entry, &controlees);
		n_controlees++;
	}
	if (list_empty(&controlees)) {
		r = -EINVAL;
		goto end;
	}

	/*
	 * Be careful, active is not equal to
	 * 'local->current_session == session'.
	 */
	is_active = fira_session_is_active(session);

	if (is_active) {
		/* If unicast refuse to add more than one controlee. */
		switch (call_id) {
		case FIRA_CALL_NEW_CONTROLEE:
			if (session->params.multi_node_mode ==
				    FIRA_MULTI_NODE_MODE_UNICAST) {
				r = -EPERM;
				goto end;
			}
			break;
		case FIRA_CALL_SET_CONTROLEE:
			r = -EBUSY;
			goto end;
		default:
			break;
		}
	}

	slot_duration_us = (session->params.slot_duration_dtu * 1000) /
						(local->llhw->dtu_freq_hz / 1000);

	switch (call_id) {
	case FIRA_CALL_SET_CONTROLEE:
		r = fira_session_set_controlees(local, session, &controlees,
						slot_duration_us, n_controlees);
		break;
	case FIRA_CALL_DEL_CONTROLEE:
		if (n_controlees > 1) {
			r = -EINVAL;
			goto end;
		} else {
			/* 'controlee' points the unique entry in the list. */
			r = fira_session_del_controlee(session, controlee,
					is_active);
		}
		break;
	/* FIRA_CALL_NEW_CONTROLEE. */
	default:
		if (n_controlees > 1) {
			r = -EINVAL;
			goto end;
		} else {
			/* 'controlee' points the unique entry in the list. */
			r = fira_session_new_controlee(local, session,
							   controlee,
							   slot_duration_us,
							   is_active);
		}
	}
	if (r)
		goto end;

	if (!is_active && local->llhw->rx_ctx_size) {
		for (i = 0; i < session->n_current_controlees; i++) {
			memset(session->rx_ctx[i], 0, local->llhw->rx_ctx_size);
		}
	}

	fira_session_fsm_controlee_list_updated(local, session);
end:
	list_for_each_entry_safe (controlee, tmp_controlee, &controlees,
				  entry) {
		kfree(controlee);
	}
	return r;
}

/**
 * fira_session_get_controlees() - Get list of controlees.
 * @local: FiRa context.
 * @session_id: FiRa session id.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_get_controlees(struct fira_local *local, u32 session_id,
				       const struct genl_info *info)
{
	const struct fira_session *session;
	struct sk_buff *msg;
	struct nlattr *controlees, *controlee_attr;
	struct fira_controlee *controlee;

	session = fira_get_session_by_session_id(local, session_id);
	if (!session)
		return -ENOENT;

	msg = mcps802154_region_call_alloc_reply_skb(local->llhw,
						     &local->region,
						     FIRA_CALL_GET_CONTROLEES,
						     NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_ID, session->id))
		goto nla_put_failure;

	controlees = nla_nest_start(msg, FIRA_CALL_ATTR_CONTROLEES);
	if (!controlees)
		goto nla_put_failure;

	list_for_each_entry (controlee, &session->current_controlees, entry) {
		controlee_attr = nla_nest_start(msg, 1);
		if (!controlee_attr)
			goto nla_put_failure;
		if (nla_put_le16(msg, FIRA_CALL_CONTROLEE_ATTR_SHORT_ADDR,
				 controlee->short_addr))
			goto nla_put_failure;
		if (controlee->sub_session &&
		    nla_put_u32(msg, FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID,
				controlee->sub_session_id))
			goto nla_put_failure;
		nla_nest_end(msg, controlee_attr);
	}

	nla_nest_end(msg, controlees);

	return mcps802154_region_call_reply(local->llhw, msg);
nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

int fira_get_capabilities(struct fira_local *local,
			  const struct genl_info *info)
{
	struct sk_buff *msg;
	struct nlattr *capabilities;
	u64 hw_flags = local->llhw->flags;
	u32 sts_caps = fira_crypto_get_capabilities();

	if (!info)
		return 0;

	msg = mcps802154_region_call_alloc_reply_skb(local->llhw,
						     &local->region,
						     FIRA_CALL_GET_CAPABILITIES,
						     NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	capabilities = nla_nest_start(msg, FIRA_CALL_ATTR_CAPABILITIES);
	if (!capabilities) {
		goto nla_put_failure;
	}

#define P(name, type, value)                                                   \
	do {                                                                   \
		if (nla_put_##type(msg, FIRA_CAPABILITY_ATTR_##name, value)) { \
			goto nla_put_failure;                                  \
		}                                                              \
	} while (0)
#define F(name)                                                       \
	do {                                                          \
		if (nla_put_flag(msg, FIRA_CAPABILITY_ATTR_##name)) { \
			goto nla_put_failure;                         \
		}                                                     \
	} while (0)
#define C(name, hw_flag)                  \
	do {                              \
		if (hw_flags & (hw_flag)) \
			F(name);          \
	} while (0)
#define S(mode)                                             \
	do {                                                \
		if (sts_caps & (1 << FIRA_STS_MODE_##mode)) \
			F(STS_##mode);                      \
	} while (0)

	/* Main session capabilities. */
	P(FIRA_PHY_VERSION_RANGE, u32, 0x01010101);
	P(FIRA_MAC_VERSION_RANGE, u32, 0x01010101);
	P(DEVICE_CLASS, u8, 1);
	F(DEVICE_TYPE_CONTROLEE_RESPONDER);
	F(DEVICE_TYPE_CONTROLLER_INITIATOR);
	F(MULTI_NODE_MODE_UNICAST);
	F(MULTI_NODE_MODE_ONE_TO_MANY);
	F(RANGING_ROUND_USAGE_DS_TWR);
	P(NUMBER_OF_CONTROLEES_MAX, u32, FIRA_CONTROLEES_MAX);
	/* Behaviour. */
	F(ROUND_HOPPING);
	F(BLOCK_STRIDING);
	/* Radio. */
	P(CHANNEL_NUMBER, u16,
	  local->llhw->hw->phy->supported
		  .channels[local->llhw->hw->phy->current_page]);
	C(RFRAME_CONFIG_SP1,
	  MCPS802154_LLHW_STS_SEGMENT_1 | MCPS802154_LLHW_STS_SEGMENT_2);
	C(RFRAME_CONFIG_SP3,
	  MCPS802154_LLHW_STS_SEGMENT_1 | MCPS802154_LLHW_STS_SEGMENT_2);
	C(PRF_MODE_BPRF, MCPS802154_LLHW_BPRF);
	C(PRF_MODE_HPRF, MCPS802154_LLHW_HPRF);
	C(PREAMBLE_DURATION_32, MCPS802154_LLHW_PSR_32);
	C(PREAMBLE_DURATION_64, MCPS802154_LLHW_PSR_64);
	C(SFD_ID_0, MCPS802154_LLHW_SFD_4A);
	C(SFD_ID_1, MCPS802154_LLHW_SFD_4Z_4);
	C(SFD_ID_2, MCPS802154_LLHW_SFD_4Z_8);
	C(SFD_ID_3, MCPS802154_LLHW_SFD_4Z_16);
	C(SFD_ID_4, MCPS802154_LLHW_SFD_4Z_32);
	F(NUMBER_OF_STS_SEGMENTS_0);
	C(NUMBER_OF_STS_SEGMENTS_1, MCPS802154_LLHW_STS_SEGMENT_1);
	C(NUMBER_OF_STS_SEGMENTS_2, MCPS802154_LLHW_STS_SEGMENT_2);
	C(NUMBER_OF_STS_SEGMENTS_3, MCPS802154_LLHW_STS_SEGMENT_3);
	C(NUMBER_OF_STS_SEGMENTS_4, MCPS802154_LLHW_STS_SEGMENT_4);
	C(PSDU_DATA_RATE_6M81, MCPS802154_LLHW_DATA_RATE_6M81);
	C(PSDU_DATA_RATE_7M80, MCPS802154_LLHW_DATA_RATE_7M80);
	C(PSDU_DATA_RATE_27M2, MCPS802154_LLHW_DATA_RATE_27M2);
	C(PSDU_DATA_RATE_31M2, MCPS802154_LLHW_DATA_RATE_31M2);
	C(BPRF_PHR_DATA_RATE_850K, MCPS802154_LLHW_PHR_DATA_RATE_850K);
	C(BPRF_PHR_DATA_RATE_6M81, MCPS802154_LLHW_PHR_DATA_RATE_6M81);
	F(TX_ADAPTIVE_PAYLOAD_POWER);
	/* Antenna. */
	P(RX_ANTENNA_PAIRS, u32, local->llhw->rx_antenna_pairs);
	P(TX_ANTENNAS, u32, local->llhw->tx_antennas);
	/* STS and crypto capabilities. */
	S(STATIC);
	S(DYNAMIC);
	S(DYNAMIC_INDIVIDUAL_KEY);
	S(PROVISIONED);
	S(PROVISIONED_INDIVIDUAL_KEY);
	/* Report. */
	C(AOA_AZIMUTH, MCPS802154_LLHW_AOA_AZIMUTH);
	C(AOA_AZIMUTH_FULL, MCPS802154_LLHW_AOA_AZIMUTH_FULL);
	C(AOA_ELEVATION, MCPS802154_LLHW_AOA_ELEVATION);
	C(AOA_FOM, MCPS802154_LLHW_AOA_FOM);
#undef C
#undef F
#undef P

	nla_nest_end(msg, capabilities);
	return mcps802154_region_call_reply(local->llhw, msg);
nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

int fira_session_get_count(struct fira_local *local)
{
	struct fira_session *session;
	struct sk_buff *msg;
	u32 count = 0;

	list_for_each_entry (session, &local->inactive_sessions, entry) {
		count++;
	}

	list_for_each_entry (session, &local->active_sessions, entry) {
		count++;
	}

	msg = mcps802154_region_call_alloc_reply_skb(
		local->llhw, &local->region, FIRA_CALL_SESSION_GET_COUNT,
		NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_COUNT, count))
		goto nla_put_failure;

	return mcps802154_region_call_reply(local->llhw, msg);

nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

int fira_session_control(struct fira_local *local, enum fira_call call_id,
			 const struct nlattr *params,
			 const struct genl_info *info)
{
	u32 session_id;
	struct nlattr *attrs[FIRA_CALL_ATTR_MAX + 1];
	int r;

	/* Special case of get count that doesn't need params. */
	if (call_id == FIRA_CALL_SESSION_GET_COUNT)
		return fira_session_get_count(local);

	if (!params)
		return -EINVAL;
	r = nla_parse_nested(attrs, FIRA_CALL_ATTR_MAX, params,
			     fira_call_nla_policy, info->extack);

	if (r)
		return r;

	if (!attrs[FIRA_CALL_ATTR_SESSION_ID])
		return -EINVAL;

	session_id = nla_get_u32(attrs[FIRA_CALL_ATTR_SESSION_ID]);
	trace_region_fira_session_control(local, session_id, call_id);

	switch (call_id) {
	case FIRA_CALL_SESSION_INIT:
		return fira_session_init(local, session_id);
	case FIRA_CALL_SESSION_START:
		return fira_session_start(local, session_id, info);
	case FIRA_CALL_SESSION_STOP:
		return fira_session_stop(local, session_id);
	case FIRA_CALL_SESSION_DEINIT:
		return fira_session_deinit(local, session_id);
	case FIRA_CALL_SESSION_SET_PARAMS:
		return fira_session_set_parameters(
			local, session_id, attrs[FIRA_CALL_ATTR_SESSION_PARAMS],
			info);
	case FIRA_CALL_SET_CONTROLEE:
	case FIRA_CALL_NEW_CONTROLEE:
	case FIRA_CALL_DEL_CONTROLEE:
		return fira_manage_controlees(local, call_id, session_id,
					      attrs[FIRA_CALL_ATTR_CONTROLEES],
					      info);
	case FIRA_CALL_GET_CONTROLEES:
		return fira_session_get_controlees(local, session_id, info);
	case FIRA_CALL_SESSION_GET_PARAMS:
		return fira_session_get_parameters(local, session_id);
	case FIRA_CALL_SESSION_GET_STATE:
		return fira_session_get_state(local, session_id);
	default:
		return -EINVAL;
	}
}
