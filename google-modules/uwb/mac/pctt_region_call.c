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

#include <linux/errno.h>
#include <linux/ieee802154.h>

#include <net/pctt_region_nl.h>
#include <net/mcps802154_frame.h>

#include "pctt_access.h"
#include "pctt_region.h"
#include "pctt_region_call.h"
#include "pctt_session.h"
#include "pctt_trace.h"

static const struct nla_policy pctt_call_nla_policy[PCTT_CALL_ATTR_MAX + 1] = {
	[PCTT_CALL_ATTR_CMD_ID] = { .type = NLA_U8 },
	[PCTT_CALL_ATTR_RESULT_DATA] = { .type = NLA_NESTED },
	[PCTT_CALL_ATTR_SESSION_ID] = { .type = NLA_U32 },
	[PCTT_CALL_ATTR_SESSION_STATE] = { .type = NLA_U8 },
	[PCTT_CALL_ATTR_SESSION_PARAMS] = { .type = NLA_NESTED },
};

static const struct nla_policy pctt_session_param_nla_policy[PCTT_SESSION_PARAM_ATTR_MAX +
							     1] = {
	[PCTT_SESSION_PARAM_ATTR_DEVICE_ROLE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_DEVICE_ROLE_INITIATOR,
	},
	[PCTT_SESSION_PARAM_ATTR_SHORT_ADDR] = { .type = NLA_U16 },
	[PCTT_SESSION_PARAM_ATTR_DESTINATION_SHORT_ADDR] = { .type = NLA_U16 },
	[PCTT_SESSION_PARAM_ATTR_SLOT_DURATION_RSTU] = { .type = NLA_U32 },
	[PCTT_SESSION_PARAM_ATTR_RX_ANTENNA_SELECTION] = { .type = NLA_U8 },
	[PCTT_SESSION_PARAM_ATTR_TX_ANTENNA_SELECTION] = { .type = NLA_U8 },
	[PCTT_SESSION_PARAM_ATTR_CHANNEL_NUMBER] = { .type = NLA_U8 },
	[PCTT_SESSION_PARAM_ATTR_PREAMBLE_CODE_INDEX] = { .type = NLA_U8 },
	[PCTT_SESSION_PARAM_ATTR_RFRAME_CONFIG] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_RFRAME_CONFIG_SP3,
	},
	[PCTT_SESSION_PARAM_ATTR_PRF_MODE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_PRF_MODE_HPRF_HIGH_RATE,
	},
	[PCTT_SESSION_PARAM_ATTR_PREAMBLE_DURATION] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_PREAMBLE_DURATION_64,
	},
	[PCTT_SESSION_PARAM_ATTR_SFD_ID] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_SFD_ID_4,
	},
	[PCTT_SESSION_PARAM_ATTR_NUMBER_OF_STS_SEGMENTS] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_NUMBER_OF_STS_SEGMENTS_4_SEGMENTS,
	},
	[PCTT_SESSION_PARAM_ATTR_PSDU_DATA_RATE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_PSDU_DATA_RATE_31M2,
	},
	[PCTT_SESSION_PARAM_ATTR_BPRF_PHR_DATA_RATE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_PHR_DATA_RATE_6M81,
	},
	[PCTT_SESSION_PARAM_ATTR_MAC_FCS_TYPE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_MAC_FCS_TYPE_CRC_32,
	},
	[PCTT_SESSION_PARAM_ATTR_TX_ADAPTIVE_PAYLOAD_POWER] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_BOOLEAN_MAX,
	},
	[PCTT_SESSION_PARAM_ATTR_STS_INDEX] = { .type = NLA_U32 },
	[PCTT_SESSION_PARAM_ATTR_STS_LENGTH] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = PCTT_STS_LENGTH_128,
	},
	[PCTT_SESSION_PARAM_ATTR_NUM_PACKETS] = { .type = NLA_U32 },
	[PCTT_SESSION_PARAM_ATTR_T_GAP] = { .type = NLA_U32 },
	[PCTT_SESSION_PARAM_ATTR_T_START] = { .type = NLA_U32 },
	[PCTT_SESSION_PARAM_ATTR_T_WIN] = { .type = NLA_U32 },
	[PCTT_SESSION_PARAM_ATTR_RANDOMIZE_PSDU] = { .type = NLA_U8 },
	[PCTT_SESSION_PARAM_ATTR_PHR_RANGING_BIT] = { .type = NLA_U8 },
	[PCTT_SESSION_PARAM_ATTR_RMARKER_TX_START] = { .type = NLA_U32 },
	[PCTT_SESSION_PARAM_ATTR_RMARKER_RX_START] = { .type = NLA_U32 },
	[PCTT_SESSION_PARAM_ATTR_STS_INDEX_AUTO_INCR] = { .type = NLA_U8 },
	[PCTT_SESSION_PARAM_ATTR_DATA_PAYLOAD] = {
		.type = NLA_BINARY,
		.len = PCTT_PAYLOAD_MAX_LEN
	},
};

int pctt_call_session_get_state(struct pctt_local *local)
{
	struct pctt_session *session = &local->session;
	struct sk_buff *msg;

	msg = mcps802154_region_call_alloc_reply_skb(
		local->llhw, &local->region, PCTT_CALL_SESSION_GET_STATE,
		NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, PCTT_CALL_ATTR_SESSION_ID, PCTT_SESSION_ID))
		goto nla_put_failure;

	if (nla_put_u8(msg, PCTT_CALL_ATTR_SESSION_STATE, session->state))
		goto nla_put_failure;

	return mcps802154_region_call_reply(local->llhw, msg);

nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

int pctt_call_session_get_params(struct pctt_local *local)
{
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	struct nlattr *params;
	struct sk_buff *msg;

	msg = mcps802154_region_call_alloc_reply_skb(
		local->llhw, &local->region, PCTT_CALL_SESSION_GET_PARAMS,
		NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, PCTT_CALL_ATTR_SESSION_ID, PCTT_SESSION_ID))
		goto nla_put_failure;

	params = nla_nest_start(msg, PCTT_CALL_ATTR_SESSION_PARAMS);
	if (!params)
		goto nla_put_failure;

#define P(attr, member, type, conv)                                            \
	do {                                                                   \
		type x = p->member;                                            \
		if (nla_put_##type(msg, PCTT_SESSION_PARAM_ATTR_##attr, conv)) \
			goto nla_put_failure;                                  \
	} while (0)
	P(DEVICE_ROLE, device_role, u8, x);
	P(SHORT_ADDR, short_addr, u16, x);
	P(DESTINATION_SHORT_ADDR, dst_short_addr, u16, x);
	P(RX_ANTENNA_SELECTION, rx_antenna_selection, u8, x);
	P(TX_ANTENNA_SELECTION, tx_antenna_selection, u8, x);
	P(SLOT_DURATION_RSTU, slot_duration_dtu, u32,
	  x / local->llhw->rstu_dtu);
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
	P(TX_ADAPTIVE_PAYLOAD_POWER, tx_adaptive_payload_power, u8, x);
	P(STS_INDEX, sts_index, u32, x);
	P(STS_LENGTH, sts_length, u8, x);
	P(NUM_PACKETS, num_packets, u32, x);
	P(T_GAP, gap_duration_dtu, u32,
	  (((u64)x * 1000) / (local->llhw->dtu_freq_hz / 1000)));
	P(T_START, t_start, u32, x);
	P(T_WIN, t_win, u32, x);
	P(RANDOMIZE_PSDU, randomize_psdu, u8, x);
	P(PHR_RANGING_BIT, phr_ranging_bit, u8, x);
	P(RMARKER_TX_START, rmarker_tx_start, u32, x);
	P(RMARKER_RX_START, rmarker_rx_start, u32, x);
	P(STS_INDEX_AUTO_INCR, sts_index_auto_incr, u8, x);
#undef P
	nla_nest_end(msg, params);

	return mcps802154_region_call_reply(local->llhw, msg);
nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

static int pctt_call_session_set_params(struct pctt_local *local,
					const struct nlattr *params,
					const struct genl_info *info)
{
	struct nlattr *attrs[PCTT_SESSION_PARAM_ATTR_MAX + 1];
	struct pctt_session *session = &local->session;
	struct pctt_session_params *p = &session->params;
	int r;

	if (!params)
		return -EINVAL;
	if (session->test_on_going)
		return -EBUSY;

	r = nla_parse_nested(attrs, PCTT_SESSION_PARAM_ATTR_MAX, params,
			     pctt_session_param_nla_policy, info->extack);
	if (r)
		return r;

#define P(attr, member, type, conv)                                     \
	do {                                                            \
		int x;                                                  \
		if (attrs[PCTT_SESSION_PARAM_ATTR_##attr]) {            \
			x = nla_get_##type(                             \
				attrs[PCTT_SESSION_PARAM_ATTR_##attr]); \
			p->member = conv;                               \
		}                                                       \
	} while (0)
#define PMEMNCPY(attr, member, size)                                   \
	do {                                                           \
		if (attrs[PCTT_SESSION_PARAM_ATTR_##attr]) {           \
			struct nlattr *attr =                          \
				attrs[PCTT_SESSION_PARAM_ATTR_##attr]; \
			int len = nla_len(attr);                       \
			memcpy(p->member, nla_data(attr), len);        \
			p->size = len;                                 \
		}                                                      \
	} while (0)

	P(DEVICE_ROLE, device_role, u8, x);
	P(SHORT_ADDR, short_addr, u16, x);
	P(DESTINATION_SHORT_ADDR, dst_short_addr, u16, x);
	P(RX_ANTENNA_SELECTION, rx_antenna_selection, u8, x);
	P(TX_ANTENNA_SELECTION, tx_antenna_selection, u8, x);
	P(SLOT_DURATION_RSTU, slot_duration_dtu, u32,
	  x * local->llhw->rstu_dtu);
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
	P(TX_ADAPTIVE_PAYLOAD_POWER, tx_adaptive_payload_power, u8, x);
	P(STS_INDEX, sts_index, u32, x);
	P(STS_LENGTH, sts_length, u8, x);
	P(NUM_PACKETS, num_packets, u32, x);
	P(T_GAP, gap_duration_dtu, u32,
	  ((u64)x * (local->llhw->dtu_freq_hz / 1000)) / 1000);
	P(T_START, t_start, u32, x);
	P(T_WIN, t_win, u32, x);
	P(RANDOMIZE_PSDU, randomize_psdu, u8, x);
	P(PHR_RANGING_BIT, phr_ranging_bit, u8, x);
	P(RMARKER_TX_START, rmarker_tx_start, u32, x);
	P(RMARKER_RX_START, rmarker_rx_start, u32, x);
	P(STS_INDEX_AUTO_INCR, sts_index_auto_incr, u8, x);
	PMEMNCPY(DATA_PAYLOAD, data_payload, data_payload_len);
#undef PMEMNCPY
#undef P

	return 0;
}

static int pctt_call_cmd(struct pctt_local *local,
			 const struct nlattr *cmd_id_attr,
			 const struct genl_info *info)
{
	struct pctt_session *session = &local->session;
	enum pctt_id_attrs cmd_id;

	if (!cmd_id_attr)
		return -EINVAL;
	cmd_id = nla_get_u8(cmd_id_attr);

	if (session->test_on_going) {
		if (cmd_id == PCTT_ID_ATTR_STOP_TEST &&
		    !session->stop_request) {
			session->stop_request = true;
			mcps802154_reschedule(local->llhw);
			return 0;
		}
		return -EBUSY;
	} else {
		u32 now_dtu;
		int r;

		if (cmd_id == PCTT_ID_ATTR_STOP_TEST)
			return 0;

		/* FIXME: Used only to detect dw3000_is_active. */
		r = mcps802154_get_current_timestamp_dtu(local->llhw, &now_dtu);
		if (r)
			return r;
		r = pctt_session_start_test(local, cmd_id, info);
		if (r)
			return r;
	}

	mcps802154_reschedule(local->llhw);
	return 0;
}

int pctt_call_session_control(struct pctt_local *local, enum pctt_call call_id,
			      const struct nlattr *params,
			      const struct genl_info *info)
{
	struct pctt_session *session = &local->session;
	struct nlattr *attrs[PCTT_CALL_ATTR_MAX + 1];
	int r;

	if (session->state == PCTT_SESSION_STATE_DEINIT)
		return -EPERM;
	if (!params || !info)
		return -EINVAL;
	r = nla_parse_nested(attrs, PCTT_CALL_ATTR_MAX, params,
			     pctt_call_nla_policy, info->extack);
	if (r)
		return r;

	if (call_id == PCTT_CALL_SESSION_SET_PARAMS)
		return pctt_call_session_set_params(
			local, attrs[PCTT_CALL_ATTR_SESSION_PARAMS], info);
	else
		return pctt_call_cmd(local, attrs[PCTT_CALL_ATTR_CMD_ID], info);
}
