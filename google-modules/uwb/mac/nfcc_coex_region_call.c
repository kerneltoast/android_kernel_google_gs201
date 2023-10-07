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

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/limits.h>

#include <net/nfcc_coex_region_nl.h>
#include <net/mcps802154_frame.h>

#include "mcps802154_qorvo.h"
#include "nfcc_coex_session.h"
#include "nfcc_coex_region_call.h"
#include "nfcc_coex_trace.h"

static const struct nla_policy nfcc_coex_call_nla_policy[NFCC_COEX_CALL_ATTR_MAX +
							 1] = {
	[NFCC_COEX_CALL_ATTR_CCC_SESSION_PARAMS] = { .type = NLA_NESTED },
};

static const struct nla_policy nfcc_coex_session_param_nla_policy
	[NFCC_COEX_CCC_SESSION_PARAM_ATTR_MAX + 1] = {
		[NFCC_COEX_CCC_SESSION_PARAM_ATTR_TIME0_NS] = { .type = NLA_U64 },
		[NFCC_COEX_CCC_SESSION_PARAM_ATTR_CHANNEL_NUMBER] = { .type = NLA_U8 },
		[NFCC_COEX_CCC_SESSION_PARAM_ATTR_VERSION] =
			NLA_POLICY_RANGE(NLA_U8, 2, 3),
	};

/**
 * nfcc_coex_session_set_parameters() - Set NFCC coexistence session parameters.
 * @local: NFCC coexistence context.
 * @params: Nested attribute containing session parameters.
 * @info: Request information.
 * @now_ns: current kernel time.
 *
 * Return: 0 or error.
 */
static int nfcc_coex_session_set_parameters(struct nfcc_coex_local *local,
					    const struct nlattr *params,
					    const struct genl_info *info,
					    u64 now_ns)
{
	struct nlattr *attrs[NFCC_COEX_CCC_SESSION_PARAM_ATTR_MAX + 1];
	struct nfcc_coex_session *session = &local->session;
	struct nfcc_coex_session_params *p = &session->params;
	/* Maximum dtu duration is INT32_MAX. */
	const s64 max_time0_ns =
		(S32_MAX * NS_PER_SECOND) / local->llhw->dtu_freq_hz;
	int r;

	if (!params)
		return -EINVAL;

	r = nla_parse_nested(attrs, NFCC_COEX_CCC_SESSION_PARAM_ATTR_MAX,
			     params, nfcc_coex_session_param_nla_policy,
			     info->extack);
	if (r)
		return r;

#define P(attr, member, type, conv)                                              \
	do {                                                                     \
		type x;                                                          \
		if (attrs[NFCC_COEX_CCC_SESSION_PARAM_ATTR_##attr]) {            \
			x = nla_get_##type(                                      \
				attrs[NFCC_COEX_CCC_SESSION_PARAM_ATTR_##attr]); \
			p->member = conv;                                        \
		}                                                                \
	} while (0)

	P(TIME0_NS, time0_ns, u64, x);
	P(CHANNEL_NUMBER, channel_number, u8, x);
	P(VERSION, version, u8, x);

#undef P

	if (!attrs[NFCC_COEX_CCC_SESSION_PARAM_ATTR_TIME0_NS]) {
		p->time0_ns = (NS_PER_SECOND * local->llhw->anticip_dtu) /
				      local->llhw->dtu_freq_hz + now_ns;
	}

	if ((s64)(p->time0_ns - now_ns) > max_time0_ns)
		return -ERANGE;
	return 0;
}

/**
 * nfcc_coex_session_start() - Start NFCC coex session.
 * @local: NFCC coexistence context.
 * @info: Request information.
 * @now_ns: current kernel time.
 *
 * Return: 0 or error.
 */
static int nfcc_coex_session_start(struct nfcc_coex_local *local,
				   const struct genl_info *info, u64 now_ns)
{
	struct nfcc_coex_session *session = &local->session;
	const struct nfcc_coex_session_params *p = &session->params;
	u32 now_dtu;
	s64 diff_ns;
	s64 diff_dtu;
	int r;

	WARN_ON(session->state == NFCC_COEX_STATE_STARTED);

	trace_region_nfcc_coex_session_start(local, p);
	r = mcps802154_get_current_timestamp_dtu(local->llhw, &now_dtu);
	if (r)
		return r;

	diff_ns = p->time0_ns - now_ns;
	diff_dtu = div64_s64(diff_ns * local->llhw->dtu_freq_hz, NS_PER_SECOND);
	/* If the requested start date is in the past, start immediately */
	if (diff_dtu < local->llhw->anticip_dtu) {
		pr_warn("dw3000: Computed start date is in the past, scheduling"
			" to anticip_dtu instead");
		diff_dtu = local->llhw->anticip_dtu;
	}

	session->region_demand.timestamp_dtu = now_dtu + diff_dtu;
	session->region_demand.max_duration_dtu = 0;
	session->event_portid = info->snd_portid;
	session->first_access = true;
	nfcc_coex_set_state(local, NFCC_COEX_STATE_STARTED);

	mcps802154_reschedule(local->llhw);
	return 0;
}

/**
 * nfcc_coex_session_start_all() - Start all for a NFCC coex session.
 * @local: NFCC coexistence context.
 * @params: Call parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int nfcc_coex_session_start_all(struct nfcc_coex_local *local,
				       const struct nlattr *params,
				       const struct genl_info *info)
{
	struct nlattr *attrs[NFCC_COEX_CALL_ATTR_MAX + 1];
	int r;
	u64 now_ns;

	if (!params)
		return -EINVAL;

	r = nla_parse_nested(attrs, NFCC_COEX_CALL_ATTR_MAX, params,
			     nfcc_coex_call_nla_policy, info->extack);
	if (r)
		return r;

	if (local->session.state == NFCC_COEX_STATE_STARTED)
		return -EBUSY;

	nfcc_coex_session_init(local);
	now_ns = ktime_to_ns(ktime_get_boottime());
	r = nfcc_coex_session_set_parameters(
		local, attrs[NFCC_COEX_CALL_ATTR_CCC_SESSION_PARAMS], info,
		now_ns);
	if (r)
		return r;

	r = nfcc_coex_session_start(local, info, now_ns);
	if (r)
		return r;

	return 0;
}

/**
 * nfcc_coex_session_stop() - Stop NFCC coex session.
 * @local: NFCC coexistence context.
 *
 * Return: 0 or error.
 */
static int nfcc_coex_session_stop(struct nfcc_coex_local *local)
{
	struct nfcc_coex_session *session = &local->session;

	trace_region_nfcc_coex_session_stop(local);
	if (session->state == NFCC_COEX_STATE_STARTED) {
		nfcc_coex_set_state(local, NFCC_COEX_STATE_STOPPING);
		mcps802154_schedule_invalidate(local->llhw);
	}
	return 0;
}

int nfcc_coex_session_control(struct nfcc_coex_local *local, u32 call_id,
			      const struct nlattr *params,
			      const struct genl_info *info)
{
	switch (call_id) {
	case NFCC_COEX_CALL_CCC_SESSION_START:
		return nfcc_coex_session_start_all(local, params, info);
	default:
	case NFCC_COEX_CALL_CCC_SESSION_STOP:
		return nfcc_coex_session_stop(local);
	}
}
