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

#include "fira_session_fsm_init.h"
#include "fira_session_fsm_idle.h"
#include "fira_session_fsm_active.h"
#include "fira_session.h"
#include "fira_trace.h"

static void
fira_session_fsm_idle_parameters_updated(struct fira_local *local,
					 struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;

	if (session->measurements.reset) {
		session->measurements.reset = false;
		session->measurements.sequence = params->meas_seq;
	}
	if (!fira_session_is_ready(local, session)) {
		fira_session_fsm_change_state(local, session,
					      &fira_session_fsm_init);
	}
}

static void
fira_session_fsm_idle_controlee_list_updated(struct fira_local *local,
					     struct fira_session *session)
{
	if (!fira_session_is_ready(local, session)) {
		fira_session_fsm_change_state(local, session,
					      &fira_session_fsm_init);
	}
}

static int fira_session_fsm_idle_start(struct fira_local *local,
				       struct fira_session *session,
				       const struct genl_info *info)
{
	const struct fira_session_params *params = &session->params;
	struct mcps802154_hrp_uwb_params *hrp = &session->hrp_uwb_params;
	u32 now_dtu;
	int r;
	int i;
	int slot_duration_us;

	trace_region_fira_session_params(session, params);
	for (i = 0; i < params->meas_seq.n_steps; i++) {
		const struct fira_measurement_sequence_step *step;

		step = &params->meas_seq.steps[i];
		trace_region_fira_session_meas_seq_params(session, step, i);
	}
	slot_duration_us = (session->params.slot_duration_dtu * 1000) /
			   (local->llhw->dtu_freq_hz / 1000);

	r = mcps802154_get_current_timestamp_dtu(local->llhw, &now_dtu);
	if (r)
		return r;

	/* Update session. */
	session->event_portid = info->snd_portid;
	session->block_start_valid = false;
	session->block_index = 0;
	session->round_index = 0;
	session->controlee.synchronised = false;
	session->last_access_timestamp_dtu = now_dtu;

	r = fira_sts_init(session, slot_duration_us,
			  mcps802154_get_current_channel(local->llhw));
	if (r)
		return r;

	/* Set radio parameters. */
	switch (params->prf_mode) {
	case FIRA_PRF_MODE_BPRF:
		hrp->prf = MCPS802154_PRF_64;
		break;
	case FIRA_PRF_MODE_HPRF:
		hrp->prf = MCPS802154_PRF_125;
		break;
	default:
		hrp->prf = MCPS802154_PRF_250;
		break;
	}
	hrp->psr = params->preamble_duration == FIRA_PREAMBULE_DURATION_64 ?
			   MCPS802154_PSR_64 :
			   MCPS802154_PSR_32;
	hrp->sfd_selector = (enum mcps802154_sfd)params->sfd_id;
	hrp->phr_hi_rate = params->phr_data_rate == FIRA_PHR_DATA_RATE_6M81;
	switch (params->psdu_data_rate) {
	default:
	case FIRA_PSDU_DATA_RATE_6M81:
		hrp->data_rate = MCPS802154_DATA_RATE_6M81;
		break;
	case FIRA_PSDU_DATA_RATE_7M80:
		hrp->data_rate = MCPS802154_DATA_RATE_7M80;
		break;
	case FIRA_PSDU_DATA_RATE_27M2:
		hrp->data_rate = MCPS802154_DATA_RATE_27M2;
		break;
	case FIRA_PSDU_DATA_RATE_31M2:
		hrp->data_rate = MCPS802154_DATA_RATE_31M2;
		break;
	}
	fira_session_fsm_change_state(local, session, &fira_session_fsm_active);

	mcps802154_reschedule(local->llhw);
	return 0;
}

/* not static: shared with init state */
int fira_session_fsm_idle_check_parameters(const struct fira_session *session,
					   struct nlattr **attrs)
{
	enum fira_session_param_attrs i;

	for (i = FIRA_SESSION_PARAM_ATTR_UNSPEC + 1;
	     i <= FIRA_SESSION_PARAM_ATTR_MAX; i++) {
		const struct nlattr *attr = attrs[i];

		if (!attr)
			/* Attribute not provided. */
			continue;

		switch (i) {
		case FIRA_SESSION_PARAM_ATTR_STS_CONFIG:
			if (fira_crypto_get_capabilities() &
			    (1 << nla_get_u8(attr)))
				continue;
			else
				return -EINVAL;
			break;
		/* no check on other parameters */
		default:
			break;
		}
	}
	return 0;
}

const struct fira_session_fsm_state fira_session_fsm_idle = {
	.id = FIRA_SESSION_STATE_ID_IDLE,
	.parameters_updated = fira_session_fsm_idle_parameters_updated,
	.controlee_list_updated = fira_session_fsm_idle_controlee_list_updated,
	.start = fira_session_fsm_idle_start,
	.check_parameters = fira_session_fsm_idle_check_parameters,
};
