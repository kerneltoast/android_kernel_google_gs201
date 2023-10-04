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

static void fira_session_fsm_init_enter(struct fira_local *local,
					struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;

	session->measurements.sequence = params->meas_seq;

	if (fira_session_is_ready(local, session)) {
		fira_session_fsm_change_state(local, session,
					      &fira_session_fsm_idle);
	}
}

void fira_session_fsm_init_parameters_updated(struct fira_local *local,
					      struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;

	if (session->measurements.reset) {
		session->measurements.reset = false;
		session->measurements.sequence = params->meas_seq;
	}
	if (fira_session_is_ready(local, session)) {
		fira_session_fsm_change_state(local, session,
					      &fira_session_fsm_idle);
	}
}

static void
fira_session_fsm_init_controlee_list_updated(struct fira_local *local,
					     struct fira_session *session)
{
	if (fira_session_is_ready(local, session)) {
		fira_session_fsm_change_state(local, session,
					      &fira_session_fsm_idle);
	}
}

int fira_session_fsm_idle_check_parameters(const struct fira_session *session,
					   struct nlattr **attrs);

const struct fira_session_fsm_state fira_session_fsm_init = {
	.id = FIRA_SESSION_STATE_ID_INIT,
	.enter = fira_session_fsm_init_enter,
	.parameters_updated = fira_session_fsm_init_parameters_updated,
	.controlee_list_updated = fira_session_fsm_init_controlee_list_updated,
	.check_parameters = fira_session_fsm_idle_check_parameters,
};
