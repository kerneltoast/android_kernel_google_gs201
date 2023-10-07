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

#include <linux/errno.h>

#include "fira_session_fsm_init.h"
#include "fira_session_fsm_idle.h"
#include "fira_session_fsm_active.h"
#include "fira_session.h"
#include "fira_access.h"
#include "fira_trace.h"

void fira_session_fsm_initialise(struct fira_local *local,
				 struct fira_session *session)
{
	list_add(&session->entry, &local->inactive_sessions);
	session->state = &fira_session_fsm_init;
	WARN_ON(!session->state->enter);
	session->state->enter(local, session);
}

void fira_session_fsm_uninit(struct fira_local *local,
			     struct fira_session *session)
{
	if (session->state->leave)
		session->state->leave(local, session);

	trace_region_fira_session_fsm_change_state(
		session, FIRA_SESSION_STATE_ID_DEINIT);
	list_del(&session->entry);
}

void fira_session_fsm_change_state(
	struct fira_local *local, struct fira_session *session,
	const struct fira_session_fsm_state *new_state)
{
	if (session->state->leave)
		session->state->leave(local, session);
	trace_region_fira_session_fsm_change_state(session, new_state->id);
	session->state = new_state;
	if (session->state->enter)
		session->state->enter(local, session);
}

bool fira_session_is_active(const struct fira_session *session)
{
	return session->state == &fira_session_fsm_active;
}

enum fira_session_state_id
fira_session_get_state_id(const struct fira_session *session)
{
	return session->state->id;
}

int fira_session_fsm_check_parameters(const struct fira_session *session,
				      struct nlattr **attrs)
{
	WARN_ON(!session->state->check_parameters);
	return session->state->check_parameters(session, attrs);
}

void fira_session_fsm_parameters_updated(struct fira_local *local,
					 struct fira_session *session)
{
	/* The handler is defined for all states. */
	WARN_ON(!session->state->parameters_updated);
	session->state->parameters_updated(local, session);
}

void fira_session_fsm_controlee_list_updated(struct fira_local *local,
					     struct fira_session *session)
{
	if (session->state->controlee_list_updated)
		session->state->controlee_list_updated(local, session);
}

int fira_session_fsm_start(struct fira_local *local,
			   struct fira_session *session,
			   const struct genl_info *info)
{
	if (session->state->start)
		return session->state->start(local, session, info);
	return -EINVAL;
}

int fira_session_fsm_stop(struct fira_local *local,
			  struct fira_session *session)
{
	if (session->state->stop)
		return session->state->stop(local, session);
	return 0;
}

int fira_session_fsm_get_demand(const struct fira_local *local,
				const struct fira_session *session,
				u32 next_timestamp_dtu, int max_duration_dtu,
				struct fira_session_demand *session_demand)
{
	/*
	 * fira_get_demand will not call this function without an
	 * active session.
	 */
	WARN_ON(!session->state->get_demand);
	return session->state->get_demand(local, session, next_timestamp_dtu,
					  max_duration_dtu, session_demand);
}

struct mcps802154_access *
fira_session_fsm_get_access(struct fira_local *local,
			    struct fira_session *session,
			    const struct fira_session_demand *session_demand)
{
	/*
	 * fira_get_access will not call this function without an
	 * active session.
	 */
	WARN_ON(!session->state->get_access);
	return session->state->get_access(local, session, session_demand);
}

void fira_session_fsm_access_done(struct fira_local *local,
				  struct fira_session *session, bool error)
{
	WARN_ON(!session->state->access_done);
	return session->state->access_done(local, session, error);
}

void fira_session_fsm_check_missed_ranging(struct fira_local *local,
					   struct fira_session *session,
					   u32 timestamp_dtu)
{
	WARN_ON(!session->state->check_missed_ranging);
	return session->state->check_missed_ranging(local, session,
						    timestamp_dtu);
}
