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

#ifndef NET_MCPS802154_FIRA_SESSION_FSM_H
#define NET_MCPS802154_FIRA_SESSION_FSM_H

#include <linux/ieee802154.h>

#include "fira_access.h"

/* Forward declaration. */
struct fira_local;
struct fira_session;
struct fira_session_demand;

/**
 * enum fira_session_state_id - State of the FiRa session.
 * @FIRA_SESSION_STATE_ID_INIT:
 *     Initial state, session is not ready yet.
 * @FIRA_SESSION_STATE_ID_DEINIT:
 *     Session does not exist.
 * @FIRA_SESSION_STATE_ID_ACTIVE:
 *     Session is currently active.
 * @FIRA_SESSION_STATE_ID_IDLE:
 *     Session is ready to start, but not currently active.
 */
enum fira_session_state_id {
	FIRA_SESSION_STATE_ID_INIT,
	FIRA_SESSION_STATE_ID_DEINIT,
	FIRA_SESSION_STATE_ID_ACTIVE,
	FIRA_SESSION_STATE_ID_IDLE,
};

/**
 * struct fira_session_fsm_state - FiRa session FSM state.
 *
 * This structure contains the callbacks which are called on an event to handle
 * the transition from the current state.
 */
struct fira_session_fsm_state {
	/** @id: Name of state. */
	enum fira_session_state_id id;
	/** @enter: Run when the state is entered. */
	void (*enter)(struct fira_local *local, struct fira_session *session);
	/** @leave: Run when the state is left. */
	void (*leave)(struct fira_local *local, struct fira_session *session);
	/** @check_parameters: Handle a check parameters. */
	int (*check_parameters)(const struct fira_session *session,
				struct nlattr **attrs);
	/** @parameters_updated: Handle parameters updated event. */
	void (*parameters_updated)(struct fira_local *local,
				   struct fira_session *session);
	/** @controlee_list_updated: Handle controlee list updated event. */
	void (*controlee_list_updated)(struct fira_local *local,
				       struct fira_session *session);
	/** @start: Handle start. */
	int (*start)(struct fira_local *local, struct fira_session *session,
		     const struct genl_info *info);
	/** @stop: Handle stop. */
	int (*stop)(struct fira_local *local, struct fira_session *session);
	/** @get_demand: Handle the get demand. */
	int (*get_demand)(const struct fira_local *local,
			  const struct fira_session *session,
			  u32 next_timestamp_dtu, int max_duration_dtu,
			  struct fira_session_demand *session_demand);
	/** @get_access: Handle the get access. */
	struct mcps802154_access *(*get_access)(
		struct fira_local *local, struct fira_session *session,
		const struct fira_session_demand *session_demand);
	/** @access_done: Handle end of access. */
	void (*access_done)(struct fira_local *local,
			    struct fira_session *session, bool error);
	/** @check_missed_ranging: Handle the check of missed ranging. */
	void (*check_missed_ranging)(struct fira_local *local,
				     struct fira_session *session,
				     u32 timestamp_dtu);
};

/**
 * fira_session_fsm_change_state() - Change the state of the FSM.
 * @local: FiRa context.
 * @session: Session context.
 * @new_state: New to state to use in the FSM.
 *
 * This function shall be called only by fira_session_fsm files.
 */
void fira_session_fsm_change_state(
	struct fira_local *local, struct fira_session *session,
	const struct fira_session_fsm_state *new_state);

/**
 * fira_session_is_active() - Return the active status of the session.
 * @session: Session context.
 *
 * Return: True is the session is active, false otherwise.
 */
bool fira_session_is_active(const struct fira_session *session);

/**
 * fira_session_fsm_initialise() - Initialize the FSM.
 * @local: FiRa context.
 * @session: Session context.
 */
void fira_session_fsm_initialise(struct fira_local *local,
				 struct fira_session *session);

/**
 * fira_session_fsm_uninit() - Uninitialise the FSM.
 * @local: FiRa context.
 * @session: Session context.
 */
void fira_session_fsm_uninit(struct fira_local *local,
			     struct fira_session *session);

/**
 * fira_session_get_state_id() - Get current state id (for reporting).
 * @session: Session context.
 *
 * Return: State id value.
 */
enum fira_session_state_id
fira_session_get_state_id(const struct fira_session *session);

/**
 * fira_session_fsm_check_parameters() - Check parameters change ask by upper
 * layer.
 * @session: Session context.
 * @attrs: Netlink attributs.
 *
 * Return: 0 on success, errno when change are refused.
 */
int fira_session_fsm_check_parameters(const struct fira_session *session,
				      struct nlattr **attrs);

/**
 * fira_session_fsm_parameters_updated() - Parameters updated by upper layer.
 * @local: FiRa context.
 * @session: Session context.
 */
void fira_session_fsm_parameters_updated(struct fira_local *local,
					 struct fira_session *session);

/**
 * fira_session_fsm_controlee_list_updated() - Controlee list updated by upper
 * layer.
 * @local: FiRa context.
 * @session: Session context.
 */
void fira_session_fsm_controlee_list_updated(struct fira_local *local,
					     struct fira_session *session);

/**
 * fira_session_fsm_start() - Start request from upper layer.
 * @local: FiRa context.
 * @session: Session context.
 * @info: Netlink info used only for the portid.
 *
 * Return: 0 on success, errno otherwise.
 */
int fira_session_fsm_start(struct fira_local *local,
			   struct fira_session *session,
			   const struct genl_info *info);

/**
 * fira_session_fsm_stop() - Stop request from upper layer.
 * @local: FiRa context.
 * @session: Session context.
 *
 * Return: 0 on success, errno otherwise.
 */
int fira_session_fsm_stop(struct fira_local *local,
			  struct fira_session *session);

/**
 * fira_session_fsm_get_demand() - Request the next ranging round of the session.
 * @local: FiRa context.
 * @session: Session context.
 * @next_timestamp_dtu: Timestamp to start a demand.
 * @max_duration_dtu: Max duration obligation to be consider by the session.
 * @session_demand: Wish of the session when the return value is 1.
 *
 * Return: 1 for a session demand otherwise 0 for no demand.
 */
int fira_session_fsm_get_demand(const struct fira_local *local,
				const struct fira_session *session,
				u32 next_timestamp_dtu, int max_duration_dtu,
				struct fira_session_demand *session_demand);

/**
 * fira_session_fsm_get_access() - Get access to process.
 * @local: FiRa context.
 * @session: Session context.
 * @session_demand: Next access built by the get_demand.
 *
 * Return: The access for fproc, or NULL pointer.
 */
struct mcps802154_access *
fira_session_fsm_get_access(struct fira_local *local,
			    struct fira_session *session,
			    const struct fira_session_demand *session_demand);

/**
 * fira_session_fsm_access_done() - End of the access to report.
 * @local: FiRa context.
 * @session: Session context.
 * @error: True when an error happen.
 */
void fira_session_fsm_access_done(struct fira_local *local,
				  struct fira_session *session, bool error);

/**
 * fira_session_fsm_check_missed_ranging() - Report a missed ranging if exist.
 * @local: FiRa context.
 * @session: Session context.
 * @timestamp_dtu: Timestamp dtu where no fallback is possible.
 */
void fira_session_fsm_check_missed_ranging(struct fira_local *local,
					   struct fira_session *session,
					   u32 timestamp_dtu);

#endif /* NET_MCPS802154_FIRA_SESSION_FSM_H */
