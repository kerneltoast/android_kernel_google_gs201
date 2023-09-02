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

#ifndef FIRA_ACCESS_H
#define FIRA_ACCESS_H

#include <net/mcps802154_schedule.h>

/* Forward declaration. */
struct fira_local;
struct fira_session;
struct fira_session_demand;

/**
 * fira_session_get_slot_count() - Return the number of slot for the session.
 * @session: FiRa session context.
 *
 * Return: Number of slot.
 */
int fira_session_get_slot_count(const struct fira_session *session);

/**
 * fira_get_access_controller() - Build the access for controller.
 * @local: FiRa region context.
 * @fsd: FiRa Session Demand from the get_demand of the session fsm.
 *
 * Return: A valid access.
 */
struct mcps802154_access *
fira_get_access_controller(struct fira_local *local,
			   const struct fira_session_demand *fsd);

/**
 * fira_get_access_controlee() - Build the access for controlee.
 * @local: FiRa region context.
 * @fsd: FiRa Session Demand from the get_demand of the session fsm.
 *
 * Return: A valid access.
 */
struct mcps802154_access *
fira_get_access_controlee(struct fira_local *local,
			  const struct fira_session_demand *fsd);

#endif /* FIRA_ACCESS_H */
