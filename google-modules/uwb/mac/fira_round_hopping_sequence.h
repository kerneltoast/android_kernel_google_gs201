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

#ifndef NET_MCPS802154_FIRA_ROUND_HOPPING_SEQUENCE_H
#define NET_MCPS802154_FIRA_ROUND_HOPPING_SEQUENCE_H

struct fira_session;

/**
 * fira_round_hopping_sequence_init() - Initialize round hopping context.
 * @session: Session.
 *
 * Return: 0 or error.
 */
int fira_round_hopping_sequence_init(struct fira_session *session);

/**
 * fira_round_hopping_sequence_destroy() - Destroy round hopping context.
 * @session: Session.
 */
void fira_round_hopping_sequence_destroy(struct fira_session *session);

/**
 * fira_round_hopping_sequence_get() - Get round index for block index.
 * @session: Session.
 * @block_index: Block index.
 *
 * Return: Round index.
 */
int fira_round_hopping_sequence_get(const struct fira_session *session,
				    int block_index);

#endif /* NET_MCPS802154_FIRA_ROUND_HOPPING_SEQUENCE_H */
