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
#ifndef PCTT_REGION_CALL_H
#define PCTT_REGION_CALL_H

#include "pctt_region.h"

/**
 * pctt_call_session_control() - PCTT specific region call
 * @local: PCTT region context.
 * @call_id: Identifier of the call procedure.
 * @params: Nested attribute containing procedure parameters.
 * @info: Request information.
 *
 * Return: 0 on success -errno otherwise.
 */
int pctt_call_session_control(struct pctt_local *local, enum pctt_call call_id,
			      const struct nlattr *params,
			      const struct genl_info *info);

/**
 * pctt_call_session_get_state() - Return current state on netlink reply.
 * @local: PCTT region context.
 *
 * Return: 0 on success -errno otherwise.
 */
int pctt_call_session_get_state(struct pctt_local *local);

/**
 * pctt_call_session_get_params() - Return session params on netlink reply
 * @local: PCTT region context.
 *
 * Return: 0 on success -errno otherwise.
 */
int pctt_call_session_get_params(struct pctt_local *local);

#endif /* PCTT_REGION_CALL_H */
