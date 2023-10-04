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
#ifndef NET_MCPS802154_NFCC_COEX_REGION_CALL_H
#define NET_MCPS802154_NFCC_COEX_REGION_CALL_H

#include "nfcc_coex_region.h"

/**
 * nfcc_coex_session_control() - Control nfcc_coex session.
 * @local: Vendor context.
 * @call_id: Identifier of the nfcc_coex procedure.
 * @params: Call parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
int nfcc_coex_session_control(struct nfcc_coex_local *local, u32 call_id,
			      const struct nlattr *params,
			      const struct genl_info *info);

#endif /* NET_MCPS802154_NFCC_COEX_REGION_CALL_H */
