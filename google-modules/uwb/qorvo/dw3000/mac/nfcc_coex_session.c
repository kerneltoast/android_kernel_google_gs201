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

#include "nfcc_coex_session.h"
#include "nfcc_coex_region.h"
#include "nfcc_coex_trace.h"

void nfcc_coex_session_init(struct nfcc_coex_local *local)
{
	struct nfcc_coex_session_params *p = &local->session.params;

	memset(p, 0, sizeof(*p));

	/* Default protocol version is V2 */
	p->version = 3;
}

void nfcc_coex_session_update(struct nfcc_coex_local *local,
			      struct nfcc_coex_session *session,
			      u32 next_timestamp_dtu, int region_duration_dtu)
{
	struct mcps802154_region_demand *rd = &session->region_demand;

	if (is_before_dtu(rd->timestamp_dtu, next_timestamp_dtu)) {
		int shift_dtu = next_timestamp_dtu - rd->timestamp_dtu;

		/* Date is late. */
		trace_region_nfcc_coex_session_update_late(local, shift_dtu, 0);
		rd->timestamp_dtu = next_timestamp_dtu;
		rd->max_duration_dtu = 0;
	}
}
