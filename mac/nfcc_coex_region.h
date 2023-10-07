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

#ifndef NET_NFCC_COEX_REGION_H
#define NET_NFCC_COEX_REGION_H

#include <net/mcps802154_schedule.h>
#include <net/vendor_cmd.h>
#include "nfcc_coex_session.h"

/**
 * struct nfcc_coex_local - Local context.
 */
struct nfcc_coex_local {
	/**
	 * @region: Region instance returned to MCPS.
	 */
	struct mcps802154_region region;
	/**
	 * @llhw: Low-level device pointer.
	 */
	struct mcps802154_llhw *llhw;
	/**
	 * @access: Access returned to MCPS.
	 */
	struct mcps802154_access access;
	/**
	 * @session: Unique session on the NFCC controller.
	 */
	struct nfcc_coex_session session;
};

static inline struct nfcc_coex_local *
region_to_local(struct mcps802154_region *region)
{
	return container_of(region, struct nfcc_coex_local, region);
}

static inline struct nfcc_coex_local *
access_to_local(struct mcps802154_access *access)
{
	return container_of(access, struct nfcc_coex_local, access);
}

/**
 * nfcc_coex_set_state() - Set the new state.
 * @local: NFCC coex context.
 * @new_state: New nfcc_coex state.
 */
void nfcc_coex_set_state(struct nfcc_coex_local *local,
			 enum nfcc_coex_state new_state);

/**
 * nfcc_coex_report() - Send notification to upper layer.
 * @local: Local nfcc coex context.
 */
void nfcc_coex_report(struct nfcc_coex_local *local);

#endif /* NET_NFCC_COEX_REGION_H */
