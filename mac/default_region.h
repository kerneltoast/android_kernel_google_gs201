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

#ifndef NET_MCPS802154_DEFAULT_REGION_H
#define NET_MCPS802154_DEFAULT_REGION_H

#include <net/mcps802154_schedule.h>

/**
 * struct default_local - Local context.
 */
struct default_local {
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
	 * @queue: Queue of frames to be transmitted.
	 */
	struct sk_buff_head queue;
	/**
	 * @n_queued: Number of queued frames. This also includes frame being
	 * transmitted which is no longer in &queue.
	 */
	atomic_t n_queued;
	/**
	 * @retries: Number of retries done on the current tx frame.
	 */
	int retries;
};

static inline struct default_local *
region_to_local(struct mcps802154_region *region)
{
	return container_of(region, struct default_local, region);
}

static inline struct default_local *
access_to_local(struct mcps802154_access *access)
{
	return container_of(access, struct default_local, access);
}

int mcps802154_default_region_init(void);
void mcps802154_default_region_exit(void);

#endif /* NET_MCPS802154_DEFAULT_REGION_H */
