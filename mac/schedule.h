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

#ifndef MCPS802154_SCHEDULE_H
#define MCPS802154_SCHEDULE_H

#include <linux/kernel.h>
#include <net/mcps802154_schedule.h>

struct mcps802154_local;

/**
 * struct mcps802154_schedule_region - Region as defined in the schedule.
 */
struct mcps802154_schedule_region {
	/**
	 * @region: Pointer to the open region.
	 */
	struct mcps802154_region *region;
	/**
	 * @start_dtu: Region start from the start of the schedule.
	 */
	int start_dtu;
	/**
	 * @duration_dtu: Region duration or 0 for endless region.
	 */
	int duration_dtu;
	/**
	 * @once: Schedule the region once, ignoring the remaining region duration.
	 */
	bool once;
};

/**
 * struct mcps802154_schedule - Schedule.
 */
struct mcps802154_schedule {
	/**
	 * @start_timestamp_dtu: Date of the schedule start, might be too far in
	 * the past for endless schedule.
	 */
	u32 start_timestamp_dtu;
	/**
	 * @duration_dtu: Schedule duration or 0 for endless schedule.
	 */
	int duration_dtu;
	/**
	 * @regions: Table of regions.
	 */
	struct mcps802154_schedule_region *regions;
	/**
	 * @n_regions: Number of regions in the schedule.
	 */
	size_t n_regions;
	/**
	 * @current_index: Index of the current region.
	 */
	int current_index;
};

/**
 * struct mcps802154_schedule_update_local - Private part of a schedule
 * update context.
 */
struct mcps802154_schedule_update_local {
	/**
	 * @schedule_update: Public part.
	 */
	struct mcps802154_schedule_update schedule_update;
	/**
	 * @local: MCPS private data.
	 */
	struct mcps802154_local *local;
};

static inline struct mcps802154_schedule_update_local *schedule_update_to_local(
	const struct mcps802154_schedule_update *schedule_update)
{
	return container_of(schedule_update,
			    struct mcps802154_schedule_update_local,
			    schedule_update);
}

/**
 * mcps802154_schedule_clear() - Clear schedule and release regions.
 * @local: MCPS private data.
 */
void mcps802154_schedule_clear(struct mcps802154_local *local);

/**
 * mcps802154_schedule_update() - Initialize or update the schedule.
 * @local: MCPS private data.
 * @next_timestamp_dtu: Date of next access opportunity.
 *
 * Request the scheduler to update the schedule.
 *
 * Return: 1 or error.
 */
int mcps802154_schedule_update(struct mcps802154_local *local,
			       u32 next_timestamp_dtu);

#endif /* NET_MCPS802154_SCHEDULE_H */
