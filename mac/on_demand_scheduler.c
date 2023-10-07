/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
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
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 *
 * 802.15.4 mac common part sublayer, on_demand scheduler.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <net/mcps802154_schedule.h>

#include "mcps802154_i.h"
#include "on_demand_scheduler.h"
#include "warn_return.h"

/**
 * struct mcps802154_on_demand_local - local context for on demand scheduler.
 */
struct mcps802154_on_demand_local {
	/**
	 * @scheduler: Common scheduler context.
	 */
	struct mcps802154_scheduler scheduler;
	/**
	 * @llhw: Low layer hardware attached.
	 */
	struct mcps802154_llhw *llhw;
	/**
	 * @idle_region: Idle region to delay start of region selected.
	 */
	struct mcps802154_region *idle_region;
};

static inline struct mcps802154_on_demand_local *
scheduler_to_plocal(const struct mcps802154_scheduler *scheduler)
{
	return container_of(scheduler, struct mcps802154_on_demand_local,
			    scheduler);
}

static struct mcps802154_scheduler *
mcps802154_on_demand_scheduler_open(struct mcps802154_llhw *llhw)
{
	struct mcps802154_on_demand_local *plocal;

	plocal = kmalloc(sizeof(*plocal), GFP_KERNEL);
	if (!plocal)
		goto open_failure;

	plocal->idle_region = mcps802154_region_open(llhw, "idle", NULL, NULL);
	if (!plocal->idle_region) {
		goto open_failure;
	}

	plocal->llhw = llhw;
	plocal->scheduler.n_regions = 0;
	return &plocal->scheduler;

open_failure:
	kfree(plocal);
	return NULL;
}

static void
mcps802154_on_demand_scheduler_close(struct mcps802154_scheduler *scheduler)
{
	struct mcps802154_on_demand_local *plocal =
		scheduler_to_plocal(scheduler);

	kfree(plocal->idle_region);
	kfree(plocal);
}

static int mcps802154_on_demand_scheduler_get_next_region(
	struct mcps802154_on_demand_local *plocal, struct list_head *regions,
	const struct mcps802154_region *first_region, u32 next_timestamp_dtu,
	struct mcps802154_region_demand *next_demand,
	struct mcps802154_region **next_region)
{
	struct mcps802154_region *region;
	int max_duration_dtu = 0;
	int r;

	*next_region = NULL;
	list_for_each_entry (region, regions, ca_entry) {
		struct mcps802154_region_demand candidate = {};

		if (first_region && region == first_region)
			continue;

		r = mcps802154_region_get_demand(
			plocal->llhw, region, next_timestamp_dtu, &candidate);
		switch (r) {
		case 0:
			/* The region doesn't have a demand. */
			continue;
		case 1:
			/* The region have a demand. */
			break;
		default:
			return r;
		}

		/* Reduce duration of candidate region with less priority. */
		if (max_duration_dtu &&
		    (!candidate.max_duration_dtu ||
		     is_before_dtu(next_timestamp_dtu + max_duration_dtu,
				   candidate.timestamp_dtu +
					   candidate.max_duration_dtu)))
			candidate.max_duration_dtu = max_duration_dtu -
						     candidate.timestamp_dtu +
						     next_timestamp_dtu;

		/* Arbitrate between regions. */
		if (!*next_region ||
		    is_before_dtu(candidate.timestamp_dtu,
				  next_demand->timestamp_dtu)) {
			*next_region = region;
			*next_demand = candidate;
			/* Is there some time remaining for a region with
			 * less priority? */
			if (!is_before_dtu(next_timestamp_dtu,
					   next_demand->timestamp_dtu))
				break;
			else
				max_duration_dtu = next_demand->timestamp_dtu -
						   next_timestamp_dtu;
		}
	}

	return *next_region ? 1 : 0;
}

static int mcps802154_on_demand_scheduler_update_schedule(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_schedule_update *schedule_update,
	u32 next_timestamp_dtu)
{
	struct mcps802154_on_demand_local *plocal =
		scheduler_to_plocal(scheduler);
	struct list_head *regions;
	struct mcps802154_region_demand next_demand;
	struct mcps802154_region *next_region = NULL;
	u32 start_in_schedule_dtu;
	int r;

	mcps802154_schedule_get_regions(plocal->llhw, &regions);
	r = mcps802154_on_demand_scheduler_get_next_region(
		plocal, regions, NULL, next_timestamp_dtu, &next_demand,
		&next_region);
	if (r < 0)
		return r;

	if (!next_region)
		return -ENOENT;

	start_in_schedule_dtu = next_demand.timestamp_dtu - next_timestamp_dtu;

	r = mcps802154_schedule_set_start(schedule_update, next_timestamp_dtu);
	WARN_RETURN(r);

	r = mcps802154_schedule_recycle(schedule_update, 0,
					MCPS802154_DURATION_NO_CHANGE);
	/* Can not fail, only possible error is invalid parameters. */
	WARN_RETURN(r);

	if (next_demand.max_duration_dtu)
		next_demand.max_duration_dtu += start_in_schedule_dtu;
 	start_in_schedule_dtu = 0;

	if (start_in_schedule_dtu)
		/* Don't give the access to the region too early.
		 * And provide advantages:
		 *  - to have a region inserted with a CA invalidate schedule.
		 *  - Reduce latency with TX frame prepared close to region
		 *    start date. */
		r = mcps802154_schedule_add_region(schedule_update,
						   plocal->idle_region, 0,
						   start_in_schedule_dtu,
						   false);
	r = mcps802154_schedule_add_region(schedule_update, next_region,
					   start_in_schedule_dtu,
					   next_demand.max_duration_dtu, true);

	return r;
}

static int mcps802154_on_demand_scheduler_get_next_demands(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_region *region, u32 timestamp_dtu,
	int duration_dtu, int delta_dtu,
	struct mcps802154_region_demand *demands)
{
	struct mcps802154_on_demand_local *plocal =
		scheduler_to_plocal(scheduler);
	struct list_head *regions;
	bool is_demands_set = false;
	u32 next_timestamp_dtu = timestamp_dtu;
	int r;

	mcps802154_schedule_get_regions(plocal->llhw, &regions);

	while (true) {
		struct mcps802154_region_demand next_demand;
		struct mcps802154_region *next_region = NULL;

		r = mcps802154_on_demand_scheduler_get_next_region(
			plocal, regions, region, next_timestamp_dtu,
			&next_demand, &next_region);
		if (r < 0)
			return r;
		if (!r || !next_demand.max_duration_dtu ||
		    !is_before_dtu(next_demand.timestamp_dtu,
				   timestamp_dtu + duration_dtu))
			break;
		if (!is_demands_set) {
			*demands = next_demand;
			is_demands_set = true;
		} else if (!is_before_dtu(demands->timestamp_dtu +
						  demands->max_duration_dtu +
						  delta_dtu,
					  next_demand.timestamp_dtu)) {
			demands->max_duration_dtu =
				next_demand.timestamp_dtu +
				next_demand.max_duration_dtu -
				demands->timestamp_dtu;
		} else {
			break;
		}

		if (!is_before_dtu(demands->timestamp_dtu +
					   demands->max_duration_dtu,
				   timestamp_dtu + duration_dtu))
			break;

		next_timestamp_dtu =
			demands->timestamp_dtu + demands->max_duration_dtu;
	}
	return is_demands_set ? 1 : 0;
}

static struct mcps802154_scheduler_ops
	mcps802154_on_demand_scheduler_scheduler = {
		.owner = THIS_MODULE,
		.name = "on_demand",
		.open = mcps802154_on_demand_scheduler_open,
		.close = mcps802154_on_demand_scheduler_close,
		.set_parameters = NULL, /* No scheduler parameters for now. */
		.update_schedule =
			mcps802154_on_demand_scheduler_update_schedule,
		.get_next_demands =
			mcps802154_on_demand_scheduler_get_next_demands,
	};

int __init mcps802154_on_demand_scheduler_init(void)
{
	return mcps802154_scheduler_register(
		&mcps802154_on_demand_scheduler_scheduler);
}

void __exit mcps802154_on_demand_scheduler_exit(void)
{
	mcps802154_scheduler_unregister(
		&mcps802154_on_demand_scheduler_scheduler);
}
