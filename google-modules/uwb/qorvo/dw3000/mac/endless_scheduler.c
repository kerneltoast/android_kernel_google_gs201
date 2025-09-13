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

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <net/mcps802154_schedule.h>

#include "endless_scheduler.h"
#include "warn_return.h"

struct mcps802154_endless_local {
	struct mcps802154_scheduler scheduler;
	struct mcps802154_llhw *llhw;
};

static inline struct mcps802154_endless_local *
scheduler_to_plocal(const struct mcps802154_scheduler *scheduler)
{
	return container_of(scheduler, struct mcps802154_endless_local,
			    scheduler);
}

static struct mcps802154_scheduler *
mcps802154_endless_scheduler_open(struct mcps802154_llhw *llhw)
{
	struct mcps802154_endless_local *plocal;

	plocal = kmalloc(sizeof(*plocal), GFP_KERNEL);
	if (!plocal)
		return NULL;
	plocal->llhw = llhw;
	plocal->scheduler.n_regions = 1;
	return &plocal->scheduler;
}

static void
mcps802154_endless_scheduler_close(struct mcps802154_scheduler *scheduler)
{
	struct mcps802154_endless_local *plocal =
		scheduler_to_plocal(scheduler);

	kfree(plocal);
}
static int mcps802154_endless_scheduler_update_schedule(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_schedule_update *schedule_update,
	u32 next_timestamp_dtu)
{
	struct mcps802154_endless_local *plocal =
		scheduler_to_plocal(scheduler);
	struct list_head *regions = NULL;
	struct mcps802154_region *region;
	int r, n_regions;

	n_regions = mcps802154_schedule_get_regions(plocal->llhw, &regions);
	if (!n_regions)
		return -ENOENT;
	WARN_RETURN_ON(n_regions > 1, -EINVAL);

	region = list_first_entry(regions, typeof(*region), ca_entry);

	r = mcps802154_schedule_set_start(
		schedule_update, schedule_update->expected_start_timestamp_dtu);
	/* Can not fail, only possible error is invalid parameters. */
	WARN_RETURN(r);

	r = mcps802154_schedule_recycle(schedule_update, 0,
					MCPS802154_DURATION_NO_CHANGE);
	/* Can not fail, only possible error is invalid parameters. */
	WARN_RETURN(r);

	r = mcps802154_schedule_add_region(schedule_update, region, 0, 0,
					   false);

	return r;
}

static struct mcps802154_scheduler_ops mcps802154_endless_scheduler_scheduler = {
	.owner = THIS_MODULE,
	.name = "endless",
	.open = mcps802154_endless_scheduler_open,
	.close = mcps802154_endless_scheduler_close,
	.update_schedule = mcps802154_endless_scheduler_update_schedule,
};

static struct mcps802154_scheduler_ops mcps802154_default_scheduler_scheduler = {
	.owner = THIS_MODULE,
	.name = "default",
	.open = mcps802154_endless_scheduler_open,
	.close = mcps802154_endless_scheduler_close,
	.update_schedule = mcps802154_endless_scheduler_update_schedule,
};

int __init mcps802154_endless_scheduler_init(void)
{
	return mcps802154_scheduler_register(
		&mcps802154_endless_scheduler_scheduler);
}

void __exit mcps802154_endless_scheduler_exit(void)
{
	mcps802154_scheduler_unregister(
		&mcps802154_endless_scheduler_scheduler);
}

int __init mcps802154_default_scheduler_init(void)
{
	return mcps802154_scheduler_register(
		&mcps802154_default_scheduler_scheduler);
}

void __exit mcps802154_default_scheduler_exit(void)
{
	mcps802154_scheduler_unregister(
		&mcps802154_default_scheduler_scheduler);
}
