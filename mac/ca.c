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
#include <linux/list.h>
#include <linux/string.h>

#include "mcps802154_i.h"
#include "schedulers.h"
#include "trace.h"

struct mcps802154_access_common_ops ca_access_ops = {};

static int mcps802154_ca_trace_int(struct mcps802154_local *local, const int r)
{
	trace_ca_return_int(local, r);
	return r;
}

static void mcps802154_ca_close_scheduler(struct mcps802154_local *local)
{
	struct mcps802154_region *region, *r;
	struct mcps802154_ca *ca = &local->ca;
	struct list_head *regions = &ca->regions;

	mcps802154_schedule_clear(local);
	if (local->ca.scheduler) {
		mcps802154_scheduler_close(local->ca.scheduler);
		local->ca.scheduler = NULL;
	}
	list_for_each_entry_safe (region, r, regions, ca_entry) {
		list_del(&region->ca_entry);
		mcps802154_region_close(&local->llhw, region);
	}
	ca->n_regions = 0;
}

static int check_and_get_region(struct mcps802154_ca *ca,
				const char *scheduler_name, u32 region_id,
				const char *region_name,
				struct mcps802154_region **sel_region)
{
	struct mcps802154_scheduler *scheduler = ca->scheduler;
	struct list_head *regions = &ca->regions;
	struct mcps802154_region *region;

	/* Check scheduler is the correct one. */
	if (!scheduler || strcmp(scheduler->ops->name, scheduler_name) ||
	    !region_name)
		return -EINVAL;

	list_for_each_entry (region, regions, ca_entry) {
		if (!strcmp(region_name, region->ops->name) &&
		    region->id == region_id) {
			*sel_region = region;
			return 0;
		}
	}
	return -ENOENT;
}

void mcps802154_ca_init(struct mcps802154_local *local)
{
	struct mcps802154_ca *ca = &local->ca;

	local->ca.held = false;
	local->ca.reset = false;
	INIT_LIST_HEAD(&ca->regions);
	ca->n_regions = 0;
}

void mcps802154_ca_uninit(struct mcps802154_local *local)
{
}

int mcps802154_ca_start(struct mcps802154_local *local)
{
	int r;

	if (!local->ca.scheduler) {
		r = mcps802154_ca_set_scheduler(local, "default", NULL, NULL);
		if (r)
			return r;

		r = mcps802154_ca_set_region(local, "default", 0, "default",
					     NULL, NULL);
		if (r)
			return r;
	}

	local->start_stop_request = true;
	mcps802154_fproc_schedule_change(local);

	return local->started ? 0 : -EIO;
}

void mcps802154_ca_stop(struct mcps802154_local *local)
{
	local->start_stop_request = false;
	mcps802154_fproc_schedule_change(local);
}

void mcps802154_ca_notify_stop(struct mcps802154_local *local)
{
	struct mcps802154_region *region;
	struct mcps802154_ca *ca = &local->ca;
	struct list_head *regions = &ca->regions;

	mcps802154_scheduler_notify_stop(local->ca.scheduler);
	list_for_each_entry (region, regions, ca_entry) {
		mcps802154_region_notify_stop(&local->llhw, region);
	}
}

void mcps802154_ca_close(struct mcps802154_local *local)
{
	mcps802154_ca_close_scheduler(local);
}

int mcps802154_ca_set_scheduler(struct mcps802154_local *local,
				const char *name,
				const struct nlattr *params_attr,
				struct netlink_ext_ack *extack)
{
	struct mcps802154_scheduler *scheduler;

	trace_ca_set_scheduler(local, name);

	if (local->started)
		return mcps802154_ca_trace_int(local, -EBUSY);
	/* Open new scheduler. */
	scheduler = mcps802154_scheduler_open(local, name, params_attr, extack);
	if (!scheduler)
		return mcps802154_ca_trace_int(local, -EINVAL);
	/* Close previous scheduler and set the new one. */
	mcps802154_ca_close_scheduler(local);
	local->ca.scheduler = scheduler;

	return mcps802154_ca_trace_int(local, 0);
}

int mcps802154_ca_set_region(struct mcps802154_local *local,
			     const char *scheduler_name, u32 region_id,
			     const char *region_name,
			     const struct nlattr *params_attr,
			     struct netlink_ext_ack *extack)
{
	struct mcps802154_ca *ca = &local->ca;
	struct list_head *regions = &ca->regions;
	struct mcps802154_scheduler *scheduler;
	struct mcps802154_region *region, *old_region;
	struct list_head *position = regions;
	bool region_id_present = false;

	trace_ca_set_region(local, scheduler_name, region_id, region_name);

	scheduler = local->ca.scheduler;
	/* Check scheduler is the correct one. */
	if (!scheduler || strcmp(scheduler->ops->name, scheduler_name) ||
	    !region_name)
		return mcps802154_ca_trace_int(local, -EINVAL);

	/* Check if we need to replace an already opened region. */
	list_for_each_entry (old_region, regions, ca_entry) {
		if (old_region->id == region_id) {
			region_id_present = true;
			break;
		} else if (old_region->id < region_id) {
			position = &old_region->ca_entry;
		} else {
			break;
		}
	}

	/* Regions number is limited by the scheduler */
	if (!region_id_present && scheduler->n_regions &&
	    ca->n_regions >= scheduler->n_regions)
		return mcps802154_ca_trace_int(local, -ENOSPC);

	region = mcps802154_region_open(&local->llhw, region_name, params_attr,
					extack);

	if (!region)
		return mcps802154_ca_trace_int(local, -EINVAL);
	region->id = region_id;

	if (region_id_present) {
		list_replace(&old_region->ca_entry, &region->ca_entry);
		mcps802154_region_close(&local->llhw, old_region);
	} else {
		list_add(&region->ca_entry, position);
		ca->n_regions++;
	}

	return mcps802154_ca_trace_int(local, 0);
}

int mcps802154_ca_scheduler_set_parameters(struct mcps802154_local *local,
					   const char *name,
					   const struct nlattr *params_attr,
					   struct netlink_ext_ack *extack)
{
	struct mcps802154_scheduler *scheduler;
	int r;

	trace_ca_set_scheduler_parameters(local, name);

	scheduler = local->ca.scheduler;

	/* Check scheduler is the correct one. */
	if (!scheduler || strcmp(scheduler->ops->name, name))
		return mcps802154_ca_trace_int(local, -EINVAL);

	r = mcps802154_scheduler_set_parameters(scheduler, params_attr, extack);
	return mcps802154_ca_trace_int(local, r);
}

int mcps802154_ca_scheduler_call(struct mcps802154_local *local,
				 const char *scheduler_name, u32 call_id,
				 const struct nlattr *params_attr,
				 const struct genl_info *info)
{
	struct mcps802154_scheduler *scheduler;
	int r;

	trace_ca_scheduler_call(local, scheduler_name, call_id);

	scheduler = local->ca.scheduler;
	/* Check scheduler is the correct one. */
	if (!scheduler || strcmp(scheduler->ops->name, scheduler_name))
		return mcps802154_ca_trace_int(local, -EINVAL);
	r = mcps802154_scheduler_call(scheduler, call_id, params_attr, info);
	return mcps802154_ca_trace_int(local, r);
}

int mcps802154_ca_set_region_parameters(struct mcps802154_local *local,
					const char *scheduler_name,
					u32 region_id, const char *region_name,
					const struct nlattr *params_attr,
					struct netlink_ext_ack *extack)
{
	struct mcps802154_region *region;
	int r;

	trace_ca_set_region_params(local, scheduler_name, region_id,
				   region_name);

	r = check_and_get_region(&local->ca, scheduler_name, region_id,
				 region_name, &region);
	if (r)
		goto end;

	r = mcps802154_region_set_parameters(&local->llhw, region, params_attr,
					     extack);

end:
	return mcps802154_ca_trace_int(local, r);
}

int mcps802154_ca_call_region(struct mcps802154_local *local,
			      const char *scheduler_name, u32 region_id,
			      const char *region_name, u32 call_id,
			      const struct nlattr *params_attr,
			      const struct genl_info *info)
{
	struct mcps802154_region *region;
	int r;

	trace_ca_call_region(local, scheduler_name, region_id, region_name,
			     call_id);

	r = check_and_get_region(&local->ca, scheduler_name, region_id,
				 region_name, &region);
	if (r)
		goto end;

	r = mcps802154_region_call(&local->llhw, region, call_id, params_attr,
				   info);

end:
	return mcps802154_ca_trace_int(local, r);
}

int mcps802154_ca_xmit_skb(struct mcps802154_local *local, struct sk_buff *skb)
{
	struct mcps802154_region *region;
	int r = -EOPNOTSUPP;

	list_for_each_entry (region, &local->ca.regions, ca_entry) {
		if (region->ops->xmit_skb) {
			if (region->ops->xmit_skb(region, skb)) {
				r = 0;
				break;
			}
		}
	}
	return r;
}

/**
 * mcps802154_ca_next_region() - Check the current region is still valid, if not
 * change region.
 * @local: MCPS private data.
 * @next_timestamp_dtu: Date of next access opportunity.
 *
 * Return: 0 if unchanged, 1 if changed, or negative error.
 */
static int mcps802154_ca_next_region(struct mcps802154_local *local,
				     u32 next_timestamp_dtu)
{
	struct mcps802154_schedule *sched = &local->ca.schedule;
	struct mcps802154_schedule_region *sched_region;
	int next_dtu = next_timestamp_dtu - sched->start_timestamp_dtu;
	bool changed = 0;
	bool once;

	sched_region = &sched->regions[sched->current_index];
	once = sched_region->once;

	/* If the region schedule is over, select the next region if
	 * possible. */
	while (once || (sched_region->duration_dtu != 0 &&
			next_dtu - sched_region->start_dtu >=
				sched_region->duration_dtu)) {
		sched->current_index++;
		changed = 1;
		once = false;

		/* No more region, need a new schedule. */
		if (sched->current_index >= sched->n_regions) {
			/* Reduce the schedule duration when not fully used. */
			if (sched_region->once && sched_region->duration_dtu) {
				sched->duration_dtu =
					next_timestamp_dtu -
					sched->start_timestamp_dtu;
			}
			return mcps802154_schedule_update(local,
							  next_timestamp_dtu);
		}

		sched_region = &sched->regions[sched->current_index];
	}

	return changed;
}

/**
 * mcps802154_ca_nothing() - Fill and return a nothing access.
 * @local: MCPS private data.
 * @timestamp_dtu: Start of nothing period.
 * @duration_dtu: Duration of nothing period, or 0 for endless.
 *
 * Return: Pointer to access allocated inside the context.
 */
struct mcps802154_access *mcps802154_ca_nothing(struct mcps802154_local *local,
						u32 timestamp_dtu,
						int duration_dtu)
{
	struct mcps802154_access *access = &local->ca.idle_access;

	access->method = MCPS802154_ACCESS_METHOD_NOTHING;
	access->common_ops = &ca_access_ops;
	access->timestamp_dtu = timestamp_dtu;
	access->duration_dtu = duration_dtu;
	return access;
}

struct mcps802154_access *
mcps802154_ca_get_access(struct mcps802154_local *local, u32 next_timestamp_dtu)
{
	struct mcps802154_schedule *sched = &local->ca.schedule;
	struct mcps802154_schedule_region *sched_region;
	struct mcps802154_region *region;
	struct mcps802154_access *access;
	u32 idle_timestamp_dtu, region_start_timestamp_dtu;
	int next_in_region_dtu, region_duration_dtu;
	int r, changed;

	local->ca.held = false;

	trace_ca_get_access(local, next_timestamp_dtu);

	if (local->ca.reset) {
		mcps802154_schedule_clear(local);
		local->ca.reset = false;
	}

	/* Do not examine accesses later than this date. */
	idle_timestamp_dtu = next_timestamp_dtu + local->llhw.idle_dtu;
	while (1) {
		/* Need a schedule. */
		if (!sched->n_regions)
			r = mcps802154_schedule_update(local,
						       next_timestamp_dtu);
		else
			/* Need a region. */
			r = mcps802154_ca_next_region(local,
						      next_timestamp_dtu);

		/* Stay in IDLE when no schedule. */
		if (r == -ENOENT)
			return mcps802154_ca_nothing(local, next_timestamp_dtu,
						     0);
		else if (r < 0)
			return NULL;

		changed = r;
		sched_region = &sched->regions[sched->current_index];
		region = sched_region->region;
		region_start_timestamp_dtu =
			sched->start_timestamp_dtu + sched_region->start_dtu;
		region_duration_dtu = sched_region->duration_dtu;

		if (changed) {
			if (is_before_dtu(next_timestamp_dtu,
					  region_start_timestamp_dtu)) {
				/* Access date may be postponed. */
				next_timestamp_dtu = region_start_timestamp_dtu;
			}
		}

		/* Get access. */
		if (region_duration_dtu)
			next_in_region_dtu =
				next_timestamp_dtu - region_start_timestamp_dtu;
		else
			next_in_region_dtu = 0;
		trace_region_get_access(local, region, next_timestamp_dtu,
					next_in_region_dtu,
					region_duration_dtu);
		access = region->ops->get_access(region, next_timestamp_dtu,
						 next_in_region_dtu,
						 region_duration_dtu);

		if (access)
			return access;

		/* If no access is found, look for next region, or wait. */
		if (region_duration_dtu) {
			u32 region_end_timestamp_dtu =
				region_start_timestamp_dtu +
				region_duration_dtu;

			if (is_before_dtu(idle_timestamp_dtu,
					  region_end_timestamp_dtu)) {
				return mcps802154_ca_nothing(
					local, next_timestamp_dtu,
					region_end_timestamp_dtu -
						next_timestamp_dtu);
			}

			/* Continue after the current region. */
			next_timestamp_dtu = region_end_timestamp_dtu;
		} else {
			return mcps802154_ca_nothing(local, next_timestamp_dtu,
						     0);
		}
	}
}

void mcps802154_ca_may_reschedule(struct mcps802154_local *local)
{
	if (!local->ca.held)
		mcps802154_fproc_schedule_change(local);
}

void mcps802154_ca_access_hold(struct mcps802154_local *local)
{
	local->ca.held = true;
}

void mcps802154_ca_invalidate_schedule(struct mcps802154_local *local)
{
	local->ca.reset = true;

	mcps802154_fproc_schedule_change(local);
}
