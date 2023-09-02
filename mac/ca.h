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

#ifndef NET_MCPS802154_CA_H
#define NET_MCPS802154_CA_H

#include <linux/atomic.h>

#include "schedule.h"

struct mcps802154_local;

/**
 * struct mcps802154_ca - CA private data.
 */
struct mcps802154_ca {
	/**
	 * @schedule: Current schedule.
	 */
	struct mcps802154_schedule schedule;
	/**
	 * @scheduler: Scheduler responsible to maintain the schedule
	 * or NULL when not chosen yet.
	 */
	struct mcps802154_scheduler *scheduler;
	/**
	 * @regions: List of regions currently available in the schedule.
	 */
	struct list_head regions;
	/**
	 * @n_regions: current number of opened regions.
	 */
	int n_regions;
	/**
	 * @held: Whether access is currently held and cannot change.
	 */
	bool held;
	/**
	 * @reset: Whether the schedule was invalidated and need to be changed.
	 */
	bool reset;
	/**
	 * @idle_access: Access used to wait when there is nothing to do.
	 */
	struct mcps802154_access idle_access;
};

/**
 * mcps802154_ca_init() - Initialize CA.
 * @local: MCPS private data.
 */
void mcps802154_ca_init(struct mcps802154_local *local);

/**
 * mcps802154_ca_uninit() - Uninitialize CA.
 * @local: MCPS private data.
 */
void mcps802154_ca_uninit(struct mcps802154_local *local);

/**
 * mcps802154_ca_start() - Start device.
 * @local: MCPS private data.
 *
 * FSM mutex should be locked.
 *
 * Return: 0 or error.
 */
int mcps802154_ca_start(struct mcps802154_local *local);

/**
 * mcps802154_ca_stop() - Request to stop device.
 * @local: MCPS private data.
 *
 * FSM mutex should be locked.
 *
 * This is asynchronous, caller needs to wait !local->started.
 */
void mcps802154_ca_stop(struct mcps802154_local *local);

/**
 * mcps802154_ca_notify_stop() - Notify that device has been stopped.
 * @local: MCPS private data.
 *
 * FSM mutex should be locked.
 *
 */
void mcps802154_ca_notify_stop(struct mcps802154_local *local);

/**
 * mcps802154_ca_close() - Request to close all the schedules.
 * @local: MCPS private data.
 *
 * FSM mutex should be locked.
 */
void mcps802154_ca_close(struct mcps802154_local *local);

/**
 * mcps802154_ca_set_scheduler() - Set the scheduler responsible for managing
 * the schedule, and configure its parameters.
 * @local: MCPS private data.
 * @name: Scheduler name.
 * @params_attr: Nested attribute containing region parameters. May be NULL.
 * @extack: Extended ACK report structure.
 *
 * FSM mutex should be locked.
 *
 * Device should not be started for the moment.
 *
 * Return: 0 or error.
 */
int mcps802154_ca_set_scheduler(struct mcps802154_local *local,
				const char *name,
				const struct nlattr *params_attr,
				struct netlink_ext_ack *extack);

/**
 * mcps802154_ca_set_region() - Set scheduler's region.
 * @local: MCPS private data.
 * @scheduler_name: Scheduler name.
 * @region_id: Identifier of the region, scheduler specific.
 * @region_name: Name of region to attach to the scheduler.
 * @params_attr: Nested attribute containing region parameters.
 * @extack: Extended ACK report structure.
 *
 * FSM mutex should be locked.
 *
 * Return: 0 or error.
 */
int mcps802154_ca_set_region(struct mcps802154_local *local,
			     const char *scheduler_name, u32 region_id,
			     const char *region_name,
			     const struct nlattr *params_attr,
			     struct netlink_ext_ack *extack);

/**
 * mcps802154_ca_scheduler_set_parameters() - Set the scheduler parameters.
 * @local: MCPS private data.
 * @name: Scheduler name.
 * @params_attr: Nested attribute containing region parameters.
 * @extack: Extended ACK report structure.
 *
 * FSM mutex should be locked.
 *
 * Return: 0 or error.
 */
int mcps802154_ca_scheduler_set_parameters(struct mcps802154_local *local,
					   const char *name,
					   const struct nlattr *params_attr,
					   struct netlink_ext_ack *extack);

/**
 * mcps802154_ca_scheduler_call() - Call scheduler specific procedure.
 * @local: MCPS private data.
 * @scheduler_name: Scheduler name.
 * @call_id: Identifier of the procedure, scheduler specific.
 * @params_attr: Nested attribute containing procedure parameters.
 * @info: Request information.
 *
 * FSM mutex should be locked.
 *
 * Return: 0 or error.
 */
int mcps802154_ca_scheduler_call(struct mcps802154_local *local,
				 const char *scheduler_name, u32 call_id,
				 const struct nlattr *params_attr,
				 const struct genl_info *info);

/**
 * mcps802154_ca_set_region_parameters() - Set the region parameters.
 * @local: MCPS private data.
 * @scheduler_name: Scheduler name.
 * @region_id: Identifier of the region, scheduler specific.
 * @region_name: Name of the region to call.
 * @params_attr: Nested attribute containing region parameters.
 * @extack: Extended ACK report structure.
 *
 * FSM mutex should be locked.
 *
 * Return: 0 or error.
 */
int mcps802154_ca_set_region_parameters(struct mcps802154_local *local,
					const char *scheduler_name,
					u32 region_id, const char *region_name,
					const struct nlattr *params_attr,
					struct netlink_ext_ack *extack);

/**
 * mcps802154_ca_call_region() - Call region specific procedure.
 * @local: MCPS private data.
 * @scheduler_name: Scheduler name.
 * @region_id: Identifier of the region, scheduler specific.
 * @region_name: Name of the region to call.
 * @call_id: Identifier of the procedure, region specific.
 * @params_attr: Nested attribute containing procedure parameters.
 * @info: Request information.
 *
 * FSM mutex should be locked.
 *
 * Return: 0 or error.
 */
int mcps802154_ca_call_region(struct mcps802154_local *local,
			      const char *scheduler_name, u32 region_id,
			      const char *region_name, u32 call_id,
			      const struct nlattr *params_attr,
			      const struct genl_info *info);

/**
 * mcps802154_ca_xmit_skb() - Transmit the buffer through the first region
 * that accepts it.
 * @local: MCPS private data.
 * @skb: Buffer to be transmitted.
 *
 * Return: 0 or error.
 */
int mcps802154_ca_xmit_skb(struct mcps802154_local *local, struct sk_buff *skb);

/**
 * mcps802154_ca_get_access() - Compute and return access.
 * @local: MCPS private data.
 * @next_timestamp_dtu: Date of next access opportunity.
 *
 * Return: A pointer to current access.
 */
struct mcps802154_access *
mcps802154_ca_get_access(struct mcps802154_local *local,
			 u32 next_timestamp_dtu);

/**
 * mcps802154_ca_may_reschedule() - If needed, request FProc to change access.
 * @local: MCPS private data.
 *
 * FSM mutex should be locked.
 *
 * When something has changed that could impact the current access, this
 * function should be called to evaluate the change and notify FProc. This
 * should be done for example when a new frame is queued.
 */
void mcps802154_ca_may_reschedule(struct mcps802154_local *local);

/**
 * mcps802154_ca_access_hold() - Prevent any reschedule until access is done.
 * @local: MCPS private data.
 */
void mcps802154_ca_access_hold(struct mcps802154_local *local);

/**
 * mcps802154_ca_invalidate_schedule() - Invalidate the schedule and force update.
 * @local: MCPS private data.
 *
 * FSM mutex should be locked.
 *
 * When something has changed that impact the current schedule, this function
 * can be called to invalidate the schedule and force an update.
 * The update will be done after the end of current access.
 * This API should be called for example when a region parameter changes.
 */
void mcps802154_ca_invalidate_schedule(struct mcps802154_local *local);

#endif /* NET_MCPS802154_CA_H */
