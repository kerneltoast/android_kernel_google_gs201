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
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string.h>

#include "mcps802154_i.h"
#include "schedulers.h"

static LIST_HEAD(registered_schedulers);
static DEFINE_MUTEX(registered_schedulers_lock);

int mcps802154_scheduler_register(struct mcps802154_scheduler_ops *scheduler_ops)
{
	struct mcps802154_scheduler_ops *ops;
	int r = 0;

	if (WARN_ON(!scheduler_ops || !scheduler_ops->owner ||
		    !scheduler_ops->name || !scheduler_ops->open ||
		    !scheduler_ops->close || !scheduler_ops->update_schedule))
		return -EINVAL;

	mutex_lock(&registered_schedulers_lock);

	list_for_each_entry (ops, &registered_schedulers, registered_entry) {
		if (WARN_ON(strcmp(ops->name, scheduler_ops->name) == 0)) {
			r = -EBUSY;
			goto unlock;
		}
	}

	list_add(&scheduler_ops->registered_entry, &registered_schedulers);

unlock:
	mutex_unlock(&registered_schedulers_lock);

	return r;
}
EXPORT_SYMBOL(mcps802154_scheduler_register);

void mcps802154_scheduler_unregister(
	struct mcps802154_scheduler_ops *scheduler_ops)
{
	mutex_lock(&registered_schedulers_lock);
	list_del(&scheduler_ops->registered_entry);
	mutex_unlock(&registered_schedulers_lock);
}
EXPORT_SYMBOL(mcps802154_scheduler_unregister);

struct mcps802154_scheduler *
mcps802154_scheduler_open(struct mcps802154_local *local, const char *name,
			  const struct nlattr *params_attr,
			  struct netlink_ext_ack *extack)
{
	struct mcps802154_scheduler_ops *ops;
	struct mcps802154_scheduler *scheduler;
	bool found = false;

	mutex_lock(&registered_schedulers_lock);

	list_for_each_entry (ops, &registered_schedulers, registered_entry) {
		if (strcmp(ops->name, name) == 0) {
			if (try_module_get(ops->owner))
				found = true;
			break;
		}
	}

	mutex_unlock(&registered_schedulers_lock);

	if (!found)
		return NULL;

	scheduler = ops->open(&local->llhw);
	if (!scheduler) {
		module_put(ops->owner);
		return NULL;
	}

	scheduler->ops = ops;
	if (mcps802154_scheduler_set_parameters(scheduler, params_attr,
						extack)) {
		ops->close(scheduler);
		return NULL;
	}

	return scheduler;
}

void mcps802154_scheduler_close(struct mcps802154_scheduler *scheduler)
{
	const struct mcps802154_scheduler_ops *ops;

	ops = scheduler->ops;
	ops->close(scheduler);
	module_put(ops->owner);
}

void mcps802154_scheduler_notify_stop(struct mcps802154_scheduler *scheduler)
{
	const struct mcps802154_scheduler_ops *ops;

	if (!scheduler)
		return;

	ops = scheduler->ops;
	if (ops->notify_stop)
		ops->notify_stop(scheduler);
}

int mcps802154_scheduler_set_parameters(struct mcps802154_scheduler *scheduler,
					const struct nlattr *params_attr,
					struct netlink_ext_ack *extack)
{
	if (!params_attr)
		return 0;

	if (!scheduler->ops->set_parameters)
		return -EOPNOTSUPP;

	return scheduler->ops->set_parameters(scheduler, params_attr, extack);
}

int mcps802154_scheduler_call(struct mcps802154_scheduler *scheduler,
			      u32 call_id, const struct nlattr *params_attr,
			      const struct genl_info *info)
{
	if (!scheduler->ops->call)
		return -EOPNOTSUPP;

	return scheduler->ops->call(scheduler, call_id, params_attr, info);
}
