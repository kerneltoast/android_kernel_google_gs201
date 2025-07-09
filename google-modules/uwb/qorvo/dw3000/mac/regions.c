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
#include <linux/netdevice.h>

#include "mcps802154_i.h"
#include "trace.h"

static LIST_HEAD(registered_regions);
static DEFINE_MUTEX(registered_regions_lock);

int mcps802154_region_register(struct mcps802154_region_ops *region_ops)
{
	struct mcps802154_region_ops *ops;
	int r = 0;

	if (WARN_ON(!region_ops || !region_ops->owner || !region_ops->name ||
		    !region_ops->open || !region_ops->close ||
		    !region_ops->get_access))
		return -EINVAL;

	mutex_lock(&registered_regions_lock);

	list_for_each_entry (ops, &registered_regions, registered_entry) {
		if (WARN_ON(strcmp(ops->name, region_ops->name) == 0)) {
			r = -EBUSY;
			goto unlock;
		}
	}

	list_add(&region_ops->registered_entry, &registered_regions);

unlock:
	mutex_unlock(&registered_regions_lock);

	return r;
}
EXPORT_SYMBOL_GPL(mcps802154_region_register);

void mcps802154_region_unregister(struct mcps802154_region_ops *region_ops)
{
	mutex_lock(&registered_regions_lock);
	list_del(&region_ops->registered_entry);
	mutex_unlock(&registered_regions_lock);
}
EXPORT_SYMBOL_GPL(mcps802154_region_unregister);

struct mcps802154_region *
mcps802154_region_open(struct mcps802154_llhw *llhw, const char *name,
		       const struct nlattr *params_attr,
		       struct netlink_ext_ack *extack)
{
	struct mcps802154_region_ops *ops;
	struct mcps802154_region *region;
	bool found = false;

	mutex_lock(&registered_regions_lock);

	list_for_each_entry (ops, &registered_regions, registered_entry) {
		if (strcmp(ops->name, name) == 0) {
			if (try_module_get(ops->owner))
				found = true;
			break;
		}
	}

	mutex_unlock(&registered_regions_lock);

	if (!found)
		return NULL;

	region = ops->open(llhw);
	if (!region) {
		module_put(ops->owner);
		return NULL;
	}

	region->ops = ops;
	if (mcps802154_region_set_parameters(llhw, region, params_attr,
					     extack)) {
		ops->close(region);
		return NULL;
	}

	return region;
}
EXPORT_SYMBOL_GPL(mcps802154_region_open);

void mcps802154_region_close(struct mcps802154_llhw *llhw,
			     struct mcps802154_region *region)
{
	const struct mcps802154_region_ops *ops;

	ops = region->ops;
	ops->close(region);
	module_put(ops->owner);
}
EXPORT_SYMBOL_GPL(mcps802154_region_close);

void mcps802154_region_notify_stop(struct mcps802154_llhw *llhw,
				   struct mcps802154_region *region)
{
	if (!region->ops->notify_stop)
		return;

	trace_region_notify_stop(region);
	region->ops->notify_stop(region);
}
EXPORT_SYMBOL_GPL(mcps802154_region_notify_stop);

int mcps802154_region_set_parameters(struct mcps802154_llhw *llhw,
				     struct mcps802154_region *region,
				     const struct nlattr *params_attr,
				     struct netlink_ext_ack *extack)
{
	if (!params_attr)
		return 0;

	if (!region->ops->set_parameters)
		return -EOPNOTSUPP;

	return region->ops->set_parameters(region, params_attr, extack);
}
EXPORT_SYMBOL_GPL(mcps802154_region_set_parameters);

int mcps802154_region_call(struct mcps802154_llhw *llhw,
			   struct mcps802154_region *region, u32 call_id,
			   const struct nlattr *params_attr,
			   const struct genl_info *info)
{
	if (!region->ops->call)
		return -EOPNOTSUPP;

	return region->ops->call(region, call_id, params_attr, info);
}
EXPORT_SYMBOL_GPL(mcps802154_region_call);

int mcps802154_region_get_demand(struct mcps802154_llhw *llhw,
				 struct mcps802154_region *region,
				 u32 next_timestamp_dtu,
				 struct mcps802154_region_demand *demand)
{
	int r;

	if (!region->ops->get_demand)
		return -EOPNOTSUPP;

	trace_region_get_demand(region, next_timestamp_dtu);
	r = region->ops->get_demand(region, next_timestamp_dtu, demand);
	trace_region_get_demand_return(region, demand, r);

	return r;
}
EXPORT_SYMBOL_GPL(mcps802154_region_get_demand);

void mcps802154_region_xmit_done(struct mcps802154_llhw *llhw,
				 struct mcps802154_region *region,
				 struct sk_buff *skb, bool ok)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	if (ok) {
		ieee802154_xmit_complete(local->hw, skb, false);
	} else {
		ieee802154_wake_queue(local->hw);
		dev_kfree_skb_any(skb);
	}
}
EXPORT_SYMBOL_GPL(mcps802154_region_xmit_done);

void mcps802154_region_rx_skb(struct mcps802154_llhw *llhw,
			      struct mcps802154_region *region,
			      struct sk_buff *skb, u8 lqi)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	ieee802154_rx_irqsafe(local->hw, skb, lqi);
}
EXPORT_SYMBOL_GPL(mcps802154_region_rx_skb);

int mcps802154_region_deferred(struct mcps802154_llhw *llhw,
			       struct mcps802154_region *region)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	if (local->fproc.deferred && local->fproc.deferred != region)
		return -EINVAL;

	local->fproc.deferred = region;

	return 0;
}
EXPORT_SYMBOL_GPL(mcps802154_region_deferred);
