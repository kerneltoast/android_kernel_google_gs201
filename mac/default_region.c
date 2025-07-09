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
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/netdevice.h>

#include <net/mcps802154_schedule.h>

#include "mcps802154_i.h"

#include "default_region.h"
#include "warn_return.h"

static struct mcps802154_region_ops default_region_ops;

static void
mcps802154_default_rx_frame(struct mcps802154_access *access, int frame_idx,
			    struct sk_buff *skb,
			    const struct mcps802154_rx_frame_info *info,
			    enum mcps802154_rx_error_type error)
{
	struct default_local *local = access_to_local(access);

	mcps802154_region_rx_skb(local->llhw, &local->region, skb, info->lqi);
}

static struct sk_buff *
mcps802154_default_tx_frame(struct mcps802154_access *access, int frame_idx)
{
	struct default_local *local = access_to_local(access);

	return skb_dequeue(&local->queue);
}

static void
mcps802154_default_tx_return(struct mcps802154_access *access, int frame_idx,
			     struct sk_buff *skb,
			     enum mcps802154_access_tx_return_reason reason)
{
	struct default_local *local = access_to_local(access);
	struct mcps802154_local *mlocal = llhw_to_local(local->llhw);

	if (reason == MCPS802154_ACCESS_TX_RETURN_REASON_FAILURE) {
		local->retries++;
		if (local->retries <= mlocal->pib.mac_max_frame_retries) {
			/* Retry the frame. */
			skb_queue_head(&local->queue, skb);
		} else {
			local->retries = 0;
			mcps802154_region_xmit_done(local->llhw, &local->region,
						    skb, false);
		}
	} else if (reason == MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL) {
		skb_queue_head(&local->queue, skb);
	} else {
		local->retries = 0;
		mcps802154_region_xmit_done(local->llhw, &local->region, skb,
					    true);
	}
}

struct mcps802154_access_ops default_access_ops = {
	.rx_frame = mcps802154_default_rx_frame,
	.tx_get_frame = mcps802154_default_tx_frame,
	.tx_return = mcps802154_default_tx_return,
};

static struct mcps802154_region *
mcps802154_default_open(struct mcps802154_llhw *llhw)
{
	struct default_local *local;

	local = kmalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;

	local->llhw = llhw;
	local->region.ops = &default_region_ops;
	skb_queue_head_init(&local->queue);
	local->retries = 0;
	return &local->region;
}

static void mcps802154_default_close(struct mcps802154_region *region)
{
	struct default_local *local = region_to_local(region);

	skb_queue_purge(&local->queue);
	kfree(local);
}

static void mcps802154_default_notify_stop(struct mcps802154_region *region)
{
	struct default_local *local = region_to_local(region);

	skb_queue_purge(&local->queue);
}

static struct mcps802154_access *
mcps802154_default_get_access(struct mcps802154_region *region,
			      u32 next_timestamp_dtu, int next_in_region_dtu,
			      int region_duration_dtu)
{
	struct default_local *local = region_to_local(region);

	local->access.method = skb_queue_empty(&local->queue) ?
				       MCPS802154_ACCESS_METHOD_IMMEDIATE_RX :
				       MCPS802154_ACCESS_METHOD_IMMEDIATE_TX;
	local->access.ops = &default_access_ops;
	return &local->access;
}

static int mcps802154_default_xmit_skb(struct mcps802154_region *region,
				       struct sk_buff *skb)
{
	struct default_local *local = region_to_local(region);

	skb_queue_tail(&local->queue, skb);
	return 1;
}

static struct mcps802154_region_ops default_region_ops = {
	.owner = THIS_MODULE,
	.name = "default",
	.open = mcps802154_default_open,
	.close = mcps802154_default_close,
	.notify_stop = mcps802154_default_notify_stop,
	.get_access = mcps802154_default_get_access,
	.xmit_skb = mcps802154_default_xmit_skb
};

int __init mcps802154_default_region_init(void)
{
	return mcps802154_region_register(&default_region_ops);
}

void __exit mcps802154_default_region_exit(void)
{
	mcps802154_region_unregister(&default_region_ops);
}
