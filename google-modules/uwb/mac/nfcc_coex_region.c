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
#include <linux/math64.h>
#include <linux/printk.h>

#include <linux/netdevice.h>

#include <net/mcps802154_schedule.h>
#include "net/nfcc_coex_region_nl.h"

#include "nfcc_coex_region.h"
#include "nfcc_coex_region_call.h"
#include "nfcc_coex_access.h"
#include "nfcc_coex_session.h"
#include "nfcc_coex_trace.h"

static struct mcps802154_region_ops nfcc_coex_region_ops;

static struct mcps802154_region *nfcc_coex_open(struct mcps802154_llhw *llhw)
{
	struct nfcc_coex_local *local;

	local = kzalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;

	local->llhw = llhw;
	local->region.ops = &nfcc_coex_region_ops;
	local->session.state = NFCC_COEX_STATE_IDLE;

	return &local->region;
}

static void nfcc_coex_close(struct mcps802154_region *region)
{
	struct nfcc_coex_local *local = region_to_local(region);

	kfree(local);
}

static void nfcc_coex_notify_stop(struct mcps802154_region *region)
{
	struct nfcc_coex_local *local = region_to_local(region);

	trace_region_nfcc_coex_notify_stop(local);
}

static int nfcc_coex_call(struct mcps802154_region *region, u32 call_id,
			  const struct nlattr *attrs,
			  const struct genl_info *info)
{
	struct nfcc_coex_local *local = region_to_local(region);

	trace_region_nfcc_coex_call(local, call_id);
	switch (call_id) {
	case NFCC_COEX_CALL_CCC_SESSION_START:
	case NFCC_COEX_CALL_CCC_SESSION_STOP:
		return nfcc_coex_session_control(local, call_id, attrs, info);
	default:
		return -EINVAL;
	}
}

static int nfcc_coex_get_demand(struct mcps802154_region *region,
				u32 next_timestamp_dtu,
				struct mcps802154_region_demand *demand)
{
	struct nfcc_coex_local *local = region_to_local(region);
	const struct nfcc_coex_session *session = &local->session;
	const struct mcps802154_region_demand *rd = &session->region_demand;

	demand->max_duration_dtu = 0;

	switch (session->state) {
	case NFCC_COEX_STATE_STARTED:
		if (is_before_dtu(rd->timestamp_dtu, next_timestamp_dtu))
			demand->timestamp_dtu = next_timestamp_dtu;
		else
			demand->timestamp_dtu = rd->timestamp_dtu;
		return 1;

	case NFCC_COEX_STATE_STOPPING:
		if (session->first_access) {
			if (is_before_dtu(rd->timestamp_dtu,
					  next_timestamp_dtu))
				demand->timestamp_dtu = next_timestamp_dtu;
			else
				demand->timestamp_dtu = rd->timestamp_dtu;
		} else
			demand->timestamp_dtu = next_timestamp_dtu;
		return 1;

	default:
		return 0;
	}
}

void nfcc_coex_set_state(struct nfcc_coex_local *local,
			 enum nfcc_coex_state new_state)
{
	struct nfcc_coex_session *session = &local->session;

	trace_region_nfcc_coex_set_state(local, new_state);
	session->state = new_state;
}

void nfcc_coex_report(struct nfcc_coex_local *local)
{
	struct nfcc_coex_session *session = &local->session;
	const struct llhw_vendor_cmd_nfcc_coex_get_access_info *get_access_info =
		&session->get_access_info;
	struct sk_buff *msg;
	int r;

	trace_region_nfcc_coex_report(local, get_access_info);
	msg = mcps802154_region_event_alloc_skb(
		local->llhw, &local->region,
		NFCC_COEX_CALL_CCC_SESSION_NOTIFICATION, session->event_portid,
		NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return;

#define P(attr, type, value)                                            \
	do {                                                            \
		if (nla_put_##type(msg, NFCC_COEX_CALL_ATTR_CCC_##attr, \
				   value)) {                            \
			goto nla_put_failure;                           \
		}                                                       \
	} while (0)

	P(WATCHDOG_TIMEOUT, u8, get_access_info->watchdog_timeout);
	P(STOPPED, u8, get_access_info->stop);
#undef P

	r = mcps802154_region_event(local->llhw, msg);
	if (r == -ECONNREFUSED)
		/* TODO stop. */
		;
	return;

nla_put_failure:
	trace_region_nfcc_coex_report_nla_put_failure(local);
	kfree_skb(msg);
}

static struct mcps802154_region_ops nfcc_coex_region_ops = {
	/* clang-format off */
	.owner = THIS_MODULE,
	.name = "nfcc_coex",
	.open = nfcc_coex_open,
	.close = nfcc_coex_close,
	.notify_stop = nfcc_coex_notify_stop,
	.call = nfcc_coex_call,
	.get_access = nfcc_coex_get_access,
	.get_demand = nfcc_coex_get_demand,
	/* clang-format on */
};

int __init nfcc_coex_region_init(void)
{
	return mcps802154_region_register(&nfcc_coex_region_ops);
}

void __exit nfcc_coex_region_exit(void)
{
	mcps802154_region_unregister(&nfcc_coex_region_ops);
}

module_init(nfcc_coex_region_init);
module_exit(nfcc_coex_region_exit);

MODULE_DESCRIPTION("Vendor Region for IEEE 802.15.4 MCPS");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
