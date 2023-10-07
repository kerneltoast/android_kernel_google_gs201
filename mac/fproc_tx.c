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
#include <linux/ieee802154.h>
#include <linux/netdevice.h>

#include "mcps802154_fproc.h"
#include "mcps802154_i.h"
#include "llhw-ops.h"

static void mcps802154_fproc_tx_tx_done(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;

	access->ops->tx_return(access, 0, local->fproc.tx_skb,
			       MCPS802154_ACCESS_TX_RETURN_REASON_CONSUMED);
	local->fproc.tx_skb = NULL;

	mcps802154_fproc_access_done(local, false);

	/* Next access. */
	mcps802154_fproc_access_now(local);
}

static void mcps802154_fproc_tx_schedule_change(struct mcps802154_local *local)
{
	/* Nothing, wait TX done. */
}

static const struct mcps802154_fproc_state mcps802154_fproc_tx = {
	.name = "tx",
	.tx_done = mcps802154_fproc_tx_tx_done,
	.schedule_change = mcps802154_fproc_tx_schedule_change,
};

static void mcps802154_fproc_tx_wack_rx_frame(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;

	/* Read frame. */
	struct sk_buff *skb = NULL;
	struct mcps802154_rx_frame_info info = {
		.flags = MCPS802154_RX_FRAME_INFO_LQI,
	};
	access->error = llhw_rx_get_frame(local, &skb, &info);
	if (!access->error) {
		/* Is it an ack frame? With same seq number? */
		if (IEEE802154_FC_TYPE(skb->data[0]) ==
				IEEE802154_FC_TYPE_ACK &&
				skb->data[IEEE802154_FC_LEN] ==
				local->fproc.tx_skb->data[IEEE802154_FC_LEN]) {
			/* Ack frame. */
			access->ops->tx_return(
					access, 0, local->fproc.tx_skb,
					MCPS802154_ACCESS_TX_RETURN_REASON_CONSUMED);
		} else {
			/* Not an ack or read failure or a bad sequence number. */
			access->ops->tx_return(
					access, 0, local->fproc.tx_skb,
					MCPS802154_ACCESS_TX_RETURN_REASON_FAILURE);
		}
		dev_kfree_skb_any(skb);
		local->fproc.tx_skb = NULL;
		mcps802154_fproc_access_done(local, false);
		mcps802154_fproc_access_now(local);
	} else if (access->error && mcps802154_fproc_is_non_recoverable_error(access)) {
		mcps802154_fproc_broken_handle(local);
	} else {
		mcps802154_fproc_access_done(local, false);
		mcps802154_fproc_access_now(local);
	}
}

/* Used for error and time-out. */
static void mcps802154_fproc_tx_wack_rx_timeout(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;

	access->ops->tx_return(access, 0, local->fproc.tx_skb,
			       MCPS802154_ACCESS_TX_RETURN_REASON_FAILURE);
	local->fproc.tx_skb = NULL;

	mcps802154_fproc_access_done(local, false);

	/* Next access. */
	mcps802154_fproc_access_now(local);
}

static void
mcps802154_fproc_tx_wack_rx_error(struct mcps802154_local *local,
				  enum mcps802154_rx_error_type error)
{
	mcps802154_fproc_tx_wack_rx_timeout(local);
}

static void mcps802154_fproc_tx_wack_tx_done(struct mcps802154_local *local)
{
	/* Nothing, wait for ack. */
}

static void
mcps802154_fproc_tx_wack_schedule_change(struct mcps802154_local *local)
{
	/* Nothing, wait for ack. */
}

static const struct mcps802154_fproc_state mcps802154_fproc_tx_wack = {
	.name = "tx_wack",
	.rx_frame = mcps802154_fproc_tx_wack_rx_frame,
	.rx_timeout = mcps802154_fproc_tx_wack_rx_timeout,
	.rx_error = mcps802154_fproc_tx_wack_rx_error,
	.tx_done = mcps802154_fproc_tx_wack_tx_done,
	.schedule_change = mcps802154_fproc_tx_wack_schedule_change,
};

#define IEEE802154_AIFS_DURATION_SYMBOLS 12

int mcps802154_fproc_tx_handle(struct mcps802154_local *local,
			       struct mcps802154_access *access)
{
	int r;
	u8 ack_req;
	struct mcps802154_tx_frame_config tx_config = {};
	struct sk_buff *skb = access->ops->tx_get_frame(access, 0);

	if (!skb)
		return -ENOMEM;

	ack_req = skb->data[0] & IEEE802154_FC_ACK_REQ;
	if (ack_req) {
		tx_config.rx_enable_after_tx_dtu =
			IEEE802154_AIFS_DURATION_SYMBOLS *
			local->llhw.symbol_dtu;
	}

	r = llhw_tx_frame(local, skb, &tx_config, 0, 0);
	if (r) {
		access->ops->tx_return(
			access, 0, skb,
			MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL);
		return r;
	}

	local->fproc.tx_skb = skb;
	mcps802154_ca_access_hold(local);
	if (ack_req)
		mcps802154_fproc_change_state(local, &mcps802154_fproc_tx_wack);
	else
		mcps802154_fproc_change_state(local, &mcps802154_fproc_tx);
	return 0;
}
