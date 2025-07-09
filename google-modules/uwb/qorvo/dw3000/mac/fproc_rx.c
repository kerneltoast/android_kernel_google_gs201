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

#include "mcps802154_fproc.h"
#include "mcps802154_i.h"
#include "llhw-ops.h"

static void
mcps802154_fproc_rx_wait_tx_done_tx_done(struct mcps802154_local *local)
{
	/* End current access and ask for next one. */
	mcps802154_fproc_access_done(local, false);
	mcps802154_fproc_access_now(local);
}

static void
mcps802154_fproc_rx_wait_tx_done_schedule_change(struct mcps802154_local *local)
{
	/* Nothing, wait for tx_done. */
}

static const struct mcps802154_fproc_state mcps802154_fproc_rx_wait_tx_done = {
	.name = "rx_wait_tx_done",
	.tx_done = mcps802154_fproc_rx_wait_tx_done_tx_done,
	.schedule_change = mcps802154_fproc_rx_wait_tx_done_schedule_change,
};

static void mcps802154_fproc_rx_rx_frame(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;

	/* Read frame. */
	struct sk_buff *skb = NULL;
	struct mcps802154_rx_frame_info info = {
		.flags = MCPS802154_RX_FRAME_INFO_LQI,
	};
	access->error = llhw_rx_get_frame(local, &skb, &info);
	if (!access->error) {
		access->ops->rx_frame(access, 0, skb, &info,
				      MCPS802154_RX_ERROR_NONE);
		/* If auto-ack was sent, wait for tx_done. */
		if (info.flags & MCPS802154_RX_FRAME_INFO_AACK) {
			mcps802154_fproc_change_state(
				local, &mcps802154_fproc_rx_wait_tx_done);
			return;
		}
	}
	if (access->error) {
		if (mcps802154_fproc_is_non_recoverable_error(access)) {
			mcps802154_fproc_access_done(local, true);
			mcps802154_fproc_broken_handle(local);
			return;
		}
	}
	/* Next access. */
	mcps802154_fproc_access_done(local, false);
	mcps802154_fproc_access_now(local);
}

static void mcps802154_fproc_rx_rx_error(struct mcps802154_local *local,
					 enum mcps802154_rx_error_type error)
{
	mcps802154_fproc_access_done(local, true);
	/* Next access. */
	mcps802154_fproc_access_now(local);
}

static void mcps802154_fproc_rx_schedule_change(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;
	/* Disable RX. */
	access->error = llhw_rx_disable(local);
	if (access->error == -EBUSY)
		/* Wait for RX result. */
		return;

	if (access->error) {
		if (mcps802154_fproc_is_non_recoverable_error(access)) {
			mcps802154_fproc_access_done(local, true);
			mcps802154_fproc_broken_handle(local);
			return;
		}
	}
		/* Next access. */
		mcps802154_fproc_access_done(local, false);
		mcps802154_fproc_access_now(local);
}

static const struct mcps802154_fproc_state mcps802154_fproc_rx = {
	.name = "rx",
	.rx_frame = mcps802154_fproc_rx_rx_frame,
	.rx_error = mcps802154_fproc_rx_rx_error,
	.schedule_change = mcps802154_fproc_rx_schedule_change,
};

int mcps802154_fproc_rx_handle(struct mcps802154_local *local,
			       struct mcps802154_access *access)
{
	struct mcps802154_rx_frame_config rx_config = {
		.flags = MCPS802154_RX_FRAME_CONFIG_AACK,
		.timeout_dtu = -1,
	};
	access->error = llhw_rx_enable(local, &rx_config, 0, 0);
	if (access->error)
		return access->error;

	mcps802154_fproc_change_state(local, &mcps802154_fproc_rx);

	return 0;
}
