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
#include <linux/printk.h>

#include "mcps802154_i.h"
#include "trace.h"

static void mcps802154_fproc_broken_enter(struct mcps802154_local *local)
{
	trace_fproc_broken_enter(local);
	pr_err_ratelimited("mcps802154: entering broken state for %s\n",
			   wpan_phy_name(local->hw->phy));
	local->broken = true;
}

static void mcps802154_fproc_broken_leave(struct mcps802154_local *local)
{
	local->broken = false;
}

static void mcps802154_fproc_broken_ignore(struct mcps802154_local *local)
{
}

static void
mcps802154_fproc_broken_ignore_rx_error(struct mcps802154_local *local,
					enum mcps802154_rx_error_type error)
{
}

static void
mcps802154_fproc_broken_schedule_change(struct mcps802154_local *local)
{
	if (!local->start_stop_request)
		mcps802154_fproc_stopped_handle(local);
}

static const struct mcps802154_fproc_state mcps802154_fproc_broken = {
	.name = "broken",
	.enter = mcps802154_fproc_broken_enter,
	.leave = mcps802154_fproc_broken_leave,
	.rx_frame = mcps802154_fproc_broken_ignore,
	.rx_timeout = mcps802154_fproc_broken_ignore,
	.rx_error = mcps802154_fproc_broken_ignore_rx_error,
	.tx_done = mcps802154_fproc_broken_ignore,
	.broken = mcps802154_fproc_broken_ignore,
	.schedule_change = mcps802154_fproc_broken_schedule_change,
};

void mcps802154_fproc_broken_handle(struct mcps802154_local *local)
{
	if (!local->start_stop_request)
		/* Try to stop anyway. */
		mcps802154_fproc_stopped_handle(local);
	else
		mcps802154_fproc_change_state(local, &mcps802154_fproc_broken);
}
