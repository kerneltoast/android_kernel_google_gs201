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

#include "mcps802154_i.h"
#include "llhw-ops.h"

static void mcps802154_fproc_stopped_enter(struct mcps802154_local *local)
{
	mcps802154_ca_notify_stop(local);
	local->started = false;
	wake_up(&local->wq);
}

static void mcps802154_fproc_stopped_leave(struct mcps802154_local *local)
{
	local->started = true;
}

static void mcps802154_fproc_stopped_ignore(struct mcps802154_local *local)
{
	WARN_ON_ONCE(1);
}

static void
mcps802154_fproc_stopped_ignore_rx_error(struct mcps802154_local *local,
					 enum mcps802154_rx_error_type error)
{
	mcps802154_fproc_stopped_ignore(local);
}

static void
mcps802154_fproc_stopped_schedule_change(struct mcps802154_local *local)
{
	int r;

	if (local->start_stop_request) {
		r = llhw_start(local);
		if (r)
			goto err;

		mcps802154_fproc_access_now(local);
		if (local->broken)
			goto err_stop;
	}

	return;

err_stop:
	local->start_stop_request = false;
	local->fproc.state->schedule_change(local);
err:
	/* Device is broken, but stopped. */
	WARN_ON(local->started);
}

const struct mcps802154_fproc_state mcps802154_fproc_stopped = {
	.name = "stopped",
	.enter = mcps802154_fproc_stopped_enter,
	.leave = mcps802154_fproc_stopped_leave,
	.rx_frame = mcps802154_fproc_stopped_ignore,
	.rx_timeout = mcps802154_fproc_stopped_ignore,
	.rx_error = mcps802154_fproc_stopped_ignore_rx_error,
	.tx_done = mcps802154_fproc_stopped_ignore,
	.broken = mcps802154_fproc_stopped_ignore,
	.schedule_change = mcps802154_fproc_stopped_schedule_change,
};

void mcps802154_fproc_stopped_handle(struct mcps802154_local *local)
{
	llhw_stop(local);
	mcps802154_fproc_access_reset(local);
	mcps802154_schedule_clear(local);
	mcps802154_fproc_change_state(local, &mcps802154_fproc_stopped);
}
