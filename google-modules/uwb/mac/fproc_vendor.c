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

#include "mcps802154_i.h"

static void
mcps802154_fproc_vendor_handle_callback_return(struct mcps802154_local *local,
					       int r)
{
	struct mcps802154_access *access = local->fproc.access;
	/* Fetch/copy needed data before access_done.
	 * Avoid usage of access pointer after access done call. */
	int duration_dtu = access->duration_dtu;
	u32 next_access_dtu = access->timestamp_dtu + duration_dtu;
	/* Filter-out the 'stop' request as error. */
	bool error = r != 1;

	if (!r)
		return;

	access->error = r;
	mcps802154_fproc_access_done(local, error);
	if (error) {
		mcps802154_fproc_broken_handle(local);
	} else if (duration_dtu) {
		mcps802154_fproc_access(local, next_access_dtu);
	} else {
		mcps802154_fproc_access_now(local);
	}
}

static void mcps802154_fproc_vendor_rx_frame(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;
	int r;

	if (access->vendor_ops->rx_frame)
		r = access->vendor_ops->rx_frame(access);
	else
		r = -EOPNOTSUPP;
	mcps802154_fproc_vendor_handle_callback_return(local, r);
}

static void mcps802154_fproc_vendor_rx_timeout(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;
	int r;

	if (access->vendor_ops->rx_timeout)
		r = access->vendor_ops->rx_timeout(access);
	else
		r = -EOPNOTSUPP;
	mcps802154_fproc_vendor_handle_callback_return(local, r);
}

static void
mcps802154_fproc_vendor_rx_error(struct mcps802154_local *local,
				 enum mcps802154_rx_error_type error)
{
	struct mcps802154_access *access = local->fproc.access;
	int r;

	if (access->vendor_ops->rx_error)
		r = access->vendor_ops->rx_error(access, error);
	else
		r = -EOPNOTSUPP;
	mcps802154_fproc_vendor_handle_callback_return(local, r);
}

static void mcps802154_fproc_vendor_tx_done(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;
	int r;

	if (access->vendor_ops->tx_done)
		r = access->vendor_ops->tx_done(access);
	else
		r = -EOPNOTSUPP;
	mcps802154_fproc_vendor_handle_callback_return(local, r);
}

static void mcps802154_fproc_vendor_broken(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;
	int r;

	if (access->vendor_ops->broken)
		r = access->vendor_ops->broken(access);
	else
		r = -EOPNOTSUPP;
	mcps802154_fproc_vendor_handle_callback_return(local, r);
}

static void
mcps802154_fproc_vendor_timer_expired(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;

	if (access->vendor_ops->timer_expired) {
		int r;

		r = access->vendor_ops->timer_expired(access);
		mcps802154_fproc_vendor_handle_callback_return(local, r);
	}
}

static void
mcps802154_fproc_vendor_schedule_change(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;

	if (access->vendor_ops->schedule_change) {
		int r;

		r = access->vendor_ops->schedule_change(access);
		mcps802154_fproc_vendor_handle_callback_return(local, r);
	}
}

static const struct mcps802154_fproc_state mcps802154_fproc_vendor = {
	.name = "vendor",
	.rx_frame = mcps802154_fproc_vendor_rx_frame,
	.rx_timeout = mcps802154_fproc_vendor_rx_timeout,
	.rx_error = mcps802154_fproc_vendor_rx_error,
	.tx_done = mcps802154_fproc_vendor_tx_done,
	.broken = mcps802154_fproc_vendor_broken,
	.timer_expired = mcps802154_fproc_vendor_timer_expired,
	.schedule_change = mcps802154_fproc_vendor_schedule_change,
};

int mcps802154_fproc_vendor_handle(struct mcps802154_local *local,
				   struct mcps802154_access *access)
{
	mcps802154_fproc_change_state(local, &mcps802154_fproc_vendor);

	if (access->vendor_ops->handle) {
		return access->vendor_ops->handle(access);
	}
	return 0;
}
