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
#include <linux/module.h>

#include "mcps802154_i.h"
#include "llhw-ops.h"

bool mcps802154_fproc_is_non_recoverable_error(struct mcps802154_access *access)
{
	bool rc = false;
	if (access->error < 0) {
		switch (access->error) {
			case -ETIME:
			case -EIO:
			case -EAGAIN:
				rc = false;
				break;
			default:
				pr_err("mcps: error %d is not recoverable", access->error);
				rc = true;
				break;
		}
	}
	return rc;
}

void mcps802154_fproc_init(struct mcps802154_local *local)
{
	local->fproc.state = &mcps802154_fproc_stopped;
	WARN_ON(!local->fproc.state->enter);
	local->fproc.state->enter(local);
}

void mcps802154_fproc_uninit(struct mcps802154_local *local)
{
	WARN_ON(local->fproc.access);
	WARN_ON(local->fproc.tx_skb);
	WARN_ON(local->started);
	WARN_ON(local->fproc.deferred);
}

void mcps802154_fproc_change_state(
	struct mcps802154_local *local,
	const struct mcps802154_fproc_state *new_state)
{
	if (local->fproc.state->leave)
		local->fproc.state->leave(local);
	local->fproc.state = new_state;
	if (local->fproc.state->enter)
		local->fproc.state->enter(local);
}

void mcps802154_fproc_access(struct mcps802154_local *local,
			     u32 next_timestamp_dtu)
{
	struct mcps802154_access *access;

	if (!local->start_stop_request) {
		mcps802154_fproc_stopped_handle(local);
		return;
	}

	access = mcps802154_ca_get_access(local, next_timestamp_dtu);
	if (unlikely(!access)) {
		mcps802154_fproc_broken_handle(local);
		return;
	}

	local->fproc.access = access;
	local->fproc.frame_idx = 0;

	switch (access->method) {
	case MCPS802154_ACCESS_METHOD_NOTHING:
		access->error = mcps802154_fproc_nothing_handle(local, access);
		break;
	case MCPS802154_ACCESS_METHOD_IDLE:
		access->error = mcps802154_fproc_idle_handle(local, access);
		break;
	case MCPS802154_ACCESS_METHOD_IMMEDIATE_RX:
		access->error = mcps802154_fproc_rx_handle(local, access);
		break;
	case MCPS802154_ACCESS_METHOD_IMMEDIATE_TX:
		access->error = mcps802154_fproc_tx_handle(local, access);
		break;
	case MCPS802154_ACCESS_METHOD_MULTI:
		access->error = mcps802154_fproc_multi_handle(local, access);
		break;
	case MCPS802154_ACCESS_METHOD_VENDOR:
		access->error = mcps802154_fproc_vendor_handle(local, access);
		break;
	default:
		access->error = -1;
	}

	if (access->error) {
		if (mcps802154_fproc_is_non_recoverable_error(access)) {
			mcps802154_fproc_access_done(local, true);
			mcps802154_fproc_broken_handle(local);
		} else {
			mcps802154_fproc_access_done(local, false);
			mcps802154_fproc_access_now(local);
		}
	}
}

void mcps802154_fproc_access_now(struct mcps802154_local *local)
{
	int r = 0;
	u32 next_timestamp_dtu = 0;

	if (local->start_stop_request)
		r = llhw_get_current_timestamp_dtu(local, &next_timestamp_dtu);

	if (r)
		mcps802154_fproc_broken_handle(local);
	else
		mcps802154_fproc_access(local, next_timestamp_dtu +
						       local->llhw.anticip_dtu);
}

void mcps802154_fproc_access_done(struct mcps802154_local *local, bool error)
{
	struct mcps802154_access *access = local->fproc.access;

	if (access->common_ops->access_done)
		access->common_ops->access_done(access, error);
	local->fproc.access = NULL;
}

void mcps802154_fproc_access_reset(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;

	if (access) {
		if (local->fproc.tx_skb) {
			WARN_ON(access->method ==
				MCPS802154_ACCESS_METHOD_VENDOR);
			access->ops->tx_return(
				access, local->fproc.frame_idx,
				local->fproc.tx_skb,
				MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL);
			local->fproc.tx_skb = NULL;
		}
		mcps802154_fproc_access_done(local, false);
		local->fproc.access = NULL;
	}
}

static void mcps802154_broken_safe(struct mcps802154_local *local)
{
	if (local->fproc.state->broken)
		local->fproc.state->broken(local);
	else
		mcps802154_fproc_broken_handle(local);
}

static void mcps802154_fproc_call_deferred(struct mcps802154_local *local)
{
	struct mcps802154_region *region = local->fproc.deferred;

	if (region) {
		local->fproc.deferred = NULL;
		region->ops->deferred(region);
	}
}

void mcps802154_fproc_schedule_change(struct mcps802154_local *local)
{
	local->fproc.state->schedule_change(local);
	mcps802154_fproc_call_deferred(local);
}

void mcps802154_rx_frame(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	trace_llhw_event_rx_frame(local);
	if (local->fproc.state->rx_frame)
		local->fproc.state->rx_frame(local);
	else
		mcps802154_broken_safe(local);
	mcps802154_fproc_call_deferred(local);
	trace_llhw_event_done(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_rx_frame);

void mcps802154_rx_timeout(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	trace_llhw_event_rx_timeout(local);
	if (local->fproc.state->rx_timeout)
		local->fproc.state->rx_timeout(local);
	else
		mcps802154_broken_safe(local);
	mcps802154_fproc_call_deferred(local);
	trace_llhw_event_done(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_rx_timeout);

void mcps802154_rx_error(struct mcps802154_llhw *llhw,
			 enum mcps802154_rx_error_type error)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	trace_llhw_event_rx_error(local, error);
	if (local->fproc.state->rx_error)
		local->fproc.state->rx_error(local, error);
	else
		mcps802154_broken_safe(local);
	mcps802154_fproc_call_deferred(local);
	trace_llhw_event_done(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_rx_error);

void mcps802154_tx_done(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	trace_llhw_event_tx_done(local);
	if (local->fproc.state->tx_done)
		local->fproc.state->tx_done(local);
	else
		mcps802154_broken_safe(local);
	mcps802154_fproc_call_deferred(local);
	trace_llhw_event_done(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_tx_done);

void mcps802154_tx_too_late(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	if (local->fproc.state->tx_too_late)
		local->fproc.state->tx_too_late(local);
	else
		mcps802154_broken_safe(local);
	trace_llhw_event_done(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_tx_too_late);

void mcps802154_rx_too_late(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	if (local->fproc.state->rx_too_late)
		local->fproc.state->rx_too_late(local);
	else
		mcps802154_broken_safe(local);
	trace_llhw_event_done(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_rx_too_late);

void mcps802154_broken(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	trace_llhw_event_broken(local);
	mcps802154_broken_safe(local);
	mcps802154_fproc_call_deferred(local);
	trace_llhw_event_done(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_broken);

void mcps802154_timer_expired(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	trace_llhw_event_timer_expired(local);
	if (local->fproc.state->timer_expired)
		local->fproc.state->timer_expired(local);
	mcps802154_fproc_call_deferred(local);
	trace_llhw_event_done(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_timer_expired);
