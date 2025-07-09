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
#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <net/rtnetlink.h>

#include "mcps802154_i.h"
#include "llhw-ops.h"
#include "default_region.h"
#include "idle_region.h"
#include "endless_scheduler.h"
#include "on_demand_scheduler.h"
#include "nl.h"
#include "warn_return.h"

static LIST_HEAD(registered_llhw);
static DEFINE_MUTEX(registered_llhw_lock);

static void mcps802154_tx_event(struct work_struct *work)
{
	struct mcps802154_local *local = txwork_to_local(work);

	mutex_lock(&local->fsm_lock);
	if (likely(local->started))
		mcps802154_ca_may_reschedule(local);
	mutex_unlock(&local->fsm_lock);
}

struct mcps802154_llhw *mcps802154_alloc_llhw(size_t priv_data_len,
					      const struct mcps802154_ops *ops)
{
	static atomic_t llhw_counter = ATOMIC_INIT(0);
	int idx;
	struct ieee802154_hw *hw;
	struct mcps802154_local *local;
	size_t priv_size;

	if (WARN_ON(!ops || !ops->start || !ops->stop || !ops->tx_frame ||
		    !ops->rx_enable || !ops->rx_disable || !ops->rx_get_frame ||
		    !ops->rx_get_error_frame))
		return NULL;

	priv_size = ALIGN(sizeof(*local), NETDEV_ALIGN) + priv_data_len;
	hw = ieee802154_alloc_hw(priv_size, &mcps802154_ops);
	if (!hw)
		return NULL;

	idx = atomic_inc_return(&llhw_counter);
	if (idx < 0) {
		/* Wrapped! */
		atomic_dec(&llhw_counter);
		ieee802154_free_hw(hw);
		return NULL;
	}

	local = hw->priv;
	local->hw = hw;
	local->llhw.hw = hw;
	local->llhw.priv = (char *)local + ALIGN(sizeof(*local), NETDEV_ALIGN);
	local->ops = ops;
	local->hw_idx = idx - 1;
	init_waitqueue_head(&local->wq);
	mutex_init(&local->fsm_lock);
	INIT_WORK(&local->tx_work, mcps802154_tx_event);
	mutex_lock(&local->fsm_lock);
	mcps802154_ca_init(local);
	mcps802154_fproc_init(local);
	mutex_unlock(&local->fsm_lock);

	return &local->llhw;
}
EXPORT_SYMBOL(mcps802154_alloc_llhw);

void mcps802154_free_llhw(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&local->fsm_lock);
	mcps802154_fproc_uninit(local);
	mcps802154_ca_uninit(local);
	mutex_unlock(&local->fsm_lock);
	mutex_destroy(&local->fsm_lock);

	WARN_ON(waitqueue_active(&local->wq));
#ifndef __KERNEL__
	destroy_waitqueue_head(&local->wq);
#endif

	ieee802154_free_hw(local->hw);
}
EXPORT_SYMBOL(mcps802154_free_llhw);

int mcps802154_register_llhw(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);
	int r;

	llhw->hw->flags |= IEEE802154_HW_FRAME_RETRIES;

	r = ieee802154_register_hw(local->hw);
	if (r)
		return r;

	local->pib.mac_extended_addr = local->hw->phy->perm_extended_addr;
	local->pib.mac_pan_id = IEEE802154_PAN_ID_BROADCAST;
	local->pib.mac_promiscuous = false;
	local->pib.mac_short_addr = IEEE802154_ADDR_SHORT_BROADCAST;
	local->pib.phy_current_channel.page = local->hw->phy->current_page;
	local->pib.phy_current_channel.channel =
		local->hw->phy->current_channel;
	local->pib.phy_current_channel.preamble_code =
		llhw->current_preamble_code;
	local->mac_pan_coord = false;

	mutex_lock(&registered_llhw_lock);
	list_add(&local->registered_entry, &registered_llhw);
	mutex_unlock(&registered_llhw_lock);

	return 0;
}
EXPORT_SYMBOL(mcps802154_register_llhw);

void mcps802154_unregister_llhw(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	mutex_lock(&registered_llhw_lock);
	list_del(&local->registered_entry);
	mutex_unlock(&registered_llhw_lock);
	ieee802154_unregister_hw(local->hw);
	mutex_lock(&local->fsm_lock);
	mcps802154_ca_close(local);
	mutex_unlock(&local->fsm_lock);
}
EXPORT_SYMBOL(mcps802154_unregister_llhw);

__le64 mcps802154_get_extended_addr(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return local->pib.mac_extended_addr;
}
EXPORT_SYMBOL(mcps802154_get_extended_addr);

__le16 mcps802154_get_pan_id(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return local->pib.mac_pan_id;
}
EXPORT_SYMBOL(mcps802154_get_pan_id);

__le16 mcps802154_get_short_addr(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return local->pib.mac_short_addr;
}
EXPORT_SYMBOL(mcps802154_get_short_addr);

const struct mcps802154_channel *
mcps802154_get_current_channel(struct mcps802154_llhw *llhw)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return &local->pib.phy_current_channel;
}
EXPORT_SYMBOL(mcps802154_get_current_channel);

int mcps802154_get_current_timestamp_dtu(struct mcps802154_llhw *llhw,
					 u32 *timestamp_dtu)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	if (!local->started)
		return -ENETDOWN;

	return llhw_get_current_timestamp_dtu(local, timestamp_dtu);
}
EXPORT_SYMBOL(mcps802154_get_current_timestamp_dtu);

u64 mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
	struct mcps802154_llhw *llhw, u32 tx_timestamp_dtu,
	const struct mcps802154_hrp_uwb_params *hrp_uwb_params,
	const struct mcps802154_channel *channel_params, int ant_set_id)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return llhw_tx_timestamp_dtu_to_rmarker_rctu(local, tx_timestamp_dtu,
						     hrp_uwb_params,
						     channel_params,
						     ant_set_id);
}
EXPORT_SYMBOL(mcps802154_tx_timestamp_dtu_to_rmarker_rctu);

s64 mcps802154_difference_timestamp_rctu(struct mcps802154_llhw *llhw,
					 u64 timestamp_a_rctu,
					 u64 timestamp_b_rctu)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return llhw_difference_timestamp_rctu(local, timestamp_a_rctu,
					      timestamp_b_rctu);
}
EXPORT_SYMBOL(mcps802154_difference_timestamp_rctu);

int mcps802154_rx_get_measurement(struct mcps802154_llhw *llhw, void *rx_ctx,
				  struct mcps802154_rx_measurement_info *info)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return llhw_rx_get_measurement(local, rx_ctx, info);
}
EXPORT_SYMBOL(mcps802154_rx_get_measurement);

int mcps802154_compute_frame_duration_dtu(struct mcps802154_llhw *llhw,
					  int payload_bytes)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return llhw_compute_frame_duration_dtu(local, payload_bytes);
}
EXPORT_SYMBOL(mcps802154_compute_frame_duration_dtu);

int mcps802154_vendor_cmd(struct mcps802154_llhw *llhw, u32 vendor_id,
			  u32 subcmd, void *data, size_t data_len)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return llhw_vendor_cmd(local, vendor_id, subcmd, data, data_len);
}
EXPORT_SYMBOL(mcps802154_vendor_cmd);

int mcps802154_check_hrp_uwb_params(
	struct mcps802154_llhw *llhw,
	const struct mcps802154_hrp_uwb_params *hrp_uwb_params)
{
	struct mcps802154_local *local = llhw_to_local(llhw);

	return llhw_check_hrp_uwb_params(local, hrp_uwb_params);
}
EXPORT_SYMBOL(mcps802154_check_hrp_uwb_params);

struct mcps802154_local *mcps802154_get_first_by_idx(int hw_idx)
{
	struct mcps802154_local *result = NULL, *local;

	ASSERT_RTNL();

	mutex_lock(&registered_llhw_lock);
	list_for_each_entry (local, &registered_llhw, registered_entry) {
		if (local->hw_idx >= hw_idx) {
			result = local;
			break;
		}
	}
	mutex_unlock(&registered_llhw_lock);

	return result;
}

int __init mcps802154_init(void)
{
	int r;

	r = mcps802154_nl_init();
	if (r)
		return r;
	r = mcps802154_default_region_init();
	WARN_RETURN(r);
	r = mcps802154_idle_region_init();
	WARN_RETURN(r);
	r = mcps802154_endless_scheduler_init();
	WARN_ON(r);
	r = mcps802154_default_scheduler_init();
	WARN_ON(r);
	r = mcps802154_on_demand_scheduler_init();
	WARN_ON(r);

	return r;
}

void __exit mcps802154_exit(void)
{
	mcps802154_on_demand_scheduler_exit();
	mcps802154_default_scheduler_exit();
	mcps802154_endless_scheduler_exit();
	mcps802154_idle_region_exit();
	mcps802154_default_region_exit();
	mcps802154_nl_exit();
}

module_init(mcps802154_init);
module_exit(mcps802154_exit);

MODULE_DESCRIPTION("IEEE 802.15.4 MAC common part sublayer");
MODULE_AUTHOR("Nicolas Schodet <nicolas.schodet@qorvo.com>");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
