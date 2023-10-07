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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <net/rtnetlink.h>
#include <linux/jiffies.h>

#include "mcps802154_i.h"
#include "llhw-ops.h"

#define DW3000_MAX_STOP_WAIT 10000

static int mcps802154_start(struct ieee802154_hw *hw)
{
	struct mcps802154_local *local = hw->priv;
	int r;

	ASSERT_RTNL();
	WARN_ON(local->started);

	mutex_lock(&local->fsm_lock);
	local->pib.phy_current_channel.page = local->hw->phy->current_page;
	local->pib.phy_current_channel.channel =
		local->hw->phy->current_channel;
	local->pib.phy_current_channel.preamble_code =
		local->llhw.current_preamble_code;
	r = llhw_set_channel(local, local->pib.phy_current_channel.page,
			     local->pib.phy_current_channel.channel,
			     local->pib.phy_current_channel.preamble_code);
	if (!r)
		r = mcps802154_ca_start(local);
	mutex_unlock(&local->fsm_lock);

	return r;
}

static void mcps802154_stop(struct ieee802154_hw *hw)
{
	struct mcps802154_local *local = hw->priv;
	int rc;

	ASSERT_RTNL();
	WARN_ON(!local->started);

	mutex_lock(&local->fsm_lock);
	mcps802154_ca_stop(local);
	mutex_unlock(&local->fsm_lock);

	rc = wait_event_timeout(local->wq, !local->started, msecs_to_jiffies(DW3000_MAX_STOP_WAIT));
	if (!rc)
		pr_err("%s timeout elapsed, event !local->started = false\n", __func__);
}

static int mcps802154_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct mcps802154_local *local = hw->priv;
	int r;

	if (unlikely(!local->started)) {
		r = -EPIPE;
	} else {
		r = mcps802154_ca_xmit_skb(local, skb);
	}

	schedule_work(&local->tx_work);
	return r;
}

static int mcps802154_ed(struct ieee802154_hw *hw, u8 *level)
{
	/* Not supported for the moment (and not used in Linux SoftMAC). */
	return -EOPNOTSUPP;
}

static int mcps802154_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct mcps802154_local *local = hw->priv;
	int r;

	if (!local->ops->set_channel)
		return -EOPNOTSUPP;

	mutex_lock(&local->fsm_lock);
	r = llhw_set_channel(local, page, channel,
			     local->pib.phy_current_channel.preamble_code);
	if (!r) {
		local->pib.phy_current_channel.page = page;
		local->pib.phy_current_channel.channel = channel;
	}
	mutex_unlock(&local->fsm_lock);

	return r;
}

static int mcps802154_set_hw_addr_filt(struct ieee802154_hw *hw,
				       struct ieee802154_hw_addr_filt *filt,
				       unsigned long changed)
{
	struct mcps802154_local *local = hw->priv;
	int r;

	if (!local->ops->set_hw_addr_filt)
		return -EOPNOTSUPP;

	mutex_lock(&local->fsm_lock);
	if (local->started) {
		r = -EBUSY;
	} else {
		r = llhw_set_hw_addr_filt(local, filt, changed);
		if (!r) {
			if (changed & IEEE802154_AFILT_PANID_CHANGED)
				local->pib.mac_pan_id = filt->pan_id;
			if (changed & IEEE802154_AFILT_SADDR_CHANGED)
				local->pib.mac_short_addr = filt->short_addr;
			if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED)
				local->pib.mac_extended_addr = filt->ieee_addr;
			if (changed & IEEE802154_AFILT_PANC_CHANGED)
				local->mac_pan_coord = filt->pan_coord;
		}
	}
	mutex_unlock(&local->fsm_lock);

	return r;
}

static int mcps802154_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	struct mcps802154_local *local = hw->priv;

	if (retries < 0 || retries > 7)
		return -EINVAL;
	local->pib.mac_max_frame_retries = retries;

	return 0;
}

static int mcps802154_set_promiscuous_mode(struct ieee802154_hw *hw, bool on)
{
	struct mcps802154_local *local = hw->priv;
	int r;

	if (!local->ops->set_promiscuous_mode)
		return -EOPNOTSUPP;

	mutex_lock(&local->fsm_lock);
	r = llhw_set_promiscuous_mode(local, on);
	if (!r)
		local->pib.mac_promiscuous = on;
	mutex_unlock(&local->fsm_lock);

	return r;
}

#ifdef CONFIG_HAVE_IEEE802154_SCANNING
static void mcps802154_sw_scan_start(struct ieee802154_hw *hw, __le64 addr)
{
	struct mcps802154_local *local = hw->priv;

	if (!local->ops->set_scanning_mode)
		return;

	mutex_lock(&local->fsm_lock);
	llhw_set_scanning_mode(local, true);
	mutex_unlock(&local->fsm_lock);
}

static void mcps802154_sw_scan_complete(struct ieee802154_hw *hw)
{
	struct mcps802154_local *local = hw->priv;

	if (!local->ops->set_scanning_mode)
		return;

	mutex_lock(&local->fsm_lock);
	llhw_set_scanning_mode(local, false);
	mutex_unlock(&local->fsm_lock);
}
#endif

const struct ieee802154_ops mcps802154_ops = {
	.owner = THIS_MODULE,
	.start = mcps802154_start,
	.stop = mcps802154_stop,
	.xmit_async = mcps802154_xmit_async,
	.ed = mcps802154_ed,
	.set_channel = mcps802154_set_channel,
	.set_hw_addr_filt = mcps802154_set_hw_addr_filt,
	.set_frame_retries = mcps802154_set_frame_retries,
	.set_promiscuous_mode = mcps802154_set_promiscuous_mode,
#ifdef CONFIG_HAVE_IEEE802154_SCANNING
	.sw_scan_start = mcps802154_sw_scan_start,
	.sw_scan_complete = mcps802154_sw_scan_complete,
#endif
};
