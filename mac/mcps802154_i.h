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

#ifndef NET_MCPS802154_MCPS802154_I_H
#define NET_MCPS802154_MCPS802154_I_H

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/mac802154.h>
#include <net/mcps802154.h>

#include "ca.h"
#include "fproc.h"

/**
 * struct mcps802154_pib - PIB (PAN Information Base): this is a database of
 * 802.15.4 settings.
 */
struct mcps802154_pib {
	/**
	 * @mac_extended_addr: Current extended address.
	 */
	__le64 mac_extended_addr;
	/**
	 * @mac_pan_id: The identifier of the PAN on which the device is
	 * operating. 0xffff if the device is not associated.
	 */
	__le16 mac_pan_id;
	/**
	 * @mac_short_addr: The address the device uses to communicate inside
	 * its PAN. 0xffff if the device is not associated, 0xfffe if the device
	 * is associated but has no short address.
	 */
	__le16 mac_short_addr;
	/**
	 * @mac_promiscuous: Indicate whether the promiscuous mode is enabled.
	 */
	bool mac_promiscuous;
	/**
	 * @mac_max_frame_retries: Number of retries on TX.
	 */
	s8 mac_max_frame_retries;
	/**
	 * @phy_current_channel: Current channel parameters.
	 */
	struct mcps802154_channel phy_current_channel;
};

/**
 * struct mcps802154_local - MCPS private data.
 */
struct mcps802154_local {
	/**
	 * @llhw: Low-level hardware.
	 */
	struct mcps802154_llhw llhw;
	/**
	 * @hw: Pointer to MCPS hw instance.
	 */
	struct ieee802154_hw *hw;
	/**
	 * @ops: Low-level driver operations.
	 */
	const struct mcps802154_ops *ops;
	/**
	 * @hw_idx: Index of hardware.
	 */
	int hw_idx;
	/**
	 * @cur_cmd_info: Current netlink command.
	 */
	struct genl_info *cur_cmd_info;
	/**
	 * @registered_entry: Entry in list of registered low-level driver.
	 */
	struct list_head registered_entry;
	/**
	 * @wq: Wake queue for synchronous operation with an asynchronous
	 * implementation.
	 */
	wait_queue_head_t wq;
	/**
	 * @ca: Channel access context.
	 */
	struct mcps802154_ca ca;
	/**
	 * @fproc: Frame processing context.
	 */
	struct mcps802154_fproc fproc;
	/**
	 * @fsm_lock: FSM lock to avoid multiple access.
	 */
	struct mutex fsm_lock;
	/**
	 * @tx_work: Transmit work to schedule async actions.
	 */
	struct work_struct tx_work;
	/**
	 * @start_stop_request: Request to start (true) or stop (false) from
	 * mac802154 layer.
	 */
	bool start_stop_request;
	/**
	 * @started: Current started state.
	 */
	bool started;
	/**
	 * @broken: Currently broken.
	 */
	bool broken;
	/**
	 * @pib: PAN Information Base.
	 */
	struct mcps802154_pib pib;
	/**
	 * @mac_pan_coord: Indicate whether the hardware filtering should operate as
	 * coordinator.
	 */
	bool mac_pan_coord;
};

static inline struct mcps802154_local *
llhw_to_local(struct mcps802154_llhw *llhw)
{
	return container_of(llhw, struct mcps802154_local, llhw);
}

static inline struct mcps802154_local *
txwork_to_local(struct work_struct *tx_work)
{
	return container_of(tx_work, struct mcps802154_local, tx_work);
}

struct mcps802154_local *mcps802154_get_first_by_idx(int hw_idx);

extern const struct ieee802154_ops mcps802154_ops;

#endif /* NET_MCPS802154_MCPS802154_I_H */
