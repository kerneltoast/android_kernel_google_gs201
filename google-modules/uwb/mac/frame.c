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

#include <linux/ieee802154.h>
#include <linux/module.h>
#include <net/mcps802154_frame.h>
#include <net/mcps802154.h>

#include "mcps802154_i.h"

struct sk_buff *mcps802154_frame_alloc(struct mcps802154_llhw *llhw,
				       unsigned int size, gfp_t flags)
{
	struct sk_buff *skb;
	size_t hlen;
	size_t tlen;

	hlen = llhw->hw->extra_tx_headroom;
	tlen = IEEE802154_FCS_LEN;

	skb = alloc_skb(hlen + size + tlen, flags);
	if (!skb)
		return NULL;

	skb_reserve(skb, hlen);
	skb_tailroom_reserve(skb, size, tlen);

	return skb;
}
EXPORT_SYMBOL(mcps802154_frame_alloc);
