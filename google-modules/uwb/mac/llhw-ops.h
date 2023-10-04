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

#ifndef LLHW_OPS_H
#define LLHW_OPS_H

#include <linux/errno.h>

#include "mcps802154_i.h"
#include "trace.h"

static inline int llhw_start(struct mcps802154_local *local)
{
	int r;

	trace_llhw_start(local);
	r = local->ops->start(&local->llhw);
	trace_llhw_return_int(local, r);
	return r;
}

static inline void llhw_stop(struct mcps802154_local *local)
{
	trace_llhw_stop(local);
	local->ops->stop(&local->llhw);
	trace_llhw_return_void(local);
}

static inline int llhw_tx_frame(struct mcps802154_local *local,
				struct sk_buff *skb,
				const struct mcps802154_tx_frame_config *config,
				int frame_idx, int next_delay_dtu)
{
	int r;

	trace_llhw_tx_frame(local, config, frame_idx, next_delay_dtu);
	r = local->ops->tx_frame(&local->llhw, skb, config, frame_idx,
				 next_delay_dtu);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_rx_enable(struct mcps802154_local *local,
				 const struct mcps802154_rx_frame_config *info,
				 int frame_idx, int next_delay_dtu)
{
	int r;

	trace_llhw_rx_enable(local, info, frame_idx, next_delay_dtu);
	r = local->ops->rx_enable(&local->llhw, info, frame_idx,
				  next_delay_dtu);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_rx_disable(struct mcps802154_local *local)
{
	int r;

	trace_llhw_rx_disable(local);
	r = local->ops->rx_disable(&local->llhw);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_rx_get_frame(struct mcps802154_local *local,
				    struct sk_buff **skb,
				    struct mcps802154_rx_frame_info *info)
{
	int r;

	trace_llhw_rx_get_frame(local, info);
	r = local->ops->rx_get_frame(&local->llhw, skb, info);
	trace_llhw_return_rx_frame(local, r, info);
	return r;
}

static inline int llhw_rx_get_error_frame(struct mcps802154_local *local,
					  struct mcps802154_rx_frame_info *info)
{
	int r;

	trace_llhw_rx_get_error_frame(local, info);
	r = local->ops->rx_get_error_frame(&local->llhw, info);
	trace_llhw_return_rx_frame(local, r, info);
	return r;
}

static inline int llhw_idle(struct mcps802154_local *local, bool timestamp,
			    u32 timestamp_dtu)
{
	int r;

	if (timestamp)
		trace_llhw_idle_timestamp(local, timestamp_dtu);
	else
		trace_llhw_idle(local);
	r = local->ops->idle(&local->llhw, timestamp, timestamp_dtu);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_reset(struct mcps802154_local *local)
{
	int r;

	trace_llhw_reset(local);
	r = local->ops->reset(&local->llhw);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_get_current_timestamp_dtu(struct mcps802154_local *local,
						 u32 *timestamp_dtu)
{
	int r;

	trace_llhw_get_current_timestamp_dtu(local);
	r = local->ops->get_current_timestamp_dtu(&local->llhw, timestamp_dtu);
	trace_llhw_return_timestamp_dtu(local, r, *timestamp_dtu);
	return r;
}

static inline u64 llhw_tx_timestamp_dtu_to_rmarker_rctu(
	struct mcps802154_local *local, u32 tx_timestamp_dtu,
	const struct mcps802154_hrp_uwb_params *hrp_uwb_params,
	const struct mcps802154_channel *channel_params, int ant_set_id)
{
	return local->ops->tx_timestamp_dtu_to_rmarker_rctu(
		&local->llhw, tx_timestamp_dtu, hrp_uwb_params, channel_params,
		ant_set_id);
}

static inline s64 llhw_difference_timestamp_rctu(struct mcps802154_local *local,
						 u64 timestamp_a_rctu,
						 u64 timestamp_b_rctu)
{
	return local->ops->difference_timestamp_rctu(
		&local->llhw, timestamp_a_rctu, timestamp_b_rctu);
}

static inline int
llhw_compute_frame_duration_dtu(struct mcps802154_local *local,
				int payload_bytes)
{
	return local->ops->compute_frame_duration_dtu(&local->llhw,
						      payload_bytes);
}

static inline int llhw_set_channel(struct mcps802154_local *local, u8 page,
				   u8 channel, u8 preamble_code)
{
	int r;

	trace_llhw_set_channel(local, page, channel, preamble_code);
	r = local->ops->set_channel(&local->llhw, page, channel, preamble_code);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int __nocfi
llhw_set_hrp_uwb_params(struct mcps802154_local *local,
			const struct mcps802154_hrp_uwb_params *params)
{
	int r;

	trace_llhw_set_hrp_uwb_params(local, params);
	r = local->ops->set_hrp_uwb_params(&local->llhw, params);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int
llhw_set_sts_params(struct mcps802154_local *local,
		    const struct mcps802154_sts_params *params)
{
	int r;

	trace_llhw_set_sts_params(local, params);
	if (local->ops->set_sts_params)
		r = local->ops->set_sts_params(&local->llhw, params);
	else
		r = -EOPNOTSUPP;
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_set_hw_addr_filt(struct mcps802154_local *local,
					struct ieee802154_hw_addr_filt *filt,
					unsigned long changed)
{
	int r;

	trace_llhw_set_hw_addr_filt(local, filt, changed);
	r = local->ops->set_hw_addr_filt(&local->llhw, filt, changed);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_set_txpower(struct mcps802154_local *local, s32 mbm)
{
	int r;

	trace_llhw_set_txpower(local, mbm);
	r = local->ops->set_txpower(&local->llhw, mbm);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_set_cca_ed_level(struct mcps802154_local *local, s32 mbm)
{
	int r;

	trace_llhw_set_cca_ed_level(local, mbm);
	r = local->ops->set_cca_ed_level(&local->llhw, mbm);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_set_promiscuous_mode(struct mcps802154_local *local,
					    bool on)
{
	int r;

	trace_llhw_set_promiscuous_mode(local, on);
	r = local->ops->set_promiscuous_mode(&local->llhw, on);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_set_scanning_mode(struct mcps802154_local *local,
					 bool on)
{
	int r;

	trace_llhw_set_scanning_mode(local, on);
	r = local->ops->set_scanning_mode(&local->llhw, on);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_set_calibration(struct mcps802154_local *local,
				       const char *key, void *value,
				       size_t length)
{
	int r;

	trace_llhw_set_calibration(local, key);
	r = local->ops->set_calibration(&local->llhw, key, value, length);
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_get_calibration(struct mcps802154_local *local,
				       const char *key, void *value,
				       size_t length)
{
	int r;

	trace_llhw_get_calibration(local, key);
	r = local->ops->get_calibration(&local->llhw, key, value, length);
	trace_llhw_return_int(local, r);
	return r;
}

static inline const char *const *
llhw_list_calibration(struct mcps802154_local *local)
{
	const char *const *r;

	trace_llhw_list_calibration(local);
	if (local->ops->list_calibration) {
		r = local->ops->list_calibration(&local->llhw);
	} else {
		r = NULL;
	}
	trace_llhw_return_void(local);
	return r;
}

static inline int llhw_vendor_cmd(struct mcps802154_local *local, u32 vendor_id,
				  u32 subcmd, void *data, size_t data_len)
{
	int r;

	trace_llhw_vendor_cmd(local, vendor_id, subcmd, data_len);
	if (local->ops->vendor_cmd)
		r = local->ops->vendor_cmd(&local->llhw, vendor_id, subcmd,
					   data, data_len);
	else
		r = -EOPNOTSUPP;
	trace_llhw_return_int(local, r);
	return r;
}

static inline int llhw_check_hrp_uwb_params(
	struct mcps802154_local *local,
	const struct mcps802154_hrp_uwb_params *hrp_uwb_params)
{
	int r;

	trace_llhw_check_hrp_uwb_params(local, hrp_uwb_params);
	if (local->ops->check_hrp_uwb_params)
		r = local->ops->check_hrp_uwb_params(&local->llhw,
						     hrp_uwb_params);
	else
		r = -EOPNOTSUPP;
	trace_llhw_return_int(local, r);
	return r;
}

static inline int
llhw_rx_get_measurement(struct mcps802154_local *local, void *rx_ctx,
			struct mcps802154_rx_measurement_info *info)
{
	int r;
	trace_llhw_rx_get_measurement(local, rx_ctx);
	if (local->ops->rx_get_measurement)
		r = local->ops->rx_get_measurement(&local->llhw, rx_ctx, info);
	else
		r = -EOPNOTSUPP;
	trace_llhw_return_measurement(local, r, info);
	return r;
}

#ifdef CONFIG_MCPS802154_TESTMODE
static inline int llhw_testmode_cmd(struct mcps802154_local *local, void *data,
				    int len)
{
	int r;

	trace_llhw_testmode_cmd(local);
	r = local->ops->testmode_cmd(&local->llhw, data, len);
	trace_llhw_return_int(local, r);
	return r;
}
#endif /* CONFIG_MCPS802154_TESTMODE */

#endif /* LLHW_OPS_H */
