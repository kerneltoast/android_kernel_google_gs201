/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021-2022 Qorvo US, Inc.
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

#include <linux/math64.h>
#include "pctt_access.h"
#include "pctt_region.h"
#include "pctt_region_call.h"
#include "llhw-ops.h"
#include "mcps802154_qorvo.h"

#include <net/mcps802154_frame.h>
#include <net/pctt_region_nl.h>
#include <net/pctt_region_params.h>
#include <asm/unaligned.h>

#include "warn_return.h"

#define PCTT_STS_FOM_THRESHOLD 153
/* The FC-PHY shall have a block timing tolerance of +/-100 ppm as
   specified in IEEE Std 802.15.4z-2020, subclause 6.9.7.2. */
#define PCTT_MARGIN_PPM 200

static inline int pctt_rx_margin(int duration)
{
	return duration / (1000000 / PCTT_MARGIN_PPM);
}

static void
pctt_set_sts_params(struct mcps802154_sts_params *sts_params,
		    const struct pctt_session_params *session_params)
{
	const u8 key[AES_KEYSIZE_128] = { 0x14, 0x14, 0x86, 0x74, 0xd1, 0xd3,
					  0x36, 0xaa, 0xf8, 0x60, 0x50, 0xa8,
					  0x14, 0xeb, 0x22, 0xf };
	u8 *iv = sts_params->v;
	u8 seg_len = session_params->sts_length == PCTT_STS_LENGTH_128 ?
			     128 :
			     session_params->sts_length == PCTT_STS_LENGTH_32 ?
			     32 :
			     64;

	sts_params->n_segs = session_params->number_of_sts_segments;
	sts_params->seg_len = seg_len;
	sts_params->sp2_tx_gap_4chips = 0;
	sts_params->sp2_rx_gap_4chips[0] = 0;
	sts_params->sp2_rx_gap_4chips[1] = 0;
	sts_params->sp2_rx_gap_4chips[2] = 0;
	sts_params->sp2_rx_gap_4chips[3] = 0;

	/* Overflow is not propagated to the next IV */
	put_unaligned_be32(0x362eeb34u, &iv[0]);
	put_unaligned_be32(0xc44fa8fbu + session_params->sts_index,
			   &iv[sizeof(u32)]);
	put_unaligned_be64(0xd37ec3ca1f9a3de4ull, &iv[sizeof(u64)]);
	memcpy(sts_params->key, key, AES_KEYSIZE_128);
}

static void pctt_randomize_psdu(struct pctt_local *local)
{
	struct pctt_session *session = &local->session;
	struct pctt_session_params *p = &session->params;

	if (p->randomize_psdu && session->first_access) {
		const int A = 1664525, B = 1013904223;
		/* First byte of data is used as seed. */
		u32 state = p->data_payload[0];
		u8 *buf = p->data_payload;
		int size = p->data_payload_len;
		int i;
		for (i = 0; i < size; i++) {
			state = A * state + B;
			buf[i] = state >> 8;
		}
	}
}

/**
 * pctt_access_setup_frame() - Fill an access frame from a PCTT slot.
 * @local: PCTT context.
 * @slot: Corresponding slot.
 * @frame_dtu: Frame transmission or reception date.
 * @frame: Access frame.
 * @sts_params: Where to store STS parameters.
 */
static void pctt_access_setup_frame(struct pctt_local *local,
				    const struct pctt_slot *slot,
				    const u32 frame_dtu,
				    struct mcps802154_access_frame *frame,
				    struct mcps802154_sts_params *sts_params)
{
	struct mcps802154_sts_params *sts_params_for_access = NULL;
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	bool is_rframe = p->rframe_config != PCTT_RFRAME_CONFIG_SP0;

	if (is_rframe) {
		pctt_set_sts_params(sts_params, p);
		sts_params_for_access = sts_params;
	}

	if (slot->is_tx) {
		u8 flags = slot->is_immediate ?
				   0 :
				   MCPS802154_TX_FRAME_CONFIG_TIMESTAMP_DTU;

		if (is_rframe) {
			if (p->rframe_config == PCTT_RFRAME_CONFIG_SP3)
				flags |= MCPS802154_TX_FRAME_CONFIG_SP3;
			else if (p->rframe_config == PCTT_RFRAME_CONFIG_SP2)
				flags |= MCPS802154_TX_FRAME_CONFIG_SP2;
			else
				flags |= MCPS802154_TX_FRAME_CONFIG_SP1;
		}
		*frame = (struct mcps802154_access_frame){
			.is_tx = true,
			.tx_frame_config = {
				.timestamp_dtu = frame_dtu,
				.flags = flags,
				.ant_set_id = p->tx_antenna_selection,
			},
			.sts_params = sts_params_for_access,
		};
	} else {
		u8 flags = slot->is_immediate ?
				   0 :
				   MCPS802154_RX_FRAME_CONFIG_TIMESTAMP_DTU;
		u16 request = MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
			      MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU |
			      MCPS802154_RX_FRAME_INFO_RSSI;

		if (is_rframe) {
			request |= MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM;
			flags |= MCPS802154_RX_FRAME_CONFIG_RANGING;
			if (session->cmd_id == PCTT_ID_ATTR_SS_TWR) {
				flags |=
					MCPS802154_RX_FRAME_CONFIG_RANGING_PDOA;
				request |=
					MCPS802154_RX_FRAME_INFO_RANGING_PDOA;
			}
			if (p->rframe_config == PCTT_RFRAME_CONFIG_SP3)
				flags |= MCPS802154_RX_FRAME_CONFIG_SP3;
			else if (p->rframe_config == PCTT_RFRAME_CONFIG_SP2)
				flags |= MCPS802154_RX_FRAME_CONFIG_SP2;
			else
				flags |= MCPS802154_RX_FRAME_CONFIG_SP1;
		}
		*frame = (struct mcps802154_access_frame){
			.is_tx = false,
			.rx = {
				.frame_config = {
					.timestamp_dtu = frame_dtu,
					.flags = flags,
					.timeout_dtu = slot->timeout_dtu,
					.ant_set_id = p->rx_antenna_selection,
				},
				.frame_info_flags_request = request,
			},
			.sts_params = sts_params_for_access,
		};
	}
}

static struct sk_buff *pctt_tx_get_frame(struct mcps802154_access *access,
					 int frame_idx)
{
	struct pctt_local *local = access_to_local(access);
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	struct sk_buff *skb = NULL;

	if (p->data_payload_len) {
		skb = mcps802154_frame_alloc(local->llhw, p->data_payload_len,
					     GFP_KERNEL);
		if (skb)
			skb_put_data(skb, p->data_payload, p->data_payload_len);
	}

	return skb;
}

static void pctt_tx_return(struct mcps802154_access *access, int frame_idx,
			   struct sk_buff *skb,
			   enum mcps802154_access_tx_return_reason reason)
{
	struct pctt_local *local = access_to_local(access);

	kfree_skb(skb);

	/* Error on TX. */
	if (reason == MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL)
		local->results.status = PCTT_STATUS_RANGING_TX_FAILED;
	else
		local->frames_remaining_nb--;
}

static bool pctt_rx_sts_good(const struct mcps802154_rx_frame_info *i)
{
	int idx;
	if (!(i->flags & MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM))
		return false;
	for (idx = 0; idx < MCPS802154_STS_N_SEGS_MAX; idx++) {
		if (i->ranging_sts_fom[idx] < PCTT_STS_FOM_THRESHOLD)
			return false;
	}
	return true;
}

static void pctt_rx_frame_ss_twr(struct pctt_local *local,
				 const struct mcps802154_rx_frame_info *info)
{
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	struct pctt_test_ss_twr_results *ss_twr = &local->results.tests.ss_twr;
	bool is_responser = p->device_role == PCTT_DEVICE_ROLE_RESPONDER;

	if (info) {
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU)) {
			local->results.status =
				PCTT_STATUS_RANGING_RX_PHY_TOA_FAILED;
			return;
		}
		if (!pctt_rx_sts_good(info)) {
			local->results.status =
				PCTT_STATUS_RANGING_RX_PHY_STS_FAILED;
			return;
		}
		if (info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA) {
			struct mcps802154_rx_measurement_info info = {};
			int r;

			info.flags |= MCPS802154_RX_MEASUREMENTS_AOAS;
			r = mcps802154_rx_get_measurement(local->llhw, NULL,
							  &info);
			if (!r &&
			    info.flags & MCPS802154_RX_MEASUREMENTS_AOAS &&
			    info.n_aoas) {
				/* TODO: Find which aoas index to use */
				ss_twr->pdoa_azimuth_deg_q7 =
					map_rad_q11_to_deg_q7(
						info.aoas[0].pdoa_rad_q11);
				ss_twr->aoa_azimuth_deg_q7 =
					map_rad_q11_to_deg_q7(
						info.aoas[0].aoa_rad_q11);
			}
		}

		if (info->flags & MCPS802154_RX_FRAME_INFO_RSSI) {
			ss_twr->rssi = info->rssi;
		}

		ss_twr->rx_timestamps_rctu = info->timestamp_rctu;

		/* Resync timestamps. */
		if (is_responser) {
			struct mcps802154_access *access = &local->access;
			struct mcps802154_access_frame *frame =
				&local->frames[1];
			struct mcps802154_sts_params *sts_params =
				&local->sts_params[1];
			struct pctt_slot *s = &local->slots[1];
			u32 frame_dtu;

			if (!(info->flags &
			      MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU)) {
				local->results.status =
					PCTT_STATUS_RANGING_RX_PHY_DEC_FAILED;
				return;
			}

			frame_dtu = info->timestamp_dtu + p->slot_duration_dtu;

			ss_twr->tx_timestamps_rctu =
				mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
					local->llhw, frame_dtu,
					access->hrp_uwb_params, access->channel,
					p->tx_antenna_selection);

			pctt_access_setup_frame(local, s, frame_dtu, frame,
						sts_params);

			frame_dtu += p->slot_duration_dtu;

			access->timestamp_dtu = info->timestamp_dtu;
			access->duration_dtu = frame_dtu - info->timestamp_dtu;
			access->n_frames = 2;
		}

		/* Tround time of Initiator or Treply time of Responder. */
		ss_twr->measurement_rctu = mcps802154_difference_timestamp_rctu(
			local->llhw,
			is_responser ? ss_twr->tx_timestamps_rctu :
				       ss_twr->rx_timestamps_rctu,
			is_responser ? ss_twr->rx_timestamps_rctu :
				       ss_twr->tx_timestamps_rctu);
	}
}

static void pctt_rx_frame_per_rx(struct pctt_local *local, struct sk_buff *skb,
				 const struct mcps802154_rx_frame_info *info,
				 enum mcps802154_rx_error_type error)
{
	struct pctt_test_per_rx_results *per_rx = &local->results.tests.per_rx;

	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	bool has_sts = p->rframe_config != PCTT_RFRAME_CONFIG_SP0;

	if (info) {
		if (info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU) {
			session->next_timestamp_dtu = info->timestamp_dtu;
			session->first_rx_synchronized = true;
		}
		if (info->flags & MCPS802154_RX_FRAME_INFO_RSSI) {
			if (!per_rx->rssi || per_rx->rssi > info->rssi)
				per_rx->rssi = info->rssi;
		}
	}
	session->next_timestamp_dtu +=
		p->gap_duration_dtu - pctt_rx_margin(p->gap_duration_dtu);

	switch (error) {
	case MCPS802154_RX_ERROR_NONE:
	case MCPS802154_RX_ERROR_BAD_CKSUM:
		per_rx->acq_detect++;
		per_rx->sync_cir_ready++;
		per_rx->sfd_found++;
		per_rx->eof++;
		if (has_sts && pctt_rx_sts_good(info))
			per_rx->sts_found++;
		if (skb && (skb->len != p->data_payload_len ||
			    (!p->randomize_psdu &&
			     memcmp(skb->data, p->data_payload, skb->len))))
			per_rx->psdu_bit_error++;
		if (error == MCPS802154_RX_ERROR_BAD_CKSUM)
			per_rx->psdu_dec_error++;
		break;
	case MCPS802154_RX_ERROR_SFD_TIMEOUT:
		per_rx->acq_detect++;
		per_rx->sfd_fail++;
		break;
	case MCPS802154_RX_ERROR_UNCORRECTABLE:
	case MCPS802154_RX_ERROR_FILTERED:
	case MCPS802154_RX_ERROR_HPDWARN:
	case MCPS802154_RX_ERROR_OTHER:
	case MCPS802154_RX_ERROR_PHR_DECODE:
		per_rx->acq_detect++;
		per_rx->sync_cir_ready++;
		per_rx->sfd_found++;
		if (error == MCPS802154_RX_ERROR_OTHER) {
			per_rx->psdu_dec_error++;
		} else if (error == MCPS802154_RX_ERROR_PHR_DECODE) {
			per_rx->phr_dec_error++;
		}
		break;
	case MCPS802154_RX_ERROR_TIMEOUT:
		per_rx->rx_fail++;
		break;
	}
}

static void pctt_rx_frame_rx(struct pctt_local *local, struct sk_buff *skb,
			     const struct mcps802154_rx_frame_info *info)
{
	struct pctt_test_rx_results *rx = &local->results.tests.rx;

	if (skb) {
		int len = min(skb->len, (unsigned int)PCTT_PAYLOAD_MAX_LEN);

		rx->psdu_data_len = len;
		memcpy(rx->psdu_data, skb->data, len);
	}

	if (info) {
		if (info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU) {
			/* 8ns unit for both stats. */
			rx->rx_done_ts_int = (info->timestamp_rctu >> 32) &
					     0xfffffffe;
			rx->rx_done_ts_frac = info->timestamp_rctu & 0xffff;
		} else {
			local->results.status =
				PCTT_STATUS_RANGING_RX_PHY_TOA_FAILED;
		}
		if (info->flags & MCPS802154_RX_FRAME_INFO_RSSI)
			rx->rssi = info->rssi;
	} else
		local->results.status = PCTT_STATUS_RANGING_RX_TIMEOUT;
}

static void pctt_rx_frame(struct mcps802154_access *access, int frame_idx,
			  struct sk_buff *skb,
			  const struct mcps802154_rx_frame_info *info,
			  enum mcps802154_rx_error_type error)
{
	struct pctt_local *local = access_to_local(access);
	struct pctt_session *session = &local->session;
	struct llhw_vendor_cmd_pctt_get_frame_info frame_info = {};

	local->frames_remaining_nb--;

	if (error == MCPS802154_RX_ERROR_BAD_CKSUM) {
		struct mcps802154_access_frame *frame =
			&access->frames[frame_idx];
		int r;

		frame_info.info.flags = frame->rx.frame_info_flags_request;
		r = mcps802154_vendor_cmd(local->llhw, VENDOR_QORVO_OUI,
					  LLHW_VENDOR_CMD_PCTT_GET_FRAME_INFO,
					  &frame_info, sizeof(frame_info));
		if (!r) {
			skb = frame_info.skb;
			info = &frame_info.info;
		}
	}

	if (session->cmd_id == PCTT_ID_ATTR_SS_TWR)
		pctt_rx_frame_ss_twr(local, info);
	else if (session->cmd_id == PCTT_ID_ATTR_PER_RX)
		pctt_rx_frame_per_rx(local, skb, info, error);
	else
		pctt_rx_frame_rx(local, skb, info);

	switch (error) {
	case MCPS802154_RX_ERROR_NONE:
		break;
	case MCPS802154_RX_ERROR_SFD_TIMEOUT:
	case MCPS802154_RX_ERROR_TIMEOUT:
		local->results.status = PCTT_STATUS_RANGING_RX_TIMEOUT;
		break;
	case MCPS802154_RX_ERROR_FILTERED:
	case MCPS802154_RX_ERROR_BAD_CKSUM:
		local->results.status = PCTT_STATUS_RANGING_RX_MAC_DEC_FAILED;
		break;
	case MCPS802154_RX_ERROR_UNCORRECTABLE:
	case MCPS802154_RX_ERROR_HPDWARN:
	case MCPS802154_RX_ERROR_OTHER:
	case MCPS802154_RX_ERROR_PHR_DECODE:
		local->results.status = PCTT_STATUS_RANGING_RX_PHY_DEC_FAILED;
		break;
	}
	if (skb)
		kfree_skb(skb);
}

static void pctt_access_done(struct mcps802154_access *access, bool error)
{
	struct pctt_local *local = access_to_local(access);
	struct pctt_session *session = &local->session;
	bool end_of_test = false;

	if (session->cmd_id == PCTT_ID_ATTR_PER_RX)
		local->results.tests.per_rx.attempts++;

	if (error && !local->results.status)
		local->results.status = PCTT_STATUS_RANGING_INTERNAL_ERROR;

	switch (session->cmd_id) {
	case PCTT_ID_ATTR_LOOPBACK:
	case PCTT_ID_ATTR_SS_TWR:
	case PCTT_ID_ATTR_RX:
		end_of_test = true;
		break;
	default:
		/* Only stop rx tests when all packets are received. */
		if (local->results.status != PCTT_STATUS_RANGING_SUCCESS &&
		    session->cmd_id != PCTT_ID_ATTR_PER_RX)
			end_of_test = true;
		if (session->stop_request || !local->frames_remaining_nb)
			end_of_test = true;
		break;
	}

	if (end_of_test) {
		int r;

		r = mcps802154_vendor_cmd(local->llhw, VENDOR_QORVO_OUI,
					  LLHW_VENDOR_CMD_PCTT_SETUP_HW, NULL,
					  0);

		if (r)
			local->results.status =
				PCTT_STATUS_RANGING_INTERNAL_ERROR;
		pctt_report(local);
		pctt_session_set_state(local, PCTT_SESSION_STATE_IDLE);
		session->test_on_going = false;
		session->stop_request = false;
	}
}

struct mcps802154_access_ops pctt_access_ops = {
	.common = {
		.access_done = pctt_access_done,
	},
	.tx_get_frame = pctt_tx_get_frame,
	.tx_return = pctt_tx_return,
	.rx_frame = pctt_rx_frame,
};

static struct mcps802154_access *
pctt_get_access_periodic_tx(struct pctt_local *local, u32 next_timestamp_dtu)
{
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	struct mcps802154_access *access = &local->access;
	struct pctt_slot *s = local->slots;
	u32 frame_dtu;
	access->hrp_uwb_params = &session->hrp_uwb_params;

	/* Unique frame in this access. */
	*s = (struct pctt_slot){
		.is_tx = true,
	};
	frame_dtu = session->first_access ? next_timestamp_dtu :
					    session->next_timestamp_dtu;

	pctt_access_setup_frame(local, s, frame_dtu, &local->frames[0],
				&local->sts_params[0]);

	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &pctt_access_ops;
	access->duration_dtu = 0;
	access->n_frames = 1;
	access->frames = local->frames;
	access->timestamp_dtu = frame_dtu;
	/* Compute next transmit date. */
	session->next_timestamp_dtu = frame_dtu + p->gap_duration_dtu;

	pctt_randomize_psdu(local);

	return access;
}

static struct mcps802154_access *
pctt_get_access_per_rx(struct pctt_local *local, u32 next_timestamp_dtu)
{
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	struct mcps802154_access *access = &local->access;
	struct pctt_slot *s = local->slots;
	u32 frame_timestamp_dtu;
	access->hrp_uwb_params = &session->hrp_uwb_params;

	/* Unique frame in this access. */
	*s = (struct pctt_slot){
		.is_immediate = !session->first_rx_synchronized,
		.timeout_dtu = session->first_rx_synchronized ?
				       2 * pctt_rx_margin(p->gap_duration_dtu) :
				       -1,
	};

	frame_timestamp_dtu = session->first_rx_synchronized ?
				      session->next_timestamp_dtu :
				      next_timestamp_dtu;

	pctt_access_setup_frame(local, s, frame_timestamp_dtu,
				&local->frames[0], &local->sts_params[0]);

	access->ops = &pctt_access_ops;
	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->timestamp_dtu = frame_timestamp_dtu;
	access->duration_dtu =
		session->first_rx_synchronized ? p->gap_duration_dtu : 0;
	access->n_frames = 1;
	access->frames = local->frames;

	return access;
}

static int pctt_handle_loopback(struct mcps802154_access *access)
{
	struct pctt_local *local = access_to_local(access);
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	struct llhw_vendor_cmd_pctt_handle_loopback handle_loopback = {};

	handle_loopback.ant_set_id = p->tx_antenna_selection;
	handle_loopback.data_payload = p->data_payload;
	handle_loopback.data_payload_len = p->data_payload_len;

	return mcps802154_vendor_cmd(local->llhw, VENDOR_QORVO_OUI,
				     LLHW_VENDOR_CMD_PCTT_HANDLE_LOOPBACK,
				     &handle_loopback, sizeof(handle_loopback));
}

static int pctt_tx_done_loopback(struct mcps802154_access *access)
{
	struct pctt_local *local = access_to_local(access);
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	struct llhw_vendor_cmd_pctt_get_loopback_info loopback_info = {};
	int r;

	r = mcps802154_vendor_cmd(local->llhw, VENDOR_QORVO_OUI,
				  LLHW_VENDOR_CMD_PCTT_GET_LOOPBACK_INFO,
				  &loopback_info, sizeof(loopback_info));
	if (r)
		return r;

	local->results.status = loopback_info.success ?
					PCTT_STATUS_RANGING_SUCCESS :
					PCTT_STATUS_RANGING_TX_FAILED;

	local->results.tests.loopback.rssi = loopback_info.rssi;

	if (loopback_info.success) {
		/* Compare data received with the one sent. */
		struct sk_buff *rx_skb = loopback_info.skb;
		WARN_RETURN_ON(!rx_skb, -EFAULT);

		if ((rx_skb->len != p->data_payload_len) ||
		    memcmp(rx_skb->data, p->data_payload, rx_skb->len)) {
			local->results.status = PCTT_STATUS_RANGING_TX_FAILED;
		}

		/* Free rx_frame skb. */
		kfree_skb(rx_skb);
	}

	local->results.tests.loopback.rx_ts_int =
		(u32)(loopback_info.rx_timestamp_rctu >> PCTT_TIMESTAMP_SHIFT);
	local->results.tests.loopback.rx_ts_frac =
		(u16)(loopback_info.rx_timestamp_rctu &
		      (((unsigned long)1 << PCTT_TIMESTAMP_SHIFT) - 1));
	local->results.tests.loopback.tx_ts_int =
		(u32)(loopback_info.tx_timestamp_rctu >> PCTT_TIMESTAMP_SHIFT);
	local->results.tests.loopback.tx_ts_frac =
		(u16)(loopback_info.tx_timestamp_rctu &
		      (((unsigned long)1 << PCTT_TIMESTAMP_SHIFT) - 1));

	/* Request end of current access. */
	return 1;
}

struct mcps802154_access_vendor_ops pctt_access_ops_loopback = {
	.common = {
		.access_done = pctt_access_done,
	},
	.handle = pctt_handle_loopback,
	.tx_done = pctt_tx_done_loopback,
};

static struct mcps802154_access *
pctt_get_access_loopback(struct pctt_local *local, u32 next_timestamp_dtu)
{
	struct mcps802154_access *access = &local->access;

	access->method = MCPS802154_ACCESS_METHOD_VENDOR;
	access->vendor_ops = &pctt_access_ops_loopback;
	access->duration_dtu = 0;
	access->timestamp_dtu = next_timestamp_dtu;
	access->n_frames = 0;
	access->frames = NULL;
	return access;
}

static struct mcps802154_access *
pctt_get_access_ss_twr(struct pctt_local *local, u32 next_timestamp_dtu)
{
	struct mcps802154_access *access = &local->access;
	struct pctt_session *session = &local->session;
	struct pctt_slot *s = local->slots;
	const struct pctt_session_params *p = &session->params;
	const bool is_initiator = p->device_role == PCTT_DEVICE_ROLE_INITIATOR;
	int nb_frames;
	u32 frame_dtu;
	int i;
	access->hrp_uwb_params = &session->hrp_uwb_params;

	/* First frames. */
	*s = (struct pctt_slot){
		.is_tx = is_initiator,
		.timeout_dtu = -1,
	};
	s++;
	/* Second frames. */
	*s = (struct pctt_slot){
		.is_tx = !is_initiator,
	};
	s++;

	if (is_initiator) {
		struct pctt_test_ss_twr_results *ss_twr =
			&local->results.tests.ss_twr;

		ss_twr->tx_timestamps_rctu =
			mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
				local->llhw, next_timestamp_dtu,
				access->hrp_uwb_params, access->channel,
				p->tx_antenna_selection);
	}

	frame_dtu = next_timestamp_dtu;
	nb_frames = is_initiator ? 2 : 1;

	for (i = 0; i < nb_frames; i++) {
		struct mcps802154_access_frame *frame = &local->frames[i];
		struct mcps802154_sts_params *sts_params =
			&local->sts_params[i];

		s = &local->slots[i];
		pctt_access_setup_frame(local, s, frame_dtu, frame, sts_params);
		frame_dtu += p->slot_duration_dtu;
	}

	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &pctt_access_ops;
	access->timestamp_dtu = next_timestamp_dtu;
	access->frames = local->frames;
	access->n_frames = nb_frames;
	access->duration_dtu = frame_dtu - next_timestamp_dtu;

	return access;
}

struct mcps802154_access *pctt_get_access(struct mcps802154_region *region,
					  u32 next_timestamp_dtu,
					  int next_in_region_dtu,
					  int region_duration_dtu)
{
	struct pctt_local *local = region_to_local(region);
	struct pctt_session *session = &local->session;
	struct mcps802154_access *access = NULL;

	if (!session->test_on_going)
		return NULL;

	switch (session->cmd_id) {
	case PCTT_ID_ATTR_PERIODIC_TX:
		access = pctt_get_access_periodic_tx(local, next_timestamp_dtu);
		break;
	case PCTT_ID_ATTR_PER_RX:
	case PCTT_ID_ATTR_RX:
		access = pctt_get_access_per_rx(local, next_timestamp_dtu);
		break;
	case PCTT_ID_ATTR_LOOPBACK:
		access = pctt_get_access_loopback(local, next_timestamp_dtu);
		break;
	case PCTT_ID_ATTR_SS_TWR:
		access = pctt_get_access_ss_twr(local, next_timestamp_dtu);
		break;
	default: /* LCOV_EXCL_START */
		/* Impossible to cover with unit test.
		 * The only way is a memory corruption on the cmd_id. */
		break;
		/* LCOV_EXCL_STOP */
	}

	WARN_ON(!access);
	if (session->first_access) {
		int r;

		r = mcps802154_vendor_cmd(local->llhw, VENDOR_QORVO_OUI,
					  LLHW_VENDOR_CMD_PCTT_SETUP_HW,
					  &session->setup_hw,
					  sizeof(session->setup_hw));
		if (r) {
			local->results.status =
				PCTT_STATUS_RANGING_INTERNAL_ERROR;
			pctt_report(local);
			pctt_session_set_state(local, PCTT_SESSION_STATE_IDLE);
			return NULL;
		}
	}

	session->first_access = false;
	return access;
}
