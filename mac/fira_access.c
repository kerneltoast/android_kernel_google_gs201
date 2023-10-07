/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2022 Qorvo US, Inc.
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

#include "fira_round_hopping_sequence.h"
#include "fira_access.h"
#include "fira_session.h"
#include "fira_frame.h"
#include "fira_trace.h"
#include "fira_sts.h"

#include <asm/unaligned.h>
#include <linux/string.h>
#include <linux/ieee802154.h>
#include <linux/math64.h>
#include <linux/limits.h>
#include <linux/errno.h>
#include <net/mcps802154_frame.h>

#include "warn_return.h"

#define FIRA_STS_FOM_THRESHOLD 153
#define FIRA_RSSI_MAX 0xff

/**
 * sat_fp() - Saturate the range of fixed-point
 * @x: fixed-point value on s32.
 *
 * Return: value saturate to s16 range
 */
static s16 sat_fp(s32 x)
{
	if (x > S16_MAX)
		return S16_MAX;
	else if (x < S16_MIN)
		return S16_MIN;
	else
		return x;
}

/**
 * map_q11_to_2pi() - Map a Fixed Point angle to an signed 16 bit interger
 * @x: angle as Q11 fixed_point value in range [-PI, PI]
 *
 * Return: the angle mapped to [INT16_MIN, INT16_MAX]
 */
static s16 map_q11_to_2pi(s16 x)
{
	const s16 pi = 6434; /* Same as round(M_PI * (1 << 11)). */

	s32 temp = (s32)x * S16_MAX;
	temp /= pi;

	return sat_fp(temp);
}

/**
 * fira_access_setup_frame() - Fill an access frame from a FiRa slot.
 * @local: FiRa context.
 * @session: Session.
 * @frame: Access frame.
 * @sts_params: Where to store STS parameters.
 * @slot: Corresponding slot.
 * @frame_dtu: frame transmission or reception date.
 * @is_tx: true for TX.
 */
static void fira_access_setup_frame(struct fira_local *local,
				    struct fira_session *session,
				    struct mcps802154_access_frame *frame,
				    struct mcps802154_sts_params *sts_params,
				    const struct fira_slot *slot, u32 frame_dtu,
				    bool is_tx)
{
	const struct fira_session_params *params = &session->params;
	struct mcps802154_access *access = &local->access;
	struct mcps802154_sts_params *sts_params_for_access = NULL;
	int rframe_config = session->params.rframe_config;

	const struct fira_measurement_sequence_step *current_ms_step =
		fira_session_get_meas_seq_step(session);

	bool is_rframe = slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX;
	bool is_last_rframe = slot->message_id == FIRA_MESSAGE_ID_RANGING_FINAL;
	bool is_first_frame = slot->message_id == FIRA_MESSAGE_ID_CONTROL;
	bool request_rssi = session->params.report_rssi;
	if (is_rframe) {
		fira_sts_get_sts_params(session, slot, sts_params->v,
					sizeof(sts_params->v), sts_params->key,
					sizeof(sts_params->key));
		sts_params->n_segs = params->number_of_sts_segments;
		sts_params->seg_len =
			params->sts_length == FIRA_STS_LENGTH_128 ?
				128 :
				params->sts_length == FIRA_STS_LENGTH_32 ? 32 :
									   64;
		sts_params->sp2_tx_gap_4chips = 0;
		sts_params->sp2_rx_gap_4chips[0] = 0;
		sts_params->sp2_rx_gap_4chips[1] = 0;
		sts_params->sp2_rx_gap_4chips[2] = 0;
		sts_params->sp2_rx_gap_4chips[3] = 0;
		sts_params_for_access = sts_params;
	}

	if (is_tx) {
		u8 flags = MCPS802154_TX_FRAME_CONFIG_TIMESTAMP_DTU;

		/* Add a small margin to the Tx timestamps. */
		if (!is_first_frame)
			frame_dtu += FIRA_TX_MARGIN_US *
				     local->llhw->dtu_freq_hz / 1000000;

		if (is_rframe) {
			struct fira_ranging_info *ranging_info;

			ranging_info =
				&local->ranging_info[slot->ranging_index];
			ranging_info->timestamps_rctu[slot->message_id] =
				mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
					local->llhw, frame_dtu,
					access->hrp_uwb_params, access->channel,
					slot->tx_ant_set);

			flags |= MCPS802154_TX_FRAME_CONFIG_RANGING;
			if (rframe_config == FIRA_RFRAME_CONFIG_SP3)
				flags |= MCPS802154_TX_FRAME_CONFIG_SP3;
			else
				flags |= MCPS802154_TX_FRAME_CONFIG_SP1;
			if (!is_last_rframe)
				flags |=
					MCPS802154_TX_FRAME_CONFIG_KEEP_RANGING_CLOCK;
		} else if (is_first_frame) {
			flags |= MCPS802154_TX_FRAME_CONFIG_RANGING_ROUND;
		}
		*frame = (struct mcps802154_access_frame){
			.is_tx = true,
			.tx_frame_config = {
				.timestamp_dtu = frame_dtu,
				.flags = flags,
				.ant_set_id = slot->tx_ant_set,
			},
			.sts_params = sts_params_for_access,
		};
	} else {
		u8 flags = MCPS802154_RX_FRAME_CONFIG_TIMESTAMP_DTU;
		u16 request = 0;

		if (request_rssi)
			request |= MCPS802154_RX_FRAME_INFO_RSSI;
		if (is_rframe) {
			flags |= MCPS802154_RX_FRAME_CONFIG_RANGING;
			if (rframe_config == FIRA_RFRAME_CONFIG_SP3)
				flags |= MCPS802154_RX_FRAME_CONFIG_SP3;
			else
				flags |= MCPS802154_RX_FRAME_CONFIG_SP1;
			if (!is_last_rframe)
				flags |=
					MCPS802154_RX_FRAME_CONFIG_KEEP_RANGING_CLOCK;
			request |= MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
				   MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM;
			if (current_ms_step->type !=
			    FIRA_MEASUREMENT_TYPE_RANGE) {
				flags |=
					MCPS802154_RX_FRAME_CONFIG_RANGING_PDOA;
				request |=
					MCPS802154_RX_FRAME_INFO_RANGING_PDOA |
					MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM;
			}
			if (params->ranging_round_usage ==
				    FIRA_RANGING_ROUND_USAGE_SSTWR &&
			    session->params.device_type ==
				    FIRA_DEVICE_TYPE_CONTROLEE)
				request |=
					MCPS802154_RX_FRAME_INFO_RANGING_OFFSET;
		}
		*frame = (struct mcps802154_access_frame){
			.is_tx = false,
			.rx = {
				.frame_config = {
					.timestamp_dtu = frame_dtu,
					.flags = flags,
					.ant_set_id = slot->rx_ant_set,
				},
				.frame_info_flags_request = request,
			},
			.sts_params = sts_params_for_access,
		};
	}
}

static void fira_controlee_resync(struct fira_session *session,
				  u32 phy_sts_index, u32 timestamp_dtu)
{
	u32 block_idx, round_idx, slot_idx;
	int block_start_timestamp_dtu;
	const struct fira_session_params *params = &session->params;

	fira_sts_convert_phy_sts_idx_to_time_indexes(
		session, phy_sts_index, &block_idx, &round_idx, &slot_idx);
	block_start_timestamp_dtu =
		timestamp_dtu -
		(round_idx * session->params.round_duration_slots + slot_idx) *
			params->slot_duration_dtu;

	/* Update the session. */
	session->block_start_dtu = block_start_timestamp_dtu;
	session->block_index = block_idx;
	session->round_index = round_idx;
	session->controlee.synchronised = true;
	session->controlee.next_round_index_valid = false;
	session->controlee.block_index_sync = block_idx;
}

static bool fira_rx_sts_good(struct fira_local *local,
			     const struct mcps802154_rx_frame_info *info)
{
	int i;
	if (!(info->flags & MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM))
		return false;
	for (i = 0; i < MCPS802154_STS_N_SEGS_MAX; i++) {
		if (info->ranging_sts_fom[i] < FIRA_STS_FOM_THRESHOLD)
			return false;
	}
	return true;
}

static void fira_ranging_info_set_status(const struct fira_session *session,
					 struct fira_ranging_info *ranging_info,
					 enum fira_ranging_status status,
					 u8 slot_index)
{
	/* Report first error. */
	if (ranging_info->status)
		return;
	ranging_info->status = status;
	ranging_info->slot_index = slot_index;
	fira_session_set_range_data_ntf_status(session, ranging_info);
}

static void
fira_diagnostic_rssis(const struct mcps802154_rx_measurement_info *info,
		      struct fira_diagnostic *diagnostic)
{
	if (info->flags & MCPS802154_RX_MEASUREMENTS_RSSIS) {
		int max = max(MCPS802154_RSSIS_N_MAX - 1, info->n_rssis);
		int i;
		for (i = 0; i < max; i++)
			diagnostic->rssis_q1[i] = info->rssis_q1[i];
		diagnostic->n_rssis = i;
	}
}

static void
fira_diagnostic_aoas(const struct mcps802154_rx_measurement_info *info,
		     struct fira_diagnostic *diagnostic)
{
	int max = max(MCPS802154_RX_AOA_MEASUREMENTS_MAX - 1, info->n_aoas);
	int i;

	for (i = 0; i < max; i++)
		diagnostic->aoas[i] = info->aoas[i];
	diagnostic->n_aoas = info->n_aoas;
}

static struct mcps802154_rx_cir *
fira_diagnostic_cirs_alloc(const struct mcps802154_rx_measurement_info *info)
{
	const struct mcps802154_rx_cir_sample_window *si;
	struct mcps802154_rx_cir_sample_window *so;
	struct mcps802154_rx_cir *cirs;
	int i;
	int j;

	cirs = kmalloc(info->n_cirs * sizeof(struct mcps802154_rx_cir),
		       GFP_KERNEL);
	if (!cirs)
		return NULL;

	for (i = 0; i < info->n_cirs; i++) {
		so = &cirs[i].sample_window;
		si = &info->cirs[i].sample_window;
		so->samples =
			kmalloc(si->n_samples * si->sizeof_sample, GFP_KERNEL);

		if (!so->samples)
			goto failed;
	}
	return cirs;

failed:
	for (j = 0; j < i; j++)
		kfree(cirs[j].sample_window.samples);
	kfree(cirs);
	return NULL;
}

static void
fira_diagnostic_cirs_copy(const struct mcps802154_rx_measurement_info *info,
			  struct fira_diagnostic *diagnostic)
{
	int i;

	for (i = 0; i < info->n_cirs; i++) {
		struct mcps802154_rx_cir *cir_in;
		struct mcps802154_rx_cir *cir_out;
		struct mcps802154_rx_cir_sample_window *si;
		struct mcps802154_rx_cir_sample_window *so;

		cir_out = &diagnostic->cirs[i];
		cir_in = &info->cirs[i];
		so = &cir_out->sample_window;
		si = &cir_in->sample_window;

		cir_out->fp_index = cir_in->fp_index;
		cir_out->fp_snr = cir_in->fp_snr;
		cir_out->fp_ns_q6 = cir_in->fp_ns_q6;
		cir_out->pp_index = cir_in->pp_index;
		cir_out->pp_snr = cir_in->pp_snr;
		cir_out->pp_ns_q6 = cir_in->pp_ns_q6;
		cir_out->fp_sample_offset = cir_in->fp_sample_offset;
		so->n_samples = si->n_samples;
		so->sizeof_sample = si->sizeof_sample;

		memcpy(so->samples, si->samples,
		       si->n_samples * si->sizeof_sample);
	}
	diagnostic->n_cirs = i;
}

static void
fira_diagnostic_cirs(const struct mcps802154_rx_measurement_info *info,
		     struct fira_diagnostic *diagnostic)
{
	if (info->flags & MCPS802154_RX_MEASUREMENTS_CIRS) {
		diagnostic->cirs = fira_diagnostic_cirs_alloc(info);
		if (diagnostic->cirs)
			fira_diagnostic_cirs_copy(info, diagnostic);
		else
			diagnostic->n_cirs = 0;
	}
}

static void fira_diagnostic(struct fira_local *local,
			    struct fira_session *session, void *rx_ctx,
			    int slot_idx)
{
	const struct fira_session_params *params = &session->params;
	struct fira_diagnostic *diagnostic = &local->diagnostics[slot_idx];
	struct mcps802154_rx_measurement_info info = {};
	int r;

	WARN_ON(diagnostic->cirs);

	if (params->diagnostic_report_flags &
	    FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_RSSIS)
		info.flags |= MCPS802154_RX_MEASUREMENTS_RSSIS;
	if (params->diagnostic_report_flags &
	    FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_CIRS)
		info.flags |= MCPS802154_RX_MEASUREMENTS_CIRS;

	if (!info.flags)
		return;

	r = mcps802154_rx_get_measurement(local->llhw, rx_ctx, &info);

	if (r)
		return;

	fira_diagnostic_rssis(&info, diagnostic);
	fira_diagnostic_cirs(&info, diagnostic);
}

static void fira_diagnostic_free(struct fira_local *local)
{
	int i;

	for (i = 0; i < local->access.n_frames; i++) {
		struct fira_diagnostic *diagnostic = &local->diagnostics[i];
		int j;

		for (j = 0; j < diagnostic->n_cirs; j++)
			kfree(diagnostic->cirs[j].sample_window.samples);

		kfree(diagnostic->cirs);
		diagnostic->cirs = NULL;
		diagnostic->n_cirs = 0;
	}
}

static void fira_rx_frame_ranging(struct fira_local *local,
				  const struct fira_slot *slot,
				  struct sk_buff *skb,
				  const struct mcps802154_rx_frame_info *info)
{
	const struct fira_session *session = local->current_session;
	const struct fira_session_params *params = &session->params;
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};
	bool pdoa_info_present;

	if (!fira_rx_sts_good(local, info)) {
		fira_ranging_info_set_status(
			session, ranging_info,
			FIRA_STATUS_RANGING_RX_PHY_STS_FAILED, slot->index);
		return;
	}

	if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU)) {
		fira_ranging_info_set_status(
			session, ranging_info,
			FIRA_STATUS_RANGING_RX_PHY_TOA_FAILED, slot->index);
		return;
	}
	ranging_info->timestamps_rctu[slot->message_id] = info->timestamp_rctu;

	pdoa_info_present = info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA;

	if (pdoa_info_present) {
		struct fira_local_aoa_info *local_aoa;
		bool pdoa_fom_info_present =
			info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM;
		s16 local_pdoa_q11 = 0;
		s16 local_aoa_q11 = 0;
		const struct fira_measurement_sequence_step *current_step =
			fira_session_get_meas_seq_step(session);
		struct mcps802154_rx_measurement_info meas_info = {};
		int r;

		meas_info.flags |= MCPS802154_RX_MEASUREMENTS_AOAS;
		r = mcps802154_rx_get_measurement(
			local->llhw, ranging_info->rx_ctx, &meas_info);

		if (!r && meas_info.flags & MCPS802154_RX_MEASUREMENTS_AOAS &&
		    meas_info.n_aoas) {
			struct fira_diagnostic *diagnostic =
				&local->diagnostics[slot->index];

			/* TODO: Find which aoas index to use. */
			local_pdoa_q11 = meas_info.aoas[0].pdoa_rad_q11;
			local_aoa_q11 = meas_info.aoas[0].aoa_rad_q11;
			if (params->diagnostic_report_flags &
			    FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_AOAS)
				fira_diagnostic_aoas(&meas_info, diagnostic);
		}

		switch (current_step->type) {
		case FIRA_MEASUREMENT_TYPE_AOA:
			local_aoa = &ranging_info->local_aoa;
			break;
		case FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH:
			local_aoa = &ranging_info->local_aoa_azimuth;
			break;
		case FIRA_MEASUREMENT_TYPE_AOA_ELEVATION:
			local_aoa = &ranging_info->local_aoa_elevation;
			break;
		case FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH_ELEVATION:
			local_aoa = (slot->message_id ==
				     FIRA_MESSAGE_ID_RANGING_FINAL) ?
					    &ranging_info->local_aoa_elevation :
					    &ranging_info->local_aoa_azimuth;
			break;
		default: /* LCOV_EXCL_START */
			local_aoa = NULL;
			/* LCOV_EXCL_STOP */
		}

		/* LCOV_EXCL_START */
		if (local_aoa) {
			/* LCOV_EXCL_STOP */
			local_aoa->present = true;
			local_aoa->rx_ant_set = slot->rx_ant_set;
			local_aoa->pdoa_2pi = map_q11_to_2pi(local_pdoa_q11);
			local_aoa->aoa_2pi = map_q11_to_2pi(local_aoa_q11);
			/* LCOV_EXCL_START */
			/* FoM is always expected when PDoA present. */
			if (pdoa_fom_info_present)
				/* LCOV_EXCL_STOP */
				local_aoa->aoa_fom = info->ranging_pdoa_fom;
		}
	}

	if (info->flags & MCPS802154_RX_FRAME_INFO_RANGING_OFFSET) {
		ranging_info->clock_offset_q26 =
			div64_s64((s64)info->ranging_offset_rctu << 26,
				  info->ranging_tracking_interval_rctu);
		ranging_info->clock_offset_present = true;
	}

	if (skb) {
		if (fira_frame_header_check_decrypt(local, slot, skb,
						    &ie_get) ||
		    !fira_frame_rframe_payload_check(local, slot, skb,
						     &ie_get)) {
			fira_ranging_info_set_status(
				session, ranging_info,
				FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				slot->index);
		}
	}
}

static void fira_rx_frame_control(struct fira_local *local,
				  const struct fira_slot *slot,
				  struct sk_buff *skb,
				  const struct mcps802154_rx_frame_info *info)
{
	struct mcps802154_access *access = &local->access;
	struct fira_ranging_info *ri =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};
	const struct fira_session_params *params = NULL;
	struct fira_session *session;
	int header_len;
	__le16 src_short_addr;
	int last_slot_index = 0;
	int offset_in_access_duration_dtu;
	int left_duration_dtu;
	unsigned n_slots;
	u32 phy_sts_index;
	u8 *header;
	int r;

	if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU)) {
		fira_ranging_info_set_status(
			local->current_session, ri,
			FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED, slot->index);
		return;
	}

	offset_in_access_duration_dtu =
		info->timestamp_dtu - access->timestamp_dtu;

	/* Read the header to capture the session context. */
	header = skb->data;
	session = fira_rx_frame_control_header_check(local, slot, skb, &ie_get,
						     &phy_sts_index);
	if (!session)
		goto failed;

	fira_controlee_resync(session, phy_sts_index, info->timestamp_dtu);

	params = &session->params;
	ri->rx_ctx = session->rx_ctx[0];

	header_len = skb->data - header;
	src_short_addr = slot->controller_tx ? local->dst_short_addr :
					       slot->controlee->short_addr;

	/* Continue to decode the frame. */
	r = fira_sts_decrypt_frame(session, slot, skb, header_len, src_short_addr);
	if (r)
		goto failed;
	r = fira_frame_control_payload_check(local, skb, &ie_get, &n_slots,
					     &session->stop_inband,
					     &session->block_stride_len);
	if (!r)
		goto failed;

	left_duration_dtu =
		access->duration_dtu - offset_in_access_duration_dtu;

	/*
	 * The RCM has been received, remaining slots are: n_slots - 1.
	 * Stop if no time left to finish the ranging or if asked to.
	 */
	if (left_duration_dtu < (n_slots - 1) * params->slot_duration_dtu ||
	    session->stop_inband) {
		n_slots = 1;
	} else {
		int i;

		for (i = 1; i < n_slots; i++) {
			const struct fira_slot *slot = &local->slots[i];
			struct mcps802154_access_frame *frame =
				&local->frames[i];
			struct mcps802154_sts_params *sts_params =
				&local->sts_params[i];
			bool is_tx;
			u32 frame_dtu;

			is_tx = !slot->controller_tx;
			frame_dtu = info->timestamp_dtu +
				    params->slot_duration_dtu * slot->index;
			last_slot_index = slot->index;

			fira_access_setup_frame(local, session, frame,
						sts_params, slot, frame_dtu,
						is_tx);
		}
	}

	/* Trace the new (or not) session context and slots received. */
	trace_region_fira_rx_frame_control(local, session, left_duration_dtu,
					   n_slots);
	/* Update the access. */
	access->duration_dtu =
		offset_in_access_duration_dtu +
		(last_slot_index + 1) * params->slot_duration_dtu;
	access->n_frames = n_slots;
	return;

failed:
	session = local->current_session;
	params = &local->current_session->params;
	access->duration_dtu =
		offset_in_access_duration_dtu + params->slot_duration_dtu;
	fira_ranging_info_set_status(session, ri,
				     FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				     slot->index);
}

static void
fira_rx_frame_control_update(struct fira_local *local,
			     const struct fira_slot *slot, struct sk_buff *skb,
			     const struct mcps802154_rx_frame_info *info)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};

	if (fira_frame_header_check_decrypt(local, slot, skb, &ie_get))
		goto failed;

	return;
failed:
	fira_ranging_info_set_status(local->current_session, ranging_info,
				     FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				     slot->index);
}

static void fira_rx_frame_measurement_report(
	struct fira_local *local, const struct fira_slot *slot,
	struct sk_buff *skb, const struct mcps802154_rx_frame_info *info)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};

	if (fira_frame_header_check_decrypt(local, slot, skb, &ie_get))
		goto failed;

	if (!fira_frame_measurement_report_payload_check(local, slot, skb,
							 &ie_get))
		goto failed;

	return;
failed:
	fira_ranging_info_set_status(local->current_session, ranging_info,
				     FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				     slot->index);
}

static void
fira_rx_frame_result_report(struct fira_local *local,
			    const struct fira_slot *slot, struct sk_buff *skb,
			    const struct mcps802154_rx_frame_info *info)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};

	if (fira_frame_header_check_decrypt(local, slot, skb, &ie_get))
		goto failed;

	if (!fira_frame_result_report_payload_check(local, slot, skb, &ie_get))
		goto failed;

	return;
failed:
	fira_ranging_info_set_status(local->current_session, ranging_info,
				     FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				     slot->index);
}

static bool fira_do_process_rx_frame(const struct fira_session *session,
				     enum mcps802154_rx_error_type error,
				     struct fira_ranging_info *ranging_info,
				     u8 slot_index)
{
	enum fira_ranging_status status = FIRA_STATUS_RANGING_INTERNAL_ERROR;

	switch (error) {
	case MCPS802154_RX_ERROR_NONE:
		return true;
	case MCPS802154_RX_ERROR_SFD_TIMEOUT:
	case MCPS802154_RX_ERROR_TIMEOUT:
	case MCPS802154_RX_ERROR_HPDWARN:
		status = FIRA_STATUS_RANGING_RX_TIMEOUT;
		break;
	case MCPS802154_RX_ERROR_FILTERED:
	case MCPS802154_RX_ERROR_BAD_CKSUM:
		status = FIRA_STATUS_RANGING_RX_MAC_DEC_FAILED;
		break;
	case MCPS802154_RX_ERROR_UNCORRECTABLE:
	case MCPS802154_RX_ERROR_OTHER:
	case MCPS802154_RX_ERROR_PHR_DECODE:
		status = FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED;
		break;
	}
	fira_ranging_info_set_status(session, ranging_info, status, slot_index);
	return false;
}

static void fira_rx_frame(struct mcps802154_access *access, int frame_idx,
			  struct sk_buff *skb,
			  const struct mcps802154_rx_frame_info *info,
			  enum mcps802154_rx_error_type error)
{
	struct fira_local *local = access_to_local(access);
	const struct fira_session_params *params;
	const struct fira_slot *slot = &local->slots[frame_idx];
	struct fira_ranging_info *ri =
		&local->ranging_info[slot->ranging_index];
	/* Don't initialize session before rx_frame_control. */
	struct fira_session *session;

	trace_region_fira_rx_frame(local->current_session, slot->message_id,
				   error);

	if (info && info->flags & MCPS802154_RX_FRAME_INFO_RSSI) {
		if ((ri->n_rx_rssis + 1) > FIRA_MESSAGE_ID_MAX)
			return;

		ri->rx_rssis[ri->n_rx_rssis++] =
			info->rssi < FIRA_RSSI_MAX ? info->rssi : FIRA_RSSI_MAX;
	}

	if (fira_do_process_rx_frame(local->current_session, error, ri,
				     slot->index)) {
		switch (slot->message_id) {
		case FIRA_MESSAGE_ID_RANGING_INITIATION:
		case FIRA_MESSAGE_ID_RANGING_RESPONSE:
		case FIRA_MESSAGE_ID_RANGING_FINAL:
			fira_rx_frame_ranging(local, slot, skb, info);
			break;
		case FIRA_MESSAGE_ID_CONTROL:
			fira_rx_frame_control(local, slot, skb, info);
			break;
		case FIRA_MESSAGE_ID_MEASUREMENT_REPORT:
			fira_rx_frame_measurement_report(local, slot, skb,
							 info);
			break;
		case FIRA_MESSAGE_ID_RESULT_REPORT:
			fira_rx_frame_result_report(local, slot, skb, info);
			break;
		case FIRA_MESSAGE_ID_CONTROL_UPDATE:
			fira_rx_frame_control_update(local, slot, skb, info);
			break;
		default:
			WARN_UNREACHABLE_DEFAULT();
		}
	}
	/* Current session can change after call of rx_frame_control function. */
	session = local->current_session;
	session->last_access_timestamp_dtu = access->timestamp_dtu;
	params = &session->params;

	kfree_skb(skb);
	fira_diagnostic(local, session, ri->rx_ctx, slot->index);

	/*
	 * Controlee: Stop round on error.
	 * Controller: Stop when all ranging fails.
	 */
	/*
	 * TODO:
	 * The usage of ri->status is hidden in function called.
	 * The reason of the end of access is not limpid.
	 */
	if (ri->status != FIRA_STATUS_RANGING_SUCCESS) {
		if (params->device_type == FIRA_DEVICE_TYPE_CONTROLEE ||
		    (slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX &&
		     --local->n_ranging_valid == 0))
			access->n_frames = frame_idx + 1;
	}
}

static struct sk_buff *fira_tx_get_frame(struct mcps802154_access *access,
					 int frame_idx)
{
	struct fira_local *local = access_to_local(access);
	struct fira_session *session = local->current_session;
	const struct fira_session_params *params = &session->params;
	const struct fira_slot *slot = &local->slots[frame_idx];
	struct sk_buff *skb;
	int header_len;

	trace_region_fira_tx_get_frame(session, slot->message_id);
	if (params->rframe_config == FIRA_RFRAME_CONFIG_SP3 &&
	    slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX)
		return NULL;

	skb = mcps802154_frame_alloc(local->llhw, IEEE802154_MTU, GFP_KERNEL);
	if (!skb)
		return NULL;

	fira_frame_header_put(local, slot, skb);

	switch (slot->message_id) {
	case FIRA_MESSAGE_ID_RANGING_INITIATION:
	case FIRA_MESSAGE_ID_RANGING_RESPONSE:
		fira_frame_rframe_payload_put(local, skb);
		break;
	case FIRA_MESSAGE_ID_RANGING_FINAL:
		break;
	case FIRA_MESSAGE_ID_CONTROL:
		fira_frame_control_payload_put(local, slot, skb);
		break;
	case FIRA_MESSAGE_ID_MEASUREMENT_REPORT:
		fira_frame_measurement_report_payload_put(local, slot, skb);
		break;
	case FIRA_MESSAGE_ID_RESULT_REPORT:
		fira_frame_result_report_payload_put(local, slot, skb);
		break;
	case FIRA_MESSAGE_ID_CONTROL_UPDATE:
		break;
	default: /* LCOV_EXCL_START */
		kfree_skb(skb);
		WARN_UNREACHABLE_DEFAULT();
		return NULL;
		/* LCOV_EXCL_STOP */
	}

	header_len = mcps802154_ie_put_end(skb, false);
	WARN_ON(header_len < 0);

	if (fira_sts_encrypt_frame(local->current_session, slot, skb, header_len,
				   local->src_short_addr)) {
		kfree_skb(skb);
		return NULL;
	}

	return skb;
}

static void fira_tx_return(struct mcps802154_access *access, int frame_idx,
			   struct sk_buff *skb,
			   enum mcps802154_access_tx_return_reason reason)
{
	struct fira_local *local = access_to_local(access);
	struct fira_session *session = local->current_session;
	int i;

	kfree_skb(skb);

	/* Error on TX. */
	trace_region_fira_tx_return(session, reason);
	if (reason == MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL) {
		for (i = 0; i < local->n_ranging_info; i++) {
			local->ranging_info[i].status =
				FIRA_STATUS_RANGING_TX_FAILED;
		}
	}
}

static void fira_access_done(struct mcps802154_access *access, bool error)
{
	struct fira_local *local = access_to_local(access);
	struct fira_session *session = local->current_session;
	u32 timestamp_dtu = access->timestamp_dtu;

	trace_region_fira_access_done(local, session, access->duration_dtu,
				      error);

	/* propagate llhw error to fira session */
	session->last_error = access->error;
	fira_session_fsm_access_done(local, session, error);
	fira_diagnostic_free(local);
	if (!error)
		/* No access are infinite normally. */
		timestamp_dtu += access->duration_dtu;
	/*
	 * Must be call after FSM access done, because
	 * shared resource in local are used.
	 */
	fira_check_all_missed_ranging(local, session, timestamp_dtu);
}

static __le16 fira_access_set_short_address(struct fira_local *local,
					    const struct fira_session *session,
					    struct mcps802154_access *access)
{
	const struct fira_session_params *params = &session->params;
	__le16 src_short_addr = mcps802154_get_short_addr(local->llhw);

	if (params->short_addr != IEEE802154_ADDR_SHORT_BROADCAST &&
	    src_short_addr != params->short_addr) {
		access->hw_addr_filt = (struct ieee802154_hw_addr_filt){
			.short_addr = params->short_addr,
		};
		access->hw_addr_filt_changed = IEEE802154_AFILT_SADDR_CHANGED;
		return params->short_addr;
	}
	access->hw_addr_filt_changed = 0;
	return src_short_addr;
}

static struct mcps802154_access_ops fira_controller_access_ops = {
	.common = {
		.access_done = fira_access_done,
	},
	.rx_frame = fira_rx_frame,
	.tx_get_frame = fira_tx_get_frame,
	.tx_return = fira_tx_return,
};

int fira_session_get_slot_count(const struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	int nb_controlee = fira_session_controlees_running_count(session);
	/* Control frame. */
	int slot_count = 1;

	if (nb_controlee) {
		/* Ranging initiation frame. */
		slot_count++;
		/* Ranging response frame(s). */
		slot_count += nb_controlee;
		/* Ranging final frame. */
		if (params->ranging_round_usage ==
		    FIRA_RANGING_ROUND_USAGE_DSTWR)
			slot_count++;
		/* Measurement report frame. */
		slot_count++;
		/* Result report frame(s). */
		slot_count += nb_controlee;
	}
	return slot_count;
}

struct mcps802154_access *
fira_get_access_controller(struct fira_local *local,
			   const struct fira_session_demand *fsd)
{
	struct fira_session *session = local->current_session;
	const struct fira_session_params *params = &session->params;
	const struct fira_measurement_sequence_step *step =
		fira_session_get_meas_seq_step(session);
	struct mcps802154_access *access = &local->access;
	struct mcps802154_access_frame *frame;
	struct mcps802154_sts_params *sts_params;
	struct fira_ranging_info *ri;
	struct fira_slot *s;
	u32 frame_dtu;
	int index = 0;
	int i;
	struct fira_controlee *controlee;

	trace_region_fira_get_access_controller(local, session, fsd);

	/* Update local context (shared memory used by all sessions). */
	local->src_short_addr =
		fira_access_set_short_address(local, session, access);
	local->dst_short_addr =
		session->n_current_controlees == 1 ?
			list_first_entry(&session->current_controlees,
					 struct fira_controlee, entry)
				->short_addr :
			IEEE802154_ADDR_SHORT_BROADCAST;

	/* Update session. */
	session->last_access_timestamp_dtu = fsd->timestamp_dtu;
	session->block_start_dtu = fsd->block_start_dtu;
	session->block_index += fsd->add_blocks;
	session->block_stride_len = params->block_stride_len;
	session->round_index = fsd->round_index;
	session->controller.next_block_index =
		session->block_index + session->block_stride_len + 1;
	session->next_round_index =
		params->round_hopping ?
			fira_round_hopping_sequence_get(
				session, session->controller.next_block_index) :
			0;

	/* Build number of controlee which are stopped. */
	local->n_stopped_controlees = 0;
	list_for_each_entry (controlee, &session->current_controlees, entry) {
		if (controlee->state == FIRA_CONTROLEE_STATE_STOPPING ||
		    controlee->state == FIRA_CONTROLEE_STATE_DELETING)
			local->stopped_controlees[local->n_stopped_controlees++] =
				controlee->short_addr;
	}
	/* Build number of controlee which are running. */
	local->n_ranging_valid =
		session->n_current_controlees - local->n_stopped_controlees;

	/* Reset 'n' ranging info to store info related to controlees. */
	local->n_ranging_info = local->n_ranging_valid;
	ri = local->ranging_info;
	memset(ri, 0,
	       local->n_ranging_valid * sizeof(struct fira_ranging_info));

	/* Prepare control message slot for fira_rx_frame. */
	s = local->slots;
	s->index = index++;
	s->controller_tx = true;
	s->ranging_index = 0;
	s->tx_ant_set = step->tx_ant_set_nonranging;
	s->message_id = FIRA_MESSAGE_ID_CONTROL;
	s->controlee = NULL;
	s++;
	/* Prepare other slots. */
	if (local->n_ranging_info) {
		s->index = index++;
		s->controller_tx = true;
		s->ranging_index = 0;
		s->tx_ant_set = step->tx_ant_set_ranging;
		s->message_id = FIRA_MESSAGE_ID_RANGING_INITIATION;
		s->controlee = NULL;

		s++;
		i = 0;
		list_for_each_entry (controlee, &session->current_controlees,
				     entry) {
			if (!fira_session_controlee_active(controlee))
				continue;
			ri->short_addr = controlee->short_addr;
			ri->rx_ctx = session->rx_ctx[i];
			/* Requested in fira_report_aoa function. */
			ri++;
			s->index = index++;
			s->controller_tx = false;
			s->ranging_index = i++;
			s->rx_ant_set = fira_session_get_rx_ant_set(
				session, FIRA_MESSAGE_ID_RANGING_RESPONSE);
			s->message_id = FIRA_MESSAGE_ID_RANGING_RESPONSE;
			s->controlee = controlee;
			s++;
		}

		if (params->ranging_round_usage ==
		    FIRA_RANGING_ROUND_USAGE_DSTWR) {
			s->index = index++;
			s->controller_tx = true;
			s->ranging_index = 0;
			s->tx_ant_set = step->tx_ant_set_ranging;
			s->message_id = FIRA_MESSAGE_ID_RANGING_FINAL;
			s->controlee = NULL;
			s++;
		}

		s->index = index++;
		s->controller_tx = true;
		s->ranging_index = 0;
		s->tx_ant_set = step->tx_ant_set_nonranging;
		s->message_id = FIRA_MESSAGE_ID_MEASUREMENT_REPORT;
		s->controlee = NULL;

		s++;
		if (params->result_report_phase) {
			i = 0;
			list_for_each_entry (controlee,
					     &session->current_controlees,
					     entry) {
				if (!fira_session_controlee_active(controlee))
					continue;
				s->index = index++;
				s->controller_tx = false;
				s->ranging_index = i++;
				s->rx_ant_set = step->rx_ant_set_nonranging;
				s->message_id = FIRA_MESSAGE_ID_RESULT_REPORT;
				s->controlee = controlee;
				s++;
			}
		}
	}

	/* Configure frames for fproc. */
	frame_dtu = fsd->timestamp_dtu;
	for (i = 0; i < index; i++) {
		s = &local->slots[i];
		frame = &local->frames[i];
		sts_params = &local->sts_params[i];

		fira_access_setup_frame(local, session, frame, sts_params, s,
					frame_dtu, s->controller_tx);

		frame_dtu += params->slot_duration_dtu;
	}

	/*
	 * Configure the access.
	 * 'duration_dtu' can be decrease on reception error.
	 */
	access->ops = &fira_controller_access_ops;
	access->timestamp_dtu = fsd->timestamp_dtu;
	access->duration_dtu = frame_dtu - fsd->timestamp_dtu;
	access->n_frames = index;
	return access;
}

static struct mcps802154_access_ops fira_controlee_access_ops = {
	.common = {
		.access_done = fira_access_done,
	},
	.rx_frame = fira_rx_frame,
	.tx_get_frame = fira_tx_get_frame,
	.tx_return = fira_tx_return,
};

struct mcps802154_access *
fira_get_access_controlee(struct fira_local *local,
			  const struct fira_session_demand *fsd)
{
	/*
	 * \                    Important:
	 *  '-.__.-'            It's almost forbidden to update session
	 *  /oo |--.--,--,--.   content for a controlee. Because, the
	 *  \_.-'._i__i__i_.'   session can change on control frame received.
	 *        """""""""
	 */
	struct fira_session *session = local->current_session;
	const struct fira_session_params *params = &session->params;
	struct mcps802154_access *access = &local->access;
	struct mcps802154_access_frame *frame;
	struct fira_ranging_info *ri;
	struct fira_slot *s;
	u16 request = MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU;

	trace_region_fira_get_access_controlee(local, session, fsd);

	/* Update local context (shared memory used by all sessions). */
	local->src_short_addr =
		fira_access_set_short_address(local, session, access);
	local->dst_short_addr = params->controller_short_addr;
	local->n_stopped_controlees = 0;

	/*
	 * Update session.
	 * Updated values are used in case of bad reception (timeout/error).
	 * Otherwise on good reception, many are override.
	 * by the controlee resync function.
	 */
	session->block_start_dtu = fsd->block_start_dtu;
	session->block_index += fsd->add_blocks;

	/* Reset one ranging info to store info related to the controller. */
	local->n_ranging_info = 1;
	ri = local->ranging_info;
	memset(ri, 0, sizeof(struct fira_ranging_info));
	/*
	 * Warning:
	 * - short_addr is used when rx control have an error.
	 * - Be careful to not initialize too much in ri, because
	 *   session can change on rx control.
	 */
	ri->short_addr = params->controller_short_addr;

	/* Prepare control message slot for fira_rx_frame. */
	s = local->slots;
	s->index = 0;
	s->controller_tx = true;
	s->ranging_index = 0;
	s->rx_ant_set =
		fira_session_get_meas_seq_step(session)->rx_ant_set_nonranging;
	s->message_id = FIRA_MESSAGE_ID_CONTROL;
	s->controlee = NULL;

	/* Configure frames for fproc. */
	if (params->report_rssi)
		request |= MCPS802154_RX_FRAME_INFO_RSSI;
	frame = local->frames;
	*frame = (struct mcps802154_access_frame){
		.is_tx = false,
		.rx = {
			.frame_config = {
				.timestamp_dtu = fsd->timestamp_dtu,
				.timeout_dtu = fsd->rx_timeout_dtu,
				.flags = MCPS802154_RX_FRAME_CONFIG_RANGING_ROUND |
					MCPS802154_RX_FRAME_CONFIG_TIMESTAMP_DTU,
				.ant_set_id = s->rx_ant_set,
			},
			.frame_info_flags_request = request,
		},
	};

	/*
	 * Configure the access.
	 * 'duration_dtu' will be overridden on control frame reception.
	 */
	access->ops = &fira_controlee_access_ops;
	access->timestamp_dtu = fsd->timestamp_dtu;
	access->duration_dtu = fsd->max_duration_dtu;
	access->n_frames = 1;
	return access;
}
