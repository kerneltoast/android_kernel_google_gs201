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

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/string.h>
#include <linux/limits.h>

#include <net/mcps802154_frame.h>
#include <net/fira_region_nl.h>

#include "fira_session.h"
#include "fira_round_hopping_sequence.h"
#include "fira_access.h"
#include "fira_frame.h"
#include "fira_trace.h"

inline static int
fira_compute_minimum_rssi(const struct fira_ranging_info *ranging_data)
{
	/*
	 * We want the WORST RSSI level.
	 * Please Note : RSSI is actually a negative number, but encoded
	 * as an absolute value.
	 */
	u8 min_rssi = 0;
	int i;

	for (i = 0; i < ranging_data->n_rx_rssis; i++)
		min_rssi = max(ranging_data->rx_rssis[i], min_rssi);
	return min_rssi;
}

inline static int
fira_compute_average_rssi(const struct fira_ranging_info *ranging_data)
{
	unsigned sum;
	int i;

	if (!ranging_data->n_rx_rssis)
		return 0;

	for (i = 0, sum = 0; i < ranging_data->n_rx_rssis; i++)
		sum += ranging_data->rx_rssis[i];
	return sum / i;
}

static int fira_report_local_aoa(struct sk_buff *msg, int nest_attr_id,
				 const struct fira_local_aoa_info *info)
{
	struct nlattr *aoa;

	aoa = nla_nest_start(msg, nest_attr_id);
	if (!aoa)
		goto nla_put_failure;
#define A(x) FIRA_RANGING_DATA_MEASUREMENTS_AOA_ATTR_##x
	if (nla_put_u8(msg, A(RX_ANTENNA_SET), info->rx_ant_set))
		goto nla_put_failure;
	if (nla_put_s16(msg, A(AOA_2PI), info->aoa_2pi))
		goto nla_put_failure;
	if (nla_put_s16(msg, A(PDOA_2PI), info->pdoa_2pi))
		goto nla_put_failure;
	if (nla_put_u8(msg, A(AOA_FOM), info->aoa_fom))
		goto nla_put_failure;
#undef A
	nla_nest_end(msg, aoa);
	return 0;
nla_put_failure:
	return -EMSGSIZE;
}

inline static int fira_session_report_measurement(
	const struct fira_session *session, struct sk_buff *msg,
	const struct fira_ranging_info *ranging_data, s64 rctu_freq_hz)
{
	const struct fira_session_params *params = &session->params;
	bool report_rssi_val_present = false;
	int report_rssi_val = 0;

	if (params->report_rssi &&
	    ranging_data->status == FIRA_STATUS_RANGING_SUCCESS) {
		switch (params->report_rssi) {
		case FIRA_RSSI_REPORT_MINIMUM:
			report_rssi_val_present = true;
			report_rssi_val =
				fira_compute_minimum_rssi(ranging_data);
			break;
		case FIRA_RSSI_REPORT_AVERAGE:
			report_rssi_val_present = true;
			report_rssi_val =
				fira_compute_average_rssi(ranging_data);
			break;
		default:
			break;
		}
	}

#define A(x) FIRA_RANGING_DATA_MEASUREMENTS_ATTR_##x

	if (nla_put_u16(msg, A(SHORT_ADDR), ranging_data->short_addr) ||
	    nla_put_u8(msg, A(STATUS), ranging_data->status))
		goto nla_put_failure;

	if (ranging_data->status) {
		if (nla_put_u8(msg, A(SLOT_INDEX), ranging_data->slot_index))
			goto nla_put_failure;
		return 0;
	}
	if (ranging_data->tof_present) {
		if (nla_put_s32(msg, A(DISTANCE_MM),
				fira_rctu_to_mm(rctu_freq_hz, ranging_data->tof_rctu)))
			goto nla_put_failure;
	}
	if (ranging_data->local_aoa.present) {
		if (fira_report_local_aoa(msg, A(LOCAL_AOA),
					  &ranging_data->local_aoa))
			goto nla_put_failure;
	}
	if (ranging_data->local_aoa_azimuth.present) {
		if (fira_report_local_aoa(msg, A(LOCAL_AOA_AZIMUTH),
					  &ranging_data->local_aoa_azimuth))
			goto nla_put_failure;
	}
	if (ranging_data->local_aoa_elevation.present) {
		if (fira_report_local_aoa(msg, A(LOCAL_AOA_ELEVATION),
					  &ranging_data->local_aoa_elevation))
			goto nla_put_failure;
	}
	if (ranging_data->remote_aoa_azimuth_present) {
		if (nla_put_s16(msg, A(REMOTE_AOA_AZIMUTH_2PI),
				ranging_data->remote_aoa_azimuth_2pi))
			goto nla_put_failure;
		if (ranging_data->remote_aoa_fom_present) {
			if (nla_put_u8(msg, A(REMOTE_AOA_AZIMUTH_FOM),
				       ranging_data->remote_aoa_azimuth_fom))
				goto nla_put_failure;
		}
	}
	if (ranging_data->remote_aoa_elevation_present) {
		if (nla_put_s16(msg, A(REMOTE_AOA_ELEVATION_PI),
				ranging_data->remote_aoa_elevation_pi))
			goto nla_put_failure;
		if (ranging_data->remote_aoa_fom_present) {
			if (nla_put_u8(msg, A(REMOTE_AOA_ELEVATION_FOM),
				       ranging_data->remote_aoa_elevation_fom))
				goto nla_put_failure;
		}
	}
	if (report_rssi_val_present) {
		if (nla_put_u8(msg, A(RSSI), report_rssi_val))
			goto nla_put_failure;
	}
	if (ranging_data->data_payload_len > 0) {
		if (nla_put(msg, A(DATA_PAYLOAD_RECV),
			    ranging_data->data_payload_len,
			    ranging_data->data_payload))
			goto nla_put_failure;
	}
	if (session->data_payload.sent) {
		if (nla_put_u32(msg, A(DATA_PAYLOAD_SEQ_SENT),
				session->data_payload.seq))
			goto nla_put_failure;
	}

#undef A
	return 0;
nla_put_failure:
	return -EMSGSIZE;
}

inline static int fira_report_measurement_stopped_controlee(struct sk_buff *msg,
							    __le16 short_addr)
{
#define A(x) FIRA_RANGING_DATA_MEASUREMENTS_ATTR_##x

	if (nla_put_u16(msg, A(SHORT_ADDR), short_addr) ||
	    nla_put_u8(msg, A(STOPPED), 1))
		goto nla_put_failure;

#undef A
	return 0;
nla_put_failure:
	return -EMSGSIZE;
}

inline static int
fira_session_report_ranging_data(const struct fira_session *session,
				 const struct fira_report_info *report_info,
				 int dtu_freq_hz, int dtu_rctu,
				 struct sk_buff *msg)
{
	const struct fira_session_params *params = &session->params;
	struct nlattr *data, *measurements, *measurement;
	int ranging_interval_ms = params->block_duration_dtu *
				  (session->block_stride_len + 1) /
				  (dtu_freq_hz / 1000);
	s64 rctu_freq_hz = (s64)dtu_freq_hz * dtu_rctu;

	int i;
	int n_ranging_to_send = 0;

	data = nla_nest_start(msg, FIRA_CALL_ATTR_RANGING_DATA);
	if (!data)
		goto nla_put_failure;

	if (nla_put_u32(msg, FIRA_RANGING_DATA_ATTR_BLOCK_INDEX,
			session->block_index) ||
	    nla_put_u32(msg, FIRA_RANGING_DATA_ATTR_RANGING_INTERVAL_MS,
			ranging_interval_ms))
		goto nla_put_failure;

	if (report_info->stopped) {
		enum fira_ranging_data_attrs_stopped_values stopped;

		if (session->stop_no_response)
			stopped = FIRA_RANGING_DATA_ATTR_STOPPED_NO_RESPONSE;
		else if (session->stop_inband)
			stopped = FIRA_RANGING_DATA_ATTR_STOPPED_IN_BAND;
		else
			stopped = FIRA_RANGING_DATA_ATTR_STOPPED_REQUEST;

		if (nla_put_u8(msg, FIRA_RANGING_DATA_ATTR_STOPPED, stopped))
			goto nla_put_failure;

		/*
		  Case where measurements are not available:
		   - A controller stop request.
		   - A controller max measurements reached.
		   - A controlee stop in band.
		*/
		if ((session->params.device_type ==
			     FIRA_DEVICE_TYPE_CONTROLLER &&
		     stopped == FIRA_RANGING_DATA_ATTR_STOPPED_REQUEST) ||
		    (session->params.device_type ==
			     FIRA_DEVICE_TYPE_CONTROLEE &&
		     stopped == FIRA_RANGING_DATA_ATTR_STOPPED_IN_BAND))
			goto end_report;
	}

	for (i = 0; i < report_info->n_ranging_data; i++) {
		if (report_info->ranging_data[i].notify)
			n_ranging_to_send++;
	}

	if (n_ranging_to_send + report_info->n_stopped_controlees) {
		measurements = nla_nest_start(
			msg, FIRA_RANGING_DATA_ATTR_MEASUREMENTS);
		if (!measurements)
			goto nla_put_failure;

		for (i = 0; i < report_info->n_ranging_data; i++) {
			if (!report_info->ranging_data[i].notify)
				continue;
			measurement = nla_nest_start(msg, 1);
			if (fira_session_report_measurement(
				    session, msg, &report_info->ranging_data[i],
				    rctu_freq_hz))
				goto nla_put_failure;
			nla_nest_end(msg, measurement);
		}

		for (i = 0; i < report_info->n_stopped_controlees; i++) {
			measurement = nla_nest_start(msg, 1);
			if (fira_report_measurement_stopped_controlee(
				    msg, report_info->stopped_controlees[i]))
				goto nla_put_failure;
			nla_nest_end(msg, measurement);
		}

		nla_nest_end(msg, measurements);
	}

end_report:
	nla_nest_end(msg, data);
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int
fira_session_report_diagnostic_rssi(const struct fira_diagnostic *diagnostic,
				    struct sk_buff *msg)
{
	struct nlattr *nest;
	int i;

	nest = nla_nest_start(
		msg, FIRA_RANGING_DIAGNOSTICS_FRAME_REPORTS_ATTR_RSSIS);
	if (!nest)
		goto nla_put_failure;
	for (i = 0; i < diagnostic->n_rssis; i++) {
		if (nla_put_u8(msg, i, diagnostic->rssis_q1[i]))
			goto nla_put_failure;
	}
	nla_nest_end(msg, nest);
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int
fira_session_report_diagnostic_aoa(const struct fira_diagnostic *diagnostic,
				   struct sk_buff *msg)
{
	const struct mcps802154_rx_aoa_measurements *aoa;
	struct nlattr *aoas, *aoa_report;
	int i;

	aoas = nla_nest_start(msg,
			      FIRA_RANGING_DIAGNOSTICS_FRAME_REPORTS_ATTR_AOAS);
	if (!aoas)
		goto nla_put_failure;
	for (i = 0; i < diagnostic->n_aoas; i++) {
		aoa = &diagnostic->aoas[i];

		aoa_report = nla_nest_start(msg, i);
		if (!aoa_report)
			goto nla_put_failure;
#define A(x) FIRA_RANGING_DIAGNOSTICS_FRAME_REPORTS_AOAS_ATTR_##x
		if (nla_put_s16(msg, A(TDOA), aoa->tdoa_rctu))
			goto nla_put_failure;
		if (nla_put_s16(msg, A(PDOA), aoa->pdoa_rad_q11))
			goto nla_put_failure;
		if (nla_put_s16(msg, A(AOA), aoa->aoa_rad_q11))
			goto nla_put_failure;
		if (nla_put_u8(msg, A(FOM), aoa->fom))
			goto nla_put_failure;
		if (nla_put_u8(msg, A(TYPE), aoa->type))
			goto nla_put_failure;
#undef A
		nla_nest_end(msg, aoa_report);
	}
	nla_nest_end(msg, aoas);
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int fira_session_report_cir_samples(const struct mcps802154_rx_cir *cir,
					   struct sk_buff *msg)
{
	const struct mcps802154_rx_cir_sample_window *sw = &cir->sample_window;
	const u8 *samples = sw->samples;
	struct nlattr *sample_nest;
	int i;

	sample_nest = nla_nest_start(
		msg,
		FIRA_RANGING_DIAGNOSTICS_FRAME_REPORTS_CIRS_ATTR_FP_SAMPLE_WINDOW);
	if (!sample_nest)
		goto nla_put_failure;
	for (i = 0; i < sw->n_samples; i++) {
		const u8 *sample = &samples[i * sw->sizeof_sample];

		if (nla_put(msg, i, sw->sizeof_sample, sample))
			goto nla_put_failure;
	}
	nla_nest_end(msg, sample_nest);
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int
fira_session_report_diagnostic_cir(const struct fira_diagnostic *diagnostic,
				   struct sk_buff *msg)
{
	struct nlattr *cirs_nest, *cir_nest;
	int i;

	cirs_nest = nla_nest_start(
		msg, FIRA_RANGING_DIAGNOSTICS_FRAME_REPORTS_ATTR_CIRS);
	if (!cirs_nest)
		goto nla_put_failure;
	for (i = 0; i < diagnostic->n_cirs; i++) {
		struct mcps802154_rx_cir *cir = &diagnostic->cirs[i];

		cir_nest = nla_nest_start(msg, i);
		if (!cir_nest)
			goto nla_put_failure;
#define A(x) FIRA_RANGING_DIAGNOSTICS_FRAME_REPORTS_CIRS_ATTR_##x
		if (nla_put_u16(msg, A(FP_IDX), cir->fp_index))
			goto nla_put_failure;
		if (nla_put_s16(msg, A(FP_SNR), cir->fp_snr))
			goto nla_put_failure;
		if (nla_put_u16(msg, A(FP_NS), cir->fp_ns_q6))
			goto nla_put_failure;
		if (nla_put_u16(msg, A(PP_IDX), cir->pp_index))
			goto nla_put_failure;
		if (nla_put_s16(msg, A(PP_SNR), cir->pp_snr))
			goto nla_put_failure;
		if (nla_put_u16(msg, A(PP_NS), cir->pp_ns_q6))
			goto nla_put_failure;
		if (nla_put_u16(msg, A(FP_SAMPLE_OFFSET),
				cir->fp_sample_offset))
			goto nla_put_failure;
#undef A
		if (fira_session_report_cir_samples(cir, msg))
			goto nla_put_failure;
		nla_nest_end(msg, cir_nest);
	}
	nla_nest_end(msg, cirs_nest);
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int fira_session_report_frame_diagnostics(
	const struct fira_session *session,
	const struct fira_report_info *report_info, struct sk_buff *msg)
{
	const struct fira_session_params *params = &session->params;
	struct nlattr *frame_nest, *reports_nest;
	bool is_controller = params->device_type == FIRA_DEVICE_TYPE_CONTROLLER;
	int i;

	frame_nest = nla_nest_start(
		msg, FIRA_RANGING_DIAGNOSTICS_ATTR_FRAME_REPORTS);
	if (!frame_nest)
		goto nla_put_failure;

	for (i = 0; i < report_info->n_slots; i++) {
		const struct fira_slot *slot = &report_info->slots[i];
		int is_tx = slot->controller_tx ? is_controller :
						  !is_controller;

		reports_nest = nla_nest_start(msg, i);
		if (!reports_nest)
			goto nla_put_failure;
#define A(x) FIRA_RANGING_DIAGNOSTICS_FRAME_REPORTS_ATTR_##x
		if (nla_put_u8(msg, A(ANT_SET),
			       is_tx ? slot->tx_ant_set : slot->rx_ant_set))
			goto nla_put_failure;
		if (nla_put_u8(msg, A(ACTION), is_tx))
			goto nla_put_failure;
		if (nla_put_u8(msg, A(MSG_ID), slot->message_id))
			goto nla_put_failure;
#undef A
		/* Specific reports are done for Rx frames only. */
		if (!is_tx) {
			const struct fira_diagnostic *diagnostic =
				&report_info->diagnostics[i];

			if (params->diagnostic_report_flags &
			    FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_RSSIS) {
				if (fira_session_report_diagnostic_rssi(
					    diagnostic, msg))
					goto nla_put_failure;
			}
			if (params->diagnostic_report_flags &
			    FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_AOAS) {
				if (fira_session_report_diagnostic_aoa(
					    diagnostic, msg))
					goto nla_put_failure;
			}
			if (params->diagnostic_report_flags &
			    FIRA_RANGING_DIAGNOSTICS_FRAME_REPORT_CIRS) {
				if (fira_session_report_diagnostic_cir(
					    diagnostic, msg))
					goto nla_put_failure;
			}
		}
		nla_nest_end(msg, reports_nest);
	}
	nla_nest_end(msg, frame_nest);
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static inline int fira_session_report_ranging_diagnostics(
	const struct fira_session *session,
	const struct fira_report_info *report_info, struct sk_buff *msg)
{
	const struct fira_session_params *params = &session->params;
	struct nlattr *diagnostics_nest;

	if (!params->report_diagnostics)
		return 0;

	diagnostics_nest =
		nla_nest_start(msg, FIRA_CALL_ATTR_RANGING_DIAGNOSTICS);
	if (!diagnostics_nest)
		goto nla_put_failure;

	if (fira_session_report_frame_diagnostics(session, report_info, msg))
		goto nla_put_failure;

	nla_nest_end(msg, diagnostics_nest);
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

struct fira_session *fira_session_new(struct fira_local *local, u32 session_id)
{
	struct fira_session *session;
	struct fira_session_params *params;
	int all_rx_ctx_size = FIRA_CONTROLEES_MAX * local->llhw->rx_ctx_size;
	void *rx_ctx_base = NULL;
	int i;

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return NULL;
	if (all_rx_ctx_size) {
		rx_ctx_base = kzalloc(all_rx_ctx_size, GFP_KERNEL);
		if (!rx_ctx_base)
			goto failed;
	}

	params = &session->params;
	session->id = session_id;
	session->measurements.reset = true;

	/* Explicit default parameters as implicit is zero. */
	params->ranging_round_usage = FIRA_RANGING_ROUND_USAGE_DSTWR;
	params->short_addr = IEEE802154_ADDR_SHORT_BROADCAST;
	params->controller_short_addr = IEEE802154_ADDR_SHORT_BROADCAST;
	params->slot_duration_dtu =
		FIRA_SLOT_DURATION_RSTU_DEFAULT * local->llhw->rstu_dtu;
	params->block_duration_dtu = FIRA_BLOCK_DURATION_MS_DEFAULT *
				     (local->llhw->dtu_freq_hz / 1000);
	params->round_duration_slots = FIRA_ROUND_DURATION_SLOTS_DEFAULT;
	params->max_rr_retry = FIRA_MAX_RR_RETRY_DEFAULT;
	params->round_hopping = false;
	params->priority = FIRA_PRIORITY_DEFAULT;
	params->sts_length = FIRA_STS_LENGTH_64;
	params->sts_config = FIRA_STS_MODE_STATIC;
	params->rframe_config = FIRA_RFRAME_CONFIG_SP3;
	params->preamble_duration = FIRA_PREAMBULE_DURATION_64;
	params->sfd_id = FIRA_SFD_ID_2;
	params->number_of_sts_segments = FIRA_STS_SEGMENTS_1;
	params->meas_seq.n_steps = 1;
	params->meas_seq.steps[0].type = FIRA_MEASUREMENT_TYPE_RANGE;
	params->meas_seq.steps[0].n_measurements = 1;
	params->meas_seq.steps[0].rx_ant_set_nonranging = 0;
	params->meas_seq.steps[0].rx_ant_sets_ranging[0] = 0;
	params->meas_seq.steps[0].rx_ant_sets_ranging[1] = 0;
	params->meas_seq.steps[0].tx_ant_set_nonranging = 0;
	params->meas_seq.steps[0].tx_ant_set_ranging = 0;
	/* Report parameters. */
	params->aoa_result_req = true;
	params->report_tof = true;
	params->result_report_phase = true;
	params->range_data_ntf_config = FIRA_RANGE_DATA_NTF_ALWAYS;
	params->range_data_ntf_proximity_near_rctu = 0;
	params->range_data_ntf_proximity_far_rctu = fira_mm_to_rctu(
		local, FIRA_RANGE_DATA_NTF_PROXIMITY_FAR_MM_DEFAULT);
	params->range_data_ntf_lower_bound_aoa_azimuth_2pi =
		FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_AZIMUTH_2PI_DEFAULT;
	params->range_data_ntf_upper_bound_aoa_azimuth_2pi =
		FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_AZIMUTH_2PI_DEFAULT;
	params->range_data_ntf_lower_bound_aoa_elevation_2pi =
		FIRA_SESSION_DATA_NTF_LOWER_BOUND_AOA_ELEVATION_2PI_DEFAULT;
	params->range_data_ntf_upper_bound_aoa_elevation_2pi =
		FIRA_SESSION_DATA_NTF_UPPER_BOUND_AOA_ELEVATION_2PI_DEFAULT;

	if (fira_round_hopping_sequence_init(session))
		goto failed;

	if (all_rx_ctx_size) {
		for (i = 0; i < FIRA_CONTROLEES_MAX; i++) {
			void *rx_ctx = (char *)rx_ctx_base +
				       i * local->llhw->rx_ctx_size;
			session->rx_ctx[i] = rx_ctx;
		}
	}

	INIT_LIST_HEAD(&session->current_controlees);

	fira_session_fsm_initialise(local, session);
	return session;

failed:
	kfree(rx_ctx_base);
	kfree(session);
	return NULL;
}

void fira_session_free(struct fira_local *local, struct fira_session *session)
{
	struct fira_controlee *controlee, *tmp_controlee;

	list_for_each_entry_safe (controlee, tmp_controlee,
				  &session->current_controlees, entry) {
		list_del(&controlee->entry);
		kfree(controlee);
	}
	fira_session_fsm_uninit(local, session);
	fira_round_hopping_sequence_destroy(session);
	kfree(session->rx_ctx[0]);
	kfree_sensitive(session);
}

int fira_session_set_controlees(struct fira_local *local,
				struct fira_session *session,
				struct list_head *controlees,
				int slot_duration_us, int n_controlees)
{
	int r;
	struct fira_controlee *controlee, *tmp_controlee;

	if (!fira_frame_check_n_controlees(session, n_controlees, false))
		return -EINVAL;

	list_for_each_entry_safe (controlee, tmp_controlee,
				  &session->current_controlees, entry) {
		fira_sts_controlee_deinit(controlee);
		list_del(&controlee->entry);
		kfree(controlee);
	}
	list_for_each_entry_safe (controlee, tmp_controlee, controlees, entry) {
		r = fira_sts_controlee_init(
			session, controlee, slot_duration_us,
			mcps802154_get_current_channel(local->llhw));
		if (r)
			return r;
		list_move_tail(&controlee->entry, &session->current_controlees);
	}
	session->n_current_controlees = n_controlees;
	return 0;
}

int fira_session_new_controlee(struct fira_local *local,
				struct fira_session *session,
				struct fira_controlee *new_controlee,
				int slot_duration_us, bool active_session)
{
	int r;
	struct fira_controlee *controlee;

	if (session->n_current_controlees == FIRA_CONTROLEES_MAX)
		return -ENOBUFS;

	list_for_each_entry (controlee, &session->current_controlees, entry) {
		if (new_controlee->short_addr == controlee->short_addr)
			return -EEXIST;
	}

	if (active_session) {
		new_controlee->state = FIRA_CONTROLEE_STATE_PENDING_RUN;
	} else {
		r = fira_sts_controlee_init(session, new_controlee,
				slot_duration_us,
				mcps802154_get_current_channel(local->llhw));
		if (r)
			return r;
	}
	list_move_tail(&new_controlee->entry, &session->current_controlees);
	session->n_current_controlees++;
	return 0;
}

int fira_session_del_controlee(struct fira_session *session,
				struct fira_controlee *del_controlee,
				bool active_session)
{
	struct fira_controlee *controlee, *tmp_controlee;
	int ret = -ENOENT;

	if (session->n_current_controlees < 1)
		return -ENOENT;

	list_for_each_entry_safe (controlee, tmp_controlee,
				  &session->current_controlees, entry) {
		if (del_controlee->short_addr == controlee->short_addr) {
			if (active_session) {
				if (controlee->state ==
				    FIRA_CONTROLEE_STATE_STOPPING)
					controlee->state =
						FIRA_CONTROLEE_STATE_DELETING;
				else if (controlee->state !=
					FIRA_CONTROLEE_STATE_DELETING) {
					controlee->state =
						FIRA_CONTROLEE_STATE_PENDING_DEL;
				} else {
					fira_sts_controlee_deinit(controlee);
					list_del(&controlee->entry);
					kfree(controlee);
					session->n_current_controlees--;
				}
			}
			list_del(&del_controlee->entry);
			kfree(del_controlee);
			ret = 0;
			break;
		}
	}
	return ret;
}

void fira_session_stop_controlees(struct fira_session *session)
{
	struct fira_controlee *controlee;

	list_for_each_entry (controlee, &session->current_controlees, entry) {
		controlee->state = FIRA_CONTROLEE_STATE_PENDING_STOP;
	}
}

void fira_session_restart_controlees(struct fira_session *session)
{
	struct fira_controlee *controlee;

	list_for_each_entry (controlee, &session->current_controlees, entry) {
		if (controlee->state != FIRA_CONTROLEE_STATE_PENDING_DEL &&
		    controlee->state != FIRA_CONTROLEE_STATE_DELETING)
			controlee->state = FIRA_CONTROLEE_STATE_RUNNING;
	}
}

int fira_session_controlees_running_count(const struct fira_session *session)
{
	struct fira_controlee *controlee;
	int count = 0;

	list_for_each_entry (controlee, &session->current_controlees, entry) {
		if (controlee->state == FIRA_CONTROLEE_STATE_RUNNING ||
		    controlee->state == FIRA_CONTROLEE_STATE_PENDING_STOP ||
		    controlee->state == FIRA_CONTROLEE_STATE_PENDING_DEL)
			count++;
	}
	return count;
}

void fira_session_update_controlees(struct fira_local *local,
				    struct fira_session *session)
{
	struct fira_controlee *controlee, *tmp_controlee;
	bool reset_rx_ctx = false;
	int i, slot_duration_us;

	slot_duration_us = (session->params.slot_duration_dtu * 1000) /
		   (local->llhw->dtu_freq_hz / 1000);

	list_for_each_entry_safe (controlee, tmp_controlee,
				  &session->current_controlees, entry) {
		if (controlee->state == FIRA_CONTROLEE_STATE_PENDING_RUN) {
			controlee->state = FIRA_CONTROLEE_STATE_RUNNING;
			reset_rx_ctx = true;
			if (fira_sts_controlee_init(
				    session, controlee, slot_duration_us,
				    mcps802154_get_current_channel(local->llhw)))
				controlee->state =
					FIRA_CONTROLEE_STATE_DELETING;
		} else if (controlee->state == FIRA_CONTROLEE_STATE_RUNNING) {
			/* Stop raised by max number of measurements threshold. */
			if (session->stop_request)
				controlee->state =
					FIRA_CONTROLEE_STATE_STOPPING;
		} else if (controlee->state ==
			   FIRA_CONTROLEE_STATE_PENDING_STOP) {
			controlee->state = FIRA_CONTROLEE_STATE_STOPPING;
		} else if (controlee->state ==
			   FIRA_CONTROLEE_STATE_PENDING_DEL) {
			controlee->state = FIRA_CONTROLEE_STATE_DELETING;
		} else if (controlee->state == FIRA_CONTROLEE_STATE_DELETING) {
			list_del(&controlee->entry);
			kfree(controlee);
			session->n_current_controlees--;
			reset_rx_ctx = true;
		}
	}

	if (reset_rx_ctx && local->llhw->rx_ctx_size) {
		for (i = 0; i < session->n_current_controlees; i++) {
			memset(session->rx_ctx[i], 0, local->llhw->rx_ctx_size);
		}
	}
}

/**
 * check_parameter_proximity_range() - Check proximity range consistency.
 * @params: Current session parameters.
 *
 * Return: 0 or error.
 */
static inline int
check_parameter_proximity_range(const struct fira_session_params *params)
{
	return (params->range_data_ntf_proximity_near_rctu >
		params->range_data_ntf_proximity_far_rctu) ?
		-ERANGE :
		0;
}

/**
 * check_parameter_bound_aoa_azimuth() - Check aoa azimuth range consistency.
 * @params: Current session parameters.
 *
 * Return: 0 or error.
 */
static inline int
check_parameter_bound_aoa_azimuth(const struct fira_session_params *params)
{
        return (params->range_data_ntf_lower_bound_aoa_azimuth_2pi >
                params->range_data_ntf_upper_bound_aoa_azimuth_2pi) ?
                       -ERANGE :
                       0;
}

/**
 * check_parameter_bound_aoa_elevation() - Check aoa elevation range consistency.
 * @params: Current session parameters.
 *
 * Return: 0 or error.
 */
static inline int
check_parameter_bound_aoa_elevation(const struct fira_session_params *params)
{
        return (params->range_data_ntf_lower_bound_aoa_elevation_2pi >
                params->range_data_ntf_upper_bound_aoa_elevation_2pi) ?
                       -ERANGE :
                       0;
}

bool fira_session_is_ready(const struct fira_local *local,
			   const struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	int round_duration_dtu;

	if (params->multi_node_mode == FIRA_MULTI_NODE_MODE_UNICAST) {
		if (session->n_current_controlees > 1)
			return false;
	} else {
		/* On success, session will become active, so assume it is. */
		if (!fira_frame_check_n_controlees(
			    session, session->n_current_controlees, true))
			return false;
	}

	/* Check uwb parameters. */
	if (params->prf_mode == FIRA_PRF_MODE_BPRF) {
		/* FIXME: when preamble code index is not set, we will use
		 * the default set one, that may be for HPRF... */
		if (params->preamble_code_index != 0 &&
		    (params->preamble_code_index < 9 ||
		     params->preamble_code_index > 24))
			return false;
		if (params->sfd_id != FIRA_SFD_ID_0 &&
		    params->sfd_id != FIRA_SFD_ID_2)
			return false;
		if (params->psdu_data_rate != FIRA_PSDU_DATA_RATE_6M81)
			return false;
		if (params->preamble_duration != FIRA_PREAMBULE_DURATION_64)
			return false;
		if (params->number_of_sts_segments > FIRA_STS_SEGMENTS_1)
			return false;
	} else {
		if (params->preamble_code_index != 0 &&
		    (params->preamble_code_index < 25 ||
		     params->preamble_code_index > 32))
			return false;
		if (params->sfd_id == FIRA_SFD_ID_0)
			return false;
		if (params->prf_mode == FIRA_PRF_MODE_HPRF &&
		    params->psdu_data_rate > FIRA_PSDU_DATA_RATE_7M80)
			return false;
		if (params->prf_mode == FIRA_PRF_MODE_HPRF_HIGH_RATE &&
		    params->psdu_data_rate < FIRA_PSDU_DATA_RATE_27M2)
			return false;
	}
	if ((params->rframe_config == FIRA_RFRAME_CONFIG_SP0) &&
	    (params->number_of_sts_segments != FIRA_STS_SEGMENTS_0))
		return false;
	if ((params->rframe_config != FIRA_RFRAME_CONFIG_SP0) &&
	    (params->number_of_sts_segments == FIRA_STS_SEGMENTS_0))
		return false;

	/* Check range data ntf parameters consistency. */
	switch (params->range_data_ntf_config) {
	case FIRA_RANGE_DATA_NTF_PROXIMITY:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_CROSSING:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA_CROSSING:
		if (check_parameter_proximity_range(&session->params))
			return false;
		fallthrough;
	default:
		break;
	}
	switch (params->range_data_ntf_config) {
	case FIRA_RANGE_DATA_NTF_AOA:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA:
	case FIRA_RANGE_DATA_NTF_AOA_CROSSING:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA_CROSSING:
		if (check_parameter_bound_aoa_azimuth(&session->params))
			return false;
		if (check_parameter_bound_aoa_elevation(&session->params))
			return false;
		fallthrough;
	default:
		break;
	}
	round_duration_dtu =
		params->slot_duration_dtu * params->round_duration_slots;
	return params->slot_duration_dtu != 0 &&
	       params->block_duration_dtu != 0 &&
	       params->round_duration_slots != 0 &&
	       round_duration_dtu <= params->block_duration_dtu;
}

/**
 * is_ranging_in_proximity_bounds() - Check if ranging data are in
 * range_data_ntf proximity boundaries.
 * @session: FiRa session.
 * @ranging_data: Ranging data to be evaluated.
 *
 * Return: true if the ranging data are inside proximity boundaries.
 */
static inline bool
is_ranging_in_proximity_bounds(const struct fira_session *session,
			       const struct fira_ranging_info *ranging_data)
{
	return (ranging_data->tof_rctu >=
			session->params.range_data_ntf_proximity_near_rctu &&
		ranging_data->tof_rctu <=
			session->params.range_data_ntf_proximity_far_rctu);
}

/**
 * is_ranging_in_aoa_bounds() - Check if ranging data are in
 * range_data_ntf AoA boundaries.
 * @session: FiRa local session.
 * @ranging_data: Ranging data to be evaluated.
 *
 * Return: true if ranging data are inside AoA boundaries.
 */
static inline bool
is_ranging_in_aoa_bounds(const struct fira_session *session,
			 const struct fira_ranging_info *ranging_data)
{
	if (ranging_data->local_aoa_azimuth.present &&
	    ((ranging_data->local_aoa_azimuth.aoa_2pi <
	      session->params.range_data_ntf_lower_bound_aoa_azimuth_2pi) ||
	     (ranging_data->local_aoa_azimuth.aoa_2pi >
	      session->params.range_data_ntf_upper_bound_aoa_azimuth_2pi)))
		return false;

	if (ranging_data->local_aoa_elevation.present &&
	    ((ranging_data->local_aoa_elevation.aoa_2pi <
	      session->params.range_data_ntf_lower_bound_aoa_elevation_2pi) ||
	     (ranging_data->local_aoa_elevation.aoa_2pi >
	      session->params.range_data_ntf_upper_bound_aoa_elevation_2pi)))
		return false;
	return true;
}

void fira_session_set_range_data_ntf_status(const struct fira_session *session,
					    struct fira_ranging_info *ri)
{
	switch (session->params.range_data_ntf_config) {
	default:
	case FIRA_RANGE_DATA_NTF_DISABLED:
	case FIRA_RANGE_DATA_NTF_ALWAYS:
		ri->range_data_ntf_status = FIRA_RANGE_DATA_NTF_NONE;
		break;
	case FIRA_RANGE_DATA_NTF_PROXIMITY:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_CROSSING:
		if (ri->status != FIRA_STATUS_RANGING_SUCCESS ||
				!ri->tof_present)
			ri->range_data_ntf_status = FIRA_RANGE_DATA_NTF_ERROR;
		else
			ri->range_data_ntf_status =
				is_ranging_in_proximity_bounds(session, ri) ?
					FIRA_RANGE_DATA_NTF_IN :
					FIRA_RANGE_DATA_NTF_OUT;
		break;
	case FIRA_RANGE_DATA_NTF_AOA:
	case FIRA_RANGE_DATA_NTF_AOA_CROSSING:
		if (ri->status != FIRA_STATUS_RANGING_SUCCESS)
			ri->range_data_ntf_status = FIRA_RANGE_DATA_NTF_ERROR;
		else
			ri->range_data_ntf_status =
				is_ranging_in_aoa_bounds(session, ri) ?
					FIRA_RANGE_DATA_NTF_IN :
					FIRA_RANGE_DATA_NTF_OUT;
		break;
	case FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA_CROSSING:
		if (FIRA_STATUS_RANGING_SUCCESS || !ri->tof_present)
			ri->range_data_ntf_status = FIRA_RANGE_DATA_NTF_ERROR;
		else
			ri->range_data_ntf_status =
				is_ranging_in_proximity_bounds(session, ri) &&
						is_ranging_in_aoa_bounds(
							session, ri) ?
					FIRA_RANGE_DATA_NTF_IN :
					FIRA_RANGE_DATA_NTF_OUT;
		break;
	}
}

/**
 * send_ranging_data -- Compute if a device has data to report.
 * @config: range_data_ntf_config session parameter.
 * @ranging_status: Ranging range_data_ntf_status for the device this round.
 * @device_status: Previous device range_data_ntf_status.
 *
 * Return: true if the device has data to report, else false.
 */
static inline bool
send_ranging_data(enum fira_range_data_ntf_config config,
		  enum fira_range_data_ntf_status ranging_status,
		  enum fira_range_data_ntf_status device_status)
{
	switch (config) {
	default:
	case FIRA_RANGE_DATA_NTF_DISABLED:
		return false;
	case FIRA_RANGE_DATA_NTF_ALWAYS:
		return true;
	case FIRA_RANGE_DATA_NTF_PROXIMITY:
	case FIRA_RANGE_DATA_NTF_AOA:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA:
		return (ranging_status == FIRA_RANGE_DATA_NTF_IN) ||
		       (ranging_status == FIRA_RANGE_DATA_NTF_ERROR) ||
		       (ranging_status == FIRA_RANGE_DATA_NTF_IN_ERROR) ||
		       (ranging_status == FIRA_RANGE_DATA_NTF_OUT_ERROR);
	case FIRA_RANGE_DATA_NTF_PROXIMITY_CROSSING:
	case FIRA_RANGE_DATA_NTF_AOA_CROSSING:
	case FIRA_RANGE_DATA_NTF_PROXIMITY_AND_AOA_CROSSING:
		return (((ranging_status == FIRA_RANGE_DATA_NTF_IN) &&
			 ((device_status == FIRA_RANGE_DATA_NTF_OUT) ||
			  (device_status == FIRA_RANGE_DATA_NTF_OUT_ERROR) ||
			  (device_status == FIRA_RANGE_DATA_NTF_IN_ERROR) ||
			  (device_status == FIRA_RANGE_DATA_NTF_ERROR) ||
			  (device_status == FIRA_RANGE_DATA_NTF_NONE))) ||
			((ranging_status == FIRA_RANGE_DATA_NTF_OUT) &&
			 ((device_status == FIRA_RANGE_DATA_NTF_IN) ||
			  (device_status == FIRA_RANGE_DATA_NTF_IN_ERROR))) ||
			((ranging_status == FIRA_RANGE_DATA_NTF_ERROR) &&
			 (device_status == FIRA_RANGE_DATA_NTF_IN)));
	}
}

/**
 * get_range_data_status_update -- Compute new device's range_data_ntf_status,
 * given the ranging status and previous device status.
 * @ranging_status: Ranging range_data_ntf_status for the device this round.
 * @device_status: Previous device range_data_ntf_status.
 *
 * Return: new device range_data_ntf_status.
 */
static inline enum fira_range_data_ntf_status
get_range_data_status_update(enum fira_range_data_ntf_status ranging_status,
			     enum fira_range_data_ntf_status device_status)
{
	enum fira_range_data_ntf_status ret = FIRA_RANGE_DATA_NTF_NONE;
	switch (ranging_status) {
	case FIRA_RANGE_DATA_NTF_NONE:
	case FIRA_RANGE_DATA_NTF_IN:
	case FIRA_RANGE_DATA_NTF_OUT:
		ret = ranging_status;
		break;
	case FIRA_RANGE_DATA_NTF_ERROR:
		switch (device_status) {
		case FIRA_RANGE_DATA_NTF_NONE:
		case FIRA_RANGE_DATA_NTF_ERROR:
			ret = FIRA_RANGE_DATA_NTF_ERROR;
			break;
		case FIRA_RANGE_DATA_NTF_IN:
		case FIRA_RANGE_DATA_NTF_IN_ERROR:
			ret = FIRA_RANGE_DATA_NTF_IN_ERROR;
			break;
		case FIRA_RANGE_DATA_NTF_OUT:
		case FIRA_RANGE_DATA_NTF_OUT_ERROR:
			ret = FIRA_RANGE_DATA_NTF_OUT_ERROR;
			break;
		}
		break;
	/* defensive check, should not happened */
	/* LCOV_EXCL_START */
	case FIRA_RANGE_DATA_NTF_IN_ERROR:
	case FIRA_RANGE_DATA_NTF_OUT_ERROR:
	default:
		ret = FIRA_RANGE_DATA_NTF_NONE;
		break;
	}
	/* LCOV_EXCL_STOP */
	return ret;
}

/**
 * range_data_notif_update -- Update remote device(s) range_data_ntf_status
 * and determine if report is to be sent.
 * @local: FiRa context.
 * @session: FiRa session.
 * @report_info: Report info to be analysed.
 *
 * Return: true if there is actually data to report, else false.
 */
static bool range_data_notif_update(struct fira_local *local,
				    struct fira_session *session,
				    struct fira_report_info *report_info)
{
	bool send_report = false;
	enum fira_range_data_ntf_config config =
		session->params.range_data_ntf_config;

	if (report_info->stopped || report_info->n_stopped_controlees)
		send_report = true;

	if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLLER) {
		int i;

		for (i = 0; i < report_info->n_ranging_data; i++) {
			enum fira_range_data_ntf_status ctlee_status;
			struct fira_ranging_info *ri =
				&report_info->ranging_data[i];
			struct fira_controlee *controlee =
				fira_session_get_controlee(session,
							   ri->short_addr);

			fira_session_set_range_data_ntf_status(session, ri);
			if (!controlee) {
				/*
				 * This case can happen in Contention Based mode.
				 * In this mode, controlees are unknown. Let's notify.
				 */
				ri->notify = true;
				send_report = true;
				continue;
			}
			ctlee_status = controlee->range_data_ntf_status;
			ri->notify = send_ranging_data(
				config, ri->range_data_ntf_status,
				ctlee_status);
			controlee->range_data_ntf_status =
				get_range_data_status_update(
					ri->range_data_ntf_status,
					ctlee_status);
			send_report |= ri->notify;
		}
	} else {
		/* Controlee, so we can assume n_ranging_data == 0 or 1 */
		if (report_info->n_ranging_data) {
			enum fira_range_data_ntf_status ctlr_status =
				session->controlee.ctlr_range_data_ntf_status;
			struct fira_ranging_info *ri =
				&report_info->ranging_data[0];
			fira_session_set_range_data_ntf_status(session, ri);
			ri->notify = send_report = send_ranging_data(
				config, ri->range_data_ntf_status, ctlr_status);
			session->controlee.ctlr_range_data_ntf_status =
				get_range_data_status_update(
					ri->range_data_ntf_status, ctlr_status);
		}
	}
	return send_report;
}

void fira_session_report(struct fira_local *local, struct fira_session *session,
			 struct fira_report_info *report_info)
{
	struct sk_buff *msg;

	if (!range_data_notif_update(local, session, report_info))
		return;

	trace_region_fira_session_report(session, report_info);
	msg = mcps802154_region_event_alloc_skb(local->llhw, &local->region,
						FIRA_CALL_SESSION_NOTIFICATION,
						session->event_portid,
						NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_ID, session->id))
		goto nla_put_failure;
	if (nla_put_u32(msg, FIRA_CALL_ATTR_SEQUENCE_NUMBER,
			session->sequence_number))
		goto nla_put_failure;
	if (fira_session_report_ranging_data(session, report_info,
					     local->llhw->dtu_freq_hz,
					     local->llhw->dtu_rctu, msg))
		goto nla_put_failure;
	if (fira_session_report_ranging_diagnostics(session, report_info, msg))
		goto nla_put_failure;
	session->sequence_number++;
	session->data_payload.sent = false;

	skb_queue_tail(&local->report_queue, msg);
	schedule_work(&local->report_work);
	return;

nla_put_failure:
	kfree_skb(msg);
}
