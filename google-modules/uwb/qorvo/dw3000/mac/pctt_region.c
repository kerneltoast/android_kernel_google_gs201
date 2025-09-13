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

#include <linux/errno.h>

#include <net/pctt_region_nl.h>
#include <net/mcps802154_frame.h>

#include "pctt_trace.h"
#include "pctt_region.h"
#include "pctt_region_call.h"
#include "pctt_access.h"
#include "llhw-ops.h"

static struct mcps802154_region_ops pctt_region_ops;

static struct mcps802154_region *pctt_open(struct mcps802154_llhw *llhw)
{
	struct pctt_local *local;

	local = kzalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;
	local->llhw = llhw;
	local->region.ops = &pctt_region_ops;
	local->session.state = PCTT_SESSION_STATE_DEINIT;

	return &local->region;
}

static void pctt_close(struct mcps802154_region *region)
{
	struct pctt_local *local = region_to_local(region);

	kfree_sensitive(local);
}

static int pctt_call(struct mcps802154_region *region, u32 call_id,
		     const struct nlattr *attrs, const struct genl_info *info)
{
	struct pctt_local *local = region_to_local(region);

	switch (call_id) {
	case PCTT_CALL_SESSION_INIT:
		return pctt_session_init(local);
	case PCTT_CALL_SESSION_DEINIT:
		return pctt_session_deinit(local);
	case PCTT_CALL_SESSION_GET_STATE:
		return pctt_call_session_get_state(local);
	case PCTT_CALL_SESSION_GET_PARAMS:
		return pctt_call_session_get_params(local);
	case PCTT_CALL_SESSION_SET_PARAMS:
	case PCTT_CALL_SESSION_CMD:
		return pctt_call_session_control(local, call_id, attrs, info);
	default:
		return -EINVAL;
	}
}

static int pctt_report_periodic_tx(struct pctt_local *local,
				   struct sk_buff *msg)
{
	trace_region_pctt_report_periodic_tx(local->results.status);

	if (nla_put_u8(msg, PCTT_RESULT_DATA_ATTR_STATUS,
		       local->results.status))
		return -EMSGSIZE;
	return 0;
}

static int pctt_report_per_rx(struct pctt_local *local, struct sk_buff *msg)
{
	const struct pctt_test_per_rx_results *per_rx =
		&local->results.tests.per_rx;

	trace_region_pctt_report_per_rx(local->results.status, per_rx);

#define P(attr, type, value)                                          \
	do {                                                          \
		if (nla_put_##type(msg, PCTT_RESULT_DATA_ATTR_##attr, \
				   value)) {                          \
			goto nla_put_failure;                         \
		}                                                     \
	} while (0)
	P(STATUS, u8, PCTT_STATUS_RANGING_SUCCESS);
	P(ATTEMPTS, u32, per_rx->attempts);
	P(ACQ_DETECT, u32, per_rx->acq_detect);
	P(ACQ_REJECT, u32, per_rx->acq_reject);
	P(RX_FAIL, u32, per_rx->rx_fail);
	P(SYNC_CIR_READY, u32, per_rx->sync_cir_ready);
	P(SFD_FAIL, u32, per_rx->sfd_fail);
	P(SFD_FOUND, u32, per_rx->sfd_found);
	P(PHR_DEC_ERROR, u32, per_rx->phr_dec_error);
	P(PHR_BIT_ERROR, u32, per_rx->phr_bit_error);
	P(PSDU_DEC_ERROR, u32, per_rx->psdu_dec_error);
	P(PSDU_BIT_ERROR, u32, per_rx->psdu_bit_error);
	P(STS_FOUND, u32, per_rx->sts_found);
	P(EOF, u32, per_rx->eof);
	P(RSSI, u8, per_rx->rssi);
#undef P
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int pctt_report_rx(struct pctt_local *local, struct sk_buff *msg)
{
	const struct pctt_test_rx_results *rx = &local->results.tests.rx;

	trace_region_pctt_report_rx(local->results.status, rx);

#define P(attr, type, value)                                          \
	do {                                                          \
		if (nla_put_##type(msg, PCTT_RESULT_DATA_ATTR_##attr, \
				   value)) {                          \
			goto nla_put_failure;                         \
		}                                                     \
	} while (0)
	P(STATUS, u8, local->results.status);
	P(RX_DONE_TS_INT, u32, rx->rx_done_ts_int);
	P(RX_DONE_TS_FRAC, u16, rx->rx_done_ts_frac);
	P(AOA_AZIMUTH, s16, rx->aoa_azimuth);
	P(AOA_ELEVATION, s16, rx->aoa_elevation);
	P(TOA_GAP, u8, rx->toa_gap);
	P(PHR, u16, rx->phr);
	P(RSSI, u8, rx->rssi);
	P(PSDU_DATA_LEN, u16, rx->psdu_data_len);
	if (rx->psdu_data_len > 0 &&
	    nla_put(msg, PCTT_RESULT_DATA_ATTR_PSDU_DATA, rx->psdu_data_len,
		    rx->psdu_data))
		goto nla_put_failure;
#undef P
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int pctt_report_loopback(struct pctt_local *local, struct sk_buff *msg)
{
	struct pctt_session *session = &local->session;
	const struct pctt_session_params *p = &session->params;
	trace_region_pctt_report_loopback(local->results.status);

#define P(attr, type, value)                                          \
	do {                                                          \
		if (nla_put_##type(msg, PCTT_RESULT_DATA_ATTR_##attr, \
				   value)) {                          \
			goto nla_put_failure;                         \
		}                                                     \
	} while (0)

	P(STATUS, u8, local->results.status);
	P(RSSI, u8, local->results.tests.loopback.rssi);
	P(RX_TS_INT, u32, local->results.tests.loopback.rx_ts_int);
	P(RX_TS_FRAC, u16, local->results.tests.loopback.rx_ts_frac);
	P(TX_TS_INT, u32, local->results.tests.loopback.tx_ts_int);
	P(TX_TS_FRAC, u16, local->results.tests.loopback.tx_ts_frac);

	/* If test succeeded, return data that was sent (and received) as
	* PSDU payload. */
	if (!local->results.status) {
		P(PSDU_DATA_LEN, u16, p->data_payload_len);
		if (nla_put(msg, PCTT_RESULT_DATA_ATTR_PSDU_DATA,
			    p->data_payload_len, p->data_payload)) {
			goto nla_put_failure;
		}
	} else {
		P(PSDU_DATA_LEN, u16, 0);
	}
#undef P
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int pctt_report_ss_twr(struct pctt_local *local, struct sk_buff *msg)
{
	const struct pctt_test_ss_twr_results *ss_twr =
		&local->results.tests.ss_twr;

	trace_region_pctt_report_ss_twr(local->results.status, ss_twr);
#define P(attr, type, value)                                          \
	do {                                                          \
		if (nla_put_##type(msg, PCTT_RESULT_DATA_ATTR_##attr, \
				   value)) {                          \
			goto nla_put_failure;                         \
		}                                                     \
	} while (0)
	P(STATUS, u8, local->results.status);
	P(MEASUREMENT, u32, ss_twr->measurement_rctu);
	P(PDOA_AZIMUTH_DEG_Q7, s16, ss_twr->pdoa_azimuth_deg_q7);
	P(PDOA_ELEVATION_DEG_Q7, s16, ss_twr->pdoa_elevation_deg_q7);
	P(AOA_AZIMUTH_DEG_Q7, s16, ss_twr->aoa_azimuth_deg_q7);
	P(AOA_ELEVATION_DEG_Q7, s16, ss_twr->aoa_elevation_deg_q7);
	P(RSSI, u8, ss_twr->rssi);
#undef P
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

void pctt_report(struct pctt_local *local)
{
	struct pctt_session *session = &local->session;
	struct sk_buff *msg;
	struct nlattr *data;

	msg = mcps802154_region_event_alloc_skb(local->llhw, &local->region,
						PCTT_CALL_SESSION_NOTIFICATION,
						session->event_portid,
						NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return;

	if (nla_put_u8(msg, PCTT_CALL_ATTR_CMD_ID, session->cmd_id))
		goto nla_put_failure;

	data = nla_nest_start(msg, PCTT_CALL_ATTR_RESULT_DATA);
	if (!data)
		goto nla_put_failure;

	switch (session->cmd_id) {
	case PCTT_ID_ATTR_PERIODIC_TX:
		if (pctt_report_periodic_tx(local, msg))
			goto nla_put_failure;
		break;
	case PCTT_ID_ATTR_PER_RX:
		if (pctt_report_per_rx(local, msg))
			goto nla_put_failure;
		break;
	case PCTT_ID_ATTR_RX:
		if (pctt_report_rx(local, msg))
			goto nla_put_failure;
		break;
	case PCTT_ID_ATTR_LOOPBACK:
		if (pctt_report_loopback(local, msg))
			goto nla_put_failure;
		break;
	case PCTT_ID_ATTR_SS_TWR:
		if (pctt_report_ss_twr(local, msg))
			goto nla_put_failure;
		break;
	default: /* LCOV_EXCL_START */
		/* Impossible to cover with unit test.
		 * The only way is a memory corruption on the cmd_id. */
		goto nla_put_failure;
		/* LCOV_EXCL_STOP */
	}

	nla_nest_end(msg, data);
	mcps802154_region_event(local->llhw, msg);
	return;

nla_put_failure:
	trace_region_pctt_report_nla_put_failure(session->cmd_id);
	kfree_skb(msg);
}

static struct mcps802154_region_ops pctt_region_ops = {
	/* clang-format off */
	.owner = THIS_MODULE,
	.name = "pctt",
	.open = pctt_open,
	.close = pctt_close,
	.call = pctt_call,
	.get_access = pctt_get_access,
	/* clang-format on */
};

int __init pctt_region_init(void)
{
	return mcps802154_region_register(&pctt_region_ops);
}

void __exit pctt_region_exit(void)
{
	mcps802154_region_unregister(&pctt_region_ops);
}

module_init(pctt_region_init);
module_exit(pctt_region_exit);

MODULE_DESCRIPTION("PCTT Region for IEEE 802.15.4 MCPS");
MODULE_AUTHOR("Clement Calmels <clement.calmels@qorvo.com>");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
