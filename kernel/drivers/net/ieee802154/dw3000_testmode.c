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

#include <linux/slab.h>
#include <net/netlink.h>

#include "dw3000.h"
#include "dw3000_core.h"
#include "dw3000_trc.h"
#include "dw3000_testmode.h"
#include "dw3000_testmode_nl.h"

int set_hrp_uwb_params(struct mcps802154_llhw *llhw, int prf, int psr,
		       int sfd_selector, int phr_rate, int data_rate);
int set_channel(struct mcps802154_llhw *llhw, u8 page, u8 channel,
		u8 preamble_code);

static const struct nla_policy dw3000_tm_policy[DW3000_TM_ATTR_MAX + 1] = {
	[DW3000_TM_ATTR_CMD] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_RX_GOOD_CNT] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_RX_BAD_CNT] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_RSSI_DATA] = { .type = NLA_BINARY,
				       .len = DW3000_TM_RSSI_DATA_MAX_LEN },
	[DW3000_TM_ATTR_OTP_ADDR] = { .type = NLA_U16 },
	[DW3000_TM_ATTR_OTP_VAL] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_OTP_DONE] = { .type = NLA_U8 },
	[DW3000_TM_ATTR_DEEP_SLEEP_DELAY_MS] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CONTTX_FRAME_LENGHT] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CONTTX_RATE] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CONTTX_DURATION] = { .type = NLA_S32 },
	[DW3000_TM_ATTR_PSR] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_SFD] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_PHR_RATE] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_DATA_RATE] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_PAGE] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CHANNEL] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_PREAMBLE_CODE] = { .type = NLA_U32 },
};

struct do_tm_cmd_params {
	struct mcps802154_llhw *llhw;
	struct nlattr **nl_attr;
};

static int do_tm_cmd_start_rx_diag(struct dw3000 *dw, const void *in, void *out)
{
	int rc;
	/**
	 * Since the MCPS enables RX by default on the device at startup before
	 * running any test, enabling it once again manually via testmode
	 * command will cause the device to raise repeated IRQ and flood
	 * this driver's event thread.
	 * As a workaround, we disable RX before performing the testmode
	 * command.
	 */
	rc = dw3000_rx_disable(dw);
	if (rc)
		return rc;
	/* Enable receiver and promiscuous mode */
	rc = dw3000_rx_enable(dw, 0, 0, 0);
	if (rc)
		return rc;
	rc = dw3000_set_promiscuous(dw, true);
	if (rc)
		return rc;
	/* Enable statistics */
	return dw3000_rx_stats_enable(dw, true);
}

static int do_tm_cmd_stop_rx_diag(struct dw3000 *dw, const void *in, void *out)
{
	int rc;
	/* Disable receiver and promiscuous mode */
	rc = dw3000_rx_disable(dw);
	if (rc)
		return rc;
	rc = dw3000_set_promiscuous(dw, false);
	if (rc)
		return rc;
	/* Disable statistics */
	return dw3000_rx_stats_enable(dw, false);
}

static int do_tm_cmd_get_rx_diag(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	struct dw3000_stats *stats = &dw->stats;
	struct sk_buff *nl_skb;
	size_t rssi_len;
	int count = stats->count[DW3000_STATS_RX_GOOD];
	int rc;

	/* TODO: we don't send RSSI data for error frames. We should change this. */
	rssi_len = count < (DW3000_RSSI_REPORTS_MAX << 1) ?
			   count :
			   DW3000_RSSI_REPORTS_MAX << 1;
	rssi_len *= sizeof(struct dw3000_rssi);

	/**
	 * Allocate netlink message. The approximated size includes
	 * the testmode's command id and data.
	 */
	nl_skb = mcps802154_testmode_alloc_reply_skb(
		params->llhw, 3 * sizeof(u32) + rssi_len);
	if (!nl_skb) {
		dev_err(dw->dev, "failed to alloc skb reply\n");
		return -ENOMEM;
	}
	/* Append good and bad RX counters to the netlink message */
	rc = nla_put_u32(nl_skb, DW3000_TM_ATTR_RX_GOOD_CNT,
			 stats->count[DW3000_STATS_RX_GOOD]);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode rx counter: %d\n", rc);
		goto nla_put_failure;
	}
	rc = nla_put_u32(nl_skb, DW3000_TM_ATTR_RX_BAD_CNT,
			 stats->count[DW3000_STATS_RX_ERROR] +
				 stats->count[DW3000_STATS_RX_TO]);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode rx counter: %d\n", rc);
		goto nla_put_failure;
	}
	/* Append RSSI data to the netlink message */
	rc = nla_put(nl_skb, DW3000_TM_ATTR_RSSI_DATA, rssi_len, stats->rssi);
	if (rc) {
		dev_err(dw->dev, "failed to copy testmode data: %d\n", rc);
		goto nla_put_failure;
	}
	return mcps802154_testmode_reply(params->llhw, nl_skb);

nla_put_failure:
	nlmsg_free(nl_skb);
	return rc;
}

static int do_tm_cmd_clear_rx_diag(struct dw3000 *dw, const void *in, void *out)
{
	/* Clear statistics */
	dw3000_rx_stats_clear(dw);
	return 0;
}

static int do_tm_cmd_start_tx_cwtone(struct dw3000 *dw, const void *in,
				     void *out)
{
	int rc;
	/* Disable receiver */
	rc = dw3000_rx_disable(dw);
	if (rc)
		return rc;
	/* Play repeated CW tone */
	return dw3000_tx_setcwtone(dw, true);
}

static int do_tm_cmd_stop_tx_cwtone(struct dw3000 *dw, const void *in,
				    void *out)
{
	/* Stop repeated CW tone */
	return dw3000_tx_setcwtone(dw, false);
}

static int do_tm_cmd_otp_read(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	struct sk_buff *msg;
	u32 otp_val;
	u16 otp_addr;
	int rc;

	/* Verify the OTP address attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_OTP_ADDR])
		return -EINVAL;

	otp_addr = nla_get_u16(params->nl_attr[DW3000_TM_ATTR_OTP_ADDR]);
	/* Verify if the given OTP address exceeds the limit */
	if (otp_addr > DW3000_OTP_ADDRESS_LIMIT)
		return -EINVAL;
	/* Read at OTP address */
	rc = dw3000_otp_read32(dw, otp_addr, &otp_val);
	if (rc)
		return rc;
	/**
	 * Allocate netlink message. The approximated size includes
	 * the testmode's command id and data.
	 */
	msg = mcps802154_testmode_alloc_reply_skb(params->llhw,
						  2 * sizeof(u32));
	if (!msg) {
		dev_err(dw->dev, "failed to alloc skb reply\n");
		return -ENOMEM;
	}
	/* Append OTP memory's value to the netlink message */
	rc = nla_put_u32(msg, DW3000_TM_ATTR_OTP_VAL, otp_val);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode otp val: %d\n", rc);
		goto nla_put_failure;
	}
	return mcps802154_testmode_reply(params->llhw, msg);

nla_put_failure:
	nlmsg_free(msg);
	return rc;
}

static int do_tm_cmd_otp_write(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	struct sk_buff *msg;
	u32 otp_val;
	u16 otp_addr;
	u8 otp_done;
	int rc;

	/* Verify the OTP address attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_OTP_ADDR] ||
	    !params->nl_attr[DW3000_TM_ATTR_OTP_VAL])
		return -EINVAL;

	otp_addr = nla_get_u16(params->nl_attr[DW3000_TM_ATTR_OTP_ADDR]);
	/* Verify if the given OTP address exceeds the limit */
	if (otp_addr > DW3000_OTP_ADDRESS_LIMIT)
		return -EINVAL;
	otp_val = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_OTP_VAL]);
	/* Write at OTP address */
	rc = dw3000_otp_write32(dw, otp_addr, otp_val);
	otp_done = (rc) ? 0 : 1;
	/**
	 * Allocate netlink message. The approximated size includes
	 * the testmode's command id and data.
	 */
	msg = mcps802154_testmode_alloc_reply_skb(params->llhw,
						  2 * sizeof(u32));
	if (!msg) {
		dev_err(dw->dev, "failed to alloc skb reply\n");
		return -ENOMEM;
	}
	/* Append OTP memory's value to the netlink message */
	rc = nla_put_u8(msg, DW3000_TM_ATTR_OTP_DONE, otp_done);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode otp done: %d\n", rc);
		goto nla_put_failure;
	}
	return mcps802154_testmode_reply(params->llhw, msg);

nla_put_failure:
	nlmsg_free(msg);
	return rc;
}

static int do_tm_cmd_deep_sleep(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u32 delay;

	/* Verify the delay attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_DEEP_SLEEP_DELAY_MS])
		return -EINVAL;
	delay = nla_get_u32(
		params->nl_attr[DW3000_TM_ATTR_DEEP_SLEEP_DELAY_MS]);
	dw->deep_sleep_state.next_operational_state = DW3000_OP_STATE_IDLE_PLL;
	return dw3000_deep_sleep_and_wakeup(dw, delay * 1000);
}

static int do_tm_cmd_start_cont_tx(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u32 frame_length;
	u32 rate;
	s32 duration;
	int rc;

	/* Verify mandatory attributes */
	if (!params->nl_attr[DW3000_TM_ATTR_CONTTX_FRAME_LENGHT] ||
	    !params->nl_attr[DW3000_TM_ATTR_CONTTX_RATE])
		return -EINVAL;

	frame_length = nla_get_u32(
		params->nl_attr[DW3000_TM_ATTR_CONTTX_FRAME_LENGHT]);
	if (frame_length < 4)
		return -EINVAL;
	rate = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CONTTX_RATE]);

	/* Disable receiver */
	rc = dw3000_rx_disable(dw);
	if (rc)
		return rc;

	dw->config.stsMode = DW3000_STS_MODE_OFF;

	rc = dw3000_testmode_continuous_tx_start(dw, frame_length, rate);
	if (rc)
		return rc;

	if (params->nl_attr[DW3000_TM_ATTR_CONTTX_DURATION]) {
		duration = nla_get_s32(
			params->nl_attr[DW3000_TM_ATTR_CONTTX_DURATION]);
		msleep(duration * 1000);
		rc = dw3000_testmode_continuous_tx_stop(dw);
	}

	return rc;
}

static int do_tm_cmd_stop_cont_tx(struct dw3000 *dw, const void *in, void *out)
{
	return dw3000_testmode_continuous_tx_stop(dw);
}

static int do_tm_cmd_set_hrp_uwb_params(struct dw3000 *dw, const void *in,
					void *out)
{
	const struct do_tm_cmd_params *params = in;
	u32 psr;
	u32 sfd;
	u32 phr_rate;
	u32 data_rate;

	/* Verify the psr attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_PSR])
		return -EINVAL;
	psr = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_PSR]);

	/* Verify the sfd attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_SFD])
		return -EINVAL;
	sfd = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_SFD]);

	/* Verify the phr_rate attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_PHR_RATE])
		return -EINVAL;
	phr_rate = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_PHR_RATE]);

	/* Verify the data_rate attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_DATA_RATE])
		return -EINVAL;
	data_rate = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_DATA_RATE]);

	return set_hrp_uwb_params(params->llhw, 0, psr, sfd, phr_rate,
				  data_rate);
}

static int do_tm_cmd_set_channel(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u32 page;
	u32 channel;
	u32 preamble_code;

	/* Verify the page attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_PAGE])
		return -EINVAL;
	page = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_PAGE]);

	/* Verify the channel attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CHANNEL])
		return -EINVAL;
	channel = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CHANNEL]);

	/* Verify the preamble_code attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_PREAMBLE_CODE])
		return -EINVAL;
	preamble_code =
		nla_get_u32(params->nl_attr[DW3000_TM_ATTR_PREAMBLE_CODE]);

	return set_channel(params->llhw, page, channel, preamble_code);
}

int dw3000_tm_cmd(struct mcps802154_llhw *llhw, void *data, int len)
{
	struct dw3000 *dw = llhw->priv;
	struct do_tm_cmd_params params;
	struct dw3000_stm_command cmd = { NULL, &params, NULL };
	struct nlattr *attr[DW3000_TM_ATTR_MAX + 1];

	static const cmd_func cmds[__DW3000_TM_CMD_AFTER_LAST] = {
		[DW3000_TM_CMD_START_RX_DIAG] = do_tm_cmd_start_rx_diag,
		[DW3000_TM_CMD_STOP_RX_DIAG] = do_tm_cmd_stop_rx_diag,
		[DW3000_TM_CMD_GET_RX_DIAG] = do_tm_cmd_get_rx_diag,
		[DW3000_TM_CMD_CLEAR_RX_DIAG] = do_tm_cmd_clear_rx_diag,
		[DW3000_TM_CMD_OTP_READ] = do_tm_cmd_otp_read,
		[DW3000_TM_CMD_OTP_WRITE] = do_tm_cmd_otp_write,
		[DW3000_TM_CMD_START_TX_CWTONE] = do_tm_cmd_start_tx_cwtone,
		[DW3000_TM_CMD_STOP_TX_CWTONE] = do_tm_cmd_stop_tx_cwtone,
		[DW3000_TM_CMD_START_CONTINUOUS_TX] = do_tm_cmd_start_cont_tx,
		[DW3000_TM_CMD_STOP_CONTINUOUS_TX] = do_tm_cmd_stop_cont_tx,
		[DW3000_TM_CMD_DEEP_SLEEP] = do_tm_cmd_deep_sleep,
		[DW3000_TM_CMD_SET_HRP_PARAMS] = do_tm_cmd_set_hrp_uwb_params,
		[DW3000_TM_CMD_SET_CHANNEL] = do_tm_cmd_set_channel,
	};
	u32 tm_cmd;
	int ret;

	ret = nla_parse(attr, DW3000_TM_ATTR_MAX, data, len, dw3000_tm_policy,
			NULL);

	if (ret)
		return ret;

	if (!attr[DW3000_TM_ATTR_CMD])
		return -EINVAL;

	params = (struct do_tm_cmd_params){ llhw, attr };

	tm_cmd = nla_get_u32(attr[DW3000_TM_ATTR_CMD]);
	/* Share the testmode's command with each thread-safe function */
	trace_dw3000_tm_cmd(dw, tm_cmd);

	if (tm_cmd < __DW3000_TM_CMD_AFTER_LAST && cmds[tm_cmd]) {
		cmd.cmd = cmds[tm_cmd];
		ret = dw3000_enqueue_generic(dw, &cmd);
	} else
		ret = -EOPNOTSUPP;

	trace_dw3000_return_int(dw, ret);
	return ret;
}
