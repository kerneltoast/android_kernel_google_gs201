/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
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
#include "dw3000_nfcc_coex_mcps.h"
#include "dw3000_nfcc_coex_msg.h"
#include "dw3000_nfcc_coex_buffer.h"
#include "dw3000_nfcc_coex_core.h"
#include "dw3000.h"
#include "dw3000_trc.h"
#include "dw3000_core.h"

#define DW3000_NFCC_COEX_WATCHDOG_DEFAULT_DURATION_MS 24000

static int dw3000_nfcc_coex_wakeup_and_send(struct dw3000 *dw,
					    s32 idle_duration_dtu,
					    u32 send_timestamp_dtu)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;
	int r;

	trace_dw3000_nfcc_coex_wakeup_and_send(
		dw, nfcc_coex->send, idle_duration_dtu, send_timestamp_dtu);

	if (idle_duration_dtu > dw->llhw->anticip_dtu) {
		r = dw3000_idle(dw, true, send_timestamp_dtu,
				dw3000_nfcc_coex_idle_timeout,
				DW3000_OP_STATE_MAX);
		goto wakeup_and_send_end;
	} else if (dw->current_operational_state ==
		   DW3000_OP_STATE_DEEP_SLEEP) {
		r = dw3000_deepsleep_wakeup_now(dw,
						dw3000_nfcc_coex_idle_timeout,
						send_timestamp_dtu,
						DW3000_OP_STATE_MAX);
		goto wakeup_and_send_end;
	}

	r = dw3000_nfcc_coex_configure(dw);
	if (r)
		goto wakeup_and_send_end;
	r = dw3000_nfcc_coex_message_send(dw);
	if (r)
		goto wakeup_and_send_end;
	return 0;

wakeup_and_send_end:
	if (r)
		dw3000_nfcc_coex_disable(dw);
	return r;
}

/**
 * dw3000_nfcc_coex_handle_access() - handle access to provide to NFCC.
 * @dw: Driver context.
 * @data: Address of handle access information.
 * @data_len: Number of byte of the data object.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_handle_access(struct dw3000 *dw, void *data,
					  int data_len)
{
	const struct llhw_vendor_cmd_nfcc_coex_handle_access *info = data;
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;
	const u32 dtu_per_ms = dw->llhw->dtu_freq_hz / 1000;
	u32 now_dtu;
	s32 idle_duration_dtu;
	int r;

	if (!info || data_len != sizeof(*info))
		return -EINVAL;

	if (timer_pending(&nfcc_coex->watchdog_timer)) {
		trace_dw3000_nfcc_coex_err(dw, "watchdog timer is pending");
		return -EBUSY;
	}

	r = dw3000_nfcc_coex_enable(dw, info->chan);
	if (r)
		return r;

	now_dtu = dw3000_get_dtu_time(dw);
	idle_duration_dtu = info->timestamp_dtu - now_dtu;

	trace_dw3000_nfcc_coex_handle_access(dw, info, idle_duration_dtu);
	nfcc_coex->send = info->start ? DW3000_NFCC_COEX_SEND_CLK_SYNC :
					DW3000_NFCC_COEX_SEND_CLK_OFFSET;
	if (info->start) {
		nfcc_coex->version = info->version;
		/*
		 * Save first start session date, to retrieve MSB bits lost
		 * for next received timestamp through session_time0_dtu.
		 * It's saved because between session_time0_dtu and
		 * access_start_dtu, a deep sleep can occur, and so
		 * `dtu_to_sys_time` must be call after the wake up.
		 * Between access_start_dtu and now, the duration can be less
		 * than 2 ms (fira slot) in multi-region feature.
		 * So the margin is like a second anticip dtu to add to provide
		 * time for NFC handling.
		 */
		nfcc_coex->access_start_dtu =
			info->timestamp_dtu + dw3000_nfcc_coex_margin_dtu;
	}
	nfcc_coex->watchdog_timer.expires =
		jiffies +
		msecs_to_jiffies((info->timestamp_dtu - now_dtu) / dtu_per_ms +
				 DW3000_NFCC_COEX_WATCHDOG_DEFAULT_DURATION_MS);
	add_timer(&nfcc_coex->watchdog_timer);

	/* Send message and so release the SPI close to the nfc_coex_margin. */
	return dw3000_nfcc_coex_wakeup_and_send(dw, idle_duration_dtu,
						info->timestamp_dtu);
}

/**
 * dw3000_nfcc_coex_get_access_information() - Forward access info cached.
 * @dw: Driver context.
 * @data: Address where to write access information.
 * @data_len: Number of byte of the data object.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_get_access_information(struct dw3000 *dw,
						   void *data, int data_len)
{
	const struct llhw_vendor_cmd_nfcc_coex_get_access_info *access_info =
		&dw->nfcc_coex.access_info;

	if (!data || data_len != sizeof(*access_info))
		return -EINVAL;

	memcpy(data, access_info, data_len);
	return 0;
}

/**
 * dw3000_nfcc_coex_stop() - Stop NFCC.
 * @dw: Driver context.
 * @data: Address of stop information.
 * @data_len: Number of byte of the data object.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_stop(struct dw3000 *dw, void *data, int data_len)
{
	const struct llhw_vendor_cmd_nfcc_coex_stop *info = data;
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;
	const u32 dtu_per_ms = dw->llhw->dtu_freq_hz / 1000;
	u32 now_dtu, send_timestamp_dtu;
	s32 idle_duration_dtu;
	int r;

	if (data_len && data_len != sizeof(*info))
		return -EINVAL;

	if (timer_pending(&nfcc_coex->watchdog_timer)) {
		trace_dw3000_nfcc_coex_err(dw, "watchdog timer is pending");
		return -EBUSY;
	}

	r = dw3000_nfcc_coex_enable(dw, dw->config.chan);
	if (r)
		return r;

	nfcc_coex->send = DW3000_NFCC_COEX_SEND_STOP;

	if (info) {
		now_dtu = dw3000_get_dtu_time(dw);
		send_timestamp_dtu = info->timestamp_dtu;
		idle_duration_dtu = send_timestamp_dtu - now_dtu;
		nfcc_coex->watchdog_timer.expires =
			jiffies +
			msecs_to_jiffies(
				(info->timestamp_dtu - now_dtu) / dtu_per_ms +
				DW3000_NFCC_COEX_WATCHDOG_DEFAULT_DURATION_MS);
		nfcc_coex->version = info->version;
	} else {
		send_timestamp_dtu = 0;
		idle_duration_dtu = 0;
		nfcc_coex->watchdog_timer.expires =
			jiffies +
			msecs_to_jiffies(
				DW3000_NFCC_COEX_WATCHDOG_DEFAULT_DURATION_MS);
		/* Cancel wakeup timer launch by idle() */
		dw3000_idle_cancel_timer(dw);
	}

	add_timer(&nfcc_coex->watchdog_timer);

	/* Send message and so release the SPI close to the nfc_coex_margin. */
	return dw3000_nfcc_coex_wakeup_and_send(dw, idle_duration_dtu,
						send_timestamp_dtu);
}

/**
 * dw3000_nfcc_coex_vendor_cmd() - Vendor NFCC coexistence command processing.
 *
 * @dw: Driver context.
 * @vendor_id: Vendor Identifier on 3 bytes.
 * @subcmd: Sub-command identifier.
 * @data: Null or data related with the sub-command.
 * @data_len: Length of the data
 *
 * Return: 0 on success, 1 to request a stop, error on other value.
 */
int dw3000_nfcc_coex_vendor_cmd(struct dw3000 *dw, u32 vendor_id, u32 subcmd,
				void *data, size_t data_len)
{
	/* NFCC needs a D0 chip or above. C0 does not have 2 SPI interfaces. */
	if (__dw3000_chip_version == DW3000_C0_VERSION)
		return -EOPNOTSUPP;

	switch (subcmd) {
	case LLHW_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS:
		return dw3000_nfcc_coex_handle_access(dw, data, data_len);
	case LLHW_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION:
		return dw3000_nfcc_coex_get_access_information(dw, data,
							       data_len);
	case LLHW_VENDOR_CMD_NFCC_COEX_STOP:
		return dw3000_nfcc_coex_stop(dw, data, data_len);
	default:
		return -EINVAL;
	}
}
