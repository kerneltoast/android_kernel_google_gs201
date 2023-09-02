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
#include "dw3000_nfcc_coex_core.h"
#include "dw3000_nfcc_coex_buffer.h"
#include "dw3000_nfcc_coex_msg.h"
#include "dw3000_nfcc_coex.h"
#include "dw3000_core.h"
#include "dw3000_trc.h"
#include "dw3000_mcps.h"

#include <linux/module.h>

/* dw3000_nfcc_coex_margin_dtu:
 * - Can't be bigger than ANTICIP_DTU (trouble with CLOCK_SYNC).
 * - Lower than 4ms is really dangerous. */
unsigned dw3000_nfcc_coex_margin_dtu = US_TO_DTU(16000);
module_param_named(nfcc_coex_margin_dtu, dw3000_nfcc_coex_margin_dtu, uint,
		   0444);
MODULE_PARM_DESC(
	nfcc_coex_margin_dtu,
	"Margin in dtu needed to give access to the NFCC controller and for the NFCC controller"
	" to wake up (default is anticip_dtu)");

/**
 * dw3000_nfcc_coex_enable_SPIxMAVAIL_interrupts() - Enable all SPI available interrupts.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_enable_SPIxMAVAIL_interrupts(struct dw3000 *dw)
{
	int rc;
	u8 reg;

	/* Clear SPI1MAVAIL and SPI2MAVAIL interrupt status. */
	rc = dw3000_clear_dss_status(
		dw, DW3000_DSS_STAT_SPI1_AVAIL_BIT_MASK |
			    DW3000_DSS_STAT_SPI2_AVAIL_BIT_MASK);
	if (rc)
		return rc;
	/* Disable SPIRDY in SYS_MASK. If it is enabled, the IRQ2 will not work.
	 * It is an undocumented feature.
	 */
	rc = dw3000_reg_modify32(
		dw, DW3000_SYS_ENABLE_LO_ID, 0,
		(u32)~DW3000_SYS_ENABLE_LO_SPIRDY_ENABLE_BIT_MASK, 0);
	if (rc)
		return rc;
	/* Enable the dual SPI interrupt for SPI */
	rc = dw3000_reg_modify32(dw, DW3000_SYS_CFG_ID, 0, U32_MAX,
				 (u32)DW3000_SYS_CFG_DS_IE2_BIT_MASK);
	if (rc)
		return rc;
	/* The masked write transactions do not work on the SPI_SEM register.
	 * So, a read, modify, write sequence is mandatory on this register.
	 *
	 * The 16 bits SPI_SEM register can be accessed as two 8 bits registers.
	 * So, only read the upper 8 bits for performance.
	 */
	rc = dw3000_reg_read8(dw, DW3000_SPI_SEM_ID, 1, &reg);
	if (rc)
		return rc;
	/* Set SPI1MAVAIL and SPI2MAVAIL masks */
	reg |= (DW3000_SPI_SEM_SPI1MAVAIL_BIT_MASK >> 8) |
	       (DW3000_SPI_SEM_SPI2MAVAIL_BIT_MASK >> 8);
	return dw3000_reg_write8(dw, DW3000_SPI_SEM_ID, 1, reg);
}

/**
 * dw3000_nfcc_coex_disable_SPIxMAVAIL_interrupts() - Disable all SPI available interrupts.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_disable_SPIxMAVAIL_interrupts(struct dw3000 *dw)
{
	int rc;
	u8 reg;

	/* Please read the comment in enable_SPIxMAVAIL_interrupts() for SPI_SEM access. */
	rc = dw3000_reg_read8(dw, DW3000_SPI_SEM_ID, 1, &reg);
	if (rc)
		return rc;
	/* Reset SPI1MAVAIL and SPI2MAVAIL masks. */
	reg &= ~(DW3000_SPI_SEM_SPI1MAVAIL_BIT_MASK >> 8) &
	       ~(DW3000_SPI_SEM_SPI2MAVAIL_BIT_MASK >> 8);
	rc = dw3000_reg_write8(dw, DW3000_SPI_SEM_ID, 1, reg);
	if (rc)
		return rc;
	/* Disable the dual SPI interrupt for SPI. */
	return dw3000_reg_modify32(dw, DW3000_SYS_CFG_ID, 0,
				   (u32)~DW3000_SYS_CFG_DS_IE2_BIT_MASK, 0);
}

/**
 * dw3000_nfcc_coex_update_access_info() - Update access info cache.
 * @dw: Driver context.
 * @buffer: buffer to read.
 */
static void dw3000_nfcc_coex_update_access_info(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer)
{
	struct llhw_vendor_cmd_nfcc_coex_get_access_info *access_info =
		&dw->nfcc_coex.access_info;
	struct dw3000_nfcc_coex_rx_msg_info rx_msg_info = {};
	int r;

	r = dw3000_nfcc_coex_message_check(dw, buffer, &rx_msg_info);
	if (r) {
		trace_dw3000_nfcc_coex_warn(dw, "message check failure");
		goto failure;
	}

	/* Update access_info. */
	access_info->stop = !rx_msg_info.next_slot_found;
	access_info->watchdog_timeout = false;
	if (rx_msg_info.next_slot_found) {
		/* Request the handle earlier to the mac layer. */
		access_info->next_timestamp_dtu =
			rx_msg_info.next_timestamp_dtu -
			dw3000_nfcc_coex_margin_dtu;
		access_info->next_duration_dtu = rx_msg_info.next_duration_dtu +
						 dw3000_nfcc_coex_margin_dtu;
	}
	return;

failure:
	/* When buffer content is unexpected then request a stop. */
	memset(access_info, 0, sizeof(*access_info));
	access_info->stop = true;
}

/**
 * dw3000_nfcc_coex_configure() - Configure the nfcc_coex.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_configure(struct dw3000 *dw)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;
	bool channel_changed = nfcc_coex->original_channel != dw->config.chan;
	int r;

	if (nfcc_coex->configured)
		return 0;

	trace_dw3000_nfcc_coex_configure(dw);
	if (channel_changed) {
		r = dw3000_configure_chan(dw);
		if (r) {
			trace_dw3000_nfcc_coex_err(dw,
						   "configure channel fails");
			return r;
		}
	}
	r = dw3000_nfcc_coex_prepare_config(dw);
	if (r) {
		trace_dw3000_nfcc_coex_warn(dw,
					    "prepare the configuration fails");
		return r;
	}

	r = dw3000_nfcc_coex_enable_SPIxMAVAIL_interrupts(dw);
	if (r) {
		trace_dw3000_nfcc_coex_err(
			dw, "SPIxMAVAIL interrupts enable failed");
		return r;
	}

	nfcc_coex->configured = true;
	return 0;
}

/**
 * dw3000_nfcc_coex_do_watchdog_timeout() - Do watchdog timeout event in workqueue.
 * @dw: Driver context.
 * @in: Data to read.
 * @out: Data to write.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_do_watchdog_timeout(struct dw3000 *dw,
						const void *in, void *out)
{
	/* Update status for GET_ACCESS_INFORMATION vendor. */
	dw->nfcc_coex.access_info.watchdog_timeout = true;
	/* Do same as dw3000_nfcc_coex_disable function without decawave
	 * register access which it probably locked by NFCC. */
	dw->nfcc_coex.enabled = false;
	dw->config.chan = dw->nfcc_coex.original_channel;
	mcps802154_broken(dw->llhw);

	return 0;
}

/**
 * dw3000_nfcc_coex_do_spi1_avail() - Do SPI1 available irq event in workqueue.
 * @dw: Driver context.
 * @in: Data to read.
 * @out: Data to write.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_do_spi1_avail(struct dw3000 *dw, const void *in,
					  void *out)
{
	struct dw3000_nfcc_coex_buffer buffer;
	int r;

	/* Is watchdog timer running? */
	r = dw3000_nfcc_coex_cancel_watchdog(dw);
	if (r) {
		trace_dw3000_nfcc_coex_warn(
			dw, "spi available without timer pending");
		goto spi1_avail_failure;
	}

	r = dw3000_nfcc_coex_read_buffer(dw, &buffer,
					 DW3000_NFCC_COEX_MSG_IN_SIZE);
	if (r)
		goto spi1_avail_failure;

	dw3000_nfcc_coex_update_access_info(dw, &buffer);
	dw3000_nfcc_coex_disable(dw);
	mcps802154_tx_done(dw->llhw);
	return 0;

spi1_avail_failure:
	dw3000_nfcc_coex_disable(dw);
	mcps802154_broken(dw->llhw);
	return r;
}

/**
 * dw3000_nfcc_coex_watchdog_timeout() - Watchdog timeout event.
 * @timer: Timer context.
 */
static void dw3000_nfcc_coex_watchdog_timeout(struct timer_list *timer)
{
	struct dw3000_nfcc_coex *nfcc_coex =
		from_timer(nfcc_coex, timer, watchdog_timer);
	struct dw3000 *dw = nfcc_coex_to_dw(nfcc_coex);
	struct dw3000_stm_command cmd = { dw3000_nfcc_coex_do_watchdog_timeout,
					  NULL, NULL };

	trace_dw3000_nfcc_coex_watchdog_timeout(dw);
	dw3000_enqueue_timer(dw, &cmd);
}

/**
 * dw3000_nfcc_coex_cancel_watchdog() - Cancel the watchdog timer.
 * @dw: Driver context.
 *
 * Return: 0 on success, otherwise -errno on error.
 */
int dw3000_nfcc_coex_cancel_watchdog(struct dw3000 *dw)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;

	trace_dw3000_nfcc_coex_cancel_watchdog(dw);
	/* Is watchdog timer running? */
	if (!del_timer(&nfcc_coex->watchdog_timer))
		return -ENOENT;
	return 0;
}

/**
 * dw3000_nfcc_coex_spi1_avail() - Handle SPI1 available interrupt.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_spi1_avail(struct dw3000 *dw)
{
	struct dw3000_stm_command cmd = { dw3000_nfcc_coex_do_spi1_avail, NULL,
					  NULL };

	trace_dw3000_nfcc_coex_spi1_avail(dw);
	return dw3000_enqueue_generic(dw, &cmd);
}

/**
 * dw3000_nfcc_coex_idle_timeout() - Idle expired.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_idle_timeout(struct dw3000 *dw)
{
	int r;

	trace_dw3000_nfcc_coex_idle_timeout(dw);
	r = dw3000_nfcc_coex_configure(dw);
	if (r)
		goto idle_timeout_failure;
	r = dw3000_nfcc_coex_message_send(dw);
	if (r)
		goto idle_timeout_failure;
	return 0;

idle_timeout_failure:
	dw3000_nfcc_coex_disable(dw);
	mcps802154_broken(dw->llhw);
	return r;
}

/**
 * dw3000_nfcc_coex_init() - Initialize NFCC coexistence context.
 * @dw: Driver context.
 */
void dw3000_nfcc_coex_init(struct dw3000 *dw)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;

	memset(nfcc_coex, 0, sizeof(*nfcc_coex));
	timer_setup(&nfcc_coex->watchdog_timer,
		    dw3000_nfcc_coex_watchdog_timeout, 0);
}

/**
 * dw3000_nfcc_coex_enable() - Enable NFCC coexistence.
 * @dw: Driver context.
 * @channel: Channel number (5 or 9).
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_enable(struct dw3000 *dw, u8 channel)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;

	trace_dw3000_nfcc_coex_enable(dw, channel);
	if (nfcc_coex->enabled)
		return -EBUSY;

	/* Save current channel. */
	nfcc_coex->original_channel = dw->config.chan;
	nfcc_coex->configured = false;
	nfcc_coex->enabled = true;
	/* Set the new channel. */
	dw->config.chan = channel;
	return 0;
}

/**
 * dw3000_nfcc_coex_disable() - Disable NFCC coexistence.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_disable(struct dw3000 *dw)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;
	bool channel_changed = nfcc_coex->original_channel != dw->config.chan;
	int r;

	trace_dw3000_nfcc_coex_disable(dw);
	if (!dw->nfcc_coex.enabled)
		return 0;

	dw->config.chan = dw->nfcc_coex.original_channel;
	dw->nfcc_coex.enabled = false;

	if (dw->nfcc_coex.configured) {
		r = dw3000_nfcc_coex_disable_SPIxMAVAIL_interrupts(dw);
		if (r)
			return r;
		if (channel_changed) {
			r = dw3000_configure_chan(dw);
			if (r)
				return r;
		}
		r = dw3000_nfcc_coex_restore_config(dw);
		if (r)
			return r;
	}
	return 0;
}
