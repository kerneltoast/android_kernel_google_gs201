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
#ifndef __DW3000_COEX_H
#define __DW3000_COEX_H

#include "dw3000.h"

static inline int dw3000_coex_stop(struct dw3000 *dw);

/**
 * dw3000_coex_gpio - change the state of the GPIO used for WiFi coexistence
 * @dw: the DW device
 * @state: the new GPIO state to set
 * @delay_us: delay before toggling the GPIO.
 *
 * This function only call the version dependent coex_gpio function.
 *
 * It cannot be called if dw3000_coex_init() has fail or if no coex gpio
 * is defined. So no need to test chip_ops.
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_coex_gpio(struct dw3000 *dw, int state, int delay_us)
{
	int rc;

	/* Use SPI in queuing mode */
	dw3000_spi_queue_start(dw);
	rc = dw->chip_ops->coex_gpio(dw, state, delay_us);
	if (rc)
		return dw3000_spi_queue_reset(dw, rc);
	rc = dw3000_spi_queue_flush(dw);
	if (!rc)
		dw->coex_status = state;
	return rc;
}

/**
 * dw3000_coex_start - Handle WiFi coex gpio at start of uwb exchange.
 * @dw: the DW device
 * @trx_delayed: pointer to tx/rx_delayed parameter to update
 * @trx_date_dtu: pointer to tx/rx_date_dtu parameter to update
 * @cur_time_dtu: current device time in DTU
 *
 * Calculate a timer delay in us and programme it to ensure the WiFi coex
 * GPIO is asserted at least `coex_delay_us` before the next operation.
 *
 * This function also reset the GPIO immediatly if the calculated timer delay
 * is more than `coex_interval_us`.
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_coex_start(struct dw3000 *dw, bool *trx_delayed,
				    u32 *trx_date_dtu, u32 cur_time_dtu)
{
	int delay_us;
	int timer_us = 0;

	if (dw->coex_gpio < 0 || !dw->coex_enabled)
		return 0;
	/* Add a margin for required SPI transactions to the coex delay time
	 * to ensure GPIO change at right time. */
	delay_us = dw->coex_delay_us + dw->coex_margin_us;
	if (*trx_delayed == false) {
		/* Change to delayed TX/RX with the minimum required delay */
		*trx_date_dtu = cur_time_dtu + US_TO_DTU(delay_us);
		*trx_delayed = true;
		/* Leave timer_us to 0 to set gpio now. */
	} else {
		/* Calculate timer duration to program. */
		/* V                                     TX
		 * |       time_difference_us            |
		 * | margin | timer           |   delay  |
		 *                            G
		 *
		 * timer = time_difference_us - (delay + margin)
		 */
		int time_difference_dtu = *trx_date_dtu - cur_time_dtu;
		int time_difference_us = DTU_TO_US(time_difference_dtu);
		if (time_difference_us > delay_us)
			timer_us = time_difference_us - delay_us;
		/* else, too late for timer, set gpio now */
	}
	trace_dw3000_coex_gpio_start(dw, timer_us, dw->coex_status,
				     dw->coex_interval_us);
	if (dw->coex_status) {
		if (timer_us < dw->coex_interval_us)
			return 0; /* Nothing more to do */
		dw3000_coex_stop(dw);
	}
	/* Set coexistence gpio on chip */
	return dw3000_coex_gpio(dw, true, timer_us);
}

/**
 * dw3000_coex_stop - Handle WiFi coex gpio at end of uwb exchange.
 * @dw: the DW device
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_coex_stop(struct dw3000 *dw)
{
	if (dw->coex_gpio < 0 || !dw->coex_enabled)
		return 0;

	trace_dw3000_coex_gpio_stop(dw, dw->coex_status);
	if (!dw->coex_status)
		return 0;
	/* Reset coex GPIO on chip */
	return dw3000_coex_gpio(dw, false, 0);
}

/**
 * dw3000_coex_init - Initialise WiFi coex gpio
 * @dw: the DW device
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_coex_init(struct dw3000 *dw)
{
	int rc;

	if (dw->coex_gpio < 0)
		return 0;
	/* Sanity check chip dependent functions */
	if (!dw->chip_ops || !dw->chip_ops->coex_gpio ||
	    !dw->chip_ops->coex_init)
		return -ENOTSUPP;
	/* Call chip dependent initialisation */
	rc = dw->chip_ops->coex_init(dw);
	if (unlikely(rc)) {
		dev_err(dw->dev,
			"WiFi coexistence configuration has failed (%d)\n", rc);
	}
	dw->coex_status = false;
	return rc;
}

#endif /* __DW3000_COEX_H */
