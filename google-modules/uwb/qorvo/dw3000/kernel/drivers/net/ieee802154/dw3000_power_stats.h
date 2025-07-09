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

/**
 * dw3000_power_stats() - compute time elapsed in dw3000 states
 * @dw: the DW device on which state is changed
 * @state: the new state
 * @len_or_date: frame length to be transmitted or RX dates in DTU
 *
 * Update power statistics according current state leaved and new state.
 * States > RUN are handled specifically.
 */
static inline void dw3000_power_stats(struct dw3000 *dw, int state,
				      int len_or_date)
{
	struct dw3000_power *pw = &dw->power;
	int cstate = min(pw->cur_state, DW3000_PWR_RUN);
	int nstate = min(state, DW3000_PWR_RUN);
	u64 boot_time_ns = ktime_get_boottime_ns();
	s64 duration;
	u32 adjust;
	u32 cur_dtu_time;

	/* Trace call */
	trace_dw3000_power_stats(dw, state, boot_time_ns, len_or_date);
	/* Sanity checks first */
	if (state < 0 || state >= DW3000_PWR_MAX)
		return;
	/* Calculate duration of current state. */
	duration = boot_time_ns - pw->start_time;
	/* Update basic state statistics */
	pw->stats[cstate].dur += duration;
	if (nstate != cstate)
		pw->stats[nstate].count++;
	/* Handle specific states */
	cstate = pw->cur_state;
	switch (state) {
	case DW3000_PWR_IDLE:
		adjust = 0;
		if (cstate == DW3000_PWR_TX) {
			/* TX duration is just saved pw->tx_adjust */
			adjust = (u32)pw->tx_adjust;
		} else if (cstate == DW3000_PWR_RX) {
			/* RX duration is just cur_dtu_time-pw->rx_start */
			cur_dtu_time = (u32)len_or_date;
			if (!cur_dtu_time) {
				/* RX frame end time is not given, so get current DTU. */
				cur_dtu_time =
					dw3000_ktime_to_dtu(dw, boot_time_ns);
			}
			adjust = cur_dtu_time - pw->rx_start;
		}
		pw->stats[cstate].dur += adjust;
		if (state != cstate)
			pw->stats[DW3000_PWR_IDLE].count++;
		break;
	case DW3000_PWR_TX:
		/* TX time is calculated according frame len only */
		pw->tx_adjust =
			dw3000_frame_duration_dtu(dw, len_or_date, true);
		pw->stats[DW3000_PWR_TX].count++;
		break;
	case DW3000_PWR_RX:
		/* RX time is calculated using start time and reception time */
		cur_dtu_time = (u32)len_or_date;
		if (!cur_dtu_time) {
			/* Start time is unknown for immediate RX but we need
			   it, so get current DTU time. */
			cur_dtu_time = dw3000_ktime_to_dtu(dw, boot_time_ns);
		}
		pw->rx_start = cur_dtu_time;
		pw->stats[DW3000_PWR_RX].count++;
		break;
	case DW3000_PWR_RUN:
		/* Entering RUN state also enter IDLE state */
		if (state != cstate)
			pw->stats[DW3000_PWR_IDLE].count++;
		break;
	default:
		break;
	}
	/* Update current information */
	pw->start_time = boot_time_ns;
	pw->cur_state = state;
}
