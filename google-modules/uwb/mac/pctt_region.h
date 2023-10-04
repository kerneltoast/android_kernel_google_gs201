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

#ifndef PCTT_REGION_H
#define PCTT_REGION_H

#include <linux/kernel.h>
#include <net/mcps802154_schedule.h>
#include <net/vendor_cmd.h>
#include <linux/types.h>
#include <net/pctt_region_params.h>
#include <net/pctt_region_nl.h>

#include "pctt_session.h"

#define PCTT_SESSION_ID 0
#define PCTT_BOOLEAN_MAX 1
#define PCTT_FRAMES_MAX 2

#define PCTT_TIMESTAMP_SHIFT 9
/**
 * map_rad_q11_to_deg_q7() - Map a Fixed Point angle to a signed 16-bit integer
 * @ang_rad_q11: angle as Q11 fixed_point value in range [-PI, PI]
 *
 * Return: the angle mapped to deg q7
 */
static inline s16 map_rad_q11_to_deg_q7(int ang_rad_q11)
{
	/* 180 / (pi * (1 << 4)) => ~3,581. */
	return ang_rad_q11 * 3581 / 1000;
}

/**
 * struct pctt_test_per_rx_results - PER_RX result for report.
 */
struct pctt_test_per_rx_results {
	/**
	 * @attempts: No. of RX attempts.
	 */
	u32 attempts;
	/**
	 * @acq_detect: No. of times signal was detected.
	 */
	u32 acq_detect;
	/**
	 * @acq_reject: No. of times signal was rejected.
	 */
	u32 acq_reject;
	/**
	 * @rx_fail: No. of times RX did not go beyond ACQ stage.
	 */
	u32 rx_fail;
	/**
	 * @sync_cir_ready: No. of times sync CIR ready event was received.
	 */
	u32 sync_cir_ready;
	/**
	 * @sfd_fail: No. of time RX was stuck at either ACQ detect or sync CIR ready.
	 */
	u32 sfd_fail;
	/**
	 * @sfd_found: No. of times SFD was found.
	 */
	u32 sfd_found;
	/**
	 * @phr_dec_error: No. of times PHR decode failed.
	 */
	u32 phr_dec_error;
	/**
	 * @phr_bit_error: No. of times PHR bits in error.
	 */
	u32 phr_bit_error;
	/**
	 * @psdu_dec_error: No. of times payload decode failed.
	 */
	u32 psdu_dec_error;
	/**
	 * @psdu_bit_error: No. of times payload bits in error.
	 */
	u32 psdu_bit_error;
	/**
	 * @sts_found: No. of times STS detection was successful.
	 */
	u32 sts_found;
	/**
	 * @eof: No. of times end of frame event was triggered.
	 */
	u32 eof;
	/**
	 * @rssi: Received signal strength indication (RSSI).
	 */
	u8 rssi;
};

/**
 * struct pctt_test_rx_results - RX result for report.
 */
struct pctt_test_rx_results {
	/**
	 * @rx_done_ts_int: Integer part of timestamp 1/124.8Mhz ticks.
	 */
	u32 rx_done_ts_int;
	/**
	 * @rx_done_ts_frac: Fractional part of timestamp in 1/(128 * 499.2Mhz) ticks.
	 */
	u16 rx_done_ts_frac;
	/**
	 * @aoa_azimuth: AoA azimuth in degrees and it is a signed value in Q9.7 format.
	 */
	s16 aoa_azimuth;
	/**
	 * @aoa_elevation: AoA elevation in degrees and it is a signed value in Q9.7 format.
	 */
	s16 aoa_elevation;
	/**
	 * @phr: Received PHR (bits 0-12 as per IEEE spec).
	 */
	u16 phr;
	/**
	 * @psdu_data_len: Length of PSDU Data(N).
	 */
	u16 psdu_data_len;
	/**
	 * @toa_gap: ToA of main path minus ToA of first path in nanosecond.
	 */
	u8 toa_gap;
	/**
	 * @rssi: Received signal strength indication (RSSI).
	 */
	u8 rssi;
	/**
	 * @psdu_data: Received PSDU Data[0:N] bytes.
	 */
	u8 psdu_data[PCTT_PAYLOAD_MAX_LEN];
};

/**
 * struct pctt_test_ss_twr_results - SS_TWR result for report.
 */
struct pctt_test_ss_twr_results {
	/**
	 * @tx_timestamps_rctu: Timestamps of transmitted frame.
	 */
	u64 tx_timestamps_rctu;
	/**
	 * @rx_timestamps_rctu: Timestamps of received frame.
	 */
	u64 rx_timestamps_rctu;
	/**
	 * @measurement_rctu: Contains Tround time of Initiator or
	 * Treply time of Responder depending on DEVICE_ROLE option.
	 */
	u32 measurement_rctu;
	/**
	 * @pdoa_azimuth_deg_q7: Phase Difference of Arrival Azimuth in deg Q7
	 */
	s16 pdoa_azimuth_deg_q7;
	/**
	 * @aoa_azimuth_deg_q7: AoA Azimuth in deg Q7
	 */
	s16 aoa_azimuth_deg_q7;
	/**
	 * @pdoa_elevation_deg_q7: Phase Difference of Arrival Elevation in deg Q7
	 */
	s16 pdoa_elevation_deg_q7;
	/**
	 * @aoa_elevation_deg_q7: AoA Elevation in deg Q7
	 */
	s16 aoa_elevation_deg_q7;
	/**
	 * @rssi: Received signal strength indication (RSSI).
	 */
	u8 rssi;
};

/**
 * struct pctt_test_loopback_results - LOOPBACK result for report.
 */
struct pctt_test_loopback_results {
	/**
	 * @rssi: Received signal strength indication (RSSI).
	 */
	u8 rssi;
	/**
	 * @tx_ts_int: Integer part of TX timestamp in 1/124.8 us. resolution.
	 */
	u32 tx_ts_int;
	/**
	 * @tx_ts_frac: Fractional part of TX timestamp in 1/124.8/512 us. resolution.
	 */
	u16 tx_ts_frac;
	/**
	 * @rx_ts_int: Integer part of Rx timestamp in 1/124.8 us. resolution.
	 */
	u32 rx_ts_int;
	/**
	 * @rx_ts_frac: Fractional part of RX timestamp in 1/124.8/512 us. resolution.
	 */
	u16 rx_ts_frac;
};

/**
 * union pctt_tests_results - All commands notifications.
 */
union pctt_tests_results {
	/**
	 * @per_rx: Result of the PER_RX command.
	 */
	struct pctt_test_per_rx_results per_rx;
	/**
	 * @rx: Result of the RX command.
	 */
	struct pctt_test_rx_results rx;
	/**
	 * @ss_twr: Result of the SS_TWR command.
	 */
	struct pctt_test_ss_twr_results ss_twr;
	/**
	 * @loopback: Result of the LOOPBACK command.
	 */
	struct pctt_test_loopback_results loopback;
};

/**
 * union pctt_results - Main notification for all commands.
 */
struct pctt_results {
	/**
	 * @status: Result of the command done.
	 */
	enum pctt_status_ranging status;
	/**
	 * @tests: Result detail.
	 */
	union pctt_tests_results tests;
};

/**
 * struct pctt_slot - Information on an active slot.
 */
struct pctt_slot {
	/**
	 * @is_tx: Is transmit frame?
	 */
	bool is_tx;
	/**
	 * @is_rframe: Is ranging frame?
	 */
	bool is_rframe;
	/**
	 * @is_immediate: True when the frame is immediate.
	 */
	bool is_immediate;
	/**
	 * @timeout_dtu: see (mcps802154_rx_frame_config).timeout_dtu.
	 */
	int timeout_dtu;
};

struct pctt_local {
	/**
	 * @region: Region instance returned to MCPS.
	 */
	struct mcps802154_region region;
	/**
	 * @llhw: Low-level device pointer.
	 */
	struct mcps802154_llhw *llhw;
	/**
	 * @access: Access returned to MCPS.
	 */
	struct mcps802154_access access;
	/**
	 * @session: Unique session on the PCTT.
	 */
	struct pctt_session session;
	/**
	 * @frames: Access frames referenced from access.
	 */
	struct mcps802154_access_frame frames[PCTT_FRAMES_MAX];
	/**
	 * @sts_params: STS parameters for access frames.
	 */
	struct mcps802154_sts_params sts_params[PCTT_FRAMES_MAX];
	/**
	 * @slots: Descriptions of each active slots for the current session.
	 */
	struct pctt_slot slots[PCTT_FRAMES_MAX];
	/**
	 * @results: Test result used for notification at the end of test.
	 */
	struct pctt_results results;
	/**
	 * @frames_remaining_nb: Number of frame remaining to do for the current test.
	 */
	int frames_remaining_nb;
};

static inline struct pctt_local *
region_to_local(struct mcps802154_region *region)
{
	return container_of(region, struct pctt_local, region);
}

static inline struct pctt_local *
access_to_local(struct mcps802154_access *access)
{
	return container_of(access, struct pctt_local, access);
}

/**
 * pctt_report() - PCTT Report.
 * @local: pctt context.
 */
void pctt_report(struct pctt_local *local);

/**
 * pctt_session_notify_state_change() - Notify session state change to upper layers.
 * @local: context.
 */
void pctt_session_notify_state_change(struct pctt_local *local);

#endif /* PCTT_REGION_H */
