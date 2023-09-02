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

#ifndef NET_PCTT_REGION_NL_H
#define NET_PCTT_REGION_NL_H

/**
 * enum pctt_call - PCTT calls identifiers.
 *
 * @PCTT_CALL_SESSION_INIT:
 *	Initialize PCTT session.
 * @PCTT_CALL_SESSION_CMD:
 *	Identifier of command to do: start test, stop.
 *	TODO: Could be move in session parameters.
 * @PCTT_CALL_SESSION_DEINIT:
 *	Deinit PCTT session.
 * @PCTT_CALL_SESSION_GET_STATE:
 *	Get session state.
 * @PCTT_CALL_SESSION_GET_PARAMS:
 *	Request parameters.
 * @PCTT_CALL_SESSION_SET_PARAMS:
 *	Set session parameters.
 * @PCTT_CALL_SESSION_NOTIFICATION:
 *	Notify session reports.
 * @PCTT_CALL_MAX: Internal use.
 */
enum pctt_call {
	PCTT_CALL_SESSION_INIT,
	PCTT_CALL_SESSION_CMD,
	PCTT_CALL_SESSION_DEINIT,
	PCTT_CALL_SESSION_GET_STATE,
	PCTT_CALL_SESSION_GET_PARAMS,
	PCTT_CALL_SESSION_SET_PARAMS,
	PCTT_CALL_SESSION_NOTIFICATION,
	PCTT_CALL_MAX,
};

enum pctt_call_attrs {
	PCTT_CALL_ATTR_UNSPEC,
	PCTT_CALL_ATTR_CMD_ID,
	PCTT_CALL_ATTR_RESULT_DATA,
	PCTT_CALL_ATTR_SESSION_ID,
	PCTT_CALL_ATTR_SESSION_STATE,
	PCTT_CALL_ATTR_SESSION_PARAMS,

	__PCTT_CALL_ATTR_AFTER_LAST,
	PCTT_CALL_ATTR_MAX = __PCTT_CALL_ATTR_AFTER_LAST - 1,
};

/**
 * enum pctt_session_param_attrs - Pctt session parameters attributes.
 *
 * @PCTT_SESSION_PARAM_ATTR_DEVICE_ROLE:
 *	Responder (0) or initiator (1)
 * @PCTT_SESSION_PARAM_ATTR_SHORT_ADDR:
 *	Override device address for this session (UCI: DEVICE_MAC_ADDRESS)
 * @PCTT_SESSION_PARAM_ATTR_DESTINATION_SHORT_ADDR:
 *	Controller short addresses (UCI: DST_MAC_ADDRESS) [controlee only]
 * @PCTT_SESSION_PARAM_ATTR_RX_ANTENNA_SELECTION:
 *	Antenna set to use during RX phases
 * @PCTT_SESSION_PARAM_ATTR_TX_ANTENNA_SELECTION:
 *	Antenna set to use during Tx phases
 * @PCTT_SESSION_PARAM_ATTR_SLOT_DURATION_RSTU:
 *	Duration of a slot in RSTU, default 2400. (2 ms)
 * @PCTT_SESSION_PARAM_ATTR_CHANNEL_NUMBER:
 *	Override channel for this session: 5, 6, 8, 9, 10, 12, 13 or 14
 * @PCTT_SESSION_PARAM_ATTR_PREAMBLE_CODE_INDEX:
 *	Override preamble code for this session, BPRF (9-24),
 *	HPRF (25-32, not supported)
 * @PCTT_SESSION_PARAM_ATTR_RFRAME_CONFIG:
 *	SP0 (0), SP1 (1), SP2 (2, unused, not in PcTt 1.1) or SP3 (3, default)
 * @PCTT_SESSION_PARAM_ATTR_PRF_MODE:
 *	BPRF (0, default) or HPRF (1, not supported)
 * @PCTT_SESSION_PARAM_ATTR_PREAMBLE_DURATION:
 *	64 (1, default) or 32 (0, only for HPRF)
 * @PCTT_SESSION_PARAM_ATTR_SFD_ID:
 *	BPRF (0 or 2), HPRF (1-4, not supported), default 2
 * @PCTT_SESSION_PARAM_ATTR_NUMBER_OF_STS_SEGMENTS:
 *	0-2, default to 0 for SP0, default to 1 for SP1 & SP3, 2 not supported
 * @PCTT_SESSION_PARAM_ATTR_PSDU_DATA_RATE:
 *	6.81 Mbps (0, default), 7.80 Mbps (1, not supported),
 *	27.2 Mbps (2, not supported), 31.2 Mbps (3, not supported)
 * @PCTT_SESSION_PARAM_ATTR_BPRF_PHR_DATA_RATE:
 *	850 kbps (0, default) or 6.81 Mbps (1)
 * @PCTT_SESSION_PARAM_ATTR_MAC_FCS_TYPE:
 *	CRC16 (0, default) or CRC32 (1, not supported)
 * @PCTT_SESSION_PARAM_ATTR_TX_ADAPTIVE_PAYLOAD_POWER:
 *	Disable adaptive payload power for TX (0, default) or enable (1)
 * @PCTT_SESSION_PARAM_ATTR_STS_INDEX:
 *	STS index initialization value
 * @PCTT_SESSION_PARAM_ATTR_STS_LENGTH:
 *  	Number of symbols in a STS segment. 32 (0x00), 64 (0x01, default) or 128
 *  	symbols (0x02)
 * @PCTT_SESSION_PARAM_ATTR_NUM_PACKETS:
 * 	Number of packets (default 1000).
 * @PCTT_SESSION_PARAM_ATTR_T_GAP:
 *  	Gap between start of one packet to the next in µs (default 2000).
 * @PCTT_SESSION_PARAM_ATTR_T_START:
 *  	Max. time from the start of T_GAP to SFD found state in µs (default
 *	450us).
 * @PCTT_SESSION_PARAM_ATTR_T_WIN:
 *  	Max. time for which RX is looking for a packet from the start of T_GAP
 * 	in µs (default 750).
 * @PCTT_SESSION_PARAM_ATTR_RANDOMIZE_PSDU:
 *  	Disable (0, default) or enable (1) PSDU randomization.
 * @PCTT_SESSION_PARAM_ATTR_PHR_RANGING_BIT:
 *  	Disable (0, default) or enable (1) ranging bit field of PHR in both BPRF
 * 	and HPRF.
 * @PCTT_SESSION_PARAM_ATTR_RMARKER_TX_START:
 *  Start time of TX in 1/(128*499.2MHz) units.
 * @PCTT_SESSION_PARAM_ATTR_RMARKER_RX_START:
 *  Start time of RX in 1/(128*499.2MHz) units.
 * @PCTT_SESSION_PARAM_ATTR_STS_INDEX_AUTO_INCR:
 *  	Disable (0, default) or enable (1) incrementation of STS_INDEX config
 * 	value for every frame in PER Rx/Periodic TX test.
 * @PCTT_SESSION_PARAM_ATTR_DATA_PAYLOAD:
 *	PSDU Data.
 * @PCTT_SESSION_PARAM_ATTR_UNSPEC: Invalid command.
 * @__PCTT_SESSION_PARAM_ATTR_AFTER_LAST: Internal use.
 * @PCTT_SESSION_PARAM_ATTR_MAX: Internal use.
 */
enum pctt_session_param_attrs {
	PCTT_SESSION_PARAM_ATTR_UNSPEC,
	/* Main session parameters */
	PCTT_SESSION_PARAM_ATTR_DEVICE_ROLE,
	PCTT_SESSION_PARAM_ATTR_SHORT_ADDR,
	PCTT_SESSION_PARAM_ATTR_DESTINATION_SHORT_ADDR,
	PCTT_SESSION_PARAM_ATTR_RX_ANTENNA_SELECTION,
	PCTT_SESSION_PARAM_ATTR_TX_ANTENNA_SELECTION,
	/* Timings */
	PCTT_SESSION_PARAM_ATTR_SLOT_DURATION_RSTU,
	/* Radio */
	PCTT_SESSION_PARAM_ATTR_CHANNEL_NUMBER,
	PCTT_SESSION_PARAM_ATTR_PREAMBLE_CODE_INDEX,
	PCTT_SESSION_PARAM_ATTR_RFRAME_CONFIG,
	PCTT_SESSION_PARAM_ATTR_PRF_MODE,
	PCTT_SESSION_PARAM_ATTR_PREAMBLE_DURATION,
	PCTT_SESSION_PARAM_ATTR_SFD_ID,
	PCTT_SESSION_PARAM_ATTR_NUMBER_OF_STS_SEGMENTS,
	PCTT_SESSION_PARAM_ATTR_PSDU_DATA_RATE,
	PCTT_SESSION_PARAM_ATTR_BPRF_PHR_DATA_RATE,
	PCTT_SESSION_PARAM_ATTR_MAC_FCS_TYPE,
	PCTT_SESSION_PARAM_ATTR_TX_ADAPTIVE_PAYLOAD_POWER,
	/* STS and crypto */
	PCTT_SESSION_PARAM_ATTR_STS_INDEX,
	PCTT_SESSION_PARAM_ATTR_STS_LENGTH,
	/* Test configuration parameters */
	PCTT_SESSION_PARAM_ATTR_NUM_PACKETS,
	PCTT_SESSION_PARAM_ATTR_T_GAP,
	PCTT_SESSION_PARAM_ATTR_T_START,
	PCTT_SESSION_PARAM_ATTR_T_WIN,
	PCTT_SESSION_PARAM_ATTR_RANDOMIZE_PSDU,
	PCTT_SESSION_PARAM_ATTR_PHR_RANGING_BIT,
	PCTT_SESSION_PARAM_ATTR_RMARKER_TX_START,
	PCTT_SESSION_PARAM_ATTR_RMARKER_RX_START,
	PCTT_SESSION_PARAM_ATTR_STS_INDEX_AUTO_INCR,
	/* Payload */
	PCTT_SESSION_PARAM_ATTR_DATA_PAYLOAD,

	__PCTT_SESSION_PARAM_ATTR_AFTER_LAST,
	PCTT_SESSION_PARAM_ATTR_MAX = __PCTT_SESSION_PARAM_ATTR_AFTER_LAST - 1
};

enum pctt_id_attrs {
	PCTT_ID_ATTR_UNSPEC,

	PCTT_ID_ATTR_PERIODIC_TX,
	PCTT_ID_ATTR_PER_RX,
	PCTT_ID_ATTR_RX,
	PCTT_ID_ATTR_LOOPBACK,
	PCTT_ID_ATTR_SS_TWR,
	PCTT_ID_ATTR_STOP_TEST,

	__PCTT_ID_ATTR_AFTER_LAST,
	PCTT_ID_ATTR_MAX = __PCTT_ID_ATTR_AFTER_LAST - 1
};

enum pctt_result_data_attrs {
	PCTT_RESULT_DATA_ATTR_UNSPEC,

	PCTT_RESULT_DATA_ATTR_STATUS,
	PCTT_RESULT_DATA_ATTR_ATTEMPTS,
	PCTT_RESULT_DATA_ATTR_ACQ_DETECT,
	PCTT_RESULT_DATA_ATTR_ACQ_REJECT,
	PCTT_RESULT_DATA_ATTR_RX_FAIL,
	PCTT_RESULT_DATA_ATTR_SYNC_CIR_READY,
	PCTT_RESULT_DATA_ATTR_SFD_FAIL,
	PCTT_RESULT_DATA_ATTR_SFD_FOUND,
	PCTT_RESULT_DATA_ATTR_PHR_DEC_ERROR,
	PCTT_RESULT_DATA_ATTR_PHR_BIT_ERROR,
	PCTT_RESULT_DATA_ATTR_PSDU_DEC_ERROR,
	PCTT_RESULT_DATA_ATTR_PSDU_BIT_ERROR,
	PCTT_RESULT_DATA_ATTR_STS_FOUND,
	PCTT_RESULT_DATA_ATTR_EOF,

	PCTT_RESULT_DATA_ATTR_RX_DONE_TS_INT,
	PCTT_RESULT_DATA_ATTR_RX_DONE_TS_FRAC,
	PCTT_RESULT_DATA_ATTR_AOA_AZIMUTH,
	PCTT_RESULT_DATA_ATTR_AOA_ELEVATION,
	PCTT_RESULT_DATA_ATTR_TOA_GAP,
	PCTT_RESULT_DATA_ATTR_PHR,
	PCTT_RESULT_DATA_ATTR_PSDU_DATA_LEN,
	PCTT_RESULT_DATA_ATTR_PSDU_DATA,
	PCTT_RESULT_DATA_ATTR_TX_TS_INT,
	PCTT_RESULT_DATA_ATTR_TX_TS_FRAC,
	PCTT_RESULT_DATA_ATTR_RX_TS_INT,
	PCTT_RESULT_DATA_ATTR_RX_TS_FRAC,

	PCTT_RESULT_DATA_ATTR_MEASUREMENT,

	PCTT_RESULT_DATA_ATTR_PDOA_AZIMUTH_DEG_Q7,
	PCTT_RESULT_DATA_ATTR_PDOA_ELEVATION_DEG_Q7,
	PCTT_RESULT_DATA_ATTR_RSSI,
	PCTT_RESULT_DATA_ATTR_AOA_AZIMUTH_DEG_Q7,
	PCTT_RESULT_DATA_ATTR_AOA_ELEVATION_DEG_Q7,

	__PCTT_RESULT_DATA_ATTR_AFTER_LAST,
	PCTT_RESULT_DATA_ATTR_MAX = __PCTT_RESULT_DATA_ATTR_AFTER_LAST - 1,
};

#endif /* NET_PCTT_REGION_NL_H */
