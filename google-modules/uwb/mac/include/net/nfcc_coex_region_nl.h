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

#ifndef NFCC_COEX_REGION_NL_H
#define NFCC_COEX_REGION_NL_H

/**
 * enum nfcc_coex_call - NFCC coexistence calls identifiers.
 *
 * @NFCC_COEX_CALL_CCC_SESSION_START:
 *	Start CCC session.
 * @NFCC_COEX_CALL_CCC_SESSION_STOP:
 *	Stop CCC session.
 * @NFCC_COEX_CALL_CCC_SESSION_NOTIFICATION:
 *	Notify session reports.
 * @NFCC_COEX_CALL_MAX: Internal use.
 */
enum nfcc_coex_call {
	NFCC_COEX_CALL_CCC_SESSION_START,
	NFCC_COEX_CALL_CCC_SESSION_STOP,
	NFCC_COEX_CALL_CCC_SESSION_NOTIFICATION,
	NFCC_COEX_CALL_MAX,
};

/**
 * enum nfcc_coex_call_attrs - NFCC coexistence call attributes.
 *
 * @NFCC_COEX_CALL_ATTR_CCC_SESSION_PARAMS:
 *	Session parameters.
 * @NFCC_COEX_CALL_ATTR_CCC_WATCHDOG_TIMEOUT:
 *	Watchdog trigged.
 * @NFCC_COEX_CALL_ATTR_CCC_STOPPED:
 *	Session stopped.
 *
 * @NFCC_COEX_CALL_ATTR_UNSPEC: Invalid command.
 * @__NFCC_COEX_CALL_ATTR_AFTER_LAST: Internal use.
 * @NFCC_COEX_CALL_ATTR_MAX: Internal use.
 */
enum nfcc_coex_call_attrs {
	NFCC_COEX_CALL_ATTR_UNSPEC,

	NFCC_COEX_CALL_ATTR_CCC_SESSION_PARAMS,
	NFCC_COEX_CALL_ATTR_CCC_WATCHDOG_TIMEOUT,
	NFCC_COEX_CALL_ATTR_CCC_STOPPED,

	__NFCC_COEX_CALL_ATTR_AFTER_LAST,
	NFCC_COEX_CALL_ATTR_MAX = __NFCC_COEX_CALL_ATTR_AFTER_LAST - 1
};

/**
 * enum nfcc_coex_ccc_session_param_attrs - NFCC coexistence session parameters attributes.
 *
 * @NFCC_COEX_CCC_SESSION_PARAM_ATTR_TIME0_NS:
 *	Initiation time in unit of ns, default 0.
 * @NFCC_COEX_CCC_SESSION_PARAM_ATTR_CHANNEL_NUMBER:
 *	Override channel for this session: 5, 6, 8, 9, 10, 12, 13 or 14.
  * @NFCC_COEX_CCC_SESSION_PARAM_ATTR_VERSION:
 *	Protocol version to be used.
 *
 * @NFCC_COEX_CCC_SESSION_PARAM_ATTR_UNSPEC: Invalid command.
 * @__NFCC_COEX_CCC_SESSION_PARAM_ATTR_AFTER_LAST: Internal use.
 * @NFCC_COEX_CCC_SESSION_PARAM_ATTR_MAX: Internal use.
 */
enum nfcc_coex_ccc_session_param_attrs {
	NFCC_COEX_CCC_SESSION_PARAM_ATTR_UNSPEC,

	NFCC_COEX_CCC_SESSION_PARAM_ATTR_TIME0_NS,
	NFCC_COEX_CCC_SESSION_PARAM_ATTR_CHANNEL_NUMBER,
	NFCC_COEX_CCC_SESSION_PARAM_ATTR_VERSION,

	__NFCC_COEX_CCC_SESSION_PARAM_ATTR_AFTER_LAST,
	NFCC_COEX_CCC_SESSION_PARAM_ATTR_MAX =
		__NFCC_COEX_CCC_SESSION_PARAM_ATTR_AFTER_LAST - 1
};

#endif /* NFCC_COEX_REGION_NL_H */
