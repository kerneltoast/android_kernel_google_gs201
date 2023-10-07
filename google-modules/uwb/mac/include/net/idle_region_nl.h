/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2022 Qorvo US, Inc.
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

#ifndef IDLE_REGION_NL_H
#define IDLE_REGION_NL_H

/**
 * enum idle_param_attrs - Idle parameters attributes.
 *
 * @IDLE_PARAM_ATTR_MIN_DURATION_DTU:
 *	Minimum duration of an access.
 * @IDLE_PARAM_ATTR_MAX_DURATION_DTU:
 *	Maximum duration of an access.
 *
 * @IDLE_PARAM_ATTR_UNSPEC: Invalid command.
 * @__IDLE_PARAM_ATTR_AFTER_LAST: Internal use.
 * @IDLE_PARAM_ATTR_MAX: Internal use.
 */
enum idle_param_attrs {
	IDLE_PARAM_ATTR_UNSPEC,

	IDLE_PARAM_ATTR_MIN_DURATION_DTU,
	IDLE_PARAM_ATTR_MAX_DURATION_DTU,

	__IDLE_PARAM_ATTR_AFTER_LAST,
	IDLE_PARAM_ATTR_MAX = __IDLE_PARAM_ATTR_AFTER_LAST - 1
};

#endif /* IDLE_REGION_NL_H */
