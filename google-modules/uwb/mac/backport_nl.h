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

#ifndef BACKPORT_NL_H
#define BACKPORT_NL_H

#include <linux/types.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 7, 0)
#define NLA_POLICY_FULL_RANGE(tp, _range) \
	{                                 \
		.type = (tp)              \
	}

#else

/* NLA_POLICY_FULL_RANGE expect to have a range defined.
 * This define exist for backport linux compilations, as:
 *  - struct netlink_range_validation don't exist,
 *  - unused object are considered as error. */
#define ADD_NETLINK_RANGE_VALIDATION

#endif

#endif /* BACKPORT_NL_H */
