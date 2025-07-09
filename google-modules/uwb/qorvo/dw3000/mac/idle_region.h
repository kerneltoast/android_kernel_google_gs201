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

#ifndef IDLE_REGION_H
#define IDLE_REGION_H

struct idle_params {
	/**
	 * @min_duration_dtu: Minimum duration of an access.
	 * If min is 0, no minimum is required on get_access.
	 */
	int min_duration_dtu;
	/**
	 * @max_duration_dtu: Maximum duration of an access.
	 */
	int max_duration_dtu;
};

int mcps802154_idle_region_init(void);
void mcps802154_idle_region_exit(void);

#endif /* IDLE_REGION_H */
