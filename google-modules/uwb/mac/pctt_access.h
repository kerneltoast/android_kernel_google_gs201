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

#ifndef PCTT_ACCESS_H
#define PCTT_ACCESS_H

#include <net/mcps802154_schedule.h>

/**
 * pctt_get_access() - Get access for a given region at the given timestamp.
 * @region: Region.
 * @next_timestamp_dtu: Date of next access opportunity.
 * @next_in_region_dtu: Region start from the start of the access opportunity.
 * @region_duration_dtu: Region duration, or 0 for endless region.
 *
 * Return: The access.
 */
struct mcps802154_access *pctt_get_access(struct mcps802154_region *region,
					  u32 next_timestamp_dtu,
					  int next_in_region_dtu,
					  int region_duration_dtu);

#endif /* PCTT_ACCESS_H */
