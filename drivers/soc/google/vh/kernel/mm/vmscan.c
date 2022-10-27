// SPDX-License-Identifier: GPL-2.0-only
/* vmscan.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/mm.h>

void vh_shrink_inactive_list_blk_plug(void *data, bool *do_plug)
{
	*do_plug = true;
}

void vh_reclaim_pages_plug(void *data, bool *do_plug)
{
	*do_plug = true;
}
