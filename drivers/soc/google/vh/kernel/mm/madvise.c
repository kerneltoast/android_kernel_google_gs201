// SPDX-License-Identifier: GPL-2.0-only
/* madvise.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/mman.h>
#include <linux/mm.h>

void vh_do_madvise_blk_plug(void *data, int behavior, bool *do_plug)
{
	if (behavior == MADV_PAGEOUT)
		*do_plug = false;
}
