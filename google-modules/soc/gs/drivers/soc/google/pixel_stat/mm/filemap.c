// SPDX-License-Identifier: GPL-2.0-only
/* filemap.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/mm.h>

DECLARE_PER_CPU(unsigned long, pgcache_miss);
DECLARE_PER_CPU(unsigned long, pgcache_hit);

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */
void vh_filemap_get_folio_mod(void *data, struct address_space *mapping,
			pgoff_t index, int fgp_flags,
			gfp_t gfp_mask, struct folio *folio)
{
	if (!folio)
		this_cpu_inc(pgcache_miss);
	else
		this_cpu_inc(pgcache_hit);
}

void rvh_mapping_shrinkable(void *data, bool *shrinkable)
{
	*shrinkable = true;
}
