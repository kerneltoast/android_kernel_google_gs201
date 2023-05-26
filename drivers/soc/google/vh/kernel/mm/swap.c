// SPDX-License-Identifier: GPL-2.0-only
/* swap.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/mm.h>

void vh_pagevec_drain(void *data, struct page *page, bool *ret)
{
	if (!*ret && is_migrate_cma_page(page))
		*ret = true;
}
