// SPDX-License-Identifier: GPL-2.0-only
/* buffer.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/mm.h>

void vh_bh_lru_install(void *data, struct page *page, bool *skip)
{
	if (is_migrate_cma_page(page))
		*skip = true;
}
