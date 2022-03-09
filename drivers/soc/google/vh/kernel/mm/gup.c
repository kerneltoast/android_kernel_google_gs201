// SPDX-License-Identifier: GPL-2.0-only
/* cma.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include "../../include/gup.h"
#include <linux/mm.h>
#include <linux/pageblock-flags.h>

void vh_try_grab_compound_head(void *data, struct page *page, int refs,
			       unsigned int flags, bool *ret)
{
	if (is_migrate_cma_page(page) && refs == 1)
		*ret = true;
}

void vh___get_user_pages_remote(void *data, int *locked,
				unsigned int *gup_flags, struct page **pages)
{
	if (!locked && pages)
		*gup_flags |= FOLL_LONGTERM;
}

void vh_android_vh_get_user_pages(void *data, unsigned int *gup_flags,
				  struct page **pages)
{
	if (pages)
		*gup_flags |= FOLL_LONGTERM;
}

void vh_internal_get_user_pages_fast(void *data, unsigned int *gup_flags,
				     struct page **pages)
{
	if (pages)
		*gup_flags |= FOLL_LONGTERM;
}

void vh_pin_user_pages(void *data, unsigned int *gup_flags,
		       struct page **pages)
{
	if (pages)
		*gup_flags |= FOLL_LONGTERM;
}
