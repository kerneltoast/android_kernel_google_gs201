/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _VH_GUP_H
#define _VH_GUP_H

#include <trace/hooks/gup.h>

void vh_try_grab_compound_head(void *data, struct page *page, int refs,
			       unsigned int flags, bool *ret);
void vh___get_user_pages_remote(void *data, int *locked,
				unsigned int *gup_flags, struct page **pages);
void vh_android_vh_get_user_pages(void *data, unsigned int *gup_flags,
				  struct page **pages);
void vh_internal_get_user_pages_fast(void *data, unsigned int *gup_flags,
				     struct page **pages);
void vh_pin_user_pages(void *data, unsigned int *gup_flags,
		       struct page **pages);

#endif
