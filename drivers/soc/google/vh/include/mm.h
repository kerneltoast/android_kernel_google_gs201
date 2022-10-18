/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _VH_MM_H
#define _VH_MM_H
#include <trace/hooks/mm.h>

void vh_pagevec_drain(void *data, struct page *page, bool *ret);

#endif
