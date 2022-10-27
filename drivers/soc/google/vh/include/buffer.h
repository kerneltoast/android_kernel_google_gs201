/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _VH_BUFFER_H
#define _VH_BUFFER_H
#include <trace/hooks/buffer.h>

void vh_bh_lru_install(void *data, struct page *page, bool *skip);

#endif
