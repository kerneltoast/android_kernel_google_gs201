/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _VH_MM_H
#define _VH_MM_H
#include <trace/hooks/mm.h>

void vh_pagevec_drain(void *data, struct page *page, bool *ret);
void vh_zap_pte_range_tlb_start(void *data, void *unused);
void vh_zap_pte_range_tlb_force_flush(void *data, struct page *page, bool *flush);
void vh_zap_pte_range_tlb_end(void *data, void *unused);
void vh_skip_lru_disable(void *data, bool *skip);

#endif
