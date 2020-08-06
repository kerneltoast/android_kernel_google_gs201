/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _BIGO_SLC_H_
#define _BIGO_SLC_H_

#include "bigo_priv.h"

void bigo_pt_resize_cb(void *data, int id, size_t size_allocated);
int bigo_pt_client_enable(struct bigo_core *core);
void bigo_pt_client_disable(struct bigo_core *core);
void bigo_get_cache_info(struct bigo_core *core, struct bigo_cache_info *cinfo);
void bigo_bypass_ssmt_pid(struct bigo_core *core);
#endif //_BIGO_SLC_H_
