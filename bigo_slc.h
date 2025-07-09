/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2022 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _BIGO_SLC_H_
#define _BIGO_SLC_H_

#include "bigo_priv.h"

#if IS_ENABLED(ENABLE_SLC)
#include <soc/google/pt.h>
int bigo_pt_client_register(struct device_node *node, struct bigo_core *core);
void bigo_pt_client_unregister(struct bigo_core *core);
int bigo_pt_client_enable(struct bigo_core *core);
void bigo_pt_client_disable(struct bigo_core *core);
void bigo_get_cache_info(struct bigo_core *core, struct bigo_cache_info *cinfo);
void bigo_bypass_ssmt_pid(struct bigo_core *core, bool dec_mode);
#else
static inline int bigo_pt_client_register(struct device_node *node, struct bigo_core *core) { return 0; }
static inline void bigo_pt_client_unregister(struct bigo_core *core) { }
static inline int bigo_pt_client_enable(struct bigo_core *core) { return 0; }
static inline void bigo_pt_client_disable(struct bigo_core *core) { }
static inline void bigo_get_cache_info(struct bigo_core *core, struct bigo_cache_info *cinfo) { }
static inline void bigo_bypass_ssmt_pid(struct bigo_core *core, bool dec_mode) { }
#endif

#endif //_BIGO_SLC_H_
