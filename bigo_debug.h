/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2021 Google LLC.
 *
 * Author: Ruofei Ma <ruofeim@google.com>
 */

#ifndef _BIGO_DEBUG_H_
#define _BIGO_DEBUG_H_

#include "bigo_priv.h"

#if IS_ENABLED(CONFIG_DEBUG_FS)
void bigo_init_debugfs(struct bigo_core *core);
void bigo_uninit_debugfs(struct bigo_core *core);
#else
static inline void bigo_init_debugfs(struct bigo_core *core) { }
static inline void bigo_uninit_debugfs(struct bigo_core *core) { }
#endif

#endif /* _BIGO_DEBUG_H_ */
