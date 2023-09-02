/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _BIGO_OF_H_
#define _BIGO_OF_H_

#include "bigo_priv.h"

int bigo_of_dt_parse(struct bigo_core *core);
void bigo_of_dt_release(struct bigo_core *core);

#endif //_BIGO_OF_H_
