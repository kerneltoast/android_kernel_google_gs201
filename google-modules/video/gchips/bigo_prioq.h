/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2021 Google LLC.
 *
 * Author: Ruofei Ma <ruofeim@google.com>
 */

#ifndef _BIGO_PRIOQ_H_
#define _BIGO_PRIOQ_H_

#include "bigo_priv.h"

bool dequeue_prioq(struct bigo_core *core, struct bigo_job **job,
                        bool *should_stop);
int enqueue_prioq(struct bigo_core *core, struct bigo_inst *inst);

void clear_job_from_prioq(struct bigo_core *core, struct bigo_inst *inst);

#endif //_BIGO_PRIOQ_H_
