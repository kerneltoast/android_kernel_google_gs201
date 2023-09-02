/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _BIGO_PM_H_
#define _BIGO_PM_H_

#include <linux/device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

#include "bigo_priv.h"

int bigo_pm_init(struct bigo_core *core);

#if IS_ENABLED(CONFIG_PM)
int bigo_runtime_suspend(struct device *dev);
int bigo_runtime_resume(struct device *dev);
#endif
void bigo_update_qos(struct bigo_core *core);
void bigo_clocks_off(struct bigo_core *core);
void bigo_mark_qos_dirty(struct bigo_core *core);

#endif //_BIGO_PM_H_
