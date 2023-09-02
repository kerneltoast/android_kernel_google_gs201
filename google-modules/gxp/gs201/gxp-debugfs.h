/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP debugfs support.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_DEBUGFS_H__
#define __GXP_DEBUGFS_H__

#include "gxp-internal.h"

void gxp_create_debugfs(struct gxp_dev *gxp);
void gxp_remove_debugfs(struct gxp_dev *gxp);

#endif /* __GXP_DEBUGFS_H__ */
