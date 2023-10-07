/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP debugfs support.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_DEBUGFS_H__
#define __GXP_DEBUGFS_H__

#include "gxp-internal.h"

/*
 * Creates the GXP debug FS directory and assigns to @gxp->d_entry.
 * On failure a warning is logged and @gxp->d_entry is NULL.
 */
void gxp_create_debugdir(struct gxp_dev *gxp);
void gxp_create_debugfs(struct gxp_dev *gxp);
void gxp_remove_debugdir(struct gxp_dev *gxp);

#endif /* __GXP_DEBUGFS_H__ */
