/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU Ultra Short Reach utilities.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_USR_H__
#define __EDGETPU_USR_H__

#include "edgetpu-config.h"
#include "edgetpu-device-group.h"

#ifdef EDGETPU_HAS_VN

/*
 * Trains the USR links of devices in @group.
 *
 * Caller ensures edgetpu_device_group_nth_etdev available to @group and holds
 * @group->lock.
 */
void edgetpu_usr_init_group(struct edgetpu_device_group *group);

/*
 * Stop the USR ring for all devices in @group.
 * Caller holds @group->lock.
 */
void edgetpu_usr_release_group(struct edgetpu_device_group *group);

#else /* !EDGETPU_HAS_VN */

static inline void edgetpu_usr_init_group(struct edgetpu_device_group *group)
{
}

static inline void
edgetpu_usr_release_group(struct edgetpu_device_group *group)
{
}
#endif /* EDGETPU_HAS_VN */

#endif /* __EDGETPU_USR_H__ */
