/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP wakelock support
 *
 * Copyright (C) 2022 Google LLC
 */
#ifndef __GXP_WAKELOCK_H__
#define __GXP_WAKELOCK_H__

#include "gxp-internal.h"
#include "gxp.h"

struct gxp_wakelock_manager {
	/* Protects count and suspended */
	struct mutex lock;
	uint count;
	bool suspended;
};

/**
 * gxp_telemetry_init() - Initialize wakelock support
 * @gxp: The GXP device to initialize wakelock support for
 *
 * Return:
 * * 0       - Success
 * * -ENOMEM - Insufficient memory is available to initialize support
 */
int gxp_wakelock_init(struct gxp_dev *gxp);

/**
 * gxp_wakelock_acquire() - Increment the GXP wakelock counter
 * @gxp: The GXP device to increment the wakelock counter for
 *
 * If the wakelock counter transitions from 0 to 1, this will result in BLK_AUR
 * being powered on.
 *
 * Return:
 * * 0       - Success
 * * -EAGAIN - The system is suspending and BLK_AUR cannot be powered on
 * * Other   - An attempt to power on BLK_AUR failed
 */
int gxp_wakelock_acquire(struct gxp_dev *gxp);

/**
 * gxp_wakelock_release() - Decrement the GXP wakelock counter
 * @gxp: The GXP device to decrement the wakelock counter for
 *
 * If the wakelock counter transitions from 1 to 0, this will result in BLK_AUR
 * being powered off. In the event BLK_AUR cannot be powered off, a message
 * will be logged, but the wakelock will still be released.
 */
void gxp_wakelock_release(struct gxp_dev *gxp);

/**
 * gxp_wakelock_suspend() - Check if the wakelock will allow a system suspend
 * @gxp: The GXP device to check the wakelock of
 *
 * Return:
 * * 0       - The wakelock has been suspended and is ready for system suspend
 * * -EAGAIN - The wakelock is held, and system suspend should be aborted
 */
int gxp_wakelock_suspend(struct gxp_dev *gxp);

/**
 * gxp_wakelock_resume() - Notify the wakelock that system suspend has exited
 * @gxp: The GXP device to notify the wakelock of
 *
 * Return:
 * * 0 - The wakelock is ready to be acquired again
 */
int gxp_wakelock_resume(struct gxp_dev *gxp);

#endif /* __GXP_WAKELOCK_H__ */
