// SPDX-License-Identifier: GPL-2.0-only
/* freeze.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/types.h>

/*
 * Freezing of user space tasks may be failed due to process being in
 * uninterruptible sleep state. Currently,the logging of unfrozen tasks
 * is disabled by default. Although it can be enabled for debugging via
 * an adb command, it's usually hard to reproduce the issue. This hook
 * is used to enable logging to record unfrozen tasks.
 */
void vh_try_to_freeze_todo_logging_pixel_mod(void *data, bool *logging_on)
{
	*logging_on = true;
}
