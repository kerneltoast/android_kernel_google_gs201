/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright 2020 Google LLC. All Rights Reserved.
 *
 * aoc service to send cmds to aoc
 */

#ifndef _AOC_UWB_SERVICE_DEV_H
#define _AOC_UWB_SERVICE_DEV_H

ssize_t aoc_uwb_service_send(void *cmd, size_t size);
bool aoc_uwb_service_ready(void);

#endif /* _AOC_UWB_SERVICE_DEV_H */
