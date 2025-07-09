/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright 2022 Google LLC. All Rights Reserved.
 *
 * aoc service to send cmds to aoc tbn server
 */

#ifndef _AOC_TBN_SERVICE_DEV_H
#define _AOC_TBN_SERVICE_DEV_H

ssize_t aoc_tbn_service_write(void *cmd, size_t size);
ssize_t aoc_tbn_service_read(void *cmd, size_t size);
bool aoc_tbn_service_ready(void);

#endif /* _AOC_TBN_SERVICE_DEV_H */
