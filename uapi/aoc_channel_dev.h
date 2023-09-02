/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Channelized character IPC device interface for AoC services
 *
 * Copyright (c) 2021 Google LLC
 *
 */

#define AOCC_IOCTL_MAGIC 0xac

#define AOCC_IOCTL_ENABLE_SH_MEM_DOORBELL _IO(AOCC_IOCTL_MAGIC, 1)

